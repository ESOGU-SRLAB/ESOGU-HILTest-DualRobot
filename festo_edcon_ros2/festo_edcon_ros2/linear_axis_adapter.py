#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from control_msgs.msg import JointTrajectoryControllerState
from controller_manager_msgs.srv import SwitchController
from std_srvs.srv import Trigger
from ur_dashboard_msgs.srv import Popup
import time
from collections import deque

# Festo EdCon kütüphanesi importları
from edcon.edrive.motion_handler import MotionHandler
from edcon.edrive.com_modbus import ComModbus

class LinearAxisControllerAdapter(Node):
    def __init__(self):
        super().__init__('linear_axis_controller_adapter')
        self.get_logger().info("Linear Axis Controller Adapter started.")

        # Parametreler
        self.declare_parameter('ip_address', '192.168.4.1')
        self.declare_parameter('joint_name', 'ur10e_base_to_robot_mount')
        self.declare_parameter('update_frequency_hz', 500.0)
        self.declare_parameter('command_send_frequency_hz', 100.0)
        self.declare_parameter('interpolation_alpha', 0.2)
        self.declare_parameter('min_movement_threshold', 0.001)
        self.declare_parameter('use_moveit_velocity', True)
        self.declare_parameter('max_velocity_m_per_s', 1.0)  # m/s
        self.declare_parameter('min_velocity_m_per_s', 0.05)
        
        # Güvenlik parametreleri
        self.declare_parameter('connection_timeout_sec', 0.2)  # 200ms - Hızlı tepki
        self.declare_parameter('enable_safety_stop', True)

        self.ip_address = self.get_parameter('ip_address').get_parameter_value().string_value
        self.joint_name = self.get_parameter('joint_name').get_parameter_value().string_value
        self.update_frequency_hz = self.get_parameter('update_frequency_hz').get_parameter_value().double_value
        self.command_send_frequency_hz = self.get_parameter('command_send_frequency_hz').get_parameter_value().double_value
        self.interpolation_alpha = self.get_parameter('interpolation_alpha').get_parameter_value().double_value
        self.min_movement_threshold = self.get_parameter('min_movement_threshold').get_parameter_value().double_value
        self.use_moveit_velocity = self.get_parameter('use_moveit_velocity').get_parameter_value().bool_value
        self.max_velocity_m_per_s = self.get_parameter('max_velocity_m_per_s').get_parameter_value().double_value
        self.min_velocity_m_per_s = self.get_parameter('min_velocity_m_per_s').get_parameter_value().double_value
        
        # Güvenlik parametrelerini al
        self.connection_timeout_sec = self.get_parameter('connection_timeout_sec').get_parameter_value().double_value
        self.enable_safety_stop = self.get_parameter('enable_safety_stop').get_parameter_value().bool_value

        self.current_position_meters = 0.0
        self.target_position_meters = 0.0
        self.interpolated_position_meters = 0.0
        self.command_velocity_m_per_s = 0.5

        self.command_buffer = deque(maxlen=5)
        self.velocity_buffer = deque(maxlen=5)
        self.last_command_time = time.time()
        
        # Güvenlik değişkenleri
        self.last_successful_communication = time.time()
        self.safety_stop_triggered = False

        self.com = None
        self.motion_handler = None
        self.position_scaling = 1.0
        self.velocity_scaling = 1.0
        self.connected_to_festo = False

        # Festo EdCon bağlantısı
        try:
            self.com = ComModbus(ip_address=self.ip_address, timeout_ms=1000)
            pnu_pos_scaling_exp = self.com.read_pnu(11724, 0)
            pnu_vel_scaling_exp = self.com.read_pnu(11725, 0)
            self.position_scaling = 10 ** (-pnu_pos_scaling_exp)
            self.velocity_scaling = 10 ** (-pnu_vel_scaling_exp)
            self.get_logger().info(f"Position scaling: {self.position_scaling}")
            self.get_logger().info(f"Velocity scaling: {self.velocity_scaling}")

            self.motion_handler = MotionHandler(self.com, config_mode="write")
            base_velocity_pnu = self.com.read_pnu(12345, 0)
            self.motion_handler.base_velocity = base_velocity_pnu

            self.motion_handler.acknowledge_faults()
            self.motion_handler.enable_powerstage()
            self.get_logger().info("Festo powerstage activated")

            initial_position_festo_units = self.motion_handler.current_position()
            self.current_position_meters = (initial_position_festo_units / self.position_scaling)
            self.target_position_meters = self.current_position_meters
            self.interpolated_position_meters = self.current_position_meters
            self.get_logger().info(f"Start position: {self.current_position_meters:.4f} m")
            self.connected_to_festo = True

        except Exception as e:
            self.get_logger().error(f"Festo connection error: {e}")
            self.connected_to_festo = False

        self.subscription = self.create_subscription(
            JointState,
            '/linear_axis/target_position_cmd',
            self.joint_state_callback,
            10
        )

        # State'den hız bilgisini al
        from control_msgs.msg import JointTrajectoryControllerState
        self.state_subscription = self.create_subscription(
            JointTrajectoryControllerState,
            '/scaled_joint_trajectory_controller/controller_state',
            self.controller_state_callback,
            10
        )

        self.joint_state_publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.status_timer = self.create_timer(1.0 / self.update_frequency_hz, self.publish_status)
        self.command_timer = self.create_timer(1.0 / self.command_send_frequency_hz, self.send_motion_command)
        
        # UR Controller Manager servisi (Controller'ları durdurma)
        self.stop_controller_client = self.create_client(
            SwitchController,
            '/controller_manager/switch_controller'
        )
        
        # UR Dashboard Protective Stop servisi (Opsiyonel - Fiziksel frenler)
        self.protective_stop_client = self.create_client(
            Trigger,
            '/dashboard_client/protective_stop'
        )
        
        # UR Dashboard Popup servisi (Pendant'ta uyarı gösterme)
        self.popup_client = self.create_client(
            Popup,
            '/dashboard_client/popup'
        )
        
        # Bağlantı izleme timer'ı - 100ms (Çok hızlı tepki için)
        self.connection_monitor_timer = self.create_timer(0.1, self.monitor_connection)

        self.get_logger().info(f'Adapter started:')
        self.get_logger().info(f'  - Command frequency: {self.command_send_frequency_hz} Hz')
        self.get_logger().info(f'  - Interpolation alpha: {self.interpolation_alpha}')
        self.get_logger().info(f'  - Use MoveIt velocity: {self.use_moveit_velocity}')

    def trajectory_callback(self, msg: JointTrajectory):
        """Process trajectory's from MoveIt."""
        try:
            if not msg.points:
                return
            
            # Son point'i al (hedef)
            last_point = msg.points[-1]
            
            # Joint index
            try:
                joint_index = msg.joint_names.index(self.joint_name)
            except ValueError:
                self.get_logger().warn(f'{self.joint_name} not found')
                return

            # Position (in meters)
            new_target = last_point.positions[joint_index]
            
            self.command_buffer.append(new_target)
            
            # Moving average
            if len(self.command_buffer) >= 3:
                self.target_position_meters = sum(self.command_buffer) / len(self.command_buffer)
            else:
                self.target_position_meters = new_target
            
            self.last_command_time = time.time()
            
        except Exception as e:
            self.get_logger().error(f"Trajectory callback error: {e}")

    def controller_state_callback(self, msg: JointTrajectoryControllerState):
        """Get desired velocity from controller state."""
        try:
            if not msg.desired.velocities:
                return
            
            try:
                joint_index = msg.joint_names.index(self.joint_name)
            except ValueError:
                return
            
            # İstenen hızı al (state'deki desired velocities)
            velocity_m_per_s = abs(msg.reference.velocities[joint_index])
            # Sınırla
            self.command_velocity_m_per_s = max(
                self.min_velocity_m_per_s,
                min(velocity_m_per_s, self.max_velocity_m_per_s)
            )

            self.command_velocity_m_per_s = self.command_velocity_m_per_s*10
            
            self.velocity_buffer.append(self.command_velocity_m_per_s)
            
            self.get_logger().debug(
                f'State: Linear velocity = {self.command_velocity_m_per_s:.3f} m/s'
            )
            
        except Exception as e:
            self.get_logger().error(f"State callback error: {e}")

    def joint_state_callback(self, msg: JointState):
        """Fallback: old JointState subscription."""
        if msg.name and msg.position:
            try:
                joint_index = msg.name.index(self.joint_name)
                new_target = msg.position[joint_index]
                
                self.command_buffer.append(new_target)
                
                if len(self.command_buffer) >= 3:
                    self.target_position_meters = sum(self.command_buffer) / len(self.command_buffer)
                else:
                    self.target_position_meters = new_target
                
                self.last_command_time = time.time()
                
            except ValueError:
                self.get_logger().warn(f'{self.joint_name} not found')

    def calculate_adaptive_velocity(self, position_error):
        """Fallback adaptive velocity."""
        if position_error > 0.1:
            return 0.1
        elif position_error > 0.05:
            return 0.1
        elif position_error > 0.02:
            return 0.1
        else:
            return 0.1

    def monitor_connection(self):
        """Lineer eksen bağlantısını sürekli izle"""
        if not self.enable_safety_stop:
            return
            
        current_time = time.time()
        time_since_last_comm = current_time - self.last_successful_communication
        
        # Bağlantı timeout kontrolü
        if time_since_last_comm > self.connection_timeout_sec and not self.safety_stop_triggered:
            self.get_logger().error(
                f"LINEAR AXIS CONNECTION LOST! "
                f"No communication for {time_since_last_comm:.2f}s. "
                f"STOPPING ROBOT FOR SAFETY!"
            )
            self.trigger_safety_stop()
        
        # Festo haberleşme testi
        if self.connected_to_festo:
            try:
                # Basit bir okuma testi
                _ = self.motion_handler.current_position()
                self.last_successful_communication = current_time
                
                # Eğer daha önce stop tetiklendiyse ve bağlantı geri geldiyse
                if self.safety_stop_triggered:
                    self.get_logger().info("Linear axis connection restored.")
                    self.safety_stop_triggered = False
                    
            except Exception as e:
                self.get_logger().warn(f"Connection test failed: {e}")
                self.connected_to_festo = False

    def trigger_safety_stop(self):
        """Güvenlik durdurmayı tetikle - PROTECTIVE STOP (External Control de durur)"""
        self.safety_stop_triggered = True
        
        self.get_logger().error("=" * 60)
        self.get_logger().error("🚨 SAFETY STOP - LINEAR AXIS CONNECTION LOST! 🚨")
        self.get_logger().error("=" * 60)
        
        # ÖNCELİK 1: PROTECTIVE STOP - Bu hem robotu durdurur hem de External Control'ü sonlandırır
        self.trigger_protective_stop()
        
        # ÖNCELİK 2: Controller'ı da durdur (Çifte güvenlik)
        self.stop_robot_controller()
    
    def show_pendant_popup(self):
        """UR Pendant ekranında uyarı mesajı göster"""
        try:
            if not self.popup_client.wait_for_service(timeout_sec=0.3):
                self.get_logger().warn("Popup service not available")
                return
            
            request = Popup.Request()
            request.message = (
                "⚠️ SAFETY STOP ACTIVATED ⚠️\n\n"
                "LINEAR AXIS CONNECTION LOST!\n\n"
                "Robot has been stopped for safety.\n"
                "Please check linear axis connection\n"
                "before resuming operation."
            )
            
            future = self.popup_client.call_async(request)
            future.add_done_callback(self.popup_callback)
            
            self.get_logger().error(">>> PENDANT POPUP DISPLAYED <<<")
            
        except Exception as e:
            self.get_logger().warn(f"Failed to show popup: {e}")
    
    def popup_callback(self, future):
        """Popup yanıtını işle"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().error("✓ Warning popup displayed on pendant!")
            else:
                self.get_logger().warn(f"Popup failed: {response.answer}")
        except Exception as e:
            self.get_logger().warn(f"Popup callback error: {e}")
    
    def stop_robot_controller(self):
        """UR Robot trajectory controller'ını HEMEN durdur"""
        try:
            if not self.stop_controller_client.wait_for_service(timeout_sec=0.1):
                self.get_logger().error("Controller manager service not available!")
                return
            
            request = SwitchController.Request()
            # Aktif controller'ı durdur
            request.stop_controllers = ['scaled_joint_trajectory_controller']
            request.start_controllers = []
            request.strictness = SwitchController.Request.BEST_EFFORT
            request.start_asap = False
            request.timeout = rclpy.duration.Duration(seconds=0.0).to_msg()
            
            future = self.stop_controller_client.call_async(request)
            
            self.get_logger().error("⏹️  >>> STOPPING CONTROLLER NOW <<<")
            
            # Callback ekle
            future.add_done_callback(self.stop_controller_callback)
            
        except Exception as e:
            self.get_logger().error(f"Failed to stop controller: {e}")
    
    def stop_controller_callback(self, future):
        """Controller durdurma sonucunu kontrol et"""
        try:
            response = future.result()
            if response.ok:
                self.get_logger().error("✓ Robot controller STOPPED successfully!")
            else:
                self.get_logger().error("✗ Failed to stop robot controller!")
        except Exception as e:
            self.get_logger().error(f"Stop controller callback error: {e}")
    
    def trigger_protective_stop(self):
        """UR Protective Stop - ACİL DURDURMA (External Control da sonlanır)"""
        try:
            if not self.protective_stop_client.wait_for_service(timeout_sec=0.1):
                self.get_logger().error("⚠️ Protective stop service NOT AVAILABLE!")
                return
            
            request = Trigger.Request()
            future = self.protective_stop_client.call_async(request)
            future.add_done_callback(self.protective_stop_callback)
            
            self.get_logger().error("🛑 >>> PROTECTIVE STOP TRIGGERED NOW! <<<")
            self.get_logger().error("    External Control program will STOP!")
            
        except Exception as e:
            self.get_logger().error(f"❌ Failed to trigger protective stop: {e}")
    
    def protective_stop_callback(self, future):
        """Protective stop yanıtını işle"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().error("✅ PROTECTIVE STOP EXECUTED!")
                self.get_logger().error("   → Robot STOPPED")
                self.get_logger().error("   → External Control TERMINATED")
                self.get_logger().error("   → Pendant will show protective stop screen")
            else:
                self.get_logger().error(f"❌ Protective stop failed: {response.answer}")
        except Exception as e:
            self.get_logger().error(f"❌ Protective stop callback error: {e}")

    def send_motion_command(self):
        # Güvenlik kontrolü
        if self.safety_stop_triggered:
            return
            
        if not self.connected_to_festo:
            return

        try:
            # Exponential smoothing
            self.interpolated_position_meters = (
                self.interpolation_alpha * self.target_position_meters +
                (1.0 - self.interpolation_alpha) * self.interpolated_position_meters
            )
            
            position_error = abs(self.interpolated_position_meters - self.current_position_meters)
            
            if position_error < self.min_movement_threshold:
                # print("Saglanamadi")
                return
            
            # Hız seçimi
            if self.use_moveit_velocity and self.velocity_buffer:
                velocity_m_per_s = sum(self.velocity_buffer) / len(self.velocity_buffer)
                # self.get_logger().info(f'Hız komutu Moveit2 tarafından gönderildi')
            else:
                velocity_m_per_s = self.calculate_adaptive_velocity(position_error)
                # self.get_logger().info(f'Hız komutu STATİK OLARAK gönderildi')
            
            # Festo units'e çevir (cm -> Festo units)
            target_position_cm = self.interpolated_position_meters
            target_position_cm = target_position_cm
            # self.get_logger().info(f"TARGET POSITION: {target_position_cm:.4f} m")
            target_position_festo_units = int(target_position_cm * self.position_scaling)
            # self.get_logger().info(f"TARGET POSITION FESTO UNITS: {target_position_festo_units:.4f} m")
            velocity_cm_per_s = velocity_m_per_s*1000
            velocity_festo_units_per_s = int(velocity_cm_per_s * self.velocity_scaling)

            # Komut gönder
            self.motion_handler.position_task(
                target_position_festo_units,
                velocity_festo_units_per_s,
                absolute=True,
                nonblocking=True
            )
            
            # Başarılı komut gönderimi
            self.last_successful_communication = time.time()
            
            if position_error > 0.005:
                self.get_logger().info(
                    f"Goal: {self.interpolated_position_meters:.3f}m, "
                    f"Current: {self.current_position_meters:.3f}m, "
                    f"Error: {position_error*1000:.1f}mm, "
                    f"Velocity: {velocity_m_per_s:.3f}m/s"
                )

        except Exception as e:
            self.get_logger().error(f"Command error: {e}")
            self.connected_to_festo = False

    def publish_status(self):
        if not self.connected_to_festo:
            return
        try:
            current_position_festo_units = self.motion_handler.current_position()
            current_velocity_festo_units_per_s = self.motion_handler.current_velocity()

            self.current_position_meters = (current_position_festo_units / self.position_scaling)
            current_velocity_m_per_s = (current_velocity_festo_units_per_s / self.velocity_scaling) / 100

            joint_state_msg = JointState()
            joint_state_msg.header.stamp = self.get_clock().now().to_msg()
            joint_state_msg.name = [self.joint_name]
            joint_state_msg.position = [self.current_position_meters]
            joint_state_msg.velocity = [current_velocity_m_per_s]
            self.joint_state_publisher.publish(joint_state_msg)
            
            # Başarılı durum okuma
            self.last_successful_communication = time.time()

        except Exception as e:
            self.get_logger().error(f"Status error: {e}")
            self.connected_to_festo = False

    def cleanup(self):
        if self.motion_handler:
            self.get_logger().info("Powerstage shutting down...")
            self.motion_handler.disable_powerstage()

def main(args=None):
    rclpy.init(args=args)
    node = LinearAxisControllerAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()