#!/usr/bin/env python3
"""
AGV Bridge Node
ROS1 topic'lerini roslibpy üzerinden dinleyip ROS2'ye publish eder.
Noetic PC'deki rosbridge_websocket'e bağlanır.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int8, Bool, Float64
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
import roslibpy
import threading


class AGVBridgeNode(Node):
    def __init__(self):
        super().__init__('agv_bridge_node')

        # Parametreler
        self.declare_parameter('rosbridge_host', '192.168.3.6')
        self.declare_parameter('rosbridge_port', 9090)

        host = self.get_parameter('rosbridge_host').get_parameter_value().string_value
        port = self.get_parameter('rosbridge_port').get_parameter_value().integer_value

        self.get_logger().info(f'Rosbridge bağlantısı: ws://{host}:{port}')

        # roslibpy client
        self.ros1_client = roslibpy.Ros(host=host, port=port)

        # ROS2 Publishers
        self.odom_pub = self.create_publisher(Odometry, '/agv/odom', 10)
        # world_to_agv prismatik joint için — robot_state_publisher bu olmadan
        # ota_base_link ve altındaki tüm TF'leri (AGV + Kawasaki) yayımlamaz
        self.agv_joint_pub = self.create_publisher(JointState, '/joint_states', 10)

        # En son bilinen AGV konumu (odom callback'ten thread-safe güncellenir)
        self._agv_position = 0.0
        self._agv_velocity = 0.0
        self._agv_lock = threading.Lock()

        # AGV goal rate-limiter: topic_based_ros2_control 100Hz komut yazar;
        # her komut ROS1 action server'a yeni goal gönderir ve öncekini iptal eder.
        # Bunu önlemek için en fazla 2Hz (500ms) ve min 2cm değişimde goal gönder.
        self._last_sent_agv_goal = None
        self._last_goal_send_time = 0.0
        self._goal_update_interval = 0.5   # saniye (2 Hz)
        self._goal_min_change = 0.05       # metre (5 cm) — acil hedef değişimi

        # ros2_control → AGV komut köprüsü:
        # topic_based_ros2_control/TopicBasedSystem, JointState olarak /agv/joint_cmd'e yazar;
        # buradan world_to_agv pozisyonu alınıp /agv/goal_position (Float64)'e çevrilir;
        # agv_controller_node oradan rosbridge üzerinden ROS1 action server'a iletir.
        self.goal_pub = self.create_publisher(Float64, '/agv/goal_position', 10)
        self.joint_cmd_sub = self.create_subscription(
            JointState, '/agv/joint_cmd', self._joint_cmd_callback, 10)

        # 50 Hz sabit timer — TF_OLD_DATA'yı önlemek için JointState burada
        # publish edilir; böylece robot_state_publisher her zaman monoton artan
        # zaman damgası alır (odom callback'indeki roslibpy thread'inden değil)
        self.create_timer(0.02, self._publish_agv_joint_state)
        self.mobile_status_pub = self.create_publisher(String, '/agv/mobile_status', 10)
        self.emg_pub = self.create_publisher(Int8, '/agv/emg', 10)
        self.ui_emg_pub = self.create_publisher(Int8, '/agv/ui_emg', 10)
        self.light_curtain_pub = self.create_publisher(Bool, '/agv/light_curtain', 10)
        self.low_laser_pub = self.create_publisher(String, '/agv/low_laser', 10)
        self.cmd_vel_echo_pub = self.create_publisher(Twist, '/agv/cmd_vel_echo', 10)

        # ROS1 Subscribers (roslibpy)
        self.ros1_subs = []

        # Bağlantıyı ayrı thread'de başlat
        self.bridge_thread = threading.Thread(target=self._connect_and_subscribe, daemon=True)
        self.bridge_thread.start()

        # Durum kontrolü timer
        self.create_timer(5.0, self._check_connection)

        self.get_logger().info('AGV Bridge Node başlatıldı')

    def _connect_and_subscribe(self):
        """Rosbridge'e bağlan ve ROS1 topic'lerine abone ol."""
        try:
            self.ros1_client.run()
            self.get_logger().info('Rosbridge bağlantısı başarılı!')

            # /odom -> /agv/odom
            odom_sub = roslibpy.Topic(self.ros1_client, '/odom', 'nav_msgs/Odometry')
            odom_sub.subscribe(self._odom_callback)
            self.ros1_subs.append(odom_sub)

            # /mobile_status -> /agv/mobile_status
            status_sub = roslibpy.Topic(self.ros1_client, '/mobile_status', 'std_msgs/String')
            status_sub.subscribe(self._mobile_status_callback)
            self.ros1_subs.append(status_sub)

            # /emg -> /agv/emg
            emg_sub = roslibpy.Topic(self.ros1_client, '/emg', 'std_msgs/Int8')
            emg_sub.subscribe(self._emg_callback)
            self.ros1_subs.append(emg_sub)

            # /ui_emg -> /agv/ui_emg
            ui_emg_sub = roslibpy.Topic(self.ros1_client, '/ui_emg', 'std_msgs/Int8')
            ui_emg_sub.subscribe(self._ui_emg_callback)
            self.ros1_subs.append(ui_emg_sub)

            # /light_curtain -> /agv/light_curtain
            lc_sub = roslibpy.Topic(self.ros1_client, '/light_curtain', 'std_msgs/Bool')
            lc_sub.subscribe(self._light_curtain_callback)
            self.ros1_subs.append(lc_sub)

            # /low_laser -> /agv/low_laser
            laser_sub = roslibpy.Topic(self.ros1_client, '/low_laser', 'std_msgs/String')
            laser_sub.subscribe(self._low_laser_callback)
            self.ros1_subs.append(laser_sub)

            # /cmd_vel -> /agv/cmd_vel_echo (sadece okuma amaçlı)
            cmd_sub = roslibpy.Topic(self.ros1_client, '/cmd_vel', 'geometry_msgs/Twist')
            cmd_sub.subscribe(self._cmd_vel_callback)
            self.ros1_subs.append(cmd_sub)

            self.get_logger().info(f'{len(self.ros1_subs)} ROS1 topic aboneliği oluşturuldu')

        except Exception as e:
            self.get_logger().error(f'Rosbridge bağlantı hatası: {e}')

    def _odom_callback(self, msg):
        agv_x = msg['pose']['pose']['position']['x']
        agv_vel = msg['twist']['twist']['linear']['x']

        # AGV konumunu thread-safe güncelle (timer ayrı thread'de okur)
        with self._agv_lock:
            self._agv_position = agv_x
            self._agv_velocity = agv_vel

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = agv_x
        odom.pose.pose.position.y = msg['pose']['pose']['position']['y']
        odom.pose.pose.position.z = msg['pose']['pose']['position']['z']
        odom.pose.pose.orientation.x = msg['pose']['pose']['orientation']['x']
        odom.pose.pose.orientation.y = msg['pose']['pose']['orientation']['y']
        odom.pose.pose.orientation.z = msg['pose']['pose']['orientation']['z']
        odom.pose.pose.orientation.w = msg['pose']['pose']['orientation']['w']
        odom.twist.twist.linear.x = agv_vel
        odom.twist.twist.linear.y = msg['twist']['twist']['linear']['y']
        odom.twist.twist.linear.z = msg['twist']['twist']['linear']['z']
        odom.twist.twist.angular.x = msg['twist']['twist']['angular']['x']
        odom.twist.twist.angular.y = msg['twist']['twist']['angular']['y']
        odom.twist.twist.angular.z = msg['twist']['twist']['angular']['z']
        self.odom_pub.publish(odom)

    def _joint_cmd_callback(self, msg: JointState):
        """ros2_control'dan gelen JointState komutunu /agv/goal_position (Float64)'e çevirir.
        topic_based_ros2_control/TopicBasedSystem, komutları JointState olarak /agv/joint_cmd'e yazar;
        bu callback world_to_agv pozisyonunu alıp agv_controller_node'un beklediği
        Float64 formatına dönüştürür.

        Rate-limit: 100Hz komut akışını 2Hz'e düşürür. Her güncellemede ROS1
        action server'a yeni goal gönderilmesi AGV'nin sürekli iptal/yeniden
        başlatma döngüsüne girmesini engeller."""
        if 'world_to_agv' not in msg.name:
            return  # bu mesajda world_to_agv yok, atla

        if len(msg.position) == 0:
            return

        # topic_based_ros2_control yalnızca position command interface'i olan
        # joint'leri position[] array'ine yazar. world_to_agv bu sistemdeki tek
        # position-commanded joint olduğundan position[0] her zaman world_to_agv'dir.
        new_goal = msg.position[0]
        now = self.get_clock().now().nanoseconds * 1e-9

        # İlk goal, anlamlı değişim varsa VEYA büyük ani değişimde gönder
        # NOT: Eski logic "elapsed >= 0.5 OR change >= 0.05" idi → değer
        # değişmese bile her 0.5s'de goal gönderiyordu → ROS1 action server
        # sürekli preempt olup AGV hiç hareket edemiyordu.
        # Düzeltme: zaman geçse bile minimum 2mm değişim şartı eklendi.
        if self._last_sent_agv_goal is None:
            send = True
        else:
            change = abs(new_goal - self._last_sent_agv_goal)
            elapsed = now - self._last_goal_send_time
            min_meaningful_change = 0.002  # 2mm — gürültüyü filtrele
            send = (change >= self._goal_min_change) or \
                   (elapsed >= self._goal_update_interval and change >= min_meaningful_change)

        if not send:
            return

        self._last_sent_agv_goal = new_goal
        self._last_goal_send_time = now

        goal = Float64()
        goal.data = new_goal
        self.goal_pub.publish(goal)

    def _publish_agv_joint_state(self):
        """50 Hz timer callback — world_to_agv joint'ini her zaman ROS2
        node'unun kendi saatiyle yayımlar; roslibpy arka plan thread'inden
        bağımsız olduğu için TF_OLD_DATA uyarısı oluşmaz."""
        with self._agv_lock:
            pos = self._agv_position
            vel = self._agv_velocity

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = ['world_to_agv']
        js.position = [pos]
        js.velocity = [vel]
        js.effort = [0.0]
        self.agv_joint_pub.publish(js)

    def _mobile_status_callback(self, msg):
        ros2_msg = String()
        ros2_msg.data = msg['data']
        self.mobile_status_pub.publish(ros2_msg)

    def _emg_callback(self, msg):
        ros2_msg = Int8()
        ros2_msg.data = msg['data']
        self.emg_pub.publish(ros2_msg)

    def _ui_emg_callback(self, msg):
        ros2_msg = Int8()
        ros2_msg.data = msg['data']
        self.ui_emg_pub.publish(ros2_msg)

    def _light_curtain_callback(self, msg):
        ros2_msg = Bool()
        ros2_msg.data = msg['data']
        self.light_curtain_pub.publish(ros2_msg)

    def _low_laser_callback(self, msg):
        ros2_msg = String()
        ros2_msg.data = msg['data']
        self.low_laser_pub.publish(ros2_msg)

    def _cmd_vel_callback(self, msg):
        twist = Twist()
        twist.linear.x = msg['linear']['x']
        twist.linear.y = msg['linear']['y']
        twist.linear.z = msg['linear']['z']
        twist.angular.x = msg['angular']['x']
        twist.angular.y = msg['angular']['y']
        twist.angular.z = msg['angular']['z']
        self.cmd_vel_echo_pub.publish(twist)

    def _check_connection(self):
        if self.ros1_client.is_connected:
            self.get_logger().debug('Rosbridge bağlantısı aktif')
        else:
            self.get_logger().warn('Rosbridge bağlantısı koptu! Yeniden bağlanılıyor...')
            self.bridge_thread = threading.Thread(target=self._connect_and_subscribe, daemon=True)
            self.bridge_thread.start()

    def destroy_node(self):
        self.get_logger().info('Bridge kapatılıyor...')
        for sub in self.ros1_subs:
            sub.unsubscribe()
        if self.ros1_client.is_connected:
            self.ros1_client.terminate()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AGVBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
