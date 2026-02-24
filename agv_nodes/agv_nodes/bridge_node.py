#!/usr/bin/env python3
"""
AGV Bridge Node
ROS1 topic'lerini roslibpy üzerinden dinleyip ROS2'ye publish eder.
Noetic PC'deki rosbridge_websocket'e bağlanır.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int8, Bool
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
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
        odom = Odometry()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = msg['pose']['pose']['position']['x']
        odom.pose.pose.position.y = msg['pose']['pose']['position']['y']
        odom.pose.pose.position.z = msg['pose']['pose']['position']['z']
        odom.pose.pose.orientation.x = msg['pose']['pose']['orientation']['x']
        odom.pose.pose.orientation.y = msg['pose']['pose']['orientation']['y']
        odom.pose.pose.orientation.z = msg['pose']['pose']['orientation']['z']
        odom.pose.pose.orientation.w = msg['pose']['pose']['orientation']['w']
        odom.twist.twist.linear.x = msg['twist']['twist']['linear']['x']
        odom.twist.twist.linear.y = msg['twist']['twist']['linear']['y']
        odom.twist.twist.linear.z = msg['twist']['twist']['linear']['z']
        odom.twist.twist.angular.x = msg['twist']['twist']['angular']['x']
        odom.twist.twist.angular.y = msg['twist']['twist']['angular']['y']
        odom.twist.twist.angular.z = msg['twist']['twist']['angular']['z']
        self.odom_pub.publish(odom)

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
