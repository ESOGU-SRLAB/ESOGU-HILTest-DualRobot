#!/usr/bin/env python3
"""
AGV Position Node
/agv/odom topic'ini dinleyip AGV'nin pozisyon ve hız bilgisini gösterir.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String


class AGVPositionNode(Node):
    def __init__(self):
        super().__init__('agv_position_node')

        # Parametre: yazdırma sıklığı (Hz)
        self.declare_parameter('print_rate', 2.0)
        print_rate = self.get_parameter('print_rate').get_parameter_value().double_value

        # Subscriber
        self.odom_sub = self.create_subscription(
            Odometry, '/agv/odom', self._odom_callback, 10)

        self.status_sub = self.create_subscription(
            String, '/agv/mobile_status', self._status_callback, 10)

        # Publisher: basitleştirilmiş pozisyon verisi
        self.position_pub = self.create_publisher(String, '/agv/position', 10)

        # Durum değişkenleri
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_status = 'Bilinmiyor'

        # Periyodik yazdırma
        self.print_timer = self.create_timer(1.0 / print_rate, self._print_status)

        self.get_logger().info('AGV Position Node başlatıldı')

    def _odom_callback(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_vx = msg.twist.twist.linear.x
        self.current_vy = msg.twist.twist.linear.y

    def _status_callback(self, msg: String):
        self.current_status = msg.data

    def _print_status(self):
        pos_str = (
            f'x={self.current_x:.4f} m | '
            f'y={self.current_y:.4f} m | '
            f'vx={self.current_vx:.4f} m/s | '
            f'status={self.current_status}'
        )
        self.get_logger().info(pos_str)

        # Basitleştirilmiş pozisyon publish
        pos_msg = String()
        pos_msg.data = (
            f'{{"x": {self.current_x:.4f}, '
            f'"y": {self.current_y:.4f}, '
            f'"vx": {self.current_vx:.4f}, '
            f'"status": "{self.current_status}"}}'
        )
        self.position_pub.publish(pos_msg)


def main(args=None):
    rclpy.init(args=args)
    node = AGVPositionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
