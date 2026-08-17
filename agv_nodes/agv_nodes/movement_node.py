#!/usr/bin/env python3
"""
AGV Movement Node
AGV'yi sürekli ileri-geri hareket ettirir.
Varsayılan: 3 sn ileri (0.03 m/s), 3 sn geri (-0.03 m/s), tekrar...

Kullanım:
  ros2 run agv_nodes movement_node
  ros2 run agv_nodes movement_node --ros-args -p velocity:=0.05 -p duration:=2.0
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import roslibpy
import threading
import time


class AGVMovementNode(Node):
    def __init__(self):
        super().__init__('agv_movement_node')

        # Parametreler
        self.declare_parameter('rosbridge_host', '192.168.3.6')
        self.declare_parameter('rosbridge_port', 9090)
        self.declare_parameter('velocity', -0.1)
        self.declare_parameter('duration', 10.0)

        self.host = self.get_parameter('rosbridge_host').get_parameter_value().string_value
        self.port = self.get_parameter('rosbridge_port').get_parameter_value().integer_value
        self.velocity = self.get_parameter('velocity').get_parameter_value().double_value
        self.duration = self.get_parameter('duration').get_parameter_value().double_value

        self.get_logger().info(f'Rosbridge: ws://{self.host}:{self.port}')
        self.get_logger().info(f'Hız: {self.velocity} m/s, Süre: {self.duration} s')

        # roslibpy client
        self.ros1_client = roslibpy.Ros(host=self.host, port=self.port)
        self.goal_topic = None
        self.connected = False
        self.goal_seq = 0
        self.running = True

        # Mevcut hız (action server'a gönderilen)
        self.current_velocity = 0.0

        # ROS2 Publisher: durum bilgisi
        self.status_pub = self.create_publisher(String, '/agv/movement_status', 10)

        # Periyodik goal gönderimi (10 Hz)
        self.send_timer = self.create_timer(0.1, self._send_goal_periodic)

        # Bağlan ve osilatif hareketi başlat
        self.connect_thread = threading.Thread(target=self._connect_and_start, daemon=True)
        self.connect_thread.start()

        self.get_logger().info('AGV Movement Node başlatıldı')

    def _connect_and_start(self):
        """Rosbridge'e bağlan ve osilatif hareketi başlat."""
        try:
            self.ros1_client.run()
            self.goal_topic = roslibpy.Topic(
                self.ros1_client,
                '/mobile_platform/goal',
                'mobile_manipulator_action/MobilePlatformActionGoal'
            )
            self.connected = True
            self.get_logger().info('Rosbridge bağlantısı başarılı!')

            # Osilatif hareket döngüsü
            self._oscillate()

        except Exception as e:
            self.get_logger().error(f'Bağlantı hatası: {e}')

    def _oscillate(self):
        """Sürekli ileri-geri hareket döngüsü."""
        cycle = 0
        while self.running:
            cycle += 1

            # İLERİ
            self.current_velocity = self.velocity
            self.get_logger().info(
                f'[Döngü {cycle}] İLERİ: {self.velocity} m/s, {self.duration} sn')
            self._publish_status(f'İleri: {self.velocity} m/s')
            time.sleep(self.duration)

            if not self.running:
                break

            # DUR (yön değişimi öncesi)
            self.current_velocity = 0.0
            self._publish_status('Yön değiştiriliyor...')
            time.sleep(0.5)

            if not self.running:
                break

            # GERİ
            self.current_velocity = -self.velocity
            self.get_logger().info(
                f'[Döngü {cycle}] GERİ: {-self.velocity} m/s, {self.duration} sn')
            self._publish_status(f'Geri: {-self.velocity} m/s')
            time.sleep(self.duration)

            if not self.running:
                break

            # DUR (yön değişimi öncesi)
            self.current_velocity = 0.0
            self._publish_status('Yön değiştiriliyor...')
            time.sleep(0.5)

    def _send_goal_periodic(self):
        """10 Hz ile action server'a goal gönder."""
        if not self.connected or self.goal_topic is None:
            return

        if abs(self.current_velocity) < 0.001:
            direction = 0.0
            speed = 0.0
        else:
            direction = 1.0 if self.current_velocity > 0 else -1.0
            speed = abs(self.current_velocity)

        goal_msg = {
            'header': {
                'seq': self.goal_seq,
                'stamp': {'secs': 0, 'nsecs': 0},
                'frame_id': ''
            },
            'goal_id': {
                'stamp': {'secs': 0, 'nsecs': 0},
                'id': f'ros2_move_{self.goal_seq}'
            },
            'goal': {
                'goal': [],
                'manuel': [direction, speed],
                'cancel': 0
            }
        }

        try:
            self.goal_topic.publish(roslibpy.Message(goal_msg))
            self.goal_seq += 1
        except Exception as e:
            self.get_logger().error(f'Goal gönderme hatası: {e}')

    def _publish_status(self, status):
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

    def destroy_node(self):
        self.get_logger().info('Movement node kapatılıyor, robot durduruluyor...')
        self.running = False
        self.current_velocity = 0.0
        time.sleep(0.3)  # Son STOP goal'ü gönderilsin
        if self.goal_topic:
            self.goal_topic.unadvertise()
        if self.ros1_client.is_connected:
            self.ros1_client.terminate()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AGVMovementNode()
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
