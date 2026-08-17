#!/usr/bin/env python3
"""
AGV Controller Node
ROS2 topic üzerinden Float64 pozisyon komutu alıp, bunu ROS1 action server'ın
GOAL moduna TEK BİR KERE gönderir. Böylece action server kendi içindeki 25 Hz
döngüsüyle yumuşak bir şekilde hedefe gidip durur (takılma olmaz).

Kullanım:
  ros2 run agv_nodes agv_controller_node
  ros2 topic pub /agv/goal_position std_msgs/msg/Float64 "{data: 1.0}" --once
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import roslibpy
import threading

class AGVControllerNode(Node):
    def __init__(self):
        super().__init__('agv_controller_node')

        # Parametreler
        self.declare_parameter('rosbridge_host', '192.168.3.6')
        self.declare_parameter('rosbridge_port', 9090)
        self.declare_parameter('velocity', 0.05)  # Action server için hedef hız

        self.host = self.get_parameter('rosbridge_host').get_parameter_value().string_value
        self.port = self.get_parameter('rosbridge_port').get_parameter_value().integer_value
        self.velocity = self.get_parameter('velocity').get_parameter_value().double_value

        self.goal_seq = 0
        self.ros1_client = roslibpy.Ros(host=self.host, port=self.port)
        self.goal_topic = None

        # Hedef pozisyon sub
        self.goal_sub = self.create_subscription(
            Float64, '/agv/goal_position', self._goal_callback, 10
        )

        # Bağlantıyı başlat
        self.connect_thread = threading.Thread(target=self._connect, daemon=True)
        self.connect_thread.start()

        self.get_logger().info('AGV Controller Node başlatıldı')
        self.get_logger().info(f'Rosbridge: ws://{self.host}:{self.port} | Hız: {self.velocity} m/s')

    def _connect(self):
        try:
            self.ros1_client.run()
            self.goal_topic = roslibpy.Topic(
                self.ros1_client,
                '/mobile_platform/goal',
                'mobile_manipulator_action/MobilePlatformActionGoal'
            )
            self.get_logger().info('Rosbridge bağlantısı DÜZGÜN ŞEKİLDE sağlandı!')
        except Exception as e:
            self.get_logger().error(f'Bağlantı hatası: {e}')

    def _goal_callback(self, msg: Float64):
        """Yeni bir hedef geldiğinde Action Server'ın GOAL moduna TEK SEFER gönderilir."""
        if not self.goal_topic or not self.ros1_client.is_connected:
            self.get_logger().warn('Rosbridge henüz bağlı değil, komut atlandı.')
            return

        target_x = float(msg.data)
        self.get_logger().info(f'--- YENİ HEDEF --- Hedef: {target_x:.4f} m | Hız: {self.velocity} m/s')

        goal_msg = {
            'header': {
                'seq': self.goal_seq,
                'stamp': {'secs': 0, 'nsecs': 0},
                'frame_id': ''
            },
            'goal_id': {
                'stamp': {'secs': 0, 'nsecs': 0},
                'id': f'goal_{self.goal_seq}'
            },
            'goal': {
                'goal': [self.velocity, target_x],  # [vel, pose] C++ action server bekliyor
                'manuel': [],
                'cancel': 0
            }
        }

        try:
            self.goal_topic.publish(roslibpy.Message(goal_msg))
            self.goal_seq += 1
            self.get_logger().info('Komut başarıyla gönderildi, agv hareket ediyor.')
        except Exception as e:
            self.get_logger().error(f'Komut gönderilirken hata oluştu: {e}')

    def destroy_node(self):
        self.get_logger().info('Node kapatılıyor...')
        if self.ros1_client.is_connected:
            self.ros1_client.terminate()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AGVControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except:
            pass

if __name__ == '__main__':
    main()
