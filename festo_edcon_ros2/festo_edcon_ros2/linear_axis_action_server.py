#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from festo_edcon_ros2.action import MoveAxis

class LinearAxisActionServer(Node):
    def __init__(self):
        super().__init__('linear_axis_action_server')
        self._action_server = ActionServer(
            self,
            MoveAxis,
            'move_linear_axis',
            self.execute_callback)
        
        self.target_position_publisher = self.create_publisher(
            Float64, '/linear_axis/target_position', 10)
        
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)
            
        self.current_position = 0.0

    def joint_state_callback(self, msg):
        # Assuming the adapter only publishes one joint
        if msg.position:
            self.current_position = msg.position[0]

    def execute_callback(self, goal_handle):
        self.get_logger().info(f'Executing goal: Gidilecek Pozisyon {goal_handle.request.target_position:.2f}m')

        # Hedef pozisyonu adaptör düğümüne yayınla
        target_position_msg = Float64()
        target_position_msg.data = goal_handle.request.target_position
        self.target_position_publisher.publish(target_position_msg)

        feedback_msg = MoveAxis.Feedback()
        
        # Daha sağlam bir bekleme döngüsü için rclpy.spin_once yerine time.sleep kullan
        import time 
        
        # 1mm tolerans ile hedefe varılıp varılmadığını kontrol et
        while abs(self.current_position - goal_handle.request.target_position) > 0.001:
            # Görev iptal edildiyse döngüden çık
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal Canceled')
                return MoveAxis.Result()

            # Geri bildirim (feedback) mesajı yayınla
            feedback_msg.current_position = self.current_position
            goal_handle.publish_feedback(feedback_msg)
            
            # Kısa bir süre bekle, bu ROS2'nin diğer işlerini yapmasına olanak tanır
            time.sleep(0.1) 

        goal_handle.succeed()

        result = MoveAxis.Result()
        result.success = True
        result.final_position = self.current_position
        self.get_logger().info(f'Hedef tamamlandı. Son Pozisyon: {result.final_position:.4f}m')
        return result

def main(args=None):
    rclpy.init(args=args)
    node = LinearAxisActionServer()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
