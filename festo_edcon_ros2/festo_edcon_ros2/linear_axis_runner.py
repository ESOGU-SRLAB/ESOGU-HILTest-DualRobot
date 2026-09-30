#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from festo_edcon_ros2.action import MoveAxis

class MoveAxisClient(Node):
    def __init__(self, waypoints):
        super().__init__('move_axis_client')
        self._client = ActionClient(self, MoveAxis, 'move_linear_axis')
        self.waypoints = waypoints
        self.index = 0

    def send_next_goal(self):
        if self.index >= len(self.waypoints):
            self.get_logger().info('Tüm hedefler tamamlandı.')
            rclpy.shutdown()
            return

        target_position = self.waypoints[self.index]
        self.get_logger().info(f'Goal gönderiliyor: {target_position}')
        goal_msg = MoveAxis.Goal()
        goal_msg.target_position = float(target_position)

        self._client.wait_for_server()
        self._send_goal_future = self._client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal reddedildi.')
            rclpy.shutdown()
            return

        self.get_logger().info('Goal kabul edildi.')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info('Hedef başarıyla tamamlandı.')
        else:
            self.get_logger().info('Hedef başarısız.')
        self.index += 1
        self.send_next_goal()

def main():
    rclpy.init()
    waypoints = [0.3, 0.5, 1.0]  # Dilediğin hedefleri yaz
    node = MoveAxisClient(waypoints)
    node.send_next_goal()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
