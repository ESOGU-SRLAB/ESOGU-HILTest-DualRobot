#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose

class TcpTransformNode(Node):
    def __init__(self):
        super().__init__('tcp_transform_node')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # TCP pozisyonunuzu burada tanımlayın
        self.tcp_pose = PoseStamped()
        self.tcp_pose.header.frame_id = 'sim_ur10e_base'
        # self.tcp_pose.pose.position.x = ...
        # self.tcp_pose.pose.orientation.w = ...
        self.timer = self.create_timer(1.0, self.transform_tcp)

    def transform_tcp(self):
        try:
            trans = self.tf_buffer.lookup_transform('world', 'sim_ur10e_base', rclpy.time.Time())
            tcp_in_world = do_transform_pose(self.tcp_pose, trans)
            self.get_logger().info(f"TCP in world: {tcp_in_world}")
        except Exception as e:
            self.get_logger().warn(f"Transform failed: {e}")

def main():
    rclpy.init()
    node = TcpTransformNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()