#!/usr/bin/env python3
# coding=utf-8

import rclpy
from rclpy.node import Node
from sir_robot_ros_interface.srv import ManipulatorPoseIno2


def call_service(node, srv_name, srv_type, srv_request=None):
    """Call input service function"""
    client = node.create_client(srv_type, srv_name)
    
    if not client.wait_for_service(timeout_sec=5.0):
        node.get_logger().error(f'Service {srv_name} not available')
        return None
    
    future = client.call_async(srv_request)
    rclpy.spin_until_future_complete(node, future)
    
    if future.result() is not None:
        return future.result()
    else:
        node.get_logger().error('Service call failed')
        return None


if __name__ == '__main__':
    rclpy.init()
    
    node = Node('service_client')
    
    req = ManipulatorPoseIno2.Request()
    
    req.path = "/home/ifarlab/colcon_ws/src/sir_robot_ros_interface/sir_robot_ros_interface/csv/35269.csv"
    print(req)
    
    resp = call_service(node, srv_name="/manipulator_service_test", srv_type=ManipulatorPoseIno2, srv_request=req)
    
    if resp is not None:
        print(resp.status)
    
    node.destroy_node()
    rclpy.shutdown()