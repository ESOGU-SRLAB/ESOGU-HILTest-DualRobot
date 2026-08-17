#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS2 Node — Sends a JointTrajectory message to /kawasaki_controller/joint_trajectory
This replaces low-level torque control and makes the robot move in Gazebo.
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np

class TrajectorySender(Node):
    def __init__(self):
        super().__init__('trajectory_publisher')
        self.pub = self.create_publisher(JointTrajectory, '/kawasaki_controller/joint_trajectory', 10)

        self.declare_parameter('traj_type', 'sin')
        self.declare_parameter('amp_deg', 20.0)
        self.declare_parameter('freq_hz', 0.2)
        self.declare_parameter('t_total', 10.0)
        self.declare_parameter('dt', 0.01)

        self.traj_type = self.get_parameter('traj_type').value
        self.amp = np.deg2rad(self.get_parameter('amp_deg').value)
        self.freq = self.get_parameter('freq_hz').value
        self.t_total = self.get_parameter('t_total').value
        self.dt = self.get_parameter('dt').value

        self.joint_names = [f"joint{i+1}" for i in range(6)]

        self.timer = self.create_timer(2.0, self.publish_trajectory_once)
        self.get_logger().info("TrajectorySender initialized. Will publish once after 2s...")

    def publish_trajectory_once(self):
        self.get_logger().info("Publishing trajectory to /kawasaki_controller/joint_trajectory")

        msg = JointTrajectory()
        msg.joint_names = self.joint_names

        t_points = np.arange(0, self.t_total, self.dt)
        for t in t_points:
            point = JointTrajectoryPoint()
            if self.traj_type == 'sin':
                q = self.amp * np.sin(2*np.pi*self.freq*t) * np.ones(6)
                dq = self.amp * 2*np.pi*self.freq * np.cos(2*np.pi*self.freq*t) * np.ones(6)
            else:
                q = np.zeros(6); dq = np.zeros(6)

            point.positions = q.tolist()
            point.velocities = dq.tolist()
            point.time_from_start.sec = int(t)
            point.time_from_start.nanosec = int((t - int(t)) * 1e9)
            msg.points.append(point)

        self.pub.publish(msg)
        self.get_logger().info(f"Trajectory with {len(msg.points)} points sent.")
        self.destroy_timer(self.timer)

def main(args=None):
    rclpy.init(args=args)
    node = TrajectorySender()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
