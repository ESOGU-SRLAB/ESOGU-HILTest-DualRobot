#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS2 Node – RS005L inverse dynamics + PD controller
Publishes joint torques to /rs_controller/torque_cmd
and subscribes to /joint_states and /rs_controller/joint_ref

Author: Cem Süha Yılmaz (2025)
"""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from rs005l_controller.model import (
    mass_matrix, gravity_vector, coriolis_matrix, friction_torque
)

KP = np.array([8.0, 8.0, 6.0, 4.0, 3.0, 3.0])
KD = np.array([1.8, 1.8, 1.4, 1.0, 0.8, 0.8])
TAU_MAX = np.array([180, 180, 120, 80, 60, 60], dtype=float)

class RSControllerNode(Node):
    def __init__(self):
        super().__init__('rs_controller_node')

        # Parameters
        self.declare_parameter('scenario', 'S3')
        self.scenario = self.get_parameter('scenario').get_parameter_value().string_value

        # Subscribers
        self.create_subscription(JointState, '/joint_states', self.joint_state_cb, 10)
        self.create_subscription(JointState, '/rs_controller/joint_ref', self.joint_ref_cb, 10)

        # Publisher
        self.torque_pub = self.create_publisher(Float64MultiArray,
                                                '/rs_controller/torque_cmd', 10)

        # Internal states
        self.q = np.zeros(6)
        self.dq = np.zeros(6)
        self.q_ref = np.zeros(6)
        self.dq_ref = np.zeros(6)
        self.last_time = self.get_clock().now()

        # Control loop timer
        self.timer = self.create_timer(0.01, self.control_loop)  # 100 Hz

        self.get_logger().info("RS005L controller node initialized (100 Hz)")

    # ------------------------------------------------------------------
    def joint_state_cb(self, msg: JointState):
        if len(msg.position) >= 6:
            self.q = np.array(msg.position[:6]) * 180.0 / np.pi  # rad→deg
        if len(msg.velocity) >= 6:
            self.dq = np.array(msg.velocity[:6]) * 180.0 / np.pi

    def joint_ref_cb(self, msg: JointState):
        if len(msg.position) >= 6:
            self.q_ref = np.array(msg.position[:6]) * 180.0 / np.pi
        if len(msg.velocity) >= 6:
            self.dq_ref = np.array(msg.velocity[:6]) * 180.0 / np.pi

    # ------------------------------------------------------------------
    def control_loop(self):
        q, dq, qd, dq_d = self.q, self.dq, self.q_ref, self.dq_ref
        e = qd - q
        de = dq_d - dq

        M = mass_matrix(q)
        C = coriolis_matrix(q, dq)
        g = gravity_vector(q)
        F = friction_torque(dq)

        qdd_des = -KD * de - KP * e

        # scenario adjustments
        scenario = self.scenario
        if scenario == 'S2':
            g *= 1.3
        elif scenario == 'S4':
            M *= 0.8; C *= 0.8; g *= 0.8

        tau = M @ qdd_des + C @ dq + g + F
        if scenario == 'S3':
            tau = np.clip(tau, -0.7*TAU_MAX, 0.7*TAU_MAX)
        else:
            tau = np.clip(tau, -TAU_MAX, TAU_MAX)

        msg = Float64MultiArray()
        msg.data = tau.tolist()
        self.torque_pub.publish(msg)

        self.get_logger().debug(f"Effort={np.linalg.norm(tau):.2f} | e_norm={np.linalg.norm(e):.2f}")

# ----------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = RSControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
