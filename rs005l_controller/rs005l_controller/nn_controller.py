#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS2 Node – Self-contained Neural Network Controller for RS005L arm
Generates its own trajectory and publishes torque commands.

Author: Cem Süha Yılmaz (2025)
"""

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import torch
import torch.nn as nn
import torch.optim as optim

from rs005l_controller.model import (
    mass_matrix, gravity_vector, coriolis_matrix, friction_torque
)

KP = np.array([8.0, 8.0, 6.0, 4.0, 3.0, 3.0])
KD = np.array([1.8, 1.8, 1.4, 1.0, 0.8, 0.8])
TAU_MAX = np.array([180, 180, 120, 80, 60, 60], dtype=float)


# ----------------------------------------------------------------------
# Neural Network correction model
# ----------------------------------------------------------------------
class NNCorrection(nn.Module):
    def __init__(self, input_dim=12, hidden=64, output_dim=6):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(input_dim, hidden),
            nn.Tanh(),
            nn.Linear(hidden, hidden),
            nn.Tanh(),
            nn.Linear(hidden, output_dim)
        )

    def forward(self, x):
        return self.net(x)


# ----------------------------------------------------------------------
class NNControllerNode(Node):
    def __init__(self):
        super().__init__('nn_controller_node')

        # ROS parameters
        self.declare_parameter('scenario', 'S4')
        self.declare_parameter('traj_type', 'sin')
        self.declare_parameter('amp_deg', 15.0)
        self.declare_parameter('freq_hz', 0.2)
        self.declare_parameter('t_total', 20.0)

        self.scenario = self.get_parameter('scenario').get_parameter_value().string_value
        self.traj_type = self.get_parameter('traj_type').get_parameter_value().string_value
        self.amp = self.get_parameter('amp_deg').get_parameter_value().double_value
        self.freq = self.get_parameter('freq_hz').get_parameter_value().double_value
        self.t_total = self.get_parameter('t_total').get_parameter_value().double_value

        # Publishers
        self.torque_pub = self.create_publisher(Float64MultiArray,
                                                '/nn_controller/torque_cmd', 10)

        # Internal state
        self.q = np.zeros(6)
        self.dq = np.zeros(6)
        self.t = 0.0
        self.dt = 0.01

        # Neural network setup
        self.device = torch.device('cpu')
        self.net = NNCorrection().to(self.device)
        self.optimizer = optim.Adam(self.net.parameters(), lr=1e-3)
        self.loss_fn = nn.MSELoss()

        # Timer
        self.timer = self.create_timer(self.dt, self.control_loop)
        self.get_logger().info(f"NN controller initialized. Scenario={self.scenario}, trajectory={self.traj_type}")

    # ------------------------------------------------------------------
    def generate_reference(self, t):
        """Generate joint reference trajectory (degrees)"""
        if self.traj_type == "sin":
            qd = self.amp * np.sin(2 * np.pi * self.freq * t) * np.ones(6)
            dq_d = self.amp * 2 * np.pi * self.freq * np.cos(2 * np.pi * self.freq * t) * np.ones(6)
        elif self.traj_type == "step":
            qd = np.ones(6) * (self.amp if t > 2.0 else 0.0)
            dq_d = np.zeros(6)
        else:
            qd = np.zeros(6)
            dq_d = np.zeros(6)
        return qd, dq_d

    # ------------------------------------------------------------------
    def control_loop(self):
        if self.t > self.t_total:
            self.get_logger().info("Trajectory completed. Shutting down NN controller.")
            self.destroy_timer(self.timer)
            return

        qd, dq_d = self.generate_reference(self.t)
        q, dq = self.q, self.dq
        e = qd - q
        de = dq_d - dq

        # Nominal dynamics
        M = mass_matrix(q)
        C = coriolis_matrix(q, dq)
        g = gravity_vector(q)
        F = friction_torque(dq)

        qdd_des = -KD * de - KP * e
        tau_nom = M @ qdd_des + C @ dq + g + F

        # Scenario adjustments
        s = self.scenario
        if s == "S2":
            g *= 1.3
        elif s == "S4":
            M *= 0.8; C *= 0.8; g *= 0.8
        if s == "S3":
            tau_nom = np.clip(tau_nom, -0.7 * TAU_MAX, 0.7 * TAU_MAX)
        else:
            tau_nom = np.clip(tau_nom, -TAU_MAX, TAU_MAX)

        # NN correction
        x = np.concatenate([q, dq])
        x_t = torch.tensor(x, dtype=torch.float32, device=self.device).unsqueeze(0)
        delta_tau_pred = self.net(x_t)
        delta_tau = delta_tau_pred.detach().cpu().numpy().squeeze()

        tau_cmd = tau_nom + delta_tau
        tau_cmd = np.clip(tau_cmd, -TAU_MAX, TAU_MAX)

        # Publish torque
        msg = Float64MultiArray()
        msg.data = tau_cmd.tolist()
        self.torque_pub.publish(msg)

        # Online training
        delta_tau_target = torch.tensor(-KP * e - KD * de, dtype=torch.float32, device=self.device).unsqueeze(0)
        loss = self.loss_fn(delta_tau_pred, delta_tau_target)
        self.optimizer.zero_grad()
        loss.backward()
        self.optimizer.step()

        # Simple state integration (simulate position)
        self.dq += 0.001 * (qd - q)
        self.q += self.dq * self.dt

        self.t += self.dt
        self.get_logger().debug(f"t={self.t:.2f} Eff={np.linalg.norm(tau_cmd):.2f} Loss={loss.item():.4f}")


# ----------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = NNControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
