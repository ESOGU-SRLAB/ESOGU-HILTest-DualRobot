#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np
from math import sin, cos, pi
import time

# model.py: FK, Jacobian, basit IK içeriyor (RS005L) :contentReference[oaicite:2]{index=2}
from model import forward_kinematics, inverse_kinematics_xyz

class StreamRefNode(Node):
    def __init__(self):
        super().__init__('mm_stream_ref')

        # === CONFIG ===
        self.CONTROLLER_NAME = 'joint_trajectory_controller'   # senin controller adını yaz
        self.JOINT_TOPIC = f'/{self.CONTROLLER_NAME}/joint_trajectory'
        self.JOINT_NAMES = [
            'joint_1','joint_2','joint_3','joint_4','joint_5','joint_6'
        ]  # URDF/JTC sıranla aynı olmalı
        self.rate_hz = 100.0
        self.dt = 1.0/self.rate_hz
        self.horizon = 0.2          # tek nokta için time_from_start
        self.max_vel_deg = 30.0     # deg/s sınırı (gerekirse arttır)
        self.max_step_deg = self.max_vel_deg * self.dt

        # uç-efektör referans (basit bir daire)
        self.center = np.array([0.45, 0.0, 0.55])  # x,y,z (m)
        self.radius = 0.05
        self.omega = 0.25 * 2*pi  # rad/s (dönüş hızı)
        self.phase = 0.0

        # durumlar
        self.q_deg = np.zeros(6)      # son okunan pozisyon (deg)
        self.have_js = False

        # ROS I/O
        self.js_sub = self.create_subscription(
            JointState, '/joint_states', self.js_cb, 10
        )
        self.traj_pub = self.create_publisher(
            JointTrajectory, self.JOINT_TOPIC, 10
        )

        self.timer = self.create_timer(self.dt, self.loop)

        # loglar
        self.err_log = []
        self.t0 = time.time()

        self.get_logger().info(f'Publishing to {self.JOINT_TOPIC}')

    def js_cb(self, msg: JointState):
        # JTC eklem sırası ile /joint_states sırası aynı değilse eşleştirmen gerekebilir
        pos = np.array(msg.position[:6])  # rad
        self.q_deg = pos * 180.0/np.pi
        self.have_js = True

    def ee_ref(self, t):
        # dairesel referans (x,y) ve sabit z
        x = self.center[0] + self.radius * cos(self.omega*t + self.phase)
        y = self.center[1] + self.radius * sin(self.omega*t + self.phase)
        z = self.center[2]
        return np.array([x,y,z])

    def loop(self):
        if not self.have_js:
            return

        t = time.time() - self.t0
        xd = self.ee_ref(t)

        # IK: mevcut q'dan küçük adımlarla hedefe git
        qd_deg = inverse_kinematics_xyz(
            target_pos=xd,
            init_q_deg=self.q_deg.copy(),
            max_iter=60, alpha=0.08, tol=1e-4
        )  # :contentReference[oaicite:3]{index=3}

        # hız/merdiven sınırlaması: JTC'nin pürüzsüz çalışması için
        step = np.clip(qd_deg - self.q_deg, -self.max_step_deg, self.max_step_deg)
        q_cmd_deg = self.q_deg + step

        # Hata ve metrikler (ödev raporu için)
        # FK ile ee_current (model.py)
        ee_cur = forward_kinematics(self.q_deg)
        e = xd - ee_cur
        self.err_log.append((t, float(np.linalg.norm(e,2))))

        # Trajectory mesajı (tek noktalı, kısa ufuk)
        traj = JointTrajectory()
        traj.joint_names = self.JOINT_NAMES
        pt = JointTrajectoryPoint()
        pt.positions = list((q_cmd_deg * np.pi/180.0))  # rad
        # istersen hız/ivme hedefleri de eklersin:
        # pt.velocities = [0.0]*6
        # pt.accelerations = [0.0]*6
        pt.time_from_start = rclpy.duration.Duration(seconds=self.horizon).to_msg()
        traj.points = [pt]
        self.traj_pub.publish(traj)

        # opsiyonel: her 5 sn'de bir RMS/ITAE ara çıktısı
        if int(t) % 5 == 0 and len(self.err_log) > 10:
            errs = np.array([x[1] for x in self.err_log[-int(self.rate_hz*5):]])
            rms = float(np.sqrt(np.mean(errs**2)))
            self.get_logger().info(f'5s-window RMS |e| ≈ {rms:.4f} m')

def main():
    rclpy.init()
    node = StreamRefNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
