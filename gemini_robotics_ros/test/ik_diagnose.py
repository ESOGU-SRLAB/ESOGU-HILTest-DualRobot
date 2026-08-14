#!/usr/bin/env python3
"""
"Erişim yok (IK -31)" NEDEN geliyor? Tek koşuda ayırt eder.

SALT OKUMA: yalnızca /compute_ik ve /check_state_validity çağırır. İkisi de
sorgu servisidir - robot HAREKET ETMEZ, plan yürütülmez, hiçbir şey yayınlanmaz.

    python3 test/ik_diagnose.py
    python3 test/ik_diagnose.py --x 0.669 --y 0.139 --z 0.903

Sıradaki her adım farklı bir hipotezi eler:

  A. Model durumu eksik mi?      /joint_states'te eksik eklem var mı
  B. Poz gerçekten uzakta mı?    ray IK grubunda, yani taban kayabiliyor
  C. Yaw kilidi mi?              emme kabı dönel simetrik ama IK tam oryantasyon
                                 istiyor - yaw'ı taradığımızda çözülüyor mu
  D. Bütçe mi?                   uzun timeout ile çözülüyor mu
  E. Çarpışma mı?                avoid_collisions kapalıyken çözülüyor mu
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import threading
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from gemini_robotics_ros.grasp import approach_quaternion

GROUP = "real_ur10e"
EE_LINK = "ur10e_suction_cup"
TOOL_AXIS = (0.8660254, 0.5, 0.0)


def _quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def _spin_about(axis, angle):
    a = np.asarray(axis, float)
    a = a / np.linalg.norm(a)
    s = math.sin(angle / 2.0)
    return (a[0] * s, a[1] * s, a[2] * s, math.cos(angle / 2.0))


class Diag(Node):
    def __init__(self):
        super().__init__("ik_diagnose")
        self.cbg = ReentrantCallbackGroup()
        self.ik = self.create_client(GetPositionIK, "/compute_ik", callback_group=self.cbg)
        self.joint_state = None
        self.description = None
        self.create_subscription(JointState, "/joint_states",
                                 lambda m: setattr(self, "joint_state", m), 10,
                                 callback_group=self.cbg)
        from rclpy.qos import QoSProfile, QoSDurabilityPolicy
        latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, "/robot_description",
                                 lambda m: setattr(self, "description", m.data), latched,
                                 callback_group=self.cbg)

    def call(self, position, quat, timeout=1.0, avoid_collisions=True, seed=None):
        req = GetPositionIK.Request()
        req.ik_request.group_name = GROUP
        req.ik_request.ik_link_name = EE_LINK
        req.ik_request.avoid_collisions = avoid_collisions
        req.ik_request.timeout.sec = int(timeout)
        req.ik_request.timeout.nanosec = int((timeout - int(timeout)) * 1e9)
        req.ik_request.robot_state.is_diff = True
        if seed is not None:
            req.ik_request.robot_state.joint_state = seed
        pose = PoseStamped()
        pose.header.frame_id = "world"
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = map(float, position)
        (pose.pose.orientation.x, pose.pose.orientation.y,
         pose.pose.orientation.z, pose.pose.orientation.w) = map(float, quat)
        req.ik_request.pose_stamped = pose

        fut = self.ik.call_async(req)
        deadline = time.monotonic() + timeout + 3.0
        while not fut.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        res = fut.result()
        if res is None:
            return None, None
        return int(res.error_code.val), res.solution.joint_state


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--x", type=float, default=0.669)
    ap.add_argument("--y", type=float, default=0.139)
    ap.add_argument("--z", type=float, default=0.903, help="TEMAS noktası (world)")
    ap.add_argument("--tcp", type=float, default=0.0843, help="uç -> frame uzaklığı")
    args = ap.parse_args()

    rclpy.init()
    node = Diag()
    ex = MultiThreadedExecutor(4)
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()

    if not node.ik.wait_for_service(timeout_sec=8.0):
        print("HATA: /compute_ik yok - move_group çalışmıyor.")
        return 1
    for _ in range(100):
        if node.joint_state is not None and node.description is not None:
            break
        time.sleep(0.05)

    # --- A. model durumu ---
    print("=" * 72)
    print("A. MODEL DURUMU")
    print("=" * 72)
    if node.joint_state is None:
        print("  /joint_states HİÇ gelmedi.")
        model_joints = set()
    else:
        have = dict(zip(node.joint_state.name, node.joint_state.position))
        print(f"  /joint_states: {len(have)} eklem")
        model_joints = set()
        if node.description:
            import xml.etree.ElementTree as ET
            root = ET.fromstring(node.description)
            model_joints = {
                j.get("name") for j in root.findall("joint")
                if j.get("type") in ("revolute", "prismatic", "continuous")
            }
            missing = sorted(model_joints - set(have))
            print(f"  URDF'te hareketli eklem: {len(model_joints)}")
            if missing:
                print(f"  *** /joint_states'TE EKSİK: {missing}")
                print("      MoveIt'in güncel durumu eksikse IK sistematik olarak düşer.")
            else:
                print("  eksik eklem yok")
        for key in ("ur10e_base_to_robot_mount", "world_to_agv"):
            if key in have:
                print(f"    {key} = {have[key]:+.4f}")

    quat = approach_quaternion((0.0, 0.0, 1.0), approach_vector=TOOL_AXIS)
    contact = (args.x, args.y, args.z)

    # --- B/D/E: mesafe adayları ---
    print()
    print("=" * 72)
    print("B. YAKLAŞMA MESAFELERİ (koşudaki oryantasyonla, varsayılan bütçe)")
    print("=" * 72)
    for d in (0.15, 0.12, 0.10, 0.08, 0.06):
        pos = (contact[0], contact[1], contact[2] + d + args.tcp)
        code, sol = node.call(pos, quat, timeout=0.5, avoid_collisions=True)
        free, _ = (None, None) if code == 1 else node.call(pos, quat, timeout=0.5,
                                                           avoid_collisions=False)
        rail = ""
        if sol is not None and "ur10e_base_to_robot_mount" in sol.name:
            rail = f"  ray -> {sol.position[list(sol.name).index('ur10e_base_to_robot_mount')]:+.3f}"
        tag = "UYGUN" if code == 1 else ("ÇARPIŞMA" if free == 1 else "ERİŞİM YOK")
        print(f"  {d*100:4.0f} cm  z={pos[2]:.3f}  IK={code:4}  {tag}{rail}")

    # --- C. YAW TARAMASI: asıl şüpheli ---
    print()
    print("=" * 72)
    print("C. YAW TARAMASI (emme kabı dönel simetrik - yaw SERBEST olmalı)")
    print("=" * 72)
    print("  approach_quaternion yaw'ı yaw_reference=(1,0,0) ile SABİTLİYOR, ama")
    print("  IK isteği tam oryantasyon istiyor. Başka bir yaw çözülüyorsa hata bu.")
    print()
    for d in (0.15, 0.10, 0.06):
        pos = (contact[0], contact[1], contact[2] + d + args.tcp)
        ok_yaws = []
        for deg in range(0, 360, 15):
            # yaklaşma ekseni dünya -Z; onun etrafında döndürmek kabı döndürmez
            q = _quat_mul(_spin_about((0.0, 0.0, 1.0), math.radians(deg)), quat)
            code, _ = node.call(pos, q, timeout=0.3, avoid_collisions=True)
            if code == 1:
                ok_yaws.append(deg)
        verdict = "hiçbir yaw çözülmedi" if not ok_yaws else f"ÇÖZÜLEN yaw'lar: {ok_yaws}"
        print(f"  {d*100:4.0f} cm  {verdict}")

    # --- D. bütçe ---
    print()
    print("=" * 72)
    print("D. BÜTÇE (aynı poz, uzun timeout)")
    print("=" * 72)
    pos = (contact[0], contact[1], contact[2] + 0.10 + args.tcp)
    for t in (0.5, 3.0, 10.0):
        start = time.monotonic()
        code, _ = node.call(pos, quat, timeout=t, avoid_collisions=True)
        print(f"  timeout {t:5.1f}s -> IK={code:4}  ({(time.monotonic()-start)*1000:.0f} ms)")

    ex.shutdown(timeout_sec=2.0)
    node.destroy_node()
    rclpy.shutdown()
    print()
    print("YORUM:")
    print("  A'da eksik eklem varsa  -> önce onu düzelt, gerisi anlamsız.")
    print("  C'de bazı yaw'lar çözülüyorsa -> yaw kilidi. Doğrulama ve hareket")
    print("     yaw'ı serbest bırakmalı (emme kabı dönel simetrik).")
    print("  D'de uzun bütçe çözüyorsa -> ik_timeout_sec yetersiz.")
    print("  B'de hepsi ÇARPIŞMA ise  -> erişim değil, sahne/çarpışma sorunu.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
