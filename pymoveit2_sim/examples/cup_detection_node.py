#!/usr/bin/env python3
"""
Akış: WP1 -> WP2 -> WP3 -> WP4 -> scan_complete sinyali yayınla

Joint sırası (7 eleman):
  [base_to_robot_mount, shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3]
"""

from threading import Thread
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from std_msgs.msg import Bool

from pymoveit2_sim import MoveIt2 as MoveIt2_Sim
from pymoveit2_sim.robots import ur as robot


class ArmController(Node):
    def __init__(self):
        super().__init__("arm_waypoint_controller")

        cbg = ReentrantCallbackGroup()

        self.arm = MoveIt2_Sim(
            node=self,
            joint_names=robot.joint_names(),
            base_link_name=robot.base_link_name(),
            end_effector_name=robot.end_effector_name(),
            group_name=robot.MOVE_GROUP_ARM,
            callback_group=cbg,
        )
        self.arm.planner_id = "RRTConnectkConfigDefault"
        self.arm.max_velocity = 0.3
        self.arm.max_acceleration = 0.3

        # Tarama tamamlandı sinyali
        self._scan_pub = self.create_publisher(Bool, "/scan_complete", 10)

        self.get_logger().info("Arm controller ready")

    def move_arm(self, joints, label):
        self.get_logger().info(f"=== ARM: {label} ===")
        self.arm.move_to_configuration(joints)
        ok = self.arm.wait_until_executed()
        self.get_logger().info("ARM done ✅" if ok else "ARM fail ❌")
        return ok

    def move_arm_retry(self, joints, label, retries=3, sleep_between=0.4):
        for i in range(1, retries + 1):
            ok = self.move_arm(joints, f"{label} (try {i}/{retries})")
            if ok:
                return True
            time.sleep(sleep_between)
        return False


def main():
    rclpy.init()
    ctrl = ArmController()

    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(ctrl)
    t = Thread(target=executor.spin, daemon=True)
    t.start()

    time.sleep(0.5)

    # ===== WAYPOINTS (7 eleman) =====
    # Sıra: [base_to_robot_mount, shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3]

    WP1 = [
        0.892108759723234,    # ur10e_base_to_robot_mount
        -3.688792060742331,   # ur10e_shoulder_pan_joint
        -2.1437739303162116,   # ur10e_shoulder_lift_joint
        -1.773514489837773,     # ur10e_elbow_joint
        -1.3937463964686807,    # ur10e_wrist_1_joint
        -1.233626555285855,     # ur10e_wrist_2_joint
        3.633513651462601,  # ur10e_wrist_3_joint
    ]

    WP2 = [
        0.692108759723234,    # ur10e_base_to_robot_mount
        -3.688792060742331,   # ur10e_shoulder_pan_joint
        -2.1437739303162116,   # ur10e_shoulder_lift_joint
        -1.773514489837773,     # ur10e_elbow_joint
        -1.3937463964686807,    # ur10e_wrist_1_joint
        -1.233626555285855,     # ur10e_wrist_2_joint
        3.633513651462601,  # ur10e_wrist_3_joint
    ]

    WP3 = [
        0.402108759723234,    # ur10e_base_to_robot_mount
        -3.688792060742331,   # ur10e_shoulder_pan_joint
        -2.1437739303162116,   # ur10e_shoulder_lift_joint
        -1.773514489837773,     # ur10e_elbow_joint
        -1.3937463964686807,    # ur10e_wrist_1_joint
        -1.233626555285855,     # ur10e_wrist_2_joint
        3.633513651462601,  # ur10e_wrist_3_joint
    ]

    WP4 = [
        0.2124590793430805,    # ur10e_base_to_robot_mount
        -3.688792060742331,   # ur10e_shoulder_pan_joint
        -2.1437739303162116,   # ur10e_shoulder_lift_joint
        -1.773514489837773,     # ur10e_elbow_joint
        -1.3937463964686807,    # ur10e_wrist_1_joint
        -1.233626555285855,     # ur10e_wrist_2_joint
        3.633513651462601,  # ur10e_wrist_3_joint
    ]

    WP5 = [
        0.19164148352819865,    # ur10e_base_to_robot_mount
        -3.8302520849734045,   # ur10e_shoulder_pan_joint
        -2.276660721810225,   # ur10e_shoulder_lift_joint
        -1.3097428163964289,     # ur10e_elbow_joint
        -2.1156528768750062,    # ur10e_wrist_1_joint
         -2.280679559707314,     # ur10e_wrist_2_joint
        4.888195978879738,  # ur10e_wrist_3_joint
    ]

    waypoints = [
        (WP1, "WP1"),
        (WP2, "WP2"),
        (WP3, "WP3"),
        (WP4, "WP4"),
        (WP5, "WP5"),
    ]

    try:
        for joints, label in waypoints:
            if not ctrl.move_arm_retry(joints, label, retries=3):
                ctrl.get_logger().error(f"{label} başarısız, durduruluyor.")
                return

            # Waypoint'te kısa bekleme — kamera sabitken tespit şansı artır
            ctrl.get_logger().info(f"{label}: waypoint'te 3s bekleniyor")
            time.sleep(3.0)

        ctrl.get_logger().info("=== TARAMA TAMAMLANDI ✅ ===")

        # scan_complete sinyali yayınla (birkaç kere, güvenilirlik için)
        msg = Bool()
        msg.data = True
        for _ in range(5):
            ctrl._scan_pub.publish(msg)
            time.sleep(0.1)
        ctrl.get_logger().info("/scan_complete sinyali yayınlandı")

    finally:
        rclpy.shutdown()
        t.join()


if __name__ == "__main__":
    main()