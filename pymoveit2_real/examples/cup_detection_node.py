#!/usr/bin/env python3
"""
Akış: WP1 -> WP2 -> WP3 -> WP4

Joint sırası (7 eleman):
  [base_to_robot_mount, shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3]
"""

from threading import Thread
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from pymoveit2_real import MoveIt2 as MoveIt2_Sim
from pymoveit2_real.robots import ur as robot


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
        self.arm.max_velocity = 1.5
        self.arm.max_acceleration = 2.0

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
        0.5510627202725462,    # ur10e_base_to_robot_mount
        -0.8703046785097994,   # ur10e_shoulder_pan_joint
        -1.1745171528339018,   # ur10e_shoulder_lift_joint
        2.321656252132529,     # ur10e_elbow_joint
        -4.286027955081261,    # ur10e_wrist_1_joint
        -0.60656592381728,     # ur10e_wrist_2_joint
        -0.0022165086723359615,  # ur10e_wrist_3_joint
    ]

    WP2 = [
        0.390115645440091,     # ur10e_base_to_robot_mount
        -0.8900989152797043,   # ur10e_shoulder_pan_joint
        -1.139474835690132,    # ur10e_shoulder_lift_joint
        2.25138199969191,      # ur10e_elbow_joint
        -4.250553936062228,    # ur10e_wrist_1_joint
        -0.5867986574621117,   # ur10e_wrist_2_joint
        -0.0021484867272167606,  # ur10e_wrist_3_joint
    ]

    WP3 = [
        0.1851201675663802,    # ur10e_base_to_robot_mount
        -0.9108595081122506,   # ur10e_shoulder_pan_joint
        -1.0935405275334729,   # ur10e_shoulder_lift_joint
        2.158637222667278,     # ur10e_elbow_joint
        -4.203858209867298,    # ur10e_wrist_1_joint
        -0.5659714361971715,   # ur10e_wrist_2_joint
        -0.0023170394323231177,  # ur10e_wrist_3_joint
    ]

    WP4 = [
        0.05009613425347489,   # ur10e_base_to_robot_mount
        -0.9925759729846219,   # ur10e_shoulder_pan_joint
        -0.9982114026205913,   # ur10e_shoulder_lift_joint
        1.9665226530054665,    # ur10e_elbow_joint
        -4.106362536742907,    # ur10e_wrist_1_joint
        -0.4843230684640321,   # ur10e_wrist_2_joint
        -0.00285680131720156,  # ur10e_wrist_3_joint
    ]

    waypoints = [
        (WP1, "WP1"),
        (WP2, "WP2"),
        (WP3, "WP3"),
        (WP4, "WP4"),
    ]

    try:
        for joints, label in waypoints:
            if not ctrl.move_arm_retry(joints, label, retries=3):
                ctrl.get_logger().error(f"{label} başarısız, durduruluyor.")
                return

        ctrl.get_logger().info("=== DONE ✅ ===")

    finally:
        rclpy.shutdown()
        t.join()


if __name__ == "__main__":
    main()