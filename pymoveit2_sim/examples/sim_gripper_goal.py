#!/usr/bin/env python3
"""
Akış (GÜNCEL):
HOME -> PICK_START_UP -> (micro) -> PICK_START -> CLOSE -> MIDPOINT_PICK1 -> (micro) -> PLACE_NEAR -> OPEN
-> PICK_END_UP -> (micro) -> PICK_END -> CLOSE -> MIDPOINT_PICK2 -> (micro) -> PLACE_FAR -> OPEN -> HOME
"""

from threading import Thread
import time
import math

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from pymoveit2_sim import MoveIt2 as MoveIt2_Sim
from pymoveit2_sim import MoveIt2Gripper
from pymoveit2_sim.robots import ur as robot


def lerp(a: float, b: float, t: float) -> float:
    return a + (b - a) * t


def interpolate_joints(q_from, q_to, steps: int):
    out = []
    for k in range(1, steps + 1):
        t = k / (steps + 1)
        out.append([lerp(q_from[i], q_to[i], t) for i in range(len(q_from))])
    return out


class ArmGripperController(Node):
    def __init__(self):
        super().__init__("arm_gripper_full_flow_no_attach_updated")

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

        gj = robot.gripper_joint_names()
        gripper_joint_names = gj if isinstance(gj, (list, tuple)) else [gj]

        self.gripper = MoveIt2Gripper(
            node=self,
            gripper_joint_names=gripper_joint_names,
            open_gripper_joint_positions=robot.OPEN_GRIPPER_JOINT_POSITIONS,
            closed_gripper_joint_positions=robot.CLOSED_GRIPPER_JOINT_POSITIONS,
            gripper_group_name=robot.MOVE_GROUP_GRIPPER,
            callback_group=cbg,
        )

        self.get_logger().info("Arm+Gripper controller ready (NO attach/detach)")

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

    def micro_move(self, q_from, q_to, label, steps=4):
        self.get_logger().info(f"--- MICRO MOVE: {label} (steps={steps}) ---")
        for idx, q in enumerate(interpolate_joints(q_from, q_to, steps=steps), start=1):
            ok = self.move_arm_retry(q, f"{label}_STEP_{idx}", retries=2, sleep_between=0.2)
            if not ok:
                return False
            time.sleep(0.05)
        return True

    def gripper_close(self):
        self.get_logger().info("=== GRIPPER CLOSE ===")
        self.gripper.close()
        ok = self.gripper.wait_until_executed()
        time.sleep(0.3)
        self.get_logger().info("GRIPPER CLOSE ok ✅" if ok else "GRIPPER CLOSE fail ❌")
        return ok

    def gripper_open(self):
        self.get_logger().info("=== GRIPPER OPEN ===")
        self.gripper.open()
        ok = self.gripper.wait_until_executed()
        time.sleep(0.3)
        self.get_logger().info("GRIPPER OPEN ok ✅" if ok else "GRIPPER OPEN fail ❌")
        return ok


def main():
    rclpy.init()
    ctrl = ArmGripperController()

    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(ctrl)
    t = Thread(target=executor.spin, daemon=True)
    t.start()

    time.sleep(0.5)

    # ===== WAYPOINTS (7 eleman) =====
    HOME_7 = [1.0, 0.0, -math.pi / 2, 0.0, -math.pi / 2, 0.0, 0.0]

    PICK_START_7 = [
        1.74461,
        -0.08670795723907876,
        -0.4448495197483142,
        0.23373449342708064,
        1.3244954627534575,
        0.48254863159139205,
        0.48254863159139205,
    ]

    # PICK_START_UP (bardağın üstünde yaklaşma konumu)
    PICK_START_UP_7 = [
        1.74461,
        -0.08670795723907876,
        -0.4636990756698536,
        0.1300619358586177,
        1.3244954627534575,
        0.48254863159139205,
        0.591876055936317,
    ]

    MIDPOINT_PICK1_7 = [
        0.05,
        0.6534512719466772,
        -1.1473096370909923,
        0.9795485893892977,
        -2.509504211687527,
        -1.5959290680236151,
        -1.5959290680236151,
    ]

    PLACE_NEAR_7 = [
        0.05,
        0.5881061447520084,
        -0.9475043443226818,
        1.1378848591302235,
        -2.789734276387736,
        -1.5959290680236151,
        -1.5959290680236151,
    ]

    # ===== YENİ 2. PICK KISMI (SENİN JOINTLERİN) =====
    PICK_END_UP_7 = [
        1.74461,
        -0.16964600329384893,
        -1.019132656824529,
        1.337690151898534,
        1.3848140417023806,
        0.4410796085640074,
        -0.16587609210954124,
    ]

    PICK_END_7 = [
        1.74461,
        -0.12440706908215571,
        -0.9550441666912972,
        1.2981060844633028,
        1.3848140417023806,
        0.4410796085640074,
        -0.16587609210954124,

    ]

    MIDPOINT_PICK2_7 = [
        1.08721,
        0.007539822368616278,
        -1.4602122653885363,
        1.375389263741611,
        1.3848140417023806,
        0.5240176546187767,
        0.15833626974092496,
    ]

    PLACE_FAR_7 = [
        1.08721,
        0.007539822368616278,
        -0.84948665353068,
        0.7979645340118076,
        1.3848140417023806,
        0.5240176546187767,
        0.15833626974092496,
    ]

    try:
        # HOME
        if not ctrl.move_arm_retry(HOME_7, "HOME", retries=3):
            return

        # PICK_START_UP
        if not ctrl.move_arm_retry(PICK_START_UP_7, "PICK_START_UP", retries=3):
            ctrl.move_arm_retry(HOME_7, "HOME_RETURN_FAIL", retries=3)
            return

        # Hassas yaklaşma: PICK_START_UP -> PICK_START
        if not ctrl.micro_move(PICK_START_UP_7, PICK_START_7, "UP_TO_PICK_START_PRECISE", steps=4):
            ctrl.move_arm_retry(HOME_7, "HOME_RETURN_FAIL", retries=3)
            return

        # PICK_START
        if not ctrl.move_arm_retry(PICK_START_7, "PICK_START", retries=2):
            ctrl.move_arm_retry(HOME_7, "HOME_RETURN_FAIL", retries=3)
            return

        # CLOSE (pick1)
        ctrl.gripper_close()

        # MIDPOINT (pick1)
        if not ctrl.move_arm_retry(MIDPOINT_PICK1_7, "MIDPOINT_PICK1_1", retries=3):
            ctrl.move_arm_retry(HOME_7, "HOME_RETURN_FAIL", retries=3)
            return

        # micro: MID1 -> PLACE_NEAR
        if not ctrl.micro_move(MIDPOINT_PICK1_7, PLACE_NEAR_7, "MID1_TO_PLACE_NEAR", steps=4):
            ctrl.move_arm_retry(HOME_7, "HOME_RETURN_FAIL", retries=3)
            return

        # PLACE_NEAR
        if not ctrl.move_arm_retry(PLACE_NEAR_7, "PLACE_NEAR", retries=3):
            ctrl.move_arm_retry(HOME_7, "HOME_RETURN_FAIL", retries=3)
            return

        # OPEN (place1)
        ctrl.gripper_open()

        # ====== YENİ AKIŞ: PICK_END_UP -> (micro) -> PICK_END -> CLOSE -> MIDPOINT_PICK2 -> (micro) -> PLACE_FAR -> OPEN -> HOME ======

        # PICK_END_UP
        if not ctrl.move_arm_retry(PICK_END_UP_7, "PICK_END_UP", retries=3):
            ctrl.move_arm_retry(HOME_7, "HOME_RETURN_FAIL", retries=3)
            return

        # micro: PICK_END_UP -> PICK_END
        if not ctrl.micro_move(PICK_END_UP_7, PICK_END_7, "ENDUP_TO_END_PRECISE", steps=4):
            ctrl.move_arm_retry(HOME_7, "HOME_RETURN_FAIL", retries=3)
            return

        # PICK_END (oturt)
        if not ctrl.move_arm_retry(PICK_END_7, "PICK_END", retries=2):
            ctrl.move_arm_retry(HOME_7, "HOME_RETURN_FAIL", retries=3)
            return

        # CLOSE (pick2)
        ctrl.gripper_close()

        # MIDPOINT_PICK2
        if not ctrl.move_arm_retry(MIDPOINT_PICK2_7, "MIDPOINT_PICK2_1", retries=3):
            ctrl.move_arm_retry(HOME_7, "HOME_RETURN_FAIL_AFTER_PICK2", retries=3)
            return

        # micro: MIDPOINT_PICK2 -> PLACE_FAR
        if not ctrl.micro_move(MIDPOINT_PICK2_7, PLACE_FAR_7, "MID2_TO_PLACE_FAR", steps=4):
            ctrl.move_arm_retry(HOME_7, "HOME_RETURN_FAIL_AFTER_PICK2", retries=3)
            return

        # PLACE_FAR
        if not ctrl.move_arm_retry(PLACE_FAR_7, "PLACE_FAR", retries=3):
            ctrl.move_arm_retry(HOME_7, "HOME_RETURN_FAIL_AFTER_PICK2", retries=3)
            return

        # OPEN (place2)
        ctrl.gripper_open()

        # HOME
        ctrl.move_arm_retry(HOME_7, "HOME_RETURN", retries=3)
        ctrl.get_logger().info("=== DONE ✅ ===")

    finally:
        rclpy.shutdown()
        t.join()


if __name__ == "__main__":
    main()