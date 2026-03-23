#!/usr/bin/env python3
"""
Real Sensing Mirror Node
- /harmony/robot_status topic'ini dinler
- SR_MODE başladığında gerçek robot sensing_robot_v3.py ile aynı joint waypoint'lerini dolaşır
- WAITING/COMPLETE/IDLE gelince durur veya waiting pozisyonuna gider
"""

from __future__ import annotations

from threading import Thread, Event, Lock
import time
import json
import math
from typing import List, Optional

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String

from pymoveit2_real import MoveIt2 as MoveIt2_Real
from pymoveit2_real.robots import ur as robot


class RealSensingMirrorNode(Node):
    def __init__(self):
        super().__init__("real_sensing_mirror")

        # ---------------- Params ----------------
        self.declare_parameter("scan_wait_time", 1.5)
        self.declare_parameter("max_retries", 3)
        self.declare_parameter("max_velocity", 0.05)
        self.declare_parameter("max_acceleration", 0.05)

        self.scan_wait_time = float(self.get_parameter("scan_wait_time").value)
        self.max_retries = int(self.get_parameter("max_retries").value)

        cbg = ReentrantCallbackGroup()

        # ---------------- MoveIt2 (Real) ----------------
        self.moveit2 = MoveIt2_Real(
            node=self,
            joint_names=robot.joint_names(),
            base_link_name=robot.base_link_name(),
            end_effector_name=robot.end_effector_name(),
            group_name=robot.MOVE_GROUP_ARM,
            callback_group=cbg,
        )
        self.moveit2.planner_id = "ESTkConfigDefault"
        self.moveit2.max_velocity = float(self.get_parameter("max_velocity").value)
        self.moveit2.max_acceleration = float(self.get_parameter("max_acceleration").value)
        self.moveit2.cartesian_avoid_collisions = True
        self.moveit2.cartesian_jump_threshold = 2.0

        # ---------------- Subscribers ----------------
        self.status_sub = self.create_subscription(
            String, "/harmony/robot_status", self._status_cb, 10, callback_group=cbg
        )

        # ---------------- State ----------------
        self._state_lock = Lock()
        self._current_state = "IDLE"
        self._current_mode = "IDLE"

        self.start_event = Event()
        self.stop_event = Event()

        # Worker thread
        self.worker = Thread(target=self._run, daemon=True)
        self.worker.start()

        self.get_logger().info("RealSensingMirrorNode initialized — waiting for SR_MODE signal")

    # -----------------------------------------------------------------------
    def _status_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            state = str(data.get("state", "")).upper()
            mode = str(data.get("mode", "")).upper()
        except Exception:
            return

        with self._state_lock:
            prev = self._current_state
            self._current_state = state
            self._current_mode = mode

        if state == "SR_MODE" and prev != "SR_MODE":
            self.get_logger().info("SR_MODE detected → triggering real robot sensing sequence")
            self.stop_event.clear()
            self.start_event.set()

        elif state in ("IDLE", "WAITING", "COMPLETE", "CR_MODE") and prev == "SR_MODE":
            self.get_logger().info(f"State changed to {state} → stopping real robot scan")
            self.stop_event.set()
            self.start_event.clear()

    # -----------------------------------------------------------------------
    def _move_to_joints(self, joint_positions: List[float], synchronous: bool = True) -> bool:
        try:
            self.moveit2.move_to_configuration(joint_positions)
            if synchronous:
                return bool(self.moveit2.wait_until_executed())
            return True
        except Exception as e:
            self.get_logger().warning(f"Joint move failed: {e}")
            return False

    def _safe_joint_move(self, joint_positions: List[float]) -> bool:
        """Retry destekli joint hareketi."""
        for attempt in range(1, self.max_retries + 1):
            if self.stop_event.is_set():
                return False
            self.get_logger().info(f"  Joint move attempt {attempt}/{self.max_retries}")
            if self._move_to_joints(joint_positions):
                return True
            time.sleep(0.5)
        self.get_logger().warning("  All attempts failed — skipping this waypoint")
        return False

    # -----------------------------------------------------------------------
    def _run(self):
        # ----------------------------------------------------------------
        # sensing_robot_v3.py ile BİREBİR aynı joint açıları
        # prefix: ur10e_ (gerçek robot)   sim prefix'i sim_ idi
        # ----------------------------------------------------------------
        home_joints        = [1.0,  0.0,           -math.pi / 2,            0.0,             -math.pi / 2,            0.0,            0.0]
        pose2_joints       = [1.8,  math.radians(0),           math.radians(-90.0),  math.radians(0.0),   math.radians(-90.0),  math.radians(0.0),  math.radians(0.0)]
        pose3_joints       = [1.0,  math.radians(86.40),       math.radians(-44.74), math.radians(98.25), math.radians(-233.91),math.radians(-90.0),math.radians(90.0)]
        pose4_joints       = [1.0,  math.radians(86.39),       math.radians(-63.12), math.radians(90.33), math.radians(-212.68),math.radians(-90.0),math.radians(90.0)]
        pose5_joints       = [1.0,  math.radians(86.37),       math.radians(-66.12), math.radians(45.49), math.radians(-164.0), math.radians(-90.0),math.radians(90)]
        pose6_joints       = [1.0,  math.radians(94.91),       math.radians(-111.64),math.radians(-37.60),math.radians(-28.84), math.radians(-90.0),math.radians(90)]
        pose7_joints       = [1.0,  math.radians(94.84),       math.radians(-105.96),math.radians(-90.55),math.radians(-18.31), math.radians(-90.0),math.radians(90)]
        pose8_joints       = [1.0,  1.5542166358552454,        -2.3765672787497856,  -1.8370296687945291,  1.0718900852758073, -1.4855356703907372, math.radians(90)]
        pose9_joints       = [1.0,  math.radians(109.13),      math.radians(-57.99), math.radians(-83.46),math.radians(-40.12),math.radians(-90),   math.radians(90)]
        pose10_joints      = [1.0,  math.radians(108.91),      math.radians(-46.56), math.radians(-115.55),math.radians(-19.46),math.radians(-90),  math.radians(90)]
        pose11_joints      = [1.0,  math.radians(54),          math.radians(-92),    math.radians(-156),  math.radians(65),    math.radians(-36),  math.radians(93)]
        waiting_joints     = [1.0,  0.0,           -math.pi / 2,            0.0,             -math.pi / 2,            0.0,            0.0]

        scan_waypoints = [
            pose2_joints,   # home
            pose5_joints,   # sağ üst
            pose9_joints,   # orta üst
            pose6_joints,   # sol üst
            pose7_joints,   # sol orta
            pose10_joints,  # orta orta
            pose4_joints,   # sağ orta
            pose3_joints,   # sağ alt
            pose11_joints,  # orta alt
            pose8_joints,   # sol alt
        ]

        while rclpy.ok():
            # IDLE: start_event bekle
            if not self.start_event.wait(timeout=0.2):
                continue

            if self.stop_event.is_set():
                self.start_event.clear()
                continue

            self.get_logger().info("Starting real robot sensing scan sequence")

            # Home
            self.get_logger().info("Moving to home position")
            self._safe_joint_move(home_joints)
            time.sleep(1.0)

            # Waypoint sekansı
            for i, wp in enumerate(scan_waypoints):
                if self.stop_event.is_set():
                    self.get_logger().info("Stop requested — aborting scan")
                    break

                self.get_logger().info(f"Moving to scan waypoint {i+1}/{len(scan_waypoints)}")
                self._safe_joint_move(wp)

                if i < len(scan_waypoints) - 1:
                    # stop_event'e duyarlı dwell
                    t0 = time.time()
                    while time.time() - t0 < self.scan_wait_time:
                        if self.stop_event.is_set():
                            break
                        time.sleep(0.05)

            # Scan bitti veya erken kesildi → waiting pozisyonu
            self.get_logger().info("Scan complete — moving to waiting position")
            self._safe_joint_move(waiting_joints)

            self.start_event.clear()
            self.get_logger().info("Real sensing mirror: waiting for next SR_MODE signal")


def main():
    rclpy.init()
    node = RealSensingMirrorNode()
    exec_ = MultiThreadedExecutor(num_threads=4)
    exec_.add_node(node)
    try:
        exec_.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
