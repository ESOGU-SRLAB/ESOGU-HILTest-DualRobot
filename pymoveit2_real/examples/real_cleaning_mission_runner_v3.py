#!/usr/bin/env python3
"""
Real Cleaning Mission Runner v3
- cleaning_mission_runner_v3.py'nin gerçek robot versiyonu
- pymoveit2_real kullanır (move_group: real_ur10e, prefix: ur10e_)
- Aynı /harmony/... topic'leri, aynı state machine
"""

from __future__ import annotations

from threading import Thread, Event, Lock
import time
import json
import random
from typing import Any, Dict, List, Optional

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String
from geometry_msgs.msg import WrenchStamped

from pymoveit2_real import MoveIt2 as MoveIt2_Real
from pymoveit2_real.robots import ur as robot


def _now_ros(node: Node) -> str:
    t = node.get_clock().now().to_msg()
    return f"{t.sec}.{t.nanosec:09d}"


class RealHarmonyCleaningMissionRunnerV3(Node):
    def __init__(self):
        super().__init__("harmony_cleaning_runner")   # Sim ile aynı node adı → aynı topic'ler

        # ---------------- Params ----------------
        self.declare_parameter("fixed_frame", "world")
        self.declare_parameter("tool_frame", "tool0")
        self.declare_parameter("approach_height_offset", 0.3)
        self.declare_parameter("cleaning_duration", 1.5)
        self.declare_parameter("paint_y_offset", 0.30)
        self.declare_parameter("paint_x_span", 0.2)
        self.declare_parameter("move_max_retries", 5)
        self.declare_parameter("move_retry_sleep_s", 0.2)
        self.declare_parameter("cartesian_max_step", 0.01)
        self.declare_parameter("cartesian_fraction_threshold", 0.40)
        self.declare_parameter("force_rate_hz", 30.0)
        self.declare_parameter("skip_if_no_defects", True)

        self.fixed_frame    = str(self.get_parameter("fixed_frame").value)
        self.tool_frame     = str(self.get_parameter("tool_frame").value)
        self.approach_offset = float(self.get_parameter("approach_height_offset").value)
        self.cleaning_duration = float(self.get_parameter("cleaning_duration").value)
        self.paint_y_offset = float(self.get_parameter("paint_y_offset").value)
        self.paint_x_span   = float(self.get_parameter("paint_x_span").value)
        self.move_max_retries = int(self.get_parameter("move_max_retries").value)
        self.move_retry_sleep_s = float(self.get_parameter("move_retry_sleep_s").value)
        self.cartesian_max_step = float(self.get_parameter("cartesian_max_step").value)
        self.cartesian_fraction_threshold = float(self.get_parameter("cartesian_fraction_threshold").value)
        self.force_rate_hz  = float(self.get_parameter("force_rate_hz").value)
        self.skip_if_no_defects = bool(self.get_parameter("skip_if_no_defects").value)

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
        self.moveit2.max_velocity = 0.05
        self.moveit2.max_acceleration = 0.05
        self.moveit2.cartesian_avoid_collisions = True
        self.moveit2.cartesian_jump_threshold = 2.0

        # ---------------- Publishers ----------------
        self.force_pub       = self.create_publisher(WrenchStamped, "/harmony/cleaning/force", 10)
        self.robot_status_pub = self.create_publisher(String, "/harmony/robot_status", 10)

        # ---------------- Subscribers ----------------
        self.cmd_sub    = self.create_subscription(String, "/harmony/cmd_input", self._cmd_cb, 10, callback_group=cbg)
        self.defect_sub = self.create_subscription(String, "/harmony/mock_perception/defect", self._defect_cb, 10, callback_group=cbg)

        # ---------------- State ----------------
        self.confirm_event   = Event()
        self.stop_event      = Event()
        self.cleaning_active = Event()

        self.defect_lock = Lock()
        self.scan_id: Optional[str] = None
        self.defects: List[Dict[str, Any]] = []
        self.last_cleaned_scan_id: Optional[str] = None

        # Force timer
        self.force_timer = self.create_timer(1.0 / self.force_rate_hz, self._publish_force)

        # Worker thread
        self.worker = Thread(target=self._run, daemon=True)
        self.worker.start()

        self._publish_robot_status("IDLE", "IDLE", "Real CR node ready (waiting defect + CONFIRM)")
        self.get_logger().info("RealHarmonyCleaningMissionRunnerV3 initialized")

    # ─── Status ──────────────────────────────────────────────────────────────
    def _publish_robot_status(self, state: str, mode: str, note: str, level: str = "INFO"):
        payload = {
            "timestamp": _now_ros(self),
            "state": state, "mode": mode,
            "level": level, "note": note,
            "frame_id": self.fixed_frame,
        }
        msg = String()
        msg.data = json.dumps(payload)
        self.robot_status_pub.publish(msg)

    # ─── Input ───────────────────────────────────────────────────────────────
    def _extract_xyz(self, d: Dict[str, Any]) -> Optional[List[float]]:
        pos = None
        if isinstance(d.get("pos"), dict):
            pos = d["pos"]
        elif isinstance(d.get("position"), dict):
            pos = d["position"]
        elif all(k in d for k in ("x", "y", "z")):
            pos = d
        if not isinstance(pos, dict):
            return None
        try:
            return [float(pos["x"]), float(pos["y"]), float(pos["z"])]
        except Exception:
            return None

    # ─── Callbacks ───────────────────────────────────────────────────────────
    def _defect_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            if not isinstance(data, dict):
                return
        except Exception:
            return
        xyz = self._extract_xyz(data)
        if xyz is None:
            return
        with self.defect_lock:
            self.defects.append({"position": {"x": xyz[0], "y": xyz[1], "z": xyz[2]}, "raw": data})
            n = len(self.defects)
        self._publish_robot_status("WAITING", "CR", f"Defect received: {n} total. Waiting CONFIRM")
        self.get_logger().info(f"Defect collected: {n} total")

    def _cmd_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            cmd  = str(data.get("cmd", "")).upper().strip()
        except Exception:
            return

        if cmd == "CONFIRM" and self.cleaning_active.is_set():
            return   # Cleaning sırasında CONFIRM'i yoksay

        if cmd == "START":
            with self.defect_lock:
                self.defects.clear()
                self.scan_id = None
            self.get_logger().info("START received — cleared defects")
        elif cmd == "CONFIRM":
            self.confirm_event.set()
        elif cmd == "WAITING":
            self._publish_robot_status("WAITING", "CR", "WAITING command received")
        elif cmd == "STOP":
            self.stop_event.set()
            self.confirm_event.clear()
            self.cleaning_active.clear()
            self._publish_robot_status("IDLE", "IDLE", "STOP received", level="WARN")

    # ─── Force publisher ─────────────────────────────────────────────────────
    def _publish_force(self):
        if not self.cleaning_active.is_set():
            return
        self._publish_robot_status("CR_MODE", "CR", "Cleaning in progress")
        fz = random.uniform(5.0, 20.0)
        fmsg = WrenchStamped()
        fmsg.header.stamp    = self.get_clock().now().to_msg()
        fmsg.header.frame_id = self.tool_frame
        fmsg.wrench.force.z  = fz
        self.force_pub.publish(fmsg)

    # ─── Motion ──────────────────────────────────────────────────────────────
    def move_to_pose(self, position: List[float], cartesian: bool = False) -> bool:
        quat = [0.0, 0.0, 0.0, 1.0]
        try:
            self.moveit2.move_to_pose(
                position=[float(position[0]), float(position[1]), float(position[2])],
                quat_xyzw=quat,
                cartesian=cartesian,
                cartesian_max_step=self.cartesian_max_step,
                cartesian_fraction_threshold=self.cartesian_fraction_threshold,
            )
            return bool(self.moveit2.wait_until_executed())
        except Exception:
            return False

    def move_to_joint_config(self, joints: List[float]) -> bool:
        try:
            self.moveit2.move_to_configuration([float(x) for x in joints])
            return bool(self.moveit2.wait_until_executed())
        except Exception:
            return False

    def move_with_retries(self, position: List[float], try_non_cartesian_first: bool = True) -> bool:
        for attempt in range(1, self.move_max_retries + 1):
            if self.stop_event.is_set():
                return False
            ok = False
            if try_non_cartesian_first:
                ok = self.move_to_pose(position, cartesian=False)
                if not ok:
                    ok = self.move_to_pose(position, cartesian=True)
            else:
                ok = self.move_to_pose(position, cartesian=True)
                if not ok:
                    ok = self.move_to_pose(position, cartesian=False)
            if ok:
                return True
            time.sleep(self.move_retry_sleep_s)
        return False

    # ─── Worker ──────────────────────────────────────────────────────────────
    def _run(self):
        home_joints = [0.05, 0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
        time.sleep(0.5)

        while rclpy.ok():
            if self.stop_event.is_set():
                time.sleep(0.1)
                continue

            if not self.confirm_event.wait(timeout=0.2):
                continue
            self.confirm_event.clear()

            with self.defect_lock:
                defects_copy = list(self.defects)
                scan_id      = self.scan_id
                self.defects = []
                self.scan_id = None

            if len(defects_copy) == 0:
                if self.skip_if_no_defects:
                    self._publish_robot_status("COMPLETE", "CR", "No defects to clean")
                    self._publish_robot_status("IDLE", "IDLE", "Back to IDLE")
                continue

            scan_key = str(scan_id) if scan_id is not None else "no_scan_id"
            if self.last_cleaned_scan_id == scan_key:
                self._publish_robot_status("IDLE", "IDLE", "Duplicate CONFIRM/scan ignored")
                continue
            self.last_cleaned_scan_id = scan_key

            self._publish_robot_status("CR_MODE", "CR", f"Cleaning started (scan={scan_key}, n={len(defects_copy)})")
            self.cleaning_active.set()

            # Home
            self.move_to_joint_config(home_joints)
            time.sleep(0.3)

            for d in defects_copy:
                if self.stop_event.is_set():
                    break

                xyz = self._extract_xyz(d if isinstance(d, dict) else {})
                if xyz is None:
                    continue

                x, y, z = xyz
                y2 = y - self.paint_y_offset
                dx = self.paint_x_span

                targets = [
                    [x + dx, y2, z],
                    [x - dx, y2, z],
                ]

                for target in targets:
                    if self.stop_event.is_set():
                        break

                    approach_pos = [target[0] + self.approach_offset, target[1], target[2]]

                    ok = self.move_with_retries(approach_pos)
                    if not ok:
                        continue

                    ok = self.move_with_retries(target)
                    if not ok:
                        continue

                    # Dwell
                    t0 = time.time()
                    while time.time() - t0 < self.cleaning_duration:
                        if self.stop_event.is_set():
                            break
                        time.sleep(0.01)

                    # Retract
                    self.move_to_pose(approach_pos, cartesian=False)

            self.cleaning_active.clear()

            if self.stop_event.is_set():
                self._publish_robot_status("IDLE", "IDLE", "Cleaning stopped", level="WARN")
                self.stop_event.clear()
                continue

            self.move_to_joint_config(home_joints)
            self._publish_robot_status("COMPLETE", "CR", "Cleaning completed")
            time.sleep(2.0)
            self._publish_robot_status("IDLE", "IDLE", "Back to IDLE")


def main():
    rclpy.init()
    node = RealHarmonyCleaningMissionRunnerV3()
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
