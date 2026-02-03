#!/usr/bin/env python3
"""
CR (Cleaning) Node - v3.2 (debounce + scan_id lock + target order)
- defect_list veya single defect alÄ±r
- CONFIRM gelince 3 noktalÄ± geÃ§iÅŸ (sÄ±ra istenen ÅŸekilde):
    (x+dx, y-0.3, z) -> (x, y-0.3, z) -> (x-dx, y-0.3, z)
- Her hedef iÃ§in max 5 retry
- IMPORTANT:
  * Cleaning sÄ±rasÄ±nda gelen CONFIRM ignorlanÄ±r (2 tur sorununu Ã§Ã¶zer)
  * AynÄ± scan_id ikinci kez temizlenmez (safety lock)
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

from pymoveit2_sim import MoveIt2 as MoveIt2_Sim
from pymoveit2_sim.robots import ur as robot


def _now_ros(node: Node) -> str:
    t = node.get_clock().now().to_msg()
    return f"{t.sec}.{t.nanosec:09d}"


class HarmonyCleaningMissionRunnerV3(Node):
    def __init__(self):
        super().__init__("harmony_cleaning_runner")

        # ---------------- Params ----------------
        self.declare_parameter("fixed_frame", "world")
        self.declare_parameter("tool_frame", "tool0")

        # approach offset (senin eski mantÄ±k: x yÃ¶nÃ¼nde +offset)
        self.declare_parameter("approach_height_offset", 0.3)

        self.declare_parameter("cleaning_duration", 1.5)

        # 3-nokta boyama parametreleri
        self.declare_parameter("paint_y_offset", 0.30)   # y - 0.30
        self.declare_parameter("paint_x_span", 0.2)     # x +/- 0.15

        # retry
        self.declare_parameter("move_max_retries", 5)
        self.declare_parameter("move_retry_sleep_s", 0.2)

        # MoveIt cartesian tuning
        self.declare_parameter("cartesian_max_step", 0.01)
        self.declare_parameter("cartesian_fraction_threshold", 0.40)

        self.declare_parameter("force_rate_hz", 30.0)
        self.declare_parameter("skip_if_no_defects", True)

        self.fixed_frame = str(self.get_parameter("fixed_frame").value)
        self.tool_frame = str(self.get_parameter("tool_frame").value)

        self.approach_offset = float(self.get_parameter("approach_height_offset").value)
        self.cleaning_duration = float(self.get_parameter("cleaning_duration").value)

        self.paint_y_offset = float(self.get_parameter("paint_y_offset").value)
        self.paint_x_span = float(self.get_parameter("paint_x_span").value)

        self.move_max_retries = int(self.get_parameter("move_max_retries").value)
        self.move_retry_sleep_s = float(self.get_parameter("move_retry_sleep_s").value)

        self.cartesian_max_step = float(self.get_parameter("cartesian_max_step").value)
        self.cartesian_fraction_threshold = float(self.get_parameter("cartesian_fraction_threshold").value)

        self.force_rate_hz = float(self.get_parameter("force_rate_hz").value)
        self.skip_if_no_defects = bool(self.get_parameter("skip_if_no_defects").value)

        cbg = ReentrantCallbackGroup()

        # ---------------- MoveIt2 ----------------
        self.moveit2 = MoveIt2_Sim(
            node=self,
            joint_names=robot.joint_names(),
            base_link_name="world",
            end_effector_name=robot.end_effector_name(),
            group_name=robot.MOVE_GROUP_ARM,
            callback_group=cbg,
        )
        self.moveit2.planner_id = "RRTConnectkConfigDefault"
        self.moveit2.max_velocity = 0.2
        self.moveit2.max_acceleration = 0.2
        self.moveit2.cartesian_avoid_collisions = True
        self.moveit2.cartesian_jump_threshold = 2.0

        # ---------------- Publishers ----------------
        self.force_pub = self.create_publisher(WrenchStamped, "/harmony/cleaning/force", 10)
        self.robot_status_pub = self.create_publisher(String, "/harmony/robot_status", 10)

        # ---------------- Subscribers ----------------
        self.cmd_sub = self.create_subscription(String, "/harmony/cmd_input", self._cmd_cb, 10, callback_group=cbg)
        self.defect_sub = self.create_subscription(
            String, "/harmony/mock_perception/defect", self._defect_cb, 10, callback_group=cbg
        )

        # ---------------- State ----------------
        self.confirm_event = Event()
        self.stop_event = Event()
        self.cleaning_active = Event()

        self.defect_lock = Lock()
        self.scan_id: Optional[str] = None
        self.defects: List[Dict[str, Any]] = []

        # âœ… aynÄ± scan_id iki kez temizlenmesin diye kilit
        self.last_cleaned_scan_id: Optional[str] = None

        # Force timer
        self.force_timer = self.create_timer(1.0 / self.force_rate_hz, self._publish_force)

        # Worker thread
        self.worker = Thread(target=self._run, daemon=True)
        self.worker.start()

        self._publish_robot_status("IDLE", "IDLE", "CR node ready (waiting defect + CONFIRM)")
        self.get_logger().info("HarmonyCleaningMissionRunnerV3.2 initialized")

    def _publish_robot_status(self, state: str, mode: str, note: str, level: str = "INFO"):
        payload = {
            "timestamp": _now_ros(self),
            "state": state,
            "mode": mode,
            "level": level,
            "note": note,
            "frame_id": self.fixed_frame,
        }
        out = String()
        out.data = json.dumps(payload)
        self.robot_status_pub.publish(out)

    # ---------------- Input parsing ----------------
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

    # ---------------- Callbacks ----------------
    def _defect_cb(self, msg: String):
        """Collect individual defects as they arrive"""
        try:
            data = json.loads(msg.data)
            if not isinstance(data, dict):
                return
        except Exception:
            self.get_logger().warning(f"Invalid JSON on /harmony/mock_perception/defect: {msg.data[:200]}")
            return

        xyz = self._extract_xyz(data)
        if xyz is None:
            self.get_logger().warning(f"No x,y,z in defect payload")
            return

        with self.defect_lock:
            # Add defect to collection
            self.defects.append({"position": {"x": xyz[0], "y": xyz[1], "z": xyz[2]}, "raw": data})
            n = len(self.defects)

        # Transition to WAITING state when first defect arrives
        self._publish_robot_status("WAITING", "CR", f"Defect received: {n} total. Waiting CONFIRM")
        self.get_logger().info(f"Defect collected: {n} total")

    def _cmd_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            cmd = str(data.get("cmd", "")).upper().strip()
        except Exception:
            return

        # âœ… Cleaning sÄ±rasÄ±nda CONFIRM gelirse ignore (2 tur sorununu Ã§Ã¶zer)
        if cmd == "CONFIRM" and self.cleaning_active.is_set():
            return


        if cmd == "START":
            # Clear old defects when new scan starts
            with self.defect_lock:
                self.defects.clear()
                self.scan_id = None
            self.get_logger().info("START received - cleared old defects")
        elif cmd == "CONFIRM":
            self.confirm_event.set()
        elif cmd == "WAITING":
            # Stay in WAITING state - just acknowledge
            self._publish_robot_status("WAITING", "CR", "WAITING command received, staying in WAITING")
        elif cmd == "STOP":
            self.stop_event.set()
            self.confirm_event.clear()
            self.cleaning_active.clear()
            self._publish_robot_status("IDLE", "IDLE", "STOP received, abort cleaning", level="WARN")

    # ---------------- Force publisher ----------------
    def _publish_force(self):
        if not self.cleaning_active.is_set():
            return

        # Publish CR_MODE status continuously during cleaning
        self._publish_robot_status("CR_MODE", "CR", "Cleaning in progress")

        fz = random.uniform(5.0, 20.0)
        msg = WrenchStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.tool_frame
        msg.wrench.force.z = fz
        self.force_pub.publish(msg)

    # ---------------- Motion ----------------
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
        except Exception as e:
            return False

    def move_to_joint_config(self, joints: List[float]) -> bool:
        joints_f = [float(x) for x in joints]
        try:
            self.moveit2.move_to_configuration(joints_f)
            return bool(self.moveit2.wait_until_executed())
        except Exception as e:
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

    # ---------------- Worker ----------------
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
                scan_id = self.scan_id

                # âœ… consume once: CONFIRM ile birlikte eldeki veriyi tÃ¼ket
                self.defects = []
                self.scan_id = None

            if len(defects_copy) == 0:
                if self.skip_if_no_defects:
                    self._publish_robot_status("COMPLETE", "CR", "No defects to clean (complete)")
                    self._publish_robot_status("IDLE", "IDLE", "Back to IDLE")
                continue

            # âœ… aynÄ± scan_id tekrar temizlenmesin
            scan_key = str(scan_id) if scan_id is not None else "no_scan_id"
            if self.last_cleaned_scan_id == scan_key:
                self._publish_robot_status("IDLE", "IDLE", "Duplicate CONFIRM/scan ignored")
                continue
            self.last_cleaned_scan_id = scan_key

            self._publish_robot_status("CR_MODE", "CR", f"Cleaning started (scan_id={scan_key}, n={len(defects_copy)})")

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

                # âœ… Ä°STENEN SIRA:
                # 1) x+dx, 2) x, 3) x-dx
                targets = [
                    [x + dx, y2, z],
                    # [x,      y2, z],
                    [x - dx, y2, z],
                ]


                for t_idx, target in enumerate(targets):
                    if self.stop_event.is_set():
                        break

                    approach_pos = [target[0] + self.approach_offset, target[1], target[2]]

                    ok = self.move_with_retries(approach_pos, try_non_cartesian_first=True)
                    if not ok:
                        continue

                    ok = self.move_with_retries(target, try_non_cartesian_first=True)
                    if not ok:
                        continue

                    # dwell
                    t0 = time.time()
                    while time.time() - t0 < self.cleaning_duration:
                        if self.stop_event.is_set():
                            break
                        time.sleep(0.01)

                    # retract
                    self.move_to_pose(approach_pos, cartesian=False)

            self.cleaning_active.clear()

            if self.stop_event.is_set():
                self._publish_robot_status("IDLE", "IDLE", "Cleaning stopped", level="WARN")
                self.stop_event.clear()
                continue

            self.move_to_joint_config(home_joints)
            self._publish_robot_status("COMPLETE", "CR", "Cleaning completed")
            
            # Wait briefly in COMPLETE state before returning to IDLE
            time.sleep(2.0)
            self._publish_robot_status("IDLE", "IDLE", "Back to IDLE, waiting defect + CONFIRM")


def main():
    rclpy.init()
    node = HarmonyCleaningMissionRunnerV3()
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