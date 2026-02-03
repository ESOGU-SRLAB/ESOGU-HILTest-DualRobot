#!/usr/bin/env python3
"""
CR (Cleaning) Node - v2
- /harmony/mock_perception/defect_list (JSON array) alır, hafızaya kaydeder
- /harmony/cmd_input: CONFIRM gelince listeyi sırayla temizler
- Her defect için: approach -> descend -> dwell(cleaning_duration) -> retract
- /harmony/cleaning/force: cleaning_active iken tool Z'de 5..20N yayınlar
- /harmony/robot_status + /harmony/log üretir
"""

from __future__ import annotations

from threading import Thread, Event, Lock
import time
import json
import random
from typing import Any, Dict, List, Optional, Tuple

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


class HarmonyCleaningRunnerV2(Node):
    def __init__(self):
        super().__init__("harmony_cleaning_runner")

        # Params
        self.declare_parameter("fixed_frame", "world")
        self.declare_parameter("tool_frame", "tool0")

        self.declare_parameter("approach_height_offset", 0.15)
        self.declare_parameter("cleaning_duration", 1.5)
        self.declare_parameter("cleaning_offset_x", 0.30)   # senin eski mantığın
        self.declare_parameter("force_rate_hz", 30.0)

        self.declare_parameter("skip_if_no_defects", True)

        self.fixed_frame = str(self.get_parameter("fixed_frame").value)
        self.tool_frame = str(self.get_parameter("tool_frame").value)
        self.approach_offset = float(self.get_parameter("approach_height_offset").value)
        self.cleaning_duration = float(self.get_parameter("cleaning_duration").value)
        self.cleaning_offset_x = float(self.get_parameter("cleaning_offset_x").value)
        self.force_rate_hz = float(self.get_parameter("force_rate_hz").value)
        self.skip_if_no_defects = bool(self.get_parameter("skip_if_no_defects").value)

        cbg = ReentrantCallbackGroup()

        # MoveIt2
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

        # Publishers
        self.force_pub = self.create_publisher(WrenchStamped, "/harmony/cleaning/force", 10)
        self.robot_status_pub = self.create_publisher(String, "/harmony/robot_status", 10)
        self.log_pub = self.create_publisher(String, "/harmony/log", 50)

        # Subscribers
        self.cmd_sub = self.create_subscription(String, "/harmony/cmd_input", self._cmd_cb, 10, callback_group=cbg)
        self.defect_list_sub = self.create_subscription(String, "/harmony/mock_perception/defect_list", self._defect_list_cb, 10, callback_group=cbg)

        # State
        self.confirm_event = Event()
        self.stop_event = Event()
        self.cleaning_active = Event()

        self.defect_lock = Lock()
        self.scan_id: Optional[str] = None
        self.defects: List[Dict[str, Any]] = []

        # Force timer
        self.force_timer = self.create_timer(1.0 / self.force_rate_hz, self._publish_force)

        # Worker thread
        self.worker = Thread(target=self._run, daemon=True)
        self.worker.start()

        self._publish_robot_status("IDLE", "IDLE", "CR node ready (waiting defect_list + CONFIRM)")
        self._log("INFO", "NODE_START", "HarmonyCleaningRunnerV2 initialized", {"fixed_frame": self.fixed_frame})

    # ---------- Logging ----------
    def _log(self, level: str, event: str, detail: str, ctx: Optional[dict] = None):
        if ctx is None:
            ctx = {}
        payload = {
            "timestamp": _now_ros(self),
            "node": self.get_name(),
            "level": level,
            "event": event,
            "detail": detail,
            "ctx": ctx,
        }

        if level == "ERROR":
            self.get_logger().error(f"{event}: {detail} | {ctx}")
        elif level == "WARN":
            self.get_logger().warning(f"{event}: {detail} | {ctx}")
        else:
            self.get_logger().info(f"{event}: {detail} | {ctx}")

        msg = String()
        msg.data = json.dumps(payload)
        self.log_pub.publish(msg)

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

    # ---------- Callbacks ----------
    def _defect_list_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            scan_id = data.get("scan_id")
            defects = data.get("defects", [])
            if not isinstance(defects, list):
                raise ValueError("defects is not a list")
        except Exception:
            self._log("WARN", "DEFECT_LIST_INVALID", "Invalid defect_list JSON", {"raw": msg.data[:200]})
            return

        with self.defect_lock:
            self.scan_id = str(scan_id) if scan_id is not None else None
            self.defects = defects

        self._publish_robot_status("WAITING", "CR", f"Defect list received: {len(defects)} defect(s). Waiting CONFIRM")
        self._log("INFO", "DEFECT_LIST_RX", "Defect list received", {"scan_id": self.scan_id, "count": len(defects)})

    def _cmd_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            cmd = str(data.get("cmd", "")).upper().strip()
        except Exception:
            self._log("WARN", "CMD_INVALID", "Invalid JSON on /harmony/cmd_input", {"raw": msg.data[:200]})
            return

        self._log("INFO", "CMD_RX", "Command received", {"cmd": cmd})

        if cmd == "CONFIRM":
            self.confirm_event.set()
        elif cmd == "STOP":
            self.stop_event.set()
            self.confirm_event.clear()
            self.cleaning_active.clear()
            self._publish_robot_status("IDLE", "IDLE", "STOP received, abort cleaning", level="WARN")
            self._log("WARN", "ABORT", "STOP received", {})

    # ---------- Force publisher ----------
    def _publish_force(self):
        if not self.cleaning_active.is_set():
            return

        fz = random.uniform(5.0, 20.0)
        msg = WrenchStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.tool_frame
        msg.wrench.force.x = 0.0
        msg.wrench.force.y = 0.0
        msg.wrench.force.z = fz
        msg.wrench.torque.x = 0.0
        msg.wrench.torque.y = 0.0
        msg.wrench.torque.z = 0.0
        self.force_pub.publish(msg)

    # ---------- Motion ----------
    def move_to_pose(self, position: List[float], orientation: Optional[List[float]] = None, cartesian: bool = False) -> bool:
        if orientation is None:
            orientation = [-1.0, 0.0, 0.0, 0.0]
        try:
            self.moveit2.move_to_pose(
                position=position,
                quat_xyzw=orientation,
                cartesian=cartesian,
                cartesian_max_step=0.005,
                cartesian_fraction_threshold=0.95,
            )
            return bool(self.moveit2.wait_until_executed())
        except Exception as e:
            self._log("WARN", "MOVE_POSE_FAIL", "move_to_pose failed", {"err": str(e)[:200]})
            return False

    def move_to_joint_config(self, joints: List[float]) -> bool:
        try:
            self.moveit2.move_to_configuration(joints)
            return bool(self.moveit2.wait_until_executed())
        except Exception as e:
            self._log("WARN", "MOVE_JOINT_FAIL", "move_to_configuration failed", {"err": str(e)[:200]})
            return False

    # ---------- Worker ----------
    def _run(self):
        # senin eski cleaning node'daki home
        home_joints = [0.0, 0.0, -1.57, 1.57, -1.57, -1.57, 0.0]

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

            if len(defects_copy) == 0:
                msg = "CONFIRM received but defect list is empty"
                self._log("WARN", "CONFIRM_EMPTY", msg, {"scan_id": scan_id})
                if self.skip_if_no_defects:
                    self._publish_robot_status("COMPLETE", "CR", "No defects to clean (complete)")
                    self._publish_robot_status("IDLE", "IDLE", "Back to IDLE")
                continue

            # CR start
            self._publish_robot_status("CR_MODE", "CR", f"Cleaning started (scan_id={scan_id}, n={len(defects_copy)})")
            self._log("INFO", "CLEAN_START", "Cleaning started", {"scan_id": scan_id, "n": len(defects_copy)})

            self.cleaning_active.set()

            # Home
            self.move_to_joint_config(home_joints)
            time.sleep(0.5)

            # defectleri sırayla temizle
            for idx, d in enumerate(defects_copy):
                if self.stop_event.is_set():
                    break

                try:
                    pos = d.get("position", {})
                    x = float(pos.get("x")); y = float(pos.get("y")); z = float(pos.get("z"))
                except Exception:
                    self._log("WARN", "DEFECT_SKIP", "Skipping invalid defect entry", {"index": idx})
                    continue

                # hedef hesap (senin mantık: x offset)
                cleaning_pos = [x + self.cleaning_offset_x, y, z]
                approach_pos = [cleaning_pos[0], cleaning_pos[1], cleaning_pos[2] + self.approach_offset]

                self._publish_robot_status("CR_MODE", "CR", f"Cleaning {idx+1}/{len(defects_copy)}")
                self._log("INFO", "CLEAN_TARGET", "Cleaning target", {
                    "i": idx + 1,
                    "n": len(defects_copy),
                    "cleaning_pos": cleaning_pos,
                    "approach_pos": approach_pos
                })

                # approach
                ok = self.move_to_pose(approach_pos, cartesian=False)
                if not ok:
                    self._log("WARN", "APPROACH_FAIL", "Approach failed, skipping target", {"i": idx + 1})
                    continue

                # descend
                ok = self.move_to_pose(cleaning_pos, cartesian=True)
                if not ok:
                    ok = self.move_to_pose(cleaning_pos, cartesian=False)
                if not ok:
                    self._log("WARN", "DESCEND_FAIL", "Descend failed, skipping target", {"i": idx + 1})
                    continue

                # dwell (force timer aktif)
                t0 = time.time()
                while time.time() - t0 < self.cleaning_duration:
                    if self.stop_event.is_set():
                        break
                    time.sleep(0.01)

                # retract
                self.move_to_pose(approach_pos, cartesian=True)

            # finish
            self.cleaning_active.clear()

            if self.stop_event.is_set():
                self._publish_robot_status("IDLE", "IDLE", "Cleaning stopped", level="WARN")
                self._log("WARN", "CLEAN_STOPPED", "STOP during cleaning", {})
                self.stop_event.clear()
                continue

            # go home at end
            self.move_to_joint_config(home_joints)

            self._publish_robot_status("COMPLETE", "CR", "Cleaning completed")
            self._log("INFO", "CLEAN_DONE", "Cleaning completed", {"scan_id": scan_id})
            self._publish_robot_status("IDLE", "IDLE", "Back to IDLE, waiting START")


def main():
    rclpy.init()
    node = HarmonyCleaningRunnerV2()
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
