#!/usr/bin/env python3
"""
SR (Sensing/Scanning) Node - v2
- START -> 1 tur tarama tamamlanır (defect olsa bile DURMAZ)
- SR_MODE sırasında /harmony/mock_perception/defect (tekil JSON) mesajlarını toplar
- Tur bitince /harmony/mock_perception/defect_list (JSON array) yayınlar
- /harmony/robot_status, /harmony/tcp_pose, /harmony/path_plan, /harmony/log yayınlar
"""

from __future__ import annotations

from threading import Thread, Event, Lock
import time
import json
import math
from typing import Any, Dict, List, Optional, Tuple

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String, Int32
from geometry_msgs.msg import PoseStamped, Pose
from nav_msgs.msg import Path

import tf2_ros
from tf2_ros import TransformException
from rclpy.time import Time

from pymoveit2_sim import MoveIt2 as MoveIt2_Sim
from pymoveit2_sim.robots import ur as robot


def _now_ros(node: Node) -> str:
    t = node.get_clock().now().to_msg()
    return f"{t.sec}.{t.nanosec:09d}"


class HarmonySensingRobotV2(Node):
    def __init__(self):
        super().__init__("harmony_sensing_robot")

        # ---------------- Params ----------------
        self.declare_parameter("fixed_frame", "world")
        self.declare_parameter("tcp_frame", "")  # empty => use robot.end_effector_name()
        self.declare_parameter("tcp_pose_rate_hz", 30.0)
        self.declare_parameter("scan_wait_time", 1.5)
        self.declare_parameter("max_scan_cycles", 1)  # 1 tur istiyorsun; istersen paramdan artırırsın
        self.declare_parameter("publish_path_samples", 25)

        # defect list collection
        self.declare_parameter("defect_duplicate_distance", 0.15)

        self.fixed_frame = str(self.get_parameter("fixed_frame").value)
        _tcp_param = str(self.get_parameter("tcp_frame").value)
        self.tcp_frame = _tcp_param if _tcp_param else robot.end_effector_name()
        self.tcp_pose_rate_hz = float(self.get_parameter("tcp_pose_rate_hz").value)
        self.scan_wait_time = float(self.get_parameter("scan_wait_time").value)
        self.max_scan_cycles = int(self.get_parameter("max_scan_cycles").value)
        self.publish_path_samples = int(self.get_parameter("publish_path_samples").value)
        self.dup_dist = float(self.get_parameter("defect_duplicate_distance").value)

        # ---------------- MoveIt2 ----------------
        cbg = ReentrantCallbackGroup()
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
        self.moveit2.max_acceleration = 0.5
        self.moveit2.cartesian_avoid_collisions = True
        self.moveit2.cartesian_jump_threshold = 2.0

        # ---------------- TF2 (fallback for TCP pose) ----------------
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self._last_tcp_pose: Optional[Pose] = None
        self._tcp_warn_throttle_sec = 5.0
        self._last_tcp_warn_time = 0.0

        # ---------------- Publishers ----------------
        self.robot_status_pub = self.create_publisher(String, "/harmony/robot_status", 10)
        self.log_pub = self.create_publisher(String, "/harmony/log", 50)

        self.tcp_pose_pub = self.create_publisher(PoseStamped, "/harmony/tcp_pose", 10)
        self.path_plan_pub = self.create_publisher(Path, "/harmony/path_plan", 10)

        # (mevcut entegrasyonlar için)
        self.waypoint_pub = self.create_publisher(Int32, "/sensing_robot/current_waypoint", 10)

        # defect list output
        self.defect_list_pub = self.create_publisher(String, "/harmony/mock_perception/defect_list", 10)

        # ---------------- Subscribers ----------------
        self.cmd_sub = self.create_subscription(String, "/harmony/cmd_input", self._cmd_cb, 10)
        self.defect_sub = self.create_subscription(String, "/harmony/mock_perception/defect", self._defect_cb, 10)

        # ---------------- State ----------------
        self._mode_lock = Lock()
        self.state: str = "IDLE"      # IDLE | SR_MODE | WAITING | CR_MODE | COMPLETE
        self.mode: str = "IDLE"       # IDLE | SR | CR

        self.active_scan = Event()
        self.stop_requested = Event()
        self.reinspect_requested = Event()

        # defect collection
        self.defects_lock = Lock()
        self.defects: List[Dict[str, Any]] = []
        self.scan_id: Optional[str] = None

        # TCP pose timer
        self.tcp_timer = self.create_timer(1.0 / self.tcp_pose_rate_hz, self._publish_tcp_pose)

        # Worker thread
        self.worker = Thread(target=self._run_state_machine, daemon=True)
        self.worker.start()

        self._publish_robot_status("IDLE", "IDLE", "Waiting for START")
        self._log("INFO", "NODE_START", "HarmonySensingRobotV2 initialized", {"fixed_frame": self.fixed_frame})

    # ---------------- Logging ----------------
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

        # ROS console
        if level == "ERROR":
            self.get_logger().error(f"{event}: {detail} | {ctx}")
        elif level == "WARN":
            self.get_logger().warning(f"{event}: {detail} | {ctx}")
        else:
            self.get_logger().info(f"{event}: {detail} | {ctx}")

        # topic log
        msg = String()
        msg.data = json.dumps(payload)
        self.log_pub.publish(msg)

    # ---------------- Robot status ----------------
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

    def _set_state(self, state: str, mode: str, note: str, level: str = "INFO"):
        with self._mode_lock:
            self.state = state
            self.mode = mode
        self._publish_robot_status(state, mode, note, level=level)
        self._log("INFO" if level == "INFO" else level, "STATE_CHANGE", note, {"state": state, "mode": mode})

    # ---------------- Command handling ----------------
    def _cmd_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            cmd = str(data.get("cmd", "")).upper().strip()
        except Exception:
            self._log("WARN", "CMD_INVALID", "Invalid JSON on /harmony/cmd_input", {"raw": msg.data[:200]})
            return

        self._log("INFO", "CMD_RX", "Command received", {"cmd": cmd})

        if cmd == "START":
            self.stop_requested.clear()
            self.reinspect_requested.clear()

            # yeni scan
            with self.defects_lock:
                self.defects.clear()
            self.scan_id = f"scan_{int(time.time())}"

            self.active_scan.set()
            self._set_state("SR_MODE", "SR", "START received, scanning begins")

        elif cmd == "STOP":
            self.stop_requested.set()
            self.active_scan.clear()
            self._set_state("IDLE", "IDLE", "STOP received, abort to IDLE", level="WARN")

        elif cmd == "REINSPECT":
            # WAITING durumundan tekrar tarama
            self.stop_requested.clear()
            self.reinspect_requested.set()

            with self.defects_lock:
                self.defects.clear()
            self.scan_id = f"scan_{int(time.time())}"

            self.active_scan.set()
            self._set_state("SR_MODE", "SR", "REINSPECT received, scanning again")

        elif cmd == "CONFIRM":
            # cleaning node işleyecek, burada sadece status güncelle
            self._set_state("CR_MODE", "CR", "CONFIRM received (cleaning handled by CR node)")

        else:
            self._log("WARN", "CMD_UNKNOWN", "Unknown command", {"cmd": cmd})

    # ---------------- Defect collection ----------------
    def _defect_cb(self, msg: String):
        # Sadece SR_MODE sırasında topla
        if self.state != "SR_MODE":
            return

        try:
            d = json.loads(msg.data)
            pos = d.get("position", {})
            x = float(pos.get("x"))
            y = float(pos.get("y"))
            z = float(pos.get("z"))
        except Exception:
            self._log("WARN", "DEFECT_INVALID", "Invalid defect JSON", {"raw": msg.data[:200]})
            return

        # Duplicate filtre
        with self.defects_lock:
            for e in self.defects:
                ep = e.get("position", {})
                ex = float(ep.get("x", 0.0))
                ey = float(ep.get("y", 0.0))
                ez = float(ep.get("z", 0.0))
                dist = math.sqrt((x - ex) ** 2 + (y - ey) ** 2 + (z - ez) ** 2)
                if dist < self.dup_dist:
                    return
            self.defects.append(d)
            n = len(self.defects)

        self._log("INFO", "DEFECT_COLLECT", "Defect collected (scan continues)", {"count": n})

    def _publish_defect_list(self):
        with self.defects_lock:
            defects_copy = list(self.defects)

        payload = {
            "timestamp": _now_ros(self),
            "frame_id": self.fixed_frame,
            "scan_id": self.scan_id,
            "count": len(defects_copy),
            "defects": defects_copy,
        }

        out = String()
        out.data = json.dumps(payload)
        self.defect_list_pub.publish(out)

        self._log("INFO", "DEFECT_LIST_PUB", "Published defect list after scan", {
            "scan_id": self.scan_id,
            "count": len(defects_copy),
        })


# ---------------- TCP pose ----------------
    def _warn_throttled(self, event: str, detail: str, ctx: Optional[dict] = None):
        """Avoid spamming WARN logs in high-rate timers."""
        now_sec = time.time()
        if now_sec - self._last_tcp_warn_time >= self._tcp_warn_throttle_sec:
            self._last_tcp_warn_time = now_sec
            self._log("WARN", event, detail, ctx or {})

    def _get_tcp_pose(self) -> Optional[Pose]:
        """Best-effort TCP pose getter.
        Priority:
        1) MoveIt2 wrapper (if available)
        2) TF lookup (fixed_frame -> tcp_frame)
        3) last cached pose (if any)
        """
        # 1) MoveIt2 wrapper
        try:
            if hasattr(self.moveit2, "get_current_pose"):
                pose = self.moveit2.get_current_pose()
                if isinstance(pose, Pose):
                    self._last_tcp_pose = pose
                    return pose
        except Exception as e:
            self._warn_throttled("TCP_POSE_MOVEIT_FAIL", "MoveIt2 get_current_pose failed", {"err": str(e)[:200]})

        # 2) TF fallback
        try:
            # Time() => latest available
            tf_msg = self.tf_buffer.lookup_transform(self.fixed_frame, self.tcp_frame, Time())
            pose = Pose()
            pose.position.x = float(tf_msg.transform.translation.x)
            pose.position.y = float(tf_msg.transform.translation.y)
            pose.position.z = float(tf_msg.transform.translation.z)
            pose.orientation = tf_msg.transform.rotation
            self._last_tcp_pose = pose
            return pose
        except TransformException as e:
            self._warn_throttled(
                "TCP_POSE_TF_FAIL",
                "TF lookup failed for TCP pose",
                {"fixed_frame": self.fixed_frame, "tcp_frame": self.tcp_frame, "err": str(e)[:200]},
            )
        except Exception as e:
            self._warn_throttled("TCP_POSE_TF_ERR", "Unexpected TF error", {"err": str(e)[:200]})

        # 3) last cached pose
        return self._last_tcp_pose

    def _publish_tcp_pose(self):
        """Continuously publish /harmony/tcp_pose at tcp_pose_rate_hz.
        If pose source is temporarily unavailable, publishes last cached pose (if any).
        """
        pose = self._get_tcp_pose()
        if pose is None:
            return

        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = self.fixed_frame
        ps.pose = pose
        self.tcp_pose_pub.publish(ps)

    def _pose_to_xyz(self, pose: Optional[Pose]) -> Optional[Tuple[float, float, float]]:
        if pose is None:
            return None
        return (float(pose.position.x), float(pose.position.y), float(pose.position.z))

        # ---------------- Path plan (ghosting) ----------------
    def _publish_path_plan_linear(self, start_xyz: Tuple[float, float, float], goal_xyz: Tuple[float, float, float]):
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self.fixed_frame
        sx, sy, sz = start_xyz
        gx, gy, gz = goal_xyz
        n = max(2, self.publish_path_samples)
        for i in range(n):
            a = i / float(n - 1)
            p = PoseStamped()
            p.header = path.header
            p.pose = Pose()
            p.pose.position.x = sx + a * (gx - sx)
            p.pose.position.y = sy + a * (gy - sy)
            p.pose.position.z = sz + a * (gz - sz)
            p.pose.orientation.w = 1.0
            path.poses.append(p)
        self.path_plan_pub.publish(path)
        self._log("INFO", "PATH_PLAN_PUB", "Published linear ghosting path", {"samples": n})

    # ---------------- Motion helpers ----------------
    def move_to_joint_angles(self, joint_positions: List[float], synchronous: bool = True) -> bool:
        try:
            self.moveit2.move_to_configuration(joint_positions)
            if synchronous:
                return bool(self.moveit2.wait_until_executed())
            return True
        except Exception as e:
            self._log("WARN", "MOVE_JOINT_FAIL", "move_to_joint_angles failed", {"err": str(e)[:200]})
            return False

    def safe_joint_sequence(self, list_of_joint_angles: List[List[float]], wait_time: float, max_retries: int = 3) -> bool:
        for i, joint_angles in enumerate(list_of_joint_angles):
            if self.stop_requested.is_set():
                self._log("WARN", "ABORT", "Stop requested during scan sequence", {"waypoint": i})
                return False

            # waypoint publish
            w = Int32()
            w.data = i
            self.waypoint_pub.publish(w)

            self._publish_robot_status("SR_MODE", "SR", f"MOVING to waypoint={i}")
            self._log("INFO", "WAYPOINT_MOVE", "Moving to waypoint", {"waypoint": i})

            ok = False
            # Path plan: publish once per new movement (start->end) when the motion is executed
            start_pose = self._get_tcp_pose()
            start_xyz = self._pose_to_xyz(start_pose)

            ok = False
            for attempt in range(max_retries):
                if self.stop_requested.is_set():
                    return False
                ok = self.move_to_joint_angles(joint_angles, synchronous=True)
                if ok:
                    break
                self._log("WARN", "WAYPOINT_RETRY", "Retrying waypoint move", {"waypoint": i, "attempt": attempt + 1})
                time.sleep(0.5)

            if ok and start_xyz is not None:
                end_pose = self._get_tcp_pose()
                end_xyz = self._pose_to_xyz(end_pose)
                if end_xyz is not None:
                    self._publish_path_plan_linear(start_xyz, end_xyz)

            if not ok:
                self._log("ERROR", "WAYPOINT_FAIL", "Failed to reach waypoint", {"waypoint": i})
                return False

            self._publish_robot_status("SR_MODE", "SR", f"SCANNING at waypoint={i}")
            self._log("INFO", "WAYPOINT_SCAN", "Scanning at waypoint", {"waypoint": i})

            if i < len(list_of_joint_angles) - 1:
                time.sleep(wait_time)

        return True

    # ---------------- Worker / State machine ----------------
    def _run_state_machine(self):
        # Senin önceki joint setleri
        home_joints = [1.0, 0.0, -math.pi / 2, 0.0, -math.pi / 2, 0.0, 0.0]
        pose2_joints = [0.2, math.radians(0), math.radians(-90.0), math.radians(0.0), math.radians(-90.0), math.radians(0.0), math.radians(0.0)]
        pose3_joints = [1.0, math.radians(86.40), math.radians(-44.74), math.radians(98.25), math.radians(-233.91), math.radians(-90.0), math.radians(90.0)]
        pose4_joints = [1.0, math.radians(86.39), math.radians(-63.12), math.radians(90.33), math.radians(-212.68), math.radians(-90.0), math.radians(90.0)]
        pose5_joints = [1.0, math.radians(86.37), math.radians(-66.12), math.radians(45.49), math.radians(-164.0), math.radians(-90.0), math.radians(90)]
        pose6_joints = [1.0, math.radians(94.91), math.radians(-111.64), math.radians(-37.60), math.radians(-28.84), math.radians(-90.0), math.radians(90)]
        pose7_joints = [1.0, math.radians(94.84), math.radians(-105.96), math.radians(-90.55), math.radians(-18.31), math.radians(-90.0), math.radians(90)]
        pose8_joints = [1.0, 1.5542166358552454, -2.3765672787497856, -1.8370296687945291, 1.0718900852758073, -1.4855356703907372, math.radians(90)]
        pose9_joints = [1.0, math.radians(109.13), math.radians(-57.99), math.radians(-83.46), math.radians(-40.12), math.radians(-90), math.radians(90)]
        pose10_joints = [1.0, math.radians(108.91), math.radians(-46.56), math.radians(-115.55), math.radians(-19.46), math.radians(-90), math.radians(90)]
        pose11_joints = [1.0, math.radians(54), math.radians(-92), math.radians(-156), math.radians(65), math.radians(-36), math.radians(93)]

        scan_waypoints = [
            pose2_joints,
            pose6_joints,
            pose9_joints,
            pose5_joints,
            pose4_joints,
            pose10_joints,
            pose7_joints,
            pose8_joints,
            pose11_joints,
            pose3_joints,
        ]

        while rclpy.ok():
            # IDLE wait
            if not self.active_scan.wait(timeout=0.2):
                continue

            if self.stop_requested.is_set():
                self.active_scan.clear()
                continue

            self._log("INFO", "SR_START", "Starting SR scan cycle(s)", {"cycles": self.max_scan_cycles, "scan_id": self.scan_id})

            # Home
            self._publish_robot_status("SR_MODE", "SR", "Going HOME")
            self.move_to_joint_angles(home_joints, synchronous=True)
            time.sleep(1.0)

            # Scan cycles
            ok_all = True
            for cycle in range(self.max_scan_cycles):
                if self.stop_requested.is_set():
                    ok_all = False
                    break

                self._log("INFO", "SR_CYCLE", "Scan cycle begin", {"cycle": cycle + 1})
                ok = self.safe_joint_sequence(scan_waypoints, wait_time=self.scan_wait_time, max_retries=3)
                if not ok:
                    ok_all = False
                    self._log("ERROR", "SR_CYCLE_FAIL", "Scan cycle failed", {"cycle": cycle + 1})
                    break

            # Scan completed -> publish defect list
            self._publish_defect_list()

            # WAITING: CONFIRM/REINSPECT bekle
            self._set_state("WAITING", "SR", "Scan complete. Waiting for CONFIRM or REINSPECT")

            while rclpy.ok():
                if self.stop_requested.is_set():
                    break

                # REINSPECT komutu gelirse _cmd_cb SR_MODE'a alıyor ve active_scan setliyor
                if self.reinspect_requested.is_set():
                    self.reinspect_requested.clear()
                    self._log("INFO", "REINSPECT", "Reinspect requested, restarting scan", {})
                    break

                # CONFIRM gelirse state CR_MODE'a döner (cmd_cb), burada beklemeye devam edebiliriz
                time.sleep(0.1)

            if self.stop_requested.is_set():
                self._set_state("IDLE", "IDLE", "Stopped. Back to IDLE", level="WARN")
                self.stop_requested.clear()
                self.active_scan.clear()
                continue

            # SR node açısından döngü sonu
            self._set_state("COMPLETE", "IDLE", "SR complete (cleaning handled by CR node)")
            self.active_scan.clear()
            self._set_state("IDLE", "IDLE", "Back to IDLE, waiting for START")


def main():
    rclpy.init()
    node = HarmonySensingRobotV2()
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