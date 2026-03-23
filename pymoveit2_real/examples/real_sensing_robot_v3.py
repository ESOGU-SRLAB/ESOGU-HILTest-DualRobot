#!/usr/bin/env python3
"""
Real Sensing Robot v3
- sensing_robot_v3.py'nin gerçek robot versiyonu
- pymoveit2_real kullanır (move_group: real_ur10e, prefix: ur10e_)
- Geri kalan her şey aynı: topic isimler, waypoint mantığı, state machine
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

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Pose
from nav_msgs.msg import Path

import tf2_ros
from tf2_ros import TransformException
from rclpy.time import Time

from pymoveit2_real import MoveIt2 as MoveIt2_Real
from pymoveit2_real.robots import ur as robot


def _now_ros(node: Node) -> str:
    t = node.get_clock().now().to_msg()
    return f"{t.sec}.{t.nanosec:09d}"


class RealHarmonySensingRobotV3(Node):
    def __init__(self):
        super().__init__("harmony_sensing_robot")   # Node adı sim ile aynı (topic'ler ortak)

        # ---------------- Params ----------------
        self.declare_parameter("fixed_frame", "world")
        self.declare_parameter("tcp_frame", "")
        self.declare_parameter("tcp_pose_rate_hz", 30.0)
        self.declare_parameter("scan_wait_time", 1.5)
        self.declare_parameter("max_scan_cycles", 1)
        self.declare_parameter("publish_path_samples", 25)
        self.declare_parameter("confirm_timeout_sec", 30.0)
        self.declare_parameter("waiting_joints", [1.0, 0.0, -math.pi / 2, 0.0, -math.pi / 2, 0.0, 0.0])

        self.fixed_frame         = str(self.get_parameter("fixed_frame").value)
        _tcp_param               = str(self.get_parameter("tcp_frame").value)
        self.tcp_frame           = _tcp_param if _tcp_param else robot.end_effector_name()
        self.tcp_pose_rate_hz    = float(self.get_parameter("tcp_pose_rate_hz").value)
        self.scan_wait_time      = float(self.get_parameter("scan_wait_time").value)
        self.max_scan_cycles     = int(self.get_parameter("max_scan_cycles").value)
        self.publish_path_samples = int(self.get_parameter("publish_path_samples").value)
        self.confirm_timeout_sec = float(self.get_parameter("confirm_timeout_sec").value)
        self.waiting_joints      = [float(x) for x in self.get_parameter("waiting_joints").value]

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

        # ---------------- TF2 ----------------
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self._last_tcp_pose: Optional[Pose] = None
        self._tcp_warn_throttle_sec = 5.0
        self._last_tcp_warn_time    = 0.0

        # ---------------- Publishers ----------------
        self.robot_status_pub = self.create_publisher(String, "/harmony/robot_status", 10)
        self.tcp_pose_pub     = self.create_publisher(PoseStamped, "/harmony/tcp_pose", 10)
        self.path_plan_pub    = self.create_publisher(Path, "/harmony/path_plan", 10)

        # ---------------- Subscribers ----------------
        self.cmd_sub    = self.create_subscription(String, "/harmony/cmd_input", self._cmd_cb, 10)
        self.defect_sub = self.create_subscription(String, "/harmony/mock_perception/defect", self._defect_cb, 10)

        # ---------------- State ----------------
        self._mode_lock  = Lock()
        self.state: str  = "IDLE"
        self.mode: str   = "IDLE"

        self.active_scan              = Event()
        self.stop_requested           = Event()
        self.reinspect_requested      = Event()
        self.early_waiting_requested  = Event()

        # TCP pose timer
        self.tcp_timer = self.create_timer(1.0 / self.tcp_pose_rate_hz, self._publish_tcp_pose)

        # Worker thread
        self.worker = Thread(target=self._run_state_machine, daemon=True)
        self.worker.start()

        self._publish_robot_status("IDLE", "IDLE", "Waiting for START")
        self.get_logger().info(f"RealHarmonySensingRobotV3 initialized, fixed_frame={self.fixed_frame}")

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

    def _set_state(self, state: str, mode: str, note: str, level: str = "INFO"):
        with self._mode_lock:
            self.state = state
            self.mode  = mode
        self._publish_robot_status(state, mode, note, level=level)
        self.get_logger().info(f"State: {state}/{mode} — {note}")

    # ─── Commands ────────────────────────────────────────────────────────────
    def _cmd_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            cmd  = str(data.get("cmd", "")).upper().strip()
        except Exception:
            return
        self.get_logger().info(f"Command received: {cmd}")

        if cmd == "START":
            self.stop_requested.clear()
            self.reinspect_requested.clear()
            self.early_waiting_requested.clear()
            self.active_scan.set()
            self._set_state("SR_MODE", "SR", "START received, scanning begins")

        elif cmd == "STOP":
            self.stop_requested.set()
            self.active_scan.clear()
            self.early_waiting_requested.clear()
            self._set_state("IDLE", "IDLE", "STOP received", level="WARN")

        elif cmd == "REINSPECT":
            self.stop_requested.clear()
            self.reinspect_requested.set()
            self.early_waiting_requested.clear()
            self.active_scan.set()
            self._set_state("SR_MODE", "SR", "REINSPECT received")

        elif cmd == "CONFIRM":
            self._set_state("CR_MODE", "CR", "CONFIRM received (cleaning handled by CR node)")

        elif cmd == "WAITING":
            self._set_state("WAITING", "SR", "WAITING command received")

        else:
            self.get_logger().warning(f"Unknown command: {cmd}")

    def _defect_cb(self, msg: String):
        if self.state != "SR_MODE":
            return
        if not self.early_waiting_requested.is_set():
            self.early_waiting_requested.set()
            self.get_logger().info("Defect detected! Stopping scan → WAITING")

    # ─── TCP pose ────────────────────────────────────────────────────────────
    def _get_tcp_pose(self) -> Optional[Pose]:
        try:
            if hasattr(self.moveit2, "get_current_pose"):
                pose = self.moveit2.get_current_pose()
                if isinstance(pose, Pose):
                    self._last_tcp_pose = pose
                    return pose
        except Exception:
            pass
        try:
            tf_msg = self.tf_buffer.lookup_transform(self.fixed_frame, self.tcp_frame, Time())
            pose = Pose()
            pose.position.x = float(tf_msg.transform.translation.x)
            pose.position.y = float(tf_msg.transform.translation.y)
            pose.position.z = float(tf_msg.transform.translation.z)
            pose.orientation = tf_msg.transform.rotation
            self._last_tcp_pose = pose
            return pose
        except Exception:
            pass
        return self._last_tcp_pose

    def _publish_tcp_pose(self):
        if self.state == "SR_MODE":
            self._publish_robot_status("SR_MODE", "SR", "Scanning in progress")
        pose = self._get_tcp_pose()
        if pose is None:
            return
        ps = PoseStamped()
        ps.header.stamp    = self.get_clock().now().to_msg()
        ps.header.frame_id = self.fixed_frame
        ps.pose            = pose
        self.tcp_pose_pub.publish(ps)

    def _pose_to_xyz(self, pose: Optional[Pose]) -> Optional[Tuple[float, float, float]]:
        if pose is None:
            return None
        return (float(pose.position.x), float(pose.position.y), float(pose.position.z))

    def _publish_path_plan_linear(self, start_xyz, goal_xyz):
        path = Path()
        path.header.stamp    = self.get_clock().now().to_msg()
        path.header.frame_id = self.fixed_frame
        sx, sy, sz = start_xyz
        gx, gy, gz = goal_xyz
        n = max(2, self.publish_path_samples)
        for i in range(n):
            a = i / float(n - 1)
            p = PoseStamped()
            p.header = path.header
            p.pose   = Pose()
            p.pose.position.x = sx + a * (gx - sx)
            p.pose.position.y = sy + a * (gy - sy)
            p.pose.position.z = sz + a * (gz - sz)
            p.pose.orientation.w = 1.0
            path.poses.append(p)
        self.path_plan_pub.publish(path)

    # ─── Motion ──────────────────────────────────────────────────────────────
    def move_to_joint_angles(self, joint_positions: List[float], synchronous: bool = True) -> bool:
        try:
            self.moveit2.move_to_configuration(joint_positions)
            if synchronous:
                return bool(self.moveit2.wait_until_executed())
            return True
        except Exception as e:
            self.get_logger().warning(f"Joint move failed: {e}")
            return False

    def safe_joint_sequence(self, list_of_joint_angles: List[List[float]], wait_time: float, max_retries: int = 3) -> bool:
        for i, joint_angles in enumerate(list_of_joint_angles):
            if self.stop_requested.is_set():
                return False
            if self.early_waiting_requested.is_set():
                return True

            self._publish_robot_status("SR_MODE", "SR", f"MOVING to waypoint={i}")
            start_pose = self._get_tcp_pose()
            start_xyz  = self._pose_to_xyz(start_pose)

            ok = False
            for attempt in range(max_retries):
                if self.stop_requested.is_set():
                    return False
                ok = self.move_to_joint_angles(joint_angles, synchronous=True)
                if ok:
                    break
                time.sleep(0.5)

            if ok and start_xyz is not None:
                end_pose = self._get_tcp_pose()
                end_xyz  = self._pose_to_xyz(end_pose)
                if end_xyz is not None:
                    self._publish_path_plan_linear(start_xyz, end_xyz)

            if not ok:
                return False

            self._publish_robot_status("SR_MODE", "SR", f"SCANNING at waypoint={i}")

            if i < len(list_of_joint_angles) - 1:
                t0 = time.time()
                while time.time() - t0 < wait_time:
                    if self.stop_requested.is_set() or self.early_waiting_requested.is_set():
                        break
                    time.sleep(0.05)

        return True

    # ─── State machine ───────────────────────────────────────────────────────
    def _run_state_machine(self):
        # ── Joint waypoint'leri — sensing_robot_v3.py ile BİREBİR aynı ──
        home_joints   = [1.0, 0.0, -math.pi / 2, 0.0, -math.pi / 2, 0.0, 0.0]
        pose2_joints  = [1.8, math.radians(0),      math.radians(-90.0),  math.radians(0.0),    math.radians(-90.0), math.radians(0.0),   math.radians(0.0)]
        pose3_joints  = [1.0, math.radians(86.40),  math.radians(-44.74), math.radians(98.25),  math.radians(-233.91),math.radians(-90.0),math.radians(90.0)]
        pose4_joints  = [1.0, math.radians(86.39),  math.radians(-63.12), math.radians(90.33),  math.radians(-212.68),math.radians(-90.0),math.radians(90.0)]
        pose5_joints  = [1.0, math.radians(86.37),  math.radians(-66.12), math.radians(45.49),  math.radians(-164.0), math.radians(-90.0),math.radians(90)]
        pose6_joints  = [1.0, math.radians(94.91),  math.radians(-111.64),math.radians(-37.60), math.radians(-28.84), math.radians(-90.0),math.radians(90)]
        pose7_joints  = [1.0, math.radians(94.84),  math.radians(-105.96),math.radians(-90.55), math.radians(-18.31), math.radians(-90.0),math.radians(90)]
        pose8_joints  = [1.0, 1.5542166358552454,   -2.3765672787497856,  -1.8370296687945291,   1.0718900852758073, -1.4855356703907372, math.radians(90)]
        pose9_joints  = [1.0, math.radians(109.13), math.radians(-57.99), math.radians(-83.46),  math.radians(-40.12),math.radians(-90),  math.radians(90)]
        pose10_joints = [1.0, math.radians(108.91), math.radians(-46.56), math.radians(-115.55), math.radians(-19.46),math.radians(-90),  math.radians(90)]
        pose11_joints = [1.0, math.radians(54),     math.radians(-92),    math.radians(-156),    math.radians(65),    math.radians(-36),  math.radians(93)]

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
            if not self.active_scan.wait(timeout=0.2):
                continue
            if self.stop_requested.is_set():
                self.active_scan.clear()
                continue

            # Home
            self._publish_robot_status("SR_MODE", "SR", "Going HOME")
            self.move_to_joint_angles(home_joints, synchronous=True)
            time.sleep(1.0)

            # Scan cycles
            for cycle in range(self.max_scan_cycles):
                if self.stop_requested.is_set():
                    break
                ok = self.safe_joint_sequence(scan_waypoints, wait_time=self.scan_wait_time, max_retries=3)
                if not ok:
                    break

            # Defect tespit edildiyse waiting pozisyonu
            if self.early_waiting_requested.is_set():
                self._publish_robot_status("SR_MODE", "SR", "Red defect detected — going to WAITING position")
                self.move_to_joint_angles(self.waiting_joints, synchronous=True)

            # WAITING: CONFIRM bekle
            self._set_state("WAITING", "SR", "Waiting for CONFIRM (or REINSPECT/STOP)")

            t_wait_start = time.time()
            while rclpy.ok():
                if self.stop_requested.is_set():
                    break
                if self.state == "CR_MODE":
                    break
                if self.reinspect_requested.is_set():
                    self.reinspect_requested.clear()
                    break
                if self.confirm_timeout_sec > 0 and (time.time() - t_wait_start) > self.confirm_timeout_sec:
                    self._set_state("IDLE", "IDLE", "No CONFIRM — back to IDLE", level="WARN")
                    self.active_scan.clear()
                    break
                time.sleep(0.1)

            if self.stop_requested.is_set():
                self._set_state("IDLE", "IDLE", "Stopped", level="WARN")
                self.stop_requested.clear()
                self.active_scan.clear()
                self.early_waiting_requested.clear()
                continue

            self._set_state("COMPLETE", "IDLE", "SR complete")
            self.active_scan.clear()
            self.early_waiting_requested.clear()
            self._set_state("IDLE", "IDLE", "Back to IDLE, waiting for START")


def main():
    rclpy.init()
    node = RealHarmonySensingRobotV3()
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
