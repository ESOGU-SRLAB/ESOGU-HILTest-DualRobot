#!/usr/bin/env python3
"""
SR (Sensing/Scanning) Node - v3 (SimToReal entegrasyonu)
- START -> 1 tur tarama tamamlanır (defect olsa bile DURMAZ)
- SR_MODE sırasında /harmony/mock_perception/defect (tekil JSON) mesajlarını toplar
- Tur bitince /harmony/mock_perception/defect_list (JSON array) yayınlar
- /harmony/robot_status, /harmony/tcp_pose, /harmony/path_plan yayınlar
- Sim'de planlanan trajectory gerçek robota da eş zamanlı gönderilir
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
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import tf2_ros
from tf2_ros import TransformException
from rclpy.time import Time

from pymoveit2_sim import MoveIt2 as MoveIt2_Sim
from pymoveit2_sim.robots import ur as robot

from pymoveit2_real import MoveIt2 as MoveIt2_Real
from pymoveit2_real.robots import ur as realrobot


def _now_ros(node: Node) -> str:
    t = node.get_clock().now().to_msg()
    return f"{t.sec}.{t.nanosec:09d}"


# ===========================================================================
# SimToReal Bridge
# ===========================================================================
class SimToRealBridge:
    """
    Sim MoveIt2'de plan() ile üretilen trajectory'yi gerçek robota da iletir.

    Parametreler
    ------------
    real_enabled  : False yapılırsa sadece sim çalışır (güvenli test modu)
    wait_real     : True ise sim, real tamamlanmadan bir sonraki harekete geçmez
    real_velocity : Gerçek robot hız skalası (ilk denemelerde düşük tut)
    real_accel    : Gerçek robot ivme skalası
    """

    def __init__(
        self,
        node: Node,
        sim_moveit: MoveIt2_Sim,
        real_enabled: bool = True,
        wait_real: bool = True,
        real_velocity: float = 0.05,
        real_accel: float = 0.05,
        time_scale: float = 13.0,
    ):
        self._node = node
        self._sim = sim_moveit
        self._real_enabled = real_enabled
        self._wait_real = wait_real
        self._time_scale = time_scale          # >1.0 → gerçek robot yavaşlar
        self._real: Optional[MoveIt2_Real] = None

        if real_enabled:
            try:
                cb_group = ReentrantCallbackGroup()
                self._real = MoveIt2_Real(
                    node=node,
                    joint_names=realrobot.joint_names(),
                    base_link_name="world",
                    end_effector_name=realrobot.end_effector_name(),
                    group_name=realrobot.MOVE_GROUP_ARM,
                    callback_group=cb_group,
                )
                self._real.planner_id = "RRTstarkConfigDefault"
                self._real.max_velocity = real_velocity
                self._real.max_acceleration = real_accel
                self._real.cartesian_avoid_collisions = True
                self._real.cartesian_jump_threshold = 2.0
                node.get_logger().info(
                    f"[Bridge] Gerçek robot bağlandı. hız={real_velocity}, ivme={real_accel}"
                )
            except Exception as e:
                node.get_logger().error(f"[Bridge] Gerçek robot başlatılamadı: {e}")
                self._real = None
                self._real_enabled = False

    # ------------------------------------------------------------------
    def move_to_configuration(self, joint_positions: List[float]) -> bool:
        """
        Sim'de planla → sim + real'e eş zamanlı execute et.
        plan() başarısız olursa move_to_configuration fallback'e düşer (sadece sim).
        """
        joints_f = [float(j) for j in joint_positions]

        trajectory = None
        try:
            trajectory = self._sim.plan(joint_positions=joints_f)
        except Exception as e:
            self._node.get_logger().warning(f"[Bridge] Sim plan() hatası: {e}")

        if not trajectory:
            self._node.get_logger().warning(
                "[Bridge] plan() başarısız → fallback: sadece sim move_to_configuration"
            )
            try:
                self._sim.move_to_configuration(joints_f)
                return bool(self._sim.wait_until_executed())
            except Exception:
                return False

        return self._execute_both(trajectory, joint_target=joints_f)

    # ------------------------------------------------------------------
    def move_to_pose(
        self,
        position: List[float],
        quat_xyzw: List[float],
        cartesian: bool = False,
        # cartesian_max_step: float = 0.01,
        cartesian_fraction_threshold: float = 0.40,
    ) -> bool:
        """
        Sim'de planla → sim + real'e eş zamanlı execute et.
        """
        trajectory = None
        try:
            trajectory = self._sim.plan(
                position=position,
                quat_xyzw=quat_xyzw,
                cartesian=cartesian,
                # cartesian_max_step=cartesian_max_step,
                cartesian_fraction_threshold=cartesian_fraction_threshold,
            )
        except Exception as e:
            self._node.get_logger().warning(f"[Bridge] Sim pose plan() hatası: {e}")

        if not trajectory:
            self._node.get_logger().warning(
                "[Bridge] Pose plan() başarısız → fallback: sadece sim move_to_pose"
            )
            try:
                self._sim.move_to_pose(
                    position=position,
                    quat_xyzw=quat_xyzw,
                    cartesian=cartesian,
                    # cartesian_max_step=cartesian_max_step,
                    cartesian_fraction_threshold=cartesian_fraction_threshold,
                )
                return bool(self._sim.wait_until_executed())
            except Exception:
                return False

        return self._execute_both(trajectory, joint_target=None)

    # ------------------------------------------------------------------
    def _execute_both(self, sim_trajectory, joint_target: Optional[List[float]]) -> bool:
        """Sim trajectory'yi sim'e, adapt edilmişini real'e eş zamanlı execute et."""
        sim_done = Event()
        real_done = Event()
        sim_ok = [False]
        real_ok = [True]

        def run_sim():
            try:
                self._sim.execute(sim_trajectory)
                sim_ok[0] = bool(self._sim.wait_until_executed())
            except Exception as e:
                self._node.get_logger().warning(f"[Bridge] Sim execute hatası: {e}")
                sim_ok[0] = False
            finally:
                sim_done.set()

        def run_real():
            try:
                real_traj = self._adapt_trajectory_for_real(sim_trajectory)
                if real_traj is None and joint_target is not None:
                    self._node.get_logger().warning(
                        "[Bridge] Adapt başarısız → real için yeniden planlıyor"
                    )
                    real_traj = self._real.plan(joint_positions=joint_target)

                if real_traj:
                    self._real.execute(real_traj)
                    real_ok[0] = bool(self._real.wait_until_executed())
                else:
                    self._node.get_logger().warning("[Bridge] Real trajectory yok, hareket atlandı")
                    real_ok[0] = False
            except Exception as e:
                self._node.get_logger().warning(f"[Bridge] Real execute hatası: {e}")
                real_ok[0] = False
            finally:
                real_done.set()

        t_sim = Thread(target=run_sim, daemon=True)
        t_sim.start()

        t_real = None
        if self._real_enabled and self._real is not None:
            t_real = Thread(target=run_real, daemon=True)
            t_real.start()

        sim_done.wait()

        if t_real is not None and self._wait_real:
            real_done.wait(timeout=60.0)
            if not real_done.is_set():
                self._node.get_logger().warning("[Bridge] Real robot timeout (60s)")

        if not real_ok[0] and self._real_enabled:
            self._node.get_logger().warning("[Bridge] Real hareket başarısız (sim devam ediyor)")

        return sim_ok[0]

    # ------------------------------------------------------------------
    def _adapt_trajectory_for_real(self, sim_trajectory) -> Optional[JointTrajectory]:
        """
        Sim trajectory joint isimlerini real robot isimlerine çevirir.
        sim_ur10e_shoulder_pan_joint  →  ur10e_shoulder_pan_joint
        Suffix eşleştirme kullanır, prefix'leri hardcode etmez.
        """
        try:
            real_joint_names = self._real.joint_names
            sim_names = list(sim_trajectory.joint_names)
            if not sim_names:
                return None

            name_map: Dict[str, str] = {}
            for s_name in sim_names:
                for r_name in real_joint_names:
                    if s_name == r_name:
                        name_map[s_name] = r_name
                        break
                    s_parts = s_name.split("_")
                    r_parts = r_name.split("_")
                    # Son 2 kelime aynıysa eşleştir (ör: shoulder_pan_joint)
                    if len(s_parts) >= 2 and len(r_parts) >= 2 and s_parts[-2:] == r_parts[-2:]:
                        name_map[s_name] = r_name
                        break

            if not name_map:
                self._node.get_logger().warning("[Bridge] Hiçbir joint ismi eşleşmedi")
                return None

            if len(name_map) != len(sim_names):
                self._node.get_logger().warning(
                    f"[Bridge] Kısmi eşleşme: {len(name_map)}/{len(sim_names)}"
                )

            real_traj = JointTrajectory()
            real_traj.joint_names = [name_map.get(n, n) for n in sim_names]

            s = self._time_scale
            for pt in sim_trajectory.points:
                new_pt = JointTrajectoryPoint()
                new_pt.positions = list(pt.positions)

                # Hızları ve ivmeleri time_scale ile orantılı olarak düşür
                new_pt.velocities = (
                    [v / s for v in pt.velocities] if pt.velocities else [0.0] * len(sim_names)
                )
                new_pt.accelerations = (
                    [a / (s * s) for a in pt.accelerations] if pt.accelerations else [0.0] * len(sim_names)
                )

                # time_from_start'ı time_scale ile uzat
                orig_sec = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9
                scaled_sec = orig_sec * s
                new_pt.time_from_start.sec = int(scaled_sec)
                new_pt.time_from_start.nanosec = int((scaled_sec - int(scaled_sec)) * 1e9)

                real_traj.points.append(new_pt)

            return real_traj

        except Exception as e:
            self._node.get_logger().warning(f"[Bridge] Trajectory adapt hatası: {e}")
            return None

    @property
    def real_available(self) -> bool:
        return self._real_enabled and self._real is not None


# ===========================================================================
# Sensing Node
# ===========================================================================
class HarmonySensingRobotV3(Node):
    def __init__(self):
        super().__init__("harmony_sensing_robot")

        # ---------------- Params ----------------
        self.declare_parameter("fixed_frame", "world")
        self.declare_parameter("tcp_frame", "")
        self.declare_parameter("tcp_pose_rate_hz", 30.0)
        self.declare_parameter("scan_wait_time", 1.5)
        self.declare_parameter("max_scan_cycles", 1)
        self.declare_parameter("publish_path_samples", 25)
        self.declare_parameter("confirm_timeout_sec", 30.0)
        self.declare_parameter("waiting_joints", [1.0, 0.0, -math.pi / 2, 0.0, -math.pi / 2, 0.0, 0.0])
        self.declare_parameter("defect_duplicate_distance", 0.15)

        # Bridge parametreleri
        self.declare_parameter("real_robot_enabled", True)
        self.declare_parameter("real_robot_velocity", 0.05)
        self.declare_parameter("real_robot_acceleration", 0.05)
        self.declare_parameter("bridge_wait_real", True)
        self.declare_parameter("bridge_time_scale", 13.0)   # >1.0 → real robot yavaşlar

        self.fixed_frame = str(self.get_parameter("fixed_frame").value)
        _tcp_param = str(self.get_parameter("tcp_frame").value)
        self.tcp_frame = _tcp_param if _tcp_param else robot.end_effector_name()
        self.tcp_pose_rate_hz = float(self.get_parameter("tcp_pose_rate_hz").value)
        self.scan_wait_time = float(self.get_parameter("scan_wait_time").value)
        self.max_scan_cycles = int(self.get_parameter("max_scan_cycles").value)
        self.publish_path_samples = int(self.get_parameter("publish_path_samples").value)
        self.confirm_timeout_sec = float(self.get_parameter("confirm_timeout_sec").value)
        self.waiting_joints = [float(x) for x in self.get_parameter("waiting_joints").value]
        self.dup_dist = float(self.get_parameter("defect_duplicate_distance").value)

        real_enabled = bool(self.get_parameter("real_robot_enabled").value)
        real_velocity = float(self.get_parameter("real_robot_velocity").value)
        real_accel = float(self.get_parameter("real_robot_acceleration").value)
        bridge_wait_real = bool(self.get_parameter("bridge_wait_real").value)
        bridge_time_scale = float(self.get_parameter("bridge_time_scale").value)

        # ---------------- MoveIt2 (Sim) ----------------
        cbg = ReentrantCallbackGroup()
        self.moveit2 = MoveIt2_Sim(
            node=self,
            joint_names=robot.joint_names(),
            base_link_name="world",
            end_effector_name=robot.end_effector_name(),
            group_name=robot.MOVE_GROUP_ARM,
            callback_group=cbg,
        )
        self.moveit2.planner_id = "ESTkConfigDefault"
        self.moveit2.max_velocity = 1.0
        self.moveit2.max_acceleration = 1.0
        self.moveit2.cartesian_avoid_collisions = True
        self.moveit2.cartesian_jump_threshold = 2.0

        # ---------------- SimToReal Bridge ----------------
        self.bridge = SimToRealBridge(
            node=self,
            sim_moveit=self.moveit2,
            real_enabled=real_enabled,
            wait_real=bridge_wait_real,
            real_velocity=real_velocity,
            real_accel=real_accel,
            time_scale=bridge_time_scale,
        )

        # ---------------- TF2 ----------------
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self._last_tcp_pose: Optional[Pose] = None
        self._tcp_warn_throttle_sec = 5.0
        self._last_tcp_warn_time = 0.0

        # ---------------- Publishers ----------------
        self.robot_status_pub = self.create_publisher(String, "/harmony/robot_status", 10)
        self.tcp_pose_pub = self.create_publisher(PoseStamped, "/harmony/tcp_pose", 10)
        self.path_plan_pub = self.create_publisher(Path, "/harmony/path_plan", 10)

        # ---------------- Subscribers ----------------
        self.cmd_sub = self.create_subscription(String, "/harmony/cmd_input", self._cmd_cb, 10)
        self.defect_sub = self.create_subscription(String, "/harmony/mock_perception/defect", self._defect_cb, 10)

        # ---------------- State ----------------
        self._mode_lock = Lock()
        self.state: str = "IDLE"
        self.mode: str = "IDLE"

        self.active_scan = Event()
        self.stop_requested = Event()
        self.reinspect_requested = Event()
        self.early_waiting_requested = Event()

        self.tcp_timer = self.create_timer(1.0 / self.tcp_pose_rate_hz, self._publish_tcp_pose)

        self.worker = Thread(target=self._run_state_machine, daemon=True)
        self.worker.start()

        self._publish_robot_status("IDLE", "IDLE", "Waiting for START")
        self.get_logger().info(
            f"HarmonySensingRobotV3 initialized | "
            f"fixed_frame={self.fixed_frame} | "
            f"real_robot={'ENABLED' if self.bridge.real_available else 'DISABLED'}"
        )

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
        self.get_logger().info(f"State change: {state}/{mode} - {note}")

    # ---------------- Command handling ----------------
    def _cmd_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            cmd = str(data.get("cmd", "")).upper().strip()
        except Exception:
            self.get_logger().warning(f"Invalid JSON on /harmony/cmd_input: {msg.data[:200]}")
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
            self._set_state("IDLE", "IDLE", "STOP received, abort to IDLE", level="WARN")

        elif cmd == "REINSPECT":
            self.stop_requested.clear()
            self.reinspect_requested.set()
            self.early_waiting_requested.clear()
            self.active_scan.set()
            self._set_state("SR_MODE", "SR", "REINSPECT received, scanning again")

        elif cmd == "CONFIRM":
            self._set_state("CR_MODE", "CR", "CONFIRM received (cleaning handled by CR node)")

        elif cmd == "WAITING":
            self._set_state("WAITING", "SR", "WAITING command received, staying in WAITING")

        else:
            self.get_logger().warning(f"Unknown command: {cmd}")

    # ---------------- Defect detection callback ----------------
    def _defect_cb(self, msg: String):
        if self.state != "SR_MODE":
            return
        if not self.early_waiting_requested.is_set():
            self.early_waiting_requested.set()
            self.get_logger().info("Defect detected! Stopping scan and going to WAITING state")

    # ---------------- TCP pose ----------------
    def _warn_throttled(self, event: str, detail: str, ctx: Optional[dict] = None):
        now_sec = time.time()
        if now_sec - self._last_tcp_warn_time >= self._tcp_warn_throttle_sec:
            self._last_tcp_warn_time = now_sec

    def _get_tcp_pose(self) -> Optional[Pose]:
        try:
            if hasattr(self.moveit2, "get_current_pose"):
                pose = self.moveit2.get_current_pose()
                if isinstance(pose, Pose):
                    self._last_tcp_pose = pose
                    return pose
        except Exception as e:
            self._warn_throttled("TCP_POSE_MOVEIT_FAIL", str(e))

        try:
            tf_msg = self.tf_buffer.lookup_transform(self.fixed_frame, self.tcp_frame, Time())
            pose = Pose()
            pose.position.x = float(tf_msg.transform.translation.x)
            pose.position.y = float(tf_msg.transform.translation.y)
            pose.position.z = float(tf_msg.transform.translation.z)
            pose.orientation = tf_msg.transform.rotation
            self._last_tcp_pose = pose
            return pose
        except TransformException as e:
            self._warn_throttled("TCP_POSE_TF_FAIL", str(e))
        except Exception as e:
            self._warn_throttled("TCP_POSE_TF_ERR", str(e))

        return self._last_tcp_pose

    def _publish_tcp_pose(self):
        if self.state == "SR_MODE":
            self._publish_robot_status("SR_MODE", "SR", "Scanning in progress")

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

    def _publish_path_plan_linear(
        self,
        start_xyz: Tuple[float, float, float],
        goal_xyz: Tuple[float, float, float],
    ):
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

    # ---------------- Motion helpers ----------------
    def move_to_joint_angles(self, joint_positions: List[float], synchronous: bool = True) -> bool:
        """Bridge üzerinden sim planla + real'e de gönder."""
        try:
            return self.bridge.move_to_configuration(joint_positions)
        except Exception as e:
            self.get_logger().warning(f"[SR] move_to_joint_angles hatası: {e}")
            return False

    def safe_joint_sequence(
        self,
        list_of_joint_angles: List[List[float]],
        wait_time: float,
        max_retries: int = 3,
    ) -> bool:
        for i, joint_angles in enumerate(list_of_joint_angles):
            if self.stop_requested.is_set():
                return False

            if self.early_waiting_requested.is_set():
                return True

            self._publish_robot_status("SR_MODE", "SR", f"MOVING to waypoint={i}")

            start_pose = self._get_tcp_pose()
            start_xyz = self._pose_to_xyz(start_pose)

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
                end_xyz = self._pose_to_xyz(end_pose)
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

    # ---------------- Worker / State machine ----------------
    def _run_state_machine(self):
        home_joints = [1.0, 0.0, -math.pi / 2, 0.0, -math.pi / 2, 0.0, 0.0]
        pose2_joints  = [1.8, math.radians(0),      math.radians(-90.0),  math.radians(0.0),    math.radians(-90.0),  math.radians(0.0),   math.radians(0.0)]
        pose3_joints  = [1.0, math.radians(86.40),  math.radians(-44.74), math.radians(98.25),  math.radians(-233.91),math.radians(-90.0), math.radians(90.0)]
        pose4_joints  = [1.0, math.radians(86.39),  math.radians(-63.12), math.radians(90.33),  math.radians(-212.68),math.radians(-90.0), math.radians(90.0)]
        pose5_joints  = [1.0, math.radians(86.37),  math.radians(-66.12), math.radians(45.49),  math.radians(-164.0), math.radians(-90.0), math.radians(90)]
        pose6_joints  = [1.0, math.radians(94.91),  math.radians(-111.64),math.radians(-37.60), math.radians(-28.84), math.radians(-90.0), math.radians(90)]
        pose7_joints  = [1.0, math.radians(94.84),  math.radians(-105.96),math.radians(-90.55), math.radians(-18.31), math.radians(-90.0), math.radians(90)]
        pose8_joints  = [1.0, 1.5542166358552454,  -2.3765672787497856,  -1.8370296687945291,   1.0718900852758073,  -1.4855356703907372,  math.radians(90)]
        pose9_joints  = [1.0, math.radians(109.13), math.radians(-57.99), math.radians(-83.46), math.radians(-40.12), math.radians(-90),    math.radians(90)]
        pose10_joints = [1.0, math.radians(108.91), math.radians(-46.56), math.radians(-115.55),math.radians(-19.46), math.radians(-90),    math.radians(90)]
        pose11_joints = [1.0, math.radians(54),     math.radians(-92),    math.radians(-156),   math.radians(65),     math.radians(-36),    math.radians(93)]

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

            self._publish_robot_status("SR_MODE", "SR", "Going HOME")
            self.move_to_joint_angles(home_joints, synchronous=True)
            time.sleep(1.0)

            for cycle in range(self.max_scan_cycles):
                if self.stop_requested.is_set():
                    break
                ok = self.safe_joint_sequence(scan_waypoints, wait_time=self.scan_wait_time, max_retries=3)
                if not ok:
                    break

            if self.early_waiting_requested.is_set():
                self._publish_robot_status("SR_MODE", "SR", "Red defect detected. Going to WAITING position")
                self.move_to_joint_angles(self.waiting_joints, synchronous=True)

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
                    self._set_state("IDLE", "IDLE", "No CONFIRM received within timeout. Back to IDLE", level="WARN")
                    self.active_scan.clear()
                    break
                time.sleep(0.1)

            if self.stop_requested.is_set():
                self._set_state("IDLE", "IDLE", "Stopped. Back to IDLE", level="WARN")
                self.stop_requested.clear()
                self.active_scan.clear()
                self.early_waiting_requested.clear()
                continue

            self._set_state("COMPLETE", "IDLE", "SR complete (cleaning handled by CR node)")
            self.active_scan.clear()
            self.early_waiting_requested.clear()
            self._set_state("IDLE", "IDLE", "Back to IDLE, waiting for START")


def main():
    rclpy.init()
    node = HarmonySensingRobotV3()
    exec_ = MultiThreadedExecutor(num_threads=6)
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