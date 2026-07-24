#!/usr/bin/env python3
"""
SR (Sensing/Scanning) Node - v5 (yalnızca gerçek robot, HARMONY validasyon)
- START -> 1 tur tarama tamamlanır, ardından HOME pozisyonuna dönülür
- Tarama bitince senaryonun 4 yapay defect'i yayınlanır (red detection yok):
    * /harmony/mock_perception/defect       -> defect başına tekil JSON (birincil)
    * /harmony/mock_perception/defect_list  -> JSON array (yedek, latched)
    * /harmony/defect_markers               -> RViz MarkerArray
- /harmony/robot_status, /harmony/tcp_pose, /harmony/path_plan yayınlar
- Trajectory doğrudan gerçek robotta (pymoveit2_real) planlanır ve çalıştırılır;
  sim (pymoveit2_sim) tarafı ve SimToReal köprüsü tamamen kaldırılmıştır.
"""

from __future__ import annotations

from datetime import datetime
from threading import Thread, Event, Lock
import time
import json
import math
from typing import List, Optional, Tuple

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Pose
from nav_msgs.msg import Path
from visualization_msgs.msg import MarkerArray

import tf2_ros
from tf2_ros import TransformException
from rclpy.time import Time

from pymoveit2_real import MoveIt2 as MoveIt2_Real
from pymoveit2_real.robots import ur as realrobot
from pymoveit2_real import harmony_defects as hd


def _now_ros(node: Node) -> str:
    t = node.get_clock().now().to_msg()
    return f"{t.sec}.{t.nanosec:09d}"


def _latched_qos(depth: int = 1) -> QoSProfile:
    """Geç bağlanan abonelerin (arayüz, RViz) son mesajı görmesi için."""
    return QoSProfile(
        depth=depth,
        history=QoSHistoryPolicy.KEEP_LAST,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    )


# ===========================================================================
# Sensing Node
# ===========================================================================
class HarmonySensingRobotV5(Node):
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
        self.declare_parameter("waiting_joints", list(hd.HOME_JOINTS))
        self.declare_parameter("home_joints", list(hd.HOME_JOINTS))
        # Defect yayını arasındaki bekleme (arayüzün tek tek işlemesi için).
        self.declare_parameter("defect_publish_interval", 0.3)
        # MarkerArray sürekli yayınlanır: topic hiç susmaz, böylece RViz'de
        # her zaman seçilebilir. Sensing tamamlanana kadar boş dizi gider.
        self.declare_parameter("marker_publish_rate_hz", 1.0)

        # Gerçek robot hareket parametreleri
        self.declare_parameter("planner_id", "ESTkConfigDefault")
        self.declare_parameter("real_robot_velocity", 0.1)
        self.declare_parameter("real_robot_acceleration", 0.1)

        self.fixed_frame = str(self.get_parameter("fixed_frame").value)
        _tcp_param = str(self.get_parameter("tcp_frame").value)
        self.tcp_frame = _tcp_param if _tcp_param else realrobot.end_effector_name()
        self.tcp_pose_rate_hz = float(self.get_parameter("tcp_pose_rate_hz").value)
        self.scan_wait_time = float(self.get_parameter("scan_wait_time").value)
        self.max_scan_cycles = int(self.get_parameter("max_scan_cycles").value)
        self.publish_path_samples = int(self.get_parameter("publish_path_samples").value)
        self.confirm_timeout_sec = float(self.get_parameter("confirm_timeout_sec").value)
        self.waiting_joints = [float(x) for x in self.get_parameter("waiting_joints").value]
        self.home_joints = [float(x) for x in self.get_parameter("home_joints").value]
        self.defect_publish_interval = float(self.get_parameter("defect_publish_interval").value)
        self.marker_publish_rate_hz = float(self.get_parameter("marker_publish_rate_hz").value)

        planner_id = str(self.get_parameter("planner_id").value)
        real_velocity = float(self.get_parameter("real_robot_velocity").value)
        real_accel = float(self.get_parameter("real_robot_acceleration").value)

        # ---------------- MoveIt2 (Real) ----------------
        # MoveIt2'ye AYRI bir node veriyoruz. pymoveit2'nin plan() /
        # wait_until_executed() metodları içeride rclpy.spin_once(node) çağırır;
        # bu çağrı node'u rclpy'nin global executor'ına ekleyip `node.executor`'ı
        # ona çevirir ve çıkışta geri vermez. Sonuç: o node'un ABONELİKLERİ ilk
        # planlamadan sonra sessizce mesaj almaz olur (timer'lar çalışmaya devam
        # ettiği için fark edilmesi zordur). Bu node'a MoveIt2'yi bağlarsak
        # /harmony/cmd_input (STOP dahil) ve /harmony/defect_status abonelikleri
        # tarama başladıktan sonra ölür.
        self.moveit_node = Node("harmony_sensing_robot_moveit")
        cbg = ReentrantCallbackGroup()
        self.moveit2 = MoveIt2_Real(
            node=self.moveit_node,
            joint_names=realrobot.joint_names(),
            base_link_name="world",
            end_effector_name=realrobot.end_effector_name(),
            group_name=realrobot.MOVE_GROUP_ARM,
            callback_group=cbg,
        )
        self.moveit2.planner_id = planner_id
        self.moveit2.max_velocity = real_velocity
        self.moveit2.max_acceleration = real_accel
        self.moveit2.cartesian_avoid_collisions = True
        self.moveit2.cartesian_jump_threshold = 2.0

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

        # Defect yayını: tekil topic birincil, liste + marker'lar latched.
        self.defect_pub = self.create_publisher(String, hd.TOPIC_DEFECT, 10)
        self.defect_list_pub = self.create_publisher(String, hd.TOPIC_DEFECT_LIST, _latched_qos())
        self.marker_pub = self.create_publisher(MarkerArray, hd.TOPIC_DEFECT_MARKERS, _latched_qos())

        # ---------------- Subscribers ----------------
        self.cmd_sub = self.create_subscription(String, "/harmony/cmd_input", self._cmd_cb, 10)
        # Marker renklerini cleaning node'unun durum güncellemeleriyle senkron tut.
        # Marker topic'inin tek yayıncısı bu node'dur; cleaning yalnızca durum yayınlar.
        self.defect_status_sub = self.create_subscription(
            String, hd.TOPIC_DEFECT_STATUS, self._defect_status_cb, 10
        )

        # ---------------- State ----------------
        self._mode_lock = Lock()
        self.state: str = "IDLE"
        self.mode: str = "IDLE"

        self.active_scan = Event()
        self.stop_requested = Event()
        self.reinspect_requested = Event()

        # Defect görselleştirme durumu
        self._marker_lock = Lock()
        self._defects_reported = False
        self._defect_status = {}

        self.tcp_timer = self.create_timer(1.0 / self.tcp_pose_rate_hz, self._publish_tcp_pose)
        if self.marker_publish_rate_hz > 0:
            self.marker_timer = self.create_timer(
                1.0 / self.marker_publish_rate_hz, self._publish_markers
            )

        self.worker = Thread(target=self._run_state_machine, daemon=True)
        self.worker.start()

        self._publish_robot_status("IDLE", "IDLE", "Waiting for START")
        self.get_logger().info(
            f"HarmonySensingRobotV5 (REAL) initialized | "
            f"fixed_frame={self.fixed_frame} | group={realrobot.MOVE_GROUP_ARM} | "
            f"planner={planner_id} | vel={real_velocity} acc={real_accel} | "
            f"defects={len(hd.DEFECTS)}"
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
            self._reset_defects()
            self.active_scan.set()
            self._set_state("SR_MODE", "SR", "START received, scanning begins")

        elif cmd == "STOP":
            self.stop_requested.set()
            self.active_scan.clear()
            self._set_state("IDLE", "IDLE", "STOP received, abort to IDLE", level="WARN")

        elif cmd == "REINSPECT":
            self.stop_requested.clear()
            self.reinspect_requested.set()
            self._reset_defects()
            self.active_scan.set()
            self._set_state("SR_MODE", "SR", "REINSPECT received, scanning again")

        elif cmd == "CONFIRM":
            self._set_state("CR_MODE", "CR", "CONFIRM received (cleaning handled by CR node)")

        elif cmd == "WAITING":
            self._set_state("WAITING", "SR", "WAITING command received, staying in WAITING")

        else:
            self.get_logger().warning(f"Unknown command: {cmd}")

    # ---------------- Defect görselleştirme ----------------
    def _publish_markers(self):
        """MarkerArray'i periyodik yayınlar (topic hiç susmaz).

        Sensing tamamlanmadan önce boş dizi (DELETEALL) gider: RViz'de topic
        seçilebilir olur ama defect'ler senaryodan önce görünmez.
        """
        stamp = self.get_clock().now().to_msg()
        with self._marker_lock:
            reported = self._defects_reported
            statuses = dict(self._defect_status)

        if not reported:
            self.marker_pub.publish(
                hd.build_empty_marker_array(frame_id=self.fixed_frame, stamp=stamp)
            )
            return

        self.marker_pub.publish(
            hd.build_marker_array(
                statuses=statuses, frame_id=self.fixed_frame, stamp=stamp
            )
        )

    def _defect_status_cb(self, msg: String):
        """Cleaning node'undan gelen DETECTED/CLEANING/CLEANED güncellemeleri."""
        try:
            data = json.loads(msg.data)
            defect_id = str(data.get("defect_id", "")).strip()
            status = str(data.get("status", "")).strip()
        except Exception:
            return
        if not defect_id or not status:
            return
        with self._marker_lock:
            self._defect_status[defect_id] = status
        self.get_logger().info(f"Marker status update: {defect_id} -> {status}")

    def _reset_defects(self):
        """Yeni tarama başlarken marker'ları temizler."""
        with self._marker_lock:
            self._defects_reported = False
            self._defect_status = {}

    # ---------------- Defect yayını ----------------
    def _publish_defects(self):
        """Tarama bitince senaryonun 4 yapay defect'ini yayınlar.

        Kaynak: pymoveit2_real.harmony_defects. Gerçek algılama yapılmaz;
        defect'ler kapıda fiziksel olarak var olan ancak CAD modelinde
        bulunmayan hataları temsil eder.
        """
        stamp = self.get_clock().now().to_msg()
        iso = datetime.now().isoformat()
        payloads = hd.all_payloads(frame_id=self.fixed_frame, timestamp=iso)

        for payload in payloads:
            msg = String()
            msg.data = json.dumps(payload)
            self.defect_pub.publish(msg)
            p = payload["position"]
            self.get_logger().info(
                f"Defect published: {payload['defect_id']} ({payload['defect_type']}) "
                f"x={p['x']:.3f} y={p['y']:.3f} z={p['z']:.3f}"
            )
            if self.defect_publish_interval > 0:
                time.sleep(self.defect_publish_interval)

        list_msg = String()
        list_msg.data = json.dumps(payloads)
        self.defect_list_pub.publish(list_msg)

        # Bundan sonra periyodik marker timer'ı dolu diziyi yayınlar.
        with self._marker_lock:
            self._defects_reported = True
            self._defect_status = {p["defect_id"]: p["status"] for p in payloads}
        self._publish_markers()

        self._publish_robot_status(
            "SR_MODE", "SR", f"Sensing complete. {len(payloads)} defects reported"
        )

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
        """Gerçek robotta planla ve çalıştır."""
        joints_f = [float(j) for j in joint_positions]

        trajectory = None
        try:
            trajectory = self.moveit2.plan(joint_positions=joints_f)
        except Exception as e:
            self.get_logger().warning(f"[SR] plan() hatası: {e}")

        if not trajectory:
            self.get_logger().warning(
                "[SR] plan() başarısız → fallback: move_to_configuration"
            )
            try:
                self.moveit2.move_to_configuration(joints_f)
                return bool(self.moveit2.wait_until_executed())
            except Exception as e:
                self.get_logger().warning(f"[SR] move_to_configuration hatası: {e}")
                return False

        try:
            self.moveit2.execute(trajectory)
            return bool(self.moveit2.wait_until_executed())
        except Exception as e:
            self.get_logger().warning(f"[SR] execute hatası: {e}")
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
                    if self.stop_requested.is_set():
                        break
                    time.sleep(0.05)

        return True

    # ---------------- Worker / State machine ----------------
    def _run_state_machine(self):
        home_joints = self.home_joints
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

            # Tarama bitti: önce HOME'a dön, sonra defect'leri bildir.
            if not self.stop_requested.is_set():
                self._publish_robot_status("SR_MODE", "SR", "Scan finished. Returning HOME")
                self.move_to_joint_angles(home_joints, synchronous=True)
                self._publish_defects()

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
                continue

            self._set_state("COMPLETE", "IDLE", "SR complete (cleaning handled by CR node)")
            self.active_scan.clear()
            self._set_state("IDLE", "IDLE", "Back to IDLE, waiting for START")


def main():
    rclpy.init()
    node = HarmonySensingRobotV5()
    exec_ = MultiThreadedExecutor(num_threads=6)
    exec_.add_node(node)
    exec_.add_node(node.moveit_node)
    try:
        exec_.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.moveit_node.destroy_node()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
