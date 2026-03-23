#!/usr/bin/env python3
"""
SR (Sensing/Scanning) Node - v3 (HIL Playback Integrated)
- START -> 1 tur tarama tamamlanır.
- Kayıt Modu (hil_playback_mode=False): Hareketleri hesaplar, JSON'a yazar ve simüle robotu oynatır.
- Playback Modu (hil_playback_mode=True): Hareketleri önceden kaydedilmiş JSON'dan okur, Sim+Real robotlara senkronize yollar.
"""

from __future__ import annotations

import os
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

from pymoveit2_sim import MoveIt2 as MoveIt2_Sim
from pymoveit2_sim.robots import ur as robot
from pymoveit2_real import MoveIt2 as MoveIt2_Real
from pymoveit2_real.robots import ur as realrobot


def _now_ros(node: Node) -> str:
    t = node.get_clock().now().to_msg()
    return f"{t.sec}.{t.nanosec:09d}"

class TrajectoryManager:
    """Trajectory kaydetme ve yükleme işlemlerini yöneten sınıf"""
    def __init__(self, file_path="sensing_trajectories.json"):
        self.file_path = file_path
        self.trajectories = {}
        self.load_trajectories()
    
    def save_trajectory_from_msg(self, name, joint_traj_msg):
        """trajectory_msgs/JointTrajectory objesini dosyaya kaydet"""
        traj_dict = {
            'joint_names': joint_traj_msg.joint_names,
            'points': []
        }
        for point in joint_traj_msg.points:
            point_dict = {
                'positions': list(point.positions),
                'velocities': list(point.velocities),
                'accelerations': list(point.accelerations),
                'time_from_start': {
                    'sec': point.time_from_start.sec,
                    'nanosec': point.time_from_start.nanosec
                }
            }
            traj_dict['points'].append(point_dict)
        self.trajectories[name] = traj_dict
        with open(self.file_path, 'w') as f:
            json.dump(self.trajectories, f, indent=2)
            
    def load_trajectories(self):
        """Kaydedilmiş trajectory'leri dosyadan yükle"""
        if os.path.exists(self.file_path):
            with open(self.file_path, 'r') as f:
                self.trajectories = json.load(f)
            return True
        return False
    
    def get_trajectory(self, name):
        return self.trajectories.get(name, None)

class HarmonySensingRobotV3(Node):
    def __init__(self):
        super().__init__("harmony_sensing_robot")

        # ---------------- Params ----------------
        self.declare_parameter("fixed_frame", "world")
        self.declare_parameter("tcp_frame", "")
        self.declare_parameter("tcp_pose_rate_hz", 30.0)
        self.declare_parameter("scan_wait_time", 1.5)
        self.declare_parameter("max_scan_cycles", 1)  # 1 tur
        self.declare_parameter("publish_path_samples", 25)
        self.declare_parameter("confirm_timeout_sec", 30.0)
        self.declare_parameter("waiting_joints", [1.0, 0.0, -math.pi / 2, 0.0, -math.pi / 2, 0.0, 0.0])

        # defect list collection
        self.declare_parameter("defect_duplicate_distance", 0.15)
        
        # HIL Parameters
        self.declare_parameter("hil_playback_mode", False)
        self.declare_parameter("trajectory_file", "/home/cem/colcon_ws/src/sensing_trajectories.json")


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

        mode_val = self.get_parameter("hil_playback_mode").value
        if isinstance(mode_val, str):
            self.hil_playback_mode = mode_val.lower() in ("true", "1", "t", "yes")
        else:
            self.hil_playback_mode = bool(mode_val)

        self.trajectory_file = str(self.get_parameter("trajectory_file").value)

        # ---------------- Trajectory Manager ----------------
        self.traj_manager = TrajectoryManager(self.trajectory_file)

        # ---------------- MoveIt2 ----------------
        self.cbg = ReentrantCallbackGroup()
        self.moveit2 = MoveIt2_Sim(
            node=self,
            joint_names=robot.joint_names(),
            base_link_name="world",
            end_effector_name=robot.end_effector_name(),
            group_name=robot.MOVE_GROUP_ARM,
            callback_group=self.cbg,
        )
        self.moveit2.planner_id = "ESTkConfigDefault"
        self.moveit2.max_velocity = 1.0
        self.moveit2.max_acceleration = 1.0 
        self.moveit2.cartesian_avoid_collisions = True
        self.moveit2.cartesian_jump_threshold = 2.0

        # Gerçek Robot MoveIt (Eğer HIL modu aktifse)
        self.moveit2_real = None
        if self.hil_playback_mode:
            try:
                self.moveit2_real = MoveIt2_Real(
                    node=self,
                    joint_names=realrobot.joint_names(),
                    base_link_name="world",
                    end_effector_name=realrobot.end_effector_name(),
                    group_name=realrobot.MOVE_GROUP_ARM,
                    callback_group=self.cbg,
                )
                self.get_logger().info("HIL PLAYBACK MODU AKTIF: Gerçek robot MoveIt kontrolcüsü başlatıldı.")
            except Exception as e:
                self.get_logger().error(f"Gerçek robot MoveIt bağlantı hatası: {e}")
                self.hil_playback_mode = False
        else:
            self.get_logger().info("KAYIT MODU AKTIF (HIL Kapalı).")

        # ---------------- TF2 (fallback for TCP pose) ----------------
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

        # TCP pose timer
        self.tcp_timer = self.create_timer(1.0 / self.tcp_pose_rate_hz, self._publish_tcp_pose)

        # Worker thread
        self.worker = Thread(target=self._run_state_machine, daemon=True)
        self.worker.start()

        self._publish_robot_status("IDLE", "IDLE", "Waiting for START")
        self.get_logger().info(f"HarmonySensingRobotV3 initialized.")

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
        except: return

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

    def _defect_cb(self, msg: String):
        if self.state != "SR_MODE":
            return
        if not self.early_waiting_requested.is_set():
            self.early_waiting_requested.set()
            self.get_logger().info("Defect detected! Stopping scan and going to WAITING state")


    # ---------------- TCP pose & Path ----------------
    def _get_tcp_pose(self) -> Optional[Pose]:
        try:
            if hasattr(self.moveit2, "get_current_pose"):
                pose = self.moveit2.get_current_pose()
                if isinstance(pose, Pose):
                    self._last_tcp_pose = pose
                    return pose
        except: pass
        return self._last_tcp_pose

    def _publish_tcp_pose(self):
        if self.state == "SR_MODE":
            self._publish_robot_status("SR_MODE", "SR", "Scanning in progress")

        pose = self._get_tcp_pose()
        if pose is None: return

        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = self.fixed_frame
        ps.pose = pose
        self.tcp_pose_pub.publish(ps)

    def _pose_to_xyz(self, pose: Optional[Pose]) -> Optional[Tuple[float, float, float]]:
        if pose is None: return None
        return (float(pose.position.x), float(pose.position.y), float(pose.position.z))

    def _publish_path_plan_linear(self, start_xyz, goal_xyz):
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


    # ---------------- HIL Motion Logic ----------------
    def _reconstruct_traj(self, traj_dict, target_joint_names):
        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
        from builtin_interfaces.msg import Duration
        msg = JointTrajectory()
        msg.joint_names = target_joint_names
        for pt in traj_dict["points"]:
            p = JointTrajectoryPoint()
            p.positions = pt["positions"]
            p.velocities = pt.get("velocities", [0.0] * len(target_joint_names))
            p.accelerations = pt.get("accelerations", [0.0] * len(target_joint_names))
            p.time_from_start = Duration(sec=pt["time_from_start"]["sec"], nanosec=pt["time_from_start"]["nanosec"])
            msg.points.append(p)
        return msg

    def move_to_joint_angles(self, joint_positions: List[float], synchronous: bool = True, traj_name: str = "default_traj") -> bool:
        if not self.hil_playback_mode:
            # RECORD MODE
            for _ in range(3): # retry mechanism
                try:
                    traj = self.moveit2.plan(joint_positions=joint_positions)
                    if traj:
                        self.traj_manager.save_trajectory_from_msg(traj_name, traj)
                        self.moveit2.execute(traj)
                        if synchronous:
                            return bool(self.moveit2.wait_until_executed())
                        return True
                except Exception as e:
                    self.get_logger().warn(f"Plan error {traj_name}: {e}")
                    time.sleep(0.5)
            return False
        
        else:
            # PLAYBACK MODE
            self.traj_manager.load_trajectories()
            traj_dict = self.traj_manager.get_trajectory(traj_name)
            if not traj_dict:
                self.get_logger().error(f"Playback fail: Trajectory '{traj_name}' not found!")
                return False

            sim_traj = self._reconstruct_traj(traj_dict, self.moveit2.joint_names)
            real_traj = self._reconstruct_traj(traj_dict, self.moveit2_real.joint_names)

            ready_lock = Lock()
            ready_count = 0
            execute_event = Event()
            
            def exec_sim():
                nonlocal ready_count
                with ready_lock: ready_count += 1
                execute_event.wait()
                try: self.moveit2.execute(sim_traj)
                except: pass

            def exec_real():
                nonlocal ready_count
                with ready_lock: ready_count += 1
                execute_event.wait()
                try: self.moveit2_real.execute(real_traj)
                except: pass

            Thread(target=exec_sim, daemon=True).start()
            if self.moveit2_real:
                Thread(target=exec_real, daemon=True).start()

            while ready_count < 2:
                time.sleep(0.05)
            
            execute_event.set()
            time.sleep(0.2)
            
            sim_ok = False
            try: sim_ok = self.moveit2.wait_until_executed()
            except: pass

            if self.moveit2_real:
                try: self.moveit2_real.wait_until_executed()
                except: pass

            return sim_ok

    def safe_joint_sequence(self, list_of_joint_angles: List[List[float]], wait_time: float, max_retries: int = 3, cycle_index: int = 0) -> bool:
        for i, joint_angles in enumerate(list_of_joint_angles):
            if self.stop_requested.is_set():
                return False
            if self.early_waiting_requested.is_set():
                return True

            self._publish_robot_status("SR_MODE", "SR", f"MOVING to waypoint={i}")

            start_pose = self._get_tcp_pose()
            start_xyz = self._pose_to_xyz(start_pose)

            traj_name = f"scan_cycle_{cycle_index}_pt_{i}"
            ok = False
            for attempt in range(max_retries):
                if self.stop_requested.is_set():
                    return False
                ok = self.move_to_joint_angles(joint_angles, synchronous=True, traj_name=traj_name)
                if ok: break
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
        home_joints = [1.0, 0.0, -1.5708, 0.0, -1.5708, 0.0, 0.0]
        pose2_joints = [1.8, math.radians(0), math.radians(-90.0), math.radians(0.0), math.radians(-90.0), math.radians(0.0), math.radians(0.0)]
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
            pose2_joints, pose5_joints, pose9_joints, pose6_joints,
            pose7_joints, pose10_joints, pose4_joints, pose3_joints,
            pose11_joints, pose8_joints,
        ]

        while rclpy.ok():
            if not self.active_scan.wait(timeout=0.2):
                continue
            if self.stop_requested.is_set():
                self.active_scan.clear()
                continue

            self._publish_robot_status("SR_MODE", "SR", "Going HOME")
            self.move_to_joint_angles(home_joints, synchronous=True, traj_name="go_home_sr")
            time.sleep(1.0)

            for cycle in range(self.max_scan_cycles):
                if self.stop_requested.is_set():
                    break
                self.safe_joint_sequence(scan_waypoints, wait_time=self.scan_wait_time, max_retries=3, cycle_index=cycle)

            if self.early_waiting_requested.is_set():
                self._publish_robot_status("SR_MODE", "SR", "Red defect detected. Going to WAITING position")
                self.move_to_joint_angles(self.waiting_joints, synchronous=True, traj_name="go_waiting_sr")

            self._set_state("WAITING", "SR", "Waiting for CONFIRM ...")

            t_wait_start = time.time()
            while rclpy.ok():
                if self.stop_requested.is_set():
                    break
                # CONFIRM logic vs CR_MODE transitions... it awaits commands from outside
                if self.state == "CR_MODE":
                    break
                if self.reinspect_requested.is_set():
                    self.reinspect_requested.clear()
                    break
                if self.confirm_timeout_sec > 0 and (time.time() - t_wait_start) > self.confirm_timeout_sec:
                    self._set_state("IDLE", "IDLE", "No CONFIRM received. Back to IDLE", level="WARN")
                    self.active_scan.clear()
                    break
                time.sleep(0.1)

            if self.stop_requested.is_set():
                self._set_state("IDLE", "IDLE", "Stopped.", level="WARN")
                self.stop_requested.clear()
                self.active_scan.clear()
                self.early_waiting_requested.clear()
                continue

            self._set_state("COMPLETE", "IDLE", "SR complete")
            self.active_scan.clear()
            self.early_waiting_requested.clear()
            self._set_state("IDLE", "IDLE", "Waiting for START")

def main():
    rclpy.init()
    node = HarmonySensingRobotV3()
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