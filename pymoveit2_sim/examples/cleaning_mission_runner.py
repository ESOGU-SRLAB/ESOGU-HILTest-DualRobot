#!/usr/bin/env python3
"""
CR (Cleaning) Node - v3 (HIL Playback Integrated)
- defect_list veya single defect alır
- CONFIRM gelince 3 noktalı geçiş
- Kayıt Modu (hil_playback_mode=False): Hedefe hareket planlar, JSON'a yazar ve simüle robotu oynatır.
- Playback Modu (hil_playback_mode=True): JSON'dan okur, Sim+Real robotlara senkronize yollar.
"""

from __future__ import annotations

import os
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
from pymoveit2_real import MoveIt2 as MoveIt2_Real
from pymoveit2_real.robots import ur as realrobot


def _now_ros(node: Node) -> str:
    t = node.get_clock().now().to_msg()
    return f"{t.sec}.{t.nanosec:09d}"

class TrajectoryManager:
    """Trajectory kaydetme ve yükleme işlemlerini yöneten sınıf"""
    def __init__(self, file_path="cleaning_trajectories.json"):
        self.file_path = file_path
        self.trajectories = {}
        self.load_trajectories()
    
    def save_trajectory_from_msg(self, name, joint_traj_msg):
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
        if os.path.exists(self.file_path):
            with open(self.file_path, 'r') as f:
                self.trajectories = json.load(f)
            return True
        return False
    
    def get_trajectory(self, name):
        return self.trajectories.get(name, None)

class HarmonyCleaningMissionRunnerV3(Node):
    def __init__(self):
        super().__init__("harmony_cleaning_runner")

        # ---------------- Params ----------------
        self.declare_parameter("fixed_frame", "world")
        self.declare_parameter("tool_frame", "tool0")
        self.declare_parameter("approach_height_offset", 0.3)
        self.declare_parameter("cleaning_duration", 1.5)
        self.declare_parameter("paint_y_offset", 0.30)   # y - 0.30
        self.declare_parameter("paint_x_span", 0.2)     # x +/- 0.15
        self.declare_parameter("move_max_retries", 5)
        self.declare_parameter("move_retry_sleep_s", 0.2)
        self.declare_parameter("cartesian_max_step", 0.01)
        self.declare_parameter("cartesian_fraction_threshold", 0.40)
        self.declare_parameter("force_rate_hz", 30.0)
        self.declare_parameter("skip_if_no_defects", True)
        
        # HIL Parameters
        self.declare_parameter("hil_playback_mode", False)
        self.declare_parameter("trajectory_file", "/home/cem/colcon_ws/src/cleaning_trajectories.json")


        self.fixed_frame = str(self.get_parameter("fixed_frame").value)
        self.tool_frame = str(self.get_parameter("tool_frame").value)

        self.approach_offset = float(self.get_parameter("approach_height_offset").value)
        self.cleaning_duration = float(self.get_parameter("cleaning_duration").value)

        self.paint_y_offset = float(self.get_parameter("paint_y_offset").value)
        self.cartesian_max_step = float(self.get_parameter("cartesian_max_step").value) * 5.0
        self.cartesian_fraction_threshold = float(self.get_parameter("cartesian_fraction_threshold").value) / 2.0
        self.paint_x_span = float(self.get_parameter("paint_x_span").value)

        self.move_max_retries = int(self.get_parameter("move_max_retries").value)
        self.move_retry_sleep_s = float(self.get_parameter("move_retry_sleep_s").value)

        self.force_rate_hz = float(self.get_parameter("force_rate_hz").value)
        self.skip_if_no_defects = bool(self.get_parameter("skip_if_no_defects").value)

        mode_val = self.get_parameter("hil_playback_mode").value
        # Gelen arguman eger komut satirinda string algilanmissa "true" "1", aksi halde boolean ise oldugu gibi al
        if isinstance(mode_val, str):
            self.hil_playback_mode = mode_val.lower() in ("true", "1", "t", "yes")
        else:
            self.hil_playback_mode = bool(mode_val)

        self.trajectory_file = str(self.get_parameter("trajectory_file").value)

        self.cbg = ReentrantCallbackGroup()

        # ---------------- Trajectory Manager ----------------
        self.traj_manager = TrajectoryManager(self.trajectory_file)

        # ---------------- MoveIt2 ----------------
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

        # ---------------- Publishers ----------------
        self.force_pub = self.create_publisher(WrenchStamped, "/harmony/cleaning/force", 10)
        self.robot_status_pub = self.create_publisher(String, "/harmony/robot_status", 10)

        # ---------------- Subscribers ----------------
        self.cmd_sub = self.create_subscription(String, "/harmony/cmd_input", self._cmd_cb, 10, callback_group=self.cbg)
        self.defect_sub = self.create_subscription(
            String, "/harmony/mock_perception/defect", self._defect_cb, 10, callback_group=self.cbg
        )

        # ---------------- State ----------------
        self.confirm_event = Event()
        self.stop_event = Event()
        self.cleaning_active = Event()

        self.defect_lock = Lock()
        self.scan_id: Optional[str] = None
        self.defects: List[Dict[str, Any]] = []
        self.last_cleaned_scan_id: Optional[str] = None

        self.force_timer = self.create_timer(1.0 / self.force_rate_hz, self._publish_force)

        self.worker = Thread(target=self._run, daemon=True)
        self.worker.start()

        self._publish_robot_status("IDLE", "IDLE", "CR node ready (waiting defect + CONFIRM)")

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

    def _extract_xyz(self, d: Dict[str, Any]) -> Optional[List[float]]:
        pos = None
        if isinstance(d.get("pos"), dict): pos = d["pos"]
        elif isinstance(d.get("position"), dict): pos = d["position"]
        elif all(k in d for k in ("x", "y", "z")): pos = d

        if not isinstance(pos, dict): return None
        try: return [float(pos["x"]), float(pos["y"]), float(pos["z"])]
        except: return None

    def _defect_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            if not isinstance(data, dict): return
        except: return

        xyz = self._extract_xyz(data)
        if xyz is None: return

        with self.defect_lock:
            self.defects.append({"position": {"x": xyz[0], "y": xyz[1], "z": xyz[2]}, "raw": data})
            n = len(self.defects)

        self._publish_robot_status("WAITING", "CR", f"Defect received: {n} total. Waiting CONFIRM")

    def _cmd_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            cmd = str(data.get("cmd", "")).upper().strip()
        except: return

        if cmd == "CONFIRM" and self.cleaning_active.is_set():
            return

        if cmd == "START":
            with self.defect_lock:
                self.defects.clear()
                self.scan_id = None
        elif cmd == "CONFIRM":
            self.confirm_event.set()
        elif cmd == "WAITING":
            self._publish_robot_status("WAITING", "CR", "WAITING command received, staying in WAITING")
        elif cmd == "STOP":
            self.stop_event.set()
            self.confirm_event.clear()
            self.cleaning_active.clear()
            self._publish_robot_status("IDLE", "IDLE", "STOP received, abort cleaning", level="WARN")

    def _publish_force(self):
        if not self.cleaning_active.is_set():
            return
        self._publish_robot_status("CR_MODE", "CR", "Cleaning in progress")
        fz = random.uniform(5.0, 20.0)
        msg = WrenchStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.tool_frame
        msg.wrench.force.z = fz
        self.force_pub.publish(msg)

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

    def _execute_hil_traj(self, traj_name: str, get_plan_func) -> bool:
        if not self.hil_playback_mode:
            for _ in range(3):
                try:
                    traj = get_plan_func()
                    if traj:
                        self.traj_manager.save_trajectory_from_msg(traj_name, traj)
                        self.moveit2.execute(traj)
                        return bool(self.moveit2.wait_until_executed())
                except Exception as e:
                    self.get_logger().warn(f"Plan error {traj_name}: {e}")
                time.sleep(0.5)
            self.get_logger().error(f"Failed to plan/execute {traj_name} (Record Mode)")
            return False
        
        else:
            self.traj_manager.load_trajectories()
            traj_dict = self.traj_manager.get_trajectory(traj_name)
            if not traj_dict:
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

    def move_to_pose(self, position: List[float], cartesian: bool = False, traj_name: str = "pose_traj") -> bool:
        quat = [0.0, 0.0, 0.0, 1.0]
        def _plan():
            return self.moveit2.plan(
                position=[float(position[0]), float(position[1]), float(position[2])],
                quat_xyzw=quat,
                cartesian=cartesian,
                max_step=self.cartesian_max_step,
                cartesian_fraction_threshold=self.cartesian_fraction_threshold,
                tolerance_position=0.015,
                tolerance_orientation=0.05,
            )
        return self._execute_hil_traj(traj_name, _plan)

    def move_to_joint_config(self, joints: List[float], traj_name: str = "joint_traj") -> bool:
        joints_f = [float(x) for x in joints]
        def _plan():
            return self.moveit2.plan(joint_positions=joints_f)
        return self._execute_hil_traj(traj_name, _plan)

    def move_with_retries(self, position: List[float], try_non_cartesian_first: bool = True, traj_prefix: str = "traj") -> bool:
        for attempt in range(1, self.move_max_retries + 1):
            if self.stop_event.is_set():
                return False

            ok = False
            if try_non_cartesian_first:
                ok = self.move_to_pose(position, cartesian=False, traj_name=f"{traj_prefix}_noncartesian_try_{attempt}")
                if not ok:
                    ok = self.move_to_pose(position, cartesian=True, traj_name=f"{traj_prefix}_cartesian_try_{attempt}")
            else:
                ok = self.move_to_pose(position, cartesian=True, traj_name=f"{traj_prefix}_cartesian_try_{attempt}")
                if not ok:
                    ok = self.move_to_pose(position, cartesian=False, traj_name=f"{traj_prefix}_noncartesian_try_{attempt}")

            if ok:
                return True
            time.sleep(self.move_retry_sleep_s)

        return False

    # ---------------- Worker ----------------
    def _run(self):
        home_joints = [1.0, 0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
        time.sleep(0.5)

        clean_cycle = 0

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
                self.defects = []
                self.scan_id = None

            if len(defects_copy) == 0:
                if self.skip_if_no_defects:
                    self._publish_robot_status("COMPLETE", "CR", "No defects to clean (complete)")
                    self._publish_robot_status("IDLE", "IDLE", "Back to IDLE")
                continue

            scan_key = str(scan_id) if scan_id is not None else "no_scan_id"
            if self.last_cleaned_scan_id == scan_key:
                self._publish_robot_status("IDLE", "IDLE", "Duplicate CONFIRM/scan ignored")
                continue
            self.last_cleaned_scan_id = scan_key
            
            clean_cycle += 1

            self._publish_robot_status("CR_MODE", "CR", f"Cleaning started (scan_id={scan_key}, n={len(defects_copy)})")
            self.cleaning_active.set()

            # Home
            self.move_to_joint_config(home_joints, traj_name=f"clean_{clean_cycle}_home_start")
            time.sleep(0.3)

            for d_idx, d in enumerate(defects_copy):
                if self.stop_event.is_set():
                    break

                xyz = self._extract_xyz(d if isinstance(d, dict) else {})
                if xyz is None: continue

                x, y, z = xyz
                y2 = y - self.paint_y_offset
                dx = self.paint_x_span

                targets = [
                    [x + dx, y2, z],
                    [x - dx, y2, z],
                ]

                for t_idx, target in enumerate(targets):
                    if self.stop_event.is_set(): break
                    
                    traj_prefix = f"clean_{clean_cycle}_d_{d_idx}_t_{t_idx}"
                    ok = self.move_with_retries(target, try_non_cartesian_first=True, traj_prefix=traj_prefix)
                    if not ok: continue

                    t0 = time.time()
                    while time.time() - t0 < self.cleaning_duration:
                        if self.stop_event.is_set(): break
                        time.sleep(0.01)

            self.cleaning_active.clear()

            if self.stop_event.is_set():
                self._publish_robot_status("IDLE", "IDLE", "Cleaning stopped", level="WARN")
                self.stop_event.clear()
                continue

            self.move_to_joint_config(home_joints, traj_name=f"clean_{clean_cycle}_home_end")
            self._publish_robot_status("COMPLETE", "CR", "Cleaning completed")
            
            time.sleep(2.0)
            self._publish_robot_status("IDLE", "IDLE", "Back to IDLE, waiting defect + CONFIRM")

def main():
    rclpy.init()
    node = HarmonyCleaningMissionRunnerV3()
    exec_ = MultiThreadedExecutor(num_threads=4)
    exec_.add_node(node)
    try:
        exec_.spin()
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()