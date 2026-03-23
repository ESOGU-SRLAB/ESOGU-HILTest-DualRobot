#!/usr/bin/env python3
"""
CR (Cleaning) Node - v3.3 (SimToReal entegrasyonu)
- defect_list veya single defect alır
- CONFIRM gelince 3 noktalı geçiş:
    (x+dx, y-0.3, z) -> (x-dx, y-0.3, z)
- Her hedef için max 5 retry
- Cleaning sırasında gelen CONFIRM ignorlanır (2 tur sorununu çözer)
- Aynı scan_id ikinci kez temizlenmez (safety lock)
- Sim'de planlanan trajectory gerçek robota da eş zamanlı gönderilir
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
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from pymoveit2_sim import MoveIt2 as MoveIt2_Sim
from pymoveit2_sim.robots import ur as robot

from pymoveit2_real import MoveIt2 as MoveIt2_Real
from pymoveit2_real.robots import ur as realrobot


def _now_ros(node: Node) -> str:
    t = node.get_clock().now().to_msg()
    return f"{t.sec}.{t.nanosec:09d}"


# ===========================================================================
# SimToReal Bridge  (sensing_robot_v3.py ile aynı sınıf — kopyalandı)
# ===========================================================================
class SimToRealBridge:
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

    def move_to_configuration(self, joint_positions: List[float]) -> bool:
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

    def move_to_pose(
        self,
        position: List[float],
        quat_xyzw: List[float],
        cartesian: bool = False,
        # cartesian_max_step: float = 0.01,
        cartesian_fraction_threshold: float = 0.40,
    ) -> bool:
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

    def _execute_both(self, sim_trajectory, joint_target: Optional[List[float]]) -> bool:
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

    def _adapt_trajectory_for_real(self, sim_trajectory) -> Optional[JointTrajectory]:
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
# Cleaning Node
# ===========================================================================
class HarmonyCleaningMissionRunnerV3(Node):
    def __init__(self):
        super().__init__("harmony_cleaning_runner")

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

        # Bridge parametreleri
        self.declare_parameter("real_robot_enabled", True)
        self.declare_parameter("real_robot_velocity", 0.05)
        self.declare_parameter("real_robot_acceleration", 0.05)
        self.declare_parameter("bridge_wait_real", True)
        self.declare_parameter("bridge_time_scale", 13.0)   # >1.0 → real robot yavaşlar

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

        real_enabled = bool(self.get_parameter("real_robot_enabled").value)
        real_velocity = float(self.get_parameter("real_robot_velocity").value)
        real_accel = float(self.get_parameter("real_robot_acceleration").value)
        bridge_wait_real = bool(self.get_parameter("bridge_wait_real").value)
        bridge_time_scale = float(self.get_parameter("bridge_time_scale").value)

        cbg = ReentrantCallbackGroup()

        # ---------------- MoveIt2 (Sim) ----------------
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
        self.last_cleaned_scan_id: Optional[str] = None

        self.force_timer = self.create_timer(1.0 / self.force_rate_hz, self._publish_force)

        self.worker = Thread(target=self._run, daemon=True)
        self.worker.start()

        self._publish_robot_status("IDLE", "IDLE", "CR node ready (waiting defect + CONFIRM)")
        self.get_logger().info(
            f"HarmonyCleaningMissionRunnerV3.3 initialized | "
            f"real_robot={'ENABLED' if self.bridge.real_available else 'DISABLED'}"
        )

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
        try:
            data = json.loads(msg.data)
            if not isinstance(data, dict):
                return
        except Exception:
            self.get_logger().warning(f"Invalid JSON on /harmony/mock_perception/defect: {msg.data[:200]}")
            return

        xyz = self._extract_xyz(data)
        if xyz is None:
            self.get_logger().warning("No x,y,z in defect payload")
            return

        with self.defect_lock:
            self.defects.append({"position": {"x": xyz[0], "y": xyz[1], "z": xyz[2]}, "raw": data})
            n = len(self.defects)

        self._publish_robot_status("WAITING", "CR", f"Defect received: {n} total. Waiting CONFIRM")
        self.get_logger().info(f"Defect collected: {n} total")

    def _cmd_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            cmd = str(data.get("cmd", "")).upper().strip()
        except Exception:
            return

        # Cleaning sırasında CONFIRM gelirse ignore et
        if cmd == "CONFIRM" and self.cleaning_active.is_set():
            return

        if cmd == "START":
            with self.defect_lock:
                self.defects.clear()
                self.scan_id = None
            self.get_logger().info("START received - cleared old defects")
        elif cmd == "CONFIRM":
            self.confirm_event.set()
        elif cmd == "WAITING":
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

        self._publish_robot_status("CR_MODE", "CR", "Cleaning in progress")

        fz = random.uniform(5.0, 20.0)
        msg = WrenchStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.tool_frame
        msg.wrench.force.z = fz
        self.force_pub.publish(msg)

    # ---------------- Motion ----------------
    def move_to_pose(self, position: List[float], cartesian: bool = False) -> bool:
        """Bridge üzerinden sim planla + real'e de gönder."""
        quat = [0.0, 0.0, 0.0, 1.0]
        try:
            return self.bridge.move_to_pose(
                position=[float(position[0]), float(position[1]), float(position[2])],
                quat_xyzw=quat,
                cartesian=cartesian,
                # cartesian_max_step=self.cartesian_max_step,
                cartesian_fraction_threshold=self.cartesian_fraction_threshold,
            )
        except Exception as e:
            self.get_logger().warning(f"[CR] move_to_pose hatası: {e}")
            return False

    def move_to_joint_config(self, joints: List[float]) -> bool:
        """Bridge üzerinden sim planla + real'e de gönder."""
        try:
            return self.bridge.move_to_configuration([float(x) for x in joints])
        except Exception as e:
            self.get_logger().warning(f"[CR] move_to_joint_config hatası: {e}")
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

            self._publish_robot_status(
                "CR_MODE", "CR",
                f"Cleaning started (scan_id={scan_key}, n={len(defects_copy)})"
            )
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

                    ok = self.move_with_retries(target, try_non_cartesian_first=True)
                    if not ok:
                        continue

                    t0 = time.time()
                    while time.time() - t0 < self.cleaning_duration:
                        if self.stop_event.is_set():
                            break
                        time.sleep(0.01)

            self.cleaning_active.clear()

            if self.stop_event.is_set():
                self._publish_robot_status("IDLE", "IDLE", "Cleaning stopped", level="WARN")
                self.stop_event.clear()
                continue

            self.move_to_joint_config(home_joints)
            self._publish_robot_status("COMPLETE", "CR", "Cleaning completed")
            time.sleep(2.0)
            self._publish_robot_status("IDLE", "IDLE", "Back to IDLE, waiting defect + CONFIRM")


def main():
    rclpy.init()
    node = HarmonyCleaningMissionRunnerV3()
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