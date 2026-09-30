#!/usr/bin/env python3
"""
Gerçek Kawasaki + AGV robotunu sıralı waypoint'lerle hareket ettiren kontrolcü.

Mimari:
  1. plan_async() → move_group planlama servisi (real_kawasaki grubu)
  2. send_goal_async() → /kawasaki_controller/follow_joint_trajectory
     (move_group execute bypass — direkt controller)

Controller: /kawasaki_controller/follow_joint_trajectory
Group     : real_kawasaki
Joints    : [world_to_agv, joint1, joint2, joint3, joint4, joint5, joint6]
"""

import time
import math
from threading import Thread

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, QoSDurabilityPolicy,
    QoSReliabilityPolicy, QoSHistoryPolicy,
)

from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTrajectoryControllerState
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory

from pymoveit2_kawasaki_real import MoveIt2
from pymoveit2_kawasaki_real.robots import ur as kawasaki_cfg

# ── Sabitler ─────────────────────────────────────────────────────────────────
CONTROLLER    = "/kawasaki/kawasaki_controller/follow_joint_trajectory"
GROUP_NAME    = kawasaki_cfg.MOVE_GROUP_ARM           # "real_kawasaki"

POLL_INTERVAL  = 0.05
PLAN_TIMEOUT   = 15.0
MOTION_TIMEOUT = 60.0
# ── Controller state topic ───────────────────────────────────────────────────
CONTROLLER_STATE = "/kawasaki/kawasaki_controller/state"
# ─────────────────────────────────────────────────────────────────────────────


def _wait_future(future, node: Node, label: str, timeout: float = PLAN_TIMEOUT) -> bool:
    """time.sleep polling — rclpy.spin_once kullanmaz, executor ile çakışmaz."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        if future is not None and future.done():
            return True
        time.sleep(POLL_INTERVAL)
    node.get_logger().error(f"[{label}] Timeout ({timeout:.1f}s)")
    return False


def _make_fjt_goal(trajectory: JointTrajectory) -> FollowJointTrajectory.Goal:
    goal = FollowJointTrajectory.Goal()
    goal.trajectory = trajectory
    return goal


# =============================================================================
class KawasakiController(Node):
    """Kawasaki + AGV robotunu gerçek donanım üzerinde hareket ettirir."""

    def __init__(self):
        super().__init__("kawasaki_real_controller")

        cb = ReentrantCallbackGroup()

        # ── MoveIt2 planlama arayüzü ──────────────────────────────────────
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=kawasaki_cfg.joint_names(),           # prefix yok
            base_link_name=kawasaki_cfg.base_link_name(),
            end_effector_name=kawasaki_cfg.end_effector_name(),
            group_name=GROUP_NAME,
            callback_group=cb,
        )
        self.moveit2.planner_id       = "RRTConnectkConfigDefault"
        self.moveit2.max_velocity     = 0.01
        self.moveit2.max_acceleration = 0.01

        # ── Doğrudan controller action client (execution) ─────────────────
        _qos = QoSProfile(
            durability=QoSDurabilityPolicy.VOLATILE,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._ctrl = ActionClient(
            node=self,
            action_type=FollowJointTrajectory,
            action_name=CONTROLLER,
            goal_service_qos_profile=_qos,
            callback_group=cb,
        )

        # ── Mevcut eklem pozisyonu (controller state'ten otomatik okunur) ──
        # /kawasaki_controller/state → actual.positions alanından güncellenir.
        # /joint_states yayınlanmasa bile doğru başlangıç state'i sağlar.
        self._current_joints: list = [0.0] * len(kawasaki_cfg.joint_names())
        self._joint_state_received = False

        self.create_subscription(
            JointTrajectoryControllerState,
            CONTROLLER_STATE,
            self._controller_state_cb,
            10,
            callback_group=cb,
        )

        self.get_logger().info(f"Kontrolcü hazır — group: {GROUP_NAME}")
        self.get_logger().info(f"Controller: {CONTROLLER}")

    def _controller_state_cb(self, msg: JointTrajectoryControllerState):
        """Controller'dan gelen actual pozisyonu saklar."""
        if msg.actual.positions:
            self._current_joints = list(msg.actual.positions)
            self._joint_state_received = True

    def _wait_for_joint_state(self, timeout: float = 10.0) -> bool:
        """İlk controller state mesajını bekler."""
        self.get_logger().info("Eklem pozisyonu bekleniyor...")
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self._joint_state_received:
                self.get_logger().info(
                    f"✓ Başlangıç pozisyonu alındı: "
                    f"{[round(math.degrees(v), 1) for v in self._current_joints]}"
                )
                return True
            time.sleep(0.1)
        self.get_logger().warn(
            "Controller state alınamadı — home (sıfır) pozisyonu kullanılıyor."
        )
        return False

    # ─────────────────────────────────────────────────────────────────────────
    def wait_for_controller(self, timeout: float = 10.0) -> bool:
        self.get_logger().info("Controller bekleniyor...")
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self._ctrl.server_is_ready():
                self.get_logger().info("✓ Controller hazır.")
                return True
            time.sleep(1.0)
        self.get_logger().error("Controller timeout içinde hazır olmadı!")
        return False

    # ─────────────────────────────────────────────────────────────────────────
    def _make_start_state(self) -> JointState:
        """Takip edilen mevcut pozisyondan JointState oluşturur."""
        js = JointState()
        js.name = kawasaki_cfg.joint_names()
        js.position = [float(p) for p in self._current_joints]
        return js

    def _plan(self, joint_positions: list, label: str,
              max_retries: int = 3):
        for attempt in range(max_retries):
            self.get_logger().info(
                f"[{label}] Planlama {attempt + 1}/{max_retries}..."
            )
            future = self.moveit2.plan_async(
                joint_positions=joint_positions,
                start_joint_state=self._make_start_state(),
            )
            if future is None:
                self.get_logger().warn(f"[{label}] plan_async() None döndürdü.")
                time.sleep(0.3)
                continue

            if not _wait_future(future, self, label, PLAN_TIMEOUT):
                continue

            traj = self.moveit2.get_trajectory(future)
            if traj is not None:
                self.get_logger().info(f"[{label}] ✓ Plan hazır.")
                return traj
            self.get_logger().warn(f"[{label}] Plan bulunamadı, tekrar deneniyor.")
            time.sleep(0.3)

        self.get_logger().error(f"[{label}] Tüm planlama denemeleri başarısız!")
        return None

    # ─────────────────────────────────────────────────────────────────────────
    def _execute(self, traj: JointTrajectory, label: str) -> bool:
        """Planlanmış trajectory'yi direkt controller'a gönderir."""
        goal = _make_fjt_goal(traj)
        send_future = self._ctrl.send_goal_async(goal)
        self.get_logger().info(f"[{label}] → Controller'a gönderildi.")

        # Goal kabul bekleme
        if not _wait_future(send_future, self, label, timeout=5.0):
            self.get_logger().error(f"[{label}] Goal future zaman aşımına uğradı.")
            return False

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f"[{label}] Goal REJECTed!")
            return False

        self.get_logger().info(f"[{label}] Goal kabul edildi, hareket başladı.")

        # Hareket sonucu bekleme
        result_future = goal_handle.get_result_async()
        if not _wait_future(result_future, self, label, timeout=MOTION_TIMEOUT):
            self.get_logger().error(f"[{label}] Hareket timeout ({MOTION_TIMEOUT}s)!")
            return False

        from action_msgs.msg import GoalStatus
        result = result_future.result()
        success = (result.status == GoalStatus.STATUS_SUCCEEDED)
        if success:
            self.get_logger().info(f"[{label}] ✓ Hareket tamamlandı.")
        else:
            self.get_logger().warn(f"[{label}] Hareket başarısız: status={result.status}")
        return success

    # ─────────────────────────────────────────────────────────────────────────
    def move_to(self, joint_positions: list, label: str = "WP",
                max_retries: int = 3) -> bool:
        """Tek bir waypoint'e git."""
        traj = self._plan(joint_positions, label, max_retries)
        if traj is None:
            return False
        return self._execute(traj, label)
        # _current_joints artık subscriber üzerinden otomatik güncellenir.

    # ─────────────────────────────────────────────────────────────────────────
    def run_sequence(self, waypoints: list, wait_between: float = 1.5,
                     max_retries: int = 3):
        """Waypoint listesini sırayla uygular."""
        total = len(waypoints)
        for i, wp in enumerate(waypoints):
            self.get_logger().info(
                f"\n{'='*60}\n"
                f"  Waypoint {i + 1}/{total}\n"
                f"  AGV: {wp[0]:.3f}  "
                f"j1-j6: {[round(math.degrees(v), 1) for v in wp[1:]]}\n"
                f"{'='*60}"
            )
            ok = self.move_to(wp, label=f"WP{i+1}", max_retries=max_retries)
            if not ok:
                self.get_logger().error(
                    f"Waypoint {i+1} başarısız! Sıradaki nokta atlanıyor."
                )
            if i < total - 1:
                time.sleep(wait_between)


# =============================================================================
# WAYPOINT TANIMLARI
# =============================================================================

def get_waypoints():
    """
    Her satır: [world_to_agv, joint1, joint2, joint3, joint4, joint5, joint6]
    """
    return [
        [0.63, math.radians(74.732),  math.radians(-8.589),   math.radians(-30.942),
         math.radians(-108.911), math.radians(85.335),  math.radians(37.744)],

        [0.63, math.radians(74.732),  math.radians(-62.050),  math.radians(-132.843),
         math.radians(-99.243),  math.radians(72.803),  math.radians(-10.063)],

        [1.15, math.radians(74.732),  math.radians(-8.589),   math.radians(-30.942),
         math.radians(-108.911), math.radians(85.335),  math.radians(37.744)],

        [1.15, math.radians(74.732),  math.radians(-62.050),  math.radians(-132.843),
         math.radians(-99.243),  math.radians(72.803),  math.radians(-10.063)],

        [1.65, math.radians(74.732),  math.radians(-8.589),   math.radians(-30.942),
         math.radians(-108.911), math.radians(85.335),  math.radians(37.744)],

        [1.65, math.radians(74.732),  math.radians(-62.050),  math.radians(-132.843),
         math.radians(-99.243),  math.radians(72.803),  math.radians(-10.063)],
    ]


# =============================================================================
# MAIN
# =============================================================================

def main():
    rclpy.init()

    node = KawasakiController()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    if not node.wait_for_controller(timeout=15.0):
        node.get_logger().error("Controller hazır değil, çıkılıyor.")
        rclpy.shutdown()
        return

    # Robotun gerçek başlangıç pozisyonunu controller state'ten oku
    node._wait_for_joint_state(timeout=10.0)

    waypoints = get_waypoints()

    loop_counter = 0

    try:
        node.get_logger().info("=== KAWASAKI REAL — SONSUZ DÖNGÜ BAŞLATILIYOR ===")
        node.get_logger().info(f"Toplam {len(waypoints)} waypoint/döngü.")
        node.get_logger().info("Durdurmak için Ctrl+C")

        while rclpy.ok():
            loop_counter += 1
            node.get_logger().info(f"=== DÖNGÜ {loop_counter} ===")

            node.run_sequence(waypoints, wait_between=1.5, max_retries=3)

            node.get_logger().info(
                f"Döngü {loop_counter} tamamlandı. 3s bekleniyor..."
            )
            time.sleep(3.0)

    except KeyboardInterrupt:
        node.get_logger().info(
            f"Kullanıcı tarafından durduruldu. Toplam döngü: {loop_counter}"
        )
    except Exception as e:
        import traceback
        node.get_logger().error(f"Hata: {e}\n{traceback.format_exc()}")
    finally:
        node.get_logger().info("Kapatılıyor...")
        rclpy.shutdown()
        executor_thread.join(timeout=5.0)


if __name__ == "__main__":
    main()
