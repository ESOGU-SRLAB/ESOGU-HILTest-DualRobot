#!/usr/bin/env python3
"""
İki robotu GERÇEKTEN EŞ ZAMANLI hareket ettiren kontrolcü.

=== NEDEN ÖNCE ÇALIŞMADI ===

  Tüm önceki yöntemler şu paylaşımlı sunuculara gidiyordu:
    /sim/move_action          → move_group  (sıralı, tek işlemli)
    /sim/execute_trajectory   → move_group  (sıralı, tek işlemli)
  
  Bu sunucular aynı anda yalnızca BİR hedef işler.

=== GERÇEK ÇÖZÜM ===

  Her robotun KENDİ bağımsız controller action server'ı var:
    UR10e    → /sim/sim_scaled_joint_trajectory_controller/follow_joint_trajectory
    Kawasaki → /sim/kawasaki_controller/follow_joint_trajectory

  Bu server'lar BİRBİRİNDEN BAĞIMSIZ çalışır.

  1. plan_async() ile her robot için trajectory planla (paylaşımlı plan servisi, sıralı OK)
  2. Her robotun KENDİ controller'ına send_goal_async() ile AYNI ANDA gönder
  3. Her iki future tamamlanana kadar time.sleep() polling yap
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
from trajectory_msgs.msg import JointTrajectory

from pymoveit2_sim import MoveIt2 as MoveIt2_UR
from pymoveit2_sim.robots import ur as ur_cfg

from pymoveit2_kawasaki_sim import MoveIt2 as MoveIt2_Kawasaki
from pymoveit2_kawasaki_sim.robots import ur as kawasaki_cfg

# ── MoveIt planning group isimleri ───────────────────────────────────────────
ROBOT_A_GROUP = "sim_ur10e"
ROBOT_B_GROUP = "sim_kawasaki"

# ── Doğrudan controller action isimleri ──────────────────────────────────────
CONTROLLER_A = "/sim/sim_scaled_joint_trajectory_controller/follow_joint_trajectory"
CONTROLLER_B = "/sim/kawasaki_controller/follow_joint_trajectory"
# ─────────────────────────────────────────────────────────────────────────────

POLL_INTERVAL  = 0.05
PLAN_TIMEOUT   = 15.0
MOTION_TIMEOUT = 60.0


# =============================================================================
# Yardımcı araçlar
# =============================================================================

def _wait_future(future, node, label: str, timeout: float = PLAN_TIMEOUT) -> bool:
    """time.sleep polling — rclpy.spin_once yok, executor ile çakışmaz."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        if future is not None and future.done():
            return True
        time.sleep(POLL_INTERVAL)
    node.get_logger().error(f"[{label}] Planlama timeout ({timeout}s)")
    return False


def _make_fjt_goal(trajectory: JointTrajectory) -> FollowJointTrajectory.Goal:
    """JointTrajectory → FollowJointTrajectory.Goal dönüşümü."""
    goal = FollowJointTrajectory.Goal()
    goal.trajectory = trajectory
    return goal


# =============================================================================
# Kontrolcü Node
# =============================================================================

class DualRobotController(Node):
    """
    iki aşamalı mimari:
      1. plan_async()  — her robot için sırayla (tek plan servisi)
      2. send_goal_async() — her robotun kendi controller'ına aynı anda
    """

    def __init__(self):
        super().__init__("dual_robot_controller")

        cb_a = ReentrantCallbackGroup()
        cb_b = ReentrantCallbackGroup()

        # ── MoveIt2 planlama interface'leri (YALNIZCA PLANLAMA için kullanılır) ──
        self.moveit2_a = MoveIt2_UR(
            node=self,
            joint_names=ur_cfg.joint_names(),
            base_link_name=ur_cfg.base_link_name(),
            end_effector_name=ur_cfg.end_effector_name(),
            group_name=ROBOT_A_GROUP,
            callback_group=cb_a,
        )
        self.moveit2_a.planner_id       = "RRTConnectkConfigDefault"
        self.moveit2_a.max_velocity     = 0.1
        self.moveit2_a.max_acceleration = 0.1

        self.moveit2_b = MoveIt2_Kawasaki(
            node=self,
            joint_names=kawasaki_cfg.joint_names(prefix="sim_kawasaki_"),
            base_link_name=kawasaki_cfg.base_link_name(prefix="sim_kawasaki_"),
            end_effector_name=kawasaki_cfg.end_effector_name(prefix="sim_kawasaki_"),
            group_name=ROBOT_B_GROUP,
            callback_group=cb_b,
        )
        self.moveit2_b.planner_id       = "RRTConnectkConfigDefault"
        self.moveit2_b.max_velocity     = 0.1
        self.moveit2_b.max_acceleration = 0.1

        # ── Doğrudan controller action client'ları (EXECUTION için) ─────────
        # Bunlar move_group'u TAMAMEN BYPASS eder.
        _qos = QoSProfile(
            durability=QoSDurabilityPolicy.VOLATILE,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._ctrl_a = ActionClient(
            node=self,
            action_type=FollowJointTrajectory,
            action_name=CONTROLLER_A,
            goal_service_qos_profile=_qos,
            callback_group=cb_a,
        )
        self._ctrl_b = ActionClient(
            node=self,
            action_type=FollowJointTrajectory,
            action_name=CONTROLLER_B,
            goal_service_qos_profile=_qos,
            callback_group=cb_b,
        )

        self.get_logger().info("Dual robot kontrolcüsü hazır.")
        self.get_logger().info(f"  Robot A controller: {CONTROLLER_A}")
        self.get_logger().info(f"  Robot B controller: {CONTROLLER_B}")

    def _wait_for_controllers(self, timeout: float = 10.0) -> bool:
        """Her iki controller action server'ının hazır olmasını bekler."""
        self.get_logger().info("Controller'lar bekleniyor...")
        deadline = time.time() + timeout
        while time.time() < deadline:
            a_ready = self._ctrl_a.server_is_ready()
            b_ready = self._ctrl_b.server_is_ready()
            if a_ready and b_ready:
                self.get_logger().info("✓ Her iki controller hazır.")
                return True
            self.get_logger().info(
                f"  Robot-A controller: {'hazır' if a_ready else 'bekleniyor...'}"
            )
            self.get_logger().info(
                f"  Robot-B controller: {'hazır' if b_ready else 'bekleniyor...'}"
            )
            time.sleep(1.0)
        self.get_logger().error("Controller'lar timeout içinde hazır olmadı!")
        return False

    # ─────────────────────────────────────────────────────────────────────────
    # Planlama
    # ─────────────────────────────────────────────────────────────────────────
    def _plan(self, moveit2, joint_positions: list, label: str,
              max_retries: int = 3) -> "Optional[JointTrajectory]":
        for attempt in range(max_retries):
            self.get_logger().info(
                f"[{label}] Planlama {attempt + 1}/{max_retries}..."
            )
            future = moveit2.plan_async(joint_positions=joint_positions)
            if future is None:
                self.get_logger().warn(f"[{label}] plan_async() None döndürdü.")
                time.sleep(0.3)
                continue

            # Executor future'ı işler, biz sadece tamamlanmasını bekleriz
            if not _wait_future(future, self, label):
                continue

            traj = moveit2.get_trajectory(future)
            if traj is not None:
                self.get_logger().info(f"[{label}] ✓ Plan hazır.")
                return traj
            self.get_logger().warn(f"[{label}] Plan bulunamadı, tekrar deneniyor.")
            time.sleep(0.3)

        self.get_logger().error(f"[{label}] Tüm planlama denemeleri başarısız!")
        return None

    # ─────────────────────────────────────────────────────────────────────────
    # Eş zamanlı hareket
    # ─────────────────────────────────────────────────────────────────────────
    def move_both_simultaneously(
        self,
        joint_positions_a: list,
        joint_positions_b: list,
        max_retries: int = 3,
    ) -> tuple:
        """
        1. Robot A için plan yap
        2. Robot B için plan yap
        3. İKİSİNİ AYNI ANDA kendi controller'larına gönder
        4. Her ikisinin bitmesini paralel bekle
        """
        # ── 1 & 2: Planlama (sıralı — tek plan servisi buna izin veriyor) ──
        traj_a = self._plan(self.moveit2_a, joint_positions_a, "Robot-A", max_retries)
        traj_b = self._plan(self.moveit2_b, joint_positions_b, "Robot-B", max_retries)

        if traj_a is None and traj_b is None:
            return False, False

        # ── 3: AYNI ANDA send_goal_async() — iki farklı controller ──────────
        # send_goal_async() non-blocking future döner.
        # İki satır arasında <1ms gecikme → pratikte eş zamanlı.
        future_send_a = None
        future_send_b = None

        if traj_a is not None:
            goal_a = _make_fjt_goal(traj_a)
            future_send_a = self._ctrl_a.send_goal_async(goal_a)
            self.get_logger().info("[Robot-A] → Trajectory gönderildi (UR10e controller)")

        if traj_b is not None:
            goal_b = _make_fjt_goal(traj_b)
            future_send_b = self._ctrl_b.send_goal_async(goal_b)
            self.get_logger().info("[Robot-B] → Trajectory gönderildi (Kawasaki controller)")

        # ── 4: Her iki robotun bitmesini PARALEL thread'lerle bekle ────────────
        # _wait_for_fjt bloklayıcı ama time.sleep kullanır — iki thread aynı anda bekleyebilir
        from threading import Thread, Event
        result_a = [False]
        result_b = [False]
        ev_a = Event()
        ev_b = Event()

        def _do_wait_a():
            if future_send_a:
                result_a[0] = self._wait_for_fjt(future_send_a, "Robot-A")
            ev_a.set()

        def _do_wait_b():
            if future_send_b:
                result_b[0] = self._wait_for_fjt(future_send_b, "Robot-B")
            ev_b.set()

        Thread(target=_do_wait_a, daemon=True).start()
        Thread(target=_do_wait_b, daemon=True).start()

        ev_a.wait(timeout=MOTION_TIMEOUT + 10.0)
        ev_b.wait(timeout=MOTION_TIMEOUT + 10.0)

        self.get_logger().info(
            f"Tamamlandı → Robot-A: {'✓' if result_a[0] else '✗'}  "
            f"Robot-B: {'✓' if result_b[0] else '✗'}"
        )
        return result_a[0], result_b[0]

    def _wait_for_fjt(self, send_future, label: str) -> bool:
        """
        FollowJointTrajectory goal'unun kabul edilmesini ve
        tamamlanmasını bekler. rclpy.spin_once() kullanmaz.
        """
        # Goal accept/reject bekleme
        if not _wait_future(send_future, self, label, timeout=5.0):
            self.get_logger().error(f"[{label}] Goal kabul future'ı zaman aşımına uğradı.")
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
        if not success:
            self.get_logger().warn(f"[{label}] Hareket başarısız: status={result.status}")
        return success

    # ─────────────────────────────────────────────────────────────────────────
    def run_dual_sequence(
        self,
        configs_a: list,
        configs_b: list,
        wait_between: float = 1.5,
        max_retries: int = 3,
    ):
        count = max(len(configs_a), len(configs_b))

        def cyclic(lst, n):
            return [lst[i % len(lst)] for i in range(n)]

        configs_a = cyclic(configs_a, count)
        configs_b = cyclic(configs_b, count)

        for i, (cfg_a, cfg_b) in enumerate(zip(configs_a, configs_b)):
            self.get_logger().info(
                f"\n{'='*60}\n"
                f"  Waypoint {i + 1}/{count}: EŞ ZAMANLI hareket\n"
                f"{'='*60}"
            )
            self.move_both_simultaneously(cfg_a, cfg_b, max_retries=max_retries)

            if i < count - 1:
                time.sleep(wait_between)

    def move_home(self, home_a: list, home_b: list):
        self.get_logger().info("Home pozisyonuna gönderiliyor...")
        self.move_both_simultaneously(home_a, home_b)


# =============================================================================
# WAYPOINT TANIMLARI
# =============================================================================

def get_robot_a_configs():
    home     = [1.0, 0.0, -math.pi/2, 0.0, -math.pi/2, 0.0, 0.0]
    vr_top   = [1.9, math.radians(-15.36), math.radians(-115.21), math.radians(-51.42),
                math.radians(-13.16), math.radians(110.86), math.radians(90.0)]
    vr_mid   = [1.9, math.radians(-15.39), math.radians(-117.15), math.radians(-82.83),
                math.radians(20.18), math.radians(110.77), math.radians(90.0)]
    vr_below = [1.9, math.radians(-15.36), math.radians(-147.23), math.radians(-90.96),
                math.radians(58.35), math.radians(110.72), math.radians(90.0)]
    inter    = [1.0, math.radians(34.37), math.radians(-55.44), math.radians(-122.10),
                math.radians(-2.37), math.radians(60.99), math.radians(0)]
    p2_down  = [1.0, math.radians(-19.36), math.radians(-122.69), math.radians(-51.80),
                math.radians(-2.51), math.radians(116.43), math.radians(0.0)]
    p3_down  = [1.0, math.radians(34.56), math.radians(-117.73), math.radians(-53.48),
                math.radians(-13.09), math.radians(55.02), math.radians(0.0)]
    vl_top   = [0.055, math.radians(25.22), math.radians(-135.98), math.radians(-9.01),
                math.radians(-23.58), math.radians(61.88), math.radians(-90.0)]
    vl_mid   = [0.055, math.radians(25.17), math.radians(-123.55), math.radians(-60.94),
                math.radians(15.93), math.radians(61.75), math.radians(-90.0)]
    return home, [vr_top, vr_mid, vr_below, inter, p2_down, p3_down, vl_top, vl_mid]


def get_robot_b_configs():
    home = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    pt1 = [0.9, math.radians(15.837),   math.radians(-28.815),  math.radians(-76.154),
           math.radians(-154.640), math.radians(31.511), math.radians(47.310)]
    pt2 = [0.1, math.radians(-104.117), math.radians(20.814),  math.radians(-31.719),
           math.radians(-290.426), math.radians(94.088), math.radians(116.370)]
    pt3 = [0.5, math.radians(-71.503),  math.radians(24.562),  math.radians(-25.947),
           math.radians(-270.191), math.radians(68.745), math.radians(118.992)]
    pt4 = [0.3, math.radians(-71.503),  math.radians(0.877),   math.radians(-101.176),
           math.radians(-287.050), math.radians(77.113), math.radians(172.441)]
    pt5 = [1.0, math.radians(-71.503),  math.radians(38.622),  math.radians(-125.932),
           math.radians(-289.487), math.radians(98.656), math.radians(231.511)]
    pt6 = [1.5, math.radians(-104.117), math.radians(20.814),  math.radians(-31.719),
           math.radians(-290.426), math.radians(94.088), math.radians(116.370)]
    pt7 = [1.2, math.radians(15.837),   math.radians(-28.815), math.radians(-76.154),
           math.radians(-154.640), math.radians(31.511), math.radians(47.310)]
    return home, [pt1, pt2, pt3, pt4, pt5, pt6, pt7]


# =============================================================================
# MAIN
# =============================================================================

def main():
    rclpy.init()

    controller = DualRobotController()

    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(controller)
    executor_thread = Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    # Controller'ların hazır olmasını bekle
    if not controller._wait_for_controllers(timeout=15.0):
        controller.get_logger().error("Controller'lar hazır değil, çıkılıyor.")
        rclpy.shutdown()
        return

    home_a, configs_a = get_robot_a_configs()
    home_b, configs_b = get_robot_b_configs()

    loop_counter = 0

    try:
        controller.get_logger().info("=== EŞ ZAMANLI DUAL ROBOT DÖNGÜSÜ BAŞLATILIYOR ===")
        controller.get_logger().info("Durdurmak için Ctrl+C")

        controller.move_home(home_a, home_b)
        time.sleep(2.0)

        while rclpy.ok():
            loop_counter += 1
            controller.get_logger().info(f"=== DÖNGÜ {loop_counter} ===")

            controller.run_dual_sequence(
                configs_a=configs_a,
                configs_b=configs_b,
                wait_between=1.5,
                max_retries=3,
            )

            controller.get_logger().info(
                f"Döngü {loop_counter} tamamlandı. 3s bekleniyor..."
            )
            time.sleep(3.0)

    except KeyboardInterrupt:
        controller.get_logger().info(
            f"Durduruldu. Toplam döngü: {loop_counter}"
        )
    except Exception as e:
        import traceback
        controller.get_logger().error(f"Hata: {e}\n{traceback.format_exc()}")
    finally:
        controller.get_logger().info("Kapatılıyor...")
        controller.move_home(home_a, home_b)
        time.sleep(2.0)
        rclpy.shutdown()
        executor_thread.join(timeout=5.0)


if __name__ == "__main__":
    main()