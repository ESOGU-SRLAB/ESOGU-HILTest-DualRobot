#!/usr/bin/env python3
"""
Kawasaki robot kolu eklem hedefleri + AGV pozisyon kontrolü
Belirli noktalarda AGV'yi X ekseninde hareket ettirir.
  - Point 4 → AGV X=0.6
  - Point 6 → AGV X=1.0
  - Point 9 → AGV X=1.5
  - Döngü sonu → AGV X=0.2 (başlangıca dönüş)
"""

from threading import Thread
import time
import math

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from pymoveit2_kawasaki_sim import MoveIt2 as MoveIt2_Sim
from pymoveit2_kawasaki_sim import MoveIt2State
from pymoveit2_kawasaki_sim.robots import ur as robot


class JointAGVController(Node):
    def __init__(self):
        super().__init__("joint_agv_controller")

        callback_group = ReentrantCallbackGroup()

        # MoveIt 2 arayüzü
        self.moveit2 = MoveIt2_Sim(
            node=self,
            joint_names=robot.joint_names(),
            base_link_name=robot.base_link_name(),
            end_effector_name=robot.end_effector_name(),
            group_name=robot.MOVE_GROUP_ARM,
            callback_group=callback_group,
        )

        self.moveit2.planner_id = "RRTConnectkConfigDefault"
        self.moveit2.max_velocity = 0.05
        self.moveit2.max_acceleration = 0.5
        self.moveit2.cartesian_avoid_collisions = True
        self.moveit2.cartesian_jump_threshold = 2.0

        self.planning_attempts = 10
        self.planning_time = 10.0

        # ── AGV publisher / subscriber ──
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            "/sim/ota_base_controller/cmd_vel_unstamped",
            10,
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            "/sim/ota_base_controller/odom",
            self._odom_cb,
            10,
        )

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.odom_received = False

        self.get_logger().info("JointAGVController başlatıldı")

    # ────────────────────────── AGV helpers ──────────────────────────
    def _odom_cb(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        o = msg.pose.pose.orientation
        siny_cosp = 2.0 * (o.w * o.z + o.x * o.y)
        cosy_cosp = 1.0 - 2.0 * (o.y * o.y + o.z * o.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
        self.odom_received = True

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def _distance_to(self, gx: float, gy: float) -> float:
        return math.hypot(gx - self.current_x, gy - self.current_y)

    def _angle_to(self, gx: float, gy: float) -> float:
        return math.atan2(gy - self.current_y, gx - self.current_x)

    def move_agv_to_x(self, goal_x: float, tolerance: float = 0.05):
        """
        AGV'yi X ekseninde goal_x konumuna götürür.
        Hedef arkadaysa dönmeden geri gider.
        """
        self.get_logger().info("AGV odometry bekleniyor...")
        wait = 0
        while not self.odom_received and wait < 50:
            rclpy.spin_once(self, timeout_sec=0.1)
            wait += 1
        if not self.odom_received:
            self.get_logger().error("Odometry alınamadı – AGV hareketi iptal!")
            return

        goal_y = self.current_y  # Y sabit
        self.get_logger().info(
            f"AGV: X={self.current_x:.2f} → hedef X={goal_x:.2f}"
        )

        linear_speed = 0.15
        angular_speed = 0.4
        angle_tol = 0.20
        reverse_thresh = math.pi / 2.0
        consecutive_ok = 0
        required = 5
        twist = Twist()

        for step in range(2000):
            rclpy.spin_once(self, timeout_sec=0.05)
            dist = self._distance_to(goal_x, goal_y)

            if step % 20 == 0:
                self.get_logger().info(
                    f"  AGV X={self.current_x:.2f}  mesafe={dist:.3f}"
                )

            if dist < tolerance:
                consecutive_ok += 1
                if consecutive_ok >= required:
                    self.get_logger().info(
                        f"AGV hedefe ulaştı  X={self.current_x:.2f}"
                    )
                    break
                twist.linear.x = 0.0
                twist.angular.z = 0.0
            else:
                consecutive_ok = 0
                target_angle = self._angle_to(goal_x, goal_y)
                adiff = self._normalize_angle(target_angle - self.current_yaw)

                if abs(adiff) > reverse_thresh:
                    back = self._normalize_angle(self.current_yaw + math.pi)
                    bdiff = self._normalize_angle(target_angle - back)
                    sf = min(1.0, dist / 0.5)
                    twist.linear.x = -linear_speed * sf
                    twist.angular.z = -0.3 * bdiff
                elif abs(adiff) > angle_tol:
                    twist.linear.x = 0.0
                    twist.angular.z = angular_speed if adiff > 0 else -angular_speed
                else:
                    sf = min(1.0, dist / 0.5)
                    twist.linear.x = linear_speed * sf
                    twist.angular.z = 0.3 * adiff

            self.cmd_vel_pub.publish(twist)

        # Dur
        stop = Twist()
        for _ in range(10):
            self.cmd_vel_pub.publish(stop)
            time.sleep(0.05)
        self.get_logger().info(f"AGV durdu  X={self.current_x:.2f}")

    # ────────────────────────── Arm helpers ──────────────────────────
    def move_to_joint_angles(self, joint_positions, synchronous=True) -> bool:
        self.get_logger().info(f"Joint hedefi: {[f'{math.degrees(j):.1f}°' for j in joint_positions]}")
        try:
            self.moveit2.move_to_configuration(joint_positions)
            if synchronous:
                ok = self.moveit2.wait_until_executed()
                if ok:
                    self.get_logger().info("Joint hareketi başarılı!")
                else:
                    self.get_logger().warn("Joint hareketi başarısız!")
                return ok
            return True
        except Exception as e:
            self.get_logger().error(f"Joint hareket hatası: {e}")
            return False

    def safe_joint_sequence(
        self,
        configs: list,
        wait_time: float = 2.0,
        max_retries: int = 3,
        agv_moves: dict = None,
    ):
        """
        Sıralı joint hareketi.  agv_moves = {point_index: target_x}
        point_index 0-tabanlı (point1 = 0, point4 = 3, ...)
        """
        for i, joints in enumerate(configs):
            self.get_logger().info(
                f"=== Point {i+1}/{len(configs)} ==="
            )

            for attempt in range(max_retries):
                self.get_logger().info(f"  Deneme {attempt+1}/{max_retries}")
                if self.move_to_joint_angles(joints):
                    break
                if attempt < max_retries - 1:
                    time.sleep(0.5)
            else:
                self.get_logger().error(
                    f"  {max_retries} deneme başarısız – nokta atlanıyor"
                )

            # Bu noktadan sonra AGV hareketi var mı?
            if agv_moves and i in agv_moves:
                target_x = agv_moves[i]
                self.get_logger().info(
                    f">>> Point {i+1} sonrası AGV → X={target_x}"
                )
                self.move_agv_to_x(target_x)
                time.sleep(2.0)

            if i < len(configs) - 1:
                time.sleep(wait_time)


# ═══════════════════════════════════════════════════════════════════
#  main
# ═══════════════════════════════════════════════════════════════════
def main():
    rclpy.init()

    ctrl = JointAGVController()

    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(ctrl)
    Thread(target=executor.spin, daemon=True).start()
    ctrl.create_rate(1.0).sleep()

    # ── Joint hedefleri (derece → radyan) ──
    home = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    point1 = [math.radians(v) for v in [15.837, -28.815, -76.154, -154.640, 31.511, 47.310]]
    point2 = [math.radians(v) for v in [-104.117, 20.814, -31.719, -290.426, 94.088, 116.370]]
    point3 = [math.radians(v) for v in [-71.503, 24.562, -25.947, -270.191, 68.745, 118.992]]
    point4 = [math.radians(v) for v in [-71.503, 0.877, -101.176, -287.050, 77.113, 172.441]]
    point5 = [math.radians(v) for v in [-71.503, 38.622, -125.932, -289.487, 98.656, 231.511]]
    point6 = [math.radians(v) for v in [-71.503, 0.877, -101.176, -287.050, 77.113, 172.441]]
    point7 = [math.radians(v) for v in [-71.503, 24.562, -25.947, -270.191, 68.745, 118.992]]
    point8 = [math.radians(v) for v in [-104.117, 20.814, -31.719, -290.426, 94.088, 116.370]]
    point9 = [math.radians(v) for v in [15.837, -28.815, -76.154, -154.640, 31.511, 47.310]]

    configs = [point1, point2, point3, point4, point5, point6, point7, point8, point9]

    # ── AGV hareketleri (0-tabanlı index → hedef X) ──
    # Point 4 (idx 3) → X=0.6
    # Point 6 (idx 5) → X=1.0
    # Point 9 (idx 8) → X=1.5
    agv_moves = {
        3: 0.6,   # point4 sonrası AGV → X=0.6
        5: 1.0,   # point6 sonrası AGV → X=1.0
        8: 1.5,   # point9 sonrası AGV → X=1.5
    }

    loop = 0
    try:
        ctrl.get_logger().info("=== JOINT + AGV DÖNGÜSÜ BAŞLIYOR ===")
        ctrl.get_logger().info("Ctrl+C ile durdurun")

        # Başlangıç: home + AGV 0.2
        ctrl.move_to_joint_angles(home)
        time.sleep(2.0)

        while rclpy.ok():
            loop += 1
            ctrl.get_logger().info(f"════ DÖNGÜ {loop} ════")

            ctrl.safe_joint_sequence(
                configs,
                wait_time=1.5,
                max_retries=3,
                agv_moves=agv_moves,
            )

            # Döngü sonu: AGV başlangıca dön → X=0.2
            ctrl.get_logger().info(">>> Döngü sonu – AGV → X=0.2 (başlangıç)")
            ctrl.move_agv_to_x(0.2)
            time.sleep(2.0)

            ctrl.get_logger().info(f"Döngü {loop} bitti. 3 s bekleniyor...")
            time.sleep(3.0)

    except KeyboardInterrupt:
        ctrl.get_logger().info(f"Durduruldu ({loop} döngü)")
    except Exception as e:
        ctrl.get_logger().error(f"Hata: {e}")
    finally:
        ctrl.get_logger().info("Güvenli kapatma: home pozisyonuna dönülüyor...")
        ctrl.move_to_joint_angles(home)
        rclpy.shutdown()


if __name__ == "__main__":
    main()
