#!/usr/bin/env python3
"""
cup_sort_node  (2 Aşamalı)
===========================
Aşama 1 — TOPLAMA:
  /detections_3d_world dinler, gelen tüm tespitleri biriktirir.
  Aynı bardağın farklı waypoint'lerden tekrar tespitini önlemek için
  mesafe eşiği ile duplike temizleme yapar.
  /scan_complete sinyali gelene kadar toplamaya devam eder.

Aşama 2 — SIRALAMA:
  Biriken bardak listesini sınıfa göre tek tek pick-and-place yapar.
  intact → WP_INTACT, damaged → WP_DAMAGED
"""

from threading import Thread, Lock, Event
import time
import math

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from std_msgs.msg import Bool

from pymoveit2_sim import MoveIt2 as MoveIt2_Sim
from pymoveit2_sim import MoveIt2Gripper
from pymoveit2_sim.robots import ur as robot
from yolo_msgs.msg import DetectionArray


class CupSortNode(Node):
    """2 aşamalı bardak sıralama: önce topla, sonra pick-and-place."""

    # Aynı bardak sayma mesafe eşiği (metre)
    DEDUP_DISTANCE = 0.15
    # Gripper yönelimi: tool0 Z-ekseni world -Z (aşağı)
    # tf2_echo world sim_ur10e_tool0 çıktısından alınan değer
    GRIPPER_DOWN_QUAT = [0.363, 0.609, -0.587, 0.390]

    # tool0 → gripper kavrama merkezi offset (tool0 frame'inde)
    # tf2_echo sim_ur10e_tool0 sim_ur10e_gripper_upper_1 → [0.142, 0.102, +0.049]
    # tf2_echo sim_ur10e_tool0 sim_ur10e_gripper_lower_1 → [0.142, 0.102, -0.049]
    # Kavrama merkezi (ortalaması): [0.142, 0.102, 0.0]
    TCP_OFFSET_IN_TOOL0 = [0.142, 0.102, 0.0]

    @staticmethod
    def _quat_rotate(quat_xyzw, vec):
        """Quaternion [x,y,z,w] ile 3D vektörü döndür → world frame offset."""
        x, y, z, w = quat_xyzw
        # Rotation matrix from quaternion
        r00 = 1 - 2*(y*y + z*z)
        r01 = 2*(x*y - z*w)
        r02 = 2*(x*z + y*w)
        r10 = 2*(x*y + z*w)
        r11 = 1 - 2*(x*x + z*z)
        r12 = 2*(y*z - x*w)
        r20 = 2*(x*z - y*w)
        r21 = 2*(y*z + x*w)
        r22 = 1 - 2*(x*x + y*y)
        return [
            r00*vec[0] + r01*vec[1] + r02*vec[2],
            r10*vec[0] + r11*vec[1] + r12*vec[2],
            r20*vec[0] + r21*vec[1] + r22*vec[2],
        ]



    def __init__(self):
        super().__init__("cup_sort_node")

        cbg = ReentrantCallbackGroup()

        # --- Arm ---
        self.arm = MoveIt2_Sim(
            node=self,
            joint_names=robot.joint_names(),
            base_link_name=robot.base_link_name(),
            end_effector_name=robot.end_effector_name(),
            group_name=robot.MOVE_GROUP_ARM,
            callback_group=cbg,
        )
        self.arm.planner_id = "ESTkConfigDefault"
        self.arm.allowed_planning_time = 10.0  # varsayılan 0.5s çok kısa
        self.arm.num_planning_attempts = 20    # varsayılan 5 yetersiz
        self.arm.max_velocity = 0.2
        self.arm.max_acceleration = 0.2
        # --- Gripper ---
        gj = robot.gripper_joint_names()
        gripper_joint_names = gj if isinstance(gj, (list, tuple)) else [gj]

        self.gripper = MoveIt2Gripper(
            node=self,
            gripper_joint_names=gripper_joint_names,
            open_gripper_joint_positions=robot.OPEN_GRIPPER_JOINT_POSITIONS,
            closed_gripper_joint_positions=robot.CLOSED_GRIPPER_JOINT_POSITIONS,
            gripper_group_name=robot.MOVE_GROUP_GRIPPER,
            callback_group=cbg,
        )

        # --- Waypoints (7 eleman, joint sırası) ---
        self.HOME = [1.0, 0.0, -math.pi / 2, 0.0, -math.pi / 2, 0.0, 0.0]

        # intact bardaklar buraya bırakılır
        self.WP_INTACT = [
            1.08721,
            0.007539822368616278,
            -0.84948665353068,
            0.7979645340118076,
            1.3848140417023806,
            0.5240176546187767,
            0.15833626974092496,
        ]

        # damaged bardaklar buraya bırakılır
        self.WP_DAMAGED = [
            0.05,
            0.5881061447520084,
            -0.9475043443226818,
            1.1378848591302235,
            -2.789734276387736,
            -1.5959290680236151,
            -1.5959290680236151,
        ]

        # --- Toplama aşaması verileri ---
        self._cup_list_lock = Lock()
        # Her eleman: {"class_name": str, "x": float, "y": float, "z": float,
        #              "score": float, "count": int}
        self._cup_list = []
        self._scan_complete = Event()

        # --- Subscriber'lar ---
        self.create_subscription(
            DetectionArray,
            "/detections_3d_world",
            self._detection_cb,
            10,
        )
        self.create_subscription(
            Bool,
            "/scan_complete",
            self._scan_complete_cb,
            10,
        )

        self.get_logger().info(
            "cup_sort_node hazır — Aşama 1: tarama tespitlerini topluyorum..."
        )

    # ------------------------------------------------------------------
    # Callback'ler
    # ------------------------------------------------------------------
    def _detection_cb(self, msg: DetectionArray):
        """Gelen tespitleri biriktirir (Aşama 1). scan_complete'e kadar tüm tespitler kabul edilir."""
        if self._scan_complete.is_set():
            return

        if not msg.detections:
            return

        with self._cup_list_lock:
            for det in msg.detections:
                bx = det.bbox3d.center.position.x
                by = det.bbox3d.center.position.y
                bz = det.bbox3d.center.position.z
                score = det.score
                cls = det.class_name

                # Geçerlilik kontrolü
                if by > 0.0 or bz < 0.75:
                    self.get_logger().debug(
                        f"Filtrelendi [{cls}] ({bx:.3f}, {by:.3f}, {bz:.3f}): "
                        f"by={by:.3f} > 0.0 veya bz={bz:.3f} < 0.75",
                        throttle_duration_sec=2.0,
                    )
                    continue

                # Duplike kontrolü: mevcut listedeki en yakın bardak
                merged = False
                for cup in self._cup_list:
                    dist = math.sqrt(
                        (cup["x"] - bx) ** 2
                        + (cup["y"] - by) ** 2
                        + (cup["z"] - bz) ** 2
                    )
                    if dist < self.DEDUP_DISTANCE:
                        # Aynı bardak — ilk pozisyonu koru, sadece sayacı artır
                        cup["count"] += 1
                        # Sınıfı en yüksek skorlu tespitten al
                        if score > cup["score"]:
                            cup["score"] = score
                            cup["class_name"] = cls
                        merged = True
                        break

                if not merged:
                    self._cup_list.append({
                        "class_name": cls,
                        "x": bx,
                        "y": by,
                        "z": bz,
                        "score": score,
                        "count": 1,
                    })
                    self.get_logger().info(
                        f"Yeni bardak #{len(self._cup_list)}: [{cls}] "
                        f"({bx:.3f}, {by:.3f}, {bz:.3f})"
                    )

    def _scan_complete_cb(self, msg: Bool):
        """cup_detection_node taramayı bitirdiğinde sinyali alır."""
        if msg.data and not self._scan_complete.is_set():
            self._scan_complete.set()
            with self._cup_list_lock:
                n = len(self._cup_list)
            self.get_logger().info(
                f"✅ /scan_complete alındı! Toplam {n} benzersiz bardak tespit edildi."
            )

    # ------------------------------------------------------------------
    # Arm yardımcıları
    # ------------------------------------------------------------------
    def move_arm_joints(self, joints, label, retries=3, sleep_between=0.4):
        for i in range(1, retries + 1):
            self.get_logger().info(f"=== ARM: {label} (try {i}/{retries}) ===")
            self.arm.move_to_configuration(joints)
            ok = self.arm.wait_until_executed()
            self.get_logger().info("ARM done ✅" if ok else "ARM fail ❌")
            if ok:
                return True
            time.sleep(sleep_between)
        return False

    def move_arm_pose(self, position, quat_xyzw, label, retries=20,
                      cartesian=False, sleep_between=0.4):
        for i in range(1, retries + 1):
            self.get_logger().info(
                f"=== ARM POSE: {label} (try {i}/{retries}) "
                f"pos=[{position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f}] ==="
            )

            self.arm.move_to_pose(
                position=position,
                quat_xyzw=quat_xyzw,
                frame_id="world",
                cartesian=cartesian,
                tolerance_position=0.005,
                tolerance_orientation=0.2,
            )
            ok = self.arm.wait_until_executed()
            self.get_logger().info("ARM POSE done ✅" if ok else "ARM POSE fail ❌")
            if ok:
                return True
            time.sleep(sleep_between)
        return False

    def gripper_open(self):
        self.get_logger().info("=== GRIPPER OPEN ===")
        self.gripper.open()
        ok = self.gripper.wait_until_executed()
        time.sleep(0.3)
        return ok

    def gripper_close(self):
        self.get_logger().info("=== GRIPPER CLOSE ===")
        self.gripper.close()
        ok = self.gripper.wait_until_executed()
        time.sleep(0.3)
        return ok

    # ------------------------------------------------------------------
    # Ana döngü (ayrı thread'de çalışır)
    # ------------------------------------------------------------------
    def run_loop(self):
        """2 aşamalı ana döngü."""

        # ====== AŞAMA 1: TOPLAMA ======
        self.get_logger().info("=" * 50)
        self.get_logger().info("AŞAMA 1: Tarama tespitleri toplanıyor...")
        self.get_logger().info("  cup_detection_node'un taramayı bitirmesini bekliyorum.")
        self.get_logger().info("=" * 50)

        while rclpy.ok() and not self._scan_complete.is_set():
            self._scan_complete.wait(timeout=2.0)
            with self._cup_list_lock:
                n = len(self._cup_list)
            if not self._scan_complete.is_set():
                self.get_logger().info(
                    f"Tarama devam ediyor... şu ana kadar {n} bardak.",
                    throttle_duration_sec=10.0,
                )

        # Tarama bitti — listeyi al
        with self._cup_list_lock:
            cups = list(self._cup_list)

        if not cups:
            self.get_logger().warn("Hiç bardak tespit edilmedi! Çıkılıyor.")
            return

        # Özet yazdır
        intact_cups = [c for c in cups if c["class_name"] == "intact"]
        damaged_cups = [c for c in cups if c["class_name"] == "damaged"]

        self.get_logger().info("=" * 50)
        self.get_logger().info(
            f"AŞAMA 2: SIRALAMA — {len(cups)} bardak "
            f"({len(intact_cups)} intact, {len(damaged_cups)} damaged)"
        )
        for i, c in enumerate(cups, 1):
            self.get_logger().info(
                f"  #{i} [{c['class_name']}] "
                f"({c['x']:.3f}, {c['y']:.3f}, {c['z']:.3f}) "
                f"score={c['score']:.2f} görülme={c['count']}"
            )
        self.get_logger().info("=" * 50)

        # ====== Bardakları collision object olarak ekle ======
        for i, c in enumerate(cups, 1):
            cid = f"cup_{i}"
            self.arm.add_collision_cylinder(
                id=cid,
                height=0.10,
                radius=0.04,
                position=[c["x"], c["y"], c["z"]],
                quat_xyzw=[0.0, 0.0, 0.0, 1.0],
                frame_id="world",
            )
            self.get_logger().info(
                f"Collision eklendi: {cid} → "
                f"({c['x']:.3f}, {c['y']:.3f}, {c['z']:.3f})"
            )
        time.sleep(1.0)  # planning scene güncellenmesi için bekle

        # ====== AŞAMA 2: SIRALAMA (pick-and-place) ======
        self.gripper_open()

        success_count = 0
        fail_count = 0

        for i, cup in enumerate(cups, 1):
            self.get_logger().info(
                f"\n--- Bardak {i}/{len(cups)}: [{cup['class_name']}] "
                f"({cup['x']:.3f}, {cup['y']:.3f}, {cup['z']:.3f}) ---"
            )

            cid = f"cup_{i}"
            self.arm.remove_collision_object(cid)
            self.get_logger().info(f"Collision kaldırıldı: {cid}")
            time.sleep(0.5)

            ok = self._pick_and_place(cup["x"], cup["y"], cup["z"], cup["class_name"])
            if ok:
                success_count += 1
                self.get_logger().info(
                    f"✅ Bardak {i}/{len(cups)} [{cup['class_name']}] başarılı."
                )
            else:
                fail_count += 1
                self.get_logger().error(
                    f"❌ Bardak {i}/{len(cups)} [{cup['class_name']}] başarısız! "
                    "Devam ediliyor..."
                )
                self.gripper_open()

        # Tüm pick-place bitti → HOME'a dön
        self.move_arm_joints(self.HOME, "HOME_FINAL")

        # Son özet
        self.get_logger().info("=" * 50)
        self.get_logger().info(
            f"TAMAMLANDI: {success_count}/{len(cups)} bardak yerleştirildi "
            f"({fail_count} başarısız)."
        )
        self.get_logger().info("=" * 50)



    def _pick_and_place(self, cup_x, cup_y, cup_z, class_name) -> bool:
        """Tek bir bardağ için pick-and-place dizisi."""
        quat = self.GRIPPER_DOWN_QUAT

        # tool0'ı öyle konumlandır ki gripper kavrama merkezi bardağa gelsin
        # TCP offset'i gerçek quaternion ile world frame'e dönüştür
        tcp_world = self._quat_rotate(quat, self.TCP_OFFSET_IN_TOOL0)
        pick_x = cup_x - tcp_world[0]
        pick_y = cup_y - tcp_world[1]
        pick_z = cup_z - tcp_world[2]

        self.get_logger().info(
            f"Bardak: ({cup_x:.3f}, {cup_y:.3f}, {cup_z:.3f}) → "
            f"TCP offset world: ({tcp_world[0]:.3f}, {tcp_world[1]:.3f}, {tcp_world[2]:.3f}) → "
            f"tool0 hedef: ({pick_x:.3f}, {pick_y:.3f}, {pick_z:.3f})"
        )

        # 1. 30cm yukarıdan yaklaş (güvenli yaklaşım)
        # pos_30cm = [pick_x, pick_y, pick_z + 0.30]
        # if not self.move_arm_pose(pos_30cm, quat, "APPROACH_30CM"):
        #     return False

        # 2. 10cm yukarıya in (ince yaklaşım)
        pos_10cm = [pick_x, pick_y, pick_z + 0.05]
        if not self.move_arm_pose(pos_10cm, quat, "APPROACH_5CM"):
            return False

        # 3. Gripper kapat (kavra)
        self.gripper_close()
        time.sleep(0.5)

        # 4. Sınıfa göre hedef waypoint'e git
        if class_name == "intact":
            target_wp = self.WP_INTACT
            target_label = "WP_INTACT"
        else:
            target_wp = self.WP_DAMAGED
            target_label = "WP_DAMAGED"

        self.get_logger().info(f"Sınıf: {class_name} → {target_label}")
        if not self.move_arm_joints(target_wp, target_label):
            return False

        # 5. Gripper aç (bırak)
        self.gripper_open()
        time.sleep(0.3)

        return True


def main():
    rclpy.init()
    node = CupSortNode()

    executor = rclpy.executors.MultiThreadedExecutor(4)
    executor.add_node(node)
    spin_thread = Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    time.sleep(1.0)

    try:
        node.run_loop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join()


if __name__ == "__main__":
    main()
