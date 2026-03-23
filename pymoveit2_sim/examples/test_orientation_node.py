#!/usr/bin/env python3
"""
test_orientation_node
=====================
Belirli bir noktaya farklı oryantasyonlarla giderek
gripper'ın hangi quaternion ile dimdik aşağıya baktığını test eder.

Kullanım:
  ros2 run pymoveit2_sim test_orientation_node
"""

from threading import Thread
import time
import math

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from pymoveit2_sim import MoveIt2 as MoveIt2_Sim
from pymoveit2_sim.robots import ur as robot


# ---- Test edilecek oryantasyonlar (xyzw) ----
ORIENTATIONS = [
    # {"name": "Q1: 180° Y   [0, 1, 0, 0]",      "q": [0.0, 1.0, 0.0, 0.0]},
    # {"name": "Q2: 180° X   [1, 0, 0, 0]",      "q": [1.0, 0.0, 0.0, 0.0]},
    # {"name": "Q3: 180° Z   [0, 0, 1, 0]",      "q": [0.0, 0.0, 1.0, 0.0]},
    # {"name": "Q4: 90° Y    [0, 0.707, 0, 0.707]", "q": [0.0, 0.7071068, 0.0, 0.7071068]},
    # {"name": "Q5: -90° Y   [0, -0.707, 0, 0.707]", "q": [0.0, -0.7071068, 0.0, 0.7071068]},
    # {"name": "Q6: 90° X    [0.707, 0, 0, 0.707]", "q": [0.7071068, 0.0, 0.0, 0.7071068]}, bu belki
    {"name": "Q7: -90° X   [-0.707, 0, 0, 0.707]", "q": [0.159, 0.677, -0.462, 0.551]},
    # {"name": "Q8: identity [0, 0, 0, 1]",      "q": [0.0, 0.0, 0.0, 1.0]},
]

TARGET_POS = [0.910, -0.216, 1.355]

HOME_JOINTS = [1.0, 0.0, -math.pi / 2, 0.0, -math.pi / 2, 0.0, 0.0]


class TestOrientationNode(Node):
    def __init__(self):
        super().__init__("test_orientation_node")

        cbg = ReentrantCallbackGroup()

        self.arm = MoveIt2_Sim(
            node=self,
            joint_names=robot.joint_names(),
            base_link_name=robot.base_link_name(),
            end_effector_name=robot.end_effector_name(),
            group_name=robot.MOVE_GROUP_ARM,
            callback_group=cbg,
        )
        self.arm.planner_id = "RRTConnectkConfigDefault"
        self.arm.allowed_planning_time = 10.0
        self.arm.num_planning_attempts = 20
        self.arm.max_velocity = 0.5
        self.arm.max_acceleration = 0.5

        self.get_logger().info("test_orientation_node hazır")

    def go_home(self):
        self.get_logger().info("=== HOME'A DÖN ===")
        self.arm.move_to_configuration(HOME_JOINTS)
        self.arm.wait_until_executed()
        time.sleep(1.0)

    def try_orientation(self, name, quat, position):
        self.get_logger().info(f"\n{'='*60}")
        self.get_logger().info(f"DENEME: {name}")
        self.get_logger().info(f"  Quaternion (xyzw): {quat}")
        self.get_logger().info(f"  Pozisyon: {position}")
        self.get_logger().info(f"{'='*60}")

        self.arm.move_to_pose(
            position=position,
            quat_xyzw=quat,
            frame_id="world",
            tolerance_position=0.01,
            tolerance_orientation=0.2,
        )
        ok = self.arm.wait_until_executed()

        if ok:
            self.get_logger().info(f"✅ {name} — BAŞARILI! Gripper'ı kontrol et.")
            self.get_logger().info("   5 saniye bekleniyor...")
            time.sleep(5.0)
        else:
            self.get_logger().warn(f"❌ {name} — BAŞARISIZ (planlama hatası)")
            time.sleep(1.0)

        return ok

    def run(self):
        time.sleep(1.0)

        self.get_logger().info("\n" + "=" * 60)
        self.get_logger().info(f"Hedef pozisyon: {TARGET_POS}")
        self.get_logger().info(f"Toplam {len(ORIENTATIONS)} oryantasyon denenecek")
        self.get_logger().info("=" * 60)

        for i, ori in enumerate(ORIENTATIONS, 1):
            self.get_logger().info(f"\n--- [{i}/{len(ORIENTATIONS)}] ---")

            # Önce HOME'a dön
            self.go_home()

            # Oryantasyonu dene
            ok = self.try_orientation(ori["name"], ori["q"], TARGET_POS)

            if ok:
                self.get_logger().info(
                    f"✅ '{ori['name']}' başarılı. "
                    "Gripper dik mi? Terminalde devam etmek için Enter'a bas."
                )

        # Bitince HOME
        self.go_home()

        self.get_logger().info("\n" + "=" * 60)
        self.get_logger().info("TÜM DENEMELER TAMAMLANDI")
        self.get_logger().info(
            "Dik olan quaternion'u cup_sort_node'da GRIPPER_DOWN_QUAT olarak ayarla."
        )
        self.get_logger().info("=" * 60)


def main():
    rclpy.init()
    node = TestOrientationNode()

    executor = rclpy.executors.MultiThreadedExecutor(4)
    executor.add_node(node)
    spin_thread = Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join()


if __name__ == "__main__":
    main()
