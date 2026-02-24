#!/usr/bin/env python3
"""
MoveIt2 ile grippera komut gönderen ROS2 node'u.
Gripper'ı 0.01 pozisyonuna hareket ettirir.
"""

from threading import Thread
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from pymoveit2_sim import MoveIt2Gripper
from pymoveit2_sim.robots import ur as robot


class GripperController(Node):
    def __init__(self):
        super().__init__("sim_gripper_controller")

        # Callback group
        callback_group = ReentrantCallbackGroup()

        # MoveIt2Gripper arayüzünü oluştur
        self.gripper = MoveIt2Gripper(
            node=self,
            gripper_joint_names=[robot.gripper_joint_names()],
            open_gripper_joint_positions=robot.OPEN_GRIPPER_JOINT_POSITIONS,
            closed_gripper_joint_positions=robot.CLOSED_GRIPPER_JOINT_POSITIONS,
            gripper_group_name=robot.MOVE_GROUP_GRIPPER,
            callback_group=callback_group,
        )

        self.get_logger().info("Gripper kontrolcüsü başlatıldı")

    def move_gripper_to(self, position: float, synchronous: bool = True):
        """
        Gripper'ı belirtilen pozisyona hareket ettirir.
        
        Args:
            position: Hedef gripper pozisyonu (metre cinsinden)
            synchronous: Hareketin bitmesini bekle
        """
        self.get_logger().info(f"Gripper {position} pozisyonuna gönderiliyor...")

        try:
            self.gripper.move_to_position(position)

            if synchronous:
                success = self.gripper.wait_until_executed()
                if success:
                    self.get_logger().info(
                        f"Gripper başarıyla {position} pozisyonuna ulaştı!"
                    )
                else:
                    self.get_logger().warn("Gripper hareketi tamamlanamadı!")
                return success
        except Exception as e:
            self.get_logger().error(f"Gripper hareket hatası: {str(e)}")
            return False

    def open_gripper(self, synchronous: bool = True):
        """Gripper'ı aç."""
        self.get_logger().info("Gripper açılıyor...")
        self.gripper.open()
        if synchronous:
            return self.gripper.wait_until_executed()

    def close_gripper(self, synchronous: bool = True):
        """Gripper'ı kapat."""
        self.get_logger().info("Gripper kapatılıyor...")
        self.gripper.close()
        if synchronous:
            return self.gripper.wait_until_executed()


def main():
    rclpy.init()

    gripper_controller = GripperController()

    # Multi-threaded executor
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(gripper_controller)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()
    gripper_controller.create_rate(1.0).sleep()

    try:
        gripper_controller.get_logger().info(
            "=== GRİPPER KONTROL BAŞLATILIYOR ==="
        )

        # Sıralı gripper pozisyonları
        positions = [0.005, 0.01, 0.015, 0.02, 0.025]

        for i, pos in enumerate(positions):
            gripper_controller.get_logger().info(
                f"--- Adım {i+1}/{len(positions)}: {pos} pozisyonuna gidiliyor ---"
            )
            gripper_controller.move_gripper_to(pos)

            # Son pozisyon değilse 10 saniye bekle
            if i < len(positions) - 1:
                gripper_controller.get_logger().info("10 saniye bekleniyor...")
                time.sleep(10.0)

        gripper_controller.get_logger().info("Tüm gripper komutları tamamlandı.")

    except KeyboardInterrupt:
        gripper_controller.get_logger().info("Program kullanıcı tarafından durduruldu")
    except Exception as e:
        gripper_controller.get_logger().error(f"Beklenmeyen hata: {str(e)}")
    finally:
        gripper_controller.get_logger().info("Kapatılıyor...")
        rclpy.shutdown()
        executor_thread.join()


if __name__ == "__main__":
    main()
