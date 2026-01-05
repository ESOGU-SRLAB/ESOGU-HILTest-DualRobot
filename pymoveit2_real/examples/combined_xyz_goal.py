#!/usr/bin/env python3
"""
Fixed dual robot controller - Yörünge paylaşım mantığı ile güncellendi (Eklem Adı Uyuşmazlığı Düzeltildi)
"""

import copy
from threading import Thread
import time
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from pymoveit2_sim import MoveIt2 as MoveIt2_Sim
from pymoveit2_real import MoveIt2 as MoveIt2_Real
from pymoveit2_sim.robots import ur as simrobot
from pymoveit2_real.robots import ur as realrobot


class FixedDualController(Node):
    def __init__(self):
        super().__init__("fixed_dual_controller")
        
        callback_group = ReentrantCallbackGroup()
        
        self.sim_moveit = MoveIt2_Sim(
            node=self,
            joint_names=simrobot.joint_names(),
            base_link_name="world",
            end_effector_name=simrobot.end_effector_name(),
            group_name=simrobot.MOVE_GROUP_ARM,
            callback_group=callback_group,
        )
        
        self.real_moveit = MoveIt2_Real(
            node=self,
            joint_names=realrobot.joint_names(),
            base_link_name="world",
            end_effector_name=realrobot.end_effector_name(),
            group_name=realrobot.MOVE_GROUP_ARM,
            callback_group=callback_group,
        )
        
        self.sim_moveit.max_velocity = 0.05
        self.sim_moveit.max_acceleration = 0.05
        
        # ÖNEMLI: Çarpışma önleme ayarları
        self.sim_moveit.cartesian_avoid_collisions = True  # Cartesian hareketlerde çarpışma önleme
        self.sim_moveit.cartesian_jump_threshold = 2.0     # Ani sıçramaları engelle
        
        self.real_moveit.max_velocity = 0.05
        self.real_moveit.max_acceleration = 0.05
        
        # ÖNEMLI: Çarpışma önleme ayarları
        self.real_moveit.cartesian_avoid_collisions = True  # Cartesian hareketlerde çarpışma önleme
        self.real_moveit.cartesian_jump_threshold = 2.0     # Ani sıçramaları engelle
        
        self.get_logger().info("Fixed Dual Controller (Yörünge Paylaşımlı) başlatıldı")

    def synchronized_move_to_position(self, position, orientation, wait_time=0.0):
        self.get_logger().info(f"=== YÖRÜNGE PAYLAŞIMLI HAREKET: {position} ===")

        self.get_logger().info("Simülasyonda yörünge planlanıyor...")
        try:
            trajectory = self.sim_moveit.plan(
                position=position,
                quat_xyzw=orientation,
                cartesian=False
            )
            if not trajectory:
                self.get_logger().error("Yörünge planlaması BAŞARISIZ! (Boş yörünge). Hareket iptal edildi.")
                return False
        except Exception as e:
            self.get_logger().error(f"Planlama sırasında hata oluştu: {str(e)}")
            return False

        self.get_logger().info("Yörünge başarıyla planlandı. Gerçek robot için başlangıç noktası ve eklem adları güncelleniyor...")

        try:
            real_joint_state_msg = self.real_moveit.joint_state
            if not real_joint_state_msg:
                self.get_logger().error("Gerçek robotun eklem durumu alınamadı (joint_state is None)! Hareket iptal edildi.")
                return False

            real_trajectory = copy.deepcopy(trajectory)

            # =================================================================
            # !!! EN KRİTİK DÜZELTME BURADA !!!
            # Simülasyonun planladığı yörüngedeki 'sim_ur10e_' ön ekli eklem isimlerini,
            # gerçek robotun beklediği doğru eklem isimleriyle değiştiriyoruz.
            # Bu, "Joint not found" hatasını çözecektir.
            real_trajectory.joint_names = self.real_moveit.joint_names
            # =================================================================

            # Eklem sıralama mantığımız hala gerekli ve doğru.
            joint_positions_map = {name: pos for name, pos in zip(real_joint_state_msg.name, real_joint_state_msg.position)}
            expected_joint_order = self.real_moveit.joint_names
            # Gerçek robotun `joint_names` listesinde olmayan eklemleri atla
            real_current_joints_ordered = [joint_positions_map[name] for name in expected_joint_order if name in joint_positions_map]

            # Eğer tüm eklemler bulunamadıysa hata ver
            if len(real_current_joints_ordered) != len(expected_joint_order):
                self.get_logger().error("Gerçek robottan gelen joint_state mesajında tüm beklenen eklemler bulunamadı!")
                return False

            if real_trajectory.points:
                real_trajectory.points[0].positions = real_current_joints_ordered
                real_trajectory.points[0].velocities = [0.0] * len(expected_joint_order)
                real_trajectory.points[0].accelerations = [0.0] * len(expected_joint_order)
                real_trajectory.points[0].time_from_start.sec = 0
                real_trajectory.points[0].time_from_start.nanosec = 0
            else:
                self.get_logger().error("Planlanan yörüngede hiçbir nokta bulunamadı! Hareket iptal edildi.")
                return False

            self.get_logger().info("Yörünge yamandı, sıralandı ve tercüme edildi. Şimdi uygulanacak.")

            sim_thread = Thread(target=self.sim_moveit.execute, args=(trajectory,))
            real_thread = Thread(target=self.real_moveit.execute, args=(real_trajectory,))

            sim_thread.start()
            real_thread.start()
            sim_thread.join()
            real_thread.join()

            sim_success = self.sim_moveit.wait_until_executed()
            real_success = self.real_moveit.wait_until_executed()

            if sim_success and real_success:
                self.get_logger().info("=== YÖRÜNGE PAYLAŞIMLI HAREKET BAŞARILI ===")
                if wait_time > 0:
                    time.sleep(wait_time)
                return True
            else:
                self.get_logger().error(f"Hareket başarısız! Sim: {sim_success}, Real: {real_success}")
                return False

        except Exception as e:
            self.get_logger().error(f"Yörünge çalıştırılırken (veya yamalanırken) beklenmeyen hata oluştu: {str(e)}")
            return False
            
    def safe_move_sequence(self, positions_list, orientations_list=None, wait_time=2.0):
        if orientations_list is None:
            orientations_list = [[0.0, 0.0, 0.7071, 0.7071]] * len(positions_list)
        
        self.get_logger().info(f"=== PAYLAŞIMLI YÖRÜNGE SIRASI BAŞLIYOR: {len(positions_list)} pozisyon ===")
        
        success_count = 0
        
        for i, (pos, orient) in enumerate(zip(positions_list, orientations_list)):
            self.get_logger().info(f"Sıra {i+1}/{len(positions_list)}: {pos}")
            
            success = self.synchronized_move_to_position(pos, orient)
            
            if success:
                success_count += 1
                if i < len(positions_list) - 1:
                    self.get_logger().info(f"Sonraki hareket için {wait_time} saniye bekleniyor...")
                    time.sleep(wait_time)
            else:
                self.get_logger().error(f"Pozisyon {i+1} başarısız, sıra durduruluyor!")
                break
        
        self.get_logger().info(f"=== SIRA TAMAMLANDI: {success_count}/{len(positions_list)} başarılı ===")
        return success_count
    
    def move_home_safe(self):
        safe_intermediate = [-0.3, 0.3, 1.2]
        home_position = [-0.3, 0.3, 0.95]
        home_orientation = [0.0, 0.0, 0.7071, 0.7071]
        
        self.get_logger().info("Senkronize (yörünge paylaşımlı) güvenli ana pozisyona dönüş başlatılıyor...")
        
        self.get_logger().info("1. Adım: Güvenli yüksekliğe çıkılıyor...")
        success1 = self.synchronized_move_to_position(safe_intermediate, home_orientation)
        
        if success1:
            time.sleep(1.0)
            
            self.get_logger().info("2. Adım: Ana pozisyona iniliyor...")
            success2 = self.synchronized_move_to_position(home_position, home_orientation)
            
            return success2
        else:
            self.get_logger().error("Güvenli pozisyona ulaşılamadı, ana pozisyona dönüş iptal edildi.")
            return False

def main():
    rclpy.init()
    
    robot_controller = FixedDualController()
    
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(robot_controller)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()
    
    time.sleep(1.0) 
    
    safe_positions = [
        [-0.5, 1.0, 1.55],
        [-1.0, 1.0, 1.55],
        [-0.5, 1.0, 1.55],
        [-0.5, 1.65, 1.55],
        [-1.0, 1.65, 1.55],
        [-0.5, 1.65, 1.55],
        [-0.5, 1.0, 1.55]
    ]
    
    loop_counter = 0
    
    try:
        robot_controller.get_logger().info("=== SENKRONIZE (Yörünge Paylaşımlı) DÖNGÜ BAŞLATILIYOR ===")
        robot_controller.get_logger().info("Durdurmak için Ctrl+C tuşlayın")
        
        robot_controller.move_home_safe()
        time.sleep(2.0)
        
        while rclpy.ok():
            loop_counter += 1
            robot_controller.get_logger().info(f"=== DÖNGÜ {loop_counter} BAŞLIYOR ===")
            
            robot_controller.safe_move_sequence(safe_positions, wait_time=1.0)
            
            robot_controller.get_logger().info(f"Döngü {loop_counter} tamamlandı. 3 saniye bekleniyor...")
            time.sleep(3.0)
            
            if loop_counter % 10 == 0:
                robot_controller.get_logger().info(f"Bakım molası - Ana pozisyona dönülüyor (Döngü: {loop_counter})")
                robot_controller.move_home_safe()
                time.sleep(2.0)
        
    except KeyboardInterrupt:
        robot_controller.get_logger().info(f"Program kullanıcı tarafından durduruldu (Toplam {loop_counter} döngü)")
    except Exception as e:
        robot_controller.get_logger().error(f"Beklenmeyen hata: {str(e)}")
    
    finally:
        robot_controller.get_logger().info("Güvenli kapatılıyor...")
        rclpy.shutdown()
        executor_thread.join()

if __name__ == "__main__":
    main()