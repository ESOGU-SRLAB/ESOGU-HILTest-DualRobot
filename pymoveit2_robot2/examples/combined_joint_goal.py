#!/usr/bin/env python3
"""
İki robotu eş zamanlı kontrol eden çarpışma önleyici sistem - Geliştirilmiş versiyon
"""

from threading import Thread, Lock
import time
import numpy as np

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.node import Node

from pymoveit2 import MoveIt2, MoveIt2State
# Combined robot config'inizi import edin
from pymoveit2.robots import combined_ur as robot


class DualRobotCollisionController(Node):
    def __init__(self):
        super().__init__("dual_robot_collision_controller")
        
        # Farklı callback group'ları kullanarak thread güvenliği sağla
        self.callback_group_combined = ReentrantCallbackGroup()
        self.callback_group_robot1 = MutuallyExclusiveCallbackGroup()
        self.callback_group_robot2 = MutuallyExclusiveCallbackGroup()
        
        # Thread güvenliği için lock'lar
        self.robot1_lock = Lock()
        self.robot2_lock = Lock()
        self.combined_lock = Lock()

        # SADECE Combined planning group kullan (en güvenilir yöntem)
        self.moveit2_combined = MoveIt2(
            node=self,
            joint_names=robot.joint_names(),  # Tüm joint'ler (robot1 + robot2)
            base_link_name=robot.base_link_name(),
            end_effector_name=robot.end_effector_name(),
            group_name=robot.MOVE_GROUP_ARM,  # "combined"
            callback_group=self.callback_group_combined,
        )
        
        # Güvenlik ayarları
        self.setup_safety_parameters(self.moveit2_combined)
        
        self.get_logger().info("İki robotlu sistem başlatıldı - SADECE COMBINED GROUP kullanılıyor")
        
    def setup_safety_parameters(self, moveit2_interface):
        """Güvenlik parametrelerini ayarla"""
        moveit2_interface.planner_id = "RRTstarkConfigDefault"
        moveit2_interface.max_velocity = 0.15  # Daha yavaş
        moveit2_interface.max_acceleration = 0.15
        moveit2_interface.cartesian_avoid_collisions = True
        moveit2_interface.cartesian_jump_threshold = 2.0
    
    def move_robots_to_joint_positions(self, robot1_joints, robot2_joints, synchronous=True):
        """
        İki robotu joint pozisyonlarına eş zamanlı hareket ettir - EN GÜVENİLİR YÖNTEM
        
        Args:
            robot1_joints (list): Robot1 için 6 joint değeri (radyan)
            robot2_joints (list): Robot2 için 6 joint değeri (radyan)
        """
        if len(robot1_joints) != 6 or len(robot2_joints) != 6:
            self.get_logger().error("Her robot için 6 joint değeri gerekli!")
            return False
            
        # Combined joint pozisyonu (robot1 + robot2) - SIRALAMANIZ ÖNEMLİ!
        combined_joints = robot1_joints + robot2_joints
        
        self.get_logger().info(f"Eş zamanlı joint hareketi:")
        self.get_logger().info(f"  Robot1 joints: {[round(j, 3) for j in robot1_joints]}")
        self.get_logger().info(f"  Robot2 joints: {[round(j, 3) for j in robot2_joints]}")
        
        try:
            with self.combined_lock:
                self.moveit2_combined.move_to_configuration(combined_joints)
                
                if synchronous:
                    success = self.moveit2_combined.wait_until_executed()
                    if success:
                        self.get_logger().info("✅ Eş zamanlı joint hareketi BAŞARILI!")
                        return True
                    else:
                        self.get_logger().error("❌ Eş zamanlı joint hareketi BAŞARISIZ!")
                        return False
                        
        except Exception as e:
            self.get_logger().error(f"Joint hareket hatası: {str(e)}")
            return False
            
        return True
    
    def get_current_joint_positions(self):
        """Mevcut joint pozisyonlarını al"""
        try:
            current_joints = self.moveit2_combined.joint_state
            if current_joints and len(current_joints) >= 12:  # 6 + 6 joint
                robot1_joints = current_joints[:6]
                robot2_joints = current_joints[6:12]
                return robot1_joints, robot2_joints
            else:
                self.get_logger().warn("Joint state alınamadı, varsayılan pozisyonlar kullanılıyor")
                return [0.0] * 6, [0.0] * 6
        except Exception as e:
            self.get_logger().error(f"Joint state hatası: {str(e)}")
            return [0.0] * 6, [0.0] * 6
    
    def interpolate_joint_motion(self, start_joints1, start_joints2, 
                                end_joints1, end_joints2, steps=10):
        """Joint hareketi için interpolasyon - daha yumuşak hareket"""
        start1 = np.array(start_joints1)
        start2 = np.array(start_joints2)
        end1 = np.array(end_joints1)
        end2 = np.array(end_joints2)
        
        for i in range(steps + 1):
            alpha = i / steps
            
            # Linear interpolation
            current1 = start1 + alpha * (end1 - start1)
            current2 = start2 + alpha * (end2 - start2)
            
            success = self.move_robots_to_joint_positions(
                current1.tolist(), current2.tolist(), synchronous=True
            )
            
            if not success:
                self.get_logger().warn(f"Interpolasyon adımı {i} başarısız!")
                return False
                
            time.sleep(0.2)  # Adımlar arası kısa bekleme
            
        return True
    
    def predefined_joint_positions(self):
        """Önceden tanımlanmış güvenli joint pozisyonları"""
        return {
            "home": {
                "robot1": [0.0, -1.57, 0.0, -1.57, 0.0, 0.0],  # Tipik home pozisyonu
                "robot2": [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
            },
            "ready": {
                "robot1": [-0.5, -1.2, -0.5, -1.8, 0.0, 0.0],
                "robot2": [0.5, -1.2, 0.5, -1.8, 0.0, 0.0]
            },
            "work_pose1": {
                "robot1": [-0.8, -1.0, -1.0, -2.0, 0.0, 0.0],
                "robot2": [0.8, -1.0, 1.0, -2.0, 0.0, 0.0]
            },
            "work_pose2": {
                "robot1": [-1.2, -0.8, -1.2, -1.5, 0.0, 0.0],
                "robot2": [1.2, -0.8, 1.2, -1.5, 0.0, 0.0]
            },
            "meet_pose": {  # Robotlar birbirine yakın (DİKKATLİ!)
                "robot1": [-0.2, -1.2, -0.3, -1.8, 0.0, 0.0],
                "robot2": [0.2, -1.2, 0.3, -1.8, 0.0, 0.0]
            }
        }
    
    def coordinated_sequence_joints(self):
        """Joint bazlı koordineli sekans - EN GÜVENİLİR"""
        self.get_logger().info("=== JOİNT BAZLI KOORDİNELİ SEKANS ===")
        
        poses = self.predefined_joint_positions()
        sequence = ["home", "ready", "work_pose1", "work_pose2", "meet_pose", "ready", "home"]
        
        current_robot1, current_robot2 = self.get_current_joint_positions()
        
        for i, pose_name in enumerate(sequence):
            self.get_logger().info(f"Adım {i+1}/{len(sequence)}: {pose_name}")
            
            target_robot1 = poses[pose_name]["robot1"]
            target_robot2 = poses[pose_name]["robot2"]
            
            # Interpolasyonlu hareket (daha güvenli)
            success = self.interpolate_joint_motion(
                current_robot1, current_robot2,
                target_robot1, target_robot2,
                steps=5
            )
            
            if success:
                current_robot1 = target_robot1
                current_robot2 = target_robot2
                self.get_logger().info(f"✅ {pose_name} pozisyonu başarılı")
            else:
                self.get_logger().error(f"❌ {pose_name} pozisyonu başarısız!")
                break
                
            time.sleep(1.0)
    
    def collaborative_pickup_sequence(self):
        """İşbirlikçi alma sekansı - joint bazlı"""
        self.get_logger().info("=== İŞBİRLİKÇİ ALMA SEKANSı ===")
        
        # Özel pozisyonlar - obje alma simulasyonu
        pickup_sequence = [
            # Başlangıç
            ([0.0, -1.57, 0.0, -1.57, 0.0, 0.0], [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]),
            
            # Robot1 objeye yaklaşır, Robot2 bekler
            ([-0.8, -1.2, -0.8, -1.5, 0.0, 0.0], [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]),
            
            # Robot1 objeyi alır (aşağı iner)
            ([-0.8, -1.4, -0.6, -1.3, 0.0, 0.0], [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]),
            
            # Robot1 objeyi kaldırır, Robot2 transfer pozisyonuna
            ([-0.5, -1.2, -0.8, -1.6, 0.0, 0.0], [0.3, -1.3, 0.5, -1.7, 0.0, 0.0]),
            
            # Transfer - robotlar yaklaşır
            ([-0.2, -1.1, -0.5, -1.8, 0.0, 0.0], [0.2, -1.1, 0.5, -1.8, 0.0, 0.0]),
            
            # Robot2 objeyi alır, Robot1 geri çekilir
            ([-0.5, -1.3, -0.8, -1.5, 0.0, 0.0], [0.6, -1.2, 0.8, -1.6, 0.0, 0.0]),
            
            # Final - ana pozisyonlara dönüş
            ([0.0, -1.57, 0.0, -1.57, 0.0, 0.0], [0.0, -1.57, 0.0, -1.57, 0.0, 0.0])
        ]
        
        for i, (joints1, joints2) in enumerate(pickup_sequence):
            self.get_logger().info(f"İşbirlikçi adım {i+1}/{len(pickup_sequence)}")
            
            success = self.move_robots_to_joint_positions(joints1, joints2, synchronous=True)
            
            if success:
                self.get_logger().info(f"✅ Adım {i+1} başarılı")
            else:
                self.get_logger().error(f"❌ Adım {i+1} başarısız!")
                break
                
            time.sleep(2.0)  # İşlem zamanı
    
    def move_robots_home_safe(self):
        """Güvenli ana pozisyona dönüş"""
        self.get_logger().info("İki robot da güvenli ana pozisyona dönüyor...")
        
        home_joints1 = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
        home_joints2 = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
        
        # Mevcut pozisyondan interpolasyonlu hareket
        current_robot1, current_robot2 = self.get_current_joint_positions()
        
        return self.interpolate_joint_motion(
            current_robot1, current_robot2,
            home_joints1, home_joints2,
            steps=8
        )
    
    def emergency_stop(self):
        """Acil durma"""
        self.get_logger().warn("🚨 ACİL DURMA AKTİF!")
        try:
            with self.combined_lock:
                self.moveit2_combined.stop()
        except Exception as e:
            self.get_logger().error(f"Acil durma hatası: {str(e)}")


def main():
    rclpy.init()
    
    dual_robot_controller = DualRobotCollisionController()
    
    # Tek executor kullan - daha stabil
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(dual_robot_controller)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()
    
    # Başlangıç için bekleme
    time.sleep(3.0)
    
    try:
        dual_robot_controller.get_logger().info("=== İKİ ROBOTLU SİSTEM BAŞLATILIYOR ===")
        dual_robot_controller.get_logger().info("SADECE COMBINED GROUP ve JOİNT HAREKETLERİ kullanılıyor")
        
        # Ana pozisyona git
        dual_robot_controller.move_robots_home_safe()
        time.sleep(2.0)
        
        # Joint bazlı koordineli sekans
        dual_robot_controller.coordinated_sequence_joints()
        time.sleep(2.0)
        
        # İşbirlikçi görev
        dual_robot_controller.collaborative_pickup_sequence()
        time.sleep(2.0)
        
        # Infinite loop
        loop_counter = 0
        while rclpy.ok():
            loop_counter += 1
            dual_robot_controller.get_logger().info(f"=== DÖNGÜ {loop_counter} ===")
            
            dual_robot_controller.coordinated_sequence_joints()
            time.sleep(2.0)
            
            dual_robot_controller.collaborative_pickup_sequence()
            time.sleep(3.0)
            
            # Her 3 döngüde bir ana pozisyona dön
            if loop_counter % 3 == 0:
                dual_robot_controller.move_robots_home_safe()
                time.sleep(2.0)
        
    except KeyboardInterrupt:
        dual_robot_controller.get_logger().info("Program kullanıcı tarafından durduruldu")
    except Exception as e:
        dual_robot_controller.get_logger().error(f"Beklenmeyen hata: {str(e)}")
    
    finally:
        dual_robot_controller.get_logger().info("Güvenli kapatılıyor...")
        dual_robot_controller.move_robots_home_safe()
        time.sleep(1.0)
        rclpy.shutdown()
        executor_thread.join()


if __name__ == "__main__":
    main()
