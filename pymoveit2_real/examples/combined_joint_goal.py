#!/usr/bin/env python3
"""
Dual robot controller for synchronized JOINT ANGLE movements.
(Yörünge paylaşım mantığı ve yeniden deneme mekanizması ile)
"""

import copy
from threading import Thread
import time
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
import math

# Gerekli MoveIt2 kütüphaneleri
from pymoveit2_sim import MoveIt2 as MoveIt2_Sim
from pymoveit2_real import MoveIt2 as MoveIt2_Real
from pymoveit2_sim.robots import ur as simrobot
from pymoveit2_real.robots import ur as realrobot

class FixedDualController(Node):
    def __init__(self):
        super().__init__("fixed_dual_controller_joint_retry")
        
        callback_group = ReentrantCallbackGroup()
        
        self.sim_moveit = MoveIt2_Sim(
            node=self,
            joint_names=simrobot.joint_names(),
            base_link_name="world",
            end_effector_name=simrobot.end_effector_name(),
            group_name=simrobot.MOVE_GROUP_ARM,
            callback_group=callback_group,
        )
        self.sim_moveit.planner_id = "RRTConnectkConfigDefault"
        self.real_moveit = MoveIt2_Real(
            node=self,
            joint_names=realrobot.joint_names(),
            base_link_name="world",
            end_effector_name=realrobot.end_effector_name(),
            group_name=realrobot.MOVE_GROUP_ARM,
            callback_group=callback_group,
        )
        self.real_moveit.planner_id = "RRTConnectkConfigDefault"
        self.sim_moveit.max_velocity = 0.07
        self.sim_moveit.max_acceleration = 0.07
        self.real_moveit.max_velocity = 0.05
        self.real_moveit.max_acceleration = 0.05
        
        self.get_logger().info("Fixed Dual Controller (Yeniden Denemeli) başlatıldı")

    def synchronized_move_to_joints(self, joint_positions, wait_time=0.0):
        # BU FONKSİYON DEĞİŞMEDİ
        self.get_logger().info(f"=== YÖRÜNGE PAYLAŞIMLI EKLEM HAREKETİ BAŞLIYOR ===")

        self.get_logger().info("Simülasyonda eklem hedefi için yörünge planlanıyor...")
        try:
            trajectory = self.sim_moveit.plan(joint_positions=joint_positions)
            if not trajectory:
                self.get_logger().error("Eklem hedefi için yörünge planlaması BAŞARISIZ! Hareket iptal.")
                return False
        except Exception as e:
            self.get_logger().error(f"Eklem hedefi planlaması sırasında hata: {str(e)}")
            return False

        self.get_logger().info("Yörünge planlandı. Gerçek robot için yamalama başlıyor...")

        try:
            real_joint_state_msg = self.real_moveit.joint_state
            if not real_joint_state_msg:
                self.get_logger().error("Gerçek robotun eklem durumu alınamadı! Hareket iptal.")
                return False

            real_trajectory = copy.deepcopy(trajectory)
            real_trajectory.joint_names = self.real_moveit.joint_names
            
            joint_positions_map = {name: pos for name, pos in zip(real_joint_state_msg.name, real_joint_state_msg.position)}
            expected_joint_order = self.real_moveit.joint_names
            real_current_joints_ordered = [joint_positions_map[name] for name in expected_joint_order if name in joint_positions_map]

            if len(real_current_joints_ordered) != len(expected_joint_order):
                self.get_logger().error("Gerçek robottan beklenen tüm eklemler bulunamadı!")
                return False

            if real_trajectory.points:
                real_trajectory.points[0].positions = real_current_joints_ordered
                real_trajectory.points[0].velocities = [0.0] * len(expected_joint_order)
                real_trajectory.points[0].accelerations = [0.0] * len(expected_joint_order)
                real_trajectory.points[0].time_from_start.sec = 0
                real_trajectory.points[0].time_from_start.nanosec = 0
            else:
                self.get_logger().error("Planlanan yörüngede nokta bulunamadı! Hareket iptal.")
                return False

            self.get_logger().info("Yörünge yamandı. Eş zamanlı uygulanacak.")

            sim_thread = Thread(target=self.sim_moveit.execute, args=(trajectory,))
            real_thread = Thread(target=self.real_moveit.execute, args=(real_trajectory,))

            sim_thread.start()
            real_thread.start()
            sim_thread.join()
            real_thread.join()

            sim_success = self.sim_moveit.wait_until_executed()
            real_success = self.real_moveit.wait_until_executed()

            if sim_success and real_success:
                self.get_logger().info("=== YÖRÜNGE PAYLAŞIMLI EKLEM HAREKETİ BAŞARILI ===")
                if wait_time > 0:
                    time.sleep(wait_time)
                return True
            else:
                self.get_logger().error(f"Eklem hareketi başarısız! Sim: {sim_success}, Real: {real_success}")
                return False
        except Exception as e:
            self.get_logger().error(f"Yörünge çalıştırılırken hata oluştu: {str(e)}")
            return False

# --- ANA UYGULAMA DÖNGÜSÜ (YENİDEN DENEME MANTIĞI İLE) ---
def main():
    rclpy.init()
    
    robot_controller = FixedDualController()
    
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(robot_controller)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()
    
    time.sleep(2.0)
    
    home_joints = [0.0, -math.pi/2, math.pi/2, -math.pi/2, -math.pi/2, 0.0]
    pose2_joints = [math.radians(-25.91), math.radians(-112.9), math.radians(-58.29), math.radians(-20.6), math.radians(122.96), math.radians(61.53)]
    pose3_joints = [math.radians(-30.16), math.radians(-144.62), math.radians(-104.24), math.radians(56.34), math.radians(126.94), math.radians(60.25)]
    pose4_joints = [math.radians(28.29), math.radians(-112.9), math.radians(-60.63), math.radians(-15.19), math.radians(65.06), math.radians(61.53)]
    pose5_joints = [math.radians(36.19), math.radians(-140.42), math.radians(-124.24), math.radians(71.80), math.radians(61.68), math.radians(73.15)]
    pose6_joints = [math.radians(-2.01), math.radians(-64.85), math.radians(-131.44), math.radians(7.79), math.radians(98.62), math.radians(65.41)]
    pose7_joints = [math.radians(-35), math.radians(-125), math.radians(-133), math.radians(264), math.radians(-120), math.radians(88)]
    pose8_joints = [math.radians(-26), math.radians(-129), math.radians(-115), math.radians(250), math.radians(-111), math.radians(86)]
    
    safe_joint_configurations = [
        pose2_joints,
        pose3_joints,
        pose6_joints,
        pose4_joints,
        pose5_joints,
        # pose6_joints,
        # pose7_joints,
        # pose8_joints,
    ]
    
    loop_counter = 0
    max_retries = 3  # Bir nokta için maksimum deneme sayısı
    
    try:
        robot_controller.get_logger().info("=== SENKRONİZE EKLEM HEDEFİ DÖNGÜSÜ (Yeniden Denemeli) BAŞLATILIYOR ===")
        robot_controller.get_logger().info("Durdurmak için Ctrl+C tuşlayın")
        
        robot_controller.get_logger().info("Başlangıç için `home_joints` pozisyonuna gidiliyor...")
        robot_controller.synchronized_move_to_joints(home_joints)
        time.sleep(2.0)
        
        while rclpy.ok():
            loop_counter += 1
            robot_controller.get_logger().info(f"=== EKLEM DÖNGÜSÜ {loop_counter} BAŞLIYOR ===")
            
            # =================================================================
            # !!! İSTENEN YENİDEN DENEME MANTIĞI BURADA !!!
            # =================================================================
            for i, joints in enumerate(safe_joint_configurations):
                robot_controller.get_logger().info(f"Sıradaki hedef {i+1}/{len(safe_joint_configurations)} için hareket başlıyor.")
                
                # Belirli bir noktaya hareketi denemek için iç döngü
                for attempt in range(max_retries):
                    robot_controller.get_logger().info(f"  -> Deneme {attempt + 1}/{max_retries}...")
                    robot_controller.get_logger().info(f"  -> Hareket başarılı, ({joints}) gidiliyor.")
                    # Hareketi dene (arada bekleme süresi vermeden)
                    success = robot_controller.synchronized_move_to_joints(joints, wait_time=0.0)
                    
                    if success:
                        robot_controller.get_logger().info(f"  -> Hareket başarılı, ({joints}) gidiliyor.")
                        robot_controller.get_logger().info(f"  -> Hareket başarılı!")
                        break  # Hareket başarılı, deneme döngüsünden çık
                    else:
                        robot_controller.get_logger().warning(f"  -> Deneme {attempt + 1} başarısız oldu.")
                        if attempt < max_retries - 1:
                            time.sleep(0.5) # Sonraki denemeden önce kısa bir bekleme
                
                # Bu 'else' bloğu, yukarıdaki 'for attempt' döngüsü 'break' ile kesilmezse çalışır.
                # Yani, tüm denemeler başarısız olduysa bu kod çalışır.
                else:
                    robot_controller.get_logger().error(
                        f"Tüm denemeler ({max_retries}) başarısız oldu. "
                        f"Bu nokta atlanıyor ve sıradaki noktaya geçiliyor."
                    )

                # İki ana nokta arasında bekleme süresi (hareket başarılı olsa da olmasa da)
                if i < len(safe_joint_configurations) - 1:
                    robot_controller.get_logger().info(f"Sonraki hedef için 1.5 saniye bekleniyor...")
                    time.sleep(1.5)
            # =================================================================
            
            robot_controller.get_logger().info(f"Döngü {loop_counter} tamamlandı.")
            time.sleep(3.0)
        
    except KeyboardInterrupt:
        robot_controller.get_logger().info(f"Program kullanıcı tarafından durduruldu (Toplam {loop_counter} döngü)")
    except Exception as e:
        robot_controller.get_logger().error(f"Beklenmeyen bir hata oluştu: {str(e)}")
    
    finally:
        robot_controller.get_logger().info("Güvenli kapatılıyor... Son kez ana pozisyona dönülüyor.")
        robot_controller.synchronized_move_to_joints(home_joints)
        rclpy.shutdown()
        executor_thread.join()

if __name__ == "__main__":
    main()