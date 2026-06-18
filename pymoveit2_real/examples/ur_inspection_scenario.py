#!/usr/bin/env python3
"""
Single robot controller with trajectory recording and playback system.
Designed for fake hardware setup.
"""

import json
import os
from threading import Thread
import time
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
import math

# MoveIt2 kütüphanesi (sadece gerçek robot)
from pymoveit2_real import MoveIt2 as MoveIt2_Real
from pymoveit2_real.robots import ur as realrobot

class TrajectoryManager:
    """Trajectory kaydetme ve yükleme işlemlerini yöneten sınıf"""
    
    def __init__(self, file_path="real_trajectories.json"):
        self.file_path = file_path
        self.trajectories = {}
    
    def save_trajectory(self, name, trajectory):
        """Trajectory'yi dosyaya kaydet"""
        traj_dict = {
            'joint_names': trajectory.joint_names,
            'points': []
        }
        
        for point in trajectory.points:
            point_dict = {
                'positions': list(point.positions),
                'velocities': list(point.velocities),
                'accelerations': list(point.accelerations),
                'time_from_start': {
                    'sec': point.time_from_start.sec,
                    'nanosec': point.time_from_start.nanosec
                }
            }
            traj_dict['points'].append(point_dict)
        
        self.trajectories[name] = traj_dict
        
        # Dosyaya kaydet
        with open(self.file_path, 'w') as f:
            json.dump(self.trajectories, f, indent=2)
    
    def load_trajectories(self):
        """Kaydedilmiş trajectory'leri dosyadan yükle"""
        if os.path.exists(self.file_path):
            with open(self.file_path, 'r') as f:
                self.trajectories = json.load(f)
            return True
        return False
    
    def get_trajectory(self, name):
        """Belirli bir trajectory'yi al"""
        return self.trajectories.get(name, None)
    
    def list_trajectories(self):
        """Kayıtlı trajectory isimlerini listele"""
        return list(self.trajectories.keys())

class SingleRobotController(Node):
    def __init__(self):
        super().__init__("single_robot_controller_with_recording")
        
        callback_group = ReentrantCallbackGroup()
        
        # Sadece gerçek robot (fake hardware ile çalışacak)
        self.robot = MoveIt2_Real(
            node=self,
            joint_names=realrobot.joint_names(),
            base_link_name="world",
            end_effector_name=realrobot.end_effector_name(),
            group_name=realrobot.MOVE_GROUP_ARM,
            callback_group=callback_group,
        )
        self.robot.planner_id = "SBLkConfigDefault"
        self.robot.max_velocity = 0.1
        self.robot.max_acceleration = 0.1
        
        # Trajectory yöneticisi
        self.trajectory_manager = TrajectoryManager()
        
        self.get_logger().info("Single Robot Controller with Trajectory Recording başlatıldı")

    def record_trajectory_between_points(self, start_joints, end_joints, trajectory_name, max_retries=20):
        """İki nokta arası trajectory'yi kaydet (yeniden deneme ile)"""
        self.get_logger().info(f"Trajectory kaydediliyor: {trajectory_name}")
        
        for attempt in range(max_retries):
            try:
                self.get_logger().info(f"  Deneme {attempt + 1}/{max_retries}")
                
                # Start pozisyonuna git
                self.robot.move_to_configuration(start_joints)
                success = self.robot.wait_until_executed()
                if not success:
                    self.get_logger().warning(f"Start pozisyonuna gidemedi, deneme {attempt + 1}")
                    time.sleep(1.0)
                    continue
                
                time.sleep(0.5)
                
                # End pozisyonu için trajectory planla
                trajectory = self.robot.plan(joint_positions=end_joints)
                if not trajectory:
                    self.get_logger().warning(f"Planning başarısız, deneme {attempt + 1}")
                    time.sleep(1.0)
                    continue
                
                # Trajectory'yi kaydet
                self.trajectory_manager.save_trajectory(trajectory_name, trajectory)
                self.get_logger().info(f"✓ Trajectory kaydedildi: {trajectory_name}")
                
                # Hareketi tamamla
                self.robot.execute(trajectory)
                self.robot.wait_until_executed()
                return True
                
            except Exception as e:
                self.get_logger().warning(f"Deneme {attempt + 1} hatası: {str(e)}")
                time.sleep(1.0)
        
        self.get_logger().error(f"✗ Tüm denemeler başarısız: {trajectory_name}")
        return False

    def execute_recorded_trajectory(self, trajectory_name, max_retries=6):
        """Kaydedilmiş trajectory'yi çalıştır (yeniden deneme ile)"""
        traj_dict = self.trajectory_manager.get_trajectory(trajectory_name)
        if not traj_dict:
            self.get_logger().error(f"Trajectory bulunamadı: {trajectory_name}")
            return False
        
        for attempt in range(max_retries):
            try:
                self.get_logger().info(f"Kaydedilmiş trajectory çalıştırılıyor: {trajectory_name} (Deneme {attempt + 1})")
                
                # Trajectory'yi yeniden oluştur
                from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
                from builtin_interfaces.msg import Duration
                
                trajectory = JointTrajectory()
                trajectory.joint_names = traj_dict['joint_names']
                
                for point_data in traj_dict['points']:
                    point = JointTrajectoryPoint()
                    point.positions = point_data['positions']
                    point.velocities = point_data['velocities']
                    point.accelerations = point_data['accelerations']
                    point.time_from_start = Duration()
                    point.time_from_start.sec = point_data['time_from_start']['sec']
                    point.time_from_start.nanosec = point_data['time_from_start']['nanosec']
                    trajectory.points.append(point)
                
                joint_state_msg = self.robot.joint_state
                if joint_state_msg and trajectory.points:
                    joint_positions_map = {name: pos for name, pos in zip(joint_state_msg.name, joint_state_msg.position)}
                    expected_joint_order = self.robot.joint_names
                    current_joints_ordered = [joint_positions_map[name] for name in expected_joint_order if name in joint_positions_map]
                    
                    if len(current_joints_ordered) == len(expected_joint_order):
                        trajectory.points[0].positions = current_joints_ordered
                        trajectory.points[0].velocities = [0.0] * len(expected_joint_order)
                        trajectory.points[0].accelerations = [0.0] * len(expected_joint_order)
                        trajectory.points[0].time_from_start.sec = 0
                        trajectory.points[0].time_from_start.nanosec = 0
                
                self.robot.execute(trajectory)
                success = self.robot.wait_until_executed()
                
                if success:
                    self.get_logger().info(f"✓ Trajectory başarılı: {trajectory_name}")
                    return True
                else:
                    self.get_logger().warning(f"Trajectory başarısız, deneme {attempt + 1}")
                    time.sleep(0.5)
                    
            except Exception as e:
                self.get_logger().warning(f"Deneme {attempt + 1} hatası: {str(e)}")
                time.sleep(0.5)
        
        self.get_logger().error(f"✗ Tüm denemeler başarısız: {trajectory_name}")
        return False

    def move_to_joints_safe(self, joint_positions, max_retries=6):
        """Güvenli eklem hareketi (yeniden deneme ile)"""
        for attempt in range(max_retries):
            try:
                self.get_logger().info(f"Eklem hareketine başlanıyor (Deneme {attempt + 1})")
                self.robot.move_to_configuration(joint_positions)
                success = self.robot.wait_until_executed()
                if success:
                    return True
                else:
                    self.get_logger().warning(f"Hareket başarısız, deneme {attempt + 1}")
                    time.sleep(1.0)
            except Exception as e:
                self.get_logger().warning(f"Hareket hatası deneme {attempt + 1}: {str(e)}")
                time.sleep(1.0)
        return False

# =========================================================================================
# === GÜNCELLENMIŞ MAIN FONKSIYONU =========================================================
# =========================================================================================
def main():
    rclpy.init()
    
    robot_controller = SingleRobotController()
    
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(robot_controller)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()
    
    time.sleep(2.0)
    
    # --- GÜNCELLENMIŞ WAYPOINT'LER ---
    home_joints = [1.0, 0.0, -math.pi/2, 0.0, -math.pi/2, 0.0, 0.0]

    # Pose 1 Ailesi (very_right) - Radyana çevrildi
    vr_top = [1.9, math.radians(-15.36), math.radians(-115.21), math.radians(-51.42), math.radians(-13.16), math.radians(110.86), math.radians(90.0)]
    vr_middle = [1.9, math.radians(-15.39), math.radians(-117.15), math.radians(-82.83), math.radians(20.18), math.radians(110.77), math.radians(90.0)]
    vr_below = [1.9, math.radians(-15.36), math.radians(-147.23), math.radians(-90.96), math.radians(58.35), math.radians(110.72), math.radians(90.0)]
    
    # Ara nokta (Pose 1'den Pose 2'ye geçiş için)
    intermediate_point = [1.0, math.radians(34.37), math.radians(-55.44), math.radians(-122.10), math.radians(-2.37), math.radians(60.99), math.radians(0)]

    # Pose 2 Ailesi (sağ_üst_göz)
    pose2_joints_downwards = [1.0, math.radians(-19.36), math.radians(-122.69), math.radians(-51.80), math.radians(-2.51), math.radians(116.43), math.radians(0.0)]
    pose2_joints_left = [1.0, math.radians(-19.36), math.radians(-122.69), math.radians(-51.80), math.radians(-2.51), math.radians(116.43), math.radians(90.0)]
    pose2_joints_upwards = [1.0, math.radians(-19.36), math.radians(-122.69), math.radians(-51.80), math.radians(-2.51), math.radians(116.43), math.radians(180.0)]
    pose2_joints_right = [1.0, math.radians(-19.36), math.radians(-122.69), math.radians(-51.80), math.radians(-2.51), math.radians(116.43), math.radians(270.0)]
    pose2_joints_tofront = [1.0, math.radians(-19.36), math.radians(-122.69), math.radians(-51.80), math.radians(-2.51), math.radians(180.0), math.radians(-90.0)]

    # Pose 3 Ailesi (sol_üst_göz)
    pose3_joints_downwards = [1.0, math.radians(34.56), math.radians(-117.73), math.radians(-53.48), math.radians(-13.09), math.radians(55.02), math.radians(0.0)]
    pose3_joints_left = [1.0, math.radians(34.56), math.radians(-117.73), math.radians(-53.48), math.radians(-13.09), math.radians(55.02), math.radians(90.0)]
    pose3_joints_upwards = [1.0, math.radians(34.56), math.radians(-117.73), math.radians(-53.48), math.radians(-13.09), math.radians(55.02), math.radians(180.0)]
    pose3_joints_right = [1.0, math.radians(34.56), math.radians(-117.73), math.radians(-53.48), math.radians(-13.09), math.radians(55.02), math.radians(270.0)]
    pose3_joints_tofront = [1.0, math.radians(34.56), math.radians(-117.73), math.radians(-53.48), math.radians(-13.09), math.radians(180.0), math.radians(-90.0)]

    # Pose 4 Ailesi (very_left) - Radyana çevrildi
    vl_top = [0.055, math.radians(25.22), math.radians(-135.98), math.radians(-9.01), math.radians(-23.58), math.radians(61.88), math.radians(-90.0)]
    vl_middle = [0.055, math.radians(25.17), math.radians(-123.55), math.radians(-60.94), math.radians(15.93), math.radians(61.75), math.radians(-90.0)]
    vl_below = [0.055, math.radians(25.19), math.radians(-158.32), math.radians(-75.32), math.radians(65.15), math.radians(61.69), math.radians(-90.0)]

    # Pose 5 Ailesi (sol_alt_göz) - GÜNCELLENDI
    pose5_joints_downwards = [1.0, math.radians(43.79), math.radians(-150.71), math.radians(-97.00), math.radians(67.98), math.radians(51.56), math.radians(0.0)]
    pose5_joints_left = [1.0, math.radians(46.29), math.radians(-149.39), math.radians(-101.79), math.radians(71.48), math.radians(49.05), math.radians(90.0)]
    pose5_joints_upwards = [1.0, math.radians(46.29), math.radians(-149.39), math.radians(-101.79), math.radians(71.48), math.radians(49.05), math.radians(180.0)]
    pose5_joints_right = [1.0, math.radians(46.29), math.radians(-149.39), math.radians(-101.79), math.radians(71.48), math.radians(49.05), math.radians(270.0)]

    # Alt ara nokta (Pose 5 ve 6 arası)
    below_intermediate_point = [1.0, math.radians(62.61), math.radians(-146.06), math.radians(-120.64), math.radians(87.28), math.radians(32.73), math.radians(0.0)]

    # Pose 6 Ailesi (sağ_alt_göz) - GÜNCELLENDI
    pose6_joints_downwards = [1.0, math.radians(-23.72), math.radians(-148.62), math.radians(-105.66), math.radians(70.17), math.radians(111.45), math.radians(0.0)]
    pose6_joints_left = [1.0, math.radians(-23.72), math.radians(-148.62), math.radians(-105.66), math.radians(70.17), math.radians(111.45), math.radians(90.0)]
    pose6_joints_upwards = [1.0, math.radians(-23.72), math.radians(-148.62), math.radians(-105.66), math.radians(70.17), math.radians(111.45), math.radians(180.0)]
    pose6_joints_right = [1.0, math.radians(-23.72), math.radians(-148.62), math.radians(-105.66), math.radians(70.17), math.radians(111.45), math.radians(270.0)]
    
    # --- YENİ VE DETAYLI TRAJECTORY SIRALAMASI ---
    trajectory_sequence = [
        # 1. Başlangıç Hareketi
        ("trajectory_home_to_pose1_top", home_joints, vr_top, "Home -> Pose1 Top (Very Right)"),
        
        # 2. Pose 1 (Very Right) Sekansı
        ("trajectory_pose1_top_to_middle", vr_top, vr_middle, "Pose1: Top -> Middle"),
        ("trajectory_pose1_middle_to_below", vr_middle, vr_below, "Pose1: Middle -> Below"),
        
        # 3. Pose 1'den Ara Noktaya ve Pose 2'ye Geçiş
        ("trajectory_pose1_to_pose2", vr_below, pose2_joints_downwards, "Pose1 (Below) -> Pose2: Down"),
        # ("trajectory_intermediate_to_pose2", intermediate_point, pose2_joints_downwards, "Intermediate -> Pose2 (Down)"),
        
        # 4. Pose 2 (Sağ Üst Göz) Bilek Döndürme Sekansı
        ("trajectory_pose2_down_to_left", pose2_joints_downwards, pose2_joints_left, "Pose2: Down -> Left"),
        ("trajectory_pose2_left_to_up", pose2_joints_left, pose2_joints_upwards, "Pose2: Left -> Up"),
        ("trajectory_pose2_up_to_right", pose2_joints_upwards, pose2_joints_right, "Pose2: Up -> Right"),
        ("trajectory_pose2_right_to_front", pose2_joints_right, pose2_joints_tofront, "Pose2: Right -> Front"),
        
        # 5. Pose 2'den Pose 3'e Geçiş
        ("trajectory_pose2_to_pose3", pose2_joints_tofront, pose3_joints_downwards, "Pose2 (Front) -> Pose3 (Down)"),
        
        # 6. Pose 3 (Sol Üst Göz) Bilek Döndürme Sekansı
        ("trajectory_pose3_down_to_left", pose3_joints_downwards, pose3_joints_left, "Pose3: Down -> Left"),
        ("trajectory_pose3_left_to_up", pose3_joints_left, pose3_joints_upwards, "Pose3: Left -> Up"),
        ("trajectory_pose3_up_to_right", pose3_joints_upwards, pose3_joints_right, "Pose3: Up -> Right"),
        ("trajectory_pose3_right_to_front", pose3_joints_right, pose3_joints_tofront, "Pose3: Right -> Front"),
        
        # 7. Pose 3'ten Ara Noktaya ve Pose 4'e Geçiş
        ("trajectory_pose3_to_intermediate", pose3_joints_tofront, intermediate_point, "Pose3 (Front) -> Intermediate"),
        ("trajectory_intermediate_to_pose4_top", intermediate_point, vl_top, "Intermediate -> Pose4 Top (Very Left)"),
        
        # 8. Pose 4 (Very Left) Sekansı
        ("trajectory_pose4_top_to_middle", vl_top, vl_middle, "Pose4: Top -> Middle"),
        ("trajectory_pose4_middle_to_below", vl_middle, vl_below, "Pose4: Middle -> Below"),
        
        # 9. Pose 4'ten Pose 5'e Geçiş
        ("trajectory_pose4_to_pose5", vl_below, pose5_joints_downwards, "Pose4 (Below) -> Pose5 (Down)"),
        
        # 10. Pose 5 (Sol Alt Göz) Bilek Döndürme Sekansı
        ("trajectory_pose5_down_to_left", pose5_joints_downwards, pose5_joints_left, "Pose5: Down -> Left"),
        ("trajectory_pose5_left_to_up", pose5_joints_left, pose5_joints_upwards, "Pose5: Left -> Up"),
        ("trajectory_pose5_up_to_right", pose5_joints_upwards, pose5_joints_right, "Pose5: Up -> Right"),
        
        # 11. Pose 5'ten Alt Ara Noktaya ve Pose 6'ya Geçiş
        ("trajectory_pose5_to_below_intermediate", pose5_joints_right, below_intermediate_point, "Pose5 (Right) -> Below Intermediate"),
        ("trajectory_below_intermediate_to_pose6", below_intermediate_point, pose6_joints_downwards, "Below Intermediate -> Pose6 (Down)"),
        
        # 12. Pose 6 (Sağ Alt Göz) Bilek Döndürme Sekansı
        ("trajectory_pose6_down_to_left", pose6_joints_downwards, pose6_joints_left, "Pose6: Down -> Left"),
        ("trajectory_pose6_left_to_up", pose6_joints_left, pose6_joints_upwards, "Pose6: Left -> Up"),
        ("trajectory_pose6_up_to_right", pose6_joints_upwards, pose6_joints_right, "Pose6: Up -> Right"),
        
        # 13. Döngüyü Kapatmak İçin Başlangıç Pozuna Dönüş
        ("trajectory_return_to_start", pose6_joints_right, vr_top, "Pose6 (Right) -> Pose1 Top (Return for Loop)")
    ]

    try:
        # Kayıtlı trajectory'leri yükle
        robot_controller.trajectory_manager.load_trajectories()
        recorded_trajectories = robot_controller.trajectory_manager.list_trajectories()
        
        expected_trajectories = [traj[0] for traj in trajectory_sequence]
        
        missing_trajectories = [traj for traj in expected_trajectories if traj not in recorded_trajectories]
        
        if not missing_trajectories:
            # PLAYBACK MODU
            robot_controller.get_logger().info(f"Kayıtlı trajectory'ler bulundu: {recorded_trajectories}")
            robot_controller.get_logger().info("=== PLAYBACK MODU BAŞLATILIYOR ===")
            
            loop_counter = 0
            while rclpy.ok():
                loop_counter += 1
                robot_controller.get_logger().info(f"=== DÖNGÜ {loop_counter} BAŞLIYOR ===")
                
                all_success = True
                for i, (traj_name, start_pos, end_pos, description) in enumerate(trajectory_sequence):
                    if loop_counter > 1 and traj_name == "trajectory_home_to_pose1_top":
                        continue

                    robot_controller.get_logger().info(f"Adım {i+1}/{len(trajectory_sequence)}: {description} ({traj_name})")
                    success = robot_controller.execute_recorded_trajectory(traj_name)
                    
                    if success:
                        robot_controller.get_logger().info(f"✓ Adım {i+1} başarılı: {description}")
                    else:
                        robot_controller.get_logger().error(f"✗ Adım {i+1} başarısız: {description}")
                        all_success = False
                    
                    time.sleep(1.0)
                
                if all_success:
                    robot_controller.get_logger().info(f"🎯 Döngü {loop_counter} TAMAMLANDI - Tüm adımlar başarılı!")
                else:
                    robot_controller.get_logger().warning(f"⚠️ Döngü {loop_counter} hatalarla tamamlandı")
                
                time.sleep(3.0)
        
        else:
            # KAYIT MODU
            robot_controller.get_logger().info("=== KAYIT MODU BAŞLATILIYOR ===")
            robot_controller.get_logger().info(f"Toplam {len(expected_trajectories)} trajectory kaydedilmesi gerekiyor")
            robot_controller.get_logger().info(f"Eksik trajectory'ler: {missing_trajectories}")
            
            robot_controller.get_logger().info("Home pozisyonuna gidiliyor...")
            robot_controller.move_to_joints_safe(home_joints)
            time.sleep(2.0)
            
            while True:
                robot_controller.trajectory_manager.load_trajectories()
                current_trajectories = robot_controller.trajectory_manager.list_trajectories()
                
                remaining_trajectories = [traj for traj in expected_trajectories if traj not in current_trajectories]
                
                if not remaining_trajectories:
                    robot_controller.get_logger().info("=== TÜM TRAJECTORY'LER BAŞARIYLA KAYDEDİLDİ ===")
                    robot_controller.get_logger().info("Playback moduna geçiliyor...")
                    break
                
                robot_controller.get_logger().info(f"Kalan trajectory sayısı: {len(remaining_trajectories)}")
                
                for traj_name, start_joints, end_joints, description in trajectory_sequence:
                    if traj_name in current_trajectories:
                        continue
                    
                    robot_controller.get_logger().info(f"Kaydediliyor: {description} ({traj_name})")
                    success = robot_controller.record_trajectory_between_points(start_joints, end_joints, traj_name, max_retries=10)
                    
                    if success:
                        robot_controller.get_logger().info(f"✓ {description} kaydedildi")
                    else:
                        robot_controller.get_logger().error(f"✗ {description} kayıt başarısız")
                    
                    time.sleep(2.0)
                
                if remaining_trajectories:
                    robot_controller.get_logger().info("Tüm trajectory'ler henüz kaydedilemedi. Tekrar deneniyor...")
                    time.sleep(5.0)
            
            robot_controller.get_logger().info("Program yeniden başlatılacak ve playback modu çalışacak...")
            return
    
    except KeyboardInterrupt:
        robot_controller.get_logger().info("Program kullanıcı tarafından durduruldu")
    except Exception as e:
        robot_controller.get_logger().error(f"Beklenmeyen hata: {str(e)}")
    
    finally:
        robot_controller.get_logger().info("Güvenli kapatılıyor...")
        robot_controller.move_to_joints_safe(home_joints)
        rclpy.shutdown()
        executor_thread.join()

if __name__ == "__main__":
    main()