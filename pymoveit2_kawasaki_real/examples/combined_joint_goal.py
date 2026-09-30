#!/usr/bin/env python3
"""
Single robot controller with trajectory recording and playback system.
Designed for SIMULATION setup.
"""

import json
import os
from threading import Thread
import time
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
import math

# MoveIt2 simulation library
from pymoveit2_sim import MoveIt2 as MoveIt2_Sim
from pymoveit2_sim.robots import ur as simrobot

class TrajectoryManager:
    """Trajectory kaydetme ve yükleme işlemlerini yöneten sınıf"""
    
    def __init__(self, file_path="sim_recorded_trajectories.json"):
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

class SimRobotController(Node):
    def __init__(self):
        super().__init__("sim_robot_controller_with_recording")
        
        callback_group = ReentrantCallbackGroup()
        
        # MoveIt2 Simulation Interface
        self.robot = MoveIt2_Sim(
            node=self,
            joint_names=simrobot.joint_names(),
            base_link_name=simrobot.base_link_name(),
            end_effector_name=simrobot.end_effector_name(),
            group_name=simrobot.MOVE_GROUP_ARM,
            callback_group=callback_group,
        )
        self.robot.planner_id = "RRTConnectkConfigDefault"
        self.robot.max_velocity = 0.1
        self.robot.max_acceleration = 0.1
        
        # Trajectory yöneticisi
        self.trajectory_manager = TrajectoryManager()
        
        self.get_logger().info("Simulation Robot Controller with Trajectory Recording başlatıldı")

    def record_trajectory_between_points(self, start_joints, end_joints, trajectory_name, max_retries=5):
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
                # plan() metodu simülasyon kütüphanesinde farklılık gösterebilir.
                # Normalde move_to_configuration zaten bir planlama yapar.
                # Burada direkt planı elde etmek için farklı bir yaklaşım gerekebilir.
                # Ancak şimdilik execute edilmiş planı yakalamaya çalışalım.
                # Bu kısım `pymoveit2` ve `pymoveit2_sim` arasındaki farklara göre uyarlanmalıdır.
                # `plan()` metodu `pymoveit2_sim` içinde olmayabilir, bu yüzden `move_to_configuration` kullanıyoruz.
                # Planı kaydetmek için, hareket etmeden önce bir plan oluşturmamız gerekir.
                # `plan_kinematic_path` bu işi görür.
                (plan_success, trajectory) = self.robot.plan_kinematic_path(joint_positions=end_joints)

                if not plan_success or not trajectory:
                    self.get_logger().warning(f"Planning başarısız, deneme {attempt + 1}")
                    time.sleep(1.0)
                    continue

                # Trajectory'yi kaydet
                self.trajectory_manager.save_trajectory(trajectory_name, trajectory.joint_trajectory)
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

    def execute_recorded_trajectory(self, trajectory_name, max_retries=3):
        """Kaydedilmiş trajectory'yi çalıştır (yeniden deneme ile)"""
        traj_dict = self.trajectory_manager.get_trajectory(trajectory_name)
        if not traj_dict:
            self.get_logger().error(f"Trajectory bulunamadı: {trajectory_name}")
            return False
        
        for attempt in range(max_retries):
            try:
                self.get_logger().info(f"Kaydedilmiş trajectory çalıştırılıyor: {trajectory_name} (Deneme {attempt + 1})")
                
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
                
                # Simülasyonda state'i direkt almak yerine beklemek daha güvenli olabilir.
                # Bu kısım `pymoveit2_sim` ile uyumlu olmalıdır.
                # `pymoveit2_sim`'de `self.robot.get_planning_scene()` gibi metodlar kullanılabilir.
                # Şimdilik bu kısmı basitleştiriyoruz, çünkü simülasyonda genellikle başlangıç durumu daha nettir.
                
                # Çalıştır
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

    def move_to_joints_safe(self, joint_positions, max_retries=3):
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

def main():
    rclpy.init()
    
    robot_controller = SimRobotController()
    
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(robot_controller)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()
    
    time.sleep(2.0)
    
    # Waypoint'ler (Gerçek robotla aynı)
    home_joints = [0.0, -math.pi/2, 0.0, -math.pi/2, 0.0, 0.0]
    pose2_joints = [math.radians(-25.91), math.radians(-112.9), math.radians(-58.29), math.radians(-20.6), math.radians(122.96), math.radians(61.53)]
    pose3_joints = [math.radians(-30.16), math.radians(-144.62), math.radians(-104.24), math.radians(56.34), math.radians(126.94), math.radians(60.25)]
    pose4_joints = [math.radians(28.29), math.radians(-112.9), math.radians(-60.63), math.radians(-15.19), math.radians(65.06), math.radians(61.53)]
    pose5_joints = [math.radians(36.19), math.radians(-140.42), math.radians(-124.24), math.radians(71.80), math.radians(61.68), math.radians(73.15)]
    intermediate_point = [math.radians(-2.01), math.radians(-64.85), math.radians(-131.44), math.radians(7.79), math.radians(98.62), math.radians(65.41)]
    
    trajectory_sequence = [
        ("trajectory_home_0", home_joints, pose2_joints, "Home -> Pose2"),
        ("trajectory_0_1", pose2_joints, pose3_joints, "Pose2 -> Pose3"), 
        ("trajectory_1_intermediate", pose3_joints, intermediate_point, "Pose3 -> Intermediate"),
        ("trajectory_intermediate_2", intermediate_point, pose4_joints, "Intermediate -> Pose4"),
        ("trajectory_2_3", pose4_joints, pose5_joints, "Pose4 -> Pose5"),
        ("trajectory_return_to_start", pose5_joints, pose2_joints, "Pose5 -> Pose2 (Return)")
    ]
    
    try:
        # Kayıtlı SİMÜLASYON trajectory'lerini yükle
        robot_controller.trajectory_manager.load_trajectories()
        recorded_trajectories = robot_controller.trajectory_manager.list_trajectories()
        
        expected_trajectories = [traj[0] for traj in trajectory_sequence]
        
        missing_trajectories = [traj for traj in expected_trajectories if traj not in recorded_trajectories]
        
        if not missing_trajectories:
            # PLAYBACK MODU
            robot_controller.get_logger().info(f"Kayıtlı simülasyon trajectory'leri bulundu: {recorded_trajectories}")
            robot_controller.get_logger().info("=== SİMÜLASYON PLAYBACK MODU BAŞLATILIYOR ===")
            
            loop_counter = 0
            while rclpy.ok():
                loop_counter += 1
                robot_controller.get_logger().info(f"=== DÖNGÜ {loop_counter} BAŞLIYOR ===")
                
                all_success = True
                for i, (traj_name, start_pos, end_pos, description) in enumerate(trajectory_sequence):
                    if loop_counter > 1 and traj_name == "trajectory_home_0":
                        continue

                    robot_controller.get_logger().info(f"Adım {i+1}/{len(trajectory_sequence)}: {description} ({traj_name})")
                    success = robot_controller.execute_recorded_trajectory(traj_name)
                    
                    if success:
                        robot_controller.get_logger().info(f"✓ Adım {i+1} başarılı: {description}")
                    else:
                        robot_controller.get_logger().error(f"✗ Adım {i+1} başarısız: {description}")
                        all_success = False
                    
                    time.sleep(2.0)
                
                if all_success:
                    robot_controller.get_logger().info(f"🎯 Döngü {loop_counter} TAMAMLANDI - Tüm adımlar başarılı!")
                else:
                    robot_controller.get_logger().warning(f"⚠️ Döngü {loop_counter} hatalarla tamamlandı")
                
                time.sleep(3.0)
        
        else:
            # KAYIT MODU
            robot_controller.get_logger().info("=== SİMÜLASYON KAYIT MODU BAŞLATILIYOR ===")
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