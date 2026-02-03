#!/usr/bin/env python3
"""
Cleaning Mission Runner - CSV dosyasından okunan kırmızı noktalara gider.
Tespit edilen noktalara temizlik robotu ile gitmek için kullanılır.
"""

from threading import Thread
import time
import os
import csv
import glob
from typing import List, Dict

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

from pymoveit2_sim import MoveIt2 as MoveIt2_Sim
from pymoveit2_sim.robots import ur as robot


class CleaningMissionRunner(Node):
    def __init__(self):
        super().__init__("cleaning_mission_runner")
        
        # Parametreler
        self.declare_parameter('csv_dir', '/home/cem/colcon_ws/src/pymoveit2_sim/examples/detection_results')
        self.declare_parameter('approach_height_offset', 0.15)
        self.declare_parameter('cleaning_duration', 1.5)
        self.declare_parameter('max_points', 20)  # Maksimum nokta sayısı (test için)
        self.declare_parameter('chassis_z_height', 1.1)  # Şase yüksekliği
        self.declare_parameter('cleaning_offset_x', 0.30)  # X yönünde offset
        
        self.csv_dir = self.get_parameter('csv_dir').value
        self.approach_offset = self.get_parameter('approach_height_offset').value
        self.cleaning_duration = self.get_parameter('cleaning_duration').value
        self.max_points = self.get_parameter('max_points').value
        self.chassis_z_height = self.get_parameter('chassis_z_height').value
        self.cleaning_offset_x = self.get_parameter('cleaning_offset_x').value
        
        # Callback group
        callback_group = ReentrantCallbackGroup()

        # MoveIt 2 interface - sim_robot_goal.py ile aynı yapılandırma
        self.moveit2 = MoveIt2_Sim(
            node=self,
            joint_names=robot.joint_names(),
            base_link_name="world",  # ÖNEMLİ: world olmalı
            end_effector_name=robot.end_effector_name(),
            group_name=robot.MOVE_GROUP_ARM,
            callback_group=callback_group,
        )
        
        # Planner ayarları
        self.moveit2.planner_id = "RRTConnectkConfigDefault"
        self.moveit2.max_velocity = 0.2
        self.moveit2.max_acceleration = 0.2
        
        # Çarpışma önleme
        self.moveit2.cartesian_avoid_collisions = True
        self.moveit2.cartesian_jump_threshold = 2.0
        
        # Status publisher
        self.status_publisher = self.create_publisher(String, '/cleaning_robot/status', 10)
        self.progress_publisher = self.create_publisher(String, '/progress_status', 10)
        
        # Progress durumu
        self.current_progress = "IDLE"
        
        # Progress timer (1 Hz)
        self.progress_timer = self.create_timer(1.0, self._publish_progress_callback)
        
        # Hedefler
        self.targets: List[Dict] = []
        self.cleaned_count = 0
    
    def find_latest_csv(self) -> str:
        """En son CSV dosyasını bul"""
        pattern = os.path.join(self.csv_dir, "red_points_*.csv")
        csv_files = glob.glob(pattern)
        
        if not csv_files:
            return None
        
        # En yeni dosyayı seç
        latest_file = max(csv_files, key=os.path.getmtime)
        return latest_file
    
    def publish_progress(self, progress: str):
        """Progress durumunu ayarla ve yayınla"""
        self.current_progress = progress
        msg = String()
        msg.data = progress
        self.progress_publisher.publish(msg)
    
    def _publish_progress_callback(self):
        """Timer callback - progress durumunu periyodik yayınla"""
        msg = String()
        msg.data = self.current_progress
        self.progress_publisher.publish(msg)
    
    def load_targets_from_csv(self, csv_file: str) -> bool:
        """CSV dosyasından hedefleri yükle - PointCloud2 formatı"""
        try:
            with open(csv_file, 'r') as f:
                reader = csv.DictReader(f)
                self.targets = []
                
                for row in reader:
                    # Yeni CSV formatı (PointCloud2):
                    # timestamp, waypoint_index, pixel_x, pixel_y, area, world_x, world_y, world_z
                    target = {
                        'pixel_x': int(row['pixel_x']),
                        'pixel_y': int(row['pixel_y']),
                        'area': float(row['area']),
                        # World frame pozisyonu (PointCloud2'den TF ile hesaplanmış)
                        'world_x': float(row.get('world_x', 0)),
                        'world_y': float(row.get('world_y', 0)),
                        'world_z': float(row.get('world_z', 0)),
                        'waypoint_index': int(row.get('waypoint_index', 0))
                    }
                    self.targets.append(target)
                    
                    if len(self.targets) >= self.max_points:
                        break
            
            return True
            
        except Exception as e:
            return False
    
    def move_to_joint_config(self, joint_positions):
        """Eklem konfigürasyonuna git"""
        try:
            self.moveit2.move_to_configuration(joint_positions)
            success = self.moveit2.wait_until_executed()
            return success
        except Exception as e:
            return False
    
    def move_to_pose(self, position, orientation=None, cartesian=False):
        """Pozisyona git - sim_robot_goal.py ile uyumlu"""
        if orientation is None:
            # Varsayılan orientasyon (sim_robot_goal.py'dan)
            orientation = [1.0, 0.0, 0.0, 0.0]
        
        try:
            self.moveit2.move_to_pose(
                position=position,
                quat_xyzw=orientation,
                cartesian=cartesian,
                cartesian_max_step=0.005,  # Daha küçük adımlar
                cartesian_fraction_threshold=0.95,
            )
            success = self.moveit2.wait_until_executed()
            return success
        except Exception as e:
            return False
    
    def clean_point(self, index: int, target: Dict) -> bool:
        """Bir noktayı temizle - PointCloud2 world pozisyonuna göre git"""
        
        # World frame pozisyonu (PointCloud2 + TF ile hesaplanmış)
        world_x = target.get('world_x', 0)
        world_y = target.get('world_y', 0)
        world_z = target.get('world_z', 0)
        
        # Güvenlik kontrolü: Pozisyon makul mu?
        # Şase yerden ~0.3m ile ~2m arasında
        z_is_valid = (0.25 <= world_z <= 2.1)  # Biraz tolerans
        position_is_valid = z_is_valid and (world_x != 0 or world_y != 0)
        
        # Temizlik pozisyonunu belirle
        if not position_is_valid:
            return False
        
        # Temizlik pozisyonu: world pozisyonuna offset ekle
        cleaning_pos = [
            world_x + self.cleaning_offset_x,  # X yönünde offset
            world_y,
            world_z
        ]
        
        # Orientasyon - sim_robot_goal.py'dan
        orientation = [-1.0, 0.0, 0.0, 0.0]
        
        # 1. Yaklaşma pozisyonu (üstten)
        approach_pos = [
            cleaning_pos[0],
            cleaning_pos[1],
            cleaning_pos[2] + self.approach_offset
        ]
        
        msg = String()
        msg.data = f"APPROACHING|POINT:{index+1}|TOTAL:{len(self.targets)}"
        self.status_publisher.publish(msg)
        
        success = self.move_to_pose(approach_pos, orientation, cartesian=False)
        if not success:
            return False
        
        time.sleep(0.3)
        
        # 2. Temizlik pozisyonuna in
        msg.data = f"CLEANING|POINT:{index+1}|TOTAL:{len(self.targets)}"
        self.status_publisher.publish(msg)
        
        success = self.move_to_pose(cleaning_pos, orientation, cartesian=True)
        if not success:
            success = self.move_to_pose(cleaning_pos, orientation, cartesian=False)
        
        if not success:
            return False
        
        # 3. Temizlik işlemi
        time.sleep(self.cleaning_duration)
        
        # 4. Yukarı çık
        self.move_to_pose(approach_pos, orientation, cartesian=True)
        
        self.cleaned_count += 1
        return True
    
    def run_mission(self):
        """Temizlik görevini çalıştır"""
        # CSV dosyasını bul
        csv_file = self.find_latest_csv()
        if not csv_file:
            return
        
        # Hedefleri yükle
        if not self.load_targets_from_csv(csv_file):
            return
        
        if not self.targets:
            return
        
        # CLEANING başladı - sadece başlangıç log'u
        self.get_logger().info("=== CLEANING MISSION STARTED ===")
        self.publish_progress("CLEANING")
        
        # Home pozisyonu - 7 eklem (linear axis + 6 UR eklem)
        # Linear axis = 0.0, UR eklemleri: [0, -90°, 90°, -90°, -90°, 0]
        home_joints = [0.0, 0.0, -1.57, 1.57, -1.57, -1.57, 0.0]
        
        self.move_to_joint_config(home_joints)
        time.sleep(1.0)
        
        # Her noktayı temizle
        failed_points = []
        for i, target in enumerate(self.targets):
            try:
                success = self.clean_point(i, target)
                if not success:
                    failed_points.append(i + 1)
            except Exception as e:
                failed_points.append(i + 1)
            
            time.sleep(0.5)
        
        # Home'a dön
        home_joints = [0.0, 0.0, -1.57, 1.57, -1.57, -1.57, 0.0]
        self.move_to_joint_config(home_joints)
        
        # Sadece bitiş log'u
        self.get_logger().info(f"=== CLEANING MISSION COMPLETED ({self.cleaned_count}/{len(self.targets)} cleaned) ===")
        
        # CLEANING COMPLETED yayınla
        self.publish_progress("CLEANING COMPLETED")
        
        msg = String()
        msg.data = f"COMPLETE|CLEANED:{self.cleaned_count}|TOTAL:{len(self.targets)}"
        self.status_publisher.publish(msg)


def main():
    rclpy.init()
    
    node = CleaningMissionRunner()
    
    # Multi-threaded executor
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    
    # Görevi ayrı thread'de çalıştır
    thread = Thread(target=node.run_mission)
    thread.start()
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        thread.join(timeout=1.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
