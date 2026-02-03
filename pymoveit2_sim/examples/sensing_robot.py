#!/usr/bin/env python3
"""
Çarpışma önleyici fonksiyon tabanlı pose ve joint hedefleri ile robot hareketi
"""

from threading import Thread
import time
import math  # Radyan dönüşümleri için eklendi

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from pymoveit2_sim import MoveIt2 as MoveIt2_Sim
from pymoveit2_sim import MoveIt2State
from pymoveit2_sim.robots import ur as robot

from std_msgs.msg import String, Int32


class CollisionAwareRobotController(Node):
    def __init__(self):
        super().__init__("sensing_robot")
        
        # Create callback group that allows execution of callbacks in parallel without restrictions
        callback_group = ReentrantCallbackGroup()

        # Create MoveIt 2 interface
        self.moveit2 = MoveIt2_Sim(
            node=self,
            joint_names=robot.joint_names(),
            base_link_name=robot.base_link_name(),
            end_effector_name=robot.end_effector_name(),
            group_name=robot.MOVE_GROUP_ARM,
            callback_group=callback_group,
        )
        
        # Planner ayarları - Daha iyi planlayıcılar
        self.moveit2.planner_id = "RRTConnectkConfigDefault"

        # Güvenlik için daha düşük hızlar
        self.moveit2.max_velocity = 0.2
        self.moveit2.max_acceleration = 0.5
        
        # ÖNEMLI: Çarpışma önleme ayarları
        self.moveit2.cartesian_avoid_collisions = True
        self.moveit2.cartesian_jump_threshold = 2.0
        
        # Planlama denemeleri ve zaman limiti
        self.planning_attempts = 10
        self.planning_time = 10.0
        
        # Status publishers - Red detection node için
        self.status_publisher = self.create_publisher(String, '/sensing_robot/status', 10)
        self.waypoint_publisher = self.create_publisher(Int32, '/sensing_robot/current_waypoint', 10)
        self.progress_publisher = self.create_publisher(String, '/progress_status', 10)
        self.current_waypoint_index = 0
        self.current_progress = "IDLE"
        
        # Progress durumunu periyodik olarak yayınla (1 Hz)
        self.progress_timer = self.create_timer(1.0, self._publish_progress_callback)
        
    def move_to_position(self, position, orientation=None, cartesian=False, 
                        cartesian_max_step=0.005, cartesian_fraction_threshold=0.95,
                        synchronous=True, planning_attempts=None, planning_time=None):
        """
        Güvenli pozisyon hareketi (çarpışma önleme ile)
        (Bu fonksiyon değiştirilmemiştir)
        """
        if orientation is None:
            orientation = [0.0, 0.0, 0.0, 1.0]
            
        if planning_attempts is None:
            planning_attempts = self.planning_attempts
        if planning_time is None:
            planning_time = self.planning_time
        
        original_attempts = getattr(self.moveit2, 'planning_attempts', 5)
        original_time = getattr(self.moveit2, 'planning_time', 5.0)
        
        try:
            if hasattr(self.moveit2, 'planning_attempts'):
                self.moveit2.planning_attempts = planning_attempts
            if hasattr(self.moveit2, 'planning_time'):
                self.moveit2.planning_time = planning_time
                
            self.moveit2.move_to_pose(
                position=position,
                quat_xyzw=orientation,
                cartesian=cartesian,
                cartesian_max_step=cartesian_max_step,
                cartesian_fraction_threshold=cartesian_fraction_threshold,
            )
            
            if synchronous:
                success = self.moveit2.wait_until_executed()
                return success
            else:
                return True
                
        except Exception as e:
            return False
        finally:
            if hasattr(self.moveit2, 'planning_attempts'):
                self.moveit2.planning_attempts = original_attempts
            if hasattr(self.moveit2, 'planning_time'):
                self.moveit2.planning_time = original_time
    
    def move_to_joint_angles(self, joint_positions, synchronous=True):
        """
        Robotu belirli eklem açılarına (radyan cinsinden) güvenli bir şekilde hareket ettirir.
        """
        try:
            self.moveit2.move_to_configuration(joint_positions)
            
            if synchronous:
                success = self.moveit2.wait_until_executed()
                return success
            else:
                return True
                
        except Exception as e:
            return False

    def safe_pose_sequence(self, positions_list, orientations_list=None, wait_time=2.0):
        """
        Güvenli sıralı POSE hareketi.
        """
        if orientations_list is None:
            orientations_list = [[0.0, 0.0, 0.0, 1.0]] * len(positions_list)
        
        for i, (pos, orient) in enumerate(zip(positions_list, orientations_list)):
            self.move_to_position(pos, orient, cartesian=False, synchronous=True)
            
            if i < len(positions_list) - 1:
                time.sleep(wait_time)

    def safe_joint_sequence(self, list_of_joint_angles, wait_time=2.0, max_retries=3):
        """
        Bir dizi eklem konfigürasyonu arasında güvenli sıralı hareket yapar.
        """
        for i, joint_angles in enumerate(list_of_joint_angles):
            
            # Waypoint indeksini yayınla
            self.publish_waypoint(i)
            
            # Hareket başlıyor - MOVING status
            self.publish_status("MOVING")
            
            # Belirli bir noktaya hareketi denemek için iç döngü
            for attempt in range(max_retries):
                success = self.move_to_joint_angles(joint_angles, synchronous=True)
                
                if success:
                    break
                else:
                    if attempt < max_retries - 1:
                        time.sleep(0.5)

            # Hareket tamamlandı - SCANNING status
            self.publish_status("SCANNING")
            
            # İki ana nokta arasında bekleme süresi
            if i < len(list_of_joint_angles) - 1:
                time.sleep(wait_time)

    def move_home_safe(self):
        """Güvenli ana pozisyona dönüş"""
        safe_intermediate = [-0.3, 0.3, 1.2]
        home_position = [-0.3, 0.3, 0.95]
        home_orientation = [0.0, 0.0, 0.0, 1.0]
        
        self.move_to_position(safe_intermediate, home_orientation, cartesian=False)
        time.sleep(1.0)
        self.move_to_position(home_position, home_orientation, cartesian=True)

    def check_planning_scene(self):
        """Planning scene kontrolü"""
        pass
    
    def publish_status(self, status: str):
        """Robot durumunu yayınla"""
        msg = String()
        msg.data = f"STATUS:{status}|WAYPOINT:{self.current_waypoint_index}"
        self.status_publisher.publish(msg)
    
    def publish_waypoint(self, index: int):
        """Mevcut waypoint indeksini yayınla"""
        self.current_waypoint_index = index
        msg = Int32()
        msg.data = index
        self.waypoint_publisher.publish(msg)
    
    def publish_progress(self, progress: str):
        """Progress durumunu ayarla (timer ile yayınlanacak)"""
        self.current_progress = progress
        msg = String()
        msg.data = progress
        self.progress_publisher.publish(msg)
    
    def _publish_progress_callback(self):
        """Timer callback - progress durumunu periyodik yayınla"""
        msg = String()
        msg.data = self.current_progress
        self.progress_publisher.publish(msg)


def main():
    rclpy.init()
    
    robot_controller = CollisionAwareRobotController()
    
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(robot_controller)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()
    robot_controller.create_rate(1.0).sleep()
    
    robot_controller.check_planning_scene()
    
    home_joints = [1.0, 0.0, -math.pi/2, 0.0, -math.pi/2, 0.0, 0.0]
    pose2_joints = [0.2, math.radians(0), math.radians(-90.0), math.radians(0.0), math.radians(-90.0), math.radians(0.0), math.radians(0.0)]    #senser aldı
    pose3_joints = [1.0, math.radians(86.40), math.radians(-44.74), math.radians(98.25), math.radians(-233.91), math.radians(-90.0), math.radians(90.0)] #sağ alt
    pose4_joints = [1.0, math.radians(86.39), math.radians(-63.12), math.radians(90.33), math.radians(-212.68), math.radians(-90.0), math.radians(90.0)]   #sağ orta
    pose5_joints = [1.0, math.radians(86.37), math.radians(-66.12), math.radians(45.49), math.radians(-164.0), math.radians(-90.0), math.radians(90)]    #sağ üst
    pose6_joints = [1.0, math.radians(94.91), math.radians(-111.64), math.radians(-37.60), math.radians(-28.84), math.radians(-90.0), math.radians(90)] #sol üst
    pose7_joints = [1.0, math.radians(94.84), math.radians(-105.96), math.radians(-90.55), math.radians(-18.31), math.radians(-90.0), math.radians(90)]  #sol orta
    pose8_joints = [1.0, 1.5542166358552454, -2.3765672787497856, -1.8370296687945291, 1.0718900852758073, -1.4855356703907372, math.radians(90)]  #sol alt
    pose9_joints = [1.0, math.radians(109.13), math.radians(-57.99), math.radians(-83.46), math.radians(-40.12), math.radians(-90), math.radians(90)]  #orta üst(düzenle)
    pose10_joints = [1.0, math.radians(108.91), math.radians(-46.56), math.radians(-115.55), math.radians(-19.46), math.radians(-90), math.radians(90)]  #orta orta(düzenle)
    pose11_joints = [1.0, math.radians(54), math.radians(-92), math.radians(-156), math.radians(65), math.radians(-36), math.radians(93)]  #orta alt(düzenle)
    safe_joint_configurations = [
        pose2_joints,   #sensör
        pose6_joints,   #sol üst
        pose9_joints,   #orta üst
        pose5_joints,   #sağ üst
        pose4_joints,   #sağ orta
        pose10_joints,  #orta orta
        pose7_joints,   #sol orta
        pose8_joints,   #sol alt
        pose11_joints,  #orta alt
        pose3_joints    #sağ alt
    ]
    
    loop_counter = 0
    max_loops = 2
    
    try:
        # SENSING MISSION STARTED
        robot_controller.get_logger().info("=== SENSING MISSION STARTED ===")
        
        robot_controller.move_to_joint_angles(home_joints, synchronous=True)
        time.sleep(2.0)
        
        robot_controller.publish_progress("SENSING")
        
        while rclpy.ok() and loop_counter < max_loops:
            loop_counter += 1
            robot_controller.publish_progress("SENSING")
            robot_controller.safe_joint_sequence(safe_joint_configurations, wait_time=1.5, max_retries=3)
            robot_controller.publish_status("CYCLE_COMPLETE")
            time.sleep(3.0)
        
        # SENSING MISSION COMPLETED
        robot_controller.publish_progress("SENSING COMPLETED")
        robot_controller.get_logger().info("=== SENSING MISSION COMPLETED ===")
        
        # Cleaning mission başlayana kadar bekle (30 saniye)
        # Bu süre içinde progress_status yayınlanmaya devam eder
        wait_time = 30
        for i in range(wait_time):
            time.sleep(1.0)
            robot_controller.publish_progress("SENSING COMPLETED")
        
    except KeyboardInterrupt:
        pass
    except Exception as e:
        pass
    
    finally:
        robot_controller.move_to_joint_angles(home_joints, synchronous=True)
        rclpy.shutdown()
        executor_thread.join()


if __name__ == "__main__":
    main()