#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from threading import Thread
import time

from geometry_msgs.msg import Pose, PoseStamped
from pymoveit2 import MoveIt2
from pymoveit2.robots import combined_ur as robot

# GEREKLİ KÜTÜPHANELER
import tf2_ros
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose

class DualArmController(Node):
    """
    İki robot kolu için IK hesaplayan ve ardından bu hedeflere
    eş zamanlı olarak hareket eden bir ROS2 Düğümü.
    """
    def __init__(self):
        super().__init__("dual_arm_controller_node")
        self.world_frame_id = "world"
        # --- TF2 Listener Kurulumu ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.get_logger().info("TF2 Listener başlatıldı.")
        # --- Callback Grupları ve Executor Kurulumu ---
        # Her arayüz için ayrı iletişim kanalları
        self.callback_group_robot1 = MutuallyExclusiveCallbackGroup()
        self.callback_group_robot2 = MutuallyExclusiveCallbackGroup()
        self.callback_group_combined = ReentrantCallbackGroup() # Ortak hareket için

        # --- MoveIt2 Arayüzlerini Oluşturma ---
    
        # Robot 1'i kendi callback grubuna ata
        self.moveit2_robot1 = MoveIt2(
            node=self,
            joint_names=robot.robot1_joint_names(),
            base_link_name=robot.robot1_base_link_name(),
            end_effector_name=robot.robot1_end_effector_name(),
            group_name="robot1",
            callback_group=self.callback_group_robot1, # Robot 1 için özel grup
        )
        self.get_logger().info("Robot 1 için MoveIt2 arayüzü başarıyla başlatıldı.")

        # Robot 2'yi kendi callback grubuna ata
        self.moveit2_robot2 = MoveIt2(
            node=self,
            joint_names=robot.robot2_joint_names(),
            base_link_name=robot.robot2_base_link_name(),
            end_effector_name=robot.robot2_end_effector_name(),
            group_name="robot2",
            callback_group=self.callback_group_robot2, # Robot 2 için özel grup
        )
        self.get_logger().info("Robot 2 için MoveIt2 arayüzü başarıyla başlatıldı.")

        self.moveit2_combined = MoveIt2(
            node=self,
            joint_names=robot.joint_names(),  # Tüm joint'ler (robot1 + robot2)
            base_link_name=robot.base_link_name(),
            end_effector_name=robot.end_effector_name(),
            group_name="combinedgroup",  # "combined"
            callback_group=self.callback_group_combined,
        )
        self.get_logger().info("Combined Robot için MoveIt2 arayüzü başarıyla başlatıldı.")
        self._setup_safety_parameters(self.moveit2_combined)
        self.get_logger().info("Tüm MoveIt2 arayüzleri başarıyla başlatıldı.")
    

    def _setup_safety_parameters(self, moveit2_interface):
        """Güvenlik ve planlama parametrelerini ayarlar."""
        moveit2_interface.planner_id = "RRTstarkConfigDefault"
        moveit2_interface.max_velocity = 0.3
        moveit2_interface.max_acceleration = 0.3
        moveit2_interface.cartesian_avoid_collisions = True
        moveit2_interface.cartesian_jump_threshold = 5.0
        # Planlama süresini artır
        moveit2_interface.planning_time = 5.0
    
    def create_pose(self, position, orientation=None):
        """Verilen pozisyon ve oryantasyondan Pose oluşturur."""
        pose = Pose()
        pose.position.x = position[0]
        pose.position.y = position[1]
        pose.position.z = position[2]
        
        if orientation is None:
            # Varsayılan oryantasyon (180 derece x ekseni etrafında döndürülmüş)
            pose.orientation.x = 1.0
            pose.orientation.y = 0.0
            pose.orientation.z = 0.0
            pose.orientation.w = 0.0
        else:
            pose.orientation.x = orientation[0]
            pose.orientation.y = orientation[1]
            pose.orientation.z = orientation[2]
            pose.orientation.w = orientation[3]
        
        return pose
    
    def transform_pose_to_base_frame(self, target_pose_world: Pose, target_frame: str):
        """Hedef pozu world'den robotun base_link'ine dönüştürür."""
        pose_stamped_world = PoseStamped()
        pose_stamped_world.header.frame_id = self.world_frame_id
        pose_stamped_world.header.stamp = self.get_clock().now().to_msg()
        pose_stamped_world.pose = target_pose_world

        try:
            # tf_buffer.transform metodunu kullanarak dönüşümü tek adımda yap
            pose_stamped_base = self.tf_buffer.transform(
                pose_stamped_world, target_frame, timeout=rclpy.duration.Duration(seconds=1.0)
            )
            self.get_logger().info(f"Poz, '{self.world_frame_id}' frame'inden '{target_frame}' frame'ine başarıyla dönüştürüldü.")
            return pose_stamped_base.pose
        except (tf2_ros.TransformException) as e:
            self.get_logger().error(f"TF dönüşüm hatası: {e}")
            return None

    def solve_ik_with_manual_transform(self, target_pose_r1_world, target_pose_r2_world):
        self.get_logger().info("--- Manuel TF Dönüşümü ile IK Çözme Başlatılıyor ---")

        target_pose_r1_base = self.transform_pose_to_base_frame(target_pose_r1_world, self.moveit2_robot1.base_link_name)
        target_pose_r2_base = self.transform_pose_to_base_frame(target_pose_r2_world, self.moveit2_robot2.base_link_name)
        
        future_r1, future_r2 = None, None

        if target_pose_r1_base:
            future_r1 = self.moveit2_robot1.compute_ik_async(
                position=[target_pose_r1_base.position.x, target_pose_r1_base.position.y, target_pose_r1_base.position.z],
                quat_xyzw=[target_pose_r1_base.orientation.x, target_pose_r1_base.orientation.y, target_pose_r1_base.orientation.z, target_pose_r1_base.orientation.w]
            )
        
        if target_pose_r2_base:
            future_r2 = self.moveit2_robot2.compute_ik_async(
                position=[target_pose_r2_base.position.x, target_pose_r2_base.position.y, target_pose_r2_base.position.z],
                quat_xyzw=[target_pose_r2_base.orientation.x, target_pose_r2_base.orientation.y, target_pose_r2_base.orientation.z, target_pose_r2_base.orientation.w]
            )

        # İki IK çözümünü de bekle
        start_time = time.time()
        while rclpy.ok() and time.time() - start_time < 5.0: # 5 saniye timeout
            r1_done = future_r1.done() if future_r1 else True
            r2_done = future_r2.done() if future_r2 else True
            if r1_done and r2_done:
                break
            time.sleep(0.01)

        robot1_joints = self._extract_joint_values(future_r1.result(), robot.robot1_joint_names(), "Robot 1") if future_r1 and future_r1.done() else None
        robot2_joints = self._extract_joint_values(future_r2.result(), robot.robot2_joint_names(), "Robot 2") if future_r2 and future_r2.done() else None
        
        if robot1_joints or robot2_joints:
            self.get_logger().info("En az bir IK çözümü başarılı. Hareket başlatılıyor.")
            return self.move_robots_to_joint_positions(robot1_joints, robot2_joints)
        else:
            self.get_logger().error("Her iki robot için de IK çözümü başarısız.")
            return False

    def _extract_joint_values(self, ik_result, expected_joint_names, robot_id):
        """IK sonucundan eklem pozisyonlarını doğru sırada çıkaran yardımcı fonksiyon."""
        if not ik_result or ik_result.error_code.val != 1:
            self.get_logger().error(f"{robot_id} IK Çözümü BAŞARISIZ!")
            return None

        self.get_logger().info(f"{robot_id} IK Çözümü Başarılı!")
        joint_state = ik_result.solution.joint_state
        
        # Gelen sonucu beklenen eklem sırasına göre dizecek bir sözlük
        position_dict = {name: pos for name, pos in zip(joint_state.name, joint_state.position)}
        
        # Beklenen sıraya göre listeyi oluştur
        ordered_positions = [position_dict[name] for name in expected_joint_names]
        
        return ordered_positions

    def move_robots_to_joint_positions(self, robot1_joints: list, robot2_joints: list, max_retries=3):
        """İki robotu verilen eklem pozisyonlarına hareket ettirir. None olan robot yerinde kalır."""
        
        # Mevcut robot pozisyonlarını al
        if robot1_joints is None:
            # Robot 1 için mevcut pozisyonu kullan (yerinde kal)
            current_joint_state = self.moveit2_combined.joint_state
            if current_joint_state is None:
                self.get_logger().error("Robot 1 mevcut pozisyonu alınamadı!")
                return False
            robot1_joints = list(current_joint_state.position[:6])  # İlk 6 joint Robot 1
            self.get_logger().info("Robot 1 yerinde kalacak (mevcut pozisyon kullanılıyor)")
        
        if robot2_joints is None:
            # Robot 2 için mevcut pozisyonu kullan (yerinde kal)
            current_joint_state = self.moveit2_combined.joint_state
            if current_joint_state is None:
                self.get_logger().error("Robot 2 mevcut pozisyonu alınamadı!")
                return False
            robot2_joints = list(current_joint_state.position[6:12])  # Son 6 joint Robot 2
            self.get_logger().info("Robot 2 yerinde kalacak (mevcut pozisyon kullanılıyor)")

        if len(robot1_joints) != 6 or len(robot2_joints) != 6:
            self.get_logger().error("Hatalı eklem sayısı! Hareket iptal edildi.")
            return False

        # SRDF dosyanızdaki sıraya göre birleştir
        combined_joints = robot1_joints + robot2_joints
        
        for attempt in range(max_retries):
            self.get_logger().info(f"Hareket denemesi {attempt + 1}/{max_retries}")
            self.get_logger().info("Birleşik gruba hareket komutu gönderiliyor...")
            
            # Farklı planlayıcıları dene
            planners = ["RRTConnect", "RRT", "PRM"]
            self.moveit2_combined.planner_id = planners[attempt % len(planners)]
            self.get_logger().info(f"Kullanılan planlayıcı: {self.moveit2_combined.planner_id}")
            
            try:
                self.moveit2_combined.move_to_configuration(combined_joints)
                success = self.moveit2_combined.wait_until_executed()

                if success:
                    self.get_logger().info("Hareket BAŞARILI!")
                    return True
                else:
                    self.get_logger().warn(f"Hareket başarısız (planner: {self.moveit2_combined.planner_id})")
            except Exception as e:
                self.get_logger().warn(f"Hareket hatası: {str(e)}")
            
            # Başarısızlık durumunda kısa bekleme
            if attempt < max_retries - 1:
                time.sleep(1.0)
        
        self.get_logger().error("Tüm hareket denemeleri başarısız!")
        return False
        
    def move_to_home(self):
        combined_joints_home= [0.0, -1.57, 0.0, -1.57, 0.0, 0.0 ] + [0.0, -1.57, 0.0, -1.57, 0.0, 0.0 ]
        try:
            self.moveit2_combined.move_to_configuration(combined_joints_home)
            success = self.moveit2_combined.wait_until_executed()
            if success:
                self.get_logger().info("Eş zamanlı hareket BAŞARILI!")
                return True
            else:
                self.get_logger().warn(f"Hareket başarısız (planner: {self.moveit2_combined.planner_id})")
        except Exception as e:
            self.get_logger().warn(f"Hareket hatası: {str(e)}")
    
    def move_starter(self):
        combined_joints_starter= [-0.8, -1.57, 0.0, -1.57, 0.0, 0.0 ] + [+0.8, -1.57, 0.0, -1.57, 0.0, 0.0 ]
        try:
            self.moveit2_combined.move_to_configuration(combined_joints_starter)
            success = self.moveit2_combined.wait_until_executed()
            if success:
                self.get_logger().info("Eş zamanlı hareket BAŞARILI!")
                return True
            else:
                self.get_logger().warn(f"Hareket başarısız (planner: {self.moveit2_combined.planner_id})")
        except Exception as e:
            self.get_logger().warn(f"Hareket hatası: {str(e)}")

def main():
    rclpy.init()
    controller_node = DualArmController()
    
    # Not: TF Listener'ın kendi spin thread'i olduğu için executor'a gerek kalmayabilir
    # ancak callback gruplarını yönetmek için hala gerekli.
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(controller_node)
    
    # spin'i ana thread'de çalıştırmak yerine ayrı bir thread'de çalıştırmak daha iyi bir pratik
    executor_thread = Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    
    time.sleep(3.0) # TF ve diğer servislerin başlaması için bekleme

    loop_counter = 0
    robot1_waypoints = [[-0.3,0.3,1.2],[-0.7,0.3,1.2]]
    # ,[-0.3,0.3,1.2],[-0.3,0.0,1.2],[-0.3,-0.3,1.2],[-0.7,-0.3,1.2],[-0.3,-0.3,1.2]]
    robot2_waypoints = [[-1.7,0.3,1.4],[-1.3,0.3,1.4]]
    # ,[0.3,0.3,1.1],[0.0,0.3,1.1],[0.3,-0.3,1.1],[0.3,-0.7,1.1],[0.3,0.3,1.1]]
    robot1_poses = [controller_node.create_pose(p) for p in robot1_waypoints]
    robot2_poses = [controller_node.create_pose(p) for p in robot2_waypoints]

    try:
        while rclpy.ok():
            loop_counter += 1
            controller_node.get_logger().info(f"=== DÖNGÜ {loop_counter} BAŞLIYOR ===")
            controller_node.move_starter()
            for i, (r1_pose, r2_pose) in enumerate(zip(robot1_poses, robot2_poses)):
                if not rclpy.ok(): break
                controller_node.get_logger().info(f"Döngü {loop_counter} - Waypoint {i+1}")
                success = controller_node.solve_ik_with_manual_transform(r1_pose, r2_pose)
                if not success:
                    controller_node.get_logger().warn(f"Waypoint {i+1} başarısız! Eve dönülüyor.")
                    controller_node.move_to_home()
                    continue
                time.sleep(1.0)
            controller_node.get_logger().info(f"=== DÖNGÜ {loop_counter} TAMAMLANDI ===")
            time.sleep(2.0)
    except KeyboardInterrupt:
        controller_node.get_logger().info("Kullanıcı tarafından durduruldu.")
    except Exception as e:
        controller_node.get_logger().error(f"Beklenmeyen bir hata oluştu: {e}")
        import traceback
        traceback.print_exc()
    finally:
        rclpy.shutdown()
        executor_thread.join()

if __name__ == "__main__":
    main()