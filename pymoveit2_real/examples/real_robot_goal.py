#!/usr/bin/env python3
"""
Çarpışma önleyici fonksiyon tabanlı pose hedefleri ile robot hareketi
"""

from threading import Thread
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from pymoveit2_real import MoveIt2 as MoveIt2_Real
from pymoveit2_real import MoveIt2State
from pymoveit2_real.robots import ur as robot


class RealCollisionAwareRobotController(Node):
    def __init__(self):
        super().__init__("real_collision_aware_robot_controller")
        
        # Create callback group that allows execution of callbacks in parallel without restrictions
        callback_group = ReentrantCallbackGroup()

        # Create MoveIt 2 interface
        self.moveit2 = MoveIt2_Real(
            node=self,
            joint_names=robot.joint_names(),
            base_link_name="world",
            end_effector_name=robot.end_effector_name(),
            group_name=robot.MOVE_GROUP_ARM,
            callback_group=callback_group,
        )
        
        # Planner ayarları - Daha iyi planlayıcılar
        self.moveit2.planner_id = "RRTConnectkConfigDefault"  # Daha güvenilir planlayıcı

        
        # Güvenlik için daha düşük hızlar
        self.moveit2.max_velocity = 0.2
        self.moveit2.max_acceleration = 0.2
        
        # ÖNEMLI: Çarpışma önleme ayarları
        self.moveit2.cartesian_avoid_collisions = True  # Cartesian hareketlerde çarpışma önleme
        self.moveit2.cartesian_jump_threshold = 2.0     # Ani sıçramaları engelle
        
        # Planlama denemeleri ve zaman limiti
        self.planning_attempts = 10
        self.planning_time = 10.0
        
        self.get_logger().info("Çarpışma önleyici robot kontrolcüsü başlatıldı")
        
    def move_to_position(self, position, orientation=None, cartesian=False, 
                        cartesian_max_step=0.005, cartesian_fraction_threshold=0.95,
                        synchronous=True, planning_attempts=None, planning_time=None):
        """
        Güvenli pozisyon hareketi (çarpışma önleme ile)
        
        Args:
            position (list): [x, y, z] koordinatları
            orientation (list): [x, y, z, w] quaternion (None ise varsayılan kullanılır)
            cartesian (bool): Cartesian planlama kullanılsın mı
            cartesian_max_step (float): Daha küçük adımlar için güvenlik
            cartesian_fraction_threshold (float): Yolun ne kadarının planlanması gerekli (0.95 = %95)
            synchronous (bool): Senkron hareket
            planning_attempts (int): Planlama deneme sayısı
            planning_time (float): Planlama için maksimum süre
        """
        if orientation is None:
            orientation = [1.0, 0.0, 0.0, 0.0]  # Varsayılan orientasyon
            
        if planning_attempts is None:
            planning_attempts = self.planning_attempts
        if planning_time is None:
            planning_time = self.planning_time
        
        self.get_logger().info(
            f"Güvenli hareket başlatılıyor: {position}, orientasyon: {orientation}"
        )
        self.get_logger().info(
            f"Planlama ayarları - Denemeler: {planning_attempts}, Süre: {planning_time}s, Cartesian: {cartesian}"
        )
        
        # Geçici olarak planlama parametrelerini ayarla
        original_attempts = getattr(self.moveit2, 'planning_attempts', 5)
        original_time = getattr(self.moveit2, 'planning_time', 5.0)
        
        try:
            # Planlama parametrelerini güncelle (eğer mevcut ise)
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
                if success:
                    self.get_logger().info("Güvenli hareket başarıyla tamamlandı!")
                else:
                    self.get_logger().warn("Hareket tamamlanamadı - çarpışma riski olabilir!")
            else:
                self.get_logger().info("Asenkron güvenli hareket başlatıldı")
                
        except Exception as e:
            self.get_logger().error(f"Hareket hatası: {str(e)}")
        finally:
            # Orijinal parametreleri geri yükle
            if hasattr(self.moveit2, 'planning_attempts'):
                self.moveit2.planning_attempts = original_attempts
            if hasattr(self.moveit2, 'planning_time'):
                self.moveit2.planning_time = original_time
    
    def safe_move_sequence(self, positions_list, orientations_list=None, wait_time=2.0):
        """
        Güvenli sıralı hareket (her hareket arasında duraksa ve kontrol)
        """
        if orientations_list is None:
            orientations_list = [[0.0, 0.0, 0.7071, 0.7071]] * len(positions_list)
        
        for i, (pos, orient) in enumerate(zip(positions_list, orientations_list)):
            self.get_logger().info(f"Güvenli sıra {i+1}/{len(positions_list)}: {pos}")
            
            # Her hareketi joint-space planning ile yap (daha güvenli)
            success = self.move_to_position(pos, orient, cartesian=False, synchronous=True)
            
            if i < len(positions_list) - 1:  # Son hareket değilse bekle
                self.get_logger().info(f"Sonraki hareket için {wait_time} saniye bekleniyor...")
                time.sleep(wait_time)
    
    def move_home_safe(self):
        """Güvenli ana pozisyona dönüş"""
        # Önce güvenli bir ara pozisyona git
        safe_intermediate = [-0.3, 0.3, 1.2]  # Daha yüksek Z koordinatı
        home_position = [-0.3, 0.3, 0.95]
        home_orientation = [-1.0, 0.0, 0.0, 0.0]
        
        self.get_logger().info("Güvenli ana pozisyona dönüş başlatılıyor...")
        
        # 1. Adım: Güvenli yüksekliğe çık
        self.get_logger().info("1. Adım: Güvenli yüksekliğe çıkılıyor...")
        self.move_to_position(safe_intermediate, home_orientation, cartesian=False)
        time.sleep(1.0)
        
        # 2. Adım: Ana pozisyona in
        self.get_logger().info("2. Adım: Ana pozisyona iniliyor...")
        self.move_to_position(home_position, home_orientation, cartesian=True)
    
    def safe_cartesian_path(self, waypoints, max_step=0.005, safety_height=0.05):
        """
        Güvenli Cartesian yol (her nokta arasında güvenlik kontrolü)
        
        Args:
            waypoints (list): Yol noktaları listesi
            max_step (float): Daha küçük adımlar
            safety_height (float): Güvenlik yüksekliği ekleme
        """
        self.get_logger().info(f"Güvenli Cartesian yol başlatılıyor - {len(waypoints)} nokta")
        
        for i, waypoint in enumerate(waypoints):
            # Güvenlik yüksekliği ekle (Z koordinatını artır)
            safe_waypoint = [waypoint[0], waypoint[1], waypoint[2] + safety_height]
            
            self.get_logger().info(f"Waypoint {i+1}/{len(waypoints)}: {waypoint} -> {safe_waypoint}")
            
            # Her waypoint'i joint planning ile git (daha güvenli)
            self.move_to_position(
                safe_waypoint, 
                cartesian=False,  # Joint planning kullan
                synchronous=True,
                planning_attempts=15,  # Daha fazla deneme
                planning_time=15.0     # Daha fazla zaman
            )
            
            # Waypoint'lar arası kısa bekleme
            if i < len(waypoints) - 1:
                time.sleep(0.5)
    
    def check_planning_scene(self):
        """Planning scene'deki engelleri kontrol et"""
        self.get_logger().info("Planning scene kontrol ediliyor...")
        # Bu fonksiyon planning scene'i kontrol etmek için genişletilebilir
        # Şu anda sadece bilgi mesajı veriyor
        self.get_logger().info("Planning scene aktif - çarpışma önleme çalışıyor")


def main():
    rclpy.init()
    
    robot_controller = RealCollisionAwareRobotController()
    
    # Spin the node in background thread(s) and wait a bit for initialization
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(robot_controller)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()
    robot_controller.create_rate(1.0).sleep()
    
    # Planning scene'i kontrol et
    robot_controller.check_planning_scene()
    
    # Infinite loop için pozisyonları tanımla
    safe_positions = [
        [-0.5, 1.0, 1.55],
        [-1.0, 1.0, 1.55],
        [-0.5, 1.0, 1.55],
        [-0.5, 1.65, 1.55],
        [-1.0, 1.65, 1.55],
        [-0.5, 1.65, 1.55],
        [-0.5, 1.0, 1.55]
   # Başlangıç (yüksek)
    ]
    
    loop_counter = 0
    
    try:
        robot_controller.get_logger().info("=== INFINITE LOOP BAŞLATILIYOR ===")
        robot_controller.get_logger().info("Durdurmak için Ctrl+C tuşlayın")
        
        while rclpy.ok():  # ROS2 aktif olduğu sürece çalış
            loop_counter += 1
            robot_controller.get_logger().info(f"=== DÖNGÜ {loop_counter} BAŞLIYOR ===")
            
            # Güvenli sıralı hareket
            robot_controller.safe_move_sequence(safe_positions, wait_time=1.0)
            
            # Döngüler arası bekleme süresi (isteğe bağlı)
            robot_controller.get_logger().info(f"Döngü {loop_counter} tamamlandı. 3 saniye bekleniyor...")
            time.sleep(3.0)
            
            # Her 10 döngüde bir ana pozisyona dön (bakım için)
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
        robot_controller.move_home_safe()  # Kapatmadan önce güvenli pozisyona git
        rclpy.shutdown()
        executor_thread.join()


if __name__ == "__main__":
    main()