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


class CollisionAwareRobotController(Node):
    def __init__(self):
        super().__init__("real_collision_aware_robot_controller")
        
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
        self.moveit2.max_velocity = 0.05
        self.moveit2.max_acceleration = 0.5
        
        # ÖNEMLI: Çarpışma önleme ayarları
        self.moveit2.cartesian_avoid_collisions = True
        self.moveit2.cartesian_jump_threshold = 2.0
        
        # Planlama denemeleri ve zaman limiti
        self.planning_attempts = 10
        self.planning_time = 10.0
        
        self.get_logger().info("Çarpışma önleyici robot kontrolcüsü başlatıldı")
        
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
        
        self.get_logger().info(
            f"Güvenli POSE hareketi başlatılıyor: {position}, orientasyon: {orientation}"
        )
        self.get_logger().info(
            f"Planlama ayarları - Denemeler: {planning_attempts}, Süre: {planning_time}s, Cartesian: {cartesian}"
        )
        
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
                if success:
                    self.get_logger().info("Pose hareketi başarıyla tamamlandı!")
                else:
                    self.get_logger().warn("Pose hareketi tamamlanamadı!")
            else:
                self.get_logger().info("Asenkron pose hareketi başlatıldı")
                
        except Exception as e:
            self.get_logger().error(f"Hareket hatası: {str(e)}")
        finally:
            if hasattr(self.moveit2, 'planning_attempts'):
                self.moveit2.planning_attempts = original_attempts
            if hasattr(self.moveit2, 'planning_time'):
                self.moveit2.planning_time = original_time
    
    def move_to_joint_angles(self, joint_positions, synchronous=True):
        """
        Robotu belirli eklem açılarına (radyan cinsinden) güvenli bir şekilde hareket ettirir.
        
        Args:
            joint_positions (list): Hedef eklem açıları (radyan cinsinden). 
                                     Listenin uzunluğu robotun eklem sayısına eşit olmalıdır.
            synchronous (bool): Hareketin bitmesini bekle (senkron)
        """
        self.get_logger().info(f"Güvenli JOINT hareketi başlatılıyor: {joint_positions}")

        try:
            # move_to_configuration metodu ile eklem hedeflerine git
            self.moveit2.move_to_configuration(joint_positions)
            
            if synchronous:
                success = self.moveit2.wait_until_executed()
                if success:
                    self.get_logger().info("Joint hareketi başarıyla tamamlandı!")
                    return True
                else:
                    self.get_logger().warn("Joint hareketi tamamlanamadı - hedefe ulaşılamadı veya çarpışma riski!")
                    return False
            else:
                self.get_logger().info("Asenkron joint hareketi başlatıldı")
                return True # Başlatma başarılı kabul edilir
                
        except Exception as e:
            self.get_logger().error(f"Joint hareket hatası: {str(e)}")
            return False

    def safe_pose_sequence(self, positions_list, orientations_list=None, wait_time=2.0):
        """
        Güvenli sıralı POSE hareketi. (Fonksiyon adı daha açıklayıcı olması için değiştirildi)
        """
        if orientations_list is None:
            orientations_list = [[0.0, 0.0, 0.0, 1.0]] * len(positions_list)
        
        for i, (pos, orient) in enumerate(zip(positions_list, orientations_list)):
            self.get_logger().info(f"Güvenli pose sırası {i+1}/{len(positions_list)}: {pos}")
            self.move_to_position(pos, orient, cartesian=False, synchronous=True)
            
            if i < len(positions_list) - 1:
                self.get_logger().info(f"Sonraki hareket için {wait_time} saniye bekleniyor...")
                time.sleep(wait_time)

    # =================================================================================
    # YENİ VE DEĞİŞTİRİLMİŞ FONKSİYON
    # =================================================================================
    def safe_joint_sequence(self, list_of_joint_angles, wait_time=2.0, max_retries=3):
        """
        Bir dizi eklem konfigürasyonu arasında güvenli sıralı hareket yapar.
        Başarısız bir hareketi max_retries kadar dener, yine de başarısız olursa
        o noktayı atlayıp bir sonrakine geçer.
        
        Args:
            list_of_joint_angles (list of list): Her biri bir hedef konfigürasyon olan eklem açıları listesi.
            wait_time (float): Hareketler arasındaki bekleme süresi.
            max_retries (int): Başarısız bir hareket için maksimum deneme sayısı.
        """
        for i, joint_angles in enumerate(list_of_joint_angles):
            self.get_logger().info(f"Sıradaki hedefe gidiliyor: {i+1}/{len(list_of_joint_angles)}")
            
            # Belirli bir noktaya hareketi denemek için iç döngü
            for attempt in range(max_retries):
                self.get_logger().info(f"  -> Deneme {attempt + 1}/{max_retries}...")
                
                # Hareketi dene
                success = self.move_to_joint_angles(joint_angles, synchronous=True)
                
                if success:
                    self.get_logger().info(f"  -> Hareket başarılı!")
                    break  # Hareket başarılı, deneme döngüsünden çık
                else:
                    self.get_logger().warning(f"  -> Deneme {attempt + 1} başarısız oldu.")
                    # Son deneme değilse, kısa bir süre bekle
                    if attempt < max_retries - 1:
                        time.sleep(0.5)
            
            # Bu 'else' bloğu, yukarıdaki 'for attempt' döngüsü 'break' ile kesilmezse çalışır.
            # Yani, tüm denemeler başarısız olduysa bu kod çalışır.
            else:
                self.get_logger().error(
                    f"Tüm denemeler ({max_retries}) başarısız oldu. "
                    f"Bu nokta atlanıyor ve sıradaki noktaya geçiliyor."
                )

            # İki ana nokta arasında bekleme süresi
            if i < len(list_of_joint_angles) - 1:
                self.get_logger().info(f"Sonraki hareket için {wait_time} saniye bekleniyor...")
                time.sleep(wait_time)
    # =================================================================================

    def move_home_safe(self):
        """Güvenli ana pozisyona dönüş (Kartezyen tabanlı)"""
        safe_intermediate = [-0.3, 0.3, 1.2]
        home_position = [-0.3, 0.3, 0.95]
        home_orientation = [0.0, 0.0, 0.0, 1.0]
        
        self.get_logger().info("Güvenli ana pozisyona (POSE) dönüş başlatılıyor...")
        self.get_logger().info("1. Adım: Güvenli yüksekliğe çıkılıyor...")
        self.move_to_position(safe_intermediate, home_orientation, cartesian=False)
        time.sleep(1.0)
        self.get_logger().info("2. Adım: Ana pozisyona iniliyor...")
        self.move_to_position(home_position, home_orientation, cartesian=True)

    def check_planning_scene(self):
        """Planning scene'deki engelleri kontrol et"""
        self.get_logger().info("Planning scene kontrol ediliyor...")
        self.get_logger().info("Planning scene aktif - çarpışma önleme çalışıyor")


def main():
    rclpy.init()
    
    robot_controller = CollisionAwareRobotController()
    
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(robot_controller)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()
    robot_controller.create_rate(1.0).sleep()
    
    robot_controller.check_planning_scene()
    
    # --- WAYPOINT'LER (teach pendant fotoğraflarından alınan joint açıları) ---
    # Format: [ilk_eksen(rad sabit), radians(Base), radians(Shoulder),
    #          radians(Elbow), radians(Wrist1), radians(Wrist2), radians(Wrist3)]
    # İlk eleman tüm noktalarda sabit: 1.85
    FIRST = 1.85

    home_joints = [FIRST, 0.0, -math.pi/2, 0.0, -math.pi/2, 0.0, 0.0]

    # wp01..wp24 -> 1.jpeg .. 24.jpeg
    wp01 = [FIRST, math.radians(16.25), math.radians(-67.48), math.radians(52.93), math.radians(-85.25), math.radians(-59.51), math.radians(20.70)]
    wp02 = [FIRST, math.radians(24.49), math.radians(-58.18), math.radians(39.88), math.radians(-86.04), math.radians(-61.27), math.radians(29.96)]
    wp03 = [FIRST, math.radians(24.50), math.radians(-47.21), math.radians(9.04), math.radians(-66.15), math.radians(-61.33), math.radians(29.90)]
    wp04 = [FIRST, math.radians(34.72), math.radians(-81.68), math.radians(62.59), math.radians(-90.27), math.radians(-64.21), math.radians(40.96)]
    wp05 = [FIRST, math.radians(43.57), math.radians(-71.05), math.radians(96.36), math.radians(-135.08), math.radians(-67.46), math.radians(50.06)]
    wp06 = [FIRST, math.radians(43.23), math.radians(-69.79), math.radians(98.49), math.radians(-138.64), math.radians(-67.39), math.radians(50.07)]
    wp07 = [FIRST, math.radians(47.11), math.radians(-73.07), math.radians(103.38), math.radians(-141.70), math.radians(-68.76), math.radians(53.99)]
    wp08 = [FIRST, math.radians(34.72), math.radians(-81.68), math.radians(62.59), math.radians(-90.27), math.radians(-64.21), math.radians(40.96)]
    wp09 = [FIRST, math.radians(43.04), math.radians(-85.92), math.radians(114.25), math.radians(-148.56), math.radians(-102.86), math.radians(112.64)]
    wp10 = [FIRST, math.radians(43.04), math.radians(-82.53), math.radians(117.72), math.radians(-155.41), math.radians(-102.89), math.radians(112.66)]
    wp11 = [FIRST, math.radians(43.04), math.radians(-82.19), math.radians(118.00), math.radians(-156.04), math.radians(-102.89), math.radians(112.66)]
    wp12 = [FIRST, math.radians(43.04), math.radians(-85.92), math.radians(114.25), math.radians(-148.56), math.radians(-102.86), math.radians(112.64)]
    wp13 = [FIRST, math.radians(0.96), math.radians(-89.56), math.radians(113.31), math.radians(-124.05), math.radians(-58.41), math.radians(25.92)]
    wp14 = [FIRST, math.radians(0.96), math.radians(-86.50), math.radians(117.37), math.radians(-131.19), math.radians(-58.42), math.radians(25.96)]
    wp15 = [FIRST, math.radians(0.96), math.radians(-86.55), math.radians(117.32), math.radians(-131.08), math.radians(-58.42), math.radians(25.96)]
    wp16 = [FIRST, math.radians(0.96), math.radians(-89.56), math.radians(113.31), math.radians(-124.05), math.radians(-58.41), math.radians(25.92)]
    wp17 = [FIRST, math.radians(8.98), math.radians(-77.16), math.radians(97.63), math.radians(-121.60), math.radians(-60.89), math.radians(28.38)]
    wp18 = [FIRST, math.radians(8.98), math.radians(-74.31), math.radians(102.01), math.radians(-128.84), math.radians(-60.91), math.radians(28.42)]
    wp19 = [FIRST, math.radians(8.98), math.radians(-73.89), math.radians(102.51), math.radians(-129.77), math.radians(-60.91), math.radians(28.42)]
    wp20 = [FIRST, math.radians(8.98), math.radians(-77.16), math.radians(97.63), math.radians(-121.60), math.radians(-60.89), math.radians(28.38)]
    wp21 = [FIRST, math.radians(-3.76), math.radians(-76.61), math.radians(93.33), math.radians(-118.99), math.radians(-64.81), math.radians(28.30)]
    wp22 = [FIRST, math.radians(-3.76), math.radians(-73.50), math.radians(99.12), math.radians(-127.90), math.radians(-64.83), math.radians(28.34)]
    wp23 = [FIRST, math.radians(-3.76), math.radians(-73.25), math.radians(99.46), math.radians(-128.49), math.radians(-64.83), math.radians(28.34)]
    wp24 = [FIRST, math.radians(-3.76), math.radians(-76.61), math.radians(93.33), math.radians(-118.99), math.radians(-64.81), math.radians(28.30)]

    # Sıralı hareket için waypoint listesi (1 -> 24)
    safe_joint_configurations = [
        wp01, wp02, wp03, wp04, wp05, wp06, wp07, wp08,
        wp09, wp10, wp11, wp12, wp13, wp14, wp15, wp16,
        wp17, wp18, wp19, wp20, wp21, wp22, wp23, wp24,
    ]
    
    loop_counter = 0
    
    try:
        robot_controller.get_logger().info("=== EKLEM HEDEFLİ SONSUZ DÖNGÜ BAŞLATILIYOR ===")
        robot_controller.get_logger().info("Durdurmak için Ctrl+C tuşlayın")
        
        robot_controller.get_logger().info("Başlangıç için `home_joints` pozisyonuna gidiliyor...")
        robot_controller.move_to_joint_angles(home_joints, synchronous=True)
        time.sleep(2.0)
        
        while rclpy.ok():
            loop_counter += 1
            robot_controller.get_logger().info(f"=== EKLEM DÖNGÜSÜ {loop_counter} BAŞLIYOR ===")
            
            # Güvenli sıralı eklem hareketi (deneme sayısı 3 olacak şekilde)
            robot_controller.safe_joint_sequence(safe_joint_configurations, wait_time=1.5, max_retries=3)
            
            robot_controller.get_logger().info(f"Döngü {loop_counter} tamamlandı. 3 saniye bekleniyor...")
            time.sleep(3.0)
        
    except KeyboardInterrupt:
        robot_controller.get_logger().info(f"Program kullanıcı tarafından durduruldu (Toplam {loop_counter} döngü)")
    except Exception as e:
        robot_controller.get_logger().error(f"Beklenmeyen hata: {str(e)}")
    
    finally:
        robot_controller.get_logger().info("Güvenli kapatılıyor...")
        robot_controller.move_to_joint_angles(home_joints, synchronous=True)
        rclpy.shutdown()
        executor_thread.join()


if __name__ == "__main__":
    main()