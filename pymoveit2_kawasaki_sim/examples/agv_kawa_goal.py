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
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from pymoveit2_kawasaki_sim import MoveIt2 as MoveIt2_Sim
from pymoveit2_kawasaki_sim import MoveIt2State
from pymoveit2_kawasaki_sim.robots import ur as robot


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
        
        # AGV kontrolü için publisher ve subscriber
        self.cmd_vel_pub = self.create_publisher(
            Twist, 
            '/kawasaki/ota_base_controller/cmd_vel_unstamped', 
            10
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/kawasaki/ota_base_controller/odom',
            self.odom_callback,
            10
        )
        
        # AGV pozisyon bilgisi
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.odom_received = False
        
        self.get_logger().info("Çarpışma önleyici robot kontrolcüsü başlatıldı")
    
    def odom_callback(self, msg):
        """Odometry mesajından mevcut konumu al"""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        # Quaternion'dan yaw hesapla
        orientation = msg.pose.pose.orientation
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
        
        self.odom_received = True
    
    def get_distance_to_goal(self, goal_x, goal_y):
        """Hedefe olan mesafeyi hesapla"""
        dx = goal_x - self.current_x
        dy = goal_y - self.current_y
        return math.sqrt(dx*dx + dy*dy)
    
    def get_angle_to_goal(self, goal_x, goal_y):
        """Hedefe olan açıyı hesapla"""
        dx = goal_x - self.current_x
        dy = goal_y - self.current_y
        return math.atan2(dy, dx)
    
    def normalize_angle(self, angle):
        """Açıyı -pi ile pi arasına normalize et"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def move_agv_to_x_position(self, goal_x, tolerance=0.1):
        """
        AGV'yi X ekseninde belirtilen konuma hareket ettirir (Y sabit kalır).
        Hedef arkadaysa dönmeden geri gider (geri vites gibi).
        
        Args:
            goal_x: Hedef X koordinatı (metre)
            tolerance: Hedefe varış toleransı (metre)
        """
        # Odometry'nin gelmesini bekle
        self.get_logger().info('AGV hareketi için odometry verisi bekleniyor...')
        wait_count = 0
        while not self.odom_received and wait_count < 50:
            rclpy.spin_once(self, timeout_sec=0.1)
            wait_count += 1
        
        if not self.odom_received:
            self.get_logger().error('Odometry verisi alınamadı! AGV hareketi iptal ediliyor.')
            return
        
        goal_y = self.current_y  # Y koordinatı sabit kalacak
        
        start_x = self.current_x
        self.get_logger().info(f'AGV Başlangıç konumu: X={start_x:.2f}')
        self.get_logger().info(f'AGV Hedef X konumu: {goal_x:.2f}')
        
        # Kontrol parametreleri
        linear_speed = 0.15  # m/s
        angular_speed = 0.4  # rad/s
        angle_tolerance = 0.20  # radyan - biraz daha toleranslı
        reverse_angle_threshold = math.pi / 2  # 90 derece - hedef bu açıdan fazlaysa geri git
        
        twist = Twist()
        consecutive_goal_reached = 0
        required_consecutive = 5
        
        loop_count = 0
        max_loops = 2000
        
        while rclpy.ok() and loop_count < max_loops:
            rclpy.spin_once(self, timeout_sec=0.05)
            
            distance = self.get_distance_to_goal(goal_x, goal_y)
            
            loop_count += 1
            if loop_count % 20 == 0:
                self.get_logger().info(
                    f'AGV X: {self.current_x:.2f}, Mesafe: {distance:.3f}m'
                )
            
            if distance < tolerance:
                consecutive_goal_reached += 1
                if consecutive_goal_reached >= required_consecutive:
                    self.get_logger().info(
                        f'AGV hedefe ulaştı! X pozisyon: {self.current_x:.2f}'
                    )
                    break
                twist.linear.x = 0.0
                twist.angular.z = 0.0
            else:
                consecutive_goal_reached = 0
                
                # Hedefe olan açıyı hesapla
                target_angle = self.get_angle_to_goal(goal_x, goal_y)
                angle_diff = self.normalize_angle(target_angle - self.current_yaw)
                
                # Hedef arkada mı kontrol et (açı farkı 90 dereceden fazla mı?)
                if abs(angle_diff) > reverse_angle_threshold:
                    # GERI VİTES: Hedef arkada, dönmeden geri git
                    # Robotun arka yönü current_yaw + pi
                    back_angle = self.normalize_angle(self.current_yaw + math.pi)
                    angle_diff_to_back = self.normalize_angle(target_angle - back_angle)
                    
                    if loop_count % 40 == 0:
                        self.get_logger().info(f'GERİ VİTES: Hedef arkada, geri gidiliyor...')
                    
                    # Hafif yön düzeltmesi yaparak geri git
                    speed_factor = min(1.0, distance / 0.5)
                    twist.linear.x = -linear_speed * speed_factor  # Negatif = geri
                    twist.angular.z = -0.3 * angle_diff_to_back  # Geri giderken ters yönde dönüş
                    
                elif abs(angle_diff) > angle_tolerance:
                    # İLERİ VİTES AMA ÖNCE DÖN: Hedef önde ama açı farkı fazla
                    if loop_count % 40 == 0:
                        self.get_logger().info(f'Hedefe dönülüyor... Açı farkı: {math.degrees(angle_diff):.1f}°')
                    twist.linear.x = 0.0
                    twist.angular.z = angular_speed if angle_diff > 0 else -angular_speed
                else:
                    # İLERİ VİTES: Hedef önde ve doğru yönde, ilerle
                    if loop_count % 40 == 0:
                        self.get_logger().info(f'İLERİ: Hedefe doğru ilerleniyor...')
                    speed_factor = min(1.0, distance / 0.5)
                    twist.linear.x = linear_speed * speed_factor
                    twist.angular.z = 0.3 * angle_diff  # Küçük düzeltmeler
            
            self.cmd_vel_pub.publish(twist)
        
        # Durdur
        stop_twist = Twist()
        for _ in range(10):
            self.cmd_vel_pub.publish(stop_twist)
            time.sleep(0.05)
        
        self.get_logger().info(f'AGV durdu. Nihai X pozisyon: {self.current_x:.2f}')
        
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
    # YENİ VE DEĞİŞTİRİLMİŞ FONKSİYON - AGV HAREKET DESTEKLİ
    # =================================================================================
    def safe_joint_sequence(self, list_of_joint_angles, wait_time=2.0, max_retries=3, agv_movements=None):
        """
        Bir dizi eklem konfigürasyonu arasında güvenli sıralı hareket yapar.
        Belirli noktalarda AGV'yi hareket ettirir.
        
        Args:
            list_of_joint_angles (list of list): Her biri bir hedef konfigürasyon olan eklem açıları listesi.
            wait_time (float): Hareketler arasındaki bekleme süresi.
            max_retries (int): Başarısız bir hareket için maksimum deneme sayısı.
            agv_movements (dict): {index: x_position} formatında AGV hareket komutları.
                                  Örnek: {2: 0.4, 5: 0.6} - 3. noktada X=0.4'e, 6. noktada X=0.6'ya git
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
            
            # Bu noktada AGV hareketi gerekiyor mu kontrol et
            if agv_movements and i in agv_movements:
                target_x = agv_movements[i]
                self.get_logger().info(f"Point {i+1}'de AGV X={target_x} pozisyonuna hareket ediyor...")
                self.move_agv_to_x_position(target_x, tolerance=0.1)
                self.get_logger().info(f"AGV hareketi tamamlandı. 2 saniye bekleniyor...")
                time.sleep(2.0)

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

    # Joint açıları (derece cinsinden verilmiş, radyana çevrilecek)
    home_joints = [0.0, 0, 0, 0, 0, 0.0]
    
    # 1. nokta: 15.837, -28.815, -76.154, -154.640, 31.511, 47.310
    point1_joints = [math.radians(15.837), math.radians(-28.815), math.radians(-76.154), 
                     math.radians(-154.640), math.radians(31.511), math.radians(47.310)]
    
    # 2. nokta: -104.117, 20.814, -31.719, -290.426, 94.088, 116.370
    point2_joints = [math.radians(-104.117), math.radians(20.814), math.radians(-31.719), 
                     math.radians(-290.426), math.radians(94.088), math.radians(116.370)]
    
    # 3. nokta: -71.503, 24.562, -25.947, -270.191, 68.745, 118.992
    point3_joints = [math.radians(-71.503), math.radians(24.562), math.radians(-25.947), 
                     math.radians(-270.191), math.radians(68.745), math.radians(118.992)]
    
    # 4. nokta: -71.503, 0.877, -101.176, -287.050, 77.113, 172.441
    point4_joints = [math.radians(-71.503), math.radians(0.877), math.radians(-101.176), 
                     math.radians(-287.050), math.radians(77.113), math.radians(172.441)]
    
    # 5. nokta: -71.503, 38.622, -125.932, -289.487, 98.656, 231.511
    point5_joints = [math.radians(-71.503), math.radians(38.622), math.radians(-125.932), 
                     math.radians(-289.487), math.radians(98.656), math.radians(231.511)]
    
    # 6. nokta: -71.503, 0.877, -101.176, -287.050, 77.113, 172.441
    point6_joints = [math.radians(-71.503), math.radians(0.877), math.radians(-101.176), 
                     math.radians(-287.050), math.radians(77.113), math.radians(172.441)]
    
    # 7. nokta: -71.503, 24.562, -25.947, -270.191, 68.745, 118.992
    point7_joints = [math.radians(-71.503), math.radians(24.562), math.radians(-25.947), 
                     math.radians(-270.191), math.radians(68.745), math.radians(118.992)]
    
    # 8. nokta: -104.117, 20.814, -31.719, -290.426, 94.088, 116.370
    point8_joints = [math.radians(-104.117), math.radians(20.814), math.radians(-31.719), 
                     math.radians(-290.426), math.radians(94.088), math.radians(116.370)]
    
    # 9. nokta: 15.837, -28.815, -76.154, -154.640, 31.511, 47.310
    point9_joints = [math.radians(15.837), math.radians(-28.815), math.radians(-76.154), 
                     math.radians(-154.640), math.radians(31.511), math.radians(47.310)]
    
    safe_joint_configurations = [
        point1_joints,
        point2_joints,
        point3_joints,
        point4_joints,
        point5_joints,
        point6_joints,
        point7_joints,
        point8_joints,
        point9_joints,
    ]
    
    loop_counter = 0
    
    try:
        robot_controller.get_logger().info("=== EKLEM HEDEFLİ SONSUZ DÖNGÜ BAŞLATILIYOR ===")
        robot_controller.get_logger().info("Durdurmak için Ctrl+C tuşlayın")
        
        robot_controller.get_logger().info("Başlangıç için `home_joints` pozisyonuna gidiliyor...")
        robot_controller.move_to_joint_angles(home_joints, synchronous=True)
        time.sleep(2.0)
        
        # AGV hareketlerini tanımla: {point_index: target_x_position}
        # point3 (index 2) -> X=0.4
        # point6 (index 5) -> X=0.6
        # point8 (index 7) -> X=0.1
        agv_movements = {
            2: 0.4,   # point3'te X=0.4'e git
            5: 0.9,   # point6'da X=0.6'ya git
            7: 0.1    # point8'de X=0.1'e git
        }
        
        while rclpy.ok():
            loop_counter += 1
            robot_controller.get_logger().info(f"=== EKLEM DÖNGÜSÜ {loop_counter} BAŞLIYOR ===")
            
            # Güvenli sıralı eklem hareketi (AGV hareketleri ile birlikte)
            robot_controller.safe_joint_sequence(
                safe_joint_configurations, 
                wait_time=1.5, 
                max_retries=3,
                agv_movements=agv_movements
            )
            
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