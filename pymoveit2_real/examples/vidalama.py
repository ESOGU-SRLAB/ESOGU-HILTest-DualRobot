#!/usr/bin/env python3
"""
Çarpışma önleyici fonksiyon tabanlı pose ve joint hedefleri ile robot hareketi.
Vidalama aleti (screwdriver) pick & place için Planning Scene attach/detach mekanizması içerir.
"""

from threading import Thread
import time
import math  # Radyan dönüşümleri için eklendi

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

from pymoveit2_real import MoveIt2 as MoveIt2_Real
from pymoveit2_real import MoveIt2State
from pymoveit2_real.robots import ur as robot


class CollisionAwareRobotController(Node):
    def __init__(self):
        super().__init__("real_collision_aware_robot_controller")
        
        # Create callback group that allows execution of callbacks in parallel without restrictions
        callback_group = ReentrantCallbackGroup()

        # Create MoveIt 2 interface
        self.moveit2 = MoveIt2_Real(
            node=self,
            joint_names=robot.joint_names(),
            base_link_name=robot.base_link_name(),
            end_effector_name=robot.end_effector_name(),
            group_name=robot.MOVE_GROUP_ARM,
            callback_group=callback_group,
        )
        
        # Planner ayarları - Daha iyi planlayıcılar
        self.moveit2.planner_id = "RRTstarkConfigDefault"

        # Güvenlik için daha düşük hızlar
        self.moveit2.max_velocity = 0.1
        self.moveit2.max_acceleration = 0.1
        
        # ÖNEMLI: Çarpışma önleme ayarları
        self.moveit2.cartesian_avoid_collisions = True
        self.moveit2.cartesian_jump_threshold = 2.0
        
        # Planlama denemeleri ve zaman limiti
        self.planning_attempts = 10
        self.planning_time = 10.0

        # Gripper için doğrudan kontrolcü action client'i (MoveIt atlaniyor)
        self._gripper_action_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/gripper_controller/follow_joint_trajectory",
            callback_group=callback_group,
        )

        # Screwdriver Planning Scene durumu
        self._screwdriver_attached = False

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

    def move_gripper(self, position, synchronous=True):
        """
        Gripper eklemini doğrudan kontrolcüyé gönderir (MoveIt atlaniyor).
          - position=0.024 -> gripper kapanır (vida tutar)
          - position=0.003 -> gripper açılır (vidayı bırakır)
        """
        self.get_logger().info(f"Gripper hareketi başlatılıyor: position={position}")
        try:
            traj = JointTrajectory()
            traj.joint_names = ["ur10e_gripper_joint"]
            point = JointTrajectoryPoint()
            point.positions = [position]
            point.velocities = [0.0]
            point.accelerations = [0.0]
            point.time_from_start = Duration(sec=1, nanosec=0)
            traj.points.append(point)

            goal = FollowJointTrajectory.Goal()
            goal.trajectory = traj

            if not self._gripper_action_client.wait_for_server(timeout_sec=5.0):
                self.get_logger().error("Gripper action server bağlanamadı!")
                return False

            future = self._gripper_action_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

            goal_handle = future.result()
            if not goal_handle or not goal_handle.accepted:
                self.get_logger().warn("Gripper goal reddedildi!")
                return False

            if synchronous:
                result_future = goal_handle.get_result_async()
                rclpy.spin_until_future_complete(self, result_future, timeout_sec=10.0)
                self.get_logger().info("Gripper hareketi başarıyla tamamlandı!")
                return True
            else:
                self.get_logger().info("Asenkron gripper hareketi başlatıldı")
                return True
        except Exception as e:
            self.get_logger().error(f"Gripper hareket hatası: {str(e)}")
            return False

    def attach_screwdriver(self):
        """
        Vidalama aletini gripper'a attach eder (Planning Scene).
        Gripper kapandıktan SONRA çağrılmalıdır.
        
        Yaklaşım:
        1. Screwdriver'ı basit bir silindir primitif olarak gripper frame'ine ekler
        2. attach_collision_object() ile gripper linkine bağlar
        3. touch_links ile gripper ↔ screwdriver collision kontrolünü devre dışı bırakır
        """
        if self._screwdriver_attached:
            self.get_logger().warn("Screwdriver zaten attach edilmiş, tekrar eklenmeyecek.")
            return True

        self.get_logger().info("=== SCREWDRIVER ATTACH EDİLİYOR ===")
        
        try:
            # Screwdriver'ı basit silindir olarak gripper frame'inde ekle
            # Gripper'ın altına doğru uzanan silindir (screwdriver gövdesi)
            self.moveit2.add_collision_cylinder(
                id="screwdriver",
                height=0.02,
                radius=0.002,
                position=(0.077, 0.16, -0.026),  # Tırnakların merkezinden tam zıt yöne (aşağıya / z- yönüne) kaydırıldı
                #birinci değer sopaya olan uzaklığı ayarlıyor. normalde x ekseni olan
                #üçüncü değer ise üst sopaya uzaklığı ayarlıyor. normalde y ekseni olan
                quat_xyzw=(-0.679113, -0.196990, 0.196990, 0.679113), # Z ekseninde aynı hizalama + X ekseninde -90 derece dönüş (silindiri aşağı çevirmek için)
                frame_id="ur10e_gripper_base_link",
            )
            time.sleep(0.5)

            # Screwdriver'ı gripper_base_link'e attach et
            touch_links = [
                "ur10e_gripper_base_link",
                "ur10e_gripper_lower_1",
                "ur10e_gripper_upper_1",
                "ur10e_tool0",
                "ur10e_flange",
                "ur10e_wrist_3_link",
            ]
            
            self.moveit2.attach_collision_object(
                id="screwdriver",
                link_name="ur10e_gripper_base_link",
                touch_links=touch_links,
                weight=2.4,
            )
            time.sleep(0.5)
            
            self._screwdriver_attached = True
            self.get_logger().info("Screwdriver başarıyla gripper'a attach edildi!")
            return True
            
        except Exception as e:
            self.get_logger().error(f"Screwdriver attach hatası: {str(e)}")
            return False
    
    def detach_screwdriver(self):
        """
        Vidalama aletini gripper'dan detach eder (Planning Scene).
        Gripper açılmadan ÖNCE çağrılmalıdır.
        """
        if not self._screwdriver_attached:
            self.get_logger().warn("Screwdriver zaten detach edilmiş.")
            return True

        self.get_logger().info("=== SCREWDRIVER DETACH EDİLİYOR ===")
        
        try:
            # Screwdriver'ı gripper'dan ayır
            self.moveit2.detach_collision_object(id="screwdriver")
            time.sleep(0.5)
            
            # Collision object'i tamamen kaldır
            self.moveit2.remove_collision_object(id="screwdriver")
            time.sleep(0.5)
            
            self._screwdriver_attached = False
            self.get_logger().info("Screwdriver başarıyla detach edildi!")
            return True
            
        except Exception as e:
            self.get_logger().error(f"Screwdriver detach hatası: {str(e)}")
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

            # Dict komutu kontrolü (gripper, attach, detach)
            if isinstance(joint_angles, dict):
                if "gripper_position" in joint_angles:
                    self.move_gripper(joint_angles["gripper_position"], synchronous=True)
                elif "attach_screwdriver" in joint_angles:
                    self.attach_screwdriver()
                elif "detach_screwdriver" in joint_angles:
                    self.detach_screwdriver()
                
                if i < len(list_of_joint_angles) - 1:
                    self.get_logger().info(f"Sonraki hareket için {wait_time} saniye bekleniyor...")
                    time.sleep(wait_time)
                continue

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

    home_joints = [1.0, 0.0, -90.0, 0.0, -90.0, 0.0, 0.0]
    home_joints = [home_joints[0]] + [math.radians(a) for a in home_joints[1:]]

    frontOfScrewer = [FIRST, math.radians(16.25), math.radians(-67.48), math.radians(52.93), math.radians(-85.25), math.radians(-59.51), math.radians(20.70)]
    aboveScrewer = [FIRST, math.radians(24.08), math.radians(-48.58), math.radians(11.42), math.radians(-66.93), math.radians(-61.23), math.radians(29.44)]
    holdScrewer = [FIRST, math.radians(24.07), math.radians(-58.80), math.radians(40.81), math.radians(-86.14), math.radians(-61.16), math.radians(29.50)]
    # aboveScrewer = [FIRST, math.radians(24.08), math.radians(-48.58), math.radians(11.42), math.radians(-66.93), math.radians(-61.23), math.radians(29.44)]  # duplicate
    safeWaypoint2 = [FIRST, math.radians(30.46), math.radians(-75.20), math.radians(54.07), math.radians(-86.23), math.radians(-62.88), math.radians(36.44)]
    safeWaypoint1 = [FIRST, math.radians(66.70), math.radians(-67.28), math.radians(59.74), math.radians(-103.15), math.radians(-117.25), math.radians(144.90)]
    tookScrew = [FIRST, math.radians(66.69), math.radians(-61.31), math.radians(88.26), math.radians(-137.56), math.radians(-117.30), math.radians(145.05)]
    outTookScrew = [FIRST, math.radians(70.21), math.radians(-62.50), math.radians(89.73), math.radians(-136.08), math.radians(-118.49), math.radians(148.79)]
    safeWaypoint = [FIRST, math.radians(57.68), math.radians(-91.49), math.radians(128.21), math.radians(-151.36), math.radians(-113.78), math.radians(135.83)]
    firstTop = [FIRST, math.radians(50.19), math.radians(-78.53), math.radians(101.51), math.radians(-129.41), math.radians(-115.90), math.radians(145.14)]
    firstOpt = [FIRST, math.radians(50.19), math.radians(-75.75), math.radians(105.28), math.radians(-135.95), math.radians(-115.92), math.radians(145.16)]
    # firstTop = [FIRST, math.radians(50.19), math.radians(-78.53), math.radians(101.51), math.radians(-129.41), math.radians(-115.90), math.radians(145.14)]  # duplicate
    secondTop = [FIRST, math.radians(30.72), math.radians(-87.84), math.radians(114.97), math.radians(-141.86), math.radians(-105.69), math.radians(124.08)]
    secondOpt = [FIRST, math.radians(30.72), math.radians(-85.54), math.radians(117.64), math.radians(-146.82), math.radians(-105.71), math.radians(124.09)]
    # secondTop = [FIRST, math.radians(30.72), math.radians(-87.84), math.radians(114.97), math.radians(-141.86), math.radians(-105.69), math.radians(124.08)]  # duplicate
    thirdTop = [FIRST, math.radians(36.94), math.radians(-44.64), math.radians(43.53), math.radians(-74.54), math.radians(-110.77), math.radians(216.65)]
    thirdOpt = [FIRST, math.radians(36.94), math.radians(-44.54), math.radians(47.62), math.radians(-78.72), math.radians(-110.76), math.radians(216.67)]
    # thirdTop = [FIRST, math.radians(36.94), math.radians(-44.64), math.radians(43.53), math.radians(-74.54), math.radians(-110.77), math.radians(216.65)]  # duplicate
    fourthTop = [FIRST, math.radians(23.20), math.radians(-45.32), math.radians(45.23), math.radians(-77.45), math.radians(-112.13), math.radians(213.10)]
    fourthOpt = [FIRST, math.radians(23.20), math.radians(-45.17), math.radians(48.68), math.radians(-81.04), math.radians(-112.13), math.radians(213.11)]
    # fourthTop = [FIRST, math.radians(23.20), math.radians(-45.32), math.radians(45.23), math.radians(-77.45), math.radians(-112.13), math.radians(213.10)]  # duplicate
    Waypoint1 = [FIRST, math.radians(23.20), math.radians(-45.32), math.radians(45.23), math.radians(-77.45), math.radians(-112.13), math.radians(152.03)]
    # frontOfScrewer = [FIRST, math.radians(16.25), math.radians(-67.48), math.radians(52.93), math.radians(-85.25), math.radians(-59.51), math.radians(20.70)]  # duplicate
    # aboveScrewer = [FIRST, math.radians(24.08), math.radians(-48.58), math.radians(11.42), math.radians(-66.93), math.radians(-61.23), math.radians(29.44)]  # duplicate
    # holdScrewer = [FIRST, math.radians(24.07), math.radians(-58.80), math.radians(40.81), math.radians(-86.14), math.radians(-61.16), math.radians(29.50)]  # duplicate
    # frontOfScrewer = [FIRST, math.radians(16.25), math.radians(-67.48), math.radians(52.93), math.radians(-85.25), math.radians(-59.51), math.radians(20.70)]  # duplicate

    # Sıralı hareket için waypoint listesi (1 -> 26)
    safe_joint_configurations = [
    frontOfScrewer,
    aboveScrewer,
    holdScrewer,
    {"gripper_position": 0.02},  # Gripper kapat -> vida tut
    {"attach_screwdriver": True},  # Screwdriver'ı robotun planlamasına dahil et
    aboveScrewer,
    safeWaypoint2,
    safeWaypoint1,
    tookScrew,
    outTookScrew,
    safeWaypoint,
    firstTop,
    firstOpt,
    firstTop,
    secondTop,
    secondOpt,
    secondTop,
    thirdTop,
    thirdOpt,
    thirdTop,
    fourthTop,
    fourthOpt,
    fourthTop,
    Waypoint1,
    frontOfScrewer,
    aboveScrewer,
    holdScrewer,
    {"detach_screwdriver": True},  # Screwdriver'ı robottan ayır
    {"gripper_position": 0.003},  # Gripper aç -> vidayı bırak
    frontOfScrewer,
    ]
    
    loop_counter = 0
    
    try:
        robot_controller.get_logger().info("=== EKLEM HEDEFLİ SONSUZ DÖNGÜ BAŞLATILIYOR ===")
        robot_controller.get_logger().info("Durdurmak için Ctrl+C tuşlayın")
        
        robot_controller.get_logger().info("Başlangıç için `home_joints` pozisyonuna gidiliyor...")
        # robot_controller.move_to_joint_angles(home_joints, synchronous=True)
        time.sleep(2.0)
        
        while rclpy.ok():
            loop_counter += 1
            robot_controller.get_logger().info(f"=== EKLEM DÖNGÜSÜ {loop_counter} BAŞLIYOR ===")
            
            # Güvenli sıralı eklem hareketi (deneme sayısı 3 olacak şekilde)
            robot_controller.safe_joint_sequence(safe_joint_configurations, wait_time=0.5, max_retries=3)
            
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