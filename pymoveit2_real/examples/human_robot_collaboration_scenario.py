#!/usr/bin/env python3
"""
Çarpışma önleyici fonksiyon tabanlı pose ve joint hedefleri ile robot hareketi.
Vidalama aleti (screwdriver) pick & place için Planning Scene attach/detach mekanizması içerir.
"""

from threading import Thread
import os
import re
import time
import math  # Radyan dönüşümleri için eklendi

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from ur_msgs.srv import SetIO
from ur_msgs.msg import IOStates
from rcl_interfaces.srv import GetParameters

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
        
        # --- Planlayıcı ayarları ---------------------------------------
        # DİKKAT: MoveIt2 nesnesinin özellik adları 'allowed_planning_time' ve
        # 'num_planning_attempts'. Eskiden burada 'planning_time' /
        # 'planning_attempts' yazılıyordu; böyle bir özellik olmadığı için
        # ayarlar MoveIt'e HİÇ ulaşmıyor, her plan pymoveit2'nin 0.5 sn
        # varsayılanıyla koşuyordu. RRTstar bir OPTİMİZE EDİCİ planlayıcıdır ve
        # verilen sürenin tamamını kullanır; 0.5 sn ile 7 eklemli bu grupta
        # sürekli "Planning failed! Error code: FAILURE" veriyordu.
        self.declare_parameter("planner_id", "RRTConnectkConfigDefault")
        self.declare_parameter("planning_time", 5.0)
        self.declare_parameter("planning_attempts", 10)

        # --- Hızlar (MoveIt ölçekleme çarpanları, 0.0-1.0) ------------
        # travel_speed : noktalar arası normal seyir
        # screw_speed  : xTop <-> xOpt geçişleri; vidalama sırasında robot
        #                belirgin biçimde daha yavaş hareket etmeli
        self.declare_parameter("travel_speed", 0.1)
        self.declare_parameter("screw_speed", 0.01)

        self.travel_speed = float(self.get_parameter("travel_speed").value)
        self.screw_speed = float(self.get_parameter("screw_speed").value)

        # --- Gripper konumlari (ur10e_gripper_joint, prismatic) -------
        # URDF konvansiyonu: joint = 0.0  -> TAM ACIK
        #                    joint buyudukce tirnaklar kapanir (nominal ust sinir 0.025)
        # gripper_open_position   : her turun basinda gidilecek "tam acik" konum
        # gripper_release_position: vidalama aletini birakirken kullanilan konum
        # gripper_closed_position : vidalama aletini tutarken kullanilan konum
        #   0.026, URDF'teki ust limit (nominal 0.025 strok + 0.001 pay), yani
        #   "donanimin izin verdigi kadar kapali". Surucu bu degeri gercek
        #   mekanik sinira (position_max_ - position_min_) kirpiyor.
        self.declare_parameter("gripper_open_position", 0.0)
        self.declare_parameter("gripper_release_position", 0.003)
        self.declare_parameter("gripper_closed_position", 0.026)
        # Gripper kendi ic kontrolcusuyle hareket ediyor; trajectory action'i
        # bitse bile tirnaklar birkac yuz ms daha yol aliyor. Bir sonraki kol
        # hareketine gecmeden once bu kadar bekle.
        self.declare_parameter("gripper_settle_time", 0.7)

        self.gripper_open_position = float(self.get_parameter("gripper_open_position").value)
        self.gripper_release_position = float(self.get_parameter("gripper_release_position").value)
        self.gripper_closed_position = float(self.get_parameter("gripper_closed_position").value)
        self.gripper_settle_time = float(self.get_parameter("gripper_settle_time").value)

        self.moveit2.max_velocity = self.travel_speed
        self.moveit2.max_acceleration = self.travel_speed
        
        # ÖNEMLI: Çarpışma önleme ayarları
        self.moveit2.cartesian_avoid_collisions = True
        self.moveit2.cartesian_jump_threshold = 2.0
        
        self.planning_time = float(self.get_parameter("planning_time").value)
        self.planning_attempts = int(self.get_parameter("planning_attempts").value)

        self.moveit2.planner_id = str(self.get_parameter("planner_id").value)
        self.moveit2.allowed_planning_time = self.planning_time
        self.moveit2.num_planning_attempts = self.planning_attempts
        self.get_logger().info(
            f"Planlayıcı: {self.moveit2.planner_id}, "
            f"süre: {self.moveit2.allowed_planning_time}s, "
            f"deneme: {self.moveit2.num_planning_attempts}"
        )

        # Gripper için doğrudan kontrolcü action client'i (MoveIt atlaniyor)
        self._gripper_action_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/gripper_controller/follow_joint_trajectory",
            callback_group=callback_group,
        )

        # Screwdriver Planning Scene durumu
        self._screwdriver_attached = False

        # =================================================================
        # UR GPIO ARAYÜZÜ  (pendant'taki "GPIO" sekmesinin ROS 2 karşılığı)
        #   Çıkış yazma : /io_and_status_controller/set_io    (ur_msgs/srv/SetIO)
        #   Giriş okuma : /io_and_status_controller/io_states (ur_msgs/msg/IOStates)
        #
        # Pin numaralandırması (SetIO ve IOStates aynı indeksi kullanır):
        #    0 -  7  -> standard digital output / input
        #    8 - 15  -> configurable output / input
        #   16 - 17  -> tool digital output / input
        #
        # NOT: Bu komutlar RTDE üzerinden gider; External Control programı
        #      koşmasa bile IO yazılır/okunur (hareket için ise program şart).
        # =================================================================
        # --- IFARLAB hücresinde ölçülen pin haritası (find_io_pins.py ile tespit edildi) ---
        #   ÇIKIŞ  standard_digital_out[0] -> vidalama SIKMA      <-- burada kullanılıyor
        #   ÇIKIŞ  standard_digital_out[1] -> vidalama SÖKME      (şimdilik kullanılmıyor)
        #   GİRİŞ  standard_digital_in[7]  -> YEŞİL buton         <-- burada kullanılıyor
        #   GİRİŞ  standard_digital_in[6]  -> KIRMIZI buton       (şimdilik kullanılmıyor)
        #   GİRİŞ  standard_digital_in[5]  -> BEYAZ buton         (şimdilik kullanılmıyor)
        self.declare_parameter("screwdriver_pin", 0)          # vidalama SIKMA çıkışı (DOUT0)
        self.declare_parameter("screwdriver_reverse_pin", 1)  # vidalama SÖKME çıkışı (DOUT1) - rezerve
        self.declare_parameter("green_button_pin", 7)         # operatörün yeşil butonu (DIN7)
        self.declare_parameter("red_button_pin", 6)           # kırmızı buton (DIN6) - rezerve
        self.declare_parameter("white_button_pin", 5)         # beyaz buton (DIN5) - rezerve
        self.declare_parameter("green_button_active_high", True)  # buton NC ise False yapın
        self.declare_parameter("green_button_timeout", 0.0)   # 0.0 = sonsuza kadar bekle

        # GPIO yalnızca GERÇEK robotta anlamlı: use_fake_hardware:=true iken
        # mock_components dijital girişleri hep 0 döndürür, yani yeşil buton
        # beklemesi sonsuza kadar takılır. Bu yüzden sim/fake modda GPIO
        # tamamen devre dışı bırakılır.
        #   auto      -> use_fake_hardware / ENV_USE_FAKE_HARDWARE'e ve io_states
        #                yayınının varlığına bakarak kendisi karar verir
        #   force_on  -> zorla açık,  force_off -> zorla kapalı
        #
        # DİKKAT: değerler bilerek "force_on/force_off"; çıplak "on"/"off"
        # komut satırında YAML kuralı gereği BOOLEAN olarak ayrıştırılır ve
        # string parametreye atanamayıp node'u düşürür. Yine de launch'tan
        # string olarak gelirlerse aşağıda normalize ediliyorlar.
        self.declare_parameter("gpio_mode", "auto")
        # use_fake_hardware'ı elle vermek zorunda değilsiniz: auto modda
        # robot_description'daki donanım eklentisine bakarak kendisi anlar.
        # Bu parametre sadece o tespiti geçersiz kılmak isterseniz var.
        self.declare_parameter("use_fake_hardware", False)
        self.declare_parameter("description_node", "/robot_state_publisher")
        self.declare_parameter("fake_button_delay", 0.0)  # GPIO kapalıyken buton yerine beklenecek süre

        self.screwdriver_pin = int(self.get_parameter("screwdriver_pin").value)
        self.screwdriver_reverse_pin = int(self.get_parameter("screwdriver_reverse_pin").value)
        self.green_button_pin = int(self.get_parameter("green_button_pin").value)
        self.red_button_pin = int(self.get_parameter("red_button_pin").value)
        self.white_button_pin = int(self.get_parameter("white_button_pin").value)
        self.green_button_active_high = bool(self.get_parameter("green_button_active_high").value)
        self.green_button_timeout = float(self.get_parameter("green_button_timeout").value)
        self.gpio_mode = self._normalize_gpio_mode(self.get_parameter("gpio_mode").value)
        self.use_fake_hardware = bool(self.get_parameter("use_fake_hardware").value)
        self.description_node = str(self.get_parameter("description_node").value)
        self.fake_button_delay = float(self.get_parameter("fake_button_delay").value)
        # resolve_gpio_availability() çağrılana kadar GPIO kullanılmaz
        self.io_enabled = False
        self.io_states_topic = "/io_and_status_controller/io_states"

        self._set_io_client = self.create_client(
            SetIO, "/io_and_status_controller/set_io", callback_group=callback_group
        )
        self._digital_inputs = {}
        self._io_states_seen = False
        # io_states'in EN SON ne zaman guncellendigi. Callback'ler durursa
        # _digital_inputs sessizce donar; bu damga onu gorunur kilar.
        self._io_states_stamp = 0.0
        # main()'deki arka plan executor'u. pymoveit2 node'u kopardiginda
        # (asagidaki _ensure_executor aciklamasi) geri baglamak icin lazim.
        self._background_executor = None
        self._screwdriver_running = False
        self.create_subscription(
            IOStates,
            self.io_states_topic,
            self._io_states_callback,
            10,
            callback_group=callback_group,
        )

        self.get_logger().info(
            f"GPIO ayarları -> vidalama SIKMA: DOUT{self.screwdriver_pin}, "
            f"SÖKME: DOUT{self.screwdriver_reverse_pin}, "
            f"yeşil buton: DIN{self.green_button_pin} "
            f"(active_high={self.green_button_active_high}, gpio_mode={self.gpio_mode})"
        )

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
        
        # Doğru özellik adları: allowed_planning_time / num_planning_attempts
        original_attempts = self.moveit2.num_planning_attempts
        original_time = self.moveit2.allowed_planning_time

        try:
            self.moveit2.num_planning_attempts = planning_attempts
            self.moveit2.allowed_planning_time = planning_time


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
            self.moveit2.num_planning_attempts = original_attempts
            self.moveit2.allowed_planning_time = original_time
    
    def move_to_joint_angles(self, joint_positions, synchronous=True, speed=None):
        """
        Robotu belirli eklem açılarına (radyan cinsinden) güvenli bir şekilde hareket ettirir.

        Args:
            joint_positions (list): Hedef eklem açıları (radyan cinsinden).
                                     Listenin uzunluğu robotun eklem sayısına eşit olmalıdır.
            synchronous (bool): Hareketin bitmesini bekle (senkron)
            speed (float|None): Bu harekete özel MoveIt hız/ivme ölçekleme çarpanı
                                (0.0-1.0). None ise travel_speed kullanılır.
                                Hareket bitince önceki değer geri yüklenir.
        """
        original_velocity = self.moveit2.max_velocity
        original_acceleration = self.moveit2.max_acceleration

        if speed is not None:
            speed = max(0.001, min(1.0, float(speed)))
            self.moveit2.max_velocity = speed
            self.moveit2.max_acceleration = speed
            self.get_logger().info(
                f"Güvenli JOINT hareketi başlatılıyor (hız ölçeği {speed}): {joint_positions}"
            )
        else:
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
        finally:
            # Hıza özel ayarı her durumda geri al ki sonraki hareketler
            # yanlışlıkla yavaş kalmasın.
            self.moveit2.max_velocity = original_velocity
            self.moveit2.max_acceleration = original_acceleration

    def _ensure_executor(self):
        """
        Node'u arka plandaki executor'e geri baglar.

        NEDEN GEREKLI: pymoveit2_real/moveit2.py icinde `rclpy.spin_once(self._node)`
        cagrilari var (ornegin wait_until_executed(), satir 793). rclpy'nin
        spin_once'i once `executor.add_node(node)` yapiyor; Node.executor setter'i
        de bunun icin `current_executor.remove_node(self)` cagirip node'u bizim
        MultiThreadedExecutor'umuzden KOPARIYOR. Ardindan spin_once'in
        `finally: executor.remove_node(node)` satiri global executor'den de
        cikariyor. Net sonuc: node HICBIR executor'e bagli degil.

        O andan itibaren arka plan thread'i bos bir executor dondurur ve
        node'un abonelik callback'leri (io_states dahil) hic calismaz:
        _digital_inputs donar, yesil buton sonsuza kadar goremeyiz; ayni sekilde
        servis future'lari da (set_io) tamamlanmaz.

        DIKKAT - `self.executor is None` kontrolu ISE YARAMAZ: rclpy'nin
        Executor.remove_node()'u node'u yalnizca kendi _nodes kumesinden cikarir,
        Node.__executor_weakref'i TEMIZLEMEZ. Yani spin_once'tan sonra
        node.executor hala global executor'u dondurur (None degildir), oysa node
        hicbir executor'un _nodes kumesinde degildir ve kimse onu spin etmez.
        Dogru olcut: arka plan executor'unun node listesinde miyiz?
        """
        ex = self._background_executor
        if ex is None:
            return
        try:
            if self not in ex.get_nodes():
                ex.add_node(self)
                self.get_logger().warn(
                    "Node executor'den kopmustu (pymoveit2 spin_once), geri baglandi."
                )
        except Exception as e:
            self.get_logger().warn(f"Executor'e yeniden baglanilamadi: {e}")

    def _wait_for_future(self, future, timeout):
        """
        Bir future'ın tamamlanmasını bekler.

        rclpy.spin_until_future_complete(self, ...) BİLEREK kullanılmıyor: node
        zaten main()'deki MultiThreadedExecutor tarafından spin ediliyor ve o
        çağrı geçici bir SingleThreadedExecutor açıp node.executor'ü ona
        devrediyor, geri vermiyor. Sonuç: node arka plan executor'ünde sessizce
        sağır kalıyor (ve wait-set çakışmasından RCLError ile düşebiliyor).
        """
        self._ensure_executor()
        deadline = time.time() + timeout
        while not future.done() and time.time() < deadline and rclpy.ok():
            time.sleep(0.01)
        return future.done()

    def open_gripper(self, synchronous=True):
        """
        Gripper'ı TAM AÇIK konuma getirir (gripper_open_position, varsayılan 0.0).

        Senaryonun her turunun başında çağrılır: bir önceki tur yarıda kesilmiş
        veya gripper elle kapalı bırakılmış olabilir; tırnaklar kapalıyken
        `holdScrewer` noktasına gitmek vidalama aletine çarpar.
        """
        self.get_logger().info("Gripper başlangıç konumuna (TAM AÇIK) getiriliyor...")
        return self.move_gripper(self.gripper_open_position, synchronous=synchronous)

    def move_gripper(self, position, synchronous=True):
        """
        Gripper eklemini doğrudan kontrolcüye gönderir (MoveIt atlaniyor).

        ur10e_gripper_joint prismatic ve URDF'te 0.0 = TAM AÇIK olacak şekilde
        tanımlı; değer büyüdükçe tırnaklar kapanır (nominal üst sınır 0.025):
          - position=0.0   -> tam açık   (tur başlangıcı)
          - position=0.003 -> açık/bırak (vidalama aletini bırakır)
          - position=0.026 -> tam kapalı (vidalama aletini tutar)
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
            if not self._wait_for_future(future, 10.0):
                self.get_logger().error("Gripper goal isteğine yanıt gelmedi (timeout)!")
                return False

            goal_handle = future.result()
            if not goal_handle or not goal_handle.accepted:
                self.get_logger().warn("Gripper goal reddedildi!")
                return False

            if synchronous:
                result_future = goal_handle.get_result_async()
                if not self._wait_for_future(result_future, 10.0):
                    self.get_logger().error("Gripper hareketi zaman aşımına uğradı!")
                    return False
                # Trajectory action'i bitti; ancak gripper'in kendi ic
                # kontrolcusu tirnaklari hala hareket ettiriyor olabilir.
                if self.gripper_settle_time > 0.0:
                    time.sleep(self.gripper_settle_time)
                self.get_logger().info("Gripper hareketi başarıyla tamamlandı!")
                return True
            else:
                self.get_logger().info("Asenkron gripper hareketi başlatıldı")
                return True
        except Exception as e:
            self.get_logger().error(f"Gripper hareket hatası: {str(e)}")
            return False

    # =================================================================================
    # GPIO: DİJİTAL ÇIKIŞ YAZMA / DİJİTAL GİRİŞ OKUMA
    # =================================================================================
    def _detect_fake_hardware_from_description(self, timeout=5.0):
        """
        robot_description'ı okuyup UR10e'nin hangi donanım eklentisiyle
        yüklendiğine bakar. Böylece use_fake_hardware'ı elle vermeye gerek kalmaz.

        Döner: True (fake/sim), False (gerçek), None (karar verilemedi).
        """
        service = f"{self.description_node}/get_parameters"
        client = self.create_client(GetParameters, service)
        try:
            if not client.wait_for_service(timeout_sec=timeout):
                self.get_logger().warn(f"'{service}' bulunamadı; robot_description okunamadı.")
                return None

            request = GetParameters.Request()
            request.names = ["robot_description"]
            future = client.call_async(request)
            if not self._wait_for_future(future, timeout):
                self.get_logger().warn("robot_description isteği yanıtlanmadı.")
                return None

            values = future.result().values
            if not values or not values[0].string_value:
                self.get_logger().warn("robot_description boş döndü.")
                return None
            description = values[0].string_value
        finally:
            self.destroy_client(client)

        # Bu senaryo GERÇEK kolu (ur10e_ öneki, sim_ur10e_ değil) sürüyor.
        # O kolun ros2_control bloğunu bulup eklentisine bakıyoruz.
        arm_joint = f"{robot.prefix}shoulder_pan_joint"
        for block in re.findall(r"<ros2_control\b.*?</ros2_control>", description, re.S):
            if f'"{arm_joint}"' not in block:
                continue
            plugin_match = re.search(r"<plugin>\s*([^<\s]+)\s*</plugin>", block)
            plugin = plugin_match.group(1) if plugin_match else "?"
            fake_markers = ("mock_components", "fake_components",
                            "gz_ros2_control", "ign_ros2_control", "gazebo_ros2_control")
            is_fake = any(marker in plugin for marker in fake_markers)
            self.get_logger().info(
                f"robot_description: '{arm_joint}' eklentisi -> {plugin} "
                f"({'SİM/FAKE' if is_fake else 'GERÇEK donanım'})"
            )
            return is_fake

        self.get_logger().warn(
            f"robot_description içinde '{arm_joint}' taşıyan ros2_control bloğu yok."
        )
        return None

    @staticmethod
    def _normalize_gpio_mode(value):
        """'on'/'off'/'true'/'false' gibi yazımları kanonik biçime çevirir."""
        text = str(value).strip().lower()
        if text in ("force_on", "on", "true", "1", "enabled", "yes"):
            return "force_on"
        if text in ("force_off", "off", "false", "0", "disabled", "no"):
            return "force_off"
        return "auto"

    def resolve_gpio_availability(self, probe_timeout=5.0):
        """
        GPIO'nun bu çalıştırmada kullanılıp kullanılmayacağına karar verir.
        main() içinde, executor dönmeye başladıktan SONRA çağrılmalıdır
        (ROS grafiğinin keşfedilmesi gerekiyor).
        """
        if self.gpio_mode == "force_off":
            self.io_enabled = False
            self.get_logger().warn(
                "gpio_mode=force_off -> vidalama ve yeşil buton DEVRE DIŞI. "
                "Vidalama adımları atlanacak, buton beklenmeyecek."
            )
            return False

        if self.gpio_mode == "auto":
            env_fake = os.environ.get("ENV_USE_FAKE_HARDWARE", "").strip().lower()
            fake = self.use_fake_hardware or env_fake in ("true", "1", "yes")
            if not fake:
                # Asıl kaynak: robot_description'daki donanım eklentisi.
                detected = self._detect_fake_hardware_from_description()
                if detected is not None:
                    fake = detected
            if fake:
                self.io_enabled = False
                self.get_logger().warn(
                    "Sim/fake donanım tespit edildi -> GPIO DEVRE DIŞI.\n"
                    "  mock_components dijital girişleri hep 0 döndürdüğü için "
                    "yeşil buton beklemesi sonsuza kadar takılırdı.\n"
                    "  Vidalama adımları atlanacak, buton beklenmeyecek. "
                    "Gerçek robotta gpio_mode=auto kendiliğinden açılır."
                )
                return False

        # Gerçek donanım bekleniyor: io_and_status_controller gerçekten var mı?
        deadline = time.time() + probe_timeout
        while time.time() < deadline and rclpy.ok():
            if self.count_publishers(self.io_states_topic) > 0 and self._set_io_client.service_is_ready():
                self.io_enabled = True
                self.get_logger().info("GPIO AKTİF: io_and_status_controller bulundu.")
                return True
            time.sleep(0.2)

        if self.gpio_mode == "force_on":
            self.io_enabled = True
            self.get_logger().error(
                "gpio_mode=force_on ama io_and_status_controller bulunamadı! "
                "GPIO çağrıları büyük ihtimalle başarısız olacak."
            )
            return True

        self.io_enabled = False
        self.get_logger().error(
            f"'{self.io_states_topic}' yayını ve set_io servisi bulunamadı -> GPIO DEVRE DIŞI.\n"
            "  Sürücü ayakta mı? 'ros2 control list_controllers' ile "
            "io_and_status_controller'ın 'active' olduğunu doğrulayın.\n"
            "  Zorlamak için: gpio_mode:=force_on"
        )
        return False

    def _io_states_callback(self, msg: IOStates):
        """io_and_status_controller'dan gelen dijital giriş durumlarını sakla."""
        for digital in msg.digital_in_states:
            self._digital_inputs[digital.pin] = digital.state
        self._io_states_seen = True
        self._io_states_stamp = time.time()

    def set_digital_output(self, pin, state, timeout=5.0):
        """
        UR kontrolcüsündeki bir dijital çıkışı set/reset eder.
        Pendant'taki `set_digital_out(pin, True/False)` ile aynı işi yapar.
        """
        if not self.io_enabled:
            self.get_logger().warn(f"[GPIO KAPALI] DOUT{pin}={int(bool(state))} atlandı.")
            return True

        if not self._set_io_client.wait_for_service(timeout_sec=timeout):
            self.get_logger().error(
                "set_io servisi bulunamadı! io_and_status_controller aktif mi? "
                "(ros2 control list_controllers ile kontrol edin)"
            )
            return False

        request = SetIO.Request()
        request.fun = SetIO.Request.FUN_SET_DIGITAL_OUT
        request.pin = int(pin)
        request.state = float(SetIO.Request.STATE_ON if state else SetIO.Request.STATE_OFF)

        future = self._set_io_client.call_async(request)
        # Node zaten arka plandaki executor tarafından spin ediliyor; burada
        # sadece future'ın tamamlanmasını bekliyoruz (spin_until_future_complete
        # node'un sahipliğini çalacağı için kullanılmıyor).
        deadline = time.time() + timeout
        while not future.done() and time.time() < deadline and rclpy.ok():
            time.sleep(0.01)

        if not future.done():
            self.get_logger().error(f"set_io yanıt vermedi (pin={pin}, state={state})")
            return False

        success = bool(future.result().success)
        if success:
            self.get_logger().info(f"DOUT{pin} -> {'ON' if state else 'OFF'}")
        else:
            self.get_logger().warn(f"DOUT{pin} ayarlanamadı (state={state})")
        return success

    def screwdriver(self, on: bool, reverse: bool = False):
        """
        Vidalama aletini çalıştırır / durdurur (dijital çıkış üzerinden).

        reverse=False -> DOUT0 (sıkma), reverse=True -> DOUT1 (sökme).
        Kapatırken her iki çıkış da düşürülür; ikisi aynı anda HIGH kalmasın.
        """
        pin = self.screwdriver_reverse_pin if reverse else self.screwdriver_pin
        direction = "SÖKME" if reverse else "SIKMA"

        if not on:
            self.get_logger().info("=== VİDALAMA DURDURULUYOR ===")
            ok_forward = self.set_digital_output(self.screwdriver_pin, False)
            ok_reverse = self.set_digital_output(self.screwdriver_reverse_pin, False)
            success = ok_forward and ok_reverse
            if success:
                self._screwdriver_running = False
            return success

        self.get_logger().info(f"=== VİDALAMA BAŞLATILIYOR ({direction}) ===")
        success = self.set_digital_output(pin, True)
        if success:
            self._screwdriver_running = True
        return success

    def get_digital_input(self, pin):
        """Son alınan dijital giriş değerini döndürür (henüz veri yoksa None)."""
        return self._digital_inputs.get(int(pin))

    def wait_for_green_button(self, timeout=None, poll_period=0.05, require_release=True):
        """
        Operatörün yeşil butona basmasını bekler (dijital giriş üzerinde YÜKSELEN KENAR).

        Args:
            timeout: saniye; None ise parametredeki green_button_timeout kullanılır,
                     0.0 ise süresiz bekler.
            require_release: buton çağrı anında zaten basılıysa önce bırakılmasını
                     bekler; böylece tek basış iki vidalamayı tetiklemez.
        """
        pin = self.green_button_pin

        if not self.io_enabled:
            if self.fake_button_delay > 0.0:
                self.get_logger().warn(
                    f"[GPIO KAPALI] Yeşil buton yerine {self.fake_button_delay:.1f} sn bekleniyor."
                )
                time.sleep(self.fake_button_delay)
            else:
                self.get_logger().warn("[GPIO KAPALI] Yeşil buton beklenmiyor, devam ediliyor.")
            return True

        if timeout is None:
            timeout = self.green_button_timeout
        deadline = None if timeout <= 0.0 else time.time() + timeout

        # io_states akışının başlamasını bekle
        self._ensure_executor()
        wait_start = time.time()
        while not self._io_states_seen and rclpy.ok():
            if time.time() - wait_start > 5.0:
                self.get_logger().error(
                    "/io_and_status_controller/io_states yayını yok! "
                    "io_and_status_controller aktif mi?"
                )
                return False
            time.sleep(0.05)

        def pressed():
            raw = self._digital_inputs.get(pin)
            if raw is None:
                # Sessizce False donmek bu fonksiyonu sonsuz beklemeye sokar;
                # cagiran taraf bunu ayirt edebilsin diye None donuyoruz.
                return None
            return raw if self.green_button_active_high else (not raw)

        if require_release and pressed():
            self.get_logger().warn(f"DIN{pin} zaten basılı durumda, bırakılması bekleniyor...")
            while rclpy.ok() and pressed():
                if deadline is not None and time.time() > deadline:
                    self.get_logger().error("Yeşil buton bırakılmadı (timeout).")
                    return False
                time.sleep(poll_period)

        self.get_logger().info(f">>> OPERATÖR BEKLENİYOR: yeşil butona basın (DIN{pin}) <<<")
        last_log = time.time()
        while rclpy.ok():
            # pymoveit2 bir onceki hareket sirasinda node'u executor'den koparmis
            # olabilir; koparilmissa io_states callback'i hic calismaz ve
            # _digital_inputs donar. Her turda bagi kontrol et.
            self._ensure_executor()

            state = pressed()
            if state:
                self.get_logger().info("Yeşil butona basıldı, devam ediliyor.")
                return True

            now = time.time()
            if deadline is not None and now > deadline:
                self.get_logger().error("Yeşil buton beklenirken zaman aşımı!")
                return False

            if now - last_log > 10.0:
                if state is None:
                    self.get_logger().error(
                        f"DIN{pin} io_states mesajlarinda HIC yok! Buton baska bir "
                        f"pine bagli olabilir (green_button_pin parametresi) veya "
                        f"io_and_status_controller bu girisi yayinlamiyor."
                    )
                elif now - self._io_states_stamp > 2.0:
                    # Sessiz kilitlenmenin tipik sebebi: node executor'den kopmus.
                    self.get_logger().error(
                        f"io_states {now - self._io_states_stamp:.1f} sn'dir "
                        f"guncellenmiyor; dijital girisler BAYAT. Buton basilsa bile "
                        f"gorulmez."
                    )
                else:
                    self.get_logger().info(f"... hâlâ yeşil buton bekleniyor (DIN{pin})")
                last_log = now

            time.sleep(poll_period)

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

            # Hıza özel hareket: {"joints": [...], "speed": 0.01}
            move_speed = None
            if isinstance(joint_angles, dict) and "joints" in joint_angles:
                move_speed = joint_angles.get("speed")
                joint_angles = joint_angles["joints"]

            # Dict komutu kontrolü (gripper, attach, detach, gpio)
            if isinstance(joint_angles, dict):
                if "gripper_position" in joint_angles:
                    self.move_gripper(joint_angles["gripper_position"], synchronous=True)
                elif "attach_screwdriver" in joint_angles:
                    self.attach_screwdriver()
                elif "detach_screwdriver" in joint_angles:
                    self.detach_screwdriver()
                elif "screwdriver" in joint_angles:
                    self.screwdriver(
                        bool(joint_angles["screwdriver"]),
                        reverse=bool(joint_angles.get("reverse", False)),
                    )
                elif "wait_green_button" in joint_angles:
                    self.wait_for_green_button(timeout=joint_angles.get("timeout"))
                elif "wait" in joint_angles:
                    self.get_logger().info(f"{joint_angles['wait']} saniye bekleniyor...")
                    time.sleep(float(joint_angles["wait"]))
                
                if i < len(list_of_joint_angles) - 1:
                    self.get_logger().info(f"Sonraki hareket için {wait_time} saniye bekleniyor...")
                    time.sleep(wait_time)
                continue

            # Belirli bir noktaya hareketi denemek için iç döngü
            for attempt in range(max_retries):
                self.get_logger().info(f"  -> Deneme {attempt + 1}/{max_retries}...")
                
                # Hareketi dene
                success = self.move_to_joint_angles(
                    joint_angles, synchronous=True, speed=move_speed
                )
                
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
    # pymoveit2 icindeki rclpy.spin_once() cagrilari node'u bu executor'den
    # koparabiliyor; _ensure_executor() geri baglayabilsin diye referansi ver.
    robot_controller._background_executor = executor
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()
    robot_controller.create_rate(1.0).sleep()
    
    robot_controller.check_planning_scene()

    # GPIO gerçek robotta mı, sim/fake modda mı? Hareket başlamadan karar ver.
    robot_controller.resolve_gpio_availability()
    
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
    safeWaypoint2 = [FIRST, math.radians(30.46), math.radians(-75.20), math.radians(54.07), math.radians(-86.23), math.radians(-62.88), math.radians(36.44)]
    safeWaypoint1 = [FIRST, math.radians(66.70), math.radians(-67.28), math.radians(59.74), math.radians(-103.15), math.radians(-117.25), math.radians(144.90)]
    tookScrew = [FIRST, math.radians(66.69), math.radians(-61.31), math.radians(88.26), math.radians(-137.56), math.radians(-117.30), math.radians(145.05)]
    outTookScrew = [FIRST, math.radians(70.21), math.radians(-62.50), math.radians(89.73), math.radians(-136.08), math.radians(-118.49), math.radians(148.79)]
    safeWaypoint = [FIRST, math.radians(57.68), math.radians(-91.49), math.radians(128.21), math.radians(-151.36), math.radians(-113.78), math.radians(135.83)]
    firstTop = [FIRST, math.radians(50.19), math.radians(-78.53), math.radians(101.51), math.radians(-129.41), math.radians(-115.90), math.radians(145.14)]
    firstOpt = [FIRST, math.radians(50.19), math.radians(-75.75), math.radians(105.28), math.radians(-135.95), math.radians(-115.92), math.radians(145.16)]
    secondTop = [FIRST, math.radians(30.72), math.radians(-87.84), math.radians(114.97), math.radians(-141.86), math.radians(-105.69), math.radians(124.08)]
    secondOpt = [FIRST, math.radians(30.72), math.radians(-85.54), math.radians(117.64), math.radians(-146.82), math.radians(-105.71), math.radians(124.09)]
    thirdTop = [1.8420303208638846, 0.6366738326549317, -0.7821781689244712,
                0.7426429613881561, -1.29837890700152, -1.9427011974646786,
                3.766707435569096]
    thirdOpt = [1.8420157451813972, 0.6364952750453047, -0.7821448608379735,
                0.833901430926128, -1.3709123037217756, -1.9426966327044723,
                3.7666595437667088]
    fourthTop = [FIRST, math.radians(23.20), math.radians(-45.32), math.radians(45.23), math.radians(-77.45), math.radians(-112.13), math.radians(213.10)]
    fourthOpt = [FIRST, math.radians(23.20), math.radians(-45.17), math.radians(48.68), math.radians(-81.04), math.radians(-112.13), math.radians(213.11)]
    Waypoint1 = [FIRST, math.radians(23.20), math.radians(-45.32), math.radians(45.23), math.radians(-77.45), math.radians(-112.13), math.radians(152.03)]
   
    # Vidalama geçişlerinde kullanılacak hız ölçeği (screw_speed parametresi)
    SCREW_SPEED = robot_controller.screw_speed

    # Gripper konumları (parametrelerden; URDF: 0.0 = TAM AÇIK)
    GRIPPER_OPEN = robot_controller.gripper_open_position       # tur başı: tam açık
    GRIPPER_RELEASE = robot_controller.gripper_release_position  # aleti bırak
    GRIPPER_CLOSED = robot_controller.gripper_closed_position    # aleti tut

    # Sıralı hareket için waypoint listesi
    safe_joint_configurations = [
    # --- TUR BAŞLANGICI: gripper'ın hangi konumda kaldığı bilinmiyor;
    #     kola dokunmadan önce tırnakları TAM AÇIK konuma getir ---
    {"gripper_position": GRIPPER_OPEN},
    frontOfScrewer,
    aboveScrewer,
    holdScrewer,
    {"gripper_position": GRIPPER_CLOSED},  # Gripper kapat -> vida tut
    {"attach_screwdriver": True},  # Screwdriver'ı robotun planlamasına dahil et
    aboveScrewer,
    safeWaypoint2,
    safeWaypoint1,
    tookScrew,
    outTookScrew,
    safeWaypoint,
    firstTop,
    # --- 1. VİDA: operatör yeşil butona basınca vidalama başlar ---
    {"wait_green_button": True},   # pendant: yeşil buton (dijital giriş) beklenir
    {"screwdriver": True},         # pendant: set_digital_out(screwdriver_pin, True)
    # Vidalama geçişleri YAVAŞ: screw_speed (varsayılan 0.01) ölçeğiyle
    {"joints": firstOpt, "speed": SCREW_SPEED},   # vidayı sıkarken yavaşça aşağı in
    {"wait": 1.5},                 # vidanın oturması için bekle
    {"screwdriver": False},        # firstTop'a dönmeden önce vidalamayı kapat
    {"joints": firstTop, "speed": SCREW_SPEED},   # yavaşça geri çık
    secondTop,
    # --- 2. VİDA: operatör yeşil butona basınca vidalama başlar ---
    {"wait_green_button": True},   # pendant: yeşil buton (dijital giriş) beklenir
    {"screwdriver": True},         # pendant: set_digital_out(screwdriver_pin, True)
    # Vidalama geçişleri YAVAŞ: screw_speed (varsayılan 0.01) ölçeğiyle
    {"joints": secondOpt, "speed": SCREW_SPEED},   # vidayı sıkarken yavaşça aşağı in
    {"wait": 1.5},                 # vidanın oturması için bekle
    {"screwdriver": False},        # secondTop'a dönmeden önce vidalamayı kapat
    {"joints": secondTop, "speed": SCREW_SPEED},   # yavaşça geri çık
    thirdTop,
    # --- 3. VİDA: operatör yeşil butona basınca vidalama başlar ---
    {"wait_green_button": True},   # pendant: yeşil buton (dijital giriş) beklenir
    {"screwdriver": True},         # pendant: set_digital_out(screwdriver_pin, True)
    # Vidalama geçişleri YAVAŞ: screw_speed (varsayılan 0.01) ölçeğiyle
    {"joints": thirdOpt, "speed": SCREW_SPEED},   # vidayı sıkarken yavaşça aşağı in
    {"wait": 1.5},                 # vidanın oturması için bekle
    {"screwdriver": False},        # thirdTop'a dönmeden önce vidalamayı kapat
    {"joints": thirdTop, "speed": SCREW_SPEED},   # yavaşça geri çık
    fourthTop,
    # --- 4. VİDA: operatör yeşil butona basınca vidalama başlar ---
    {"wait_green_button": True},   # pendant: yeşil buton (dijital giriş) beklenir
    {"screwdriver": True},         # pendant: set_digital_out(screwdriver_pin, True)
    # Vidalama geçişleri YAVAŞ: screw_speed (varsayılan 0.01) ölçeğiyle
    {"joints": fourthOpt, "speed": SCREW_SPEED},   # vidayı sıkarken yavaşça aşağı in
    {"wait": 1.5},                 # vidanın oturması için bekle
    {"screwdriver": False},        # fourthTop'a dönmeden önce vidalamayı kapat
    {"joints": fourthTop, "speed": SCREW_SPEED},   # yavaşça geri çık
    Waypoint1,
    frontOfScrewer,
    aboveScrewer,
    holdScrewer,
    {"detach_screwdriver": True},  # Screwdriver'ı robottan ayır
    {"gripper_position": GRIPPER_RELEASE},  # Gripper aç -> vidayı bırak
    frontOfScrewer,
    ]
    
    loop_counter = 0
    
    try:
        robot_controller.get_logger().info("=== EKLEM HEDEFLİ SONSUZ DÖNGÜ BAŞLATILIYOR ===")
        robot_controller.get_logger().info("Durdurmak için Ctrl+C tuşlayın")
        
        # Program açılışında gripper'ın konumu bilinmiyor (önceki çalıştırma
        # yarıda kesilmiş, tırnaklar kapalı kalmış olabilir). Kol hareket
        # etmeden ÖNCE tam açık konuma getir.
        robot_controller.open_gripper(synchronous=True)

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
        # Her durumda vidalama motorunu kapat (çıkış açık kalmasın)
        try:
            robot_controller.screwdriver(False)
        except Exception as e:
            robot_controller.get_logger().error(f"Vidalama kapatılamadı: {str(e)}")
        robot_controller.move_to_joint_angles(home_joints, synchronous=True)
        rclpy.shutdown()
        executor_thread.join()


if __name__ == "__main__":
    main()