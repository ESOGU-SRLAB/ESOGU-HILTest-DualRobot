#!/usr/bin/env python3
"""
Dille pick and place: doğal dil komutu -> Gemini ER 2 -> MoveIt -> UR10e + VGC10.

Akış:
    /gemini/command ("düz panelin ortasından tut ve konveyöre koy")
        -> SCAN pozuna git (bilek kamerası tezgahı görsün)
        -> derinlikten ER görüntüsü üret (gerçek SICK'in RGB'si yok)
        -> ER 2 plan: {"pick": "flat panel", "place": "conveyor belt"}
        -> ER 2 pointing + deprojeksiyon + yüzey düzlemi: konum VE normal
        -> düzlemsellik + eğim + çalışma alanı denetimleri
        -> APPROACH -> TOUCH -> VAKUM AÇ -> LIFT -> TRANSFER -> BIRAK -> HOME

VAKUM MODU FARKLARI (use_vacuum_gripper:=true):
  - VGC10'da aktüe edilen eklem yok; kavrama Modbus komutuyla yapılır
    (vgc10_msgs/OnRobotVGOutput). MoveIt2Gripper bu donanımda tanımsızdır.
  - Vakum SRDF'inde "gripper" planlama grubu YOK; uç link suction_cup.
  - Emme kabı yüzeye DİK oturmak zorunda, bu yüzden kavrama oryantasyonu
    sabit değil, yüzey normalinden hesaplanır (bkz. grasp.py).
  - Yaklaşma ve kaldırma dünya Z'si boyunca değil, NORMAL boyunca yapılır;
    dikey bir yüzeyden parça çekmek ancak böyle doğru olur.
"""

from __future__ import annotations

import faulthandler
import json
import math
import signal
import sys
import threading
import time
from datetime import datetime
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from .er_client import make_er_client
from .grasp import (
    approach_quaternion, is_graspable, quaternion_to_matrix,
    surface_frame_quaternion,
)
from .locator import GeminiLocator
from .reachability import ReachabilityChecker
from .vacuum import VacuumGripper


Point3 = Tuple[float, float, float]


class GeminiPickPlaceNode(Node):
    # pymoveit2 MoveIt2.__is_motion_requested / __is_executing (name mangling)
    _MOTION_REQUESTED_ATTR = "_MoveIt2__is_motion_requested"
    _EXECUTING_ATTR = "_MoveIt2__is_executing"

    def __init__(self):
        super().__init__("gemini_pick_place")

        # main() burayı doldurur. Node.executor zayıf referans tutar ve düğüm
        # executor'dan düştüğünde None döner; geri bağlanabilmek için güçlü bir
        # referans şart (bkz. _ensure_attached).
        self._owner_executor: Optional[MultiThreadedExecutor] = None

        # --- ER / algı ---
        self.declare_parameter("backend", "gemini")
        self.declare_parameter("model", "gemini-robotics-er-2-preview")
        self.declare_parameter("thinking_level", "high")
        self.declare_parameter("jpeg_quality", 90)
        self.declare_parameter("vertex_project", "")
        self.declare_parameter("vertex_location", "us-central1")

        self.declare_parameter("image_topic", "/sim/image")
        self.declare_parameter("cloud_topic", "/sim/pointcloud")
        self.declare_parameter("depth_topic", "/sim/depth/image")
        self.declare_parameter("camera_info_topic", "/sim/camera_info")
        self.declare_parameter("intensity_topic", "/intensity")
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("camera_frame_override", "")
        self.declare_parameter("deprojection", "auto")
        self.declare_parameter("sample_window", 5)

        # ER görüntü kaynağı: render (derinlikten) | rgb (yalnız sim)
        self.declare_parameter("er_image_source", "render")
        self.declare_parameter("render_mode", "normals")
        self.declare_parameter("render_min_m", 0.3)
        self.declare_parameter("render_max_m", 4.0)
        self.declare_parameter("publish_er_image", True)
        self.declare_parameter("er_image_rate_hz", 2.0)

        # --- yüzey / kavrama denetimi ---
        self.declare_parameter("patch_radius_px", 12)
        self.declare_parameter("patch_depth_band", 0.05)
        self.declare_parameter("patch_min_points", 25)
        self.declare_parameter("max_surface_rms", 0.004)   # 4 mm düzlemsellik artığı
        self.declare_parameter("min_surface_inliers", 40)
        self.declare_parameter("max_surface_tilt_deg", 180.0)  # 180 = eğim denetimi kapalı
        self.declare_parameter("require_surface", True)

        # --- hareket ---
        # real : /move_action (gerçek MoveIt). real_to_sim_bridge zaten Gazebo'yu
        #        aynaladığı için sim'de de hareket görülür ve gerçek robota
        #        geçerken hiçbir şey değişmez. digital_twin GEREKMEZ.
        # sim  : /sim/move_action (yalnız digital_twin:=true iken vardır).
        self.declare_parameter("moveit_backend", "real")
        self.declare_parameter("planning_frame", "world")
        # Boş bırakılırsa backend'e göre seçilir (real -> ur10e_suction_cup).
        self.declare_parameter("end_effector", "")
        # Düğüm açılışında SCAN pozuna git (kamera konveyörü görsün).
        self.declare_parameter("move_to_scan_on_start", True)
        self.declare_parameter("startup_delay_sec", 8.0)
        self.declare_parameter("planner_id", "RRTConnectkConfigDefault")
        self.declare_parameter("max_velocity", 0.2)
        self.declare_parameter("max_acceleration", 0.2)
        self.declare_parameter("planning_attempts", 10)
        self.declare_parameter("planning_time", 10.0)

        # 7 eklem: base_to_robot_mount (ray) + 6 UR eklemi
        # Çok pozlu tarama. Bilek kamerası kolla hareket ettiği için tek bakış
        # pozundan hücrenin tamamı görünmüyor: konveyör bir pozdan, toolkit
        # bambaşka bir pozdan görünüyor (aralarında 1.17 m ray hareketi var).
        # Tespitler world frame'ine çevrildiği için farklı pozlardan gelen
        # sonuçlar tek bir haritada birikir.
        #
        # ROS parametreleri iç içe sözlük listesi taşımadığı için paralel
        # diziler kullanılıyor; scan_pose_joints 7'nin katı olarak düzleştirilir.
        self.declare_parameter("scan_pose_names", [""])
        self.declare_parameter("scan_pose_joints", [0.0])
        self.declare_parameter("scan_pose_height_lo_m", [0.0])
        self.declare_parameter("scan_pose_height_hi_m", [0.0])
        self.declare_parameter("scan_pose_render_mode", [""])
        self.declare_parameter("scan_settle_sec", 1.0)
        # Uç eleman geometrisi. İkisi de `ur10e_suction_cup` mesh'inden ölçüldü.
        # Emme yönü eksene hizalı DEĞİL: kap frame X'inden 30 derece eğik.
        self.declare_parameter("tool_approach_vector", [0.8660254, 0.5, 0.0])
        # Kap ağzı, eksen boyunca 84.2 mm. `measure_tcp` ile ÖLÇÜLÜR, elle
        # yazılmaz (bkz. tool_geometry.py).
        self.declare_parameter("tool_tip_offset", [0.072920, 0.042100, 0.0])
        self.declare_parameter("normal_snap_deg", 15.0)
        # Bırakma hedefinde dikeyden bu kadar sapmış bir yüzey normali kabul
        # edilmez, yerine yukarıdan iniş kullanılır (bkz. _grasp_frame).
        # 0 verilirse denetim kapanır ve ölçülen normal olduğu gibi kullanılır.
        self.declare_parameter("place_max_normal_tilt_deg", 45.0)
        self.declare_parameter("scan_joints", [1.0, 0.0, -1.5708, 0.0, -1.5708, 0.0, 0.0])
        self.declare_parameter("home_joints", [1.0, 0.0, -1.5708, 0.0, -1.5708, 0.0, 0.0])

        # Yüzey normali bulunamazsa kullanılan yedek oryantasyon (yukarıdan aşağı).
        self.declare_parameter("fallback_quat_xyzw", [1.0, 0.0, 0.0, 0.0])
        # Kartezyen planın KABUL edilmesi için tamamlanması gereken oran.
        # pymoveit2 fraction >= eşik olduğunda planı YÜRÜTÜR (moveit2.py:689) -
        # ve kısaltılmış yörünge de yürütülür. 0.9'da 148 mm'lik TOUCH hareketi
        # 14.8 mm eksik kalabiliyor ve bu "başarılı" sayılıyordu. Vakumlu
        # kavramada eksik kalan son milimetreler doğrudan tutamamak demek.
        self.declare_parameter("cartesian_min_fraction", 0.99)
        # Hedefi KAVRAMADAN ÖNCE MoveIt'e sor (/compute_ik + çarpışma denetimi).
        # Kapatmak, ulaşılamaz bir hedefi ancak parça elde kaldıktan sonra
        # öğrenmek demektir.
        self.declare_parameter("validate_targets", True)
        self.declare_parameter("ik_timeout_sec", 3.0)
        # Yaklaşma mesafesi UYARLANIR: ilki tutmazsa sırayla küçültülür.
        # Ölçüldü (11 Ağu 2026, toolkit üst sırası): 0.15'te önkol kablo
        # kanalına giriyor, 0.10 ve altı geçerli. Tek sabit değer, raf gözü
        # gibi dar yerlerle bant gibi açık yerlere aynı anda uymuyor.
        self.declare_parameter(
            "approach_distance_candidates", [0.15, 0.12, 0.10, 0.08, 0.06])
        self.declare_parameter("approach_distance", 0.15)  # normal boyunca yaklaşma
        self.declare_parameter("touch_offset", 0.002)      # kabın yüzeye bastırma payı
        self.declare_parameter("lift_distance", 0.20)      # normal boyunca çekme
        self.declare_parameter("place_clearance", 0.05)
        # Bırakmadan önce place_clearance'a inilsin mi. Raf gözünde iniş
        # planlanamıyor (gövde sığmıyor), parça da yukarıdan bırakılabiliyor.
        self.declare_parameter("descend_before_release", False)

        # --- kavrayıcı ---
        self.declare_parameter("gripper_type", "vacuum")  # vacuum | none
        self.declare_parameter("vacuum_command_topic", "/OnRobotVGOutput")
        self.declare_parameter("vacuum_status_topic", "/OnRobotVGInput")
        self.declare_parameter("vacuum_pct", 60)
        self.declare_parameter("vacuum_use_both_channels", True)
        self.declare_parameter("vacuum_require_confirmation", False)
        self.declare_parameter("vacuum_confirm_pct", 15.0)
        self.declare_parameter("vacuum_timeout_sec", 3.0)
        # Gazebo DetachableJoint köprüsü. Gerçek robotta bu topic'leri kimse
        # dinlemez; boş bırakılırsa hiç yayınlanmaz.
        self.declare_parameter("sim_attach_topic", "/vacuum/attach")
        self.declare_parameter("sim_detach_topic", "/vacuum/detach")

        # --- taşınan parçanın çarpışma gövdesi ---
        # Kavranan parça artık robotun bir parçasıdır; MoveIt bilmezse onu
        # rafa/konveyöre/kolun kendisine sürter. Boş liste = kapalı.
        self.declare_parameter("payload_size", [0.14, 0.083, 0.03])
        self.declare_parameter("payload_object_id", "gemini_payload")
        # Parçaya DEĞMESİ normal olan link'ler. Verilmezse kavranan an
        # çarpışma sayılır ve hiçbir plan üretilemez. Boş = önekten türet.
        self.declare_parameter("payload_touch_links", [""])
        # Boyutu derinlikten ÖLÇ (kapalıysa hep payload_size kullanılır).
        self.declare_parameter("payload_measure", True)
        self.declare_parameter("payload_margin_m", 0.005)
        self.declare_parameter("payload_min_height_m", 0.004)
        self.declare_parameter("payload_max_height_m", 0.30)

        # --- güvenlik ---
        self.declare_parameter("workspace_min", [-2.0, -2.0, 0.60])
        self.declare_parameter("workspace_max", [2.0, 3.0, 2.00])
        self.declare_parameter("dry_run", False)

        def get(name):
            return self.get_parameter(name).value

        self.world_frame = str(get("world_frame"))
        self.planning_frame = str(get("planning_frame"))
        self.approach_distance = float(get("approach_distance"))
        self.cartesian_min_fraction = float(get("cartesian_min_fraction"))
        self.validate_targets = bool(get("validate_targets"))
        self.ik_timeout_sec = float(get("ik_timeout_sec"))
        candidates = [float(v) for v in get("approach_distance_candidates") if v > 0]
        # Yapılandırılmış approach_distance her zaman İLK denenen olmalı: liste
        # bir yedek zinciridir, ayarı sessizce ezen bir şey değil.
        self.approach_candidates = [self.approach_distance] + [
            d for d in candidates if abs(d - self.approach_distance) > 1e-9
        ]
        self.touch_offset = float(get("touch_offset"))
        self.lift_distance = float(get("lift_distance"))
        self.place_clearance = float(get("place_clearance"))
        self.descend_before_release = bool(get("descend_before_release"))
        self.fallback_quat = [float(v) for v in get("fallback_quat_xyzw")]
        self.place_max_normal_tilt_deg = float(get("place_max_normal_tilt_deg"))
        self.scan_joints = [float(v) for v in get("scan_joints")]
        self.home_joints = [float(v) for v in get("home_joints")]
        self.scan_settle_sec = float(get("scan_settle_sec"))
        self.tool_approach_vector = [float(v) for v in get("tool_approach_vector")]
        self.tool_tip_offset = np.asarray(
            [float(v) for v in get("tool_tip_offset")], dtype=np.float64
        )
        self.scan_poses = self._load_scan_poses(get)
        self.workspace_min = [float(v) for v in get("workspace_min")]
        self.workspace_max = [float(v) for v in get("workspace_max")]
        self.dry_run = bool(get("dry_run"))
        self.gripper_type = str(get("gripper_type"))

        self.max_surface_rms = float(get("max_surface_rms"))
        self.min_surface_inliers = int(get("min_surface_inliers"))
        self.max_surface_tilt_deg = float(get("max_surface_tilt_deg"))
        self.require_surface = bool(get("require_surface"))

        cbg = ReentrantCallbackGroup()

        render_mode = str(get("render_mode"))
        er_source = str(get("er_image_source"))
        er_client = make_er_client(
            backend=str(get("backend")),
            model=str(get("model")),
            thinking_level=str(get("thinking_level")),
            jpeg_quality=int(get("jpeg_quality")),
            project=str(get("vertex_project")),
            location=str(get("vertex_location")),
            context_key="rgb" if er_source == "rgb" else render_mode,
            tool="vacuum" if self.gripper_type == "vacuum" else "parallel",
        )

        self.locator = GeminiLocator(
            node=self,
            er_client=er_client,
            image_topic=str(get("image_topic")),
            cloud_topic=str(get("cloud_topic")),
            depth_topic=str(get("depth_topic")),
            camera_info_topic=str(get("camera_info_topic")),
            intensity_topic=str(get("intensity_topic")),
            world_frame=self.world_frame,
            camera_frame_override=str(get("camera_frame_override")),
            deprojection=str(get("deprojection")),
            sample_window=int(get("sample_window")),
            er_image_source=er_source,
            render_mode=render_mode,
            render_min_m=float(get("render_min_m")),
            render_max_m=float(get("render_max_m")),
            publish_er_image=bool(get("publish_er_image")),
            er_image_rate_hz=float(get("er_image_rate_hz")),
            patch_radius_px=int(get("patch_radius_px")),
            patch_depth_band=float(get("patch_depth_band")),
            patch_min_points=int(get("patch_min_points")),
            tool_approach_vector=self.tool_approach_vector,
            normal_snap_deg=float(get("normal_snap_deg")),
            measure_payload=bool(get("payload_measure")),
            payload_margin_m=float(get("payload_margin_m")),
            payload_min_height_m=float(get("payload_min_height_m")),
            payload_max_height_m=float(get("payload_max_height_m")),
        )

        # MoveIt backend seçimi. pymoveit2_sim ve pymoveit2_real, action adlarını
        # GÖRELİ olarak sabitlemiş fork'lar: "sim/move_action" ve "move_action".
        # Bu düğüm global namespace'te kaldığı için doğru hedefe çözülürler -
        # düğümü /sim'e itmek "sim/sim/move_action" üretir ve hiçbir şeye bağlanmaz.
        self.moveit_backend = str(get("moveit_backend")).lower()
        if self.moveit_backend == "sim":
            from pymoveit2_sim import MoveIt2 as MoveIt2Class
            from pymoveit2_sim.robots import ur as robot
        elif self.moveit_backend == "real":
            from pymoveit2_real import MoveIt2 as MoveIt2Class
            from pymoveit2_real.robots import ur as robot
        else:
            raise ValueError(
                f"moveit_backend {self.moveit_backend!r} geçersiz (real | sim)"
            )
        self.robot = robot

        end_effector = str(get("end_effector")).strip()
        if not end_effector:
            # real SRDF: <chain base_link="ur10e_table" tip_link="ur10e_suction_cup"/>
            end_effector = f"{robot.prefix}suction_cup" if self.gripper_type == "vacuum" \
                else robot.end_effector_name()
        self.end_effector = end_effector

        self.arm = MoveIt2Class(
            node=self,
            joint_names=robot.joint_names(),
            base_link_name=self.planning_frame,
            end_effector_name=end_effector,
            group_name=robot.MOVE_GROUP_ARM,
            callback_group=cbg,
        )
        self.arm.planner_id = str(get("planner_id"))
        self.arm.max_velocity = float(get("max_velocity"))
        self.arm.max_acceleration = float(get("max_acceleration"))
        self.arm.cartesian_avoid_collisions = True
        if hasattr(self.arm, "planning_attempts"):
            self.arm.planning_attempts = int(get("planning_attempts"))
        if hasattr(self.arm, "planning_time"):
            self.arm.planning_time = float(get("planning_time"))

        # VGC10: eklem değil, Modbus komutu. Vakum SRDF'inde gripper grubu
        # olmadığı için MoveIt2Gripper burada kullanılamaz.
        self.vacuum: Optional[VacuumGripper] = None
        if self.gripper_type == "vacuum":
            self.vacuum = VacuumGripper(
                node=self,
                command_topic=str(get("vacuum_command_topic")),
                status_topic=str(get("vacuum_status_topic")),
                vacuum_pct=int(get("vacuum_pct")),
                use_both_channels=bool(get("vacuum_use_both_channels")),
                grip_confirm_pct=float(get("vacuum_confirm_pct")),
                grip_timeout_sec=float(get("vacuum_timeout_sec")),
                require_confirmation=bool(get("vacuum_require_confirmation")),
                sim_attach_topic=str(get("sim_attach_topic")),
                sim_detach_topic=str(get("sim_detach_topic")),
            )

        # Taşınan parçanın çarpışma gövdesi
        size = [float(v) for v in get("payload_size")]
        self.payload_size = tuple(size) if len(size) == 3 and all(
            v > 0 for v in size) else None
        self.payload_id = str(get("payload_object_id"))
        touch_links = [str(v) for v in get("payload_touch_links") if str(v).strip()]
        self.payload_touch_links = touch_links or [
            f"{robot.prefix}suction_cup",
            f"{robot.prefix}vacuum_gripper",
            f"{robot.prefix}tool0",
            f"{robot.prefix}wrist_3_link",
        ]
        if self.payload_size is None:
            self.get_logger().warn(
                "payload_size verilmedi: taşınan parça MoveIt sahnesine "
                "EKLENMEYECEK, yani planlayıcı onu görmeyecek."
            )

        # Hedef doğrulaması: aynı MoveIt örneğine /compute_ik ile sorar.
        # Reentrant grupta, çünkü yanıtı görev thread'i beklerken düğümün
        # varsayılan (MutuallyExclusive) grubu MoveIt trafiğiyle dolu olabilir.
        self.reachability = ReachabilityChecker(
            node=self,
            group_name=robot.MOVE_GROUP_ARM,
            ik_link=self.end_effector,
            planning_frame=self.planning_frame,
            timeout_sec=self.ik_timeout_sec,
            callback_group=cbg,
        ) if self.validate_targets else None

        self.status_pub = self.create_publisher(String, "/gemini/status", 10)

        # pymoveit2'nin bayrakları name-mangled: MoveIt2.__is_executing ->
        # _MoveIt2__is_executing. _wait_motion() bunları yokluyor (bkz. aşağısı).
        self._can_poll_motion = all(
            hasattr(self.arm, attr)
            for attr in (self._MOTION_REQUESTED_ATTR, self._EXECUTING_ATTR)
        )
        if not self._can_poll_motion:
            self.get_logger().error(
                "pymoveit2 hareket bayrakları bulunamadı; wait_until_executed() "
                "kullanılacak - bu düğümü executor'dan düşürür!"
            )

        # Komut aboneliği KENDİ Reentrant grubunda olmalı, düğümün varsayılan
        # grubunda değil.
        #
        # pymoveit2_real, MoveGroup/ExecuteTrajectory action istemcilerini ve
        # plan/planning_scene servis istemcilerini callback_group VERMEDEN
        # oluşturuyor (moveit2.py:172, 238, 210, 224, 276, 292 - yalnız
        # compute_fk/ik grubu alıyor). Bunlar düğümün varsayılan
        # MutuallyExclusive grubuna düşüyor ve orada aynı anda tek callback
        # çalışabiliyor.
        self._command_cbg = ReentrantCallbackGroup()
        self.create_subscription(
            String, "/gemini/command", self._on_command, 10,
            callback_group=self._command_cbg,
        )

        self._payload_attached = False
        self._busy = threading.Lock()
        # Hangi tarama pozunda olduğumuz; -1 = bilinmiyor (gereksiz hareketi
        # atlamak için kullanılıyor, her ER çağrısı ~5 s).
        self._current_scan_pose = -1

        # Açılışta SCAN pozuna git: kamera bilekte olduğu için, sorgudan önce
        # konveyörü gören sabit bir poza gitmezse ER 2'ye hücre duvarları sorulur.
        #
        # ROS timer'ı DEĞİL, düz bir wall-clock thread'i kullanılıyor. use_sim_time
        # true iken ROS timer'ı sim saatine bağlanıyor ve bu tek seferlik açılış
        # eylemi hiç tetiklenmiyordu (ölçüldü: sim_time=false iken çalışıyor,
        # true iken 24 sn boyunca hiç ateşlemedi). Bir bootstrap adımının sim
        # saatiyle bir ilgisi yok; gerçek zamanda beklemesi hem doğru hem de
        # Gazebo duraklatılsa bile çalışır.
        if bool(get("move_to_scan_on_start")):
            threading.Thread(
                target=self._startup_scan_after,
                args=(float(get("startup_delay_sec")),),
                daemon=True,
            ).start()

        # Uç eleman sabitlerini robotun KENDİ modeline karşı doğrula. Bloklamaz,
        # başarısız olursa yalnız sessiz kalır (bkz. _verify_tool_geometry).
        threading.Thread(target=self._verify_tool_geometry, daemon=True).start()

        mode = "DRY RUN (hareket yok)" if self.dry_run else "canlı"
        self.get_logger().info(
            f"gemini_pick_place hazır [{mode}] | ER={er_client.name} | "
            f"MoveIt={self.moveit_backend} ({robot.MOVE_GROUP_ARM}) | "
            f"kavrayıcı={self.gripper_type} | uç={end_effector} | "
            f"ER görüntüsü={er_source}"
            + (f"/{render_mode}" if er_source == "render" else "")
        )

    # --- durum yayını -----------------------------------------------------

    def _status(self, state: str, detail: str = "", **extra) -> None:
        payload = {
            "timestamp": datetime.now().isoformat(),
            "state": state,
            "detail": detail,
        }
        payload.update(extra)
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.status_pub.publish(msg)
        self.get_logger().info(f"[{state}] {detail}")

    # --- hareket yardımcıları ---------------------------------------------

    def _ensure_attached(self) -> None:
        """
        Düğümü kendi MultiThreadedExecutor'ına geri bağla.

        pymoveit2 içindeki her `rclpy.spin_once(self._node, ...)` çağrısı düğümü
        global SingleThreadedExecutor'a EKLER (moveit2.py:527, 639, 747, 1189,
        1281). rclpy'de bir düğüm aynı anda tek bir executor'a ait olabilir:
        Node.executor setter'ı (node.py:272-284) yeni executor'ı atamadan önce
        eskisinde `remove_node(self)` çağırır. spin_once da `finally` bloğunda
        düğümü global executor'dan çıkarır (rclpy/__init__.py:207-210). Net
        sonuç: ilk MoveIt hareketinden sonra düğüm HİÇBİR executor'a ait
        değildir. Süreç yaşar, publisher'lar ve düz thread'ler çalışır, ama
        abonelikler ve parametre servisleri bir daha asla tetiklenmez.

        add_node() zaten üye olan düğümde no-op; değilse geri ekler ve wait
        set'i yeniden kurmak için guard'ı tetikler.
        """
        executor = self._owner_executor
        if executor is not None:
            executor.add_node(self)

    def _begin_motion(self) -> None:
        """Komut göndermeden ÖNCE başarı bayrağını sıfırla.

        pymoveit2 planlama başarısız olduğunda ("Cannot execute motion because
        the provided/planned trajectory is invalid") hiç hedef göndermez ve
        __is_motion_requested'ı hiç set etmez. Bayrağı sıfırlamazsak _wait_motion
        anında dönüp BİR ÖNCEKİ hareketten kalan motion_suceeded=True değerini
        okur - yani başarısız hareket başarılı görünür.

        Bu tam olarak yaşandı: 10 Ağu 2026'da kartezyen TOUCH planı %46.7'de
        kaldı, kol parçaya hiç değmedi, ama _move_pose True döndü, vakum havada
        açıldı ve görev [DONE] "tamamlandı" diye bitti.
        """
        try:
            self.arm.motion_suceeded = False
        except AttributeError:  # pragma: no cover - fork değişirse
            pass

    def _wait_motion(self, timeout_sec: float = 120.0) -> bool:
        """
        Hareketin bitmesini bekle - `wait_until_executed()` KULLANMADAN.

        Onun döngüsü `rclpy.spin_once(node)` çağırıyor; yukarıdaki nedenle bu
        düğümü executor'dan kalıcı olarak düşürüyor. Bayrakları güncelleyen
        MoveIt callback'lerini zaten bizim executor'ımız çalıştırıyor, o yüzden
        burada sadece yokluyoruz.

        DİKKAT: _begin_motion() bu çağrıdan önce gelmeli. Hiç hedef gönderilmemiş
        olması da bir başarısızlıktır ve buradan False dönmesi ona bağlı.
        """
        if not self._can_poll_motion:
            ok = self.arm.wait_until_executed()
            self._ensure_attached()
            return bool(ok)

        deadline = time.monotonic() + timeout_sec
        while True:
            busy = getattr(self.arm, self._MOTION_REQUESTED_ATTR, False) or getattr(
                self.arm, self._EXECUTING_ATTR, False
            )
            if not busy:
                return bool(getattr(self.arm, "motion_suceeded", False))
            if time.monotonic() > deadline:
                self.get_logger().error(
                    f"Hareket {timeout_sec:.0f} s içinde bitmedi, vazgeçildi"
                )
                return False
            time.sleep(0.02)

    def _move_joints(self, joints: List[float], label: str, retries: int = 3) -> bool:
        if self.dry_run:
            self.get_logger().info(f"[DRY RUN] joint hedefi atlandı: {label}")
            return True
        for attempt in range(1, retries + 1):
            self.get_logger().info(f"ARM joint -> {label} (deneme {attempt}/{retries})")
            self._begin_motion()
            self.arm.move_to_configuration(joints)
            # plan_async() de joint_state beklerken spin_once çağırabiliyor
            # (moveit2.py:639); beklemeye girmeden önce düğümü geri bağla,
            # yoksa goal/result callback'leri hiç çalışmaz ve _wait_motion
            # boşuna zaman aşımına uğrar.
            self._ensure_attached()
            ok = self._wait_motion()
            self._ensure_attached()
            if ok:
                return True
            time.sleep(0.4)
        self.get_logger().error(f"ARM joint hedefi başarısız: {label}")
        return False

    def _frame_position_for_tip(
        self, tip_position: Point3, quat_xyzw: Sequence[float]
    ) -> Point3:
        """Kap AĞZI istenen noktaya gelsin diye uç eleman FRAME'inin gideceği yer.

        MoveIt hedefi `end_effector` link frame'inin ORİJİNİ için verilir, ama
        parçaya değen yer kabın ağzı. `ur10e_suction_cup` mesh'inde ağız
        halkası, frame orijininden emme ekseni boyunca 84.2 mm ötede.

        Bu sayı İKİ YÖNLÜ hassastır ve hatası doğrudan mesafeye yazılır:
          offset FAZLA  -> frame gereğinden geride durur, kap yüzeye DEĞMEZ
          offset EKSİK  -> kap yüzeyin içine bastırır
        156 mm yazıldığı sürece (11 Ağu 2026'da düzeltildi) kap her hedefin
        71.8 mm üstünde kalıyordu. Ölçüm için: tool_geometry.measure_tcp.
        """
        if not np.any(self.tool_tip_offset):
            return tip_position
        rotation = quaternion_to_matrix(quat_xyzw)
        target = np.asarray(tip_position, dtype=np.float64) - rotation @ self.tool_tip_offset
        return (float(target[0]), float(target[1]), float(target[2]))

    def _move_pose(
        self,
        position: Point3,
        quat_xyzw: Sequence[float],
        label: str,
        cartesian: bool = False,
        retries: int = 2,
    ) -> bool:
        # `position` KAP AĞZININ gitmesi istenen nokta; MoveIt'e verilecek olan
        # ise frame orijini.
        frame_position = self._frame_position_for_tip(position, quat_xyzw)

        if self.dry_run:
            self.get_logger().info(
                f"[DRY RUN] pose atlandı: {label} -> uç "
                f"({position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f}) "
                f"frame ({frame_position[0]:.3f}, {frame_position[1]:.3f}, "
                f"{frame_position[2]:.3f}) "
                f"q=({', '.join(f'{q:.3f}' for q in quat_xyzw)})"
            )
            return True
        for attempt in range(1, retries + 1):
            self.get_logger().info(
                f"ARM pose -> {label} uç=({position[0]:.3f}, {position[1]:.3f}, "
                f"{position[2]:.3f}) frame=({frame_position[0]:.3f}, "
                f"{frame_position[1]:.3f}, {frame_position[2]:.3f}) "
                f"deneme {attempt}/{retries}"
            )
            self._begin_motion()
            self.arm.move_to_pose(
                position=list(frame_position),
                quat_xyzw=list(quat_xyzw),
                cartesian=cartesian,
                cartesian_max_step=0.005,
                cartesian_fraction_threshold=self.cartesian_min_fraction,
            )
            self._ensure_attached()
            ok = self._wait_motion()
            self._ensure_attached()
            if ok:
                return True
            time.sleep(0.4)
        self.get_logger().error(f"ARM pose hedefi başarısız: {label}")
        return False

    def _grip(self) -> bool:
        if self.vacuum is None or self.dry_run:
            return True
        return self.vacuum.grip()

    def _release(self) -> bool:
        if self.vacuum is None or self.dry_run:
            return True
        return self.vacuum.release()

    # --- taşınan parçanın çarpışma gövdesi ---------------------------------

    def _payload_box(
        self, detection: Dict[str, Any], contact: Point3, quat_xyzw: Sequence[float]
    ):
        """(boyut, merkez, kuaterniyon, kaynak) - ölçüm varsa onu, yoksa config.

        Ölçüm TESPİT anında yapılıyor (locator), çünkü kavrama anında kamera
        artık parçayı değil emme kabını görüyor.
        """
        measured = detection.get("payload") if isinstance(detection, dict) else None
        if measured:
            return (
                tuple(measured["size"]),
                tuple(measured["centre"]),
                tuple(measured["quat_xyzw"]),
                "ölçüldü",
            )
        if self.payload_size is None:
            return None
        length, width, height = self.payload_size
        normal = self._payload_normal(quat_xyzw)
        # `contact` parçanın ÜST yüzeyi (kabın değdiği yer); kutunun merkezi
        # normal boyunca yarım yükseklik AŞAĞIDA.
        centre = self._offset(contact, normal, -height / 2.0)
        # Kutunun oryantasyonu KAVRAMA kuaterniyonu DEĞİL: o, uç elemanın
        # 30 derecelik eğik eksenini içerir ve kutuyu eğik gösterirdi. Kutunun
        # +Z'si yüzey normali boyunca; düzlem içi dönme bu yolda ÖLÇÜLMÜYOR,
        # dünya X'ine sabitleniyor (ölçüm yolunda gerçekten ölçülüyor).
        return (self.payload_size, centre, surface_frame_quaternion(normal),
                "config (ölçülemedi)")

    def _attach_payload(
        self, detection: Dict[str, Any], contact: Point3, quat_xyzw: Sequence[float]
    ) -> None:
        """Kavranan parçayı planlama sahnesine ekleyip uca iliştirir.

        NEDEN: parça kavrandıktan sonra robotun bir parçasıdır. MoveIt bunu
        bilmezse kutuyu rafın kenarına, konveyöre ya da kolun kendisine
        sürtebilir - planlayıcı için orada hiçbir şey yoktur. İliştirilen
        nesne, kolun her hareketinde onunla birlikte taşınır ve çarpışma
        denetimine girer.
        """
        if self.dry_run:
            return
        resolved = self._payload_box(detection, contact, quat_xyzw)
        if resolved is None:
            return
        (length, width, height), centre, box_quat, source = resolved

        try:
            self.arm.add_collision_box(
                id=self.payload_id,
                size=(length, width, height),
                position=list(centre),
                quat_xyzw=list(box_quat),
                frame_id=self.planning_frame,
            )
            time.sleep(0.3)  # sahnenin güncellenmesi için
            # touch_links: kavrayıcının parçaya DEĞMESİ normaldir; bunları
            # vermezsek MoveIt kavranan anı çarpışma sayar ve hiçbir plan
            # üretemez.
            self.arm.attach_collision_object(
                id=self.payload_id,
                link_name=self.end_effector,
                touch_links=self.payload_touch_links,
            )
            time.sleep(0.3)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"Parça çarpışma gövdesi eklenemedi: {exc}")
            return
        self._payload_attached = True
        self.get_logger().info(
            f"Parça MoveIt sahnesine iliştirildi: {self.payload_id} "
            f"({length * 1000:.0f}x{width * 1000:.0f}x{height * 1000:.0f} mm, {source})"
        )

    def _payload_normal(self, quat_xyzw: Sequence[float]) -> Point3:
        """Kavrama oryantasyonundan yüzey normalini geri çıkarır.

        Kavrama kuaterniyonu, uç eleman eksenini -normal'e eşler; dolayısıyla
        normal = -(R * eksen).
        """
        rotation = quaternion_to_matrix(quat_xyzw)
        axis = np.asarray(self.tool_approach_vector, dtype=np.float64)
        axis = axis / (np.linalg.norm(axis) or 1.0)
        direction = -(rotation @ axis)
        return (float(direction[0]), float(direction[1]), float(direction[2]))

    def _detach_payload(self, force: bool = False) -> None:
        """Parçayı uçtan ayırır ve sahneden siler.

        Silmek şart: yalnızca detach edilirse kutu, bırakıldığı yerde SABİT bir
        engel olarak sahnede kalır ve bir sonraki görevde oraya yaklaşan kol
        hayalet bir engelle karşılaşır.

        force=True: görev başında, önceki bir koşudan (ya da çöken bir
        düğümden) kalıntı olmadığından emin olmak için.
        """
        if self.dry_run:
            return
        if not (force or self._payload_attached):
            return
        try:
            self.arm.detach_collision_object(self.payload_id)
            time.sleep(0.3)
            self.arm.remove_collision_object(self.payload_id)
            time.sleep(0.2)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"Parça sahneden kaldırılamadı: {exc}")
            return
        if self._payload_attached:
            self.get_logger().info("Parça MoveIt sahnesinden kaldırıldı")
        self._payload_attached = False

    # --- hedef seçimi ------------------------------------------------------

    def _load_scan_poses(self, get) -> List[Dict[str, Any]]:
        """scan_pose_* paralel dizilerini poz listesine çevirir.

        Diziler boşsa tek pozlu eski davranışa (scan_joints) düşer, böylece
        mevcut config'ler çalışmaya devam eder.
        """
        names = [str(v) for v in get("scan_pose_names") if str(v).strip()]
        flat = [float(v) for v in get("scan_pose_joints")]
        lo_list = [float(v) for v in get("scan_pose_height_lo_m")]
        hi_list = [float(v) for v in get("scan_pose_height_hi_m")]
        mode_list = [str(v).strip() for v in get("scan_pose_render_mode")]
        default_mode = str(get("render_mode"))

        n_joints = len(self.scan_joints)
        if not names or len(flat) < n_joints:
            self.get_logger().info(
                "scan_pose_* tanımlı değil; tek pozlu scan_joints kullanılıyor."
            )
            return [{
                "name": "scan",
                "joints": list(self.scan_joints),
                "height_lo_m": -0.005,
                "height_hi_m": 0.12,
                "render_mode": default_mode,
            }]

        if len(flat) != len(names) * n_joints:
            raise ValueError(
                f"scan_pose_joints uzunluğu {len(flat)}, beklenen "
                f"{len(names)} x {n_joints} = {len(names) * n_joints}. "
                "Diziyi poz sırasına göre düzleştirin."
            )

        poses: List[Dict[str, Any]] = []
        for index, name in enumerate(names):
            start = index * n_joints
            mode = mode_list[index] if index < len(mode_list) and mode_list[index] else default_mode
            poses.append({
                "name": name,
                "joints": flat[start:start + n_joints],
                "height_lo_m": lo_list[index] if index < len(lo_list) else -0.005,
                "height_hi_m": hi_list[index] if index < len(hi_list) else 0.12,
                "render_mode": mode,
            })
        self.get_logger().info(
            "Tarama pozları: " + ", ".join(
                f"{p['name']}/{p['render_mode']}"
                + (f" [{p['height_lo_m']:+.3f}..{p['height_hi_m']:+.3f} m]"
                   if p["render_mode"] == "relief" else "")
                for p in poses
            )
        )
        return poses

    def _goto_scan_pose(self, index: int) -> bool:
        """Tarama pozuna git, render penceresini ayarla, veriyi tazele."""
        pose = self.scan_poses[index]
        if not self._move_joints(pose["joints"], f"SCAN[{pose['name']}]"):
            return False
        self.locator.set_render_mode(pose["render_mode"])
        self.locator.set_height_window(pose["height_lo_m"], pose["height_hi_m"])
        # Kamera ve nokta bulutu bir önceki pozun karesini taşıyor olabilir;
        # taze veri gelmeden sorgu sorarsak ER'ye YANLIŞ sahneyi göstermiş
        # oluruz - üstelik tespit world frame'ine o anki TF ile çevrileceği
        # için sessizce hatalı bir 3D nokta üretirdi.
        time.sleep(max(0.0, self.scan_settle_sec))
        self._current_scan_pose = index
        return True

    def _find_across_poses(
        self, query: str, need_grasp: bool, label: str
    ) -> Optional[Dict[str, Any]]:
        """Hedefi tarama pozlarında sırayla arar, bulduğu ilk yerde durur.

        Bulunduğu poz `_current_scan_pose`'ta kalır; çağıran taraf isterse
        oradan devam eder. Erken çıkış önemli: her ER pointing çağrısı ~5 s.
        """
        for index, pose in enumerate(self.scan_poses):
            if index != self._current_scan_pose:
                if not self._goto_scan_pose(index):
                    self.get_logger().warn(
                        f"SCAN[{pose['name']}] pozuna gidilemedi, atlanıyor."
                    )
                    continue
            detection = self._select_target(query, need_grasp=need_grasp)
            if detection is not None:
                self.get_logger().info(
                    f"{label}: '{query}' -> SCAN[{pose['name']}] pozunda bulundu"
                )
                return detection
            self.get_logger().info(
                f"{label}: '{query}' SCAN[{pose['name']}] pozunda yok"
            )
        return None

    def _in_workspace(self, position: Point3) -> bool:
        return all(
            self.workspace_min[i] <= position[i] <= self.workspace_max[i] for i in range(3)
        )

    @staticmethod
    def _position_of(detection: Dict[str, Any]) -> Point3:
        position = detection["position"]
        return (position["x"], position["y"], position["z"])

    def _grasp_frame(
        self, detection: Dict[str, Any], for_grasp: bool = True
    ) -> Tuple[Point3, Point3, Sequence[float]]:
        """Tespitten (temas noktası, dışa bakan normal, kavrama kuaterniyonu) üretir.

        Yüzey yaması varsa merkezi ve normali kullanılır; yoksa dünya +Z normali
        ve sabit yedek oryantasyona düşülür.

        BIRAKMA hedefinde (for_grasp=False) neredeyse YATAY bir normal
        reddedilir ve yerine dünya +Z konur. Gerekçe fizik: bir parça yatay
        normalli bir yüzeye BIRAKILAMAZ, yerçekimi düşürür. Böyle bir normal
        her zaman ölçüm/işaretleme artefaktıdır.

        11 Ağu 2026'da ölçüldü: ER üst bölmeyi doğru gösterdi ama işaret ettiği
        piksel rafın ÖN YÜZÜNE düştü, normal (0, -1, 0) çıktı ve kol göze
        yukarıdan inmek yerine YANDAN girmeye çalıştı. Beş yaklaşma mesafesinin
        beşi de çarpıştı (temas: stackable_bin <-> wrist_2 / cable_channel).
        Aynı nokta yukarıdan inişle 15 cm'de geçerli.
        """
        surface = detection.get("surface")
        if surface is None:
            return self._position_of(detection), (0.0, 0.0, 1.0), self.fallback_quat

        centroid = (
            surface["centroid"]["x"], surface["centroid"]["y"], surface["centroid"]["z"]
        )
        normal = (surface["normal"]["x"], surface["normal"]["y"], surface["normal"]["z"])

        if not for_grasp and self.place_max_normal_tilt_deg > 0.0:
            vector = np.asarray(normal, dtype=np.float64)
            length = float(np.linalg.norm(vector))
            if length > 1e-9:
                tilt = float(np.degrees(np.arccos(
                    np.clip(vector[2] / length, -1.0, 1.0))))
                if tilt > self.place_max_normal_tilt_deg:
                    self.get_logger().info(
                        f"Bırakma normali dikeyden {tilt:.0f} derece sapmış "
                        f"(> {self.place_max_normal_tilt_deg:.0f}); yukarıdan "
                        "inişe çevrildi - yatay bir yüzeye parça bırakılamaz."
                    )
                    normal = (0.0, 0.0, 1.0)
                    return centroid, normal, list(approach_quaternion(
                        normal, approach_vector=tuple(self.tool_approach_vector)))

        return centroid, normal, surface["quat_xyzw"]

    @staticmethod
    def _offset(point: Point3, direction: Point3, distance: float) -> Point3:
        vector = np.asarray(direction, dtype=np.float64)
        length = float(np.linalg.norm(vector))
        if length < 1e-9:
            return point
        vector = vector / length
        return tuple(float(point[i] + vector[i] * distance) for i in range(3))

    def _select_target(self, query: str, need_grasp: bool) -> Optional[Dict[str, Any]]:
        """Sorguyu konumlandırır ve tüm denetimlerden geçen ilk tespiti döndürür.

        need_grasp=True iken yüzey düzlemselliği de aranır: vakum kabı eğik ya
        da kırık bir yüzeyde tutmaz, oraya gitmek boşa hareket olur.
        """
        # Yük ölçümü YALNIZCA kavranacak hedefte anlamlı. Bırakma hedefinde
        # ölçüm rafın kendisini "parça" sanıyordu (1339x1025x145 mm) - hem
        # yanıltıcı hem de aday başına boşa zaman.
        detections = self.locator.locate(
            query, max_items=5, measure_payload=need_grasp
        )
        for detection in detections:
            position = self._position_of(detection)
            if not self._in_workspace(position):
                self.get_logger().warn(
                    f"'{detection['label']}' çalışma alanı dışında, atlandı: "
                    f"({position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f})"
                )
                continue

            if need_grasp and self.gripper_type == "vacuum":
                surface = detection.get("surface")
                if surface is None:
                    if self.require_surface:
                        self.get_logger().warn(
                            f"'{detection['label']}' için yüzey düzlemi çıkarılamadı, "
                            "atlandı (require_surface=true)."
                        )
                        continue
                else:
                    from .grasp import SurfacePatch

                    patch = SurfacePatch(
                        centroid=(0.0, 0.0, 0.0),
                        normal=(0.0, 0.0, 1.0),
                        rms_residual=float(surface["rms_residual"]),
                        inliers=int(surface["inliers"]),
                        extent=float(surface["extent"]),
                    )
                    # Eğim denetimi ÖLÇÜLEN normale bakmalı. surface["normal"]
                    # dünya eksenine oturtulmuş hali; onunla denetlersek
                    # gerçekten eğik bir yüzey de "düz" görünür ve denetim
                    # anlamsızlaşır.
                    measured = surface.get("normal_measured", surface["normal"])
                    normal_world = (measured["x"], measured["y"], measured["z"])
                    ok, reason = is_graspable(
                        patch,
                        max_rms=self.max_surface_rms,
                        min_inliers=self.min_surface_inliers,
                        max_tilt_deg=self.max_surface_tilt_deg,
                        normal_world=normal_world,
                    )
                    if not ok:
                        self.get_logger().warn(
                            f"'{detection['label']}' vakumla tutulamaz: {reason}"
                        )
                        continue

            # Son denetim: kol oraya GERÇEKTEN gidebiliyor mu? Bu, ER'nin
            # noktasının doğru ama ulaşılmaz olduğu durumu ayıklar - ve
            # kavramadan ÖNCE ayıklar.
            approach_distance = self._validate_reachable(detection, need_grasp)
            if approach_distance is None:
                continue
            detection["approach_distance"] = approach_distance

            return detection
        return None

    def _validate_reachable(
        self, detection: Dict[str, Any], need_grasp: bool
    ) -> Optional[float]:
        """Hedefe gidilebilen bir yaklaşma mesafesi bulur; yoksa None.

        Yaklaşma mesafesini SIRAYLA küçültür. Tek sabit değer iki farklı yere
        birden uymuyor: bant gibi açık bir yüzeyde 150 mm iyi, raf gözünde
        aynı 150 mm kolu ya rafın tavanına ya kendi kablo kanalına sokuyor.
        Küçülterek denemek, geometriyi elle ayarlamadan ikisini de çözer.

        Temas pozu da denetlenir (need_grasp): yalnızca yaklaşmaya bakmak,
        son 150 mm'de çarpışan bir hedefi geçirirdi.
        """
        if self.reachability is None:
            return self.approach_distance

        contact, normal, quat = self._grasp_frame(detection, for_grasp=need_grasp)
        label = detection.get("label", "?")
        seed = getattr(self.arm, "joint_state", None)
        last_reason = ""

        for distance in self.approach_candidates:
            approach_tip = self._offset(contact, normal, distance)
            approach_frame = self._frame_position_for_tip(approach_tip, quat)
            result = self.reachability.check(approach_frame, quat, seed, label)
            if not result:
                last_reason = f"yaklaşma {distance * 100:.0f} cm: {result.reason}"
                continue

            if need_grasp:
                touch_tip = self._offset(contact, normal, self.touch_offset)
                touch_frame = self._frame_position_for_tip(touch_tip, quat)
                touch = self.reachability.check(touch_frame, quat, seed, label)
                if not touch:
                    last_reason = f"temas pozu: {touch.reason}"
                    # Temas pozu mesafeden bağımsız; küçültmek işe yaramaz.
                    break

            if abs(distance - self.approach_distance) > 1e-9:
                self.get_logger().info(
                    f"'{label}' için yaklaşma mesafesi {self.approach_distance * 100:.0f} "
                    f"cm yerine {distance * 100:.0f} cm (yakını çarpışıyordu)"
                )
            return distance

        # Konumu da yaz: "gidilemiyor" tek başına hiçbir şey öğretmiyor.
        # Reddedilen adayların NEREDE olduğunu görmeden ER'nin mi yanıldığı
        # yoksa geometrinin mi dar olduğu ayrılamıyor.
        self.get_logger().warn(
            f"'{label}' hedefine gidilemiyor - {last_reason} "
            f"[hedef ({contact[0]:.3f}, {contact[1]:.3f}, {contact[2]:.3f}), "
            f"normal ({normal[0]:.2f}, {normal[1]:.2f}, {normal[2]:.2f})]"
        )
        return None

    # --- ana akış ---------------------------------------------------------

    def _verify_tool_geometry(self) -> None:
        """tool_tip_offset'i robotun URDF mesh'ine karşı doğrular.

        NEDEN: bu sabitin hatası hareket mesafesine BİREBİR yazılır ama belirti
        algı hatası gibi görünür - kol yanlış yerde durur, insan da kamerayı
        suçlar. 11 Ağu 2026'da değer 156 mm girilmişti, mesh'te ölçülen 84.2 mm;
        kap her yüzeyin 71.8 mm üstünde kalıyor ve vakum boşa çekiyordu.

        Doğrulama BLOKLAMAZ ve HATA VERMEZ: URDF okunamazsa (robot_state_publisher
        yok, mesh dosyası taşınmış, mesh STL değil) sadece susar. Amaç görevi
        durdurmak değil, sessizce yanlış bir sayıyla çalışmayı imkânsız kılmak.
        """
        try:
            from rcl_interfaces.srv import GetParameters

            from .tool_geometry import measure_from_urdf

            time.sleep(2.0)  # keşif otursun
            client = self.create_client(
                GetParameters, "/robot_state_publisher/get_parameters",
                callback_group=self._command_cbg,
            )
            if not client.wait_for_service(timeout_sec=10.0):
                self.get_logger().debug(
                    "Uç eleman doğrulaması atlandı: robot_description servisi yok"
                )
                return
            try:
                future = client.call_async(
                    GetParameters.Request(names=["robot_description"])
                )
                deadline = time.monotonic() + 10.0
                while not future.done() and time.monotonic() < deadline:
                    time.sleep(0.05)
                response = future.result()
            finally:
                # İstemciyi bırak: tek seferlik bir doğrulama için düğümde
                # kalıcı bir varlık tutmanın anlamı yok, her wait set
                # yeniden kurulumunda gereksiz yere dolaşılıyor.
                self.destroy_client(client)
            if response is None or not response.values:
                return
            urdf_xml = response.values[0].string_value
            if not urdf_xml:
                return

            tip = measure_from_urdf(urdf_xml, self.end_effector)
        except Exception as exc:  # noqa: BLE001 - doğrulama asla görevi düşürmesin
            self.get_logger().debug(f"Uç eleman doğrulaması yapılamadı: {exc}")
            return

        configured = float(np.linalg.norm(self.tool_tip_offset))
        delta_mm = (configured - tip.distance) * 1000.0
        approach = np.asarray(self.tool_approach_vector, dtype=np.float64)
        approach_len = float(np.linalg.norm(approach)) or 1.0
        cosine = float(np.dot(np.asarray(tip.axis), approach)) / approach_len
        axis_deg = math.degrees(math.acos(max(-1.0, min(1.0, cosine))))

        summary = (
            f"Uç eleman ölçümü ({self.end_effector}): ağız "
            f"{tip.distance * 1000:.1f} mm, yarıçap {tip.ring_radius * 1000:.1f} mm, "
            f"config {configured * 1000:.1f} mm (fark {delta_mm:+.1f} mm), "
            f"eksen farkı {axis_deg:.2f} derece"
        )
        if abs(delta_mm) > 3.0 or axis_deg > 2.0:
            self.get_logger().error(
                summary + " -- UYUŞMUYOR. Kap hedefin "
                + ("ÜSTÜNDE kalır" if delta_mm > 0 else "İÇİNE bastırır")
                + ". Düzeltmek için: ros2 run gemini_robotics_ros measure_tcp"
            )
        else:
            self.get_logger().info(summary + " -- uyuşuyor")

    def _startup_scan_after(self, delay_sec: float) -> None:
        """Gerçek zamanda bekleyip bir kez SCAN pozuna götürür."""
        time.sleep(max(0.0, delay_sec))

        # Bu arada bir komut geldiyse onun SCAN'i zaten bu işi yapıyor; araya
        # girip iki hareketi çakıştırmayalım.
        if not self._busy.acquire(blocking=False):
            self.get_logger().info("Açılış SCAN'i atlandı: görev zaten başlamış.")
            return
        try:
            self._status("STARTUP_SCAN", "Açılış SCAN pozuna gidiliyor")
            self._current_scan_pose = -1
            ok = self._goto_scan_pose(0)
            self._status(
                "READY" if ok else "WARN",
                "SCAN pozunda, komut bekleniyor" if ok
                else "SCAN pozuna gidilemedi - MoveIt hazır mı? "
                     "(ros2 action info /move_action)",
            )
        except Exception as exc:
            self._status("ERROR", f"Açılış SCAN hatası: {exc}")
        finally:
            self._busy.release()

    def _on_command(self, msg: String) -> None:
        command = msg.data.strip()
        if not command:
            return
        if not self._busy.acquire(blocking=False):
            self.get_logger().warn(f"Görev sürüyor, komut atlandı: {command!r}")
            return
        threading.Thread(target=self._run_mission, args=(command,), daemon=True).start()

    def _run_mission(self, command: str) -> None:
        try:
            self._execute(command)
        except Exception as exc:
            self._status("ERROR", f"Görev istisna ile bitti: {exc}")
        finally:
            self._busy.release()

    def _execute(self, command: str) -> None:
        self._status("START", command)

        # Önceki koşudan kalıntı temizliği. İki ayrı yerde kalıntı olabiliyor:
        # MoveIt sahnesinde iliştirilmiş bir kutu (düğüm çökmüşse kalır) ve
        # Gazebo'da kurulu bir DetachableJoint (eklenti başlangıçta bağlı
        # geliyor). İkisi de sessizce yanlış davranış üretir.
        self._detach_payload(force=True)
        if self.vacuum is not None and not self.dry_run:
            self.vacuum.sim_detach()

        # 1) Kamerayı ilk tarama pozuna götür. Bilek kamerası kolla birlikte
        #    hareket ettiği için sorgudan ÖNCE sabit bir bakış pozu şart.
        self._current_scan_pose = -1
        if not self._goto_scan_pose(0):
            self._status("FAILED", "İlk tarama pozuna gidilemedi")
            return

        if not self.locator.has_image():
            self._status("FAILED", "ER görüntüsü üretilemiyor - derinlik akışı var mı?")
            return

        # 2) Komutu ER 2 ile plana çevir
        plan = self.locator.plan(command)
        if not plan or not plan.get("pick"):
            self._status("FAILED", "ER komuttan bir plan çıkaramadı")
            return
        self._status(
            "PLANNED",
            f"pick={plan['pick']!r} place={plan.get('place')!r}",
            reasoning=plan.get("reasoning", ""),
        )

        # 3) Kavranacak yüzeyi bul (konum + normal + düzlemsellik denetimi).
        #    Tarama pozlarında gezilir; bulunan ilk pozda durulur.
        pick_detection = self._find_across_poses(plan["pick"], True, "PICK")
        if pick_detection is None:
            self._status(
                "FAILED",
                f"Kavranabilir hedef hiçbir tarama pozunda bulunamadı: {plan['pick']!r}",
            )
            return

        contact, normal, grasp_quat = self._grasp_frame(pick_detection)
        surface = pick_detection.get("surface")
        self._status(
            "PICK_LOCATED",
            pick_detection["label"],
            position={"x": contact[0], "y": contact[1], "z": contact[2]},
            normal={"x": normal[0], "y": normal[1], "z": normal[2]},
            surface_rms_mm=round(surface["rms_residual"] * 1000, 2) if surface else None,
        )

        # 4) Bırakma hedefi. Konveyör ve toolkit aynı kadraja sığmadığı için
        #    bunu da tarama pozlarında ararız. Bulunamazsa görev BAŞARISIZ:
        #    parçayı alıp havada tutup "tamamlandı" demek yanlış rapor olurdu.
        place_detection = None
        if plan.get("place"):
            place_detection = self._find_across_poses(plan["place"], False, "PLACE")
            if place_detection is None:
                self._status(
                    "FAILED",
                    f"Bırakma hedefi hiçbir tarama pozunda bulunamadı: {plan['place']!r}",
                )
                return
            self._status(
                "PLACE_LOCATED",
                place_detection["label"],
                position=place_detection["position"],
            )

        # 5) Kavrama: normal boyunca yaklaş, kabı yüzeye oturt, vakumu aç
        #    Mesafe hedefe göre: _validate_reachable, çarpışmayan ilk değeri
        #    seçip tespite yazdı.
        pick_approach_distance = float(
            pick_detection.get("approach_distance", self.approach_distance))
        approach = self._offset(contact, normal, pick_approach_distance)
        touch = self._offset(contact, normal, self.touch_offset)

        if not self._move_pose(approach, grasp_quat, "APPROACH_PICK"):
            self._status("FAILED", "Kavrama yaklaşma pozuna gidilemedi")
            return
        if not self._move_pose(touch, grasp_quat, "TOUCH", cartesian=True):
            self._status("FAILED", "Emme kabı yüzeye oturtulamadı")
            return

        if not self._grip():
            self._status("FAILED", "Vakum kurulamadı (parça tutulmadı)")
            # Kabı yüzeyden çek, pompayı kapat, eve dön.
            self._release()
            self._move_pose(approach, grasp_quat, "RETREAT_AFTER_FAIL", cartesian=True)
            self._move_joints(self.home_joints, "HOME")
            return
        self._status("GRASPED", pick_detection["label"])
        # Parça artık robotun bir parçası: hem Gazebo'da (DetachableJoint,
        # vakum komutuyla birlikte) hem MoveIt sahnesinde. Bundan SONRAKİ her
        # plan onu taşıyor ve çarpışma denetimine sokuyor.
        self._attach_payload(pick_detection, contact, grasp_quat)

        # Parçayı yüzeyden NORMAL boyunca çek: dikey bir yüzeyde dünya +Z'ye
        # kaldırmak parçayı yüzeye sürter.
        lift = self._offset(contact, normal, max(self.lift_distance,
                                                 pick_approach_distance))
        if not self._move_pose(lift, grasp_quat, "LIFT", cartesian=True):
            self._status("FAILED", "Kaldırma başarısız")
            return

        # 6) Taşı ve bırak
        if place_detection is not None:
            # for_grasp=False: doğrulama neyi onayladıysa hareket de onu
            # kullanmalı. Aynı bayrak geçilmezse doğrulama yukarıdan inişi
            # onaylar, hareket yandan girmeyi dener - ve sessizce çarpar.
            place_contact, place_normal, place_quat = self._grasp_frame(
                place_detection, for_grasp=False)
            place_approach_distance = float(
                place_detection.get("approach_distance", self.approach_distance))
            above_place = self._offset(
                place_contact, place_normal, place_approach_distance)
            release_pose = self._offset(place_contact, place_normal, self.place_clearance)

            if not self._move_pose(above_place, place_quat, "APPROACH_PLACE"):
                self._status("FAILED", "Bırakma yaklaşma pozuna gidilemedi")
                return

            # Bırakmadan önce inmek İSTEĞE BAĞLI. Raf gözüne inerken kartezyen
            # plan sürekli yarıda kalıyordu (%20, sonra %75) - gripper gövdesi
            # göze sığmıyor. Parça zaten gözün üstünden bırakılabildiği için
            # iniş varsayılan olarak KAPALI; açmak isteyen descend_before_release
            # ile açar.
            descended = False
            if self.descend_before_release:
                descended = self._move_pose(
                    release_pose, place_quat, "RELEASE", cartesian=True
                )
                if not descended:
                    self._status("FAILED", "Bırakma pozuna inilemedi")
                    return
            else:
                self.get_logger().info(
                    "Bırakma inişi atlandı (descend_before_release=false); "
                    "parça yaklaşma pozundan bırakılıyor"
                )

            self._release()
            # SIRA ÖNEMLİ: önce bırak, sonra sahneden düşür. Ters sırada,
            # parça hâlâ uca bağlıyken sahneden silinmiş olurdu ve bırakma
            # anındaki hareketler onu görmezdi.
            self._detach_payload()
            self._status(
                "RELEASED",
                "Parça bırakıldı" + ("" if descended else " (yaklaşma pozundan)"),
            )
            # İnmediysek zaten yaklaşma pozundayız; geri çekilecek bir şey yok.
            if descended:
                self._move_pose(above_place, place_quat, "RETREAT", cartesian=True)

        self._move_joints(self.home_joints, "HOME")
        self._status("DONE", "Görev tamamlandı")


def install_spin_once_guard():
    """Düğümü GLOBAL executor'a kaptırmayan bir rclpy.spin_once kurar.

    SORUN. pymoveit2 beş yerde (moveit2.py:527, 639, 747, 1189, 1281) şunu
    yapıyor:

        while not future.done():
            rclpy.spin_once(self._node, timeout_sec=1.0)

    rclpy.spin_once ise düğümü global bir SingleThreadedExecutor'a EKLER. Bir
    düğüm aynı anda tek bir executor'a ait olabildiği için bu, düğümü bizim
    MultiThreadedExecutor'ımızdan söker - biz tam da o sırada spin ediyorken.

    Bizim executor'ımız o anda wait_for_ready_callbacks içinde waitable'ları
    dolaşıyor olabiliyor; action client'ın wait set indeksleri artık ÖTEKİ
    executor'ın (daha küçük) wait set'ine göre yazılmış oluyor ve şu patlıyor:

        RCLError: Failed to get number of ready entities for action client:
        wait set index for status subscription is out of bounds

    Bu istisna executor.spin()'den dışarı çıkıyor, main yalnızca
    KeyboardInterrupt yakaladığı için finally çalışıp düğümü yok ediyor ve
    rclpy'yi kapatıyor. Ardından görev thread'i bir sonraki spin_once'ta ölü
    context'e çarpıp "AttributeError: __enter__" veriyor, sonra da kapanmış
    publisher'a yazmaya çalışıp "InvalidHandle" veriyor. Süreç exit 1.
    Ölçüldü: 11 Ağu 2026, TOUCH hareketinin planlanması sırasında. MoveIt
    tarafı sağlıklıydı (aynı saniyede kartezyen yolu %100 hesaplamıştı) -
    hata tamamen rclpy sahiplik yarışı.

    ÇÖZÜM. Düğüm ZATEN bir executor'a aitse spin_once onu global executor'a
    hiç vermesin, sadece kısa bir uyku ile beklesin. Callback'leri bizim
    executor'ımız çalıştırmaya devam ettiği için future yine tamamlanır;
    değişen tek şey, düğümün sahibinin hiç el değiştirmemesi.

    Neden pymoveit2'yi düzeltmiyoruz: o fork'u başka paketler de kullanıyor
    (viewpoint_planner, multirobot_viewpoint_planner...). Orada spin_once'ı
    uykuya çevirmek, düğümü kendi executor'ında spin ETMEYEN bir çağırana
    kilitlenme yaşatır. Bu yama yalnızca BU sürecin içinde geçerli.

    Uyarı: bu yama ile plan()/execute() çağrıları, düğümün varsayılan
    (MutuallyExclusive) callback grubunda ÇALIŞMAYAN bir thread'den
    yapılmalıdır - görev akışı zaten kendi thread'inde çalışıyor.
    """
    original_spin_once = rclpy.spin_once

    def guarded_spin_once(node, *, executor=None, timeout_sec=None):
        if executor is None and node.executor is not None:
            # Sahibi var: global executor'a verme, sadece bekle.
            time.sleep(0.02 if timeout_sec is None else min(timeout_sec, 0.02))
            return
        return original_spin_once(node, executor=executor, timeout_sec=timeout_sec)

    rclpy.spin_once = guarded_spin_once
    return original_spin_once


def main():
    # `kill -USR1 <pid>` dumps every thread's Python stack to stderr. py-spy needs
    # ptrace privileges we do not have on this machine; this does not.
    faulthandler.enable()
    if hasattr(faulthandler, "register"):
        faulthandler.register(signal.SIGUSR1, file=sys.stderr, all_threads=True)

    # pymoveit2'yi ilk kez çağırmadan ÖNCE kurulmalı.
    install_spin_once_guard()

    rclpy.init()
    node = GeminiPickPlaceNode()
    executor = MultiThreadedExecutor(4)
    executor.add_node(node)
    node._owner_executor = executor
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception as exc:  # noqa: BLE001
        # Sessizce exit 1 ile ölmesin: executor çöktüğünde log'da görünen tek
        # şey alt thread'lerin ARDIL hatalarıydı (ölü context, InvalidHandle)
        # ve asıl neden kayboluyordu.
        node.get_logger().fatal(f"Executor çöktü, düğüm kapanıyor: {exc!r}")
        raise
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
