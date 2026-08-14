#!/usr/bin/env python3
# ============================================================
# File: double_ros2_km_bridge.py
# Version: 1.1.0 - ROS2 -> MongoDB Atlas bridge (Kafka simdilik KAPALI)
#
#   - MongoDB'ye UR10e + Kawasaki eklem konumlari canli (real-time) yazilir.
#   - Kafka tarafi ENABLE_KAFKA = False ile kapatildi. Tekrar acmak icin
#     tek satiri True yapman yeterli (baglantilar/callback'ler geri gelir).
#
# Belge formati:  { ts, ur10e: [7 sayi], kawasaki: [7 sayi] }
#
# UR10e (7 joint)   -> /dynamic_joint_states topic'inden
# Kawasaki (7 joint) -> /joint_states topic'inden
# Iki topic ayri geldigi icin "latest-of-each" mantigiyla birlestirilir.
# ============================================================

import rclpy
import json
import time
import threading
from datetime import datetime, timezone
from queue import Queue, Empty
from kafka import KafkaProducer, KafkaConsumer
from std_msgs.msg import String
from control_msgs.msg import DynamicJointState
from sensor_msgs.msg import JointState, Image, PointCloud2
from visualization_msgs.msg import InteractiveMarkerUpdate
from moveit_msgs.msg import PlanningScene
from ur_msgs.msg import ToolDataMsg
from geometry_msgs.msg import WrenchStamped, PoseStamped
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from control_msgs.msg import JointTrajectoryControllerState

from pymongo import MongoClient

# ------------------------------------------------------------
# KAFKA ANAHTARI
#   False -> Kafka tamamen kapali, sadece MongoDB calisir.
#   True  -> Eski Kafka davranisi geri gelir (tum abonelikler + producer).
# ------------------------------------------------------------
ENABLE_KAFKA = False

# ------------------------------------------------------------
# MongoDB baglanti bilgileri  (DOGRUDAN KODA GOMULU)
#   >>> Sadece 'cluster0.xxxx.mongodb.net' kismini Atlas'taki gercek
#       cluster adresinle degistir. Kullanici/sifre: cem / matisse
# ------------------------------------------------------------
MONGO_URI  = "mongodb+srv://ifarlabmatisse_db_user:2WcRMeMyCCuZfcPl@cem.dphrcjw.mongodb.net/?"
MONGO_DB   = "ifarlabmatisse_db_user"
MONGO_COLL = "joint_states"

# Yazma hizi (Hz). Arayuz "en son kaydi" okudugu icin yuksek olmasi gerekmez.
MONGO_RATE_HZ = 20.0

# "upsert" -> tek dokumani surekli gunceller (M0 512MB icin ONERILEN, depolama ~0)
# "insert" -> her karede yeni dokuman ekler (kisa gecmis tamponu; TTL ile eskir)
MONGO_MODE = "upsert"

# insert modunda dokumanlarin kac saniye sonra otomatik silinecegi (0 = kapali)
MONGO_TTL_SECONDS = 3600

# Belgedeki sabit joint sirasi (arayuz bu siraya gore okuyacak)
UR10E_JOINTS = [
    "ur10e_base_to_robot_mount",   # metre (lineer eksen)
    "ur10e_shoulder_pan_joint",    # radyan
    "ur10e_shoulder_lift_joint",
    "ur10e_elbow_joint",
    "ur10e_wrist_1_joint",
    "ur10e_wrist_2_joint",
    "ur10e_wrist_3_joint",
]
KAWASAKI_JOINTS = [
    "world_to_agv",                # metre (agv konumu)
    "joint1",                      # radyan
    "joint2",
    "joint3",
    "joint4",
    "joint5",
    "joint6",
]

# ------------------------------------------------------------
# QoS setup
# ------------------------------------------------------------
qos = QoSProfile(depth=10)
qos.reliability = QoSReliabilityPolicy.RELIABLE


class double_ros2_km_bridge:
    def __init__(self, args=None):
        rclpy.init(args=args)
        self.node = rclpy.create_node("double_ros2_km_bridge")

        # ------------------------------------------------------------
        # MongoDB icin gerekli TEK abonelik.
        #   /joint_states 14 eklemin HEPSINI tasir (2 yayinci: UR10e ve
        #   Kawasaki mesajlari donusumlu gelir). Eklemler isme gore ayiklandigi
        #   icin tek bu topic yeterli; /dynamic_joint_states'e gerek yok.
        # ------------------------------------------------------------
        self.node.create_subscription(JointState, '/joint_states', self.joint_states_real_callback, qos)

        # ------------------------------------------------------------
        # Kafka'ya ozel diger abonelikler (sadece ENABLE_KAFKA=True iken)
        #   (/dynamic_joint_states eski Kafka koprusunun UR10e kaynagiydi;
        #    Mongo icin kullanilmiyor, sadece Kafka acilinca devreye girer.)
        # ------------------------------------------------------------
        if ENABLE_KAFKA:
            self.node.create_subscription(DynamicJointState, '/dynamic_joint_states', self.joint_states_callback, qos)
            self.node.create_subscription(Image, '/sim/image', self.sim_image_callback, qos)
            self.node.create_subscription(PointCloud2, '/sim/pointcloud', self.sim_point_cloud_callback, qos)
            self.node.create_subscription(JointState, '/sim/joint_states', self.sim_joint_states_callback, qos)
            self.node.create_subscription(PlanningScene, '/monitored_planning_scene', self.monitored_planning_scene_callback, qos)
            self.node.create_subscription(InteractiveMarkerUpdate, '/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/update', self.interactive_marker_update_callback, qos)
            self.node.create_subscription(ToolDataMsg, '/io_and_status_controller/tool_data', self.tool_data_callback, qos)
            self.node.create_subscription(WrenchStamped, '/force_torque_sensor_broadcaster/wrench', self.force_torque_callback, qos)
            self.node.create_subscription(PoseStamped, '/tcp_pose_broadcaster/pose', self.tcp_pose_callback, qos)
            self.node.create_subscription(JointTrajectoryControllerState, 'scaled_joint_trajectory_controller/controller_state', self.controller_state_callback, qos)

        # ------------------------------------------------------------
        # Kafka Producer Setup (sadece ENABLE_KAFKA=True iken)
        # ------------------------------------------------------------
        if ENABLE_KAFKA:
            self.kafka_queue = Queue(maxsize=100000)
            self.producer = KafkaProducer(
                bootstrap_servers='localhost:9092',
                client_id='ros2_km_bridge_realtime',
                acks=0,                      # no wait for broker ack
                linger_ms=0,                 # send immediately
                batch_size=4096,             # small batch size (~4KB)
                buffer_memory=33554432,      # 32MB buffer
                max_in_flight_requests_per_connection=1,
                value_serializer=lambda m: json.dumps(m).encode('utf-8')
            )

            if self.producer.bootstrap_connected():
                self.node.get_logger().info("✅ Kafka producer connected (real-time mode).")
            else:
                self.node.get_logger().warn("⚠️ Kafka producer not connected! Check localhost:9092")

            # Start worker and auto flusher
            self.start_kafka_worker()
            self.start_kafka_auto_flusher()
        else:
            self.node.get_logger().info("ℹ️ Kafka KAPALI (ENABLE_KAFKA=False) - sadece MongoDB akisi calisiyor.")

        # ------------------------------------------------------------
        # MongoDB Setup (canli birlesik joint akisi)
        # ------------------------------------------------------------
        self.latest = {name: None for name in (UR10E_JOINTS + KAWASAKI_JOINTS)}
        self.latest_lock = threading.Lock()
        self.collection = None
        self.setup_mongo()
        if self.collection is not None:
            self.start_mongo_worker()

        # ------------------------------------------------------------
        # Kafka -> ROS Setup (sadece ENABLE_KAFKA=True iken)
        # ------------------------------------------------------------
        if ENABLE_KAFKA:
            self.consumer1 = KafkaConsumer(
                'kafka_to_bridge_topic_1',
                bootstrap_servers='localhost:9092',
                value_deserializer=lambda m: json.loads(m.decode('utf-8')),
                auto_offset_reset='latest',
                enable_auto_commit=True,
                group_id='ros2_km_bridge_realtime'
            )
            self.pub1 = self.node.create_publisher(String, 'bridge_to_ros2_topic_1', qos)
            threading.Thread(target=self.kafka_reader, args=(self.consumer1, self.pub1), daemon=True).start()

        self.node.get_logger().info("🚀 Real-time ROS2 -> MongoDB bridge running...")
        rclpy.spin(self.node)
        rclpy.shutdown()

    # ============================================================
    # MongoDB
    # ============================================================
    def setup_mongo(self):
        try:
            client = MongoClient(MONGO_URI, serverSelectionTimeoutMS=5000)
            client.admin.command('ping')  # baglantiyi dogrula
            self.collection = client[MONGO_DB][MONGO_COLL]
            # insert modunda otomatik eskime icin TTL index
            if MONGO_MODE == "insert" and MONGO_TTL_SECONDS > 0:
                try:
                    self.collection.create_index("ts", expireAfterSeconds=MONGO_TTL_SECONDS)
                except Exception as e:
                    self.node.get_logger().warn(f"⚠️ TTL index olusturulamadi: {e}")
            self.node.get_logger().info(
                f"✅ MongoDB baglantisi basarili -> {MONGO_DB}.{MONGO_COLL} "
                f"(mode={MONGO_MODE}, rate={MONGO_RATE_HZ}Hz)")
        except Exception as e:
            self.collection = None
            self.node.get_logger().error(f"❌ MongoDB baglantisi basarisiz: {e}")

    def update_latest(self, positions):
        """positions: {joint_name: position_float} - hedef joint'leri gunceller."""
        if not positions:
            return
        with self.latest_lock:
            for name in self.latest:
                if name in positions and positions[name] is not None:
                    self.latest[name] = positions[name]

    def start_mongo_worker(self):
        """
        Her zaman EN GUNCEL kareyi yazar (drop-to-latest).
        ROS callback'leri sadece self.latest'i gunceller; bu thread her
        yazimda o anki son degeri alir -> ara kareler atlanir, gecikme birikmez.

        Not: Onceki surumde 20Hz ureten bir yayinci + FIFO kuyruk vardi. Bulut
        Atlas'a yazim ~150ms surdugu icin kuyruk boşalamiyor, eski kareler
        sirayla yazilip veri dakikalarca geriye dusuyordu. Kuyrugu kaldirdik.
        """
        period = 1.0 / MONGO_RATE_HZ if MONGO_RATE_HZ > 0 else 0.05

        def worker():
            count = 0
            while True:
                t0 = time.time()
                with self.latest_lock:
                    ur10e = [self.latest[n] for n in UR10E_JOINTS]
                    kawa  = [self.latest[n] for n in KAWASAKI_JOINTS]

                # Henuz hicbir robottan veri gelmediyse bekle
                if all(v is None for v in ur10e) and all(v is None for v in kawa):
                    time.sleep(period)
                    continue

                doc = {
                    "ts": datetime.now(timezone.utc),   # yazim aninda damgala
                    "ur10e": ur10e,
                    "kawasaki": kawa,
                }
                try:
                    if MONGO_MODE == "insert":
                        self.collection.insert_one(doc)
                    else:  # upsert: tek dokumani surekli guncelle (depolama ~0)
                        self.collection.replace_one({"_id": "latest"},
                                                    {**doc, "_id": "latest"},
                                                    upsert=True)
                    count += 1
                    if count % 200 == 0:
                        self.node.get_logger().info(f"🍃 MongoDB: {count} kayit yazildi.")
                except Exception as e:
                    self.node.get_logger().warn(f"⚠️ MongoDB yazma hatasi: {e}")
                    time.sleep(0.5)

                # Hiz siniri: yazim zaten period'dan uzun surdiyse ek bekleme yok
                dt = time.time() - t0
                if dt < period:
                    time.sleep(period - dt)

        threading.Thread(target=worker, daemon=True).start()

    # ============================================================
    # Kafka Worker Thread'leri (sadece ENABLE_KAFKA=True iken cagrilir)
    # ============================================================
    def start_kafka_worker(self):
        def worker():
            send_count = 0
            while True:
                try:
                    topic, data = self.kafka_queue.get(timeout=1)
                    self.producer.send(topic, data)
                    send_count += 1
                    if send_count % 10 == 0:
                        self.producer.flush()  # frequent flush for near real-time
                except Empty:
                    continue
                except Exception as e:
                    print(f"[Kafka Worker Error] {e}")
        threading.Thread(target=worker, daemon=True).start()

    def start_kafka_auto_flusher(self):
        """Flush Kafka producer buffer periodically (every 0.5s)"""
        def flusher():
            while True:
                time.sleep(0.5)
                try:
                    self.producer.flush()
                except Exception as e:
                    print(f"[Kafka Flusher Error] {e}")
        threading.Thread(target=flusher, daemon=True).start()

    def safe_publish(self, topic, data):
        """Non-blocking enqueue to Kafka (sadece ENABLE_KAFKA=True iken anlamli)"""
        if not ENABLE_KAFKA:
            return
        try:
            self.kafka_queue.put_nowait((topic, data))
        except:
            print(f"⚠️ Kafka queue full, dropping message for {topic}")

    def kafka_reader(self, consumer, publisher):
        for message in consumer:
            msg = String()
            msg.data = message.value.get("data", "")
            publisher.publish(msg)
            print(f"⬅️ Kafka -> ROS [{message.topic}]")

    # ------------------------------------------------------------
    # ROS Callbacks (MongoDB icin kritik olan iki tanesi)
    # ------------------------------------------------------------
    def joint_states_callback(self, msg):
        data = {
            joint_name: {
                iface_name: iface_value
                for iface_name, iface_value in zip(joint_interfaces.interface_names, joint_interfaces.values)
            }
            for joint_name, joint_interfaces in zip(msg.joint_names, msg.interface_values)
        }
        data["header.stamp.sec"] = msg.header.stamp.sec
        data["header.stamp.nanosec"] = msg.header.stamp.nanosec
        self.safe_publish('dynamic_joint_states_topic', data)

        # --- MongoDB: hedef joint'lerin 'position' degerlerini yakala ---
        positions = {
            name: vals.get("position")
            for name, vals in data.items()
            if isinstance(vals, dict) and "position" in vals
        }
        self.update_latest(positions)

    def joint_states_real_callback(self, msg):
        current_time = self.node.get_clock().now()

        data = {
            name: {
                'position': msg.position[i] if i < len(msg.position) else None,
                'velocity': msg.velocity[i] if i < len(msg.velocity) else None,
                'effort': msg.effort[i] if i < len(msg.effort) else None
            }
            for i, name in enumerate(msg.name)
        }
        data["header.stamp.sec"] = msg.header.stamp.sec
        data["header.stamp.nanosec"] = msg.header.stamp.nanosec
        data["ros_time.sec"] = current_time.seconds_nanoseconds()[0]
        data["ros_time.nanosec"] = current_time.seconds_nanoseconds()[1]

        self.safe_publish('joint_states_topic', data)

        # --- MongoDB: Kawasaki joint1-6 + world_to_agv burada gelir ---
        positions = {
            name: msg.position[i]
            for i, name in enumerate(msg.name)
            if i < len(msg.position)
        }
        self.update_latest(positions)

    # ------------------------------------------------------------
    # Asagidaki callback'ler sadece ENABLE_KAFKA=True iken abone edilir.
    # (Kapaliyken cagrilmazlar; ileride Kafka'yi acinca geri devreye girerler.)
    # ------------------------------------------------------------
    def sim_joint_states_callback(self, msg):
        current_time = self.node.get_clock().now()

        data = {
            name: {
                'position': msg.position[i] if i < len(msg.position) else None,
                'velocity': msg.velocity[i] if i < len(msg.velocity) else None,
                'effort': msg.effort[i] if i < len(msg.effort) else None
            }
            for i, name in enumerate(msg.name)
        }
        data["sim_time.sec"] = msg.header.stamp.sec
        data["sim_time.nanosec"] = msg.header.stamp.nanosec
        data["header.stamp.sec"] = current_time.seconds_nanoseconds()[0]
        data["header.stamp.nanosec"] = current_time.seconds_nanoseconds()[1]

        self.safe_publish('sim_joint_states_topic', data)

    def sim_image_callback(self, msg):
        import base64
        import cv2
        import numpy as np

        current_time = self.node.get_clock().now()
        print(f"🔍 DEBUG: Image encoding = {msg.encoding}, size = {len(msg.data)}, width={msg.width}, height={msg.height}")

        try:
            if msg.encoding == '32FC1':
                depth_array = np.frombuffer(msg.data, dtype=np.float32)
                depth_image = depth_array.reshape((msg.height, msg.width))
                depth_image = np.nan_to_num(depth_image, nan=0.0, posinf=0.0, neginf=0.0)

                if depth_image.max() > depth_image.min():
                    normalized = ((depth_image - depth_image.min()) /
                                (depth_image.max() - depth_image.min()) * 255).astype(np.uint8)
                else:
                    normalized = np.zeros_like(depth_image, dtype=np.uint8)

            elif msg.encoding == 'rgb8':
                img_array = np.frombuffer(msg.data, dtype=np.uint8)
                img = img_array.reshape((msg.height, msg.width, 3))
                normalized = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

            elif msg.encoding == 'bgr8':
                img_array = np.frombuffer(msg.data, dtype=np.uint8)
                normalized = img_array.reshape((msg.height, msg.width, 3))

            elif msg.encoding == 'mono8':
                img_array = np.frombuffer(msg.data, dtype=np.uint8)
                normalized = img_array.reshape((msg.height, msg.width))

            else:
                raise ValueError(f"Unsupported encoding: {msg.encoding}")

            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 80]
            result, encoded_img = cv2.imencode('.jpg', normalized, encode_param)

            if result:
                jpg_base64 = base64.b64encode(encoded_img.tobytes()).decode('utf-8')

                data = {
                    "sim_time": {"sec": msg.header.stamp.sec, "nanosec": msg.header.stamp.nanosec},
                    "header": {
                        "stamp": {
                            "sec": current_time.seconds_nanoseconds()[0],
                            "nanosec": current_time.seconds_nanoseconds()[1]
                        },
                        "frame_id": msg.header.frame_id
                    },
                    "height": msg.height,
                    "width": msg.width,
                    "encoding": msg.encoding,
                    "original_data_size": len(msg.data),
                    "compressed_size": len(encoded_img),
                    "compression_ratio": round(len(msg.data) / len(encoded_img), 2),
                    "image_jpeg_base64": jpg_base64
                }
                self.safe_publish('sim_image_topic', data)
            else:
                raise Exception("JPEG encoding failed")

        except Exception as e:
            import traceback
            error_details = traceback.format_exc()
            self.node.get_logger().error(f"Image processing error: {e}\n{error_details}")

    def sim_point_cloud_callback(self, msg):
        current_time = self.node.get_clock().now()
        data = {
            "sim_time": {"sec": msg.header.stamp.sec, "nanosec": msg.header.stamp.nanosec},
            "header": {
                "stamp": {
                    "sec": current_time.seconds_nanoseconds()[0],
                    "nanosec": current_time.seconds_nanoseconds()[1]
                }
            },
            "height": msg.height,
            "width": msg.width,
            "point_step": msg.point_step,
            "is_dense": msg.is_dense,
            "total_points": msg.height * msg.width
        }
        self.safe_publish('sim_point_cloud_topic', data)

    def monitored_planning_scene_callback(self, msg):
        data = {
            "name": msg.name,
            "robot_model_name": msg.robot_model_name,
            "is_diff": msg.is_diff,
            "header": {"sec": msg.robot_state.joint_state.header.stamp.sec,
                       "nanosec": msg.robot_state.joint_state.header.stamp.nanosec}
        }
        self.safe_publish('monitored_planning_scene_topic', data)

    def interactive_marker_update_callback(self, msg):
        data = {"server_id": msg.server_id, "seq_num": msg.seq_num, "type": msg.type}
        self.safe_publish('interactive_marker_update_topic', data)

    def tool_data_callback(self, msg):
        data = {
            "analog_input2": msg.analog_input2,
            "tool_current": msg.tool_current,
            "tool_temperature": msg.tool_temperature,
            "tool_mode": msg.tool_mode
        }
        self.safe_publish('tool_data_topic', data)

    def force_torque_callback(self, msg):
        data = {
            "force": {"x": msg.wrench.force.x, "y": msg.wrench.force.y, "z": msg.wrench.force.z},
            "torque": {"x": msg.wrench.torque.x, "y": msg.wrench.torque.y, "z": msg.wrench.torque.z},
            "header": {"sec": msg.header.stamp.sec, "nanosec": msg.header.stamp.nanosec}
        }
        self.safe_publish('force_torque_sensor_topic', data)

    def tcp_pose_callback(self, msg):
        data = {
            "pose": {
                "position": {"x": msg.pose.position.x, "y": msg.pose.position.y, "z": msg.pose.position.z},
                "orientation": {
                    "x": msg.pose.orientation.x,
                    "y": msg.pose.orientation.y,
                    "z": msg.pose.orientation.z,
                    "w": msg.pose.orientation.w
                }
            },
            "header": {"sec": msg.header.stamp.sec, "nanosec": msg.header.stamp.nanosec}
        }
        self.safe_publish('tcp_pose_topic', data)

    def serialize_trajectory_point(self, traj_point):
        """JointTrajectoryPoint nesnesini dict'e çevirir"""
        return {
            "positions": list(traj_point.positions) if traj_point.positions else [],
            "velocities": list(traj_point.velocities) if traj_point.velocities else [],
            "accelerations": list(traj_point.accelerations) if traj_point.accelerations else [],
            "effort": list(traj_point.effort) if traj_point.effort else [],
            "time_from_start": {
                "sec": traj_point.time_from_start.sec,
                "nanosec": traj_point.time_from_start.nanosec
            }
        }

    def controller_state_callback(self, msg):
        current_time = self.node.get_clock().now()
        data = {
            "header": {
                "stamp": {
                    "sec": current_time.seconds_nanoseconds()[0],
                    "nanosec": current_time.seconds_nanoseconds()[1]
                },
                "frame_id": msg.header.frame_id
            },
            "original_timestamp": {"sec": msg.header.stamp.sec, "nanosec": msg.header.stamp.nanosec},
            "joint_names": list(msg.joint_names),
            "reference": self.serialize_trajectory_point(msg.reference),
            "feedback": self.serialize_trajectory_point(msg.feedback),
            "error": self.serialize_trajectory_point(msg.error),
            "output": self.serialize_trajectory_point(msg.output),
            "desired": self.serialize_trajectory_point(msg.desired),
            "actual": self.serialize_trajectory_point(msg.actual),
        }
        self.safe_publish('controller_state_topic', data)


# ------------------------------------------------------------
# Main Entry
# ------------------------------------------------------------
if __name__ == '__main__':
    double_ros2_km_bridge()
