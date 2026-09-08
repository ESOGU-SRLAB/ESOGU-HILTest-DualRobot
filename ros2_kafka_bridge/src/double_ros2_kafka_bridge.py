#!/usr/bin/env python3
# ============================================================
# File: double_ros2_kafka_bridge_real_time.py
# Version: 3.2.0 - True real-time Kafka + ROS2 bridge
# ============================================================

import rclpy
import json
import time
import threading
from queue import Queue, Empty, Full
from kafka import KafkaProducer, KafkaConsumer
from std_msgs.msg import String
from control_msgs.msg import DynamicJointState
from sensor_msgs.msg import JointState, Image, PointCloud2
from visualization_msgs.msg import InteractiveMarkerUpdate
from moveit_msgs.msg import PlanningScene
from ur_msgs.msg import ToolDataMsg
from geometry_msgs.msg import WrenchStamped, PoseStamped
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from control_msgs.msg import JointTrajectoryControllerState

# ------------------------------------------------------------
# QoS setup
# ------------------------------------------------------------
qos = QoSProfile(depth=10)
qos.reliability = QoSReliabilityPolicy.RELIABLE

# ------------------------------------------------------------
# Use-case tagging
# ------------------------------------------------------------
# The dashboard (user_interface/app.py) publishes the name of the scenario that
# is currently running on a LATCHED topic. Every document this bridge forwards
# to Kafka is stamped with it, so the collected data can be filtered by use case
# downstream (Elasticsearch / the Data Analytics tab).
#
# TRANSIENT_LOCAL is mandatory on BOTH ends. This bridge is long-lived and may be
# restarted at any point during a run; with the default VOLATILE durability it
# would never receive a value published before it subscribed and would tag the
# whole run as IDLE.
USE_CASE_TOPIC = "/testbed/use_case"
USE_CASE_IDLE = "IDLE"
USE_CASE_FIELD = "use_case"

# The run id identifies ONE EXECUTION of a scenario, where the use case only
# names the scenario. It arrives on its own latched topic, so this bridge can
# adopt it without the /testbed/use_case payload changing shape for anyone else
# still reading that topic as a bare string.
#
# Empty means "no run in progress" and is left off the document entirely rather
# than stored as "": a missing field aggregates cleanly as missing, whereas an
# empty string becomes a real term that shows up in every run listing.
RUN_ID_TOPIC = "/testbed/run_id"
RUN_ID_FIELD = "run_id"

latched_qos = QoSProfile(depth=1)
latched_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
latched_qos.reliability = QoSReliabilityPolicy.RELIABLE


class double_ros2_kafka_bridge:
    def __init__(self, args=None):
        rclpy.init(args=args)
        self.node = rclpy.create_node("double_ros2_kafka_bridge_real_time")

        # ------------------------------------------------------------
        # Active use case (latched) — stamped onto every outgoing document
        # ------------------------------------------------------------
        self.use_case = USE_CASE_IDLE
        self.run_id = ""

        # Dropped-message accounting for safe_publish(). Losing data silently is
        # how the 89% loss went unnoticed; these make it a visible rate.
        self.dropped_total = 0
        self.dropped_at_last_log = 0
        self.last_drop_log = 0.0

        # Health-report counters, drained once a minute by start_health_reporter().
        # Written from the worker thread and read from the reporter thread without
        # a lock on purpose: a lock on this path would cost more than the numbers
        # are worth, and a stats line that is off by a message or two is still
        # telling the truth about whether the bridge is keeping up.
        self.sent_counts = {}
        self.sent_total = 0
        self.lat_sum = 0.0
        self.lat_max = 0.0
        self.send_errors = 0
        self.errors_at_last_log = 0
        self.last_send_error = ""
        self.node.create_subscription(String, USE_CASE_TOPIC, self.use_case_callback, latched_qos)
        self.node.create_subscription(String, RUN_ID_TOPIC, self.run_id_callback, latched_qos)

        # ------------------------------------------------------------
        # ROS → Kafka Subscriptions
        # ------------------------------------------------------------
        self.node.create_subscription(DynamicJointState, '/dynamic_joint_states', self.joint_states_callback, qos)
        self.node.create_subscription(DynamicJointState, '/kawasaki/dynamic_joint_states', self.kawasaki_joint_states_callback, qos)
        self.node.create_subscription(Image, '/sim/image', self.sim_image_callback, qos)
        self.node.create_subscription(PointCloud2, '/sim/pointcloud', self.sim_point_cloud_callback, qos)
        self.node.create_subscription(JointState, '/sim/joint_states', self.sim_joint_states_callback, qos)
        self.node.create_subscription(PlanningScene, '/monitored_planning_scene', self.monitored_planning_scene_callback, qos)
        self.node.create_subscription(InteractiveMarkerUpdate, '/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/update', self.interactive_marker_update_callback, qos)
        self.node.create_subscription(ToolDataMsg, '/io_and_status_controller/tool_data', self.tool_data_callback, qos)
        self.node.create_subscription(WrenchStamped, '/force_torque_sensor_broadcaster/wrench', self.force_torque_callback, qos)
        self.node.create_subscription(PoseStamped, '/tcp_pose_broadcaster/pose', self.tcp_pose_callback, qos)
        self.node.create_subscription(JointTrajectoryControllerState,'scaled_joint_trajectory_controller/controller_state',self.controller_state_callback,qos)
        self.node.create_subscription(JointState, '/joint_states', self.joint_states_real_callback, qos)

        # ------------------------------------------------------------
        # Kafka Producer Setup (Real-time optimized)
        # ------------------------------------------------------------s
        self.kafka_queue = Queue(maxsize=100000)
        # MEASURED 4 Sep 2026 on the live cell: with the previous settings
        # (linger_ms=0, batch_size=4096, max_in_flight=1) the bridge delivered
        # 53 Hz out of a 500 Hz source — 175 of 178 sampled seconds were
        # completely silent, then a 7000 msg/s burst. ~89% of the cell's data
        # never reached Kafka. The numbers below are chosen against the real
        # message sizes, so batching can actually happen:
        #
        #   batch_size    a dynamic_joint_states message measures 3.59 KB, so
        #                 the old 4096 fit ONE message per batch — every record
        #                 became its own request. 128 KB holds ~35 of them.
        #   linger_ms     0 meant "never wait", which defeats batching outright.
        #                 5 ms costs nothing perceptible at 500 Hz and lets a
        #                 batch actually fill.
        #   max_in_flight 1 forced strict serialisation: one stalled request
        #                 stopped the whole bridge. 5 is the library default.
        #   compression   joint-state JSON is highly repetitive; lz4 is cheap
        #                 enough to run on the producer thread.
        self.producer = KafkaProducer(
            bootstrap_servers='localhost:9092',
            client_id='ros2_kafka_bridge_realtime',
            # acks=1, NOT 0. MEASURED 4 Sep 2026 against this cell's broker
            # (Kafka 4.0 KRaft + kafka-python 2.2.3): with acks=0 the producer
            # accepts every record and delivers NONE of them — 0 of 1500 landed,
            # four runs in a row, with and without compression, while acks=1
            # landed 1500 of 1500 every time. Because acks=0 asks for no broker
            # response, nothing raises and no counter notices: send() returns
            # happily and the data is simply gone. That is what produced months
            # of "the bridge is publishing but Elasticsearch is empty".
            # Do not set this back to 0 to chase throughput; acks=1 measured
            # FASTER here (0.25 s vs 0.36 s for the same 2000 records).
            acks=1,
            linger_ms=5,                 # let a batch fill before sending
            batch_size=131072,           # 128KB — ~35 joint-state messages
            buffer_memory=67108864,      # 64MB buffer
            max_in_flight_requests_per_connection=5,
            compression_type='lz4',
            value_serializer=lambda m: json.dumps(m).encode('utf-8')
        )

        if self.producer.bootstrap_connected():
            self.node.get_logger().info("✅ Kafka producer connected (real-time mode).")
        else:
            self.node.get_logger().warn("⚠️ Kafka producer not connected! Check localhost:9092")

        # Start worker, auto flusher and the once-a-minute health report
        self.start_kafka_worker()
        self.start_kafka_auto_flusher()
        self.start_health_reporter()

        # ------------------------------------------------------------
        # Kafka → ROS Setup (optional)
        # ------------------------------------------------------------
        self.consumer1 = KafkaConsumer(
            'kafka_to_bridge_topic_1',
            bootstrap_servers='localhost:9092',
            value_deserializer=lambda m: json.loads(m.decode('utf-8')),
            auto_offset_reset='latest',
            enable_auto_commit=True,
            group_id='ros2_bridge_realtime'
        )

        self.pub1 = self.node.create_publisher(String, 'bridge_to_ros2_topic_1', qos)
        threading.Thread(target=self.kafka_reader, args=(self.consumer1, self.pub1), daemon=True).start()

        self.node.get_logger().info("🚀 Real-time ROS2 ↔ Kafka bridge running...")
        rclpy.spin(self.node)
        rclpy.shutdown()

    # ------------------------------------------------------------
    # Kafka Worker Thread: Sends queued messages immediately
    # ------------------------------------------------------------
    def start_kafka_worker(self):
        def worker():
            # NO flush() here. py-spy caught this thread parked in
            # producer.flush() -> await_flush_completion(): at ~2500 msg/s the
            # old "flush every 10 messages" put a full-buffer barrier in the
            # path 250 times a second and was the direct cause of the 49-126
            # second stalls. start_kafka_auto_flusher() already flushes every
            # 0.5 s, which is what bounds latency when the cell goes quiet.
            while True:
                try:
                    topic, data, t_enq = self.kafka_queue.get(timeout=1)
                    # add_errback takes a bound method, not a lambda: this runs
                    # ~2500 times a second and a fresh closure per message would
                    # be pure garbage. Counting FAILED deliveries is what makes
                    # the minute report honest — the counter below still counts
                    # sends, and a send is only a handoff to the producer.
                    self.producer.send(topic, data).add_errback(self._on_send_error)
                    # How long this message sat in the queue. This is THE
                    # real-time number: near zero means the bridge is keeping
                    # up, a growing value means Kafka is not draining as fast
                    # as the cell produces.
                    lat = time.monotonic() - t_enq
                    self.sent_counts[topic] = self.sent_counts.get(topic, 0) + 1
                    self.sent_total += 1
                    self.lat_sum += lat
                    if lat > self.lat_max:
                        self.lat_max = lat
                except Empty:
                    continue
                except Exception as e:
                    self.node.get_logger().error(f"Kafka worker hatasi: {e}")
        threading.Thread(target=worker, daemon=True).start()

    def start_kafka_auto_flusher(self):
        """Flush Kafka producer buffer periodically (every 0.5s)"""
        def flusher():
            while True:
                time.sleep(0.5)
                try:
                    self.producer.flush()
                except Exception as e:
                    self.node.get_logger().error(f"Kafka flusher hatasi: {e}")
        threading.Thread(target=flusher, daemon=True).start()

    def _on_send_error(self, exc):
        """Delivery failed at the broker. Counted, never logged from here —
        this runs on the producer's network thread, once per failed record."""
        self.send_errors += 1
        self.last_send_error = repr(exc)[:160]

    # ------------------------------------------------------------
    # Health Reporter Thread: one line per minute, nothing per message
    # ------------------------------------------------------------
    def start_health_reporter(self, period=60.0):
        """Report once a minute whether the bridge is actually keeping up.

        Replaces the 16 per-message prints this file used to make. Those made the
        launch terminal unreadable and, worse, hid the real failure: on 4 Sep 2026
        the bridge was silently dropping ~89% of the cell's data behind a wall of
        "Published [...]" lines.

        The four numbers that matter, in order of how quickly they tell you
        something is wrong:

          dusen     messages that never made it into the queue at all. Anything
                    above zero is data loss, and the line becomes a WARNING.
          bekleme   how long a message waited in the queue before being handed to
                    Kafka. This is the "is it real time?" number — single-digit
                    milliseconds is healthy, a climbing peak means Kafka is not
                    draining fast enough.
          kuyruk    queue depth at the instant of the report. Steady near zero is
                    healthy; steadily rising means the same thing as above.
          Hz        per-topic throughput, so a single dead subscription is
                    obvious next to the ones that are still flowing.

        A window with no traffic at all logs its own explicit line: silence used
        to be indistinguishable from a wedged bridge.
        """
        def reporter():
            while True:
                time.sleep(period)

                # Snapshot and reset. Reading then zeroing can lose the handful of
                # messages the worker writes in between; that is acceptable for a
                # stats line and keeps this off the hot path.
                counts, self.sent_counts = self.sent_counts, {}
                total, self.sent_total = self.sent_total, 0
                lat_sum, self.lat_sum = self.lat_sum, 0.0
                lat_max, self.lat_max = self.lat_max, 0.0
                dropped = self.dropped_total - self.dropped_at_last_log
                self.dropped_at_last_log = self.dropped_total
                failed = self.send_errors - self.errors_at_last_log
                self.errors_at_last_log = self.send_errors

                depth = self.kafka_queue.qsize()
                log = self.node.get_logger()

                if total == 0 and dropped == 0:
                    log.warn(
                        f"📊 {period:.0f} sn: Kafka'ya HIC mesaj gitmedi "
                        f"(kuyruk {depth}, use_case={self.use_case}). "
                        f"Hucre bosta degilse kopru tikanmis demektir.")
                    continue

                avg_ms = (lat_sum / total * 1000.0) if total else 0.0
                head = (f"📊 {period:.0f} sn: {total} mesaj ({total / period:.0f} Hz) | "
                        f"kuyruk {depth} | bekleme ort {avg_ms:.1f} ms / tepe "
                        f"{lat_max * 1000.0:.0f} ms | dusen {dropped} | "
                        f"teslim edilemeyen {failed}")
                detail = "   " + " · ".join(
                    f"{t.replace('_topic', '')} {c / period:.0f} Hz"
                    for t, c in sorted(counts.items(), key=lambda kv: -kv[1]))

                if dropped or failed:
                    note = f"  ⚠️ VERI KAYBI (kuyruk {self.dropped_total}, "
                    note += f"teslim {self.send_errors}"
                    if self.last_send_error:
                        note += f": {self.last_send_error}"
                    log.warn(head + note + ")")
                    log.warn(detail)
                else:
                    log.info(head)
                    log.info(detail)
        threading.Thread(target=reporter, daemon=True).start()

    def use_case_callback(self, msg):
        """Adopt the use case the dashboard is broadcasting."""
        name = (msg.data or "").strip() or USE_CASE_IDLE
        if name != self.use_case:
            self.use_case = name
            self.node.get_logger().info(f"🏷️  USE_CASE = {name}")

    def run_id_callback(self, msg):
        """Adopt the run id the dashboard is broadcasting."""
        run_id = (msg.data or "").strip()
        if run_id != self.run_id:
            self.run_id = run_id
            self.node.get_logger().info(f"🔖 RUN_ID = {run_id or '-'}")

    def safe_publish(self, topic, data):
        """Non-blocking enqueue to Kafka.

        The use-case and run-id stamps are applied HERE rather than in each
        callback: this is the single choke point every topic passes through, so
        two lines tag joint states, TCP poses, wrenches, controller state and
        images alike. The dicts are built fresh in each callback, so mutating is
        safe.
        """
        data[USE_CASE_FIELD] = self.use_case
        if self.run_id:
            data[RUN_ID_FIELD] = self.run_id
        try:
            self.kafka_queue.put_nowait((topic, data, time.monotonic()))
        except Full:
            # Counted only — never logged from here. When the queue backs up this
            # path runs ~2500 times a second, so any logging in it would both burn
            # a core and flood the terminal. The minute report turns the count into
            # a rate and raises itself to a warning when it is non-zero.
            self.dropped_total += 1

    # ------------------------------------------------------------
    # Kafka → ROS Reader
    # ------------------------------------------------------------
    def kafka_reader(self, consumer, publisher):
        for message in consumer:
            msg = String()
            msg.data = message.value.get("data", "")
            publisher.publish(msg)

    # ------------------------------------------------------------
    # ROS → Kafka Callbacks
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

    def kawasaki_joint_states_callback(self, msg):
        """Kawasaki + AGV gercek eklem durumlari.

        NEDEN AYRI BIR TOPIC: /joint_states bu hucrede UR10e eklemlerini
        tasiyor (ur10e_*), Kawasaki'ninkiler yalnizca /kawasaki/dynamic_joint_states
        uzerinde. Bu yuzden ros-kawasaki-joint-states 8 Haz 2026'dan beri bostu ve
        arayuzdeki "Kawasaki — Joint Positions (Real)" paneli hicbir sey cizmiyordu:
        veri hic toplanmiyordu (4 Eyl 2026'da tespit edildi).

        Kafka topic'i de ayri tutuluyor. joint_states_topic'e karistirilsaydi
        KafkatoElastic_KawaAGVJointStates.py'nin 'joint1' filtresi iki kaynagi
        birbirine gecirirdi.

        Mesaj joint1..joint6'nin yani sira world_to_agv ve tekerlekleri de
        tasiyor; hepsi oldugu gibi aktariliyor - panel joint1..6'yi okuyor,
        gerisi AGV tarafi icin hazir duruyor.
        """
        data = {
            joint_name: {
                iface_name: iface_value
                for iface_name, iface_value in zip(joint_interfaces.interface_names, joint_interfaces.values)
            }
            for joint_name, joint_interfaces in zip(msg.joint_names, msg.interface_values)
        }
        data["header.stamp.sec"] = msg.header.stamp.sec
        data["header.stamp.nanosec"] = msg.header.stamp.nanosec
        self.safe_publish('kawasaki_dynamic_joint_states_topic', data)

    def sim_joint_states_callback(self, msg):
        # Gerçek ROS2 timestamp'i al
        current_time = self.node.get_clock().now()
        
        data = {
            name: {
                'position': msg.position[i] if i < len(msg.position) else None,
                'velocity': msg.velocity[i] if i < len(msg.velocity) else None,
                'effort': msg.effort[i] if i < len(msg.effort) else None
            }
            for i, name in enumerate(msg.name)
        }
        # Simülasyon zamanı
        data["sim_time.sec"] = msg.header.stamp.sec
        data["sim_time.nanosec"] = msg.header.stamp.nanosec
        
        # Gerçek ROS2 timestamp'i (header.stamp olarak)
        data["header.stamp.sec"] = current_time.seconds_nanoseconds()[0]
        data["header.stamp.nanosec"] = current_time.seconds_nanoseconds()[1]
        
        self.safe_publish('sim_joint_states_topic', data)

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

    def sim_image_callback(self, msg):
        import base64
        import cv2
        import numpy as np
        
        current_time = self.node.get_clock().now()
        
        # Debug: encoding tipini kontrol et
        
        try:
            # Encoding tipine göre işlem yap
            if msg.encoding == '32FC1':
                # Depth image (32-bit float)
                depth_array = np.frombuffer(msg.data, dtype=np.float32)
                depth_image = depth_array.reshape((msg.height, msg.width))
                depth_image = np.nan_to_num(depth_image, nan=0.0, posinf=0.0, neginf=0.0)
                
                if depth_image.max() > depth_image.min():
                    normalized = ((depth_image - depth_image.min()) / 
                                (depth_image.max() - depth_image.min()) * 255).astype(np.uint8)
                else:
                    normalized = np.zeros_like(depth_image, dtype=np.uint8)
                
            elif msg.encoding == 'rgb8':
                # RGB image
                img_array = np.frombuffer(msg.data, dtype=np.uint8)
                img = img_array.reshape((msg.height, msg.width, 3))
                normalized = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                
            elif msg.encoding == 'bgr8':
                # BGR image
                img_array = np.frombuffer(msg.data, dtype=np.uint8)
                normalized = img_array.reshape((msg.height, msg.width, 3))
                
            elif msg.encoding == 'mono8':
                # Grayscale image
                img_array = np.frombuffer(msg.data, dtype=np.uint8)
                normalized = img_array.reshape((msg.height, msg.width))
                
            else:
                raise ValueError(f"Unsupported encoding: {msg.encoding}")
            
            # JPEG olarak compress et
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 80]
            result, encoded_img = cv2.imencode('.jpg', normalized, encode_param)
            
            if result:
                jpg_base64 = base64.b64encode(encoded_img.tobytes()).decode('utf-8')
                
                data = {
                    "sim_time": {
                        "sec": msg.header.stamp.sec,
                        "nanosec": msg.header.stamp.nanosec
                    },
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
            
            # Hata durumunda sadece metadata gönder
            data = {
                "sim_time": {
                    "sec": msg.header.stamp.sec,
                    "nanosec": msg.header.stamp.nanosec
                },
                "header": {
                    "stamp": {
                        "sec": current_time.seconds_nanoseconds()[0],
                        "nanosec": current_time.seconds_nanoseconds()[1]
                    }
                },
                "width": msg.width,
                "height": msg.height,
                "encoding": msg.encoding,
                "error": str(e),
                "error_details": error_details
            }
            self.safe_publish('sim_image_topic', data)

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
                "position": {
                    "x": msg.pose.position.x,
                    "y": msg.pose.position.y,
                    "z": msg.pose.position.z
                },
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
        """
        /scaled_joint_trajectory_controller/controller_state topic'inden 
        gelen JointTrajectoryControllerState mesajını Kafka'ya aktarır
        """
        current_time = self.node.get_clock().now()
        
        data = {
            "header": {
                "stamp": {
                    "sec": current_time.seconds_nanoseconds()[0],
                    "nanosec": current_time.seconds_nanoseconds()[1]
                },
                "frame_id": msg.header.frame_id
            },
            "original_timestamp": {
                "sec": msg.header.stamp.sec,
                "nanosec": msg.header.stamp.nanosec
            },
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
    double_ros2_kafka_bridge()
