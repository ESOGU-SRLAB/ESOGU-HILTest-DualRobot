#!/usr/bin/env python3
"""
ESOGÜ Robotics Lab — Scenario Control Dashboard
Flask + SocketIO backend for managing ROS2 launch scenarios,
camera streaming, and real-time joint state monitoring.
"""

import os
import sys
import signal
import subprocess
import threading
import time
import json
from collections import deque
from datetime import datetime

from flask import Flask, Response, render_template, jsonify, send_from_directory
from flask_socketio import SocketIO, emit

# ==============================================================================
# Flask App Setup
# ==============================================================================
app = Flask(__name__,
            template_folder="templates",
            static_folder="static")
app.config["SECRET_KEY"] = "esogu-robotics-lab-2026"
socketio = SocketIO(app, cors_allowed_origins="*", async_mode="threading")

# ==============================================================================
# Global State
# ==============================================================================

class ScenarioManager:
    """Manages ROS2 launch process lifecycle."""

    # Scenario definitions: name → (hil_params, scenario_launch_cmd)
    SCENARIOS = {
        "multi_robot_inspection": {
            "label": "Multi-Robot Inspection Scenario",
            "hil_params": "",
            "scenario_cmd": "ros2 launch pymoveit2_real multirobot_inspection_scenario.launch.py",
        },
        "ur10e_inspection": {
            "label": "UR10e Inspection Scenario",
            "hil_params": "",
            "scenario_cmd": "ros2 launch pymoveit2_real ur_inspection_scenario.launch.py",
        },
        "pick_and_place": {
            "label": "Pick & Place Scenario",
            "hil_params": "use_vacuum_gripper:=true",
            "scenario_cmd": "ros2 launch pymoveit2_real pick_and_place_scenario.launch.py",
        },
        "human_robot_collaboration": {
            "label": "Human-Robot Collaboration Scenario",
            "hil_params": "use_gripper:=true",
            "scenario_cmd": "ros2 launch pymoveit2_real human_robot_collaboration_scenario.launch.py",
        },
    }

    HIL_BASE_CMD = "ros2 launch my_robot_cell_control hil_test_whole_unified.launch.py"

    def __init__(self, socketio_instance):
        self.socketio = socketio_instance
        self.hil_process = None
        self.scenario_process = None
        self.current_scenario = None
        self.hil_status = "stopped"          # stopped, starting, running, stopping
        self.scenario_status = "stopped"     # stopped, starting, running, stopping
        self.robot_confirmed = False
        self._lock = threading.Lock()
        self._log_threads = []
        
        # Geçiçi Log Dosyası Ayarları
        log_dir = os.path.join(os.path.dirname(__file__), "log")
        os.makedirs(log_dir, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_filepath = os.path.join(log_dir, f"dashboard_log_{timestamp}.txt")
        print(f"[Log] Loglar şu dosyaya yazılacak: {self.log_filepath}")

    def _emit_status(self):
        """Push current status to all connected clients."""
        self.socketio.emit("status_update", {
            "hil_status": self.hil_status,
            "scenario_status": self.scenario_status,
            "current_scenario": self.current_scenario,
            "robot_confirmed": self.robot_confirmed,
        })

    def _emit_log(self, source, message):
        """Push a log message to all connected clients and write to file."""
        timestamp = datetime.now().strftime("%H:%M:%S")
        
        # Terminal/Arayüz logu
        self.socketio.emit("log_message", {
            "timestamp": timestamp,
            "source": source,
            "message": message,
        })
        
        # Dosyaya yaz (Geçici hata ayıklama için)
        try:
            with open(self.log_filepath, "a", encoding="utf-8") as f:
                f.write(f"[{timestamp}] [{source}] {message}\n")
        except Exception as e:
            pass

    def _stream_output(self, process, source_name):
        """Stream subprocess stdout/stderr to frontend log panel."""
        try:
            for line in iter(process.stdout.readline, ""):
                if line:
                    self._emit_log(source_name, line.rstrip())
                if process.poll() is not None:
                    break
        except (ValueError, OSError):
            pass  # Process closed

    def _kill_process(self, process, name, timeout=10):
        """Gracefully kill a process: SIGINT → wait → SIGTERM → wait → SIGKILL."""
        if process is None or process.poll() is not None:
            return

        self._emit_log("SYSTEM", f"🛑 {name} durduruluyor (SIGINT)...")

        try:
            # Send SIGINT to process group (like Ctrl+C)
            os.killpg(os.getpgid(process.pid), signal.SIGINT)
        except (ProcessLookupError, OSError):
            return

        # Wait for graceful shutdown
        try:
            process.wait(timeout=timeout)
            self._emit_log("SYSTEM", f"✅ {name} başarıyla durduruldu.")
            return
        except subprocess.TimeoutExpired:
            pass

        # SIGTERM
        self._emit_log("SYSTEM", f"⚠️ {name} yanıt vermedi, SIGTERM gönderiliyor...")
        try:
            os.killpg(os.getpgid(process.pid), signal.SIGTERM)
            process.wait(timeout=5)
            self._emit_log("SYSTEM", f"✅ {name} SIGTERM ile durduruldu.")
            return
        except (subprocess.TimeoutExpired, ProcessLookupError, OSError):
            pass

        # SIGKILL (last resort)
        self._emit_log("SYSTEM", f"🔴 {name} zorla kapatılıyor (SIGKILL)...")
        try:
            os.killpg(os.getpgid(process.pid), signal.SIGKILL)
            process.wait(timeout=3)
        except (ProcessLookupError, OSError, subprocess.TimeoutExpired):
            pass

        self._emit_log("SYSTEM", f"✅ {name} kapatıldı.")

    def _start_process(self, cmd, name):
        """Start a subprocess with process group for clean shutdown."""
        self._emit_log("SYSTEM", f"🚀 Başlatılıyor: {cmd}")
        
        # Her komuttan önce ROS 2 ortam değişkenlerini (workspace) yüklüyoruz
        full_cmd = f"source /opt/ros/humble/setup.bash && source /home/cem/colcon_ws/install/setup.bash && {cmd}"
        
        # DISPLAY vb. değişkenleri alt prosese aktar (RViz, Gazebo vb. GUI için)
        my_env = os.environ.copy()
        
        # OpenCV (cv2) import edildiğinde kendi QT_QPA_PLATFORM_PLUGIN_PATH ortam değişkenini ayarlar.
        # Bu değişken alt süreçlere geçtiğinde RViz ve Gazebo'nun açılmasını (xcb hatası) engeller.
        # Bu nedenle, alt süreçlere aktarılmadan önce bu değişkeni temizliyoruz.
        if "QT_QPA_PLATFORM_PLUGIN_PATH" in my_env:
            del my_env["QT_QPA_PLATFORM_PLUGIN_PATH"]
        
        process = subprocess.Popen(
            full_cmd,
            shell=True,
            executable="/bin/bash",
            env=my_env,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            stdin=subprocess.PIPE,
            text=True,
            bufsize=1,
            preexec_fn=os.setsid,  # New process group for clean SIGINT
        )

        # Start output streaming thread
        log_thread = threading.Thread(
            target=self._stream_output,
            args=(process, name),
            daemon=True,
        )
        log_thread.start()
        self._log_threads.append(log_thread)

        return process

    def stop_all(self):
        """Emergency stop: kill everything."""
        with self._lock:
            self._emit_log("SYSTEM", "🚨 ACİL DURDURMA — Tüm süreçler kapatılıyor!")

            self.scenario_status = "stopping"
            self._emit_status()
            self._kill_process(self.scenario_process, "Senaryo")
            self.scenario_process = None
            self.scenario_status = "stopped"

            self.hil_status = "stopping"
            self._emit_status()
            self._kill_process(self.hil_process, "HIL")
            self.hil_process = None
            self.hil_status = "stopped"

            self.current_scenario = None
            self.robot_confirmed = False

            self._emit_status()
            self._emit_log("SYSTEM", "✅ Tüm süreçler durduruldu.")

    def start_scenario(self, scenario_key, use_fake_hardware=False):
        """Start a scenario: stop existing → start HIL → wait for confirm → start scenario."""
        if scenario_key not in self.SCENARIOS:
            self._emit_log("SYSTEM", f"❌ Bilinmeyen senaryo: {scenario_key}")
            return

        def _run():
            with self._lock:
                scenario = self.SCENARIOS[scenario_key]

                # 1. Stop existing processes
                if self.scenario_process or self.hil_process:
                    self._emit_log("SYSTEM", "📋 Mevcut süreçler durduruluyor...")
                    self.scenario_status = "stopping"
                    self._emit_status()
                    self._kill_process(self.scenario_process, "Senaryo")
                    self.scenario_process = None
                    self.scenario_status = "stopped"

                    self.hil_status = "stopping"
                    self._emit_status()
                    self._kill_process(self.hil_process, "HIL")
                    self.hil_process = None
                    self.hil_status = "stopped"
                    self._emit_status()

                    time.sleep(2)  # Brief pause between stop and start

                # 2. Start HIL
                self.current_scenario = scenario_key
                self.robot_confirmed = False
                self.hil_status = "starting"
                self._emit_status()

                hil_cmd = self.HIL_BASE_CMD
                if scenario["hil_params"]:
                    hil_cmd += f" {scenario['hil_params']}"
                if use_fake_hardware:
                    hil_cmd += " use_fake_hardware:=true use_mock_hardware:=true fake_sensor_commands:=true"

                self.hil_process = self._start_process(hil_cmd, "HIL")
                self.hil_status = "running"
                self._emit_status()

                self._emit_log("SYSTEM",
                    f"✅ HIL başlatıldı: {scenario['label']}. "
                    "Robotun hazır olduğunu onaylayın.")

        thread = threading.Thread(target=_run, daemon=True)
        thread.start()

    def confirm_robot_ready(self):
        """User confirms robot is ready → start the scenario launch."""
        def _run():
            with self._lock:
                if not self.current_scenario or self.hil_status != "running":
                    self._emit_log("SYSTEM", "❌ HIL çalışmıyor, onay verilemez.")
                    return

                scenario = self.SCENARIOS[self.current_scenario]
                self.robot_confirmed = True
                self.scenario_status = "starting"
                self._emit_status()

                self._emit_log("SYSTEM", "✅ Robot hazır onayı alındı. Senaryo başlatılıyor...")

                self.scenario_process = self._start_process(
                    scenario["scenario_cmd"], "SENARYO"
                )
                self.scenario_status = "running"
                self._emit_status()

                self._emit_log("SYSTEM", f"🚀 {scenario['label']} çalışıyor!")

        thread = threading.Thread(target=_run, daemon=True)
        thread.start()

    def get_status(self):
        """Get current status as dict."""
        return {
            "hil_status": self.hil_status,
            "scenario_status": self.scenario_status,
            "current_scenario": self.current_scenario,
            "robot_confirmed": self.robot_confirmed,
        }


# Initialize the scenario manager
scenario_mgr = ScenarioManager(socketio)


# ==============================================================================
# Joint State Subscriber (ROS2)
# ==============================================================================

class JointStateCollector:
    """Collects joint states from ROS2 topics and pushes via SocketIO."""

    def __init__(self, socketio_instance, buffer_seconds=20):
        self.socketio = socketio_instance
        self.buffer_seconds = buffer_seconds
        # Circular buffers: deque of {timestamp, joint_name: angle}
        self.real_data = deque(maxlen=buffer_seconds * 1000)  # up to 1000 Hz
        self.sim_data = deque(maxlen=buffer_seconds * 1000)
        self._rclpy_thread = None
        self._running = False
        self._data_lock = threading.Lock()

    def start(self):
        """Start the ROS2 subscriber thread."""
        self._running = True
        self._rclpy_thread = threading.Thread(target=self._ros_spin, daemon=True)
        self._rclpy_thread.start()

        # Start a periodic push thread
        self._push_thread = threading.Thread(target=self._periodic_push, daemon=True)
        self._push_thread.start()

    def _ros_spin(self):
        """ROS2 subscriber spin loop."""
        try:
            import rclpy
            from rclpy.node import Node as RosNode
            from sensor_msgs.msg import JointState

            rclpy.init()
            node = RosNode("dashboard_joint_listener")

            def real_cb(msg):
                data = {"t": time.time()}
                for name, pos in zip(msg.name, msg.position):
                    data[name] = pos
                with self._data_lock:
                    self.real_data.append(data)

            def sim_cb(msg):
                data = {"t": time.time()}
                for name, pos in zip(msg.name, msg.position):
                    data[name] = pos
                with self._data_lock:
                    self.sim_data.append(data)

            node.create_subscription(JointState, "/joint_states", real_cb, 10)
            node.create_subscription(JointState, "/sim/joint_states", sim_cb, 10)

            node.get_logger().info("Dashboard joint state listener started.")

            while self._running and rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.1)

            node.destroy_node()
            rclpy.shutdown()

        except Exception as e:
            print(f"[JointStateCollector] ROS2 error: {e}")
            print("[JointStateCollector] Joint state monitoring disabled.")

    def _periodic_push(self):
        """Push joint state data to frontend every 200ms."""
        while self._running:
            time.sleep(0.2)  # 5 Hz update rate for charts

            try:
                now = time.time()
                cutoff = now - self.buffer_seconds

                # Filter to last N seconds and downsample
                with self._data_lock:
                    real_filtered = [d for d in self.real_data if d["t"] > cutoff]
                    sim_filtered = [d for d in self.sim_data if d["t"] > cutoff]

                # Downsample to max ~100 points for performance
                real_sampled = self._downsample(real_filtered, 100)
                sim_sampled = self._downsample(sim_filtered, 100)

                self.socketio.emit("joint_states", {
                    "real": real_sampled,
                    "sim": sim_sampled,
                    "now": now,
                    "window": self.buffer_seconds,
                })
            except Exception as e:
                print(f"[JointStateCollector] Push error: {e}")

    def _downsample(self, data, max_points):
        """Simple downsample by skipping entries."""
        if len(data) <= max_points:
            return data
        step = len(data) / max_points
        return [data[int(i * step)] for i in range(max_points)]

    def stop(self):
        self._running = False


joint_collector = JointStateCollector(socketio)


# ==============================================================================
# Camera Streaming
# ==============================================================================

class CameraStreamer:
    """MJPEG camera streaming for both Gazebo and real-world cameras."""

    def __init__(self):
        self._gazebo_frame = None
        self._real_frame = None
        self._gazebo_lock = threading.Lock()
        self._real_lock = threading.Lock()
        self._gazebo_running = False
        self._real_running = False

    def start_gazebo_stream(self):
        """Start Gazebo camera via gz.transport."""
        self._gazebo_running = True
        thread = threading.Thread(target=self._gazebo_worker, daemon=True)
        thread.start()

    def _gazebo_worker(self):
        """Subscribe to Gazebo camera topic via gz.transport."""
        try:
            from gz.transport13 import Node as GzNode
            from gz.msgs10.image_pb2 import Image as GzImage
            from PIL import Image as PILImage
            import io

            def cb(msg):
                try:
                    img = PILImage.frombytes("RGB", (msg.width, msg.height), msg.data)
                    buf = io.BytesIO()
                    img.save(buf, format="JPEG", quality=70)
                    with self._gazebo_lock:
                        self._gazebo_frame = buf.getvalue()
                except Exception as e:
                    print(f"[CameraStreamer] Gazebo frame decode error: {e}")

            gz_node = GzNode()
            gz_node.subscribe(GzImage, "/web_camera/image", cb)

            print("[CameraStreamer] Gazebo camera subscriber started.")
            while self._gazebo_running:
                time.sleep(0.1)

        except ImportError:
            print("[CameraStreamer] gz.transport not available. Gazebo camera disabled.")
        except Exception as e:
            print(f"[CameraStreamer] Gazebo camera error: {e}")

    def start_real_stream(self):
        """Start real-world RTSP camera stream."""
        self._real_running = True
        thread = threading.Thread(target=self._real_worker, daemon=True)
        thread.start()

    def _real_worker(self):
        """Capture RTSP stream and convert to MJPEG."""
        try:
            import cv2
            rtsp_url = "rtsp://192.168.4.51:554/live/0"

            while self._real_running:
                cap = cv2.VideoCapture(rtsp_url)
                if not cap.isOpened():
                    print(f"[CameraStreamer] RTSP bağlantısı başarısız: {rtsp_url}")
                    print("[CameraStreamer] 5 saniye sonra tekrar denenecek...")
                    time.sleep(5)
                    continue

                print("[CameraStreamer] Real camera connected.")
                while self._real_running:
                    ret, frame = cap.read()
                    if not ret:
                        print("[CameraStreamer] RTSP frame alınamadı, yeniden bağlanılıyor...")
                        break
                    _, jpeg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
                    with self._real_lock:
                        self._real_frame = jpeg.tobytes()

                cap.release()
                time.sleep(2)

        except ImportError:
            print("[CameraStreamer] OpenCV not available. Real camera disabled.")
        except Exception as e:
            print(f"[CameraStreamer] Real camera error: {e}")

    def generate_gazebo(self):
        """MJPEG generator for Gazebo camera."""
        while True:
            with self._gazebo_lock:
                frame = self._gazebo_frame
            if frame:
                yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"
                       + frame + b"\r\n")
            time.sleep(0.05)  # ~20 FPS max

    def generate_real(self):
        """MJPEG generator for real camera."""
        while True:
            with self._real_lock:
                frame = self._real_frame
            if frame:
                yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"
                       + frame + b"\r\n")
            time.sleep(0.05)

    def stop(self):
        self._gazebo_running = False
        self._real_running = False


camera_streamer = CameraStreamer()


# ==============================================================================
# Health Check
# ==============================================================================

def check_ros2_health():
    """Check ROS2 system health by looking at available topics/controllers."""
    health = {
        "joint_states": False,
        "sim_joint_states": False,
        "controller_manager": False,
        "sim_controller_manager": False,
    }

    try:
        result = subprocess.run(
            ["ros2", "topic", "list"],
            capture_output=True, text=True, timeout=5
        )
        topics = result.stdout.strip().split("\n") if result.returncode == 0 else []

        health["joint_states"] = "/joint_states" in topics
        health["sim_joint_states"] = "/sim/joint_states" in topics
        health["controller_manager"] = any(
            "/controller_manager" in t for t in topics
        )
        health["sim_controller_manager"] = any(
            "/sim/controller_manager" in t for t in topics
        )
    except (subprocess.TimeoutExpired, FileNotFoundError):
        pass

    return health


# ==============================================================================
# Flask Routes
# ==============================================================================

@app.route("/")
def index():
    return render_template("index.html")

@app.route("/figures/<path:filename>")
def serve_figures(filename):
    """Serve logo images from the figures directory."""
    figures_dir = os.path.join(os.path.dirname(__file__), "figures")
    return send_from_directory(figures_dir, filename)

@app.route("/stream/gazebo")
def gazebo_stream():
    return Response(
        camera_streamer.generate_gazebo(),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )

@app.route("/stream/real")
def real_stream():
    return Response(
        camera_streamer.generate_real(),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )

@app.route("/api/status")
def api_status():
    return jsonify(scenario_mgr.get_status())

@app.route("/api/health")
def api_health():
    return jsonify(check_ros2_health())


# ==============================================================================
# SocketIO Events
# ==============================================================================

@socketio.on("connect")
def handle_connect():
    emit("status_update", scenario_mgr.get_status())

@socketio.on("start_scenario")
def handle_start_scenario(data):
    scenario_key = data.get("scenario")
    use_fake_hardware = data.get("use_fake_hardware", False)
    scenario_mgr.start_scenario(scenario_key, use_fake_hardware)

@socketio.on("confirm_robot")
def handle_confirm_robot():
    scenario_mgr.confirm_robot_ready()

@socketio.on("stop_all")
def handle_stop_all():
    thread = threading.Thread(target=scenario_mgr.stop_all, daemon=True)
    thread.start()

@socketio.on("request_health")
def handle_health_check():
    health = check_ros2_health()
    emit("health_update", health)


# ==============================================================================
# Main Entry Point
# ==============================================================================

def main():
    print("=" * 60)
    print("  ESOGÜ Robotics Lab — Scenario Control Dashboard")
    print("  http://localhost:8080")
    print("=" * 60)

    # Start camera streamers
    camera_streamer.start_gazebo_stream()
    camera_streamer.start_real_stream()

    # Start joint state collector
    joint_collector.start()

    try:
        socketio.run(app, host="0.0.0.0", port=8080, debug=False, allow_unsafe_werkzeug=True)
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        camera_streamer.stop()
        joint_collector.stop()
        scenario_mgr.stop_all()


if __name__ == "__main__":
    main()
