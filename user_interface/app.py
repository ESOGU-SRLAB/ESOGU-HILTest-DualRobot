#!/usr/bin/env python3
"""
Robotic Testbed as a Service — ESOGÜ IFARLAB Control Dashboard
Flask + SocketIO backend for managing ROS2 launch scenarios, camera streaming,
real-time joint state monitoring, and UR10e anomaly detection.
"""

import os
import sys
import shlex
import signal
import subprocess
import threading
import time
import json
import urllib.request
import urllib.error
from collections import deque
from datetime import datetime

from flask import Flask, Response, render_template, jsonify, send_from_directory, request
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

# Bu dosya <workspace>/src/user_interface/app.py konumunda olduğu için workspace
# kökünü dosya yolundan türetiyoruz (makineye özel mutlak yol yazmamak için).
WORKSPACE_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
WORKSPACE_SETUP = os.path.join(WORKSPACE_ROOT, "install", "setup.bash")

class ScenarioManager:
    """Manages ROS2 launch process lifecycle."""

    # Scenario definitions: name → (hil_params, scenario_launch_cmd)
    SCENARIOS = {
        "multi_robot_inspection": {
            "label": "Multi-Robot Inspection Scenario",
            "hil_params": "",
            "scenario_cmd": "ros2 launch multirobot_viewpoint_planner multirobot_inspection.launch.py",
            # use_fake_hardware açıkken only_sim:=true, gerçek robot bağlıyken only_sim:=false
            "sim_flag": "only_sim",
        },
        "ur10e_inspection": {
            "label": "UR10e Inspection Scenario",
            "hil_params": "",
            "scenario_cmd": "ros2 launch viewpoint_planner inspection_execution.launch.py",
            "sim_flag": "only_sim",
        },
        "pick_and_place": {
            "label": "Pick & Place Scenario",
            "hil_params": "use_vacuum_gripper:=true",
            "scenario_cmd": "ros2 launch gemini_robotics_ros gemini_pick_place.launch.py",
            # gemini_pick_place'in TEK argümanı mode ve değeri sim|real - yani
            # sim_flag'in ürettiği "bayrak:=true/false" biçimine uymuyor.
            # (argüman adı, fake hardware açıkken, gerçek robot bağlıyken)
            "mode_arg": ("mode", "sim", "real"),
            # Senaryo ayağa kalkınca komut penceresi açılsın: bu senaryoda görev
            # serbest metinle veriliyor (/gemini/command).
            "command_prompt": True,
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
        self.use_fake_hardware = False       # Son başlatılan senaryonun sim/gerçek modu
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
        print(f"[Log] Logs will be written to: {self.log_filepath}")

    def _emit_status(self):
        """Push current status to all connected clients.

        get_status() üzerinden gidiyor: iki yerde ayrı ayrı sözlük kurulunca
        biri güncellenip diğeri unutuluyordu (yeni alanlar yalnızca sayfa
        yenilenince görünüyordu).
        """
        self.socketio.emit("status_update", self.get_status())

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

        self._emit_log("SYSTEM", f"🛑 Stopping {name} (SIGINT)...")

        try:
            # Send SIGINT to process group (like Ctrl+C)
            os.killpg(os.getpgid(process.pid), signal.SIGINT)
        except (ProcessLookupError, OSError):
            return

        # Wait for graceful shutdown
        try:
            process.wait(timeout=timeout)
            self._emit_log("SYSTEM", f"✅ {name} successfully stopped.")
            return
        except subprocess.TimeoutExpired:
            pass

        # SIGTERM
        self._emit_log("SYSTEM", f"⚠️ {name} did not respond, sending SIGTERM...")
        try:
            os.killpg(os.getpgid(process.pid), signal.SIGTERM)
            process.wait(timeout=5)
            self._emit_log("SYSTEM", f"✅ {name} stopped via SIGTERM.")
            return
        except (subprocess.TimeoutExpired, ProcessLookupError, OSError):
            pass

        # SIGKILL (last resort)
        self._emit_log("SYSTEM", f"🔴 Force killing {name} (SIGKILL)...")
        try:
            os.killpg(os.getpgid(process.pid), signal.SIGKILL)
            process.wait(timeout=3)
        except (ProcessLookupError, OSError, subprocess.TimeoutExpired):
            pass

        self._emit_log("SYSTEM", f"✅ {name} closed.")

    def _start_process(self, cmd, name):
        """Start a subprocess with process group for clean shutdown."""
        self._emit_log("SYSTEM", f"🚀 Starting: {cmd}")
        
        # Her komuttan önce ROS 2 ortam değişkenlerini (workspace) yüklüyoruz
        full_cmd = f"source /opt/ros/humble/setup.bash && source {WORKSPACE_SETUP} && {cmd}"
        
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
            self._emit_log("SYSTEM", "🚨 EMERGENCY STOP — Terminating all processes!")

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
            self.use_fake_hardware = False
            self.robot_confirmed = False

            self._emit_status()
            self._emit_log("SYSTEM", "✅ All processes stopped.")

    def start_scenario(self, scenario_key, use_fake_hardware=False):
        """Start a scenario: stop existing → start HIL → wait for confirm → start scenario."""
        if scenario_key not in self.SCENARIOS:
            self._emit_log("SYSTEM", f"❌ Unknown scenario: {scenario_key}")
            return

        def _run():
            with self._lock:
                scenario = self.SCENARIOS[scenario_key]

                # 1. Stop existing processes
                if self.scenario_process or self.hil_process:
                    self._emit_log("SYSTEM", "📋 Stopping current processes...")
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
                self.use_fake_hardware = use_fake_hardware
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
                    f"✅ HIL initialized: {scenario['label']}. "
                    "Please confirm that the robot is ready.")

        thread = threading.Thread(target=_run, daemon=True)
        thread.start()

    @staticmethod
    def build_scenario_cmd(scenario, use_fake_hardware):
        """Senaryo launch komutunu sim/gerçek moduna göre kurar.

        İki farklı biçim var, çünkü launch dosyaları farklı:
          sim_flag : "only_sim:=true|false"   (boolean bayrak)
          mode_arg : "mode:=sim|real"         (adlandırılmış mod)
        gemini_pick_place ikincisini kullanıyor - tek argümanı mode ve
        boolean kabul etmiyor (choices=["sim", "real"]).
        """
        cmd = scenario["scenario_cmd"]
        if scenario.get("sim_flag"):
            cmd += f" {scenario['sim_flag']}:={'true' if use_fake_hardware else 'false'}"
        if scenario.get("mode_arg"):
            arg_name, fake_value, real_value = scenario["mode_arg"]
            cmd += f" {arg_name}:={fake_value if use_fake_hardware else real_value}"
        return cmd

    def confirm_robot_ready(self):
        """User confirms robot is ready → start the scenario launch."""
        def _run():
            with self._lock:
                if not self.current_scenario or self.hil_status != "running":
                    self._emit_log("SYSTEM", "❌ HIL is not running, cannot confirm.")
                    return

                scenario = self.SCENARIOS[self.current_scenario]
                self.robot_confirmed = True
                self.scenario_status = "starting"
                self._emit_status()

                self._emit_log("SYSTEM", "✅ Robot confirmation received. Starting scenario...")

                scenario_cmd = self.build_scenario_cmd(
                    scenario, self.use_fake_hardware)

                self.scenario_process = self._start_process(scenario_cmd, "SENARYO")
                self.scenario_status = "running"
                self._emit_status()

                self._emit_log("SYSTEM", f"🚀 {scenario['label']} is running!")

                # Serbest metinle görev alan senaryolarda komut penceresini aç.
                if scenario.get("command_prompt"):
                    self.socketio.emit("command_prompt", {
                        "scenario": self.current_scenario,
                        "label": scenario["label"],
                    })

        thread = threading.Thread(target=_run, daemon=True)
        thread.start()

    # --------------------------------------------------------------------
    # Serbest metin görev komutu (/gemini/command)
    # --------------------------------------------------------------------

    COMMAND_TOPIC = "/gemini/command"
    COMMAND_TIMEOUT_SEC = 180

    @staticmethod
    def build_command_publish_cmd(text):
        """Kullanıcının yazdığı metni güvenli bir `ros2 topic pub` komutuna çevirir.

        İKİ ayrı kaçış katmanı var ve ikisi de gerekli:

        1. YAML: değer tek tırnak içinde veriliyor, dolayısıyla metindeki her
           tek tırnak İKİYE katlanmalı ('' YAML'da tek tırnak demektir). Aksi
           halde "toolkit's" yazan bir komut YAML'ı ortasından kapatır.
        2. Kabuk: komut /bin/bash üzerinden çalıştığı için tüm YAML yükü
           shlex.quote'tan geçer; böylece $, `, " ve boşluklar kabuk tarafından
           yorumlanmaz.

        Satır sonları tek boşluğa indiriliyor: YAML'ın tek tırnaklı skaleri çok
        satırlıyı katlama kurallarıyla ele alıyor ve metin sessizce değişiyor.
        """
        normalized = " ".join(str(text).split())
        yaml_payload = "{data: '" + normalized.replace("'", "''") + "'}"
        return (
            f"ros2 topic pub --once {ScenarioManager.COMMAND_TOPIC} "
            f"std_msgs/String {shlex.quote(yaml_payload)}"
        ), normalized

    def publish_command(self, text):
        """Görev komutunu /gemini/command'a yayınlar (ayrı thread'de)."""
        cmd, normalized = self.build_command_publish_cmd(text)
        if not normalized:
            self._emit_log("SYSTEM", "❌ Empty command, nothing published.")
            self.socketio.emit("command_result", {
                "ok": False, "message": "Command is empty.",
            })
            return

        def _run():
            if self.scenario_status != "running":
                self._emit_log("SYSTEM",
                    "⚠️ Scenario is not running; publishing anyway — the command "
                    "will wait until a subscriber appears.")

            self._emit_log("SYSTEM", f"💬 Task command: {normalized}")
            self._emit_log("SYSTEM", f"🚀 {cmd}")
            # `--once` Humble'da varsayılan olarak -w 1 demek: eşleşen bir abone
            # bulunana kadar BEKLER. Yani düğümler henüz kalkmadıysa mesaj
            # kaybolmaz, komut bekler - bu yüzden zaman aşımı şart, yoksa
            # düğüm hiç gelmezse süreç sonsuza kadar asılı kalır.
            full_cmd = (
                f"source /opt/ros/humble/setup.bash && "
                f"source {WORKSPACE_SETUP} && {cmd}"
            )
            my_env = os.environ.copy()
            my_env.pop("QT_QPA_PLATFORM_PLUGIN_PATH", None)
            try:
                result = subprocess.run(
                    full_cmd, shell=True, executable="/bin/bash", env=my_env,
                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                    text=True, timeout=self.COMMAND_TIMEOUT_SEC,
                )
            except subprocess.TimeoutExpired:
                self._emit_log("SYSTEM",
                    f"❌ No subscriber on {self.COMMAND_TOPIC} within "
                    f"{self.COMMAND_TIMEOUT_SEC} s — command NOT delivered. "
                    "Is gemini_pick_place running?")
                self.socketio.emit("command_result", {
                    "ok": False,
                    "message": f"No subscriber on {self.COMMAND_TOPIC}. "
                               "Command was not delivered.",
                })
                return

            output = (result.stdout or "").strip()
            for line in output.splitlines():
                if line.strip():
                    self._emit_log("COMMAND", line.rstrip())

            if result.returncode == 0:
                self._emit_log("SYSTEM", "✅ Task command published.")
                self.socketio.emit("command_result", {
                    "ok": True, "message": "Command published to "
                                           f"{self.COMMAND_TOPIC}.",
                })
            else:
                self._emit_log("SYSTEM",
                    f"❌ Publish failed (exit {result.returncode}).")
                self.socketio.emit("command_result", {
                    "ok": False,
                    "message": f"ros2 topic pub failed (exit {result.returncode}). "
                               "See the log panel.",
                })

        threading.Thread(target=_run, daemon=True).start()

    def get_status(self):
        """Get current status as dict."""
        scenario = self.SCENARIOS.get(self.current_scenario, {})
        return {
            "hil_status": self.hil_status,
            "scenario_status": self.scenario_status,
            "current_scenario": self.current_scenario,
            "robot_confirmed": self.robot_confirmed,
            # Bu senaryo serbest metin komut alıyor mu (Send Command butonu)
            "command_prompt": bool(scenario.get("command_prompt")),
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
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.node import Node as RosNode
            from sensor_msgs.msg import JointState

            if not rclpy.ok():
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

            # Özel executor: global executor'ı paylaşmak iki collector arasında
            # sessiz sağırlığa yol açıyor (ayrıntı AnomalyCollector içinde).
            executor = SingleThreadedExecutor()
            executor.add_node(node)

            while self._running and rclpy.ok():
                executor.spin_once(timeout_sec=0.1)

            executor.remove_node(node)
            node.destroy_node()
            # rclpy.shutdown() ÇAĞIRMA: context artık AnomalyCollector ile
            # paylaşılıyor, burada kapatmak onun düğümünü de öldürür.

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
# Anomaly Detection (ROS2)
# ==============================================================================

ANOMALY_LOG_DIR = os.path.expanduser("~/anomali_kayit")
ANOMALY_LABEL_FILE = os.path.join(ANOMALY_LOG_DIR, "etiketler.json")


class AnomalyCollector:
    """Listens to the anomaly_detection node and pushes it to the UI over SocketIO.

    Same pattern as JointStateCollector, with two differences:

    1. rclpy.init() is GUARDED. JointStateCollector already initialises it, and a
       second call blows up with "rcl_init called while already initialized".
    2. The alarm is captured as an EDGE, not as a level. The detector decides at
       20 Hz while the UI is fed at 5 Hz, so sampling the instantaneous value
       drops short events. Measured: the clearest real event of 21 Aug 2026 (the
       pick & place collision, peak 146.4) lasted only 0.25 s -- a five-decision
       window.
    """

    THR_FUSED = 18.0        # value baked into fusion_config.json
    THR_RESIDUAL = 1.6008
    THR_RAW = 3.4461

    def __init__(self, socketio_instance, buffer_seconds=60):
        self.socketio = socketio_instance
        self.buffer_seconds = buffer_seconds
        self.samples = deque(maxlen=buffer_seconds * 25)   # ~20 Hz decisions
        self._lock = threading.Lock()
        self._running = False
        self._connected_at = 0.0
        self._n_msgs = 0
        # Edge latch: have we seen an alarm since the last push?
        self._alarm_edge = False
        self._alarm_now = False
        self._last_alarm_t = 0.0

    def start(self):
        self._running = True
        threading.Thread(target=self._ros_spin, daemon=True).start()
        threading.Thread(target=self._periodic_push, daemon=True).start()

    def _ros_spin(self):
        try:
            import rclpy
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.node import Node as RosNode
            from std_msgs.msg import Bool, Float32, Float32MultiArray

            if not rclpy.ok():
                rclpy.init()
            node = RosNode("dashboard_anomaly_listener")
            ns = "/ur10e_anomaly_detector"

            def detail_cb(msg):
                d = list(msg.data)
                if len(d) < 11:
                    return
                rec = {
                    "t": time.time(),
                    "s_kal": d[0], "s_ham": d[1],
                    "fused": d[4], "thr": d[5], "thr_ad": d[6],
                    "hit_abs": bool(d[7]), "hit_ad": bool(d[8]),
                    "hit_kal": bool(d[9]), "hit_ham": bool(d[10]),
                }
                if len(d) >= 15:
                    rec["moving"] = bool(d[11])
                    rec["qd_peak"] = d[12]
                with self._lock:
                    self.samples.append(rec)
                    self._n_msgs += 1
                    if not self._connected_at:
                        self._connected_at = rec["t"]

            def det_cb(msg):
                with self._lock:
                    self._alarm_now = bool(msg.data)
                    if msg.data:
                        self._alarm_edge = True          # latch; cleared by push
                        self._last_alarm_t = time.time()

            node.create_subscription(Float32MultiArray, ns + "/detail", detail_cb, 20)
            node.create_subscription(Bool, ns + "/detected", det_cb, 20)
            node.create_subscription(Float32, ns + "/score", lambda m: None, 10)
            node.get_logger().info("Dashboard anomaly listener started.")

            # A DEDICATED executor is mandatory. Without one, rclpy.spin_once(node)
            # adds the node to the GLOBAL executor and leaves it there. Once both
            # collectors land on that same global executor, two threads race on the
            # same wait set: the node disappears from the ROS graph, the
            # subscriptions never receive data, and no error is ever printed.
            # (Exactly what happened on 21 Aug 2026.)
            executor = SingleThreadedExecutor()
            executor.add_node(node)
            try:
                while self._running and rclpy.ok():
                    executor.spin_once(timeout_sec=0.1)
            finally:
                executor.remove_node(node)
                node.destroy_node()

        except Exception as e:
            print(f"[AnomalyCollector] ROS2 error: {e}")
            print("[AnomalyCollector] Anomaly monitoring disabled.")

    def _periodic_push(self):
        while self._running:
            time.sleep(0.2)
            try:
                now = time.time()
                with self._lock:
                    cutoff = now - self.buffer_seconds
                    seri = [r for r in self.samples if r["t"] > cutoff]
                    edge, self._alarm_edge = self._alarm_edge, False
                    alarm_now = self._alarm_now
                    last_alarm = self._last_alarm_t
                    n = self._n_msgs
                    t0 = self._connected_at
                son = seri[-1] if seri else None
                canli = bool(son and now - son["t"] < 2.0)
                self.socketio.emit("anomaly_update", {
                    "connected": canli,
                    "alarm": alarm_now,
                    "alarm_edge": edge,          # did an alarm occur in this window?
                    "last_alarm_ago": (now - last_alarm) if last_alarm else None,
                    "thresholds": {"fused": self.THR_FUSED,
                                   "residual": self.THR_RESIDUAL,
                                   "raw": self.THR_RAW},
                    "current": son,
                    "series": self._downsample(seri, 240),
                    "decision_hz": (n / (now - t0)) if t0 and now > t0 else 0.0,
                    "now": now,
                    "window": self.buffer_seconds,
                })
            except Exception as e:
                print(f"[AnomalyCollector] Push error: {e}")

    @staticmethod
    def _downsample(data, max_points):
        if len(data) <= max_points:
            return data
        step = len(data) / max_points
        return [data[int(i * step)] for i in range(max_points)]

    def stop(self):
        self._running = False


anomaly_collector = AnomalyCollector(socketio)


def _anomaly_labels():
    try:
        with open(ANOMALY_LABEL_FILE, encoding="utf-8") as f:
            return json.load(f)
    except Exception:
        return {}


def _read_anomaly_events(limit=200):
    """Reads the olaylar_*.jsonl files and merges the started/ended record pairs.

    PEAK is the primary column in the table: on 21 Aug 2026, reading the entry
    value (8.5) without seeing the peak (90.55) led to a wrong interpretation. The
    peak only exists in the `anomali_bitti` record, hence the pairing.
    """
    import glob
    etiket = _anomaly_labels()
    olaylar = []
    for path in sorted(glob.glob(os.path.join(ANOMALY_LOG_DIR, "**", "olaylar_*.jsonl"),
                                 recursive=True)):
        kosu = os.path.basename(path).replace("olaylar_", "").replace(".jsonl", "")
        acik = {}
        try:
            with open(path, encoding="utf-8") as f:
                for satir in f:
                    satir = satir.strip()
                    if not satir:
                        continue
                    try:
                        e = json.loads(satir)
                    except ValueError:
                        continue
                    sira = e.get("sira")
                    if e.get("olay") == "anomali_basladi":
                        eid = f"{kosu}#{sira}"
                        kayit = {
                            "id": eid, "kosu": kosu, "sira": sira,
                            "zaman": e.get("zaman"), "t_ros": e.get("t_ros"),
                            "giris": e.get("birlesik"), "esik": e.get("esik"),
                            "kural": e.get("kural"), "tetikleyen": e.get("tetikleyen"),
                            "s_kal": e.get("s_kal"), "s_ham": e.get("s_ham"),
                            "q": e.get("q"), "akim": e.get("akim"),
                            "sure_s": None, "tepe": None,
                            "etiket": etiket.get(eid, {}).get("etiket", "?"),
                            "not": etiket.get(eid, {}).get("not", ""),
                        }
                        acik[sira] = kayit
                        olaylar.append(kayit)
                    elif e.get("olay") == "anomali_bitti" and sira in acik:
                        acik[sira]["sure_s"] = e.get("sure_s")
                        acik[sira]["tepe"] = e.get("tepe")
        except OSError:
            continue
    # An unfinished event has no known peak; use its entry value so the table
    # does not show a blank cell.
    for o in olaylar:
        if o["tepe"] is None:
            o["tepe"] = o["giris"]
            o["devam"] = True
    olaylar.sort(key=lambda o: (o.get("t_ros") or 0), reverse=True)
    return olaylar[:limit]


@app.route("/api/anomaly/events")
def api_anomaly_events():
    try:
        return jsonify({"ok": True, "events": _read_anomaly_events()})
    except Exception as e:
        return jsonify({"ok": False, "error": str(e), "events": []}), 500


@app.route("/api/anomaly/label", methods=["POST"])
def api_anomaly_label():
    """Operator label. Does NOT touch the detector's logs; writes to a separate file."""
    data = request.get_json(silent=True) or {}
    eid = data.get("id")
    etk = data.get("etiket")
    if not eid or etk not in ("gercek", "yanlis", "?"):
        return jsonify({"ok": False, "error": "invalid id or label"}), 400
    try:
        os.makedirs(ANOMALY_LOG_DIR, exist_ok=True)
        tum = _anomaly_labels()
        tum[eid] = {"etiket": etk, "not": (data.get("not") or "")[:500],
                    "zaman": time.strftime("%Y-%m-%dT%H:%M:%S")}
        tmp = ANOMALY_LABEL_FILE + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(tum, f, ensure_ascii=False, indent=2)
        os.replace(tmp, ANOMALY_LABEL_FILE)
        return jsonify({"ok": True, "etiket": etk})
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)}), 500


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

    def _get_rviz_window(self):
        try:
            from Xlib import display
            def get_window_by_name(window, name):
                try:
                    w_name = window.get_wm_name()
                except Exception:
                    w_name = None
                if w_name and name in w_name:
                    return window
                for child in window.query_tree().children:
                    res = get_window_by_name(child, name)
                    if res:
                        return res
                return None
            
            d = display.Display()
            root = d.screen().root
            return get_window_by_name(root, "RViz")
        except Exception:
            return None

    def _try_rviz_capture(self, rviz_win):
        try:
            from Xlib import X
            from PIL import Image
            import numpy as np
            import cv2
            
            if rviz_win:
                geom = rviz_win.get_geometry()
                raw = rviz_win.get_image(0, 0, geom.width, geom.height, X.ZPixmap, 0xffffffff)
                img = Image.frombytes("RGB", (geom.width, geom.height), raw.data, "raw", "BGRX")
                cv_img = cv2.cvtColor(np.array(img), cv2.COLOR_RGB2BGR)
                
                # Resize if the window is too large (to save bandwidth)
                if geom.width > 1280:
                    scale = 1280 / geom.width
                    cv_img = cv2.resize(cv_img, (1280, int(geom.height * scale)))
                    
                _, jpeg = cv2.imencode(".jpg", cv_img, [cv2.IMWRITE_JPEG_QUALITY, 70])
                return jpeg.tobytes()
        except Exception:
            pass
        return None

    def _real_worker(self):
        """Capture RTSP stream and convert to MJPEG, fallback to RViz."""
        try:
            import cv2
            rtsp_url = "rtsp://192.168.3.51:554/live/0"

            while self._real_running:
                cap = cv2.VideoCapture(rtsp_url)
                if not cap.isOpened():
                    print(f"[CameraStreamer] RTSP connection failed: {rtsp_url}")
                    print("[CameraStreamer] Fallback: Searching for RViz window...")
                    
                    # Try RViz streaming for 5 seconds, then check RTSP again
                    start_time = time.time()
                    rviz_win = self._get_rviz_window()
                    
                    while self._real_running and (time.time() - start_time) < 5:
                        # Geriye dönük pencere kontrolü
                        if not rviz_win:
                            rviz_win = self._get_rviz_window()
                            
                        rviz_frame = self._try_rviz_capture(rviz_win) if rviz_win else None
                        
                        if rviz_frame:
                            with self._real_lock:
                                self._real_frame = rviz_frame
                            time.sleep(0.05)  # ~20 FPS
                        else:
                            time.sleep(1.0)
                    continue

                print("[CameraStreamer] Real camera connected.")
                while self._real_running:
                    ret, frame = cap.read()
                    if not ret:
                        print("[CameraStreamer] Failed to capture RTSP frame, reconnecting...")
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
# Elasticsearch Proxy (read-only) — powers the Data Analytics tab
# ==============================================================================
# These routes ONLY issue read queries (_search) against the local Elasticsearch.
# Nothing is ever written, updated, deleted, or remapped — the existing
# ROS2 → Kafka → Elasticsearch → MariaDB → Grafana pipeline is untouched.

ES_BASE_URL = os.environ.get("ES_URL", "http://localhost:9200")


def _es_search(index, body, timeout=20):
    """Issue a read-only _search against Elasticsearch and return parsed JSON."""
    url = f"{ES_BASE_URL}/{index}/_search"
    data = json.dumps(body).encode("utf-8")
    req = urllib.request.Request(
        url, data=data,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        return json.loads(resp.read().decode("utf-8"))


def _parse_time_param(value):
    """Accept epoch milliseconds, epoch seconds, or an ISO string. Return as-is
    for ES (which understands ISO + epoch_millis). Numbers are treated as ms."""
    if value is None or value == "":
        return None
    try:
        # Pure number → epoch millis
        return int(float(value))
    except (TypeError, ValueError):
        return value  # ISO string, let ES parse it


@app.route("/api/es/range")
def es_range():
    """Return the min/max timestamp available in an index (for 'fit to data')."""
    index = request.args.get("index", "")
    time_field = request.args.get("time_field", "@timestamp")
    if not index:
        return jsonify({"error": "index required"}), 400
    body = {
        "size": 0,
        "aggs": {
            "mn": {"min": {"field": time_field}},
            "mx": {"max": {"field": time_field}},
        },
    }
    try:
        res = _es_search(index, body)
        agg = res.get("aggregations", {})
        return jsonify({
            "min": agg.get("mn", {}).get("value"),
            "max": agg.get("mx", {}).get("value"),
            "min_str": agg.get("mn", {}).get("value_as_string"),
            "max_str": agg.get("mx", {}).get("value_as_string"),
        })
    except (urllib.error.URLError, urllib.error.HTTPError, ValueError) as e:
        return jsonify({"error": str(e)}), 502


@app.route("/api/es/timeseries")
def es_timeseries():
    """Downsampled time-series via a date_histogram with per-field averages.

    Query params:
      index       ES index name
      fields      comma-separated numeric field paths (e.g. ur10e_elbow_joint.position)
      time_field  date field to bucket on (default @timestamp)
      from, to    range bounds (epoch ms or ISO); omit for all data
      points      target number of buckets (default 500)
    """
    index = request.args.get("index", "")
    fields = [f for f in request.args.get("fields", "").split(",") if f]
    time_field = request.args.get("time_field", "@timestamp")
    # Upper bound stays under Elasticsearch's default search.max_buckets (65536)
    # so the date_histogram can't blow the bucket limit and error out.
    points = max(10, min(60000, int(request.args.get("points", 500))))
    frm = _parse_time_param(request.args.get("from"))
    to = _parse_time_param(request.args.get("to"))

    if not index or not fields:
        return jsonify({"error": "index and fields required"}), 400

    # Build the range filter. We always apply a lower floor (TIME_FLOOR) to drop
    # implausible pre-2000 timestamps. The sim joint-state stream carries Gazebo
    # sim-clock stamps that start at 0 on each launch, so those docs land in 1970.
    # Left unfiltered they stretch the @timestamp span to ~56 years, which forces
    # the date_histogram into huge buckets and flattens the whole chart into a few
    # averaged points. Filtering them here (read-only, query-time) restores detail
    # without touching any stored data.
    TIME_FLOOR = "2000-01-01"
    rng = {"gte": frm if frm is not None else TIME_FLOOR}
    if to is not None:
        rng["lte"] = to
    query = {"range": {time_field: rng}}

    # Determine the bucket interval. If we have explicit numeric ms bounds use
    # them; otherwise ask ES for the actual min/max so buckets fit the data.
    try:
        if isinstance(frm, int) and isinstance(to, int):
            span_ms = max(1, to - frm)
        else:
            r = _es_search(index, {
                "size": 0,
                "query": query,
                "aggs": {
                    "mn": {"min": {"field": time_field}},
                    "mx": {"max": {"field": time_field}},
                },
            })
            agg = r.get("aggregations", {})
            mn = agg.get("mn", {}).get("value")
            mx = agg.get("mx", {}).get("value")
            if mn is None or mx is None:
                return jsonify({"time": [], "series": {}})
            span_ms = max(1, int(mx - mn))
    except (urllib.error.URLError, urllib.error.HTTPError, ValueError) as e:
        return jsonify({"error": str(e)}), 502

    interval_ms = max(1, span_ms // points)

    body = {
        "size": 0,
        "query": query,
        "aggs": {
            "ts": {
                "date_histogram": {
                    "field": time_field,
                    "fixed_interval": f"{interval_ms}ms",
                    "min_doc_count": 1,
                },
                "aggs": {
                    f"f{i}": {"avg": {"field": f}}
                    for i, f in enumerate(fields)
                },
            }
        },
    }

    try:
        res = _es_search(index, body)
    except (urllib.error.URLError, urllib.error.HTTPError, ValueError) as e:
        return jsonify({"error": str(e)}), 502

    buckets = res.get("aggregations", {}).get("ts", {}).get("buckets", [])
    out_time = []
    out_series = {f: [] for f in fields}
    for b in buckets:
        out_time.append(b["key"])  # epoch millis
        for i, f in enumerate(fields):
            val = b.get(f"f{i}", {}).get("value")
            out_series[f].append(val)

    return jsonify({"time": out_time, "series": out_series})


@app.route("/api/es/scatter3d")
def es_scatter3d():
    """Return up to `limit` raw x/y/z points for 3D scatter (e.g. TCP path).

    Query params:
      index            ES index name (default ros-tcp-pose-topic)
      x, y, z          field paths (default pose.position.{x,y,z})
      time_field       field used for range filter + ordering (default header.sec)
      time_unit        's' or 'ms' — unit of time_field values (default 's')
      from, to         range bounds in epoch ms; converted to time_unit
      limit            max points (default 3000)
    """
    index = request.args.get("index", "ros-tcp-pose-topic")
    fx = request.args.get("x", "pose.position.x")
    fy = request.args.get("y", "pose.position.y")
    fz = request.args.get("z", "pose.position.z")
    time_field = request.args.get("time_field", "header.sec")
    time_unit = request.args.get("time_unit", "s")
    limit = max(10, min(10000, int(request.args.get("limit", 3000))))
    frm = _parse_time_param(request.args.get("from"))
    to = _parse_time_param(request.args.get("to"))

    def to_unit(ms):
        return ms / 1000.0 if time_unit == "s" else ms

    if isinstance(frm, int) or isinstance(to, int):
        rng = {}
        if isinstance(frm, int):
            rng["gte"] = to_unit(frm)
        if isinstance(to, int):
            rng["lte"] = to_unit(to)
        query = {"range": {time_field: rng}}
    else:
        query = {"match_all": {}}

    body = {
        "size": limit,
        "_source": [fx, fy, fz, time_field],
        "query": query,
        "sort": [{time_field: {"order": "desc"}}],
    }

    try:
        res = _es_search(index, body)
    except (urllib.error.URLError, urllib.error.HTTPError, ValueError) as e:
        return jsonify({"error": str(e)}), 502

    def dig(src, path):
        cur = src
        for part in path.split("."):
            if isinstance(cur, dict) and part in cur:
                cur = cur[part]
            else:
                return None
        return cur

    xs, ys, zs = [], [], []
    for hit in res.get("hits", {}).get("hits", []):
        src = hit.get("_source", {})
        xv, yv, zv = dig(src, fx), dig(src, fy), dig(src, fz)
        if xv is None or yv is None or zv is None:
            continue
        xs.append(xv)
        ys.append(yv)
        zs.append(zv)

    # Returned newest-first; reverse so the path runs chronologically.
    return jsonify({"x": xs[::-1], "y": ys[::-1], "z": zs[::-1]})


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

@socketio.on("send_command")
def handle_send_command(data):
    scenario_mgr.publish_command((data or {}).get("text", ""))

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
    print("  Robotic Testbed as a Service")
    print("  ESOGÜ - IFARLAB — Control Dashboard")
    print("  http://localhost:8080")
    print("=" * 60)

    # Start camera streamers
    camera_streamer.start_gazebo_stream()
    camera_streamer.start_real_stream()

    # Start joint state collector
    joint_collector.start()

    # Start anomaly detection collector
    anomaly_collector.start()

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
