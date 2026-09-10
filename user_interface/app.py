#!/usr/bin/env python3
"""
Robotic Testbed as a Service — ESOGÜ IFARLAB Control Dashboard
Flask + SocketIO backend for managing ROS2 launch scenarios, camera streaming,
real-time joint state monitoring, and UR10e anomaly detection.
"""

import os
import re
import struct
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
# Workspace bootstrap — must run before anything imports a workspace ROS package
# ==============================================================================
# Bu dosya <workspace>/src/user_interface/app.py konumunda olduğu için workspace
# kökünü dosya yolundan türetiyoruz (makineye özel mutlak yol yazmamak için).
WORKSPACE_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
WORKSPACE_SETUP = os.path.join(WORKSPACE_ROOT, "install", "setup.bash")

def _bootstrap_workspace_env():
    """Make every colcon package under install/ importable (sys.path) and
    resolvable via ament_index (AMENT_PREFIX_PATH), regardless of whether the
    shell that launched `python3 app.py` sourced install/setup.bash.

    Everything else in this file only ever needed rclpy/sensor_msgs, which ship
    with the system ROS install and work with no workspace sourcing at all -- so
    this went unnoticed until Free Move, the first feature to need a WORKSPACE
    package (pymoveit2_real) and workspace ament packages (the *_moveit_config
    ones, mobile_manipulator_description for meshes). Without this, starting
    app.py from a plain terminal (ROS sourced, workspace not) makes every
    FreeMoveArm silently fail with "No module named 'pymoveit2_real'" and every
    Kawasaki mesh 404 from get_package_share_directory() failing to find its
    package -- both looked like application bugs but were really just missing
    environment, and it is not reasonable to expect that to be remembered by
    hand on every terminal that starts this dashboard.
    """
    import glob

    install_dir = os.path.join(WORKSPACE_ROOT, "install")
    if not os.path.isdir(install_dir):
        return

    # sys.path: every package's Python install location, ament_python
    # ("lib/python3.X/site-packages") and setuptools/ament_cmake_python
    # ("local/lib/python3.X/dist-packages") alike.
    py_globs = [
        os.path.join(install_dir, "*", "lib", "python3*", "site-packages"),
        os.path.join(install_dir, "*", "local", "lib", "python3*", "dist-packages"),
    ]
    for pattern in py_globs:
        for path in glob.glob(pattern):
            if path not in sys.path:
                sys.path.insert(0, path)

    # AMENT_PREFIX_PATH: every package directory that has its own ament_index
    # resource marker is a valid prefix (this is exactly what install/setup.bash
    # aggregates); get_package_share_directory() and friends read this.
    prefixes = set(
        filter(None, os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep))
    )
    marker_glob = os.path.join(install_dir, "*", "share", "ament_index", "resource_index", "packages")
    for marker_dir in glob.glob(marker_glob):
        # marker_dir = install/<pkg>/share/ament_index/resource_index/packages
        pkg_prefix = marker_dir[: -len(os.path.join("share", "ament_index", "resource_index", "packages"))].rstrip(os.sep)
        prefixes.add(pkg_prefix)
    os.environ["AMENT_PREFIX_PATH"] = os.pathsep.join(sorted(prefixes))

_bootstrap_workspace_env()

import free_move

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


# ==============================================================================
# Use-Case Broadcaster (ROS2)
# ==============================================================================
# The dashboard is the only process that knows which scenario is running; the
# ROS2 → Kafka bridge is the only process that writes into the data pipeline.
# This node is the link between them. It publishes the active use-case name,
# and the bridge stamps every document it forwards with whatever it last heard
# (see ros2_kafka_bridge/src/double_ros2_kafka_bridge.py::safe_publish).
#
# TRANSIENT_LOCAL durability is NOT optional. The bridge is a long-lived process
# that can be (re)started at any moment, including in the middle of a run. With
# the default VOLATILE durability it would never see a value published before it
# subscribed, and would silently tag a whole run as IDLE.

USE_CASE_TOPIC = "/testbed/use_case"
USE_CASE_IDLE = "IDLE"

# The run identifier goes out on its OWN latched topic rather than being folded
# into the use-case payload. double_ros2_km_bridge.py also subscribes to
# /testbed/use_case and reads the message as a bare string; turning that payload
# into JSON would silently tag every one of its documents with the whole blob.
# A second topic is additive — a bridge that does not know about it is unchanged.
RUN_ID_TOPIC = "/testbed/run_id"


class UseCaseBroadcaster:
    """Publishes the active use case and run id on latched ROS2 topics.

    `use_case` says WHICH scenario produced a document; `run_id` says WHICH
    EXECUTION of it. Without the second one, the seventh pick-and-place run is
    indistinguishable from the first, and "compare yesterday's run with today's"
    — the question a testbed actually gets asked — cannot be expressed.
    """

    def __init__(self):
        self._pub = None
        self._run_pub = None
        self._msg_cls = None
        self._running = False
        self._current = USE_CASE_IDLE
        self._run_id = ""
        self._lock = threading.Lock()

    def start(self):
        self._running = True
        threading.Thread(target=self._ros_spin, daemon=True).start()

    def stop(self):
        self._running = False

    def _ros_spin(self):
        try:
            import rclpy
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.node import Node as RosNode
            from rclpy.qos import (QoSProfile, QoSDurabilityPolicy,
                                   QoSReliabilityPolicy)
            from std_msgs.msg import String as RosString

            # Guarded like the other collectors: whoever gets here first
            # initialises the shared context.
            if not rclpy.ok():
                rclpy.init()
            node = RosNode("dashboard_use_case_publisher")

            qos = QoSProfile(depth=1)
            qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
            qos.reliability = QoSReliabilityPolicy.RELIABLE

            with self._lock:
                self._msg_cls = RosString
                self._pub = node.create_publisher(RosString, USE_CASE_TOPIC, qos)
                self._run_pub = node.create_publisher(RosString, RUN_ID_TOPIC, qos)
                pending = self._current
                pending_run = self._run_id

            # Publish immediately: set() may have been called before the node
            # existed (the page accepts clicks from the moment it loads).
            self._publish_now(pending, pending_run)
            node.get_logger().info(
                f"Use-case broadcaster up on {USE_CASE_TOPIC} and {RUN_ID_TOPIC} "
                f"(latched), current = {pending} / run = {pending_run or '-'}")

            # A DEDICATED executor, for the same reason as AnomalyCollector:
            # sharing the global one makes nodes vanish from the graph silently.
            executor = SingleThreadedExecutor()
            executor.add_node(node)
            try:
                while self._running and rclpy.ok():
                    executor.spin_once(timeout_sec=0.2)
            finally:
                executor.remove_node(node)
                node.destroy_node()

        except Exception as e:
            print(f"[UseCaseBroadcaster] ROS2 error: {e}")
            print("[UseCaseBroadcaster] Collected data will NOT be tagged.")

    def _publish_now(self, name, run_id):
        with self._lock:
            pub, run_pub, cls = self._pub, self._run_pub, self._msg_cls
        if pub is None or cls is None:
            return False
        try:
            pub.publish(cls(data=name))
            if run_pub is not None:
                run_pub.publish(cls(data=run_id))
            return True
        except Exception as e:
            print(f"[UseCaseBroadcaster] publish failed: {e}")
            return False

    @staticmethod
    def _mint_run_id(name):
        """A run id that is readable, sortable and unique.

        Readable because it is read straight off a chart legend; sortable so a
        terms aggregation lists runs in the order they happened; and with a
        random tail because two runs of the same scenario can start inside the
        same second.
        """
        stamp = datetime.utcnow().strftime("%Y%m%dT%H%M%SZ")
        return f"{name}-{stamp}-{os.urandom(2).hex()}"

    def set(self, name):
        """Set the active use case, minting a run id when a run begins.

        Safe to call before the ROS node exists: the values are remembered and
        published as soon as the publishers come up.
        """
        name = (str(name or "").strip() or USE_CASE_IDLE)
        with self._lock:
            changed = name != self._current
            self._current = name
            if name == USE_CASE_IDLE:
                # Nothing is running, so nothing should be tagged with a run.
                self._run_id = ""
            elif changed or not self._run_id:
                self._run_id = self._mint_run_id(name)
            run_id = self._run_id
        if self._publish_now(name, run_id):
            print(f"[UseCaseBroadcaster] USE_CASE = {name} RUN_ID = {run_id or '-'}")

    @property
    def current(self):
        with self._lock:
            return self._current

    @property
    def current_run_id(self):
        with self._lock:
            return self._run_id


use_case_pub = UseCaseBroadcaster()


class ScenarioManager:
    """Manages ROS2 launch process lifecycle."""

    # Scenario definitions: name → (hil_params, scenario_launch_cmd)
    SCENARIOS = {
        "multi_robot_inspection": {
            "label": "Multi-Robot Inspection Scenario",
            # Stamped onto every document the ROS2 → Kafka bridge forwards.
            "use_case": "MULTIROBOT_INSPECTION",
            "hil_params": "",
            "scenario_cmd": "ros2 launch multirobot_viewpoint_planner multirobot_inspection.launch.py",
            # use_fake_hardware açıkken only_sim:=true, gerçek robot bağlıyken only_sim:=false
            "sim_flag": "only_sim",
            # multirobot_inspection.launch.py also starts multirobot_viewpoint_visualizer,
            # a PERMANENT node with no on_exit handler registered -- so `ros2 launch`
            # itself never terminates once the tour is actually done (see memory
            # "inspection-launch-never-exits"). Watch the one-shot executor processes
            # (ur_inspection_node / kawasaki_inspection_node) inside the launch's own
            # process group instead: see _watch_executor_completion().
            "executor_pattern": "inspection_node",
        },
        "ur10e_inspection": {
            "label": "UR10e Inspection Scenario",
            # Stamped onto every document the ROS2 → Kafka bridge forwards.
            "use_case": "UR10E_INSPECTION",
            "hil_params": "",
            "scenario_cmd": "ros2 launch viewpoint_planner inspection_execution.launch.py",
            "sim_flag": "only_sim",
            # Same story as multi_robot_inspection above: inspection_execution.launch.py
            # also starts a permanent viewpoint_visualizer node. Pattern is more specific
            # than "inspection_node" on purpose -- the executable here is
            # "inspection_executor_node", which "inspection_node" would NOT match as a
            # substring (unlike ur_inspection_node/kawasaki_inspection_node above).
            "executor_pattern": "inspection_executor_node",
        },
        "pick_and_place": {
            "label": "Pick & Place Scenario",
            # Stamped onto every document the ROS2 → Kafka bridge forwards.
            "use_case": "PICKPLACE",
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
            # Stamped onto every document the ROS2 → Kafka bridge forwards.
            "use_case": "HRC",
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
        # Free Move (drag-to-jog tab) ve senaryolar HIL'i birbirinden bağımsız iki
        # farklı şekilde ayağa kaldırıyor - ikisi aynı anda açık olamaz.
        self.free_move_active = False
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

        if source_name != "SENARYO":
            return
        # human_robot_collaboration exits its OWN process cleanly (on_exit=Shutdown
        # in its launch file), so this EOF-based path is enough for it. But
        # multi_robot_inspection / ur10e_inspection each also start a PERMANENT
        # visualizer node with no on_exit handler -- `ros2 launch` for those never
        # exits on its own even once the actual work is done (see memory
        # "inspection-launch-never-exits"), so process.wait() below just blocks
        # forever for them; confirm_robot_ready() spawns a SEPARATE
        # _watch_executor_completion() watcher for those instead. Both paths funnel
        # into the same _mark_natural_completion() and are safe to run concurrently
        # (whichever notices first wins; the other's guard is then a no-op).
        process.wait()
        self._mark_natural_completion(process)

    def _mark_natural_completion(self, process):
        """Flip scenario_status back to "stopped" once `process` (the SENARYO
        subprocess) is confirmed done, WITHOUT touching current_scenario or
        hil_status: HIL is still up and homed, so a "Run Again" click can relaunch
        the same scenario_cmd immediately via confirm_robot_ready() instead of a full
        HIL restart. Called from both _stream_output (EOF on the launch process
        itself) and _watch_executor_completion (the one-shot executor node(s) inside
        it have all exited) -- see the comment in _stream_output for why a launch
        exit alone is not always a valid completion signal."""
        with self._lock:
            # Only treat this as a natural completion if nothing has already
            # superseded it -- a manual Stop or a fresh Start both flip
            # scenario_status away from "running" (to "stopping") before killing the
            # old process, so this guard can't fire for either of those.
            if self.scenario_process is not process or self.scenario_status != "running":
                return
            self.scenario_status = "stopped"
            label = self.SCENARIOS.get(self.current_scenario, {}).get(
                "label", self.current_scenario)
            returncode = process.poll()
            if returncode in (0, None):
                self._emit_log("SYSTEM", f"✅ {label} finished on its own.")
            else:
                self._emit_log("SYSTEM", f"⚠️ {label} exited (code {returncode}).")
            self._emit_status()

    def _watch_executor_completion(self, process, pattern):
        """For scenarios whose launch file has a permanent side node and no on_exit
        handler (see the "executor_pattern" comments on SCENARIOS and memory
        "inspection-launch-never-exits") -- `ros2 launch` itself never terminates
        once the actual work is done, so _stream_output's EOF-based detection never
        fires. Poll the ACTUAL one-shot executor process(es) inside the launch's
        process group instead: once they have appeared and then all disappeared, the
        run is over, independent of the launch wrapper or the permanent visualizer.
        Mirrors doors_inspection/doors_mission_node.py's _await_tour_end(), the
        existing working pattern for this exact problem in this workspace."""
        try:
            pgid = os.getpgid(process.pid)
        except (ProcessLookupError, OSError):
            return
        seen = False
        # If the executors never even appear (launch itself failed to come up),
        # don't watch forever -- give up and let a manual Stop or the EOF-based path
        # (if the launch process happens to die outright) handle it instead.
        grace_deadline = time.monotonic() + 90.0
        while True:
            if process.poll() is not None:
                return  # launch process itself exited; _stream_output's path handles it
            with self._lock:
                if self.scenario_process is not process:
                    return  # superseded by a manual stop/restart
            try:
                out = subprocess.run(
                    ["pgrep", "-g", str(pgid), "-f", pattern],
                    capture_output=True, text=True, timeout=5).stdout
            except (subprocess.SubprocessError, OSError):
                out = ""
            pids = [p for p in out.split() if p]
            if pids:
                seen = True
            elif seen:
                time.sleep(1.0)  # let stdout flush the final lines before marking done
                self._mark_natural_completion(process)
                return
            elif time.monotonic() > grace_deadline:
                self._emit_log("SYSTEM",
                    f"⚠️ Executor process ('{pattern}') never appeared after 90s; "
                    "giving up on auto-detecting completion for this run.")
                return
            time.sleep(1.0)

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
            # Nothing is running any more: stop tagging collected data with the
            # scenario that just ended.
            use_case_pub.set(USE_CASE_IDLE)

            self._emit_status()
            self._emit_log("SYSTEM", "✅ All processes stopped.")

        # Outside the lock: FreeMoveArm.stop() joins its ROS threads (up to 5s each)
        # and must not hold up other status-affecting calls while it does.
        free_move.manager.stop()
        with self._lock:
            self.free_move_active = False
            self._emit_status()

    def start_scenario(self, scenario_key, use_fake_hardware=False, data_acquisition=False):
        """Start a scenario: stop existing → start HIL → wait for confirm → start scenario."""
        if scenario_key not in self.SCENARIOS:
            self._emit_log("SYSTEM", f"❌ Unknown scenario: {scenario_key}")
            return
        if self.free_move_active:
            self._emit_log("SYSTEM",
                "❌ Free Move is active. Disable it before starting a scenario.")
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
                # Deliberately IDLE, not scenario["use_case"]: HIL bring-up takes
                # 30-60 s of controller spawning and homing that is not part of
                # the scenario. Tagging it would prepend that noise to every run.
                use_case_pub.set(USE_CASE_IDLE)
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
                # data_acquisition:=false is the launch file's own default, so it
                # is only appended when the toggle is ON — same convention as
                # use_fake_hardware above.
                if data_acquisition:
                    hil_cmd += " data_acquisition:=true"

                self.hil_process = self._start_process(hil_cmd, "HIL")
                self.hil_status = "running"
                self._emit_status()

                self._emit_log("SYSTEM",
                    f"✅ HIL initialized: {scenario['label']}. "
                    "Please confirm that the robot is ready.")

        thread = threading.Thread(target=_run, daemon=True)
        thread.start()

    def start_free_move(self, use_fake_hardware=False):
        """Bring up HIL alone (no scripted scenario) and start both FreeMoveArm
        controllers. Call on its own thread — waiting for the arms to come up can
        take up to ~60s.
        """
        with self._lock:
            if self.current_scenario is not None or self.scenario_status == "running":
                self._emit_log("SYSTEM",
                    "❌ A scenario is active. Stop it before enabling Free Move.")
                return
            if self.hil_process or self.scenario_process:
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
                time.sleep(2)

            use_case_pub.set(USE_CASE_IDLE)
            self.current_scenario = None
            self.free_move_active = True
            self.use_fake_hardware = use_fake_hardware
            self.hil_status = "starting"
            self._emit_status()

            hil_cmd = self.HIL_BASE_CMD
            if use_fake_hardware:
                hil_cmd += " use_fake_hardware:=true use_mock_hardware:=true fake_sensor_commands:=true"

            self.hil_process = self._start_process(hil_cmd, "HIL")
            self.hil_status = "running"
            self._emit_status()
            self._emit_log("SYSTEM",
                "✅ HIL initialized for Free Move. Waiting for both arms to come up...")

        # Outside the lock: this can take up to a minute and must not block
        # emergency-stop or status polling from other clients while it does.
        free_move.manager.start()

        # Both arms' _ros_spin threads start concurrently (manager.start() above),
        # so the two wait_ready() calls must run concurrently too -- calling them
        # back to back turned "wait up to 60s" into "wait up to 60+60=120s" whenever
        # the first one timed out, since the second one's clock didn't start until
        # the first returned.
        def _wait(arm, method, timeout, result):
            arm_obj = getattr(free_move.manager, arm)
            result[arm] = getattr(arm_obj, method)(timeout=timeout)

        ready = {}
        threads = [
            threading.Thread(target=_wait, args=("ur", "wait_ready", 60.0, ready), daemon=True),
            threading.Thread(target=_wait, args=("kawasaki", "wait_ready", 60.0, ready), daemon=True),
        ]
        for t in threads:
            t.start()
        for t in threads:
            t.join()
        ur_ready, kawa_ready = ready["ur"], ready["kawasaki"]

        if not ur_ready:
            err = free_move.manager.ur.last_error
            self._emit_log("SYSTEM",
                f"⚠️ Free Move: UR10e failed to initialize" + (f" ({err})" if err else " (timed out, no error captured)") + ".")
        if not kawa_ready:
            err = free_move.manager.kawasaki.last_error
            self._emit_log("SYSTEM",
                f"⚠️ Free Move: Kawasaki failed to initialize" + (f" ({err})" if err else " (timed out, no error captured)") + ".")

        # The node/executor being up does not mean a full joint state has arrived yet
        # -- /joint_states is split across several publishers (see free_move.py's
        # module docstring), so wait for every joint this arm needs before computing
        # FK, or the very first marker placement is wrong (all-zero/default config).
        js_ready = {}
        js_threads = []
        if ur_ready:
            js_threads.append(threading.Thread(
                target=_wait, args=("ur", "wait_joint_state", 20.0, js_ready), daemon=True))
        if kawa_ready:
            js_threads.append(threading.Thread(
                target=_wait, args=("kawasaki", "wait_joint_state", 20.0, js_ready), daemon=True))
        for t in js_threads:
            t.start()
        for t in js_threads:
            t.join()
        ur_js = js_ready.get("ur", False)
        kawa_js = js_ready.get("kawasaki", False)
        if ur_ready and not ur_js:
            self._emit_log("SYSTEM", "⚠️ Free Move: UR10e joint state incomplete after 20s.")
        if kawa_ready and not kawa_js:
            self._emit_log("SYSTEM", "⚠️ Free Move: Kawasaki joint state incomplete after 20s.")
        self._emit_log("SYSTEM", "✅ Free Move ready.")

        self.socketio.emit("freemove_ready", {
            "ur": {
                "ready": ur_ready,
                "pose": free_move.manager.ur.current_tcp_pose(),
                "params": free_move.manager.ur.get_params(),
                "planner_ids": freemove_planner_ids("real_ur10e"),
            },
            "kawasaki": {
                "ready": kawa_ready,
                "pose": free_move.manager.kawasaki.current_tcp_pose(),
                "params": free_move.manager.kawasaki.get_params(),
                "planner_ids": freemove_planner_ids("real_kawasaki"),
            },
        })

    def stop_free_move(self):
        """Disable Free Move and tear the cell back down to idle (mirrors stop_all,
        scoped to the free-move-only HIL bring-up). Call on its own thread."""
        with self._lock:
            if not self.free_move_active:
                return
            self._emit_log("SYSTEM", "🛑 Disabling Free Move...")

        free_move.manager.stop()

        with self._lock:
            self.free_move_active = False
            self.hil_status = "stopping"
            self._emit_status()
            self._kill_process(self.hil_process, "HIL")
            self.hil_process = None
            self.hil_status = "stopped"
            self.current_scenario = None
            self._emit_status()
            self._emit_log("SYSTEM", "✅ Free Move disabled.")

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

                # Tag BEFORE launching, so the first documents of the run are
                # already stamped. The topic is latched, so the bridge picks the
                # value up even if it happens to restart mid-run.
                use_case_pub.set(scenario.get("use_case", USE_CASE_IDLE))

                self.scenario_process = self._start_process(scenario_cmd, "SENARYO")
                self.scenario_status = "running"
                self._emit_status()

                self._emit_log("SYSTEM", f"🚀 {scenario['label']} is running!")

                # multi_robot_inspection / ur10e_inspection: the launch process
                # itself never exits (permanent visualizer, no on_exit handler --
                # see "executor_pattern" on SCENARIOS), so _stream_output's EOF-based
                # completion detection never fires for them. Watch the actual
                # executor node(s) instead.
                executor_pattern = scenario.get("executor_pattern")
                if executor_pattern:
                    watch_thread = threading.Thread(
                        target=self._watch_executor_completion,
                        args=(self.scenario_process, executor_pattern),
                        daemon=True)
                    watch_thread.start()

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
            "free_move_active": self.free_move_active,
            # Bu senaryo serbest metin komut alıyor mu (Send Command butonu)
            "command_prompt": bool(scenario.get("command_prompt")),
            # Toplanan veriye şu an basılan etiket
            "use_case": use_case_pub.current,
            "run_id": use_case_pub.current_run_id,
        }


# Initialize the scenario manager
scenario_mgr = ScenarioManager(socketio)

def _freemove_on_error(arm_name, message):
    """free_move.FreeMoveArm._ros_spin calls this on any init/runtime exception, so
    it reaches the dashboard's own log instead of only this process's stdout."""
    scenario_mgr._emit_log("FREEMOVE", f"❌ {arm_name}: {message}")

free_move.on_error = _freemove_on_error


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

    # Fallbacks only. The live thresholds arrive with every decision on
    # ~/detail (fused threshold at index 5) and are adopted as soon as the
    # first message lands. Hard-coding them was a real defect: the fused
    # threshold moved 18.0 -> 1.4 when the detector was recalibrated on the
    # cell, and the residual/raw thresholds moved 1.6008/3.4461 -> 1.3723/0.3934
    # with the v3 models. A UI that keeps the old constants draws the alarm
    # line in the wrong place and mislabels which model drove a decision.
    THR_FUSED = 1.4
    THR_RESIDUAL = 1.3723
    THR_RAW = 0.3934

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
                # Adopt the live threshold. It is regime-dependent on the
                # detector side (a static and a moving value), so the constant
                # above can never be right for both; whatever the node applied
                # to THIS decision is the only correct alarm line to draw.
                if rec["thr"] > 0:
                    self.THR_FUSED = rec["thr"]
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

# ==============================================================================
# Free Move — URDF + mesh serving for the browser 3D viewer
# ==============================================================================
# Read-only conveniences: xacro is the SAME whole-cell file
# hil_test_whole_unified.launch.py already flattens for robot_description, we just
# also do it here (as a static file the browser can fetch) and rewrite its mesh
# URIs so they resolve to plain HTTP instead. This xacro bakes meshes in as TWO
# different URI styles, both needing a rewrite: some are proper "package://pkg/..."
# refs, but most of the UR10e/environment meshes come out as raw "file:///abs/path"
# (this cell's macros resolve mesh paths with $(find pkg) or a workspace-relative
# property rather than writing a literal "package://" string) — without handling
# the second form, everything except the Kawasaki (which does use package://)
# silently fails to fetch and never renders.

FREE_MOVE_XACRO = os.path.join(
    WORKSPACE_ROOT, "src", "Universal_Robots_ROS2_Tutorials", "my_robot_cell",
    "my_robot_cell_control", "urdf", "whole_cell_hw.urdf.xacro")

# start_free_move() never passes use_vacuum_gripper/use_gripper/harmony, so
# hil_test_whole_unified.launch.py always resolves to this package for Free Move
# (its own declared defaults for those three args are all "false").
FREE_MOVE_OMPL_YAML = os.path.join(
    WORKSPACE_ROOT, "src", "real_ifarlab_moveit_config", "config", "ompl_planning.yaml")
_freemove_planner_cache = None

def freemove_planner_ids(group_name):
    """planner_configs list for `group_name` from ompl_planning.yaml, read directly
    instead of asking move_group for it: query_planner_interface was tried first but
    only ever surfaced one planner id on this MoveIt build, while the yaml itself lists
    twelve per group (measured 7 Sep 2026) -- reading the file is simply reliable."""
    global _freemove_planner_cache
    if _freemove_planner_cache is None:
        try:
            import yaml
            with open(FREE_MOVE_OMPL_YAML) as f:
                _freemove_planner_cache = yaml.safe_load(f) or {}
        except OSError as e:
            print(f"[freemove_planner_ids] could not read {FREE_MOVE_OMPL_YAML}: {e}")
            _freemove_planner_cache = {}
    group = _freemove_planner_cache.get(group_name) or {}
    return list(group.get("planner_configs", []))

_PACKAGE_URI_RE = re.compile(r'package://([A-Za-z0-9_]+)/')
_FILE_URI_RE = re.compile(r'file://(/+[^"]+)')
# ota_base.xacro writes "file://$(find pkg)/..." for a couple of meshes -- xacro's
# own $(find) substitution does not fire there (pre-existing quirk in that source
# file, reproduces with a plain `xacro` CLI run too), so it survives to here as a
# literal, unexpanded string. Fold it into the package:// form before that regex runs.
_UNEXPANDED_FIND_RE = re.compile(r'file://\$\(find ([A-Za-z0-9_]+)\)/')

def _rewrite_file_uri(match):
    """file:///abs/path -> localmesh/<relpath>, only when the path resolves under
    WORKSPACE_ROOT (both the source tree and the colcon install/ share dirs live
    there). Anything else is left as-is rather than opening a serve-any-file route."""
    raw_path = "/" + match.group(1).lstrip("/")  # file:////a/b -> /a/b
    real = os.path.realpath(raw_path)
    root = os.path.realpath(WORKSPACE_ROOT)
    if real == root or real.startswith(root + os.sep):
        return "localmesh/" + os.path.relpath(real, root)
    return match.group(0)

@app.route("/freemove/urdf")
def freemove_urdf():
    """Flatten whole_cell_hw.urdf.xacro and rewrite every mesh URI (package:// and
    file://) to an HTTP path so the browser can fetch every mesh."""
    try:
        result = subprocess.run(
            ["bash", "-c",
             f"source /opt/ros/humble/setup.bash && source {WORKSPACE_SETUP} && "
             f"xacro {shlex.quote(FREE_MOVE_XACRO)}"],
            capture_output=True, text=True, timeout=30,
        )
    except subprocess.TimeoutExpired:
        return jsonify({"error": "xacro timed out"}), 504

    if result.returncode != 0:
        return jsonify({"error": "xacro failed", "stderr": result.stdout[-4000:]}), 500

    # Relative to /freemove/urdf's own URL, NOT absolute: urdf-loader resolves any
    # non-"package://" mesh path as `workingPath + path` where workingPath is the
    # URDF URL's own directory ("/freemove/"). A leading "/" here would double up
    # into "//freemove/mesh/..." — a protocol-relative URL the browser mis-resolves.
    urdf_xml = _UNEXPANDED_FIND_RE.sub(r"package://\1/", result.stdout)
    urdf_xml = _PACKAGE_URI_RE.sub(r"mesh/\1/", urdf_xml)
    urdf_xml = _FILE_URI_RE.sub(_rewrite_file_uri, urdf_xml)
    return Response(urdf_xml, mimetype="application/xml")

@app.route("/freemove/mesh/<pkg>/<path:relpath>")
def freemove_mesh(pkg, relpath):
    """Serve one mesh file referenced by a package:// URI in the flattened URDF."""
    try:
        from ament_index_python.packages import get_package_share_directory
        share_dir = get_package_share_directory(pkg)
    except Exception:
        return jsonify({"error": f"unknown package: {pkg}"}), 404

    share_dir = os.path.realpath(share_dir)
    full_path = os.path.realpath(os.path.join(share_dir, relpath))
    if not (full_path == share_dir or full_path.startswith(share_dir + os.sep)):
        return jsonify({"error": "invalid path"}), 400
    if not os.path.isfile(full_path):
        return jsonify({"error": "not found"}), 404

    directory, filename = os.path.split(full_path)
    return send_from_directory(directory, filename)

@app.route("/freemove/localmesh/<path:relpath>")
def freemove_localmesh(relpath):
    """Serve one mesh file referenced by a file:// URI in the flattened URDF,
    scoped to files under the workspace root (see _rewrite_file_uri)."""
    root = os.path.realpath(WORKSPACE_ROOT)
    full_path = os.path.realpath(os.path.join(root, relpath))
    if not (full_path == root or full_path.startswith(root + os.sep)):
        return jsonify({"error": "invalid path"}), 400
    if not os.path.isfile(full_path):
        return jsonify({"error": "not found"}), 404

    directory, filename = os.path.split(full_path)
    return send_from_directory(directory, filename)

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
# These routes ONLY issue read queries (_search, _mapping, _cat) against the
# local Elasticsearch. Nothing is ever written, updated, deleted, or remapped —
# the existing ROS2 → Kafka → Elasticsearch → MariaDB → Grafana pipeline is
# untouched.

ES_BASE_URL = os.environ.get("ES_URL", "http://localhost:9200")

# 2000-01-01 in epoch milliseconds.
#
# The sim joint-state stream carries Gazebo sim-clock stamps that restart at 0
# on every launch, so those documents land in 1970. Left unfiltered they stretch
# the @timestamp span to ~56 years, which forces the date_histogram into huge
# buckets and flattens every chart into a handful of averaged points. Filtering
# them at query time (read-only) restores the detail without touching stored
# data.
TIME_FLOOR_MS = 946684800000

# The four scenarios the dashboard can tag data with, plus IDLE. Used as the
# fallback for the use-case filter when Elasticsearch is unreachable or the
# index has no tagged documents yet.
KNOWN_USE_CASES = [s["use_case"] for s in ScenarioManager.SCENARIOS.values()
                   if s.get("use_case")] + [USE_CASE_IDLE]


# Candidate time fields, tried in this order when the caller does not name one.
#
# The Kafka -> Elasticsearch ingest only stamps `@timestamp` on some topics; the
# rest carry nothing but the ROS header stamp, under one of several paths. The
# UI used to hardcode "@timestamp unless the index is ros-tcp-pose-topic", which
# left every other unstamped index unusable. `unit` is the unit the field is
# STORED in ("ms" for a date/epoch-millis field, "s" for an epoch-seconds one).
TIME_FIELD_CANDIDATES = [
    ("@timestamp", "ms"),
    ("header.sec", "s"),
    ("header.stamp.sec", "s"),
    ("timestamp", "s"),                 # ur-rtde-data: float epoch seconds
    ("ros_time.sec", "s"),
    ("kafka_metadata.timestamp", "ms"),
]

_TIME_FIELD_CACHE = {}       # index -> (fetched_at, resolution dict)
_TIME_FIELD_TTL = 300.0


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


def _es_get(path, timeout=20):
    """Issue a read-only GET against Elasticsearch and return parsed JSON."""
    req = urllib.request.Request(f"{ES_BASE_URL}/{path}", method="GET")
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        return json.loads(resp.read().decode("utf-8"))


ES_ERRORS = (urllib.error.URLError, urllib.error.HTTPError, ValueError, KeyError)


class QueryParamError(ValueError):
    """A request the user can fix — a malformed DSL fragment, a bad bound.

    Raised while the query is assembled, which happens before each endpoint's
    own try/except, so it is turned into a 400 by an app-level handler rather
    than escaping as a 500.
    """


@app.errorhandler(QueryParamError)
def _on_query_param_error(e):
    return jsonify({"error": str(e)}), 400


def _es_fail(e):
    """Uniform 502 body for an Elasticsearch failure."""
    return jsonify({"error": f"{type(e).__name__}: {e}"}), 502


def _parse_time_param(value):
    """Accept epoch milliseconds, epoch seconds, or an ISO string. Numbers are
    treated as epoch millis; strings are passed through for ES to parse."""
    if value is None or value == "":
        return None
    try:
        return int(float(value))
    except (TypeError, ValueError):
        return value  # ISO string, let ES parse it


# ------------------------------------------------------------------------------
# Field mapping
# ------------------------------------------------------------------------------

_MAPPING_CACHE = {}          # index → (fetched_at, [{path, type}])
_MAPPING_TTL = 60.0


def _flatten_mapping(props, prefix=""):
    """Flatten an ES mapping tree into a list of {path, type} entries."""
    out = []
    for name, spec in (props or {}).items():
        path = prefix + name
        if "properties" in spec:
            out.extend(_flatten_mapping(spec["properties"], path + "."))
            continue
        out.append({"path": path, "type": spec.get("type", "object")})
        # Multi-fields: a dynamically mapped string is `text` with a `.keyword`
        # subfield, and only the subfield is aggregatable.
        for sub, sspec in (spec.get("fields") or {}).items():
            out.append({"path": f"{path}.{sub}", "type": sspec.get("type", "keyword")})
    return out


def _index_fields(index):
    """Return (and cache) the flattened field list for an index pattern."""
    now = time.time()
    hit = _MAPPING_CACHE.get(index)
    if hit and now - hit[0] < _MAPPING_TTL:
        return hit[1]
    try:
        res = _es_get(f"{index}/_mapping")
    except ES_ERRORS:
        return hit[1] if hit else []
    fields = {}
    # An index pattern can resolve to several indices; merge their mappings.
    for spec in res.values():
        for f in _flatten_mapping(spec.get("mappings", {}).get("properties", {})):
            fields[f["path"]] = f["type"]
    flat = sorted(({"path": p, "type": t} for p, t in fields.items()),
                  key=lambda f: f["path"])
    _MAPPING_CACHE[index] = (now, flat)
    return flat


def _aggregatable(index, field):
    """Return the form of `field` usable in a term/terms query or aggregation.

    A dynamically mapped string lands as `text` with a `.keyword` subfield, and
    aggregating or term-matching on the `text` form silently returns nothing (or
    errors out, fielddata being disabled). This resolves `use_case` to
    `use_case.keyword` automatically, so callers never have to know how the
    field happened to be mapped.
    """
    if not field:
        return field
    types = {f["path"]: f["type"] for f in _index_fields(index)}
    if types.get(field) == "text" and f"{field}.keyword" in types:
        return f"{field}.keyword"
    return field


# ------------------------------------------------------------------------------
# Time-field resolution
# ------------------------------------------------------------------------------

def _resolve_time_field(index, force=False):
    """Pick a time field that actually carries usable values in `index`.

    Scoring, rather than "first candidate that exists", because existence alone
    picks the wrong field: ros-sim-pointcloud2 has `header.sec` on 37 k documents
    that all sit at the 1970 sim-clock origin, and `header.stamp.sec` on 309 k
    that span the real range. So each candidate is counted ABOVE the time floor,
    and a genuine `date` field wins outright when it has any such documents —
    that is the field the ingest intended, the rest are fallbacks for the topics
    it never stamped.

    Returns {time_field, time_unit, type, docs, usable, candidates[]}. `usable`
    is False when no candidate holds a document above the floor (ur-rtde-data's
    sibling ros-interactive-marker-update-topic stores empty documents), so the
    UI can say so instead of drawing a silently empty chart.
    """
    now = time.time()
    hit = _TIME_FIELD_CACHE.get(index)
    if hit and not force and now - hit[0] < _TIME_FIELD_TTL:
        return hit[1]

    types = {f["path"]: f["type"] for f in _index_fields(index)}
    cands = [(p, u) for p, u in TIME_FIELD_CANDIDATES if p in types]

    out = {"time_field": None, "time_unit": "ms", "type": None,
           "docs": 0, "usable": False, "candidates": []}

    if cands:
        aggs = {}
        for i, (path, unit) in enumerate(cands):
            floor = TIME_FLOOR_MS if unit == "ms" else TIME_FLOOR_MS // 1000
            aggs[f"c{i}"] = {"filter": {"range": {path: {"gte": floor}}}}
        try:
            res = _es_search(index, {"size": 0, "aggs": aggs})
        except ES_ERRORS:
            return hit[1] if hit else out
        agg = res.get("aggregations", {})
        for i, (path, unit) in enumerate(cands):
            out["candidates"].append({
                "path": path, "unit": unit, "type": types.get(path),
                "docs": agg.get(f"c{i}", {}).get("doc_count", 0),
            })

        dates = [c for c in out["candidates"]
                 if c["type"] == "date" and c["docs"] > 0]
        pool = dates or [c for c in out["candidates"] if c["docs"] > 0]
        if pool:
            best = max(pool, key=lambda c: c["docs"])
            out.update({"time_field": best["path"], "time_unit": best["unit"],
                        "type": best["type"], "docs": best["docs"],
                        "usable": True})

    _TIME_FIELD_CACHE[index] = (now, out)
    return out


def _resolved_or(index, time_field, time_unit):
    """Fill in whichever of (time_field, time_unit) the caller left blank."""
    if time_field and time_unit:
        return time_field, time_unit
    if not index:
        return time_field or "@timestamp", time_unit or "ms"
    r = _resolve_time_field(index)
    return (time_field or r["time_field"] or "@timestamp",
            time_unit or r["time_unit"] or "ms")


# ------------------------------------------------------------------------------
# Query building — shared by every endpoint below
# ------------------------------------------------------------------------------

def _time_range_clause(time_field, frm, to, time_unit="ms"):
    """Range clause on the time field, in the unit that field is stored in.

    `header.sec` holds epoch SECONDS while `@timestamp` is a date field; the UI
    always sends epoch milliseconds, so numeric bounds are converted here.

    The result must stay an int. A true division leaves a float, which json
    serializes in scientific notation (946684800000.0 -> 9.466848E11), and a
    `date` field rejects that: `epoch_millis` parses integers only, so every
    @timestamp-based query came back as a 400 and every chart drew empty.
    """
    div = 1000 if time_unit == "s" else 1

    def conv(v):
        if isinstance(v, bool) or not isinstance(v, (int, float)):
            return v
        return int(v) // div

    rng = {"gte": conv(frm) if frm is not None else conv(TIME_FLOOR_MS)}
    if to is not None:
        rng["lte"] = conv(to)
    return {"range": {time_field: rng}}


def _filter_clause(index, f):
    """Translate one UI filter into (es_clause, negate).

    Filters arrive as {"field": ..., "op": ..., "value": ...}; `op` is one of
    is / is_not / one_of / not_one_of / gt / gte / lt / lte / between /
    exists / missing / contains.
    """
    field = (f or {}).get("field")
    if not field:
        return None
    op = (f.get("op") or "is").strip()
    val = f.get("value")

    if op == "exists":
        return ({"exists": {"field": field}}, False)
    if op == "missing":
        return ({"exists": {"field": field}}, True)
    if op in ("is", "is_not"):
        return ({"term": {_aggregatable(index, field): val}}, op == "is_not")
    if op in ("one_of", "not_one_of"):
        vals = val if isinstance(val, list) else [val]
        vals = [v for v in vals if v is not None and v != ""]
        if not vals:
            return None
        return ({"terms": {_aggregatable(index, field): vals}}, op == "not_one_of")
    if op in ("gt", "gte", "lt", "lte"):
        if val is None or val == "":
            return None
        return ({"range": {field: {op: val}}}, False)
    if op == "between":
        lo, hi = (list(val) + [None, None])[:2] if isinstance(val, (list, tuple)) \
            else (None, None)
        rng = {}
        if lo is not None and lo != "":
            rng["gte"] = lo
        if hi is not None and hi != "":
            rng["lte"] = hi
        return ({"range": {field: rng}}, False) if rng else None
    if op == "contains":
        return ({"match_phrase": {field: val}}, False)
    return None


def _request_filters():
    """Parse the `filters` query parameter (URL-encoded JSON array)."""
    raw = request.args.get("filters", "")
    if not raw:
        return []
    try:
        parsed = json.loads(raw)
    except ValueError:
        return []
    return parsed if isinstance(parsed, list) else []


# Aggregation and scripting entry points are refused in a hand-written fragment.
# Not because they could write — `_search` cannot — but because they would let a
# pasted snippet run arbitrary Painless or replace the endpoint's own aggregation.
_DSL_FORBIDDEN = {"script", "script_score", "aggs", "aggregations", "scripted_metric"}


def _dsl_keys(node):
    """Every object key appearing anywhere inside a parsed DSL fragment."""
    out, stack = set(), [node]
    while stack:
        cur = stack.pop()
        if isinstance(cur, dict):
            out.update(cur.keys())
            stack.extend(cur.values())
        elif isinstance(cur, list):
            stack.extend(cur)
    return out


# `field > 5` — the spaced comparison Kibana's KQL accepts, and the first thing
# anyone types into a query bar. Plain Lucene has no such form: it parses the
# pieces as loose terms and, with lenient matching on, returns nothing at all.
# Silently zero is the worst possible answer, so the form is rewritten to the
# Lucene equivalent (`field:>5`) before the query is built.
_CMP_RE = re.compile(
    r'(?<![\w.:@"])([A-Za-z_@][\w.@-]*)\s*(>=|<=|>|<)\s*(-?\d+(?:\.\d+)?)')


def _rewrite_comparisons(q):
    """Apply the KQL-style comparison rewrite outside of quoted text."""
    parts = q.split('"')
    for i in range(0, len(parts), 2):        # even chunks sit outside quotes
        parts[i] = _CMP_RE.sub(r"\1:\2\3", parts[i])
    return '"'.join(parts)


def _query_string_clause(q):
    """The one-line query bar (Lucene + KQL comparisons) -> a query_string clause."""
    q = (q or "").strip()
    if not q:
        return None
    return {"query_string": {"query": _rewrite_comparisons(q),
                             "analyze_wildcard": True,
                             "default_field": "*", "lenient": True}}


def _raw_dsl_clause(raw):
    """A hand-written DSL fragment from the query bar.

    Read-only survives structurally: whatever comes back here is only ever
    placed inside the `filter` array of a `_search` body, so it can narrow the
    result set and do nothing else. Both `{"query": {...}}` (what Kibana and the
    ES docs show) and a bare clause are accepted, so a snippet can be pasted
    without reshaping it first.
    """
    raw = (raw or "").strip()
    if not raw:
        return None
    try:
        parsed = json.loads(raw)
    except ValueError as e:
        raise QueryParamError(f"invalid JSON — {e}")
    if not isinstance(parsed, dict) or not parsed:
        raise QueryParamError("expected a JSON object holding one query clause")
    if set(parsed) == {"query"} and isinstance(parsed["query"], dict):
        parsed = parsed["query"]
    bad = _DSL_FORBIDDEN.intersection(_dsl_keys(parsed))
    if bad:
        raise QueryParamError(
            f"not allowed in a filter fragment: {', '.join(sorted(bad))}")
    return parsed


def _build_query(index, time_field, frm, to, filters, time_unit="ms",
                 q=None, dsl=None):
    """Assemble the bool query every endpoint shares.

    time range + filter pills + query bar (`q`) + raw DSL fragment (`dsl`),
    all AND-ed together inside `bool.filter`.
    """
    must = [_time_range_clause(time_field, frm, to, time_unit)]
    must_not = []
    for f in filters or []:
        parsed = _filter_clause(index, f)
        if not parsed:
            continue
        clause, negate = parsed
        (must_not if negate else must).append(clause)
    qs = _query_string_clause(q)
    if qs:
        must.append(qs)
    raw = _raw_dsl_clause(dsl)      # raises ValueError on a bad fragment
    if raw:
        must.append(raw)
    bool_q = {"filter": must}
    if must_not:
        bool_q["must_not"] = must_not
    return {"bool": bool_q}


def _common_params(default_index="", default_time_field=None):
    """Pull the parameters every endpoint accepts off the request.

    A caller that does not name a time field gets the one `_resolve_time_field`
    finds for the index, instead of an assumed `@timestamp` that most of the
    ros-* indices do not carry.
    """
    index = request.args.get("index", default_index)
    time_field = request.args.get("time_field") or default_time_field
    time_unit = request.args.get("time_unit") or ""
    time_field, time_unit = _resolved_or(index, time_field, time_unit)
    frm = _parse_time_param(request.args.get("from"))
    to = _parse_time_param(request.args.get("to"))
    filters = _request_filters()
    query = _build_query(index, time_field, frm, to, filters, time_unit,
                         q=request.args.get("q"), dsl=request.args.get("dsl"))
    return index, time_field, time_unit, frm, to, query


def _dig(src, path):
    """Read a dotted path out of a nested _source dict."""
    cur = src
    for part in path.split("."):
        if isinstance(cur, dict) and part in cur:
            cur = cur[part]
        else:
            return None
    return cur


# ------------------------------------------------------------------------------
# Discovery endpoints — let the UI build filters and panels without hardcoding
# ------------------------------------------------------------------------------

@app.route("/api/es/indices")
def es_indices():
    """List the ros-* indices, so the panel builder can offer real choices."""
    pattern = request.args.get("pattern", "ros-*")
    try:
        rows = _es_get(f"_cat/indices/{pattern}?format=json&h=index,docs.count")
    except ES_ERRORS as e:
        return _es_fail(e)
    out = [{"index": r.get("index"), "docs": int(r.get("docs.count") or 0)}
           for r in rows if not (r.get("index") or "").startswith(".")]
    out.sort(key=lambda r: r["index"])
    return jsonify({"indices": out})


@app.route("/api/es/fields")
def es_fields():
    """Flattened field list for an index, with types (drives autocomplete)."""
    index = request.args.get("index", "")
    if not index:
        return jsonify({"error": "index required"}), 400
    fields = _index_fields(index)
    numeric = {"long", "integer", "short", "byte", "double", "float",
               "half_float", "scaled_float"}
    return jsonify({
        "fields": fields,
        "numeric": [f["path"] for f in fields if f["type"] in numeric],
        "keyword": [f["path"] for f in fields if f["type"] == "keyword"],
        "date": [f["path"] for f in fields if f["type"] == "date"],
    })


@app.route("/api/es/terms")
def es_terms():
    """Distinct values of a field with their counts — populates the dropdowns.

    Used for the use-case picker: `field=use_case` resolves to `use_case.keyword`
    automatically when the index mapped it dynamically as text.
    """
    index, time_field, time_unit, frm, to, query = _common_params()
    field = request.args.get("field", "use_case")
    size = max(1, min(500, int(request.args.get("size", 50))))
    if not index:
        return jsonify({"error": "index required"}), 400

    body = {
        "size": 0,
        "query": query,
        "aggs": {
            "vals": {"terms": {"field": _aggregatable(index, field),
                               "size": size, "order": {"_count": "desc"}}},
            # Documents collected before use-case tagging existed have no such
            # field. Report them so the UI can offer "untagged (legacy data)"
            # rather than pretending they are not there.
            "missing": {"missing": {"field": _aggregatable(index, field)}},
        },
    }
    try:
        res = _es_search(index, body)
    except ES_ERRORS as e:
        return _es_fail(e)

    agg = res.get("aggregations", {})
    buckets = [{"value": b["key"], "count": b["doc_count"]}
               for b in agg.get("vals", {}).get("buckets", [])]
    return jsonify({
        "values": buckets,
        "missing": agg.get("missing", {}).get("doc_count", 0),
    })


@app.route("/api/es/use_cases")
def es_use_cases():
    """Use-case names the dashboard knows about (static, always available)."""
    return jsonify({"use_cases": KNOWN_USE_CASES,
                    "active": use_case_pub.current,
                    "run_id": use_case_pub.current_run_id})


@app.route("/api/es/range")
def es_range():
    """Return the min/max timestamp available in an index (for 'fit to data')."""
    index = request.args.get("index", "")
    if not index:
        return jsonify({"error": "index required"}), 400
    time_field, time_unit = _resolved_or(
        index, request.args.get("time_field"), request.args.get("time_unit"))
    filters = _request_filters()
    # Floor applied here too, else the 1970 sim-clock documents win the min.
    query = _build_query(index, time_field, None, None, filters, time_unit,
                         q=request.args.get("q"), dsl=request.args.get("dsl"))
    body = {
        "size": 0,
        "query": query,
        "aggs": {
            "mn": {"min": {"field": time_field}},
            "mx": {"max": {"field": time_field}},
        },
    }
    try:
        res = _es_search(index, body)
    except ES_ERRORS as e:
        return _es_fail(e)
    agg = res.get("aggregations", {})
    # Always answer in epoch milliseconds. Now that the time field is resolved
    # per index, "fit to data" can land on an epoch-SECONDS field, and the UI
    # feeds this straight into new Date(...).
    mul = 1000 if time_unit == "s" else 1
    mn = agg.get("mn", {}).get("value")
    mx = agg.get("mx", {}).get("value")
    return jsonify({
        "min": mn * mul if mn is not None else None,
        "max": mx * mul if mx is not None else None,
        "min_str": agg.get("mn", {}).get("value_as_string"),
        "max_str": agg.get("mx", {}).get("value_as_string"),
        "time_field": time_field,
        "time_unit": time_unit,
    })


@app.route("/api/es/time_field")
def es_time_field():
    """Which field this index can actually be plotted against.

    The UI asks once per index and then stops guessing: it used to hardcode
    "@timestamp unless the index is ros-tcp-pose-topic", which left the other
    unstamped indices drawing empty charts with no explanation.
    """
    index = request.args.get("index", "")
    if not index:
        return jsonify({"error": "index required"}), 400
    force = request.args.get("refresh") in ("1", "true", "yes")
    return jsonify(_resolve_time_field(index, force=force))


_NUMERIC_ES_TYPES = {"long", "integer", "short", "byte", "double", "float",
                     "half_float", "scaled_float"}


@app.route("/api/es/field_summary")
def es_field_summary():
    """What one field looks like inside the current selection.

    Drives the Discover field sidebar: click a field, see how it is actually
    distributed before deciding to make it a column or a filter. The shape of
    the answer follows the field's type — top values for a keyword, quartiles
    for a number, a span for a date — because those are the useful questions
    for each, and a terms aggregation over a float is meaningless.
    """
    index, time_field, time_unit, frm, to, query = _common_params()
    field = request.args.get("field", "")
    size = max(1, min(50, int(request.args.get("size", 8))))
    if not index or not field:
        return jsonify({"error": "index and field required"}), 400

    types = {f["path"]: f["type"] for f in _index_fields(index)}
    ftype = types.get(field, "unknown")
    agg_field = _aggregatable(index, field)

    aggs = {
        "present": {"filter": {"exists": {"field": field}}},
        "distinct": {"cardinality": {"field": agg_field}},
    }
    if ftype in _NUMERIC_ES_TYPES:
        aggs["stats"] = {"stats": {"field": field}}
        aggs["pct"] = {"percentiles": {"field": field,
                                       "percents": [5, 25, 50, 75, 95]}}
    elif ftype == "date":
        aggs["stats"] = {"stats": {"field": field}}
    else:
        aggs["top"] = {"terms": {"field": agg_field, "size": size,
                                 "order": {"_count": "desc"}}}

    try:
        res = _es_search(index, {"size": 0, "track_total_hits": True,
                                 "query": query, "aggs": aggs})
    except ES_ERRORS as e:
        return _es_fail(e)

    agg = res.get("aggregations", {})
    total = res.get("hits", {}).get("total", {})
    total = total.get("value") if isinstance(total, dict) else total
    present = agg.get("present", {}).get("doc_count", 0)

    out = {
        "field": field, "type": ftype, "agg_field": agg_field,
        "total": total, "present": present, "missing": max(0, (total or 0) - present),
        "distinct": agg.get("distinct", {}).get("value"),
        "kind": "number" if ftype in _NUMERIC_ES_TYPES else
                ("date" if ftype == "date" else "terms"),
    }
    if "top" in agg:
        out["top"] = [{"value": b["key"], "count": b["doc_count"],
                       "pct": (b["doc_count"] / present * 100) if present else 0}
                      for b in agg["top"].get("buckets", [])]
    if "stats" in agg:
        out["stats"] = agg["stats"]
    if "pct" in agg:
        out["percentiles"] = agg["pct"].get("values", {})
    return jsonify(out)


@app.route("/api/es/query_preview")
def es_query_preview():
    """The Elasticsearch query the current UI state produces.

    Backs the "Show query" panel: the filter pills, the use-case chips, the
    range picker and the query bar all funnel into one bool query, and being
    able to read it is both the fastest way to debug an empty chart and the
    thing that makes the pill builder teachable.
    """
    index, time_field, time_unit, frm, to, query = _common_params()
    if not index:
        return jsonify({"error": "index required"}), 400
    return jsonify({
        "index": index,
        "time_field": time_field,
        "time_unit": time_unit,
        "query": query,
        "body": {"size": 0, "query": query},
        "curl": f"curl -s -H 'Content-Type: application/json' "
                f"'{ES_BASE_URL}/{index}/_search' -d "
                f"'{json.dumps({'size': 0, 'query': query})}'",
    })


# ------------------------------------------------------------------------------
# Visualization endpoints
# ------------------------------------------------------------------------------

_SPAN_CACHE = {}        # (index, time_field, query json) -> (computed_at, span)
_SPAN_TTL = 5.0


def _span_ms(index, time_field, frm, to, query):
    """Milliseconds covered by the current selection, for bucket sizing.

    Memoised for a few seconds: a dashboard refresh fires every panel at once
    and panels sharing an index ask for the very same span, which was one
    redundant round trip each. Eleven panels over four indices went from eleven
    span queries to four. The TTL is short enough that a changed range or filter
    (both of which change the cache key anyway) is never served stale.
    """
    if isinstance(frm, int) and isinstance(to, int):
        return max(1, to - frm)

    key = (index, time_field, json.dumps(query, sort_keys=True))
    now = time.time()
    hit = _SPAN_CACHE.get(key)
    if hit and now - hit[0] < _SPAN_TTL:
        return hit[1]

    res = _es_search(index, {
        "size": 0, "query": query,
        "aggs": {"mn": {"min": {"field": time_field}},
                 "mx": {"max": {"field": time_field}}},
    })
    agg = res.get("aggregations", {})
    mn, mx = agg.get("mn", {}).get("value"), agg.get("mx", {}).get("value")
    span = None if (mn is None or mx is None) else max(1, int(mx - mn))

    if len(_SPAN_CACHE) > 256:          # bounded; it is a per-refresh scratchpad
        _SPAN_CACHE.clear()
    _SPAN_CACHE[key] = (now, span)
    return span


# ------------------------------------------------------------------------------
# Corpus overview — what the pipeline has actually collected
# ------------------------------------------------------------------------------

_OVERVIEW_CACHE = {}      # (from, to) -> (computed_at, payload)
_OVERVIEW_TTL = 30.0

# Everything the pipeline writes, not just `ros-*`. The dashboard's index picker
# uses `ros-*`, which silently omits ur-rtde-data and ros_sim_image — together
# 719 k documents and 9 GB, a fifth of the archive. A headline count that leaves
# those out is simply wrong, so system indices (leading dot) are the only thing
# excluded here.
def _corpus_indices():
    rows = _es_get("_cat/indices?format=json&h=index,docs.count,store.size&bytes=b")
    out = []
    for r in rows:
        name = r.get("index") or ""
        if name.startswith("."):
            continue
        try:
            docs = int(r.get("docs.count") or 0)
        except (TypeError, ValueError):
            docs = 0
        try:
            size = int(r.get("store.size") or 0)
        except (TypeError, ValueError):
            size = 0
        out.append({"index": name, "docs": docs, "bytes": size})
    out.sort(key=lambda r: -r["docs"])
    return out


@app.route("/api/es/overview")
def es_overview():
    """Inventory of the whole Elasticsearch corpus, for the KPI row.

    Per index: document count, size on disk, the time field that actually works
    there, the span it covers, and how much of it falls in the selected range.
    The tiles used to show the peak elbow effort of one hardcoded joint, which
    said nothing about the data set as a whole.
    """
    frm = _parse_time_param(request.args.get("from"))
    to = _parse_time_param(request.args.get("to"))
    key = (frm, to)
    now = time.time()
    hit = _OVERVIEW_CACHE.get(key)
    if hit and now - hit[0] < _OVERVIEW_TTL:
        return jsonify(hit[1])

    try:
        rows = _corpus_indices()
    except ES_ERRORS as e:
        return _es_fail(e)

    total_docs = sum(r["docs"] for r in rows)
    total_bytes = sum(r["bytes"] for r in rows)
    t_min = t_max = None
    in_range = 0
    usable = 0
    tagged = 0

    for r in rows:
        if not r["docs"]:
            r.update({"usable": False, "time_field": None, "in_range": 0})
            continue
        res = _resolve_time_field(r["index"])
        r["time_field"] = res["time_field"]
        r["time_unit"] = res["time_unit"]
        r["usable"] = res["usable"]
        r["t_min"] = r["t_max"] = None
        r["in_range"] = 0
        if not res["usable"]:
            continue
        usable += 1

        mul = 1000 if res["time_unit"] == "s" else 1
        query = _build_query(r["index"], res["time_field"], frm, to, [],
                             res["time_unit"])
        try:
            sub = _es_search(r["index"], {
                "size": 0, "track_total_hits": True, "query": query,
                "aggs": {"mn": {"min": {"field": res["time_field"]}},
                         "mx": {"max": {"field": res["time_field"]}},
                         "tagged": {"filter": {"exists": {
                             "field": _aggregatable(r["index"], "use_case")}}}},
            })
        except ES_ERRORS:
            continue

        agg = sub.get("aggregations", {})
        hits = sub.get("hits", {}).get("total", {})
        r["in_range"] = hits.get("value") if isinstance(hits, dict) else hits
        in_range += r["in_range"] or 0
        r["tagged"] = agg.get("tagged", {}).get("doc_count", 0)
        tagged += r["tagged"]

        mn = agg.get("mn", {}).get("value")
        mx = agg.get("mx", {}).get("value")
        if mn is not None:
            r["t_min"] = mn * mul
            t_min = r["t_min"] if t_min is None else min(t_min, r["t_min"])
        if mx is not None:
            r["t_max"] = mx * mul
            t_max = r["t_max"] if t_max is None else max(t_max, r["t_max"])

    payload = {
        "totals": {
            "docs": total_docs,
            "bytes": total_bytes,
            "indices": len(rows),
            "usable": usable,
            "unusable": len(rows) - usable,
            "in_range": in_range,
            "tagged": tagged,
            "t_min": t_min,
            "t_max": t_max,
            "span_ms": (t_max - t_min) if (t_min is not None and t_max is not None) else None,
        },
        "indices": rows,
        "ranged": frm is not None or to is not None,
    }
    _OVERVIEW_CACHE.clear()
    _OVERVIEW_CACHE[key] = (now, payload)
    return jsonify(payload)


@app.route("/api/es/timeseries")
def es_timeseries():
    """Downsampled time-series via a date_histogram.

    Query params (on top of index/time_field/from/to/filters):
      fields    comma-separated numeric field paths
      points    target number of buckets (default 500)
      stat      avg (default) | min | max | envelope | count
                `envelope` returns avg AND min AND max per field. Averaging
                alone hides spikes: at 500 points over a long run each bucket
                covers seconds, and a 0.25 s collision peak disappears into the
                mean. The envelope is what makes such events visible.
                `count` ignores `fields` entirely and returns the bucket
                doc_count as the single series "doc_count::count" — this is what
                draws the document-volume histogram above the Discover table.
      split_by  keyword field to break each series down by (e.g. use_case).
                Series are then keyed "<field>||<split value>".
    """
    index, time_field, time_unit, frm, to, query = _common_params()
    fields = [f for f in request.args.get("fields", "").split(",") if f]
    # Upper bound stays under Elasticsearch's default search.max_buckets (65536).
    points = max(10, min(60000, int(request.args.get("points", 500))))
    stat = request.args.get("stat", "avg")
    split_by = request.args.get("split_by", "").strip()

    count_only = stat == "count"
    if not index or (not fields and not count_only):
        return jsonify({"error": "index and fields required"}), 400

    if count_only:
        fields, stats = ["doc_count"], ["count"]
    else:
        stats = ["avg", "min", "max"] if stat == "envelope" else \
            [stat if stat in ("avg", "min", "max") else "avg"]

    try:
        span = _span_ms(index, time_field, frm, to, query)
    except ES_ERRORS as e:
        return _es_fail(e)
    if span is None:
        return jsonify({"time": [], "series": {}, "stats": stats})

    # header.sec is stored in seconds, so the span comes back in seconds too.
    if time_unit == "s":
        span *= 1000
    interval_ms = max(1, span // points)

    metrics = {}
    if not count_only:
        for i, f in enumerate(fields):
            for st in stats:
                metrics[f"{st}{i}"] = {st: {"field": f}}

    if split_by:
        split_field = _aggregatable(index, split_by)
        split_agg = {"terms": {"field": split_field, "size": 20}}
        if metrics:                     # count mode needs no sub-aggregation
            split_agg["aggs"] = metrics
        inner = {"split": split_agg}
    else:
        inner = metrics

    if time_unit == "s":
        hist = {"histogram": {"field": time_field,
                              "interval": max(0.001, interval_ms / 1000.0),
                              "min_doc_count": 1}}
    else:
        hist = {"date_histogram": {"field": time_field,
                                   "fixed_interval": f"{interval_ms}ms",
                                   "min_doc_count": 1}}
    if inner:
        hist["aggs"] = inner

    try:
        res = _es_search(index, {"size": 0, "query": query, "aggs": {"ts": hist}})
    except ES_ERRORS as e:
        return _es_fail(e)

    buckets = res.get("aggregations", {}).get("ts", {}).get("buckets", [])
    out_time = []
    out_series = {}
    groups = []

    def key_of(field, st, group=None):
        base = f"{field}::{st}"
        return f"{base}||{group}" if group is not None else base

    if split_by:
        # Collect the group names first so every series has the same length.
        for b in buckets:
            for sb in b.get("split", {}).get("buckets", []):
                if sb["key"] not in groups:
                    groups.append(sb["key"])
        for f in fields:
            for st in stats:
                for g in groups:
                    out_series[key_of(f, st, g)] = []
        for b in buckets:
            out_time.append(b["key"])
            present = {sb["key"]: sb for sb in b.get("split", {}).get("buckets", [])}
            for i, f in enumerate(fields):
                for st in stats:
                    for g in groups:
                        sb = present.get(g)
                        if sb is None:
                            val = None
                        elif count_only:
                            val = sb.get("doc_count")
                        else:
                            val = sb.get(f"{st}{i}", {}).get("value")
                        out_series[key_of(f, st, g)].append(val)
    else:
        for f in fields:
            for st in stats:
                out_series[key_of(f, st)] = []
        for b in buckets:
            out_time.append(b["key"])
            for i, f in enumerate(fields):
                for st in stats:
                    out_series[key_of(f, st)].append(
                        b.get("doc_count") if count_only
                        else b.get(f"{st}{i}", {}).get("value"))

    # header.sec buckets come back in seconds; the UI plots epoch millis.
    if time_unit == "s":
        out_time = [t * 1000 for t in out_time]

    return jsonify({"time": out_time, "series": out_series,
                    "stats": stats, "groups": groups})


@app.route("/api/es/histogram")
def es_histogram():
    """Value distribution of a numeric field.

    Answers "how is effort distributed", and with split_by=use_case, "how does
    that distribution differ between the four scenarios".
    """
    index, time_field, time_unit, frm, to, query = _common_params()
    field = request.args.get("field", "")
    bins = max(5, min(200, int(request.args.get("bins", 40))))
    split_by = request.args.get("split_by", "").strip()
    interval = request.args.get("interval")

    if not index or not field:
        return jsonify({"error": "index and field required"}), 400

    try:
        if interval:
            step = float(interval)
        else:
            res = _es_search(index, {"size": 0, "query": query,
                                     "aggs": {"st": {"stats": {"field": field}}}})
            st = res.get("aggregations", {}).get("st", {})
            lo, hi = st.get("min"), st.get("max")
            if lo is None or hi is None:
                return jsonify({"bins": [], "series": {}, "groups": []})
            step = (hi - lo) / bins if hi > lo else 1.0
            if step <= 0:
                step = 1.0
    except ES_ERRORS as e:
        return _es_fail(e)

    hist = {"histogram": {"field": field, "interval": step, "min_doc_count": 0}}
    if split_by:
        aggs = {"split": {"terms": {"field": _aggregatable(index, split_by),
                                    "size": 20},
                          "aggs": {"h": hist}}}
    else:
        aggs = {"h": hist}

    try:
        res = _es_search(index, {"size": 0, "query": query, "aggs": aggs})
    except ES_ERRORS as e:
        return _es_fail(e)

    agg = res.get("aggregations", {})
    if split_by:
        groups, per_group = [], {}
        for sb in agg.get("split", {}).get("buckets", []):
            groups.append(sb["key"])
            per_group[sb["key"]] = {b["key"]: b["doc_count"]
                                    for b in sb.get("h", {}).get("buckets", [])}
        edges = sorted({k for g in per_group.values() for k in g})
        series = {g: [per_group[g].get(e, 0) for e in edges] for g in groups}
    else:
        buckets = agg.get("h", {}).get("buckets", [])
        edges = [b["key"] for b in buckets]
        series = {"all": [b["doc_count"] for b in buckets]}
        groups = ["all"]

    return jsonify({"bins": edges, "series": series,
                    "groups": groups, "interval": step})


@app.route("/api/es/percentiles")
def es_percentiles():
    """Percentiles per field, optionally split by a keyword field.

    Feeds the box plots: with split_by=use_case this is the single chart that
    shows how the four scenarios load the robot differently.
    """
    index, time_field, time_unit, frm, to, query = _common_params()
    fields = [f for f in request.args.get("fields", "").split(",") if f]
    split_by = request.args.get("split_by", "").strip()
    percents = [float(p) for p in
                request.args.get("percents", "5,25,50,75,95").split(",") if p]

    if not index or not fields:
        return jsonify({"error": "index and fields required"}), 400

    metrics = {f"p{i}": {"percentiles": {"field": f, "percents": percents}}
               for i, f in enumerate(fields)}
    if split_by:
        aggs = {"split": {"terms": {"field": _aggregatable(index, split_by),
                                    "size": 20},
                          "aggs": metrics}}
    else:
        aggs = metrics

    try:
        res = _es_search(index, {"size": 0, "query": query, "aggs": aggs})
    except ES_ERRORS as e:
        return _es_fail(e)

    agg = res.get("aggregations", {})

    def extract(container):
        out = {}
        for i, f in enumerate(fields):
            vals = container.get(f"p{i}", {}).get("values", {})
            # ES keys percentile results as stringified floats ("50.0").
            out[f] = {str(p): vals.get(str(float(p))) for p in percents}
        return out

    if split_by:
        groups, series = [], {}
        for sb in agg.get("split", {}).get("buckets", []):
            groups.append(sb["key"])
            series[sb["key"]] = extract(sb)
    else:
        groups = ["all"]
        series = {"all": extract(agg)}

    return jsonify({"groups": groups, "series": series,
                    "fields": fields, "percents": percents})


@app.route("/api/es/stats")
def es_stats():
    """Summary numbers for the KPI tiles: doc count, time span, per-field stats."""
    index, time_field, time_unit, frm, to, query = _common_params()
    fields = [f for f in request.args.get("fields", "").split(",") if f]
    split_by = request.args.get("split_by", "").strip()
    if not index:
        return jsonify({"error": "index required"}), 400

    metrics = {f"s{i}": {"stats": {"field": f}} for i, f in enumerate(fields)}
    aggs = dict(metrics)
    aggs["t_min"] = {"min": {"field": time_field}}
    aggs["t_max"] = {"max": {"field": time_field}}
    if split_by:
        aggs["split"] = {"terms": {"field": _aggregatable(index, split_by),
                                   "size": 20},
                         "aggs": metrics}

    try:
        res = _es_search(index, {"size": 0, "query": query,
                                 "track_total_hits": True, "aggs": aggs})
    except ES_ERRORS as e:
        return _es_fail(e)

    agg = res.get("aggregations", {})
    mul = 1000.0 if time_unit == "s" else 1.0
    t_min = agg.get("t_min", {}).get("value")
    t_max = agg.get("t_max", {}).get("value")
    total = res.get("hits", {}).get("total", {})

    groups, by_group = [], {}
    for sb in agg.get("split", {}).get("buckets", []):
        groups.append(sb["key"])
        by_group[sb["key"]] = {
            "count": sb.get("doc_count"),
            **{f: sb.get(f"s{i}", {}) for i, f in enumerate(fields)},
        }

    return jsonify({
        "count": total.get("value") if isinstance(total, dict) else total,
        "t_min": t_min * mul if t_min is not None else None,
        "t_max": t_max * mul if t_max is not None else None,
        "duration_ms": (t_max - t_min) * mul
                       if (t_min is not None and t_max is not None) else None,
        "series": {f: agg.get(f"s{i}", {}) for i, f in enumerate(fields)},
        "groups": groups,
        "by_group": by_group,
    })


@app.route("/api/es/points")
def es_points():
    """Raw x/y[/z] points for the 2D and 3D scatter panels.

    `mode` decides HOW the points are picked:
      spread (default) — one document per time bucket, evenly across the range.
      latest           — the newest `limit` documents.

    `latest` was the only behaviour before, and it quietly misled: over a wide
    range it returns the tail of the data, not a picture of the whole range, so
    a TCP path looked like it only covered the last few seconds.
    """
    index, time_field, time_unit, frm, to, query = _common_params(
        default_index="ros-tcp-pose-topic")
    fx = request.args.get("x", "pose.position.x")
    fy = request.args.get("y", "pose.position.y")
    fz = request.args.get("z", "pose.position.z")
    fc = request.args.get("color", "").strip()
    limit = max(10, min(10000, int(request.args.get("limit", 3000))))
    mode = request.args.get("mode", "spread")

    axes = [a for a in (fx, fy, fz, fc) if a]
    source = list(dict.fromkeys(axes + [time_field]))

    try:
        if mode == "spread":
            span = _span_ms(index, time_field, frm, to, query)
            if span is None:
                return jsonify({"x": [], "y": [], "z": [], "color": [], "t": []})
            if time_unit == "s":
                span *= 1000

            # Two passes, because uniform time bucketing samples clustered data
            # badly. The TCP stream is 1.19 M documents inside roughly 190 short
            # recording windows scattered over eight months: asking for 4000
            # evenly spaced buckets and one document each returned 31 points,
            # since every other bucket falls in a gap and `min_doc_count` drops
            # it. The first pass counts the windows that actually hold
            # documents; the second scales the per-window sample so the point
            # budget is spent where there is something to draw.
            def _hist(interval_ms):
                if time_unit == "s":
                    return {"histogram": {"field": time_field,
                                          "interval": max(0.001, interval_ms / 1000.0),
                                          "min_doc_count": 1}}
                return {"date_histogram": {"field": time_field,
                                           "fixed_interval": f"{interval_ms}ms",
                                           "min_doc_count": 1}}

            # Finer than the point budget, so short bursts resolve separately
            # instead of collapsing into one bucket. Kept under ES's default
            # search.max_buckets (65536).
            buckets_target = max(limit, min(60000, limit * 8))
            interval_ms = max(1, span // buckets_target)

            probe = _es_search(index, {"size": 0, "query": query,
                                       "aggs": {"ts": _hist(interval_ms)}})
            occupied = len(probe.get("aggregations", {})
                                .get("ts", {}).get("buckets", []))
            if not occupied:
                return jsonify({"x": [], "y": [], "z": [], "color": [], "t": []})

            per_bucket = max(1, min(500, -(-limit // occupied)))
            hist = _hist(interval_ms)
            hist["aggs"] = {"doc": {"top_hits": {
                "size": per_bucket, "_source": source,
                "sort": [{time_field: {"order": "asc"}}],
            }}}
            res = _es_search(index, {"size": 0, "query": query,
                                     "aggs": {"ts": hist}})
            hits = [h
                    for b in res.get("aggregations", {}).get("ts", {}).get("buckets", [])
                    for h in b.get("doc", {}).get("hits", {}).get("hits", [])]
            # Overshoot is possible when the windows are unevenly filled; thin
            # it evenly rather than truncating, which would cut the path short.
            if len(hits) > limit:
                step = len(hits) / float(limit)
                hits = [hits[int(i * step)] for i in range(limit)]
        else:
            res = _es_search(index, {
                "size": limit, "_source": source, "query": query,
                "sort": [{time_field: {"order": "desc"}}],
            })
            hits = res.get("hits", {}).get("hits", [])[::-1]  # chronological
    except ES_ERRORS as e:
        return _es_fail(e)

    xs, ys, zs, cs, ts = [], [], [], [], []
    for hit in hits:
        src = hit.get("_source", {})
        xv, yv = _dig(src, fx), _dig(src, fy)
        zv = _dig(src, fz) if fz else 0
        if xv is None or yv is None or (fz and zv is None):
            continue
        xs.append(xv)
        ys.append(yv)
        zs.append(zv)
        if fc:
            cs.append(_dig(src, fc))
        tv = _dig(src, time_field)
        ts.append(tv * 1000 if (tv is not None and time_unit == "s") else tv)

    return jsonify({"x": xs, "y": ys, "z": zs, "color": cs, "t": ts})


# ==============================================================================
# Cell geometry — the chassis mesh drawn under the TCP path
# ==============================================================================
#
# The TCP pose is published by tcp_pose_broadcaster in the `ur10e_base` frame
# (ur_controllers.yaml: `frame_id: $(var tf_prefix)base`), while the chassis
# meshes are authored in the cell frame. Everything below exists to move one
# into the other. The chain, read off the URDF:
#
#   world -> table            my_robot_cell.urdf.xacro: origin 0 0 0
#   table -> robot_mount      my_robot_cell_macro.xacro: PRISMATIC along +y,
#                             origin (-0.158, 0.115, 0.58635)
#   robot_mount -> base_link  ur_robot macro invoked with origin 0 0 0
#   base_link  -> base        ur_macro.xacro: rpy 0 0 pi  (UR convention)
#
# so, with q the linear-axis position,
#   p_world = RotZ(pi) * p_base + (-0.158, 0.115 + q, 0.58635)
# and the inverse used here (RotZ(pi) is its own inverse):
#   x_base = -(x_world + 0.158)
#   y_base = -(y_world - 0.115 - q)
#   z_base =   z_world - 0.58635
#
# q is not a constant: the robot rides the linear axis. The mesh is therefore
# cached at q = 0 and the axis position is added to y at request time, which is
# exact because the axis is a pure +y translation.

CHASSIS_MESH_DIR = os.environ.get("CHASSIS_MESH_DIR", os.path.join(
    WORKSPACE_ROOT, "src", "Universal_Robots_ROS2_Description",
    "meshes", "ur10e", "collision", "chassis_last"))

# The xacro that positions the linear axis, so the mount offset is read from the
# robot description rather than copied into this file. Only this one joint is
# parsed: it is the cell's own calibration and the value most likely to change.
CELL_MACRO_XACRO = os.path.join(
    WORKSPACE_ROOT, "src", "Universal_Robots_ROS2_Tutorials", "my_robot_cell",
    "my_robot_cell_description", "urdf", "my_robot_cell_macro.xacro")

# Fallback if the xacro moves or stops parsing — the values as of this writing.
_MOUNT_ORIGIN_FALLBACK = (-0.158, 0.115, 0.58635)
_MOUNT_AXIS_FALLBACK = (0.0, 1.0, 0.0)

CHASSIS_INDEX = "ros-joint-states"
CHASSIS_AXIS_FIELD = "ur10e_base_to_robot_mount.position"

_MOUNT_CACHE = {}     # xacro mtime -> (origin, axis)
_MESH_CACHE = {}      # (dir signature, grid) -> mesh dict at q = 0


def _mount_transform():
    """(origin, axis) of the table -> robot_mount prismatic joint, from the xacro."""
    try:
        mtime = os.path.getmtime(CELL_MACRO_XACRO)
    except OSError:
        return _MOUNT_ORIGIN_FALLBACK, _MOUNT_AXIS_FALLBACK
    hit = _MOUNT_CACHE.get(mtime)
    if hit:
        return hit

    origin, axis = _MOUNT_ORIGIN_FALLBACK, _MOUNT_AXIS_FALLBACK
    try:
        with open(CELL_MACRO_XACRO, encoding="utf-8") as fh:
            text = fh.read()
        # The joint block, then its origin/axis. Deliberately narrow: a general
        # xacro evaluation would need properties, includes and substitution args.
        block = re.search(
            r'<joint[^>]*name="[^"]*base_to_robot_mount"[^>]*type="prismatic".*?</joint>',
            text, re.S)
        if block:
            b = block.group(0)
            m = re.search(r'<origin[^>]*xyz="([^"]+)"', b)
            if m:
                vals = [float(v) for v in m.group(1).split()]
                if len(vals) == 3:
                    origin = tuple(vals)
            m = re.search(r'<axis[^>]*xyz="([^"]+)"', b)
            if m:
                vals = [float(v) for v in m.group(1).split()]
                if len(vals) == 3:
                    axis = tuple(vals)
    except (OSError, ValueError):
        pass

    _MOUNT_CACHE.clear()
    _MOUNT_CACHE[mtime] = (origin, axis)
    return origin, axis


def _stl_triangles(path):
    """Yield (v0, v1, v2) from a binary or ASCII STL, in the file's own units."""
    with open(path, "rb") as fh:
        head = fh.read(84)
        if len(head) < 84:
            return
        count = struct.unpack("<I", head[80:84])[0]
        # A binary STL is exactly 84 + 50*count bytes. ASCII files begin with
        # "solid" too, so the length is what actually distinguishes them.
        if os.path.getsize(path) == 84 + 50 * count:
            for _ in range(count):
                rec = fh.read(50)
                if len(rec) < 50:
                    return
                d = struct.unpack("<12fH", rec)
                yield (d[3:6], d[6:9], d[9:12])
            return

    with open(path, encoding="utf-8", errors="replace") as fh:
        tri = []
        for line in fh:
            line = line.strip()
            if line.startswith("vertex"):
                parts = line.split()
                if len(parts) >= 4:
                    tri.append(tuple(float(v) for v in parts[1:4]))
                    if len(tri) == 3:
                        yield tuple(tri)
                        tri = []


def _dir_signature(path):
    """(name, size, mtime) of every STL in the directory — the cache key."""
    try:
        names = sorted(n for n in os.listdir(path) if n.lower().endswith(".stl"))
    except OSError:
        return None
    sig = []
    for n in names:
        try:
            st = os.stat(os.path.join(path, n))
        except OSError:
            continue
        sig.append((n, st.st_size, int(st.st_mtime)))
    return tuple(sig)


def _load_chassis_mesh(grid=0.005, scale=0.001):
    """Load, transform and decimate the chassis into a Plotly mesh3d payload.

    Returned in the `ur10e_base` frame with the linear axis at q = 0; the caller
    adds q to y. Decimation is vertex clustering on a `grid`-metre lattice:
    122 004 CAD triangles is far more than a background reference needs, and at
    5 mm the 40-60 mm structural members stay perfectly legible while the
    payload drops to about a twentieth.
    """
    sig = _dir_signature(CHASSIS_MESH_DIR)
    if sig is None:
        raise FileNotFoundError(CHASSIS_MESH_DIR)
    key = (sig, round(grid, 6), round(scale, 9))
    hit = _MESH_CACHE.get(key)
    if hit:
        return hit

    (ox, oy, oz), _axis = _mount_transform()

    vmap, xs, ys, zs = {}, [], [], []
    fi, fj, fk = [], [], []
    raw_tris = 0
    files = sorted(n for n in os.listdir(CHASSIS_MESH_DIR)
                   if n.lower().endswith(".stl"))

    for name in files:
        for tri in _stl_triangles(os.path.join(CHASSIS_MESH_DIR, name)):
            raw_tris += 1
            idx = []
            for vx, vy, vz in tri:
                # mesh units -> metres (cell frame), then cell frame -> base
                # frame at q = 0.
                bx = -(vx * scale - ox)
                by = -(vy * scale - oy)
                bz = vz * scale - oz
                cell = (round(bx / grid), round(by / grid), round(bz / grid))
                j = vmap.get(cell)
                if j is None:
                    j = len(xs)
                    vmap[cell] = j
                    xs.append(cell[0] * grid)
                    ys.append(cell[1] * grid)
                    zs.append(cell[2] * grid)
                idx.append(j)
            # Clustering collapses thin slivers onto a single vertex; those are
            # no longer triangles.
            if idx[0] != idx[1] and idx[1] != idx[2] and idx[0] != idx[2]:
                fi.append(idx[0])
                fj.append(idx[1])
                fk.append(idx[2])

    mesh = {
        "x": [round(v, 4) for v in xs],
        "y": [round(v, 4) for v in ys],
        "z": [round(v, 4) for v in zs],
        "i": fi, "j": fj, "k": fk,
        "parts": len(files),
        "triangles_raw": raw_tris,
        "triangles": len(fi),
        "vertices": len(xs),
        "grid_m": grid,
        "frame": "ur10e_base",
        "source": CHASSIS_MESH_DIR,
    }
    _MESH_CACHE.clear()          # one grid setting is in play at a time
    _MESH_CACHE[key] = mesh
    return mesh


def _linear_axis_position(frm, to):
    """Where the linear axis sat over the selected range.

    The chassis is fixed in the cell and the robot rides the axis, so in the
    `ur10e_base` frame the chassis moves. One number has to stand in for the
    whole selection; the median is used because the axis parks: over the full
    archive more than half the samples sit within a millimetre of 1.000 m even
    though the axis travels 0.05-1.90 m. The spread is reported alongside so the
    UI can say when a single position is a poor summary.
    """
    time_field, time_unit = _resolved_or(CHASSIS_INDEX, None, None)
    query = _build_query(CHASSIS_INDEX, time_field, frm, to, [], time_unit)
    body = {"size": 0, "query": query, "aggs": {
        "p": {"percentiles": {"field": CHASSIS_AXIS_FIELD,
                              "percents": [5, 50, 95]}},
        "s": {"stats": {"field": CHASSIS_AXIS_FIELD}},
    }}
    res = _es_search(CHASSIS_INDEX, body)
    agg = res.get("aggregations", {})
    pct = agg.get("p", {}).get("values", {}) or {}
    st = agg.get("s", {}) or {}
    return {
        "q": pct.get("50.0"),
        "q_p5": pct.get("5.0"),
        "q_p95": pct.get("95.0"),
        "q_min": st.get("min"),
        "q_max": st.get("max"),
        "samples": st.get("count"),
    }


@app.route("/api/mesh/chassis")
def api_chassis_mesh():
    """The cell chassis as a Plotly mesh3d, placed in the TCP pose's own frame.

    Read straight off the STLs the robot description uses, so the drawing cannot
    drift from the model MoveIt and the octomap see. Query params: `q` to pin the
    linear-axis position (metres), `grid` for the decimation lattice, plus the
    usual from/to used to work out q when it is not given.
    """
    grid = max(0.001, min(0.05, float(request.args.get("grid", 0.005))))
    frm = _parse_time_param(request.args.get("from"))
    to = _parse_time_param(request.args.get("to"))

    try:
        mesh = _load_chassis_mesh(grid=grid)
    except FileNotFoundError as e:
        return jsonify({"error": f"chassis meshes not found: {e}"}), 404
    except (OSError, struct.error, ValueError) as e:
        return jsonify({"error": f"{type(e).__name__}: {e}"}), 500

    axis = {"q": None, "q_min": None, "q_max": None, "samples": 0}
    q_source = "requested"
    q_arg = request.args.get("q")
    if q_arg not in (None, ""):
        try:
            q = float(q_arg)
        except ValueError:
            return jsonify({"error": f"q is not a number: {q_arg!r}"}), 400
    else:
        try:
            axis = _linear_axis_position(frm, to)
        except ES_ERRORS:
            axis = {"q": None, "q_min": None, "q_max": None, "samples": 0}
        q = axis.get("q")
        q_source = "median"
        if q is None:
            # No joint states in range: draw at the axis origin and say so
            # rather than silently placing the chassis somewhere invented.
            q = 0.0
            q_source = "unknown"

    out = dict(mesh)
    out["y"] = [round(v + q, 4) for v in mesh["y"]]
    out["q"] = round(q, 4)
    out["q_source"] = q_source
    out.update({k: axis.get(k) for k in ("q_p5", "q_p95", "q_min", "q_max", "samples")})
    return jsonify(out)


@app.route("/api/es/scatter3d")
def es_scatter3d():
    """Backwards-compatible alias for /api/es/points."""
    return es_points()


# Field families that are wide, near-constant and never what someone opens
# Discover to look at. ros-joint-states carries 118 leaves, ~70 of which are
# UR10e GPIO bits and firmware version numbers; taking the first 25 alphabetically
# filled the table with `digital_input_11` and pushed every joint out of view.
_NOISY_COLUMN_PATTERNS = (
    "kafka_metadata.",
    ".digital_input_", ".digital_output_",
    ".analog_io_type_", ".safety_status_bit_",
    ".robot_status_bit_", ".tool_analog_input_",
    "get_robot_software_version.",
    ".nanosec",          # the seconds half is the time field and already leads
)

_DEFAULT_COLUMN_LIMIT = 12


def _default_columns(index, time_field, hits):
    """Columns to show when the caller named none.

    Time field and the run tags first, then the leaves that are not part of a
    known-noisy family, capped — the field sidebar is how a specific column gets
    added, so the default only has to be a sane starting point.
    """
    leaves = [f["path"] for f in _index_fields(index)
              if not f["path"].endswith(".keyword")]
    head = [c for c in (time_field, "use_case", "run_id") if c in leaves]
    rest = [c for c in leaves
            if c not in head
            and not any(pat in c for pat in _NOISY_COLUMN_PATTERNS)]
    columns = head + rest[:_DEFAULT_COLUMN_LIMIT]

    if not columns and hits:
        # An index with no mapped properties (ros-interactive-marker-update-topic
        # stores empty documents): fall back to whatever the first hit holds.
        columns = [k for k, v in (hits[0].get("_source") or {}).items()
                   if not isinstance(v, (dict, list))][:_DEFAULT_COLUMN_LIMIT]
    return columns


@app.route("/api/es/docs")
def es_docs():
    """Raw documents for the Discover-style table, and CSV export.

    Params: fields (columns), size, offset, sort, order,
            format=json|csv|raw.
    `raw` returns each hit's untouched _source alongside _id/_index, which is
    what the expandable row in the Discover table shows — the flattened `rows`
    form throws away exactly the nesting a ROS document is interesting for.
    """
    index, time_field, time_unit, frm, to, query = _common_params()
    fields = [f for f in request.args.get("fields", "").split(",") if f]
    size = max(1, min(1000, int(request.args.get("size", 100))))
    offset = max(0, min(9000, int(request.args.get("offset", 0))))
    sort_field = request.args.get("sort") or time_field
    order = "asc" if request.args.get("order", "desc") == "asc" else "desc"
    fmt = request.args.get("format", "json")

    if not index:
        return jsonify({"error": "index required"}), 400

    body = {
        "size": size, "from": offset, "query": query,
        "track_total_hits": True,
        "sort": [{sort_field: {"order": order}}],
    }
    # `raw` wants the whole document; a column list only narrows the flat forms.
    if fields and fmt != "raw":
        body["_source"] = list(dict.fromkeys(fields + [time_field]))

    try:
        res = _es_search(index, body)
    except ES_ERRORS as e:
        return _es_fail(e)

    hits = res.get("hits", {}).get("hits", [])
    total = res.get("hits", {}).get("total", {})
    total = total.get("value") if isinstance(total, dict) else total

    if fmt == "raw":
        # The default column list rides along so the table and the expandable
        # raw view are served by one request instead of two.
        return jsonify({
            "total": total,
            "time_field": time_field,
            "time_unit": time_unit,
            "columns": _default_columns(index, time_field, hits),
            "hits": [{"_id": h.get("_id"), "_index": h.get("_index"),
                      "_source": h.get("_source", {})} for h in hits],
        })

    columns = fields or _default_columns(index, time_field, hits)

    rows = []
    for h in hits:
        src = h.get("_source", {})
        rows.append([_dig(src, c) for c in columns])

    if fmt == "csv":
        import csv
        import io as _io
        buf = _io.StringIO()
        w = csv.writer(buf)
        w.writerow(columns)
        w.writerows(rows)
        return Response(
            buf.getvalue(), mimetype="text/csv",
            headers={"Content-Disposition":
                     f'attachment; filename="{index}.csv"'})

    return jsonify({
        "total": total,
        "columns": columns,
        "rows": rows,
        "time_field": time_field,
        "time_unit": time_unit,
    })


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
    data_acquisition = data.get("data_acquisition", False)
    scenario_mgr.start_scenario(scenario_key, use_fake_hardware, data_acquisition)

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

# --- Free Move -------------------------------------------------------------- #

@socketio.on("freemove_start")
def handle_freemove_start(data):
    use_fake_hardware = (data or {}).get("use_fake_hardware", False)
    thread = threading.Thread(
        target=scenario_mgr.start_free_move, args=(use_fake_hardware,), daemon=True)
    thread.start()

@socketio.on("freemove_stop")
def handle_freemove_stop():
    thread = threading.Thread(target=scenario_mgr.stop_free_move, daemon=True)
    thread.start()

_freemove_check_log_state = {}  # arm -> (last_error, last_logged_at)

@socketio.on("freemove_check")
def handle_freemove_check(data):
    data = data or {}
    arm_name = data.get("arm")
    arm = free_move.manager.arm(arm_name)
    if arm is None:
        emit("freemove_ik_result", {"arm": arm_name, "ok": False, "error": "unknown_arm"})
        return
    result = arm.check_ik(data.get("position"), data.get("quat_xyzw"))
    emit("freemove_ik_result", {"arm": arm_name, **result})

    # This fires many times per second while dragging, so only log a failure into the
    # terminal when the error actually CHANGES (or every 5s while it persists) --
    # otherwise "why is the ghost not showing" is invisible without a debugger, but
    # logging every single check would flood the terminal solid.
    if not result["ok"]:
        last_error, last_at = _freemove_check_log_state.get(arm_name, (None, 0))
        now = time.time()
        if result.get("error") != last_error or now - last_at > 5:
            scenario_mgr._emit_log("FREEMOVE",
                f"🔍 {arm_name}: IK check failing ({result.get('error')}).")
            _freemove_check_log_state[arm_name] = (result.get("error"), now)
    elif arm_name in _freemove_check_log_state:
        del _freemove_check_log_state[arm_name]

@socketio.on("freemove_plan")
def handle_freemove_plan(data):
    data = dict(data or {})

    def _run():
        arm_name = data.get("arm")
        arm = free_move.manager.arm(arm_name)
        if arm is None:
            socketio.emit("freemove_plan_result",
                {"arm": arm_name, "ok": False, "error": "unknown_arm"})
            return
        result = arm.plan(data.get("position"), data.get("quat_xyzw"))
        socketio.emit("freemove_plan_result", {"arm": arm_name, **result})
        if result["ok"]:
            scenario_mgr._emit_log("FREEMOVE", f"📐 {arm_name}: plan OK.")
        else:
            scenario_mgr._emit_log("FREEMOVE", f"📐 {arm_name}: plan failed ({result.get('error')}).")

    threading.Thread(target=_run, daemon=True).start()

@socketio.on("freemove_execute")
def handle_freemove_execute(data):
    data = dict(data or {})

    def _run():
        arm_name = data.get("arm")
        arm = free_move.manager.arm(arm_name)
        if arm is None:
            socketio.emit("freemove_execute_status",
                {"arm": arm_name, "state": "failed", "error": "unknown_arm"})
            return
        socketio.emit("freemove_execute_status", {"arm": arm_name, "state": "executing"})
        scenario_mgr._emit_log("FREEMOVE", f"🚀 {arm_name}: executing move...")
        result = arm.execute(data.get("position"), data.get("quat_xyzw"))
        socketio.emit("freemove_execute_status", {
            "arm": arm_name,
            "state": "succeeded" if result["ok"] else "failed",
            "error": result.get("error"),
        })
        if result["ok"]:
            scenario_mgr._emit_log("FREEMOVE", f"✅ {arm_name}: move complete.")
        else:
            scenario_mgr._emit_log("FREEMOVE", f"❌ {arm_name}: move failed ({result.get('error')}).")

    threading.Thread(target=_run, daemon=True).start()

@socketio.on("freemove_plan_joint")
def handle_freemove_plan_joint(data):
    data = dict(data or {})

    def _run():
        arm_name = data.get("arm")
        arm = free_move.manager.arm(arm_name)
        if arm is None:
            socketio.emit("freemove_plan_joint_result",
                {"arm": arm_name, "ok": False, "error": "unknown_arm"})
            return
        result = arm.plan_joint(data.get("joint_positions") or {})
        socketio.emit("freemove_plan_joint_result", {"arm": arm_name, **result})
        if result["ok"]:
            scenario_mgr._emit_log("FREEMOVE", f"📐 {arm_name}: joint plan OK.")
        else:
            scenario_mgr._emit_log("FREEMOVE",
                f"📐 {arm_name}: joint plan failed ({result.get('error')}).")

    threading.Thread(target=_run, daemon=True).start()

@socketio.on("freemove_execute_joint")
def handle_freemove_execute_joint(data):
    data = dict(data or {})

    def _run():
        arm_name = data.get("arm")
        arm = free_move.manager.arm(arm_name)
        if arm is None:
            socketio.emit("freemove_execute_status",
                {"arm": arm_name, "state": "failed", "error": "unknown_arm"})
            return
        socketio.emit("freemove_execute_status", {"arm": arm_name, "state": "executing"})
        scenario_mgr._emit_log("FREEMOVE", f"🚀 {arm_name}: executing joint move...")
        result = arm.execute_joint(data.get("joint_positions") or {})
        socketio.emit("freemove_execute_status", {
            "arm": arm_name,
            "state": "succeeded" if result["ok"] else "failed",
            "error": result.get("error"),
        })
        if result["ok"]:
            scenario_mgr._emit_log("FREEMOVE", f"✅ {arm_name}: joint move complete.")
        else:
            scenario_mgr._emit_log("FREEMOVE",
                f"❌ {arm_name}: joint move failed ({result.get('error')}).")

    threading.Thread(target=_run, daemon=True).start()

@socketio.on("freemove_cancel")
def handle_freemove_cancel(data):
    arm_name = (data or {}).get("arm")
    arm = free_move.manager.arm(arm_name)
    if arm is not None:
        arm.cancel()
        scenario_mgr._emit_log("FREEMOVE", f"🛑 {arm_name}: cancel requested.")

@socketio.on("freemove_set_params")
def handle_freemove_set_params(data):
    data = data or {}
    arm_name = data.get("arm")
    arm = free_move.manager.arm(arm_name)
    if arm is None:
        emit("freemove_params_result", {"arm": arm_name, "ok": False, "error": "unknown_arm"})
        return
    result = arm.set_params(data.get("params") or {})
    emit("freemove_params_result", {"arm": arm_name, **result})
    if result["ok"]:
        scenario_mgr._emit_log("FREEMOVE", f"⚙️ {arm_name}: planning settings updated.")
    else:
        scenario_mgr._emit_log("FREEMOVE", f"⚙️ {arm_name}: settings update failed ({result.get('error')}).")

@socketio.on("freemove_set_padding")
def handle_freemove_set_padding(data):
    data = dict(data or {})

    def _run():
        arm_name = data.get("arm")
        arm = free_move.manager.arm(arm_name)
        if arm is None:
            socketio.emit("freemove_padding_result",
                {"arm": arm_name, "ok": False, "error": "unknown_arm"})
            return
        result = arm.set_padding(data.get("padding_m", 0.0))
        socketio.emit("freemove_padding_result", {"arm": arm_name, **result})
        if result["ok"]:
            scenario_mgr._emit_log("FREEMOVE",
                f"⚙️ {arm_name}: padding {data.get('padding_m', 0.0) * 100:.1f} cm applied "
                f"to {result.get('links_padded')} links.")
        else:
            scenario_mgr._emit_log("FREEMOVE",
                f"⚙️ {arm_name}: padding update failed ({result.get('error')}).")

    threading.Thread(target=_run, daemon=True).start()


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

    # Start the use-case broadcaster (tags everything the Kafka bridge collects)
    use_case_pub.start()

    try:
        port = int(os.environ.get("DASHBOARD_PORT", "8080"))
        socketio.run(app, host="0.0.0.0", port=port, debug=False,
                     allow_unsafe_werkzeug=True)
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        camera_streamer.stop()
        joint_collector.stop()
        scenario_mgr.stop_all()
        use_case_pub.stop()


if __name__ == "__main__":
    main()
