#!/usr/bin/env python3
"""
Doors — Multirobot Sensing / Cleaning Dashboard
Flask + SocketIO backend, derived from harmony_user_interface/app.py.

WHAT IS DIFFERENT FROM THE HARMONY DASHBOARD
────────────────────────────────────────────
Almost nothing on this side, and that is on purpose. Every topic the dashboard
speaks -- /harmony/cmd_input, /harmony/robot_status, the defect topics,
/harmony/scenario_event -- is implemented identically by doors_mission_node, so the
charts, the defect table and the pop-ups all work unchanged.

The real differences:

  * MISSION_CMD points at doors_inspection's doors_mission.launch.py instead of
    my_robot_cell_control's sensing_and_cleaning_mission.launch.py. That launch runs
    doors_mission_node (which owns the doors inspection) plus the SAME
    cleaning_mission_runner.py the HARMONY scenario uses.
  * Two extra toggles -- only_sim and force_replan -- are forwarded to that launch.
  * BOTH arms are charted: 14 joint series instead of the UR's 7.
  * Two camera panels side by side -- the real cell and the Gazebo twin.
  * The KPI3 scenario fault is GONE. It is a HARMONY validation artefact.
  * Port 8082, so this and the HARMONY dashboard can run side by side.

Sensing itself is a genuinely different thing: HARMONY scanned with the UR alone on
hand-written waypoints, here both arms run the planned coverage tour and it takes
minutes rather than seconds. That is why robot_status carries "Scanning: UR 4/9
Kawasaki 2/3" progress -- doors_mission_node scrapes it from the launch output.

The four defects are the same four, imported from pymoveit2_real.harmony_defects by
the mission node rather than redefined, so the two scenarios cannot drift apart.
"""

import os
import sys
import signal
import subprocess
import threading
import time
import json
import random
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
app.config["SECRET_KEY"] = "doors-sensing-cleaning-2026"
socketio = SocketIO(app, cors_allowed_origins="*", async_mode="threading")

# ==============================================================================
# HARMONY validasyon senaryosu sabitleri
# ==============================================================================

# Defect topic'leri (pymoveit2_real.harmony_defects ile aynı isimler)
TOPIC_DEFECT = "/harmony/mock_perception/defect"
TOPIC_DEFECT_LIST = "/harmony/mock_perception/defect_list"
TOPIC_DEFECT_STATUS = "/harmony/defect_status"

# NOTE: the HARMONY dashboard also defines a KPI3 scenario fault here ("E-1042,
# Lineer eksen haberleşmesi sağlanamıyor"), fired when sensing reached its last
# viewpoint. It belongs to the HARMONY validation run and has been removed from the
# doors dashboard along with its generator, socket handlers and modal.

# ==============================================================================
# Global State
# ==============================================================================

class ProcessManager:
    """Manages the ROS2 launch process lifecycle for the doors mission."""

    HIL_CMD = "ros2 launch my_robot_cell_control hil_test_whole_unified.launch.py harmony:=true"
    # DOORS: doors_mission_node + the unchanged cleaning runner. sensing_robot.py is
    # deliberately not in this launch -- it subscribes to the same /harmony/cmd_input
    # and would start its own scan on START, commanding the UR from two places.
    MISSION_CMD = "ros2 launch doors_inspection doors_mission.launch.py"

    def __init__(self, socketio_instance):
        self.socketio = socketio_instance
        self.hil_process = None
        self.mission_process = None
        self.hil_status = "stopped"  # stopped, starting, running, stopping
        self.mission_status = "stopped"  # stopped, starting, running, stopping
        self.robot_confirmed = False
        self._lock = threading.Lock()
        self._log_threads = []

        # Log file
        log_dir = os.path.join(os.path.dirname(__file__), "log")
        os.makedirs(log_dir, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_filepath = os.path.join(log_dir, f"doors_log_{timestamp}.txt")
        print(f"[Log] Logs will be written to: {self.log_filepath}")

    def _emit_status(self):
        """Push current status to all connected clients."""
        self.socketio.emit("status_update", {
            "hil_status": self.hil_status,
            "mission_status": self.mission_status,
            "robot_confirmed": self.robot_confirmed,
        })

    def _emit_log(self, source, message):
        """Push a log message to all connected clients and write to file."""
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.socketio.emit("log_message", {
            "timestamp": timestamp,
            "source": source,
            "message": message,
        })
        try:
            with open(self.log_filepath, "a", encoding="utf-8") as f:
                f.write(f"[{timestamp}] [{source}] {message}\n")
        except Exception:
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
            pass

    def _kill_process(self, process, name, timeout=10):
        """Gracefully kill a process: SIGINT → wait → SIGTERM → wait → SIGKILL."""
        if process is None or process.poll() is not None:
            return

        self._emit_log("SYSTEM", f"🛑 Stopping {name} (SIGINT)...")

        try:
            os.killpg(os.getpgid(process.pid), signal.SIGINT)
        except (ProcessLookupError, OSError):
            return

        try:
            process.wait(timeout=timeout)
            self._emit_log("SYSTEM", f"✅ {name} successfully stopped.")
            return
        except subprocess.TimeoutExpired:
            pass

        self._emit_log("SYSTEM", f"⚠️ {name} did not respond, sending SIGTERM...")
        try:
            os.killpg(os.getpgid(process.pid), signal.SIGTERM)
            process.wait(timeout=5)
            self._emit_log("SYSTEM", f"✅ {name} stopped via SIGTERM.")
            return
        except (subprocess.TimeoutExpired, ProcessLookupError, OSError):
            pass

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

        full_cmd = f"source /opt/ros/humble/setup.bash && source /home/cem/colcon_ws/install/setup.bash && {cmd}"

        my_env = os.environ.copy()
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
            preexec_fn=os.setsid,
        )

        log_thread = threading.Thread(
            target=self._stream_output,
            args=(process, name),
            daemon=True,
        )
        log_thread.start()
        self._log_threads.append(log_thread)

        return process

    def start_hil(self, use_fake_hardware=False):
        """Start the HIL system."""
        def _run():
            with self._lock:
                if self.hil_process:
                    self._emit_log("SYSTEM", "📋 Stopping current process...")
                    self.hil_status = "stopping"
                    self._emit_status()
                    self._kill_process(self.hil_process, "HIL")
                    self.hil_process = None
                    self.hil_status = "stopped"
                    self._emit_status()
                    time.sleep(2)

                self.hil_status = "starting"
                self._emit_status()

                hil_cmd = self.HIL_CMD
                if use_fake_hardware:
                    hil_cmd += " use_fake_hardware:=true use_mock_hardware:=true fake_sensor_commands:=true"

                self.hil_process = self._start_process(hil_cmd, "HIL")
                self.hil_status = "running"
                self.robot_confirmed = False
                self._emit_status()

                self._emit_log("SYSTEM",
                    "✅ HIL system initialized. "
                    "Please confirm when robots are ready.")

        thread = threading.Thread(target=_run, daemon=True)
        thread.start()

    def confirm_robot_ready(self, only_sim=True, force_replan=False):
        """User confirms robots are ready → start the mission launch.

        only_sim / force_replan are doors-specific and are forwarded to
        doors_mission.launch.py, which hands them to the inspection launch when START
        arrives. They are read at CONFIRM time rather than at HIL start so the
        operator can still change them while the cell is coming up.
        """
        def _run():
            with self._lock:
                if self.hil_status != "running":
                    self._emit_log("SYSTEM", "❌ HIL is not running, cannot confirm.")
                    return

                self.robot_confirmed = True
                self.mission_status = "starting"
                self._emit_status()

                mission_cmd = (
                    f"{self.MISSION_CMD}"
                    f" only_sim:={'true' if only_sim else 'false'}"
                    f" force_replan:={'true' if force_replan else 'false'}"
                )
                self._emit_log("SYSTEM", "✅ Robot confirmation received. Starting mission...")
                if force_replan:
                    self._emit_log(
                        "SYSTEM",
                        "♻️ force_replan is ON: the cache is ignored and every viewpoint "
                        "is planned again. Slower, but required after a collision-model "
                        "change (cached paths are replayed WITHOUT a collision recheck).")

                self.mission_process = self._start_process(
                    mission_cmd, "MISSION"
                )
                self.mission_status = "running"
                self._emit_status()

                self._emit_log("SYSTEM", "🚀 Sensing & Cleaning Mission is running!")

        thread = threading.Thread(target=_run, daemon=True)
        thread.start()

    def stop_all(self):
        """Emergency stop: kill everything."""
        with self._lock:
            self._emit_log("SYSTEM", "🚨 EMERGENCY STOP — Terminating all processes!")

            self.mission_status = "stopping"
            self._emit_status()
            self._kill_process(self.mission_process, "Mission")
            self.mission_process = None
            self.mission_status = "stopped"

            self.hil_status = "stopping"
            self._emit_status()
            self._kill_process(self.hil_process, "HIL")
            self.hil_process = None
            self.hil_status = "stopped"

            self.robot_confirmed = False

            self._emit_status()
            self._emit_log("SYSTEM", "✅ All processes stopped.")

    def get_status(self):
        """Get current status as dict."""
        return {
            "hil_status": self.hil_status,
            "mission_status": self.mission_status,
            "robot_confirmed": self.robot_confirmed,
        }


process_mgr = ProcessManager(socketio)


# ==============================================================================
# ROS2 Data Collector (Joint States, Robot Status, Force, Defect)
# ==============================================================================

class ROS2DataCollector:
    """Collects data from ROS2 topics and pushes via SocketIO."""

    def __init__(self, socketio_instance, buffer_seconds=20):
        self.socketio = socketio_instance
        self.buffer_seconds = buffer_seconds

        # Joint state buffers. TWO POSITION STREAMS, not position+velocity: the two
        # joint charts are a HIL comparison -- the real cell on the left, the Gazebo
        # twin on the right, same joints, same colours, so a divergence between the
        # two is visible at a glance. Velocities are no longer collected; nothing
        # consumed them once the second chart became the sim mirror.
        self.joint_position_data = deque(maxlen=buffer_seconds * 1000)
        self.sim_position_data = deque(maxlen=buffer_seconds * 1000)

        # Force data buffer
        self.force_data = deque(maxlen=buffer_seconds * 1000)

        # Robot status
        self.current_mode = "IDLE"
        self.current_note = ""

        # Defect tablosu: defect_id -> {id, type, x, y, z, status, frame_id}
        # Sıralama sensing'in yayın sırasına göre korunur (DEF-01 ... DEF-04).
        self.defects = {}
        self.defect_order = []
        # Sensing tamamlandı pop-up'ı tur başına bir kez gösterilir.
        self._sensing_complete_notified = False
        # Cleaning tamamlandı pop-up'ı tur başına bir kez gösterilir.
        self._cleaning_complete_notified = False

        # Kalıcı komut yayıncısı. `ros2 topic pub -1` her çağrıda yeni bir node
        # kurup discovery tamamlanmadan çıktığı için ilk tıklamalar kaybolabiliyordu;
        # burada uzun ömürlü node üzerinden yayınlıyoruz.
        self._cmd_pub = None

        self._rclpy_thread = None
        self._running = False
        self._data_lock = threading.Lock()

    def publish_cmd(self, cmd_name):
        """Komutu kalıcı ROS node'u üzerinden yayınlar.

        Returns True if published; False if ROS is unavailable (caller falls
        back to the subprocess publisher).
        """
        pub = self._cmd_pub
        if pub is None:
            return False
        try:
            from std_msgs.msg import String
            msg = String()
            msg.data = json.dumps({"cmd": cmd_name})
            pub.publish(msg)
            return True
        except Exception as e:
            print(f"[ROS2DataCollector] publish_cmd error: {e}")
            return False

    # ------------------------------------------------------------------
    def _upsert_defect(self, payload):
        """Gelen defect JSON'unu tabloya ekler/gunceller."""
        defect_id = str(payload.get("defect_id", "")).strip()
        if not defect_id:
            return
        pos = payload.get("position", {}) or {}
        entry = {
            "id": defect_id,
            "type": payload.get("defect_type", ""),
            "x": float(pos.get("x", 0.0)),
            "y": float(pos.get("y", 0.0)),
            "z": float(pos.get("z", 0.0)),
            "frame_id": payload.get("frame_id", "world"),
            "status": payload.get("status", "DETECTED"),
        }
        with self._data_lock:
            if defect_id not in self.defects:
                self.defect_order.append(defect_id)
            else:
                # Durum bilgisi ayri topic'ten gelmis olabilir, ezme.
                entry["status"] = self.defects[defect_id].get("status", entry["status"])
            self.defects[defect_id] = entry

    def _defect_snapshot(self):
        with self._data_lock:
            return [self.defects[d] for d in self.defect_order if d in self.defects]

    def clear_defects(self):
        """Yeni bir sensing turu baslarken eski defect'leri temizler."""
        with self._data_lock:
            self.defects = {}
            self.defect_order = []
            self._sensing_complete_notified = False
            self._cleaning_complete_notified = False
        self.socketio.emit("defect_list", {"defects": []})

    def reset_ui(self):
        """Arayüzü ilk mission başlangıcı gibi sıfırlar.

        Defect tablosu, sayaçlar, mod göstergesi ve pop-up flag'leri temizlenir;
        arayüz grafik/log tarafını da sıfırlasın diye 'mission_reset' yayınlanır.
        Robot süreçlerine (HIL/mission) dokunmaz — yalnızca gösterim durumu.
        """
        self.clear_defects()  # defects + flags + boş defect_list
        with self._data_lock:
            self.current_mode = "IDLE"
            self.current_note = ""
        self.socketio.emit("robot_info", {"mode": "IDLE", "note": "", "defect_count": 0})
        self.socketio.emit("mission_reset", {
            "timestamp": datetime.now().strftime("%H:%M:%S"),
        })

    def _notify_sensing_complete(self, count):
        """Sensing tamamlandiginda operatore pop-up gonderir (tur basina bir kez)."""
        with self._data_lock:
            if self._sensing_complete_notified:
                return
            self._sensing_complete_notified = True
        self.socketio.emit("sensing_complete", {
            "count": count,
            "timestamp": datetime.now().strftime("%H:%M:%S"),
        })

    def _notify_cleaning_complete(self, data):
        """Cleaning tamamlandiginda operatore pop-up gonderir (tur basina bir kez)."""
        with self._data_lock:
            if self._cleaning_complete_notified:
                return
            self._cleaning_complete_notified = True
        try:
            cleaned = int(data.get("cleaned", 0))
        except (TypeError, ValueError):
            cleaned = 0
        try:
            total = int(data.get("total", 0))
        except (TypeError, ValueError):
            total = 0
        self.socketio.emit("cleaning_complete", {
            "cleaned": cleaned,
            "total": total,
            "timestamp": datetime.now().strftime("%H:%M:%S"),
        })

    def start(self):
        """Start the ROS2 subscriber thread."""
        self._running = True
        self._rclpy_thread = threading.Thread(target=self._ros_spin, daemon=True)
        self._rclpy_thread.start()

        self._push_thread = threading.Thread(target=self._periodic_push, daemon=True)
        self._push_thread.start()

    def _ros_spin(self):
        """ROS2 subscriber spin loop."""
        try:
            import rclpy
            from rclpy.node import Node as RosNode
            from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy,
                                   QoSProfile, QoSReliabilityPolicy)
            from sensor_msgs.msg import JointState
            from std_msgs.msg import String
            from geometry_msgs.msg import WrenchStamped

            rclpy.init()
            node = RosNode("doors_dashboard_listener")

            # defect_list latched yayinlanir; gec baglanan arayuz de listeyi alir.
            latched = QoSProfile(
                depth=1,
                history=QoSHistoryPolicy.KEEP_LAST,
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            )

            # BOTH arms. The doors job is a two-arm tour, so the charts show the
            # Kawasaki alongside the UR (the HARMONY dashboard only ever needed the
            # UR because its sensing ran on that arm alone).
            #
            # Names below are CANONICAL. Both the real cell and the Gazebo twin are
            # normalised onto them by _canonical_joint(), so the same joint keeps the
            # same key, colour and line style in both charts -- which is the only way
            # a real-vs-sim comparison is readable. Verified against the live cell:
            #
            #   /joint_states      ur10e_shoulder_pan_joint  joint1 .. joint6  world_to_agv
            #   /sim/joint_states  sim_ur10e_shoulder_pan_joint  sim_kawasaki_joint1 ..
            #                      sim_world_to_agv
            #
            # Note the real UR rail arrives as ur10e_base_to_robot_mount (prefixed),
            # and both topics also carry wheel_left_base / wheel_right_base, which are
            # not in this list and are therefore dropped.
            self.target_joints = [
                # UR10e + its linear rail
                "shoulder_pan_joint", "shoulder_lift_joint",
                "elbow_joint", "wrist_1_joint",
                "wrist_2_joint", "wrist_3_joint",
                "base_to_robot_mount",
                # Kawasaki RS005L + its AGV rail
                "joint1", "joint2", "joint3", "joint4", "joint5", "joint6",
                "world_to_agv",
            ]

            def _canonical_joint(name):
                """Map a real OR sim joint name onto the shared canonical name.

                Order matters: sim_ur10e_ and sim_kawasaki_ must be tested before the
                bare sim_ prefix, or sim_ur10e_elbow_joint would come out as
                ur10e_elbow_joint instead of elbow_joint.
                """
                if name.startswith("sim_ur10e_"):
                    return name[len("sim_ur10e_"):]
                if name.startswith("sim_kawasaki_"):
                    return name[len("sim_kawasaki_"):]
                if name.startswith("sim_"):
                    return name[len("sim_"):]        # sim_world_to_agv -> world_to_agv
                if name.startswith("ur10e_"):
                    return name[len("ur10e_"):]
                return name                          # joint1..6, world_to_agv

            def _collect_positions(msg, buffer):
                """Index positions by index, not by zip.

                zip(name, position) stops at the shortest sequence, so a publisher
                that sends a short array would silently drop joints. The arm
                broadcasters and the AGV rail bridge publish in SEPARATE messages
                here, so each message legitimately carries only part of the cell.
                """
                data = {"t": time.time()}
                positions = msg.position or []
                for i, name in enumerate(msg.name):
                    canonical = _canonical_joint(name)
                    if canonical in self.target_joints and i < len(positions):
                        data[canonical] = positions[i]
                if len(data) > 1:
                    with self._data_lock:
                        buffer.append(data)

            def joint_state_cb(msg):
                _collect_positions(msg, self.joint_position_data)

            def sim_joint_state_cb(msg):
                _collect_positions(msg, self.sim_position_data)

            def robot_status_cb(msg):
                try:
                    data = json.loads(msg.data)
                    state = data.get("state", "IDLE")
                    note = data.get("note", "")
                    with self._data_lock:
                        self.current_mode = state
                        self.current_note = note
                except (json.JSONDecodeError, KeyError):
                    pass

            def defect_cb(msg):
                """Birincil topic: defect basina tekil JSON."""
                try:
                    self._upsert_defect(json.loads(msg.data))
                except (json.JSONDecodeError, TypeError, ValueError):
                    return
                self.socketio.emit("defect_list", {"defects": self._defect_snapshot()})

            def defect_list_cb(msg):
                """Yedek topic: tum defect'ler tek JSON array icinde."""
                try:
                    payloads = json.loads(msg.data)
                except (json.JSONDecodeError, TypeError, ValueError):
                    return
                if not isinstance(payloads, list):
                    return
                for payload in payloads:
                    if isinstance(payload, dict):
                        self._upsert_defect(payload)
                snapshot = self._defect_snapshot()
                self.socketio.emit("defect_list", {"defects": snapshot})
                # Bu topic sensing turu bittiginde bir kez yayinlanir; sensing'in
                # tamamlandiginin en guvenilir isareti budur.
                if snapshot:
                    self._notify_sensing_complete(len(snapshot))

            def defect_status_cb(msg):
                """Cleaning node'undan gelen DETECTED/CLEANING/CLEANED guncellemeleri."""
                try:
                    data = json.loads(msg.data)
                    defect_id = str(data.get("defect_id", "")).strip()
                    status = str(data.get("status", "")).strip()
                except (json.JSONDecodeError, TypeError, ValueError):
                    return
                if not defect_id or not status:
                    return
                with self._data_lock:
                    if defect_id in self.defects:
                        self.defects[defect_id]["status"] = status
                self.socketio.emit("defect_list", {"defects": self._defect_snapshot()})

            def force_cb(msg):
                t = time.time()
                force_z = msg.wrench.force.z
                with self._data_lock:
                    self.force_data.append({"t": t, "force_z": force_z})

            def scenario_event_cb(msg):
                """Sensing node'undan gelen senaryo olayları.

                Only CLEANING_COMPLETE is acted on here. doors_mission_node still
                publishes LAST_VIEWPOINT_REACHED when the tour ends -- it is a real
                event and other tools may use it -- but this dashboard ignores it:
                the KPI3 scenario fault belongs to the HARMONY validation run, not to
                the doors job, so it was removed here rather than left firing.
                """
                try:
                    data = json.loads(msg.data)
                    event = str(data.get("event", "")).strip().upper()
                except (json.JSONDecodeError, TypeError, ValueError):
                    return
                if event == "CLEANING_COMPLETE":
                    self._notify_cleaning_complete(data)

            # Kalıcı komut yayıncısı (buton tıklamaları buradan gider).
            self._cmd_pub = node.create_publisher(String, "/harmony/cmd_input", 10)

            node.create_subscription(JointState, "/joint_states", joint_state_cb, 10)
            # The Gazebo twin's mirror of the SAME 16 joints, sim_-prefixed. NOT
            # /sim/sim_joint_states -- that one carries 32 names including the
            # barriers, tables, kabinet and the human model, none of which belong on
            # a robot-joint chart. Confirmed against the running cell.
            node.create_subscription(
                JointState, "/sim/joint_states", sim_joint_state_cb, 10)
            node.create_subscription(String, "/harmony/robot_status", robot_status_cb, 10)
            node.create_subscription(String, TOPIC_DEFECT, defect_cb, 10)
            node.create_subscription(String, TOPIC_DEFECT_LIST, defect_list_cb, latched)
            node.create_subscription(String, TOPIC_DEFECT_STATUS, defect_status_cb, 10)
            node.create_subscription(WrenchStamped, "/harmony/cleaning/force", force_cb, 10)
            node.create_subscription(String, "/harmony/scenario_event", scenario_event_cb, 10)

            node.get_logger().info("Doors dashboard listener started.")

            while self._running and rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.1)

            self._cmd_pub = None
            node.destroy_node()
            rclpy.shutdown()

        except Exception as e:
            self._cmd_pub = None
            print(f"[ROS2DataCollector] ROS2 error: {e}")
            print("[ROS2DataCollector] ROS2 data monitoring disabled.")

    def _periodic_push(self):
        """Push data to frontend every 200ms."""
        while self._running:
            time.sleep(0.2)

            try:
                now = time.time()
                cutoff = now - self.buffer_seconds

                with self._data_lock:
                    pos_filtered = [d for d in self.joint_position_data if d["t"] > cutoff]
                    sim_filtered = [d for d in self.sim_position_data if d["t"] > cutoff]
                    force_filtered = [d for d in self.force_data if d["t"] > cutoff]

                    # Mode mapping
                    mode_map = {
                        "SR_MODE": "Sensing Robot",
                        "CR_MODE": "Cleaning Robot",
                        "WAITING": "WAITING",
                        "IDLE": "IDLE",
                    }
                    mode_display = mode_map.get(self.current_mode, self.current_mode)
                    note = self.current_note
                    defect = len(self.defect_order)

                pos_sampled = self._downsample(pos_filtered, 100)
                sim_sampled = self._downsample(sim_filtered, 100)
                force_sampled = self._downsample(force_filtered, 100)

                self.socketio.emit("chart_data", {
                    "positions": pos_sampled,
                    "sim_positions": sim_sampled,
                    "forces": force_sampled,
                    "now": now,
                    "window": self.buffer_seconds,
                })

                self.socketio.emit("robot_info", {
                    "mode": mode_display,
                    "note": note,
                    "defect_count": defect,
                })

            except Exception as e:
                print(f"[ROS2DataCollector] Push error: {e}")

    def _downsample(self, data, max_points):
        """Simple downsample by skipping entries."""
        if len(data) <= max_points:
            return data
        step = len(data) / max_points
        return [data[int(i * step)] for i in range(max_points)]

    def stop(self):
        self._running = False


data_collector = ROS2DataCollector(socketio)


# ==============================================================================
# Command Publisher (ROS2 topic pub)
# ==============================================================================

class CommandPublisher:
    """Publishes commands to /harmony/cmd_input.

    Birincil yol, ROS2DataCollector'un kalıcı node'u üzerinden doğrudan
    yayınlamaktır: `ros2 topic pub -1` her seferinde yeni bir node kurar ve
    discovery tamamlanmadan çıkabildiği için mesaj sessizce düşerdi (butona
    birkaç kez basmak gerekiyordu). Subprocess yolu yalnızca ROS başlatılamamışsa
    yedek olarak kullanılır.
    """

    @staticmethod
    def publish(cmd_name):
        if data_collector.publish_cmd(cmd_name):
            return
        print(f"[CommandPublisher] ROS unavailable, falling back to subprocess for {cmd_name}")
        CommandPublisher._publish_via_subprocess(cmd_name)

    @staticmethod
    def _publish_via_subprocess(cmd_name):
        cmd = (
            f'ros2 topic pub /harmony/cmd_input std_msgs/msg/String '
            f'"{{data: \'{{\\\"cmd\\\":\\\"{cmd_name}\\\"}}\'}}" -1'
        )
        full_cmd = f"source /opt/ros/humble/setup.bash && source /home/cem/colcon_ws/install/setup.bash && {cmd}"

        my_env = os.environ.copy()
        if "QT_QPA_PLATFORM_PLUGIN_PATH" in my_env:
            del my_env["QT_QPA_PLATFORM_PLUGIN_PATH"]

        try:
            process = subprocess.Popen(
                full_cmd,
                shell=True,
                executable="/bin/bash",
                env=my_env,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
            )
            # Don't wait — fire and forget (with timeout thread)
            def _wait():
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    process.kill()
            threading.Thread(target=_wait, daemon=True).start()
        except Exception as e:
            print(f"[CommandPublisher] Error publishing {cmd_name}: {e}")


cmd_publisher = CommandPublisher()


# ==============================================================================
# Camera Streaming
# ==============================================================================

class CameraStreamer:
    """MJPEG streaming for the Gazebo camera and the real-world camera.

    The Gazebo half is taken from user_interface/app.py: the twin's camera is not a
    ROS topic here, it is published on the Gazebo transport bus, so it is read with
    gz.transport rather than rclpy. Both halves degrade quietly -- a missing
    gz.transport or an unreachable RTSP endpoint disables that feed and leaves the
    other one running.
    """

    def __init__(self):
        self._gazebo_frame = None
        self._real_frame = None
        self._gazebo_lock = threading.Lock()
        self._real_lock = threading.Lock()
        self._gazebo_running = False
        self._real_running = False

    def start_gazebo_stream(self):
        """Start the Gazebo camera via gz.transport."""
        self._gazebo_running = True
        thread = threading.Thread(target=self._gazebo_worker, daemon=True)
        thread.start()

    def _gazebo_worker(self):
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

    def generate_gazebo(self):
        """MJPEG generator for the Gazebo camera."""
        while True:
            with self._gazebo_lock:
                frame = self._gazebo_frame
            if frame:
                yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"
                       + frame + b"\r\n")
            time.sleep(0.05)

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

                    start_time = time.time()
                    rviz_win = self._get_rviz_window()

                    while self._real_running and (time.time() - start_time) < 5:
                        if not rviz_win:
                            rviz_win = self._get_rviz_window()

                        rviz_frame = self._try_rviz_capture(rviz_win) if rviz_win else None

                        if rviz_frame:
                            with self._real_lock:
                                self._real_frame = rviz_frame
                            time.sleep(0.05)
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
    return jsonify(process_mgr.get_status())


# ==============================================================================
# SocketIO Events
# ==============================================================================

@socketio.on("connect")
def handle_connect():
    emit("status_update", process_mgr.get_status())
    # Geç bağlanan istemci mevcut defect tablosunu hemen görsün.
    emit("defect_list", {"defects": data_collector._defect_snapshot()})

@socketio.on("start_hil")
def handle_start_hil(data):
    use_fake_hardware = data.get("use_fake_hardware", False)
    process_mgr.start_hil(use_fake_hardware)

@socketio.on("confirm_robot")
def handle_confirm_robot(data=None):
    data = data or {}
    process_mgr.confirm_robot_ready(
        only_sim=bool(data.get("only_sim", True)),
        force_replan=bool(data.get("force_replan", False)),
    )

@socketio.on("stop_all")
def handle_stop_all():
    thread = threading.Thread(target=process_mgr.stop_all, daemon=True)
    thread.start()

@socketio.on("reset_mission")
def handle_reset_mission():
    """Arayüzü sıfırlar (defect'ler, sayaçlar, mod, grafik/log gösterimi)."""
    data_collector.reset_ui()
    process_mgr._emit_log("SYSTEM", "🔄 Arayüz sıfırlandı — yeni mission'a hazır.")

#: /harmony/cmd_input üzerinden yayınlanabilen komutlar
VALID_CMDS = ("START", "CONFIRM", "REINSPECT", "STOP")


@socketio.on("publish_cmd")
def handle_publish_cmd(data):
    cmd = data.get("cmd", "")
    if cmd in VALID_CMDS:
        # Yeni bir tarama başlıyorsa eski defect tablosunu temizle.
        if cmd == "START":
            data_collector.clear_defects()
        cmd_publisher.publish(cmd)
        process_mgr._emit_log("SYSTEM", f"📡 Published command: {cmd}")
    else:
        process_mgr._emit_log("SYSTEM", f"❌ Unknown command: {cmd}")


# ==============================================================================
# Main Entry Point
# ==============================================================================

def main():
    print("=" * 60)
    print("  Doors — Multirobot Sensing / Cleaning Dashboard")
    print("  http://localhost:8082")
    print("=" * 60)

    # Start camera streamers
    camera_streamer.start_gazebo_stream()
    camera_streamer.start_real_stream()

    # Start ROS2 data collector
    data_collector.start()

    try:
        # 8082, not 8081: the HARMONY dashboard keeps that port so both can run.
        socketio.run(app, host="0.0.0.0", port=8082, debug=False, allow_unsafe_werkzeug=True)
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        camera_streamer.stop()
        data_collector.stop()
        process_mgr.stop_all()


if __name__ == "__main__":
    main()
