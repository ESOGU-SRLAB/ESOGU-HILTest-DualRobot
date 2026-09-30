#!/usr/bin/env python3
"""Doors mission orchestrator — the doors counterpart of pymoveit2_real's
sensing_robot.py.

WHY THIS EXISTS
───────────────
The HARMONY dashboard drives everything through /harmony/cmd_input and listens on
/harmony/robot_status, the defect topics and /harmony/scenario_event. Its sensing
side is a LONG-LIVED node that waits for START.

The doors inspection is the opposite shape: ur_inspection_node and
kawasaki_inspection_node start moving the moment they come up (inspection_base.run()
has no command gate) and the processes EXIT when the tour is done. They publish no
status at all — only log lines.

This node bridges the two. It speaks exactly the same topic contract as
sensing_robot.py, so the dashboard works unchanged, but instead of driving an arm
itself it owns the doors_inspection launch as a subprocess:

    START     -> spawn `ros2 launch doors_inspection doors_inspection.launch.py`
                 and translate its stdout into /harmony/robot_status progress
    tour ends -> LAST_VIEWPOINT_REACHED (the dashboard's KPI3 fault pop-up), then
                 publish the four scenario defects + RViz markers
    CONFIRM   -> hand over to cleaning_mission_runner (which drives the UR only)
    STOP      -> cancel the running trajectories, WAIT FOR THE ARMS TO STOP, and
                 only then tear the launch down

MEASURED, AND THE REASON END DETECTION LOOKS THE WAY IT DOES: the inspection launch
does NOT exit when the tour is over. multirobot_inspection.launch.py also starts
multirobot_viewpoint_visualizer, a permanent node, and registers no on_exit handler,
so `ros2 launch` stays up indefinitely after both arm executors have exited (verified
by process listing: both *_inspection_node processes gone, launch + visualizer still
alive). Waiting on the launch process would therefore hang forever. Instead the tour
is considered finished when the arm executor processes appear and then disappear, and
this node tears the launch down itself.

Deliberately NOT done here: gating the inspection nodes on a command. That would
mean editing multirobot_viewpoint_planner, which the validated chassis job shares.
Everything in this file is additive.

The defect coordinates come from pymoveit2_real.harmony_defects — the SAME module
the HARMONY scenario uses, imported rather than copied so the two scenarios can
never drift apart. Those four defects already sit on the doors (all four x values
fall inside the door bounding box), which is why the cleaning side needs no changes.
"""

from __future__ import annotations

import json
import os
import re
import shlex
import signal
import subprocess
import threading
import time
from datetime import datetime
from typing import List, Optional

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)

from action_msgs.srv import CancelGoal
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray

# Single source of truth for the scenario defects, shared with HARMONY.
from pymoveit2_real import harmony_defects as hd


def _now_ros(node: Node) -> str:
    t = node.get_clock().now().to_msg()
    return f"{t.sec}.{t.nanosec:09d}"


def _latched_qos(depth: int = 1) -> QoSProfile:
    """So a late-joining dashboard or RViz still sees the last message."""
    return QoSProfile(
        depth=depth,
        history=QoSHistoryPolicy.KEEP_LAST,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    )


#: "[UR 4/9] Moving UR->ur_vp_004."  — inspection_base.py:1755
RE_PROGRESS = re.compile(r"\[(\w[\w ]*?) (\d+)/(\d+)\]\s+Moving")
#: "[UR] sequence complete: 9/9 viewpoints captured."  — inspection_base.py:1763
RE_COMPLETE = re.compile(r"\[(\w[\w ]*?)\]\s+sequence complete:\s*(\d+)/(\d+)")


class DoorsMissionNode(Node):

    def __init__(self):
        super().__init__("doors_mission_node")

        # ---------------- Params ----------------
        self.declare_parameter("fixed_frame", "world")
        # Forwarded to doors_inspection.launch.py. only_sim decides WHICH CAMERAS are
        # captured; it is not the same idea as the HIL launch's use_fake_hardware
        # (which decides whether the ROBOT is real), even though with fake hardware
        # only_sim=true is the only sensible combination.
        self.declare_parameter("only_sim", True)
        # Cached trajectories are replayed WITHOUT re-checking collision, so this must
        # be flipped once after any change to the collision model.
        self.declare_parameter("force_replan", False)
        # Escape hatch: anything else to append, e.g. "kawasaki_velocity:=0.05".
        self.declare_parameter("extra_inspection_args", "")
        self.declare_parameter("defect_publish_interval", 0.3)
        self.declare_parameter("marker_publish_rate_hz", 1.0)
        # Status heartbeat. Without one, robot_status is only published on state
        # changes, so a dashboard that connects (or reconnects) mid-run sees nothing
        # until the next event. sensing_robot.py avoids this by republishing from its
        # TCP-pose timer; this is the same idea stated explicitly.
        self.declare_parameter("status_heartbeat_hz", 1.0)
        # STOP path. These default to the same values the inspection nodes use, so a
        # cancel here reaches the very goals they sent.
        self.declare_parameter("ur_controller_action",
                               "/scaled_joint_trajectory_controller/follow_joint_trajectory")
        self.declare_parameter("kawasaki_controller_action",
                               "/kawasaki/kawasaki_controller/follow_joint_trajectory")
        self.declare_parameter("stop_settle_timeout_s", 8.0)
        self.declare_parameter("stop_still_velocity", 0.01)   # rad/s or m/s
        # How long the arm executors may take to appear after the launch starts.
        # Generous: move_group discovery and the planning-scene setup happen first.
        self.declare_parameter("inspection_start_timeout_s", 90.0)

        self.fixed_frame = self.get_parameter("fixed_frame").value
        self.defect_publish_interval = float(
            self.get_parameter("defect_publish_interval").value)
        marker_rate = float(self.get_parameter("marker_publish_rate_hz").value)

        # ---------------- Publishers ----------------
        self.robot_status_pub = self.create_publisher(String, "/harmony/robot_status", 10)
        self.scenario_event_pub = self.create_publisher(String, "/harmony/scenario_event", 10)
        self.defect_pub = self.create_publisher(String, hd.TOPIC_DEFECT, 10)
        self.defect_list_pub = self.create_publisher(
            String, hd.TOPIC_DEFECT_LIST, _latched_qos())
        self.marker_pub = self.create_publisher(
            MarkerArray, hd.TOPIC_DEFECT_MARKERS, _latched_qos())

        # ---------------- Subscribers ----------------
        cbg = ReentrantCallbackGroup()
        self.create_subscription(String, "/harmony/cmd_input", self._cmd_cb, 10,
                                 callback_group=cbg)
        self.create_subscription(String, hd.TOPIC_DEFECT_STATUS,
                                 self._defect_status_cb, 10, callback_group=cbg)
        # Only used to confirm the arms have actually stopped after a STOP.
        self.create_subscription(JointState, "/joint_states", self._joint_state_cb, 10,
                                 callback_group=cbg)

        # Cancel-all clients. The action spec treats a CancelGoal request with a zero
        # goal_id and zero stamp as "cancel every goal on this server", so a plain
        # service client is enough — no ActionClient and no goal handle bookkeeping.
        self._cancel_clients = {}
        for tag, param in (("UR", "ur_controller_action"),
                           ("Kawasaki", "kawasaki_controller_action")):
            action = self.get_parameter(param).value
            self._cancel_clients[tag] = self.create_client(
                CancelGoal, f"{action}/_action/cancel_goal", callback_group=cbg)

        # ---------------- State ----------------
        self._mode_lock = threading.Lock()
        self.state = "IDLE"
        self.mode = "IDLE"
        self._note = "Waiting for START"
        self._level = "INFO"

        self._proc: Optional[subprocess.Popen] = None
        self._proc_lock = threading.Lock()
        self.start_requested = threading.Event()
        self.stop_requested = threading.Event()

        self._marker_lock = threading.Lock()
        self._defects_reported = False
        self._defect_status = {}

        # arm label -> (captured, planned), scraped from the launch's log lines.
        self._result_lock = threading.Lock()
        self._arm_results = {}

        self._vel_lock = threading.Lock()
        self._max_abs_vel = 0.0
        self._vel_stamp = 0.0

        if marker_rate > 0:
            self.create_timer(1.0 / marker_rate, self._publish_markers)
        hb = float(self.get_parameter("status_heartbeat_hz").value)
        if hb > 0:
            self.create_timer(1.0 / hb, self._heartbeat)

        self.worker = threading.Thread(target=self._run_state_machine, daemon=True)
        self.worker.start()

        self._publish_robot_status("IDLE", "IDLE", "Waiting for START")
        self.get_logger().info(
            f"DoorsMissionNode ready | only_sim={self.get_parameter('only_sim').value} "
            f"| force_replan={self.get_parameter('force_replan').value}")

    # ================================================================== #
    # Status
    # ================================================================== #
    def _publish_robot_status(self, state: str, mode: str, note: str, level: str = "INFO"):
        # Remember it so the heartbeat can repeat the latest line verbatim.
        with self._mode_lock:
            self._note = note
            self._level = level
        payload = {
            "timestamp": _now_ros(self),
            "state": state,
            "mode": mode,
            "level": level,
            "note": note,
            "frame_id": self.fixed_frame,
        }
        msg = String()
        msg.data = json.dumps(payload)
        self.robot_status_pub.publish(msg)

    def _heartbeat(self):
        with self._mode_lock:
            state, mode, note, level = self.state, self.mode, self._note, self._level
        self._publish_robot_status(state, mode, note, level=level)

    def _set_state(self, state: str, mode: str, note: str, level: str = "INFO"):
        with self._mode_lock:
            self.state = state
            self.mode = mode
        self._publish_robot_status(state, mode, note, level=level)
        self.get_logger().info(f"State change: {state}/{mode} - {note}")

    def _publish_scenario_event(self, event: str, note: str = "", extra=None):
        payload = {"event": event, "note": note, "timestamp": _now_ros(self)}
        if extra:
            payload.update(extra)
        msg = String()
        msg.data = json.dumps(payload)
        self.scenario_event_pub.publish(msg)
        self.get_logger().info(f"Scenario event published: {event}")

    # ================================================================== #
    # Commands
    # ================================================================== #
    def _cmd_cb(self, msg: String):
        try:
            cmd = str(json.loads(msg.data).get("cmd", "")).upper().strip()
        except Exception:
            self.get_logger().warning(
                f"Invalid JSON on /harmony/cmd_input: {msg.data[:200]}")
            return

        self.get_logger().info(f"Command received: {cmd}")

        if cmd in ("START", "REINSPECT"):
            if self._inspection_running():
                self.get_logger().warning(
                    f"{cmd} ignored: an inspection is already running.")
                return
            self.stop_requested.clear()
            self._reset_defects()
            self.start_requested.set()
            self._set_state("SR_MODE", "SR", f"{cmd} received, doors inspection begins")

        elif cmd == "STOP":
            self.stop_requested.set()
            self.start_requested.clear()
            # Run the stop sequence off the callback thread: cancelling and waiting
            # for the arms takes seconds and must not block the executor.
            threading.Thread(target=self._stop_sequence, daemon=True).start()

        elif cmd == "CONFIRM":
            self._set_state("CR_MODE", "CR",
                            "CONFIRM received (cleaning handled by CR node)")

        elif cmd == "WAITING":
            self._set_state("WAITING", "SR", "WAITING command received, staying in WAITING")

        else:
            self.get_logger().warning(f"Unknown command: {cmd}")

    # ================================================================== #
    # Defects / markers  (mirrors sensing_robot.py so RViz behaves the same)
    # ================================================================== #
    def _reset_defects(self):
        with self._marker_lock:
            self._defects_reported = False
            self._defect_status = {}

    def _defect_status_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            defect_id = str(data.get("defect_id", "")).strip()
            status = str(data.get("status", "")).strip()
        except Exception:
            return
        if not defect_id or not status:
            return
        with self._marker_lock:
            self._defect_status[defect_id] = status

    def _publish_markers(self):
        """Keeps the marker topic alive at all times: DELETEALL before sensing has
        reported, the real array afterwards. RViz can then select the topic from the
        start without defects appearing before the scenario says so."""
        stamp = self.get_clock().now().to_msg()
        with self._marker_lock:
            reported = self._defects_reported
            statuses = dict(self._defect_status)
        if not reported:
            self.marker_pub.publish(
                hd.build_empty_marker_array(frame_id=self.fixed_frame, stamp=stamp))
        else:
            self.marker_pub.publish(
                hd.build_marker_array(statuses=statuses, frame_id=self.fixed_frame,
                                      stamp=stamp))

    def _publish_defects(self):
        stamp_iso = datetime.now().isoformat()
        payloads = hd.all_payloads(frame_id=self.fixed_frame, timestamp=stamp_iso)

        for payload in payloads:
            msg = String()
            msg.data = json.dumps(payload)
            self.defect_pub.publish(msg)
            p = payload["position"]
            self.get_logger().info(
                f"Defect published: {payload['defect_id']} ({payload['defect_type']}) "
                f"x={p['x']:.3f} y={p['y']:.3f} z={p['z']:.3f}")
            if self.defect_publish_interval > 0:
                time.sleep(self.defect_publish_interval)

        list_msg = String()
        list_msg.data = json.dumps(payloads)
        self.defect_list_pub.publish(list_msg)

        with self._marker_lock:
            self._defects_reported = True
            self._defect_status = {p["defect_id"]: p["status"] for p in payloads}
        self._publish_markers()

        self._publish_robot_status(
            "SR_MODE", "SR", f"Sensing complete. {len(payloads)} defects reported")

    # ================================================================== #
    # Inspection subprocess
    # ================================================================== #
    def _inspection_running(self) -> bool:
        with self._proc_lock:
            return self._proc is not None and self._proc.poll() is None

    def _build_inspection_cmd(self) -> List[str]:
        only_sim = "true" if self.get_parameter("only_sim").value else "false"
        force_replan = "true" if self.get_parameter("force_replan").value else "false"
        cmd = ["ros2", "launch", "doors_inspection", "doors_inspection.launch.py",
               f"only_sim:={only_sim}", f"force_replan:={force_replan}"]
        extra = str(self.get_parameter("extra_inspection_args").value or "").strip()
        if extra:
            cmd += shlex.split(extra)
        return cmd

    def _spawn_inspection(self) -> bool:
        cmd = self._build_inspection_cmd()
        self.get_logger().info("Launching inspection: " + " ".join(cmd))
        try:
            proc = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                # Own process group so the whole launch tree can be signalled at once.
                preexec_fn=os.setsid,
            )
        except Exception as e:
            self.get_logger().error(f"Failed to launch doors_inspection: {e}")
            self._set_state("IDLE", "IDLE", f"Inspection launch failed: {e}", level="ERROR")
            return False
        with self._proc_lock:
            self._proc = proc
        return True

    def _consume_inspection_output(self, proc: subprocess.Popen):
        """Turn the inspection's log lines into robot_status progress, and record each
        arm's final tally.

        Reading stdout is what keeps this node out of multirobot_viewpoint_planner:
        the two log lines matched here already existed, so no shared code changes to
        get a progress indicator. If those lines are ever reworded, progress silently
        degrades to "running" and the completion tally is empty — the run itself and
        its end detection (which watches processes, not text) are unaffected.

        Runs on its own thread: the tour is detected as finished by polling for the
        inspection node processes, which has to happen while stdout is still draining.
        """
        if proc.stdout is None:
            return
        per_arm = {}
        try:
            for line in iter(proc.stdout.readline, ""):
                if not line:
                    break
                line = line.rstrip()
                if self.stop_requested.is_set():
                    # Keep draining so the child never blocks on a full pipe, but stop
                    # reporting progress for a run that is being torn down.
                    continue

                m = RE_PROGRESS.search(line)
                if m:
                    arm, idx, total = m.group(1), int(m.group(2)), int(m.group(3))
                    per_arm[arm] = (idx, total)
                    note = "  ".join(f"{a} {i}/{n}" for a, (i, n) in sorted(per_arm.items()))
                    self._publish_robot_status("SR_MODE", "SR", f"Scanning: {note}")
                    continue

                m = RE_COMPLETE.search(line)
                if m:
                    arm, saved, total = m.group(1), int(m.group(2)), int(m.group(3))
                    with self._result_lock:
                        self._arm_results[arm] = (saved, total)
                    self._publish_robot_status(
                        "SR_MODE", "SR", f"{arm} finished: {saved}/{total} viewpoints")
        except (ValueError, OSError):
            pass

    def _inspection_node_pids(self, pgid: int) -> List[str]:
        """PIDs of the arm executors still alive inside the launch's process group."""
        try:
            out = subprocess.run(
                ["pgrep", "-g", str(pgid), "-f", "inspection_node"],
                capture_output=True, text=True, timeout=5).stdout
        except (subprocess.SubprocessError, OSError):
            return []
        return [p for p in out.split() if p]

    def _await_tour_end(self, proc: subprocess.Popen) -> str:
        """Block until the two arm executors have finished.

        WHY NOT WAIT FOR THE LAUNCH TO EXIT: multirobot_inspection.launch.py also
        starts multirobot_viewpoint_visualizer, which is a permanent node, and the
        launch registers no on_exit handler. So `ros2 launch` keeps running long after
        both arms are done — waiting on it would hang forever. Watching the executor
        processes is independent of that, and of the log wording.

        Returns "done", "stopped", or "died".
        """
        pgid = os.getpgid(proc.pid)
        grace_deadline = time.monotonic() + float(
            self.get_parameter("inspection_start_timeout_s").value)
        seen_nodes = False

        while True:
            if self.stop_requested.is_set():
                return "stopped"
            if proc.poll() is not None:
                return "died"

            pids = self._inspection_node_pids(pgid)
            if pids:
                seen_nodes = True
            elif seen_nodes:
                # They came up and are now all gone: the tour is over, one way or
                # another. Give stdout a moment to flush the final lines.
                time.sleep(1.0)
                return "done"
            elif time.monotonic() > grace_deadline:
                self.get_logger().error(
                    "Inspection nodes never appeared; is the launch failing?")
                return "died"

            time.sleep(1.0)

    def _run_state_machine(self):
        while rclpy.ok():
            if not self.start_requested.wait(timeout=0.5):
                continue
            self.start_requested.clear()
            if self.stop_requested.is_set():
                continue

            self._publish_robot_status("SR_MODE", "SR", "Starting doors inspection")
            with self._result_lock:
                self._arm_results = {}
            if not self._spawn_inspection():
                continue

            with self._proc_lock:
                proc = self._proc
            reader = threading.Thread(
                target=self._consume_inspection_output, args=(proc,), daemon=True)
            reader.start()

            outcome = self._await_tour_end(proc)

            # The launch never exits on its own (the visualizer is permanent), so the
            # orchestrator always tears it down. Leaving it up would also make the next
            # START collide with a second visualizer on the same topic.
            self._teardown_inspection(cancel_first=(outcome == "stopped"))
            reader.join(timeout=3.0)

            with self._result_lock:
                results = dict(self._arm_results)
            captured = sum(s for s, _ in results.values())
            summary = ", ".join(f"{a} {s}/{t}" for a, (s, t) in sorted(results.items()))

            if outcome == "stopped" or self.stop_requested.is_set():
                self._set_state("IDLE", "IDLE", "Inspection stopped by operator",
                                level="WARN")
                continue

            if outcome == "died":
                self._set_state("IDLE", "IDLE",
                                "Inspection launch ended unexpectedly", level="ERROR")
                continue

            if not results or captured == 0:
                # The arms ran but captured nothing — a missing plan file, an
                # unavailable controller, no /joint_states. Treated as a failure so the
                # operator is never shown defects for a tour that did not happen.
                self._set_state(
                    "IDLE", "IDLE",
                    "Inspection captured no viewpoints; check the launch log",
                    level="ERROR")
                continue

            self._publish_robot_status("SR_MODE", "SR",
                                       f"Inspection complete: {summary}")

            # Whole-tour completion is the doors equivalent of HARMONY's
            # "last viewpoint reached": with two arms there is no single last
            # viewpoint, so the end of the tour is the only unambiguous moment.
            self._publish_scenario_event(
                "LAST_VIEWPOINT_REACHED",
                "Doors inspection tamamlandı, defect'ler yayınlanıyor")
            self._publish_defects()
            self._set_state("WAITING", "SR", "Waiting for CONFIRM (or REINSPECT/STOP)")

    # ================================================================== #
    # STOP
    # ================================================================== #
    def _joint_state_cb(self, msg: JointState):
        if not msg.velocity:
            return
        with self._vel_lock:
            self._max_abs_vel = max(abs(v) for v in msg.velocity)
            self._vel_stamp = time.monotonic()

    def _cancel_all_trajectories(self):
        """Cancel every goal on both arm controllers.

        An empty CancelGoal request (zero goal_id, zero stamp) means "cancel all" per
        the action spec, so this reaches the goals the inspection nodes sent without
        this node having to know anything about them.
        """
        for tag, client in self._cancel_clients.items():
            if not client.service_is_ready():
                if not client.wait_for_service(timeout_sec=1.0):
                    self.get_logger().warning(
                        f"{tag}: cancel service unavailable; skipping cancel.")
                    continue
            try:
                client.call_async(CancelGoal.Request())
                self.get_logger().info(f"{tag}: cancel-all sent.")
            except Exception as e:
                self.get_logger().warning(f"{tag}: cancel failed: {e}")

    def _wait_until_still(self) -> bool:
        """Block until the reported joint velocities settle, or the timeout expires."""
        timeout = float(self.get_parameter("stop_settle_timeout_s").value)
        thresh = float(self.get_parameter("stop_still_velocity").value)
        deadline = time.monotonic() + timeout
        still_since = None
        while time.monotonic() < deadline:
            with self._vel_lock:
                vmax, stamp = self._max_abs_vel, self._vel_stamp
            fresh = stamp > 0 and (time.monotonic() - stamp) < 1.0
            if fresh and vmax < thresh:
                still_since = still_since or time.monotonic()
                if time.monotonic() - still_since > 0.5:
                    self.get_logger().info(
                        f"Arms settled (max |v| = {vmax:.4f}).")
                    return True
            else:
                still_since = None
            time.sleep(0.05)
        self.get_logger().warning(
            "Arms did not report a settled velocity before the timeout; "
            "tearing the launch down anyway.")
        return False

    def _teardown_inspection(self, cancel_first: bool):
        """Shut the inspection launch down and forget it.

        cancel_first is for the STOP path: on real hardware, killing the launch alone
        leaves the goal the controller is ALREADY executing untouched, so the arm
        would keep moving with nothing left to cancel it. On the normal path the arms
        have finished, so there is nothing to cancel and the extra service call would
        only slow the handover to cleaning down.
        """
        with self._proc_lock:
            proc = self._proc
            self._proc = None
        if proc is None:
            return
        if cancel_first:
            self._cancel_all_trajectories()
            self._wait_until_still()
        if proc.poll() is None:
            self._kill_process(proc)

    def _stop_sequence(self):
        """Cancel trajectories, confirm the arms stopped, then tear the launch down."""
        self._publish_robot_status("SR_MODE", "SR",
                                   "STOP received, cancelling trajectories", level="WARN")
        self._teardown_inspection(cancel_first=True)
        self._set_state("IDLE", "IDLE", "STOP complete, back to IDLE", level="WARN")

    def _kill_process(self, proc: subprocess.Popen):
        for sig, name, wait_s in ((signal.SIGINT, "SIGINT", 10),
                                  (signal.SIGTERM, "SIGTERM", 5),
                                  (signal.SIGKILL, "SIGKILL", 3)):
            try:
                os.killpg(os.getpgid(proc.pid), sig)
            except (ProcessLookupError, OSError):
                return
            self.get_logger().info(f"Inspection launch: sent {name}.")
            try:
                proc.wait(timeout=wait_s)
                return
            except subprocess.TimeoutExpired:
                continue

    # ================================================================== #
    def destroy_node(self):
        with self._proc_lock:
            running = self._proc is not None and self._proc.poll() is None
        if running:
            self.get_logger().info("Shutting down: stopping the inspection launch.")
            self._teardown_inspection(cancel_first=True)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DoorsMissionNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
