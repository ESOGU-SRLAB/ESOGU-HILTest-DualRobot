#!/usr/bin/env python3
"""
Linear axis (Festo CMMT / EdCon, PROFIDRIVE telegram 111) <-> ros2_control adapter.

DATA FLOW
    scaled_joint_trajectory_controller  (command_interfaces: [position])
        -> writes the rail position command interface at controller-manager rate
    topic_based_ros2_control/TopicBasedSystem
        -> publishes it as JointState on /linear_axis/target_position_cmd
    THIS NODE
        -> streams that setpoint to the drive and publishes the measured rail
           state on /linear_axis/joint_states, which TopicBasedSystem reads back
           as its state interface. That topic is DEDICATED on purpose: the shared
           /joint_states already carries this joint, republished by the
           joint_state_broadcaster from the very state interface we are feeding,
           so using it would close a loop in which TopicBasedSystem consumes its
           own output. The joint_state_broadcaster still forwards the resulting
           (correct) rail state to /joint_states for MoveIt, TF and the bridges.

The setpoint stream arriving on /linear_axis/target_position_cmd is ALREADY the
MoveIt trajectory sampled at controller rate. It must be passed through untouched:
any smoothing here is pure tracking lag. The drive is therefore driven in
CONTINUOUS UPDATE mode -- setpoints are accepted without re-triggering a traversing
task -- with a speed limit derived from the setpoint's own time derivative plus a
proportional term on the following error.

WHY THE MODBUS ACCESS LOOKS UNUSUAL
    ComModbus owns a background I/O thread that performs the actual Modbus
    transaction every `cycle_time` ms. TelegramHandler.update_inputs/update_outputs
    call recv_io/send_io in BLOCKING mode, i.e. each one waits for the next I/O
    cycle (~10 ms). Calling them from ROS timer callbacks oversubscribes the
    executor by an order of magnitude and makes every timer run late. So all drive
    access lives in one dedicated thread that performs exactly ONE non-blocking
    exchange per control cycle, and ROS callbacks only touch plain Python floats.

    For the same reason we do NOT call MotionHandler.position_task(): its
    ready_for_motion() precondition issues a blocking update_inputs() and, when the
    drive is not operational, silently drops the command while logging
    "Traversing task aborted". Readiness is tracked by our own state machine from
    the ZSW1 status word instead, and faults are actually acted upon.
"""
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from controller_manager_msgs.srv import SwitchController
from std_srvs.srv import Trigger
from ur_dashboard_msgs.srv import Popup

from edcon.edrive.motion_handler import MotionHandler
from edcon.edrive.com_modbus import ComModbus


# Drive lifecycle states.
ST_DISCONNECTED = "DISCONNECTED"   # no Modbus link / handler
ST_ENABLING = "ENABLING"           # link up, acknowledging faults + powerstage
ST_RUNNING = "RUNNING"             # operation_enabled, streaming setpoints
ST_FAULT = "FAULT"                 # drive reported a fault, awaiting recovery


class LinearAxisControllerAdapter(Node):
    def __init__(self):
        super().__init__('linear_axis_controller_adapter')

        # --- Parameters -------------------------------------------------- #
        self.declare_parameter('ip_address', '192.168.3.1')
        self.declare_parameter('joint_name', 'ur10e_base_to_robot_mount')
        self.declare_parameter('command_topic', '/linear_axis/target_position_cmd')
        # Dedicated state topic, NOT /joint_states -- see the comment on
        # joint_states_topic in my_robot_cell_macro.xacro. Publishing the measured
        # rail state onto the shared /joint_states put this node in a feedback loop
        # with the joint_state_broadcaster and froze the controller's rail state.
        self.declare_parameter('joint_state_topic', '/linear_axis/joint_states')

        # Rates. control_rate_hz drives the Modbus exchange; there is no point
        # going faster than the ComModbus I/O thread (modbus_cycle_time_ms).
        self.declare_parameter('control_rate_hz', 100.0)
        self.declare_parameter('joint_state_rate_hz', 100.0)
        self.declare_parameter('modbus_cycle_time_ms', 10)
        self.declare_parameter('modbus_timeout_ms', 1000)

        # Tracking law: v_cmd = ff_gain*|v_ff| + position_kp*|error|, clamped.
        #
        # Now that the drive actually honours the commanded speed, these gains decide
        # how SMOOTH the motion is. The rail steps between batched targets, so what
        # matters is the ratio of commanded speed to the trajectory's own speed: at
        # 1.0 the rail moves continuously, well above it the rail sprints to each
        # target and then waits (the "tik tik" stutter). Keep the feedforward at 1.0
        # and let a modest proportional term close the residual lag.
        self.declare_parameter('position_kp', 1.0)
        self.declare_parameter('feedforward_gain', 1.0)
        # The drive saturates at ~322 mm/s (measured); staying below keeps commands
        # inside the linear region where the word means what it says.
        self.declare_parameter('max_velocity_m_per_s', 0.30)
        # Must be BELOW typical trajectory speed (~35 mm/s) or the floor alone
        # re-creates the stutter.
        self.declare_parameter('min_velocity_m_per_s', 0.01)
        # Correction factor on the velocity word. 1.0 is correct for this drive
        # (word = mm/s = v[m/s] * velocity_scaling); only touch it if a different
        # drive reports a different velocity unit.
        self.declare_parameter('velocity_command_scale', 1.0)
        self.declare_parameter('velocity_estimate_alpha', 0.4)
        # Setpoint batching -- see _write_setpoint for the measurements behind this.
        # Smaller = finer path following but closer to the cycle-by-cycle stall;
        # larger = coarser path but a freer profile. 10 mm measured 14.2 mm error.
        self.declare_parameter('setpoint_batch_m', 0.01)
        self.declare_parameter('settle_write_sec', 0.15)

        # Safety. The dangerous failure is NOT "no comms" -- it is the rail falling
        # behind while the arm keeps executing the trajectory.
        self.declare_parameter('enable_safety_stop', True)
        # What to DO when the rail cannot follow:
        #   'warn'            -> log only. The right default while the drive's velocity
        #                        scaling is being commissioned: a latching stop turns a
        #                        tuning problem into a dead system.
        #   'stop_controller' -> deactivate scaled_joint_trajectory_controller. Note this
        #                        aborts the running MoveIt goal AND every later one until
        #                        the controller is re-activated (which this node now does
        #                        automatically once the rail tracks again).
        self.declare_parameter('safety_action', 'warn')
        self.declare_parameter('following_error_limit', 0.15)
        self.declare_parameter('following_error_grace_sec', 1.0)
        self.declare_parameter('comm_timeout_sec', 1.0)
        self.declare_parameter('command_timeout_sec', 0.5)
        self.declare_parameter('trigger_protective_stop', False)

        # Recovery.
        self.declare_parameter('auto_recover', True)
        self.declare_parameter('recover_interval_sec', 3.0)

        g = self.get_parameter
        self.ip_address = g('ip_address').value
        self.joint_name = g('joint_name').value
        self.command_topic = g('command_topic').value
        self.joint_state_topic = g('joint_state_topic').value
        self.control_rate_hz = float(g('control_rate_hz').value)
        self.joint_state_rate_hz = float(g('joint_state_rate_hz').value)
        self.modbus_cycle_time_ms = int(g('modbus_cycle_time_ms').value)
        self.modbus_timeout_ms = int(g('modbus_timeout_ms').value)
        self.position_kp = float(g('position_kp').value)
        self.feedforward_gain = float(g('feedforward_gain').value)
        self.max_velocity = float(g('max_velocity_m_per_s').value)
        self.min_velocity = float(g('min_velocity_m_per_s').value)
        self.velocity_command_scale = float(g('velocity_command_scale').value)
        self.velocity_alpha = float(g('velocity_estimate_alpha').value)
        self.setpoint_batch_m = float(g('setpoint_batch_m').value)
        self.settle_write_sec = float(g('settle_write_sec').value)
        self.enable_safety_stop = bool(g('enable_safety_stop').value)
        self.safety_action = str(g('safety_action').value)
        self.following_error_limit = float(g('following_error_limit').value)
        self.following_error_grace = float(g('following_error_grace_sec').value)
        self.comm_timeout_sec = float(g('comm_timeout_sec').value)
        self.command_timeout_sec = float(g('command_timeout_sec').value)
        self.use_protective_stop = bool(g('trigger_protective_stop').value)
        self.auto_recover = bool(g('auto_recover').value)
        self.recover_interval = float(g('recover_interval_sec').value)

        # --- Shared state (guarded by _lock) ----------------------------- #
        self._lock = threading.Lock()
        self._setpoint_m = None        # commanded rail position [m], None until first cmd
        self._setpoint_stamp = 0.0     # monotonic time of last accepted command
        self._setpoint_vel = 0.0       # estimated d(setpoint)/dt [m/s]
        self._prev_setpoint_m = None
        self._prev_setpoint_stamp = None

        self._measured_m = 0.0         # measured rail position [m]
        self._measured_v = 0.0         # measured rail velocity [m/s], differentiated
        self._prev_measured_m = None
        self._prev_measured_stamp = None
        self._have_measurement = False
        self._last_io_ok = 0.0         # monotonic time of last successful exchange

        # Diagnostics: what we asked the drive for, so a wrong velocity scaling is
        # visible in the log instead of having to be inferred from motion.
        self._cmd_velocity = 0.0       # commanded speed limit [m/s]
        self._cmd_vel_word = 0         # the integer actually written to mdi_velocity
        self._warned_zero_vel = False
        self._last_written_target = None  # last mdi_tarpos actually issued [m]

        self._state = ST_DISCONNECTED
        self._fault_code = 0
        self._following_error = 0.0
        self._error_exceeded_since = None
        self._safety_stop_latched = False
        self._controller_deactivated = False

        # --- Drive objects (touched ONLY by the drive thread) ------------- #
        self.com = None
        self.motion_handler = None
        self.telegram = None
        self.position_scaling = 1.0
        self.velocity_scaling = 1.0
        self._last_connect_attempt = 0.0
        self._last_recover_attempt = 0.0
        self._traversing_edge_done = False
        self._last_written_target = None

        # --- ROS interfaces ---------------------------------------------- #
        self.create_subscription(
            JointState, self.command_topic, self.command_callback, 10)
        self.joint_state_publisher = self.create_publisher(
            JointState, self.joint_state_topic, 10)
        self.create_timer(1.0 / self.joint_state_rate_hz, self.publish_joint_state)
        self.create_timer(1.0, self.report_status)

        self.stop_controller_client = self.create_client(
            SwitchController, '/controller_manager/switch_controller')
        self.protective_stop_client = self.create_client(
            Trigger, '/dashboard_client/protective_stop')
        self.popup_client = self.create_client(Popup, '/dashboard_client/popup')

        # --- Drive thread ------------------------------------------------ #
        self._running = True
        self._drive_thread = threading.Thread(target=self._drive_loop, daemon=True)
        self._drive_thread.start()

        self.get_logger().info(
            f"Linear axis adapter started: {self.ip_address}, joint '{self.joint_name}'")
        self.get_logger().info(
            f"  control {self.control_rate_hz:.0f} Hz | joint_states "
            f"{self.joint_state_rate_hz:.0f} Hz | modbus cycle "
            f"{self.modbus_cycle_time_ms} ms")
        self.get_logger().info(
            f"  tracking: v = {self.feedforward_gain:.2f}*|v_ff| + "
            f"{self.position_kp:.2f}*|err|, clamped to "
            f"[{self.min_velocity:.3f}, {self.max_velocity:.3f}] m/s")
        self.get_logger().info(
            f"  setpoint batching: re-issue target every "
            f"{self.setpoint_batch_m * 1000:.0f} mm "
            f"(settle write after {self.settle_write_sec:.2f} s)")

    # ====================================================================== #
    # ROS side -- these callbacks never touch Modbus, so they never block.
    # ====================================================================== #
    def command_callback(self, msg: JointState):
        """Position setpoint from TopicBasedSystem (the JTC's rail command).

        Passed through with NO filtering: this stream is the MoveIt trajectory
        already sampled at controller rate. Smoothing it would only add lag.
        The feedforward velocity is the time derivative of this stream, since the
        controller is configured with a position-only command interface and does
        not publish a velocity command.
        """
        try:
            idx = msg.name.index(self.joint_name)
        except ValueError:
            self.get_logger().warn(
                f"'{self.joint_name}' not in command message", throttle_duration_sec=5.0)
            return
        if idx >= len(msg.position):
            return

        target = float(msg.position[idx])
        now = time.monotonic()

        # Prefer an explicit velocity command if the controller ever provides one.
        vel = None
        if idx < len(msg.velocity):
            v = float(msg.velocity[idx])
            if v == v:  # not NaN
                vel = v

        with self._lock:
            if vel is None and self._prev_setpoint_stamp is not None:
                dt = now - self._prev_setpoint_stamp
                if dt > 1e-4:
                    raw = (target - self._prev_setpoint_m) / dt
                    # Filter the velocity ESTIMATE only. This costs no position
                    # lag -- the setpoint itself is still passed through raw.
                    a = self.velocity_alpha
                    vel = a * raw + (1.0 - a) * self._setpoint_vel
            self._prev_setpoint_m = target
            self._prev_setpoint_stamp = now
            self._setpoint_m = target
            self._setpoint_stamp = now
            if vel is not None:
                self._setpoint_vel = vel

    def publish_joint_state(self):
        """Publish the measured rail state; TopicBasedSystem consumes it as state."""
        with self._lock:
            if not self._have_measurement:
                return
            pos, vel = self._measured_m, self._measured_v

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [self.joint_name]
        msg.position = [pos]
        msg.velocity = [vel]
        self.joint_state_publisher.publish(msg)

    def report_status(self):
        """One consolidated status line per second instead of a per-cycle flood."""
        with self._lock:
            state = self._state
            err = self._following_error
            sp = self._setpoint_m
            pos = self._measured_m
            meas_v = self._measured_v
            cmd_v = self._cmd_velocity
            word = self._cmd_vel_word
            fault = self._fault_code

        if state == ST_RUNNING:
            if sp is not None and abs(err) > 0.002:
                # cmd vs measured speed is the diagnostic that matters: if the drive
                # honours mdi_velocity these two track each other.
                self.get_logger().info(
                    f"[{state}] target {sp:.4f} m | actual {pos:.4f} m | "
                    f"error {err * 1000.0:+.1f} mm | "
                    f"v_cmd {cmd_v:.3f} m/s (word {word}) | v_meas {meas_v:.3f} m/s")
        elif state == ST_FAULT:
            self.get_logger().error(
                f"[{state}] drive fault code {fault} | actual {pos:.4f} m",
                throttle_duration_sec=5.0)
        else:
            self.get_logger().warn(f"[{state}] actual {pos:.4f} m",
                                   throttle_duration_sec=5.0)

    # ====================================================================== #
    # Drive thread -- the ONLY place that talks to the drive.
    # ====================================================================== #
    def _drive_loop(self):
        period = 1.0 / self.control_rate_hz
        next_tick = time.monotonic()
        while self._running:
            next_tick += period
            try:
                self._drive_cycle()
            except Exception as exc:  # noqa: BLE001 - loop must never die
                self.get_logger().error(f"Drive cycle error: {exc}",
                                        throttle_duration_sec=2.0)
                self._teardown_drive()
            self._check_safety()

            sleep_for = next_tick - time.monotonic()
            if sleep_for > 0.0:
                time.sleep(sleep_for)
            else:
                next_tick = time.monotonic()  # overrun: resynchronise

    def _drive_cycle(self):
        if self.com is None or not self.com.io_active():
            self._connect()
            return

        # ONE non-blocking exchange per cycle. The ComModbus I/O thread performs
        # the actual Modbus transaction; we only read/write its buffers.
        data = self.com.recv_io(nonblocking=True)
        if data is not None:
            self.telegram.input_bytes(data)
            self._decode_state()

        self._step_state_machine()
        self.com.send_io(self.telegram.output_bytes(), nonblocking=True)

    def _decode_state(self):
        tg = self.telegram
        pos = tg.xist_a.value / self.position_scaling
        now = time.monotonic()

        # Differentiate the measured POSITION rather than decoding nist_b. Position is
        # unambiguous (it reads back in metres and matches reality); the velocity word's
        # scaling is the very thing under suspicion, so it must not be trusted here.
        vel = 0.0
        if self._prev_measured_stamp is not None:
            dt = now - self._prev_measured_stamp
            if dt > 1e-3:
                raw = (pos - self._prev_measured_m) / dt
                vel = 0.2 * raw + 0.8 * self._measured_v
                self._prev_measured_m = pos
                self._prev_measured_stamp = now
            else:
                vel = self._measured_v
        else:
            self._prev_measured_m = pos
            self._prev_measured_stamp = now

        with self._lock:
            self._measured_m = pos
            self._measured_v = vel
            self._have_measurement = True
            self._last_io_ok = now
            self._fault_code = int(tg.fault_code)
            if self._setpoint_m is not None:
                self._following_error = self._setpoint_m - pos

    def _step_state_machine(self):
        tg = self.telegram
        now = time.monotonic()

        if tg.zsw1.fault_present:
            if self._state != ST_FAULT:
                self.get_logger().error(
                    f"Drive reported a FAULT (code {int(tg.fault_code)}). "
                    "Motion commands are being rejected until it is cleared.")
                self._set_state(ST_FAULT)
                self._traversing_edge_done = False
                self._last_written_target = None
            if self.auto_recover and (now - self._last_recover_attempt) > self.recover_interval:
                self._last_recover_attempt = now
                self._attempt_recovery()
            return

        if not tg.zsw1.operation_enabled:
            # This is exactly the condition that made position_task() log
            # "Traversing task aborted" 100x/s while silently doing nothing.
            if self._state != ST_ENABLING:
                self.get_logger().warn(
                    "Drive is not operation_enabled (powerstage off or PLC control "
                    "not granted) -- no motion possible.")
                self._set_state(ST_ENABLING)
                self._traversing_edge_done = False
                self._last_written_target = None
            if self.auto_recover and (now - self._last_recover_attempt) > self.recover_interval:
                self._last_recover_attempt = now
                self._attempt_recovery()
            return

        if self._state != ST_RUNNING:
            self.get_logger().info("Drive is operational -- streaming setpoints.")
            self._set_state(ST_RUNNING)
            self._error_exceeded_since = None
            self._clear_safety_latch()

        self._write_setpoint()

    def _write_setpoint(self):
        """Stream the current setpoint into the telegram (continuous update).

        The target is BATCHED, not written every cycle. Measured on this drive with
        a setpoint ramping at 80 mm/s (linear_axis_diagnose.py, tests C and D):

            target rewritten every cycle  ->  147.6 mm max following error
            target rewritten every 10 mm  ->   14.2 mm max following error

        Both land exactly on the final position, so batching costs nothing. A target
        that moves every cycle restarts the drive's positioning profile before it can
        accelerate, which is what made the rail creep at ~5 mm/s while the arm ran
        the full trajectory.
        """
        with self._lock:
            setpoint = self._setpoint_m
            sp_stamp = self._setpoint_stamp
            sp_vel = self._setpoint_vel
            measured = self._measured_m

        now = time.monotonic()
        if setpoint is None:
            return
        if (now - sp_stamp) > self.command_timeout_sec:
            # No fresh command: hold the last setpoint but drop the feedforward,
            # so the drive settles instead of continuing at trajectory speed.
            sp_vel = 0.0

        error = setpoint - measured
        velocity = (self.feedforward_gain * abs(sp_vel)
                    + self.position_kp * abs(error))
        velocity = max(self.min_velocity, min(velocity, self.max_velocity))

        # mdi_velocity is in mm/s, MEASURED by sweeping the raw word against achieved
        # speed (linear_axis_diagnose.py sweep): words 20/50/200 produced 20.3/50.4/
        # 201.2 mm/s -- a 1.00 ratio -- and saturated at ~322 mm/s (the drive maximum)
        # from word 1000 up. velocity_scaling (1000) IS the m/s -> mm/s conversion;
        # the extra factor of 1000 this adapter used to apply put every command far
        # above saturation, which is why the drive appeared to ignore the velocity
        # and always ran flat out.
        word = int(round(
            velocity * self.velocity_scaling * self.velocity_command_scale))
        if word < 1:
            if not self._warned_zero_vel:
                self._warned_zero_vel = True
                self.get_logger().error(
                    f"Velocity word rounds to {word} for a {velocity:.4f} m/s command "
                    f"(velocity_scaling={self.velocity_scaling}, "
                    f"velocity_command_scale={self.velocity_command_scale}). "
                    "Raise min_velocity_m_per_s.")
            word = 1

        # Decide whether this cycle re-issues the target. Between re-issues the
        # telegram is still sent every cycle (the drive loop does that), just with
        # an unchanged mdi_tarpos, so the drive gets to finish its profile.
        if self._last_written_target is None:
            reissue = True
        elif abs(setpoint - self._last_written_target) >= self.setpoint_batch_m:
            reissue = True
        else:
            # The stream has settled (trajectory ended) but the last batch left a
            # sub-threshold remainder: write the exact final target once so the axis
            # lands on it instead of stopping up to setpoint_batch_m short.
            reissue = (setpoint != self._last_written_target
                       and (now - sp_stamp) > self.settle_write_sec)

        tg = self.telegram
        if reissue:
            tg.mdi_tarpos.value = int(setpoint * self.position_scaling)
            tg.mdi_velocity.value = word
            self._last_written_target = setpoint
        with self._lock:
            self._cmd_velocity = velocity
            self._cmd_vel_word = word
        tg.pos_stw1.activate_setup = False
        tg.pos_stw1.activate_mdi = True
        tg.pos_stw1.absolute_position = True
        # THE fix for the abort/restart cycle: with continuous_update the drive
        # accepts a new setpoint every cycle without needing a new traversing task.
        tg.pos_stw1.continuous_update = True

        if not self._traversing_edge_done:
            # One rising edge is still required to enter MDI positioning; produce
            # it across two cycles, then hold the bit high forever.
            tg.stw1.activate_traversing_task = False
            self._traversing_edge_done = True
        else:
            tg.stw1.activate_traversing_task = True

    # --- Connection / recovery ------------------------------------------ #
    def _connect(self):
        now = time.monotonic()
        if (now - self._last_connect_attempt) < self.recover_interval:
            return
        self._last_connect_attempt = now
        self._set_state(ST_DISCONNECTED)
        self._teardown_drive()

        try:
            self.get_logger().info(f"Connecting to drive at {self.ip_address} ...")
            com = ComModbus(ip_address=self.ip_address,
                            cycle_time=self.modbus_cycle_time_ms,
                            timeout_ms=self.modbus_timeout_ms)
            self.position_scaling = 10 ** (-com.read_pnu(11724, 0))
            self.velocity_scaling = 10 ** (-com.read_pnu(11725, 0))

            handler = MotionHandler(com, config_mode="write")
            handler.base_velocity = com.read_pnu(12345, 0)
            handler.over_v = 100.0
            handler.over_acc = 100.0
            handler.over_dec = 100.0

            self.com = com
            self.motion_handler = handler
            self.telegram = handler.telegram
            self._traversing_edge_done = False
            self._last_written_target = None

            self.get_logger().info(
                f"Connected. position_scaling={self.position_scaling}, "
                f"velocity_scaling={self.velocity_scaling}, "
                f"base_velocity={handler.base_velocity}")

            handler.acknowledge_faults()
            handler.enable_powerstage()
            self._set_state(ST_ENABLING)

            # Seed the setpoint with the current position so the drive is never
            # commanded to jump on startup.
            pos = handler.current_position() / self.position_scaling
            with self._lock:
                self._measured_m = pos
                self._have_measurement = True
                if self._setpoint_m is None:
                    self._setpoint_m = pos
                    self._setpoint_stamp = time.monotonic()
            self.get_logger().info(f"Start position: {pos:.4f} m")

        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Connection failed: {exc}",
                                    throttle_duration_sec=5.0)
            self._teardown_drive()

    def _attempt_recovery(self):
        """Bounded, rate-limited fault recovery. Blocking, but only in this thread."""
        try:
            self.get_logger().warn("Attempting drive recovery "
                                   "(acknowledge faults + enable powerstage) ...")
            if not self.motion_handler.acknowledge_faults(timeout=1.0):
                self.get_logger().error(
                    f"Fault acknowledge failed (code {int(self.telegram.fault_code)}). "
                    "The fault may need clearing at the drive.",
                    throttle_duration_sec=10.0)
                return
            if not self.motion_handler.enable_powerstage(timeout=2.0):
                self.get_logger().error("Powerstage enable failed.",
                                        throttle_duration_sec=10.0)
                return
            self.get_logger().info("Drive recovery succeeded.")
            self._traversing_edge_done = False
            self._last_written_target = None
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Recovery error: {exc}", throttle_duration_sec=5.0)
            self._teardown_drive()

    def _teardown_drive(self):
        if self.com is not None:
            try:
                self.com.shutdown()
            except Exception:  # noqa: BLE001
                pass
        self.com = None
        self.motion_handler = None
        self.telegram = None
        self._traversing_edge_done = False
        self._last_written_target = None
        self._set_state(ST_DISCONNECTED)

    def _set_state(self, state):
        with self._lock:
            self._state = state

    # ====================================================================== #
    # Safety
    # ====================================================================== #
    def _check_safety(self):
        if not self.enable_safety_stop:
            return

        now = time.monotonic()
        with self._lock:
            state = self._state
            error = abs(self._following_error)
            last_io = self._last_io_ok
            sp_stamp = self._setpoint_stamp
            have = self._have_measurement

        commanded = (now - sp_stamp) < self.command_timeout_sec
        comm_dead = have and (now - last_io) > self.comm_timeout_sec

        # The rail lagging while the arm executes a trajectory is the real hazard.
        lagging = commanded and (state != ST_RUNNING or error > self.following_error_limit)

        if lagging or comm_dead:
            if self._error_exceeded_since is None:
                self._error_exceeded_since = now
            elif (now - self._error_exceeded_since) > self.following_error_grace:
                if comm_dead:
                    reason = f"no drive I/O for {now - last_io:.2f}s"
                elif state != ST_RUNNING:
                    reason = f"drive in state {state} while a trajectory is running"
                else:
                    reason = f"following error {error * 1000.0:.0f} mm exceeds limit"
                self._trigger_safety_stop(reason)
        else:
            self._error_exceeded_since = None
            self._clear_safety_latch()

    def _clear_safety_latch(self):
        if not self._safety_stop_latched:
            return
        self.get_logger().info("Linear axis is tracking again; safety latch cleared.")
        self._safety_stop_latched = False
        if self._controller_deactivated:
            # Deactivating the controller is not a one-way door: without this the
            # first violation kills every subsequent MoveIt goal ("Can't accept new
            # action goals. Controller is not running.") until relaunch.
            self._switch_controller(activate=True)

    def _trigger_safety_stop(self, reason):
        if self._safety_stop_latched:
            return
        self._safety_stop_latched = True

        if self.safety_action == 'warn':
            self.get_logger().warn(
                f"LINEAR AXIS NOT FOLLOWING: {reason} "
                "(safety_action='warn' -- motion is NOT being stopped)")
            return

        self.get_logger().error("=" * 60)
        self.get_logger().error("SAFETY STOP -- LINEAR AXIS CANNOT FOLLOW THE TRAJECTORY")
        self.get_logger().error(f"Reason: {reason}")
        self.get_logger().error("=" * 60)

        if self.use_protective_stop:
            self._call_protective_stop()
        self._switch_controller(activate=False)
        self._show_pendant_popup(reason)

    def _switch_controller(self, activate):
        name = 'scaled_joint_trajectory_controller'
        if not self.stop_controller_client.service_is_ready():
            self.get_logger().error("Controller manager service not available.")
            return
        req = SwitchController.Request()
        req.start_controllers = [name] if activate else []
        req.stop_controllers = [] if activate else [name]
        req.strictness = SwitchController.Request.BEST_EFFORT
        req.start_asap = False
        req.timeout = rclpy.duration.Duration(seconds=0.0).to_msg()
        future = self.stop_controller_client.call_async(req)
        what = f"{'activate' if activate else 'deactivate'} {name}"
        future.add_done_callback(lambda f: self._log_service_result(f, what, 'ok'))
        self._controller_deactivated = not activate
        self.get_logger().warn(f">>> {what.upper()} <<<")

    def _call_protective_stop(self):
        if not self.protective_stop_client.service_is_ready():
            self.get_logger().error("Protective stop service not available.")
            return
        future = self.protective_stop_client.call_async(Trigger.Request())
        future.add_done_callback(
            lambda f: self._log_service_result(f, "protective stop", 'success'))
        self.get_logger().error(">>> PROTECTIVE STOP TRIGGERED <<<")

    def _show_pendant_popup(self, reason):
        if not self.popup_client.service_is_ready():
            return
        req = Popup.Request()
        req.message = ("SAFETY STOP\n\n"
                       "The linear axis could not follow the trajectory.\n\n"
                       f"{reason}\n\n"
                       "Check the linear axis before resuming.")
        future = self.popup_client.call_async(req)
        future.add_done_callback(
            lambda f: self._log_service_result(f, "pendant popup", 'success'))

    def _log_service_result(self, future, what, ok_field):
        """Service results arrive on the ROS executor, not the drive thread."""
        try:
            response = future.result()
            if getattr(response, ok_field):
                self.get_logger().error(f"{what}: OK")
            else:
                self.get_logger().error(f"{what}: FAILED")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"{what}: error {exc}")

    # ====================================================================== #
    def cleanup(self):
        self._running = False
        if self._drive_thread.is_alive():
            self._drive_thread.join(timeout=2.0)
        if self.motion_handler is not None:
            self.get_logger().info("Disabling powerstage ...")
            try:
                self.motion_handler.disable_powerstage()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f"Powerstage shutdown failed: {exc}")
        self._teardown_drive()


def main(args=None):
    rclpy.init(args=args)
    node = LinearAxisControllerAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
