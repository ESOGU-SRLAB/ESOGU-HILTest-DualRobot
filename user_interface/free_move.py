#!/usr/bin/env python3
"""
Free Move: drag-to-jog TCP control for the UR10e and Kawasaki arms.

Concurrency model mirrors UseCaseBroadcaster / JointStateCollector / AnomalyCollector
in app.py: each arm gets its OWN rclpy Node + OWN SingleThreadedExecutor, spun in a
loop on ONE dedicated daemon thread (_ros_spin). Nothing else may ever call
rclpy.spin_once() on that node.

That restriction matters here more than in the other collectors, because
pymoveit2_real's blocking convenience wrappers (compute_ik(), compute_fk(), plan(),
wait_until_executed()) all internally do `while not done: rclpy.spin_once(self._node,
...)`. Calling any of them from outside _ros_spin's thread would spin the node from a
second place at once and corrupt the executor's wait set -- see the identical warning
in multirobot_viewpoint_planner/{ur,kawasaki}_inspection_node.py, which avoid the same
wrappers for the same reason. So every public method below only ever uses the
`*_async()` / `query_state()` / `motion_suceeded` surface, and waits by polling with
time.sleep() from the CALLING thread while the dedicated thread's own spin_once loop
is what actually advances the future/state. That calling thread is always a
Flask-SocketIO handler thread, never _ros_spin's thread.

Joint state: /joint_states on the real cell comes from THREE separate publishers that
each cover only part of the robot (measured 21 Aug 2026 -- see project memory
anomali-joint-states-cok-yayinci), and inspection_base.py works around this the same
way we do here: merge every message's names into a persistent dict keyed by name
(never trust a single message, or MoveIt2Real's own internal joint_state cache, to
carry the WHOLE arm). We also always pass that merged state explicitly into
plan_async()/compute_fk_async() rather than letting pymoveit2 fall back to its
internal cache -- besides being unreliable under a fragmented topic, plan_async()'s
fallback path (when no start state is given and its cache is empty) calls the bare
rclpy.spin_once(self._node, ...) in a retry loop, which is the exact "second spinner"
hazard the module docstring above is about.
"""
import re
import threading
import time

# Must match multirobot_viewpoint_planner/kawasaki_inspection_node.py::KAWASAKI_JOINT_NAMES.
# Kept as a local literal rather than imported: that module pulls in scipy/control_msgs
# at import time for machinery Free Move does not need.
KAWASAKI_JOINT_NAMES = [
    "world_to_agv", "joint1", "joint2", "joint3", "joint4", "joint5", "joint6",
]

# Must match pymoveit2_real/pymoveit2_real/robots/ur.py.
UR_JOINT_NAMES = [
    "ur10e_base_to_robot_mount",
    "ur10e_shoulder_pan_joint",
    "ur10e_shoulder_lift_joint",
    "ur10e_elbow_joint",
    "ur10e_wrist_1_joint",
    "ur10e_wrist_2_joint",
    "ur10e_wrist_3_joint",
]
UR_END_EFFECTOR = "ur10e_tool0"
UR_GROUP = "real_ur10e"
# Same filter multirobot_viewpoint_planner/ur_inspection_node.py uses for collision padding.
UR_LINK_RE = re.compile(r"^ur10e_")

KAWASAKI_END_EFFECTOR = "link6"
KAWASAKI_GROUP = "real_kawasaki"
# Same filter multirobot_viewpoint_planner/kawasaki_inspection_node.py uses.
KAWASAKI_LINK_RE = re.compile(r"^link[1-6]$")

WORLD_FRAME = "world"

IK_TIMEOUT = 2.0
PLAN_TIMEOUT = 10.0
# Was 60.0, then 120.0 -- too short for the Kawasaki at its default 0.02 velocity
# scale (5x slower than the UR's 0.1 default): a real Free Move drag routinely took
# longer than that, so execute()/execute_joint() cancelled a trajectory that was
# still genuinely in progress ("controller TIMED OUT", reported 2026-09-11). Kept
# equal to whole_cell_kawasaki_controllers.yaml's own goal_time tolerance -- raise
# both together, never just one (if EXECUTE_TIMEOUT is smaller, this code cancels a
# fine trajectory early; if the controller's goal_time is smaller, the CONTROLLER
# aborts before this code even gets a chance to time out).
EXECUTE_TIMEOUT = 240.0
SCENE_TIMEOUT = 5.0
POLL_INTERVAL = 0.02

# Applied on connect, before the operator touches the settings panel. Match the
# "production" numbers measured in project memory ompl-planner-choice-chassis
# (10 attempts / 5s budget beats the pymoveit2 defaults of 5 attempts / 0.5s, which
# is too little time for a 7-DOF group with a non-trivial planner). planner_id empty
# means "let MoveIt pick" -- it falls back to RRTConnect, the proven choice there.
# max_velocity/max_acceleration are NOT here -- they default differently per arm
# (see UR_DEFAULT_* / KAWASAKI_DEFAULT_* below) and get merged in per-instance.
DEFAULT_PARAMS = {
    "num_planning_attempts": 10,
    "allowed_planning_time": 5.0,
    "planner_id": "",
}
# Every key set_params() is allowed to touch -- DEFAULT_PARAMS plus the two that are
# seeded per-arm instead of from a shared default (see UR_DEFAULT_*/KAWASAKI_DEFAULT_*).
PARAM_KEYS = (*DEFAULT_PARAMS.keys(), "max_velocity", "max_acceleration")
DEFAULT_PADDING_M = 0.04

# Kawasaki defaults much slower than UR on purpose -- operator request, not a
# measured limit: the AGV rail makes a Kawasaki jog cover more ground per radian
# of joint motion than the UR does, so the same scaling factor feels far faster
# on Kawasaki in practice.
UR_DEFAULT_VELOCITY = 0.1
UR_DEFAULT_ACCELERATION = 0.1
KAWASAKI_DEFAULT_VELOCITY = 0.02
KAWASAKI_DEFAULT_ACCELERATION = 0.02

# Set by app.py at import time. _ros_spin's except block always print()s (visible
# only if someone happens to be watching this process's own stdout, which for a
# long-running dashboard nobody usually is), and additionally calls this if set so
# the error reaches the operator through the dashboard's own log/terminal.
on_error = None  # callable(arm_name: str, message: str) -> None


class FreeMoveArm:
    """One arm's MoveIt2 handle, owned by a single dedicated ROS thread."""

    def __init__(self, name, joint_names, end_effector_name, group_name, link_re,
                 base_link_name=WORLD_FRAME, default_velocity=0.2, default_acceleration=0.2):
        self.name = name
        self._joint_names = list(joint_names)
        self._end_effector_name = end_effector_name
        self._group_name = group_name
        self._base_link_name = base_link_name
        self._link_re = link_re

        self._node = None
        self._moveit2 = None
        self._scene_clients = None  # (get_planning_scene, apply_planning_scene)
        self._ik_cli = None
        self._validity_cli = None
        self._running = False
        self._thread = None
        self._ready = threading.Event()
        # Guards the _node/_moveit2/_scene_clients handles (set once in _ros_spin).
        self._handle_lock = threading.Lock()
        # Serializes execute() calls against each other on this arm -- move_to_pose()
        # silently no-ops if a previous goal's __is_motion_requested/__is_executing
        # flags are still set, so overlapping Execute clicks must not race.
        self._exec_lock = threading.Lock()

        # Joint state merged by name across every /joint_states message (see module
        # docstring). Read/written from different threads, so it gets its own lock
        # rather than reusing _handle_lock (that one is held only briefly).
        self._joint_lock = threading.Lock()
        self._joint_pos = {}

        self._params = dict(DEFAULT_PARAMS)
        self._params["max_velocity"] = default_velocity
        self._params["max_acceleration"] = default_acceleration
        self._padding_m = DEFAULT_PADDING_M
        self._last_error = None
        # The exact trajectory_msgs/JointTrajectory the last successful plan()/
        # plan_joint() call computed (and the ghost preview animates), guarded by
        # _exec_lock. execute()/execute_joint() run THIS directly instead of asking
        # MoveIt to plan again -- see the long comment on execute() for why that
        # matters.
        self._last_trajectory = None

    # --- lifecycle --------------------------------------------------- #
    def start(self):
        if self._thread is not None:
            return
        self._running = True
        self._ready.clear()
        self._last_error = None
        with self._joint_lock:
            self._joint_pos = {}
        self._thread = threading.Thread(
            target=self._ros_spin, daemon=True,
            name=f"free_move_{self.name}")
        self._thread.start()

    def stop(self):
        self._running = False
        thread, self._thread = self._thread, None
        if thread is not None:
            thread.join(timeout=5.0)
        self._ready.clear()

    def wait_ready(self, timeout=15.0):
        return self._ready.wait(timeout)

    def wait_joint_state(self, timeout=20.0):
        """Block (polling, never spin_once) until every joint this arm needs has
        been seen at least once on /joint_states."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self._current_joint_state_msg() is not None:
                return True
            time.sleep(0.1)
        return False

    @property
    def ready(self):
        return self._ready.is_set()

    @property
    def last_error(self):
        return self._last_error

    def _ros_spin(self):
        try:
            import rclpy
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.node import Node as RosNode
            from rclpy.callback_groups import ReentrantCallbackGroup
            from sensor_msgs.msg import JointState
            from moveit_msgs.srv import (
                GetPlanningScene, ApplyPlanningScene, GetPositionIK, GetStateValidity)
            from pymoveit2_real import MoveIt2 as MoveIt2Real

            if not rclpy.ok():
                rclpy.init()
            node = RosNode(f"dashboard_free_move_{self.name}")
            cb_group = ReentrantCallbackGroup()
            moveit2 = MoveIt2Real(
                node=node,
                joint_names=self._joint_names,
                base_link_name=self._base_link_name,
                end_effector_name=self._end_effector_name,
                group_name=self._group_name,
                callback_group=cb_group,
                # CRITICAL: without this, move_to_pose()/move_to_configuration() take
                # their OTHER branch -- self.execute(self.plan(...)) -- and that
                # plan() is the BLOCKING wrapper (bare rclpy.spin_once(self._node,
                # ...) in a loop), the exact "second spinner" this module's docstring
                # warns about, except here it was happening on every single Execute
                # click via a call this file never made directly. use_move_group_action
                # routes both through the MoveGroup action's send_goal_async() instead,
                # which only ever registers a done-callback -- no spinning of its own,
                # safe under the same dedicated-executor-only rule as everything else
                # here. This was very likely the real cause of "the system gets tired
                # and starts timing out after a while": each Execute was corrupting
                # this node's own executor wait set out from under it.
                use_move_group_action=True,
            )
            self._apply_params_to(moveit2, self._params)

            def _joint_states_cb(msg):
                with self._joint_lock:
                    for n, p in zip(msg.name, msg.position):
                        self._joint_pos[n] = p

            node.create_subscription(
                JointState, "/joint_states", _joint_states_cb, 10,
                callback_group=cb_group)

            get_scene_cli = node.create_client(
                GetPlanningScene, "get_planning_scene", callback_group=cb_group)
            apply_scene_cli = node.create_client(
                ApplyPlanningScene, "apply_planning_scene", callback_group=cb_group)
            # Our OWN /compute_ik and /check_state_validity clients rather than
            # pymoveit2's: pymoveit2's compute_ik_async() never sets ik_link_name, so it
            # silently solves for the planning GROUP's default SRDF tip link -- ur10e_tool0
            # for real_ur10e (matches our end effector, so it happens to be fine there),
            # but link7 for real_kawasaki (we use link6) -- see the identical note in
            # inspection_base.py, which hits the same limitation and works around it the
            # same way.
            ik_cli = node.create_client(
                GetPositionIK, "compute_ik", callback_group=cb_group)
            validity_cli = node.create_client(
                GetStateValidity, "check_state_validity", callback_group=cb_group)

            with self._handle_lock:
                self._node = node
                self._moveit2 = moveit2
                self._scene_clients = (get_scene_cli, apply_scene_cli)
                self._ik_cli = ik_cli
                self._validity_cli = validity_cli

            # Dedicated executor: sharing the global one makes nodes vanish from the
            # graph silently (see the identical note on the other collectors in app.py).
            executor = SingleThreadedExecutor()
            executor.add_node(node)
            self._ready.set()
            try:
                while self._running and rclpy.ok():
                    executor.spin_once(timeout_sec=0.1)
            finally:
                executor.remove_node(node)
                node.destroy_node()
                with self._handle_lock:
                    self._node = None
                    self._moveit2 = None
                    self._scene_clients = None
                    self._ik_cli = None
                    self._validity_cli = None
        except Exception as e:
            import traceback
            detail = traceback.format_exc()
            print(f"[FreeMoveArm:{self.name}] ROS2 error: {detail}")
            self._last_error = str(e)
            if on_error:
                try:
                    on_error(self.name, str(e))
                except Exception:
                    pass
        finally:
            self._ready.clear()

    def _get_moveit2(self):
        with self._handle_lock:
            return self._moveit2

    def _get_scene_clients(self):
        with self._handle_lock:
            return self._scene_clients

    def _get_ik_cli(self):
        with self._handle_lock:
            return self._ik_cli

    def _get_validity_cli(self):
        with self._handle_lock:
            return self._validity_cli

    @staticmethod
    def _wait_future(future, timeout):
        if future is None:
            return None
        deadline = time.monotonic() + timeout
        while not future.done():
            if time.monotonic() > deadline:
                return None
            time.sleep(POLL_INTERVAL)
        return future

    def _current_joint_state_msg(self):
        """A sensor_msgs/JointState covering exactly self._joint_names, built from the
        merged-by-name cache, or None while any of them is still unseen."""
        with self._joint_lock:
            pos = dict(self._joint_pos)
        if not all(n in pos for n in self._joint_names):
            return None
        from sensor_msgs.msg import JointState
        msg = JointState()
        msg.name = list(self._joint_names)
        msg.position = [pos[n] for n in self._joint_names]
        return msg

    # --- thread-safe operations (call from any thread EXCEPT _ros_spin's) --- #
    # IMPORTANT: never call moveit2's blocking wrappers (compute_ik(), plan(),
    # wait_until_executed()) here -- they call the bare rclpy.spin_once(node, ...)
    # internally, which steals the node from the dedicated executor above and kills
    # its spin loop. Always use the *_async() + poll-with-sleep pattern instead, so
    # the ONLY thing that ever spins `node` is the loop in _ros_spin.

    def check_ik(self, position, quat_xyzw):
        """{"ok": bool, "joint_positions": {name: rad}|None, "collision_free": bool|None,
        "error": str|None}. avoid_collisions is OFF here on purpose: this is the live
        drag preview, and the point is to always show the operator where the arm (and
        its camera) would end up -- same as RViz's goal-state marker, which shows the
        pose and highlights collisions rather than just going blank. A colliding pose
        still gets found by IK and shown (tinted, via the separate validity check
        below); Plan/Execute are unaffected -- OMPL's own state validity checking during
        planning is always collision-aware regardless of this flag."""
        ik_cli = self._get_ik_cli()
        if ik_cli is None:
            return {"ok": False, "joint_positions": None, "collision_free": None, "error": "not_ready"}
        if not ik_cli.wait_for_service(timeout_sec=SCENE_TIMEOUT):
            return {"ok": False, "joint_positions": None, "collision_free": None,
                    "error": "compute_ik service unavailable"}
        try:
            from moveit_msgs.srv import GetPositionIK
            req = GetPositionIK.Request()
            req.ik_request.group_name = self._group_name
            req.ik_request.ik_link_name = self._end_effector_name
            req.ik_request.avoid_collisions = False
            req.ik_request.pose_stamped.header.frame_id = self._base_link_name
            req.ik_request.pose_stamped.pose.position.x = float(position[0])
            req.ik_request.pose_stamped.pose.position.y = float(position[1])
            req.ik_request.pose_stamped.pose.position.z = float(position[2])
            req.ik_request.pose_stamped.pose.orientation.x = float(quat_xyzw[0])
            req.ik_request.pose_stamped.pose.orientation.y = float(quat_xyzw[1])
            req.ik_request.pose_stamped.pose.orientation.z = float(quat_xyzw[2])
            req.ik_request.pose_stamped.pose.orientation.w = float(quat_xyzw[3])
            req.ik_request.timeout.sec = int(IK_TIMEOUT)
            start_state = self._current_joint_state_msg()
            if start_state is not None:
                req.ik_request.robot_state.joint_state = start_state

            future = ik_cli.call_async(req)
            if self._wait_future(future, IK_TIMEOUT) is None or future.result() is None:
                return {"ok": False, "joint_positions": None, "collision_free": None, "error": "timeout"}
            res = future.result()
            if res.error_code.val != res.error_code.SUCCESS:
                return {
                    "ok": False, "joint_positions": None, "collision_free": None,
                    "error": f"no_ik_solution ({res.error_code.val})",
                }
            joints = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
            collision_free = self._check_state_validity(joints)
            return {"ok": True, "joint_positions": joints, "collision_free": collision_free, "error": None}
        except Exception as e:
            return {"ok": False, "joint_positions": None, "collision_free": None, "error": str(e)}

    def _check_state_validity(self, joint_positions):
        """True/False, or None if the check itself could not be completed (treated as
        "unknown" by the caller, not as a failure)."""
        cli = self._get_validity_cli()
        if cli is None or not cli.wait_for_service(timeout_sec=SCENE_TIMEOUT):
            return None
        try:
            from moveit_msgs.srv import GetStateValidity
            from sensor_msgs.msg import JointState
            req = GetStateValidity.Request()
            req.group_name = self._group_name
            js = JointState()
            js.name = list(joint_positions.keys())
            js.position = list(joint_positions.values())
            req.robot_state.joint_state = js
            future = cli.call_async(req)
            if self._wait_future(future, IK_TIMEOUT) is None or future.result() is None:
                return None
            return bool(future.result().valid)
        except Exception:
            return None

    def plan(self, position, quat_xyzw):
        """{"ok": bool, "trajectory": {"joint_names": [...], "points": [...]}, "error": ...}"""
        moveit2 = self._get_moveit2()
        if moveit2 is None:
            return {"ok": False, "trajectory": None, "error": "not_ready"}
        start_state = self._current_joint_state_msg()
        if start_state is None:
            return {"ok": False, "trajectory": None, "error": "no_joint_state"}
        try:
            future = moveit2.plan_async(
                position=position, quat_xyzw=quat_xyzw, cartesian=False,
                start_joint_state=start_state)
            if self._wait_future(future, PLAN_TIMEOUT) is None:
                return {"ok": False, "trajectory": None, "error": "timeout"}
            traj = moveit2.get_trajectory(future, cartesian=False)
            if traj is None:
                return {"ok": False, "trajectory": None, "error": "no_plan_found"}
            self._last_trajectory = traj
            points = [
                {
                    "positions": list(pt.positions),
                    "time_from_start": pt.time_from_start.sec
                    + pt.time_from_start.nanosec * 1e-9,
                }
                for pt in traj.points
            ]
            return {
                "ok": True,
                "trajectory": {"joint_names": list(traj.joint_names), "points": points},
                "error": None,
            }
        except Exception as e:
            return {"ok": False, "trajectory": None, "error": str(e)}

    def execute(self, position, quat_xyzw):
        """Run the EXACT trajectory the last plan() call computed. Blocks the calling
        thread until the motion finishes or times out -- callers must run this on
        their own thread.

        This used to call moveit2.move_to_pose(...), which -- because FreeMoveArm
        constructs MoveIt2Real with use_move_group_action=True -- sends a fresh
        MoveGroup action goal and lets move_group plan AGAIN from scratch, completely
        independently of the plan() call that had just been used to render the ghost
        preview. OMPL is non-deterministic (and IK for a given pose is not unique --
        the same Cartesian target has multiple valid elbow/wrist configurations), so
        that second, independent plan routinely picked a different joint-space path,
        sometimes even a different FINAL joint configuration for the same TCP goal:
        exactly the "ghost and the real robot end up in different poses" and "extra
        jerky" symptoms reported 2026-09-09. Running the stored trajectory directly
        (moveit2.execute(), which talks straight to the controller, no re-planning)
        guarantees the real robot follows the identical path the ghost just animated,
        every time. position/quat_xyzw are accepted only for logging/API symmetry with
        plan() -- the frontend already refuses to enable Execute unless the current
        target still matches the pose that was last successfully planned."""
        moveit2 = self._get_moveit2()
        if moveit2 is None:
            return {"ok": False, "error": "not_ready"}
        if self._last_trajectory is None:
            return {"ok": False, "error": "no_plan"}
        if not self._exec_lock.acquire(blocking=False):
            return {"ok": False, "error": "busy"}
        try:
            from pymoveit2_real import MoveIt2State
            moveit2.execute(self._last_trajectory)
            deadline = time.monotonic() + EXECUTE_TIMEOUT
            # NEVER wait_until_executed() here -- see module docstring. Poll the plain
            # state flags instead; the dedicated thread's executor advances them.
            while moveit2.query_state() != MoveIt2State.IDLE:
                if time.monotonic() > deadline:
                    moveit2.cancel_execution()
                    return {"ok": False, "error": "timeout"}
                time.sleep(POLL_INTERVAL)
            if moveit2.motion_suceeded:
                return {"ok": True, "error": None}
            return {"ok": False, "error": "motion_failed"}
        except Exception as e:
            return {"ok": False, "error": str(e)}
        finally:
            self._exec_lock.release()

    def cancel(self):
        moveit2 = self._get_moveit2()
        if moveit2 is None:
            return
        try:
            moveit2.cancel_execution()
        except Exception as e:
            print(f"[FreeMoveArm:{self.name}] cancel failed: {e}")

    def plan_joint(self, joint_positions):
        """Joint-space counterpart of plan() -- goal given as {joint_name: radians}
        instead of a Cartesian pose, no IK involved at all."""
        moveit2 = self._get_moveit2()
        if moveit2 is None:
            return {"ok": False, "trajectory": None, "error": "not_ready"}
        start_state = self._current_joint_state_msg()
        if start_state is None:
            return {"ok": False, "trajectory": None, "error": "no_joint_state"}
        try:
            names = list(joint_positions.keys())
            positions = [float(joint_positions[n]) for n in names]
            future = moveit2.plan_async(
                joint_positions=positions, joint_names=names, start_joint_state=start_state)
            if self._wait_future(future, PLAN_TIMEOUT) is None:
                return {"ok": False, "trajectory": None, "error": "timeout"}
            traj = moveit2.get_trajectory(future, cartesian=False)
            if traj is None:
                return {"ok": False, "trajectory": None, "error": "no_plan_found"}
            self._last_trajectory = traj
            points = [
                {
                    "positions": list(pt.positions),
                    "time_from_start": pt.time_from_start.sec
                    + pt.time_from_start.nanosec * 1e-9,
                }
                for pt in traj.points
            ]
            return {
                "ok": True,
                "trajectory": {"joint_names": list(traj.joint_names), "points": points},
                "error": None,
            }
        except Exception as e:
            return {"ok": False, "trajectory": None, "error": str(e)}

    def execute_joint(self, joint_positions):
        """Joint-space counterpart of execute() -- runs the exact trajectory the last
        plan_joint() call computed, for the same reason execute() no longer calls
        move_to_pose(): move_to_configuration() would plan AGAIN from scratch and could
        land the real robot on a different path/configuration than the ghost just
        showed. Same safety notes apply (needs use_move_group_action=True on the
        MoveIt2Real construction, never wait_until_executed()); blocks the calling
        thread, run on its own. joint_positions is accepted only for API symmetry with
        plan_joint() -- the frontend already gates the Execute button on the current
        slider goal still matching what was last successfully planned."""
        moveit2 = self._get_moveit2()
        if moveit2 is None:
            return {"ok": False, "error": "not_ready"}
        if self._last_trajectory is None:
            return {"ok": False, "error": "no_plan"}
        if not self._exec_lock.acquire(blocking=False):
            return {"ok": False, "error": "busy"}
        try:
            from pymoveit2_real import MoveIt2State
            moveit2.execute(self._last_trajectory)
            deadline = time.monotonic() + EXECUTE_TIMEOUT
            while moveit2.query_state() != MoveIt2State.IDLE:
                if time.monotonic() > deadline:
                    moveit2.cancel_execution()
                    return {"ok": False, "error": "timeout"}
                time.sleep(POLL_INTERVAL)
            if moveit2.motion_suceeded:
                return {"ok": True, "error": None}
            return {"ok": False, "error": "motion_failed"}
        except Exception as e:
            return {"ok": False, "error": str(e)}
        finally:
            self._exec_lock.release()

    def current_tcp_pose(self):
        """{"position": [x,y,z], "quat_xyzw": [x,y,z,w]} for the end effector, or None."""
        moveit2 = self._get_moveit2()
        if moveit2 is None:
            return None
        joint_state = self._current_joint_state_msg()
        if joint_state is None:
            return None
        try:
            future = moveit2.compute_fk_async(joint_state=joint_state)
            if self._wait_future(future, IK_TIMEOUT) is None:
                return None
            pose = moveit2.get_compute_fk_result(future)
            if pose is None:
                return None
            p, q = pose.pose.position, pose.pose.orientation
            return {"position": [p.x, p.y, p.z], "quat_xyzw": [q.x, q.y, q.z, q.w]}
        except Exception as e:
            print(f"[FreeMoveArm:{self.name}] FK failed: {e}")
            return None

    # --- planning parameters ------------------------------------------ #

    @staticmethod
    def _apply_params_to(moveit2, params):
        moveit2.max_velocity = float(params["max_velocity"])
        moveit2.max_acceleration = float(params["max_acceleration"])
        moveit2.num_planning_attempts = int(params["num_planning_attempts"])
        moveit2.allowed_planning_time = float(params["allowed_planning_time"])
        moveit2.planner_id = str(params["planner_id"])

    def get_params(self):
        return {**self._params, "padding_m": self._padding_m}

    def set_params(self, updates):
        """Merge `updates` (a subset of PARAM_KEYS) and apply them to the live
        MoveIt2 object. Affects both plan() and execute(): pymoveit2's plan service
        reuses the same request object the move-group action goal does."""
        merged = dict(self._params)
        for key in PARAM_KEYS:
            if key in updates:
                merged[key] = updates[key]
        moveit2 = self._get_moveit2()
        if moveit2 is not None:
            try:
                self._apply_params_to(moveit2, merged)
            except Exception as e:
                return {"ok": False, "error": str(e), "params": self.get_params()}
        self._params = merged
        return {"ok": True, "error": None, "params": self.get_params()}

    def set_padding(self, padding_m):
        """Apply link_padding (metres) to every link this arm owns, the same
        GetPlanningScene/ApplyPlanningScene pattern inspection_base.py uses."""
        clients = self._get_scene_clients()
        if clients is None:
            return {"ok": False, "error": "not_ready"}
        get_cli, apply_cli = clients
        try:
            from moveit_msgs.srv import GetPlanningScene, ApplyPlanningScene
            from moveit_msgs.msg import PlanningScene, PlanningSceneComponents, LinkPadding

            if not get_cli.wait_for_service(timeout_sec=SCENE_TIMEOUT):
                return {"ok": False, "error": "get_planning_scene unavailable"}
            req = GetPlanningScene.Request()
            req.components.components = (
                PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
                | PlanningSceneComponents.WORLD_OBJECT_NAMES
            )
            future = get_cli.call_async(req)
            if self._wait_future(future, SCENE_TIMEOUT) is None or future.result() is None:
                return {"ok": False, "error": "get_planning_scene failed"}

            names = list(future.result().scene.allowed_collision_matrix.entry_names)
            arm_links = [n for n in names if self._link_re.match(n)]
            if not arm_links:
                return {"ok": False, "error": "no links matched for this arm"}

            diff = PlanningScene()
            diff.is_diff = True
            diff.link_padding = [
                LinkPadding(link_name=n, padding=float(padding_m)) for n in arm_links
            ]
            if not apply_cli.wait_for_service(timeout_sec=SCENE_TIMEOUT):
                return {"ok": False, "error": "apply_planning_scene unavailable"}
            af = apply_cli.call_async(ApplyPlanningScene.Request(scene=diff))
            if self._wait_future(af, SCENE_TIMEOUT) is None or af.result() is None:
                return {"ok": False, "error": "apply_planning_scene failed"}
            if not af.result().success:
                return {"ok": False, "error": "apply_planning_scene reported failure"}

            self._padding_m = float(padding_m)
            return {"ok": True, "error": None, "links_padded": len(arm_links)}
        except Exception as e:
            return {"ok": False, "error": str(e)}


class FreeMoveManager:
    """Owns both arms; started/stopped together by the dashboard."""

    def __init__(self):
        self.ur = FreeMoveArm(
            "ur", UR_JOINT_NAMES, UR_END_EFFECTOR, UR_GROUP, UR_LINK_RE,
            default_velocity=UR_DEFAULT_VELOCITY, default_acceleration=UR_DEFAULT_ACCELERATION)
        self.kawasaki = FreeMoveArm(
            "kawasaki", KAWASAKI_JOINT_NAMES, KAWASAKI_END_EFFECTOR, KAWASAKI_GROUP,
            KAWASAKI_LINK_RE,
            default_velocity=KAWASAKI_DEFAULT_VELOCITY, default_acceleration=KAWASAKI_DEFAULT_ACCELERATION)
        self._active = False

    @property
    def active(self):
        return self._active

    def start(self):
        if self._active:
            return
        self._active = True
        self.ur.start()
        self.kawasaki.start()

    def stop(self):
        self._active = False
        self.ur.cancel()
        self.kawasaki.cancel()
        self.ur.stop()
        self.kawasaki.stop()

    def arm(self, name):
        return {"ur": self.ur, "kawasaki": self.kawasaki}.get(name)


manager = FreeMoveManager()
