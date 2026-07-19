#!/usr/bin/env python3
"""
Single-arm inspection node -- shared base.

Each concrete inspection node (UR, Kawasaki) drives EXACTLY ONE arm: it plans that
arm (collision-aware, via the real MoveIt) to each of its coverage viewpoints,
dispatches straight to that arm's controller, then grabs the latest SICK cloud from
its own camera(s), transforms it into the world frame, and writes per viewpoint:
    <base>/<sim|real>_pcds/<ur|kawasaki>_data/pcds/<N>.pcd   -- the cloud
    <base>/<sim|real>_pcds/<ur|kawasaki>_data/poses/<N>.txt  -- OctoMap origin
                             "x y z qx qy qz qw" (world frame)

The two arms run as SEPARATE processes (one node each) -- there is no inter-arm
barrier; they are parallel by construction. Cross-arm collision awareness is still
honoured because it lives in the shared move_group planning scene (fed by both arms'
/joint_states), not in this executor.

This base holds everything the two arms share (planning, dispatch, trajectory cache,
capture, TF, planning-scene padding, measured-arrival gating helpers) and a template
`run()` / `run_sequence()`. Subclasses provide the arm's MoveIt2 object, controller,
capture streams, planning style (pose-goal vs joint-space) and settle strategy.
"""
import json
import os
import re
import time

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from moveit_msgs.srv import GetPlanningScene, ApplyPlanningScene
from moveit_msgs.msg import PlanningScene, PlanningSceneComponents, LinkPadding
from sensor_msgs.msg import PointCloud2, JointState
from sensor_msgs_py import point_cloud2 as pc2
from tf2_ros import Buffer, TransformListener
from scipy.spatial.transform import Rotation

import open3d as o3d


class InspectionNodeBase(Node):
    """Base for a single-arm inspection node. `robot_tag` is "ur" or "kawasaki"."""

    def __init__(self, node_name, robot_tag):
        super().__init__(node_name)
        self.robot_tag = robot_tag

        # Filled in by the subclass before run(): the arm's MoveIt2 planner, its
        # controller action client, a human label, capture streams and the plan keys.
        self.moveit = None
        self.ctrl = None
        self.arm_label = robot_tag.upper()
        self.streams = []
        self.viewpoints_key = f"{robot_tag}_viewpoints"
        self.start_pose_param = None       # e.g. "start_pose_ur"
        self.return_home_param = None       # e.g. "return_home_ur"
        self.home_positions_param = None    # e.g. "ur_home_joint_positions"
        # UR skips aligning a FRESH plan to its start (it already starts at the current
        # pose) -- avoids a spurious rail back-and-forth. Kawasaki keeps the old
        # always-align (its _at_pose guard makes it a no-op when already at start).
        self.align_on_fresh = True
        # UR REBASES each trajectory's first point onto the current measured pose (like
        # the proven ur_inspection_scenario playback) instead of doing a separate
        # "align to recorded start" move -- this is what stops the rail lurching back
        # and forth. Kawasaki keeps _align_to_traj_start (its constraint: unchanged).
        self.rebase_to_current = False

        self._declare_shared_params()

        self.plan_file = self.get_parameter("plan_file").value
        self.world_frame = self.get_parameter("world_frame").value
        self.only_sim = bool(self.get_parameter("only_sim").value)

        self._cb = ReentrantCallbackGroup()
        self._plan_timeout = self.get_parameter("plan_timeout_sec").value
        self._motion_timeout = self.get_parameter("motion_timeout_sec").value

        # Planning-scene service clients (collision padding).
        self._get_scene_cli = self.create_client(
            GetPlanningScene, "get_planning_scene", callback_group=self._cb)
        self._apply_scene_cli = self.create_client(
            ApplyPlanningScene, "apply_planning_scene", callback_group=self._cb)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)

        # Latest measured position per joint, MERGED across every /joint_states
        # publisher (the arm broadcaster and the AGV bridge publish in SEPARATE
        # messages). Used by _wait_until_reached to gate advancement on real arrival.
        self._joint_pos = {}
        self._last_plan_error = None  # MoveItErrorCodes.val of the most recent failed plan
        self.create_subscription(
            JointState, "/joint_states", self._joint_states_cb, 10, callback_group=self._cb)

        # Where cached trajectories live: <plans>/trajectories/ (next to the plan JSON)
        # unless trajectory_cache_dir overrides it.
        tcd = self.get_parameter("trajectory_cache_dir").value
        self._traj_dir = os.path.expanduser(tcd) if tcd else os.path.join(
            os.path.dirname(os.path.abspath(os.path.expanduser(self.plan_file))), "trajectories")

    # ------------------------------------------------------------------ #
    # Parameters shared by every arm.
    # ------------------------------------------------------------------ #
    def _declare_shared_params(self):
        self.declare_parameter(
            "plan_file",
            "/home/cem/colcon_ws/src/multirobot_viewpoint_planner/plans/multirobot_viewpoint_plan.json")
        # only_sim gates whether the REAL SICK is captured alongside the sim one.
        self.declare_parameter("only_sim", True)
        # Single-arm gating for REAL-WORLD testing. Each node self-resolves whether it
        # should run (see should_run): only_ur=True -> only the UR node runs; both False
        # (default) -> cooperative run; both True is contradictory and runs both.
        self.declare_parameter("only_ur", False)
        self.declare_parameter("only_kawasaki", False)
        self.declare_parameter("output_base_dir", os.path.expanduser("~/colcon_ws/src/pcds"))

        self.declare_parameter("world_frame", "world")
        self.declare_parameter("capture_settle_sec", 1.5)
        self.declare_parameter("capture_timeout_sec", 6.0)
        self.declare_parameter("transform_to_world", True)

        self.declare_parameter("plan_timeout_sec", 30.0)
        self.declare_parameter("motion_timeout_sec", 120.0)
        self.declare_parameter("allowed_planning_time", 5.0)
        self.declare_parameter("num_planning_attempts", 10)
        self.declare_parameter("plan_attempts", 4)

        # Floor collision box so MoveIt never plans an arm below ground.
        self.declare_parameter("add_ground_plane", True)
        self.declare_parameter("ground_plane_z", -0.02)
        self.declare_parameter("ground_plane_size", 6.0)
        self.declare_parameter("ground_plane_thickness", 0.2)
        self.declare_parameter("collision_padding", 0.04)

        # Measured-arrival gating (Kawasaki uses it; UR trusts its controller).
        self.declare_parameter("arrival_timeout_sec", 90.0)
        self.declare_parameter("arrival_joint_tol", 0.10)   # rad, revolute joints
        self.declare_parameter("arrival_linear_tol", 0.08)  # m, world_to_agv / UR rail

        # Trajectory cache.
        self.declare_parameter("use_trajectory_cache", True)
        self.declare_parameter("force_replan", False)
        self.declare_parameter("trajectory_cache_dir", "")
        self.declare_parameter("traj_start_tol_rad", 0.05)
        self.declare_parameter("traj_start_tol_m", 0.03)

        # Fixed START pose + per-viewpoint HOME-BEFORE detour.
        self.declare_parameter("go_to_start", True)
        self.declare_parameter("home_before_viewpoints", [])

        # Pose-goal planning tolerances (used only when an arm plans to a Cartesian
        # pose goal -- the UR). Kept here because _plan_pose lives in the base.
        self.declare_parameter("pose_goal_pos_tol", 0.005)   # m
        self.declare_parameter("pose_goal_ori_tol", 0.02)    # rad
        # A joint goal whose |angle| is within this margin of the +/-2*pi limit edge is
        # wrapped into (-pi, pi] before planning (OMPL goal-region rescue).
        self.declare_parameter("normalize_edge_margin", 0.2)  # rad

    # ------------------------------------------------------------------ #
    # Hooks the subclass overrides.
    # ------------------------------------------------------------------ #
    def should_run(self):
        """True if THIS arm should run given only_ur/only_kawasaki (+ enable). Both
        only_* True is contradictory -> run both (warned once)."""
        raise NotImplementedError

    def _use_pose_goal(self):
        """True -> plan to a Cartesian pose goal (UR). False -> joint-space."""
        return False

    def _pose_target_link(self):
        """target_link for the pose goal (the camera optical frame). UR only."""
        return None

    def _settle(self, send, targets, label):
        """Wait for a dispatched move to finish -- arm-specific. UR: controller result
        only. Kawasaki: measured arrival."""
        raise NotImplementedError

    def _viewpoint_targets(self, traj, vp):
        """Arrival targets for a viewpoint move (Kawasaki) or None (UR, controller-only)."""
        return None

    # ------------------------------------------------------------------ #
    # Capture streams.
    # ------------------------------------------------------------------ #
    def _make_stream(self, name, robot, topic, sensor_frame, subtree):
        pcd_dir = os.path.join(subtree, "pcds")
        pose_dir = os.path.join(subtree, "poses")
        os.makedirs(pcd_dir, exist_ok=True)
        os.makedirs(pose_dir, exist_ok=True)
        return {
            "name": name, "robot": robot, "topic": topic, "sensor_frame": sensor_frame,
            "subtree": subtree, "pcd_dir": pcd_dir, "pose_dir": pose_dir,
            "latest_cloud": None,
        }

    def _make_cloud_cb(self, stream):
        def _cb(msg: PointCloud2):
            stream["latest_cloud"] = msg
        return _cb

    def _subscribe_streams(self):
        """Create one PointCloud2 subscription per stream. Call after self.streams set."""
        for s in self.streams:
            s["sub"] = self.create_subscription(
                PointCloud2, s["topic"], self._make_cloud_cb(s),
                qos_profile_sensor_data, callback_group=self._cb)
        streams_desc = ", ".join(f"{s['name']}('{s['topic']}' -> {s['subtree']})" for s in self.streams)
        self.get_logger().info(
            f"{self.arm_label} inspection ready [only_sim={self.only_sim}]. "
            f"plan='{self.plan_file}'. Capture streams: {streams_desc}.")

    # ------------------------------------------------------------------ #
    # Planning scene: ground plane + collision padding.
    # ------------------------------------------------------------------ #
    def _add_ground_plane(self):
        if not self.get_parameter("add_ground_plane").value:
            return
        top_z = self.get_parameter("ground_plane_z").value
        size = self.get_parameter("ground_plane_size").value
        thickness = self.get_parameter("ground_plane_thickness").value
        center_z = top_z - thickness / 2.0
        for _ in range(2):
            self.moveit.add_collision_box(
                id="ground_plane",
                size=(size, size, thickness),
                position=(0.0, 0.0, center_z),
                quat_xyzw=(0.0, 0.0, 0.0, 1.0),
                frame_id=self.world_frame,
            )
            time.sleep(0.3)
        self.get_logger().info(
            f"Ground plane added: {size}x{size}x{thickness} m box, top face at z={top_z:.3f} "
            f"in '{self.world_frame}'.")

    def _arm_link_names(self, scene):
        """The MOVING links of THIS arm, to which collision padding is applied.
        Overridden per arm (UR 'ur10e_*', Kawasaki 'link1'..'link6')."""
        raise NotImplementedError

    def _apply_collision_padding(self):
        padding = float(self.get_parameter("collision_padding").value)
        if padding <= 0.0:
            return
        if not self._get_scene_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().warning("get_planning_scene unavailable; skipping collision padding.")
            return
        req = GetPlanningScene.Request()
        req.components.components = (PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
                                     | PlanningSceneComponents.WORLD_OBJECT_NAMES)
        future = self._get_scene_cli.call_async(req)
        if not self._wait_future(future, 5.0) or future.result() is None:
            self.get_logger().warning("get_planning_scene failed; skipping collision padding.")
            return
        arm_links = self._arm_link_names(future.result().scene)
        if not arm_links:
            self.get_logger().warning("No moving arm links found; skipping collision padding.")
            return
        diff = PlanningScene()
        diff.is_diff = True
        diff.link_padding = [LinkPadding(link_name=n, padding=padding) for n in arm_links]
        if not self._apply_scene_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().warning("apply_planning_scene unavailable; skipping collision padding.")
            return
        af = self._apply_scene_cli.call_async(ApplyPlanningScene.Request(scene=diff))
        if not self._wait_future(af, 5.0) or af.result() is None:
            self.get_logger().warning("apply_planning_scene failed; collision padding may be off.")
            return
        self.get_logger().info(
            f"Collision padding {padding * 100:.1f} cm applied to {len(arm_links)} moving arm links. "
            f"success={af.result().success}.")

    # ------------------------------------------------------------------ #
    # Capture + save.
    # ------------------------------------------------------------------ #
    def _lookup_sensor_pose(self, frame_id, fallback_frame):
        for target in (frame_id, fallback_frame):
            if not target:
                continue
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.world_frame, target, rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=1.0))
                t = tf.transform.translation
                q = tf.transform.rotation
                return np.array([t.x, t.y, t.z]), np.array([q.x, q.y, q.z, q.w]), target
            except Exception:
                continue
        return None

    def capture_and_save(self, index, active_robots, fallback_by_robot):
        """Grab a fresh cloud from every stream whose robot moved this step and save
        each into its robot's own tree. Returns True if at least one saved."""
        settle = self.get_parameter("capture_settle_sec").value
        active_streams = [s for s in self.streams if s["robot"] in active_robots]
        for s in active_streams:
            s["latest_cloud"] = None
        time.sleep(settle)

        saved_any = False
        for s in active_streams:
            if self._save_stream(s, index, fallback_by_robot.get(s["robot"])):
                saved_any = True
        return saved_any

    def _save_stream(self, stream, index, fallback_pose=None):
        timeout = self.get_parameter("capture_timeout_sec").value
        tag = f"{stream['name']}][{index}"

        deadline = time.monotonic() + timeout
        while stream["latest_cloud"] is None and time.monotonic() < deadline:
            time.sleep(0.1)
        msg = stream["latest_cloud"]
        if msg is None:
            self.get_logger().error(
                f"[{tag}] No point cloud within {timeout:.1f}s on '{stream['topic']}'. Skipping save.")
            return False

        pts = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        pts = np.array([[p[0], p[1], p[2]] for p in pts], dtype=np.float64)
        if pts.size == 0:
            self.get_logger().warning(f"[{tag}] Captured cloud is empty; skipping save.")
            return False

        pose = self._lookup_sensor_pose(msg.header.frame_id, stream["sensor_frame"])
        if pose is not None:
            trans, quat, used_frame = pose
        elif fallback_pose is not None:
            trans, quat = fallback_pose
            used_frame = "plan_viewpoint(fallback)"
            self.get_logger().warning(
                f"[{tag}] TF sensor pose unavailable; using the planned viewpoint pose as origin.")
        else:
            self.get_logger().error(
                f"[{tag}] No TF and no fallback pose; saving cloud in its own frame, origin=0.")
            trans, quat, used_frame = np.zeros(3), np.array([0, 0, 0, 1.0]), msg.header.frame_id

        if self.get_parameter("transform_to_world").value and pose is not None:
            R = Rotation.from_quat(quat).as_matrix()
            pts = pts @ R.T + trans  # sensor -> world
            cloud_frame = self.world_frame
        else:
            cloud_frame = msg.header.frame_id

        pcd_path = os.path.join(stream["pcd_dir"], f"{index}.pcd")
        txt_path = os.path.join(stream["pose_dir"], f"{index}.txt")
        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(pts)
        o3d.io.write_point_cloud(pcd_path, cloud)
        with open(txt_path, "w") as f:
            f.write(f"{trans[0]:.6f} {trans[1]:.6f} {trans[2]:.6f} "
                    f"{quat[0]:.6f} {quat[1]:.6f} {quat[2]:.6f} {quat[3]:.6f}\n")

        self.get_logger().info(
            f"[{tag}] Saved {len(pts)} points -> {pcd_path} (frame={cloud_frame}); "
            f"origin -> {txt_path} (from {used_frame}).")
        return True

    # ------------------------------------------------------------------ #
    # Small helpers.
    # ------------------------------------------------------------------ #
    @staticmethod
    def _wait_future(future, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if future is not None and future.done():
                return True
            time.sleep(0.05)
        return False

    def _wait_joint_state(self, timeout=15.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.moveit.joint_state is not None:
                return True
            time.sleep(0.1)
        self.get_logger().error(
            f"[{self.arm_label}] joint states never arrived for joints {self.moveit.joint_names}.")
        return False

    @staticmethod
    def _subset(full_names, full_positions, wanted_names):
        lut = dict(zip(full_names, full_positions))
        try:
            return [float(lut[n]) for n in wanted_names]
        except KeyError:
            return None

    def _joint_states_cb(self, msg: JointState):
        for n, p in zip(msg.name, msg.position):
            self._joint_pos[n] = p

    # ------------------------------------------------------------------ #
    # Planning.
    # ------------------------------------------------------------------ #
    def _plan(self, joint_positions, joint_names, label):
        moveit2 = self.moveit
        attempts = max(1, int(self.get_parameter("plan_attempts").value))
        for attempt in range(1, attempts + 1):
            future = moveit2.plan_async(
                joint_positions=list(joint_positions), joint_names=list(joint_names),
                start_joint_state=moveit2.joint_state)
            if future is None:
                self.get_logger().warning(
                    f"[{label}] plan_async returned None (attempt {attempt}/{attempts}).")
            elif not self._wait_future(future, self._plan_timeout):
                self.get_logger().warning(
                    f"[{label}] planning timed out (attempt {attempt}/{attempts}).")
            else:
                traj = moveit2.get_trajectory(future)
                if traj is not None and traj.points:
                    if attempt > 1:
                        self.get_logger().info(
                            f"[{label}] planning succeeded on attempt {attempt}/{attempts}.")
                    return traj
                code = self._plan_error_code(future)
                self._last_plan_error = code
                self.get_logger().warning(
                    f"[{label}] planning failed / empty trajectory "
                    f"(attempt {attempt}/{attempts}) -- MoveIt reason: "
                    f"{self._moveit_err_name(code)}.")
        self.get_logger().error(
            f"[{label}] planning failed after {attempts} attempts "
            f"(last reason: {self._moveit_err_name(self._last_plan_error)}); "
            "skipping this viewpoint.")
        return None

    def _plan_pose(self, position, quat_xyzw, target_link, label):
        """Plan to a CARTESIAN pose goal (target_link at position+quat in world). Used
        for the UR so MoveIt re-solves IK to the intended camera pose instead of
        replaying a possibly wrong-oriented stored joint solution."""
        moveit2 = self.moveit
        attempts = max(1, int(self.get_parameter("plan_attempts").value))
        pos_tol = float(self.get_parameter("pose_goal_pos_tol").value)
        ori_tol = float(self.get_parameter("pose_goal_ori_tol").value)
        for attempt in range(1, attempts + 1):
            future = moveit2.plan_async(
                position=list(position), quat_xyzw=list(quat_xyzw),
                frame_id=self.world_frame, target_link=target_link,
                tolerance_position=pos_tol, tolerance_orientation=ori_tol,
                start_joint_state=moveit2.joint_state)
            if future is None:
                self.get_logger().warning(
                    f"[{label}] plan_async(pose) returned None (attempt {attempt}/{attempts}).")
            elif not self._wait_future(future, self._plan_timeout):
                self.get_logger().warning(
                    f"[{label}] pose planning timed out (attempt {attempt}/{attempts}).")
            else:
                traj = moveit2.get_trajectory(future)
                if traj is not None and traj.points:
                    if attempt > 1:
                        self.get_logger().info(
                            f"[{label}] pose planning succeeded on attempt {attempt}/{attempts}.")
                    return traj
                code = self._plan_error_code(future)
                self._last_plan_error = code
                self.get_logger().warning(
                    f"[{label}] pose planning failed / empty (attempt {attempt}/{attempts}) "
                    f"-- MoveIt reason: {self._moveit_err_name(code)}.")
        self.get_logger().error(
            f"[{label}] pose planning failed after {attempts} attempts "
            f"(last reason: {self._moveit_err_name(self._last_plan_error)}).")
        return None

    @staticmethod
    def _pose_goal_match(cached_pg, pg, pos_tol=0.01, ori_tol=0.05):
        """True if two [px,py,pz,qx,qy,qz,qw] pose goals agree within tolerance."""
        if not cached_pg or len(cached_pg) != 7 or len(pg) != 7:
            return False
        if np.linalg.norm(np.array(cached_pg[:3]) - np.array(pg[:3])) > pos_tol:
            return False
        dot = abs(float(np.dot(np.array(cached_pg[3:]), np.array(pg[3:]))))
        ang = 2.0 * np.arccos(min(1.0, dot))
        return ang <= ori_tol

    @staticmethod
    def _is_prismatic(name):
        # Linear/prismatic axes in either group -- must NOT be angle-wrapped.
        return any(k in name for k in
                   ("mount", "base_to_robot", "world_to_agv", "rail", "wheel"))

    def _normalize_goal(self, goal, names):
        """Rescue ONLY the revolute joints whose stored IK branch sits right at the
        +/-2*pi joint-limit edge (e.g. shoulder_pan = 6.283 = 2*pi, or joint6 = 6.283),
        where OMPL's goal region collapses at the limit and planning returns a bare
        FAILURE. Such a joint is wrapped into (-pi, pi] -- the SAME physical pose (theta
        and theta +/- 2*pi are identical, so collisions/orientation are unchanged) but
        deep in the interior where OMPL can sample.

        Every other joint is LEFT EXACTLY AS PLANNED: the planner already validated those
        angles as in-limits IK, so re-wrapping them (the old 'nearest current angle' rule)
        only risks pushing an in-limits value -- e.g. joint1 = 1.928 -> -4.355 -- OUT of
        its limit, which is what sent the Kawasaki to unrelated, wrongly-oriented poses.
        Returns (new_goal, changed)."""
        out = list(goal)
        changed = False
        two_pi = 2.0 * np.pi
        edge = float(self.get_parameter("normalize_edge_margin").value)
        for i, n in enumerate(names):
            if self._is_prismatic(n):
                continue
            w = float(goal[i])
            if abs(w) < two_pi - edge:
                continue                                  # comfortably interior -> trust it
            nw = (w + np.pi) % two_pi - np.pi             # -> (-pi, pi], deep interior
            if abs(nw - w) > 1e-6:
                out[i] = nw
                changed = True
        return out, changed

    @staticmethod
    def _moveit_err_name(code):
        return {
            1: "SUCCESS", 99999: "FAILURE", -1: "PLANNING_FAILED",
            -2: "INVALID_MOTION_PLAN",
            -3: "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE",
            -4: "CONTROL_FAILED", -5: "UNABLE_TO_AQUIRE_SENSOR_DATA",
            -6: "TIMED_OUT", -7: "PREEMPTED",
            -10: "START_STATE_IN_COLLISION",
            -11: "START_STATE_VIOLATES_PATH_CONSTRAINTS",
            -12: "GOAL_IN_COLLISION", -13: "GOAL_VIOLATES_PATH_CONSTRAINTS",
            -14: "GOAL_CONSTRAINTS_VIOLATED", -15: "INVALID_GROUP_NAME",
            -16: "INVALID_GOAL_CONSTRAINTS", -17: "INVALID_ROBOT_STATE",
            -18: "INVALID_LINK_NAME", -19: "INVALID_OBJECT_NAME",
            -21: "FRAME_TRANSFORM_FAILURE", -22: "COLLISION_CHECKING_UNAVAILABLE",
            -23: "ROBOT_STATE_STALE", -24: "SENSOR_INFO_STALE",
            -25: "COMMUNICATION_FAILURE", -31: "NO_IK_SOLUTION",
        }.get(code, f"UNKNOWN({code})")

    @staticmethod
    def _plan_error_code(future):
        try:
            res = future.result()
            mpr = getattr(res, "motion_plan_response", None)
            if mpr is not None:
                return mpr.error_code.val
            ec = getattr(res, "error_code", None)
            return ec.val if ec is not None else None
        except Exception:
            return None

    # ------------------------------------------------------------------ #
    # Dispatch + completion.
    # ------------------------------------------------------------------ #
    def _dispatch(self, traj, label):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        return self.ctrl.send_goal_async(goal)

    def _await(self, send_future, label):
        if send_future is None:
            return False
        if not self._wait_future(send_future, 5.0):
            self.get_logger().error(f"[{label}] controller did not accept the goal in time.")
            return False
        handle = send_future.result()
        if not handle.accepted:
            self.get_logger().error(f"[{label}] goal REJECTED by controller.")
            return False
        result_future = handle.get_result_async()
        if not self._wait_future(result_future, self._motion_timeout):
            self.get_logger().error(f"[{label}] motion timed out.")
            return False
        wrapped = result_future.result()
        status = wrapped.status
        if status != GoalStatus.STATUS_SUCCEEDED:
            res = getattr(wrapped, "result", None)
            err = getattr(res, "error_code", None)
            errstr = getattr(res, "error_string", "") or ""
            self.get_logger().warning(
                f"[{label}] motion did not succeed (status={status}, "
                f"error_code={err} {self._ftj_err_name(err)}"
                + (f", '{errstr}'" if errstr else "") + ").")
            return False
        return True

    @staticmethod
    def _ftj_err_name(code):
        return {
            0: "SUCCESSFUL", -1: "INVALID_GOAL", -2: "INVALID_JOINTS",
            -3: "OLD_HEADER_TIMESTAMP", -4: "PATH_TOLERANCE_VIOLATED",
            -5: "GOAL_TOLERANCE_VIOLATED",
        }.get(code, "UNKNOWN")

    def _confirm_accepted(self, send_future, label):
        """Confirm the controller ACCEPTED the goal (we then gate on measured arrival,
        not on the action result). Returns True if accepted."""
        if send_future is None:
            return False
        if not self._wait_future(send_future, 5.0):
            self.get_logger().error(f"[{label}] controller did not accept the goal in time.")
            return False
        handle = send_future.result()
        if not handle.accepted:
            self.get_logger().error(f"[{label}] goal REJECTED by controller.")
            return False
        return True

    # ------------------------------------------------------------------ #
    # Measured-arrival gating (used by the Kawasaki subclass).
    # ------------------------------------------------------------------ #
    def _arrival_targets(self, vp):
        """{joint_name: target_position} for this arm's group joints at vp."""
        pos = self._subset(vp["joint_names"], vp["joint_positions"], self.moveit.joint_names)
        if pos is None:
            return {}
        return dict(zip(self.moveit.joint_names, pos))

    def _arrival_targets_from_traj(self, traj, vp):
        """Arrival targets = the trajectory's ACTUAL final commanded joint positions,
        NOT the raw plan joint_positions. This matters because the goal we execute may
        have been angle-normalized (a wound-up 2*pi joint wrapped into the interior):
        gating on the raw plan value then compares against an angle the robot was never
        commanded to (e.g. joint6 target 6.283 vs commanded 0 -> a permanent err=-6.283
        that never clears). Any group joint the trajectory does NOT carry (e.g. the async
        AGV rail, if it is not a planned joint) falls back to its raw plan target so its
        arrival is still gated."""
        final = {}
        if traj is not None and traj.points:
            final = dict(zip(traj.joint_names, traj.points[-1].positions))
        raw = self._arrival_targets(vp)
        out = {}
        for n in self.moveit.joint_names:
            if n in final:
                out[n] = float(final[n])
            elif n in raw:
                out[n] = float(raw[n])
        return out

    def _arrival_tol_for(self, name):
        """Linear joints (AGV rail / UR base rail) use a metre tolerance; every other
        (revolute) joint uses the radian tolerance."""
        if "world_to_agv" in name or "base_to_robot_mount" in name:
            return float(self.get_parameter("arrival_linear_tol").value)
        return float(self.get_parameter("arrival_joint_tol").value)

    def _log_arrival(self, tag, targets, tol, timed_out=False):
        pos = dict(self._joint_pos)
        parts = []
        for n, t in targets.items():
            if n in pos:
                err = pos[n] - t
                mark = "" if abs(err) <= tol[n] else "   <-- NOT reached"
                parts.append(f"{n}: err={err:+.3f} (tol {tol[n]:.3f}){mark}")
            else:
                parts.append(f"{n}: NO STATE in /joint_states   <-- missing feedback")
        body = "\n  ".join(parts)
        if timed_out:
            self.get_logger().warning(f"{tag} arrival TIMEOUT -- capturing anyway:\n  {body}")
        else:
            self.get_logger().info(f"{tag} waiting for arrival:\n  {body}")

    def _wait_until_reached(self, targets, tag):
        """Poll /joint_states until every commanded joint is within tolerance of its
        target, or arrival_timeout_sec elapses. Returns True if reached."""
        timeout = float(self.get_parameter("arrival_timeout_sec").value)
        tol = {n: self._arrival_tol_for(n) for n in targets}
        deadline = time.monotonic() + timeout
        last_log = 0.0
        while time.monotonic() < deadline:
            pos = dict(self._joint_pos)
            if all(n in pos for n in targets):
                if all(abs(pos[n] - targets[n]) <= tol[n] for n in targets):
                    self.get_logger().info(f"{tag} reached target (all joints within tolerance).")
                    return True
            now = time.monotonic()
            if now - last_log >= 5.0:
                last_log = now
                self._log_arrival(tag, targets, tol)
            time.sleep(0.1)
        self._log_arrival(tag, targets, tol, timed_out=True)
        return False

    @staticmethod
    def _fallback_pose(vp):
        return (np.array(vp["position"], dtype=float),
                Rotation.from_matrix(np.array(vp["rotation"], dtype=float)).as_quat())

    # ------------------------------------------------------------------ #
    # Trajectory cache: replay a saved path if valid, else plan + save.
    # ------------------------------------------------------------------ #
    def _start_tol_for(self, name):
        if "world_to_agv" in name or "base_to_robot_mount" in name:
            return float(self.get_parameter("traj_start_tol_m").value)
        return float(self.get_parameter("traj_start_tol_rad").value)

    def _traj_cache_path(self, vp_id):
        return os.path.join(self._traj_dir, f"{self.robot_tag}_{vp_id}.json")

    def _cache_valid(self, cached, joint_names, goal, pose_goal=None):
        """A cached path is valid only if it targets the SAME joints and the SAME goal
        as the current plan. In pose mode the cache must carry a matching pose_goal."""
        if list(cached.get("joint_names", [])) != list(joint_names):
            return False
        if pose_goal is not None:
            return self._pose_goal_match(cached.get("pose_goal"), pose_goal)
        gp = cached.get("goal_positions")
        if gp is None or goal is None or len(gp) != len(goal):
            return False
        return all(abs(a - b) <= self._start_tol_for(n)
                   for n, a, b in zip(joint_names, gp, goal))

    def _serialize_traj(self, jt, vp_id, goal, pose_goal=None):
        pts = []
        for p in jt.points:
            t = p.time_from_start
            pts.append({
                "positions": [float(x) for x in p.positions],
                "velocities": [float(x) for x in p.velocities],
                "accelerations": [float(x) for x in p.accelerations],
                "time_from_start": t.sec + t.nanosec * 1e-9,
            })
        out = {
            "arm": self.robot_tag,
            "vp_id": vp_id,
            "joint_names": list(jt.joint_names),
            "goal_positions": [float(x) for x in goal] if goal is not None else [],
            "start_positions": [float(x) for x in jt.points[0].positions] if jt.points else [],
            "points": pts,
        }
        if pose_goal is not None:
            out["pose_goal"] = [float(x) for x in pose_goal]
        return out

    @staticmethod
    def _deserialize_traj(cached):
        jt = JointTrajectory()
        jt.joint_names = list(cached["joint_names"])
        for pd in cached["points"]:
            p = JointTrajectoryPoint()
            p.positions = [float(x) for x in pd.get("positions", [])]
            p.velocities = [float(x) for x in pd.get("velocities", [])]
            p.accelerations = [float(x) for x in pd.get("accelerations", [])]
            t = float(pd.get("time_from_start", 0.0))
            p.time_from_start = Duration(sec=int(t), nanosec=int(round((t - int(t)) * 1e9)))
            jt.points.append(p)
        return jt

    def _get_trajectory(self, vp, label, idx, force_fresh=False):
        """Cache-or-plan. Returns (JointTrajectory | None, from_cache: bool).
        from_cache lets the caller skip aligning a FRESH plan to its start (it already
        starts at the current pose)."""
        moveit2 = self.moveit
        vp_id = vp.get("id") or f"{self.robot_tag}_step{idx}"
        goal = self._subset(vp["joint_names"], vp["joint_positions"], moveit2.joint_names)

        # Pose mode (UR): plan to the camera optical-frame POSE the planner solved IK for,
        # so MoveIt re-derives a valid, correctly-oriented joint solution. Joint-space
        # arms (Kawasaki) fall through unchanged.
        use_pose = (self._use_pose_goal()
                    and vp.get("position") is not None and vp.get("rotation") is not None)
        pose_goal = position = quat = None
        target_link = self._pose_target_link()
        if use_pose:
            position = [float(x) for x in vp["position"]]
            quat = Rotation.from_matrix(np.array(vp["rotation"], dtype=float)).as_quat()  # xyzw
            pose_goal = list(position) + [float(x) for x in quat]

        if goal is None and not use_pose:
            self.get_logger().warning(f"[{label}] viewpoint {vp_id} missing a group joint; skipping.")
            return None, False

        use_cache = bool(self.get_parameter("use_trajectory_cache").value)
        force = bool(self.get_parameter("force_replan").value)
        path = self._traj_cache_path(vp_id)

        # force_fresh (home-before detour): plan straight from the CURRENT pose (home)
        # to the goal and do NOT touch the cache -- we deliberately approach from home.
        if use_cache and not force and not force_fresh and os.path.exists(path):
            try:
                with open(path) as f:
                    cached = json.load(f)
            except Exception as e:
                cached = None
                self.get_logger().warning(f"[{label}] {vp_id}: could not read cache {path}: {e}")
            if cached and self._cache_valid(cached, moveit2.joint_names, goal, pose_goal=pose_goal):
                self.get_logger().info(
                    f"[{label}] {vp_id}: using CACHED trajectory ({len(cached['points'])} pts) "
                    "(will align to its start first).")
                return self._deserialize_traj(cached), True
            elif cached is not None:
                self.get_logger().info(f"[{label}] {vp_id}: cached goal differs from plan; replanning.")

        traj = None
        saved_pose_goal = None
        used_goal = goal
        if use_pose:
            # PRIMARY for the UR: Cartesian pose goal -> correct camera orientation.
            traj = self._plan_pose(position, quat, target_link, f"{label} {vp_id} [pose]")
            if traj is not None:
                saved_pose_goal = pose_goal
                used_goal = [float(x) for x in traj.points[-1].positions]

        if traj is None and goal is not None:
            # Joint-space (Kawasaki always; UR only if the pose goal failed). If any
            # revolute joint sits at a wound-up/limit-edge value, try the angle-normalized
            # equivalent FIRST (same pose, better-conditioned goal).
            ngoal, changed = self._normalize_goal(goal, moveit2.joint_names)
            attempts_order = []
            if changed:
                attempts_order.append((ngoal, " [normalized]"))
            attempts_order.append((goal, ""))
            for cand, sfx in attempts_order:
                traj = self._plan(cand, moveit2.joint_names, f"{label} {vp_id}{sfx}")
                if traj is not None:
                    used_goal = cand
                    break

        if traj is None:
            return None, False
        if use_cache and not force_fresh:
            try:
                os.makedirs(self._traj_dir, exist_ok=True)
                tmp = path + ".tmp"
                with open(tmp, "w") as f:
                    json.dump(self._serialize_traj(traj, vp_id, used_goal,
                                                   pose_goal=saved_pose_goal), f, indent=1)
                os.replace(tmp, path)
                self.get_logger().info(f"[{label}] {vp_id}: trajectory saved -> {path}")
            except Exception as e:
                self.get_logger().warning(f"[{label}] {vp_id}: could not save trajectory: {e}")
        return traj, False

    # ------------------------------------------------------------------ #
    # Moves.
    # ------------------------------------------------------------------ #
    def _home_arm(self, home_deg, label):
        """Plan (collision-aware) + dispatch this arm to its home pose. home_deg index 0
        is a linear axis in METRES; indices 1..6 are revolute joints in DEGREES."""
        moveit2, ctrl = self.moveit, self.ctrl
        if moveit2 is None or ctrl is None:
            return
        if len(home_deg) != len(moveit2.joint_names):
            self.get_logger().warning(
                f"[{label} home] expected {len(moveit2.joint_names)} joint values, got "
                f"{len(home_deg)}; skipping homing.")
            return
        home_rad = [float(home_deg[0])] + [float(np.deg2rad(v)) for v in home_deg[1:]]
        self.get_logger().info(
            f"[{label}] returning to home pose (axis0={home_deg[0]} m, joints={home_deg[1:]} deg)...")
        traj = self._plan(home_rad, moveit2.joint_names, f"{label} home")
        if traj is None:
            self.get_logger().warning(
                f"[{label} home] could not plan a path to the home pose; leaving the arm where it is.")
            return
        send = self._dispatch(traj, f"{label} home")
        if self._await(send, f"{label} home"):
            self.get_logger().info(f"[{label}] returned to home pose.")
        else:
            self.get_logger().warning(f"[{label} home] homing motion did not complete successfully.")

    def _at_pose(self, targets):
        """True iff the robot's measured joints are all within the START-match tol of
        targets."""
        pos = dict(self._joint_pos)
        for n, t in targets.items():
            if n not in pos or abs(pos[n] - t) > self._start_tol_for(n):
                return False
        return True

    def _move_to_pose(self, pose_deg, label):
        """Move this arm to pose_deg (index 0 linear m, 1..6 deg) UNLESS already there.
        Planned collision-aware; completion decided by the arm's _settle."""
        moveit2, ctrl = self.moveit, self.ctrl
        if moveit2 is None or ctrl is None:
            return
        if len(pose_deg) != len(moveit2.joint_names):
            self.get_logger().warning(
                f"[{label}] expected {len(moveit2.joint_names)} joint values, got "
                f"{len(pose_deg)}; skipping.")
            return
        target_rad = [float(pose_deg[0])] + [float(np.deg2rad(v)) for v in pose_deg[1:]]
        targets = dict(zip(moveit2.joint_names, target_rad))
        if self._at_pose(targets):
            self.get_logger().info(f"[{label}] already at start pose; no move needed.")
            return
        self.get_logger().info(
            f"[{label}] NOT at start pose -> moving there "
            f"(axis0={pose_deg[0]} m, joints={list(pose_deg[1:])} deg).")
        traj = self._plan(target_rad, moveit2.joint_names, label)
        if traj is None:
            self.get_logger().warning(f"[{label}] could not plan to start pose; leaving arm where it is.")
            return
        send = self._dispatch(traj, label)
        self._settle(send, targets, f"[{label}]")

    def _rebase_traj_to_current(self, traj, label):
        """Overwrite the trajectory's FIRST point with the arm's CURRENT measured joint
        positions (velocities/accelerations 0, t=0), so the controller interpolates
        smoothly from where the robot actually IS straight to the goal. This is exactly
        the proven ur_inspection_scenario playback trick and it REPLACES the separate
        'align to recorded start' move (_align_to_traj_start) that made the rail lurch
        back and forth. If any joint's measured value is missing we leave the recorded
        start as-is (never fabricate a start point)."""
        if traj is None or not traj.points:
            return
        pos = dict(self._joint_pos)
        names = list(traj.joint_names)
        if not all(n in pos for n in names):
            self.get_logger().warning(
                f"[{label}] cannot rebase trajectory start (missing joint feedback); "
                "running recorded start as-is.")
            return
        p0 = traj.points[0]
        p0.positions = [float(pos[n]) for n in names]
        p0.velocities = [0.0] * len(names)
        p0.accelerations = [0.0] * len(names)
        p0.time_from_start = Duration(sec=0, nanosec=0)

    def _align_to_traj_start(self, traj, from_cache, label):
        """Ensure the arm is at `traj`'s FIRST point before it is dispatched. A CACHED
        trajectory may start a little off, so we do ONE short collision-aware move to
        its start. A FRESH plan already starts at the current pose, so it is skipped
        (self.align_on_fresh False for the UR -- avoids a spurious rail back-and-forth)."""
        moveit2, ctrl = self.moveit, self.ctrl
        if moveit2 is None or ctrl is None or traj is None or not traj.points:
            return
        if not from_cache and not self.align_on_fresh:
            return
        start = [float(x) for x in traj.points[0].positions]
        targets = dict(zip(list(traj.joint_names), start))
        if self._at_pose(targets):
            return  # already at the recorded start (fresh plans always land here)
        self.get_logger().info(f"[{label}] aligning to cached trajectory start before replay.")
        plan = self._plan(start, list(traj.joint_names), f"{label} align-start")
        if plan is None:
            self.get_logger().warning(
                f"[{label}] could not plan to the cached start; running cached traj from current pose.")
            return
        send = self._dispatch(plan, f"{label} align-start")
        self._settle(send, targets, f"[{label} align]")

    def _go_to_start(self):
        """Send this arm to its fixed START pose before the sequence runs, so the first
        (possibly cached) trajectory begins from a known, matching state."""
        if not self.get_parameter("go_to_start").value:
            return
        self._move_to_pose(
            list(self.get_parameter(self.start_pose_param).value), f"{self.arm_label} start")

    # ------------------------------------------------------------------ #
    # Sequence.
    # ------------------------------------------------------------------ #
    def run_sequence(self, vps):
        """Plan/align/dispatch/settle/capture each of THIS arm's viewpoints in order.
        Returns the number of viewpoints whose cloud was captured."""
        label = self.arm_label
        home_before = list(self.get_parameter("home_before_viewpoints").value)
        start_pose = list(self.get_parameter(self.start_pose_param).value)
        n = len(vps)
        saved = 0
        for idx, vp in enumerate(vps, start=1):
            vp_id = vp.get("id")
            home_first = vp_id in home_before
            if home_first:
                self.get_logger().warning(
                    f"[{label} {idx}/{n}] {vp_id} in home_before_viewpoints -> homing first, "
                    "approaching FRESH from home.")
                self._move_to_pose(start_pose, f"{label} home-before")

            has_goal = bool(vp.get("joint_positions")) or (
                self._use_pose_goal() and vp.get("position") is not None
                and vp.get("rotation") is not None)
            if not has_goal:
                self.get_logger().warning(f"[{label} {idx}/{n}] {vp_id} has no goal; skipping.")
                continue

            traj, from_cache = self._get_trajectory(vp, label, idx, force_fresh=home_first)
            if traj is None:
                self.get_logger().warning(f"[{label} {idx}/{n}] {vp_id} could not be planned; skipping.")
                continue

            if self.rebase_to_current:
                # UR: rebase the trajectory onto the current pose (no separate move).
                self._rebase_traj_to_current(traj, label)
            else:
                # Kawasaki: unchanged -- separate align to the recorded start.
                self._align_to_traj_start(traj, from_cache, label)
            self.get_logger().info(f"[{label} {idx}/{n}] Moving {label}->{vp_id}.")
            send = self._dispatch(traj, label)
            targets = self._viewpoint_targets(traj, vp)
            self._settle(send, targets, f"[{label} {idx}/{n}]")

            if self.capture_and_save(idx, {self.robot_tag}, {self.robot_tag: self._fallback_pose(vp)}):
                saved += 1

        self.get_logger().info(f"[{label}] sequence complete: {saved}/{n} viewpoints captured.")
        return saved

    # ------------------------------------------------------------------ #
    def run(self):
        if not os.path.exists(self.plan_file):
            self.get_logger().error(f"Plan file not found: {self.plan_file}.")
            return
        if not self.should_run():
            self.get_logger().info(
                f"{self.arm_label} node idle (only_ur/only_kawasaki/enable gating); exiting.")
            return
        with open(self.plan_file) as f:
            plan = json.load(f)
        vps = plan.get(self.viewpoints_key, [])
        if not vps:
            self.get_logger().error(
                f"Plan has no '{self.viewpoints_key}'; nothing to execute for {self.arm_label}.")
            return

        if not self.ctrl.wait_for_server(timeout_sec=15.0):
            self.get_logger().error(
                f"{self.arm_label} controller not available; aborting.")
            return
        if not self._wait_joint_state():
            self.get_logger().error(f"No /joint_states for the {self.arm_label} group; aborting.")
            return

        self._add_ground_plane()
        self._apply_collision_padding()

        self.get_logger().info(
            f"{self.arm_label} executing {len(vps)} viewpoints (independent single-arm node).")

        # Send this arm to its known START pose first, so the first (possibly cached)
        # trajectory begins from the exact state it was recorded at.
        self._go_to_start()

        t_start = time.monotonic()
        saved = self.run_sequence(vps)

        if self.get_parameter(self.return_home_param).value:
            self._home_arm(list(self.get_parameter(self.home_positions_param).value), self.arm_label)

        trees = " + ".join(s["subtree"] for s in self.streams)
        self.get_logger().info(
            f"{self.arm_label} inspection complete [only_sim={self.only_sim}]: "
            f"{saved}/{len(vps)} viewpoints captured into {trees} "
            f"in {time.monotonic() - t_start:.1f}s.")

    # ------------------------------------------------------------------ #
    def _resolve_only_flags(self):
        """(only_ur, only_kawasaki) after resolving the both-True contradiction to
        'run both' (warned once)."""
        only_ur = bool(self.get_parameter("only_ur").value)
        only_kawa = bool(self.get_parameter("only_kawasaki").value)
        if only_ur and only_kawa:
            self.get_logger().warning(
                "only_ur AND only_kawasaki are both True (contradictory); running BOTH arms.")
            only_ur = only_kawa = False
        return only_ur, only_kawa
