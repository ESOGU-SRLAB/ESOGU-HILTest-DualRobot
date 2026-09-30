#!/usr/bin/env python3
"""
Real UR10e inspection executor (drives the real MoveIt/controllers via
pymoveit2_real; the real->sim bridge mirrors the motion onto the Gazebo robot).

For every UR10e viewpoint in viewpoint_planner/plans/viewpoint_plan.json it:
  1. sends the UR10e to the viewpoint -- using the JOINT solution already stored
     in the plan, so no IK/planning ambiguity at execution time;
  2. once it has arrived, grabs the latest SICK point cloud, transforms it into
     the world frame, and writes:
        <output_dir>/<N>.pcd  -- the captured cloud (world frame)
        <output_dir>/<N>.txt  -- the camera/sensor origin for OctoMap:
                                 "x y z qx qy qz qw" (world frame)
     with N = 1, 2, 3, ...

This replaces the old MOCK viewpoint_executor_node for real/sim runs.
"""
import json
import os
import time
from threading import Thread

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.srv import GetPlanningScene, ApplyPlanningScene
from moveit_msgs.msg import PlanningScene, PlanningSceneComponents, LinkPadding
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from tf2_ros import Buffer, TransformListener
from scipy.spatial.transform import Rotation

import open3d as o3d

from pymoveit2_real import MoveIt2 as MoveIt2Real
from pymoveit2_real.robots import ur as ur_robot


class InspectionExecutor(Node):
    def __init__(self):
        super().__init__("inspection_executor_node")

        self.declare_parameter("plan_file", '/home/cem/colcon_ws/src/viewpoint_planner/plans/viewpoint_plan.json')
        # only_sim gates whether the REAL SICK is captured alongside the sim one:
        #   True            -> not connected to the physical system: capture ONLY
        #                      the Gazebo SICK. The real stream is never even
        #                      subscribed, so nothing waits on /sick_points.
        #   False (default) -> connected to the real cell: capture BOTH the sim
        #                      SICK AND the real SICK at every viewpoint, writing
        #                      each into its own tree (single_ur10e/sim_data/ and
        #                      single_ur10e/real_data/).
        # The sim stream ALWAYS runs either way.
        self.declare_parameter("only_sim", False)
        # Root of the capture tree. This is the SINGLE-UR10e scenario, so each
        # stream gets its own <base>/single_ur10e/<sim|real>_data/ subtree (kept
        # separate from the multi-robot tree so the solo-vs-multi comparison never
        # mixes their clouds). Each subtree is split into pcds/ (the clouds) and
        # poses/ (the origin txts) so the octomap builder can point --pcd_dir /
        # --pose_dir straight at them.
        self.declare_parameter("output_base_dir", os.path.expanduser("~/colcon_ws/src/pcds"))
        # Point cloud topic per stream. The real hardware publishes on /sick_points.
        self.declare_parameter("sim_pointcloud_topic", "/sim/pointcloud")
        self.declare_parameter("real_pointcloud_topic", "/sick_points")
        self.declare_parameter("world_frame", "world")
        # Fallback sensor frame per stream if the cloud has no usable frame_id.
        self.declare_parameter("sim_sensor_frame", "sim_ur10e_depth_optical_frame")
        self.declare_parameter("real_sensor_frame", "ur10e_depth_optical_frame")
        self.declare_parameter("capture_settle_sec", 1.5)
        self.declare_parameter("capture_timeout_sec", 6.0)
        self.declare_parameter("transform_to_world", True)
        self.declare_parameter("ur_velocity", 0.1)
        self.declare_parameter("ur_acceleration", 0.1)
        # The controller is driven DIRECTLY (bypassing the single move_group action
        # server). This is the FollowJointTrajectory action server of the UR's own
        # controller.
        self.declare_parameter(
            "ur_controller_action", "/scaled_joint_trajectory_controller/follow_joint_trajectory")
        self.declare_parameter("plan_timeout_sec", 30.0)
        self.declare_parameter("motion_timeout_sec", 120.0)
        # Execution traversal order. When True, the loaded viewpoints are reordered
        # into a sweep that STARTS at the largest-Y viewpoint (the FRONT of the
        # chassis) and then proceeds to the nearest neighbour each step
        # (nearest-neighbour + 2-opt). This overrides whatever order the plan JSON
        # was saved in. False -> visit the plan's stored order as-is.
        self.declare_parameter("order_by_proximity", True)
        # Motion-planning effort. Every viewpoint here already PASSED an IK check at
        # plan time (a joint solution is stored), so a plan failure means the
        # collision-aware motion planner just didn't find a path in the time it was
        # given -- NOT that the pose is unreachable. Give it a lot more room and
        # retry, so we actually visit every reachable viewpoint instead of skipping.
        #   allowed_planning_time     : seconds per single planner run (was 0.5 in
        #                               pymoveit2 default -- far too short here).
        #   num_planning_attempts     : parallel planner runs inside ONE plan call.
        #   plan_attempts             : how many times we re-issue the whole plan
        #                               call before giving up (RRT-Connect is
        #                               randomized, so a fresh try often succeeds).
        self.declare_parameter("allowed_planning_time", 10.0)
        self.declare_parameter("num_planning_attempts", 10)
        self.declare_parameter("plan_attempts", 4)
        # OMPL planner to use for the UR group. Empty string keeps move_group's
        # default (RRTConnect for this cell). Set e.g. "LBKPIECEkConfigDefault" or
        # "BKPIECEkConfigDefault" to A/B different planners without touching
        # ompl_planning.yaml. Must be one of the group's configured planners in
        # <moveit_config>/config/ompl_planning.yaml (real_ur10e block).
        self.declare_parameter("planner_id", "")
        # Ground plane: a large thin collision box added to the planning scene so
        # MoveIt treats any motion below the floor as a collision and never plans
        # the arm underground. Its TOP surface sits at `ground_plane_z` (world
        # frame). Default -0.02 m keeps a 2 cm gap below the nominal z=0 floor so
        # it never clips table/chassis geometry that legitimately rests ON the
        # floor (which would wrongly flag the start state as in-collision); raise
        # it to 0.0 for a strict floor if no start-state collisions appear.
        self.declare_parameter("add_ground_plane", True)
        self.declare_parameter("ground_plane_z", -0.02)
        self.declare_parameter("ground_plane_size", 6.0)
        self.declare_parameter("ground_plane_thickness", 0.2)
        # Collision padding: inflate the MOVING arm links by this margin (m) for
        # collision checking, so MoveIt keeps the arm this far from every obstacle
        # (chassis, tables, the floor) instead of skimming past them. This is real
        # collision *padding*, not the allowed-collision matrix
        # (which does the opposite -- it lets specific pairs touch). Every 'ur10e_'
        # collision body is padded: the moving arm chain, the static furniture AND
        # the inspected chassis (its collision mesh under-approximates the real
        # part, so it needs the margin -- excluding it let the arm hit the chassis).
        # 0.0 disables it. Lower it (e.g. 0.02) to widen inspection corridors at the
        # cost of a smaller safety margin.
        self.declare_parameter("collision_padding", 0.03)
        # EXTRA padding applied ONLY to the inspected chassis links
        # (every 'ur10e_...chassis...' collision body: ur10e_chassis +
        # ur10e_chassis_partN). The real cables / cable channels that run along
        # the arm are NOT modelled, so they snag on the chassis where the arm
        # dips into its cavities. Giving the chassis a larger padding than the
        # rest keeps the arm this far off the chassis (leaving room for the
        # unmodelled cables) WITHOUT over-constraining the arm against the floor
        # and furniture (those stay at `collision_padding`). Set to 0.0 to fall
        # back to a single uniform `collision_padding` for everything.
        # NOTE: this is a per-link ABSOLUTE padding, not additive with
        # `collision_padding`; the chassis uses this value, the rest uses
        # `collision_padding`.
        self.declare_parameter("chassis_collision_padding", 0.10)
        # Home / rest pose the UR returns to once the whole inspection is done.
        # Seven values in ur_robot.joint_names() order: the FIRST is the linear
        # rail (ur10e_base_to_robot_mount) in METRES; the remaining six are the
        # arm joints in DEGREES (converted to radians internally). The move is
        # planned collision-aware exactly like every viewpoint hop.
        self.declare_parameter("return_home", True)
        self.declare_parameter(
            "home_joint_positions", [1.0, 0.0, -90.0, 0.0, -90.0, 0.0, 0.0])

        self.plan_file = self.get_parameter("plan_file").value
        self.world_frame = self.get_parameter("world_frame").value

        # Build the capture streams. The sim stream always runs; the real stream
        # is added only when connected to the physical system (only_sim == False).
        # Each stream owns its topic, fallback sensor frame, output dirs and its
        # own latest-cloud buffer, so both can be captured at the same viewpoint.
        self.only_sim = bool(self.get_parameter("only_sim").value)
        base = os.path.expanduser(self.get_parameter("output_base_dir").value)
        # Single-UR10e scenario tree: <base>/single_ur10e/{sim_data,real_data}/.
        scenario_dir = os.path.join(base, "single_ur10e")
        self.streams = [self._make_stream(
            "sim",
            self.get_parameter("sim_pointcloud_topic").value,
            self.get_parameter("sim_sensor_frame").value,
            os.path.join(scenario_dir, "sim_data"))]
        if not self.only_sim:
            self.streams.append(self._make_stream(
                "real",
                self.get_parameter("real_pointcloud_topic").value,
                self.get_parameter("real_sensor_frame").value,
                os.path.join(scenario_dir, "real_data")))

        cb = ReentrantCallbackGroup()
        self._plan_timeout = self.get_parameter("plan_timeout_sec").value
        self._motion_timeout = self.get_parameter("motion_timeout_sec").value

        # Planning-scene service clients (used to push collision padding into the
        # shared scene that move_group plans against).
        self._get_scene_cli = self.create_client(
            GetPlanningScene, "get_planning_scene", callback_group=cb)
        self._apply_scene_cli = self.create_client(
            ApplyPlanningScene, "apply_planning_scene", callback_group=cb)

        # --- UR10e MoveIt2 (rail + 6 arm joints) -- PLANNING ONLY ---
        # We only use MoveIt to *plan* (collision-aware); execution is dispatched
        # straight to the controller below so both arms can move at once.
        self.ur = MoveIt2Real(
            node=self,
            joint_names=ur_robot.joint_names(),
            base_link_name=self.world_frame,
            end_effector_name=ur_robot.end_effector_name(),
            group_name=ur_robot.MOVE_GROUP_ARM,
            callback_group=cb,
        )
        self.ur.max_velocity = self.get_parameter("ur_velocity").value
        self.ur.max_acceleration = self.get_parameter("ur_acceleration").value
        self.ur.allowed_planning_time = self.get_parameter("allowed_planning_time").value
        self.ur.num_planning_attempts = self.get_parameter("num_planning_attempts").value
        # Pin the OMPL planner if one was requested; otherwise leave move_group's
        # default (empty planner_id) untouched.
        planner_id = self.get_parameter("planner_id").value
        if planner_id:
            self.ur.planner_id = planner_id
            self.get_logger().info(
                f"UR motion planner pinned to '{planner_id}' "
                f"(allowed_planning_time={self.ur.allowed_planning_time}s).")
        else:
            self.get_logger().info(
                "UR motion planner: move_group default (RRTConnect), "
                f"allowed_planning_time={self.ur.allowed_planning_time}s.")
        # Direct controller action client (execution, bypassing move_group).
        self._ur_ctrl = ActionClient(
            self, FollowJointTrajectory,
            self.get_parameter("ur_controller_action").value, callback_group=cb)

        # --- Point cloud subscriptions, one per stream (bridge is lazy:
        #     subscribing to /sim/pointcloud activates the sim->real bridge). ---
        for s in self.streams:
            s["sub"] = self.create_subscription(
                PointCloud2, s["topic"],
                self._make_cloud_cb(s), qos_profile_sensor_data, callback_group=cb,
            )

        # --- TF for sensor pose / cloud transform ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)

        streams_desc = ", ".join(f"{s['name']}('{s['topic']}' -> {s['subtree']})" for s in self.streams)
        self.get_logger().info(
            f"InspectionExecutor ready [only_sim={self.only_sim}]. plan='{self.plan_file}'. "
            f"Capture streams: {streams_desc}."
        )

    def _make_stream(self, name, topic, sensor_frame, subtree):
        """Create a capture-stream descriptor and its output directories."""
        pcd_dir = os.path.join(subtree, "pcds")
        pose_dir = os.path.join(subtree, "poses")
        os.makedirs(pcd_dir, exist_ok=True)
        os.makedirs(pose_dir, exist_ok=True)
        return {
            "name": name, "topic": topic, "sensor_frame": sensor_frame,
            "subtree": subtree, "pcd_dir": pcd_dir, "pose_dir": pose_dir,
            "latest_cloud": None,
        }

    def _make_cloud_cb(self, stream):
        """Return a subscription callback that buffers the latest cloud for `stream`."""
        def _cb(msg: PointCloud2):
            stream["latest_cloud"] = msg
        return _cb

    # ------------------------------------------------------------------ #
    def _add_ground_plane(self):
        """Publish a floor collision box into the shared planning scene.

        A single large, thin box in the world frame whose top face sits at
        `ground_plane_z`. move_group's PlanningSceneMonitor picks it up on the
        /collision_object topic, so every subsequent plan treats going below the
        floor as a collision. Published a couple of times with a short gap to beat
        any subscription race at startup."""
        if not self.get_parameter("add_ground_plane").value:
            return
        top_z = self.get_parameter("ground_plane_z").value
        size = self.get_parameter("ground_plane_size").value
        thickness = self.get_parameter("ground_plane_thickness").value
        # Box is centered half a thickness below its top face.
        center_z = top_z - thickness / 2.0
        for _ in range(2):
            self.ur.add_collision_box(
                id="ground_plane",
                size=(size, size, thickness),
                position=(0.0, 0.0, center_z),
                quat_xyzw=(0.0, 0.0, 0.0, 1.0),
                frame_id=self.world_frame,
            )
            time.sleep(0.3)
        self.get_logger().info(
            f"Ground plane added to planning scene: {size}x{size}x{thickness} m box, "
            f"top face at z={top_z:.3f} in '{self.world_frame}' (blocks motion below the floor)."
        )

    # ------------------------------------------------------------------ #
    # Links under the 'ur10e_' prefix to EXCLUDE from collision padding. Kept EMPTY
    # on purpose: excluding the chassis was tried and REVERTED -- the chassis
    # collision meshes under-approximate the real part, so the arm's own 0.03 m
    # padding alone was not enough and the arm CONTACTED the chassis. Padding the
    # chassis too (0.03 arm + 0.03 chassis => 0.06 m clearance) is what keeps the
    # arm safely off it. To trade a little safety for wider inspection corridors,
    # lower `collision_padding` instead of excluding the chassis here.
    _NO_PAD_TOKENS = ()

    # Substring that identifies the inspected-chassis collision bodies
    # (ur10e_chassis, ur10e_chassis_partN). These get `chassis_collision_padding`
    # instead of the base `collision_padding`. No arm or furniture link contains
    # it, so the match is unambiguous.
    _CHASSIS_TOKEN = "chassis"

    def _padded_link_names(self, scene):
        """Links to inflate by `collision_padding`: every 'ur10e_' collision body
        (moving arm chain + static furniture + the inspected chassis), minus any
        token in _NO_PAD_TOKENS (currently none)."""
        names = list(scene.allowed_collision_matrix.entry_names)
        return [n for n in names
                if n.startswith("ur10e_")
                and not any(tok in n for tok in self._NO_PAD_TOKENS)]

    def _padding_for_link(self, name, base_pad, chassis_pad):
        """Absolute padding for one link: the chassis bodies use `chassis_pad`,
        everything else uses `base_pad`."""
        return chassis_pad if self._CHASSIS_TOKEN in name else base_pad

    def _apply_collision_padding(self):
        """Inflate the collision bodies in the shared scene: `collision_padding`
        for the arm + furniture, `chassis_collision_padding` for the chassis."""
        padding = float(self.get_parameter("collision_padding").value)
        chassis_padding = float(self.get_parameter("chassis_collision_padding").value)
        if chassis_padding <= 0.0:
            # Chassis-specific padding disabled -> uniform base padding.
            chassis_padding = padding
        if padding <= 0.0 and chassis_padding <= 0.0:
            return
        if not self._get_scene_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().warning(
                "get_planning_scene service unavailable; skipping collision padding.")
            return
        req = GetPlanningScene.Request()
        req.components.components = (PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
                                     | PlanningSceneComponents.WORLD_OBJECT_NAMES)
        future = self._get_scene_cli.call_async(req)
        if not self._wait_future(future, 5.0) or future.result() is None:
            self.get_logger().warning(
                "get_planning_scene call failed/timed out; skipping collision padding.")
            return

        arm_links = self._padded_link_names(future.result().scene)
        if not arm_links:
            self.get_logger().warning(
                "No paddable arm links found in the planning scene; skipping collision padding.")
            return

        diff = PlanningScene()
        diff.is_diff = True
        diff.link_padding = [
            LinkPadding(link_name=n,
                        padding=self._padding_for_link(n, padding, chassis_padding))
            for n in arm_links
            if self._padding_for_link(n, padding, chassis_padding) > 0.0
        ]

        if not self._apply_scene_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().warning(
                "apply_planning_scene service unavailable; skipping collision padding.")
            return
        af = self._apply_scene_cli.call_async(ApplyPlanningScene.Request(scene=diff))
        if not self._wait_future(af, 5.0) or af.result() is None:
            self.get_logger().warning(
                "apply_planning_scene call failed/timed out; collision padding may be off.")
            return
        n_chassis = sum(1 for n in arm_links if self._CHASSIS_TOKEN in n)
        n_other = len(arm_links) - n_chassis
        self.get_logger().info(
            f"Collision padding applied: {padding * 100:.1f} cm to {n_other} arm/furniture "
            f"links, {chassis_padding * 100:.1f} cm to {n_chassis} chassis links. "
            f"success={af.result().success}."
        )

    # ------------------------------------------------------------------ #
    def _lookup_sensor_pose(self, frame_id, fallback_frame):
        """world->sensor transform as (translation[3], quaternion[x,y,z,w]) or None."""
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

    def capture_and_save(self, index, fallback_pose=None):
        """Grab a fresh cloud from EVERY active stream at this viewpoint and save
        each into its own tree. Returns True if at least one stream saved."""
        settle = self.get_parameter("capture_settle_sec").value
        # Drop any pre-motion cloud on every stream, then settle ONCE so all
        # streams capture only frames that arrived after the arm stopped.
        for s in self.streams:
            s["latest_cloud"] = None
        time.sleep(settle)

        saved_any = False
        for s in self.streams:
            if self._save_stream(s, index, fallback_pose):
                saved_any = True
        return saved_any

    def _save_stream(self, stream, index, fallback_pose=None):
        """Wait for a fresh cloud on one stream, transform to world, write files."""
        timeout = self.get_parameter("capture_timeout_sec").value
        tag = f"{stream['name']}][{index}"

        deadline = time.monotonic() + timeout
        while stream["latest_cloud"] is None and time.monotonic() < deadline:
            time.sleep(0.1)
        msg = stream["latest_cloud"]
        if msg is None:
            self.get_logger().error(
                f"[{tag}] No point cloud received within {timeout:.1f}s on "
                f"'{stream['topic']}'. Skipping save."
            )
            return False

        pts = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        pts = np.array([[p[0], p[1], p[2]] for p in pts], dtype=np.float64)
        if pts.size == 0:
            self.get_logger().warning(f"[{tag}] Captured cloud is empty; skipping save.")
            return False

        # Resolve the sensor pose (world <- sensor) for both cloud transform and
        # the OctoMap origin file.
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
        # OctoMap sensor origin: "x y z qx qy qz qw" in the world frame.
        with open(txt_path, "w") as f:
            f.write(f"{trans[0]:.6f} {trans[1]:.6f} {trans[2]:.6f} "
                    f"{quat[0]:.6f} {quat[1]:.6f} {quat[2]:.6f} {quat[3]:.6f}\n")

        self.get_logger().info(
            f"[{tag}] Saved {len(pts)} points -> {pcd_path} (frame={cloud_frame}); "
            f"origin -> {txt_path} (from {used_frame})."
        )
        return True

    # ------------------------------------------------------------------ #
    #  Simultaneous dual-arm motion helpers.
    #  MoveIt is used ONLY to plan (collision-aware). Execution is dispatched
    #  straight to each robot's own FollowJointTrajectory controller, so the two
    #  arms run in parallel instead of contending for the single move_group
    #  action server (which serves one goal at a time).
    # ------------------------------------------------------------------ #
    @staticmethod
    def _wait_future(future, timeout):
        """Poll a future without spinning (the executor thread does the spinning)."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if future is not None and future.done():
                return True
            time.sleep(0.05)
        return False

    def _wait_joint_state(self, moveit2, label, timeout=15.0):
        """Wait for a MoveIt2 object's joint-state cache to populate (no spinning).

        moveit2 only caches /joint_states once EVERY one of its joint_names is
        present in the message, so this also validates the object was built with a
        joint set the running system actually publishes."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if moveit2.joint_state is not None:
                return True
            time.sleep(0.1)
        self.get_logger().error(
            f"[{label}] joint states never arrived for joints {moveit2.joint_names}.")
        return False

    @staticmethod
    def _subset(full_names, full_positions, wanted_names):
        """Pick out the positions for `wanted_names` from a whole-robot solution."""
        lut = dict(zip(full_names, full_positions))
        try:
            return [float(lut[n]) for n in wanted_names]
        except KeyError:
            return None

    def _plan(self, moveit2, joint_positions, joint_names, label):
        """Plan (no execution) and return the JointTrajectory, or None.

        Retries the whole plan call up to `plan_attempts` times before giving up.
        Every viewpoint here already passed an IK check at plan time, so a failure
        is the randomized motion planner not finding a collision-free path within
        the budget -- re-issuing the plan (a fresh random seed) usually succeeds on
        one of the next tries. Only after exhausting all attempts do we skip.

        `start_joint_state` is passed explicitly so plan_async never enters its
        "Joint states are not available yet!" gate (which spins on this thread and
        fights the background executor). moveit2.joint_state is guaranteed populated
        by the wait in run()."""
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
                self.get_logger().warning(
                    f"[{label}] planning failed / empty trajectory "
                    f"(attempt {attempt}/{attempts}).")
        self.get_logger().error(
            f"[{label}] planning failed after {attempts} attempts; skipping this viewpoint.")
        return None

    def _dispatch(self, action_client, traj, label):
        """Send a planned trajectory to a controller; return the goal-handle future (async)."""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        return action_client.send_goal_async(goal)

    def _await(self, send_future, label):
        """Wait for goal acceptance + execution result of a dispatched trajectory."""
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
        status = result_future.result().status
        if status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().warning(f"[{label}] motion did not succeed (status={status}).")
            return False
        return True

    # ------------------------------------------------------------------ #
    @staticmethod
    def _order_for_execution(vps):
        """Reorder viewpoints into the execution sweep the operator asked for:
        START at the largest-Y viewpoint (the FRONT of the chassis), then continue
        to the nearest neighbour each step (nearest-neighbour + 2-opt on Euclidean
        position distance). Only the visiting ORDER changes -- the same viewpoints
        are returned. The 2-opt refinement never moves index 0, so the max-Y start
        is always preserved."""
        n = len(vps)
        if n <= 2:
            return list(vps)
        pos = np.array([np.asarray(v["position"], dtype=float) for v in vps])
        dist = np.linalg.norm(pos[:, None, :] - pos[None, :, :], axis=2)

        # Anchor: largest Y = front of the chassis.
        start = int(np.argmax(pos[:, 1]))
        unvisited = set(range(n))
        tour = [start]
        unvisited.discard(start)
        while unvisited:
            last = tour[-1]
            nxt = min(unvisited, key=lambda j: dist[last, j])
            tour.append(nxt)
            unvisited.discard(nxt)

        # 2-opt: un-cross edges of the open path (leaves tour[0] fixed).
        improved = True
        while improved:
            improved = False
            for a in range(n - 1):
                for b in range(a + 1, n - 1):
                    i, j = tour[a], tour[a + 1]
                    k, l = tour[b], tour[b + 1]
                    if dist[i, k] + dist[j, l] + 1e-9 < dist[i, j] + dist[k, l]:
                        tour[a + 1:b + 1] = tour[a + 1:b + 1][::-1]
                        improved = True

        # Or-opt: relocate short chains (length 1-3) to a cheaper spot in the
        # path. 2-opt can only REVERSE a segment, so it leaves the long
        # back-and-forth hops that happen when one viewpoint sits far off the
        # main sweep; Or-opt moves that stray viewpoint next to its true
        # neighbour and shortens the tour further. tour[0] (the max-Y front
        # start) is never relocated. n is small (~15) so the full O(n^2)
        # recompute per candidate is negligible.
        def path_len(t):
            return sum(dist[t[p], t[p + 1]] for p in range(len(t) - 1))

        for seg_len in (1, 2, 3):
            if seg_len >= n:
                break
            improved = True
            while improved:
                improved = False
                base = path_len(tour)
                for a in range(1, n - seg_len + 1):     # never lift the start
                    seg = tour[a:a + seg_len]
                    rest = tour[:a] + tour[a + seg_len:]
                    for b in range(1, len(rest) + 1):   # never insert before start
                        cand = rest[:b] + seg + rest[b:]
                        if path_len(cand) + 1e-9 < base:
                            tour = cand
                            improved = True
                            break
                    if improved:
                        break

        return [vps[t] for t in tour]

    # ------------------------------------------------------------------ #
    def _go_home(self):
        """Send the UR back to its rest/home pose once inspection is finished.

        `home_joint_positions` is in ur_robot.joint_names() order: index 0 is the
        linear rail in METRES (kept as-is); indices 1..6 are the arm joints in
        DEGREES (converted to radians here). Planned collision-aware and
        dispatched to the controller like every viewpoint move, so it never
        drives the arm through the chassis."""
        if not self.get_parameter("return_home").value:
            return
        home = list(self.get_parameter("home_joint_positions").value)
        if len(home) != len(self.ur.joint_names):
            self.get_logger().warning(
                f"home_joint_positions has {len(home)} values but the UR group has "
                f"{len(self.ur.joint_names)} joints; skipping homing.")
            return
        # Rail (index 0) is metres and stays as-is; arm joints (1..6) are degrees.
        home_rad = [float(home[0])] + [float(np.deg2rad(v)) for v in home[1:]]
        self.get_logger().info(
            f"Inspection done -- returning UR10e to home pose "
            f"(rail={home[0]} m, arm={home[1:]} deg)...")
        traj = self._plan(self.ur, home_rad, self.ur.joint_names, "UR home")
        if traj is None:
            self.get_logger().warning(
                "Could not plan a path to the home pose; leaving the arm where it is.")
            return
        send = self._dispatch(self._ur_ctrl, traj, "UR home")
        if self._await(send, "UR home"):
            self.get_logger().info("UR10e returned to home pose.")
        else:
            self.get_logger().warning("Homing motion did not complete successfully.")

    # ------------------------------------------------------------------ #
    def run(self):
        if not os.path.exists(self.plan_file):
            self.get_logger().error(f"Plan file not found: {self.plan_file}.")
            return
        with open(self.plan_file) as f:
            plan = json.load(f)
        ur_vps = plan.get("ur_viewpoints", [])
        if not ur_vps:
            self.get_logger().error("Plan has no UR viewpoints; nothing to execute.")
            return

        # Execution traversal order: start at the front of the chassis (largest Y)
        # then sweep by proximity. Overrides the plan's stored order.
        if self.get_parameter("order_by_proximity").value:
            ur_vps = self._order_for_execution(ur_vps)
            first = ur_vps[0]
            self.get_logger().info(
                f"Execution order: starting at largest-Y viewpoint {first.get('id')} "
                f"(y={float(first['position'][1]):.3f}, chassis front), then nearest-neighbour + 2-opt.")

        # Wait for the controller action server up front.
        if not self._ur_ctrl.wait_for_server(timeout_sec=15.0):
            self.get_logger().error(
                f"UR controller action server '{self.get_parameter('ur_controller_action').value}' "
                "not available; aborting.")
            return

        # Wait (without spinning -- the background executor fills this) for the
        # MoveIt2 object's joint-state cache, so the first plan_async can be handed
        # a valid start_joint_state and never spins on this thread.
        if not self._wait_joint_state(self.ur, "UR"):
            self.get_logger().error("No /joint_states for the UR group; aborting.")
            return

        # move_group is up (we just planned nothing yet, but joint states/controllers
        # are ready) -- register the floor so no plan dips the arm below ground, and
        # inflate the arm links so plans keep a safety margin from every obstacle.
        self._add_ground_plane()
        self._apply_collision_padding()

        self.get_logger().info(f"Executing {len(ur_vps)} UR viewpoints...")
        t_start = time.monotonic()
        saved = 0
        for i, vp in enumerate(ur_vps, start=1):
            if not vp.get("joint_positions"):
                self.get_logger().warning(f"[{i}] Viewpoint {vp.get('id')} has no joint solution; skipping.")
                continue

            # --- Plan the UR motion first (planning does not move the robot). ---
            # The plan stores a whole-robot solution; target only the group's own
            # joints so the goal has no out-of-group constraints.
            ur_pos = self._subset(vp["joint_names"], vp["joint_positions"], self.ur.joint_names)
            if ur_pos is None:
                self.get_logger().warning(f"[{i}] Viewpoint {vp.get('id')} missing a UR joint; skipping.")
                continue
            ur_traj = self._plan(self.ur, ur_pos, self.ur.joint_names, f"UR {vp.get('id')}")
            if ur_traj is None:
                continue

            remaining = len(ur_vps) - i
            self.get_logger().info(
                f"[{i}/{len(ur_vps)}] Moving UR10e to {vp.get('id')} "
                f"({remaining} waypoint{'s' if remaining != 1 else ''} remaining after this).")

            # --- Dispatch to the controller and wait for the motion to finish. ---
            ur_send = self._dispatch(self._ur_ctrl, ur_traj, "UR")
            if not self._await(ur_send, "UR"):
                self.get_logger().warning(f"[{i}] UR motion failed; capturing anyway.")

            # Sensor pose fallback from the planned viewpoint (position + rotation matrix).
            fallback = (np.array(vp["position"], dtype=float),
                        Rotation.from_matrix(np.array(vp["rotation"], dtype=float)).as_quat())
            if self.capture_and_save(i, fallback_pose=fallback):
                saved += 1
                self.get_logger().info(
                    f"[{i}/{len(ur_vps)}] Captured. {remaining} waypoint"
                    f"{'s' if remaining != 1 else ''} remaining.")

        # Inspection sweep finished -- park the arm at its home/rest pose.
        self._go_home()

        trees = " + ".join(s["subtree"] for s in self.streams)
        self.get_logger().info(
            f"Inspection complete [only_sim={self.only_sim}]: {saved}/{len(ur_vps)} viewpoints "
            f"captured into {trees} (each pcds/ + poses/) in {time.monotonic() - t_start:.1f}s.")


def main(args=None):
    rclpy.init(args=args)
    node = InspectionExecutor()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    Thread(target=executor.spin, daemon=True).start()
    time.sleep(3.0)  # let action servers / TF / bridge come up

    try:
        node.run()  # MoveIt action calls on the main thread
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
