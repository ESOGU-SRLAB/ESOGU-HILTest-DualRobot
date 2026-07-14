#!/usr/bin/env python3
"""
Multi-robot inspection executor.

Both the UR10e and the Kawasaki now carry a SICK camera and visit their OWN
allocated coverage viewpoints (see multirobot_planner_node). For each execution
step it:
  1. plans BOTH arms (collision-aware, via the real MoveIt) to their step-i
     viewpoint using the stored IK joint solutions, then dispatches both straight
     to their own controllers so they move SIMULTANEOUSLY;
  2. once both arrive, grabs the latest SICK cloud from EACH active camera,
     transforms it into the world frame, and writes per robot:
        <base>/<sim|real>_pcds/<ur|kawasaki>_data/pcds/<N>.pcd   -- the cloud
        <base>/<sim|real>_pcds/<ur|kawasaki>_data/poses/<N>.txt  -- OctoMap origin
                                 "x y z qx qy qz qw" (world frame)

Streams captured:
  sim_ur       : /sim/pointcloud            -> sim_pcds/ur_data
  sim_kawasaki : /sim/kawasaki/pointcloud   -> sim_pcds/kawasaki_data
  real_ur      : /sick_points               -> real_pcds/ur_data       (only_sim=false)
  real_kawasaki: /kawasaki/pointcloud       -> real_pcds/kawasaki_data (only_sim=false)

The sim streams always run; the real streams are added only when connected to the
physical cell (only_sim=false).
"""
import json
import os
import re
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

# Joints of the Kawasaki planning group ("real_kawasaki"). The plan stores a
# whole-robot solution per viewpoint; the MoveIt2 object must be built with ONLY
# these joints (its joint-state cache is dropped unless every one of its
# joint_names appears in /joint_states).
KAWASAKI_JOINT_NAMES = [
    "world_to_agv", "joint1", "joint2", "joint3", "joint4", "joint5", "joint6",
]


class MultiRobotExecutor(Node):
    def __init__(self):
        super().__init__("multirobot_executor_node")

        self.declare_parameter("plan_file", '/home/cem/colcon_ws/src/multirobot_viewpoint_planner/plans/multirobot_viewpoint_plan.json')
        # only_sim gates whether the REAL SICKs are captured alongside the sim ones:
        #   True  (default) -> not connected to hardware: capture ONLY the two
        #                      Gazebo SICKs (ur + kawasaki). Real streams are never
        #                      subscribed.
        #   False           -> connected to the real cell: capture BOTH sim AND real
        #                      SICK for each arm at every viewpoint.
        self.declare_parameter("only_sim", True)
        # Root of the capture tree: <base>/<sim|real>_pcds/<ur|kawasaki>_data/{pcds,poses}.
        self.declare_parameter("output_base_dir", os.path.expanduser("~/colcon_ws/src/pcds"))

        # Point cloud topics, per stream.
        self.declare_parameter("sim_ur_topic", "/sim/pointcloud")
        self.declare_parameter("sim_kawasaki_topic", "/sim/kawasaki/pointcloud")
        self.declare_parameter("real_ur_topic", "/sick_points")
        self.declare_parameter("real_kawasaki_topic", "/kawasaki/pointcloud")

        self.declare_parameter("world_frame", "world")
        # Fallback sensor frames per stream (if the cloud has no usable frame_id).
        self.declare_parameter("sim_ur_sensor_frame", "sim_ur10e_depth_optical_frame")
        self.declare_parameter("sim_kawasaki_sensor_frame", "sim_kawasaki_camera_rgb_optic_frame")
        self.declare_parameter("real_ur_sensor_frame", "ur10e_depth_optical_frame")
        self.declare_parameter("real_kawasaki_sensor_frame", "camera_rgb_optic_frame")

        self.declare_parameter("capture_settle_sec", 1.5)
        self.declare_parameter("capture_timeout_sec", 6.0)
        self.declare_parameter("transform_to_world", True)

        self.declare_parameter("ur_velocity", 0.1)
        self.declare_parameter("ur_acceleration", 0.1)
        self.declare_parameter("enable_kawasaki", True)
        self.declare_parameter("kawasaki_velocity", 0.015)
        self.declare_parameter("kawasaki_acceleration", 0.015)
        # Controllers are driven directly (bypassing the single move_group action
        # server) so the two arms move simultaneously.
        self.declare_parameter(
            "ur_controller_action", "/scaled_joint_trajectory_controller/follow_joint_trajectory")
        self.declare_parameter(
            "kawasaki_controller_action", "/kawasaki/kawasaki_controller/follow_joint_trajectory")

        self.declare_parameter("plan_timeout_sec", 30.0)
        self.declare_parameter("motion_timeout_sec", 120.0)
        # Motion-planning effort (each viewpoint already passed IK at plan time, so
        # a plan failure is the randomized planner not finding a path in time --
        # give it room and retry). See the solo executor for the rationale.
        self.declare_parameter("allowed_planning_time", 5.0)
        self.declare_parameter("num_planning_attempts", 10)
        self.declare_parameter("plan_attempts", 4)
        # Floor collision box so MoveIt never plans an arm below ground.
        self.declare_parameter("add_ground_plane", True)
        self.declare_parameter("ground_plane_z", -0.02)
        self.declare_parameter("ground_plane_size", 6.0)
        self.declare_parameter("ground_plane_thickness", 0.2)
        # Safety margin the moving arm links keep from every obstacle (m).
        self.declare_parameter("collision_padding", 0.04)

        self.plan_file = self.get_parameter("plan_file").value
        self.world_frame = self.get_parameter("world_frame").value

        # --- Build the capture streams (robot-tagged). ---
        self.only_sim = bool(self.get_parameter("only_sim").value)
        base = os.path.expanduser(self.get_parameter("output_base_dir").value)
        self.streams = [
            self._make_stream(
                "sim_ur", "ur",
                self.get_parameter("sim_ur_topic").value,
                self.get_parameter("sim_ur_sensor_frame").value,
                os.path.join(base, "sim_pcds", "ur_data")),
            self._make_stream(
                "sim_kawasaki", "kawasaki",
                self.get_parameter("sim_kawasaki_topic").value,
                self.get_parameter("sim_kawasaki_sensor_frame").value,
                os.path.join(base, "sim_pcds", "kawasaki_data")),
        ]
        if not self.only_sim:
            self.streams.append(self._make_stream(
                "real_ur", "ur",
                self.get_parameter("real_ur_topic").value,
                self.get_parameter("real_ur_sensor_frame").value,
                os.path.join(base, "real_pcds", "ur_data")))
            self.streams.append(self._make_stream(
                "real_kawasaki", "kawasaki",
                self.get_parameter("real_kawasaki_topic").value,
                self.get_parameter("real_kawasaki_sensor_frame").value,
                os.path.join(base, "real_pcds", "kawasaki_data")))

        cb = ReentrantCallbackGroup()
        self._cb = cb
        self._plan_timeout = self.get_parameter("plan_timeout_sec").value
        self._motion_timeout = self.get_parameter("motion_timeout_sec").value

        # Planning-scene service clients (collision padding).
        self._get_scene_cli = self.create_client(
            GetPlanningScene, "get_planning_scene", callback_group=cb)
        self._apply_scene_cli = self.create_client(
            ApplyPlanningScene, "apply_planning_scene", callback_group=cb)

        # --- UR10e MoveIt2 (rail + 6 arm joints), PLANNING ONLY ---
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
        self._ur_ctrl = ActionClient(
            self, FollowJointTrajectory,
            self.get_parameter("ur_controller_action").value, callback_group=cb)

        # --- Kawasaki MoveIt2 (now a full camera-carrying arm) -- set up in run(). ---
        self.kawa = None
        self._kawa_ctrl = None

        # --- Point cloud subscriptions, one per stream. ---
        for s in self.streams:
            s["sub"] = self.create_subscription(
                PointCloud2, s["topic"],
                self._make_cloud_cb(s), qos_profile_sensor_data, callback_group=cb,
            )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)

        streams_desc = ", ".join(f"{s['name']}('{s['topic']}' -> {s['subtree']})" for s in self.streams)
        self.get_logger().info(
            f"MultiRobotExecutor ready [only_sim={self.only_sim}]. plan='{self.plan_file}'. "
            f"Capture streams: {streams_desc}."
        )

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

    # ------------------------------------------------------------------ #
    def _setup_kawasaki(self, plan):
        """Build the Kawasaki interface. It now visits its own coverage viewpoints."""
        if not self.get_parameter("enable_kawasaki").value:
            return
        vp = next((v for v in plan.get("kawasaki_viewpoints", []) if v.get("joint_names")), None)
        if vp is None:
            self.get_logger().warning(
                "Kawasaki enabled but the plan has no kawasaki viewpoints with joint solutions; "
                "Kawasaki will stay idle.")
            return
        self.kawa = MoveIt2Real(
            node=self,
            joint_names=list(KAWASAKI_JOINT_NAMES),
            base_link_name=self.world_frame,
            end_effector_name="link6",
            group_name="real_kawasaki",
            callback_group=self._cb,
        )
        self.kawa.max_velocity = self.get_parameter("kawasaki_velocity").value
        self.kawa.max_acceleration = self.get_parameter("kawasaki_acceleration").value
        self.kawa.allowed_planning_time = self.get_parameter("allowed_planning_time").value
        self.kawa.num_planning_attempts = self.get_parameter("num_planning_attempts").value
        self._kawa_ctrl = ActionClient(
            self, FollowJointTrajectory,
            self.get_parameter("kawasaki_controller_action").value, callback_group=self._cb)
        self.get_logger().info("Kawasaki camera arm enabled (visits its own coverage viewpoints).")

    # ------------------------------------------------------------------ #
    def _add_ground_plane(self):
        if not self.get_parameter("add_ground_plane").value:
            return
        top_z = self.get_parameter("ground_plane_z").value
        size = self.get_parameter("ground_plane_size").value
        thickness = self.get_parameter("ground_plane_thickness").value
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
            f"Ground plane added: {size}x{size}x{thickness} m box, top face at z={top_z:.3f} "
            f"in '{self.world_frame}'.")

    def _arm_link_names(self, scene):
        """Pick the MOVING arm links (UR 'ur10e_*' + Kawasaki 'link1'..'link6')."""
        names = list(scene.allowed_collision_matrix.entry_names)
        kawa = re.compile(r"^link[1-6]$")
        return [n for n in names if n.startswith("ur10e_") or kawa.match(n)]

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
        """Grab a fresh cloud from every active stream whose robot moved this step
        and save each into its robot's own tree. Returns True if at least one saved."""
        settle = self.get_parameter("capture_settle_sec").value
        for s in self.streams:
            s["latest_cloud"] = None
        time.sleep(settle)

        saved_any = False
        for s in self.streams:
            if s["robot"] not in active_robots:
                continue
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
    @staticmethod
    def _wait_future(future, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if future is not None and future.done():
                return True
            time.sleep(0.05)
        return False

    def _wait_joint_state(self, moveit2, label, timeout=15.0):
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
        lut = dict(zip(full_names, full_positions))
        try:
            return [float(lut[n]) for n in wanted_names]
        except KeyError:
            return None

    def _plan(self, moveit2, joint_positions, joint_names, label):
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
                    f"[{label}] planning failed / empty trajectory (attempt {attempt}/{attempts}).")
        self.get_logger().error(
            f"[{label}] planning failed after {attempts} attempts; skipping this viewpoint.")
        return None

    def _dispatch(self, action_client, traj, label):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        return action_client.send_goal_async(goal)

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
        status = result_future.result().status
        if status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().warning(f"[{label}] motion did not succeed (status={status}).")
            return False
        return True

    @staticmethod
    def _fallback_pose(vp):
        return (np.array(vp["position"], dtype=float),
                Rotation.from_matrix(np.array(vp["rotation"], dtype=float)).as_quat())

    def _plan_arm(self, moveit2, vp, label):
        """Subset the whole-robot solution to the group's joints and plan. Returns
        the trajectory or None (missing joint / plan failure)."""
        pos = self._subset(vp["joint_names"], vp["joint_positions"], moveit2.joint_names)
        if pos is None:
            self.get_logger().warning(f"[{label}] viewpoint {vp.get('id')} missing a group joint; skipping.")
            return None
        return self._plan(moveit2, pos, moveit2.joint_names, f"{label} {vp.get('id')}")

    # ------------------------------------------------------------------ #
    def run(self):
        if not os.path.exists(self.plan_file):
            self.get_logger().error(f"Plan file not found: {self.plan_file}.")
            return
        with open(self.plan_file) as f:
            plan = json.load(f)
        ur_vps = plan.get("ur_viewpoints", [])
        kawa_vps = plan.get("kawasaki_viewpoints", [])
        if not ur_vps and not kawa_vps:
            self.get_logger().error("Plan has no viewpoints for either arm; nothing to execute.")
            return

        self._setup_kawasaki(plan)

        if not self._ur_ctrl.wait_for_server(timeout_sec=15.0):
            self.get_logger().error(
                f"UR controller '{self.get_parameter('ur_controller_action').value}' not available; "
                "aborting.")
            return
        if self._kawa_ctrl is not None and not self._kawa_ctrl.wait_for_server(timeout_sec=15.0):
            self.get_logger().warning(
                f"Kawasaki controller '{self.get_parameter('kawasaki_controller_action').value}' "
                "not available; continuing with UR only.")
            self._kawa_ctrl = None

        if not self._wait_joint_state(self.ur, "UR"):
            self.get_logger().error("No /joint_states for the UR group; aborting.")
            return
        if self._kawa_ctrl is not None and not self._wait_joint_state(self.kawa, "Kawasaki"):
            self.get_logger().warning("No /joint_states for the Kawasaki group; continuing with UR only.")
            self._kawa_ctrl = None

        self._add_ground_plane()
        self._apply_collision_padding()

        kawa_active = self._kawa_ctrl is not None
        n_steps = max(len(ur_vps), len(kawa_vps) if kawa_active else 0)
        self.get_logger().info(
            f"Executing {n_steps} steps (UR: {len(ur_vps)} viewpoints, "
            f"Kawasaki: {len(kawa_vps) if kawa_active else 0} viewpoints, simultaneous).")

        t_start = time.monotonic()
        saved = 0
        for i in range(1, n_steps + 1):
            ur_vp = ur_vps[i - 1] if i <= len(ur_vps) else None
            kawa_vp = kawa_vps[i - 1] if (kawa_active and i <= len(kawa_vps)) else None

            # --- Plan BOTH arms first (planning does not move the robot). ---
            ur_traj = None
            if ur_vp is not None and ur_vp.get("joint_positions"):
                ur_traj = self._plan_arm(self.ur, ur_vp, "UR")
            kawa_traj = None
            if kawa_vp is not None and kawa_vp.get("joint_positions"):
                kawa_traj = self._plan_arm(self.kawa, kawa_vp, "Kawasaki")

            if ur_traj is None and kawa_traj is None:
                self.get_logger().warning(f"[{i}/{n_steps}] Neither arm could be planned; skipping step.")
                continue

            self.get_logger().info(
                f"[{i}/{n_steps}] Moving"
                + (f" UR->{ur_vp.get('id')}" if ur_traj is not None else "")
                + (f" Kawasaki->{kawa_vp.get('id')}" if kawa_traj is not None else "")
                + " (simultaneous).")

            # --- Dispatch BOTH, THEN wait -- parallel execution. ---
            ur_send = self._dispatch(self._ur_ctrl, ur_traj, "UR") if ur_traj is not None else None
            kawa_send = self._dispatch(self._kawa_ctrl, kawa_traj, "Kawasaki") \
                if kawa_traj is not None else None

            ur_ok = self._await(ur_send, "UR") if ur_send is not None else False
            kawa_ok = self._await(kawa_send, "Kawasaki") if kawa_send is not None else False
            if ur_send is not None and not ur_ok:
                self.get_logger().warning(f"[{i}] UR motion failed; capturing anyway.")
            if kawa_send is not None and not kawa_ok:
                self.get_logger().warning(f"[{i}] Kawasaki motion failed; capturing anyway.")

            # Capture only the arms that actually moved this step, each into its
            # own <robot>_data tree with THIS step's index.
            active_robots = set()
            fallback_by_robot = {}
            if ur_traj is not None:
                active_robots.add("ur")
                fallback_by_robot["ur"] = self._fallback_pose(ur_vp)
            if kawa_traj is not None:
                active_robots.add("kawasaki")
                fallback_by_robot["kawasaki"] = self._fallback_pose(kawa_vp)

            if self.capture_and_save(i, active_robots, fallback_by_robot):
                saved += 1

        trees = " + ".join(s["subtree"] for s in self.streams)
        self.get_logger().info(
            f"Multi-robot inspection complete [only_sim={self.only_sim}]: {saved}/{n_steps} steps "
            f"captured into {trees} (each pcds/ + poses/) in {time.monotonic() - t_start:.1f}s.")


def main(args=None):
    rclpy.init(args=args)
    node = MultiRobotExecutor()

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
