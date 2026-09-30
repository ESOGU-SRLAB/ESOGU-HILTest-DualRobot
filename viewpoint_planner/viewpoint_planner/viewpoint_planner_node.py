import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from moveit_msgs.srv import GetPlanningScene, ApplyPlanningScene, GetMotionPlan
from moveit_msgs.msg import (PlanningScene, PlanningSceneComponents,
                             LinkPadding, CollisionObject)
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
import json
import os
import time
import traceback
import numpy as np
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState
from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest, RobotState

from viewpoint_planner.mesh_analyzer import MeshAnalyzer
from viewpoint_planner.viewpoint_generator import ViewpointGenerator
from viewpoint_planner.set_cover_optimizer import SetCoverOptimizer
from viewpoint_planner.reachability_checker import ReachabilityChecker
from viewpoint_planner.viewpoint_clusterer import ViewpointClusterer


class _SceneClient(Node):
    """A node of its own for the planning-scene service calls.

    WHY A SEPARATE NODE: `plan_callback` runs inside this package's service callback
    while `main()` holds the planner node in `rclpy.spin()`. Calling
    `rclpy.spin_until_future_complete(self, ...)` on the PLANNER node from in there
    cannot make progress -- the executor is already spinning it -- so the future never
    completes. That is exactly what used to happen: /get_planning_scene timed out on all
    10 retries, `match_execution_scene` reported "link padding on 0 links", and the
    ground plane + padding were never actually applied. Plan-time IK therefore ran
    against a BARE scene for every plan this package has ever produced, which is the
    real source of the "N viewpoints unreachable at execution" surprise this feature was
    written to prevent. ReachabilityChecker got this right by being its own node, which
    is why IK itself worked.

    Spinning a DIFFERENT node here is safe: nothing else owns it.
    """

    def __init__(self):
        super().__init__('viewpoint_planner_scene_client')
        self.get_cli = self.create_client(GetPlanningScene, '/get_planning_scene')
        self.apply_cli = self.create_client(ApplyPlanningScene, '/apply_planning_scene')

    def call(self, cli, req, timeout=5.0):
        if not cli.service_is_ready():
            return None
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        return fut.result()


class _PlanClient(Node):
    """Own node for the /plan_kinematic_path calls the joint-space ordering makes to
    validate its tour. Same reason as _SceneClient: those calls run inside the planner's
    service callback, where spinning the planner node itself cannot complete a future."""

    UR = ["ur10e_base_to_robot_mount", "ur10e_shoulder_pan_joint",
          "ur10e_shoulder_lift_joint", "ur10e_elbow_joint", "ur10e_wrist_1_joint",
          "ur10e_wrist_2_joint", "ur10e_wrist_3_joint"]

    def __init__(self, group):
        super().__init__('viewpoint_planner_plan_client')
        self.group = group
        self.cli = self.create_client(GetMotionPlan, '/plan_kinematic_path')

    def ready(self, timeout=5.0):
        return self.cli.wait_for_service(timeout_sec=timeout)

    def plan_ok(self, start, goal, budget):
        """True if move_group returns a path from `start` to `goal` (UR joints, rail
        first) through the full pipeline -- including the time parameterisation and the
        final validity check the executor's own plans go through."""
        r = MotionPlanRequest()
        r.group_name = self.group
        r.allowed_planning_time = float(budget)
        r.num_planning_attempts = 10
        r.max_velocity_scaling_factor = r.max_acceleration_scaling_factor = 0.1
        js = JointState()
        js.name = list(self.UR)
        js.position = [float(v) for v in start]
        st = RobotState()
        st.joint_state = js
        st.is_diff = True
        r.start_state = st
        con = Constraints()
        for n, v in zip(self.UR, goal):
            jc = JointConstraint()
            jc.joint_name = n
            jc.position = float(v)
            jc.tolerance_above = jc.tolerance_below = 1e-4
            jc.weight = 1.0
            con.joint_constraints.append(jc)
        r.goal_constraints.append(con)
        req = GetMotionPlan.Request()
        req.motion_plan_request = r
        fut = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=float(budget) * 3 + 30)
        res = fut.result()
        return res is not None and res.motion_plan_response.error_code.val == 1


class ViewpointPlannerNode(Node):
    # Links under the arm prefix to EXCLUDE from plan-time padding. Kept EMPTY to
    # MATCH inspection_executor_node._NO_PAD_TOKENS: excluding the chassis was tried
    # and reverted (its collision mesh under-approximates the part, so the arm hit
    # it). Plan-time IK pads the same links the executor does -- arm + furniture +
    # chassis -- so the two collision scenes stay identical.
    _NO_PAD_TOKENS = ()

    def __init__(self):
        super().__init__('viewpoint_planner_node')

        # --- Mesh / coverage parameters ---
        self.declare_parameter('mesh_path', '')
        self.declare_parameter('mesh_scale', 0.001)
        self.declare_parameter('coverage_threshold', 0.98)
        # 0.0 -> minimize number of viewpoints (pure max-coverage set cover).
        # >0.0 -> also penalize robot travel between viewpoints (may add a few
        # viewpoints but shortens the UR10e path). See SetCoverOptimizer.
        self.declare_parameter('set_cover_travel_weight', 0.0)
        # Hard budget on the number of UR10e viewpoints. <=0 means "no budget"
        # (cover until coverage_threshold). Set e.g. 12 to force "complete the
        # inspection in at most 12 stops" -- greedy picks the most informative
        # ones first, so this yields the best-coverage 12-viewpoint plan.
        self.declare_parameter('max_viewpoints', 0)
        # AUTONOMOUS waypoint count: the greedy set cover stops once the best next
        # viewpoint would cover fewer than this FRACTION of the total target points
        # as NEW surface (the marginal-gain floor). This lets the number of
        # waypoints emerge from the chassis geometry (the knee of the
        # diminishing-returns curve) instead of the hand-picked max_viewpoints cap.
        # 0.0 disables it. See SetCoverOptimizer.min_marginal_coverage.
        self.declare_parameter('min_marginal_coverage', 0.005)
        # Sensing-geometry density knobs. Raise these to lift the GEOMETRIC
        # COVERAGE CEILING on a large/complex mesh (more surface targets and more
        # candidate viewing positions), at the cost of runtime.
        self.declare_parameter('target_sample_points', 5000)
        self.declare_parameter('num_base_points', 250)
        # When True (default) IK rejects poses where the ARM collides with the
        # scene/chassis. Set False to measure pure kinematic reachability -- a
        # diagnostic to tell "arm can't reach" apart from "arm would hit the
        # cage". See ReachabilityChecker.avoid_collisions.
        self.declare_parameter('ik_avoid_collisions', True)
        # Pose clustering (Step 3b): merge selected viewpoints that are within
        # cluster_position_m AND cluster_orientation_deg of each other into one
        # averaged waypoint, provided the merged pose still sees >=
        # cluster_min_coverage_ratio of the cluster's combined targets and is IK
        # reachable (else the cluster keeps its best real member).
        self.declare_parameter('cluster_merge_enabled', False)
        self.declare_parameter('cluster_position_m', 0.25)
        self.declare_parameter('cluster_orientation_deg', 25.0)
        self.declare_parameter('cluster_min_coverage_ratio', 0.9)

        # --- Camera model parameters (SICK Visionary-T Mini V3S145-1AAAAAA) ---
        # Defaults match the datasheet and the Gazebo rgbd_camera sensor in
        # simrobot_ur_macro.xacro. Override via config/sick_tmini_params.yaml.
        self.declare_parameter('camera.horizontal_fov_deg', 70.0)
        self.declare_parameter('camera.vertical_fov_deg', 60.0)
        self.declare_parameter('camera.fov_safety_margin_deg', 3.0)
        self.declare_parameter('camera.min_range_m', 0.2)
        self.declare_parameter('camera.max_range_m', 1.0)
        self.declare_parameter('camera.frame_rate_hz', 5.0)
        self.declare_parameter('camera.viewpoint_distances', [0.3, 0.4, 0.5])
        self.declare_parameter('camera.tilt_variations_deg', [-15.0, 0.0, 15.0])
        self.declare_parameter('camera.max_incidence_angle_deg', 75.0)

        # --- UR10e (camera-carrying arm) reachability parameters ---
        self.declare_parameter('ur_group_name', 'real_ur10e')
        self.declare_parameter('ur_base_frame', 'world')
        self.declare_parameter('ur_tool_frame', 'ur10e_depth_optical_frame')

        # --- Plan/execution scene matching ---
        # The plan-time IK reachability check (avoid_collisions=True) checks the
        # arm against move_group's CURRENT planning scene. Execution
        # (inspection_executor_node) first adds a ground-plane box and inflates
        # the arm links by `collision_padding`, THEN motion-plans to each stored
        # joint solution. If we DON'T replicate that scene here, IK happily
        # accepts poses whose arm dips under the floor or grazes the chassis --
        # they enter the plan, then fail motion planning at execution (the
        # "N viewpoints unreachable" surprise). These defaults MUST match the
        # inspection_execution launch defaults so the two scenes are identical.
        self.declare_parameter('match_execution_scene', True)
        self.declare_parameter('add_ground_plane', True)
        self.declare_parameter('ground_plane_z', -0.02)
        self.declare_parameter('ground_plane_size', 6.0)
        self.declare_parameter('ground_plane_thickness', 0.2)
        # 4 cm on every ur10e_* link, the multirobot cell value.
        self.declare_parameter('collision_padding', 0.04)
        # OPTIONAL separate margin for the chassis links, mirroring the executor's
        # `chassis_collision_padding`. 0.0 -> the chassis gets collision_padding like
        # every other link, which is what the multirobot cell does. Whatever it is set
        # to, planner and executor must agree, or plan-time IK and execution see
        # different worlds. (Default was 0.10 until 2026-09-10.)
        self.declare_parameter('chassis_collision_padding', 0.0)
        # Substring identifying the inspected chassis bodies among the padded links.
        self.declare_parameter('chassis_link_token', 'chassis')
        # Prefix identifying the MOVING arm links to pad (matches executor).
        self.declare_parameter('arm_link_prefix', 'ur10e_')

        # Reorder the selected viewpoints into a short cartesian visiting path
        # (nearest-neighbour + 2-opt) so the arm sweeps neighbouring stops instead
        # of jumping across the chassis in greedy coverage-discovery order. Changes
        # ONLY the visiting order, never which viewpoints are kept. False -> keep
        # the greedy selection order.
        self.declare_parameter('order_by_proximity', True)
        # WHERE the sweep starts:
        #   'max_y'    -> the largest-Y viewpoint (the FRONT of the chassis),
        #   'min_y'    -> the smallest-Y viewpoint (the BACK),
        #   'coverage' -> index 0, the highest-coverage viewpoint.
        # max_y is the default because that is the sweep the operator asked for, and
        # because the executor used to impose it by re-ordering the plan at run time --
        # ordering now lives HERE alone, so the plan JSON is what actually gets driven.
        self.declare_parameter('order_anchor', 'max_y')
        # HOW the tour is built once the anchor is chosen:
        #   'y_bands'   -> monotone sweep: every viewpoint in the first Y band, then the
        #                  next, and so on; the arm never returns to a band it has left.
        #                  Inside a band the stops still take the shortest path.
        #   'proximity' -> the globally shortest 3D path. It is free to walk back up the
        #                  chassis whenever that is cheaper, and it does: measured on the
        #                  two-arm cell, 9 of 20 hops moved against the sweep direction
        #                  and the tour climbed 2.28 m of Y backwards in total.
        #   'joint'     -> (default for this package) order by how far the ARM has to move,
        #                  not by where the camera is. See _order_by_joint_space.
        self.declare_parameter('order_mode', 'joint')
        # --- 'joint' ordering ---
        # Random arm postures per viewpoint used to collect its alternative IK branches.
        self.declare_parameter('order_ik_seeds', 10)
        self.declare_parameter('order_ik_timeout', 0.1)
        # What a metre of rail travel costs, in radians of joint travel. Same value the
        # multirobot planner uses for its branch re-chaining.
        self.declare_parameter('order_rail_weight', 2.0)
        # Where the arm is when the sequence starts: MUST equal the executor's
        # `start_pose` (index 0 rail in METRES, 1..6 arm joints in DEGREES).
        self.declare_parameter('order_start_pose', [1.0, 0.0, -90.0, 0.0, -90.0, 0.0, 0.0])
        # Validate every hop of the tour with a real motion plan and repair the ones that
        # fail, so the plan file only ever holds a chain the executor can drive.
        self.declare_parameter('order_check_feasibility', True)
        self.declare_parameter('order_plan_time', 3.0)          # s per validation call
        # How many (viewpoint, branch) candidates to try, cheapest first, before widening
        # the search at one step of the tour.
        self.declare_parameter('order_max_tries_per_step', 8)
        self.declare_parameter('order_feasibility_budget_s', 240.0)
        # Depth of one Y band, metres. Measured on the two-arm plans: at 0.30 the worst
        # backward step falls from 0.76 m to 0.22 m for ~13% more path length. Smaller
        # approaches a strict sort by Y (perfectly monotone, ~35% longer); larger drifts
        # back towards 'proximity'.
        self.declare_parameter('order_band_width_m', 0.30)

        # WORKSPACE KEEP-OUT BOX (world frame). The occlusion ray-trace only knows
        # the chassis mesh, so it cannot tell that a candidate camera pose sitting
        # BEHIND the linear rail (or any cell structure) is physically blocked --
        # the robot then drives there and captures 0 chassis points. This box culls
        # such candidates up front: a candidate is kept ONLY if its camera position
        # lies inside [min, max] on every axis. Set the faces that face open space
        # very wide (e.g. +-100) and clip only the side(s) you want to forbid (e.g.
        # set y-min just in front of the rail so no viewpoint is generated behind
        # it). Disabled by default; the planner logs the chassis AABB at plan time
        # so you can pick sensible numbers. Values are metres in the 'world' frame.
        # Where the plan is written. Was hardcoded, which made it impossible to keep two
        # plans side by side (and the trajectory cache lives next to this file).
        self.declare_parameter(
            'output_plan_file',
            '/home/cem/colcon_ws/src/viewpoint_planner/plans/viewpoint_plan.json')

        self.declare_parameter('workspace_filter_enabled', False)
        self.declare_parameter('workspace_bounds_min', [-100.0, -100.0, -100.0])
        self.declare_parameter('workspace_bounds_max', [100.0, 100.0, 100.0])

        self.mesh_path = self.get_parameter('mesh_path').value
        self.mesh_scale = self.get_parameter('mesh_scale').value
        self.coverage_threshold = self.get_parameter('coverage_threshold').value

        self.camera_config = {
            'horizontal_fov_deg': self.get_parameter('camera.horizontal_fov_deg').value,
            'vertical_fov_deg': self.get_parameter('camera.vertical_fov_deg').value,
            'fov_safety_margin_deg': self.get_parameter('camera.fov_safety_margin_deg').value,
            'min_range_m': self.get_parameter('camera.min_range_m').value,
            'max_range_m': self.get_parameter('camera.max_range_m').value,
            'frame_rate_hz': self.get_parameter('camera.frame_rate_hz').value,
            'viewpoint_distances': list(self.get_parameter('camera.viewpoint_distances').value),
            'tilt_variations_deg': list(self.get_parameter('camera.tilt_variations_deg').value),
            'max_incidence_angle_deg': self.get_parameter('camera.max_incidence_angle_deg').value,
        }

        self.plan_srv = self.create_service(Trigger, '~/plan', self.plan_callback)

        self.get_logger().info(
            f"Viewpoint Planner Node initialized. mesh_path='{self.mesh_path}', "
            f"coverage_threshold={self.coverage_threshold}, camera_config={self.camera_config}."
        )

    def _match_execution_scene(self):
        """Replicate the executor's planning scene (ground-plane box + arm link
        padding) into move_group BEFORE the IK reachability filter runs.

        This is the fix for the "many viewpoints unreachable at execution"
        problem: plan-time /compute_ik(avoid_collisions=True) and execution-time
        motion planning must see the SAME collision world, otherwise IK accepts
        poses whose arm body enters the floor or the chassis padding and the plan
        silently fills with viewpoints that can never be executed. Applied as a
        single PlanningScene diff via /apply_planning_scene, exactly like
        inspection_executor_node does. Best-effort: warns and continues (measuring
        only kinematic+base reachability) if the services are unavailable.

        Returns a stamp describing what was actually applied. It is written into the
        plan file so a plan produced against a bare scene can never be mistaken for a
        verified one months later."""
        stamp = {'applied': False, 'reason': 'disabled'}
        if not self.get_parameter('match_execution_scene').value:
            self.get_logger().warning(
                "match_execution_scene=False: plan-time IK uses move_group's bare scene. "
                "Selected viewpoints may fail motion planning at execution (floor/padding).")
            return stamp

        base_frame = self.get_parameter('ur_base_frame').value
        # Its own node -- see _SceneClient for why this cannot be done on `self`.
        scene_cli = _SceneClient()
        try:
            return self._match_execution_scene_impl(scene_cli, base_frame, stamp)
        finally:
            scene_cli.destroy_node()

    def _match_execution_scene_impl(self, scene_cli, base_frame, stamp):
        diff = PlanningScene()
        diff.is_diff = True

        # --- Ground-plane collision box (world frame, thin, top face at z). ---
        if self.get_parameter('add_ground_plane').value:
            top_z = float(self.get_parameter('ground_plane_z').value)
            size = float(self.get_parameter('ground_plane_size').value)
            thickness = float(self.get_parameter('ground_plane_thickness').value)
            box = SolidPrimitive()
            box.type = SolidPrimitive.BOX
            box.dimensions = [size, size, thickness]
            pose = Pose()
            pose.position.x = 0.0
            pose.position.y = 0.0
            pose.position.z = top_z - thickness / 2.0  # centre half a thickness below top face
            pose.orientation.w = 1.0
            co = CollisionObject()
            co.header.frame_id = base_frame
            co.id = "ground_plane"
            co.primitives = [box]
            co.primitive_poses = [pose]
            co.operation = CollisionObject.ADD
            diff.world.collision_objects = [co]

        # --- Arm/furniture link padding (from the live ACM, chassis excluded). ---
        padding = float(self.get_parameter('collision_padding').value)
        prefix = self.get_parameter('arm_link_prefix').value
        if padding > 0.0:
            arm_links = []
            if scene_cli.get_cli.wait_for_service(timeout_sec=5.0):
                # move_group's PlanningSceneMonitor can take a moment after startup
                # to publish a fully-populated ACM; an early query returns an EMPTY
                # entry_names list -- the "link padding on 0 'ur10e_' links" bug that
                # let plan-time IK run without padding while the executor applied it,
                # so viewpoints passed planning then failed motion planning. Retry
                # until the ACM is populated (same components the executor requests).
                for _ in range(10):
                    req = GetPlanningScene.Request()
                    req.components.components = (
                        PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
                        | PlanningSceneComponents.WORLD_OBJECT_NAMES)
                    res = scene_cli.call(req=req, cli=scene_cli.get_cli)
                    if res is not None and res.scene.allowed_collision_matrix.entry_names:
                        arm_links = [
                            n for n in res.scene.allowed_collision_matrix.entry_names
                            if n.startswith(prefix)
                            and not any(t in n for t in self._NO_PAD_TOKENS)]
                        break
                    time.sleep(0.5)
            if arm_links:
                chassis_pad = float(self.get_parameter('chassis_collision_padding').value)
                token = str(self.get_parameter('chassis_link_token').value)
                if chassis_pad <= 0.0:
                    chassis_pad = padding

                def _pad_for(name):
                    return chassis_pad if token in name else padding

                diff.link_padding = [LinkPadding(link_name=n, padding=_pad_for(n))
                                     for n in arm_links if _pad_for(n) > 0.0]
            else:
                self.get_logger().warning(
                    "Could not read paddable links from /get_planning_scene after retries; "
                    "plan-time IK will run WITHOUT link padding (execution uses "
                    f"{padding} m -- some selected viewpoints may fail at execution).")

        if not diff.world.collision_objects and not diff.link_padding:
            stamp['reason'] = 'nothing to apply (no ground plane, no paddable links)'
            return stamp

        if not scene_cli.apply_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().warning(
                "/apply_planning_scene unavailable; plan-time IK scene NOT matched to execution "
                "(selected viewpoints may fail motion planning at execution).")
            stamp['reason'] = 'apply_planning_scene unavailable'
            return stamp
        applied = scene_cli.call(req=ApplyPlanningScene.Request(scene=diff),
                                 cli=scene_cli.apply_cli)
        if applied is None or not applied.success:
            self.get_logger().error(
                "apply_planning_scene did not confirm success; plan-time IK is running "
                "against a scene that does NOT match execution.")
            stamp['reason'] = 'apply_planning_scene did not succeed'
            return stamp
        # Give move_group's PlanningSceneMonitor a moment to ingest the diff
        # before the first /compute_ik query hits it.
        time.sleep(0.5)
        n_pad = len(diff.link_padding)
        has_floor = bool(diff.world.collision_objects)
        chassis_pad = float(self.get_parameter('chassis_collision_padding').value) or padding
        token = str(self.get_parameter('chassis_link_token').value)
        n_chassis = sum(1 for lp in diff.link_padding if token in lp.link_name)
        stamp = {
            'applied': True,
            'ground_plane': has_floor,
            'ground_plane_z': float(self.get_parameter('ground_plane_z').value),
            'ground_plane_size': float(self.get_parameter('ground_plane_size').value),
            'ground_plane_thickness': float(self.get_parameter('ground_plane_thickness').value),
            'collision_padding': padding,
            'chassis_collision_padding': chassis_pad,
            'arm_link_prefix': prefix,
            'padded_links': n_pad,
            'padded_chassis_links': n_chassis,
        }
        self.get_logger().info(
            f"Plan-time scene matched to execution: ground_plane={'on' if has_floor else 'off'}, "
            f"padding {padding} m on {n_pad - n_chassis} '{prefix}' link(s) and "
            f"{chassis_pad} m on {n_chassis} chassis link(s). "
            "IK now rejects poses the executor could not reach.")
        return stamp

    def _make_reachability_fn(self, checker):
        """Builds a candidate -> (reachable, joint_solution) callable for the
        optimizer's reachability gate. Returns None (geometric-only selection,
        UNVERIFIED) if the IK service never comes up, logged loudly so a plan of
        unreachable poses is never mistaken for a verified one."""
        if not checker.wait_for_service():
            self.get_logger().warning(
                f"/compute_ik unavailable for group '{checker.group_name}'. Running set-cover "
                "WITHOUT reachability verification -- selected viewpoints are geometric only and "
                "may NOT be reachable at execution time."
            )
            return None

        def _fn(candidate):
            quat = Rotation.from_matrix(candidate['rotation']).as_quat()  # [x, y, z, w]
            return checker.check_ik(candidate['position'], quat)

        return _fn

    def _apply_workspace_filter(self, candidates):
        """Drop candidates whose camera position falls outside the world-frame
        keep-out box (workspace_bounds_min/max). No-op when disabled. This removes
        physically-blocked poses (e.g. behind the linear rail) that the mesh-only
        occlusion check cannot see, before they waste coverage-matrix / IK work."""
        if not self.get_parameter('workspace_filter_enabled').value:
            return candidates
        lo = np.asarray(self.get_parameter('workspace_bounds_min').value, dtype=float)
        hi = np.asarray(self.get_parameter('workspace_bounds_max').value, dtype=float)
        if lo.shape != (3,) or hi.shape != (3,):
            self.get_logger().warning(
                "workspace_bounds_min/max must each have 3 values [x,y,z]; skipping workspace filter.")
            return candidates
        kept = [c for c in candidates
                if np.all(np.asarray(c['position'], dtype=float) >= lo)
                and np.all(np.asarray(c['position'], dtype=float) <= hi)]
        removed = len(candidates) - len(kept)
        self.get_logger().info(
            f"Workspace keep-out box: kept {len(kept)}/{len(candidates)} candidates "
            f"(removed {removed} outside min={lo.tolist()} max={hi.tolist()} in 'world')."
        )
        if not kept:
            self.get_logger().error(
                "Workspace filter removed ALL candidates -- the box is too tight or misplaced. "
                "Check workspace_bounds against the logged chassis AABB.")
        return kept

    def plan_callback(self, request, response):
        self.get_logger().info('Planning requested.')
        t_start = time.monotonic()

        if not self.mesh_path or not os.path.exists(self.mesh_path):
            self.get_logger().error(f"Mesh path invalid or not set: '{self.mesh_path}'.")
            response.success = False
            response.message = f"Mesh path invalid: {self.mesh_path}"
            return response

        # ReachabilityChecker is a full rclpy Node. Track it so we can destroy it
        # in the finally block below -- otherwise every ~/plan call leaks a node
        # and re-registers a duplicate node name on the next call.
        ur_checker = None

        try:
            # 1. Mesh Analysis
            self.get_logger().info('Step 1: Mesh Analysis')
            t0 = time.monotonic()
            analyzer = MeshAnalyzer(self.mesh_path, scale=self.mesh_scale, logger=self.get_logger())
            analyzer.load_mesh()
            target_points, target_normals, _ = analyzer.sample_surface(
                num_points=self.get_parameter('target_sample_points').value)
            self.get_logger().info(f"Step 1 done in {time.monotonic() - t0:.2f}s.")

            # 2. Viewpoint Generation
            self.get_logger().info('Step 2: Viewpoint Generation')
            t0 = time.monotonic()
            vg = ViewpointGenerator(analyzer, config=self.camera_config, logger=self.get_logger())
            candidates = vg.generate_candidates(
                target_points, target_normals,
                num_base_points=self.get_parameter('num_base_points').value)
            # Log the chassis AABB (world frame) so the operator can pick sensible
            # workspace_bounds to clip poses behind the rail / cell structures.
            aabb_min, aabb_max = analyzer.mesh.bounds
            self.get_logger().info(
                f"Chassis AABB (world, m): min={aabb_min.tolist()} max={aabb_max.tolist()}. "
                "Use these to set workspace_bounds_min/max for the keep-out box.")
            # Cull physically-blocked candidates (e.g. behind the linear rail) that
            # the mesh-only occlusion test cannot detect, before the expensive steps.
            candidates = self._apply_workspace_filter(candidates)
            coverage_matrix = vg.compute_coverage_matrix(candidates, target_points, target_normals)
            self.get_logger().info(f"Step 2 done in {time.monotonic() - t0:.2f}s.")

            # 3. Reachability-aware Set Cover Optimization (UR10e)
            #
            # Reachability is folded INTO the greedy selection instead of being a
            # separate post-filter. Previously we ran set cover over all
            # candidates and then dropped the unreachable ones -- which destroyed
            # the carefully minimized set and made the reported coverage a lie
            # (the surviving viewpoints no longer achieved it). Now only
            # robot-reachable viewpoints can be selected, so the final coverage
            # is what the robot can actually deliver, using the fewest stops.
            self.get_logger().info('Step 3: Reachability-aware Set Cover Optimization (UR10e)')
            t0 = time.monotonic()

            avoid_collisions = self.get_parameter('ik_avoid_collisions').value
            # Stamped into the plan file: what the reachability sweep was verified
            # against. Both early exits below leave it saying 'not applied', which is
            # exactly what a reader of the plan needs to know.
            ik_scene = {'applied': False, 'reason': 'not attempted'}
            if not avoid_collisions:
                self.get_logger().warning(
                    "ik_avoid_collisions=False: measuring PURE KINEMATIC reachability. Selected "
                    "viewpoints may put the arm in collision at execution time -- diagnostic use only."
                )
                ik_scene['reason'] = 'ik_avoid_collisions=False'
            elif self.get_parameter('match_execution_scene').value:
                # Load the floor + arm padding into move_group so the IK
                # collision check below matches what the executor will face.
                ik_scene = self._match_execution_scene()
            ur_checker = ReachabilityChecker(
                group_name=self.get_parameter('ur_group_name').value,
                base_frame=self.get_parameter('ur_base_frame').value,
                tool_frame=self.get_parameter('ur_tool_frame').value,
                avoid_collisions=avoid_collisions,
            )
            reachability_fn = self._make_reachability_fn(ur_checker)

            max_vp = self.get_parameter('max_viewpoints').value
            max_vp = max_vp if max_vp and max_vp > 0 else None

            optimizer = SetCoverOptimizer(
                coverage_threshold=self.coverage_threshold,
                travel_weight=self.get_parameter('set_cover_travel_weight').value,
                logger=self.get_logger(),
                min_marginal_coverage=self.get_parameter('min_marginal_coverage').value,
            )
            selected_candidates, selected_indices, final_coverage = optimizer.optimize(
                candidates, coverage_matrix,
                max_viewpoints=max_vp,
                reachability_fn=reachability_fn,
            )
            ur_checker.log_error_breakdown()
            self.get_logger().info(
                f"Step 3 done in {time.monotonic() - t0:.2f}s. "
                f"{len(selected_candidates)} reachable UR10e viewpoints selected."
            )

            # 3b. Optional pose clustering: merge viewpoints that are close in
            # position AND viewing direction into a single averaged waypoint,
            # re-verifying coverage + IK for each merged pose. Cuts the count of
            # near-duplicate stops without silently losing coverage.
            if self.get_parameter('cluster_merge_enabled').value:
                self.get_logger().info('Step 3b: Pose Clustering (merge nearby/similar viewpoints)')
                t0 = time.monotonic()
                clusterer = ViewpointClusterer(
                    position_thresh=self.get_parameter('cluster_position_m').value,
                    orientation_thresh_deg=self.get_parameter('cluster_orientation_deg').value,
                    min_coverage_ratio=self.get_parameter('cluster_min_coverage_ratio').value,
                    logger=self.get_logger(),
                )
                selected_masks = [coverage_matrix[idx] for idx in selected_indices]
                intersector = vg.make_intersector()
                selected_candidates, merged_masks = clusterer.cluster_and_merge(
                    selected_candidates, selected_masks, vg,
                    target_points, target_normals, intersector, reachability_fn,
                )
                # Recompute final coverage and the per-viewpoint contribution
                # metadata (rank/new_points_covered) for the merged set so the
                # plan file and visualizer stay consistent.
                num_t = coverage_matrix.shape[1]
                covered = np.zeros(num_t, dtype=bool)
                for i, (vp, mask) in enumerate(zip(selected_candidates, merged_masks)):
                    new_pts = int(np.sum(mask & ~covered))
                    covered |= mask
                    vp['rank'] = i
                    vp['new_points_covered'] = new_pts
                    vp['total_points_visible'] = int(mask.sum())
                    vp['cumulative_coverage'] = float(covered.sum() / num_t)
                final_coverage = float(covered.sum() / num_t)
                self.get_logger().info(
                    f"Step 3b done in {time.monotonic() - t0:.2f}s. "
                    f"{len(selected_candidates)} viewpoints after clustering, coverage {final_coverage * 100:.1f}%."
                )

            # Assign ids in informativeness order (rank 0 = most informative) so
            # vp_000 stays the highest-coverage stop regardless of visiting order.
            for i, vp in enumerate(selected_candidates):
                vp['id'] = f'vp_{i:03d}'

            # 4. Route optimization: reorder the selected viewpoints into a short
            # cartesian visiting path so the arm sweeps neighbouring stops instead
            # of jumping in greedy coverage-discovery order. Selection is unchanged
            # (same viewpoints, same 'rank'/'new_points_covered' informativeness);
            # only the LIST order the executor visits in changes.
            if self.get_parameter('order_by_proximity').value:
                anchor = str(self.get_parameter('order_anchor').value)
                mode = str(self.get_parameter('order_mode').value)
                width = float(self.get_parameter('order_band_width_m').value)
                self._order_skipped = []
                if mode == 'joint' and reachability_fn is not None:
                    selected_candidates, how = self._order_by_joint_space(
                        selected_candidates, ur_checker)
                    anchor = 'start pose'
                elif mode == 'y_bands' or mode == 'joint':
                    if mode == 'joint':
                        self.get_logger().warning(
                            "order_mode='joint' needs /compute_ik, which is unavailable; "
                            "falling back to 'y_bands'.")
                    selected_candidates = self._order_by_y_bands(
                        selected_candidates, anchor, width)
                    how = (f"monotone Y sweep in {width:.2f} m bands "
                           "(shortest path inside each band)")
                else:
                    selected_candidates = self._order_by_proximity(
                        selected_candidates, anchor=anchor)
                    how = "shortest cartesian path (nearest-neighbour + 2-opt + Or-opt)"
                self.get_logger().info(
                    f"Viewpoints reordered by {how}, starting at '{anchor}'. "
                    "This IS the order the executor drives -- it no longer re-orders.")

            # Reachable viewpoints already carry their IK joint solution from the
            # optimizer's reachability gate (when the IK service was available).
            reachable_vps = selected_candidates

            # 5. Save Plan
            plan = {
                "coverage_achieved": float(final_coverage),
                "coverage_threshold": float(self.coverage_threshold),
                "total_viewpoints": len(reachable_vps),
                "camera_config": self.camera_config,
                # What the reachability sweep was verified against. applied=False means
                # the viewpoints below were NOT checked against the executor's floor and
                # padding, so some may be unreachable at execution time.
                "ik_scene": ik_scene,
                # Viewpoints the joint-space ordering could not connect to the validated
                # chain from anywhere; they are NOT in ur_viewpoints, and
                # coverage_achieved above still counts them, so it overstates.
                "skipped_viewpoints": list(getattr(self, '_order_skipped', [])),
                "ur_viewpoints": [],
            }

            for vp in reachable_vps:
                vp_dict = {
                    "id": vp['id'],
                    "position": vp['position'].tolist(),
                    "rotation": vp['rotation'].tolist(),
                    # Informativeness metadata from the greedy optimizer: used by
                    # the visualizers to rank/colour viewpoints by how much new
                    # coverage each one adds (rank 0 = most informative).
                    "rank": vp.get('rank'),
                    "new_points_covered": vp.get('new_points_covered'),
                    "total_points_visible": vp.get('total_points_visible'),
                    "cumulative_coverage": vp.get('cumulative_coverage'),
                }
                # If we have joint solution from IK, save it. JointState.name is
                # a list[str] but .position is an array.array (from the ROS
                # message) which json cannot serialize -- cast both to plain
                # lists.
                if 'joint_solution' in vp and vp['joint_solution']:
                    vp_dict['joint_names'] = list(vp['joint_solution'].name)
                    vp_dict['joint_positions'] = list(vp['joint_solution'].position)
                plan["ur_viewpoints"].append(vp_dict)

            plan_file = os.path.expanduser(
                self.get_parameter('output_plan_file').value)
            os.makedirs(os.path.dirname(plan_file), exist_ok=True)
            with open(plan_file, 'w') as f:
                json.dump(plan, f, indent=2)

            total_time = time.monotonic() - t_start
            if final_coverage < self.coverage_threshold:
                self.get_logger().warning(
                    f"Planning finished in {total_time:.2f}s but coverage target was NOT met: "
                    f"{final_coverage * 100:.1f}% achieved vs {self.coverage_threshold * 100:.1f}% requested. "
                    f"See earlier warnings for likely cause (occlusion/FOV/reachability)."
                )
            else:
                self.get_logger().info(
                    f"Planning finished successfully in {total_time:.2f}s. "
                    f"Coverage: {final_coverage * 100:.1f}%. {len(reachable_vps)} UR10e viewpoints. "
                    f"Saved to {plan_file}."
                )

            response.success = True
            response.message = (
                f"Plan created successfully. Coverage: {final_coverage*100:.1f}%. "
                f"UR viewpoints: {len(reachable_vps)}. Saved to {plan_file}."
            )

        except Exception as e:
            self.get_logger().error(f"Planning failed after {time.monotonic() - t_start:.2f}s: {e}")
            self.get_logger().error(traceback.format_exc())
            response.success = False
            response.message = f"Error: {e}"

        finally:
            # Always tear down the checker node we created for this request so
            # repeated ~/plan calls don't leak nodes or clash on node names.
            if ur_checker is not None:
                try:
                    ur_checker.destroy_node()
                except Exception as e:
                    self.get_logger().warning(f"Failed to destroy reachability checker node: {e}")

        return response

    # UR group joints, rail first -- the order every distance below is computed in.
    _UR_JOINTS = ["ur10e_base_to_robot_mount", "ur10e_shoulder_pan_joint",
                  "ur10e_shoulder_lift_joint", "ur10e_elbow_joint", "ur10e_wrist_1_joint",
                  "ur10e_wrist_2_joint", "ur10e_wrist_3_joint"]
    # Joints the executor can unwind (a second in-limits equivalent exists). The rail is
    # prismatic and the UR10e ELBOW is limited to +/-pi, so neither ever wraps: their
    # travel is the plain difference. Matches the executor's own "5/7 of this arm's
    # joints can be unwound".
    _UR_WRAPPABLE = [False, True, True, False, True, True, True]
    _UR_WRAP_LIMIT = 2.0 * np.pi - 0.05     # limit of the wrappable joints, minus margin

    @staticmethod
    def _wrap_pi(x):
        return (float(x) + np.pi) % (2.0 * np.pi) - np.pi

    def _canonical(self, q):
        """Wrappable joints into [-pi, pi]; rail and elbow as they are."""
        return [float(q[0])] + [self._wrap_pi(v) if w else float(v)
                                for v, w in zip(q[1:], self._UR_WRAPPABLE[1:])]

    def _step_goal(self, a, b):
        """Configuration the arm reaches going from `a` to branch `b` the SHORT way --
        exactly what the executor's goal unwinding does: rail and elbow move by the plain
        difference, every other joint by the wrapped one, flipped the other way round
        only where the short way would leave the joint limits."""
        out = [float(b[0])]
        for x, y, w in zip(a[1:], b[1:], self._UR_WRAPPABLE[1:]):
            if not w:
                out.append(float(y))
                continue
            g = float(x) + self._wrap_pi(float(y) - float(x))
            if g > self._UR_WRAP_LIMIT:
                g -= 2.0 * np.pi
            elif g < -self._UR_WRAP_LIMIT:
                g += 2.0 * np.pi
            out.append(g)
        return out

    def _joint_dist(self, a, b, rail_weight):
        """Arm travel between two UR configurations the short way (see _step_goal), the
        rail charged at `rail_weight` radians per metre."""
        d = abs(float(a[0]) - float(b[0])) * rail_weight
        for x, y, w in zip(a[1:], b[1:], self._UR_WRAPPABLE[1:]):
            d += abs(self._wrap_pi(float(y) - float(x))) if w else abs(float(y) - float(x))
        return d

    def _order_by_joint_space(self, vps, checker):
        """Order the viewpoints by how far the ARM moves between them, pick each one's IK
        branch so the tour stays short, and VALIDATE the resulting chain hop by hop.

        WHY: the cartesian orders put two viewpoints next to each other because their
        CAMERAS are close, which says nothing about the arm. Measured on the 2026-09-10
        single-UR plans: vp_019 -> vp_016 were 0.58 m apart as cameras but 546 deg apart
        as arm configurations and had no path between them. Ordering by joint distance
        alone was not enough either -- the executor then picked its own nearest branch at
        every stop, drifted off the chain the planner had optimised, and met 537 / 578 /
        757 deg reconfigurations it could not plan. So the chain is now fixed HERE, proven
        drivable here, and the executor simply follows it (use_pose_goal:=false,
        nearest_branch_ik:=false).

        HOW:
          1. Collect each viewpoint's IK branches (stored solution + `order_ik_seeds`
             random postures, collision-aware against the matched scene).
          2. Starting at the executor's START pose, repeatedly rank every (unvisited
             viewpoint, branch) by the arm travel to it and PLAN the cheapest ones with
             move_group (`order_plan_time` each); the first that plans is the next stop.
             Each hop is therefore proven when it is added -- the transition cost AND
             its feasibility decide where the arm goes next.
          3. Store the validated, continuously unwound configurations as the viewpoints'
             joint solutions -- the exact values the executor will be sent.

        Returns (ordered_vps, description)."""
        UR = self._UR_JOINTS
        n = len(vps)
        if n <= 1:
            return list(vps), "joint-space order (nothing to order)"
        n_seeds = max(0, int(self.get_parameter('order_ik_seeds').value))
        ik_timeout = float(self.get_parameter('order_ik_timeout').value)
        w = float(self.get_parameter('order_rail_weight').value)
        sp = list(self.get_parameter('order_start_pose').value)
        start = [float(sp[0])] + [float(np.deg2rad(v)) for v in sp[1:7]]
        check = bool(self.get_parameter('order_check_feasibility').value)
        plan_time = float(self.get_parameter('order_plan_time').value)
        budget_s = float(self.get_parameter('order_feasibility_budget_s').value)
        rng = np.random.default_rng(0)
        t0 = time.monotonic()

        # --- 1. branches per viewpoint (canonical form) ---
        branches = []
        for vp in vps:
            sol = vp.get('joint_solution')
            lut = dict(zip(sol.name, sol.position)) if sol is not None else {}
            stored = ([float(lut[j]) for j in UR] if all(j in lut for j in UR) else None)
            found = [self._canonical(stored)] if stored is not None else []
            quat = Rotation.from_matrix(np.asarray(vp['rotation'], dtype=float)).as_quat()
            rail = stored[0] if stored is not None else start[0]
            for _ in range(n_seeds):
                seed = JointState()
                seed.name = list(UR)
                seed.position = [rail] + [float(v) for v in rng.uniform(-np.pi, np.pi, 6)]
                ok, js = checker.check_ik(vp['position'], quat, timeout=ik_timeout,
                                          seed_joint_state=seed, count_errors=False)
                if not ok or js is None:
                    continue
                q = dict(zip(js.name, js.position))
                if not all(j in q for j in UR):
                    continue
                cand = self._canonical([float(q[j]) for j in UR])
                if not any(self._joint_dist(cand, f, w) < 0.02 for f in found):
                    found.append(cand)
            branches.append(found if found else [self._canonical(start)])
        t_branch = time.monotonic() - t0

        # --- 2. build the tour one VALIDATED hop at a time ---
        # From wherever the arm is, rank every (unvisited viewpoint, branch) by the arm
        # travel to it and plan the cheapest ones in turn; the first that plans becomes
        # the next stop. Every hop is proven when it is added, so there is nothing to
        # repair afterwards. (A first version built the whole tour and then repaired
        # the failing hops: on the 2026-09-10 plan a quarter of candidate hops failed,
        # every repair reshuffled the tour, and it never converged within 240 s.)
        max_tries = max(1, int(self.get_parameter('order_max_tries_per_step').value))
        plan_cli = None
        if check:
            plan_cli = _PlanClient(self.get_parameter('ur_group_name').value)
            if not plan_cli.ready():
                self.get_logger().warning(
                    "/plan_kinematic_path unavailable -- the joint-space tour will NOT be "
                    "validated hop by hop.")
                plan_cli.destroy_node()
                plan_cli = None

        tour, chain, stragglers, skipped = [], [], [], []
        unvisited = set(range(n))
        cur = list(start)
        plan_calls = failed_tries = unproven = 0
        while unvisited:
            cands = sorted((self._joint_dist(cur, b, w), j, k)
                           for j in unvisited for k, b in enumerate(branches[j]))
            chosen = None
            for rank, (_, j, k) in enumerate(cands):
                goal = self._step_goal(cur, branches[j][k])
                if plan_cli is None:
                    chosen = (j, goal)
                    unproven += 1 if check else 0
                    break
                if time.monotonic() - t0 > budget_s:
                    chosen = (j, goal)
                    unproven += 1
                    break
                if rank == max_tries:
                    self.get_logger().info(
                        f"Joint-space order: none of the {max_tries} cheapest next stops "
                        f"plans from here ({len(tour)} placed); widening to all "
                        f"{len(cands)} candidates.")
                plan_calls += 1
                if plan_cli.plan_ok(cur, goal, plan_time):
                    chosen = (j, goal)
                    break
                failed_tries += 1
            if chosen is None:
                # Nothing left plans from this configuration: set the cheapest one aside
                # and try to insert it elsewhere in the chain afterwards.
                _, j, k = cands[0]
                stragglers.append(j)
                self.get_logger().warning(
                    f"Joint-space order: no remaining viewpoint plans from after "
                    f"{vps[tour[-1]]['id'] if tour else 'the start pose'}; will try to "
                    f"insert {vps[j]['id']} elsewhere in the chain.")
                unvisited.discard(j)
                continue
            j, goal = chosen
            tour.append(j)
            chain.append(goal)
            unvisited.discard(j)
            cur = goal
        # --- stragglers: insert where BOTH the way in and the way out plan ---
        for j in stragglers:
            placed = False
            if plan_cli is not None:
                # cheapest insertion points first (added arm travel)
                spots = []
                for pos in range(len(tour) + 1):
                    prev_q = start if pos == 0 else chain[pos - 1]
                    for k, b in enumerate(branches[j]):
                        g = self._step_goal(prev_q, b)
                        add = self._joint_dist(prev_q, g, w)
                        if pos < len(tour):
                            add += self._joint_dist(g, chain[pos], w)
                        spots.append((add, pos, k))
                spots.sort()
                for add, pos, k in spots[:3 * max_tries]:
                    if time.monotonic() - t0 > budget_s:
                        break
                    prev_q = start if pos == 0 else chain[pos - 1]
                    g = self._step_goal(prev_q, branches[j][k])
                    plan_calls += 1
                    if not plan_cli.plan_ok(prev_q, g, plan_time):
                        failed_tries += 1
                        continue
                    if pos < len(tour):
                        nxt = self._step_goal(g, self._canonical(chain[pos]))
                        plan_calls += 1
                        if not plan_cli.plan_ok(g, nxt, plan_time):
                            failed_tries += 1
                            continue
                    # insert, then re-express the rest of the chain continuously from here
                    tour.insert(pos, j)
                    chain.insert(pos, g)
                    for q in range(pos + 1, len(chain)):
                        chain[q] = self._step_goal(chain[q - 1], self._canonical(chain[q]))
                    placed = True
                    self.get_logger().info(
                        f"Joint-space order: {vps[j]['id']} inserted at stop {pos + 1} "
                        "with both hops planned.")
                    break
            if not placed:
                skipped.append(vps[j]['id'])
                self.get_logger().error(
                    f"Joint-space order: {vps[j]['id']} cannot be reached from ANY point of "
                    "the validated chain -- left OUT of the tour (listed as "
                    "skipped_viewpoints in the plan) instead of making the executor burn "
                    "minutes on it.")
        if plan_cli is not None:
            plan_cli.destroy_node()

        # --- 5. write the validated configurations back ---
        ordered, hops, prev_q = [], [], list(start)
        for k, i in enumerate(tour):
            vp = dict(vps[i])
            q = chain[k]
            sol = vp.get('joint_solution')
            if sol is not None:
                lut = dict(zip(sol.name, sol.position))
                lut.update({j: v for j, v in zip(UR, q)})
                js = JointState()
                js.name = list(sol.name)
                js.position = [float(lut[nm]) for nm in sol.name]
                vp['joint_solution'] = js
            hops.append(self._joint_dist(prev_q, q, w))
            prev_q = q
            ordered.append(vp)

        n_br = [len(b) for b in branches]
        verdict = ("every hop PLANNED OK" if check and unproven == 0
                   else f"{unproven} hop(s) NOT validated" if check else "not validated")
        self.get_logger().info(
            f"Joint-space order: {n} viewpoints, {min(n_br)}-{max(n_br)} IK branches each "
            f"({t_branch:.1f}s). Arm travel {np.rad2deg(sum(hops)):.0f} deg, largest single "
            f"hop {np.rad2deg(max(hops)):.0f} deg. Validation: {plan_calls} plan calls, "
            f"{failed_tries} candidate hop(s) rejected, {len(skipped)} viewpoint(s) "
            f"unreachable and left out -- {verdict} "
            f"({time.monotonic() - t0:.1f}s total).")
        self._order_skipped = skipped
        return ordered, "arm joint-space travel (branch-aware, validated hop by hop)"

    @staticmethod
    def _order_by_proximity(vps, anchor='max_y'):
        """Reorder viewpoints into the SHORTEST cartesian visiting path, ignoring any
        sweep direction. Only the ORDER changes -- the exact same set of viewpoints (and
        their rank / contribution metadata) is returned.

        anchor selects the START viewpoint:
          'coverage' -> index 0 (the highest-coverage viewpoint),
          'max_y'    -> the largest-Y viewpoint (the FRONT of the chassis),
          'min_y'    -> the smallest-Y viewpoint (the BACK).

        Beware what this optimises: a pure shortest-path tour is free to walk back up
        the chassis whenever that is cheaper in 3D, and it does. Use 'y_bands' for a
        monotone sweep instead.
        """
        n = len(vps)
        if n <= 2:
            return list(vps)
        pos = np.array([np.asarray(v['position'], dtype=np.float64) for v in vps])
        dist = np.linalg.norm(pos[:, None, :] - pos[None, :, :], axis=2)
        if anchor == 'max_y':
            start = int(np.argmax(pos[:, 1]))
        elif anchor == 'min_y':
            start = int(np.argmin(pos[:, 1]))
        else:
            start = 0
        return [vps[t] for t in ViewpointPlannerNode._short_path(dist, start)]

    @staticmethod
    def _order_by_y_bands(vps, anchor='max_y', band_width=0.30):
        """Monotone sweep along Y: finish one Y band before moving to the next.

        The arm starts at the anchor end of the chassis, visits every viewpoint whose Y
        falls in the first `band_width` metres, then the next band, and so on -- it never
        returns to a band it has left. Inside a band the stops are ordered by the same
        shortest-path routine as 'proximity', starting from whichever viewpoint is
        closest to where the previous band ended, so the local motion stays short while
        the global progress stays one-directional.

        band_width is the whole trade-off: at 0 it degenerates into a strict sort by Y
        (shortest possible sweep in Y, longest in X/Z), and at more than the chassis
        length it degenerates into plain 'proximity'. It should be about the depth of the
        region the camera covers from one Y position.

        anchor gives the direction: 'max_y' sweeps front-to-back (descending Y), 'min_y'
        back-to-front. Any other value has no direction, so the caller should use
        'proximity' instead.
        """
        n = len(vps)
        if n <= 2:
            return list(vps)
        pos = np.array([np.asarray(v['position'], dtype=np.float64) for v in vps])
        y = pos[:, 1]
        descending = (anchor != 'min_y')
        width = max(float(band_width), 1e-6)
        # Band 0 is always the band the sweep STARTS in, whichever way it runs.
        edge = y.max() if descending else y.min()
        band = np.floor(np.abs(y - edge) / width).astype(int)

        order, prev = [], None
        for b in sorted(set(band.tolist())):
            idx = [i for i in range(n) if band[i] == b]
            if len(idx) == 1:
                order.append(idx[0])
                prev = pos[idx[0]]
                continue
            sub = pos[idx]
            d = np.linalg.norm(sub[:, None, :] - sub[None, :, :], axis=2)
            if prev is None:
                # Very first stop: the extreme-Y viewpoint, so the sweep really does
                # begin at the end of the chassis the anchor names.
                start = int(np.argmax(sub[:, 1]) if descending else np.argmin(sub[:, 1]))
            else:
                start = int(np.argmin(np.linalg.norm(sub - prev, axis=1)))
            tour = ViewpointPlannerNode._short_path(d, start)
            order.extend(idx[k] for k in tour)
            prev = sub[tour[-1]]
        return [vps[i] for i in order]

    @staticmethod
    def _short_path(dist, start):
        """Open shortest-path order over `dist`, pinned at `start`: nearest-neighbour
        seed, then 2-opt + Or-opt refinement (neither ever moves index 0 of the tour).
        Returns a list of indices into `dist`. n is small, so this is instant."""
        n = dist.shape[0]
        if n <= 2:
            return list(range(n)) if start == 0 else [start] + [i for i in range(n) if i != start]

        # Nearest-neighbour seed tour starting from the anchor.
        unvisited = set(range(n))
        tour = [start]
        unvisited.discard(start)
        while unvisited:
            last = tour[-1]
            nxt = min(unvisited, key=lambda j: dist[last, j])
            tour.append(nxt)
            unvisited.discard(nxt)

        # 2-opt: repeatedly un-cross edges of the open path until no swap helps.
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

        # Or-opt: relocate short chains (length 1-3) to a cheaper spot. 2-opt can only
        # REVERSE a segment, so it leaves the long back-and-forth hops that happen when
        # one viewpoint sits far off the main sweep; Or-opt moves that stray viewpoint
        # next to its true neighbour and shortens the path further.
        def path_len(t):
            return sum(dist[t[p], t[p + 1]] for p in range(len(t) - 1))

        for seg_len in (1, 2, 3):
            if seg_len >= n:
                break
            improved = True
            while improved:
                improved = False
                base = path_len(tour)
                for a in range(1, n - seg_len + 1):     # never lift the anchor
                    seg = tour[a:a + seg_len]
                    rest = tour[:a] + tour[a + seg_len:]
                    for b in range(1, len(rest) + 1):   # never insert before anchor
                        cand = rest[:b] + seg + rest[b:]
                        if path_len(cand) + 1e-9 < base:
                            tour = cand
                            improved = True
                            break
                    if improved:
                        break

        return tour


def main(args=None):
    rclpy.init(args=args)
    node = ViewpointPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
