import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from moveit_msgs.srv import GetPlanningScene, ApplyPlanningScene
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

from viewpoint_planner.mesh_analyzer import MeshAnalyzer
from viewpoint_planner.viewpoint_generator import ViewpointGenerator
from viewpoint_planner.set_cover_optimizer import SetCoverOptimizer
from viewpoint_planner.reachability_checker import ReachabilityChecker
from viewpoint_planner.viewpoint_clusterer import ViewpointClusterer


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
        self.declare_parameter('collision_padding', 0.03)
        # Prefix identifying the MOVING arm links to pad (matches executor).
        self.declare_parameter('arm_link_prefix', 'ur10e_')

        # Reorder the selected viewpoints into a short cartesian visiting path
        # (nearest-neighbour + 2-opt) so the arm sweeps neighbouring stops instead
        # of jumping across the chassis in greedy coverage-discovery order. Changes
        # ONLY the visiting order, never which viewpoints are kept. False -> keep
        # the greedy selection order.
        self.declare_parameter('order_by_proximity', True)

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
        only kinematic+base reachability) if the services are unavailable."""
        if not self.get_parameter('match_execution_scene').value:
            self.get_logger().warning(
                "match_execution_scene=False: plan-time IK uses move_group's bare scene. "
                "Selected viewpoints may fail motion planning at execution (floor/padding).")
            return

        base_frame = self.get_parameter('ur_base_frame').value
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
            get_cli = self.create_client(GetPlanningScene, '/get_planning_scene')
            arm_links = []
            if get_cli.wait_for_service(timeout_sec=5.0):
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
                    fut = get_cli.call_async(req)
                    rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
                    res = fut.result()
                    if res is not None and res.scene.allowed_collision_matrix.entry_names:
                        arm_links = [
                            n for n in res.scene.allowed_collision_matrix.entry_names
                            if n.startswith(prefix)
                            and not any(t in n for t in self._NO_PAD_TOKENS)]
                        break
                    time.sleep(0.5)
            self.destroy_client(get_cli)
            if arm_links:
                diff.link_padding = [LinkPadding(link_name=n, padding=padding) for n in arm_links]
            else:
                self.get_logger().warning(
                    "Could not read paddable links from /get_planning_scene after retries; "
                    "plan-time IK will run WITHOUT link padding (execution uses "
                    f"{padding} m -- some selected viewpoints may fail at execution).")

        if not diff.world.collision_objects and not diff.link_padding:
            return

        apply_cli = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
        if not apply_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().warning(
                "/apply_planning_scene unavailable; plan-time IK scene NOT matched to execution "
                "(selected viewpoints may fail motion planning at execution).")
            self.destroy_client(apply_cli)
            return
        fut = apply_cli.call_async(ApplyPlanningScene.Request(scene=diff))
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        self.destroy_client(apply_cli)
        # Give move_group's PlanningSceneMonitor a moment to ingest the diff
        # before the first /compute_ik query hits it.
        time.sleep(0.5)
        n_pad = len(diff.link_padding)
        has_floor = bool(diff.world.collision_objects)
        self.get_logger().info(
            f"Plan-time scene matched to execution: ground_plane={'on' if has_floor else 'off'}, "
            f"link padding {padding} m on {n_pad} '{prefix}' link(s). "
            "IK now rejects poses the executor could not reach.")

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
            if not avoid_collisions:
                self.get_logger().warning(
                    "ik_avoid_collisions=False: measuring PURE KINEMATIC reachability. Selected "
                    "viewpoints may put the arm in collision at execution time -- diagnostic use only."
                )
            elif self.get_parameter('match_execution_scene').value:
                # Load the floor + arm padding into move_group so the IK
                # collision check below matches what the executor will face.
                self._match_execution_scene()
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
                selected_candidates = self._order_by_proximity(selected_candidates)
                self.get_logger().info(
                    "Viewpoints reordered by cartesian proximity "
                    "(nearest-neighbour + 2-opt) to shorten arm travel.")

            # Reachable viewpoints already carry their IK joint solution from the
            # optimizer's reachability gate (when the IK service was available).
            reachable_vps = selected_candidates

            # 5. Save Plan
            plan = {
                "coverage_achieved": float(final_coverage),
                "coverage_threshold": float(self.coverage_threshold),
                "total_viewpoints": len(reachable_vps),
                "camera_config": self.camera_config,
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

            plan_file = '/home/ifarlab/colcon_ws/src/viewpoint_planner/plans/viewpoint_plan.json'
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

    @staticmethod
    def _order_by_proximity(vps):
        """Reorder viewpoints into a short cartesian visiting path so the arm
        sweeps neighbouring stops instead of criss-crossing the chassis. Only the
        ORDER changes -- the exact same set of viewpoints (and their rank /
        contribution metadata) is returned.

        Nearest-neighbour tour (anchored at the highest-coverage viewpoint, index
        0 after informativeness ordering) followed by 2-opt refinement. Distance =
        Euclidean distance between viewpoint positions (a proxy for arm travel).
        With the handful of stops in a plan this is effectively instant.
        """
        n = len(vps)
        if n <= 2:
            return list(vps)
        pos = np.array([np.asarray(v['position'], dtype=np.float64) for v in vps])
        dist = np.linalg.norm(pos[:, None, :] - pos[None, :, :], axis=2)

        # Nearest-neighbour seed tour starting from index 0 (highest coverage).
        unvisited = set(range(n))
        tour = [0]
        unvisited.discard(0)
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

        return [vps[t] for t in tour]


def main(args=None):
    rclpy.init(args=args)
    node = ViewpointPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
