import json
import os
import time
import traceback

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from std_msgs.msg import String
from std_srvs.srv import Trigger
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState

# Reuse the single-arm viewpoint_planner core verbatim (this package depends on
# it). Only the ALLOCATION across the two arms is new -- everything up to and
# including the shared candidate pool + coverage matrix is identical to the solo
# baseline, which keeps the solo-vs-multi comparison honest.
from viewpoint_planner.mesh_analyzer import MeshAnalyzer
from viewpoint_planner.viewpoint_generator import ViewpointGenerator
from viewpoint_planner.reachability_checker import ReachabilityChecker
from viewpoint_planner.joint_wrap import (describe_changes, parse_joint_limits,
                                          wrap_to_reference)

from multirobot_viewpoint_planner.robot_allocator import RobotAllocator


class MultiRobotPlannerNode(Node):
    """Plans a shared coverage set and allocates it across the UR10e and the
    Kawasaki (both now carry a SICK camera). Writes
    multirobot_viewpoint_planner/plans/multirobot_viewpoint_plan.json
    with separate `ur_viewpoints` and `kawasaki_viewpoints` lists, each a real
    coverage viewpoint (NOT a mirror point) with its own IK joint solution."""

    def __init__(self):
        super().__init__('multirobot_planner_node')

        # --- Mesh / coverage parameters (same knobs as the solo planner) ---
        self.declare_parameter('mesh_path', '')
        self.declare_parameter('mesh_scale', 0.001)
        self.declare_parameter('coverage_threshold', 0.98)
        self.declare_parameter('target_sample_points', 5000)
        self.declare_parameter('num_base_points', 400)
        self.declare_parameter('ik_avoid_collisions', True)
        # Per-ARM budget (0 = no budget, cover until threshold). With two arms
        # running in parallel the wall-clock cost is ~max(per-arm counts), so this
        # caps "each arm makes at most N stops".
        self.declare_parameter('max_viewpoints_per_robot', 0)
        # PER-ARM overrides of the budget above (0 = no override, use the shared one).
        # They exist because the arms are not equally fast: the wall-clock cost of a
        # run is set by the slower arm, so capping only that arm is what shortens it.
        # Both default to 0, so a config that does not mention them behaves exactly as
        # before -- the chassis job is deliberately left on the shared cap.
        self.declare_parameter('max_viewpoints_ur', 0)
        self.declare_parameter('max_viewpoints_kawasaki', 0)
        # True -> a candidate reachable by both arms goes to the less-loaded arm
        # (shorter parallel run). False -> fixed arm priority order, UR first, so the
        # Kawasaki only receives candidates the UR cannot reach.
        self.declare_parameter('balance_load', True)
        # Tail cutter: stop adding viewpoints once the best remaining one would add
        # fewer than this many NEW target points. The coverage curve has a sharp
        # knee, then a long tail of viewpoints that each add only 1-3 points. Raise
        # this to cut the tail and get far fewer waypoints (small coverage cost).
        self.declare_parameter('min_new_points', 1)
        # Fractional tail cutter (mirrors the solo viewpoint_planner). Stop once the
        # best remaining viewpoint adds fewer than this FRACTION of all target points
        # as new coverage -- scales with mesh density (0.005 = 0.5% ~= 25 new points
        # on a 5000-target mesh). The effective floor is max(min_new_points, this).
        # 0 disables it. This is the "cut at 0.5%" knob the solo baseline uses.
        self.declare_parameter('min_marginal_coverage', 0.005)

        # --- Camera model (shared: both arms carry the same SICK) ---
        self.declare_parameter('camera.horizontal_fov_deg', 70.0)
        self.declare_parameter('camera.vertical_fov_deg', 60.0)
        self.declare_parameter('camera.fov_safety_margin_deg', 3.0)
        self.declare_parameter('camera.min_range_m', 0.2)
        self.declare_parameter('camera.max_range_m', 1.8)
        self.declare_parameter('camera.frame_rate_hz', 5.0)
        self.declare_parameter('camera.viewpoint_distances', [0.3, 0.6, 1.0, 1.4])
        self.declare_parameter('camera.tilt_variations_deg', [-15.0, 0.0, 15.0])
        self.declare_parameter('camera.max_incidence_angle_deg', 80.0)

        # --- UR10e reachability (camera on ur10e_depth_optical_frame) ---
        self.declare_parameter('ur_group_name', 'real_ur10e')
        self.declare_parameter('ur_base_frame', 'world')
        self.declare_parameter('ur_tool_frame', 'ur10e_depth_optical_frame')

        # WORKSPACE KEEP-OUT BOX for the UR ONLY (world frame, metres). The mesh-only
        # occlusion ray-trace cannot tell that a candidate camera pose BEHIND the UR's
        # linear rail is physically blocked, NOR that a pose near the FLOOR is
        # unreachable (the arm would hit the ground). This box makes the UR treat any
        # candidate whose camera position is outside [min, max] as UNREACHABLE, so the
        # allocator gives it to the Kawasaki instead (or drops it) -- the Kawasaki, on
        # its own base, is never constrained by this box, so Kawa coverage is preserved.
        #   X upper = -0.20: the rail sits at world X=-0.158 (cell at world origin,
        #     travels along Y); -0.20 (rail X + ~4 cm margin) rejects behind-rail poses.
        #   Z lower = 0.20: floor clearance -- rejects near-ground poses the UR can't
        #     reach without hitting the floor. Raise if the arm still grazes the floor;
        #     lower toward 0 if it wrongly drops legitimate low chassis viewpoints.
        # Set enabled False to disable. Same box the solo viewpoint_planner uses.
        self.declare_parameter('ur_workspace_filter_enabled', True)
        self.declare_parameter('ur_workspace_bounds_min', [-100.0, -100.0, 0.20])
        self.declare_parameter('ur_workspace_bounds_max', [-0.20, 100.0, 100.0])

        # --- Kawasaki reachability. The viewpoint is where the Kawasaki's SICK
        #     CAMERA must be. But MoveIt's /compute_ik for the real_kawasaki group
        #     (chain tip link6) does not reliably honour ik_link_name for the
        #     camera frame, which sits 4 fixed joints beyond link6 (including the
        #     big joint7 rpy) -- it ends up orienting link6, so the camera faces
        #     the wrong way. So we solve IK for the group's OWN tip (link6) and
        #     pre-apply the fixed camera->link6 transform (looked up from TF) to
        #     the target pose. That places the CAMERA exactly at the viewpoint,
        #     orientation guaranteed correct, independent of ik_link handling.
        self.declare_parameter('kawasaki_group_name', 'real_kawasaki')
        self.declare_parameter('kawasaki_base_frame', 'world')
        # The frame the generated viewpoint pose refers to (where the camera goes).
        self.declare_parameter('kawasaki_camera_frame', 'camera_rgb_optic_frame')
        # The Kawasaki group's IK tip link (what /compute_ik actually solves for).
        self.declare_parameter('kawasaki_group_tip', 'link6')
        # True -> robust tip+offset IK (recommended). False -> target the camera
        # frame directly via ik_link_name (only if your MoveIt honours it).
        self.declare_parameter('kawasaki_ik_via_tip', True)
        # Camera optical-axis convention correction (THE orientation fix). The
        # viewpoint generator produces poses in the ROS optical convention where
        # the camera looks along +Z. That is correct for the UR, whose *real*
        # depth_optical_frame (ur_macro.xacro, rpy 0 0 0) has +Z as its view axis.
        # The Kawasaki's camera_rgb_optic_frame (rs005l_macro.xacro) instead has
        # its view axis along +X (verified from saved clouds: captured geometry
        # sits along +X of that frame). So the target orientation for the Kawasaki
        # camera is R_cam = R_view @ C, where C maps the generator's +Z view axis
        # onto the frame's actual +X view axis. rpy(-pi/2,-pi/2,0) does exactly
        # that (it is the very rotation that relates the UR's sim vs real optical
        # frames). Tune only if the Kawasaki still does not square up to the chassis.
        self.declare_parameter('kawasaki_view_axis_correction_rpy',
                               [-1.5707963, -1.5707963, 0.0])

        self.declare_parameter('output_plan_file',
                               '/home/ifarlab/colcon_ws/src/multirobot_viewpoint_planner/plans/multirobot_viewpoint_plan.json')

        # Reorder each arm's selected viewpoints into a short cartesian visiting
        # path (nearest-neighbour + 2-opt) so the arm sweeps neighbouring stops
        # instead of jumping across the chassis in greedy coverage-discovery
        # order. Changes ONLY the visiting order, never which viewpoints are
        # kept. False -> keep the greedy selection order.
        self.declare_parameter('order_by_proximity', True)
        # Per-arm START anchor for the visiting sweep. The two arms approach the
        # chassis from OPPOSITE ends so they don't converge on the same region:
        #   UR       -> 'max_y' (largest Y, the FRONT of the chassis),
        #   Kawasaki -> 'min_y' (smallest Y, the BACK of the chassis).
        # Options: 'max_y' | 'min_y' | 'coverage' (start at the highest-coverage VP).
        self.declare_parameter('ur_order_anchor', 'max_y')
        self.declare_parameter('kawasaki_order_anchor', 'min_y')

        # 2*pi UNWINDING of the stored IK solutions. IK returns one arbitrary branch of
        # a solution family, so the tour can store e.g. +100 deg at one stop and -260 deg
        # at the next: the same physical joint angle, but a whole extra turn to drive.
        # Once the visiting order is fixed we walk the tour and re-express each angle as
        # the equivalent nearest the PREVIOUS stop, whenever the joint's URDF limits
        # allow it. Physically identical poses, so coverage and reachability are
        # untouched -- only the numbers the executor drives between.
        # Re-solve each viewpoint's IK seeded with the previous viewpoint's solution and
        # keep it only when it is closer, so the tour stays on ONE IK branch instead of
        # flipping between shoulder/elbow/wrist configurations that reach the same pose
        # from opposite arm postures. Unlike unwinding, this changes WHICH solution is
        # stored -- but never the viewpoint pose, so coverage is unaffected.
        self.declare_parameter('rechain_ik', True)
        # pick_ik runs with random restarts, so repeating the seeded query samples
        # different branches; we keep the closest that solves.
        self.declare_parameter('rechain_ik_attempts', 4)
        # Seed variants for the re-solve (see _seed_variants). Repetition alone only
        # samples branches when the solver restarts randomly, which the Kawasaki's KDL
        # does not do -- these give it something to converge away from.
        self.declare_parameter('rechain_rail_joint', 'world_to_agv')
        self.declare_parameter('rechain_rail_probe', 0.25)   # m, +/- rail seed offset; 0 disables
        self.declare_parameter('rechain_rail_weight', 2.0)   # rad per m of rail travel
        self.declare_parameter('rechain_yaw_probe', True)    # try joint1 flipped +/- pi
        self.declare_parameter('rechain_yaw_joint', 'joint1')

        self.declare_parameter('unwind_joint_goals', True)
        self.declare_parameter('wrap_limit_margin', 0.05)  # rad clear of each limit
        self.declare_parameter('wrap_min_gain', 0.35)      # rad (~20 deg) minimum saving

        # Latched URDF -> joint limits for the unwinding above.
        self._urdf_xml = None
        self._joint_limits = None
        self.create_subscription(
            String, '/robot_description', self._robot_description_cb,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       history=HistoryPolicy.KEEP_LAST))

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
            f"MultiRobotPlannerNode ready. mesh='{self.mesh_path}', "
            f"coverage_threshold={self.coverage_threshold}."
        )

    def _lookup_static_offset(self, parent_frame, child_frame, timeout=8.0):
        """Return (R, p): the pose of child_frame expressed in parent_frame, from
        /tf_static. Uses a short-lived node spun locally (the service callback
        blocks this node's own executor, so we can't rely on our own TF buffer).
        Returns None if the transform never arrives."""
        from tf2_ros import Buffer, TransformListener
        tmp = rclpy.create_node('mrvp_tf_lookup')
        buf = Buffer()
        TransformListener(buf, tmp)
        tf = None
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(tmp, timeout_sec=0.1)
            try:
                if buf.can_transform(parent_frame, child_frame, rclpy.time.Time()):
                    tf = buf.lookup_transform(parent_frame, child_frame, rclpy.time.Time())
                    break
            except Exception:
                pass
        tmp.destroy_node()
        if tf is None:
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        R = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        return R, np.array([t.x, t.y, t.z])

    def _analytic_kawasaki_offset(self):
        """Fixed pose of the group tip (link6) IN the camera optical frame,
        computed directly from the URDF chain in rs005l_macro.xacro. Using the
        URDF constants (instead of a TF lookup) makes IK-via-tip robust: the real
        (unprefixed) camera/link6 frames are NOT in TF during a sim run, so the
        old /tf_static lookup silently failed and fell back to a legacy ik_link
        path that MoveIt does not honour past the group tip -- which is why the
        correction never reached the camera and the Kawasaki kept facing wrong.

        Chain:
            link6 --joint7 (rpy 0 0 -1.96)------------------------> link7
                  --camera_joint (xyz .0145 .0125 .185,
                                   rpy 0 -pi/2 pi/2)--------------> camera(=optic)
            (camera_rgb_joint and camera_rgb_optic_joint are identity)

        Returns (R_off, p_off) = pose of link6 expressed in the camera frame, i.e.
        the same thing _lookup_static_offset(camera, link6) would have returned.
        """
        def rpy(r, p, y):
            return Rotation.from_euler('xyz', [r, p, y]).as_matrix()
        R67 = rpy(0.0, 0.0, -1.96)                    # link6 -> link7
        R7c = rpy(0.0, -np.pi / 2, np.pi / 2)         # link7 -> camera
        p7c = np.array([0.0145, 0.0125, 0.185])
        R6c = R67 @ R7c                               # camera IN link6
        p6c = R67 @ p7c                               # (joint7 has zero translation)
        R_off = R6c.T                                 # link6 IN camera
        p_off = -R6c.T @ p6c
        return R_off, p_off

    def _make_kawasaki_reach_fn(self, checker, offset, correction):
        """candidate -> (reachable, joint_solution) for the Kawasaki.

        Two corrections vs the naive UR path, both required for the camera to
        actually face the chassis:

          1. VIEW-AXIS CONVENTION (`correction`, a 3x3): the generator's pose has
             the view along +Z, but the Kawasaki camera_rgb_optic_frame views
             along +X. R_cam = R_view @ correction re-aims the frame so its real
             view axis points at the surface.
          2. IK VIA TIP (`offset` = (R_off, p_off) = pose of the group tip in the
             camera frame): we solve IK for the group's own tip with the fixed
             camera->tip transform pre-applied, so the CAMERA lands at the target
             even though the camera frame is several fixed joints past the tip.
             For the legacy direct-ik_link path pass offset = (I, 0).
        """
        if not checker.wait_for_service():
            self.get_logger().warning(
                f"/compute_ik unavailable for group '{checker.group_name}' (Kawasaki). Its viewpoints "
                "will be selected WITHOUT reachability verification.")
            return None
        R_off, p_off = offset
        C = np.asarray(correction, dtype=np.float64)

        def _fn(candidate, seed=None, count_errors=True):
            R_view = np.asarray(candidate['rotation'], dtype=np.float64)
            pos = np.asarray(candidate['position'], dtype=np.float64)
            R_cam = R_view @ C                    # orientation the camera must reach
            R_tip = R_cam @ R_off                 # -> the group tip's orientation
            p_tip = pos + R_cam @ p_off           # -> the group tip's position
            quat = Rotation.from_matrix(R_tip).as_quat()  # [x, y, z, w]
            return checker.check_ik(p_tip, quat, seed_joint_state=seed,
                                    count_errors=count_errors)

        return _fn

    def _ur_workspace_gate(self):
        """Return a callable(position)->bool that is True when the camera position
        is inside the UR keep-out box, or None when the box is disabled. Used to
        veto behind-rail candidates for the UR only (the Kawasaki is never gated)."""
        if not self.get_parameter('ur_workspace_filter_enabled').value:
            return None
        lo = np.asarray(self.get_parameter('ur_workspace_bounds_min').value, dtype=float)
        hi = np.asarray(self.get_parameter('ur_workspace_bounds_max').value, dtype=float)
        if lo.shape != (3,) or hi.shape != (3,):
            self.get_logger().warning(
                "ur_workspace_bounds_min/max must each have 3 values [x,y,z]; UR workspace gate off.")
            return None
        self.get_logger().info(
            f"UR workspace keep-out box active: camera must be within min={lo.tolist()} "
            f"max={hi.tolist()} (world). Behind-rail candidates are handed to the Kawasaki.")
        return lambda p: bool(np.all(np.asarray(p, dtype=float) >= lo)
                              and np.all(np.asarray(p, dtype=float) <= hi))

    def _make_reachability_fn(self, checker, label, workspace_gate=None):
        """candidate -> (reachable, joint_solution) for one arm. Returns None
        (arm treated as able to reach everything, UNVERIFIED) if /compute_ik never
        comes up, logged loudly so an unverified plan is never mistaken for a
        verified one. `workspace_gate` (optional callable(position)->bool) vetoes
        candidates outside this arm's allowed world-frame region BEFORE the IK
        query -- an out-of-box candidate is reported unreachable for this arm."""
        if not checker.wait_for_service():
            self.get_logger().warning(
                f"/compute_ik unavailable for group '{checker.group_name}' ({label}). Its viewpoints "
                "will be selected WITHOUT reachability verification and may fail at execution time."
            )
            return None

        def _fn(candidate, seed=None, count_errors=True):
            if workspace_gate is not None and not workspace_gate(candidate['position']):
                return False, None
            quat = Rotation.from_matrix(candidate['rotation']).as_quat()  # [x, y, z, w]
            return checker.check_ik(candidate['position'], quat, seed_joint_state=seed,
                                    count_errors=count_errors)

        return _fn

    def plan_callback(self, request, response):
        self.get_logger().info('Multi-robot planning requested.')
        t_start = time.monotonic()

        if not self.mesh_path or not os.path.exists(self.mesh_path):
            self.get_logger().error(f"Mesh path invalid or not set: '{self.mesh_path}'.")
            response.success = False
            response.message = f"Mesh path invalid: {self.mesh_path}"
            return response

        ur_checker = None
        kawa_checker = None
        try:
            # 1. Mesh analysis (identical to solo baseline).
            self.get_logger().info('Step 1: Mesh Analysis')
            t0 = time.monotonic()
            analyzer = MeshAnalyzer(self.mesh_path, scale=self.mesh_scale, logger=self.get_logger())
            analyzer.load_mesh()
            target_points, target_normals, _ = analyzer.sample_surface(
                num_points=self.get_parameter('target_sample_points').value)
            self.get_logger().info(f"Step 1 done in {time.monotonic() - t0:.2f}s.")

            # 2. Shared candidate pool + coverage matrix (identical to solo).
            self.get_logger().info('Step 2: Viewpoint Generation (shared pool)')
            t0 = time.monotonic()
            vg = ViewpointGenerator(analyzer, config=self.camera_config, logger=self.get_logger())
            candidates = vg.generate_candidates(
                target_points, target_normals,
                num_base_points=self.get_parameter('num_base_points').value)
            coverage_matrix = vg.compute_coverage_matrix(candidates, target_points, target_normals)
            self.get_logger().info(
                f"Step 2 done in {time.monotonic() - t0:.2f}s. {len(candidates)} candidates.")

            # 3. Build a reachability function per arm. The Kawasaki solves IK for
            #    its own camera optical frame, same as the UR.
            avoid_collisions = self.get_parameter('ik_avoid_collisions').value
            if not avoid_collisions:
                self.get_logger().warning(
                    "ik_avoid_collisions=False: measuring PURE KINEMATIC reachability -- selected "
                    "viewpoints may be in collision at execution time (diagnostic only).")

            ur_checker = ReachabilityChecker(
                group_name=self.get_parameter('ur_group_name').value,
                base_frame=self.get_parameter('ur_base_frame').value,
                tool_frame=self.get_parameter('ur_tool_frame').value,
                avoid_collisions=avoid_collisions,
            )

            kawa_group = self.get_parameter('kawasaki_group_name').value
            kawa_base = self.get_parameter('kawasaki_base_frame').value
            kawa_cam = self.get_parameter('kawasaki_camera_frame').value
            kawa_tip = self.get_parameter('kawasaki_group_tip').value
            via_tip = self.get_parameter('kawasaki_ik_via_tip').value

            # View-axis convention correction (camera_rgb_optic_frame views along
            # +X, not the generator's +Z). Applied in BOTH IK paths below.
            corr_rpy = list(self.get_parameter('kawasaki_view_axis_correction_rpy').value)
            C = Rotation.from_euler('xyz', corr_rpy).as_matrix()
            self.get_logger().info(
                f"Kawasaki view-axis correction rpy={corr_rpy} applied "
                "(generator +Z view -> camera_rgb_optic_frame +X view).")

            # offset = pose of the group tip in the camera frame. Identity means
            # "target the camera frame directly" (legacy path); a real transform
            # means "solve IK for the group's own tip (link6) with the fixed
            # camera->tip transform applied", which MoveIt always honours.
            offset = (np.eye(3), np.zeros(3))
            if via_tip:
                # Computed from the URDF chain, NOT from TF: the real camera/link6
                # frames are absent from TF during a sim run, so a TF lookup would
                # fail and quietly drop us onto the unreliable legacy path.
                offset = self._analytic_kawasaki_offset()
                self.get_logger().info(
                    f"Kawasaki IK via tip '{kawa_tip}' with analytic camera->tip offset "
                    f"(link6-in-camera t={offset[1].round(4).tolist()}); "
                    "camera placed exactly at each viewpoint.")

            kawa_checker = ReachabilityChecker(
                group_name=kawa_group,
                base_frame=kawa_base,
                # When solving via the tip we set ik_link to the tip; otherwise we
                # target the camera frame directly (legacy path).
                tool_frame=(kawa_tip if via_tip else kawa_cam),
                avoid_collisions=avoid_collisions,
            )
            kawa_reach_fn = self._make_kawasaki_reach_fn(kawa_checker, offset, C)

            # UR-only workspace gate (behind-rail veto). The Kawasaki gets no gate,
            # so candidates the UR can't legitimately reach still go to the Kawasaki.
            ur_ws_gate = self._ur_workspace_gate()
            ur_reach_fn = self._make_reachability_fn(
                ur_checker, 'UR10e', workspace_gate=ur_ws_gate)
            robots = [
                {'name': 'ur', 'reachability_fn': ur_reach_fn},
                {'name': 'kawasaki', 'reachability_fn': kawa_reach_fn},
            ]

            # 4. Greedy multi-robot allocation over the shared pool.
            self.get_logger().info('Step 3: Multi-robot allocation')
            t0 = time.monotonic()
            max_per = self.get_parameter('max_viewpoints_per_robot').value
            # Keys must match the 'name' fields in `robots` above.
            per_arm_caps = {
                'ur': self.get_parameter('max_viewpoints_ur').value,
                'kawasaki': self.get_parameter('max_viewpoints_kawasaki').value,
            }
            allocator = RobotAllocator(
                coverage_threshold=self.coverage_threshold,
                max_per_robot=max_per,
                max_per_robot_by_name=per_arm_caps,
                balance=self.get_parameter('balance_load').value,
                min_new_points=self.get_parameter('min_new_points').value,
                min_marginal_coverage=self.get_parameter('min_marginal_coverage').value,
                logger=self.get_logger(),
            )
            assignments, final_coverage = allocator.allocate(candidates, coverage_matrix, robots)
            ur_checker.log_error_breakdown()
            kawa_checker.log_error_breakdown()
            self.get_logger().info(f"Step 3 done in {time.monotonic() - t0:.2f}s.")

            ur_vps = assignments.get('ur', [])
            kawa_vps = assignments.get('kawasaki', [])

            # Reorder each arm into a short cartesian visiting path so the arm
            # sweeps neighbouring stops instead of jumping in greedy discovery
            # order. Selection is unchanged; only the visiting order (and thus
            # 'rank' + list order the executor consumes) changes.
            if self.get_parameter('order_by_proximity').value:
                ur_anchor = self.get_parameter('ur_order_anchor').value
                kawa_anchor = self.get_parameter('kawasaki_order_anchor').value
                ur_vps = self._order_by_proximity(ur_vps, anchor=ur_anchor)
                kawa_vps = self._order_by_proximity(kawa_vps, anchor=kawa_anchor)
                self.get_logger().info(
                    "Viewpoints reordered by cartesian proximity (nearest-neighbour "
                    f"+ 2-opt + Or-opt). UR starts at '{ur_anchor}', Kawasaki at "
                    f"'{kawa_anchor}' (opposite ends of the chassis).")

            for i, vp in enumerate(ur_vps):
                vp['id'] = f'ur_vp_{i:03d}'
                vp['rank'] = i
            for i, vp in enumerate(kawa_vps):
                vp['id'] = f'kawa_vp_{i:03d}'
                vp['rank'] = i

            # Ids exist and the visiting order is final, so the two continuity passes can
            # run. Order matters: first move each viewpoint onto an IK branch near its
            # predecessor (a genuinely different arm configuration reaching the same
            # pose), THEN unwind whatever whole-turn offsets remain within that branch.
            self._rechain_ik(ur_vps, ur_reach_fn, 'UR tour')
            self._rechain_ik(kawa_vps, kawa_reach_fn, 'Kawasaki tour')
            self._unwind_tour(ur_vps, 'UR tour')
            self._unwind_tour(kawa_vps, 'Kawasaki tour')

            # 5. Save plan.
            plan = {
                "coverage_achieved": float(final_coverage),
                "coverage_threshold": float(self.coverage_threshold),
                "objective": "multi_robot_max_coverage_balanced",
                "total_ur_viewpoints": len(ur_vps),
                "total_kawasaki_viewpoints": len(kawa_vps),
                "camera_config": self.camera_config,
                "ur_viewpoints": [self._vp_to_dict(vp) for vp in ur_vps],
                "kawasaki_viewpoints": [self._vp_to_dict(vp) for vp in kawa_vps],
            }
            plan_file = os.path.expanduser(self.get_parameter('output_plan_file').value)
            os.makedirs(os.path.dirname(plan_file), exist_ok=True)
            with open(plan_file, 'w') as f:
                json.dump(plan, f, indent=2)

            total_time = time.monotonic() - t_start
            msg = (
                f"Multi-robot plan created. Coverage: {final_coverage * 100:.1f}%. "
                f"UR: {len(ur_vps)} viewpoints, Kawasaki: {len(kawa_vps)} viewpoints "
                f"(parallel run ~= {max(len(ur_vps), len(kawa_vps))} stops). Saved to {plan_file}."
            )
            self.get_logger().info(f"Planning finished in {total_time:.2f}s. {msg}")
            response.success = True
            response.message = msg

        except Exception as e:
            self.get_logger().error(f"Planning failed after {time.monotonic() - t_start:.2f}s: {e}")
            self.get_logger().error(traceback.format_exc())
            response.success = False
            response.message = f"Error: {e}"
        finally:
            for checker in (ur_checker, kawa_checker):
                if checker is not None:
                    try:
                        checker.destroy_node()
                    except Exception as e:
                        self.get_logger().warning(f"Failed to destroy reachability checker node: {e}")

        return response

    # ------------------------------------------------------------------ #
    # 2*pi unwinding of the stored IK solutions.
    # ------------------------------------------------------------------ #
    def _robot_description_cb(self, msg: String):
        if self._urdf_xml is None:
            self._urdf_xml = msg.data

    def _get_joint_limits(self, timeout=5.0):
        """Parse the latched URDF once. An empty table means 'do not unwind anything',
        which is the safe degradation: goals stay exactly as IK produced them."""
        if self._joint_limits is not None:
            return self._joint_limits
        deadline = time.monotonic() + timeout
        while self._urdf_xml is None and time.monotonic() < deadline:
            time.sleep(0.1)
        if self._urdf_xml is None:
            self.get_logger().warning(
                'No /robot_description available -- joint limits unknown, so the stored '
                'IK solutions are left un-unwound.')
            self._joint_limits = {}
            return self._joint_limits
        try:
            self._joint_limits = parse_joint_limits(self._urdf_xml)
            wrappable = sorted(n for n, v in self._joint_limits.items() if v.wrappable)
            self.get_logger().info(
                f'Joint limits parsed for {len(self._joint_limits)} joints; '
                f'{len(wrappable)} can be unwound: {wrappable}')
        except Exception as e:
            self.get_logger().warning(f'Could not parse /robot_description ({e}); '
                                      'IK solutions left un-unwound.')
            self._joint_limits = {}
        return self._joint_limits

    def _joint_distance(self, sol_a, sol_b):
        """Cost of moving between two IK solutions, in radians.

        Revolute joints count by their SHORTEST equivalent, so a pure 2*pi difference --
        which the unwinding pass removes anyway -- does not masquerade as a branch change.

        PRISMATIC joints (the Kawasaki's `world_to_agv` rail) count too, converted with
        `rechain_rail_weight` rad/m. They used to be skipped entirely, which is wrong for
        the Kawasaki: its group is REDUNDANT (rail + 6 joints), so the rail is a free
        parameter of every IK solve and a candidate that drives the AGV a metre used to
        score as free. Measured on the 2026-08-03 plan the Kawasaki tour drove the rail
        3.33 m over 8 viewpoints."""
        if sol_a is None or sol_b is None:
            return float('inf')
        limits = self._joint_limits or {}
        rail_w = float(self.get_parameter('rechain_rail_weight').value)
        b = dict(zip(sol_b.name, sol_b.position))
        total = 0.0
        for n, pa in zip(sol_a.name, sol_a.position):
            if n not in b:
                continue
            lim = limits.get(n)
            d = abs(float(b[n]) - float(pa))
            if lim is not None and not lim.is_revolute:
                total += d * rail_w
                continue
            total += min(d, abs(d - 2.0 * np.pi))
        return total

    def _seed_variants(self, prev, current, vp_id):
        """Seeds to try for one viewpoint's IK re-solve, best-guess first.

        Repeating the SAME seeded query only samples new branches when the solver
        restarts randomly, which is true of pick_ik in `mode: global` (the UR) but NOT of
        KDL (the Kawasaki group): KDL is a local solver and returns the same answer every
        time, so `rechain_ik_attempts` alone buys the Kawasaki exactly one candidate.
        These variants push it into genuinely different parts of the solution space:

          - `prev`, the plain chained seed, and `current`, so the existing solution can
            always be re-converged to.
          - RAIL PROBES: prev with the rail shifted +/- `rechain_rail_probe`. This is the
            redundancy resolution knob -- the same camera pose is reachable from a range
            of AGV positions, and the nearest one is usually not the one an unseeded solve
            picked.
          - BASE-YAW PROBES: prev with joint1 turned +/- pi. The 2026-08-03 plan had a
            248 deg joint1 swing in one hop, and joint1 spans exactly +/-180 so unwinding
            can never touch it; only a different branch can.

        Every variant is clamped to the URDF limits, so a probe can never propose an
        out-of-range seed."""
        limits = self._joint_limits or {}

        def clamp(name, value):
            lim = limits.get(name)
            if lim is None:
                return value
            return float(np.clip(value, lim.lower + 1e-6, lim.upper - 1e-6))

        def shifted(base, changes):
            if base is None:
                return None
            out = JointState()
            out.name = list(base.name)
            pos = [float(v) for v in base.position]
            touched = False
            for jname, delta in changes.items():
                if jname not in out.name:
                    continue
                i = out.name.index(jname)
                new = clamp(jname, pos[i] + delta)
                if abs(new - pos[i]) > 1e-4:
                    pos[i] = new
                    touched = True
            if not touched:
                return None
            out.position = pos
            return out

        rail = str(self.get_parameter('rechain_rail_joint').value)
        probe = float(self.get_parameter('rechain_rail_probe').value)
        seeds = [prev, current]
        if probe > 0.0:
            seeds += [shifted(prev, {rail: probe}), shifted(prev, {rail: -probe})]
        if bool(self.get_parameter('rechain_yaw_probe').value):
            yaw = str(self.get_parameter('rechain_yaw_joint').value)
            seeds += [shifted(prev, {yaw: np.pi}), shifted(prev, {yaw: -np.pi})]
        return [s for s in seeds if s is not None]

    def _rechain_ik(self, vps, reach_fn, label):
        """Re-solve each viewpoint's IK SEEDED with the previous viewpoint's solution,
        and keep the result only when it is closer in joint space than what we already
        had.

        Why this exists: every pose has several IK branches (shoulder/elbow/wrist flips)
        that reach it from completely different arm configurations. Allocation solves each
        viewpoint independently -- and pick_ik runs in `mode: global`, i.e. random restarts
        -- so neighbouring viewpoints routinely land on different branches. The arm then
        swings through hundreds of degrees to move a few centimetres, which is NOT
        something 2*pi unwinding can fix: the configurations are genuinely different.

        The viewpoint POSE is never changed here, only which solution reaches it, so
        coverage is untouched. A viewpoint keeps its original solution whenever the
        re-solve fails or is no better, so this can only reduce travel."""
        if not vps or reach_fn is None or not self.get_parameter('rechain_ik').value:
            return
        attempts = max(1, int(self.get_parameter('rechain_ik_attempts').value))
        self._get_joint_limits()
        rail_before = self._tour_rail_travel(vps)

        improved = 0
        saved = 0.0
        prev = vps[0].get('joint_solution')
        for vp in vps[1:]:
            current = vp.get('joint_solution')
            if prev is None:
                prev = current
                continue
            best, best_d = current, self._joint_distance(prev, current)
            # Two ways to reach a different branch: repeat the query (only samples
            # anything when the solver restarts randomly, i.e. pick_ik/global on the UR)
            # and vary the SEED (the only thing that moves KDL on the Kawasaki). Both are
            # tried, and a result is kept solely when it is strictly closer, so this can
            # never make a tour worse.
            for seed in self._seed_variants(prev, current, vp.get('id', '?')):
                for _ in range(attempts):
                    ok, sol = reach_fn(vp, seed=seed, count_errors=False)
                    if not ok or sol is None:
                        continue
                    d = self._joint_distance(prev, sol)
                    if d < best_d:
                        best, best_d = sol, d
            if best is not current and best is not None:
                before = self._joint_distance(prev, current)
                self.get_logger().info(
                    f"  {vp.get('id', '?')}: closer IK branch found -- "
                    f"{np.rad2deg(before):.0f} -> {np.rad2deg(best_d):.0f} deg from the "
                    "previous viewpoint.")
                vp['joint_solution'] = best
                improved += 1
                saved += before - best_d
            prev = vp.get('joint_solution')

        rail_after = self._tour_rail_travel(vps)
        if improved:
            self.get_logger().info(
                f'{label}: re-seeded {improved} viewpoint(s) onto a nearer IK branch, '
                f'removing {np.rad2deg(saved):.0f} deg of travel.')
        else:
            self.get_logger().info(f'{label}: no viewpoint had a nearer IK branch.')
        if rail_before is not None and rail_after is not None:
            self.get_logger().info(
                f'{label}: rail travel {rail_before:.3f} -> {rail_after:.3f} m.')

    def _tour_rail_travel(self, vps):
        """Total prismatic (AGV rail) travel along an ordered tour, in metres. None when
        the tour has no prismatic joint, i.e. for the UR."""
        limits = self._joint_limits or {}
        rail = str(self.get_parameter('rechain_rail_joint').value)
        vals = []
        for vp in vps:
            sol = vp.get('joint_solution')
            if sol is None or rail not in sol.name:
                return None
            vals.append(float(sol.position[list(sol.name).index(rail)]))
        lim = limits.get(rail)
        if lim is not None and lim.is_revolute:
            return None
        return float(np.abs(np.diff(vals)).sum()) if len(vals) > 1 else 0.0

    def _unwind_tour(self, vps, label):
        """Walk an ORDERED tour and re-express each viewpoint's joint angles as the
        2*pi-equivalents nearest the PREVIOUS viewpoint, within the URDF limits.

        The first viewpoint is left exactly as IK produced it: there is no meaningful
        reference for it here, and the executor unwinds it against the arm's real
        measured pose at run time anyway. Poses are unchanged throughout -- theta and
        theta +/- 2*pi place every link identically -- so this cannot alter coverage,
        reachability or collisions, only how far the arm drives between stops."""
        if not vps or not self.get_parameter('unwind_joint_goals').value:
            return
        limits = self._get_joint_limits()
        if not limits:
            return
        margin = float(self.get_parameter('wrap_limit_margin').value)
        min_gain = float(self.get_parameter('wrap_min_gain').value)

        reference = None
        total_saved = 0.0
        rewritten = 0
        for vp in vps:
            sol = vp.get('joint_solution')
            if sol is None:
                continue
            names = list(sol.name)
            positions = [float(x) for x in sol.position]
            if reference is not None:
                positions, changes = wrap_to_reference(
                    positions, names, reference, limits, margin=margin, min_gain=min_gain)
                if changes:
                    rewritten += len(changes)
                    total_saved += sum(c[3] for c in changes)
                    self.get_logger().info(
                        f"  {vp.get('id', '?')}: {describe_changes(changes)}")
                    sol.position = positions
            reference = dict(zip(names, positions))

        if rewritten:
            self.get_logger().info(
                f'{label}: unwound {rewritten} joint goal(s) across the tour, removing '
                f'{np.rad2deg(total_saved):.0f} deg of needless rotation.')
        else:
            self.get_logger().info(f'{label}: no joint goals needed unwinding.')

    @staticmethod
    def _order_by_proximity(vps, anchor='coverage'):
        """Reorder viewpoints into a short cartesian visiting path so the arm
        sweeps neighbouring stops instead of criss-crossing the chassis. Only the
        ORDER changes -- the exact same set of viewpoints is returned.

        Nearest-neighbour tour from a chosen ANCHOR viewpoint, then 2-opt + Or-opt
        refinement (which never move the anchor). Distance = Euclidean distance
        between viewpoint positions (a proxy for arm travel). With <=30 stops per
        arm this is effectively instant.

        anchor selects the START viewpoint:
          'coverage' -> index 0 (the highest-coverage VP the greedy put first),
          'max_y'    -> the largest-Y viewpoint  (the FRONT of the chassis),
          'min_y'    -> the smallest-Y viewpoint (the BACK of the chassis).
        """
        n = len(vps)
        if n <= 2:
            return list(vps)
        pos = np.array([np.asarray(v['position'], dtype=np.float64) for v in vps])
        dist = np.linalg.norm(pos[:, None, :] - pos[None, :, :], axis=2)

        # Pick the anchor (start) index per the requested rule.
        if anchor == 'max_y':
            start = int(np.argmax(pos[:, 1]))
        elif anchor == 'min_y':
            start = int(np.argmin(pos[:, 1]))
        else:
            start = 0  # 'coverage': greedy already put the best-coverage VP first

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

        # Or-opt: relocate short chains (length 1-3) to a cheaper spot. 2-opt can
        # only REVERSE a segment, so it leaves the long back-and-forth hops that
        # happen when one viewpoint sits far off the main sweep; Or-opt moves that
        # stray viewpoint next to its true neighbour and shortens the path further.
        # tour[0] (the highest-coverage anchor) is never relocated. n is small so
        # the full O(n^2) recompute per candidate is negligible.
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

        return [vps[t] for t in tour]

    @staticmethod
    def _vp_to_dict(vp):
        d = {
            "id": vp['id'],
            "robot": vp.get('robot'),
            "position": np.asarray(vp['position']).tolist(),
            "rotation": np.asarray(vp['rotation']).tolist(),
            "rank": vp.get('rank'),
            "global_order": vp.get('global_order'),
            "new_points_covered": vp.get('new_points_covered'),
            "total_points_visible": vp.get('total_points_visible'),
            "cumulative_coverage": vp.get('cumulative_coverage'),
        }
        # JointState.name is a list[str]; .position is an array.array the json
        # module cannot serialize -- cast both to plain lists.
        sol = vp.get('joint_solution')
        if sol is not None:
            d['joint_names'] = list(sol.name)
            d['joint_positions'] = list(sol.position)
        return d


def main(args=None):
    rclpy.init(args=args)
    node = MultiRobotPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
