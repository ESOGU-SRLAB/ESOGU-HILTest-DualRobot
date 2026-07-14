import json
import os
import time
import traceback

import numpy as np
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from scipy.spatial.transform import Rotation

# Reuse the single-arm viewpoint_planner core verbatim (this package depends on
# it). Only the ALLOCATION across the two arms is new -- everything up to and
# including the shared candidate pool + coverage matrix is identical to the solo
# baseline, which keeps the solo-vs-multi comparison honest.
from viewpoint_planner.mesh_analyzer import MeshAnalyzer
from viewpoint_planner.viewpoint_generator import ViewpointGenerator
from viewpoint_planner.reachability_checker import ReachabilityChecker

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
        # True -> a candidate reachable by both arms goes to the less-loaded arm
        # (shorter parallel run). False -> fixed arm priority order.
        self.declare_parameter('balance_load', True)
        # Tail cutter: stop adding viewpoints once the best remaining one would add
        # fewer than this many NEW target points. The coverage curve has a sharp
        # knee, then a long tail of viewpoints that each add only 1-3 points. Raise
        # this to cut the tail and get far fewer waypoints (small coverage cost).
        self.declare_parameter('min_new_points', 1)

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
                               '/home/cem/colcon_ws/src/multirobot_viewpoint_planner/plans/multirobot_viewpoint_plan.json')

        # Reorder each arm's selected viewpoints into a short cartesian visiting
        # path (nearest-neighbour + 2-opt) so the arm sweeps neighbouring stops
        # instead of jumping across the chassis in greedy coverage-discovery
        # order. Changes ONLY the visiting order, never which viewpoints are
        # kept. False -> keep the greedy selection order.
        self.declare_parameter('order_by_proximity', True)

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

        def _fn(candidate):
            R_view = np.asarray(candidate['rotation'], dtype=np.float64)
            pos = np.asarray(candidate['position'], dtype=np.float64)
            R_cam = R_view @ C                    # orientation the camera must reach
            R_tip = R_cam @ R_off                 # -> the group tip's orientation
            p_tip = pos + R_cam @ p_off           # -> the group tip's position
            quat = Rotation.from_matrix(R_tip).as_quat()  # [x, y, z, w]
            return checker.check_ik(p_tip, quat)

        return _fn

    def _make_reachability_fn(self, checker, label):
        """candidate -> (reachable, joint_solution) for one arm. Returns None
        (arm treated as able to reach everything, UNVERIFIED) if /compute_ik never
        comes up, logged loudly so an unverified plan is never mistaken for a
        verified one."""
        if not checker.wait_for_service():
            self.get_logger().warning(
                f"/compute_ik unavailable for group '{checker.group_name}' ({label}). Its viewpoints "
                "will be selected WITHOUT reachability verification and may fail at execution time."
            )
            return None

        def _fn(candidate):
            quat = Rotation.from_matrix(candidate['rotation']).as_quat()  # [x, y, z, w]
            return checker.check_ik(candidate['position'], quat)

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

            robots = [
                {'name': 'ur', 'reachability_fn': self._make_reachability_fn(ur_checker, 'UR10e')},
                {'name': 'kawasaki', 'reachability_fn': kawa_reach_fn},
            ]

            # 4. Greedy multi-robot allocation over the shared pool.
            self.get_logger().info('Step 3: Multi-robot allocation')
            t0 = time.monotonic()
            max_per = self.get_parameter('max_viewpoints_per_robot').value
            allocator = RobotAllocator(
                coverage_threshold=self.coverage_threshold,
                max_per_robot=max_per,
                balance=self.get_parameter('balance_load').value,
                min_new_points=self.get_parameter('min_new_points').value,
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
                ur_vps = self._order_by_proximity(ur_vps)
                kawa_vps = self._order_by_proximity(kawa_vps)
                self.get_logger().info(
                    "Viewpoints reordered by cartesian proximity "
                    "(nearest-neighbour + 2-opt) to shorten arm travel.")

            for i, vp in enumerate(ur_vps):
                vp['id'] = f'ur_vp_{i:03d}'
                vp['rank'] = i
            for i, vp in enumerate(kawa_vps):
                vp['id'] = f'kawa_vp_{i:03d}'
                vp['rank'] = i

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

    @staticmethod
    def _order_by_proximity(vps):
        """Reorder viewpoints into a short cartesian visiting path so the arm
        sweeps neighbouring stops instead of criss-crossing the chassis. Only the
        ORDER changes -- the exact same set of viewpoints is returned.

        Nearest-neighbour tour (anchored at the highest-coverage viewpoint, which
        the greedy put first) followed by 2-opt refinement. Distance = Euclidean
        distance between viewpoint positions (a proxy for arm travel). With <=30
        stops per arm this is effectively instant.
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
