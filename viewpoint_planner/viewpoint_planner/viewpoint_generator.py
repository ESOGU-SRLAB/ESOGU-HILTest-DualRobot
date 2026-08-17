import logging
import numpy as np
import trimesh
from scipy.spatial.transform import Rotation


class ViewpointGenerator:
    """
    Generates candidate viewpoints from the target points and calculates coverage.
    """
    def __init__(self, mesh_analyzer, config=None, logger=None):
        self.mesh_analyzer = mesh_analyzer
        self.logger = logger or logging.getLogger(__name__)
        self.config = config or self.default_config()
        self.logger.info(f"ViewpointGenerator config: {self.config}")

    def default_config(self):
        """
        Default camera model, based on the SICK Visionary-T Mini V3S145-1AAAAAA
        datasheet (https://cdn.sick.com/media/pdf/6/06/906/dataSheet_V3S145-1AAAAAA_1127086_en.pdf):
          - Field of view: 70 deg (H) x 60 deg (Q) -- matches the Gazebo
            rgbd_camera sensor defined on depth_optical_frame in
            simrobot_ur_macro.xacro (horizontal_fov=1.22 rad ~= 69.9 deg,
            derived vertical FOV from the 512x424 aspect ratio ~= 60 deg).
          - Sensor resolution: 512 x 424 (matches datasheet exactly).
          - `fov_safety_margin_deg` shrinks the *effective* FOV used for
            coverage/occlusion checks, to keep viewpoints away from the lens
            edge where distortion/accuracy is worse. Set to 0.0 to use the
            full nominal FOV.
          - Real hardware supports up to 30 fps; the simulated sensor is
            throttled to 5 Hz (see depth_optical_frame sensor update_rate in
            simrobot_ur_macro.xacro) to keep the processing pipeline
            realtime. `frame_rate_hz` here matches the simulated rate.
        """
        return {
            'horizontal_fov_deg': 70.0,
            'vertical_fov_deg': 60.0,
            'fov_safety_margin_deg': 3.0,
            'min_range_m': 0.2,
            'max_range_m': 1.0,
            'frame_rate_hz': 5.0,
            'viewpoint_distances': [0.3, 0.4, 0.5],
            'tilt_variations_deg': [-15, 0, 15],
            'max_incidence_angle_deg': 75.0
        }

    def generate_candidates(self, target_points, target_normals, num_base_points=250):
        """
        Generates candidate viewpoints based on target surface normals.

        num_base_points controls how many surface points spawn candidates. This
        is the main lever on the GEOMETRIC COVERAGE CEILING: a target can only be
        covered if some candidate's viewing frustum reaches it, so too few base
        points leaves gaps on a large/complex mesh. Raising it increases the
        ceiling at the cost of more candidates (slower coverage matrix + more IK
        queries). Each base point yields len(distances) * len(tilts) candidates.
        """
        candidates = []
        # Cast to float64 explicitly: mesh_analyzer always produces float
        # arrays in practice, but integer input (e.g. from tests or hand-built
        # callers) would otherwise crash the in-place division below under
        # numpy's strict same_kind casting rules.
        target_points = np.asarray(target_points, dtype=np.float64)
        target_normals = np.asarray(target_normals, dtype=np.float64)

        # Downsample the target points to ~num_base_points candidate seeds.
        step = max(1, len(target_points) // max(1, num_base_points))

        base_points = target_points[::step]
        base_normals = target_normals[::step]

        for pt, normal in zip(base_points, base_normals):
            for d in self.config['viewpoint_distances']:
                # The camera should look AT the target point, so we move in the direction of the normal
                vp_pos = pt + normal * d

                # Default orientation: camera Z-axis points towards -normal
                # In ROS camera frames: Z is forward, X is right, Y is down
                z_axis = -normal

                # To fully define the orientation, we need an up vector (Y axis or X axis)
                world_up = np.array([0, 0, 1])

                # Handle case where normal is parallel to world_up
                if np.abs(np.dot(z_axis, world_up)) > 0.99:
                    world_up = np.array([1, 0, 0])

                x_axis = np.cross(world_up, z_axis)
                x_axis /= np.linalg.norm(x_axis)
                y_axis = np.cross(z_axis, x_axis)

                base_rot_mat = np.column_stack((x_axis, y_axis, z_axis))

                for tilt in self.config['tilt_variations_deg']:
                    if tilt == 0:
                        candidates.append({'position': vp_pos, 'rotation': base_rot_mat})
                    else:
                        # Tilt around X or Y axis to get variations
                        r_tilt = Rotation.from_euler('x', tilt, degrees=True)
                        rot_mat = base_rot_mat @ r_tilt.as_matrix()
                        candidates.append({'position': vp_pos, 'rotation': rot_mat})

        self.logger.info(
            f"Generated {len(candidates)} candidate viewpoints from {len(base_points)} base surface points "
            f"(step={step}, distances={self.config['viewpoint_distances']}, "
            f"tilts={self.config['tilt_variations_deg']})."
        )
        return candidates

    def make_intersector(self):
        """Builds the trimesh ray intersector used for occlusion checks."""
        if self.mesh_analyzer.mesh is None:
            self.mesh_analyzer.load_mesh()
        # Ensure pyembree is installed for performance, otherwise fall back to
        # the slower built-in R-tree intersector.
        try:
            intersector = trimesh.ray.ray_pyembree.RayMeshIntersector(self.mesh_analyzer.mesh)
            self.logger.debug("Using pyembree-accelerated ray intersector.")
        except AttributeError:
            self.logger.warning(
                "pyembree not found. Falling back to the built-in R-tree ray intersector "
                "(this can be significantly slower for dense meshes/candidate sets)."
            )
            intersector = trimesh.ray.ray_triangle.RayMeshIntersector(self.mesh_analyzer.mesh)
        return intersector

    def visible_mask(self, pos, rot, target_points, target_normals, intersector, stats=None):
        """Boolean mask over target_points: which targets a single camera pose
        (pos; rot columns X/Y/Z, camera looks along +Z) can see, honouring range,
        FOV, incidence angle and occlusion. Reused by both the full coverage
        matrix and the clusterer's re-verification of merged poses. When `stats`
        (a dict) is given, accumulates rejection counts by reason.
        """
        pos = np.asarray(pos, dtype=np.float64)
        rot = np.asarray(rot, dtype=np.float64)

        margin = self.config.get('fov_safety_margin_deg', 0.0)
        hfov = np.deg2rad(self.config['horizontal_fov_deg'] - margin)
        vfov = np.deg2rad(self.config['vertical_fov_deg'] - margin)
        min_range = self.config['min_range_m']
        max_range = self.config['max_range_m']
        max_incidence = np.cos(np.deg2rad(self.config['max_incidence_angle_deg']))

        vecs = target_points - pos
        dists = np.linalg.norm(vecs, axis=1)
        dist_mask = (dists >= min_range) & (dists <= max_range)

        vecs_cam = vecs @ rot
        z_cam = vecs_cam[:, 2]
        z_cam_safe = np.where(z_cam > 1e-6, z_cam, 1e-6)
        x_angles = np.abs(np.arctan(vecs_cam[:, 0] / z_cam_safe))
        y_angles = np.abs(np.arctan(vecs_cam[:, 1] / z_cam_safe))
        fov_mask = (x_angles <= hfov / 2) & (y_angles <= vfov / 2) & (z_cam > 0)

        safe_dists = np.where(dists > 1e-9, dists, 1e-9)
        ray_dirs = vecs / safe_dists[:, np.newaxis]
        dot_prods = np.sum(ray_dirs * target_normals, axis=1)
        incidence_mask = dot_prods <= -max_incidence

        mask = dist_mask & fov_mask & incidence_mask

        if stats is not None:
            stats['distance'] = stats.get('distance', 0) + int(np.sum(~dist_mask))
            stats['fov'] = stats.get('fov', 0) + int(np.sum(dist_mask & ~fov_mask))
            stats['incidence'] = stats.get('incidence', 0) + int(np.sum(dist_mask & fov_mask & ~incidence_mask))

        # Occlusion check: a target is hidden if the first mesh hit along the ray
        # is closer than the target itself (2 cm tolerance for self-hits).
        visible_indices = np.where(mask)[0]
        pre_occlusion_count = len(visible_indices)
        if pre_occlusion_count > 0:
            ray_origins = np.tile(pos, (pre_occlusion_count, 1))
            ray_directions = ray_dirs[visible_indices]
            locations, index_ray, index_tri = intersector.intersects_location(
                ray_origins=ray_origins, ray_directions=ray_directions, multiple_hits=False
            )
            for r_idx, hit_loc in zip(index_ray, locations):
                orig_idx = visible_indices[r_idx]
                if np.linalg.norm(hit_loc - pos) < dists[orig_idx] - 0.02:
                    mask[orig_idx] = False

        if stats is not None:
            stats['occluded'] = stats.get('occluded', 0) + (pre_occlusion_count - int(np.sum(mask)))
            stats['visible'] = stats.get('visible', 0) + int(np.sum(mask))
        return mask

    def compute_coverage_matrix(self, candidates, target_points, target_normals):
        """
        Computes a boolean matrix [num_candidates, num_targets]
        indicating which targets are visible from which candidates.
        """
        self.logger.info(
            f"Computing coverage matrix for {len(candidates)} candidates x {len(target_points)} targets. "
            "This may take a while..."
        )
        num_c = len(candidates)
        num_t = len(target_points)
        coverage_matrix = np.zeros((num_c, num_t), dtype=bool)
        target_points = np.asarray(target_points, dtype=np.float64)
        target_normals = np.asarray(target_normals, dtype=np.float64)

        intersector = self.make_intersector()

        # Accumulates rejection counts by reason across all candidates.
        stats = {}
        for i, cand in enumerate(candidates):
            coverage_matrix[i, :] = self.visible_mask(
                cand['position'], cand['rotation'], target_points, target_normals, intersector, stats=stats)
            if (i + 1) % 50 == 0 or (i + 1) == num_c:
                self.logger.info(f"Processed {i + 1}/{num_c} candidates...")

        occluded_total = stats.get('occluded', 0)
        visible_total = stats.get('visible', 0)
        rejected_by_distance = stats.get('distance', 0)
        rejected_by_fov = stats.get('fov', 0)
        rejected_by_incidence = stats.get('incidence', 0)

        candidates_with_no_coverage = int(np.sum(coverage_matrix.sum(axis=1) == 0))
        self.logger.info(
            f"Coverage matrix complete. Total visible (point,candidate) pairs: {visible_total}, "
            f"occluded/rejected by ray-trace: {occluded_total}. "
            f"{candidates_with_no_coverage}/{num_c} candidates cover zero target points "
            "(will never be selected by the optimizer)."
        )

        # DECISIVE diagnostic: how much coverage is even *possible* ignoring the
        # robot. A target seen by >=1 candidate is geometrically coverable; the
        # rest can never be covered no matter how good IK/reachability is. This
        # separates a *sensing-geometry* bottleneck (raise this ceiling with more
        # candidates / larger range / better viewing angles / a cleaner mesh)
        # from a *reachability* bottleneck (fix IK/mounting/linear-axis).
        targets_covered_by_any = int(np.sum(coverage_matrix.sum(axis=0) > 0))
        geometric_ceiling = targets_covered_by_any / num_t if num_t else 0.0
        self.logger.info(
            f"GEOMETRIC COVERAGE CEILING: {targets_covered_by_any}/{num_t} "
            f"({geometric_ceiling * 100:.1f}%) targets are visible from at least one candidate, "
            "ignoring reachability. No plan can exceed this. If it is far below your "
            "coverage_threshold, the bottleneck is SENSING GEOMETRY (candidate density / range / "
            "FOV / occlusion / mesh), not the robot. If it is high but achieved coverage is low, "
            "the bottleneck is REACHABILITY (IK / mounting / linear axis)."
        )
        total_pairs = num_c * num_t
        self.logger.info(
            f"Rejection breakdown over {total_pairs} (candidate,target) pairs -- "
            f"out of range: {rejected_by_distance}, outside FOV: {rejected_by_fov}, "
            f"incidence angle too steep (>{self.config['max_incidence_angle_deg']} deg): {rejected_by_incidence}, "
            f"occluded: {occluded_total}, visible: {visible_total}. "
            "Use this to decide which camera parameter to relax (range/FOV/max_incidence) if coverage is low."
        )
        if candidates_with_no_coverage == num_c:
            self.logger.error(
                "No candidate viewpoint covers any target point. Check camera FOV/range config, "
                "mesh scale, and viewpoint_distances -- coverage optimization will fail."
            )

        return coverage_matrix
