import logging
import numpy as np
from scipy.spatial.transform import Rotation


class ViewpointClusterer:
    """
    Reduces the number of selected viewpoints by clustering ones that are close
    in BOTH position and viewing direction, then replacing each cluster with a
    single averaged pose.

    Crucially, an averaged pose is *synthetic*: it may be unreachable or may not
    actually see what the cluster's members saw. So every merged pose is
    re-verified (recompute its visible target set + IK reachability); if it fails
    either test the cluster falls back to keeping its single best real member.
    This guarantees the merge never silently drops coverage below the accepted
    ratio and never emits an unreachable waypoint.
    """
    def __init__(self, position_thresh=0.25, orientation_thresh_deg=25.0,
                 min_coverage_ratio=0.9, logger=None):
        self.position_thresh = position_thresh
        self.orientation_thresh = np.deg2rad(orientation_thresh_deg)
        self.min_coverage_ratio = min_coverage_ratio
        self.logger = logger or logging.getLogger(__name__)

    @staticmethod
    def _look_axis(rot):
        return np.asarray(rot, dtype=np.float64)[:, 2]

    def _cluster(self, viewpoints):
        """Greedy single-pass clustering: seed with the highest-coverage
        viewpoint, absorb every not-yet-assigned viewpoint within both
        thresholds. `viewpoints` must already be ordered by descending coverage
        so seeds are the most informative."""
        n = len(viewpoints)
        assigned = [False] * n
        clusters = []
        for i in range(n):
            if assigned[i]:
                continue
            assigned[i] = True
            cluster = [i]
            pos_i = np.asarray(viewpoints[i]['position'], dtype=np.float64)
            look_i = self._look_axis(viewpoints[i]['rotation'])
            for j in range(i + 1, n):
                if assigned[j]:
                    continue
                pos_j = np.asarray(viewpoints[j]['position'], dtype=np.float64)
                if np.linalg.norm(pos_i - pos_j) > self.position_thresh:
                    continue
                cos_ang = np.clip(np.dot(look_i, self._look_axis(viewpoints[j]['rotation'])), -1.0, 1.0)
                if np.arccos(cos_ang) > self.orientation_thresh:
                    continue
                assigned[j] = True
                cluster.append(j)
            clusters.append(cluster)
        return clusters

    def cluster_and_merge(self, viewpoints, coverage_masks, generator,
                          target_points, target_normals, intersector, reachability_fn):
        """
        viewpoints: list of dicts with 'position', 'rotation' (and optionally
            'joint_solution'); coverage_masks[i] is the boolean visibility mask
            of viewpoints[i] over target_points.
        reachability_fn: candidate -> (reachable, joint_solution), or None to
            skip IK re-verification (averaged poses accepted on coverage alone).

        Returns (merged_viewpoints, merged_masks). Each merged viewpoint carries
        its (possibly new) 'joint_solution' and matching mask so the caller can
        recompute exact final coverage.
        """
        n = len(viewpoints)
        if n <= 1:
            return list(viewpoints), list(coverage_masks)

        order = sorted(range(n), key=lambda i: -int(coverage_masks[i].sum()))
        ordered_vps = [viewpoints[i] for i in order]
        ordered_masks = [coverage_masks[i] for i in order]

        clusters = self._cluster(ordered_vps)

        merged_vps = []
        merged_masks = []
        merged_by_average = 0
        fell_back_coverage = 0
        fell_back_ik = 0

        for cluster in clusters:
            if len(cluster) == 1:
                merged_vps.append(ordered_vps[cluster[0]])
                merged_masks.append(ordered_masks[cluster[0]])
                continue

            union = np.zeros(len(target_points), dtype=bool)
            for idx in cluster:
                union |= ordered_masks[idx]
            union_count = int(union.sum())

            avg_pos = np.mean([np.asarray(ordered_vps[i]['position'], dtype=np.float64)
                               for i in cluster], axis=0)
            avg_rot = Rotation.from_matrix(
                [np.asarray(ordered_vps[i]['rotation'], dtype=np.float64) for i in cluster]
            ).mean().as_matrix()

            merged_mask = generator.visible_mask(
                avg_pos, avg_rot, target_points, target_normals, intersector)
            keeps_enough = union_count == 0 or int(merged_mask.sum()) >= self.min_coverage_ratio * union_count

            reachable, joint = True, None
            if keeps_enough and reachability_fn is not None:
                reachable, joint = reachability_fn({'position': avg_pos, 'rotation': avg_rot})

            if keeps_enough and reachable:
                merged_vps.append({'position': avg_pos, 'rotation': avg_rot, 'joint_solution': joint})
                merged_masks.append(merged_mask)
                merged_by_average += 1
            else:
                # Fall back to the cluster's best (highest-coverage) real member.
                # Track *why* the average was rejected so the caller can tell a
                # coverage problem (relax min_coverage_ratio) from an IK problem
                # (averaged pose lands in an unreachable gap -- averaging simply
                # cannot help in a reachability-sparse cell).
                if not keeps_enough:
                    fell_back_coverage += 1
                else:
                    fell_back_ik += 1
                best = cluster[0]  # ordered_masks is coverage-desc, so first is best
                merged_vps.append(ordered_vps[best])
                merged_masks.append(ordered_masks[best])

        num_t = len(target_points)
        cov_before = np.zeros(num_t, dtype=bool)
        for m in ordered_masks:
            cov_before |= m
        cov_after = np.zeros(num_t, dtype=bool)
        for m in merged_masks:
            cov_after |= m

        self.logger.info(
            f"Pose clustering: {n} -> {len(merged_vps)} viewpoints "
            f"({len(clusters)} clusters; {merged_by_average} merged to an averaged pose, "
            f"fallback: {fell_back_coverage} because the averaged pose lost coverage, "
            f"{fell_back_ik} because it was IK-unreachable). "
            f"Coverage {cov_before.sum() / num_t * 100:.2f}% -> {cov_after.sum() / num_t * 100:.2f}% "
            f"(pos_thresh={self.position_thresh} m, "
            f"orient_thresh={np.rad2deg(self.orientation_thresh):.0f} deg, "
            f"min_coverage_ratio={self.min_coverage_ratio})."
        )
        return merged_vps, merged_masks
