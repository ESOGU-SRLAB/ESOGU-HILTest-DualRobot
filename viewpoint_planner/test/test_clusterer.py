import numpy as np

from viewpoint_planner.viewpoint_clusterer import ViewpointClusterer


def _look(z):
    z = np.array(z, float)
    z /= np.linalg.norm(z)
    up = np.array([0, 0, 1.0])
    if abs(np.dot(z, up)) > 0.99:
        up = np.array([1.0, 0, 0])
    x = np.cross(up, z)
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    return np.column_stack((x, y, z))


class _FakeGen:
    """Merged averaged pose is declared to see targets 0..6."""
    def __init__(self, mask):
        self._mask = mask

    def visible_mask(self, pos, rot, tp, tn, inter):
        return self._mask.copy()


def _base_case():
    vps = [
        {'position': np.array([0.0, 0, 0.0]), 'rotation': _look([1, 0, 0]), 'joint_solution': 'J0'},
        {'position': np.array([0.1, 0.05, 0.0]), 'rotation': _look([1, 0.1, 0]), 'joint_solution': 'J1'},
        {'position': np.array([2.0, 0, 0]), 'rotation': _look([1, 0, 0]), 'joint_solution': 'J2'},
        {'position': np.array([0.12, 0, 0]), 'rotation': _look([0, 1, 0]), 'joint_solution': 'J3'},
    ]
    masks = [np.zeros(10, bool) for _ in range(4)]
    masks[0][0:5] = True
    masks[1][3:7] = True
    masks[2][7:10] = True
    masks[3][5:8] = True
    return vps, masks


def test_merges_near_and_similar_only():
    vps, masks = _base_case()
    gen = _FakeGen(np.array([True] * 7 + [False] * 3))
    cl = ViewpointClusterer(position_thresh=0.25, orientation_thresh_deg=25, min_coverage_ratio=0.9)
    merged, mmasks = cl.cluster_and_merge(vps, masks, gen, np.zeros((10, 3)), np.zeros((10, 3)), None,
                                          reachability_fn=lambda c: (True, 'Javg'))
    # vp0+vp1 merge; vp2 alone; vp3 (different look) alone -> 3 total.
    assert len(merged) == 3
    cov = np.zeros(10, bool)
    for m in mmasks:
        cov |= m
    assert cov.sum() == 10  # coverage preserved


def test_falls_back_when_merged_pose_unreachable():
    vps, masks = _base_case()
    gen = _FakeGen(np.array([True] * 7 + [False] * 3))
    cl = ViewpointClusterer(position_thresh=0.25, orientation_thresh_deg=25, min_coverage_ratio=0.9)
    # IK always fails -> merged averaged pose rejected, cluster keeps its best member.
    merged, mmasks = cl.cluster_and_merge(vps, masks, gen, np.zeros((10, 3)), np.zeros((10, 3)), None,
                                          reachability_fn=lambda c: (False, None))
    # The A-cluster keeps one real member (its best), so still 3 viewpoints but
    # every kept viewpoint is a real one with an original joint solution.
    assert len(merged) == 3
    assert all(vp.get('joint_solution') in ('J0', 'J1', 'J2', 'J3') for vp in merged)


def test_falls_back_when_merged_pose_loses_coverage():
    vps, masks = _base_case()
    # Averaged pose only sees 2 of the cluster's 7 targets -> below ratio -> fallback.
    gen = _FakeGen(np.array([True, True] + [False] * 8))
    cl = ViewpointClusterer(position_thresh=0.25, orientation_thresh_deg=25, min_coverage_ratio=0.9)
    merged, _ = cl.cluster_and_merge(vps, masks, gen, np.zeros((10, 3)), np.zeros((10, 3)), None,
                                     reachability_fn=lambda c: (True, 'Javg'))
    assert all(vp.get('joint_solution') in ('J0', 'J1', 'J2', 'J3') for vp in merged)


def test_no_merge_below_two():
    cl = ViewpointClusterer()
    vps = [{'position': np.zeros(3), 'rotation': np.eye(3)}]
    merged, masks = cl.cluster_and_merge(vps, [np.ones(5, bool)], None, np.zeros((5, 3)), np.zeros((5, 3)), None, None)
    assert len(merged) == 1
