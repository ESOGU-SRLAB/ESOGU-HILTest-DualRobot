import numpy as np
from viewpoint_planner.set_cover_optimizer import SetCoverOptimizer

def test_set_cover_optimizer():
    optimizer = SetCoverOptimizer(coverage_threshold=1.0)
    
    candidates = [
        {'position': np.array([0, 0, 0])},
        {'position': np.array([1, 0, 0])},
        {'position': np.array([2, 0, 0])}
    ]
    
    # 3 candidates, 5 targets
    # Candidate 0 covers targets 0, 1
    # Candidate 1 covers targets 2, 3
    # Candidate 2 covers target 4
    coverage_matrix = np.array([
        [True, True, False, False, False],
        [False, False, True, True, False],
        [False, False, False, False, True]
    ])
    
    selected_cands, selected_indices, final_coverage = optimizer.optimize(candidates, coverage_matrix)
    
    assert len(selected_cands) == 3
    assert set(selected_indices) == {0, 1, 2}
    assert final_coverage == 1.0
