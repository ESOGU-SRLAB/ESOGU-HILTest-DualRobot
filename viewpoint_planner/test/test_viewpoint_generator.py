import pytest
import numpy as np
from viewpoint_planner.viewpoint_generator import ViewpointGenerator

class MockMeshAnalyzer:
    def __init__(self):
        self.mesh = None
        
def test_viewpoint_generator_initialization():
    vg = ViewpointGenerator(MockMeshAnalyzer())
    # Default FOV matches the SICK Visionary-T Mini V3S145-1AAAAAA datasheet (70 x 60 deg),
    # which also matches the Gazebo rgbd_camera sensor defined in simrobot_ur_macro.xacro.
    assert vg.config['horizontal_fov_deg'] == 70.0
    assert vg.config['vertical_fov_deg'] == 60.0
    
def test_generate_candidates():
    vg = ViewpointGenerator(MockMeshAnalyzer())
    target_points = np.array([[0, 0, 0]])
    target_normals = np.array([[0, 0, 1]])
    
    candidates = vg.generate_candidates(target_points, target_normals)
    # distances: 0.3, 0.4, 0.5 (3 distances)
    # tilts: -15, 0, 15 (3 tilts)
    # Total candidates = 1 * 3 * 3 = 9
    assert len(candidates) == 9
    
    # Check if candidate position is along the normal
    pos0 = candidates[0]['position']
    assert np.allclose(pos0, np.array([0, 0, 0.3]) )
