import os
import pytest
import numpy as np
from viewpoint_planner.mesh_analyzer import MeshAnalyzer

def test_mesh_analyzer_initialization():
    analyzer = MeshAnalyzer("dummy_path.stl", scale=0.001)
    assert analyzer.mesh_path == "dummy_path.stl"
    assert analyzer.scale == 0.001
    assert analyzer.mesh is None

def test_load_nonexistent_mesh():
    analyzer = MeshAnalyzer("nonexistent.stl")
    with pytest.raises(FileNotFoundError):
        analyzer.load_mesh()

def test_sample_surface(tmp_path):
    import trimesh
    # Create a simple box mesh
    mesh = trimesh.creation.box(extents=(1000, 1000, 1000)) # 1m box in mm
    mesh_path = tmp_path / "test_box.stl"
    mesh.export(str(mesh_path))
    
    analyzer = MeshAnalyzer(str(mesh_path), scale=0.001)
    # 1000 points
    points, normals, face_indices = analyzer.sample_surface(num_points=1000, min_normal_z=-0.9)
    
    assert points is not None
    assert normals is not None
    assert len(points) == len(normals)
    assert len(points) > 0
    # The box has some downward faces (z normal = -1), they should be filtered out
    assert np.all(normals[:, 2] >= -0.9)
