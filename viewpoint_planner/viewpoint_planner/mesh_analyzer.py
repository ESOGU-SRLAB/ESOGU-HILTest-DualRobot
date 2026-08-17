import os
import logging
import trimesh
import numpy as np


class MeshAnalyzer:
    """
    Analyzes the chassis STL file to extract surface points and normals
    for inspection viewpoint planning.
    """
    def __init__(self, mesh_path, scale=0.001, logger=None):
        self.mesh_path = mesh_path
        self.scale = scale
        self.mesh = None
        # Allow an rclpy node logger to be injected so log records show up
        # under the owning node's name/output. Falls back to the standard
        # `logging` module for standalone/CLI/test usage.
        self.logger = logger or logging.getLogger(__name__)

    def load_mesh(self):
        if not os.path.exists(self.mesh_path):
            self.logger.error(f"Mesh file not found: {self.mesh_path}")
            raise FileNotFoundError(f"Mesh file not found: {self.mesh_path}")

        self.logger.info(f"Loading mesh from {self.mesh_path} (scale={self.scale})...")
        self.mesh = trimesh.load(self.mesh_path)

        if not hasattr(self.mesh, "faces") or len(self.mesh.faces) == 0:
            self.logger.error(
                f"Loaded mesh has no faces. File may be empty/corrupt or not a surface mesh: {self.mesh_path}"
            )
            raise ValueError(f"Mesh has no faces: {self.mesh_path}")

        # Apply scale (typically mm to m)
        self.mesh.apply_scale(self.scale)
        self.logger.info(
            f"Loaded mesh with {len(self.mesh.faces)} faces and {len(self.mesh.vertices)} vertices "
            f"(bounds after scaling: min={self.mesh.bounds[0].tolist()}, max={self.mesh.bounds[1].tolist()}, "
            f"extents={self.mesh.extents.tolist()})."
        )

        if not self.mesh.is_watertight:
            self.logger.warning(
                "Mesh is not watertight. Surface sampling and normals may be unreliable "
                "at open edges; consider repairing chassis.stl."
            )

    def sample_surface(self, num_points=15000, min_normal_z=-0.95):
        """
        Samples the surface of the mesh to get target points for inspection.
        Filters out points pointing strictly downwards (e.g., bottom surface).
        """
        if self.mesh is None:
            self.load_mesh()

        self.logger.info(f"Sampling {num_points} points from the surface...")
        # Use trimesh to sample points on the surface uniformly
        points, face_indices = trimesh.sample.sample_surface(self.mesh, num_points)

        # Get face normals for the sampled points
        normals = self.mesh.face_normals[face_indices]

        # Filter out points facing downwards based on min_normal_z
        # If normal_z is very negative, it means it's facing down (e.g. bottom of chassis).
        valid_mask = normals[:, 2] >= min_normal_z

        points = points[valid_mask]
        normals = normals[valid_mask]
        face_indices = face_indices[valid_mask]

        dropped = num_points - len(points)
        self.logger.info(
            f"Retained {len(points)} points after filtering downward-facing surfaces "
            f"(dropped {dropped}, min_normal_z={min_normal_z})."
        )

        if len(points) == 0:
            self.logger.error(
                "All sampled points were filtered out. Check min_normal_z threshold and "
                "mesh orientation (normals may be inverted)."
            )
        elif len(points) < 0.1 * num_points:
            self.logger.warning(
                f"Only {len(points)}/{num_points} points retained after filtering. "
                "Coverage planning may under-sample the chassis; verify mesh normals/orientation."
            )

        return points, normals, face_indices

    def analyze_and_save(self, output_path, num_points=15000, min_normal_z=-0.95):
        points, normals, face_indices = self.sample_surface(num_points, min_normal_z)

        bounds = self.mesh.bounds
        mesh_bounds = {
            'min': bounds[0],
            'max': bounds[1],
            'center': self.mesh.centroid,
            'extents': self.mesh.extents
        }

        # Create output directory if it doesn't exist
        os.makedirs(os.path.dirname(os.path.abspath(output_path)), exist_ok=True)

        # Save to npz
        np.savez(
            output_path,
            points=points,
            normals=normals,
            face_indices=face_indices,
            mesh_bounds_min=mesh_bounds['min'],
            mesh_bounds_max=mesh_bounds['max'],
            mesh_center=mesh_bounds['center'],
            mesh_extents=mesh_bounds['extents']
        )
        self.logger.info(f"Saved analysis results to {output_path}")
        return points, normals


if __name__ == '__main__':
    import argparse
    logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(name)s: %(message)s")

    parser = argparse.ArgumentParser(description="Mesh Analyzer for Viewpoint Planning")
    parser.add_argument('--mesh', type=str, required=True, help="Path to chassis.stl")
    parser.add_argument('--output', type=str, default='chassis_analysis.npz', help="Output npz file path")
    parser.add_argument('--samples', type=int, default=15000, help="Number of points to sample")
    parser.add_argument('--scale', type=float, default=0.001, help="Scale factor (default 0.001 for mm to m)")

    args = parser.parse_args()

    analyzer = MeshAnalyzer(args.mesh, scale=args.scale)
    analyzer.analyze_and_save(args.output, num_points=args.samples)
