#!/usr/bin/env python3
"""Sample the two car doors into "part" point clouds — the BELIEF MAP input.

The chassis octomap pipeline (pcd2octomap_builder) works in two stages:

  1. belief map  : chassis_pcd/pcds/PART*.pcd, one PCD per CAD part, each painted a
                   different colour and inserted as FREE. This is the ground truth of
                   "surface that exists and therefore ought to be seen".
  2. occupancy   : the captured sensor clouds are stamped onto that belief map; only
                   points that land on a belief voxel count, and per-part
                   occupied/belief voxel ratios become the coverage report.

The doors need exactly the same stage-1 input, but the chassis got 57 PCDs for free
because its CAD was delivered as 57 separate part STLs. The doors are two STLs, and a
two-line coverage report ("windowdoor 94%") says nothing about WHERE the miss is. So
each door is diced into a grid of patches over its two large axes (Y and Z), and each
patch becomes one "part". The report then reads like a map of the door, and octovis
shows a colour patchwork where an unseen patch stands out immediately.

Set patches_y = patches_z = 1 to get the literal one-part-per-door behaviour.

UNITS: the door STLs are in METRES (see merge_door_meshes), unlike chassis.stl which
is millimetres. The points are therefore written in metres and the octomap builder
must run with parts_scale=1.0. The chassis builder hard-codes a /1000 conversion —
that is exactly why the doors get their own builder.

The output is a build artefact (~700k points), so it lives in the same cache
directory as the merged mesh and is rebuilt whenever a source STL or the patch
layout changes.
"""
import argparse
import hashlib
import json
import os
import shutil
import sys

from doors_inspection.merge_door_meshes import DEFAULT_DOORS

DEFAULT_PARTS_DIR = os.path.expanduser("~/.cache/doors_inspection/door_parts")

# Points per square metre. The chassis parts were sampled at this same density, and at
# the 2 cm octomap resolution it puts ~40 samples in every voxel the surface touches —
# dense enough that no belief voxel is missed through unlucky sampling.
DEFAULT_DENSITY = 100_000

# Patch grid per door. 3 across (Y) x 4 up (Z) = 12 patches of roughly 40 x 38 cm on a
# 1.2 x 1.5 m door, which is the scale at which a missed viewpoint actually shows up.
DEFAULT_PATCHES_Y = 3
DEFAULT_PATCHES_Z = 4

MANIFEST = "parts_manifest.json"


def _log(msg):
    print(f"[doors_parts_prep] {msg}", flush=True)


def _config_id(sources, density, py, pz):
    """Fingerprint of everything that changes the output, so a patch-layout change
    invalidates the cache just like a modified STL does."""
    h = hashlib.sha1()
    for s in sources:
        mtime = os.path.getmtime(s) if os.path.exists(s) else 0.0
        h.update(f"{s}:{mtime:.3f}".encode())
    h.update(f"{density}:{py}:{pz}".encode())
    return h.hexdigest()[:16]


def is_stale(parts_dir, config_id):
    """True unless parts_dir holds a completed build of exactly this configuration."""
    try:
        with open(os.path.join(parts_dir, MANIFEST)) as f:
            return json.load(f).get("config_id") != config_id
    except (OSError, ValueError):
        return True


def write_pcd_binary(path, points):
    """Write an (N,3) float array as a binary PCD.

    Binary rather than the ASCII the chassis used: same 700k points go from ~20 MB of
    text to 8 MB, and PCL's loadPCDFile reads either without being told which.
    """
    import numpy as np

    pts = np.asarray(points, dtype=np.float32).reshape(-1, 3)
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z\n"
        "SIZE 4 4 4\n"
        "TYPE F F F\n"
        "COUNT 1 1 1\n"
        f"WIDTH {len(pts)}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {len(pts)}\n"
        "DATA binary\n"
    )
    with open(path, "wb") as f:
        f.write(header.encode("ascii"))
        f.write(pts.tobytes())


def build(sources=None, parts_dir=None, density=DEFAULT_DENSITY,
          patches_y=DEFAULT_PATCHES_Y, patches_z=DEFAULT_PATCHES_Z, force=False):
    """Sample every door into patch PCDs under `parts_dir`. Returns that directory."""
    import numpy as np

    sources = list(sources or DEFAULT_DOORS)
    parts_dir = parts_dir or DEFAULT_PARTS_DIR
    patches_y = max(1, int(patches_y))
    patches_z = max(1, int(patches_z))

    missing = [s for s in sources if not os.path.exists(s)]
    if missing:
        raise FileNotFoundError(f"door mesh(es) not found: {missing}")

    config_id = _config_id(sources, density, patches_y, patches_z)
    if not force and not is_stale(parts_dir, config_id):
        _log(f"up to date, reusing {parts_dir}")
        return parts_dir

    import trimesh  # lazy: a launch file should not pay for it when the cache is warm

    # Rebuild from empty. Leaving old patch files behind would silently keep parts of a
    # previous grid in the belief map, and the coverage report would count surface that
    # is no longer part of the current layout.
    if os.path.isdir(parts_dir):
        shutil.rmtree(parts_dir)
    os.makedirs(parts_dir, exist_ok=True)

    total_pts = 0
    total_area = 0.0
    written = []

    for src in sources:
        door = os.path.splitext(os.path.basename(src))[0]
        mesh = trimesh.load(src, force="mesh")
        if not hasattr(mesh, "faces") or len(mesh.faces) == 0:
            raise ValueError(f"mesh has no faces: {src}")

        n = max(10_000, int(mesh.area * density))
        pts, _ = trimesh.sample.sample_surface(mesh, n)
        pts = np.asarray(pts, dtype=np.float64)

        lo, hi = mesh.bounds
        # Equal-width bins over this door's own bounding box, so the patch grid follows
        # the door rather than the pair (the two doors sit at different Y).
        y_edges = np.linspace(lo[1], hi[1], patches_y + 1)
        z_edges = np.linspace(lo[2], hi[2], patches_z + 1)
        iy = np.clip(np.digitize(pts[:, 1], y_edges[1:-1]), 0, patches_y - 1)
        iz = np.clip(np.digitize(pts[:, 2], z_edges[1:-1]), 0, patches_z - 1)

        _log(f"{door}: faces={len(mesh.faces)} area={mesh.area:.3f} m^2 "
             f"-> {len(pts)} samples, {patches_y}x{patches_z} patches")
        total_area += float(mesh.area)

        for jy in range(patches_y):
            for jz in range(patches_z):
                sel = pts[(iy == jy) & (iz == jz)]
                if len(sel) == 0:
                    # A door outline is not a rectangle; corner patches can be empty.
                    continue
                # "<door>__y<j>z<j>" -- the builder groups its report by the text before
                # "__", which is what turns 24 patch lines into 2 per-door totals.
                name = f"{door}__y{jy}z{jz}.pcd"
                write_pcd_binary(os.path.join(parts_dir, name), sel)
                written.append(name)
                total_pts += len(sel)

    with open(os.path.join(parts_dir, MANIFEST), "w") as f:
        json.dump({
            "config_id": config_id,
            "sources": sources,
            "density": density,
            "patches_y": patches_y,
            "patches_z": patches_z,
            "parts": sorted(written),
            "total_points": total_pts,
            "total_area_m2": round(total_area, 4),
            "units": "metres (build the octomap with parts_scale=1.0)",
        }, f, indent=2)

    _log(f"wrote {len(written)} part PCDs, {total_pts} points, {total_area:.3f} m^2 "
         f"-> {parts_dir}")
    return parts_dir


def ensure_parts(sources=None, parts_dir=None, density=DEFAULT_DENSITY,
                 patches_y=DEFAULT_PATCHES_Y, patches_z=DEFAULT_PATCHES_Z, force=False):
    """build() that never raises, so a launch file can proceed and let the builder
    report the empty parts directory with its own error message."""
    try:
        return build(sources, parts_dir, density, patches_y, patches_z, force)
    except Exception as e:                                   # noqa: BLE001
        _log(f"ERROR: {e}")
        return parts_dir or DEFAULT_PARTS_DIR


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--door", action="append", dest="doors", default=None,
                   help="source STL (repeatable); defaults to windowdoor + windowlessdoor")
    p.add_argument("--parts_dir", default=DEFAULT_PARTS_DIR,
                   help="where the patch PCDs are written")
    p.add_argument("--density", type=int, default=DEFAULT_DENSITY,
                   help="surface samples per square metre")
    p.add_argument("--patches_y", type=int, default=DEFAULT_PATCHES_Y,
                   help="patches across the door (1 = whole door is one part)")
    p.add_argument("--patches_z", type=int, default=DEFAULT_PATCHES_Z,
                   help="patches up the door (1 = whole door is one part)")
    p.add_argument("--force", action="store_true", help="rebuild even if up to date")
    a = p.parse_args(argv if argv is not None else sys.argv[1:])
    build(a.doors, a.parts_dir, a.density, a.patches_y, a.patches_z, a.force)
    return 0


if __name__ == "__main__":
    sys.exit(main())
