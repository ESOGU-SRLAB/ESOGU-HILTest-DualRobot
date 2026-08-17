#!/usr/bin/env python3
"""Merge the car-door STLs into ONE target mesh for the coverage planner.

Why a merged file instead of a new planner: `multirobot_planner_node` is already
mesh-agnostic (`mesh_path` / `mesh_scale` are parameters) but it analyses exactly ONE
mesh. The doors job wants a SINGLE combined plan over both doors, so the cheapest
honest way to get there is to hand it one mesh that already contains both. Nothing in
viewpoint_planner or multirobot_viewpoint_planner has to change, which also means the
chassis pipeline stays byte-for-byte the one that has been validated on hardware.

Concatenating (rather than planning per door) is also what makes occlusion correct:
in one mesh each door blocks rays to the other, exactly as the real pair does.

UNITS: the door STLs are already in METRES (the URDF loads them with
scale="1 1 1", unlike chassis.stl which is millimetres). They are written out
unchanged, so the planner must run with mesh_scale=1.0. Passing 0.001 here would
shrink the doors to a 2 mm speck and every viewpoint would be unreachable.

The merged file is a build artefact, not source: it is ~45 MB (the two doors carry
~900k triangles between them), so it is written to a cache directory outside the
repo and rebuilt automatically whenever a source STL is newer.
"""
import argparse
import os
import sys

DESC_PKG = "/home/ifarlab/colcon_ws/src/Universal_Robots_ROS2_Description"
DOOR_MESH_DIR = os.path.join(DESC_PKG, "meshes/ur10e/collision")

DEFAULT_DOORS = [
    os.path.join(DOOR_MESH_DIR, "windowdoor.stl"),
    os.path.join(DOOR_MESH_DIR, "windowlessdoor.stl"),
]
DEFAULT_OUTPUT = os.path.expanduser("~/.cache/doors_inspection/doors_merged.stl")


def _log(msg):
    print(f"[doors_mesh_prep] {msg}", flush=True)


def is_stale(output, sources):
    """True if `output` must be rebuilt: missing, or older than any source."""
    if not os.path.exists(output):
        return True
    out_mtime = os.path.getmtime(output)
    return any(os.path.getmtime(s) > out_mtime for s in sources if os.path.exists(s))


def merge(sources=None, output=None, force=False):
    """Concatenate the door meshes into `output`. Returns the output path.

    No transform is applied: each STL already carries its own world placement (the
    URDF joints that hold the doors are identity), so concatenation alone puts both
    doors where they really stand relative to each other and to the two arms.
    """
    sources = list(sources or DEFAULT_DOORS)
    output = output or DEFAULT_OUTPUT

    missing = [s for s in sources if not os.path.exists(s)]
    if missing:
        raise FileNotFoundError(f"door mesh(es) not found: {missing}")

    if not force and not is_stale(output, sources):
        _log(f"up to date, reusing {output}")
        return output

    import trimesh  # imported lazily: launch files should not pay for it on every run

    parts = []
    for s in sources:
        m = trimesh.load(s, force="mesh")
        if not hasattr(m, "faces") or len(m.faces) == 0:
            raise ValueError(f"mesh has no faces: {s}")
        lo, hi = m.bounds
        _log(f"{os.path.basename(s):22s} faces={len(m.faces):7d} area={m.area:6.3f} m^2 "
             f"bounds x[{lo[0]:+.3f},{hi[0]:+.3f}] y[{lo[1]:+.3f},{hi[1]:+.3f}] "
             f"z[{lo[2]:+.3f},{hi[2]:+.3f}]")
        parts.append(m)

    merged = trimesh.util.concatenate(parts)
    lo, hi = merged.bounds
    _log(f"merged: faces={len(merged.faces)} area={merged.area:.3f} m^2 "
         f"bounds x[{lo[0]:+.3f},{hi[0]:+.3f}] y[{lo[1]:+.3f},{hi[1]:+.3f}] "
         f"z[{lo[2]:+.3f},{hi[2]:+.3f}] (METRES; run the planner with mesh_scale=1.0)")

    os.makedirs(os.path.dirname(os.path.abspath(output)), exist_ok=True)
    tmp = output + ".tmp"
    merged.export(tmp, file_type="stl")
    os.replace(tmp, output)
    _log(f"wrote {output} ({os.path.getsize(output) / 1e6:.1f} MB)")
    return output


def ensure_merged(sources=None, output=None, force=False):
    """merge() that never raises; returns the path anyway so launch can proceed and
    the planner reports the missing mesh itself with its own error message."""
    try:
        return merge(sources, output, force)
    except Exception as e:                                   # noqa: BLE001
        _log(f"ERROR: {e}")
        return output or DEFAULT_OUTPUT


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--door", action="append", dest="doors", default=None,
                   help="source STL (repeatable); defaults to windowdoor + windowlessdoor")
    p.add_argument("--output", default=DEFAULT_OUTPUT, help="merged STL path")
    p.add_argument("--force", action="store_true", help="rebuild even if up to date")
    args = p.parse_args(argv if argv is not None else sys.argv[1:])
    merge(args.doors, args.output, args.force)
    return 0


if __name__ == "__main__":
    sys.exit(main())
