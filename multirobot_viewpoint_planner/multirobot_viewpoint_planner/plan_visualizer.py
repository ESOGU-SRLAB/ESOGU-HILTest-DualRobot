#!/usr/bin/env python3
"""
Static (offscreen) visualizer for a multirobot_viewpoint_plan.json.

Renders the chassis mesh plus BOTH arms' camera viewpoints -- the UR10e and the
Kawasaki, each a real coverage viewpoint with its own orientation -- into a
single PNG/JPEG. Same offscreen tool as the solo viewpoint_planner
plan_visualizer (matplotlib 'Agg' backend: no RViz, no display, works headless),
adapted to the two-arm multi-robot plan schema (`ur_viewpoints` +
`kawasaki_viewpoints`).

The two arms are told apart by colour family: UR10e in green->red by new-coverage
contribution (as the solo tool), Kawasaki in a blue gradient (dark = most, light
= least). Both are shown by default (in the multi-robot plan the Kawasaki entries
are REAL viewpoints, not the hidden mirror points of the solo plan).

By default it lays the scene out from FOUR angles (isometric + front + side +
top) in one 2x2 figure. Pass --single for one isometric view.

Usage:
    ros2 run multirobot_viewpoint_planner multirobot_plan_visualizer
    ros2 run multirobot_viewpoint_planner multirobot_plan_visualizer --single --show-triads
    python3 plan_visualizer.py --plan <ws>/src/multirobot_viewpoint_planner/plans/multirobot_viewpoint_plan.json --mesh /path/chassis.stl
"""
import argparse
import json
import os
import sys

import numpy as np

import matplotlib
matplotlib.use('Agg')  # offscreen: render straight to a file, no display needed
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


DEFAULT_OUTPUT = ('/home/cem/colcon_ws/src/multirobot_viewpoint_planner/figures/'
                  'multirobot_viewpoint_plan.png')
DEFAULT_PLAN = '/home/cem/colcon_ws/src/multirobot_viewpoint_planner/plans/multirobot_viewpoint_plan.json'
DEFAULT_MESH = ('/home/cem/colcon_ws/src/Universal_Robots_ROS2_Description/'
                'meshes/ur10e/collision/chassis.stl')

UR_CMAP = 'RdYlGn'    # UR contribution: green (most) -> red (least)
KAWA_CMAP = 'Blues'   # Kawasaki contribution: dark blue (most) -> light (least)
UR_SOLID = 'darkorange'
KAWA_SOLID = 'royalblue'

# Cache the (subsampled) chassis geometry so a multi-view figure loads the STL
# from disk only once instead of once per subplot.
_MESH_CACHE = {}


def load_plan(plan_path):
    with open(plan_path, 'r') as f:
        return json.load(f)


def _load_mesh_geometry(mesh_path, scale, max_faces):
    """Load + scale + subsample the chassis mesh once; return (triangles, centroid)."""
    key = (mesh_path, scale, max_faces)
    if key in _MESH_CACHE:
        return _MESH_CACHE[key]

    try:
        import trimesh
    except ImportError:
        print("[WARN] trimesh not available -- skipping chassis mesh, drawing waypoints only.",
              file=sys.stderr)
        _MESH_CACHE[key] = (None, None)
        return _MESH_CACHE[key]

    if not os.path.exists(mesh_path):
        print(f"[WARN] Mesh not found: {mesh_path} -- skipping chassis surface.", file=sys.stderr)
        _MESH_CACHE[key] = (None, None)
        return _MESH_CACHE[key]

    mesh = trimesh.load(mesh_path)
    mesh.apply_scale(scale)
    centroid = mesh.centroid.copy()

    faces = mesh.faces
    n_faces = len(faces)
    # matplotlib's 3D renderer bogs down on >~20k polygons, so plot every Nth
    # face when the mesh is dense. Collision meshes stay recognisable even so.
    if n_faces > max_faces:
        stride = int(np.ceil(n_faces / max_faces))
        faces = faces[::stride]
        print(f"[INFO] Mesh has {n_faces} faces; plotting every {stride}th "
              f"({len(faces)}) to keep rendering responsive.")

    triangles = mesh.vertices[faces]
    _MESH_CACHE[key] = (triangles, centroid)
    return _MESH_CACHE[key]


def add_mesh(ax, mesh_path, scale, max_faces, alpha):
    """Draw the chassis surface (subsampled for speed) and return its centroid."""
    triangles, centroid = _load_mesh_geometry(mesh_path, scale, max_faces)
    if triangles is None:
        return centroid
    # A Poly3DCollection can't be shared across axes, so build a fresh one per ax.
    coll = Poly3DCollection(triangles, alpha=alpha, facecolor='lightgray',
                            edgecolor='gray', linewidths=0.05)
    ax.add_collection3d(coll)
    return centroid


def draw_waypoint(ax, pos, rot, arrow_len, look_color, show_triads):
    """Draws one waypoint: an orientation arrow (line of sight = +Z), optionally
    the full X(red)/Y(green)/Z(blue) triad."""
    pos = np.asarray(pos, dtype=float)
    rot = np.asarray(rot, dtype=float)
    z_axis = rot[:, 2]

    ax.quiver(pos[0], pos[1], pos[2],
              z_axis[0], z_axis[1], z_axis[2],
              length=arrow_len, color=look_color, linewidth=1.5,
              arrow_length_ratio=0.3)

    if show_triads:
        for axis_idx, color in ((0, 'red'), (1, 'green'), (2, 'blue')):
            a = rot[:, axis_idx]
            ax.quiver(pos[0], pos[1], pos[2], a[0], a[1], a[2],
                      length=arrow_len * 0.6, color=color, linewidth=1.0,
                      arrow_length_ratio=0.3)


def set_equal_aspect(ax, all_points):
    """Force a 1:1:1 data aspect so orientations aren't visually distorted."""
    if len(all_points) == 0:
        return
    pts = np.asarray(all_points)
    mins = pts.min(axis=0)
    maxs = pts.max(axis=0)
    center = (mins + maxs) / 2.0
    span = (maxs - mins).max() / 2.0
    span = max(span, 1e-3)
    ax.set_xlim(center[0] - span, center[0] + span)
    ax.set_ylim(center[1] - span, center[1] + span)
    ax.set_zlim(center[2] - span, center[2] + span)
    if hasattr(ax, 'set_box_aspect'):
        ax.set_box_aspect((1, 1, 1))


def _draw_arm(ax, fig, vps, cmap_name, solid_color, args, add_colorbar, cbar_label, all_points):
    """Draw one arm's arrows + scatter, coloured by new-coverage contribution."""
    import matplotlib.cm as cm
    from matplotlib.colors import Normalize

    if not vps:
        return
    contribs = [vp.get('new_points_covered') for vp in vps
                if vp.get('new_points_covered') is not None]
    use_contrib = args.color_by_contribution and len(contribs) == len(vps) and bool(contribs)
    cmap = cm.get_cmap(cmap_name)
    max_contrib = max(contribs) if contribs else 0

    for vp in vps:
        pos = vp['position']
        all_points.append(pos)
        if use_contrib and max_contrib:
            color = cmap(0.25 + 0.75 * vp['new_points_covered'] / max_contrib)
        else:
            color = solid_color
        draw_waypoint(ax, pos, vp['rotation'], args.arrow_length, color, args.show_triads)
        rank = vp.get('rank')
        if args.label_top_n and rank is not None and rank < args.label_top_n:
            ax.text(pos[0], pos[1], pos[2] + 0.03, f"#{rank}", fontsize=7, color='black')

    arm_pos = np.array([vp['position'] for vp in vps])
    if use_contrib and max_contrib:
        c = [vp['new_points_covered'] for vp in vps]
        sc = ax.scatter(arm_pos[:, 0], arm_pos[:, 1], arm_pos[:, 2], c=c, cmap=cmap_name,
                        s=40, depthshade=False, norm=Normalize(vmin=0, vmax=max_contrib))
        if add_colorbar:
            cbar = fig.colorbar(sc, ax=ax, shrink=0.5, pad=0.08)
            cbar.set_label(cbar_label, fontsize=8)
    else:
        ax.scatter(arm_pos[:, 0], arm_pos[:, 1], arm_pos[:, 2],
                   color=solid_color, s=35, depthshade=False)


def draw_scene(fig, ax, plan, args, add_colorbar, include_ur=True, include_kawa=True):
    """Draw the chassis + both arms' viewpoints (with orientation) onto one 3D axis."""
    ur_vps = plan.get('ur_viewpoints', []) if include_ur else []
    kawa_vps = plan.get('kawasaki_viewpoints', []) if include_kawa else []
    all_points = []

    centroid = add_mesh(ax, args.mesh, args.scale, args.max_faces, args.mesh_alpha)
    if centroid is not None:
        all_points.append(centroid)
        ax.scatter(*centroid, color='black', marker='*', s=120, depthshade=False)

    _draw_arm(ax, fig, ur_vps, UR_CMAP, UR_SOLID, args,
              add_colorbar and include_ur, 'UR10e new points / viewpoint', all_points)
    _draw_arm(ax, fig, kawa_vps, KAWA_CMAP, KAWA_SOLID, args,
              add_colorbar and include_kawa, 'Kawasaki new points / viewpoint', all_points)

    set_equal_aspect(ax, all_points)
    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_zlabel('Z [m]')
    return all_points


def _legend_handles(ur_vps, kawa_vps, include_ur, include_kawa):
    handles = [
        Line2D([0], [0], marker='*', color='w', markerfacecolor='black',
               markersize=12, label='Chassis center'),
    ]
    if include_ur:
        handles.append(Line2D([0], [0], marker='o', color='w', markerfacecolor=UR_SOLID,
                              markersize=8, label=f'UR10e viewpoints ({len(ur_vps)})'))
    if include_kawa:
        handles.append(Line2D([0], [0], marker='o', color='w', markerfacecolor=KAWA_SOLID,
                              markersize=8, label=f'Kawasaki viewpoints ({len(kawa_vps)})'))
    handles.append(Line2D([0], [0], color='gray', lw=2, label='Line of sight (+Z)'))
    return handles


def _with_suffix(path, suffix):
    """Insert `suffix` before the file extension: foo.png -> foo<suffix>.png."""
    root, ext = os.path.splitext(path)
    return f"{root}{suffix}{ext}"


def render_figure(plan, args, output, include_ur, include_kawa):
    """Render ONE image (combined / UR-only / Kawasaki-only) to `output`."""
    ur_vps = plan.get('ur_viewpoints', [])
    kawa_vps = plan.get('kawasaki_viewpoints', [])

    triad_note = " (RGB triads = X/Y/Z)" if args.show_triads else " (arrow = line of sight)"
    parts = []
    if include_ur:
        parts.append(f"{len(ur_vps)} UR10e viewpoints")
    if include_kawa:
        parts.append(f"{len(kawa_vps)} Kawasaki viewpoints")
    coverage = plan.get('coverage_achieved')
    cov_str = f" | coverage {coverage * 100:.1f}%" if coverage is not None else ""
    who = "UR10e " if (include_ur and not include_kawa) else \
          "Kawasaki " if (include_kawa and not include_ur) else "Multi-robot "
    title = f"{who}inspection plan: {', '.join(parts)}{cov_str}{triad_note}"

    if args.multiview:
        views = [('Isometric', args.elev, args.azim),
                 ('Front (down -Y)', 5, -90),
                 ('Side (down +X)', 5, 0),
                 ('Top (down -Z)', 89, -90)]
        fig = plt.figure(figsize=(args.figsize * 1.25, args.figsize * 1.25))
        for i, (name, elev, azim) in enumerate(views, start=1):
            ax = fig.add_subplot(2, 2, i, projection='3d')
            draw_scene(fig, ax, plan, args, add_colorbar=(i == 1),
                       include_ur=include_ur, include_kawa=include_kawa)
            ax.view_init(elev=elev, azim=azim)
            ax.set_title(name, fontsize=10)
        fig.suptitle(title, fontsize=12)
    else:
        fig = plt.figure(figsize=(args.figsize, args.figsize))
        ax = fig.add_subplot(111, projection='3d')
        draw_scene(fig, ax, plan, args, add_colorbar=True,
                   include_ur=include_ur, include_kawa=include_kawa)
        ax.view_init(elev=args.elev, azim=args.azim)
        ax.set_title(title)

    fig.legend(handles=_legend_handles(ur_vps, kawa_vps, include_ur, include_kawa),
               loc='lower center', ncol=4, fontsize=8)

    out = os.path.expanduser(output)
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    fig.tight_layout(rect=(0, 0.03, 1, 0.97))
    fig.savefig(out, dpi=args.dpi)
    plt.close(fig)
    n_ur = len(ur_vps) if include_ur else 0
    n_kw = len(kawa_vps) if include_kawa else 0
    print(f"[OK] Saved visualization to {out} ({n_ur} UR viewpoints, {n_kw} Kawasaki viewpoints).")


def visualize(args):
    plan = load_plan(args.plan)
    base = os.path.expanduser(args.output)

    # Default 'all' -> combined view PLUS one figure per arm.
    targets = []
    if args.only in ('all', 'combined'):
        targets.append((base, True, True))
    if args.only in ('all', 'ur'):
        targets.append((_with_suffix(base, '_ur10e'), True, False))
    if args.only in ('all', 'kawasaki'):
        targets.append((_with_suffix(base, '_kawasaki'), False, True))

    for output, include_ur, include_kawa in targets:
        render_figure(plan, args, output, include_ur, include_kawa)


def build_parser():
    p = argparse.ArgumentParser(description="Render a multirobot_viewpoint_plan.json to a PNG/JPEG.")
    p.add_argument('--plan', default=DEFAULT_PLAN,
                   help='Path to multirobot_viewpoint_plan.json')
    p.add_argument('--mesh', default=DEFAULT_MESH, help='Path to chassis.stl')
    p.add_argument('--scale', type=float, default=0.001,
                   help='Mesh scale (must match the planner; default 0.001 mm->m)')
    p.add_argument('--output', default=DEFAULT_OUTPUT,
                   help='Base output image path (.png or .jpg/.jpeg). The per-arm '
                        'figures get _ur10e / _kawasaki inserted before the extension.')
    p.add_argument('--only', choices=('all', 'combined', 'ur', 'kawasaki'), default='all',
                   help="Which figure(s) to write: 'all' (default) -> combined + UR-only "
                        "+ Kawasaki-only; or just one of them.")
    p.add_argument('--single', dest='multiview', action='store_false',
                   help='Render one isometric view instead of the 2x2 multi-view')
    p.add_argument('--show-triads', action='store_true',
                   help='Draw full X/Y/Z orientation triads instead of just the line of sight')
    p.add_argument('--no-contribution-color', dest='color_by_contribution', action='store_false',
                   help='Disable colouring viewpoints by new-coverage contribution')
    p.add_argument('--label-top-n', type=int, default=15,
                   help='Annotate the N most informative viewpoints/arm with their rank (0 = none)')
    p.add_argument('--arrow-length', type=float, default=0.20, help='Orientation arrow length [m]')
    p.add_argument('--max-faces', type=int, default=20000,
                   help='Max mesh faces to render (subsampled above this)')
    p.add_argument('--mesh-alpha', type=float, default=0.25, help='Chassis surface transparency')
    p.add_argument('--dpi', type=int, default=150, help='Output image DPI')
    p.add_argument('--figsize', type=float, default=9.0, help='Figure size (inches, square base)')
    p.add_argument('--elev', type=float, default=25.0, help='Isometric view elevation angle')
    p.add_argument('--azim', type=float, default=-60.0, help='Isometric view azimuth angle')
    p.set_defaults(multiview=True, color_by_contribution=True)
    return p


def main(argv=None):
    args = build_parser().parse_args(argv)
    if not os.path.exists(os.path.expanduser(args.plan)):
        print(f"[ERROR] Plan file not found: {args.plan}. Generate it first via the "
              "multirobot_planner_node ~/plan service.", file=sys.stderr)
        return 1
    visualize(args)
    return 0


if __name__ == '__main__':
    sys.exit(main())
