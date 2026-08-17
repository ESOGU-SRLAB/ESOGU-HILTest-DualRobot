# doors_inspection

Cooperative UR10e + Kawasaki coverage inspection of the two car doors
(`windowdoor.stl`, `windowlessdoor.stl`) instead of the chassis.

## What is new here, and what is not

Nothing from the validated pipeline is forked. `multirobot_planner_node`,
`ur_inspection_node` and `kawasaki_inspection_node` are already fully parameterised, so
this package runs **those same executables** with door-specific parameters. Fixes made
for the chassis job (trajectory cache validation, 2*pi goal unwinding, nearest-branch
IK) therefore apply to the doors job automatically.

New code, all of it door-specific:

| file | why it exists |
|---|---|
| `merge_door_meshes.py` | the planner analyses exactly ONE mesh, and the doors are wanted as one combined tour, so the two STLs are concatenated. That also makes occlusion correct — in one mesh each door blocks rays to the other, as the real pair does. |
| `door_parts.py` | dices each door into a grid of patch PCDs: the belief-map ground truth for the octomap stage. |
| `src/build_doors_octomap.cpp` | belief + occupancy octomap and the coverage report, i.e. `pcd2octomap_builder` adapted to the doors. |

The package is therefore **hybrid** (`ament_cmake` + `ament_cmake_python`): the octomap
builder must link the real octomap library, and there are no usable Python bindings for
`ColorOcTree` serialisation here. The Python module and the launch files install exactly
as they did before; `setup.py` is gone because ament_cmake ignores it, and the two
console entry points now live in `scripts/`.

## Facts about the door meshes

| | `windowdoor` | `windowlessdoor` |
|---|---|---|
| faces | 773 109 | 129 462 |
| area | 2.880 m² | 3.941 m² |
| x | −1.211 … −0.794 | −1.210 … −0.806 |
| y | +1.301 … +2.478 | +0.088 … +1.319 |
| z | +0.204 … +1.708 | +0.201 … +1.706 |

Merged: 902 571 faces, **6.821 m²** (the chassis is 31.97 m², so ~1/5).

**Units:** the door STLs are already in **metres** (the URDF loads them with
`scale="1 1 1"`), unlike `chassis.stl` which is millimetres. The planning launch sets
`mesh_scale:=1.0`. Passing `0.001` would shrink the doors to a 2 mm speck and every
viewpoint would come back unreachable.

**Both faces** are inspected. Each door carries near-equal +X and −X area (48.8 % /
48.8 % on the windowless one), and the two arms sit on opposite sides — the UR rail at
x ≈ −0.16, the Kawasaki AGV rail at x ≈ −2.2, the doors between them at x ≈ −1.2 … −0.79.
So the +X faces fall to the UR and the −X faces to the Kawasaki by geometry; the
allocator resolves it from reachability, with no hand-written side rule.

## Use

```bash
colcon build --packages-select doors_inspection && source install/setup.bash

# 1) plan (merges the meshes first, then waits for the service call)
ros2 launch doors_inspection doors_planning.launch.py
# The service is the node's own '~/plan', so it follows the node name set in the
# launch file. Beware of stale '/plan_multirobot_viewpoints' entries left in the
# discovery graph by a killed run: calling those just blocks forever.
ros2 service call /doors_planner_node/plan std_srvs/srv/Trigger

# 2) execute on both arms
ros2 launch doors_inspection doors_inspection.launch.py
ros2 launch doors_inspection doors_inspection.launch.py only_sim:=false
ros2 launch doors_inspection doors_inspection.launch.py force_replan:=true

# 3) build the octomap of what was actually captured
ros2 launch doors_inspection doors_octomap.launch.py            # sim captures
ros2 launch doors_inspection doors_octomap.launch.py mode:=real
octovis ~/colcon_ws/src/pcds/doors/sim_pcds/occupancyMap_doors_sim.ot
```

Paths follow the plan file, so the doors job never collides with the chassis job:

* plan → `doors_inspection/plans/doors_viewpoint_plan.json`
* trajectory cache → `doors_inspection/plans/trajectories/`
* clouds → `~/colcon_ws/src/pcds/doors/{sim,real}_pcds/{ur,kawasaki}_data/`
* merged mesh (45 MB build artefact, not source) → `~/.cache/doors_inspection/doors_merged.stl`,
  rebuilt whenever a source STL is newer. Force it with `force_merge:=true`, or run
  `ros2 run doors_inspection doors_mesh_prep --force`.
* belief-map patches (8 MB build artefact) → `~/.cache/doors_inspection/door_parts/`,
  rebuilt whenever a source STL **or the patch grid** changes. `force_parts:=true`, or
  `ros2 run doors_inspection doors_parts_prep --force`.
* octomaps → `<capture dir>/{belief,occupancy}Map_doors_<mode>.ot`, next to the clouds
  they were built from, so a sim map never overwrites a real one.

## Octomap stage

Same two-stage recipe as the chassis (`pcd2octomap_builder/build_octomap`): a belief
map of surface that *ought* to be seen, then the captured clouds stamped onto it, and
per-part occupied/belief voxel ratios as the coverage report. `insertRay` is not used —
carving free space along the ray would destroy the belief map — only endpoints are
marked occupied, keeping the belief colour.

Three things forced a door-specific builder rather than reuse of the chassis one:

1. **Units.** The chassis builder hard-codes a `/1000` because its CAD parts are
   millimetres. The doors are metres, so scaling is the `--parts_scale` parameter and
   defaults to `1.0` here. Running the chassis builder on the doors would shrink them to
   a 1.2 mm speck and the report would read 0 %.
2. **Data root.** `~/colcon_ws/src/pcds/doors/{sim,real}_pcds`, never the chassis tree.
3. **Report granularity.** The chassis got 57 part PCDs for free from its CAD. The doors
   are two STLs, and "windowdoor 94 %" says nothing about *where* the miss is — so
   `door_parts.py` dices each door into a 3×4 patch grid (`patches_y`, `patches_z`;
   `1×1` gives literal one-part-per-door). The builder groups patches by the name before
   `__` and prints patch lines, a per-door total and an overall total. octovis then shows
   a colour patchwork where an unseen patch is obvious.

Measured on the 13 sim captures currently on disk, at the default 2 cm:

```
windowdoor      94.20 %  (8094 / 8592 voxel)
windowlessdoor  93.81 %  (5960 / 6353 voxel)
KAPILAR TOPLAM  94.04 %  (14054 / 14945 voxel)      run time 0.4 s
```

Two caveats worth knowing before reading that number:

* **`windowlessdoor` is a zero-thickness double-sided shell** (trimesh: volume 0.0, +X
  area 1.951 m² = −X area 1.951 m²). Its two faces occupy the *same* voxel at any
  resolution, so a voxel counts as covered when **either** side was seen. `windowdoor`
  is a real solid (volume 0.084 m³, ≈ 6 cm thick), so its two faces do land in different
  voxels. Octomap coverage is therefore not the same quantity as the planner's
  face-level coverage, and reads higher on the windowless door.
* **`snap:=true`** lets a captured point one voxel off the surface still count
  (26-neighbour search). It rescues real-data hand-eye drift, but flatters the number,
  so it is off by default — on the sim data it moves the total by only +0.09 %.

## Scene

The doors and the platform they hang on must be the only workpiece between the two
arms — the chassis is commented out of the URDF separately. This package does not touch
any URDF, mesh or SRDF.
