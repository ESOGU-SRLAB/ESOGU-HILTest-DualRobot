# pymoveit2

Basic Python interface for MoveIt 2 built on top of ROS 2 actions and services.

> Note: The official Python library for MoveIt 2 `moveit_py` is now available. Check the announcement [here](https://picknik.ai/moveit/ros/python/google/2023/04/28/GSOC-MoveIt-2-Python-Bindings.html)!

> This is a local fork (`pymoveit2_real`) with cell-specific additions on top of upstream.
> The one behaviour change that is **on by default** is documented in
> [2π Joint-Goal Unwinding](#2π-joint-goal-unwinding-local-addition) below — read it before
> debugging why a goal angle does not match the number you passed in.

## 2π Joint-Goal Unwinding (local addition)

### What it does

Every joint goal is re-expressed as the **2π-equivalent nearest the state the motion starts
from**, subject to that joint's URDF limits.

For a revolute joint, `θ` and `θ ± 2π` are the *same physical configuration* — identical
link poses, so identical collisions and identical tool orientation — but very different
numbers to travel to. IK returns one arbitrary branch of the solution family, so a goal can
easily sit a whole needless turn away from where the arm already is. Left alone, the arm
dutifully drives that extra turn: wasted cycle time, extra wear, and cable wind-up.

```
arm is at 0°, goal stored as -350°   ->  planned as +10°   (340° of travel saved)
```

### Why the limit check is the whole point

A candidate is accepted **only** if it lands inside `[lower + margin, upper - margin]` of
that joint's URDF limits. An earlier limit-blind version of this rule was tried and had to
be reverted: it rewrote an in-limits value into one the joint could not physically reach,
and the arm ended up at unrelated, wrongly-oriented poses.

With the limit check, the transform is automatically a no-op exactly where it must be:

| joint span | behaviour |
| --- | --- |
| `> 2π` | a second in-limits equivalent exists → the nearest one is used |
| `== 2π` | the only alternative sits on the opposite limit edge → margin rejects it → unchanged |
| `< 2π` | no alternative exists → unchanged |
| prismatic | never touched — metres do not wrap |

On this cell that works out to 5 of 7 joints for the UR group and 2 of 7 for the Kawasaki
group; the counts are logged once, at startup, when the URDF is parsed.

### Where it hooks in

`MoveIt2.set_joint_goal()` — the single choke point that both `move_to_configuration()` and
`plan_async()` (and therefore `plan()`) pass through. Existing call sites need no changes.

Joint limits come from `/robot_description`, subscribed with `TRANSIENT_LOCAL` durability
because `robot_state_publisher` latches it. **Nothing ever blocks on it**: until the model
arrives the limit table is empty and every wrap is a no-op, so a run without a robot
description behaves exactly as it did before this feature existed.

### Knobs

Constructor arguments on `MoveIt2` (and anything derived from it):

| argument | default | meaning |
| --- | --- | --- |
| `unwind_joint_goals` | `True` | master switch |
| `unwind_limit_margin` | `0.05` | radians kept clear of each limit edge |
| `unwind_min_gain` | `0.35` | only rewrite a joint when it saves at least this much travel (≈20°), so goals are not churned for nothing |

New public API:

- `MoveIt2.unwind_joint_goal(joint_positions, joint_names=None, reference=None)` — apply the
  transform directly; returns the input unchanged whenever unwinding is off, the URDF is
  unavailable, or nothing is worth rewriting.
- `MoveIt2.joint_limits` — `{joint_name: JointLimit}` parsed from `/robot_description`
  (`{}` if it has not arrived). Parsed once, then cached.
- `MoveIt2.set_joint_goal(..., reference=...)` — unwind against a specific configuration
  instead of the measured one. `plan_async` passes the caller's `start_joint_state` here,
  so the goal is unwound against the state the plan actually starts from.

The implementation lives in [`pymoveit2_real/joint_wrap.py`](./pymoveit2_real/joint_wrap.py)
— pure Python and stdlib only, no ROS imports, so offline tooling can share it.

### Where it is deliberately turned OFF

- **`MoveIt2Gripper`** — a gripper's open/closed positions are fixed configurations of
  narrow or prismatic joints, so unwinding could never rewrite them. Off so a gripper does
  not carry a pointless `/robot_description` subscription.
- **The multirobot inspection executors** (`ur_inspection_node`, `kawasaki_inspection_node`)
  — those unwind goals themselves with more context: they tag the trajectory cache with the
  goal policy, unwind recorded paths, and keep a deliberately *raw* last-resort planning
  attempt after the unwound and normalized ones. Letting `MoveIt2` also unwind would turn
  that raw fallback into a duplicate of the first attempt, and would make their
  `wrap_goals_to_current:=false` A/B switch stop disabling anything.

### Notes

- **Idempotent.** Re-wrapping an already-wrapped goal against the same reference saves
  nothing, so it changes nothing. Callers that already unwind are safe to double up.
- **Cannot change where the arm ends up.** The rewritten goal is the same physical
  configuration, so reachability, collisions and tool orientation are unaffected — only the
  distance travelled to get there.
- **Trajectory caches are not policy-aware here.** If your use case replays recorded
  trajectories, note that this transform applies at goal-setting time only; a replayed path
  bypasses it entirely.
- `unwind_joint_goals=False` restores the previous behaviour exactly.

<div align="center" class="tg-wrap">
<table>
<tbody>
  <tr>
    <td width="25%"><img width="100%" src="https://user-images.githubusercontent.com/22929099/147369355-5f1b33ef-2e18-4042-9ea3-cd85b1a78fa0.gif" alt="Animation of ex_joint_goal.py"/></td>
    <td width="25%"><img width="100%" src="https://user-images.githubusercontent.com/22929099/147369356-b8ad2f4c-1996-47ac-9bfb-7fccd243fd56.gif" alt="Animation of ex_pose_goal.py"/></td>
    <td width="25%"><img width="100%" src="https://user-images.githubusercontent.com/22929099/147369354-640831e2-4661-4f3d-8fc2-3e97d7766e1a.gif" alt="Animation of ex_gripper.py"/></td>
    <td width="25%"><img width="100%" src="https://user-images.githubusercontent.com/22929099/147374152-50128188-ab73-4d55-a537-b641325ce9c6.gif" alt="Animation of ex_servo.py"/></td>
  </tr>
  <tr>
    <td width="25%"><div align="center">Joint Goal</div></td>
    <td width="25%"><div align="center">Pose Goal</div></td>
    <td width="25%"><div align="center">Gripper Action</div></td>
    <td width="25%"><div align="center">MoveIt 2 Servo</div></td>
  </tr>
</tbody>
</table>
</div>

## Instructions

### Dependencies

These are the primary dependencies required to use this project.

- ROS 2 [Galactic](https://docs.ros.org/en/galactic/Installation.html), [Humble](https://docs.ros.org/en/humble/Installation.html) or [Iron](https://docs.ros.org/en/iron/Installation.html)
- [MoveIt 2](https://moveit.ros.org/install-moveit2/binary) corresponding to the selected ROS 2 distribution

All additional dependencies are installed via [rosdep](https://wiki.ros.org/rosdep) during the building process below.

### Building

Clone this repository, install dependencies and build with [colcon](https://colcon.readthedocs.io).

```bash
# Clone this repository into your favourite ROS 2 workspace
git clone https://github.com/AndrejOrsula/pymoveit2.git
# Install dependencies
rosdep install -y -r -i --rosdistro ${ROS_DISTRO} --from-paths .
# Build
colcon build --merge-install --symlink-install --cmake-args "-DCMAKE_BUILD_TYPE=Release"
```

### Sourcing

Before utilising this package, remember to source the ROS 2 workspace.

```bash
source install/local_setup.bash
```

This enables importing of `pymoveit2` module from external workspaces.

## Examples

To demonstrate `pymoveit2` usage, [examples](./examples) directory contains scripts that demonstrate the basic functionality. Additional examples can be found under [ign_moveit2_examples](https://github.com/AndrejOrsula/ign_moveit2_examples) repository.

Prior to running the examples, configure an environment for control of a robot with MoveIt 2. For instance, one of the following launch scripts from [panda_ign_moveit2](https://github.com/AndrejOrsula/panda_ign_moveit2) repository can be used.

```bash
# RViz (fake) ROS 2 control
ros2 launch panda_moveit_config ex_fake_control.launch.py
# Gazebo (simulated) ROS 2 control
ros2 launch panda_moveit_config ex_ign_control.launch.py
```

After that, the individual scripts can be run.

```bash
# Move to joint configuration
ros2 run pymoveit2 ex_joint_goal.py --ros-args -p joint_positions:="[1.57, -1.57, 0.0, -1.57, 0.0, 1.57, 0.7854]"
# Move to Cartesian pose (motion in either joint or Cartesian space)
ros2 run pymoveit2 ex_pose_goal.py --ros-args -p position:="[0.25, 0.0, 1.0]" -p quat_xyzw:="[0.0, 0.0, 0.0, 1.0]" -p cartesian:=False
# Repeatadly toggle the gripper (or use "open"/"close" actions)
ros2 run pymoveit2 ex_gripper.py --ros-args -p action:="toggle"
# Example of using MoveIt 2 Servo to move the end-effector in a circular motion
ros2 run pymoveit2 ex_servo.py
# Example of adding a collision object with primitive geometry to the planning scene of MoveIt 2
ros2 run pymoveit2 ex_collision_primitive.py --ros-args -p shape:="sphere" -p position:="[0.5, 0.0, 0.5]" -p dimensions:="[0.04]"
# Example of adding a collision object with mesh geometry to the planning scene of MoveIt 2
ros2 run pymoveit2 ex_collision_mesh.py --ros-args -p action:="add" -p position:="[0.5, 0.0, 0.5]" -p quat_xyzw:="[0.0, 0.0, -0.707, 0.707]"
```

## Directory Structure

The following directory structure is utilised for this package.

```bash
.
├── examples/                # [dir] Examples and cell scenario nodes
├── launch/                  # [dir] Launch files for the scenarios above
├── pymoveit2_real/          # [dir] The Python module
    ├── robots/              # [dir] Presets for robots (data that can be extracted from URDF/SRDF)
    ├── gripper_command.py   # Interface for Gripper that is controlled by GripperCommand
    ├── gripper_interface.py # Combined MoveIt 2 Gripper + GripperCommand interface
    ├── harmony_defects.py   # Fixed synthetic defect set shared by the HARMONY scenarios
    ├── joint_wrap.py        # Limit-aware 2*pi joint-goal unwinding (see the section above)
    ├── moveit2_gripper.py   # Interface for MoveIt 2 Gripper that is controlled by JointTrajectoryController
    ├── moveit2_servo.py     # Interface for MoveIt 2 Servo that enables real-time control in Cartesian Space
    ├── moveit2.py           # Interface for MoveIt 2 that enables planning and execution of trajectories
    ├── trajectory_store.py  # Record-once / replay-forever JSON store for planned trajectories
    └── utils.py             # Small shared helpers
├── tools/                   # [dir] Standalone helper scripts
├── CMakeLists.txt           # Colcon-enabled CMake recipe
└── package.xml              # ROS 2 package metadata
```
