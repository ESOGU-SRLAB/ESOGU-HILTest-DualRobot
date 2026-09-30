#!/usr/bin/env python3
"""
OMPL planner benchmark for the two inspection arms.

Answers "which planner, and with how many attempts, gives the shortest arm travel on
THIS cell" with numbers instead of folklore. It calls MoveIt's /plan_kinematic_path
service only -- nothing is ever executed, so it is safe to run against the live cell
with the real robots powered on. It does need move_group up (the HIL launch), because
it plans against move_group's CURRENT planning scene (ground plane, padding, octomap).

  ros2 launch my_robot_cell_control hil_test_whole_unified.launch.py ...   # in another shell
  python3 tools/planner_bench.py --arm kawasaki \
      --planners RRTConnectkConfigDefault#1,RRTConnectkConfigDefault#10,TRRTkConfigDefault

Each row plans the viewpoint chain (home -> vp0 -> vp1 -> ...) `--reps` times and
reports, for the hops EVERY row solved:

  solved      hops solved / hops attempted (a planner that cannot solve a viewpoint
              costs you that viewpoint, so read this column before the travel column)
  travel deg  summed revolute joint travel along the returned path -- the number the
              nearest-branch IK work is trying to reduce
  rail m      summed prismatic travel (AGV / UR linear axis)
  exec s      duration of the time-parameterised trajectory at the given scaling, i.e.
              what the hop will actually cost on the floor
  plan s/hop  MoveIt's reported planning time

A planner name may carry its own attempt count as `Name#N`, which overrides
--attempts for that row; that is how the num_planning_attempts sweep is run.

Measured on the chassis plan, 2026-09-05, mock-hardware HIL, group real_kawasaki, the
8 plannable Kawasaki viewpoints (kawa_vp_001's planned goal is in collision with the
ground plane, so it is skipped -- see --skip):

  RRTConnect x1   4680 deg    RRTConnect x10  3663 deg   <- attempts, not the planner,
  RRTConnect x3   4364 deg    RRTConnect x20  3603 deg      is the real lever (-22%)
  TRRT x10        4074 deg
  RRT x1 / RRT* x1: 5.8 / 6.4 s PER HOP for no gain (they were configured with
  MaximizeMinClearanceObjective, which optimises clearance, not length -- fixed since).
  SBL / KPIECE / BKPIECE / LBKPIECE: 0 of 24, real_kawasaki had no projection_evaluator.

Caveats worth keeping in mind when reading a run:
  * The OTHER arm stays wherever it is parked, so absolute success rates depend on it.
    Compare planners within one run, never across runs.
  * The chain uses each viewpoint's PLANNED goal, not the branch the executor would
    pick, so the numbers are planner-only and do not include the branch-IK saving.
  * OMPL is stochastic; --reps 3 is the minimum that means anything.
"""
import argparse
import json
import time

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped  # noqa: F401  (kept for pose-goal variants)
from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest, RobotState
from moveit_msgs.srv import GetMotionPlan
from sensor_msgs.msg import JointState

ARMS = {
    "kawasaki": dict(
        group="real_kawasaki",
        key="kawasaki_viewpoints",
        joints=["world_to_agv", "joint1", "joint2", "joint3",
                "joint4", "joint5", "joint6"],
        rail="world_to_agv",
        home=[1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    ),
    "ur": dict(
        group="real_ur10e",
        key="ur_viewpoints",
        joints=["ur10e_base_to_robot_mount", "ur10e_shoulder_pan_joint",
                "ur10e_shoulder_lift_joint", "ur10e_elbow_joint",
                "ur10e_wrist_1_joint", "ur10e_wrist_2_joint", "ur10e_wrist_3_joint"],
        rail="ur10e_base_to_robot_mount",
        home=[0.0, 0.0, -1.5708, 0.0, -1.5708, 0.0, 0.0],
    ),
}


class Bench(Node):
    def __init__(self, arm):
        super().__init__("planner_bench")
        self.arm = arm
        self.cli = self.create_client(GetMotionPlan, "plan_kinematic_path")

    def plan(self, start, goal, planner, budget, attempts, scaling):
        req = GetMotionPlan.Request()
        r = MotionPlanRequest()
        r.group_name = self.arm["group"]
        r.planner_id = planner
        r.allowed_planning_time = float(budget)
        r.num_planning_attempts = int(attempts)
        r.max_velocity_scaling_factor = scaling
        r.max_acceleration_scaling_factor = scaling
        js = JointState()
        js.name = list(self.arm["joints"])
        js.position = [float(v) for v in start]
        rs = RobotState()
        rs.joint_state = js
        rs.is_diff = True                    # leave the rest of the cell where it is
        r.start_state = rs
        con = Constraints()
        for n, v in zip(self.arm["joints"], goal):
            jc = JointConstraint()
            jc.joint_name = n
            jc.position = float(v)
            jc.tolerance_above = jc.tolerance_below = 1e-4
            jc.weight = 1.0
            con.joint_constraints.append(jc)
        r.goal_constraints.append(con)
        req.motion_plan_request = r

        t0 = time.time()
        fut = self.cli.call_async(req)
        rclpy.spin_until_future_complete(
            self, fut, timeout_sec=budget * attempts + 30.0)
        wall = time.time() - t0
        res = fut.result()
        if res is None:
            return None, wall, None, "no response"
        rp = res.motion_plan_response
        if rp.error_code.val != 1 or not rp.trajectory.joint_trajectory.points:
            return None, wall, rp.planning_time, rp.error_code.val
        t = rp.trajectory.joint_trajectory
        a = np.array([p.positions for p in t.points])
        rev = [i for i, n in enumerate(t.joint_names) if n != self.arm["rail"]]
        rail = [i for i, n in enumerate(t.joint_names) if n == self.arm["rail"]]
        end = t.points[-1].time_from_start
        return ((float(np.sum(np.abs(np.diff(a[:, rev], axis=0)))),
                 float(np.sum(np.abs(np.diff(a[:, rail], axis=0)))) if rail else 0.0,
                 end.sec + end.nanosec * 1e-9),
                wall, rp.planning_time, 1)


def goals(arm, plan_file, skip):
    plan = json.load(open(plan_file))
    out = []
    for vp in plan[arm["key"]]:
        if vp["id"] in skip:
            continue
        lut = dict(zip(vp["joint_names"], vp["joint_positions"]))
        out.append((vp["id"], [float(lut[n]) for n in arm["joints"]]))
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--plan", default=("/home/cem/colcon_ws/src/"
                                       "multirobot_viewpoint_planner/plans/"
                                       "multirobot_viewpoint_plan.json"))
    ap.add_argument("--arm", choices=sorted(ARMS), default="kawasaki")
    ap.add_argument("--planners", default="RRTConnectkConfigDefault",
                    help="comma separated; 'Name#N' overrides --attempts for that row")
    ap.add_argument("--budget", type=float, default=5.0, help="allowed_planning_time")
    ap.add_argument("--attempts", type=int, default=10, help="num_planning_attempts")
    ap.add_argument("--reps", type=int, default=3)
    ap.add_argument("--hops", type=int, default=0, help="0 = every viewpoint")
    ap.add_argument("--scaling", type=float, default=0.08,
                    help="velocity/acceleration scaling, for the exec s column")
    ap.add_argument("--skip", default="kawa_vp_001",
                    help="viewpoint ids to leave out (comma separated)")
    args = ap.parse_args()

    arm = ARMS[args.arm]
    vps = goals(arm, args.plan, set(x for x in args.skip.split(",") if x))
    if args.hops:
        vps = vps[:args.hops]
    planners = args.planners.split(",")

    rclpy.init()
    b = Bench(arm)
    if not b.cli.wait_for_service(timeout_sec=15.0):
        print("/plan_kinematic_path never appeared -- is move_group running?")
        return
    print(f"{args.arm} / {arm['group']}: {len(vps)} hops, {args.reps} rep(s), "
          f"{args.budget}s budget, scaling {args.scaling}\n")

    rows = {}
    for pl in planners:
        name, _, att = pl.partition("#")
        per_hop = {vid: [] for vid, _ in vps}
        ptimes, errs = [], {}
        for _ in range(args.reps):
            cur = list(arm["home"])
            for vid, goal in vps:
                out, wall, ptime, code = b.plan(
                    cur, goal, name, args.budget,
                    int(att) if att else args.attempts, args.scaling)
                ptimes.append(ptime if ptime else wall)
                if out is None:
                    errs[code] = errs.get(code, 0) + 1
                else:
                    per_hop[vid].append(out)
                cur = goal          # same chain for every planner, failures included
        rows[pl] = (per_hop, ptimes, errs)

    hops = [vid for vid, _ in vps]
    common = [h for h in hops if all(rows[p][0][h] for p in planners)]
    print(f"{'planner':34s} {'solved':>8s} {'travel deg':>11s} {'rail m':>7s} "
          f"{'exec s':>8s} {'plan s/hop':>11s}")
    print(f"(travel/rail/exec are summed over the {len(common)} hop(s) every row solved; "
          f"errors: 1=ok, -2=INVALID_MOTION_PLAN, 99999=FAILURE)")
    for pl in planners:
        per_hop, ptimes, errs = rows[pl]
        solved = sum(len(v) for v in per_hop.values())
        total = len(hops) * args.reps
        if not common:
            print(f"{pl:34s} {solved:3d}/{total:<4d}  (no hop solved by every row) "
                  f"errors={errs}")
            continue
        agg = [np.mean([o[i] for o in per_hop[h]]) for h in common for i in (0,)]
        rail = [np.mean([o[1] for o in per_hop[h]]) for h in common]
        dur = [np.mean([o[2] for o in per_hop[h]]) for h in common]
        print(f"{pl:34s} {solved:3d}/{total:<4d} {np.rad2deg(np.sum(agg)):11.0f} "
              f"{np.sum(rail):7.2f} {np.sum(dur):8.1f} {np.mean(ptimes):11.2f}")
        miss = [h for h in hops if len(per_hop[h]) < args.reps]
        if miss:
            print(f"{'':34s}   partial: "
                  + ", ".join(f"{h}({len(per_hop[h])}/{args.reps})" for h in miss)
                  + f"   errors={errs}")
    b.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
