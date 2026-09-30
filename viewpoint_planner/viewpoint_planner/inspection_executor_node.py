#!/usr/bin/env python3
"""
Single-UR10e inspection executor.

Drives the UR10e (linear rail + 6 arm joints) to every viewpoint in the plan JSON and
captures its SICK cloud at each stop, writing
    <output_base_dir>/single_ur10e/<sim|real>_data/pcds/<N>.pcd
    <output_base_dir>/single_ur10e/<sim|real>_data/poses/<N>.txt

Everything that makes a run REPEATABLE lives in `inspection_base.InspectionNodeBase`:
the trajectory cache, 2*pi goal unwinding, nearest-branch IK, start handling, the
ground plane and the padding. This file is only the UR-shaped part of it -- which
MoveIt group, which camera topics, how the trajectory is dispatched, and the chassis
padding rule.

TWO THINGS ARE DELIBERATELY DIFFERENT FROM THE PRE-2026-09-09 VERSION OF THIS NODE:

  1. It no longer re-plans every viewpoint on every run. A path is planned once,
     recorded to <plans>/trajectories/ur_<vp_id>.json and replayed thereafter, so the
     arm drives the path that was validated instead of a fresh sample from a
     randomized planner. `force_replan:=true` rebuilds the cache.
  2. It executes through move_group's `execute_trajectory` action rather than pushing
     a raw FollowJointTrajectory goal at the controller. The raw bypass is what made
     the rail lurch on the two-arm cell; move_group's execution manager routes the
     full 7-joint trajectory to the scaled_joint_trajectory_controller the way the
     proven pymoveit2_real examples do. Set `execute_via_move_group:=false` to go back
     to the raw controller path.
"""
import os
import time
from threading import Thread

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.action import ExecuteTrajectory

from pymoveit2_real import MoveIt2 as MoveIt2Real
from pymoveit2_real.robots import ur as ur_robot

from viewpoint_planner.inspection_base import InspectionNodeBase


class InspectionExecutor(InspectionNodeBase):
    # Links whose name carries this token are the inspected chassis; they get
    # `chassis_collision_padding` instead of the uniform `collision_padding`.
    _CHASSIS_TOKEN = "chassis"
    # Links matching any of these are never padded (currently none; kept as the one
    # place to exempt a body whose mesh is already generous).
    _NO_PAD_TOKENS = ()

    def __init__(self):
        super().__init__("inspection_executor_node", robot_tag="ur")

        # --- UR motion ---------------------------------------------------- #
        self.declare_parameter("ur_velocity", 0.1)
        self.declare_parameter("ur_acceleration", 0.1)
        self.declare_parameter(
            "ur_controller_action",
            "/scaled_joint_trajectory_controller/follow_joint_trajectory")
        # True  -> execute through move_group (proven; keeps the rail smooth).
        # False -> raw FollowJointTrajectory straight at the controller (the old path).
        self.declare_parameter("execute_via_move_group", True)

        # --- Pose-goal planning ------------------------------------------- #
        # The plan stores each viewpoint as a CAMERA pose plus one IK solution. Planning
        # to the pose (not to that stored solution) lets MoveIt/our branch search pick a
        # cheaper arm configuration that puts the camera in exactly the same place, so
        # the captured cloud is identical while the arm travels far less.
        # Default False: drive the joint configuration the planner chose and validated
        # for the whole tour (see viewpoint_planner_node._order_by_joint_space).
        self.declare_parameter("use_pose_goal", False)
        self.declare_parameter("tool_frame", "ur10e_depth_optical_frame")

        # --- Capture streams ---------------------------------------------- #
        self.declare_parameter("sim_pointcloud_topic", "/sim/pointcloud")
        self.declare_parameter("real_pointcloud_topic", "/sick_points")
        self.declare_parameter("sim_sensor_frame", "sim_ur10e_depth_optical_frame")
        self.declare_parameter("real_sensor_frame", "ur10e_sick_optical_frame")

        # --- OPTIONAL separate margin for the chassis ----------------------- #
        # 0.0 (default) -> the chassis gets collision_padding like every other link,
        # which is what the multirobot cell does (4 cm on every ur10e_* link). A value
        # > 0 is absolute per-link padding for the chassis, NOT additive. It was 0.10
        # until 2026-09-10; set to match the multirobot cell.
        self.declare_parameter("chassis_collision_padding", 0.0)

        # --- Start / home -------------------------------------------------- #
        # Index 0 is the rail in METRES, 1..6 the arm joints in DEGREES.
        self.declare_parameter("start_pose", [1.0, 0.0, -90.0, 0.0, -90.0, 0.0, 0.0])
        self.declare_parameter("return_home", True)
        self.declare_parameter(
            "home_joint_positions", [1.0, 0.0, -90.0, 0.0, -90.0, 0.0, 0.0])

        self.arm_label = "UR"
        self.start_pose_param = "start_pose"
        self.return_home_param = "return_home"
        self.home_positions_param = "home_joint_positions"
        # Rebase each trajectory's first point onto the measured pose instead of walking
        # to the recorded start; the base falls back to an explicit collision-aware walk
        # whenever the gap is too big for that to be a correction rather than a jump.
        self.rebase_to_current = True
        self.align_on_fresh = False

        # Capture streams: sim always, real only on the physical cell.
        base = os.path.expanduser(self.get_parameter("output_base_dir").value)
        scenario_dir = os.path.join(base, "single_ur10e")
        self.streams = [self._make_stream(
            "sim", "ur",
            self.get_parameter("sim_pointcloud_topic").value,
            self.get_parameter("sim_sensor_frame").value,
            os.path.join(scenario_dir, "sim_data"))]
        if not self.only_sim:
            self.streams.append(self._make_stream(
                "real", "ur",
                self.get_parameter("real_pointcloud_topic").value,
                self.get_parameter("real_sensor_frame").value,
                os.path.join(scenario_dir, "real_data")))
        self._subscribe_streams()

        # --- MoveIt2 (rail + 6 arm joints), PLANNING ONLY ------------------ #
        self.moveit = MoveIt2Real(
            node=self,
            joint_names=ur_robot.joint_names(),
            base_link_name=self.world_frame,
            end_effector_name=ur_robot.end_effector_name(),
            group_name=ur_robot.MOVE_GROUP_ARM,
            callback_group=self._cb,
            # This node unwinds goals itself, with more context than MoveIt2 has: it
            # stamps the trajectory cache with the goal policy, unwinds recorded paths,
            # and keeps a deliberately RAW last-resort attempt after the unwound and
            # normalized ones. Letting MoveIt2 also unwind would make
            # wrap_goals_to_current:=false stop disabling anything.
            unwind_joint_goals=False,
        )
        # Setting this enables the base's nearest-branch IK: it solves /compute_ik for
        # `tool_frame` against this group instead of letting the goal sampler pick an
        # arbitrary branch.
        self.ik_group_name = ur_robot.MOVE_GROUP_ARM
        self.moveit.max_velocity = self.get_parameter("ur_velocity").value
        self.moveit.max_acceleration = self.get_parameter("ur_acceleration").value
        self.moveit.allowed_planning_time = self.get_parameter("allowed_planning_time").value
        self.moveit.num_planning_attempts = self.get_parameter("num_planning_attempts").value
        self.moveit.planner_id = self.get_parameter("planner_id").value

        self._via_move_group = bool(self.get_parameter("execute_via_move_group").value)
        if self._via_move_group:
            # Driven directly (own future-based await over the shared executor), NOT via
            # moveit.execute()/wait_until_executed(): the latter's internal spin_once()
            # corrupts this node's wait set and starves the capture subscriptions.
            self.ctrl = ActionClient(
                self, ExecuteTrajectory, "execute_trajectory", callback_group=self._cb)
        else:
            self.ctrl = ActionClient(
                self, FollowJointTrajectory,
                self.get_parameter("ur_controller_action").value,
                callback_group=self._cb)

        planner = self.moveit.planner_id or "move_group default (RRTConnect)"
        self.get_logger().info(
            f"InspectionExecutor ready [only_sim={self.only_sim}]. plan='{self.plan_file}'. "
            f"planner={planner}, execution="
            f"{'move_group/execute_trajectory' if self._via_move_group else 'raw controller'}. "
            f"Capture streams: "
            + ", ".join(f"{s['name']}('{s['topic']}' -> {s['subtree']})" for s in self.streams))

    # --- Hooks ------------------------------------------------------------ #
    def _use_pose_goal(self):
        return bool(self.get_parameter("use_pose_goal").value)

    def _pose_target_link(self):
        return self.get_parameter("tool_frame").value

    def _arm_link_names(self, scene):
        """Every 'ur10e_' collision body: the moving arm chain, the static furniture and
        the inspected chassis. Padding differs per link -- see _padding_for_link."""
        names = list(scene.allowed_collision_matrix.entry_names)
        return [n for n in names
                if n.startswith("ur10e_")
                and not any(tok in n for tok in self._NO_PAD_TOKENS)]

    def _padding_for_link(self, name, base_padding):
        chassis = float(self.get_parameter("chassis_collision_padding").value)
        if chassis <= 0.0:
            return base_padding
        return chassis if self._CHASSIS_TOKEN in name else base_padding

    def _dispatch(self, traj, label):
        if not self._via_move_group:
            goal = FollowJointTrajectory.Goal()
            goal.trajectory = traj
            return self.ctrl.send_goal_async(goal)
        # move_group validates the goal's start state and ABORTS outright if its newest
        # /joint_states is over 1 s old ("couldn't receive full current joint state
        # within 1s") -- the controller never sees the goal and the arm silently does
        # not move. Every dispatch goes through here, so one freshness wait covers them.
        self._wait_fresh_joint_state(label)
        goal = ExecuteTrajectory.Goal()
        goal.trajectory.joint_trajectory = traj
        return self.ctrl.send_goal_async(goal)

    def _settle(self, send, targets, label):
        # Completion is the execution result ALONE -- no measured-arrival gating, so the
        # real UR (whose rail feedback may not reach this node's /joint_states) is never
        # blocked by a tolerance check.
        if self._await(send, label):
            self.get_logger().info(f"{label} reached.")
            return True
        self.get_logger().warning(f"{label} motion did not report success; continuing anyway.")
        return False

    def _viewpoint_targets(self, traj, vp):
        return None  # controller result only; no measured-arrival gate


def main(args=None):
    rclpy.init(args=args)
    node = InspectionExecutor()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    Thread(target=executor.spin, daemon=True).start()
    time.sleep(3.0)  # let action servers / TF / bridge come up

    try:
        node.run()  # MoveIt action calls on the main thread
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
