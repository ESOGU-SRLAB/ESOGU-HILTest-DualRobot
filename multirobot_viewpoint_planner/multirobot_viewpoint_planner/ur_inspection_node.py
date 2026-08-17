#!/usr/bin/env python3
"""
UR10e single-arm inspection node.

Drives the UR10e (rail + 6 arm joints) to its own coverage viewpoints and captures
its SICK camera. The UR plans to a CARTESIAN pose goal (the camera optical frame the
planner solved IK for) so MoveIt re-derives a valid, correctly-oriented joint
solution. Completion is decided by the CONTROLLER's action result ALONE -- no
executor-side measured-arrival tolerance gating -- so the real UR (whose rail feedback
may not appear on this arm's /joint_states) is never blocked by a tolerance check.

EXECUTION PATH (the rail fix): the UR does NOT dispatch a raw FollowJointTrajectory goal
straight to the controller. Instead it executes through MoveIt's `execute_trajectory`
action (move_group) via `self.moveit.execute()` + `self.moveit.wait_until_executed()` --
the EXACT primitive of the 110-loop-proven pymoveit2_real examples
(ur_inspection_scenario / combined_joint_goal). move_group's execution manager then
routes the full 7-joint trajectory (rail + 6 arm) to the scaled_joint_trajectory_controller
the way that proven system did, instead of the raw-controller bypass that made the rail
lurch. Kawasaki keeps the raw-controller dispatch (its constraint: unchanged).

See inspection_base.InspectionNodeBase for the shared machinery.
"""
import os
import time
from threading import Thread

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from moveit_msgs.action import ExecuteTrajectory

from pymoveit2_real import MoveIt2 as MoveIt2Real
from pymoveit2_real.robots import ur as ur_robot

from multirobot_viewpoint_planner.inspection_base import InspectionNodeBase


class URInspectionNode(InspectionNodeBase):
    def __init__(self):
        super().__init__("ur_inspection_node", robot_tag="ur")

        # UR-specific parameters.
        self.declare_parameter("ur_velocity", 0.1)
        self.declare_parameter("ur_acceleration", 0.1)
        self.declare_parameter(
            "ur_controller_action",
            "/scaled_joint_trajectory_controller/follow_joint_trajectory")
        # Pose-goal planning (THE orientation fix): plan the UR to the camera optical
        # frame the planner solved IK for, so MoveIt re-solves a correctly-oriented pose.
        self.declare_parameter("ur_use_pose_goal", True)
        self.declare_parameter("ur_tool_frame", "ur10e_depth_optical_frame")
        # Capture streams.
        self.declare_parameter("sim_ur_topic", "/sim/pointcloud")
        self.declare_parameter("real_ur_topic", "/sick_points")
        self.declare_parameter("sim_ur_sensor_frame", "sim_ur10e_depth_optical_frame")
        self.declare_parameter("real_ur_sensor_frame", "ur10e_sick_optical_frame")
        # Start / home poses (index 0 rail in METRES, 1..6 arm joints in DEGREES).
        self.declare_parameter("start_pose_ur", [1.0, 0.0, -90.0, 0.0, -90.0, 0.0, 0.0])
        self.declare_parameter("return_home_ur", True)
        self.declare_parameter("ur_home_joint_positions", [1.0, 0.0, -90.0, 0.0, -90.0, 0.0, 0.0])
        # NOTE: the SICK camera collision shell (a box on the optics face, one on
        # the back, and one on each of the four sides) used to be ATTACHED here at
        # runtime because the sick_camera mesh under-reports the real housing. It
        # now lives in the URDF as six <collision> boxes on the ur10e_sick_camera
        # link (Universal_Robots_ROS2_Description/urdf/ur_macro.xacro), so
        # move_group knows about it during both recording and playback without this
        # node adding anything. See that link's comment for the geometry.

        self.arm_label = "UR"
        self.start_pose_param = "start_pose_ur"
        self.return_home_param = "return_home_ur"
        self.home_positions_param = "ur_home_joint_positions"
        # UR: rebase every trajectory's first point onto the current measured pose
        # (proven ur_inspection_scenario playback), INSTEAD of a separate align-to-start
        # move -- this is what stops the rail's spurious back-and-forth.
        self.rebase_to_current = True
        self.align_on_fresh = False  # (align path unused now that rebase is on)

        # Capture streams: sim always; real only when connected to the physical cell.
        base = os.path.expanduser(self.get_parameter("output_base_dir").value)
        self.streams = [self._make_stream(
            "sim_ur", "ur",
            self.get_parameter("sim_ur_topic").value,
            self.get_parameter("sim_ur_sensor_frame").value,
            os.path.join(base, "sim_pcds", "ur_data"))]
        if not self.only_sim:
            self.streams.append(self._make_stream(
                "real_ur", "ur",
                self.get_parameter("real_ur_topic").value,
                self.get_parameter("real_ur_sensor_frame").value,
                os.path.join(base, "real_pcds", "ur_data")))
        self._subscribe_streams()

        # UR10e MoveIt2 (rail + 6 arm joints), PLANNING ONLY.
        self.moveit = MoveIt2Real(
            node=self,
            joint_names=ur_robot.joint_names(),
            base_link_name=self.world_frame,
            end_effector_name=ur_robot.end_effector_name(),
            group_name=ur_robot.MOVE_GROUP_ARM,
            callback_group=self._cb,
            # This executor unwinds goals itself, with more context than MoveIt2 has:
            # it tags the trajectory cache with the goal policy, unwinds recorded paths,
            # and keeps a deliberately RAW last-resort attempt after the unwound and
            # normalized ones. Letting MoveIt2 also unwind would turn that raw fallback
            # into a duplicate of the first attempt and would make
            # wrap_goals_to_current:=false stop disabling anything.
            unwind_joint_goals=False,
        )
        # Enables the base's nearest-branch IK (it solves /compute_ik for ur_tool_frame
        # against this group instead of letting the pose goal pick an arbitrary branch).
        self.ik_group_name = ur_robot.MOVE_GROUP_ARM
        self.moveit.max_velocity = self.get_parameter("ur_velocity").value
        self.moveit.max_acceleration = self.get_parameter("ur_acceleration").value
        self.moveit.allowed_planning_time = self.get_parameter("allowed_planning_time").value
        self.moveit.num_planning_attempts = self.get_parameter("num_planning_attempts").value
        # UR executes via MoveIt's execute_trajectory action (move_group) -- the proven
        # robot.execute() path. move_group's execution manager routes the full rail+arm
        # trajectory to the scaled_joint_trajectory_controller the way the 110-loop system
        # did. We drive this action DIRECTLY (own future-based await via the shared
        # MultiThreadedExecutor), NOT moveit.execute()/wait_until_executed() -- the latter's
        # internal rclpy.spin_once() corrupts our executor's wait set and kills the spin
        # thread, which then starves the capture subscriptions ("No point cloud").
        self.ctrl = ActionClient(
            self, ExecuteTrajectory, "execute_trajectory", callback_group=self._cb)

    # --- Hooks ---------------------------------------------------------- #
    def should_run(self):
        only_ur, only_kawa = self._resolve_only_flags()
        # UR runs unless only_kawasaki (with only_ur False). Both-True already reset.
        return (not only_kawa) or only_ur

    def _use_pose_goal(self):
        return bool(self.get_parameter("ur_use_pose_goal").value)

    def _pose_target_link(self):
        return self.get_parameter("ur_tool_frame").value

    def _arm_link_names(self, scene):
        names = list(scene.allowed_collision_matrix.entry_names)
        return [n for n in names if n.startswith("ur10e_")]

    def _dispatch(self, traj, label):
        # PROVEN PATH: drive move_group's execute_trajectory action with the full rail+arm
        # trajectory (the robot.execute() equivalent), then await it with the base's
        # future-based _await over the shared MultiThreadedExecutor -- no spin_once, so the
        # executor's wait set stays intact and the capture subscriptions keep flowing.
        # move_group validates the goal's start state first and ABORTS it outright if its
        # newest /joint_states is over 1 s old ("couldn't receive full current joint state
        # within 1s") -- the controller never sees the goal and the arm silently does not
        # move. Every UR dispatch (viewpoints, align-to-start, start pose, homing) goes
        # through here, so one freshness wait in front of the send covers them all.
        self._wait_fresh_joint_state(label)
        goal = ExecuteTrajectory.Goal()
        goal.trajectory.joint_trajectory = traj
        return self.ctrl.send_goal_async(goal)

    def _settle(self, send, targets, label):
        # UR trusts the execute_trajectory result ALONE -- no measured-arrival gating.
        if self._await(send, label):
            self.get_logger().info(f"{label} reached (execute_trajectory succeeded).")
            return True
        self.get_logger().warning(f"{label} motion did not report success; continuing anyway.")
        return False

    def _viewpoint_targets(self, traj, vp):
        return None  # controller-only; no measured-arrival gate


def main(args=None):
    rclpy.init(args=args)
    node = URInspectionNode()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    Thread(target=executor.spin, daemon=True).start()
    time.sleep(3.0)  # let action servers / TF come up

    try:
        node.run()  # MoveIt action calls on the main thread
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
