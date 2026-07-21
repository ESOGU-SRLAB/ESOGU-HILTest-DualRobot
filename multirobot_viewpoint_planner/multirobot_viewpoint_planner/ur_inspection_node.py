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
import math
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
        # SICK collision top-up (see _setup_planning_scene_extras). The sick_camera mesh
        # stops ~1.9 cm short of the real housing on the optics side, so MoveIt happily
        # planned the camera into the UR's own joints (ur_vp_011). Rather than editing the
        # URDF we attach one box to the camera link in the planning scene.
        self.declare_parameter("ur_camera_box_link", "ur10e_sick_camera")
        # x, y, z in the sick_camera frame. +Z IS THE VIEWING DIRECTION: the planner's
        # viewpoint `rotation` aims its column 2 (+Z) at the surface (that is the arrow
        # plan_visualizer draws), and depth_optical_frame -- which this node commands to
        # exactly that rotation and which is unrotated w.r.t. sick_camera -- therefore
        # looks along sick_camera +Z. (depth_frame/rgb_frame differ only in Y because that
        # is the RGB<->depth baseline, NOT the lens direction.)
        # The mesh's front face ends at z=+0.0342, so a 0.019-thick box sitting OUTSIDE it
        # is centred at 0.0342 + 0.019/2 = 0.0437, over the face's centre (-0.0301,
        # 0.0174). size/position are x,y,z in the BOX's own frame, which the yaw below
        # rotates onto the housing -- so 0.067 and 0.070 run along the housing's edges,
        # not along the link axes.
        self.declare_parameter("ur_camera_box_size", [0.067, 0.070, 0.019])
        self.declare_parameter("ur_camera_box_position", [-0.0301, 0.0174, 0.0437])
        # The HOUSING IS ROTATED about the viewing axis inside this link: a min-area-rect
        # fit to the mesh's front face lands on 60.00 deg -- the very angle already in the
        # URDF as ur_sick_optical_joint's rpy="0 0 1.04720". In that rotated (camera-
        # aligned) frame the face measures 7.98 x 6.97 cm, which is why the asked-for
        # 7.0 cm side fits it exactly. An axis-aligned box can only ever sit askew here.
        self.declare_parameter("ur_camera_box_yaw_deg", 60.0)
        # Links the box is ALLOWED to touch: only the rigid mount it is bolted to. Every
        # other link (wrists, forearm, cable channel) stays checked -- those are exactly
        # the collisions this box exists to catch.
        self.declare_parameter(
            "ur_camera_box_touch_links",
            ["ur10e_sick_camera", "ur10e_flange", "ur10e_tool0"])

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
        )
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

    def _setup_planning_scene_extras(self):
        """Top up the SICK camera's collision geometry with one box on its optics face.

        The sick_camera collision MESH under-reports the real housing on the viewing side,
        so MoveIt planned paths that swing the camera into the UR's own joints. The box is
        ATTACHED to the camera link (not added as a world object) so it travels with the
        arm and is checked against the rest of the robot on every plan."""
        link = self.get_parameter("ur_camera_box_link").value
        if not link:
            return
        size = tuple(float(v) for v in self.get_parameter("ur_camera_box_size").value)
        pos = tuple(float(v) for v in self.get_parameter("ur_camera_box_position").value)
        touch = list(self.get_parameter("ur_camera_box_touch_links").value)
        yaw = math.radians(float(self.get_parameter("ur_camera_box_yaw_deg").value))
        quat = (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))  # yaw about +Z
        box_id = "ur_sick_camera_front"

        # Define it IN the camera frame, then attach it to that same link: the pose is then
        # the box's fixed offset on the camera, independent of where the arm happens to be.
        self.moveit.add_collision_box(
            id=box_id, size=size, position=pos, quat_xyzw=quat, frame_id=link)
        time.sleep(0.3)
        self.moveit.attach_collision_object(id=box_id, link_name=link, touch_links=touch)
        time.sleep(0.3)
        self.get_logger().info(
            f"SICK camera collision top-up: {size[0]*100:.1f}x{size[1]*100:.1f}x"
            f"{size[2]*100:.1f} cm box attached to '{link}' at xyz={pos}, "
            f"yaw={math.degrees(yaw):.1f} deg (touch_links={touch}).")

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
