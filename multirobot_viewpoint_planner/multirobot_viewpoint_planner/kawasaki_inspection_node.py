#!/usr/bin/env python3
"""
Kawasaki RS005L single-arm inspection node.

Drives the Kawasaki (AGV rail world_to_agv + 6 arm joints) to its own coverage
viewpoints and captures its SICK camera. The Kawasaki plans in JOINT SPACE (with the
+/-2*pi limit-edge normalization) and gates advancement on MEASURED arrival, because
its AGV is driven through an async rosbridge link whose JTC action result finishes long
before the slow AGV physically arrives.

See inspection_base.InspectionNodeBase for the shared machinery.
"""
import os
import re
import time
from threading import Thread

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from control_msgs.action import FollowJointTrajectory

from pymoveit2_real import MoveIt2 as MoveIt2Real

from multirobot_viewpoint_planner.inspection_base import InspectionNodeBase

# Joints of the Kawasaki planning group ("real_kawasaki"). The plan stores a
# whole-robot solution per viewpoint; the MoveIt2 object must be built with ONLY these
# joints (its joint-state cache is dropped unless every one appears in /joint_states).
KAWASAKI_JOINT_NAMES = [
    "world_to_agv", "joint1", "joint2", "joint3", "joint4", "joint5", "joint6",
]


class KawasakiInspectionNode(InspectionNodeBase):
    def __init__(self):
        super().__init__("kawasaki_inspection_node", robot_tag="kawasaki")

        # Kawasaki-specific parameters.
        self.declare_parameter("enable_kawasaki", True)
        self.declare_parameter("kawasaki_velocity", 0.015)
        self.declare_parameter("kawasaki_acceleration", 0.015)
        self.declare_parameter(
            "kawasaki_controller_action",
            "/kawasaki/kawasaki_controller/follow_joint_trajectory")
        # Capture streams.
        self.declare_parameter("sim_kawasaki_topic", "/sim/kawasaki/pointcloud")
        self.declare_parameter("real_kawasaki_topic", "/kawasaki/pointcloud")
        self.declare_parameter("sim_kawasaki_sensor_frame", "sim_kawasaki_camera_rgb_optic_frame")
        # Fallback frame if the cloud's own header frame can't be looked up. Aligned to
        # the CALIBRATED sick frame the HIL launch stamps the real cloud with, so a
        # momentary TF miss never reverts to the old uncalibrated camera_rgb_optic_frame.
        self.declare_parameter("real_kawasaki_sensor_frame", "kawasaki_sick_optical_frame")
        # Start / home poses (index 0 AGV rail in METRES, 1..6 arm joints in DEGREES).
        self.declare_parameter("start_pose_kawasaki", [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        # Homing also commands the AGV, so it is OPT-IN (default off).
        self.declare_parameter("return_home_kawasaki", False)
        self.declare_parameter("kawasaki_home_joint_positions", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        self.arm_label = "Kawasaki"
        self.start_pose_param = "start_pose_kawasaki"
        self.return_home_param = "return_home_kawasaki"
        self.home_positions_param = "kawasaki_home_joint_positions"
        self.align_on_fresh = True  # preserve the existing Kawasaki behaviour

        # Capture streams: sim always; real only when connected to the physical cell.
        base = os.path.expanduser(self.get_parameter("output_base_dir").value)
        self.streams = [self._make_stream(
            "sim_kawasaki", "kawasaki",
            self.get_parameter("sim_kawasaki_topic").value,
            self.get_parameter("sim_kawasaki_sensor_frame").value,
            os.path.join(base, "sim_pcds", "kawasaki_data"))]
        if not self.only_sim:
            self.streams.append(self._make_stream(
                "real_kawasaki", "kawasaki",
                self.get_parameter("real_kawasaki_topic").value,
                self.get_parameter("real_kawasaki_sensor_frame").value,
                os.path.join(base, "real_pcds", "kawasaki_data")))
        self._subscribe_streams()

        # Kawasaki MoveIt2 (AGV rail + 6 arm joints), PLANNING ONLY.
        self.moveit = MoveIt2Real(
            node=self,
            joint_names=list(KAWASAKI_JOINT_NAMES),
            base_link_name=self.world_frame,
            end_effector_name="link6",
            group_name="real_kawasaki",
            callback_group=self._cb,
        )
        self.moveit.max_velocity = self.get_parameter("kawasaki_velocity").value
        self.moveit.max_acceleration = self.get_parameter("kawasaki_acceleration").value
        self.moveit.allowed_planning_time = self.get_parameter("allowed_planning_time").value
        self.moveit.num_planning_attempts = self.get_parameter("num_planning_attempts").value
        self.ctrl = ActionClient(
            self, FollowJointTrajectory,
            self.get_parameter("kawasaki_controller_action").value, callback_group=self._cb)

    # --- Hooks ---------------------------------------------------------- #
    def should_run(self):
        only_ur, only_kawa = self._resolve_only_flags()
        if not self.get_parameter("enable_kawasaki").value:
            return False
        # Kawasaki runs unless only_ur (with only_kawa False). Both-True already reset.
        return (not only_ur) or only_kawa

    def _use_pose_goal(self):
        return False  # joint-space + normalization

    def _arm_link_names(self, scene):
        names = list(scene.allowed_collision_matrix.entry_names)
        kawa = re.compile(r"^link[1-6]$")
        return [n for n in names if kawa.match(n)]

    def _settle(self, send, targets, label):
        # Kawasaki: gate on MEASURED arrival (its async AGV settles after the JTC result).
        if self._confirm_accepted(send, label):
            if targets:
                return self._wait_until_reached(targets, label)
            return True
        return False

    def _viewpoint_targets(self, traj, vp):
        return self._arrival_targets_from_traj(traj, vp)


def main(args=None):
    rclpy.init(args=args)
    node = KawasakiInspectionNode()

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
