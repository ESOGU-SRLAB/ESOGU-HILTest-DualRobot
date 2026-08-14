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

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from control_msgs.action import FollowJointTrajectory
from scipy.spatial.transform import Rotation

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
        # These two MUST match multirobot_planner_node's values of the same name, or the
        # executor's IK targets a different pose than the planner solved for.
        self.declare_parameter("kawasaki_view_axis_correction_rpy",
                               [-1.5707963, -1.5707963, 0.0])
        self.declare_parameter("kawasaki_group_tip", "link6")
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
        # Return home after the last viewpoint, as the UR has always done. Homing this
        # arm also commands the AGV, which is why it used to be opt-out; keep
        # return_home_kawasaki:=false for a run where the AGV must not move at the end.
        self.declare_parameter("return_home_kawasaki", True)
        # HOME == START, deliberately, mirroring the UR (whose ur_home_joint_positions
        # and start_pose_ur are the same seven values). The old default put the rail at
        # 0.0 m -- a metre away from the 1.0 m the run starts at -- so simply enabling
        # homing would have ended every run by driving the slow AGV somewhere the plan
        # never chose. Set the rail element back to 0.0 here if the AGV's physical dock,
        # rather than its start, is what "home" should mean.
        self.declare_parameter("kawasaki_home_joint_positions", [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        self.arm_label = "Kawasaki"
        self.start_pose_param = "start_pose_kawasaki"
        self.return_home_param = "return_home_kawasaki"
        self.home_positions_param = "kawasaki_home_joint_positions"
        self.align_on_fresh = True  # preserve the existing Kawasaki behaviour
        # Same playback trick the UR has used since its travel was fixed: rebase the
        # recorded path's first point onto the measured pose instead of driving a
        # separate align hop to it. _prepare_start unwinds whole turns out of the path
        # first, so on this arm the rebase only ever has to absorb a small residual --
        # and it refuses (falling back to the explicit walk) if the gap is larger.
        self.rebase_to_current = True

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
            # See the matching note in ur_inspection_node: the executor owns unwinding
            # here (cache policy tag, recorded-path unwinding, raw last-resort attempt),
            # so MoveIt2's own pass is turned off to avoid duplicating it.
            unwind_joint_goals=False,
        )
        # Enables nearest-branch IK in the base class (see _ik_target below).
        self.ik_group_name = "real_kawasaki"
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

    # --- Nearest-branch IK ------------------------------------------------ #
    # The Kawasaki stays JOINT-SPACE: the stored goal remains the primary target and the
    # AGV rail is never re-chosen here. What this adds is a search among the OTHER IK
    # solutions of the very same camera pose, so that when the arm is not where the plan
    # assumed (first viewpoint out of home, a home detour, a skipped viewpoint, a cached
    # path being realigned) it can take the nearest branch instead of the stored one.
    # The pose is identical in every candidate, so the captured cloud cannot change.
    #
    # `vp['rotation']` is the GENERATOR's view frame (+Z view axis). Turning it into
    # something IK can use needs exactly the chain multirobot_planner_node uses in
    # _make_kawasaki_reach_fn, and the two MUST stay in step or the camera aims wrong:
    #   R_cam = R_view @ C          C = view-axis correction (+Z view -> camera +X view)
    #   R_tip = R_cam @ R_off       (R_off, p_off) = pose of link6 IN the camera frame
    #   p_tip = pos + R_cam @ p_off
    # Solving for the group's own tip (link6) rather than the camera frame is deliberate:
    # MoveIt does not honour an ik_link past the group tip, which is the bug that once
    # left the Kawasaki camera facing the wrong way.

    @staticmethod
    def _camera_to_tip():
        """(R_off, p_off): pose of link6 IN camera_rgb_optic_frame, straight from the
        URDF constants in rs005l_macro.xacro (NOT from TF -- the unprefixed frames are
        absent from TF during a sim run, and a silent lookup failure here would aim the
        camera wrongly). Mirrors _analytic_kawasaki_offset in the planner.

            link6 --joint7 (rpy 0 0 -1.96)--> link7
                  --camera_joint (xyz .0145 .0125 .185, rpy 0 -pi/2 pi/2)--> camera
        """
        def rpy(r, p, y):
            return Rotation.from_euler("xyz", [r, p, y]).as_matrix()
        R67 = rpy(0.0, 0.0, -1.96)
        R7c = rpy(0.0, -np.pi / 2.0, np.pi / 2.0)
        p7c = np.array([0.0145, 0.0125, 0.185])
        R6c = R67 @ R7c
        p6c = R67 @ p7c
        return R6c.T, -R6c.T @ p6c

    def _ik_target(self, vp):
        if vp.get("position") is None or vp.get("rotation") is None:
            return None
        C = Rotation.from_euler(
            "xyz", list(self.get_parameter("kawasaki_view_axis_correction_rpy").value)
        ).as_matrix()
        R_off, p_off = self._camera_to_tip()
        R_cam = np.asarray(vp["rotation"], dtype=float) @ C
        pos = np.asarray(vp["position"], dtype=float)
        R_tip = R_cam @ R_off
        p_tip = pos + R_cam @ p_off
        quat = Rotation.from_matrix(R_tip).as_quat()  # xyzw
        return [float(v) for v in p_tip], quat, self.get_parameter(
            "kawasaki_group_tip").value

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
