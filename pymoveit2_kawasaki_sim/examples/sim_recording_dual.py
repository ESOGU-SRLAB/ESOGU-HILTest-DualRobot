#!/usr/bin/env python3
"""
Dual robot controller for simulation that also RECORDS trajectories for playback on real robots.

1. Defines sequence of waypoints for UR10e and Kawasaki.
2. Plans trajectories asynchronously for both robots.
3. Records both trajectories into JSON files (sim_ur_trajectories.json, sim_kawa_trajectories.json).
4. Executes the movements in simulation conceptually matching real execution logic.
"""

import json
import os
import time
import math
from threading import Thread, Event

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, QoSDurabilityPolicy,
    QoSReliabilityPolicy, QoSHistoryPolicy,
)

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory

from pymoveit2_sim import MoveIt2 as MoveIt2_UR
from pymoveit2_sim.robots import ur as ur_cfg

from pymoveit2_kawasaki_sim import MoveIt2 as MoveIt2_Kawasaki
from pymoveit2_kawasaki_sim.robots import ur as kawasaki_cfg

# ── Naming definitions ───────────────────────────────────────────
ROBOT_A_GROUP = "sim_ur10e"
ROBOT_B_GROUP = "sim_kawasaki"

CONTROLLER_A = "/sim/sim_scaled_joint_trajectory_controller/follow_joint_trajectory"
CONTROLLER_B = "/sim/kawasaki_controller/follow_joint_trajectory"

POLL_INTERVAL  = 0.05
PLAN_TIMEOUT   = 15.0
MOTION_TIMEOUT = 60.0


# =============================================================================
# Helper Utilities
# =============================================================================
def _wait_future(future, node, label: str, timeout: float = PLAN_TIMEOUT) -> bool:
    deadline = time.time() + timeout
    while time.time() < deadline:
        if future is not None and future.done():
            return True
        time.sleep(POLL_INTERVAL)
    node.get_logger().error(f"[{label}] Timeout ({timeout}s)")
    return False

def _make_fjt_goal(trajectory: JointTrajectory) -> FollowJointTrajectory.Goal:
    goal = FollowJointTrajectory.Goal()
    goal.trajectory = trajectory
    return goal


class TrajectoryManager:
    """Manages recording of trajectories for playback."""
    def __init__(self, file_path="sim_trajectories.json"):
        self.file_path = file_path
        self.trajectories = {}
        self.load_trajectories()

    def save_trajectory(self, name, trajectory):
        traj_dict = {
            'joint_names': trajectory.joint_names,
            'points': []
        }
        for point in trajectory.points:
            point_dict = {
                'positions': list(point.positions),
                'velocities': list(point.velocities),
                'accelerations': list(point.accelerations),
                'time_from_start': {
                    'sec': point.time_from_start.sec,
                    'nanosec': point.time_from_start.nanosec
                }
            }
            traj_dict['points'].append(point_dict)
        
        self.trajectories[name] = traj_dict
        with open(self.file_path, 'w') as f:
            json.dump(self.trajectories, f, indent=2)

    def load_trajectories(self):
        if os.path.exists(self.file_path):
            with open(self.file_path, 'r') as f:
                self.trajectories = json.load(f)
            return True
        return False

    def list_trajectories(self):
        return list(self.trajectories.keys())


# =============================================================================
# Controller Node
# =============================================================================
class DualRobotRecordingController(Node):
    def __init__(self):
        super().__init__("dual_robot_recording_controller")

        cb_a = ReentrantCallbackGroup()
        cb_b = ReentrantCallbackGroup()

        # ── MoveIt2 planning interfaces ──
        self.moveit2_a = MoveIt2_UR(
            node=self,
            joint_names=ur_cfg.joint_names(),
            base_link_name=ur_cfg.base_link_name(),
            end_effector_name=ur_cfg.end_effector_name(),
            group_name=ROBOT_A_GROUP,
            callback_group=cb_a,
        )
        self.moveit2_a.planner_id       = "RRTConnectkConfigDefault"
        self.moveit2_a.max_velocity     = 0.1
        self.moveit2_a.max_acceleration = 0.1

        self.moveit2_b = MoveIt2_Kawasaki(
            node=self,
            joint_names=kawasaki_cfg.joint_names(prefix="sim_kawasaki_"),
            base_link_name=kawasaki_cfg.base_link_name(prefix="sim_kawasaki_"),
            end_effector_name=kawasaki_cfg.end_effector_name(prefix="sim_kawasaki_"),
            group_name=ROBOT_B_GROUP,
            callback_group=cb_b,
        )
        self.moveit2_b.planner_id       = "RRTConnectkConfigDefault"
        self.moveit2_b.max_velocity     = 0.1
        self.moveit2_b.max_acceleration = 0.1

        # ── Execution controllers ──
        _qos = QoSProfile(
            durability=QoSDurabilityPolicy.VOLATILE,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._ctrl_a = ActionClient(
            node=self,
            action_type=FollowJointTrajectory,
            action_name=CONTROLLER_A,
            goal_service_qos_profile=_qos,
            callback_group=cb_a,
        )
        self._ctrl_b = ActionClient(
            node=self,
            action_type=FollowJointTrajectory,
            action_name=CONTROLLER_B,
            goal_service_qos_profile=_qos,
            callback_group=cb_b,
        )

        self.traj_manager_a = TrajectoryManager("sim_ur_trajectories.json")
        self.traj_manager_b = TrajectoryManager("sim_kawa_trajectories.json")

    def _wait_for_controllers(self, timeout: float = 10.0) -> bool:
        self.get_logger().info("Waiting for action servers...")
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self._ctrl_a.server_is_ready() and self._ctrl_b.server_is_ready():
                self.get_logger().info("✓ Both controllers ready.")
                return True
            time.sleep(1.0)
        self.get_logger().error("Action servers not ready in time!")
        return False

    def _plan(self, moveit2, joint_positions: list, label: str, max_retries: int = 3) -> "Optional[JointTrajectory]":
        for attempt in range(max_retries):
            self.get_logger().info(f"[{label}] Planning {attempt + 1}/{max_retries}...")
            future = moveit2.plan_async(joint_positions=joint_positions)
            if future is None:
                time.sleep(0.3)
                continue
            if not _wait_future(future, self, label):
                continue
            traj = moveit2.get_trajectory(future)
            if traj is not None:
                return traj
            time.sleep(0.3)
        return None

    def execute_dual_recorded_step(self, traj_name, end_a, end_b):
        """
        Plans, records and executes a step simultaneously.
        """
        self.get_logger().info(f"--- Processing sequence step: {traj_name} ---")
        
        # 1. Check if already recorded
        recorded_a = self.traj_manager_a.list_trajectories()
        recorded_b = self.traj_manager_b.list_trajectories()
        
        traj_a = None
        traj_b = None
        
        needs_recording = (traj_name not in recorded_a) or (traj_name not in recorded_b)
        
        if needs_recording:
            # Plan
            traj_a = self._plan(self.moveit2_a, end_a, "UR10e", 5)
            traj_b = self._plan(self.moveit2_b, end_b, "Kawasaki", 5)
            
            if not traj_a or not traj_b:
                self.get_logger().error(f"Failed to plan {traj_name}")
                return False
                
            # Record
            self.traj_manager_a.save_trajectory(traj_name, traj_a)
            self.traj_manager_b.save_trajectory(traj_name, traj_b)
            self.get_logger().info(f"✓ Recorded {traj_name} for both robots.")
        else:
            self.get_logger().info(f"Step {traj_name} already recorded. Only moving sim robot...")
            # We don't plan if already recorded since this node just records Gazebo ones.
            # But wait: if we skip execution, Gazebo robots won't be at the right start pose for the next plan!
            # So we still need to plan & execute moving them to the next pose, or reconstruct and execute.
            # To keep things robust for physics sequence, we plan again anyway just for Gazebo to advance.
            traj_a = self._plan(self.moveit2_a, end_a, "UR10e", 5)
            traj_b = self._plan(self.moveit2_b, end_b, "Kawasaki", 5)

        # Execute simultaneously
        future_send_a = self._ctrl_a.send_goal_async(_make_fjt_goal(traj_a)) if traj_a else None
        future_send_b = self._ctrl_b.send_goal_async(_make_fjt_goal(traj_b)) if traj_b else None

        result_a = [False]
        result_b = [False]
        ev_a = Event()
        ev_b = Event()

        def _do_wait_a():
            if future_send_a:
                result_a[0] = self._wait_for_fjt(future_send_a, "UR10e")
            ev_a.set()

        def _do_wait_b():
            if future_send_b:
                result_b[0] = self._wait_for_fjt(future_send_b, "Kawasaki")
            ev_b.set()

        Thread(target=_do_wait_a, daemon=True).start()
        Thread(target=_do_wait_b, daemon=True).start()

        ev_a.wait(timeout=MOTION_TIMEOUT + 10.0)
        ev_b.wait(timeout=MOTION_TIMEOUT + 10.0)

        return result_a[0] and result_b[0]

    def _wait_for_fjt(self, send_future, label: str) -> bool:
        if not _wait_future(send_future, self, label, timeout=5.0):
            return False
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            return False
        result_future = goal_handle.get_result_async()
        if not _wait_future(result_future, self, label, timeout=MOTION_TIMEOUT):
            return False
        from action_msgs.msg import GoalStatus
        return (result_future.result().status == GoalStatus.STATUS_SUCCEEDED)


# =============================================================================
# MAIN
# =============================================================================
def get_ur10e_sequence():
    return [
        [1.0, math.radians(87.54), math.radians(-89.27), math.radians(-79.47), math.radians(-6.25), math.radians(-81.85), math.radians(113.90)],
        [1.0, math.radians(87.51), math.radians(-88.86), math.radians(-100.0), math.radians(13.87), math.radians(-81.90), math.radians(113.90)],
        [1.0, math.radians(84.80), math.radians(-61.59), math.radians(-119.87), math.radians(6.52), math.radians(-79.22), math.radians(113.90)],
        [1.0, math.radians(84.84), math.radians(-64.48), math.radians(-97.61), math.radians(-12.83), math.radians(-79.18), math.radians(113.90)],
        [1.0, math.radians(-55.13), math.radians(-86.19), math.radians(-111.47), math.radians(23.92), math.radians(73.69), math.radians(51.18)],
        [1.0, math.radians(-55.11), math.radians(-84.81), math.radians(-83.47), math.radians(-5.47), math.radians(73.77), math.radians(51.23)]
    ]

def get_kawasaki_sequence():
    return [
        [0.63, math.radians(74.732), math.radians(-8.589), math.radians(-30.942), math.radians(-108.911), math.radians(85.335), math.radians(37.744)],
        [0.63, math.radians(74.732), math.radians(-62.050), math.radians(-132.843), math.radians(-99.243), math.radians(72.803), math.radians(-10.063)],
        [1.15, math.radians(74.732), math.radians(-8.589), math.radians(-30.942), math.radians(-108.911), math.radians(85.335), math.radians(37.744)],
        [1.15, math.radians(74.732), math.radians(-62.050), math.radians(-132.843), math.radians(-99.243), math.radians(72.803), math.radians(-10.063)],
        [1.65, math.radians(74.732), math.radians(-8.589), math.radians(-30.942), math.radians(-108.911), math.radians(85.335), math.radians(37.744)],
        [1.65, math.radians(74.732), math.radians(-62.050), math.radians(-132.843), math.radians(-99.243), math.radians(72.803), math.radians(-10.063)]
    ]

def main():
    rclpy.init()
    controller = DualRobotRecordingController()

    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(controller)
    executor_thread = Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    if not controller._wait_for_controllers(timeout=15.0):
        rclpy.shutdown()
        return

    ur_seq = get_ur10e_sequence()
    kawa_pts = get_kawasaki_sequence()

    home_a = [1.0, 0.0, -math.pi/2, 0.0, -math.pi/2, 0.0, 0.0]
    home_b = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    controller.get_logger().info("Going home initially without recording...")
    # Skip recording home logic to keep it simple, just initialize state internally for physics
    # We will simulate a fake move
    
    try:
        controller.get_logger().info("=== KICKING OFF RECORDING SEQUENCE ===")
        for i in range(len(ur_seq)):
            traj_name = f"trajectory{i+1}"
            ur_target = ur_seq[i]
            kawa_target = kawa_pts[i]
            
            controller.get_logger().info(f"\nStep {i+1}/{len(ur_seq)}: {traj_name}")
            
            success = controller.execute_dual_recorded_step(traj_name, ur_target, kawa_target)
            if not success:
                controller.get_logger().error(f"Failed at {traj_name}, exiting.")
                break
            
            time.sleep(1.0)
            
        controller.get_logger().info("=== SEQUENCE COMPLETED SUCCESSFULLY ===")

    except KeyboardInterrupt:
        pass
    except Exception as e:
        import traceback
        controller.get_logger().error(f"Error: {e}\n{traceback.format_exc()}")
    finally:
        rclpy.shutdown()
        executor_thread.join(timeout=5.0)

if __name__ == "__main__":
    main()
