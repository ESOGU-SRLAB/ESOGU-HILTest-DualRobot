#!/usr/bin/env python3
"""
HIL Test System: Fixed version with proper synchronization for linear axis
Runs 10 complete loops of all trajectories
"""

import json
import os
import copy
from threading import Thread, Event, Lock
import time
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
import math

# MoveIt2 libraries
from pymoveit2_sim import MoveIt2 as MoveIt2_Sim
from pymoveit2_real import MoveIt2 as MoveIt2_Real
from pymoveit2_sim.robots import ur as simrobot
from pymoveit2_real.robots import ur as realrobot

class SynchronizedTrajectoryManager:
    """Synchronized trajectory management for HIL testing"""
   
    def __init__(self, sim_file="sim_trajectories.json", real_file="real_trajectories.json"):
        self.sim_file = sim_file
        self.real_file = real_file
        self.sim_trajectories = {}
        self.real_trajectories = {}
   
    def load_all_trajectories(self):
        """Load both simulation and real robot trajectories"""
        sim_loaded = self.load_trajectories(self.sim_file, 'sim')
        real_loaded = self.load_trajectories(self.real_file, 'real')
        return sim_loaded and real_loaded
   
    def load_trajectories(self, file_path, robot_type):
        """Load trajectories from file"""
        if os.path.exists(file_path):
            with open(file_path, 'r') as f:
                data = json.load(f)
                if robot_type == 'sim':
                    self.sim_trajectories = data
                else:
                    self.real_trajectories = data
            return True
        return False
   
    def get_trajectory_pair(self, name):
        """Get matching trajectories for both robots"""
        sim_traj = self.sim_trajectories.get(name)
        real_traj = self.real_trajectories.get(name)
        return sim_traj, real_traj

class HILSynchronizedController(Node):
    def __init__(self):
        super().__init__("hil_synchronized_controller")
        cb_group = ReentrantCallbackGroup()

        # --- Simulation MoveIt2 setup ---
        try:
            self.sim_moveit = MoveIt2_Sim(
                node=self,
                joint_names=simrobot.joint_names(),
                base_link_name="world",
                end_effector_name=simrobot.end_effector_name(),
                group_name=simrobot.MOVE_GROUP_ARM,
                callback_group=cb_group,
            )
            self.sim_moveit.planner_id = "RRTstarkConfigDefault"
            self.sim_moveit.max_velocity = 0.1
            self.sim_moveit.max_acceleration = 0.1
            self.sim_available = True
            self.get_logger().info("✅ Simulation MoveIt2 initialized.")
        except Exception as e:
            self.sim_moveit = None
            self.sim_available = False
            self.get_logger().error(f"❌ Simulation MoveIt2 init failed: {e}")

        # --- Real MoveIt2 setup ---
        try:
            self.real_moveit = MoveIt2_Real(
                node=self,
                joint_names=realrobot.joint_names(),
                base_link_name="world",
                end_effector_name=realrobot.end_effector_name(),
                group_name=realrobot.MOVE_GROUP_ARM,
                callback_group=cb_group,
            )
            self.real_moveit.planner_id = "RRTstarkConfigDefault"
            self.real_moveit.max_velocity = 0.05
            self.real_moveit.max_acceleration = 0.05
            self.real_available = True
            self.get_logger().info("✅ Real MoveIt2 initialized.")
        except Exception as e:
            self.real_moveit = None
            self.real_available = False
            self.get_logger().error(f"❌ Real MoveIt2 init failed: {e}")

        self.traj_manager = SynchronizedTrajectoryManager()

    # ===================================================================
    # JOINT STATE HELPERS
    # ===================================================================
    def get_fresh_joint_state(self, moveit_interface, timeout=3.0):
        start = time.time()
        samples = []
        while time.time() - start < timeout and len(samples) < 3:
            if moveit_interface.joint_state:
                joints = [
                    moveit_interface.joint_state.position[
                        moveit_interface.joint_state.name.index(n)
                    ]
                    if n in moveit_interface.joint_state.name else 0.0
                    for n in moveit_interface.joint_names
                ]
                samples.append(joints)
            time.sleep(0.1)
        return samples[-1] if samples else None

    def wait_for_joint_states(self, timeout=10.0):
        start = time.time()
        while time.time() - start < timeout:
            sim_ready = not self.sim_available or (self.sim_moveit and self.sim_moveit.joint_state)
            real_ready = not self.real_available or (self.real_moveit and self.real_moveit.joint_state)
            if sim_ready and real_ready:
                time.sleep(0.5)
                return True
            time.sleep(0.1)
        return False

    # ===================================================================
    # TRAJECTORY RECONSTRUCTION
    # ===================================================================
    def reconstruct_trajectory_from_dict(self, traj_dict, joint_names):
        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
        from builtin_interfaces.msg import Duration

        msg = JointTrajectory()
        msg.joint_names = joint_names

        for pt in traj_dict["points"]:
            p = JointTrajectoryPoint()
            p.positions = pt["positions"]
            p.velocities = pt.get("velocities", [0.0] * len(joint_names))
            p.accelerations = pt.get("accelerations", [0.0] * len(joint_names))
            p.time_from_start = Duration(
                sec=pt["time_from_start"]["sec"], nanosec=pt["time_from_start"]["nanosec"]
            )
            msg.points.append(p)
        return msg

    # ===================================================================
    # PHYSICAL STABILITY WAIT
    # ===================================================================
    def wait_for_physical_motion_done(self, moveit_interface, tol=0.001, stable_time=1.0, timeout=90.0):
        start = time.time()
        last = self.get_fresh_joint_state(moveit_interface)
        stable_since = None
        while time.time() - start < timeout:
            current = self.get_fresh_joint_state(moveit_interface)
            if not current:
                continue
            diff = max(abs(a - b) for a, b in zip(current, last))
            last = current
            if diff < tol:
                if stable_since is None:
                    stable_since = time.time()
                elif time.time() - stable_since > stable_time:
                    self.get_logger().info("✅ Physical motion stabilized.")
                    return True
            else:
                stable_since = None
            time.sleep(0.25)
        return False

    # ===================================================================
    # SYNCHRONIZED EXECUTION
    # ===================================================================
    def execute_synchronized_trajectory(self, traj_name):
        sim_dict, real_dict = self.traj_manager.get_trajectory_pair(traj_name)
        if not sim_dict or not real_dict:
            self.get_logger().error(f"❌ Missing trajectory: {traj_name}")
            return False

        if not self.wait_for_joint_states():
            self.get_logger().error("❌ Joint state timeout.")
            return False

        sim_traj = (
            self.reconstruct_trajectory_from_dict(sim_dict, self.sim_moveit.joint_names)
            if self.sim_available
            else None
        )
        real_traj = (
            self.reconstruct_trajectory_from_dict(real_dict, self.real_moveit.joint_names)
            if self.real_available
            else None
        )

        self.get_logger().info(f"▶️ Starting synchronized execution: {traj_name}")
        start_time = time.time()

        # --- Senkronizasyon event’leri ---
        from threading import Lock, Event, Thread
        ready_lock = Lock()
        ready_count = 0
        execute_event = Event()
        total_threads = int(self.sim_available) + int(self.real_available)

        def exec_sim():
            nonlocal ready_count
            with ready_lock:
                ready_count += 1
            execute_event.wait()
            self.sim_moveit.execute(sim_traj)   # ✅ async_execute yerine execute()

        def exec_real():
            nonlocal ready_count
            with ready_lock:
                ready_count += 1
            execute_event.wait()
            self.real_moveit.execute(real_traj)  # ✅ async_execute yerine execute()

        if self.sim_available:
            Thread(target=exec_sim, daemon=True).start()
        if self.real_available:
            Thread(target=exec_real, daemon=True).start()

        while ready_count < total_threads:
            time.sleep(0.05)

        execute_event.set()
        time.sleep(0.5)  # Goal gönderimi için küçük gecikme

        sim_ok, real_ok = True, True

        if self.sim_available:
            try:
                sim_ok = self.sim_moveit.wait_until_executed()
                self.get_logger().info("✅ Sim motion completed.")
            except Exception as e:
                self.get_logger().warn(f"⚠️ Sim motion error: {e}")
                sim_ok = False

        if self.real_available:
            try:
                real_ok = self.real_moveit.wait_until_executed()
                physical_done = self.wait_for_physical_motion_done(self.real_moveit)
                real_ok = real_ok and physical_done
            except Exception as e:
                self.get_logger().warn(f"⚠️ Real motion error: {e}")
                real_ok = False

        total = time.time() - start_time
        self.get_logger().info(f"⏱ Trajectory {traj_name} finished in {total:.2f}s")
        return sim_ok and real_ok

def main():
    rclpy.init()
    controller = HILSynchronizedController()
   
    if not controller.sim_available and not controller.real_available:
        controller.get_logger().error("Neither simulation nor real robot available. Exiting.")
        return
   
    if not controller.traj_manager.load_all_trajectories():
        controller.get_logger().error("❌ Could not load trajectory files.")
        return

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(controller)
    Thread(target=executor.spin, daemon=True).start()

    time.sleep(3.0)

   
    # Move to home position
    # home_joints = [1.0, 0.0, -math.pi/2, 0.0, -math.pi/2, 0.0, 0.0]
    # controller.get_logger().info("Moving to home position...")
    # if not controller.move_to_joints_synchronized(home_joints):
    #     controller.get_logger().error("Failed to move to home position")
        # Continue anyway for debugging
   
    time.sleep(3.0)
   
    # Trajectory sequence
    trajectory_names = [
        "trajectory_home_to_pose1_top",
        "trajectory_pose1_top_to_middle",
        "trajectory_pose1_middle_to_below",
        "trajectory_pose1_to_pose2",
        "trajectory_pose2_down_to_left",
        "trajectory_pose2_left_to_up",
        "trajectory_pose2_up_to_right",
        "trajectory_pose2_right_to_front",
        "trajectory_pose2_to_pose3",
        "trajectory_pose3_down_to_left",
        "trajectory_pose3_left_to_up",
        "trajectory_pose3_up_to_right",
        "trajectory_pose3_right_to_front",
        "trajectory_pose3_to_intermediate",
        "trajectory_intermediate_to_pose4_top",
        "trajectory_pose4_top_to_middle",
        "trajectory_pose4_middle_to_below",
        "trajectory_pose4_to_pose5",
        "trajectory_pose5_down_to_left",
        "trajectory_pose5_left_to_up",
        "trajectory_pose5_up_to_right",
        "trajectory_pose5_to_below_intermediate",
        "trajectory_below_intermediate_to_pose6",
        "trajectory_pose6_down_to_left",
        "trajectory_pose6_left_to_up",
        "trajectory_pose6_up_to_right",
        "trajectory_return_to_start"
    ]
   
    try:
        for i in range(1, 11):
            controller.get_logger().info(f"=== LOOP {i}/10 ===")
            for name in trajectory_names:
                controller.execute_synchronized_trajectory(name)
                time.sleep(1.5)
    except KeyboardInterrupt:
        controller.get_logger().info("Stopped by user.")
    finally:
        controller.get_logger().info("Shutting down...")
        executor.remove_node(controller)
        rclpy.shutdown()


if __name__ == "__main__":
    main()