#!/usr/bin/env python3
"""
HIL Test System: Synchronized trajectory playback with trajectory sharing approach
"""

import json
import os
import copy
from threading import Thread
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
        
        callback_group = ReentrantCallbackGroup()
        
        # Initialize simulation robot
        self.sim_moveit = MoveIt2_Sim(
            node=self,
            joint_names=simrobot.joint_names(),
            base_link_name="world",
            end_effector_name=simrobot.end_effector_name(),
            group_name=simrobot.MOVE_GROUP_ARM,
            callback_group=callback_group,
        )
        
        # Initialize real robot
        self.real_moveit = MoveIt2_Real(
            node=self,
            joint_names=realrobot.joint_names(),
            base_link_name="world",
            end_effector_name=realrobot.end_effector_name(),
            group_name=realrobot.MOVE_GROUP_ARM,
            callback_group=callback_group,
        )
        
        # Set planning parameters
        self.real_moveit.planner_id = "RRTConnectkConfigDefault"
        self.real_moveit.max_velocity = 0.05
        self.real_moveit.max_acceleration = 0.05

        self.sim_moveit.planner_id = "RRTConnectkConfigDefault"
        self.sim_moveit.max_velocity = 0.07
        self.sim_moveit.max_acceleration = 0.07
        
        self.traj_manager = SynchronizedTrajectoryManager()
        self.get_logger().info("HIL Synchronized Controller initialized")

    def reconstruct_trajectory_from_dict(self, traj_dict, joint_names):
        """Reconstruct ROS trajectory message from dictionary"""
        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
        from builtin_interfaces.msg import Duration
        
        trajectory = JointTrajectory()
        trajectory.joint_names = joint_names
        
        for point_data in traj_dict['points']:
            point = JointTrajectoryPoint()
            point.positions = point_data['positions']
            point.velocities = point_data.get('velocities', [0.0]*len(joint_names))
            point.accelerations = point_data.get('accelerations', [0.0]*len(joint_names))
            
            point.time_from_start = Duration()
            point.time_from_start.sec = point_data['time_from_start']['sec']
            point.time_from_start.nanosec = point_data['time_from_start']['nanosec']
            
            trajectory.points.append(point)
        
        return trajectory

    def execute_synchronized_trajectory(self, trajectory_name, max_retries=3):
        """Use MoveIt's native multi-robot execution"""
        sim_traj_dict, real_traj_dict = self.traj_manager.get_trajectory_pair(trajectory_name)
        if not sim_traj_dict or not real_traj_dict:
            return False
        
        # Reconstruct trajectories
        sim_traj = self.reconstruct_trajectory_from_dict(sim_traj_dict, self.sim_moveit.joint_names)
        real_traj = copy.deepcopy(sim_traj)
        real_traj.joint_names = self.real_moveit.joint_names
        
        # Patch real trajectory starting point
        real_joint_state = self.real_moveit.joint_state
        if real_joint_state and real_traj.points:
            joint_positions_map = {name: pos for name, pos in zip(real_joint_state.name, real_joint_state.position)}
            real_current_joints = [joint_positions_map.get(name, 0.0) for name in self.real_moveit.joint_names]
            real_traj.points[0].positions = real_current_joints
            real_traj.points[0].velocities = [0.0] * len(self.real_moveit.joint_names)
            real_traj.points[0].accelerations = [0.0] * len(self.real_moveit.joint_names)
        
        # Execute with precise timing
        self.get_logger().info(f"Starting synchronized execution of {trajectory_name}")
        
        # Start both executions at exactly the same time
        start_time = time.time()
        
        # Execute sim
        self.sim_moveit.execute(sim_traj)
        
        # Small delay to ensure first execution started
        time.sleep(0.01)
        
        # Execute real  
        self.real_moveit.execute(real_traj)
        
        # Wait for both to complete
        sim_success = self.sim_moveit.wait_until_executed()
        real_success = self.real_moveit.wait_until_executed()
        
        execution_time = time.time() - start_time
        self.get_logger().info(f"Execution completed in {execution_time:.2f}s - Sim: {sim_success}, Real: {real_success}")
        
        return sim_success and real_success

    def move_to_joints_synchronized(self, joint_positions):
        """Move both robots sequentially"""
        self.get_logger().info("Planning trajectory for sim robot...")
        
        # Wait for joint states
        max_wait = 5
        wait_start = time.time()
        while not self.sim_moveit.joint_state or not self.real_moveit.joint_state:
            if time.time() - wait_start > max_wait:
                self.get_logger().error("Timeout waiting for joint states")
                return False
            time.sleep(0.1)
        
        # Plan for sim robot
        sim_traj = self.sim_moveit.plan(joint_positions=joint_positions)
        if not sim_traj:
            self.get_logger().error("Sim trajectory planning failed")
            return False
        
        # Copy and patch for real robot
        real_traj = copy.deepcopy(sim_traj)
        real_traj.joint_names = self.real_moveit.joint_names
        
        real_joint_state = self.real_moveit.joint_state
        if real_joint_state and real_traj.points:
            joint_positions_map = {name: pos for name, pos in zip(real_joint_state.name, real_joint_state.position)}
            real_current_joints = [joint_positions_map.get(name, 0.0) for name in self.real_moveit.joint_names]
            real_traj.points[0].positions = real_current_joints
            real_traj.points[0].velocities = [0.0] * len(self.real_moveit.joint_names)
            real_traj.points[0].accelerations = [0.0] * len(self.real_moveit.joint_names)
        
        # Execute sequentially instead of parallel
        self.get_logger().info("Executing sim robot first...")
        self.sim_moveit.execute(sim_traj)
        sim_success = self.sim_moveit.wait_until_executed()
        
        self.get_logger().info("Executing real robot...")
        self.real_moveit.execute(real_traj)
        real_success = self.real_moveit.wait_until_executed()
        
        self.get_logger().info(f"Results - Sim: {sim_success}, Real: {real_success}")
        return sim_success and real_success

def main():
    rclpy.init()
    controller = HILSynchronizedController()
    
    executor = rclpy.executors.MultiThreadedExecutor(4)
    executor.add_node(controller)
    executor_thread = Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    
    time.sleep(2.0)
    
    # Load trajectories
    if not controller.traj_manager.load_all_trajectories():
        controller.get_logger().error("Could not load trajectory files.")
        return
    
    # Move to home position
    home_joints = [0.0, -math.pi/2, 0.0, -math.pi/2, 0.0, 0.0]
    controller.get_logger().info("Moving to home position...")
    if not controller.move_to_joints_synchronized(home_joints):
        controller.get_logger().error("Failed to move to home position")
        return
    
    time.sleep(2.0)
    
    # Trajectory sequence
    trajectory_names = [
        "trajectory_home_0",
        "trajectory_0_1",
        "trajectory_1_intermediate", 
        "trajectory_intermediate_2",
        "trajectory_2_3",
        "trajectory_return_to_start"
    ]
    
    try:
        loop_counter = 0
        while rclpy.ok():
            loop_counter += 1
            controller.get_logger().info(f"=== HIL LOOP {loop_counter} ===")
            
            all_success = True
            for traj_name in trajectory_names:
                if loop_counter > 1 and traj_name == "trajectory_home_0":
                    continue
                    
                success = controller.execute_synchronized_trajectory(traj_name)
                if not success:
                    all_success = False
                time.sleep(0.5)
            
            if all_success:
                controller.get_logger().info("✅ Loop completed successfully")
            else:
                controller.get_logger().warning("⚠️ Loop completed with errors")
            
            time.sleep(1.0)
    except KeyboardInterrupt:
        controller.get_logger().info("HIL test stopped by user")
    finally:
        controller.get_logger().info("Returning to home position...")
        controller.move_to_joints_synchronized(home_joints)
        rclpy.shutdown()
        executor_thread.join()

if __name__ == "__main__":
    main()