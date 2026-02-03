#!/usr/bin/env python3
"""
Launch file to start both UR robot (real_robot_joint_goal) and Kawasaki robot (ROS2KawasakiRobotMove) nodes
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # UR Robot Node (Python) - real_robot_joint_goal
    ur_robot_node = Node(
        package="pymoveit2_real",
        executable="real_robot_joint_goal.py",
        name="ur_robot_controller",
        output="screen",
        emulate_tty=True,
        parameters=[
            {"use_sim_time": False}
        ],
    )
    
    # Kawasaki Robot Node (C++) - ROS2KawasakiRobotMove
    kawasaki_robot_node = Node(
        package="sir_robot_ros_interface",
        executable="ROS2KawasakiRobotMove",
        name="kawasaki_robot_controller",
        output="screen",
        emulate_tty=True,
        parameters=[
            {"use_sim_time": False}
        ],
    )
    
    # Create launch description
    ld = LaunchDescription()
    
    # Add nodes to launch description
    ld.add_action(ur_robot_node)
    ld.add_action(kawasaki_robot_node)
    
    return ld
