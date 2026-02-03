#!/usr/bin/env python3
"""
HARMONY Launch (example-like)
- sensing_robot + red_detection hemen başlar
- watcher: /harmony/robot_status state==WAITING görünce cleaning_mission_runner başlatır
"""

import subprocess
import threading
import time
import json

from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node

import rclpy
from std_msgs.msg import String



def generate_launch_description():
    sensing_robot_node = Node(
        package="pymoveit2_sim",
        executable="sensing_robot_v2.py",
        name="harmony_sensing_robot",
        output="screen",
        emulate_tty=True,
        parameters=[{"fixed_frame": "world"}],
    )

    red_detection_node = Node(
        package="pymoveit2_sim",
        executable="red_detection_node_v2.py",
        name="harmony_red_detection",
        output="screen",
        emulate_tty=True,
        parameters=[{"world_frame": "world"}],
    )

    cleaning_runner_node = Node(
    package="pymoveit2_sim",          
    executable="cleaning_mission_runner_v2.py",  
    name="harmony_cleaning_runner",
    output="screen",
    emulate_tty=True,
    parameters=[{"fixed_frame": "world"}],
)


    return LaunchDescription([
        # start_watcher_action,
        sensing_robot_node,
        red_detection_node,
        cleaning_runner_node,
    ])
