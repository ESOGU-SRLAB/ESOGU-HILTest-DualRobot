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
        executable="sensing_robot.py",
        name="harmony_sensing_robot",
        output="screen",
        emulate_tty=True,
        parameters=[{
            "fixed_frame": "world",
            # Fake hardware / digital twin testinde real tarafı yavaşlatma.
            # 13.0 → 1.0: sim ve real aynı hızda gider, 60 sn'lik real timeout aşılmaz.
            # Gerçek fiziksel robota (use_fake_hardware:=false) geçince 2-3'e çek.
            "bridge_time_scale": 1.0,
            "bridge_wait_real": True,
            "real_robot_velocity": 0.2,
            "real_robot_acceleration": 0.2,
        }],
    )

    red_detection_node = Node(
        package="pymoveit2_sim",
        executable="red_detection_node.py",
        name="harmony_red_detection",
        output="screen",
        emulate_tty=True,
        parameters=[{"world_frame": "world"}],
    )

    cleaning_runner_node = Node(
    package="pymoveit2_sim",
    executable="cleaning_mission_runner.py",
    name="harmony_cleaning_runner",
    output="screen",
    emulate_tty=True,
    parameters=[{
        "fixed_frame": "world",
        # sensing_robot ile aynı: fake/digital twin testinde real'i yavaşlatma.
        "bridge_time_scale": 1.0,
        "bridge_wait_real": True,
        "real_robot_velocity": 0.2,
        "real_robot_acceleration": 0.2,
    }],
)


    return LaunchDescription([
        # start_watcher_action,
        sensing_robot_node,
        red_detection_node,
        cleaning_runner_node,
    ])
