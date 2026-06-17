#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_kafka_bridge',
            executable='double_ros2_pub_sub',
            name='double_ros2_pub_sub_node',
            output='screen'
        ),
        Node(
            package='ros2_kafka_bridge',
            executable='double_kafka_pro_con',
            name='double_kafka_pro_con_node',
            output='screen'
        ),
        Node(
            package='ros2_kafka_bridge',
            executable='double_ros2_kafka_bridge',
            name='double_ros2_kafka_bridge_node',
            output='screen'
        ),
    ])
