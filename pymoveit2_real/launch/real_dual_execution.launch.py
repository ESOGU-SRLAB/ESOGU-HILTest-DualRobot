import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    nodes = []

    # The Python Orchestrator handling UR MoveIt2 integration and Sequence sync
    orchestrator_node = Node(
        package="pymoveit2_real",
        executable="real_dual_executor.py",
        name="real_dual_executor",
        output="screen",
        emulate_tty=True,
    )

    # The C++ Arm Executor bypassing ros2_control to talk to Kawasaki via TCP
    kawasaki_arm_node = Node(
        package="sir_robot_ros_interface",
        executable="KawasakiArmNode",
        name="kawasaki_arm_executor",
        output="screen",
        emulate_tty=True,
    )

    nodes.append(orchestrator_node)
    nodes.append(kawasaki_arm_node)

    return LaunchDescription(nodes)
