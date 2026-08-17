from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    human_robot_collaboration_node = Node(
        package="pymoveit2_real",
        executable="human_robot_collaboration_scenario.py",
        name="human_robot_collaboration_scenario",
        output="screen",
        emulate_tty=True,
    )

    return LaunchDescription([human_robot_collaboration_node])
