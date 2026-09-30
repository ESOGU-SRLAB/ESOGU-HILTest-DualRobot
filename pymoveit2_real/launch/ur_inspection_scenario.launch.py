from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ur_inspection_node = Node(
        package="pymoveit2_real",
        executable="ur_inspection_scenario.py",
        name="ur_inspection_scenario",
        output="screen",
        emulate_tty=True,
    )

    return LaunchDescription([ur_inspection_node])
