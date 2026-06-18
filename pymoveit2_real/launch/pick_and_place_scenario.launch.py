from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pick_and_place_node = Node(
        package="pymoveit2_real",
        executable="pick_and_place_scenario.py",
        name="pick_and_place_scenario",
        output="screen",
        emulate_tty=True,
    )

    return LaunchDescription([pick_and_place_node])
