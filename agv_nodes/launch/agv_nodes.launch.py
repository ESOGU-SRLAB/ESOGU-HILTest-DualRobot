"""Launch file for AGV nodes."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    rosbridge_host_arg = DeclareLaunchArgument(
        'rosbridge_host',
        default_value='192.168.3.6',
        description='Rosbridge WebSocket sunucu adresi (Noetic PC)'
    )

    rosbridge_port_arg = DeclareLaunchArgument(
        'rosbridge_port',
        default_value='9090',
        description='Rosbridge WebSocket portu'
    )

    bridge_node = Node(
        package='agv_nodes',
        executable='bridge_node',
        name='agv_bridge_node',
        output='screen',
        parameters=[{
            'rosbridge_host': LaunchConfiguration('rosbridge_host'),
            'rosbridge_port': LaunchConfiguration('rosbridge_port'),
        }]
    )

    position_node = Node(
        package='agv_nodes',
        executable='position_node',
        name='agv_position_node',
        output='screen',
        parameters=[{
            'print_rate': 2.0,
        }]
    )

    movement_node = Node(
        package='agv_nodes',
        executable='movement_node',
        name='agv_movement_node',
        output='screen',
        parameters=[{
            'rosbridge_host': LaunchConfiguration('rosbridge_host'),
            'rosbridge_port': LaunchConfiguration('rosbridge_port'),
        }]
    )

    return LaunchDescription([
        rosbridge_host_arg,
        rosbridge_port_arg,
        bridge_node,
        position_node,
        movement_node,
    ])
