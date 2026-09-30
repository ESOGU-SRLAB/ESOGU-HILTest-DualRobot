import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    plan_file_arg = DeclareLaunchArgument(
        'plan_file',
        default_value='/home/cem/colcon_ws/src/viewpoint_planner/plans/viewpoint_plan.json',
        description='Path to the generated plan JSON file'
    )

    executor_node = Node(
        package='viewpoint_planner',
        executable='viewpoint_executor_node',
        name='viewpoint_executor_node',
        output='screen',
        parameters=[{
            'plan_file': LaunchConfiguration('plan_file')
        }]
    )
    
    visualizer_node = Node(
        package='viewpoint_planner',
        executable='viewpoint_visualizer',
        name='viewpoint_visualizer',
        output='screen'
    )
    
    return LaunchDescription([
        plan_file_arg,
        executor_node,
        visualizer_node
    ])
