import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('viewpoint_planner'), 'config', 'sick_tmini_params.yaml'
    )

    mesh_path_arg = DeclareLaunchArgument(
        'mesh_path',
        default_value='/home/cem/colcon_ws/src/Universal_Robots_ROS2_Description/meshes/ur10e/collision/chassis.stl',
        description='Path to the chassis.stl file'
    )

    coverage_threshold_arg = DeclareLaunchArgument(
        'coverage_threshold',
        default_value='0.98',
        description='Target coverage threshold (0.0 to 1.0)'
    )

    output_plan_file_arg = DeclareLaunchArgument(
        'output_plan_file',
        default_value='/home/cem/colcon_ws/src/viewpoint_planner/plans/viewpoint_plan.json',
        description='Where to write the generated plan JSON. The executor reads it from '
                    'here and keeps its trajectory cache in <plans>/trajectories/.')
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description=(
            'Path to the camera/reachability parameters yaml '
            '(default: config/sick_tmini_params.yaml, based on the SICK '
            'Visionary-T Mini V3S145-1AAAAAA datasheet)'
        )
    )

    planner_node = Node(
        package='viewpoint_planner',
        executable='viewpoint_planner_node',
        name='viewpoint_planner_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'mesh_path': LaunchConfiguration('mesh_path'),
                'output_plan_file': LaunchConfiguration('output_plan_file'),
                'coverage_threshold': LaunchConfiguration('coverage_threshold'),
            },
        ]
    )

    visualizer_node = Node(
        package='viewpoint_planner',
        executable='viewpoint_visualizer',
        name='viewpoint_visualizer',
        output='screen'
    )

    return LaunchDescription([
        mesh_path_arg,
        output_plan_file_arg,
        coverage_threshold_arg,
        config_file_arg,
        planner_node,
        visualizer_node
    ])
