import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    plan_file_arg = DeclareLaunchArgument(
        'plan_file', default_value='/home/cem/colcon_ws/src/multirobot_viewpoint_planner/plans/multirobot_viewpoint_plan.json',
        description='Path to the generated multi-robot plan JSON')
    only_sim_arg = DeclareLaunchArgument(
        'only_sim', default_value='true',
        description="true -> capture ONLY the two Gazebo SICKs (ur + kawasaki). "
                    "false -> also capture the real SICKs (/sick_points, /kawasaki/pointcloud).")
    output_base_dir_arg = DeclareLaunchArgument(
        'output_base_dir', default_value=os.path.expanduser('~/colcon_ws/src/pcds'),
        description='Root of the capture tree; <base>/<sim|real>_pcds/<ur|kawasaki>_data/{pcds,poses}')
    enable_kawasaki_arg = DeclareLaunchArgument(
        'enable_kawasaki', default_value='true',
        description='Move the Kawasaki to its own coverage viewpoints and capture its camera')
    allowed_planning_time_arg = DeclareLaunchArgument(
        'allowed_planning_time', default_value='5.0',
        description='Seconds the motion planner may spend per single run')
    num_planning_attempts_arg = DeclareLaunchArgument(
        'num_planning_attempts', default_value='10',
        description='Parallel planner runs inside one plan call')
    plan_attempts_arg = DeclareLaunchArgument(
        'plan_attempts', default_value='4',
        description='How many times to re-issue the whole plan call before skipping a viewpoint')
    add_ground_plane_arg = DeclareLaunchArgument(
        'add_ground_plane', default_value='true',
        description='Add a floor collision box so MoveIt never plans an arm below ground')
    ground_plane_z_arg = DeclareLaunchArgument(
        'ground_plane_z', default_value='-0.02',
        description='World-frame height of the floor collision box top face')
    collision_padding_arg = DeclareLaunchArgument(
        'collision_padding', default_value='0.04',
        description='Safety margin (m) the arms keep from every obstacle (0.0 = off). '
                    'Applied uniformly to both arms: UR (ur10e_*) and Kawasaki (link1..link6).')

    executor_node = Node(
        package='multirobot_viewpoint_planner',
        executable='multirobot_executor_node',
        name='multirobot_executor_node',
        output='screen',
        parameters=[{
            'plan_file': LaunchConfiguration('plan_file'),
            'only_sim': LaunchConfiguration('only_sim'),
            'output_base_dir': LaunchConfiguration('output_base_dir'),
            'enable_kawasaki': LaunchConfiguration('enable_kawasaki'),
            'allowed_planning_time': LaunchConfiguration('allowed_planning_time'),
            'num_planning_attempts': LaunchConfiguration('num_planning_attempts'),
            'plan_attempts': LaunchConfiguration('plan_attempts'),
            'add_ground_plane': LaunchConfiguration('add_ground_plane'),
            'ground_plane_z': LaunchConfiguration('ground_plane_z'),
            'collision_padding': LaunchConfiguration('collision_padding'),
        }],
    )

    return LaunchDescription([
        plan_file_arg, only_sim_arg, output_base_dir_arg, enable_kawasaki_arg,
        allowed_planning_time_arg, num_planning_attempts_arg, plan_attempts_arg,
        add_ground_plane_arg, ground_plane_z_arg, collision_padding_arg,
        executor_node,
    ])
