"""Plan a combined coverage tour over BOTH car doors and write it to
doors_inspection/plans/doors_viewpoint_plan.json.

Runs the SAME multirobot_planner_node as the chassis job: it is already
mesh-agnostic, so the only door-specific things here are the merged target mesh,
mesh_scale=1.0 (the door STLs are metres, not millimetres) and this package's
parameter file.

Call the service once the node is up:
    ros2 service call /plan_multirobot_viewpoints std_srvs/srv/Trigger
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from doors_inspection.merge_door_meshes import DEFAULT_DOORS, DEFAULT_OUTPUT, ensure_merged

PLANS_DIR = '/home/cem/colcon_ws/src/doors_inspection/plans'


def _launch_setup(context, *args, **kwargs):
    # Build the merged target mesh BEFORE the planner starts. Done here rather than
    # as a separate process so the ordering is guaranteed: no race between the merge
    # finishing and the planner reading the file. It is a no-op (a stat call) unless
    # a source STL changed.
    merged = ensure_merged(
        sources=[p for p in LaunchConfiguration('doors').perform(context).split(',') if p],
        output=LaunchConfiguration('merged_mesh').perform(context),
        force=LaunchConfiguration('force_merge').perform(context).lower() in ('true', '1'),
    )

    planner_node = Node(
        package='multirobot_viewpoint_planner',
        executable='multirobot_planner_node',
        name='doors_planner_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'mesh_path': merged,
                # THE door-specific value. windowdoor.stl / windowlessdoor.stl are
                # already in metres (the URDF loads them with scale="1 1 1"),
                # unlike chassis.stl which is millimetres.
                'mesh_scale': 1.0,
                'coverage_threshold': LaunchConfiguration('coverage_threshold'),
                'output_plan_file': LaunchConfiguration('output_plan_file'),
            },
        ],
    )

    visualizer_node = Node(
        package='multirobot_viewpoint_planner',
        executable='multirobot_viewpoint_visualizer',
        name='doors_viewpoint_visualizer',
        output='screen',
        parameters=[{'plan_file': LaunchConfiguration('output_plan_file')}],
    )

    return [planner_node, visualizer_node]


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('doors_inspection'), 'config', 'doors_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'doors', default_value=','.join(DEFAULT_DOORS),
            description='Comma-separated source door STLs merged into one target mesh.'),
        DeclareLaunchArgument(
            'merged_mesh', default_value=DEFAULT_OUTPUT,
            description='Where the merged door mesh is cached (a build artefact, ~45 MB, '
                        'rebuilt automatically whenever a source STL is newer).'),
        DeclareLaunchArgument(
            'force_merge', default_value='false',
            description='true -> rebuild the merged mesh even if it looks up to date.'),
        DeclareLaunchArgument(
            'config_file', default_value=default_config,
            description='Camera / reachability / allocation parameters.'),
        DeclareLaunchArgument(
            'coverage_threshold', default_value='0.98',
            description='Stop once 98% of the reachable targets are covered. Lowered '
                        'from 0.992 after the first real run: with the Kawasaki capped '
                        'at 3 viewpoints the last tour only reached 96.84%, so 0.992 '
                        'never fired and the tail cutter ended the run instead. Passed '
                        'AFTER the config file and therefore overrides it.'),
        DeclareLaunchArgument(
            'output_plan_file',
            default_value=os.path.join(PLANS_DIR, 'doors_viewpoint_plan.json'),
            description='Where to write the generated doors plan JSON.'),
        OpaqueFunction(function=_launch_setup),
    ])
