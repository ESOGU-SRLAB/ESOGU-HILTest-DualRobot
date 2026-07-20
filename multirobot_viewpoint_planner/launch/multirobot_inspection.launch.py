import os
from typing import List

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
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
    only_ur_arg = DeclareLaunchArgument(
        'only_ur', default_value='false',
        description='Real-world testing: true -> run the UR10e ONLY (Kawasaki idle).')
    only_kawasaki_arg = DeclareLaunchArgument(
        'only_kawasaki', default_value='false',
        description='Real-world testing: true -> run the Kawasaki ONLY (UR10e idle). '
                    'Both false (default) -> cooperative multi-robot run.')
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
    use_trajectory_cache_arg = DeclareLaunchArgument(
        'use_trajectory_cache', default_value='true',
        description='true -> replay each viewpoint\'s saved trajectory from '
                    '<plans>/trajectories/ if valid (deterministic, pre-validated paths); '
                    'plan+save on first run / cache miss. false -> always plan fresh with MoveIt.')
    force_replan_arg = DeclareLaunchArgument(
        'force_replan', default_value='false',
        description='true -> ignore and OVERWRITE any cached trajectory (rebuild the cache '
                    'from scratch, e.g. after changing collision padding or the SRDF).')
    home_before_viewpoints_arg = DeclareLaunchArgument(
        'home_before_viewpoints', default_value="[]",
        description='Viewpoint ids that must NOT be approached directly from the previous '
                    'viewpoint. The arm detours via its start/home pose, then walks to that '
                    'viewpoint\'s RECORDED trajectory start; BOTH hops are cached like normal '
                    'trajectories, and the viewpoint keeps its own recorded path. Empty by '
                    'default: the detour existed to dodge a cable-channel pinch on the hop '
                    'into ur_vp_009, which is no longer needed now that the linear axis '
                    'tracks the trajectory properly. Set e.g. "[\'ur_vp_009\']" to re-enable.')
    viewpoint_joint_overrides_arg = DeclareLaunchArgument(
        'viewpoint_joint_overrides',
        default_value="['ur_vp_009:0.088,-308,-185,-15,-256,-277,210',"
                      "'ur_vp_010:0.279,-308,-160,-23,-302,-320,233']",
        description='Hand-tuned joint targets that REPLACE a viewpoint\'s planned goal, as '
                    '"<vp_id>:<axis0_m>,<j1_deg>,...,<j6_deg>" (the start/home pose '
                    'convention: index 0 in METRES, the rest in DEGREES). ur_vp_009 and '
                    'ur_vp_010 are overridden because the planner\'s poses pinch a physical '
                    'cable channel that is absent from the collision model. An overridden '
                    'viewpoint is planned in JOINT space, is still captured, and still '
                    'caches its trajectory. Set "[]" to use the planned poses everywhere.')

    # Parameters shared by both single-arm inspection nodes. Each node declares its
    # own full param set (with defaults); these are the launch-exposed overrides.
    # only_ur / only_kawasaki / enable_kawasaki are resolved INSIDE each node
    # (should_run): the "wrong" arm's node self-idles and exits.
    common_params = {
        'plan_file': LaunchConfiguration('plan_file'),
        'only_sim': LaunchConfiguration('only_sim'),
        'output_base_dir': LaunchConfiguration('output_base_dir'),
        'enable_kawasaki': LaunchConfiguration('enable_kawasaki'),
        'only_ur': LaunchConfiguration('only_ur'),
        'only_kawasaki': LaunchConfiguration('only_kawasaki'),
        'allowed_planning_time': LaunchConfiguration('allowed_planning_time'),
        'num_planning_attempts': LaunchConfiguration('num_planning_attempts'),
        'plan_attempts': LaunchConfiguration('plan_attempts'),
        'add_ground_plane': LaunchConfiguration('add_ground_plane'),
        'ground_plane_z': LaunchConfiguration('ground_plane_z'),
        'collision_padding': LaunchConfiguration('collision_padding'),
        'use_trajectory_cache': LaunchConfiguration('use_trajectory_cache'),
        'force_replan': LaunchConfiguration('force_replan'),
        'home_before_viewpoints': ParameterValue(
            LaunchConfiguration('home_before_viewpoints'), value_type=List[str]),
        'viewpoint_joint_overrides': ParameterValue(
            LaunchConfiguration('viewpoint_joint_overrides'), value_type=List[str]),
    }

    # Two independent single-arm nodes (separate processes -> parallel by construction).
    ur_node = Node(
        package='multirobot_viewpoint_planner',
        executable='ur_inspection_node',
        name='ur_inspection_node',
        output='screen',
        parameters=[common_params],
    )
    kawasaki_node = Node(
        package='multirobot_viewpoint_planner',
        executable='kawasaki_inspection_node',
        name='kawasaki_inspection_node',
        output='screen',
        parameters=[common_params],
    )

    # RViz waypoint visualizer alongside: draws BOTH arms' viewpoint arrows
    # (UR green->red, Kawasaki dark->light blue) from the same plan JSON.
    visualizer_node = Node(
        package='multirobot_viewpoint_planner',
        executable='multirobot_viewpoint_visualizer',
        name='multirobot_viewpoint_visualizer',
        output='screen',
        parameters=[{
            'plan_file': LaunchConfiguration('plan_file'),
        }],
    )

    return LaunchDescription([
        plan_file_arg, only_sim_arg, output_base_dir_arg, enable_kawasaki_arg,
        only_ur_arg, only_kawasaki_arg,
        allowed_planning_time_arg, num_planning_attempts_arg, plan_attempts_arg,
        add_ground_plane_arg, ground_plane_z_arg, collision_padding_arg,
        use_trajectory_cache_arg, force_replan_arg, home_before_viewpoints_arg,
        viewpoint_joint_overrides_arg,
        ur_node, kawasaki_node, visualizer_node,
    ])
