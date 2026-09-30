import os
from typing import List

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    plan_file_arg = DeclareLaunchArgument(
        'plan_file', default_value='/home/cem/colcon_ws/src/viewpoint_planner/plans/viewpoint_plan.json',
        description='Path to the generated plan JSON. The trajectory cache lives next '
                    'to it, in <plans>/trajectories/.')
    only_sim_arg = DeclareLaunchArgument(
        'only_sim', default_value='true',
        description="true -> capture ONLY the Gazebo SICK (single_ur10e/sim_data/). "
                    "false -> also capture the real SICK /sick_points "
                    "(single_ur10e/real_data/) at every viewpoint.")
    output_base_dir_arg = DeclareLaunchArgument(
        'output_base_dir', default_value=os.path.expanduser('~/colcon_ws/src/pcds'),
        description='Root of the capture tree; per-stream '
                    '<base>/single_ur10e/<sim|real>_data/{pcds,poses}/')

    # --- Planning ---------------------------------------------------------- #
    allowed_planning_time_arg = DeclareLaunchArgument(
        'allowed_planning_time', default_value='10.0',
        description='Seconds the motion planner may spend per single run')
    planner_id_arg = DeclareLaunchArgument(
        'planner_id', default_value='',
        description="OMPL planner for the UR group. Empty (default) -> move_group's own "
                    "fallback, RRTConnect, which is what every recorded trajectory so far "
                    "was planned with. LBKPIECEkConfigDefault is the one measured "
                    "alternative worth an A/B (shorter paths, same reliability, ~1.2 s "
                    "per hop instead of 0.15). Must exist in the real_ur10e block of "
                    "ompl_planning.yaml.")
    num_planning_attempts_arg = DeclareLaunchArgument(
        'num_planning_attempts', default_value='10',
        description='Parallel planner runs inside one plan call; MoveIt keeps the '
                    'shortest. Measured on the two-arm cell: 1 -> 10 attempts cut arm '
                    'travel by ~22% for ~0.15 s of extra planning.')
    plan_attempts_arg = DeclareLaunchArgument(
        'plan_attempts', default_value='4',
        description='How many times to re-issue the whole plan call before skipping a viewpoint')

    # --- Planning scene ---------------------------------------------------- #
    add_ground_plane_arg = DeclareLaunchArgument(
        'add_ground_plane', default_value='true',
        description='Add a floor collision box so MoveIt never plans the arm below ground')
    ground_plane_z_arg = DeclareLaunchArgument(
        'ground_plane_z', default_value='-0.02',
        description='World-frame height of the floor collision box top face')
    collision_padding_arg = DeclareLaunchArgument(
        'collision_padding', default_value='0.04',
        description='Safety margin (m) applied uniformly to every ur10e_* link -- arm, '
                    'furniture AND chassis -- via MoveIt link padding (0.0 = off). 0.04 is '
                    'the multirobot cell value. MUST match the planner-side value, or the '
                    'plan contains viewpoints the executor cannot reach.')
    chassis_collision_padding_arg = DeclareLaunchArgument(
        'chassis_collision_padding', default_value='0.0',
        description='OPTIONAL separate margin (m) for the chassis links only. 0.0 '
                    '(default) = the chassis gets collision_padding like everything else, '
                    'which is exactly what the multirobot cell does. Was 0.10 until '
                    '2026-09-10; set to match the multirobot cell.')

    # --- Trajectory cache -------------------------------------------------- #
    use_trajectory_cache_arg = DeclareLaunchArgument(
        'use_trajectory_cache', default_value='true',
        description="true -> replay each viewpoint's saved trajectory from "
                    '<plans>/trajectories/ if it is still valid (deterministic, '
                    'pre-validated paths); false -> plan every viewpoint live.')
    force_replan_arg = DeclareLaunchArgument(
        'force_replan', default_value='false',
        description='true -> ignore and OVERWRITE any cached trajectory. Needed after '
                    'anything the cache cannot notice by itself: a URDF/SRDF geometry '
                    'change, different padding, a new ground plane. Deleting a single '
                    'trajectories/ur_<vp_id>.json is the per-viewpoint equivalent.')

    # --- Goal selection ---------------------------------------------------- #
    use_pose_goal_arg = DeclareLaunchArgument(
        'use_pose_goal', default_value='false',
        description="false (default) -> drive the joint configuration stored in the plan. "
                    "The planner's joint-space ordering picks every viewpoint's IK branch "
                    'so the WHOLE chain is short, and validates each hop with a real '
                    'plan; following it is what keeps the arm out of reconfigurations it '
                    'cannot plan. true -> plan to the camera pose and let a per-stop '
                    'branch search choose, which drifts off that chain. Same camera pose '
                    'either way, so the captured cloud is identical.')
    wrap_goals_to_current_arg = DeclareLaunchArgument(
        'wrap_goals_to_current', default_value='true',
        description='true -> before planning, re-express every revolute goal as the '
                    "2*pi-equivalent NEAREST the arm's current measured angle, where the "
                    'URDF limits allow. Same physical pose, far less travel.')
    nearest_branch_ik_arg = DeclareLaunchArgument(
        'nearest_branch_ik', default_value='false',
        description='true -> solve IK ourselves for the viewpoint camera pose, seeded at '
                    'the measured pose, and take the closest branch that yields a path. '
                    'Off by default here: it is GREEDY (nearest from wherever the arm is '
                    'now) and, on 2026-09-10, walked the single UR off the planned chain '
                    'into 537-757 deg reconfigurations. The planner now makes that choice '
                    'for the whole tour instead.')
    branch_plan_candidates_arg = DeclareLaunchArgument(
        'branch_plan_candidates', default_value='3',
        description='How many nearest-branch candidates to actually PLAN before picking '
                    'one. Candidates are ranked by goal-space distance, which is only a '
                    'proxy for the path the planner returns. 1 restores '
                    'first-that-plans. Costs planning time only while RECORDING.')
    ik_random_seeds_arg = DeclareLaunchArgument(
        'ik_random_seeds', default_value='0',
        description='How many RANDOM in-limits arm postures to add to the branch-IK seed '
                    "list per viewpoint. The UR's pick_ik already restarts randomly in "
                    "'mode: global', so 0 is right here; the Kawasaki's KDL needed 12.")
    ik_random_seed_arg = DeclareLaunchArgument(
        'ik_random_seed', default_value='0',
        description='RNG seed behind ik_random_seeds, so a recording run is repeatable.')

    # --- Start / home ------------------------------------------------------ #
    go_to_start_arg = DeclareLaunchArgument(
        'go_to_start', default_value='true',
        description='Send the arm to its fixed START pose before the sequence, so the '
                    'first (possibly cached) trajectory begins from the state it was '
                    'recorded at.')
    return_home_arg = DeclareLaunchArgument(
        'return_home', default_value='true',
        description='Send the arm back to its home pose after the last viewpoint.')
    home_before_viewpoints_arg = DeclareLaunchArgument(
        'home_before_viewpoints', default_value="[]",
        description='Viewpoint ids that must NOT be approached directly from the previous '
                    'viewpoint. The arm detours via its start/home pose, then walks to '
                    "that viewpoint's recorded trajectory start. Both hops are recorded "
                    "transitions. Example: \"['ur_vp_009']\".")

    # --- Motion ------------------------------------------------------------ #
    ur_velocity_arg = DeclareLaunchArgument(
        'ur_velocity', default_value='0.1',
        description='MoveIt velocity scaling factor (fraction of the joint_limits.yaml '
                    'limits, not a speed).')
    ur_acceleration_arg = DeclareLaunchArgument(
        'ur_acceleration', default_value='0.1',
        description='Acceleration scaling factor.')
    execute_via_move_group_arg = DeclareLaunchArgument(
        'execute_via_move_group', default_value='true',
        description="true -> execute through move_group's execute_trajectory action (the "
                    'proven path; keeps the rail smooth). false -> push a raw '
                    'FollowJointTrajectory goal straight at the controller, which is what '
                    'this node did before 2026-09-09 and what made the rail lurch on the '
                    'two-arm cell.')

    executor_node = Node(
        package='viewpoint_planner',
        executable='inspection_executor_node',
        name='inspection_executor_node',
        output='screen',
        parameters=[{
            'plan_file': LaunchConfiguration('plan_file'),
            'only_sim': LaunchConfiguration('only_sim'),
            'output_base_dir': LaunchConfiguration('output_base_dir'),
            'allowed_planning_time': LaunchConfiguration('allowed_planning_time'),
            'planner_id': LaunchConfiguration('planner_id'),
            'num_planning_attempts': LaunchConfiguration('num_planning_attempts'),
            'plan_attempts': LaunchConfiguration('plan_attempts'),
            'add_ground_plane': LaunchConfiguration('add_ground_plane'),
            'ground_plane_z': LaunchConfiguration('ground_plane_z'),
            'collision_padding': LaunchConfiguration('collision_padding'),
            'chassis_collision_padding': LaunchConfiguration('chassis_collision_padding'),
            'use_trajectory_cache': LaunchConfiguration('use_trajectory_cache'),
            'force_replan': LaunchConfiguration('force_replan'),
            'use_pose_goal': LaunchConfiguration('use_pose_goal'),
            'wrap_goals_to_current': LaunchConfiguration('wrap_goals_to_current'),
            'nearest_branch_ik': LaunchConfiguration('nearest_branch_ik'),
            'branch_plan_candidates': LaunchConfiguration('branch_plan_candidates'),
            'ik_random_seeds': LaunchConfiguration('ik_random_seeds'),
            'ik_random_seed': LaunchConfiguration('ik_random_seed'),
            'go_to_start': LaunchConfiguration('go_to_start'),
            'return_home': LaunchConfiguration('return_home'),
            'home_before_viewpoints': ParameterValue(
                LaunchConfiguration('home_before_viewpoints'), value_type=List[str]),
            'ur_velocity': LaunchConfiguration('ur_velocity'),
            'ur_acceleration': LaunchConfiguration('ur_acceleration'),
            'execute_via_move_group': LaunchConfiguration('execute_via_move_group'),
        }],
    )

    # RViz waypoint visualizer alongside (UR-only arrows), watching the same plan.
    visualizer_node = Node(
        package='viewpoint_planner',
        executable='viewpoint_visualizer',
        name='viewpoint_visualizer',
        output='screen',
        parameters=[{
            'plan_file': LaunchConfiguration('plan_file'),
        }],
    )

    return LaunchDescription([
        plan_file_arg, only_sim_arg, output_base_dir_arg,
        allowed_planning_time_arg, planner_id_arg, num_planning_attempts_arg,
        plan_attempts_arg,
        add_ground_plane_arg, ground_plane_z_arg, collision_padding_arg,
        chassis_collision_padding_arg,
        use_trajectory_cache_arg, force_replan_arg,
        use_pose_goal_arg, wrap_goals_to_current_arg, nearest_branch_ik_arg,
        branch_plan_candidates_arg, ik_random_seeds_arg, ik_random_seed_arg,
        go_to_start_arg, return_home_arg, home_before_viewpoints_arg,
        ur_velocity_arg, ur_acceleration_arg, execute_via_move_group_arg,
        executor_node, visualizer_node,
    ])
