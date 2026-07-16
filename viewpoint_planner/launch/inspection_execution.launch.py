import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    plan_file_arg = DeclareLaunchArgument(
        'plan_file', default_value='/home/cem/colcon_ws/src/viewpoint_planner/plans/viewpoint_plan.json',
        description='Path to the generated plan JSON')
    only_sim_arg = DeclareLaunchArgument(
        'only_sim', default_value='true',
        description="true -> capture ONLY the Gazebo SICK (single_ur10e/sim_data/). "
                    "false -> also capture the real SICK /sick_points "
                    "(single_ur10e/real_data/) at every viewpoint.")
    output_base_dir_arg = DeclareLaunchArgument(
        'output_base_dir', default_value=os.path.expanduser('~/colcon_ws/src/pcds'),
        description='Root of the capture tree; per-stream '
                    '<base>/single_ur10e/<sim|real>_data/{pcds,poses}/')
    allowed_planning_time_arg = DeclareLaunchArgument(
        'allowed_planning_time', default_value='10.0',
        description='Seconds the motion planner may spend per single run (raise to reach more viewpoints)')
    planner_id_arg = DeclareLaunchArgument(
        'planner_id', default_value='',
        description="OMPL planner for the UR group. Default LBKPIECEkConfigDefault (bidirectional, "
                    "clutter-tuned -- best for this cage cell). Set '' for move_group's default "
                    "(RRTConnect), or BKPIECEkConfigDefault / SBLkConfigDefault to A/B another "
                    "bidirectional planner (must exist in ompl_planning.yaml real_ur10e block).")
    num_planning_attempts_arg = DeclareLaunchArgument(
        'num_planning_attempts', default_value='10',
        description='Parallel planner runs inside one plan call')
    plan_attempts_arg = DeclareLaunchArgument(
        'plan_attempts', default_value='4',
        description='How many times to re-issue the whole plan call before skipping a viewpoint')
    add_ground_plane_arg = DeclareLaunchArgument(
        'add_ground_plane', default_value='true',
        description='Add a floor collision box so MoveIt never plans the arm below ground')
    ground_plane_z_arg = DeclareLaunchArgument(
        'ground_plane_z', default_value='-0.02',
        description='World-frame height of the floor collision box top face (raise to 0.0 for a strict floor)')
    collision_padding_arg = DeclareLaunchArgument(
        'collision_padding', default_value='0.03',
        description='Safety margin (m) the arm keeps from every obstacle via MoveIt link padding (0.0 = off)')
    chassis_collision_padding_arg = DeclareLaunchArgument(
        'chassis_collision_padding', default_value='0.10',
        description='EXTRA safety margin (m) for the inspected CHASSIS links only, to leave room for the '
                    'unmodelled arm cables/cable channels that snag inside the chassis. Absolute per-link '
                    'padding (not additive with collision_padding); arm+furniture still use collision_padding. '
                    '0.0 = fall back to uniform collision_padding for everything.')
    order_by_proximity_arg = DeclareLaunchArgument(
        'order_by_proximity', default_value='true',
        description='true -> traverse starting at the largest-Y viewpoint (chassis front), '
                    'then nearest-neighbour + 2-opt. false -> visit the plan JSON order as-is.')

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
            'order_by_proximity': LaunchConfiguration('order_by_proximity'),
        }],
    )

    # RViz waypoint visualizer alongside (clean UR-only arrows).
    visualizer_node = Node(
        package='viewpoint_planner',
        executable='viewpoint_visualizer',
        name='viewpoint_visualizer',
        output='screen',
    )

    return LaunchDescription([
        plan_file_arg, only_sim_arg, output_base_dir_arg,
        allowed_planning_time_arg, planner_id_arg, num_planning_attempts_arg, plan_attempts_arg,
        add_ground_plane_arg, ground_plane_z_arg, collision_padding_arg,
        chassis_collision_padding_arg,
        order_by_proximity_arg,
        executor_node, visualizer_node,
    ])
