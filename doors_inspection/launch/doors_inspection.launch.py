"""Execute the doors plan on both arms.

This does NOT re-implement the executors. It includes multirobot_viewpoint_planner's
inspection launch with the doors plan and a doors output tree, so the UR and Kawasaki
nodes, the trajectory cache, the 2*pi unwinding and the nearest-branch IK are exactly
the ones already validated on the chassis job -- and any later fix there lands here too.

Two paths follow the plan file automatically and therefore stay separate from the
chassis job's:
  * the trajectory cache -> doors_inspection/plans/trajectories/
  * the captured clouds  -> <output_base_dir>/{sim,real}_pcds/{ur,kawasaki}_data/

Typical use:
    ros2 launch doors_inspection doors_inspection.launch.py
    ros2 launch doors_inspection doors_inspection.launch.py only_sim:=false
    ros2 launch doors_inspection doors_inspection.launch.py force_replan:=true
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

PLANS_DIR = '/home/cem/colcon_ws/src/doors_inspection/plans'

# Arguments forwarded verbatim to the upstream inspection launch. Kept as a list so
# adding a knob there needs one line here, not a new block.
FORWARDED = [
    ('only_sim', 'true',
     'true -> capture only the two Gazebo SICKs; false -> also the real ones.'),
    ('enable_kawasaki', 'true', 'Run the Kawasaki as well as the UR.'),
    ('only_ur', 'false', 'true -> UR only, Kawasaki idle.'),
    ('only_kawasaki', 'false', 'true -> Kawasaki only, UR idle.'),
    ('allowed_planning_time', '5.0', 'Seconds per planning run.'),
    ('num_planning_attempts', '10', 'Parallel planner runs inside one plan call.'),
    ('plan_attempts', '4', 'Re-issues of the whole plan call before skipping a viewpoint.'),
    ('add_ground_plane', 'true', 'Add a floor collision box.'),
    ('ground_plane_z', '-0.02', 'World Z of the floor box top face.'),
    ('collision_padding', '0.04', 'Safety margin (m) both arms keep from every obstacle.'),
    ('use_trajectory_cache', 'true', 'Replay saved trajectories when they are still valid.'),
    ('force_replan', 'false', 'Ignore and overwrite the cache (use after a scene change).'),
    ('wrap_goals_to_current', 'true', 'Re-express goals as the nearest 2*pi equivalent.'),
    ('nearest_branch_ik', 'true', 'Go to the closest IK branch of the same camera pose.'),
    ('branch_max_rail_shift', '0.0', 'Metres a branch may move the AGV rail (0 locks it).'),
    ('branch_plan_candidates', '3',
     'Plan this many branch candidates and keep the shortest ACTUAL path (1 = the old '
     'first-that-plans behaviour).'),
    # DOORS-SPECIFIC: 0.03 instead of the 0.015 the chassis job runs at, i.e. double
    # speed -- 9 deg/s on joint1-3 and 3.7 cm/s on the AGV rail. Overridden here rather
    # than in the node so the validated chassis job is untouched.
    ('kawasaki_velocity', '0.03',
     'MoveIt velocity scaling factor for the Kawasaki (fraction of its joint limits).'),
    ('kawasaki_acceleration', '0.03',
     'Acceleration scaling factor. A no-op today: joint_limits.yaml declares no '
     'acceleration limits, so there is nothing to scale. Kept in step with the '
     'velocity factor.'),
    ('return_home_ur', 'true', 'Send the UR home after the last viewpoint.'),
    ('return_home_kawasaki', 'true',
     'Send the Kawasaki home after the last viewpoint. Home is its start pose, so the '
     'AGV rail drives back too.'),
    # Empty again: the kawa_vp_002 home detour was dropped after the real run showed the
    # direct hop is fine on hardware. It had cost ~1.5 min (2 m of extra rail travel plus
    # 13 rad of extra joint travel) for no measured benefit. Set to "['kawa_vp_002']" to
    # bring it back, or name any other viewpoint that must be approached via home rather
    # than straight from its predecessor.
    ('home_before_viewpoints', "[]",
     'Viewpoint ids approached via the home pose instead of directly from the previous '
     'viewpoint.'),
]


def generate_launch_description():
    upstream = os.path.join(
        get_package_share_directory('multirobot_viewpoint_planner'),
        'launch', 'multirobot_inspection.launch.py')

    args = [
        DeclareLaunchArgument(
            'plan_file',
            default_value=os.path.join(PLANS_DIR, 'doors_viewpoint_plan.json'),
            description='Doors plan produced by doors_planning.launch.py.'),
        DeclareLaunchArgument(
            'output_base_dir',
            default_value=os.path.expanduser('~/colcon_ws/src/pcds/doors'),
            description='Root of the doors capture tree; kept apart from the chassis '
                        'captures so the two jobs never overwrite each other.'),
    ]
    args += [DeclareLaunchArgument(n, default_value=d, description=h)
             for n, d, h in FORWARDED]

    include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(upstream),
        launch_arguments=[('plan_file', LaunchConfiguration('plan_file')),
                          ('output_base_dir', LaunchConfiguration('output_base_dir'))]
        + [(n, LaunchConfiguration(n)) for n, _, _ in FORWARDED],
    )

    return LaunchDescription(args + [include])
