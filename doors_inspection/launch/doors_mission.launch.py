"""Doors mission: the dashboard-driven sensing + cleaning run.

This is the doors counterpart of my_robot_cell_control's
sensing_and_cleaning_mission.launch.py, and it starts TWO nodes:

    doors_mission_node          -- waits on /harmony/cmd_input, owns the
                                   doors_inspection launch, publishes the scenario
                                   defects when the tour finishes
    cleaning_mission_runner.py  -- UNCHANGED from the HARMONY scenario: it waits for
                                   defects + CONFIRM and drives the UR only

sensing_robot.py is DELIBERATELY ABSENT. It subscribes to the same
/harmony/cmd_input and would start its own hand-written scan on START, so the UR
would be commanded from two places at once. doors_mission_node replaces it.

The Kawasaki needs no special handling during cleaning: cleaning_mission_runner
never commands it, and the doors inspection ends with return_home_kawasaki:=true,
so it is already parked at home when cleaning starts.

Expected order (this is what the dashboard does):
    1. ros2 launch my_robot_cell_control hil_test_whole_unified.launch.py harmony:=true
    2. ros2 launch doors_inspection doors_mission.launch.py
    3. publish START   on /harmony/cmd_input   -> doors inspection runs
    4. publish CONFIRM on /harmony/cmd_input   -> cleaning runs

Standalone use (without the dashboard):
    ros2 launch doors_inspection doors_mission.launch.py only_sim:=false
    ros2 topic pub -1 /harmony/cmd_input std_msgs/msg/String "{data: '{\"cmd\":\"START\"}'}"
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    mission_node = Node(
        package="doors_inspection",
        executable="doors_mission_node",
        name="doors_mission_node",
        output="screen",
        emulate_tty=True,
        parameters=[{
            "fixed_frame": "world",
            "only_sim": LaunchConfiguration("only_sim"),
            "force_replan": LaunchConfiguration("force_replan"),
            "extra_inspection_args": LaunchConfiguration("extra_inspection_args"),
            "defect_publish_interval": 0.3,
        }],
    )

    # Byte-identical configuration to the HARMONY mission launch: the defects and the
    # joint configurations it drives to are the same ones, so there is nothing
    # doors-specific to change here.
    cleaning_runner_node = Node(
        package="pymoveit2_real",
        executable="cleaning_mission_runner.py",
        name="harmony_cleaning_runner",
        output="screen",
        emulate_tty=True,
        parameters=[{
            "fixed_frame": "world",
            "real_robot_velocity": 0.1,
            "real_robot_acceleration": 0.1,
            "cleaning_duration": 3.0,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "only_sim", default_value="true",
            description="Forwarded to doors_inspection.launch.py: true -> capture only "
                        "the two Gazebo SICKs, false -> also the real ones. This is "
                        "about which CAMERAS are read, which is not the same question "
                        "as the HIL launch's use_fake_hardware (real ROBOT or not), "
                        "even though fake hardware only makes sense with only_sim:=true."),
        DeclareLaunchArgument(
            "force_replan", default_value="false",
            description="Forwarded to doors_inspection.launch.py. Cached trajectories "
                        "are replayed WITHOUT re-checking collision, so this must be "
                        "true for one run after any change to the collision model "
                        "(most recently the link7 gripper-pin box)."),
        DeclareLaunchArgument(
            "extra_inspection_args", default_value="",
            description="Extra 'name:=value' arguments appended to the doors_inspection "
                        "launch, e.g. \"kawasaki_velocity:=0.05\". Escape hatch so the "
                        "orchestrator does not need a parameter per knob."),
        mission_node,
        cleaning_runner_node,
    ])
