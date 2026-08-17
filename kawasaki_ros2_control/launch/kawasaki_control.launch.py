# kawasaki_control.launch.py
# Brings up the real Kawasaki RS005L with ros2_control.
#
# Usage:
#   ros2 launch kawasaki_ros2_control kawasaki_control.launch.py \
#       robot_ip:=192.168.3.7

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # ---- Declare arguments ------------------------------------------------
    declared_arguments = [
        DeclareLaunchArgument(
            "robot_ip",
            default_value="192.168.3.7",
            description="IP address of the Kawasaki RS005L robot controller.",
        ),
        DeclareLaunchArgument(
            "robot_port",
            default_value="11111",
            description="TCP port of the Kawasaki robot controller.",
        ),
        DeclareLaunchArgument(
            "tf_prefix",
            default_value='""',
            description="Prefix prepended to all joint names. "
                        "Useful in multi-robot setups.",
        ),
        DeclareLaunchArgument(
            "description_package",
            default_value="rs005l_description",
            description="Package that contains the robot URDF/xacro files.",
        ),
        DeclareLaunchArgument(
            "description_file",
            default_value="rs005l.urdf.xacro",
            description="Xacro file (relative to <description_package>/urdf/).",
        ),
        DeclareLaunchArgument(
            "controllers_file",
            default_value="kawasaki_controllers.yaml",
            description="Controller configuration YAML "
                        "(relative to kawasaki_ros2_control/config/).",
        ),
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Launch RViz2 for visualisation.",
        ),
    ]

    # ---- Retrieve configurations ------------------------------------------
    robot_ip          = LaunchConfiguration("robot_ip")
    robot_port        = LaunchConfiguration("robot_port")
    tf_prefix         = LaunchConfiguration("tf_prefix")
    description_pkg   = LaunchConfiguration("description_package")
    description_file  = LaunchConfiguration("description_file")
    controllers_file  = LaunchConfiguration("controllers_file")
    launch_rviz       = LaunchConfiguration("launch_rviz")

    # ---- Build robot description (URDF) via xacro -------------------------
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(description_pkg), "urdf", description_file]),
            " ",
            "name:=rs005l",
            " ",
            "tf_prefix:=", tf_prefix,
            " ",
            "use_fake_hardware:=false",
            " ",
            "sim_ignition:=false",
            " ",
            "robot_ip:=", robot_ip,
            " ",
            "robot_port:=", robot_port,
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(
            value=robot_description_content, value_type=str
        )
    }

    controllers_yaml = PathJoinSubstitution(
        [FindPackageShare("kawasaki_ros2_control"), "config", controllers_file]
    )

    rviz_config = PathJoinSubstitution(
        [FindPackageShare("rs005l_description"), "rviz", "urdf.rviz"]
    )

    # ---- Nodes ------------------------------------------------------------

    # robot_state_publisher – publishes /tf from URDF + joint_states
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # controller_manager – loads hardware interface + controllers
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controllers_yaml],
        output="both",
    )

    # Spawners – start after controller_manager is up
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager"],
    )

    # Start trajectory controller only after joint_state_broadcaster is active
    delay_jtc = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[joint_trajectory_controller_spawner],
        )
    )

    # RViz2 (optional)
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        condition=UnlessCondition(launch_rviz),  # inverted below
    )
    # NOTE: UnlessCondition means "don't launch if launch_rviz is *false*".
    # We want to launch when launch_rviz == "true", so we restructure:
    from launch.conditions import IfCondition
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        condition=IfCondition(launch_rviz),
    )

    return LaunchDescription(
        declared_arguments + [
            robot_state_publisher,
            ros2_control_node,
            joint_state_broadcaster_spawner,
            delay_jtc,
            rviz_node,
        ]
    )
