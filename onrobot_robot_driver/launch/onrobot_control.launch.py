# Copyright (c) 2025 Touchlab Limited. All Rights Reserved
# Unauthorized copying or modifications of this file, via any medium is strictly prohibited.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def launch_setup(context, *args, **kwargs):

    # Initialize Arguments
    port = LaunchConfiguration("port")
    address = LaunchConfiguration("address")
    # General arguments
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    controllers_file = LaunchConfiguration("controllers_file")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    prefix = LaunchConfiguration("prefix")
    force = LaunchConfiguration("force")
    position_filtering = LaunchConfiguration("position_filtering")
    velocity_filtering = LaunchConfiguration("velocity_filtering")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    launch_rviz = LaunchConfiguration("launch_rviz")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "urdf", description_file]),
            " ",
            "prefix:=",
            prefix,
            " ",
            "port:=",
            port,
            " ",
            "address:=",
            address,
            " ",
            "force:=",
            force,
            " ",
            "position_filtering:=",
            position_filtering,
            " ",
            "velocity_filtering:=",
            velocity_filtering,
            " ",
            "use_fake_hardware:=",
            use_fake_hardware,
            " ",
        ]
    )
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    initial_joint_controllers = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "config", controllers_file]
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "rviz",
         "onrobot_2fg7_config.rviz"]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace="onrobot",
        parameters=[robot_description, initial_joint_controllers],
        output="both",
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="onrobot",
        output="both",
        parameters=[robot_description],
    )

    rviz_node = Node(
        package="rviz2",
        condition=IfCondition(launch_rviz),
        executable="rviz2",
        name="rviz2",
        namespace="onrobot",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace="onrobot",
        arguments=["joint_state_broadcaster",
                   "--controller-manager", "controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace="onrobot",
        arguments=["gripper_controller", "-c", "controller_manager"],
    )

    # Delay rviz start after `joint_state_broadcaster`
    delayed_rviz_spawner = RegisterEventHandler(
      event_handler=OnProcessExit(
          target_action=joint_state_broadcaster_spawner,
          on_exit=[rviz_node],
      )
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delayed_robot_controller_spawner = RegisterEventHandler(
      event_handler=OnProcessExit(
          target_action=joint_state_broadcaster_spawner,
          on_exit=[robot_controller_spawner],
      )
    )

    nodes_to_start = [
        control_node,
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        delayed_robot_controller_spawner,
    ]

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []
    # Onrobot specific arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "port", description="CAN port.",
            default_value="502",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "address", description="IP address of gripper",
            default_value="'192.168.3.56'",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "force", description="Force measurement provided by the gripper",
            default_value="",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "position_filtering", description="Position filtering for the gripper",
            default_value="",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "velocity_filtering", description="Velocity filtering for the gripper",
            default_value="",
        )
    )
    # General arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "runtime_config_package",
            default_value="onrobot_robot_driver",
            description='Package with the controller\'s configuration in "config" folder. \
        Usually the argument is not set, it enables use of a custom setup.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value="onrobot_2fg7_controllers.yaml",
            description="YAML file with the controllers configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="onrobot_2fg7_description",
            description="Description package with robot URDF/XACRO files. Usually the argument \
        is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="onrobot_2fg7.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed than also joint names in the controllers' configuration \
        have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Start robot with mock hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_joint_controller",
            default_value="joint_trajectory_controller",
            description="Initially loaded robot controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "activate_joint_controller",
            default_value="true",
            description="Activate loaded joint controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz", default_value="true", description="Launch RViz?")
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
