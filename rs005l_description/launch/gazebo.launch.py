import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get package path
    pkg_path = get_package_share_directory('rs005l_description')
    
    # Arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "gz_gui",
            default_value="true",
            description="Start Gazebo GUI"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "world_file",
            default_value="empty.sdf",
            description="Gazebo world file"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Launch RViz"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value="rs005l_controllers.yaml",
            description="Controller configuration file"
        )
    )

    # Paths
    pkg_description = FindPackageShare("rs005l_description")
    urdf_file = PathJoinSubstitution([pkg_description, "urdf", "rs005l.urdf"])
    rviz_config = PathJoinSubstitution([pkg_description, "urdf.rviz"])
    controllers_file = PathJoinSubstitution(
        [pkg_description, "config", LaunchConfiguration("controllers_file")]
    )

    # Robot description
    robot_description_content = Command(["cat ", urdf_file])
    robot_description = {"robot_description": robot_description_content}

    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": True}, robot_description],
    )

    controller_manager = Node(
    package="controller_manager",
    executable="ros2_control_node",
    parameters=[
        robot_description,
        controllers_file,
        {"use_sim_time": True}
    ],
    output="screen",
)

    # Gazebo Ignition
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py"
            ])
        ]),
        launch_arguments={
            "gz_args": ["-r -v 4 ", LaunchConfiguration("world_file")]
        }.items(),
        condition=IfCondition(LaunchConfiguration("gz_gui"))
    )

    # Spawn robot
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic", "robot_description",
            "-name", "RS005L",
            "-z", "0.0"
        ],
    )

    # Gazebo-ROS bridge for clock
    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )

    # Joint State Broadcaster spawner
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager"
        ],
        parameters=[{"use_sim_time": True}],
    )

    # Arm Controller spawner
    arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "kawasaki_controller",
            "--controller-manager", "/controller_manager"
        ],
        parameters=[{"use_sim_time": True}],
    )

    # RViz
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(LaunchConfiguration("launch_rviz")),
    )

    # Delay RViz after joint_state_broadcaster
    delay_rviz = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[rviz],
        ),
        condition=IfCondition(LaunchConfiguration("launch_rviz")),
    )

    return LaunchDescription(
        declared_arguments + [
            controller_manager,
            robot_state_publisher,
            gazebo,
            spawn_entity,
            gz_bridge,
            joint_state_broadcaster,
            arm_controller,
            delay_rviz,
        ]
    )