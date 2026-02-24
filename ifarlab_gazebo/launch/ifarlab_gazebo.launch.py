import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    EnvironmentVariable,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def launch_setup(context, *args, **kwargs):
    # ==================== Arguments ====================
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    launch_rviz = LaunchConfiguration("launch_rviz", default="true")
    gazebo_gui = LaunchConfiguration("gazebo_gui", default="true")
    world_file = LaunchConfiguration("world_file")

    # ==================== Gazebo Environment Variables ====================
    # UR10e Resources
    ur_pkg_share = get_package_share_directory("my_robot_cell_description")
    
    # Kawasaki & AGV Resources
    kawasaki_pkg_share = get_package_share_directory("mobile_manipulator_description")
    kawasaki_model_path = os.path.join(kawasaki_pkg_share, "model")
    kawasaki_share_root = os.path.dirname(kawasaki_pkg_share)

    set_ign_resource = SetEnvironmentVariable(
        name="IGN_GAZEBO_RESOURCE_PATH",
        value=[
            kawasaki_model_path, ":",
            kawasaki_share_root, ":",
            EnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", default_value=""),
        ],
    )

    set_gz_resource = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[
            kawasaki_model_path, ":",
            kawasaki_share_root, ":",
            EnvironmentVariable("GZ_SIM_RESOURCE_PATH", default_value=""),
        ],
    )

    # ==================== Robot Description ====================
    ifarlab_controllers = PathJoinSubstitution(
        [FindPackageShare("ifarlab_gazebo"), "config", "ifarlab_controllers.yaml"]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("ifarlab_gazebo"), "urdf", "combined_system.urdf.xacro"]
            ),
            " ",
            "sim_ignition:=true",
            " ",
            "simulation_controllers:=",
            ifarlab_controllers,
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    # ==================== Gazebo Launch ====================
    gz_launch_description_with_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments={"gz_args": [" -r -v 4 ", world_file]}.items(),
        condition=IfCondition(gazebo_gui),
    )

    gz_launch_description_without_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments={"gz_args": [" -s -r -v 4 ", world_file]}.items(),
        condition=UnlessCondition(gazebo_gui),
    )

    # ==================== Spawn Entity ====================
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-string",
            robot_description_content,
            "-name",
            "ifarlab_combined_system",
            "-allow_renaming",
            "true",
            # We spawn everything at 0,0,0 because the parts are offset inside the URDF
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.0",
            "-R", "0.0",
            "-P", "0.0",
            "-Y", "0.0",
        ],
    )

    # ==================== Core ROS Nodes ====================
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"use_sim_time": use_sim_time}, robot_description],
    )

    ur_rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("my_robot_cell_description"), "rviz", "whole_urdf.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", ur_rviz_config_file],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(launch_rviz),
    )

    # Custom TF from world -> odom for AGV starting pose
    # Note: URDF places ota_base_link wherever TF tells it, relative to odom.
    world_to_odom_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "-2.301949", "0.530401", "0.17",  # x, y, z
            "0", "0", "0.707107", "0.707107",  # qx, qy, qz, qw (yaw=90 degrees)
            "world", "odom",
        ],
        output="screen",
    )

    # UR10e depth optical frame TF
    ur_tf_installer = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "-0.038", "0.050", "0.055", "0.001", "0.707", "-0.001", "0.707",
            "sim_ur10e_wrist_3_link", "sim_ur10e_wrist_3_link/ur10e_depth_optical_frame",
        ],
    )

    # ==================== Bridges ====================
    ur_gz_sim_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "--ros-args",
            "-p",
            f"config_file:={os.path.join(get_package_share_directory('my_robot_cell_gz'), 'bridge', 'bridgos.yaml')}",
        ],
        output="screen",
    )

    agv_cmd_vel_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
        ],
        remappings=[
            ("/cmd_vel", "/ota_base_controller/cmd_vel_unstamped"),
        ],
        output="screen",
    )

    agv_tf_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
        ],
        output="screen",
    )

    # Adapter nodes
    ur_linear_axis_adapter_node = Node(
        package="festo_edcon_ros2",
        executable="linear_axis_adapter.py",
        name="linear_axis_adapter_node",
        output="screen"
    )

    ur_real_to_sim_bridge_node = Node(
        package="my_robot_cell_control",
        executable="real_to_sim_bridge",
        name="real_to_sim_bridge",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # ==================== Controller Spawners ====================
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    delay_rviz = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        ),
        condition=IfCondition(launch_rviz),
    )

    # UR10e Controller
    ur_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["sim_scaled_joint_trajectory_controller", "-c", "/controller_manager"],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # Kawasaki Controller
    kawasaki_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["kawasaki_controller", "-c", "/controller_manager"],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # AGV Base Controller
    ota_base_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ota_base_controller", "-c", "/controller_manager"],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    ur_io_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["sim_io_and_status_controller", "-c", "/controller_manager"],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    ur_speed_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["sim_speed_scaling_state_broadcaster", "-c", "/controller_manager"],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    delayed_controllers = TimerAction(
        period=5.0,
        actions=[
            joint_state_broadcaster_spawner,
            delay_rviz,
            ur_controller_spawner,
            kawasaki_controller_spawner,
            ota_base_controller_spawner,
            ur_io_spawner,
            ur_speed_spawner,
        ],
    )

    # ==================== Node List ====================
    nodes_to_start = [
        set_ign_resource,
        set_gz_resource,
        gz_launch_description_with_gui,
        gz_launch_description_without_gui,
        robot_state_publisher_node,
        world_to_odom_tf,
        ur_tf_installer,
        ur_gz_sim_bridge,
        agv_cmd_vel_bridge,
        agv_tf_bridge,
        ur_linear_axis_adapter_node,
        ur_real_to_sim_bridge_node,
        gz_spawn_entity,
        delayed_controllers,
    ]

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "world_file",
            default_value="/home/cem/colcon_ws/src/mobile_manipulator_description/worlds/ifarlab.sdf",
            description="Gazebo world file path.",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
