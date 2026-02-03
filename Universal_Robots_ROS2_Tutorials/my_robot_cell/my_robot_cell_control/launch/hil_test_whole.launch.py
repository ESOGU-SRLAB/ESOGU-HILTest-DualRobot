# HIL TEST WHOLE - Kawasaki Sim + UR10e Sim + UR10e Real
# Bu launch dosyası 3 robotu paralel olarak başlatır:
# 1. Kawasaki robot (simülasyon - /kawasaki namespace)
# 2. UR10e robot (simülasyon - /sim namespace)
# 3. UR10e robot (gerçek dünya - namespace yok)

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    Command,
    FindExecutable,
    EnvironmentVariable,
)
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch_ros.parameter_descriptions import ParameterValue


def launch_setup(context, *args, **kwargs):
    # ==================== Ortak Argümanlar ====================
    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_rviz = LaunchConfiguration("launch_rviz")
    gazebo_gui = LaunchConfiguration("gazebo_gui")
    world_file = LaunchConfiguration("world_file")

    # ==================== UR10e Gerçek Robot Argümanları ====================
    ur_type = LaunchConfiguration("ur_type")
    robot_ip = LaunchConfiguration("robot_ip")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    fake_sensor_commands = LaunchConfiguration("fake_sensor_commands")
    real_description_package = LaunchConfiguration("real_description_package")
    real_description_file = LaunchConfiguration("real_description_file")
    kinematics_params_file = LaunchConfiguration("kinematics_params_file")
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")
    activate_joint_controller = LaunchConfiguration("activate_joint_controller")
    headless_mode = LaunchConfiguration("headless_mode")
    real_tf_prefix = LaunchConfiguration("real_tf_prefix")

    # ==================== UR10e Simülasyon Argümanları ====================
    ur_sim_namespace = LaunchConfiguration("ur_sim_namespace")
    ur_sim_prefix = LaunchConfiguration("ur_sim_prefix")
    ur_sim_controllers_file = LaunchConfiguration("ur_sim_controllers_file")
    ur_sim_description_package = LaunchConfiguration("ur_sim_description_package")
    ur_sim_description_file = LaunchConfiguration("ur_sim_description_file")
    ur_sim_initial_joint_controller = LaunchConfiguration("ur_sim_initial_joint_controller")

    # ==================== Kawasaki Simülasyon Argümanları ====================
    kawasaki_namespace = LaunchConfiguration("kawasaki_namespace")
    kawasaki_runtime_config_package = LaunchConfiguration("kawasaki_runtime_config_package")
    kawasaki_controllers_file = LaunchConfiguration("kawasaki_controllers_file")
    kawasaki_description_package = LaunchConfiguration("kawasaki_description_package")
    kawasaki_description_file = LaunchConfiguration("kawasaki_description_file")
    kawasaki_tf_prefix = LaunchConfiguration("kawasaki_tf_prefix")
    kawasaki_start_joint_controller = LaunchConfiguration("kawasaki_start_joint_controller")
    kawasaki_initial_joint_controller = LaunchConfiguration("kawasaki_initial_joint_controller")

    # ==================== Gazebo Ortam Değişkenleri ====================
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

    # ==================== UR10e Gerçek Robot RViz Config ====================
    rviz_config_file_real = PathJoinSubstitution(
        [FindPackageShare(real_description_package), "rviz", "whole_real.rviz"]
    )

    # ==================== UR10e Simülasyon Robot Description ====================
    ur_sim_initial_joint_controllers = PathJoinSubstitution(
        [FindPackageShare("my_robot_cell_gz"), "config", ur_sim_controllers_file]
    )

    ur_sim_robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([
            FindPackageShare(ur_sim_description_package),
            "urdf",
            ur_sim_description_file
        ]),
        " ur_type:=", ur_type,
        " tf_prefix:=", ur_sim_prefix,
        " sim_ignition:=true",
        " simulation_controllers:=", ur_sim_initial_joint_controllers,
    ])

    ur_sim_robot_description = {"robot_description": ur_sim_robot_description_content}

    # ==================== Kawasaki Robot Description ====================
    kawasaki_initial_joint_controllers = PathJoinSubstitution(
        [FindPackageShare(kawasaki_runtime_config_package), "config", kawasaki_controllers_file]
    )

    kawasaki_robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([
            FindPackageShare(kawasaki_description_package),
            "urdf",
            kawasaki_description_file
        ]),
        " name:=rs005l",
        " tf_prefix:=", kawasaki_tf_prefix,
        " sim_ignition:=true",
        " simulation_controllers:=", kawasaki_initial_joint_controllers,
        " use_sim_time:=true",
    ])

    kawasaki_robot_description = {
        "robot_description": ParameterValue(kawasaki_robot_description_content, value_type=str)
    }

    # ==================== Gazebo Başlatma ====================
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

    # ==================== 1. GERÇEK UR10e ROBOT GRUBU (NAMESPACE YOK) ====================
    real_robot_launch_group = GroupAction(
        actions=[
            LogInfo(msg="[1/3] Gerçek UR10e robot bileşenleri başlatılıyor..."),
            
            # UR Robot Driver
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare("ur_robot_driver"),
                        "launch",
                        "ur_control.launch.py",
                    ])
                ),
                launch_arguments={
                    "ur_type": ur_type,
                    "robot_ip": robot_ip,
                    "tf_prefix": real_tf_prefix,
                    "launch_rviz": "false",  # ur_control kendi RViz'ini açmasın
                    "use_fake_hardware": use_fake_hardware,
                    "fake_sensor_commands": fake_sensor_commands,
                    "description_package": real_description_package,
                    "description_file": real_description_file,
                    "kinematics_params_file": kinematics_params_file,
                    "initial_joint_controller": initial_joint_controller,
                    "activate_joint_controller": activate_joint_controller,
                    "headless_mode": headless_mode,
                }.items(),
            ),
            
            # MoveIt Config (gerçek robot için)
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('real_robot_moveit_config'),
                        'launch',
                        'move_group.launch.py'
                    )
                ),
                launch_arguments={
                    "use_sim_time": "false",
                }.items(),
            ),
            
            # Linear Axis Adapter
            Node(
                package="festo_edcon_ros2",
                executable="linear_axis_adapter.py",
                name="linear_axis_adapter_node",
                output="screen",
                parameters=[{"use_sim_time": False}],
            ),
            
            # RViz (gerçek robot için) - Her zaman açılır
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2_real",
                output="log",
                arguments=["-d", rviz_config_file_real],
                parameters=[{"use_sim_time": False}],
            ),
        ]
    )

    # ==================== 2. KAWASAKI SİMÜLASYON GRUBU (/kawasaki namespace) ====================
    kawasaki_gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-string",
            kawasaki_robot_description_content,
            "-name",
            "mobile_manipulator",
            "-allow_renaming",
            "true",
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.0",
        ],
    )

    kawasaki_nodes_in_namespace = GroupAction(
        actions=[
            LogInfo(msg="[2/3] Kawasaki simülasyon bileşenleri başlatılıyor..."),
            PushRosNamespace(kawasaki_namespace),
            
            # Robot State Publisher
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="both",
                parameters=[{"use_sim_time": True}, kawasaki_robot_description],
            ),
            
            # Joint State Broadcaster
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["kawasaki_joint_state_broadcaster", "--controller-manager", "/kawasaki/controller_manager"],
                parameters=[{"use_sim_time": True}],
            ),
            
            # Joint Trajectory Controller
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[kawasaki_initial_joint_controller, "-c", "/kawasaki/controller_manager"],
                parameters=[{"use_sim_time": True}],
                condition=IfCondition(kawasaki_start_joint_controller),
            ),
            
            # MoveIt
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('kawasaki_moveit_config'),
                        'launch',
                        'move_group.launch.py'
                    )
                ),
                launch_arguments={
                    "use_sim_time": "true",
                    "namespace": kawasaki_namespace,
                }.items(),
            ),
        ]
    )

    # Kawasaki controller'larını 2 saniye gecikmeli başlat
    delayed_kawasaki_controllers = TimerAction(
        period=2.0,
        actions=[kawasaki_nodes_in_namespace],
    )

    # ==================== 3. UR10e SİMÜLASYON GRUBU (/sim namespace) ====================
    ur_sim_gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-world", "ifarlab",
            "-topic", "/sim/robot_description",
            "-name", "my_robot_cell_sim",
            "-allow_renaming", "true",
            "-x", "0",
            "-y", "0",
            "-z", "0"
        ],
        output="screen",
    )

    ur_sim_rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(ur_sim_description_package), "rviz", "whole_sim.rviz"]
    )

    ur_sim_nodes_in_namespace = GroupAction(
        actions=[
            LogInfo(msg="[3/3] UR10e simülasyon bileşenleri başlatılıyor..."),
            PushRosNamespace(ur_sim_namespace),
            
            # Robot State Publisher
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="both",
                parameters=[
                    {"robot_description": ur_sim_robot_description_content},
                    {"use_sim_time": True},
                ],
            ),
            
            # Gazebo-ROS Bridge
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                arguments=[
                    "--ros-args",
                    "-p", f"config_file:={os.path.join(get_package_share_directory('my_robot_cell_gz'), 'bridge', 'bridgos.yaml')}"
                ],
                output="screen",
            ),
            
            # TF Static Transform
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                arguments=[
                    '0', '0', '0', '0', '0', '0',
                    'sim_ur10e_depth_optical_frame',
                    'my_robot_cell_sim/sim_ur10e_wrist_3_link/sim_ur10e_depth_optical_frame'
                ],
                parameters=[{"use_sim_time": True}],
            ),
            
            # Joint State Broadcaster (5 saniye gecikme)
            TimerAction(
                period=5.0,
                actions=[
                    Node(
                        package="controller_manager",
                        executable="spawner",
                        arguments=["sim_joint_state_broadcaster", "--controller-manager", "/sim/controller_manager"],
                        output="screen",
                    ),
                ]
            ),
            
            # Joint Trajectory Controller (7 saniye gecikme)
            TimerAction(
                period=7.0,
                actions=[
                    Node(
                        package="controller_manager",
                        executable="spawner",
                        arguments=[ur_sim_initial_joint_controller, "--controller-manager", "/sim/controller_manager"],
                        output="screen",
                    ),
                ]
            ),
            
            # MoveIt
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('sim_robot_moveit_config'),
                        'launch',
                        'move_group.launch.py'
                    )
                ),
                launch_arguments={
                    "use_sim_time": "true",
                }.items(),
            ),
            
            # Real to Sim Bridge - GLOBAL NAMESPACE'DE BAŞLATILIYOR (aşağıda)
            # Gerçek robottan veri okuyup simülasyona göndermesi için
            # namespace dışında olmalı
            
            # Linear Axis Adapter (sim)
            # Node(
            #     package="festo_edcon_ros2",
            #     executable="linear_axis_adapter.py",
            #     name="linear_axis_adapter_node_sim",
            #     output="screen",
            #     parameters=[{"use_sim_time": True}],
            # ),
            
            # RViz (simülasyon için) - Her zaman açılır
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2_sim",
                output="log",
                arguments=["-d", ur_sim_rviz_config_file],
                parameters=[{"use_sim_time": True}],
            ),
        ]
    )

    # UR10e simülasyonunu 10 saniye gecikme ile başlat
    delayed_ur_sim_spawn_and_nodes = TimerAction(
        period=10.0,
        actions=[
            ur_sim_gz_spawn_entity,
            ur_sim_nodes_in_namespace,
        ],
    )

    # ==================== REAL TO SIM BRIDGE (GLOBAL NAMESPACE) ====================
    # Bu node gerçek robottan (/joint_states) veri okuyup
    # simülasyon robotuna (/sim/...) gönderir, bu yüzden namespace dışında olmalı
    real_to_sim_bridge_delayed = TimerAction(
        period=15.0,  # Simülasyon controller'ları başladıktan sonra (7+3 sn buffer)
        actions=[
            Node(
                package="my_robot_cell_control",
                executable="real_to_sim_bridge",
                name="real_to_sim_bridge",
                output="screen",
                parameters=[
                    {"use_sim_time": False},  # Gerçek robot zamanını kullan
                    {"update_rate": 10.0},     # 10 Hz güncelleme
                    {"trajectory_time": 0.1},  # Smooth geçiş için
                ],
            )
        ]
    )

    # ==================== Başlatılacak Tüm Düğümler ====================
    nodes_to_start = [
        # Ortam değişkenleri
        set_ign_resource,
        set_gz_resource,
        
        # Gazebo başlat
        gz_launch_description_with_gui,
        gz_launch_description_without_gui,
        
        # 1. Gerçek robot (hemen başlat)
        real_robot_launch_group,
        
        # 2. Kawasaki simülasyonu (spawn + 2 sn sonra controller'lar)
        kawasaki_gz_spawn_entity,
        delayed_kawasaki_controllers,
        
        # 3. UR10e simülasyonu (10 sn sonra spawn + controller'lar)
        delayed_ur_sim_spawn_and_nodes,
        
        # 4. Real-to-Sim Bridge (15 sn sonra, tüm controller'lar hazır olduktan sonra)
        real_to_sim_bridge_delayed,
    ]

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []

    # ==================== Ortak Argümanlar ====================
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation time (true) or real time (false).",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Launch RViz?"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gazebo_gui",
            default_value="true",
            description="Start gazebo with GUI?"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "world_file",
            default_value="/home/cem/colcon_ws/src/Universal_Robots_ROS2_Tutorials/my_robot_cell/my_robot_cell_gz/worlds/ifarlab.sdf",
            description="Gazebo world file path.",
        )
    )

    # ==================== UR10e Gerçek Robot Argümanları ====================
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            default_value="ur10e",
            description="Type/series of used UR robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            default_value="192.168.3.5",
            description="IP address by which the robot can be reached.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Start robot with fake hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "fake_sensor_commands",
            default_value="false",
            description="Enable fake command interfaces for sensors.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "real_description_package",
            default_value="my_robot_cell_control",
            description="Description package for real robot",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "real_description_file",
            default_value="my_robot_cell_controlled.urdf.xacro",
            description="URDF/XACRO description file with the real robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "kinematics_params_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("my_robot_cell_control"),
                "config",
                "my_robot_calibration.yaml",
            ]),
            description="The calibration configuration of the actual robot used.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_joint_controller",
            default_value="scaled_joint_trajectory_controller",
            description="Initially loaded robot controller (for real robot).",
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
            "headless_mode",
            default_value="false",
            description="Enable headless mode for robot control",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "real_tf_prefix",
            default_value="ur10e_",
            description="Gerçek robot TF frame'leri için kullanılacak önek.",
        )
    )

    # ==================== UR10e Simülasyon Argümanları ====================
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_sim_namespace",
            default_value="sim",
            description="Namespace for UR10e simulation.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_sim_prefix",
            default_value="sim_",
            description="TF prefix for UR10e simulation.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_sim_controllers_file",
            default_value="simrobot_ur_controllers.yaml",
            description="YAML file with UR sim controllers configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_sim_description_package",
            default_value="my_robot_cell_description",
            description="Description package for UR simulation.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_sim_description_file",
            default_value="simrobot_my_robot_cell.urdf.xacro",
            description="URDF/XACRO description file for UR simulation.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_sim_initial_joint_controller",
            default_value="sim_scaled_joint_trajectory_controller",
            description="UR simulation robot controller to start.",
        )
    )

    # ==================== Kawasaki Argümanları ====================
    declared_arguments.append(
        DeclareLaunchArgument(
            "kawasaki_namespace",
            default_value="/kawasaki",
            description="Namespace for Kawasaki robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "kawasaki_runtime_config_package",
            default_value="mobile_manipulator_description",
            description="Package with Kawasaki controller's configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "kawasaki_controllers_file",
            default_value="ns_rs005l_controllers.yaml",
            description="YAML file with Kawasaki controllers configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "kawasaki_description_package",
            default_value="mobile_manipulator_description",
            description="Description package for Kawasaki robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "kawasaki_description_file",
            default_value="mobile_manipulator.urdf.xacro",
            description="URDF/XACRO description file for Kawasaki robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "kawasaki_tf_prefix",
            default_value='""',
            description="TF prefix for Kawasaki robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "kawasaki_start_joint_controller",
            default_value="true",
            description="Start Kawasaki joint controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "kawasaki_initial_joint_controller",
            default_value="kawasaki_controller",
            description="Kawasaki robot controller to start.",
        )
    )

    return LaunchDescription(declared_arguments + [
        LogInfo(msg=["HIL Test Whole - 3 Robot Başlatma Yapılandırması Yükleniyor..."]),
        LogInfo(msg=["1. UR10e Gerçek Robot (namespace yok)"]),
        LogInfo(msg=["2. Kawasaki Simülasyon (/kawasaki)"]),
        LogInfo(msg=["3. UR10e Simülasyon (/sim)"]),
        OpaqueFunction(function=launch_setup)
    ])
