# ==============================================================================
# HIL TEST WHOLE (UNIFIED URDF) - Tek URDF ile Real + Sim
# ==============================================================================
# Bu launch dosyası, sim ve real tarafı için birleşik URDF'ler kullanarak
# tüm sistemi ayağa kaldırır:
#
# REAL TARAF (namespace yok):
#   - whole_cell_real.urdf.xacro → tek URDF
#     * UR10e + Lineer Eksen (gerçek donanım, ur_robot_driver)
#     * AGV + Kawasaki RS005L (sadece collision, ros2_control yok)
#     * ifarlab_environment
#   - Tek robot_state_publisher → tam TF ağacı, MoveIt collision check
#   - UR10e controller_manager + linear_axis_adapter
#   - RViz
#
# SIM TARAF (/sim namespace):
#   - whole_cell_sim.urdf.xacro → tek URDF
#     * UR10e + Lineer Eksen (Gazebo sim, sim_ur10e_ prefix)
#     * AGV + Kawasaki RS005L (Gazebo sim, ros2_control var)
#     * ifarlab_environment
#   - Tek Gazebo spawn → tek gz_ros2_control plugin
#   - Tek controller_manager (/sim/controller_manager) altında tüm controller'lar
#   - Gazebo-ROS bridge'leri
#   - RViz
#
# GLOBAL:
#   - real_to_sim_bridge (gerçek robot → simülasyon senkronizasyonu)
#   - world → odom static TF
#   - Gazebo TF bridge
# ==============================================================================

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction,
    LogInfo,
    OpaqueFunction,
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
    kinematics_params_file = LaunchConfiguration("kinematics_params_file")
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")
    activate_joint_controller = LaunchConfiguration("activate_joint_controller")
    headless_mode = LaunchConfiguration("headless_mode")
    real_tf_prefix = LaunchConfiguration("real_tf_prefix")

    # ==================== Sim Argümanları ====================
    sim_namespace = LaunchConfiguration("sim_namespace")
    sim_tf_prefix = LaunchConfiguration("sim_tf_prefix")
    sim_controllers_file = LaunchConfiguration("sim_controllers_file")
    sim_ur_initial_joint_controller = LaunchConfiguration("sim_ur_initial_joint_controller")
    kawasaki_start_joint_controller = LaunchConfiguration("kawasaki_start_joint_controller")

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

    # ======================================================================
    # 1. GERÇEK ROBOT TARAFI (NAMESPACE YOK) - whole_cell_real.urdf.xacro
    # ======================================================================
    # ur_control.launch.py kullanarak gerçek UR10e'yi başlatıyoruz.
    # whole_cell_real.urdf.xacro, UR10e + Lineer Eksen (gerçek hw) yanı sıra
    # AGV + Kawasaki'yi de collision-only olarak içerir.
    # Bu sayede tek robot_state_publisher ile tam TF ağacı oluşur.

    rviz_config_file_real = PathJoinSubstitution(
        [FindPackageShare("my_robot_cell_control"), "rviz", "whole_real.rviz"]
    )

    real_robot_launch_group = GroupAction(
        actions=[
            LogInfo(msg="[1/2] Gerçek robot bileşenleri başlatılıyor (unified URDF)..."),

            # UR Robot Driver (whole_cell_real.urdf.xacro ile)
            # Bu launch, robot_state_publisher + controller_manager + UR driver'ı
            # tek seferde ayağa kaldırır.
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
                    "launch_rviz": "false",
                    "use_fake_hardware": use_fake_hardware,
                    "fake_sensor_commands": fake_sensor_commands,
                    "description_package": "my_robot_cell_control",
                    "description_file": "whole_cell_real.urdf.xacro",
                    "kinematics_params_file": kinematics_params_file,
                    "initial_joint_controller": initial_joint_controller,
                    "activate_joint_controller": activate_joint_controller,
                    "headless_mode": headless_mode,
                }.items(),
            ),

            # Linear Axis Adapter (Festo lineer eksen → ROS topic)
            Node(
                package="festo_edcon_ros2",
                executable="linear_axis_adapter.py",
                name="linear_axis_adapter_node",
                output="screen",
                parameters=[{"use_sim_time": False}],
            ),

            # RViz (gerçek robot görünümü)
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2_real",
                output="log",
                arguments=["-d", rviz_config_file_real],
                parameters=[{"use_sim_time": False}],
                condition=IfCondition(launch_rviz),
            ),
        ]
    )

    # ======================================================================
    # 2. SİMÜLASYON TARAFI (/sim namespace) - whole_cell_sim.urdf.xacro
    # ======================================================================
    # Tek bir URDF ile tüm sim bileşenleri (UR10e + Kawasaki + AGV) 
    # tek bir Gazebo modeli olarak spawn edilir.
    # Tek controller_manager (/sim/controller_manager) tüm controller'ları yönetir.

    sim_controllers_yaml = PathJoinSubstitution(
        [FindPackageShare("my_robot_cell_control"), "config", sim_controllers_file]
    )

    sim_robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([
            FindPackageShare("my_robot_cell_control"),
            "urdf",
            "whole_cell_sim.urdf.xacro"
        ]),
        " ur_type:=", ur_type,
        " tf_prefix:=", sim_tf_prefix,
        " sim_ignition:=true",
        " simulation_controllers:=", sim_controllers_yaml,
    ])

    # Gazebo'ya model spawn (tek model, tüm bileşenler dahil)
    sim_gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-world", "ifarlab",
            "-topic", "/sim/robot_description",
            "-name", "whole_cell_sim",
            "-allow_renaming", "true",
            "-x", "0",
            "-y", "0",
            "-z", "0",
        ],
        output="screen",
    )

    sim_rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("my_robot_cell_description"), "rviz", "whole_real.rviz"]
    )

    sim_nodes_in_namespace = GroupAction(
        actions=[
            LogInfo(msg="[2/2] Simülasyon bileşenleri başlatılıyor (unified URDF)..."),
            PushRosNamespace(sim_namespace),

            # Robot State Publisher (tek URDF, tam TF ağacı)
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="both",
                parameters=[
                    {"robot_description": ParameterValue(sim_robot_description_content, value_type=str)},
                    {"use_sim_time": True},
                ],
            ),

            # Gazebo-ROS Bridge (sensör verileri için)
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                arguments=[
                    "--ros-args",
                    "-p", "config_file:={}".format(
                        os.path.join(
                            get_package_share_directory("my_robot_cell_gz"),
                            "bridge",
                            "bridgos.yaml"
                        )
                    ),
                ],
                output="screen",
            ),

            # TF Static Transform (kamera frame remapping)
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=[
                    "0", "0", "0", "0", "0", "0",
                    "sim_ur10e_depth_optical_frame",
                    "whole_cell_sim/sim_ur10e_wrist_3_link/sim_ur10e_depth_optical_frame",
                ],
                parameters=[{"use_sim_time": True}],
            ),

            # === UR10e Sim Controller'ları ===
            # Joint State Broadcaster (5 sn gecikme - Gazebo hazır olsun)
            TimerAction(
                period=5.0,
                actions=[
                    Node(
                        package="controller_manager",
                        executable="spawner",
                        arguments=[
                            "sim_joint_state_broadcaster",
                            "--controller-manager", "/sim/controller_manager",
                        ],
                        output="screen",
                    ),
                ],
            ),

            # UR10e Joint Trajectory Controller (7 sn gecikme)
            TimerAction(
                period=7.0,
                actions=[
                    Node(
                        package="controller_manager",
                        executable="spawner",
                        arguments=[
                            sim_ur_initial_joint_controller,
                            "--controller-manager", "/sim/controller_manager",
                        ],
                        output="screen",
                    ),
                ],
            ),

            # === Kawasaki Controller'ları ===
            # Kawasaki Joint State Broadcaster (5 sn gecikme)
            TimerAction(
                period=5.0,
                actions=[
                    Node(
                        package="controller_manager",
                        executable="spawner",
                        arguments=[
                            "kawasaki_joint_state_broadcaster",
                            "--controller-manager", "/sim/controller_manager",
                        ],
                        parameters=[{"use_sim_time": True}],
                        output="screen",
                    ),
                ],
            ),

            # Kawasaki Joint Trajectory Controller (7 sn gecikme)
            TimerAction(
                period=7.0,
                actions=[
                    Node(
                        package="controller_manager",
                        executable="spawner",
                        arguments=[
                            "kawasaki_controller",
                            "--controller-manager", "/sim/controller_manager",
                        ],
                        parameters=[{"use_sim_time": True}],
                        output="screen",
                        condition=IfCondition(kawasaki_start_joint_controller),
                    ),
                ],
            ),

            # === AGV Controller ===
            # OTA Base Controller / Diff Drive (7 sn gecikme)
            TimerAction(
                period=7.0,
                actions=[
                    Node(
                        package="controller_manager",
                        executable="spawner",
                        arguments=[
                            "ota_base_controller",
                            "--controller-manager", "/sim/controller_manager",
                        ],
                        parameters=[{"use_sim_time": True}],
                        output="screen",
                    ),
                ],
            ),

            # RViz (simülasyon görünümü)
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2_sim",
                output="log",
                arguments=["-d", sim_rviz_config_file],
                parameters=[{"use_sim_time": True}],
                condition=IfCondition(launch_rviz),
            ),
        ]
    )

    # Sim spawn ve node'ları 10 sn gecikme ile başlat (Gazebo hazır olsun)
    delayed_sim_spawn_and_nodes = TimerAction(
        period=10.0,
        actions=[
            sim_gz_spawn_entity,
            sim_nodes_in_namespace,
        ],
    )

    # ======================================================================
    # 3. GLOBAL BILEŞENLER (namespace dışında)
    # ======================================================================

    # AGV cmd_vel bridge: ROS /sim/ota_base_controller/cmd_vel_unstamped <-> Gazebo /cmd_vel
    agv_cmd_vel_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
        ],
        remappings=[
            ("/cmd_vel", "/sim/ota_base_controller/cmd_vel_unstamped"),
        ],
        output="screen",
    )

    # Gazebo → ROS TF bridge (diff_drive plugin odom→ota_base_link TF yayınlıyor)
    gz_tf_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
        ],
        output="screen",
    )

    # world → odom static transform (AGV spawn pozisyonuyla eşleşmeli)
    world_to_odom_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "-2.301949", "0.530401", "0.17",   # x, y, z
            "0", "0", "0.707107", "0.707107",  # qx, qy, qz, qw (yaw=90°)
            "world", "odom",
        ],
        output="screen",
    )

    # === MoveIt Move Group (sim namespace) ===
    # TimerAction içindeki GroupAction ile namespace'i garanti ediyoruz.
    # (TimerAction, üst GroupAction'daki PushRosNamespace scope'unu kaybeder.)
    # delayed_sim_moveit = TimerAction(
    #     period=25.0,  # Sim spawn(10) + controller'lar(7) + buffer
    #     actions=[
    #         GroupAction(
    #             actions=[
    #                 PushRosNamespace(sim_namespace),
    #                 IncludeLaunchDescription(
    #                     PythonLaunchDescriptionSource(
    #                         PathJoinSubstitution([
    #                             FindPackageShare("sim_ifarlab_moveit_config"),
    #                             "launch",
    #                             "move_group.launch.py",
    #                         ])
    #                     ),
    #                 ),
    #             ]
    #         ),
    #     ],
    # )

    # Real-to-Sim Bridge (gerçek robottan sim'e joint state senkronizasyonu)
    # Tüm controller'lar hazır olduktan sonra başlat (15 sn)
    real_to_sim_bridge_delayed = TimerAction(
        period=30.0,  # Sim controller'lar + MoveIt hazır olduktan sonra
        actions=[
            Node(
                package="my_robot_cell_control",
                executable="real_to_sim_bridge",
                name="real_to_sim_bridge",
                output="screen",
                parameters=[
                    {"use_sim_time": False},
                    {"update_rate": 10.0},
                    {"trajectory_time": 0.1},
                ],
            ),
        ],
    )

    # ==================== Başlatılacak Tüm Düğümler ====================
    nodes_to_start = [
        # Ortam değişkenleri
        set_ign_resource,
        set_gz_resource,

        # Gazebo başlat
        gz_launch_description_with_gui,
        gz_launch_description_without_gui,

        # Gazebo → ROS TF bridge
        gz_tf_bridge,

        # world → odom static transform
        world_to_odom_tf,

        # AGV cmd_vel bridge
        agv_cmd_vel_bridge,

        # 1. Gerçek robot (hemen başlat - unified URDF)
        real_robot_launch_group,

        # 2. Simülasyon (10 sn sonra - unified URDF, tek spawn)
        delayed_sim_spawn_and_nodes,

        # 3. MoveIt (25 sn sonra - kendi GroupAction ile /sim namespace)
        # delayed_sim_moveit,

        # 4. Real-to-Sim Bridge (30 sn sonra)
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
            description="Simülasyon zamanı (true) veya gerçek zaman (false).",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="RViz başlatılsın mı?",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gazebo_gui",
            default_value="true",
            description="Gazebo GUI ile başlatılsın mı?",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "world_file",
            default_value="/home/cem/colcon_ws/src/mobile_manipulator_description/worlds/ifarlab.sdf",
            description="Gazebo dünya dosyası yolu.",
        )
    )

    # ==================== UR10e Gerçek Robot Argümanları ====================
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            default_value="ur10e",
            description="Kullanılan UR robot tipi/serisi.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            default_value="192.168.3.5",
            description="Robotun IP adresi.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Robotu sahte donanımla başlat.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "fake_sensor_commands",
            default_value="false",
            description="Sensörler için sahte komut arayüzlerini etkinleştir.",
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
            description="Gerçek robotun kalibrasyon yapılandırması.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_joint_controller",
            default_value="scaled_joint_trajectory_controller",
            description="Gerçek robot için başlangıç controller'ı.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "activate_joint_controller",
            default_value="true",
            description="Yüklenen joint controller aktif edilsin mi?",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "headless_mode",
            default_value="false",
            description="Headless mod etkinleştirilsin mi?",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "real_tf_prefix",
            default_value="ur10e_",
            description="Gerçek robot TF frame öneki.",
        )
    )

    # ==================== Simülasyon Argümanları ====================
    declared_arguments.append(
        DeclareLaunchArgument(
            "sim_namespace",
            default_value="sim",
            description="Simülasyon namespace'i.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "sim_tf_prefix",
            default_value="sim_",
            description="Simülasyon TF prefix (UR10e için sim_ur10e_ olur).",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "sim_controllers_file",
            default_value="whole_cell_sim_controllers.yaml",
            description="Birleşik simülasyon controller yapılandırma dosyası.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "sim_ur_initial_joint_controller",
            default_value="sim_scaled_joint_trajectory_controller",
            description="UR10e simülasyonu için başlangıç controller'ı.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "kawasaki_start_joint_controller",
            default_value="true",
            description="Kawasaki joint controller başlatılsın mı?",
        )
    )

    return LaunchDescription(declared_arguments + [
        LogInfo(msg=["============================================================"]),
        LogInfo(msg=["HIL Test Whole (Unified URDF) - 2 Taraflı Sistem Başlatılıyor"]),
        LogInfo(msg=["============================================================"]),
        LogInfo(msg=["REAL: UR10e + Lineer Eksen (gerçek hw) + AGV/Kawasaki (collision)"]),
        LogInfo(msg=["SIM:  UR10e + Lineer Eksen + AGV + Kawasaki (tek Gazebo model)"]),
        LogInfo(msg=["============================================================"]),
        OpaqueFunction(function=launch_setup),
    ])
