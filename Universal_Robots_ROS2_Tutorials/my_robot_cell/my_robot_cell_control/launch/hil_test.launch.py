# (DİJİTAL İKİZ İÇİN CONTROLLER ÇAKIŞMASI DÜZELTİLMİŞ HAL - SIMROBOT_IFARLAB_GAZEBO İLE ENTEGRE)

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
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    Command,
    FindExecutable,
)
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import SetEnvironmentVariable


def launch_setup(context, *args, **kwargs):
    # --- Argüman Değerlerini Al ---
    ur_type = LaunchConfiguration("ur_type")
    robot_ip = LaunchConfiguration("robot_ip")
    launch_rviz = LaunchConfiguration("launch_rviz")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    fake_sensor_commands = LaunchConfiguration("fake_sensor_commands")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    kinematics_params_file = LaunchConfiguration("kinematics_params_file")
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")
    activate_joint_controller = LaunchConfiguration("activate_joint_controller")
    headless_mode = LaunchConfiguration("headless_mode")
    namespace = LaunchConfiguration("namespace")
    sim_namespace = LaunchConfiguration("sim_namespace")
    tf_prefix = LaunchConfiguration("tf_prefix")
    
    # Gripper için eklenen dinamik xacro argümanları
    gripper_ip = LaunchConfiguration("gripper_ip")
    gripper_port = LaunchConfiguration("gripper_port")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    
    # Gazebo argümanları - simrobot_ifarlab_gazebo için
    use_gazebo = LaunchConfiguration("use_gazebo")
    sim_launch_rviz = LaunchConfiguration("sim_launch_rviz")
    sim_gazebo_gui = LaunchConfiguration("sim_gazebo_gui")
    sim_world_file = LaunchConfiguration("sim_world_file")

    real_robot_actions = []
    simulation_actions = []

    # use_gazebo değerini context'ten al
    use_gazebo_value = context.launch_configurations.get("use_gazebo", "false")
    sim_namespace_value = context.launch_configurations.get("sim_namespace", "sim")

    # === GERÇEK ROBOT BAŞLATMA GRUBU (NAMESPACE OLMADAN) ===
    real_robot_launch_group = GroupAction(
        actions=[
            LogInfo(msg="Gerçek robot bileşenleri başlatılıyor..."),
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
                    "tf_prefix": tf_prefix,
                    "launch_rviz": "false",  # RViz'i ayrı başlatacağız
                    "use_fake_hardware": use_fake_hardware,
                    "fake_sensor_commands": fake_sensor_commands,
                    "description_package": description_package,
                    "description_file": description_file,
                    "kinematics_params_file": kinematics_params_file,
                    "initial_joint_controller": initial_joint_controller,
                    "activate_joint_controller": activate_joint_controller,
                    "headless_mode": headless_mode,
                    "use_gripper": LaunchConfiguration("use_gripper"),
                    # Gerçek robot URDF'i oluşturulurken gripper IP/Port bilgilerini aktarıyoruz
                    "gripper_ip": gripper_ip,
                    "gripper_port": gripper_port,
                    "use_mock_hardware": use_mock_hardware,
                }.items(),
            ),
            # MoveIt config'i gerçek robot için namespace olmadan başlat
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
            )
        ]
    )
    real_robot_actions.append(real_robot_launch_group)

    linear_axis_adapter_node = Node(
        package="festo_edcon_ros2",
        executable="linear_axis_adapter.py",
        name="linear_axis_adapter_node",
        output="screen"
    )
    real_robot_actions.append(linear_axis_adapter_node)

    # SICK Visionary T Mini Node - sistem ayağa kalkarken hemen başlat
    sick_visionary_node = Node(
        package="sick_visionary_ros",
        executable="sick_visionary_t_mini_node",
        name="sick_visionary_t_mini_node",
        output="screen",
        parameters=[{"use_sim_time": False}],
    )
    real_robot_actions.append(sick_visionary_node)
    
    # Point Cloud Transformer Node - 15 saniye gecikme ile başlat
    pointcloud_transformer_delayed = TimerAction(
        period=15.0,  # 15 saniye bekle
        actions=[
            Node(
                package="pointcloud_transformer",
                executable="pointcloud_transformer_node",
                name="pointcloud_transformer_node",
                output="screen",
                parameters=[{"use_sim_time": False}],
            )
        ]
    )
    real_robot_actions.append(pointcloud_transformer_delayed)
    
    # Gerçek robot için RViz config
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "rviz", "view_robot_real.rviz"]
    )
    rviz_config_file_sim = PathJoinSubstitution(
        [FindPackageShare(description_package), "rviz", "view_robot_sim.rviz"]
    )

    # Gerçek robot için RViz (namespace olmadan)
    rviz_node_real = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_real",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(launch_rviz),
        parameters=[{"use_sim_time": False}],
    )
    real_robot_actions.append(rviz_node_real)

    # === SİMÜLASYON BİLEŞENLERİ (SADECE 'use_gazebo' true ise oluşturulur) ===
    if use_gazebo_value.lower() == 'true':
    
        # Controller dosyasının tam yolunu belirtin
        controller_params_file = PathJoinSubstitution([
            FindPackageShare("my_robot_cell_gz"),
            "config",
            "simrobot_ur_controllers.yaml"
        ])
        
        # Robot description'ı simülasyon için hazırla (XACRO DİNAMİK ARGÜMANLARI BURADA)
        robot_description_content = Command([
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([
                FindPackageShare("my_robot_cell_description"),
                "urdf",
                "simrobot_my_robot_cell.urdf.xacro"
            ]),
            " ur_type:=", ur_type,
            " tf_prefix:=sim_",
            " sim_ignition:=true",
            " simulation_controllers:=", controller_params_file,
            " gripper_ip:=", gripper_ip,
            " gripper_port:=", gripper_port,
            " use_fake_hardware:=", use_fake_hardware,
        ])
        
        # --- SİMÜLASYON NAMESPACE GRUBU (SIM_NAMESPACE İLE) ---
        sim_group = GroupAction(
            actions=[
                LogInfo(msg=f"Simülasyon bileşenleri başlatılıyor - namespace: {sim_namespace_value}"),
                PushRosNamespace(sim_namespace),
                
                # Robot State Publisher
                Node(
                    package="robot_state_publisher",
                    executable="robot_state_publisher",
                    output="both",
                    parameters=[
                        {"robot_description": ParameterValue(robot_description_content, value_type=str)},
                        {"use_sim_time": True},
                    ],
                ),
                
                # Gazebo'yu başlat
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        PathJoinSubstitution([
                            FindPackageShare("ros_gz_sim"),
                            "launch",
                            "gz_sim.launch.py"
                        ])
                    ),
                    launch_arguments={
                        "gz_args": ["-r -v4 ", sim_world_file],
                        "on_exit_shutdown": "true"
                    }.items(),
                ),
                
                # Robot'u Gazebo'ya spawn et
                Node(
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
                ),
                
                # Gazebo-ROS köprüsü
                Node(
                    package="ros_gz_bridge",
                    executable="parameter_bridge",
                    arguments=[
                        "--ros-args",
                        "-p", f"config_file:={os.path.join(get_package_share_directory('my_robot_cell_gz'), 'bridge', 'bridgos.yaml')}"
                    ],
                    output="screen",
                ),

                # URDF Düzeltmesine göre güncellenmiş hali aşağıda
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
                
                # Controller Spawner'lar - GECİKMELİ OLARAK
                TimerAction(
                    period=5.0,  # 5 saniye bekle
                    actions=[
                        Node(
                            package="controller_manager",
                            executable="spawner",
                            arguments=["sim_joint_state_broadcaster", "--controller-manager", "/sim/controller_manager"],
                            output="screen",
                        ),
                    ]
                ),
                
                TimerAction(
                    period=7.0,  # 7 saniye bekle
                    actions=[
                        Node(
                            package="controller_manager",
                            executable="spawner",
                            arguments=["sim_scaled_joint_trajectory_controller", "--controller-manager", "/sim/controller_manager"],
                            output="screen",
                        ),
                    ]
                ),
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
                
                # Real to Sim Bridge Node - Gerçek robot ile simüle robot arasında köprü
                Node(
                    package="my_robot_cell_control",
                    executable="real_to_sim_bridge",
                    name="real_to_sim_bridge",
                    output="screen",
                    parameters=[{"use_sim_time": True}],
                ),
                
                # Gerçek robot için RViz (namespace olmadan)
                Node(
                    package="rviz2",
                    executable="rviz2",
                    name="rviz2_real",
                    output="log",
                    arguments=["-d", rviz_config_file_sim],
                    condition=IfCondition(launch_rviz),
                    parameters=[{"use_sim_time": True}],
                )
            ]
        )
    
    # Simülasyon eylemlerini gecikmeli olarak başlatmak için TimerAction kullan
    delayed_sim_launch = TimerAction(
        period=3.0,  # 3 saniye gecikme ile başlat
        actions=[sim_group]
    )
    simulation_actions.append(delayed_sim_launch)

    # === GRIPPER CONTROLLER SPAWNER (use_gripper true ise) ===
    gripper_actions = []
    use_gripper_value = context.launch_configurations.get("use_gripper", "false")

    if use_gripper_value.lower() == 'true':
        onrobot_launch = TimerAction(
            period=10.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        PathJoinSubstitution([
                            FindPackageShare("onrobot_robot_driver"),
                            "launch",
                            "onrobot_control.launch.py",
                        ])
                    ),
                    launch_arguments={
                        "address": gripper_ip,
                        "port": gripper_port,
                        "use_fake_hardware": use_fake_hardware,
                    }.items(),
                ),
            ]
        )
        gripper_actions.append(onrobot_launch)

    # Paralel başlatma için eylemler
    final_actions = []
    
    # Gerçek robot eylemlerini hemen başlat
    final_actions.extend(real_robot_actions)

    # Gripper controller spawner ekle
    if gripper_actions:
        final_actions.extend(gripper_actions)
    
    # Simülasyon eylemlerini de ekle (paralel)
    if simulation_actions:
        final_actions.extend(simulation_actions)

    return final_actions


def generate_launch_description():
    # --- Argümanları Tanımla ---
    declared_arguments = []
    
    # Temel robot argümanları
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
    
    # --- GRIPPER İÇİN EKLENEN YENİ ARGÜMANLAR ---
    declared_arguments.append(
        DeclareLaunchArgument(
            "gripper_ip",
            default_value="192.168.3.56",
            description="IP address of the OnRobot Gripper.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gripper_port",
            default_value="502",
            description="Port of the OnRobot Gripper.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="false",
            description="Set to true to use mock/fake hardware for the gripper.",
        )
    )
    # --------------------------------------------

    declared_arguments.append(
        DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz for real robot?")
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
            "description_package",
            default_value="my_robot_cell_control",
            description="description package for real robot",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="my_robot_cell_controlled.urdf.xacro",
            description="URDF/XACRO description file with the real robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "kinematics_params_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("my_robot_cell_control"),
                    "config",
                    "my_robot_calibration.yaml",
                ]
            ),
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
            "namespace",
            default_value="",
            description="Gerçek robot için kullanılacak namespace (boş bırakılabilir)."
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "sim_namespace",
            default_value="sim",
            description="Simülasyon için kullanılacak namespace."
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tf_prefix",
            default_value="ur10e_",
            description="Gerçek robot TF frame'leri için kullanılacak önek."
        )
    )

    # Gazebo özel argümanları (sim_ prefix ile)
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_gazebo",
            default_value="true",
            description="Use Gazebo simulation instead of real robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "sim_launch_rviz", 
            default_value="true", 
            description="Launch RViz for simulation?"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "sim_gazebo_gui", 
            default_value="true", 
            description="Start gazebo with GUI?"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "sim_world_file",
            default_value="/home/cem/colcon_ws/src/Universal_Robots_ROS2_Tutorials/my_robot_cell/my_robot_cell_gz/worlds/digital_twin_world.sdf",
            description="Gazebo world file path.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_gripper",
            default_value="false",
            description="Enable gripper on the robot.",
        )
    )

    return LaunchDescription(declared_arguments + [
        LogInfo(msg=["Dijital İkiz Robot başlatma yapılandırması yükleniyor..."]),
        
        # UR paketinin launch dosyasına özel argümanlarımızı sızdırmak için Environment Variable kullanıyoruz:
        SetEnvironmentVariable('ENV_USE_GRIPPER', LaunchConfiguration('use_gripper')),
        SetEnvironmentVariable('ENV_USE_FAKE_HARDWARE', LaunchConfiguration('use_fake_hardware')),
        SetEnvironmentVariable('ENV_GRIPPER_IP', LaunchConfiguration('gripper_ip')),
        SetEnvironmentVariable('ENV_GRIPPER_PORT', LaunchConfiguration('gripper_port')),
        
        OpaqueFunction(function=launch_setup)
    ])