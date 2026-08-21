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
    ExecuteProcess,
    IncludeLaunchDescription,
    GroupAction,
    LogInfo,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
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
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue


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

    # ==================== Gripper Argümanları ====================
    gripper_ip = LaunchConfiguration("gripper_ip")
    gripper_port = LaunchConfiguration("gripper_port")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    use_gripper_value = context.launch_configurations.get("use_gripper", "false")
    digital_twin_value = context.launch_configurations.get("digital_twin", "false")
    harmony = LaunchConfiguration("harmony")

    # ==================== Gazebo Ortam Değişkenleri ====================
    kawasaki_pkg_share = get_package_share_directory("mobile_manipulator_description")
    kawasaki_model_path = os.path.join(kawasaki_pkg_share, "model")
    kawasaki_share_root = os.path.dirname(kawasaki_pkg_share)

    # Konveyör belt plugin yolları
    _conveyor_pkg = get_package_share_directory("conveyorbelt_gz")
    _conveyor_share_root = os.path.dirname(_conveyor_pkg)
    _conveyor_lib = os.path.join(
        os.path.dirname(os.path.dirname(_conveyor_pkg)), "lib"
    )

    # gz_ros2_control plugin yolu (kaynak derlemede GZ_SIM_SYSTEM_PLUGIN_PATH gerekmez)
    _gz_ros2_control_pkg = get_package_share_directory("gz_ros2_control")
    _gz_ros2_control_lib = os.path.join(
        os.path.dirname(os.path.dirname(_gz_ros2_control_pkg)), "lib"
    )

    set_ign_resource = SetEnvironmentVariable(
        name="IGN_GAZEBO_RESOURCE_PATH",
        value=[
            kawasaki_model_path, ":",
            kawasaki_share_root, ":",
            _conveyor_share_root, ":",
            EnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", default_value=""),
        ],
    )

    set_gz_resource = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[
            kawasaki_model_path, ":",
            kawasaki_share_root, ":",
            _conveyor_share_root, ":",
            EnvironmentVariable("GZ_SIM_RESOURCE_PATH", default_value=""),
        ],
    )

    set_gz_system_plugin = SetEnvironmentVariable(
        name="IGN_GAZEBO_SYSTEM_PLUGIN_PATH",
        value=[
            _gz_ros2_control_lib, ":",
            _conveyor_lib, ":",
            EnvironmentVariable("IGN_GAZEBO_SYSTEM_PLUGIN_PATH", default_value=""),
        ],
    )

    set_gz_sim_system_plugin = SetEnvironmentVariable(
        name="GZ_SIM_SYSTEM_PLUGIN_PATH",
        value=[
            _gz_ros2_control_lib, ":",
            _conveyor_lib, ":",
            EnvironmentVariable("GZ_SIM_SYSTEM_PLUGIN_PATH", default_value=""),
        ],
    )

    # ==================== Gazebo Başlatma ====================
    # NOT: Gazebo, sim GroupAction'ı (PushRosNamespace) içinde başlatılır.
    # Çalışan hil_test.launch.py ile aynı yapı. Bu, gz_ros2_control
    # plugin'inin /sim namespace'inde controller_manager oluşturmasını sağlar.

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

    # ======================================================================
    # KAWASAKI + AGV CONTROLLER YERLEŞİMİ (fake ve gerçek için AYNI topoloji)
    # ======================================================================
    # Her iki modda da (use_fake_hardware true/false) Kawasaki + AGV, UR CM'inden
    # AYRI bir controller_manager'da çalışır. UR CM control_group:=ur ile yalnızca
    # UR10e + lineer ekseni yükler; Kawasaki + AGV ise control_group:=kawasaki ile
    # üretilen ikinci CM'e taşınır.
    #   * false (gerçek): bloklayan Kawasaki SIR TCP soketini (KawasakiHardwareInterface)
    #     UR'ın 500 Hz gerçek zamanlı döngüsünden izole eder.
    #   * true  (fake): aynı CM mock_components ile çalışır; tek fark URDF'e
    #     use_fake_hardware:=true geçilmesidir.
    # Bu ikinci CM "/kawasaki" NAMESPACE'inde koşar (node: /kawasaki/controller_manager).
    #   ÖNEMLİ: CM'i `name:=...` ile yeniden adlandırmak YASAK — bu global
    #   `-r __node:=...` remap'ine dönüşür ve CM'in yüklediği TÜM controller
    #   node'larına sızar; controller yanlış isimle açılıp kendi parametrelerini
    #   (joints/command_interfaces) bulamaz ve configure olamaz. Namespace sızmaz.
    #   SONUÇ: action/topic isimleri /kawasaki ile öneklenir, örn.
    #   /kawasaki/kawasaki_controller/follow_joint_trajectory (her iki modda da AYNI).
    #   joint_states ise global /joint_states'e remap edilir (tek robot_state_publisher).
    # UR CM her iki modda da yalnızca UR + lineer ekseni sahiplenir.
    control_group_value = "ur"

    # Hem fake hem gerçek mod aynı Kawasaki + AGV controller dosyasını kullanır.
    kawasaki_cm_controllers_yaml = os.path.join(
        kawasaki_pkg_share, "config", "whole_cell_kawasaki_controllers.yaml"
    )

    # Ayrı Kawasaki + AGV controller_manager (fake ve gerçek için ortak).
    # Tek fark: URDF'e geçilen use_fake_hardware değeri (mock vs gerçek donanım).
    kawasaki_robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([
            FindPackageShare("my_robot_cell_control"),
            "urdf",
            "whole_cell_hw.urdf.xacro",
        ]),
        " control_group:=kawasaki",
        " ur_type:=", ur_type,
        " use_fake_hardware:=", use_fake_hardware,
    ])
    kawasaki_robot_description = {
        "robot_description": ParameterValue(
            kawasaki_robot_description_content, value_type=str
        )
    }

    # NOT: name= YERİNE namespace= kullanılıyor (yukarıdaki açıklamaya bakın).
    # Broadcaster /kawasaki/joint_states'e yayınlar; tek global
    # robot_state_publisher /joint_states dinlediği için geri remap ediyoruz.
    # (Remap, controller'ların CM'in global argümanlarını miras almasıyla
    # broadcaster node'una da uygulanır.)
    kawasaki_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace="kawasaki",
        parameters=[
            kawasaki_robot_description,
            ParameterFile(kawasaki_cm_controllers_yaml, allow_substs=True),
            {"use_sim_time": False},
        ],
        remappings=[("/kawasaki/joint_states", "/joint_states")],
        output="screen",
    )

    # Spawner'lar --param-file kullanmıyor: controller parametreleri CM'e verilen
    # ParameterFile'dan (/** wildcard) global argüman mirası ile okunur.
    kawasaki_cm_spawners = TimerAction(
        period=15.0,  # İkinci CM hazır olduktan sonra
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                namespace="kawasaki",
                arguments=[
                    "kawasaki_joint_state_broadcaster",
                    "--controller-manager", "/kawasaki/controller_manager",
                ],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                namespace="kawasaki",
                arguments=[
                    "kawasaki_controller",
                    "--controller-manager", "/kawasaki/controller_manager",
                ],
                output="screen",
            ),
        ],
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
                        FindPackageShare("ur_bringup"),
                        "launch",
                        "ur_control.launch.py",
                    ])
                ),
                launch_arguments={
                    "ur_type": ur_type,
                    "robot_ip": robot_ip,
                    "tf_prefix": real_tf_prefix,
                    # Her iki modda da UR CM yalnızca UR10e + lineer ekseni yükler
                    # (control_group=ur); Kawasaki + AGV ayrı /kawasaki CM'e taşınır.
                    "control_group": control_group_value,
                    "launch_rviz": "false",
                    "use_fake_hardware": use_fake_hardware,
                    "fake_sensor_commands": fake_sensor_commands,
                    "description_package": "my_robot_cell_control",
                    "description_file": "whole_cell_hw.urdf.xacro",
                    "kinematics_params_file": kinematics_params_file,
                    "initial_joint_controller": initial_joint_controller,
                    "activate_joint_controller": activate_joint_controller,
                    "headless_mode": headless_mode,
                    "use_gripper": LaunchConfiguration("use_gripper"),
                    "gripper_ip": gripper_ip,
                    "gripper_port": gripper_port,
                    "use_mock_hardware": use_mock_hardware,
                    "use_vacuum_gripper": LaunchConfiguration("use_vacuum_gripper"),
                }.items(),
            ),

            # Linear Axis Adapter (Festo lineer eksen → ROS topic)
            Node(
                package="festo_edcon_ros2",
                executable="linear_axis_adapter.py",
                name="linear_axis_adapter_node",
                output="screen",
                parameters=[{"use_sim_time": False}],
                condition=UnlessCondition(use_fake_hardware),
            ),

            # AGV Bridge Node — ROS1 Noetic PC (192.168.3.6:9090) ile köprü
            # world_to_agv prismatik joint'ini /joint_states'e publish eder;
            # robot_state_publisher bu olmadan AGV + Kawasaki TF ağacını yayımlamaz.
            # roslibpy bağlantı yoksa kendi içinde retry yapar, timer gerekmez.
            Node(
                package="agv_nodes",
                executable="bridge_node",
                name="agv_bridge_node",
                output="screen",
                parameters=[{"use_sim_time": False}],
                condition=UnlessCondition(use_fake_hardware),
            ),

            # AGV Controller Node — /agv/goal_position → ROS1 action goal
            Node(
                package="agv_nodes",
                executable="agv_controller_node",
                name="agv_controller_node",
                output="screen",
                parameters=[{"use_sim_time": False}],
                condition=UnlessCondition(use_fake_hardware),
            ),

            # Kawasaki RS005L + AGV: artık UR CM'inde DEĞİL, ayrı bir
            # controller_manager'da (kawasaki_controller_manager) çalışır.
            # CM düğümü + controller spawner'ları aşağıda eklenir.
            kawasaki_control_node,
            kawasaki_cm_spawners,

            # TimerAction (aşağıdaki yorum bloğu kaldırıldı, yukarıdaki ile değiştirildi)
            # TimerAction(
            #     period=8.0,
            #     actions=[
            #         Node(
            #             package="sir_robot_ros_interface",
            #             executable="ROS2KawasakiRobotJointStatePublisher",
            #             name="kawasaki_joint_state_publisher",
            #             output="screen",
            #             parameters=[{"use_sim_time": False}],
            #         ),
            #     ],
            # ),

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
        " use_gripper:=", LaunchConfiguration("use_gripper"),
        " use_vacuum_gripper:=", LaunchConfiguration("use_vacuum_gripper"),
    ])

    sim_rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("my_robot_cell_description"), "rviz", "whole_real.rviz"]
    )

    # Simülasyon GroupAction: Gazebo, spawn, robot_state_publisher ve
    # controller spawner'lar hepsi aynı PushRosNamespace içinde.
    # Bu yapı çalışan hil_test.launch.py ile aynıdır.
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

            # Gazebo Başlatma (GroupAction içinde, namespace ile birlikte)
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
                ),
                launch_arguments={
                    "gz_args": ["-r -v 8 ", world_file],
                    "on_exit_shutdown": "true",
                }.items(),
            ),

            # Gazebo'ya model spawn (tek model, tüm bileşenler dahil)
            Node(
                package="ros_gz_sim",
                executable="create",
                arguments=[
                    "-world", "cem",
                    "-topic", "/sim/robot_description",
                    "-name", "whole_cell_sim",
                    "-allow_renaming", "true",
                    "-x", "0",
                    "-y", "0",
                    "-z", "0",
                ],
                output="screen",
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

            # TF Static Transform (Kawasaki kamera frame remapping)
            # Kawasaki SICK bulutu gz-scoped frame_id ile yayınlanıyor
            # (whole_cell_sim/sim_kawasaki_link6/sim_kawasaki_camera_rgb_optic_frame);
            # bu bir TF frame'i olmadığı için RViz bulutu yerleştiremiyordu.
            # UR kamerasındaki ile aynı alias: gerçek TF frame'ine bağla.
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=[
                    "0", "0", "0", "0", "0", "0",
                    "sim_kawasaki_camera_rgb_optic_frame",
                    "whole_cell_sim/sim_kawasaki_link6/sim_kawasaki_camera_rgb_optic_frame",
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

            # Sim Gripper Controller (9 sn gecikme - sadece use_gripper:=true ise)
            TimerAction(
                period=9.0,
                actions=[
                    Node(
                        package="controller_manager",
                        executable="spawner",
                        arguments=[
                            "sim_gripper_controller",
                            "--controller-manager", "/sim/controller_manager",
                        ],
                        output="screen",
                        condition=IfCondition(LaunchConfiguration("use_gripper")),
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

    # Sim GroupAction'ı gecikmeli başlat (gerçek robot hazır olsun)
    delayed_sim_spawn_and_nodes = TimerAction(
        period=3.0,  # Çalışan hil_test.launch.py ile aynı (3 sn)
        actions=[
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

    # === VAKUMLA TUTMA KÖPRÜSÜ (ROS → Gazebo) ===
    # whole_cell_sim.urdf.xacro'daki DetachableJoint eklentisi bu iki gz
    # topic'ini dinliyor. Tek yönlü ([ değil ]): ROS'tan Gazebo'ya.
    # Gerçek robotta bu köprü hiç çalışmaz; gemini_robotics_ros yine de aynı
    # ROS topic'lerine yayınlar ve kimse dinlemediği için zararsızdır.
    vacuum_attach_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="vacuum_attach_bridge",
        arguments=[
            "/vacuum/attach@std_msgs/msg/Empty]gz.msgs.Empty",
            "/vacuum/detach@std_msgs/msg/Empty]gz.msgs.Empty",
        ],
        output="screen",
    )

    # DetachableJoint BAŞLANGIÇTA BAĞLI gelir: child_model dünyada belirir
    # belirmez eklemi kurar (ölçüldü, 11 Ağu 2026). RedCube 22. saniyede spawn
    # edildiğine göre, hemen ardından bir kez detach yayınlanmazsa kutu daha
    # ilk hareketten itibaren emme kabına kaynaklı olur ve kolun peşinden
    # sürüklenir. gz topic doğrudan kullanılıyor: köprünün o ana kadar ayağa
    # kalkmış olmasına bağlı kalmamak için.
    initial_detach = TimerAction(
        period=26.0,  # conveyor_spawn (22 s) + spawn'ın oturması
        actions=[
            ExecuteProcess(
                cmd=["gz", "topic", "-t", "/vacuum/detach",
                     "-m", "gz.msgs.Empty", "-p", ""],
                output="screen",
            ),
        ],
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

    # === MoveIt Move Group (sim & real) ===
    # Gripper/vacuum flag'larını context'ten oku (hem sim hem real MoveIt seçiminde kullanılır)
    use_vac_str = LaunchConfiguration("use_vacuum_gripper").perform(context)
    use_grip_str = LaunchConfiguration("use_gripper").perform(context)

    # === MoveIt Move Group (sim namespace) ===
    # Seçilen gripper/vacuum ayarlarına göre doğru sim MoveIt config paketini dinamik olarak yükler.
    if use_vac_str.lower() == 'true':
        sim_moveit_pkg = "sim_ifarlab_vacuum_moveit_config"
    elif use_grip_str.lower() == 'true':
        sim_moveit_pkg = "sim_ifarlab_gripper_moveit_config"
    else:
        sim_moveit_pkg = "sim_ifarlab_moveit_config"

    # TimerAction içindeki GroupAction ile namespace'i garanti ediyoruz.
    # (TimerAction, üst GroupAction'daki PushRosNamespace scope'unu kaybeder.)
    delayed_sim_moveit = TimerAction(
        period=25.0,  # Sim spawn(10) + controller'lar(7) + buffer
        actions=[
            GroupAction(
                actions=[
                    PushRosNamespace(sim_namespace),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            PathJoinSubstitution([
                                FindPackageShare(sim_moveit_pkg),
                                "launch",
                                "move_group.launch.py",
                            ])
                        ),
                    ),
                ]
            ),
        ],
    )

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
                    # 10 Hz / 0.1 s made the mirrored robots move in visible steps: the
                    # horizon equalled the update period, so each command finished and
                    # the arm STOPPED until the next one arrived.
                    # 125 Hz clears the slower real source (Kawasaki broadcaster 100 Hz;
                    # UR 500 Hz) and the 0.016 s horizon is 2x the period, so commands
                    # overlap. The bridge caches joints across messages and publishes on
                    # a timer, so this rate is what each arm actually gets -- before that
                    # the mixed /joint_states stream left the Kawasaki updating ~8 Hz.
                    {"update_rate": ParameterValue(
                        LaunchConfiguration("bridge_update_rate"), value_type=float)},
                    {"trajectory_time": ParameterValue(
                        LaunchConfiguration("bridge_trajectory_time"), value_type=float)},
                    # Forward the measured joint velocities so the controller blends
                    # through each waypoint instead of arriving at rest on every one.
                    {"pass_through_velocities": ParameterValue(
                        LaunchConfiguration("bridge_pass_velocities"), value_type=bool)},
                ],
            ),
        ],
    )

    # === MoveIt Move Group (Real Robot) ===
    # Seçilen gripper/vacuum/harmony ayarlarına göre doğru MoveIt config paketini dinamik olarak yükler.
    # NOT: use_vac_str ve use_grip_str yukarıda sim MoveIt seçiminde de kullanılıyor.

    if harmony.perform(context).lower() == 'true':
        real_moveit_pkg = "harmony_moveit_config"
    elif use_vac_str.lower() == 'true':
        real_moveit_pkg = "real_ifarlab_vacuum_moveit_config"
    elif use_grip_str.lower() == 'true':
        real_moveit_pkg = "real_ifarlab_gripper_moveit_config"
    else:
        real_moveit_pkg = "real_ifarlab_moveit_config"

    delayed_real_moveit = TimerAction(
        period=15.0,  # Controller'lar ve ortam hazır olduktan sonra
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare(real_moveit_pkg),
                        "launch",
                        "move_group.launch.py",
                    ])
                ),
            ),
        ],
    )

    # === KONVEYÖR BELT (Gazebo plugin'li, hareketli) ===
    # URDF'deki statik conveyor_belt linki devre dışı; aynı pozisyonda plugin'li SDF spawn edilir.
    #
    # POZİSYON REFERANSI: my_robot_cell_macro.xacro'daki conveyor_belt linki
    # (gerçek dünyada ölçülmüş konum). IFRA paketinin kendi STL'i farklı bir
    # modeldir ve kendi koordinatlarında duruyordu; iki mesh dikey ışınla
    # ölçülüp bant yüzeyleri üst üste getirildi:
    #
    #   bant yüzeyi        URDF (gerçek)      IFRA (eski)      fark
    #   x merkez             648.75 mm         741.0 mm      -92.0 mm
    #   y merkez              58.00 mm         110.0 mm      -52.0 mm
    #   üst yüzey z          851.76 mm         843.33 mm      +8.4 mm
    #   bant uzunluğu        956.0 mm          956.0 mm         0    (aynı konveyör)
    #
    # Eski spawn (0.7422, 0.0966, 0.0) IFRA STL'inin kendi origin offset'ini
    # telafi ediyordu, gerçek konumu değil. Yeni değerler = eski + yukarıdaki fark.
    # SDF'in belt_moving z'si (0.8434) mesh'ten türetildiği için z kaymasıyla
    # birlikte taşınır; bant çarpışma yüzeyi 851.8 mm'ye oturur.
    conveyor_spawn = TimerAction(
        period=22.0,  # Gazebo(0) + sim spawn(10) + robot spawn buffer
        actions=[
            Node(
                package="ros_gz_sim",
                executable="create",
                arguments=[
                    # Dünya adı SDF'ten gelir: mobile_manipulator_description/worlds/
                    # ifarlab.sdf içinde <world name="cem">. Dosya adı "cem" olsa da
                    # dünya adı "cem"; burada "cem" yazılırsa /world/cem/create
                    # servisi hiç var olmaz ve spawn "timed out" ile sessizce düşer.
                    # Robot spawn'ı zaten "-world cem" kullanıyor (yukarıda).
                    "-world", "cem",
                    "-file", os.path.join(_conveyor_pkg, "sdf", "conveyor.sdf"),
                    "-name", "conveyor",
                    "-allow_renaming", "false",
                    "-x", "0.6502",
                    "-y", "0.0446",
                    "-z", "0.0084",
                ],
                output="screen",
            ),
            Node(
                package="ros_gz_sim",
                executable="create",
                arguments=[
                    # Dünya adı SDF'ten gelir: mobile_manipulator_description/worlds/
                    # ifarlab.sdf içinde <world name="cem">. Dosya adı "cem" olsa da
                    # dünya adı "cem"; burada "cem" yazılırsa /world/cem/create
                    # servisi hiç var olmaz ve spawn "timed out" ile sessizce düşer.
                    # Robot spawn'ı zaten "-world cem" kullanıyor (yukarıda).
                    "-world", "cem",
                    "-file", os.path.join(_conveyor_pkg, "sdf", "RedCube.sdf"),
                    "-name", "RedCube",
                    "-allow_renaming", "true",
                    # Konveyör kaydırıldı; kutu da yeni bant merkezine bırakılır
                    # (bant görsel merkezi x=0.6490, y=0.0580; z serbest düşer).
                    "-x", "0.649",
                    "-y", "0.058",
                    "-z", "1",
                ],
                output="screen",
            ),
        ]
    )

    # === GERÇEK ROBOT GRIPPER CONTROLLER SPAWNER ===
    # NOT: gripper donanımı zaten ur_control.launch.py tarafından ana controller manager'a
    # yükleniyor. Burada sadece gripper_controller'ı spawn ediyoruz.
    gripper_controller_spawner = None
    if use_gripper_value.lower() == 'true':
        gripper_param_file = PathJoinSubstitution([
            FindPackageShare("my_robot_cell_control"),
            "config",
            "gripper_controllers.yaml"
        ])
        gripper_controller_spawner = TimerAction(
            period=15.0,  # UR driver ayağa kalktıktan sonra spawn et
            actions=[
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=[
                        "gripper_controller",
                        "--controller-manager", "/controller_manager",
                        "--controller-type", "joint_trajectory_controller/JointTrajectoryController",
                        "--param-file", gripper_param_file,
                    ],
                    output="screen",
                ),
            ]
        )

    # === ROSBRIDGE WEBSOCKET SERVER ===
    # Foxglove, web arayüzleri veya harici istemciler için WebSocket bridge (port 9090)
    # Gazebo Harmonic built-in WebSocket plugin kaldırıldığından rosbridge kullanıyoruz.
    # rosbridge_server = IncludeLaunchDescription(
    #     AnyLaunchDescriptionSource(
    #         PathJoinSubstitution([
    #             FindPackageShare("rosbridge_server"),
    #             "launch",
    #             "rosbridge_websocket_launch.xml",
    #         ])
    #     ),
    #     launch_arguments={
    #         "port": "9090",
    #         "address": "0.0.0.0",
    #     }.items(),
    # )

    # ==================== SICK Visionary-T Mini Kameraları ====================
    # 1. UR10e kamerası (varsayılan ayarlar: 192.168.3.50, data port 2114)
    sick_visionary_ur = Node(
        package="sick_visionary_ros",
        executable="sick_visionary_t_mini_node",
        name="sick_visionary_t_mini_ur",
        output="screen",
        parameters=[
            {"remote_device_ip": "192.168.3.50"},
            {"data_port": 2114},
            {"control_port": 5000},
            # Gerçek SICK'e özel kalibre optik frame (ur_macro.xacro'daki
            # ur_sick_optical_joint, rpy 0 0 60°). depth_optical_frame'e damgalarsak
            # bulut ~60° dönük gelir; bu frame düzeltmeyi TF ile uygular.
            {"frame_id": "ur10e_sick_optical_frame"},
            {"use_sim_time": False},
        ],
        condition=UnlessCondition(use_fake_hardware),
    )

    # 2. İkinci kamera (192.168.3.21, data port 2122)
    # GEÇİCİ DEVRE DIŞI: İkinci SICK kamera, UR robotuyla aynı NIC/subnet
    # (192.168.3.0/24, enp0s31f6) üzerinde olduğu için UR reverse interface'in
    # 500 Hz real-time trafiğini boğuyor ve bağlantı sürekli düşüyordu
    # ("Connection to reverse interface dropped" flapping). Ağ izolasyonu
    # yapılana kadar bu kamera launch'tan çıkarıldı.
    sick_visionary_kawasaki = Node(
        package="sick_visionary_ros",
        executable="sick_visionary_t_mini_node",
        name="sick_visionary_t_mini_kawasaki",
        namespace="sick_kawasaki",
        output="screen",
        parameters=[
            {"remote_device_ip": "192.168.3.21"},
            {"data_port": 2113},
            {"control_port": 2122},
            # GERÇEK SICK'e özel, kalibre edilebilir optik frame. URDF'teki
            # camera_rgb_optic_frame Gazebo sim kamerası için oryante edilmiştir
            # (<gazebo reference=camera_rgb_optic_frame>), gerçek SICK için 90°
            # dönüktür. Bu yüzden gerçek bulutu, camera_rgb_optic_frame'in altına
            # yerleştirilen ve kaw_opt_{roll,pitch,yaw} argümanlarıyla ayarlanan
            # ayrı bir frame'e damgalıyoruz (aşağıdaki kawasaki_sick_optical_tf).
            # Bu, UR'daki ayrı depth_optical_frame kalibrasyon zincirinin karşılığı.
            {"frame_id": "kawasaki_sick_optical_frame"},
            {"use_sim_time": False},
        ],
        condition=UnlessCondition(use_fake_hardware),
    )


    # ==================== Point Cloud Transformer'lar ====================
    # Her SICK kamerası için bir transformer: kameranın "points" bulutunu capture
    # pipeline'ın beklediği topic'e taşır (kaynak frame != target_frame ise
    # target_frame'e dönüştürür; eşitse olduğu gibi yeniden yayınlar). SICK
    # node'ları TCP'ye bağlanıp yayına başlasın diye 15 sn gecikmeyle başlatılır.
    # Sadece gerçek donanımda (use_fake_hardware:=false).
    #
    #   UR10e:    /points               -> /sick_points        (ur10e_depth_optical_frame)
    #   Kawasaki: /sick_kawasaki/points -> /kawasaki/pointcloud (kawasaki_sick_optical_frame)
    #
    # Çıktı topic'leri multirobot_executor_node'un beklediği isimlerdir
    # (real_ur_topic=/sick_points, real_kawasaki_topic=/kawasaki/pointcloud).
    pointcloud_transformer_delayed = TimerAction(
        period=15.0,
        actions=[
            Node(
                package="pointcloud_transformer",
                executable="pointcloud_transformer_node",
                name="pointcloud_transformer_ur",
                output="screen",
                parameters=[{
                    "input_topic": "/points",
                    "output_topic": "/sick_points",
                    # Match the sick node frame_id so the transformer stays a no-op
                    # relay; the ~60° correction is applied by the URDF TF
                    # (ur_sick_optical_joint), not by re-expressing coordinates.
                    "target_frame": "ur10e_sick_optical_frame",
                    "use_sim_time": False,
                }],
                condition=UnlessCondition(use_fake_hardware),
            ),
            Node(
                package="pointcloud_transformer",
                executable="pointcloud_transformer_node",
                name="pointcloud_transformer_kawasaki",
                output="screen",
                parameters=[{
                    "input_topic": "/sick_kawasaki/points",
                    "output_topic": "/kawasaki/pointcloud",
                    "target_frame": "kawasaki_sick_optical_frame",
                    "use_sim_time": False,
                }],
                condition=UnlessCondition(use_fake_hardware),
            ),
        ]
    )

    # ==================== Başlatılacak Tüm Düğümler ====================
    nodes_to_start = [
        # Ortam değişkenleri
        set_ign_resource,
        set_gz_resource,
        set_gz_system_plugin,
        set_gz_sim_system_plugin,

        # Gazebo artık sim GroupAction içinde başlatılıyor (satır yukarıda)

        # Gazebo → ROS TF bridge
        # gz_tf_bridge,

        # world → odom static transform
        world_to_odom_tf,

        # AGV cmd_vel bridge
        agv_cmd_vel_bridge,

        # 1. Gerçek robot (hemen başlat - unified URDF)
        real_robot_launch_group,

        # 2. Simülasyon (10 sn sonra - unified URDF, tek spawn)
        delayed_sim_spawn_and_nodes,

        # Vakumla tutma köprüsü (ROS → Gazebo DetachableJoint)
        vacuum_attach_bridge,

        # 3. Konveyör belt (22 sn sonra - Gazebo + robot spawn tamamlanınca)
        conveyor_spawn,

        # 3b. RedCube spawn olur olmaz eklem kurulduğu için bir kez ayır (26 sn)
        initial_detach,

        # 4. Sim MoveIt (25 sn sonra - kendi GroupAction ile /sim namespace)
        # Sadece digital_twin=true ise eklenecek.

        # 5. Real MoveIt (25 sn sonra - namespace yok)
        delayed_real_moveit,

        # 6. Real-to-Sim Bridge (30 sn sonra) — digital_twin=true ise devre dışı
        # real_to_sim_bridge_delayed,

        # 7. Rosbridge WebSocket Server (port 9090)
        # rosbridge_server,

        # 8. SICK Visionary-T Mini Kameraları (gerçek donanım)
        sick_visionary_ur,
        # sick_visionary_kawasaki,  # GEÇİCİ DEVRE DIŞI (ağ çekişmesi — yukarıdaki nota bakın)

        # 9. Point Cloud Transformer (gerçek donanım, 15 sn gecikme)
        pointcloud_transformer_delayed,
    ]

    if gripper_controller_spawner:
        nodes_to_start.append(gripper_controller_spawner)

    # === OnRobot VGC10 sürücüsü (yalnız GERÇEK vakum donanımında) ===
    #
    # İKİ KOŞUL BİRDEN: use_vacuum_gripper=true VE use_fake_hardware=false.
    #
    # use_fake_hardware=true iken başlatılmamalı, çünkü sürücü açılır açılmaz
    # 192.168.3.56:502'ye Modbus bağlantısı kurmaya çalışır - sahte donanımla
    # çalışırken ortada gripper yoktur ve bu yalnızca gürültü üretir.
    #
    # NEDEN BURADA: gemini_pick_place.launch.py bu sürücüyü BAŞLATMIYOR ve
    # elle unutulduğunda arıza sinsi oluyordu - vacuum_require_confirmation
    # açıkken /OnRobotVGInput hiç gelmez, dolayısıyla HER kavrama başarısız
    # sayılır ve görev "vakum kurulamadı" diye düşer. Hücreyi ayağa kaldıran
    # launch, gripper'ı da ayağa kaldırsın.
    #
    # DİKKAT (13 Ağu 2026, gerçek hücrede ölçüldü): sürücünün "Connected to
    # gripper" logu komutların çalıştığının KANITI DEĞİL. Gerçek doğrulama
    # /OnRobotVGInput'un yayınlaması (yüzde = gvca / 10).
    use_fake_str = LaunchConfiguration("use_fake_hardware").perform(context)
    if use_vac_str.lower() == "true" and use_fake_str.lower() != "true":
        nodes_to_start.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare("vgc10_control"),
                        "launch", "bringup.launch.py",
                    ])
                ])
            )
        )
    elif use_vac_str.lower() == "true":
        nodes_to_start.append(LogInfo(
            msg="VGC10 sürücüsü BAŞLATILMADI (use_fake_hardware=true). "
                "Gerçek gripper için use_fake_hardware:=false ile çalıştırın."
        ))

    # === UR10e anomali tespiti (yalnız GERÇEK donanımda) ===
    #
    # use_fake_hardware=true iken ÇALIŞMAZ. Sebebi VGC10'unkiyle aynı değil, daha
    # sinsi: sahte donanımda /joint_states'in effort alanı gerçek motor akımı
    # değildir. Dedektörün bütün kalıntı hesabı `effort = actual_current`
    # varsayımına dayanır (UR sürücüsü hardware_interface.cpp içinde bu alana
    # actual_current yazar). Mock donanımda bu alan ya sıfırdır ya da komut
    # torkudur; her iki durumda da skorlar anlamsız olur ama sistem hatasız
    # çalışmaya devam eder -- yani sessizce yanlış sonuç üretir.
    #
    # Eşik ve kural seçimleri paketin içine gömülüdür (fusion_config.json:
    # fused_threshold = 18,0; uyarlanabilir kural kapalı). İkisi de 21 Ağu 2026'da
    # bu hücrede ölçüldü; ayrıntı anomaly_detection/docs raporunun 10. bölümünde.
    #
    # Gecikme: KTS yayıncısı ve joint_state_broadcaster ayağa kalkmadan başlarsa
    # düğüm veri beklerken "veri gelmiyor" uyarısı basar. 30 s, real_to_sim_bridge
    # ile aynı gerekçeyle seçildi.
    use_anomaly_str = LaunchConfiguration("use_anomaly_detection").perform(context)
    if use_anomaly_str.lower() == "true" and use_fake_str.lower() != "true":
        nodes_to_start.append(TimerAction(
            period=30.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        PathJoinSubstitution([
                            FindPackageShare("anomaly_detection"),
                            "launch", "detector.launch.py",
                        ])
                    ])
                )
            ],
        ))
    elif use_anomaly_str.lower() == "true":
        nodes_to_start.append(LogInfo(
            msg="Anomali tespiti BAŞLATILMADI (use_fake_hardware=true). Sahte "
                "donanımda effort alanı gerçek motor akımı değildir, kalıntı "
                "hesabı anlamsız olur."
        ))

    # digital_twin=false (varsayılan) ise real_to_sim_bridge başlat
    if digital_twin_value.lower() != 'true':
        nodes_to_start.append(real_to_sim_bridge_delayed)
    else:
        # digital_twin=true ise sim_moveit başlat
        nodes_to_start.append(delayed_sim_moveit)

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

    # ==================== Gripper Argümanları ====================
    declared_arguments.append(
        DeclareLaunchArgument(
            "gripper_ip",
            default_value="192.168.3.56",
            description="OnRobot Gripper IP adresi.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gripper_port",
            default_value="502",
            description="OnRobot Gripper port numarası.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="false",
            description="Gripper için mock/fake donanım kullan.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_gripper",
            default_value="false",
            description="Gripper'ı etkinleştir.",
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

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_vacuum_gripper",
            default_value="false",
            description="Enable vacuum gripper on the robot.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_anomaly_detection",
            default_value="true",
            description="UR10e anomali tespiti başlatılsın mı? "
                        "use_fake_hardware:=true iken bu değere bakılmaksızın "
                        "başlatılmaz (sahte donanımda effort alanı gerçek motor "
                        "akımı değildir).",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "harmony",
            default_value="false",
            description="Harmony modu. true ise harmony_moveit_config paketiyle MoveIt başlatılır.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "digital_twin",
            default_value="false",
            description="Digital twin modu. true ise real_to_sim_bridge devre dışı kalır (sim bağımsız çalışır).",
        )
    )

    # real_to_sim_bridge ayarları — komut satırından denenebilsin diye argüman yapıldı.
    # Eski (titreyen ama çalışan) davranış:
    #   bridge_update_rate:=10.0 bridge_trajectory_time:=0.1 bridge_pass_velocities:=false
    declared_arguments.append(
        DeclareLaunchArgument(
            "bridge_update_rate",
            default_value="25.0",
            description="real_to_sim_bridge'in sim'e komut yayınlama hızı (Hz). ÜST SINIR "
                        "sim controller'ının adım hızıdır (Gazebo max_step_size = 10 ms), "
                        "veri hızı değil: daha hızlı yayınlarsan her komut controller "
                        "onu ilerletemeden yenisiyle değişir ve sim robot hiç hareket "
                        "etmez. Akıcılık komut hızından değil controller'ın ara "
                        "değerlemesinden gelir.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "bridge_trajectory_time",
            default_value="0.1",
            description="Her komutun ufku (s). Güncelleme periyodunun ~2 katı olmalı; "
                        "eşit olursa sim robot komutlar arasında durur.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "bridge_pass_velocities",
            default_value="true",
            description="Ölçülen eklem hızlarını komuta ekle. false ise her waypoint "
                        "duruşta komut edilir (eski davranış).",
        )
    )

    return LaunchDescription(declared_arguments + [
        LogInfo(msg=["============================================================"]),
        LogInfo(msg=["HIL Test Whole (Unified URDF) - 2 Taraflı Sistem Başlatılıyor"]),
        LogInfo(msg=["============================================================"]),
        LogInfo(msg=["REAL: UR10e + Lineer Eksen (gerçek hw) + AGV/Kawasaki (collision)"]),
        LogInfo(msg=["SIM:  UR10e + Lineer Eksen + AGV + Kawasaki (tek Gazebo model)"]),
        LogInfo(msg=["============================================================"]),
        SetEnvironmentVariable('ENV_USE_GRIPPER', LaunchConfiguration('use_gripper')),
        SetEnvironmentVariable('ENV_USE_VACUUM_GRIPPER', LaunchConfiguration('use_vacuum_gripper')),
        SetEnvironmentVariable('ENV_USE_FAKE_HARDWARE', LaunchConfiguration('use_fake_hardware')),
        SetEnvironmentVariable('ENV_GRIPPER_IP', LaunchConfiguration('gripper_ip')),
        SetEnvironmentVariable('ENV_GRIPPER_PORT', LaunchConfiguration('gripper_port')),
        OpaqueFunction(function=launch_setup),
    ])