# ==============================================================================
# MOBILE MANIPULATOR BRINGUP
# ==============================================================================
# UR10e + Lineer Eksen VE Kawasaki RS005L + AGV'yi tek bir RViz penceresinde
# birlikte görüntüler ve kontrol eder.
#
# KULLANIM:
#   ros2 launch mobile_manipulator_description mobile_manipulator_bringup.launch.py
#   ros2 launch mobile_manipulator_description mobile_manipulator_bringup.launch.py use_fake_hardware:=false
#
# use_fake_hardware:=true  (varsayılan)
#   - Tüm eksenler mock_components/GenericSystem ile simüle edilir
#   - Gerçek robot bağlantısı gerektirmez
#   - Hızlı test ve geliştirme için
#
# use_fake_hardware:=false
#   - UR10e: ur_robot_driver (192.168.3.5)
#   - Lineer eksen: Festo EDCON linear_axis_adapter
#   - Kawasaki RS005L: kawasaki_ros2_control (192.168.3.7:11111)
#   - AGV: topic_based_ros2_control + agv_bridge_node (192.168.3.6:9090)
#
# URDF: my_robot_cell_control/urdf/whole_cell_hw.urdf.xacro
# CONTROLLERS: mobile_manipulator_description/config/whole_cell_hw_controllers.yaml
# ==============================================================================

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    Command,
    FindExecutable,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.parameter_descriptions import ParameterValue


def launch_setup(context, *args, **kwargs):
    # ==================== Argümanlar ====================
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    fake_sensor_commands = LaunchConfiguration("fake_sensor_commands")
    robot_ip = LaunchConfiguration("robot_ip")
    ur_type = LaunchConfiguration("ur_type")
    headless_mode = LaunchConfiguration("headless_mode")
    launch_rviz = LaunchConfiguration("launch_rviz")
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")
    activate_joint_controller = LaunchConfiguration("activate_joint_controller")

    # ==================== Paket dizinleri ====================
    mm_desc_pkg = get_package_share_directory("mobile_manipulator_description")
    my_cell_ctrl_pkg = get_package_share_directory("my_robot_cell_control")

    # ==================== Dosya yolları ====================
    urdf_file = PathJoinSubstitution([
        FindPackageShare("my_robot_cell_control"),
        "urdf",
        "whole_cell_hw.urdf.xacro",
    ])

    controllers_file = os.path.join(
        mm_desc_pkg, "config", "whole_cell_hw_controllers.yaml"
    )

    kinematics_params_file = PathJoinSubstitution([
        FindPackageShare("my_robot_cell_control"),
        "config",
        context.launch_configurations.get("ur_type", "ur10e"),
        "default_kinematics.yaml",
    ])

    rviz_config_file = PathJoinSubstitution([
        FindPackageShare("my_robot_cell_control"),
        "rviz",
        "whole_real.rviz",
    ])

    # ==================== Robot Description (URDF) ====================
    robot_description_content = ParameterValue(
        Command([
            FindExecutable(name="xacro"),
            " ",
            urdf_file,
            " use_fake_hardware:=", use_fake_hardware,
            " fake_sensor_commands:=", fake_sensor_commands,
            " robot_ip:=", robot_ip,
            " ur_type:=", ur_type,
            " headless_mode:=", headless_mode,
            " kinematics_params:=", kinematics_params_file,
        ]),
        value_type=str,
    )

    robot_description = {"robot_description": robot_description_content}

    # ==================== Robot State Publisher ====================
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    # ==================== Controller Manager ====================
    # fake hardware için standart ros2_control_node
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            controllers_file,
        ],
        output="screen",
        condition=IfCondition(use_fake_hardware),
    )

    # gerçek hardware için ur_ros2_control_node (UR driver gerektirir)
    ur_ros2_control_node = Node(
        package="ur_robot_driver",
        executable="ur_ros2_control_node",
        parameters=[
            robot_description,
            controllers_file,
        ],
        output="screen",
        condition=UnlessCondition(use_fake_hardware),
    )

    # ==================== UR Gerçek Donanım Ek Nodları ====================
    dashboard_client_node = Node(
        package="ur_robot_driver",
        executable="dashboard_client",
        name="dashboard_client",
        output="screen",
        emulate_tty=True,
        parameters=[{"robot_ip": robot_ip}],
        condition=UnlessCondition(use_fake_hardware),
    )

    robot_state_helper_node = Node(
        package="ur_robot_driver",
        executable="robot_state_helper",
        name="ur_robot_state_helper",
        output="screen",
        parameters=[
            {"headless_mode": headless_mode},
            {"robot_ip": robot_ip},
        ],
        condition=UnlessCondition(use_fake_hardware),
    )

    urscript_interface = Node(
        package="ur_robot_driver",
        executable="urscript_interface",
        parameters=[{"robot_ip": robot_ip}],
        output="screen",
        condition=UnlessCondition(use_fake_hardware),
    )

    controller_stopper_node = Node(
        package="ur_robot_driver",
        executable="controller_stopper_node",
        name="controller_stopper",
        output="screen",
        emulate_tty=True,
        condition=UnlessCondition(use_fake_hardware),
        parameters=[
            {"headless_mode": headless_mode},
        ],
    )

    # ==================== Lineer Eksen Gerçek Donanım ====================
    linear_axis_adapter_node = Node(
        package="festo_edcon_ros2",
        executable="linear_axis_adapter.py",
        name="linear_axis_adapter_node",
        output="screen",
        parameters=[{"use_sim_time": False}],
        condition=UnlessCondition(use_fake_hardware),
    )

    # ==================== AGV Gerçek Donanım Nodları ====================
    agv_bridge_node = Node(
        package="agv_nodes",
        executable="bridge_node",
        name="agv_bridge_node",
        output="screen",
        parameters=[{"use_sim_time": False}],
        condition=UnlessCondition(use_fake_hardware),
    )

    agv_controller_node = Node(
        package="agv_nodes",
        executable="agv_controller_node",
        name="agv_controller_node",
        output="screen",
        parameters=[{"use_sim_time": False}],
        condition=UnlessCondition(use_fake_hardware),
    )

    # ==================== Controller Spawner'lar ====================
    # joint_state_broadcaster — hemen başlat
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # UR10e I/O ve durum controller'ları
    io_and_status_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["io_and_status_controller", "-c", "/controller_manager"],
        output="screen",
    )

    speed_scaling_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["speed_scaling_state_broadcaster", "-c", "/controller_manager"],
        output="screen",
    )

    force_torque_sensor_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["force_torque_sensor_broadcaster", "-c", "/controller_manager"],
        output="screen",
    )

    # UR10e hareket controller'ı (activate_joint_controller'a göre)
    initial_joint_controller_spawner_started = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[initial_joint_controller, "-c", "/controller_manager"],
        output="screen",
        condition=IfCondition(activate_joint_controller),
    )

    initial_joint_controller_spawner_stopped = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[initial_joint_controller, "-c", "/controller_manager", "--stopped"],
        output="screen",
        condition=UnlessCondition(activate_joint_controller),
    )

    # Kawasaki + AGV controller (200ms gecikme ile — controller_manager'ın hazır olmasını bekle)
    kawasaki_controller_spawner = TimerAction(
        period=2.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["kawasaki_controller", "-c", "/controller_manager"],
                output="screen",
            ),
        ],
    )

    # AGV diff drive controller (sadece fake hw — gerçek hw'da topic_based ile yönetilir)
    ota_base_controller_spawner = TimerAction(
        period=2.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["ota_base_controller", "-c", "/controller_manager", "--stopped"],
                output="screen",
                condition=IfCondition(use_fake_hardware),
            ),
        ],
    )

    # ==================== RViz ====================
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        output="screen",
        condition=IfCondition(launch_rviz),
    )

    # ==================== Başlatma Listesi ====================
    nodes_to_start = [
        LogInfo(msg="[mobile_manipulator_bringup] Başlatılıyor..."),

        # URDF + TF
        robot_state_publisher_node,

        # Controller manager (fake veya gerçek)
        ros2_control_node,
        ur_ros2_control_node,

        # UR gerçek donanım nodları
        dashboard_client_node,
        robot_state_helper_node,
        urscript_interface,
        controller_stopper_node,

        # Lineer eksen gerçek donanım
        linear_axis_adapter_node,

        # AGV gerçek donanım
        agv_bridge_node,
        agv_controller_node,

        # Controller spawner'lar
        joint_state_broadcaster_spawner,
        io_and_status_controller_spawner,
        speed_scaling_state_broadcaster_spawner,
        force_torque_sensor_broadcaster_spawner,
        initial_joint_controller_spawner_started,
        initial_joint_controller_spawner_stopped,
        kawasaki_controller_spawner,
        ota_base_controller_spawner,

        # RViz
        rviz_node,
    ]

    return nodes_to_start


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="true",
            description="mock_components kullan (true) veya gerçek donanıma bağlan (false)",
        ),
        DeclareLaunchArgument(
            "fake_sensor_commands",
            default_value="false",
            description="Sahte sensör komutları etkinleştir (use_fake_hardware:=true ile)",
        ),
        DeclareLaunchArgument(
            "robot_ip",
            default_value="192.168.3.5",
            description="UR10e robot IP adresi",
        ),
        DeclareLaunchArgument(
            "ur_type",
            default_value="ur10e",
            description="UR robot tipi",
        ),
        DeclareLaunchArgument(
            "headless_mode",
            default_value="false",
            description="UR headless mod (External Control programı gerektirmez)",
        ),
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="RViz başlat",
        ),
        DeclareLaunchArgument(
            "initial_joint_controller",
            default_value="scaled_joint_trajectory_controller",
            choices=[
                "scaled_joint_trajectory_controller",
                "joint_trajectory_controller",
                "forward_position_controller",
                "forward_velocity_controller",
            ],
            description="UR10e için aktif edilecek controller",
        ),
        DeclareLaunchArgument(
            "activate_joint_controller",
            default_value="true",
            description="initial_joint_controller'ı otomatik aktif et",
        ),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
