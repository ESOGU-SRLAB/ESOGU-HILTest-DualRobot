# (SADECE SİMÜLASYON İÇİN DÜZENLENMİŞ KOD - SIMROBOT_IFARLAB_GAZEBO İLE ENTEGRE)

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction,
    LogInfo,
    OpaqueFunction,
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
from launch.conditions import IfCondition


def launch_setup(context, *args, **kwargs):
    # --- Argüman Değerlerini Al ---
    ur_type = LaunchConfiguration("ur_type")
    sim_namespace = LaunchConfiguration("sim_namespace")
    sim_launch_rviz = LaunchConfiguration("sim_launch_rviz")
    sim_world_file = LaunchConfiguration("sim_world_file")
    
    # Simülasyon için controller dosyasının tam yolunu belirtin
    controller_params_file = PathJoinSubstitution([
        FindPackageShare("my_robot_cell_control"),
        "config",
        "simrobot_ur_controllers.yaml"
    ])
    
    # Robot description'ı simülasyon için hazırla
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
    ])
    
    # --- SİMÜLASYON NAMESPACE GRUBU (SIM_NAMESPACE İLE) ---
    sim_group = GroupAction(
        actions=[
            LogInfo(msg=["Simülasyon bileşenleri başlatılıyor..."]),
            PushRosNamespace(sim_namespace),
            
            # Robot State Publisher
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="both",
                parameters=[
                    {"robot_description": robot_description_content},
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
                    "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                ],
                output="screen",
            ),
            
            # Controller Spawner'lar - Gecikmeli olarak başlatılıyor
            TimerAction(
                period=5.0,  # 5 saniye bekle
                actions=[
                    Node(
                        package="controller_manager",
                        executable="spawner",
                        arguments=["sim_joint_state_broadcaster", "--controller-manager", "/sim/controller_manager", "-t", "joint_state_broadcaster/JointStateBroadcaster"],
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
                        arguments=["sim_scaled_joint_trajectory_controller", "--controller-manager", "/sim/controller_manager", "-t", "ur_controllers/ScaledJointTrajectoryController"],
                        output="screen",
                    ),
                ]
            ),
            
            # MoveIt config'i simülasyon için namespace içinde başlat
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

            # Simülasyon için RViz
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2_sim",
                output="log",
                arguments=["-d", PathJoinSubstitution([
                    FindPackageShare("my_robot_cell_control"),
                    "rviz",
                    "view_robot_sim.rviz"
                ])],
                condition=IfCondition(sim_launch_rviz),
                parameters=[{"use_sim_time": True}],
            ),
        ]
    )
    
    return [sim_group]


def generate_launch_description():
    # --- Sadece Simülasyon İçin Gerekli Argümanları Tanımla ---
    declared_arguments = []
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            default_value="ur10e",
            description="Type/series of used UR robot.",
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
            "sim_launch_rviz", 
            default_value="true", 
            description="Launch RViz for simulation?"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "sim_world_file",
            default_value="/home/cem/colcon_ws/src/Universal_Robots_ROS2_Tutorials/my_robot_cell/my_robot_cell_gz/worlds/digital_twin_world.sdf",
            description="Gazebo world file path.",
        )
    )

    return LaunchDescription(declared_arguments + [
        LogInfo(msg=["Simülasyon Robotu başlatma yapılandırması yükleniyor..."]),
        OpaqueFunction(function=launch_setup)
    ])