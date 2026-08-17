"""
HIL Control Launch — Terminal 2
=================================
hil_test_whole_unified.launch.py (Terminal 1) çalışırken bu dosya
kalan 3 bileşeni sıralı olarak başlatır:

  t=0s   → AGV Bridge + AGV Controller (ROS1 bağlantısı olmasa bile hemen çalışır)
  t=8s   → MoveGroup (real_robot_moveit_config) — Terminal 1'deki controller_manager
           hazır olduktan sonra başlatılır

Terminal 3:
  ros2 launch pymoveit2_real multirobot_inspection_scenario.launch.py

Argümanlar:
  rosbridge_host  : AGV ROS1 PC adresi   (default: 192.168.3.6)
  rosbridge_port  : Rosbridge WS portu   (default: 9090)
  moveit_delay    : move_group başlama gecikmesi saniye   (default: 8.0)
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    LogInfo,
)

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():

    # ── Argümanlar ──────────────────────────────────────────────────────────
    rosbridge_host_arg = DeclareLaunchArgument(
        "rosbridge_host",
        default_value="192.168.3.6",
        description="AGV ROS1 Noetic PC IP adresi",
    )
    rosbridge_port_arg = DeclareLaunchArgument(
        "rosbridge_port",
        default_value="9090",
        description="Rosbridge WebSocket portu",
    )
    moveit_delay_arg = DeclareLaunchArgument(
        "moveit_delay",
        default_value="8.0",
        description="move_group başlamadan önce beklenen süre (saniye)",
    )

    # ── t=0s : AGV Bridge Node ───────────────────────────────────────────────
    agv_bridge_node = Node(
        package="agv_nodes",
        executable="bridge_node",
        name="agv_bridge_node",
        output="screen",
        parameters=[{
            "rosbridge_host": LaunchConfiguration("rosbridge_host"),
            "rosbridge_port": LaunchConfiguration("rosbridge_port"),
        }],
    )

    # ── t=0s : AGV Controller Node ──────────────────────────────────────────
    agv_controller_node = Node(
        package="agv_nodes",
        executable="agv_controller_node",
        name="agv_controller_node",
        output="screen",
        parameters=[{
            "rosbridge_host": LaunchConfiguration("rosbridge_host"),
            "rosbridge_port": LaunchConfiguration("rosbridge_port"),
        }],
    )

    # ── t=moveit_delay : MoveGroup ──────────────────────────────────────────
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("real_robot_moveit_config"),
                "launch",
                "move_group.launch.py",
            ])
        ),
    )

    gazebo_bridge = Node(
        package="my_robot_cell_control",
        executable="real_to_sim_bridge",
        name="real_to_sim_bridge",
        output="screen",
        parameters=[{
            # The ceiling is the SIM CONTROLLER's step (Gazebo max_step_size = 10 ms),
            # not the data rate: publishing faster replaces every command before the
            # controller can advance it and the twin freezes outright. 25 Hz gives it
            # 4 updates per command; the 0.1 s horizon keeps commands overlapping.
            "update_rate": 25.0,
            "trajectory_time": 0.1,
            "pass_through_velocities": True,
        }],
    )


    return LaunchDescription([
        # Argümanlar
        rosbridge_host_arg,
        rosbridge_port_arg,
        moveit_delay_arg,

        # t=0s — AGV bileşenleri (bağlantı yoksa kendi içinde retry yapar)
        LogInfo(msg="[t=0s] AGV Bridge ve Controller başlatılıyor..."),
        agv_bridge_node,
        agv_controller_node,
        gazebo_bridge,

        # t=moveit_delay — MoveIt2 (controller_manager hazır olmalı)
        TimerAction(
            period=LaunchConfiguration("moveit_delay"),
            actions=[
                LogInfo(msg="[t=moveit_delay] MoveGroup başlatılıyor..."),
                move_group_launch,
            ],
        ),

    ])
