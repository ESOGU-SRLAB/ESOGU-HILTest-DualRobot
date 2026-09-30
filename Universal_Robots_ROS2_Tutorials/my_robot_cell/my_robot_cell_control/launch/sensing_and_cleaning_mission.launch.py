#!/usr/bin/env python3
"""
HARMONY Sensing & Cleaning Mission Launch

Akış:
  1. Sensing Start (/harmony/cmd_input {"cmd":"START"})
     -> sensing_robot kapıyı tarar, HOME'a döner, senaryonun 4 yapay defect'ini
        yayınlar (/harmony/mock_perception/defect + defect_list + MarkerArray)
  2. Cleaning Start (/harmony/cmd_input {"cmd":"CONFIRM"})
     -> cleaning_mission_runner defect'lere y sırasıyla gider, en sonda HOME'a döner

NOT: Görev doğrudan gerçek robot üzerinde çalışır (pymoveit2_real).
Gazebo/sim tarafı (pymoveit2_sim) ve SimToReal köprüsü kullanılmıyor.

red_detection_node ARTIK KULLANILMIYOR: defect'ler algılanmıyor, HARMONY
validasyon senaryosu gereği sabit koordinatlardan yayınlanıyor. Node paketde
duruyor; aşağıdaki launch argümanı ile tekrar açılabilir
(enable_red_detection:=true).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    enable_red_detection = LaunchConfiguration("enable_red_detection")

    sensing_robot_node = Node(
        package="pymoveit2_real",
        executable="sensing_robot.py",
        name="harmony_sensing_robot",
        output="screen",
        emulate_tty=True,
        parameters=[{
            "fixed_frame": "world",
            # Gerçek robot hız/ivme skalası. İlk denemelerde düşük tut,
            # güvenli olduğuna kanaat getirince kademeli artır.
            "real_robot_velocity": 0.1,
            "real_robot_acceleration": 0.1,
            # Defect'ler arasında arayüzün nefes alması için kısa bekleme.
            "defect_publish_interval": 0.3,
        }],
    )

    cleaning_runner_node = Node(
        package="pymoveit2_real",
        executable="cleaning_mission_runner.py",
        name="harmony_cleaning_runner",
        output="screen",
        emulate_tty=True,
        parameters=[{
            "fixed_frame": "world",
            # sensing_robot ile aynı hız/ivme skalası.
            "real_robot_velocity": 0.1,
            "real_robot_acceleration": 0.1,
            # Her defect'te bekleme süresi (giderim gösterimi).
            "cleaning_duration": 3.0,
        }],
    )

    # Devre dışı: kırmızı tespiti artık senaryonun parçası değil.
    red_detection_node = Node(
        package="pymoveit2_real",
        executable="red_detection_node.py",
        name="harmony_red_detection",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(enable_red_detection),
        parameters=[{
            "world_frame": "world",
            # Gerçek hücre topic'leri: SICK bulutu pointcloud_transformer_ur
            # tarafından /points -> /sick_points'e taşınıyor.
            "image_topic": "/intensity",
            "pointcloud_topic": "/sick_points",
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "enable_red_detection",
            default_value="false",
            description="Kırmızı tespit node'unu başlat. HARMONY validasyon "
                        "senaryosunda defect'ler sabit koordinatlardan gelir, "
                        "bu yüzden varsayılan false.",
        ),
        sensing_robot_node,
        cleaning_runner_node,
        red_detection_node,
    ])
