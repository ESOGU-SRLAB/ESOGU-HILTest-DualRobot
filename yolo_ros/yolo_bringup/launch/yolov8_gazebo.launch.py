# YOLOv8 Gazebo Launch File
# Gazebo simülasyonu için preprocessing node + YOLO node'unu birlikte başlatır.

import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Preprocessing node - Gazebo görüntüsünü YOLO için hazırlar
    preprocess_node = Node(
        package="yolo_ros",
        executable="gazebo_image_preprocess",
        name="gazebo_image_preprocess",
        output="screen",
        parameters=[
            {
                "input_topic": LaunchConfiguration("input_image_topic", default="/image"),
                "output_topic": "/sim/image",
                "enable_clahe": True,
                "clahe_clip_limit": 2.0,
                "clahe_grid_size": 8,
                "enable_blur": True,
                "blur_kernel_size": 3,
                "enable_noise": True,
                "noise_sigma": 5.0,
                "force_bgr_conversion": True,
                "image_reliability": 1,  # BEST_EFFORT
            }
        ],
    )

    # YOLO node - preprocessed görüntüyü alır
    yolo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("yolo_bringup"),
                "launch",
                "yolo.launch.py",
            )
        ),
        launch_arguments={
            "model": LaunchConfiguration(
                "model", default="/home/cem/colcon_ws/src/yolo_ros/best.pt"
            ),
            "tracker": LaunchConfiguration("tracker", default="bytetrack.yaml"),
            "device": LaunchConfiguration("device", default="cpu"),
            "enable": LaunchConfiguration("enable", default="True"),
            "threshold": LaunchConfiguration("threshold", default="0.3"),
            "input_image_topic": "/sim/image",  # preprocessed topic
            "image_reliability": LaunchConfiguration(
                "image_reliability", default="1"
            ),
            "namespace": LaunchConfiguration("namespace", default="yolo"),
        }.items(),
    )

    return LaunchDescription([preprocess_node, yolo_launch])