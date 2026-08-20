"""UR10e anomali tespiti düğümünü başlatır."""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# Model, kalibrasyon ve FMU çözücüsü paketin share dizinine kurulur.
BASE = get_package_share_directory("anomaly_detection")


def generate_launch_description():
    args = [
        DeclareLaunchArgument("models_base", default_value=BASE,
                             description="model/kalibrasyon dosyalarının kök dizini"),
        DeclareLaunchArgument("tf_prefix", default_value="ur10e_"),
        DeclareLaunchArgument("joint_states_topic", default_value="/joint_states"),
        DeclareLaunchArgument("wrench_topic",
                             default_value="/force_torque_sensor_broadcaster/ft_data"),
        DeclareLaunchArgument("adaptive", default_value="true",
                             description="mutlak eşiğin yanında medyan+k·MAD kuralı "
                                         "(yavaş kayma ve iyi uyan koşular için şart)"),
        DeclareLaunchArgument("adaptive_k", default_value="8.0"),
    ]
    b = LaunchConfiguration("models_base")
    node = Node(
        package="anomaly_detection",
        executable="detector",
        name="ur10e_anomaly_detector",
        output="screen",
        parameters=[{
            "residual_model_dir": [b, "/residual_ae_v2"],
            "raw_model_dir": [b, "/raw_ae_v2"],
            "fusion_config": [b, "/fusion_v2/fusion_config.json"],
            "current_to_torque": [b, "/current_to_torque.json"],
            "residual_calibration": [b, "/residual_calibration_clean.json"],
            "solver_resources": [b, "/resources"],
            "tf_prefix": LaunchConfiguration("tf_prefix"),
            "joint_states_topic": LaunchConfiguration("joint_states_topic"),
            "wrench_topic": LaunchConfiguration("wrench_topic"),
            "adaptive": ParameterValue(LaunchConfiguration("adaptive"), value_type=bool),
            "adaptive_k": ParameterValue(LaunchConfiguration("adaptive_k"), value_type=float),
        }],
    )
    return LaunchDescription(args + [node])
