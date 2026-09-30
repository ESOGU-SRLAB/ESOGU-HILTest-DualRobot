"""UR10e anomali tespiti düğümünü başlatır. v4: KTS yok, use_case zorunlu."""
from pathlib import Path

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
        # ZORUNLU niyetinde - yük/ofset düzeltmesi buna göre seçilir. Yanlış
        # senaryo verilirse kalıntı yüzlerce Nm sapabilir.
        DeclareLaunchArgument("use_case", default_value="UR10E_INSPECTION",
                             description="HRC | MULTIROBOT_INSPECTION | PICKPLACE | "
                                         "UR10E_INSPECTION - çalışan göreve göre VERİLMELİ"),
        DeclareLaunchArgument("quantile", default_value="p99.9",
                             description="eşik persentili: p97 | p99 | p99.9 | p99.99 "
                                         "- fusion_config.json'un dördünü de taşıdığı "
                                         "PROVISIONAL (offline) değerlerden seçilir"),
        DeclareLaunchArgument("tf_prefix", default_value="ur10e_"),
        DeclareLaunchArgument("joint_states_topic", default_value="/joint_states"),
        DeclareLaunchArgument("adaptive", default_value="false",
                             description="v3'te gerçek robotta kalıntının poza bağlı "
                                         "olduğu ölçüldüğü için kapatılmıştı; v4'ün "
                                         "senaryo-başına yük düzeltmesi bunu azaltmış "
                                         "olabilir ama HENÜZ doğrulanmadı"),
        DeclareLaunchArgument("adaptive_k", default_value="8.0"),
        DeclareLaunchArgument("freeze_timeout", default_value="3.0",
                             description="s; alarm bundan uzun sürerse rejim "
                                         "değişimi sayılır, taban çizgisi çözülür"),
        DeclareLaunchArgument("motion_qd_min", default_value="-1.0",
                             description="rad/s; -1 = fusion_config.json'daki (0,02)"),
        DeclareLaunchArgument("log_dir", default_value=str(Path.home() / "anomali_kayit"),
                             description="olay/skor kayıtları; boş dize kaydı kapatır"),
        DeclareLaunchArgument("log_scores", default_value="true",
                             description="her kararı CSV'ye yaz (~170 MB/gün)"),
    ]
    b = LaunchConfiguration("models_base")
    node = Node(
        package="anomaly_detection",
        executable="detector",
        name="ur10e_anomaly_detector",
        output="screen",
        parameters=[{
            "residual_model_dir": [b, "/residual_ae_v4"],
            "raw_model_dir": [b, "/raw_ae_v4"],
            "fusion_config": [b, "/fusion_v4/fusion_config.json"],
            "current_to_torque": [b, "/current_to_torque.json"],
            "residual_calibration": [b, "/residual_calibration.json"],
            "solver_resources": [b, "/resources"],
            "use_case": LaunchConfiguration("use_case"),
            "quantile": LaunchConfiguration("quantile"),
            "tf_prefix": LaunchConfiguration("tf_prefix"),
            "joint_states_topic": LaunchConfiguration("joint_states_topic"),
            "adaptive": ParameterValue(LaunchConfiguration("adaptive"), value_type=bool),
            "adaptive_k": ParameterValue(LaunchConfiguration("adaptive_k"), value_type=float),
            "freeze_timeout": ParameterValue(LaunchConfiguration("freeze_timeout"), value_type=float),
            "motion_qd_min": ParameterValue(LaunchConfiguration("motion_qd_min"), value_type=float),
            "log_dir": LaunchConfiguration("log_dir"),
            "log_scores": ParameterValue(LaunchConfiguration("log_scores"), value_type=bool),
        }],
    )
    return LaunchDescription(args + [node])
