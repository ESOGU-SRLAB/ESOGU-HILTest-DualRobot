"""UR10e anomali tespiti düğümünü başlatır."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
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
                             default_value="/force_torque_sensor_broadcaster/wrench"),
        DeclareLaunchArgument("adaptive", default_value="false",
                             description="mutlak eşiğin yanında medyan+k·MAD kuralı; "
                                         "gerçek robotta ölçüldü, k=8..30 taramasının "
                                         "hiçbirinde gerçek olayı yakalamadı, yalnız "
                                         "yanlış alarm ekledi -- varsayılan kapalı"),
        DeclareLaunchArgument("adaptive_k", default_value="8.0"),
        DeclareLaunchArgument("freeze_timeout", default_value="3.0",
                             description="s; alarm bundan uzun sürerse rejim "
                                         "değişimi sayılır, taban çizgisi çözülür"),
        DeclareLaunchArgument("motion_qd_min", default_value="-1.0",
                             description="rad/s; UYARLANABİLİR taban çizgisinin kapısı, "
                                         "yalnız robot hareket ederken beslenir; "
                                         "negatif = kapı hep açık"),
        DeclareLaunchArgument("regime_qd_min", default_value="-1.0",
                             description="rad/s; EŞİK REJİMİNİ (duran/hareketli) seçen "
                                         "ayrı eşik. motion_qd_min ile karıştırılmamalı: "
                                         "bu değer çevrimdışı değerlendirmedekiyle birebir "
                                         "aynı olmalı. -1 = fusion_config.json ne diyorsa"),
        DeclareLaunchArgument("cell_calibrated", default_value="true",
                             description="eşiği ve normalleştirme sınırlarını GERÇEK "
                                         "HÜCREDE ölçülmüş config'den al (varsayılan). "
                                         "false = çevrimdışı doğrulamadan türetilmiş "
                                         "config; makalenin çevrimdışı tablosuyla aynı "
                                         "ama SAHADA KULLANILMAMALI — 2026-08-26'da "
                                         "kararların %58'i alarm oldu."),
        DeclareLaunchArgument("threshold_override", default_value="0.0",
                             description="pozitifse fusion_config.json'daki θ_mutlak "
                                         "yerine bu kullanılır"),
        DeclareLaunchArgument("log_dir", default_value=str(Path.home() / "anomali_kayit"),
                             description="olay/skor kayıtları; boş dize kaydı kapatır"),
        DeclareLaunchArgument("log_scores", default_value="true",
                             description="her kararı CSV'ye yaz (~170 MB/gün)"),
    ]
    b = LaunchConfiguration("models_base")
    # Paket 28.08.2026'dan beri YALNIZ v3 taşıyor. v2 artefaktları (sürtünmesiz
    # kalıntıyla eğitilmiş bildiri modelleri) backup_anomaly_detection/ altına
    # alındı; oradan reproduce_erratum.sh ile çevrimdışı yeniden üretilir.
    # Tek kuşak kalması, 26.08.2026'da yaşanan uyuşmazlığı yapısal olarak
    # imkânsız kılıyor: o gün model yolları v2'yi, friction_model ise v3'ü
    # gösteriyordu ve düğüm detector.py'deki kilitle hata verip durmuştu.
    # Çevrimdışı türetilen eşik bu donanıma taşınmıyor — bu çalışmanın merkezi
    # bulgusu. 2026-08-21'de v2 için ölçüldü (0,6436 → 18,0; 28 kat), 2026-08-26'da
    # v3 için tekrarlandı (0,8553 ile kararların %58'i alarm). Bu yüzden dağıtım
    # varsayılanı, temiz bir gerçek hücre koşusundan kalibre edilmiş config'dir;
    # bildirinin min–max formülü aynen korunur, yalnız sınırların ve θ'nın alındığı
    # küme değişir (calibrate_cell.py).
    cell = LaunchConfiguration("cell_calibrated")
    cfg = PythonExpression(
        ["'", b, "/fusion_v3/fusion_config_cell.json' if '", cell,
         "'.lower() == 'true' else '", b, "/fusion_v3/fusion_config.json'"])
    node = Node(
        package="anomaly_detection",
        executable="detector",
        name="ur10e_anomaly_detector",
        output="screen",
        parameters=[{
            "residual_model_dir": [b, "/residual_ae_v3"],
            "raw_model_dir": [b, "/raw_ae_v3"],
            "fusion_config": cfg,
            "current_to_torque": [b, "/current_to_torque.json"],
            "residual_calibration": [b, "/residual_calibration_fric.json"],
            "friction_model": [b, "/friction_model.json"],
            "solver_resources": [b, "/resources"],
            "tf_prefix": LaunchConfiguration("tf_prefix"),
            "joint_states_topic": LaunchConfiguration("joint_states_topic"),
            "wrench_topic": LaunchConfiguration("wrench_topic"),
            "adaptive": ParameterValue(LaunchConfiguration("adaptive"), value_type=bool),
            "adaptive_k": ParameterValue(LaunchConfiguration("adaptive_k"), value_type=float),
            "freeze_timeout": ParameterValue(LaunchConfiguration("freeze_timeout"), value_type=float),
            "motion_qd_min": ParameterValue(LaunchConfiguration("motion_qd_min"), value_type=float),
            "regime_qd_min": ParameterValue(LaunchConfiguration("regime_qd_min"), value_type=float),
            "threshold_override": ParameterValue(LaunchConfiguration("threshold_override"), value_type=float),
            "log_dir": LaunchConfiguration("log_dir"),
            "log_scores": ParameterValue(LaunchConfiguration("log_scores"), value_type=bool),
        }],
    )
    return LaunchDescription(args + [node])
