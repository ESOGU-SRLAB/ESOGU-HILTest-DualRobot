import os
from glob import glob

from setuptools import setup

package_name = "anomaly_detection"


def assets(subdir, pattern="*"):
    """Çalışma zamanı varlıklarını share/ altına aynı yapıyla kurar."""
    return (f"share/{package_name}/{subdir}",
            [f for f in glob(f"{subdir}/{pattern}") if os.path.isfile(f)])


setup(
    name=package_name,
    version="4.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        ("share/" + package_name + "/launch", ["launch/detector.launch.py"]),
        # ── çalışma zamanı varlıkları ──
        # Düğüm bunları kurulu share dizininden okur; böylece depoyu klonlayan
        # herkeste mutlak yol düzenlemesi gerekmez.
        # v4 — DAĞITIM VARSAYILANI. KTS yok, senaryo-başına yük/sürtünme düzeltmesi,
        # ölçülmüş birleşim ağırlığı. v3 (gerçek hücrede kalibre edilmiş ama KTS'li
        # eski nesil) backup_anomaly_detection/anomaly_detection_v3_real_cell/
        # altında duruyor - SİLİNMEDİ, yalnız kurulum yolundan çıkarıldı.
        assets("residual_ae_v4"),
        assets("raw_ae_v4"),
        assets("fusion_v4"),
        assets("resources"),
        assets("resources/schemas"),
        # Sürtünme + senaryo-başına ofset/yük - TEK dosya (eskiden iki ayrı
        # dosyaydı: friction_model.json + residual_calibration_fric.json).
        ("share/" + package_name, ["current_to_torque.json", "residual_calibration.json"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Cem Süha Yılmaz",
    maintainer_email="cshyilmaz@gmail.com",
    description="UR10e çevrimiçi anomali tespiti: KTS'siz kalıntı + ham LSTM "
                "özkodlayıcı birleşimi, senaryo başına kalibre edilmiş düzeltme.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "detector = anomaly_detection.detector_node:main",
            "replay_publisher = anomaly_detection.replay_publisher:main",
            # Kaydedilmiş bir oturumu arayüze geri yayınlar (robot/dedektör gerekmez).
            "replay_scores = anomaly_detection.replay_scores:main",
        ],
    },
)
