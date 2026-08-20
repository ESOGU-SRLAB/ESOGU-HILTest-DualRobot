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
    version="2.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        ("share/" + package_name + "/launch", ["launch/detector.launch.py"]),
        # ── çalışma zamanı varlıkları ──
        # Düğüm bunları kurulu share dizininden okur; böylece depoyu klonlayan
        # herkeste mutlak yol düzenlemesi gerekmeden çalışır.
        assets("residual_ae_v2"),
        assets("raw_ae_v2"),
        assets("fusion_v2"),
        assets("resources"),
        assets("resources/schemas"),
        ("share/" + package_name,
         ["current_to_torque.json", "residual_calibration_clean.json"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Cem Süha Yılmaz",
    maintainer_email="cshyilmaz@gmail.com",
    description="UR10e çevrimiçi anomali tespiti: FMU tabanlı kalıntı ayrıştırma ve "
                "ikili LSTM özkodlayıcı birleşimi (245.pdf).",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "detector = anomaly_detection.detector_node:main",
            "replay_publisher = anomaly_detection.replay_publisher:main",
        ],
    },
)
