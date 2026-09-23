from setuptools import setup

package_name = "anomaly_detection_v2"

setup(
    name=package_name,
    version="0.1.0",
    # Şimdilik gerçek bir ROS düğümü yok, yalnızca "anomaly_detection_v2/" adında
    # boş bir Python paketi var (__init__.py). İleride çevrimiçi dedektör düğümünü
    # buraya yazınca (features_v2.py, detector_v2.py, detector_node_v2.py gibi)
    # bu satır değişmeyecek, dosyalar otomatik pakete dahil olacak.
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Cem Süha Yılmaz",
    maintainer_email="cshyilmaz@gmail.com",
    description="UR10e anomali tespiti - sıfırdan yeniden yapım.",
    license="Apache-2.0",
)
