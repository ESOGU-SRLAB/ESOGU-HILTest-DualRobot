import os
from glob import glob

from setuptools import find_packages, setup

package_name = "gemini_robotics_ros"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        (os.path.join("share", package_name), ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Cem Süha Yılmaz",
    maintainer_email="cshyilmaz@gmail.com",
    description="Gemini Robotics ER 2 <-> ROS 2 / MoveIt köprüsü (UR10e, Gazebo Harmonic)",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "perception_node = gemini_robotics_ros.perception_node:main",
            "pick_place_node = gemini_robotics_ros.pick_place_node:main",
            "er_probe = gemini_robotics_ros.er_probe:main",
            "measure_tcp = gemini_robotics_ros.tool_geometry:main",
        ],
    },
)
