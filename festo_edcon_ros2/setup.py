from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'festo_edcon_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Action files
        (os.path.join('share', package_name, 'action'), glob('action/*.action')),
        # Launch files (eğer varsa)
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cem',
    maintainer_email='cem@example.com',
    description='Festo EDCON ROS2 Package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'linear_axis_runner = festo_edcon_ros2.linear_axis_runner:main',
            'linear_axis_adapter = festo_edcon_ros2.linear_axis_adapter:main',
            'linear_axis_action_server = festo_edcon_ros2.linear_axis_action_server:main',
        ],
    },
)