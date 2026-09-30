from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'agv_nodes'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools', 'roslibpy'],
    zip_safe=True,
    maintainer='cem',
    maintainer_email='cem@todo.com',
    description='ROS2 nodes for AGV control via rosbridge',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bridge_node = agv_nodes.bridge_node:main',
            'position_node = agv_nodes.position_node:main',
            'movement_node = agv_nodes.movement_node:main',
            'agv_controller_node = agv_nodes.agv_controller_node:main',
        ],
    },
)
