import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'multirobot_viewpoint_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'scipy', 'numpy'],
    zip_safe=True,
    maintainer='Cem',
    maintainer_email='cshyilmaz@gmail.com',
    description='Multi-robot (UR10e + Kawasaki) cooperative viewpoint planning for chassis inspection',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'multirobot_planner_node = multirobot_viewpoint_planner.multirobot_planner_node:main',
            'ur_inspection_node = multirobot_viewpoint_planner.ur_inspection_node:main',
            'kawasaki_inspection_node = multirobot_viewpoint_planner.kawasaki_inspection_node:main',
            'multirobot_viewpoint_visualizer = multirobot_viewpoint_planner.visualization:main',
            'multirobot_plan_visualizer = multirobot_viewpoint_planner.plan_visualizer:main',
        ],
    },
)
