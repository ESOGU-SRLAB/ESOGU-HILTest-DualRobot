import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'viewpoint_planner'

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
    # open3d was previously listed here but is never imported anywhere in
    # this package -- dropped to avoid an unnecessary heavy dependency.
    install_requires=['setuptools', 'trimesh', 'scipy', 'numpy', 'matplotlib'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@todo.todo',
    description='Dual-robot viewpoint planning for chassis inspection',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'viewpoint_planner_node = viewpoint_planner.viewpoint_planner_node:main',
            'viewpoint_executor_node = viewpoint_planner.viewpoint_executor_node:main',
            'inspection_executor_node = viewpoint_planner.inspection_executor_node:main',
            'viewpoint_visualizer = viewpoint_planner.visualization:main',
            'plan_visualizer = viewpoint_planner.plan_visualizer:main',
        ],
    },
)
