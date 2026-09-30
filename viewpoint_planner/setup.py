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
    # open3d IS imported (inspection_base writes the captured clouds as .pcd via
    # o3d.io.write_point_cloud); an earlier comment here claimed otherwise.
    install_requires=['setuptools', 'trimesh', 'scipy', 'numpy', 'matplotlib', 'open3d'],
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
