from setuptools import setup, find_packages

package_name = 'rs005l_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cem',
    maintainer_email='cem@example.com',
    description='RS005L PD and NN controllers with validation & logging',
    license='MIT',
    entry_points={
        'console_scripts': [
            'rs_controller = rs005l_controller.rs_controller:main',
            'nn_controller = rs005l_controller.nn_controller:main',
            'scenario_logger = rs005l_controller.scenario_logger:main',
            'trajectory_publisher = rs005l_controller.trajectory_publisher:main',
        ],
    },
)
