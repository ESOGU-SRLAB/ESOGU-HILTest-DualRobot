#!/usr/bin/python3

# ===================================== COPYRIGHT ===================================== #
#                                                                                       #
#  IFRA (Intelligent Flexible Robotics and Assembly) Group, CRANFIELD UNIVERSITY        #
#  Created on behalf of the IFRA Group at Cranfield University, United Kingdom          #
#  E-mail: IFRA@cranfield.ac.uk                                                         #
#                                                                                       #
#  Licensed under the Apache-2.0 License.                                               #
#  You may not use this file except in compliance with the License.                     #
#  You may obtain a copy of the License at: http://www.apache.org/licenses/LICENSE-2.0  #
#                                                                                       #
#  Unless required by applicable law or agreed to in writing, software distributed      #
#  under the License is distributed on an "as-is" basis, without warranties or          #
#  conditions of any kind, either express or implied. See the License for the specific  #
#  language governing permissions and limitations under the License.                    #
#                                                                                       #
#  IFRA Group - Cranfield University                                                    #
#  AUTHORS: Mikel Bueno Viso - Mikel.Bueno-Viso@cranfield.ac.uk                         #
#           Dr. Seemal Asif  - s.asif@cranfield.ac.uk                                   #
#           Prof. Phil Webb  - p.f.webb@cranfield.ac.uk                                 #
#                                                                                       #
#  Date: June, 2023.                                                                    #
#                                                                                       #
# ===================================== COPYRIGHT ===================================== #

# ======= CITE OUR WORK ======= #
# You can cite our work with the following statement:
# IFRA-Cranfield (2023) Gazebo-ROS 2 Conveyor Belt Plugin. URL: https://github.com/IFRA-Cranfield/IFRA_ConveyorBelt.

# conveyorbelt.launch.py:
# Launch file for the IFRA_ConveyorBelt GAZEBO IGNITION (Fortress) SIMULATION in ROS 2 Humble.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg = get_package_share_directory('conveyorbelt_gz')
    # share_root = .../install/conveyorbelt_gz/share
    # Gazebo resolves model://conveyorbelt_gz/... relative to this directory.
    share_root = os.path.dirname(pkg)

    conveyor_sdf = os.path.join(pkg, 'sdf', 'conveyor.sdf')

    # ── Resource paths so model://conveyorbelt_gz/meshes/... resolves correctly ──
    set_ign_resource = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[
            share_root, ':',
            EnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', default_value=''),
        ],
    )
    set_gz_resource = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            share_root, ':',
            EnvironmentVariable('GZ_SIM_RESOURCE_PATH', default_value=''),
        ],
    )

    # ── Plugin path so libros2gz_conveyorbelt_system.so is found ──
    set_plugin_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_SYSTEM_PLUGIN_PATH',
        value=[
            os.path.join(os.path.dirname(share_root), 'lib'), ':',
            EnvironmentVariable('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', default_value=''),
        ],
    )

    # ── Launch Ignition Gazebo (Fortress) with empty world ──
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare('ros_gz_sim'), '/launch/gz_sim.launch.py']
        ),
        launch_arguments={
            'gz_args': '-r -v 1 empty.sdf',
            'on_exit_shutdown': 'true',
        }.items(),
    )

    # ── Spawn conveyor model ──
    spawn_conveyor = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_conveyor',
        arguments=[
            '-file', conveyor_sdf,
            '-name', 'conveyor',
            '-x', '0', '-y', '0', '-z', '0',
        ],
        output='screen',
    )

    # ── Spawn RedCube ──
    redcube_sdf = os.path.join(pkg, 'sdf', 'RedCube.sdf')

    spawn_redcube = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_redcube',
        arguments=[
            '-file', redcube_sdf,
            '-name', 'RedCube',
            '-x', '0.0', '-y', '-0.5', '-z', '0.9',
        ],
        output='screen',
    )

    # ── ROS↔Gazebo bridge: clock + conveyor topics ──
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
        ],
        output='screen',
    )

    return LaunchDescription([
        set_ign_resource,
        set_gz_resource,
        set_plugin_path,
        gz_sim,
        spawn_conveyor,
        spawn_redcube,
        gz_bridge,
    ])