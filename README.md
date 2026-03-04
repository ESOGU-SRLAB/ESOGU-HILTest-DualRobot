This repository contains the ROS 2 packages for both a dual-robot UR10e system and Hardware-in-the-Loop (HIL) test system developed at the Intelligent Factory and Robotics Laboratory (IFARLAB) of Eskişehir Osmangazi University (ESOGU).
![WhatsApp Image 2026-01-05 at 12 37 18](https://github.com/user-attachments/assets/e54bcc60-3736-45e5-98fb-7963c768c673)
⚠️ Important Setup Step: Updating File Paths

The current file structure uses absolute file paths within some .xacro files. To run this repository on your system without issues, you must update these paths according to your own username and workspace directory.

TODO: Before using the repository, edit the specified file path below and any other similar absolute paths to match your system configuration.

For example, line 83 of the firstrobot_my_robot_cell_macro.xacro file, located in the my_robot_cell_description package, is as follows:
```bash
    <mesh filename="file:////home/cem/colcon_ws/src/Universal_Robots_ROS2_Description/meshes/ur10e/collision/linear_axis_moving_link.stl" scale="0.001 0.001 0.001"/>
```
You must replace the /home/cem/ part in this line with your own username. For example, if your username is ifarlab, the line should be updated as follows:
```bash
    <mesh filename="file:////home/ifarlab/colcon_ws/src/Universal_Robots_ROS2_Description/meshes/ur10e/collision/linear_axis_moving_link.stl" scale="0.001 0.001 0.001"/>
```
Getting Started

Clone the project.
```bash
    cd ~/colcon_ws/src
    git clone https://github.com/ESOGU-SRLAB/ESOGU-DualRobot.git
```
Move the files coming to the ESOGU-HILTest-DualRobot cluster to the /src directory.

Install dependencies,
```bash
    cd ~/colcon_ws
    rosdep init
    rosdep update
    rosdep install --from-paths src -y --ignore-src
```
build the workspace
```bash
    colcon build
    source install/setup.bash
```
Two primary systems can be run with this repository.

System 1: HIL Test System (Real + Simulation)

This system provides a Hardware-in-the-Loop (HIL) test environment that runs the real robot system at IFARLAB and the Gazebo simulation simultaneously.

```bash
  ros2 launch my_robot_cell_control hil_test.launch.py
```
If you want to test the system without real robots using fake_hardware, you can add the use_fake_hardware:=true parameter to the command:
```bash
  ros2 launch my_robot_cell_control hil_test.launch.py use_fake_hardware:=true
```
<img width="1918" height="995" alt="image" src="https://github.com/user-attachments/assets/53fff91c-f95a-47ab-8b29-73f854e41077" />

After the system is up and running, you can use the following command to start the MoveIt 2! controllers and move the robots:

```bash
  ros2 run pymoveit2_real combined_joint_goal.py
```

System 2: Dual Robot HIL Test System (Kawasaki RS005L + UR10e)
This option launches a Gazebo simulation environment consisting only of a dual UR10e robot setup. It is used for working entirely in simulation without real robot hardware.
Currently, that system is developing.
To launch the dual-robot Gazebo simulation, run the following command:
```bash
  ros2 launch mobile_manipulator_description whole_ifarlab_gazebo.launch.py
```
<img width="1918" height="995" alt="image" src="https://github.com/user-attachments/assets/45eab4f8-863c-4745-8bb2-bee5ec282690" />



  
