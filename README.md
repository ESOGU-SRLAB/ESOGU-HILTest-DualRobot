This repository contains the ROS 2 packages for both a dual-robot UR10e system and Hardware-in-the-Loop (HIL) test system developed at the Intelligent Factory and Robotics Laboratory (IFARLAB) of Eskişehir Osmangazi University (ESOGU).

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
<img width="1948" height="988" alt="Screenshot from 2025-08-27 16-16-42" src="https://github.com/user-attachments/assets/a59565eb-ebab-4ada-a54d-641ddbddd034" />

<img width="1948" height="988" alt="Screenshot from 2025-08-27 16-17-20" src="https://github.com/user-attachments/assets/d3c00fb4-e9b3-487c-9a17-97796a6eeb29" />
After the system is up and running, you can use the following command to start the MoveIt 2! controllers and move the robots:

```bash
  ros2 run pymoveit2_real combined_joint_goal.py
```

System 2: Dual UR10e Robot Gazebo Simulation
This option launches a Gazebo simulation environment consisting only of a dual UR10e robot setup. It is used for working entirely in simulation without real robot hardware.
To launch the dual-robot Gazebo simulation, run the following command:
```bash
  ros2 launch my_robot_cell_gz dualrobot_ifarlab_gazebo.launch.py
```
<img width="1948" height="988" alt="Screenshot from 2025-08-27 16-18-28" src="https://github.com/user-attachments/assets/fd330ce5-c2e8-4fb4-b95e-8045d51d5da3" />

Once the simulation environment is running, you can use the following commands to move the robots via MoveIt 2!.
To move the robots by providing x, y, z position goals, enter the following command:
```bash
  ros2 launch my_robot_cell_gz dual_robot_moveit_program.launch.py
```
To move the robots by providing target joint angles, enter the following command:
```bash
  ros2 launch my_robot_cell_gz dual_robot_joint_moveit_program.launch.py
```


  
