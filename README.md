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
UR10e robot with 2FG14 Gripper 

```bash
  ros2 launch my_robot_cell_gz gripperrobot_ifarlab_gazebo.launch.py use_gripper:=true
```
<img width="1911" height="948" alt="Screenshot from 2026-02-10 13-30-14" src="https://github.com/user-attachments/assets/18b3f464-3197-4398-b9a1-250a99f3ba74" />


  
