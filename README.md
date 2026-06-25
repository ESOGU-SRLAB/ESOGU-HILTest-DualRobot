This repository contains the ROS 2 packages for both a dual-robot UR10e system and Hardware-in-the-Loop (HIL) test system developed at the Intelligent Factory and Robotics Laboratory (IFARLAB) of Eskişehir Osmangazi University (ESOGU).
![WhatsApp Image 2026-01-05 at 12 37 18](https://github.com/user-attachments/assets/e54bcc60-3736-45e5-98fb-7963c768c673)

## Requirements

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Harmonic (for simulation)

⚠️ Important Setup Step: Updating File Paths

Some `.xacro` files reference meshes using **absolute file paths**. To run this repository on your system you must update these paths to match your own username and workspace directory.

For example, line 167 of `my_robot_cell_macro.xacro` (in the `my_robot_cell_description` package) looks like this:

```xml
    <mesh filename="file:////home/cem/colcon_ws/src/Universal_Robots_ROS2_Description/meshes/ur10e/collision/linear_axis_moving_link.stl" scale="0.001 0.001 0.001"/>
```

You must replace the `/home/cem/` part with your own username. For example, if your username is `ifarlab`:

```xml
    <mesh filename="file:////home/ifarlab/colcon_ws/src/Universal_Robots_ROS2_Description/meshes/ur10e/collision/linear_axis_moving_link.stl" scale="0.001 0.001 0.001"/>
```
Getting Started

Clone the project.
```bash
    cd ~/colcon_ws/src
    git clone -b v2 https://github.com/ESOGU-SRLAB/ESOGU-DualRobot.git
```
Move the files coming to the ESOGU-HILTest-DualRobot cluster to the /src directory.

Install dependencies,
```bash
    cd ~/colcon_ws
    sudo rosdep init   # only needed the first time; safe to skip if it reports "already initialized"
    rosdep update
    rosdep install --from-paths src -y --ignore-src
```
Set the Gazebo Harmonic environment variable, then build the workspace. The `gz_ros2_control` package must be built against Gazebo Harmonic, so it is built explicitly after the main build:

```bash
    echo 'export GZ_VERSION=harmonic' >> ~/.bashrc
    source ~/.bashrc

    cd ~/colcon_ws
    colcon build
    colcon build --packages-select gz_ros2_control --cmake-args -DCMAKE_BUILD_TYPE=Release
    source install/setup.bash
```

The workspace is now ready.

## Running the System

To launch the full system:

```bash
ros2 launch my_robot_cell_control hil_test_whole_unified.launch.py
```

If you are working **without physical robots**, run with `use_fake_hardware:=true`:

```bash
ros2 launch my_robot_cell_control hil_test_whole_unified.launch.py use_fake_hardware:=true
```

Implementations for OnRobot VGC10 vacuum gripper and OnRobot 2FG14 gripper completed, both of them can enable with parameters.
For the **OnRobot VGC10 vacuum gripper without physical robots**:

```bash
ros2 launch my_robot_cell_control hil_test_whole_unified.launch.py use_fake_hardware:=true use_vacuum_gripper:=true
```

For the **OnRobot 2FG14 gripper without physical robots**:

```bash
ros2 launch my_robot_cell_control hil_test_whole_unified.launch.py use_fake_hardware:=true use_gripper:=true
```

<img width="1501" height="906" alt="2026-06-25T09:01:37 750723920" src="https://github.com/user-attachments/assets/84b38ea4-c768-4ee4-8f91-13e0ed1b9667" />


  
