# sir_robot_ros_interface - ROS2 Humble

✅ **Fully migrated from ROS1 to ROS2 Humble**

## Overview

This package provides a ROS2 interface for controlling Kawasaki RS005L robots. It includes:
- Custom message and service definitions
- C++ library for robot communication
- Test executables for robot control
- Python service client example

## Packages

### 1. agv_msgs
Custom messages and services for AGV communication.

**Messages (12):**
- BatteryData, ControlData, PowerData, RobotLow
- Route, Signal, TaskCom, TaskPlan
- TaskRequest, TaskStatus, Taskf, YskParam

**Services (1):**
- FabMsgs

### 2. sir_robot_ros_interface  
Main package for robot control interface.

**Messages (2):**
- Pose, Poses

**Services (3):**
- ManipulatorPose
- ManipulatorPoseIno
- ManipulatorPoseIno2

**Executables (5):**
- KawasakiRobotTestForLinux
- KawasakiRobotTestForLinux_Valu3s
- KawasakiRobotTestForLinux_Valu3s_ino_2
- KawasakiRobotJointPublisher
- ros_service_client.py

## Quick Start

### Build

```bash
cd /home/cem/colcon_ws
colcon build --packages-select agv_msgs sir_robot_ros_interface
source install/setup.bash
```

### Usage

```bash
# Run robot control nodes
ros2 run sir_robot_ros_interface KawasakiRobotTestForLinux
ros2 run sir_robot_ros_interface KawasakiRobotTestForLinux_Valu3s
ros2 run sir_robot_ros_interface KawasakiRobotTestForLinux_Valu3s_ino_2

# Run joint publisher
ros2 run sir_robot_ros_interface KawasakiRobotJointPublisher

# Run Python service client
ros2 run sir_robot_ros_interface ros_service_client.py
```

### Inspect Interfaces

```bash
# List all messages
ros2 interface list | grep agv_msgs
ros2 interface list | grep sir_robot_ros_interface

# Show message/service details
ros2 interface show agv_msgs/msg/TaskCom
ros2 interface show sir_robot_ros_interface/srv/ManipulatorPose

# List executables
ros2 pkg executables sir_robot_ros_interface
```

## Migration from ROS1

This package has been fully migrated from ROS1 to ROS2 Humble. For detailed migration notes, see [MIGRATION_NOTES.md](MIGRATION_NOTES.md).

### Key Changes:
- ✅ All C++ code migrated to `rclcpp`
- ✅ All Python code migrated to `rclpy`  
- ✅ Message and service definitions updated to ROS2 format
- ✅ Build system changed from `catkin` to `ament_cmake`
- ✅ All executables working in ROS2

## Dependencies

- ROS2 Humble
- rclcpp
- std_msgs
- sensor_msgs
- Eigen3

## License

TODO

## Maintainers

- Sezgin <sezgin@todo.todo>
- Didem <didem@todo.todo>
