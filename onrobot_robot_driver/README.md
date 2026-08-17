The **OnRobot Robot Driver** package provides a hardware interface for controlling the OnRobot 2FG7 gripper with ROS2 Control. It includes both configuration and launch files, as well as a C++ API to facilitate hardware interaction.

## ROS Control

The `onrobot_robot_driver` package integrates with **ROS2 Control** to support the following functionalities:

- **Position Command**: Allows sending position commands to the gripper.
- **Position State**: Provides feedback on the current joint positions.
- **Velocity State**: Provides feedback on the current joint velocities.

---

## Dependencies

- **ros2_control**: The package is built on ros2_control and requires a working ROS2 installation.
- **OnRobot API**: The driver is based on the [OnRobot API](https://github.com/touchlab-avatarx/onrobot_api) library. Add this package to your workspace as a source checkout using vcs.
- **onrobot_2fg7_description**: [Robot description](https://github.com/touchlab-avatarx/onrobot_2fg7_description) Add this package to your workspace as a source checkout using vcs.

---

## Installation

Follow [Colcon](https://colcon.readthedocs.io/en/released/user/quick-start.html) instructions for colcon package installation.

---

## Example usage

Start the controller:
```
ros2 launch onrobot_robot_driver onrobot_control.launch.py
```
