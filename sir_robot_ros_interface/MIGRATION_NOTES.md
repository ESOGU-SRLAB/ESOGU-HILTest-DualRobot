# ROS1 to ROS2 Humble Migration Notes

## Migrated Packages

### 1. agv_msgs
✅ **Successfully migrated and built**

**Changes made:**
- Updated `package.xml` from format 2 to format 3
- Changed `catkin` to `ament_cmake`
- Updated dependencies: `message_generation/message_runtime` → `rosidl_default_generators/rosidl_default_runtime`
- Updated `CMakeLists.txt` to use ROS2 build system
- Renamed service file: `fab_msgs.srv` → `FabMsgs.srv` (ROS2 naming convention)
- Fixed service separator: `----` → `---`

**Available interfaces:**
- 12 messages: BatteryData, ControlData, PowerData, RobotLow, Route, Signal, TaskCom, TaskPlan, TaskRequest, TaskStatus, Taskf, YskParam
- 1 service: FabMsgs

### 2. sir_robot_ros_interface
✅ **Successfully migrated and built** (library, messages/services, and executables)

**Changes made:**
- Updated `package.xml` from format 2 to format 3
- Changed build system from `catkin` to `ament_cmake`
- Updated dependencies:
  - `roscpp/rospy` → `rclcpp/rclpy`
  - `message_generation/message_runtime` → `rosidl_default_generators/rosidl_default_runtime`
  - Added `eigen3_cmake_module` for Eigen3 support
  - Added `sensor_msgs` dependency
- Updated `CMakeLists.txt` to use ROS2 build system
- Renamed library target: `sir_robot_ros_interface` → `sir_robot_ros_interface_lib` (to avoid naming conflict with package)
- Renamed service files (ROS2 naming convention):
  - `ManipulatorPose_ino.srv` → `ManipulatorPoseIno.srv`
  - `ManipulatorPose_ino_2.srv` → `ManipulatorPoseIno2.srv`
- Fixed Eigen includes:
  - `#include <Core>` → `#include <Eigen/Core>`
  - `#include <Dense>` → `#include <Eigen/Dense>`
  - `#include <StdVector>` → `#include <Eigen/StdVector>`
- Used modern ROS2 API: `rosidl_get_typesupport_target()` instead of deprecated `rosidl_target_interfaces()`

**Available interfaces:**
- 2 messages: Pose, Poses
- 3 services: ManipulatorPose, ManipulatorPoseIno, ManipulatorPoseIno2

**✅ All test executables migrated to ROS2:**
The following executables have been successfully migrated and are ready to use:
- `KawasakiRobotTestForLinux` - Main robot control node
- `KawasakiRobotTestForLinux_Valu3s` - Service-based robot control
- `KawasakiRobotTestForLinux_Valu3s_ino_2` - Advanced robot control with path planning
- `KawasakiRobotJointPublisher` - Joint state publisher
- `ros_service_client.py` - Python service client

**Migrated files:**
- `include/khi_ota_comm.h` - Updated to use `rclcpp` and ROS2 message types
- `include/khi_ota_comm_valu3s.h` - Updated to use `rclcpp` and ROS2 message types
- `tests/KawasakiRobotTestForLinux.cpp` - Fully migrated to ROS2
- `tests/KawasakiRobotTestForLinux_Valu3s.cpp` - Fully migrated to ROS2
- `tests/KawasakiRobotTestForLinux_Valu3s_ino_2.cpp` - Fully migrated to ROS2
- `tests/KawasakiRobotJointPublisher.cpp` - Fully migrated to ROS2
- `scripts/ros_service_client.py` - Fully migrated to ROS2

## Building the Packages

```bash
cd /home/cem/colcon_ws
colcon build --packages-select agv_msgs sir_robot_ros_interface
source install/setup.bash
```

## Running the Executables

```bash
# Source the workspace
source /home/cem/colcon_ws/install/setup.bash

# Run executables
ros2 run sir_robot_ros_interface KawasakiRobotTestForLinux
ros2 run sir_robot_ros_interface KawasakiRobotTestForLinux_Valu3s
ros2 run sir_robot_ros_interface KawasakiRobotTestForLinux_Valu3s_ino_2
ros2 run sir_robot_ros_interface KawasakiRobotJointPublisher
ros2 run sir_robot_ros_interface ros_service_client.py
```

## Testing

Check available interfaces:
```bash
# agv_msgs
ros2 interface list | grep agv_msgs
ros2 interface show agv_msgs/msg/BatteryData
ros2 interface show agv_msgs/srv/FabMsgs

# sir_robot_ros_interface
ros2 interface list | grep sir_robot_ros_interface
ros2 interface show sir_robot_ros_interface/msg/Pose
ros2 interface show sir_robot_ros_interface/srv/ManipulatorPose

# List executables
ros2 pkg executables sir_robot_ros_interface
```

## Key ROS2 Migration Changes Made

### C++ Code Changes:

1. **Headers:**
   - `ros/ros.h` → `rclcpp/rclcpp.hpp`
   - `std_msgs/String.h` → `std_msgs/msg/string.hpp`
   - `agv_msgs/TaskCom.h` → `agv_msgs/msg/task_com.hpp`
   - `sensor_msgs/JointState.h` → `sensor_msgs/msg/joint_state.hpp`
   - `<StdVector>` → `<Eigen/StdVector>`

2. **Message Types:**
   - `ConstPtr&` → `SharedPtr`
   - `agv_msgs::TaskCom` → `agv_msgs::msg::TaskCom`
   - `std_msgs::String` → `std_msgs::msg::String`
   - Access: `msg->data` (same) but type changed

3. **Node Creation:**
   ```cpp
   // ROS1
   ros::init(argc, argv, "node_name");
   ros::NodeHandle n;
   
   // ROS2
   rclcpp::init(argc, argv);
   auto node = rclcpp::Node::make_shared("node_name");
   ```

4. **Publishers:**
   ```cpp
   // ROS1
   ros::Publisher pub = n.advertise<Type>("topic", 10);
   pub.publish(msg);
   
   // ROS2
   auto pub = node->create_publisher<Type>("topic", 10);
   pub->publish(msg);
   ```

5. **Subscribers:**
   ```cpp
   // ROS1
   ros::Subscriber sub = n.subscribe("topic", 10, callback);
   // or with class method:
   ros::Subscriber sub = n.subscribe("topic", 10, &Class::method, &instance);
   
   // ROS2
   auto sub = node->create_subscription<Type>("topic", 10, callback);
   // or with std::bind:
   auto sub = node->create_subscription<Type>(
       "topic", 10, 
       std::bind(&Class::method, &instance, std::placeholders::_1));
   ```

6. **Services:**
   ```cpp
   // ROS1
   ros::ServiceServer service = n.advertiseService("service", callback);
   bool callback(Request &req, Response &res) { ... return true; }
   
   // ROS2
   auto service = node->create_service<Type>("service", callback);
   void callback(const std::shared_ptr<Request> req, 
                 std::shared_ptr<Response> res) { ... }
   ```

7. **Spinning:**
   ```cpp
   // ROS1
   ros::spin();          // blocking
   ros::spinOnce();      // non-blocking
   
   // ROS2
   rclcpp::spin(node);           // blocking
   rclcpp::spin_some(node);      // non-blocking
   ```

8. **Timing:**
   ```cpp
   // ROS1
   ros::Rate rate(10);
   ros::Duration(5.0).sleep();
   ros::Time::now();
   
   // ROS2
   rclcpp::Rate rate(10);
   rclcpp::sleep_for(std::chrono::seconds(5));
   rclcpp::Clock().now();
   ```

### Python Code Changes:

1. **Import:**
   ```python
   # ROS1
   import rospy
   from pkg.srv import ServiceName, ServiceNameRequest, ServiceNameResponse
   
   # ROS2
   import rclpy
   from rclpy.node import Node
   from pkg.srv import ServiceName
   ```

2. **Node Creation:**
   ```python
   # ROS1
   rospy.init_node('node_name')
   
   # ROS2
   rclpy.init()
   node = Node('node_name')
   ```

3. **Service Client:**
   ```python
   # ROS1
   client = rospy.ServiceProxy('service', ServiceType)
   rospy.wait_for_service('service')
   response = client(request)
   
   # ROS2
   client = node.create_client(ServiceType, 'service')
   client.wait_for_service(timeout_sec=5.0)
   future = client.call_async(request)
   rclpy.spin_until_future_complete(node, future)
   response = future.result()
   ```

4. **Cleanup:**
   ```python
   # ROS1
   # (automatic)
   
   # ROS2
   node.destroy_node()
   rclpy.shutdown()
   ```

## Common ROS1 to ROS2 Changes

### C++ API Changes
| ROS1 | ROS2 |
|------|------|
| `ros::init()` | `rclcpp::init()` |
| `ros::NodeHandle nh` | `auto node = std::make_shared<rclcpp::Node>("name")` |
| `nh.advertise<Type>("topic", 10)` | `node->create_publisher<Type>("topic", 10)` |
| `nh.subscribe("topic", 10, callback)` | `node->create_subscription<Type>("topic", 10, callback)` |
| `nh.serviceClient<Type>("service")` | `node->create_client<Type>("service")` |
| `nh.advertiseService("service", callback)` | `node->create_service<Type>("service", callback)` |
| `ros::spin()` | `rclcpp::spin(node)` |
| `ros::Rate rate(10)` | `rclcpp::Rate rate(10)` |
| `ROS_INFO()` | `RCLCPP_INFO(node->get_logger(), ...)` |

### Build System Changes
| ROS1 | ROS2 |
|------|------|
| `catkin` | `ament_cmake` |
| `package.xml` format 2 | `package.xml` format 3 |
| `catkin_package()` | `ament_package()` |
| `find_package(catkin ...)` | `find_package(ament_cmake ...)` |
| `add_message_files()` | `rosidl_generate_interfaces()` |
| `generate_messages()` | (included in rosidl_generate_interfaces) |
| `${catkin_INCLUDE_DIRS}` | `ament_target_dependencies()` |
| `${catkin_LIBRARIES}` | `ament_target_dependencies()` |

## References
- [ROS2 Migration Guide](https://docs.ros.org/en/humble/The-ROS2-Project/Contributing/Migration-Guide.html)
- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [ament_cmake User Documentation](https://docs.ros.org/en/humble/How-To-Guides/Ament-CMake-Documentation.html)
