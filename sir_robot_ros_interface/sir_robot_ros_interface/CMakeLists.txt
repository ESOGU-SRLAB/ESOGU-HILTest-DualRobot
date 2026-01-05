cmake_minimum_required(VERSION 3.8)
project(sir_robot_ros_interface)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(agv_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

# Generate messages and services
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Pose.msg"
  "msg/Poses.msg"
  "srv/ManipulatorPose.srv"
  "srv/ManipulatorPoseIno.srv"
  "srv/ManipulatorPoseIno2.srv"
  DEPENDENCIES std_msgs
)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# Build library
add_library(sir_robot_ros_interface_lib
  src/SIRLogger.cpp
  src/SIRRobot/KawasakiRS005LRobot.cpp
  src/SIRRobot/SIRConnection.cpp
  src/SIRRobot/SIRLinConnection.cpp
  src/SIRRobot/SIRPacketPackager.cpp
  src/SIRRobot/SIRPacketParser.cpp
  src/SIRRobot/SIRPacketTest.cpp
  src/SIRRobot/SIRPoseDB.cpp
  src/SIRRobot/SIRPoseQueueDB.cpp
  src/SIRRobot/SIRRobot.cpp
  src/SIRRobot/SIRRobotCommands.cpp
  src/SIRRobot/SIRRobotPacketPackager.cpp
  src/SIRRobot/SIRRobotPacketParser.cpp
  src/SIRRobot/SIRWinConnection.cpp
)

ament_target_dependencies(sir_robot_ros_interface_lib
  rclcpp
  std_msgs
  agv_msgs
  Eigen3
)

# This is required to use the messages/services from the same package
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(sir_robot_ros_interface_lib "${cpp_typesupport_target}")

# Install library
install(TARGETS sir_robot_ros_interface_lib
  EXPORT sir_robot_ros_interface_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Install documentation (commented out - directory does not exist)
# install(DIRECTORY doc
#   DESTINATION share/${PROJECT_NAME}
# )

# Export dependencies
ament_export_dependencies(
  rclcpp
  std_msgs
  agv_msgs
  Eigen3
  rosidl_default_runtime
)

ament_export_include_directories(include)
ament_export_libraries(sir_robot_ros_interface_lib)
ament_export_targets(sir_robot_ros_interface_lib)

# Build executables
add_executable(KawasakiRobotTestForLinux tests/KawasakiRobotTestForLinux.cpp)
ament_target_dependencies(KawasakiRobotTestForLinux rclcpp std_msgs agv_msgs Eigen3)
target_link_libraries(KawasakiRobotTestForLinux sir_robot_ros_interface_lib "${cpp_typesupport_target}")

add_executable(KawasakiRobotTestForLinux_Valu3s tests/KawasakiRobotTestForLinux_Valu3s.cpp)
ament_target_dependencies(KawasakiRobotTestForLinux_Valu3s rclcpp std_msgs agv_msgs Eigen3)
target_link_libraries(KawasakiRobotTestForLinux_Valu3s sir_robot_ros_interface_lib "${cpp_typesupport_target}")

add_executable(KawasakiRobotTestForLinux_Valu3s_ino_2 tests/KawasakiRobotTestForLinux_Valu3s_ino_2.cpp)
ament_target_dependencies(KawasakiRobotTestForLinux_Valu3s_ino_2 rclcpp std_msgs agv_msgs Eigen3)
target_link_libraries(KawasakiRobotTestForLinux_Valu3s_ino_2 sir_robot_ros_interface_lib "${cpp_typesupport_target}")

add_executable(KawasakiRobotJointPublisher tests/KawasakiRobotJointPublisher.cpp)
ament_target_dependencies(KawasakiRobotJointPublisher rclcpp std_msgs sensor_msgs Eigen3)
target_link_libraries(KawasakiRobotJointPublisher sir_robot_ros_interface_lib "${cpp_typesupport_target}")

add_executable(ROS2KawasakiRobotConnectionTest tests/ROS2KawasakiRobotConnectionTest.cpp)
ament_target_dependencies(ROS2KawasakiRobotConnectionTest rclcpp std_msgs agv_msgs Eigen3)
target_link_libraries(ROS2KawasakiRobotConnectionTest sir_robot_ros_interface_lib "${cpp_typesupport_target}")

add_executable(ROS2KawasakiRobotJointStatePublisher tests/ROS2KawasakiRobotJointStatePublisher.cpp)
ament_target_dependencies(ROS2KawasakiRobotJointStatePublisher rclcpp std_msgs sensor_msgs agv_msgs Eigen3)
target_link_libraries(ROS2KawasakiRobotJointStatePublisher sir_robot_ros_interface_lib "${cpp_typesupport_target}")

add_executable(ROS2KawasakiRobotGazeboJSP tests/ROS2KawasakiRobotGazeboJSP.cpp)
ament_target_dependencies(ROS2KawasakiRobotGazeboJSP rclcpp std_msgs sensor_msgs trajectory_msgs agv_msgs Eigen3)
target_link_libraries(ROS2KawasakiRobotGazeboJSP sir_robot_ros_interface_lib "${cpp_typesupport_target}")

add_executable(ROS2KawasakiRobotMove tests/ROS2KawasakiRobotMove.cpp)
ament_target_dependencies(ROS2KawasakiRobotMove rclcpp std_msgs sensor_msgs trajectory_msgs agv_msgs Eigen3)
target_link_libraries(ROS2KawasakiRobotMove sir_robot_ros_interface_lib "${cpp_typesupport_target}")

# Install executables
install(TARGETS
  KawasakiRobotTestForLinux
  KawasakiRobotTestForLinux_Valu3s
  KawasakiRobotTestForLinux_Valu3s_ino_2
  KawasakiRobotJointPublisher
  ROS2KawasakiRobotConnectionTest
  ROS2KawasakiRobotJointStatePublisher
  ROS2KawasakiRobotGazeboJSP
  ROS2KawasakiRobotMove
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/ros_service_client.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()