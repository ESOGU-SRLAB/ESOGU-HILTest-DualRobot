cmake_minimum_required(VERSION 3.8)
project(my_robot_cell_control)

# C++ standard
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

# Build real_to_sim_bridge executable
add_executable(real_to_sim_bridge src/real_to_sim_bridge.cpp)
ament_target_dependencies(real_to_sim_bridge
  rclcpp
  sensor_msgs
  trajectory_msgs
)

# Install executables
install(TARGETS
  real_to_sim_bridge
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config urdf rviz scripts
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/real_to_gazebo.py
  scripts/usb_camera_node.py
DESTINATION lib/${PROJECT_NAME}
)


ament_package()
