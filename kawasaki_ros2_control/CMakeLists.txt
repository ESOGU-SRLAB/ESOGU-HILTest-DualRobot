cmake_minimum_required(VERSION 3.8)
project(kawasaki_ros2_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(sir_robot_ros_interface REQUIRED)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

# Hardware interface plugin library
add_library(kawasaki_hardware_interface SHARED
  src/kawasaki_hardware_interface.cpp
)

target_include_directories(kawasaki_hardware_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

# The SIR headers copied into include/sir_robot_ros_interface/ use flat
# relative includes (e.g. #include "SIRConnection.h").  Adding the
# subdirectory itself to the include path resolves those flat includes.
target_include_directories(kawasaki_hardware_interface PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/sir_robot_ros_interface>
)

ament_target_dependencies(kawasaki_hardware_interface
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
  sir_robot_ros_interface
  Eigen3
)

target_link_libraries(kawasaki_hardware_interface
  sir_robot_ros_interface::sir_robot_ros_interface_lib
)

pluginlib_export_plugin_description_file(hardware_interface kawasaki_ros2_control.xml)

install(TARGETS kawasaki_hardware_interface
  EXPORT export_kawasaki_hardware_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES kawasaki_ros2_control.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(kawasaki_hardware_interface)
ament_export_targets(export_kawasaki_hardware_interface HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
  sir_robot_ros_interface
  Eigen3
)

ament_package()
