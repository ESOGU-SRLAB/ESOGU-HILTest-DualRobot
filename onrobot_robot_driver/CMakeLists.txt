cmake_minimum_required(VERSION 3.8)
project(onrobot_robot_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  message("${PROJECT_NAME}: You did not request a specific build type: selecting 'Release'.")
  set(CMAKE_BUILD_TYPE Release)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(controller_manager REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(onrobot_api REQUIRED)
find_package(onrobot_2fg7_description REQUIRED)


include_directories(include)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_manager
  controller_manager_msgs
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

add_library(onrobot_robot_driver_plugin
  SHARED
  src/hardware_interface.cpp
)
ament_target_dependencies(
  onrobot_robot_driver_plugin
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

target_link_libraries(onrobot_robot_driver_plugin onrobot_api)
target_include_directories(
  onrobot_robot_driver_plugin
  PRIVATE
  include
)

pluginlib_export_plugin_description_file(hardware_interface hardware_interface_plugin.xml)

install(
  TARGETS onrobot_robot_driver_plugin
  DESTINATION lib
)
install(
  DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(
  include
)
ament_export_libraries(
  onrobot_robot_driver_plugin
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

ament_package()