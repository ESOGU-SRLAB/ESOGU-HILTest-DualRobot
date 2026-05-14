cmake_minimum_required(VERSION 3.10)
project(ros2_conveyorbelt)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(conveyorbelt_msgs REQUIRED)

# Gazebo Harmonic (gz-sim8)
find_package(gz-sim8 REQUIRED)
find_package(gz-math7 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-plugin2 REQUIRED)

add_library(ros2gz_conveyorbelt_system SHARED
  src/ros2gz_conveyorbelt_system.cpp
)
target_include_directories(ros2gz_conveyorbelt_system PUBLIC include)
target_link_libraries(ros2gz_conveyorbelt_system
  gz-sim8::gz-sim8
  gz-math7::gz-math7
  gz-transport13::gz-transport13
  gz-plugin2::gz-plugin2
)
ament_target_dependencies(ros2gz_conveyorbelt_system
  rclcpp
  conveyorbelt_msgs
)

install(TARGETS ros2gz_conveyorbelt_system LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)

ament_package()