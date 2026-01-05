cmake_minimum_required(VERSION 3.8)
cmake_policy(SET CMP0074 NEW)

project(sick_visionary_ros VERSION 1.1.2 LANGUAGES CXX)

### FIND PACKAGES ###
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL 1.3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(diagnostic_updater REQUIRED)

### COMPILER FLAGS ###
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 14)   # ROS2 Humble genellikle 14/17 ile uyumlu
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

### INCLUDE DIRECTORIES ###
include_directories(
  include
  sick_visionary_cpp_shared
  ${PCL_INCLUDE_DIRS}
)

### BUILD SHARED LIBRARY ###
add_subdirectory(sick_visionary_cpp_shared)

### EXECUTABLES ###
add_executable(sick_visionary_t_mini_node src/visionary_t_mini.cpp)

target_link_libraries(sick_visionary_t_mini_node
  sick_visionary_cpp_shared
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  Boost::system
  Boost::thread
)

ament_target_dependencies(sick_visionary_t_mini_node
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
  pcl_conversions
  diagnostic_updater
)


add_definitions(${PCL_DEFINITIONS})

### INSTALL ###
install(TARGETS
  sick_visionary_t_mini_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

### EXPORTS ###
ament_package()
