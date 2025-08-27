cmake_minimum_required(VERSION 3.8)
project(my_robot_cell_control)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config urdf rviz scripts
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/real_to_gazebo.py
DESTINATION lib/${PROJECT_NAME}
)


ament_package()
