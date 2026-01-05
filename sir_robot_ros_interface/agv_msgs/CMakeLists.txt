cmake_minimum_required(VERSION 3.8)
project(agv_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BatteryData.msg"
  "msg/ControlData.msg"
  "msg/PowerData.msg"
  "msg/RobotLow.msg"
  "msg/TaskCom.msg"
  "msg/TaskRequest.msg"
  "msg/TaskStatus.msg"
  "msg/TaskPlan.msg"
  "msg/YskParam.msg"
  "msg/Route.msg"
  "msg/Signal.msg"
  "msg/Taskf.msg"
  "srv/FabMsgs.srv"
  DEPENDENCIES std_msgs
)

ament_package()
