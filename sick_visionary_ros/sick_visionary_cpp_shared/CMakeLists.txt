cmake_minimum_required(VERSION 3.16)
project(sick_visionary_cpp_shared
        VERSION 1.1.2
        DESCRIPTION "Open source drivers for the SICK Visionary-S and Visionary-T Mini cameras"
        LANGUAGES CXX)

# ament_cmake build tool
find_package(ament_cmake REQUIRED)

# ROS2 dependencies
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(Boost REQUIRED)

# Threads
find_package(Threads REQUIRED)

### Options
option(BUILD_SHARED_LIBS "Build using shared libraries" OFF)
option(VISIONARY_SHARED_ENABLE_AUTOIP "Enables SOPAS Auto-IP device scan code (needs boost ptree)" ON)
option(VISIONARY_SHARED_ENABLE_UNITTESTS "Enable google-test unit tests" OFF)
option(VISIONARY_SHARED_USE_BUNDLED_BOOST "Use bundled Boost" ON)

# Compiler flags
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Source files
set(VISIONARY_SHARED_SRCS
    src/UdpSocket.cpp src/TcpSocket.cpp
    src/CoLaBProtocolHandler.cpp src/CoLa2ProtocolHandler.cpp
    src/AuthenticationLegacy.cpp src/AuthenticationSecure.cpp
    src/CoLaParameterReader.cpp src/CoLaParameterWriter.cpp
    src/CoLaCommand.cpp src/CoLaParameterReader.cpp src/CoLaParameterWriter.cpp
    3pp/md5/MD5.cpp 3pp/sha256/SHA256.cpp
    src/VisionaryControl.cpp src/ControlSession.cpp
    src/VisionaryDataStream.cpp src/FrameGrabberBase.cpp
    src/VisionaryData.cpp src/VisionarySData.cpp src/VisionaryTData.cpp src/VisionaryTMiniData.cpp
    src/PointCloudPlyWriter.cpp
)

if(VISIONARY_SHARED_ENABLE_AUTOIP)
    list(APPEND VISIONARY_SHARED_SRCS src/VisionaryAutoIPScan.cpp)
endif()

# Public headers
set(VISIONARY_SHARED_PUBLIC_HEADERS
    src/UdpSocket.h src/TcpSocket.h src/ITransport.h
    src/CoLaBProtocolHandler.h src/CoLa2ProtocolHandler.h src/IProtocolHandler.h
    src/AuthenticationLegacy.h src/AuthenticationSecure.h src/IAuthentication.h
    src/CoLaParameterReader.h src/CoLaParameterWriter.h
    src/CoLaCommand.h src/CoLaCommandType.h src/CoLaError.h
    src/ControlSession.h src/VisionaryControl.h src/FrameGrabberBase.h src/FrameGrabber.h
    src/VisionaryDataStream.h
    src/VisionaryData.h src/VisionarySData.h src/VisionaryTData.h src/VisionaryTMiniData.h
    src/PointCloudPlyWriter.h src/PointXYZ.h
)

# Create library
add_library(${PROJECT_NAME} ${VISIONARY_SHARED_SRCS})
target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        3pp/md5 3pp/sha256
)
target_link_libraries(${PROJECT_NAME} Threads::Threads)
ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    sensor_msgs
    image_transport
    cv_bridge
    diagnostic_updater
    pcl_conversions
    pcl_ros
)

# Compiler settings
set_target_properties(${PROJECT_NAME} PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED YES
    CXX_EXTENSIONS OFF
)

# Install
install(TARGETS ${PROJECT_NAME}
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    PUBLIC_HEADER DESTINATION include
)

install(DIRECTORY src/
    DESTINATION include/${PROJECT_NAME}
    FILES_MATCHING PATTERN "*.h"
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
    rclcpp
    sensor_msgs
    image_transport
    cv_bridge
    diagnostic_updater
    pcl_conversions
    pcl_ros
)
ament_package()
