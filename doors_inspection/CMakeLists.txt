cmake_minimum_required(VERSION 3.8)
project(doors_inspection)

# HYBRID PACKAGE. This started as pure ament_python (planning + execution launch files
# and the mesh-prep module). The octomap stage needs the real octomap library, and
# there are no usable Python bindings for ColorOcTree serialisation on this machine, so
# the package became ament_cmake + ament_cmake_python: the Python module and the launch
# files install exactly as before, and build_doors_octomap is added alongside them.
# The alternative -- a separate C++ package -- was ruled out deliberately: the doors
# job is meant to live in ONE package.

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)          # std::filesystem
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(octomap REQUIRED)
find_package(Eigen3 REQUIRED)

# ── Python: module + console shims ──────────────────────────────────────────
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/doors_mesh_prep
  scripts/doors_parts_prep
  scripts/doors_mission_node
  DESTINATION lib/${PROJECT_NAME}
)

# ── C++: octomap builder ────────────────────────────────────────────────────
add_executable(build_doors_octomap src/build_doors_octomap.cpp)
target_include_directories(build_doors_octomap PUBLIC
  ${PCL_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(build_doors_octomap
  ${PCL_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
  Eigen3::Eigen
)
ament_target_dependencies(build_doors_octomap rclcpp)

install(TARGETS build_doors_octomap
  DESTINATION lib/${PROJECT_NAME}
)

# ── Data ────────────────────────────────────────────────────────────────────
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
