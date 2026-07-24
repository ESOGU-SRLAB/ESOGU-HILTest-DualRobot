cmake_minimum_required(VERSION 3.5)
project(pymoveit2_real)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install Python package
ament_python_install_package(pymoveit2_real)

# Install examples
set(EXAMPLES_DIR examples)
install(PROGRAMS
    ${EXAMPLES_DIR}/combined_xyz_goal.py
    ${EXAMPLES_DIR}/combined_joint_goal.py
    ${EXAMPLES_DIR}/real_robot_goal.py
    ${EXAMPLES_DIR}/real_robot_joint_goal.py
    ${EXAMPLES_DIR}/ur_inspection_scenario.py
    ${EXAMPLES_DIR}/real_dual_executor.py
    ${EXAMPLES_DIR}/real_robot_gripper_goal.py
    ${EXAMPLES_DIR}/human_robot_collaboration_scenario.py
    ${EXAMPLES_DIR}/pick_and_place_scenario.py
    ${EXAMPLES_DIR}/sensing_robot.py
    ${EXAMPLES_DIR}/cleaning_mission_runner.py
    ${EXAMPLES_DIR}/red_detection_node.py
    DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
)

# Setup the project
ament_package()
