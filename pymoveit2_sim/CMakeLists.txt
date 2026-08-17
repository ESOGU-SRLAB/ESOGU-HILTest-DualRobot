cmake_minimum_required(VERSION 3.5)
project(pymoveit2_sim)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install Python package
ament_python_install_package(pymoveit2_sim)

# Install examples
set(EXAMPLES_DIR examples)
install(PROGRAMS
    ${EXAMPLES_DIR}/sensing_robot.py
    ${EXAMPLES_DIR}/cleaning_mission_runner.py
    ${EXAMPLES_DIR}/red_detection_node.py
    ${EXAMPLES_DIR}/sim_robot_goal.py
    ${EXAMPLES_DIR}/sim_robot_joint_goal.py
    ${EXAMPLES_DIR}/sim_gripper_goal.py


    DESTINATION lib/${PROJECT_NAME}
)

# Setup the project
ament_package()
