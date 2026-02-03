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
    ${EXAMPLES_DIR}/sensing_robot_v2.py
    ${EXAMPLES_DIR}/cleaning_mission_runner_v2.py
    ${EXAMPLES_DIR}/red_detection_node_v2.py
    ${EXAMPLES_DIR}/sensing_robot_v3.py
    ${EXAMPLES_DIR}/cleaning_mission_runner_v3.py
    ${EXAMPLES_DIR}/red_detection_node_v3.py
    ${EXAMPLES_DIR}/ex_clear_planning_scene.py
    ${EXAMPLES_DIR}/ex_collision_mesh.py
    ${EXAMPLES_DIR}/ex_collision_primitive.py
    ${EXAMPLES_DIR}/ex_fk.py
    ${EXAMPLES_DIR}/ex_gripper.py
    ${EXAMPLES_DIR}/ex_ik.py
    ${EXAMPLES_DIR}/ex_joint_goal.py
    ${EXAMPLES_DIR}/ex_orientation_path_constraint.py
    ${EXAMPLES_DIR}/ex_pose_goal.py
    ${EXAMPLES_DIR}/combined_ex_pose_goal.py
    ${EXAMPLES_DIR}/combined_joint_goal.py
    ${EXAMPLES_DIR}/sim_robot_goal.py
    ${EXAMPLES_DIR}/sim_robot_joint_goal.py
    ${EXAMPLES_DIR}/dualrobot_sim_robot_goal.py
    ${EXAMPLES_DIR}/ex_servo.py

    DESTINATION lib/${PROJECT_NAME}
)

# Setup the project
ament_package()
