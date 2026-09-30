// kawasaki_hardware_interface.cpp
// ROS 2 Control hardware interface for Kawasaki RS005L
// using the SIR TCP/IP communication library.

#include "kawasaki_ros2_control/kawasaki_hardware_interface.hpp"

#include <algorithm>
#include <stdexcept>

// Ensure internal SIR headers (SIRRobot/ subdir) are reachable when
// the compiler processes includes triggered by KawasakiRS005LRobot.h.

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace kawasaki_ros2_control
{

// ---------------------------------------------------------------------------
// on_init – called once when the plugin is loaded; parse URDF parameters
// ---------------------------------------------------------------------------
hardware_interface::CallbackReturn KawasakiHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // ---- Required parameter: robot_ip ----
  if (info_.hardware_parameters.count("robot_ip") == 0) {
    RCLCPP_FATAL(
      rclcpp::get_logger("KawasakiHardwareInterface"),
      "Parameter 'robot_ip' not found in URDF <hardware> block.");
    return hardware_interface::CallbackReturn::ERROR;
  }
  robot_ip_ = info_.hardware_parameters.at("robot_ip");

  // ---- Optional: robot_port ----
  if (info_.hardware_parameters.count("robot_port") > 0) {
    robot_port_ = std::stoi(info_.hardware_parameters.at("robot_port"));
  }

  // ---- Optional: cmd_threshold_rad ----
  if (info_.hardware_parameters.count("cmd_threshold_rad") > 0) {
    cmd_threshold_rad_ = std::stod(info_.hardware_parameters.at("cmd_threshold_rad"));
  }

  // ---- Validate joint count ----
  if (info_.joints.size() != NUM_JOINTS) {
    RCLCPP_FATAL(
      rclcpp::get_logger("KawasakiHardwareInterface"),
      "Expected %zu joints, got %zu. Check your URDF.", NUM_JOINTS, info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // ---- Validate interfaces per joint ----
  for (const auto & joint : info_.joints) {
    // Must have exactly one command interface: position
    if (joint.command_interfaces.size() != 1 ||
      joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("KawasakiHardwareInterface"),
        "Joint '%s' must have exactly one 'position' command interface.",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    // Must have position (and optionally velocity) state interfaces
    bool has_position = false;
    for (const auto & si : joint.state_interfaces) {
      if (si.name == hardware_interface::HW_IF_POSITION) {
        has_position = true;
      }
    }
    if (!has_position) {
      RCLCPP_FATAL(
        rclcpp::get_logger("KawasakiHardwareInterface"),
        "Joint '%s' must have a 'position' state interface.", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // ---- Resize buffers ----
  hw_positions_.assign(NUM_JOINTS, 0.0);
  hw_velocities_.assign(NUM_JOINTS, 0.0);
  hw_commands_.assign(NUM_JOINTS, 0.0);
  prev_commands_.assign(NUM_JOINTS, std::numeric_limits<double>::quiet_NaN());

  RCLCPP_INFO(
    rclcpp::get_logger("KawasakiHardwareInterface"),
    "Initialized – robot_ip=%s  port=%d  cmd_threshold=%.4f rad",
    robot_ip_.c_str(), robot_port_, cmd_threshold_rad_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// on_configure – create robot objects (no network connection yet)
// ---------------------------------------------------------------------------
hardware_interface::CallbackReturn KawasakiHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("KawasakiHardwareInterface"), "Configuring...");

  logger_ = std::make_unique<SIRLogger>(
    "kawasaki_hw_interface.log", SIRLOGLEVEL::LOGLEVEL_DEBUG);

  connection_ = std::make_unique<SIRLinConnection>(
    logger_.get(), robot_ip_.c_str(), robot_port_);

  // MPT_JOINT: commands are in joint space (degrees)
  // MT_P2P   : point-to-point motion between waypoints
  robot_ = std::make_unique<KawasakiRS005LRobot>(
    connection_.get(), logger_.get(),
    nullptr,     // no SIRPoseDB
    MPT_JOINT,
    MT_P2P);

  robot_->setWaitForCommand(2000);

  RCLCPP_INFO(rclcpp::get_logger("KawasakiHardwareInterface"), "Configured.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// on_cleanup – destroy robot objects
// ---------------------------------------------------------------------------
hardware_interface::CallbackReturn KawasakiHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  robot_.reset();
  connection_.reset();
  logger_.reset();
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// export_state_interfaces
// ---------------------------------------------------------------------------
std::vector<hardware_interface::StateInterface>
KawasakiHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  interfaces.reserve(NUM_JOINTS * 2);

  for (size_t i = 0; i < NUM_JOINTS; ++i) {
    interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
    interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]);
  }
  return interfaces;
}

// ---------------------------------------------------------------------------
// export_command_interfaces
// ---------------------------------------------------------------------------
std::vector<hardware_interface::CommandInterface>
KawasakiHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.reserve(NUM_JOINTS);

  for (size_t i = 0; i < NUM_JOINTS; ++i) {
    interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]);
  }
  return interfaces;
}

// ---------------------------------------------------------------------------
// on_activate – connect to robot, read initial positions
// ---------------------------------------------------------------------------
hardware_interface::CallbackReturn KawasakiHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    rclcpp::get_logger("KawasakiHardwareInterface"),
    "Connecting to Kawasaki robot at %s:%d ...", robot_ip_.c_str(), robot_port_);

  if (!robot_->Connect()) {
    RCLCPP_ERROR(
      rclcpp::get_logger("KawasakiHardwareInterface"),
      "Failed to connect to robot.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Non-blocking socket – read/write calls return immediately
  connection_->setBlockingMode(0);

  // Read initial joint positions so commands start at the current pose
  SIRMatrix joint_pose(NUM_JOINTS, 1);
  if (!robot_->getJointPose(&joint_pose)) {
    RCLCPP_ERROR(
      rclcpp::get_logger("KawasakiHardwareInterface"),
      "Failed to read initial joint positions after connect.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (size_t i = 0; i < NUM_JOINTS; ++i) {
    hw_positions_[i] = deg2rad(joint_pose(static_cast<int>(i), 0));
    hw_commands_[i]  = hw_positions_[i];  // hold current pose on first activate
    prev_commands_[i] = hw_commands_[i];
  }

  activated_ = true;

  RCLCPP_INFO(
    rclcpp::get_logger("KawasakiHardwareInterface"),
    "Robot connected. Initial positions (rad): "
    "[%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
    hw_positions_[0], hw_positions_[1], hw_positions_[2],
    hw_positions_[3], hw_positions_[4], hw_positions_[5]);

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// on_deactivate – gracefully close the connection
// ---------------------------------------------------------------------------
hardware_interface::CallbackReturn KawasakiHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("KawasakiHardwareInterface"), "Deactivating...");

  activated_ = false;

  if (robot_) {
    std::lock_guard<std::mutex> lock(robot_mutex_);
    robot_->close();
  }

  RCLCPP_INFO(rclcpp::get_logger("KawasakiHardwareInterface"), "Deactivated.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// read – poll joint positions from the robot
// ---------------------------------------------------------------------------
hardware_interface::return_type KawasakiHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!activated_) {
    return hardware_interface::return_type::OK;
  }

  std::lock_guard<std::mutex> lock(robot_mutex_);

  SIRMatrix joint_pose(NUM_JOINTS, 1);
  if (!robot_->getJointPose(&joint_pose)) {
    // Keep last known values; warn at most every 5 s
    RCLCPP_WARN_THROTTLE(
      rclcpp::get_logger("KawasakiHardwareInterface"),
      *rclcpp::Clock::make_shared(),
      5000,
      "getJointPose() failed – using last known positions.");
    return hardware_interface::return_type::OK;
  }

  for (size_t i = 0; i < NUM_JOINTS; ++i) {
    hw_positions_[i] = deg2rad(joint_pose(static_cast<int>(i), 0));
    hw_velocities_[i] = 0.0;  // robot does not report joint velocities
  }

  return hardware_interface::return_type::OK;
}

// ---------------------------------------------------------------------------
// write – send position target to the robot when command changes
//
// Strategy (single-point streaming):
//   If any joint command changed by more than cmd_threshold_rad_ AND the robot
//   has finished its previous motion (RS_STOP), we:
//     1. clear() the waypoint queue
//     2. add()   the new target as a single waypoint
//     3. move()  to execute
//
// This is safe even in non-blocking mode: move() returns immediately and the
// robot starts moving toward the target.  If a new command arrives while the
// robot is still in RS_MOVE we skip it; the next control cycle will retry.
// ---------------------------------------------------------------------------
hardware_interface::return_type KawasakiHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!activated_) {
    return hardware_interface::return_type::OK;
  }

  // --- Check whether any commanded position changed enough ---
  bool commands_changed = false;
  for (size_t i = 0; i < NUM_JOINTS; ++i) {
    if (std::isnan(prev_commands_[i]) ||
      std::abs(hw_commands_[i] - prev_commands_[i]) > cmd_threshold_rad_)
    {
      commands_changed = true;
      break;
    }
  }

  if (!commands_changed) {
    return hardware_interface::return_type::OK;
  }

  std::lock_guard<std::mutex> lock(robot_mutex_);

  // --- Only send when robot is idle (not mid-motion) ---
  ROBOTSTATUS status = robot_->getStatus();
  if (status == RS_MOVE) {
    // Robot is still executing previous command; skip this cycle.
    // The controller will continue issuing updated setpoints.
    return hardware_interface::return_type::OK;
  }

  // --- Build target in degrees (Kawasaki API uses degrees) ---
  SIRMatrix target(NUM_JOINTS, 1);
  for (size_t i = 0; i < NUM_JOINTS; ++i) {
    target(static_cast<int>(i), 0) = rad2deg(hw_commands_[i]);
  }

  // --- Clear waypoint queue, add single target, execute ---
  robot_->clear();

  int packet_id = robot_->add(target);
  if (packet_id < 0) {
    RCLCPP_WARN_THROTTLE(
      rclcpp::get_logger("KawasakiHardwareInterface"),
      *rclcpp::Clock::make_shared(),
      5000,
      "robot.add() returned negative packet ID – skipping write.");
    return hardware_interface::return_type::OK;
  }

  if (!robot_->move()) {
    RCLCPP_WARN_THROTTLE(
      rclcpp::get_logger("KawasakiHardwareInterface"),
      *rclcpp::Clock::make_shared(),
      5000,
      "robot.move() failed – skipping write.");
    return hardware_interface::return_type::OK;
  }

  prev_commands_ = hw_commands_;

  return hardware_interface::return_type::OK;
}

}  // namespace kawasaki_ros2_control

// Register as a pluginlib plugin
PLUGINLIB_EXPORT_CLASS(
  kawasaki_ros2_control::KawasakiHardwareInterface,
  hardware_interface::SystemInterface)
