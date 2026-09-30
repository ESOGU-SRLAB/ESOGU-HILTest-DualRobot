#ifndef KAWASAKI_ROS2_CONTROL__KAWASAKI_HARDWARE_INTERFACE_HPP_
#define KAWASAKI_ROS2_CONTROL__KAWASAKI_HARDWARE_INTERFACE_HPP_

#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

// ros2_control
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

// SIR Robot library (use subdirectory-qualified paths so internal
// relative includes like "SIRRobot/SIRConnection.h" resolve correctly)
#include "sir_robot_ros_interface/KawasakiRS005LRobot.h"
#include "sir_robot_ros_interface/SIRLinConnection.h"
#include "sir_robot_ros_interface/SIRLogger.h"
#include "sir_robot_ros_interface/SIRTypeDefs.h"

namespace kawasaki_ros2_control
{

/**
 * @brief ros2_control SystemInterface plugin for the Kawasaki RS005L robot.
 *
 * Communication uses the proprietary SIR TCP/IP protocol implemented in
 * sir_robot_ros_interface. The hardware interface performs:
 *   - read()  : polls the robot joint positions via getJointPose()
 *   - write() : streams a single target waypoint via clear()/add()/move()
 *               whenever the commanded position changes beyond a threshold
 *
 * URDF hardware parameters:
 *   robot_ip   (string, required) – e.g. "192.168.3.7"
 *   robot_port (int,    optional, default 11111)
 *   cmd_threshold_rad (double, optional, default 0.001) – minimum change in
 *               radians to trigger a new motion command to the robot.
 */
class KawasakiHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(KawasakiHardwareInterface)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  static constexpr size_t NUM_JOINTS = 6;

  static double deg2rad(double deg) { return deg * M_PI / 180.0; }
  static double rad2deg(double rad) { return rad * 180.0 / M_PI; }

  // ---- Parameters (read from URDF <hardware> block) ----
  std::string robot_ip_;
  int robot_port_{ 11111 };
  double cmd_threshold_rad_{ 0.001 };  // ~0.057 deg

  // ---- SIR Robot objects ----
  std::unique_ptr<SIRLogger> logger_;
  std::unique_ptr<SIRLinConnection> connection_;
  std::unique_ptr<KawasakiRS005LRobot> robot_;

  // ---- Shared state/command buffers (exported to controller manager) ----
  std::vector<double> hw_positions_;   // radians, updated by read()
  std::vector<double> hw_velocities_;  // radians/s (always 0 – robot doesn't report)
  std::vector<double> hw_commands_;    // radians, written by controller

  // ---- Internal bookkeeping ----
  std::vector<double> prev_commands_;  // last position target sent to robot
  bool activated_{ false };

  // Mutex protecting robot_ access from read() and write() which may be
  // called from different threads by the controller manager.
  mutable std::mutex robot_mutex_;
};

}  // namespace kawasaki_ros2_control

#endif  // KAWASAKI_ROS2_CONTROL__KAWASAKI_HARDWARE_INTERFACE_HPP_
