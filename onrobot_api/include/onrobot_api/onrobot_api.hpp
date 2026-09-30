// Copyright (c) 2025 Touchlab Limited. All Rights Reserved
// Unauthorized copying or modifications of this file, via any medium is strictly prohibited.

#ifndef ONROBOT_API__ONROBOT_API_HPP_
#define ONROBOT_API__ONROBOT_API_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <vector>

namespace onrobot
{

/**
 * @class Robot2FG7
 * @brief Class for interfacing with the OnRobot 2FG7 gripper.
 *
 * This class provides methods to initialize, control, and retrieve data from the OnRobot 2FG7 gripper.
 */
class Robot2FG7
{
public:
  /**
   * @brief Construct a new Robot2FG7 object.
   */
  Robot2FG7();

  /**
   * @brief Destroy the Robot2FG7 object.
   */
  virtual ~Robot2FG7();

  /**
   * @brief Initialize the gripper.
   *
   * @param address The IP address of the gripper.
   * @param port The port number for communication (default is 502).
   */
  void init(const std::string& address, int port = 502);

  /**
   * @brief Get the current state of the gripper.
   *
   * @param measured_position The measured position of the gripper (m).
   * @param measured_velocity The measured velocity of the gripper (m/s).
   * @param measured_force The measured force applied by the gripper (N).
   * @param target_position The target position of the gripper (m).
   * @param timeout Timeout duration (milliseconds) for the operation (default is infinite).
   * @return true if the state was successfully retrieved, false otherwise.
   */
  bool get_state(double& measured_position,
                 double& measured_velocity,
                 double& measured_force,
                 double& target_position,
                 std::chrono::duration<long double> timeout = std::chrono::seconds(-1));

  /**
   * @brief Set the target position of the gripper.
   *
   * @param position The target position to set (m).
   */
  void set_position(double position);

  /**
   * @brief Set the target force of the gripper.
   *
   * @param force The target force to set (N).
   */
  void set_force(double force);

  /**
   * @brief Get the minimum position limit of the gripper.
   *
   * @return The minimum position limit (m).
   */
  double get_min_position();

  /**
   * @brief Get the maximum position limit of the gripper.
   *
   * @return The maximum position limit (m).
   */
  double get_max_position();

  /**
   * @brief Get the minimum force limit of the gripper.
   *
   * @return The minimum force limit (m).
   */
  double get_min_force();

  /**
   * @brief Get the maximum force limit of the gripper.
   *
   * @return The maximum force limit (N).
   */
  double get_max_force();

  /**
   * @brief Get the serial number of the gripper.
   *
   * @return The serial number as a string.
   */
  std::string get_serial();

  /**
   * @brief Get the firmware version of the gripper.
   *
   * @return The firmware version as a string.
   */
  std::string get_firmware();

private:
  /**
   * @class Implementation
   * @brief Private implementation class for the Robot2FG7.
   */
  class Implementation;

  /// Unique pointer to the private implementation.
  std::unique_ptr<Implementation> impl_;
};

}  // namespace onrobot

#endif  // ONROBOT_API__ONROBOT_API_HPP_
