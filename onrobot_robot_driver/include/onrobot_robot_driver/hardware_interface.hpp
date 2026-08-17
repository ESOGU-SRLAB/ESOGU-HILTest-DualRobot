// Copyright (c) 2025 Touchlab Limited. All Rights Reserved
// Unauthorized copying or modifications of this file, via any medium is strictly prohibited.

#ifndef ONROBOT_ROBOT_DRIVER__HARDWARE_INTERFACE_HPP_
#define ONROBOT_ROBOT_DRIVER__HARDWARE_INTERFACE_HPP_

#include <memory>
#include <vector>

#include <hardware_interface/system_interface.hpp>

#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"

#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/node.hpp"

#include "onrobot_api/onrobot_api.hpp"

namespace onrobot
{

class Onrobot2FG7PositionHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(Onrobot2FG7PositionHardwareInterface)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & system_info) final;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() final;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() final;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) final;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) final;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) final;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) final;

protected:
  std::unique_ptr<Robot2FG7> api_;

  std::vector<double> joints_offset_;

  // exposed to hw_interface
  std::vector<double> hw_commands_;
  std::vector<double> hw_states_position_;
  std::vector<double> hw_states_velocity_;
  std::vector<double> hw_states_command_;
  std::vector<double> hw_states_force_;
  double position_min_;
  double position_max_;

  // filter parameters
  double alpha_;
  double beta_;
  double force_;
  bool use_commanded;
  bool command_initialized_;  // İlk dış komut gelene kadar write() devre dışı
  std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;

  // callback handles for dynamic configuration
  std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_handle_pos;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_handle_vel;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_handle_force;
  rclcpp::Node::SharedPtr node_;
};

}   // namespace onrobot

#endif   // ONROBOT_ROBOT_DRIVER__HARDWARE_INTERFACE_HPP_
