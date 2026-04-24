// Copyright (c) 2025 Touchlab Limited. All Rights Reserved
// Unauthorized copying or modifications of this file, via any medium is strictly prohibited.

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include "onrobot_robot_driver/hardware_interface.hpp"

namespace onrobot
{

auto logger = rclcpp::get_logger("Onrobot2FG7PositionHardwareInterface");

hardware_interface::CallbackReturn
Onrobot2FG7PositionHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
      return hardware_interface::CallbackReturn::ERROR;
  }

  // init variables for exporting state and command interfaces
  hw_states_position_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_velocity_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_command_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_force_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  rclcpp::NodeOptions options;
  options.arguments({ "--ros-args", "-r", "__node:=onrobot_driver" + info_.name });

  node_ = rclcpp::Node::make_shared("onrobot_driver_", options);

  alpha_ = std::atof(info_.hardware_parameters["position_filtering"].c_str());
  beta_ = std::atof(info_.hardware_parameters["velocity_filtering"].c_str());
  force_ = std::atof(info_.hardware_parameters["force"].c_str());
  use_commanded = info_.hardware_parameters["use_commanded"] == "true";

  node_->declare_parameter("position_filtering", alpha_);
  node_->declare_parameter("velocity_filtering", beta_);
  node_->declare_parameter("force", force_);

  param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(node_);

  auto cb = [this](const rclcpp::Parameter & p) {
    if (p.get_name() == "position_filtering")
      alpha_ = p.as_double();
    if (p.get_name() == "velocity_filtering")
      beta_ = p.as_double();
    if (p.get_name() == "force")
    {
      force_ = p.as_double();
      if(api_ != nullptr) api_->set_force(force_);
    }
  };
  cb_handle_pos = param_subscriber_->add_parameter_callback("position_filtering", cb);
  cb_handle_vel = param_subscriber_->add_parameter_callback("velocity_filtering", cb);
  cb_handle_force = param_subscriber_->add_parameter_callback("force", cb);
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
Onrobot2FG7PositionHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (std::size_t i = 0; i < info_.joints.size(); ++i)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_position_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocity_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_states_force_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
Onrobot2FG7PositionHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (std::size_t i = 0; i < info_.joints.size(); ++i)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn
Onrobot2FG7PositionHardwareInterface::on_activate(
                                        const rclcpp_lifecycle::State & /*previous_state*/)
{
  api_ = std::unique_ptr<Robot2FG7>(new Robot2FG7());
  int port = std::stoi(info_.hardware_parameters["port"]);
  std::string address = info_.hardware_parameters["address"];
  try
  {
    api_->init(address, port);
  }
  catch(...)
  {
    RCLCPP_FATAL_STREAM(logger, "Can't connect to port " << port);
    return hardware_interface::CallbackReturn::ERROR;
  }

  position_max_ = api_->get_max_position() * 0.5;
  position_min_ = api_->get_min_position() * 0.5;

  RCLCPP_INFO_STREAM(logger, "Onrobot 2FG7connected.");

  RCLCPP_INFO_STREAM(logger, "firmware '" << api_->get_firmware() <<
                              "', serial '" << api_->get_serial() << "'");

  api_->set_force(force_);

  for (std::size_t i = 0; i < info_.joints.size(); ++i)
  {
    if (std::isnan(hw_states_position_[i]))
    {
      hw_states_position_[i] = position_max_;
    }
    if (std::isnan(hw_states_velocity_[i]))
    {
      hw_states_velocity_[i] = 0.0;
    }

    if (std::isnan(hw_commands_[i]))
    {
      hw_commands_[0] = position_max_;
    }
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
Onrobot2FG7PositionHardwareInterface::on_deactivate(const rclcpp_lifecycle::State & )
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type Onrobot2FG7PositionHardwareInterface::read(const rclcpp::Time& time,
                                                                  const rclcpp::Duration& period)
{
  if (rclcpp::ok())
  {
    rclcpp::spin_some(node_);
  }

  double measured_position;
  double measured_velocity;
  double measured_force;
  double last_command;
  api_->get_state(measured_position, measured_velocity,
                  measured_force, last_command);
  // Use commanded position instead of measured if requested
  measured_position = use_commanded ? last_command * 0.5 : measured_position * 0.5;
  measured_velocity *= 0.5;

  // low-pass filtering
  hw_states_position_[0] = hw_states_position_[0] * (1.0 - alpha_) + measured_position * alpha_;
  hw_states_velocity_[0] = hw_states_velocity_[0] * (1.0 - beta_) + measured_velocity * beta_;

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type Onrobot2FG7PositionHardwareInterface::write(
  const rclcpp::Time& time, const rclcpp::Duration& period)
{
  double cmd = std::max(position_min_, std::min(position_max_, hw_commands_[0]));
  api_->set_position(cmd * 2.0);
  return hardware_interface::return_type::OK;
}

}  // namespace onrobot

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(onrobot::Onrobot2FG7PositionHardwareInterface,
  hardware_interface::SystemInterface)
