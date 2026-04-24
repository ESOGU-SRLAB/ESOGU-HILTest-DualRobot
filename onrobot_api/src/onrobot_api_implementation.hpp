// Copyright (c) 2025 Touchlab Limited. All Rights Reserved
// Unauthorized copying or modifications of this file, via any medium is strictly prohibited.

#ifndef ONROBOT_API_IMPLEMENTATION_HPP_
#define ONROBOT_API_IMPLEMENTATION_HPP_

#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "onrobot_api/onrobot_api.hpp"
#include "modbus/modbus.h"

constexpr uint32_t ARDUINO_BAUD_RATE = 115200U;

namespace onrobot
{

class Robot2FG7::Implementation
{
public:
  Implementation();
  virtual ~Implementation();
  void init(const std::string& address, int port);
  bool get_state(double& measured_position,
                 double& measured_velocity,
                 double& measured_force,
                 double& target_position,
                 std::chrono::duration<long double> timeout = std::chrono::seconds(-1));
  void set_position(double position);
  void set_force(double force);
  std::string get_serial();
  std::string get_firmware();

  void main_loop();

  void read_state(const uint16_t* read_buffer, double dt);
  void reset_offset();
  void read_serial();
  void read_firmware();
  void stop();

  std::shared_ptr<modbus_t> modbus_;

  std::unique_ptr<std::thread> main_thread_;
  std::mutex get_mutex_;
  std::mutex set_mutex_;

  std::condition_variable data_condition_;
  bool data_available_;
  bool is_running_;
  double command_;
  double position_;
  double velocity_;
  double last_command_;
  double force_;
  double force_measured_;

  bool grip_;
  double width_internal_;
  double width_external_;
  double width_min_internal_;
  double width_max_internal_;
  double width_min_external_;
  double width_max_external_;

  std::string serial_number_;
  std::string firmware_;
};

}  // namespace onrobot

#endif  // ONROBOT_API_IMPLEMENTATION_HPP_
