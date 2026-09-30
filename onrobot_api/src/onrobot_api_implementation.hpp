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
  // DIKKAT: bu uyelerin HEPSI acikca initialize edilmeli. Eskiden yalnizca
  // force_ constructor'da atanıyordu; geri kalani belirsiz (cop) degerle
  // basliyordu. read_state() ilk cagrisinda
  //     velocity_ = velocity_ * 0.95 + 0.05 * ((width - position_) / dt)
  // hesabi bu cop degerleri kullanip astronomik bir hiz uretiyordu; bu deger
  // /joint_states uzerinden disari yayiliyordu.
  bool data_available_ = false;
  bool is_running_ = false;
  bool first_read_ = true;
  double command_ = 0.0;
  double position_ = 0.0;
  double velocity_ = 0.0;
  double last_command_ = 0.0;
  double force_ = 10.0;
  double force_measured_ = 0.0;

  bool grip_ = false;
  double width_internal_ = 0.0;
  double width_external_ = 0.0;
  double width_min_internal_ = 0.0;
  double width_max_internal_ = 0.0;
  double width_min_external_ = 0.0;
  double width_max_external_ = 0.0;

  std::string serial_number_;
  std::string firmware_;
};

}  // namespace onrobot

#endif  // ONROBOT_API_IMPLEMENTATION_HPP_
