// Copyright (c) 2025 Touchlab Limited. All Rights Reserved
// Unauthorized copying or modifications of this file, via any medium is strictly prohibited.

#include <chrono>
#include <cmath>
#include <condition_variable>
#include <exception>
#include <fstream>
#include <memory>
#include <mutex>
#include <iomanip>
#include <sstream>
#include <thread>
#include <vector>

#include "onrobot_api/onrobot_api.hpp"
#include "onrobot_api_implementation.hpp"
#include "modbus/modbus.h"
#include "utils.hpp"

using namespace std::chrono_literals;

namespace onrobot
{

Robot2FG7::Implementation::Implementation() : force_(10.0)
{
}

Robot2FG7::Implementation::~Implementation()
{
  is_running_ = false;
  if (main_thread_ && main_thread_->joinable()) {
    main_thread_->join();
    main_thread_.reset();
  }

  if (modbus_)
  {
    stop();
    modbus_close(modbus_.get());
  }
  modbus_ = nullptr;
}

void Robot2FG7::Implementation::init(const std::string& address, int port)
{
  modbus_ = std::shared_ptr<modbus_t>(modbus_new_tcp(address.c_str(), port), modbus_free);

  if (modbus_ == nullptr) {
    THROW_SIMPLE("Can't allocate modbus object");
  }

  modbus_set_response_timeout(modbus_.get(), 0, 500000);
  modbus_set_indication_timeout(modbus_.get(), 0, 500000);
  modbus_set_byte_timeout(modbus_.get(), 0, 500000);

  modbus_set_slave(modbus_.get(), 66);

  if (modbus_connect(modbus_.get()) == -1) {
      THROW_SIMPLE("Failed to connect: " << modbus_strerror(errno));
  }

  reset_offset();

  read_serial();

  read_firmware();

  // Initial read
  uint16_t read_buffer[8];
  int ret = modbus_read_registers(modbus_.get(), 256, 8, read_buffer);
  if (ret <= 0)
    THROW_SIMPLE("Can't communicate with robot " << address << ":" << port <<
                 " " << modbus_strerror(errno));
  read_state(read_buffer, 1.0);

  // Mevcut ölçülen pozisyonu başlangıç komutu olarak kullan (gripper'ı hareket ettirme)
  command_ = position_;
  last_command_ = command_;

  is_running_ = true;
  main_thread_.reset(new std::thread([&]() { main_loop(); }));
}

void Robot2FG7::Implementation::read_state(const uint16_t* read_buffer, double dt)
{
  std::unique_lock<std::mutex> get_guard(get_mutex_);
  grip_ = read_buffer[0] == 2;
  width_internal_ = static_cast<double>(read_buffer[1]) * 1e-4;
  width_external_ = static_cast<double>(read_buffer[2]) * 1e-4;
  width_min_internal_ = static_cast<double>(read_buffer[3]) * 1e-4;
  width_max_internal_ = static_cast<double>(read_buffer[4]) * 1e-4;
  width_min_external_ = static_cast<double>(read_buffer[5]) * 1e-4;
  width_max_external_ = static_cast<double>(read_buffer[6]) * 1e-4;
  force_measured_ = static_cast<double>(static_cast<int16_t>(read_buffer[7]));
  double alpha = 0.05;
  velocity_ = velocity_ * (1.0 - alpha) + alpha * ((width_internal_ - position_) / dt);
  position_ = width_internal_;
  data_available_ = true;
  data_condition_.notify_all();
  get_guard.unlock();
}

void Robot2FG7::Implementation::reset_offset()
{
  if (modbus_write_register(modbus_.get(), 1027, 0) != 1)
    THROW_SIMPLE("Can't write registers: " << modbus_strerror(errno));
  if (modbus_write_register(modbus_.get(), 3, 0) != 1)
    THROW_SIMPLE("Can't write registers: " << modbus_strerror(errno));
  if (modbus_write_register(modbus_.get(), 3, 103) != 1)
    THROW_SIMPLE("Can't write registers: " << modbus_strerror(errno));
  if (modbus_write_register(modbus_.get(), 3, 0) != 1)
    THROW_SIMPLE("Can't write registers: " << modbus_strerror(errno));
}

void Robot2FG7::Implementation::read_serial()
{
  uint16_t read_buffer[5];
  modbus_read_registers(modbus_.get(), 1545, 5, read_buffer);
  serial_number_ = "          ";
  for (int i = 0; i < 5; ++i)
  {
    serial_number_[i * 2 + 0] = static_cast<char>(read_buffer[i] >> 8);
    serial_number_[i * 2 + 1] = static_cast<char>(read_buffer[i] & 0xff);
  }
}

void Robot2FG7::Implementation::read_firmware()
{
  uint16_t read_buffer[6];
  modbus_read_registers(modbus_.get(), 1540, 6, read_buffer);
  std::stringstream ss;
  ss << std::to_string(read_buffer[0] >> 8) << "." <<
        std::to_string(read_buffer[0] & 0xff) << "." <<
        std::to_string(read_buffer[1] & 0xff) << "#";
  for (int i = 2; i < 4; ++i)
  {
    ss << std::uppercase << std::hex << (read_buffer[i] >> 8);
    ss << std::uppercase << std::hex << (read_buffer[i] & 0xff);
  }

  firmware_ = ss.str();
}

void Robot2FG7::Implementation::stop()
{
  modbus_write_register(modbus_.get(), 3, 0);
}

bool Robot2FG7::Implementation::get_state(double& measured_position,
                double& measured_velocity,
                double& measured_force,
                double& target_position,
                std::chrono::duration<long double> timeout)
{
  std::unique_lock<std::mutex> guard(get_mutex_);
  if (timeout > 0s) {
    if (data_condition_.wait_until(
          guard, std::chrono::system_clock::now() + timeout,
          [&] { return data_available_ == true; })) {
      measured_position = position_;
      measured_velocity = velocity_;
      measured_force = force_measured_;
      target_position = last_command_;
      data_available_ = false;
      guard.unlock();
      return true;
    } else {
      guard.unlock();
      return false;
    }
  } else {
    measured_position = position_;
    measured_velocity = velocity_;
    measured_force = force_measured_;
    target_position = last_command_;
    guard.unlock();
    return true;
  }
}

void Robot2FG7::Implementation::set_position(double position)
{
  if(position < width_min_internal_ || position > width_max_internal_)
    THROW_SIMPLE("Target position outside bounds (" << width_min_internal_ << "m, "
      << width_max_internal_ << "m) " << " got " << position << "m");
  std::unique_lock<std::mutex> set_guard(set_mutex_);
  command_ = position;
  set_guard.unlock();
}

void Robot2FG7::Implementation::set_force(double force)
{
  if(force < 20 || force > 140)
    THROW_SIMPLE("Force outside bounds (20N, 140N)");
  std::unique_lock<std::mutex> set_guard(set_mutex_);
  force_ = force;
  set_guard.unlock();
}

std::string Robot2FG7::Implementation::get_serial()
{
  return serial_number_;
}

std::string Robot2FG7::Implementation::get_firmware()
{
  return firmware_;
}

#define NUM_WRITE 4
#define NUM_READ 8

void Robot2FG7::Implementation::main_loop()
{
  uint16_t write_buffer[NUM_WRITE];
  uint16_t read_buffer[NUM_READ];
  auto t0 = std::chrono::steady_clock::now();
  while (is_running_)
  {
    try
    {
      std::unique_lock<std::mutex> set_guard(set_mutex_);
      auto t1 = std::chrono::steady_clock::now();
      double dt =  static_cast<std::chrono::duration<double>>(t1 - t0).count();
      t0 = t1;
      double velocity = (command_ - last_command_) / dt;
      last_command_ = command_;
      write_buffer[0] = static_cast<uint16_t>(std::fabs(command_) * 1e4);  // Position
      write_buffer[1] = static_cast<uint16_t>(std::fabs(force_));  // Force
      write_buffer[2] = static_cast<uint16_t>(10 + std::fabs(velocity) * 1e3);  // Velocity
      write_buffer[3] = 1;  // Enable control
      set_guard.unlock();

      int ret = modbus_write_and_read_registers(modbus_.get(),
        0, NUM_WRITE, write_buffer,
        256, NUM_READ, read_buffer);

      read_state(read_buffer, dt);
    }
    catch (const std::exception & e)
    {
      WARNING(e.what());
    }
  }
  is_running_ = false;
}

}  // namespace onrobot
