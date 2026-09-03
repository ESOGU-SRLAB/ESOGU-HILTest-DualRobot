// Copyright (c) 2025 Touchlab Limited. All Rights Reserved
// Unauthorized copying or modifications of this file, via any medium is strictly prohibited.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <exception>
#include <fstream>
#include <memory>
#include <mutex>
#include <iomanip>
#include <limits>
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

Robot2FG7::Implementation::Implementation()
{
  // Tum uyeler artik header'da in-class initializer ile sifirlaniyor.
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

  if (first_read_)
  {
    // Ilk okumada gecmis yok: turev alinacak onceki konum da yok. Hizi 0
    // kabul et, konumu olculen genislikle tohumla. Aksi halde ilk ornekte
    // (width - position_) farki tamamen anlamsiz olur.
    position_ = width_internal_;
    velocity_ = 0.0;
    first_read_ = false;
  }
  else
  {
    // dt'yi tabanla: cok kucuk dt turevi patlatiyor (eskiden main_loop hiz
    // siniri olmadigi icin dt mikrosaniye mertebesine inebiliyordu).
    const double safe_dt = std::max(dt, 1e-3);
    const double alpha = 0.05;
    velocity_ = velocity_ * (1.0 - alpha) + alpha * ((width_internal_ - position_) / safe_dt);
    position_ = width_internal_;
  }

  // Sayisal bir bozulma disari sizmasin: /joint_states'e giden hiz, asagi
  // akistaki her tuketiciye (ornegin real_to_sim_bridge) aynen iletiliyor.
  if (!std::isfinite(velocity_))
  {
    velocity_ = 0.0;
  }

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

// --- Komut tazeleme politikasi (OLCUMLE belirlendi, 2026-09-03) -------------
// Olcum 1: JTC reference 10 s boyunca sabit 0.0127 m iken gripper fiziksel
//          olarak 66.0 <-> 85.8 mm arasinda salindi. Salinimi ROS emretmiyor;
//          komut blogunu HER dongude control=1 ile yeniden yazmak cihazda
//          hareketi tekrar tekrar tetikliyor.
// Olcum 2: Komutu sadece hedef degisince bir kez yazmak da yetmiyor; o zaman
//          hareket yarida kaliyor (111.3 mm komut edilmisken gripper 91 mm'de
//          durdu ve bir daha kimildamadi).
//
// Dogru politika ikisinin arasi: hedef degistiginde yazmaya baslanir ve gripper
// DURANA kadar tazelenir; durduktan sonra yazma kesilir. Boylece hem hareket
// tamamlanir hem de yerlesince yeniden tetiklenmez. "Durdu" olcusu olculen
// genisligin degismemesidir -- bu hem hedefe varmayi (bosa kapanma) hem de bir
// cisme tutunmayi (kavrama, hedefe hic varilmaz) doğru kapsar.
static constexpr auto LOOP_PERIOD = std::chrono::milliseconds(10);  // 100 Hz
static constexpr double SETTLE_EPS = 1e-4;   // 0.1 mm = cihaz cozunurlugu
static constexpr int SETTLE_TICKS = 20;      // 20 x 10 ms = 200 ms hareketsizlik
// 2FG7 hiz komutu yuzde cinsinden, gecerli aralik 10-100. Eskiden bu register
// 10 + |(command-last_command)/dt| * 1e3 ile hesaplaniyordu; dt mikrosaniye
// mertebesine indiginde deger 25000'lere ciktigi icin cihaza gecersiz hiz
// gidiyordu. Sabit deger hem gecerli hem de tekrarlanan yazimlari ayni yapiyor.
static constexpr double SPEED_PERCENT = 50.0;

void Robot2FG7::Implementation::main_loop()
{
  uint16_t write_buffer[NUM_WRITE];
  uint16_t read_buffer[NUM_READ];
  auto t0 = std::chrono::steady_clock::now();

  double asserted_command = std::numeric_limits<double>::quiet_NaN();
  double last_width = std::numeric_limits<double>::quiet_NaN();
  int settle_ticks = 0;
  bool settled = false;

  while (is_running_)
  {
    const auto loop_start = std::chrono::steady_clock::now();
    try
    {
      std::unique_lock<std::mutex> set_guard(set_mutex_);
      const auto t1 = std::chrono::steady_clock::now();
      const double dt = static_cast<std::chrono::duration<double>>(t1 - t0).count();
      t0 = t1;
      const double command = command_;
      const double force = force_;
      last_command_ = command;
      set_guard.unlock();

      // Hedef degistiyse yeniden tazelemeye basla.
      if (!(std::fabs(command - asserted_command) < SETTLE_EPS))
      {
        asserted_command = command;
        settled = false;
        settle_ticks = 0;
      }

      int ret;
      if (!settled)
      {
        write_buffer[0] = static_cast<uint16_t>(std::fabs(command) * 1e4);  // Position
        write_buffer[1] = static_cast<uint16_t>(std::fabs(force));          // Force
        write_buffer[2] = static_cast<uint16_t>(SPEED_PERCENT);             // Velocity
        write_buffer[3] = 1;                                                // Enable control
        ret = modbus_write_and_read_registers(modbus_.get(),
          0, NUM_WRITE, write_buffer,
          256, NUM_READ, read_buffer);
      }
      else
      {
        // Yerlesti: komut registerlarina DOKUNMA, sadece durumu oku.
        ret = modbus_read_registers(modbus_.get(), 256, NUM_READ, read_buffer);
      }

      if (ret > 0)
      {
        read_state(read_buffer, dt);

        double width;
        {
          std::lock_guard<std::mutex> get_guard(get_mutex_);
          width = position_;
        }
        if (!settled)
        {
          if (std::isfinite(last_width) && std::fabs(width - last_width) < SETTLE_EPS)
          {
            if (++settle_ticks >= SETTLE_TICKS)
            {
              settled = true;
            }
          }
          else
          {
            settle_ticks = 0;
          }
        }
        last_width = width;
      }
    }
    catch (const std::exception & e)
    {
      WARNING(e.what());
    }
    std::this_thread::sleep_until(loop_start + LOOP_PERIOD);
  }
  is_running_ = false;
}

}  // namespace onrobot
