// Copyright (c) 2025 Touchlab Limited. All Rights Reserved
// Unauthorized copying or modifications of this file, via any medium is strictly prohibited.

#include "onrobot_api/onrobot_api.hpp"
#include "onrobot_api_implementation.hpp"

namespace onrobot
{

Robot2FG7::Robot2FG7()
{
  impl_.reset(new Implementation());
}

Robot2FG7::~Robot2FG7()
{
  impl_.reset();
}

void Robot2FG7::init(const std::string& address, int port)
{
  impl_->init(address, port);
}

bool Robot2FG7::get_state(double& measured_position,
                double& measured_velocity,
                double& measured_force,
                double& target_position,
                std::chrono::duration<long double> timeout)
{
  return impl_->get_state(measured_position, measured_velocity, measured_force, target_position,
    timeout);
}

void Robot2FG7::set_position(double position)
{
  impl_->set_position(position);
}

void Robot2FG7::set_force(double force)
{
  impl_->set_force(force);
}

std::string Robot2FG7::get_serial()
{
  return impl_->get_serial();
}

std::string Robot2FG7::get_firmware()
{
  return impl_->get_firmware();
}

double Robot2FG7::get_min_position()
{
  return impl_->width_min_internal_;
}
double Robot2FG7::get_max_position()
{
  return impl_->width_max_internal_;
}
double Robot2FG7::get_min_force()
{
  return 20.0;
}
double Robot2FG7::get_max_force()
{
  return 140.0;
}

}  // namespace onrobot
