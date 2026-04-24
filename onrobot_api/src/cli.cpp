// Copyright (c) 2025 Touchlab Limited. All Rights Reserved
// Unauthorized copying or modifications of this file, via any medium is strictly prohibited.

#include <signal.h>
#include <string.h>
#include <getopt.h>

#include <exception>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>

#include <onrobot_api/onrobot_api.hpp>
#include "utils.hpp"

using onrobot::Robot2FG7;

using namespace std::chrono_literals;

bool is_running;

void handler(int signal)
{
  is_running = false;
}

enum ACTION
{
  ACTION_NONE,
  ACTION_INFO,
  ACTION_SET_POSITION,
  ACTION_SET_FORCE,
  ACTION_GET_STATE
};

int main(int argc, char** argv)
{
  int opt;
  int num_options = 0;
  double force, position;
  ACTION action = ACTION_NONE;

  while ((opt = getopt(argc, argv, "if:p:")) != -1)
  {
    num_options++;
    switch (opt)
    {
    case 'i':
      action = ACTION_INFO;
      break;
    case 'f':
      if (optarg)  // Validate optarg
      {
        try
        {
          force = std::stod(optarg);  // Convert optarg to double
          action = ACTION_SET_FORCE;
        }
        catch (const std::invalid_argument& e)
        {
          INFO("Error: Invalid value for -f (force). Must be a number.");
          return 1;
        }
      } else {
        INFO("Error: Missing argument for -f (force).");
        return 1;
      }
      break;
    case 'p':
      if (optarg)  // Validate optarg
      {
        try
        {
          position = std::stod(optarg);  // Convert optarg to double
          action = ACTION_SET_POSITION;
        }
        catch (const std::invalid_argument& e)
        {
          INFO("Error: Invalid value for -p (position). Must be a number.");
          return 1;
        }
      } else {
        INFO("Error: Missing argument for -p (position).");
        return 1;
      }
      break;
    case '?':
      num_options = 0;
      opt = -1;
      break;
    }
  }

  std::vector<std::string> args;
  if (optind < argc)
  {
    while (optind < argc)
      args.push_back(std::string(argv[optind++]));
  }

  if (num_options == 0 || args.empty())
  {
    INFO("Usage:\n" <<
         "onrobot_api [-i] <address> [port]\n" <<
         "onrobot_api [-f force] <address> [port]\n" <<
         "onrobot_api [-p position] <address> [port]");
    INFO("Options:");
    INFO("  -i \t\tPrint device info");
    INFO("  -f force\tSet gripper force");
    INFO("  -p position\tSet gripper position");
    return 1;
  }

  std::string address = args[0];
  int port = (args.size() > 1) ? std::stoi(args[1]) : 502;

  try
  {
    Robot2FG7 gripper;
    gripper.init(address, port);

    switch (action)
    {
      case ACTION_INFO:
        {
          INFO("Gripper Address: " << address);
          INFO("Gripper Port: " << port);
          INFO("Serial Number: " << gripper.get_serial());
          INFO("Firmware Version: " << gripper.get_firmware());
          INFO("Position Limits: [" << gripper.get_min_position() << ", " <<
            gripper.get_max_position() << "] m");
          INFO("Force Limits: [" << gripper.get_min_force() << ", " <<
            gripper.get_max_force() << "] N");
        }
        break;
      case ACTION_SET_POSITION:
        {
          gripper.set_position(position);
          INFO("Set gripper position to " << position << " m");
          std::this_thread::sleep_for(2s);  // Allow some time for the command to take effect
        }
        break;
      case ACTION_SET_FORCE:
        {
          gripper.set_force(force);
          INFO("Set gripper force to " << force << " N");
        }
        break;
      case ACTION_GET_STATE:
        {
          double measured_position, measured_velocity, measured_force, target_position;
          if (gripper.get_state(measured_position, measured_velocity, measured_force,
            target_position))
          {
            INFO("Gripper State:");
            INFO("  Measured Position: " << measured_position << " m");
            INFO("  Measured Velocity: " << measured_velocity << " m/s");
            INFO("  Measured Force: " << measured_force << " N");
            INFO("  Target Position: " << target_position << " m");
          } else {
            INFO("Failed to retrieve gripper state");
          }
        }
        break;
    }
  }
  catch(const std::runtime_error& e)
  {
    INFO(e.what());
    return 1;
  }

  return 0;
}
