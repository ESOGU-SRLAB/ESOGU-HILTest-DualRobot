// Copyright (c) 2025 Touchlab Limited. All Rights Reserved
// Unauthorized copying or modifications of this file, via any medium is strictly prohibited.

#include <signal.h>
#include <string.h>
#include <math.h>

#include <string>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <thread>

#include <onrobot_api/onrobot_api.hpp>


using onrobot::Robot2FG7;

using namespace std::chrono_literals;

bool is_running;

void handler(int signal)
{
    is_running = false;
}

int main(int argc, char ** argv)
{
    std::string port(argv[argc - 1]);
    std::string address(argv[argc - 2]);
    std::cout << "Address: " << address << "\n";
    std::cout << "Port: " << port << "\n";
    int port_ = std::atoi(port.c_str());
    Robot2FG7 robot;
    std::cout << "Initialising\n";
    robot.init(address, port_);
    std::cout << "Serial number: " << robot.get_serial() << "\n";
    std::cout << "Firmware: " << robot.get_firmware() << "\n";

    double position;
    double velocity;
    double effort;
    double last_command;
    double command = 0.8;

    // Handle SIGTERM and SIGINT
    struct sigaction action;
    memset(&action, 0, sizeof(struct sigaction));
    action.sa_handler = handler;
    sigaction(SIGTERM, &action, NULL);
    sigaction(SIGINT, &action, NULL);

    robot.set_force(20.0);

    is_running = true;
    double t = 0.0;
    while(is_running)
    {
        command = std::min(0.061, std::max( 0.023, sin(t * M_PI / 5.0) * 0.01 + 0.04));
        robot.set_position(command);
        if (robot.get_state(position, velocity, effort, last_command, 100ms))
        {
            std::cout << std::fixed << std::setprecision(3) << position * 1e3 << "mm\t" <<
                         std::fixed << std::setprecision(3) << velocity * 1e3 << "mm/s\t" <<
                         std::fixed << std::setprecision(0) << effort << "N\t" <<
                         std::fixed << std::setprecision(3) << last_command * 1e3 << "mm\n";
        }
        std::this_thread::sleep_for(10ms);
        t += 0.01;
    }
    return 0;
}
