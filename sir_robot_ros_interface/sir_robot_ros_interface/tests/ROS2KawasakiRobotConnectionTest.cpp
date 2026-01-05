#define ROBOTFORLINUXTEST
#ifdef ROBOTFORLINUXTEST

#include <rclcpp/rclcpp.hpp>

#include "KawasakiRS005LRobot.h"
#include "SIRLinConnection.h"
#include "SIRTypeDefs.h"
#include "SIRLogger.h"
#include "SIRConnection.h"

#include <iostream>
#include <memory>
#include <string>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("kawasaki_robot_connection_monitor");

  // ---- Robot objects ----
  auto logger = std::make_unique<SIRLogger>("log.txt", SIRLOGLEVEL::LOGLEVEL_DEBUG);

  const std::string robot_ip = "192.168.3.7";
  const int robot_port = 11111;

  auto con = std::make_unique<SIRLinConnection>(
      logger.get(),
      robot_ip.c_str(),
      robot_port);

  // 3. arg: SIRPoseDB*  -> nullptr
  KawasakiRS005LRobot robot(
      con.get(),
      logger.get(),
      nullptr,
      MPT_TASK,
      MT_LINEAR);

  con->setBlockingMode(0);
  robot.setWaitForCommand(2000);

  // ---- Connect ----
  if (!robot.Connect()) {
    std::cout << "Could not connect to robot..." << std::endl;
    RCLCPP_ERROR(node->get_logger(),
                 "Could not connect to robot at %s:%d",
                 robot_ip.c_str(),
                 robot_port);
    rclcpp::shutdown();
    return 1;
  }

  std::cout << "Connected to Kawasaki robot." << std::endl;
  RCLCPP_INFO(node->get_logger(),
              "Connected to Kawasaki robot at %s:%d",
              robot_ip.c_str(),
              robot_port);

  // ---- 1 Hz heartbeat ----
  rclcpp::Rate rate(1.0);  // 1 Hz

  while (rclcpp::ok()) {
    std::cout << "[KAWASAKI] Robot connection is alive." << std::endl;
    RCLCPP_INFO(node->get_logger(),
                "Robot connection is alive.");

    rclcpp::spin_some(node);
    rate.sleep();
  }

  // NOT disconnecting on purpose
  RCLCPP_WARN(node->get_logger(),
              "ROS shutting down. Robot connection was kept open.");

  rclcpp::shutdown();
  return 0;
}

#endif
