#define ROBOTFORLINUXTEST
#ifdef ROBOTFORLINUXTEST

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "KawasakiRS005LRobot.h"
#include "SIRLinConnection.h"
#include "SIRTypeDefs.h"
#include "SIRLogger.h"
#include "SIRConnection.h"

#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <thread>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/StdVector>

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(SIRMatrix)

// Degree to radian dönüşüm fonksiyonu
float deg2rad(float degree) {
  return degree * (M_PI / 180.0);
}

// Radian to degree dönüşüm fonksiyonu
float rad2deg(float radian) {
  return radian * (180.0 / M_PI);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("kawasaki_robot_connection_monitor");

  auto joint_state_publisher = node->create_publisher<sensor_msgs::msg::JointState>("kawasaki/joint_states", 10);

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
  
  // Bağlantı kurulduktan SONRA blocking mode ayarla
  con->setBlockingMode(0);
  
  SIRMatrix jointPose(6,1);
  robot.getJointPose(&jointPose);

  sensor_msgs::msg::JointState joint_state_msg;
  joint_state_msg.header.frame_id = "base_link";
  joint_state_msg.name = {"joint2", "joint3", "joint1", "joint4", "joint5", "joint6"};
  joint_state_msg.position.resize(6);
  joint_state_msg.velocity.resize(6);
  joint_state_msg.effort.resize(6);
  
  // Robot DEGREE cinsinden veri veriyor, ROS2'de RADIAN kullanılmalı
  for (int i = 0; i < 6; i++) {
    joint_state_msg.position[i] = deg2rad(jointPose(i,0)); // Degree -> Radian
    // joint_state_msg.velocity[i] = 0.0;
    // joint_state_msg.effort[i] = 0.0;
  }

  joint_state_publisher->publish(joint_state_msg);

  std::cout << "\nInitial Joint Positions (from robot in degrees, published in radians):" << std::endl;
  for (int i = 0; i < 6; i++) {
    std::cout << "Joint " << i+1 << ": " << jointPose(i,0) << " deg -> " 
              << deg2rad(jointPose(i,0)) << " rad" << std::endl;
  }
  std::cout << std::endl;

  while (rclcpp::ok()){
        // Robottan güncel joint pozisyonlarını al
        if (robot.getJointPose(&jointPose)) {
          joint_state_msg.header.stamp = node->now();
          
          // Robot DEGREE cinsinden veri veriyor, RADIAN'a çevir
          for (int i = 0; i < 6; i++) {
            joint_state_msg.position[i] = deg2rad(jointPose(i,0));
          }
          
          joint_state_publisher->publish(joint_state_msg);
          
          // Her 50 iterasyonda bir ekrana yazdır (yaklaşık 5 saniyede bir)
          static int counter = 0;
          if (++counter >= 50) {
            counter = 0;
            std::cout << "Publishing joint states (deg -> rad):" << std::endl;
            for (int i = 0; i < 6; i++) {
              std::cout << "  J" << i+1 << ": " << jointPose(i,0) 
                        << " deg -> " << joint_state_msg.position[i] << " rad" << std::endl;
            }
            std::cout << std::endl;
          }
        } else {
          RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 5000,
                               "Failed to get joint pose from robot");
        }
        
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 10 Hz
    }

  // Bağlantıyı düzgün kapat
  // robot.Disconnect();

  RCLCPP_INFO(node->get_logger(),
              "ROS shutting down. Robot connection closed properly.");

  rclcpp::shutdown();
  return 0;
}

#endif