#define ROBOTFORLINUXTEST
#ifdef ROBOTFORLINUXTEST

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

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
  auto node = rclcpp::Node::make_shared("kawasaki_robot_gazebo_bridge");
  
  // Joint state publisher - robottan okunan pozisyonları yayınlar
  auto joint_state_publisher = node->create_publisher<sensor_msgs::msg::JointState>("kawasaki/joint_states", 10);
  
  // Joint trajectory publisher - Gazebo kontrolcüsüne komut gönderir
  // QoS profili BEST_EFFORT olmalı (kontrolcü ile uyumlu olması için)
  rclcpp::QoS qos_profile(10);
  qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  auto joint_trajectory_publisher = node->create_publisher<trajectory_msgs::msg::JointTrajectory>(
    "/kawasaki/kawasaki_controller/joint_trajectory", qos_profile);
  
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
  
  RCLCPP_INFO(node->get_logger(), "Connected to robot successfully!");
  
  // Joint isimleri - Kontrolcünün beklediği sıra
  std::vector<std::string> joint_names = {
    "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"
  };
  
  // HİÇBİR REMAP YOK - Direkt 1:1 eşleştirme
  // jointPose(0) -> joint1
  // jointPose(1) -> joint2
  // jointPose(2) -> joint3
  // jointPose(3) -> joint4
  // jointPose(4) -> joint5
  // jointPose(5) -> joint6
  
  // İlk pozisyonu oku
  SIRMatrix jointPose(6,1);
  robot.getJointPose(&jointPose);

  std::cout << "\nInitial Joint Positions (from robot in degrees, will be published in radians):" << std::endl;
  for (int i = 0; i < 6; i++) {
    std::cout << "Joint " << i+1 << ": " << jointPose(i,0) << " deg -> " 
              << deg2rad(jointPose(i,0)) << " rad" << std::endl;
  }
  std::cout << std::endl;

  // Ana döngü - Daha yavaş gönder (1 Hz), kontrolcü mesajları işleyebilsin
  rclcpp::Rate loop_rate(1); // 1 Hz - saniyede 1 komut
  int seq = 0;
  
  RCLCPP_INFO(node->get_logger(), "Starting main loop - reading from robot and sending to Gazebo controller...");
  
  while (rclcpp::ok()) {
    // Robottan güncel joint pozisyonlarını al
    if (robot.getJointPose(&jointPose)) {
      auto current_time = node->now();
      
      // 1. Joint State mesajı oluştur ve yayınla
      sensor_msgs::msg::JointState joint_state_msg;
      joint_state_msg.header.stamp = current_time;
      joint_state_msg.header.frame_id = "base_link";
      joint_state_msg.name = joint_names;
      joint_state_msg.position.resize(6);
      joint_state_msg.velocity.resize(6);
      joint_state_msg.effort.resize(6);
      
      // Robot DEGREE cinsinden veri veriyor, RADIAN'a çevir
      // Direkt 1:1 eşleştirme - hiçbir remap yok
      for (int i = 0; i < 6; i++) {
        joint_state_msg.position[i] = deg2rad(jointPose(i,0));
        joint_state_msg.velocity[i] = 0.0;
        joint_state_msg.effort[i] = 0.0;
      }
      
      joint_state_publisher->publish(joint_state_msg);
      
      // 2. Joint Trajectory mesajı oluştur ve Gazebo kontrolcüsüne gönder
      trajectory_msgs::msg::JointTrajectory joint_trajectory_msg;
      joint_trajectory_msg.header.stamp = rclcpp::Time(0); // Boş timestamp - hemen başlat
      joint_trajectory_msg.joint_names = joint_names;
      
      trajectory_msgs::msg::JointTrajectoryPoint point;
      point.positions.resize(6);
      point.velocities.resize(6);
      point.accelerations.resize(6);
      
      // Gazebo kontrolcüsü de RADIAN bekliyor
      // Direkt 1:1 eşleştirme - hiçbir remap yok
      for (int i = 0; i < 6; i++) {
        point.positions[i] = deg2rad(jointPose(i,0));
        point.velocities[i] = 0.0;
        point.accelerations[i] = 0.0;
      }
      point.time_from_start = rclcpp::Duration::from_seconds(0.5); // 0.5 saniye - daha hızlı tepki
      
      joint_trajectory_msg.points.push_back(point);
      joint_trajectory_publisher->publish(joint_trajectory_msg);
      
      // Her iterasyonda ekrana yazdır
      if (seq % 5 == 0) {
        std::cout << "Publishing [" << seq << "] (deg -> rad, direct 1:1 mapping, no remap):" << std::endl;
        for (int i = 0; i < 6; i++) {
          float original_deg = jointPose(i,0);
          float rad_value = deg2rad(original_deg);
          std::cout << "  jointPose(" << i << ") = " << original_deg << " deg -> " 
                    << joint_names[i] << " = " << rad_value << " rad" << std::endl;
        }
        std::cout << std::endl;
      }
      
      seq++;
    } else {
      RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 5000,
                           "Failed to get joint pose from robot");
    }
    
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  // Bağlantıyı düzgün kapat
  // robot.Disconnect();

  RCLCPP_INFO(node->get_logger(),
              "ROS shutting down. Robot connection closed properly.");

  rclcpp::shutdown();
  return 0;
}

#endif