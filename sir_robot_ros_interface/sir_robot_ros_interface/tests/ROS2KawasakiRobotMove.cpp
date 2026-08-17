// ROS2KawasakiRobotMove.cpp - Simple Kawasaki Robot Movement Node with Gazebo Publishing

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "KawasakiRS005LRobot.h"
#include "SIRLinConnection.h"
#include "SIRTypeDefs.h"

#include <Eigen/StdVector>
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(SIRMatrix)

#include <iostream>
#include <vector>
#include <string>
#include <thread>
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

// Degree to radian conversion
float deg2rad(float degree) {
  return degree * M_PI / 180.0f;
}

// Radian to degree conversion
float rad2deg(float radian) {
  return radian * 180.0f / M_PI;
}

class KawasakiMoveNode : public rclcpp::Node
{
public:
  KawasakiMoveNode()
  : Node("kawasaki_move_node"),
    movement_done_(false)
  {
    // Robot parametreleri
    robot_ip_   = this->declare_parameter<std::string>("robot_ip", "192.168.3.7");
    robot_port_ = this->declare_parameter<int>("robot_port", 11111);

    // Publishers for Gazebo
    joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(
      "kawasaki/joint_states", 10);
    
    // QoS profile BEST_EFFORT for Gazebo controller compatibility
    rclcpp::QoS qos_profile(10);
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    joint_trajectory_publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/kawasaki/kawasaki_controller/joint_trajectory", qos_profile);

    RCLCPP_INFO(this->get_logger(), "Kawasaki Move Node started. robot_ip=%s port=%d",
                robot_ip_.c_str(), robot_port_);

    // Robot hareketini başlat
    std::thread(&KawasakiMoveNode::runRobotMovement, this).detach();
  }

private:
  void runRobotMovement()
  {
    RCLCPP_INFO(this->get_logger(), "Starting robot movement...");

    // Robot bağlantısı ve logger oluştur
    SIRLogger *logger = new SIRLogger("log.txt", SIRLOGLEVEL::LOGLEVEL_DEBUG);
    SIRConnection *con = new SIRLinConnection(logger, robot_ip_.c_str(), robot_port_);

    // MPT_JOINT kullan - eklem açıları gönderiyoruz (derece cinsinden)
    KawasakiRS005LRobot robot(con, logger, nullptr, MPT_JOINT, MT_P2P);

    // Robot'a bağlan
    if (!robot.Connect()) {
      RCLCPP_ERROR(this->get_logger(), "Could not connect to robot at %s:%d", 
                   robot_ip_.c_str(), robot_port_);
      cleanup(robot, con, logger);
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Successfully connected to robot.");

    // Non-blocking mode ayarla
    con->setBlockingMode(0);
    robot.setWaitForCommand(2000);

    std::this_thread::sleep_for(2s);

    // Hareket noktalarını hazırla
    std::vector<SIRMatrix> waypoints = createWaypoints();

    RCLCPP_INFO(this->get_logger(), "Moving robot through %zu waypoints...", waypoints.size());

    // Tüm noktaları robota gönder
    for (size_t i = 0; i < waypoints.size(); i++) {
      int packet_id = robot.add(waypoints[i]);
      RCLCPP_INFO(this->get_logger(), "Added waypoint %zu, packet ID: %d", i+1, packet_id);
      std::cout << "Waypoint " << i+1 << ": ";
      for (int j = 0; j < 6; j++) {
        std::cout << waypoints[i](j, 0) << " ";
      }
      std::cout << std::endl;
    }

    // Hareketi başlat
    if (!robot.move()) {
      RCLCPP_ERROR(this->get_logger(), "robot.move() failed!");
      cleanup(robot, con, logger);
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Robot moving...");

    // Robot duruncaya kadar bekle VE pozisyonu Gazebo'ya yayınla
    SIRMatrix jointPose(6, 1);
    std::vector<std::string> joint_names = {
      "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"
    };
    
    while (robot.getStatus() != RS_STOP) {
      // Robottan güncel pozisyonu al
      if (robot.getJointPose(&jointPose)) {
        publishToGazebo(jointPose, joint_names);
      }
      
      std::this_thread::sleep_for(100ms);  // 10 Hz publish rate
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                           "Robot still moving...");
    }

    RCLCPP_INFO(this->get_logger(), "Robot reached final position.");
    
    // Son pozisyonu da yayınla
    if (robot.getJointPose(&jointPose)) {
      publishToGazebo(jointPose, joint_names);
    }
    
    movement_done_ = true;

    // Bağlantıyı kapat
    if (robot.close()) {
      RCLCPP_INFO(this->get_logger(), "Robot connection closed successfully.");
    } else {
      RCLCPP_WARN(this->get_logger(), "Robot connection close reported failure.");
    }

    cleanup(robot, con, logger);

    RCLCPP_INFO(this->get_logger(), "Movement completed. Node will continue running...");
  }

  std::vector<SIRMatrix> createWaypoints()
  {
    std::vector<SIRMatrix> waypoints;

    // 1. nokta: 15.837, -28.815, -76.154, -154.640, 31.511, 47.310
    SIRMatrix point1(6, 1);
    point1 << 15.837, -28.815, -76.154, -154.640, 31.511, 47.310;
    waypoints.push_back(point1);

    // 2. nokta: 39.390, -34.935, -76.969, -126.004, 46.171, 32.372
    SIRMatrix point2(6, 1);
    point2 << -104.117, 20.814, -31.719, -290.426, 94.088, 116.370;
    waypoints.push_back(point2);

    // 3. nokta: 15.118, -20.317, -72.776, -151.849, 26.688, 43.579
    SIRMatrix point3(6, 1);
    point3 << -71.503, 24.562, -25.947, -270.191, 68.745, 118.992;
    waypoints.push_back(point3);

    SIRMatrix point4(6, 1);
    point4 << -71.503, 0.877, -101.176, -287.050, 77.113, 172.441;
    waypoints.push_back(point4);

    SIRMatrix point5(6, 1);
    point5 << -71.503, 38.622, -125.932, -289.487, 98.656, 231.511;
    waypoints.push_back(point5);

    SIRMatrix point6(6, 1);
    point6 << -71.503, 0.877, -101.176, -287.050, 77.113, 172.441;
    waypoints.push_back(point6);

    SIRMatrix point7(6, 1);
    point7 << -71.503, 24.562, -25.947, -270.191, 68.745, 118.992;
    waypoints.push_back(point7);

    SIRMatrix point8(6, 1);
    point8 << -104.117, 20.814, -31.719, -290.426, 94.088, 116.370;
    waypoints.push_back(point8);

    SIRMatrix point9(6, 1);
    point9 << 15.837, -28.815, -76.154, -154.640, 31.511, 47.310;
    waypoints.push_back(point9);

    return waypoints;
  }

  void publishToGazebo(const SIRMatrix& jointPose, const std::vector<std::string>& joint_names)
  {
    auto current_time = this->now();
    
    // 1. Joint State message (for monitoring)
    sensor_msgs::msg::JointState joint_state_msg;
    joint_state_msg.header.stamp = current_time;
    joint_state_msg.header.frame_id = "base_link";
    joint_state_msg.name = joint_names;
    joint_state_msg.position.resize(6);
    joint_state_msg.velocity.resize(6);
    joint_state_msg.effort.resize(6);
    
    // Convert degree to radian for ROS/Gazebo
    for (int i = 0; i < 6; i++) {
      joint_state_msg.position[i] = deg2rad(jointPose(i, 0));
      joint_state_msg.velocity[i] = 0.0;
      joint_state_msg.effort[i] = 0.0;
    }
    
    joint_state_publisher_->publish(joint_state_msg);
    
    // 2. Joint Trajectory message (for Gazebo controller)
    trajectory_msgs::msg::JointTrajectory joint_trajectory_msg;
    joint_trajectory_msg.header.stamp = rclcpp::Time(0);  // Execute immediately
    joint_trajectory_msg.joint_names = joint_names;
    
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions.resize(6);
    point.velocities.resize(6);
    point.accelerations.resize(6);
    
    for (int i = 0; i < 6; i++) {
      point.positions[i] = deg2rad(jointPose(i, 0));
      point.velocities[i] = 0.0;
      point.accelerations[i] = 0.0;
    }
    
    point.time_from_start = rclcpp::Duration::from_seconds(0.1);
    joint_trajectory_msg.points.push_back(point);
    joint_trajectory_publisher_->publish(joint_trajectory_msg);
  }

  void cleanup(KawasakiRS005LRobot &robot, SIRConnection *con, SIRLogger *logger)
  {
    (void)robot;
    delete con;
    delete logger;
  }

private:
  std::string robot_ip_;
  int robot_port_;
  bool movement_done_;
  
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_trajectory_publisher_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<KawasakiMoveNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
