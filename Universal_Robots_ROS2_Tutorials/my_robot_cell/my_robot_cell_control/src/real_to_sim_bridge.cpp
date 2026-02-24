// real_to_sim_bridge.cpp - Bridge between real robots and simulated robots
// Supports: UR10E + Kawasaki RS005L + AGV (OTA base)

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <string>
#include <vector>
#include <map>
#include <chrono>
#include <cmath>
#include <mutex>

using namespace std::chrono_literals;

class RealToSimBridge : public rclcpp::Node
{
public:
  RealToSimBridge()
  : Node("real_to_sim_bridge"),
    ur_last_update_time_(this->now()),
    kawasaki_last_update_time_(this->now()),
    real_agv_odom_received_(false),
    sim_agv_odom_received_(false)
  {
    // Parameters
    update_rate_ = this->declare_parameter<double>("update_rate", 10.0);
    trajectory_time_ = this->declare_parameter<double>("trajectory_time", 0.5);
    
    // AGV P-controller gains
    agv_kp_linear_ = this->declare_parameter<double>("agv_kp_linear", 1.5);
    agv_kp_angular_ = this->declare_parameter<double>("agv_kp_angular", 2.0);
    agv_max_linear_vel_ = this->declare_parameter<double>("agv_max_linear_vel", 1.0);
    agv_max_angular_vel_ = this->declare_parameter<double>("agv_max_angular_vel", 2.0);
    
    // ==================== UR10E CONFIGURATION ====================
    // Joint name mapping: real -> sim
    ur_joint_mapping_ = {
      {"ur10e_shoulder_pan_joint", "sim_ur10e_shoulder_pan_joint"},
      {"ur10e_shoulder_lift_joint", "sim_ur10e_shoulder_lift_joint"},
      {"ur10e_elbow_joint", "sim_ur10e_elbow_joint"},
      {"ur10e_wrist_1_joint", "sim_ur10e_wrist_1_joint"},
      {"ur10e_wrist_2_joint", "sim_ur10e_wrist_2_joint"},
      {"ur10e_wrist_3_joint", "sim_ur10e_wrist_3_joint"},
      {"ur10e_base_to_robot_mount", "sim_ur10e_base_to_robot_mount"}
    };
    
    // Define the correct joint order for the UR sim controller
    ur_sim_joint_order_ = {
      "sim_ur10e_base_to_robot_mount",
      "sim_ur10e_shoulder_pan_joint",
      "sim_ur10e_shoulder_lift_joint",
      "sim_ur10e_elbow_joint",
      "sim_ur10e_wrist_1_joint",
      "sim_ur10e_wrist_2_joint",
      "sim_ur10e_wrist_3_joint"
    };

    // Subscriber to real UR10E robot joint states
    ur_joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 
      rclcpp::QoS(10).best_effort(),
      std::bind(&RealToSimBridge::urJointStateCallback, this, std::placeholders::_1));

    // Publisher to UR sim robot trajectory controller (BEST_EFFORT to match controller)
    ur_trajectory_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/sim/sim_scaled_joint_trajectory_controller/joint_trajectory", 
      rclcpp::QoS(10).best_effort());

    // ==================== KAWASAKI CONFIGURATION ====================
    // Kawasaki joint names - same in real and Gazebo (no remap needed)
    kawasaki_joint_names_ = {
      "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"
    };

    // Subscriber to Kawasaki robot joint states
    // (Published by ROS2KawasakiRobotMove or ROS2KawasakiRobotGazeboJSP nodes)
    kawasaki_joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/kawasaki/joint_states",
      rclcpp::QoS(10).best_effort(),
      std::bind(&RealToSimBridge::kawasakiJointStateCallback, this, std::placeholders::_1));

    // Publisher to Kawasaki Gazebo controller (BEST_EFFORT for controller compatibility)
    kawasaki_trajectory_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/kawasaki/kawasaki_controller/joint_trajectory",
      rclcpp::QoS(10).best_effort());

    // ==================== AGV CONFIGURATION ====================
    // Subscriber to real AGV odometry (from bridge_node.py)
    agv_real_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/agv/odom",
      rclcpp::QoS(10),
      std::bind(&RealToSimBridge::agvRealOdomCallback, this, std::placeholders::_1));

    // Subscriber to Gazebo AGV odometry (from ota_base_controller)
    agv_sim_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/kawasaki/ota_base_controller/odom",
      rclcpp::QoS(10),
      std::bind(&RealToSimBridge::agvSimOdomCallback, this, std::placeholders::_1));

    // Publisher to Gazebo AGV diff_drive controller
    agv_cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/kawasaki/ota_base_controller/cmd_vel_unstamped",
      rclcpp::QoS(10));

    // Timer for AGV P-controller loop
    agv_control_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / update_rate_)),
      std::bind(&RealToSimBridge::agvControlLoop, this));

    // ==================== LOG INFO ====================
    RCLCPP_INFO(this->get_logger(), "Real-to-Sim Bridge Node started");
    RCLCPP_INFO(this->get_logger(), "Update rate: %.2f Hz", update_rate_);
    RCLCPP_INFO(this->get_logger(), "--- UR10E ---");
    RCLCPP_INFO(this->get_logger(), "  Listening to: /joint_states");
    RCLCPP_INFO(this->get_logger(), "  Publishing to: /sim/sim_scaled_joint_trajectory_controller/joint_trajectory");
    RCLCPP_INFO(this->get_logger(), "--- Kawasaki ---");
    RCLCPP_INFO(this->get_logger(), "  Listening to: /kawasaki/joint_states");
    RCLCPP_INFO(this->get_logger(), "  Publishing to: /kawasaki/kawasaki_controller/joint_trajectory");
    RCLCPP_INFO(this->get_logger(), "--- AGV (P-controller) ---");
    RCLCPP_INFO(this->get_logger(), "  Real odom: /agv/odom");
    RCLCPP_INFO(this->get_logger(), "  Sim odom:  /kawasaki/ota_base_controller/odom");
    RCLCPP_INFO(this->get_logger(), "  Cmd output: /kawasaki/ota_base_controller/cmd_vel_unstamped");
    RCLCPP_INFO(this->get_logger(), "  Kp_linear=%.2f, Kp_angular=%.2f", agv_kp_linear_, agv_kp_angular_);
  }

private:
  // ==================== UR10E CALLBACK ====================
  void urJointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    // Rate limiting
    auto current_time = this->now();
    auto time_diff = (current_time - ur_last_update_time_).seconds();
    
    if (time_diff < (1.0 / update_rate_)) {
      return;  // Skip this update
    }
    
    ur_last_update_time_ = current_time;

    // Create joint trajectory message for sim robot
    trajectory_msgs::msg::JointTrajectory traj_msg;
    traj_msg.header.stamp = rclcpp::Time(0);  // Execute immediately

    // Build a map of real joint name to position
    std::map<std::string, double> real_joint_positions;
    for (size_t i = 0; i < msg->name.size(); ++i) {
      real_joint_positions[msg->name[i]] = msg->position[i];
    }

    // Prepare joint names and positions in the correct order for sim robot
    std::vector<double> sim_joint_positions;
    sim_joint_positions.reserve(ur_sim_joint_order_.size());
    
    for (const auto& sim_joint_name : ur_sim_joint_order_) {
      // Find the corresponding real joint name
      std::string real_joint_name;
      for (const auto& pair : ur_joint_mapping_) {
        if (pair.second == sim_joint_name) {
          real_joint_name = pair.first;
          break;
        }
      }
      
      // Get the position from real robot
      auto it = real_joint_positions.find(real_joint_name);
      if (it != real_joint_positions.end()) {
        sim_joint_positions.push_back(it->second);
      } else {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "UR joint %s not found in real robot data", real_joint_name.c_str());
        sim_joint_positions.push_back(0.0);  // Default value
      }
    }

    // Set joint names in correct order
    traj_msg.joint_names = ur_sim_joint_order_;

    // Create trajectory point
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = sim_joint_positions;
    
    // Set velocities to zero (let controller handle interpolation)
    point.velocities.resize(sim_joint_positions.size(), 0.0);
    point.accelerations.resize(sim_joint_positions.size(), 0.0);
    
    // Time from start for smooth motion
    point.time_from_start = rclcpp::Duration::from_seconds(0.1);

    traj_msg.points.push_back(point);

    // Publish trajectory
    ur_trajectory_pub_->publish(traj_msg);

    RCLCPP_DEBUG(this->get_logger(), "Published UR trajectory with %zu joints", ur_sim_joint_order_.size());
  }

  // ==================== KAWASAKI CALLBACK ====================
  void kawasakiJointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    // Rate limiting
    auto current_time = this->now();
    auto time_diff = (current_time - kawasaki_last_update_time_).seconds();
    
    if (time_diff < (1.0 / update_rate_)) {
      return;  // Skip this update
    }
    
    kawasaki_last_update_time_ = current_time;

    // Build a map of incoming joint positions
    std::map<std::string, double> joint_positions;
    for (size_t i = 0; i < msg->name.size(); ++i) {
      joint_positions[msg->name[i]] = msg->position[i];
    }

    // Create Joint Trajectory message for Kawasaki Gazebo controller
    trajectory_msgs::msg::JointTrajectory traj_msg;
    traj_msg.header.stamp = rclcpp::Time(0);  // Execute immediately
    traj_msg.joint_names = kawasaki_joint_names_;

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions.resize(kawasaki_joint_names_.size());
    point.velocities.resize(kawasaki_joint_names_.size(), 0.0);
    point.accelerations.resize(kawasaki_joint_names_.size(), 0.0);

    for (size_t i = 0; i < kawasaki_joint_names_.size(); ++i) {
      auto it = joint_positions.find(kawasaki_joint_names_[i]);
      if (it != joint_positions.end()) {
        // Joint states already in radians (converted by the Kawasaki ROS node)
        point.positions[i] = it->second;
      } else {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "Kawasaki joint %s not found in joint state data",
                             kawasaki_joint_names_[i].c_str());
        point.positions[i] = 0.0;
      }
    }

    point.time_from_start = rclcpp::Duration::from_seconds(0.1);
    traj_msg.points.push_back(point);

    // Publish to Kawasaki Gazebo controller
    kawasaki_trajectory_pub_->publish(traj_msg);

    RCLCPP_DEBUG(this->get_logger(), "Published Kawasaki trajectory with %zu joints",
                 kawasaki_joint_names_.size());
  }

  // ==================== AGV ODOM CALLBACKS ====================
  void agvRealOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(agv_mutex_);
    real_agv_x_ = msg->pose.pose.position.x;
    real_agv_y_ = msg->pose.pose.position.y;
    real_agv_yaw_ = quaternionToYaw(
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w);
    real_agv_odom_received_ = true;
  }

  void agvSimOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(agv_mutex_);
    sim_agv_x_ = msg->pose.pose.position.x;
    sim_agv_y_ = msg->pose.pose.position.y;
    sim_agv_yaw_ = quaternionToYaw(
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w);
    sim_agv_odom_received_ = true;
  }

  // ==================== AGV P-CONTROLLER LOOP ====================
  void agvControlLoop()
  {
    std::lock_guard<std::mutex> lock(agv_mutex_);

    if (!real_agv_odom_received_ || !sim_agv_odom_received_) {
      RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                            "Waiting for AGV odom data (real:%s, sim:%s)",
                            real_agv_odom_received_ ? "OK" : "NO",
                            sim_agv_odom_received_ ? "OK" : "NO");
      return;
    }

    // Position error in world frame
    double dx = real_agv_x_ - sim_agv_x_;
    double dy = real_agv_y_ - sim_agv_y_;

    // Yaw error (normalize to [-pi, pi])
    double dyaw = normalizeAngle(real_agv_yaw_ - sim_agv_yaw_);

    // Transform position error to robot body frame
    double cos_yaw = std::cos(sim_agv_yaw_);
    double sin_yaw = std::sin(sim_agv_yaw_);
    double error_forward = dx * cos_yaw + dy * sin_yaw;   // Along robot's x-axis
    double error_lateral = -dx * sin_yaw + dy * cos_yaw;  // Along robot's y-axis

    // Distance error in body-forward direction
    double distance_error = std::sqrt(dx * dx + dy * dy);

    // If the target is behind us, we need to consider the heading to target
    double angle_to_target = std::atan2(dy, dx);
    double heading_error = normalizeAngle(angle_to_target - sim_agv_yaw_);

    // P-controller output
    geometry_msgs::msg::Twist cmd_vel;

    // If far from target, first rotate toward it, then drive forward
    if (distance_error > 0.02) {  // 2cm threshold
      // Compute linear velocity (proportional to forward error)
      cmd_vel.linear.x = agv_kp_linear_ * error_forward;
      
      // Compute angular velocity (combine heading error + yaw alignment)
      // When far: prioritize heading toward target
      // When close: prioritize final yaw alignment
      double heading_weight = std::min(distance_error / 0.5, 1.0);  // Smooth blend
      double angular_error = heading_weight * heading_error + (1.0 - heading_weight) * dyaw;
      cmd_vel.angular.z = agv_kp_angular_ * angular_error;
    } else {
      // Close enough in position, just fix yaw
      cmd_vel.linear.x = 0.0;
      cmd_vel.angular.z = agv_kp_angular_ * dyaw;

      // If yaw is also close enough, stop
      if (std::abs(dyaw) < 0.02) {  // ~1 degree
        cmd_vel.angular.z = 0.0;
      }
    }

    // Clamp velocities
    cmd_vel.linear.x = clamp(cmd_vel.linear.x, -agv_max_linear_vel_, agv_max_linear_vel_);
    cmd_vel.angular.z = clamp(cmd_vel.angular.z, -agv_max_angular_vel_, agv_max_angular_vel_);

    agv_cmd_vel_pub_->publish(cmd_vel);

    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                          "AGV tracking: dist_err=%.3f yaw_err=%.3f cmd_vx=%.3f cmd_wz=%.3f",
                          distance_error, dyaw, cmd_vel.linear.x, cmd_vel.angular.z);
  }

  // ==================== UTILITY FUNCTIONS ====================
  static double quaternionToYaw(double qx, double qy, double qz, double qw)
  {
    // yaw (z-axis rotation) from quaternion
    double siny_cosp = 2.0 * (qw * qz + qx * qy);
    double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    return std::atan2(siny_cosp, cosy_cosp);
  }

  static double normalizeAngle(double angle)
  {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
  }

  static double clamp(double value, double min_val, double max_val)
  {
    return std::max(min_val, std::min(value, max_val));
  }

private:
  // Parameters
  double update_rate_;
  double trajectory_time_;
  
  // ==================== UR10E members ====================
  std::map<std::string, std::string> ur_joint_mapping_;
  std::vector<std::string> ur_sim_joint_order_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr ur_joint_state_sub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr ur_trajectory_pub_;
  rclcpp::Time ur_last_update_time_;

  // ==================== Kawasaki members ====================
  std::vector<std::string> kawasaki_joint_names_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr kawasaki_joint_state_sub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr kawasaki_trajectory_pub_;
  rclcpp::Time kawasaki_last_update_time_;

  // ==================== AGV members ====================
  // AGV P-controller gains
  double agv_kp_linear_;
  double agv_kp_angular_;
  double agv_max_linear_vel_;
  double agv_max_angular_vel_;

  // Odom state (protected by mutex for thread safety)
  std::mutex agv_mutex_;
  double real_agv_x_ = 0.0, real_agv_y_ = 0.0, real_agv_yaw_ = 0.0;
  double sim_agv_x_ = 0.0, sim_agv_y_ = 0.0, sim_agv_yaw_ = 0.0;
  bool real_agv_odom_received_;
  bool sim_agv_odom_received_;

  // ROS communication
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr agv_real_odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr agv_sim_odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr agv_cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr agv_control_timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RealToSimBridge>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
