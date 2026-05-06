// real_to_sim_bridge.cpp - Bridge between real robots and simulated robots
// Supports: UR10E arm joints + OnRobot 2FG7 gripper (real → sim mirroring)

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

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
    ur_last_update_time_(this->now())
  {
    // Parameters
    update_rate_ = this->declare_parameter<double>("update_rate", 10.0);
    trajectory_time_ = this->declare_parameter<double>("trajectory_time", 0.5);
    
    // ...existing code...
    
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

    // ==================== GRIPPER CONFIGURATION ====================
    gripper_real_joint_name_ = "ur10e_gripper_joint";
    gripper_sim_joint_name_  = "sim_ur10e_gripper_joint";

    gripper_trajectory_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/sim/sim_gripper_controller/joint_trajectory",
      rclcpp::QoS(10).best_effort());

    // ==================== LOG INFO ====================
    RCLCPP_INFO(this->get_logger(), "Real-to-Sim Bridge Node started (UR10E + Gripper)");
    RCLCPP_INFO(this->get_logger(), "Update rate: %.2f Hz", update_rate_);
    RCLCPP_INFO(this->get_logger(), "  Listening to: /joint_states");
    RCLCPP_INFO(this->get_logger(), "  Publishing to: /sim/sim_scaled_joint_trajectory_controller/joint_trajectory");
    RCLCPP_INFO(this->get_logger(), "  Publishing to: /sim/sim_gripper_controller/joint_trajectory");
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

    // ==================== GRIPPER BRIDGE ====================
    auto gripper_it = real_joint_positions.find(gripper_real_joint_name_);
    if (gripper_it != real_joint_positions.end()) {
      trajectory_msgs::msg::JointTrajectory gripper_traj;
      gripper_traj.header.stamp = rclcpp::Time(0);
      gripper_traj.joint_names = {gripper_sim_joint_name_};

      trajectory_msgs::msg::JointTrajectoryPoint gp;
      gp.positions     = {gripper_it->second};
      gp.velocities    = {0.0};
      gp.accelerations = {0.0};
      gp.time_from_start = rclcpp::Duration::from_seconds(0.1);

      gripper_traj.points.push_back(gp);
      gripper_trajectory_pub_->publish(gripper_traj);

      RCLCPP_DEBUG(this->get_logger(), "Published gripper trajectory: %.4f", gripper_it->second);
    }
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

  // ==================== GRIPPER members ====================
  std::string gripper_real_joint_name_;
  std::string gripper_sim_joint_name_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr gripper_trajectory_pub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RealToSimBridge>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
