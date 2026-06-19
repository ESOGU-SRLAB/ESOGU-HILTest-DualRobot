// real_to_sim_bridge.cpp - Bridge between real robots and simulated robots
// Supports: UR10E arm joints + OnRobot 2FG7 gripper + Kawasaki RS005L + AGV (real → sim mirroring)
//
// İlk senkronizasyonda uzun trajectory_time kullanılır (sim robotun gerçek
// robot pozisyonuna yumuşak şekilde gitmesi için). Ardından normal rate'e geçer.
// Eksik joint'ler sessizce atlanır (driver henüz hazır değilse).

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
    last_update_time_(this->now()),
    ur_initial_sync_done_(false),
    kawasaki_initial_sync_done_(false)
  {
    // Parameters
    update_rate_ = this->declare_parameter<double>("update_rate", 50.0);
    trajectory_time_ = this->declare_parameter<double>("trajectory_time", 1.0 / update_rate_);
    // İlk senkronizasyon süresi: sim robot mevcut pozisyonundan gerçek robot
    // pozisyonuna bu sürede gider. Büyük değer = daha yumuşak geçiş.
    initial_sync_time_ = this->declare_parameter<double>("initial_sync_time", 3.0);
    
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

    // Subscriber to real robot joint states (UR + Kawasaki + AGV share /joint_states)
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 
      rclcpp::QoS(10).best_effort(),
      std::bind(&RealToSimBridge::jointStateCallback, this, std::placeholders::_1));

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

    // ==================== KAWASAKI + AGV CONFIGURATION ====================
    // Joint name mapping: real -> sim
    kawasaki_joint_mapping_ = {
      {"world_to_agv", "sim_world_to_agv"},
      {"joint1", "sim_kawasaki_joint1"},
      {"joint2", "sim_kawasaki_joint2"},
      {"joint3", "sim_kawasaki_joint3"},
      {"joint4", "sim_kawasaki_joint4"},
      {"joint5", "sim_kawasaki_joint5"},
      {"joint6", "sim_kawasaki_joint6"}
    };

    // Define the correct joint order for the Kawasaki sim controller
    kawasaki_sim_joint_order_ = {
      "sim_world_to_agv",
      "sim_kawasaki_joint1",
      "sim_kawasaki_joint2",
      "sim_kawasaki_joint3",
      "sim_kawasaki_joint4",
      "sim_kawasaki_joint5",
      "sim_kawasaki_joint6"
    };

    // Publisher to Kawasaki sim trajectory controller (BEST_EFFORT to match controller)
    kawasaki_trajectory_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/sim/kawasaki_controller/joint_trajectory",
      rclcpp::QoS(10).best_effort());

    // Precompute reverse mapping: sim_joint_name -> real_joint_name (UR)
    for (const auto& sim_joint_name : ur_sim_joint_order_) {
      for (const auto& pair : ur_joint_mapping_) {
        if (pair.second == sim_joint_name) {
          ur_reverse_mapping_[sim_joint_name] = pair.first;
          break;
        }
      }
    }

    // Precompute reverse mapping: sim_joint_name -> real_joint_name (Kawasaki)
    for (const auto& sim_joint_name : kawasaki_sim_joint_order_) {
      for (const auto& pair : kawasaki_joint_mapping_) {
        if (pair.second == sim_joint_name) {
          kawasaki_reverse_mapping_[sim_joint_name] = pair.first;
          break;
        }
      }
    }

    // ==================== LOG INFO ====================
    RCLCPP_INFO(this->get_logger(), "Real-to-Sim Bridge Node started (UR10E + Gripper + Kawasaki + AGV)");
    RCLCPP_INFO(this->get_logger(), "  Update rate: %.1f Hz, trajectory_time: %.3f s", update_rate_, trajectory_time_);
    RCLCPP_INFO(this->get_logger(), "  Initial sync time: %.1f s (smooth first move)", initial_sync_time_);
    RCLCPP_INFO(this->get_logger(), "  Listening to: /joint_states");
    RCLCPP_INFO(this->get_logger(), "  Publishing UR to:       /sim/sim_scaled_joint_trajectory_controller/joint_trajectory");
    RCLCPP_INFO(this->get_logger(), "  Publishing Gripper to:  /sim/sim_gripper_controller/joint_trajectory");
    RCLCPP_INFO(this->get_logger(), "  Publishing Kawasaki to: /sim/kawasaki_controller/joint_trajectory");
  }

private:
  // ==================== JOINT STATE CALLBACK ====================
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    // Rate limiting
    auto current_time = this->now();
    auto time_diff = (current_time - last_update_time_).seconds();
    
    if (time_diff < (1.0 / update_rate_)) {
      return;  // Skip this update
    }
    
    last_update_time_ = current_time;

    // Build a map of real joint name to position (used by all bridges)
    std::map<std::string, double> real_joint_positions;
    for (size_t i = 0; i < msg->name.size(); ++i) {
      real_joint_positions[msg->name[i]] = msg->position[i];
    }

    // ==================== UR10E BRIDGE ====================
    publishUrTrajectory(real_joint_positions);

    // ==================== GRIPPER BRIDGE ====================
    publishGripperTrajectory(real_joint_positions);

    // ==================== KAWASAKI + AGV BRIDGE ====================
    publishKawasakiTrajectory(real_joint_positions);
  }

  // ==================== UR10E TRAJECTORY PUBLISH ====================
  void publishUrTrajectory(const std::map<std::string, double>& real_joint_positions)
  {
    // Collect positions; skip entirely if no UR joints found
    std::vector<double> sim_joint_positions;
    std::vector<std::string> found_sim_joints;
    sim_joint_positions.reserve(ur_sim_joint_order_.size());
    found_sim_joints.reserve(ur_sim_joint_order_.size());

    for (const auto& sim_joint_name : ur_sim_joint_order_) {
      const std::string& real_joint_name = ur_reverse_mapping_.at(sim_joint_name);
      auto it = real_joint_positions.find(real_joint_name);
      if (it != real_joint_positions.end()) {
        found_sim_joints.push_back(sim_joint_name);
        sim_joint_positions.push_back(it->second);
      } else {
        // Joint not yet available — log once per 10s, don't add to trajectory
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
                             "UR joint '%s' not found in /joint_states (driver not ready?)",
                             real_joint_name.c_str());
      }
    }

    if (found_sim_joints.empty()) {
      return;  // No UR joints available yet
    }

    // Determine trajectory_time: use initial_sync_time for the first command
    double traj_time = trajectory_time_;
    if (!ur_initial_sync_done_) {
      traj_time = initial_sync_time_;
      ur_initial_sync_done_ = true;
      RCLCPP_INFO(this->get_logger(),
                  "UR initial sync: sending first trajectory with %.1f s duration (%zu joints)",
                  traj_time, found_sim_joints.size());
    }

    // Create joint trajectory message
    trajectory_msgs::msg::JointTrajectory traj_msg;
    traj_msg.header.stamp = rclcpp::Time(0);  // Execute immediately
    traj_msg.joint_names = found_sim_joints;

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = sim_joint_positions;
    point.velocities.resize(sim_joint_positions.size(), 0.0);
    point.accelerations.resize(sim_joint_positions.size(), 0.0);
    point.time_from_start = rclcpp::Duration::from_seconds(traj_time);

    traj_msg.points.push_back(point);
    ur_trajectory_pub_->publish(traj_msg);

    RCLCPP_DEBUG(this->get_logger(), "Published UR trajectory with %zu joints", found_sim_joints.size());
  }

  // ==================== GRIPPER TRAJECTORY PUBLISH ====================
  void publishGripperTrajectory(const std::map<std::string, double>& real_joint_positions)
  {
    auto gripper_it = real_joint_positions.find(gripper_real_joint_name_);
    if (gripper_it != real_joint_positions.end()) {
      trajectory_msgs::msg::JointTrajectory gripper_traj;
      gripper_traj.header.stamp = rclcpp::Time(0);
      gripper_traj.joint_names = {gripper_sim_joint_name_};

      trajectory_msgs::msg::JointTrajectoryPoint gp;
      gp.positions     = {gripper_it->second};
      gp.velocities    = {0.0};
      gp.accelerations = {0.0};
      gp.time_from_start = rclcpp::Duration::from_seconds(trajectory_time_);

      gripper_traj.points.push_back(gp);
      gripper_trajectory_pub_->publish(gripper_traj);

      RCLCPP_DEBUG(this->get_logger(), "Published gripper trajectory: %.4f", gripper_it->second);
    }
  }

  // ==================== KAWASAKI + AGV TRAJECTORY PUBLISH ====================
  void publishKawasakiTrajectory(const std::map<std::string, double>& real_joint_positions)
  {
    // Collect positions; skip entirely if no Kawasaki/AGV joints found
    std::vector<double> sim_joint_positions;
    std::vector<std::string> found_sim_joints;
    sim_joint_positions.reserve(kawasaki_sim_joint_order_.size());
    found_sim_joints.reserve(kawasaki_sim_joint_order_.size());

    for (const auto& sim_joint_name : kawasaki_sim_joint_order_) {
      const std::string& real_joint_name = kawasaki_reverse_mapping_.at(sim_joint_name);
      auto it = real_joint_positions.find(real_joint_name);
      if (it != real_joint_positions.end()) {
        found_sim_joints.push_back(sim_joint_name);
        sim_joint_positions.push_back(it->second);
      } else {
        // Joint not yet available — log once per 10s
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
                             "Kawasaki joint '%s' not found in /joint_states (driver not ready?)",
                             real_joint_name.c_str());
      }
    }

    if (found_sim_joints.empty()) {
      return;  // No Kawasaki/AGV joints available yet
    }

    // Determine trajectory_time: use initial_sync_time for the first command
    double traj_time = trajectory_time_;
    if (!kawasaki_initial_sync_done_) {
      traj_time = initial_sync_time_;
      kawasaki_initial_sync_done_ = true;
      RCLCPP_INFO(this->get_logger(),
                  "Kawasaki initial sync: sending first trajectory with %.1f s duration (%zu joints)",
                  traj_time, found_sim_joints.size());
    }

    // Create joint trajectory message
    trajectory_msgs::msg::JointTrajectory traj_msg;
    traj_msg.header.stamp = rclcpp::Time(0);  // Execute immediately
    traj_msg.joint_names = found_sim_joints;

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = sim_joint_positions;
    point.velocities.resize(sim_joint_positions.size(), 0.0);
    point.accelerations.resize(sim_joint_positions.size(), 0.0);
    point.time_from_start = rclcpp::Duration::from_seconds(traj_time);

    traj_msg.points.push_back(point);
    kawasaki_trajectory_pub_->publish(traj_msg);

    RCLCPP_DEBUG(this->get_logger(), "Published Kawasaki trajectory with %zu joints", found_sim_joints.size());
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
  double initial_sync_time_;
  
  // Shared
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Time last_update_time_;

  // ==================== UR10E members ====================
  std::map<std::string, std::string> ur_joint_mapping_;
  std::map<std::string, std::string> ur_reverse_mapping_;  // sim -> real (precomputed)
  std::vector<std::string> ur_sim_joint_order_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr ur_trajectory_pub_;
  bool ur_initial_sync_done_;

  // ==================== GRIPPER members ====================
  std::string gripper_real_joint_name_;
  std::string gripper_sim_joint_name_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr gripper_trajectory_pub_;

  // ==================== KAWASAKI + AGV members ====================
  std::map<std::string, std::string> kawasaki_joint_mapping_;
  std::map<std::string, std::string> kawasaki_reverse_mapping_;  // sim -> real (precomputed)
  std::vector<std::string> kawasaki_sim_joint_order_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr kawasaki_trajectory_pub_;
  bool kawasaki_initial_sync_done_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RealToSimBridge>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
