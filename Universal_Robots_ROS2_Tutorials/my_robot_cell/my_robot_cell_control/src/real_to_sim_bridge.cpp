// real_to_sim_bridge.cpp - Bridge between real UR10e robot and simulated robot

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <string>
#include <vector>
#include <map>
#include <chrono>

using namespace std::chrono_literals;

class RealToSimBridge : public rclcpp::Node
{
public:
  RealToSimBridge()
  : Node("real_to_sim_bridge"),
    last_update_time_(this->now())
  {
    // Parameters
    update_rate_ = this->declare_parameter<double>("update_rate", 10.0);
    trajectory_time_ = this->declare_parameter<double>("trajectory_time", 0.5);
    
    // Joint name mapping: real -> sim
    joint_mapping_ = {
      {"ur10e_shoulder_pan_joint", "sim_ur10e_shoulder_pan_joint"},
      {"ur10e_shoulder_lift_joint", "sim_ur10e_shoulder_lift_joint"},
      {"ur10e_elbow_joint", "sim_ur10e_elbow_joint"},
      {"ur10e_wrist_1_joint", "sim_ur10e_wrist_1_joint"},
      {"ur10e_wrist_2_joint", "sim_ur10e_wrist_2_joint"},
      {"ur10e_wrist_3_joint", "sim_ur10e_wrist_3_joint"},
      {"ur10e_base_to_robot_mount", "sim_ur10e_base_to_robot_mount"}
    };
    
    // Define the correct joint order for the controller
    sim_joint_order_ = {
      "sim_ur10e_base_to_robot_mount",
      "sim_ur10e_shoulder_pan_joint",
      "sim_ur10e_shoulder_lift_joint",
      "sim_ur10e_elbow_joint",
      "sim_ur10e_wrist_1_joint",
      "sim_ur10e_wrist_2_joint",
      "sim_ur10e_wrist_3_joint"
    };

    // Subscriber to real robot joint states
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 
      rclcpp::QoS(10).best_effort(),
      std::bind(&RealToSimBridge::jointStateCallback, this, std::placeholders::_1));

    // Publisher to sim robot trajectory controller (BEST_EFFORT to match controller)
    joint_trajectory_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/sim/sim_scaled_joint_trajectory_controller/joint_trajectory", 
      rclcpp::QoS(10).best_effort());

    RCLCPP_INFO(this->get_logger(), "Real-to-Sim Bridge Node started");
    RCLCPP_INFO(this->get_logger(), "Update rate: %.2f Hz", update_rate_);
    RCLCPP_INFO(this->get_logger(), "Trajectory execution time: %.2f seconds", trajectory_time_);
    RCLCPP_INFO(this->get_logger(), "Listening to: /joint_states");
    RCLCPP_INFO(this->get_logger(), "Publishing to: /sim/sim_scaled_joint_trajectory_controller/joint_trajectory");
  }

private:
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    // Rate limiting
    auto current_time = this->now();
    auto time_diff = (current_time - last_update_time_).seconds();
    
    if (time_diff < (1.0 / update_rate_)) {
      return;  // Skip this update
    }
    
    last_update_time_ = current_time;

    // Create joint trajectory message for sim robot
    trajectory_msgs::msg::JointTrajectory traj_msg;
    traj_msg.header.stamp = rclcpp::Time(0);  // Execute immediately (like Kawasaki node)

    // Build a map of real joint name to position
    std::map<std::string, double> real_joint_positions;
    for (size_t i = 0; i < msg->name.size(); ++i) {
      real_joint_positions[msg->name[i]] = msg->position[i];
    }

    // Prepare joint names and positions in the correct order for sim robot
    std::vector<double> sim_joint_positions;
    sim_joint_positions.reserve(sim_joint_order_.size());
    
    for (const auto& sim_joint_name : sim_joint_order_) {
      // Find the corresponding real joint name
      std::string real_joint_name;
      for (const auto& pair : joint_mapping_) {
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
                             "Joint %s not found in real robot data", real_joint_name.c_str());
        sim_joint_positions.push_back(0.0);  // Default value
      }
    }

    // Set joint names in correct order
    traj_msg.joint_names = sim_joint_order_;

    // Create trajectory point
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = sim_joint_positions;
    
    // Set velocities to zero (let controller handle interpolation)
    point.velocities.resize(sim_joint_positions.size(), 0.0);
    point.accelerations.resize(sim_joint_positions.size(), 0.0);
    
    // Time from start for smooth motion (use shorter time like Kawasaki)
    point.time_from_start = rclcpp::Duration::from_seconds(0.1);

    traj_msg.points.push_back(point);

    // Publish trajectory
    joint_trajectory_pub_->publish(traj_msg);

    RCLCPP_DEBUG(this->get_logger(), "Published trajectory with %zu joints", sim_joint_order_.size());
  }

private:
  // Parameters
  double update_rate_;
  double trajectory_time_;
  
  // Joint name mapping
  std::map<std::string, std::string> joint_mapping_;
  std::vector<std::string> sim_joint_order_;
  
  // ROS communication
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_trajectory_pub_;
  
  // Rate limiting
  rclcpp::Time last_update_time_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RealToSimBridge>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
