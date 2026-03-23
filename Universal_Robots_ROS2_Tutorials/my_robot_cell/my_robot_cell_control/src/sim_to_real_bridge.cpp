// sim_to_real_bridge.cpp
// Simülasyondaki UR10E pozisyonlarını gerçek robota gönderir
// UYARI: Gerçek fiziksel harekete yol açar!

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <std_msgs/msg/bool.hpp>

using namespace std::chrono_literals;

class SimToRealBridge : public rclcpp::Node
{
public:
  SimToRealBridge()
  : Node("sim_to_real_bridge"),
    ur_last_update_time_(this->now()),
    bridge_enabled_(false)
  {
    update_rate_         = this->declare_parameter<double>("update_rate", 10.0);
    trajectory_time_     = this->declare_parameter<double>("trajectory_time", 0.5);
    max_joint_velocity_  = this->declare_parameter<double>("max_joint_velocity", 0.5);

    // Sim joint -> Real joint mapping
    ur_joint_mapping_ = {
      {"sim_ur10e_base_to_robot_mount", "ur10e_base_to_robot_mount"},
      {"sim_ur10e_shoulder_pan_joint",  "ur10e_shoulder_pan_joint"},
      {"sim_ur10e_shoulder_lift_joint", "ur10e_shoulder_lift_joint"},
      {"sim_ur10e_elbow_joint",         "ur10e_elbow_joint"},
      {"sim_ur10e_wrist_1_joint",       "ur10e_wrist_1_joint"},
      {"sim_ur10e_wrist_2_joint",       "ur10e_wrist_2_joint"},
      {"sim_ur10e_wrist_3_joint",       "ur10e_wrist_3_joint"},
    };

    ur_real_joint_order_ = {
      "ur10e_base_to_robot_mount",
      "ur10e_shoulder_pan_joint",
      "ur10e_shoulder_lift_joint",
      "ur10e_elbow_joint",
      "ur10e_wrist_1_joint",
      "ur10e_wrist_2_joint",
      "ur10e_wrist_3_joint"
    };

    // Sim'den joint state oku
    ur_sim_joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/sim/joint_states",
      rclcpp::QoS(10).best_effort(),
      std::bind(&SimToRealBridge::urSimJointStateCallback, this, std::placeholders::_1));

    // Gerçek UR10E controller'ına yaz (RELIABLE — gerçek robot için zorunlu)
    ur_real_trajectory_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/scaled_joint_trajectory_controller/joint_trajectory",
      rclcpp::QoS(10).reliable());

    // Güvenlik: enable/disable topic
    enable_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/sim_to_real/enable",
      10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        bridge_enabled_ = msg->data;
        RCLCPP_WARN(this->get_logger(),
          bridge_enabled_
            ? "⚠️  Sim-to-Real ETKINLEŞTIRILDI — Gerçek robot hareket edecek!"
            : "✅  Sim-to-Real DEVRE DIŞI");
      });

    RCLCPP_INFO(this->get_logger(), "SimToReal Bridge başlatıldı");
    RCLCPP_INFO(this->get_logger(), "Update rate: %.2f Hz", update_rate_);
    RCLCPP_INFO(this->get_logger(), "Max joint velocity: %.2f rad/s", max_joint_velocity_);
    RCLCPP_INFO(this->get_logger(), "Dinleniyor : /sim/joint_states");
    RCLCPP_INFO(this->get_logger(), "Yayınlıyor : /scaled_joint_trajectory_controller/joint_trajectory");
    RCLCPP_WARN(this->get_logger(), "Etkinleştirmek için:");
    RCLCPP_WARN(this->get_logger(), "  ros2 topic pub /sim_to_real/enable std_msgs/msg/Bool '{data: true}' --once");
  }

private:
  void urSimJointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (!bridge_enabled_) return;

    // Rate limiting
    auto now = this->now();
    if ((now - ur_last_update_time_).seconds() < 1.0 / update_rate_) return;
    ur_last_update_time_ = now;

    // Sim joint pozisyonlarını map'e al
    std::map<std::string, double> sim_positions;
    for (size_t i = 0; i < msg->name.size(); ++i)
      sim_positions[msg->name[i]] = msg->position[i];

    // Gerçek robot için sıralı pozisyon vektörü oluştur
    std::vector<double> target_positions;
    target_positions.reserve(ur_real_joint_order_.size());

    for (const auto& real_joint : ur_real_joint_order_) {
      // Real joint adına karşılık gelen sim joint adını bul
      std::string sim_joint;
      for (const auto& [s, r] : ur_joint_mapping_)
        if (r == real_joint) { sim_joint = s; break; }

      auto it = sim_positions.find(sim_joint);
      if (it == sim_positions.end()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "Sim joint bulunamadı: %s — komut atlandı", sim_joint.c_str());
        return;  // Eksik veriyle gerçek robota komut gönderme!
      }
      target_positions.push_back(it->second);
    }

    // Güvenlik: ani pozisyon sıçraması / hız kontrolü
    if (!ur_last_positions_.empty()) {
      for (size_t i = 0; i < target_positions.size(); ++i) {
        double delta    = std::abs(target_positions[i] - ur_last_positions_[i]);
        double velocity = delta * update_rate_;
        if (velocity > max_joint_velocity_) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
            "⚠️  Joint %zu hız aşımı: %.3f rad/s > %.3f rad/s — komut atlandı",
            i, velocity, max_joint_velocity_);
          return;
        }
      }
    }
    ur_last_positions_ = target_positions;

    // Trajectory mesajı oluştur
    trajectory_msgs::msg::JointTrajectory traj;
    traj.header.stamp = rclcpp::Time(0);
    traj.joint_names  = ur_real_joint_order_;

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions     = target_positions;
    point.velocities.resize(target_positions.size(), 0.0);
    point.accelerations.resize(target_positions.size(), 0.0);
    point.time_from_start = rclcpp::Duration::from_seconds(trajectory_time_);

    traj.points.push_back(point);
    ur_real_trajectory_pub_->publish(traj);

    RCLCPP_DEBUG(get_logger(), "UR10E trajectory yayınlandı");
  }

  // Parametreler
  double update_rate_;
  double trajectory_time_;
  double max_joint_velocity_;
  bool   bridge_enabled_;

  // UR10E
  std::map<std::string, std::string> ur_joint_mapping_;
  std::vector<std::string>           ur_real_joint_order_;
  std::vector<double>                ur_last_positions_;
  rclcpp::Time                       ur_last_update_time_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr    ur_sim_joint_state_sub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr ur_real_trajectory_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr             enable_sub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimToRealBridge>());
  rclcpp::shutdown();
  return 0;
}