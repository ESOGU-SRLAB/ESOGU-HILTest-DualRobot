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
    last_state_time_(this->now()),
    last_rate_report_(this->now()),
    ur_initial_sync_done_(false),
    kawasaki_initial_sync_done_(false)
  {
    // Parameters
    // THE CEILING IS THE SIM CONTROLLER'S UPDATE PERIOD, NOT THE DATA RATE.
    // Each published trajectory REPLACES the one the controller is running and restarts
    // it from the current position. gz_ros2_control steps the controllers once per
    // Gazebo physics step, and this world uses max_step_size 0.01, so the sim
    // controllers really update every 10 ms no matter what update_rate says in the
    // controller YAML. Publishing faster than that means every single controller update
    // finds a brand-new trajectory and samples it at t=0, where the target still equals
    // the current position -- the mirrored robot then does not move AT ALL. Measured at
    // 125 Hz (8 ms): all 14 sim joints frozen to 6 decimals over 1679 samples.
    //
    // 25 Hz gives the controller 4 updates per command. Smoothness does NOT come from
    // the command rate -- the controller interpolates between waypoints at its own
    // rate, and pass_through_velocities makes that interpolation C1 -- so a low command
    // rate with a long horizon looks better than a high one that keeps restarting.
    // Raise this only after lowering max_step_size in the world SDF.
    update_rate_ = this->declare_parameter<double>("update_rate", 25.0);
    // HORIZON MUST OUTLAST THE UPDATE PERIOD. Every published trajectory REPLACES the
    // one the controller is running, so if the horizon equals the period (the old
    // 10 Hz / 0.1 s pairing) the arm reaches the goal and STOPS, then waits for the next
    // message -- and any jitter in /joint_states becomes a visible hitch. At 2x the
    // period each command arrives while the previous is still mid-flight, so the
    // controller re-splines from a moving state and the motion stays continuous. The
    // cost is a tracking lag of one horizon (40 ms at 50 Hz), which is not visible.
    trajectory_time_ = this->declare_parameter<double>("trajectory_time", 2.5 / update_rate_);
    // Forward the MEASURED joint velocities instead of commanding zero at every
    // waypoint. This is the main reason the mirrored robots used to stutter: a
    // single-point trajectory whose end velocity is 0 tells joint_trajectory_controller
    // to arrive AT REST, so at 10 Hz the sim robot performed ten accelerate-decelerate
    // cycles per second. With the real velocity attached the controller blends through
    // the waypoint instead of stopping on it. Set false to get the old behaviour back.
    pass_through_velocities_ = this->declare_parameter<bool>("pass_through_velocities", true);
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

    // Subscriber to real robot joint states (UR + Kawasaki + AGV share /joint_states).
    // Depth 1: only the newest sample of each broadcaster matters, and a deeper queue
    // on a 500 Hz topic just adds latency when a cycle runs long.
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states",
      rclcpp::QoS(1).best_effort(),
      std::bind(&RealToSimBridge::jointStateCallback, this, std::placeholders::_1));

    // Fixed-rate publishing, decoupled from the input (see jointStateCallback).
    // Integer nanoseconds rather than a floating-point seconds duration: same result,
    // but nothing about the unit is left to template deduction at the call site.
    const auto period = std::chrono::nanoseconds(
      static_cast<int64_t>(1.0e9 / update_rate_));
    publish_timer_ = this->create_wall_timer(
      period, std::bind(&RealToSimBridge::publishTimerCallback, this));

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
    RCLCPP_INFO(this->get_logger(), "  Velocity pass-through: %s",
                pass_through_velocities_ ? "on" : "off (waypoints commanded at rest)");
    if (trajectory_time_ <= 1.0 / update_rate_) {
      RCLCPP_WARN(this->get_logger(),
                  "trajectory_time (%.3f s) is not longer than the update period "
                  "(%.3f s): every command will finish before the next arrives and the "
                  "mirrored robots will move in steps. Use about 2x the period.",
                  trajectory_time_, 1.0 / update_rate_);
    }
    // Hard guard against the failure that froze the twin completely: publishing faster
    // than the sim controllers can step means they never get past t=0 of any command.
    // The sim controller period equals the Gazebo physics step (max_step_size), 10 ms
    // in this cell, so anything above ~50 Hz is already suspect.
    if (update_rate_ > 50.0) {
      RCLCPP_WARN(this->get_logger(),
                  "update_rate %.1f Hz is %.1f ms per command. The sim controllers step "
                  "once per Gazebo physics step (max_step_size, 10 ms in ifarlab.sdf), "
                  "so each command may be REPLACED before the controller ever advances "
                  "it and the mirrored robots can stop moving entirely. Lower this, or "
                  "lower max_step_size in the world first.",
                  update_rate_, 1000.0 / update_rate_);
    }
    RCLCPP_INFO(this->get_logger(), "  Initial sync time: %.1f s (smooth first move)", initial_sync_time_);
    RCLCPP_INFO(this->get_logger(), "  Listening to: /joint_states");
    RCLCPP_INFO(this->get_logger(), "  Publishing UR to:       /sim/sim_scaled_joint_trajectory_controller/joint_trajectory");
    RCLCPP_INFO(this->get_logger(), "  Publishing Gripper to:  /sim/sim_gripper_controller/joint_trajectory");
    RCLCPP_INFO(this->get_logger(), "  Publishing Kawasaki to: /sim/kawasaki_controller/joint_trajectory");
  }

private:
  // ==================== JOINT STATE CALLBACK ====================
  //
  // RECORD EVERY MESSAGE, PUBLISH ON A TIMER. This used to rate-limit here and publish
  // straight from the callback, which was the real cause of the residual stutter.
  // /joint_states is fed by TWO broadcasters and NO message carries both robots
  // (measured on the running cell):
  //     joint_state_broadcaster           500 Hz, 7 joints (UR + linear axis)
  //     /kawasaki/..._broadcaster         100 Hz, 9 joints (joint1..6 + wheels)
  // Rate-limiting the mixed stream accepts one message per period WHOEVER sent it, so
  // roughly 5 of every 6 accepted samples were UR-only. The Kawasaki bridge then found
  // none of its joints and returned early: at a nominal 50 Hz it was really updating
  // about 8 times a second, at random intervals. The UR fared better but still measured
  // 39 Hz with gaps up to 102 ms.
  //
  // So the callback now only CACHES (the union of everything seen so far, per joint) and
  // a fixed-rate timer does the publishing. Each arm then gets evenly spaced commands at
  // exactly update_rate regardless of which broadcaster happened to talk last.
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    last_state_time_ = this->now();

    // Bounded by position.size(), not name.size(): a JointState is allowed to carry
    // names without positions (some aggregators do), and indexing past the end here
    // would take the whole bridge down.
    const size_t n_pos = std::min(msg->name.size(), msg->position.size());
    for (size_t i = 0; i < n_pos; ++i) {
      latest_positions_[msg->name[i]] = msg->position[i];
    }
    if (msg->position.size() < msg->name.size()) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
                           "/joint_states carries %zu names but only %zu positions; "
                           "the extra joints are ignored.",
                           msg->name.size(), msg->position.size());
    }

    // Velocities are OPTIONAL: only used when the publisher supplies one per joint.
    // Without them every waypoint is commanded at rest and the mirrored robot stutters.
    if (pass_through_velocities_) {
      const size_t n_vel = std::min(msg->name.size(), msg->velocity.size());
      for (size_t i = 0; i < n_vel; ++i) {
        latest_velocities_[msg->name[i]] = msg->velocity[i];
      }
      if (msg->velocity.empty()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
                             "pass_through_velocities is on but /joint_states carries no "
                             "velocities; falling back to zero-velocity waypoints, which "
                             "makes the mirrored motion step-and-stop.");
      }
    }
  }

  // ==================== FIXED-RATE PUBLISH ====================
  void publishTimerCallback()
  {
    std::map<std::string, double> positions;
    std::map<std::string, double> velocities;
    rclcpp::Time last_seen;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (latest_positions_.empty()) {
        return;                       // nothing has arrived yet
      }
      positions = latest_positions_;  // copied under the lock, published outside it
      velocities = latest_velocities_;
      last_seen = last_state_time_;
    }

    // The cache is deliberately never cleared, so a driver that stops publishing leaves
    // the sim holding its last pose rather than jumping. Say so once in a while, since
    // a frozen twin otherwise looks like a bridge fault.
    if ((this->now() - last_seen).seconds() > 1.0) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "No /joint_states for %.1f s; still mirroring the last known "
                           "pose.", (this->now() - last_seen).seconds());
    }

    publishUrTrajectory(positions, velocities);
    publishGripperTrajectory(positions, velocities);
    publishKawasakiTrajectory(positions, velocities);

    // Report the rate actually achieved. The configured rate and the delivered one
    // have already diverged once during development, and a mirror that silently runs
    // at the wrong rate looks exactly like a stuttering one -- so the check stays.
    // What does NOT stay is the periodic INFO: on a shared terminal it was a line
    // every five seconds for the whole session. Report once when the rate first
    // settles, then stay quiet unless it actually drifts.
    ++publish_count_;
    const auto now = this->now();
    const double window = (now - last_rate_report_).seconds();
    if (window >= 5.0) {
      const double achieved = publish_count_ / window;
      if (!rate_reported_) {
        RCLCPP_INFO(this->get_logger(), "Mirroring at %.1f Hz (configured %.1f Hz).",
                    achieved, update_rate_);
        rate_reported_ = true;
      } else if (std::abs(achieved - update_rate_) > 0.2 * update_rate_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 30000,
                             "Mirror rate drifted: %.1f Hz achieved vs %.1f Hz "
                             "configured.", achieved, update_rate_);
      }
      publish_count_ = 0;
      last_rate_report_ = now;
    }
  }

  // Measured velocity for `real_joint_name`, or 0.0 when unknown. Also returns 0.0
  // during the initial sync: that hop is a long deliberate catch-up move and must
  // finish at rest.
  double velocityFor(const std::map<std::string, double>& vels,
                     const std::string& real_joint_name, bool initial_sync) const
  {
    if (initial_sync) {
      return 0.0;
    }
    auto it = vels.find(real_joint_name);
    return (it == vels.end()) ? 0.0 : it->second;
  }

  // ==================== UR10E TRAJECTORY PUBLISH ====================
  void publishUrTrajectory(const std::map<std::string, double>& real_joint_positions,
                           const std::map<std::string, double>& real_joint_velocities)
  {
    // Collect positions; skip entirely if no UR joints found
    std::vector<double> sim_joint_positions;
    std::vector<std::string> found_sim_joints;
    std::vector<std::string> found_real_joints;
    sim_joint_positions.reserve(ur_sim_joint_order_.size());
    found_sim_joints.reserve(ur_sim_joint_order_.size());
    found_real_joints.reserve(ur_sim_joint_order_.size());

    for (const auto& sim_joint_name : ur_sim_joint_order_) {
      const std::string& real_joint_name = ur_reverse_mapping_.at(sim_joint_name);
      auto it = real_joint_positions.find(real_joint_name);
      if (it != real_joint_positions.end()) {
        found_sim_joints.push_back(sim_joint_name);
        found_real_joints.push_back(real_joint_name);
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
    const bool initial_sync = !ur_initial_sync_done_;
    if (initial_sync) {
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
    point.velocities.reserve(found_real_joints.size());
    for (const auto& real_joint_name : found_real_joints) {
      point.velocities.push_back(
        velocityFor(real_joint_velocities, real_joint_name, initial_sync));
    }
    // Accelerations stay 0: the real driver does not publish them, and the controller
    // only needs position+velocity to build a continuous spline through the waypoint.
    point.accelerations.resize(sim_joint_positions.size(), 0.0);
    point.time_from_start = rclcpp::Duration::from_seconds(traj_time);

    traj_msg.points.push_back(point);
    ur_trajectory_pub_->publish(traj_msg);

    RCLCPP_DEBUG(this->get_logger(), "Published UR trajectory with %zu joints", found_sim_joints.size());
  }

  // ==================== GRIPPER TRAJECTORY PUBLISH ====================
  void publishGripperTrajectory(const std::map<std::string, double>& real_joint_positions,
                                const std::map<std::string, double>& real_joint_velocities)
  {
    auto gripper_it = real_joint_positions.find(gripper_real_joint_name_);
    if (gripper_it != real_joint_positions.end()) {
      trajectory_msgs::msg::JointTrajectory gripper_traj;
      gripper_traj.header.stamp = rclcpp::Time(0);
      gripper_traj.joint_names = {gripper_sim_joint_name_};

      trajectory_msgs::msg::JointTrajectoryPoint gp;
      gp.positions     = {gripper_it->second};
      gp.velocities    = {velocityFor(real_joint_velocities, gripper_real_joint_name_, false)};
      gp.accelerations = {0.0};
      gp.time_from_start = rclcpp::Duration::from_seconds(trajectory_time_);

      gripper_traj.points.push_back(gp);
      gripper_trajectory_pub_->publish(gripper_traj);

      RCLCPP_DEBUG(this->get_logger(), "Published gripper trajectory: %.4f", gripper_it->second);
    }
  }

  // ==================== KAWASAKI + AGV TRAJECTORY PUBLISH ====================
  void publishKawasakiTrajectory(const std::map<std::string, double>& real_joint_positions,
                                 const std::map<std::string, double>& real_joint_velocities)
  {
    // Collect positions; skip entirely if no Kawasaki/AGV joints found
    std::vector<double> sim_joint_positions;
    std::vector<std::string> found_sim_joints;
    std::vector<std::string> found_real_joints;
    sim_joint_positions.reserve(kawasaki_sim_joint_order_.size());
    found_sim_joints.reserve(kawasaki_sim_joint_order_.size());
    found_real_joints.reserve(kawasaki_sim_joint_order_.size());

    for (const auto& sim_joint_name : kawasaki_sim_joint_order_) {
      const std::string& real_joint_name = kawasaki_reverse_mapping_.at(sim_joint_name);
      auto it = real_joint_positions.find(real_joint_name);
      if (it != real_joint_positions.end()) {
        found_sim_joints.push_back(sim_joint_name);
        found_real_joints.push_back(real_joint_name);
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
    const bool initial_sync = !kawasaki_initial_sync_done_;
    if (initial_sync) {
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
    point.velocities.reserve(found_real_joints.size());
    for (const auto& real_joint_name : found_real_joints) {
      point.velocities.push_back(
        velocityFor(real_joint_velocities, real_joint_name, initial_sync));
    }
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
  bool pass_through_velocities_;
  
  // Shared
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::Time last_update_time_;
  // Union of every joint seen so far. Held across messages precisely because no single
  // /joint_states message carries both robots.
  std::mutex state_mutex_;
  std::map<std::string, double> latest_positions_;
  std::map<std::string, double> latest_velocities_;
  rclcpp::Time last_state_time_;
  rclcpp::Time last_rate_report_;
  uint64_t publish_count_{0};
  bool rate_reported_{false};

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
