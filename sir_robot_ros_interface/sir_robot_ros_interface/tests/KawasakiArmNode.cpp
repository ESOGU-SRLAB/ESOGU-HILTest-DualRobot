#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

#include "KawasakiRS005LRobot.h"
#include "SIRLinConnection.h"
#include "SIRTypeDefs.h"

#include <Eigen/StdVector>
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(SIRMatrix)

#include <vector>
#include <array>
#include <chrono>
#include <thread>
#include <mutex>
#include <atomic>
#include <cmath>
#include <map>
#include "khi_ota_comm_valu3s.h"
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>

using namespace std::chrono_literals;

class KawasakiArmNode : public rclcpp::Node {
public:
  KawasakiArmNode() : Node("kawasaki_arm_executor"), arm_moving_(false), move_pending_(false), monitor_running_(false) {
    robot_ip_   = this->declare_parameter<std::string>("robot_ip", "192.168.3.7");
    robot_port_ = this->declare_parameter<int>("robot_port", 11111);

    status_pub_ = this->create_publisher<std_msgs::msg::String>("/kawa/status", 10);
    
    target_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/kawa/target_joints", 10, std::bind(&KawasakiArmNode::targetCallback, this, std::placeholders::_1));

    // === Sim Sync: real Kawasaki + AGV → Gazebo kawasaki_controller ===
    // Sim controller joint order (whole_cell_sim_controllers.yaml):
    //   [0] world_to_agv         ← /agv/odom  position.x
    //   [1] sim_kawasaki_joint1  ← robot direct read (rad)
    //   ...  [6] sim_kawasaki_joint6
    sim_kawa_traj_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/sim/kawasaki_controller/joint_trajectory",
      rclcpp::QoS(10).best_effort());

    // /joint_states → robot_state_publisher TF ağacı (RViz)
    // /kawasaki/joint_states → diğer node'lar için
    joint_states_pub_   = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    kawa_js_pub_        = this->create_publisher<sensor_msgs::msg::JointState>("/kawasaki/joint_states", 10);

    // AGV odom → world_to_agv joint değeri
    agv_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/agv/odom",
      rclcpp::QoS(10).best_effort(),
      std::bind(&KawasakiArmNode::agvOdomCallback, this, std::placeholders::_1));

    // 10 Hz timer: combined [world_to_agv, sim_kawasaki_joint1..6] → sim
    sim_sync_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&KawasakiArmNode::simSyncTimerCallback, this));

    kawa_joint_positions_.fill(0.0);
    agv_position_ = 0.0;

    // Kalıcı robot izleme thread'ini başlat
    // Bu thread robot'a bağlanır, eklem açılarını okur ve hem sim sync hem de
    // /joint_states yayınını doğrudan yönetir — dışarıdaki hiçbir node'a bağımlı değil.
    monitor_running_.store(true);
    monitor_thread_ = std::thread(&KawasakiArmNode::jointMonitorThread, this);

    RCLCPP_INFO(this->get_logger(), "Kawasaki Arm Executor Node started. Waiting for target joints...");
    RCLCPP_INFO(this->get_logger(), "Joint monitor thread started: connecting to %s:%d",
                robot_ip_.c_str(), robot_port_);
    publishStatus("Idle");
  }

  ~KawasakiArmNode() {
    monitor_running_.store(false);
    if (monitor_thread_.joinable()) monitor_thread_.join();
  }

private:
  // ================================================================
  // Kalıcı robot izleme thread'i
  // ROS2KawasakiRobotJointStatePublisher'ın yaptığı her şeyi
  // doğrudan bu node içinde yapar — dışarıdaki node'a bağımlılık yok.
  // ================================================================
  void jointMonitorThread()
  {
    static const std::vector<std::string> joint_names =
      {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};

    auto d2r = [](double deg) { return deg * M_PI / 180.0; };

    while (monitor_running_.load() && rclcpp::ok())
    {
      // Bağlantı kurulmadıysa veya kopmuşsa yeniden dene
      // MPT_JOINT: hem getJointPose okuma hem de add/move hareketi bu modda çalışır.
      // Bir BAĞLANTI — hem izleme hem hareket bu thread'den yönetilir.
      auto logger = std::make_unique<SIRLogger>("kawa_monitor.txt", SIRLOGLEVEL::LOGLEVEL_DEBUG);
      auto con    = std::make_unique<SIRLinConnection>(logger.get(), robot_ip_.c_str(), robot_port_);
      KawasakiRS005LRobot robot(con.get(), logger.get(), nullptr, MPT_JOINT, MT_P2P);

      if (!robot.Connect()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "[Monitor] Robot bağlantısı kurulamadı, 3s sonra tekrar...");
        std::this_thread::sleep_for(3000ms);
        continue;
      }

      con->setBlockingMode(0);
      robot.setWaitForCommand(2000);
      RCLCPP_INFO(this->get_logger(), "[Monitor] Robot bağlantısı kuruldu (%s:%d)",
                  robot_ip_.c_str(), robot_port_);

      SIRMatrix pose(6, 1);
      while (monitor_running_.load() && rclcpp::ok())
      {
        if (!robot.getJointPose(&pose)) {
          RCLCPP_WARN(this->get_logger(), "[Monitor] getJointPose başarısız, yeniden bağlanıyor...");
          break;  // dış döngüye dön → yeniden bağlan
        }

        // kawa_joint_positions_ güncelle (simSyncTimerCallback tarafından okunur)
        {
          std::lock_guard<std::mutex> lk(sync_mutex_);
          for (int i = 0; i < 6; ++i)
            kawa_joint_positions_[i] = d2r(pose(i, 0));
        }

        // /joint_states → robot_state_publisher TF ağacı (RViz)
        // /kawasaki/joint_states → diğer node'lar için
        sensor_msgs::msg::JointState js;
        js.header.stamp    = this->now();
        js.header.frame_id = "base_link";
        js.name            = joint_names;
        js.position.resize(6);
        js.velocity.resize(6, 0.0);
        js.effort.resize(6, 0.0);
        for (int i = 0; i < 6; ++i)
          js.position[i] = d2r(pose(i, 0));

        joint_states_pub_->publish(js);
        kawa_js_pub_->publish(js);

        // ── Bekleyen hareket komutu varsa AYNI bağlantıyla çalıştır ──
        // Yeni bağlantı açılmaz → "Invalid packet type" hatası olmaz.
        if (move_pending_.load()) {
          move_pending_.store(false);
          std::vector<double> targets;
          {
            std::lock_guard<std::mutex> lk(move_mutex_);
            targets = move_pending_targets_;
          }

          SIRMatrix point(6, 1);
          point << targets[0], targets[1], targets[2],
                   targets[3], targets[4], targets[5];

          RCLCPP_INFO(this->get_logger(),
                      "Executing Kawasaki move to [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
                      point(0), point(1), point(2), point(3), point(4), point(5));

          int packet_id = robot.add(point);
          RCLCPP_INFO(this->get_logger(), "Added waypoint, packet ID: %d", packet_id);

          if (!robot.move()) {
            RCLCPP_ERROR(this->get_logger(), "Arm robot.move() failed!");
            arm_moving_ = false;
            publishStatus("Error: Move Failed");
            break;  // bağlantıyı yeniden kur
          }

          // Hareket sırasında da joint pose oku ve sim/RViz'i güncelle
          // (ROS2KawasakiRobotMove ile aynı davranış)
          while (robot.getStatus() != RS_STOP &&
                 monitor_running_.load() && rclcpp::ok()) {
            if (robot.getJointPose(&pose)) {
              {
                std::lock_guard<std::mutex> lk(sync_mutex_);
                for (int i = 0; i < 6; ++i)
                  kawa_joint_positions_[i] = d2r(pose(i, 0));
              }
              sensor_msgs::msg::JointState js_mv;
              js_mv.header.stamp    = this->now();
              js_mv.header.frame_id = "base_link";
              js_mv.name            = joint_names;
              js_mv.position.resize(6);
              js_mv.velocity.resize(6, 0.0);
              js_mv.effort.resize(6, 0.0);
              for (int i = 0; i < 6; ++i)
                js_mv.position[i] = d2r(pose(i, 0));
              joint_states_pub_->publish(js_mv);
              kawa_js_pub_->publish(js_mv);
            }
            std::this_thread::sleep_for(100ms);
          }

          RCLCPP_INFO(this->get_logger(), "Arm reached target successfully.");
          arm_moving_ = false;
          publishStatus("Done");
        }

        std::this_thread::sleep_for(100ms);  // 10 Hz
      }
    }
  }

  void targetCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (arm_moving_) {
        RCLCPP_WARN(this->get_logger(), "Arm is currently moving. Ignoring new target geometry.");
        return;
    }

    if (msg->data.size() != 6) {
        RCLCPP_ERROR(this->get_logger(), "Received target with %zu joints, but expected 6.", msg->data.size());
        return;
    }

    arm_moving_ = true;
    publishStatus("Busy");
    {
      std::lock_guard<std::mutex> lk(move_mutex_);
      move_pending_targets_ = std::vector<double>(msg->data.begin(), msg->data.end());
    }
    // Monitor thread AYNI bağlantıyla hareketi çalıştıracak
    move_pending_.store(true);
  }

  void agvOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(sync_mutex_);
    agv_position_ = msg->pose.pose.position.x;
  }

  void simSyncTimerCallback()
  {
    static const std::vector<std::string> sim_joint_names = {
      "world_to_agv",
      "sim_kawasaki_joint1", "sim_kawasaki_joint2", "sim_kawasaki_joint3",
      "sim_kawasaki_joint4", "sim_kawasaki_joint5", "sim_kawasaki_joint6"
    };

    double agv_pos;
    std::array<double, 6> kawa_pos;
    {
      std::lock_guard<std::mutex> lock(sync_mutex_);
      agv_pos  = agv_position_;
      kawa_pos = kawa_joint_positions_;
    }

    trajectory_msgs::msg::JointTrajectory traj;
    traj.header.stamp = rclcpp::Time(0);  // execute immediately
    traj.joint_names  = sim_joint_names;

    trajectory_msgs::msg::JointTrajectoryPoint pt;
    pt.positions.resize(7);
    pt.velocities.resize(7, 0.0);
    pt.accelerations.resize(7, 0.0);
    pt.positions[0] = agv_pos;
    for (int i = 0; i < 6; ++i) {
      pt.positions[i + 1] = kawa_pos[i];
    }
    pt.time_from_start = rclcpp::Duration::from_seconds(0.1);
    traj.points.push_back(pt);

    sim_kawa_traj_pub_->publish(traj);
  }

  void publishStatus(const std::string& state) {
      std_msgs::msg::String msg;
      msg.data = state;
      status_pub_->publish(msg);
  }

  bool arm_moving_;
  std::string robot_ip_;
  int robot_port_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr target_sub_;

  // === Sim Sync members ===
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr sim_kawa_traj_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr kawa_js_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr agv_odom_sub_;
  rclcpp::TimerBase::SharedPtr sim_sync_timer_;
  std::array<double, 6> kawa_joint_positions_;
  double agv_position_;
  std::mutex sync_mutex_;

  // === Hareket komutu kuyruğu (monitor thread tarafından tüketilir) ===
  std::atomic<bool> move_pending_;
  std::vector<double> move_pending_targets_;
  std::mutex move_mutex_;

  // === Kalıcı izleme thread'i ===
  std::thread monitor_thread_;
  std::atomic<bool> monitor_running_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<KawasakiArmNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
