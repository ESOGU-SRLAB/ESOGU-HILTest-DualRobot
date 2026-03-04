#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "KawasakiRS005LRobot.h"
#include "SIRLinConnection.h"
#include "SIRTypeDefs.h"

#include <Eigen/StdVector>
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(SIRMatrix)

#include <vector>
#include <cmath>
#include <chrono>
#include <thread>
#include "khi_ota_comm_valu3s.h"

float deg2rad(float degree) {
  return degree * M_PI / 180.0f;
}

OTA_STATE state = OTA_STATE_WAIT;

using namespace std::chrono_literals;


class AGVSequenceMoveNode : public rclcpp::Node {
public:
  AGVSequenceMoveNode()
      : Node("agv_sequence_move"), current_target_index_(0), waiting_for_arrival_(false), sequence_done_(false), current_position_(0.0), agv_is_idle_(true), arm_moving_(false), arm_sequence_done_(false) {
    // Hedefler ve parametreler sonraki adımda güncellenecek
    targets_ = {0.65, 1.15, 1.65};
    tolerance_ = this->declare_parameter<double>("tolerance", 0.1);
    wait_time_ = this->declare_parameter<double>("wait_time", 2.0);
    robot_ip_   = this->declare_parameter<std::string>("robot_ip", "192.168.3.7");
    robot_port_ = this->declare_parameter<int>("robot_port", 11111);

    // Topic names are already matching bridge_node.py
    goal_pub_ = this->create_publisher<std_msgs::msg::Float64>("/agv/goal_position", 10);
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/agv/odom", 10, std::bind(&AGVSequenceMoveNode::odomCallback, this, std::placeholders::_1));
    status_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/agv/mobile_status", 10, std::bind(&AGVSequenceMoveNode::statusCallback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
        500ms, std::bind(&AGVSequenceMoveNode::controlLoop, this));

    RCLCPP_INFO(this->get_logger(), "AGV Sequence Move Node tetiklendi.");
    RCLCPP_INFO(this->get_logger(), "Tüm node'ların ayağa kalkması için 3 sn bekleniyor...");

    startup_timer_ = this->create_wall_timer(
        3s, std::bind(&AGVSequenceMoveNode::startSequence, this));
  }

private:
  void statusCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    std::string current_status = msg->data;

    // Durum değiştiğinde bilgi vermek için (sürekli yazdırmamak adına)
    if (current_status != last_agv_status_) {
      if (current_status == "Moving") {
        RCLCPP_INFO(this->get_logger(), "AGV hareket halinde (Moving). Robot kolu komut alımına KAPALI.");
        agv_is_idle_ = false;
        // TODO: robot_arm_ready_ = false; (İlerleyen çalışmalarda kullanılacak)
      } else if (current_status == "Idle") {
        RCLCPP_INFO(this->get_logger(), "AGV beklemede (Idle). Robot kolu komut almaya HAZIR.");
        agv_is_idle_ = true;
        // TODO: robot_arm_ready_ = true;  (İlerleyen çalışmalarda kullanılacak)
      }
      last_agv_status_ = current_status;
    }

    if (state == OTA_STATE_WAIT && current_status == "start") {
      RCLCPP_INFO(this->get_logger(), "OTA has arrived, OPERATE moduna geçiliyor.");
      state = OTA_STATE_OPERATE;
    }
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    current_position_ = msg->pose.pose.position.x;
  }

  void startSequence()
  {
    startup_timer_->cancel();
    RCLCPP_INFO(this->get_logger(), "=== AGV Seq Başlıyor ===");
    RCLCPP_INFO(this->get_logger(), "Mevcut konum: x = %.4f m", current_position_);
    state = OTA_STATE_OPERATE; // Durumu operasyona çek, ki controlLoop çalışabilsin!
    sendNextTarget();
  }


  void controlLoop() {
    switch (state) {
      case OTA_STATE_WAIT:
        // AGV'nin başlatılması için "başlat" komutu bekleniyor
        // Burada bir topic veya başka bir tetikleyici eklenebilir
        break;
      case OTA_STATE_OPERATE:
        if (sequence_done_ || !waiting_for_arrival_) {
          break;
        }

        if (arm_moving_) {
          break; // Robot kolu hareket ediyor, controlLoop'ta bekle
        }

        if (arm_sequence_done_) {
          arm_sequence_done_ = false;
          waiting_for_arrival_ = false;
          current_target_index_++;
          if (current_target_index_ >= targets_.size()) {
            RCLCPP_INFO(this->get_logger(), "=== TÜM HEDEFLER BAŞARIYLA TAMAMLANDI ===");
            sequence_done_ = true;
            state = OTA_STATE_FINISH;
            break;
          }
          RCLCPP_INFO(this->get_logger(), "Sonraki hedefe geçmeden %.1f saniye bekleniyor...", wait_time_);
          wait_timer_ = this->create_wall_timer(
              std::chrono::duration<double>(wait_time_),
              std::bind(&AGVSequenceMoveNode::sendNextTarget, this));
          break;
        }

        {
          double diff = std::abs(targets_[current_target_index_] - current_position_);
          // Hedefe varıldığını hem mesafe hem de 'Idle' durumuyla doğruluyoruz.
          if (diff <= tolerance_ && agv_is_idle_) {
            RCLCPP_INFO(this->get_logger(),
                        "--> Hedef %zu'e ULAŞILDI ve AGV durdu (Idle)! (Mevcut X: %.4f m, Beklenen: %.4f m)",
                        current_target_index_ + 1, current_position_, targets_[current_target_index_]);
            
            RCLCPP_INFO(this->get_logger(), "Robot kolu W1 ve W2 hedeflerine doğru ilerliyor...");
            arm_moving_ = true;
            arm_sequence_done_ = false;
            std::thread(&AGVSequenceMoveNode::runArmSequence, this).detach();
          }
        }
        break;
      case OTA_STATE_FINISH:
        RCLCPP_INFO(this->get_logger(), "AGV görevi tamamlandı, bekleme moduna geçiliyor.");
        state = OTA_STATE_WAIT;
        break;
      default:
        break;
    }
  }

  void sendNextTarget() {
    if (wait_timer_) {
      wait_timer_->cancel();
    }
    if (current_target_index_ >= targets_.size()) {
      return;
    }
    double target = targets_[current_target_index_];
    RCLCPP_INFO(this->get_logger(),
                "[*] Hedef %zu gönderiliyor: x = %.4f m",
                current_target_index_ + 1, target);
    std_msgs::msg::Float64 msg;
    msg.data = target;
    goal_pub_->publish(msg);
    waiting_for_arrival_ = true;
  }

  void runArmSequence()
  {
    SIRLogger *logger = new SIRLogger("log.txt", SIRLOGLEVEL::LOGLEVEL_DEBUG);
    SIRConnection *con = new SIRLinConnection(logger, robot_ip_.c_str(), robot_port_);

    KawasakiRS005LRobot robot(con, logger, nullptr, MPT_JOINT, MT_P2P);

    if (!robot.Connect()) {
      RCLCPP_ERROR(this->get_logger(), "Could not connect to arm at %s:%d. Skipping arm movement.", 
                   robot_ip_.c_str(), robot_port_);
      delete con;
      delete logger;
      arm_moving_ = false;
      arm_sequence_done_ = true;
      return;
    }
    
    con->setBlockingMode(0);
    robot.setWaitForCommand(2000);

    std::this_thread::sleep_for(1s);

    // Waypoint'leri hazırla
    std::vector<SIRMatrix> waypoints;

    // 1 - 121.801, 12.790, -26.167, -69.391, 109.266, 22.759
    SIRMatrix point1(6, 1);
    point1 << 121.801, 12.790, -26.167, -69.391, 109.266, 22.759;
    waypoints.push_back(point1);
    
    // 2 - 80.217, 2.237, -39.290, -102.203, 82.983, 18.407
    SIRMatrix point2(6, 1);
    point2 << 80.217, 2.237, -39.290, -102.203, 82.983, 18.407;
    waypoints.push_back(point2);

    RCLCPP_INFO(this->get_logger(), "Moving robot through %zu waypoints...", waypoints.size());

    // Tüm noktaları robota ekle
    for (size_t i = 0; i < waypoints.size(); i++) {
      int packet_id = robot.add(waypoints[i]);
      RCLCPP_INFO(this->get_logger(), "Added waypoint %zu, packet ID: %d", i+1, packet_id);
    }

    if (!robot.move()) {
      RCLCPP_ERROR(this->get_logger(), "Arm robot.move() failed!");
      robot.close();
      delete con;
      delete logger;
      arm_moving_ = false;
      arm_sequence_done_ = true;
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Arm moving to W1 and W2...");
    
    while (robot.getStatus() != RS_STOP) {
      std::this_thread::sleep_for(100ms);
    }

    RCLCPP_INFO(this->get_logger(), "Arm reached W2.");
    
    robot.close();
    delete con;
    delete logger;

    // Signal main thread to continue AGV sequence
    arm_moving_ = false;
    arm_sequence_done_ = true;
  }

  std::vector<double> targets_;
  size_t current_target_index_;
  bool waiting_for_arrival_;
  bool sequence_done_;
  double current_position_;
  double tolerance_;
  double wait_time_;
  bool agv_is_idle_;
  bool arm_moving_;
  bool arm_sequence_done_;
  std::string robot_ip_;
  int robot_port_;

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr goal_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr startup_timer_;
  rclcpp::TimerBase::SharedPtr wait_timer_;
  std::string last_agv_status_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AGVSequenceMoveNode>();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
