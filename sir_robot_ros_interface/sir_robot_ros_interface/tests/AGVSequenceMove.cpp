// AGVSequenceMove.cpp
// AGV'yi sırasıyla 1.0, 0.5, 1.5 koordinatlarına hareket ettiren koordinatör node.

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <vector>
#include <cmath>
#include <chrono>

using namespace std::chrono_literals;

class AGVSequenceMoveNode : public rclcpp::Node
{
public:
  AGVSequenceMoveNode()
  : Node("agv_sequence_move"),
    current_target_index_(0),
    waiting_for_arrival_(false),
    sequence_done_(false),
    current_position_(0.0)
  {
    // C++ action_server3.cpp içindeki hedefe yaklaşma durma eşiği 0.005'tir.
    // Toleransı biraz daha yüksek (0.02) vererek ROS2 tarafının kesin anlamasını sağlıyoruz.
    targets_ = {1.0, 0.5, 1.5};
    tolerance_ = this->declare_parameter<double>("tolerance", 0.02);
    wait_time_ = this->declare_parameter<double>("wait_time", 2.0);

    goal_pub_ = this->create_publisher<std_msgs::msg::Float64>("/agv/goal_position", 10);
    
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/agv/odom", 10, std::bind(&AGVSequenceMoveNode::odomCallback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
      500ms, std::bind(&AGVSequenceMoveNode::controlLoop, this));

    RCLCPP_INFO(this->get_logger(), "AGV Sequence Move Node tetiklendi.");
    RCLCPP_INFO(this->get_logger(), "Tüm node'ların ayağa kalkması için 3 sn bekleniyor...");

    startup_timer_ = this->create_wall_timer(
      3s, std::bind(&AGVSequenceMoveNode::startSequence, this));
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    current_position_ = msg->pose.pose.position.x;
  }

  void startSequence()
  {
    startup_timer_->cancel();
    RCLCPP_INFO(this->get_logger(), "=== AGV Seq Başlıyor ===");
    RCLCPP_INFO(this->get_logger(), "Mevcut konum: x = %.4f m", current_position_);
    sendNextTarget();
  }

  void controlLoop()
  {
    if (sequence_done_ || !waiting_for_arrival_) {
      return;
    }

    double diff = std::abs(targets_[current_target_index_] - current_position_);

    // Eğer tolerans payı içine girersek hedefe ulaştık sayıyoruz
    if (diff <= tolerance_) {
      RCLCPP_INFO(this->get_logger(),
        "--> Hedef %zu'e ULAŞILDI! (Mevcut X: %.4f m, Beklenen: %.4f m)",
        current_target_index_ + 1, current_position_, targets_[current_target_index_]);

      waiting_for_arrival_ = false;
      current_target_index_++;

      if (current_target_index_ >= targets_.size()) {
        RCLCPP_INFO(this->get_logger(), "=== TÜM HEDEFLER BAŞARIYLA TAMAMLANDI ===");
        sequence_done_ = true;
        return;
      }

      RCLCPP_INFO(this->get_logger(), "Sonraki hedefe geçmeden %.1f saniye bekleniyor...", wait_time_);
      wait_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(wait_time_),
        std::bind(&AGVSequenceMoveNode::sendNextTarget, this));
    }
  }

  void sendNextTarget()
  {
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

  std::vector<double> targets_;
  size_t current_target_index_;
  bool waiting_for_arrival_;
  bool sequence_done_;
  double current_position_;
  double tolerance_;
  double wait_time_;

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr goal_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr startup_timer_;
  rclcpp::TimerBase::SharedPtr wait_timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AGVSequenceMoveNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
