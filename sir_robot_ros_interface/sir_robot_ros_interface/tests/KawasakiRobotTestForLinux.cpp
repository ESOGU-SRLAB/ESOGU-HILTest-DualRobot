// khi_ota_comm_node.cpp  (ROS2 Humble)

#include <rclcpp/rclcpp.hpp>

#include "agv_msgs/msg/task_com.hpp"   // ROS2 msg: agv_msgs::msg::TaskCom

#include "KawasakiRS005LRobot.h"
#include "SIRLinConnection.h"
#include "SIRTypeDefs.h"
#include "khi_ota_comm.h"

#include <Eigen/StdVector>
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(SIRMatrix)

#include <iostream>
#include <vector>
#include <string>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>

using namespace std::chrono_literals;


static std::vector<SIRMatrix> pathPlanner(const std::string & boxType)
{
  std::vector<SIRMatrix> path;

  SIRMatrix P_Top_Box(6,1);
  SIRMatrix P_Top_Vehicle(6,1);
  SIRMatrix P_Home(6,1);

  SIRMatrix P_Grip(6,1);
  SIRMatrix P_Before_Grip(6,1);
  SIRMatrix P_Before_Drop(6,1);
  SIRMatrix P_Drop(6,1);

  SIRMatrix P_Grip2(6,1);
  SIRMatrix P_Before_Grip2(6,1);
  SIRMatrix P_Before_Drop2(6,1);
  SIRMatrix P_Drop2(6,1);

  P_Top_Box     << -694.890,  82.273, 313.291, 85.340, 178.828, 153.062;
  P_Top_Vehicle <<   28.606, 700.716, 313.287, 85.366, 178.828, 153.090;
  P_Home        <<  -14.793, 291.511, 479.387, 85.348, 178.828, 153.068;

  P_Grip        << -693.328, 201.249, -91.597, 85.320, 178.827, 153.041;
  P_Before_Grip << -693.328, 201.249, 275.292, 85.320, 178.827, 153.041;

  P_Before_Drop <<  330.606, 700.716, 275.287, 85.366, 178.828, 153.090;
  P_Drop        <<  330.606, 700.716,-100.445, 85.271, 178.828, 152.991;

  P_Grip2        << -743.907, -54.142,  10.640, 85.337, 178.829, 153.058;
  P_Before_Grip2 << -746.907, -54.142, 275.292, 85.337, 178.829, 153.058;

  P_Before_Drop2 <<    0.606, 700.716, 275.287, 85.366, 178.828, 153.090;
  P_Drop2        <<    0.606, 700.716,  -5.445, 85.271, 178.828, 152.991;

  if (boxType == "B") {
    path.push_back(P_Home);
    path.push_back(P_Top_Box);
    path.push_back(P_Before_Grip);
    path.push_back(P_Grip);

    path.push_back(P_Before_Grip);
    path.push_back(P_Top_Box);
    path.push_back(P_Top_Vehicle);
    path.push_back(P_Before_Drop);
    path.push_back(P_Drop);

    path.push_back(P_Before_Drop);
    path.push_back(P_Top_Vehicle);
    path.push_back(P_Home);
  }
  else if (boxType == "C") {
    path.push_back(P_Home);
    path.push_back(P_Top_Box);
    path.push_back(P_Before_Grip2);
    path.push_back(P_Grip2);

    path.push_back(P_Before_Grip2);
    path.push_back(P_Top_Box);
    path.push_back(P_Top_Vehicle);
    path.push_back(P_Before_Drop2);
    path.push_back(P_Drop2);

    path.push_back(P_Before_Drop2);
    path.push_back(P_Top_Vehicle);
    path.push_back(P_Home);
  }

  return path;
}

class KhiOtaCommNode : public rclcpp::Node
{
public:
  KhiOtaCommNode()
  : Node("khi_ota_comm"),
    state_(OTA_STATE_WAIT),
    operate_running_(false),
    finish_pub_count_(0)
  {
    // Parametreler (isterseniz launch üzerinden override edebilirsiniz)
    robot_ip_   = this->declare_parameter<std::string>("robot_ip", "192.168.3.7");
    robot_port_ = this->declare_parameter<int>("robot_port", 11111);

    // ROS2 Subscriber/Publisher
    sub_ = this->create_subscription<agv_msgs::msg::TaskCom>(
      "ota_status", rclcpp::QoS(1000),
      std::bind(&KhiOtaCommNode::otaCallback, this, std::placeholders::_1));

    pub_ = this->create_publisher<agv_msgs::msg::TaskCom>("rk_hmi_status", rclcpp::QoS(1000));

    // 10 Hz state-machine tick
    timer_ = this->create_wall_timer(
      100ms, std::bind(&KhiOtaCommNode::tick, this));

    RCLCPP_INFO(this->get_logger(), "khi_ota_comm ROS2 node started. robot_ip=%s port=%d",
                robot_ip_.c_str(), robot_port_);
  }

private:
  void otaCallback(const agv_msgs::msg::TaskCom::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(data_mtx_);
    station_id_    = msg->station_id;
    product_name_  = msg->product_name;
    product_count_ = msg->product_count;
    duty_          = msg->duty;
    status_        = msg->status;
  }

  void tick()
  {
    // Snapshot al (thread-safe)
    int station_id, status;
    std::string product_name;
    {
      std::lock_guard<std::mutex> lk(data_mtx_);
      station_id   = station_id_;
      status       = status_;
      product_name = product_name_;
    }

    switch (state_.load())
    {
      case OTA_STATE_WAIT:
      {
        if (station_id == 1 && status == 1) {
          RCLCPP_INFO(this->get_logger(), "OTA has arrived -> OPERATE");
          state_.store(OTA_STATE_OPERATE);
        } else {
          RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                               "Waiting for OTA arrival (station_id=%d status=%d)", station_id, status);
        }
        break;
      }

      case OTA_STATE_OPERATE:
      {
        // OPERATE tek sefer tetiklensin
        if (!operate_running_.exchange(true)) {
          // product_name: "B" / "C"
          std::thread(&KhiOtaCommNode::runRobotOperate, this, product_name).detach();
        }
        break;
      }

      case OTA_STATE_FINISH:
      {
        // ROS1’de 10 kere publish edip 5sn bekliyordunuz.
        // Burada 10 tick boyunca publish edip ardından 5sn bekleyip WAIT'e dönüyoruz.
        if (finish_pub_count_ < 10) {
          agv_msgs::msg::TaskCom p_msg;
          p_msg.station_id = 1;
          p_msg.product_name = "B";
          p_msg.product_count = 1;
          p_msg.duty = 1;
          p_msg.status = 1;
          pub_->publish(p_msg);
          finish_pub_count_++;
        } else {
          // 5s bekleme (blocking kısa; isterseniz timer tabanlı da yapılır)
          RCLCPP_INFO(this->get_logger(), "FINISH done. Sleeping 5s then WAIT.");
          std::this_thread::sleep_for(5s);
          finish_pub_count_ = 0;
          state_.store(OTA_STATE_WAIT);
        }
        break;
      }

      default:
        state_.store(OTA_STATE_WAIT);
        break;
    }
  }

  void runRobotOperate(const std::string & boxType)
  {
    RCLCPP_INFO(this->get_logger(), "Robot operate thread started. boxType=%s", boxType.c_str());

    // Robot bağlantısı / logger
    SIRLogger *logger = new SIRLogger("log.txt", SIRLOGLEVEL::LOGLEVEL_DEBUG);
    SIRConnection *con = new SIRLinConnection(logger, robot_ip_.c_str(), robot_port_);

    KawasakiRS005LRobot robot(con, logger, nullptr, MPT_TASK, MT_LINEAR);

    // Connect
    if (!robot.Connect()) {
      RCLCPP_ERROR(this->get_logger(), "Could not connect to robot.");
      cleanupRobot(robot, con, logger);
      operate_running_.store(false);
      state_.store(OTA_STATE_WAIT);
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Connected to robot.");

    con->setBlockingMode(0);
    robot.setWaitForCommand(2000);

    std::this_thread::sleep_for(2s);

    std::vector<SIRMatrix> path = pathPlanner(boxType);
    if (path.empty()) {
      RCLCPP_ERROR(this->get_logger(), "pathPlanner returned empty path for boxType=%s", boxType.c_str());
      cleanupRobot(robot, con, logger);
      operate_running_.store(false);
      state_.store(OTA_STATE_WAIT);
      return;
    }

    // open Gripper (aynı mantık)
    robot.setSignal(1,false);
    robot.setSignal(2,false);
    robot.setSignal(1,true);

    // İlk segment (0..3)
    for (int i = 0; i < 4; i++) {
      std::cout << "send packet id: " << robot.add(path[i]) << std::endl;
    }
    if (!robot.move()) {
      RCLCPP_ERROR(this->get_logger(), "robot.move() failed at segment1");
    }
    while (robot.getStatus() != RS_STOP) {
      std::this_thread::sleep_for(1s);
    }

    // Grip
    robot.setSignal(1,false);
    std::this_thread::sleep_for(1s);
    robot.setSignal(2,true);
    std::this_thread::sleep_for(2s);

    // İkinci segment (4..8)
    for (int i = 4; i < 9; i++) {
      std::cout << "send packet id: " << robot.add(path[i]) << std::endl;
    }
    if (!robot.move()) {
      RCLCPP_ERROR(this->get_logger(), "robot.move() failed at segment2");
    }
    while (robot.getStatus() != RS_STOP) {
      std::this_thread::sleep_for(1s);
    }

    // Drop
    robot.setSignal(2,false);
    std::this_thread::sleep_for(1s);
    robot.setSignal(1,true);
    std::this_thread::sleep_for(2s);

    // Üçüncü segment (9..11)
    for (int i = 9; i < 12; i++) {
      std::cout << "send packet id: " << robot.add(path[i]) << std::endl;
    }
    if (!robot.move()) {
      RCLCPP_ERROR(this->get_logger(), "robot.move() failed at segment3");
    }
    while (robot.getStatus() != RS_STOP) {
      std::this_thread::sleep_for(1s);
    }

    robot.setSignal(1,false);
    std::this_thread::sleep_for(2s);

    // close
    if (robot.close()) {
      RCLCPP_INFO(this->get_logger(), "Robot connection closed.");
    } else {
      RCLCPP_WARN(this->get_logger(), "Robot connection close reported failure.");
    }

    cleanupRobot(robot, con, logger);

    // FINISH'e geç
    state_.store(OTA_STATE_FINISH);
    operate_running_.store(false);

    RCLCPP_INFO(this->get_logger(), "Robot operate thread finished -> FINISH");
  }

  void cleanupRobot(KawasakiRS005LRobot &robot, SIRConnection *con, SIRLogger *logger)
  {
    (void)robot;
    delete con;
    delete logger;
  }

private:
  // ROS2
  rclcpp::Subscription<agv_msgs::msg::TaskCom>::SharedPtr sub_;
  rclcpp::Publisher<agv_msgs::msg::TaskCom>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // OTA latest data
  std::mutex data_mtx_;
  int station_id_{0};
  std::string product_name_{"B"};
  int product_count_{0};
  int duty_{0};
  int status_{0};

  // State
  std::atomic<int> state_;
  std::atomic<bool> operate_running_;
  int finish_pub_count_;

  // Params
  std::string robot_ip_;
  int robot_port_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<KhiOtaCommNode>();

  // Tek thread yeter; robot operasyonu ayrı thread’de.
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
