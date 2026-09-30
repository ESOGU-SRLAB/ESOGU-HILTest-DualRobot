#include <string>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <iomanip>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "agv_msgs/msg/task_com.hpp"



class otaListener
{
public:
  void khiOtaCommCallback(const agv_msgs::msg::TaskCom::SharedPtr s_msg);
  int s_station_id, s_product_count, s_duty, s_status;
  std::string s_product_name;
};



int p_station_id, p_product_count, p_duty, p_status;
std::string p_product_name;


enum OTA_STATE{
    OTA_STATE_WAIT = 0,
    OTA_STATE_OPERATE,
    OTA_STATE_FINISH
};

/*bool WAIT = false;
bool OPERATE = false;
bool FINISH = false;*/






