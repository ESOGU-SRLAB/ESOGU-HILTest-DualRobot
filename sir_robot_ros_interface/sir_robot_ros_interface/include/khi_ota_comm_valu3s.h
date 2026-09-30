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
  void khiOtaCommCallback(const std_msgs::msg::String::SharedPtr s_msg);
  std::string pose_data;
};

std::string p_product_name;


enum OTA_STATE{
    OTA_STATE_WAIT = 0,
    OTA_STATE_OPERATE,
    OTA_STATE_FINISH
};

/*bool WAIT = false;
bool OPERATE = false;
bool FINISH = false;*/






