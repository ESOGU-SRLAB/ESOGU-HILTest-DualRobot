#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <Eigen/StdVector>
#include <filesystem>
#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <cmath>


sensor_msgs::msg::JointState output_msg;


double degreesToRadians(double degrees) {
    return degrees * (M_PI / 180.0);
}


void inputCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {

    if (msg->data.size() == 6) {

        std::vector<double> values;
        for (size_t i = 0; i < 6; i++) {
            double radians = degreesToRadians(msg->data[i]);
            values.push_back(radians);
        }

        //values.push_back(msg->data[0]);
        //values.push_back(msg->data[1]);
        //values.push_back(msg->data[2]);
        //values.push_back(msg->data[3]);
        //values.push_back(msg->data[4]);
        //values.push_back(msg->data[5]);
    
        std::cout << "Received values: ";
        for (const auto& value : values) {
            std::cout << value << ", ";
        }

        output_msg.header.stamp = rclcpp::Clock().now();
        output_msg.position = values;
    } 
    else {
        std::cout << "Error: message does not contain 6 elements" << std::endl;
    }

}


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("right_rokos_joint_publisher");

    auto sub = node->create_subscription<std_msgs::msg::Float64MultiArray>(
        "joint_poses", 1000, inputCallback);
    auto pub = node->create_publisher<sensor_msgs::msg::JointState>(
        "right_rokos/joint_states", 1000);


    rclcpp::Rate loop_rate(30);
    while (rclcpp::ok()){

        std::cout <<"Joints == "<< output_msg.position.size() << " values" <<std::endl;
        pub->publish(output_msg);
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}