#define ROBOTFORLINUXTEST
#ifdef ROBOTFORLINUXTEST


#include "KawasakiRS005LRobot.h"
#include "SIRLinConnection.h"
#include "SIRTypeDefs.h"
#include "khi_ota_comm_valu3s.h"
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sir_robot_ros_interface/srv/manipulator_pose_ino2.hpp>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <Eigen/StdVector>
#include <filesystem>

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(SIRMatrix)

rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr JointPosePublisher;

using namespace std;
  SIRLogger *logger = new SIRLogger("log.txt", SIRLOGLEVEL::LOGLEVEL_DEBUG);
  SIRConnection *con = new SIRLinConnection(logger,"192.168.3.7",11111);
  KawasakiRS005LRobot robot(con, logger,nullptr, MPT_JOINT, MT_P2P);
  //KawasakiRS005LRobot robot(con, logger,nullptr, MPT_TASK, MT_LINEAR);
  
  int cancel_data = 0;
  

void cancelCallback(const std_msgs::msg::Int8::SharedPtr msg)
{
    cancel_data = msg->data;
}

OTA_STATE state = OTA_STATE_WAIT;

float rad2deg(float radian){
    float degree = radian * (180.0 / M_PI);
    return degree;
}

bool checkCorrectStop(SIRMatrix poses, SIRMatrix pos){
    float th = 1.0;
    for (int i = 0; i < 6; i++) {
        if ((abs(abs(poses(i)) - abs(pos(i))) > th)){
            return false;
        }
    }   
    return true;
}

bool pathExists(const std::string& path) {
    std::ifstream file(path.c_str());
    return file.good();
}

void add(const std::shared_ptr<sir_robot_ros_interface::srv::ManipulatorPoseIno2::Request> req,
         std::shared_ptr<sir_robot_ros_interface::srv::ManipulatorPoseIno2::Response> res)
{
    std::cout<< req->path << std::endl;

    std::ifstream file(req->path);
    std::string line;
    std::vector<std::vector<std::string>> data;

    if(!pathExists(req->path)) //Exits the program and outputs this message if the file is not found
    {
        std::cout << "File not found." << std::endl;
        res->status = "NoFile";
        robot.close();
        return;
    }
    rclcpp::Rate loop_rate(30);

    std::getline(file, line); // Read and discard the first line

    while (std::getline(file, line)) {
        std::stringstream lineStream(line);
        std::string cell;
        std::vector<std::string> row;

        while (std::getline(lineStream, cell, ',')) {
            row.push_back(cell);
        }

        data.push_back(row);

    }

    file.close();

    if(state==OTA_STATE_WAIT){
      if (data.empty() == false){
        std::cout << "OTA has arrived" << std::endl;
        state = OTA_STATE_OPERATE;
      }
      else{
        std::cout << "Waiting for OTA arrival" << std::endl;
      }

    }
    if(state==OTA_STATE_OPERATE){
        if (!robot.Connect()) {
            cout << "Could not connect to robot..." << endl;
            return;
        }
        else
            cout << "Connected..." << endl;
        //********* CONNECTED **************

        con->setBlockingMode(0);
        robot.setWaitForCommand(2000);
        //sleep(2);

        //open Gripper
        if (robot.setSignal(1,false))
            cout << "success to set signal..." << endl;
        else{
            cout << "fail to set signal...1" << endl;
            res->status = "abort";
            robot.close();
            return;
        }
        if (robot.setSignal(2,false))
            cout << "success to set signal..." << endl;
        else{
            cout << "fail to set signal...2" << endl;
            res->status = "abort";
            robot.close();
            return;
        }
        if (robot.setSignal(1,true))
            cout << "success to set signal..." << endl;
        else{
            cout << "fail to set signal...3" << endl;
            res->status = "abort";
            robot.close();
            return;
        }
        SIRMatrix poses(6,1);
        SIRMatrix last_poses(6,1);
        int count = 0;

        std::cout<< data.size() << std::endl;

        for (int i = 0; i < data.size(); ++i) {
            float pose1 = rad2deg(std::stof(data[i][0]));
            float pose2 = rad2deg(std::stof(data[i][1]));
            float pose3 = rad2deg(std::stof(data[i][2]));
            float pose4 = rad2deg(std::stof(data[i][3]));
            float pose5 = rad2deg(std::stof(data[i][4]));
            float pose6 = rad2deg(std::stof(data[i][5]));

            poses<<pose1, pose2, pose3, pose4, pose5, pose6; 
            std::cout<< pose1 <<" "<< pose2 <<" "<< pose3 <<" "<< pose4 <<" "<< pose5 <<" "<< pose6 << std::endl;
            // std::cout<< poses << std::endl;
            last_poses = poses;
            robot.add(poses);
        }

        // for (auto const& pose_list : req.poses) {
        //     if ((pose_list.pose.size() % 6) != 0){
        //         std::cout << pose_list.pose.size() << " on item " << count <<std::endl;
        //         res.status = "Wrong Input Lenght";
        //         return true;
        //     }
// 
        //     SIRMatrix poses(6,1);
        //     poses<<pose_list.pose[0], pose_list.pose[1], pose_list.pose[2], pose_list.pose[3], pose_list.pose[4], pose_list.pose[5];
        //     last_poses = poses;
        //     count++;
        //     robot.add(poses);
        // }

        robot.setSpeed(9.0);

        if (robot.move())
            cout << "the robot is moving" << endl;
        else{
            cout << "the robot is not moving" << endl;
            res->status = "abort";
            robot.close();
            return;
        }
        //hareketin bitimini bekle...
        while(robot.getStatus()!=RS_STOP){
            //sleep(1);
        //    std::cout<<cancel_data<<std::endl;
        //    if (cancel_data == 1){
        //       cout << "fail to set signal...4" << endl;
        //       res->status = "abort";
        //       robot.close();
        //       break;
        //    }
        //     rclcpp::spin_some(node);
        //    loop_rate.sleep();

        // Publish Joint States
            
            SIRMatrix joint_pose(6,1);
            robot.getJointPose(&joint_pose);

            std_msgs::msg::Float64MultiArray msg;
            msg.data.resize(6);
            msg.data[0] = joint_pose(0);
            msg.data[1] = joint_pose(1);
            msg.data[2] = joint_pose(2);
            msg.data[3] = joint_pose(3);
            msg.data[4] = joint_pose(4);
            msg.data[5] = joint_pose(5);

            cout <<"Joints == "<< msg.data.size() << " values" << endl;

            JointPosePublisher->publish(msg);

            // rclcpp::spin_some(node);
            loop_rate.sleep();
        }

        SIRMatrix pos(6,1);
        robot.getJointPose(&pos);
        
        if (checkCorrectStop(last_poses, pos) == false){
            cout << "stopped at wrong position" << endl;
            res->status = "abort";
            robot.close();
            return;
        }
    

        if (robot.setSignal(1,false))
            cout << "success to set signal..." << endl;
        else{
            cout << "fail to set signal...4" << endl;
            res->status = "abort";
            robot.close();
            return;
        }
        //sleep(2);

        if (robot.close())
            cout << "The connection is succesfully closed..." << endl;
        else{
            cout << "The connection is not succesfully closed..." << endl;
            res->status = "abort";
            robot.close();
            return;
        }
        //**********************************
        state = OTA_STATE_FINISH;

    }

    if(state == OTA_STATE_FINISH){
        rclcpp::sleep_for(std::chrono::seconds(1));
        state = OTA_STATE_WAIT;
        res->status = "succeed";
    }
}

int main(int argc, char **argv) {


  //SIRConnection *con = new SIRLinConnection(logger,"127.0.0.1",7777);
  //con->setBlockingMode(1);

  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("khi_ota_comm");
  
  JointPosePublisher = node->create_publisher<std_msgs::msg::Float64MultiArray>("joint_poses", 1000);
  auto service = node->create_service<sir_robot_ros_interface::srv::ManipulatorPoseIno2>(
      "manipulator_service", add);
  auto sub = node->create_subscription<std_msgs::msg::Int8>(
      "manipulator/cancel", 1000, cancelCallback);


  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}

#endif

