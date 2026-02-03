//
// Copyright (c) 2023 SICK AG
// SPDX-License-Identifier: Unlicense
//

#include <boost/thread.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <std_msgs/msg/header.hpp>
#include <memory>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/publisher.hpp>

#include "VisionaryControl.h"
#include "VisionaryDataStream.h"
#include "VisionaryTMiniData.h"

using namespace visionary;
using namespace std::chrono_literals;

std::shared_ptr<VisionaryControl> gControl;
std::shared_ptr<VisionaryTMiniData> gDataHandler;

image_transport::Publisher gPubDepth, gPubIntensity, gPubStatemap;
rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr gPubCameraInfo;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr gPubPoints;

std::shared_ptr<diagnostic_updater::Updater> updater;

std::string gFrameId;
std::string gDeviceIdent;
bool gEnableDepth = true, gEnableIntensity = true, gEnableStatemap = true, gEnablePoints = true;

boost::mutex gDataMtx;
bool gReceive = true;

int gNumSubs = 0;

void driver_diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "driver running");
  stat.add("frame_id", gFrameId);
  stat.add("device_ident", gDeviceIdent);
}

void publishCameraInfo(const std_msgs::msg::Header &header, VisionaryTMiniData &dataHandler)
{
  sensor_msgs::msg::CameraInfo ci;
  ci.header = header;

  ci.height = dataHandler.getHeight();
  ci.width = dataHandler.getWidth();

  ci.d.resize(5, 0.0);
  ci.d[0] = dataHandler.getCameraParameters().k1;
  ci.d[1] = dataHandler.getCameraParameters().k2;
  ci.d[2] = dataHandler.getCameraParameters().p1;
  ci.d[3] = dataHandler.getCameraParameters().p2;
  ci.d[4] = dataHandler.getCameraParameters().k3;

  for (int i = 0; i < 9; i++) ci.k[i] = 0.0;
  ci.k[0] = dataHandler.getCameraParameters().fx;
  ci.k[4] = dataHandler.getCameraParameters().fy;
  ci.k[2] = dataHandler.getCameraParameters().cx;
  ci.k[5] = dataHandler.getCameraParameters().cy;
  ci.k[8] = 1.0;

  for (int i = 0; i < 12; i++) ci.p[i] = 0.0;
  ci.p[0] = dataHandler.getCameraParameters().fx;
  ci.p[5] = dataHandler.getCameraParameters().fy;
  ci.p[10] = 1.0;
  ci.p[2] = dataHandler.getCameraParameters().cx;
  ci.p[6] = dataHandler.getCameraParameters().cy;

  gPubCameraInfo->publish(ci);
}

void publishDepth(const std_msgs::msg::Header &header, VisionaryTMiniData &dataHandler)
{
  std::vector<uint16_t> vec = dataHandler.getDistanceMap();
  cv::Mat m(dataHandler.getHeight(), dataHandler.getWidth(), CV_16UC1);
  memcpy(m.data, vec.data(), vec.size() * sizeof(uint16_t));

  auto msg = cv_bridge::CvImage(header, sensor_msgs::image_encodings::TYPE_16UC1, m).toImageMsg();
  gPubDepth.publish(msg);
}

void publishIntensity(const std_msgs::msg::Header &header, VisionaryTMiniData &dataHandler)
{
  std::vector<uint16_t> vec = dataHandler.getIntensityMap();
  cv::Mat m(dataHandler.getHeight(), dataHandler.getWidth(), CV_16UC1);
  memcpy(m.data, vec.data(), vec.size() * sizeof(uint16_t));

  auto msg = cv_bridge::CvImage(header, sensor_msgs::image_encodings::TYPE_16UC1, m).toImageMsg();
  gPubIntensity.publish(msg);
}

void publishStateMap(const std_msgs::msg::Header &header, VisionaryTMiniData &dataHandler)
{
  std::vector<uint16_t> vec = dataHandler.getStateMap();
  cv::Mat m(dataHandler.getHeight(), dataHandler.getWidth(), CV_16UC1);
  memcpy(m.data, vec.data(), vec.size() * sizeof(uint16_t));

  auto msg = cv_bridge::CvImage(header, sensor_msgs::image_encodings::TYPE_16UC1, m).toImageMsg();
  gPubStatemap.publish(msg);
}

void publishPointCloud(const std_msgs::msg::Header &header, VisionaryTMiniData &dataHandler)
{
  using PointCloud = sensor_msgs::msg::PointCloud2;

  auto cloudMsg = std::make_shared<PointCloud>();
  cloudMsg->header = header;
  cloudMsg->height = dataHandler.getHeight();
  cloudMsg->width = dataHandler.getWidth();
  cloudMsg->is_dense = false;
  cloudMsg->is_bigendian = false;

  cloudMsg->fields.resize(4);
  cloudMsg->fields[0].name = "x";
  cloudMsg->fields[1].name = "y";
  cloudMsg->fields[2].name = "z";
  cloudMsg->fields[3].name = "intensity";

  int offset = 0;
  for (size_t d = 0; d < 3; ++d, offset += sizeof(float)) {
    cloudMsg->fields[d].offset = offset;
    cloudMsg->fields[d].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloudMsg->fields[d].count = 1;
  }
  cloudMsg->fields[3].offset = offset;
  cloudMsg->fields[3].datatype = sensor_msgs::msg::PointField::UINT16;
  cloudMsg->fields[3].count = 1;
  offset += sizeof(uint16_t);

  cloudMsg->point_step = offset;
  cloudMsg->row_step = cloudMsg->point_step * cloudMsg->width;
  cloudMsg->data.resize(cloudMsg->height * cloudMsg->row_step);

  std::vector<PointXYZ> pointCloud;
  dataHandler.generatePointCloud(pointCloud);
  dataHandler.transformPointCloud(pointCloud);

  auto itIntens = dataHandler.getIntensityMap().begin();
  auto itPC = pointCloud.begin();
  size_t cloudSize = dataHandler.getHeight() * dataHandler.getWidth();
  for (size_t index = 0; index < cloudSize; ++index, ++itIntens, ++itPC) {
    memcpy(&cloudMsg->data[index * cloudMsg->point_step + cloudMsg->fields[0].offset], &*itPC, sizeof(PointXYZ));
    memcpy(&cloudMsg->data[index * cloudMsg->point_step + cloudMsg->fields[3].offset], &*itIntens, sizeof(uint16_t));
  }

  gPubPoints->publish(*cloudMsg);
}

void publish_frame(VisionaryTMiniData &dataHandler)
{
  std_msgs::msg::Header header;
  header.stamp = rclcpp::Clock().now();
  header.frame_id = gFrameId;

  if (gPubCameraInfo) publishCameraInfo(header, dataHandler);
  if (gEnableDepth) publishDepth(header, dataHandler);
  if (gEnableIntensity) publishIntensity(header, dataHandler);
  if (gEnableStatemap) publishStateMap(header, dataHandler);
  if (gEnablePoints) publishPointCloud(header, dataHandler);
}

void thr_publish_frame()
{
  boost::lock_guard<boost::mutex> lock(gDataMtx);
  publish_frame(*gDataHandler);
}

void thr_receive_frame(std::shared_ptr<VisionaryDataStream> pDataStream,
                       std::shared_ptr<VisionaryTMiniData> pDataHandler)
{
  while (gReceive) {
    if (!pDataStream->getNextFrame()) continue;
    if (gDataMtx.try_lock()) {
      gDataHandler = pDataHandler;
      gDataMtx.unlock();
      boost::thread thr(&thr_publish_frame);
    }
  }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("sick_visionary_t_mini");

  // Declare parameters
  node->declare_parameter<std::string>("remote_device_ip", "192.168.3.50");
  node->declare_parameter<std::string>("frame_id", "ur10e_depth_optical_frame");
  node->declare_parameter<bool>("enable_depth", true);
  node->declare_parameter<bool>("enable_intensity", true);
  node->declare_parameter<bool>("enable_statemap", true);
  node->declare_parameter<bool>("enable_points", true);

  std::string remoteDeviceIp;
  node->get_parameter("remote_device_ip", remoteDeviceIp);
  node->get_parameter("frame_id", gFrameId);
  node->get_parameter("enable_depth", gEnableDepth);
  node->get_parameter("enable_intensity", gEnableIntensity);
  node->get_parameter("enable_statemap", gEnableStatemap);
  node->get_parameter("enable_points", gEnablePoints);

  // Create publishers
  image_transport::ImageTransport it(node);
  gPubCameraInfo = node->create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", 10);
  if (gEnableDepth) gPubDepth = it.advertise("depth", 10);
  if (gEnablePoints) gPubPoints = node->create_publisher<sensor_msgs::msg::PointCloud2>("points", 10);
  if (gEnableIntensity) gPubIntensity = it.advertise("intensity", 10);
  if (gEnableStatemap) gPubStatemap = it.advertise("statemap", 10);

  // Connect to device
  auto pDataHandler = std::make_shared<VisionaryTMiniData>();
  auto pDataStream = std::make_shared<VisionaryDataStream>(pDataHandler);
  gControl = std::make_shared<VisionaryControl>();

  RCLCPP_INFO(node->get_logger(), "Connecting to device at %s", remoteDeviceIp.c_str());
  if (!gControl->open(VisionaryControl::ProtocolType::COLA_2, remoteDeviceIp.c_str(), 5000)) {
    RCLCPP_ERROR(node->get_logger(), "Connection with device control channel failed");
    return -1;
  }
  gControl->stopAcquisition();
  if (!pDataStream->open(remoteDeviceIp.c_str(), 2114u)) {
    RCLCPP_ERROR(node->get_logger(), "Connection with device data channel failed");
    return -1;
  }
  RCLCPP_INFO(node->get_logger(), "Connected with Visionary-T Mini");
  gControl->startAcquisition(); 
  
  gDeviceIdent = gControl->getDeviceIdent();
  


  updater = std::make_shared<diagnostic_updater::Updater>(node);
  updater->setHardwareID(gDeviceIdent);
  updater->add("driver", driver_diagnostics);

  auto timer = node->create_wall_timer(1s, [&]() { updater->force_update(); });


  boost::thread rec_thr(boost::bind(&thr_receive_frame, pDataStream, pDataHandler));

  rclcpp::spin(node);

  gReceive = false;
  rec_thr.join();

  gControl->stopAcquisition();
  gControl->close();
  pDataStream->close();

  rclcpp::shutdown();
  return 0;
}
