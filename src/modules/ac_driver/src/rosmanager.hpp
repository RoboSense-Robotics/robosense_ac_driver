/*********************************************************************************************************************
  Copyright 2025 RoboSense Technology Co., Ltd

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

  http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*********************************************************************************************************************/
#ifndef RSROSMANAGER_HPP
#define RSROSMANAGER_HPP

#if defined(ROS_FOUND)
#include <ros/ros.h>
#elif defined(ROS2_FOUND)
#include <rclcpp/rclcpp.hpp>
#endif // ROS_ROS2_FOUND
#include "calibmanager.hpp"
#include "convertmanager.hpp"

// 创建ROS/ROS2 Publisher别名
#if defined(ROS_FOUND)
// Publisher(s)
// image
using ROS_PUBLISHER_IMAGE = ros::Publisher;
using ROS_PUBLISHER_IMAGE_PTR = std::shared_ptr<ROS_PUBLISHER_IMAGE>;
// compressedimage
using ROS_PUBLISHER_COMPRESSED_IMAGE = ros::Publisher;
using ROS_PUBLISHER_COMPRESSED_IMAGE_PTR =
    std::shared_ptr<ROS_PUBLISHER_COMPRESSED_IMAGE>;
// pointcloud2
using ROS_PUBLISHER_POINTCLOUD2 = ros::Publisher;
using ROS_PUBLISHER_POINTCLOUD2_PTR =
    std::shared_ptr<ROS_PUBLISHER_POINTCLOUD2>;
// imu
using ROS_PUBLISHER_IMU = ros::Publisher;
using ROS_PUBLISHER_IMU_PTR = std::shared_ptr<ROS_PUBLISHER_IMU>;
// camerainfo
using ROS_PUBLISHER_CAMERAINFO = ros::Publisher;
using ROS_PUBLISHER_CAMERAINFO_PTR = std::shared_ptr<ROS_PUBLISHER_CAMERAINFO>;
// rsacdevicecalib
using ROS_PUBLISHER_RSACDEVICECALIB = ros::Publisher;
using ROS_PUBLISHER_RSACDEVICECALIB_PTR =
    std::shared_ptr<ROS_PUBLISHER_RSACDEVICECALIB>;
#elif defined(ROS2_FOUND)
// Publisher(s)
// image
using ROS_PUBLISHER_IMAGE = rclcpp::Publisher<sensor_msgs::msg::Image>;
using ROS_PUBLISHER_IMAGE_PTR = std::shared_ptr<ROS_PUBLISHER_IMAGE>;
// compressedimage
using ROS_PUBLISHER_COMPRESSED_IMAGE =
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>;
using ROS_PUBLISHER_COMPRESSED_IMAGE_PTR =
    std::shared_ptr<ROS_PUBLISHER_COMPRESSED_IMAGE>;
// pointcloud2
using ROS_PUBLISHER_POINTCLOUD2 =
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>;
using ROS_PUBLISHER_POINTCLOUD2_PTR =
    std::shared_ptr<ROS_PUBLISHER_POINTCLOUD2>;
// imu
using ROS_PUBLISHER_IMU = rclcpp::Publisher<sensor_msgs::msg::Imu>;
using ROS_PUBLISHER_IMU_PTR = std::shared_ptr<ROS_PUBLISHER_IMU>;
// camerainfo
using ROS_PUBLISHER_CAMERAINFO =
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>;
using ROS_PUBLISHER_CAMERAINFO_PTR = std::shared_ptr<ROS_PUBLISHER_CAMERAINFO>;
// rsacdevicecalib
using ROS_PUBLISHER_RSACDEVICECALIB = rclcpp::Publisher<ROS_RSACDEVICECALIB>;
using ROS_PUBLISHER_RSACDEVICECALIB_PTR =
    std::shared_ptr<ROS_PUBLISHER_RSACDEVICECALIB>;
// 零拷贝消息 Publisher(s)
// RsImage1M
using ROS_ZEROCOPY_PUBLISHER_RSIMAGE1M =
    rclcpp::Publisher<ROS_ZEROCOPY_IMAGE1M>;
using ROS_ZEROCOPY_PUBLISHER_RSIMAGE1M_PTR =
    std::shared_ptr<ROS_ZEROCOPY_PUBLISHER_RSIMAGE1M>;
// RsImage2M
using ROS_ZEROCOPY_PUBLISHER_RSIMAGE2M =
    rclcpp::Publisher<ROS_ZEROCOPY_IMAGE2M>;
using ROS_ZEROCOPY_PUBLISHER_RSIMAGE2M_PTR =
    std::shared_ptr<ROS_ZEROCOPY_PUBLISHER_RSIMAGE2M>;
// RsImage4M
using ROS_ZEROCOPY_PUBLISHER_RSIMAGE4M =
    rclcpp::Publisher<ROS_ZEROCOPY_IMAGE4M>;
using ROS_ZEROCOPY_PUBLISHER_RSIMAGE4M_PTR =
    std::shared_ptr<ROS_ZEROCOPY_PUBLISHER_RSIMAGE4M>;
// RsImage6M
using ROS_ZEROCOPY_PUBLISHER_RSIMAGE6M =
    rclcpp::Publisher<ROS_ZEROCOPY_IMAGE6M>;
using ROS_ZEROCOPY_PUBLISHER_RSIMAGE6M_PTR =
    std::shared_ptr<ROS_ZEROCOPY_PUBLISHER_RSIMAGE6M>;
// RsImage8M
using ROS_ZEROCOPY_PUBLISHER_RSIMAGE8M =
    rclcpp::Publisher<ROS_ZEROCOPY_IMAGE8M>;
using ROS_ZEROCOPY_PUBLISHER_RSIMAGE8M_PTR =
    std::shared_ptr<ROS_ZEROCOPY_PUBLISHER_RSIMAGE8M>;
// RsPointCloud1M
using ROS_ZEROCOPY_PUBLISHER_POINTCLOUD1M =
    rclcpp::Publisher<ROS_ZEROCOPY_POINTCLOUD1M>;
using ROS_PUBLISHER_POINTCLOUD1M_PTR =
    std::shared_ptr<ROS_ZEROCOPY_PUBLISHER_POINTCLOUD1M>;
// RsPointCloud2M
using ROS_ZEROCOPY_PUBLISHER_POINTCLOUD2M =
    rclcpp::Publisher<ROS_ZEROCOPY_POINTCLOUD2M>;
using ROS_PUBLISHER_POINTCLOUD2M_PTR =
    std::shared_ptr<ROS_ZEROCOPY_PUBLISHER_POINTCLOUD2M>;
// RsPointCloud4M
using ROS_ZEROCOPY_PUBLISHER_POINTCLOUD4M =
    rclcpp::Publisher<ROS_ZEROCOPY_POINTCLOUD4M>;
using ROS_PUBLISHER_POINTCLOUD4M_PTR =
    std::shared_ptr<ROS_ZEROCOPY_PUBLISHER_POINTCLOUD4M>;
// RsPointCloud6M
using ROS_ZEROCOPY_PUBLISHER_POINTCLOUD6M =
    rclcpp::Publisher<ROS_ZEROCOPY_POINTCLOUD6M>;
using ROS_PUBLISHER_POINTCLOUD6M_PTR =
    std::shared_ptr<ROS_ZEROCOPY_PUBLISHER_POINTCLOUD6M>;
// RsPointCloud8M
using ROS_ZEROCOPY_PUBLISHER_POINTCLOUD8M =
    rclcpp::Publisher<ROS_ZEROCOPY_POINTCLOUD8M>;
using ROS_PUBLISHER_POINTCLOUD8M_PTR =
    std::shared_ptr<ROS_ZEROCOPY_PUBLISHER_POINTCLOUD8M>;
#endif // defined(ROS_ROS2_FOUND)

namespace robosense {
namespace interface {

class RSRosManager {
public:
  using Ptr = std::shared_ptr<RSRosManager>;
  using ConstPtr = std::shared_ptr<const RSRosManager>;

public:
  RSRosManager() = default;
  ~RSRosManager() = default;

public:
#if defined(ROS_FOUND)
  template <typename ROS_MESSAGE_TYPE>
  static std::shared_ptr<ros::Publisher>
  create_publisher(ros::NodeHandle &nh, const std::string &topic_name,
                   const uint32_t depth) {
    std::shared_ptr<ros::Publisher> publisherPtr;
    try {
      publisherPtr = std::make_shared<ros::Publisher>();
    } catch (...) {
      RS_SPDLOG_ERROR(std::string("Create Ros Topic = ") + topic_name +
                      " Failed !");
      return nullptr;
    }
    *publisherPtr = nh.advertise<ROS_MESSAGE_TYPE>(topic_name, depth);
    return publisherPtr;
  }
#elif defined(ROS2_FOUND)
  template <typename ROS_MESSAGE_TYPE>
  static std::shared_ptr<rclcpp::Publisher<ROS_MESSAGE_TYPE>>
  create_publisher(rclcpp::Node *pNode, const std::string &topic_name,
                   const rclcpp::QoS &qos) {
    if (pNode == nullptr) {
      RS_SPDLOG_ERROR(std::string("Create Ros2 Topic = ") + topic_name +
                      " Failed !");
      return nullptr;
    }
    return pNode->create_publisher<ROS_MESSAGE_TYPE>(topic_name, qos);
  }
#endif // defined(ROS_ROS2_FOUND)
};

} // namespace interface
} // namespace robosense

#endif // RSROSMANAGER_HPP