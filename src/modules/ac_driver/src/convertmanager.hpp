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
#ifndef RSCONVERTMANAGER_HPP
#define RSCONVERTMANAGER_HPP

#include "hyper_vision/logmanager/logmanager.h"
#define ENABLE_USE_PCL_LIDAR_MSG_TYPE (1)
#if ENABLE_USE_PCL_LIDAR_MSG_TYPE
#include "rs_driver/msg/pcl_point_cloud_msg.hpp"
#else
#include "rs_driver/msg/point_cloud_msg.hpp"
#endif // ENABLE_USE_PCL_LIDAR_MSG_TYPE
#include "rs_driver/msg/imu_data_msg.hpp"

#if defined(ROS_FOUND)
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#elif defined(ROS2_FOUND)
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
// 零拷贝消息
#include <robosense_msgs/msg/rs_image1_m.hpp>
#include <robosense_msgs/msg/rs_image2_m.hpp>
#include <robosense_msgs/msg/rs_image4_m.hpp>
#include <robosense_msgs/msg/rs_image6_m.hpp>
#include <robosense_msgs/msg/rs_image8_m.hpp>
#include <robosense_msgs/msg/rs_point_cloud1_m.hpp>
#include <robosense_msgs/msg/rs_point_cloud2_m.hpp>
#include <robosense_msgs/msg/rs_point_cloud4_m.hpp>
#include <robosense_msgs/msg/rs_point_cloud6_m.hpp>
#include <robosense_msgs/msg/rs_point_cloud8_m.hpp>
#endif // ROS_ROS2_FOUND

#if defined(ROS_FOUND)
using ROS_STRING = std_msgs::String;
using ROS_IMAGE = sensor_msgs::Image;
using ROS_COMPRESSED_IMAGE = sensor_msgs::CompressedImage;
using ROS_POINTCLOUD2 = sensor_msgs::PointCloud2;
using ROS_IMU = sensor_msgs::Imu;
using ROS_TIME = ros::Time;
#elif defined(ROS2_FOUND)
using ROS_STRING = std_msgs::msg::String;
using ROS_IMAGE = sensor_msgs::msg::Image;
using ROS_COMPRESSED_IMAGE = sensor_msgs::msg::CompressedImage;
using ROS_POINTCLOUD2 = sensor_msgs::msg::PointCloud2;
using ROS_IMU = sensor_msgs::msg::Imu;
using ROS_TIME = rclcpp::Time;
// 零拷贝消息
using ROS_ZEROCOPY_IMAGE1M = robosense_msgs::msg::RsImage1M;
using ROS_ZEROCOPY_IMAGE1M_PTR = std::shared_ptr<ROS_ZEROCOPY_IMAGE1M>;
using ROS_ZEROCOPY_IMAGE2M = robosense_msgs::msg::RsImage2M;
using ROS_ZEROCOPY_IMAGE2M_PTR = std::shared_ptr<ROS_ZEROCOPY_IMAGE2M>;
using ROS_ZEROCOPY_IMAGE4M = robosense_msgs::msg::RsImage4M;
using ROS_ZEROCOPY_IMAGE4M_PTR = std::shared_ptr<ROS_ZEROCOPY_IMAGE4M>;
using ROS_ZEROCOPY_IMAGE6M = robosense_msgs::msg::RsImage6M;
using ROS_ZEROCOPY_IMAGE6M_PTR = std::shared_ptr<ROS_ZEROCOPY_IMAGE6M>;
using ROS_ZEROCOPY_IMAGE8M = robosense_msgs::msg::RsImage8M;
using ROS_ZEROCOPY_IMAGE8M_PTR = std::shared_ptr<ROS_ZEROCOPY_IMAGE8M>;
using ROS_ZEROCOPY_POINTCLOUD1M = robosense_msgs::msg::RsPointCloud1M;
using ROS_ZEROCOPY_POINTCLOUD1M_PTR =
    std::shared_ptr<ROS_ZEROCOPY_POINTCLOUD1M>;
using ROS_ZEROCOPY_POINTCLOUD2M = robosense_msgs::msg::RsPointCloud2M;
using ROS_ZEROCOPY_POINTCLOUD2M_PTR =
    std::shared_ptr<ROS_ZEROCOPY_POINTCLOUD2M>;
using ROS_ZEROCOPY_POINTCLOUD4M = robosense_msgs::msg::RsPointCloud4M;
using ROS_ZEROCOPY_POINTCLOUD4M_PTR =
    std::shared_ptr<ROS_ZEROCOPY_POINTCLOUD4M>;
using ROS_ZEROCOPY_POINTCLOUD6M = robosense_msgs::msg::RsPointCloud6M;
using ROS_ZEROCOPY_POINTCLOUD6M_PTR =
    std::shared_ptr<ROS_ZEROCOPY_POINTCLOUD6M>;
using ROS_ZEROCOPY_POINTCLOUD8M = robosense_msgs::msg::RsPointCloud8M;
using ROS_ZEROCOPY_POINTCLOUD8M_PTR =
    std::shared_ptr<ROS_ZEROCOPY_POINTCLOUD8M>;
#endif // defined(ROS_ROS2_FOUND)
using ROS_STRING_PTR = std::shared_ptr<ROS_STRING>;
using ROS_IMAGE_PTR = std::shared_ptr<ROS_IMAGE>;
using ROS_COMPRESSED_IMAGE_PTR = std::shared_ptr<ROS_COMPRESSED_IMAGE>;
using ROS_POINTCLOUD2_PTR = std::shared_ptr<ROS_POINTCLOUD2>;
using ROS_IMU_PTR = std::shared_ptr<ROS_IMU>;
using ROS_TIME_PTR = std::shared_ptr<ROS_TIME>;

// 定义std::make_shared<T>宏
#define MAKE_SHARED_ROS_STRING std::make_shared<ROS_STRING>()
#define MAKE_SHARED_ROS_IMAGE std::make_shared<ROS_IMAGE>()
#define MAKE_SHARED_ROS_COMPRESSED_IMAGE                                       \
  std::make_shared<ROS_COMPRESSED_IMAGE>()
#define MAKE_SHARED_ROS_POINTCLOUD2 std::make_shared<ROS_POINTCLOUD2>()
#define MAKE_SHARED_ROS_IMU std::make_shared<ROS_IMU>()
#define MAKE_SHARED_ROS_TIME std::make_shared<ROS_TIME>()

namespace robosense {
namespace convert {

class RSConvertManager {
public:
  using Ptr = std::shared_ptr<RSConvertManager>;
  using ConstPtr = std::shared_ptr<const RSConvertManager>;

public:
  RSConvertManager() = default;
  ~RSConvertManager() = default;

public:
  static ROS_TIME secondsToRosStamp(const double timestampS) {
    static const int64_t RS_SENCOND_TO_NANOSECOND_FACTOR = 1000000000ull;
    const uint64_t timestampNs = timestampS * RS_SENCOND_TO_NANOSECOND_FACTOR;
    uint32_t sec = timestampNs / RS_SENCOND_TO_NANOSECOND_FACTOR;
    uint32_t nsec = timestampNs % RS_SENCOND_TO_NANOSECOND_FACTOR;
    return ROS_TIME(sec, nsec);
  }

  static double rosStampToSeconds(const ROS_TIME &rosStamp) {
#if defined(ROS_FOUND)
    double timestampS = rosStamp.toSec();
#elif defined(ROS2_FOUND)
    double timestampS = rosStamp.seconds();
#endif // ROS_ROS2_FOUND

    return timestampS;
  }

  static ROS_TIME nanosencodsToRosStamp(const uint64_t timestampNs) {
    static const int64_t RS_SENCOND_TO_NANOSECOND_FACTOR = 1000000000ull;
    uint32_t sec = timestampNs / RS_SENCOND_TO_NANOSECOND_FACTOR;
    uint32_t nsec = timestampNs % RS_SENCOND_TO_NANOSECOND_FACTOR;

    return ROS_TIME(sec, nsec);
  }

  static uint64_t rosStampToNanoseconds(const ROS_TIME rosStamp) {
#if defined(ROS_FOUND)
    uint64_t timestampNs = rosStamp.toNSec();
#elif defined(ROS2_FOUND)
    uint64_t timestampNs = rosStamp.nanoseconds();
#endif // ROS_ROS2_FOUND
    return timestampNs;
  }

public:
  static bool isLittleEndian() {
    int i = 1;
    return *(char *)&i == 1;
  }

  static bool isBigEndian() { return !isLittleEndian(); }

public:
  static int
  toRosPointCloud2Message(const std::shared_ptr<PointCloudT<PointXYZI>> &frame,
                          ROS_POINTCLOUD2_PTR &cloud_msg) {
    if (frame == nullptr || cloud_msg == nullptr) {
      RS_SPDLOG_ERROR("Input frame or/and cloud_msg is Nullptr !");
      return -1;
    }

    int ret = updateRosPointCloud2MessageXYZI(frame, cloud_msg);
    if (ret != 0) {
      RS_SPDLOG_ERROR("Input frame convert to cloud_msg Failed: ret = " +
                      std::to_string(ret));
      return -2;
    }

    return 0;
  }

  static int
  toRosPointCloud2Message(const std::shared_ptr<PointCloudT<PointXYZIF>> &frame,
                          ROS_POINTCLOUD2_PTR &cloud_msg) {
    if (frame == nullptr || cloud_msg == nullptr) {
      RS_SPDLOG_ERROR("Input frame or/and cloud_msg is Nullptr !");
      return -1;
    }

    int ret = updateRosPointCloud2MessageXYZIF(frame, cloud_msg);
    if (ret != 0) {
      RS_SPDLOG_ERROR("Input frame convert to cloud_msg Failed: ret = " +
                      std::to_string(ret));
      return -2;
    }

    return 0;
  }

  static int toRosPointCloud2Message(
      const std::shared_ptr<PointCloudT<PointXYZIRT>> &frame,
      ROS_POINTCLOUD2_PTR &cloud_msg) {
    if (frame == nullptr || cloud_msg == nullptr) {
      RS_SPDLOG_ERROR("Input frame or/and cloud_msg is Nullptr !");
      return -1;
    }

    int ret = updateRosPointCloud2MessageXYZIRT(frame, cloud_msg);
    if (ret != 0) {
      RS_SPDLOG_ERROR("Input frame convert to cloud_msg Failed: ret = " +
                      std::to_string(ret));
      return -2;
    }

    return 0;
  }

  static int toRosPointCloud2Message(
      const std::shared_ptr<PointCloudT<PointXYZIRTF>> &frame,
      ROS_POINTCLOUD2_PTR &cloud_msg) {
    if (frame == nullptr || cloud_msg == nullptr) {
      RS_SPDLOG_ERROR("Input frame or/and cloud_msg is Nullptr !");
      return -1;
    }

    int ret = updateRosPointCloud2MessageXYZIRTF(frame, cloud_msg);
    if (ret != 0) {
      RS_SPDLOG_ERROR("Input frame convert to cloud_msg Failed: ret = " +
                      std::to_string(ret));
      return -2;
    }

    return 0;
  }

public:
  static int toRosImageMessage(const uint32_t width, const uint32_t height,
                               const ROS_TIME custom_time,
                               const std::string &frame_id, const uint8_t *data,
                               const size_t data_size, ROS_IMAGE_PTR &rgb_msg) {
    if (data == nullptr || rgb_msg == nullptr) {
      RS_SPDLOG_ERROR("Input data or/and rgb_msg is Nullptr !");
      return -1;
    } else if (width * height * 3 != data_size) {
      RS_SPDLOG_ERROR(
          "Input width*height*3 = " + std::to_string(width * height * 3) +
          " not match data_size = " + std::to_string(data_size));
      return -2;
    }

    rgb_msg->header.stamp = custom_time;
    rgb_msg->encoding = "rgb8";
    rgb_msg->is_bigendian = isBigEndian();
    rgb_msg->data.resize(data_size);
    memcpy(rgb_msg->data.data(), data, data_size);

    rgb_msg->header.frame_id = frame_id;
    rgb_msg->height = height;
    rgb_msg->width = width;
    rgb_msg->step = rgb_msg->width * 3 * 1;

    return 0;
  }

public:
  static int
  toRosImuMessage(const std::shared_ptr<robosense::lidar::ImuData> msgPtr,
                  const std::string &frame_id, ROS_IMU_PTR &imu_msg) {
    if (msgPtr == nullptr || imu_msg == nullptr) {
      RS_SPDLOG_ERROR("Input msgPtr or/and imu_msg is Nullptr !");
      return -1;
    }

    const auto custom_time = secondsToRosStamp(msgPtr->timestamp);
    imu_msg->header.stamp = custom_time;
    imu_msg->header.frame_id = frame_id;

    // Populate IMU message with acceleration and gyro data
    imu_msg->linear_acceleration.x = msgPtr->linear_acceleration_x;
    imu_msg->linear_acceleration.y = msgPtr->linear_acceleration_y;
    imu_msg->linear_acceleration.z = msgPtr->linear_acceleration_z;
    imu_msg->angular_velocity.x = msgPtr->angular_velocity_x;
    imu_msg->angular_velocity.y = msgPtr->angular_velocity_y;
    imu_msg->angular_velocity.z = msgPtr->angular_velocity_z;
    return 0;
  }

#if defined(ROS2_FOUND)
public:
  template <typename ZEROCOPY_MESSAGE_T>
  static int
  toZeroCopyCloudMessage(const std::shared_ptr<PointCloudT<PointXYZI>> &frame,
                         ZEROCOPY_MESSAGE_T *cloud_msg) {
    if (frame == nullptr || cloud_msg == nullptr) {
      RS_SPDLOG_ERROR("Input frame or/and cloud_msg is Nullptr !");
      return -1;
    }

    int ret = updateZeroCopyCloudMessageXYZI(frame, cloud_msg);
    if (ret != 0) {

      return -2;
    }

    return 0;
  }

  template <typename ZEROCOPY_MESSAGE_T>
  static int
  toZeroCopyCloudMessage(const std::shared_ptr<PointCloudT<PointXYZIF>> &frame,
                         ZEROCOPY_MESSAGE_T *cloud_msg) {
    if (frame == nullptr || cloud_msg == nullptr) {
      RS_SPDLOG_ERROR("Input frame or/and cloud_msg is Nullptr !");
      return -1;
    }

    int ret = updateZeroCopyCloudMessageXYZIF(frame, cloud_msg);
    if (ret != 0) {
      RS_SPDLOG_ERROR("Input frame convert to cloud_msg Failed: ret = " +
                      std::to_string(ret));
      return -2;
    }

    return 0;
  }

  template <typename ZEROCOPY_MESSAGE_T>
  static int
  toZeroCopyCloudMessage(const std::shared_ptr<PointCloudT<PointXYZIRT>> &frame,
                         ZEROCOPY_MESSAGE_T *cloud_msg) {
    if (frame == nullptr || cloud_msg == nullptr) {
      RS_SPDLOG_ERROR("Input frame or/and cloud_msg is Nullptr !");
      return -1;
    }

    int ret = updateZeroCopyCloudMessageXYZIRT(frame, cloud_msg);
    if (ret != 0) {
      RS_SPDLOG_ERROR("Input frame convert to cloud_msg Failed: ret = " +
                      std::to_string(ret));
      return -2;
    }

    return 0;
  }

  template <typename ZEROCOPY_MESSAGE_T>
  static int toZeroCopyCloudMessage(
      const std::shared_ptr<PointCloudT<PointXYZIRTF>> &frame,
      ZEROCOPY_MESSAGE_T *cloud_msg) {
    if (frame == nullptr || cloud_msg == nullptr) {
      RS_SPDLOG_ERROR("Input frame or/and cloud_msg is Nullptr !");
      return -1;
    }

    int ret = updateZeroCopyCloudMessageXYZIRTF(frame, cloud_msg);
    if (ret != 0) {
      RS_SPDLOG_ERROR("Input frame convert to cloud_msg Failed: ret = " +
                      std::to_string(ret));
      return -2;
    }

    return 0;
  }

public:
  template <typename ZEROCOPY_MESSAGE_T>
  static int toZeroCopyImageMessage(const uint32_t width, const uint32_t height,
                                    const rclcpp::Time custom_time,
                                    const std::string &frame_id,
                                    const uint8_t *data, const size_t data_size,
                                    ZEROCOPY_MESSAGE_T *rgb_msg) {
    if (data == nullptr || rgb_msg == nullptr) {
      RS_SPDLOG_ERROR("Input data or/and rgb_msg is Nullptr !");
      return -1;
    } else if (width * height * 3 != data_size) {
      RS_SPDLOG_ERROR(
          "Input width*height*3 = " + std::to_string(width * height * 3) +
          " not match data_size = " + std::to_string(data_size));
      return -2;
    } else if (rgb_msg->data.size() < data_size) {
      RS_SPDLOG_ERROR(
          "zerocopy data size = " + std::to_string(rgb_msg->data.size()) +
          " not match data_size = " + std::to_string(data_size));
      return -3;
    }

    // 构造head
    rgb_msg->header.stamp = custom_time;
    const char *id_str = frame_id.c_str();
    std::copy(id_str, id_str + strlen(id_str) + 1,
              rgb_msg->header.frame_id.begin());

    // 构造内容
    rgb_msg->height = height;
    rgb_msg->width = width;
    const char *encoding_str = "rgb8";
    std::copy(encoding_str, encoding_str + strlen(encoding_str) + 1,
              rgb_msg->encoding.begin());
    rgb_msg->is_bigendian = isBigEndian();
    rgb_msg->step = rgb_msg->width * 3 * 1;
    std::copy(data, data + data_size, rgb_msg->data.begin());

    return 0;
  }
#endif // defined(ROS2_FOUND)
private:
  static int updateRosPointCloud2MessageXYZI(
      const std::shared_ptr<PointCloudT<PointXYZI>> &frame,
      ROS_POINTCLOUD2_PTR &cloud_msg) {
    int ret = 0;
    // 构造PointField
    ret = updateRosPointFieldXYZI(cloud_msg);
    if (ret != 0) {
      RS_SPDLOG_ERROR("Update Ros2 cloud_msg PointField Failed: ret = " +
                      std::to_string(ret));
      return -1;
    }

    // 构造其他内容
    ret = updateRosCloudMessageContent<PointXYZI>(frame, cloud_msg);
    if (ret != 0) {
      RS_SPDLOG_ERROR("Update Ros2 cloud_msg Content Failed: ret = " +
                      std::to_string(ret));
      return -2;
    }

    return 0;
  }

  static int updateRosPointCloud2MessageXYZIF(
      const std::shared_ptr<PointCloudT<PointXYZIF>> &frame,
      ROS_POINTCLOUD2_PTR &cloud_msg) {
    int ret = 0;
    // 构造PointField
    ret = updateRosPointFieldXYZIF(cloud_msg);
    if (ret != 0) {
      RS_SPDLOG_ERROR("Update Ros2 cloud_msg PointField Failed: ret = " +
                      std::to_string(ret));
      return -1;
    }

    // 构造其他内容
    ret = updateRosCloudMessageContent<PointXYZIF>(frame, cloud_msg);
    if (ret != 0) {
      RS_SPDLOG_ERROR("Update Ros2 cloud_msg Content Failed: ret = " +
                      std::to_string(ret));
      return -2;
    }

    return 0;
  }

  static int updateRosPointCloud2MessageXYZIRT(
      const std::shared_ptr<PointCloudT<PointXYZIRT>> &frame,
      ROS_POINTCLOUD2_PTR &cloud_msg) {
    int ret = 0;

    // 构造PointField
    ret = updateRosPointFieldXYZIRT(cloud_msg);
    if (ret != 0) {
      RS_SPDLOG_ERROR("Update Ros2 cloud_msg PointField Failed: ret = " +
                      std::to_string(ret));
      return -1;
    }

    // 构造其他内容
    ret = updateRosCloudMessageContent<PointXYZIRT>(frame, cloud_msg);
    if (ret != 0) {
      RS_SPDLOG_ERROR("Update Ros2 cloud_msg Content Failed: ret = " +
                      std::to_string(ret));
      return -2;
    }

    return 0;
  }

  static int updateRosPointCloud2MessageXYZIRTF(
      const std::shared_ptr<PointCloudT<PointXYZIRTF>> &frame,
      ROS_POINTCLOUD2_PTR &cloud_msg) {
    int ret = 0;

    // 构造PointField
    ret = updateRosPointFieldXYZIRTF(cloud_msg);
    if (ret != 0) {
      RS_SPDLOG_ERROR("Update Ros2 cloud_msg PointField Failed: ret = " +
                      std::to_string(ret));
      return -1;
    }

    // 构造其他内容
    ret = updateRosCloudMessageContent<PointXYZIRTF>(frame, cloud_msg);
    if (ret != 0) {
      RS_SPDLOG_ERROR("Update Ros2 cloud_msg Content Failed: ret = " +
                      std::to_string(ret));
      return -2;
    }

    return 0;
  }

  template <typename POINT_T>
  static int updateRosCloudMessageContent(
      const std::shared_ptr<PointCloudT<POINT_T>> &frame,
      ROS_POINTCLOUD2_PTR &cloud_msg) {
    if (frame == nullptr || cloud_msg == nullptr) {
      RS_SPDLOG_ERROR("Input frame or/and cloud_msg is Nullptr !");
      return -1;
    }

    // 构造Header
    cloud_msg->header.frame_id = frame->frame_id;
    cloud_msg->header.stamp = secondsToRosStamp(frame->timestamp);

    cloud_msg->height = frame->height;
    cloud_msg->width = frame->width;
    cloud_msg->is_bigendian = isBigEndian();
    cloud_msg->is_dense = frame->is_dense;
    cloud_msg->row_step = cloud_msg->point_step * cloud_msg->width;

    // 拷贝内容
    const size_t total_size = cloud_msg->row_step * cloud_msg->height;
    cloud_msg->data.resize(total_size);
    memcpy(cloud_msg->data.data(), frame->points.data(), total_size);

    return 0;
  }

  static int updateRosPointFieldXYZI(ROS_POINTCLOUD2_PTR &cloud_msg) {
    if (cloud_msg == nullptr) {
      RS_SPDLOG_ERROR("Input cloud_msg is Nullptr !");
      return -1;
    }

    // Define the structure of the point fields in the point cloud
    cloud_msg->fields.resize(4);
    cloud_msg->fields[0].name = "x";
#if ENABLE_USE_PCL_LIDAR_MSG_TYPE
    cloud_msg->fields[0].offset = 0;
#else
    cloud_msg->fields[0].offset = 0;
#endif // ENABLE_USE_PCL_LIDAR_MSG_TYPE
#if defined(ROS_FOUND)
    cloud_msg->fields[0].datatype = sensor_msgs::PointField::FLOAT32;
#elif defined(ROS2_FOUND)
    cloud_msg->fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
#endif // ROS_ROS2_FOUND
    cloud_msg->fields[0].count = 1;

    cloud_msg->fields[1].name = "y";
#if ENABLE_USE_PCL_LIDAR_MSG_TYPE
    cloud_msg->fields[1].offset = 4;
#else
    cloud_msg->fields[0].offset = 4;
#endif // ENABLE_USE_PCL_LIDAR_MSG_TYPE
#if defined(ROS_FOUND)
    cloud_msg->fields[1].datatype = sensor_msgs::PointField::FLOAT32;
#elif defined(ROS2_FOUND)
    cloud_msg->fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
#endif // ROS_ROS2_FOUND
    cloud_msg->fields[1].count = 1;

    cloud_msg->fields[2].name = "z";
#if ENABLE_USE_PCL_LIDAR_MSG_TYPE
    cloud_msg->fields[2].offset = 8;
#else
    cloud_msg->fields[2].offset = 8;
#endif // ENABLE_USE_PCL_LIDAR_MSG_TYPE
#if defined(ROS_FOUND)
    cloud_msg->fields[2].datatype = sensor_msgs::PointField::FLOAT32;
#elif defined(ROS2_FOUND)
    cloud_msg->fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
#endif // ROS_ROS2_FOUND
    cloud_msg->fields[2].count = 1;

    cloud_msg->fields[3].name = "intensity";
#if ENABLE_USE_PCL_LIDAR_MSG_TYPE
    cloud_msg->fields[3].offset = 16;
#if defined(ROS_FOUND)
    cloud_msg->fields[3].datatype = sensor_msgs::PointField::FLOAT32;
#elif defined(ROS2_FOUND)
    cloud_msg->fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
#endif // ROS_ROS2_FOUND
#else
    cloud_msg->fields[3].offset = 12;
#if defined(ROS_FOUND)
    cloud_msg->fields[3].datatype = sensor_msgs::PointField::UINT8;
#elif defined(ROS2_FOUND)
    cloud_msg->fields[3].datatype = sensor_msgs::msg::PointField::UINT8;
#endif // ROS_ROS2_FOUND
#endif // ENABLE_USE_PCL_LIDAR_MSG_TYPE
    cloud_msg->fields[3].count = 1;

#if ENABLE_USE_PCL_LIDAR_MSG_TYPE
    cloud_msg->point_step = 32;
#else
    cloud_msg->point_step = 16;
#endif // ENABLE_USE_PCL_LIDAR_MSG_TYPE

    return 0;
  }

  static int updateRosPointFieldXYZIF(ROS_POINTCLOUD2_PTR &cloud_msg) {
    if (cloud_msg == nullptr) {
      RS_SPDLOG_ERROR("Input cloud_msg is Nullptr !");
      return -1;
    }

    // Define the structure of the point fields in the point cloud
    cloud_msg->fields.resize(5);
    cloud_msg->fields[0].name = "x";
#if ENABLE_USE_PCL_LIDAR_MSG_TYPE
    cloud_msg->fields[0].offset = 0;
#else
    cloud_msg->fields[0].offset = 0;
#endif // ENABLE_USE_PCL_LIDAR_MSG_TYPE
#if defined(ROS_FOUND)
    cloud_msg->fields[0].datatype = sensor_msgs::PointField::FLOAT32;
#elif defined(ROS2_FOUND)
    cloud_msg->fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
#endif // ROS_ROS2_FOUND
    cloud_msg->fields[0].count = 1;

    cloud_msg->fields[1].name = "y";
#if ENABLE_USE_PCL_LIDAR_MSG_TYPE
    cloud_msg->fields[1].offset = 4;
#else
    cloud_msg->fields[0].offset = 4;
#endif // ENABLE_USE_PCL_LIDAR_MSG_TYPE
#if defined(ROS_FOUND)
    cloud_msg->fields[1].datatype = sensor_msgs::PointField::FLOAT32;
#elif defined(ROS2_FOUND)
    cloud_msg->fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
#endif // ROS_ROS2_FOUND
    cloud_msg->fields[1].count = 1;

    cloud_msg->fields[2].name = "z";
#if ENABLE_USE_PCL_LIDAR_MSG_TYPE
    cloud_msg->fields[2].offset = 8;
#else
    cloud_msg->fields[2].offset = 8;
#endif // ENABLE_USE_PCL_LIDAR_MSG_TYPE
#if defined(ROS_FOUND)
    cloud_msg->fields[2].datatype = sensor_msgs::PointField::FLOAT32;
#elif defined(ROS2_FOUND)
    cloud_msg->fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
#endif // ROS_ROS2_FOUND
    cloud_msg->fields[2].count = 1;

    cloud_msg->fields[3].name = "intensity";
#if ENABLE_USE_PCL_LIDAR_MSG_TYPE
    cloud_msg->fields[3].offset = 16;
#if defined(ROS_FOUND)
    cloud_msg->fields[3].datatype = sensor_msgs::PointField::FLOAT32;
#elif defined(ROS2_FOUND)
    cloud_msg->fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
#endif // ROS_ROS2_FOUND
#else
    cloud_msg->fields[3].offset = 12;
#if defined(ROS_FOUND)
    cloud_msg->fields[3].datatype = sensor_msgs::PointField::UINT8;
#elif defined(ROS2_FOUND)
    cloud_msg->fields[3].datatype = sensor_msgs::msg::PointField::UINT8;
#endif // ROS_ROS2_FOUND
#endif // ENABLE_USE_PCL_LIDAR_MSG_TYPE
    cloud_msg->fields[3].count = 1;

    cloud_msg->fields[4].name = "feature";
#if ENABLE_USE_PCL_LIDAR_MSG_TYPE
    cloud_msg->fields[4].offset = 20;
#else
    cloud_msg->fields[4].offset = 13;
#endif // ENABLE_USE_PCL_LIDAR_MSG_TYPE
#if defined(ROS_FOUND)
    cloud_msg->fields[4].datatype = sensor_msgs::PointField::UINT8;
#elif defined(ROS2_FOUND)
    cloud_msg->fields[4].datatype = sensor_msgs::msg::PointField::UINT8;
#endif // ROS_ROS2_FOUND
    cloud_msg->fields[4].count = 1;

#if ENABLE_USE_PCL_LIDAR_MSG_TYPE
    cloud_msg->point_step = 32;
#else
    cloud_msg->point_step = 16;
#endif // ENABLE_USE_PCL_LIDAR_MSG_TYPE

    return 0;
  }

  static int updateRosPointFieldXYZIRT(ROS_POINTCLOUD2_PTR &cloud_msg) {
    if (cloud_msg == nullptr) {
      RS_SPDLOG_ERROR("Input cloud_msg is Nullptr !");
      return -1;
    }

    // Define the structure of the point fields in the point cloud
    cloud_msg->fields.resize(6);
    cloud_msg->fields[0].name = "x";
#if ENABLE_USE_PCL_LIDAR_MSG_TYPE
    cloud_msg->fields[0].offset = 0;
#else
    cloud_msg->fields[0].offset = 0;
#endif // ENABLE_USE_PCL_LIDAR_MSG_TYPE
#if defined(ROS_FOUND)
    cloud_msg->fields[0].datatype = sensor_msgs::PointField::FLOAT32;
#elif defined(ROS2_FOUND)
    cloud_msg->fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
#endif // ROS_ROS2_FOUND
    cloud_msg->fields[0].count = 1;

    cloud_msg->fields[1].name = "y";
#if ENABLE_USE_PCL_LIDAR_MSG_TYPE
    cloud_msg->fields[1].offset = 4;
#else
    cloud_msg->fields[0].offset = 4;
#endif // ENABLE_USE_PCL_LIDAR_MSG_TYPE
#if defined(ROS_FOUND)
    cloud_msg->fields[1].datatype = sensor_msgs::PointField::FLOAT32;
#elif defined(ROS2_FOUND)
    cloud_msg->fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
#endif // ROS_ROS2_FOUND
    cloud_msg->fields[1].count = 1;

    cloud_msg->fields[2].name = "z";
#if ENABLE_USE_PCL_LIDAR_MSG_TYPE
    cloud_msg->fields[2].offset = 8;
#else
    cloud_msg->fields[2].offset = 8;
#endif // ENABLE_USE_PCL_LIDAR_MSG_TYPE
#if defined(ROS_FOUND)
    cloud_msg->fields[2].datatype = sensor_msgs::PointField::FLOAT32;
#elif defined(ROS2_FOUND)
    cloud_msg->fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
#endif // ROS_ROS2_FOUND
    cloud_msg->fields[2].count = 1;

    cloud_msg->fields[3].name = "intensity";
#if ENABLE_USE_PCL_LIDAR_MSG_TYPE
    cloud_msg->fields[3].offset = 16;
#if defined(ROS_FOUND)
    cloud_msg->fields[3].datatype = sensor_msgs::PointField::FLOAT32;
#elif defined(ROS2_FOUND)
    cloud_msg->fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
#endif // ROS_ROS2_FOUND
#else
    cloud_msg->fields[3].offset = 12;
#if defined(ROS_FOUND)
    cloud_msg->fields[3].datatype = sensor_msgs::PointField::UINT8;
#elif defined(ROS2_FOUND)
    cloud_msg->fields[3].datatype = sensor_msgs::msg::PointField::UINT8;
#endif // ROS_ROS2_FOUND
#endif // ENABLE_USE_PCL_LIDAR_MSG_TYPE
    cloud_msg->fields[3].count = 1;

    cloud_msg->fields[4].name = "ring";
#if ENABLE_USE_PCL_LIDAR_MSG_TYPE
    cloud_msg->fields[4].offset = 20;
#else
    cloud_msg->fields[4].offset = 14;
#endif // ENABLE_USE_PCL_LIDAR_MSG_TYPE
#if defined(ROS_FOUND)
    cloud_msg->fields[4].datatype = sensor_msgs::PointField::UINT16;
#elif defined(ROS2_FOUND)
    cloud_msg->fields[4].datatype = sensor_msgs::msg::PointField::UINT16;
#endif // ROS_ROS2_FOUND
    cloud_msg->fields[4].count = 1;

    cloud_msg->fields[5].name = "timestamp";
#if ENABLE_USE_PCL_LIDAR_MSG_TYPE
    cloud_msg->fields[5].offset = 24;
#else
    cloud_msg->fields[5].offset = 16;
#endif // ENABLE_USE_PCL_LIDAR_MSG_TYPE
#if defined(ROS_FOUND)
    cloud_msg->fields[5].datatype = sensor_msgs::PointField::FLOAT64;
#elif defined(ROS2_FOUND)
    cloud_msg->fields[5].datatype = sensor_msgs::msg::PointField::FLOAT64;
#endif // ROS_ROS2_FOUND
    cloud_msg->fields[5].count = 1;

#if ENABLE_USE_PCL_LIDAR_MSG_TYPE
    cloud_msg->point_step = 32;
#else
    cloud_msg->point_step = 24;
#endif // ENABLE_USE_PCL_LIDAR_MSG_TYPE
    return 0;
  }

  static int updateRosPointFieldXYZIRTF(ROS_POINTCLOUD2_PTR &cloud_msg) {
    if (cloud_msg == nullptr) {
      RS_SPDLOG_ERROR("Input cloud_msg is Nullptr !");
      return -1;
    }

    // Define the structure of the point fields in the point cloud
    cloud_msg->fields.resize(7);
    cloud_msg->fields[0].name = "x";
#if ENABLE_USE_PCL_LIDAR_MSG_TYPE
    cloud_msg->fields[0].offset = 0;
#else
    cloud_msg->fields[0].offset = 0;
#endif // ENABLE_USE_PCL_LIDAR_MSG_TYPE
#if defined(ROS_FOUND)
    cloud_msg->fields[0].datatype = sensor_msgs::PointField::FLOAT32;
#elif defined(ROS2_FOUND)
    cloud_msg->fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
#endif // ROS_ROS2_FOUND
    cloud_msg->fields[0].count = 1;

    cloud_msg->fields[1].name = "y";
#if ENABLE_USE_PCL_LIDAR_MSG_TYPE
    cloud_msg->fields[1].offset = 4;
#else
    cloud_msg->fields[0].offset = 4;
#endif // ENABLE_USE_PCL_LIDAR_MSG_TYPE
#if defined(ROS_FOUND)
    cloud_msg->fields[1].datatype = sensor_msgs::PointField::FLOAT32;
#elif defined(ROS2_FOUND)
    cloud_msg->fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
#endif // ROS_ROS2_FOUND
    cloud_msg->fields[1].count = 1;

    cloud_msg->fields[2].name = "z";
#if ENABLE_USE_PCL_LIDAR_MSG_TYPE
    cloud_msg->fields[2].offset = 8;
#else
    cloud_msg->fields[2].offset = 8;
#endif // ENABLE_USE_PCL_LIDAR_MSG_TYPE
#if defined(ROS_FOUND)
    cloud_msg->fields[2].datatype = sensor_msgs::PointField::FLOAT32;
#elif defined(ROS2_FOUND)
    cloud_msg->fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
#endif // ROS_ROS2_FOUND
    cloud_msg->fields[2].count = 1;

    cloud_msg->fields[3].name = "intensity";
#if ENABLE_USE_PCL_LIDAR_MSG_TYPE
    cloud_msg->fields[3].offset = 16;
#if defined(ROS_FOUND)
    cloud_msg->fields[3].datatype = sensor_msgs::PointField::FLOAT32;
#elif defined(ROS2_FOUND)
    cloud_msg->fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
#endif // ROS_ROS2_FOUND
#else
    cloud_msg->fields[3].offset = 12;
#if defined(ROS_FOUND)
    cloud_msg->fields[3].datatype = sensor_msgs::PointField::UINT8;
#elif defined(ROS2_FOUND)
    cloud_msg->fields[3].datatype = sensor_msgs::msg::PointField::UINT8;
#endif // ROS_ROS2_FOUND
#endif // ENABLE_USE_PCL_LIDAR_MSG_TYPE
    cloud_msg->fields[3].count = 1;

    // PCL和自定义数据结构不一样: ring/feature字段
#if ENABLE_USE_PCL_LIDAR_MSG_TYPE
    cloud_msg->fields[5].name = "ring";
    cloud_msg->fields[5].offset = 20;
#if defined(ROS_FOUND)
    cloud_msg->fields[5].datatype = sensor_msgs::PointField::UINT16;
#elif defined(ROS2_FOUND)
    cloud_msg->fields[5].datatype = sensor_msgs::msg::PointField::UINT16;
#endif // ROS_ROS2_FOUND
    cloud_msg->fields[5].count = 1;

    cloud_msg->fields[4].name = "feature";
    cloud_msg->fields[4].offset = 22;
#if defined(ROS_FOUND)
    cloud_msg->fields[4].datatype = sensor_msgs::PointField::UINT8;
#elif defined(ROS2_FOUND)
    cloud_msg->fields[4].datatype = sensor_msgs::msg::PointField::UINT8;
#endif // ROS_ROS2_FOUND
    cloud_msg->fields[4].count = 1;
#else
    cloud_msg->fields[4].name = "feature";
    cloud_msg->fields[4].offset = 13;
#if defined(ROS_FOUND)
    cloud_msg->fields[4].datatype = sensor_msgs::PointField::UINT8;
#elif defined(ROS2_FOUND)
    cloud_msg->fields[4].datatype = sensor_msgs::msg::PointField::UINT8;
#endif // ROS_ROS2_FOUND
    cloud_msg->fields[4].count = 1;

    cloud_msg->fields[5].name = "ring";
    cloud_msg->fields[5].offset = 14;
#if defined(ROS_FOUND)
    cloud_msg->fields[5].datatype = sensor_msgs::PointField::UINT16;
#elif defined(ROS2_FOUND)
    cloud_msg->fields[5].datatype = sensor_msgs::msg::PointField::UINT16;
#endif // ROS_ROS2_FOUND
    cloud_msg->fields[5].count = 1;
#endif // ENABLE_USE_PCL_LIDAR_MSG_TYPE

    cloud_msg->fields[6].name = "timestamp";
#if ENABLE_USE_PCL_LIDAR_MSG_TYPE
    cloud_msg->fields[6].offset = 24;
#else
    cloud_msg->fields[6].offset = 16;
#endif // ENABLE_USE_PCL_LIDAR_MSG_TYPE
#if defined(ROS_FOUND)
    cloud_msg->fields[6].datatype = sensor_msgs::PointField::FLOAT64;
#elif defined(ROS2_FOUND)
    cloud_msg->fields[6].datatype = sensor_msgs::msg::PointField::FLOAT64;
#endif // ROS_ROS2_FOUND
    cloud_msg->fields[6].count = 1;

#if ENABLE_USE_PCL_LIDAR_MSG_TYPE
    cloud_msg->point_step = 32;
#else
    cloud_msg->point_step = 24;
#endif // ENABLE_USE_PCL_LIDAR_MSG_TYPE

    return 0;
  }

#if defined(ROS2_FOUND)
private:
  template <typename ZEROCOPY_MESSAGE_T>
  static int updateZeroCopyCloudMessageXYZI(
      const std::shared_ptr<PointCloudT<PointXYZI>> &frame,
      ZEROCOPY_MESSAGE_T *cloud_msg) {
    int ret = 0;
    // 构造PointField
    ret = updateZeroCopyPointFieldXYZI<ZEROCOPY_MESSAGE_T>(cloud_msg);
    if (ret != 0) {
      RS_SPDLOG_ERROR("Update Ros2 cloud_msg PointField Failed: ret = " +
                      std::to_string(ret));
      return -1;
    }

    // 构造其他内容
    ret = updateZeroCopyCloudMessageContent<PointXYZI, ZEROCOPY_MESSAGE_T>(
        frame, cloud_msg);
    if (ret != 0) {
      RS_SPDLOG_ERROR("Update Ros2 cloud_msg Content Failed: ret = " +
                      std::to_string(ret));
      return -2;
    }

    return 0;
  }

  template <typename ZEROCOPY_MESSAGE_T>
  static int updateZeroCopyCloudMessageXYZIF(
      const std::shared_ptr<PointCloudT<PointXYZIF>> &frame,
      ZEROCOPY_MESSAGE_T *cloud_msg) {
    int ret = 0;
    // 构造PointField
    ret = updateZeroCopyPointFieldXYZIF<ZEROCOPY_MESSAGE_T>(cloud_msg);
    if (ret != 0) {
      RS_SPDLOG_ERROR("Update Ros2 cloud_msg PointField Failed: ret = " +
                      std::to_string(ret));
      return -1;
    }

    // 构造其他内容
    ret = updateZeroCopyCloudMessageContent<PointXYZIF, ZEROCOPY_MESSAGE_T>(
        frame, cloud_msg);
    if (ret != 0) {
      RS_SPDLOG_ERROR("Update Ros2 cloud_msg Content Failed: ret = " +
                      std::to_string(ret));
      return -2;
    }

    return 0;
  }

  template <typename ZEROCOPY_MESSAGE_T>
  static int updateZeroCopyCloudMessageXYZIRT(
      const std::shared_ptr<PointCloudT<PointXYZIRT>> &frame,
      ZEROCOPY_MESSAGE_T *cloud_msg) {
    int ret = 0;

    // 构造PointField
    ret = updateZeroCopyPointFieldXYZIRT<ZEROCOPY_MESSAGE_T>(cloud_msg);
    if (ret != 0) {
      RS_SPDLOG_ERROR("Update Ros2 cloud_msg PointField Failed: ret = " +
                      std::to_string(ret));
      return -1;
    }

    // 构造其他内容
    ret = updateZeroCopyCloudMessageContent<PointXYZIRT, ZEROCOPY_MESSAGE_T>(
        frame, cloud_msg);
    if (ret != 0) {
      RS_SPDLOG_ERROR("Update Ros2 cloud_msg Content Failed: ret = " +
                      std::to_string(ret));
      return -2;
    }

    return 0;
  }

  template <typename ZEROCOPY_MESSAGE_T>
  static int updateZeroCopyCloudMessageXYZIRTF(
      const std::shared_ptr<PointCloudT<PointXYZIRTF>> &frame,
      ZEROCOPY_MESSAGE_T *cloud_msg) {
    int ret = 0;

    // 构造PointField
    ret = updateZeroCopyPointFieldXYZIRTF<ZEROCOPY_MESSAGE_T>(cloud_msg);
    if (ret != 0) {
      RS_SPDLOG_ERROR("Update Ros2 cloud_msg PointField Failed: ret = " +
                      std::to_string(ret));
      return -1;
    }

    // 构造其他内容
    ret = updateZeroCopyCloudMessageContent<PointXYZIRTF, ZEROCOPY_MESSAGE_T>(
        frame, cloud_msg);
    if (ret != 0) {
      RS_SPDLOG_ERROR("Update Ros2 cloud_msg Content Failed: ret = " +
                      std::to_string(ret));
      return -2;
    }

    return 0;
  }

  template <typename POINT_T, typename ZEROCOPY_MESSAGE_T>
  static int updateZeroCopyCloudMessageContent(
      const std::shared_ptr<PointCloudT<POINT_T>> &frame,
      ZEROCOPY_MESSAGE_T *cloud_msg) {
    if (frame == nullptr || cloud_msg == nullptr) {
      RS_SPDLOG_ERROR("Input frame or/and cloud_msg is Nullptr !");
      return -1;
    }

    // 构造Header
    const std::string &point_frame_id = frame->frame_id;
    const char *id_str = point_frame_id.c_str();
    std::copy(id_str, id_str + strlen(id_str) + 1,
              cloud_msg->header.frame_id.begin());
    cloud_msg->height = frame->height;
    cloud_msg->width = frame->width;
    cloud_msg->row_step = cloud_msg->point_step * cloud_msg->width;

    const size_t total_size = cloud_msg->row_step * cloud_msg->height;
    if (cloud_msg->data.size() < total_size) {
      return -2;
    }

    // 构造head
    double tail_stamp = frame->timestamp;
    uint32_t sec = static_cast<uint32_t>(tail_stamp);
    uint32_t nsec = static_cast<uint32_t>((tail_stamp - sec) * 1e9);
    auto custom_time = rclcpp::Time(sec, nsec);
    cloud_msg->header.stamp = custom_time;

    // Define the structure of the point fields in the point cloud
    cloud_msg->is_bigendian = isBigEndian();
    cloud_msg->is_dense = frame->is_dense;

    // 拷贝内容
    memcpy(cloud_msg->data.data(), frame->points.data(), total_size);

    return 0;
  }

  template <typename ZEROCOPY_MESSAGE_T>
  static int updateZeroCopyPointFieldXYZI(ZEROCOPY_MESSAGE_T *cloud_msg) {
    if (cloud_msg == nullptr) {
      RS_SPDLOG_ERROR("Input cloud_msg is Nullptr !");
      return -1;
    }

    const char *field_names[] = {"x", "y", "z", "intensity"};
    for (int i = 0; i < 4; i++) {
      std::copy(field_names[i], field_names[i] + strlen(field_names[i]) + 1,
                cloud_msg->fields[i].name.begin());
    }

    // x
#if ENABLE_USE_PCL_LIDAR_MSG_TYPE
    cloud_msg->fields[0].offset = 0;
#else
    cloud_msg->fields[0].offset = 0;
#endif // ENABLE_USE_PCL_LIDAR_MSG_TYPE
    cloud_msg->fields[0].datatype = robosense_msgs::msg::RsPointField::FLOAT32;
    cloud_msg->fields[0].count = 1;

    // y
#if ENABLE_USE_PCL_LIDAR_MSG_TYPE
    cloud_msg->fields[1].offset = 4;
#else
    cloud_msg->fields[0].offset = 4;
#endif // ENABLE_USE_PCL_LIDAR_MSG_TYPE
    cloud_msg->fields[1].datatype = robosense_msgs::msg::RsPointField::FLOAT32;
    cloud_msg->fields[1].count = 1;

    // z
#if ENABLE_USE_PCL_LIDAR_MSG_TYPE
    cloud_msg->fields[2].offset = 8;
#else
    cloud_msg->fields[2].offset = 8;
#endif // ENABLE_USE_PCL_LIDAR_MSG_TYPE
    cloud_msg->fields[2].datatype = robosense_msgs::msg::RsPointField::FLOAT32;
    cloud_msg->fields[2].count = 1;

    // intensity
#if ENABLE_USE_PCL_LIDAR_MSG_TYPE
    cloud_msg->fields[3].offset = 16;
    cloud_msg->fields[3].datatype = robosense_msgs::msg::RsPointField::FLOAT32;
#else
    cloud_msg->fields[3].offset = 12;
    cloud_msg->fields[3].datatype = robosense_msgs::msg::RsPointField::UINT8;
#endif // ENABLE_USE_PCL_LIDAR_MSG_TYPE
    cloud_msg->fields[3].count = 1;

#if ENABLE_USE_PCL_LIDAR_MSG_TYPE
    cloud_msg->point_step = 32;
#else
    cloud_msg->point_step = 16;
#endif // ENABLE_USE_PCL_LIDAR_MSG_TYPE

    return 0;
  }

  template <typename ZEROCOPY_MESSAGE_T>
  static int updateZeroCopyPointFieldXYZIF(ZEROCOPY_MESSAGE_T *cloud_msg) {
    if (cloud_msg == nullptr) {
      RS_SPDLOG_ERROR("Input cloud_msg is Nullptr !");
      return -1;
    }

    const char *field_names[] = {"x", "y", "z", "intensity", "feature"};
    for (int i = 0; i < 5; i++) {
      std::copy(field_names[i], field_names[i] + strlen(field_names[i]) + 1,
                cloud_msg->fields[i].name.begin());
    }

    // x
#if ENABLE_USE_PCL_LIDAR_MSG_TYPE
    cloud_msg->fields[0].offset = 0;
#else
    cloud_msg->fields[0].offset = 0;
#endif // ENABLE_USE_PCL_LIDAR_MSG_TYPE
    cloud_msg->fields[0].datatype = robosense_msgs::msg::RsPointField::FLOAT32;
    cloud_msg->fields[0].count = 1;

    // y
#if ENABLE_USE_PCL_LIDAR_MSG_TYPE
    cloud_msg->fields[1].offset = 4;
#else
    cloud_msg->fields[0].offset = 4;
#endif // ENABLE_USE_PCL_LIDAR_MSG_TYPE
    cloud_msg->fields[1].datatype = robosense_msgs::msg::RsPointField::FLOAT32;
    cloud_msg->fields[1].count = 1;

    // z
#if ENABLE_USE_PCL_LIDAR_MSG_TYPE
    cloud_msg->fields[2].offset = 8;
#else
    cloud_msg->fields[2].offset = 8;
#endif // ENABLE_USE_PCL_LIDAR_MSG_TYPE
    cloud_msg->fields[2].datatype = robosense_msgs::msg::RsPointField::FLOAT32;
    cloud_msg->fields[2].count = 1;

    // intensity
#if ENABLE_USE_PCL_LIDAR_MSG_TYPE
    cloud_msg->fields[3].offset = 16;
    cloud_msg->fields[3].datatype = robosense_msgs::msg::RsPointField::FLOAT32;
#else
    cloud_msg->fields[3].offset = 12;
    cloud_msg->fields[3].datatype = robosense_msgs::msg::RsPointField::UINT8;
#endif // ENABLE_USE_PCL_LIDAR_MSG_TYPE
    cloud_msg->fields[3].count = 1;

    // feature
#if ENABLE_USE_PCL_LIDAR_MSG_TYPE
    cloud_msg->fields[4].offset = 20;
#else
    cloud_msg->fields[4].offset = 13;
#endif // ENABLE_USE_PCL_LIDAR_MSG_TYPE
    cloud_msg->fields[3].datatype = robosense_msgs::msg::RsPointField::UINT8;
    cloud_msg->fields[3].count = 1;

#if ENABLE_USE_PCL_LIDAR_MSG_TYPE
    cloud_msg->point_step = 32;
#else
    cloud_msg->point_step = 16;
#endif // ENABLE_USE_PCL_LIDAR_MSG_TYPE
    return 0;
  }

  template <typename ZEROCOPY_MESSAGE_T>
  static int updateZeroCopyPointFieldXYZIRT(ZEROCOPY_MESSAGE_T *cloud_msg) {
    if (cloud_msg == nullptr) {
      RS_SPDLOG_ERROR("Input cloud_msg is Nullptr !");
      return -1;
    }
    const char *field_names[] = {"x",         "y",    "z",
                                 "intensity", "ring", "timestamp"};
    for (int i = 0; i < 6; i++) {
      std::copy(field_names[i], field_names[i] + strlen(field_names[i]) + 1,
                cloud_msg->fields[i].name.begin());
    }

    // x
#if ENABLE_USE_PCL_LIDAR_MSG_TYPE
    cloud_msg->fields[0].offset = 0;
#else
    cloud_msg->fields[0].offset = 0;
#endif // ENABLE_USE_PCL_LIDAR_MSG_TYPE
    cloud_msg->fields[0].datatype = robosense_msgs::msg::RsPointField::FLOAT32;
    cloud_msg->fields[0].count = 1;

    // y
#if ENABLE_USE_PCL_LIDAR_MSG_TYPE
    cloud_msg->fields[1].offset = 4;
#else
    cloud_msg->fields[0].offset = 4;
#endif // ENABLE_USE_PCL_LIDAR_MSG_TYPE
    cloud_msg->fields[1].datatype = robosense_msgs::msg::RsPointField::FLOAT32;
    cloud_msg->fields[1].count = 1;

    // z
#if ENABLE_USE_PCL_LIDAR_MSG_TYPE
    cloud_msg->fields[2].offset = 8;
#else
    cloud_msg->fields[2].offset = 8;
#endif // ENABLE_USE_PCL_LIDAR_MSG_TYPE
    cloud_msg->fields[2].datatype = robosense_msgs::msg::RsPointField::FLOAT32;
    cloud_msg->fields[2].count = 1;

    // intensity
#if ENABLE_USE_PCL_LIDAR_MSG_TYPE
    cloud_msg->fields[3].offset = 16;
    cloud_msg->fields[3].datatype = robosense_msgs::msg::RsPointField::FLOAT32;
#else
    cloud_msg->fields[3].offset = 12;
    cloud_msg->fields[3].datatype = robosense_msgs::msg::RsPointField::UINT8;
#endif // ENABLE_USE_PCL_LIDAR_MSG_TYPE
    cloud_msg->fields[3].count = 1;

    // ring
#if ENABLE_USE_PCL_LIDAR_MSG_TYPE
    cloud_msg->fields[4].offset = 20;
#else
    cloud_msg->fields[4].offset = 14;
#endif // ENABLE_USE_PCL_LIDAR_MSG_TYPE
    cloud_msg->fields[4].datatype = robosense_msgs::msg::RsPointField::UINT16;
    cloud_msg->fields[4].count = 1;

    // timestamp
#if ENABLE_USE_PCL_LIDAR_MSG_TYPE
    cloud_msg->fields[5].offset = 24;
#else
    cloud_msg->fields[5].offset = 16;
#endif // ENABLE_USE_PCL_LIDAR_MSG_TYPE
    cloud_msg->fields[5].datatype = robosense_msgs::msg::RsPointField::FLOAT64;
    cloud_msg->fields[5].count = 1;

#if ENABLE_USE_PCL_LIDAR_MSG_TYPE
    cloud_msg->point_step = 32;
#else
    cloud_msg->point_step = 24;
#endif // ENABLE_USE_PCL_LIDAR_MSG_TYPE
    return 0;
  }

  template <typename ZEROCOPY_MESSAGE_T>
  static int updateZeroCopyPointFieldXYZIRTF(ZEROCOPY_MESSAGE_T *cloud_msg) {
    if (cloud_msg == nullptr) {
      RS_SPDLOG_ERROR("Input cloud_msg is Nullptr !");
      return -1;
    }

    const char *field_names[] = {"x",    "y",       "z",        "intensity",
                                 "ring", "feature", "timestamp"};
    for (int i = 0; i < 7; i++) {
      std::copy(field_names[i], field_names[i] + strlen(field_names[i]) + 1,
                cloud_msg->fields[i].name.begin());
    }

    // x
#if ENABLE_USE_PCL_LIDAR_MSG_TYPE
    cloud_msg->fields[0].offset = 0;
#else
    cloud_msg->fields[0].offset = 0;
#endif // ENABLE_USE_PCL_LIDAR_MSG_TYPE
    cloud_msg->fields[0].datatype = robosense_msgs::msg::RsPointField::FLOAT32;
    cloud_msg->fields[0].count = 1;

    // y
#if ENABLE_USE_PCL_LIDAR_MSG_TYPE
    cloud_msg->fields[1].offset = 4;
#else
    cloud_msg->fields[0].offset = 4;
#endif // ENABLE_USE_PCL_LIDAR_MSG_TYPE
    cloud_msg->fields[1].datatype = robosense_msgs::msg::RsPointField::FLOAT32;
    cloud_msg->fields[1].count = 1;

    // z
#if ENABLE_USE_PCL_LIDAR_MSG_TYPE
    cloud_msg->fields[2].offset = 8;
#else
    cloud_msg->fields[2].offset = 8;
#endif // ENABLE_USE_PCL_LIDAR_MSG_TYPE
    cloud_msg->fields[2].datatype = robosense_msgs::msg::RsPointField::FLOAT32;
    cloud_msg->fields[2].count = 1;

    // intensity
#if ENABLE_USE_PCL_LIDAR_MSG_TYPE
    cloud_msg->fields[3].offset = 16;
    cloud_msg->fields[3].datatype = robosense_msgs::msg::RsPointField::FLOAT32;
#else
    cloud_msg->fields[3].offset = 12;
    cloud_msg->fields[3].datatype = robosense_msgs::msg::RsPointField::UINT8;
#endif // ENABLE_USE_PCL_LIDAR_MSG_TYPE
    cloud_msg->fields[3].count = 1;

    // PCL和自定义数据结构不一样: ring/feature字段
#if ENABLE_USE_PCL_LIDAR_MSG_TYPE
    // ring
    cloud_msg->fields[4].offset = 20;
    cloud_msg->fields[4].datatype = robosense_msgs::msg::RsPointField::UINT16;
    cloud_msg->fields[4].count = 1;

    // feature
    cloud_msg->fields[5].offset = 22;
    cloud_msg->fields[5].datatype = robosense_msgs::msg::RsPointField::UINT8;
    cloud_msg->fields[5].count = 1;
#else
    // feature
    cloud_msg->fields[4].offset = 13;
    cloud_msg->fields[4].datatype = robosense_msgs::msg::RsPointField::UINT8;
    cloud_msg->fields[4].count = 1;

    // ring
    cloud_msg->fields[5].offset = 14;
    cloud_msg->fields[5].datatype = robosense_msgs::msg::RsPointField::UINT16;
    cloud_msg->fields[5].count = 1;
#endif // ENABLE_USE_PCL_LIDAR_MSG_TYPE

    // timestamp
#if ENABLE_USE_PCL_LIDAR_MSG_TYPE
    cloud_msg->fields[6].offset = 24;
#else
    cloud_msg->fields[6].offset = 16;
#endif // ENABLE_USE_PCL_LIDAR_MSG_TYPE
    cloud_msg->fields[6].datatype = robosense_msgs::msg::RsPointField::FLOAT64;
    cloud_msg->fields[6].count = 1;

#if ENABLE_USE_PCL_LIDAR_MSG_TYPE
    cloud_msg->point_step = 32;
#else
    cloud_msg->point_step = 24;
#endif // ENABLE_USE_PCL_LIDAR_MSG_TYPE
    return 0;
  }
#endif // defined(ROS2_FOUND)
};

} // namespace convert
} // namespace robosense

#endif // RSCONVERTMANAGER_HPP
