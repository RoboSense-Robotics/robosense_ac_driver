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
#ifndef DEVICEMANAGER_H
#define DEVICEMANAGER_H

// #include "hyper_vision/common/common.h"
#include "libusb/libusb.h"
#include "rs_driver/api/lidar_driver.hpp"
#include "rs_driver/msg/pcl_point_cloud_msg.hpp"
#include <atomic>
#include <fstream>
#include <functional>
#include <list>
#include <queue>
#include <set>
#include <string>

// 定义点云类型
using RsPointXYZIRT = PointXYZIRT;

#define RS_TIMESTAMP_NS                                                        \
  (std::chrono::time_point_cast<std::chrono::nanoseconds>(                     \
       std::chrono::system_clock::now()))                                      \
      .time_since_epoch()                                                      \
      .count();

// 进行类型映射
namespace robosense {
namespace lidar {
using ImageData = ImageMsg;
using MonoImageData = MonoImageMsg;
using StereoImageData = StereoImageMsg;
} // namespace lidar
} // namespace robosense

namespace robosense {
namespace device {

typedef enum DeviceEventType {
  DEVICE_EVENT_DETACH = 0,
  DEVICE_EVENT_ATTACH,

  DEVICE_EVENT_UNKNOWN = 255,
} DeviceEventType_t;

typedef struct DeviceEvent {
  DeviceEventType_t event_type;
  uint32_t uuid_size;
  char uuid[128];
  robosense::lidar::LidarType lidar_type;
} DeviceEvent_t;

typedef enum DeviceInterfaceType {
  DEVICE_INTERFACE_USB = 0,
  DEVICE_INTERFACE_GMSL,
} DeviceInterfaceType_t;

class RSDeviceInterfaceUtil {
public:
  using Ptr = std::shared_ptr<RSDeviceInterfaceUtil>;
  using ConstPtr = std::shared_ptr<const RSDeviceInterfaceUtil>;

public:
  RSDeviceInterfaceUtil() = default;
  ~RSDeviceInterfaceUtil() = default;

public:
  static DeviceInterfaceType
  fromStringToDeviceInterfaceType(const std::string &stype) {
    DeviceInterfaceType type = DeviceInterfaceType::DEVICE_INTERFACE_USB;
    if (stype == "gmsl") {
      type = DeviceInterfaceType::DEVICE_INTERFACE_GMSL;
    }
    return type;
  }
};

class RSDeviceOpenConfig {
public:
  using Ptr = std::shared_ptr<RSDeviceOpenConfig>;
  using ConstPtr = std::shared_ptr<const RSDeviceOpenConfig>;

public:
  RSDeviceOpenConfig() { reset(); }
  ~RSDeviceOpenConfig() {}

public:
  void reset() {
    device_uuid.clear();
    device_path.clear();
    image_width = 1920;
    image_height = 1080;
    image_input_fps = 30;
    imu_input_fps = 200;
    image_format = robosense::lidar::frame_format::FRAME_FORMAT_RGB24;
    lidar_type = robosense::lidar::LidarType::RS_AC1;
    enable_use_lidar_clock = true;
    enable_use_dense_points = false;
    enable_use_first_point_ts = false;
    angle_calib_basic_dir_path.clear();
    enable_pointcloud_send = true;
    enable_ac1_image_send = true;
    enable_ac2_left_image_send = true;
    enable_ac2_right_image_send = true;
    enable_imu_send = true;
  }

public:
  std::string device_uuid;
  std::string device_path;
  int32_t image_width;
  int32_t image_height;
  int32_t image_input_fps;
  int32_t imu_input_fps;
  robosense::lidar::InputType input_type;
  robosense::lidar::frame_format image_format;
  robosense::lidar::LidarType lidar_type;
  bool enable_use_lidar_clock;
  bool enable_use_dense_points;
  bool enable_use_first_point_ts;
  std::string angle_calib_basic_dir_path;
  bool enable_pointcloud_send;
  bool enable_ac1_image_send;
  bool enable_ac2_left_image_send;
  bool enable_ac2_right_image_send;
  bool enable_imu_send;
#if defined(ENABLE_SUPPORT_RS_DRIVER_ALGORITHM)
  robosense::lidar::AlgorithmParam algorithm_param;
#endif // defined(ENABLE_SUPPORT_RS_DRIVER_ALGORITHM)
};

enum class RS_CHANNEL_ID_TYPE {
  RS_CHANNEL_ID_UNKNOWN = 0,

  RS_CHANNEL_ID_POINTCLOUD,
  RS_CHANNEL_ID_POINTCLOUD_AC2_WAVE2,
  RS_CHANNEL_ID_RGB_IMAGE,
  RS_CHANNEL_ID_RGB_LEFT_IMAGE,
  RS_CHANNEL_ID_RGB_RIGHT_IMAGE,
  RS_CHANNEL_ID_RGB_RECTIFY_IMAGE,
  RS_CHANNEL_ID_RGB_RECTIFY_LEFT_IMAGE,
  RS_CHANNEL_ID_RGB_RECTIFY_RIGHT_IMAGE,
  RS_CHANNEL_ID_JPEG_IMAGE,
  RS_CHANNEL_ID_JPEG_LEFT_IMAGE,
  RS_CHANNEL_ID_JPEG_RIGHT_IMAGE,
  RS_CHANNEL_ID_JPEG_RECTIFY_IMAGE,
  RS_CHANNEL_ID_JPEG_RECTIFY_LEFT_IMAGE,
  RS_CHANNEL_ID_JPEG_RECTIFY_RIGHT_IMAGE,
  RS_CHANNEL_ID_IMU,
};

class RSTimestampItem {
public:
  using Ptr = std::shared_ptr<RSTimestampItem>;
  using ConstPtr = std::shared_ptr<const RSTimestampItem>;

public:
  RSTimestampItem() = default;
  ~RSTimestampItem() = default;

public:
  RS_CHANNEL_ID_TYPE channel_id = RS_CHANNEL_ID_TYPE::RS_CHANNEL_ID_UNKNOWN;
  uint64_t message_timestamp_ns = 0; // ROS消息时间
  uint64_t timestamp_ns = 0;         // 驱动时间戳
  uint64_t sot_timestamp_ns = 0;     // 驱动SOT时间戳
  uint64_t sot_timestamp_rt_ns = 0;  // 驱动SOT_RT时间戳
  uint64_t receive_timestamp_ns = 0;
  uint64_t process_timestamp_ns = 0;
  uint64_t publish_timestamp_ns = 0;
};

class RSTimestampManager {
public:
  using Ptr = std::shared_ptr<RSTimestampManager>;
  using ConstPtr = std::shared_ptr<const RSTimestampManager>;

public:
  RSTimestampManager() {}
  ~RSTimestampManager() {
    {
      std::lock_guard<std::mutex> lg(flush_mtx_);
      is_running_ = false;
      flush_cond_.notify_all();
    }

    if (thread_ptr_ && thread_ptr_->joinable()) {
      thread_ptr_->join();
    }
    thread_ptr_.reset();
  }

public:
  int init(const std::string &timestamp_file_path,
           const uint32_t output_block_size = 1000) {
    if (timestamp_file_path.empty()) {
      RS_ERROR << "Input Timestamp File Path Is Empty !" << RS_REND;
      return -1;
    }
    output_block_size_ = output_block_size;
    timestamp_file_path_ = timestamp_file_path;

    int ret = init();
    if (ret != 0) {
      RS_ERROR << "RSTimestampManager Initial Failed: ret = " << ret << RS_REND;
      return -2;
    }

    return 0;
  }

  int addChannelId(const RS_CHANNEL_ID_TYPE id, const std::string &topic_name) {
    if (channel_id_mapper_.find(id) == channel_id_mapper_.end()) {
      channel_id_mapper_.insert({id, topic_name});
    }
    return 0;
  }

  int addTimestamp(const RSTimestampItem::Ptr &timestampPtr) {
    if (timestampPtr) {
      if (channel_id_mapper_.find(timestampPtr->channel_id) !=
          channel_id_mapper_.end()) {
        std::lock_guard<std::mutex> lg(flush_mtx_);
        active_buffer_->push_back(timestampPtr);
        if (active_buffer_->size() >= output_block_size_) {
          flush_buffer_.push(active_buffer_);
          flush_cond_.notify_one();
          active_buffer_ = std::make_shared<
              std::list<robosense::device::RSTimestampItem::Ptr>>();
        }
      }
    }
    return 0;
  }

private:
  int init() {
    try {
      ofstr_ptr_.reset(new std::ofstream(
          timestamp_file_path_, std::ios_base::out | std::ios_base::binary));
    } catch (...) {
      RS_ERROR << "Malloc fstream Failed: timestamp_file_path_ = "
               << timestamp_file_path_ << RS_REND;
      return -1;
    }

    if (!ofstr_ptr_->is_open()) {
      RS_ERROR << "Open File: timestamp_file_path_ = " << timestamp_file_path_
               << " To Write Failed !" << RS_REND;
      return -2;
    }
    (*ofstr_ptr_) << "topic_name, timestamp_ns(ns), sot_timestamp(ns), "
                     "sot_timestamp_rt(ns), "
                     "message_timestamp(ns), receive_timestamp(ns), "
                     "process_timestamp(ns), publish_timestamp(ns)"
                  << std::endl;

    try {
      active_buffer_.reset(
          new std::list<robosense::device::RSTimestampItem::Ptr>());
    } catch (...) {
      RS_ERROR << "Malloc Active Buffer Failed !";
      return -3;
    }

    try {
      is_running_ = true;
      thread_ptr_.reset(new std::thread(&RSTimestampManager::workThread, this));
    } catch (...) {
      is_running_ = false;
      RS_ERROR << "Malloc Work Thread Failed !" << RS_REND;
      return -4;
    }

    return 0;
  }

  void workThread() {
    while (is_running_) {
      // 交换到临时变量
      std::shared_ptr<std::list<robosense::device::RSTimestampItem::Ptr>>
          flush_buffer;
      {
        std::unique_lock<std::mutex> lg(flush_mtx_);
        flush_cond_.wait(
            lg, [this] { return !flush_buffer_.empty() || !is_running_; });
        if (!is_running_) {
          break;
        }
        flush_buffer = flush_buffer_.front();
        flush_buffer_.pop();
      }

      if (!flush_buffer) {
        continue;
      }
      for (auto iter = flush_buffer->begin(); iter != flush_buffer->end();
           ++iter) {
        const auto &timestampPtr = *iter;
        if (timestampPtr == nullptr) {
          continue;
        }

        (*ofstr_ptr_) << channel_id_mapper_[timestampPtr->channel_id] << ","
                      << std::to_string(timestampPtr->timestamp_ns) << ","
                      << std::to_string(timestampPtr->sot_timestamp_ns) << ","
                      << std::to_string(timestampPtr->sot_timestamp_rt_ns)
                      << ","
                      << std::to_string(timestampPtr->message_timestamp_ns)
                      << ","
                      << std::to_string(timestampPtr->receive_timestamp_ns)
                      << ","
                      << std::to_string(timestampPtr->process_timestamp_ns)
                      << ","
                      << std::to_string(timestampPtr->publish_timestamp_ns)
                      << std::endl;
      }
      ofstr_ptr_->flush();
    }
  }

private:
  std::map<RS_CHANNEL_ID_TYPE, std::string> channel_id_mapper_;
  std::shared_ptr<std::ofstream> ofstr_ptr_;
  std::string timestamp_file_path_;
  uint32_t output_block_size_;
  std::mutex flush_mtx_;
  std::condition_variable flush_cond_;
  bool is_running_;
  std::shared_ptr<std::thread> thread_ptr_;
  std::shared_ptr<std::list<robosense::device::RSTimestampItem::Ptr>>
      active_buffer_;
  std::queue<
      std::shared_ptr<std::list<robosense::device::RSTimestampItem::Ptr>>>
      flush_buffer_;
};

class DeviceManager : public std::enable_shared_from_this<DeviceManager> {
public:
  using Ptr = std::shared_ptr<DeviceManager>;
  using ConstPtr = std::shared_ptr<const DeviceManager>;

public:
  using RS_DEVICE_EVENT_CALLBACK =
      std::function<void(const DeviceEvent &deviceEvent)>;

  using RS_DEVICE_POINTCLOUD_CALLBACK =
      std::function<void(const std::shared_ptr<PointCloudT<RsPointXYZIRT>> &,
                         const std::string &, const RSTimestampItem::Ptr &)>;

  using RS_DEVICE_IMAGE_CALLBACK =
      std::function<void(const std::shared_ptr<robosense::lidar::ImageData> &,
                         const std::string &, const RSTimestampItem::Ptr &)>;

  using RS_DEVICE_IMU_CALLBACK =
      std::function<void(const std::shared_ptr<robosense::lidar::ImuData> &,
                         const std::string &, const RSTimestampItem::Ptr &)>;

  using RS_DEVICE_EXCEPTION_CALLBACK =
      std::function<void(const robosense::lidar::Error &)>;

private:
  using RS_DEVICE_DRIVER =
      robosense::lidar::LidarDriver<PointCloudT<RsPointXYZIRT>>;
  using RS_DEVICE_DRIVER_PTR = std::shared_ptr<RS_DEVICE_DRIVER>;

public:
  class DeviceInfoItem {
  public:
    using Ptr = std::shared_ptr<DeviceInfoItem>;
    using ConstPtr = std::shared_ptr<const DeviceInfoItem>;

  public:
    DeviceInfoItem() {
      uuid.clear();
      is_attach = false;
      is_usb_mode = false;
      i_sn = 0;
      driver_ptr = nullptr;
      is_pause = true;
      lidar_type = robosense::lidar::LidarType::RS_AC1;
    }

    ~DeviceInfoItem() = default;

  public:
    std::string uuid;
    bool is_attach;
    bool is_usb_mode;
    uint8_t i_sn;
    RS_DEVICE_DRIVER_PTR driver_ptr;
    bool is_pause;
    robosense::lidar::LidarType lidar_type;
  };

public:
  DeviceManager();

  ~DeviceManager();

  bool init(const robosense::device::DeviceInterfaceType deviceInterfaceType,
            const bool isEnableDebug = false);

  int stop();

  int openDevice(const RSDeviceOpenConfig &device_config);

  int closeDevice(const std::string &device_uuid, const bool isMetux = true);

  int pauseDevice(const std::string &device_uuid, const bool isPauseOp);

  int closeDevices();

  std::set<std::string> getDevices();

  bool getDeviceType(const std::string &uuid, bool &is_usb_mode);

  bool getDeviceInfo(const std::string &uuid,
                     robosense::lidar::DeviceInfo &device_info);

  void regDeviceEventCallback(const RS_DEVICE_EVENT_CALLBACK &event_cb);

  void regPointCloudCallback(const RS_DEVICE_POINTCLOUD_CALLBACK &pc_cb);

  void regImageDataCallback(const RS_DEVICE_IMAGE_CALLBACK &image_cb);

  void regImuDataCallback(const RS_DEVICE_IMU_CALLBACK &imu_cb);

  void regExceptionCallback(const RS_DEVICE_EXCEPTION_CALLBACK &except_cb);

private:
  int findDevices(std::map<std::string, DeviceInfoItem> &devices_map);
  std::shared_ptr<PointCloudT<RsPointXYZIRT>> localGetPointCloudCallback();
  std::shared_ptr<robosense::lidar::ImageData> localGetImageDataCallback();
  std::shared_ptr<robosense::lidar::ImageData> localGetImageDataCallback2();
  std::shared_ptr<robosense::lidar::ImuData> localGetImuDataCallback();
  void localRunPointCloudCallback(
      const std::shared_ptr<PointCloudT<RsPointXYZIRT>> &msgPtr,
      const uint32_t &uuid_id, const RSTimestampItem::Ptr &timestampPtr);
  void localRunImageCallback(
      const std::shared_ptr<robosense::lidar::ImageData> &msgPtr,
      const uint32_t &uuid_id, const RSTimestampItem::Ptr &timestampPtr);
  void
  localRunImuCallback(const std::shared_ptr<robosense::lidar::ImuData> &msgPtr,
                      const uint32_t &uuid_id,
                      const RSTimestampItem::Ptr &timestampPtr);
  void localRunExceptCallback(const robosense::lidar::Error &error);
  void hotplugWorkThread();
  void processPointCloudQueue();
  void processImageQueue();
  void processImuQueue();

private:
  std::thread _m_thread;
  std::thread _usb_thread;
  libusb_context *_usb_ctx;
  int _kill_handler_thread;
  bool _start;
  bool _inited;
  RS_DEVICE_EVENT_CALLBACK _event_cb;
  RS_DEVICE_POINTCLOUD_CALLBACK _pc_cb;
  RS_DEVICE_IMAGE_CALLBACK _image_cb;
  RS_DEVICE_IMU_CALLBACK _imu_cb;
  RS_DEVICE_EXCEPTION_CALLBACK _except_cb;

  std::mutex _devices_map_mtx;
  std::map<std::string, DeviceInfoItem> _devices_map;

  bool _enable_debug;
  std::atomic<bool> _is_stoping_;

  robosense::device::DeviceInterfaceType _deviceInterfaceType;

private:
  const uint32_t VENDOR_ID = 0x3840;
  const uint32_t PRODUCT_ID = 0x1010;
  const uint32_t PRODUCT_ID2 = 0x1020;

private:
  template <typename MESSAGE_T> class RSMessageBufferItem {
  public:
    using Ptr = std::shared_ptr<RSMessageBufferItem<MESSAGE_T>>;
    using ConstPtr = std::shared_ptr<const RSMessageBufferItem<MESSAGE_T>>;

  public:
    RSMessageBufferItem() = default;
    ~RSMessageBufferItem() = default;

  public:
    uint32_t uuid_id;
    std::shared_ptr<MESSAGE_T> msgPtr;
    RSTimestampItem::Ptr timestampPtr;
  };

  // 消息处理队列
  std::queue<RSMessageBufferItem<PointCloudT<RsPointXYZIRT>>> _pointCloudQueue;
  std::queue<RSMessageBufferItem<robosense::lidar::ImageData>> _imageQueue;
  std::queue<RSMessageBufferItem<robosense::lidar::ImuData>> _imuQueue;

  uint32_t _global_uuid_id = 0;
  std::map<uint32_t, std::string> _uuid_id_mapper;

  // 队列互斥锁
  std::mutex _pointCloudQueueMutex;
  std::mutex _imageQueueMutex;
  std::mutex _imuQueueMutex;

  // 条件变量
  std::condition_variable _pointCloudQueueCond;
  std::condition_variable _imageQueueCond;
  std::condition_variable _imuQueueCond;

  // 线程停止标志
  std::atomic<bool> _stopProcessingThreads;

  // 消息处理线程
  std::thread _pointCloudProcessingThread;
  std::thread _imageProcessingThread;
  std::thread _imuProcessingThread;
};

} // namespace device
} // namespace robosense

#endif // DEVICEMANAGER_H
