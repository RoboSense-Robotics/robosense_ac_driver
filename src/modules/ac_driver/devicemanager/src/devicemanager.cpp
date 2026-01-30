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
#include "hyper_vision/devicemanager/devicemanager.h"

namespace robosense {
namespace device {

DeviceManager::DeviceManager() {
  _usb_ctx = nullptr;
  _kill_handler_thread = 1;
  _start = false;
  _inited = false;
  _event_cb = nullptr;
  _stopProcessingThreads = false;

  // 启动消息处理线程
  _pointCloudProcessingThread =
      std::thread(&DeviceManager::processPointCloudQueue, this);
  _imageProcessingThread = std::thread(&DeviceManager::processImageQueue, this);
  _imuProcessingThread = std::thread(&DeviceManager::processImuQueue, this);
}

DeviceManager::~DeviceManager() { stop(); }

bool DeviceManager::init(
    const robosense::device::DeviceInterfaceType deviceInterfaceType,
    const bool isEnableDebug) {
  if (_inited) {
    return true;
  }
  _is_stoping_ = false;

  _deviceInterfaceType = deviceInterfaceType;
  _enable_debug = isEnableDebug;
  RS_SPDLOG_INFO("DeviceManager: _enable_debug = " +
                 std::to_string(_enable_debug));

  int res;
  if (_deviceInterfaceType ==
      robosense::device::DeviceInterfaceType::DEVICE_INTERFACE_USB) {
    if (!_usb_ctx) {
      res = libusb_init(&_usb_ctx);
      if (res < 0) {
        _usb_ctx = nullptr;
        RS_SPDLOG_ERROR("DeviceManager: Initial USB Context Failed: res = " +
                        std::to_string(res));
        return false;
      }
    }

    auto thread_usb_event = [this](void *ptr) {
      struct timeval tv = {0, 100000};
      while (!_kill_handler_thread) {
        libusb_handle_events_timeout_completed(_usb_ctx, &tv,
                                               &_kill_handler_thread);
      }
    };

    _kill_handler_thread = 0;
    try {
      _usb_thread = std::thread(thread_usb_event, this);
    } catch (const std::system_error &e) {
      _kill_handler_thread = 0;
      RS_SPDLOG_ERROR("DeviceManager: Create USB Event Thread Failed !");
      return false;
    }

    _start = true;
    try {
      _m_thread = std::thread(&DeviceManager::hotplugWorkThread, this);
    } catch (const std::system_error &e) {
      _start = false;
      RS_SPDLOG_ERROR("DeviceManager: Create Device Hotplug Thread Failed !");
      return false;
    }
  }

  _inited = true;

  return true;
}

int DeviceManager::stop() {
  if (!_inited) {
    return 0;
  }
  _is_stoping_ = true;

  if (_deviceInterfaceType ==
      robosense::device::DeviceInterfaceType::DEVICE_INTERFACE_USB) {
    if (_start) {
      _start = false;
      if (_m_thread.joinable()) {
        _m_thread.join();
      }
    }
  }

  int ret = closeDevices();
  if (ret != 0) {
    RS_SPDLOG_ERROR("DeviceManager: Close All Devices Failed !");
    return -1;
  } else {
    RS_SPDLOG_INFO("DeviceManager: Close All Device Successed !");
  }

  if (_deviceInterfaceType ==
      robosense::device::DeviceInterfaceType::DEVICE_INTERFACE_USB) {
    if (!_kill_handler_thread) {
      _kill_handler_thread = 1;
      if (_usb_thread.joinable()) {
        _usb_thread.join();
      }
    }

    if (_usb_ctx) {
      libusb_exit(_usb_ctx);
      _usb_ctx = nullptr;
    }
  }

  if (_stopProcessingThreads == false) {
    _stopProcessingThreads = true;
    _pointCloudQueueCond.notify_all();
    if (_pointCloudProcessingThread.joinable()) {
      _pointCloudProcessingThread.join();
    }
    _imageQueueCond.notify_all();
    if (_imageProcessingThread.joinable()) {
      _imageProcessingThread.join();
    }
    _imuQueueCond.notify_all();
    if (_imuProcessingThread.joinable()) {
      _imuProcessingThread.join();
    }
  }

  _inited = false;

  return 0;
}

int DeviceManager::openDevice(const RSDeviceOpenConfig &device_config) {
  std::lock_guard<std::mutex> lg(_devices_map_mtx);
  auto iterMap = _devices_map.find(device_config.device_uuid);
  switch (_deviceInterfaceType) {
  case robosense::device::DeviceInterfaceType::DEVICE_INTERFACE_USB: {
    if (iterMap == _devices_map.end()) {
      RS_SPDLOG_ERROR("DeviceManager: device uuid = " +
                      device_config.device_uuid + " Not Attach !");
      return -1;
    }
    break;
  }
  case robosense::device::DeviceInterfaceType::DEVICE_INTERFACE_GMSL: {
    if (iterMap == _devices_map.end()) {
      DeviceInfoItem item;
      item.uuid = device_config.device_uuid;
      item.lidar_type = device_config.lidar_type;
      _devices_map.insert({device_config.device_uuid, item});
      iterMap = _devices_map.find(device_config.device_uuid);
    }
    break;
  }
  }

  if (iterMap->second.driver_ptr != nullptr) {
    RS_SPDLOG_INFO("DeviceManager: device uuid = " + device_config.device_uuid +
                   " Already Open, Not Need Open Again !");
    return 0;
  }

  RS_DEVICE_DRIVER_PTR driver_ptr;
  try {
    driver_ptr.reset(new RS_DEVICE_DRIVER());
  } catch (...) {
    RS_SPDLOG_ERROR(
        "DeviceManager: device uuid = " + device_config.device_uuid +
        " Malloc Device Driver Failed !");
    return -2;
  }
  std::weak_ptr<DeviceManager> weak_this = shared_from_this();

  // 将设备UUID 映射为 ID
  const std::string &regDeviceUUID = device_config.device_uuid;
  uint32_t regDeviceUUID_ID = 0;
  bool is_match = false;
  for (auto iterMap = _uuid_id_mapper.begin(); iterMap != _uuid_id_mapper.end();
       ++iterMap) {
    if (iterMap->second == regDeviceUUID) {
      is_match = true;
      regDeviceUUID_ID = iterMap->first;
      break;
    }
  }
  if (!is_match) {
    _global_uuid_id++;
    regDeviceUUID_ID = _global_uuid_id;
    _uuid_id_mapper.insert({regDeviceUUID_ID, regDeviceUUID});
  }

  // 获取lidar_type
  const auto &lidar_type = iterMap->second.lidar_type;
  const auto &get_pc_cb =
      std::bind(&DeviceManager::localGetPointCloudCallback, this);
  const auto &put_pc_cb =
      [weak_this, regDeviceUUID_ID](
          const std::shared_ptr<PointCloudT<RsPointXYZIRT>> &msgPtr) {
        // AERROR << "RUN HERE";
        uint64_t timestamp_ns = RS_TIMESTAMP_NS;
        auto share_this = weak_this.lock();
        if (!share_this || !msgPtr) {
          return;
        }
        // AERROR << "RUN HERE";
        RSTimestampItem::Ptr timestampItemPtr(new RSTimestampItem());
        timestampItemPtr->receive_timestamp_ns = timestamp_ns;
        timestampItemPtr->timestamp_ns = msgPtr->timestamp * 1e9;
        timestampItemPtr->sot_timestamp_ns = msgPtr->sot_timestamp * 1e9;
        timestampItemPtr->sot_timestamp_rt_ns = msgPtr->sot_timestamp_rt * 1e9;
        share_this->localRunPointCloudCallback(msgPtr, regDeviceUUID_ID,
                                               timestampItemPtr);
      };
  driver_ptr->regPointCloudCallback(get_pc_cb, put_pc_cb);

  const auto &put_image_cb =
      [weak_this, regDeviceUUID_ID](
          const std::shared_ptr<robosense::lidar::ImageData> &msgPtr) {
        // AERROR << "RUN HERE";
        uint64_t timestamp_ns = RS_TIMESTAMP_NS;
        auto share_this = weak_this.lock();
        if (!share_this || !msgPtr) {
          return;
        }
        // RS_SPDLOG_INFO("RUN HERE: " << msgPtr->frame_format );
        // AERROR << "RUN HERE";
        RSTimestampItem::Ptr timestampItemPtr(new RSTimestampItem());
        timestampItemPtr->receive_timestamp_ns = timestamp_ns;
        timestampItemPtr->timestamp_ns = msgPtr->timestamp * 1e9;
        timestampItemPtr->sot_timestamp_ns = msgPtr->sot_timestamp * 1e9;
        timestampItemPtr->sot_timestamp_rt_ns = msgPtr->sot_timestamp_rt * 1e9;
        share_this->localRunImageCallback(msgPtr, regDeviceUUID_ID,
                                          timestampItemPtr);
      };

  if (lidar_type == robosense::lidar::LidarType::RS_AC1) {
    const auto &get_image_cb =
        std::bind(&DeviceManager::localGetImageDataCallback, this);
    driver_ptr->regImageDataCallback(get_image_cb, put_image_cb);
  } else if (lidar_type == robosense::lidar::LidarType::RS_AC2) {
    const auto &get_image_cb =
        std::bind(&DeviceManager::localGetImageDataCallback2, this);
    driver_ptr->regImageDataCallback(get_image_cb, put_image_cb);
  }

  const auto &get_imu_cb =
      std::bind(&DeviceManager::localGetImuDataCallback, this);
  const auto &put_imu_cb =
      [weak_this, regDeviceUUID_ID](
          const std::shared_ptr<robosense::lidar::ImuData> &msgPtr) {
        // AERROR << "RUN HERE";
        uint64_t timestamp_ns = RS_TIMESTAMP_NS;
        auto share_this = weak_this.lock();
        if (!share_this || !msgPtr) {
          return;
        }
        // AERROR << "RUN HERE";
        RSTimestampItem::Ptr timestampItemPtr(new RSTimestampItem());
        timestampItemPtr->receive_timestamp_ns = timestamp_ns;
        timestampItemPtr->timestamp_ns = msgPtr->timestamp * 1e9;
        timestampItemPtr->sot_timestamp_ns = 0;
        timestampItemPtr->sot_timestamp_rt_ns = 0;
        share_this->localRunImuCallback(msgPtr, regDeviceUUID_ID,
                                        timestampItemPtr);
      };
  driver_ptr->regImuDataCallback(get_imu_cb, put_imu_cb);

  driver_ptr->regExceptionCallback(std::bind(
      &DeviceManager::localRunExceptCallback, this, std::placeholders::_1));

  robosense::lidar::RSDriverParam driverParam;
  driverParam.lidar_type = lidar_type;
  driverParam.input_param.image_fps = device_config.image_input_fps;
  driverParam.input_param.image_width = device_config.image_width;
  driverParam.input_param.image_height = device_config.image_height;
  driverParam.input_param.imu_fps = device_config.imu_input_fps;
  driverParam.input_param.enable_image = true; // enable image output
  driverParam.input_param.image_format = device_config.image_format;
  driverParam.input_type = device_config.input_type;
  driverParam.input_param.device_uuid = device_config.device_uuid;
  driverParam.input_param.device_path = device_config.device_path;
  driverParam.decoder_param.dense_points =
      device_config.enable_use_dense_points;
  driverParam.decoder_param.use_lidar_clock =
      device_config.enable_use_lidar_clock;
  driverParam.decoder_param.ts_first_point =
      device_config.enable_use_first_point_ts;

  driverParam.decoder_param.enable_point_cloud =
      device_config.enable_pointcloud_send;
  driverParam.decoder_param.enable_imu = device_config.enable_imu_send;
  switch (driverParam.lidar_type) {
  case robosense::lidar::LidarType::RS_AC1: {
    driverParam.decoder_param.image_mode =
        (device_config.enable_ac1_image_send ? 0 : 3);
    break;
  }
  case robosense::lidar::LidarType::RS_AC2: {
    if (device_config.enable_ac2_left_image_send &&
        device_config.enable_ac2_right_image_send) {
      driverParam.decoder_param.image_mode = 0;
    } else if (device_config.enable_ac2_left_image_send) {
      driverParam.decoder_param.image_mode = 1;
    } else if (device_config.enable_ac2_right_image_send) {
      driverParam.decoder_param.image_mode = 2;
    } else {
      driverParam.decoder_param.image_mode = 3;
    }
    break;
  }
  }

  // AC2 需要配置
  driverParam.decoder_param.config_from_file =
      !device_config.angle_calib_basic_dir_path.empty();
  if (driverParam.decoder_param.config_from_file) {
    driverParam.decoder_param.angle_path =
        device_config.angle_calib_basic_dir_path;
  }
  driverParam.decoder_param.sync_timestamp_offset =
      device_config.timestamp_compensate_s;
#if defined(ENABLE_SUPPORT_RS_DRIVER_ALGORITHM)
  driverParam.decoder_param.algorithm_param = device_config.algorithm_param;
#endif // defined(ENABLE_SUPPORT_RS_DRIVER_ALGORITHM)
  driverParam.print();

  bool isSuccess = driver_ptr->init(driverParam);
  if (!isSuccess) {
    RS_SPDLOG_ERROR("DeviceManager: device uuid = " +
                    device_config.device_uuid + " Inital Failed !");
    return -3;
  }

  isSuccess = driver_ptr->start();
  if (!isSuccess) {
    RS_SPDLOG_ERROR("DeviceManager: device uuid = " +
                    device_config.device_uuid + " Start Failed !");
    return -4;
  }

  iterMap->second.is_pause = false;
  iterMap->second.driver_ptr = driver_ptr;

  return 0;
}

int DeviceManager::closeDevice(const std::string &device_uuid,
                               const bool isMetux) {
  if (isMetux) {
    RS_DEVICE_DRIVER_PTR driver_ptr;
    {
      std::lock_guard<std::mutex> lg(_devices_map_mtx);
      auto iterMap = _devices_map.find(device_uuid);
      if (iterMap == _devices_map.end()) {
        RS_SPDLOG_ERROR("DeviceManager: Device uuid = " + device_uuid +
                        " Already Detach !");
        return 0;
      } else if (iterMap->second.driver_ptr == nullptr) {
        RS_SPDLOG_INFO("DeviceManager: Device uuid = " + device_uuid +
                       " Not Open, So Not Need Close !");
        return 0;
      }
      driver_ptr = iterMap->second.driver_ptr;
    }

    if (driver_ptr != nullptr) {
      driver_ptr->stop();
      driver_ptr.reset();
    }

    {
      std::lock_guard<std::mutex> lg(_devices_map_mtx);
      auto iterMap = _devices_map.find(device_uuid);
      if (iterMap != _devices_map.end()) {
        iterMap->second.driver_ptr.reset();
      }
    }
  } else {
    auto iterMap = _devices_map.find(device_uuid);
    if (iterMap == _devices_map.end()) {
      RS_SPDLOG_ERROR("DeviceManager: Device uuid = " + device_uuid +
                      " Already Detach !");
      return 0;
    } else if (iterMap->second.driver_ptr == nullptr) {
      RS_SPDLOG_INFO("DeviceManager: Device uuid = " + device_uuid +
                     " Not Open, So Not Need Close !");
      return 0;
    }
    iterMap->second.driver_ptr->stop();
    iterMap->second.driver_ptr.reset();
  }
  return 0;
}

int DeviceManager::pauseDevice(const std::string &device_uuid,
                               const bool isPauseOp) {
  std::lock_guard<std::mutex> lg(_devices_map_mtx);
  auto iterMap = _devices_map.find(device_uuid);
  if (iterMap == _devices_map.end()) {
    RS_SPDLOG_ERROR("DeviceManager: Device uuid = " + device_uuid +
                    " Not Attach, So Not Need " +
                    (isPauseOp ? "Pause" : "Play") + " !");
    return -1;
  }
  iterMap->second.is_pause = isPauseOp;

  return 0;
}

int DeviceManager::closeDevices() {
  std::lock_guard<std::mutex> lg(_devices_map_mtx);
  while (!_devices_map.empty()) {
    const std::string &device_uuid = _devices_map.begin()->second.uuid;
    int ret = closeDevice(device_uuid, false);
    if (ret != 0) {
      RS_SPDLOG_ERROR("DeviceManager: device uuid = " + device_uuid +
                      " Close Failed: ret = " + std::to_string(ret));
      return -1;
    }
    _devices_map.erase(_devices_map.begin());
  }
  _devices_map.clear();

  return 0;
}

std::set<std::string> DeviceManager::getDevices() {
  std::lock_guard<std::mutex> lg(_devices_map_mtx);
  std::set<std::string> uuids;
  for (auto iterMap = _devices_map.begin(); iterMap != _devices_map.end();
       ++iterMap) {
    const DeviceInfoItem &deviceItem = iterMap->second;
    if (deviceItem.is_attach) {
      uuids.insert(iterMap->first);
    }
  }
  return uuids;
}

bool DeviceManager::getDeviceType(const std::string &uuid, bool &is_usb_mode) {
  std::lock_guard<std::mutex> lg(_devices_map_mtx);
  for (auto iterMap = _devices_map.begin(); iterMap != _devices_map.end();
       ++iterMap) {
    const DeviceInfoItem &deviceItem = iterMap->second;
    if (uuid == deviceItem.uuid) {
      is_usb_mode = deviceItem.is_usb_mode;
      return true;
    }
  }

  return false;
}

bool DeviceManager::getDeviceInfo(const std::string &uuid,
                                  robosense::lidar::DeviceInfo &device_info) {
  std::lock_guard<std::mutex> lg(_devices_map_mtx);
  for (auto iterMap = _devices_map.begin(); iterMap != _devices_map.end();
       ++iterMap) {
    const DeviceInfoItem &deviceItem = iterMap->second;
    if (uuid == deviceItem.uuid && deviceItem.driver_ptr != nullptr) {
      return deviceItem.driver_ptr->getDeviceInfo(device_info);
    }
  }
  return false;
}

void DeviceManager::regDeviceEventCallback(
    const RS_DEVICE_EVENT_CALLBACK &event_cb) {
  if (event_cb) {
    _event_cb = event_cb;
  }
}

void DeviceManager::regPointCloudCallback(
    const RS_DEVICE_POINTCLOUD_CALLBACK &pc_cb) {
  if (pc_cb) {
    _pc_cb = pc_cb;
  }
}

void DeviceManager::regImageDataCallback(
    const RS_DEVICE_IMAGE_CALLBACK &image_cb) {
  if (image_cb) {
    _image_cb = image_cb;
  }
}

void DeviceManager::regImuDataCallback(const RS_DEVICE_IMU_CALLBACK &imu_cb) {
  if (imu_cb) {
    _imu_cb = imu_cb;
  }
}

void DeviceManager::regExceptionCallback(
    const RS_DEVICE_EXCEPTION_CALLBACK &except_cb) {
  if (except_cb) {
    _except_cb = except_cb;
  }
}

std::shared_ptr<PointCloudT<RsPointXYZIRT>>
DeviceManager::localGetPointCloudCallback() {
  std::shared_ptr<PointCloudT<RsPointXYZIRT>> pointCloudMsgPtr;
  try {
    pointCloudMsgPtr.reset(new PointCloudT<RsPointXYZIRT>());
  } catch (...) {
    return nullptr;
  }
  return pointCloudMsgPtr;
}

std::shared_ptr<robosense::lidar::ImageData>
DeviceManager::localGetImageDataCallback() {
  std::shared_ptr<robosense::lidar::ImageData> imageDataMsgPtr;
  try {
    imageDataMsgPtr.reset(new robosense::lidar::MonoImageData());
  } catch (...) {
    return nullptr;
  }
  return imageDataMsgPtr;
}

std::shared_ptr<robosense::lidar::ImageData>
DeviceManager::localGetImageDataCallback2() {
  std::shared_ptr<robosense::lidar::ImageData> imageDataMsgPtr;
  try {
    imageDataMsgPtr.reset(new robosense::lidar::StereoImageData());
  } catch (...) {
    return nullptr;
  }
  return imageDataMsgPtr;
}

std::shared_ptr<robosense::lidar::ImuData>
DeviceManager::localGetImuDataCallback() {
  std::shared_ptr<robosense::lidar::ImuData> imuDataMsgPtr;
  try {
    imuDataMsgPtr.reset(new robosense::lidar::ImuData());
  } catch (...) {
    return nullptr;
  }
  return imuDataMsgPtr;
}

void DeviceManager::localRunPointCloudCallback(
    const std::shared_ptr<PointCloudT<RsPointXYZIRT>> &msgPtr,
    const uint32_t &uuid_id, const RSTimestampItem::Ptr &timestampPtr) {
  if (_is_stoping_.load()) {
    return;
  }

  const std::string &uuid = _uuid_id_mapper[uuid_id];
  {
    std::lock_guard<std::mutex> lg(_devices_map_mtx);
    if (!(_devices_map.find(uuid) != _devices_map.end() &&
          !_devices_map[uuid].is_pause)) {
      return;
    }
  }
  if (msgPtr) {
    RSMessageBufferItem<PointCloudT<RsPointXYZIRT>> item;
    item.uuid_id = uuid_id;
    item.msgPtr = msgPtr;
    item.timestampPtr = timestampPtr;
    std::lock_guard<std::mutex> lg(_pointCloudQueueMutex);
    _pointCloudQueue.push(std::move(item));
    _pointCloudQueueCond.notify_one(); // 通知等待的线程
  }
}

void DeviceManager::localRunImageCallback(
    const std::shared_ptr<robosense::lidar::ImageData> &msgPtr,
    const uint32_t &uuid_id, const RSTimestampItem::Ptr &timestampPtr) {
  if (_is_stoping_.load()) {
    return;
  }

  const std::string &uuid = _uuid_id_mapper[uuid_id];
  {
    std::lock_guard<std::mutex> lg(_devices_map_mtx);
    if (!(_devices_map.find(uuid) != _devices_map.end() &&
          !_devices_map[uuid].is_pause)) {
      return;
    }
  }
  if (msgPtr) {
    RSMessageBufferItem<robosense::lidar::ImageData> item;
    item.uuid_id = uuid_id;
    item.msgPtr = msgPtr;
    item.timestampPtr = timestampPtr;
    std::lock_guard<std::mutex> lg(_imageQueueMutex);
    _imageQueue.push(std::move(item));
    _imageQueueCond.notify_one(); // 通知等待的线程
  }
}

void DeviceManager::localRunImuCallback(
    const std::shared_ptr<robosense::lidar::ImuData> &msgPtr,
    const uint32_t &uuid_id, const RSTimestampItem::Ptr &timestampPtr) {
  if (_is_stoping_.load()) {
    return;
  }

  const std::string &uuid = _uuid_id_mapper[uuid_id];
  {
    std::lock_guard<std::mutex> lg(_devices_map_mtx);
    if (!(_devices_map.find(uuid) != _devices_map.end() &&
          !_devices_map[uuid].is_pause)) {
      return;
    }
  }
  if (msgPtr) {
    RSMessageBufferItem<robosense::lidar::ImuData> item;
    item.uuid_id = uuid_id;
    item.msgPtr = msgPtr;
    item.timestampPtr = timestampPtr;
    std::lock_guard<std::mutex> lg(_imuQueueMutex);
    _imuQueue.push(std::move(item));
    _imuQueueCond.notify_one(); // 通知等待的线程
  }
}

void DeviceManager::localRunExceptCallback(
    const robosense::lidar::Error &error) {
  if (_except_cb) {
    _except_cb(error);
  }
}

int DeviceManager::findDevices(
    std::map<std::string, DeviceInfoItem> &devices_map) {
  devices_map.clear();
  libusb_device **usb_dev_list;
  ssize_t num_usb_devices = libusb_get_device_list(_usb_ctx, &usb_dev_list);
  if (num_usb_devices < 0) {
    RS_SPDLOG_ERROR("get device list failed: num_usb_devices = " +
                    std::to_string(num_usb_devices));
    return -1;
  }

  for (int i = 0; i < num_usb_devices; i++) {
    libusb_device *device = usb_dev_list[i];
    libusb_device_descriptor desc;
    if (libusb_get_device_descriptor(device, &desc) != LIBUSB_SUCCESS) {
      RS_SPDLOG_ERROR("get device descriptor failed !");
      continue;
    }
    if (_enable_debug) {
      RS_SPDLOG_INFO("usb device index = " + std::to_string(i));
    }
    if (desc.idVendor == VENDOR_ID &&
        (desc.idProduct == PRODUCT_ID || desc.idProduct == PRODUCT_ID2)) {
      if (_enable_debug) {
        RS_SPDLOG_INFO("matched usb device index = " + std::to_string(i) +
                       ", desc.idVendor = " + std::to_string(desc.idVendor) +
                       ", desc.idProduct = " + std::to_string(desc.idProduct));
      }
      libusb_device_handle *handle;
      std::string uuid("0");
      std::string compare("0");
      if (libusb_open(device, &handle) == 0) {
        unsigned char serial[256];
        int ret = libusb_get_string_descriptor_ascii(handle, desc.iSerialNumber,
                                                     serial, sizeof(serial));
        if (ret > 0) {
          uuid = std::string(reinterpret_cast<char *>(serial));
        }
        libusb_close(handle);

        if (_enable_debug) {
          RS_SPDLOG_INFO("findDevice ===> " + uuid +
                         ", i_sn = " + std::to_string(desc.iSerialNumber));
        }

        // 获取uuid 成功
        if (uuid != compare && devices_map.find(uuid) == devices_map.end()) {
          DeviceInfoItem dev_i;
          // dev_i.dev = device;
          dev_i.is_attach = true;
          dev_i.uuid = uuid;
          dev_i.i_sn = desc.iSerialNumber;
          dev_i.driver_ptr = nullptr;
          dev_i.is_usb_mode = true;
          // if (desc.idVendor == VENDOR_ID && desc.idProduct == PRODUCT_ID) {
          //   dev_i.is_usb_mode = true;
          // } else {
          //   dev_i.is_usb_mode = false;
          // }
          // libusb_ref_device(device);
          if (desc.idProduct == PRODUCT_ID) {
            dev_i.lidar_type = robosense::lidar::LidarType::RS_AC1;
          } else if (desc.idProduct == PRODUCT_ID2) {
            dev_i.lidar_type = robosense::lidar::LidarType::RS_AC2;
          }
          devices_map.insert({uuid, dev_i});
        }
      }
    }
  }
  libusb_free_device_list(usb_dev_list, 1);

  return 0;
}

void DeviceManager::hotplugWorkThread() {
  std::vector<DeviceInfoItem> attachDevices;
  std::vector<DeviceInfoItem> detachDevices;

  while (_start) {
    // Step1: Search Device(s)
    std::map<std::string, DeviceInfoItem> deviceItems;
    int ret = findDevices(deviceItems);
    if (ret != 0) {
      RS_SPDLOG_ERROR("find Devices Failed: ret = " + std::to_string(ret));
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      continue;
    }

    if (_enable_debug) {
      RS_SPDLOG_INFO("deviceItems size = " +
                     std::to_string(deviceItems.size()));
    }

    attachDevices.clear();
    detachDevices.clear();
    std::map<std::string, DeviceInfoItem> erase_devices_map;
    {
      std::lock_guard<std::mutex> lg(_devices_map_mtx);
      // Step3: 获取不存在的设备
      for (auto iterMap = _devices_map.begin();
           iterMap != _devices_map.end();) {
        auto iterMap2 = deviceItems.find(iterMap->first);
        if (iterMap2 == deviceItems.end()) {
          // std::cout << "detach: uuid = " << iterMap->first
          //           << ", _devices_map size = " << _devices_map.size()
          //           << std::endl;
          detachDevices.push_back(iterMap->second);
          erase_devices_map.insert({iterMap->first, iterMap->second});
          iterMap = _devices_map.erase(iterMap);
        } else {
          ++iterMap;
        }
      }

      // Step2: 获取新的设备
      for (auto iterMap = deviceItems.begin(); iterMap != deviceItems.end();
           ++iterMap) {
        auto iterMap2 = _devices_map.find(iterMap->second.uuid);
        if (iterMap2 == _devices_map.end()) {
          _devices_map.insert({iterMap->first, iterMap->second});
          attachDevices.push_back(iterMap->second);
          // std::cout << "attach: uuid = " << iterMap->first
          //           << ", _devices_map size = " << _devices_map.size()
          //           << std::endl;
        }
      }
    }

    // 自动删除掉线的设备
    erase_devices_map.clear();

    // Step4: 通知设备事件
    if (_event_cb) {
      // Attach 事件
      for (const auto &item : attachDevices) {
        DeviceEvent_t event;
        event.event_type = robosense::device::DEVICE_EVENT_ATTACH;
        event.uuid_size = std::min(item.uuid.size(), sizeof(event.uuid));
        memcpy(event.uuid, item.uuid.c_str(), event.uuid_size);
        event.lidar_type = item.lidar_type;
        _event_cb(event);
      }
      // Detach 事件
      for (const auto &item : detachDevices) {
        DeviceEvent_t event;
        event.event_type = robosense::device::DEVICE_EVENT_DETACH;
        event.uuid_size = std::min(item.uuid.size(), sizeof(event.uuid));
        memcpy(event.uuid, item.uuid.c_str(), event.uuid_size);
        _event_cb(event);
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }
}

void DeviceManager::processPointCloudQueue() {
  while (!_stopProcessingThreads) {
    RSMessageBufferItem<PointCloudT<RsPointXYZIRT>> item;
    {
      std::unique_lock<std::mutex> lock(_pointCloudQueueMutex);
      _pointCloudQueueCond.wait(lock, [this] {
        return !_pointCloudQueue.empty() || _stopProcessingThreads;
      });
      if (_stopProcessingThreads)
        break;
      item = _pointCloudQueue.front();
      _pointCloudQueue.pop();
    }
    if (item.msgPtr && _pc_cb) {
      _pc_cb(item.msgPtr, _uuid_id_mapper[item.uuid_id], item.timestampPtr);
    }
  }
}

void DeviceManager::processImageQueue() {
  while (!_stopProcessingThreads) {
    RSMessageBufferItem<robosense::lidar::ImageData> item;
    {
      std::unique_lock<std::mutex> lock(_imageQueueMutex);
      _imageQueueCond.wait(lock, [this] {
        return !_imageQueue.empty() || _stopProcessingThreads;
      });
      if (_stopProcessingThreads)
        break;
      item = _imageQueue.front();
      _imageQueue.pop();
    }
    if (item.msgPtr && _image_cb) {
      _image_cb(item.msgPtr, _uuid_id_mapper[item.uuid_id], item.timestampPtr);
    }
  }
}

void DeviceManager::processImuQueue() {
  while (!_stopProcessingThreads) {
    RSMessageBufferItem<robosense::lidar::ImuData> item;
    {
      std::unique_lock<std::mutex> lock(_imuQueueMutex);
      _imuQueueCond.wait(lock, [this] {
        return !_imuQueue.empty() || _stopProcessingThreads;
      });
      if (_stopProcessingThreads)
        break;
      item = _imuQueue.front();
      _imuQueue.pop();
    }
    if (item.msgPtr && _imu_cb) {
      _imu_cb(item.msgPtr, _uuid_id_mapper[item.uuid_id], item.timestampPtr);
    }
  }
}

} // namespace device
} // namespace robosense
