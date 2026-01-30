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
#ifndef RSCALIBMANAGER_HPP
#define RSCALIBMANAGER_HPP

#include "hyper_vision/logmanager/logmanager.h"
#if defined(ROS_FOUND)
#include <robosense_msgs/RsACDeviceCalib.h>
#include <sensor_msgs/CameraInfo.h>
#elif defined(ROS2_FOUND)
#include <robosense_msgs/msg/rs_ac_device_calib.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#endif // ROS_ROS2_FOUND
#include <algorithm>
#include <opencv2/calib3d.hpp>
#include <yaml-cpp/yaml.h>

#if defined(ROS_FOUND)
using ROS_RSACDEVICECALIB = robosense_msgs::RsACDeviceCalib;
using ROS_CAMERAINFO = sensor_msgs::CameraInfo;
#elif defined(ROS2_FOUND)
using ROS_RSACDEVICECALIB = robosense_msgs::msg::RsACDeviceCalib;
using ROS_CAMERAINFO = sensor_msgs::msg::CameraInfo;
#endif // defined(ROS_ROS2_FOUND)
using ROS_RSACDEVICECALIB_PTR = std::shared_ptr<ROS_RSACDEVICECALIB>;
using ROS_CAMERAINFO_PTR = std::shared_ptr<ROS_CAMERAINFO>;

namespace robosense {
namespace calib {

class RSCalibManager {
public:
  using Ptr = std::shared_ptr<RSCalibManager>;
  using ConstPtr = std::shared_ptr<const RSCalibManager>;

public:
  RSCalibManager() = default;
  ~RSCalibManager() = default;

public:
  static bool isFileExist(const std::string &filePath) {
    int ret = access(filePath.c_str(), 0);

    if (ret != 0) {
      return false;
    }

    return true;
  }

  static bool checkImageCalibFileIsAC1(const std::string &filePath) {
    YAML::Node calibNode;
    try {
      calibNode = YAML::LoadFile(filePath);
    } catch (...) {
      RS_SPDLOG_ERROR("Load Yaml File Failed: filePath = " + filePath);
      return false;
    }

    if (!calibNode["DEVICE_ID"].IsDefined() ||
        !calibNode["Sensor"].IsDefined()) {
      RS_SPDLOG_ERROR(
          "Load Yaml File: " + filePath +
          " Successed, But Not Include \"DEVICE_ID\" or/and \"Sensor\" "
          "Node !");
      return false;
    }

    YAML::Node sensorNode = calibNode["Sensor"];
    if (sensorNode["Camera"].IsDefined() &&
        !sensorNode["Camera_R"].IsDefined() && sensorNode["IMU"].IsDefined()) {
      return true;
    }

    RS_SPDLOG_ERROR(
        "Load Yaml File: " + filePath +
        " Successed, But Not Include \"Camera\" or/and \"IMU\" Node | or "
        "Include \"Camera_R\"!");

    return false;
  }

  static bool checkImageCalibFileIsAC2(const std::string &filePath) {
    YAML::Node calibNode;
    try {
      calibNode = YAML::LoadFile(filePath);
    } catch (...) {
      RS_SPDLOG_ERROR("Load Yaml File Failed: filePath = " + filePath);
      return false;
    }

    if (!calibNode["DEVICE_ID"].IsDefined() ||
        !calibNode["Sensor"].IsDefined()) {
      RS_SPDLOG_ERROR(
          "Load Yaml File: " + filePath +
          " Successed, But Not Include \"DEVICE_ID\" or/and \"Sensor\" "
          "Node !");
      return false;
    }

    YAML::Node sensorNode = calibNode["Sensor"];
    if (sensorNode["Camera"].IsDefined() &&
        sensorNode["Camera_R"].IsDefined() && sensorNode["IMU"].IsDefined()) {
      return true;
    }

    RS_SPDLOG_ERROR(
        "Load Yaml File: " + filePath +
        " Successed, But Not Include \"Camera\" Or/And \"Camera_R\" "
        "or/and \"IMU\" Node !");

    return false;
  }

  static std::string toLowerCase(const std::string &input) {
    std::string result = input;
    std::transform(result.begin(), result.end(), result.begin(), ::tolower);
    return result;
  }

  static int parserAC1DeviceCalibInfo(const std::string &filePath,
                                      ROS_RSACDEVICECALIB &device_calib_msg) {
    YAML::Node calibNode;
    try {
      calibNode = YAML::LoadFile(filePath);
    } catch (...) {
      RS_SPDLOG_ERROR("Load Yaml File Failed: filePath = " + filePath);
      return -1;
    }

    if (!calibNode["DEVICE_ID"].IsDefined() ||
        !calibNode["Sensor"].IsDefined()) {
      RS_SPDLOG_ERROR(
          "Load Yaml File: " + filePath +
          " Successed, But Not Include \"DEVICE_ID\" or/and \"Sensor\" "
          "Node !");
      return -2;
    }

    YAML::Node sensorNode = calibNode["Sensor"];
    if (!sensorNode["Camera"].IsDefined() || !sensorNode["IMU"].IsDefined()) {
      RS_SPDLOG_ERROR(
          "Load Yaml File: " + filePath +
          " Successed, But Not Include \"Camera\" Or/And \"IMU\" Node !");
      return -3;
    }

    // DEVICE_ID
    device_calib_msg.device_id = calibNode["DEVICE_ID"].as<std::string>();

    {
      // CAMERA
      const YAML::Node &cameraNode = sensorNode["Camera"];
      const std::vector<double> &int_matrix =
          cameraNode["intrinsic"]["int_matrix"].as<std::vector<double>>();
      const std::vector<double> &dist_coeff =
          cameraNode["intrinsic"]["dist_coeff"].as<std::vector<double>>();
      const YAML::Node &extrinsic_trans =
          cameraNode["extrinsic"]["translation"];
      const YAML::Node &extrinsic_quat = cameraNode["extrinsic"]["quaternion"];
      const std::vector<int32_t> &image_size =
          cameraNode["intrinsic"]["image_size"].as<std::vector<int32_t>>();

      double fx = int_matrix[0]; // fx
      double fy = int_matrix[4]; // fy
      double cx = int_matrix[2]; // cx
      double cy = int_matrix[5]; // cy

      device_calib_msg.camcx = cx;
      device_calib_msg.camcy = cy;
      device_calib_msg.camdistcoeff1 = dist_coeff[0];
      device_calib_msg.camdistcoeff2 = dist_coeff[1];
      device_calib_msg.camdistcoeff3 = dist_coeff[2];
      device_calib_msg.camdistcoeff4 = dist_coeff[3];
      device_calib_msg.camdistcoeff5 = dist_coeff[4];
      device_calib_msg.camdistcoeff6 = dist_coeff[5];
      device_calib_msg.camdistcoeff7 = dist_coeff[6];
      device_calib_msg.camdistcoeff8 = dist_coeff[7];
      device_calib_msg.camfx = fx;
      device_calib_msg.camfy = fy;

      // CAMERA -> LIDAR
      device_calib_msg.camtolidarqw = extrinsic_quat["w"].as<double>();
      device_calib_msg.camtolidarqx = extrinsic_quat["x"].as<double>();
      device_calib_msg.camtolidarqy = extrinsic_quat["y"].as<double>();
      device_calib_msg.camtolidarqz = extrinsic_quat["z"].as<double>();
      device_calib_msg.camtolidartx = extrinsic_trans["x"].as<double>();
      device_calib_msg.camtolidarty = extrinsic_trans["y"].as<double>();
      device_calib_msg.camtolidartz = extrinsic_trans["z"].as<double>();

      // 左相机更新即可
      device_calib_msg.imagewidth = image_size[0];
      device_calib_msg.imageheight = image_size[1];
    }

    // 无须填写
    // CAMERA_R
    // CAMERA_R -> CAMERA

    {
      // IMU -> LIDAR
      const YAML::Node &imuNode = sensorNode["IMU"];
      const YAML::Node &extrinsic_trans = imuNode["extrinsic"]["translation"];
      const YAML::Node &extrinsic_quat = imuNode["extrinsic"]["quaternion"];

      device_calib_msg.imutolidarqw = extrinsic_quat["w"].as<double>();
      device_calib_msg.imutolidarqx = extrinsic_quat["x"].as<double>();
      device_calib_msg.imutolidarqy = extrinsic_quat["y"].as<double>();
      device_calib_msg.imutolidarqz = extrinsic_quat["z"].as<double>();
      device_calib_msg.imutolidartx = extrinsic_trans["x"].as<double>();
      device_calib_msg.imutolidarty = extrinsic_trans["y"].as<double>();
      device_calib_msg.imutolidartz = extrinsic_trans["z"].as<double>();
    }

    return 0;
  }

  static int parserAC2DeviceCalibInfo(const std::string &filePath,
                                      ROS_RSACDEVICECALIB &device_calib_msg) {
    YAML::Node calibNode;
    try {
      calibNode = YAML::LoadFile(filePath);
    } catch (...) {
      RS_SPDLOG_ERROR("Load Yaml File Failed: filePath = " + filePath);
      return -1;
    }

    if (!calibNode["DEVICE_ID"].IsDefined() ||
        !calibNode["Sensor"].IsDefined()) {
      RS_SPDLOG_ERROR(
          "Load Yaml File: " + filePath +
          " Successed, But Not Include \"DEVICE_ID\" or/and \"Sensor\" "
          "Node !");
      return -2;
    }

    YAML::Node sensorNode = calibNode["Sensor"];
    if (!sensorNode["Camera"].IsDefined() ||
        !sensorNode["Camera_R"].IsDefined() || !sensorNode["IMU"].IsDefined()) {
      RS_SPDLOG_ERROR(
          "Load Yaml File: " + filePath +
          " Successed, But Not Include \"Camera\" Or/And \"Camera_R\" "
          "Or/And \"IMU\" Node !");
      return -3;
    }

    // DEVICE_ID
    device_calib_msg.device_id = calibNode["DEVICE_ID"].as<std::string>();

    {
      // CAMERA
      const YAML::Node &cameraNode = sensorNode["Camera"];
      const std::vector<double> &int_matrix =
          cameraNode["intrinsic"]["int_matrix"].as<std::vector<double>>();
      const std::vector<double> &dist_coeff =
          cameraNode["intrinsic"]["dist_coeff"].as<std::vector<double>>();
      const YAML::Node &extrinsic_trans =
          cameraNode["extrinsic"]["translation"];
      const YAML::Node &extrinsic_quat = cameraNode["extrinsic"]["quaternion"];
      const std::vector<int32_t> &image_size =
          cameraNode["intrinsic"]["image_size"].as<std::vector<int32_t>>();

      double fx = int_matrix[0]; // fx
      double fy = int_matrix[4]; // fy
      double cx = int_matrix[2]; // cx
      double cy = int_matrix[5]; // cy

      device_calib_msg.camcx = cx;
      device_calib_msg.camcy = cy;
      device_calib_msg.camdistcoeff1 = dist_coeff[0];
      device_calib_msg.camdistcoeff2 = dist_coeff[1];
      device_calib_msg.camdistcoeff3 = dist_coeff[2];
      device_calib_msg.camdistcoeff4 = dist_coeff[3];
      device_calib_msg.camdistcoeff5 = dist_coeff[4];
      device_calib_msg.camdistcoeff6 = dist_coeff[5];
      device_calib_msg.camdistcoeff7 = dist_coeff[6];
      device_calib_msg.camdistcoeff8 = dist_coeff[7];
      device_calib_msg.camfx = fx;
      device_calib_msg.camfy = fy;

      // CAMERA -> LIDAR
      device_calib_msg.camtolidarqw = extrinsic_quat["w"].as<double>();
      device_calib_msg.camtolidarqx = extrinsic_quat["x"].as<double>();
      device_calib_msg.camtolidarqy = extrinsic_quat["y"].as<double>();
      device_calib_msg.camtolidarqz = extrinsic_quat["z"].as<double>();
      device_calib_msg.camtolidartx = extrinsic_trans["x"].as<double>();
      device_calib_msg.camtolidarty = extrinsic_trans["y"].as<double>();
      device_calib_msg.camtolidartz = extrinsic_trans["z"].as<double>();

      // 左相机更新即可
      device_calib_msg.imagewidth = image_size[0];
      device_calib_msg.imageheight = image_size[1];
    }

    {
      const YAML::Node &cameraNode = sensorNode["Camera_R"];
      const std::vector<double> &int_matrix =
          cameraNode["intrinsic"]["int_matrix"].as<std::vector<double>>();
      const std::vector<double> &dist_coeff =
          cameraNode["intrinsic"]["dist_coeff"].as<std::vector<double>>();
      const YAML::Node &extrinsic_trans =
          cameraNode["extrinsic"]["translation"];
      const YAML::Node &extrinsic_quat = cameraNode["extrinsic"]["quaternion"];

      double fx = int_matrix[0]; // fx
      double fy = int_matrix[4]; // fy
      double cx = int_matrix[2]; // cx
      double cy = int_matrix[5]; // cy

      // CAMERA_R
      device_calib_msg.camrcx = cx;
      device_calib_msg.camrcy = cy;
      device_calib_msg.camrdistcoeff1 = dist_coeff[0];
      device_calib_msg.camrdistcoeff2 = dist_coeff[1];
      device_calib_msg.camrdistcoeff3 = dist_coeff[2];
      device_calib_msg.camrdistcoeff4 = dist_coeff[3];
      device_calib_msg.camrdistcoeff5 = dist_coeff[4];
      device_calib_msg.camrdistcoeff6 = dist_coeff[5];
      device_calib_msg.camrdistcoeff7 = dist_coeff[6];
      device_calib_msg.camrdistcoeff8 = dist_coeff[7];
      device_calib_msg.camrfx = fx;
      device_calib_msg.camrfy = fy;

      // CAMERA_R -> CAMERA
      device_calib_msg.camrtocamqw = extrinsic_quat["w"].as<double>();
      device_calib_msg.camrtocamqx = extrinsic_quat["x"].as<double>();
      device_calib_msg.camrtocamqy = extrinsic_quat["y"].as<double>();
      device_calib_msg.camrtocamqz = extrinsic_quat["z"].as<double>();
      device_calib_msg.camrtocamtx = extrinsic_trans["x"].as<double>();
      device_calib_msg.camrtocamty = extrinsic_trans["y"].as<double>();
      device_calib_msg.camrtocamtz = extrinsic_trans["z"].as<double>();
    }

    {
      // IMU -> LIDAR
      const YAML::Node &imuNode = sensorNode["IMU"];
      const YAML::Node &extrinsic_trans = imuNode["extrinsic"]["translation"];
      const YAML::Node &extrinsic_quat = imuNode["extrinsic"]["quaternion"];

      device_calib_msg.imutolidarqw = extrinsic_quat["w"].as<double>();
      device_calib_msg.imutolidarqx = extrinsic_quat["x"].as<double>();
      device_calib_msg.imutolidarqy = extrinsic_quat["y"].as<double>();
      device_calib_msg.imutolidarqz = extrinsic_quat["z"].as<double>();
      device_calib_msg.imutolidartx = extrinsic_trans["x"].as<double>();
      device_calib_msg.imutolidarty = extrinsic_trans["y"].as<double>();
      device_calib_msg.imutolidartz = extrinsic_trans["z"].as<double>();
    }

    return 0;
  }

  static int
  parserDeviceCalibInfo(const std::string &uuid,
                        const robosense::lidar::DeviceInfo &device_info,
                        ROS_RSACDEVICECALIB &device_calib_msg) {
    // 分别对应AC1/AC2的标定参数
    auto calib_params = device_info.calib_params;
    const auto &calib_params_str = device_info.calib_params_str;
    if (calib_params.empty() && calib_params_str.empty()) {
      RS_SPDLOG_ERROR("Parser Device Calib Info Failed: calib_params and "
                      "calib_params_str is empty !");
      return -1;
    }

    if (!calib_params_str.empty()) {
      // AC1 Case1:
      YAML::Node calib_node;
      try {
        calib_node = YAML::Load(calib_params_str);
      } catch (...) {
        RS_SPDLOG_ERROR(
            "Calib Params Convert To Yaml Node Failed: calib_params_str = " +
            calib_params_str);
        return -2;
      }
      // std::cout << "calib_node = " << calib_node << std::endl;
      for (auto iter = calib_node.begin(); iter != calib_node.end(); ++iter) {
        const std::string &key = iter->first.as<std::string>();
        const std::string &key2 = toLowerCase(key);
        const double value = calib_node[key].as<double>();
        calib_params.insert({key2, value});
      }
      // 兼容旧版数据
      if (device_info.device_id.empty()) {
        device_calib_msg.device_id = toLowerCase(uuid);
      }
      if (calib_params.find("imagewidth") == calib_params.end()) {
        calib_params.insert({"imagewidth", 1920}); // AC1 固定分辨率
      }
      if (calib_params.find("imageheight") == calib_params.end()) {
        calib_params.insert({"imageheight", 1080}); // AC1 固定分辨率
      }
    }
    if (!device_info.device_id.empty()) {
      device_calib_msg.device_id = device_info.device_id;
    }

    // 标定参数
    for (auto iterMap = calib_params.begin(); iterMap != calib_params.end();
         ++iterMap) {
      const std::string &key = toLowerCase(iterMap->first);
      const double value = iterMap->second;
      if (key == "imagewidth") {
        device_calib_msg.imagewidth = value;
      } else if (key == "imageheight") {
        device_calib_msg.imageheight = value;
      } else if (key == "camcx") {
        device_calib_msg.camcx = value;
      } else if (key == "camcy") {
        device_calib_msg.camcy = value;
      } else if (key == "camdistcoeff1") {
        device_calib_msg.camdistcoeff1 = value;
      } else if (key == "camdistcoeff2") {
        device_calib_msg.camdistcoeff2 = value;
      } else if (key == "camdistcoeff3") {
        device_calib_msg.camdistcoeff3 = value;
      } else if (key == "camdistcoeff4") {
        device_calib_msg.camdistcoeff4 = value;
      } else if (key == "camdistcoeff5") {
        device_calib_msg.camdistcoeff5 = value;
      } else if (key == "camdistcoeff6") {
        device_calib_msg.camdistcoeff6 = value;
      } else if (key == "camdistcoeff7") {
        device_calib_msg.camdistcoeff7 = value;
      } else if (key == "camdistcoeff8") {
        device_calib_msg.camdistcoeff8 = value;
      } else if (key == "camfx") {
        device_calib_msg.camfx = value;
      } else if (key == "camfy") {
        device_calib_msg.camfy = value;
      } else if (key == "camrcx") {
        device_calib_msg.camrcx = value;
      } else if (key == "camrcy") {
        device_calib_msg.camrcy = value;
      } else if (key == "camrdistcoeff1") {
        device_calib_msg.camrdistcoeff1 = value;
      } else if (key == "camrdistcoeff2") {
        device_calib_msg.camrdistcoeff2 = value;
      } else if (key == "camrdistcoeff3") {
        device_calib_msg.camrdistcoeff3 = value;
      } else if (key == "camrdistcoeff4") {
        device_calib_msg.camrdistcoeff4 = value;
      } else if (key == "camrdistcoeff5") {
        device_calib_msg.camrdistcoeff5 = value;
      } else if (key == "camrdistcoeff6") {
        device_calib_msg.camrdistcoeff6 = value;
      } else if (key == "camrdistcoeff7") {
        device_calib_msg.camrdistcoeff7 = value;
      } else if (key == "camrdistcoeff8") {
        device_calib_msg.camrdistcoeff8 = value;
      } else if (key == "camrfx") {
        device_calib_msg.camrfx = value;
      } else if (key == "camrfy") {
        device_calib_msg.camrfy = value;
      } else if (key == "camrtocamqw") {
        device_calib_msg.camrtocamqw = value;
      } else if (key == "camrtocamqx") {
        device_calib_msg.camrtocamqx = value;
      } else if (key == "camrtocamqy") {
        device_calib_msg.camrtocamqy = value;
      } else if (key == "camrtocamqz") {
        device_calib_msg.camrtocamqz = value;
      } else if (key == "camrtocamtx") {
        device_calib_msg.camrtocamtx = value;
      } else if (key == "camrtocamty") {
        device_calib_msg.camrtocamty = value;
      } else if (key == "camrtocamtz") {
        device_calib_msg.camrtocamtz = value;
      } else if (key == "camtolidarqw") {
        device_calib_msg.camtolidarqw = value;
      } else if (key == "camtolidarqx") {
        device_calib_msg.camtolidarqx = value;
      } else if (key == "camtolidarqy") {
        device_calib_msg.camtolidarqy = value;
      } else if (key == "camtolidarqz") {
        device_calib_msg.camtolidarqz = value;
      } else if (key == "camtolidartx") {
        device_calib_msg.camtolidartx = value;
      } else if (key == "camtolidarty") {
        device_calib_msg.camtolidarty = value;
      } else if (key == "camtolidartz") {
        device_calib_msg.camtolidartz = value;
      } else if (key == "imutolidarqw") {
        device_calib_msg.imutolidarqw = value;
      } else if (key == "imutolidarqx") {
        device_calib_msg.imutolidarqx = value;
      } else if (key == "imutolidarqy") {
        device_calib_msg.imutolidarqy = value;
      } else if (key == "imutolidarqz") {
        device_calib_msg.imutolidarqz = value;
      } else if (key == "imutolidartx") {
        device_calib_msg.imutolidartx = value;
      } else if (key == "imutolidarty") {
        device_calib_msg.imutolidarty = value;
      } else if (key == "imutolidartz") {
        device_calib_msg.imutolidartz = value;
      } else {
        RS_SPDLOG_WARN("Not Support key = " + key);
        continue;
      }
      RS_SPDLOG_INFO("key = " + key +
                     ", value = " + std::to_string(iterMap->second));
    }
    RS_SPDLOG_INFO(std::string("device_id    = ") + device_calib_msg.device_id);
    RS_SPDLOG_INFO(std::string("image_width  = ") +
                   std::to_string(device_calib_msg.imagewidth));
    RS_SPDLOG_INFO(std::string("image_height = ") +
                   std::to_string(device_calib_msg.imageheight));

    return 0;
  }

  static ROS_CAMERAINFO_PTR parserAC1CameraInfo(const std::string &filePath) {
    ROS_CAMERAINFO_PTR camera_info_ptr;
    YAML::Node calibNode;
    try {
      calibNode = YAML::LoadFile(filePath);
    } catch (...) {
      RS_SPDLOG_ERROR("Load Yaml File Failed: filePath = " + filePath);
      return nullptr;
    }

    if (!calibNode["DEVICE_ID"].IsDefined() ||
        !calibNode["Sensor"].IsDefined()) {
      RS_SPDLOG_ERROR(
          "Load Yaml File: " + filePath +
          " Successed, But Not Include \"DEVICE_ID\" or/and \"Sensor\" "
          "Node !");
      return nullptr;
    }

    YAML::Node sensorNode = calibNode["Sensor"];
    if (!sensorNode["Camera"].IsDefined()) {
      RS_SPDLOG_ERROR("Load Yaml File: " + filePath +
                      " Successed, But Not Include \"Camera\" Node !");
      return nullptr;
    }

    camera_info_ptr = parserCameraInfo(sensorNode["Camera"]);

    return camera_info_ptr;
  }

  static std::pair<ROS_CAMERAINFO_PTR, ROS_CAMERAINFO_PTR>
  parserAC2CameraInfo(const std::string &filePath) {
    std::pair<ROS_CAMERAINFO_PTR, ROS_CAMERAINFO_PTR>
        left_right_camera_info_ptr;

    YAML::Node calibNode;
    try {
      calibNode = YAML::LoadFile(filePath);
    } catch (...) {
      RS_SPDLOG_ERROR("Load Yaml File Failed: filePath = " + filePath);
      return {nullptr, nullptr};
    }

    if (!calibNode["DEVICE_ID"].IsDefined() ||
        !calibNode["Sensor"].IsDefined()) {
      RS_SPDLOG_ERROR(
          "Load Yaml File: " + filePath +
          " Successed, But Not Include \"DEVICE_ID\" or/and \"Sensor\" "
          "Node !");
      return {nullptr, nullptr};
    }

    YAML::Node sensorNode = calibNode["Sensor"];
    if (!(sensorNode["Camera"].IsDefined() &&
          sensorNode["Camera_R"].IsDefined())) {
      RS_SPDLOG_ERROR("Load Yaml File: " + filePath +
                      " Successed, But Not Include \"Camera\" Or/And "
                      "\"Camera_R\" Node !");
      return {nullptr, nullptr};
    }

    left_right_camera_info_ptr.first = parserCameraInfo(sensorNode["Camera"]);
    left_right_camera_info_ptr.second =
        parserCameraInfo(sensorNode["Camera_R"]);

    return left_right_camera_info_ptr;
  }

  static ROS_CAMERAINFO_PTR parserCameraInfo(const YAML::Node &calibNode) {
    ROS_CAMERAINFO_PTR camera_info_ptr(new ROS_CAMERAINFO());
    auto &cam_info = *camera_info_ptr;
    cam_info.distortion_model = "rational_polynomial";
    cam_info.binning_x = 0;
    cam_info.binning_y = 0;

#if defined(ROS_FOUND)
    auto &K = camera_info_ptr->K;
    auto &D = camera_info_ptr->D;
    auto &P = camera_info_ptr->P;
#elif defined(ROS2_FOUND)
    auto &K = camera_info_ptr->k;
    auto &D = camera_info_ptr->d;
    auto &P = camera_info_ptr->p;
#endif // defined(ROS_ROS2_FOUND)

    if (calibNode["intrinsic"]) {
      YAML::Node intrinsic_node = calibNode["intrinsic"];
      if (intrinsic_node["int_matrix"]) {
        std::vector<double> int_matrix =
            intrinsic_node["int_matrix"].as<std::vector<double>>();
        double fx = int_matrix[0]; // fx
        double fy = int_matrix[4]; // fy
        double cx = int_matrix[2]; // cx
        double cy = int_matrix[5]; // cy
        K = {int_matrix[0], int_matrix[1], int_matrix[2],
             int_matrix[3], int_matrix[4], int_matrix[5],
             int_matrix[6], int_matrix[7], int_matrix[8]};

        P = {int_matrix[0], int_matrix[1], int_matrix[2], 0.0,
             int_matrix[3], int_matrix[4], int_matrix[5], 0.0,
             int_matrix[6], int_matrix[7], int_matrix[8], 1.0};
      }
      if (intrinsic_node["dist_coeff"]) {
        std::vector<double> dist_coeff =
            intrinsic_node["dist_coeff"].as<std::vector<double>>();
        for (int i = 0; i < 8; i++) {
          D.push_back(dist_coeff[i]);
        }
      }
      if (intrinsic_node["image_size"]) {
        std::vector<int> image_size =
            intrinsic_node["image_size"].as<std::vector<int>>();
        cam_info.width = image_size[0];
        cam_info.height = image_size[1];
        cam_info.roi.x_offset = 0;
        cam_info.roi.y_offset = 0;
        cam_info.roi.width = image_size[0];
        cam_info.roi.height = image_size[1];
      }
    }

    return camera_info_ptr;
  }

  static int parserImageRectifyMap(const ROS_CAMERAINFO_PTR &camera_info_ptr,
                                   cv::Mat &map1, cv::Mat &map2) {
    if (camera_info_ptr) {
#if defined(ROS_FOUND)
      const auto &K = camera_info_ptr->K;
      const auto &D = camera_info_ptr->D;
#elif defined(ROS2_FOUND)
      const auto &K = camera_info_ptr->k;
      const auto &D = camera_info_ptr->d;
#endif // defined(ROS_ROS2_FOUND)
      cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << K[0], K[1], K[2], K[3],
                              K[4], K[5], K[6], K[7], K[8]);
      cv::Mat distCoeffsMat = (cv::Mat_<double>(1, 8) << D[0], D[1], D[2], D[3],
                               D[4], D[5], D[6], D[7]);

      cv::initUndistortRectifyMap(
          cameraMatrix, distCoeffsMat, cv::Mat(), cameraMatrix,
          cv::Size(camera_info_ptr->width, camera_info_ptr->height), CV_16SC2,
          map1, map2);
    } else {
      RS_SPDLOG_ERROR("Parser OpenCV Rectify Mapper Failed !");
      return -1;
    }

    return 0;
  }

  static ROS_CAMERAINFO_PTR
  parserAC1CameraInfo(const ROS_RSACDEVICECALIB &deviceInfo) {
    ROS_CAMERAINFO_PTR camera_info_ptr(new ROS_CAMERAINFO());
    auto &cam_info = *camera_info_ptr;
    cam_info.distortion_model = "rational_polynomial";
    cam_info.binning_x = 0;
    cam_info.binning_y = 0;
#if defined(ROS_FOUND)
    auto &K = camera_info_ptr->K;
    auto &D = camera_info_ptr->D;
    auto &P = camera_info_ptr->P;
#elif defined(ROS2_FOUND)
    auto &K = camera_info_ptr->k;
    auto &D = camera_info_ptr->d;
    auto &P = camera_info_ptr->p;
#endif // defined(ROS_ROS2_FOUND)
    K = {deviceInfo.camfx,
         0,
         deviceInfo.camcx,
         0,
         deviceInfo.camfy,
         deviceInfo.camcy,
         0,
         0,
         1};
    P = {deviceInfo.camfx,
         0,
         deviceInfo.camcx,
         0,
         0,
         deviceInfo.camfy,
         deviceInfo.camcy,
         0,
         0,
         0,
         0,
         1};
    D = {deviceInfo.camdistcoeff1, deviceInfo.camdistcoeff2,
         deviceInfo.camdistcoeff3, deviceInfo.camdistcoeff4,
         deviceInfo.camdistcoeff5, deviceInfo.camdistcoeff6,
         deviceInfo.camdistcoeff7, deviceInfo.camdistcoeff8};

    cam_info.width = deviceInfo.imagewidth;
    cam_info.height = deviceInfo.imageheight;
    cam_info.roi.x_offset = 0;
    cam_info.roi.y_offset = 0;
    cam_info.roi.width = cam_info.width;
    cam_info.roi.height = cam_info.height;

    return camera_info_ptr;
  }

  static std::pair<ROS_CAMERAINFO_PTR, ROS_CAMERAINFO_PTR>
  parserAC2CameraInfo(const ROS_RSACDEVICECALIB &deviceInfo) {
    std::pair<ROS_CAMERAINFO_PTR, ROS_CAMERAINFO_PTR>
        left_right_camera_info_ptr;
    // Left Camera
    {
      ROS_CAMERAINFO_PTR camera_info_ptr(new ROS_CAMERAINFO());
      auto &cam_info = *camera_info_ptr;
      cam_info.distortion_model = "rational_polynomial";
      cam_info.binning_x = 0;
      cam_info.binning_y = 0;

#if defined(ROS_FOUND)
      auto &K = camera_info_ptr->K;
      auto &D = camera_info_ptr->D;
      auto &P = camera_info_ptr->P;
#elif defined(ROS2_FOUND)
      auto &K = camera_info_ptr->k;
      auto &D = camera_info_ptr->d;
      auto &P = camera_info_ptr->p;
#endif // defined(ROS_ROS2_FOUND)

      K = {deviceInfo.camfx,
           0,
           deviceInfo.camcx,
           0,
           deviceInfo.camfy,
           deviceInfo.camcy,
           0,
           0,
           1};
      P = {deviceInfo.camfx,
           0,
           deviceInfo.camcx,
           0,
           0,
           deviceInfo.camfy,
           deviceInfo.camcy,
           0,
           0,
           0,
           0,
           1};
      D = {deviceInfo.camdistcoeff1, deviceInfo.camdistcoeff2,
           deviceInfo.camdistcoeff3, deviceInfo.camdistcoeff4,
           deviceInfo.camdistcoeff5, deviceInfo.camdistcoeff6,
           deviceInfo.camdistcoeff7, deviceInfo.camdistcoeff8};

      cam_info.width = deviceInfo.imagewidth;
      cam_info.height = deviceInfo.imageheight;
      cam_info.roi.x_offset = 0;
      cam_info.roi.y_offset = 0;
      cam_info.roi.width = cam_info.width;
      cam_info.roi.height = cam_info.height;

      left_right_camera_info_ptr.first = camera_info_ptr;
    }

    // Right Camera
    {
      ROS_CAMERAINFO_PTR camera_info_ptr(new ROS_CAMERAINFO());
      auto &cam_info = *camera_info_ptr;
      cam_info.distortion_model = "rational_polynomial";
      cam_info.binning_x = 0;
      cam_info.binning_y = 0;

#if defined(ROS_FOUND)
      auto &K = camera_info_ptr->K;
      auto &D = camera_info_ptr->D;
      auto &P = camera_info_ptr->P;
#elif defined(ROS2_FOUND)
      auto &K = camera_info_ptr->k;
      auto &D = camera_info_ptr->d;
      auto &P = camera_info_ptr->p;
#endif // defined(ROS_ROS2_FOUND)

      K = {deviceInfo.camrfx,
           0,
           deviceInfo.camrcx,
           0,
           deviceInfo.camrfy,
           deviceInfo.camrcy,
           0,
           0,
           1};
      P = {deviceInfo.camrfx,
           0,
           deviceInfo.camrcx,
           0,
           0,
           deviceInfo.camrfy,
           deviceInfo.camrcy,
           0,
           0,
           0,
           0,
           1};
      D = {deviceInfo.camrdistcoeff1, deviceInfo.camrdistcoeff2,
           deviceInfo.camrdistcoeff3, deviceInfo.camrdistcoeff4,
           deviceInfo.camrdistcoeff5, deviceInfo.camrdistcoeff6,
           deviceInfo.camrdistcoeff7, deviceInfo.camrdistcoeff8};

      cam_info.width = deviceInfo.imagewidth;
      cam_info.height = deviceInfo.imageheight;
      cam_info.roi.x_offset = 0;
      cam_info.roi.y_offset = 0;
      cam_info.roi.width = cam_info.width;
      cam_info.roi.height = cam_info.height;

      left_right_camera_info_ptr.second = camera_info_ptr;
    }

    return left_right_camera_info_ptr;
  }
};

} // namespace calib
} // namespace robosense

#endif // RSCALIBMANAGER_HPP