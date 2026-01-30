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

/**
 * @file metaS_node.cpp
 * @brief ROS/ROS2 Node for publishing RGB, depth, and IMU data from a
 * SuperSense device.
 *
 * This node retrieves data from a metaS device and publishes it to ROS/ROS2
 * topics:
 * - RGB images on the "/rs_camera/rgb" topic
 * - Depth point clouds on the "/rs_lidar/points" topic
 * - IMU data on the "/rs_imu" topic
 */

#include <chrono>
#include <condition_variable>
#include <cstring>
#include <iostream>
#include <memory>
#include <queue>
#include <thread>
#include <vector>

#ifdef RK3588
#include "rga/RgaUtils.h"
#include "rga/im2d.hpp"
#else
#include "hyper_vision/codec/colorcodec.h"
#include "hyper_vision/codec/jpegcoder.h"
#endif
#include "hyper_vision/devicemanager/devicemanager.h"
#include "rosmanager.hpp"

enum class RS_IMAGE_SOURCE_TYPE : int {
  RS_IMAGE_SOURCE_AC1 = 0,
  RS_IMAGE_SOURCE_AC2_LEFT,
  RS_IMAGE_SOURCE_AC2_RIGHT,
};

class RSImageCropConfig {
public:
  using Ptr = std::shared_ptr<RSImageCropConfig>;
  using ConstPtr = std::shared_ptr<const RSImageCropConfig>;

public:
  RSImageCropConfig() { reset(); }
  ~RSImageCropConfig() = default;

public:
  void reset() {
    image_crop_enable = false;
    image_crop_top = 0;
    image_crop_bottom = 0;
    image_crop_left = 0;
    image_crop_right = 0;
  }

  void updateCrop(const int32_t crop_top, const int32_t crop_bottom,
                  const int32_t crop_left, const int32_t crop_right) {
    reset();
    if (crop_top || crop_bottom || crop_left || crop_right) {
      image_crop_enable = true;
    }

    image_crop_top = crop_top;
    image_crop_bottom = crop_bottom;
    image_crop_left = crop_left;
    image_crop_right = crop_right;
  }

  bool checkIsCropImage() const { return image_crop_enable; }

  std::string toString() const {
    std::string str = "[" + std::to_string(image_crop_top) + "," +
                      std::to_string(image_crop_bottom) + "," +
                      std::to_string(image_crop_left) + "," +
                      std::to_string(image_crop_right) + "]";

    return str;
  }

  int32_t getCropWidth() { return image_crop_left + image_crop_right; }

  int32_t getCropHeight() { return image_crop_top + image_crop_bottom; }

  int32_t getCropTop() const { return image_crop_top; }

  int32_t getCropBottom() const { return image_crop_bottom; }

  int32_t getCropLeft() const { return image_crop_left; }

  int32_t getCropRight() const { return image_crop_right; }

private:
  bool image_crop_enable;
  int32_t image_crop_top;
  int32_t image_crop_bottom;

  int32_t image_crop_left;
  int32_t image_crop_right;
};

class MSPublisher
#if defined(ROS2_FOUND)
    : public rclcpp::Node
#endif // defined(ROS2_FOUND)
{
public:
#if defined(ROS_FOUND)
  MSPublisher()
#elif defined(ROS2_FOUND)
  MSPublisher(const std::string &node_name)
      : Node(node_name)
#endif // defined(ROS2_FOUND)
  {
  }

  /**
   * @brief Destructor cleans up the device object.
   */
  ~MSPublisher() {
    stopDeviceManager();

    stopRgbWorkThreads();

    stopRgbCodec();

    stopJpegWorkThreads();

    stopJpegEncoder();

    stopDeviceCalibInfoWorkThread();
  }

public:
  int init() {
    int ret = 0;
    // Initial Parameters
    ret = initParameters();
    if (ret != 0) {
      const std::string &error_info =
          "Initial Driver Parameter(s) Failed: ret = " + std::to_string(ret);
      RS_SPDLOG_ERROR(error_info);
      return -1;
    } else {
      const std::string &error_info =
          "Initial Driver Parameter(s) Successed ! ";
      RS_SPDLOG_INFO(error_info);
    }

    // Initial Device Manager
    ret = initDeviceManager();
    if (ret != 0) {
      const std::string &error_info =
          "Initial Driver Device Manager Failed: ret = " + std::to_string(ret);
      RS_SPDLOG_ERROR(error_info);
      return -2;
    } else {
      const std::string &error_info =
          "Initial Driver Device Manager Successed ! ";
      RS_SPDLOG_INFO(error_info);
    }

    // GMSL Open
    if (device_interface_type ==
        robosense::device::DeviceInterfaceType::DEVICE_INTERFACE_GMSL) {
      image_width_driver = image_gmsl_width_ac2_driver;
      image_height_driver = image_gmsl_height_ac2_driver;

      // 更新AC类型
      lidar_type = robosense::lidar::LidarType::RS_AC2;
      ret = openDevice(gmsl_device_number);
      if (ret != 0) {
        const std::string &error_info =
            "Open Device: gmsl_device_number = " + gmsl_device_number +
            " Failed !";
        RS_SPDLOG_ERROR(error_info);
        return -3;
      } else {
        const std::string &error_info =
            "Open Device: gmsl_device_number = " + gmsl_device_number +
            " Successed !";
        RS_SPDLOG_INFO(error_info);
      }
    }

    RS_SPDLOG_INFO("Driver Initial Successed !");

    return 0;
  }

private:
  int initParameters() {
#if defined(ROS_FOUND)
    ros::NodeHandle private_nh("~"); // parameter node
    private_nh.param<std::string>("device_interface", device_interface, "usb");
    private_nh.param<int32_t>("image_input_fps", image_input_fps, 30);
    private_nh.param<int32_t>("imu_input_fps", imu_input_fps, 200);
    private_nh.param<bool>("enable_jpeg", enable_jpeg, false);
    private_nh.param<bool>("enable_rectify", enable_rectify, false);
    private_nh.param<bool>("enable_rectify_jpeg", enable_rectify_jpeg, false);
    private_nh.param<int32_t>("jpeg_quality", jpeg_quality, 70);
    private_nh.param<std::string>("topic_prefix", topic_prefix, "");
    private_nh.param<std::string>("serial_number", serial_number, "");
    private_nh.param<std::string>("gmsl_device_number", gmsl_device_number,
                                  "/dev/video30");
    private_nh.param<std::string>("point_frame_id", point_frame_id, "rslidar");
    private_nh.param<std::string>("ac1_image_frame_id", ac1_image_frame_id,
                                  "rslidar");
    private_nh.param<std::string>("ac2_left_image_frame_id",
                                  ac2_left_image_frame_id, "rslidar");
    private_nh.param<std::string>("ac2_right_image_frame_id",
                                  ac2_right_image_frame_id, "rslidar");
    private_nh.param<std::string>("imu_frame_id", imu_frame_id, "rslidar");
    private_nh.param<bool>("enable_angle_and_device_calib_info_from_device",
                           enable_angle_and_device_calib_info_from_device,
                           false);
    private_nh.param<std::string>("angle_calib_basic_dir_path",
                                  angle_calib_basic_dir_path, "");
    private_nh.param<bool>("enable_device_calib_info_from_device_pripority",
                           enable_device_calib_info_from_device_pripority,
                           false);
    private_nh.param<std::string>("device_calib_file_path",
                                  device_calib_file_path, "");
    private_nh.param<bool>("device_manager_debug", device_manager_debug, false);
    private_nh.param<bool>("enable_use_lidar_clock", enable_use_lidar_clock,
                           false);
    private_nh.param<double>("timestamp_compensate_s", timestamp_compensate_s,
                             0.0);
    private_nh.param<bool>("enable_use_dense_points", enable_use_dense_points,
                           false);
    private_nh.param<bool>("enable_use_first_point_ts",
                           enable_use_first_point_ts, false);
    private_nh.param<bool>("enable_ac2_pointcloud_wave_split",
                           enable_ac2_pointcloud_wave_split, false);
    private_nh.param<bool>("enable_ros2_zero_copy", enable_ros2_zero_copy,
                           false);
    private_nh.param<std::string>("timestamp_output_dir_path",
                                  timestamp_output_dir_path, "");
    private_nh.param<bool>("enable_pointcloud_send", enable_pointcloud_send,
                           true);
    private_nh.param<bool>("enable_ac1_image_send", enable_ac1_image_send,
                           true);
    private_nh.param<bool>("enable_ac2_left_image_send",
                           enable_ac2_left_image_send, true);
    private_nh.param<bool>("enable_ac2_right_image_send",
                           enable_ac2_right_image_send, true);
    private_nh.param<bool>("enable_imu_send", enable_imu_send, true);
    // spdlog
    log_config.reset();
    private_nh.param<std::string>("log_file_dir_path",
                                  log_config.log_file_dir_path, "");
    private_nh.param<int32_t>("log_level", log_config.log_level, 2);
    private_nh.param<bool>("is_log_file_trunc", log_config.is_log_file_trunc,
                           true);
    // ac1 crop
    int32_t ac1_crop_top, ac1_crop_bottom, ac1_crop_left, ac1_crop_right;
    private_nh.param<int32_t>("ac1_crop_top", ac1_crop_top, 0);
    private_nh.param<int32_t>("ac1_crop_bottom", ac1_crop_bottom, 0);
    private_nh.param<int32_t>("ac1_crop_left", ac1_crop_left, 0);
    private_nh.param<int32_t>("ac1_crop_right", ac1_crop_right, 0);
    ac1_crop_config.updateCrop(ac1_crop_top, ac1_crop_bottom, ac1_crop_left,
                               ac1_crop_right);
    // ac2 left crop
    int32_t ac2_left_crop_top, ac2_left_crop_bottom, ac2_left_crop_left,
        ac2_left_crop_right;
    private_nh.param<int32_t>("ac2_left_crop_top", ac2_left_crop_top, 0);
    private_nh.param<int32_t>("ac2_left_crop_bottom", ac2_left_crop_bottom, 0);
    private_nh.param<int32_t>("ac2_left_crop_left", ac2_left_crop_left, 0);
    private_nh.param<int32_t>("ac2_left_crop_right", ac2_left_crop_right, 0);
    ac2_left_crop_config.updateCrop(ac2_left_crop_top, ac2_left_crop_bottom,
                                    ac2_left_crop_left, ac2_left_crop_right);
    // ac2 right crop
    int32_t ac2_right_crop_top, ac2_right_crop_bottom, ac2_right_crop_left,
        ac2_right_crop_right;
    private_nh.param<int32_t>("ac2_right_crop_top", ac2_right_crop_top, 0);
    private_nh.param<int32_t>("ac2_right_crop_bottom", ac2_right_crop_bottom,
                              0);
    private_nh.param<int32_t>("ac2_right_crop_left", ac2_right_crop_left, 0);
    private_nh.param<int32_t>("ac2_right_crop_right", ac2_right_crop_right, 0);
    ac2_right_crop_config.updateCrop(ac2_right_crop_top, ac2_right_crop_bottom,
                                     ac2_right_crop_left, ac2_right_crop_right);
#if defined(ENABLE_SUPPORT_RS_DRIVER_ALGORITHM)
    // AC2 Denoise parameter
    private_nh.param<bool>("enable_denoise",
                           algorithm_param.denoise_param.enable_denoise, false);
    private_nh.param<bool>("enable_smooth",
                           algorithm_param.denoise_param.enable_smooth, false);
    int32_t dist_x_win_cfg, dist_y_win_cfg;
    private_nh.param<int32_t>("dist_x_win_cfg", dist_x_win_cfg, 1);
    algorithm_param.denoise_param.dist_x_win_cfg = dist_x_win_cfg;
    private_nh.param<int32_t>("dist_y_win_cfg", dist_y_win_cfg, 1);
    algorithm_param.denoise_param.dist_y_win_cfg = dist_y_win_cfg;
    int32_t dist_valid_thresholds_0, dist_valid_thresholds_1,
        dist_valid_thresholds_2, dist_valid_thresholds_3,
        dist_valid_thresholds_4;
    private_nh.param<int32_t>("dist_valid_thresholds_0",
                              dist_valid_thresholds_0, 3);
    algorithm_param.denoise_param.dist_valid_thresholds[0] =
        dist_valid_thresholds_0;
    private_nh.param<int32_t>("dist_valid_thresholds_1",
                              dist_valid_thresholds_1, 3);
    algorithm_param.denoise_param.dist_valid_thresholds[1] =
        dist_valid_thresholds_1;
    private_nh.param<int32_t>("dist_valid_thresholds_2",
                              dist_valid_thresholds_2, 3);
    algorithm_param.denoise_param.dist_valid_thresholds[2] =
        dist_valid_thresholds_2;
    private_nh.param<int32_t>("dist_valid_thresholds_3",
                              dist_valid_thresholds_3, 2);
    algorithm_param.denoise_param.dist_valid_thresholds[3] =
        dist_valid_thresholds_3;
    private_nh.param<int32_t>("dist_valid_thresholds_4",
                              dist_valid_thresholds_4, 2);
    algorithm_param.denoise_param.dist_valid_thresholds[4] =
        dist_valid_thresholds_4;
    int32_t max_process_distance, min_process_distance;
    private_nh.param<int32_t>("max_process_distance", max_process_distance,
                              65535);
    algorithm_param.denoise_param.max_process_distance = max_process_distance;
    private_nh.param<int32_t>("min_process_distance", min_process_distance, 0);
    algorithm_param.denoise_param.min_process_distance = min_process_distance;
    // AC2 Edge parameter
    int32_t edge_kernel_size;
    private_nh.param<int32_t>("edge_kernel_size", edge_kernel_size, 3);
    algorithm_param.edge_param.edge_kernel_size = edge_kernel_size;
    // AC2 Deblooming parameter
    private_nh.param<bool>("enable_debloom",
                           algorithm_param.debloom_param.enable_debloom, false);
    int32_t search_range, distance_diff_threshold, delete_intensity_threshold,
        edge_thresholds_0, edge_thresholds_1, intensity_mutation_thresholds_0,
        intensity_mutation_thresholds_1, target_intensity_thresholds_0,
        target_intensity_thresholds_1, target_intensity_thresholds_2;
    private_nh.param<int32_t>("search_range", search_range, 5);
    algorithm_param.debloom_param.search_range = search_range;
    private_nh.param<int32_t>("distance_diff_threshold",
                              distance_diff_threshold, 50);
    algorithm_param.debloom_param.distance_diff_threshold =
        distance_diff_threshold;
    private_nh.param<int32_t>("delete_intensity_threshold",
                              delete_intensity_threshold, 80);
    algorithm_param.debloom_param.delete_intensity_threshold =
        delete_intensity_threshold;
    private_nh.param<int32_t>("edge_thresholds_0", edge_thresholds_0, 1000);
    algorithm_param.debloom_param.edge_thresholds[0] = edge_thresholds_0;
    private_nh.param<int32_t>("edge_thresholds_1", edge_thresholds_1, 1000);
    algorithm_param.debloom_param.edge_thresholds[1] = edge_thresholds_1;
    private_nh.param<int32_t>("intensity_mutation_thresholds_0",
                              intensity_mutation_thresholds_0, 50);
    private_nh.param<int32_t>("intensity_mutation_thresholds_1",
                              intensity_mutation_thresholds_1, 36);
    algorithm_param.debloom_param.intensity_mutation_thresholds[0] =
        intensity_mutation_thresholds_0;
    algorithm_param.debloom_param.intensity_mutation_thresholds[1] =
        intensity_mutation_thresholds_1;
    private_nh.param<int32_t>("target_intensity_thresholds_0",
                              target_intensity_thresholds_0, 10);
    private_nh.param<int32_t>("target_intensity_thresholds_1",
                              target_intensity_thresholds_1, 45);
    private_nh.param<int32_t>("target_intensity_thresholds_2",
                              target_intensity_thresholds_2, 100);
    algorithm_param.debloom_param.target_intensity_thresholds[0] =
        target_intensity_thresholds_0;
    algorithm_param.debloom_param.target_intensity_thresholds[1] =
        target_intensity_thresholds_1;
    algorithm_param.debloom_param.target_intensity_thresholds[2] =
        target_intensity_thresholds_2;
    // AC2 Detrail parameter
    private_nh.param<bool>("enable_detrail",
                           algorithm_param.detrail_param.enable_detrail, false);
    int32_t fwhm_thresholds_0, fwhm_thresholds_1, fwhm_thresholds_2,
        fwhm_thresholds_3;
    int32_t detrail_edge_thresholds_0, detrail_edge_thresholds_1,
        detrail_edge_thresholds_2;
    int32_t distance_thresholds_0, distance_thresholds_1, distance_thresholds_2;
    int32_t noise_thresholds_0, noise_thresholds_1;
    int32_t peak_value_threshold;
    int32_t detrail_distance_diff_threshold;
    private_nh.param<int32_t>("fwhm_thresholds_0", fwhm_thresholds_0, 95);
    private_nh.param<int32_t>("fwhm_thresholds_1", fwhm_thresholds_0, 90);
    private_nh.param<int32_t>("fwhm_thresholds_2", fwhm_thresholds_0, 85);
    private_nh.param<int32_t>("fwhm_thresholds_3", fwhm_thresholds_0, 80);
    algorithm_param.detrail_param.fwhm_thresholds[0] = fwhm_thresholds_0;
    algorithm_param.detrail_param.fwhm_thresholds[1] = fwhm_thresholds_1;
    algorithm_param.detrail_param.fwhm_thresholds[2] = fwhm_thresholds_2;
    algorithm_param.detrail_param.fwhm_thresholds[3] = fwhm_thresholds_3;
    private_nh.param<int32_t>("detrail_edge_thresholds_0",
                              detrail_edge_thresholds_0, 200);
    private_nh.param<int32_t>("detrail_edge_thresholds_1",
                              detrail_edge_thresholds_1, 500);
    private_nh.param<int32_t>("detrail_edge_thresholds_2",
                              detrail_edge_thresholds_2, 4000);
    algorithm_param.detrail_param.edge_thresholds[0] =
        detrail_edge_thresholds_0;
    algorithm_param.detrail_param.edge_thresholds[1] =
        detrail_edge_thresholds_1;
    algorithm_param.detrail_param.edge_thresholds[2] =
        detrail_edge_thresholds_2;
    private_nh.param<int32_t>("distance_thresholds_0", distance_thresholds_0,
                              600);
    private_nh.param<int32_t>("distance_thresholds_1", distance_thresholds_1,
                              1000);
    private_nh.param<int32_t>("distance_thresholds_2", distance_thresholds_2,
                              1500);
    algorithm_param.detrail_param.distance_thresholds[0] =
        distance_thresholds_0;
    algorithm_param.detrail_param.distance_thresholds[1] =
        distance_thresholds_1;
    algorithm_param.detrail_param.distance_thresholds[2] =
        distance_thresholds_2;
    private_nh.param<int32_t>("noise_thresholds_0", noise_thresholds_0, 15);
    private_nh.param<int32_t>("noise_thresholds_1", noise_thresholds_1, 200);
    algorithm_param.detrail_param.noise_thresholds[0] = noise_thresholds_0;
    algorithm_param.detrail_param.noise_thresholds[1] = noise_thresholds_1;
    private_nh.param<int32_t>("peak_value_threshold", peak_value_threshold, 10);
    algorithm_param.detrail_param.peak_value_threshold = peak_value_threshold;
    private_nh.param<int32_t>("detrail_distance_diff_threshold",
                              detrail_distance_diff_threshold, 30);
    algorithm_param.detrail_param.distance_diff_threshold =
        detrail_distance_diff_threshold;
    // AC2 Frame filter parameter
    private_nh.param<bool>(
        "enable_frame_filter",
        algorithm_param.frame_filter_param.enable_frame_filter, true);
    private_nh.param<bool>(
        "enable_save_raw_data",
        algorithm_param.frame_filter_param.enable_save_raw_data, false);
    int32_t smooth_frame_count, imu_motion_detect_frame_count,
        imu_motion_threshold, stationary_ratio;
    private_nh.param<int32_t>("smooth_frame_count", smooth_frame_count, 5);
    algorithm_param.frame_filter_param.smooth_frame_count = smooth_frame_count;
    private_nh.param<int32_t>("imu_motion_detect_frame_count",
                              imu_motion_detect_frame_count, 5);
    algorithm_param.frame_filter_param.imu_motion_detect_frame_count =
        imu_motion_detect_frame_count;
    private_nh.param<int32_t>("imu_motion_threshold", imu_motion_threshold, 3);
    algorithm_param.frame_filter_param.imu_motion_threshold =
        imu_motion_threshold;
    private_nh.param<int32_t>("stationary_ratio", stationary_ratio, 10);
    algorithm_param.frame_filter_param.stationary_ratio = stationary_ratio;
#endif // defined(ENABLE_SUPPORT_RS_DRIVER_ALGORITHM)
#elif defined(ROS2_FOUND)
    device_interface =
        declare_parameter<std::string>("device_interface", "usb");
    image_input_fps = declare_parameter<int32_t>("image_input_fps", 30);
    imu_input_fps = declare_parameter<int32_t>("imu_input_fps", 200);
    enable_jpeg = declare_parameter<bool>("enable_jpeg", false);
    enable_rectify = declare_parameter<bool>("enable_rectify", false);
    enable_rectify_jpeg = declare_parameter<bool>("enable_rectify_jpeg", false);
    jpeg_quality = declare_parameter<int32_t>("jpeg_quality", 70);
    topic_prefix = declare_parameter<std::string>("topic_prefix", "");
    serial_number = declare_parameter<std::string>("serial_number", "");
    gmsl_device_number =
        declare_parameter<std::string>("gmsl_device_number", "/dev/video30");
    point_frame_id =
        declare_parameter<std::string>("point_frame_id", "rslidar");
    ac1_image_frame_id =
        declare_parameter<std::string>("ac1_image_frame_id", "rslidar");
    ac2_left_image_frame_id =
        declare_parameter<std::string>("ac2_left_image_frame_id", "rslidar");
    ac2_right_image_frame_id =
        declare_parameter<std::string>("ac2_right_image_frame_id", "rslidar");
    imu_frame_id = declare_parameter<std::string>("imu_frame_id", "rslidar");
    enable_angle_and_device_calib_info_from_device = declare_parameter<bool>(
        "enable_angle_and_device_calib_info_from_device", true);
    angle_calib_basic_dir_path =
        declare_parameter<std::string>("angle_calib_basic_dir_path", "");
    enable_device_calib_info_from_device_pripority = declare_parameter<bool>(
        "enable_device_calib_info_from_device_pripority", true);
    device_calib_file_path =
        declare_parameter<std::string>("device_calib_file_path", "");
    device_manager_debug =
        declare_parameter<bool>("device_manager_debug", false);
    enable_use_lidar_clock =
        declare_parameter<bool>("enable_use_lidar_clock", false);
    timestamp_compensate_s =
        declare_parameter<double>("timestamp_compensate_s", 0.0);
    enable_use_dense_points =
        declare_parameter<bool>("enable_use_dense_points", false);
    enable_use_first_point_ts =
        declare_parameter<bool>("enable_use_first_point_ts", false);
    enable_ac2_pointcloud_wave_split =
        declare_parameter<bool>("enable_ac2_pointcloud_wave_split", false);
    enable_ros2_zero_copy =
        declare_parameter<bool>("enable_ros2_zero_copy", false);
    timestamp_output_dir_path =
        declare_parameter<std::string>("timestamp_output_dir_path", "");
    enable_pointcloud_send =
        declare_parameter<bool>("enable_pointcloud_send", true);
    enable_ac1_image_send =
        declare_parameter<bool>("enable_ac1_image_send", true);
    enable_ac2_left_image_send =
        declare_parameter<bool>("enable_ac2_left_image_send", true);
    enable_ac2_right_image_send =
        declare_parameter<bool>("enable_ac2_right_image_send", true);
    enable_imu_send = declare_parameter<bool>("enable_imu_send", true);
    // spdlog
    log_config.reset();
    log_config.log_file_dir_path =
        declare_parameter<std::string>("log_file_dir_path", "");
    log_config.log_level = declare_parameter<int32_t>("log_level", 2);
    log_config.is_log_file_trunc =
        declare_parameter<bool>("is_log_file_trunc", true);
    // ac1 crop
    int32_t ac1_crop_top, ac1_crop_bottom, ac1_crop_left, ac1_crop_right;
    ac1_crop_top = declare_parameter<int32_t>("ac1_crop_top", 0);
    ac1_crop_bottom = declare_parameter<int32_t>("ac1_crop_bottom", 0);
    ac1_crop_left = declare_parameter<int32_t>("ac1_crop_left", 0);
    ac1_crop_right = declare_parameter<int32_t>("ac1_crop_right", 0);
    ac1_crop_config.updateCrop(ac1_crop_top, ac1_crop_bottom, ac1_crop_left,
                               ac1_crop_right);
    // ac2 left crop
    int32_t ac2_left_crop_top, ac2_left_crop_bottom, ac2_left_crop_left,
        ac2_left_crop_right;
    ac2_left_crop_top = declare_parameter<int32_t>("ac2_left_crop_top", 0);
    ac2_left_crop_bottom =
        declare_parameter<int32_t>("ac2_left_crop_bottom", 0);
    ac2_left_crop_left = declare_parameter<int32_t>("ac2_left_crop_left", 0);
    ac2_left_crop_right = declare_parameter<int32_t>("ac2_left_crop_right", 0);
    ac2_left_crop_config.updateCrop(ac2_left_crop_top, ac2_left_crop_bottom,
                                    ac2_left_crop_left, ac2_left_crop_right);
    // ac2 right crop
    int32_t ac2_right_crop_top, ac2_right_crop_bottom, ac2_right_crop_left,
        ac2_right_crop_right;
    ac2_right_crop_top = declare_parameter<int32_t>("ac2_right_crop_top", 0);
    ac2_right_crop_bottom =
        declare_parameter<int32_t>("ac2_right_crop_bottom", 0);
    ac2_right_crop_left = declare_parameter<int32_t>("ac2_right_crop_left", 0);
    ac2_right_crop_right =
        declare_parameter<int32_t>("ac2_right_crop_right", 0);
    ac2_right_crop_config.updateCrop(ac2_right_crop_top, ac2_right_crop_bottom,
                                     ac2_right_crop_left, ac2_right_crop_right);
#if defined(ENABLE_SUPPORT_RS_DRIVER_ALGORITHM)
    // AC2 Denoise parameter
    algorithm_param.denoise_param.enable_denoise =
        declare_parameter<bool>("enable_denoise", false);
    algorithm_param.denoise_param.enable_smooth =
        declare_parameter<bool>("enable_smooth", false);
    algorithm_param.denoise_param.dist_x_win_cfg =
        declare_parameter<int32_t>("dist_x_win_cfg", 1);
    algorithm_param.denoise_param.dist_y_win_cfg =
        declare_parameter<int32_t>("dist_y_win_cfg", 1);
    algorithm_param.denoise_param.dist_valid_thresholds[0] =
        declare_parameter<int32_t>("dist_valid_thresholds_0", 3);
    algorithm_param.denoise_param.dist_valid_thresholds[1] =
        declare_parameter<int32_t>("dist_valid_thresholds_1", 3);
    algorithm_param.denoise_param.dist_valid_thresholds[2] =
        declare_parameter<int32_t>("dist_valid_thresholds_2", 3);
    algorithm_param.denoise_param.dist_valid_thresholds[3] =
        declare_parameter<int32_t>("dist_valid_thresholds_3", 2);
    algorithm_param.denoise_param.dist_valid_thresholds[4] =
        declare_parameter<int32_t>("dist_valid_thresholds_4", 2);
    algorithm_param.denoise_param.max_process_distance =
        declare_parameter<int32_t>("max_process_distance", 65535);
    algorithm_param.denoise_param.min_process_distance =
        declare_parameter<int32_t>("min_process_distance", 0);
    // AC2 Edge parameter
    algorithm_param.edge_param.edge_kernel_size =
        declare_parameter<int32_t>("edge_kernel_size", 3);
    // AC2 Deblooming parameter
    algorithm_param.debloom_param.enable_debloom =
        declare_parameter<bool>("enable_debloom", false);
    algorithm_param.debloom_param.search_range =
        declare_parameter<int32_t>("search_range", 5);
    algorithm_param.debloom_param.distance_diff_threshold =
        declare_parameter<int32_t>("distance_diff_threshold", 50);
    algorithm_param.debloom_param.delete_intensity_threshold =
        declare_parameter<int32_t>("delete_intensity_threshold", 200);
    algorithm_param.debloom_param.edge_thresholds[0] =
        declare_parameter<int32_t>("edge_thresholds_0", 2000);
    algorithm_param.debloom_param.edge_thresholds[1] =
        declare_parameter<int32_t>("edge_thresholds_1", 200);
    algorithm_param.debloom_param.intensity_mutation_thresholds[0] =
        declare_parameter<int32_t>("intensity_mutation_thresholds_0", 50);
    algorithm_param.debloom_param.intensity_mutation_thresholds[1] =
        declare_parameter<int32_t>("intensity_mutation_thresholds_1", 36);
    algorithm_param.debloom_param.target_intensity_thresholds[0] =
        declare_parameter<int32_t>("target_intensity_thresholds_0", 10);
    algorithm_param.debloom_param.target_intensity_thresholds[1] =
        declare_parameter<int32_t>("target_intensity_thresholds_1", 45);
    algorithm_param.debloom_param.target_intensity_thresholds[2] =
        declare_parameter<int32_t>("target_intensity_thresholds_2", 100);
    // AC2 Trail parameter
    algorithm_param.detrail_param.enable_detrail =
        declare_parameter<bool>("enable_detrail", false);
    algorithm_param.detrail_param.fwhm_thresholds[0] =
        declare_parameter<int32_t>("fwhm_thresholds_0", 95);
    algorithm_param.detrail_param.fwhm_thresholds[1] =
        declare_parameter<int32_t>("fwhm_thresholds_1", 90);
    algorithm_param.detrail_param.fwhm_thresholds[2] =
        declare_parameter<int32_t>("fwhm_thresholds_2", 85);
    algorithm_param.detrail_param.fwhm_thresholds[3] =
        declare_parameter<int32_t>("fwhm_thresholds_3", 80);
    algorithm_param.detrail_param.edge_thresholds[0] =
        declare_parameter<int32_t>("detrail_edge_thresholds_0", 200);
    algorithm_param.detrail_param.edge_thresholds[1] =
        declare_parameter<int32_t>("detrail_edge_thresholds_1", 500);
    algorithm_param.detrail_param.edge_thresholds[2] =
        declare_parameter<int32_t>("detrail_edge_thresholds_2", 4000);
    algorithm_param.detrail_param.distance_thresholds[0] =
        declare_parameter<int32_t>("distance_thresholds_0", 600);
    algorithm_param.detrail_param.distance_thresholds[1] =
        declare_parameter<int32_t>("distance_thresholds_1", 1000);
    algorithm_param.detrail_param.distance_thresholds[2] =
        declare_parameter<int32_t>("distance_thresholds_2", 1500);
    algorithm_param.detrail_param.noise_thresholds[0] =
        declare_parameter<int32_t>("noise_thresholds_0", 15);
    algorithm_param.detrail_param.noise_thresholds[1] =
        declare_parameter<int32_t>("noise_thresholds_1", 200);
    algorithm_param.detrail_param.peak_value_threshold =
        declare_parameter<int32_t>("peak_value_threshold", 10);
    algorithm_param.detrail_param.distance_diff_threshold =
        declare_parameter<int32_t>("detrail_distance_diff_threshold", 30);
    // AC2 Frame filter parameter
    algorithm_param.frame_filter_param.enable_frame_filter =
        declare_parameter<bool>("enable_frame_filter", false);
    algorithm_param.frame_filter_param.enable_save_raw_data =
        declare_parameter<bool>("enable_save_raw_data", false);
    algorithm_param.frame_filter_param.smooth_frame_count =
        declare_parameter<int32_t>("smooth_frame_count", 5);
    algorithm_param.frame_filter_param.imu_motion_detect_frame_count =
        declare_parameter<int32_t>("imu_motion_detect_frame_count", 5);
    algorithm_param.frame_filter_param.imu_motion_threshold =
        declare_parameter<int32_t>("imu_motion_threshold", 3);
    algorithm_param.frame_filter_param.stationary_ratio =
        declare_parameter<int32_t>("stationary_ratio", 10);
#endif // defined(ENABLE_SUPPORT_RS_DRIVER_ALGORITHM)
#endif // defined(ROS_ROS2_FOUND)

    // 强制ROS1 设置为非零拷贝
#if defined(ROS_FOUND)
    enable_ros2_zero_copy = false;
#endif // defined(ROS_FOUND)

    // Initial Log
    int ret = initLogManager();
    if (ret != 0) {
      logError("Initial Log Manager Failed: ret = " + std::to_string(ret));
      return -1;
    }

    device_interface_type = robosense::device::RSDeviceInterfaceUtil::
        fromStringToDeviceInterfaceType(device_interface);
    if (device_interface_type ==
            robosense::device::DeviceInterfaceType::DEVICE_INTERFACE_GMSL &&
        gmsl_device_number.empty()) {
      const std::string &error_info = "Setting GMSL Device Interface But Not "
                                      "Setting \"gmsl_device_number\"";
      RS_SPDLOG_ERROR(error_info);
      return -1;
    }
    if (enable_ac2_pointcloud_wave_split) {
      const std::string &error_info =
          "Enable Use AC2 PointCloud Wave Split: Not Support Use Dense Points, "
          "Force enable_use_dense_points = false !";
      RS_SPDLOG_WARN(error_info);
      enable_use_dense_points = false;
    }

    // 创建全部话题的名称(s)
    initTopicNames();

    std::ostringstream ofstr;
    ofstr << "device_interface = " << device_interface
          << ", image_input_fps = " << image_input_fps
          << ", imu_input_fps = " << imu_input_fps
          << ", enable_jpeg = " << enable_jpeg
          << ", enable_rectify = " << enable_rectify
          << ", enable_rectify_jpeg = " << enable_rectify_jpeg
          << ", jpeg_quality = " << jpeg_quality
          << ", topic_prefix = " << topic_prefix
          << ", serial_number = " << serial_number
          << ", gmsl_device_number = " << gmsl_device_number
          << ", enable_angle_and_device_calib_info_from_device = "
          << enable_angle_and_device_calib_info_from_device
          << ", angle_calib_basic_dir_path = " << angle_calib_basic_dir_path
          << ", enable_device_calib_info_from_device_pripority = "
          << enable_device_calib_info_from_device_pripority
          << ", device_calib_file_path = " << device_calib_file_path
          << ", device_manager_debug = " << device_manager_debug
          << ", enable_use_lidar_clock = " << enable_use_lidar_clock
          << ", timestamp_compensate_s = "
          << std::to_string(timestamp_compensate_s)
          << ", enable_use_dense_points = " << enable_use_dense_points
          << ", enable_use_first_point_ts = " << enable_use_first_point_ts
          << ", enable_ac2_pointcloud_wave_split = "
          << enable_ac2_pointcloud_wave_split
          << ", timestamp_output_dir_path = " << timestamp_output_dir_path
          << ", enable_pointcloud_send = " << enable_pointcloud_send
          << ", enable_ac1_image_send = " << enable_ac1_image_send
          << ", enable_ac2_left_image_send = " << enable_ac2_left_image_send
          << ", enable_ac2_right_image_send = " << enable_ac2_right_image_send
          << ", enable_imu_send = " << enable_imu_send
          << ", enable_ros2_zero_copy(only for ros2) = "
          << enable_ros2_zero_copy
          << ", ac1 crop = " << ac1_crop_config.toString()
          << ", ac2 left crop = " << ac2_left_crop_config.toString()
          << ", ac2 right crop = " << ac2_right_crop_config.toString();
    RS_SPDLOG_INFO(ofstr.str());

    if (!(enable_pointcloud_send || enable_ac1_image_send ||
          enable_ac2_left_image_send || enable_ac2_right_image_send ||
          enable_imu_send)) {
      RS_SPDLOG_WARN("No Any Data Need Output By ROS/ROS2, AC Driver Exit !");
      return -2;
    }

    return 0;
  }

  int initLogManager() {
    // 更新log文件夹路径
    if (!log_config.log_file_dir_path.empty()) {
      std::string tmp_topic_prefix = topic_prefix;
      if (!topic_prefix.empty()) {
        std::replace(tmp_topic_prefix.begin(), tmp_topic_prefix.end(), '/',
                     '_');
      } else {
        tmp_topic_prefix = "ac2_log";
      }
      const std::string &full_file_path =
          log_config.log_file_dir_path + "/" + tmp_topic_prefix + "_" +
          robosense::device::RSTimeFormatUtil().currentTimeString() + ".log";
      log_config.log_file_path = full_file_path;
    }
    int ret = robosense::log::RSLogManager::init(log_config);
    if (ret != 0) {
      RS_ERROR << "Initial Log Manager Failed: ret = " << ret << RS_REND;
      return -1;
    }
    return 0;
  }

  int initDeviceManager() {
    try {
      device_manager_ptr.reset(new robosense::device::DeviceManager());
    } catch (...) {
      RS_SPDLOG_ERROR("Malloc Device Manager Failed !");
      return -1;
    }

    device_manager_ptr->regDeviceEventCallback(std::bind(
        &MSPublisher::deviceEventCallback, this, std::placeholders::_1));

    device_manager_ptr->regPointCloudCallback(
        std::bind(&MSPublisher::pointCloudCallback, this, std::placeholders::_1,
                  std::placeholders::_2, std::placeholders::_3));

    device_manager_ptr->regImageDataCallback(
        std::bind(&MSPublisher::imageCallback, this, std::placeholders::_1,
                  std::placeholders::_2, std::placeholders::_3));

    device_manager_ptr->regImuDataCallback(
        std::bind(&MSPublisher::imuCallback, this, std::placeholders::_1,
                  std::placeholders::_2, std::placeholders::_3));

    device_manager_ptr->regExceptionCallback(std::bind(
        &MSPublisher::exceptionCallback, this, std::placeholders::_1));

    bool isSuccess =
        device_manager_ptr->init(device_interface_type, device_manager_debug);
    if (!isSuccess) {
      RS_SPDLOG_ERROR("Device Manager Initial Failed !");
      return -2;
    } else {
      RS_SPDLOG_INFO("Initial Device Manager Successed !");
    }

    return 0;
  }

  int initTimestampManager() {
    std::string tmp_topic_prefix = topic_prefix;
    if (!topic_prefix.empty()) {
      std::replace(tmp_topic_prefix.begin(), tmp_topic_prefix.end(), '/', '_');
    } else {
      tmp_topic_prefix = "ac2_stat";
    }
    const std::string &full_file_path =
        timestamp_output_dir_path + "/" + tmp_topic_prefix + "_" +
        robosense::device::RSTimeFormatUtil().currentTimeString() + ".csv";
    try {
      timestamp_manager_ptr.reset(new robosense::device::RSTimestampManager());
    } catch (...) {
      RS_SPDLOG_ERROR("Malloc Timestamp Manager Failed !");
      return -1;
    }

    int ret = timestamp_manager_ptr->init(full_file_path, 1200);
    if (ret != 0) {
      const std::string &error_info =
          "Initial Timestamp Manager Failed: ret = " + std::to_string(ret);
      RS_SPDLOG_ERROR(error_info);
      return -2;
    } else {
      const std::string &error_info =
          "Initial Timestamp Manager Successed: full_file_path = " +
          full_file_path;
      RS_SPDLOG_INFO(error_info);
    }

    // 更新时间戳配置
    if (timestamp_manager_ptr) {
      timestamp_manager_ptr->addChannelId(
          robosense::device::RS_CHANNEL_ID_TYPE::RS_CHANNEL_ID_RGB_IMAGE,
          topic_name);

      timestamp_manager_ptr->addChannelId(
          robosense::device::RS_CHANNEL_ID_TYPE::RS_CHANNEL_ID_RGB_LEFT_IMAGE,
          left_topic_name);

      timestamp_manager_ptr->addChannelId(
          robosense::device::RS_CHANNEL_ID_TYPE::RS_CHANNEL_ID_RGB_RIGHT_IMAGE,
          right_topic_name);

      timestamp_manager_ptr->addChannelId(
          robosense::device::RS_CHANNEL_ID_TYPE::
              RS_CHANNEL_ID_RGB_RECTIFY_IMAGE,
          rectify_topic_name);

      timestamp_manager_ptr->addChannelId(
          robosense::device::RS_CHANNEL_ID_TYPE::
              RS_CHANNEL_ID_RGB_RECTIFY_LEFT_IMAGE,
          rectify_left_topic_name);

      timestamp_manager_ptr->addChannelId(
          robosense::device::RS_CHANNEL_ID_TYPE::
              RS_CHANNEL_ID_RGB_RECTIFY_RIGHT_IMAGE,
          rectify_right_topic_name);

      timestamp_manager_ptr->addChannelId(
          robosense::device::RS_CHANNEL_ID_TYPE::RS_CHANNEL_ID_POINTCLOUD,
          pointcloud_topic_name);

      timestamp_manager_ptr->addChannelId(
          robosense::device::RS_CHANNEL_ID_TYPE::
              RS_CHANNEL_ID_POINTCLOUD_AC2_WAVE2,
          pointcloud_ac2_wave2_topic_name);

      timestamp_manager_ptr->addChannelId(
          robosense::device::RS_CHANNEL_ID_TYPE::RS_CHANNEL_ID_IMU,
          imu_topic_name);

      timestamp_manager_ptr->addChannelId(
          robosense::device::RS_CHANNEL_ID_TYPE::RS_CHANNEL_ID_JPEG_IMAGE,
          jpeg_topic_name);

      timestamp_manager_ptr->addChannelId(
          robosense::device::RS_CHANNEL_ID_TYPE::RS_CHANNEL_ID_JPEG_LEFT_IMAGE,
          jpeg_left_topic_name);

      timestamp_manager_ptr->addChannelId(
          robosense::device::RS_CHANNEL_ID_TYPE::RS_CHANNEL_ID_JPEG_RIGHT_IMAGE,
          jpeg_right_topic_name);

      timestamp_manager_ptr->addChannelId(
          robosense::device::RS_CHANNEL_ID_TYPE::
              RS_CHANNEL_ID_JPEG_RECTIFY_IMAGE,
          jpeg_rectify_topic_name);

      timestamp_manager_ptr->addChannelId(
          robosense::device::RS_CHANNEL_ID_TYPE::
              RS_CHANNEL_ID_JPEG_RECTIFY_LEFT_IMAGE,
          jpeg_rectify_left_topic_name);

      timestamp_manager_ptr->addChannelId(
          robosense::device::RS_CHANNEL_ID_TYPE::
              RS_CHANNEL_ID_JPEG_RECTIFY_RIGHT_IMAGE,
          jpeg_rectify_right_topic_name);
    }

    return 0;
  }

  int stopTimestampManager() {
    if (timestamp_manager_ptr) {
      timestamp_manager_ptr.reset();
    }
    RS_SPDLOG_INFO("Stop Timestamp Manager Successed !");
    return 0;
  }

  int stopDeviceManager() {
    if (device_manager_ptr) {
      device_manager_ptr->stop();
    }
    device_manager_ptr.reset();
    RS_SPDLOG_INFO("Stop Device Manager Successed !");
    return 0;
  }

  int initTopicNames() {
    topic_name = topic_prefix + "/rs_camera/color/image_raw";
    left_topic_name = topic_prefix + "/rs_camera/left/color/image_raw";
    right_topic_name = topic_prefix + "/rs_camera/right/color/image_raw";
    rectify_topic_name = topic_prefix + "/rs_camera/rect/color/image_raw";
    rectify_left_topic_name =
        topic_prefix + "/rs_camera/left/rect/color/image_raw";
    rectify_right_topic_name =
        topic_prefix + "/rs_camera/right/rect/color/image_raw";
    pointcloud_topic_name = topic_prefix + "/rs_lidar/points";
    pointcloud_ac2_wave2_topic_name =
        topic_prefix + "/rs_lidar/ac2_wave2/points";
    imu_topic_name = topic_prefix + "/rs_imu";
    jpeg_topic_name = topic_prefix + "/rs_camera/color/image_raw/compressed";
    jpeg_left_topic_name =
        topic_prefix + "/rs_camera/left/color/image_raw/compressed";
    jpeg_right_topic_name =
        topic_prefix + "/rs_camera/right/color/image_raw/compressed";
    jpeg_rectify_topic_name =
        topic_prefix + "/rs_camera/rect/color/image_raw/compressed";
    jpeg_rectify_left_topic_name =
        topic_prefix + "/rs_camera/left/rect/color/image_raw/compressed";
    jpeg_rectify_right_topic_name =
        topic_prefix + "/rs_camera/right/rect/color/image_raw/compressed";
    camera_info_topic_name = topic_prefix + "/rs_camera/color/camera_info";
    camera_info_left_topic_name =
        topic_prefix + "/rs_camera/left/color/camera_info";
    camera_info_right_topic_name =
        topic_prefix + "/rs_camera/right/color/camera_info";
    ac_device_calib_info_topic_name = topic_prefix + "/device_calib_info";
    return 0;
  }

  template <typename ROS_MESSAGE_TYPE>
#if defined(ROS_FOUND)
  std::shared_ptr<ros::Publisher>
#elif defined(ROS2_FOUND)
  std::shared_ptr<rclcpp::Publisher<ROS_MESSAGE_TYPE>>
#endif // defined(ROS_ROS2_FOUND)
  create_ros_publisher(const std::string &topic_name, const uint32_t depth) {
#if defined(ROS_FOUND)
    auto publisherPtr =
        robosense::interface::RSRosManager::create_publisher<ROS_MESSAGE_TYPE>(
            nh, topic_name, depth);
#elif defined(ROS2_FOUND)
    auto publisherPtr =
        robosense::interface::RSRosManager::create_publisher<ROS_MESSAGE_TYPE>(
            this, topic_name, rclcpp::QoS(depth));
#endif // defined(ROS_ROS2_FOUND)
    if (publisherPtr == nullptr) {
      RS_SPDLOG_ERROR(std::string("Create TopicName = ") + topic_name +
                      " Failed !");
    }
    return publisherPtr;
  }

  int initPublishers() {
    if (lidar_type != robosense::lidar::LidarType::RS_AC2) {
      enable_ac2_pointcloud_wave_split = false;
      const std::string &error_info =
          "lidar_type = " + robosense::lidar::lidarTypeToStr(lidar_type) +
          " Not Support PointCloud Wave Split, Force "
          "enable_ac2_pointcloud_wave_split = false !";
      RS_SPDLOG_WARN(error_info);
      enable_ac2_pointcloud_wave_split = false;
    }

    if (!enable_ros2_zero_copy) {
      if (lidar_type == robosense::lidar::LidarType::RS_AC1) {
        if (enable_ac1_image_send) {
          publisher_rgb = create_ros_publisher<ROS_IMAGE>(topic_name, 10);
          if (enable_rectify) {
            publisher_rgb_rectify =
                create_ros_publisher<ROS_IMAGE>(rectify_topic_name, 10);
          }
        } else {
          RS_SPDLOG_WARN("Disable AC1 Image Rgb Send By ROS2 !");
        }
      } else if (lidar_type == robosense::lidar::LidarType::RS_AC2) {
        if (enable_ac2_left_image_send) {
          publisher_rgb_left =
              create_ros_publisher<ROS_IMAGE>(left_topic_name, 10);
          if (enable_rectify) {
            publisher_rgb_rectify_left =
                create_ros_publisher<ROS_IMAGE>(rectify_left_topic_name, 10);
          }
        } else {
          RS_SPDLOG_WARN("Disable AC2 Left Image Rgb Send By ROS2 !");
        }
        if (enable_ac2_right_image_send) {
          publisher_rgb_right =
              create_ros_publisher<ROS_IMAGE>(right_topic_name, 10);
          if (enable_rectify) {
            publisher_rgb_rectify_right =
                create_ros_publisher<ROS_IMAGE>(rectify_right_topic_name, 10);
          }
        } else {
          RS_SPDLOG_WARN("Disable AC2 Right Image Rgb Send By ROS2 !");
        }
      }
      if (enable_pointcloud_send) {
        publisher_depth =
            create_ros_publisher<ROS_POINTCLOUD2>(pointcloud_topic_name, 10);
        if (enable_ac2_pointcloud_wave_split) {
          publisher_depth_ac2_wave2 = create_ros_publisher<ROS_POINTCLOUD2>(
              pointcloud_ac2_wave2_topic_name, 10);
        }
      } else {
        RS_SPDLOG_WARN("Disable PointCloud Send By ROS2 !");
      }
    }
#if defined(ROS2_FOUND)
    else {
      if (lidar_type == robosense::lidar::LidarType::RS_AC1) {
        if (enable_ac1_image_send) {
          publisher_rgb_loan =
              create_ros_publisher<ROS_ZEROCOPY_IMAGE8M>(topic_name, 10);
          if (enable_rectify) {
            publisher_rgb_rectify_loan =
                create_ros_publisher<ROS_ZEROCOPY_IMAGE8M>(rectify_topic_name,
                                                           10);
          }
        } else {
          RS_SPDLOG_WARN("Disable AC1 Image Rgb ZeroCopy Send By ROS2 !");
        }
        if (enable_pointcloud_send) {
          publisher_depth_loan =
              create_ros_publisher<robosense_msgs::msg::RsPointCloud1M>(
                  pointcloud_topic_name, 10);
        } else {
          RS_SPDLOG_WARN("Disable PointCloud ZeroCopy Send By ROS2 !");
        }
      } else if (lidar_type == robosense::lidar::LidarType::RS_AC2) {
        if (enable_ac2_left_image_send) {
          publisher_rgb_left_loan =
              create_ros_publisher<ROS_ZEROCOPY_IMAGE4M>(left_topic_name, 10);
          if (enable_rectify) {
            publisher_rgb_rectify_left_loan =
                create_ros_publisher<ROS_ZEROCOPY_IMAGE4M>(
                    rectify_left_topic_name, 10);
          }
        } else {
          RS_SPDLOG_WARN("Disable AC2 Left Image Rgb ZeroCopy Send By ROS2 !");
        }
        if (enable_ac2_right_image_send) {
          publisher_rgb_right_loan =
              create_ros_publisher<ROS_ZEROCOPY_IMAGE4M>(right_topic_name, 10);
          if (enable_rectify) {
            publisher_rgb_rectify_right_loan =
                create_ros_publisher<ROS_ZEROCOPY_IMAGE4M>(
                    rectify_right_topic_name, 10);
          }
        } else {
          RS_SPDLOG_WARN("Disable AC2 Right Image Rgb ZeroCopy Send By ROS2 !");
        }
        if (enable_pointcloud_send) {
          publisher_depth_ac2_loan =
              create_ros_publisher<ROS_ZEROCOPY_POINTCLOUD4M>(
                  pointcloud_topic_name, 10);
          if (enable_ac2_pointcloud_wave_split) {
            publisher_depth_ac2_wave2_loan =
                create_ros_publisher<ROS_ZEROCOPY_POINTCLOUD4M>(
                    pointcloud_ac2_wave2_topic_name, 10);
          }
        } else {
          RS_SPDLOG_WARN("Disable PointCloud ZeroCopy Send By ROS2 !");
        }
      }
    }
#endif // defined(ROS2_FOUND)
    if (enable_imu_send) {
      publisher_imu = create_ros_publisher<ROS_IMU>(imu_topic_name, 10);
    } else {
      RS_SPDLOG_WARN("Disable Imu Send By ROS2 !");
    }
    if (enable_jpeg) {
      if (lidar_type == robosense::lidar::LidarType::RS_AC1) {
        if (enable_ac1_image_send) {
          publisher_jpeg =
              create_ros_publisher<ROS_COMPRESSED_IMAGE>(jpeg_topic_name, 10);
        } else {
          RS_SPDLOG_WARN("Disable AC1 Image Jpeg Send By ROS2 !");
        }
      } else if (lidar_type == robosense::lidar::LidarType::RS_AC2) {
        if (enable_ac2_left_image_send) {
          publisher_jpeg_left = create_ros_publisher<ROS_COMPRESSED_IMAGE>(
              jpeg_left_topic_name, 10);
        } else {
          RS_SPDLOG_WARN("Disable AC2 Left Image Jpeg Send By ROS2 !");
        }
        if (enable_ac2_right_image_send) {
          publisher_jpeg_right = create_ros_publisher<ROS_COMPRESSED_IMAGE>(
              jpeg_right_topic_name, 10);
        } else {
          RS_SPDLOG_WARN("Disable AC2 Right Image Jpeg Send By ROS2 !");
        }
      }
    }
    if (enable_rectify_jpeg) {
      if (lidar_type == robosense::lidar::LidarType::RS_AC1) {
        if (enable_ac1_image_send) {
          publisher_jpeg_rectify = create_ros_publisher<ROS_COMPRESSED_IMAGE>(
              jpeg_rectify_topic_name, 10);
        } else {
          RS_SPDLOG_WARN("Disable AC1 Rectify Image Jpeg Send By ROS2 !");
        }
      } else if (lidar_type == robosense::lidar::LidarType::RS_AC2) {
        if (enable_ac2_left_image_send) {
          publisher_jpeg_rectify_left =
              create_ros_publisher<ROS_COMPRESSED_IMAGE>(
                  jpeg_rectify_left_topic_name, 10);
        } else {
          RS_SPDLOG_WARN("Disable AC2 Rectify Left Image Jpeg Send By ROS2 !");
        }
        if (enable_ac2_right_image_send) {
          publisher_jpeg_rectify_right =
              create_ros_publisher<ROS_COMPRESSED_IMAGE>(
                  jpeg_rectify_right_topic_name, 10);
        } else {
          RS_SPDLOG_WARN("Disable AC2 Rectify Right Image Jpeg Send By ROS2 !");
        }
      }
    }

    return 0;
  }

  int initImageBuffer() {
    // 更新图像分辨率信息和初始化缓冲区
    if (lidar_type == robosense::lidar::LidarType::RS_AC1) {
      image_width_rgb = image_width_ac1;
      image_height_rgb = image_height_ac1;
      image_width_driver = image_width_ac1;
      image_height_driver = image_height_ac1;
    } else if (lidar_type == robosense::lidar::LidarType::RS_AC2) {
      image_width_rgb = image_width_ac2_rgb;
      image_height_rgb = image_height_ac2_rgb;
      if (device_interface_type ==
          robosense::device::DeviceInterfaceType::DEVICE_INTERFACE_USB) {
        if (enable_angle_and_device_calib_info_from_device) {
          image_width_driver = image_usb_with_angle_calib_width_ac2_driver;
          image_height_driver = image_usb_with_angle_calib_height_ac2_driver;
        } else {
          image_width_driver = image_usb_width_ac2_driver;
          image_height_driver = image_usb_height_ac2_driver;
        }
      } else if (device_interface_type ==
                 robosense::device::DeviceInterfaceType::
                     DEVICE_INTERFACE_GMSL) {
        image_width_driver = image_gmsl_width_ac2_driver;
        image_height_driver = image_gmsl_height_ac2_driver;
      }
    }
    // Non-Crop Case
    nv12_image_size = robosense::color::ColorCodec::NV12ImageSize(
        image_width_rgb, image_height_rgb);
    rgb_image_size = robosense::color::ColorCodec::RGBImageSize(
        image_width_rgb, image_height_rgb);

    if (lidar_type == robosense::lidar::LidarType::RS_AC1) {
      image_crop_width_rgb = image_width_rgb - ac1_crop_config.getCropWidth();
      image_crop_height_rgb =
          image_height_rgb - ac1_crop_config.getCropHeight();
      rgb_crop_image_size = robosense::color::ColorCodec::RGBImageSize(
          image_crop_width_rgb, image_crop_height_rgb);
      rgb_buf.resize(rgb_image_size, 0);
      crop_rgb_buf.resize(rgb_image_size, 0);
    } else if (lidar_type == robosense::lidar::LidarType::RS_AC2) {
      if (ac2_left_crop_config.getCropWidth() !=
              ac2_right_crop_config.getCropWidth() ||
          ac2_left_crop_config.getCropHeight() !=
              ac2_right_crop_config.getCropHeight()) {
        RS_SPDLOG_ERROR(
            "AC2 Left Image Crop Setting Not Match AC2 Right Image Crop "
            "Setting !");
        return -1;
      }
      // left
      image_left_crop_width_rgb =
          image_width_rgb - ac2_left_crop_config.getCropWidth();
      image_left_crop_height_rgb =
          image_height_rgb - ac2_left_crop_config.getCropHeight();
      rgb_left_crop_image_size = robosense::color::ColorCodec::RGBImageSize(
          image_left_crop_width_rgb, image_left_crop_height_rgb);
      rgb_left_buf.resize(rgb_image_size, 0);
      crop_rgb_left_buf.resize(rgb_image_size, 0);

      // right
      image_right_crop_width_rgb =
          image_width_rgb - ac2_right_crop_config.getCropWidth();
      image_right_crop_height_rgb =
          image_height_rgb - ac2_right_crop_config.getCropHeight();
      rgb_right_crop_image_size = robosense::color::ColorCodec::RGBImageSize(
          image_right_crop_width_rgb, image_left_crop_height_rgb);
      rgb_right_buf.resize(rgb_image_size, 0);
      crop_rgb_right_buf.resize(rgb_image_size, 0);
    }

    return 0;
  }

  int initRgbCodec() {
    int ret = 0;
    if (lidar_type == robosense::lidar::LidarType::RS_AC1) {
      try {
        rgb_codec_ptr.reset(new robosense::color::ColorCodec());
      } catch (...) {
        RS_SPDLOG_ERROR("Malloc AC1 Codec(s) Failed !");
        return -1;
      }
      ret = rgb_codec_ptr->init(image_width_rgb, image_height_rgb);
      if (ret != 0) {
        RS_SPDLOG_ERROR("AC1 Codec(s) Initial Failed: ret = " +
                        std::to_string(ret));
        return -2;
      }
    } else if (lidar_type == robosense::lidar::LidarType::RS_AC2) {
      // Left
      try {
        rgb_left_codec_ptr.reset(new robosense::color::ColorCodec());
      } catch (...) {
        RS_SPDLOG_ERROR("Malloc AC2 Left Codec(s) Failed !");
        return -3;
      }
      ret = rgb_left_codec_ptr->init(image_width_rgb, image_height_rgb);
      if (ret != 0) {
        RS_SPDLOG_ERROR("AC2 Codec(s) Left Initial Failed: ret = " +
                        std::to_string(ret));
        return -4;
      }

      // Right
      try {
        rgb_right_codec_ptr.reset(new robosense::color::ColorCodec());
      } catch (...) {
        RS_SPDLOG_ERROR("Malloc AC2 Right Codec(s) Failed !");
        return -5;
      }
      ret = rgb_right_codec_ptr->init(image_width_rgb, image_height_rgb);
      if (ret != 0) {
        RS_SPDLOG_ERROR("AC2 Codec(s) Right Initial Failed: ret = " +
                        std::to_string(ret));
        return -6;
      }
    }
    return 0;
  }

  int stopRgbCodec() {
    if (lidar_type == robosense::lidar::LidarType::RS_AC1) {
      rgb_codec_ptr.reset();
    } else if (lidar_type == robosense::lidar::LidarType::RS_AC2) {
      rgb_left_codec_ptr.reset();
      rgb_right_codec_ptr.reset();
    }
    return 0;
  }

  int initJpegEncoder() {
    int ret = 0;
    if (enable_jpeg) {
      robosense::jpeg::JpegCodesConfig config;
      config.coderType = robosense::jpeg::JPEG_CODER_TYPE::RS_JPEG_CODER_ENCODE;
      config.jpegQuality = jpeg_quality;
      config.gpuDeviceId = 0;
      config.imageFrameFormat = robosense::common::FRAME_FORMAT_RGB24;

      if (lidar_type == robosense::lidar::LidarType::RS_AC1) {
        config.imageWidth = image_crop_width_rgb;
        config.imageHeight = image_crop_height_rgb;
        // AC1
        try {
          jpeg_encoder_ptr.reset(new robosense::jpeg::JpegCoder());
        } catch (...) {
          RS_SPDLOG_ERROR("Malloc AC1 Jpeg Encoder Failed !");
          return -1;
        }

        ret = jpeg_encoder_ptr->init(config);
        if (ret != 0) {
          RS_SPDLOG_ERROR("Initial AC1 Jpeg Encoder Failed: ret = " +
                          std::to_string(ret));
          return -2;
        }
      } else if (lidar_type == robosense::lidar::LidarType::RS_AC2) {
        // Left
        config.imageWidth = image_left_crop_width_rgb;
        config.imageHeight = image_left_crop_height_rgb;
        try {
          jpeg_left_encoder_ptr.reset(new robosense::jpeg::JpegCoder());
        } catch (...) {
          RS_SPDLOG_ERROR("Malloc AC2 Jpeg Left Encoder Failed !");
          return -3;
        }

        ret = jpeg_left_encoder_ptr->init(config);
        if (ret != 0) {
          RS_SPDLOG_ERROR("Initial AC2 Jpeg Left Encoder Failed: ret = " +
                          std::to_string(ret));
          return -4;
        }

        // Right
        config.imageWidth = image_right_crop_width_rgb;
        config.imageHeight = image_right_crop_height_rgb;
        try {
          jpeg_right_encoder_ptr.reset(new robosense::jpeg::JpegCoder());
        } catch (...) {
          RS_SPDLOG_ERROR("Malloc AC2 Jpeg Right Encoder Failed !");
          return -5;
        }

        ret = jpeg_right_encoder_ptr->init(config);
        if (ret != 0) {
          RS_SPDLOG_ERROR("Initial AC2 Jpeg Right Encoder Failed: ret = " +
                          std::to_string(ret));
          return -6;
        }
      }
      RS_SPDLOG_INFO("Enable Jpeg: Create Jpeg Encoder(s) Successed !");
    } else {
      RS_SPDLOG_INFO("Disable Jpeg: Not Need Create Jpeg Encoder(s) !");
    }

    if (enable_rectify_jpeg) {
      robosense::jpeg::JpegCodesConfig config;
      config.coderType = robosense::jpeg::JPEG_CODER_TYPE::RS_JPEG_CODER_ENCODE;
      config.jpegQuality = jpeg_quality;
      config.gpuDeviceId = 0;
      config.imageFrameFormat = robosense::common::FRAME_FORMAT_RGB24;

      if (lidar_type == robosense::lidar::LidarType::RS_AC1) {
        config.imageWidth = image_crop_width_rgb;
        config.imageHeight = image_crop_height_rgb;
        // AC1
        try {
          jpeg_rectify_encoder_ptr.reset(new robosense::jpeg::JpegCoder());
        } catch (...) {
          RS_SPDLOG_ERROR("Malloc AC1 Rectify Jpeg Encoder Failed !");
          return -1;
        }

        ret = jpeg_rectify_encoder_ptr->init(config);
        if (ret != 0) {
          RS_SPDLOG_ERROR("Initial AC1 Rectify Jpeg Encoder Failed: ret = " +
                          std::to_string(ret));
          return -2;
        }
      } else if (lidar_type == robosense::lidar::LidarType::RS_AC2) {
        // Left
        config.imageWidth = image_left_crop_width_rgb;
        config.imageHeight = image_left_crop_height_rgb;
        try {
          jpeg_rectify_left_encoder_ptr.reset(new robosense::jpeg::JpegCoder());
        } catch (...) {
          RS_SPDLOG_ERROR("Malloc AC2 Rectify Jpeg Left Encoder Failed !");
          return -3;
        }

        ret = jpeg_rectify_left_encoder_ptr->init(config);
        if (ret != 0) {
          RS_SPDLOG_ERROR(
              "Initial AC2 Rectify Jpeg Left Encoder Failed: ret = " +
              std::to_string(ret));
          return -4;
        }

        // Right
        config.imageWidth = image_right_crop_width_rgb;
        config.imageHeight = image_right_crop_height_rgb;
        try {
          jpeg_rectify_right_encoder_ptr.reset(
              new robosense::jpeg::JpegCoder());
        } catch (...) {
          RS_SPDLOG_ERROR("Malloc AC2 Rectify Jpeg Right Encoder Failed !");
          return -5;
        }

        ret = jpeg_rectify_right_encoder_ptr->init(config);
        if (ret != 0) {
          RS_SPDLOG_ERROR(
              "Initial AC2 Rectify Jpeg Right Encoder Failed: ret = " +
              std::to_string(ret));
          return -6;
        }
      }
      RS_SPDLOG_INFO(
          "Enable Rectify Jpeg: Create Rectify Jpeg Encoder(s) Successed !");
    } else {
      RS_SPDLOG_INFO(
          "Disable Rectify Jpeg: Not Need Create Rectify Jpeg Encoder(s) !");
    }

    return 0;
  }

  int stopJpegEncoder() {
    if (enable_jpeg) {
      if (lidar_type == robosense::lidar::LidarType::RS_AC1) {
        jpeg_encoder_ptr.reset();
      } else if (lidar_type == robosense::lidar::LidarType::RS_AC2) {
        jpeg_left_encoder_ptr.reset();
        jpeg_right_encoder_ptr.reset();
      }
      RS_SPDLOG_INFO("Enable Jpeg: Stop Jpeg Encoder(s) Successed !");
    } else {
      RS_SPDLOG_INFO("Disable Jpeg: Not Need Stop Jpeg Encoder(s) !");
    }

    if (enable_rectify_jpeg) {
      if (lidar_type == robosense::lidar::LidarType::RS_AC1) {
        jpeg_rectify_encoder_ptr.reset();
      } else if (lidar_type == robosense::lidar::LidarType::RS_AC2) {
        jpeg_rectify_left_encoder_ptr.reset();
        jpeg_rectify_right_encoder_ptr.reset();
      }
      RS_SPDLOG_INFO(
          "Enable Rectify Jpeg: Stop Rectify Jpeg Encoder(s) Successed !");
    } else {
      RS_SPDLOG_INFO(
          "Disable Rectify Jpeg: Not Need Stop Rectify Jpeg Encoder(s) !");
    }

    return 0;
  }

  int initRgbWorkThreads() {
    if (lidar_type == robosense::lidar::LidarType::RS_AC1) {
      try {
        is_rgb_running_ = true;
        rgb_thread_ptr.reset(
            new std::thread(&MSPublisher::rgbProcessWorkThread, this));
      } catch (...) {
        is_rgb_running_ = false;
        RS_SPDLOG_ERROR("Malloc AC1 Rgb Work Thread(s) Failed !");
        return -1;
      }
    } else if (lidar_type == robosense::lidar::LidarType::RS_AC2) {
      try {
        is_rgb_left_running_ = true;
        rgb_left_thread_ptr.reset(
            new std::thread(&MSPublisher::rgbLeftProcessWorkThread, this));
      } catch (...) {
        is_rgb_left_running_ = false;
        RS_SPDLOG_ERROR("Malloc AC2 Rgb Left Work Thread(s) Failed !");
        return -2;
      }

      try {
        is_rgb_right_running_ = true;
        rgb_right_thread_ptr.reset(
            new std::thread(&MSPublisher::rgbRightProcessWorkThread, this));
      } catch (...) {
        is_rgb_right_running_ = false;
        RS_SPDLOG_ERROR("Malloc AC2 Rgb Right Work Thread(s) Failed !");
        return -3;
      }
    }
    RS_SPDLOG_INFO("Enable Rgb: Create Rgb Work Thread(s) Successed !");

    if (enable_rectify) {
      if (lidar_type == robosense::lidar::LidarType::RS_AC1) {
        try {
          is_rgb_rectify_running_ = true;
          rgb_rectify_thread_ptr.reset(
              new std::thread(&MSPublisher::rgbRectifyProcessWorkThread, this));
        } catch (...) {
          is_rgb_rectify_running_ = false;
          RS_SPDLOG_ERROR("Malloc AC1 Rgb Rectify Work Thread(s) Failed !");
          return -1;
        }
      } else if (lidar_type == robosense::lidar::LidarType::RS_AC2) {
        try {
          is_rgb_rectify_left_running_ = true;
          rgb_rectify_left_thread_ptr.reset(new std::thread(
              &MSPublisher::rgbRectifyLeftProcessWorkThread, this));
        } catch (...) {
          is_rgb_rectify_left_running_ = false;
          RS_SPDLOG_ERROR(
              "Malloc AC2 Rgb Rectify Left Work Thread(s) Failed !");
          return -2;
        }

        try {
          is_rgb_rectify_right_running_ = true;
          rgb_rectify_right_thread_ptr.reset(new std::thread(
              &MSPublisher::rgbRectifyRightProcessWorkThread, this));
        } catch (...) {
          is_rgb_rectify_right_running_ = false;
          RS_SPDLOG_ERROR(
              "Malloc AC2 Rgb Rectify Right Work Thread(s) Failed !");
          return -3;
        }
      }
      RS_SPDLOG_INFO(
          "Enable Rgb Rectify: Create Rgb Rectify Work Thread(s) Successed !");
    } else {
      RS_SPDLOG_INFO(
          "Dsiable Rgb Rectify: Not Create Rectify Work Thread(s) !");
    }

    return 0;
  }

  int stopRgbWorkThreads() {
    if (lidar_type == robosense::lidar::LidarType::RS_AC1) {
      // ac1 rgb
      if (is_rgb_running_) {
        {
          std::lock_guard<std::mutex> lock(rgb_mutex_);
          is_rgb_running_ = false;
          rgb_condition_.notify_all();
        }
        if (rgb_thread_ptr && rgb_thread_ptr->joinable()) {
          rgb_thread_ptr->join();
        }
        rgb_thread_ptr.reset();
      }
      RS_SPDLOG_INFO("Stop AC1 Rgb Work Thread(s) Successed !");
    } else if (lidar_type == robosense::lidar::LidarType::RS_AC2) {
      // left rgb
      if (is_rgb_left_running_) {
        {
          std::lock_guard<std::mutex> lock(rgb_left_mutex_);
          is_rgb_left_running_ = false;
          rgb_left_condition_.notify_all();
        }
        if (rgb_left_thread_ptr && rgb_left_thread_ptr->joinable()) {
          rgb_left_thread_ptr->join();
        }
        rgb_left_thread_ptr.reset();
      }

      // right rgb
      if (is_rgb_right_running_) {
        {
          std::lock_guard<std::mutex> lock(rgb_right_mutex_);
          is_rgb_right_running_ = false;
          rgb_right_condition_.notify_all();
        }
        if (rgb_right_thread_ptr && rgb_right_thread_ptr->joinable()) {
          rgb_right_thread_ptr->join();
        }
        rgb_right_thread_ptr.reset();
      }
      RS_SPDLOG_INFO("Stop AC2 Rgb Work Thread(s) Successed !");
    }

    if (enable_rectify) {
      if (lidar_type == robosense::lidar::LidarType::RS_AC1) {
        // ac1 rgb
        if (is_rgb_rectify_running_) {
          {
            std::lock_guard<std::mutex> lock(rgb_rectify_mutex_);
            is_rgb_rectify_running_ = false;
            rgb_rectify_condition_.notify_all();
          }
          if (rgb_rectify_thread_ptr && rgb_rectify_thread_ptr->joinable()) {
            rgb_rectify_thread_ptr->join();
          }
          rgb_rectify_thread_ptr.reset();
        }
        RS_SPDLOG_INFO("Stop AC1 Rgb Rectify Work Thread(s) Successed !");
      } else if (lidar_type == robosense::lidar::LidarType::RS_AC2) {
        // left rgb
        if (is_rgb_rectify_left_running_) {
          {
            std::lock_guard<std::mutex> lock(rgb_rectify_left_mutex_);
            is_rgb_rectify_left_running_ = false;
            rgb_rectify_left_condition_.notify_all();
          }
          if (rgb_rectify_left_thread_ptr &&
              rgb_rectify_left_thread_ptr->joinable()) {
            rgb_rectify_left_thread_ptr->join();
          }
          rgb_rectify_left_thread_ptr.reset();
        }

        // right rgb
        if (is_rgb_rectify_right_running_) {
          {
            std::lock_guard<std::mutex> lock(rgb_rectify_right_mutex_);
            is_rgb_rectify_right_running_ = false;
            rgb_rectify_right_condition_.notify_all();
          }
          if (rgb_rectify_right_thread_ptr &&
              rgb_rectify_right_thread_ptr->joinable()) {
            rgb_rectify_right_thread_ptr->join();
          }
          rgb_rectify_right_thread_ptr.reset();
        }
        RS_SPDLOG_INFO(
            "Enable Rgb Rectify: Stop AC2 Rgb Rectify Work Thread(s) "
            "Successed !");
      }
    } else {
      RS_SPDLOG_INFO("Disable Rgb Rectify: Not Nees Stop AC2 Rgb Rectify Work "
                     "Thread(s) !");
    }

    return 0;
  }

  int initDeviceCalibInfoWorkThread() {
    try {
      is_device_info_running_ = true;
      device_info_thread_ptr.reset(
          new std::thread(&MSPublisher::deviceInfoProcessWorkThread, this));
    } catch (...) {
      is_device_info_running_ = false;
      RS_SPDLOG_ERROR("Create Device Calibration Publish Work Thread Failed !");
      return -1;
    }
    return 0;
  }

  int stopDeviceCalibInfoWorkThread() {
    if (is_device_info_running_) {
      is_device_info_running_ = false;
    }
    if (device_info_thread_ptr && device_info_thread_ptr->joinable()) {
      device_info_thread_ptr->join();
    }
    device_info_thread_ptr.reset();

    return 0;
  }

  int initJpegWorkThreads() {
    if (enable_jpeg) {
      if (lidar_type == robosense::lidar::LidarType::RS_AC1) {
        try {
          is_jpeg_running_ = true;
          jpeg_thread_ptr.reset(
              new std::thread(&MSPublisher::jpegProcessWorkThread, this));
        } catch (...) {
          is_jpeg_running_ = false;
          RS_SPDLOG_ERROR("Malloc AC1 Jpeg Work Thread Failed !");
          return -1;
        }
      } else if (lidar_type == robosense::lidar::LidarType::RS_AC2) {
        try {
          is_jpeg_left_running_ = true;
          jpeg_left_thread_ptr.reset(
              new std::thread(&MSPublisher::jpegLeftProcessWorkThread, this));
        } catch (...) {
          is_jpeg_left_running_ = false;
          RS_SPDLOG_ERROR("Malloc AC2 Jpeg Left Work Thread Failed !");
          return -2;
        }

        try {
          is_jpeg_right_running_ = true;
          jpeg_right_thread_ptr.reset(
              new std::thread(&MSPublisher::jpegRightProcessWorkThread, this));
        } catch (...) {
          is_jpeg_right_running_ = false;
          RS_SPDLOG_ERROR("Malloc AC2 Jpeg Right Work Thread Failed !");
          return -3;
        }
      }
      RS_SPDLOG_INFO("Enable Jpeg: Create Jpeg Work Thread(s) Successed !");
    } else {
      RS_SPDLOG_INFO("Disable Jpeg: Not Need Create Jpeg Work Thread(s) !");
    }

    if (enable_rectify_jpeg) {
      if (lidar_type == robosense::lidar::LidarType::RS_AC1) {
        try {
          is_jpeg_rectify_running_ = true;
          jpeg_rectify_thread_ptr.reset(new std::thread(
              &MSPublisher::jpegRectifyProcessWorkThread, this));
        } catch (...) {
          is_jpeg_rectify_running_ = false;
          RS_SPDLOG_ERROR("Malloc AC1 Rectify Jpeg Work Thread Failed !");
          return -1;
        }
      } else if (lidar_type == robosense::lidar::LidarType::RS_AC2) {
        try {
          is_jpeg_rectify_left_running_ = true;
          jpeg_rectify_left_thread_ptr.reset(new std::thread(
              &MSPublisher::jpegRectifyLeftProcessWorkThread, this));
        } catch (...) {
          is_jpeg_rectify_left_running_ = false;
          RS_SPDLOG_ERROR("Malloc AC2 Rectify Jpeg Left Work Thread Failed !");
          return -2;
        }

        try {
          is_jpeg_rectify_right_running_ = true;
          jpeg_rectify_right_thread_ptr.reset(new std::thread(
              &MSPublisher::jpegRectifyRightProcessWorkThread, this));
        } catch (...) {
          is_jpeg_rectify_right_running_ = false;
          RS_SPDLOG_ERROR("Malloc AC2 Rectify Jpeg Right Work Thread Failed !");
          return -3;
        }
      }
      RS_SPDLOG_INFO("Enable Rectify Jpeg: Create Rectify Jpeg Work Thread(s) "
                     "Successed !");
    } else {
      RS_SPDLOG_INFO("Disable Rectify Jpeg: Not Need Create Rectify Jpeg Work "
                     "Thread(s) !");
    }

    return 0;
  }

  int stopJpegWorkThreads() {
    if (enable_jpeg) {
      if (lidar_type == robosense::lidar::LidarType::RS_AC1) {
        // ac1 jpeg
        {
          std::lock_guard<std::mutex> lock(jpeg_mutex_);
          is_jpeg_running_ = false;
          jpeg_condition_.notify_all();
        }
        if (jpeg_thread_ptr && jpeg_thread_ptr->joinable()) {
          jpeg_thread_ptr->join();
        }
        jpeg_thread_ptr.reset();
        RS_SPDLOG_INFO("Stop AC1 Jpeg Work Thread(s) Successed !");
      } else if (lidar_type == robosense::lidar::LidarType::RS_AC2) {
        // left jpeg
        {
          std::lock_guard<std::mutex> lock(jpeg_left_mutex_);
          is_jpeg_left_running_ = false;
          jpeg_left_condition_.notify_all();
        }
        if (jpeg_left_thread_ptr && jpeg_left_thread_ptr->joinable()) {
          jpeg_left_thread_ptr->join();
        }
        jpeg_left_thread_ptr.reset();

        // right jpeg
        {
          std::lock_guard<std::mutex> lg(jpeg_right_mutex_);
          is_jpeg_right_running_ = false;
          jpeg_right_condition_.notify_all();
        }
        if (jpeg_right_thread_ptr && jpeg_right_thread_ptr->joinable()) {
          jpeg_right_thread_ptr->join();
        }
        jpeg_right_thread_ptr.reset();
      }
      RS_SPDLOG_INFO("Stop AC2 Jpeg Work Thread(s) Successed !");
    } else {
      RS_SPDLOG_INFO("Disable Jpeg: Not Need Stop Jpeg Work Thread(s) !");
    }

    if (enable_rectify_jpeg) {
      if (lidar_type == robosense::lidar::LidarType::RS_AC1) {
        // ac1 jpeg
        {
          std::lock_guard<std::mutex> lock(jpeg_rectify_mutex_);
          is_jpeg_rectify_running_ = false;
          jpeg_rectify_condition_.notify_all();
        }
        if (jpeg_rectify_thread_ptr && jpeg_rectify_thread_ptr->joinable()) {
          jpeg_rectify_thread_ptr->join();
        }
        jpeg_rectify_thread_ptr.reset();
        RS_SPDLOG_INFO("Stop AC1 Rectify Jpeg Work Thread(s) Successed !");
      } else if (lidar_type == robosense::lidar::LidarType::RS_AC2) {
        // left jpeg
        {
          std::lock_guard<std::mutex> lock(jpeg_rectify_left_mutex_);
          is_jpeg_rectify_left_running_ = false;
          jpeg_rectify_left_condition_.notify_all();
        }
        if (jpeg_rectify_left_thread_ptr &&
            jpeg_rectify_left_thread_ptr->joinable()) {
          jpeg_rectify_left_thread_ptr->join();
        }
        jpeg_rectify_left_thread_ptr.reset();

        // right jpeg
        {
          std::lock_guard<std::mutex> lg(jpeg_rectify_right_mutex_);
          is_jpeg_rectify_right_running_ = false;
          jpeg_rectify_right_condition_.notify_all();
        }
        if (jpeg_rectify_right_thread_ptr &&
            jpeg_rectify_right_thread_ptr->joinable()) {
          jpeg_rectify_right_thread_ptr->join();
        }
        jpeg_rectify_right_thread_ptr.reset();
      }
      RS_SPDLOG_INFO("Stop AC2 Rectify Jpeg Work Thread(s) Successed !");
    } else {
      RS_SPDLOG_INFO(
          "Disable Rectify Jpeg: Not Need Stop Rectify Jpeg Work Thread(s) !");
    }

    return 0;
  }

  void checkInputFrequence() {
    int new_image_input_fps = 30;
    int new_imu_input_fps = 200;
    if (lidar_type == robosense::lidar::LidarType::RS_AC1) {
      if (image_input_fps != 30 && image_input_fps != 15 &&
          image_input_fps != 10) {
        uint32_t diff_30 = std::abs(image_input_fps - 30);
        uint32_t diff_15 = std::abs(image_input_fps - 15);
        uint32_t diff_10 = std::abs(image_input_fps - 10);

        uint32_t min_diff = diff_30;

        if (diff_15 < min_diff) {
          new_image_input_fps = 15;
          min_diff = diff_15;
        }
        if (diff_10 < min_diff) {
          new_image_input_fps = 10;
          min_diff = diff_10;
        }
        image_input_fps = new_image_input_fps;
      }
      if (imu_input_fps != 100 && imu_input_fps != 200) {
        new_imu_input_fps =
            std::abs(imu_input_fps - 100) < std::abs(imu_input_fps - 200) ? 100
                                                                          : 200;
      }
      if (new_image_input_fps != image_input_fps) {
        RS_SPDLOG_WARN("AC1 Image Input Hz Force From: " +
                       std::to_string(image_input_fps) + " To " +
                       std::to_string(new_image_input_fps));
        image_input_fps = new_image_input_fps;
      }
      if (new_imu_input_fps != imu_input_fps) {
        RS_SPDLOG_WARN(
            "AC1 Imu Input Hz Force From: " + std::to_string(imu_input_fps) +
            " To " + std::to_string(new_imu_input_fps));
        imu_input_fps = new_imu_input_fps;
      }
    } else if (lidar_type == robosense::lidar::LidarType::RS_AC2) {
      uint32_t diff_15 = std::abs(image_input_fps - 15);
      uint32_t diff_10 = std::abs(image_input_fps - 10);

      new_image_input_fps = 15;
      uint32_t min_diff = diff_15;
      if (diff_10 < min_diff) {
        new_image_input_fps = 10;
        min_diff = diff_10;
      }

      if (new_image_input_fps != image_input_fps) {
        RS_SPDLOG_WARN("AC2 Image Input Hz Force From: " +
                       std::to_string(image_input_fps) + " To " +
                       std::to_string(new_image_input_fps));
        image_input_fps = new_image_input_fps;
      }

      new_imu_input_fps = 200;
      if (new_imu_input_fps != imu_input_fps) {
        imu_input_fps = new_imu_input_fps;
        RS_SPDLOG_WARN(
            "AC2 Imu Input Hz Force From: " + std::to_string(imu_input_fps) +
            " To " + std::to_string(new_imu_input_fps));
      }
    }
  }

  robosense::device::RSDeviceOpenConfig
  makeDeviceOpenConfig(const std::string &uuid,
                       const robosense::lidar::LidarType lidar_type) {
    robosense::device::RSDeviceOpenConfig deviceOpenConfig;
    deviceOpenConfig.device_uuid = uuid;
    deviceOpenConfig.device_path = uuid;
    deviceOpenConfig.image_width = image_width_driver;
    deviceOpenConfig.image_height = image_height_driver;

    deviceOpenConfig.image_input_fps = image_input_fps;
    deviceOpenConfig.imu_input_fps = imu_input_fps;
    deviceOpenConfig.input_type =
        (device_interface_type ==
                 robosense::device::DeviceInterfaceType::DEVICE_INTERFACE_USB
             ? robosense::lidar::InputType::USB
             : robosense::lidar::InputType::GMSL);
    deviceOpenConfig.lidar_type = lidar_type;
    deviceOpenConfig.enable_use_lidar_clock = enable_use_lidar_clock;
    deviceOpenConfig.enable_use_dense_points = enable_use_dense_points;
    deviceOpenConfig.enable_use_first_point_ts = enable_use_first_point_ts;

    // 图像颜色格式
    if (lidar_type == robosense::lidar::LidarType::RS_AC1) {
#if defined(RK3588) || defined(JETSON_ORIN)
      deviceOpenConfig.image_format =
          robosense::lidar::frame_format::FRAME_FORMAT_NV12;
#else
      deviceOpenConfig.image_format =
          robosense::lidar::frame_format::FRAME_FORMAT_RGB24;
#endif // define(RK3588_JETSON_ORIN)
    } else if (lidar_type == robosense::lidar::LidarType::RS_AC2) {
      deviceOpenConfig.image_format =
          (device_interface_type == robosense::device::DEVICE_INTERFACE_USB
               ? robosense::lidar::frame_format::FRAME_FORMAT_XR24
               : robosense::lidar::frame_format::FRAME_FORMAT_GREY);
      deviceOpenConfig.angle_calib_basic_dir_path = angle_calib_basic_dir_path;
    }

    // 设置数据输出
    deviceOpenConfig.enable_pointcloud_send = enable_pointcloud_send;
    deviceOpenConfig.enable_ac1_image_send = enable_ac1_image_send;
    deviceOpenConfig.enable_ac2_left_image_send = enable_ac2_left_image_send;
    deviceOpenConfig.enable_ac2_right_image_send = enable_ac2_right_image_send;
    deviceOpenConfig.enable_imu_send = enable_imu_send;
    deviceOpenConfig.timestamp_compensate_s = timestamp_compensate_s;

    // AC2 Algorithm Param(s)
#if defined(ENABLE_SUPPORT_RS_DRIVER_ALGORITHM)
    deviceOpenConfig.algorithm_param = algorithm_param;
#endif // defined(ENABLE_SUPPORT_RS_DRIVER_ALGORITHM)

    return deviceOpenConfig;
  }

  int openDevice(const std::string &uuid) {
    int ret = 0;
    // Initial Timestamp Manager
    if (!timestamp_output_dir_path.empty()) {
      ret = initTimestampManager();
      if (ret != 0) {
        RS_SPDLOG_ERROR("Initial Timestamp Manager Failed: ret = " +
                        std::to_string(ret));
        return -1;
      }
    }

    // Initial publishers
    ret = initPublishers();
    if (ret != 0) {
      RS_SPDLOG_ERROR(
          "Found Device uuid = " + uuid +
          ", Initial Ros Publisher(s) Failed: ret = " + std::to_string(ret));
      return -2;
    }

    // Initial Image Buffer(s)
    ret = initImageBuffer();
    if (ret != 0) {
      RS_SPDLOG_ERROR(
          "Found Device uuid = " + uuid +
          ", Initial Image Buffer Failed: ret = " + std::to_string(ret));
      return -3;
    }

    // Initial Rgb Codec
    ret = initRgbCodec();
    if (ret != 0) {
      RS_SPDLOG_ERROR(
          "Found Device uuid = " + uuid +
          ", Initial Rgb Codec(s) Failed: ret = " + std::to_string(ret));
      return -4;
    }

    // Initial Rgb Thread(s)
    ret = initRgbWorkThreads();
    if (ret != 0) {
      RS_SPDLOG_ERROR(
          "Found Device uuid = " + uuid +
          ", Initial Rgb Work Thread(s) Failed: ret = " + std::to_string(ret));
      return -5;
    }

    // Initial Jpeg Encoder
    ret = initJpegEncoder();
    if (ret != 0) {
      RS_SPDLOG_ERROR(
          "Found Device uuid = " + uuid +
          ", Initial Jpeg Encoder(s) Failed: ret = " + std::to_string(ret));
      return -6;
    }

    // Initial Jpeg Thread(s)
    ret = initJpegWorkThreads();
    if (ret != 0) {
      RS_SPDLOG_ERROR(
          "Found Device uuid = " + uuid +
          ", Initial Jpeg Work Thread(s) Failed: ret = " + std::to_string(ret));
      return -7;
    }

    // 检查输入数据帧率
    checkInputFrequence();

    // 构造设备打开配置
    robosense::device::RSDeviceOpenConfig deviceOpenConfig =
        makeDeviceOpenConfig(uuid, lidar_type);

    ret = device_manager_ptr->openDevice(deviceOpenConfig);
    if (ret != 0) {
      RS_SPDLOG_ERROR("Device uuid = " + uuid +
                      " Open Device Failed: ret = " + std::to_string(ret));
      return -8;
    }

    {
      std::lock_guard<std::mutex> lg(current_device_uuid_mtx);
      current_device_uuid = uuid;
      RS_SPDLOG_INFO("Device uuid = " + uuid + " Open Successed !");
    }

    // Initial Device Calibration Info Thread
    ret = initDeviceCalibInfoWorkThread();
    if (ret != 0) {
      RS_SPDLOG_ERROR("Found Device uuid = " + uuid +
                      ", Initial Device Calib Info Thread Failed: ret = " +
                      std::to_string(ret));
      return -9;
    }

    return 0;
  }

  int closeDevice(const std::string &uuid) {
    int ret = 0;

    ret = device_manager_ptr->closeDevice(uuid, true);
    if (ret != 0) {
      RS_SPDLOG_ERROR("Device uuid = " + uuid +
                      " Detach Close Failed: ret = " + std::to_string(ret));
      return -1;
    }

    // 关闭Rgb Thread(s)
    ret = stopRgbWorkThreads();
    if (ret != 0) {
      RS_SPDLOG_ERROR("Stop Rgb Work Thread(s) Failed: ret = " +
                      std::to_string(ret));
      return -2;
    }

    // 关闭RgbCodec
    ret = stopRgbCodec();
    if (ret != 0) {
      RS_SPDLOG_ERROR("Stop Rgb Codec(s) Failed: ret = " + std::to_string(ret));
      return -3;
    }

    // 关闭Jpeg Thread(s)
    ret = stopJpegWorkThreads();
    if (ret != 0) {
      RS_SPDLOG_ERROR("Stop Jpeg Work Thread(s) Failed: ret = " +
                      std::to_string(ret));
      return -4;
    }

    // 关闭JpegEncoder
    ret = stopJpegEncoder();
    if (ret != 0) {
      RS_SPDLOG_ERROR("Stop Jpeg Encoder(s) Failed: ret = " +
                      std::to_string(ret));
      return -5;
    }

    // Initial Device Calibration Info Thread
    ret = stopDeviceCalibInfoWorkThread();
    if (ret != 0) {
      RS_SPDLOG_ERROR("Stop Device Calib Info Thread Failed: ret = " +
                      std::to_string(ret));
      return -6;
    }

    // 关闭Timestamp Manager
    if (!timestamp_output_dir_path.empty()) {
      stopTimestampManager();
    }

    {
      std::lock_guard<std::mutex> lg(current_device_uuid_mtx);
      current_device_uuid.clear();
      // device_info状态复位
      current_device_info_ready = false;
      current_device_info_valid = false;
      // camera_info信息复位
      camera_info_ptr.reset();
      left_camera_info_ptr.reset();
      right_camera_info_ptr.reset();
      // 去畸变的状态复位
      camera_rectify_map_valid = false;
      left_camera_rectify_map_valid = false;
      right_camera_rectify_map_valid = false;
      RS_SPDLOG_INFO("Device uuid = " + uuid + " Close Successed !");
    }

    return 0;
  }

  void deviceEventCallback(const robosense::device::DeviceEvent &deviceEvent) {
    int ret;
    switch (deviceEvent.event_type) {
    case robosense::device::DeviceEventType::DEVICE_EVENT_ATTACH: {
      const std::string &uuid =
          std::string(deviceEvent.uuid, deviceEvent.uuid_size);
      {
        std::lock_guard<std::mutex> lg(current_device_uuid_mtx);
        if (uuid == current_device_uuid) {
          RS_SPDLOG_INFO("Device uuid = " + uuid + " Already Open !");
          return;
        } else if (!current_device_uuid.empty()) {
          RS_SPDLOG_INFO(
              "Current Device uuid = " + current_device_uuid +
              " Already Open, Attach Device uuid = " + uuid +
              " Not Open: Because Of Not Support Open Multi-Device !");
          return;
        }
      }

      // 设备过滤启用时
      if (!serial_number.empty() && uuid != serial_number) {
        RS_SPDLOG_WARN("Current Find Device UUID: " + uuid +
                       " Not Setting serial_number: " + serial_number);
        return;
      }

      // 更新AC类型
      lidar_type = deviceEvent.lidar_type;

      // 打开设备
      ret = openDevice(uuid);
      if (ret != 0) {
        RS_SPDLOG_ERROR("Open Device uuid = " + uuid + ", lidar_type = " +
                        robosense::lidar::lidarTypeToStr(lidar_type));
        return;
      }

      break;
    }
    case robosense::device::DeviceEventType::DEVICE_EVENT_DETACH: {
      const std::string &uuid =
          std::string(deviceEvent.uuid, deviceEvent.uuid_size);
      {
        std::lock_guard<std::mutex> lg(current_device_uuid_mtx);
        if (uuid != current_device_uuid || current_device_uuid.empty()) {
          RS_SPDLOG_INFO("Device uuid = " + uuid +
                         " Detach But Not Need Processed !");
          return;
        }
      }

      // 关闭设备
      ret = closeDevice(uuid);
      if (ret != 0) {
        RS_SPDLOG_ERROR("Close Device uuid = " + uuid);
        return;
      }

      break;
    }
    default: {
      break;
    }
    }
  }

  void pointCloudCallback(
      const std::shared_ptr<PointCloudT<RsPointXYZIRT>> &msgPtr,
      const std::string &uuid,
      const robosense::device::RSTimestampItem::Ptr &timestampPtr) {
    (void)(uuid);
    if (msgPtr && enable_pointcloud_send) {
      depth_handle(msgPtr, timestampPtr);
    }
  }

  void
  imageCallback(const std::shared_ptr<robosense::lidar::ImageData> &msgPtr,
                const std::string &uuid,
                const robosense::device::RSTimestampItem::Ptr &timestampPtr) {
    (void)(uuid);
    if (msgPtr) {
      uint64_t mono_timestamp_ns = 0;
      uint64_t stereo_left_timestamp_ns = 0;
      uint64_t stereo_right_timestamp_ns = 0;
      if (msgPtr->camera_mode == robosense::lidar::CameraMode::MONO) {
        std::shared_ptr<robosense::lidar::MonoImageData> mono_ptr =
            std::dynamic_pointer_cast<robosense::lidar::MonoImageData>(msgPtr);
        if (!mono_ptr) {
          RS_SPDLOG_ERROR("Dynamic Convert To MonoImageData Is Nullptr !");
          return;
        }

        // 记录原始的timestamp
        mono_timestamp_ns = mono_ptr->timestamp * 1e9;
      } else if (msgPtr->camera_mode == robosense::lidar::CameraMode::STEREO) {
        std::shared_ptr<robosense::lidar::StereoImageData> stereo_ptr =
            std::dynamic_pointer_cast<robosense::lidar::StereoImageData>(
                msgPtr);
        if (!stereo_ptr) {
          RS_SPDLOG_ERROR("Dynamic Convert To StereoImageData Is Nullptr !");
          return;
        }

        // 记录原始的timestamp
        stereo_left_timestamp_ns = stereo_ptr->left_timestamp * 1e9;
        stereo_right_timestamp_ns = stereo_ptr->right_timestamp * 1e9;
      }

      if (lidar_type == robosense::lidar::LidarType::RS_AC1) {
        // AC1 相机
        if (enable_ac1_image_send) {
          // 更新timestamp日志中的timestamp_ns
          timestampPtr->timestamp_ns = mono_timestamp_ns;
          std::lock_guard<std::mutex> lock(rgb_mutex_);
          rgb_queue_.push({msgPtr, timestampPtr});
          rgb_condition_.notify_one();
        }

      } else if (lidar_type == robosense::lidar::LidarType::RS_AC2) {
        // AC2 相机
        // 左相机: rgb
        if (enable_ac2_left_image_send) {
          // 更新timestamp日志中的timestamp_ns
          timestampPtr->timestamp_ns = stereo_left_timestamp_ns;
          std::lock_guard<std::mutex> lock(rgb_left_mutex_);
          rgb_left_queue_.push({msgPtr, timestampPtr});
          rgb_left_condition_.notify_one();
        }
        // 右相机: rgb
        if (enable_ac2_right_image_send) {
          robosense::device::RSTimestampItem::Ptr rightItemstampPtr(
              new robosense::device::RSTimestampItem(*timestampPtr));
          // 更新timestamp日志中的timestamp_ns
          rightItemstampPtr->timestamp_ns = stereo_right_timestamp_ns;
          std::lock_guard<std::mutex> lock(rgb_right_mutex_);
          rgb_right_queue_.push({msgPtr, rightItemstampPtr});
          rgb_right_condition_.notify_one();
        }
      }
    }
  }

  void
  imuCallback(const std::shared_ptr<robosense::lidar::ImuData> &msgPtr,
              const std::string &uuid,
              const robosense::device::RSTimestampItem::Ptr &timestampPtr) {
    (void)(uuid);
    if (msgPtr && enable_imu_send) {
      imu_handle(msgPtr, timestampPtr);
    }
  }

  void exceptionCallback(const robosense::lidar::Error &error) {
    const std::string &error_info = "AC Driver Error: " + error.toString();
    RS_SPDLOG_ERROR(error_info);
  }

  void
  jpeg_handle(const RS_IMAGE_SOURCE_TYPE image_source_type,
              const std::shared_ptr<robosense::lidar::ImageData> &frame,
              const robosense::device::RSTimestampItem::Ptr &timestampPtr) {
    int ret;
    // 构造custom_time
    auto custom_time = robosense::convert::RSConvertManager::secondsToRosStamp(
        frame->timestamp);

    // 获取缓冲区
    std::shared_ptr<uint8_t> frame_data_ptr;
    switch (image_source_type) {
    case RS_IMAGE_SOURCE_TYPE::RS_IMAGE_SOURCE_AC1: {
      if (frame->camera_mode != robosense::lidar::CameraMode::MONO) {
        RS_SPDLOG_WARN("AC1 Image Not MonoImageData !");
        return;
      }
      std::shared_ptr<robosense::lidar::MonoImageData> imagePtr =
          std::dynamic_pointer_cast<robosense::lidar::MonoImageData>(frame);
      if (imagePtr == nullptr) {
        RS_SPDLOG_WARN("AC1 Image Right Cast To MonoImageData Failed !");
        return;
      }
      frame_data_ptr = imagePtr->data;
      // 时间戳
      custom_time = robosense::convert::RSConvertManager::secondsToRosStamp(
          imagePtr->timestamp);
      break;
    }
    case RS_IMAGE_SOURCE_TYPE::RS_IMAGE_SOURCE_AC2_LEFT: {
      if (frame->camera_mode != robosense::lidar::CameraMode::STEREO) {
        RS_SPDLOG_WARN("AC2 Image Left Not StereoImageData !");
        return;
      }
      std::shared_ptr<robosense::lidar::StereoImageData> imagePtr =
          std::dynamic_pointer_cast<robosense::lidar::StereoImageData>(frame);
      if (imagePtr == nullptr) {
        RS_SPDLOG_WARN("AC2 Image Right Cast To StereoImageData Failed !");
        return;
      }
      frame_data_ptr = imagePtr->left_data;
      // 时间戳
      custom_time = robosense::convert::RSConvertManager::secondsToRosStamp(
          imagePtr->left_timestamp);
      break;
    }
    case RS_IMAGE_SOURCE_TYPE::RS_IMAGE_SOURCE_AC2_RIGHT: {
      if (frame->camera_mode != robosense::lidar::CameraMode::STEREO) {
        RS_SPDLOG_WARN("AC2 Image Right Not StereoImageData !");
        return;
      }
      std::shared_ptr<robosense::lidar::StereoImageData> imagePtr =
          std::dynamic_pointer_cast<robosense::lidar::StereoImageData>(frame);
      if (imagePtr == nullptr) {
        RS_SPDLOG_WARN("AC2 Image Right Cast To StereoImageData Failed !");
        return;
      }
      frame_data_ptr = imagePtr->right_data;
      // 时间戳
      custom_time = robosense::convert::RSConvertManager::secondsToRosStamp(
          imagePtr->right_timestamp);
      break;
    }
    }

    if (frame_data_ptr == nullptr) {
      RS_SPDLOG_WARN("Image Jpeg Encode Frame Data Is Nullptr !");
      return;
    }

    size_t jpegBufferLen = rgb_image_size;
    auto jpeg_msg = MAKE_SHARED_ROS_COMPRESSED_IMAGE;
    jpeg_msg->data.resize(jpegBufferLen);
    unsigned char *jpegBuffer = jpeg_msg->data.data();
    switch (image_source_type) {
    case RS_IMAGE_SOURCE_TYPE::RS_IMAGE_SOURCE_AC1: {
      ret = jpeg_encoder_ptr->encode((unsigned char *)frame_data_ptr.get(),
                                     frame->data_bytes, jpegBuffer,
                                     jpegBufferLen);
      if (ret != 0) {
        RS_SPDLOG_ERROR("AC1 Image Jpeg Encode Failed: ret = " +
                        std::to_string(ret));
        return;
      }
      break;
    }
    case RS_IMAGE_SOURCE_TYPE::RS_IMAGE_SOURCE_AC2_LEFT: {
      ret = jpeg_left_encoder_ptr->encode((unsigned char *)frame_data_ptr.get(),
                                          frame->data_bytes, jpegBuffer,
                                          jpegBufferLen);
      if (ret != 0) {
        RS_SPDLOG_ERROR("AC2 Image Left Jpeg Encode Failed: ret = " +
                        std::to_string(ret));
        return;
      }
      break;
    }
    case RS_IMAGE_SOURCE_TYPE::RS_IMAGE_SOURCE_AC2_RIGHT: {
      ret = jpeg_right_encoder_ptr->encode(
          (unsigned char *)frame_data_ptr.get(), frame->data_bytes, jpegBuffer,
          jpegBufferLen);
      if (ret != 0) {
        RS_SPDLOG_ERROR("AC2 Image Right Jpeg Encode Failed: ret = " +
                        std::to_string(ret));
        return;
      }
      break;
    }
    }

    // Publish the jpeg frame as a robosense message
    jpeg_msg->header.stamp = custom_time;
#if defined(ENABLE_USE_CUDA)
    jpeg_msg->format = "rgb8; jpeg compressed rgb8";
#else
    jpeg_msg->format = "rgb8; jpeg compressed bgr8";
#endif // ENABLE_USE_CUDA
    jpeg_msg->data.resize(jpegBufferLen);

    switch (image_source_type) {
    case RS_IMAGE_SOURCE_TYPE::RS_IMAGE_SOURCE_AC1: {
      jpeg_msg->header.frame_id = ac1_image_frame_id;

      timestampPtr->process_timestamp_ns = RS_TIMESTAMP_NS;
      publisher_jpeg->publish(*jpeg_msg);
      timestampPtr->publish_timestamp_ns = RS_TIMESTAMP_NS;
      if (timestamp_manager_ptr) {
        timestampPtr->channel_id =
            robosense::device::RS_CHANNEL_ID_TYPE::RS_CHANNEL_ID_JPEG_IMAGE;
        timestamp_manager_ptr->addTimestamp(timestampPtr);
      }
      break;
    }
    case RS_IMAGE_SOURCE_TYPE::RS_IMAGE_SOURCE_AC2_LEFT: {
      jpeg_msg->header.frame_id = ac2_left_image_frame_id;
      timestampPtr->process_timestamp_ns = RS_TIMESTAMP_NS;
      publisher_jpeg_left->publish(*jpeg_msg);
      timestampPtr->publish_timestamp_ns = RS_TIMESTAMP_NS;
      if (timestamp_manager_ptr) {
        timestampPtr->channel_id = robosense::device::RS_CHANNEL_ID_TYPE::
            RS_CHANNEL_ID_JPEG_LEFT_IMAGE;
        timestamp_manager_ptr->addTimestamp(timestampPtr);
      }
      break;
    }
    case RS_IMAGE_SOURCE_TYPE::RS_IMAGE_SOURCE_AC2_RIGHT: {
      jpeg_msg->header.frame_id = ac2_right_image_frame_id;
      timestampPtr->process_timestamp_ns = RS_TIMESTAMP_NS;
      publisher_jpeg_right->publish(*jpeg_msg);
      timestampPtr->publish_timestamp_ns = RS_TIMESTAMP_NS;
      if (timestamp_manager_ptr) {
        timestampPtr->channel_id = robosense::device::RS_CHANNEL_ID_TYPE::
            RS_CHANNEL_ID_JPEG_RIGHT_IMAGE;
        timestamp_manager_ptr->addTimestamp(timestampPtr);
      }
      break;
    }
    }
  }

  void jpeg_rectify_handle(
      const RS_IMAGE_SOURCE_TYPE image_source_type,
      const std::shared_ptr<robosense::lidar::ImageData> &frame,
      const robosense::device::RSTimestampItem::Ptr &timestampPtr) {
    int ret;
    // 构造custom_time
    auto custom_time = robosense::convert::RSConvertManager::secondsToRosStamp(
        frame->timestamp);

    // 获取缓冲区
    std::shared_ptr<uint8_t> frame_data_ptr;
    switch (image_source_type) {
    case RS_IMAGE_SOURCE_TYPE::RS_IMAGE_SOURCE_AC1: {
      if (frame->camera_mode != robosense::lidar::CameraMode::MONO) {
        RS_SPDLOG_WARN("AC1 Image Not MonoImageData !");
        return;
      }
      std::shared_ptr<robosense::lidar::MonoImageData> imagePtr =
          std::dynamic_pointer_cast<robosense::lidar::MonoImageData>(frame);
      if (imagePtr == nullptr) {
        RS_SPDLOG_WARN("AC1 Image Right Cast To MonoImageData Failed !");
        return;
      }
      frame_data_ptr = imagePtr->data;
      // 时间戳
      custom_time = robosense::convert::RSConvertManager::secondsToRosStamp(
          imagePtr->timestamp);
      break;
    }
    case RS_IMAGE_SOURCE_TYPE::RS_IMAGE_SOURCE_AC2_LEFT: {
      if (frame->camera_mode != robosense::lidar::CameraMode::STEREO) {
        RS_SPDLOG_WARN("AC2 Image Left Not StereoImageData !");
        return;
      }
      std::shared_ptr<robosense::lidar::StereoImageData> imagePtr =
          std::dynamic_pointer_cast<robosense::lidar::StereoImageData>(frame);
      if (imagePtr == nullptr) {
        RS_SPDLOG_WARN("AC2 Image Right Cast To StereoImageData Failed !");
        return;
      }
      frame_data_ptr = imagePtr->left_data;
      // 时间戳
      custom_time = robosense::convert::RSConvertManager::secondsToRosStamp(
          imagePtr->left_timestamp);
      break;
    }
    case RS_IMAGE_SOURCE_TYPE::RS_IMAGE_SOURCE_AC2_RIGHT: {
      if (frame->camera_mode != robosense::lidar::CameraMode::STEREO) {
        RS_SPDLOG_WARN("AC2 Image Right Not StereoImageData !");
        return;
      }
      std::shared_ptr<robosense::lidar::StereoImageData> imagePtr =
          std::dynamic_pointer_cast<robosense::lidar::StereoImageData>(frame);
      if (imagePtr == nullptr) {
        RS_SPDLOG_WARN("AC2 Image Right Cast To StereoImageData Failed !");
        return;
      }
      frame_data_ptr = imagePtr->right_data;
      // 时间戳
      custom_time = robosense::convert::RSConvertManager::secondsToRosStamp(
          imagePtr->right_timestamp);
      break;
    }
    }

    if (frame_data_ptr == nullptr) {
      RS_SPDLOG_WARN("Image Jpeg Encode Frame Data Is Nullptr !");
      return;
    }

    size_t jpegBufferLen = rgb_image_size;
    auto jpeg_msg = MAKE_SHARED_ROS_COMPRESSED_IMAGE;
    jpeg_msg->data.resize(jpegBufferLen);
    unsigned char *jpegBuffer = jpeg_msg->data.data();
    switch (image_source_type) {
    case RS_IMAGE_SOURCE_TYPE::RS_IMAGE_SOURCE_AC1: {
      ret = jpeg_rectify_encoder_ptr->encode(
          (unsigned char *)frame_data_ptr.get(), frame->data_bytes, jpegBuffer,
          jpegBufferLen);
      if (ret != 0) {
        RS_SPDLOG_ERROR("AC1 Image Jpeg Encode Failed: ret = " +
                        std::to_string(ret));
        return;
      }
      break;
    }
    case RS_IMAGE_SOURCE_TYPE::RS_IMAGE_SOURCE_AC2_LEFT: {
      ret = jpeg_rectify_left_encoder_ptr->encode(
          (unsigned char *)frame_data_ptr.get(), frame->data_bytes, jpegBuffer,
          jpegBufferLen);
      if (ret != 0) {
        RS_SPDLOG_ERROR("AC2 Image Left Jpeg Encode Failed: ret = " +
                        std::to_string(ret));
        return;
      }
      break;
    }
    case RS_IMAGE_SOURCE_TYPE::RS_IMAGE_SOURCE_AC2_RIGHT: {
      ret = jpeg_rectify_right_encoder_ptr->encode(
          (unsigned char *)frame_data_ptr.get(), frame->data_bytes, jpegBuffer,
          jpegBufferLen);
      if (ret != 0) {
        RS_SPDLOG_ERROR("AC2 Image Right Jpeg Encode Failed: ret = " +
                        std::to_string(ret));
        return;
      }
      break;
    }
    }

    // Publish the jpeg frame as a robosense message
    jpeg_msg->header.stamp = custom_time;
#if defined(ENABLE_USE_CUDA)
    jpeg_msg->format = "rgb8; jpeg compressed rgb8";
#else
    jpeg_msg->format = "rgb8; jpeg compressed bgr8";
#endif // ENABLE_USE_CUDA
    jpeg_msg->data.resize(jpegBufferLen);

    switch (image_source_type) {
    case RS_IMAGE_SOURCE_TYPE::RS_IMAGE_SOURCE_AC1: {
      jpeg_msg->header.frame_id = ac1_image_frame_id;

      timestampPtr->process_timestamp_ns = RS_TIMESTAMP_NS;
      publisher_jpeg_rectify->publish(*jpeg_msg);
      timestampPtr->publish_timestamp_ns = RS_TIMESTAMP_NS;
      if (timestamp_manager_ptr) {
        timestampPtr->channel_id = robosense::device::RS_CHANNEL_ID_TYPE::
            RS_CHANNEL_ID_JPEG_RECTIFY_IMAGE;
        timestamp_manager_ptr->addTimestamp(timestampPtr);
      }
      break;
    }
    case RS_IMAGE_SOURCE_TYPE::RS_IMAGE_SOURCE_AC2_LEFT: {
      jpeg_msg->header.frame_id = ac2_left_image_frame_id;
      timestampPtr->process_timestamp_ns = RS_TIMESTAMP_NS;
      publisher_jpeg_rectify_left->publish(*jpeg_msg);
      timestampPtr->publish_timestamp_ns = RS_TIMESTAMP_NS;
      if (timestamp_manager_ptr) {
        timestampPtr->channel_id = robosense::device::RS_CHANNEL_ID_TYPE::
            RS_CHANNEL_ID_JPEG_RECTIFY_LEFT_IMAGE;
        timestamp_manager_ptr->addTimestamp(timestampPtr);
      }
      break;
    }
    case RS_IMAGE_SOURCE_TYPE::RS_IMAGE_SOURCE_AC2_RIGHT: {
      jpeg_msg->header.frame_id = ac2_right_image_frame_id;
      timestampPtr->process_timestamp_ns = RS_TIMESTAMP_NS;
      publisher_jpeg_rectify_right->publish(*jpeg_msg);
      timestampPtr->publish_timestamp_ns = RS_TIMESTAMP_NS;
      if (timestamp_manager_ptr) {
        timestampPtr->channel_id = robosense::device::RS_CHANNEL_ID_TYPE::
            RS_CHANNEL_ID_JPEG_RECTIFY_RIGHT_IMAGE;
        timestamp_manager_ptr->addTimestamp(timestampPtr);
      }
      break;
    }
    }
  }

  int crop_rgb_image(uint8_t *rgb_data_buf, uint8_t *crop_rgb_data_buf,
                     const RS_IMAGE_SOURCE_TYPE image_source_type) {

    if (rgb_data_buf == nullptr || crop_rgb_data_buf == nullptr) {
      RS_SPDLOG_ERROR(
          "Crop Rgb Image Input rgb_data_buf or crop_rgb_data_buf is "
          "Nullptr !");
      return -1;
    }

    int rgb_width_step = image_width_rgb * 3;
    int crop_rgb_width_step = 0;

    int start_row = 0;
    int end_row = image_height_rgb;
    int start_col = 0;
    int end_col = image_width_rgb;
    switch (image_source_type) {
    case RS_IMAGE_SOURCE_TYPE::RS_IMAGE_SOURCE_AC1: {
      crop_rgb_width_step = image_crop_width_rgb * 3;

      start_row = ac1_crop_config.getCropTop();
      end_row = image_height_rgb - ac1_crop_config.getCropBottom();
      start_col = ac1_crop_config.getCropLeft();
      end_col = image_width_rgb - ac1_crop_config.getCropRight();
      break;
    }
    case RS_IMAGE_SOURCE_TYPE::RS_IMAGE_SOURCE_AC2_LEFT: {
      crop_rgb_width_step = image_left_crop_width_rgb * 3;

      start_row = ac2_left_crop_config.getCropTop();
      end_row = image_height_rgb - ac2_left_crop_config.getCropBottom();
      start_col = ac2_left_crop_config.getCropLeft();
      end_col = image_width_rgb - ac2_left_crop_config.getCropRight();
      break;
    }
    case RS_IMAGE_SOURCE_TYPE::RS_IMAGE_SOURCE_AC2_RIGHT: {
      crop_rgb_width_step = image_right_crop_width_rgb * 3;

      start_row = ac2_right_crop_config.getCropTop();
      end_row = image_height_rgb - ac2_right_crop_config.getCropBottom();
      start_col = ac2_right_crop_config.getCropLeft();
      end_col = image_width_rgb - ac2_right_crop_config.getCropRight();
      break;
    }
    }

    // 进行内存拷贝
    int crop_rgb_offset = 0;
    for (int j = start_row; j < end_row; ++j) {
      int rgb_offset = j * rgb_width_step + start_col * 3;
      memcpy(crop_rgb_data_buf + crop_rgb_offset, rgb_data_buf + rgb_offset,
             crop_rgb_width_step);
      crop_rgb_offset += crop_rgb_width_step;
    }

    return 0;
  }

  void rgb_handle(const RS_IMAGE_SOURCE_TYPE image_source_type,
                  const std::shared_ptr<robosense::lidar::ImageData> &frame,
                  const robosense::device::RSTimestampItem::Ptr &timestampPtr) {
    int ret = 0;
    // 校验数据分辨率
    if (frame->width != image_width_rgb || frame->height != image_height_rgb) {
      RS_SPDLOG_WARN("Image Size Not Match !");
      return;
    }

    // 构造custom_time时间戳
    auto custom_time = robosense::convert::RSConvertManager::secondsToRosStamp(
        frame->timestamp);

    // 根据类型确定数据
    std::shared_ptr<uint8_t> frame_data_ptr;
    uint8_t *rgb_data_buf = nullptr;
    uint8_t *crop_rgb_data_buf = nullptr;
    switch (image_source_type) {
    case RS_IMAGE_SOURCE_TYPE::RS_IMAGE_SOURCE_AC1: {
      if (frame->camera_mode != robosense::lidar::CameraMode::MONO) {
        RS_SPDLOG_WARN("AC1 Image Not MonoImageData !");
        return;
      }
      std::shared_ptr<robosense::lidar::MonoImageData> imagePtr =
          std::dynamic_pointer_cast<robosense::lidar::MonoImageData>(frame);
      if (imagePtr == nullptr) {
        RS_SPDLOG_WARN("AC1 Image Right Cast To MonoImageData Failed !");
        return;
      }
      frame_data_ptr = imagePtr->data;
      rgb_data_buf = rgb_buf.data();
      crop_rgb_data_buf = crop_rgb_buf.data();
      // 时间戳
      custom_time = robosense::convert::RSConvertManager::secondsToRosStamp(
          imagePtr->timestamp);
      // 更新消息时间戳
      timestampPtr->message_timestamp_ns =
          robosense::convert::RSConvertManager::rosStampToNanoseconds(
              custom_time);
      timestampPtr->sot_timestamp_ns = imagePtr->sot_timestamp * 1e9;
      timestampPtr->sot_timestamp_rt_ns = imagePtr->sot_timestamp_rt * 1e9;
      break;
    }
    case RS_IMAGE_SOURCE_TYPE::RS_IMAGE_SOURCE_AC2_LEFT: {
      if (frame->camera_mode != robosense::lidar::CameraMode::STEREO) {
        RS_SPDLOG_WARN("AC2 Image Left Not StereoImageData !");
        return;
      }
      std::shared_ptr<robosense::lidar::StereoImageData> imagePtr =
          std::dynamic_pointer_cast<robosense::lidar::StereoImageData>(frame);
      if (imagePtr == nullptr) {
        RS_SPDLOG_WARN("AC2 Image Right Cast To StereoImageData Failed !");
        return;
      }
      frame_data_ptr = imagePtr->left_data;
      rgb_data_buf = rgb_left_buf.data();
      crop_rgb_data_buf = crop_rgb_left_buf.data();
      // 时间戳
      custom_time = robosense::convert::RSConvertManager::secondsToRosStamp(
          imagePtr->left_timestamp);
      // 更新消息时间戳
      timestampPtr->message_timestamp_ns =
          robosense::convert::RSConvertManager::rosStampToNanoseconds(
              custom_time);
      timestampPtr->sot_timestamp_ns = imagePtr->left_sot_timestamp * 1e9;
      timestampPtr->sot_timestamp_rt_ns = imagePtr->left_sot_timestamp_rt * 1e9;
      break;
    }
    case RS_IMAGE_SOURCE_TYPE::RS_IMAGE_SOURCE_AC2_RIGHT: {
      if (frame->camera_mode != robosense::lidar::CameraMode::STEREO) {
        RS_SPDLOG_WARN("AC2 Image Right Not StereoImageData !");
        return;
      }
      std::shared_ptr<robosense::lidar::StereoImageData> imagePtr =
          std::dynamic_pointer_cast<robosense::lidar::StereoImageData>(frame);
      if (imagePtr == nullptr) {
        RS_SPDLOG_WARN("AC2 Image Right Cast To StereoImageData Failed !");
        return;
      }
      frame_data_ptr = imagePtr->right_data;
      rgb_data_buf = rgb_right_buf.data();
      crop_rgb_data_buf = crop_rgb_right_buf.data();
      // 时间戳
      custom_time = robosense::convert::RSConvertManager::secondsToRosStamp(
          imagePtr->right_timestamp);
      // 更新消息时间戳
      timestampPtr->message_timestamp_ns =
          robosense::convert::RSConvertManager::rosStampToNanoseconds(
              custom_time);
      timestampPtr->sot_timestamp_ns = imagePtr->right_sot_timestamp * 1e9;
      timestampPtr->sot_timestamp_rt_ns =
          imagePtr->right_sot_timestamp_rt * 1e9;
      break;
    }
    }

    if (frame_data_ptr == nullptr) {
      RS_SPDLOG_WARN("Image Rgb Frame Data Is Nullptr !");
      return;
    }

    // 颜色空间变换
    if (frame->frame_format ==
        robosense::lidar::frame_format::FRAME_FORMAT_NV12) {
#if defined(RK3588)
      // 特殊处理
      ret = nv12_to_rgb_rk3588(frame_data_ptr.get(), frame->data_bytes,
                               frame->width, frame->height, rgb_data_buf);

      if (ret != 0) {
        return;
      }
#else
      // 其他普通处理
      switch (image_source_type) {
      case RS_IMAGE_SOURCE_TYPE::RS_IMAGE_SOURCE_AC1: {
        ret = rgb_codec_ptr->NV12ToRGB(frame_data_ptr.get(), frame->data_bytes,
                                       rgb_data_buf, rgb_image_size);
        if (ret != 0) {
          RS_SPDLOG_WARN("AC1 NV12 Image To Rgb Failed: ret = " +
                         std::to_string(ret));
          return;
        }

        break;
      }
      case RS_IMAGE_SOURCE_TYPE::RS_IMAGE_SOURCE_AC2_LEFT: {
        ret = rgb_left_codec_ptr->NV12ToRGB(frame_data_ptr.get(),
                                            frame->data_bytes, rgb_data_buf,
                                            rgb_image_size);
        if (ret != 0) {
          RS_SPDLOG_WARN("AC2 Left NV12 Image To Rgb Failed: ret = " +
                         std::to_string(ret));
          return;
        }

        break;
      }
      case RS_IMAGE_SOURCE_TYPE::RS_IMAGE_SOURCE_AC2_RIGHT: {
        ret = rgb_right_codec_ptr->NV12ToRGB(frame_data_ptr.get(),
                                             frame->data_bytes, rgb_data_buf,
                                             rgb_image_size);
        if (ret != 0) {
          RS_SPDLOG_WARN("AC2 Right NV12 Image To Rgb Failed: ret = " +
                         std::to_string(ret));
          return;
        }
        break;
      }
      } // switch
#endif // defined(RK3588)
    } else if (frame->frame_format ==
                   robosense::lidar::frame_format::FRAME_FORMAT_RGB24 ||
               frame->frame_format ==
                   robosense::lidar::frame_format::FRAME_FORMAT_XR24 ||
               frame->frame_format ==
                   robosense::lidar::frame_format::FRAME_FORMAT_GREY) {
      rgb_data_buf = frame_data_ptr.get();
    } else {
      // Not Support
      RS_SPDLOG_WARN("Image Rgb Frame Not Support Format: " +
                     std::to_string(frame->frame_format) +
                     ", data_bytes = " + std::to_string(frame->data_bytes));
      return;
    }

    // 剪切图像
    int32_t rgb_image_data_size = rgb_image_size;
    int32_t rgb_image_data_width = image_width_rgb;
    int32_t rgb_image_data_height = image_height_rgb;
    std::string rgb_image_data_frame_id;
    bool is_rgb_image_crop = false;
    switch (image_source_type) {
    case RS_IMAGE_SOURCE_TYPE::RS_IMAGE_SOURCE_AC1: {
      if (ac1_crop_config.checkIsCropImage()) {
        crop_rgb_image(rgb_data_buf, crop_rgb_data_buf, image_source_type);
        is_rgb_image_crop = true;
        rgb_image_data_size = rgb_crop_image_size;
        rgb_image_data_width = image_crop_width_rgb;
        rgb_image_data_height = image_crop_height_rgb;
      }
      rgb_image_data_frame_id = ac1_image_frame_id;
      break;
    }
    case RS_IMAGE_SOURCE_TYPE::RS_IMAGE_SOURCE_AC2_LEFT: {
      if (ac2_left_crop_config.checkIsCropImage()) {
        crop_rgb_image(rgb_data_buf, crop_rgb_data_buf, image_source_type);
        is_rgb_image_crop = true;
        rgb_image_data_size = rgb_left_crop_image_size;
        rgb_image_data_width = image_left_crop_width_rgb;
        rgb_image_data_height = image_left_crop_height_rgb;
      }
      rgb_image_data_frame_id = ac2_left_image_frame_id;
      break;
    }
    case RS_IMAGE_SOURCE_TYPE::RS_IMAGE_SOURCE_AC2_RIGHT: {
      if (ac2_right_crop_config.checkIsCropImage()) {
        crop_rgb_image(rgb_data_buf, crop_rgb_data_buf, image_source_type);
        is_rgb_image_crop = true;
        rgb_image_data_size = rgb_right_crop_image_size;
        rgb_image_data_width = image_right_crop_width_rgb;
        rgb_image_data_height = image_right_crop_height_rgb;
      }
      rgb_image_data_frame_id = ac2_right_image_frame_id;
      break;
    }
    } // switch
    if (is_rgb_image_crop) {
      rgb_data_buf = crop_rgb_data_buf;
    }

    // Publish ROS/ROS2 RGB Image Message
    if (!enable_ros2_zero_copy) {
      auto rgb_msg = MAKE_SHARED_ROS_IMAGE;
      // 构造Ros/Ros2 消息
      ret = robosense::convert::RSConvertManager::toRosImageMessage(
          rgb_image_data_width, rgb_image_data_height, custom_time,
          rgb_image_data_frame_id, rgb_data_buf, rgb_image_data_size, rgb_msg);
      if (ret != 0) {
        RS_SPDLOG_ERROR(
            "Convert To ROS/ROS2 Message Failed: rgb_image_data_frame_id = " +
            rgb_image_data_frame_id);
        return;
      }

      // 发送Ros/Ros2 消息
      robosense::device::RS_CHANNEL_ID_TYPE channel_id =
          robosense::device::RS_CHANNEL_ID_TYPE::RS_CHANNEL_ID_RGB_IMAGE;
      switch (image_source_type) {
      case RS_IMAGE_SOURCE_TYPE::RS_IMAGE_SOURCE_AC1: {
        timestampPtr->process_timestamp_ns = RS_TIMESTAMP_NS;
        if (camera_info_ptr) {
          ROS_CAMERAINFO camera_info = *camera_info_ptr;
          camera_info.header = rgb_msg->header;
          publisher_camera_info->publish(camera_info);
        }
        publisher_rgb->publish(std::move(*rgb_msg));
        channel_id =
            robosense::device::RS_CHANNEL_ID_TYPE::RS_CHANNEL_ID_RGB_IMAGE;
        break;
      }
      case RS_IMAGE_SOURCE_TYPE::RS_IMAGE_SOURCE_AC2_LEFT: {
        timestampPtr->process_timestamp_ns = RS_TIMESTAMP_NS;
        if (left_camera_info_ptr) {
          ROS_CAMERAINFO camera_info = *left_camera_info_ptr;
          camera_info.header = rgb_msg->header;
          publisher_left_camera_info->publish(camera_info);
        }
        publisher_rgb_left->publish(std::move(*rgb_msg));
        channel_id =
            robosense::device::RS_CHANNEL_ID_TYPE::RS_CHANNEL_ID_RGB_LEFT_IMAGE;
        break;
      }
      case RS_IMAGE_SOURCE_TYPE::RS_IMAGE_SOURCE_AC2_RIGHT: {
        timestampPtr->process_timestamp_ns = RS_TIMESTAMP_NS;
        if (right_camera_info_ptr) {
          ROS_CAMERAINFO camera_info = *right_camera_info_ptr;
          camera_info.header = rgb_msg->header;
          publisher_right_camera_info->publish(camera_info);
        }
        publisher_rgb_right->publish(std::move(*rgb_msg));
        channel_id = robosense::device::RS_CHANNEL_ID_TYPE::
            RS_CHANNEL_ID_RGB_RIGHT_IMAGE;
        break;
      }
      } // switch
      timestampPtr->publish_timestamp_ns = RS_TIMESTAMP_NS;
      if (timestamp_manager_ptr) {
        timestampPtr->channel_id = channel_id;
        timestamp_manager_ptr->addTimestamp(timestampPtr);
      }
    }
#if defined(ROS2_FOUND)
    else if (enable_ros2_zero_copy) {
      switch (image_source_type) {
      case RS_IMAGE_SOURCE_TYPE::RS_IMAGE_SOURCE_AC1: {
        rclcpp::LoanedMessage<ROS_ZEROCOPY_IMAGE8M> loanedMsg =
            publisher_rgb_loan->borrow_loaned_message();
        if (!loanedMsg.is_valid()) {
          // 获取消息失败，丢弃该消息
          RS_SPDLOG_ERROR("Failed to get AC1 Rgb LoanMessage !");
          return;
        }
        // 引用方式获取实际的消息
        auto &msg = loanedMsg.get();
        auto rgb_msg = &msg;
        // 构造零拷贝消息
        robosense::convert::RSConvertManager::toZeroCopyImageMessage<
            ROS_ZEROCOPY_IMAGE8M>(rgb_image_data_width, rgb_image_data_height,
                                  custom_time, rgb_image_data_frame_id,
                                  rgb_data_buf, rgb_image_data_size, rgb_msg);

        timestampPtr->process_timestamp_ns = RS_TIMESTAMP_NS;
        if (camera_info_ptr) {
          ROS_CAMERAINFO camera_info = *camera_info_ptr;
          camera_info.header.stamp = rgb_msg->header.stamp;
          camera_info.header.frame_id = rgb_image_data_frame_id;
          publisher_camera_info->publish(camera_info);
        }
        publisher_rgb_loan->publish(std::move(*rgb_msg));
        timestampPtr->publish_timestamp_ns = RS_TIMESTAMP_NS;
        if (timestamp_manager_ptr) {
          timestampPtr->channel_id =
              robosense::device::RS_CHANNEL_ID_TYPE::RS_CHANNEL_ID_RGB_IMAGE;
          timestamp_manager_ptr->addTimestamp(timestampPtr);
        }
        break;
      }
      case RS_IMAGE_SOURCE_TYPE::RS_IMAGE_SOURCE_AC2_LEFT: {
        rclcpp::LoanedMessage<ROS_ZEROCOPY_IMAGE4M> loanedMsg =
            publisher_rgb_left_loan->borrow_loaned_message();
        if (!loanedMsg.is_valid()) {
          // 获取消息失败，丢弃该消息
          RS_SPDLOG_ERROR("Failed to get AC2 Rgb Left LoanMessage !");
          return;
        }
        // 引用方式获取实际的消息
        auto &msg = loanedMsg.get();
        auto rgb_msg = &msg;

        // 构造零拷贝消息
        robosense::convert::RSConvertManager::toZeroCopyImageMessage<
            ROS_ZEROCOPY_IMAGE4M>(rgb_image_data_width, rgb_image_data_height,
                                  custom_time, rgb_image_data_frame_id,
                                  rgb_data_buf, rgb_image_data_size, rgb_msg);

        timestampPtr->process_timestamp_ns = RS_TIMESTAMP_NS;
        if (left_camera_info_ptr) {
          ROS_CAMERAINFO camera_info = *left_camera_info_ptr;
          camera_info.header.stamp = rgb_msg->header.stamp;
          camera_info.header.frame_id = rgb_image_data_frame_id;
          publisher_left_camera_info->publish(camera_info);
        }
        publisher_rgb_left_loan->publish(std::move(*rgb_msg));
        timestampPtr->publish_timestamp_ns = RS_TIMESTAMP_NS;
        if (timestamp_manager_ptr) {
          timestampPtr->channel_id = robosense::device::RS_CHANNEL_ID_TYPE::
              RS_CHANNEL_ID_RGB_LEFT_IMAGE;
          timestamp_manager_ptr->addTimestamp(timestampPtr);
        }
        break;
      }
      case RS_IMAGE_SOURCE_TYPE::RS_IMAGE_SOURCE_AC2_RIGHT: {
        rclcpp::LoanedMessage<ROS_ZEROCOPY_IMAGE4M> loanedMsg =
            publisher_rgb_right_loan->borrow_loaned_message();
        if (!loanedMsg.is_valid()) {
          // 获取消息失败，丢弃该消息
          RS_SPDLOG_ERROR("Failed to get AC2 Rgb Right LoanMessage !");
          return;
        }
        // 引用方式获取实际的消息
        auto &msg = loanedMsg.get();
        auto rgb_msg = &msg;

        // 构造零拷贝消息
        robosense::convert::RSConvertManager::toZeroCopyImageMessage<
            ROS_ZEROCOPY_IMAGE4M>(rgb_image_data_width, rgb_image_data_height,
                                  custom_time, rgb_image_data_frame_id,
                                  rgb_data_buf, rgb_image_data_size, rgb_msg);

        timestampPtr->process_timestamp_ns = RS_TIMESTAMP_NS;
        if (right_camera_info_ptr) {
          ROS_CAMERAINFO camera_info = *right_camera_info_ptr;
          camera_info.header.stamp = rgb_msg->header.stamp;
          camera_info.header.frame_id = rgb_image_data_frame_id;
          publisher_right_camera_info->publish(camera_info);
        }
        publisher_rgb_right_loan->publish(std::move(*rgb_msg));

        break;
      }
      } // switch
      timestampPtr->publish_timestamp_ns = RS_TIMESTAMP_NS;
      if (timestamp_manager_ptr) {
        timestampPtr->channel_id = robosense::device::RS_CHANNEL_ID_TYPE::
            RS_CHANNEL_ID_RGB_RIGHT_IMAGE;
        timestamp_manager_ptr->addTimestamp(timestampPtr);
      }
    }
#endif // defined(ROS2_FOUND)

    // 使能JPEG时，RGB数据继续提供JPEG Encoder处理
    if (enable_jpeg) {
      robosense::device::RSTimestampItem::Ptr jpegTimestampPtr(
          new robosense::device::RSTimestampItem(*timestampPtr));
      // Case1: NV12 -> RGB And/Or Crop
      // Case2: RGB -> RGB And Crop
      bool is_need_new_rgb_msg = false;
      if (!(frame->frame_format ==
                robosense::lidar::frame_format::FRAME_FORMAT_RGB24 ||
            frame->frame_format ==
                robosense::lidar::frame_format::FRAME_FORMAT_XR24)) {
        is_need_new_rgb_msg = true;
      }
      is_need_new_rgb_msg = is_need_new_rgb_msg || is_rgb_image_crop;

      std::shared_ptr<robosense::lidar::ImageData> jpegImagePtr = frame;
      switch (image_source_type) {
      case RS_IMAGE_SOURCE_TYPE::RS_IMAGE_SOURCE_AC1: {
        if (is_need_new_rgb_msg) {
          std::shared_ptr<robosense::lidar::MonoImageData> rgbImagePtr(
              new robosense::lidar::MonoImageData());
          rgbImagePtr->frame_format =
              robosense::lidar::frame_format_t::FRAME_FORMAT_RGB24;
          rgbImagePtr->data_bytes = rgb_image_data_size;
          rgbImagePtr->width = rgb_image_data_width;
          rgbImagePtr->height = rgb_image_data_height;
          rgbImagePtr->timestamp = frame->timestamp;
          rgbImagePtr->camera_mode = frame->camera_mode;
          rgbImagePtr->state = frame->state;

          rgbImagePtr->data =
              std::shared_ptr<uint8_t>(new uint8_t[rgbImagePtr->data_bytes],
                                       std::default_delete<uint8_t[]>());
          memcpy(rgbImagePtr->data.get(), rgb_data_buf,
                 rgbImagePtr->data_bytes);

          jpegImagePtr = rgbImagePtr;
        }

        std::lock_guard<std::mutex> lock(jpeg_mutex_);
        jpeg_queue_.push({jpegImagePtr, jpegTimestampPtr});
        jpeg_condition_.notify_one();
        break;
      }
      case RS_IMAGE_SOURCE_TYPE::RS_IMAGE_SOURCE_AC2_LEFT: {
        if (is_need_new_rgb_msg) {
          std::shared_ptr<robosense::lidar::StereoImageData> imagePtr =
              std::dynamic_pointer_cast<robosense::lidar::StereoImageData>(
                  frame);
          std::shared_ptr<robosense::lidar::StereoImageData> rgbImagePtr(
              new robosense::lidar::StereoImageData());
          rgbImagePtr->frame_format =
              robosense::lidar::frame_format_t::FRAME_FORMAT_RGB24;
          rgbImagePtr->data_bytes = rgb_image_data_size;
          rgbImagePtr->width = rgb_image_data_width;
          rgbImagePtr->height = rgb_image_data_height;
          rgbImagePtr->timestamp = frame->timestamp;
          rgbImagePtr->left_timestamp = imagePtr->left_timestamp;
          rgbImagePtr->camera_mode = frame->camera_mode;
          rgbImagePtr->state = frame->state;

          rgbImagePtr->left_data =
              std::shared_ptr<uint8_t>(new uint8_t[rgbImagePtr->data_bytes],
                                       std::default_delete<uint8_t[]>());
          memcpy(rgbImagePtr->left_data.get(), rgb_data_buf,
                 rgbImagePtr->data_bytes);

          jpegImagePtr = rgbImagePtr;
        }
        std::lock_guard<std::mutex> lock(jpeg_left_mutex_);
        jpeg_left_queue_.push({jpegImagePtr, jpegTimestampPtr});
        jpeg_left_condition_.notify_one();
        break;
      }
      case RS_IMAGE_SOURCE_TYPE::RS_IMAGE_SOURCE_AC2_RIGHT: {
        if (is_need_new_rgb_msg) {
          std::shared_ptr<robosense::lidar::StereoImageData> imagePtr =
              std::dynamic_pointer_cast<robosense::lidar::StereoImageData>(
                  frame);
          std::shared_ptr<robosense::lidar::StereoImageData> rgbImagePtr(
              new robosense::lidar::StereoImageData());
          rgbImagePtr->frame_format =
              robosense::lidar::frame_format_t::FRAME_FORMAT_RGB24;
          rgbImagePtr->data_bytes = rgb_image_data_size;
          rgbImagePtr->width = rgb_image_data_width;
          rgbImagePtr->height = rgb_image_data_height;
          rgbImagePtr->timestamp = frame->timestamp;
          rgbImagePtr->right_timestamp = imagePtr->right_timestamp;
          rgbImagePtr->camera_mode = frame->camera_mode;
          rgbImagePtr->state = frame->state;

          rgbImagePtr->right_data =
              std::shared_ptr<uint8_t>(new uint8_t[rgbImagePtr->data_bytes],
                                       std::default_delete<uint8_t[]>());
          memcpy(rgbImagePtr->right_data.get(), rgb_data_buf,
                 rgbImagePtr->data_bytes);

          jpegImagePtr = rgbImagePtr;
        }
        std::lock_guard<std::mutex> lock(jpeg_right_mutex_);
        jpeg_right_queue_.push({jpegImagePtr, jpegTimestampPtr});
        jpeg_right_condition_.notify_one();
        break;
      }
      } // switch
    }   // if(enable_jpeg)

    // 使能Rectify
    if (enable_rectify) {
      robosense::device::RSTimestampItem::Ptr jpegTimestampPtr(
          new robosense::device::RSTimestampItem(*timestampPtr));
      bool is_need_new_rgb_msg = true;

      std::shared_ptr<robosense::lidar::ImageData> rgbRectifyImagePtr = frame;
      switch (image_source_type) {
      case RS_IMAGE_SOURCE_TYPE::RS_IMAGE_SOURCE_AC1: {
        if (is_need_new_rgb_msg) {
          std::shared_ptr<robosense::lidar::MonoImageData> rgbImagePtr(
              new robosense::lidar::MonoImageData());
          rgbImagePtr->frame_format =
              robosense::lidar::frame_format_t::FRAME_FORMAT_RGB24;
          rgbImagePtr->data_bytes = rgb_image_data_size;
          rgbImagePtr->width = rgb_image_data_width;
          rgbImagePtr->height = rgb_image_data_height;
          rgbImagePtr->timestamp = frame->timestamp;
          rgbImagePtr->camera_mode = frame->camera_mode;
          rgbImagePtr->state = frame->state;

          rgbImagePtr->data =
              std::shared_ptr<uint8_t>(new uint8_t[rgbImagePtr->data_bytes],
                                       std::default_delete<uint8_t[]>());
          memcpy(rgbImagePtr->data.get(), rgb_data_buf,
                 rgbImagePtr->data_bytes);

          rgbRectifyImagePtr = rgbImagePtr;
        }

        std::lock_guard<std::mutex> lock(rgb_rectify_mutex_);
        rgb_rectify_queue_.push({rgbRectifyImagePtr, jpegTimestampPtr});
        rgb_rectify_condition_.notify_one();
        break;
      }
      case RS_IMAGE_SOURCE_TYPE::RS_IMAGE_SOURCE_AC2_LEFT: {
        if (is_need_new_rgb_msg) {
          std::shared_ptr<robosense::lidar::StereoImageData> imagePtr =
              std::dynamic_pointer_cast<robosense::lidar::StereoImageData>(
                  frame);
          std::shared_ptr<robosense::lidar::StereoImageData> rgbImagePtr(
              new robosense::lidar::StereoImageData());
          rgbImagePtr->frame_format =
              robosense::lidar::frame_format_t::FRAME_FORMAT_RGB24;
          rgbImagePtr->data_bytes = rgb_image_data_size;
          rgbImagePtr->width = rgb_image_data_width;
          rgbImagePtr->height = rgb_image_data_height;
          rgbImagePtr->timestamp = frame->timestamp;
          rgbImagePtr->left_timestamp = imagePtr->left_timestamp;
          rgbImagePtr->camera_mode = frame->camera_mode;
          rgbImagePtr->state = frame->state;

          rgbImagePtr->left_data =
              std::shared_ptr<uint8_t>(new uint8_t[rgbImagePtr->data_bytes],
                                       std::default_delete<uint8_t[]>());
          memcpy(rgbImagePtr->left_data.get(), rgb_data_buf,
                 rgbImagePtr->data_bytes);

          rgbRectifyImagePtr = rgbImagePtr;
        }
        std::lock_guard<std::mutex> lock(rgb_rectify_left_mutex_);
        rgb_rectify_left_queue_.push({rgbRectifyImagePtr, jpegTimestampPtr});
        rgb_rectify_left_condition_.notify_one();
        break;
      }
      case RS_IMAGE_SOURCE_TYPE::RS_IMAGE_SOURCE_AC2_RIGHT: {
        if (is_need_new_rgb_msg) {
          std::shared_ptr<robosense::lidar::StereoImageData> imagePtr =
              std::dynamic_pointer_cast<robosense::lidar::StereoImageData>(
                  frame);
          std::shared_ptr<robosense::lidar::StereoImageData> rgbImagePtr(
              new robosense::lidar::StereoImageData());
          rgbImagePtr->frame_format =
              robosense::lidar::frame_format_t::FRAME_FORMAT_RGB24;
          rgbImagePtr->data_bytes = rgb_image_data_size;
          rgbImagePtr->width = rgb_image_data_width;
          rgbImagePtr->height = rgb_image_data_height;
          rgbImagePtr->timestamp = frame->timestamp;
          rgbImagePtr->right_timestamp = imagePtr->right_timestamp;
          rgbImagePtr->camera_mode = frame->camera_mode;
          rgbImagePtr->state = frame->state;

          rgbImagePtr->right_data =
              std::shared_ptr<uint8_t>(new uint8_t[rgbImagePtr->data_bytes],
                                       std::default_delete<uint8_t[]>());
          memcpy(rgbImagePtr->right_data.get(), rgb_data_buf,
                 rgbImagePtr->data_bytes);

          rgbRectifyImagePtr = rgbImagePtr;
        }
        std::lock_guard<std::mutex> lock(rgb_rectify_right_mutex_);
        rgb_rectify_right_queue_.push({rgbRectifyImagePtr, jpegTimestampPtr});
        rgb_rectify_right_condition_.notify_one();
        break;
      }
      } // switch
    }   // if(enable_rectify)
  }

  void rgb_rectify_handle(
      const RS_IMAGE_SOURCE_TYPE image_source_type,
      const std::shared_ptr<robosense::lidar::ImageData> &frame,
      const robosense::device::RSTimestampItem::Ptr &timestampPtr) {
    int ret = 0;
    // 构造custom_time消息
    auto custom_time = robosense::convert::RSConvertManager::secondsToRosStamp(
        frame->timestamp);

    // 根据类型确定数据
    std::shared_ptr<uint8_t> frame_data_ptr;
    switch (image_source_type) {
    case RS_IMAGE_SOURCE_TYPE::RS_IMAGE_SOURCE_AC1: {
      if (frame->camera_mode != robosense::lidar::CameraMode::MONO) {
        RS_SPDLOG_WARN("AC1 Image Not MonoImageData !");
        return;
      }
      std::shared_ptr<robosense::lidar::MonoImageData> imagePtr =
          std::dynamic_pointer_cast<robosense::lidar::MonoImageData>(frame);
      if (imagePtr == nullptr) {
        RS_SPDLOG_WARN("AC1 Image Right Cast To MonoImageData Failed !");
        return;
      }
      frame_data_ptr = imagePtr->data;
      // 消息时间戳
      custom_time = robosense::convert::RSConvertManager::secondsToRosStamp(
          imagePtr->timestamp);
      // 更新消息时间戳
      timestampPtr->message_timestamp_ns =
          robosense::convert::RSConvertManager::rosStampToNanoseconds(
              custom_time);
      timestampPtr->sot_timestamp_ns = imagePtr->sot_timestamp * 1e9;
      timestampPtr->sot_timestamp_rt_ns = imagePtr->sot_timestamp_rt * 1e9;
      break;
    }
    case RS_IMAGE_SOURCE_TYPE::RS_IMAGE_SOURCE_AC2_LEFT: {
      if (frame->camera_mode != robosense::lidar::CameraMode::STEREO) {
        RS_SPDLOG_WARN("AC2 Image Left Not StereoImageData !");
        return;
      }
      std::shared_ptr<robosense::lidar::StereoImageData> imagePtr =
          std::dynamic_pointer_cast<robosense::lidar::StereoImageData>(frame);
      if (imagePtr == nullptr) {
        RS_SPDLOG_WARN("AC2 Image Right Cast To StereoImageData Failed !");
        return;
      }
      frame_data_ptr = imagePtr->left_data;
      // 消息时间戳
      custom_time = robosense::convert::RSConvertManager::secondsToRosStamp(
          imagePtr->left_timestamp);
      // 更新消息时间戳
      timestampPtr->message_timestamp_ns =
          robosense::convert::RSConvertManager::rosStampToNanoseconds(
              custom_time);
      timestampPtr->sot_timestamp_ns = imagePtr->left_sot_timestamp * 1e9;
      timestampPtr->sot_timestamp_rt_ns = imagePtr->left_sot_timestamp_rt * 1e9;
      break;
    }
    case RS_IMAGE_SOURCE_TYPE::RS_IMAGE_SOURCE_AC2_RIGHT: {
      if (frame->camera_mode != robosense::lidar::CameraMode::STEREO) {
        RS_SPDLOG_WARN("AC2 Image Right Not StereoImageData !");
        return;
      }
      std::shared_ptr<robosense::lidar::StereoImageData> imagePtr =
          std::dynamic_pointer_cast<robosense::lidar::StereoImageData>(frame);
      if (imagePtr == nullptr) {
        RS_SPDLOG_WARN("AC2 Image Right Cast To StereoImageData Failed !");
        return;
      }
      frame_data_ptr = imagePtr->right_data;
      // 消息时间戳
      custom_time = robosense::convert::RSConvertManager::secondsToRosStamp(
          imagePtr->right_timestamp);
      // 更新消息时间戳
      timestampPtr->message_timestamp_ns =
          robosense::convert::RSConvertManager::rosStampToNanoseconds(
              custom_time);
      timestampPtr->sot_timestamp_ns = imagePtr->right_sot_timestamp * 1e9;
      timestampPtr->sot_timestamp_rt_ns =
          imagePtr->right_sot_timestamp_rt * 1e9;
      break;
    }
    }

    // 进行去畸变
    int32_t rgb_image_data_height = image_height_rgb;
    int32_t rgb_image_data_width = image_width_rgb;
    int32_t rgb_image_data_size = rgb_image_size;
    std::string rgb_image_data_frame_id;
    cv::Mat outputRectImage;
    switch (image_source_type) {
    case RS_IMAGE_SOURCE_TYPE::RS_IMAGE_SOURCE_AC1: {
      if (!camera_rectify_map_valid) {
        RS_SPDLOG_WARN("AC1 Image Rectify Map Not Valid !");
        return;
      }
      cv::Mat inputRgbMat(frame->height, frame->width, CV_8UC3,
                          frame_data_ptr.get());
      cv::remap(inputRgbMat, outputRectImage, camera_rectify_map1,
                camera_rectify_map2, cv::INTER_LINEAR);
      rgb_image_data_frame_id = ac1_image_frame_id;
      break;
    }
    case RS_IMAGE_SOURCE_TYPE::RS_IMAGE_SOURCE_AC2_LEFT: {
      if (!left_camera_rectify_map_valid) {
        RS_SPDLOG_WARN("AC2 Image Left Rectify Map Not Valid !");
        return;
      }
      cv::Mat inputRgbMat(frame->height, frame->width, CV_8UC3,
                          frame_data_ptr.get());
      cv::remap(inputRgbMat, outputRectImage, left_camera_rectify_map1,
                left_camera_rectify_map2, cv::INTER_LINEAR);
      rgb_image_data_frame_id = ac2_left_image_frame_id;
      break;
    }
    case RS_IMAGE_SOURCE_TYPE::RS_IMAGE_SOURCE_AC2_RIGHT: {
      if (!right_camera_rectify_map_valid) {
        RS_SPDLOG_WARN("AC2 Image Right Rectify Map Not Valid !");
        return;
      }
      cv::Mat inputRgbMat(frame->height, frame->width, CV_8UC3,
                          frame_data_ptr.get());
      cv::remap(inputRgbMat, outputRectImage, right_camera_rectify_map1,
                right_camera_rectify_map2, cv::INTER_LINEAR);
      rgb_image_data_frame_id = ac2_right_image_frame_id;
      break;
    }
    } // switch
    // std::cout << "run here 333" << std::endl;
    if (outputRectImage.empty()) {
      RS_SPDLOG_WARN("Image Rectify Failed: rgb_image_data_frame_id = " +
                     rgb_image_data_frame_id);
      return;
    }

    // 更新去畸变RGB接口:
    rgb_image_data_height = outputRectImage.rows;
    rgb_image_data_width = outputRectImage.cols;
    rgb_image_data_size = rgb_image_data_width * rgb_image_data_height * 3;
    uint8_t *rgb_data_buf = (uint8_t *)outputRectImage.data;

    // Publish ROS/ROS2 RGB Image Message
    if (!enable_ros2_zero_copy) {
      auto rgb_msg = MAKE_SHARED_ROS_IMAGE;
      // 构造Ros消息
      ret = robosense::convert::RSConvertManager::toRosImageMessage(
          rgb_image_data_width, rgb_image_data_height, custom_time,
          rgb_image_data_frame_id, rgb_data_buf, rgb_image_data_size, rgb_msg);
      if (ret != 0) {
        RS_SPDLOG_ERROR(
            "Convert To Ros/Ros2 Message Failed: rgb_image_data_frame_id = " +
            rgb_image_data_frame_id);
        return;
      }

      timestampPtr->process_timestamp_ns = RS_TIMESTAMP_NS;
      robosense::device::RS_CHANNEL_ID_TYPE channel_id = robosense::device::
          RS_CHANNEL_ID_TYPE::RS_CHANNEL_ID_RGB_RECTIFY_IMAGE;
      switch (image_source_type) {
      case RS_IMAGE_SOURCE_TYPE::RS_IMAGE_SOURCE_AC1: {
        publisher_rgb_rectify->publish(std::move(*rgb_msg));
        channel_id = robosense::device::RS_CHANNEL_ID_TYPE::
            RS_CHANNEL_ID_RGB_RECTIFY_IMAGE;
        break;
      }
      case RS_IMAGE_SOURCE_TYPE::RS_IMAGE_SOURCE_AC2_LEFT: {
        timestampPtr->process_timestamp_ns = RS_TIMESTAMP_NS;
        publisher_rgb_rectify_left->publish(std::move(*rgb_msg));
        channel_id = robosense::device::RS_CHANNEL_ID_TYPE::
            RS_CHANNEL_ID_RGB_RECTIFY_LEFT_IMAGE;
        break;
      }
      case RS_IMAGE_SOURCE_TYPE::RS_IMAGE_SOURCE_AC2_RIGHT: {
        timestampPtr->process_timestamp_ns = RS_TIMESTAMP_NS;
        publisher_rgb_rectify_right->publish(std::move(*rgb_msg));
        channel_id = robosense::device::RS_CHANNEL_ID_TYPE::
            RS_CHANNEL_ID_RGB_RECTIFY_RIGHT_IMAGE;
        break;
      }
      } // switch
      timestampPtr->publish_timestamp_ns = RS_TIMESTAMP_NS;
      if (timestamp_manager_ptr) {
        timestampPtr->channel_id = channel_id;
        timestamp_manager_ptr->addTimestamp(timestampPtr);
      }
    }
#if defined(ROS2_FOUND)
    else if (enable_ros2_zero_copy) {
      robosense::device::RS_CHANNEL_ID_TYPE channel_id = robosense::device::
          RS_CHANNEL_ID_TYPE::RS_CHANNEL_ID_RGB_RECTIFY_IMAGE;
      switch (image_source_type) {
      case RS_IMAGE_SOURCE_TYPE::RS_IMAGE_SOURCE_AC1: {
        rclcpp::LoanedMessage<ROS_ZEROCOPY_IMAGE8M> loanedMsg =
            publisher_rgb_loan->borrow_loaned_message();
        if (!loanedMsg.is_valid()) {
          // 获取消息失败，丢弃该消息
          RS_SPDLOG_ERROR("Failed to get AC1 Rgb LoanMessage !");
          return;
        }
        // 引用方式获取实际的消息
        auto &msg = loanedMsg.get();
        auto rgb_msg = &msg;
        robosense::convert::RSConvertManager::toZeroCopyImageMessage<
            ROS_ZEROCOPY_IMAGE8M>(rgb_image_data_width, rgb_image_data_height,
                                  custom_time, rgb_image_data_frame_id,
                                  rgb_data_buf, rgb_image_data_size, rgb_msg);
        timestampPtr->process_timestamp_ns = RS_TIMESTAMP_NS;
        publisher_rgb_rectify_loan->publish(std::move(*rgb_msg));
        channel_id = robosense::device::RS_CHANNEL_ID_TYPE::
            RS_CHANNEL_ID_RGB_RECTIFY_IMAGE;
        break;
      }
      case RS_IMAGE_SOURCE_TYPE::RS_IMAGE_SOURCE_AC2_LEFT: {
        rclcpp::LoanedMessage<ROS_ZEROCOPY_IMAGE4M> loanedMsg =
            publisher_rgb_left_loan->borrow_loaned_message();
        if (!loanedMsg.is_valid()) {
          // 获取消息失败，丢弃该消息
          RS_SPDLOG_ERROR("Failed to get AC2 Rgb Left LoanMessage !");
          return;
        }
        // 引用方式获取实际的消息
        auto &msg = loanedMsg.get();
        auto rgb_msg = &msg;
        robosense::convert::RSConvertManager::toZeroCopyImageMessage<
            ROS_ZEROCOPY_IMAGE4M>(rgb_image_data_width, rgb_image_data_height,
                                  custom_time, rgb_image_data_frame_id,
                                  rgb_data_buf, rgb_image_data_size, rgb_msg);
        timestampPtr->process_timestamp_ns = RS_TIMESTAMP_NS;
        publisher_rgb_rectify_left_loan->publish(std::move(*rgb_msg));
        channel_id = robosense::device::RS_CHANNEL_ID_TYPE::
            RS_CHANNEL_ID_RGB_RECTIFY_LEFT_IMAGE;
        break;
      }
      case RS_IMAGE_SOURCE_TYPE::RS_IMAGE_SOURCE_AC2_RIGHT: {
        rclcpp::LoanedMessage<ROS_ZEROCOPY_IMAGE4M> loanedMsg =
            publisher_rgb_right_loan->borrow_loaned_message();
        if (!loanedMsg.is_valid()) {
          // 获取消息失败，丢弃该消息
          RS_SPDLOG_ERROR("Failed to get AC2 Rgb Right LoanMessage !");
          return;
        }
        // 引用方式获取实际的消息
        auto &msg = loanedMsg.get();
        auto rgb_msg = &msg;
        robosense::convert::RSConvertManager::toZeroCopyImageMessage<
            ROS_ZEROCOPY_IMAGE4M>(rgb_image_data_width, rgb_image_data_height,
                                  custom_time, rgb_image_data_frame_id,
                                  rgb_data_buf, rgb_image_data_size, rgb_msg);
        timestampPtr->process_timestamp_ns = RS_TIMESTAMP_NS;
        publisher_rgb_rectify_right_loan->publish(std::move(*rgb_msg));
        channel_id = robosense::device::RS_CHANNEL_ID_TYPE::
            RS_CHANNEL_ID_RGB_RECTIFY_RIGHT_IMAGE;
        break;
      }
      } // switch
      timestampPtr->publish_timestamp_ns = RS_TIMESTAMP_NS;
      if (timestamp_manager_ptr) {
        timestampPtr->channel_id = robosense::device::RS_CHANNEL_ID_TYPE::
            RS_CHANNEL_ID_RGB_RECTIFY_LEFT_IMAGE;
        timestamp_manager_ptr->addTimestamp(timestampPtr);
      }
    }
#endif // defined(ROS2_FOUND)

    // 使能JPEG时，RGB数据继续提供JPEG Encoder处理
    if (enable_rectify_jpeg) {
      robosense::device::RSTimestampItem::Ptr jpegTimestampPtr(
          new robosense::device::RSTimestampItem(*timestampPtr));
      bool is_need_new_rgb_msg = true;
      std::shared_ptr<robosense::lidar::ImageData> jpegRectifyImagePtr = frame;
      switch (image_source_type) {
      case RS_IMAGE_SOURCE_TYPE::RS_IMAGE_SOURCE_AC1: {
        if (is_need_new_rgb_msg) {
          std::shared_ptr<robosense::lidar::MonoImageData> rgbImagePtr(
              new robosense::lidar::MonoImageData());
          rgbImagePtr->frame_format =
              robosense::lidar::frame_format_t::FRAME_FORMAT_RGB24;
          rgbImagePtr->data_bytes = rgb_image_data_size;
          rgbImagePtr->width = rgb_image_data_width;
          rgbImagePtr->height = rgb_image_data_height;
          rgbImagePtr->timestamp = frame->timestamp;
          rgbImagePtr->camera_mode = frame->camera_mode;
          rgbImagePtr->state = frame->state;

          rgbImagePtr->data =
              std::shared_ptr<uint8_t>(new uint8_t[rgbImagePtr->data_bytes],
                                       std::default_delete<uint8_t[]>());
          memcpy(rgbImagePtr->data.get(), rgb_data_buf,
                 rgbImagePtr->data_bytes);

          jpegRectifyImagePtr = rgbImagePtr;
        }

        std::lock_guard<std::mutex> lock(jpeg_rectify_mutex_);
        jpeg_rectify_queue_.push({jpegRectifyImagePtr, jpegTimestampPtr});
        jpeg_rectify_condition_.notify_one();
        break;
      }
      case RS_IMAGE_SOURCE_TYPE::RS_IMAGE_SOURCE_AC2_LEFT: {
        if (is_need_new_rgb_msg) {
          std::shared_ptr<robosense::lidar::StereoImageData> imagePtr =
              std::dynamic_pointer_cast<robosense::lidar::StereoImageData>(
                  frame);
          std::shared_ptr<robosense::lidar::StereoImageData> rgbImagePtr(
              new robosense::lidar::StereoImageData());
          rgbImagePtr->frame_format =
              robosense::lidar::frame_format_t::FRAME_FORMAT_RGB24;
          rgbImagePtr->data_bytes = rgb_image_data_size;
          rgbImagePtr->width = rgb_image_data_width;
          rgbImagePtr->height = rgb_image_data_height;
          rgbImagePtr->timestamp = frame->timestamp;
          rgbImagePtr->left_timestamp = imagePtr->left_timestamp;
          rgbImagePtr->camera_mode = frame->camera_mode;
          rgbImagePtr->state = frame->state;

          rgbImagePtr->left_data =
              std::shared_ptr<uint8_t>(new uint8_t[rgbImagePtr->data_bytes],
                                       std::default_delete<uint8_t[]>());
          memcpy(rgbImagePtr->left_data.get(), rgb_data_buf,
                 rgbImagePtr->data_bytes);

          jpegRectifyImagePtr = rgbImagePtr;
        }
        std::lock_guard<std::mutex> lock(jpeg_rectify_left_mutex_);
        jpeg_rectify_left_queue_.push({jpegRectifyImagePtr, jpegTimestampPtr});
        jpeg_rectify_left_condition_.notify_one();
        break;
      }
      case RS_IMAGE_SOURCE_TYPE::RS_IMAGE_SOURCE_AC2_RIGHT: {
        if (is_need_new_rgb_msg) {
          std::shared_ptr<robosense::lidar::StereoImageData> imagePtr =
              std::dynamic_pointer_cast<robosense::lidar::StereoImageData>(
                  frame);
          std::shared_ptr<robosense::lidar::StereoImageData> rgbImagePtr(
              new robosense::lidar::StereoImageData());
          rgbImagePtr->frame_format =
              robosense::lidar::frame_format_t::FRAME_FORMAT_RGB24;
          rgbImagePtr->data_bytes = rgb_image_data_size;
          rgbImagePtr->width = rgb_image_data_width;
          rgbImagePtr->height = rgb_image_data_height;
          rgbImagePtr->timestamp = frame->timestamp;
          rgbImagePtr->right_timestamp = imagePtr->right_timestamp;
          rgbImagePtr->camera_mode = frame->camera_mode;
          rgbImagePtr->state = frame->state;

          rgbImagePtr->right_data =
              std::shared_ptr<uint8_t>(new uint8_t[rgbImagePtr->data_bytes],
                                       std::default_delete<uint8_t[]>());
          memcpy(rgbImagePtr->right_data.get(), rgb_data_buf,
                 rgbImagePtr->data_bytes);

          jpegRectifyImagePtr = rgbImagePtr;
        }
        std::lock_guard<std::mutex> lock(jpeg_rectify_right_mutex_);
        jpeg_rectify_right_queue_.push({jpegRectifyImagePtr, jpegTimestampPtr});
        jpeg_rectify_right_condition_.notify_one();
        break;
      }
      } // switch
    }   // if(enable_rectify_jpeg)
  }

  void
  depth_handle(const std::shared_ptr<PointCloudT<RsPointXYZIRT>> &frame,
               const robosense::device::RSTimestampItem::Ptr &timestampPtr) {
    int ret = 0;
    if (!enable_ac2_pointcloud_wave_split) {
      if (!enable_ros2_zero_copy) {
        auto cloud_msg = MAKE_SHARED_ROS_POINTCLOUD2;

        // 构造ROS/ROS2 消息
        frame->frame_id = point_frame_id;
        ret = robosense::convert::RSConvertManager::toRosPointCloud2Message(
            frame, cloud_msg);
        if (ret != 0) {
          RS_SPDLOG_ERROR("Convert To PointCloud2 Ros/Ros2 Message Failed: "
                          "point_frame_id = " +
                          point_frame_id + ", ret = " + std::to_string(ret) +
                          " !");
          return;
        }

        timestampPtr->process_timestamp_ns = RS_TIMESTAMP_NS;
        publisher_depth->publish(*cloud_msg);
        timestampPtr->publish_timestamp_ns = RS_TIMESTAMP_NS;
        if (timestamp_manager_ptr) {
          timestampPtr->channel_id =
              robosense::device::RS_CHANNEL_ID_TYPE::RS_CHANNEL_ID_POINTCLOUD;
          // 更新消息时间戳
          timestampPtr->message_timestamp_ns =
              robosense::convert::RSConvertManager::rosStampToNanoseconds(
                  cloud_msg->header.stamp);
          timestamp_manager_ptr->addTimestamp(timestampPtr);
        }
      }
#if defined(ROS2_FOUND)
      else {
        if (lidar_type == robosense::lidar::LidarType::RS_AC1) {
          auto loanedMsg = publisher_depth_loan->borrow_loaned_message();
          // 判断消息是否可用，可能出现获取消息失败导致消息不可用的情况
          if (!loanedMsg.is_valid()) {
            // 获取消息失败，丢弃该消息
            RS_SPDLOG_ERROR("Failed to get LoanMessage(AC1 PointCloud) !");
            return;
          }
          // 引用方式获取实际的消息
          auto &msg = loanedMsg.get();
          auto cloud_msg = &msg;

          frame->frame_id = point_frame_id;
          ret = robosense::convert::RSConvertManager::toZeroCopyCloudMessage(
              frame, cloud_msg);
          if (ret != 0) {
            RS_SPDLOG_ERROR(
                "Convert To robosense_msgs::msg::RsPointCloud1M Ros/Ros2 "
                "Message Failed: point_frame_id = " +
                point_frame_id + ", ret = " + std::to_string(ret) + " !");
            return;
          }

          timestampPtr->process_timestamp_ns = RS_TIMESTAMP_NS;
          const auto cloud_msg_header = cloud_msg->header;
          try {
            publisher_depth_loan->publish(std::move(loanedMsg));
          } catch (...) {
            RS_WARNING
                << "Publish AC1 ZeroCopy PointCloud Ros Message Failed !";
          }
          timestampPtr->publish_timestamp_ns = RS_TIMESTAMP_NS;
          if (timestamp_manager_ptr) {
            timestampPtr->channel_id =
                robosense::device::RS_CHANNEL_ID_TYPE::RS_CHANNEL_ID_POINTCLOUD;
            // 更新消息时间戳
            timestampPtr->message_timestamp_ns =
                robosense::convert::RSConvertManager::rosStampToNanoseconds(
                    cloud_msg_header.stamp);
            timestamp_manager_ptr->addTimestamp(timestampPtr);
          }
        } else if (lidar_type == robosense::lidar::LidarType::RS_AC2) {
          auto loanedMsg = publisher_depth_ac2_loan->borrow_loaned_message();
          // 判断消息是否可用，可能出现获取消息失败导致消息不可用的情况
          if (!loanedMsg.is_valid()) {
            // 获取消息失败，丢弃该消息
            RS_SPDLOG_ERROR("Failed to get LoanMessage(AC2 PointCloud) !");
            return;
          }
          // 引用方式获取实际的消息
          auto &msg = loanedMsg.get();
          auto cloud_msg = &msg;

          frame->frame_id = point_frame_id;
          ret = robosense::convert::RSConvertManager::toZeroCopyCloudMessage(
              frame, cloud_msg);
          if (ret != 0) {
            RS_SPDLOG_ERROR(
                "Convert To robosense_msgs::msg::RsPointCloud4M Ros/Ros2 "
                "Message Failed: point_frame_id = " +
                point_frame_id + ", ret = " + std::to_string(ret) + " !");
            return;
          }

          timestampPtr->process_timestamp_ns = RS_TIMESTAMP_NS;
          const auto cloud_msg_header = cloud_msg->header;
          try {
            publisher_depth_ac2_loan->publish(std::move(loanedMsg));
          } catch (...) {
            RS_WARNING
                << "Publish AC2 ZeroCopy PointCloud Ros Message Failed !";
          }
          timestampPtr->publish_timestamp_ns = RS_TIMESTAMP_NS;
          if (timestamp_manager_ptr) {
            timestampPtr->channel_id =
                robosense::device::RS_CHANNEL_ID_TYPE::RS_CHANNEL_ID_POINTCLOUD;
            // 更新消息时间戳
            timestampPtr->message_timestamp_ns =
                robosense::convert::RSConvertManager::rosStampToNanoseconds(
                    cloud_msg_header.stamp);
            timestamp_manager_ptr->addTimestamp(timestampPtr);
          }
        }
      } // if (!enable_ros2_zero_copy)
#endif  // defined(ROS2_FOUND)
    } else {
      if (ac2_wave1_pointcloud_ptr == nullptr) {
        ac2_wave1_pointcloud_ptr.reset(new PointCloudT<RsPointXYZIRT>());
      }
      if (ac2_wave2_pointcloud_ptr == nullptr) {
        ac2_wave2_pointcloud_ptr.reset(new PointCloudT<RsPointXYZIRT>());
      }

      const size_t point_cnt = frame->size();
      if (point_cnt % 2 != 0) {
        RS_SPDLOG_ERROR("AC2 PointCloud Point Cnt Not 2xTimes: point_cnt = " +
                        std::to_string(point_cnt) +
                        ", point_frame_id = " + point_frame_id);
        return;
      }

      ac2_wave1_pointcloud_ptr->resize(point_cnt / 2);
      ac2_wave1_pointcloud_ptr->frame_id = point_frame_id;

      ac2_wave2_pointcloud_ptr->resize(point_cnt / 2);
      ac2_wave2_pointcloud_ptr->frame_id = point_frame_id;

      int index_wave1 = 0;
      int index_wave2 = 0;
      for (size_t i = 0; i < point_cnt; ++i) {
        if (i % 2 == 0) {
          ac2_wave1_pointcloud_ptr->points[index_wave1] = frame->points[i];
          ++index_wave1;
        } else {
          ac2_wave2_pointcloud_ptr->points[index_wave2] = frame->points[i];
          ++index_wave2;
        }
      }

      if (!enable_ros2_zero_copy) {
        auto cloud_ac2_wave1_msg = MAKE_SHARED_ROS_POINTCLOUD2;
        auto cloud_ac2_wave2_msg = MAKE_SHARED_ROS_POINTCLOUD2;
        ret = robosense::convert::RSConvertManager::toRosPointCloud2Message(
            ac2_wave1_pointcloud_ptr, cloud_ac2_wave1_msg);
        if (ret != 0) {
          RS_SPDLOG_ERROR(
              "Convert To PointCloud2 Ros/Ros2 "
              "Message Failed(cloud_ac2_wave1_msg): point_frame_id = " +
              point_frame_id + ", ret = " + std::to_string(ret) + " !");
          return;
        }

        ret = robosense::convert::RSConvertManager::toRosPointCloud2Message(
            ac2_wave2_pointcloud_ptr, cloud_ac2_wave2_msg);
        if (ret != 0) {
          RS_SPDLOG_ERROR(
              "Convert To PointCloud2 Ros/Ros2 "
              "Message Failed(cloud_ac2_wave2_msg): point_frame_id = " +
              point_frame_id + ", ret = " + std::to_string(ret) + " !");
          return;
        }

        timestampPtr->process_timestamp_ns = RS_TIMESTAMP_NS;
        publisher_depth->publish(*cloud_ac2_wave1_msg);
        publisher_depth_ac2_wave2->publish(*cloud_ac2_wave2_msg);
        timestampPtr->publish_timestamp_ns = RS_TIMESTAMP_NS;
        if (timestamp_manager_ptr) {
          timestampPtr->channel_id =
              robosense::device::RS_CHANNEL_ID_TYPE::RS_CHANNEL_ID_POINTCLOUD;
          // 更新消息时间戳
          timestampPtr->message_timestamp_ns =
              robosense::convert::RSConvertManager::rosStampToNanoseconds(
                  cloud_ac2_wave1_msg->header.stamp);
          timestamp_manager_ptr->addTimestamp(timestampPtr);

          robosense::device::RSTimestampItem::Ptr copyItemPtr(
              new robosense::device::RSTimestampItem(*timestampPtr));
          copyItemPtr->channel_id = robosense::device::RS_CHANNEL_ID_TYPE::
              RS_CHANNEL_ID_POINTCLOUD_AC2_WAVE2;
          // 更新消息时间戳
          copyItemPtr->message_timestamp_ns =
              robosense::convert::RSConvertManager::rosStampToNanoseconds(
                  cloud_ac2_wave2_msg->header.stamp);
          timestamp_manager_ptr->addTimestamp(copyItemPtr);
        }
      }
#if defined(ROS2_FOUND)
      else {
        auto loanedWave1Msg = publisher_depth_ac2_loan->borrow_loaned_message();
        // 判断消息是否可用，可能出现获取消息失败导致消息不可用的情况
        if (!loanedWave1Msg.is_valid()) {
          // 获取消息失败，丢弃该消息
          RS_SPDLOG_ERROR("Failed to get LoanMessage(cloud_ac2_wave1_msg) !");
          return;
        }
        // 引用方式获取实际的消息
        auto &wave1_msg = loanedWave1Msg.get();
        auto cloud_ac2_wave1_msg = &wave1_msg;

        auto loanedWave2Msg =
            publisher_depth_ac2_wave2_loan->borrow_loaned_message();
        // 判断消息是否可用，可能出现获取消息失败导致消息不可用的情况
        if (!loanedWave2Msg.is_valid()) {
          // 获取消息失败，丢弃该消息
          RS_SPDLOG_ERROR("Failed to get LoanMessage(cloud_ac2_wave2_msg) !");
          return;
        }
        // 引用方式获取实际的消息
        auto &wave2_msg = loanedWave2Msg.get();
        auto cloud_ac2_wave2_msg = &wave2_msg;

        ret = robosense::convert::RSConvertManager::toZeroCopyCloudMessage(
            ac2_wave1_pointcloud_ptr, cloud_ac2_wave1_msg);
        if (ret != 0) {
          RS_SPDLOG_ERROR(
              "Convert To robosense_msgs::msg::RsPointCloud4M Ros/Ros2 "
              "Message Failed(cloud_ac2_wave1_msg): point_frame_id = " +
              point_frame_id + ", ret = " + std::to_string(ret) + " !");
          return;
        }

        ret = robosense::convert::RSConvertManager::toZeroCopyCloudMessage(
            ac2_wave2_pointcloud_ptr, cloud_ac2_wave2_msg);
        if (ret != 0) {
          RS_SPDLOG_ERROR(
              "Convert To robosense_msgs::msg::RsPointCloud4M Ros/Ros2 "
              "Message Failed(cloud_ac2_wave2_msg): point_frame_id = " +
              point_frame_id + ", ret = " + std::to_string(ret) + " !");
          return;
        }

        const auto cloud_wave1_header = cloud_ac2_wave1_msg->header;
        const auto cloud_wave2_header = cloud_ac2_wave2_msg->header;
        timestampPtr->process_timestamp_ns = RS_TIMESTAMP_NS;
        try {
          publisher_depth_ac2_loan->publish(std::move(loanedWave1Msg));
        } catch (...) {
          RS_SPDLOG_WARN("Publish AC2 Wave1 ZeroCopy Ros Message Failed !");
        }
        try {
          publisher_depth_ac2_wave2_loan->publish(std::move(loanedWave2Msg));
        } catch (...) {
          RS_SPDLOG_WARN("Publish AC2 Wave2 ZeroCopy Ros Message Failed !");
        }
        timestampPtr->publish_timestamp_ns = RS_TIMESTAMP_NS;
        if (timestamp_manager_ptr) {
          timestampPtr->channel_id =
              robosense::device::RS_CHANNEL_ID_TYPE::RS_CHANNEL_ID_POINTCLOUD;
          // 更新消息时间戳
          timestampPtr->message_timestamp_ns =
              robosense::convert::RSConvertManager::rosStampToNanoseconds(
                  cloud_wave1_header.stamp);
          timestamp_manager_ptr->addTimestamp(timestampPtr);

          robosense::device::RSTimestampItem::Ptr copyItemPtr(
              new robosense::device::RSTimestampItem(*timestampPtr));
          copyItemPtr->channel_id = robosense::device::RS_CHANNEL_ID_TYPE::
              RS_CHANNEL_ID_POINTCLOUD_AC2_WAVE2;
          // 更新消息时间戳
          copyItemPtr->message_timestamp_ns =
              robosense::convert::RSConvertManager::rosStampToNanoseconds(
                  cloud_wave2_header.stamp);
          timestamp_manager_ptr->addTimestamp(copyItemPtr);
        }
      } // if(!enable_ros2_zero_copy)
#endif  // defined(ROS2_FOUND)
    }   // if (!enable_ac2_pointcloud_wave_split)
  }

  void imu_handle(const std::shared_ptr<robosense::lidar::ImuData> &msgPtr,
                  const robosense::device::RSTimestampItem::Ptr &timestampPtr) {
    auto imu_msg = MAKE_SHARED_ROS_IMU;
    robosense::convert::RSConvertManager::toRosImuMessage(msgPtr, imu_frame_id,
                                                          imu_msg);
    timestampPtr->process_timestamp_ns = RS_TIMESTAMP_NS;
    publisher_imu->publish(*imu_msg);
    timestampPtr->publish_timestamp_ns = RS_TIMESTAMP_NS;
    if (timestamp_manager_ptr) {
      timestampPtr->channel_id =
          robosense::device::RS_CHANNEL_ID_TYPE::RS_CHANNEL_ID_IMU;
      // 更新消息时间戳
      timestampPtr->message_timestamp_ns =
          robosense::convert::RSConvertManager::rosStampToNanoseconds(
              imu_msg->header.stamp);
      timestamp_manager_ptr->addTimestamp(timestampPtr);
    }
  }

  void logError(const std::string &message) {
#if defined(ROS_FOUND)
    ROS_ERROR("%s", message.c_str());
#elif defined(ROS2_FOUND)
    RCLCPP_ERROR(this->get_logger(), message.c_str());
#endif // ROS_ROS2_FOUND
  }

  void logWarn(const std::string &message) {
#if defined(ROS_FOUND)
    ROS_WARN("%s", message.c_str());
#elif defined(ROS2_FOUND)
    RCLCPP_WARN(this->get_logger(), message.c_str());
#endif // ROS_ROS2_FOUND
  }

  void logInfo(const std::string &message) {
#if defined(ROS_FOUND)
    ROS_INFO("%s", message.c_str());
#elif defined(ROS2_FOUND)
    RCLCPP_INFO(this->get_logger(), message.c_str());
#endif // ROS_ROS2_FOUND
  }

  void shutdown() {
#if defined(ROS_FOUND)
    ros::shutdown();
#elif defined(ROS2_FOUND)
    rclcpp::shutdown();
#endif // ROS_ROS2_FOUND
  }

#ifdef RK3588
  int nv12_to_rgb_rk3588(uint8_t *data, uint32_t data_bytes, uint32_t width,
                         uint32_t height, uint8_t *rgb_buf) {
    int ret = 0;
    int src_format;
    int dst_format;
    char *src_buf, *dst_buf;
    int dst_buf_size;

    rga_buffer_t src_img, dst_img;
    rga_buffer_handle_t src_handle, dst_handle;

    memset(&src_img, 0, sizeof(src_img));
    memset(&dst_img, 0, sizeof(dst_img));

    src_format = RK_FORMAT_YCbCr_420_SP;
    dst_format = RK_FORMAT_RGB_888;

    dst_buf_size = width * height * get_bpp_from_format(RK_FORMAT_RGB_888);

    dst_buf = (char *)rgb_buf.data();

    memset(dst_buf, 0x80, dst_buf_size);

    src_handle = importbuffer_virtualaddr(data, data_bytes);
    dst_handle = importbuffer_virtualaddr(dst_buf, dst_buf_size);
    if (src_handle == 0 || dst_handle == 0) {
      printf("%s importbuffer failed!\n", __func__);
      if (src_handle)
        releasebuffer_handle(src_handle);
      if (dst_handle)
        releasebuffer_handle(dst_handle);
      return -1;
    }

    src_img = wrapbuffer_handle(src_handle, width, height, src_format);
    dst_img = wrapbuffer_handle(dst_handle, width, height, dst_format);

    ret = imcheck(src_img, dst_img, {}, {});
    if (IM_STATUS_NOERROR != ret) {
      printf("%s %d, check error! %s", __func__, __LINE__,
             imStrError((IM_STATUS)ret));
      if (src_handle)
        releasebuffer_handle(src_handle);
      if (dst_handle)
        releasebuffer_handle(dst_handle);
      return -2;
    }

    ret = imcvtcolor(src_img, dst_img, src_format, dst_format);
    if (ret != IM_STATUS_SUCCESS) {
      printf("%s imcvtcolor running failed, %s\n", __func__,
             imStrError((IM_STATUS)ret));
      return -3;
    }

    if (src_handle)
      releasebuffer_handle(src_handle);
    if (dst_handle)
      releasebuffer_handle(dst_handle);

    return 0;
  }
#endif // RK3588

  void rgbProcessWorkThread() {
    while (is_rgb_running_) {
      std::pair<std::shared_ptr<robosense::lidar::ImageData>,
                robosense::device::RSTimestampItem::Ptr>
          frame;
      {
        std::unique_lock<std::mutex> lock(rgb_mutex_);
        rgb_condition_.wait(
            lock, [this] { return !rgb_queue_.empty() || !is_rgb_running_; });

        if (!is_rgb_running_) {
          break;
        }

        frame = rgb_queue_.front();
        rgb_queue_.pop();
      }

      if (frame.first) {
        rgb_handle(RS_IMAGE_SOURCE_TYPE::RS_IMAGE_SOURCE_AC1, frame.first,
                   frame.second);
      }
    }
    RS_SPDLOG_INFO("AC1 Rgb Work Thread Exit !");
  }

  void rgbLeftProcessWorkThread() {
    while (is_rgb_left_running_) {
      std::pair<std::shared_ptr<robosense::lidar::ImageData>,
                robosense::device::RSTimestampItem::Ptr>
          frame;
      {
        std::unique_lock<std::mutex> lock(rgb_left_mutex_);
        rgb_left_condition_.wait(lock, [this] {
          return !rgb_left_queue_.empty() || !is_rgb_left_running_;
        });

        if (!is_rgb_left_running_) {
          break;
        }

        frame = rgb_left_queue_.front();
        rgb_left_queue_.pop();
      }

      if (frame.first) {
        rgb_handle(RS_IMAGE_SOURCE_TYPE::RS_IMAGE_SOURCE_AC2_LEFT, frame.first,
                   frame.second);
      }
    }
    RS_SPDLOG_INFO("AC2 Rgb Left Work Thread Exit !");
  }

  void rgbRightProcessWorkThread() {
    while (is_rgb_right_running_) {
      std::pair<std::shared_ptr<robosense::lidar::ImageData>,
                robosense::device::RSTimestampItem::Ptr>
          frame;
      {
        std::unique_lock<std::mutex> lock(rgb_right_mutex_);
        rgb_right_condition_.wait(lock, [this] {
          return !rgb_right_queue_.empty() || !is_rgb_right_running_;
        });

        if (!is_rgb_right_running_) {
          break;
        }

        frame = rgb_right_queue_.front();
        rgb_right_queue_.pop();
      }

      if (frame.first) {
        rgb_handle(RS_IMAGE_SOURCE_TYPE::RS_IMAGE_SOURCE_AC2_RIGHT, frame.first,
                   frame.second);
      }
    }
    RS_SPDLOG_INFO("AC2 Rgb Right Work Thread Exit !");
  }

  void rgbRectifyProcessWorkThread() {
    while (is_rgb_rectify_running_) {
      std::pair<std::shared_ptr<robosense::lidar::ImageData>,
                robosense::device::RSTimestampItem::Ptr>
          frame;
      {
        std::unique_lock<std::mutex> lock(rgb_rectify_mutex_);
        rgb_rectify_condition_.wait(lock, [this] {
          return !rgb_rectify_queue_.empty() || !is_rgb_rectify_running_;
        });

        if (!is_rgb_rectify_running_) {
          break;
        }

        frame = rgb_rectify_queue_.front();
        rgb_rectify_queue_.pop();
      }

      if (frame.first) {
        rgb_rectify_handle(RS_IMAGE_SOURCE_TYPE::RS_IMAGE_SOURCE_AC1,
                           frame.first, frame.second);
      }
    }
    RS_SPDLOG_INFO("AC1 Rgb Rectify Work Thread Exit !");
  }

  void rgbRectifyLeftProcessWorkThread() {
    while (is_rgb_rectify_left_running_) {
      std::pair<std::shared_ptr<robosense::lidar::ImageData>,
                robosense::device::RSTimestampItem::Ptr>
          frame;
      {
        std::unique_lock<std::mutex> lock(rgb_rectify_left_mutex_);
        rgb_rectify_left_condition_.wait(lock, [this] {
          return !rgb_rectify_left_queue_.empty() ||
                 !is_rgb_rectify_left_running_;
        });

        if (!is_rgb_rectify_left_running_) {
          break;
        }

        frame = rgb_rectify_left_queue_.front();
        rgb_rectify_left_queue_.pop();
      }

      if (frame.first) {
        rgb_rectify_handle(RS_IMAGE_SOURCE_TYPE::RS_IMAGE_SOURCE_AC2_LEFT,
                           frame.first, frame.second);
      }
    }
    RS_SPDLOG_INFO("AC2 Rgb Rectify Left Work Thread Exit !");
  }

  void rgbRectifyRightProcessWorkThread() {
    while (is_rgb_rectify_right_running_) {
      std::pair<std::shared_ptr<robosense::lidar::ImageData>,
                robosense::device::RSTimestampItem::Ptr>
          frame;
      {
        std::unique_lock<std::mutex> lock(rgb_rectify_right_mutex_);
        rgb_rectify_right_condition_.wait(lock, [this] {
          return !rgb_rectify_right_queue_.empty() ||
                 !is_rgb_rectify_right_running_;
        });

        if (!is_rgb_rectify_right_running_) {
          break;
        }

        frame = rgb_rectify_right_queue_.front();
        rgb_rectify_right_queue_.pop();
      }

      if (frame.first) {
        rgb_rectify_handle(RS_IMAGE_SOURCE_TYPE::RS_IMAGE_SOURCE_AC2_RIGHT,
                           frame.first, frame.second);
      }
    }
    RS_SPDLOG_INFO("AC2 Rgb Rectify Right Work Thread Exit !");
  }

  void jpegProcessWorkThread() {
    while (is_jpeg_running_) {
      std::pair<std::shared_ptr<robosense::lidar::ImageData>,
                robosense::device::RSTimestampItem::Ptr>
          frame;
      {
        std::unique_lock<std::mutex> lock(jpeg_mutex_);
        jpeg_condition_.wait(
            lock, [this] { return !jpeg_queue_.empty() || !is_jpeg_running_; });

        if (!is_jpeg_running_) {
          break;
        }

        frame = jpeg_queue_.front();
        jpeg_queue_.pop();
      }

      if (frame.first) {
        jpeg_handle(RS_IMAGE_SOURCE_TYPE::RS_IMAGE_SOURCE_AC1, frame.first,
                    frame.second);
      }
    }
    RS_SPDLOG_INFO("AC1 Jpeg Work Thread Exit !");
  }

  void jpegLeftProcessWorkThread() {
    while (is_jpeg_left_running_) {
      std::pair<std::shared_ptr<robosense::lidar::ImageData>,
                robosense::device::RSTimestampItem::Ptr>
          frame;
      {
        std::unique_lock<std::mutex> lock(jpeg_left_mutex_);
        jpeg_left_condition_.wait(lock, [this] {
          return !jpeg_left_queue_.empty() || !is_jpeg_left_running_;
        });

        if (!is_jpeg_left_running_) {
          break;
        }

        frame = jpeg_left_queue_.front();
        jpeg_left_queue_.pop();
      }

      if (frame.first) {
        jpeg_handle(RS_IMAGE_SOURCE_TYPE::RS_IMAGE_SOURCE_AC2_LEFT, frame.first,
                    frame.second);
      }
    }
    RS_SPDLOG_INFO("AC2 Jpeg Left Work Thread Exit !");
  }

  void jpegRightProcessWorkThread() {
    while (is_jpeg_right_running_) {
      std::pair<std::shared_ptr<robosense::lidar::ImageData>,
                robosense::device::RSTimestampItem::Ptr>
          frame;
      {
        std::unique_lock<std::mutex> lock(jpeg_right_mutex_);
        jpeg_right_condition_.wait(lock, [this] {
          return !jpeg_right_queue_.empty() || !is_jpeg_right_running_;
        });

        if (!is_jpeg_right_running_) {
          break;
        }

        frame = jpeg_right_queue_.front();
        jpeg_right_queue_.pop();
      }

      if (frame.first) {
        jpeg_handle(RS_IMAGE_SOURCE_TYPE::RS_IMAGE_SOURCE_AC2_RIGHT,
                    frame.first, frame.second);
      }
    }
    RS_SPDLOG_INFO("AC2 Jpeg Right Work Thread Exit !");
  }

  void jpegRectifyProcessWorkThread() {
    while (is_jpeg_rectify_running_) {
      std::pair<std::shared_ptr<robosense::lidar::ImageData>,
                robosense::device::RSTimestampItem::Ptr>
          frame;
      {
        std::unique_lock<std::mutex> lock(jpeg_rectify_mutex_);
        jpeg_rectify_condition_.wait(lock, [this] {
          return !jpeg_rectify_queue_.empty() || !is_jpeg_rectify_running_;
        });

        if (!is_jpeg_rectify_running_) {
          break;
        }

        frame = jpeg_rectify_queue_.front();
        jpeg_rectify_queue_.pop();
      }

      if (frame.first) {
        jpeg_rectify_handle(RS_IMAGE_SOURCE_TYPE::RS_IMAGE_SOURCE_AC1,
                            frame.first, frame.second);
      }
    }
    RS_SPDLOG_INFO("AC1 Jpeg Rectify Work Thread Exit !");
  }

  void jpegRectifyLeftProcessWorkThread() {
    while (is_jpeg_rectify_left_running_) {
      std::pair<std::shared_ptr<robosense::lidar::ImageData>,
                robosense::device::RSTimestampItem::Ptr>
          frame;
      {
        std::unique_lock<std::mutex> lock(jpeg_rectify_left_mutex_);
        jpeg_rectify_left_condition_.wait(lock, [this] {
          return !jpeg_rectify_left_queue_.empty() ||
                 !is_jpeg_rectify_left_running_;
        });

        if (!is_jpeg_rectify_left_running_) {
          break;
        }

        frame = jpeg_rectify_left_queue_.front();
        jpeg_rectify_left_queue_.pop();
      }

      if (frame.first) {
        jpeg_rectify_handle(RS_IMAGE_SOURCE_TYPE::RS_IMAGE_SOURCE_AC2_LEFT,
                            frame.first, frame.second);
      }
    }
    RS_SPDLOG_INFO("AC2 Jpeg Rectify Left Work Thread Exit !");
  }

  void jpegRectifyRightProcessWorkThread() {
    while (is_jpeg_rectify_right_running_) {
      std::pair<std::shared_ptr<robosense::lidar::ImageData>,
                robosense::device::RSTimestampItem::Ptr>
          frame;
      {
        std::unique_lock<std::mutex> lock(jpeg_rectify_right_mutex_);
        jpeg_rectify_right_condition_.wait(lock, [this] {
          return !jpeg_rectify_right_queue_.empty() ||
                 !is_jpeg_rectify_right_running_;
        });

        if (!is_jpeg_rectify_right_running_) {
          break;
        }

        frame = jpeg_rectify_right_queue_.front();
        jpeg_rectify_right_queue_.pop();
      }

      if (frame.first) {
        jpeg_rectify_handle(RS_IMAGE_SOURCE_TYPE::RS_IMAGE_SOURCE_AC2_RIGHT,
                            frame.first, frame.second);
      }
    }
    RS_SPDLOG_INFO("AC2 Jpeg Rectify Right Work Thread Exit !");
  }

  void deviceInfoProcessWorkThread() {
    while (is_device_info_running_) {
      std::this_thread::sleep_for(std::chrono::seconds(1));
      {
        std::lock_guard<std::mutex> lg(current_device_uuid_mtx);
        if (!current_device_info_valid) {
          if (initCameraInfoAndDeviceCalibInfo(current_device_uuid)) {
            RS_SPDLOG_WARN("Current Device uuid = " + current_device_uuid +
                           " Not Get Valid Device Info !");
            continue;
          }
        }
      }

      // 构造消息&发布
      ROS_RSACDEVICECALIB msg = current_device_calib_msg;
      msg.header.frame_id = "/device_calib_info";
#if defined(ROS_FOUND)
      msg.header.stamp = ros::Time::now();
#elif defined(ROS2_FOUND)
      msg.header.stamp = this->now();
#endif // ROS_ROS2_FOUND
      publisher_device_calib_info->publish(msg);
    }
    RS_SPDLOG_INFO("Device Info Work Thread Exist !");
  }

  int loadCameraCalibInfoFromDevice(const std::string &uuid) {
    current_device_info_ready = false;
    bool isSuccess =
        device_manager_ptr->getDeviceInfo(uuid, current_device_info);
    if (isSuccess) {
      if (robosense::calib::RSCalibManager::parserDeviceCalibInfo(
              uuid, current_device_info, current_device_calib_msg)) {
        return -1;
      } else {
        current_device_info_ready = true;
        return 0;
      }
    } else {
      return -2;
    }
  }

  int initDeviceCalibInfoFromDevice() {
    if (current_device_info_ready) {
      publisher_device_calib_info = create_ros_publisher<ROS_RSACDEVICECALIB>(
          ac_device_calib_info_topic_name, 3);
    } else {
      RS_SPDLOG_WARN("Can Not Parser Device Calib Info From Device !");
      return -1;
    }

    return 0;
  }

  int initCameraInfoFromDevice() {
    if (current_device_info_ready) {
      if (lidar_type == robosense::lidar::LidarType::RS_AC1) {
        const auto &camerainfo_ptr =
            robosense::calib::RSCalibManager::parserAC1CameraInfo(
                current_device_calib_msg);
        if (camerainfo_ptr) {
          publisher_camera_info =
              create_ros_publisher<ROS_CAMERAINFO>(camera_info_topic_name, 10);
          camera_info_ptr = camerainfo_ptr;
        } else {
          camera_info_ptr.reset();
          RS_SPDLOG_WARN("AC1 Parser Image Calibration Failed !");
          return -1;
        }

        if (enable_rectify && camera_info_ptr) {
          int ret = robosense::calib::RSCalibManager::parserImageRectifyMap(
              camera_info_ptr, camera_rectify_map1, camera_rectify_map2);
          if (ret == 0) {
            camera_rectify_map_valid = true;
          } else {
            camera_rectify_map_valid = false;
            RS_WARNING
                << "AC1 Create Rectify Map From Image Calibration Failed !";
          }
        }
      } else if (lidar_type == robosense::lidar::LidarType::RS_AC2) {
        const auto &left_right_camera_info_ptr =
            robosense::calib::RSCalibManager::parserAC2CameraInfo(
                current_device_calib_msg);
        if (left_right_camera_info_ptr.first &&
            left_right_camera_info_ptr.second) {
          if (enable_ac2_left_image_send) {
            publisher_left_camera_info = create_ros_publisher<ROS_CAMERAINFO>(
                camera_info_left_topic_name, 10);
            left_camera_info_ptr = left_right_camera_info_ptr.first;
          }
          if (enable_ac2_right_image_send) {
            publisher_right_camera_info = create_ros_publisher<ROS_CAMERAINFO>(
                camera_info_right_topic_name, 10);
            right_camera_info_ptr = left_right_camera_info_ptr.second;
          }
        } else {
          left_camera_info_ptr.reset();
          right_camera_info_ptr.reset();
          RS_SPDLOG_WARN("AC2 Parser Image Calibration Failed !");
          return -2;
        }

        if (enable_rectify && left_camera_info_ptr) {
          int ret = robosense::calib::RSCalibManager::parserImageRectifyMap(
              left_camera_info_ptr, left_camera_rectify_map1,
              left_camera_rectify_map2);
          if (ret == 0) {
            left_camera_rectify_map_valid = true;
          } else {
            left_camera_rectify_map_valid = false;
            RS_SPDLOG_WARN("AC2 Create Left Rectify Map From Image Calibration "
                           "Failed !");
          }
        }
        if (enable_rectify && right_camera_info_ptr) {
          int ret = robosense::calib::RSCalibManager::parserImageRectifyMap(
              right_camera_info_ptr, right_camera_rectify_map1,
              right_camera_rectify_map2);
          if (ret == 0) {
            right_camera_rectify_map_valid = true;
          } else {
            right_camera_rectify_map_valid = false;
            RS_SPDLOG_WARN(
                "AC2 Create Right Rectify Map From Image Calibration "
                "Failed !");
          }
        }
      }
    } else {
      RS_SPDLOG_WARN("Can Not Parser Camera Info From Device !");
      return -3;
    }

    return 0;
  }

  int initDeviceCalibInfoFromFiles() {
    if (robosense::calib::RSCalibManager::isFileExist(device_calib_file_path)) {
      if (lidar_type == robosense::lidar::LidarType::RS_AC1 &&
          robosense::calib::RSCalibManager::checkImageCalibFileIsAC1(
              device_calib_file_path)) {
        int ret = robosense::calib::RSCalibManager::parserAC1DeviceCalibInfo(
            device_calib_file_path, current_device_calib_msg);
        if (ret != 0) {
          RS_SPDLOG_WARN("Parser AC1 Device Calib Info From Files: " +
                         device_calib_file_path + " Failed !");
          return -1;
        }
      } else if (lidar_type == robosense::lidar::LidarType::RS_AC2 &&
                 robosense::calib::RSCalibManager::checkImageCalibFileIsAC2(
                     device_calib_file_path)) {
        int ret = robosense::calib::RSCalibManager::parserAC2DeviceCalibInfo(
            device_calib_file_path, current_device_calib_msg);
        if (ret != 0) {
          RS_SPDLOG_WARN("Parser AC2 Device Calib Info From Files: " +
                         device_calib_file_path + " Failed !");
          return -2;
        }
      }
    } else {
      RS_SPDLOG_WARN("Image Calibration File: " + device_calib_file_path +
                     " Not Exist !");
      return -3;
    }

    publisher_device_calib_info = create_ros_publisher<ROS_RSACDEVICECALIB>(
        ac_device_calib_info_topic_name, 3);

    return 0;
  }

  int initCameraInfoFromFiles() {
    if (robosense::calib::RSCalibManager::isFileExist(device_calib_file_path)) {
      if (lidar_type == robosense::lidar::LidarType::RS_AC1 &&
          robosense::calib::RSCalibManager::checkImageCalibFileIsAC1(
              device_calib_file_path)) {
        if (enable_ac1_image_send) {
          const auto &camerainfo_ptr =
              robosense::calib::RSCalibManager::parserAC1CameraInfo(
                  device_calib_file_path);
          if (camerainfo_ptr) {
            publisher_camera_info = create_ros_publisher<ROS_CAMERAINFO>(
                camera_info_topic_name, 10);
            camera_info_ptr = camerainfo_ptr;
          } else {
            camera_info_ptr.reset();
            RS_SPDLOG_WARN("AC1 Parser Image Calibration Failed !");
            return -1;
          }

          if (enable_rectify && camera_info_ptr) {
            int ret = robosense::calib::RSCalibManager::parserImageRectifyMap(
                camera_info_ptr, camera_rectify_map1, camera_rectify_map2);
            if (ret == 0) {
              camera_rectify_map_valid = true;
            } else {
              camera_rectify_map_valid = false;
              RS_SPDLOG_WARN(
                  "AC1 Create Rectify Map From Image Calibration Failed !");
            }
          }
        }
      } else if (lidar_type == robosense::lidar::LidarType::RS_AC2 &&
                 robosense::calib::RSCalibManager::checkImageCalibFileIsAC2(
                     device_calib_file_path)) {
        const auto &left_right_camera_info_ptr =
            robosense::calib::RSCalibManager::parserAC2CameraInfo(
                device_calib_file_path);
        if (left_right_camera_info_ptr.first &&
            left_right_camera_info_ptr.second) {
          if (enable_ac2_left_image_send) {
            publisher_left_camera_info = create_ros_publisher<ROS_CAMERAINFO>(
                camera_info_left_topic_name, 10);
            left_camera_info_ptr = left_right_camera_info_ptr.first;
          }
          if (enable_ac2_right_image_send) {
            publisher_right_camera_info = create_ros_publisher<ROS_CAMERAINFO>(
                camera_info_right_topic_name, 10);
            right_camera_info_ptr = left_right_camera_info_ptr.second;
          }
        } else {
          left_camera_info_ptr.reset();
          right_camera_info_ptr.reset();
          RS_SPDLOG_WARN("AC2 Parser Image Calibration Failed !");
          return -2;
        }

        if (enable_rectify && left_camera_info_ptr) {
          int ret = robosense::calib::RSCalibManager::parserImageRectifyMap(
              left_camera_info_ptr, left_camera_rectify_map1,
              left_camera_rectify_map2);
          if (ret == 0) {
            left_camera_rectify_map_valid = true;
          } else {
            left_camera_rectify_map_valid = false;
            RS_SPDLOG_WARN("AC2 Create Left Rectify Map From Image Calibration "
                           "Failed !");
          }
        }
        if (enable_rectify && right_camera_info_ptr) {
          int ret = robosense::calib::RSCalibManager::parserImageRectifyMap(
              right_camera_info_ptr, right_camera_rectify_map1,
              right_camera_rectify_map2);
          if (ret == 0) {
            right_camera_rectify_map_valid = true;
          } else {
            right_camera_rectify_map_valid = false;
            RS_SPDLOG_WARN(
                "AC2 Create Right Rectify Map From Image Calibration "
                "Failed !");
          }
        }
      }
    } else {
      RS_SPDLOG_WARN("Image Calibration File: " + device_calib_file_path +
                     " Not Exist !");
      return -3;
    }

    return 0;
  }

  int initCameraInfoAndDeviceCalibInfo(const std::string &uuid) {
    // Step1: Device 载入Camera Calib Info
    loadCameraCalibInfoFromDevice(uuid);

    // Step2: 载入相关配置信息
    bool isSuccess = false;
    if (enable_device_calib_info_from_device_pripority) {
      // std::cout << "run here 1" << std::endl;
      int ret1 = initCameraInfoFromDevice();
      // std::cout << "run here 2" << std::endl;
      int ret2 = initDeviceCalibInfoFromDevice();
      isSuccess = (ret1 == 0) && (ret2 == 0);

      if (isSuccess) {
        RS_SPDLOG_INFO("Device priority(1): Load Device Calib Info From Device "
                       "Successed !");
      } else {
        RS_SPDLOG_WARN("Device priority(1): ret1 = " + std::to_string(ret1) +
                       ", ret2 = " + std::to_string(ret2) +
                       ": Load Device Calib Info From Device Failed !");
      }

      if (!isSuccess) {
        // std::cout << "run here 3" << std::endl;
        int ret1 = initCameraInfoFromFiles();
        // std::cout << "run here 4" << std::endl;
        int ret2 = initDeviceCalibInfoFromFiles();
        isSuccess = (ret1 == 0) && (ret2 == 0);

        if (isSuccess) {
          RS_SPDLOG_INFO(
              "Device priority(2): Load Device Calib Info From Files "
              "Successed !");
        } else {
          RS_SPDLOG_WARN("Device priority(2): ret1 = " + std::to_string(ret1) +
                         ", ret2 = " + std::to_string(ret2) +
                         ": Load Device Calib Info From Files Failed !");
        }
      }
    } else {
      // std::cout << "run here 5" << std::endl;
      int ret1 = initCameraInfoFromFiles();
      // std::cout << "run here 6" << std::endl;
      int ret2 = initDeviceCalibInfoFromFiles();
      isSuccess = (ret1 == 0) && (ret2 == 0);

      if (isSuccess) {
        RS_SPDLOG_INFO("Files priority(1): Load Device Calib Info From Files "
                       "Successed !");
      } else {
        RS_SPDLOG_WARN("Files priority(1): ret1 = " + std::to_string(ret1) +
                       ", ret2 = " + std::to_string(ret2) +
                       ": Load Device Calib Info From Files Failed !");
      }

      if (!isSuccess) {
        // std::cout << "run here 7" << std::endl;
        int ret1 = initCameraInfoFromDevice();
        // std::cout << "run here 8" << std::endl;
        int ret2 = initDeviceCalibInfoFromDevice();
        isSuccess = (ret1 == 0) && (ret2 == 0);

        if (isSuccess) {
          RS_SPDLOG_INFO(
              "Files priority(2): Load Device Calib Info From Device "
              "Successed ! ");
        } else {
          RS_SPDLOG_WARN("Files priority(2): ret1 = " + std::to_string(ret1) +
                         ", ret2 = " + std::to_string(ret2) +
                         ": Load Device Calib Info From Device Failed !");
        }
      }
    }

    // 更新状态
    current_device_info_valid = isSuccess;

    return (isSuccess ? 0 : -1);
  }

private:
  // RGB 处理线程
  std::shared_ptr<std::thread> rgb_thread_ptr;
  std::mutex rgb_mutex_;
  std::condition_variable rgb_condition_;
  std::queue<std::pair<std::shared_ptr<robosense::lidar::ImageData>,
                       robosense::device::RSTimestampItem::Ptr>>
      rgb_queue_;
  bool is_rgb_running_ = false;

  std::shared_ptr<std::thread> rgb_left_thread_ptr;
  std::mutex rgb_left_mutex_;
  std::condition_variable rgb_left_condition_;
  std::queue<std::pair<std::shared_ptr<robosense::lidar::ImageData>,
                       robosense::device::RSTimestampItem::Ptr>>
      rgb_left_queue_;
  bool is_rgb_left_running_ = false;

  std::shared_ptr<std::thread> rgb_right_thread_ptr;
  std::mutex rgb_right_mutex_;
  std::condition_variable rgb_right_condition_;
  std::queue<std::pair<std::shared_ptr<robosense::lidar::ImageData>,
                       robosense::device::RSTimestampItem::Ptr>>
      rgb_right_queue_;
  bool is_rgb_right_running_ = false;

  // Rectify RGB 处理线程
  std::shared_ptr<std::thread> rgb_rectify_thread_ptr;
  std::mutex rgb_rectify_mutex_;
  std::condition_variable rgb_rectify_condition_;
  std::queue<std::pair<std::shared_ptr<robosense::lidar::ImageData>,
                       robosense::device::RSTimestampItem::Ptr>>
      rgb_rectify_queue_;
  bool is_rgb_rectify_running_ = false;

  std::shared_ptr<std::thread> rgb_rectify_left_thread_ptr;
  std::mutex rgb_rectify_left_mutex_;
  std::condition_variable rgb_rectify_left_condition_;
  std::queue<std::pair<std::shared_ptr<robosense::lidar::ImageData>,
                       robosense::device::RSTimestampItem::Ptr>>
      rgb_rectify_left_queue_;
  bool is_rgb_rectify_left_running_ = false;

  std::shared_ptr<std::thread> rgb_rectify_right_thread_ptr;
  std::mutex rgb_rectify_right_mutex_;
  std::condition_variable rgb_rectify_right_condition_;
  std::queue<std::pair<std::shared_ptr<robosense::lidar::ImageData>,
                       robosense::device::RSTimestampItem::Ptr>>
      rgb_rectify_right_queue_;
  bool is_rgb_rectify_right_running_ = false;

private:
  // JPEG 处理线程
  std::shared_ptr<std::thread> jpeg_thread_ptr;
  std::mutex jpeg_mutex_;
  std::condition_variable jpeg_condition_;
  std::queue<std::pair<std::shared_ptr<robosense::lidar::ImageData>,
                       robosense::device::RSTimestampItem::Ptr>>
      jpeg_queue_;
  bool is_jpeg_running_ = false;

  std::shared_ptr<std::thread> jpeg_left_thread_ptr;
  std::mutex jpeg_left_mutex_;
  std::condition_variable jpeg_left_condition_;
  std::queue<std::pair<std::shared_ptr<robosense::lidar::ImageData>,
                       robosense::device::RSTimestampItem::Ptr>>
      jpeg_left_queue_;
  bool is_jpeg_left_running_ = false;

  std::shared_ptr<std::thread> jpeg_right_thread_ptr;
  std::mutex jpeg_right_mutex_;
  std::condition_variable jpeg_right_condition_;
  std::queue<std::pair<std::shared_ptr<robosense::lidar::ImageData>,
                       robosense::device::RSTimestampItem::Ptr>>
      jpeg_right_queue_;
  bool is_jpeg_right_running_ = false;

  // Rectify Jpeg 处理线程
  std::shared_ptr<std::thread> jpeg_rectify_thread_ptr;
  std::mutex jpeg_rectify_mutex_;
  std::condition_variable jpeg_rectify_condition_;
  std::queue<std::pair<std::shared_ptr<robosense::lidar::ImageData>,
                       robosense::device::RSTimestampItem::Ptr>>
      jpeg_rectify_queue_;
  bool is_jpeg_rectify_running_ = false;

  std::shared_ptr<std::thread> jpeg_rectify_left_thread_ptr;
  std::mutex jpeg_rectify_left_mutex_;
  std::condition_variable jpeg_rectify_left_condition_;
  std::queue<std::pair<std::shared_ptr<robosense::lidar::ImageData>,
                       robosense::device::RSTimestampItem::Ptr>>
      jpeg_rectify_left_queue_;
  bool is_jpeg_rectify_left_running_ = false;

  std::shared_ptr<std::thread> jpeg_rectify_right_thread_ptr;
  std::mutex jpeg_rectify_right_mutex_;
  std::condition_variable jpeg_rectify_right_condition_;
  std::queue<std::pair<std::shared_ptr<robosense::lidar::ImageData>,
                       robosense::device::RSTimestampItem::Ptr>>
      jpeg_rectify_right_queue_;
  bool is_jpeg_rectify_right_running_ = false;

private:
  // DeviceInfo 处理线程
  bool is_device_info_running_ = false;
  std::shared_ptr<std::thread> device_info_thread_ptr;

private:
  // ROS/ROS2 publishers for RGB images, depth point clouds, and IMU data
  bool enable_ros2_zero_copy = false;
#if defined(ROS_FOUND)
  ros::NodeHandle nh;
#elif defined(ROS2_FOUND)
  ROS_ZEROCOPY_PUBLISHER_RSIMAGE8M_PTR publisher_rgb_loan;
  ROS_ZEROCOPY_PUBLISHER_RSIMAGE4M_PTR publisher_rgb_left_loan;
  ROS_ZEROCOPY_PUBLISHER_RSIMAGE4M_PTR publisher_rgb_right_loan;
  ROS_ZEROCOPY_PUBLISHER_RSIMAGE8M_PTR publisher_rgb_rectify_loan;
  ROS_ZEROCOPY_PUBLISHER_RSIMAGE4M_PTR publisher_rgb_rectify_left_loan;
  ROS_ZEROCOPY_PUBLISHER_RSIMAGE4M_PTR publisher_rgb_rectify_right_loan;
  ROS_PUBLISHER_POINTCLOUD1M_PTR publisher_depth_loan;
  ROS_PUBLISHER_POINTCLOUD4M_PTR publisher_depth_ac2_loan;
  ROS_PUBLISHER_POINTCLOUD4M_PTR publisher_depth_ac2_wave2_loan;
#endif // ROS_ROS2_FOUND
  ROS_PUBLISHER_CAMERAINFO_PTR publisher_camera_info;
  ROS_PUBLISHER_CAMERAINFO_PTR publisher_left_camera_info;
  ROS_PUBLISHER_CAMERAINFO_PTR publisher_right_camera_info;
  ROS_PUBLISHER_IMAGE_PTR publisher_rgb;
  ROS_PUBLISHER_IMAGE_PTR publisher_rgb_left;
  ROS_PUBLISHER_IMAGE_PTR publisher_rgb_right;
  ROS_PUBLISHER_IMAGE_PTR publisher_rgb_rectify;
  ROS_PUBLISHER_IMAGE_PTR publisher_rgb_rectify_left;
  ROS_PUBLISHER_IMAGE_PTR publisher_rgb_rectify_right;
  ROS_PUBLISHER_POINTCLOUD2_PTR publisher_depth;
  ROS_PUBLISHER_POINTCLOUD2_PTR publisher_depth_ac2_wave2;
  ROS_PUBLISHER_IMU_PTR publisher_imu;
  ROS_PUBLISHER_COMPRESSED_IMAGE_PTR publisher_jpeg;
  ROS_PUBLISHER_COMPRESSED_IMAGE_PTR publisher_jpeg_left;
  ROS_PUBLISHER_COMPRESSED_IMAGE_PTR publisher_jpeg_right;
  ROS_PUBLISHER_COMPRESSED_IMAGE_PTR publisher_jpeg_rectify;
  ROS_PUBLISHER_COMPRESSED_IMAGE_PTR publisher_jpeg_rectify_left;
  ROS_PUBLISHER_COMPRESSED_IMAGE_PTR publisher_jpeg_rectify_right;
  ROS_PUBLISHER_RSACDEVICECALIB_PTR publisher_device_calib_info;

  // 设备管理
  std::string device_interface;
  robosense::device::DeviceInterfaceType device_interface_type;
  std::string gmsl_device_number;
  std::shared_ptr<robosense::device::DeviceManager> device_manager_ptr;
  std::mutex current_device_uuid_mtx;
  std::string current_device_uuid;
  bool current_device_info_ready = false;
  bool current_device_info_valid = false;
  robosense::lidar::DeviceInfo current_device_info;
  ROS_RSACDEVICECALIB current_device_calib_msg;

  // 驱动相关
  int image_input_fps = 30;
  int imu_input_fps = 200;
  bool enable_jpeg = false;
  bool enable_rectify = false;
  bool enable_rectify_jpeg = false;
  int jpeg_quality = 70;
  std::string angle_calib_basic_dir_path = "";
  std::string device_calib_file_path = "";
  // 畸变映射矩阵
  bool camera_rectify_map_valid = false;
  cv::Mat camera_rectify_map1;
  cv::Mat camera_rectify_map2;
  bool left_camera_rectify_map_valid = false;
  cv::Mat left_camera_rectify_map1;
  cv::Mat left_camera_rectify_map2;
  bool right_camera_rectify_map_valid = false;
  cv::Mat right_camera_rectify_map1;
  cv::Mat right_camera_rectify_map2;
  // 相机内参
  ROS_CAMERAINFO_PTR camera_info_ptr;
  ROS_CAMERAINFO_PTR left_camera_info_ptr;
  ROS_CAMERAINFO_PTR right_camera_info_ptr;
  bool device_manager_debug = false;
  bool enable_use_lidar_clock = false;
  double timestamp_compensate_s = 0.0;
  bool enable_use_dense_points = false;
  bool enable_use_first_point_ts = false;
  bool enable_ac2_pointcloud_wave_split = false;
  bool enable_angle_and_device_calib_info_from_device = false;
  bool enable_device_calib_info_from_device_pripority = false;

  bool enable_pointcloud_send = true;
  bool enable_ac1_image_send = true;
  bool enable_ac2_left_image_send = true;
  bool enable_ac2_right_image_send = true;
  bool enable_imu_send = true;

  std::string timestamp_output_dir_path = "";
  robosense::device::RSTimestampManager::Ptr timestamp_manager_ptr;

  // Frame Id
  std::string point_frame_id = "rslidar";
  std::string ac1_image_frame_id = "rslidar";
  std::string ac2_left_image_frame_id = "rslidar";
  std::string ac2_right_image_frame_id = "rslidar";
  std::string imu_frame_id = "rslidar";

  // Log 配置
  robosense::log::RSLogConfig log_config;

  // JEPG 图像编码
  int32_t image_width_rgb;
  int32_t image_height_rgb;
  int32_t image_crop_width_rgb;
  int32_t image_crop_height_rgb;
  int32_t image_left_crop_width_rgb;
  int32_t image_left_crop_height_rgb;
  int32_t image_right_crop_width_rgb;
  int32_t image_right_crop_height_rgb;
  int32_t rgb_image_size;
  int32_t nv12_image_size;
  int32_t rgb_crop_image_size;
  int32_t rgb_left_crop_image_size;
  int32_t rgb_right_crop_image_size;
  int32_t image_width_driver;
  int32_t image_height_driver;
  robosense::jpeg::JpegCoder::Ptr jpeg_encoder_ptr;
  robosense::jpeg::JpegCoder::Ptr jpeg_left_encoder_ptr;
  robosense::jpeg::JpegCoder::Ptr jpeg_right_encoder_ptr;
  robosense::jpeg::JpegCoder::Ptr jpeg_rectify_encoder_ptr;
  robosense::jpeg::JpegCoder::Ptr jpeg_rectify_left_encoder_ptr;
  robosense::jpeg::JpegCoder::Ptr jpeg_rectify_right_encoder_ptr;
  robosense::lidar::LidarType lidar_type;

  // 数据缓冲区
  std::vector<uint8_t> rgb_buf;
  std::vector<uint8_t> rgb_left_buf;
  std::vector<uint8_t> rgb_right_buf;

  std::vector<uint8_t> crop_rgb_buf;
  std::vector<uint8_t> crop_rgb_left_buf;
  std::vector<uint8_t> crop_rgb_right_buf;

  std::string topic_prefix = "";
  std::string serial_number = "";

  robosense::color::ColorCodec::Ptr rgb_codec_ptr;
  robosense::color::ColorCodec::Ptr rgb_left_codec_ptr;
  robosense::color::ColorCodec::Ptr rgb_right_codec_ptr;

  RSImageCropConfig ac1_crop_config;
  RSImageCropConfig ac2_left_crop_config;
  RSImageCropConfig ac2_right_crop_config;

#if defined(ENABLE_SUPPORT_RS_DRIVER_ALGORITHM)
  robosense::lidar::AlgorithmParam algorithm_param;
#endif // ENABLE_SUPPORT_RS_DRIVER_ALGORITHM

  std::shared_ptr<PointCloudT<RsPointXYZIRT>> ac2_wave1_pointcloud_ptr;
  std::shared_ptr<PointCloudT<RsPointXYZIRT>> ac2_wave2_pointcloud_ptr;

private:
  const int image_width_ac1 = 1920;
  const int image_height_ac1 = 1080;
  const int image_usb_width_ac2_driver = 1616;
  const int image_usb_height_ac2_driver = 2592;
  const int image_usb_with_angle_calib_width_ac2_driver = 1616;
  const int image_usb_with_angle_calib_height_ac2_driver = 2636;
  const int image_gmsl_width_ac2_driver = 6464;
  const int image_gmsl_height_ac2_driver = 2592;
  const int image_width_ac2_rgb = 1600;
  const int image_height_ac2_rgb = 1200;

private:
  std::string topic_name;
  std::string left_topic_name;
  std::string right_topic_name;
  std::string rectify_topic_name;
  std::string rectify_left_topic_name;
  std::string rectify_right_topic_name;
  std::string pointcloud_topic_name;
  std::string pointcloud_ac2_wave2_topic_name;
  std::string imu_topic_name;
  std::string jpeg_topic_name;
  std::string jpeg_left_topic_name;
  std::string jpeg_right_topic_name;
  std::string jpeg_rectify_topic_name;
  std::string jpeg_rectify_left_topic_name;
  std::string jpeg_rectify_right_topic_name;
  std::string camera_info_topic_name;
  std::string camera_info_left_topic_name;
  std::string camera_info_right_topic_name;
  std::string ac_device_calib_info_topic_name;
};

/**
 * @brief Main function initializes the ROS2 node and spins it.
 * @param argc Argument count.
 * @param argv Argument values.
 * @return Exit status.
 */
int main(int argc, char **argv) {
  const uint64_t timestamp_ns = RS_TIMESTAMP_NS;
  const std::string &node_name = "ms_node" + std::to_string(timestamp_ns);
#if defined(ROS_FOUND)
  ros::init(argc, argv, node_name.c_str());
  MSPublisher ms_publisher;
  int ret = ms_publisher.init();
  if (ret != 0) {
    std::cerr << "initial robosense ac1/ac2 drivers failed: ret = " << ret
              << std::endl;
  } else {
    std::cout << "initial robosense ac1/ac2 drivers successed !" << std::endl;
  }
  ros::spin();
#elif defined(ROS2_FOUND)
  rclcpp::init(argc, argv);
  std::shared_ptr<MSPublisher> publisherPtr =
      std::make_shared<MSPublisher>(node_name);
  int ret = publisherPtr->init();
  if (ret != 0) {
    std::cerr << "initial robosense ac1/ac2 drivers failed: ret = " << ret
              << std::endl;
  } else {
    std::cout << "initial robosense ac1/ac2 drivers successed !" << std::endl;
  }
  rclcpp::spin(publisherPtr);
  rclcpp::shutdown();
#endif // defined(ROS_ROS2_FOUND)

  return 0;
}
