# ac_driver

[中文文档](https://github.com/RoboSense-Robotics/robosense_ac_ros2_sdk_infra/blob/main/modules/ac_driver/README_CN.md)

## 1. Introduction

ac_driver is the ROS/ROS2 middleware node for the AC1/AC2 driver, which is used to receive sensor data from AC1/AC2, integrate the data, and then publish it for use by other nodes. This includes data from three sources: camera, lidar, and IMU.

## 2. Installation

### 2.1 Build (Linux + ROS 2)

Ensure you have a `ROS 2` distribution installed. This project has been developed and tested on `ROS 2 Humble`.

With your `ROS 2` environment ready, clone the repository into your workspace using the following commands:

```bash
# Using ssh
git clone git@github.com:RoboSense-Robotics/robosense_ac_ros2_sdk_infra.git
# Using http
git clone https://github.com/RoboSense-Robotics/robosense_ac_ros2_sdk_infra.git
```
Then, prepare system build environment base on Radxa and X86.

#### 2.1.1 Radxa Development Board

install ffmpeg-rockchip depend libs, refer to https://docs.radxa.com/rock5/rock5b/app-development/rtsp?target=ffmpeg:
```bash
sudo apt-get update
sudo apt-get install build-essential cmake git libdrm-dev librga-dev librockchip-mpp-dev libsdl2*-dev libx264-dev libx265-dev pkg-config
```
If you cannot access the apt source list to install the dependency libraries, please download the source code and install it. For reference, https://github.com/nyanmisaka/ffmpeg-rockchip/wiki/Compilation

install ffmpeg-rockchip:
```bash
git clone https://github.com/nyanmisaka/ffmpeg-rockchip
pushd ffmpeg-rockchip/
./configure --prefix=/usr --enable-gpl --enable-version3 --enable-libdrm --enable-rkmpp --enable-rkrga --enable-libx264 --enable-libx265 --enable-ffplay --enable-alsa --enable-sndio --enable-x11grab --enable-xv --extra-libs="-lasound -lx264 -lsndio -lX11 -lXv" --extra-cflags="-I/usr/include/alsa -I/usr/local/include" --extra-ldflags="-L/usr/lib/aarch64-linux-gnu -L/usr/local/lib"
make -j$(nproc)
sudo make install
popd
```

After the installation is complete, execute the following command to configure the system dependency library environment:
```bash
sudo ln -s /usr/lib/aarch64-linux-gnu/librga.so.2.1.0 /usr/lib/aarch64-linux-gnu/librga.so
sudo ln -s /usr/lib/aarch64-linux-gnu/libdrm.so.2.123.0 /usr/lib/aarch64-linux-gnu/libdrm.so
```
#### 2.1.2 X86 Board
If use GPU to compression the jpeg image, you should install the third library: https://github.com/CESNET/GPUJPEG 
#### 2.1.3 Jetson Orin Platform

ensure that the CUDA environment is properly installed. Follow these steps:

1. Install the necessary CUDA libraries:
```bash
sudo apt-get update
sudo apt-get install nvidia-cuda-toolkit
```

2. Verify the CUDA installation:
```bash
nvcc --version
```

3. Ensure that ROS2 and OpenCV dependencies are installed. If use GPU to compression the jpeg image, you should install the third library: https://github.com/CESNET/GPUJPEG 

4. Follow the instructions in section 2.1.3 to build the project.

#### 2.1.4 Build
Then, enter the modules directory, Run the following commands to compile:

```bash
# Full build
colcon build

# Or build individually
colcon build --symlink-install --packages-select robosense_msgs
colcon build --symlink-install --packages-select ac_driver
```

## 3. Usage

### 3.1 Prepare the ac_driver environment
Refresh the bash profile of the workspace to ensure that the environment configuration of the components is ok.
Run the following commands:
```bash
source install/setup.bash
```

**Note:** Before starting the driver, ensure that the `ROS_DOMAIN_ID` is set correctly. Failure to set this may result in abnormal driver processes or data publishing delays. You can set it as follows:
```bash
export ROS_DOMAIN_ID=<your_domain_id>
```
Replace `<your_domain_id>` with the appropriate domain ID for your ROS 2 environment.

### 3.2 Run the ac_driver Node
The ac_driver node can bu run using the ros run command. 

```sh
For AC1: 
roslaunch ac_driver start_ac1.launch start_rviz_node:=false|true 

For AC2: 
roslaunch ac_driver start_ac2_usb.launch start_rviz_node:=false|true 
roslaunch ac_driver start_ac2_gmsl.launch start_rviz_node:=false|true 
```

The ac_driver node can be run using the ros2 run command.

1. Non-zero-copy mode
```bash
Step1: setting enable_ros2_zero_copy is false
For AC1: 
ros2 launch ac_driver start_ac1.launch.py start_rviz_node:=false|true 
For AC2: 
ros2 launch ac_driver start_ac2_usb.launch.py start_rviz_node:=false|true 
ros2 launch ac_driver start_ac2_gmsl.launch.py start_rviz_node:=false|true 
```
2. Zero-copy mode (only for ROS2 Humble With FastDDS)
```bash
Step1: envirnment setting 
export FASTRTPS_DEFAULT_PROFILES_FILE=ac_driver/conf/shm_fastdds.xml
export RMW_FASTRTPS_USE_QOS_FROM_XML=1

Step2: setting enable_ros2_zero_copy is true 
For AC1: 
ros2 launch ac_driver start_ac1.launch.py start_rviz_node:=false|true 
For AC2: 
ros2 launch ac_driver start_ac2_usb.launch.py start_rviz_node:=false|true 
ros2 launch ac_driver start_ac2_gmsl.launch.py start_rviz_node:=false|true 
```

#### Parameter Description
- device_interface : AC1/AC2 Connect Mode, support "usb"/"gmsl". The default value is "usb"；
- `image_input_fps`: The image sensor output frame rates, where the AC1's frame rates supported by images include: 10Hz/15Hz/30Hz. the AC2's frame rates supported by images include: 10hz(gmsl mode), 15hz(usb mode). 
- `imu_input_fps`:  The imu sensor output frame rates, where the frame rates supported by Imu include: 100Hz/200Hz. The default value is 200
- `enable_jpeg`: Whether to enable JPEG image compression. The default value is `false` (disabled). When enabled, it compresses images using JPEG to reduce bandwidth usage but increases CPU usage. You can enable it as follows:
- enable_rectify: Whether to enable Image Rectify. The default value is false(disabled). 
- enable_rectify_jpeg: Whether to enable JPEG rectify image compression. The default value is false(disabled). 
- `jpeg_quality`: The quality of JPEG image compression, default is 70
- serial_number: AC1/AC2 USB Serial Number; If not setting the driver will open the first device which is found. 
- gmsl_device_number: AC2 Connect by GMSL Mode，e.g. "/dev/video30"
- point_frame_id: PointCloud message ROS/ROS2's  frame_id, The default value is "rslidar"
- ac1_image_frame_id: AC1 image message ROS/ROS2's  frame_id, The default value is"rslidar"
- ac2_left_image_frame_id: AC2 left image message ROS/ROS2's  frame_id, The default value is"rslidar"
- ac2_right_image_frame_id: AC2 right image message ROS/ROS2's  frame_id, The default value is"rslidar"
- imu_frame_id: Imu message ROS/ROS2's frame_id, The default value is"rslidar"
- enable_angle_and_device_calib_info_from_device: Whether to enable read angle calibration and device calibration from device. The default value is true. 
- angle_calib_basic_dir_path：device angle calibration file path, only for ac2. The default value is ""(empty string). 
- enable_device_calib_info_from_device_pripority: Whether use calibration from device with high pripority. The default value is true
- device_calib_file_path: The offline device calibration file path. The default value is ""(empty string); 
- device_manager_debug: Whether enable debug mode, The default value is false 
- enable_use_lidar_clock: Whether using device time clock , The default value is false 
- timestamp_compensate_s: If enable_use_lidar_clock = true, then the timestamp compensate value. The default value is 0.0, unit is second(s) 
- enable_use_dense_points：Whether enable output dense point mode，The default value is false 
- enable_use_first_point_ts：Whether using the first point timestamp as pointcloud frame timestamp，The default value is false 
- enable_ac2_pointcloud_wave_split: Whether Split AC2 PointCloud By Wave Number, The default value is false  
- timestamp_output_dir_path: The timestamp statistical output directory path, if is empty that means no statistical output 
- enable_ros2_zero_copy:  Whether Using Ros2 Zero-Copy Send Message. The default value is false. 
- enable_pointcloud_send: Whether send PointCloud message by ROS/ROS2, The default value is true  
- enable_ac1_image_send: Whether send AC Image message by ROS/ROS2, The default value is true 
- enable_ac2_left_image_send: Whether send AC2 left image message by ROS/ROS2, The default value is true 
- enable_ac2_right_image_send: Whether send AC2 right image message by ROS/ROS2, The default value is true 
- enable_imu_send: Whether send IMU message by ROS/ROS2, The default value is true 
- spdlog setting: log_file_dir_path/log_level/is_log_file_trunc: log output directory path/log output minimum level, default is 2(INFO)/whether truncate file if   log file exist 
- ac1_crop_top/ac1_crop_bottom/ac1_crop_left/ac1_crop_right: AC1 image crop setting, The default value is 0 that means don't crop 
- ac2_left_crop_top/ac2_left_crop_bottom/ac2_left_crop_left/ac2_left_crop_right: AC2 left image crop setting, The default value is 0 that means don't crop
-  ac2_right_crop_top/ac2_right_crop_bottom/ac2_right_crop_left/ac2_right_crop_right, AC2 right image crop setting, The default value is 0 that means don't crop 

Depending on the method of starting the node, as mentioned in 1/2 above, if starting through the **ros2 run** command, parameters can be passed in through the **-- param** command; If using the **ros2 launch** startup command, modify the startup parameter settings in the **start.launch** file.

#### Format Description

- At present, image transcoding optimization has been done on rk3588 and jetson orin platforms, and the default driver is directly exported to nv12 and released into rgb24 format images through hardware;

- Other platforms use rgb24 by default and adopt cpu jepg compression. The compression function considers the impact of performance. The compression function is disabled by default and can be turned on by enable-jpeg switch.

### 3.3 View the published sensor data.

#### 3.3.1 View Published Sensor Data Through a User Interface

To view the published sensor data via a graphical interface, you can use tools like rviz2 in ROS2. Here are the steps:
1. Install rviz:
Ensure that rviz is installed:
```bash
sudo apt-get install ros-<ros2-distro>-rviz2
```
2. Launch rviz:
Start rviz to visualize the sensor data:
```bash
rviz2
```
3. Configure rviz:
In the rviz interface, add the necessary displays to visualize different types of sensor data:
For image data, add the Image display.
For point cloud data from lidar, add the PointCloud2 display.
For IMU data, add the IMU display.

4. Select Topics:
Configure the displays to subscribe to the appropriate topics published by the ac_driver node.

By following these steps, you can view the published sensor data from the ac_driver node using graphical interfaces like rviz.

#### 3.3.2 Recording and Viewing Data
You can use the built-in ROS2 bag recording tool to record and then play back the data for viewing. Here are the steps:

1. Record Data:
Use the ros2 bag record command to record data from specific topics. For example, to record data from all topics, you can use:
```bash
ros2 bag record -a
```

To record data from specific topics, specify the topic names:
```bash
ros2 bag record /topic1 /topic2
```
2. Play Back Data:
Once the recording is complete, you can play back the recorded data using the ros2 bag play command:
```bash
ros2 bag play <bagfile>
```
Replace <bagfile> with the path to your recorded bag file.

3. View Data:
While playing back the data, you can use tools like rviz to view the data.
For more detailed instructions on recording and playing back data, you can refer to the ROS2 documentation on recording and playing back data .


## 4. Features
### 4.1  Dependencies
The ac_driver node relies on several key libraries and packages to function properly. Here is a detailed list of the dependencies:

#### 4.1.1 ROS2 Core Libraries:
* rclcpp: The ROS2 C++ client library, providing the core functionality for ROS2 nodes.
* sensor_msgs: Provides standard message types for common sensor data, such as images and point clouds.
* std_msgs: Provides standard message types for basic data types, such as integers, floats, and strings.
#### 4.1.2 robosense_msgs:
This custom ROS2 package defines the message formats for H.265 compressed images and other sensor data specific to the AC sensors. It is essential for the ac_driver node to interpret and publish the sensor data correctly.

### 4.2 Topic 
1. AC1 Lidar Topic: 
   - Topic name: /rs_lidar/points
   
   * ROS2 zero-copy message type(custom message): robosense_msgs/msg/RsPointCloud1M
   * ROS2 message type: sensor_msgs/msg/PointCloud2 
   * ROS    message type: sensor_msgs/PointCloud2 
   
2. AC2 Lidar Topic: 

   - Topic name: /rs_lidar/points

   * ROS2 zero-copy message type(custom message): robosense_msgs/msg/RsPointCloud4M
   * ROS2 message type: sensor_msgs/msg/PointCloud2 
   * ROS   message type: sensor_msgs/PointCloud2 

3. AC1/AC2 Imu Topic: 

   - Topic name: /rs_imu
   - ROS2 message type: sensor_msgs/msg/Imu
   - ROS   message type: sensor_msgs/Imu  

4. AC1 Camera JPEG Image Topic: 

   - Topic name: /rs_camera/color/image_raw/compressed
   - ROS2 message type: sensor_msgs/msg/CompressedImage
   - ROS    message type: sensor_msgs/CompressedImage  

5. AC1 Rectify Camera JPEG Image Topic: 

   - Topic name: /rs_camera/rect/color/image_raw/compressed
   - ROS2 message type: sensor_msgs/msg/CompressedImage
   - ROS    message type: sensor_msgs/CompressedImage  

6. AC1 Camera RGB Image Topic: 

   - Topic name: /rs_camera/color/image_raw 
   - ROS2 zero-copy message type(custom message): robosense_msgs/msg/RsImage8M
   - ROS2 message type: sensor_msgs/msg/Image
   - ROS    message type: sensor_msgs/Image 

7. AC1 Rectify Camera RGB Image Topic: 

   - Topic name: /rs_camera/rect/color/image_raw 
   - ROS2 zero-copy message type(custom message): robosense_msgs/msg/RsImage8M
   - ROS2 message type: sensor_msgs/msg/Image
   - ROS    message type: sensor_msgs/Image 

8. AC2 Camera JPEG Image Topic: 

   - Left Camera Topic name:  /rs_camera/left/color/image_raw/compressed
   - Right Camera Topic name:  /rs_camera/right/color/image_raw/compressed 
   - ROS2 message type: sensor_msgs/msg/CompressedImage
   - ROS   message type: sensor_msgs/CompressedImage 

9. AC2 Rectify Camera JPEG Image Topic: 

   - Left Camera Topic name:  /rs_camera/left/rect/color/image_raw/compressed
   - Right Camera Topic name::  /rs_camera/right/rect/color/image_raw/compressed 
   - ROS2 message type: sensor_msgs/msg/CompressedImage
   - ROS   message type: sensor_msgs/CompressedImage 

10. AC2 Camera RGB Image Topic: 

    - Left Camera Topic name: /rs_camera/left/rect/color/image_raw/compressed 
    - Right Camera Topic name:: /rs_camera/right/color/image_raw 

    * ROS2 zero-copy message type(custom message): robosense_msgs/msg/RsImage4M
    * ROS2 message type: sensor_msgs/msg/Image
    * ROS   message type: sensor_msgs/Image 

11. AC2 Rectify Camera RGB Image Topic: 

    - Left Camera Topic name:  /rs_camera/left/rect/color/image_raw
    - Right Camera Topic Name: /rs_camera/right/rect/color/image_raw 

    * ROS2 zero-copy message type(custom message): robosense_msgs/msg/RsImage4M
    * ROS2 message type: sensor_msgs/msg/Image
    * ROS   message type: sensor_msgs/Image 

## 5. Limitations

### 5.1 Zero-Copy Usage Limitations

- Compared to the ROS2 publisher/subscriber data transmission method, using zero-copy transmission has the following limitations:
  - Currently only supports the Humble version. It is recommended to use RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT for QOS Reliability (it is recommended to directly use `rclcpp::SensorDataQoS()` to set QOS).
  - QOS History only supports KEEPLAST, does not support KEEPALL, and KEEPLAST cannot be set too large due to memory limitations. Currently, it is set to a maximum of 256MB.
  - The size of the transmitted message is fixed, i.e., the `sizeof` value of the message does not change. It cannot contain variable-length data types, such as strings or dynamic arrays.
  - RMW_QOS_POLICY_RELIABILITY_RELIABLE has stability issues under multiple communication methods.
  - Can only be used for inter-process communication on the same device and cannot be transmitted across devices.
  - The publisher message must be obtained first and then assigned before sending. It must also check whether it was successfully obtained.
  - The message received by the subscriber is only valid within the callback function and cannot be used outside the callback function.
- How to use the zero-copy message example for c++ 

  - RS_IMAGE_TYPE: robosense_msgs::msg::RsImage4M或robosense_msgs::msg::RsImage8M 
  - RS_POINTCLOUD_TYPE: robosense_msgs::msg::RsPointCloud1M 或 robosense_msgs::msg::RsPointCloud4M 

```
template <typename RS_IMAGE_TYPE>
static int convertToRosImage(const typename RS_IMAGE_TYPE::SharedPtr &msgPtr,
                             sensor_msgs::msg::Image &rosMsg) {
  rosMsg.header.frame_id =
      std::string((const char *)msgPtr->header.frame_id.data());
  rosMsg.header.stamp = msgPtr->header.stamp;

  rosMsg.height = msgPtr->height;
  rosMsg.width = msgPtr->width;

  rosMsg.encoding = std::string((const char *)msgPtr->encoding.data());
  rosMsg.is_bigendian = msgPtr->is_bigendian;
  rosMsg.step = msgPtr->step;

  const size_t data_size = 3 * msgPtr->height * msgPtr->width;
  rosMsg.data.resize(data_size, 0);
  memcpy(rosMsg.data.data(), msgPtr->data.data(), data_size);

  return 0;
}

template <typename RS_POINTCLOUD_TYPE>
static int
convertToRosPointCloud2(const typename RS_POINTCLOUD_TYPE::SharedPtr &msgPtr,
                        sensor_msgs::msg::PointCloud2 &rosMsg) {
  rosMsg.header.frame_id =
      std::string((const char *)msgPtr->header.frame_id.data());
  rosMsg.header.stamp = msgPtr->header.stamp;

  rosMsg.height = msgPtr->height;
  rosMsg.width = msgPtr->width;

  rosMsg.fields.resize(6);
  for (int i = 0; i < 6; ++i) {
    const auto &rs_field = msgPtr->fields[i];
    auto &field = rosMsg.fields[i];

    field.name = std::string((const char *)rs_field.name.data());
    field.offset = rs_field.offset;
    field.datatype = rs_field.datatype;
    field.count = rs_field.count;
  }

  rosMsg.is_bigendian = msgPtr->is_bigendian;
  rosMsg.point_step = msgPtr->point_step;
  rosMsg.row_step = msgPtr->row_step;
  rosMsg.is_dense = msgPtr->is_dense;

  const size_t data_size = msgPtr->row_step * msgPtr->height;
  rosMsg.data.resize(data_size, 0);
  memcpy(rosMsg.data.data(), msgPtr->data.data(), data_size);

  return 0;
}
```



### 5.2 Performance

* For images at 30fps, the CPU usage on Orin Nano is measured at 105%. If JPEG compression is disabled, CPU usage can be reduced by 40-50%. Disable it as needed by commenting out the following code:
```cpp
{
    std::lock_guard<std::mutex> lock(jpeg_mutex_);
    jpeg_queue_.push(msgPtr);
    jpeg_condition_.notify_one();
}
```


