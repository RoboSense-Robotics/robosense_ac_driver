# ac_driver

[README](https://github.com/RoboSense-Robotics/robosense_ac_ros2_sdk_infra/blob/main/modules/ac_driver/README.md)

## 1. 简介

ac_driver是AC传感器驱动的ROS中间件节点,用于接收传感器数据，整合和发布给其它节点使用。传感器数据包括摄像头，激光雷达和IMU。

## 2. 构建

### 2.1 编译 (Linux + ROS/ROS2)

确保ROS/ROS2的编译环境已安装，整个工程的开发和测试基于的是ROS Noetic版本/ROS2 Humble版本，

[ROS Noetic 官方安装文档](https://wiki.ros.org/ROS/Installation)

[ROS2 Humble官方安装文档](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)

ROS/ROS2环境准备好后，使用以下命令将代码下载到工作目录下。

```bash
# Using ssh
git clone git@github.com:RoboSense-Robotics/robosense_ac_ros2_sdk_infra.git
# Using http
git clone https://github.com/RoboSense-Robotics/robosense_ac_ros2_sdk_infra.git
```
然后进行系统编译依赖的准备操作，如下基于Radxa开发板及X86环境。

#### 2.1.1 Radxa开发板

安装第三方依赖库,执行以下命令:

```bash
sudo apt-get update
sudo apt-get install build-essential cmake git libdrm-dev librga-dev librockchip-mpp-dev libsdl2*-dev pkg-config
```
在安装完成后，执行以下命令配置系统依赖库环境:

```bash
sudo ln -s /usr/lib/aarch64-linux-gnu/librga.so.2.1.0 /usr/lib/aarch64-linux-gnu/librga.so
sudo ln -s /usr/lib/aarch64-linux-gnu/libdrm.so.2.123.0 /usr/lib/aarch64-linux-gnu/libdrm.so
```
#### 2.1.2 X86
如果支持Nvidia GPU, 则参考2.1.3节安装必要的CUDA相关依赖， 如果不支持，则确保安装Opencv； 
#### 2.1.3 Jetson Orin 平台

请确保CUDA环境已正确安装。以下是步骤：

1. 安装必要的CUDA库：
```bash
sudo apt-get update
sudo apt-get install nvidia-cuda-toolkit
```

2. 验证CUDA安装：
```bash
nvcc --version
```

3. 确保已安装ROS2和OpenCV的相关依赖。

4. 按照2.1.4节中的说明进行项目构建。

#### 2.1.4 编译

##### 2.1.4.1 ROS 环境

进入src所在目录， 使用以下命令进行编译:

`caktin_make`

##### 2.1.4.2 ROS2 环境

进入src所在目录, 使用以下命令进行编译:

```bash
# 全量编译
colcon build
```
## 3. 运行

### 3.1 准备环境

### 3.1.1 ROS环境

刷新工作目录下的bash配置文件，确保组件的配置是完整的。
使用以下命令刷新:

`source devel/setup.bash `

### 3.1.2 ROS2 环境

刷新工作目录下的bash配置文件，确保组件的配置是完整的。
使用以下命令刷新:

```bash
source install/setup.bash
```

**注意:** 启动驱动前，请确保正确设置 `ROS_DOMAIN_ID`，否则可能导致驱动进程异常或数据发布延迟异常。可以通过以下命令设置：

```bash
export ROS_DOMAIN_ID=<your_domain_id>
```
将 `<your_domain_id>` 替换为您ROS 2环境中适当的域ID。

### 3.2 运行ac_driver节点

### 3.2.1 ROS环境

使用以下命令运行ac_driver节点

```bash
roslaunch ac_driver start.launch 
或
roscore 2>&1 >/dev/null &
rosrun ac_driver ms_node [_device_interface:="usb" _image_input_fps:=30 _imu_input_fps:=200 _enable_jpeg:=false _jpeg_quality:=70 _topic_prefix:="" _serial_number:="" _gmsl_device_number:="/dev/video30" _angle_calib_basic_dir_path:="" _device_manager_debug:=false _point_frame_id:="rslidar" _ac1_image_frame_id:="rslidar"  _ac2_left_image_frame_id:="rslidar" _ac2_right_image_frame_id:="rslidar"  _imu_frame_id:="rslidar"  _enable_use_lidar_clock:=false  _enable_use_dense_points:=false  _enable_use_first_point_ts:= false _timestamp_output_dir_path=""]
```

### 3.2.2 ROS2 环境

使用以下命令运行ac_driver节点

1. 非零拷贝模式
```bash
ros2 run ac_driver ms_node [--ros-args --param device_interface:="usb" --param image_input_fps:=30 --param imu_input_fps:=200 --param enable_jpeg:=false --param jpeg_quality:=70 --param topic_prefix:="" --param serial_number:="" --param gmsl_device_number="/dev/video30" --param angle_calib_basic_dir_path:="" --param device_manager_debug:=false --param point_frame_id:="rslidar" --param ac1_image_frame_id:="rslidar" --param ac2_left_image_frame_id:="rslidar" --param ac2_right_image_frame_id:="rslidar" --param imu_frame_id:="rslidar" --param enable_use_lidar_clock:=false --param enable_use_dense_points:=false --param enable_use_first_point_ts:= false --param timestamp_output_dir_path:=""]
或 
ros2 launch ac_driver start.launch.py 
```
2. 零拷贝模式(仅限ros2 humble版本使用FASTDDS进行通信时)
```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=ac_driver/conf/shm_fastdds.xml
export RMW_FASTRTPS_USE_QOS_FROM_XML=1
ros2 run ac_driver ms_node [--ros-args --param device_interface:="usb" --param image_input_fps:=30 --param imu_input_fps:=200 --param enable_jpeg:=false --param jpeg_quality:=70 --param topic_prefix:="" --param serial_number:="" --param gmsl_device_number="/dev/video30" --param angle_calib_basic_dir_path:="" --param device_manager_debug:=false --param point_frame_id:="rslidar" --param ac1_image_frame_id:="rslidar" --param ac2_left_image_frame_id:="rslidar" --param ac2_right_image_frame_id:="rslidar" --param imu_frame_id:="rslidar" --param enable_use_lidar_clock:=false --param enable_use_dense_points:=false --param enable_use_first_point_ts:= false --param timestamp_output_dir_path:=""]
或
export FASTRTPS_DEFAULT_PROFILES_FILE=ac_driver/conf/shm_fastdds.xml
export RMW_FASTRTPS_USE_QOS_FROM_XML=1
ros2 launch ac_driver start.launch.py
```

### 3.2.3 参数说明

- device_interface : 相机接入的方法: 支持"usb"/"gmsl"，默认为"usb"；
- `image_input_fps`: AC1图像支持的帧率包含: 10Hz/15Hz/30Hz, AC2图像支持的帧率包含: 10Hz(gmsl模式), 15Hz(usb模式)；
- `imu_input_fps`: Imu支持的帧率包含: 100Hz/200Hz，默认为200Hz;
- `enable_jpeg`: 是否启用JPEG图像压缩，默认值为`false`， 在不包含CUDA的环境中，启用后会增加CPU使用率；
- enable_rectify: 是否启用图像去畸变，默认值为false, 
- enable_rectify_jpeg: 是否启用JPEG去畸变图像压缩，默认为false, 
- `jpeg_quality`: JPEG图像压缩的质量，默认为70; 
- topic_prefix: 话题名称前缀，默认为""; 
- serial_number: AC1/AC2设备序列号，驱动默认打开第一个发现的AC1/AC2设备，如果配置该值，则打开指定的设备 
- gmsl_device_number: AC2 使用gmsl接入时的设备号，例如"/dev/video30"
- point_frame_id: 点云的ROS/ROS2 frame_id, 默认为"rslidar"
- ac1_image_frame_id: AC1 图像的 ROS/ROS2 frame_id, 默认为"rslidar"
- ac2_left_image_frame_id: AC2 左图像的ROS/ROS2 frame_id, 默认为"rslidar"
- ac2_right_image_frame_id: AC2 右图像的ROS/ROS2 frame_id, 默认为"rslidar"
- imu_frame_id: Imu的ROS/ROS2 frame_id, 默认为"rslidar"
- enable_angle_and_device_calib_info_from_device: 表示是否从设备中读取标定信息，默认为true 
- angle_calib_basic_dir_path： AC2设备的角标文件所在文件夹路径，对AC2有效
- enable_device_calib_info_from_device_pripority: 使用设备中的标定信息的优先级，默认为True，表示如果能够从设备读到标定信息，则优先使用 
- device_calib_file_path: 设备的离线标定文件路径 
- device_manager_debug: 是否开启设备管理Debug模式，默认关闭
- enable_use_lidar_clock: 是否使用AC1/AC2的设备时间，默认关闭 
- enable_use_dense_points：是否AC1/AC2输出稠密点，默认关闭 
- enable_use_first_point_ts：点云帧时间戳是否为第一个点的时间戳，默认为关闭
- enable_ac2_pointcloud_wave_split: 对于AC2是否根据回波编号进行分离，默认为关闭
- timestamp_output_dir_path:  表示输出时间戳信息的保存文件夹路径，如果为空，表示不输出 
- enable_pointcloud_send: 表示是否通过ROS/ROS2发送点云数据，默认为开启 
- enable_ac1_image_send: 表示是否通过ROS/ROS2发送AC1图像数据，默认为开启 
- enable_ac2_left_image_send: 表示是否通过ROS/ROS2发送AC2图像数据，默认为开启 
- enable_ac2_right_image_send: 表示是否通过ROS/ROS2发送AC2图像数据，默认为开启 
- enable_imu_send: 表示是否通过ROS/ROS2发送IMU数据，默认为开启
- AC1相机图像裁减配置: ac1_crop_top/ac1_crop_bottom/ac1_crop_left/ac1_crop_right, 默认为0，表示不裁剪
- AC2 左相机图像裁剪配置: ac2_left_crop_top/ac2_left_crop_bottom/ac2_left_crop_left/ac2_left_crop_right, 默认为0，表示不裁剪 
- AC2 右相机图像裁剪配置: ac2_right_crop_top/ac2_right_crop_bottom/ac2_right_crop_left/ac2_right_crop_right, 默认为0，表示不裁剪

根据启动节点的方法不同， 可以在launch文件或者启动命令中传输参数，使用launch文件启动时，将自动启动rviz工具



### 3.2.4 格式说明

- 当前对jetson orin平台对图像做了转码优化，默认驱动直接出nv12，通过GPU转成rgb24格式图像发布；

- 其他平台默认直出rgb24，并采用cpu/gpu jepg压缩，压缩功能考虑到性能的影响，默认关闭压缩功能，可以通过enable_jpeg开关打开。



### 3.3 查看发布的传感器数据

### 3.3.1 ROS环境

#### 3.3.1.1 rviz工具显示传感器数据

可以使用rviz工具工具显示图像和点云数据，以下是rviz的安装步骤:

1. 安装rviz:

```bash
sudo apt-get install ros-<ros-distro>-rviz
```

2. 运行rviz:

```bash
rviz
```

3. 在rviz2中的"Display"面板，根据4.2节的话题名称进行配置显示对应话题即可，注意对应点云的frame_id为"rslidar", rviz2 中需要设置为相同的frame_id 

#### 3.3.1.2 数据保存和播放

可通过ROS2的录制工具进行数据录制，按以下步骤操作:

1. 数据保存 

```bash
rosbag record -a
```

录制特定topic可使用以下命令:

```bash
rosbag record /topic1 /topic2
```

2. 数据播放

```bash
rosbag play <bagfile>
```

替换<bagfile>为相应录制的路径文件

### 3.3.2 ROS2 环境

#### 3.3.2.1 rviz2工具显示传感器数据

可以使用rviz2工具工具显示图像和点云数据，以下是rviz2的安装步骤:

1. 安装rviz2:
```bash
sudo apt-get install ros-<ros2-distro>-rviz2
```
2. 运行rviz2:
```bash
rviz2
```
3. 在rviz2中的"Display"面板，根据4.2节的话题名称进行配置显示对应话题即可，注意对应点云的frame_id为"rslidar", rviz2 中需要设置为相同的frame_id 


#### 3.3.2.2 数据保存和播放

可通过ROS2的录制工具进行数据录制，按以下步骤操作:

1. 数据保存 
```bash
ros2 bag record -a
```

录制特定topic可使用以下命令:
```bash
ros2 bag record /topic1 /topic2
```
2. 数据播放
```bash
ros2 bag play <bagfile>
```
替换<bagfile>为相应录制的路径文件

## 4. 特性
### 4.1  依赖
ac_driver节点依赖以下关键的库和软件包:

#### 4.1.1 ROS2 Core 库:
* rclcpp: ROS2 C++ 客户端库, 提供ROS2的核心功能.
* std_msgs: ROS2的标准消息。
#### 4.1.2 robosense_msgs:
为H.265定制的ROS2消息及零拷贝模式消息，用于传输AC传感器的数据。

### 4.2 话题名称和类型说明
1. AC1 Lidar Topic: 

   - 话题名称: /rs_lidar/points

   * ROS2 零拷贝模式消息类型(自定义): robosense_msgs/msg/RsPointCloud1M
   * ROS2 非零拷贝模式消息类型: sensor_msgs/msg/PointCloud2 
   * ROS    消息类型: sensor_msgs/PointCloud2 

2. AC2 Lidar Topic: 

   - 话题名称: /rs_lidar/points

   * ROS2 零拷贝模式消息类型(自定义): robosense_msgs/msg/RsPointCloud4M
   * ROS2 非零拷贝模式消息类型: sensor_msgs/msg/PointCloud2 
   * ROS    消息类型: sensor_msgs/PointCloud2 

3. AC1/AC2 Imu Topic: 

   - 话题名称: /rs_imu
   - ROS2 消息类型: sensor_msgs/msg/Imu
   - ROS   消息类型: sensor_msgs/Imu  

4. AC1 Camera JPEG Image Topic: 

   - 话题名称: /rs_camera/color/image_raw/compressed
   - ROS2 消息类型: sensor_msgs/msg/CompressedImage
   - ROS    消息类型: sensor_msgs/CompressedImage  

5. AC1 Rectify Camera JPEG Image Topic: 

   - 话题名称: /rs_camera/rect/color/image_raw/compressed
   - ROS2 消息类型: sensor_msgs/msg/CompressedImage
   - ROS    消息类型: sensor_msgs/CompressedImage  

6. AC1 Camera RGB Image Topic: 

   - 话题名称: /rs_camera/color/image_raw 
   - ROS2 零拷贝模式消息类型(自定义): robosense_msgs/msg/RsImage8M
   - ROS2 非零拷贝模式消息类型: sensor_msgs/msg/Image
   - ROS    消息类型: sensor_msgs/Image 

7. AC1 Rectify Camera RGB Image Topic: 

   - 话题名称: /rs_camera/rect/color/image_raw 
   - ROS2 零拷贝模式消息类型(自定义): robosense_msgs/msg/RsImage8M
   - ROS2 非零拷贝模式消息类型: sensor_msgs/msg/Image
   - ROS    消息类型: sensor_msgs/Image 

8. AC2 Camera JPEG Image Topic: 

   - 左相机话题名称:  /rs_camera/left/color/image_raw/compressed
   - 右相机话题名称:  /rs_camera/right/color/image_raw/compressed 
   - ROS2 消息类型: sensor_msgs/msg/CompressedImage
   - ROS   消息类型: sensor_msgs/CompressedImage 

9. AC2 Rectify Camera JPEG Image Topic: 

   - 左相机话题名称:  /rs_camera/left/rect/color/image_raw/compressed
   - 右相机话题名称:  /rs_camera/right/rect/color/image_raw/compressed 
   - ROS2 消息类型: sensor_msgs/msg/CompressedImage
   - ROS   消息类型: sensor_msgs/CompressedImage 

10. AC2 Camera RGB Image Topic: 

   - 左相机话题名称: /rs_camera/left/color/image_raw
   - 右相机话题名称: /rs_camera/right/color/image_raw 

   * ROS2 零拷贝模式消息类型(自定义): robosense_msgs/msg/RsImage4M
   * ROS2 非零拷贝模式消息类型: sensor_msgs/msg/Image
   * ROS   消息类型: sensor_msgs/Image 

11. AC2 Rectify Camera RGB Image Topic: 

    - 左相机话题名称: /rs_camera/left/rect/color/image_raw
    - 右相机话题名称: /rs_camera/right/rect/color/image_raw 

    * ROS2 零拷贝模式消息类型(自定义): robosense_msgs/msg/RsImage4M
    * ROS2 非零拷贝模式消息类型: sensor_msgs/msg/Image
    * ROS   消息类型: sensor_msgs/Image 

## 5. 注意事项
### 5.1 零拷贝使用限制
- 和ROS2的publisher/subscriber数据传输方式相比，使用零拷贝传输存在以下限制：
  - 当前仅支持Humble版本，推荐QOS Reliability使用RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT（建议直接使用rclcpp::SensorDataQoS()设置QOS）
  - QOS History只支持KEEPLAST，不支持KEEPALL，且KEEPLAST不能设置太大，有内存限制，目前设置为最大占用256M内存
  - 传输的消息大小是固定的，即消息的sizeof值是不变的，不能包含可变长度类型数据，例如：string，动态数组
  - RMW_QOS_POLICY_RELIABILITY_RELIABLE在多种通信方式下存在稳定性问题
  - 只能用于同一设备进程间通信，不可跨设备传输
  - publisher消息要先获取再赋值发送，且要判断是否获取成功
  - subscriber收到的消息有效期仅限回调函数中，不能在回调函数之外使用
- 如何处理零拷贝消息C++代码举例:  
  - RS_IMAGE_TYPE：表示robosense_msgs::msg::RsImage4M或robosense_msgs::msg::RsImage8M 
  - RS_POINTCLOUD_TYPE: 表示robosense_msgs::msg::RsPointCloud1M 或 robosense_msgs::msg::RsPointCloud4M 

```c++
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

### 5.2 运行性能

* 图像30fps在orin nano实测cpu占用为105，若取消jpeg压缩则可降低40~50%,可按需取消，取消方法：将以下代码注释掉：
```cpp
{
    std::lock_guard<std::mutex> lock(jpeg_mutex_);
    jpeg_queue_.push(msgPtr);
    jpeg_condition_.notify_one();
}
```