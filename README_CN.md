# robosense_ac_driver

[English Version](README.md)

## 1. 简介

robosense_ac_driver 是一个基于 ROS/ROS2 的驱动包，用于支持 RoboSense AC1/AC2 传感器: 

- 提供设备驱动: *4.1节*
- 实现传感器数据的读取和处理: 去畸变、发送标定参数、JPEG压缩、零拷贝/非零拷贝数据发送 
- ROS2 docker镜像: 跨平台编译和本地编译环境的 Docker 容器（仅支持ROS 2 Humble）, 包含了容器管理、镜像管理以及自动化环境设置等功能，详情请查看当前目录下[tools/compilation_envirment/README_CN.md](/tools/compilation_envirment/README_CN.md)

## 2. 前置依赖

支持的典型配置：

  - Ubuntu 22.04 (Jammy Jellyfish) + ROS2 Humble (预编译安装)  
    安装请参考：[ROS2 Humble 官方安装文档](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)

  - Ubuntu 20.04 (Focal Fossa) + ROS2 Humble (源码编译)  
    安装请参考：[ROS2 Humble 官方安装文档](https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html)

  - Ubuntu 20.04 (Focal Fossa) + ROS Noetic (预编译安装)  
    安装请参考：[ROS Noetic 官方安装文档](https://wiki.ros.org/noetic/Installation/Ubuntu)

> 💡 提示：若使用其他 Ubuntu 版本（如 24.04），需选择对应的 ROS2 发行版，并自行适配兼容性。

## 3. 代码下载

本项目包含 Git 子模块，请按以下命令完整克隆代码：

```bash
# 克隆主仓库（含子模块）
git clone --recursive https://github.com/RoboSense-Robotics/robosense_ac_driver.git

# 如果已克隆但未初始化子模块，可手动更新
cd robosense_ac_driver
git submodule update --init --recursive
```

## 4. 权限配置与硬件连接

### 4.1 权限配置

首次连接 AC1/AC2 传感器时，需配置 USB 设备访问权限，以避免后续运行驱动时需要 root 权限。

进入本工程的 `scripts` 目录（将 `/your/workspace` 替换为你的实际路径）：

```bash
cd /your/workspace/robosense_ac_driver/src/modules/ac_driver/scripts
sudo bash AC_usb_permission.sh  # 每台电脑仅需执行一次
```

该脚本会安装 udev 规则，使当前用户能直接访问传感器设备。

> 💡 执行完成后，重新插拔设备即可生效，通常无需重启系统。若设备仍无法识别，可尝试重启 udev 服务或重启电脑。

### 4.2 硬件连接

AC1/AC2 传感器数据传输量较大，必须使用 USB 3.0（或更高）接口直连主机。建议使用原装或高质量 USB 3.0 线缆，避免通过 USB 集线器连接。

连接后，可通过 `lsusb` 确认设备是否被系统识别，应能看到 RoboSense 相关的设备条目。

> 💡 USB 3.0 接口通常为蓝色，并带有 “SS” 标识。若 `lsusb` 未显示设备，请检查：
> - 线缆是否支持数据传输（非仅充电线）；
> - 是否已正确执行权限配置脚本；
> - 设备电源是否正常（观察指示灯）。
>
> 如需图形化工具（仅限带桌面环境的系统），可安装 `usbview`

## 5. 编译 ac_driver

打开一个新的终端窗口，并将当前路径切换至包含 `ac_driver` 源码的工作空间根目录：

```bash
cd /your/workspace/robosense_ac_driver
```

> 💡 请将 `/your/workspace` 替换为你实际存放源码的路径。

在完成前置依赖后，可根据您使用的 ROS 版本选择对应的编译方式。

### ROS2

加载 ROS2 环境变量，然后使用 `colcon` 进行构建：

```bash
source /opt/ros/humble/setup.bash
rm -rf build/ install/ log/  # （可选）清理旧构建文件
colcon build
```

### ROS

加载 ROS 环境变量，然后使用 `catkin_make` 进行构建：

```bash
source /opt/ros/noetic/setup.bash
rm -rf build/ devel/  # （可选）清理旧构建文件
catkin_make
```

## 6. 运行 ac_driver

成功编译 ac_driver 后，就可以启动节点了。请根据所用 ROS 版本执行对应命令。

### ROS2

- 对于 AC1
```bash
source install/setup.bash
ros2 launch ac_driver start_ac1.launch.py start_rviz_node:=false|true 
```

- 对于 AC2
```bash
source install/setup.bash
ros2 launch ac_driver start_ac2_usb.launch.py start_rviz_node:=false|true 
或 
ros2 launch ac_driver start_ac2_gmsl.launch.py start_rviz_node:=false|true 
```

### ROS

- 对于 AC1
```bash
source devel/setup.bash
roslaunch ac_driver start_ac1.launch start_rviz_node:=false|true 
```

- 对于 AC2
```bash
source devel/setup.bash
roslaunch ac_driver start_ac2_usb.launch start_rviz_node:=false|true 
或 
roslaunch ac_driver start_ac2_gmsl.launch start_rviz_node:=false|true 
```

## 7. 话题名称及消息类型

> 💡 说明：下方表格中，每行“话题类型”第一行为 ROS2 格式，第二行为 ROS 格式。

对于 AC1

| 话题名称                                      | 话题类型                                                               | 含义 |
|----------------------------------------------|----------------------------------------------------------------------|-----|
| `/rs_camera/color/image_raw`                 | `sensor_msgs/msg/Image` <br> `sensor_msgs/Image`                     | 彩色相机原始图像数据 |
| `/rs_camera/color/image_raw/compressed`      | `sensor_msgs/msg/CompressedImage` <br> `sensor_msgs/CompressedImage` | 彩色相机原始图像的压缩版本 |
| `/rs_camera/rect/color/image_raw`            | `sensor_msgs/msg/Image` <br> `sensor_msgs/Image`                     | 经过校正后的彩色相机图像数据 |
| `/rs_camera/rect/color/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` <br> `sensor_msgs/CompressedImage` | 经过校正后的彩色相机图像数据的压缩版本 |
| `/rs_lidar/points`                           | `sensor_msgs/msg/PointCloud2` <br> `sensor_msgs/PointCloud2`         | 点云数据 frame_id 为 rslidar |
| `/rs_imu`                                    | `sensor_msgs/msg/Imu` <br> `sensor_msgs/Imu`                         | IMU（惯性测量单元）数据 |

对于 AC2

| 话题名称                                            | 话题类型                                                               | 含义 |
|----------------------------------------------------|----------------------------------------------------------------------|-----|
| `/rs_camera/left/color/image_raw`                  | `sensor_msgs/msg/Image` <br> `sensor_msgs/Image`                     | 左相机原始图像数据 |
| `/rs_camera/left/color/image_raw/compressed`       | `sensor_msgs/msg/CompressedImage` <br> `sensor_msgs/CompressedImage` | 左相机原始图像的压缩版本 |
| `/rs_camera/left/rect/color/image_raw`             | `sensor_msgs/msg/Image` <br> `sensor_msgs/Image`                     | 经过校正后的左相机图像数据 |
| `/rs_camera/left/rect/color/image_raw/compressed`  | `sensor_msgs/msg/CompressedImage` <br> `sensor_msgs/CompressedImage` | 经过校正后的左相机图像数据的压缩版本 |
| `/rs_camera/right/color/image_raw`                 | `sensor_msgs/msg/Image` <br> `sensor_msgs/Image`                     | 右相机原始图像数据 |
| `/rs_camera/right/color/image_raw/compressed`      | `sensor_msgs/msg/CompressedImage` <br> `sensor_msgs/CompressedImage` | 右相机原始图像的压缩版本 |
| `/rs_camera/right/rect/color/image_raw`            | `sensor_msgs/msg/Image` <br> `sensor_msgs/Image`                     | 经过校正后的右相机图像数据 |
| `/rs_camera/right/rect/color/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` <br> `sensor_msgs/CompressedImage` | 经过校正后的右相机图像数据的压缩版本 |
| `/rs_lidar/points`                                 | `sensor_msgs/msg/PointCloud2` <br> `sensor_msgs/PointCloud2`         | 点云数据 frame_id 为 rslidar |
| `/rs_imu`                                          | `sensor_msgs/msg/Imu` <br> `sensor_msgs/Imu`                         | IMU（惯性测量单元）数据 |

## 8. 服务(Service)名称及类型

> 💡 说明：下方表格中，每行“服务类型”第一行为 ROS2 格式，第二行为 ROS 格式。

| 服务名称                       | 服务类型                                     | 含义 |
|--------------------------------|----------------------------------------------|-----|
| `/rs_lidar/set_laser_control`  | `std_srvs/srv/SetBool` <br> `std_srvs/SetBool` | 控制激光发射的开关（仅支持 RS-AC1）<br>请求字段 `data`：`true` 开启激光，`false` 关闭激光<br>响应字段 `success`/`message` 表示操作结果 |

> 详情可查阅 [AC Driver README_CN.md](src/modules/ac_driver/README_CN.md#43-服务service名称和类型说明)
