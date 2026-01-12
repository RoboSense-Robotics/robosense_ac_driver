# robosense_ac_driver

[English Version](README.md)

## 1. 简介

robosense_ac_driver 是一个基于 ROS/ROS2 的驱动包，用于支持 RoboSense AC1/AC2 传感器，提供设备驱动以及配套的相机消息压缩与解压缩功能。

## 2. 前置依赖

支持的典型配置：
  - Ubuntu 20.04 (Focal Fossa) + ROS 2 Humble (源码编译)

    安装请参考：[ROS 2 Humble 官方安装文档](https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html)
    > 💡 提示：若使用其他 Ubuntu 版本（如 22.04 或 24.04），需选择对应的 ROS 2 发行版，并自行适配兼容性。

  - Ubuntu 20.04 (Focal Fossa) + ROS Noetic (预编译安装)

    安装请参考：[ROS Noetic 官方安装文档](https://wiki.ros.org/noetic/Installation/Ubuntu)

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

在完成前置依赖后，可开始编译 ac_driver 驱动程序。

1. 打开一个新的终端窗口，并将当前路径切换至包含 `ac_driver` 源码的工作空间根目录：

```bash
cd /your/workspace/robosense_ac_driver
```

> 💡 请将 `/your/workspace` 替换为你实际存放源码的路径。

2. 加载已安装的 ROS 2 Humble 环境变量，然后使用 `colcon` 进行构建：

```bash
source /opt/ros/humble/setup.bash
rm -rf build/ install/ log/  # （可选）若工作空间中已存在 install 或 build 目录
colcon build
```

## 6. 运行 ac_driver

成功编译 ac_driver 后，就可以启动节点了。

1. 设置运行环境

在启动 `ac_driver` 节点之前，首先需要加载编译生成的环境变量。这将确保当前终端能够识别并运行新构建的包。打开一个新终端，执行以下命令：

```bash
source install/setup.bash
```

2. 启动节点

- 对于 AC1：
  ```bash
  ros2 launch ac_driver start_ac1.launch.py
  ```

- 对于 AC2：
  ```bash
  ros2 launch ac_driver start_ac2_usb.launch.py
  ```

## 7. 话题名称及消息类型

- /rs_camera/color/image_raw                                         ->  sensor_msgs(::msg):::Image
- /rs_camera/color/image_raw/compressed                  -> sensor_msgs(::msg)::CompressedImage  
- /rs_camera/rect/color/image_raw                                 -> sensor_msgs(::msg):::Image
- /rs_camera/rect/color/image_raw/compressed          -> sensor_msgs(::msg)::CompressedImage 
- /rs_camera/left/color/image_raw                                   ->  sensor_msgs(::msg):::Image 
- /rs_camera/left/color/image_raw/compressed            -> sensor_msgs(::msg)::CompressedImage 
- /rs_camera/left/rect/color/image_raw                           -> sensor_msgs(::msg):::Image
- /rs_camera/left/rect/color/image_raw/compressed    ->  sensor_msgs(::msg)::CompressedImage 
- /rs_camera/right/color/image_raw                                 -> sensor_msgs(::msg):::Image 
- /rs_camera/right/color/image_raw/compressed          -> sensor_msgs(::msg)::CompressedImage  
- /rs_camra/right/rect/color/image_raw                           -> sensor_msgs(::msg):::Image 
- /rs_camera/right/rect/color/image_raw/compressed -> sensor_msgs(::msg)::CompressedImage  
- /rs_lidar/points                                                                   -> sensor_msgs(::msg):::PointCloud2, 其中点云的frame_id为"rslidar"
- /rs_imu                                                                                 -> sensor_msgs(::msg):::Imu  
