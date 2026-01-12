# robosense_ac_driver

[中文文档](README_CN.md)

## 1. Introduction

`robosense_ac_driver` is a ROS/ROS2-based driver package designed to support RoboSense AC1/AC2 sensors. It provides device drivers along with functionalities for compressing and decompressing associated camera image messages.

## 2. Prerequisites

Supported Typical Configurations:
  - Ubuntu 20.04 (Focal Fossa) + ROS 2 Humble (built from source)

    Installation instructions: [ROS 2 Humble Official Installation Guide](https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html)  
    > 💡 Note: If using other Ubuntu versions (e.g., 22.04 or 24.04), please select the corresponding ROS 2 distribution and handle compatibility adjustments yourself.

  - Ubuntu 20.04 (Focal Fossa) + ROS Noetic (pre-compiled installation)  
  
    Installation instructions: [ROS Noetic Official Installation Guide](https://wiki.ros.org/noetic/Installation/Ubuntu)

## 3. Code Download

This project includes Git submodules. Please clone the repository completely using the following commands:

```bash
# Clone the main repository (including submodules)
git clone --recursive https://github.com/RoboSense-Robotics/robosense_ac_driver.git

# If already cloned but submodules not initialized, update manually
cd robosense_ac_driver
git submodule update --init --recursive
```

## 4. Permission Setup and Hardware Connection

### 4.1 Permission Configuration

When connecting an AC1/AC2 sensor for the first time, USB device access permissions must be configured to avoid requiring root privileges when running the driver later.

Navigate to the `scripts` directory in this project (replace `/your/workspace` with your actual path):

```bash
cd /your/workspace/robosense_ac_driver/src/modules/ac_driver/scripts
sudo bash AC_usb_permission.sh  # Run this script only once per machine
```

This script installs udev rules that allow the current user to directly access the sensor device.

> 💡 After execution, simply unplug and replug the device for the changes to take effect, usually no system reboot is required. If the device is still not recognized, try restarting the udev service or rebooting your computer.

### 4.2 Hardware Connection

The AC1/AC2 sensors generate large amounts of data and must be connected directly to a USB 3.0 (or higher) port on the host machine.

- Use the original or a high-quality USB 3.0 cable.
- Avoid connecting through a USB hub.

After connection, verify device recognition using `lsusb` You should see an entry related to RoboSense.

> 💡 USB 3.0 ports are typically blue and marked with an "SS" (SuperSpeed) symbol.
> If `lsusb` does not show the device, check:
> - Whether the cable supports data transfer (not just charging).
> - Whether the permission script was executed correctly.
> - Whether the device is powered on (check the status LED).
>
> For systems with a desktop environment, you may optionally install a graphical tool like `usbview`.

## 5. Building ac_driver

Once prerequisites are met, proceed to build the `ac_driver`.

1. Open a new terminal and navigate to the root directory of your workspace containing the `ac_driver` source code:

```bash
cd /your/workspace/robosense_ac_driver
```

> 💡 Replace `/your/workspace` with your actual path.

2. Load the ROS 2 Humble environment and build using `colcon`:

```bash
source /opt/ros/humble/setup.bash
rm -rf build/ install/ log/  # (Optional) Clean previous build artifacts
colcon build
```

## 6. Running ac_driver

After successful compilation, you can launch the driver node.

1. Setting Up the Runtime Environment

Before launching the `ac_driver` node, load the environment variables generated during the build. Open a new terminal and run:

```bash
source install/setup.bash
```

2. Launching the Node

- For AC1:
  ```bash
  ros2 launch ac_driver start_ac1.launch.py
  ```

- For AC2:
  ```bash
  ros2 launch ac_driver start_ac2_usb.launch.py
  ```

## 7. Topic Name And Data Type 

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
