# robosense_ac_driver

[中文文档](README_CN.md)

## 1. Introduction

`robosense_ac_driver` is a ROS/ROS2-based driver package designed to support RoboSense AC1/AC2 sensors. It provides device drivers along with functionalities for compressing and decompressing associated camera image messages.

## 2. Prerequisites

Supported Typical Configurations:

  - Ubuntu 22.04 (Jammy Jellyfish) + ROS2 Humble (pre-compiled installation)  
    Installation guide: [ROS2 Humble Official Installation Guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)

  - Ubuntu 20.04 (Focal Fossa) + ROS2 Humble (built from source)  
    Installation guide: [ROS2 Humble Official Installation Guide](https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html)  

  - Ubuntu 20.04 (Focal Fossa) + ROS Noetic (pre-compiled installation)  
    Installation guide: [ROS Noetic Official Installation Guide](https://wiki.ros.org/noetic/Installation/Ubuntu)

> 💡 Note: If using other Ubuntu versions (e.g., 22.04 or 24.04), please select the corresponding ROS2 distribution and handle compatibility adjustments yourself.

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

## 5. Build ac_driver

Open a new terminal window and change the current directory to the root of your workspace containing the `ac_driver` source code:

```bash
cd /your/workspace/robosense_ac_driver
```

> 💡 Please replace `/your/workspace` with the actual path where your source code is stored.

After installing all prerequisites, choose the appropriate build method based on your ROS version.

### ROS2

Source the ROS2 environment and build using `colcon`:

```bash
source /opt/ros/humble/setup.bash
rm -rf build/ install/ log/  # (Optional) Clean previous build files
colcon build
```

### ROS

Source the ROS environment and build using `catkin_make`:

```bash
source /opt/ros/noetic/setup.bash
rm -rf build/ devel/  # (Optional) Clean previous build files
catkin_make
```

## 6. Run ac_driver

Once `ac_driver` has been successfully built, you can launch the node. Execute the corresponding command based on your ROS version.

### ROS2

- For AC1:
```bash
source install/setup.bash
ros2 launch ac_driver start_ac1.launch.py
```

- For AC2:
```bash
source install/setup.bash
ros2 launch ac_driver start_ac2_usb.launch.py
```

### ROS

- For AC1:
```bash
source devel/setup.bash
roslaunch ac_driver start_ac1.launch
```

- For AC2:
```bash
source devel/setup.bash
roslaunch ac_driver start_ac2_usb.launch
```

## 7. Topic Name And Data Type 

> 💡 Note: In the table below, each "Topic Type" row contains the ROS2 format first, followed by the ROS format.

For AC1
| Topic Name                                   | Topic Type                                                           | Description                      |
|----------------------------------------------|----------------------------------------------------------------------|----------------------------------|
| `/rs_camera/color/image_raw`                 | `sensor_msgs/msg/Image` <br> `sensor_msgs/Image`                     | Raw color camera image data |
| `/rs_camera/color/image_raw/compressed`      | `sensor_msgs/msg/CompressedImage` <br> `sensor_msgs/CompressedImage` | Compressed version of raw color camera image data |
| `/rs_camera/rect/color/image_raw`            | `sensor_msgs/msg/Image` <br> `sensor_msgs/Image`                     | Rectified color camera image data |
| `/rs_camera/rect/color/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` <br> `sensor_msgs/CompressedImage` | Compressed version of rectified color camera image data |
| `/rs_lidar/points`                           | `sensor_msgs/msg/PointCloud2` <br> `sensor_msgs/PointCloud2`         | Point cloud data with frame_id as rslidar |
| `/rs_imu`                                    | `sensor_msgs/msg/Imu` <br> `sensor_msgs/Imu`                         | IMU (Inertial Measurement Unit) data |

For AC2
| Topic Name                                         | Topic Type                                                           | Description                      |
|----------------------------------------------------|----------------------------------------------------------------------|----------------------------------|
| `/rs_camera/left/color/image_raw`                  | `sensor_msgs/msg/Image` <br> `sensor_msgs/Image`                     | Raw left camera image data |
| `/rs_camera/left/color/image_raw/compressed`       | `sensor_msgs/msg/CompressedImage` <br> `sensor_msgs/CompressedImage` | Compressed version of raw left camera image data |
| `/rs_camera/left/rect/color/image_raw`             | `sensor_msgs/msg/Image` <br> `sensor_msgs/Image`                     | Rectified left camera image data |
| `/rs_camera/left/rect/color/image_raw/compressed`  | `sensor_msgs/msg/CompressedImage` <br> `sensor_msgs/CompressedImage` | Compressed version of rectified left camera image data |
| `/rs_camera/right/color/image_raw`                 | `sensor_msgs/msg/Image` <br> `sensor_msgs/Image`                     | Raw right camera image data |
| `/rs_camera/right/color/image_raw/compressed`      | `sensor_msgs/msg/CompressedImage` <br> `sensor_msgs/CompressedImage` | Compressed version of raw right camera image data |
| `/rs_camera/right/rect/color/image_raw`            | `sensor_msgs/msg/Image` <br> `sensor_msgs/Image`                     | Rectified right camera image data |
| `/rs_camera/right/rect/color/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` <br> `sensor_msgs/CompressedImage` | Compressed version of rectified right camera image data |
| `/rs_lidar/points`                                 | `sensor_msgs/msg/PointCloud2` <br> `sensor_msgs/PointCloud2`         | Point cloud data with frame_id as rslidar |
| `/rs_imu`                                          | `sensor_msgs/msg/Imu` <br> `sensor_msgs/Imu`                         | IMU (Inertial Measurement Unit) data |