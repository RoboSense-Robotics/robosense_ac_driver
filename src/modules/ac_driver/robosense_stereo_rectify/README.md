# robosense_stereo_rectify

基于 OpenCV 的双目图像矫正工具，支持：
- 双目立体矫正（`rectify_mode = 0`）
- 单目去畸变（`rectify_mode = 1`）
- 透传模式（`rectify_mode = -1`，不做矫正）

工程同时提供了一个 demo，可读取标定 YAML、执行图像重映射，并导出矫正后的标定结果。

## 目录结构

```text
.
├── include/stereo_rectifier.h   # 核心接口
├── src/stereo_rectifier.cpp     # 核心实现
├── demo/example.cpp             # 示例程序
├── config/*.yaml                # 标定样例
└── data/*.png|jpg               # 测试图像
```

## 编译

```bash
mkdir -p build
cd build
cmake ..
make -j
```

## 运行 Demo

在 `build/` 目录下执行：

```bash
./stereo_rectify_demo <left_img> <right_img> [calibration.yaml] [rectified_out.yaml] [mode]
```

示例：

```bash
./stereo_rectify_demo ../data/left.png ../data/right.png \
  config/1111BEDC0028_calibration.yaml \
  config/1111BEDC0028_rectified_calibration.yaml \
  stereo
```

输出文件：
- `left_rect.png`
- `right_rect.png`（`mode=stereo` 时）
- 矫正后标定 YAML（`rectified_out.yaml`）


## 核心接口

头文件：`include/stereo_rectifier.h`

主要类：
- `robosense::StereoRectifier`

核心方法：
- `bool Initialize(const DeviceCalibration&, int rectify_mode=1, double alpha=0.0, const cv::Size& new_image_size=cv::Size())`
- `bool Remap(const cv::Mat& left_raw, const cv::Mat& right_raw, cv::Mat* left_rectified, cv::Mat* right_rectified) const`
- `bool SingleImageRemap(int cam_idx, const cv::Mat& img_raw, cv::Mat* img_rectified) const`
- `DeviceCalibration GetRectifiedCalibrationInfo()`

其中：
- `rectify_mode = 0`：立体矫正（调用 `cv::stereoRectify`）
- `rectify_mode = 1`：仅更新相机内参并去畸变（调用 `cv::getOptimalNewCameraMatrix`）
- `rectify_mode = -1`：不计算映射，输出与输入一致
- `alpha`：OpenCV 裁剪系数，`0.0 ~ 1.0`

## 默认模式

AC1：暂不进行畸变矫正：rectify_mode = -1，其他参数默认。

AC2：畸变矫正使用单目矫正模式：rectify_mode = 1， alpha = 0， new_image_size = cv::Size(1920, 1080)。
