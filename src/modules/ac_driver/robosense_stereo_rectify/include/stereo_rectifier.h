#pragma once

#include <cstdint>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>

namespace robosense {

namespace calibration {

struct Extrinsic {
    cv::Vec3d translation = cv::Vec3d(0.0, 0.0, 0.0);
    cv::Vec4d quaternion = cv::Vec4d(1.0, 0.0, 0.0, 0.0);  // (w, x, y, z)
};

struct CameraIntrinsic {
    std::string model;
    cv::Matx33d int_matrix = cv::Matx33d::eye();
    std::vector<double> dist_coeff = std::vector<double>(14, 0.0);
    cv::Size image_size;
};

struct LidarCalibration {
    std::string base_link;
    std::string frame;
    Extrinsic extrinsic;
};

struct CameraCalibration {
    std::string base_link;
    std::string frame;
    Extrinsic extrinsic;
    CameraIntrinsic intrinsic;
};

struct ImuCalibration {
    std::string base_link;
    std::string frame;
    Extrinsic extrinsic;
};

struct SensorCalibration {
    LidarCalibration lidar;
    CameraCalibration camera;
    CameraCalibration camera_r;
    ImuCalibration imu;
};

struct DeviceCalibration {
    std::string device_id;
    std::string cali_time;
    SensorCalibration sensor;
};

}  // namespace calibration


class StereoRectifier {
public:
    StereoRectifier() = default;
    ~StereoRectifier() = default;

    bool Initialize(const calibration::DeviceCalibration& calibration,
                    const int rectify_mode = 1,   // raw/stereo/single undistortion = -1/0/1
                    double alpha = 0.0,           // sensible/keep all pixels = 0.0~1.0
                    const cv::Size& new_image_size = cv::Size(1920, 1080)
                    );

    bool Remap(const cv::Mat& left_raw,
               const cv::Mat& right_raw,
               cv::Mat* left_rectified,
               cv::Mat* right_rectified) const;

    bool SingleImageRemap(const int cam_idx,   // left/right = 0/1
                          const cv::Mat& img_raw, 
                          cv::Mat* img_rectified) const;

    calibration::DeviceCalibration GetRectifiedCalibrationInfo();

    bool IsInitialized() const { return initialized_; }

private:
    static cv::Mat DistortionToCv(const std::vector<double>& dist, int target_size = 8);
    static cv::Matx33d QuaternionToRotation(const cv::Vec4d& quaternion);
    static cv::Vec4d RotationToQuaternion(const cv::Matx33d& rotation);

private:
    bool initialized_ = false;
    int rectify_mode_ = 0;
    calibration::DeviceCalibration calibration_;
    cv::Size raw_image_size_;
    cv::Size rectified_image_size_;

    cv::Mat R1_;
    cv::Mat R2_;
    cv::Mat P1_;
    cv::Mat P2_;
    cv::Mat Q_;
    
    cv::Mat left_map1_;
    cv::Mat left_map2_;
    cv::Mat right_map1_;
    cv::Mat right_map2_;
};

}  // namespace robosense
