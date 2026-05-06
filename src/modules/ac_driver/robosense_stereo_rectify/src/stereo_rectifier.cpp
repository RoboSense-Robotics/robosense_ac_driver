#include "stereo_rectifier.h"
#include <iostream>
#include <algorithm>
#include <cmath>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

namespace robosense {

cv::Mat StereoRectifier::DistortionToCv(const std::vector<double>& dist, int target_size) {
    // OpenCV calibration APIs accept distortion as Nx1 double matrix.
    const int out_size = std::max(0, target_size);
    cv::Mat out = cv::Mat::zeros(out_size, 1, CV_64F);
    const int copy_size = std::min<int>(dist.size(), out_size);
    for (int i = 0; i < copy_size; ++i) {
        out.at<double>(i, 0) = dist[i];
    }
    return out;
}

cv::Matx33d StereoRectifier::QuaternionToRotation(const cv::Vec4d& quaternion) {
    // Quaternion storage convention in this project is (w, x, y, z).
    double w = quaternion[0];
    double x = quaternion[1];
    double y = quaternion[2];
    double z = quaternion[3];
    const double n = std::sqrt(w * w + x * x + y * y + z * z);
    if (n <= 0.0) {
        return cv::Matx33d::eye();
    }
    w /= n;
    x /= n;
    y /= n;
    z /= n;

    const double xx = x * x;
    const double yy = y * y;
    const double zz = z * z;
    const double xy = x * y;
    const double xz = x * z;
    const double yz = y * z;
    const double wx = w * x;
    const double wy = w * y;
    const double wz = w * z;

    return cv::Matx33d(
        1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz),       2.0 * (xz + wy),
        2.0 * (xy + wz),       1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx),
        2.0 * (xz - wy),       2.0 * (yz + wx),       1.0 - 2.0 * (xx + yy));
}

cv::Vec4d StereoRectifier::RotationToQuaternion(const cv::Matx33d& rotation) {
    // Robust matrix->quaternion conversion with branch selection on major diagonal.
    const double trace = rotation(0, 0) + rotation(1, 1) + rotation(2, 2);
    double w = 1.0;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    if (trace > 0.0) {
        const double s = std::sqrt(trace + 1.0) * 2.0;
        w = 0.25 * s;
        x = (rotation(2, 1) - rotation(1, 2)) / s;
        y = (rotation(0, 2) - rotation(2, 0)) / s;
        z = (rotation(1, 0) - rotation(0, 1)) / s;
    } else if (rotation(0, 0) > rotation(1, 1) && rotation(0, 0) > rotation(2, 2)) {
        const double s = std::sqrt(1.0 + rotation(0, 0) - rotation(1, 1) - rotation(2, 2)) * 2.0;
        w = (rotation(2, 1) - rotation(1, 2)) / s;
        x = 0.25 * s;
        y = (rotation(0, 1) + rotation(1, 0)) / s;
        z = (rotation(0, 2) + rotation(2, 0)) / s;
    } else if (rotation(1, 1) > rotation(2, 2)) {
        const double s = std::sqrt(1.0 + rotation(1, 1) - rotation(0, 0) - rotation(2, 2)) * 2.0;
        w = (rotation(0, 2) - rotation(2, 0)) / s;
        x = (rotation(0, 1) + rotation(1, 0)) / s;
        y = 0.25 * s;
        z = (rotation(1, 2) + rotation(2, 1)) / s;
    } else {
        const double s = std::sqrt(1.0 + rotation(2, 2) - rotation(0, 0) - rotation(1, 1)) * 2.0;
        w = (rotation(1, 0) - rotation(0, 1)) / s;
        x = (rotation(0, 2) + rotation(2, 0)) / s;
        y = (rotation(1, 2) + rotation(2, 1)) / s;
        z = 0.25 * s;
    }

    const double n = std::sqrt(w * w + x * x + y * y + z * z);
    if (n <= 0.0) {
        return cv::Vec4d(1.0, 0.0, 0.0, 0.0);
    }
    return cv::Vec4d(w / n, x / n, y / n, z / n);
}

bool StereoRectifier::Initialize(const calibration::DeviceCalibration& calibration,
                                 const int rectify_mode,
                                 double alpha,
                                 const cv::Size& new_image_size
                                 ) {
    calibration_ = calibration;
    rectify_mode_ = rectify_mode;
    const auto& left_cam = calibration.sensor.camera;
    const auto& right_cam = calibration.sensor.camera_r;

    raw_image_size_ = left_cam.intrinsic.image_size;
    if (raw_image_size_.width <= 0 || raw_image_size_.height <= 0) {
        initialized_ = false;
        return false;
    }
    if (right_cam.intrinsic.image_size != raw_image_size_) {
        initialized_ = false;
        return false;
    }
    if (raw_image_size_.width == 1280 && raw_image_size_.height == 960) {
        // 1280x960 calibration is configured to passthrough mode. for AC2 A0 device
        rectify_mode_ = -1;
    }

    rectified_image_size_ = new_image_size;
    if (rectified_image_size_.width <= 0 || rectified_image_size_.height <= 0 || rectify_mode_ == -1) {
        rectified_image_size_ = raw_image_size_;
    }   
    R1_ = cv::Mat(cv::Matx33d::eye());
    R2_ = cv::Mat(cv::Matx33d::eye());

    if(rectify_mode_ >= 0){
        try {
        const cv::Mat cam_mat1(left_cam.intrinsic.int_matrix);
        const cv::Mat cam_mat2(right_cam.intrinsic.int_matrix);
        const cv::Mat dist1 = DistortionToCv(left_cam.intrinsic.dist_coeff);
        const cv::Mat dist2 = DistortionToCv(right_cam.intrinsic.dist_coeff);

        const cv::Matx33d R_mat = QuaternionToRotation(right_cam.extrinsic.quaternion);
        const cv::Mat R(R_mat);
        cv::Mat T(3, 1, CV_64F);
        T.at<double>(0, 0) = right_cam.extrinsic.translation[0];
        T.at<double>(1, 0) = right_cam.extrinsic.translation[1];
        T.at<double>(2, 0) = right_cam.extrinsic.translation[2];

        const cv::Size image_size(raw_image_size_.width, raw_image_size_.height);
        const cv::Size rectify_size(rectified_image_size_.width, rectified_image_size_.height);
        // Input calibration stores right->left extrinsic; stereoRectify expects left->right.
        cv::Mat R_inv = R.inv();
        cv::Mat T_inv = -R_inv * T;
        
        if(rectify_mode_ == 0){
            cv::stereoRectify(
            cam_mat1, dist1, cam_mat2, dist2, image_size,
            R_inv, T_inv,
            R1_, R2_, P1_, P2_, Q_,
            cv::CALIB_ZERO_DISPARITY, alpha, rectify_size, nullptr, nullptr);
        } else if(rectify_mode_ == 1){
            P1_ = cv::getOptimalNewCameraMatrix(cam_mat1, dist1, image_size, alpha, rectify_size);
            P2_ = cv::getOptimalNewCameraMatrix(cam_mat2, dist2, image_size, alpha, rectify_size);
        }       
 
        cv::initUndistortRectifyMap(
            cam_mat1, dist1, R1_, P1_, rectify_size, CV_32FC1, left_map1_, left_map2_);
        cv::initUndistortRectifyMap(
            cam_mat2, dist2, R2_, P2_, rectify_size, CV_32FC1, right_map1_, right_map2_);

        } catch (const cv::Exception&) {
            initialized_ = false;
            return false;
        }
    }    

    initialized_ = true;
    return true;
}

calibration::DeviceCalibration StereoRectifier::GetRectifiedCalibrationInfo() {
    // Start from original calibration and overwrite fields that change after rectification.
    calibration::DeviceCalibration rect_calibration = calibration_;

    if(rectify_mode_ < 0) return rect_calibration;

    auto& left = rect_calibration.sensor.camera;
    auto& right = rect_calibration.sensor.camera_r;

    left.intrinsic.model = "Pinhole";
    right.intrinsic.model = "Pinhole";
    left.intrinsic.image_size = rectified_image_size_;
    right.intrinsic.image_size = rectified_image_size_;

    const double fx1 = P1_.at<double>(0, 0);
    const double fy1 = P1_.at<double>(1, 1);
    const double cx1 = P1_.at<double>(0, 2);
    const double cy1 = P1_.at<double>(1, 2);
    left.intrinsic.int_matrix = cv::Matx33d(
        fx1, 0.0, cx1,
        0.0, fy1, cy1,
        0.0, 0.0, 1.0);

    const double fx2 = P2_.at<double>(0, 0);
    const double fy2 = P2_.at<double>(1, 1);
    const double cx2 = P2_.at<double>(0, 2);
    const double cy2 = P2_.at<double>(1, 2);
    right.intrinsic.int_matrix = cv::Matx33d(
        fx2, 0.0, cx2,
        0.0, fy2, cy2,
        0.0, 0.0, 1.0);

    left.intrinsic.dist_coeff.assign(14, 0.0);
    right.intrinsic.dist_coeff.assign(14, 0.0);

    // In single-camera undistortion mode, keep original camera extrinsics unchanged.
    if(rectify_mode_ == 1) {
        return rect_calibration;
    }

    // Rectified stereo uses aligned camera axes; baseline is encoded in P2(0,3).
    const double baseline_x = (std::abs(fx2) > 1e-12) ? (-P2_.at<double>(0, 3) / fx2) : 0.0;
    right.extrinsic.translation = cv::Vec3d(baseline_x, 0.0, 0.0);
    right.extrinsic.quaternion = cv::Vec4d(1.0, 0.0, 0.0, 0.0);

    // Transform from rectified-left frame back to original-left frame.
    const cv::Mat Rotate_recL_to_L = R1_;

    cv::Matx44d T_recL2L_4x4 = cv::Matx44d::eye();
    cv::Mat T_recL2L(T_recL2L_4x4);
    for(int v=0; v<3; v++)
    {
      for(int u=0; u<3; u++)
      {
        T_recL2L.at<double>(v,u) = Rotate_recL_to_L.at<double>(v,u);
      }
    }
    cv::Mat Rotate_lidar_to_L(QuaternionToRotation(left.extrinsic.quaternion));
    cv::Mat Trans_lidar_to_L(3, 1, CV_64F);
    Trans_lidar_to_L.at<double>(0, 0) = left.extrinsic.translation[0];
    Trans_lidar_to_L.at<double>(1, 0) = left.extrinsic.translation[1];
    Trans_lidar_to_L.at<double>(2, 0) = left.extrinsic.translation[2];
    
    // Compose LiDAR->left-camera in homogeneous form for subsequent frame conversion.
    cv::Matx44d T_lidar2leftcam_4x4 = cv::Matx44d::eye();
    cv::Mat T_lidar2leftcam(T_lidar2leftcam_4x4);
    for(int v=0; v<3; v++)
    {
      for(int u=0; u<3; u++)
      {
        T_lidar2leftcam.at<double>(v,u) = Rotate_lidar_to_L.at<double>(v,u);
      }
    }
    for(int i=0; i<3; i++)
    {
      T_lidar2leftcam.at<double>(i, 3) = Trans_lidar_to_L.at<double>(i,0);
    }
    T_lidar2leftcam = T_lidar2leftcam * T_recL2L.inv();

    for(int v=0; v<3; v++)
    {
      for(int u=0; u<3; u++)
      {
        Rotate_lidar_to_L.at<double>(v,u) = T_lidar2leftcam.at<double>(v,u);
      }
    }
    std::cout << "T_lidar2leftcam: " << T_lidar2leftcam << std::endl;
    left.extrinsic.quaternion = RotationToQuaternion(Rotate_lidar_to_L);
    left.extrinsic.translation[0] =  T_lidar2leftcam.at<double>(0, 3);
    left.extrinsic.translation[1] =  T_lidar2leftcam.at<double>(1, 3);
    left.extrinsic.translation[2] =  T_lidar2leftcam.at<double>(2, 3);

    return rect_calibration;
}

bool StereoRectifier::Remap(const cv::Mat& left_raw,
                            const cv::Mat& right_raw,
                            cv::Mat* left_rectified,
                            cv::Mat* right_rectified) const {
    if (!initialized_ || left_rectified == nullptr || right_rectified == nullptr) {
        return false;
    }
    if (left_raw.empty() || right_raw.empty()) {
        return false;
    }
    if (left_raw.type() != CV_8UC3 || right_raw.type() != CV_8UC3) {
        return false;
    }
    if (left_raw.cols != raw_image_size_.width || left_raw.rows != raw_image_size_.height ||
        right_raw.cols != raw_image_size_.width || right_raw.rows != raw_image_size_.height) {
        return false;
    }

    if(rectify_mode_ >=0 ){
        try {
            left_rectified->create(rectified_image_size_.height, rectified_image_size_.width, CV_8UC3);
            right_rectified->create(rectified_image_size_.height, rectified_image_size_.width, CV_8UC3);

            cv::remap(left_raw, *left_rectified, left_map1_, left_map2_, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar());
            cv::remap(right_raw, *right_rectified, right_map1_, right_map2_, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar());
        } catch (const cv::Exception&) {
            return false;
        }
    } else {
        left_rectified->create(rectified_image_size_.height, rectified_image_size_.width, CV_8UC3);
        right_rectified->create(rectified_image_size_.height, rectified_image_size_.width, CV_8UC3);
        *left_rectified = left_raw.clone();
        *right_rectified = right_raw.clone();
    }
    
    return true;
}


bool StereoRectifier::SingleImageRemap(const int cam_idx, const cv::Mat& img_raw, cv::Mat* img_rectified) const {
    if (!initialized_ || img_rectified == nullptr) {
        return false;
    }
    if (img_raw.empty()) {
        return false;
    }
    if (img_raw.type() != CV_8UC3) {
        return false;
    }
    if (img_raw.cols != raw_image_size_.width || img_raw.rows != raw_image_size_.height) {
        return false;
    }
    if(cam_idx != 0 && cam_idx !=1){
        return false;
    }
    if(rectify_mode_ >=0 ){
        try {
            img_rectified->create(rectified_image_size_.height, rectified_image_size_.width, CV_8UC3);
            if(cam_idx == 0){
                cv::remap(img_raw, *img_rectified, left_map1_, left_map2_, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar());
            } else if(cam_idx == 1){
                cv::remap(img_raw, *img_rectified, right_map1_, right_map2_, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar());
            }
            
        } catch (const cv::Exception&) {
            return false;
        }
    } else {
        img_rectified->create(rectified_image_size_.height, rectified_image_size_.width, CV_8UC3);
        *img_rectified = img_raw.clone();
    }
    return true;
}

}  // namespace robosense
