#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

#include "stereo_rectifier.h"

namespace {

bool LoadExtrinsic(const YAML::Node& extr_node, robosense::calibration::Extrinsic* extrinsic) {
    if (!extr_node || extrinsic == nullptr) {
        return false;
    }

    const YAML::Node t_node = extr_node["translation"];
    const YAML::Node q_node = extr_node["quaternion"];
    if (!t_node || !q_node) {
        return false;
    }

    extrinsic->translation = cv::Vec3d(
        t_node["x"].as<double>(),
        t_node["y"].as<double>(),
        t_node["z"].as<double>());

    // YAML stores quaternion fields as x/y/z/w; convert into internal (w, x, y, z).
    extrinsic->quaternion = cv::Vec4d(
        q_node["w"].as<double>(),
        q_node["x"].as<double>(),
        q_node["y"].as<double>(),
        q_node["z"].as<double>());
    return true;
}

bool LoadCameraCalibration(const YAML::Node& cam_node, robosense::calibration::CameraCalibration* camera) {
    if (!cam_node || camera == nullptr) {
        return false;
    }

    camera->base_link = cam_node["base_link"].as<std::string>();
    camera->frame = cam_node["frame"].as<std::string>();
    if (!LoadExtrinsic(cam_node["extrinsic"], &camera->extrinsic)) {
        return false;
    }

    const YAML::Node intrinsic = cam_node["intrinsic"];
    if (!intrinsic) {
        return false;
    }

    camera->intrinsic.model = intrinsic["model"].as<std::string>();

    const YAML::Node int_matrix = intrinsic["int_matrix"];
    if (!int_matrix || int_matrix.size() != 9) {
        return false;
    }
    cv::Matx33d k;
    for (int i = 0; i < 9; ++i) {
        // YAML int_matrix is a flattened 3x3 row-major sequence.
        k.val[i] = int_matrix[i].as<double>();
    }
    camera->intrinsic.int_matrix = k;

    const YAML::Node dist_coeff = intrinsic["dist_coeff"];
    if (!dist_coeff) {
        return false;
    }
    camera->intrinsic.dist_coeff.clear();
    camera->intrinsic.dist_coeff.reserve(dist_coeff.size());
    for (const auto& v : dist_coeff) {
        camera->intrinsic.dist_coeff.push_back(v.as<double>());
    }

    const YAML::Node image_size = intrinsic["image_size"];
    if (!image_size || image_size.size() != 2) {
        return false;
    }
    camera->intrinsic.image_size = cv::Size(
        image_size[0].as<int>(),
        image_size[1].as<int>());

    return true;
}

bool LoadCalibrationFromFile(const std::string& path, robosense::calibration::DeviceCalibration* calib) {
    if (calib == nullptr) {
        return false;
    }

    try {
        // Expected schema matches config/1111BEDC0028_calibration.yaml.
        const YAML::Node root = YAML::LoadFile(path);
        calib->device_id = root["DEVICE_ID"].as<std::string>();
        calib->cali_time = root["CALI_TIME"].as<std::string>();

        const YAML::Node sensor = root["Sensor"];
        if (!sensor) {
            return false;
        }

        auto& lidar = calib->sensor.lidar;
        const YAML::Node lidar_node = sensor["Lidar"];
        if (!lidar_node) {
            return false;
        }
        lidar.base_link = lidar_node["base_link"].as<std::string>();
        lidar.frame = lidar_node["frame"].as<std::string>();
        if (!LoadExtrinsic(lidar_node["extrinsic"], &lidar.extrinsic)) {
            return false;
        }

        if (!LoadCameraCalibration(sensor["Camera"], &calib->sensor.camera)) {
            return false;
        }
        if (!LoadCameraCalibration(sensor["Camera_R"], &calib->sensor.camera_r)) {
            return false;
        }

        auto& imu = calib->sensor.imu;
        const YAML::Node imu_node = sensor["IMU"];
        if (!imu_node) {
            return false;
        }
        imu.base_link = imu_node["base_link"].as<std::string>();
        imu.frame = imu_node["frame"].as<std::string>();
        if (!LoadExtrinsic(imu_node["extrinsic"], &imu.extrinsic)) {
            return false;
        }
    } catch (const YAML::Exception&) {
        return false;
    }

    return true;
}

void EmitExtrinsic(YAML::Emitter* out, const robosense::calibration::Extrinsic& extrinsic) {
    *out << YAML::Key << "extrinsic" << YAML::Value << YAML::BeginMap;
    *out << YAML::Key << "translation" << YAML::Value << YAML::BeginMap;
    *out << YAML::Key << "x" << YAML::Value << extrinsic.translation[0];
    *out << YAML::Key << "y" << YAML::Value << extrinsic.translation[1];
    *out << YAML::Key << "z" << YAML::Value << extrinsic.translation[2];
    *out << YAML::EndMap;
    *out << YAML::Key << "quaternion" << YAML::Value << YAML::BeginMap;
    *out << YAML::Key << "x" << YAML::Value << extrinsic.quaternion[1];
    *out << YAML::Key << "y" << YAML::Value << extrinsic.quaternion[2];
    *out << YAML::Key << "z" << YAML::Value << extrinsic.quaternion[3];
    *out << YAML::Key << "w" << YAML::Value << extrinsic.quaternion[0];
    *out << YAML::EndMap;
    *out << YAML::EndMap;
}

void EmitCamera(YAML::Emitter* out, const robosense::calibration::CameraCalibration& camera) {
    *out << YAML::Key << "base_link" << YAML::Value << camera.base_link;
    *out << YAML::Key << "frame" << YAML::Value << camera.frame;
    EmitExtrinsic(out, camera.extrinsic);
    *out << YAML::Key << "intrinsic" << YAML::Value << YAML::BeginMap;
    *out << YAML::Key << "model" << YAML::Value << camera.intrinsic.model;

    *out << YAML::Key << "int_matrix" << YAML::Value << YAML::BeginSeq;
    for (int i = 0; i < 9; ++i) {
        *out << camera.intrinsic.int_matrix.val[i];
    }
    *out << YAML::EndSeq;

    *out << YAML::Key << "dist_coeff" << YAML::Value << YAML::BeginSeq;
    for (const double coeff : camera.intrinsic.dist_coeff) {
        *out << coeff;
    }
    *out << YAML::EndSeq;

    *out << YAML::Key << "image_size" << YAML::Value << YAML::BeginSeq
         << camera.intrinsic.image_size.width
         << camera.intrinsic.image_size.height
         << YAML::EndSeq;
    *out << YAML::EndMap;
}

bool SaveCalibrationToFile(const std::string& path, const robosense::calibration::DeviceCalibration& calib) {
    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "DEVICE_ID" << YAML::Value << calib.device_id;
    out << YAML::Key << "CALI_TIME" << YAML::Value << calib.cali_time;
    out << YAML::Key << "Sensor" << YAML::Value << YAML::BeginMap;

    out << YAML::Key << "Lidar" << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "base_link" << YAML::Value << calib.sensor.lidar.base_link;
    out << YAML::Key << "frame" << YAML::Value << calib.sensor.lidar.frame;
    EmitExtrinsic(&out, calib.sensor.lidar.extrinsic);
    out << YAML::EndMap;

    out << YAML::Key << "Camera" << YAML::Value << YAML::BeginMap;
    EmitCamera(&out, calib.sensor.camera);
    out << YAML::EndMap;

    out << YAML::Key << "Camera_R" << YAML::Value << YAML::BeginMap;
    EmitCamera(&out, calib.sensor.camera_r);
    out << YAML::EndMap;

    out << YAML::Key << "IMU" << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "base_link" << YAML::Value << calib.sensor.imu.base_link;
    out << YAML::Key << "frame" << YAML::Value << calib.sensor.imu.frame;
    EmitExtrinsic(&out, calib.sensor.imu.extrinsic);
    out << YAML::EndMap;

    out << YAML::EndMap;
    out << YAML::EndMap;

    std::ofstream ofs(path);
    if (!ofs.is_open()) {
        return false;
    }
    ofs << out.c_str() << "\n";
    return static_cast<bool>(ofs);
}

void PrintCalibrationInfo(robosense::calibration::DeviceCalibration &calib)
{
    const auto print_vec3 = [](const cv::Vec3d& v) {
        std::cout << "[" << v[0] << ", " << v[1] << ", " << v[2] << "]";
    };
    const auto print_quat = [](const cv::Vec4d& q) {
        std::cout << "[w=" << q[0] << ", x=" << q[1] << ", y=" << q[2] << ", z=" << q[3] << "]";
    };
    const auto print_k = [](const cv::Matx33d& k) {
        std::cout << "[[" << k(0, 0) << ", " << k(0, 1) << ", " << k(0, 2) << "], "
                  << "[" << k(1, 0) << ", " << k(1, 1) << ", " << k(1, 2) << "], "
                  << "[" << k(2, 0) << ", " << k(2, 1) << ", " << k(2, 2) << "]]";
    };
    const auto print_dist = [](const std::vector<double>& d) {
        std::cout << "[";
        for (size_t i = 0; i < d.size(); ++i) {
            if (i != 0) {
                std::cout << ", ";
            }
            std::cout << d[i];
        }
        std::cout << "]";
    };
    const auto print_extrinsic = [&](const char* name, const robosense::calibration::Extrinsic& e) {
        std::cout << name << " extrinsic:\n";
        std::cout << "  translation = ";
        print_vec3(e.translation);
        std::cout << "\n  quaternion  = ";
        print_quat(e.quaternion);
        std::cout << "\n";
    };
    const auto print_camera = [&](const char* name, const robosense::calibration::CameraCalibration& c) {
        std::cout << name << ":\n";
        std::cout << "  base_link = " << c.base_link << "\n";
        std::cout << "  frame     = " << c.frame << "\n";
        print_extrinsic("  camera", c.extrinsic);
        std::cout << "  intrinsic.model      = " << c.intrinsic.model << "\n";
        std::cout << "  intrinsic.image_size = " << c.intrinsic.image_size.width
                  << "x" << c.intrinsic.image_size.height << "\n";
        std::cout << "  intrinsic.K          = ";
        print_k(c.intrinsic.int_matrix);
        std::cout << "\n";
        std::cout << "  intrinsic.dist_coeff = ";
        print_dist(c.intrinsic.dist_coeff);
        std::cout << "\n";
    };

    std::cout << std::fixed << std::setprecision(6);
    std::cout << "DeviceCalibration:\n";
    std::cout << "  device_id = " << calib.device_id << "\n";
    std::cout << "  cali_time = " << calib.cali_time << "\n";

    std::cout << "Lidar:\n";
    std::cout << "  base_link = " << calib.sensor.lidar.base_link << "\n";
    std::cout << "  frame     = " << calib.sensor.lidar.frame << "\n";
    print_extrinsic("  lidar", calib.sensor.lidar.extrinsic);

    print_camera("Left camera", calib.sensor.camera);
    print_camera("Right camera", calib.sensor.camera_r);

    std::cout << "IMU:\n";
    std::cout << "  base_link = " << calib.sensor.imu.base_link << "\n";
    std::cout << "  frame     = " << calib.sensor.imu.frame << "\n";
    print_extrinsic("  imu", calib.sensor.imu.extrinsic);
}

}  // namespace

int main(int argc, char** argv) {
    try {
        if (argc < 3) {
            std::cerr << "Usage: <exe> left.png right.png [calibration.yaml] [rectified_out.yaml]\n";
            return 1;
        }

        const std::string left_path = argv[1];
        const std::string right_path = argv[2];
        const std::string calib_path = (argc >= 4) ? argv[3] : "config/1111BEDC0028_calibration.yaml";
        const std::string rectified_out_path = (argc >= 5) ? argv[4] : "config/1111BEDC0028_rectified_calibration.yaml";
        const std::string calibration_mode = (argc >= 6) ? argv[5] : "single";

        cv::Mat left_cv = cv::imread(left_path, cv::IMREAD_COLOR);
        cv::Mat right_cv = cv::imread(right_path, cv::IMREAD_COLOR);
        if (left_cv.empty() || right_cv.empty()) {
            throw std::runtime_error("Failed to read input images");
        }

        if (!left_cv.isContinuous()) {
            left_cv = left_cv.clone();
        }
        if (!right_cv.isContinuous()) {
            right_cv = right_cv.clone();
        }

        robosense::calibration::DeviceCalibration calib;
        if (!LoadCalibrationFromFile(calib_path, &calib)) {
            throw std::runtime_error("Failed to load calibration file: " + calib_path);
        }

        const int width = calib.sensor.camera.intrinsic.image_size.width;
        const int height = calib.sensor.camera.intrinsic.image_size.height;
        if (left_cv.cols != width || left_cv.rows != height || right_cv.cols != width || right_cv.rows != height) {
            throw std::runtime_error("Input image size does not match calibration image_size");
        }

        robosense::StereoRectifier rectifier;
        if (!rectifier.Initialize(calib, 1)) {
            throw std::runtime_error("Initialize failed");
        }

        // Print rectified calibration to verify intrinsic/extrinsic updates.
        robosense::calibration::DeviceCalibration rect_calib = rectifier.GetRectifiedCalibrationInfo();
        if (!SaveCalibrationToFile(rectified_out_path, rect_calib)) {
            throw std::runtime_error("Failed to save rectified calibration file: " + rectified_out_path);
        }
        std::cout << "Saved rectified calibration to: " << rectified_out_path << "\n";
        PrintCalibrationInfo(rect_calib);
        if(calibration_mode == "stereo"){
            cv::Mat left_rectified;
            cv::Mat right_rectified;
            const auto remap_start = std::chrono::steady_clock::now();
            const bool ok = rectifier.Remap(left_cv, right_cv, &left_rectified, &right_rectified);
            const auto remap_end = std::chrono::steady_clock::now();
            const double remap_ms =
                std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(remap_end - remap_start).count();
            std::cout << "Remap time: " << remap_ms << " ms\n";
            if (!ok || left_rectified.empty() || right_rectified.empty()) {
                throw std::runtime_error("Remap failed");
            }

            cv::imwrite("left_rect.png", left_rectified);
            cv::imwrite("right_rect.png", right_rectified);


            cv::imshow("left_rect", left_rectified);
            cv::imshow("right_rect", right_rectified);
            char key = cv::waitKey(0);
        } else {
            cv::Mat left_rectified;
            const auto remap_start = std::chrono::steady_clock::now();
            const bool ok = rectifier.SingleImageRemap(0, left_cv, &left_rectified);
            const auto remap_end = std::chrono::steady_clock::now();
            const double remap_ms =
                std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(remap_end - remap_start).count();
            std::cout << "SingleImageRemap time: " << remap_ms << " ms\n";
            if (!ok || left_rectified.empty()) {
                throw std::runtime_error("SingleImageRemap failed");
            }

            cv::imwrite("left_rect.png", left_rectified);
            cv::imshow("left_rect", left_rectified);
            char key = cv::waitKey(0);
        }
        

  
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Stereo rectification failed: " << e.what() << "\n";
        return 1;
    }
}
