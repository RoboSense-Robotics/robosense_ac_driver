#include "hyper_vision/codec/colorcodec.h"
#include <fstream>
#include <iostream>

int main(int argc, char **argv) {

  // fisheye:
  {
    const int imageWidth = 1920;
    const int imageHeight = 1536;

    robosense::color::ColorCodec colorCodec;
    int ret = colorCodec.init(imageWidth, imageHeight);
    if (ret != 0) {
      std::cout << "init fisheye ColorCodec failed !" << std::endl;
      return -1;
    }

    // to_rgb
    {
#if defined(ENABLE_USE_CUDA)
      const std::string &nv12_dir_path =
          "/media/sti/ee28dfb8-da74-4739-88ab-b2b836e8b78c/super_sensor_sdk_v1/"
          "test/color_test/gpu/to_rgb/fisheye/nv12";

      const std::string &yuv420p_dir_path =
          "/media/sti/ee28dfb8-da74-4739-88ab-b2b836e8b78c/super_sensor_sdk_v1/"
          "test/color_test/gpu/to_rgb/fisheye/yuv420p";

      const std::string &yuyv422_dir_path =
          "/media/sti/ee28dfb8-da74-4739-88ab-b2b836e8b78c/super_sensor_sdk_v1/"
          "test/color_test/gpu/to_rgb/fisheye/yuyv422";
#else
      const std::string &nv12_dir_path =
          "/media/sti/ee28dfb8-da74-4739-88ab-b2b836e8b78c/super_sensor_sdk_v1/"
          "test/color_test/cpu/to_rgb/fisheye/nv12";

      const std::string &yuv420p_dir_path =
          "/media/sti/ee28dfb8-da74-4739-88ab-b2b836e8b78c/super_sensor_sdk_v1/"
          "test/color_test/cpu/to_rgb/fisheye/yuv420p";

      const std::string &yuyv422_dir_path =
          "/media/sti/ee28dfb8-da74-4739-88ab-b2b836e8b78c/super_sensor_sdk_v1/"
          "test/color_test/cpu/to_rgb/fisheye/yuyv422";
#endif // defined(ENABLE_USE_CUDA)
      const std::vector<std::string> nv12_names = {
          "1736135282216826112.jpeg.nv12", "1736135282517041920.jpeg.nv12",
          "1736135282316897024.jpeg.nv12", "1736135282617114880.jpeg.nv12",
          "1736135282416971008.jpeg.nv12",
      };

      const std::vector<std::string> yuv420p_names = {
          "1736135283417696000.jpeg.yuv420p",
          "1736135283717911040.jpeg.yuv420p",
          "1736135283517767936.jpeg.yuv420p",
          "1736135283817985024.jpeg.yuv420p",
          "1736135283617839104.jpeg.yuv420p",
      };

      const std::vector<std::string> yuyv422_names = {
          "1736135289422039040.jpeg.yuyv422",
          "1736135289722255872.jpeg.yuyv422",
          "1736135289522107904.jpeg.yuyv422",
          "1736135289822328064.jpeg.yuyv422",
          "1736135289622181888.jpeg.yuyv422",
      };

      {
        const int nv12_image_size = robosense::color::ColorCodec::NV12ImageSize(
            imageWidth, imageHeight);
        const int rgb_image_size =
            robosense::color::ColorCodec::RGBImageSize(imageWidth, imageHeight);
        std::cout << "start fisheye to_rgb: nv12 " << std::endl;
        for (size_t i = 0; i < nv12_names.size(); ++i) {
          const std::string &nv12_file_path =
              nv12_dir_path + "/" + nv12_names[i];

          std::vector<unsigned char> nv12_buffer(nv12_image_size, '\0');
          std::ifstream ifstr(nv12_file_path,
                              std::ios_base::in | std::ios_base::binary);
          if (!ifstr.is_open()) {
            std::cout << "open nv12 to read failed: " << nv12_file_path
                      << std::endl;
            return -1;
          }

          if (!ifstr.read(reinterpret_cast<char *>(nv12_buffer.data()),
                          nv12_image_size)) {
            std::cout << "read nv12 failed: " << nv12_file_path << std::endl;
            return -2;
          }

          std::vector<unsigned char> rgb_buffer(rgb_image_size, '\0');
          int rgb_buffer_len = rgb_image_size;

          int ret = colorCodec.NV12ToRGB(nv12_buffer.data(), nv12_image_size,
                                         rgb_buffer.data(), rgb_buffer_len);
          if (ret != 0) {
            std::cout << "nv12 to rgb failed: " << nv12_file_path
                      << ", ret = " << ret << std::endl;
            return -3;
          }

          {
            const std::string &jpeg_file_path = nv12_file_path + ".jpeg";
            cv::Mat rgbMat(imageHeight, imageWidth, CV_8UC3, rgb_buffer.data());

            cv::Mat bgrMat;
            cv::cvtColor(rgbMat, bgrMat, cv::COLOR_RGB2BGR);
            if (bgrMat.empty()) {
              std::cout << "nv12 to rgb: rgb->bgr failed !" << std::endl;
              return -4;
            }

            bool isSuccess = cv::imwrite(jpeg_file_path, bgrMat,
                                         {cv::IMWRITE_JPEG_QUALITY, 100});
            if (!isSuccess) {
              std::cout << "nv12 to rgb: write jpeg failed: " << jpeg_file_path
                        << std::endl;
              return -5;
            }
          }

          std::cout << "fisheye to_rgb: nv12: " << nv12_file_path
                    << " successed !" << std::endl;
        }

        std::cout << "end fisheye to_rgb: nv12" << std::endl;
      }

      {
        const int yuv420p_image_size =
            robosense::color::ColorCodec::YUV420PImageSize(imageWidth,
                                                           imageHeight);
        const int rgb_image_size =
            robosense::color::ColorCodec::RGBImageSize(imageWidth, imageHeight);
        std::cout << "start fisheye to_rgb: yuv420p " << std::endl;

        for (size_t i = 0; i < yuv420p_names.size(); ++i) {
          const std::string &yuv420p_file_path =
              yuv420p_dir_path + "/" + yuv420p_names[i];

          std::vector<unsigned char> yuv420p_buffer(yuv420p_image_size, '\0');
          std::ifstream ifstr(yuv420p_file_path,
                              std::ios_base::in | std::ios_base::binary);
          if (!ifstr.is_open()) {
            std::cout << "open yuv420p to read failed: " << yuv420p_file_path
                      << std::endl;
            return -1;
          }

          if (!ifstr.read(reinterpret_cast<char *>(yuv420p_buffer.data()),
                          yuv420p_image_size)) {
            std::cout << "read yuv420p failed: " << yuv420p_file_path
                      << std::endl;
            return -2;
          }

          std::vector<unsigned char> rgb_buffer(rgb_image_size, '\0');
          int rgb_buffer_len = rgb_image_size;

          int ret =
              colorCodec.YUV420PToRGB(yuv420p_buffer.data(), yuv420p_image_size,
                                      rgb_buffer.data(), rgb_buffer_len);
          if (ret != 0) {
            std::cout << "yuv420p to rgb failed: " << yuv420p_file_path
                      << ", ret = " << ret << std::endl;
            return -3;
          }

          {
            const std::string &jpeg_file_path = yuv420p_file_path + ".jpeg";
            cv::Mat rgbMat(imageHeight, imageWidth, CV_8UC3, rgb_buffer.data());

            cv::Mat bgrMat;
            cv::cvtColor(rgbMat, bgrMat, cv::COLOR_RGB2BGR);
            if (bgrMat.empty()) {
              std::cout << "yuv420p to rgb: rgb->bgr failed !" << std::endl;
              return -4;
            }

            bool isSuccess = cv::imwrite(jpeg_file_path, bgrMat,
                                         {cv::IMWRITE_JPEG_QUALITY, 100});
            if (!isSuccess) {
              std::cout << "yuv420p to rgb: write jpeg failed: "
                        << jpeg_file_path << std::endl;
              return -5;
            }
          }

          std::cout << "fisheye to_rgb: yuv420p: " << yuv420p_file_path
                    << " successed !" << std::endl;
        }

        std::cout << "end fisheye to_rgb: yuv420p" << std::endl;
      }

      {
        const int yuyv422_image_size =
            robosense::color::ColorCodec::YUYV422ImageSize(imageWidth,
                                                           imageHeight);
        const int rgb_image_size =
            robosense::color::ColorCodec::RGBImageSize(imageWidth, imageHeight);

        std::cout << "start fisheye to_rgb: yuyv422 " << std::endl;
        for (size_t i = 0; i < yuyv422_names.size(); ++i) {
          const std::string &yuyv422_file_path =
              yuyv422_dir_path + "/" + yuyv422_names[i];

          std::vector<unsigned char> yuyv422_buffer(yuyv422_image_size, '\0');
          std::ifstream ifstr(yuyv422_file_path,
                              std::ios_base::in | std::ios_base::binary);
          if (!ifstr.is_open()) {
            std::cout << "open yuyv422 to read failed: " << yuyv422_file_path
                      << std::endl;
            return -1;
          }

          if (!ifstr.read(reinterpret_cast<char *>(yuyv422_buffer.data()),
                          yuyv422_image_size)) {
            std::cout << "read yuyv422 failed: " << yuyv422_file_path
                      << std::endl;
            return -2;
          }

          std::vector<unsigned char> rgb_buffer(rgb_image_size, '\0');
          int rgb_buffer_len = rgb_image_size;

          int ret =
              colorCodec.YUYV422ToRGB(yuyv422_buffer.data(), yuyv422_image_size,
                                      rgb_buffer.data(), rgb_buffer_len);
          if (ret != 0) {
            std::cout << "yuyv422 to rgb failed: " << yuyv422_file_path
                      << ", ret = " << ret << std::endl;
            return -3;
          }

          {
            const std::string &jpeg_file_path = yuyv422_file_path + ".jpeg";
            cv::Mat rgbMat(imageHeight, imageWidth, CV_8UC3, rgb_buffer.data());

            cv::Mat bgrMat;
            cv::cvtColor(rgbMat, bgrMat, cv::COLOR_RGB2BGR);
            if (bgrMat.empty()) {
              std::cout << "yuyv422 to rgb: rgb->bgr failed !" << std::endl;
              return -4;
            }

            bool isSuccess = cv::imwrite(jpeg_file_path, bgrMat,
                                         {cv::IMWRITE_JPEG_QUALITY, 100});
            if (!isSuccess) {
              std::cout << "yuyv422 to rgb: write jpeg failed: "
                        << jpeg_file_path << std::endl;
              return -5;
            }
          }

          std::cout << "fisheye to_rgb: yuyv422: " << yuyv422_file_path
                    << " successed !" << std::endl;
        }
        std::cout << "end fisheye to_rgb: yuyv422" << std::endl;
      }
    }

    // from_rgb
    {
#if defined(ENABLE_USE_CUDA)
      const std::string &rgb_dir_path =
          "/media/sti/ee28dfb8-da74-4739-88ab-b2b836e8b78c/super_sensor_sdk_v1/"
          "test/color_test/gpu/from_rgb/fisheye";
#else
      const std::string &rgb_dir_path =
          "/media/sti/ee28dfb8-da74-4739-88ab-b2b836e8b78c/super_sensor_sdk_v1/"
          "test/color_test/cpu/from_rgb/fisheye";
#endif // defined(ENABLE_USE_CUDA)
      const std::vector<std::string> rgb_names = {
          "1736135282116753920.jpeg.rgb", "1736135282416971008.jpeg.rgb",
          "1736135282216826112.jpeg.rgb", "1736135282517041920.jpeg.rgb",
          "1736135282316897024.jpeg.rgb",
      };

      const int rgb_image_size =
          robosense::color::ColorCodec::RGBImageSize(imageWidth, imageHeight);
      const int nv12_image_size =
          robosense::color::ColorCodec::NV12ImageSize(imageWidth, imageHeight);
      const int yuv420p_image_size =
          robosense::color::ColorCodec::YUV420PImageSize(imageWidth,
                                                         imageHeight);
      const int yuyv422_image_size =
          robosense::color::ColorCodec::YUYV422ImageSize(imageWidth,
                                                         imageHeight);

      std::cout << "start fisheye from_rgb " << std::endl;
      for (size_t i = 0; i < rgb_names.size(); ++i) {
        const std::string &rgb_file_path = rgb_dir_path + "/" + rgb_names[i];

        std::ifstream ifstr(rgb_file_path,
                            std::ios_base::in | std::ios_base::binary);
        if (!ifstr.is_open()) {
          std::cout << "open rgb to read failed: " << rgb_file_path
                    << std::endl;
          return -1;
        }

        std::vector<unsigned char> rgb_to_nv12_buffer(rgb_image_size, '\0');
        std::vector<unsigned char> rgb_to_yuv420p_buffer(rgb_image_size, '\0');
        std::vector<unsigned char> rgb_to_yuyv422_buffer(rgb_image_size, '\0');

        if (!ifstr.read(reinterpret_cast<char *>(rgb_to_nv12_buffer.data()),
                        rgb_image_size)) {
          std::cout << "read rgb file: " << rgb_file_path << " failed !"
                    << std::endl;
          return -2;
        }
        memcpy(rgb_to_yuv420p_buffer.data(), rgb_to_nv12_buffer.data(),
               rgb_image_size);
        memcpy(rgb_to_yuyv422_buffer.data(), rgb_to_nv12_buffer.data(),
               rgb_image_size);

        std::vector<unsigned char> nv12_buffer(nv12_image_size, '\0');
        int nv12_buffer_len = nv12_image_size;
        int ret =
            colorCodec.RGBToNV12(rgb_to_nv12_buffer.data(), rgb_image_size,
                                 nv12_buffer.data(), nv12_buffer_len);
        if (ret != 0) {
          std::cout << "rgb to nv12 failed: " << rgb_file_path << std::endl;
          return -3;
        }

        {
          const std::string &jpeg_file_path = rgb_file_path + ".nv12.jpeg";
          cv::Mat nv12Mat(imageHeight * 3 / 2, imageWidth, CV_8UC1,
                          nv12_buffer.data());

          cv::Mat bgrMat;
          cv::cvtColor(nv12Mat, bgrMat, cv::COLOR_YUV2BGR_NV12);
          if (bgrMat.empty()) {
            std::cout << "rgb to nv12: rgb->bgr failed !" << std::endl;
            return -4;
          }

          bool isSuccess = cv::imwrite(jpeg_file_path, bgrMat,
                                       {cv::IMWRITE_JPEG_QUALITY, 100});
          if (!isSuccess) {
            std::cout << "rgb to nv12: write jpeg failed: " << jpeg_file_path
                      << std::endl;
            return -5;
          }
        }

        std::vector<unsigned char> yuv420p_buffer(yuv420p_image_size, '\0');
        int yuv420p_buffer_len = yuv420p_image_size;
        ret = colorCodec.RGBToYUV420P(rgb_to_yuv420p_buffer.data(),
                                      rgb_image_size, yuv420p_buffer.data(),
                                      yuv420p_buffer_len);
        if (ret != 0) {
          std::cout << "rgb to yuv420p failed: " << rgb_file_path << std::endl;
          return -3;
        }

        {
          const std::string &jpeg_file_path = rgb_file_path + ".yuv420p.jpeg";
          cv::Mat yuv420pMat(imageHeight * 3 / 2, imageWidth, CV_8UC1,
                             yuv420p_buffer.data());

          cv::Mat bgrMat;
          cv::cvtColor(yuv420pMat, bgrMat, cv::COLOR_YUV2BGR_I420);
          if (bgrMat.empty()) {
            std::cout << "rgb to yuv420p: rgb->bgr failed !" << std::endl;
            return -4;
          }

          bool isSuccess = cv::imwrite(jpeg_file_path, bgrMat,
                                       {cv::IMWRITE_JPEG_QUALITY, 100});
          if (!isSuccess) {
            std::cout << "rgb to yuv420p: write jpeg failed: " << jpeg_file_path
                      << std::endl;
            return -5;
          }
        }

        std::vector<unsigned char> yuyv422_buffer(yuyv422_image_size, '\0');
        int yuyv422_buffer_len = yuyv422_image_size;
        ret = colorCodec.RGBToYUYV422(rgb_to_yuyv422_buffer.data(),
                                      rgb_image_size, yuyv422_buffer.data(),
                                      yuyv422_buffer_len);
        if (ret != 0) {
          std::cout << "rgb to yuyv422 failed: " << rgb_file_path
                    << ", ret = " << ret << std::endl;
          return -3;
        }

        {
          const std::string &jpeg_file_path = rgb_file_path + ".yuyv422.jpeg";
          cv::Mat yuyv422Mat(imageHeight, imageWidth, CV_8UC2,
                             yuyv422_buffer.data());

          cv::Mat bgrMat;
          cv::cvtColor(yuyv422Mat, bgrMat, cv::COLOR_YUV2BGR_YUYV);
          if (bgrMat.empty()) {
            std::cout << "rgb to yuyv422: rgb->bgr failed !" << std::endl;
            return -4;
          }

          bool isSuccess = cv::imwrite(jpeg_file_path, bgrMat,
                                       {cv::IMWRITE_JPEG_QUALITY, 100});
          if (!isSuccess) {
            std::cout << "rgb to yuyv422: write jpeg failed: " << jpeg_file_path
                      << std::endl;
            return -5;
          }
        }
        std::cout << "from_rgb: " << rgb_file_path << " successed !"
                  << std::endl;
      }
      std::cout << "end fisheye from_rgb " << std::endl;
    }

    // to yuyv422
    {
#if defined(ENABLE_USE_CUDA)
      const std::string &nv12_dir_path =
          "/media/sti/ee28dfb8-da74-4739-88ab-b2b836e8b78c/super_sensor_sdk_v1/"
          "test/color_test/gpu/to_yuyv422/fisheye/nv12";
      const std::string &yuv420p_dir_path =
          "/media/sti/ee28dfb8-da74-4739-88ab-b2b836e8b78c/super_sensor_sdk_v1/"
          "test/color_test/gpu/to_yuyv422/fisheye/yuv420p";
#else
      const std::string &nv12_dir_path =
          "/media/sti/ee28dfb8-da74-4739-88ab-b2b836e8b78c/super_sensor_sdk_v1/"
          "test/color_test/cpu/to_yuyv422/fisheye/nv12";
      const std::string &yuv420p_dir_path =
          "/media/sti/ee28dfb8-da74-4739-88ab-b2b836e8b78c/super_sensor_sdk_v1/"
          "test/color_test/cpu/to_yuyv422/fisheye/yuv420p";
#endif // defined(ENABLE_USE_CUDA)
      const std::vector<std::string> nv12_names = {
          "1736135304633039104.jpeg.nv12", "1736135304933257984.jpeg.nv12",
          "1736135304733113088.jpeg.nv12", "1736135305033330944.jpeg.nv12",
          "1736135304833185024.jpeg.nv12",
      };

      const std::vector<std::string> yuv420p_names = {
          "1736135284818706944.jpeg.yuv420p",
          "1736135285118944000.jpeg.yuv420p",
          "1736135284918787072.jpeg.yuv420p",
          "1736135285218995968.jpeg.yuv420p",
          "1736135285018853888.jpeg.yuv420p",
      };

      // nv12 -> yuyv422
      {
        const int nv12_image_size = robosense::color::ColorCodec::NV12ImageSize(
            imageWidth, imageHeight);
        const int yuyv422_image_size =
            robosense::color::ColorCodec::YUYV422ImageSize(imageWidth,
                                                           imageHeight);
        std::cout << "start to_yuyv422: nv12 " << std::endl;
        for (size_t i = 0; i < nv12_names.size(); ++i) {
          const std::string &nv12_file_path =
              nv12_dir_path + "/" + nv12_names[i];

          std::vector<unsigned char> nv12_buffer(nv12_image_size, '\0');
          std::ifstream ifstr(nv12_file_path,
                              std::ios_base::in | std::ios_base::binary);
          if (!ifstr.is_open()) {
            std::cout << "open nv12 to read failed: " << nv12_file_path
                      << std::endl;
            return -1;
          }

          if (!ifstr.read(reinterpret_cast<char *>(nv12_buffer.data()),
                          nv12_image_size)) {
            std::cout << "read nv12 failed: " << nv12_file_path << std::endl;
            return -2;
          }

          std::vector<unsigned char> yuyv422_buffer(yuyv422_image_size, '\0');
          int yuyv422_buffer_len = yuyv422_image_size;
          int ret = colorCodec.NV12ToRgbToYUV422(
              nv12_buffer.data(), nv12_image_size, yuyv422_buffer.data(),
              yuyv422_buffer_len);
          if (ret != 0) {
            std::cout << "nv12->yuyv422 failed: " << nv12_file_path
                      << ": ret = " << ret << std::endl;
            return -3;
          }

          {
            const std::string &jpeg_file_path =
                nv12_file_path + ".yuyv422.jpeg";
            cv::Mat yuyv422Mat(imageHeight, imageWidth, CV_8UC2,
                               yuyv422_buffer.data());

            cv::Mat bgrMat;
            cv::cvtColor(yuyv422Mat, bgrMat, cv::COLOR_YUV2BGR_YUYV);
            if (bgrMat.empty()) {
              std::cout << "nv12->yuyv422: rgb->bgr failed !" << std::endl;
              return -4;
            }

            bool isSuccess = cv::imwrite(jpeg_file_path, bgrMat,
                                         {cv::IMWRITE_JPEG_QUALITY, 100});
            if (!isSuccess) {
              std::cout << "nv12->yuyv422: write jpeg failed: "
                        << jpeg_file_path << std::endl;
              return -5;
            }
          }

          std::cout << "nv12->yuyv422: " << nv12_file_path << " successed !"
                    << std::endl;
        }
        std::cout << "end to_yuyv422: nv12 " << std::endl;
      }

      // yuv420p -> yuyu422
      {
        const int yuv420p_image_size =
            robosense::color::ColorCodec::YUV420PImageSize(imageWidth,
                                                           imageHeight);
        const int yuyv422_image_size =
            robosense::color::ColorCodec::YUYV422ImageSize(imageWidth,
                                                           imageHeight);
        std::cout << "start to_yuyv422: yuv420p" << std::endl;
        for (size_t i = 0; i < yuv420p_names.size(); ++i) {
          const std::string &yuv420p_file_path =
              yuv420p_dir_path + "/" + yuv420p_names[i];

          std::vector<unsigned char> yuv420p_buffer(yuv420p_image_size, '\0');
          std::ifstream ifstr(yuv420p_file_path,
                              std::ios_base::in | std::ios_base::binary);
          if (!ifstr.is_open()) {
            std::cout << "open yuv420p to read failed: " << yuv420p_file_path
                      << std::endl;
            return -1;
          }

          if (!ifstr.read(reinterpret_cast<char *>(yuv420p_buffer.data()),
                          yuv420p_image_size)) {
            std::cout << "read yuv420p failed: " << yuv420p_file_path
                      << std::endl;
            return -2;
          }

          std::vector<unsigned char> yuyv422_buffer(yuyv422_image_size, '\0');
          int yuyv422_buffer_len = yuyv422_image_size;
          int ret = colorCodec.YUV420PToRgbToYUYV422(
              yuv420p_buffer.data(), yuv420p_image_size, yuyv422_buffer.data(),
              yuyv422_buffer_len);
          if (ret != 0) {
            std::cout << "yuv420p->yuyv422 failed: " << yuv420p_file_path
                      << ", ret = " << ret << std::endl;
            return -3;
          }

          {
            const std::string &jpeg_file_path =
                yuv420p_file_path + ".yuyv422.jpeg";
            cv::Mat yuyv422Mat(imageHeight, imageWidth, CV_8UC2,
                               yuyv422_buffer.data());

            cv::Mat bgrMat;
            cv::cvtColor(yuyv422Mat, bgrMat, cv::COLOR_YUV2BGR_YUYV);
            if (bgrMat.empty()) {
              std::cout << "yuv420p->yuyv422: rgb->bgr failed !" << std::endl;
              return -4;
            }

            bool isSuccess = cv::imwrite(jpeg_file_path, bgrMat,
                                         {cv::IMWRITE_JPEG_QUALITY, 100});
            if (!isSuccess) {
              std::cout << "yuv420p->yuyv422: write jpeg failed: "
                        << jpeg_file_path << std::endl;
              return -5;
            }
          }

          std::cout << "yuv420p->yuyv422: " << yuv420p_file_path
                    << " successed !" << std::endl;
        }

        std::cout << "end to_yuyv422: yuv420p" << std::endl;
      }
    }

    // to yuv420x
    {
#if defined(ENABLE_USE_CUDA)
      const std::string &yuyv422_dir_path =
          "/media/sti/ee28dfb8-da74-4739-88ab-b2b836e8b78c/super_sensor_sdk_v1/"
          "test/color_test/gpu/to_yuv420x/fisheye/yuyv422";
#else
      const std::string &yuyv422_dir_path =
          "/media/sti/ee28dfb8-da74-4739-88ab-b2b836e8b78c/super_sensor_sdk_v1/"
          "test/color_test/cpu/to_yuv420x/fisheye/yuyv422";
#endif // defined(ENABLE_USE_CUDA)

      const std::vector<std::string> yuyv422_names = {
          "1736135301730940928.jpeg.yuyv422",
          "1736135302031161088.jpeg.yuyv422",
          "1736135301831015936.jpeg.yuyv422",
          "1736135302131229952.jpeg.yuyv422",
          "1736135301931087104.jpeg.yuyv422",
      };

      const int nv12_image_size =
          robosense::color::ColorCodec::NV12ImageSize(imageWidth, imageHeight);

      const int yuv420p_image_size =
          robosense::color::ColorCodec::YUV420PImageSize(imageWidth,
                                                         imageHeight);
      const int yuyv422_image_size =
          robosense::color::ColorCodec::YUYV422ImageSize(imageWidth,
                                                         imageHeight);

      std::cout << "start to_yuyv420x " << std::endl;
      for (size_t i = 0; i < yuyv422_names.size(); ++i) {
        const std::string &yuyv422_file_path =
            yuyv422_dir_path + "/" + yuyv422_names[i];

        std::vector<unsigned char> yuyv422_to_nv12_buffer(yuyv422_image_size,
                                                          '\0');
        std::vector<unsigned char> yuyv422_to_yuv420p_buffer(yuyv422_image_size,
                                                             '\0');

        std::ifstream ifstr(yuyv422_file_path,
                            std::ios_base::in | std::ios_base::binary);
        if (!ifstr.is_open()) {
          std::cout << "open yuyv422 to read failed: " << yuyv422_file_path
                    << std::endl;
          return -1;
        }

        if (!ifstr.read(reinterpret_cast<char *>(yuyv422_to_nv12_buffer.data()),
                        yuyv422_image_size)) {
          std::cout << "read yuyv422 failed: " << yuyv422_file_path
                    << std::endl;
          return -2;
        }

        memcpy(yuyv422_to_yuv420p_buffer.data(), yuyv422_to_nv12_buffer.data(),
               yuyv422_image_size);

        std::vector<unsigned char> nv12_buffer(nv12_image_size, '\0');
        std::vector<unsigned char> yuv420p_buffer(yuv420p_image_size, '\0');

        int nv12_buffer_len = nv12_image_size;
        int ret = colorCodec.YUYV422ToRgbToNV12(
            yuyv422_to_nv12_buffer.data(), yuyv422_image_size,
            nv12_buffer.data(), nv12_buffer_len);
        if (ret != 0) {
          std::cout << "yuyv422->nv12 failed: " << yuyv422_file_path
                    << std::endl;
          return -3;
        }

        {
          const std::string &jpeg_file_path = yuyv422_file_path + ".nv12.jpeg";
          cv::Mat nv12Mat(imageHeight * 3 / 2, imageWidth, CV_8UC1,
                          nv12_buffer.data());

          cv::Mat bgrMat;
          cv::cvtColor(nv12Mat, bgrMat, cv::COLOR_YUV2BGR_NV12);
          if (bgrMat.empty()) {
            std::cout << "yuyv422->nv12: rgb->bgr failed !" << std::endl;
            return -4;
          }

          bool isSuccess = cv::imwrite(jpeg_file_path, bgrMat,
                                       {cv::IMWRITE_JPEG_QUALITY, 100});
          if (!isSuccess) {
            std::cout << "yuyv422->nv12: write jpeg failed: " << jpeg_file_path
                      << std::endl;
            return -5;
          }
        }

        int yuv420p_buffer_len = yuv420p_image_size;
        ret = colorCodec.YUYV422ToRgbToYUV420P(
            yuyv422_to_yuv420p_buffer.data(), yuyv422_image_size,
            yuv420p_buffer.data(), yuv420p_buffer_len);
        if (ret != 0) {
          std::cout << "yuyv422->yuv420p failed: " << yuyv422_file_path
                    << std::endl;
          return -3;
        }

        {
          const std::string &jpeg_file_path =
              yuyv422_file_path + ".yuv420p.jpeg";
          cv::Mat yuv420pMat(imageHeight * 3 / 2, imageWidth, CV_8UC1,
                             yuv420p_buffer.data());

          cv::Mat bgrMat;
          cv::cvtColor(yuv420pMat, bgrMat, cv::COLOR_YUV2BGR_I420);
          if (bgrMat.empty()) {
            std::cout << "yuyv422->yuv420p: rgb->bgr failed !" << std::endl;
            return -4;
          }

          bool isSuccess = cv::imwrite(jpeg_file_path, bgrMat,
                                       {cv::IMWRITE_JPEG_QUALITY, 100});
          if (!isSuccess) {
            std::cout << "yuyv422->yuv420p: write jpeg failed: "
                      << jpeg_file_path << std::endl;
            return -5;
          }
        }

        std::cout << "yuyv422->yuv420x: " << yuyv422_file_path << " successed !"
                  << std::endl;
      }
      std::cout << "end to_yuyv420x" << std::endl;
    }
  }

  // pinhole:
  {
    const int imageWidth = 1920;
    const int imageHeight = 1080;

    robosense::color::ColorCodec colorCodec;
    int ret = colorCodec.init(imageWidth, imageHeight);
    if (ret != 0) {
      std::cout << "init pinhole ColorCodec failed !" << std::endl;
      return -1;
    }

    // to_rgb
    {
#if defined(ENABLE_USE_CUDA)
      const std::string &nv12_dir_path =
          "/media/sti/ee28dfb8-da74-4739-88ab-b2b836e8b78c/super_sensor_sdk_v1/"
          "test/color_test/gpu/to_rgb/pinhole/nv12";

      const std::string &yuv420p_dir_path =
          "/media/sti/ee28dfb8-da74-4739-88ab-b2b836e8b78c/super_sensor_sdk_v1/"
          "test/color_test/gpu/to_rgb/pinhole/yuv420p";

      const std::string &yuyv422_dir_path =
          "/media/sti/ee28dfb8-da74-4739-88ab-b2b836e8b78c/super_sensor_sdk_v1/"
          "test/color_test/gpu/to_rgb/pinhole/yuyv422";
#else
      const std::string &nv12_dir_path =
          "/media/sti/ee28dfb8-da74-4739-88ab-b2b836e8b78c/super_sensor_sdk_v1/"
          "test/color_test/cpu/to_rgb/pinhole/nv12";

      const std::string &yuv420p_dir_path =
          "/media/sti/ee28dfb8-da74-4739-88ab-b2b836e8b78c/super_sensor_sdk_v1/"
          "test/color_test/cpu/to_rgb/pinhole/yuv420p";

      const std::string &yuyv422_dir_path =
          "/media/sti/ee28dfb8-da74-4739-88ab-b2b836e8b78c/super_sensor_sdk_v1/"
          "test/color_test/cpu/to_rgb/pinhole/yuyv422";
#endif // defined(ENABLE_USE_CUDA)

      const std::vector<std::string> nv12_names = {
          "1736135307535139072.jpeg.nv12", "1736135307835355904.jpeg.nv12",
          "1736135307635212032.jpeg.nv12", "1736135307935430912.jpeg.nv12",
          "1736135307735283968.jpeg.nv12",
      };

      const std::vector<std::string> yuv420p_names = {
          "1736135307735283968.jpeg.yuv420p",
          "1736135308035503104.jpeg.yuv420p",
          "1736135307835355904.jpeg.yuv420p",
          "1736135308135576064.jpeg.yuv420p",
          "1736135307935430912.jpeg.yuv420p",
      };

      const std::vector<std::string> yuyv422_names = {
          "1736135305533692928.jpeg.yuyv422",
          "1736135305833907968.jpeg.yuyv422",
          "1736135305633765120.jpeg.yuyv422",
          "1736135305933981952.jpeg.yuyv422",
          "1736135305733839104.jpeg.yuyv422",
      };

      {
        const int nv12_image_size = robosense::color::ColorCodec::NV12ImageSize(
            imageWidth, imageHeight);
        const int rgb_image_size =
            robosense::color::ColorCodec::RGBImageSize(imageWidth, imageHeight);
        std::cout << "start pinhole to_rgb: nv12 " << std::endl;
        for (size_t i = 0; i < nv12_names.size(); ++i) {
          const std::string &nv12_file_path =
              nv12_dir_path + "/" + nv12_names[i];

          std::vector<unsigned char> nv12_buffer(nv12_image_size, '\0');
          std::ifstream ifstr(nv12_file_path,
                              std::ios_base::in | std::ios_base::binary);
          if (!ifstr.is_open()) {
            std::cout << "open nv12 to read failed: " << nv12_file_path
                      << std::endl;
            return -1;
          }

          if (!ifstr.read(reinterpret_cast<char *>(nv12_buffer.data()),
                          nv12_image_size)) {
            std::cout << "read nv12 failed: " << nv12_file_path << std::endl;
            return -2;
          }

          std::vector<unsigned char> rgb_buffer(rgb_image_size, '\0');
          int rgb_buffer_len = rgb_image_size;

          int ret = colorCodec.NV12ToRGB(nv12_buffer.data(), nv12_image_size,
                                         rgb_buffer.data(), rgb_buffer_len);
          if (ret != 0) {
            std::cout << "nv12 to rgb failed: " << nv12_file_path
                      << ", ret = " << ret << std::endl;
            return -3;
          }

          {
            const std::string &jpeg_file_path = nv12_file_path + ".jpeg";
            cv::Mat rgbMat(imageHeight, imageWidth, CV_8UC3, rgb_buffer.data());

            cv::Mat bgrMat;
            cv::cvtColor(rgbMat, bgrMat, cv::COLOR_RGB2BGR);
            if (bgrMat.empty()) {
              std::cout << "nv12 to rgb: rgb->bgr failed !" << std::endl;
              return -4;
            }

            bool isSuccess = cv::imwrite(jpeg_file_path, bgrMat,
                                         {cv::IMWRITE_JPEG_QUALITY, 100});
            if (!isSuccess) {
              std::cout << "nv12 to rgb: write jpeg failed: " << jpeg_file_path
                        << std::endl;
              return -5;
            }
          }

          std::cout << "pinhole to_rgb: nv12: " << nv12_file_path
                    << " successed !" << std::endl;
        }

        std::cout << "end pinhole to_rgb: nv12" << std::endl;
      }

      {
        const int yuv420p_image_size =
            robosense::color::ColorCodec::YUV420PImageSize(imageWidth,
                                                           imageHeight);
        const int rgb_image_size =
            robosense::color::ColorCodec::RGBImageSize(imageWidth, imageHeight);
        std::cout << "start pinhole to_rgb: yuv420p " << std::endl;

        for (size_t i = 0; i < yuv420p_names.size(); ++i) {
          const std::string &yuv420p_file_path =
              yuv420p_dir_path + "/" + yuv420p_names[i];

          std::vector<unsigned char> yuv420p_buffer(yuv420p_image_size, '\0');
          std::ifstream ifstr(yuv420p_file_path,
                              std::ios_base::in | std::ios_base::binary);
          if (!ifstr.is_open()) {
            std::cout << "open yuv420p to read failed: " << yuv420p_file_path
                      << std::endl;
            return -1;
          }

          if (!ifstr.read(reinterpret_cast<char *>(yuv420p_buffer.data()),
                          yuv420p_image_size)) {
            std::cout << "read yuv420p failed: " << yuv420p_file_path
                      << std::endl;
            return -2;
          }

          std::vector<unsigned char> rgb_buffer(rgb_image_size, '\0');
          int rgb_buffer_len = rgb_image_size;

          int ret =
              colorCodec.YUV420PToRGB(yuv420p_buffer.data(), yuv420p_image_size,
                                      rgb_buffer.data(), rgb_buffer_len);
          if (ret != 0) {
            std::cout << "yuv420p to rgb failed: " << yuv420p_file_path
                      << ", ret = " << ret << std::endl;
            return -3;
          }

          {
            const std::string &jpeg_file_path = yuv420p_file_path + ".jpeg";
            cv::Mat rgbMat(imageHeight, imageWidth, CV_8UC3, rgb_buffer.data());

            cv::Mat bgrMat;
            cv::cvtColor(rgbMat, bgrMat, cv::COLOR_RGB2BGR);
            if (bgrMat.empty()) {
              std::cout << "yuv420p to rgb: rgb->bgr failed !" << std::endl;
              return -4;
            }

            bool isSuccess = cv::imwrite(jpeg_file_path, bgrMat,
                                         {cv::IMWRITE_JPEG_QUALITY, 100});
            if (!isSuccess) {
              std::cout << "yuv420p to rgb: write jpeg failed: "
                        << jpeg_file_path << std::endl;
              return -5;
            }
          }

          std::cout << "pinhole to_rgb: yuv420p: " << yuv420p_file_path
                    << " successed !" << std::endl;
        }

        std::cout << "end pinhole to_rgb: yuv420p" << std::endl;
      }

      {
        const int yuyv422_image_size =
            robosense::color::ColorCodec::YUYV422ImageSize(imageWidth,
                                                           imageHeight);
        const int rgb_image_size =
            robosense::color::ColorCodec::RGBImageSize(imageWidth, imageHeight);

        std::cout << "start pinhole to_rgb: yuyv422 " << std::endl;
        for (size_t i = 0; i < yuyv422_names.size(); ++i) {
          const std::string &yuyv422_file_path =
              yuyv422_dir_path + "/" + yuyv422_names[i];

          std::vector<unsigned char> yuyv422_buffer(yuyv422_image_size, '\0');
          std::ifstream ifstr(yuyv422_file_path,
                              std::ios_base::in | std::ios_base::binary);
          if (!ifstr.is_open()) {
            std::cout << "open yuyv422 to read failed: " << yuyv422_file_path
                      << std::endl;
            return -1;
          }

          if (!ifstr.read(reinterpret_cast<char *>(yuyv422_buffer.data()),
                          yuyv422_image_size)) {
            std::cout << "read yuyv422 failed: " << yuyv422_file_path
                      << std::endl;
            return -2;
          }

          std::vector<unsigned char> rgb_buffer(rgb_image_size, '\0');
          int rgb_buffer_len = rgb_image_size;

          int ret =
              colorCodec.YUYV422ToRGB(yuyv422_buffer.data(), yuyv422_image_size,
                                      rgb_buffer.data(), rgb_buffer_len);
          if (ret != 0) {
            std::cout << "yuyv422 to rgb failed: " << yuyv422_file_path
                      << ", ret = " << ret << std::endl;
            return -3;
          }

          {
            const std::string &jpeg_file_path = yuyv422_file_path + ".jpeg";
            cv::Mat rgbMat(imageHeight, imageWidth, CV_8UC3, rgb_buffer.data());

            cv::Mat bgrMat;
            cv::cvtColor(rgbMat, bgrMat, cv::COLOR_RGB2BGR);
            if (bgrMat.empty()) {
              std::cout << "yuyv422 to rgb: rgb->bgr failed !" << std::endl;
              return -4;
            }

            bool isSuccess = cv::imwrite(jpeg_file_path, bgrMat,
                                         {cv::IMWRITE_JPEG_QUALITY, 100});
            if (!isSuccess) {
              std::cout << "yuyv422 to rgb: write jpeg failed: "
                        << jpeg_file_path << std::endl;
              return -5;
            }
          }

          std::cout << "pinhole to_rgb: yuyv422: " << yuyv422_file_path
                    << " successed !" << std::endl;
        }
        std::cout << "end pinhole to_rgb: yuyv422" << std::endl;
      }
    }

    // from_rgb
    {
#if defined(ENABLE_USE_CUDA)
      const std::string &rgb_dir_path =
          "/media/sti/ee28dfb8-da74-4739-88ab-b2b836e8b78c/super_sensor_sdk_v1/"
          "test/color_test/gpu/from_rgb/pinhole";
#else
      const std::string &rgb_dir_path =
          "/media/sti/ee28dfb8-da74-4739-88ab-b2b836e8b78c/super_sensor_sdk_v1/"
          "test/color_test/cpu/from_rgb/pinhole";
#endif // defined(ENABLE_USE_CUDA)

      const std::vector<std::string> rgb_names = {
          "1736135283517767936.jpeg.rgb", "1736135283817985024.jpeg.rgb",
          "1736135283617839104.jpeg.rgb", "1736135283918057984.jpeg.rgb",
          "1736135283717911040.jpeg.rgb",
      };

      const int rgb_image_size =
          robosense::color::ColorCodec::RGBImageSize(imageWidth, imageHeight);
      const int nv12_image_size =
          robosense::color::ColorCodec::NV12ImageSize(imageWidth, imageHeight);
      const int yuv420p_image_size =
          robosense::color::ColorCodec::YUV420PImageSize(imageWidth,
                                                         imageHeight);
      const int yuyv422_image_size =
          robosense::color::ColorCodec::YUYV422ImageSize(imageWidth,
                                                         imageHeight);

      std::cout << "start pinhole from_rgb " << std::endl;
      for (size_t i = 0; i < rgb_names.size(); ++i) {
        const std::string &rgb_file_path = rgb_dir_path + "/" + rgb_names[i];

        std::ifstream ifstr(rgb_file_path,
                            std::ios_base::in | std::ios_base::binary);
        if (!ifstr.is_open()) {
          std::cout << "open rgb to read failed: " << rgb_file_path
                    << std::endl;
          return -1;
        }

        std::vector<unsigned char> rgb_to_nv12_buffer(rgb_image_size, '\0');
        std::vector<unsigned char> rgb_to_yuv420p_buffer(rgb_image_size, '\0');
        std::vector<unsigned char> rgb_to_yuyv422_buffer(rgb_image_size, '\0');

        if (!ifstr.read(reinterpret_cast<char *>(rgb_to_nv12_buffer.data()),
                        rgb_image_size)) {
          std::cout << "read rgb file: " << rgb_file_path << " failed !"
                    << std::endl;
          return -2;
        }
        memcpy(rgb_to_yuv420p_buffer.data(), rgb_to_nv12_buffer.data(),
               rgb_image_size);
        memcpy(rgb_to_yuyv422_buffer.data(), rgb_to_nv12_buffer.data(),
               rgb_image_size);

        std::vector<unsigned char> nv12_buffer(nv12_image_size, '\0');
        int nv12_buffer_len = nv12_image_size;
        int ret =
            colorCodec.RGBToNV12(rgb_to_nv12_buffer.data(), rgb_image_size,
                                 nv12_buffer.data(), nv12_buffer_len);
        if (ret != 0) {
          std::cout << "rgb to nv12 failed: " << rgb_file_path << std::endl;
          return -3;
        }

        {
          const std::string &jpeg_file_path = rgb_file_path + ".nv12.jpeg";
          cv::Mat nv12Mat(imageHeight * 3 / 2, imageWidth, CV_8UC1,
                          nv12_buffer.data());

          cv::Mat bgrMat;
          cv::cvtColor(nv12Mat, bgrMat, cv::COLOR_YUV2BGR_NV12);
          if (bgrMat.empty()) {
            std::cout << "rgb to nv12: rgb->bgr failed !" << std::endl;
            return -4;
          }

          bool isSuccess = cv::imwrite(jpeg_file_path, bgrMat,
                                       {cv::IMWRITE_JPEG_QUALITY, 100});
          if (!isSuccess) {
            std::cout << "rgb to nv12: write jpeg failed: " << jpeg_file_path
                      << std::endl;
            return -5;
          }
        }

        std::vector<unsigned char> yuv420p_buffer(yuv420p_image_size, '\0');
        int yuv420p_buffer_len = yuv420p_image_size;
        ret = colorCodec.RGBToYUV420P(rgb_to_yuv420p_buffer.data(),
                                      rgb_image_size, yuv420p_buffer.data(),
                                      yuv420p_buffer_len);
        if (ret != 0) {
          std::cout << "rgb to yuv420p failed: " << rgb_file_path << std::endl;
          return -3;
        }

        {
          const std::string &jpeg_file_path = rgb_file_path + ".yuv420p.jpeg";
          cv::Mat yuv420pMat(imageHeight * 3 / 2, imageWidth, CV_8UC1,
                             yuv420p_buffer.data());

          cv::Mat bgrMat;
          cv::cvtColor(yuv420pMat, bgrMat, cv::COLOR_YUV2BGR_I420);
          if (bgrMat.empty()) {
            std::cout << "rgb to yuv420p: rgb->bgr failed !" << std::endl;
            return -4;
          }

          bool isSuccess = cv::imwrite(jpeg_file_path, bgrMat,
                                       {cv::IMWRITE_JPEG_QUALITY, 100});
          if (!isSuccess) {
            std::cout << "rgb to yuv420p: write jpeg failed: " << jpeg_file_path
                      << std::endl;
            return -5;
          }
        }

        std::vector<unsigned char> yuyv422_buffer(yuyv422_image_size, '\0');
        int yuyv422_buffer_len = yuyv422_image_size;
        ret = colorCodec.RGBToYUYV422(rgb_to_yuyv422_buffer.data(),
                                      rgb_image_size, yuyv422_buffer.data(),
                                      yuyv422_buffer_len);
        if (ret != 0) {
          std::cout << "rgb to yuyv422 failed: " << rgb_file_path
                    << ", ret = " << ret << std::endl;
          return -3;
        }

        {
          const std::string &jpeg_file_path = rgb_file_path + ".yuyv422.jpeg";
          cv::Mat yuyv422Mat(imageHeight, imageWidth, CV_8UC2,
                             yuyv422_buffer.data());

          cv::Mat bgrMat;
          cv::cvtColor(yuyv422Mat, bgrMat, cv::COLOR_YUV2BGR_YUYV);
          if (bgrMat.empty()) {
            std::cout << "rgb to yuyv422: rgb->bgr failed !" << std::endl;
            return -4;
          }

          bool isSuccess = cv::imwrite(jpeg_file_path, bgrMat,
                                       {cv::IMWRITE_JPEG_QUALITY, 100});
          if (!isSuccess) {
            std::cout << "rgb to yuyv422: write jpeg failed: " << jpeg_file_path
                      << std::endl;
            return -5;
          }
        }
        std::cout << "from_rgb: " << rgb_file_path << " successed !"
                  << std::endl;
      }
      std::cout << "end pinhole from_rgb " << std::endl;
    }

    // to yuyv422
    {
#if defined(ENABLE_USE_CUDA)
      const std::string &nv12_dir_path =
          "/media/sti/ee28dfb8-da74-4739-88ab-b2b836e8b78c/super_sensor_sdk_v1/"
          "test/color_test/gpu/to_yuyv422/pinhole/nv12";
      const std::string &yuv420p_dir_path =
          "/media/sti/ee28dfb8-da74-4739-88ab-b2b836e8b78c/super_sensor_sdk_v1/"
          "test/color_test/gpu/to_yuyv422/pinhole/yuv420p";
#else
      const std::string &nv12_dir_path =
          "/media/sti/ee28dfb8-da74-4739-88ab-b2b836e8b78c/super_sensor_sdk_v1/"
          "test/color_test/cpu/to_yuyv422/pinhole/nv12";
      const std::string &yuv420p_dir_path =
          "/media/sti/ee28dfb8-da74-4739-88ab-b2b836e8b78c/super_sensor_sdk_v1/"
          "test/color_test/cpu/to_yuyv422/pinhole/yuv420p";
#endif // defined(ENABLE_USE_CUDA)
      const std::vector<std::string> nv12_names = {
          "1736135284518491904.jpeg.nv12", "1736135284818706944.jpeg.nv12",
          "1736135284618563072.jpeg.nv12", "1736135284918787072.jpeg.nv12",
          "1736135284718636032.jpeg.nv12",
      };

      const std::vector<std::string> yuv420p_names = {
          "1736135288021025024.jpeg.yuv420p",
          "1736135288321242880.jpeg.yuv420p",
          "1736135288121096960.jpeg.yuv420p",
          "1736135288421312000.jpeg.yuv420p",
          "1736135288221168128.jpeg.yuv420p",
      };

      // nv12 -> yuyv422
      {
        const int nv12_image_size = robosense::color::ColorCodec::NV12ImageSize(
            imageWidth, imageHeight);
        const int yuyv422_image_size =
            robosense::color::ColorCodec::YUYV422ImageSize(imageWidth,
                                                           imageHeight);
        std::cout << "start to_yuyv422: nv12 " << std::endl;
        for (size_t i = 0; i < nv12_names.size(); ++i) {
          const std::string &nv12_file_path =
              nv12_dir_path + "/" + nv12_names[i];

          std::vector<unsigned char> nv12_buffer(nv12_image_size, '\0');
          std::ifstream ifstr(nv12_file_path,
                              std::ios_base::in | std::ios_base::binary);
          if (!ifstr.is_open()) {
            std::cout << "open nv12 to read failed: " << nv12_file_path
                      << std::endl;
            return -1;
          }

          if (!ifstr.read(reinterpret_cast<char *>(nv12_buffer.data()),
                          nv12_image_size)) {
            std::cout << "read nv12 failed: " << nv12_file_path << std::endl;
            return -2;
          }

          std::vector<unsigned char> yuyv422_buffer(yuyv422_image_size, '\0');
          int yuyv422_buffer_len = yuyv422_image_size;
          int ret = colorCodec.NV12ToRgbToYUV422(
              nv12_buffer.data(), nv12_image_size, yuyv422_buffer.data(),
              yuyv422_buffer_len);
          if (ret != 0) {
            std::cout << "nv12->yuyv422 failed: " << nv12_file_path
                      << ": ret = " << ret << std::endl;
            return -3;
          }

          {
            const std::string &jpeg_file_path =
                nv12_file_path + ".yuyv422.jpeg";
            cv::Mat yuyv422Mat(imageHeight, imageWidth, CV_8UC2,
                               yuyv422_buffer.data());

            cv::Mat bgrMat;
            cv::cvtColor(yuyv422Mat, bgrMat, cv::COLOR_YUV2BGR_YUYV);
            if (bgrMat.empty()) {
              std::cout << "nv12->yuyv422: rgb->bgr failed !" << std::endl;
              return -4;
            }

            bool isSuccess = cv::imwrite(jpeg_file_path, bgrMat,
                                         {cv::IMWRITE_JPEG_QUALITY, 100});
            if (!isSuccess) {
              std::cout << "nv12->yuyv422: write jpeg failed: "
                        << jpeg_file_path << std::endl;
              return -5;
            }
          }

          std::cout << "nv12->yuyv422: " << nv12_file_path << " successed !"
                    << std::endl;
        }
        std::cout << "end to_yuyv422: nv12 " << std::endl;
      }

      // yuv420p -> yuyu422
      {
        const int yuv420p_image_size =
            robosense::color::ColorCodec::YUV420PImageSize(imageWidth,
                                                           imageHeight);
        const int yuyv422_image_size =
            robosense::color::ColorCodec::YUYV422ImageSize(imageWidth,
                                                           imageHeight);
        std::cout << "start to_yuyv422: yuv420p" << std::endl;
        for (size_t i = 0; i < yuv420p_names.size(); ++i) {
          const std::string &yuv420p_file_path =
              yuv420p_dir_path + "/" + yuv420p_names[i];

          std::vector<unsigned char> yuv420p_buffer(yuv420p_image_size, '\0');
          std::ifstream ifstr(yuv420p_file_path,
                              std::ios_base::in | std::ios_base::binary);
          if (!ifstr.is_open()) {
            std::cout << "open yuv420p to read failed: " << yuv420p_file_path
                      << std::endl;
            return -1;
          }

          if (!ifstr.read(reinterpret_cast<char *>(yuv420p_buffer.data()),
                          yuv420p_image_size)) {
            std::cout << "read yuv420p failed: " << yuv420p_file_path
                      << std::endl;
            return -2;
          }

          std::vector<unsigned char> yuyv422_buffer(yuyv422_image_size, '\0');
          int yuyv422_buffer_len = yuyv422_image_size;
          int ret = colorCodec.YUV420PToRgbToYUYV422(
              yuv420p_buffer.data(), yuv420p_image_size, yuyv422_buffer.data(),
              yuyv422_buffer_len);
          if (ret != 0) {
            std::cout << "yuv420p->yuyv422 failed: " << yuv420p_file_path
                      << ", ret = " << ret << std::endl;
            return -3;
          }

          {
            const std::string &jpeg_file_path =
                yuv420p_file_path + ".yuyv422.jpeg";
            cv::Mat yuyv422Mat(imageHeight, imageWidth, CV_8UC2,
                               yuyv422_buffer.data());

            cv::Mat bgrMat;
            cv::cvtColor(yuyv422Mat, bgrMat, cv::COLOR_YUV2BGR_YUYV);
            if (bgrMat.empty()) {
              std::cout << "yuv420p->yuyv422: rgb->bgr failed !" << std::endl;
              return -4;
            }

            bool isSuccess = cv::imwrite(jpeg_file_path, bgrMat,
                                         {cv::IMWRITE_JPEG_QUALITY, 100});
            if (!isSuccess) {
              std::cout << "yuv420p->yuyv422: write jpeg failed: "
                        << jpeg_file_path << std::endl;
              return -5;
            }
          }

          std::cout << "yuv420p->yuyv422: " << yuv420p_file_path
                    << " successed !" << std::endl;
        }

        std::cout << "end to_yuyv422: yuv420p" << std::endl;
      }
    }

    // to yuv420x
    {
#if defined(ENABLE_USE_CUDA)
      const std::string &yuyv422_dir_path =
          "/media/sti/ee28dfb8-da74-4739-88ab-b2b836e8b78c/super_sensor_sdk_v1/"
          "test/color_test/gpu/to_yuv420x/pinhole/yuyv422";
#else
      const std::string &yuyv422_dir_path =
          "/media/sti/ee28dfb8-da74-4739-88ab-b2b836e8b78c/super_sensor_sdk_v1/"
          "test/color_test/cpu/to_yuv420x/pinhole/yuyv422";
#endif // defined(ENABLE_USE_CUDA)
      const std::vector<std::string> yuyv422_names = {
          "1736135301330651904.jpeg.yuyv422",
          "1736135301630868992.jpeg.yuyv422",
          "1736135301430725888.jpeg.yuyv422",
          "1736135301730940928.jpeg.yuyv422",
          "1736135301530797056.jpeg.yuyv422",
      };

      const int nv12_image_size =
          robosense::color::ColorCodec::NV12ImageSize(imageWidth, imageHeight);

      const int yuv420p_image_size =
          robosense::color::ColorCodec::YUV420PImageSize(imageWidth,
                                                         imageHeight);
      const int yuyv422_image_size =
          robosense::color::ColorCodec::YUYV422ImageSize(imageWidth,
                                                         imageHeight);

      std::cout << "start to_yuyv420x " << std::endl;
      for (size_t i = 0; i < yuyv422_names.size(); ++i) {
        const std::string &yuyv422_file_path =
            yuyv422_dir_path + "/" + yuyv422_names[i];

        std::vector<unsigned char> yuyv422_to_nv12_buffer(yuyv422_image_size,
                                                          '\0');
        std::vector<unsigned char> yuyv422_to_yuv420p_buffer(yuyv422_image_size,
                                                             '\0');

        std::ifstream ifstr(yuyv422_file_path,
                            std::ios_base::in | std::ios_base::binary);
        if (!ifstr.is_open()) {
          std::cout << "open yuyv422 to read failed: " << yuyv422_file_path
                    << std::endl;
          return -1;
        }

        if (!ifstr.read(reinterpret_cast<char *>(yuyv422_to_nv12_buffer.data()),
                        yuyv422_image_size)) {
          std::cout << "read yuyv422 failed: " << yuyv422_file_path
                    << std::endl;
          return -2;
        }

        memcpy(yuyv422_to_yuv420p_buffer.data(), yuyv422_to_nv12_buffer.data(),
               yuyv422_image_size);

        std::vector<unsigned char> nv12_buffer(nv12_image_size, '\0');
        std::vector<unsigned char> yuv420p_buffer(yuv420p_image_size, '\0');

        int nv12_buffer_len = nv12_image_size;
        int ret = colorCodec.YUYV422ToRgbToNV12(
            yuyv422_to_nv12_buffer.data(), yuyv422_image_size,
            nv12_buffer.data(), nv12_buffer_len);
        if (ret != 0) {
          std::cout << "yuyv422->nv12 failed: " << yuyv422_file_path
                    << std::endl;
          return -3;
        }

        {
          const std::string &jpeg_file_path = yuyv422_file_path + ".nv12.jpeg";
          cv::Mat nv12Mat(imageHeight * 3 / 2, imageWidth, CV_8UC1,
                          nv12_buffer.data());

          cv::Mat bgrMat;
          cv::cvtColor(nv12Mat, bgrMat, cv::COLOR_YUV2BGR_NV12);
          if (bgrMat.empty()) {
            std::cout << "yuyv422->nv12: rgb->bgr failed !" << std::endl;
            return -4;
          }

          bool isSuccess = cv::imwrite(jpeg_file_path, bgrMat,
                                       {cv::IMWRITE_JPEG_QUALITY, 100});
          if (!isSuccess) {
            std::cout << "yuyv422->nv12: write jpeg failed: " << jpeg_file_path
                      << std::endl;
            return -5;
          }
        }

        int yuv420p_buffer_len = yuv420p_image_size;
        ret = colorCodec.YUYV422ToRgbToYUV420P(
            yuyv422_to_yuv420p_buffer.data(), yuyv422_image_size,
            yuv420p_buffer.data(), yuv420p_buffer_len);
        if (ret != 0) {
          std::cout << "yuyv422->yuv420p failed: " << yuyv422_file_path
                    << std::endl;
          return -3;
        }

        {
          const std::string &jpeg_file_path =
              yuyv422_file_path + ".yuv420p.jpeg";
          cv::Mat yuv420pMat(imageHeight * 3 / 2, imageWidth, CV_8UC1,
                             yuv420p_buffer.data());

          cv::Mat bgrMat;
          cv::cvtColor(yuv420pMat, bgrMat, cv::COLOR_YUV2BGR_I420);
          if (bgrMat.empty()) {
            std::cout << "yuyv422->yuv420p: rgb->bgr failed !" << std::endl;
            return -4;
          }

          bool isSuccess = cv::imwrite(jpeg_file_path, bgrMat,
                                       {cv::IMWRITE_JPEG_QUALITY, 100});
          if (!isSuccess) {
            std::cout << "yuyv422->yuv420p: write jpeg failed: "
                      << jpeg_file_path << std::endl;
            return -5;
          }
        }

        std::cout << "yuyv422->yuv420x: " << yuyv422_file_path << " successed !"
                  << std::endl;
      }
      std::cout << "end to_yuyv420x" << std::endl;
    }
  }

#if defined(ENABLE_USE_CUDA)
  std::cout << "color test finished(GPU) !" << std::endl;
#else
  std::cout << "color test finished(CPU) !" << std::endl;
#endif // defined(ENABLE_USE_CUDA)

  return 0;
}