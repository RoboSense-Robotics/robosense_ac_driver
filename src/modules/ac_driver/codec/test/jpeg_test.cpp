#include "hyper_vision/codec/colorcodec.h"
#include "hyper_vision/codec/jpegcoder.h"
#include <fstream>
#include <iostream>

int main(int argc, char **argv) {
  // fisheye
  {
    const int imageWidth = 1920;
    const int imageHeight = 1536;
    const int rgb_image_size =
        robosense::color::ColorCodec::RGBImageSize(imageWidth, imageHeight);
    const int nv12_image_size =
        robosense::color::ColorCodec::NV12ImageSize(imageWidth, imageHeight);
    const int yuv420p_image_size =
        robosense::color::ColorCodec::YUV420PImageSize(imageWidth, imageHeight);
    const int yuyv422_image_size =
        robosense::color::ColorCodec::YUYV422ImageSize(imageWidth, imageHeight);

    // rgb
    {
      robosense::jpeg::JpegCoder jpegEncoder;
      robosense::jpeg::JpegCoder jpegDecoder;

      robosense::jpeg::JpegCodesConfig config;
      config.coderType = robosense::jpeg::JPEG_CODER_TYPE::RS_JPEG_CODER_ENCODE;
      config.imageFrameFormat = robosense::common::FRAME_FORMAT_RGB24;
      config.imageWidth = imageWidth;
      config.imageHeight = imageHeight;
      config.gpuDeviceId = 0;

      int ret = jpegEncoder.init(config);
      if (ret != 0) {
        std::cout << "fisheye jpeg encoder(rgb) initial failed: ret = " << ret
                  << std::endl;
        return -1;
      }

      auto config2 = config;
      config2.coderType =
          robosense::jpeg::JPEG_CODER_TYPE::RS_JPEG_CODER_DECODE;
      ret = jpegDecoder.init(config2);
      if (ret != 0) {
        std::cout << "fisheye jpeg decoder(rgb) initial failed: ret = " << ret
                  << std::endl;
        return -2;
      }

#if defined(ENABLE_USE_CUDA)
      const std::string &rgb_dir_path =
          "/media/sti/ee28dfb8-da74-4739-88ab-b2b836e8b78c/super_sensor_sdk_v1/"
          "test/jpeg_test/gpu/fisheye/rgb";
#else
      const std::string &rgb_dir_path =
          "/media/sti/ee28dfb8-da74-4739-88ab-b2b836e8b78c/super_sensor_sdk_v1/"
          "test/jpeg_test/cpu/fisheye/rgb";
#endif // defined(ENABLE_USE_CUDA)
      const std::vector<std::string> rgb_names = {
          "1736135291623630080.jpeg.rgb", "1736135291923846912.jpeg.rgb",
          "1736135291723704064.jpeg.rgb", "1736135292023918080.jpeg.rgb",
          "1736135291823773952.jpeg.rgb",
      };

      for (size_t i = 0; i < rgb_names.size(); ++i) {
        const std::string &rgb_file_path = rgb_dir_path + "/" + rgb_names[i];
        std::vector<unsigned char> rgb_buffer(rgb_image_size, '\0');
        std::ifstream ifstr(rgb_file_path,
                            std::ios_base::in | std::ios_base::binary);
        if (!ifstr.is_open()) {
          std::cout << "rgb -> jpeg: open file to read failed: "
                    << rgb_file_path << std::endl;
          return -1;
        }

        if (!ifstr.read(reinterpret_cast<char *>(rgb_buffer.data()),
                        rgb_image_size)) {
          std::cout << "rgb -> jpeg: read data failed: " << rgb_file_path
                    << std::endl;
          return -2;
        }

        std::vector<unsigned char> jpeg_buffer(rgb_image_size * 1.5, '\0');
        size_t jpeg_buffer_len = jpeg_buffer.size();
        int ret = jpegEncoder.encode(rgb_buffer.data(), rgb_image_size,
                                     jpeg_buffer.data(), jpeg_buffer_len);
        if (ret < 0) {
          std::cout << "rgb -> jpeg: encode failed: " << rgb_file_path
                    << ", ret = " << ret << std::endl;
          return -3;
        }

        const std::string &jpeg_file_path = rgb_file_path + ".jpeg";
        std::ofstream ofstr(jpeg_file_path,
                            std::ios_base::out | std::ios_base::binary);
        if (!ofstr.is_open()) {
          std::cout << "rgb -> jpeg: open file to write jpeg data failed: "
                    << jpeg_file_path << std::endl;
          return -4;
        }

        if (!ofstr.write(reinterpret_cast<const char *>(jpeg_buffer.data()),
                         jpeg_buffer_len)) {
          std::cout << "rgb -> jpeg: write jpeg data failed: " << jpeg_file_path
                    << std::endl;
          return -5;
        }
        ofstr.close();

        std::vector<unsigned char> jpeg_decode_buffer(rgb_image_size, '\0');
        size_t jpeg_decode_buffer_len = rgb_image_size;
        ret = jpegDecoder.decode(jpeg_buffer.data(), jpeg_buffer_len,
                                 jpeg_decode_buffer.data(),
                                 jpeg_decode_buffer_len);

        if (ret != 0) {
          std::cout << "rgb -> jpeg: decode failed: " << jpeg_file_path
                    << ", ret = " << ret << std::endl;
          return -6;
        }

        const std::string &jpeg_decode_file_path =
            rgb_file_path + ".decode.jpeg";
        cv::Mat rgbMat(config.imageHeight, config.imageWidth, CV_8UC3,
                       jpeg_decode_buffer.data());

        cv::Mat bgrMat;
        cv::cvtColor(rgbMat, bgrMat, cv::COLOR_RGB2BGR);
        if (bgrMat.empty()) {
          std::cout << "rgb -> jpeg: RGB->BGR failed !" << std::endl;
          return -7;
        }

        bool isSuccess = cv::imwrite(jpeg_decode_file_path, bgrMat,
                                     {cv::IMWRITE_JPEG_QUALITY, 100});
        if (!isSuccess) {
          std::cout << "rgb -> jpeg: write decode jpeg failed !" << std::endl;
          return -8;
        }
      }
    }

    // nv12
    {
      robosense::jpeg::JpegCoder jpegEncoder;
      robosense::jpeg::JpegCoder jpegDecoder;

      robosense::jpeg::JpegCodesConfig config;
      config.coderType = robosense::jpeg::JPEG_CODER_TYPE::RS_JPEG_CODER_ENCODE;
      config.imageFrameFormat = robosense::common::FRAME_FORMAT_NV12;
      config.imageWidth = imageWidth;
      config.imageHeight = imageHeight;
      config.gpuDeviceId = 0;

      int ret = jpegEncoder.init(config);
      if (ret != 0) {
        std::cout << "fisheye jpeg encoder(nv12) initial failed: ret = " << ret
                  << std::endl;
        return -1;
      }

      auto config2 = config;
      config2.coderType =
          robosense::jpeg::JPEG_CODER_TYPE::RS_JPEG_CODER_DECODE;
      ret = jpegDecoder.init(config2);
      if (ret != 0) {
        std::cout << "fisheye jpeg decoder(nv12) initial failed: ret = " << ret
                  << std::endl;
        return -2;
      }

#if defined(ENABLE_USE_CUDA)
      const std::string &nv12_dir_path =
          "/media/sti/ee28dfb8-da74-4739-88ab-b2b836e8b78c/super_sensor_sdk_v1/"
          "test/jpeg_test/gpu/fisheye/nv12";
#else
      const std::string &nv12_dir_path =
          "/media/sti/ee28dfb8-da74-4739-88ab-b2b836e8b78c/super_sensor_sdk_v1/"
          "test/jpeg_test/cpu/fisheye/nv12";
#endif // defined(ENABLE_USE_CUDA)
      const std::vector<std::string> nv12_names = {
          "1736135289321965056.jpeg.nv12", "1736135289622181888.jpeg.nv12",
          "1736135289422039040.jpeg.nv12", "1736135289722255872.jpeg.nv12",
          "1736135289522107904.jpeg.nv12",
      };

      for (size_t i = 0; i < nv12_names.size(); ++i) {
        const std::string &nv12_file_path = nv12_dir_path + "/" + nv12_names[i];
        std::vector<unsigned char> nv12_buffer(nv12_image_size, '\0');
        std::ifstream ifstr(nv12_file_path,
                            std::ios_base::in | std::ios_base::binary);
        if (!ifstr.is_open()) {
          std::cout << "nv12 -> jpeg: open file to read failed: "
                    << nv12_file_path << std::endl;
          return -1;
        }

        if (!ifstr.read(reinterpret_cast<char *>(nv12_buffer.data()),
                        nv12_image_size)) {
          std::cout << "nv12 -> jpeg: read data failed: " << nv12_file_path
                    << std::endl;
          return -2;
        }

        std::vector<unsigned char> jpeg_buffer(rgb_image_size * 1.5, '\0');
        size_t jpeg_buffer_len = jpeg_buffer.size();
        int ret = jpegEncoder.encode(nv12_buffer.data(), nv12_image_size,
                                     jpeg_buffer.data(), jpeg_buffer_len);
        if (ret < 0) {
          std::cout << "nv12 -> jpeg: encode failed: " << nv12_file_path
                    << ", ret = " << ret << std::endl;
          return -3;
        }

        const std::string &jpeg_file_path = nv12_file_path + ".jpeg";
        std::ofstream ofstr(jpeg_file_path,
                            std::ios_base::out | std::ios_base::binary);
        if (!ofstr.is_open()) {
          std::cout << "nv12 -> jpeg: open file to write jpeg data failed: "
                    << jpeg_file_path << std::endl;
          return -4;
        }

        if (!ofstr.write(reinterpret_cast<const char *>(jpeg_buffer.data()),
                         jpeg_buffer_len)) {
          std::cout << "nv12 -> jpeg: write jpeg data failed: "
                    << jpeg_file_path << std::endl;
          return -5;
        }
        ofstr.close();

        std::vector<unsigned char> jpeg_decode_buffer(nv12_image_size, '\0');
        size_t jpeg_decode_buffer_len = nv12_image_size;
        ret = jpegDecoder.decode(jpeg_buffer.data(), jpeg_buffer_len,
                                 jpeg_decode_buffer.data(),
                                 jpeg_decode_buffer_len);

        if (ret != 0) {
          std::cout << "nv12 -> jpeg: decode failed: " << jpeg_file_path
                    << ", ret = " << ret << std::endl;
          return -6;
        }

        const std::string &jpeg_decode_file_path =
            nv12_file_path + ".decode.jpeg";
        cv::Mat nv12Mat(config.imageHeight * 3 / 2, config.imageWidth, CV_8UC1,
                        jpeg_decode_buffer.data());

        cv::Mat bgrMat;
        cv::cvtColor(nv12Mat, bgrMat, cv::COLOR_YUV2BGR_NV12);
        if (bgrMat.empty()) {
          std::cout << "nv12 -> jpeg: NV12->BGR failed !" << std::endl;
          return -7;
        }

        bool isSuccess = cv::imwrite(jpeg_decode_file_path, bgrMat,
                                     {cv::IMWRITE_JPEG_QUALITY, 100});
        if (!isSuccess) {
          std::cout << "nv12 -> jpeg: write decode jpeg failed !" << std::endl;
          return -8;
        }
      }
    }

    // yuv420p
    {
      robosense::jpeg::JpegCoder jpegEncoder;
      robosense::jpeg::JpegCoder jpegDecoder;

      robosense::jpeg::JpegCodesConfig config;
      config.coderType = robosense::jpeg::JPEG_CODER_TYPE::RS_JPEG_CODER_ENCODE;
      config.imageFrameFormat = robosense::common::FRAME_FORMAT_YUV420P;
      config.imageWidth = imageWidth;
      config.imageHeight = imageHeight;
      config.gpuDeviceId = 0;

      int ret = jpegEncoder.init(config);
      if (ret != 0) {
        std::cout << "fisheye jpeg encoder(yuv420p) initial failed: ret = "
                  << ret << std::endl;
        return -1;
      }

      auto config2 = config;
      config2.coderType =
          robosense::jpeg::JPEG_CODER_TYPE::RS_JPEG_CODER_DECODE;
      ret = jpegDecoder.init(config2);
      if (ret != 0) {
        std::cout << "fisheye jpeg decoder(yuv420p) initial failed: ret = "
                  << ret << std::endl;
        return -2;
      }

#if defined(ENABLE_USE_CUDA)
      const std::string &yuv420p_dir_path =
          "/media/sti/ee28dfb8-da74-4739-88ab-b2b836e8b78c/super_sensor_sdk_v1/"
          "test/jpeg_test/gpu/fisheye/yuv420p";
#else
      const std::string &yuv420p_dir_path =
          "/media/sti/ee28dfb8-da74-4739-88ab-b2b836e8b78c/super_sensor_sdk_v1/"
          "test/jpeg_test/cpu/fisheye/yuv420p";
#endif // defined(ENABLE_USE_CUDA)

      const std::vector<std::string> yuv420p_names = {
          "1736135296327032064.jpeg.yuv420p",
          "1736135296627248128.jpeg.yuv420p",
          "1736135296427102976.jpeg.yuv420p",
          "1736135296727322112.jpeg.yuv420p",
          "1736135296527175936.jpeg.yuv420p",
      };

      for (size_t i = 0; i < yuv420p_names.size(); ++i) {
        const std::string &yuv420p_file_path =
            yuv420p_dir_path + "/" + yuv420p_names[i];
        std::vector<unsigned char> yuv420p_buffer(yuv420p_image_size, '\0');
        std::ifstream ifstr(yuv420p_file_path,
                            std::ios_base::in | std::ios_base::binary);
        if (!ifstr.is_open()) {
          std::cout << "yuv420p -> jpeg: open file to read failed: "
                    << yuv420p_file_path << std::endl;
          return -1;
        }

        if (!ifstr.read(reinterpret_cast<char *>(yuv420p_buffer.data()),
                        yuv420p_image_size)) {
          std::cout << "yuv420p -> jpeg: read data failed: "
                    << yuv420p_file_path << std::endl;
          return -2;
        }

        std::vector<unsigned char> jpeg_buffer(rgb_image_size * 1.5, '\0');
        size_t jpeg_buffer_len = jpeg_buffer.size();
        int ret = jpegEncoder.encode(yuv420p_buffer.data(), yuv420p_image_size,
                                     jpeg_buffer.data(), jpeg_buffer_len);
        if (ret < 0) {
          std::cout << "yuv420p -> jpeg: encode failed: " << yuv420p_file_path
                    << ", ret = " << ret << std::endl;
          return -3;
        }

        const std::string &jpeg_file_path = yuv420p_file_path + ".jpeg";
        std::ofstream ofstr(jpeg_file_path,
                            std::ios_base::out | std::ios_base::binary);
        if (!ofstr.is_open()) {
          std::cout << "yuv420p -> jpeg: open file to write jpeg data failed: "
                    << jpeg_file_path << std::endl;
          return -4;
        }

        if (!ofstr.write(reinterpret_cast<const char *>(jpeg_buffer.data()),
                         jpeg_buffer_len)) {
          std::cout << "yuv420p -> jpeg: write jpeg data failed: "
                    << jpeg_file_path << std::endl;
          return -5;
        }
        ofstr.close();

        std::vector<unsigned char> jpeg_decode_buffer(yuv420p_image_size, '\0');
        size_t jpeg_decode_buffer_len = yuv420p_image_size;
        ret = jpegDecoder.decode(jpeg_buffer.data(), jpeg_buffer_len,
                                 jpeg_decode_buffer.data(),
                                 jpeg_decode_buffer_len);

        if (ret != 0) {
          std::cout << "yuv420p -> jpeg: decode failed: " << jpeg_file_path
                    << ", ret = " << ret << std::endl;
          return -6;
        }

        const std::string &jpeg_decode_file_path =
            yuv420p_file_path + ".decode.jpeg";
        cv::Mat yuv420pMat(config.imageHeight * 3 / 2, config.imageWidth,
                           CV_8UC1, jpeg_decode_buffer.data());

        cv::Mat bgrMat;
        cv::cvtColor(yuv420pMat, bgrMat, cv::COLOR_YUV2BGR_I420);
        if (bgrMat.empty()) {
          std::cout << "yuv420p -> jpeg: YUV420P->BGR failed !" << std::endl;
          return -7;
        }

        bool isSuccess = cv::imwrite(jpeg_decode_file_path, bgrMat,
                                     {cv::IMWRITE_JPEG_QUALITY, 100});
        if (!isSuccess) {
          std::cout << "yuv420p -> jpeg: write decode jpeg failed !"
                    << std::endl;
          return -8;
        }
      }
    }

    // yuyv422
    {
      robosense::jpeg::JpegCoder jpegEncoder;
      robosense::jpeg::JpegCoder jpegDecoder;

      robosense::jpeg::JpegCodesConfig config;
      config.coderType = robosense::jpeg::JPEG_CODER_TYPE::RS_JPEG_CODER_ENCODE;
      config.imageFrameFormat = robosense::common::FRAME_FORMAT_YUYV422;
      config.imageWidth = imageWidth;
      config.imageHeight = imageHeight;
      config.gpuDeviceId = 0;

      int ret = jpegEncoder.init(config);
      if (ret != 0) {
        std::cout << "fisheye jpeg encoder(yuyv422) initial failed: ret = "
                  << ret << std::endl;
        return -1;
      }

      auto config2 = config;
      config2.coderType =
          robosense::jpeg::JPEG_CODER_TYPE::RS_JPEG_CODER_DECODE;
      ret = jpegDecoder.init(config2);
      if (ret != 0) {
        std::cout << "fisheye jpeg decoder(yuyv422) initial failed: ret = "
                  << ret << std::endl;
        return -2;
      }

#if defined(ENABLE_USE_CUDA)
      const std::string &yuyv422_dir_path =
          "/media/sti/ee28dfb8-da74-4739-88ab-b2b836e8b78c/super_sensor_sdk_v1/"
          "test/jpeg_test/gpu/fisheye/yuyv422";
#else
      const std::string &yuyv422_dir_path =
          "/media/sti/ee28dfb8-da74-4739-88ab-b2b836e8b78c/super_sensor_sdk_v1/"
          "test/jpeg_test/cpu/fisheye/yuyv422";
#endif // defined(ENABLE_USE_CUDA)
      const std::vector<std::string> yuyv422_names = {
          "1736135282316897024.jpeg.yuyv422",
          "1736135282617114880.jpeg.yuyv422",
          "1736135282416971008.jpeg.yuyv422",
          "1736135282717186048.jpeg.yuyv422",
          "1736135282517041920.jpeg.yuyv422",
      };

      for (size_t i = 0; i < yuyv422_names.size(); ++i) {
        const std::string &yuyv422_file_path =
            yuyv422_dir_path + "/" + yuyv422_names[i];
        std::vector<unsigned char> yuyv422_buffer(yuyv422_image_size, '\0');
        std::ifstream ifstr(yuyv422_file_path,
                            std::ios_base::in | std::ios_base::binary);
        if (!ifstr.is_open()) {
          std::cout << "yuyv422 -> jpeg: open file to read failed: "
                    << yuyv422_file_path << std::endl;
          return -1;
        }

        if (!ifstr.read(reinterpret_cast<char *>(yuyv422_buffer.data()),
                        yuyv422_image_size)) {
          std::cout << "yuyv422 -> jpeg: read data failed: "
                    << yuyv422_file_path << std::endl;
          return -2;
        }

        std::vector<unsigned char> jpeg_buffer(rgb_image_size * 1.5, '\0');
        size_t jpeg_buffer_len = jpeg_buffer.size();
        int ret = jpegEncoder.encode(yuyv422_buffer.data(), yuyv422_image_size,
                                     jpeg_buffer.data(), jpeg_buffer_len);
        if (ret < 0) {
          std::cout << "yuyv422 -> jpeg: encode failed: " << yuyv422_file_path
                    << ", ret = " << ret << std::endl;
          return -3;
        }

        const std::string &jpeg_file_path = yuyv422_file_path + ".jpeg";
        std::ofstream ofstr(jpeg_file_path,
                            std::ios_base::out | std::ios_base::binary);
        if (!ofstr.is_open()) {
          std::cout << "yuyv422 -> jpeg: open file to write jpeg data failed: "
                    << jpeg_file_path << std::endl;
          return -4;
        }

        if (!ofstr.write(reinterpret_cast<const char *>(jpeg_buffer.data()),
                         jpeg_buffer_len)) {
          std::cout << "yuyv422 -> jpeg: write jpeg data failed: "
                    << jpeg_file_path << std::endl;
          return -5;
        }
        ofstr.close();

        std::vector<unsigned char> jpeg_decode_buffer(yuyv422_image_size, '\0');
        size_t jpeg_decode_buffer_len = yuyv422_image_size;
        ret = jpegDecoder.decode(jpeg_buffer.data(), jpeg_buffer_len,
                                 jpeg_decode_buffer.data(),
                                 jpeg_decode_buffer_len);

        if (ret != 0) {
          std::cout << "yuyv422 -> jpeg: decode failed: " << jpeg_file_path
                    << ", ret = " << ret << std::endl;
          return -6;
        }

        const std::string &jpeg_decode_file_path =
            yuyv422_file_path + ".decode.jpeg";
        cv::Mat yuyv422Mat(config.imageHeight, config.imageWidth, CV_8UC2,
                           jpeg_decode_buffer.data());

        cv::Mat bgrMat;
        cv::cvtColor(yuyv422Mat, bgrMat, cv::COLOR_YUV2BGR_YUYV);
        if (bgrMat.empty()) {
          std::cout << "yuyv422 -> jpeg: YUV420P->BGR failed !" << std::endl;
          return -7;
        }

        bool isSuccess = cv::imwrite(jpeg_decode_file_path, bgrMat,
                                     {cv::IMWRITE_JPEG_QUALITY, 100});
        if (!isSuccess) {
          std::cout << "yuyv422 -> jpeg: write decode jpeg failed !"
                    << std::endl;
          return -8;
        }
      }
    }
  }

  // pinhole
  {
    const int imageWidth = 1920;
    const int imageHeight = 1080;
    const int rgb_image_size =
        robosense::color::ColorCodec::RGBImageSize(imageWidth, imageHeight);
    const int nv12_image_size =
        robosense::color::ColorCodec::NV12ImageSize(imageWidth, imageHeight);
    const int yuv420p_image_size =
        robosense::color::ColorCodec::YUV420PImageSize(imageWidth, imageHeight);
    const int yuyv422_image_size =
        robosense::color::ColorCodec::YUYV422ImageSize(imageWidth, imageHeight);

    // rgb
    {
      robosense::jpeg::JpegCoder jpegEncoder;
      robosense::jpeg::JpegCoder jpegDecoder;

      robosense::jpeg::JpegCodesConfig config;
      config.coderType = robosense::jpeg::JPEG_CODER_TYPE::RS_JPEG_CODER_ENCODE;
      config.imageFrameFormat = robosense::common::FRAME_FORMAT_RGB24;
      config.imageWidth = imageWidth;
      config.imageHeight = imageHeight;
      config.gpuDeviceId = 0;

      int ret = jpegEncoder.init(config);
      if (ret != 0) {
        std::cout << "pinhole jpeg encoder(rgb) initial failed: ret = " << ret
                  << std::endl;
        return -1;
      }

      auto config2 = config;
      config2.coderType =
          robosense::jpeg::JPEG_CODER_TYPE::RS_JPEG_CODER_DECODE;
      ret = jpegDecoder.init(config2);
      if (ret != 0) {
        std::cout << "pinhole jpeg decoder(rgb) initial failed: ret = " << ret
                  << std::endl;
        return -2;
      }

#if defined(ENABLE_USE_CUDA)
      const std::string &rgb_dir_path =
          "/media/sti/ee28dfb8-da74-4739-88ab-b2b836e8b78c/super_sensor_sdk_v1/"
          "test/jpeg_test/gpu/pinhole/rgb";
#else
      const std::string &rgb_dir_path =
          "/media/sti/ee28dfb8-da74-4739-88ab-b2b836e8b78c/super_sensor_sdk_v1/"
          "test/jpeg_test/cpu/pinhole/rgb";
#endif // defined(ENABLE_USE_CUDA)
      const std::vector<std::string> rgb_names = {
          "1736135300730217984.jpeg.rgb", "1736135301030435072.jpeg.rgb",
          "1736135300830289920.jpeg.rgb", "1736135301130508032.jpeg.rgb",
          "1736135300930361088.jpeg.rgb",
      };

      for (size_t i = 0; i < rgb_names.size(); ++i) {
        const std::string &rgb_file_path = rgb_dir_path + "/" + rgb_names[i];
        std::vector<unsigned char> rgb_buffer(rgb_image_size, '\0');
        std::ifstream ifstr(rgb_file_path,
                            std::ios_base::in | std::ios_base::binary);
        if (!ifstr.is_open()) {
          std::cout << "rgb -> jpeg: open file to read failed: "
                    << rgb_file_path << std::endl;
          return -1;
        }

        if (!ifstr.read(reinterpret_cast<char *>(rgb_buffer.data()),
                        rgb_image_size)) {
          std::cout << "rgb -> jpeg: read data failed: " << rgb_file_path
                    << std::endl;
          return -2;
        }

        std::vector<unsigned char> jpeg_buffer(rgb_image_size * 1.5, '\0');
        size_t jpeg_buffer_len = jpeg_buffer.size();
        int ret = jpegEncoder.encode(rgb_buffer.data(), rgb_image_size,
                                     jpeg_buffer.data(), jpeg_buffer_len);
        if (ret < 0) {
          std::cout << "rgb -> jpeg: encode failed: " << rgb_file_path
                    << ", ret = " << ret << std::endl;
          return -3;
        }

        const std::string &jpeg_file_path = rgb_file_path + ".jpeg";
        std::ofstream ofstr(jpeg_file_path,
                            std::ios_base::out | std::ios_base::binary);
        if (!ofstr.is_open()) {
          std::cout << "rgb -> jpeg: open file to write jpeg data failed: "
                    << jpeg_file_path << std::endl;
          return -4;
        }

        if (!ofstr.write(reinterpret_cast<const char *>(jpeg_buffer.data()),
                         jpeg_buffer_len)) {
          std::cout << "rgb -> jpeg: write jpeg data failed: " << jpeg_file_path
                    << std::endl;
          return -5;
        }
        ofstr.close();

        std::vector<unsigned char> jpeg_decode_buffer(rgb_image_size, '\0');
        size_t jpeg_decode_buffer_len = rgb_image_size;
        ret = jpegDecoder.decode(jpeg_buffer.data(), jpeg_buffer_len,
                                 jpeg_decode_buffer.data(),
                                 jpeg_decode_buffer_len);

        if (ret != 0) {
          std::cout << "rgb -> jpeg: decode failed: " << jpeg_file_path
                    << ", ret = " << ret << std::endl;
          return -6;
        }

        const std::string &jpeg_decode_file_path =
            rgb_file_path + ".decode.jpeg";
        cv::Mat rgbMat(config.imageHeight, config.imageWidth, CV_8UC3,
                       jpeg_decode_buffer.data());

        cv::Mat bgrMat;
        cv::cvtColor(rgbMat, bgrMat, cv::COLOR_RGB2BGR);
        if (bgrMat.empty()) {
          std::cout << "rgb -> jpeg: RGB->BGR failed !" << std::endl;
          return -7;
        }

        bool isSuccess = cv::imwrite(jpeg_decode_file_path, bgrMat,
                                     {cv::IMWRITE_JPEG_QUALITY, 100});
        if (!isSuccess) {
          std::cout << "rgb -> jpeg: write decode jpeg failed !" << std::endl;
          return -8;
        }
      }
    }

    // nv12
    {
      robosense::jpeg::JpegCoder jpegEncoder;
      robosense::jpeg::JpegCoder jpegDecoder;

      robosense::jpeg::JpegCodesConfig config;
      config.coderType = robosense::jpeg::JPEG_CODER_TYPE::RS_JPEG_CODER_ENCODE;
      config.imageFrameFormat = robosense::common::FRAME_FORMAT_NV12;
      config.imageWidth = imageWidth;
      config.imageHeight = imageHeight;
      config.gpuDeviceId = 0;

      int ret = jpegEncoder.init(config);
      if (ret != 0) {
        std::cout << "pinhole jpeg encoder(nv12) initial failed: ret = " << ret
                  << std::endl;
        return -1;
      }

      auto config2 = config;
      config2.coderType =
          robosense::jpeg::JPEG_CODER_TYPE::RS_JPEG_CODER_DECODE;
      ret = jpegDecoder.init(config2);
      if (ret != 0) {
        std::cout << "pinhole jpeg decoder(nv12) initial failed: ret = " << ret
                  << std::endl;
        return -2;
      }

#if defined(ENABLE_USE_CUDA)
      const std::string &nv12_dir_path =
          "/media/sti/ee28dfb8-da74-4739-88ab-b2b836e8b78c/super_sensor_sdk_v1/"
          "test/jpeg_test/gpu/pinhole/nv12";
#else
      const std::string &nv12_dir_path =
          "/media/sti/ee28dfb8-da74-4739-88ab-b2b836e8b78c/super_sensor_sdk_v1/"
          "test/jpeg_test/cpu/pinhole/nv12";
#endif // defined(ENABLE_USE_CUDA)
      const std::vector<std::string> nv12_names = {
          "1736135291623630080.jpeg.nv12", "1736135291923846912.jpeg.nv12",
          "1736135291723704064.jpeg.nv12", "1736135292023918080.jpeg.nv12",
          "1736135291823773952.jpeg.nv12",
      };

      for (size_t i = 0; i < nv12_names.size(); ++i) {
        const std::string &nv12_file_path = nv12_dir_path + "/" + nv12_names[i];
        std::vector<unsigned char> nv12_buffer(nv12_image_size, '\0');
        std::ifstream ifstr(nv12_file_path,
                            std::ios_base::in | std::ios_base::binary);
        if (!ifstr.is_open()) {
          std::cout << "nv12 -> jpeg: open file to read failed: "
                    << nv12_file_path << std::endl;
          return -1;
        }

        if (!ifstr.read(reinterpret_cast<char *>(nv12_buffer.data()),
                        nv12_image_size)) {
          std::cout << "nv12 -> jpeg: read data failed: " << nv12_file_path
                    << std::endl;
          return -2;
        }

        std::vector<unsigned char> jpeg_buffer(rgb_image_size * 1.5, '\0');
        size_t jpeg_buffer_len = jpeg_buffer.size();
        int ret = jpegEncoder.encode(nv12_buffer.data(), nv12_image_size,
                                     jpeg_buffer.data(), jpeg_buffer_len);
        if (ret < 0) {
          std::cout << "nv12 -> jpeg: encode failed: " << nv12_file_path
                    << ", ret = " << ret << std::endl;
          return -3;
        }

        const std::string &jpeg_file_path = nv12_file_path + ".jpeg";
        std::ofstream ofstr(jpeg_file_path,
                            std::ios_base::out | std::ios_base::binary);
        if (!ofstr.is_open()) {
          std::cout << "nv12 -> jpeg: open file to write jpeg data failed: "
                    << jpeg_file_path << std::endl;
          return -4;
        }

        if (!ofstr.write(reinterpret_cast<const char *>(jpeg_buffer.data()),
                         jpeg_buffer_len)) {
          std::cout << "nv12 -> jpeg: write jpeg data failed: "
                    << jpeg_file_path << std::endl;
          return -5;
        }
        ofstr.close();

        std::vector<unsigned char> jpeg_decode_buffer(nv12_image_size, '\0');
        size_t jpeg_decode_buffer_len = nv12_image_size;
        ret = jpegDecoder.decode(jpeg_buffer.data(), jpeg_buffer_len,
                                 jpeg_decode_buffer.data(),
                                 jpeg_decode_buffer_len);

        if (ret != 0) {
          std::cout << "nv12 -> jpeg: decode failed: " << jpeg_file_path
                    << ", ret = " << ret << std::endl;
          return -6;
        }

        const std::string &jpeg_decode_file_path =
            nv12_file_path + ".decode.jpeg";
        cv::Mat nv12Mat(config.imageHeight * 3 / 2, config.imageWidth, CV_8UC1,
                        jpeg_decode_buffer.data());

        cv::Mat bgrMat;
        cv::cvtColor(nv12Mat, bgrMat, cv::COLOR_YUV2BGR_NV12);
        if (bgrMat.empty()) {
          std::cout << "nv12 -> jpeg: NV12->BGR failed !" << std::endl;
          return -7;
        }

        bool isSuccess = cv::imwrite(jpeg_decode_file_path, bgrMat,
                                     {cv::IMWRITE_JPEG_QUALITY, 100});
        if (!isSuccess) {
          std::cout << "nv12 -> jpeg: write decode jpeg failed !" << std::endl;
          return -8;
        }
      }
    }

    // yuv420p
    {
      robosense::jpeg::JpegCoder jpegEncoder;
      robosense::jpeg::JpegCoder jpegDecoder;

      robosense::jpeg::JpegCodesConfig config;
      config.coderType = robosense::jpeg::JPEG_CODER_TYPE::RS_JPEG_CODER_ENCODE;
      config.imageFrameFormat = robosense::common::FRAME_FORMAT_YUV420P;
      config.imageWidth = imageWidth;
      config.imageHeight = imageHeight;
      config.gpuDeviceId = 0;

      int ret = jpegEncoder.init(config);
      if (ret != 0) {
        std::cout << "pinhole jpeg encoder(yuv420p) initial failed: ret = "
                  << ret << std::endl;
        return -1;
      }

      auto config2 = config;
      config2.coderType =
          robosense::jpeg::JPEG_CODER_TYPE::RS_JPEG_CODER_DECODE;
      ret = jpegDecoder.init(config2);
      if (ret != 0) {
        std::cout << "pinhole jpeg decoder(yuv420p) initial failed: ret = "
                  << ret << std::endl;
        return -2;
      }

#if defined(ENABLE_USE_CUDA)
      const std::string &yuv420p_dir_path =
          "/media/sti/ee28dfb8-da74-4739-88ab-b2b836e8b78c/super_sensor_sdk_v1/"
          "test/jpeg_test/gpu/pinhole/yuv420p";
#else
      const std::string &yuv420p_dir_path =
          "/media/sti/ee28dfb8-da74-4739-88ab-b2b836e8b78c/super_sensor_sdk_v1/"
          "test/jpeg_test/cpu/pinhole/yuv420p";
#endif // defined(ENABLE_USE_CUDA)

      const std::vector<std::string> yuv420p_names = {
          "1736135307635212032.jpeg.yuv420p",
          "1736135307935430912.jpeg.yuv420p",
          "1736135307735283968.jpeg.yuv420p",
          "1736135308035503104.jpeg.yuv420p",
          "1736135307835355904.jpeg.yuv420p",
      };

      for (size_t i = 0; i < yuv420p_names.size(); ++i) {
        const std::string &yuv420p_file_path =
            yuv420p_dir_path + "/" + yuv420p_names[i];
        std::vector<unsigned char> yuv420p_buffer(yuv420p_image_size, '\0');
        std::ifstream ifstr(yuv420p_file_path,
                            std::ios_base::in | std::ios_base::binary);
        if (!ifstr.is_open()) {
          std::cout << "yuv420p -> jpeg: open file to read failed: "
                    << yuv420p_file_path << std::endl;
          return -1;
        }

        if (!ifstr.read(reinterpret_cast<char *>(yuv420p_buffer.data()),
                        yuv420p_image_size)) {
          std::cout << "yuv420p -> jpeg: read data failed: "
                    << yuv420p_file_path << std::endl;
          return -2;
        }

        std::vector<unsigned char> jpeg_buffer(rgb_image_size * 1.5, '\0');
        size_t jpeg_buffer_len = jpeg_buffer.size();
        int ret = jpegEncoder.encode(yuv420p_buffer.data(), yuv420p_image_size,
                                     jpeg_buffer.data(), jpeg_buffer_len);
        if (ret < 0) {
          std::cout << "yuv420p -> jpeg: encode failed: " << yuv420p_file_path
                    << ", ret = " << ret << std::endl;
          return -3;
        }

        const std::string &jpeg_file_path = yuv420p_file_path + ".jpeg";
        std::ofstream ofstr(jpeg_file_path,
                            std::ios_base::out | std::ios_base::binary);
        if (!ofstr.is_open()) {
          std::cout << "yuv420p -> jpeg: open file to write jpeg data failed: "
                    << jpeg_file_path << std::endl;
          return -4;
        }

        if (!ofstr.write(reinterpret_cast<const char *>(jpeg_buffer.data()),
                         jpeg_buffer_len)) {
          std::cout << "yuv420p -> jpeg: write jpeg data failed: "
                    << jpeg_file_path << std::endl;
          return -5;
        }
        ofstr.close();

        std::vector<unsigned char> jpeg_decode_buffer(yuv420p_image_size, '\0');
        size_t jpeg_decode_buffer_len = yuv420p_image_size;
        ret = jpegDecoder.decode(jpeg_buffer.data(), jpeg_buffer_len,
                                 jpeg_decode_buffer.data(),
                                 jpeg_decode_buffer_len);

        if (ret != 0) {
          std::cout << "yuv420p -> jpeg: decode failed: " << jpeg_file_path
                    << ", ret = " << ret << std::endl;
          return -6;
        }

        const std::string &jpeg_decode_file_path =
            yuv420p_file_path + ".decode.jpeg";
        cv::Mat yuv420pMat(config.imageHeight * 3 / 2, config.imageWidth,
                           CV_8UC1, jpeg_decode_buffer.data());

        cv::Mat bgrMat;
        cv::cvtColor(yuv420pMat, bgrMat, cv::COLOR_YUV2BGR_I420);
        if (bgrMat.empty()) {
          std::cout << "yuv420p -> jpeg: YUV420P->BGR failed !" << std::endl;
          return -7;
        }

        bool isSuccess = cv::imwrite(jpeg_decode_file_path, bgrMat,
                                     {cv::IMWRITE_JPEG_QUALITY, 100});
        if (!isSuccess) {
          std::cout << "yuv420p -> jpeg: write decode jpeg failed !"
                    << std::endl;
          return -8;
        }
      }
    }

    // yuyv422
    {
      robosense::jpeg::JpegCoder jpegEncoder;
      robosense::jpeg::JpegCoder jpegDecoder;

      robosense::jpeg::JpegCodesConfig config;
      config.coderType = robosense::jpeg::JPEG_CODER_TYPE::RS_JPEG_CODER_ENCODE;
      config.imageFrameFormat = robosense::common::FRAME_FORMAT_YUYV422;
      config.imageWidth = imageWidth;
      config.imageHeight = imageHeight;
      config.gpuDeviceId = 0;

      int ret = jpegEncoder.init(config);
      if (ret != 0) {
        std::cout << "pinhole jpeg encoder(yuyv422) initial failed: ret = "
                  << ret << std::endl;
        return -1;
      }

      auto config2 = config;
      config2.coderType =
          robosense::jpeg::JPEG_CODER_TYPE::RS_JPEG_CODER_DECODE;
      ret = jpegDecoder.init(config2);
      if (ret != 0) {
        std::cout << "pinhole jpeg decoder(yuyv422) initial failed: ret = "
                  << ret << std::endl;
        return -2;
      }

#if defined(ENABLE_USE_CUDA)
      const std::string &yuyv422_dir_path =
          "/media/sti/ee28dfb8-da74-4739-88ab-b2b836e8b78c/super_sensor_sdk_v1/"
          "test/jpeg_test/gpu/pinhole/yuyv422";
#else
      const std::string &yuyv422_dir_path =
          "/media/sti/ee28dfb8-da74-4739-88ab-b2b836e8b78c/super_sensor_sdk_v1/"
          "test/jpeg_test/cpu/pinhole/yuyv422";
#endif // defined(ENABLE_USE_CUDA)
      const std::vector<std::string> yuyv422_names = {
          "1736135305533692928.jpeg.yuyv422",
          "1736135305833907968.jpeg.yuyv422",
          "1736135305633765120.jpeg.yuyv422",
          "1736135305933981952.jpeg.yuyv422",
          "1736135305733839104.jpeg.yuyv422",
      };

      for (size_t i = 0; i < yuyv422_names.size(); ++i) {
        const std::string &yuyv422_file_path =
            yuyv422_dir_path + "/" + yuyv422_names[i];
        std::vector<unsigned char> yuyv422_buffer(yuyv422_image_size, '\0');
        std::ifstream ifstr(yuyv422_file_path,
                            std::ios_base::in | std::ios_base::binary);
        if (!ifstr.is_open()) {
          std::cout << "yuyv422 -> jpeg: open file to read failed: "
                    << yuyv422_file_path << std::endl;
          return -1;
        }

        if (!ifstr.read(reinterpret_cast<char *>(yuyv422_buffer.data()),
                        yuyv422_image_size)) {
          std::cout << "yuyv422 -> jpeg: read data failed: "
                    << yuyv422_file_path << std::endl;
          return -2;
        }

        std::vector<unsigned char> jpeg_buffer(rgb_image_size * 1.5, '\0');
        size_t jpeg_buffer_len = jpeg_buffer.size();
        int ret = jpegEncoder.encode(yuyv422_buffer.data(), yuyv422_image_size,
                                     jpeg_buffer.data(), jpeg_buffer_len);
        if (ret < 0) {
          std::cout << "yuyv422 -> jpeg: encode failed: " << yuyv422_file_path
                    << ", ret = " << ret << std::endl;
          return -3;
        }

        const std::string &jpeg_file_path = yuyv422_file_path + ".jpeg";
        std::ofstream ofstr(jpeg_file_path,
                            std::ios_base::out | std::ios_base::binary);
        if (!ofstr.is_open()) {
          std::cout << "yuyv422 -> jpeg: open file to write jpeg data failed: "
                    << jpeg_file_path << std::endl;
          return -4;
        }

        if (!ofstr.write(reinterpret_cast<const char *>(jpeg_buffer.data()),
                         jpeg_buffer_len)) {
          std::cout << "yuyv422 -> jpeg: write jpeg data failed: "
                    << jpeg_file_path << std::endl;
          return -5;
        }
        ofstr.close();

        std::vector<unsigned char> jpeg_decode_buffer(yuyv422_image_size, '\0');
        size_t jpeg_decode_buffer_len = yuyv422_image_size;
        ret = jpegDecoder.decode(jpeg_buffer.data(), jpeg_buffer_len,
                                 jpeg_decode_buffer.data(),
                                 jpeg_decode_buffer_len);

        if (ret != 0) {
          std::cout << "yuyv422 -> jpeg: decode failed: " << jpeg_file_path
                    << ", ret = " << ret << std::endl;
          return -6;
        }

        const std::string &jpeg_decode_file_path =
            yuyv422_file_path + ".decode.jpeg";
        cv::Mat yuyv422Mat(config.imageHeight, config.imageWidth, CV_8UC2,
                           jpeg_decode_buffer.data());

        cv::Mat bgrMat;
        cv::cvtColor(yuyv422Mat, bgrMat, cv::COLOR_YUV2BGR_YUYV);
        if (bgrMat.empty()) {
          std::cout << "yuyv422 -> jpeg: YUV420P->BGR failed !" << std::endl;
          return -7;
        }

        bool isSuccess = cv::imwrite(jpeg_decode_file_path, bgrMat,
                                     {cv::IMWRITE_JPEG_QUALITY, 100});
        if (!isSuccess) {
          std::cout << "yuyv422 -> jpeg: write decode jpeg failed !"
                    << std::endl;
          return -8;
        }
      }
    }
  }
#if defined(ENABLE_USE_CUDA)
  std::cout << "jpeg test finished(GPU) !" << std::endl;
#else
  std::cout << "jpeg test finished(CPU) !" << std::endl;
#endif // defined(ENABLE_USE_CUDA)

  return 0;
}