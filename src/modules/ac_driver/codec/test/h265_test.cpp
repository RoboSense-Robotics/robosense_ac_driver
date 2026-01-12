#include "hyper_vision/codec/colorcodec.h"
#include "hyper_vision/codec/h265coder.h"

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

    const std::string &fisheye_dir_path =
        "/media/sti/ee28dfb8-da74-4739-88ab-b2b836e8b78c/super_sensor_sdk_v1/"
        "test/h265_test/fisheye";

    const std::string &nv12_dir_path = fisheye_dir_path + "/nv12/";
    const std::string &nv12_name_file_path = nv12_dir_path + "/name.txt";

    const std::string &rgb_dir_path = fisheye_dir_path + "/rgb/";
    const std::string &rgb_name_file_path = rgb_dir_path + "/name.txt";

    const std::string &yuv420p_dir_path = fisheye_dir_path + "/yuv420p/";
    const std::string &yuv420p_name_file_path = yuv420p_dir_path + "/name.txt";

    const std::string &yuyv422_dir_path = fisheye_dir_path + "/yuyv422/";
    const std::string &yuyv422_name_file_path = yuyv422_dir_path + "/name.txt";

    // rgb
    {
      robosense::h265::H265Coder h265Encoder;
      robosense::h265::H265Coder h265Decoder;

      robosense::h265::H265CodesConfig config;
      config.codeType = robosense::h265::H265_CODER_TYPE::RS_H265_CODER_ENCODE;
      config.imageFrameFormat =
          robosense::common::ImageFrameFormat::FRAME_FORMAT_RGB24;
      config.imgWidth = imageWidth;
      config.imgHeight = imageHeight;
      config.imgFreq = 10;
      config.codesRateType =
          robosense::h265::H265_CODES_RATE_TYPE::RS_H265_CODES_CUSTOM;
      config.imgCodeRate = 4000000;

      auto config2 = config;
      config2.codeType = robosense::h265::H265_CODER_TYPE::RS_H265_CODER_DECODE;

      int ret = h265Encoder.init(config);
      if (ret != 0) {
        std::cout << "fisheye h265 encoder(rgb) initial failed: ret = " << ret
                  << std::endl;
        return -1;
      }

      ret = h265Decoder.init(config2);
      if (ret != 0) {
        std::cout << "fisheye h265 decoder(rgb) initial failed: ret = " << ret
                  << std::endl;
        return -2;
      }

      // 获取文件名称
      std::vector<std::string> rgb_names;
      {
        std::ifstream ifstr(rgb_name_file_path,
                            std::ios_base::in | std::ios_base::binary);
        if (!ifstr.is_open()) {
          std::cout << "rgb->h265: open rgb name.txt to read failed: "
                    << rgb_name_file_path << std::endl;
          return -3;
        }

        std::string content;
        while (std::getline(ifstr, content)) {
          if (content.empty()) {
            continue;
          }
          rgb_names.push_back(content);
        }
      }

      std::sort(rgb_names.begin(), rgb_names.end());

      std::vector<std::string> h265_file_paths;
      for (size_t i = 0; i < rgb_names.size(); ++i) {
        const std::string &rgb_file_path = rgb_dir_path + "/" + rgb_names[i];
        std::vector<unsigned char> rgb_buffer(rgb_image_size, '\0');
        std::ifstream ifstr(rgb_file_path,
                            std::ios_base::in | std::ios_base::binary);
        if (!ifstr.is_open()) {
          std::cout << "rgb->h265: open rgb file to read failed: "
                    << rgb_file_path << std::endl;
          return -4;
        }

        if (!ifstr.read(reinterpret_cast<char *>(rgb_buffer.data()),
                        rgb_image_size)) {
          std::cout << "rgb->h265: read rgb data failed: " << rgb_file_path
                    << std::endl;
          return -5;
        }

        robosense::h265::H265Coder::CODER_BUFFER_PTR_VEC h265EncodeBufferPtr;
        std::vector<bool> iFrameFlags;
        int ret = h265Encoder.encode(rgb_buffer.data(), rgb_image_size,
                                     h265EncodeBufferPtr, iFrameFlags);
        if (ret != 0) {
          std::cout << "rgb->h265: h265 encode failed: ret = " << ret
                    << std::endl;
          return -6;
        }

        h265_file_paths.push_back(rgb_file_path);
        if (h265EncodeBufferPtr.empty()) {
          std::cout << "rgb->h265: " << rgb_file_path << " successed !"
                    << std::endl;
          continue;
        }

        for (size_t j = 0; j < h265EncodeBufferPtr.size(); ++j) {
          robosense::h265::H265Coder::CODER_BUFFER_PTR singleBufferPtr =
              h265EncodeBufferPtr[j];

          robosense::h265::H265Coder::CODER_BUFFER_PTR_VEC h265DecodeBufferPtr;
          ret =
              h265Decoder.decode(singleBufferPtr->data(),
                                 singleBufferPtr->size(), h265DecodeBufferPtr);
          if (ret != 0) {
            std::cout << "rgb->h265: h265 decode failed: ret = " << ret
                      << std::endl;
            return -7;
          }

          for (size_t k = 0; k < h265DecodeBufferPtr.size(); ++k) {
            const std::string decode_rgb_file_path = h265_file_paths[0];
            h265_file_paths.erase(h265_file_paths.begin());

            const std::string &h265_decode_file_path =
                decode_rgb_file_path + ".decode.jpeg";
            cv::Mat rgbMat(config.imgHeight, config.imgWidth, CV_8UC3,
                           h265DecodeBufferPtr[k]->data());

            cv::Mat bgrMat;
            cv::cvtColor(rgbMat, bgrMat, cv::COLOR_RGB2BGR);
            if (bgrMat.empty()) {
              std::cout << "rgb->h265: RGB->BGR failed !" << std::endl;
              return -7;
            }

            bool isSuccess = cv::imwrite(h265_decode_file_path, bgrMat,
                                         {cv::IMWRITE_JPEG_QUALITY, 100});
            if (!isSuccess) {
              std::cout << "rgb->h265: write decode jpeg failed !" << std::endl;
              return -8;
            }
          }
        }

        if (i == rgb_names.size() - 1) {
          robosense::h265::H265Coder::CODER_BUFFER_PTR_VEC h265EncodeBufferPtr;
          std::vector<bool> iFrameFlags;
          int ret = h265Encoder.encodeFlush(h265EncodeBufferPtr, iFrameFlags);
          if (ret != 0) {
            std::cout << "rgb->h265: h265 encode flush failed: ret = " << ret
                      << std::endl;
            return -1;
          }

          for (size_t j = 0; j < h265EncodeBufferPtr.size(); ++j) {
            robosense::h265::H265Coder::CODER_BUFFER_PTR singleBufferPtr =
                h265EncodeBufferPtr[j];

            robosense::h265::H265Coder::CODER_BUFFER_PTR_VEC
                h265DecodeBufferPtr;
            ret = h265Decoder.decode(singleBufferPtr->data(),
                                     singleBufferPtr->size(),
                                     h265DecodeBufferPtr);
            if (ret != 0) {
              std::cout << "rgb->h265: h265 decode failed: ret = " << ret
                        << std::endl;
              return -7;
            }

            for (size_t k = 0; k < h265DecodeBufferPtr.size(); ++k) {
              const std::string decode_rgb_file_path = h265_file_paths[0];
              h265_file_paths.erase(h265_file_paths.begin());

              const std::string &h265_decode_file_path =
                  decode_rgb_file_path + ".decode.jpeg";
              cv::Mat rgbMat(config.imgHeight, config.imgWidth, CV_8UC3,
                             h265DecodeBufferPtr[k]->data());

              cv::Mat bgrMat;
              cv::cvtColor(rgbMat, bgrMat, cv::COLOR_RGB2BGR);
              if (bgrMat.empty()) {
                std::cout << "rgb->h265: RGB->BGR failed !" << std::endl;
                return -7;
              }

              bool isSuccess = cv::imwrite(h265_decode_file_path, bgrMat,
                                           {cv::IMWRITE_JPEG_QUALITY, 100});
              if (!isSuccess) {
                std::cout << "rgb->h265: write decode jpeg failed !"
                          << std::endl;
                return -8;
              }
            }
          }

          {
            robosense::h265::H265Coder::CODER_BUFFER_PTR_VEC
                h265DecodeBufferPtr;
            ret = h265Decoder.decodeFlush(h265DecodeBufferPtr);
            if (ret != 0) {
              std::cout << "rgb->h265: h265 decode flush failed: ret = " << ret
                        << std::endl;
              return -7;
            }

            for (size_t k = 0; k < h265DecodeBufferPtr.size(); ++k) {
              const std::string decode_rgb_file_path = h265_file_paths[0];
              h265_file_paths.erase(h265_file_paths.begin());

              const std::string &h265_decode_file_path =
                  decode_rgb_file_path + ".decode.jpeg";
              cv::Mat rgbMat(config.imgHeight, config.imgWidth, CV_8UC3,
                             h265DecodeBufferPtr[k]->data());

              cv::Mat bgrMat;
              cv::cvtColor(rgbMat, bgrMat, cv::COLOR_RGB2BGR);
              if (bgrMat.empty()) {
                std::cout << "rgb->h265: RGB->BGR failed !" << std::endl;
                return -7;
              }

              bool isSuccess = cv::imwrite(h265_decode_file_path, bgrMat,
                                           {cv::IMWRITE_JPEG_QUALITY, 100});
              if (!isSuccess) {
                std::cout << "rgb->h265: write decode jpeg failed !"
                          << std::endl;
                return -8;
              }
            }
          }
        }

        std::cout << "rgb->h265: " << rgb_file_path << " successed !"
                  << std::endl;
      }
    }

    // nv12
    {
      robosense::h265::H265Coder h265Encoder;
      robosense::h265::H265Coder h265Decoder;

      robosense::h265::H265CodesConfig config;
      config.codeType = robosense::h265::H265_CODER_TYPE::RS_H265_CODER_ENCODE;
      config.imageFrameFormat =
          robosense::common::ImageFrameFormat::FRAME_FORMAT_NV12;
      config.imgWidth = imageWidth;
      config.imgHeight = imageHeight;
      config.imgFreq = 10;
      config.codesRateType =
          robosense::h265::H265_CODES_RATE_TYPE::RS_H265_CODES_CUSTOM;
      config.imgCodeRate = 4000000;

      auto config2 = config;
      config2.codeType = robosense::h265::H265_CODER_TYPE::RS_H265_CODER_DECODE;

      int ret = h265Encoder.init(config);
      if (ret != 0) {
        std::cout << "fisheye h265 encoder(nv12) initial failed: ret = " << ret
                  << std::endl;
        return -1;
      }

      ret = h265Decoder.init(config2);
      if (ret != 0) {
        std::cout << "fisheye h265 decoder(nv12) initial failed: ret = " << ret
                  << std::endl;
        return -2;
      }

      // 获取文件名称
      std::vector<std::string> nv12_names;
      {
        std::ifstream ifstr(nv12_name_file_path,
                            std::ios_base::in | std::ios_base::binary);
        if (!ifstr.is_open()) {
          std::cout << "nv12->h265: open nv12 name.txt to read failed: "
                    << nv12_name_file_path << std::endl;
          return -3;
        }

        std::string content;
        while (std::getline(ifstr, content)) {
          if (content.empty()) {
            continue;
          }
          nv12_names.push_back(content);
        }
      }

      std::sort(nv12_names.begin(), nv12_names.end());

      std::vector<std::string> h265_file_paths;
      for (size_t i = 0; i < nv12_names.size(); ++i) {
        const std::string &yuv420p_file_path =
            nv12_dir_path + "/" + nv12_names[i];

        std::vector<unsigned char> yuv420p_buffer(yuv420p_image_size, '\0');
        std::ifstream ifstr(yuv420p_file_path,
                            std::ios_base::in | std::ios_base::binary);
        if (!ifstr.is_open()) {
          std::cout << "nv12->h265: open nv12 file to read failed: "
                    << yuv420p_file_path << std::endl;
          return -4;
        }

        if (!ifstr.read(reinterpret_cast<char *>(yuv420p_buffer.data()),
                        yuv420p_image_size)) {
          std::cout << "nv12->h265: read yuv420p data failed: "
                    << yuv420p_file_path << std::endl;
          return -5;
        }

        robosense::h265::H265Coder::CODER_BUFFER_PTR_VEC h265EncodeBufferPtr;
        std::vector<bool> iFrameFlags;
        int ret = h265Encoder.encode(yuv420p_buffer.data(), yuv420p_image_size,
                                     h265EncodeBufferPtr, iFrameFlags);
        if (ret != 0) {
          std::cout << "nv12->h265: h265 encode failed: ret = " << ret
                    << std::endl;
          return -6;
        }

        h265_file_paths.push_back(yuv420p_file_path);
        if (h265EncodeBufferPtr.empty()) {
          std::cout << "nv12->h265: " << yuv420p_file_path << " successed !"
                    << std::endl;
          continue;
        }

        for (size_t j = 0; j < h265EncodeBufferPtr.size(); ++j) {
          robosense::h265::H265Coder::CODER_BUFFER_PTR singleBufferPtr =
              h265EncodeBufferPtr[j];

          robosense::h265::H265Coder::CODER_BUFFER_PTR_VEC h265DecodeBufferPtr;
          ret =
              h265Decoder.decode(singleBufferPtr->data(),
                                 singleBufferPtr->size(), h265DecodeBufferPtr);
          if (ret != 0) {
            std::cout << "nv12->h265: h265 decode failed: ret = " << ret
                      << std::endl;
            return -7;
          }

          for (size_t k = 0; k < h265DecodeBufferPtr.size(); ++k) {
            const std::string decode_yuv420p_file_path = h265_file_paths[0];
            h265_file_paths.erase(h265_file_paths.begin());

            const std::string &h265_decode_file_path =
                decode_yuv420p_file_path + ".decode.jpeg";
            cv::Mat yuv420pMat(config.imgHeight * 3 / 2, config.imgWidth,
                               CV_8UC1, h265DecodeBufferPtr[k]->data());

            cv::Mat bgrMat;
            cv::cvtColor(yuv420pMat, bgrMat, cv::COLOR_YUV2BGR_NV12);
            if (bgrMat.empty()) {
              std::cout << "nv12->h265: YUV420P->BGR failed !" << std::endl;
              return -7;
            }

            bool isSuccess = cv::imwrite(h265_decode_file_path, bgrMat,
                                         {cv::IMWRITE_JPEG_QUALITY, 100});
            if (!isSuccess) {
              std::cout << "nv12->h265: write decode jpeg failed !"
                        << std::endl;
              return -8;
            }
          }
        }

        if (i == nv12_names.size() - 1) {
          robosense::h265::H265Coder::CODER_BUFFER_PTR_VEC h265EncodeBufferPtr;
          std::vector<bool> iFrameFlags;
          int ret = h265Encoder.encodeFlush(h265EncodeBufferPtr, iFrameFlags);
          if (ret != 0) {
            std::cout << "nv12->h265: h265 encode flush failed: ret = " << ret
                      << std::endl;
            return -1;
          }

          for (size_t j = 0; j < h265EncodeBufferPtr.size(); ++j) {
            robosense::h265::H265Coder::CODER_BUFFER_PTR singleBufferPtr =
                h265EncodeBufferPtr[j];

            robosense::h265::H265Coder::CODER_BUFFER_PTR_VEC
                h265DecodeBufferPtr;
            ret = h265Decoder.decode(singleBufferPtr->data(),
                                     singleBufferPtr->size(),
                                     h265DecodeBufferPtr);
            if (ret != 0) {
              std::cout << "nv12->h265: h265 decode failed: ret = " << ret
                        << std::endl;
              return -7;
            }

            for (size_t k = 0; k < h265DecodeBufferPtr.size(); ++k) {
              const std::string decode_yuv420p_file_path = h265_file_paths[0];
              h265_file_paths.erase(h265_file_paths.begin());

              const std::string &h265_decode_file_path =
                  decode_yuv420p_file_path + ".decode.jpeg";
              cv::Mat yuv420pMat(config.imgHeight * 3 / 2, config.imgWidth,
                                 CV_8UC1, h265DecodeBufferPtr[k]->data());

              cv::Mat bgrMat;
              cv::cvtColor(yuv420pMat, bgrMat, cv::COLOR_YUV2BGR_NV12);
              if (bgrMat.empty()) {
                std::cout << "nv12->h265: YUV420P->BGR failed !" << std::endl;
                return -7;
              }

              bool isSuccess = cv::imwrite(h265_decode_file_path, bgrMat,
                                           {cv::IMWRITE_JPEG_QUALITY, 100});
              if (!isSuccess) {
                std::cout << "nv12->h265: write decode jpeg failed !"
                          << std::endl;
                return -8;
              }
            }
          }

          {
            robosense::h265::H265Coder::CODER_BUFFER_PTR_VEC
                h265DecodeBufferPtr;
            ret = h265Decoder.decodeFlush(h265DecodeBufferPtr);
            if (ret != 0) {
              std::cout << "nv12->h265: h265 decode flush failed: ret = " << ret
                        << std::endl;
              return -7;
            }

            for (size_t k = 0; k < h265DecodeBufferPtr.size(); ++k) {
              const std::string decode_yuv420p_file_path = h265_file_paths[0];
              h265_file_paths.erase(h265_file_paths.begin());

              const std::string &h265_decode_file_path =
                  decode_yuv420p_file_path + ".decode.jpeg";
              cv::Mat yuv420pMat(config.imgHeight * 3 / 2, config.imgWidth,
                                 CV_8UC1, h265DecodeBufferPtr[k]->data());

              cv::Mat bgrMat;
              cv::cvtColor(yuv420pMat, bgrMat, cv::COLOR_YUV2BGR_NV12);
              if (bgrMat.empty()) {
                std::cout << "nv12->h265: YUV420P->BGR failed !" << std::endl;
                return -7;
              }

              bool isSuccess = cv::imwrite(h265_decode_file_path, bgrMat,
                                           {cv::IMWRITE_JPEG_QUALITY, 100});
              if (!isSuccess) {
                std::cout << "nv12->h265: write decode jpeg failed !"
                          << std::endl;
                return -8;
              }
            }
          }
        }

        std::cout << "nv12->h265: " << yuv420p_file_path << " successed !"
                  << std::endl;
      }
    }

    // yuv420p
    {
      robosense::h265::H265Coder h265Encoder;
      robosense::h265::H265Coder h265Decoder;

      robosense::h265::H265CodesConfig config;
      config.codeType = robosense::h265::H265_CODER_TYPE::RS_H265_CODER_ENCODE;
      config.imageFrameFormat =
          robosense::common::ImageFrameFormat::FRAME_FORMAT_YUV420P;
      config.imgWidth = imageWidth;
      config.imgHeight = imageHeight;
      config.imgFreq = 10;
      config.codesRateType =
          robosense::h265::H265_CODES_RATE_TYPE::RS_H265_CODES_CUSTOM;
      config.imgCodeRate = 4000000;

      auto config2 = config;
      config2.codeType = robosense::h265::H265_CODER_TYPE::RS_H265_CODER_DECODE;

      int ret = h265Encoder.init(config);
      if (ret != 0) {
        std::cout << "fisheye h265 encoder(yuv420p) initial failed: ret = "
                  << ret << std::endl;
        return -1;
      }

      ret = h265Decoder.init(config2);
      if (ret != 0) {
        std::cout << "fisheye h265 decoder(yuv420p) initial failed: ret = "
                  << ret << std::endl;
        return -2;
      }

      // 获取文件名称
      std::vector<std::string> yuv420p_names;
      {
        std::ifstream ifstr(yuv420p_name_file_path,
                            std::ios_base::in | std::ios_base::binary);
        if (!ifstr.is_open()) {
          std::cout << "yuv420p->h265: open yuv420p name.txt to read failed: "
                    << yuv420p_name_file_path << std::endl;
          return -3;
        }

        std::string content;
        while (std::getline(ifstr, content)) {
          if (content.empty()) {
            continue;
          }
          yuv420p_names.push_back(content);
        }
      }

      std::sort(yuv420p_names.begin(), yuv420p_names.end());

      std::vector<std::string> h265_file_paths;
      for (size_t i = 0; i < yuv420p_names.size(); ++i) {
        const std::string &yuv420p_file_path =
            yuv420p_dir_path + "/" + yuv420p_names[i];

        std::vector<unsigned char> yuv420p_buffer(yuv420p_image_size, '\0');
        std::ifstream ifstr(yuv420p_file_path,
                            std::ios_base::in | std::ios_base::binary);
        if (!ifstr.is_open()) {
          std::cout << "yuv420p->h265: open yuv420p file to read failed: "
                    << yuv420p_file_path << std::endl;
          return -4;
        }

        if (!ifstr.read(reinterpret_cast<char *>(yuv420p_buffer.data()),
                        yuv420p_image_size)) {
          std::cout << "yuv420p->h265: read yuv420p data failed: "
                    << yuv420p_file_path << std::endl;
          return -5;
        }

        robosense::h265::H265Coder::CODER_BUFFER_PTR_VEC h265EncodeBufferPtr;
        std::vector<bool> iFrameFlags;
        int ret = h265Encoder.encode(yuv420p_buffer.data(), yuv420p_image_size,
                                     h265EncodeBufferPtr, iFrameFlags);
        if (ret != 0) {
          std::cout << "yuv420p->h265: h265 encode failed: ret = " << ret
                    << std::endl;
          return -6;
        }

        h265_file_paths.push_back(yuv420p_file_path);
        if (h265EncodeBufferPtr.empty()) {
          std::cout << "yuv420p->h265: " << yuv420p_file_path << " successed !"
                    << std::endl;
          continue;
        }

        for (size_t j = 0; j < h265EncodeBufferPtr.size(); ++j) {
          robosense::h265::H265Coder::CODER_BUFFER_PTR singleBufferPtr =
              h265EncodeBufferPtr[j];

          robosense::h265::H265Coder::CODER_BUFFER_PTR_VEC h265DecodeBufferPtr;
          ret =
              h265Decoder.decode(singleBufferPtr->data(),
                                 singleBufferPtr->size(), h265DecodeBufferPtr);
          if (ret != 0) {
            std::cout << "yuv420p->h265: h265 decode failed: ret = " << ret
                      << std::endl;
            return -7;
          }

          for (size_t k = 0; k < h265DecodeBufferPtr.size(); ++k) {
            const std::string decode_yuv420p_file_path = h265_file_paths[0];
            h265_file_paths.erase(h265_file_paths.begin());

            const std::string &h265_decode_file_path =
                decode_yuv420p_file_path + ".decode.jpeg";
            cv::Mat yuv420pMat(config.imgHeight * 3 / 2, config.imgWidth,
                               CV_8UC1, h265DecodeBufferPtr[k]->data());

            cv::Mat bgrMat;
            cv::cvtColor(yuv420pMat, bgrMat, cv::COLOR_YUV2BGR_I420);
            if (bgrMat.empty()) {
              std::cout << "yuv420p->h265: YUV420P->BGR failed !" << std::endl;
              return -7;
            }

            bool isSuccess = cv::imwrite(h265_decode_file_path, bgrMat,
                                         {cv::IMWRITE_JPEG_QUALITY, 100});
            if (!isSuccess) {
              std::cout << "yuv420p->h265: write decode jpeg failed !"
                        << std::endl;
              return -8;
            }
          }
        }

        if (i == yuv420p_names.size() - 1) {
          robosense::h265::H265Coder::CODER_BUFFER_PTR_VEC h265EncodeBufferPtr;
          std::vector<bool> iFrameFlags;
          int ret = h265Encoder.encodeFlush(h265EncodeBufferPtr, iFrameFlags);
          if (ret != 0) {
            std::cout << "yuv420p->h265: h265 encode flush failed: ret = "
                      << ret << std::endl;
            return -1;
          }

          for (size_t j = 0; j < h265EncodeBufferPtr.size(); ++j) {
            robosense::h265::H265Coder::CODER_BUFFER_PTR singleBufferPtr =
                h265EncodeBufferPtr[j];

            robosense::h265::H265Coder::CODER_BUFFER_PTR_VEC
                h265DecodeBufferPtr;
            ret = h265Decoder.decode(singleBufferPtr->data(),
                                     singleBufferPtr->size(),
                                     h265DecodeBufferPtr);
            if (ret != 0) {
              std::cout << "yuv420p->h265: h265 decode failed: ret = " << ret
                        << std::endl;
              return -7;
            }

            for (size_t k = 0; k < h265DecodeBufferPtr.size(); ++k) {
              const std::string decode_yuv420p_file_path = h265_file_paths[0];
              h265_file_paths.erase(h265_file_paths.begin());

              const std::string &h265_decode_file_path =
                  decode_yuv420p_file_path + ".decode.jpeg";
              cv::Mat yuv420pMat(config.imgHeight * 3 / 2, config.imgWidth,
                                 CV_8UC1, h265DecodeBufferPtr[k]->data());

              cv::Mat bgrMat;
              cv::cvtColor(yuv420pMat, bgrMat, cv::COLOR_YUV2BGR_I420);
              if (bgrMat.empty()) {
                std::cout << "yuv420p->h265: YUV420P->BGR failed !"
                          << std::endl;
                return -7;
              }

              bool isSuccess = cv::imwrite(h265_decode_file_path, bgrMat,
                                           {cv::IMWRITE_JPEG_QUALITY, 100});
              if (!isSuccess) {
                std::cout << "yuv420p->h265: write decode jpeg failed !"
                          << std::endl;
                return -8;
              }
            }
          }

          {
            robosense::h265::H265Coder::CODER_BUFFER_PTR_VEC
                h265DecodeBufferPtr;
            ret = h265Decoder.decodeFlush(h265DecodeBufferPtr);
            if (ret != 0) {
              std::cout << "yuv420p->h265: h265 decode flush failed: ret = "
                        << ret << std::endl;
              return -7;
            }

            for (size_t k = 0; k < h265DecodeBufferPtr.size(); ++k) {
              const std::string decode_yuv420p_file_path = h265_file_paths[0];
              h265_file_paths.erase(h265_file_paths.begin());

              const std::string &h265_decode_file_path =
                  decode_yuv420p_file_path + ".decode.jpeg";
              cv::Mat yuv420pMat(config.imgHeight * 3 / 2, config.imgWidth,
                                 CV_8UC1, h265DecodeBufferPtr[k]->data());

              cv::Mat bgrMat;
              cv::cvtColor(yuv420pMat, bgrMat, cv::COLOR_YUV2BGR_I420);
              if (bgrMat.empty()) {
                std::cout << "yuv420p->h265: YUV420P->BGR failed !"
                          << std::endl;
                return -7;
              }

              bool isSuccess = cv::imwrite(h265_decode_file_path, bgrMat,
                                           {cv::IMWRITE_JPEG_QUALITY, 100});
              if (!isSuccess) {
                std::cout << "yuv420p->h265: write decode jpeg failed !"
                          << std::endl;
                return -8;
              }
            }
          }
        }

        std::cout << "yuv420p->h265: " << yuv420p_file_path << " successed !"
                  << std::endl;
      }
    }

    // yuyv422
    {
      robosense::h265::H265Coder h265Encoder;
      robosense::h265::H265Coder h265Decoder;

      robosense::h265::H265CodesConfig config;
      config.codeType = robosense::h265::H265_CODER_TYPE::RS_H265_CODER_ENCODE;
      config.imageFrameFormat =
          robosense::common::ImageFrameFormat::FRAME_FORMAT_YUYV422;
      config.imgWidth = imageWidth;
      config.imgHeight = imageHeight;
      config.imgFreq = 10;
      config.codesRateType =
          robosense::h265::H265_CODES_RATE_TYPE::RS_H265_CODES_CUSTOM;
      config.imgCodeRate = 4000000;

      auto config2 = config;
      config2.codeType = robosense::h265::H265_CODER_TYPE::RS_H265_CODER_DECODE;

      int ret = h265Encoder.init(config);
      if (ret != 0) {
        std::cout << "fisheye h265 encoder(yuyv422) initial failed: ret = "
                  << ret << std::endl;
        return -1;
      }

      ret = h265Decoder.init(config2);
      if (ret != 0) {
        std::cout << "fisheye h265 decoder(yuyv422) initial failed: ret = "
                  << ret << std::endl;
        return -2;
      }

      // 获取文件名称
      std::vector<std::string> yuyv422_names;
      {
        std::ifstream ifstr(yuyv422_name_file_path,
                            std::ios_base::in | std::ios_base::binary);
        if (!ifstr.is_open()) {
          std::cout << "yuyv422->h265: open yuyv422 name.txt to read failed:"
                    << yuyv422_name_file_path << std::endl;
          return -3;
        }

        std::string content;
        while (std::getline(ifstr, content)) {
          if (content.empty()) {
            continue;
          }
          yuyv422_names.push_back(content);
        }
      }

      std::sort(yuyv422_names.begin(), yuyv422_names.end());

      std::vector<std::string> h265_file_paths;
      for (size_t i = 0; i < yuyv422_names.size(); ++i) {
        const std::string &yuyv422_file_path =
            yuyv422_dir_path + "/" + yuyv422_names[i];

        std::vector<unsigned char> yuyv422_buffer(yuyv422_image_size, '\0');
        std::ifstream ifstr(yuyv422_file_path,
                            std::ios_base::in | std::ios_base::binary);
        if (!ifstr.is_open()) {
          std::cout << "yuyv422->h265: open yuyv422 file to read failed: "
                    << yuyv422_file_path << std::endl;
          return -4;
        }

        if (!ifstr.read(reinterpret_cast<char *>(yuyv422_buffer.data()),
                        yuyv422_image_size)) {
          std::cout << "yuyv422->h265: read yuyv422 data failed: "
                    << yuyv422_file_path << std::endl;
          return -5;
        }

        robosense::h265::H265Coder::CODER_BUFFER_PTR_VEC h265EncodeBufferPtr;
        std::vector<bool> iFrameFlags;
        int ret = h265Encoder.encode(yuyv422_buffer.data(), yuyv422_image_size,
                                     h265EncodeBufferPtr, iFrameFlags);
        if (ret != 0) {
          std::cout << "yuyv422->h265: h265 encode failed: ret = " << ret
                    << std::endl;
          return -6;
        }

        h265_file_paths.push_back(yuyv422_file_path);
        if (h265EncodeBufferPtr.empty()) {
          std::cout << "yuyv422->h265: " << yuyv422_file_path << " successed !"
                    << std::endl;
          continue;
        }

        for (size_t j = 0; j < h265EncodeBufferPtr.size(); ++j) {
          robosense::h265::H265Coder::CODER_BUFFER_PTR singleBufferPtr =
              h265EncodeBufferPtr[j];

          robosense::h265::H265Coder::CODER_BUFFER_PTR_VEC h265DecodeBufferPtr;
          ret =
              h265Decoder.decode(singleBufferPtr->data(),
                                 singleBufferPtr->size(), h265DecodeBufferPtr);
          if (ret != 0) {
            std::cout << "yuyv422->h265: h265 decode failed: ret = " << ret
                      << std::endl;
            return -7;
          }

          for (size_t k = 0; k < h265DecodeBufferPtr.size(); ++k) {
            const std::string decode_yuv420p_file_path = h265_file_paths[0];
            h265_file_paths.erase(h265_file_paths.begin());

            const std::string &h265_decode_file_path =
                decode_yuv420p_file_path + ".decode.jpeg";
            cv::Mat yuyv422Mat(config.imgHeight, config.imgWidth, CV_8UC2,
                               h265DecodeBufferPtr[k]->data());

            cv::Mat bgrMat;
            cv::cvtColor(yuyv422Mat, bgrMat, cv::COLOR_YUV2BGR_YUYV);
            if (bgrMat.empty()) {
              std::cout << "yuyv422->h265: YUV420P->BGR failed !" << std::endl;
              return -7;
            }

            bool isSuccess = cv::imwrite(h265_decode_file_path, bgrMat,
                                         {cv::IMWRITE_JPEG_QUALITY, 100});
            if (!isSuccess) {
              std::cout << "yuyv422->h265: write decode jpeg failed !"
                        << std::endl;
              return -8;
            }
          }
        }

        if (i == yuyv422_names.size() - 1) {
          robosense::h265::H265Coder::CODER_BUFFER_PTR_VEC h265EncodeBufferPtr;
          std::vector<bool> iFrameFlags;
          int ret = h265Encoder.encodeFlush(h265EncodeBufferPtr, iFrameFlags);
          if (ret != 0) {
            std::cout << "yuyv422->h265: h265 encode flush failed: ret = "
                      << ret << std::endl;
            return -1;
          }

          for (size_t j = 0; j < h265EncodeBufferPtr.size(); ++j) {
            robosense::h265::H265Coder::CODER_BUFFER_PTR singleBufferPtr =
                h265EncodeBufferPtr[j];

            robosense::h265::H265Coder::CODER_BUFFER_PTR_VEC
                h265DecodeBufferPtr;
            ret = h265Decoder.decode(singleBufferPtr->data(),
                                     singleBufferPtr->size(),
                                     h265DecodeBufferPtr);
            if (ret != 0) {
              std::cout << "yuyv422->h265: h265 decode failed: ret = " << ret
                        << std::endl;
              return -7;
            }

            for (size_t k = 0; k < h265DecodeBufferPtr.size(); ++k) {
              const std::string decode_yuv420p_file_path = h265_file_paths[0];
              h265_file_paths.erase(h265_file_paths.begin());

              const std::string &h265_decode_file_path =
                  decode_yuv420p_file_path + ".decode.jpeg";
              cv::Mat yuyv422Mat(config.imgHeight, config.imgWidth, CV_8UC2,
                                 h265DecodeBufferPtr[k]->data());

              cv::Mat bgrMat;
              cv::cvtColor(yuyv422Mat, bgrMat, cv::COLOR_YUV2BGR_YUYV);
              if (bgrMat.empty()) {
                std::cout << "yuyv422->h265: YUV420P->BGR failed !"
                          << std::endl;
                return -7;
              }

              bool isSuccess = cv::imwrite(h265_decode_file_path, bgrMat,
                                           {cv::IMWRITE_JPEG_QUALITY, 100});
              if (!isSuccess) {
                std::cout << "yuyv422->h265: write decode jpeg failed !"
                          << std::endl;
                return -8;
              }
            }
          }

          {
            robosense::h265::H265Coder::CODER_BUFFER_PTR_VEC
                h265DecodeBufferPtr;
            ret = h265Decoder.decodeFlush(h265DecodeBufferPtr);
            if (ret != 0) {
              std::cout << "yuyv422->h265: h265 decode flush failed: ret = "
                        << ret << std::endl;
              return -7;
            }

            for (size_t k = 0; k < h265DecodeBufferPtr.size(); ++k) {
              const std::string decode_yuv420p_file_path = h265_file_paths[0];
              h265_file_paths.erase(h265_file_paths.begin());

              const std::string &h265_decode_file_path =
                  decode_yuv420p_file_path + ".decode.jpeg";
              cv::Mat yuv420pMat(config.imgHeight * 3 / 2, config.imgWidth,
                                 CV_8UC1, h265DecodeBufferPtr[k]->data());

              cv::Mat bgrMat;
              cv::cvtColor(yuv420pMat, bgrMat, cv::COLOR_YUV2BGR_I420);
              if (bgrMat.empty()) {
                std::cout << "yuyv422->h265: YUV420P->BGR failed !"
                          << std::endl;
                return -7;
              }

              bool isSuccess = cv::imwrite(h265_decode_file_path, bgrMat,
                                           {cv::IMWRITE_JPEG_QUALITY, 100});
              if (!isSuccess) {
                std::cout << "yuyv422->h265: write decode jpeg failed !"
                          << std::endl;
                return -8;
              }
            }
          }
        }

        std::cout << "yuyv422->h265: " << yuyv422_file_path << " successed !"
                  << std::endl;
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

    const std::string &fisheye_dir_path =
        "/media/sti/ee28dfb8-da74-4739-88ab-b2b836e8b78c/super_sensor_sdk_v1/"
        "test/h265_test/pinhole";

    const std::string &nv12_dir_path = fisheye_dir_path + "/nv12/";
    const std::string &nv12_name_file_path = nv12_dir_path + "/name.txt";

    const std::string &rgb_dir_path = fisheye_dir_path + "/rgb/";
    const std::string &rgb_name_file_path = rgb_dir_path + "/name.txt";

    const std::string &yuv420p_dir_path = fisheye_dir_path + "/yuv420p/";
    const std::string &yuv420p_name_file_path = yuv420p_dir_path + "/name.txt";

    const std::string &yuyv422_dir_path = fisheye_dir_path + "/yuyv422/";
    const std::string &yuyv422_name_file_path = yuyv422_dir_path + "/name.txt";

    // rgb
    {
      robosense::h265::H265Coder h265Encoder;
      robosense::h265::H265Coder h265Decoder;

      robosense::h265::H265CodesConfig config;
      config.codeType = robosense::h265::H265_CODER_TYPE::RS_H265_CODER_ENCODE;
      config.imageFrameFormat =
          robosense::common::ImageFrameFormat::FRAME_FORMAT_RGB24;
      config.imgWidth = imageWidth;
      config.imgHeight = imageHeight;
      config.imgFreq = 10;
      config.codesRateType =
          robosense::h265::H265_CODES_RATE_TYPE::RS_H265_CODES_CUSTOM;
      config.imgCodeRate = 4000000;

      auto config2 = config;
      config2.codeType = robosense::h265::H265_CODER_TYPE::RS_H265_CODER_DECODE;

      int ret = h265Encoder.init(config);
      if (ret != 0) {
        std::cout << "fisheye h265 encoder(rgb) initial failed: ret = " << ret
                  << std::endl;
        return -1;
      }

      ret = h265Decoder.init(config2);
      if (ret != 0) {
        std::cout << "fisheye h265 decoder(rgb) initial failed: ret = " << ret
                  << std::endl;
        return -2;
      }

      // 获取文件名称
      std::vector<std::string> rgb_names;
      {
        std::ifstream ifstr(rgb_name_file_path,
                            std::ios_base::in | std::ios_base::binary);
        if (!ifstr.is_open()) {
          std::cout << "rgb->h265: open rgb name.txt to read failed: "
                    << rgb_name_file_path << std::endl;
          return -3;
        }

        std::string content;
        while (std::getline(ifstr, content)) {
          if (content.empty()) {
            continue;
          }
          rgb_names.push_back(content);
        }
      }

      std::sort(rgb_names.begin(), rgb_names.end());

      std::vector<std::string> h265_file_paths;
      for (size_t i = 0; i < rgb_names.size(); ++i) {
        const std::string &rgb_file_path = rgb_dir_path + "/" + rgb_names[i];
        std::vector<unsigned char> rgb_buffer(rgb_image_size, '\0');
        std::ifstream ifstr(rgb_file_path,
                            std::ios_base::in | std::ios_base::binary);
        if (!ifstr.is_open()) {
          std::cout << "rgb->h265: open rgb file to read failed: "
                    << rgb_file_path << std::endl;
          return -4;
        }

        if (!ifstr.read(reinterpret_cast<char *>(rgb_buffer.data()),
                        rgb_image_size)) {
          std::cout << "rgb->h265: read rgb data failed: " << rgb_file_path
                    << std::endl;
          return -5;
        }

        robosense::h265::H265Coder::CODER_BUFFER_PTR_VEC h265EncodeBufferPtr;
        std::vector<bool> iFrameFlags;
        int ret = h265Encoder.encode(rgb_buffer.data(), rgb_image_size,
                                     h265EncodeBufferPtr, iFrameFlags);
        if (ret != 0) {
          std::cout << "rgb->h265: h265 encode failed: ret = " << ret
                    << std::endl;
          return -6;
        }

        h265_file_paths.push_back(rgb_file_path);
        if (h265EncodeBufferPtr.empty()) {
          std::cout << "rgb->h265: " << rgb_file_path << " successed !"
                    << std::endl;
          continue;
        }

        for (size_t j = 0; j < h265EncodeBufferPtr.size(); ++j) {
          robosense::h265::H265Coder::CODER_BUFFER_PTR singleBufferPtr =
              h265EncodeBufferPtr[j];

          robosense::h265::H265Coder::CODER_BUFFER_PTR_VEC h265DecodeBufferPtr;
          ret =
              h265Decoder.decode(singleBufferPtr->data(),
                                 singleBufferPtr->size(), h265DecodeBufferPtr);
          if (ret != 0) {
            std::cout << "rgb->h265: h265 decode failed: ret = " << ret
                      << std::endl;
            return -7;
          }

          for (size_t k = 0; k < h265DecodeBufferPtr.size(); ++k) {
            const std::string decode_rgb_file_path = h265_file_paths[0];
            h265_file_paths.erase(h265_file_paths.begin());

            const std::string &h265_decode_file_path =
                decode_rgb_file_path + ".decode.jpeg";
            cv::Mat rgbMat(config.imgHeight, config.imgWidth, CV_8UC3,
                           h265DecodeBufferPtr[k]->data());

            cv::Mat bgrMat;
            cv::cvtColor(rgbMat, bgrMat, cv::COLOR_RGB2BGR);
            if (bgrMat.empty()) {
              std::cout << "rgb->h265: RGB->BGR failed !" << std::endl;
              return -7;
            }

            bool isSuccess = cv::imwrite(h265_decode_file_path, bgrMat,
                                         {cv::IMWRITE_JPEG_QUALITY, 100});
            if (!isSuccess) {
              std::cout << "rgb->h265: write decode jpeg failed !" << std::endl;
              return -8;
            }
          }
        }

        if (i == rgb_names.size() - 1) {
          robosense::h265::H265Coder::CODER_BUFFER_PTR_VEC h265EncodeBufferPtr;
          std::vector<bool> iFrameFlags;
          int ret = h265Encoder.encodeFlush(h265EncodeBufferPtr, iFrameFlags);
          if (ret != 0) {
            std::cout << "rgb->h265: h265 encode flush failed: ret = " << ret
                      << std::endl;
            return -1;
          }

          for (size_t j = 0; j < h265EncodeBufferPtr.size(); ++j) {
            robosense::h265::H265Coder::CODER_BUFFER_PTR singleBufferPtr =
                h265EncodeBufferPtr[j];

            robosense::h265::H265Coder::CODER_BUFFER_PTR_VEC
                h265DecodeBufferPtr;
            ret = h265Decoder.decode(singleBufferPtr->data(),
                                     singleBufferPtr->size(),
                                     h265DecodeBufferPtr);
            if (ret != 0) {
              std::cout << "rgb->h265: h265 decode failed: ret = " << ret
                        << std::endl;
              return -7;
            }

            for (size_t k = 0; k < h265DecodeBufferPtr.size(); ++k) {
              const std::string decode_rgb_file_path = h265_file_paths[0];
              h265_file_paths.erase(h265_file_paths.begin());

              const std::string &h265_decode_file_path =
                  decode_rgb_file_path + ".decode.jpeg";
              cv::Mat rgbMat(config.imgHeight, config.imgWidth, CV_8UC3,
                             h265DecodeBufferPtr[k]->data());

              cv::Mat bgrMat;
              cv::cvtColor(rgbMat, bgrMat, cv::COLOR_RGB2BGR);
              if (bgrMat.empty()) {
                std::cout << "rgb->h265: RGB->BGR failed !" << std::endl;
                return -7;
              }

              bool isSuccess = cv::imwrite(h265_decode_file_path, bgrMat,
                                           {cv::IMWRITE_JPEG_QUALITY, 100});
              if (!isSuccess) {
                std::cout << "rgb->h265: write decode jpeg failed !"
                          << std::endl;
                return -8;
              }
            }
          }

          {
            robosense::h265::H265Coder::CODER_BUFFER_PTR_VEC
                h265DecodeBufferPtr;
            ret = h265Decoder.decodeFlush(h265DecodeBufferPtr);
            if (ret != 0) {
              std::cout << "rgb->h265: h265 decode flush failed: ret = " << ret
                        << std::endl;
              return -7;
            }

            for (size_t k = 0; k < h265DecodeBufferPtr.size(); ++k) {
              const std::string decode_rgb_file_path = h265_file_paths[0];
              h265_file_paths.erase(h265_file_paths.begin());

              const std::string &h265_decode_file_path =
                  decode_rgb_file_path + ".decode.jpeg";
              cv::Mat rgbMat(config.imgHeight, config.imgWidth, CV_8UC3,
                             h265DecodeBufferPtr[k]->data());

              cv::Mat bgrMat;
              cv::cvtColor(rgbMat, bgrMat, cv::COLOR_RGB2BGR);
              if (bgrMat.empty()) {
                std::cout << "rgb->h265: RGB->BGR failed !" << std::endl;
                return -7;
              }

              bool isSuccess = cv::imwrite(h265_decode_file_path, bgrMat,
                                           {cv::IMWRITE_JPEG_QUALITY, 100});
              if (!isSuccess) {
                std::cout << "rgb->h265: write decode jpeg failed !"
                          << std::endl;
                return -8;
              }
            }
          }
        }

        std::cout << "rgb->h265: " << rgb_file_path << " successed !"
                  << std::endl;
      }
    }

    // nv12
    {
      robosense::h265::H265Coder h265Encoder;
      robosense::h265::H265Coder h265Decoder;

      robosense::h265::H265CodesConfig config;
      config.codeType = robosense::h265::H265_CODER_TYPE::RS_H265_CODER_ENCODE;
      config.imageFrameFormat =
          robosense::common::ImageFrameFormat::FRAME_FORMAT_NV12;
      config.imgWidth = imageWidth;
      config.imgHeight = imageHeight;
      config.imgFreq = 10;
      config.codesRateType =
          robosense::h265::H265_CODES_RATE_TYPE::RS_H265_CODES_CUSTOM;
      config.imgCodeRate = 4000000;

      auto config2 = config;
      config2.codeType = robosense::h265::H265_CODER_TYPE::RS_H265_CODER_DECODE;

      int ret = h265Encoder.init(config);
      if (ret != 0) {
        std::cout << "fisheye h265 encoder(nv12) initial failed: ret = " << ret
                  << std::endl;
        return -1;
      }

      ret = h265Decoder.init(config2);
      if (ret != 0) {
        std::cout << "fisheye h265 decoder(nv12) initial failed: ret = " << ret
                  << std::endl;
        return -2;
      }

      // 获取文件名称
      std::vector<std::string> nv12_names;
      {
        std::ifstream ifstr(nv12_name_file_path,
                            std::ios_base::in | std::ios_base::binary);
        if (!ifstr.is_open()) {
          std::cout << "nv12->h265: open nv12 name.txt to read failed: "
                    << nv12_name_file_path << std::endl;
          return -3;
        }

        std::string content;
        while (std::getline(ifstr, content)) {
          if (content.empty()) {
            continue;
          }
          nv12_names.push_back(content);
        }
      }

      std::sort(nv12_names.begin(), nv12_names.end());

      std::vector<std::string> h265_file_paths;
      for (size_t i = 0; i < nv12_names.size(); ++i) {
        const std::string &yuv420p_file_path =
            nv12_dir_path + "/" + nv12_names[i];

        std::vector<unsigned char> yuv420p_buffer(yuv420p_image_size, '\0');
        std::ifstream ifstr(yuv420p_file_path,
                            std::ios_base::in | std::ios_base::binary);
        if (!ifstr.is_open()) {
          std::cout << "nv12->h265: open nv12 file to read failed: "
                    << yuv420p_file_path << std::endl;
          return -4;
        }

        if (!ifstr.read(reinterpret_cast<char *>(yuv420p_buffer.data()),
                        yuv420p_image_size)) {
          std::cout << "nv12->h265: read yuv420p data failed: "
                    << yuv420p_file_path << std::endl;
          return -5;
        }

        robosense::h265::H265Coder::CODER_BUFFER_PTR_VEC h265EncodeBufferPtr;
        std::vector<bool> iFrameFlags;
        int ret = h265Encoder.encode(yuv420p_buffer.data(), yuv420p_image_size,
                                     h265EncodeBufferPtr, iFrameFlags);
        if (ret != 0) {
          std::cout << "nv12->h265: h265 encode failed: ret = " << ret
                    << std::endl;
          return -6;
        }

        h265_file_paths.push_back(yuv420p_file_path);
        if (h265EncodeBufferPtr.empty()) {
          std::cout << "nv12->h265: " << yuv420p_file_path << " successed !"
                    << std::endl;
          continue;
        }

        for (size_t j = 0; j < h265EncodeBufferPtr.size(); ++j) {
          robosense::h265::H265Coder::CODER_BUFFER_PTR singleBufferPtr =
              h265EncodeBufferPtr[j];

          robosense::h265::H265Coder::CODER_BUFFER_PTR_VEC h265DecodeBufferPtr;
          ret =
              h265Decoder.decode(singleBufferPtr->data(),
                                 singleBufferPtr->size(), h265DecodeBufferPtr);
          if (ret != 0) {
            std::cout << "nv12->h265: h265 decode failed: ret = " << ret
                      << std::endl;
            return -7;
          }

          for (size_t k = 0; k < h265DecodeBufferPtr.size(); ++k) {
            const std::string decode_yuv420p_file_path = h265_file_paths[0];
            h265_file_paths.erase(h265_file_paths.begin());

            const std::string &h265_decode_file_path =
                decode_yuv420p_file_path + ".decode.jpeg";
            cv::Mat yuv420pMat(config.imgHeight * 3 / 2, config.imgWidth,
                               CV_8UC1, h265DecodeBufferPtr[k]->data());

            cv::Mat bgrMat;
            cv::cvtColor(yuv420pMat, bgrMat, cv::COLOR_YUV2BGR_NV12);
            if (bgrMat.empty()) {
              std::cout << "nv12->h265: YUV420P->BGR failed !" << std::endl;
              return -7;
            }

            bool isSuccess = cv::imwrite(h265_decode_file_path, bgrMat,
                                         {cv::IMWRITE_JPEG_QUALITY, 100});
            if (!isSuccess) {
              std::cout << "nv12->h265: write decode jpeg failed !"
                        << std::endl;
              return -8;
            }
          }
        }

        if (i == nv12_names.size() - 1) {
          robosense::h265::H265Coder::CODER_BUFFER_PTR_VEC h265EncodeBufferPtr;
          std::vector<bool> iFrameFlags;
          int ret = h265Encoder.encodeFlush(h265EncodeBufferPtr, iFrameFlags);
          if (ret != 0) {
            std::cout << "nv12->h265: h265 encode flush failed: ret = " << ret
                      << std::endl;
            return -1;
          }

          for (size_t j = 0; j < h265EncodeBufferPtr.size(); ++j) {
            robosense::h265::H265Coder::CODER_BUFFER_PTR singleBufferPtr =
                h265EncodeBufferPtr[j];

            robosense::h265::H265Coder::CODER_BUFFER_PTR_VEC
                h265DecodeBufferPtr;
            ret = h265Decoder.decode(singleBufferPtr->data(),
                                     singleBufferPtr->size(),
                                     h265DecodeBufferPtr);
            if (ret != 0) {
              std::cout << "nv12->h265: h265 decode failed: ret = " << ret
                        << std::endl;
              return -7;
            }

            for (size_t k = 0; k < h265DecodeBufferPtr.size(); ++k) {
              const std::string decode_yuv420p_file_path = h265_file_paths[0];
              h265_file_paths.erase(h265_file_paths.begin());

              const std::string &h265_decode_file_path =
                  decode_yuv420p_file_path + ".decode.jpeg";
              cv::Mat yuv420pMat(config.imgHeight * 3 / 2, config.imgWidth,
                                 CV_8UC1, h265DecodeBufferPtr[k]->data());

              cv::Mat bgrMat;
              cv::cvtColor(yuv420pMat, bgrMat, cv::COLOR_YUV2BGR_NV12);
              if (bgrMat.empty()) {
                std::cout << "nv12->h265: YUV420P->BGR failed !" << std::endl;
                return -7;
              }

              bool isSuccess = cv::imwrite(h265_decode_file_path, bgrMat,
                                           {cv::IMWRITE_JPEG_QUALITY, 100});
              if (!isSuccess) {
                std::cout << "nv12->h265: write decode jpeg failed !"
                          << std::endl;
                return -8;
              }
            }
          }

          {
            robosense::h265::H265Coder::CODER_BUFFER_PTR_VEC
                h265DecodeBufferPtr;
            ret = h265Decoder.decodeFlush(h265DecodeBufferPtr);
            if (ret != 0) {
              std::cout << "nv12->h265: h265 decode flush failed: ret = " << ret
                        << std::endl;
              return -7;
            }

            for (size_t k = 0; k < h265DecodeBufferPtr.size(); ++k) {
              const std::string decode_yuv420p_file_path = h265_file_paths[0];
              h265_file_paths.erase(h265_file_paths.begin());

              const std::string &h265_decode_file_path =
                  decode_yuv420p_file_path + ".decode.jpeg";
              cv::Mat yuv420pMat(config.imgHeight * 3 / 2, config.imgWidth,
                                 CV_8UC1, h265DecodeBufferPtr[k]->data());

              cv::Mat bgrMat;
              cv::cvtColor(yuv420pMat, bgrMat, cv::COLOR_YUV2BGR_NV12);
              if (bgrMat.empty()) {
                std::cout << "nv12->h265: YUV420P->BGR failed !" << std::endl;
                return -7;
              }

              bool isSuccess = cv::imwrite(h265_decode_file_path, bgrMat,
                                           {cv::IMWRITE_JPEG_QUALITY, 100});
              if (!isSuccess) {
                std::cout << "nv12->h265: write decode jpeg failed !"
                          << std::endl;
                return -8;
              }
            }
          }
        }

        std::cout << "nv12->h265: " << yuv420p_file_path << " successed !"
                  << std::endl;
      }
    }

    // yuv420p
    {
      robosense::h265::H265Coder h265Encoder;
      robosense::h265::H265Coder h265Decoder;

      robosense::h265::H265CodesConfig config;
      config.codeType = robosense::h265::H265_CODER_TYPE::RS_H265_CODER_ENCODE;
      config.imageFrameFormat =
          robosense::common::ImageFrameFormat::FRAME_FORMAT_YUV420P;
      config.imgWidth = imageWidth;
      config.imgHeight = imageHeight;
      config.imgFreq = 10;
      config.codesRateType =
          robosense::h265::H265_CODES_RATE_TYPE::RS_H265_CODES_CUSTOM;
      config.imgCodeRate = 4000000;

      auto config2 = config;
      config2.codeType = robosense::h265::H265_CODER_TYPE::RS_H265_CODER_DECODE;

      int ret = h265Encoder.init(config);
      if (ret != 0) {
        std::cout << "fisheye h265 encoder(yuv420p) initial failed: ret = "
                  << ret << std::endl;
        return -1;
      }

      ret = h265Decoder.init(config2);
      if (ret != 0) {
        std::cout << "fisheye h265 decoder(yuv420p) initial failed: ret = "
                  << ret << std::endl;
        return -2;
      }

      // 获取文件名称
      std::vector<std::string> yuv420p_names;
      {
        std::ifstream ifstr(yuv420p_name_file_path,
                            std::ios_base::in | std::ios_base::binary);
        if (!ifstr.is_open()) {
          std::cout << "yuv420p->h265: open yuv420p name.txt to read failed: "
                    << yuv420p_name_file_path << std::endl;
          return -3;
        }

        std::string content;
        while (std::getline(ifstr, content)) {
          if (content.empty()) {
            continue;
          }
          yuv420p_names.push_back(content);
        }
      }

      std::sort(yuv420p_names.begin(), yuv420p_names.end());

      std::vector<std::string> h265_file_paths;
      for (size_t i = 0; i < yuv420p_names.size(); ++i) {
        const std::string &yuv420p_file_path =
            yuv420p_dir_path + "/" + yuv420p_names[i];

        std::vector<unsigned char> yuv420p_buffer(yuv420p_image_size, '\0');
        std::ifstream ifstr(yuv420p_file_path,
                            std::ios_base::in | std::ios_base::binary);
        if (!ifstr.is_open()) {
          std::cout << "yuv420p->h265: open yuv420p file to read failed: "
                    << yuv420p_file_path << std::endl;
          return -4;
        }

        if (!ifstr.read(reinterpret_cast<char *>(yuv420p_buffer.data()),
                        yuv420p_image_size)) {
          std::cout << "yuv420p->h265: read yuv420p data failed: "
                    << yuv420p_file_path << std::endl;
          return -5;
        }

        robosense::h265::H265Coder::CODER_BUFFER_PTR_VEC h265EncodeBufferPtr;
        std::vector<bool> iFrameFlags;
        int ret = h265Encoder.encode(yuv420p_buffer.data(), yuv420p_image_size,
                                     h265EncodeBufferPtr, iFrameFlags);
        if (ret != 0) {
          std::cout << "yuv420p->h265: h265 encode failed: ret = " << ret
                    << std::endl;
          return -6;
        }

        h265_file_paths.push_back(yuv420p_file_path);
        if (h265EncodeBufferPtr.empty()) {
          std::cout << "yuv420p->h265: " << yuv420p_file_path << " successed !"
                    << std::endl;
          continue;
        }

        for (size_t j = 0; j < h265EncodeBufferPtr.size(); ++j) {
          robosense::h265::H265Coder::CODER_BUFFER_PTR singleBufferPtr =
              h265EncodeBufferPtr[j];

          robosense::h265::H265Coder::CODER_BUFFER_PTR_VEC h265DecodeBufferPtr;
          ret =
              h265Decoder.decode(singleBufferPtr->data(),
                                 singleBufferPtr->size(), h265DecodeBufferPtr);
          if (ret != 0) {
            std::cout << "yuv420p->h265: h265 decode failed: ret = " << ret
                      << std::endl;
            return -7;
          }

          for (size_t k = 0; k < h265DecodeBufferPtr.size(); ++k) {
            const std::string decode_yuv420p_file_path = h265_file_paths[0];
            h265_file_paths.erase(h265_file_paths.begin());

            const std::string &h265_decode_file_path =
                decode_yuv420p_file_path + ".decode.jpeg";
            cv::Mat yuv420pMat(config.imgHeight * 3 / 2, config.imgWidth,
                               CV_8UC1, h265DecodeBufferPtr[k]->data());

            cv::Mat bgrMat;
            cv::cvtColor(yuv420pMat, bgrMat, cv::COLOR_YUV2BGR_I420);
            if (bgrMat.empty()) {
              std::cout << "yuv420p->h265: YUV420P->BGR failed !" << std::endl;
              return -7;
            }

            bool isSuccess = cv::imwrite(h265_decode_file_path, bgrMat,
                                         {cv::IMWRITE_JPEG_QUALITY, 100});
            if (!isSuccess) {
              std::cout << "yuv420p->h265: write decode jpeg failed !"
                        << std::endl;
              return -8;
            }
          }
        }

        if (i == yuv420p_names.size() - 1) {
          robosense::h265::H265Coder::CODER_BUFFER_PTR_VEC h265EncodeBufferPtr;
          std::vector<bool> iFrameFlags;
          int ret = h265Encoder.encodeFlush(h265EncodeBufferPtr, iFrameFlags);
          if (ret != 0) {
            std::cout << "yuv420p->h265: h265 encode flush failed: ret = "
                      << ret << std::endl;
            return -1;
          }

          for (size_t j = 0; j < h265EncodeBufferPtr.size(); ++j) {
            robosense::h265::H265Coder::CODER_BUFFER_PTR singleBufferPtr =
                h265EncodeBufferPtr[j];

            robosense::h265::H265Coder::CODER_BUFFER_PTR_VEC
                h265DecodeBufferPtr;
            ret = h265Decoder.decode(singleBufferPtr->data(),
                                     singleBufferPtr->size(),
                                     h265DecodeBufferPtr);
            if (ret != 0) {
              std::cout << "yuv420p->h265: h265 decode failed: ret = " << ret
                        << std::endl;
              return -7;
            }

            for (size_t k = 0; k < h265DecodeBufferPtr.size(); ++k) {
              const std::string decode_yuv420p_file_path = h265_file_paths[0];
              h265_file_paths.erase(h265_file_paths.begin());

              const std::string &h265_decode_file_path =
                  decode_yuv420p_file_path + ".decode.jpeg";
              cv::Mat yuv420pMat(config.imgHeight * 3 / 2, config.imgWidth,
                                 CV_8UC1, h265DecodeBufferPtr[k]->data());

              cv::Mat bgrMat;
              cv::cvtColor(yuv420pMat, bgrMat, cv::COLOR_YUV2BGR_I420);
              if (bgrMat.empty()) {
                std::cout << "yuv420p->h265: YUV420P->BGR failed !"
                          << std::endl;
                return -7;
              }

              bool isSuccess = cv::imwrite(h265_decode_file_path, bgrMat,
                                           {cv::IMWRITE_JPEG_QUALITY, 100});
              if (!isSuccess) {
                std::cout << "yuv420p->h265: write decode jpeg failed !"
                          << std::endl;
                return -8;
              }
            }
          }

          {
            robosense::h265::H265Coder::CODER_BUFFER_PTR_VEC
                h265DecodeBufferPtr;
            ret = h265Decoder.decodeFlush(h265DecodeBufferPtr);
            if (ret != 0) {
              std::cout << "yuv420p->h265: h265 decode flush failed: ret = "
                        << ret << std::endl;
              return -7;
            }

            for (size_t k = 0; k < h265DecodeBufferPtr.size(); ++k) {
              const std::string decode_yuv420p_file_path = h265_file_paths[0];
              h265_file_paths.erase(h265_file_paths.begin());

              const std::string &h265_decode_file_path =
                  decode_yuv420p_file_path + ".decode.jpeg";
              cv::Mat yuv420pMat(config.imgHeight * 3 / 2, config.imgWidth,
                                 CV_8UC1, h265DecodeBufferPtr[k]->data());

              cv::Mat bgrMat;
              cv::cvtColor(yuv420pMat, bgrMat, cv::COLOR_YUV2BGR_I420);
              if (bgrMat.empty()) {
                std::cout << "yuv420p->h265: YUV420P->BGR failed !"
                          << std::endl;
                return -7;
              }

              bool isSuccess = cv::imwrite(h265_decode_file_path, bgrMat,
                                           {cv::IMWRITE_JPEG_QUALITY, 100});
              if (!isSuccess) {
                std::cout << "yuv420p->h265: write decode jpeg failed !"
                          << std::endl;
                return -8;
              }
            }
          }
        }

        std::cout << "yuv420p->h265: " << yuv420p_file_path << " successed !"
                  << std::endl;
      }
    }

    // yuyv422
    {
      robosense::h265::H265Coder h265Encoder;
      robosense::h265::H265Coder h265Decoder;

      robosense::h265::H265CodesConfig config;
      config.codeType = robosense::h265::H265_CODER_TYPE::RS_H265_CODER_ENCODE;
      config.imageFrameFormat =
          robosense::common::ImageFrameFormat::FRAME_FORMAT_YUYV422;
      config.imgWidth = imageWidth;
      config.imgHeight = imageHeight;
      config.imgFreq = 10;
      config.codesRateType =
          robosense::h265::H265_CODES_RATE_TYPE::RS_H265_CODES_CUSTOM;
      config.imgCodeRate = 4000000;

      auto config2 = config;
      config2.codeType = robosense::h265::H265_CODER_TYPE::RS_H265_CODER_DECODE;

      int ret = h265Encoder.init(config);
      if (ret != 0) {
        std::cout << "fisheye h265 encoder(yuyv422) initial failed: ret = "
                  << ret << std::endl;
        return -1;
      }

      ret = h265Decoder.init(config2);
      if (ret != 0) {
        std::cout << "fisheye h265 decoder(yuyv422) initial failed: ret = "
                  << ret << std::endl;
        return -2;
      }

      // 获取文件名称
      std::vector<std::string> yuyv422_names;
      {
        std::ifstream ifstr(yuyv422_name_file_path,
                            std::ios_base::in | std::ios_base::binary);
        if (!ifstr.is_open()) {
          std::cout << "yuyv422->h265: open yuyv422 name.txt to read failed:"
                    << yuyv422_name_file_path << std::endl;
          return -3;
        }

        std::string content;
        while (std::getline(ifstr, content)) {
          if (content.empty()) {
            continue;
          }
          yuyv422_names.push_back(content);
        }
      }

      std::sort(yuyv422_names.begin(), yuyv422_names.end());

      std::vector<std::string> h265_file_paths;
      for (size_t i = 0; i < yuyv422_names.size(); ++i) {
        const std::string &yuyv422_file_path =
            yuyv422_dir_path + "/" + yuyv422_names[i];

        std::vector<unsigned char> yuyv422_buffer(yuyv422_image_size, '\0');
        std::ifstream ifstr(yuyv422_file_path,
                            std::ios_base::in | std::ios_base::binary);
        if (!ifstr.is_open()) {
          std::cout << "yuyv422->h265: open yuyv422 file to read failed: "
                    << yuyv422_file_path << std::endl;
          return -4;
        }

        if (!ifstr.read(reinterpret_cast<char *>(yuyv422_buffer.data()),
                        yuyv422_image_size)) {
          std::cout << "yuyv422->h265: read yuyv422 data failed: "
                    << yuyv422_file_path << std::endl;
          return -5;
        }

        robosense::h265::H265Coder::CODER_BUFFER_PTR_VEC h265EncodeBufferPtr;
        std::vector<bool> iFrameFlags;
        int ret = h265Encoder.encode(yuyv422_buffer.data(), yuyv422_image_size,
                                     h265EncodeBufferPtr, iFrameFlags);
        if (ret != 0) {
          std::cout << "yuyv422->h265: h265 encode failed: ret = " << ret
                    << std::endl;
          return -6;
        }

        h265_file_paths.push_back(yuyv422_file_path);
        if (h265EncodeBufferPtr.empty()) {
          std::cout << "yuyv422->h265: " << yuyv422_file_path << " successed !"
                    << std::endl;
          continue;
        }

        for (size_t j = 0; j < h265EncodeBufferPtr.size(); ++j) {
          robosense::h265::H265Coder::CODER_BUFFER_PTR singleBufferPtr =
              h265EncodeBufferPtr[j];

          robosense::h265::H265Coder::CODER_BUFFER_PTR_VEC h265DecodeBufferPtr;
          ret =
              h265Decoder.decode(singleBufferPtr->data(),
                                 singleBufferPtr->size(), h265DecodeBufferPtr);
          if (ret != 0) {
            std::cout << "yuyv422->h265: h265 decode failed: ret = " << ret
                      << std::endl;
            return -7;
          }

          for (size_t k = 0; k < h265DecodeBufferPtr.size(); ++k) {
            const std::string decode_yuv420p_file_path = h265_file_paths[0];
            h265_file_paths.erase(h265_file_paths.begin());

            const std::string &h265_decode_file_path =
                decode_yuv420p_file_path + ".decode.jpeg";
            cv::Mat yuyv422Mat(config.imgHeight, config.imgWidth, CV_8UC2,
                               h265DecodeBufferPtr[k]->data());

            cv::Mat bgrMat;
            cv::cvtColor(yuyv422Mat, bgrMat, cv::COLOR_YUV2BGR_YUYV);
            if (bgrMat.empty()) {
              std::cout << "yuyv422->h265: YUV420P->BGR failed !" << std::endl;
              return -7;
            }

            bool isSuccess = cv::imwrite(h265_decode_file_path, bgrMat,
                                         {cv::IMWRITE_JPEG_QUALITY, 100});
            if (!isSuccess) {
              std::cout << "yuyv422->h265: write decode jpeg failed !"
                        << std::endl;
              return -8;
            }
          }
        }

        if (i == yuyv422_names.size() - 1) {
          robosense::h265::H265Coder::CODER_BUFFER_PTR_VEC h265EncodeBufferPtr;
          std::vector<bool> iFrameFlags;
          int ret = h265Encoder.encodeFlush(h265EncodeBufferPtr, iFrameFlags);
          if (ret != 0) {
            std::cout << "yuyv422->h265: h265 encode flush failed: ret = "
                      << ret << std::endl;
            return -1;
          }

          for (size_t j = 0; j < h265EncodeBufferPtr.size(); ++j) {
            robosense::h265::H265Coder::CODER_BUFFER_PTR singleBufferPtr =
                h265EncodeBufferPtr[j];

            robosense::h265::H265Coder::CODER_BUFFER_PTR_VEC
                h265DecodeBufferPtr;
            ret = h265Decoder.decode(singleBufferPtr->data(),
                                     singleBufferPtr->size(),
                                     h265DecodeBufferPtr);
            if (ret != 0) {
              std::cout << "yuyv422->h265: h265 decode failed: ret = " << ret
                        << std::endl;
              return -7;
            }

            for (size_t k = 0; k < h265DecodeBufferPtr.size(); ++k) {
              const std::string decode_yuv420p_file_path = h265_file_paths[0];
              h265_file_paths.erase(h265_file_paths.begin());

              const std::string &h265_decode_file_path =
                  decode_yuv420p_file_path + ".decode.jpeg";
              cv::Mat yuyv422Mat(config.imgHeight, config.imgWidth, CV_8UC2,
                                 h265DecodeBufferPtr[k]->data());

              cv::Mat bgrMat;
              cv::cvtColor(yuyv422Mat, bgrMat, cv::COLOR_YUV2BGR_YUYV);
              if (bgrMat.empty()) {
                std::cout << "yuyv422->h265: YUV420P->BGR failed !"
                          << std::endl;
                return -7;
              }

              bool isSuccess = cv::imwrite(h265_decode_file_path, bgrMat,
                                           {cv::IMWRITE_JPEG_QUALITY, 100});
              if (!isSuccess) {
                std::cout << "yuyv422->h265: write decode jpeg failed !"
                          << std::endl;
                return -8;
              }
            }
          }

          {
            robosense::h265::H265Coder::CODER_BUFFER_PTR_VEC
                h265DecodeBufferPtr;
            ret = h265Decoder.decodeFlush(h265DecodeBufferPtr);
            if (ret != 0) {
              std::cout << "yuyv422->h265: h265 decode flush failed: ret = "
                        << ret << std::endl;
              return -7;
            }

            for (size_t k = 0; k < h265DecodeBufferPtr.size(); ++k) {
              const std::string decode_yuv420p_file_path = h265_file_paths[0];
              h265_file_paths.erase(h265_file_paths.begin());

              const std::string &h265_decode_file_path =
                  decode_yuv420p_file_path + ".decode.jpeg";
              cv::Mat yuv420pMat(config.imgHeight * 3 / 2, config.imgWidth,
                                 CV_8UC1, h265DecodeBufferPtr[k]->data());

              cv::Mat bgrMat;
              cv::cvtColor(yuv420pMat, bgrMat, cv::COLOR_YUV2BGR_I420);
              if (bgrMat.empty()) {
                std::cout << "yuyv422->h265: YUV420P->BGR failed !"
                          << std::endl;
                return -7;
              }

              bool isSuccess = cv::imwrite(h265_decode_file_path, bgrMat,
                                           {cv::IMWRITE_JPEG_QUALITY, 100});
              if (!isSuccess) {
                std::cout << "yuyv422->h265: write decode jpeg failed !"
                          << std::endl;
                return -8;
              }
            }
          }
        }

        std::cout << "yuyv422->h265: " << yuyv422_file_path << " successed !"
                  << std::endl;
      }
    }
  }

#if defined(ENABLE_USE_CUDA)
  std::cout << "h265 test finished(GPU) !" << std::endl;
#else
  std::cout << "h265 test finished(CPU) !" << std::endl;
#endif // defined(ENABLE_USE_CUDA)

  return 0;
}