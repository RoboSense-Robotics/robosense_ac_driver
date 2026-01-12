#include "hyper_vision/codec/colorcodec.h"

namespace robosense {
namespace color {

int ColorCodec::YUYV422ToRGB(unsigned char *pSrcBuffer, const int srcBufferLen,
                             unsigned char *pDstBuffer, int &dstBufferLen) {
  if (pSrcBuffer == nullptr || srcBufferLen != image_yuyv422_size_ ||
      pDstBuffer == nullptr || dstBufferLen < image_rgb_size_) {
    return -1;
  }

#if defined(ENABLE_USE_CUDA)
  cudaMemcpy(pUchar4Buf_, pSrcBuffer, image_yuyv422_size_,
             cudaMemcpyHostToDevice);
  CUDA_YUYV422ToRGB(pUchar4Buf_, pUchar6Buf_, pRgbBuf_, image_width_,
                    image_height_);
  cudaMemcpy(pDstBuffer, pRgbBuf_, image_rgb_size_, cudaMemcpyDeviceToHost);
#else
  cv::Mat yuyvMat(image_height_, image_width_, CV_8UC2, pSrcBuffer);
  cv::Mat rgbMat;
  cv::cvtColor(yuyvMat, rgbMat, cv::COLOR_YUV2RGB_YUYV);
  if (rgbMat.empty()) {
    return -1;
  } else if (!rgbMat.isContinuous()) {
    return -2;
  }
  memcpy(pDstBuffer, rgbMat.data, image_rgb_size_);
#endif // defined(ENABLE_USE_CUDA)

  dstBufferLen = image_rgb_size_;
  return 0;
}

int ColorCodec::NV12ToRGB(unsigned char *pSrcBuffer, const int srcBufferLen,
                          unsigned char *pDstBuffer, int &dstBufferLen) {
  if (pSrcBuffer == nullptr || srcBufferLen != image_nv12_size_ ||
      pDstBuffer == nullptr || dstBufferLen < image_rgb_size_) {
    return -1;
  }

#if defined(ENABLE_USE_CUDA)
  cudaMemcpy(pYuv420xBuf_, pSrcBuffer, image_nv12_size_,
             cudaMemcpyHostToDevice);
  CUDA_NV12ToRGB(pYuv420xBuf_, pOutBuf_, pRgbBuf_, image_width_, image_height_);
  cudaMemcpy(pDstBuffer, pRgbBuf_, image_rgb_size_, cudaMemcpyDeviceToHost);
#else
  cv::Mat yuvMat(image_height_ * 3 / 2, image_width_, CV_8UC1, pSrcBuffer);
  cv::Mat rgbMat;
  cv::cvtColor(yuvMat, rgbMat, cv::COLOR_YUV2RGB_NV12);

  if (rgbMat.empty()) {
    return -2;
  } else if (!rgbMat.isContinuous()) {
    return -3;
  }
  memcpy(pDstBuffer, rgbMat.data, image_rgb_size_);
#endif // defined(ENABLE_USE_CUDA)
  dstBufferLen = image_rgb_size_;
  return 0;
}

int ColorCodec::YUV420PToRGB(unsigned char *pSrcBuffer, const int srcBufferLen,
                             unsigned char *pDstBuffer, int &dstBufferLen) {
  if (pSrcBuffer == nullptr || srcBufferLen != image_yuv420p_size_ ||
      pDstBuffer == nullptr || dstBufferLen < image_rgb_size_) {
    return -1;
  }

#if defined(ENABLE_USE_CUDA)
  cudaMemcpy(pYuv420xBuf_, pSrcBuffer, image_yuv420p_size_,
             cudaMemcpyHostToDevice);
  CUDA_YUV420PToRGB(pYuv420xBuf_, pOutBuf_, pRgbBuf_, image_width_,
                    image_height_);
  cudaMemcpy(pDstBuffer, pRgbBuf_, image_rgb_size_, cudaMemcpyDeviceToHost);
#else
  cv::Mat yuvMat(image_height_ * 3 / 2, image_width_, CV_8UC1, pSrcBuffer);
  cv::Mat rgbMat;
  cv::cvtColor(yuvMat, rgbMat, cv::COLOR_YUV2RGB_I420);

  if (rgbMat.empty()) {
    return -2;
  } else if (!rgbMat.isContinuous()) {
    return -3;
  }
  memcpy(pDstBuffer, rgbMat.data, image_rgb_size_);
#endif // defined(ENABLE_USE_CUDA)
  dstBufferLen = image_rgb_size_;
  return 0;
}

int ColorCodec::RGBToYUYV422(unsigned char *pSrcBuffer, const int srcBufferLen,
                             unsigned char *pDstBuffer, int &dstBufferLen) {
  if (pSrcBuffer == nullptr || srcBufferLen != image_rgb_size_ ||
      pDstBuffer == nullptr || dstBufferLen < image_yuyv422_size_) {
    return -1;
  }
#if defined(ENABLE_USE_CUDA)
  cudaMemcpy(pRgbBuf_, pSrcBuffer, image_rgb_size_, cudaMemcpyHostToDevice);
  CUDA_RGBToYUYV422(pRgbBuf_, pYuyv422Buf_, pDstBuffer, image_width_,
                    image_height_);
#else
  cv::Mat rgbMat(image_height_, image_width_, CV_8UC3, pSrcBuffer);
  cv::Mat yuv444Mat;
  cv::cvtColor(rgbMat, yuv444Mat, cv::COLOR_RGB2YUV); // YUV: 4:4:4
  if (yuv444Mat.empty()) {
    return -2;
  }
  // std::cout << "wxh = " << yuv444Mat.cols << "x" << yuv444Mat.rows <<
  // std::endl;
  int index = 0;
  for (int y = 0; y < image_height_; ++y) {
    cv::Vec3b *row = yuv444Mat.ptr<cv::Vec3b>(y);
    for (int x = 0; x < image_width_; ++x) {
      pDstBuffer[index] = row[x][0];
      pDstBuffer[index + 1] =
          (x % 2 == 0 ? row[x][1] : row[x][2]); // 偶数列取值: u, 奇数列取值: v
      index += 2;
    }
  }
#endif // defined(ENABLE_USE_CUDA)

  dstBufferLen = image_yuyv422_size_;
  return 0;
}

int ColorCodec::RGBToNV12(unsigned char *pSrcBuffer, const int srcBufferLen,
                          unsigned char *pDstBuffer, int &dstBufferLen) {

  if (pSrcBuffer == nullptr || srcBufferLen != image_rgb_size_ ||
      pDstBuffer == nullptr || dstBufferLen < image_nv12_size_) {
    return -1;
  }
#if defined(ENABLE_USE_CUDA)
  cudaMemcpy(pRgbBuf_, pSrcBuffer, image_rgb_size_, cudaMemcpyHostToDevice);
  CUDA_RGBToNV12(pRgbBuf_, pYuv420xBuf_, pDstBuffer, image_width_,
                 image_height_);
#else
  cv::Mat rgbMat(image_height_, image_width_, CV_8UC3, pSrcBuffer);

  cv::Mat yuv420pMat;
  cv::cvtColor(rgbMat, yuv420pMat, cv::COLOR_RGB2YUV_I420);

  if (yuv420pMat.empty()) {
    return -2;
  } else if (!yuv420pMat.isContinuous()) {
    return -3;
  }

  int ret = YUV420PToNV12(yuv420pMat.data, image_yuv420p_size_, pDstBuffer,
                          dstBufferLen);
  if (ret != 0) {
    return -4;
  }
#endif // defined(ENABLE_USE_CUDA)
  dstBufferLen = image_nv12_size_;
  return 0;
}

int ColorCodec::RGBToYUV420P(unsigned char *pSrcBuffer, const int srcBufferLen,
                             unsigned char *pDstBuffer, int &dstBufferLen) {
  if (pSrcBuffer == nullptr || srcBufferLen != image_rgb_size_ ||
      pDstBuffer == nullptr || dstBufferLen < image_yuv420p_size_) {
    return -1;
  }
#if defined(ENABLE_USE_CUDA)
  cudaMemcpy(pRgbBuf_, pSrcBuffer, image_rgb_size_, cudaMemcpyHostToDevice);
  CUDA_RGBToYUV420P(pRgbBuf_, pYuv420xBuf_, pDstBuffer, image_width_,
                    image_height_);
#else
  cv::Mat rgbMat(image_height_, image_width_, CV_8UC3, pSrcBuffer);
  cv::Mat yuv420pMat;
  cv::cvtColor(rgbMat, yuv420pMat, cv::COLOR_RGB2YUV_I420);
  if (yuv420pMat.empty()) {
    return -2;
  } else if (!yuv420pMat.isContinuous()) {
    return -3;
  }
  memcpy(pDstBuffer, yuv420pMat.data, image_yuv420p_size_);
#endif // defined(ENABLE_USE_CUDA)
  dstBufferLen = image_yuv420p_size_;

  return 0;
}

int ColorCodec::NV12ToRgbToYUV422(unsigned char *pSrcBuffer,
                                  const int srcBufferLen,
                                  unsigned char *pDstBuffer,
                                  int &dstBufferLen) {

  if (pSrcBuffer == nullptr || srcBufferLen != image_nv12_size_ ||
      pDstBuffer == nullptr || dstBufferLen < image_yuyv422_size_) {
    return -1;
  }

  int rgbBufferLen = rgb_buffer_.size();
  int ret =
      NV12ToRGB(pSrcBuffer, srcBufferLen, rgb_buffer_.data(), rgbBufferLen);
  if (ret != 0) {
    return -2;
  }

  ret = RGBToYUYV422(rgb_buffer_.data(), rgb_buffer_.size(), pDstBuffer,
                     dstBufferLen);
  if (ret != 0) {
    return -3;
  }

  return 0;
}

int ColorCodec::YUYV422ToRgbToNV12(unsigned char *pSrcBuffer,
                                   const int srcBufferLen,
                                   unsigned char *pDstBuffer,
                                   int &dstBufferLen) {
  if (pSrcBuffer == nullptr || srcBufferLen != image_yuyv422_size_ ||
      pDstBuffer == nullptr || dstBufferLen < image_nv12_size_) {
    return -1;
  }

  int rgbBufferLen = rgb_buffer_.size();
  int ret =
      YUYV422ToRGB(pSrcBuffer, srcBufferLen, rgb_buffer_.data(), rgbBufferLen);
  if (ret != 0) {
    return -2;
  }

  ret = RGBToNV12(rgb_buffer_.data(), rgb_buffer_.size(), pDstBuffer,
                  dstBufferLen);
  if (ret != 0) {
    return -3;
  }

  return 0;
}

int ColorCodec::YUV420PToRgbToYUYV422(unsigned char *pSrcBuffer,
                                      const int srcBufferLen,
                                      unsigned char *pDstBuffer,
                                      int &dstBufferLen) {
  if (pSrcBuffer == nullptr || srcBufferLen != image_yuv420p_size_ ||
      pDstBuffer == nullptr || dstBufferLen < image_yuyv422_size_) {
    return -1;
  }

  int rgbBufferLen = rgb_buffer_.size();
  int ret =
      YUV420PToRGB(pSrcBuffer, srcBufferLen, rgb_buffer_.data(), rgbBufferLen);
  if (ret != 0) {
    return -2;
  }

  ret = RGBToYUYV422(rgb_buffer_.data(), rgb_buffer_.size(), pDstBuffer,
                     dstBufferLen);
  if (ret != 0) {
    return -3;
  }

  return 0;
}

int ColorCodec::YUYV422ToRgbToYUV420P(unsigned char *pSrcBuffer,
                                      const int srcBufferLen,
                                      unsigned char *pDstBuffer,
                                      int &dstBufferLen) {
  if (pSrcBuffer == nullptr || srcBufferLen != image_yuyv422_size_ ||
      pDstBuffer == nullptr || dstBufferLen < image_yuv420p_size_) {
    return -1;
  }

  int rgbBufferLen = rgb_buffer_.size();
  int ret =
      YUYV422ToRGB(pSrcBuffer, srcBufferLen, rgb_buffer_.data(), rgbBufferLen);
  if (ret != 0) {
    return -2;
  }

  ret = RGBToYUV420P(rgb_buffer_.data(), rgb_buffer_.size(), pDstBuffer,
                     dstBufferLen);
  if (ret != 0) {
    return -3;
  }

  return 0;
}

int ColorCodec::NV12ToYUV420P(unsigned char *pSrcBuffer, const int srcBufferLen,
                              unsigned char *pDstBuffer, int &dstBufferLen) {
  if (pSrcBuffer == nullptr || srcBufferLen != image_nv12_size_ ||
      pDstBuffer == nullptr || dstBufferLen < image_nv12_size_) {
    return -1;
  }

  memcpy(pDstBuffer, pSrcBuffer, image_yuv420_y_size_);
  int index = 0;
  for (int i = 0; i < image_yuv420_uv_size_; i += 2) {
    pDstBuffer[image_yuv420p_u_offset_ + index] =
        pSrcBuffer[image_yuv420_y_size_ + i];
    pDstBuffer[image_yuv420p_v_offset_ + index] =
        pSrcBuffer[image_yuv420_y_size_ + i + 1];
    ++index;
  }
  dstBufferLen = image_nv12_size_;

  return 0;
}

int ColorCodec::YUV420PToNV12(unsigned char *pSrcBuffer, const int srcBufferLen,
                              unsigned char *pDstBuffer, int &dstBufferLen) {
  if (pSrcBuffer == nullptr || srcBufferLen != image_nv12_size_ ||
      pDstBuffer == nullptr || dstBufferLen < image_nv12_size_) {
    return -1;
  }

  memcpy(pDstBuffer, pSrcBuffer, image_yuv420_y_size_);
  int index = 0;
  for (int i = 0; i < image_yuv420_u_v_size_; ++i) {
    pDstBuffer[image_yuv420_y_size_ + index] =
        pSrcBuffer[image_yuv420p_u_offset_ + i];
    pDstBuffer[image_yuv420_y_size_ + index + 1] =
        pSrcBuffer[image_yuv420p_v_offset_ + i];
    index += 2;
  }
  dstBufferLen = image_nv12_size_;

  return 0;
}

int ColorCodec::YUYV422ToYUV422P(unsigned char *pSrcBuffer,
                                 const int srcBufferLen,
                                 unsigned char *pDstBuffer, int &dstBufferLen) {
  if (pSrcBuffer == nullptr || srcBufferLen != image_yuyv422_size_ ||
      pDstBuffer == nullptr || dstBufferLen < image_yuv422p_size_) {
    return -1;
  }

  int y_index = 0;
  int u_index = image_yuyv422p_u_offset_;
  int v_index = image_yuyv422p_v_offset_;
  for (int y = 0; y < image_height_; ++y) {
    for (int x = 0; x < image_width_; ++x) {
      int y_pos = (y * image_width_ + x) * 2;
      pDstBuffer[y_index++] = pSrcBuffer[y_pos];
      if (x % 2 == 0) {
        pDstBuffer[u_index++] = pSrcBuffer[y_pos + 1];
      } else {
        pDstBuffer[v_index++] = pSrcBuffer[y_pos + 1];
      }
    }
  }
  dstBufferLen = image_yuv422p_size_;

  return 0;
}

int ColorCodec::YUV422PToYUYV422(unsigned char *pSrcBuffer,
                                 const int srcBufferLen,
                                 unsigned char *pDstBuffer, int &dstBufferLen) {
  if (pSrcBuffer == nullptr || srcBufferLen != image_yuv422p_size_ ||
      pDstBuffer == nullptr || dstBufferLen < image_yuyv422_size_) {
    return -1;
  }

  int y_index = 0;
  int u_index = image_yuyv422p_u_offset_;
  int v_index = image_yuyv422p_v_offset_;
  int index = 0;
  for (int y = 0; y < image_height_; ++y) {
    for (int x = 0; x < image_width_; ++x) {
      int y_pos = (y * image_width_ + x) * 2;
      pDstBuffer[index++] = pSrcBuffer[y_index++];
      if (x % 2 == 0) {
        pDstBuffer[index++] = pSrcBuffer[u_index++];
      } else {
        pDstBuffer[index++] = pSrcBuffer[v_index++];
      }
    }
  }
  dstBufferLen = image_yuyv422_size_;

  return 0;
}

#if defined(ENABLE_USE_CUDA)
int ColorCodec::initCuda() {
  if (pYuv420xBuf_ == nullptr) {
    CHECK_CUDA(
        cudaMalloc(reinterpret_cast<void **>(&pYuv420xBuf_), image_nv12_size_));
  }

  if (pYuyv422Buf_ == nullptr) {
    CHECK_CUDA(cudaMalloc(reinterpret_cast<void **>(&pYuyv422Buf_),
                          image_yuyv422_size_));
  }

  if (pRgbBuf_ == nullptr) {
    CHECK_CUDA(
        cudaMalloc(reinterpret_cast<void **>(&pRgbBuf_), image_rgb_size_));
  }

  if (pOutBuf_ == nullptr) {
    CHECK_CUDA(
        cudaMalloc(reinterpret_cast<void **>(&pOutBuf_), image_rgb_size_));
  }

  if (pUchar4Buf_ == nullptr) {
    CHECK_CUDA(cudaMalloc(reinterpret_cast<void **>(&pUchar4Buf_),
                          image_rgb_size_ * sizeof(uchar4)));
  }

  if (pUchar6Buf_ == nullptr) {
    CHECK_CUDA(cudaMalloc(reinterpret_cast<void **>(&pUchar6Buf_),
                          image_rgb_size_ * sizeof(uchar6)));
  }

  return 0;
}

int ColorCodec::releaseCuda() {
  if (pYuv420xBuf_) {
    CHECK_CUDA(cudaFree(pYuv420xBuf_));
    pYuv420xBuf_ = nullptr;
  }
  if (pYuyv422Buf_) {
    CHECK_CUDA(cudaFree(pYuyv422Buf_));
    pYuyv422Buf_ = nullptr;
  }
  if (pRgbBuf_) {
    CHECK_CUDA(cudaFree(pRgbBuf_));
    pRgbBuf_ = nullptr;
  }
  if (pOutBuf_) {
    CHECK_CUDA(cudaFree(pOutBuf_));
    pOutBuf_ = nullptr;
  }
  if (pUchar4Buf_) {
    CHECK_CUDA(cudaFree(pUchar4Buf_));
    pUchar4Buf_ = nullptr;
  }
  if (pUchar6Buf_) {
    CHECK_CUDA(cudaFree(pUchar6Buf_))
    pUchar6Buf_ = nullptr;
  }
  return 0;
}

#endif // defined(ENABLE_USE_CUDA)

} // namespace color
} // namespace robosense