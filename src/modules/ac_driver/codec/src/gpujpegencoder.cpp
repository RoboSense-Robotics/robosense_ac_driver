#include "hyper_vision/codec/gpujpegencoder.h"

#if defined(ENABLE_USE_CUDA)

namespace robosense {
namespace jpeg {

GpuJpegEncoder::GpuJpegEncoder() {
  is_initial_ = false;
  gpu_device_id_ = 0;
  image_width_ = 0;
  image_height_ = 0;
  jpeg_quality_ = 70;
  support_yuv420_type_ =
      RS_GPU_JPEG_ENCODE_SUPPORT_TYPE::RS_GPU_JPEG_SUPPORT_NV12;
  yuv420p_buffer_.clear();
  encoder_ = nullptr;
  yuv420_len_ = 0;
  rgb_len_ = 0;
  sample_type_ = RS_GPU_JPEG_ENCODE_SAMPLE_TYPE::RS_GPU_JPEG_ENCODE_SAMPLE_FULL;
  expect_input_len_ = 0;
}

GpuJpegEncoder::~GpuJpegEncoder() {
  is_initial_ = false;
  if (encoder_ != nullptr) {
    gpujpeg_encoder_destroy(encoder_);
    encoder_ = nullptr;
  }
}

int GpuJpegEncoder::init(const int imageWidth, const int imageHeight,
                         const int jpegQuality,
                         const RS_GPU_JPEG_ENCODE_SUPPORT_TYPE yuv420_type,
                         const RS_GPU_JPEG_ENCODE_SAMPLE_TYPE sampleType,
                         const int gpuDeviceId) {
  sample_type_ = sampleType;
  gpu_device_id_ = gpuDeviceId;
  image_width_ = imageWidth;
  image_height_ = imageHeight;
  jpeg_quality_ = jpegQuality;
  support_yuv420_type_ = yuv420_type;
  encoder_ = nullptr;
  yuv420_len_ = image_width_ * image_height_ * 3 / 2;
  yuv420_y_len_ = image_width_ * image_height_;
  yuv420_y_offset_ = 0;
  yuv420_uv_len_ = image_width_ * image_height_ / 4;
  yuv420_u_offset_ = yuv420_y_offset_ + yuv420_y_len_;
  yuv420_v_offset_ = yuv420_u_offset_ + yuv420_uv_len_;

  rgb_len_ = image_width_ * image_height_ * 3;
  yuyv_len_ = image_width_ * image_height_ * 2;

  // init()中更新
  expect_input_len_ = 0;
  yuv420_u_sample_offset_ = 0;
  yuv420_v_sample_offset_ = 0;

  int ret = init();
  if (ret != 0) {
    return -1;
  }

  is_initial_ = true;

  return 0;
}

int GpuJpegEncoder::encode(unsigned char *yuv420Buffer, int yuv420BufferLen,
                           unsigned char *&jpegBuffer, size_t &jpegBufferLen) {
  if (!is_initial_) {
    return -1;
  } else if (yuv420Buffer == nullptr) {
    return -2;
  } else if (yuv420BufferLen != expect_input_len_) {
    return -3;
  }

  if (encoder_ != nullptr) {
    switch (sample_type_) {
    case RS_GPU_JPEG_ENCODE_SAMPLE_TYPE::RS_GPU_JPEG_ENCODE_SAMPLE_HALF:
    case RS_GPU_JPEG_ENCODE_SAMPLE_TYPE::RS_GPU_JPEG_ENCODE_SAMPLE_ONE_THIRD: {
      int sample_factor = 2;
      if (sample_type_ ==
          RS_GPU_JPEG_ENCODE_SAMPLE_TYPE::RS_GPU_JPEG_ENCODE_SAMPLE_ONE_THIRD) {
        sample_factor = 3;
      }
      int sample_factor_2 = 2 * sample_factor;
      // 半分辨率/三分之一压缩
      struct gpujpeg_encoder_input encoder_input;
      if (support_yuv420_type_ ==
          RS_GPU_JPEG_ENCODE_SUPPORT_TYPE::RS_GPU_JPEG_SUPPORT_NV12) {
        // yuv420_nv12 => half yuv420p
        int y_index = 0;
        for (int i = 0; i < image_height_; i += sample_factor) {
          for (int j = 0; j < image_width_; j += sample_factor) {
            yuv420p_sample_buffer_[y_index] =
                yuv420Buffer[i * image_width_ + j];
            ++y_index;
          }
        }
        int uv_index = 0;
        for (int i = 0; i < image_height_ / 2; i += sample_factor) {
          for (int j = 0; j < image_width_; j += sample_factor_2) {
            yuv420p_sample_buffer_[yuv420_u_sample_offset_ + uv_index] =
                yuv420Buffer[yuv420_y_len_ + i * image_width_ + j];
            yuv420p_sample_buffer_[yuv420_v_sample_offset_ + uv_index] =
                yuv420Buffer[yuv420_y_len_ + i * image_width_ + j + 1];
            ++uv_index;
          }
        }
        gpujpeg_encoder_input_set_image(&encoder_input,
                                        yuv420p_sample_buffer_.data());
      } else if (support_yuv420_type_ ==
                 RS_GPU_JPEG_ENCODE_SUPPORT_TYPE::RS_GPU_JPEG_SUPPORT_YUV420P) {
        int y_index = 0;
        for (int i = 0; i < image_height_; i += sample_factor) {
          for (int j = 0; j < image_width_; j += sample_factor) {
            yuv420p_sample_buffer_[y_index] =
                yuv420Buffer[i * image_width_ + j];
            ++y_index;
          }
        }
        int uv_index = 0;
        for (int i = 0; i < image_height_ / 4; i += sample_factor) {
          for (int j = 0; j < image_width_ / 4; j += sample_factor) {
            yuv420p_sample_buffer_[yuv420_u_sample_offset_ + uv_index] =
                yuv420Buffer[yuv420_u_offset_ + (i * image_width_ / 4) + j];
            yuv420p_sample_buffer_[yuv420_v_sample_offset_ + uv_index] =
                yuv420Buffer[yuv420_v_offset_ + (i * image_width_ / 4) + j];
            ++uv_index;
          }
        }
        gpujpeg_encoder_input_set_image(&encoder_input,
                                        yuv420p_sample_buffer_.data());
      } else if (support_yuv420_type_ ==
                 RS_GPU_JPEG_ENCODE_SUPPORT_TYPE::RS_GPU_JPEG_SUPPORT_YUYV422) {
        // 采样以及yuyv->uyvy
        int yuyv_index = 0;
        for (int i = 0; i < image_height_; i += sample_factor) {
          for (int j = 0; j < image_width_ * 2; j += 2 * sample_factor) {
            yuv420p_sample_buffer_[yuyv_index++] =
                yuv420Buffer[image_width_ * 2 * i + j + 1]; // u 或 v
            yuv420p_sample_buffer_[yuyv_index++] =
                yuv420Buffer[image_width_ * 2 * i + j]; // y
            if (sample_factor % 2 == 0 &&
                j + 2 * sample_factor + 1 < image_width_ * 2) {
              yuv420Buffer[j + 2 * sample_factor + 1] =
                  yuv420Buffer[j + 2 * sample_factor - 1];
            }
          }
        }
        gpujpeg_encoder_input_set_image(&encoder_input,
                                        yuv420p_sample_buffer_.data());
      } else if (support_yuv420_type_ ==
                 RS_GPU_JPEG_ENCODE_SUPPORT_TYPE::RS_GPU_JPEG_SUPPORT_RGB) {
        const int pitch = 3 * image_width_;
        for (int i = 0, j = 0; i < rgb_len_; i += 3, ++j) {
          int row = i / pitch;
          int col = (i - row * 3) / 3;
          if (row % sample_factor == 0 && col % sample_factor == 0) {
            rgb_sample_buffer_[j * 3] = yuv420Buffer[i];
            rgb_sample_buffer_[j * 3 + 1] = yuv420Buffer[i + 1];
            rgb_sample_buffer_[j * 3 + 2] = yuv420Buffer[i + 2];
          }
        }
        gpujpeg_encoder_input_set_image(&encoder_input,
                                        rgb_sample_buffer_.data());
      }
      int ret =
          gpujpeg_encoder_encode(encoder_, &param_, &param_image_,
                                 &encoder_input, &jpegBuffer, &jpegBufferLen);
      if (ret != 0) {
        return -4;
      }
      break;
    }
    case RS_GPU_JPEG_ENCODE_SAMPLE_TYPE::RS_GPU_JPEG_ENCODE_SAMPLE_FULL: {
      // 全分辨率压缩
      struct gpujpeg_encoder_input encoder_input;
      if (support_yuv420_type_ ==
          RS_GPU_JPEG_ENCODE_SUPPORT_TYPE::RS_GPU_JPEG_SUPPORT_NV12) {
        // yuv420_nv12 => yuv420p
        memcpy(yuv420p_buffer_.data(), yuv420Buffer + yuv420_y_offset_,
               yuv420_y_len_);
        for (int i = 0; i < yuv420_uv_len_; ++i) {
          yuv420p_buffer_[yuv420_u_offset_ + i] =
              yuv420Buffer[yuv420_y_len_ + i * 2];
          yuv420p_buffer_[yuv420_v_offset_ + i] =
              yuv420Buffer[yuv420_y_len_ + i * 2 + 1];
        }
        gpujpeg_encoder_input_set_image(&encoder_input, yuv420p_buffer_.data());
      } else if (support_yuv420_type_ ==
                 RS_GPU_JPEG_ENCODE_SUPPORT_TYPE::RS_GPU_JPEG_SUPPORT_YUYV422) {
        // yuyv->uyvy
        for (int i = 0; i < yuyv_len_; i += 2) {
          unsigned char tmp = yuv420Buffer[i + 1];
          yuv420Buffer[i + 1] = yuv420Buffer[i];
          yuv420Buffer[i] = tmp;
        }
        gpujpeg_encoder_input_set_image(&encoder_input, yuv420Buffer);
      } else {
        gpujpeg_encoder_input_set_image(&encoder_input, yuv420Buffer);
      }

      int ret =
          gpujpeg_encoder_encode(encoder_, &param_, &param_image_,
                                 &encoder_input, &jpegBuffer, &jpegBufferLen);
      if (ret != 0) {
        return -4;
      }
      break;
    }
    default: {
      break;
    }
    }
  } else {
    return -5;
  }

  return 0;
}

int GpuJpegEncoder::init() {
  int ret = gpujpeg_init_device(gpu_device_id_, 0);
  if (ret != 0) {
    return -1;
  }

  int sample_factor = 1;
  if (sample_type_ ==
      RS_GPU_JPEG_ENCODE_SAMPLE_TYPE::RS_GPU_JPEG_ENCODE_SAMPLE_HALF) {
    sample_factor = 2;
  } else if (sample_type_ == RS_GPU_JPEG_ENCODE_SAMPLE_TYPE::
                                 RS_GPU_JPEG_ENCODE_SAMPLE_ONE_THIRD) {
    sample_factor = 3;
  }
  int sample_factor_2 = sample_factor * sample_factor;
  yuv420_u_sample_offset_ = yuv420_u_offset_ / sample_factor_2;
  yuv420_v_sample_offset_ = yuv420_v_offset_ / sample_factor_2;

  if (support_yuv420_type_ ==
      RS_GPU_JPEG_ENCODE_SUPPORT_TYPE::RS_GPU_JPEG_SUPPORT_NV12) {
    yuv420p_buffer_.resize(yuv420_len_, 0);
    if (sample_type_ ==
            RS_GPU_JPEG_ENCODE_SAMPLE_TYPE::RS_GPU_JPEG_ENCODE_SAMPLE_HALF ||
        sample_type_ == RS_GPU_JPEG_ENCODE_SAMPLE_TYPE::
                            RS_GPU_JPEG_ENCODE_SAMPLE_ONE_THIRD) {
      yuv420p_sample_buffer_.resize(yuv420_len_ / sample_factor_2, 0);
    }
  } else if (support_yuv420_type_ ==
             RS_GPU_JPEG_ENCODE_SUPPORT_TYPE::RS_GPU_JPEG_SUPPORT_RGB) {
    if (sample_type_ ==
            RS_GPU_JPEG_ENCODE_SAMPLE_TYPE::RS_GPU_JPEG_ENCODE_SAMPLE_HALF ||
        sample_type_ == RS_GPU_JPEG_ENCODE_SAMPLE_TYPE::
                            RS_GPU_JPEG_ENCODE_SAMPLE_ONE_THIRD) {
      rgb_sample_buffer_.resize(rgb_len_ / sample_factor_2);
    }
  } else if (support_yuv420_type_ ==
             RS_GPU_JPEG_ENCODE_SUPPORT_TYPE::RS_GPU_JPEG_SUPPORT_YUV420P) {
    if (sample_type_ ==
            RS_GPU_JPEG_ENCODE_SAMPLE_TYPE::RS_GPU_JPEG_ENCODE_SAMPLE_HALF ||
        sample_type_ == RS_GPU_JPEG_ENCODE_SAMPLE_TYPE::
                            RS_GPU_JPEG_ENCODE_SAMPLE_ONE_THIRD) {
      yuv420p_sample_buffer_.resize(yuv420_len_ / sample_factor_2, 0);
    }
  } else if (support_yuv420_type_ ==
             RS_GPU_JPEG_ENCODE_SUPPORT_TYPE::RS_GPU_JPEG_SUPPORT_YUYV422) {
    yuv420p_buffer_.resize(yuyv_len_, 0);
    if (sample_type_ ==
            RS_GPU_JPEG_ENCODE_SAMPLE_TYPE::RS_GPU_JPEG_ENCODE_SAMPLE_HALF ||
        sample_type_ == RS_GPU_JPEG_ENCODE_SAMPLE_TYPE::
                            RS_GPU_JPEG_ENCODE_SAMPLE_ONE_THIRD) {
      yuv420p_sample_buffer_.resize(yuyv_len_ / sample_factor_2, 0);
    }
  }

  gpujpeg_set_default_parameters(&param_);
  param_.quality = jpeg_quality_;
  param_.restart_interval = 16;
  param_.interleaved = 1;

  gpujpeg_image_set_default_parameters(&param_image_);
  param_image_.width = image_width_ / sample_factor;
  param_image_.height = image_height_ / sample_factor;
  // param_image_.comp_count = 3;

  if (support_yuv420_type_ ==
      RS_GPU_JPEG_ENCODE_SUPPORT_TYPE::RS_GPU_JPEG_SUPPORT_RGB) {
    param_image_.color_space = GPUJPEG_RGB;
    param_image_.pixel_format = GPUJPEG_444_U8_P012;
    expect_input_len_ = rgb_len_;
  } else if (support_yuv420_type_ ==
             RS_GPU_JPEG_ENCODE_SUPPORT_TYPE::RS_GPU_JPEG_SUPPORT_YUYV422) {
    param_image_.color_space = GPUJPEG_YCBCR_BT601;
    param_image_.pixel_format = GPUJPEG_422_U8_P1020;
    expect_input_len_ = yuyv_len_;
  } else {
    param_image_.color_space = GPUJPEG_YCBCR_BT601;
    param_image_.pixel_format = GPUJPEG_420_U8_P0P1P2;
    expect_input_len_ = yuv420_len_;
  }

  encoder_ = gpujpeg_encoder_create(0);
  if (encoder_ == nullptr) {
    return -2;
  }

  return 0;
}

} // namespace jpeg
} // namespace robosense

#endif //
