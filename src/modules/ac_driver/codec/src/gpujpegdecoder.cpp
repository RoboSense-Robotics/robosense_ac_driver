#include "hyper_vision/codec/gpujpegdecoder.h"

#if defined(ENABLE_USE_CUDA)
namespace robosense {
namespace jpeg {

GpuJpegDecoder::GpuJpegDecoder() {
  is_initial_ = false;
  gpu_device_id_ = 0;
  image_width_ = 0;
  image_height_ = 0;
  color_space_ = GPUJPEG_RGB;
  decoder_ = nullptr;
}

GpuJpegDecoder::~GpuJpegDecoder() {
  is_initial_ = false;
  if (decoder_ != nullptr) {
    gpujpeg_decoder_destroy(decoder_);
    decoder_ = nullptr;
  }
}

int GpuJpegDecoder::init(const int imageWidth, const int imageHeight,
                         const RS_GPU_JPEG_ENCODE_SUPPORT_TYPE output_type,
                         const int gpuDeviceId) {
  gpu_device_id_ = gpuDeviceId;
  image_width_ = imageWidth;
  image_height_ = imageHeight;
  output_type_ = output_type;
  decoder_ = nullptr;

  int ret = init();
  if (ret != 0) {
    return -1;
  }

  is_initial_ = true;

  return 0;
}

int GpuJpegDecoder::decode(unsigned char *jpegBuffer, int jpegBufferLen,
                           unsigned char *rawBuffer, size_t &rawBufferLen) {
  if (!is_initial_) {
    return -1;
  } else if (jpegBuffer == nullptr) {
    return -2;
  } else if (jpegBufferLen == 0) {
    return -3;
  } else if (rawBuffer == nullptr) {
    return -4;
  }

  if (decoder_ != nullptr) {
    struct gpujpeg_decoder_output decoder_output;
    gpujpeg_decoder_output_set_default(&decoder_output);
    int ret = gpujpeg_decoder_decode(decoder_, jpegBuffer, jpegBufferLen,
                                     &decoder_output);
    if (ret != 0) {
      return -5;
    }
    if (rawBufferLen < decoder_output.data_size) {
      std::cout << "decoder_output.data_size = " << decoder_output.data_size
                << ", rawBufferLen = " << rawBufferLen << std::endl;
      return -6;
    }
    rawBufferLen = decoder_output.data_size;
    if (output_type_ ==
        RS_GPU_JPEG_ENCODE_SUPPORT_TYPE::RS_GPU_JPEG_SUPPORT_NV12) { // YUV420P
                                                                     // -> NV12
      const uint32_t y_size = image_width_ * image_height_;
      const uint32_t u_v_size = y_size / 4;
      const uint32_t u_offset = y_size;
      const uint32_t v_offset = y_size + u_v_size;
      memcpy(rawBuffer, decoder_output.data, y_size);
      uint32_t index = y_size;
      for (uint32_t i = 0; i < u_v_size; ++i) {
        rawBuffer[index] = decoder_output.data[u_offset + i];
        rawBuffer[index + 1] = decoder_output.data[v_offset + i];
        index += 2;
      }
    } else if (output_type_ ==
               RS_GPU_JPEG_ENCODE_SUPPORT_TYPE::RS_GPU_JPEG_SUPPORT_YUYV422) {
      memcpy(rawBuffer, decoder_output.data, rawBufferLen);
      // uyvy -> yuyv
      for (uint32_t i = 0; i < rawBufferLen; i += 2) {
        unsigned char tmp = rawBuffer[i + 1];
        rawBuffer[i + 1] = rawBuffer[i];
        rawBuffer[i] = tmp;
      }
    } else {
      memcpy(rawBuffer, decoder_output.data, rawBufferLen);
    }
  }

  return 0;
}

int GpuJpegDecoder::init() {
  int ret = gpujpeg_init_device(gpu_device_id_, 0);
  if (ret != 0) {
    return -1;
  }

  gpujpeg_set_default_parameters(&param_);
  param_.restart_interval = 16;
  param_.interleaved = 1;

  gpujpeg_image_set_default_parameters(&param_image_);
  param_image_.width = image_width_;
  param_image_.height = image_height_;
  // param_image_.comp_count = 3;

  switch (output_type_) {
  case RS_GPU_JPEG_ENCODE_SUPPORT_TYPE::RS_GPU_JPEG_SUPPORT_YUV420P: {
    param_image_.color_space = GPUJPEG_YCBCR_JPEG;
    param_image_.pixel_format = GPUJPEG_420_U8_P0P1P2;
    // std::cout << "run here gpu jpeg yuv420p" << std::endl;
    break;
  }
  case RS_GPU_JPEG_ENCODE_SUPPORT_TYPE::RS_GPU_JPEG_SUPPORT_NV12: {
    param_image_.color_space = GPUJPEG_YCBCR_JPEG;
    param_image_.pixel_format = GPUJPEG_420_U8_P0P1P2;
    // std::cout << "run here gpu jpeg nv12" << std::endl;
    break;
  }
  case RS_GPU_JPEG_ENCODE_SUPPORT_TYPE::RS_GPU_JPEG_SUPPORT_RGB: {
    param_image_.color_space = GPUJPEG_RGB;
    param_image_.pixel_format = GPUJPEG_444_U8_P012;
    // std::cout << "run here gpu jpeg rgb" << std::endl;
    break;
  }
  case RS_GPU_JPEG_ENCODE_SUPPORT_TYPE::RS_GPU_JPEG_SUPPORT_YUYV422: {
    param_image_.color_space = GPUJPEG_YCBCR_JPEG;
    param_image_.pixel_format = GPUJPEG_422_U8_P1020;
    // std::cout << "run here gpu jpeg yuyv422" << std::endl;
    break;
  }
  }
  // gpujpeg_decoder_set_output_format(decoder_, color_space_, pixel_format_);

  decoder_ = gpujpeg_decoder_create(0);
  if (decoder_ == nullptr) {
    return -2;
  }

  ret = gpujpeg_decoder_init(decoder_, &param_, &param_image_);
  if (ret != 0) {
    return -3;
  }

  return 0;
}

} // namespace jpeg
} // namespace robosense

#endif //
