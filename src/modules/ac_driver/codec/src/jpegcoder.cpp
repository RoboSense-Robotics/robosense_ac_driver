#include "hyper_vision/codec/jpegcoder.h"

namespace robosense {
namespace jpeg {

JpegCoder::JpegCoder() {}
JpegCoder::~JpegCoder() {}

int JpegCoder::init(const JpegCodesConfig &jpegCodesConfig) {
  jpeg_codes_config_ = jpegCodesConfig;

  int ret = init();
  if (ret != 0) {
    return -1;
  }

  return 0;
}

int JpegCoder::encode(unsigned char *yuv420Buffer, int yuv420BufferLen,
                      unsigned char *jpegBuffer, size_t &jpegBufferLen) {
#if defined(ENABLE_USE_CUDA)
  if (gpu_jpeg_encoder_ptr_ != nullptr) {
    unsigned char *jpegBufferTmp = nullptr;
    size_t jpegBufferLenTmp = jpegBufferLen;
    int ret = gpu_jpeg_encoder_ptr_->encode(yuv420Buffer, yuv420BufferLen,
                                            jpegBufferTmp, jpegBufferLenTmp);
    if (ret != 0) {
      return -1;
    } else if (jpegBuffer == nullptr) {
      return -2;
    }
    memcpy(jpegBuffer, jpegBufferTmp, jpegBufferLenTmp);
    jpegBufferLen = jpegBufferLenTmp;
  } else if (opencv_jpeg_encoder_ptr_ != nullptr) {
    int ret = opencv_jpeg_encoder_ptr_->encode(yuv420Buffer, yuv420BufferLen,
                                               jpegBuffer, jpegBufferLen);
    if (ret != 0) {
      return -3;
    }
  }
#else
  if (opencv_jpeg_encoder_ptr_ != nullptr) {
    int ret = opencv_jpeg_encoder_ptr_->encode(yuv420Buffer, yuv420BufferLen,
                                               jpegBuffer, jpegBufferLen);
    if (ret != 0) {
      return -4;
    }
  }
#endif // defined(ENABLE_USE_CUDA)
  return 0;
}

int JpegCoder::decode(unsigned char *yuv420Buffer, int yuv420BufferLen,
                      unsigned char *jpegBuffer, size_t &jpegBufferLen) {
#if defined(ENABLE_USE_CUDA)
  if (gpu_jpeg_decoder_ptr_ != nullptr) {
    int ret = gpu_jpeg_decoder_ptr_->decode(yuv420Buffer, yuv420BufferLen,
                                            jpegBuffer, jpegBufferLen);
    if (ret != 0) {
      std::cout << "jpeg gpu decoder(enable gpu) failed: ret = " << ret
                << std::endl;
      return -1;
    }
  } else if (opencv_jpeg_decoder_ptr_ != nullptr) {
    int ret = opencv_jpeg_decoder_ptr_->decode(yuv420Buffer, yuv420BufferLen,
                                               jpegBuffer, jpegBufferLen);
    if (ret != 0) {
      std::cout << "jpeg cpu decoder(enable gpu) failed: ret = " << ret
                << std::endl;
      return -2;
    }
  }
#else
  if (opencv_jpeg_decoder_ptr_ != nullptr) {
    int ret = opencv_jpeg_decoder_ptr_->decode(yuv420Buffer, yuv420BufferLen,
                                               jpegBuffer, jpegBufferLen);
    if (ret != 0) {
      std::cout << "jpeg cpu decoder(disable gpu) failed: ret = " << ret
                << std::endl;
      return -3;
    }
  }
#endif // defined(ENABLE_USE_CUDA)
  return 0;
}

int JpegCoder::init() {
  int ret = 0;
  switch (jpeg_codes_config_.coderType) {
  case JPEG_CODER_TYPE::RS_JPEG_CODER_ENCODE: {
    ret = initEncoder();
    break;
  }
  case JPEG_CODER_TYPE::RS_JPEG_CODER_DECODE: {
    ret = initDecoder();
    break;
  }
  }

  return ret;
}

int JpegCoder::initEncoder() {
#if defined(ENABLE_USE_CUDA)
  // 先初始化GPU Encoder, 如果失败再初始化Opencv Encoder
  try {
    gpu_jpeg_encoder_ptr_.reset(new GpuJpegEncoder());
  } catch (...) {
    return -1;
  }
  int ret = gpu_jpeg_encoder_ptr_->init(
      jpeg_codes_config_.imageWidth, jpeg_codes_config_.imageHeight,
      jpeg_codes_config_.jpegQuality,
      jpeg_codes_config_.codecColorSpaceTypeHelper(),
      jpeg_codes_config_.encodeSampleTypHelper(),
      jpeg_codes_config_.gpuDeviceId);
  if (ret != 0) {
    try {
      opencv_jpeg_encoder_ptr_.reset(new OpencvJpegEncoder());
    } catch (...) {
      return -2;
    }
    ret = opencv_jpeg_encoder_ptr_->init(
        jpeg_codes_config_.imageWidth, jpeg_codes_config_.imageHeight,
        jpeg_codes_config_.jpegQuality,
        jpeg_codes_config_.codecColorSpaceTypeHelper(),
        jpeg_codes_config_.encodeSampleTypHelper());

    if (ret != 0) {
      return -3;
    }
  }
#else
  try {
    opencv_jpeg_encoder_ptr_.reset(new OpencvJpegEncoder());
  } catch (...) {
    return -1;
  }
  int ret = opencv_jpeg_encoder_ptr_->init(
      jpeg_codes_config_.imageWidth, jpeg_codes_config_.imageHeight,
      jpeg_codes_config_.jpegQuality,
      jpeg_codes_config_.codecColorSpaceTypeHelper(),
      jpeg_codes_config_.encodeSampleTypHelper());

  if (ret != 0) {
    return -1;
  }
#endif // defined(ENABLE_USE_CUDA)
  return 0;
}

int JpegCoder::initDecoder() {
#if defined(ENABLE_USE_CUDA)
  // 先初始化GPU Decoder, 如果失败再初始化Opencv Decoder
  try {
    gpu_jpeg_decoder_ptr_.reset(new GpuJpegDecoder());
  } catch (...) {
    return -1;
  }

  int ret = gpu_jpeg_decoder_ptr_->init(
      jpeg_codes_config_.imageWidth, jpeg_codes_config_.imageHeight,
      jpeg_codes_config_.codecColorSpaceTypeHelper(),
      jpeg_codes_config_.gpuDeviceId);
  if (ret != 0) {
    try {
      opencv_jpeg_decoder_ptr_.reset(new OpencvJpegDecoder());
    } catch (...) {
      return -2;
    }

    ret = opencv_jpeg_decoder_ptr_->init(
        jpeg_codes_config_.imageWidth, jpeg_codes_config_.imageHeight,
        jpeg_codes_config_.codecColorSpaceTypeHelper());
    if (ret != 0) {
      return -3;
    }
  }
#else
  try {
    opencv_jpeg_decoder_ptr_.reset(new OpencvJpegDecoder());
  } catch (...) {
    return -4;
  }

  int ret = opencv_jpeg_decoder_ptr_->init(
      jpeg_codes_config_.imageWidth, jpeg_codes_config_.imageHeight,
      jpeg_codes_config_.codecColorSpaceTypeHelper());
  if (ret != 0) {
    return -5;
  }
#endif // defined(ENABLE_USE_CUDA)

  return 0;
}

} // namespace jpeg
} // namespace robosense