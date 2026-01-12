#ifndef JPEGCODER_H
#define JPEGCODER_H
#if defined(ENABLE_USE_CUDA)
#include "hyper_vision/codec/gpujpegdecoder.h"
#include "hyper_vision/codec/gpujpegencoder.h"
#endif // defined(ENABLE_USE_CUDA)
#include "hyper_vision/codec/opencvjpegdecoder.h"
#include "hyper_vision/codec/opencvjpegencoder.h"

namespace robosense {
namespace jpeg {

class JpegCoder {
public:
  using Ptr = std::shared_ptr<JpegCoder>;
  using ConstPtr = std::shared_ptr<const JpegCoder>;

public:
  JpegCoder();
  ~JpegCoder();

public:
  int init(const JpegCodesConfig &jpegCodesConfig);

  int encode(unsigned char *yuv420Buffer, int yuv420BufferLen,
             unsigned char *jpegBuffer, size_t &jpegBufferLen);

  int decode(unsigned char *yuv420Buffer, int yuv420BufferLen,
             unsigned char *jpegBuffer, size_t &jpegBufferLen);

private:
  int init();
  int initEncoder();
  int initDecoder();

private:
#if defined(ENABLE_USE_CUDA)
  GpuJpegEncoder::Ptr gpu_jpeg_encoder_ptr_;
  GpuJpegDecoder::Ptr gpu_jpeg_decoder_ptr_;
#endif // defined(ENABLE_USE_CUDA)
  OpencvJpegEncoder::Ptr opencv_jpeg_encoder_ptr_;
  OpencvJpegDecoder::Ptr opencv_jpeg_decoder_ptr_;
  JpegCodesConfig jpeg_codes_config_;
};

} // namespace jpeg
} // namespace robosense

#endif // JPEGCODER_H