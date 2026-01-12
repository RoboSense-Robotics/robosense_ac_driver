#ifndef GPUJPEGDECODER_H
#define GPUJPEGDECODER_H

#include "hyper_vision/codec/jpegcommon.h"

#if defined(ENABLE_USE_CUDA)
namespace robosense {
namespace jpeg {

class GpuJpegDecoder {
public:
  using Ptr = std::shared_ptr<GpuJpegDecoder>;
  using ConstPtr = std::shared_ptr<const GpuJpegDecoder>;

public:
  GpuJpegDecoder();
  ~GpuJpegDecoder();

public:
  int init(const int imageWidth, const int imageHeight,
           const RS_GPU_JPEG_ENCODE_SUPPORT_TYPE output_type =
               RS_GPU_JPEG_ENCODE_SUPPORT_TYPE::RS_GPU_JPEG_SUPPORT_RGB,
           const int gpuDeviceId = 0);

  int decode(unsigned char *jpegBuffer, int jpegBufferLen,
             unsigned char *rawBuffer, size_t &rawBufferLen);

private:
  int init();

private:
  bool is_initial_;
  int gpu_device_id_;
  int image_width_;
  int image_height_;
  RS_GPU_JPEG_ENCODE_SUPPORT_TYPE output_type_;
  gpujpeg_color_space color_space_;
  gpujpeg_pixel_format pixel_format_;
  struct gpujpeg_parameters param_;
  struct gpujpeg_image_parameters param_image_;
  struct gpujpeg_decoder *decoder_;
};

} // namespace jpeg
} // namespace robosense

#endif // defined(ENABLE_USE_CUDA)

#endif // GPUJPEGDECODER_H