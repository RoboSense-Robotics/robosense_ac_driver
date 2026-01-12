#ifndef GPUJPEGENCODER_H
#define GPUJPEGENCODER_H

#include "hyper_vision/codec/jpegcommon.h"
#if defined(ENABLE_USE_CUDA)
namespace robosense {
namespace jpeg {

class GpuJpegEncoder {
public:
  using Ptr = std::shared_ptr<GpuJpegEncoder>;
  using ConstPtr = std::shared_ptr<const GpuJpegEncoder>;

public:
  GpuJpegEncoder();
  ~GpuJpegEncoder();

public:
  int init(const int imageWidth, const int imageHeight, const int jpegQuality,
           const RS_GPU_JPEG_ENCODE_SUPPORT_TYPE yuv420_type =
               RS_GPU_JPEG_ENCODE_SUPPORT_TYPE::RS_GPU_JPEG_SUPPORT_NV12,
           const RS_GPU_JPEG_ENCODE_SAMPLE_TYPE sampleType =
               RS_GPU_JPEG_ENCODE_SAMPLE_TYPE::RS_GPU_JPEG_ENCODE_SAMPLE_FULL,
           const int gpuDeviceId = 0);

  int encode(unsigned char *yuv420Buffer, int yuv420BufferLen,
             unsigned char *&jpegBuffer, size_t &jpegBufferLen);

private:
  int init();

private:
  bool is_initial_;
  int gpu_device_id_;
  int image_width_;
  int image_height_;
  int jpeg_quality_;
  gpujpeg_color_space color_space_;
  int yuv420_len_;
  int yuv420_y_len_;
  int yuv420_y_offset_;
  int yuv420_uv_len_;
  int yuv420_u_offset_;
  int yuv420_v_offset_;
  int yuv420_u_sample_offset_;
  int yuv420_v_sample_offset_;
  int rgb_len_;
  int yuyv_len_;
  RS_GPU_JPEG_ENCODE_SAMPLE_TYPE sample_type_;
  int expect_input_len_;
  RS_GPU_JPEG_ENCODE_SUPPORT_TYPE support_yuv420_type_;
  std::vector<unsigned char> yuv420p_buffer_;
  std::vector<unsigned char> yuv420p_sample_buffer_;
  std::vector<unsigned char> rgb_sample_buffer_;
  struct gpujpeg_parameters param_;
  struct gpujpeg_image_parameters param_image_;
  struct gpujpeg_encoder *encoder_;
};

} // namespace jpeg
} // namespace robosense

#endif // defined(ENABLE_USE_CUDA)

#endif // GPUJPEGENCODER_H