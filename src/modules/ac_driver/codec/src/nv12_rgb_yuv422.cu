/******************************************************************************
 * Copyright 2017 RoboSense All rights reserved.
 * Suteng Innovation Technology Co., Ltd. www.robosense.ai

 * This software is provided to you directly by RoboSense and might
 * only be used to access RoboSense LiDAR. Any compilation,
 * modification, exploration, reproduction and redistribution are
 * restricted without RoboSense's prior consent.

 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESSED OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL ROBOSENSE BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/

#include "hyper_vision/codec/nv12_rgb_yuv422.h"

// //超清公式
// #define RGB2Y(R, G, B)  ( 16  + 0.183f * (R) + 0.614f * (G) + 0.062f * (B) )
// #define RGB2U(R, G, B)  ( 128 - 0.101f * (R) - 0.339f * (G) + 0.439f * (B) )
// #define RGB2V(R, G, B)  ( 128 + 0.439f * (R) - 0.399f * (G) - 0.040f * (B) )

// #define YUV2R(Y, U, V) ( 1.164f *((Y) - 16) + 1.792f * ((V) - 128) )
// #define YUV2G(Y, U, V) ( 1.164f *((Y) - 16) - 0.213f *((U) - 128) - 0.534f
// *((V) - 128) ) #define YUV2B(Y, U, V) ( 1.164f *((Y) - 16) + 2.114f *((U) -
// 128))

// BT601 Full Range
#define RGB2Y(R, G, B) (0.299f * (R) + 0.587f * (G) + 0.114f * (B))
#define RGB2U(R, G, B) (-0.1687f * (R) - 0.3313f * (G) + 0.500f * (B) + 128)
#define RGB2V(R, G, B) (0.500f * (R) - 0.4187f * (G) - 0.0813f * (B) + 128)

template <typename T>
struct vec_assert_false : std::false_type
{
};

template <class T>
struct vecTypeInfo;
#define BaseType typename vecTypeInfo<T>::Base
template <>
struct vecTypeInfo<uchar6>
{
  typedef uint8_t Base;
};

template <typename T>
inline __host__ __device__ T make_vec(BaseType x0, BaseType y0, BaseType z0,
                                      BaseType w0, BaseType x1, BaseType y1,
                                      BaseType z1, BaseType w1)
{
  static_assert(vec_assert_false<T>::value,
                "invalid vector type - supported types are uchar6, uchar8, "
                "float6, float8");
}

template <>
inline __host__ __device__ uchar6 make_vec(uint8_t x0, uint8_t y0, uint8_t z0,
                                           uint8_t w0, uint8_t x1, uint8_t y1,
                                           uint8_t z1, uint8_t w1)
{
  return {x0, y0, z0, x1, y1, z1};
}

inline __device__ __host__ int iDivUp(int a, int b)
{
  return (a % b != 0) ? (a / b + 1) : (a / b);
}

#define CLIPVALUE(x, minValue, maxValue) \
  ((x) < (minValue) ? (minValue) : ((x) > (maxValue) ? (maxValue) : (x)))

static inline __device__ float clamp(float x)
{
  return fminf(fmaxf(x, 0.0f), 255.0f);
}

static inline __device__ float3 YUV2RGB(float Y, float U, float V)
{
  U -= 128.0f;
  V -= 128.0f;

  // BT601 Full Range
  return make_float3(clamp(Y + 1.402f * V), clamp(Y - 0.3441f * U - 0.7141f * V),
                     clamp(Y + 1.772f * U));
}

__global__ static void cudaYUYVToRGB(uchar4 *src, uchar6 *dst, const int halfWidth,
                                     const int height) // size不能使用引用
{
  const int x = blockIdx.x * blockDim.x + threadIdx.x;
  const int y = blockIdx.y * blockDim.y + threadIdx.y;
  if (x >= halfWidth || y >= height)
    return;

  const uchar4 macroPx = src[y * halfWidth + x];

  // Y0 is the brightness of pixel 0, Y1 the brightness of pixel 1.
  // U and V is the color of both pixels.
  float y0, y1, u, v;

  // YUYV [ Y0 | U0 | Y1 | V0 ]
  y0 = macroPx.x;
  y1 = macroPx.z;
  u = macroPx.y;
  v = macroPx.w;

  // this function outputs two pixels from one YUYV macropixel
  const float3 px0 = YUV2RGB(y0, u, v);
  const float3 px1 = YUV2RGB(y1, u, v);

  dst[y * halfWidth + x] =
      make_vec<uchar6>(px0.x, px0.y, px0.z, 255, px1.x, px1.y, px1.z, 255);
}

__global__ static void cudaNV12ToRGB(unsigned char *src, unsigned char *dst,
                                            const int width, const int height)
{
  const int x = blockIdx.x * blockDim.x + threadIdx.x;
  const int y = blockIdx.y * blockDim.y + threadIdx.y;
  if (x >= width || y >= height)
    return;

  const int size = width * height;
  const int y_index = y * width + x;
  const int uv_index = (y / 2 * width / 2 + (x / 2)) * 2;

  float Y = src[y_index];
  float U = src[size + uv_index];
  float V = src[size + uv_index + 1];

  // this function outputs two pixels from one YUYV macropixel
  const float3 px0 = YUV2RGB(Y, U, V);

  const int output_index = y_index * 3;
  dst[output_index] = (unsigned int)(px0.x);
  dst[output_index + 1] = (unsigned int)(px0.y);
  dst[output_index + 2] = (unsigned int)(px0.z);
}

__global__ static void cudaYUV420PToRGB(unsigned char *src, unsigned char *dst,
                                        const int width, const int height)
{
  const int x = blockIdx.x * blockDim.x + threadIdx.x;
  const int y = blockIdx.y * blockDim.y + threadIdx.y;
  if (x >= width || y >= height)
    return;

  const int size = width * height;
  const int y_index = y * width + x;
  const int u_index = size + y / 2 * width / 2 + (x / 2);
  const int v_index = size * 5 / 4 + y / 2 * width / 2 + (x / 2);

  float Y = src[y_index];
  float U = src[u_index];
  float V = src[v_index];

  // this function outputs two pixels from one YUYV macropixel
  const float3 px0 = YUV2RGB(Y, U, V);

  const int output_index = y_index * 3;
  dst[output_index] = (unsigned int)(px0.x);
  dst[output_index + 1] = (unsigned int)(px0.y);
  dst[output_index + 2] = (unsigned int)(px0.z);
}

// yyyyyyy uu vv
__global__ static void cudaRGBToYUV420P(const unsigned char *dpRgbData,
                                        size_t rgbPitch,
                                        unsigned char *dpYuv420pData, int width,
                                        int height)
{
  const int w = blockIdx.x * blockDim.x + threadIdx.x;
  const int h = blockIdx.y * blockDim.y + threadIdx.y;
  if (w >= width || h >= height)
    return;

  unsigned char *dp_y_data = dpYuv420pData;
  unsigned char *dp_u_data = dp_y_data + height * width;
  unsigned char *dp_v_data = dp_u_data + height * width / 4;

  unsigned char r = dpRgbData[h * rgbPitch + w * 3 + 0];
  unsigned char g = dpRgbData[h * rgbPitch + w * 3 + 1];
  unsigned char b = dpRgbData[h * rgbPitch + w * 3 + 2];

  dp_y_data[h * width + w] = (unsigned char)(CLIPVALUE(RGB2Y(r, g, b), 0, 255));

  if (h % 2 == 0 && w % 2 == 0)
  {
    int num = h / 2 * width / 2 + w / 2;
    dp_u_data[num] = (unsigned char)(CLIPVALUE(RGB2U(r, g, b), 0, 255));
    dp_v_data[num] = (unsigned char)(CLIPVALUE(RGB2V(r, g, b), 0, 255));
  }
}

// yyyyyyyy uv uv
__global__ static void cudaRGBToYUV422_NV12(const unsigned char *dpRgbData,
                                            size_t rgbPitch, unsigned char *dpNv12Data,
                                            int width, int height)
{
  const int w = blockIdx.x * blockDim.x + threadIdx.x;
  const int h = blockIdx.y * blockDim.y + threadIdx.y;

  if (w >= width || h >= height)
    return;

  unsigned char *dp_y_data = dpNv12Data;
  unsigned char *dp_uv_data = dpNv12Data + height * width;

  unsigned char r = dpRgbData[h * rgbPitch + w * 3 + 0];
  unsigned char g = dpRgbData[h * rgbPitch + w * 3 + 1];
  unsigned char b = dpRgbData[h * rgbPitch + w * 3 + 2];

  dp_y_data[h * width + w] = (unsigned char)CLIPVALUE(RGB2Y(r, g, b), 0, 255);

  if (h % 2 == 0 && w % 2 == 0)
  {
    int num = (h / 2 * width / 2 + w / 2) * 2;
    dp_uv_data[num] = (unsigned char)(CLIPVALUE(RGB2U(r, g, b), 0, 255));
    dp_uv_data[num + 1] = (unsigned char)(CLIPVALUE(RGB2V(r, g, b), 0, 255));
  }
}

// yuyv yuyv yuyv yuyv
__global__ static void cudaRGBToYUYV422(const unsigned char *dpRgbData,
                                        size_t rgbPitch,
                                        unsigned char *dpYuv422pData, int width,
                                        int height)
{
  const int w = blockIdx.x * blockDim.x + threadIdx.x;
  const int h = blockIdx.y * blockDim.y + threadIdx.y;

  if (w >= width || h >= height)
    return;

  unsigned char r = dpRgbData[h * rgbPitch + w * 3 + 0];
  unsigned char g = dpRgbData[h * rgbPitch + w * 3 + 1];
  unsigned char b = dpRgbData[h * rgbPitch + w * 3 + 2];

  dpYuv422pData[(h * width + w) * 2] =
      (unsigned char)CLIPVALUE(RGB2Y(r, g, b), 0, 255);

  if (w % 2 == 0)
  {
    dpYuv422pData[(h * width + w) * 2 + 1] =
        (unsigned char)(CLIPVALUE(RGB2U(r, g, b), 0, 255));
    dpYuv422pData[(h * width + w) * 2 + 3] =
        (unsigned char)(CLIPVALUE(RGB2V(r, g, b), 0, 255));
  }
}

void CUDA_YUYV422ToRGB(void *cuda_in, void *cuda_out, void *output, size_t width,
                       size_t height)
{
  const int halfWidth = width;    // two pixels are output at once
  const int half_height = height; // two pixels are output at once
  const dim3 blockDim(16, 16);
  const dim3 gridDim(iDivUp(halfWidth, blockDim.x),
                     iDivUp(half_height, blockDim.y));
  cudaYUYVToRGB<<<gridDim, blockDim>>>((uchar4 *)cuda_in, (uchar6 *)cuda_out,
                                       width, height);
  cudaDeviceSynchronize();
  cudaMemcpy(output, cuda_out, width * height * 3, cudaMemcpyDeviceToDevice);
}

void CUDA_NV12ToRGB(void *cuda_in, void *cuda_out, void *output,
                           size_t width, size_t height)
{
  const dim3 blockDim(16, 16);
  const dim3 gridDim(iDivUp(width, blockDim.x), iDivUp(height, blockDim.y));
  cudaNV12ToRGB<<<gridDim, blockDim>>>(
      (unsigned char *)cuda_in, (unsigned char *)cuda_out, width, height);
  cudaDeviceSynchronize();
  cudaMemcpy(output, cuda_out, width * height * 3, cudaMemcpyDeviceToDevice);
}

void CUDA_YUV420PToRGB(void *cuda_in, void *cuda_out, void *output,
                       size_t width, size_t height)
{
  const dim3 blockDim(16, 16);
  const dim3 gridDim(iDivUp(width, blockDim.x), iDivUp(height, blockDim.y));
  cudaYUV420PToRGB<<<gridDim, blockDim>>>(
      (unsigned char *)cuda_in, (unsigned char *)cuda_out, width, height);
  cudaDeviceSynchronize();
  cudaMemcpy(output, cuda_out, width * height * 3, cudaMemcpyDeviceToDevice);
}

void CUDA_RGBToNV12(void *cuda_in, void *cuda_out, void *output,
                           size_t width, size_t height)
{
  const dim3 blockDim(16, 16);
  const dim3 gridDim(iDivUp(width, blockDim.x), iDivUp(height, blockDim.y));
  size_t rgbPitch = width * 3;

  cudaRGBToYUV422_NV12<<<gridDim, blockDim>>>((const unsigned char *)cuda_in, rgbPitch,
                                              (unsigned char *)cuda_out, width, height);

  cudaDeviceSynchronize();

  cudaMemcpy(output, cuda_out, width * height * 1.5, cudaMemcpyDeviceToHost);
}

void CUDA_RGBToYUYV422(void *cuda_in, void *cuda_out, void *output, size_t width,
                       size_t height)
{
  const dim3 blockDim(16, 16);
  const dim3 gridDim(iDivUp(width, blockDim.x), iDivUp(height, blockDim.y));
  size_t rgbPitch = width * 3;

  cudaRGBToYUYV422<<<gridDim, blockDim>>>((const unsigned char *)cuda_in, rgbPitch,
                                          (unsigned char *)cuda_out, width, height);

  cudaDeviceSynchronize();

  cudaMemcpy(output, cuda_out, width * height * 2, cudaMemcpyDeviceToHost);
}

void CUDA_RGBToYUV420P(void *cuda_in, void *cuda_out, void *output, size_t width,
                      size_t height)
{
  const dim3 blockDim(16, 16);
  const dim3 gridDim(iDivUp(width, blockDim.x), iDivUp(height, blockDim.y));
  size_t rgbPitch = width * 3;

  cudaRGBToYUV420P<<<gridDim, blockDim>>>((const unsigned char *)cuda_in, rgbPitch,
                                          (unsigned char *)cuda_out, width, height);

  cudaDeviceSynchronize();

  cudaMemcpy(output, cuda_out, width * height * 1.5, cudaMemcpyDeviceToHost);
}

void CUDA_NV12ToRgbToYUV422(void *nv12_image, void *cuda_out,
                                   void *rgb_image, void *yuv422_image,
                                   void *output, size_t width, size_t height)
{
  CUDA_NV12ToRGB(nv12_image, cuda_out, rgb_image, width, height);
  CUDA_RGBToYUYV422(rgb_image, yuv422_image, output, width, height);
}

void CUDA_YUYV422ToRgbToNV12(void *yuv422_image, void *cuda_out,
                                    void *rgb_image, void *nv12_image, void *output,
                                    size_t width, size_t height)
{
  CUDA_YUYV422ToRGB(yuv422_image, cuda_out, rgb_image, width, height);
  CUDA_RGBToNV12(rgb_image, nv12_image, output, width, height);
}

void CUDA_YUV420PToRgbToYUYV422(void *yuv420p_image, void *cuda_out,
                               void *rgb_image, void *yuv422_image,
                               void *output, size_t width, size_t height)
{
  CUDA_YUV420PToRGB(yuv420p_image, cuda_out, rgb_image, width, height);
  CUDA_RGBToYUYV422(rgb_image, yuv422_image, output, width, height);
}

void CUDA_YUYV422ToRgbToYUV420P(void *yuv422_image, void *cuda_out,
                                void *rgb_image, void *yuv420p_image, void *output,
                                size_t width, size_t height)
{
  CUDA_YUYV422ToRGB(yuv422_image, cuda_out, rgb_image, width, height);
  CUDA_RGBToYUV420P(rgb_image, yuv420p_image, output, width, height);
}