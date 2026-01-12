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

#ifndef NV12_RGB_YUV422_H
#define NV12_RGB_YUV422_H

#include <device_launch_parameters.h>
#include <stdio.h>

#include <iostream>

#include <cuda.h>
#include <cuda_runtime.h>
#include <cuda_runtime_api.h>

// CHECK_CUDA
#define CHECK_CUDA(call)                                                       \
  {                                                                            \
    cudaError_t _e = (call);                                                   \
    if (_e != cudaSuccess) {                                                   \
      std::cout << "CUDA Runtime failure: '#" << _e << "' at " << __FILE__     \
                << ":" << __LINE__ << std::endl;                               \
      exit(1);                                                                 \
    }                                                                          \
  }
  
struct /*__align__(6)*/ uchar6 {
  uint8_t x0, y0, z0, x1, y1, z1;
};

//
// 8 个点的各种格式表示
// RGB        : rgb rgb rgb rgb rgb rgb rgb rgb
// YUYV422    : yuyv yuyv yuyv yuyv
// NV12       : yyyyyyyy uv uv
// YUV420P    : yyyyyyyy uu vv

void CUDA_YUYV422ToRGB(void *cuda_in, void *cuda_out, void *output,
                       size_t width, size_t height);

void CUDA_NV12ToRGB(void *cuda_in, void *cuda_out, void *output, size_t width,
                    size_t height);

void CUDA_YUV420PToRGB(void *cuda_in, void *cuda_out, void *output,
                       size_t width, size_t height);

void CUDA_RGBToNV12(void *cuda_in, void *cuda_out, void *output, size_t width,
                    size_t height);

void CUDA_RGBToYUYV422(void *cuda_in, void *cuda_out, void *output,
                       size_t width, size_t height);

void CUDA_RGBToYUV420P(void *cuda_in, void *cuda_out, void *output, size_t width,
                      size_t height);

void CUDA_NV12ToRgbToYUV422(void *nv12_image, void *cuda_out, void *rgb_image,
                            void *yuv422_image, void *output, size_t width,
                            size_t height);

void CUDA_YUYV422ToRgbToNV12(void *yuv422_image, void *cuda_out,
                             void *rgb_image, void *nv12_image, void *output,
                             size_t width, size_t height);

void CUDA_YUV420PToRgbToYUYV422(void *yuv420p_image, void *cuda_out,
                               void *rgb_image, void *yuv422_image,
                               void *output, size_t width, size_t height);

void CUDA_YUYV422ToRgbToYUV420P(void *yuv422_image, void *cuda_out,
                                void *rgb_image, void *yuv420p_image,
                                void *output, size_t width, size_t height);

#endif // NV12_RGB_YUV422_H
