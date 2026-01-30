cmake_minimum_required(VERSION 3.5)
project(ac_driver)

## Compile as C++14, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)
set(CMAKE_BUILD_TYPE "Release")

add_compile_options(
    -O3
    # -ffast-math
    # -flto
    # -fexceptions
    # -pthread
)
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g -ggdb")
set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(arm|aarch64)$")
  file(READ "/proc/device-tree/model" COMPATIBLE_CONTENT)
  execute_process(
    COMMAND bash -c "cat /proc/cpuinfo | grep 'Rockchip RK3588' | uniq | cut -d ':' -f 2"
    OUTPUT_VARIABLE CPU_MODEL
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )
  if(COMPATIBLE_CONTENT MATCHES "Jetson")
    message(STATUS "Detected Orin platform")
    set(TARGET_ARCH "JETSON_ORIN")
    add_definitions(-DJETSON_ORIN)
  elseif(CPU_MODEL STREQUAL " Rockchip RK3588")
    message(STATUS "Detected RK3588 chip")
    set(TARGET_ARCH "RK3588")
    add_definitions(-DRK3588)
  else()
    message(STATUS "cur model: ${COMPATIBLE_CONTENT}")
    set(TARGET_ARCH "ARM")
    add_definitions(-DARM_ARCH)
  endif()
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|i686)$")
  set(TARGET_ARCH "X86")
  add_definitions(-DX86_ARCH)
else()
  message(STATUS "Unknown target architecture: ${CMAKE_SYSTEM_PROCESSOR}")
  set(TARGET_ARCH "UNKNOWN")
endif()


#av_codec#
find_library(LZMA_LIBRARY lzma)
find_library(M_LIBRARY m)
find_library(Z_LIBRARY z)

# detect ros version
if(DEFINED ENV{ROS_VERSION})
  set(ROS_VERSION $ENV{ROS_VERSION})
else()
  find_package(catkin QUIET)
  find_package(ament_cmake QUIET)

  if(catkin_FOUND)
    set(ROS_VERSION 1)
  elseif(ament_cmake_FOUND)
    set(ROS_VERSION 2)
  else()
    message(FATAL_ERROR "Can not detect ROS version automatically")
  endif()
endif()
message(STATUS "Building for ROS${ROS_VERSION}")
if(ROS_VERSION EQUAL 1)

  find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  robosense_msgs
  )
  add_definitions(-DROS_FOUND)

  catkin_package(
    CATKIN_DEPENDS roscpp std_msgs sensor_msgs robosense_msgs
  )

  # Specify additional locations of header files
  include_directories(${catkin_INCLUDE_DIRS})
elseif(ROS_VERSION EQUAL 2)
  find_package(rclcpp REQUIRED)
  find_package(ament_cmake REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(robosense_msgs REQUIRED)
  add_definitions(-DROS2_FOUND)
else()
  message(=============================================================)
  message(FATAL_STATUS "-- ROS/ROS2 Not Found ---")
  message(=============================================================)
  return() 
endif()

#
# rs_driver 
#
add_definitions(-DENABLE_USB)
add_definitions(-DDISABLE_PCAP_PARSE) 
add_definitions(-DENABLE_GMSL) 
add_definitions(-DENABLE_IMAGE) 
set(ENABLE_SUPPORT_ALGORITHM FALSE)
if(ENABLE_SUPPORT_ALGORITHM)
  add_definitions(-DENABLE_SUPPORT_RS_DRIVER_ALGORITHM) 
  message("enable support rs_driver algorithm")
else() 
  message("disable support rs_driver algorithm")
endif() 

set(ENABLE_SIMD_ACCELERATION TRUE)
if(ENABLE_SIMD_ACCELERATION)
  add_compile_definitions(ENABLE_SIMD_ACCELERATION)
  message(STATUS "SIMD acceleration enabled")
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86" OR CMAKE_SYSTEM_PROCESSOR MATCHES "amd64")
      set(X86_PLATFORM 1)
      message(STATUS "Building for x86 platform")
      
      include(CheckCXXCompilerFlag)
      
      check_cxx_compiler_flag("-mssse3" COMPILER_SUPPORTS_SSSE3)
      check_cxx_compiler_flag("-mavx2" COMPILER_SUPPORTS_AVX2)
      if(COMPILER_SUPPORTS_SSSE3)
          add_compile_definitions(ENABLE_SSSE3)
          set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mssse3")
          message(STATUS "SSSE3 optimization enabled")
      elseif(COMPILER_SUPPORTS_AVX2)
          add_compile_definitions(ENABLE_AVX2)
          set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mavx2")
          message(STATUS "AVX2 optimization enabled")
      endif()
      
  elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64" OR CMAKE_SYSTEM_PROCESSOR MATCHES "arm64")
      set(ARM64_PLATFORM 1)
      add_compile_definitions(ENABLE_ARM_NEON ENABLE_ARM64)
      message(STATUS "Building for ARM64 platform with NEON optimization")
      
  elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "arm")
      set(ARM_PLATFORM 1)
      
      include(CheckCXXCompilerFlag)
      check_cxx_compiler_flag("-mfpu=neon" COMPILER_SUPPORTS_NEON)
      
      if(COMPILER_SUPPORTS_NEON)
          add_compile_definitions(ENABLE_ARM_NEON)
          set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mfpu=neon")
          message(STATUS "ARM NEON optimization enabled")
      endif()
      
  else()
      message(STATUS "Building for generic platform")
  endif()
else()
  message(STATUS "SIMD acceleration disabled, using generic implementation")
endif()

# PCL # 
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# OpenCV 
find_package       (OpenCV REQUIRED)
include_directories("/usr/include/opencv4")
include_directories(${Opencv_INCLUDE_DIRS})
link_directories   (${Opencv_LIBRARY_DIRS}) 
set(OPENCV_LIBS  opencv_core opencv_imgcodecs opencv_imgproc opencv_calib3d) 

#Eigen 
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

find_package(yaml-cpp REQUIRED)

#
# CUDA 
#
set(enable_use_cuda_gpu true)
if(enable_use_cuda_gpu)
    find_package(CUDA QUIET)
    if(CUDA_FOUND)
        add_definitions(-DENABLE_USE_CUDA)
        message(STATUS "CUDA found - version: ${CUDA_VERSION}")
        include_directories(${CUDA_INCLUDE_DIRS})
    endif(CUDA_FOUND)  
else() 
     message("disable gpu !")
endif(enable_use_cuda_gpu) 

#
# RK3588: RGA 
#
include_directories(${THIRD_PARTY_DIR}/rga) 

#
# spdlog 
#
include_directories(${THIRD_PARTY_DIR}/spdlog)

#
# devicemanager 
# 
include_directories(${PROJECT_SOURCE_DIR}/devicemanager) 
include_directories(${PROJECT_SOURCE_DIR}/devicemanager/include/)
add_subdirectory(devicemanager)

#
# codec 
# 
include_directories(${PROJECT_SOURCE_DIR}/codec) 
include_directories(${PROJECT_SOURCE_DIR}/codec/include/)
add_subdirectory(codec)

#
# logmanager 
# 
set(enable_spdlog_use_full_file_name false)
if(enable_spdlog_use_full_file_name)
  message("enable spdlog use full file name !")
  add_definitions(-DENABLE_SPDLOG_USE_FULL_FILE_NAME)
elseif() 
  message("disable spdlog use full file name !")
endif(enable_spdlog_use_full_file_name)
set(enable_spdlog_get_cpu_num true)
if(enable_spdlog_get_cpu_num)
  message("enable spdlog get cpu number !")
  add_definitions(-DENABLE_SPDLOG_GET_CPU_NUM)
elseif(enable_spdlog_get_cpu_num)
  message("disable spdlog get cpu number !") 
endif(enable_spdlog_get_cpu_num)
include_directories(${PROJECT_SOURCE_DIR}/logmanager) 
include_directories(${PROJECT_SOURCE_DIR}/logmanager/include/)
add_subdirectory(logmanager)

#Ros2#
add_executable(ms_node src/metaS_node.cpp)
target_link_libraries(ms_node ${SS_LIBRARY} ${YAML_CPP_LIBRARIES})
if(ROS_VERSION EQUAL 1)
  target_link_libraries(ms_node ${catkin_LIBRARIES}) 
elseif(ROS_VERSION EQUAL 2)
  ament_target_dependencies(ms_node rclcpp sensor_msgs robosense_msgs)
endif() 
target_link_libraries(ms_node ${LZMA_LIBRARY})
target_link_libraries(ms_node ${M_LIBRARY})
target_link_libraries(ms_node ${Z_LIBRARY})
target_link_libraries(ms_node ${PCL_LIBRARIES})
target_link_libraries(ms_node device codec log)
if(TARGET_ARCH STREQUAL "RK3588")
  find_library(NUMA_LIBRARY numa)
  target_link_libraries(ms_node ${NUMA_LIBRARY})
  find_library(MPP_LIBRARY rockchip_mpp)
  target_link_libraries(ms_node ${MPP_LIBRARY})
  find_library(RGA_LIBRARY rga)
  target_link_libraries(ms_node ${RGA_LIBRARY})
  find_library(DRM_LIBRARY drm)
  target_link_libraries(ms_node ${DRM_LIBRARY})
  find_library(BZ2_LIBRARY bz2)
  target_link_libraries(ms_node ${BZ2_LIBRARY})
  find_library(XCB_LIBRARY xcb)
  target_link_libraries(ms_node ${XCB_LIBRARY})

  find_library(ASOUND_LIBRARY asound REQUIRED)
  find_library(SNDIO_LIBRARY sndio REQUIRED)
  find_library(X11_LIBRARY NAMES X11)
  find_library(XV_LIBRARY NAMES Xv)
  find_library(XEXT_LIBRARY NAMES Xext)
  target_link_libraries(ms_node
    ${ASOUND_LIBRARY}
    ${SNDIO_LIBRARY}
    ${X11_LIBRARY}
    ${XEXT_LIBRARY}
    ${XV_LIBRARY}
  )
else() 
    target_link_libraries(ms_node ${OPENCV_LIBS})
endif()

if(ROS_VERSION EQUAL 1)
  install(TARGETS ms_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

  install(DIRECTORY conf launch rviz
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

elseif(ROS_VERSION EQUAL 2)
  install(TARGETS ms_node
    DESTINATION lib/${PROJECT_NAME}
  )

  install(DIRECTORY conf launch rviz log 
    DESTINATION share/${PROJECT_NAME}
  )
  ament_package()
endif()