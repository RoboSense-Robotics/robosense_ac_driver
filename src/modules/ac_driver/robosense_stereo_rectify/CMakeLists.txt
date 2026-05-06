cmake_minimum_required(VERSION 3.16)

project(robosense_stereo_rectify LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(BUILD_EXAMPLES "Build example executables" ON)

find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

add_library(stereo_rectify_core
    src/stereo_rectifier.cpp
)

target_include_directories(stereo_rectify_core
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(stereo_rectify_core
    PUBLIC
        ${OpenCV_LIBS}
)

target_include_directories(stereo_rectify_core
    PUBLIC
        ${OpenCV_INCLUDE_DIRS}
)

if(BUILD_EXAMPLES)
    add_executable(stereo_rectify_demo
        demo/example.cpp
    )

    target_include_directories(stereo_rectify_demo
        PRIVATE
            ${OpenCV_INCLUDE_DIRS}
            ${YAML_CPP_INCLUDE_DIRS}
    )

    target_link_directories(stereo_rectify_demo
        PRIVATE
            ${YAML_CPP_LIBRARY_DIRS}
    )

    target_link_libraries(stereo_rectify_demo
        PRIVATE
            stereo_rectify_core
            ${OpenCV_LIBS}
            ${YAML_CPP_LIBRARIES}
    )

    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/config/1111BEDC0028_calibration.yaml")
        file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/config")
        configure_file(
            "${CMAKE_CURRENT_SOURCE_DIR}/config/1111BEDC0028_calibration.yaml"
            "${CMAKE_CURRENT_BINARY_DIR}/config/1111BEDC0028_calibration.yaml"
            COPYONLY
        )
    endif()
endif()
