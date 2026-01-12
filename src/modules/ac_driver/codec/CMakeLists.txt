cmake_minimum_required(VERSION 3.5)

#========================
# Project
#========================

set(CUR_LIB codec)
project(${CUR_LIB})

set(CMAKE_BUILD_TYPE Release)
add_definitions(-std=c++17)
add_compile_options(-std=c++17)
add_definitions(-DDEBUG_LEVEL=0)

include(cmake/srcs.cmake)

set(${CUR_LIB} PARENT_SCOPE)


