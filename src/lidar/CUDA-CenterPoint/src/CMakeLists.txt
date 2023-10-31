# SPDX-FileCopyrightText: Copyright (c) 2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: MIT
#
# Permission is hereby granted, free of charge, to any person obtaining a
# copy of this software and associated documentation files (the "Software"),
# to deal in the Software without restriction, including without limitation
# the rights to use, copy, modify, merge, publish, distribute, sublicense,
# and/or sell copies of the Software, and to permit persons to whom the
# Software is furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
# DEALINGS IN THE SOFTWARE.

cmake_minimum_required(VERSION 3.8) # C++17 support was added in CMake 3.8
project(nus_centerpoint)
set(arch ${CMAKE_HOST_SYSTEM_PROCESSOR})

set(TENSORRT_ROOT         /data/package/TensorRT-8.2.0.6)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-std=c++14 -Wextra -Wall -Wno-deprecated-declarations -O3")
set(TENSORRT_INCLUDE ${TENSORRT_ROOT}/include)
set(TENSORRT_LIB     ${TENSORRT_ROOT}/lib)

find_package(CUDA REQUIRED)
if(${CMAKE_BUILD_TYPE} STREQUAL "Debug")
  message("Using Debug Mode")
  set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} -g -G --ptxas-options=-v)
endif()

include_directories(
  ${CUDA_INCLUDE_DIRS}
  ${TENSORRT_INCLUDE}
  3rdparty/libspconv/include
  include
)

find_package(catkin REQUIRED COMPONENTS roscpp)
find_package(OpenCV REQUIRED core)
find_package(PCL REQUIRED)

include_directories(src/include/ ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} )

link_directories(
  ${TENSORRT_LIB}
  ${catkin_LIBRARIES}
  3rdparty/libspconv/lib/${arch}
)

file(GLOB_RECURSE SOURCE_FILES 
  src/*.cu 
  src/*.cpp
)

find_package(catkin
             REQUIRED
             rosbag
             sensor_msgs
             pcl_conversions
             pcl_msgs
             message_generation
             geometry_msgs
             std_msgs
             nav_msgs
             tf)

catkin_package(INCLUDE_DIRS
               include
               DEPENDS)

cuda_add_executable(${PROJECT_NAME} ros_main.cpp ${SOURCE_FILES})

target_link_LIBraries(${PROJECT_NAME}
    libnvinfer.so
    libspconv.so
    ${PCL_COMMON_LIBRARY}
    ${PCL_IO_LIBRARY}
    ${catkin_LIBRARIES}
)