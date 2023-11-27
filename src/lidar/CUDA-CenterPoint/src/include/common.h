/*
 * SPDX-FileCopyrightText: Copyright (c) 2023 NVIDIA CORPORATION & AFFILIATES.
 * All rights reserved. SPDX-License-Identifier: MIT
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#ifndef COMMON_H_
#define COMMON_H_

#include <cuda.h>
#include <cuda_fp16.h>
#include <cuda_runtime.h>
#include <cuda_runtime_api.h>
#include <math.h>

#include <algorithm>
#include <iostream>
#include <numeric>
#include <string>
#include <unordered_map>
#include <vector>

const unsigned int MAX_DET_NUM = 1000;  // nms_pre_max_size = 1000;
const unsigned int DET_CHANNEL = 11;
const unsigned int MAX_POINTS_NUM = 300000;
const unsigned int NUM_TASKS = 6;

#define checkCudaErrors(op)                                       \
  {                                                               \
    auto status = ((op));                                         \
    if (status != 0) {                                            \
      std::cout << "Cuda failure: " << cudaGetErrorString(status) \
                << " in file " << __FILE__ << ":" << __LINE__     \
                << " error status: " << status << std::endl;      \
      abort();                                                    \
    }                                                             \
  }

class Params {
 public:
  const unsigned int task_num_stride[NUM_TASKS] = {0};
  static const unsigned int num_classes = 10;
  const char *class_name[num_classes] = {
      "car",        "truck",       "construction_vehicle", "bus",
      "trailer",    "barrier",     "motorcycle",           "bicycle",
      "pedestrian", "traffic_cone"};

  const float out_size_factor = 8;
  const float voxel_size[2] = {0.1, 0.1};
  const float pc_range[2] = {-51.2, -51.2};
  const float score_threshold = 0.3;
  const float post_center_range[6] = {
      -61.2, -61.2, -10.0, 61.2, 61.2, 10.0,
  };
  const float nms_iou_threshold = 0.7;
  const unsigned int nms_pre_max_size = MAX_DET_NUM;
  const unsigned int nms_post_max_size = 83;

  const float min_x_range = -51.2;
  const float max_x_range = 51.2;
  const float min_y_range = -51.2;
  const float max_y_range = 51.2;
  const float min_z_range = -5;
  const float max_z_range = 3.0;
  // the size of a pillar
  const float pillar_x_size = 0.1;
  const float pillar_y_size = 0.1;
  const float pillar_z_size = 0.2;
  const int max_points_per_voxel = 10;

  const unsigned int max_voxels = 160000;
  const unsigned int feature_num = 5;

  Params(){};

  int getGridXSize() {
    return (int)std::round((max_x_range - min_x_range) / pillar_x_size);
  }
  int getGridYSize() {
    return (int)std::round((max_y_range - min_y_range) / pillar_y_size);
  }
  int getGridZSize() {
    return (int)std::round((max_z_range - min_z_range) / pillar_z_size);
  }
};

/*
box_encodings: (B, N, 7 + C) or (N, 7 + C) [x, y, z, dx, dy, dz, heading or
*[cos, sin], ...] anchors: (B, N, 7 + C) or (N, 7 + C) [x, y, z, dx, dy, dz,
heading, ...]
*/
struct Bndbox {
  float x;
  float y;
  float z;
  float w;
  float l;
  float h;
  float vx;
  float vy;
  float rt;
  int id;
  float score;
  Bndbox(){};
  Bndbox(float x_, float y_, float z_, float w_, float l_, float h_, float vx_,
         float vy_, float rt_, int id_, float score_)
      : x(x_),
        y(y_),
        z(z_),
        w(w_),
        l(l_),
        h(h_),
        vx(vx_),
        vy(vy_),
        rt(rt_),
        id(id_),
        score(score_) {}
};

#endif