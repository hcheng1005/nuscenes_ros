/*
 * @Description:
 * @version:
 * @Author: ChengHao
 * @Date: 2022-10-12 08:58:47
 * @LastEditors: CharlesCH hcheng1005@gmail.com
 * @LastEditTime: 2023-11-09 20:20:14
 */
#pragma once

// Eigen
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include "basicKalman.h"
#include "randomMatrice.h"

using Eigen::MatrixXf;
using Eigen::VectorXf;

#define SSIZE 6
#define MSIZE 3

#define iDistLat 0
#define iDistLong 1
#define iVrelLat 2
#define iVrelLong 3
#define iAccLat 4
#define iAccLong 5

typedef enum {
  TRK_Invalid = 0,
  TRK_Detected,
  TRK_Confirmed,
  TRK_Delete
} trace_status_enum;

// 用于航迹管理（起始、消亡）
struct trace_manager_t {
  uint age;
  uint id;
  uint match_count = 0;
  uint unmatched_count = 0;

  trace_status_enum status;

  float prob;
};

struct trace_shape_t {
  float len;
  float wid;
  float theta;
};

class RadarTracker {
 public:
  RadarTracker(uint new_id, float center_lat, float center_long, float vr,
               float len, float wid, float theta);

  ~RadarTracker() { delete randomMatriceFilter; }

  void trace_predict();
  void trace_update_kinematic_measCenter(const VectorXf &Z);
  void trace_update_kinematic_measSet(const Eigen::MatrixXf &Z);
  void trace_update_physical(float new_len, float new_wid, float new_theta);
  void trace_management(bool matchedFlag);

 private:
 public:
  trace_manager_t trace_manager;
  trace_shape_t trace_shape;

  // 定义滤波器
  RandomMatriceFilter<float, SSIZE, MSIZE> *randomMatriceFilter = nullptr;
};
