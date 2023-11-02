#pragma once


#include "Eigen/Core"
#include "Eigen/Dense"

typedef struct Rect_struct {
  double a[5], b[5], c[5];
  double corner[5][2];  // 第一个点是最低点，然后逆时针旋转
  double length, width, theta;  // theta ∈ [-90, +90];
} Rect_t;


Rect_t L_shape_Fit_Proc(const Eigen::MatrixXd point, const double theta_);