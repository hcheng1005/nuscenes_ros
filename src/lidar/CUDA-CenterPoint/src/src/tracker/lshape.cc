#include "../../include/tracker/lshape.h"

#include <algorithm>
#include <iostream>
#include <numeric>
#include <vector>

using namespace std;

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @param {vector<double>} vecNums
 * @return {*}
 */
std::pair<double, double> compute_variance(std::vector<double> vecNums) {
  std::pair<double, double> res;
  double sumNum = std::accumulate(vecNums.begin(), vecNums.end(), 0.0);
  double mean = sumNum / vecNums.size();  // 均值
  double accum = 0.0;
  for_each(vecNums.begin(), vecNums.end(),
           [&](const double d) { accum += (d - mean) * (d - mean); });
  double variance = accum / vecNums.size();  // 方差
  double stdev = sqrt(variance);             // 标准差

  res.first = variance;
  res.second = stdev;

  return res;
}

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @param {VectorXd} c1
 * @param {VectorXd} c2
 * @return {*}
 */
double area_criterion(Eigen::VectorXd c1, Eigen::VectorXd c2) {
  double c1_max = c1.maxCoeff();
  double c1_min = c1.minCoeff();

  double c2_max = c2.maxCoeff();
  double c2_min = c2.minCoeff();

  return (-1.0 * (c1_max - c1_min) * (c2_max - c2_min));
}

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @param {VectorXd} c1
 * @param {VectorXd} c2
 * @return {*}
 */
double closeness_criterion(Eigen::VectorXd c1, Eigen::VectorXd c2) {
#define MIN_VALUE (0.01)
  double c1_max = c1.maxCoeff();
  double c1_min = c1.minCoeff();

  double c2_max = c2.maxCoeff();
  double c2_min = c2.minCoeff();

  std::vector<double> d1, d2;

  for (uint16_t idx = 0; idx < c1.rows(); idx++) {
    if ((c1_max - c1[idx]) < (c1[idx] - c1_min)) {
      d1.push_back(c1_max - c1[idx]);
    } else {
      d1.push_back(c1[idx] - c1_min);
    }

    if ((c2_max - c2[idx]) < (c2[idx] - c2_min)) {
      d2.push_back(c2_max - c2[idx]);
    } else {
      d2.push_back(c2[idx] - c2_min);
    }
  }

  double bate = 0.0;
  for (uint8_t idx = 0; idx < d1.size(); idx++) {
    double d = std::max(std::min(d1.at(idx), d2.at(idx)), MIN_VALUE);
    bate += (1.0 / d);
  }

  return bate;
}

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @param {VectorXd} c1
 * @param {VectorXd} c2
 * @return {*}
 */
double variance_criterion(Eigen::VectorXd c1, Eigen::VectorXd c2) {
#define MIN_VALUE (0.01)

  double c1_max = c1.maxCoeff();
  double c1_min = c1.minCoeff();

  double c2_max = c2.maxCoeff();
  double c2_min = c2.minCoeff();

  std::vector<double> d1, d2;

  for (uint32_t idx = 0; idx < c1.rows(); idx++) {
    if ((c1_max - c1[idx]) < (c1[idx] - c1_min)) {
      d1.push_back(c1_max - c1[idx]);
    } else {
      d1.push_back(c1[idx] - c1_min);
    }

    if ((c2_max - c2[idx]) < (c2[idx] - c2_min)) {
      d2.push_back(c2_max - c2[idx]);
    } else {
      d2.push_back(c2[idx] - c2_min);
    }
  }

  std::vector<double> E1, E2;
  for (uint32_t idx = 0; idx < d1.size(); idx++) {
    if (d1.at(idx) < d2.at(idx)) {
      E1.push_back(d1.at(idx));
    } else {
      E2.push_back(d2.at(idx));
    }
  }

  std::pair<double, double> V1 = std::make_pair(0.0, 0.0);
  std::pair<double, double> V2 = std::make_pair(0.0, 0.0);

  if (E1.size() > 0) {
    V1 = compute_variance(E1);
  }

  if (E2.size() > 0) {
    V2 = compute_variance(E2);
  }

  return ((-1.0) * (V1.first + V2.first));
}

// 两直线交点求解
void calc_cross_point(const double *a, const double *b, const double *c,
                      double *x, double *y) {
  *x = (b[0] * -c[1] - b[1] * -c[0]) / (a[0] * b[1] - a[1] * b[0]);
  *y = (a[1] * -c[0] - a[0] * -c[1]) / (a[0] * b[1] - a[1] * b[0]);
}

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @param {Rect_t} &rect
 * @return {*}
 */
void calc_rect_contour(Rect_t &rect) {
  for (uint8_t idx = 0; idx < 4; idx++) {
    calc_cross_point(&rect.a[idx], &rect.b[idx], &rect.c[idx],
                     &rect.corner[idx][0], &rect.corner[idx][1]);
  }

  rect.corner[4][0] = 0.5 * (rect.corner[0][0] + rect.corner[2][0]);
  rect.corner[4][1] = 0.5 * (rect.corner[0][1] + rect.corner[2][1]);

  double temp_wid = sqrt(pow((rect.corner[0][0] - rect.corner[1][0]), 2.0) +
                         pow((rect.corner[0][1] - rect.corner[1][1]), 2.0));

  double temp_len = sqrt(pow((rect.corner[1][0] - rect.corner[2][0]), 2.0) +
                         pow((rect.corner[1][1] - rect.corner[2][1]), 2.0));

  if (temp_len > temp_wid) {
    rect.length = temp_len;
    rect.width = temp_wid;
    rect.theta *= -1.0;
  } else {
    rect.length = temp_wid;
    rect.width = temp_len;
    rect.theta =
        (rect.theta >= 0.0) ? (M_PI_2 - rect.theta) : (-M_PI_2 - rect.theta);
  }
}

Rect_t gen_rectInfo(const Eigen::MatrixXd point, const double final_theta) {
  // best angle
  double sin_s = sin(final_theta);
  double cos_s = cos(final_theta);

  Eigen::Matrix2d fina_rotate_mat;
  fina_rotate_mat << cos(final_theta), sin(final_theta),
      -1.0 * sin(final_theta), cos(final_theta);

  // final points
  Eigen::MatrixXd p_final = fina_rotate_mat * point;

  Eigen::VectorXd c1_s = p_final.row(0);
  Eigen::VectorXd c2_s = p_final.row(1);

  // 构造最终矩阵参数
  Rect_t new_Rect;
  new_Rect.theta = final_theta;

  new_Rect.a[0] = cos_s;
  new_Rect.b[0] = sin_s;
  new_Rect.c[0] = c1_s.minCoeff();

  new_Rect.a[1] = -1.0 * sin_s;
  new_Rect.b[1] = cos_s;
  new_Rect.c[1] = c2_s.minCoeff();

  new_Rect.a[2] = cos_s;
  new_Rect.b[2] = sin_s;
  new_Rect.c[2] = c1_s.maxCoeff();

  new_Rect.a[3] = -1.0 * sin_s;
  new_Rect.b[3] = cos_s;
  new_Rect.c[3] = c2_s.maxCoeff();

  new_Rect.a[4] = new_Rect.a[0];
  new_Rect.b[4] = new_Rect.b[0];
  new_Rect.c[4] = new_Rect.c[0];

  // 计算四个角点以及长宽和最终朝向角
  calc_rect_contour(new_Rect);

  return new_Rect;
}

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @param {MatrixXd} point
 * @return {*}
 */
Rect_t L_shape_Fit_Proc(const Eigen::MatrixXd point, const double theta_) {
#define ANGLE_STEP (1.0 / 180.0 * M_PI)

  Eigen::MatrixXd p2;
  Eigen::Matrix2d rotate_mat;

  double min_distance = -1e2;
  double final_theta = 0.0;

  double theta_start = -theta_ - M_PI_2 / 90.0;  // 30°
  double theta_end = -theta_ + M_PI_2 / 90.0;    // 30°
  double step_ = (M_PI_2 / 90.0);                // 3°

  while (theta_start <= theta_end) {
    // double temp_angle = step * ANGLE_STEP;
    rotate_mat << cos(theta_start), sin(theta_start), -1.0 * sin(theta_start),
        cos(theta_start);

    p2 = rotate_mat * point;

    Eigen::VectorXd c1 = p2.row(0);
    Eigen::VectorXd c2 = p2.row(1);

    /* 选择拟合准则 */
    // double d = area_criterion(c1, c2);
    // double d = closeness_criterion(c1, c2);
    double d = variance_criterion(c1, c2);
    if (min_distance < d) {
      min_distance = d;
      final_theta = theta_start;
    }

    theta_start += step_;  // each step is 2°
  }

  // std::cout << "min_distance : " << min_distance << std::endl;

  // 计算矩阵四条边的直线表达式
  Rect_t fit_Rect = gen_rectInfo(point, final_theta);

  return fit_Rect;
}
