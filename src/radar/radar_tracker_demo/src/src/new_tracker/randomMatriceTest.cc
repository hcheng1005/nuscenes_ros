#include <algorithm>
#include <iostream>
#include <random>

#include "randomMatrice.h"

// OPENCV
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

typedef float T;

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @return {*}
 */
void genConnerPos(const Eigen::VectorXf &Gt_X, const Eigen::VectorXf &Gt_Shape,
                  T *cornerPos) {
  /*
0*****1*****2
*     *     *
*     *     *7
7*****X*****3
*     *     *
*     *     *
6*****5*****4
*/
  T Xcenter = Gt_X(0), Ycenter = Gt_X(1);
  T headingAng = Gt_Shape(2), half_wid = Gt_Shape(0) * 0.5,
    half_len = Gt_Shape(1) * 0.5;

  T plusOrminus[8][2] = {
      {1.0, 1.0},   {0.0, 1.0},  {-1.0, 1.0},  {-1.0, 0.0},
      {-1.0, -1.0}, {0.0, -1.0}, {+1.0, -1.0}, {+1.0, 0.0},
  };

  T new_heading = headingAng;

  if (fabs(new_heading) > M_PI_2) {
    new_heading =
        (new_heading > 0.0) ? (new_heading - M_PI) : (new_heading + M_PI);

    if (new_heading < 0.0) {
      double tempdata = half_len;
      half_len = half_wid;
      half_wid = tempdata;
      new_heading = new_heading + M_PI_2;
    }
  }

  T RotMat[2][2] = {{cos(new_heading), sin(new_heading)},
                    {-1.0 * sin(new_heading), cos(new_heading)}};
  T tempX, tempY;

  for (uint8_t idx = 0; idx < 8; idx++) {
    tempX = Xcenter + (plusOrminus[idx][0] * half_wid) * RotMat[0][0] +
            (plusOrminus[idx][1] * half_len) * RotMat[0][1];
    tempY = Ycenter + (plusOrminus[idx][0] * half_wid) * RotMat[1][0] +
            (plusOrminus[idx][1] * half_len) * RotMat[1][1];

    cornerPos[2 * idx + 0] = tempX;
    cornerPos[2 * idx + 1] = tempY;
  }

  cornerPos[2 * 8 + 0] = Xcenter;
  cornerPos[2 * 8 + 1] = Ycenter;
}

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @param {VectorXf} &Gt_X
 * @param {VectorXf} &Gt_Shape
 * @return {*}
 */
Eigen::MatrixXf genMeas(const Eigen::VectorXf &Gt_X,
                        const Eigen::VectorXf &Gt_Shape) {
  // 创建一个随机数引擎
  std::random_device rd;
  std::mt19937 gen(rd());

  // 创建一个均匀分布的随机数生成器
  std::uniform_int_distribution<> dis(1, 4);

  std::normal_distribution<T> distribution_range(0.0, 1.0);
  std::normal_distribution<T> distribution_doppler(0.0, 0.5);

  T cornerPos[9 * 2];
  genConnerPos(Gt_X, Gt_Shape, cornerPos);

  std::vector<T> measSet;
  int totalNum = 0;
  for (int i = 0; i < 9; i++) {
    int random_num = dis(gen);
    totalNum += random_num;
    for (int i2 = 0; i2 < random_num; i2++) {
      T x_noise = distribution_range(gen);
      T y_noise = distribution_range(gen);

      T vr_gt =
          (cornerPos[i * 2 + 0] * Gt_X(2) + cornerPos[i * 2 + 1] * Gt_X(3)) /
          sqrt(pow(Gt_X(0), 2.0) + pow(Gt_X(1), 2.0));
      T vr_noise = distribution_doppler(gen);

      measSet.push_back(cornerPos[i * 2 + 0] + x_noise);
      measSet.push_back(cornerPos[i * 2 + 1] + y_noise);
      measSet.push_back(vr_gt + vr_noise);
    }
  }

  Eigen::MatrixXf Z = Eigen::MatrixXf(totalNum, 3);
  //
  for (int i = 0; i < totalNum; i++) {
    Z(i, 0) = measSet.at(i * 3 + 0);
    Z(i, 1) = measSet.at(i * 3 + 1);
    Z(i, 2) = measSet.at(i * 3 + 2);
  }

  return Z;
}

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @param {int} argc
 * @param {char} *argv
 * @return {*}
 */
int main(int argc, char *argv[]) {
  cv::Mat image = cv::Mat::zeros(600, 800, CV_8UC3);
  RandomMatriceFilter<T, 6, 3> *filter = new RandomMatriceFilter<T, 6, 3>();

  const int StepNum = 100;

  // step 1: 定义目标初始位置、速度、长宽以及朝向角
  Eigen::VectorXf Gt_X(4);
  Gt_X << -2, 10, 4, 5;

  Eigen::VectorXf Gt_Shape(3);
  Gt_Shape << 2.8, 4.8, atan2(Gt_X(2), Gt_X(3));

  Eigen::Matrix<float, 6, 1> X;
  X = X.setZero();
  T vr = (Gt_X(0) * Gt_X(2) + Gt_X(1) * Gt_X(3)) /
         sqrt(pow(Gt_X(0), 2.0) + pow(Gt_X(1), 2.0));
  X(iDistLat) = Gt_X(0);
  X(iDistLong) = Gt_X(1);
  X(iVrelLat) = 0.0;   // vr * sin(atan2(Gt_X(0), Gt_X(1)));
  X(iVrelLong) = 0.0;  // vr * cos(atan2(Gt_X(0), Gt_X(1)));

  Eigen::Matrix<float, 6, 6> P;
  P = P.setIdentity();

  filter->X = X;
  filter->P = P;

  for (int step = 0; step < StepNum; step++) {
    Gt_X(0) = Gt_X(0) + Gt_X(2) * 0.1;
    Gt_X(1) = Gt_X(1) + Gt_X(3) * 0.1;

    std::cout << " ---------- Step: " << step << " ----------" << std::endl;

    image = cv::Mat::zeros(600, 800, CV_8UC3);
    Eigen::MatrixXf Z = genMeas(Gt_X, Gt_Shape);

    std::vector<cv::Point2f> point_cloud;
    for (int i = 0; i < Z.rows(); i++) {
      // 添加点到点云数据
      cv::circle(
          image,
          cv::Point2f((Z(i, 0) + 100) / 200 * 800, 600 - Z(i, 1) / 100 * 600),
          2, cv::Scalar(0, 200, 200), -1);
    }

    filter->kalmanPredict();
    filter->kalmanUpdate(Z);

    Eigen::MatrixXf x = filter->GetX();

    cv::circle(image,
               cv::Point2f((x(0) + 100) / 200 * 800, 600 - x(1) / 100 * 600),
               10, cv::Scalar(200, 0, 0), -1);

    cv::imshow("Point Cloud Visualization", image);
    cv::waitKey(500);
  }

  return 1;
}