#pragma once

// Eigen
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <iostream>

template <typename T, uint x_dim, uint z_dim>
class basicKalmanFilter {
 public:
  basicKalmanFilter() {
    // std::cout << "build a new basicKalmanFilter" << std::endl;
  }

  virtual ~basicKalmanFilter() {
    // std::cout << "delete basicKalmanFilter" << std::endl;
  }

  // 预测
  virtual void kalmanPredict(void) {
    // 运动状态预测
    X = F * X;
    P = F * P * F.transpose() + Q;

    // 量测状态预测
    Zpre = H * X;

    S = H * P * H.transpose() + R;
    K = P * H.transpose() * S.inverse();
  }

  // 更新
  virtual void kalmanUpdate(Eigen::Matrix<T, Eigen::Dynamic, z_dim> newZ) {
    X = X + K * (newZ.transpose() - Zpre);
    P = P - K * H * P;
  }

  // 计算量测与预测的马氏距离
  virtual T computeMahalanobis(Eigen::Matrix<T, z_dim, 1> newZ) {
    T distance = sqrt((newZ - Zpre).transpose() * S.inverse() * (newZ - Zpre));
    return distance;
  }

  virtual void Set_F(){};
  virtual void Set_Q(){};
  virtual void Set_H(void) { H = H.setIdentity(); }
  virtual void Set_R(){};

  virtual Eigen::Matrix<T, x_dim, 1> GetX() { return X; };

  virtual Eigen::Matrix<T, z_dim, 1> GetZPre() { return Zpre; };

  virtual Eigen::Matrix<T, z_dim, z_dim> GetS() {
    S = H * P * H.transpose() + R;
    return S;
  };

 public:
  Eigen::Matrix<T, x_dim, 1> X;
  Eigen::Matrix<T, z_dim, 1> Zpre;
  Eigen::Matrix<T, x_dim, x_dim> P;
  Eigen::Matrix<T, x_dim, x_dim> Q;
  Eigen::Matrix<T, x_dim, x_dim> F;
  Eigen::Matrix<T, z_dim, x_dim> H;
  Eigen::Matrix<T, z_dim, z_dim> R;
  Eigen::Matrix<T, z_dim, z_dim> S;
  Eigen::Matrix<T, x_dim, z_dim> K;
};
