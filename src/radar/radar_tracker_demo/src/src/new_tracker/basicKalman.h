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

  virtual void kalmanPredict(void) {
    X = F * X;
    P = F * P * F.transpose() + Q;
  }

  virtual void kalmanUpdate(Eigen::Matrix<T, z_dim, 1> newZ) {
    auto S = H * P * H.transpose() + R;
    auto K = P * H.transpose() * S.inverse();
    auto z = H * X;
    X = X + K * (newZ - H * X);
    P = P - K * H * P;
  }

  virtual void Set_F(){};
  virtual void Set_Q(){};
  virtual void Set_H(void) { H = H.setIdentity(); }
  virtual void Set_R(){};

  virtual Eigen::Matrix<T, x_dim, 1> GetX() { return X; };

  virtual Eigen::Matrix<T, z_dim, z_dim> GetS() {
    Eigen::Matrix<T, z_dim, z_dim> S = H * P * H.transpose() + R;
    return S;
  };

 public:
  Eigen::Matrix<T, x_dim, 1> X;
  Eigen::Matrix<T, x_dim, x_dim> P;
  Eigen::Matrix<T, x_dim, x_dim> Q;
  Eigen::Matrix<T, x_dim, x_dim> F;
  Eigen::Matrix<T, z_dim, x_dim> H;
  Eigen::Matrix<T, z_dim, z_dim> R;
};
