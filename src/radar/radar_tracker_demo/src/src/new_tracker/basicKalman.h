#pragma once

// Eigen
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <iostream>

template <class T>
class basicKalmanFilter {
 public:
  basicKalmanFilter(const uint x_dim, const uint z_dim) {
    X_dim = x_dim;
    Z_dim = z_dim;

    X.resize(X_dim, 1);
    P.resize(X_dim, X_dim);

    Q.resize(X_dim, X_dim);
    F.resize(X_dim, X_dim);

    H.resize(Z_dim, X_dim);
    R.resize(Z_dim, Z_dim);
  }

  ~basicKalmanFilter() {}

  virtual void kalmanPredict(void) {
    X = F * X;
    P = F * X * F.transpose() + Q;
  }

  virtual void kalmanUpdate(Eigen::Matrix<T, Eigen::Dynamic, 1> newZ) {
    auto S = H * P * H.transpose() + R;
    auto K = P * H.transpose() * S.inverse();
    auto z = H * X;
    X = X + K * (newZ - H * X);
    P = P - K * H * P;
  }

 private:
  uint X_dim;
  uint Z_dim;
  Eigen::Matrix<T, Eigen::Dynamic, 1> X;
  Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> P;
  Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> Q;
  Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> F;
  Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> H;
  Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> R;
};
