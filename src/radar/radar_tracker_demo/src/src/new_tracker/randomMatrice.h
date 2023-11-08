
#include "basicKalman.h"

#define iDistLat 0
#define iDistLong 1
#define iVrelLat 2
#define iVrelLong 3
#define iAccLat 4
#define iAccLong 5

template <typename T, uint x_dim, uint z_dim>
class RandomMatriceFilter : public basicKalmanFilter<T, x_dim, z_dim> {
 public:
  void kalmanPredict() override {
    predictState();
    predictExtendState();
  }

  ~RandomMatriceFilter() {
    std::cout << "delete RandomMatriceFilter" << std::endl;
  }

  /**
   * @names:
   * @description: Briefly describe the function of your function
   * @return {*}
   */
  void Set_F() {
    const T dt = 0.075;
    this->F = this->F.setIdentity();
    this->F(iDistLat, iVrelLat) = dt;
    this->F(iDistLat, iAccLat) = 0.5 * dt * dt;
    this->F(iVrelLat, iAccLat) = dt;

    this->F(iDistLong, iVrelLong) = dt;
    this->F(iDistLong, iAccLong) = 0.5 * dt * dt;
    this->F(iVrelLong, iAccLong) = dt;
  };

  /**
   * @names:
   * @description: Briefly describe the function of your function
   * @return {*}
   */
  void Set_Q() {
    Eigen::Matrix<T, x_dim, z_dim> G;
    Eigen::Matrix<T, z_dim, z_dim> q;

    T dt = 0.1;
    T dt3 = powf(dt, 3.0);
    T dt2 = powf(dt, 2.0);

    G << 1 / 6 * dt3, 0, 0, 1 / 6 * dt3, 1 / 2 * dt2, 0, 0, 1 / 2 * dt2, dt, 0,
        0, dt;

    q << pow(std_accLong_, 2.0), 0.0, 0.0, pow(std_accLat_, 2.0);

    this->Q = G * q * G.transpose();
  };

  /**
   * @names:
   * @description: Briefly describe the function of your function
   * @return {*}
   */
  void Set_H(void) { this->H = this->H.setIdentity(); }

  /**
   * @names:
   * @description: Briefly describe the function of your function
   * @return {*}
   */
  void Set_R() {
    this->R << std::pow(0.2, 2.0), 0.0, 0.0, 0.0, std::pow(0.2, 2.0), 0.0, 0.0,
        0.0, std::pow(0.2, 2.0);
  };

 private:
  // vari for extendsion
  Eigen::Matrix<T, x_dim, z_dim> X_ex;
  T alpha_ = 2.0;
  T tau_ = 1.0;

  T std_accLong_, std_accLat_;  // ACC NOISE

 private:
  /**
   * @names:
   * @description: Briefly describe the function of your function
   * @return {*}
   */
  void predictState() {
    this->X = this->F * this->X;
    this->P = this->F * this->P * this->F.transpose() + this->Q;
  }

  /**
   * @names:
   * @description: Briefly describe the function of your function
   * @return {*}
   */
  void predictExtendState()  // extension info
  {
    const T dt = 0.075;

    // X_ex keep the same
    alpha_ = 2.0 + exp(-dt / tau_) * (alpha_ - 2.0);

    // make sure alpha_ is bigger than 2.0
    alpha_ = (alpha_ < 2.0) ? 2.01 : alpha_;
  }

  /**
   * @names:
   * @description: Briefly describe the function of your function
   * @return {*}
   */
  void computeMeasJacMat() {
    T x = this->X[iDistLat];
    T y = this->X[iDistLong];
    T vx = this->X[iVrelLat];
    T vy = this->X[iVrelLong];

    T range_ = sqrtf(x * x + y * y);

    // // 计算雅可比矩阵（RAV）
    // this->H << x / range_, y / range_, 0, 0, 0, 0,
    //                                 y / (range_ * range_), -x / (range_ *
    //                                 range_), 0, 0, 0, 0, y * (vx * y - vy *
    //                                 x) / (range_ * range_ * range_), x * (vy
    //                                 * x - vx * y) / (range_ * range_ *
    //                                 range_), x / range_, y / range_, 0, 0;

    // 计算雅可比矩阵（XYV）
    this->H << 1.0, 0, 0, 0, 0, 0, 0, 1.0, 0, 0, 0, 0,
        vx / range_ - x * (vx * x + vy * y) / pow(range_, 3.0),
        vy / range_ - y * (vx * x + vy * y) / pow(range_, 3.0), x / range_,
        y / range_, 0, 0;
  }

  /**
   * @names:
   * @description: Briefly describe the function of your function
   * @param {MatrixXf} meas_mat
   * @return {*}
   */
  void Update_Without_Extension(Eigen::Matrix<T, -1, -1> meas_mat) {
    Eigen::Matrix<T, z_dim, z_dim> Yk = 0.9 * X_ex + this->R;

    Eigen::Matrix<T, z_dim, z_dim> S =
        this->H * this->P * this->H.transpose() + Yk;

    Eigen::Matrix<T, x_dim, z_dim> K =
        this->P * this->H.transpose() * S.inverse();

    Eigen::Matrix<T, z_dim, 1> y_hat = meas_mat.colwise().mean();

    this->X = this->X + K * (y_hat - this->H * this->X);
    this->P = this->P - K * this->H * this->P;
  }

  /**
   * @names:
   * @description: Briefly describe the function of your function
   * @param {MatrixXf} meas_mat
   * @return {*}
   */
  void Update_With_Extension(Eigen::Matrix<T, z_dim, 1> meas_mat) {
    int meas_num = meas_mat.rows();

    Eigen::Matrix<T, z_dim, 1> y_hat = meas_mat.colwise().mean();

    Eigen::Matrix<T, -1, -1> Z_mean(
        Eigen::Matrix<T, -1, -1>::Map(y_hat.data(), meas_mat.cols()));

    auto ZK = meas_mat.rowwise() - Z_mean;

    Eigen::Matrix<T, z_dim, z_dim> Yhat = (ZK.adjoint() * ZK) / (meas_num - 1);

    // compute kalman S and K
    Eigen::Matrix<T, z_dim, z_dim> Yk = 0.9 * X_ex + this->R;

    Eigen::Matrix<T, z_dim, z_dim> S =
        this->H * this->P * this->H.transpose() + Yk / meas_num;

    Eigen::Matrix<T, x_dim, z_dim> K =
        this->P * this->H.transpose() * S.inverse();

    this->X = this->X + K * (y_hat - this->H * this->X);
    this->P = this->P - this->K * this->H * this->P;

    Eigen::Matrix<T, z_dim, z_dim> N_ =
        (y_hat - this->H * this->X) * ((y_hat - this->H * this->X).transpose());

    Eigen::Matrix<T, z_dim, z_dim> X_chol = X_ex.ldlt().matrixL();

    Eigen::Matrix<T, z_dim, z_dim> S_chol = S.ldlt().matrixL();

    Eigen::Matrix<T, z_dim, z_dim> Yk_chol = Yk.ldlt().matrixL();

    Eigen::Matrix<T, z_dim, z_dim> N_hat = X_chol * S_chol.inverse() * N_ *
                                           S_chol.inverse().transpose() *
                                           X_chol.transpose();

    Eigen::Matrix<T, z_dim, z_dim> Y_hat = X_chol * Yk_chol.inverse() * Yhat *
                                           Yk_chol.inverse().transpose() *
                                           X_chol.transpose();

    X_ex = 1.0 / (alpha_ + meas_num) * (alpha_ * X_ex + N_hat + Y_hat);

    alpha_ = alpha_ + meas_num;

    std::cout << "SPD Matrice:" << X_ex(0, 0) << " ," << X_ex(1, 1)
              << std::endl;
  }
};
