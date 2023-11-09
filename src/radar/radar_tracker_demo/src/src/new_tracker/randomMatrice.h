
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
  RandomMatriceFilter() {
    // std::cout << "START A NEW [RandomMatriceFilter]" << std::endl;
    Set_F();
    Set_Q();
    Set_H();
    Set_R();
    SetSPDMat();
  }

  /**
   * @names:
   * @description: Briefly describe the function of your function
   * @return {*}
   */
  void kalmanPredict() override {
    // 目标运动状态预测
    predictState();

    // 目标物理状态预测
    predictExtendState();
  }

  /**
   * @names:
   * @description: Briefly describe the function of your function
   * @param {Matrix<T, Eigen::Dynamic, z_dim>} newZ
   * @return {*}
   */
  void kalmanUpdate(Eigen::Matrix<T, Eigen::Dynamic, z_dim> newZ) override {
    // 目标运动状态更新
    updateKinematic(newZ);

    // 目标物理状态更新
    updatePhysical(newZ);
  }

  ~RandomMatriceFilter() {
    // std::cout << "delete RandomMatriceFilter" << std::endl;
  }

  /**
   * @names:
   * @description: Briefly describe the function of your function
   * @return {*}
   */
  void Set_F() override {
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
  void Set_Q() override {
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
  void Set_H() override { this->H = this->H.setIdentity(); }

  /**
   * @names:
   * @description: Briefly describe the function of your function
   * @return {*}
   */
  void Set_R() override {
    this->R << std::pow(0.2, 2.0), 0.0, 0.0, 0.0, std::pow(0.2, 2.0), 0.0, 0.0,
        0.0, std::pow(0.2, 2.0);
  };

  void SetSPDMat() { SPD_Mat = SPD_Mat.setOnes() * pow(0.5, 2.0); }

  void getObjectShape(T* len, T* wid, T* theta) {}

 private:
  // vari for extendsion
  Eigen::Matrix<T, 2, 2> SPD_Mat;
  T alpha_ = 2.0;
  T tau_ = 1.0;

  T std_accLong_ = 0.4, std_accLat_ = 0.4;  // ACC NOISE

 private:
  /**
   * @names:
   * @description: Briefly describe the function of your function
   * @return {*}
   */
  void predictState() {
    // 运动状态预测
    this->X = this->F * this->X;
    this->P = this->F * this->P * this->F.transpose() + this->Q;

    // 雅可比矩阵更新
    computeMeasJacMat();

    // 量测状态预测
    this->Zpre = this->H * this->X;

    Eigen::Matrix<T, z_dim, 2> H_new = this->H.block(0, 0, 3, 2);
    Eigen::Matrix<T, z_dim, z_dim> SPMMat2 =
        H_new * SPD_Mat * H_new.transpose();

    this->S = this->H * this->P * this->H.transpose() + 1.2 * SPMMat2 + this->R;
    this->K = this->P * this->H.transpose() * this->S.inverse();
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
   * @param {Matrix<T, z_dim, 1>} meas_mat
   * @return {*}
   */
  void updateKinematic(Eigen::Matrix<T, Eigen::Dynamic, z_dim> meas_mat) {
    int meas_num = meas_mat.rows();

    Eigen::Matrix<T, z_dim, 1> y_hat = meas_mat.colwise().mean();

    // compute kalman S and K
    Eigen::Matrix<T, z_dim, 2> H_new = this->H.block(0, 0, 3, 2);
    Eigen::Matrix<T, z_dim, z_dim> SPMMat2 =
        H_new * SPD_Mat * H_new.transpose();

    Eigen::Matrix<T, z_dim, z_dim> Yk = 1.2 * SPMMat2 + this->R;

    Eigen::Matrix<T, z_dim, z_dim> S =
        this->H * this->P * this->H.transpose() + Yk / meas_num;

    // Eigen::Matrix<T, z_dim, z_dim> S =
    //     this->H * this->P * this->H.transpose() + this->R;

    Eigen::Matrix<T, x_dim, z_dim> K =
        this->P * this->H.transpose() * S.inverse();

    std::cout << "X1 \n" << this->X.transpose() << std::endl;
    std::cout << "Diff: \n" << (y_hat - this->Zpre).transpose() << std::endl;

    // std::cout << "P: \n" << this->P << std::endl;
    // std::cout << "H: \n" << this->H << std::endl;
    // std::cout << "S: \n" << S << std::endl;
    // std::cout << "K: \n" << K << std::endl;

    this->X = this->X + K * (y_hat - this->Zpre);
    this->P = this->P - K * this->H * this->P;

    std::cout << "X2 \n" << this->X.transpose() << std::endl;
  }

  /**
   * @names:
   * @description: Briefly describe the function of your function
   * @param {Matrix<T, z_dim, 1>} meas_mat
   * @return {*}
   */
  void updatePhysical(Eigen::Matrix<T, Eigen::Dynamic, z_dim> meas_mat) {
    int meas_num = meas_mat.rows();

    if (meas_num <= 4) {
      std::cout << "Meas Number is too small." << std::endl;
      return;
    }

    Eigen::Matrix<T, 1, z_dim> y_hat = meas_mat.colwise().mean();
    Eigen::Matrix<T, z_dim, 1> y_hat_vet = y_hat.transpose();

    Eigen::Matrix<T, 1, z_dim> Z_mean(
        Eigen::Matrix<T, 1, z_dim>::Map(y_hat_vet.data(), meas_mat.cols()));

    auto ZK = meas_mat.rowwise() - Z_mean;

    Eigen::Matrix<T, z_dim, z_dim> Yhat = (ZK.adjoint() * ZK) / (meas_num - 1);

    // compute kalman S and K
    Eigen::Matrix<T, z_dim, 2> H_new = this->H.block(0, 0, 3, 2);
    Eigen::Matrix<T, z_dim, z_dim> SPMMat2 =
        H_new * SPD_Mat * H_new.transpose();

    Eigen::Matrix<T, z_dim, z_dim> Yk = 0.9 * SPMMat2 + this->R;

    Eigen::Matrix<T, z_dim, z_dim> S =
        this->H * this->P * this->H.transpose() + Yk / meas_num;

    Eigen::Matrix<T, z_dim, z_dim> N_ =
        (y_hat_vet - this->H * this->X) *
        ((y_hat_vet - this->H * this->X).transpose());

    Eigen::Matrix<T, z_dim, z_dim> X_chol = SPMMat2.ldlt().matrixL();

    Eigen::Matrix<T, z_dim, z_dim> S_chol = S.ldlt().matrixL();

    Eigen::Matrix<T, z_dim, z_dim> Yk_chol = Yk.ldlt().matrixL();

    Eigen::Matrix<T, z_dim, z_dim> N_hat = X_chol * S_chol.inverse() * N_ *
                                           S_chol.inverse().transpose() *
                                           X_chol.transpose();

    Eigen::Matrix<T, z_dim, z_dim> Y_hat = X_chol * Yk_chol.inverse() * Yhat *
                                           Yk_chol.inverse().transpose() *
                                           X_chol.transpose();

    SPMMat2 = 1.0 / (alpha_ + meas_num) * (alpha_ * SPMMat2 + N_hat + Y_hat);

    // 构造伪逆矩阵
    Eigen::MatrixXf H_inv = computPseudoInverse(H_new);
    Eigen::MatrixXf HT_inv = computPseudoInverse(H_new.transpose());

    SPD_Mat = H_inv * SPMMat2 * HT_inv;

    alpha_ = alpha_ + meas_num;

    std::cout << "SPD Matrice:" << SPD_Mat << std::endl;
  }

  Eigen::MatrixXf computPseudoInverse(Eigen::MatrixXf A) {
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(
        A, Eigen::ComputeThinU | Eigen::ComputeThinV);

    // 获取奇异值矩阵
    Eigen::MatrixXf singularValues = svd.singularValues();

    // 构造伪逆矩阵
    Eigen::MatrixXf A_Pinv = svd.matrixV() *
                             singularValues.asDiagonal().inverse() *
                             svd.matrixU().transpose();

    return A_Pinv;
  }
};
