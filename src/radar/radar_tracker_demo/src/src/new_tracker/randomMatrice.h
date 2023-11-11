
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
    const T dt = 0.1;
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
    Eigen::Matrix<T, x_dim, 2> G;
    Eigen::Matrix<T, 2, 2> q;

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

  void SetSPDMat() { SPD_Mat = SPD_Mat.setIdentity() * pow(1.0, 2.0); }

  void getObjectShape(T* len, T* wid, T* theta) {}

 private:
  // vari for extendsion
  Eigen::Matrix<T, 3, 3> SPD_Mat;
  T alpha_ = 2.0;
  T tau_ = 1.0;

  T std_accLong_ = 1, std_accLat_ = 1;  // ACC NOISE

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
        SPD_Mat;  // H_new * SPD_Mat * H_new.transpose();

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
    const T dt = 0.1;

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
    Eigen::Matrix<T, z_dim, z_dim> SPMMat2 = SPD_Mat;
    // H_new* SPD_Mat* H_new.transpose();

    Eigen::Matrix<T, z_dim, z_dim> Yk = 1.2 * SPMMat2 + this->R;

    Eigen::Matrix<T, z_dim, z_dim> S =
        this->H * this->P * this->H.transpose() + Yk / meas_num;

    // std::cout << "measNum: " << meas_num << std::endl;

    // Eigen::Matrix<T, z_dim, z_dim> S =
    //     this->H * this->P * this->H.transpose() + this->R;

    Eigen::Matrix<T, x_dim, z_dim> K =
        this->P * this->H.transpose() * S.inverse();

    // std::cout << "X1 \n" << this->X.transpose() << std::endl;
    // std::cout << "Diff: \n" << (y_hat - this->Zpre).transpose() << std::endl;
    // std::cout << "gain: \n"
    //           << (K * (y_hat - this->Zpre)).transpose() << std::endl;

    // std::cout << "P: \n" << this->P << std::endl;
    // // std::cout << "H: \n" << this->H << std::endl;
    // std::cout << "S: \n" << S << std::endl;
    // std::cout << "K: \n" << K << std::endl;

    this->X = this->X + K * (y_hat - this->Zpre);
    this->P = this->P - K * this->H * this->P;

    // std::cout << "X2 \n" << this->X.transpose() << std::endl;
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

    // 量测均值
    Eigen::Matrix<T, 1, z_dim> y_hat = meas_mat.colwise().mean();
    Eigen::Matrix<T, z_dim, 1> y_hat_vet = y_hat.transpose();

    auto ZK = meas_mat.rowwise() - y_hat;
    Eigen::Matrix<T, z_dim, z_dim> Yhat = (ZK.adjoint() * ZK) / (meas_num - 1);

    // compute kalman S and K
    Eigen::Matrix<T, z_dim, 2> H_new = this->H.block(0, 0, 3, 2);
    Eigen::Matrix<T, z_dim, z_dim> SPMMat2 =
        SPD_Mat;  //    H_new* SPD_Mat* H_new.transpose();

    // std::cout << "H: \n" << this->H << std::endl;

    // std::cout << "H_new: \n" << H_new << std::endl;

    Eigen::Matrix<T, z_dim, z_dim> Yk = SPMMat2 + this->R;

    Eigen::Matrix<T, z_dim, z_dim> S =
        this->H * this->P * this->H.transpose() + Yk / meas_num;

    Eigen::Matrix<T, z_dim, z_dim> N_ =
        (y_hat_vet - this->H * this->X) *
        ((y_hat_vet - this->H * this->X).transpose());

    Eigen::Matrix<T, z_dim, z_dim> X_chol = SPMMat2.llt().matrixL();
    // matSquare(SPMMat2);

    // std::cout << "SPMMat2: \n" << SPMMat2 << std::endl;
    // std::cout << "X_chol: \n" << X_chol << std::endl;
    // std::cout << "X_chol: \n" << X_chol * X_chol.transpose() << std::endl;

    Eigen::Matrix<T, z_dim, z_dim> S_chol = S.llt().matrixL();
    matSquare(S);
    std::cout << "S: \n" << S << std::endl;
    std::cout << "S_chol: \n" << S_chol << std::endl;
    std::cout << "S_chol: \n" << S_chol * S_chol.transpose() << std::endl;

    Eigen::Matrix<T, z_dim, z_dim> Yk_chol = Yk.llt().matrixL();

    // std::cout << "Yk: \n" << Yk << std::endl;
    // std::cout << "Yk_chol: \n" << Yk_chol << std::endl;
    // std::cout << "Yk_chol: \n" << Yk_chol * Yk_chol.transpose() << std::endl;

    Eigen::Matrix<T, z_dim, z_dim> N_chol = N_.llt().matrixL();
    std::cout << "X_chol: \n" << X_chol << std::endl;
    std::cout << "S_chol.inverse(): \n" << S_chol.inverse() << std::endl;
    std::cout << "N_chol: \n" << N_chol << std::endl;
    std::cout << "X_chol.inverse(): \n" << X_chol.inverse() << std::endl;

    Eigen::Matrix<T, z_dim, 1> bold_dot =
        S_chol.inverse() * (y_hat_vet - this->H * this->X);
    Eigen::Matrix<T, z_dim, z_dim> N_hat =
        X_chol * bold_dot * ((X_chol * bold_dot).transpose());

    Eigen::Matrix<T, z_dim, z_dim> Y_hat =
        X_chol * Yk_chol.inverse() * Yhat *
        ((X_chol * Yk_chol.inverse()).transpose());

    // std::cout << "SPMMat2 Matrice: \n" << SPMMat2 << std::endl;
    SPMMat2 = 1.0 / (alpha_ + meas_num) * (alpha_ * SPMMat2 + N_hat + Y_hat);
    std::cout << "N_hat: \n" << N_hat << std::endl;
    std::cout << "Y_hat: \n" << Y_hat << std::endl;
    // std::cout << "alpha_: \n" << alpha_ << std::endl;
    // std::cout << "SPMMat2 Matrice: \n" << SPMMat2 << std::endl;

    // // 构造伪逆矩阵
    // Eigen::MatrixXf H_inv = computPseudoInverse(H_new);
    // Eigen::MatrixXf HT_inv = computPseudoInverse(H_new.transpose());

    SPD_Mat = SPMMat2;  // H_inv * SPMMat2 * HT_inv;

    alpha_ = alpha_ + meas_num;

    // std::cout << "SPD Matrice: \n" << SPD_Mat << std::endl;

    getShapeValue(SPD_Mat);
  }

  void matSquare(const Eigen::Matrix<T, -1, -1> AA) {
    // Eigen::LLT<Eigen::MatrixXf> lltOfA(A);  // Cholesky分解

    // if (lltOfA.info() == Eigen::Success) {
    //   Eigen::MatrixXf B = lltOfA.matrixL();  // 获取Cholesky分解的下三角矩阵

    //   std::cout << "Matrix B:" << std::endl;
    //   std::cout << B << std::endl;
    // } else {
    //   std::cout << "Cholesky decomposition failed!" << std::endl;
    // }

    Eigen::MatrixXf A(3, 3);  // 假设有一个3x3的对称正定矩阵A
    A << 4, 2, 1, 2, 5, 3, 1, 3, 6;

    Eigen::LLT<Eigen::MatrixXf> lltOfA(AA);  // Cholesky分解

    if (lltOfA.info() == Eigen::Success) {
      Eigen::MatrixXf B = lltOfA.matrixL();  // 获取Cholesky分解的下三角矩阵

      std::cout << "Matrix B:" << std::endl;
      std::cout << B << std::endl;
      std::cout << B * B.transpose() << std::endl;
    } else {
      std::cout << "Cholesky decomposition failed!" << std::endl;
    }
  }

  void getShapeValue(const Eigen::Matrix<T, -1, -1> mat) {
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXf> eigensolver(mat);
    if (eigensolver.info() != Eigen::Success) {
      std::cerr << "Eigen solver failed!" << std::endl;
      return;
    }

    Eigen::VectorXf eigenvalues = eigensolver.eigenvalues();
    std::cout << "Eigenvalues:" << std::endl;
    std::cout << eigenvalues << std::endl;
  }

  /**
   * @names: computPseudoInverse
   * @description: 求解伪逆矩阵
   * @param {MatrixXf} A
   * @return {*}
   */
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
