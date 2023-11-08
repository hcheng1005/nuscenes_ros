/*
 * @Description:
 * @version:
 * @Author: ChengHao
 * @Date: 2022-10-11 18:27:12
 * @LastEditors: chenghao hao.cheng@wuzheng.com
 * @LastEditTime: 2023-08-21 19:10:49
 */
#include <eigen3/unsupported/Eigen/KroneckerProduct>

#include "commonfunctions.h"
#include "filter.h"

#define SPD_DIM (3U)
#define SCALE_Z (0.25F)
#define T (0.075F)
#define tau (0.1F)

using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * @name:
 * @description:
 * @return {*}
 */
Tracker::Tracker() {
  n_x_ = SSIZE;
  n_z_ = MSIZE;

  // UKF参数初始化
  uint16_t n = SSIZE;

  lambda = alpha * alpha * (n + kappa) - n;
  float c = 0.5 / (n + lambda);

  Wc.fill(c);
  Wm.fill(c);

  Wc[0] = lambda / (n + lambda) + (1.0F - alpha * alpha + beta);
  Wm[0] = lambda / (n + lambda);
}

Tracker::~Tracker() {}

/**
 * @name: CalcJacobian
 * @description: 计算雅可比矩阵
 * @param {trackTable_strcut} *trace
 * @return {*}
 */
void Tracker::CalcJacobian(trackTable_strcut *trace) {
  double x = trace->KalmanInfo.StateEst[iDistLat];
  double y = trace->KalmanInfo.StateEst[iDistLong];
  double vx = trace->KalmanInfo.StateEst[iVrelLat];
  double vy = trace->KalmanInfo.StateEst[iVrelLong];

  double range_;
  range_ = sqrtf(x * x + y * y);

#ifdef _R_A_V_
  trace->KalmanInfo.JacMat << x / range_, y / range_, 0, 0, 0, 0,
      y / (range_ * range_), -x / (range_ * range_), 0, 0, 0, 0,
      y * (vx * y - vy * x) / (range_ * range_ * range_),
      x * (vy * x - vx * y) / (range_ * range_ * range_), x / range_,
      y / range_, 0, 0;
#endif

#ifdef _X_Y_V
  trace->KalmanInfo.JacMat << 1.0, 0, 0, 0, 0, 0, 0, 1.0, 0, 0, 0, 0,
      vx / range_ - x * (vx * x + vy * y) / pow(range_, 3.0),
      vy / range_ - y * (vx * x + vy * y) / pow(range_, 3.0), x / range_,
      y / range_, 0, 0;
#endif
}

/**
 * @name: compute_F
 * @description: 计算状态转移矩阵
 * @param {trackTable_strcut} *trace
 * @return {*}
 */
MatrixXd Tracker::compute_F(trackTable_strcut *trace, double dt) {
  double alpha_long = 0.6;
  double alpha_lat = 0.6;

  if (trace->ExtendInfo.Object_Class < VEHICLE) {
    alpha_lat = 0.6;
    alpha_long = 0.6;
  }

  alpha_lat += (this->yaw_rate_delay * 0.01);
  alpha_long += (this->yaw_rate_delay * 0.01);

  double alpha_long_2 = alpha_long * alpha_long;
  double alpha_long_T = alpha_long_2 * dt;
  double exp_alpha_long_T = exp(-1.0 * alpha_long_T);

  double alpha_lat_2 = alpha_lat * alpha_lat;
  double alpha_lat_T = alpha_lat_2 * dt;
  double exp_alpha_lat_T = exp(-1.0 * alpha_lat_T);

  MatrixXd tempF = MatrixXd(n_x_, n_x_);
  tempF << 1.0, 0.0, dt, 0.0,
      (alpha_lat_T - 1.0 + exp_alpha_lat_T) / alpha_lat_2, 0.0, 0.0, 1.0, 0.0,
      dt, 0.0, (alpha_long_T - 1 + exp_alpha_long_T) / alpha_long_2, 0.0, 0.0,
      1.0, 0.0, (1.0 - exp_alpha_lat_T) / alpha_lat, 0.0, 0.0, 0.0, 0.0, 1.0,
      0.0, (1.0 - exp_alpha_long_T) / alpha_long, 0.0, 0.0, 0.0, 0.0,
      exp_alpha_lat_T, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, exp_alpha_long_T;

  return tempF;
}

/**
 * @name: UKF_Predict
 * @description: UKF预测函数
 * @param {trackTable_strcut} *trace
 * @return {*}
 */
void Tracker::Prediction(double delta_t, trackTable_strcut *trace,
                         vehicleInfo_struct *vehicleInfo) {
#ifdef USING_EKF_FILTER
  EKF_Predict(delta_t, trace, vehicleInfo);
#endif

#ifdef USING_UKF_FILTER
  UKF_Predict(delta_t, trace, vehicleInfo);
#endif

  ComputeNewS_RMMandCorners(trace);
}

void Tracker::EKF_Predict(double delta_t, trackTable_strcut *trace,
                          vehicleInfo_struct *vehicleInfo) {
  MatrixXd rotMat = MatrixXd::Zero(2, 2);
  MatrixXd rotMat2 = MatrixXd::Identity(6, 6);

  double yawange = -1.0 * (delta_t * vehicleInfo->yaw_rate);

  rotMat(0, 0) = cos(yawange);
  rotMat(0, 1) = sin(yawange);
  rotMat(1, 0) = -1.0 * sin(yawange);
  rotMat(1, 1) = cos(yawange);

  rotMat2.block<2, 2>(0, 0) = rotMat;
  rotMat2.block<2, 2>(2, 2) = rotMat;
  rotMat2.block<2, 2>(4, 4) = rotMat;

  MatrixXd tempF = compute_F(trace, delta_t);  // 状态转移矩阵
  MatrixXd tempF_T = tempF.transpose();

  trace->KalmanInfo.StateEst = tempF * trace->KalmanInfo.StateEst;
  trace->KalmanInfo.P_ =
      tempF * trace->KalmanInfo.P_ * tempF_T + trace->KalmanInfo.Q_;
  trace->KalmanInfo.StateEst2 = trace->KalmanInfo.StateEst;

  // 旋转补偿
  trace->KalmanInfo.StateEst = rotMat2 * trace->KalmanInfo.StateEst;

  EKF_Compute_S_K(trace);
}

/**
 * @name: UKF_Compue_SigmaX
 * @description: 计算状态X的sigma点
 * @param {trackTable_strcut} *trace
 * @return {*}
 */
void Tracker::UKF_Predict(double delta_t, trackTable_strcut *trace,
                          vehicleInfo_struct *vehicleInfo) {
  MatrixXd rotMat = MatrixXd::Zero(2, 2);
  MatrixXd rotMat2 = MatrixXd::Identity(6, 6);

  double yawange = (delta_t * vehicleInfo->yaw_rate);

  rotMat(0, 0) = cos(yawange);
  rotMat(0, 1) = -1.0 * sin(yawange);
  rotMat(1, 0) = sin(yawange);
  rotMat(1, 1) = cos(yawange);

  rotMat2.block<2, 2>(0, 0) = rotMat;
  rotMat2.block<2, 2>(2, 2) = rotMat;
  rotMat2.block<2, 2>(4, 4) = rotMat;

  uint16_t n = trace->KalmanInfo.StateEst.size();
  double lambda = this->lambda + n;

  MatrixXd P = lambda * trace->KalmanInfo.P_;

  MatrixXd tempF = compute_F(trace, delta_t);  // 状态转移矩阵

  MatrixXd U = P.llt().matrixL();  // cholesky分解

  VectorXd X = trace->KalmanInfo.StateEst;
  VectorXd X2 = trace->KalmanInfo.StateEst;

  // ukf--sigma point
  MatrixXd sigmaX2 = MatrixXd(SSIZE, 2 * SSIZE + 1);

  VectorXd X_prior = VectorXd::Zero(n);
  VectorXd X_prior2 = VectorXd::Zero(n);

  X = rotMat2 * X;

  // 生成2n+1个sigma点
  trace->KalmanInfo.sigmaX.col(0) = X;

  sigmaX2.col(0) = X2;
  for (uint8_t idx = 0; idx < n; idx++) {
    VectorXd temp = X + U.col(idx);
    trace->KalmanInfo.sigmaX.col(idx + 1) = temp;
    temp = X - U.col(idx);
    trace->KalmanInfo.sigmaX.col(idx + n + 1) = temp;

    temp = X2 + U.col(idx);
    sigmaX2.col(idx + 1) = temp;
    temp = X2 - U.col(idx);
    sigmaX2.col(idx + n + 1) = temp;
  }

  // sigma点预测并计算状态先验（预测）
  X_prior.fill(0.0F);
  for (uint8_t idx = 0; idx < (2 * n + 1); idx++) {
    trace->KalmanInfo.sigmaX.col(idx) =
        tempF * trace->KalmanInfo.sigmaX.col(idx);
    X_prior += Wm[idx] * trace->KalmanInfo.sigmaX.col(idx);

    sigmaX2.col(idx) = tempF * sigmaX2.col(idx);
    X_prior2 += Wm[idx] * sigmaX2.col(idx);
  }

  // 状态协方差矩阵计算（预测）
  MatrixXd P_prior = MatrixXd::Zero(n, n);
  MatrixXd X_T;  // = X.transpose();
  for (uint8_t idx = 0; idx < (2 * n + 1); idx++) {
    X = trace->KalmanInfo.sigmaX.col(idx) - X_prior;
    X_T = X.transpose();
    P_prior += (Wc[idx] * (X * X_T));
  }

  trace->KalmanInfo.StateEst = X_prior;
  trace->KalmanInfo.P_ = trace->KalmanInfo.Q_ + P_prior;

  trace->KalmanInfo.StateEst2 = X_prior2;

  // 计算新息协方差和增益
  UKF_Compute_S_K(trace);
}

/**
 * @name: UKF_Compute_S_K
 * @description: 计算UKF新息协方差矩阵和增益矩阵
 * @param {trackTable_strcut} *trace
 * @return {*}
 */
void Tracker::EKF_Compute_S_K(trackTable_strcut *trace) {
  CalcJacobian(trace);
  MatrixXd JacMat_T = trace->KalmanInfo.JacMat.transpose();

  trace->KalmanInfo.MeasPre(0) = trace->KalmanInfo.StateEst(0);
  trace->KalmanInfo.MeasPre(1) = trace->KalmanInfo.StateEst(1);
  trace->KalmanInfo.MeasPre(2) =
      (trace->KalmanInfo.StateEst(iDistLat) *
           trace->KalmanInfo.StateEst(iVrelLat) +
       trace->KalmanInfo.StateEst(iDistLong) *
           trace->KalmanInfo.StateEst(iVrelLong)) /
      (sqrt(pow(trace->KalmanInfo.StateEst(iDistLat), 2.0) +
            pow(trace->KalmanInfo.StateEst(iDistLong), 2.0)));

  trace->KalmanInfo.S_out =
      trace->KalmanInfo.JacMat * trace->KalmanInfo.P_ * JacMat_T;
  trace->KalmanInfo.K_gain = trace->KalmanInfo.P_ * JacMat_T;
}

/**
 * @name: UKF_Compute_S_K
 * @description: 计算UKF新息协方差矩阵和增益矩阵
 * @param {trackTable_strcut} *trace
 * @return {*}
 */
void Tracker::UKF_Compute_S_K(trackTable_strcut *trace) {
  uint16_t n = trace->KalmanInfo.StateEst.size();

  MatrixXd K = MatrixXd::Zero(SSIZE, MSIZE);
  MatrixXd S = MatrixXd::Zero(MSIZE, MSIZE);

  MatrixXd sigmaZ = MatrixXd(MSIZE, 2 * n + 1);
  VectorXd tempZ = VectorXd::Zero(MSIZE);
  VectorXd tempX = VectorXd::Zero(SSIZE);
  VectorXd sumZ = VectorXd::Zero(MSIZE);

  double range_ = 0.0;

  for (uint8_t idx = 0; idx < (2 * n + 1); idx++) {
    VectorXd sigmax = trace->KalmanInfo.sigmaX.col(idx);

    tempZ(iRANGE) = sigmax(iDistLat);
    tempZ(iAZI) = sigmax(iDistLong);

    range_ = sqrt(pow(tempZ(0), 2.0) + pow(tempZ(1), 2.0));

    tempZ(iVEL) = (sigmax(iDistLat) * sigmax(iVrelLat) +
                   sigmax(iDistLong) * sigmax(iVrelLong)) /
                  range_;

    sigmaZ.col(idx) = tempZ;
    sumZ += Wm[idx] * tempZ;
  }

  MatrixXd temmZ_T;
  for (uint8_t idx = 0; idx < (2 * n + 1); idx++) {
    tempZ = sigmaZ.col(idx) - sumZ;
    tempX = trace->KalmanInfo.sigmaX.col(idx) - trace->KalmanInfo.StateEst;

    temmZ_T = tempZ.transpose();

    S += Wc[idx] * (tempZ * temmZ_T);
    K += (Wc[idx] * (tempX * temmZ_T));
  }

  trace->KalmanInfo.MeasPre = sumZ;
  trace->KalmanInfo.S_out = S;
  trace->KalmanInfo.K_gain = K;
}

/**
 * @name: ComputeNewS_RMMandCorners
 * @description: 通过目标八个角点坐标计算状态协方差
 * @param {trackTable_strcut} *trace
 * @return {*}
 */
void Tracker::ComputeNewS_RMMandCorners(trackTable_strcut *trace) {
  Compute_RMM_SPDMat(trace);
}

/**
 * @name: Compute_RMM_SPDMat
 * @description:
 * @param {trackTable_strcut} *trace
 * @return {*}
 */
MatrixXd Tracker::Compute_RMM_SPDMat(trackTable_strcut *trace) {
  MatrixXd SS = MatrixXd::Zero(MSIZE, MSIZE);
  MatrixXd RotMat = MatrixXd::Zero(2, 2);
  MatrixXd eigen_val = MatrixXd::Zero(2, 2);

  double ang = trace->ExtendInfo.box_theta;
  double tempW, tempL;
  double v4len, v4wid;

  if (fabs(ang) > (0.5 * RADAR_PI)) {
    ang = (ang > 0.0) ? (ang - RADAR_PI) : (ang + RADAR_PI);
  }

  if (trace->trackState == TRACK_STATE_DETECTION) {
    tempL = trace->ExtendInfo.Length +
            abs(trace->KalmanInfo.StateEst(iVrelLong)) * 0.5 +
            sqrt(trace->KalmanInfo.P_(1.0, 1.0)) * 0.5;
    tempW = trace->ExtendInfo.Width +
            abs(trace->KalmanInfo.StateEst(iVrelLat)) * 0.5 +
            sqrt(trace->KalmanInfo.P_(0.0, 0.0)) * 0.5;

    tempL = (tempL > tempW) ? tempL : tempW;
  } else {
    if ((trace->ExtendInfo.DynProp == crossing_moving) ||
        (trace->ExtendInfo.DynProp == crossing_stationary)) {
      tempL = trace->ExtendInfo.Length * 0.6 +
              sqrt((trace->KalmanInfo.P_(1.0, 1.0)));

      tempW = trace->ExtendInfo.Width * 0.5 +
              sqrt((trace->KalmanInfo.P_(0.0, 0.0)));
    } else {
      if (trace->ExtendInfo.Object_Class <= VEHICLE) {
        tempL = trace->ExtendInfo.Length * 0.6 +
                sqrt((trace->KalmanInfo.P_(1.0, 1.0)));

        tempW = trace->ExtendInfo.Width * 0.5 +
                sqrt((trace->KalmanInfo.P_(0.0, 0.0)));
      } else {
        tempL = trace->ExtendInfo.Length * 0.4 +
                sqrt((trace->KalmanInfo.P_(1.0, 1.0)));

        tempW = trace->ExtendInfo.Width * 0.5 +
                sqrt((trace->KalmanInfo.P_(0.0, 0.0)));
      }
    }

    v4len = fabs(trace->KalmanInfo.StateEst(iVrelLong)) * 0.3;
    if (v4len > 1.0) {
      v4len = 1.0;
    }
    tempL += v4len;

    v4wid = fabs(trace->KalmanInfo.StateEst(iVrelLat)) * 0.2;
    if (v4wid > 0.5) {
      v4wid = 0.5;
    }
    tempW += v4wid;
  }

  if ((trace->ExtendInfo.Object_Class > PEDESTRIAN) ||
      (trace->trackState == TRACK_STATE_DETECTION)) {
    tempW = (tempW < 1.0) ? 1.0 : tempW;
    tempL = (tempL < 1.0) ? 1.0 : tempL;
  } else {
    tempW = (tempW < 1.0) ? 1.0 : tempW;
    tempL = (tempL < 1.0) ? 1.0 : tempL;
  }

  RotMat << cos(ang), -1.0 * sin(ang), sin(ang), cos(ang);
  eigen_val << pow(tempL, 2.0), 0.0, 0.0, pow(tempW, 2.0);

  MatrixXd rmm = RotMat * eigen_val * RotMat.transpose();
  trace->RMM.RMM_shape = rmm;

  return SS;
}

/**
 * @name: UpdateState
 * @description: UKF状态更新函数
 * @param {trackTable_strcut} *trace
 * @return {*}
 */
void Tracker::UpdateState(trackTable_strcut *trace,
                          std::vector<RadarMeasure_struct> &global_point) {
  RandomMatrices_UKF(global_point, trace);
}

/**
 * @name: RandomMatrices_UKF
 * @description: 通过随机矩阵进行状态更新
 * @param {trackTable_strcut} *trace
 * @return {*}
 */
void Tracker::RandomMatrices_UKF(std::vector<RadarMeasure_struct> &global_point,
                                 trackTable_strcut *trace) {
  VectorXd ErrorX = VectorXd::Zero(MSIZE);
  //    VectorXd ErrorX_by_corner = VectorXd::Zero(MSIZE);
  VectorXd add_val = VectorXd::Zero(SSIZE);

  MatrixXd S = trace->KalmanInfo.S_out + trace->KalmanInfo.R_;
  MatrixXd K = trace->KalmanInfo.K_gain * S.inverse();

  // 确认有关联到量测
  if (trace->MeasInfo.detInfo.size() > 0) {
    // 获取最终的有效量测
    std::vector<double> lat_pos, long_pos;

    MatrixXd meas_mat = MatrixXd(trace->MeasInfo.detInfo.size(), MSIZE);

    for (uint16_t idx = 0; idx < trace->MeasInfo.detInfo.size(); idx++) {
      matchInfo_t &sub_det = trace->MeasInfo.detInfo.at(idx);
      RadarMeasure_struct *detInfo = &global_point[sub_det.cluster_Idx];
      meas_mat(idx, 0) = detInfo->DistLat;
      meas_mat(idx, 1) = detInfo->DistLong;
      meas_mat(idx, 2) = detInfo->VrelLong;

      lat_pos.push_back(detInfo->DistLat);
      long_pos.push_back(detInfo->DistLong);
    }

    // 重新计算角点
    trace->KalmanInfo.StateEst = trace->KalmanInfo.StateEst2;

    trace->KalmanInfo.MeasPre(0) = trace->KalmanInfo.StateEst(0);
    trace->KalmanInfo.MeasPre(1) = trace->KalmanInfo.StateEst(1);
    trace->KalmanInfo.MeasPre(2) =
        (trace->KalmanInfo.StateEst(iDistLat) *
             trace->KalmanInfo.StateEst(iVrelLat) +
         trace->KalmanInfo.StateEst(iDistLong) *
             trace->KalmanInfo.StateEst(iVrelLong)) /
        (sqrt(pow(trace->KalmanInfo.StateEst(iDistLat), 2.0) +
              pow(trace->KalmanInfo.StateEst(iDistLong), 2.0)));

    ComputeCornerPos(
        trace->KalmanInfo.StateEst(iDistLat),
        trace->KalmanInfo.StateEst(iDistLong), trace->ExtendInfo.box_theta,
        trace->ExtendInfo.Width * 0.5F, trace->ExtendInfo.Length * 0.5F,
        &trace->ExtendInfo.CornerPos[0][0]);

    ErrorX = compute_diif_by_closest(trace, meas_mat);

    ErrorX(1) = Valuelimit(-1.0, 1.0, ErrorX(1));
    add_val = K * ErrorX;

    //        double lat_v_by_diff_lat = ErrorX(0) * K(2, 0);
    double lat_v_by_diff_long = ErrorX(1) * K(2, 1);

    //        if((lat_v_by_diff_lat * lat_v_by_diff_long) < 0.0) // 不同向
    //        {
    add_val(2) -= lat_v_by_diff_long;
    //        }

    // 根据当前残差数值设置QR调整方向
    adpative_QR_logic(trace, ErrorX);

    // 视情况调整增益数值
    adjust_diff(trace, ErrorX, add_val);
  }

  // 处于FOV边缘的远离的航迹，若产生较大的纵向位置差，则取消本次更新
  if ((trace->ExtendInfo.corner_type & (1 << ALL_VISIBLE)) !=
      (1 << ALL_VISIBLE)) {
    if (trace->KalmanInfo.StateEst(iVrelLong) < -1.0) {
      add_val(2) = 0.0;
      add_val(4) = 0.0;
      add_val(3) = 0.0;
      add_val(5) = 0.0;
    }
  }

  // 状态更新
  double size = trace->ExtendInfo.Width * trace->ExtendInfo.Length;
  size = Valuelimit(1.0, 40, size);
  double dec_fac = (trace->MeasInfo.detInfo.size() + 1.0) * 3.0 / size;

  dec_fac = Valuelimit(0.5, 1.0, dec_fac);

  // 状态更新
  trace->KalmanInfo.StateEst = trace->KalmanInfo.StateEst + add_val * dec_fac;

#ifdef USING_EKF_FILTER
  trace->KalmanInfo.P_ = (trace->KalmanInfo.P_ -
                          K * trace->KalmanInfo.JacMat * trace->KalmanInfo.P_);
#else
  trace->KalmanInfo.P_ = (trace->KalmanInfo.P_ - K * S * K.transpose());
#endif

#if 1
  if (trace->chosed == true) {
    std::cout << "---------------------------------" << std::endl;
    std::cout << "trkInfo->KalmanInfo.K_gain" << std::endl << K << std::endl;
    std::cout << "Error_Y_hX" << std::endl << ErrorX << std::endl;
    std::cout << "add" << std::endl << add_val << std::endl;
    std::cout << "motion " << trace->ExtendInfo.DynProp << std::endl;
    std::cout << "---------------------------------" << std::endl;
  }
#endif
}

/**
 * @name:
 * @description:
 * @param {trackTable_strcut} *trace
 * @param {MatrixXd} meas_mat
 * @return {*}
 */
Eigen::VectorXd Tracker::compute_diif_by_center(trackTable_strcut *trace,
                                                const MatrixXd meas_mat) {
  VectorXd ErrorX = VectorXd::Zero(MSIZE);

  VectorXd final_Z = VectorXd::Zero(MSIZE);
  VectorXd final_X = VectorXd::Zero(MSIZE);

  // 计算量测中心
  MatrixXd meas_center = meas_mat.colwise().mean();
  VectorXd mid_info =
      (meas_mat.colwise().maxCoeff() + meas_mat.colwise().minCoeff()) * 0.5;

  final_Z << mid_info(0), mid_info(1), meas_center(2);

  final_X = trace->KalmanInfo.MeasPre;

  ErrorX = final_Z - final_X;

  if (trace->ExtendInfo.Object_Class == PEDESTRIAN) {
    for (uint8_t idx = 0; idx < meas_mat.rows(); idx++) {
      if (fabs(meas_mat(idx, 2) - final_X(2)) < fabs(ErrorX(2))) {
        ErrorX(2) = meas_mat(idx, 2) - final_X(2);
      }
    }
  }

  return ErrorX;
}

/**
 * @name:
 * @description:
 * @param {trackTable_strcut} *trace
 * @param {MatrixXd} meas_mat
 * @return {*}
 */
Eigen::VectorXd Tracker::compute_diif_by_closest(trackTable_strcut *trace,
                                                 const MatrixXd meas_mat) {
  VectorXd ErrorX = VectorXd::Zero(MSIZE);

  VectorXd final_Z_by_center = VectorXd::Zero(MSIZE);
  VectorXd final_Z_by_corner = VectorXd::Zero(MSIZE);
  VectorXd final_X_by_center = VectorXd::Zero(MSIZE);
  VectorXd final_X_by_corner = VectorXd::Zero(MSIZE);

  // 计算量测最近点位置
  // 计算量测中心
  MatrixXd meas_center = meas_mat.colwise().mean();
  std::vector<double> meas_lat_pos, meas_long_pos;
  for (uint16_t idx = 0; idx < meas_mat.rows(); idx++) {
    meas_lat_pos.push_back(meas_mat(idx, 0));
    meas_long_pos.push_back(meas_mat(idx, 1));
  }

  auto meas_lat_minmax =
      std::minmax_element(meas_lat_pos.begin(), meas_lat_pos.end());
  auto meas_long_minmax =
      std::minmax_element(meas_long_pos.begin(), meas_long_pos.end());

  double box_wid = (*meas_lat_minmax.second - *meas_lat_minmax.first);
  double box_len = (*meas_long_minmax.second - *meas_long_minmax.first);

  MatrixXd meas_corner = MatrixXd(4, 2);
  meas_center(0) = (*meas_lat_minmax.second + *meas_lat_minmax.first) * 0.5;
  meas_center(1) = (*meas_long_minmax.second + *meas_long_minmax.first) * 0.5;
  meas_corner(0, 0) = meas_center(0) + box_wid * 0.5;
  meas_corner(0, 1) = meas_center(1) + box_len * 0.5;
  meas_corner(1, 0) = meas_center(0) - box_wid * 0.5;
  meas_corner(1, 1) = meas_center(1) + box_len * 0.5;
  meas_corner(2, 0) = meas_center(0) - box_wid * 0.5;
  meas_corner(2, 1) = meas_center(1) - box_len * 0.5;
  meas_corner(3, 0) = meas_center(0) + box_wid * 0.5;
  meas_corner(3, 1) = meas_center(1) - box_len * 0.5;

  double min_range = 1e3;
  uint8_t det_min_idx = 0;
  for (uint8_t idx = 0; idx < 4; idx++) {
    double tempRange =
        sqrt(pow(meas_corner(idx, 0), 2.0) + pow(meas_corner(idx, 1), 2.0));
    if (tempRange < min_range) {
      min_range = tempRange;
      det_min_idx = idx;
    }
  }

  // 量测中心
  final_Z_by_center << meas_center(0), meas_center(1), meas_center(2);

  // 量测最近点
  final_Z_by_corner << meas_corner(det_min_idx, 0), meas_corner(det_min_idx, 1),
      meas_center(2);

  // 计算航迹最近点位置
  min_range = 1e3;
  uint8_t trace_min_idx = 0;
  std::vector<double> trace_lat_pos, trace_long_pos;
  for (uint8_t idx = 0; idx < 4; idx++) {
    double tempRange = sqrt(pow(trace->ExtendInfo.CornerPos[idx * 2][0], 2.0) +
                            pow(trace->ExtendInfo.CornerPos[idx * 2][1], 2.0));

    trace_lat_pos.push_back(trace->ExtendInfo.CornerPos[idx * 2][0]);
    trace_long_pos.push_back(trace->ExtendInfo.CornerPos[idx * 2][1]);

    if (tempRange < min_range) {
      min_range = tempRange;
      trace_min_idx = idx;
    }
  }

  auto trace_lat_minmax =
      std::minmax_element(trace_lat_pos.begin(), trace_lat_pos.end());
  auto trace_long_minmax =
      std::minmax_element(trace_long_pos.begin(), trace_long_pos.end());

  // 航迹中心
  final_X_by_center = trace->KalmanInfo.MeasPre;

  // 航迹最近点
  final_X_by_corner << trace->ExtendInfo.CornerPos[trace_min_idx * 2][0],
      trace->ExtendInfo.CornerPos[trace_min_idx * 2][1],
      trace->KalmanInfo.MeasPre(2);

  if ((trace->ExtendInfo.corner_type & (1 << ALL_VISIBLE)) ==
      (1 << ALL_VISIBLE)) {
    // 航迹or量测横穿左右两个象限
    if (((*meas_lat_minmax.second * *meas_lat_minmax.first) <= 0.0) ||
        ((*trace_lat_minmax.second * *trace_lat_minmax.first) <= 0.0)) {
      final_Z_by_corner << meas_center(0), *meas_long_minmax.first,
          meas_center(2);
      final_X_by_corner << trace->KalmanInfo.MeasPre(0),
          *trace_long_minmax.first, trace->KalmanInfo.MeasPre(2);

      ErrorX = final_Z_by_corner - final_X_by_corner;  // TODO
    } else {
      if (det_min_idx == trace_min_idx) {
        ErrorX = final_Z_by_corner - final_X_by_corner;
      } else {
        ErrorX = final_Z_by_center - final_X_by_center;
      }
    }
  } else {
    if ((trace->ExtendInfo.corner_type & (1 << ALL_IN_POSITIVE_SIDE)) ==
        (1 << ALL_IN_POSITIVE_SIDE)) {
      final_Z_by_corner << *meas_lat_minmax.first, *meas_long_minmax.second,
          meas_center(2);
      final_X_by_corner << *trace_lat_minmax.first, *trace_long_minmax.second,
          trace->KalmanInfo.MeasPre(2);

      ErrorX = final_Z_by_corner - final_X_by_corner;  // TODO

    } else if ((trace->ExtendInfo.corner_type & (1 << ALL_IN_NEGATIVE_SIDE)) ==
               (1 << ALL_IN_NEGATIVE_SIDE)) {
      final_Z_by_corner << *meas_lat_minmax.second, *meas_long_minmax.second,
          meas_center(2);
      final_X_by_corner << *trace_lat_minmax.second, *trace_long_minmax.second,
          trace->KalmanInfo.MeasPre(2);

      ErrorX = final_Z_by_corner - final_X_by_corner;
    } else {
      ErrorX = final_Z_by_center - final_X_by_center;
    }
  }

  if (trace->trackState == TRACK_STATE_ACTIVE) {
    ErrorX(0) =
        ErrorX(0) * 0.8 + ((final_Z_by_center(0) - final_X_by_center(0)) * 0.2);
  } else {
    ErrorX(0) =
        ErrorX(0) * 0.6 + ((final_Z_by_center(0) - final_X_by_center(0)) * 0.4);
  }

  return ErrorX;
}

/**
 * @name:
 * @description:
 * @param {trackTable_strcut} *trace
 * @param {MatrixXd} meas_mat
 * @return {*}
 */
Eigen::VectorXd Tracker::compute_diif_by_side(trackTable_strcut *trace,
                                              const Eigen::MatrixXd meas_mat) {
  VectorXd ErrorX = VectorXd::Zero(MSIZE);
  VectorXd final_Z = VectorXd::Zero(MSIZE);
  VectorXd final_X = VectorXd::Zero(MSIZE);

  double temp_theta = trace->ExtendInfo.box_theta;

  if (fabs(trace->ExtendInfo.box_theta) > (0.5 * RADAR_PI)) {
    temp_theta =
        (temp_theta > 0.0) ? (temp_theta - RADAR_PI) : (temp_theta + RADAR_PI);
  }

  // 计算量测中心
  MatrixXd meas_center = meas_mat.colwise().mean();

  std::vector<double> lat_pos, long_pos;
  for (uint8_t idx = 0; idx < meas_mat.rows(); idx++) {
    lat_pos.push_back(meas_mat(idx, 0));
    long_pos.push_back(meas_mat(idx, 1));
  }

  auto meas_lat_minmax = std::minmax_element(lat_pos.begin(), lat_pos.end());
  auto meas_long_minmax = std::minmax_element(long_pos.begin(), long_pos.end());

  // 量测中心
  final_Z << (*meas_lat_minmax.first + *meas_lat_minmax.second) * 0.5,
      (*meas_long_minmax.first + *meas_long_minmax.second) * 0.5,
      meas_center(2);

  final_X = trace->KalmanInfo.MeasPre;  // 预测中心

  if (trace->trackState == TRACK_STATE_DETECTION) {
    // 未确认的远离目标（超车目标）
    if ((trace->KalmanInfo.MeasPre(iVEL) > 1.0) &&
        (fabs(trace->KalmanInfo.StateEst(iDistLong)) < 10.0)) {
      // 纵向：
      final_Z(1) = *meas_long_minmax.second;
      final_X(1) =
          (trace->KalmanInfo.MeasPre(1) + 0.4 * trace->ExtendInfo.Length);

      // 目标完全在左侧
      if ((trace->ExtendInfo.corner_type & (1 << ALL_IN_POSITIVE_SIDE)) ==
          (1 << ALL_IN_POSITIVE_SIDE)) {
        // 横向：
        final_Z(0) = *meas_lat_minmax.first;  // 选择量测最小值（最右侧）
        final_X(0) =
            (trace->KalmanInfo.MeasPre(0) - 0.5 * trace->ExtendInfo.Width);
      }

      //目标完全在右侧
      else if ((trace->ExtendInfo.corner_type & (1 << ALL_IN_NEGATIVE_SIDE)) ==
               (1 << ALL_IN_NEGATIVE_SIDE)) {
        // 横向：
        final_Z(0) = *meas_lat_minmax.second;  // 选择量测最大值（最左侧）
        final_X(0) =
            (trace->KalmanInfo.MeasPre(0) + 0.5 * trace->ExtendInfo.Width);
      }
    }
  } else {
    // 已经确认的横穿航迹
    if ((trace->ExtendInfo.DynProp == crossing_moving) ||
        (trace->ExtendInfo.DynProp == crossing_stationary)) {
      // 纵向：
      final_Z(1) = *meas_long_minmax.first;
      final_X(1) =
          (trace->KalmanInfo.MeasPre(1) - 0.5 * trace->ExtendInfo.Width);

      if ((trace->ExtendInfo.corner_type & (1 << ALL_IN_POSITIVE_SIDE)) ==
          (1 << ALL_IN_POSITIVE_SIDE)) {
        // 横向：
        final_Z(0) =
            (*meas_lat_minmax.first + meas_center(0)) * 0.5;  // 选择量测最小值
        final_X(0) =
            (trace->KalmanInfo.MeasPre(0) +
             (trace->KalmanInfo.MeasPre(0) - 0.5 * trace->ExtendInfo.Length)) *
            0.5;
      } else if ((trace->ExtendInfo.corner_type &
                  (1 << ALL_IN_NEGATIVE_SIDE)) == (1 << ALL_IN_NEGATIVE_SIDE)) {
        // 横向：
        final_Z(0) =
            (*meas_lat_minmax.second + meas_center(0)) * 0.5;  // 选择量测最小值
        final_X(0) =
            (trace->KalmanInfo.MeasPre(0) +
             (trace->KalmanInfo.MeasPre(0) + 0.5 * trace->ExtendInfo.Length)) *
            0.5;
      } else {
        // 有一部分在左侧，有一部分在右侧
        // nothing to do
      }
    } else {
      if ((fabs(temp_theta) < (20.0 * DEG2RAD)) ||
          (fabs(temp_theta) > (160.0 * DEG2RAD)))  // 视作直来直往
      {
        if ((trace->ExtendInfo.corner_type & (1 << ALL_VISIBLE)) ==
            (1 << ALL_VISIBLE)) {
          if (fabs(trace->KalmanInfo.MeasPre(0)) > 2.0) {
            // 全在左侧（正）
            if ((trace->ExtendInfo.corner_type & (1 << ALL_IN_POSITIVE_SIDE)) ==
                (1 << ALL_IN_POSITIVE_SIDE)) {
              final_Z(0) = *meas_lat_minmax.first;  // 选择量测最小值
              final_X(0) = (trace->KalmanInfo.MeasPre(0) -
                            0.5 * trace->ExtendInfo.Width);

              // 纵向：
              final_Z(1) = *meas_long_minmax.first;
              final_X(1) = (trace->KalmanInfo.MeasPre(1) -
                            0.5 * trace->ExtendInfo.Length * cos(temp_theta));
            } else if ((trace->ExtendInfo.corner_type &
                        (1 << ALL_IN_NEGATIVE_SIDE)) ==
                       (1 << ALL_IN_NEGATIVE_SIDE))  // 全在负侧
            {
              final_Z(0) = *meas_lat_minmax.second;  // 选择量测最大值
              final_X(0) = (trace->KalmanInfo.MeasPre(0) +
                            0.5 * trace->ExtendInfo.Width);

              // 纵向：
              final_Z(1) = *meas_long_minmax.first;
              final_X(1) = (trace->KalmanInfo.MeasPre(1) -
                            0.5 * trace->ExtendInfo.Length * cos(temp_theta));
            } else {
              // nothing to do
            }
          } else {
            /* 若目标处于正前方且航向角比较小，则在纵向距离上使用最近点 */
            final_Z(1) = *meas_long_minmax.first;
            final_X(1) =
                (trace->KalmanInfo.MeasPre(1) - 0.5 * trace->ExtendInfo.Length);
          }
        } else {
          // 全在左侧（正）
          if ((trace->ExtendInfo.corner_type & (1 << ALL_IN_POSITIVE_SIDE)) ==
              (1 << ALL_IN_POSITIVE_SIDE)) {
            final_Z(0) = *meas_lat_minmax.first;  // 选择量测最小值
            final_X(0) =
                (trace->KalmanInfo.MeasPre(0) - 0.5 * trace->ExtendInfo.Width);

            // 纵向：
            final_Z(1) = *meas_long_minmax.second;
            final_X(1) =
                (trace->KalmanInfo.MeasPre(1) + 0.5 * trace->ExtendInfo.Length);
          } else if ((trace->ExtendInfo.corner_type &
                      (1 << ALL_IN_NEGATIVE_SIDE)) ==
                     (1 << ALL_IN_NEGATIVE_SIDE))  // 全在负侧
          {
            final_Z(0) = *meas_lat_minmax.second;  // 选择量测最大值
            final_X(0) =
                (trace->KalmanInfo.MeasPre(0) + 0.5 * trace->ExtendInfo.Width);

            // 纵向：
            final_Z(1) = *meas_long_minmax.second;
            final_X(1) =
                (trace->KalmanInfo.MeasPre(1) + 0.5 * trace->ExtendInfo.Length);
          } else {
            // nothing to do
          }
        }
      } else {
        // nothing to do
      }
    }
  }

  ErrorX = final_Z - final_X;
  return ErrorX;
}

/**
 * @name:
 * @description:
 * @param {trackTable_strcut} *trace
 * @param {VectorXd} &ErrorX
 * @param {VectorXd} &add_val
 * @return {*}
 */
void Tracker::adjust_diff(trackTable_strcut *trace, Eigen::VectorXd &ErrorX,
                          Eigen::VectorXd &add_val) {
  double scale_ = 1.0;
  bool has_close_to_zero = false;

  double add_factor = trace->MeasInfo.x_diff_dir;

  add_factor = Valuelimit(-10.0, 10.0, add_factor);

  if (trace->ExtendInfo.Object_Class < VEHICLE) {
    return;
  }

#if 1
  if ((trace->trackState == TRACK_STATE_ACTIVE) &&
      ((trace->ExtendInfo.DynProp == crossing_moving) ||
       (trace->ExtendInfo.DynProp == crossing_stationary))) {
    // TODO : 可以尝试修改横向噪声
    if ((fabs(trace->KalmanInfo.StateEst(iDistLat)) -
         (0.5 * trace->ExtendInfo.Length)) < 5.0) {
      has_close_to_zero = true;
    }

    // 横跨左右两侧
    if (((trace->ExtendInfo.corner_type & (1 << PASS_THROUGH_LAT)) ==
         (1 << PASS_THROUGH_LAT)) ||
        has_close_to_zero) {
      // (减少往回拽的力度)
      if ((trace->KalmanInfo.StateEst(iVrelLat) * ErrorX(0)) < 0.0) {
        if (ErrorX(0) > 0.0) {
          scale_ = 0.1 + add_factor * 0.01;
        } else {
          scale_ = 0.1 + add_factor * 0.01;
        }
      } else {
        // 同方向
        if (ErrorX(0) > 0.0) {
          scale_ = 0.4 + trace->MeasInfo.x_diff_dir * 0.02;
        } else {
          scale_ = 0.4 + trace->MeasInfo.x_diff_dir * 0.02;
        }
      }

      add_val(iDistLat) *= scale_;
      add_val(iVrelLat) *= scale_;
      add_val(iAccLat) *= scale_;
    } else {
      if ((trace->KalmanInfo.StateEst(iVrelLat) * ErrorX(0)) < 0.0)  // 反向回拽
      {
        if (ErrorX(0) > 0.0) {
          scale_ = 0.4 + add_factor * 0.04;
        } else {
          scale_ = 0.4 + add_factor * 0.04;
        }
      } else  // 同向补偿
      {
        if (ErrorX(0) > 0.0) {
          scale_ = 0.6 + trace->MeasInfo.x_diff_dir * 0.01;
        } else {
          scale_ = 0.6 + trace->MeasInfo.x_diff_dir * 0.01;
        }
      }

      add_val(iDistLat) *= scale_;
      add_val(iVrelLat) *= scale_;
      add_val(iAccLat) *= scale_;

      add_val(iDistLat) = Valuelimit(-0.5, 0.5, add_val(iDistLat));
      add_val(iVrelLat) = Valuelimit(-0.5, 0.5, add_val(iVrelLat));
      add_val(iAccLat) = Valuelimit(-0.5, 0.5, add_val(iAccLat));
    }
  }
#endif
}

/**
 * @name:
 * @description:
 * @param {trackTable_strcut} *trace
 * @param {VectorXd} ErrorX
 * @return {*}
 */
void Tracker::adpative_QR_logic(trackTable_strcut *trace,
                                const Eigen::VectorXd ErrorX) {
  if (fabs(ErrorX(iDistLat)) > 0.4) {
    trace->MeasInfo.x_diff_dir += fabs(ErrorX(iDistLat));
  } else {
    trace->MeasInfo.x_diff_dir = trace->MeasInfo.x_diff_dir * 0.8;
  }

  if (fabs(ErrorX(iDistLong)) > 0.4) {
    trace->MeasInfo.y_diff_dir += fabs(ErrorX(iDistLong));
  } else {
    trace->MeasInfo.y_diff_dir = trace->MeasInfo.y_diff_dir * 0.9;
  }

  if (fabs(ErrorX(2)) > 0.3) {
    trace->MeasInfo.v_diff_dir += fabs(ErrorX(2));
  } else {
    trace->MeasInfo.v_diff_dir = trace->MeasInfo.v_diff_dir * 0.9;
  }

  // 范围限定
  trace->MeasInfo.x_diff_dir =
      Valuelimit(-10.0, 10.0, trace->MeasInfo.x_diff_dir);
  trace->MeasInfo.y_diff_dir =
      Valuelimit(-10.0, 10.0, trace->MeasInfo.y_diff_dir);
  trace->MeasInfo.v_diff_dir =
      Valuelimit(-5.0, 5.0, trace->MeasInfo.v_diff_dir);
}
