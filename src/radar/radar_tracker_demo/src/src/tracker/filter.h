/*
 * @Description:
 * @version:
 * @Author: ChengHao
 * @Date: 2022-10-12 08:58:47
 * @LastEditors: ChengHao hao.cheng@wuzheng.com
 * @LastEditTime: 2023-11-03 14:50:12
 */
#ifndef FILTER_H
#define FILTER_H

#include "dataStruct.h"


//#define USING_EKF_FILTER
#define USING_UKF_FILTER

#define _X_Y_V
// #define _R_A_V_

class Tracker
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  int n_x_;
  int n_z_;

  const double alpha = 0.01;
  const double beta = 2.0;
  const double kappa = 0.0;
  double lambda = 0.0;

  Eigen::VectorXd Wc = Eigen::VectorXd(2 * SSIZE + 1);
  Eigen::VectorXd Wm = Eigen::VectorXd(2 * SSIZE + 1);

  vehicleInfo_struct *vehicleInfo_;
  uint32_t yaw_rate_delay = 0;

  Tracker();

  virtual ~Tracker();

  void EKF_Predict(double delta_t, trackTable_strcut *trace, vehicleInfo_struct *vehicleInfo);
  void UKF_Predict(double delta_t, trackTable_strcut *trace, vehicleInfo_struct *vehicleInfo);

  void CalcJacobian(trackTable_strcut *trace);
  void EKF_Compute_S_K(trackTable_strcut *trace);

  void Prediction(double delta_t, trackTable_strcut *trace,
                  vehicleInfo_struct *vehicleInfo);

  void UpdateState(trackTable_strcut *trace, std::vector<RadarMeasure_struct> &global_point);

  MatrixXd compute_F(trackTable_strcut *trace, double dt);

  void UKF_Compute_S_K(trackTable_strcut *trace);

  void RandomMatrices_UKF(std::vector<RadarMeasure_struct> &global_point,
                          trackTable_strcut *trace);
  MatrixXd Compute_RMM_SPDMat(trackTable_strcut *trace);

  void ComputeNewS_RMMandCorners(trackTable_strcut *trace);

  Eigen::VectorXd compute_diif_by_center(trackTable_strcut *trace, const Eigen::MatrixXd meas_mat);
  Eigen::VectorXd compute_diif_by_side(trackTable_strcut *trace, const Eigen::MatrixXd meas_mat);
  Eigen::VectorXd compute_diif_by_closest(trackTable_strcut *trace, const MatrixXd meas_mat);
  void adjust_diff(trackTable_strcut *trace, Eigen::VectorXd &ErrorX, Eigen::VectorXd &add_val);

  void adpative_QR_logic(trackTable_strcut *trace, const Eigen::VectorXd ErrorX);
};

#endif
