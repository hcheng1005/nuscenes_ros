#include "../../include/tracker/kalman.h"

using namespace Eigen;

bool TRACK_DEBUG = false;

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @param {VectorXd} X
 * @param {MatrixXd} P
 * @return {*}
 */
simple_tracker::simple_tracker(Eigen::VectorXd X, Eigen::MatrixXd P,
                               uint32_t id) {
  this->X_ = X;
  this->P_ = P;

  this->H_ = Eigen::MatrixXd::Zero(Z_DIM, X_DIM);
  this->H_.block<Z_DIM, Z_DIM>(0, 0) = Eigen::MatrixXd::Identity(Z_DIM, Z_DIM);

  this->track_manage.id = id;
  this->track_manage.age = 1;
  this->track_manage.unassigned_count = 0;
  this->track_manage.track_status = TRK_Detected;
  this->track_manage.continue_assigned_count = 1;
}

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @param {box_t} &new_det
 * @return {*}
 */
simple_tracker::simple_tracker(box_t &new_det, uint32_t id) {
  this->X_ << new_det.rect.center_pos[0], new_det.rect.center_pos[1],
      new_det.rect.center_pos[2], new_det.rect.heading, new_det.rect.box_len,
      new_det.rect.box_wid, new_det.rect.box_height, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0;

  this->P_ = Eigen::MatrixXd::Zero(X_DIM, X_DIM);
  this->P_.block<3, 3>(0, 0) = Eigen::MatrixXd::Identity(3, 3) * 0.4;
  this->P_.block<1, 1>(3, 3) =
      Eigen::MatrixXd::Identity(1, 1) * pow(10.0 / 180.0 * M_PI, 2.0);
  this->P_.block<3, 3>(4, 4) = Eigen::MatrixXd::Identity(3, 3) * 1;
  this->P_.block<3, 3>(7, 7) = Eigen::MatrixXd::Identity(3, 3) * 1.0;
  this->P_.block<3, 3>(10, 10) = Eigen::MatrixXd::Identity(3, 3) * 1.0;

  this->H_ = Eigen::MatrixXd::Zero(Z_DIM, X_DIM);
  this->H_.block<Z_DIM, Z_DIM>(0, 0) = Eigen::MatrixXd::Identity(Z_DIM, X_DIM);

  // 量测噪声
  this->R_ = Eigen::MatrixXd::Zero(Z_DIM, Z_DIM);
  this->R_.block<3, 3>(0, 0) = Eigen::MatrixXd::Identity(3, 3) * pow(0.1, 2.0);
  this->R_(1, 1) = pow(0.1, 2.0);  // 提高横向距离噪声
  this->R_(2, 2) = pow(0.4, 2.0);  // 减少高度距离噪声

  this->R_(3, 3) = pow((2.0 / 180.0 * M_PI), 2.0);  // 形状航向角噪声
  this->R_.block<3, 3>(4, 4) =
      Eigen::MatrixXd::Identity(3, 3) * pow(1.0, 2.0);  // lwh形状噪声

  this->track_manage.id = id;
  this->track_manage.age = 1;
  this->track_manage.exsit_prob = new_det.score;
  this->track_manage.unassigned_count = 0;
  this->track_manage.track_status = TRK_Detected;
  this->track_manage.continue_assigned_count = 1;

  this->track_manage.shape_mana.len_max = new_det.rect.box_len;
  this->track_manage.shape_mana.wid_max = new_det.rect.box_wid;

  this->track_manage.type_manage.change_type_count = 0;
  this->track_manage.type_manage.type =
      static_cast<inner_type_enum>(new_det.type);
  this->track_manage.type_manage.type_new = this->track_manage.type_manage.type;
  this->track_manage.type_manage.score = new_det.score;


  this->track_manage.shape4out.theta = new_det.rect.heading;

  if (TRACK_DEBUG) {
    std::cout << "build a new Trace: " << this->X_.transpose() << std::endl;
  }
}

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @param {double} diff_time
 * @return {*}
 */
void simple_tracker::trace_predict(double dt) {
  diff_t = dt;
  compute_F(diff_t);
  Update_Q();
  Predict_(this->X_, this->P_, this->F_, this->Q_);
}

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @param {double} dt
 * @return {*}
 */
void simple_tracker::compute_F(const double dt) {
  double alpha_long = 1.0, alpha_lat = 1.0;
  double tmp_Q_long = fabs(track_manage.diff_x) / 1.0,
         tmp_Q_lat = fabs(track_manage.diff_y) / 1.0;
  double vel_scale_long = 1.0, vel_scale_lat = 1.0;
  double exp_alpha_long_T = 1.0, exp_alpha_lat_T = 1.0;

  if (fabs(track_manage.diff_x) < 0.1) {
    alpha_long = 10.0 + exp(tmp_Q_long / 0.1);
    exp_alpha_long_T = exp(-1.0 * alpha_long * dt);
  }

  if (fabs(track_manage.diff_y) < 0.1) {
    alpha_lat = 10.0 + exp(tmp_Q_lat);
    exp_alpha_lat_T = exp(-1.0 * alpha_lat * dt);
  }

  this->F_ << 1, 0, 0, 0, 0, 0, 0, dt, 0, 0, 0.5 * pow(dt, 2.0), 0, 0, 0, 1, 0,
      0, 0, 0, 0, 0, dt, 0, 0, 0.5 * pow(dt, 2.0), 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
      dt, 0, 0, 0.5 * pow(dt, 2.0), 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, vel_scale_long,
      0, 0, dt, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, vel_scale_lat, 0, 0, dt, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0.8, 0, 0, dt, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      exp_alpha_long_T, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, exp_alpha_lat_T,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.8;
}

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @return {*}
 */
void simple_tracker::Update_Q(void) {
  MatrixXd G = MatrixXd::Zero(9, 3);
  MatrixXd q = MatrixXd::Zero(3, 3);

  double dt = 0.1;
  double dt3 = powf(dt, 3.0);
  double dt2 = powf(dt, 2.0);

  double std_accLat_, std_accLong_, std_accZ;
  std_accLat_ = 0.04;
  std_accLong_ = 0.04;
  std_accZ = 0.1;  // 提高Z方向过程噪声

  if (this->track_manage.track_status == TRK_Confirmed) {
    double count_long_, count_lat_;
    count_long_ =
        (adaptive_Q.count_long <= 0) ? (0.0) : (adaptive_Q.count_long - 1);
    count_lat_ =
        (adaptive_Q.count_lat <= 0) ? (0.0) : (adaptive_Q.count_lat - 1);

    double new_R_lat, new_R_long;

    if (this->track_manage.age < 10) {
      std_accLong_ = 0.08;
      std_accLat_ = 0.08;
    } else {
      if (vehilce_info.vehicle_flag) {
        if (track_manage.type_manage.type == inner_type_enum::VEHICLE) {
          std_accLong_ += (fabs(track_manage.diff_x));
          std_accLat_ += (fabs(track_manage.diff_y));
        } else {
          std_accLong_ += (fabs(track_manage.diff_x));
          std_accLat_ += (fabs(track_manage.diff_y));
        }

        new_R_long = 0.10 - count_long_ * 0.015;
        new_R_long = value_limit(0.02, 0.1, new_R_long);
        new_R_lat = 0.10 - count_lat_ * 0.015;
        new_R_lat = value_limit(0.02, 0.1, new_R_lat);

        R_(0, 0) = pow(new_R_long, 2.0);
        R_(1, 1) = pow(new_R_lat, 2.0);
      } else {
        if (track_manage.type_manage.type == inner_type_enum::VEHICLE) {
          std_accLong_ += (fabs(track_manage.diff_x) / 4.0);
          std_accLat_ += (fabs(track_manage.diff_y) / 8.0);
        } else {
          std_accLong_ += (fabs(track_manage.diff_x) / 4.0);
          std_accLat_ += (fabs(track_manage.diff_y) / 4.0);
        }

        new_R_long = 0.10 - count_long_ * 0.01;
        new_R_long = value_limit(0.04, 0.1, new_R_long);
        new_R_lat = 0.10 - count_lat_ * 0.01;
        new_R_lat = value_limit(0.04, 0.1, new_R_lat);

        R_(0, 0) = pow(new_R_long, 2.0);
        R_(1, 1) = pow(new_R_lat, 2.0);
      }
    }
  }

  G << 1 / 6 * dt3, 0, 0, 0, 1 / 6 * dt3, 0, 0, 0, 1 / 6 * dt3, 1 / 2 * dt2, 0,
      0, 0, 1 / 2 * dt2, 0, 0, 0, 1 / 2 * dt2, dt, 0, 0, 0, dt, 0, 0, 0, dt;

  q << pow(std_accLong_, 2.0), 0.0, 0.0, 0.0, pow(std_accLat_, 2.0), 0.0, 0.0,
      0.0, pow(std_accZ, 2.0);

  MatrixXd temp_Q = G * q * G.transpose();

  this->Q_.block<3, 3>(0, 0) = temp_Q.block<3, 3>(0, 0);

  this->Q_(3, 3) = pow((0.4 / 180.0 * M_PI), 2.0);  // heading
  this->Q_.block<3, 3>(4, 4) =
      Eigen::MatrixXd::Identity(3, 3) * pow(0.2, 2.0);  // l,w,h

  this->Q_(6, 6) = pow(0.2, 2.0);  // 提高box高度过程噪声
  this->Q_.block<3, 6>(0, 7) = temp_Q.block<3, 6>(0, 3);
  this->Q_.block<6, 3>(7, 0) = temp_Q.block<6, 3>(3, 0);  // vel noise
  this->Q_.block<6, 6>(7, 7) = temp_Q.block<6, 6>(3, 3);  // acc noise

  if (0) {
    std::cout << "obj ID [" << std::to_string(this->track_manage.id) << "], "
              << "Q_long [" << std::to_string(adaptive_Q.count_long) << "], "
              << "diff_x [" << std::to_string(track_manage.diff_x) << "], "
              << "Q_lat [" << std::to_string(adaptive_Q.count_lat) << "], "
              << "diff_y [" << std::to_string(track_manage.diff_y) << "], "
              << std::endl;
  }
}

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @return {*}
 */
bool simple_tracker::IsAnyPartInSpecialArea(void) {
  Eigen::MatrixXd orin_pts(4, 2);
  Eigen::Matrix2d rota_mat(2, 2);
  Eigen::MatrixXd rota_pts(2, 4);

  double half_len = X_(iBoxLen) * 0.5, half_wid = X_(iBoxWid) * 0.5;

  orin_pts << +half_len, +half_wid, +half_len, -half_wid, -half_len, -half_wid,
      -half_len, +half_wid;

  rota_mat << cos(X_(iBoxHeading)), -sin(X_(iBoxHeading)), sin(X_(iBoxHeading)),
      cos(X_(iBoxHeading));

  rota_pts = rota_mat * orin_pts.transpose();

  for (uint8_t idx = 0; idx < 4; idx++) {
    if (((rota_pts(0, idx) + X_(iDisLong)) < 3.0) &&
        ((rota_pts(0, idx) + X_(iDisLong)) > -1.0)) {
      if (fabs(rota_pts(1, idx) + X_(iDisLat)) < 4.0) {
        // flag_ = true;
        return true;
      }
    }
  }

  return false;
}

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @return {*}
 */
void simple_tracker::Predict_(Eigen::VectorXd &X_, Eigen::MatrixXd &P_,
                              Eigen::MatrixXd F_, Eigen::MatrixXd Q_) {
  X_cp = X_;
  X_ = F_ * X_;

  theta_ = atan2(X_(iDisLat), X_(iDisLong));
}

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @return {*}
 */
void simple_tracker::Update_(void) {
  if (matched_info.matched_det_list.size() > 0) {
    trace_update_with_det();
  } else {
    trace_update_without_det();
  }
}

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @param {VectorXd} Z
 * @return {*}
 */
void simple_tracker::trace_update_with_det(void) {
  Eigen::VectorXd Z(Z_DIM);
  auto new_det = matched_info.matched_det_list.at(0);

  Z << new_det.rect.center_pos[0], new_det.rect.center_pos[1],
      new_det.rect.center_pos[2], new_det.rect.heading, new_det.rect.box_len,
      new_det.rect.box_wid, new_det.rect.box_height;

  compute_newZ(Z);

  if (track_manage.type_manage.type == inner_type_enum::VEHICLE) {
    // do l-shape tracking
    // l_shape_tracker(Z);
    center_tracker(Z);
  } else {
    center_tracker(Z);
  }

  // 跟踪后处理
  tracker_postprocess();
}

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @param {VectorXd} &new_Z
 * @return {*}
 */
void simple_tracker::compute_newZ(Eigen::VectorXd &new_Z) {
  double trans_angle1, trans_angle2, angle_diff1, angle_diff2;
  angle_diff1 = (new_Z(iBoxHeading) - X_(iBoxHeading));

  if (fabs(angle_diff1) > M_PI) {
    angle_diff1 = (angle_diff1 > 0.0) ? (angle_diff1 - 2 * M_PI)
                                      : (angle_diff1 + 2 * M_PI);
  }

  // std::cout << "trace ID: " << track_manage.id << std::endl;

  // std::cout << X_(iBoxHeading) / M_PI * 180.0 << ", "
  //           << new_Z(iBoxHeading) / M_PI * 180.0 << std::endl;

  if (track_manage.track_status == TRK_Detected) {
    trans_angle1 = X_(iBoxHeading);
    if (fabs(angle_diff1) > M_PI_2) {
      angle_diff1 =
          (angle_diff1 > 0.0) ? (angle_diff1 - M_PI) : (angle_diff1 + M_PI);
      trans_angle1 =
          (trans_angle1 > 0.0) ? (trans_angle1 - M_PI) : (trans_angle1 + M_PI);
    }

    trans_angle2 = (X_(iBoxHeading) > 0.0) ? (X_(iBoxHeading) - M_PI_2)
                                           : (X_(iBoxHeading) + M_PI_2);
    angle_diff2 = new_Z(iBoxHeading) - trans_angle2;

    if (fabs(angle_diff2) > M_PI) {
      angle_diff2 = (angle_diff2 > 0.0) ? (angle_diff2 - 2 * M_PI)
                                        : (angle_diff2 + 2 * M_PI);
    }

    if (fabs(angle_diff2) > M_PI_2) {
      angle_diff2 =
          (angle_diff2 > 0.0) ? (angle_diff2 - M_PI) : (angle_diff2 + M_PI);
      trans_angle2 =
          (trans_angle2 > 0.0) ? (trans_angle2 - M_PI) : (trans_angle2 + M_PI);
    }

    if (fabs(angle_diff1) < fabs(angle_diff2)) {
      X_(iBoxHeading) = trans_angle1;
    } else {
      X_(iBoxHeading) = trans_angle2;
      double temp = X_(iBoxLen);
      X_(iBoxLen) = X_(iBoxWid);
      X_(iBoxWid) = temp;
    }

  } else {
    trans_angle1 = new_Z(iBoxHeading);
    if (fabs(angle_diff1) > M_PI_2) {
      if (angle_diff1 > 0.0) {
        angle_diff1 = angle_diff1 - M_PI;
        trans_angle1 = X_(iBoxHeading) + angle_diff1;
      } else {
        angle_diff1 = angle_diff1 + M_PI;
        trans_angle1 = X_(iBoxHeading) + angle_diff1;
      }
    }

    trans_angle2 = (new_Z(iBoxHeading) > 0.0) ? (new_Z(iBoxHeading) - M_PI_2)
                                              : (new_Z(iBoxHeading) + M_PI_2);

    angle_diff2 = trans_angle2 - X_(iBoxHeading);

    if (fabs(angle_diff2) > M_PI) {
      angle_diff2 = (angle_diff2 > 0.0) ? (angle_diff2 - 2 * M_PI)
                                        : (angle_diff2 + 2 * M_PI);
    }

    if (fabs(angle_diff2) > M_PI_2) {
      if (angle_diff2 > 0.0) {
        angle_diff2 = angle_diff2 - M_PI;
        trans_angle2 = X_(iBoxHeading) + angle_diff2;
      } else {
        angle_diff2 = angle_diff2 + M_PI;
        trans_angle2 = X_(iBoxHeading) + angle_diff2;
      }
    }

    if (fabs(angle_diff1) < fabs(angle_diff2)) {
      new_Z(iBoxHeading) = trans_angle1;
    } else {
      new_Z(iBoxHeading) = trans_angle2;
      double temp = new_Z(iBoxLen);
      new_Z(iBoxLen) = new_Z(iBoxWid);
      new_Z(iBoxWid) = temp;
    }
  }

  // std::cout << X_(iBoxHeading) / M_PI * 180.0 << ", "
  //           << new_Z(iBoxHeading) / M_PI * 180.0 << std::endl;

  X_cp(iBoxHeading) = X_(iBoxHeading);
}

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @param {VectorXd} measZ
 * @return {*}
 */
void simple_tracker::center_tracker(Eigen::VectorXd measZ) {
  if (0) {
    std::cout << "-------------------------------------"
              << "center_tracker [" << std::to_string(this->track_manage.id)
              << "]-------------------------------------" << std::endl;
  }

  Eigen::MatrixXd K = Eigen::MatrixXd(X_DIM, Z_DIM);
  Eigen::MatrixXd S = Eigen::MatrixXd(Z_DIM, Z_DIM);
  Eigen::MatrixXd z_pre = Eigen::VectorXd(Z_DIM);
  Eigen::VectorXd gain_ = Eigen::VectorXd(X_DIM);
  Eigen::MatrixXd I_ = Eigen::MatrixXd::Identity(X_DIM, X_DIM);
  Eigen::VectorXd residual_final = Eigen::VectorXd(Z_DIM);

  z_pre = H_ * X_;

  // 计算残差并判定是有可用于更新状态
  residual_final = measZ - z_pre;

  if (fabs(residual_final(iBoxHeading)) > M_PI) {
    residual_final(iBoxHeading) =
        (residual_final(iBoxHeading) > 0.0)
            ? (residual_final(iBoxHeading) - 2 * M_PI)
            : (residual_final(iBoxHeading) + 2 * M_PI);
  }

  // 自适应噪声更新
  adptive_noise(residual_final);

  // 重新计算Q P
  Update_Q();
  compute_F(diff_t);
  X_ = F_ * X_cp;
  P_ = F_ * P_ * F_.transpose() + Q_;

  // 执行卡尔曼滤波
  S = H_ * P_ * H_.transpose() + R_;
  K = P_ * H_.transpose() * S.inverse();

  gain_ = K * residual_final;
  gain_(iVreLat) = value_limit(-0.3, 0.3, gain_(iVreLat));
  gain_(iVreLong) = value_limit(-0.3, 0.3, gain_(iVreLong));
  gain_(iAccLat) = value_limit(-0.2, 0.2, gain_(iAccLat));
  gain_(iAccLong) = value_limit(-0.2, 0.2, gain_(iAccLong));
  X_ = X_ + gain_;
  P_ = (I_ - K * H_) * P_;

  if (0) {
    std::cout << "measZ: " << measZ.transpose() << std::endl
              << "z_pre: " << z_pre.transpose() << std::endl
              << "diff: " << residual_final.transpose() << std::endl
              << "k * diff " << (K * residual_final).transpose() << std::endl
              << "new X_ " << X_.transpose() << std::endl;
  }
}
/**
 * @names: l_shape_tracker
 * @description: Briefly describe the function of your function
 * @param {VectorXd} measZ
 * @return {*}
 */
void simple_tracker::l_shape_tracker(Eigen::VectorXd measZ) {
  if (0) {
    std::cout << "-------------------------------------"
              << "l_shape_tracker [" << std::to_string(this->track_manage.id)
              << "]-------------------------------------" << std::endl;
  }

  Eigen::MatrixXd K = Eigen::MatrixXd(X_DIM, Z_DIM);
  Eigen::MatrixXd S = Eigen::MatrixXd(Z_DIM, Z_DIM);
  Eigen::MatrixXd z_pre = Eigen::VectorXd(Z_DIM);
  Eigen::VectorXd gain_ = Eigen::VectorXd(X_DIM);
  Eigen::VectorXd LS_X = X_;
  Eigen::MatrixXd I_ = Eigen::MatrixXd::Identity(X_DIM, X_DIM);
  Eigen::VectorXd residual_final = Eigen::VectorXd(Z_DIM);

  // 计算最终量测状态
  compute_residual2(measZ);

  if (TRACK_DEBUG) {
    std::cout << " X_ : " << X_.transpose() << std::endl;
  }

  LS_X(iDisLong) = lshape_info.x_;
  LS_X(iDisLat) = lshape_info.y_;

  z_pre = H_ * LS_X;

  // 计算残差并判定是有可用于更新状态
  residual_final = measZ - z_pre;

  // 角度残差较大
  if (fabs(residual_final(iBoxHeading)) > (M_PI_2 / 9.0)) {
    if (fabs(residual_final(iBoxHeading)) >
        (M_PI_2 / 4.5))  // 大于20度，不进行状态更新，同时视作未关联到目标
    {
      matched_info.matched_det_list.clear();
      return;  // NOTE：此处只是不进行状态更新，但是目标的航迹管理状态并未发生改变
    } else {
      // 位置和角度残差减半
      residual_final(iDisLong) *= 0.5;
      residual_final(iDisLat) *= 0.5;
      residual_final(iBoxHeading) *= 0.5;
    }
  }

  // 自适应噪声更新
  adptive_noise(residual_final);

  // 重新计算Q P
  Update_Q();
  compute_F(diff_t);
  X_ = F_ * X_cp;
  P_ = F_ * P_ * F_.transpose() + Q_;

  // 执行卡尔曼滤波
  S = H_ * P_ * H_.transpose() + R_;
  K = P_ * H_.transpose() * S.inverse();

  gain_ = K * residual_final;
  gain_(iVreLat) = value_limit(-0.3, 0.3, gain_(iVreLat));
  gain_(iVreLong) = value_limit(-0.3, 0.3, gain_(iVreLong));
  gain_(iAccLat) = value_limit(-0.2, 0.2, gain_(iAccLat));
  gain_(iAccLong) = value_limit(-0.2, 0.2, gain_(iAccLong));
  LS_X = LS_X + gain_;
  P_ = (I_ - K * H_) * P_;

  // 从角点推理回中心点
  lshape_info.theta_1 -= gain_(iBoxHeading);
  lshape_info.theta_2 -= gain_(iBoxHeading);

  // 角度再次约束在[-PI, +PI]
  if (fabs(lshape_info.theta_1) > M_PI) {
    lshape_info.theta_1 = (lshape_info.theta_1 > 0.0)
                              ? (lshape_info.theta_1 - 2.0 * M_PI)
                              : (lshape_info.theta_1 + 2.0 * M_PI);
  }

  if (fabs(lshape_info.theta_2) > M_PI) {
    lshape_info.theta_2 = (lshape_info.theta_2 > 0.0)
                              ? (lshape_info.theta_2 - 2.0 * M_PI)
                              : (lshape_info.theta_2 + 2.0 * M_PI);
  }

  // if (TRACK_DEBUG) {
  //   std::cout << "lshape_info.LS_X: " << std::endl
  //             << LS_X.transpose() << std::endl
  //             << "theta_ :[ " << std::to_string(lshape_info.theta_1) << ", "
  //             << std::to_string(lshape_info.theta_2) << "]" << std::endl;
  // }

  LS_X(iDisLong) += 0.5 * (LS_X(iBoxLen) * sin(lshape_info.theta_1) +
                           LS_X(iBoxWid) * sin(lshape_info.theta_2));
  LS_X(iDisLat) += 0.5 * (LS_X(iBoxLen) * cos(lshape_info.theta_1) +
                          LS_X(iBoxWid) * cos(lshape_info.theta_2));

  // 更新最终中心点位置
  X_ = LS_X;

  if (0) {
    std::cout << "measZ: " << measZ.transpose() << std::endl
              << "z_pre: " << z_pre.transpose() << std::endl
              << "residual_final: " << residual_final.transpose() << std::endl
              << "k * diff "
              << (K * residual_final).transpose().segment(iVreLong, 6)
              << std::endl
              << "new X_ " << X_.transpose() << std::endl;
  }
}

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @return {*}
 */
void simple_tracker::tracker_postprocess(void) {
  // 后处理流程
  // 限制长必须大于宽
  if (X_(iBoxLen) < X_(iBoxWid)) {
    double temp = X_(iBoxLen);
    X_(iBoxLen) = X_(iBoxWid);
    X_(iBoxWid) = temp;
    X_(iBoxHeading) = (X_(iBoxHeading) > 0.0) ? (X_(iBoxHeading) - M_PI_2)
                                              : (X_(iBoxHeading) + M_PI_2);
  }

  // 朝向角约束在[-180, +180]之间
  while (abs(X_(iBoxHeading)) > 3.1415926) {
    X_(iBoxHeading) = (X_(iBoxHeading) > 3.1415926)
                          ? (X_(iBoxHeading) - 2.0 * M_PI)
                          : (X_(iBoxHeading) + 2.0 * M_PI);
    if (abs(X_(iBoxHeading)) > 3.1415926) {
      if (X_(iBoxHeading) > 3.1415926) {
        X_(iBoxHeading) = 3.1415926;
      } else {
        X_(iBoxHeading) = -3.1415926;
      }
    }
  }
}

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @param {VectorXd} &measZ
 * @return {*}
 */
void simple_tracker::compute_residual2(Eigen::VectorXd &measZ) {
  // step1: 确定该航迹的l-shape信息（角点、长宽、theta）
  Eigen::Matrix2d trans_mat;
  Eigen::MatrixXd conner_pos(8, 2);
  Eigen::VectorXd input = X_;

  double angle = X_(iBoxHeading);
  double half_len = X_(iBoxLen) * 0.5;
  double half_wid = X_(iBoxWid) * 0.5;

  trans_mat << cos(angle), -sin(angle), sin(angle), cos(angle);

  conner_pos << half_len, half_wid, half_len, 0, half_len, -half_wid, 0,
      -half_wid, -half_len, -half_wid, -half_len, 0, -half_len, half_wid, 0,
      half_wid;

  Eigen::MatrixXd transed_corner = trans_mat * conner_pos.transpose();

  double min_range = 1e3;
  int min_idx = 0;
  for (int idx = 0; idx < 4; idx++) {
    double corner_range =
        sqrt(pow(transed_corner(0, 2 * idx) + X_(iDisLong), 2.0) +
             pow(transed_corner(1, 2 * idx) + X_(iDisLat), 2.0));

    if (corner_range < min_range) {
      min_range = corner_range;
      min_idx = idx;
    }
  }

  int trace_closest_idx = min_idx * 2;

  // 计算与最近角点最近的量测点
  angle = measZ(iBoxHeading);
  half_len = measZ(iBoxLen) * 0.5;
  half_wid = measZ(iBoxWid) * 0.5;

  trans_mat << cos(angle), -sin(angle), sin(angle), cos(angle);

  conner_pos << half_len, half_wid, half_len, 0, half_len, -half_wid, 0,
      -half_wid, -half_len, -half_wid, -half_len, 0, -half_len, half_wid, 0,
      half_wid;

  Eigen::MatrixXd transed_corner2 = trans_mat * conner_pos.transpose();

  min_range = 1e3;
  min_idx = 0;
  for (uint8_t idx = 0; idx < 4; idx++) {
    double corner_range =
        sqrt(pow(transed_corner2(0, 2 * idx) + measZ(iDisLong), 2.0) +
             pow(transed_corner2(1, 2 * idx) + measZ(iDisLat), 2.0));
    if (corner_range < min_range) {
      min_range = corner_range;
      min_idx = idx;
    }
  }

  int meas_closest_idx = min_idx * 2;
  int final_closest_idx = trace_closest_idx;

  if (trace_closest_idx != meas_closest_idx) {
    double cor_compared_1[4][2];
    cor_compared_1[0][0] =
        transed_corner(iDisLong, trace_closest_idx) + input(iDisLong);
    cor_compared_1[0][1] =
        transed_corner(iDisLat, trace_closest_idx) + input(iDisLat);
    cor_compared_1[1][0] =
        transed_corner2(iDisLong, trace_closest_idx) + measZ(iDisLong);
    cor_compared_1[1][1] =
        transed_corner2(iDisLat, trace_closest_idx) + measZ(iDisLat);

    cor_compared_1[2][0] =
        transed_corner(iDisLong, meas_closest_idx) + input(iDisLong);
    cor_compared_1[2][1] =
        transed_corner(iDisLat, meas_closest_idx) + input(iDisLat);
    cor_compared_1[3][0] =
        transed_corner2(iDisLong, meas_closest_idx) + measZ(iDisLong);
    cor_compared_1[3][1] =
        transed_corner2(iDisLat, meas_closest_idx) + measZ(iDisLat);

    // 选择distance较小的那个
    if ((pow(cor_compared_1[0][0] - cor_compared_1[1][0], 2.0) +
         pow(cor_compared_1[0][1] - cor_compared_1[1][1], 2.0)) >
        (pow(cor_compared_1[2][0] - cor_compared_1[3][0], 2.0) +
         pow(cor_compared_1[2][1] - cor_compared_1[3][1], 2.0))) {
      final_closest_idx = meas_closest_idx;
    }
  }

  // 计算最终量测
  measZ(iDisLong) =
      transed_corner2(iDisLong, final_closest_idx) + measZ(iDisLong);
  measZ(iDisLat) = transed_corner2(iDisLat, final_closest_idx) + measZ(iDisLat);

  // 更新航迹最近点
  lshape_info.x_ = transed_corner(iDisLong, final_closest_idx) + X_(iDisLong);
  lshape_info.y_ = transed_corner(iDisLat, final_closest_idx) + X_(iDisLat);

  lshape_info.len_ = X_(iBoxLen);
  lshape_info.wid_ = X_(iBoxWid);

  int neighbor_idx[2] = {final_closest_idx - 2, final_closest_idx + 2};

  if (neighbor_idx[0] < 0) {
    neighbor_idx[0] = 6;
  }
  if (neighbor_idx[1] > 6) {
    neighbor_idx[1] = 0;
  }

  double cor_1[2], cor_2[2];

  cor_1[iDisLong] = transed_corner(iDisLong, neighbor_idx[0]) + X_(iDisLong);
  cor_1[iDisLat] = transed_corner(iDisLat, neighbor_idx[0]) + X_(iDisLat);

  cor_2[iDisLong] = transed_corner(iDisLong, neighbor_idx[1]) + X_(iDisLong);
  cor_2[iDisLat] = transed_corner(iDisLat, neighbor_idx[1]) + X_(iDisLat);

  double cor_diff_1[2] = {(cor_1[iDisLong] - lshape_info.x_),
                          (cor_1[iDisLat] - lshape_info.y_)};
  double cor_diff_2[2] = {(cor_2[iDisLong] - lshape_info.x_),
                          (cor_2[iDisLat] - lshape_info.y_)};

  if ((pow(cor_diff_1[iDisLong], 2.0) + pow(cor_diff_1[iDisLat], 2.0)) >
      (pow(cor_diff_2[iDisLong], 2.0) + pow(cor_diff_2[iDisLat], 2.0))) {
    lshape_info.theta_1 = atan2(cor_diff_1[iDisLong], cor_diff_1[iDisLat]);
    lshape_info.theta_2 = atan2(cor_diff_2[iDisLong], cor_diff_2[iDisLat]);
  } else {
    lshape_info.theta_1 = atan2(cor_diff_2[iDisLong], cor_diff_2[iDisLat]);
    lshape_info.theta_2 = atan2(cor_diff_1[iDisLong], cor_diff_1[iDisLat]);
  }
}

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @param {MatrixXd} residual_final
 * @return {*}
 */
void simple_tracker::adptive_noise(Eigen::MatrixXd residual_final) {
  // // 纵向距离残差累计

  double scale_ = 1.0;
  if ((track_manage.diff_x * residual_final(iDisLong)) < 0.0) {
    // adaptive_Q.neg_count_long += 1.0;
    adaptive_Q.count_long = 0;
    scale_ = -1.0;
  } else {
    // adaptive_Q.neg_count_long = 0;
  }

  if (fabs(residual_final(iDisLong)) > 0.05) {
    adaptive_Q.count_long++;
  } else {
    adaptive_Q.count_long--;
    scale_ = 0.8;
  }

  adaptive_Q.count_long = value_limit(0, 10, adaptive_Q.count_long);
  // adaptive_Q.neg_count_long = value_limit(0.0, 5.0,
  // adaptive_Q.neg_count_long);
  track_manage.diff_x =
      track_manage.diff_x * scale_ +
      residual_final(iDisLong) * (1.0 + 0.1 * adaptive_Q.count_long);
  track_manage.diff_x = value_limit(-2.0, 2.0, track_manage.diff_x);

  // 横向距离残差累计
  scale_ = 1.0;
  if ((track_manage.diff_y * residual_final(iDisLat)) < 0.0) {
    // adaptive_Q.neg_count_lat += 1.0;
    adaptive_Q.count_lat = 0;
    scale_ = -1.0;
  } else {
    // adaptive_Q.neg_count_lat = 0;
  }

  if (fabs(residual_final(iDisLat)) > 0.05) {
    adaptive_Q.count_lat++;
  } else {
    adaptive_Q.count_lat--;
    scale_ = 0.8;
  }

  adaptive_Q.count_lat = value_limit(0, 10, adaptive_Q.count_lat);
  // adaptive_Q.neg_count_lat = value_limit(0.0, 5.0, adaptive_Q.neg_count_lat);
  track_manage.diff_y =
      track_manage.diff_y * scale_ +
      residual_final(iDisLat) * (1.0 + 0.1 * adaptive_Q.count_lat);
  track_manage.diff_y = value_limit(-2.0, 2.0, track_manage.diff_y);

  // 航向角残差累计
  if (fabs(residual_final(iBoxHeading)) > (M_PI / 180.0 * 2.0)) {
    this->track_manage.diff_theta += (residual_final(iBoxHeading));
    track_manage.diff_theta =
        value_limit(-M_PI_2 / 3.0, M_PI_2 / 3.0, track_manage.diff_theta);
  } else {
    this->track_manage.diff_theta *= 0.9;
  }
}

/**
 * @names: measure_valid
 * @description: Briefly describe the function of your function
 * @return {*}
 */
bool simple_tracker::measure_valid(const Eigen::VectorXd &X_,
                                   const Eigen::VectorXd &new_Z,
                                   const Eigen::VectorXd &residual_final) {
  // 判定该量测是否能用于航迹更新
  double trace_box_s = X_(iBoxLen) * X_(iBoxWid);
  double det_box_s = new_Z(iBoxLen) * new_Z(iBoxWid);

  double scale_ = (trace_box_s < det_box_s) ? (trace_box_s / det_box_s)
                                            : (det_box_s / trace_box_s);

  bool abnormal_data_ = false;

  if (matched_info.has_high_meas) {
    return true;
  }

  if ((fabs(residual_final(iDisLat) > 1.0) ||
       fabs(residual_final(iDisLong)) > 2.0) &&
      (track_manage.track_status == TRK_Confirmed)) {
    abnormal_data_ = true;
  }

  if (abnormal_data_) {
    if (scale_ < 0.5) {
      matched_info.matched_case = match_but_not_used;

      if (scale_ < 0.2) {
        matched_info.matched_det_list.clear();  // 视作本次未关联到量测
        matched_info.matched_case = no_matched;
        if (TRACK_DEBUG) {
          std::cout << "Clear Detection !!!" << std::endl;
        }
      }
    }

    if (TRACK_DEBUG) {
      std::cout << "Abnormal_Assciation" << std::endl;
    }
    return false;
  }

  return true;
}

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @param {VectorXd} Z_
 * @return {*}
 */
Eigen::VectorXd simple_tracker::compute_residual(Eigen::VectorXd measZ,
                                                 Eigen::VectorXd preZ,
                                                 bool &change_heading) {
  Eigen::VectorXd residual_center, residual_corner, residual_final;
  uint8_t corner_1, corner_2;

  residual_center = measZ - preZ;

  Eigen::VectorXd Z_1 = compute_closet_point(measZ, corner_1);
  Eigen::VectorXd X_1 = compute_closet_point(preZ, corner_2);

  std::cout << "trace closest point idx : { " << std::to_string(corner_2)
            << "},  pos:{" << X_1.transpose() << "}" << std::endl;
  std::cout << "meas closest point idx : { " << std::to_string(corner_1)
            << "},  pos:{" << Z_1.transpose() << "}" << std::endl;

  double measZ_maxminX[2], preZ_maxminX[2];
  double measZ_maxminY[2], preZ_maxminY[2];
  measZ_maxminX[0] = measZ(0) + sqrt(pow(measZ(iBoxLen) / 2.0, 2.0) +
                                     pow(measZ(iBoxWid) / 2.0, 2.0));
  measZ_maxminX[1] = measZ(0) - sqrt(pow(measZ(iBoxLen) / 2.0, 2.0) +
                                     pow(measZ(iBoxWid) / 2.0, 2.0));
  preZ_maxminX[0] = preZ(0) + sqrt(pow(preZ(iBoxLen) / 2.0, 2.0) +
                                   pow(preZ(iBoxWid) / 2.0, 2.0));
  preZ_maxminX[1] = preZ(0) - sqrt(pow(preZ(iBoxLen) / 2.0, 2.0) +
                                   pow(preZ(iBoxWid) / 2.0, 2.0));

  measZ_maxminY[0] = measZ(1) + sqrt(pow(measZ(iBoxLen) / 2.0, 2.0) +
                                     pow(measZ(iBoxWid) / 2.0, 2.0));
  measZ_maxminY[1] = measZ(1) - sqrt(pow(measZ(iBoxLen) / 2.0, 2.0) +
                                     pow(measZ(iBoxWid) / 2.0, 2.0));
  preZ_maxminY[0] = preZ(1) + sqrt(pow(preZ(iBoxLen) / 2.0, 2.0) +
                                   pow(preZ(iBoxWid) / 2.0, 2.0));
  preZ_maxminY[1] = preZ(1) - sqrt(pow(preZ(iBoxLen) / 2.0, 2.0) +
                                   pow(preZ(iBoxWid) / 2.0, 2.0));

  if (corner_1 == corner_2) {
    std::cout << " step 1: using corner point !!!" << std::endl;
    residual_final = Z_1 - X_1;
  } else {
    if (((measZ_maxminX[0] < -1.0) || (measZ_maxminX[1] > 1.0)) &&
        ((preZ_maxminX[0] < -1.0) || (preZ_maxminX[1] > 1.0)) &&
        ((measZ_maxminY[0] < -1.0) || (measZ_maxminY[1] > 1.0)) &&
        ((preZ_maxminY[0] < -1.0) || (preZ_maxminY[1] > 1.0))) {
      residual_corner = Z_1 - X_1;
    } else {
      residual_corner = measZ - preZ;
    }

    double diff_r1 =
        sqrt(pow(residual_corner(0), 2.0) + pow(residual_corner(1), 2.0));
    double diff_r2 =
        sqrt(pow(residual_center(0), 2.0) + pow(residual_center(1), 2.0));

    if (diff_r2 < diff_r1) {
      residual_final = residual_center;
      std::cout << " step 2: using center point !!!" << std::endl;
    } else {
      residual_final = residual_corner;
      std::cout << " step 3: using corner point !!!" << std::endl;
    }
  }

  if (TRACK_DEBUG) {
    std::cout << "oring residual_ " << residual_corner.transpose() << std::endl
              << "final residual_ " << residual_final.transpose() << std::endl;
  }

  return residual_final;
}

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @param {VectorXd} input
 * @return {*}
 */
Eigen::VectorXd simple_tracker::compute_closet_point(Eigen::VectorXd input,
                                                     uint8_t &closest_idx) {
  Eigen::Matrix2d trans_mat;
  Eigen::MatrixXd conner_pos(8, 2);

  Eigen::VectorXd closet_conrner = input;

  double angle = input(3);
  double half_len = input(4) * 0.5;
  double half_wid = input(5) * 0.5;

  trans_mat << cos(angle), -sin(angle), sin(angle), cos(angle);

  conner_pos << half_len, half_wid, half_len, 0, half_len, -half_wid, 0,
      -half_wid, -half_len, -half_wid, -half_len, 0, -half_len, half_wid, 0,
      half_wid;

  Eigen::MatrixXd transed_corner = trans_mat * conner_pos.transpose();

  double min_range = 1e3;
  uint8_t min_idx = 0;
  for (uint8_t idx = 0; idx < 4; idx++) {
    double corner_range = sqrt(pow(transed_corner(0, 2 * idx) + input(0), 2.0) +
                               pow(transed_corner(1, 2 * idx) + input(1), 2.0));

    if (corner_range < min_range) {
      min_range = corner_range;
      min_idx = idx;
    }
  }

  closest_idx = min_idx * 2;
  closet_conrner(0) = transed_corner(0, closest_idx) + input(0);
  closet_conrner(1) = transed_corner(1, closest_idx) + input(1);

  // 更新航迹最近点
  lshape_info.x_ = transed_corner(0, closest_idx) + input(0);
  lshape_info.y_ = transed_corner(1, closest_idx) + input(1);

  lshape_info.len_ = input(iBoxLen);
  lshape_info.wid_ = input(iBoxWid);

  int neighbor_idx[2] = {min_idx - 1, min_idx + 1};

  if (neighbor_idx[0] < 0) {
    neighbor_idx[0] = 3;
  }

  if (neighbor_idx[1] > 3) {
    neighbor_idx[1] = 0;
  }

  double neighbor_P[2][2] = {{transed_corner(0, neighbor_idx[0]) + input(0),
                              transed_corner(1, neighbor_idx[0]) + input(1)},
                             {transed_corner(0, neighbor_idx[1]) + input(0),
                              transed_corner(1, neighbor_idx[1]) + input(1)}};

  double tmp_theta_1 = atan2(neighbor_P[0][1], neighbor_P[0][0]);
  double tmp_theta_2 = atan2(neighbor_P[1][1], neighbor_P[1][0]);

  lshape_info.theta_1 = input(iBoxHeading);
  lshape_info.theta_2 = (fabs(tmp_theta_1 - input(iBoxHeading)) < (M_PI / 18.0))
                            ? (tmp_theta_2)
                            : (tmp_theta_1);

  if (TRACK_DEBUG) {
    std::cout << "closet point " << std::to_string(closest_idx) << " "
              << closet_conrner(0) << " " << closet_conrner(1) << std::endl;
  }

  return closet_conrner;
}

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @return {*}
 */
void simple_tracker::compute_extended_info_vari() {
#define FIFO_SIZE (10)
  double mea_val, vari_val;

  if (this->len_fifo.size() >= FIFO_SIZE) {
    this->len_fifo.pop_front();
  }

  this->len_fifo.push_back(this->track_manage.shape4out.len);
  ComputeMeanAndVari(this->len_fifo, &mea_val, &vari_val);

  shape_vari[0] = vari_val;

  if (this->wid_fifo.size() >= FIFO_SIZE) {
    this->wid_fifo.pop_front();
  }

  this->wid_fifo.push_back(this->track_manage.shape4out.wid);
  ComputeMeanAndVari(this->wid_fifo, &mea_val, &vari_val);

  shape_vari[1] = vari_val;

  if (this->height_fifo.size() >= FIFO_SIZE) {
    this->height_fifo.pop_front();
  }

  this->height_fifo.push_back(this->track_manage.shape4out.height);
  ComputeMeanAndVari(this->height_fifo, &mea_val, &vari_val);

  shape_vari[2] = vari_val;

  if (this->theta_fifo.size() >= FIFO_SIZE) {
    this->theta_fifo.pop_front();
  }

  this->theta_fifo.push_back(this->track_manage.shape4out.theta);
  for (uint8_t idx = 1; idx < this->theta_fifo.size(); idx++) {
    /* 约束diff-theta */
    if (fabs(this->theta_fifo.at(idx) - this->theta_fifo.at(idx - 1)) > M_PI) {
      if (this->theta_fifo.at(idx) > 0) {
        this->theta_fifo.at(idx) -= 2.0 * M_PI;
      } else {
        this->theta_fifo.at(idx) += 2.0 * M_PI;
      }
    }
  }

  ComputeMeanAndVari(this->theta_fifo, &mea_val, &vari_val);

  shape_vari[3] = vari_val;
}

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @return {*}
 */
void simple_tracker::ComputeMeanAndVari(const std::deque<double> dataArray,
                                        double *mean_val, double *vari_val) {
  double tempMean = 0.0;
  double tempVari = 0.0;

  for (const auto sub_data : dataArray) {
    tempMean += sub_data;
  }

  tempMean = tempMean / dataArray.size();

  for (const auto sub_data : dataArray) {
    tempVari += (pow((sub_data - tempMean), 2.0) / dataArray.size());
  }

  *mean_val = tempMean;
  *vari_val = tempVari;
}

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @return {*}
 */
void simple_tracker::trace_update_without_det(void) {}

double simple_tracker::value_limit(double lower_val, double upper_val,
                                   double value) {
  if (value < lower_val) {
    return lower_val;
  } else {
    if (value > upper_val) {
      return upper_val;
    } else {
      return value;
    }
  }
}