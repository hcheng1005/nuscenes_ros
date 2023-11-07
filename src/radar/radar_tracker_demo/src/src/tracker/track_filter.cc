/*
 * @description:
 * @version:
 * @Author: ChengHao
 * @Date: 2022-10-08 14:09:49
 * @LastEditors: ChengHao hao.cheng@wuzheng.com
 * @LastEditTime: 2023-11-07 14:27:10
 */

#include "track_filter.h"

#include "commonfunctions.h"
#include "common/DBSCAN.h"
#include "match.h"

/**
 * @name: track_Predict
 * @description: 航迹状态预测
 * @param {trackTable_strcut} *trackInfo
 * @param {Tracker} *Tracker_
 * @param {float} delta_t
 * @return {*}
 */
void track_Predict(trackTable_strcut *trackInfo,
                   vehicleInfo_struct *vehicleInfo, Tracker *Tracker_,
                   float delta_t) {
  Tracker_->vehicleInfo_ = vehicleInfo;

  if (fabs(Tracker_->vehicleInfo_->yaw_rate) > (5.0 * DEG2RAD)) {
    Tracker_->yaw_rate_delay = 10;
  } else {
    if (Tracker_->yaw_rate_delay > 2) {
      Tracker_->yaw_rate_delay -= 2;
    } else {
      Tracker_->yaw_rate_delay = 0;
    }
  }

  for (uint16_t track_i = 0; track_i < MAXTRACKS; track_i++) {
    if ((trackInfo[track_i].trackState == TRACK_STATE_ACTIVE) ||
        (trackInfo[track_i].trackState == TRACK_STATE_DETECTION)) {
      // 重新计算滤波器 QR
      track_compute_Q(&trackInfo[track_i], delta_t, vehicleInfo);

      track_compute_R(&trackInfo[track_i], vehicleInfo);

      Tracker_->Prediction(delta_t, &trackInfo[track_i], vehicleInfo);

      Track_Update_Corners(&trackInfo[track_i]);

      trackInfo[track_i].MeasInfo.associateNum = 0;
      trackInfo[track_i].MeasInfo.HighQuaPointNum = 0;
      trackInfo[track_i].MeasInfo.detInfo.clear();
    }
  }
}

/**
 * @name: track_update
 * @description: 航迹更新
 * @param {RadarOutput_Struct} *indata
 * @param {trackTable_strcut} *trackInfo
 * @param {Tracker} *Tracker_
 * @return {*}
 */
void track_update(std::vector<RadarMeasure_struct> &global_point,
                  trackTable_strcut *trackInfo, Tracker *Tracker_) {
  track_kinematic_update(global_point, trackInfo, Tracker_);
}

/**
 * @name: track_kinematic_update
 * @description: 航迹运动学信息更新
 * @param {RadarOutput_Struct} *indata
 * @param {trackTable_strcut} *trackInfo
 * @param {Tracker} *Tracker_
 * @return {*}
 */
void track_kinematic_update(std::vector<RadarMeasure_struct> &global_point,
                            trackTable_strcut *trace_list, Tracker *Tracker_) {
  std::string string__ = "..";

  for (uint16_t track_i = 0; track_i < MAXTRACKS; track_i++) {
    if (trace_list[track_i].trackState > TRACK_STATE_FREE) {
      trackTable_strcut *trace = &trace_list[track_i];

      // 存在概率更新
      UpdateSurvivalProb(trace, global_point, Tracker_->vehicleInfo_);

      // 计算绝对速度
      track_compute_abs_velocity(trace, Tracker_->vehicleInfo_);

      // 目标类型更新
      Track_Update_TrkType(global_point, trace);

      // 航向角更新
      Track_Update_headingAng(trace, global_point, Tracker_->vehicleInfo_);

      // 目标角坐标更新
      Track_Update_Corners(trace);

      // 目标运动属性更新
      Track_Update_DynPro(trace);

      compute_extended_info_vari(global_point, trace);

      if (trace->MeasInfo.associateNum > 0) {
        // 不执行更新算法
        if ((trace->ExtendInfo.DynProp == oncoming) &&
            (trace->KalmanInfo.MeasPre(1) < 0.0)) {
          // TODO:是否应该有特殊处理
        } else {
          Tracker_->UpdateState(trace, global_point);
        }
      }
    }
  }
}

void compute_extended_info_vari(std::vector<RadarMeasure_struct> &global_point,
                                trackTable_strcut *trace) {
  double mea_val, vari_val;

  if (trace->shape_status.len_fifo.size() >= 15) {
    trace->shape_status.len_fifo.pop_front();
  }

  trace->shape_status.len_fifo.push_back(trace->ExtendInfo.Length);
  ComputeMeanAndVari(trace->shape_status.len_fifo, &mea_val, &vari_val);

  trace->shape_status.P_[0] = vari_val;

  if (trace->shape_status.wid_fifo.size() >= 15) {
    trace->shape_status.wid_fifo.pop_front();
  }

  trace->shape_status.wid_fifo.push_back(trace->ExtendInfo.Width);
  ComputeMeanAndVari(trace->shape_status.wid_fifo, &mea_val, &vari_val);

  trace->shape_status.P_[1] = vari_val;

  if (trace->shape_status.theta_fifo.size() >= 15) {
    trace->shape_status.theta_fifo.pop_front();
  }

  trace->shape_status.theta_fifo.push_back(trace->ExtendInfo.Orin_For_Display);

  for (uint8_t idx = 1; idx < trace->shape_status.theta_fifo.size(); idx++) {
    /* 约束diff-theta */
    if (fabs(trace->shape_status.theta_fifo.at(idx) -
             trace->shape_status.theta_fifo.at(idx - 1)) > (0.5 * RADAR_PI)) {
      if (trace->shape_status.theta_fifo.at(idx) > 0) {
        trace->shape_status.theta_fifo.at(idx) -= RADAR_PI;
      } else {
        trace->shape_status.theta_fifo.at(idx) += RADAR_PI;
      }
    }
  }

  ComputeMeanAndVari(trace->shape_status.theta_fifo, &mea_val, &vari_val);

  trace->shape_status.P_[3] = vari_val;

#if 0
      if (trace->chosed == true)
      {
          std::cout << trace->shape_status.P_[0] << " " \
                  << trace->shape_status.P_[1]<< " " \
                  << trace->shape_status.P_[3] << std::endl;
      }
#endif
}

/**
 * @name:track_compute_Q
 * @description: 根据航迹状态设定不同的过程噪声Q
 * @param {trackTable_strcut} *trace
 * @param {float} delta_t
 * @return {*}
 */
void track_compute_Q(trackTable_strcut *trace, float delta_t,
                     vehicleInfo_struct *vehicleInfo) {
  MatrixXd G = MatrixXd::Zero(SSIZE, 2);
  MatrixXd q = MatrixXd::Zero(2, 2);

  double dt = delta_t;
  double dt3 = powf(dt, 3.0);
  double dt2 = powf(dt, 2.0);

  double std_accLat_, std_accLong_;

  if (trace->trackState == TRACK_STATE_DETECTION) {
    std_accLat_ = 0.4;
    std_accLong_ = 0.8;
  } else {
    std_accLat_ = 0.2 + fabs(trace->MeasInfo.x_diff_dir) * 0.2;
    std_accLong_ = 0.4 + fabs(trace->MeasInfo.y_diff_dir) * 0.25 +
                   fabs(trace->MeasInfo.v_diff_dir) * 0.15;
  }

  if (vehicleInfo->yawRateValid) {
    std_accLat_ += fabs(vehicleInfo->yaw_rate) * 1.0;
    std_accLong_ += fabs(vehicleInfo->yaw_rate) * 1.0;
  }

  G << 1 / 6 * dt3, 0, 0, 1 / 6 * dt3, 0.5 * dt2, 0, 0, 0.5 * dt2, dt, 0, 0, dt;

  q << powf(std_accLat_, 2.0), 0.0, 0.0, powf(std_accLong_, 2.0);

  trace->KalmanInfo.Q_ = G * q * G.transpose();

  if (trace->trackState == TRACK_STATE_ACTIVE) {
    if (trace->ExtendInfo.Object_Class >= VEHICLE) {
      trace->KalmanInfo.Q_(0, 0) +=
          pow(fabs(trace->MeasInfo.x_diff_dir) * 0.01, 2.0);

      // 对于20m内的航迹，降低横向噪声
      if ((trace->KalmanInfo.StateEst(iDistLong) < 20.0) &&
          (fabs(trace->KalmanInfo.StateEst(iDistLat)) < 10.0)) {
        // TBD
      } else {
        trace->KalmanInfo.Q_(2, 2) +=
            pow(fabs(trace->MeasInfo.x_diff_dir) * 0.01, 2.0);
      }

      trace->KalmanInfo.Q_(1, 1) +=
          pow(fabs(trace->MeasInfo.y_diff_dir) * 0.01, 2.0);
      trace->KalmanInfo.Q_(3, 3) +=
          (pow(fabs(trace->MeasInfo.y_diff_dir) * 0.01, 2.0) +
           pow(fabs(trace->MeasInfo.v_diff_dir) * 0.02, 2.0));
    } else {
      trace->KalmanInfo.Q_(0, 0) +=
          pow(fabs(trace->MeasInfo.x_diff_dir) * 0.01, 2.0);
      trace->KalmanInfo.Q_(2, 2) +=
          pow(fabs(trace->MeasInfo.x_diff_dir) * 0.02, 2.0);

      trace->KalmanInfo.Q_(1, 1) +=
          pow(fabs(trace->MeasInfo.y_diff_dir) * 0.01, 2.0);
      trace->KalmanInfo.Q_(3, 3) +=
          pow(fabs(trace->MeasInfo.y_diff_dir) * 0.02, 2.0);
    }
  } else {
    // 未确认航迹
    trace->KalmanInfo.Q_(0, 0) +=
        pow(fabs(trace->MeasInfo.x_diff_dir) * 0.02, 2.0);
    trace->KalmanInfo.Q_(2, 2) +=
        pow(fabs(trace->MeasInfo.x_diff_dir) * 0.04, 2.0);

    trace->KalmanInfo.Q_(1, 1) +=
        pow(fabs(trace->MeasInfo.y_diff_dir) * 0.03, 2.0);
    trace->KalmanInfo.Q_(3, 3) +=
        pow(fabs(trace->MeasInfo.y_diff_dir) * 0.06, 2.0);
  }
}

/**
 * @name:
 * @description: 根据航迹状态设定不同的量测噪声R
 * @param {RadarOutput_Struct} *indata
 * @param {trackTable_strcut} *trace
 * @return {*}
 */
void track_compute_R(trackTable_strcut *trace,
                     vehicleInfo_struct *vehicleInfo) {
  Eigen::Matrix3d R;

  float std_lat, std_long, std_v;
  double headding_Ang = trace->ExtendInfo.box_theta;

  if (fabs(headding_Ang) > (0.5 * RADAR_PI)) {
    if (headding_Ang > (0.5 * RADAR_PI)) {
      headding_Ang = headding_Ang - RADAR_PI;
    } else {
      headding_Ang = headding_Ang + RADAR_PI;
    }
  }

/* 基础方差参数 */
#ifdef ARS548_RADAR
  std_lat = 0.05 * 2.0;
  std_long = 0.05 * 1.0;
  std_v =
      0.2 * 1.0 + fabs(vehicleInfo->yaw_rate) * 10.0 + fabs(headding_Ang) * 0.5;
#else
  std_lat = 0.2 * 3.0;
  std_long = 0.2 * 2.0;
  std_v = 0.2 + fabs(vehicleInfo->yaw_rate) * 10.0 + fabs(headding_Ang) * 0.5;
#endif

  double range_ = sqrt(pow(trace->KalmanInfo.StateEst(0), 2.0) +
                       pow(trace->KalmanInfo.StateEst(1), 2.0));

  bool small_target = false;
  if ((trace->ExtendInfo.Width * trace->ExtendInfo.Length) < 2.0) {
    small_target = true;
  }

  if (trace->ExtendInfo.DynProp == crossing_moving) {
    std_v = 2.0;
    std_lat *= 0.5;  // 横穿目标，减少横向距离方差，以提高关联概率
  } else {
    std_v -= ((range_ / 10.0) * 0.1);  // 距离越远，速度方差越小
    std_v = Valuelimit(0.1, 2.0, std_v);
  }

  if ((trace->ExtendInfo.DynProp == crossing_moving) ||
      (trace->ExtendInfo.DynProp == crossing_stationary)) {
  } else {
    std_lat -= fabs(trace->MeasInfo.x_diff_dir) * 0.01;
    std_long -= fabs(trace->MeasInfo.y_diff_dir) * 0.01;
  }

  double conv_v = 0.0;
  if (small_target) {
    std_lat -= fabs(trace->MeasInfo.x_diff_dir) * 0.02;
    std_long -= fabs(trace->MeasInfo.y_diff_dir) * 0.02;

/* 基础方差参数 */
#ifdef ARS548_RADAR
    std_lat = Valuelimit(0.1, 2.0, std_lat);
    std_long = Valuelimit(0.1, 2.0, std_long);
    std_v = 0.1;
#else
    std_lat = Valuelimit(0.2, 2.0, std_lat);
    std_long = Valuelimit(0.2, 2.0, std_long);
    std_v = 0.25;
#endif
  } else {
    /* 补充径向速度方差 */
    conv_v = trace_v_covariance(trace);
    std_lat = Valuelimit(0.2, 2.0, std_lat);
    std_long = Valuelimit(0.2, 2.0, std_long);
  }

  R << pow(std_lat, 2.0F), 0, 0, 0, pow(std_long, 2.0F), 0, 0, 0,
      pow(std_v, 2.0F) + conv_v;

  trace->KalmanInfo.R_ = R;
}

/**
 * @name: trace_v_covariance
 * @description:
 * @param {trackTable_strcut} *trace
 * @return {*}
 */
double trace_v_covariance(trackTable_strcut *trace) {
  /* 根据目标各个corner的径向速度来计算径向速度方差 */
  std::vector<double> prob_v_list;
  double mean_v, conv_v;
  mean_v = trace->KalmanInfo.MeasPre(2);
  conv_v = 0.0;
  for (uint8_t idx = 0; idx < 4; idx++) {
    /* 该点纵向距离是否大于0 */
    if (trace->ExtendInfo.CornerPos[2 * idx][1] > 0) {
      double temp_V = (trace->KalmanInfo.StateEst(iVrelLat) *
                           trace->ExtendInfo.CornerPos[2 * idx][0] +
                       trace->KalmanInfo.StateEst(iVrelLong) *
                           trace->ExtendInfo.CornerPos[2 * idx][1]) /
                      sqrt(pow(trace->ExtendInfo.CornerPos[2 * idx][0], 2.0) +
                           pow(trace->ExtendInfo.CornerPos[2 * idx][1], 2.0));

      prob_v_list.push_back(temp_V);

      mean_v += temp_V;
    }
  }

  if (prob_v_list.size() > 0) {
    mean_v = mean_v / (prob_v_list.size() + 1.0);
    conv_v = pow((trace->KalmanInfo.MeasPre(2) - mean_v), 2.0);

    for (auto tempv : prob_v_list) {
      conv_v += pow((tempv - mean_v), 2.0);
    }

    conv_v = conv_v / prob_v_list.size();
  }

  return conv_v;
}

/**
 * @name: track_extendInfoUpdate
 * @description: 航迹扩展信息更新：目标长宽属性、类别、航向角、运动状态
 * @param {RadarOutput_Struct} *indata
 * @param {trackTable_strcut} *trace
 * @return {*}
 */
void track_extend_info_update(std::vector<RadarMeasure_struct> &global_point,
                              trackTable_strcut *trace_list,
                              vehicleInfo_struct *vehicleInfo) {
  trackTable_strcut *trace = nullptr;
  for (uint16_t idx = 0; idx < MAXTRACKS; idx++) {
    if (trace_list[idx].trackState == TRACK_STATE_ACTIVE ||
        trace_list[idx].trackState == TRACK_STATE_DETECTION) {
      trace = &trace_list[idx];

      // 航向角更新
      Track_Update_headingAng(trace, global_point, vehicleInfo);

      // 目标角坐标更新
      Track_Update_Corners(trace);

      // 目标运动属性更新
      Track_Update_DynPro(trace);
    }
  }
}

/**
 * @name:
 * @description:
 * @param {trackTable_strcut} *trace
 * @return {*}
 */
void track_compute_abs_velocity(trackTable_strcut *trace,
                                vehicleInfo_struct *vehicleInfo) {
#if 1
  // 计算目标真实速度
  //  从定位信息中提取线速度和角速度
  //    const apollo::common::Point3D car_linear_speed =
  //        loct_ptr->pose().linear_velocity_vrf();
  //    const apollo::common::Point3D car_angular_speed =
  //        loct_ptr->pose().angular_velocity_vrf();

  Eigen::Matrix3d sensor2car_trans;
  //    sensor2car_trans << cos(RADAR_MOUNT_YAW), -sin(RADAR_MOUNT_YAW),
  //    RADAR_MOUNT_X,
  //                        sin(RADAR_MOUNT_YAW), cos(RADAR_MOUNT_YAW),
  //                        RADAR_MOUNT_Y, 0.0, 0.0, 1.0;

  sensor2car_trans << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;

  double yaw_rate = 0.0;
  if (vehicleInfo->yawRateValid) {
    yaw_rate = vehicleInfo->yaw_rate;
  }

  // 自身旋转
  Eigen::Matrix3d rotation_car = Eigen::MatrixXd::Zero(3, 3);
  //  rotation_car.setZero();
  rotation_car(0, 1) = -1.0 * yaw_rate;
  rotation_car(1, 0) = yaw_rate;

  // radar to imu
  Eigen::Matrix3d rotation_sensor =
      sensor2car_trans.inverse() * rotation_car * sensor2car_trans;

  // 目标“本地”位置信息：x,y,z
  Eigen::Vector3d local_loc(trace->KalmanInfo.StateEst(iDistLong),
                            trace->KalmanInfo.StateEst(iDistLat), 0.0);

  // 目标“本地”速度信息：vx, vy, vz
  Eigen::Vector3d local_vel(trace->KalmanInfo.StateEst(iVrelLong),
                            trace->KalmanInfo.StateEst(iVrelLat), 0.0);

  // 计算由于旋转带来的速度
  Eigen::Vector3d angular_trans_speed = rotation_sensor * local_loc;

  Eigen::Vector3d vel2car =
      sensor2car_trans * (local_vel + angular_trans_speed);

  // 目标绝对速度
  const Eigen::Vector3d car_linear_speed_ =
      Eigen::Vector3d(vehicleInfo->vx, 0.0, 0.0);
  Eigen::Vector3d abs_vel = vel2car + car_linear_speed_;

  trace->KalmanInfo.StateEst_abs(iVrelLong) = abs_vel(0);
  trace->KalmanInfo.StateEst_abs(iVrelLat) = abs_vel(1);
#endif

#if 0
  Eigen::Vector3d angular_speed;
  angular_speed << 0.0, 0.0, yaw_rate;

  std::cout << "\n" << std::endl;
  std::cout << "Trace ID: "<< trace->trackID <<std::endl;
  std::cout << "local_loc: " << local_loc.transpose() << std::endl;
  std::cout << "local_vel: " << local_vel.transpose() << std::endl;
  std::cout << "car_linear_speed: " << car_linear_speed_.transpose()
            << std::endl;
  std::cout << "angular_speed: " << angular_speed.transpose() << std::endl;
  std::cout << "local_vel  " << local_vel.transpose() << std::endl;
  std::cout << "angular_trans_speed  " << angular_trans_speed.transpose()
            << std::endl;
  std::cout << "local_vel + angular_trans_speed  " << vel2car.transpose()
            << std::endl;

  std::cout << "abs_vel  " << abs_vel.transpose() << std::endl;
#endif
}

/**
 * @name:
 * @description: 航迹航向角更新
 * @param {trackTable_strcut} *trace
 * @return {*}
 */
void Track_Update_headingAng(trackTable_strcut *trace,
                             std::vector<RadarMeasure_struct> &global_point,
                             vehicleInfo_struct *vehicleInfo) {
  double temp_VLat, temp_VLong;
  double theta_by_v, Error_A;

  double speed_scale = 0.0;

  if (trace->trackState == TRACK_STATE_ACTIVE) {
    if (((trace->ExtendInfo.corner_type & (1 << TWO_POINT_INVISIBLE)) ==
         (1 << TWO_POINT_INVISIBLE)) ||
        ((trace->ExtendInfo.corner_type & (1 << THREE_POINT_INVISIBLE)) ==
         (1 << THREE_POINT_INVISIBLE)) ||
        ((trace->ExtendInfo.corner_type & (1 << FOUR_POINT_INVISIBLE)) ==
         (1 << FOUR_POINT_INVISIBLE))) {
      // 20230506: 补充逻辑：当目标朝向角较大时（比如转弯），继续执行朝向角更新
      // 朝向角较小时，则不更新
      if (fabs(trace->ExtendInfo.Orin_For_Display) < (10.0 * DEG2RAD)) {
        return;
      }
    }
  }

  if (fabs(trace->KalmanInfo.StateEst_abs(iVrelLat)) > 0.6) {
    temp_VLat = trace->KalmanInfo.StateEst_abs(iVrelLat);
  } else {
    temp_VLat = 0.0;
  }

  if (fabs(trace->KalmanInfo.StateEst_abs(iVrelLong)) > 0.4) {
    temp_VLong = trace->KalmanInfo.StateEst_abs(iVrelLong);
  } else {
    temp_VLong = 0.0;
  }

  if (temp_VLong == 0.0) {
    speed_scale = fabs(temp_VLat);
  } else {
    double v_long_temp = (fabs(temp_VLong) < 1.0) ? 1.0 : fabs(temp_VLong);

    if (temp_VLat != 0.0) {
      speed_scale = (fabs(temp_VLat) / v_long_temp);
    } else {
      speed_scale = fabs(temp_VLong);
    }
  }

  // 若目标处于低速状态，根据前时刻航向角状态决定扭转方向
  if ((temp_VLat == 0.0) && (temp_VLong == 0.0)) {
    return;
  } else {
    theta_by_v = atan2(temp_VLat, temp_VLong);
  }

  double factor_ = 0.2;

  if (trace->trackState == TRACK_STATE_DETECTION) {
    factor_ = 4.0;
  }

  if (trace->TCount == 1U) {
    trace->ExtendInfo.box_theta = 0.0;
  } else {
    factor_ += speed_scale;

    // 计算角度差
    double Error_A2[2];
    double heading2 = (trace->ExtendInfo.box_theta >= 0.0)
                          ? (trace->ExtendInfo.box_theta - RADAR_PI)
                          : (trace->ExtendInfo.box_theta + RADAR_PI);

    Error_A2[0] = theta_by_v - trace->ExtendInfo.box_theta;
    Error_A2[1] = theta_by_v - heading2;

    if (fabs(Error_A2[0]) > RADAR_PI) {
      Error_A2[0] = (Error_A2[0] > 0.0) ? (Error_A2[0] - 2.0 * RADAR_PI)
                                        : (Error_A2[0] + 2.0 * RADAR_PI);
    }

    if (fabs(Error_A2[1]) > RADAR_PI) {
      Error_A2[1] = (Error_A2[1] > 0.0) ? (Error_A2[1] - 2.0 * RADAR_PI)
                                        : (Error_A2[1] + 2.0 * RADAR_PI);
    }

    double minAng = 0.0;
    bool changeAngle = false;
    // 选择角度差较小的角度
    if (fabs(Error_A2[0]) <= fabs(Error_A2[1])) {
      minAng = Error_A2[0];
    } else {
      changeAngle = true;
      minAng = Error_A2[1];
    }

    Error_A = Valuelimit(-factor_ * DEG2RAD, factor_ * DEG2RAD, minAng);

    if ((changeAngle) && (trace->ExtendInfo.DynProp != moving) &&
        (trace->ExtendInfo.DynProp != stopped))  // 转换方向
    {
      trace->ExtendInfo.box_theta = heading2;
    }

    // std::cout << std::to_string(trace->trackID) << " "
    //           << std::to_string(trace->ExtendInfo.box_theta) << " "
    //           << changeAngle << " "
    //           << std::to_string(Error_A)
    //           << std::endl;

    trace->ExtendInfo.box_theta += Error_A;

    // 航向角约束[-PI, +PI]
    if (fabs(trace->ExtendInfo.box_theta) > RADAR_PI) {
      if (trace->ExtendInfo.box_theta > RADAR_PI) {
        trace->ExtendInfo.box_theta =
            trace->ExtendInfo.box_theta - 2.0 * RADAR_PI;
      } else {
        trace->ExtendInfo.box_theta =
            trace->ExtendInfo.box_theta + 2.0 * RADAR_PI;
      }
    }
  }

  /* theta for output [-90, 90] */
  // double temp_theta;
  // if (fabs(trace->ExtendInfo.box_theta) < (0.5 * RADAR_PI)) {
  //   temp_theta = trace->ExtendInfo.box_theta;
  // } else {
  //   if (trace->ExtendInfo.box_theta > 0.0) {
  //     temp_theta = trace->ExtendInfo.box_theta - RADAR_PI;
  //   } else {
  //     temp_theta = trace->ExtendInfo.box_theta + RADAR_PI;
  //   }
  // }

  // trace->ExtendInfo.Orin_For_Display = temp_theta;
  trace->ExtendInfo.Orin_For_Display = trace->ExtendInfo.box_theta;
}

/**
 * @name:
 * @description: 计算航迹的八个角点坐标
 * @param {trackTable_strcut} *trace
 * @param {bool} extendFlag
 * @return {*}
 */
void Track_Update_Corners(trackTable_strcut *trace) {
  double half_Width = trace->ExtendInfo.Width * 0.5;
  double half_length = trace->ExtendInfo.Length * 0.5;

  ComputeCornerPos(trace->KalmanInfo.StateEst(iDistLat),
                   trace->KalmanInfo.StateEst(iDistLong),
                   trace->ExtendInfo.box_theta, half_Width, half_length,
                   &trace->ExtendInfo.CornerPos[0][0]);

  trace->ExtendInfo.corner_type = (1 << RESERVED_STATUS);

  double temp_theta = trace->ExtendInfo.box_theta;

  if (fabs(trace->ExtendInfo.box_theta) > (0.5 * RADAR_PI)) {
    temp_theta =
        (temp_theta > 0.0) ? (temp_theta - RADAR_PI) : (temp_theta + RADAR_PI);
  }

  double scale_range = (pow(half_Width, 2.0) + pow(half_length, 2.0)) *
                       pow(sin(temp_theta), 2.0);

  if ((pow(trace->KalmanInfo.StateEst(iDistLat), 2.0) - scale_range) > 0.0) {
    if (trace->KalmanInfo.StateEst(iDistLat) > 0.0) {
      trace->ExtendInfo.corner_type |= (1 << ALL_IN_POSITIVE_SIDE);
    } else {
      trace->ExtendInfo.corner_type |= (1 << ALL_IN_NEGATIVE_SIDE);
    }
  } else {
    trace->ExtendInfo.corner_type |= (1 << PASS_THROUGH_LAT);
  }

  uint8_t invisible_num = 0;
  for (uint8_t i = 0; i < 4; i++) {
    if (fabs(ComputeAngle(trace->ExtendInfo.CornerPos[2 * i][0],
                          trace->ExtendInfo.CornerPos[2 * i][1])) >
        (45.0 * DEG2RAD)) {
      invisible_num++;
    }

    if (trace->ExtendInfo.CornerPos[2 * i][1] < 0.0) {
      trace->ExtendInfo.corner_type |= (1 << HAS_NEGATIVE_LONG);
    }
  }

  trace->ExtendInfo.corner_type |= (1 << invisible_num);
}

/**
 * @name: Track_Update_DynPro
 * @description: 更新航迹运动状态
 * @param {trackTable_strcut} *trace
 * @return {*}
 */
void Track_Update_DynPro(trackTable_strcut *trace) {
  ObjectDynProp_enum last_DynProb = trace->ExtendInfo.DynProp;
  ObjectDynProp_enum cur_DynProp = stopped;

  VectorXd tempState = trace->KalmanInfo.StateEst;
  VectorXd tempState2 = trace->KalmanInfo.StateEst_abs;

  // float radiR =
  //     sqrt(pow(tempState(iDistLat), 2.0) + pow(tempState(iDistLong), 2.0));

  // float posSpeed = (tempState(iDistLat) * tempState(iVrelLat) +
  //                   tempState(iDistLong) * tempState(iVrelLong)) /
  //                  radiR;

  double abs_vlat =
      (fabs(tempState2(iVrelLat)) > 0.5) ? tempState2(iVrelLat) : 0.0;
  double abs_vlong =
      (fabs(tempState2(iVrelLong)) > 0.5) ? tempState2(iVrelLong) : 0.0;

  double absSpeed = sqrt(pow(abs_vlat, 2.0F) + pow(abs_vlong, 2.0F));

  // 目标的绝对速度
  if (absSpeed < 0.5) {
    cur_DynProp = stopped;
  } else {
    if (tempState2(iVrelLong) > 0.5) {
      cur_DynProp = moving;
    } else if (tempState2(iVrelLong) < -0.5) {
      cur_DynProp = oncoming;
    }

    // 跟踪朝向信息判定横穿
    if ((fabs(trace->ExtendInfo.box_theta) > (70.0 * DEG2RAD)) &&
        (fabs(trace->ExtendInfo.box_theta) < (110.0 * DEG2RAD))) {
      cur_DynProp = crossing_moving;
    }
  }

  /* 上一时刻是运动状态 */
  if ((last_DynProb == moving) || (last_DynProb == oncoming) ||
      (last_DynProb == crossing_moving)) {
    /* 若本次是静止状态，则对静态计数器累加，并递减运动计数器 */
    if ((cur_DynProp == stopped) || (cur_DynProp == stationary_candidate)) {
      if (trace->ExtendInfo.move2stop >= MOTION_CHANGE_GATE) {
        trace->ExtendInfo.DynProp = cur_DynProp;
        trace->ExtendInfo.stop2move = 0;
      } else {
        trace->ExtendInfo.move2stop++;
      }

      if (trace->ExtendInfo.stop2move > 0) {
        trace->ExtendInfo.stop2move--;
      }
    } else {
      if (cur_DynProp == last_DynProb)  // 本次也是运动且和上次运动方向一致
      {
        if (trace->ExtendInfo.stop2move >= MOTION_CHANGE_GATE) {
          trace->ExtendInfo.DynProp = cur_DynProp;
          trace->ExtendInfo.move2stop = 0;
        } else {
          trace->ExtendInfo.stop2move++;
        }
      } else {
        trace->ExtendInfo.stop2move = 0;
      }

      /* 递减静态计数器 */
      if (trace->ExtendInfo.move2stop > 0) {
        trace->ExtendInfo.move2stop--;
      }
    }
  } else /* 上一时刻是静止状态 */
  {
    if ((cur_DynProp == moving) || (cur_DynProp == oncoming) ||
        (cur_DynProp == crossing_moving)) {
      if (trace->ExtendInfo.stop2move >= MOTION_CHANGE_GATE) {
        trace->ExtendInfo.DynProp = cur_DynProp;
        trace->ExtendInfo.move2stop = 0;
      } else {
        if ((last_DynProb == unknown) &&
            (trace->ExtendInfo.stop2move >= MOTION_CHANGE_GATE_INIT)) {
          trace->ExtendInfo.DynProp = cur_DynProp;
          trace->ExtendInfo.move2stop = 0;
        }

        trace->ExtendInfo.stop2move += ((uint)(absSpeed / 2.0) + 1.0);
      }

      if (trace->ExtendInfo.move2stop > 0) {
        trace->ExtendInfo.move2stop--;
      }
    } else {
      if (trace->ExtendInfo.move2stop >= MOTION_CHANGE_GATE) {
        if (trace->ExtendInfo.DynProp != stationary) {
          trace->ExtendInfo.DynProp = cur_DynProp;
        }

        trace->ExtendInfo.stop2move = 0;
      } else {
        trace->ExtendInfo.move2stop++;
      }

      if (trace->ExtendInfo.stop2move > 0) {
        trace->ExtendInfo.stop2move--;
      }
    }
  }
}

/**
 * @name: Track_Update_TrkType
 * @description: 目标类型以及长宽属性更新
 * @param {RadarOutput_Struct} *indata
 * @param {trackTable_strcut} *trace
 * @return {*}
 */
void Track_Update_TrkType(std::vector<RadarMeasure_struct> &global_point,
                          trackTable_strcut *trace) {
  // update shape
  track_update_shape(global_point, trace);

  // do classifity
  track_classifity(trace);
}

/**
 * @name:
 * @description: 航迹形状参数计算
 * @param {RadarOutput_Struct} *indata
 * @param {trackTable_strcut} *trackInfo
 * @param {float} *newLen
 * @param {float} *newWid
 * @return {*}
 */
void track_compute_newshape(std::vector<RadarMeasure_struct> &global_point,
                            trackTable_strcut *trackInfo, float *newLen,
                            float *newWid) {
  double assoWidth[2];
  double assoLength[2];
  double box_wid = 0.0;
  double box_len = 0.0;

  assoWidth[0] = -999.0F;
  assoWidth[1] = 999.0F;
  assoLength[0] = -999.0F;
  assoLength[1] = 999.0F;

  double max_rcs_filter_val = -100.0;

  for (uint16_t det_idx = 0; det_idx < trackInfo->MeasInfo.detInfo.size();
       det_idx++) {
    uint16_t point_idx = trackInfo->MeasInfo.detInfo[det_idx].cluster_Idx;

    if ((trackInfo->MeasInfo.detInfo[det_idx].weight < 5e-3)) {
      continue;
    }

    if (global_point[point_idx].DistLat > assoWidth[0]) {
      assoWidth[0] = global_point[point_idx].DistLat;
    }
    if (global_point[point_idx].DistLat < assoWidth[1]) {
      assoWidth[1] = global_point[point_idx].DistLat;
    }
    if (global_point[point_idx].DistLong > assoLength[0]) {
      assoLength[0] = global_point[point_idx].DistLong;
    }
    if (global_point[point_idx].DistLong < assoLength[1]) {
      assoLength[1] = global_point[point_idx].DistLong;
    }

    box_wid = assoWidth[0] - assoWidth[1];
    box_len = assoLength[0] - assoLength[1];

    if (max_rcs_filter_val < global_point[point_idx].RCS) {
      max_rcs_filter_val = global_point[point_idx].RCS;
    }
  }

  if (max_rcs_filter_val < -100.0) {
    trackInfo->MeasInfo.max_rcs_filter_val +=
        (max_rcs_filter_val - trackInfo->MeasInfo.max_rcs_filter_val) * 0.3;
  }

  box_len = (box_len > 0.5) ? box_len : 0.5;
  box_wid = (box_wid > 0.5) ? box_wid : 0.5;

  *newLen = box_len;
  *newWid = box_wid;
}

/**
 * @name: track_update_shape
 * @description: 航迹形状参数更新
 * @param {trackTable_strcut} *trace
 * @param {float} Length
 * @param {float} Width
 * @return {*}
 */
void track_update_shape(std::vector<RadarMeasure_struct> &global_point,
                        trackTable_strcut *trace) {
  float newWidth, newLen;
  float scale4extend = 0.1F;
  float scale4descrea = 0.05F;

  float scale4notAsso = 0.95F;

  float diff_Wid, diff_Len;

  // fitting meas "shape"
  float Length, Width;

  if ((trace->ExtendInfo.Object_Class >= VEHICLE) &&
      ((trace->ExtendInfo.DynProp == stationary_candidate) ||
       (trace->ExtendInfo.DynProp == stationary) ||
       (trace->ExtendInfo.DynProp == stopped))) {
    return;
  }

  if (trace->trackState == TRACK_STATE_DETECTION) {
    scale4extend = 0.2;
  }

  // 拟合量测点
  track_compute_newshape(global_point, trace, &Length, &Width);

  if (trace->MeasInfo.associateNum > 1) {
    newWidth = Width;
    newLen = Length;

    if (trace->ExtendInfo.DynProp == crossing_moving) {
      newWidth = Length;
      newLen = Width;
    }

    diff_Wid = (newWidth - trace->ExtendInfo.Width);
    diff_Len = (newLen - trace->ExtendInfo.Length);

    // 抑制正前方车辆目标的形状突变（降低double-bounce对尺寸的影响）
    if ((trace->ExtendInfo.Object_Class >= VEHICLE) &&
        (fabs(trace->KalmanInfo.StateEst(iDistLat)) < 1.0) &&
        (diff_Len > 2.0)) {
      return;
    }

    if ((trace->ExtendInfo.Object_Class >= VEHICLE) &&
        (fabs(trace->KalmanInfo.StateEst(iDistLat)) < 1.0)) {
      scale4extend = 0.02;
    }

    // 横穿航迹
    if ((trace->ExtendInfo.DynProp == crossing_moving) ||
        (trace->ExtendInfo.DynProp == crossing_stationary)) {
      if ((trace->ExtendInfo.corner_type & (1 << PASS_THROUGH_LAT)) ==
          (1 << PASS_THROUGH_LAT)) {
        diff_Wid = Valuelimit(-0.5, 0.5, diff_Wid);
        diff_Len = Valuelimit(-1.0, 1.0, diff_Len);
      } else {
        diff_Wid = Valuelimit(-0.5, 0.5, diff_Wid);
        diff_Len = Valuelimit(-2.0, 2.0, diff_Len);
      }
    } else {
      diff_Wid = Valuelimit(-0.5, 0.5, diff_Wid);
      diff_Len = Valuelimit(-2.0, 2.0, diff_Len);
    }

    if (trace->ExtendInfo.Width > newWidth) {
      trace->ExtendInfo.Width += diff_Wid * scale4descrea;
    } else {
      trace->ExtendInfo.Width += diff_Wid * scale4extend;
    }

    if (trace->ExtendInfo.Length > newLen) {
      trace->ExtendInfo.Length += diff_Len * scale4descrea;
    } else {
      trace->ExtendInfo.Length += diff_Len * scale4extend * 4.0;
    }
  } else {
    trace->ExtendInfo.Width *= scale4notAsso;
    trace->ExtendInfo.Length *= scale4notAsso;
  }

  trace->ExtendInfo.max_len =
      (trace->ExtendInfo.Length > trace->ExtendInfo.max_len)
          ? trace->ExtendInfo.Length
          : trace->ExtendInfo.max_len;

  trace->ExtendInfo.max_wid =
      (trace->ExtendInfo.Width > trace->ExtendInfo.max_wid)
          ? trace->ExtendInfo.Width
          : trace->ExtendInfo.max_wid;
}

/**
 * @name: track_classifity
 * @description: 航迹目标分类逻辑
 * @param {trackTable_strcut} *trace
 * @return {*}
 */
void track_classifity(trackTable_strcut *trace) {
  ObjectType_enum tempType;

  // 航迹绝对速度
  float RadialVel = sqrt(pow(trace->KalmanInfo.StateEst_abs(iVrelLat), 2.0) +
                         pow(trace->KalmanInfo.StateEst_abs(iVrelLong), 2.0));

  if (RadialVel > 2.0F)  //
  {
    tempType = VEHICLE;

    if (trace->ExtendInfo.Length > 6.0F) {
      tempType = TRUCK;
    }
  } else {
    tempType = PEDESTRIAN;

    if (trace->MeasInfo.max_rcs_filter_val < 10.0)  // 新增历史量测最大RCS判定
    {
      if (RadialVel > 2.0F) {
        tempType = BICYCLE;
      } else {
        if ((trace->ExtendInfo.Width >= 1.5F) ||
            (trace->ExtendInfo.Length >= 1.5F)) {
          tempType = BICYCLE;
        } else {
          tempType = PEDESTRIAN;
        }
      }

      if (((trace->ExtendInfo.Width > 1.5F) &&
           (trace->ExtendInfo.Length > 2.5F)) ||
          (trace->ExtendInfo.Length > 3.0F) ||
          ((trace->ExtendInfo.Width * trace->ExtendInfo.Length) > 4.0)) {
        tempType = VEHICLE;

        if (trace->ExtendInfo.Length > 6.0F) {
          tempType = TRUCK;
        }
      }
    } else {
      if (((trace->ExtendInfo.Width > 1.5F) &&
           (trace->ExtendInfo.Length > 2.5F)) ||
          (trace->ExtendInfo.Length > 3.0F) ||
          ((trace->ExtendInfo.Width * trace->ExtendInfo.Length) > 4.0)) {
        tempType = VEHICLE;

        if (trace->ExtendInfo.Length > 6.0F) {
          tempType = TRUCK;
        }
      }
    }
  }

  if (tempType > trace->ExtendInfo.Object_Class) {
    trace->ExtendInfo.DownCount = 0;
    if (trace->ExtendInfo.UpCount < 20) {  // 连续N帧进行目标类型切换
      trace->ExtendInfo.UpCount++;
    } else {
      trace->ExtendInfo.Object_Class = tempType;
    }
  } else {
    trace->ExtendInfo.UpCount = 0;
    if ((trace->ExtendInfo.Object_Class == BICYCLE) &&
        (tempType == PEDESTRIAN)) {
      if (trace->ExtendInfo.DownCount < 20) {
        trace->ExtendInfo.DownCount++;
      } else {
        trace->ExtendInfo.Object_Class = tempType;
      }
    }
  }

  // value limit
  track_shapeparalimit(trace);
}

/**
 * @name: track_shapeparalimit
 * @description: 对航迹的长宽属性范围进行数值限定
 * @param {trackTable_strcut} *trace
 * @return {*}
 */
void track_shapeparalimit(trackTable_strcut *trace) {
  if (trace->ExtendInfo.Object_Class >= VEHICLE) {
    if (trace->ExtendInfo.Width < 2.0) {
      trace->ExtendInfo.Width =
          (trace->ExtendInfo.max_wid > 2.0) ? 2.0 : trace->ExtendInfo.max_wid;
    }

    if (trace->ExtendInfo.Length < 4.5) {
      trace->ExtendInfo.Length =
          (trace->ExtendInfo.max_len > 2.0) ? 4.5 : trace->ExtendInfo.max_len;
    }
  } else {
    if (trace->ExtendInfo.Width < 0.5) {
      trace->ExtendInfo.Width = 0.5F;
    }

    if (trace->ExtendInfo.Length < 0.5F) {
      trace->ExtendInfo.Length = 0.5F;
    }
  }

  // set height
  switch (trace->ExtendInfo.Object_Class) {
    case PEDESTRIAN:
      trace->ExtendInfo.height = 1.7F;
      break;
    case BICYCLE:
      trace->ExtendInfo.height = 1.7F;
      break;
    case VEHICLE:
      trace->ExtendInfo.height = 1.6F;
      break;
    case TRUCK:
      trace->ExtendInfo.height = 1.7F;
      break;
    default:
      trace->ExtendInfo.height = 0.5F;
      break;
  }
}

/**
 * @name: UpdateSurvivalProb
 * @description:
 * @param {trackTable_strcut} *trackInfo
 * @return {*}
 */
void UpdateSurvivalProb(trackTable_strcut *trace,
                        std::vector<RadarMeasure_struct> &global_point,
                        vehicleInfo_struct *vehicleInfo) {
  std::vector<matchInfo_t> new_detInfo;

  if (trace->trackState >= TRACK_STATE_DETECTION) {
    // 权重归一化
    double sumW = 0.0F;

    for (auto &sub_det : trace->MeasInfo.detInfo) {
      sumW += sub_det.weight;
    }

    for (auto &sub_det : trace->MeasInfo.detInfo) {
      if (sub_det.used4Fit == false) {
        continue;
      }

      sub_det.weight = sub_det.weight / sumW;

      /*
       * 1、权重较低且rcs较小
         2、航迹只有这一个点且rcs较小
        */
      if ((trace->MeasInfo.detInfo.size() == 1) &&
          ((global_point[sub_det.cluster_Idx].RCS < -15.0) ||
           (sub_det.weight < 1e-4))) {
        continue;
      }

      /* 非车辆目标: rcs较小且权重较小 */
      if ((trace->ExtendInfo.Object_Class < VEHICLE) &&
          (global_point[sub_det.cluster_Idx].RCS < -15.0) &&
          (sub_det.weight < 0.2)) {
        continue;
      }

      /* 车辆目标：rcs较小或权重较低 */
      if ((trace->ExtendInfo.Object_Class >= VEHICLE) &&
          ((global_point[sub_det.cluster_Idx].RCS < -20.0) &&
           (sub_det.weight < 1e-3))) {
        continue;
      }

      new_detInfo.push_back(sub_det);
    }

    trace->MeasInfo.detInfo = new_detInfo;
    trace->MeasInfo.associateNum = new_detInfo.size();

    /* 根据关联情况更新存在概率 */
    if (trace->trackState == TRACK_STATE_DETECTION) {
      trace->ExtendInfo.ProbOfExist *= 0.95;
    }

    if (trace->MeasInfo.detInfo.size() > 0) {
      double add_value = 0.0;

      if (trace->trackState == TRACK_STATE_ACTIVE) {
        if (trace->ExtendInfo.Object_Class == PEDESTRIAN) {
          add_value = (0.6 + 0.4 / trace->MeasInfo.associateNum) * sumW;
        } else {
          add_value = (0.4 + 0.2 / trace->MeasInfo.associateNum) * sumW;
        }
      } else {
        add_value = (0.2 + 0.4 / trace->MeasInfo.associateNum) * sumW;
      }

      add_value =
          Valuelimit(0.0, 0.6 + trace->ManageInfo.trace_belief * 2, add_value);

      trace->ExtendInfo.ProbOfExist += add_value;

      if (fabs(vehicleInfo->yaw_rate) < (5.0 * DEG2RAD)) {
        double dec_fac = 0.0;
        double trace_size =
            trace->ExtendInfo.Width + trace->ExtendInfo.Length * 0.5;
        trace_size = Valuelimit(1.0, 40.0, trace_size);

        dec_fac = (trace->MeasInfo.detInfo.size() > 0)
                      ? (trace->MeasInfo.detInfo.size() / trace_size)
                      : ((trace->MeasInfo.detInfo.size() + 1.0) / trace_size);
        dec_fac = Valuelimit(0.8, 1.0, dec_fac);

        trace->ExtendInfo.ProbOfExist *= dec_fac;
      }
    } else {
      /* 未关联到量测的情况下，进一步降低存在概率 */
      if (trace->trackState == TRACK_STATE_ACTIVE) {
        if (trace->ExtendInfo.Object_Class >= VEHICLE) {
          trace->ExtendInfo.ProbOfExist *=
              (0.8 - trace->ManageInfo.tracking2missCount * 0.1);
        } else {
          trace->ExtendInfo.ProbOfExist *= 0.9;
        }
      } else {
        if ((trace->ExtendInfo.Length * trace->ExtendInfo.Width) <
            0.64)  // 0.8*0.8=0.64
        {
          // nothing todo
        } else {
          trace->ExtendInfo.ProbOfExist *=
              (0.7 - trace->ManageInfo.tracking2missCount * 0.1);
        }
      }
    }

    std::cout << "ProbOfExist of " << "Trace ID:[ " <<  trace->trackID << "] " << trace->ExtendInfo.ProbOfExist << std::endl;

    trace->ExtendInfo.ProbOfExist =
        Valuelimit(0.01, TRACE_PROB_LIMIT, trace->ExtendInfo.ProbOfExist);
    trace->ExtendInfo.prob_exist_output =
        trace->ExtendInfo.ProbOfExist / TRACE_PROB_LIMIT;
  }
}
