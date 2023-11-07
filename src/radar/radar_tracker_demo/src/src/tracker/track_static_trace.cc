/*
 * @description:
 * @version:
 * @Author: ChengHao
 * @Date: 2022-10-08 14:05:34
 * @LastEditors: ChengHao hao.cheng@wuzheng.com
 * @LastEditTime: 2023-11-07 14:31:41
 */

#include "track_static_trace.h"

#include "commonfunctions.h"
#include "common/DBSCAN.h"
#include "track_manage.h"

#include "../../include/common/iou.h"

#ifdef ARS548_RADAR
#define FIFO_DEPTH (3)
#else
#define FIFO_DEPTH (1)
#endif

std::deque<static_point_snapshot_t> snapshot_fifo;
static uint8_t high_motor_driven = 0;
vehicleInfo_struct vehicle_temp;

/**
 * @name:
 * @description:
 * @return {*}
 */
bool run_static_filter(void) { return (snapshot_fifo.size() >= FIFO_DEPTH); }

/**
 * @name:
 * @description:
 * @param {RadarOutput_Struct} *indata
 * @param {  } std
 * @param {  } vehicleInfo_
 * @return {*}
 */
void fifo_point_proc(RadarOutput_Struct *indata,
                     std::vector<RadarMeasure_struct> &fifo_point,
                     vehicleInfo_struct &vehicleInfo) {
  static_point_snapshot_t cur_snapshot;
  cur_snapshot.vehicleInfo = vehicleInfo;
  vehicle_temp = vehicleInfo;
  for (uint16_t n = 0; n < indata->Header.ActualRecNum; n++) {
    if((indata->RadarMeasure[n].invalid == (1 << invalid_det)) || (indata->RadarMeasure[n].RCS < -8.0))
    {
      continue;
    }
    cur_snapshot.point_set.push_back(indata->RadarMeasure[n]);
  }

  if (snapshot_fifo.size() >= FIFO_DEPTH) {
    snapshot_fifo.pop_front();
  }

  // 更新fifo
  snapshot_fifo.push_back(cur_snapshot);

  // 点云转换
  fifo_point = trans_point(snapshot_fifo);
}

/**
 * @name:
 * @description:
 * @param {  } std
 * @return {*}
 */
std::vector<RadarMeasure_struct> trans_point(
    std::deque<static_point_snapshot_t> &fifo_set) {
  std::vector<RadarMeasure_struct> all_point =
      fifo_set.at(fifo_set.size() - 1).point_set;

  if (fifo_set.size() < FIFO_DEPTH) {
    return all_point;
  }

  uint16_t fifo_idx = 1;
  for (int8_t idx = (FIFO_DEPTH - 2); idx >= 0; idx--) {
    uint16_t newID = 1000 * fifo_idx;

    static_point_snapshot_t &his_snap = fifo_set.at(idx);

    double diff_lat = 0.0;
    double diff_long =
        (FIFO_DEPTH - idx - 1) * 0.07 * his_snap.vehicleInfo.vx * (-1.0);
    double diff_ang =
        (FIFO_DEPTH - idx - 1) * 0.07 * his_snap.vehicleInfo.yaw_rate * -1.0;

    // 历史帧数据处理：补偿位移并且重分配ID
    double tempLat, tempLong;
    for (auto &sub_point : his_snap.point_set) {
      if (sub_point.DynProp == stationary) {
        tempLat = sub_point.DistLat + diff_lat;
        tempLong = sub_point.DistLong + diff_long;
        sub_point.ID = newID;

        sub_point.DistLong =
            tempLong * cos(diff_ang) + tempLat * (-1.0 * sin(diff_ang));
        sub_point.DistLat = tempLong * sin(diff_ang) + tempLat * cos(diff_ang);

        newID++;
        all_point.push_back(sub_point);
      }
    }

    fifo_idx++;
  }

  return all_point;
}

/**
 * @name: trace_meas_dbscan
 * @description: 静态点云聚类
 * @param {RadarOutput_Struct} *indata
 * @param {std::vector<DBSCAN::Point4DBSCAN>} &PointSet
 * @param {std::vector<std::vector<uint16_t>>} &clusterSet
 * @return {*}
 */
void trace_meas_dbscan(std::vector<RadarMeasure_struct> &point_fifo,
                       std::vector<std::vector<uint16_t>> &clusterSet) {
  DBSCAN::Point4DBSCAN Point;
  std::vector<DBSCAN::Point4DBSCAN> PointSet;

  for (uint16_t n = 0; n < point_fifo.size(); n++) {
    RadarMeasure_struct *sub_point = &point_fifo.at(n);
    if ((sub_point->invalid == (1 << valid_det)) &&
        (sub_point->DynProp == stationary)) {
      // 低RCS点迹不参与计算
      if ((sub_point->DistLong > STATIC_RANGE_LONG) ||
          (fabs(sub_point->DistLat) > STATIC_RANGE_LAT) ||
          (sub_point->RCS < -15.0)) {
        continue;
      }

      if ((fabs(sub_point->DistLat) < 10.0) &&
          (sub_point->DistLong < (STATIC_RANGE_LONG * 0.5)) &&
          (sub_point->RCS < -10.0) && (sub_point->ProbOfExist < 0.1)) {
        continue;
      }

      // fov外的点不进行处理
      double ang_info =
          atan2(sub_point->DistLat, sub_point->DistLong) / 3.14F * 180.0F;

      if (fabs(ang_info) > 45.0) {
        continue;
      }

      Point.PointInfo.ID = n;
      Point.PointInfo.DistLat = sub_point->DistLat;
      Point.PointInfo.DistLong = sub_point->DistLong;
      Point.PointInfo.RCS = sub_point->RCS;
      Point.PointInfo.V = sub_point->VrelLong;
      Point.PointInfo.DynProp = sub_point->DynProp;
      Point.PointInfo.valid = true;

      Point.PointInfo.Range = sqrt(pow(Point.PointInfo.DistLat, 2.0) +
                                   pow(Point.PointInfo.DistLong, 2.0));

      Point.PointInfo.Azi = ang_info;

#ifdef ARS548_RADAR
      Point.DBSCAN_para.minPts = 2;
      Point.DBSCAN_para.Search_R = 1.0F;
#else

      // 不同信噪比的点云的簇最小个数门限不同
      if(Point.PointInfo.RCS > 10.0)
      {
          Point.DBSCAN_para.minPts = 4;
          Point.DBSCAN_para.Search_R = 2.5F;
      }
      else if(Point.PointInfo.RCS > 0.0)
      {
          Point.DBSCAN_para.minPts = 4;
          Point.DBSCAN_para.Search_R = 2.0F;
      }
      else
      {
          Point.DBSCAN_para.minPts = 4;
          Point.DBSCAN_para.Search_R = 1.5F;
      }

      if(fabs(Point.PointInfo.DistLat) < 1.0)
      {
          Point.DBSCAN_para.Search_R = 2.0F;
      }

#endif

      Point.DBSCAN_para.pointType = 255;

      Point.DBSCAN_para.static_or_dyna = 1;

      PointSet.push_back(Point);
    }
  }

  // DO DBSCAN
  DBSCAN::KNN_DBSCAN(PointSet, clusterSet);

  for (auto &sub_cluster : clusterSet) {
    for (auto &sud_point_idx : sub_cluster) {
      sud_point_idx = PointSet[sud_point_idx].PointInfo.ID;
    }
  }
}

/**
 * @name:
 * @description:
 * @param {gridTrack_t} *track_list
 * @param {  } std
 * @param {  } std
 * @return {*}
 */
void creat_valid_invalid_trace_list(gridTrack_t *track_list,
                                    std::vector<uint8_t> &valid_empty_list,
                                    std::vector<uint8_t> &trace_list) {
  for (uint16_t i = 0; i < STATIC_TRACK_MAXNUM; i++) {
    if (track_list[i].status > EMPTY) {
      trace_list.push_back(i);
    } else {
      valid_empty_list.push_back(i);
    }
  }
}

/**
 * @name:
 * @description:
 * @param {gridTrack_t} *track_list
 * @param {double} delta_t
 * @return {*}
 */
void track_predict_static(gridTrack_t *track_list,
                          vehicleInfo_struct *vehicleInfo, double delta_t) {
  double yawange = (delta_t * vehicleInfo->yaw_rate);
  double vel_coef = 0.95;
  static uint high_motion_count = 0;

  if (fabs(vehicleInfo->yaw_rate) > 0.05) {
    if (high_motor_driven < 20)  // 20 * 0.075s = 1.5s
    {
      high_motor_driven += 5;
    }

    if(high_motion_count < 40)
    {
      high_motion_count += 1;
    }
  } else {
    if (high_motor_driven > 0) {
      high_motor_driven--;
      vel_coef = 0.8;
    }

    if(high_motion_count)
    {
        high_motion_count--;
    }
  }

  MatrixXd rotMat = MatrixXd::Zero(2, 2);
  MatrixXd rotMat2 = MatrixXd::Zero(6, 6);

  rotMat(0, 0) = cos(yawange);
  rotMat(0, 1) = -1.0 * sin(yawange);
  rotMat(1, 0) = sin(yawange);
  rotMat(1, 1) = cos(yawange);

  rotMat2.block<2, 2>(0, 0) = rotMat;
  rotMat2.block<2, 2>(2, 2) = rotMat;
  rotMat2.block<2, 2>(4, 4) = rotMat;

  // 运动噪声
  double dt = delta_t;
  double dt3 = powf(delta_t, 3.0);
  double dt2 = powf(delta_t, 2.0);

  double std_accLat_ = 0.2;
  double std_accLong_ = 0.2;

  if(high_motor_driven)
  {
    std_accLat_ = 0.4;
    std_accLong_ = 0.4;
  }

  Eigen::MatrixXd F = Eigen::MatrixXd(6, 6);
  Eigen::MatrixXd Q_ = Eigen::MatrixXd::Identity(6, 6);
  Eigen::MatrixXd Q_2 = Eigen::MatrixXd::Identity(6, 6);

  MatrixXd G = MatrixXd::Zero(SSIZE, 2);
  MatrixXd q = MatrixXd::Zero(2, 2);

  F << 1.0, 0.0, dt, 0.0, 0.5 * dt2, 0.0, 0.0, 1.0, 0.0, dt, 0.0, 0.5 * dt2,
      0.0, 0.0, vel_coef, 0.0, dt, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, dt, 0.0, 0.0,
      0.0, 0.0, vel_coef, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.9;

  G << 1 / 6 * dt3, 0, 0, 1 / 6 * dt3, 0.5 * dt2, 0, 0, 0.5 * dt2, dt, 0, 0, dt;
  q << powf(std_accLat_, 2.0), 0.0, 0.0, powf(std_accLong_, 2.0);
  Q_ = G * q * G.transpose();

  Eigen::MatrixXd F_T = F.transpose();

  for (uint16_t i = 0; i < STATIC_TRACK_MAXNUM; i++)  // TBD
  {
    gridTrack_t &subTrace = track_list[i];

    if((high_motion_count == 40.0) && (subTrace.status > EMPTY))
    {
        subTrace.status = EMPTY;
    }

    if (subTrace.status > EMPTY)  // can be used to build a new obj
    {
      /*航迹预测阶段
      1、状态预测
      2、存在概率预测
      */
      subTrace.age++;

      subTrace.ProbOfExist *= (subTrace.status == UNACTIVATE) ? 0.9 : 0.95;

      // 卡尔曼预测
      Q_2 = Q_;
      double angle_max = compute_max_angle(subTrace.kf_info.X_(iDistLat),
                                           subTrace.kf_info.X_(iDistLong),
                                           subTrace.len, subTrace.wid);
      if(fabs(angle_max) < (40.0 * DEG2RAD))
      {
          if (high_motor_driven > 0) {
            Q_2(0, 0) += fabs(subTrace.diff_X) * 0.002;  // add X_NOISE
            Q_2(1, 1) += fabs(subTrace.diff_Y) * 0.002;  // add Y_NOISE
            Q_2(2, 2) += fabs(subTrace.diff_X) * 0.004;  // add VX_NOISE
            Q_2(3, 3) += fabs(subTrace.diff_Y) * 0.004;  // add VY_NOISE
          } else {
            Q_2(3, 3) += (fabs(subTrace.diff_Y) * 0.004 +
                          fabs(subTrace.diff_V) * 0.004);  // add VY_NOISE
          }
      }

      kf_predict(subTrace.kf_info.X_, subTrace.kf_info.P_, F, F_T, Q_2);

      subTrace.kf_info.X_no_comp = subTrace.kf_info.X_;
      subTrace.kf_info.X_ = rotMat2 * subTrace.kf_info.X_;
    }
  }
}

/**
 * @name:
 * @description:
 * @param {vector<std::vector<uint16_t>>} &clusterSet
 * @param {  } std
 * @param {  } std
 * @param {box_dyn_static_enum} box_feature
 * @return {*}
 */
void creat_det_box(std::vector<std::vector<uint16_t>> &clusterSet,
                   std::vector<RadarMeasure_struct> &point_fifo,
                   std::vector<det_box_t> &det_box_list,
                   box_dyn_static_enum box_feature) {
  // 将簇拟合为矩形
  std::vector<float> meas_x;
  std::vector<float> meas_y;
  std::vector<float> meas_v;
  for (auto const &temp_cluster : clusterSet) {
    meas_x.clear();
    meas_y.clear();
    meas_v.clear();

    for (auto const &pointIdx : temp_cluster) {
      meas_x.push_back(point_fifo.at(pointIdx).DistLat);
      meas_y.push_back(point_fifo.at(pointIdx).DistLong);
      meas_v.push_back(point_fifo.at(pointIdx).VrelLong);
    }

    auto maxmin_val_x = std::minmax_element(meas_x.begin(), meas_x.end());
    auto maxmin_val_y = std::minmax_element(meas_y.begin(), meas_y.end());
    auto maxmin_val_v = std::minmax_element(meas_v.begin(), meas_v.end());

    det_box_t det_box;

    det_box.valid = (1 << VALID);
    det_box.member_num = temp_cluster.size();
    det_box.x_pos = (*maxmin_val_x.second + *maxmin_val_x.first) * 0.5;
    det_box.y_pos = (*maxmin_val_y.second + *maxmin_val_y.first) * 0.5;
    det_box.v_mean = (*maxmin_val_v.second + *maxmin_val_v.first) * 0.5;
    det_box.wid = (*maxmin_val_x.second - *maxmin_val_x.first);
    det_box.len = (*maxmin_val_y.second - *maxmin_val_y.first);

    if ((box_feature == STATIC_BOX) &&
        ((det_box.wid < 0.2) && (det_box.len < 0.4))) {
      det_box.valid |= (1 << NOT_USED_FOR_INIT);
    }
    if ((box_feature == STATIC_BOX) && (det_box.member_num < 4)) {
      det_box.valid |= (1 << NOT_USED_FOR_INIT);
    }

    det_box.wid = (det_box.wid < 0.4) ? 0.4 : det_box.wid;
    det_box.len = (det_box.len < 0.4) ? 0.4 : det_box.len;

    det_box.box_feature = box_feature;
    det_box.assigned_obj = RESERVED_TRACK;
    det_box.matched_trace_idx = 65535;

    det_box_list.emplace_back(det_box);
  }
}


/**
 * @name:
 * @description:
 * @param {gridTrack_t} *track_list
 * @param {vector<uint8_t>} &valid_trace_list
 * @param {  } std
 * @param {vector<std::vector<uint8_t>>} &all_assigned_box
 * @return {*}
 */
void static_trace_update(
    gridTrack_t *track_list, const std::vector<uint8_t> &valid_trace_list,
    std::vector<det_box_t> &det_box_list,
    const std::vector<std::vector<uint16_t>> &all_assigned_box) {
  box_t track_bbox, new_box;
  uint8_t assign_dyn_box = 0;
  for (uint8_t trk_idx = 0; trk_idx < valid_trace_list.size();
       trk_idx++)  // TBD
  {
    gridTrack_t &subTrace = track_list[valid_trace_list.at(trk_idx)];

    track_bbox.x = subTrace.kf_info.X_(0);
    track_bbox.y = subTrace.kf_info.X_(1);
    track_bbox.len = subTrace.len;
    track_bbox.wid = subTrace.wid;

    bool assigned_flag = false;
    assign_dyn_box = 0;

    // 若该航迹分配到量测box
    if (all_assigned_box.at(trk_idx).size() > 0) {
      // fitting a new "det_box"
      det_box_t &det_box = det_box_list.at(all_assigned_box.at(trk_idx).at(0));

      new_box.x = det_box.x_pos;
      new_box.y = det_box.y_pos;
      new_box.len = det_box.len;
      new_box.wid = det_box.wid;
      new_box.v = det_box.v_mean;

      for (auto const &det_box_idx : all_assigned_box.at(trk_idx)) {
        box_t temp_box;
        det_box_t &det_box = det_box_list.at(det_box_idx);

        temp_box.x = det_box.x_pos;
        temp_box.y = det_box.y_pos;
        temp_box.len = det_box.len;
        temp_box.wid = det_box.wid;
        temp_box.v = det_box.v_mean;

        if (det_box.box_feature == DYNAMIC_BOX) {
          assign_dyn_box++;
        }

        new_box = box_fusion(temp_box, new_box);
      }

      assigned_flag = true;
    }

    // 判定该航迹最大角度
    double angle_max = compute_max_angle(
                        subTrace.kf_info.X_no_comp(iDistLat),
                        subTrace.kf_info.X_no_comp(iDistLong), subTrace.len, subTrace.wid);

    // 20230529 add：异常box判定
    double diff_r = sqrt(pow(new_box.x - subTrace.kf_info.X_no_comp(iDistLat), 2.0) + \
                         pow(new_box.y - subTrace.kf_info.X_no_comp(iDistLong), 2.0));
    if(assigned_flag)
    {
      if(((fabs(new_box.len - subTrace.len) > 6.0) || (fabs(new_box.wid - subTrace.wid) > 4.0) || (diff_r > 5.0) ) && \
        (subTrace.status == UNACTIVATE)){
        assigned_flag = false;
      }

      if((((new_box.len - subTrace.len) > 6.0) || ((new_box.wid - subTrace.wid) > 4.0) || (diff_r > 5.0)) && \
            (subTrace.status == ACTIVATE)){
        assigned_flag = false;
      }
    }

    if (!assigned_flag) {
      subTrace.UnAssoNum++;
    } else
    {
        subTrace.UnAssoNum = 0;

        if (fabs(angle_max) < (45.0 * DEG2RAD)) {
            // 形状参数更新
            double len_add, wid_add;
            len_add = Valuelimit(-0.1, 0.2, (new_box.len - subTrace.len));
            wid_add = Valuelimit(-0.05, 0.1, (new_box.wid - subTrace.wid));

            subTrace.len += len_add;
            subTrace.wid += wid_add;

            subTrace.len = Valuelimit(0.5, 10, subTrace.len);
            subTrace.wid = Valuelimit(0.5, 10, subTrace.wid);

            double new_height = 0.5;
            if ((subTrace.len * subTrace.wid) > 4.0) {
              new_height = 1.0;

              if (subTrace.len > 6.0) {
                new_height = 1.5;
              }
            }

            if (new_height > subTrace.height) {
              subTrace.height = new_height;
            }
        }

        // 计算航迹预测量测和实际量测
        Eigen::VectorXd measZ = VectorXd(3);
        Eigen::VectorXd preZ = VectorXd(3);

        compute_predictZ_and_measZ(subTrace, new_box, preZ, measZ);

        // 若最大角度超过阈值，则不进行kalman更新过程
//        if (fabs(angle_max) < (40.0 * DEG2RAD)) {
          // 卡尔曼状态更新
          kf_update(subTrace.kf_info.X_no_comp, subTrace.kf_info.P_, preZ, measZ,
                    1.0, subTrace);

          subTrace.kf_info.X_ = subTrace.kf_info.X_no_comp;
//        }
    }

    // 本次关联到的box中，是否有运动点迹
    if (assign_dyn_box > 0) {
      subTrace.dyn_num++;
    } else {
      if (subTrace.dyn_num > 0) {
        subTrace.dyn_num--;
      }
    }

    // 存在置信度更新
    if (assigned_flag) {
      static_trace_update_prob(subTrace, track_bbox, new_box);
    }

    // 航迹状态更新
    static_trace_manag(subTrace, assigned_flag);
  }
}
/**
 * @names:
 * @description: Briefly describe the function of your function
 * @param {gridTrack_t} &subTrace
 * @param {bool} assigned_flag
 * @return {*}
 */
void static_trace_manag(gridTrack_t &subTrace, const bool assigned_flag) {
  // 航迹状态更新
  if (subTrace.status == UNACTIVATE) {
    if ((subTrace.ProbOfExist > 3.0) && (assigned_flag)) {
      subTrace.High_Coun++;
    } else {
      if (subTrace.ProbOfExist < 3.0) {
        subTrace.High_Coun = 0;
      }

      if (subTrace.High_Coun > 0) {
        subTrace.High_Coun--;
      }
    }

    if ((subTrace.age > 5) && (subTrace.High_Coun > 5.0)) {
      subTrace.status = ACTIVATE;
    }

    if ((subTrace.age <= 3) && (!assigned_flag)) {
      subTrace.status = EMPTY;
#ifdef LOG_OUTPOUT
      std::cout << " static_trace_manag P1: Delete Trace ID: " << subTrace.ID
                << std::endl;
#endif
    }
  }

  // 简单的航迹删除逻辑
  if (subTrace.status == UNACTIVATE) {
    if ((subTrace.UnAssoNum >= 3) || (subTrace.ProbOfExist < 0.25) ||
        (subTrace.ProbOfExist < (0.5 * subTrace.max_prob))) {
      subTrace.status = EMPTY;
#ifdef LOG_OUTPOUT
      std::cout << " static_trace_manag P2:Delete Trace ID: " << subTrace.ID
                << std::endl;
#endif
    }
  } else {
    double angle_min = compute_min_angle(subTrace.kf_info.X_(iDistLat),
                                         subTrace.kf_info.X_(iDistLong),
                                         subTrace.len, subTrace.wid);

    if ((subTrace.ProbOfExist < 2.0) || (fabs(angle_min) > (45.0 * DEG2RAD)) ||
        (subTrace.UnAssoNum >= 5)) {
      subTrace.status = EMPTY;
#ifdef LOG_OUTPOUT
      std::cout << " static_trace_manag P3: Delete Trace ID: " << subTrace.ID
                << std::endl;
#endif
    }
  }
}

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @param {gridTrack_t} &trace
 * @param {box_t} &track_bbox
 * @param {box_t} &det_box
 * @return {*}
 */
void static_trace_update_prob(gridTrack_t &trace, box_t &track_bbox,
                              box_t &det_box) {
  double distance_ = compute_box_distance(track_bbox, det_box);
  double s1 = track_bbox.len * track_bbox.wid;
  double s2 = det_box.len * det_box.wid;

  double ratio_ = (s1 < s2) ? (s1 / s2) : (s2 / s1);

  // 存在概率更新
  double add_val = 0.0;
#ifdef ARS548_RADAR
  add_val = (0.6 * distance_ + 0.4 * ratio_);
  add_val = Valuelimit(0.0, 1.0, add_val);
#else

  if (trace.status == UNACTIVATE) {
    ratio_ = Valuelimit(0.1, 1.0, ratio_);
    add_val = (0.2 * distance_ + 0.8 * ratio_);
    add_val = Valuelimit(0.0, 0.5, add_val);
  } else {
    ratio_ = Valuelimit(0.1, 1.0, ratio_);
    add_val = (0.4 * distance_ + 0.6 * ratio_);
    add_val = Valuelimit(0.0, 1.0, add_val);
  }

#endif

  trace.ProbOfExist += add_val;
  trace.ProbOfExist = Valuelimit(0.0, TRACE_PROB_LIMIT, trace.ProbOfExist);
  trace.Prob_Output = trace.ProbOfExist / TRACE_PROB_LIMIT;

  // 更新参数： 历史最大置信度
  if (trace.ProbOfExist > trace.max_prob) {
    trace.max_prob = trace.ProbOfExist;
  }
}
/**
 * @names:
 * @description: Briefly describe the function of your function
 * @param {gridTrack_t} &trace
 * @param {box_t} &new_box
 * @param {  } Eigen
 * @param {VectorXd} &measZ
 * @return {*}
 */
void compute_predictZ_and_measZ(gridTrack_t &trace, box_t &new_box,
                                Eigen::VectorXd &preZ, Eigen::VectorXd &measZ) {
  // 判定该航迹最大角度
  double angle_max = compute_max_angle(trace.kf_info.X_no_comp(iDistLat),
                                       trace.kf_info.X_no_comp(iDistLong),
                                       trace.len, trace.wid);

  double range_ = sqrt(pow(trace.kf_info.X_no_comp(iDistLat), 2.0) +
                       pow(trace.kf_info.X_no_comp(iDistLong), 2.0));
  double temp_data =
      trace.kf_info.X_no_comp(iDistLat) * trace.kf_info.X_no_comp(iVrelLat) +
      trace.kf_info.X_no_comp(iDistLong) * trace.kf_info.X_no_comp(iVrelLong);

  // 判定目标是否处于车辆正前方【-1，1】
  bool front_at_vehicle_ = true;
  if(((trace.kf_info.X_no_comp(iDistLat) > 0.0) && ((trace.kf_info.X_no_comp(iDistLat) - (0.5 * trace.wid)) > 1.0)) ||
      ((trace.kf_info.X_no_comp(iDistLat) < 0.0) && ((trace.kf_info.X_no_comp(iDistLat) + (0.5 * trace.wid)) < -1.0)))
  {
    front_at_vehicle_ = false;
  }

  if(!front_at_vehicle_) //目标位于车辆两侧
  {
      //TODO: 超过45°后，是否考虑不再更新
      // 超过45后，目前下边缘不可见，因此将Y基础更改为目标上边缘
      if (fabs(angle_max) > (40.0 * DEG2RAD)) {

        if(trace.kf_info.X_no_comp(iDistLat) > 0.0)
        {
            preZ(0) = trace.kf_info.X_no_comp(iDistLat) - 0.5 * trace.wid;
            measZ(0) = new_box.x - 0.5 * new_box.wid;
        }
        else
        {
            preZ(0) = trace.kf_info.X_no_comp(iDistLat) + 0.5 * trace.wid;
            measZ(0) = new_box.x + 0.5 * new_box.wid;
        }

        preZ(1) = trace.kf_info.X_no_comp(iDistLong) * 0.7 + \
                    0.3 * (trace.kf_info.X_no_comp(iDistLong) + 0.5 * trace.len);
        measZ(1) = new_box.y * 0.7 + 0.3 * (new_box.y + 0.5 * new_box.len);

//        // 航迹中心点是否超过40度
//        double center_angle = (atan(trace.kf_info.X_no_comp(iDistLat) / \
//                              (trace.kf_info.X_no_comp(iDistLong) - 0.5 * trace.len - 2.4)));

//        if(fabs(center_angle) > (45.0 * DEG2RAD))
//        {
//            preZ(1) = trace.kf_info.X_no_comp(iDistLong) * 0.7 + 0.3 * (trace.kf_info.X_no_comp(iDistLong) + 0.5 * trace.len);
//            measZ(1) = new_box.y * 0.7 + 0.3 * (new_box.y + 0.5 * new_box.len);
//        }
      }
      else
      {
         // 目标位于左侧
        if(trace.kf_info.X_no_comp(iDistLat) > 0.0)
        {
            preZ << (trace.kf_info.X_no_comp(iDistLat) - 0.5 * trace.wid),
                (trace.kf_info.X_no_comp(iDistLong) - 0.5 * trace.len),
                temp_data / range_;

            measZ << (new_box.x - 0.5 * new_box.wid),
                    (new_box.y - 0.5 * new_box.len),
                    new_box.v;
        }
        else // 目标位于右侧
        {
            preZ << (trace.kf_info.X_no_comp(iDistLat) + 0.5 * trace.wid),
                (trace.kf_info.X_no_comp(iDistLong) - 0.5 * trace.len),
                temp_data / range_;

            measZ << (new_box.x + 0.5 * new_box.wid),
                     (new_box.y - 0.5 * new_box.len),
                     new_box.v;
        }
      }

  }
  else // 目标处于本车正前方
  {
      preZ << trace.kf_info.X_no_comp(iDistLat),
          (trace.kf_info.X_no_comp(iDistLong) - 0.5 * trace.len),
          temp_data / range_;

      measZ << new_box.x, (new_box.y - 0.5 * new_box.len), new_box.v;
  }
}

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @param {gridTrack_t} *track_list
 * @param {  } std
 * @param {  } std
 * @return {*}
 */
void creat_new_static_trace(gridTrack_t *track_list,
                            std::vector<det_box_t> &det_box_list,
                            std::vector<uint8_t> valid_empty_list) {
  static uint16_t static_real_id = 1000;


  Eigen::Matrix3d rotation_car;
  rotation_car << 0, vehicle_temp.yaw_rate, 0.0,
                  -1.0 * vehicle_temp.yaw_rate, 0, 0.0,
                    0.0,0.0, 0;

  Eigen::Vector3d local_loc;//(trace_info(0), trace_info(1), trace_info(2));
  Eigen::Vector3d angular_trans_speed;

  // 未被关联的box生成新航迹
  uint16_t newObjnum = 0;

  if (valid_empty_list.size() > 0) {
    for (auto &det_box : det_box_list) {
      if (det_box.box_feature == DYNAMIC_BOX) {
        continue;
      }

      if ((det_box.valid & (1 << NOT_USED_FOR_INIT)) ||
          (det_box.valid & (1 << HAS_MATCHED))) {
        continue;
      }

      if((det_box.wid < 0.8) && (det_box.len < 0.8))
      {
        continue;
      }

      gridTrack_t &subTrace = track_list[valid_empty_list.at(newObjnum)];
      subTrace.ID = static_real_id;
      subTrace.age = 1;
      subTrace.status = UNACTIVATE;
      subTrace.NMLogic = 1;
      subTrace.UnAssoNum = 0;
      subTrace.dyn_num = 0;

      local_loc << det_box.y_pos, det_box.x_pos, 0.0;
      angular_trans_speed = rotation_car * local_loc;

      subTrace.kf_info.X_ << det_box.x_pos, det_box.y_pos, angular_trans_speed(1),
                            det_box.v_mean + angular_trans_speed(0),
                            0.0, 0.0;

      subTrace.kf_info.P_ = MatrixXd::Identity(6, 6) * 0.1;

      subTrace.wid = (det_box.wid > 1.0) ? det_box.wid : 1.0;
      subTrace.len = (det_box.len > 1.0) ? det_box.len : 1.0;
      subTrace.height = 0.5;

      subTrace.ProbOfExist = 0.3;
      subTrace.max_prob = 0.3;
      subTrace.Prob_Output = subTrace.ProbOfExist / TRACE_PROB_LIMIT;
      newObjnum++;

      static_real_id++;

      if (static_real_id > 2000) {
        static_real_id = 1000;
      }

      if (newObjnum >= valid_empty_list.size()) {
        break;
      }
    }
  }
}

/**
 * @name: kf_predict
 * @description: 卡尔曼预测
 * @param {VectorXd} &X
 * @param {MatrixXd} &P
 * @return {*}
 */
void kf_predict(Eigen::VectorXd &X, Eigen::MatrixXd &P, Eigen::MatrixXd F,
                Eigen::MatrixXd F_T, Eigen::MatrixXd Q_) {
  X = F * X;
  P = F * P * F_T + Q_;
}

/**
 * @name: kf_update
 * @description: 卡尔曼更新
 * @param {VectorXd} &X
 * @param {MatrixXd} &P
 * @param {VectorXd} meas
 * @param {double} weight
 * @param {gridTrack_t} &trace
 * @return {*}
 */
double kf_update(Eigen::VectorXd &X, Eigen::MatrixXd &P, Eigen::VectorXd preZ,
                 Eigen::VectorXd measZ, double weight, gridTrack_t &trace) {
  Eigen::MatrixXd R = Eigen::MatrixXd::Zero(3, 3);

#ifdef ARS548_RADAR
  R(0, 0) = pow(0.2 * 2.0, 2.0);
  R(1, 1) = pow(0.2 * 2.0, 2.0);
  R(2, 2) = pow(0.2 * 1.0, 2.0);
#else
  R(0, 0) = pow(0.2 * 3.0, 2.0);
  R(1, 1) = pow(0.2 * 2.0, 2.0);
  R(2, 2) = pow(0.25 * 2.0, 2.0);
#endif

  Eigen::MatrixXd H_JacMat = compute_jacMat(trace);
  Eigen::MatrixXd H_JacMat_T = H_JacMat.transpose();

  double max_nagle = compute_max_angle(trace.kf_info.X_no_comp(iDistLat),
                                       trace.kf_info.X_no_comp(iDistLong),
                                       trace.len, trace.wid);

  if (fabs(max_nagle) > (45.0 * DEG2RAD)) {
    R(2, 2) = pow(5, 2);
  } else {
    // 根据距离调整多普勒噪声
    double range_ = sqrt(pow(X(iDistLat), 2.0) + pow(X(iDistLong), 2.0));
    if (range_ < 20.0) {
      R(2, 2) = pow((0.5 + (20.0 - range_) * 0.1), 2.0);
    }
  }

  Eigen::VectorXd diff_ = measZ - preZ;

//     TODO: 横纵向差值若大于阈值，则认为关联有误，不进行update
//    if ((fabs(diff_(0)) > 2.0) || (fabs(diff_(1)) > 3.0)) {
//      return 0.0;
//    }

//    if((fabs(diff_(0)) > 2.0) || \
//       ((diff_(1) > 4.0) || (diff_(1) < -4.0)))
//    {
//        return 0.0;
//    }

  trace.diff_X *= 0.9;
  trace.diff_Y *= 0.95;
  trace.diff_V *= 0.95;

//  if (high_motor_driven > 0) {
////    diff_(0) = Valuelimit(-1.0, 1.0, diff_(0));
//    diff_(1) = Valuelimit(-1.5, 0.5, diff_(1));
//    diff_(2) = Valuelimit(-1.0, 1.0, diff_(2));
//  } else {
//    diff_(0) = Valuelimit(-1.0, 1.0, diff_(0));
//    diff_(1) = Valuelimit(-1.5, 0.5, diff_(1));
//    diff_(2) = Valuelimit(-1.0, 1.0, diff_(2));
//  }

  if((fabs(diff_(0)) > 2.0) || \
     (fabs(diff_(1)) > 4.0))
  {
      return 0.0;
  }

  diff_(0) = Valuelimit(-1.0, 1.0, diff_(0));
  diff_(1) = Valuelimit(-1.5, 0.5, diff_(1));
  diff_(2) = Valuelimit(-1.0, 1.0, diff_(2));

  if (fabs(diff_(iDistLat)) > 0.2) {
    trace.diff_X += diff_(0);
  }

  if (fabs(diff_(iDistLong)) > 0.4) {
    trace.diff_Y += diff_(1);
  }

  if (fabs(diff_(iVEL)) > 0.4) {
    trace.diff_V += diff_(2);
  }

  // 范围限定
  trace.diff_X = Valuelimit(-5.0, 5.0, trace.diff_X);
  trace.diff_Y = Valuelimit(-5.0, 5.0, trace.diff_Y);
  trace.diff_V = Valuelimit(-5.0, 5.0, trace.diff_V);

  Eigen::MatrixXd S = H_JacMat * P * H_JacMat_T + R;
  Eigen::MatrixXd S_inverse = S.inverse();
  Eigen::MatrixXd K = P * H_JacMat_T * S_inverse;
  Eigen::MatrixXd I_mat = Eigen::MatrixXd::Identity(6, 6);

  double likelihood = exp(-0.5 * (sqrt(diff_.transpose() * S_inverse * diff_)));

  //  Eigen::VectorXd add_val = weight * K * diff_;
  Eigen::VectorXd add_val = K * diff_;

  X = X + add_val;
  P = (I_mat - K * H_JacMat) * P;

#if 0
    std::cout << trace.ID << " diff_: " << diff_.transpose() << std::endl;
    std::cout << trace.ID << " K: " << K << std::endl;
    std::cout << trace.ID << " X: " << X.transpose() << std::endl;
    std::cout << trace.ID << " Age: " << trace.age << std::endl;
#endif

  return likelihood;
}

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @param {double} new_len
 * @param {double} new_wid
 * @param {double} new_theta
 * @param {gridTrack_t} &trace
 * @return {*}
 */
void extended_info_filter(double new_len, double new_wid, double new_theta,
                          gridTrack_t &trace) {
  double mea_val, vari_val;

  if (trace.shape_status.len_fifo.size() >= 15) {
    trace.shape_status.len_fifo.pop_front();
  }
  trace.shape_status.len_fifo.push_back(new_len);
  static_ComputeMeanAndVari(trace.shape_status.len_fifo, &mea_val, &vari_val);
  trace.shape_status.P_[0] = vari_val;

  if (trace.shape_status.wid_fifo.size() >= 15) {
    trace.shape_status.wid_fifo.pop_front();
  }
  trace.shape_status.wid_fifo.push_back(new_wid);
  static_ComputeMeanAndVari(trace.shape_status.wid_fifo, &mea_val, &vari_val);
  trace.shape_status.P_[1] = vari_val;

  if (trace.shape_status.theta_fifo.size() >= 15) {
    trace.shape_status.theta_fifo.pop_front();
  }
  trace.shape_status.theta_fifo.push_back(new_theta);

  for (uint8_t idx = 1; idx < trace.shape_status.theta_fifo.size(); idx++) {
    /* 约束diff-theta */
    if (fabs(trace.shape_status.theta_fifo.at(idx) -
             trace.shape_status.theta_fifo.at(idx - 1)) > (0.5 * RADAR_PI)) {
      if (trace.shape_status.theta_fifo.at(idx) > 0) {
        trace.shape_status.theta_fifo.at(idx) -= RADAR_PI;
      } else {
        trace.shape_status.theta_fifo.at(idx) += RADAR_PI;
      }
    }
  }
  static_ComputeMeanAndVari(trace.shape_status.theta_fifo, &mea_val, &vari_val);

  trace.shape_status.P_[3] = vari_val;
}

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @param {deque<double>} dataArray
 * @param {double} *mean_val
 * @param {double} *vari_val
 * @return {*}
 */
void static_ComputeMeanAndVari(const std::deque<double> dataArray,
                               double *mean_val, double *vari_val) {
  double tempMean = 0.0F;
  double tempVari = 0.0F;

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
 * @name: compute_jacMat
 * @description: 计算雅克比矩阵
 * @param {gridTrack_t} &trace
 * @return {*}
 */
Eigen::MatrixXd compute_jacMat(gridTrack_t &trace) {
  Eigen::MatrixXd H_JacMat = Eigen::MatrixXd::Zero(3, 6);

  double range_ =
      sqrt(pow(trace.kf_info.X_(0), 2.0) + pow(trace.kf_info.X_(1), 2.0));
  double range_3_2 = pow(range_, 3);

  double temp_data = trace.kf_info.X_(0) * trace.kf_info.X_(2) +
                     trace.kf_info.X_(1) * trace.kf_info.X_(3);

  double mat_20 = trace.kf_info.X_(2) / range_ -
                  trace.kf_info.X_(0) * temp_data / range_3_2;
  double mat_21 = trace.kf_info.X_(3) / range_ -
                  trace.kf_info.X_(1) * temp_data / range_3_2;
  double mat_22 = trace.kf_info.X_(0) / range_;
  double mat_23 = trace.kf_info.X_(1) / range_;

  H_JacMat(0, 0) = 1;
  H_JacMat(1, 1) = 1;
  H_JacMat(2, 0) = mat_20;
  H_JacMat(2, 1) = mat_21;
  H_JacMat(2, 2) = mat_22;
  H_JacMat(2, 3) = mat_23;

  return H_JacMat;
}

/**
 * @name: trace_abnormal_trace_delete
 * @description: 统计所有航迹横纵向速度信息，剔除异常航迹
 * @param {gridTrack_t} *trace_list
 * @return {*}
 */
void trace_abnormal_trace_delete(gridTrack_t *track_list) {
  // 统计所有确认航迹的横纵向速度
  std::vector<double> v_lat_list;
  std::vector<double> v_long_list;

  for (uint16_t i = 0; i < STATIC_TRACK_MAXNUM; i++) {
    gridTrack_t &subTrace = track_list[i];

    if (subTrace.status == ACTIVATE) {
      v_lat_list.push_back(subTrace.kf_info.X_(2));
      v_long_list.push_back(subTrace.kf_info.X_(3));
    }
  }

  if (v_lat_list.size() > 8) {
    std::sort(v_lat_list.begin(), v_lat_list.end(),
              [](double a, double b) { return b < a; });
    std::sort(v_long_list.begin(), v_long_list.end(),
              [](double a, double b) { return b < a; });

    // 计算速度中位数
    int mid_idx = (v_lat_list.size() % 2 == 0) ? (v_lat_list.size() / 2)
                                               : (v_lat_list.size() / 2 + 1);
    double mid_val_v_lat = v_lat_list.at(mid_idx);
    double mid_val_v_long = v_long_list.at(mid_idx);

    for (uint16_t i = 0; i < STATIC_TRACK_MAXNUM; i++) {
      gridTrack_t &subTrace = track_list[i];

      if (subTrace.status == ACTIVATE) {
        // TODO : HAS ERROR
        if ((fabs(subTrace.kf_info.X_(iVrelLat) - mid_val_v_lat) > 1.0) ||
            (fabs(subTrace.kf_info.X_(iVrelLong) - mid_val_v_long) > 1.0)) {
          subTrace.ProbOfExist *= 0.7;
        }
      }
    }
  }
}

/**
 * @name:
 * @description: 静态航迹与所有确认的运动航迹计算IOu，过大则删除对应静态航迹
 * @param {trackTable_strcut} *dyn_track_list
 * @param {gridTrack_t} *static_track_list
 * @return {*}
 */
void trace_compare_with_dyn_trace(trackTable_strcut *dyn_track_list,
                                  gridTrack_t *static_track_list) {
  trackTable_strcut *trace = nullptr;
  gridTrack_t *subTrace = nullptr;

  rect_point_struct r1, r2;
  for (uint16_t idx1 = 0; idx1 < STATIC_TRACK_MAXNUM; idx1++) {
    if (static_track_list[idx1].status == EMPTY) {
      continue;
    }
    subTrace = &static_track_list[idx1];

    creat_rect_box_point(subTrace->kf_info.X_(0), subTrace->kf_info.X_(1),
                         subTrace->len, subTrace->wid, 0.0, r1);

    for (uint16_t idx2 = 0; idx2 < MAXTRACKS; idx2++) {
      if (dyn_track_list[idx2].trackState == TRACK_STATE_ACTIVE) {
        trace = &dyn_track_list[idx2];

        /* 横纵向差距过大则不进行比较 */
        if ((fabs(subTrace->kf_info.X_(0) - trace->KalmanInfo.StateEst(0)) >
                 10.0 &&
             fabs(subTrace->kf_info.X_(1) - trace->KalmanInfo.StateEst(1)) >
                 10.0)) {
          // nothing to do
        } else {
          creat_rect_box_point(
              trace->KalmanInfo.StateEst(0), trace->KalmanInfo.StateEst(1),
              trace->ExtendInfo.Length, trace->ExtendInfo.Width,
              -1.0 * trace->ExtendInfo.box_theta, r2);

          double inter = intersection_area(r1, r2);

          double iou = inter / (calcularea(r1) + calcularea(r2) - inter);

          if (iou > 0.0) {
            /* 对于临时静态航迹，若iou>0，则直接删除
             * 对于确认静态航迹，若iou>0且相交面积/静态航迹面积 >
             * 0.5，则删除该航迹
             */
            if (static_track_list[idx1].status == UNACTIVATE) {
              subTrace->status = EMPTY;
#ifdef LOG_OUTPUT
              std::cout << "trace_compare_with_dyn_trace P1 Delete Trace ID: "
                        << subTrace->ID << std::endl;
#endif
            } else {
              if ((iou / calcularea(r1)) > 0.3) {
                subTrace->status = EMPTY;
#ifdef LOG_OUTPUT
                std::cout
                    << " trace_compare_with_dyn_trace P2 Delete Trace ID: "
                    << subTrace->ID << std::endl;
#endif
              }
            }
          }
        }
      }
    }
  }
}

// 计算box最近点坐标
/**
 * @name:
 * @description:
 * @param {double} x_pos
 * @param {double} y_pos
 * @param {double} len
 * @param {double} wid
 * @param {double} *new_x_pos
 * @param {double} *new_y_pos
 * @return {*}
 */
void compute_box_nearest_pos(const double x_pos, const double y_pos,
                             const double len, const double wid,
                             double *new_x_pos, double *new_y_pos) {
  *new_x_pos = x_pos;
  *new_y_pos = (y_pos - 0.5 * len);
}

/**
 * @name:
 * @description:
 * @param {double} x_pos
 * @param {double} y_pos
 * @param {double} len
 * @param {double} wid
 * @return {*}
 */
double compute_max_angle(const double x_pos, const double y_pos,
                         const double len, const double wid) {
//  double new_x_pos;

//  if (fabs(x_pos + 0.5 * wid) > fabs(x_pos - 0.5 * wid)) {
//    new_x_pos = (x_pos + 0.5 * wid);
//  } else {
//    new_x_pos = (x_pos - 0.5 * wid);
//  }

//  return (atan(new_x_pos / (y_pos - 0.5 * len - 2.4)));

  if((x_pos > 0.0) && ((x_pos - 0.5 * wid) > 0.0))
  {
    return (atan((x_pos - 0.5 * wid) / (y_pos - 0.5 * len - 2.4)));
  }
  else if((x_pos < 0.0) && ((x_pos + 0.5 * wid) < 0.0))
  {
    return (atan((x_pos + 0.5 * wid) / (y_pos - 0.5 * len - 2.4)));
  }
  else
  {
    return 0.0;
  }
}

/**
 * @name:
 * @description:
 * @param {double} x_pos
 * @param {double} y_pos
 * @param {double} len
 * @param {double} wid
 * @return {*}
 */
double compute_min_angle(const double x_pos, const double y_pos,
                         const double len, const double wid) {
  double new_x_pos;

  if (fabs(x_pos) > (0.5 * wid)) {
    if (x_pos > 0) {
      new_x_pos = fabs(x_pos - 0.5 * wid);
    } else {
      new_x_pos = fabs(x_pos + 0.5 * wid);
    }

    return (atan(new_x_pos / (y_pos + 0.5 * len)));
  } else {
    return 0.0;
  }
}

/**
 * @name:
 * @description:
 * @param {gridTrack_t} *track_list
 * @param {  } vehicleInfo_
 * @return {*}
 */
void trace_abnormal_trace_delete2(gridTrack_t *track_list,
                                  vehicleInfo_struct &vehicleInfo) {
  std::vector<std::pair<uint16_t, double>> vx_list, vy_list;
  for (uint16_t trk_idx = 0; trk_idx < STATIC_TRACK_MAXNUM; trk_idx++) {
    if (track_list[trk_idx].status < ACTIVATE) {
      continue;
    }

    std::pair<double, double> temp1, temp2;

    temp1.first = trk_idx;
    temp1.second = track_list[trk_idx].kf_info.X_(iVrelLat);

    temp2.first = trk_idx;
    temp2.second = track_list[trk_idx].kf_info.X_(iVrelLong);

    vx_list.push_back(temp1);
    vy_list.push_back(temp2);
  }

  if (vx_list.size() > 8) {
    std::sort(
        vx_list.begin(), vx_list.end(),
        [](std::pair<uint16_t, double> c1, std::pair<uint16_t, double> c2) {
          return (c1.second > c2.second);
        });

    std::sort(
        vy_list.begin(), vy_list.end(),
        [](std::pair<uint16_t, double> c1, std::pair<uint16_t, double> c2) {
          return (c1.second > c2.second);
        });

    uint8_t base_idx = vx_list.size() / 2.0;

    double mid_vx, mid_vy;

    mid_vx = (vx_list.at(base_idx - 1).second + vx_list.at(base_idx).second +
              vx_list.at(base_idx + 1).second) /
             3.0;
    mid_vy = (vy_list.at(base_idx - 1).second + vy_list.at(base_idx).second +
              vy_list.at(base_idx + 1).second) /
             3.0;

    for (uint8_t idx = 0; idx < vx_list.size(); idx++) {
      if (fabs(vehicleInfo.yaw_rate) < (0.1)) {
        if (fabs(vx_list.at(idx).second - mid_vx) > 1.5) {
          track_list[vx_list.at(idx).first].status = EMPTY;
#ifdef LOG_OUTPUT
          std::cout << " trace_abnormal_trace_delete2 P1 Delete Trace ID: "
                    << track_list[vy_list.at(idx).first].ID << std::endl;
#endif
        }

        /* 判定纵向速度差异
         *   1、与中值的差异
         *   2、与自车车速的差异
         */
        if ((fabs(vy_list.at(idx).second - mid_vy) > 2.0) &&
            (fabs(vy_list.at(idx).second + vehicleInfo.vx) > 2.0)) {
          track_list[vy_list.at(idx).first].ProbOfExist *= 0.7;

          if ((fabs(vy_list.at(idx).second + vehicleInfo.vx) > 3.0) ||
              ((vy_list.at(idx).second * vehicleInfo.vx) < 0.0)) {
            track_list[vy_list.at(idx).first].status = EMPTY;

#ifdef LOG_OUTPUT
            std::cout << "trace_abnormal_trace_delete2 P2 Delete Trace ID: "
                      << track_list[vy_list.at(idx).first].ID << std::endl;
#endif
          }
        }
      }
    }
  }
}

/**
 * @name:
 * @description:
 * @param {trackTable_strcut} *dyn_track_list
 * @param {gridTrack_t} *static_track_list
 * @param {  } vehicleInfo_
 * @return {*}
 */
void static_trace_to_dynamic(trackTable_strcut *dyn_track_list,
                             gridTrack_t *static_track_list,
                             vehicleInfo_struct &vehicleInfo) {
  trackTable_strcut *dynamic_trace = nullptr;
  gridTrack_t *static_trace = nullptr;
  bool find_pos = false;

  for (uint16_t idx1 = 0; idx1 < STATIC_TRACK_MAXNUM; idx1++) {
    static_trace = &static_track_list[idx1];

    if (static_trace->status == 0) {
      continue;
    }

    if (fabs(vehicleInfo.yaw_rate) > 0.05) {
      static_trace->dyn_num = 0;
    }

    if (static_trace->dyn_num < 10) {
      continue;
    }

    find_pos = false;
    for (uint16_t trk_idx = 0; trk_idx < MAXTRACKS; trk_idx++) {
      if (dyn_track_list[trk_idx].trackState == TRACK_STATE_FREE) {
        dynamic_trace = &dyn_track_list[trk_idx];
        find_pos = true;
        break;
      }
    }

    if (!find_pos) {
      return;
    }

    change_static2dynamic_trace(dynamic_trace, static_trace);

    static_trace->status = EMPTY;

#ifdef LOG_OUTPUT
    std::cout << " (Line 1418) Delete Trace ID: " << static_trace->ID
              << std::endl;
#endif
  }
}

/**
 * @name:
 * @description:
 * @param {trackTable_strcut} *trace
 * @param {gridTrack_t} *static_track_list
 * @return {*}
 */
void change_static2dynamic_trace(trackTable_strcut *trace,
                                 gridTrack_t *static_track_list) {
  trace->trackID = static_track_list->ID;
  trace->TCount = static_track_list->age;
  trace->birth_timestamp = static_track_list->birth_timestamp;
  trace->latest_tracked_time = static_track_list->latest_tracked_time;

  trace->trackState = (static_track_list->status == 1) ? TRACK_STATE_DETECTION
                                                       : TRACK_STATE_ACTIVE;

  trace->KalmanInfo.StateEst = static_track_list->kf_info.X_;
  trace->KalmanInfo.P_ = static_track_list->kf_info.P_;

  trace->ManageInfo.detect2TrackingCount = 0;
  trace->ManageInfo.HighProCount = 0U;
  trace->ManageInfo.NM_SUM = 10;
  trace->ManageInfo.MN_Logic_1 = 1;
  trace->ManageInfo.MN_Logic_2 = 1;

  trace->ExtendInfo.Length = static_track_list->len;
  trace->ExtendInfo.Width = static_track_list->wid;
  trace->ExtendInfo.box_theta = 0.0;
  trace->ExtendInfo.Orin_For_Display = 0.0;
  trace->ExtendInfo.ProbOfExist = static_track_list->ProbOfExist;
  trace->ExtendInfo.Object_Class = UNKNOWN;
  trace->ExtendInfo.DynProp = stationary;
  trace->ExtendInfo.UpCount = 0;
  trace->ExtendInfo.DownCount = 0;

  trace->MeasInfo.associateNum = 1;
  trace->MeasInfo.associateNumThr = 1;
  trace->MeasInfo.staticNum = 0;
  trace->MeasInfo.sum_weight = 0.0F;
  trace->MeasInfo.dopplerVari = 0.0F;
  trace->MeasInfo.x_diff_dir = 0.0;
  trace->MeasInfo.y_diff_dir = 0.0;
  trace->MeasInfo.max_rcs_filter_val = 0.0;

  Track_Init_Corners(trace, false);
}
