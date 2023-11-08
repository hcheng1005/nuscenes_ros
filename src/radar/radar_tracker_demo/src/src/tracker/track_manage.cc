/*
 * @description:
 * @version:
 * @Author: ChengHao
 * @Date: 2022-10-08 14:04:46
 * @LastEditors: ChengHao hao.cheng@wuzheng.com
 * @LastEditTime: 2023-11-03 16:30:35
 */

#include "commonfunctions.h"
#include "track_manage.h"

extern std::deque<static_point_snapshot_t> snapshot_fifo;

extern double RADAR_MOUNT_X;

/**
 * @name: track_Manage
 * @description: 航迹管理函数入口
 * @param {trackTable_strcut} *trackInfo
 * @return {*}
 */
void track_Manage(trackTable_strcut *trackInfo,
                  vehicleInfo_struct *vehicleInfo) {
  track_associate_PointThr(trackInfo);

  for (uint16_t track_i = 0; track_i < MAXTRACKS; track_i++) {
    if (trackInfo[track_i].trackState == TRACK_STATE_FREE) {
      continue;
    }

    // 航迹管理：确认航迹
    if (trackInfo[track_i].trackState == TRACK_STATE_ACTIVE) {
      track_manage_confirmedtrace(&trackInfo[track_i]);
    }

    // 航迹管理：临时航迹
    if (trackInfo[track_i].trackState == TRACK_STATE_DETECTION) {
      track_manage_unconfirmedtrace(&trackInfo[track_i], vehicleInfo);
    }
  }

  // 航迹修剪与合并
  track_merge2(trackInfo);
}

/**
 * @name: track_associate_PointThr
 * @description: 航迹关联个数阈值设定
 * @param {trackTable_strcut} *trackInfo
 * @return {*}
 */
void track_associate_PointThr(trackTable_strcut *trackInfo) {
  VectorXd StatePolar = VectorXd(MSIZE);
  trackTable_strcut *trace = nullptr;

  double range_, azi_, vel_;

  for (uint16_t track_i = 0; track_i < MAXTRACKS; track_i++) {
    trace = &trackInfo[track_i];

    range_ = sqrt(pow(trace->KalmanInfo.StateEst(iDistLat), 2.0) +
                  pow(trace->KalmanInfo.StateEst(iDistLong), 2.0));
    azi_ = ComputeAngle(trace->KalmanInfo.StateEst(iDistLat),
                        trace->KalmanInfo.StateEst(iDistLong));
    vel_ = (trace->KalmanInfo.StateEst(iDistLat) *
                trace->KalmanInfo.StateEst(iVrelLat) +
            trace->KalmanInfo.StateEst(iDistLong) *
                trace->KalmanInfo.StateEst(iVrelLong)) /
           range_;

    StatePolar << range_, azi_, vel_;

    if (trackInfo[track_i].trackState == TRACK_STATE_ACTIVE) {
      trackInfo[track_i].MeasInfo.associateNumThr = 1;

      if ((StatePolar(iRANGE) < 30.0F) &&
          (trackInfo[track_i].ExtendInfo.Object_Class >= VEHICLE)) {
        trackInfo[track_i].MeasInfo.associateNumThr = 2;
      }
    } else if (trackInfo[track_i].trackState == TRACK_STATE_DETECTION) {
      trackInfo[track_i].MeasInfo.associateNumThr = 1;
      if (StatePolar(iRANGE) < 30.0F) {
        if (StatePolar(iAZI) < (50.0 / 180 * 3.14)) {
          trackInfo[track_i].MeasInfo.associateNumThr = 2;
        }
      }
    }
  }
}

/**
 * @name: track_manage_confirmedtrace
 * @description: 航迹管理逻辑-确认航迹
 * @param {trackTable_strcut} *trace
 * @return {*}
 */
void track_manage_confirmedtrace(trackTable_strcut *trace) {
  trace->TCount++;

  if (trace->MeasInfo.associateNum == 0U) {
    trace->ManageInfo.tracking2missCount++;
  } else {
    trace->ManageInfo.tracking2missCount = 0U;
  }

  if (trace->ExtendInfo.Object_Class >= VEHICLE) {
    if ((trace->ExtendInfo.ProbOfExist <
         (TRACE_PROB_LIMIT * TRACE_DELETE_PERCENT)) ||
        (trace->ManageInfo.tracking2missCount >= 8)) {
      track_clear(trace);
    }
  } else {
    if ((trace->ExtendInfo.ProbOfExist <
         (TRACE_PROB_LIMIT * (TRACE_DELETE_PERCENT - 0.05))) ||
        (trace->ManageInfo.tracking2missCount >= 15)) {
      track_clear(trace);
    }
  }
}

/**
 * @name: track_manage_unconfirmedtrace
 * @description: 航迹管理逻辑-临时航迹
 * @param {trackTable_strcut} *trackInfo
 * @param {trackTable_strcut} *trace
 * @return {*}
 */
void track_manage_unconfirmedtrace(trackTable_strcut *trace,
                                   vehicleInfo_struct *vehicleInfo) {
  // NM滑窗逻辑
  trace->TCount++;
  trace->ManageInfo.MN_Logic_1 =
      (trace->ManageInfo.MN_Logic_1 << 1U) & 0xFFFFFFFF;
  trace->ManageInfo.MN_Logic_2 =
      (trace->ManageInfo.MN_Logic_2 << 1U) & 0xFFFFFFFF;

  // 本次关联是否有高质量点
  if (trace->MeasInfo.associateNum > 0U) {
    trace->ManageInfo.detect2TrackingCount++;
    trace->ManageInfo.detect2missCount = 0;
    trace->ManageInfo.MN_Logic_1 |= 1;
  } else {
    trace->ManageInfo.detect2missCount++;
    trace->ManageInfo.detect2TrackingCount = 0U;

    if (trace->ManageInfo.detect2missCount >= 3) {
      track_clear(trace);
      return;
    }
  }

  // 本次是否关联到量测点
  if (trace->MeasInfo.associateNum > 0) {
    trace->ManageInfo.MN_Logic_2 |= 1;
  }

  // 提前删除航迹策略
  if ((trace->ExtendInfo.ProbOfExist < 0.3F) ||
      ((trace->ManageInfo.detect2missCount >= 2) &&
       (trace->TCount <= 5)))  // 提前删除逻辑
  {
    track_clear(trace);
    return;
  }

  uint8_t high_num = 5;
  double scale = 1.0 - trace->ManageInfo.trace_belief;

  double trace_size = trace->ExtendInfo.Length * trace->ExtendInfo.Width;
  if (((trace_size < 2.0) && (trace->ExtendInfo.ProbOfExist >
                              (TRACE_PROB_LIMIT * TRACE_ACTIVE_SAMLL_TAR))) ||
      (trace->ExtendInfo.ProbOfExist >
       (TRACE_PROB_LIMIT * TRACE_ACTIVE_PERCENT * scale))) {
    trace->ManageInfo.HighProCount++;
  } else {
    if (trace->ManageInfo.HighProCount > 0) {
      trace->ManageInfo.HighProCount--;
    }
  }

  uint8_t NM1, NM2;

  trace->ManageInfo.NM_SUM = 10;
  NM1 = 7;
  NM2 = 7;

  // 计算MN数值
  uint8_t nm[2] = {0U, 0U};
  uint16_t scanNum = (trace->TCount > trace->ManageInfo.NM_SUM)
                         ? trace->ManageInfo.NM_SUM
                         : trace->TCount;

  for (uint8_t idx = 0; idx < scanNum; idx++) {
    if ((trace->ManageInfo.MN_Logic_1 & (1U << idx)) == (1U << idx)) {
      nm[0]++;
    }

    if ((trace->ManageInfo.MN_Logic_2 & (1U << idx)) == (1U << idx)) {
      nm[1]++;
    }
  }

  if (((trace_size < 0.64) &&
       (trace->ExtendInfo.ProbOfExist >
        (TRACE_PROB_LIMIT * TRACE_FINAL_CONFIRM_PERCENT_EX_SMALL))) ||
      (trace->ExtendInfo.ProbOfExist >
       (TRACE_PROB_LIMIT * TRACE_FINAL_CONFIRM_PERCENT_BIG * scale))) {
    if ((trace->ManageInfo.HighProCount >= high_num) && (nm[0] >= NM2) &&
        (nm[1] >= NM1)) {
      trace->trackState = TRACK_STATE_ACTIVE;
      trace->ManageInfo.tracking2missCount = 0U;
      trace->ManageInfo.detect2missCount = 0U;
    }
  }

  // 临时航迹删除逻辑
  if ((trace->TCount > 10) &&
      (trace->ExtendInfo.ProbOfExist < (TRACE_PROB_LIMIT * 0.5))) {
    track_clear(trace);
  }

  if ((trace->TCount > 20) &&
      (trace->ExtendInfo.ProbOfExist < (TRACE_PROB_LIMIT * 0.7))) {
    track_clear(trace);
  }

  // 删除特定区域未起始的航迹
  if ((fabs(atan2(trace->KalmanInfo.MeasPre(0),
                  trace->KalmanInfo.MeasPre(1) - 3.0)) > (M_PI_4))) {
    //
    if (trace->KalmanInfo.StateEst(iVrelLong) < -1.0) {
      track_clear(trace);
    }
  }
}

/**
 * @name: track_clear
 * @description: 清空航迹/删除航迹
 * @param {trackTable_strcut} *trace
 * @return {*}
 */
void track_clear(trackTable_strcut *trace) {
  trace->trackState = TRACK_STATE_FREE;
  trace->KalmanInfo.P_ = MatrixXd::Identity(SSIZE, SSIZE);

  trace->trackID = 0;
  trace->birth_timestamp = 0;
  trace->latest_tracked_time = 0;
  trace->ManageInfo.detect2TrackingCount = 0;
  trace->ManageInfo.detect2missCount = 0;
  trace->ManageInfo.tracking2missCount = 0;
  trace->MeasInfo.associateNum = 0;
  trace->MeasInfo.associateNumThr = 0;
  trace->TCount = 0;

  memset(&trace->ExtendInfo, 0, sizeof(ExtendInfo_struct) * 1);
}

/**
 * @name: CompareWithConfirmTrk
 * @description: 新生航迹与确认航迹比较，决定其是否正常起始
 * @param {trackTable_strcut} *trackInfo
 * @param {trackTable_strcut} *trace
 * @return {*}
 */
bool CompareWithConfirmTrk(trackTable_strcut *trackInfo,
                           trackTable_strcut *trace) {
  bool fakeTrace = false;
  float Error_Lat, Error_Long;

  trackTable_strcut *confirmTrkInfo = nullptr;

  for (uint16_t idx = 0; idx < MAXTRACKS; idx++) {
    confirmTrkInfo = &trackInfo[idx];

    if (confirmTrkInfo->trackState != TRACK_STATE_ACTIVE) {
      continue;
    }

    // step 1:是否重叠
    Error_Lat = trace->KalmanInfo.StateEst(iDistLat) -
                confirmTrkInfo->KalmanInfo.StateEst(iDistLat);
    Error_Long = trace->KalmanInfo.StateEst(iDistLong) -
                 confirmTrkInfo->KalmanInfo.StateEst(iDistLong);

    Error_Lat =
        fabs(Error_Lat) -
        ((trace->ExtendInfo.Width + confirmTrkInfo->ExtendInfo.Width) * 0.5F);
    Error_Long =
        fabs(Error_Long) -
        ((trace->ExtendInfo.Length + confirmTrkInfo->ExtendInfo.Length) * 0.5F);

    if ((Error_Lat <= 0.4F) && (Error_Long <= 0.8F))  // overlap
    {
      fakeTrace = true;
      break;
    }

    if ((Error_Lat <= 1.0F) && (Error_Long <= 2.0F) &&
        (confirmTrkInfo->ExtendInfo.Object_Class >= VEHICLE)) {
      fakeTrace = true;
      break;
    }

    if (Error_Lat > 0.0F) {
      Error_Lat = trace->KalmanInfo.StateEst(iDistLat) +
                  trace->KalmanInfo.StateEst(iVrelLat) * 2.0F -
                  (confirmTrkInfo->KalmanInfo.StateEst(iDistLat) +
                   confirmTrkInfo->KalmanInfo.StateEst(iVrelLat) * 2.0F);

      Error_Lat =
          fabs(Error_Lat) -
          ((trace->ExtendInfo.Width + confirmTrkInfo->ExtendInfo.Width) * 0.5F);
    }
    if (Error_Long > 0.0F) {
      Error_Long = trace->KalmanInfo.StateEst(iDistLong) +
                   trace->KalmanInfo.StateEst(iVrelLong) * 2.0F -
                   (confirmTrkInfo->KalmanInfo.StateEst(iDistLong) +
                    confirmTrkInfo->KalmanInfo.StateEst(iVrelLong) * 2.0F);

      Error_Long =
          fabs(Error_Long) -
          ((trace->ExtendInfo.Length + confirmTrkInfo->ExtendInfo.Length) *
           0.5F);
    }

    // step2：碰撞预测
    if ((Error_Lat <= 0.1F) && (Error_Long <= 0.2F))  // overlap
    {
      fakeTrace = true;
      break;
    }

    // has a "crossing move trace"
    if ((fabs(confirmTrkInfo->ExtendInfo.box_theta) > (80.0 / 180.0 * 3.14)) &&
        (fabs(confirmTrkInfo->ExtendInfo.box_theta) < (100.0 / 180.0 * 3.14)) &&
        (confirmTrkInfo->KalmanInfo.StateEst(iDistLong) < 10.0F) &&
        (fabs(confirmTrkInfo->KalmanInfo.StateEst(iDistLat) < 5.0F))) {
      Error_Lat = trace->KalmanInfo.StateEst(iDistLat) -
                  confirmTrkInfo->KalmanInfo.StateEst(iDistLat);

      Error_Lat = Error_Lat - confirmTrkInfo->ExtendInfo.Length * 0.5F - 1.0F;

      if (Error_Lat <= 0.0F) {
        fakeTrace = true;
        break;
      }
    }
  }

  return fakeTrace;
}

/**
 * @name: track_birth
 * @description: 新航迹起始逻辑
 * @param {RadarOutput_Struct} *indata
 * @param {trackTable_strcut} *trackInfo
 * @param {Tracker} *Tracker_
 * @return {*}
 */
void track_birth(std::vector<RadarMeasure_struct> &global_point,
                 trackTable_strcut *trackInfo,
                 vehicleInfo_struct *vehicleInfo) {
  uint16_t trackId = 0;
  fitInfo_struct fitInfo;

  std::vector<DBSCAN::Point4DBSCAN> PointSet;
  std::vector<std::vector<uint16_t>> clusterSet;

  BulidPointSet4DBSCAN(global_point, &PointSet);

  DBSCAN::KNN_DBSCAN(PointSet, clusterSet);

  std::cout << "New cluster size: " << clusterSet.size() << std::endl;

  for (auto &clusterPtr : clusterSet) {
    trackId = track_returnNewTrkId(trackInfo);

    if (trackId == 255) {
      return;
    }

    // 起始新航迹
    if (track_fittingCluster(global_point, PointSet, clusterPtr, &fitInfo)) {
      track_InitTrack(trackInfo, trackId, &fitInfo, global_point);
    }
  }
}

/**
 * @name: track_fittingCluster
 * @description: 量测拟合函数
 * @param {RadarOutput_Struct} *indata
 * @param {  } std
 * @param {  } std
 * @param {  } fitInfo_
 * @return {*}
 */
bool track_fittingCluster(std::vector<RadarMeasure_struct> &global_point,
                          std::vector<DBSCAN::Point4DBSCAN> &PointSet,
                          std::vector<uint16_t> &clusterPtr,
                          fitInfo_struct *fitInfo) {
  RadarMeasure_struct *detPtr = nullptr;
  VectorXd mCenter = VectorXd(MSIZE);
  uint16_t ValidNum = 0;

  fitInfo->clutter_num = 0;
  fitInfo->measIdx.clear();

  std::vector<double> dis_lat_list, dis_long_list, vre_list, rcs_list;

  for (uint16_t idx = 0; idx < clusterPtr.size(); idx++) {
    uint16_t n1 = PointSet[clusterPtr[idx]].PointInfo.ID;

    detPtr = &global_point[n1];

    if (detPtr->ProbOfExist < 0.1)  //噪点clutter
    {
      fitInfo->clutter_num++;
    }

    rcs_list.push_back(detPtr->RCS);

    fitInfo->measIdx.push_back(n1);

    detPtr->associateTrackID = MEASURE_NOT_ASSOCIATED + 1;

    ValidNum++;

    dis_lat_list.push_back(detPtr->DistLat);
    dis_long_list.push_back(detPtr->DistLong);
    vre_list.push_back(detPtr->VrelLong);
  }

  fitInfo->validNum = ValidNum;

  double sum_v = 0.0;
  for (const auto v_temp : vre_list) {
    sum_v += v_temp;
  }

  sum_v = sum_v / vre_list.size();

  auto lat_maxmin_val =
      std::minmax_element(dis_lat_list.begin(), dis_lat_list.end());
  auto long_maxmin_val =
      std::minmax_element(dis_long_list.begin(), dis_long_list.end());

  mCenter << (0.5 * (*lat_maxmin_val.second + *lat_maxmin_val.first)),
      0.5 * (*long_maxmin_val.second + *long_maxmin_val.first), sum_v;

  fitInfo->mCenter = mCenter;

  if (((fitInfo->clutter_num / clusterPtr.size()) > (0.8)) &&
      (fabs(mCenter(0)) > 20.0)) {
    return false;
  }

  if ((rcs_list.size() == 1) && (rcs_list.at(0) < -1.0)) {
    return false;
  }

  if ((dis_lat_list.size() == 1) && (fabs(dis_lat_list.at(0)) > 15.0)) {
    return false;
  }

  fitInfo->fit_Len = *long_maxmin_val.second - *long_maxmin_val.first;
  fitInfo->fit_Wid = *lat_maxmin_val.second - *lat_maxmin_val.first;

  if (fitInfo->fit_Len < 0.5) {
    fitInfo->fit_Len = 0.5;
  }

  if (fitInfo->fit_Wid < 0.5) {
    fitInfo->fit_Wid = 0.5;
  }

  return true;
}

/**
 * @name: BulidPointSet4DBSCAN
 * @description: 运动点迹聚类
 * @param {RadarOutput_Struct} *indata
 * @param {  } std
 * @return {*}
 */
void BulidPointSet4DBSCAN(std::vector<RadarMeasure_struct> &global_point,
                          std::vector<DBSCAN::Point4DBSCAN> *PointSet) {
  DBSCAN::Point4DBSCAN Point;

  PointSet->clear();

  for (uint16_t n = 0; n < global_point.size(); n++) {
    if ((global_point[n].invalid & (1U << notUed4Init_det)) ||
        (global_point[n].invalid & (1U << invalid_det)) ||
        (global_point[n].DynProp == stationary) ||
        (global_point[n].DynProp == unknown)) {
      continue;
    }

    Point.PointInfo.ID = n;
    Point.PointInfo.DistLat = global_point[n].DistLat;
    Point.PointInfo.DistLong = global_point[n].DistLong;
    Point.PointInfo.RCS = global_point[n].RCS;
    Point.PointInfo.V = global_point[n].VrelLong;
    Point.PointInfo.DynProp = global_point[n].DynProp;
    Point.PointInfo.valid = true;

    Point.PointInfo.Range = sqrt(pow(Point.PointInfo.DistLat, 2.0) +
                                 pow(Point.PointInfo.DistLong, 2.0));
    Point.PointInfo.Azi =
        atan2(Point.PointInfo.DistLat, Point.PointInfo.DistLong) / 3.14F *
        180.0F;
    Point.DBSCAN_para.Search_R = 1.5F + Point.PointInfo.Range * 0.02F;
    Point.DBSCAN_para.minPts = 1;
    Point.DBSCAN_para.pointType = 255;
    Point.DBSCAN_para.static_or_dyna = 0;

    PointSet->push_back(Point);
  }
}

/**
 * @name: track_returnNewTrkId
 * @description: 返回可用航迹索引号
 * @param {trackTable_strcut} *trackInfo
 * @return {*}
 */
uint8_t track_returnNewTrkId(trackTable_strcut *trackInfo) {
  uint8_t trackId = 255;

  for (uint16_t track_i = 0; track_i < MAXTRACKS; track_i++) {
    if (trackInfo[track_i].trackState == TRACK_STATE_FREE) {
      trackInfo[track_i].MeasInfo.detInfo.clear();
      trackId = track_i;
      break;
    }
  }
  return trackId;
}

/**
 * @name: track_InitTrack
 * @description: 航迹初始化
 * @param {trackTable_strcut} *trackInfo
 * @param {uint16_t} trackId
 * @param {  } fitInfo_
 * @param {RadarOutput_Struct} *indata
 * @param {Tracker} *Tracker_
 * @return {*}
 */
void track_InitTrack(trackTable_strcut *trackInfo, uint16_t valid_trace_idx,
                     fitInfo_struct *fitInfo,
                     std::vector<RadarMeasure_struct> &global_point) {
  static uint16_t trace_real_id = 0;

  uint8_t minNumber = 1;

  trackTable_strcut *trace = &trackInfo[valid_trace_idx];

  if (fitInfo->validNum >= minNumber) {
    trace->trackID = trace_real_id;
    trace->trackState = TRACK_STATE_DETECTION;

    double azi_ = atan2(fitInfo->mCenter(0), fitInfo->mCenter(1));

    trace->KalmanInfo.StateEst.fill(0);
    trace->KalmanInfo.StateEst(iDistLat) = fitInfo->mCenter(0);
    trace->KalmanInfo.StateEst(iDistLong) = fitInfo->mCenter(1);

    if (fitInfo->mCenter(2) > 0)  // 远离的目标
    {
      trace->KalmanInfo.StateEst(iVrelLat) = 0.0;
      trace->KalmanInfo.StateEst(iVrelLong) = fitInfo->mCenter(2);
    } else {
      trace->KalmanInfo.StateEst(iVrelLat) = fitInfo->mCenter(2) * sin(azi_);
      trace->KalmanInfo.StateEst(iVrelLong) = fitInfo->mCenter(2) * cos(azi_);
    }

    memset(&trace->ExtendInfo, 0, sizeof(ExtendInfo_struct) * 1);

    if (fitInfo->fit_Len < fitInfo->fit_Wid) {
      trace->ExtendInfo.Length = fitInfo->fit_Wid;
      trace->ExtendInfo.Width = fitInfo->fit_Len;
    } else {
      trace->ExtendInfo.Length = fitInfo->fit_Len;
      trace->ExtendInfo.Width = fitInfo->fit_Wid;
    }

    trace->ExtendInfo.max_len = trace->ExtendInfo.Length;
    trace->ExtendInfo.max_wid = trace->ExtendInfo.Width;

    // 判定是否为虚假航迹
    bool birthFlag = track_IsFakeTrace(trackInfo, trace);

    if (birthFlag == true) {
      track_clear(trace);
    } else {
      birthFlag = true;

      trace->MeasInfo.detInfo.clear();

      for (auto &idx : fitInfo->measIdx) {
        matchInfo_t temp;
        temp.cluster_Idx = idx;
        temp.weight = 1.0 / fitInfo->measIdx.size();
        temp.used4Fit = true;
        trace->MeasInfo.detInfo.push_back(temp);
      }

      if (birthFlag == false) {
        track_clear(trace);
      } else {
        trace->TCount = 1;
        trace->ManageInfo.detect2TrackingCount = 0;

        trace->KalmanInfo.P_ = MatrixXd::Identity(SSIZE, SSIZE) * 2.0F;

        trace->ExtendInfo.ProbOfExist = 0.3F;
        trace->ExtendInfo.Object_Class = UNKNOWN;
        trace->ExtendInfo.DynProp = unknown;
        trace->ExtendInfo.UpCount = 0;
        trace->ExtendInfo.DownCount = 0;

        trace->ManageInfo.HighProCount = 0U;

        trace->MeasInfo.associateNum = fitInfo->validNum;
        trace->MeasInfo.associateNumThr = minNumber;
        trace->MeasInfo.staticNum = 0;
        trace->MeasInfo.sum_weight = 0.0F;
        trace->MeasInfo.dopplerVari = 0.0F;

        trace->MeasInfo.x_diff_dir = 1.0;
        trace->MeasInfo.y_diff_dir = 1.0;
        trace->MeasInfo.max_rcs_filter_val = 0.0;

        trace->ManageInfo.NM_SUM = 10;
        trace->ManageInfo.MN_Logic_1 = 1;
        trace->ManageInfo.MN_Logic_2 = 1;
        trace->ManageInfo.trace_belief = 0.0;

        Track_Init_Corners(trace, false);

        trace_real_id++;
        if (trace_real_id > 1000) {
          trace_real_id = 0;
        }
      }
    }
  }
}

/**
 * @name: track_IsFakeTrace
 * @description: 虚假航迹判定逻辑
 * @param {trackTable_strcut} *trackInfo
 * @param {trackTable_strcut} *trace
 * @return {*}
 */
bool track_IsFakeTrace(trackTable_strcut *trackInfo, trackTable_strcut *trace) {
  bool fakeTrace = false;
  bool fakeTrace1 = false;

  // step 1: compare with old track
  fakeTrace = CompareWithConfirmTrk(trackInfo, trace);

  // step 2: FakeTrace by error speed
  //  fakeTrace1 = compute_multi_bounce(trackInfo, trace);

  return (fakeTrace | fakeTrace1);
}

/**
 * @name: Track_Init_Corners
 * @description: 目标八个角点坐标初始化
 * @param {trackTable_strcut} *trace
 * @param {bool} extendFlag
 * @return {*}
 */
void Track_Init_Corners(trackTable_strcut *trace, bool extendFlag) {
  float half_Width = trace->ExtendInfo.Width * 0.5F;
  float half_length = trace->ExtendInfo.Length * 0.5F;

  if (extendFlag == true) {
    half_Width *= 1.2F;

    if (trace->trackState == TRACK_STATE_DETECTION) {
      half_length *= 1.4F;
    }

    ComputeCornerPos(trace->KalmanInfo.StateEst(iDistLat),
                     trace->KalmanInfo.StateEst(iDistLong),
                     trace->ExtendInfo.box_theta * 0.0F, half_Width,
                     half_length, &trace->ExtendInfo.CornerPos[0][0]);
  } else {
    ComputeCornerPos(trace->KalmanInfo.StateEst(iDistLat),
                     trace->KalmanInfo.StateEst(iDistLong),
                     trace->ExtendInfo.box_theta, half_Width, half_length,
                     &trace->ExtendInfo.CornerPos[0][0]);
  }
}

/**
 * @name: track_merge2
 * @description: 航迹合并、删除逻辑
 * @param {trackTable_strcut} *trackInfo
 * @return {*}
 */
void track_merge2(trackTable_strcut *trackInfo) {
  trackTable_strcut *trace_old = nullptr;
  trackTable_strcut *trace_young = nullptr;

  double trace_s1, trace_s2;

  for (uint16_t idx1 = 0; idx1 < MAXTRACKS; idx1++) {
    if (trackInfo[idx1].trackState < TRACK_STATE_ACTIVE) {
      continue;
    }

    trace_s1 =
        trackInfo[idx1].ExtendInfo.Length * trackInfo[idx1].ExtendInfo.Width;

    for (uint16_t idx2 = 0; idx2 < MAXTRACKS; idx2++) {
      if ((trackInfo[idx2].trackState == TRACK_STATE_FREE) || (idx2 == idx1)) {
        continue;
      }

      trace_s2 =
          trackInfo[idx2].ExtendInfo.Length * trackInfo[idx2].ExtendInfo.Width;

      if (trackInfo[idx1].trackState > trackInfo[idx2].trackState) {
        trace_old = &trackInfo[idx1];
        trace_young = &trackInfo[idx2];
      } else {
        if (abs((int)trackInfo[idx1].TCount - (int)trackInfo[idx2].TCount) <
            5.0) {
          if ((trace_s1 > trace_s2)) {
            trace_old = &trackInfo[idx1];
            trace_young = &trackInfo[idx2];
          } else {
            trace_old = &trackInfo[idx2];
            trace_young = &trackInfo[idx1];
          }
        } else {
          if (trackInfo[idx1].TCount > trackInfo[idx2].TCount) {
            trace_old = &trackInfo[idx1];
            trace_young = &trackInfo[idx2];
          } else {
            trace_old = &trackInfo[idx2];
            trace_young = &trackInfo[idx1];
          }
        }
      }

      // 计算rotated box IOU
      rect_point_struct r1, r2;
      creat_rect_box_point(
          trace_old->KalmanInfo.StateEst(0), trace_old->KalmanInfo.StateEst(1),
          trace_old->ExtendInfo.Length, trace_old->ExtendInfo.Width,
          -1.0 * trace_old->ExtendInfo.box_theta, r1);

      creat_rect_box_point(trace_young->KalmanInfo.StateEst(0),
                           trace_young->KalmanInfo.StateEst(1),
                           trace_young->ExtendInfo.Length,
                           trace_young->ExtendInfo.Width,
                           -1.0 * trace_young->ExtendInfo.box_theta, r2);

      double iou_val = IOU_With_Rot(r1, r2);
      if (iou_val > 0.0) {
        //存在相交的情况下，进一步比较速度差
        if (((fabs(trace_old->KalmanInfo.StateEst(iVrelLat) -
                   trace_young->KalmanInfo.StateEst(iVrelLat)) < 2.0) &&
             (fabs(trace_old->KalmanInfo.StateEst(iVrelLong) -
                   trace_young->KalmanInfo.StateEst(iVrelLong)) < 1.0)) ||
            (iou_val > 0.3)) {
          //              trace_merge_newtrace(trace_old, trace_young);
          track_clear(trace_young);
        }
      }
    }
  }
}

/**
 * @name: trace_merge_newtrace
 * @description: 合并两条航迹信息
 * @param {trackTable_strcut} *trace_old
 * @param {trackTable_strcut} *trace_young
 * @return {*}
 */
void trace_merge_newtrace(trackTable_strcut *trace_old,
                          trackTable_strcut *trace_young) {
  VectorXd X_new = VectorXd(SSIZE);
  VectorXd diff_ = VectorXd(SSIZE);
  MatrixXd P_new = MatrixXd::Zero(SSIZE, SSIZE);

  double sumWeight, w1, w2;

  sumWeight =
      trace_old->ExtendInfo.ProbOfExist + trace_young->ExtendInfo.ProbOfExist;
  w1 = (double)(trace_old->ExtendInfo.ProbOfExist / sumWeight);
  w2 = (double)(trace_young->ExtendInfo.ProbOfExist / sumWeight);

  // TODO: 状态融合策略需要修改
  X_new = trace_old->KalmanInfo.StateEst * w1 +
          trace_young->KalmanInfo.StateEst * w2;

  diff_ = trace_old->KalmanInfo.StateEst - X_new;
  P_new += w1 * (trace_old->KalmanInfo.P_ + (diff_ * diff_.transpose()));

  diff_ = trace_young->KalmanInfo.StateEst - X_new;
  P_new += w2 * (trace_young->KalmanInfo.P_ + (diff_ * diff_.transpose()));

  // 状态融合
  trace_old->KalmanInfo.StateEst = X_new;
  trace_old->KalmanInfo.P_ = P_new;

  // TODO: 形状融合
}
