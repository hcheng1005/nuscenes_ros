/*
 * @Descripttion:
 * @version:
 * @Author: ChengHAO
 * @Date: 2022-09-05 18:11:33
 * @LastEditors: ChengHao hao.cheng@wuzheng.com
 * @LastEditTime: 2023-11-07 14:33:21
 */

#include "match.h"

#include "commonfunctions.h"
#include "common/DBSCAN.h"
#include "../../include/common/iou.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * @name: BulidPointSet4DBSCAN2
 * @description: 运动点迹聚类
 * @param {RadarOutput_Struct} *indata
 * @param {  } std
 * @return {*}
 */
void BulidPointSet4DBSCAN2(std::vector<RadarMeasure_struct> &fifo_point,
                           std::vector<std::vector<uint16_t>> &clusterSet,
                           std::vector<vehicleInfo_struct> &vehicleInfo) {
  DBSCAN::Point4DBSCAN Point;
  std::vector<DBSCAN::Point4DBSCAN> PointSet;

  /* 设置点云DBSCAN参数 */
  for (uint16_t n = 0; n < fifo_point.size(); n++) {
    if ((fifo_point[n].DynProp == stationary) ||
        (fifo_point[n].DynProp == unknown) ||
        (fifo_point[n].invalid & (1 << invalid_det))) {
      continue;
    }

    Point.PointInfo.ID = n;
    Point.PointInfo.DistLat = fifo_point[n].DistLat;
    Point.PointInfo.DistLong = fifo_point[n].DistLong;
    Point.PointInfo.RCS = fifo_point[n].RCS;
    Point.PointInfo.V = fifo_point[n].VrelLong;
    Point.PointInfo.DynProp = fifo_point[n].DynProp;
    Point.PointInfo.valid = true;
    Point.PointInfo.prob_exit = fifo_point[n].ProbOfExist;

    Point.PointInfo.Range = sqrt(pow(Point.PointInfo.DistLat, 2.0) +
                                 pow(Point.PointInfo.DistLong, 2.0));
    Point.PointInfo.Azi =
        atan2(Point.PointInfo.DistLat, Point.PointInfo.DistLong) / 3.14F *
        180.0F;

     // 提高正前方运动目标的关联门限
    if(fabs(Point.PointInfo.DistLat) < 2.0)
    {
        Point.DBSCAN_para.Search_R = 2.5F + fabs(Point.PointInfo.V) * 0.2;
    }
    else
    {
        if(fabs(vehicleInfo.at(0).yaw_rate) < (5.0*DEG2RAD))
        {
            Point.DBSCAN_para.Search_R = 1.5F + fabs(Point.PointInfo.V) * 0.2;
        }
        else
        {
            Point.DBSCAN_para.Search_R = 2.5F + fabs(Point.PointInfo.V) * 0.2; // 自车转弯是扩大聚类半径
        }
    }

    Point.DBSCAN_para.minPts = 1;
    Point.DBSCAN_para.pointType = 255;
    Point.DBSCAN_para.static_or_dyna = 0;

    PointSet.push_back(Point);
  }

  /* 执行DBSCAN */
  DBSCAN::KNN_DBSCAN(PointSet, clusterSet);

  /* 还原每个cluster中点云真实ID */
  for (auto &sub_cluster : clusterSet) {
    for (auto &sud_point_idx : sub_cluster) {
      sud_point_idx = PointSet[sud_point_idx].PointInfo.ID;
    }
  }
}

/**
 * @name:
 * @description:
 * @param {trackTable_strcut} *trackInfo
 * @param {  } std
 * @param {  } std
 * @param {  } std
 * @param {  } std
 * @return {*}
 */
void creat_rect(trackTable_strcut *trackInfo, std::vector<uint16_t> &TraceSet,
                std::vector<det_box_t> &det_box_list,
                std::vector<rect_point_struct> &trace_box_list,
                std::vector<rect_point_struct> &det_rect_box_list) {
  // 遍历航迹
  for (auto trackidx : TraceSet) {
    trackTable_strcut *trace = &trackInfo[trackidx];
    rect_point_struct trace_box;

    creat_rect_box_point(
        trace->KalmanInfo.StateEst(0), trace->KalmanInfo.StateEst(1),
        trace->ExtendInfo.Length * 1.5, trace->ExtendInfo.Width * 1.5,
        trace->ExtendInfo.box_theta, trace_box);

    trace_box_list.push_back(trace_box);
  }

  for (const auto det_box : det_box_list) {
    rect_point_struct cluster_box;
    creat_rect_box_point(det_box.x_pos, det_box.y_pos, det_box.len, det_box.wid,
                         0, cluster_box);
    det_rect_box_list.push_back(cluster_box);
  }
}

/**
 * @name:
 * @description:
 * @param {vector<trace_meas_pair>} &cost_matrix
 * @return {*}
 */
void sort_score(std::vector<greedy_match_info_t> &cost_matrix) {
  // 从大到小排列
  std::sort(cost_matrix.begin(), cost_matrix.end(), my_compare);
}

/**
 * @name:
 * @description:
 * @param {trace_meas_pair} &c1
 * @param {trace_meas_pair} &c2
 * @return {*}
 */
bool my_compare(const greedy_match_info_t &c1, const greedy_match_info_t &c2) {
  if (c1.iou > c2.iou) {
    return true;
  } else {
    if (c1.iou < c2.iou) {
      return false;
    } else {
      if (c1.center_dis < c2.center_dis) {
        return true;
      } else {
        return false;
      }
    }
  }
}

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @param {  } std
 * @param {trackTable_strcut} *trackInfo
 * @param {vector<uint16_t>} &TraceSet
 * @param {vector<std::vector<uint16_t>>} all_assigned_box
 * @param {vector<std::vector<uint16_t>>} &clusterSet
 * @param {vector<det_box_t>} &det_box_list
 * @return {*}
 */
void dynamic_trace_assigned_with_point(
    std::vector<RadarMeasure_struct> &fifo_point, trackTable_strcut *trackInfo,
    const std::vector<uint16_t> &TraceSet,
    const std::vector<std::vector<uint16_t>> all_assigned_box,
    const std::vector<std::vector<uint16_t>> &clusterSet,
    const std::vector<det_box_t> &det_box_list) {

  VectorXd measData2 = VectorXd(2);

  for (uint8_t trackidx = 0; trackidx < TraceSet.size(); trackidx++) {
    // 本次是否关联到目标
    if (all_assigned_box.at(all_assigned_box.size() - TraceSet.size() + trackidx).size() == 0) {
      continue;
    }

    trackTable_strcut *trace = &trackInfo[TraceSet.at(trackidx)];  // 获取索引号对应的真正的航迹序号

    std::cout << "Matched !" << "Trace ID:[ " <<  trace->trackID << "] "
              << trace->KalmanInfo.StateEst(iDistLat) << ", " << trace->KalmanInfo.StateEst(iDistLong) << std::endl;

    Eigen::Matrix2d rmm_temp, rmm_temp_inv, rmm_temp2, rmm_temp_inv2;

    if(fabs(fabs(trace->ExtendInfo.box_theta) - 90.0*DEG2RAD) < (30.0*DEG2RAD))
    {
        compute_new_ellis_mat(trace, rmm_temp, 1.2);
        rmm_temp_inv = rmm_temp.inverse();

        compute_new_ellis_mat(trace, rmm_temp2, 0.8);
        rmm_temp_inv2 = rmm_temp2.inverse();
    }
    else
    {
        compute_new_ellis_mat(trace, rmm_temp, 0.8);
        rmm_temp_inv = rmm_temp.inverse();

        compute_new_ellis_mat(trace, rmm_temp2, 0.5);
        rmm_temp_inv2 = rmm_temp2.inverse();
    }

    rect_point_struct trace_box;
    creat_rect_box_point(trace->KalmanInfo.StateEst(0),
                         trace->KalmanInfo.StateEst(1),
                         trace->ExtendInfo.Length, trace->ExtendInfo.Width,
                         -1.0 * trace->ExtendInfo.box_theta, trace_box);

    double trace_area, det_box_area;
    double i_, new_iou, prob_factor;
    trace_area = calcularea(trace_box);

    // 遍历关联的所有cluster
    std::vector<double> xpos, ypos;

    for (const auto cluster_idx : all_assigned_box.at(
             all_assigned_box.size() - TraceSet.size() + trackidx)) {
      rect_point_struct det_box;

      // 计算该cluster的面积
      creat_rect_box_point(det_box_list.at(cluster_idx).x_pos,
                           det_box_list.at(cluster_idx).y_pos,
                           det_box_list.at(cluster_idx).len,
                           det_box_list.at(cluster_idx).wid, 0.0, det_box);

      det_box_area = calcularea(det_box);
      i_ = intersection_area(trace_box, det_box);

      new_iou = 0.0;

      if (i_ > 0.0) {
        new_iou = (trace_area > det_box_area) ? (i_ / det_box_area)
                                              : (i_ / trace_area);
      }

      prob_factor = new_iou / 0.8 + 0.1;
      prob_factor = Valuelimit(0.0, 1.0, prob_factor);

      const auto subset = clusterSet.at(cluster_idx);
      for (const auto sub_meas_idx : subset)  // 计算量测与航迹的似然值
      {
        RadarMeasure_struct &detInfo = fifo_point[sub_meas_idx];
        matchInfo_t matchInfo;

        measData2 << detInfo.DistLong - trace->KalmanInfo.StateEst(1),
            detInfo.DistLat - trace->KalmanInfo.StateEst(0);

        double likeliHood_pos_big = measData2.transpose() * rmm_temp_inv * measData2;
        likeliHood_pos_big = exp(-0.5 * pow(likeliHood_pos_big, 2.0));

        double likeliHood_pos_small =
            measData2.transpose() * rmm_temp_inv2 * measData2;
        likeliHood_pos_small = exp(-0.5 * pow(likeliHood_pos_small, 2.0));

        xpos.push_back(detInfo.DistLat);
        ypos.push_back(detInfo.DistLong);

        if (likeliHood_pos_small > 0.5) {
          trace->MeasInfo.HighQuaPointNum++;
        }

        matchInfo.cluster_Idx = sub_meas_idx;

        if (trace->trackState == TRACK_STATE_DETECTION) {
          matchInfo.weight =
              (likeliHood_pos_big * 0.5 + likeliHood_pos_small * 0.5);
        } else {
          if (trace->ExtendInfo.Object_Class < VEHICLE) {
            matchInfo.weight =
                (likeliHood_pos_big * 0.4 + likeliHood_pos_small * 0.6);
          } else {
            matchInfo.weight =
                (likeliHood_pos_big * 0.6 + likeliHood_pos_small * 0.4);
          }
        }

        matchInfo.weight *= prob_factor;

        matchInfo.used4Fit = true;

        // 存储关联点迹
        trace->MeasInfo.associateNum++;
        trace->MeasInfo.detInfo.push_back(matchInfo);

        detInfo.invalid |= (1 << notUed4Init_det);
      }
    }

    // 统计所有关联点形成的box与航迹box的iou
    #if 0
    if ((trace->ExtendInfo.Length > 6.0) &&
        (fabs(trace->ExtendInfo.Orin_For_Display) < (10.0 * DEG2RAD))) {
      auto meas_lat_minmax = std::minmax_element(xpos.begin(), xpos.end());
      auto meas_long_minmax = std::minmax_element(ypos.begin(), ypos.end());

      double len = (*meas_long_minmax.second - *meas_long_minmax.first);
      double wid = (*meas_lat_minmax.second - *meas_lat_minmax.first);

      len = Valuelimit(0.5, len, len);
      wid = Valuelimit(0.5, wid, wid);

      Eigen::MatrixXd meas_mat =
          Eigen::MatrixXd(trace->MeasInfo.associateNum, 2);
      Eigen::MatrixXd Y_hat = Eigen::MatrixXd::Zero(2, 2);
      uint8_t idx = 0;
      for (auto &dets : trace->MeasInfo.detInfo) {
        meas_mat(idx, 0) = fifo_point[dets.cluster_Idx].DistLat;
        meas_mat(idx, 1) = fifo_point[dets.cluster_Idx].DistLong;
        idx++;
      }

      //        std::cout << meas_mat << std::endl;

      Eigen::MatrixXd y_hat = meas_mat.colwise().mean();

      Eigen::RowVectorXd Z_mean(
          Eigen::RowVectorXd::Map(y_hat.data(), meas_mat.cols()));

      Eigen::MatrixXd ZK = meas_mat;
      ZK.rowwise() -= Z_mean;

      // 量测位置协方差矩阵
      Y_hat = (ZK.adjoint() * ZK) / trace->MeasInfo.associateNum;

      // 做特征值分解
      Eigen::EigenSolver<Eigen::Matrix2d> es(Y_hat);

      //        std::cout<< "track ID_____________________________________" <<
      //        trace->trackID << std::endl;

      // 求解特征值
      Eigen::MatrixXcd d = es.eigenvalues();
      //        std::cout << d << std::endl;
      Eigen::MatrixXcd evecs = es.eigenvectors();
      MatrixXd B;
      B = evecs.real();
      //      std::cout << "evecs" << std::endl << evecs << std::endl;
      //        std::cout << "B" << std::endl << B << std::endl;

      // 求解特征向量
      Eigen::MatrixXcd evals = es.eigenvalues();
      Eigen::MatrixXd D_temp, D;
      D_temp = evals.real();
      D = D_temp.asDiagonal();
      //      std::cout << "evals" << std::endl << evals << std::endl;
      //      std::cout << "D_temp" << std::endl << D_temp << std::endl;
      //        std::cout << "D" << std::endl << D << std::endl;

      double len_scale = (len < trace->ExtendInfo.Length)
                              ? (len / trace->ExtendInfo.Length)
                              : (trace->ExtendInfo.Length / len);
      double wid_scale = (wid < trace->ExtendInfo.Width)
                              ? (wid / trace->ExtendInfo.Width)
                              : (trace->ExtendInfo.Width / wid);

      double fact_ = len_scale * wid_scale;

      if ((fact_ < (1 / 4)) || ((fabs(wid - trace->ExtendInfo.Width) > 2.0) &&
                                (fabs(B(0, 0)) > 0.95))) {
        trace->MeasInfo.associateNum = 0;
        trace->MeasInfo.detInfo.clear();
      }
    }
    #endif
  }
}

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @param {trackTable_strcut} *trace
 * @param {  } Eigen
 * @param {double} scale
 * @return {*}
 */
void compute_new_ellis_mat(const trackTable_strcut *trace,
                           Eigen::Matrix2d &new_rmm_mat, double scale) {
  MatrixXd RotMat = MatrixXd::Zero(2, 2);
  MatrixXd eigen_val = MatrixXd::Zero(2, 2);

  double ang = trace->ExtendInfo.box_theta;

  if (fabs(ang) > (0.5 * RADAR_PI)) {
    ang = (ang > 0.0) ? (ang - RADAR_PI) : (ang + RADAR_PI);
  }

  double tempW, tempL;

  tempL = trace->ExtendInfo.Length * scale;
  tempW = trace->ExtendInfo.Width * scale;

  tempL = Valuelimit(0.5, tempL, tempL);
  tempW = Valuelimit(0.5, tempW, tempW);

  RotMat << cos(ang), -1.0 * sin(ang), sin(ang), cos(ang);
  eigen_val << pow(tempL, 2.0), 0.0, 0.0, pow(tempW, 2.0);

  new_rmm_mat = RotMat * eigen_val * RotMat.transpose();
}

/**
 * @name: ComputeScaleOfEllipse
 * @descripttion: 计算航迹与量测位置关系尺度
 * @param {trackTable_strcut} *trace
 * @return {*}
 */
double ComputeScaleOfEllipse(trackTable_strcut *trace, double dis_lat,
                             double dis_long) {
  Eigen::Matrix2d rmm_inv = trace->RMM.RMM_shape.inverse();
  VectorXd diff = VectorXd(2);

  double likeliHood;

  diff << dis_long - trace->KalmanInfo.StateEst(1),
      dis_lat - trace->KalmanInfo.StateEst(0);

  likeliHood = diff.transpose() * rmm_inv * diff;
  likeliHood = exp(-0.5 * pow(likeliHood, 2.0));

  return likeliHood;
}
