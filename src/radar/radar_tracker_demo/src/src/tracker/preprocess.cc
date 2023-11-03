/*
 * @Description:
 * @version:
 * @Author: ChengHao
 * @Date: 2022-10-09 19:03:02
 * @LastEditors: ChengHao hao.cheng@wuzheng.com
 * @LastEditTime: 2023-11-03 14:56:48
 */
#include "preprocess.h"

#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

// 动态点集合
frame_t dynamicPointSet;

// 静态点集合
staticPointSet_struct staticPointSet;

extern double RADAR_MOUNT_X, RADAR_MOUNT_Y, RADAR_MOUNT_YAW;

/**
 * @name: delete_outliers
 * @description: 量测点迹初步筛选
 * @param {RadarOutput_Struct} *indata
 * @return {*}
 */
void delete_outliers(RadarOutput_Struct *indata) {
  RadarMeasure_struct *sub_point;
  double dis_lat, dis_long;
  uint32_t reID = 0;
  bool valid_point = false;
  for (uint16_t i = 0; i < indata->Header.ActualRecNum; i++) {
    valid_point = false;

    sub_point = &indata->RadarMeasure[i];

    sub_point->ID = reID;

    dis_lat = indata->RadarMeasure[i].DistLat;
    dis_long = indata->RadarMeasure[i].DistLong;

    // 雷达量测点云转换至IMU坐标系
    sub_point->DistLong = dis_long * cos(RADAR_MOUNT_YAW) +
                          -1.0 * dis_lat * sin(RADAR_MOUNT_YAW) + RADAR_MOUNT_X;
    sub_point->DistLat = dis_long * sin(RADAR_MOUNT_YAW) +
                         dis_lat * cos(RADAR_MOUNT_YAW) + RADAR_MOUNT_Y;

#ifndef ARS548_RADAR

    if ((indata->RadarMeasure[i].DistLat > DistLatMIN) &&
        (indata->RadarMeasure[i].DistLat < DistLatMAX) &&
        (indata->RadarMeasure[i].DistLong > DistLongMIN) &&
        (indata->RadarMeasure[i].DistLong < DistLongMAX)) {
      indata->RadarMeasure[i].associateTrackID = MEASURE_NOT_ASSOCIATED;
      indata->RadarMeasure[i].invalid = (1 << valid_det);
      valid_point = true;
    }
#else

    double point_range = sqrt(pow(indata->RadarMeasure[i].DistLat, 2.0) +
                              pow(indata->RadarMeasure[i].DistLong, 2.0));

    const double Z_limit_para[5][2] = {
        {-1.5, 2.0}, {-2.5, 2.0}, {-2.5, 2.0}, {-2.5, 2.0}, {-2.5, 2.0}};

    uint idx = point_range / 10.0;

    if ((indata->RadarMeasure[i].DynProp == stationary) ||
        (indata->RadarMeasure[i].DynProp == unknown)) {
      if ((indata->RadarMeasure[i].DistLat > DistLatMIN) &&
          (indata->RadarMeasure[i].DistLat < DistLatMAX) &&
          (indata->RadarMeasure[i].DistLong > DistLongMIN) &&
          (indata->RadarMeasure[i].DistLong < DistLongMAX)) {
        if (idx < 5) {
          if (((indata->RadarMeasure[i].DistLong_rms > Z_limit_para[idx][0]) &&
               (indata->RadarMeasure[i].DistLong_rms < Z_limit_para[idx][1]))) {
            indata->RadarMeasure[i].associateTrackID = MEASURE_NOT_ASSOCIATED;
            indata->RadarMeasure[i].invalid = (1 << valid_det);
            valid_point = true;
          }
        } else {
          indata->RadarMeasure[i].associateTrackID = MEASURE_NOT_ASSOCIATED;
          indata->RadarMeasure[i].invalid = (1 << valid_det);
          valid_point = true;
        }
      }
    } else {
      if ((indata->RadarMeasure[i].DistLat > DistLatMIN) &&
          (indata->RadarMeasure[i].DistLat < DistLatMAX) &&
          (indata->RadarMeasure[i].DistLong > DistLongMIN) &&
          (indata->RadarMeasure[i].DistLong < DistLongMAX) &&
          ((indata->RadarMeasure[i].DistLong_rms > Z_limit_para[idx][0]) &&
           (indata->RadarMeasure[i].DistLong_rms < Z_limit_para[idx][1]))) {
        indata->RadarMeasure[i].associateTrackID = MEASURE_NOT_ASSOCIATED;
        indata->RadarMeasure[i].invalid = (1 << valid_det);

        valid_point = true;
      }
    }

#endif

    if (valid_point) {
      memcpy(&indata->RadarMeasure[reID], &indata->RadarMeasure[i],
             sizeof(RadarMeasure_struct));
      reID++;
    }
  }

  // 更新有效目标个数
  indata->Header.ShouldRecNum = reID;
  indata->Header.ActualRecNum = reID;

  std::cout << "final valid number: " << indata->Header.ShouldRecNum << std::endl;

}

/**
 * @name: pointsProcess
 * @description: 量测预处理：1、动静点迹判定
 * @param {RadarOutput_Struct} &indata
 * @param {  } std
 * @param {  } staticPointSet_
 * @param {frame_t} &dynamicPointSet
 * @param {MatrixXi} &gridMap
 * @return {*}
 */
void pointsProcess(RadarOutput_Struct &indata,
                   std::vector<vehicleInfo_struct> &vehicleInfo,
                   staticPointSet_struct &staticPointSet) {
  motionStateDist(indata, vehicleInfo, staticPointSet);
}

void perprocess(RadarOutput_Struct &indata,
                std::vector<vehicleInfo_struct> &vehicleInfo) {
  // step1：动静识别
  // pointsProcess(indata, vehicleInfo, staticPointSet);

  // 点云删减
  delete_outliers(&indata);
}
