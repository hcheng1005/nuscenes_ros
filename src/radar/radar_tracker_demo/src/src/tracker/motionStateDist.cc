/*
 * @Descripttion:
 * @version:
 * @Author: ChengHAO
 * @Date: 2022-09-01 21:03:23
 * @LastEditors: ChengHao hao.cheng@wuzheng.com
 * @LastEditTime: 2023-11-03 14:59:11
 */
#include <cmath>
#include <iostream>

#include "motionStateDist.h"
#include "radar_install_message.h"

// TODO： 如果要使用动静分离函数，请填写毫米波雷达的安装参数
double RADAR_MOUNT_X = 0.0, RADAR_MOUNT_Y = 0.0, RADAR_MOUNT_YAW = 0.0;

using namespace std;
using Eigen::MatrixXd;
using Eigen::MatrixXi;
using Eigen::VectorXd;
using Eigen::VectorXi;

#ifdef ARS548_RADAR
#define BASE_GATE (0.3)
#else
#define BASE_GATE (0.24 * 2)
#endif

/**
 * @names:
 * @description: 检测点运动状态判决模块
 * @param {RadarOutput_Struct&} indata
 * @param {  } std
 * @param {staticPointSet_&} staticPointSetOut
 * @return {*}
 */

void motionStateDist2(RadarOutput_Struct &indata,
                      std::vector<vehicleInfo_struct> &vehicleInfo,
                      staticPointSet_struct &staticPointSetOut) {
  double est_ego_v = 0.0;
  int temp1 = (int)(vehicleInfo.at(0).vx / 0.25);

  est_ego_v = temp1 * 0.25;

  if (fabs(vehicleInfo.at(0).vx - est_ego_v) > 0.1) {
    //        est_ego_v += ()
  } else {
    est_ego_v += 0.25;
  }

  for (uint16_t i = 0; i < indata.Header.ActualRecNum; i++) {
    if (fabs(indata.RadarMeasure[i].VrelLong + est_ego_v) < 0.3) {
      indata.RadarMeasure[i].DynProp = stationary;
    } else {
      if (indata.RadarMeasure[i].DynProp == unknown) {
        indata.RadarMeasure[i].invalid = (1 << invalid_det);
      } else {
        indata.RadarMeasure[i].DynProp = moving;  // moving
      }
    }
  }
}

void motionStateDist(RadarOutput_Struct &indata,
                     std::vector<vehicleInfo_struct> &vehicleInfo,
                     staticPointSet_struct &staticPointSetOut) {
  float radar_install_x = 0;
  float radar_install_y = 0;
  float radar_install_yaw = 0;
  float r_temp = 0;
  float angle_temp = 0;
  float radar_vlinear = 0;
  float radar_theory_v = 0;
  float radar_v_yaw = 0;
  float point_theory_v = 0;
  float v_diff = 0;
  uint8_t sensorID = 0;
  uint8_t sensorID_temp = 127;
  int8_t odometryIndex = 0;
  VectorXd odometry_xy = VectorXd(2);
  // float odometry_yaw = 0;
  float odometry_v = 0;
  float odometry_yawrate = 0;

  //倒车判定逻辑：比较速度航向角和车辆航向角方向是否一致
  bool VehicleReverse = false;
  double v_heading = atan2(vehicleInfo[0].Vlong, vehicleInfo[0].Vlat);

  if ((fabs(fabs(v_heading) - fabs(vehicleInfo[0].heading)) >
       (10.0 * DEG2RAD)) &&
      (fabs(fabs(fabs(v_heading) + fabs(vehicleInfo[0].heading) - M_PI)) <
       (10.0 * DEG2RAD))) {
    VehicleReverse = true;
  }

  uint16_t pointNum = indata.Header.ActualRecNum;
  for (uint16_t i = 0; i < pointNum; i++) {
    sensorID = 3;  // indata.RadarMeasure[i].sensorID;
    if (sensorID != sensorID_temp) {
      sensorID_temp = sensorID;

      odometryIndex = sensorID - 1;
      odometry_v = vehicleInfo[0].vx;
      // odometry_yaw = vehicleInfo[0].yaw_seq;
      odometry_yawrate = vehicleInfo[0].yaw_rate;
      odometry_xy(0) = vehicleInfo[0].x_seq;
      odometry_xy(1) = vehicleInfo[0].y_seq;
      //            rectangle_global = vehicleCoor_to_Global(rectangle_vcoor,
      //            odometry_xy, odometry_yaw);
      odometryIndex++;

      GetInstallInfo(sensorID_temp, &radar_install_x, &radar_install_y,
                     &radar_install_yaw);

      angle_temp = atan(fabs(radar_install_x / radar_install_y));
      r_temp = sqrtf(radar_install_x * radar_install_x +
                     radar_install_y * radar_install_y);
      radar_vlinear =
          fabs(r_temp * odometry_yawrate);  //由转弯引起的雷达线速度大小
    }

#ifdef ARS548_RADAR
    if (indata.RadarMeasure[i].DynProp == stationary) {
      continue;
    }
#endif

    //雷达相对地面的真实速度（车身速度和雷达线速度的向量和速度，根据三角形边长定理求解）
    if (fabs(odometry_v) < 0.01) {
      point_theory_v = 0;
    } else {
#if 0
            if ((sensorID == RADAR_0)||(sensorID == RADAR_1))
            {
                if (odometry_yawrate < 0.0)
                {
                    radar_theory_v = sqrtf(radar_vlinear*radar_vlinear + odometry_v*odometry_v - 2*odometry_v*radar_vlinear*cos(angle_temp));
                    //雷达速度（radar_theory_v）方向
                    radar_v_yaw = asin(radar_vlinear*sin(angle_temp)/radar_theory_v);
                    //目标理论补偿的多普勒速度
                    point_theory_v = cos((radar_install_yaw + radar_v_yaw + indata.RadarMeasure[i].Azimuth))*radar_theory_v;
                }
                else
                {
                    radar_theory_v = sqrtf(radar_vlinear*radar_vlinear + odometry_v*odometry_v - 2*odometry_v*radar_vlinear*cos(M_PI - angle_temp));
                    //雷达速度（radar_theory_v）方向
                    radar_v_yaw = asin(radar_vlinear*sin(angle_temp)/radar_theory_v);
                    //目标理论补偿的多普勒速度
                    point_theory_v = cos((radar_install_yaw - radar_v_yaw + indata.RadarMeasure[i].Azimuth))*radar_theory_v;
                }
            }
            if ((sensorID == RADAR_2)||(sensorID == RADAR_3))
            {
                if (odometry_yawrate < 0.0)
                {
                    radar_theory_v = sqrtf(radar_vlinear*radar_vlinear + odometry_v*odometry_v - 2*odometry_v*radar_vlinear*cos(M_PI - angle_temp));
                    //雷达速度（radar_theory_v）方向
                    radar_v_yaw = asin(radar_vlinear*sin(angle_temp)/radar_theory_v);
                    //目标理论补偿的多普勒速度
                    point_theory_v = cos((radar_install_yaw + radar_v_yaw + indata.RadarMeasure[i].Azimuth))*radar_theory_v;
                }
                else
                {
                    radar_theory_v = sqrtf(radar_vlinear*radar_vlinear + odometry_v*odometry_v - 2*odometry_v*radar_vlinear*cos(angle_temp));
                    //雷达速度（radar_theory_v）方向
                    radar_v_yaw = asin(radar_vlinear*sin(angle_temp)/radar_theory_v);
                    //目标理论补偿的多普勒速度
                    point_theory_v = cos((radar_install_yaw - radar_v_yaw + indata.RadarMeasure[i].Azimuth))*radar_theory_v;
                }
            }

#endif
      if (odometry_yawrate < 0.0) {
        radar_theory_v =
            sqrtf(radar_vlinear * radar_vlinear + odometry_v * odometry_v -
                  2 * odometry_v * radar_vlinear * cos(RADAR_PI - angle_temp));
        //雷达速度（radar_theory_v）方向
        radar_v_yaw = asin(radar_vlinear * sin(angle_temp) / radar_theory_v);
        //目标理论补偿的多普勒速度
        point_theory_v = cos((radar_install_yaw + radar_v_yaw +
                              indata.RadarMeasure[i].Azimuth)) *
                         radar_theory_v;
      } else {
        radar_theory_v =
            sqrtf(radar_vlinear * radar_vlinear + odometry_v * odometry_v -
                  2 * odometry_v * radar_vlinear * cos(angle_temp));
        //雷达速度（radar_theory_v）方向
        radar_v_yaw = asin(radar_vlinear * sin(angle_temp) / radar_theory_v);
        //目标理论补偿的多普勒速度
        point_theory_v = cos((radar_install_yaw - radar_v_yaw +
                              indata.RadarMeasure[i].Azimuth)) *
                         radar_theory_v;
      }
    }

    //目标实际运动速度
    bool staticFlag = false;
    float v_meas = -indata.RadarMeasure[i].VrelLong;
    if (!VehicleReverse) {
      v_diff = point_theory_v - v_meas;
    } else {
      v_diff = point_theory_v + v_meas;
    }

    /* 动静分类条件（静态目标判定条件）：
     * 1.速度差值小于阈值SPEED_TH
     * 2.在yawRate小于7度且点云径向速度和车速差值小于阈值
     */

    if ((fabs(v_diff) < BASE_GATE) ||
        ((fabs(odometry_yawrate) < (DEG2RAD * 7.0F)) &&
         (fabs(indata.RadarMeasure[i].VrelLong + radar_theory_v) <
          BASE_GATE))) {
      staticFlag = true;
    }

    if (staticFlag == false) {
      // Add：当前处于转弯时，补充判定逻辑
      if (fabs(odometry_yawrate) > (5.0 * DEG2RAD)) {
        if (!VehicleReverse) {
          if (fabs(indata.RadarMeasure[i].VrelLong + odometry_v) <
              (fabs(odometry_yawrate) * 5.0 + 0.5))  //(0.1 + 0.25*2.0F))
          {
            staticFlag = true;
          }
        } else {
          if (fabs(indata.RadarMeasure[i].VrelLong - odometry_v) <
              (fabs(odometry_yawrate) * 5.0 + 0.5))  //(0.1 + 0.25*2.0F))
          {
            staticFlag = true;
          }
        }
      }
    }

    if (staticFlag) {
      indata.RadarMeasure[i].DynProp = stationary;  // stationary
    } else {
      if (indata.RadarMeasure[i].DynProp == unknown) {
        indata.RadarMeasure[i].invalid = (1 << invalid_det);
      } else {
        indata.RadarMeasure[i].DynProp = moving;  // moving
      }
    }
  }
}

/**
 * @names:
 * @description: 判断静态目标点是否处于所设矩形框内
 * @param {MatrixXd} rectangle_global
 * @return {*}
 */
bool isInRectangle(MatrixXd rectangle_global, staticPoint_struct pointIn) {
  VectorXd vector_temp;
  MatrixXd pointXY = MatrixXd(1, 2);
  pointXY(0, 0) = pointIn.x_seq;
  pointXY(0, 1) = pointIn.y_seq;

  VectorXd vector_BA = rectangle_global.row(1) - rectangle_global.row(0);
  VectorXd vector_CB = rectangle_global.row(2) - rectangle_global.row(1);
  VectorXd vector_DC = rectangle_global.row(3) - rectangle_global.row(2);
  VectorXd vector_AD = rectangle_global.row(0) - rectangle_global.row(3);

  vector_temp = rectangle_global.row(0) - pointXY;
  float dot_multi_A = vector_temp.transpose() * vector_BA;
  vector_temp = rectangle_global.row(1) - pointXY;
  float dot_multi_B = vector_temp.transpose() * vector_CB;
  vector_temp = rectangle_global.row(2) - pointXY;
  float dot_multi_C = vector_temp.transpose() * vector_DC;
  vector_temp = rectangle_global.row(3) - pointXY;
  float dot_multi_D = vector_temp.transpose() * vector_AD;

  if (((dot_multi_A >= 0) && (dot_multi_B >= 0) && (dot_multi_C >= 0) &&
       (dot_multi_D >= 0)) ||
      ((dot_multi_A <= 0) && (dot_multi_B <= 0) && (dot_multi_C <= 0) &&
       (dot_multi_D <= 0))) {
    return true;
  } else {
    return false;
  }
}

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @param {MatrixXd} rectangle_vcoor
 * @param {VectorXd} v_global
 * @param {float} v_yaw
 * @return {*}
 */
MatrixXd vehicleCoor_to_Global(MatrixXd rectangle_vcoor, VectorXd v_global,
                               float v_yaw) {
  MatrixXd rectangle_global = MatrixXd(4, 2);
  MatrixXd matrix_T = MatrixXd(2, 2);
  MatrixXd rectangle_global_temp = MatrixXd(4, 2);
  matrix_T << cos(v_yaw), sin(v_yaw), -sin(v_yaw), cos(v_yaw);

  rectangle_global_temp = rectangle_vcoor * matrix_T;
  for (uint16_t i = 0; i < 4; i++) {
    rectangle_global(i, 0) = rectangle_global_temp(i, 0) + v_global(0);
    rectangle_global(i, 1) = rectangle_global_temp(i, 1) + v_global(1);
  }
  return rectangle_global;
}

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @param {uint8_t} sensorID
 * @param {float} *x_pos
 * @param {float} *y_pos
 * @param {float} *yaw
 * @return {*}
 */
void GetInstallInfo(uint8_t sensorID, float *x_pos, float *y_pos, float *yaw) {
  *x_pos = RADAR_MOUNT_X;
  *y_pos = RADAR_MOUNT_Y;
  *yaw = RADAR_MOUNT_YAW;
}

/**
 * @names:
 * @description: 使用随机选取的n个点估计速度
 * @param {MatrixXd} dataIn: 输入的点的信息（角度信息，径向速度）
 * @param {int16_t} n: 随机选取的点的个数
 * @return {*}
 */
VectorXd model2(MatrixXd dataIn, int16_t n) {
  MatrixXd arrayA = MatrixXd(n, 2);
  VectorXd vectorB = VectorXd(n);

  arrayA.col(0) = dataIn.col(0);
  arrayA.col(1) = dataIn.col(1);
  vectorB = dataIn.col(2);

  MatrixXd tempA_T = arrayA.transpose();
  MatrixXd arrayTemp_1 = tempA_T * arrayA;
  MatrixXd arrayTemp_1_inv = arrayTemp_1.inverse();
  MatrixXd arrayTemp_2 = arrayTemp_1_inv * tempA_T;

  VectorXd result_V = arrayTemp_2 * vectorB;
  return result_V;
}

/**
 * @names: ransac
 * @description: 随机一致性采样，随机选取n个点估计模型参数，选取内点数最多的模型
 * @param {MatrixXd} dataIn 输入的点的信息（角度信息，径向速度）
 * @param {int16_t} datalen 点的个数
 * @param {int16_t} n_shuffle 随机选取的点的个数
 * @param {int16_t} itersNum itersNum：迭代次数
 * @param {float} dis 速度门限
 * @param {float} per 内点占比
 * @param {VectorXd} &vEst
 * @param {VectorXi} &inlierIndex
 * @param {uint16_t} *inlier_num
 * @return {*}
 */
bool myransac(MatrixXd dataIn, int16_t datalen, int16_t n_shuffle,
              int16_t itersNum, float dis, float per, VectorXd &vEst,
              VectorXi &inlierIndex, uint16_t *inlier_num) {
  int16_t inlierPre = 0;      // 上一次迭代中的内点总数
  int16_t inlierCurrent = 0;  // 当前迭代中的内点总数

  MatrixXd dataInlier_Curr = MatrixXd(datalen, 3);  // 当前迭代中的内点的信息
  VectorXi dataIndex_Curr = VectorXi(datalen);  // 当前迭代中的内点位置索引

  MatrixXd dataInlier_Pre = MatrixXd(datalen, 3);  // 上一次迭代中的内点的信息
  VectorXi dataIndex_Pre = VectorXi(datalen);  // 上一次迭代中的内点位置索引

  int16_t n_temp = 0;
  if (datalen < n_shuffle) {
    printf("data length error");
    return false;
  } else {
    n_temp = n_shuffle;
  }
  MatrixXd randPoint = MatrixXd(n_temp, 3);  // 随机产生的n_temp个点的信息
  VectorXd result_V = VectorXd(2);  // 随机产生的n_temp个点估计的速度
  MatrixXd result_VSET = MatrixXd(2, itersNum);

  int data_num = datalen * 0.5;
  Eigen::MatrixXd randvalue =
      (Eigen::MatrixXd::Random(1, itersNum * n_temp)).array() * data_num +
      data_num;

#if 1
  for (int16_t i = 0; i < itersNum; i++) {
    for (int16_t j = 0; j < n_temp; j++) {
      randPoint.row(j) = dataIn.row(randvalue(0, n_temp * i + j));
    }
    // 使用随机选取的n_temp个点进行速度粗估计
    result_V = model2(randPoint, n_temp);
    result_VSET.col(i) = result_V;
  }

  MatrixXd pointTemp2 = dataIn.topLeftCorner(datalen, 2);
  MatrixXd v_estim2 = pointTemp2 * result_VSET;

  uint16_t max_idx = 0;
  uint16_t max_point = 0;

  for (int16_t i = 0; i < itersNum; i++) {
    uint16_t xx =
        ((v_estim2.col(i) - dataIn.col(2)).array().abs() < dis).count();

    if (xx > max_point) {
      max_point = xx;
      max_idx = i;
    }
  }

  inlierCurrent = 0;
  MatrixXd diff_v = v_estim2.col(max_idx) - dataIn.col(2);
  for (int16_t j = 0; j < datalen; j++) {
    if (fabs(diff_v(j)) < dis) {
      dataInlier_Curr.row(inlierCurrent) = dataIn.row(j);
      dataIndex_Curr(inlierCurrent) = j;
      inlierCurrent = inlierCurrent + 1;
    }
  }

  dataInlier_Pre.topLeftCorner(inlierCurrent, 3) =
      dataInlier_Curr.topLeftCorner(inlierCurrent, 3);
  dataIndex_Pre = dataIndex_Curr;
  inlierPre = inlierCurrent;

  vEst = model2(dataInlier_Pre.topLeftCorner(inlierPre, 3), inlierPre);
  *inlier_num = inlierPre;
  inlierIndex = dataIndex_Pre;

  return true;

#endif

#if 0
    VectorXi randPointIndex = VectorXi(n_temp); // 随机产生的n_temp个点的索引

    for (int16_t i = 0; i < itersNum; i++)
    {
        inlierCurrent = 0;
        // 随机产生n_temp个点的索引
        for (int16_t randIndex = 0; randIndex < n_temp; randIndex++)
        {
            randPointIndex[randIndex] = (int)(datalen * 1.0 * rand() / RAND_MAX - 1.0);
            randPoint.row(randIndex) = dataIn.row(randPointIndex[randIndex]);
        }

        // 使用随机选取的n_temp个点进行速度粗估计
        result_V = model2(randPoint, n_temp);

        MatrixXd pointTemp2 = dataIn.topLeftCorner(datalen, 2);
        MatrixXd v_orin = dataIn.col(2);
        MatrixXd v_estim2 = pointTemp2 * result_V;
        MatrixXd diff_v = v_estim2 - v_orin;

        diff_v = diff_v.cwiseAbs();

        for(int16_t j = 0; j < datalen; j++)
        {
            if (diff_v(j, 0) < dis)
            {
                dataInlier_Curr.row(inlierCurrent) = dataIn.row(j);
                dataIndex_Curr(inlierCurrent) = j;
                inlierCurrent = inlierCurrent + 1;
            }
        }

        // 如果本轮的内点个数大于上一轮的内点数，则存储本轮的结果
        if (inlierCurrent > inlierPre)
        {
            dataInlier_Pre.topLeftCorner(inlierCurrent, 3) = dataInlier_Curr.topLeftCorner(inlierCurrent, 3);
            dataIndex_Pre = dataIndex_Curr;
            inlierPre = inlierCurrent;
        }

        // 迭代结束后，使用最多内点数的结果进行细估计（所有内点都参与）
        if ((itersNum - 1) == i)
        {
            vEst = model2(dataInlier_Pre.topLeftCorner(inlierPre, 3), inlierPre);
            *inlier_num = inlierPre;
            inlierPercent = inlierPre * 1.0 / datalen;
            inlierIndex = dataIndex_Pre;

            std::cout << vEst.transpose() << std::endl;
        }
    }

#endif
}
