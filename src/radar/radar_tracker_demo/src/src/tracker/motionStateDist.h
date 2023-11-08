/*
 * @Description:
 * @version:
 * @Author: ChengHao
 * @Date: 2022-10-12 08:58:47
 * @LastEditors: ChengHao
 * @LastEditTime: 2022-10-12 09:52:32
 */
#ifndef MOTIONSTATEDIST_H
#define MOTIONSTATEDIST_H

#include <string>
#include <vector>

#include "dataStruct.h"
#include "inttypes.h"

using Eigen::MatrixXd;
using Eigen::MatrixXi;
using Eigen::VectorXd;
using Eigen::VectorXi;

//静态目标累积帧数
#define ACCU_FRAME_NUM (10)
//动静态目标分离速度门限
#define SPEED_TH (0.45)

//检测点运动状态判决模块
void motionStateDist(RadarOutput_Struct &indata,
                     std::vector<vehicleInfo_struct> &vehicleInfo,
                     staticPointSet_struct &staticPointSetOut);

//坐标转换
MatrixXd vehicleCoor_to_Global(MatrixXd rectangle_vcoor, VectorXd v_global,
                               float v_yaw);

//判断静态目标点是否处于所设矩形框内
bool isInRectangle(MatrixXd rectangle_global, staticPoint_struct pointIn);

void GetInstallInfo(uint8_t sensorID, float *x_pos, float *y_pos, float *yaw);

VectorXd model2(MatrixXd dataIn, int16_t n);
bool myransac(MatrixXd dataIn, int16_t datalen, int16_t n_shuffle,
              int16_t itersNum, float dis, float per, VectorXd &vEst,
              VectorXi &inlierIndex, uint16_t *inlier_num);

void motionStateDist2(RadarOutput_Struct &indata,
                      std::vector<vehicleInfo_struct> &vehicleInfo,
                      staticPointSet_struct &staticPointSetOut);
#endif
