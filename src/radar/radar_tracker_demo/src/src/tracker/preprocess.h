/*
 * @Description:
 * @version:
 * @Author: ChengHao
 * @Date: 2022-10-12 08:58:47
 * @LastEditors: ChengHao hao.cheng@wuzheng.com
 * @LastEditTime: 2023-11-03 14:57:38
 */
#ifndef PREPROCESS_H
#define PREPROCESS_H

#include <string>
#include <vector>

#include "dataStruct.h"
#include "motionStateDist.h"

#define DistLongMAX (250.0F)
#define DistLongMIN (0.1F)
#define DistLatMAX (50.0)
#define DistLatMIN (-50.0F)
#define RCSMIN (-15.0F)  // not used

void delete_outliers(RadarOutput_Struct *indata);
void pointsProcess(RadarOutput_Struct &indata,
                   staticPointSet_struct &staticPointSet);
void perprocess(RadarOutput_Struct &indata,
                std::vector<vehicleInfo_struct> &vehicleInfo);


#endif
