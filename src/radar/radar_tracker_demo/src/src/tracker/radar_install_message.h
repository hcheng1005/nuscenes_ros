/*
 * @Description: 
 * @version: 
 * @Author: ChengHao
 * @Date: 2022-10-12 08:58:47
 * @LastEditors: ChengHao
 * @LastEditTime: 2022-10-12 09:52:45
 */
#ifndef RADAR_INSTALL_MESSAGE_H
#define RADAR_INSTALL_MESSAGE_H

// #include "dataStruct.h"

///////////雷达安装信息（以车辆后轴中心为原点）////////////
#define RADAR_0 (1)
#define RADAR_0_X (3.663)
#define RADAR_0_Y (-0.873)
#define RADAR_0_YAW (-1.484)  //单位：rad

#define RADAR_1 (2)
#define RADAR_1_X (3.86)
#define RADAR_1_Y (-0.7)
#define RADAR_1_YAW (-0.436)  //单位：rad

#define RADAR_2 (3)
#define RADAR_2_X (3.86)
#define RADAR_2_Y (0.7)
#define RADAR_2_YAW (0.436)  //单位：rad

#define RADAR_3 (4)
#define RADAR_3_X (3.663)
#define RADAR_3_Y (0.873)
#define RADAR_3_YAW (1.484)  //单位：rad

#define RADAR_VALID_NUM (4)  //参与工作的雷达个数

#endif
