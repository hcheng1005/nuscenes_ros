#ifndef CONTIRADAR_H
#define CONTIRADAR_H

#include <bits/stdc++.h>

#define MAXPOINTNUM 1000

//               x axis(long)
//		       ^
//                     |
//                  *  |  *
//               *     |     *
//        \    *       |       *    /
//          \ *     range(i)    * /
//            \        |        /
//              \      |      /
//                \    |    /
//      y axis(long)\  |  /
//      <---------------

typedef enum {
  Pos_LeftFront = 0,
  Pos_Front,
  Pos_RightFront,
  Pos_Right,
  Pos_RightBack,
  Pos_Back,
  Pos_LeftBack,
  Pos_Left,
  Pos_Inner
} PosIdx_enum;

typedef enum InvalidState_ {
  /* Conti408原生定义 */
  Cluster_Valid = 0x00,                        // 0
  Cluster_Invalid_low_RCS,                     // 1
  Cluster_Invalid_near_field,                  // 2
  Cluster_Invalid_far_range_Cluster,           // 3
  Cluster_Valid_low_RCS,                       // 4
  Cluster_Cluster_reserved,                    // 5
  Cluster_Invalid_high_mirror_prob,            // 6
  Cluster_Invalid_outsideFOV,                  // 7
  Cluster_Valid_due_to_elevation,              // 8
  Cluster_Valid_high_child_probability,        // 9
  Cluster_Valid_high_prob_of_50_deg_artefact,  // 10
  Cluster_Valid_but_no_local_maximum,          // 11
  Cluster_Valid_high_artefact_probability,     // 12
  Cluster_reserved2,                           // 13
  Cluster_Invalid_harmonics,                   // 14
  Cluster_Valid_95_m_in_near_range,            // 15
  Cluster_Valid_multi_target_probability,      // 16
  Cluster_Valid_suspicious_angle,              // 17

  /*跟踪算法内部定义*/
  cluster_Invalid_Asso2Line,

} InvalidState_enum;

typedef enum AmbigState_ {
  Ambig_invalid = 0x00,
  Ambig_ambiguous,
  Ambig_staggered_ramp,
  Ambig_unambiguous,
  Ambig_stationary_candidate,
} AmbigState_enum;

typedef enum AssoedType_ {
  UpdateWithRange = 0,
  UpdateWithRangeAndInner,
  UpdateWithLowWeight_R,
  UpdateWithDoppler,
  UpdateWithLowWeight_V,
  UpdateWithLowWeight_V_L2,
  UpdateButNotDoppler,
  OnlyAssoNotUpdate,
} AssoedType_enum;

typedef enum {
  UNKNOWN = 0,
  UNKNOWN_MOVABLE = 1,
  UNKNOWN_UNMOVABLE = 2,
  PEDESTRIAN = 3,
  BICYCLE = 4,
  VEHICLE = 5,
  MAX_OBJECT_TYPE = 6,
  TRUCK,
} ObjectType_enum;

typedef enum {
  moving = 0,
  stationary,
  oncoming,  //2
  stationary_candidate, //3
  unknown,
  crossing_stationary,
  crossing_moving,
  stopped,  // 7
} ObjectDynProp_enum;

typedef enum {
  TRK_Unstable = 0,
  TRK_NotSure,
  TRK_Stable,
} track_stable_enum;

typedef enum {
  valid_det = 0,
  assoed2track,
  assoed2Temptrack,
  couldReuse,
  notUed4Init_det,
  invalid_det,
} PointValid_enum;

typedef struct {
  uint32_t TimeStamp;  //时间戳
  uint16_t ID;         //检测点ID

  float DistLong;      //检测点纵向（X）距离
  float DistLong_rms;  //检测点纵向（X）距离标准差
  float VrelLong;      //检测点纵向（X）相对速度
  float VrelLong_rms;  //检测点纵向（X）相对速度标准差

  float DistLat;      //检测点横向（Y）距离
  float DistLat_rms;  //检测点横向（Y）距离标准差
  float VrelLat;      //检测点横向（Y）相对速度
  float VrelLat_rms;  //检测点横向（Y）相对速度标准差

  float Vr_abs;

  uint8_t DynProp;  //检测点运动状态，0x0~0x7:moving,stationary,oncoming,

  // stationary candidate,unknown,
  float RCS;      //检测点RCS
  float Azimuth;  //检测点方位角，单位：rad

  float ArelLong;  //跟踪目标纵向（X）相对加速度
  uint8_t ArelLong_Valid : 1;
  float ArelLong_rms;  //检测点横向（Y）加速度标准差
  uint8_t ArelLong_rms_Valid : 1;

  float ArelLat;  //跟踪目标横向（Y）相对加速度
  uint8_t ArelLat_Valid : 1;
  float ArelLat_rms;  //检测点横向（X）加速度标准差
  uint8_t ArelLat_rms_Valid : 1;

  float Class;  //跟踪目标类别
  uint8_t Class_Valid : 1;

  uint8_t OrientationAngel;  //跟踪目标方向角
  uint8_t OrientationAngel_Valid : 1;
  float Orientation_rms;  //航向角标准差
  uint8_t Orientation_rms_Valid : 1;

  float Length;  //跟踪目标长度
  uint8_t Length_Valid : 1;
  float Width;  //跟踪目标宽度
  uint8_t Width_Valid : 1;

  float MeasState;
  uint8_t MeasState_Valid : 1;
  float ProbOfExist;
  uint8_t ProbOfExist_Valid : 1;

  uint8_t Pdh0;  //检测点是虚警点（多径等原因）的概率
  uint8_t Pdh0_Valid : 1;
  AmbigState_enum AmbigState;  //检测点是否存在多普勒模糊
  uint8_t AmbigState_Valid : 1;
  uint8_t InvalidState;  //检测点是否有效的标识，分为很多种状态
  uint8_t InvalidStates_Valid : 1;

//  uint8_t Point_mode;
//  uint8_t AngIdx;
//  uint8_t AngProb;

  /* 额外属性请向下延伸定义 */
  uint8_t invalid;

  uint16_t associateTrackID;
  uint16_t closeTrk;
  uint16_t close_CT;
  uint16_t close_UCT;

  uint8_t Pos2Corner;
  bool InnerPoint;

  uint8_t sensorID;
  double lat_seq;
  double long_seq;
} RadarMeasure_struct;

typedef struct {
  double timestamp;
  double x_seq;
  double y_seq;
  double yaw_seq;
  double vx;
  double yaw_rate;
} egoInfo_t;

typedef struct {
  double timestamp;
  double sensor_id;
  double range_sc;
  double azimuth_sc;
  double rcs;
  double vr;
  double vr_compensated;
  double x_cc;
  double y_cc;
  double x_seq;
  double y_seq;
} radarScenes_t;

typedef struct RadarHeader_ {
  uint32_t TimeStamp;  //帧头时间戳
  uint32_t ShouldRecNum;
  uint32_t ActualRecNum;
  uint32_t NofNear;  //中距模式下的检测点个数
  uint32_t NofNear_Valid : 1;
  uint32_t NofFar;  //远距模式下的检测点个数
  uint32_t NofFar_Valid : 1;
  uint32_t MeasCounter;  //测量次数，>65535时从0重新开始
  uint32_t InterfaceVersion;
} RadarHeader_struct;

typedef struct {
  RadarHeader_struct Header;
  RadarMeasure_struct RadarMeasure[MAXPOINTNUM];
} RadarOutput_Struct;

typedef struct {
  std::vector<egoInfo_t> egoInfo;
  std::vector<radarScenes_t> radarScenes;
} frame_t;

typedef struct {
  uint8_t frameIndex;
  float x_seq;      //静态目标点纵向（X轴）坐标（全局）
  float y_seq;      //静态目标点横向（Y轴）坐标（全局）
  float radar_rcs;  //静态目标点RCS
} staticPoint_struct;

typedef struct {
  double vehicle_x_seq;  //静态目标点对应的车辆全局坐标，纵向（X轴）
  double vehicle_y_seq;  //静态目标点对应的车辆全局坐标，横向（Y轴）
  double vehicle_yaw_seq;  //静态目标点对应的车辆航向角
  std::vector<uint16_t> staticPointNum;
  std::vector<staticPoint_struct> staticPointSet;
} staticPointSet_struct;

#endif
