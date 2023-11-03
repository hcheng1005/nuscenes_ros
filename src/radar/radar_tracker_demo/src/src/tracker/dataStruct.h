#ifndef DATASTRUCT_H
#define DATASTRUCT_H

#include <time.h>
#include <algorithm>
#include <fstream>
#include <string>
#include <vector>


#include "inttypes.h"

#define ENV_APOLLO
#define __ContiRADAR__

#ifdef ENV_APOLLO
#include "ContiRadar.h"
#include "Eigen/Core"
#include "Eigen/Dense"
#else
#define LOG_ENBALE (1)
#define QT_ENV_ (1)
#include <sys/time.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include "ContiRadar.h"
#endif


// #define LOG_OUTPUT

//#define ARS548_RADAR

using Eigen::MatrixXd;
using Eigen::VectorXd;

#define TRACE_PROB_LIMIT        (5.0)
#define TRACE_ACTIVE_SAMLL_TAR  (0.5)
#define TRACE_ACTIVE_PERCENT    (0.8)

#define TRACE_FINAL_CONFIRM_PERCENT_EX_SMALL (0.6)
//#define TRACE_FINAL_CONFIRM_PERCENT_SMALL (0.7)
#define TRACE_FINAL_CONFIRM_PERCENT_BIG (0.9)

#define TRACE_DELETE_PERCENT    (0.3)

#define _TRK_VER_ (0x00000106)

#define T_cycle 0.1
#define RADAR_PI M_PI

#define POSITIVE_HALF_PI (M_PI_2)
#define NEGATIVE_HALF_PI (-1.0 M_PI_2)

#define RAD2DEG (180.0 / RADAR_PI)  //
#define DEG2RAD (RADAR_PI / 180.0)  // 0.01744

#define MEASURE_NUM 255

#define MEASURE_NOT_ASSOCIATED MEASURE_NUM + 1

#define MAXTRACKS (64)

#define SSIZE 6
#define MSIZE 3

#define iRANGE 0
#define iAZI 1
#define iVEL 2

#define iDistLat 0
#define iDistLong 1
#define iVrelLat 2
#define iVrelLong 3
#define iAccLat 4
#define iAccLong 5

#define InValidPoint (1)
#define ValidPoint (0)


#if 1
typedef struct {
  uint16_t ID;  // 检测点ID

  float DistLong;  // 检测点纵向（X）距离
  //  float DistLong_rms;  //检测点纵向（X）距离标准差
  float VrelLong;  // 检测点纵向（X）相对速度
  //  float VrelLong_rms;  //检测点纵向（X）相对速度标准差

  float DistLat;  // 检测点横向（Y）距离
  //  float DistLat_rms;  //检测点横向（Y）距离标准差
  float VrelLat;  // 检测点横向（Y）相对速度
  //  float VrelLat_rms;  //检测点横向（Y）相对速度标准差

  //  float Vr_abs;

  uint8_t DynProp;  // 检测点运动状态，0x0~0x7:moving,stationary,oncoming,

  // stationary candidate,unknown,
  float RCS;      // 检测点RCS
  float Azimuth;  // 检测点方位角，单位：rad

  float MeasState;
  uint8_t MeasState_Valid : 1;
  float ProbOfExist;
  uint8_t ProbOfExist_Valid : 1;

  uint8_t Pdh0;  // 检测点是虚警点（多径等原因）的概率
  uint8_t Pdh0_Valid : 1;
  AmbigState_enum AmbigState;  // 检测点是否存在多普勒模糊
  uint8_t AmbigState_Valid : 1;
  uint8_t InvalidState;  // 检测点是否有效的标识，分为很多种状态
  uint8_t InvalidStates_Valid : 1;

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
} radar_point_inner_struct;
#endif

// 跟踪目标（Object）状态结构体（空闲、临时、确认三种状态）
typedef enum {
  /*空闲状态*/
  TRACK_STATE_FREE = 0,
  /*临时状态 */
  TRACK_STATE_DETECTION,
  /*确认状态 */
  TRACK_STATE_ACTIVE,
} trackState_enum;

typedef enum {
  ALL_VISIBLE = 0,
  ONE_POINT_INVISIBLE,
  TWO_POINT_INVISIBLE,
  THREE_POINT_INVISIBLE,
  FOUR_POINT_INVISIBLE,
  ALL_IN_POSITIVE_SIDE,
  ALL_IN_NEGATIVE_SIDE,
  PASS_THROUGH_LAT,
  HAS_NEGATIVE_LONG,
  RESERVED_STATUS
} corner_type_enum;

typedef struct matchInfo_ {
  uint16_t cluster_Idx;
  float weight;
  bool used4Fit;

  double prob_exist;
} matchInfo_t;

typedef struct fittingInfo_ {
  uint8_t validNum = 0;
  float clutter_num = 0;

  VectorXd mCenter = VectorXd(MSIZE);

  std::vector<uint16_t> measIdx;

  float fit_Wid;
  float fit_Len;

} fitInfo_struct;

typedef struct {
  float xpos;
  float ypos;
} XY_Pos_t;

typedef struct assoDet_ {
  std::vector<matchInfo_t> detInfo;

  // 目标当前帧关联上的量测点数
  uint16_t associateNum;
  uint16_t associateNumThr;
  uint16_t HighQuaPointNum;

  uint8_t staticNum = 0;
  float cluster_Wid;
  float cluster_Len;
  float dopplerVari;
  float sum_weight;

  double max_rcs_filter_val = 0;

  double x_diff_dir = 1.0;
  double y_diff_dir = 1.0;
  double v_diff_dir = 1.0;
} Measurement_struct;

typedef struct kalmanInfo_ {
  // 目标估计状态向量（2D: x y vx vy ax ay or 3D: x y z vx vy vz ax ay az）
  VectorXd StateEst = VectorXd(SSIZE);

  // 目标估计状态向量（2D: x y vx vy ax ay or 3D: x y z vx vy vz ax ay az）
  VectorXd StateEst2 = VectorXd(SSIZE);

  // 目标预测状态向量（2D: x y vx vy ax ay or 3D: x y z vx vy vz ax ay az）
  VectorXd StatePre = VectorXd(SSIZE);

  // 目标在极坐标下的状态向量
  VectorXd MeasPre = VectorXd(MSIZE);

  // 目标量测预测
  VectorXd Z_Pred = VectorXd(MSIZE);

  // 目标滤波状态协方差
  MatrixXd P_ = MatrixXd(SSIZE, SSIZE);

  // 目标滤波增益
  MatrixXd K_gain = MatrixXd(SSIZE, MSIZE);

  // 目标滤波新息协方差矩阵
  MatrixXd S_out = MatrixXd(MSIZE, MSIZE);
  MatrixXd S_inv = MatrixXd(MSIZE, MSIZE);

  // 量测的雅可比矩阵
  MatrixXd JacMat = MatrixXd(MSIZE, SSIZE);

  MatrixXd Q_ = MatrixXd(SSIZE, SSIZE);

  MatrixXd R_ = MatrixXd(MSIZE, MSIZE);

  // “绝对”状态信息
  VectorXd StateEst_abs = VectorXd(SSIZE);

  // ukf--sigma point
  MatrixXd sigmaX = MatrixXd(SSIZE, 2 * SSIZE + 1);

  MatrixXd sigmaX2 = MatrixXd(SSIZE, 2 * SSIZE + 1);
} kalmanInfo_struct;

typedef struct trace_extendInfo_ {
  float ProbOfExist;  // 目标存在概率
  float prob_exist_output;

  float CornerPos[8 + 1][2];  // 航迹角点坐标信息（索引8为航迹中心位置）

  float box_theta;  // 目标朝向角
  float Orin_For_Display;
  float Length = 0.0f;
  float Width = 0.0f;
  float height = 0.0F;

  float max_len;
  float max_wid;

  uint16_t corner_type = 0;  // corner_type_enum::RESERVED_STATUS;

  // 航迹运动状态切换相关变量
  uint16_t UpCount = 0;
  uint16_t DownCount = 0;
  ObjectType_enum Object_Class = ObjectType_enum::UNKNOWN;  // 航迹目标类型

  uint8_t move2stop;
  uint8_t stop2move;
  ObjectDynProp_enum DynProp = unknown;  // 航迹运动状态

} ExtendInfo_struct;

typedef struct random_matrice_ {
  MatrixXd RMM_shape = MatrixXd(2, 2);
} Random_matrice_t;

typedef struct manage_struct {
  uint16_t detect2TrackingCount;  // 目标临时状态计数
  uint16_t detect2missCount;      // 目标临时状态时无关联情况计数
  uint16_t tracking2missCount;    // 目标确认状态时无关联情况计数

  uint16_t HighProCount;

  uint16_t NM_SUM;
  uint32_t MN_Logic_1;
  uint32_t MN_Logic_2;

  double trace_belief;
} manage_struct;


typedef struct shape_struct
{
    std::deque<double> len_fifo, wid_fifo, height_fifo, theta_fifo;
    double P_[4];

    Eigen::Vector4d X;
    Eigen::Matrix4d P;
}shape_struct;

// 跟踪目标（Object）信息结构体
typedef struct trackTable_ {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  uint16_t trackID;  // 目标ID号
  double birth_timestamp;
  double latest_tracked_time;
  uint32_t TCount;  // 目标生命计数

  manage_struct ManageInfo;
  trackState_enum trackState;    // 目标所处状态
  kalmanInfo_struct KalmanInfo;  // 滤波器过程相关变量
  Measurement_struct MeasInfo;   // 本周期关联到的量测信息
  ExtendInfo_struct ExtendInfo;  // 航迹扩展信息：长宽、航向角 目标类型等等信息

  Random_matrice_t RMM;

  shape_struct shape_status;

  // QT debug使用
  bool chosed = false;

} trackTable_strcut;

typedef enum {
  Forward_ = 0,
  Reverse_,
} gear_enum_t;

typedef struct {
  double timestamp;
  double x_seq;
  double y_seq;
  double heading;
  double yaw_seq;
  double vx;
  double yaw_rate;

  double Vlat;
  double Vlong;
  gear_enum_t gear;

   bool yawRateValid = false;
  uint32_t highYawRateCount = 0;
} vehicleInfo_struct;

typedef struct {
  std::vector<uint16_t> staticIDX;
} staticMap_t;

// 惯导信息
typedef struct {
  double TimeStamp;
  float X;
  float Y;
  float Z;
  float Vx;
  float Vy;
  float Vz;
  float YawAngle;
  float YawRate;

  float UTM_mat[4 * 4];
} GNSS_Info_t;

typedef struct {
  uint32_t occupy_age;
  uint32_t free_age;
  uint8_t mean_static_num;
  uint8_t cur_num;
  uint8_t NotFreeFlag;
} freesSpace_t;

typedef struct {
  uint16_t trk_ID;
  float covari[3];
  double eigen_val[2];
} trace_his_t;

#define STATIC_TRACK_MAXNUM (256)

typedef struct point_ {
  float latPos;
  float longPos;
  float latV;
  float longV;
} point_t;

typedef struct kf_info_ {
  Eigen::VectorXd X_ = Eigen::VectorXd(SSIZE);
  Eigen::VectorXd X_no_comp = Eigen::VectorXd(SSIZE);
  Eigen::MatrixXd P_ = Eigen::MatrixXd(SSIZE, SSIZE);
} kf_info_t;

typedef enum
{
    EMPTY = 0,
    UNACTIVATE,
    ACTIVATE,
}static_trace_status_enum;

typedef struct gridTrack_ {
  uint32_t ID;
  double birth_timestamp;
  double latest_tracked_time;
  static_trace_status_enum status;
  uint32_t age;
  uint16_t NMLogic;
  uint8_t UnAssoNum;
  point_t posInfo;

  double wid;
  double len;
  double height;

  double ProbOfExist;
  double Prob_Output;
  double max_prob;
  double High_Coun;

  double diff_X;
  double diff_Y;
  double diff_V;

  uint32_t dyn_num;

  kf_info_t kf_info;

  shape_struct shape_status;
} gridTrack_t;


typedef enum {
    DYNAMIC_BOX = 0,
    STATIC_BOX,
}box_dyn_static_enum;

typedef enum {
    ACTIVE_DYNAMIC_TRACK = 0,
    UNACTIVE_DYNAMIC_TRACK,
    ACTIVE_STATIC_TRACK,
    UNACTIVE_STATIC_TRACK,
    RESERVED_TRACK,
}assigned_obj_enum;

typedef enum
{
   VALID = 0,
   HAS_MATCHED,
   NOT_USED_FOR_INIT,
}box_status_enum;

typedef struct det_box_ {
  float wid;
  float len;
  float x_pos;
  float y_pos;

  float v_mean;

  uint8_t valid;

  uint16_t matched_trace_idx;
  box_dyn_static_enum box_feature;
  assigned_obj_enum assigned_obj;

  uint8_t member_num;
  float iou_ = 0.0;

} det_box_t;

typedef struct {
  double DistLat, DistLong, VrelLong;
  uint32_t invalid = 0;
} simple_point_t;

typedef struct {
  vehicleInfo_struct vehicleInfo;
  std::vector<RadarMeasure_struct> point_set;
} static_point_snapshot_t;




#define STATIC_RANGE_LAT (30)
#define STATIC_RANGE_LONG (50)

#define GRID_LAT_MAX (30.0)
#define GRID_LAT_MIN (-30.0)
#define GRID_LAT_STEP (2.0)
#define GRID_LONG_MAX (100.0)
#define GRID_LONG_STEP (2.0)

#define GRID_HALF_LAT (uint8_t(GRID_LAT_MAX / GRID_LAT_STEP))
#define GRID_LAT_SIZE (uint8_t((GRID_LAT_MAX - GRID_LAT_MIN) / GRID_LAT_STEP))
#define GRID_LONG_SIZE (uint8_t(GRID_LONG_MAX / GRID_LONG_STEP))

#endif
