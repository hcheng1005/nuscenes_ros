#ifndef KALMAN_H
#define KALMAN_H

#include <iostream>

#include "Eigen/Core"
#include "Eigen/Dense"
#include "cmath"
#include "vector"
#include "deque"

#include "iou.h"

/*
  X = [bbox.x, bbox.y, bbox.z, bbox.o, bbox.l, bbox.w, bbox.h, vx, vy, vz,
  ax,ay, az] Z = [bbox.x, bbox.y, bbox.z, bbox.o, bbox.l, bbox.w, bbox.h]
*/
// bool TRACK_DEBUG = true;

#define X_DIM (13)
#define Z_DIM (7)

#define iDisLong (0)
#define iDisLat (1)
#define iDisHeight (2)
#define iBoxHeading (3)
#define iBoxLen (4)
#define iBoxWid (5)
#define iBoxHeight (6)
#define iVreLong (7)
#define iVreLat (8)
#define iVreHeight (9)
#define iAccLong (10)
#define iAccLat (11)
#define iAccHeight (12)

typedef enum {
  TRK_Invalid = 0,
  TRK_Detected,
  TRK_Confirmed,
  TRK_Delete
} trace_status_enum;

typedef enum {
  UNKNOWN = 0,
  VEHICLE = 1,
  PEDESTRIAN = 2,
  BARRIAR = 3,
  TRAFFICLIGHT = 4,
  TRAFFICCONE = 5,
  BICYCLE = 6,
} inner_type_enum;

typedef struct {
  double len;
  double wid;
  double height = 100.0;
  double theta = 0.01;  // [-PI, PI]
} shape4out_t;

typedef struct {
  double len_max = 0.0;
  double wid_max = 0.0;
} shape_manage_t;

typedef enum{
  Unknow_motion = 0,
  stationary_motion,
  moving_motion,
  oncoming_motion, // 靠近
  away_motion, // 
  stop_motion
}motion_enum;


typedef struct {
  int change_type_count = 0;
  double score = 0.0;
  inner_type_enum type = UNKNOWN;
  inner_type_enum type_new = UNKNOWN;
}obj_type_manage_t;


typedef struct {
  uint32_t id = 0;
  uint32_t age = 0;
  double exsit_prob = 0.0;
  uint8_t continue_assigned_count = 0;
  uint8_t unassigned_count = 0;
  trace_status_enum track_status = TRK_Invalid;

  obj_type_manage_t type_manage;

  uint32_t motion_diff = 0;
  uint32_t same_motion = 0;
  motion_enum motion_status_last = Unknow_motion;
  motion_enum motion_status = Unknow_motion;

  double birth_time = 0.0;
  double tracking_time = 0.0;
  double latest_tracked_time = 0.0;

  double diff_x = 0.0;
  double diff_y = 0.0;
  double diff_theta = 0.0;

  shape4out_t shape4out;
  shape_manage_t shape_mana; 

} trace_manage_struct;

typedef struct {
  double ad_Q_long = 0.0;
  double ad_Q_lat = 0.0;

  double neg_count_lat = 0.0;
  double neg_count_long = 0.0;

  int count_long = 0;
  int count_lat = 0;

}trace_adaptive_Q_t;


typedef struct box_t {
  uint32_t id;
  bool reues_ = true;
  bool valid = true;
  bool dust_ = false;
  uint16_t iou_count = 0;
  rect_basic_struct rect;  // used for IOU
  double score = 0.0;
  inner_type_enum type;
} box_t;

typedef struct {
  uint32_t trace_idx;
  uint32_t det_idx;
  double trace_box_s;
  double det_box_s;
  double center_dis;
  double box_overlap;
  double iou;
} greedy_match_info_t;

typedef enum
{
  no_matched = 0,
  normal_case,
  match_but_not_used,
}matched_enum;

typedef struct
{
  matched_enum matched_case;
  std::vector<box_t> matched_det_list;
  double sum_IOU = 0.0;
  bool has_high_meas = false;
}matched_info_struct;


typedef struct{
  uint32_t motion_keepCoun = 0;
  double yawRate;
  bool vehicle_flag = false;
}vehilce_info_t;

typedef struct
{
  double len_, wid_;
  double theta_1, theta_2, diff_theta;
  double x_, y_;
  // Eigen::VectorXd LS_X = Eigen::VectorXd(X_DIM);
}lshape_info_t;

class simple_tracker {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  simple_tracker(Eigen::VectorXd X, Eigen::MatrixXd P, uint32_t id);
  simple_tracker(box_t &new_det, uint32_t id);

  virtual ~simple_tracker() {}

  void trace_predict(double diff_time);
  void Update_Q(void);
  void Predict_(Eigen::VectorXd &X_, Eigen::MatrixXd &P_, Eigen::MatrixXd F_,
                Eigen::MatrixXd Q_);
  void Update_();
  void trace_update_with_det();
  void trace_update_without_det();
  Eigen::VectorXd compute_residual(Eigen::VectorXd measZ, Eigen::VectorXd preZ,
                                   bool &change_heading);

  void compute_extended_info_vari();
  void ComputeMeanAndVari(const std::deque<double> dataArray, double *mean_val,
                          double *vari_val);
  Eigen::VectorXd compute_closet_point(Eigen::VectorXd input,
                                       uint8_t &closest_idx);

  double value_limit(double lower_val, double upper_val, double value);
  bool IsAnyPartInSpecialArea(void);

  double diff_t = 0.1;

  Eigen::VectorXd X_ = Eigen::VectorXd(X_DIM);
  Eigen::VectorXd X_cp = Eigen::VectorXd(X_DIM);
  Eigen::MatrixXd P_ = Eigen::MatrixXd(X_DIM, X_DIM);
  Eigen::MatrixXd F_ = Eigen::MatrixXd(X_DIM, X_DIM);
  Eigen::MatrixXd H_ = Eigen::MatrixXd(Z_DIM, X_DIM);
  Eigen::Vector3d abs_Vel;
  bool in_near_area = false;

  double theta_ = 0.0;

  trace_manage_struct track_manage;
  trace_adaptive_Q_t adaptive_Q;
  // std::vector<box_t> matched_det_list;
  matched_info_struct matched_info;

  std::deque<double> len_fifo, wid_fifo, height_fifo, theta_fifo;
  double shape_vari[4];

  vehilce_info_t vehilce_info;

 private:
  std::deque<double> lat_res, long_res;
  Eigen::MatrixXd Q_ = Eigen::MatrixXd::Identity(X_DIM, X_DIM) * 1.0;
  Eigen::MatrixXd R_ = Eigen::MatrixXd::Identity(Z_DIM, Z_DIM) * 2.0;

  lshape_info_t lshape_info;

  void compute_newZ(Eigen::VectorXd &new_Z);
  void compute_residual2(Eigen::VectorXd &measZ);
  void adptive_noise(Eigen::MatrixXd residual_final);
  bool measure_valid(const Eigen::VectorXd &X_, const Eigen::VectorXd &new_Z, const Eigen::VectorXd &residual_final);
  void l_shape_tracker(Eigen::VectorXd measZ);
  void center_tracker(Eigen::VectorXd measZ);
  void tracker_postprocess(void);
  void compute_F(const double dt);
};

#endif