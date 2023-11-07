/*
 * @Description:
 * @version:
 * @Author: ChengHao
 * @Date: 2022-10-12 08:58:47
 * @LastEditors: ChengHao hao.cheng@wuzheng.com
 * @LastEditTime: 2023-11-07 14:26:05
 */
#ifndef TRACK_STATIC_TRACE_H
#define TRACK_STATIC_TRACE_H

#include "dataStruct.h"
#include "common/DBSCAN.h"
#include "commonfunctions.h"



typedef std::pair<std::pair<uint32_t, uint32_t>, double> cost_pair;

bool run_static_filter(void);

void trace_meas_dbscan(std::vector<RadarMeasure_struct> &point_fifo,
                       std::vector<std::vector<uint16_t>> &clusterSet);

void kf_predict(Eigen::VectorXd &X, Eigen::MatrixXd &P, Eigen::MatrixXd F, Eigen::MatrixXd F_T, Eigen::MatrixXd Q_);

double kf_update(Eigen::VectorXd &X, Eigen::MatrixXd &P, \
                 Eigen::VectorXd preZ, VectorXd measZ, double weight, gridTrack_t &trace);

bool trace_assign_with_point(gridTrack_t &trace, std::vector<RadarMeasure_struct> &point_fifo, box_t &new_box);
Eigen::MatrixXd compute_jacMat(gridTrack_t &trace);
void trace_abnormal_trace_delete(gridTrack_t *track_list);
void trace_compare_with_dyn_trace(trackTable_strcut *dyn_track_list, gridTrack_t *static_track_list);

void fifo_point_proc(RadarOutput_Struct *indata, std::vector<RadarMeasure_struct> &fifo_point, vehicleInfo_struct &vehicleInfo);
std::vector<RadarMeasure_struct> trans_point(std::deque<static_point_snapshot_t> &fifo_set);

void compute_box_nearest_pos(const double x_pos, const double y_pos, const double len, const double wid, \
                             double *new_x_pos, double *new_y_pos);
double compute_max_angle(const double x_pos, const double y_pos, const double len, const double wid);
double compute_min_angle(const double x_pos, const double y_pos,
                         const double len, const double wid);
void trace_abnormal_trace_delete2(gridTrack_t *trace_list, vehicleInfo_struct &vehicleInfo);


void creat_valid_invalid_trace_list(gridTrack_t *track_list,
                                    std::vector<uint8_t> &valid_empty_list,
                                    std::vector<uint8_t> &trace_list);

void static_trace_assign_with_box(gridTrack_t *static_track_list, std::vector<det_box_t> &det_box_list, std::vector<std::vector<uint16_t> > &dyn_trace_assign_list);

void track_predict_static(gridTrack_t *track_list, vehicleInfo_struct *vehicleInfo, double delta_t);
void creat_det_box(std::vector<std::vector<uint16_t>> &clusterSet,
                    std::vector<RadarMeasure_struct> &point_fifo,
                    std::vector<det_box_t> &det_box_list, box_dyn_static_enum box_feature);

void creat_cost_matric(gridTrack_t *track_list,
                       const std::vector<uint8_t> &valid_trace_list,
                       const std::vector<det_box_t> &det_box_list,
                       std::vector<cost_pair> &cost_matrix);

void static_trace_update(gridTrack_t *track_list,
                         const std::vector<uint8_t> &valid_trace_list,
                         std::vector<det_box_t> &det_box_list,
                         const std::vector<std::vector<uint16_t> > &all_assigned_box);

void creat_new_static_trace(gridTrack_t *track_list, std::vector<det_box_t> &det_box_list, std::vector<uint8_t> valid_empty_list);

void static_trace_to_dynamic(trackTable_strcut *dyn_track_list,
                             gridTrack_t *static_track_list, vehicleInfo_struct &vehicleInfo);

void change_static2dynamic_trace(trackTable_strcut *trace,
                                 gridTrack_t *static_track_list);

void extended_info_filter(double new_len, double new_wid, double new_theta,
                               gridTrack_t &trace);
void static_ComputeMeanAndVari(const std::deque<double> dataArray, double *mean_val, double *vari_val);
void compute_predictZ_and_measZ(gridTrack_t &trace, box_t &new_box, VectorXd &preZ, VectorXd &measZ);
void static_trace_update_prob(gridTrack_t &trace, box_t &track_bbox, box_t &det_box);
void static_trace_manag(gridTrack_t &subTrace, const bool assigned_flag);
#endif  // TRACK_STATIC_TRACE_H
