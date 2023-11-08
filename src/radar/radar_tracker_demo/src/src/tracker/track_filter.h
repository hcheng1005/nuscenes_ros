/*
 * @Description:
 * @version:
 * @Author: ChengHao
 * @Date: 2022-10-12 08:58:47
 * @LastEditors: ChengHao hao.cheng@wuzheng.com
 * @LastEditTime: 2023-06-13 18:53:11
 */
#ifndef TRACK_FILTER_H
#define TRACK_FILTER_H

#include "dataStruct.h"
#include "filter.h"

#define MOTION_CHANGE_GATE_INIT (8)
#define MOTION_CHANGE_GATE (2 * MOTION_CHANGE_GATE_INIT)

void track_Predict(trackTable_strcut *trackInfo,
                   vehicleInfo_struct *vehicleInfo, Tracker *Tracker_,
                   float delta_t);
void track_association(std::vector<RadarMeasure_struct> &global_point,
                       trackTable_strcut *trackInfo);
void track_update(std::vector<RadarMeasure_struct> &global_point,
                  trackTable_strcut *trackInfo, Tracker *Tracker_);
void track_kinematic_update(std::vector<RadarMeasure_struct> &global_point,
                            trackTable_strcut *trace_list, Tracker *Tracker_);
void track_compute_Q(trackTable_strcut *trace, float delta_t,
                     vehicleInfo_struct *vehicleInfo);
void track_compute_R(trackTable_strcut *trace, vehicleInfo_struct *vehicleInfo);
void track_extend_info_update(std::vector<RadarMeasure_struct> &global_point,
                              trackTable_strcut *trace_list,
                              vehicleInfo_struct *vehicleInfo);
void Track_Update_TrkType(std::vector<RadarMeasure_struct> &global_point,
                          trackTable_strcut *trace);
void track_compute_newshape(std::vector<RadarMeasure_struct> &global_point,
                            trackTable_strcut *trackInfo, float *newLen,
                            float *newWid);
void track_update_shape(std::vector<RadarMeasure_struct> &global_point,
                        trackTable_strcut *trace);
void track_classifity(trackTable_strcut *trace);
void track_shapeparalimit(trackTable_strcut *trace);
void Track_Update_headingAng(trackTable_strcut *trace,
                             std::vector<RadarMeasure_struct> &global_point,
                             vehicleInfo_struct *vehicleInfo);
void Track_Update_Corners(trackTable_strcut *trace);
void Track_Update_DynPro(trackTable_strcut *trace);
double trace_v_covariance(trackTable_strcut *trace);
void track_compute_abs_velocity(trackTable_strcut *trace,
                                vehicleInfo_struct *vehicleInfo);

void UpdateSurvivalProb(trackTable_strcut *trace,
                        std::vector<RadarMeasure_struct> &global_point,
                        vehicleInfo_struct *vehicleInfo);

void compute_extended_info_vari(std::vector<RadarMeasure_struct> &global_point,
                                trackTable_strcut *trace);
#endif  // TRACK_FILTER_H
