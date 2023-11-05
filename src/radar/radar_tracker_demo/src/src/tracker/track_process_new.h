/*
 * @Description:
 * @version:
 * @Author: ChengHao
 * @Date: 2022-10-12 08:58:47
 * @LastEditors: CharlesCH hcheng1005@gmail.com
 * @LastEditTime: 2023-11-05 11:24:13
 */
#ifndef TRACK_PROCESS_NEW_H
#define TRACK_PROCESS_NEW_H

#include <string>
#include <vector>
#include <cmath>
#include <iostream>

#include "commonlib/DBSCAN.h"
#include "commonfunctions.h"
#include "commonlib/DBSCAN.h"
#include "dataStruct.h"
#include "filter.h"
#include "match.h"
#include "preprocess.h"
#include "track_manage.h"
#include "track_filter.h"
#include "track_static_trace.h"

typedef struct
{
    uint32_t trace_idx;
    assigned_obj_enum track_type;
} orin_trace_info_t;


typedef enum
{
    ANGLE_NULLSET = 0,
    ANGLE_CROSS,
    ANGLE_OVERLAP,
}angle_enum;


void track_process(trackTable_strcut *trackInfo,
                   gridTrack_t *static_track_list,
                   RadarOutput_Struct *indata,
                   std::vector<RadarMeasure_struct> &fifo_point,
                   std::vector<vehicleInfo_struct> &vehicleInfo,
                   Tracker *Tracker_, double delta_t);

void prepare_point_data( RadarOutput_Struct *indata,
            std::vector<RadarMeasure_struct> &fifo_point,
            std::vector<vehicleInfo_struct> &vehicleInfo);

void trace_predict_all(trackTable_strcut *dyn_track_list,
                      gridTrack_t *static_track_list,
                      std::vector<vehicleInfo_struct> &vehicleInfo,
                      Tracker *Tracker_, double delta_t);

void trace_assign_with_box(trackTable_strcut *dyn_track_list,
                           gridTrack_t *static_track_list,
                           std::vector<RadarMeasure_struct> &fifo_point, std::vector<vehicleInfo_struct> &vehicleInfo);

void point_cluster_and_fitting(std::vector<RadarMeasure_struct> &fifo_point,
                           std::vector<std::vector<uint16_t>> &clusterSet_all,
                            std::vector<det_box_t> &det_box_list_all, std::vector<vehicleInfo_struct> &vehicleInfo);

void dyn_trace_update_and_manage(trackTable_strcut *dyn_track_list,
                                 std::vector<RadarMeasure_struct> &fifo_point,
                                 std::vector<vehicleInfo_struct> &vehicleInfo,
                                 Tracker *Tracker_);

void All_Trace_Mage_Proc(trackTable_strcut *dyn_track_list, gridTrack_t *static_track_list);


void dynamic_trace_assign_with_dynamic_box(trackTable_strcut *dyn_track_list, \
                                           std::vector<det_box_t> &det_box_list, \
                                           std::vector<std::vector<uint16_t>> &all_assigned_box);

void compute_trace_num(trackTable_strcut *dyn_track_list,
                       std::vector<uint16_t> &TraceSet);

void compute_meas_num(std::vector<RadarMeasure_struct> &fifo_point,
                      std::vector<uint16_t> &MeasSet);

void creat_trace_box(trackTable_strcut *dyn_track_list,
                     gridTrack_t *static_track_list,
                     std::vector<orin_trace_info_t> &trace_info_list_all);

void match_alg(trackTable_strcut *dyn_track_list,
               gridTrack_t *static_track_list,
               std::vector<orin_trace_info_t> &trace_info_list,
               std::vector<det_box_t> &det_box_list_all, std::vector<std::vector<uint16_t> > &all_assigned_box);

void static_trace_update_and_merge(gridTrack_t *static_track_list,
                         std::vector<det_box_t> &det_box_list_all,
                         const std::vector<std::vector<uint16_t>> &all_assigned_box);

void dynamic_trace_update(std::vector<RadarMeasure_struct> &fifo_point,
                          trackTable_strcut *dyn_track_list,
                          const std::vector<std::vector<uint16_t>> &clusterSet_static, std::vector<det_box_t> &det_box_list,
                          std::vector<std::vector<uint16_t>> &all_assigned_box);

void clear_matched_point(std::vector<RadarMeasure_struct> &fifo_point,
                         std::vector<std::vector<uint16_t>> &clusterSet_all,
                         std::vector<det_box_t> &det_box_list_all);

void new_trace_manage_test(trackTable_strcut *dyn_track_list, gridTrack_t *static_track_list);

bool is_multi_bounce(trackTable_strcut *trace1, trackTable_strcut *trace2);
void compare_between_dynamic_trace(trackTable_strcut *dynamic_trace_1, trackTable_strcut *dynamic_trace_2);

void view_grid_test(trackTable_strcut *dyn_track_list, gridTrack_t *static_track_list);
void compute_trace_anlge(double x_pos, double y_pos, double half_len, double half_wid, double theta, double *angle_min, double *angle_max);

bool is_dynamic_trace(uint32_t trace_idx);

bool compare_static_trace(gridTrack_t *slaver_trace, gridTrack_t *master_trace);
bool compare_static_trace_with_dyntrace(gridTrack_t *static_trace, trackTable_strcut *dynamic_trace);

angle_enum compute_two_angle_position(std::pair<double, double> angle1, std::pair<double, double> angle2);
double compute_two_angle_iou(std::pair<double, double> angle1, std::pair<double, double> angle2);

void updateVehicle(vehicleInfo_struct &vehicleInfo);

void free_mem(trackTable_strcut *dyn_track_list);
#endif
