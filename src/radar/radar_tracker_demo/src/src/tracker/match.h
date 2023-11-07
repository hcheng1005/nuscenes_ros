/*
 * @Descripttion:
 * @version:
 * @Author: ChengHAO
 * @Date: 2022-09-05 13:20:02
 * @LastEditors: ChengHao hao.cheng@wuzheng.com
 * @LastEditTime: 2023-11-07 14:25:49
 */
#ifndef MATCH_H
#define MATCH_H

#include "dataStruct.h"
#include "commonfunctions.h"
#include "common/DBSCAN.h"

typedef std::pair<std::pair<int, int>, std::pair<double, double>> trace_meas_pair;

typedef struct
{
    uint32_t trace_idx;
    uint32_t det_idx;
    double trace_box_s;
    double det_box_s;
    double center_dis;
    double box_overlap;
    double iou;
    double iou_2;
}greedy_match_info_t;

void BulidPointSet4DBSCAN2(std::vector<RadarMeasure_struct> &fifo_point, std::vector<std::vector<uint16_t> > &clusterSet, std::vector<vehicleInfo_struct> &vehicleInfo);
double ComputeScaleOfEllipse(trackTable_strcut *trace, double dis_lat, double dis_long);
void compute_new_ellis_mat(const trackTable_strcut *trace, Eigen::Matrix2d &new_rmm_mat, double scale);
void sort_score(std::vector<greedy_match_info_t> &cost_matrix);
bool my_compare(const greedy_match_info_t &c1, const greedy_match_info_t &c2);

void creat_rect(trackTable_strcut *trackInfo,
                std::vector<uint16_t> &TraceSet,
                std::vector<det_box_t> &det_box_list,
                std::vector<rect_point_struct> &trace_box_list,
                std::vector<rect_point_struct> &det_rect_box_list);

void compute_cost_matrix(trackTable_strcut *trackInfo,
                           std::vector<uint16_t> &TraceSet,
                           const std::vector<det_box_t> &det_box_list,
                           const std::vector<rect_point_struct> &trace_box_list,
                           const std::vector<rect_point_struct> &det_rect_box_list,
                           std::vector<trace_meas_pair> &cost_matrix);

void dynamic_trace_assigned_with_point(std::vector<RadarMeasure_struct> &fifo_point,
                trackTable_strcut *trackInfo,
                const std::vector<uint16_t> &TraceSet,
                const std::vector<std::vector<uint16_t>> all_assigned_box,
                const std::vector<std::vector<uint16_t>> &clusterSet, const std::vector<det_box_t> &det_box_list);

#endif
