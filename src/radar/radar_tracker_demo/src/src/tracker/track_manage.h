/*
 * @Description:
 * @version:
 * @Author: ChengHao
 * @Date: 2022-10-14 15:16:17
 * @LastEditors: ChengHao hao.cheng@wuzheng.com
 * @LastEditTime: 2023-11-07 14:25:58
 */
#ifndef TRACK_MANAGE_H
#define TRACK_MANAGE_H

#include "common/DBSCAN.h"
#include "dataStruct.h"
#include "filter.h"

void track_Manage(trackTable_strcut *trackInfo,
                  vehicleInfo_struct *vehicleInfo);
void track_associate_PointThr(trackTable_strcut *trackInfo);
void track_manage_confirmedtrace(trackTable_strcut *trace);
void track_manage_unconfirmedtrace(trackTable_strcut *trace,
                                   vehicleInfo_struct *vehicleInfo);

bool CompareWithConfirmTrk(trackTable_strcut *trackInfo,
                           trackTable_strcut *trace);
void track_clear(trackTable_strcut *trace);
void track_birth(std::vector<RadarMeasure_struct> &global_point,
                 trackTable_strcut *trackInfo, vehicleInfo_struct *vehicleInfo);
void BulidPointSet4DBSCAN(std::vector<RadarMeasure_struct> &global_point,
                          std::vector<DBSCAN::Point4DBSCAN> *PointSet);
uint8_t track_returnNewTrkId(trackTable_strcut *trackInfo);
void track_InitTrack(trackTable_strcut *trackInfo, uint16_t valid_trace_idx,
                     fitInfo_struct *fitInfo,
                     std::vector<RadarMeasure_struct> &global_point);

bool track_fittingCluster(std::vector<RadarMeasure_struct> &global_point,
                          std::vector<DBSCAN::Point4DBSCAN> &PointSet,
                          std::vector<uint16_t> &clusterPtr,
                          fitInfo_struct *fitInfo);
bool track_IsFakeTrace(trackTable_strcut *trackInfo, trackTable_strcut *trace);
void Track_Init_Corners(trackTable_strcut *trace, bool extendFlag);

void track_merge2(trackTable_strcut *trackInfo);

void trace_merge_newtrace(trackTable_strcut *trace_old,
                          trackTable_strcut *trace_young);

#endif  // TRACK_MANAGE_H
