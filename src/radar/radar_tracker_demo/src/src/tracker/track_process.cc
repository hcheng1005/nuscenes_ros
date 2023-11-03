/*
 * @description:
 * @version:
 * @Author: ChengHao
 * @Date: 2022-10-08 11:43:18
 * @LastEditors: chenghao hao.cheng@wuzheng.com
 * @LastEditTime: 2023-08-30 10:53:52
 */
#include "track_process.h"

#include <cmath>
#include <iostream>

#ifdef QT_ENV_
extern std::vector<std::vector<uint16_t>> static_dbscan_result;
extern std::vector<DBSCAN::Point4DBSCAN> PointSet_result;
#endif

extern double RADAR_MOUNT_X, RADAR_MOUNT_Y;

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * @name: track_process
 * @description: 毫米波雷达算法入口
 * @param {RadarOutput_Struct} *indata ：本次毫米波雷达原始点云信息
 * @param {trackTable_strcut} *trackInfo：航迹列表
 * @param {std::vector<vehicleInfo_struct>} vehicleInfo ：车体信息
 * @param {Tracker} *Tracker_
 * @param {float} delta_t：帧间间隔时间
 * @return {*}
 */
void track_process(trackTable_strcut *dyn_track_list,
                   gridTrack_t *static_track_list,
                   RadarOutput_Struct *indata,
                   std::vector<RadarMeasure_struct> &fifo_point,
                   std::vector<vehicleInfo_struct> &vehicleInfo,
                   Tracker *Tracker_, double delta_t)
{
    RuntimeAnalyze("Init");

    /* 点云预处理 */
    prepare_point_data(indata, fifo_point, vehicleInfo);
    RuntimeAnalyze("prepare_point_data");

    /* 所有航迹状态预测 */
    trace_predict_all(dyn_track_list, static_track_list, vehicleInfo, Tracker_, delta_t);
    RuntimeAnalyze("trace_predict_all");

    /* 点云聚类、匹配 */
    trace_assign_with_box(dyn_track_list, static_track_list, fifo_point, vehicleInfo);
    RuntimeAnalyze("trace_assign_with_box");

    /* 动航迹更新和航迹管理 */
    dyn_trace_update_and_manage(dyn_track_list, fifo_point, vehicleInfo, Tracker_);
    RuntimeAnalyze("dyn_trace_update_and_manage");

    /* 静止航迹转变为动态航迹 */
    static_trace_to_dynamic(dyn_track_list, static_track_list, vehicleInfo[0]);
    RuntimeAnalyze("static_trace_to_dynamic");

    /* 虚假目标识别与删除 */
    All_Trace_Mage_Proc(dyn_track_list, static_track_list);
    RuntimeAnalyze("All_Trace_Mage_Proc");
    
    //
    free_mem(dyn_track_list);

    RuntimeAnalyze("END");
}

/**
 * @name:
 * @description:
 * @param {RadarOutput_Struct} *indata
 * @param {  } std
 * @param {  } std
 * @return {*}
 */
void prepare_point_data(RadarOutput_Struct *indata,
                        std::vector<RadarMeasure_struct> &fifo_point,
                        std::vector<vehicleInfo_struct> &vehicleInfo)
{
    /* 量测预处理: 识别动静目标 */
    perprocess(*indata, vehicleInfo);

    /* FIFO数据处理以及构建gridmap */
    fifo_point_proc(indata, fifo_point, vehicleInfo[0]);

    //车辆工况处理
    updateVehicle(vehicleInfo[0]);
}

/**
 * @name: updateVehicle
 * @description: 稳定yawRate
 * @param {vehicleInfo_struct} &vehicleInfo
 * @param {  } std
 * @param {  } std
 * @return {*}
 */
void updateVehicle(vehicleInfo_struct &vehicleInfo)
{
    if(fabs(vehicleInfo.yaw_rate) > (0.017 * 5))
    {
       if(vehicleInfo.highYawRateCount < 10)
       {
            vehicleInfo.highYawRateCount++;
       }
       else
       {
           vehicleInfo.yawRateValid = true;
       }
    }
    else
    {
        if(vehicleInfo.highYawRateCount)
        {
            vehicleInfo.highYawRateCount--;
        }
        else
        {
            vehicleInfo.yawRateValid = false;
        }
    }
}

/**
 * @name:
 * @description:
 * @param {trackTable_strcut} *dyn_track_list
 * @param {gridTrack_t} *static_track_list
 * @param {  } std
 * @param {Tracker} *Tracker_
 * @param {double} delta_t
 * @return {*}
 */
void trace_predict_all(trackTable_strcut *dyn_track_list,
                       gridTrack_t *static_track_list,
                       std::vector<vehicleInfo_struct> &vehicleInfo,
                       Tracker *Tracker_, double delta_t)
{
    /* 动态航迹预测 */
    track_Predict(dyn_track_list, &vehicleInfo[0], Tracker_, delta_t);

    /* 静态航迹预测 */
    track_predict_static(static_track_list, &vehicleInfo[0], delta_t);
}

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @param {vector<RadarMeasure_struct>} &fifo_point
 * @param {  } std
 * @param {  } std
 * @return {*}
 */
void point_cluster_and_fitting(std::vector<RadarMeasure_struct> &fifo_point,
                               std::vector<std::vector<uint16_t>> &clusterSet_all,
                               std::vector<det_box_t> &det_box_list_all,
                               std::vector<vehicleInfo_struct> &vehicleInfo)
{
    std::vector<std::vector<uint16_t>> clusterSet_static;
    std::vector<det_box_t> det_box_list_static;
    std::vector<std::vector<uint16_t>> clusterSet_dynamic;
    std::vector<det_box_t> det_box_list_dynamic;

    /* 分别进行动态点云聚类与静态点云聚类 */
    /* 静态点云聚类 */
    trace_meas_dbscan(fifo_point, clusterSet_static);
    creat_det_box(clusterSet_static, fifo_point, det_box_list_static, STATIC_BOX);

    /* 动态点云聚类 */
    BulidPointSet4DBSCAN2(fifo_point, clusterSet_dynamic, vehicleInfo);
    creat_det_box(clusterSet_dynamic, fifo_point, det_box_list_dynamic, DYNAMIC_BOX);

    /* 将动静box融合在一起后给静态航迹进行匹配关联 */
    clusterSet_all = clusterSet_static;
    for (auto &sub_cluster : clusterSet_dynamic)
    {
        clusterSet_all.push_back(sub_cluster);
    }

    det_box_list_all = det_box_list_static;
    for (auto &det_box : det_box_list_dynamic)
    {
        det_box_list_all.push_back(det_box);
    }

#ifdef QT_ENV_
    static_dbscan_result.clear();
    static_dbscan_result.shrink_to_fit();

    static_dbscan_result = clusterSet_static;

    for (auto temp : clusterSet_dynamic)
    {
        static_dbscan_result.push_back(temp);
    }
#endif
}

/**
 * @name:
 * @description:
 * @param {trackTable_strcut} *dyn_track_list
 * @param {gridTrack_t} *static_track_list
 * @param {  } std
 * @return {*}
 */
void trace_assign_with_box(trackTable_strcut *dyn_track_list,
                           gridTrack_t *static_track_list,
                           std::vector<RadarMeasure_struct> &fifo_point,
                           std::vector<vehicleInfo_struct> &vehicleInfo)
{
    std::vector<std::vector<uint16_t>> clusterSet_all;
    std::vector<det_box_t> det_box_list_all;
    std::vector<orin_trace_info_t> trace_info_list_all;
    std::vector<std::vector<uint16_t>> all_assigned_box;

    /* 聚类以及box拟合 */
    point_cluster_and_fitting(fifo_point, clusterSet_all, det_box_list_all, vehicleInfo);

    /* 构造航迹box */
    creat_trace_box(dyn_track_list, static_track_list, trace_info_list_all);

    /* 分配 */
    match_alg(dyn_track_list, static_track_list, trace_info_list_all, det_box_list_all, all_assigned_box);

    /* 清空被使用的点云 */
    clear_matched_point(fifo_point, clusterSet_all, det_box_list_all);

    /* 静态航迹更新 */
    static_trace_update_and_merge(static_track_list, det_box_list_all, all_assigned_box);

    /* 运动航迹更新 */
    dynamic_trace_update(fifo_point, dyn_track_list, clusterSet_all, det_box_list_all, all_assigned_box);
}

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @param {vector<RadarMeasure_struct>} &fifo_point
 * @param {  } std
 * @param {  } std
 * @return {*}
 */
void clear_matched_point(std::vector<RadarMeasure_struct> &fifo_point,
                         std::vector<std::vector<uint16_t>> &clusterSet_all,
                         std::vector<det_box_t> &det_box_list_all)
{
    /* 删除被静态航迹关联到的box所对应的原始点云 */
    for (uint8_t idx = 0; idx < clusterSet_all.size(); idx++)
    {
        if ((det_box_list_all.at(idx).valid & (1<<NOT_USED_FOR_INIT)) || \
            (det_box_list_all.at(idx).valid & (1<<HAS_MATCHED)))
        {
            for (const auto point_idx : clusterSet_all.at(idx))
            {
                fifo_point[point_idx].invalid |= (1 << notUed4Init_det);
            }
        }
    }
}

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @param {gridTrack_t} *static_track_list
 * @param {  } std
 * @param {  } std
 * @return {*}
 */
void static_trace_update_and_merge(gridTrack_t *static_track_list,
                                   std::vector<det_box_t> &det_box_list_all,
                                   const std::vector<std::vector<uint16_t>> &all_assigned_box)
{
    std::vector<uint8_t> valid_empty_list;
    std::vector<uint8_t> valid_trace_list;

    /* 统计静态航迹 */
    creat_valid_invalid_trace_list(static_track_list, valid_empty_list, valid_trace_list);

    /* 航迹更新 */
    static_trace_update(static_track_list, valid_trace_list, det_box_list_all, all_assigned_box);

    /* 建立新航迹 */
    creat_new_static_trace(static_track_list, det_box_list_all, valid_empty_list);
}

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @param {vector<RadarMeasure_struct>} &fifo_point
 * @param {trackTable_strcut} *dyn_track_list
 * @param {vector<std::vector<uint16_t>>} &clusterSet_static
 * @param {  } std
 * @param {  } std
 * @return {*}
 */
void dynamic_trace_update(std::vector<RadarMeasure_struct> &fifo_point,
                          trackTable_strcut *dyn_track_list,
                          const std::vector<std::vector<uint16_t>> &clusterSet_static,
                          std::vector<det_box_t> &det_box_list,
                          std::vector<std::vector<uint16_t>> &all_assigned_box)
{
    std::vector<uint16_t> TraceSet;

    compute_trace_num(dyn_track_list, TraceSet);

    dynamic_trace_assigned_with_point(fifo_point, dyn_track_list, TraceSet, all_assigned_box, clusterSet_static, det_box_list);
}

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @param {trackTable_strcut} *dyn_track_list
 * @param {gridTrack_t} *static_track_list
 * @param {  } std
 * @return {*}
 */
void creat_trace_box(trackTable_strcut *dyn_track_list,
                     gridTrack_t *static_track_list,
                     std::vector<orin_trace_info_t> &trace_info_list_all)
{
    /* 动航迹关联-匹配 */
    std::vector<uint16_t> dynamic_trace_list;
    std::vector<uint8_t> valid_empty_list;
    std::vector<uint8_t> valid_trace_list;

    /* 分别统计航迹个数与量测个数 */
    compute_trace_num(dyn_track_list, dynamic_trace_list);

    // 统计静态航迹
    creat_valid_invalid_trace_list(static_track_list, valid_empty_list, valid_trace_list);

    /* 先存静态航迹 */
    for (const auto sub_trace : valid_trace_list)
    {
        orin_trace_info_t new_trace_info;
        new_trace_info.trace_idx = sub_trace;

        new_trace_info.track_type = (static_track_list[sub_trace].status == ACTIVATE) ? \
                                    (ACTIVE_STATIC_TRACK):(UNACTIVE_STATIC_TRACK);

        trace_info_list_all.push_back(new_trace_info);
    }

    /* 再存运动航迹 */
    for (const auto sub_trace : dynamic_trace_list)
    {
        orin_trace_info_t new_trace_info;
        new_trace_info.trace_idx = sub_trace;

        new_trace_info.track_type = (dyn_track_list[sub_trace].trackState == TRACK_STATE_ACTIVE) ? \
                                    (ACTIVE_DYNAMIC_TRACK):(UNACTIVE_DYNAMIC_TRACK);

        trace_info_list_all.push_back(new_trace_info);
    }
}

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @param {trackTable_strcut} *dyn_track_list
 * @param {gridTrack_t} *static_track_list
 * @param {  } std
 * @param {  } std
 * @param {  } std
 * @return {*}
 */



void match_alg(trackTable_strcut *dyn_track_list,
               gridTrack_t *static_track_list,
               std::vector<orin_trace_info_t> &trace_info_list,
               std::vector<det_box_t> &det_box_list_all,
               std::vector<std::vector<uint16_t>> &all_assigned_box)
{
    std::vector<rect_point_struct> trace_box_list;    // ALL
    std::vector<rect_point_struct> det_rect_box_list; // ALL

    all_assigned_box.resize(trace_info_list.size());

    /* 将所有det-box转换成矩形信息方便后续进行IOU计算 */
    for (const auto det_box : det_box_list_all)
    {
        rect_point_struct cluster_box;
        creat_rect_box_point(det_box.x_pos, det_box.y_pos, det_box.len, det_box.wid, 0, cluster_box);
        det_rect_box_list.push_back(cluster_box);
    }

    /* 同样的构造所有航迹的矩形信息 */
    for (const auto sub_info : trace_info_list)
    {
        rect_point_struct trace_box;

        if ((sub_info.track_type == ACTIVE_DYNAMIC_TRACK) || (sub_info.track_type == UNACTIVE_DYNAMIC_TRACK))
        {
            trackTable_strcut &trace = dyn_track_list[sub_info.trace_idx];

            creat_rect_box_point(trace.KalmanInfo.StateEst(0), trace.KalmanInfo.StateEst(1),
                                 trace.ExtendInfo.Length * 1.0, trace.ExtendInfo.Width * 1.0,
                                 -1.0 * trace.ExtendInfo.box_theta, trace_box);
        }
        else
        {
            gridTrack_t &trace = static_track_list[sub_info.trace_idx];

            creat_rect_box_point(trace.kf_info.X_(0), trace.kf_info.X_(1),
                                 trace.len, trace.wid,
                                 0.0, trace_box);
        }

        trace_box_list.push_back(trace_box);
    }

    /* 计算代价 */
    std::vector<greedy_match_info_t> cost_matrix;
    for (uint32_t trace_idx = 0; trace_idx < trace_box_list.size(); trace_idx++)
    {
        const auto &sub_info = trace_info_list.at(trace_idx);

        double trace_info[3];

        if((sub_info.track_type == ACTIVE_DYNAMIC_TRACK) || (sub_info.track_type == UNACTIVE_DYNAMIC_TRACK))
        {
            trackTable_strcut &trace1 = dyn_track_list[sub_info.trace_idx];
            trace_info[0] = trace1.KalmanInfo.StateEst(0);
            trace_info[1] = trace1.KalmanInfo.StateEst(1);
            trace_info[2] = trace1.KalmanInfo.MeasPre(2);
        }
        else
        {
            gridTrack_t &trace2 = static_track_list[sub_info.trace_idx];
            double range_ = sqrt(pow(trace2.kf_info.X_(iDistLat), 2.0) + pow(trace2.kf_info.X_(iDistLong), 2.0));
            double temp_data = trace2.kf_info.X_(iDistLat) * trace2.kf_info.X_(iVrelLat) + trace2.kf_info.X_(iDistLong) * trace2.kf_info.X_(iVrelLong);

            trace_info[0] = trace2.kf_info.X_(0);
            trace_info[1] = trace2.kf_info.X_(1);
            trace_info[2] = temp_data / range_;
        }

        for (uint32_t det_box_idx = 0; det_box_idx < det_rect_box_list.size(); det_box_idx++)
        {
            greedy_match_info_t new_pair;
            new_pair.trace_idx = trace_idx;    // 航迹索引
            new_pair.det_idx = det_box_idx; // 聚类box索引

            new_pair.trace_box_s = calcularea(trace_box_list.at(trace_idx));
            new_pair.det_box_s = calcularea(det_rect_box_list.at(det_box_idx));
            new_pair.box_overlap = intersection_area(trace_box_list.at(trace_idx), det_rect_box_list.at(det_box_idx));
            new_pair.iou = new_pair.box_overlap / (new_pair.trace_box_s + new_pair.det_box_s - new_pair.box_overlap);

            double diff_lat = det_box_list_all.at(det_box_idx).x_pos - trace_info[0];
            double diff_long = det_box_list_all.at(det_box_idx).y_pos - trace_info[1];

            new_pair.center_dis = sqrt(pow(diff_lat, 2.0) + pow(diff_long, 2.0));

            cost_matrix.push_back(new_pair);
        }
    }

    // 从大到小排列
    sort_score(cost_matrix);

    // 构造分配表格
    bool match_table[trace_box_list.size()][det_rect_box_list.size()];
    memset(match_table, false, (trace_box_list.size() * det_rect_box_list.size()) * sizeof(bool));

    // 第一次分配
    for (const auto sub_pair : cost_matrix)
    {
        uint8_t det_box_idx = sub_pair.det_idx; // cluster 索引号
        uint8_t trace_idx = sub_pair.trace_idx;
        det_box_t &sub_det_box = det_box_list_all.at(det_box_idx);
        bool assinged_flag = false;

        // 确定该航迹是否是确认的运动航迹
        if (sub_det_box.valid & (1<<VALID) || (trace_info_list.at(trace_idx).track_type == ACTIVE_DYNAMIC_TRACK))
        {
            if (sub_pair.iou > 0.0) // 存在交叉
            {
                if((trace_info_list.at(trace_idx).track_type == ACTIVE_DYNAMIC_TRACK) ||
                   (trace_info_list.at(trace_idx).track_type == UNACTIVE_DYNAMIC_TRACK))
                {
                    trackTable_strcut &trace = dyn_track_list[trace_info_list.at(trace_idx).trace_idx];

                    // 确认运动航迹与动态box的关联逻辑
                    if((trace_info_list.at(trace_idx).track_type == ACTIVE_DYNAMIC_TRACK) && (sub_det_box.box_feature == DYNAMIC_BOX))
                    {
                        if(sub_det_box.assigned_obj == ACTIVE_DYNAMIC_TRACK)
                        {
                            // TBD
                        }
                        else
                        {
                            // 20230616: 增加速度判定 RADAR-Q74
                            // trackTable_strcut &trace = dyn_track_list[trace_info_list.at(trace_idx).trace_idx];
                            if(fabs(sub_det_box.v_mean - trace.KalmanInfo.MeasPre(2)) < 4.0)
                            {
                                //新增余弦残差
                                double a1 = atan2(sub_det_box.x_pos, sub_det_box.y_pos);
                                double a2 = atan2(trace.KalmanInfo.MeasPre(0), trace.KalmanInfo.MeasPre(1));
                                double diff_ = a1 - a2;
                                if((fabs(diff_) < 10.0 * DEG2RAD))
                                {
                                    assinged_flag = true;
                                }
                            }
                            else
                            {
                                // 远离的航迹纵向距离小于0时，多普勒计算错误导致关联异常
                                if((trace.ExtendInfo.DynProp == oncoming) && (trace.KalmanInfo.MeasPre(1) < 0.0))
                                {
                                    assinged_flag = true;
                                }
                            }
                        }
                    }

                    // 临时运动航迹与动态box的关联逻辑
                    if((trace_info_list.at(trace_idx).track_type == UNACTIVE_DYNAMIC_TRACK) && (sub_det_box.box_feature == DYNAMIC_BOX))
                    {
                          // 若该运动box已经被确认运动航迹所关联，则无法分配给临时航迹
                        if(sub_det_box.assigned_obj == ACTIVE_DYNAMIC_TRACK)
                        {
                            // TBD
                        }
                        else
                        {
                            double likelihood_pos = ComputeScaleOfEllipse(&trace, sub_det_box.x_pos, sub_det_box.y_pos);

                            if (((sub_det_box.valid & (1<<VALID)) && (likelihood_pos > 0.2)) ||
                                ((sub_det_box.valid & (1<<HAS_MATCHED)) && (likelihood_pos > 0.6)))
                            {
                                double box_ratio = sub_pair.trace_box_s / sub_pair.det_box_s;

                                if(box_ratio > 0.2)
                                {
                                    assinged_flag = true;
                                }
                            }
                        }
                    }
                }

                /* 静态航迹的第一次匹配 */
                if((trace_info_list.at(trace_idx).track_type == ACTIVE_STATIC_TRACK) || \
                   (trace_info_list.at(trace_idx).track_type == UNACTIVE_STATIC_TRACK))
                {
                    gridTrack_t &static_trace = static_track_list[trace_info_list.at(trace_idx).trace_idx];
                    double box_ratio = 0.0;

                    // static_trace with static_box
                    if (sub_det_box.box_feature == STATIC_BOX)
                    {
                        if(sub_det_box.member_num >= 4)
                        {
                            box_ratio = (sub_pair.trace_box_s > sub_pair.det_box_s) ? \
                                        (sub_pair.trace_box_s / sub_pair.det_box_s) : \
                                        (sub_pair.det_box_s / sub_pair.trace_box_s);

                            // 增加size比数值，真正的静态目标的size不会发生突变，可以过滤掉一些绿化、墙壁等目标
                            if ((box_ratio < 3.0) || (sub_pair.center_dis < 2.0) || (sub_pair.box_overlap > 1.0))
                            {
                                assinged_flag = true;
                            }
                            else
                            {
                                // 正前方情况下，存在IOU就关联
                                double distance_2_zero = 0.0;
                                if(fabs(static_trace.kf_info.X_(iDistLat)) > (0.5 * static_trace.wid))
                                {
                                    distance_2_zero = (static_trace.kf_info.X_(iDistLat) > 0.0) ? \
                                                (static_trace.kf_info.X_(iDistLat) - 0.5 * static_trace.wid) :\
                                                (-static_trace.kf_info.X_(iDistLat) + 0.5 * static_trace.wid);
                                }
                                else
                                {
                                    distance_2_zero = 0.0;
                                }

                                if(distance_2_zero < 2.0)
                                {
                                    assinged_flag = true;
                                }
                            }
                        }
                    }
                    else
                    {
                        box_ratio = (sub_pair.trace_box_s > sub_pair.det_box_s) ? \
                                    (sub_pair.trace_box_s / sub_pair.det_box_s) : \
                                    (sub_pair.det_box_s / sub_pair.trace_box_s);

                        /* 额外比较两者box的大小，防止行人和路边绿化带合并 */
                        if (box_ratio < 3.0)
                        {
                            if (((sub_det_box.assigned_obj == UNACTIVE_DYNAMIC_TRACK) &&
                                 (static_trace.status == ACTIVATE)) ||
                                ((sub_det_box.valid & (1<<VALID))))
                            {
                                // 比较速度差
                                double trace_v = (static_trace.kf_info.X_(0) * static_trace.kf_info.X_(2) +
                                                  static_trace.kf_info.X_(1) * static_trace.kf_info.X_(3)) /
                                                 (sqrt(pow(static_trace.kf_info.X_(0), 2.0) + pow(static_trace.kf_info.X_(1), 2.0)));

                                double box_v = sub_det_box.v_mean;
                                double diff_v = fabs(trace_v - box_v);

                                if (diff_v < 2.0)
                                {
                                    if ((sub_det_box.assigned_obj == UNACTIVE_DYNAMIC_TRACK) ||
                                        ((sub_det_box.valid & (1<<VALID))))
                                    {
                                        if ((sub_det_box.assigned_obj == UNACTIVE_DYNAMIC_TRACK))
                                        {
                                            all_assigned_box.at(sub_det_box.matched_trace_idx).clear();
                                        }
                                    }

                                    assinged_flag = true;
                                }
                            }
                        }
                    }
                }
            }
            else
            {
                // TODO
            }
        }


        // 若分配成功，则需更新该box相关属性
        if (assinged_flag == true)
        {
            sub_det_box.valid &= (~(1<<VALID));
            sub_det_box.valid |= (1<<HAS_MATCHED);

            // 运动航迹
            if((trace_info_list.at(trace_idx).track_type == ACTIVE_DYNAMIC_TRACK) || \
               (trace_info_list.at(trace_idx).track_type == UNACTIVE_DYNAMIC_TRACK))
            {
                trackTable_strcut &trace = dyn_track_list[trace_info_list.at(trace_idx).trace_idx];
                if (trace.trackState == TRACK_STATE_ACTIVE)
                {
                    sub_det_box.assigned_obj = ACTIVE_DYNAMIC_TRACK;
                }
                else
                {
                    sub_det_box.assigned_obj = UNACTIVE_DYNAMIC_TRACK;
                }
            }
            else
            {
                gridTrack_t &static_trace = static_track_list[trace_info_list.at(trace_idx).trace_idx];

                if (static_trace.status == ACTIVATE)
                {
                    sub_det_box.assigned_obj = ACTIVE_STATIC_TRACK;
                }
                else
                {
                    sub_det_box.assigned_obj = UNACTIVE_STATIC_TRACK;
                }
            }

            sub_det_box.matched_trace_idx = trace_idx;
            match_table[trace_idx][det_box_idx] = true;
        }
    }

    for(uint16_t meas_idx=0; meas_idx<det_rect_box_list.size(); meas_idx++)
    {
        bool is_match_activa_trace = false;
        for(uint16_t trace_idx=0; trace_idx<trace_box_list.size(); trace_idx++)
        {
            if(match_table[trace_idx][meas_idx] == true)
            {
                if((trace_info_list.at(trace_idx).track_type == ACTIVE_DYNAMIC_TRACK) || \
                   (trace_info_list.at(trace_idx).track_type == ACTIVE_STATIC_TRACK))
                {
                    is_match_activa_trace = true;
                    break;
                }
            }
        }

        if(is_match_activa_trace)
        {
            for(uint16_t trace_idx=0; trace_idx<trace_box_list.size(); trace_idx++)
            {
                if((match_table[trace_idx][meas_idx] == true) && \
                   (trace_info_list.at(trace_idx).track_type == UNACTIVE_DYNAMIC_TRACK))
                {
                   match_table[trace_idx][meas_idx] = false;
                }
            }
        }
    }

    for(uint16_t trace_idx=0; trace_idx<trace_box_list.size(); trace_idx++)
    {
        for(uint16_t meas_idx=0; meas_idx<det_rect_box_list.size(); meas_idx++)
        {
            if(match_table[trace_idx][meas_idx] == true)
            {
                all_assigned_box.at(trace_idx).push_back(meas_idx);
            }
        }
    }

    // 第二次分配：运动航迹补充分配
    for (const auto sub_pair : cost_matrix)
    {
        uint8_t det_box_idx = sub_pair.det_idx; // cluster 索引号
        uint8_t trace_idx = sub_pair.trace_idx;
        det_box_t &sub_det_box = det_box_list_all.at(det_box_idx);
        bool assinged_flag = false;

//        // 只对确认航迹进行补充关联
//        if ((trace_info_list.at(trace_idx).track_type != ACTIVE_DYNAMIC_TRACK) ||
//            (sub_det_box.matched_trace_idx == trace_idx))
//        {
//            continue;
//        }

        // 只对确认航迹进行补充关联
        if ((trace_info_list.at(trace_idx).track_type > UNACTIVE_DYNAMIC_TRACK) || (sub_det_box.matched_trace_idx == trace_idx))
        {
            continue;
        }

        trackTable_strcut &trace = dyn_track_list[trace_info_list.at(trace_idx).trace_idx];

        Eigen::Matrix3d R_ = trace.KalmanInfo.R_;
        double iou_val;

        if (((sub_det_box.valid & (1<<VALID)) == (1<<VALID)) ||
            (((sub_det_box.valid & (1<<HAS_MATCHED))) &&
             ((sub_det_box.assigned_obj == UNACTIVE_DYNAMIC_TRACK) ||
              (sub_det_box.assigned_obj == UNACTIVE_STATIC_TRACK))))
        {
            iou_val = sub_pair.iou;

            double likelihood_pos = ComputeScaleOfEllipse(&trace,
                                                          sub_det_box.x_pos,
                                                          sub_det_box.y_pos);

            double diff_v = sub_det_box.v_mean - trace.KalmanInfo.MeasPre(2);
            double likeliHood_doppler = 0.0;

            /* 对于小目标，提高速度v的噪声 */
            if (trace.ExtendInfo.Object_Class < VEHICLE)
            {
                likeliHood_doppler = exp(-0.5 * (pow(diff_v, 2.0) / (R_(2, 2) + pow(0.5 * 2, 2.0))));
            }
            else
            {
                likeliHood_doppler = exp(-0.5 * (pow(diff_v, 2.0) / R_(2, 2)));
            }

            bool same_dir = ((sub_det_box.v_mean * trace.KalmanInfo.MeasPre(2)) >= 0.0) ? true : false;

            if (sub_det_box.box_feature == DYNAMIC_BOX)
            {
                if (same_dir)
                {
                    if(trace.trackState == TRACK_STATE_DETECTION)
                    {
                        if (((likelihood_pos > 0.5) && (likeliHood_doppler > 0.1)) ||
                            ((likelihood_pos > 0.2) && (likeliHood_doppler > 0.6)) ||
                            ((likelihood_pos > 0.01) && (likeliHood_doppler > 0.5)) )
                        {
                            assinged_flag = true;
                        }
                    }
                    else
                    {
                        if(trace.ExtendInfo.Object_Class >= VEHICLE)
                        {
                            if (((likelihood_pos > 0.5) && (likeliHood_doppler > 0.1)) ||
                                ((likelihood_pos > 0.2) && (likeliHood_doppler > 0.6)) ||
                                ((likelihood_pos > 0.01) && (likeliHood_doppler > 0.5)) )
                            {
                                assinged_flag = true;
                            }
                        }
                        else
                        {
                            // 对于行人等小目标，更侧重距离
                            if((likelihood_pos > 0.3) && (likeliHood_doppler > 0.1))
                            {
                                assinged_flag = true;
                            }
                        }
                    }
                }
            }
            else // static box
            {
                double trace_box_size = trace.ExtendInfo.Length * trace.ExtendInfo.Width;
                trace_box_size = Valuelimit(1.0, trace_box_size, trace_box_size);

                double diff_v = sub_det_box.v_mean - trace.KalmanInfo.MeasPre(2);
                diff_v = (diff_v > 0.5) ? (diff_v - 0.5) : 0.0;
                double likeliHood_doppler = exp(-0.5 * (pow(diff_v, 2.0) / (trace.KalmanInfo.R_(2, 2) + pow(0.25 * 2, 2.0))));
                double det_box_size = sub_det_box.len * sub_det_box.wid;

                double box_ratio = (det_box_size / trace_box_size);

                double center_dis = sqrt(pow(sub_det_box.x_pos - trace.KalmanInfo.StateEst(iDistLat), 2.0) +\
                                         pow(sub_det_box.y_pos - trace.KalmanInfo.StateEst(iDistLong), 2.0) );

                if (box_ratio < 5.0)
                {
                    if (trace.ExtendInfo.Object_Class < VEHICLE) // 小目标不强行要求iou
                    {
                        if ((center_dis < 2.0) && (likeliHood_doppler > 0.4))
                        {
                            assinged_flag = true;
                        }
                    }
                    else
                    {
                        if (((iou_val > 0.1) && (likeliHood_doppler > 0.2)) ||
                            ((iou_val > 0.0) && (likelihood_pos > 0.4) && (likeliHood_doppler > 0.4)) ||
                            ((iou_val > 0.1) && (likelihood_pos > 0.8)))
                        {
                            assinged_flag = true;
                        }
                    }
                }
            }
        }

        if (assinged_flag == true)
        {
            sub_det_box.valid &= (~(1<<VALID));
            sub_det_box.valid |= (1<<HAS_MATCHED);

            if (trace.trackState == TRACK_STATE_ACTIVE)
            {
                sub_det_box.assigned_obj = ACTIVE_DYNAMIC_TRACK;
            }
            else
            {
                sub_det_box.assigned_obj = UNACTIVE_DYNAMIC_TRACK;
            }

            sub_det_box.matched_trace_idx = trace_idx;

            // 在对应的航迹索引号位置填充关联的cluster索引号
            bool new_det = true;
            for (const auto &sub_det : all_assigned_box.at(trace_idx))
            {
                if (sub_det == det_box_idx)
                {
                    new_det = false;
                    break;
                }
            }

            if (new_det)
            {
                all_assigned_box.at(trace_idx).push_back(det_box_idx);
            }
        }
    }

    // 第三次分配：静态航迹补充分配
    for (const auto sub_pair : cost_matrix)
    {
        uint8_t det_box_idx = sub_pair.det_idx; // cluster 索引号
        uint8_t trace_idx = sub_pair.trace_idx;
        det_box_t &sub_det_box = det_box_list_all.at(det_box_idx);
        bool assinged_flag = false;

        if ((trace_info_list.at(trace_idx).track_type < ACTIVE_STATIC_TRACK) ||
            (sub_det_box.matched_trace_idx == trace_idx))
        {
            continue;
        }

        if (((sub_det_box.valid & (1<<VALID)) == (1<<VALID)) || (sub_det_box.assigned_obj == UNACTIVE_DYNAMIC_TRACK))
        {
            /* 静态航迹的第一次匹配 */
            gridTrack_t &static_trace = static_track_list[trace_info_list.at(trace_idx).trace_idx];
            double trace_box_size = static_trace.len * static_trace.wid;
            double det_box_size = sub_det_box.len * sub_det_box.wid;
            double box_ratio = (trace_box_size > det_box_size) ? (trace_box_size / det_box_size) : (det_box_size / trace_box_size);

            double center_dis = sqrt(pow(sub_det_box.x_pos - static_trace.kf_info.X_(iDistLat), 2.0) +\
                                     pow(sub_det_box.y_pos - static_trace.kf_info.X_(iDistLong), 2.0) );

            /* 两个判定条件：
                1、形状不能相差很大
                2、距离不能相隔很远
            */
            if(sub_det_box.box_feature == STATIC_BOX)
            {
                if(sub_det_box.member_num >= 4)
                {
                    if((center_dis < 2.0) && (box_ratio < 4.0))
                    {
                        assinged_flag = true;
                    }
                    else
                    {
                        if(box_ratio < 1.5)
                        {
                            // 判定最近边距离
                            double closet_side_range = 0.0;
                            if(static_trace.kf_info.X_(iDistLat) > sub_det_box.x_pos)
                            {
                                closet_side_range = (static_trace.kf_info.X_(iDistLat) - 0.5*static_trace.wid) - \
                                        (sub_det_box.x_pos + 0.5*sub_det_box.wid);
                            }
                            else
                            {
                                closet_side_range = (static_trace.kf_info.X_(iDistLat) + 0.5*static_trace.wid) - \
                                        (sub_det_box.x_pos - 0.5*sub_det_box.wid);
                            }

                            if((fabs(closet_side_range) < 1.0) && (fabs(sub_det_box.y_pos - static_trace.kf_info.X_(iDistLong)) < 1.0))
                            {
                                assinged_flag = true;
                            }
                        }
                    }
                }
            }
            else if((sub_det_box.box_feature == DYNAMIC_BOX) && (sub_det_box.member_num > 1))
            {
                if((center_dis < 2.0) && (box_ratio < 10.0))
                {
                    assinged_flag = true;
                }
                else
                {
                    double iou_sp = (sub_pair.trace_box_s < sub_pair.det_box_s) ? \
                                    (sub_pair.box_overlap / sub_pair.trace_box_s) : \
                                    (sub_pair.box_overlap / sub_pair.det_box_s);

                    if((iou_sp > 0.5) && (box_ratio < 10.0))
                    {
                        assinged_flag = true;
                    }
                }
            }
        }

        if (assinged_flag == true)
        {
            sub_det_box.valid &= (~(1<<VALID));
            sub_det_box.valid |= (1<<HAS_MATCHED);
            sub_det_box.matched_trace_idx = trace_idx;
            all_assigned_box.at(trace_idx).push_back(det_box_idx);
        }
    }
}

/**
 * @name:
 * @description:
 * @param {trackTable_strcut} *dyn_track_list
 * @param {  } std
 * @return {*}
 */
void compute_trace_num(trackTable_strcut *dyn_track_list,
                       std::vector<uint16_t> &TraceSet)
{
    trackTable_strcut *trace = nullptr;
    // 先统计确认航迹个数
    for (uint16_t idx = 0; idx < MAXTRACKS; idx++)
    {
        if (dyn_track_list[idx].trackState == TRACK_STATE_ACTIVE)
        {
            TraceSet.push_back(idx);
            trace = &dyn_track_list[idx];

            trace->MeasInfo.HighQuaPointNum = 0U;
            trace->MeasInfo.associateNum = 0U;
            trace->MeasInfo.sum_weight = 0.0F;
            trace->MeasInfo.dopplerVari = 0.0F;
            trace->MeasInfo.detInfo.clear();
        }
    }

    // 后统计未确认航迹个数
    for (uint16_t idx = 0; idx < MAXTRACKS; idx++)
    {
        if (dyn_track_list[idx].trackState == TRACK_STATE_DETECTION)
        {
            TraceSet.push_back(idx);
            trace = &dyn_track_list[idx];
            trace->MeasInfo.HighQuaPointNum = 0U;
            trace->MeasInfo.associateNum = 0U;
            trace->MeasInfo.sum_weight = 0.0F;
            trace->MeasInfo.dopplerVari = 0.0F;
            trace->MeasInfo.detInfo.clear();
        }
    }
}

/**
 * @name:
 * @description:
 * @param {vector<RadarMeasure_struct>} &fifo_point
 * @param {  } std
 * @return {*}
 */
void compute_meas_num(std::vector<RadarMeasure_struct> &fifo_point,
                      std::vector<uint16_t> &MeasSet)
{
    // 统计运动量测个数
    for (uint16_t idx = 0; idx < fifo_point.size(); idx++)
    {
        /* delete "staticPoint" */
        if ((fifo_point[idx].DynProp == stationary) ||
            (fifo_point[idx].invalid == (1 << invalid_det)))
        {
            continue;
        }

        MeasSet.push_back(idx);
    }
}

/**
 * @name: dyn_trace_update_and_manage
 * @description:
 * @param {trackTable_strcut} *dyn_track_list
 * @param {  } std
 * @param {  } std
 * @param {Tracker} *Tracker_
 * @return {*}
 */
void dyn_trace_update_and_manage(trackTable_strcut *dyn_track_list,
                                 std::vector<RadarMeasure_struct> &fifo_point,
                                 std::vector<vehicleInfo_struct> &vehicleInfo,
                                 Tracker *Tracker_)
{
    /* 航迹更新 */
    track_update(fifo_point, dyn_track_list, Tracker_);

    /* 航迹管理 */
    track_Manage(dyn_track_list, &vehicleInfo[0]);

    /* 航迹起始 */
    track_birth(fifo_point, dyn_track_list, &vehicleInfo[0]);
}

/**
 * @name: All_Trace_Mage_Proc
 * @description:
 * @param {trackTable_strcut} *dyn_track_list
 * @param {gridTrack_t} *static_track_list
 * @return {*}
 */
void All_Trace_Mage_Proc(trackTable_strcut *dyn_track_list, gridTrack_t *static_track_list)
{
    view_grid_test(dyn_track_list, static_track_list);
}

/**
 * @name: is_multi_bounce
 * @description:
 * @param {trackTable_strcut} *dyn_track_list
 * @param {gridTrack_t} *static_track_list
 * @return {*}
 */
bool is_multi_bounce(trackTable_strcut *trace1, trackTable_strcut *trace2)
{
    double iou_ = 0.0;
    bool fakeTrace = false;
    double jumped_trace_range, jumped_dis_lat, jumped_dis_long;

    double trace_range2 = sqrt(pow(trace2->KalmanInfo.StateEst(iDistLat)-2.57837, 2.0) + \
                              pow(trace2->KalmanInfo.StateEst(iDistLong), 2.0));

    double trace_angle2 = atan2(trace2->KalmanInfo.StateEst(iDistLat)-2.57837, \
                                trace2->KalmanInfo.StateEst(iDistLong));

    rect_point_struct trace1_box, trace2_box;
    creat_rect_box_point(trace1->KalmanInfo.StateEst(iDistLat)-2.57837, trace1->KalmanInfo.StateEst(iDistLong), \
                         trace1->ExtendInfo.Length, trace1->ExtendInfo.Width, (-1.0) * trace1->ExtendInfo.box_theta, trace1_box);

    jumped_trace_range = trace_range2 * 2.0;
    jumped_dis_lat = jumped_trace_range * sin(trace_angle2);
    jumped_dis_long = jumped_trace_range * cos(trace_angle2);

    creat_rect_box_point(jumped_dis_lat, jumped_dis_long, \
                         trace2->ExtendInfo.Length * 1.2, trace2->ExtendInfo.Width * 1.5, \
                         (-1.0) * trace2->ExtendInfo.box_theta, trace2_box);


    double trace1_doppler_v = (trace1->KalmanInfo.StateEst(iDistLat) * trace1->KalmanInfo.StateEst(iVrelLat) +
                               trace1->KalmanInfo.StateEst(iDistLong) * trace1->KalmanInfo.StateEst(iVrelLong)) /
                              (sqrt(pow(trace1->KalmanInfo.StateEst(iDistLat), 2.0) + pow(trace1->KalmanInfo.StateEst(iDistLong), 2.0)));

    double trace2_doppler_v = (trace2->KalmanInfo.StateEst(iDistLat) * trace2->KalmanInfo.StateEst(iVrelLat) +
                               trace2->KalmanInfo.StateEst(iDistLong) * trace2->KalmanInfo.StateEst(iVrelLong)) /
                              (sqrt(pow(trace2->KalmanInfo.StateEst(iDistLat), 2.0) + pow(trace2->KalmanInfo.StateEst(iDistLong), 2.0)));

    iou_ = IOU_With_Rot(trace1_box, trace2_box);

    if(iou_ > 0.0)
    {
        //TODO：当距离满足时，进一步判定两者速度情况
        if((trace1->trackState <= trace2->trackState) && (fabs(trace1_doppler_v - trace2_doppler_v*2.0) < 1.0))
        {
            fakeTrace = true;
        }
    }
    else
    {
        jumped_trace_range = trace_range2 * 3.0;
        jumped_dis_lat = jumped_trace_range * sin(trace_angle2);
        jumped_dis_long = jumped_trace_range * cos(trace_angle2);

        creat_rect_box_point(jumped_dis_lat, jumped_dis_long, \
                             trace2->ExtendInfo.Length * 1.2, trace2->ExtendInfo.Width * 1.5, \
                             (-1.0) * trace2->ExtendInfo.box_theta, trace2_box);

        iou_ = IOU_With_Rot(trace1_box, trace2_box);

        if(iou_)
        {
            if((trace1->trackState <= trace2->trackState) && (fabs(trace1_doppler_v - trace2_doppler_v*3.0) < 1.0))
            {
                fakeTrace = true;
            }
        }
    }

    return fakeTrace;
}


/**
 * @names:
 * @description: Briefly describe the function of your function
 * @param {trackTable_strcut} *dynamic_trace_1
 * @param {trackTable_strcut} *dynamic_trace_2
 * @return {*}
 */
void compare_between_dynamic_trace(trackTable_strcut *dynamic_trace_1, trackTable_strcut *dynamic_trace_2)
{
    /* part1：比较其中一个航迹是否为另一个航迹的multi-bounce */
    if(is_multi_bounce(dynamic_trace_1, dynamic_trace_2))
    {
        if(dynamic_trace_1->trackState == TRACK_STATE_DETECTION)
        {
            track_clear(dynamic_trace_1);
        }
    }

    /* 两航迹是否存在重叠（overlap） */
    rect_point_struct r1, r2;
    creat_rect_box_point(dynamic_trace_2->KalmanInfo.StateEst(0), dynamic_trace_2->KalmanInfo.StateEst(1),
                         dynamic_trace_2->ExtendInfo.Length, dynamic_trace_2->ExtendInfo.Width,
                         -1.0 * dynamic_trace_2->ExtendInfo.box_theta, r1);

    creat_rect_box_point(dynamic_trace_1->KalmanInfo.StateEst(0), dynamic_trace_1->KalmanInfo.StateEst(1),
                         dynamic_trace_1->ExtendInfo.Length, dynamic_trace_1->ExtendInfo.Width,
                         -1.0 * dynamic_trace_1->ExtendInfo.box_theta, r2);

    double iou_val = IOU_With_Rot(r1, r2);

    if (iou_val > 0.1)
    {
        if(dynamic_trace_1->trackState == TRACK_STATE_DETECTION)
        {
            track_clear(dynamic_trace_1);
        }
    }
}


/**
 * @names:
 * @description: Briefly describe the function of your function
 * @param {trackTable_strcut} *dyn_track_list
 * @param {gridTrack_t} *static_track_list
 * @return {*}
 */
void view_grid_test(trackTable_strcut *dyn_track_list, gridTrack_t *static_track_list)
{
    double ang_min, ang_max;
    std::vector<std::pair<uint32_t, std::pair<double, double>>> trace_angle_list;

    //遍历所有运动航迹并计算其角度范围
    for (uint16_t idx = 0; idx < MAXTRACKS; idx++){
        if (dyn_track_list[idx].trackState > TRACK_STATE_FREE) {
            trackTable_strcut *sub_trace = &dyn_track_list[idx];

            std::pair<uint32_t, std::pair<double, double>> sub_trace_ang_info;
            sub_trace_ang_info.first = idx;
            compute_trace_anlge(sub_trace->KalmanInfo.StateEst(iDistLat) - RADAR_MOUNT_Y, sub_trace->KalmanInfo.StateEst(iDistLong) - RADAR_MOUNT_X,\
                                0.5 * sub_trace->ExtendInfo.Length, 0.5 * sub_trace->ExtendInfo.Width,
                                sub_trace->ExtendInfo.box_theta,\
                                &ang_min, &ang_max);

            sub_trace_ang_info.second.first = ang_min;
            sub_trace_ang_info.second.second = ang_max;

            trace_angle_list.push_back(sub_trace_ang_info);
        }
    }

    //遍历所有静态航迹并计算其角度范围
    for (uint16_t idx = 0; idx < STATIC_TRACK_MAXNUM; idx++) {
      if (static_track_list[idx].status > EMPTY) {
          gridTrack_t *sub_trace = &static_track_list[idx];
          std::pair<uint32_t, std::pair<double, double>> sub_trace_ang_info;
          sub_trace_ang_info.first = idx + MAXTRACKS;
          compute_trace_anlge(sub_trace->kf_info.X_(iDistLat) - RADAR_MOUNT_Y, sub_trace->kf_info.X_(iDistLong) - RADAR_MOUNT_X,\
                              0.5 * sub_trace->len, 0.5 * sub_trace->wid, 0.0,\
                              &ang_min, &ang_max);

          sub_trace_ang_info.second.first = ang_min;
          sub_trace_ang_info.second.second = ang_max;

          trace_angle_list.push_back(sub_trace_ang_info);
        }
    }

    std::vector<bool> trace_result(trace_angle_list.size(), true);

    trackTable_strcut *slaver_dynamic_trace = nullptr;
    trackTable_strcut *master_dynamic_trace = nullptr;
    gridTrack_t *slaver_static_trace = nullptr;
    gridTrack_t *master_static_trace = nullptr;

    uint32_t slaver_idx = 0;
    double slaver_trance_range, master_trace_range;
    // 遍历所有航迹（slaver_trace）
    for(auto slaver_trace_ang : trace_angle_list)
    {
        if(is_dynamic_trace(slaver_trace_ang.first))
        {
            slaver_dynamic_trace = &dyn_track_list[slaver_trace_ang.first];
            slaver_static_trace = nullptr;

            slaver_trance_range = sqrt(pow(slaver_dynamic_trace->KalmanInfo.StateEst(iDistLat), 2.0) +
                                       pow(slaver_dynamic_trace->KalmanInfo.StateEst(iDistLong), 2.0));
        }
        else
        {
            slaver_static_trace = &static_track_list[slaver_trace_ang.first - MAXTRACKS];
            slaver_dynamic_trace = nullptr;

            slaver_trance_range = sqrt(pow(slaver_static_trace->kf_info.X_(iDistLat), 2.0) +
                                       pow(slaver_static_trace->kf_info.X_(iDistLong), 2.0));
        }

        // 确定master_trace
        bool pure_trace_flag = true;
        for(auto master_trace_ang : trace_angle_list)
        {
            if(slaver_trace_ang.first == master_trace_ang.first)
            {
                continue;
            }

            if(is_dynamic_trace(master_trace_ang.first))
            {
                master_dynamic_trace = &dyn_track_list[master_trace_ang.first];
                master_static_trace = nullptr;

                master_trace_range = sqrt(pow(master_dynamic_trace->KalmanInfo.StateEst(iDistLat), 2.0) +
                                           pow(master_dynamic_trace->KalmanInfo.StateEst(iDistLong), 2.0));
            }
            else
            {
                master_static_trace = &static_track_list[master_trace_ang.first - MAXTRACKS];
                master_dynamic_trace = nullptr;

                master_trace_range = sqrt(pow(master_static_trace->kf_info.X_(iDistLat), 2.0) +
                                           pow(master_static_trace->kf_info.X_(iDistLong), 2.0));
            }

            /* 开始比较 */
            /* 静态目标之间 */
            if((slaver_static_trace != nullptr) && (master_static_trace != nullptr))
            {
                pure_trace_flag = false;
                if(trace_result.at(slaver_idx))
                {
                    trace_result.at(slaver_idx) = compare_static_trace(slaver_static_trace, master_static_trace);
                }
            }

            angle_enum angele_type = compute_two_angle_position(slaver_trace_ang.second, master_trace_ang.second);

            if((angele_type > ANGLE_NULLSET) && (slaver_trance_range > master_trace_range))
            {
                pure_trace_flag = false;
            }

            /* 运动航迹之间 */
            if((slaver_dynamic_trace != nullptr) && (master_dynamic_trace != nullptr))
            {
                if(slaver_trance_range > master_trace_range)
                {
                    // 视角完全重叠
                    if(angele_type == ANGLE_OVERLAP)
                    {
                        if((master_dynamic_trace->trackState == TRACK_STATE_ACTIVE) && \
                            (master_dynamic_trace->ExtendInfo.Object_Class > PEDESTRIAN ))
                        {
                            if(slaver_dynamic_trace->trackState == TRACK_STATE_DETECTION)
                            {
                                trace_result.at(slaver_idx) = false;
                            }
                            else
                            {
                                // 当前方车处于±2米内，则删除其前方目标
                                if(((fabs(master_dynamic_trace->KalmanInfo.StateEst(iDistLat)) - 0.5*master_dynamic_trace->ExtendInfo.Width) < 2.0) && \
                                    ((fabs(slaver_dynamic_trace->KalmanInfo.StateEst(iDistLat)) - 0.5*slaver_dynamic_trace->ExtendInfo.Width) < 2.0))
                                {
                                    trace_result.at(slaver_idx) = false;
                                }
                            }
                        }
                    }

                    if(angele_type == ANGLE_CROSS)
                    {
                        // 补充正前方特殊情况
                        if((fabs(slaver_dynamic_trace->KalmanInfo.StateEst(iDistLat)) < 2.0) && \
                            (fabs(master_dynamic_trace->KalmanInfo.StateEst(iDistLat)) < 2.0))
                        {
                            if(master_dynamic_trace->ExtendInfo.Object_Class > PEDESTRIAN)
                            {
                                if(slaver_dynamic_trace->trackState == TRACK_STATE_DETECTION)
                                {
                                    trace_result.at(slaver_idx) = false;
                                }
                            }
                        }
                    }

                    if(trace_result.at(slaver_idx))
                    {
                        /* Multi_Bounce clutter */
                        trace_result.at(slaver_idx) =  (!(is_multi_bounce(slaver_dynamic_trace, master_dynamic_trace)));
                    }

                    // 20230606 add:
                    if(trace_result.at(slaver_idx))
                    {
                        // 正前方有辆车(确认航迹)
                        if( (fabs(master_dynamic_trace->KalmanInfo.StateEst(iDistLat)) < 2.0) && \
                                (master_dynamic_trace->trackState == TRACK_STATE_ACTIVE) && \
                                (master_dynamic_trace->ExtendInfo.Object_Class >= VEHICLE ))
                        {
                            double diff_vlong = fabs(master_dynamic_trace->KalmanInfo.StateEst(iVrelLong) - \
                                                  slaver_dynamic_trace->KalmanInfo.StateEst(iVrelLong));

                            // 纵向速度差较小且运动方向一致
                            if((diff_vlong < 1.0) && ((master_dynamic_trace->KalmanInfo.StateEst(iVrelLong) * \
                                                      slaver_dynamic_trace->KalmanInfo.StateEst(iVrelLong)) > 0.0))
                            {

                                double diff_lat = fabs(master_dynamic_trace->KalmanInfo.StateEst(iDistLat) - \
                                                       slaver_dynamic_trace->KalmanInfo.StateEst(iDistLat));

                                if(diff_lat < (0.5*(master_dynamic_trace->ExtendInfo.Width + slaver_dynamic_trace->ExtendInfo.Width)))
                                {
                                    diff_lat = 0.0;
                                }
                                else
                                {
                                    diff_lat = diff_lat - 0.5*(master_dynamic_trace->ExtendInfo.Width + slaver_dynamic_trace->ExtendInfo.Width);
                                }

                                if((slaver_dynamic_trace->KalmanInfo.StateEst(iDistLong) > \
                                        master_dynamic_trace->KalmanInfo.StateEst(iDistLong)) && (diff_lat < 1.0))
                                {
                                    trace_result.at(slaver_idx) = false;
                                }
                            }
                        }
                    }
                }
            }

            /* 被运动航迹遮挡的静态航迹 */
            if((slaver_static_trace != nullptr) && (master_dynamic_trace != nullptr))
            {
                // 动\静航迹是否都处于正前方
                double dis_to_zero_1, dis_to_zero_2;

                if(fabs(master_dynamic_trace->KalmanInfo.StateEst(iDistLat)) < (master_dynamic_trace->ExtendInfo.Width * 0.5) )
                {
                    dis_to_zero_1 = 0.0;
                }
                else
                {
                    dis_to_zero_1 = (master_dynamic_trace->KalmanInfo.StateEst(iDistLat) > 0.0) ? \
                                    (master_dynamic_trace->KalmanInfo.StateEst(iDistLat) - (master_dynamic_trace->ExtendInfo.Width * 0.5)) : \
                                    (-1.0*master_dynamic_trace->KalmanInfo.StateEst(iDistLat) - (master_dynamic_trace->ExtendInfo.Width * 0.5));
                }

                if(fabs(slaver_static_trace->kf_info.X_(iDistLat)) < (slaver_static_trace->wid * 0.5) )
                {
                    dis_to_zero_2 = 0.0;
                }
                else
                {
                    dis_to_zero_2 = (slaver_static_trace->kf_info.X_(iDistLat) > 0.0) ? \
                                    (slaver_static_trace->kf_info.X_(iDistLat) - (slaver_static_trace->wid * 0.5)) : \
                                    (-1.0*slaver_static_trace->kf_info.X_(iDistLat) - (slaver_static_trace->wid * 0.5));
                }

                if(angele_type > ANGLE_NULLSET)
                {
                      if((dis_to_zero_1 < 1.0) && (dis_to_zero_2 < 1.0) && \
                        (master_dynamic_trace->trackState == TRACK_STATE_ACTIVE) && \
                        (master_dynamic_trace->ExtendInfo.Object_Class >= VEHICLE) && \
                        (master_dynamic_trace->KalmanInfo.StateEst(iDistLong) < slaver_static_trace->kf_info.X_(iDistLong)))
                      {
                        trace_result.at(slaver_idx) = false;
                      }
                      else
                      {
                          if(trace_result.at(slaver_idx))
                          {
                              trace_result.at(slaver_idx) = compare_static_trace_with_dyntrace(slaver_static_trace,\
                                                                                               master_dynamic_trace);
                          }
                      }
                }
                else
                {
                    // 视角完全重叠
                    if(angele_type == ANGLE_OVERLAP)
                    {
                        if(master_dynamic_trace->trackState == TRACK_STATE_ACTIVE)
                        {
                            //只删除未确认的静止目标
                            if(slaver_static_trace->status == 1)
                            {
                                trace_result.at(slaver_idx) = false;
                            }
                        }
                    }
                }
            }

            /* 被静态航迹遮挡的运动航迹 */
            if((slaver_dynamic_trace != nullptr) && (master_static_trace != nullptr))
            {
                if(slaver_trance_range > master_trace_range)
                {
                    // 静态航迹是否在正前方
                    if( (((master_static_trace->kf_info.X_(iDistLat) > 0.0 ) && (master_static_trace->kf_info.X_(iDistLat) - 0.5 * master_static_trace->wid) < 1.0) ||
                        ((master_static_trace->kf_info.X_(iDistLat) < 0.0 ) && (master_static_trace->kf_info.X_(iDistLat) + 0.5 * master_static_trace->wid) >- 1.0)) && \
                        (master_static_trace->status == 2))
                    {
                        if(((slaver_dynamic_trace->KalmanInfo.StateEst(iDistLat) > 0.0 ) && (slaver_dynamic_trace->KalmanInfo.StateEst(iDistLat) - 0.5 * slaver_dynamic_trace->ExtendInfo.Width) < 1.0) || \
                           ((slaver_dynamic_trace->KalmanInfo.StateEst(iDistLat) < 0.0 ) && (slaver_dynamic_trace->KalmanInfo.StateEst(iDistLat) + 0.5 * slaver_dynamic_trace->ExtendInfo.Width) >- 1.0))
                        {
                            if((master_static_trace->wid > 1.0) && (master_static_trace->len > 1.0))
                            {
                                trace_result.at(slaver_idx) = false;
                            }
                        }
                    }

//                    if(angele_type == ANGLE_OVERLAP)
//                    {
//                        double size_scale = (master_static_trace->wid * master_static_trace->len ) / \
//                                            (slaver_dynamic_trace->ExtendInfo.Width * slaver_dynamic_trace->ExtendInfo.Length) ;

//                        if(size_scale > 0.8)
//                        {
//                            if((master_static_trace->status == 2) && \
//                            (slaver_dynamic_trace->trackState < TRACK_STATE_ACTIVE))
//                            {
//                                trace_result.at(slaver_idx) = false;
//                            }
//                            else
//                            {
//                                if((master_static_trace->status == 2) && (fabs(slaver_dynamic_trace->KalmanInfo.StateEst(iDistLat))< 1.0))
//                                {
//                                    trace_result.at(slaver_idx) = false;
//                                }
//                            }
//                        }
//                    }
//                    else
//                    {
//                        if((master_static_trace->status == 2) && \
//                            (slaver_dynamic_trace->trackState < TRACK_STATE_ACTIVE))
//                        {
//                            //TODO: 若运动目标处于本车道，且静态目标也处于本车道，并且该运动目标为靠近状态
//                            if((fabs(slaver_dynamic_trace->KalmanInfo.StateEst(iDistLat))< 3.0) && \
//                                (fabs(master_static_trace->kf_info.X_(iDistLat)) < 3.0) )
//                            {
//                                if(((master_static_trace->len*master_static_trace->wid) > 4.0) && \
//                                    (slaver_dynamic_trace->KalmanInfo.StateEst(iVrelLong) < 0.0))
//                                {
//                                    double angle_iou = compute_two_angle_iou(slaver_trace_ang.second, master_trace_ang.second);

//                                    if(angle_iou > 0.4)
//                                    {
//                                        trace_result.at(slaver_idx) = false;
//                                    }
//                                }
//                            }
//                        }
//                    }
                }
            }
        }

        if(slaver_dynamic_trace != nullptr)
        {
            if(pure_trace_flag)
            {
                if(slaver_dynamic_trace->ManageInfo.trace_belief < 0.2)
                {
                    slaver_dynamic_trace->ManageInfo.trace_belief += 0.05;
                }
            }
            else
            {
                if(slaver_dynamic_trace->ManageInfo.trace_belief < 0.0)
                {
                    slaver_dynamic_trace->ManageInfo.trace_belief -= 0.1;
                }

                slaver_dynamic_trace->ManageInfo.trace_belief = Valuelimit(0.0, 1.0, slaver_dynamic_trace->ManageInfo.trace_belief);
            }
        }

        slaver_idx++;
    }

    // 查询判定结果，并删除对应航迹
    slaver_idx = 0;
    for(const auto sub_result : trace_result)
    {
        if(!sub_result)
        {
            if(is_dynamic_trace(trace_angle_list.at(slaver_idx).first))
            {
                track_clear(&dyn_track_list[trace_angle_list.at(slaver_idx).first]);
            }
            else
            {
                if(static_track_list[trace_angle_list.at(slaver_idx).first - MAXTRACKS].age < 30)
                {
                    static_track_list[trace_angle_list.at(slaver_idx).first - MAXTRACKS].status = EMPTY;
                }

                #ifdef LOG_OUTPUT
                std::cout<< " (Line 1470) Delete Trace ID: " << static_track_list[trace_angle_list.at(slaver_idx).first - MAXTRACKS].ID << std::endl;
                #endif
            }
        }

        slaver_idx++;
    }

    // 删除异常静态航迹
    for (uint16_t idx = 0; idx < STATIC_TRACK_MAXNUM; idx++) {
      if (static_track_list[idx].status > EMPTY) {
          if(static_track_list[idx].kf_info.X_(iVrelLong) > 2.0)
          {
             static_track_list[idx].status = EMPTY;
          }
        }
    }
}
/**
 * @names:
 * @description: Briefly describe the function of your function
 * @param {trackTable_strcut} *dynamic_trace_1
 * @param {trackTable_strcut} *dynamic_trace_2
 * @return {*}
 */
void compute_trace_anlge(double x_pos, double y_pos, double half_len, double half_wid, double theta, double *angle_min, double *angle_max)
{
    std::vector<double> angle_list;
    double plusOrminus[4][2] = {{1.0, 1.0},{-1.0, 1.0},{-1.0, -1.0},{+1.0, -1.0}};

    if(fabs(theta) > (0.5*RADAR_PI))
    {
        theta = (theta > 0.0) ? (theta - RADAR_PI) : (theta + RADAR_PI);
    }

    double RotMat[2][2] = {{cos(theta), sin(theta)},
                          {-1.0*sin(theta), cos(theta)}};
    double tempX, tempY;

    for (uint8_t idx = 0; idx < 4; idx++) {
      tempX = x_pos + (plusOrminus[idx][0] * half_wid) * RotMat[0][0] +
              (plusOrminus[idx][1] * half_len) * RotMat[0][1];
      tempY = y_pos + (plusOrminus[idx][0] * half_wid) * RotMat[1][0] +
              (plusOrminus[idx][1] * half_len) * RotMat[1][1];

      double temp_ang = atan2(tempX, tempY);

      if(fabs(temp_ang) > (0.5*RADAR_PI))
      {
          temp_ang = (temp_ang > 0.0)?(RADAR_PI):(-1.0*RADAR_PI);
      }

      angle_list.push_back(temp_ang);
    }

    auto min_max = std::minmax_element(angle_list.begin(), angle_list.end());

    *angle_min = *min_max.first;
    *angle_max = *min_max.second;

}
/**
 * @names:
 * @description: Briefly describe the function of your function
 * @param {trackTable_strcut} *dynamic_trace_1
 * @param {trackTable_strcut} *dynamic_trace_2
 * @return {*}
 */
bool is_dynamic_trace(uint32_t trace_idx)
{
    return ((trace_idx < MAXTRACKS) ? (true) : (false));
}

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @param {trackTable_strcut} *dynamic_trace_1
 * @param {trackTable_strcut} *dynamic_trace_2
 * @return {*}
 */
bool compare_static_trace(gridTrack_t *slaver_trace, gridTrack_t *master_trace)
{
    box_t box1, box2;

    box1.x = slaver_trace->kf_info.X_(0);
    box1.y = slaver_trace->kf_info.X_(1);
    box1.len = slaver_trace->len;
    box1.wid = slaver_trace->wid;

    box2.x = master_trace->kf_info.X_(0);
    box2.y = master_trace->kf_info.X_(1);
    box2.len = master_trace->len;
    box2.wid = master_trace->wid;

    double iou = compute_box_iou(box1, box2, 1);

    if(iou > 0.3)
    {
        double box_size1 = box1.len * box1.wid;
        double box_size2 = box2.len * box2.wid;

        if((slaver_trace->status <= master_trace->status) &&(box_size1 <= box_size2))
        {
            return false;
        }
    }
    else
    {
        // 20230506 add: 对于车辆正前方（±1m）的目标，只保留最近一个航迹
        double distance1, distance2;

        if(fabs(slaver_trace->kf_info.X_(iDistLat)) < (slaver_trace->wid * 0.5) )
        {
            distance1 = 0.0;
        }
        else
        {
            distance1 = (slaver_trace->kf_info.X_(iDistLat) > 0.0) ? \
                            (slaver_trace->kf_info.X_(iDistLat) - (slaver_trace->wid * 0.5)) : \
                            (-1.0*slaver_trace->kf_info.X_(iDistLat) - (slaver_trace->wid * 0.5));
        }

        if(fabs(master_trace->kf_info.X_(iDistLat)) < (master_trace->wid * 0.5) )
        {
            distance2 = 0.0;
        }
        else
        {
            distance2 = (master_trace->kf_info.X_(iDistLat) > 0.0) ? \
                            (master_trace->kf_info.X_(iDistLat) - (master_trace->wid * 0.5)) : \
                            (-1.0*master_trace->kf_info.X_(iDistLat) - (master_trace->wid * 0.5));
        }

        if((distance1 == 0.0) && (distance2 == 0.0))
        {
            if(slaver_trace->kf_info.X_(iDistLong) > master_trace->kf_info.X_(iDistLong))
            {
                return false;
            }
        }
        else
        {
            // 本车道【-2，2】
            if((distance1 < 2.0) && (distance2 < 2.0) && \
                (fabs(slaver_trace->kf_info.X_(iDistLat) - master_trace->kf_info.X_(iDistLat)) < ((slaver_trace->wid + master_trace->wid) * 0.5 * 0.5)))
            {
                if((slaver_trace->kf_info.X_(iDistLong) > master_trace->kf_info.X_(iDistLong)) && (slaver_trace->status < master_trace->status))
                {
                    return false;
                }
            }
        }
    }

    return true;
}
/**
 * @names:
 * @description: Briefly describe the function of your function
 * @param {trackTable_strcut} *dynamic_trace_1
 * @param {trackTable_strcut} *dynamic_trace_2
 * @return {*}
 */
bool compare_static_trace_with_dyntrace(gridTrack_t *static_trace, trackTable_strcut *dynamic_trace)
{
    bool result = true;
    rect_point_struct r1, r2;
    creat_rect_box_point(static_trace->kf_info.X_(0), static_trace->kf_info.X_(1),
                           static_trace->len, static_trace->wid, 0.0, r1);

    if ((fabs(static_trace->kf_info.X_(0) - dynamic_trace->KalmanInfo.StateEst(0)) > 10.0 &&
         fabs(static_trace->kf_info.X_(1) - dynamic_trace->KalmanInfo.StateEst(1)) > 10.0)) {
    }
    else
    {
      creat_rect_box_point(
          dynamic_trace->KalmanInfo.StateEst(0), dynamic_trace->KalmanInfo.StateEst(1),
          dynamic_trace->ExtendInfo.Length, dynamic_trace->ExtendInfo.Width,
          -1.0 * dynamic_trace->ExtendInfo.box_theta, r2);

      double inter = intersection_area(r1, r2);

      double iou = inter / (calcularea(r1) + calcularea(r2) - inter);

      if (iou > 0.0) {
        /* 对于临时静态航迹，若iou>0，则直接删除
         * 对于确认静态航迹，若iou>0且相交面积/静态航迹面积 > 0.5，则删除该航迹
        */
        if (static_trace->status == 1) {
          result = false;
        }
        else
        {
            if((iou / calcularea(r1)) > 0.3)
            {
                result = false;
            }
        }
      }
    }

    return result;
}

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @param {trackTable_strcut} *dynamic_trace_1
 * @param {trackTable_strcut} *dynamic_trace_2
 * @return {*}
 */
angle_enum compute_two_angle_position(std::pair<double, double> angle1, std::pair<double, double> angle2)
{
    double opening_angle1 = angle1.second - angle1.first;
    double opening_angle2 = angle2.second - angle2.first;

    double Opening_angle = 0;

    std::vector<double> angle_list{angle1.first, angle1.second, angle2.first, angle2.second};

    auto minmax = std::minmax_element(angle_list.begin(), angle_list.end());

    // 计算开角
    Opening_angle = *minmax.second - *minmax.first;

    if(Opening_angle >= (opening_angle1 + opening_angle2)) // 不相交
    {
        return ANGLE_NULLSET;
    }
    else
    {
        if((Opening_angle == opening_angle1) || (Opening_angle == opening_angle2)) // 某角度完全重叠
        {
            return ANGLE_OVERLAP;
        }
        else //部分重叠（相交）
        {
            return ANGLE_CROSS;
        }
    }
}

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @param {trackTable_strcut} *dynamic_trace_1
 * @param {trackTable_strcut} *dynamic_trace_2
 * @return {*}
 */
double compute_two_angle_iou(std::pair<double, double> angle1, std::pair<double, double> angle2)
{
    double opening_angle1 = angle1.second - angle1.first;
    double opening_angle2 = angle2.second - angle2.first;

    double Opening_angle = 0;

    std::vector<double> angle_list{angle1.first, angle1.second, angle2.first, angle2.second};

    auto minmax = std::minmax_element(angle_list.begin(), angle_list.end());

    // 计算开角
    Opening_angle = *minmax.second - *minmax.first;

    if(Opening_angle >= (opening_angle1 + opening_angle2)) // 不相交
    {
        return 0.0;
    }
    else
    {
        return ((opening_angle1 + opening_angle2 - Opening_angle)/Opening_angle);
    }
}


/**
 * @names: 
 * @description: Briefly describe the function of your function
 * @return {*}
 */
void free_mem(trackTable_strcut *dyn_track_list)
{
  for (uint16_t track_i = 0; track_i < MAXTRACKS; track_i++) {
      dyn_track_list[track_i].MeasInfo.associateNum = 0;
      dyn_track_list[track_i].MeasInfo.HighQuaPointNum = 0;
      std::vector<matchInfo_t>().swap(dyn_track_list[track_i].MeasInfo.detInfo);
  }
}