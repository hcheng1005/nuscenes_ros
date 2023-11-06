#include "radar_track_process.h"
#include "match.h"
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

// #define VISUALIZATION (true)
#ifdef VISUALIZATION
cv::Mat image;
#endif

void RadarTrackAlgProcess::trackProc(const float dt, std::vector<RadarType::radarPoint_t> &measSet)
{
    // 航迹预测
    trackPredict(dt);

    matchTraceWithMeas(measSet);
}

void RadarTrackAlgProcess::trackPredict(const float dt)
{
    for (auto &sub_trace : radarTraceTable)
    {
        sub_trace.trace_predict(); // 航迹状态预测
    }
}

void RadarTrackAlgProcess::matchTraceWithMeas(std::vector<RadarType::radarPoint_t> &measSet)
{
    std::vector<rect_basic_struct> traceBoxes, measBoxes;
    std::vector<std::vector<float>> costMatrix;
    std::vector<std::vector<int>> traceMatchedMeas;
    std::vector<int> measMatchedResult;

    // 先将measSet聚类成boxes
    auto radarClusters = pointCluster(measSet);

    genTraceBoxes(radarTraceTable, traceBoxes);

    genMeasClusterBoxes(radarClusters, measBoxes);

    // 计算代价矩阵
    genCostMatrixIOU(traceBoxes, measBoxes, costMatrix);

    // 第一次分配
    matchAlgGreedy(costMatrix, traceMatchedMeas, measMatchedResult);
    
    // 第二次分配：补充关联
}

void 

/**
 * @names: point_cluster
 * @description: 点云聚类
 * @param {vector<radar_point_t>} &new_meas
 * @return {*}
 */
std::vector<RadarType::radarCluster_t> RadarTrackAlgProcess::pointCluster(std::vector<RadarType::radarPoint_t> &measSet)
{
    std::vector<std::vector<uint16_t>> clusterSet;
    DBSCAN::Point4DBSCAN Point;
    std::vector<DBSCAN::Point4DBSCAN> PointSet;

#ifdef VISUALIZATION
    image = cv::Mat::zeros(600, 800, CV_8UC3); // 宽800，高600，3通道图像
#endif

    /* 设置点云DBSCAN参数 */
    std::vector<cv::Point2f> point_cloud_orin;
    for (uint16_t n = 0; n < measSet.size(); n++)
    {
        auto &sub_meas = measSet.at(n);

        // 添加点到点云数据
#ifdef VISUALIZATION
        cv::circle(image,
                   cv::Point2f((sub_meas.y_cc + 100) / 200 * 800, 600 - sub_meas.x_cc / 100 * 600),
                   3, cv::Scalar(200, 200, 200), -1);
#endif

        if ((sub_meas.x_cc < 0.0) || (fabs(sub_meas.vr_compensated) < 1.0) || (sub_meas.valid == false))
        {
            continue;
        }

        Point.PointInfo.ID = n;

        Point.PointInfo.DistLat = sub_meas.y_cc;
        Point.PointInfo.DistLong = sub_meas.x_cc;

        Point.PointInfo.Range = sub_meas.range_sc;
        Point.PointInfo.Azi = sub_meas.azimuth_sc;
        Point.PointInfo.V = sub_meas.vr;
        Point.PointInfo.RCS = sub_meas.rcs;

        // Point.PointInfo.DynProp = fifo_point[n].DynProp;
        Point.PointInfo.valid = true;
        // Point.PointInfo.prob_exit = fifo_point[n].ProbOfExist;

        Point.DBSCAN_para.Search_R = 2.5F;
        Point.DBSCAN_para.minPts = 2;
        Point.DBSCAN_para.pointType = 255;
        Point.DBSCAN_para.static_or_dyna = 0;

        PointSet.push_back(Point);
    }
    // std::cout << "DO KNN_DBSCAN " << std::endl;

    // Step 1: cluster
    DBSCAN::KNN_DBSCAN(PointSet, clusterSet);

    // std::cout << "Cluster Result:[ " << clusterSet.size() << " ]" << std::endl;///////
#ifdef VISUALIZATION
    int idx = 0;
    for (auto &sub_cluster : clusterSet)
    {
        std::vector<cv::Point2f> point_cloud;
        for (auto &pc : sub_cluster)
        {
            // 添加点到点云数据
            point_cloud.push_back(cv::Point2f((PointSet.at(pc).PointInfo.DistLat + 100) / 200 * 800,
                                              600 - PointSet.at(pc).PointInfo.DistLong / 100 * 600));
        }

        // 显示图像
        for (const cv::Point2f &point : point_cloud)
        {
            cv::circle(image, point, 2, cv::Scalar(0, idx * 20, 50 + idx * 20), -1);
        }

        idx++;
    }
#endif

    // Step 2: box fitting
    std::vector<RadarType::radarCluster_t> radar_cluters;
    for (auto &sub_cluster : clusterSet)
    {
        RadarType::radarCluster_t radar_cluster;
        radar_cluster.pc_idx = sub_cluster;

        std::vector<float> len_vec, wid_vec;
        for (const auto &pc : radar_cluster.pc_idx)
        {
            len_vec.push_back(PointSet.at(pc).PointInfo.DistLong);
            wid_vec.push_back(PointSet.at(pc).PointInfo.DistLat);
        }

        auto l = std::minmax_element(len_vec.begin(), len_vec.end());
        auto w = std::minmax_element(wid_vec.begin(), wid_vec.end());

        radar_cluster.center[1] = (*l.first + *l.second) * 0.5;
        radar_cluster.center[0] = (*w.first + *w.second) * 0.5;

        radar_cluster.len = *l.second - *l.first;
        radar_cluster.wid = *w.second - *w.first;

        if (radar_cluster.len < 0.5)
        {
            radar_cluster.len = 0.5;
        }
        if (radar_cluster.wid < 0.5)
        {
            radar_cluster.wid = 0.5;
        }

        // std::cout << "Center Pos: " << radar_cluster.center[0] << ", "
        //           << radar_cluster.center[1] << "] " << std::endl;

        radar_cluters.push_back(radar_cluster);

#ifdef VISUALIZATION
        cv::Rect rect((radar_cluster.center[0] - radar_cluster.wid * 0.5 + 100) / 200 * 800,
                      600 - (radar_cluster.center[1] + radar_cluster.len * 0.5) / 100 * 600,
                      radar_cluster.wid / 200 * 800,
                      radar_cluster.len / 100 * 600); // 定义矩形框，左上角坐标 (200, 200)，宽高 (200, 200)

        cv::rectangle(image, rect, cv::Scalar(125, 125, 125), 2); // 绿色矩形框
#endif
    }

    return radar_cluters;
}

void RadarTrackAlgProcess::genTraceBoxes(std::vector<RadarTracker> &radarTraceList, std::vector<rect_basic_struct> &traceBoxes)
{

    std::vector<rect_basic_struct> traceBoxes;
}

void RadarTrackAlgProcess::genMeasClusterBoxes(std::vector<RadarType::radarCluster_t> &radarClusters,
                                               std::vector<rect_basic_struct> &measBoxes)
{
    std::vector<rect_basic_struct> measBoxes;
}

void RadarTrackAlgProcess::genCostMatrixIOU(std::vector<rect_basic_struct> &traceBoxes,
                                            std::vector<rect_basic_struct> &measBoxes,
                                            std::vector<std::vector<float>> &costMatrix)
{
    costMatrix.clear();

    for (const auto &measBox : measBoxes)
    {
        std::vector<float> subCostMatrix;
        for (const auto &traceBox : traceBoxes)
        {
            float IOU_ = static_cast<float>(IOU_2D(measBox, traceBox));
            subCostMatrix.push_back(IOU_);
        }

        costMatrix.push_back(subCostMatrix);
    }
}