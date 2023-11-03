#include <iostream>

#include "../include/radarTracker.h"
#include "../include/randomMatrix.h"

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

using namespace RadarDemo;

#define VISUALIZATION (true)

// local vari
std::vector<RandomMatrice::Tracker> TraceList;

cv::Mat image;

/**
 * @names: point_cluster
 * @description: 点云聚类
 * @param {vector<radar_point_t>} &new_meas
 * @return {*}
 */
std::vector<radar_cluster_t> point_cluster(std::vector<radar_point_t> &new_meas)
{
    std::vector<std::vector<uint16_t>> clusterSet;
    DBSCAN::Point4DBSCAN Point;
    std::vector<DBSCAN::Point4DBSCAN> PointSet;

    image = cv::Mat::zeros(600, 800, CV_8UC3); // 宽800，高600，3通道图像

    /* 设置点云DBSCAN参数 */
    std::vector<cv::Point2f> point_cloud_orin;
    for (size_t n = 0; n < new_meas.size(); n++)
    {
        auto &sub_meas = new_meas.at(n);

        // 添加点到点云数据
        if (VISUALIZATION)
        {
            cv::circle(image,
                       cv::Point2f((sub_meas.y_cc + 100) / 200 * 800, 600 - sub_meas.x_cc / 100 * 600),
                       3, cv::Scalar(200, 200, 200), -1);
        }

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
    if (VISUALIZATION)
    {
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
    }

    // Step 2: box fitting
    std::vector<radar_cluster_t> radar_cluters;
    for (auto &sub_cluster : clusterSet)
    {
        radar_cluster_t radar_cluster;
        radar_cluster.pc_idx = sub_cluster;

        std::vector<T> len_vec, wid_vec;
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

        std::cout << "Center Pos: " << radar_cluster.center[0] << ", "
                  << radar_cluster.center[1] << "] " << std::endl;

        radar_cluters.push_back(radar_cluster);

        if (VISUALIZATION)
        {
            cv::Rect rect((radar_cluster.center[0] - radar_cluster.wid * 0.5 + 100) / 200 * 800,
                          600 - (radar_cluster.center[1] + radar_cluster.len * 0.5) / 100 * 600,
                          radar_cluster.wid / 200 * 800,
                          radar_cluster.len / 100 * 600); // 定义矩形框，左上角坐标 (200, 200)，宽高 (200, 200)

            cv::rectangle(image, rect, cv::Scalar(125, 125, 125), 2); // 绿色矩形框
        }
    }

    return radar_cluters;
}

/**
 * @names: trace_predict
 * @description: 航迹预测
 * @return {*}
 */
void trace_predict()
{
    std::cout << " Do [trace_predict] start ... " << std::endl;
    for (auto it = TraceList.begin(); it != TraceList.end(); it++)
    {
        (*it).Predict_();
    }
    std::cout << " Do [trace_predict] finish !!! " << std::endl;
}

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @param {vector<radar_point_t>} &new_meas
 * @return {*}
 */
void trace_match(std::vector<radar_point_t> &new_meas)
{
    std::cout << " Do [trace_match] start ... " << std::endl;
    VectorXf meas = VectorXf(2);

    std::vector<std::vector<VectorXf>> match_matrix_set;

    float likelihood_val;

    // step 1: match
    for (auto &trace : TraceList)
    {
        std::vector<VectorXf> trace_match_matrix;
        for (auto &point : new_meas)
        {
            if (point.valid)
            {
                meas << point.y_cc, point.x_cc;

                // 位置似然比
                likelihood_val = trace.Compute_Meas_Likelihood(meas);

                if (likelihood_val > 0.01)
                {
                    trace_match_matrix.push_back(meas);
                    point.valid = false;
                }
            }
        }

        match_matrix_set.push_back(trace_match_matrix);
    }

    // step 2: update
    int idx = 0;
    for (auto &trace : TraceList)
    {
        if (!match_matrix_set.at(idx).empty())
        {
            MatrixXf measSet = MatrixXf(match_matrix_set.at(idx).size(), 2);
            int idx2 = 0;
            for (auto &meas : match_matrix_set.at(idx))
            {
                measSet(idx2, 0) = meas(0);
                measSet(idx2, 1) = meas(1);
                idx2++;
            }
            trace.Update_State(measSet);
        }

        idx++;
    }

    std::cout << " Do [trace_match] finish !!! " << std::endl;
}

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @return {*}
 */
void trace_manage(void)
{
    // 航迹删除
    for (auto it = TraceList.begin(); it != TraceList.end();)
    {
        // (*it).Predict_();
        RandomMatrice::Tracker &trace = (*it);
        if (!trace.trace_valid())
        {
            it = TraceList.erase(it); // check next trace
        }
        else
        {
            it++;
        }
    }
}

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @param {vector<radar_point_t>} &new_meas
 * @return {*}
 */
void trace_birth(std::vector<radar_point_t> &new_meas)
{
    static int global_id = 0;

    // 航迹新生
    std::vector<radar_cluster_t> clusters;
    clusters = point_cluster(new_meas);

    for (auto &sub_cluster : clusters)
    {
        VectorXf new_x = VectorXf(2);

        new_x << sub_cluster.center[0], sub_cluster.center[1];
        std::cout << "new trace:[" << new_x(0) << ", " << new_x(1) << "]" << std::endl;

        RandomMatrice::Tracker trace(new_x, sub_cluster.len, sub_cluster.wid, global_id);
        TraceList.push_back(trace);
        global_id++;
    }
}

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @param {vector<radar_point_t>} &new_meas
 * @return {*}
 */
int radar_track_main(std::vector<radar_point_t> &new_meas)
{
    // Step 2: Trace Predict
    trace_predict();

    // Step 3: Association and Update
    trace_match(new_meas);

    trace_manage();

    trace_birth(new_meas);

    std::cout << "cur trace number: " << TraceList.size() << std::endl;

    if (VISUALIZATION)
    {
        VectorXf X_;
        for (auto &trace : TraceList)
        {
            X_ = trace.get_X();
            auto point = cv::Point2f((X_(0) + 100) / 200 * 800, 600 - (X_(1) / 100 * 600));
            cv::circle(image, point, 5, cv::Scalar(0, 255, 255), -1);
        }

        cv::imshow("Point Cloud Visualization", image);
        cv::waitKey(50);
    }

    return 1;
}