#include "radar_track_process.h"
#include "match.h"
#include "../../include/common/lshape.h"


// OPENCV
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

#define VISUALIZATION (true)
#ifdef VISUALIZATION
cv::Mat image;
#endif


RadarTrackAlgProcess::RadarTrackAlgProcess(/* args */)
{
}

RadarTrackAlgProcess::~RadarTrackAlgProcess()
{
}


void RadarTrackAlgProcess::trackProc(const float dt, std::vector<RadarType::radarPoint_t> &measSet)
{
    std::cout << " -------- trackProc START -------- " << std::endl;
    trackPredict(dt);

    matchTraceWithMeas(measSet);

    std::cout << "Do trackManager " << std::endl;

    trackManager();

    std::cout << " -------- trackProc END -------- " << std::endl;
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
    std::vector<std::vector<float>> costMatrix;

    std::vector<std::vector<RadarType::radarPoint_t>> traceMatchDets; // 航迹关联的所有点云
    std::vector<Rect_t> traceMatchBoxes; // 航迹关联的点云构成的bounding box

    // 先将measSet聚类成boxes
    auto radarClusters = pointCluster(measSet);

    // 分别构造航迹矩形框和量测矩形框
    genTraceBoxes(radarTraceTable, this->traceBoxes);
    genMeasClusterBoxes(radarClusters, this->measBoxes);

    std::cout << "traceBoxes size: " << traceBoxes.size() << ", "
                << "measBoxes size: " << measBoxes.size() << std::endl;

    // 计算代价矩阵
    std::cout << "Do genCostMatrixIOU " << std::endl;
    genCostMatrixIOU(traceBoxes, measBoxes, costMatrix);

    std::vector<int> measMatchedResult(measBoxes.size(), 0);
    std::vector<std::vector<int>> traceMatchedMeas(traceBoxes.size());

    // 第一次分配
    std::cout << "Do matchAlgGreedy " << std::endl;
    matchAlgGreedy(costMatrix, traceMatchedMeas, measMatchedResult);
    
    // TODO 第二次分配：补充关联

    // 整理最后的分配结果
    std::cout << "Do genMatchedBoxes " << std::endl;
    genMatchedBoxes(measSet, radarClusters, traceMatchedMeas, traceMatchDets, traceMatchBoxes);

    // std::cout << "Do traceUpdate " << std::endl;
    // traceUpdate(traceMatchedMeas, traceMatchDets, traceMatchBoxes);

    // 起始新航迹
    std::cout << "Do traceBirth " << std::endl;
    traceBirth(radarClusters, measMatchedResult);
}


void RadarTrackAlgProcess::genMatchedBoxes(std::vector<RadarType::radarPoint_t> &measSet,
                                            std::vector<RadarType::radarCluster_t> &radarClusters,
                                            std::vector<std::vector<int>> &traceMatchedMeas,
                                            std::vector<std::vector<RadarType::radarPoint_t>> &traceMatchDets,
                                            std::vector<Rect_t> &traceMatchBoxes)
{
    traceMatchBoxes.clear();
    traceMatchDets.clear();

    RadarTracker *subTrace = nullptr;
    uint traceIdx = 0;
    for(const auto &sunMatchClusters:traceMatchedMeas)
    {
        if(!sunMatchClusters.empty()) // 该航迹未匹配到量测
        {
            std::cout << "sunMatchClusters.size() " << sunMatchClusters.size() << std::endl;

            std::vector<RadarType::radarPoint_t> subRadarDets;
            for(const auto &subClusterIdx : sunMatchClusters)
            {
                auto &Cluster = radarClusters.at(subClusterIdx);
                for(const auto &radarPointIdx : Cluster.pc_idx)
                {
                    subRadarDets.push_back(measSet.at(radarPointIdx));
                }
            }

            traceMatchDets.push_back(subRadarDets);

            std::cout << "subRadarDets.size() " << subRadarDets.size() << std::endl;

            // box fitting
            Eigen::MatrixXd point = Eigen::MatrixXd(subRadarDets.size(), 2);

            uint idx = 0;
            for(const auto&subDet : subRadarDets)
            {
                point(idx, 0) = subDet.x_cc;
                point(idx, 1) = subDet.y_cc;
                idx++;
            }

            Rect_t matchedBox = L_shape_Fit_Proc(point, 0.0, M_PI_2, M_PI_2 / 45.0);
            // traceMatchBoxes.push_back(matchedBox);

            // 航迹状态更新
            subTrace = &radarTraceTable.at(traceIdx);
            Eigen::VectorXf Z(2);
            Z << matchedBox.center_lat, matchedBox.center_long;
            subTrace->update_kinematic(Z);
            subTrace->update_physical(matchedBox.length, matchedBox.width, 0.0); // TODO DONT CARE THETA??
            subTrace->manager(true);
        }
        else{
            subTrace = &radarTraceTable.at(traceIdx);
            subTrace->manager(false);
        }

        traceIdx++;
    }
}


/**
 * @names: point_cluster
 * @description: 点云聚类
 * @param {vector<radar_point_t>} &new_meas
 * @return {*}
 */
std::vector<RadarType::radarCluster_t> RadarTrackAlgProcess::pointCluster(std::vector<RadarType::radarPoint_t> &measSet)
{
    DBSCAN::Point4DBSCAN Point;
    std::vector<std::vector<uint16_t>> clusterSet;
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
        Point.DBSCAN_para.minPts = 1;
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

#ifdef VISUALIZATION
    cv::imshow("Point Cloud Visualization", image);
    cv::waitKey(50);
#endif

    return radar_cluters;
}

void RadarTrackAlgProcess::genTraceBoxes(std::vector<RadarTracker> &radarTraceList, 
                                         std::vector<rect_basic_struct> &traceBoxes)
{
    traceBoxes.clear();
    for(const auto& subTrace : radarTraceList)
    {
        rect_basic_struct subBox{{subTrace.trace_status.trace_kalman.X(iDistLong ),
                                    subTrace.trace_status.trace_kalman.X(iDistLat), 0.0}, 
                                    subTrace.trace_status.trace_shape.len, subTrace.trace_status.trace_shape.wid,0.0,subTrace.trace_status.trace_shape.theta,
                                    0.0};
        traceBoxes.push_back(subBox);
    }     
}

void RadarTrackAlgProcess::genMeasClusterBoxes(std::vector<RadarType::radarCluster_t> &radarClusters,
                                               std::vector<rect_basic_struct> &measBoxes)
{
    measBoxes.clear();
    for(const auto& subCluster : radarClusters)
    {
        rect_basic_struct subBox{{subCluster.center[1], subCluster.center[0], 0.0},
                                  subCluster.len, subCluster.wid, 0.0, subCluster.theta,
                                  0.0};
        measBoxes.push_back(subBox);
    }                                      
}

void RadarTrackAlgProcess::genCostMatrixIOU(std::vector<rect_basic_struct> &traceBoxes,
                                            std::vector<rect_basic_struct> &measBoxes,
                                            std::vector<std::vector<float>> &costMatrix)
{
    costMatrix.clear();

    for (const auto &measBox : measBoxes)
    {
        std::vector<float> subCostMatrix;

        std::cout << "measBox info: " << measBox.center_pos[0] << ", " << measBox.center_pos[1] << ", "
                    << measBox.box_len << ", " << measBox.box_wid << std::endl;

        for (const auto &traceBox : traceBoxes)
        {
            std::cout << "traceBox info: " << traceBox.center_pos[0] << ", " << traceBox.center_pos[1]  << ", "
                         << traceBox.box_len << ", " << traceBox.box_wid << std::endl;

            float IOU_ = static_cast<float>(IOU_2D(measBox, traceBox));

            std::cout << "IOU_ " << IOU_ << std::endl;
            
            subCostMatrix.push_back(IOU_);
        }

        costMatrix.push_back(subCostMatrix);
    }
}

void RadarTrackAlgProcess::traceUpdate(std::vector<std::vector<int>> &traceMatchedMeas,
                                        std::vector<std::vector<RadarType::radarPoint_t>> &traceMatchDets,
                                        std::vector<Rect_t> &traceMatchBoxes)
{
    uint traceIdx = 0;
    std::vector<RadarTracker>::iterator itor;

    for(itor = radarTraceTable.begin(); itor != radarTraceTable.end(); itor++)
    {
        if(!traceMatchedMeas.empty())
        {
            if(!traceMatchedMeas.at(traceIdx).empty())
            {
                // 获取匹配的bounding box
                Rect_t &matchedBox = traceMatchBoxes.at(traceIdx);

                // 航迹状态更新
                Eigen::VectorXf Z(2);
                Z << matchedBox.center_lat, matchedBox.center_long;
                (*itor).update_kinematic(Z);
                (*itor).update_physical(matchedBox.length, matchedBox.width, matchedBox.theta);
                (*itor).manager(true);
            }
            else{
                (*itor).manager(false);
            }
        }
        else{
           (*itor).manager(false); 
        }

        if((*itor).trace_status.trace_manager.status == TRK_Delete)
        {
            radarTraceTable.erase(itor);
        }

        traceIdx++;
    }
}

void RadarTrackAlgProcess::trackManager(void)
{
    std::vector<RadarTracker>::iterator itor;
    std::vector<RadarTracker> radarTraceTableNew;

    std::cout << "Final Trace Numer:[ " << radarTraceTable.size() << " ]" << std::endl;

    for(auto &subTrace : radarTraceTable)
    {
        if(subTrace.trace_status.trace_manager.status != TRK_Delete)
        {
            // std::cout << "delete it" << std::endl;
            radarTraceTableNew.push_back(subTrace);
        }
    }

    radarTraceTable = radarTraceTableNew;
}

void RadarTrackAlgProcess::traceBirth(std::vector<RadarType::radarCluster_t> &radarClusters,
                                       std::vector<int> &measMatchedResult)
{
    static uint globalID = 0;
    int clusterIdx = 0;
    for(const auto &subMeas:measMatchedResult)
    {
        if(subMeas == 0) // to create a new trace
        {
            RadarTracker newTrace(globalID, 
                                radarClusters.at(clusterIdx).center[0], radarClusters.at(clusterIdx).center[1],
                                radarClusters.at(clusterIdx).vr,
                                radarClusters.at(clusterIdx).len, radarClusters.at(clusterIdx).wid, radarClusters.at(clusterIdx).theta);
            radarTraceTable.push_back(newTrace);

            globalID++;
        }
        clusterIdx++;
    }
}

