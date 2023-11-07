#pragma once

#include "basic_type.h"
#include "radar_tracker.h"
#include "../../include/common/iou.h"
#include "../../include/common/lshape.h"

#include "common/DBSCAN.h"
#include <vector>

class RadarTrackAlgProcess
{

public:
    RadarTrackAlgProcess(/* args */);
    ~RadarTrackAlgProcess();

    void trackProc(const float dt, std::vector<RadarType::radarPoint_t> &measSet);

private:
    void trackPredict(const float dt);

    void matchTraceWithMeas(std::vector<RadarType::radarPoint_t> &measSet);

    std::vector<RadarType::radarCluster_t> pointCluster(std::vector<RadarType::radarPoint_t> &measSet);

    void genTraceBoxes(std::vector<RadarTracker> &radarTraceList,
                       std::vector<rect_basic_struct> &traceBoxes);
    void genMeasClusterBoxes(std::vector<RadarType::radarCluster_t> &radarClusters,
                             std::vector<rect_basic_struct> &measBoxes);
    void genCostMatrixIOU(std::vector<rect_basic_struct> &traceBoxes,
                          std::vector<rect_basic_struct> &measBoxes,
                          std::vector<std::vector<float>> &costMatrix);

    void genMatchedBoxes(std::vector<RadarType::radarPoint_t> &measSet,
                         std::vector<RadarType::radarCluster_t> &radarClusters,
                         std::vector<std::vector<int>> &traceMatchedMeas);

    void trackManager(void);

    void traceBirth(std::vector<RadarType::radarCluster_t> &radarClusters,
                    std::vector<int> &measMatchedResult);

    void genFinlalMatchedBox(std::vector<RadarType::radarPoint_t> &RadarDets,
                             Rect_t &matchedBox);

public:
    std::vector<RadarTracker> radarTraceTable;

private:
    std::vector<rect_basic_struct> traceBoxes, measBoxes;
};
