#ifndef DBSCAN_H
#define DBSCAN_H

#include "inttypes.h"

#include <memory>
#include <string>
#include <vector>
#include <math.h>

#define DBSCAN_Valid (1)
#define DBSCAN_InValid (0)

#define MAX_Lat (100.0F)  // from -100 to +100
#define MAX_Long (200.0F) // from +0 to +200
#define Grid_Reso (2.0F)

#define GriDSize_Lat ((size_t)(MAX_Lat * 2.0 / Grid_Reso))
#define GriDSize_Long ((size_t)(MAX_Long * 1.0 / Grid_Reso))

namespace DBSCAN
{
    enum class PointType
    {
        init = 0,
        noise,
        border,
        core,
    };

    typedef struct PointStrut_
    {
        struct
        {
            size_t ID;

            float Range;
            float Azi;
            float DistLat;
            float DistLong;
            float RCS;
            float V;

            float prob_exit;
            size_t scanLat[2];
            size_t scanLong[2];

            bool valid = true;

            uint8_t DynProp;

            PointType type = PointType::init;
        } PointInfo;

        struct
        {
            float Search_R;
            uint8_t pointType;
            size_t minPts;
            uint8_t static_or_dyna;
        } DBSCAN_para;

    } Point4DBSCAN;

    void KNN_DBSCAN(std::vector<Point4DBSCAN> &pointSet, std::vector<std::vector<size_t>> &clusterSet);
    void ComputePara(std::vector<Point4DBSCAN> &pointSet);
    void ScanPoints(size_t start_Idx, std::vector<Point4DBSCAN> &pointSet, std::vector<size_t> *scanResult, uint8_t *minNumer);
    void GridMappingPoint(std::vector<Point4DBSCAN> &pointSet);
}

#endif
