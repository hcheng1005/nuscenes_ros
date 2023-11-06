#pragma once

#include <vector>
#include "inttypes.h"

namespace RadarType
{
    typedef float T;

    typedef struct
    {
        T range_sc;
        T azimuth_sc;
        T vr;
        T rcs;
        T vr_compensated;
        T x_cc;
        T y_cc;
        T x_seq;
        T y_seq;
        bool valid = true;
    } radarPoint_t;

    typedef struct
    {
        std::vector<uint16_t> pc_idx;
        T center[2]; //
        T len;
        T wid;
        T theta;

    } radarCluster_t; // 雷达量测聚类结果

}
