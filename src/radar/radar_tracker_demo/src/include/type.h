#ifndef TYPES_H_
#define TYPES_H_

namespace RadarDemo
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
    } radar_point_t;

    typedef struct
    {
        std::vector<size_t> pc_idx;
        T center[2]; //
        T len;
        T wid;

    } radar_cluster_t; // 雷达量测聚类结果
}

#endif