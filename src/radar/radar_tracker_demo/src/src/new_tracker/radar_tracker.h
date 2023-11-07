/*
 * @Description:
 * @version:
 * @Author: ChengHao
 * @Date: 2022-10-12 08:58:47
 * @LastEditors: ChengHao hao.cheng@wuzheng.com
 * @LastEditTime: 2023-11-07 18:56:05
 */
#pragma once

// Eigen
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

using Eigen::MatrixXf;
using Eigen::VectorXf;

#define SSIZE 6
#define MSIZE 3

#define iDistLat 0
#define iDistLong 1
#define iVrelLat 2
#define iVrelLong 3
#define iAccLat 4
#define iAccLong 5


typedef enum {
  TRK_Invalid = 0,
  TRK_Detected,
  TRK_Confirmed,
  TRK_Delete
} trace_status_enum;

// 用于航迹管理（起始、消亡）
struct trace_manager_t
{
    uint age;
    uint id;
    uint match_count = 0;
    uint unmatched_count = 0;

    trace_status_enum status;

    float prob;
};

struct trace_shape_t
{
    float len;
    float wid;
    float theta;
};

struct trace_kalman_t
{
    VectorXf X = VectorXf(SSIZE);
    MatrixXf P = MatrixXf(SSIZE, SSIZE);

    MatrixXf Q = MatrixXf(SSIZE, SSIZE);

    MatrixXf R = MatrixXf(MSIZE, MSIZE);

    MatrixXf F = MatrixXf(SSIZE, SSIZE);
    MatrixXf H = MatrixXf(MSIZE, SSIZE);
};

struct trace_status_t
{
    trace_kalman_t trace_kalman;
    trace_manager_t trace_manager;
    trace_shape_t trace_shape;
};

class RadarTracker
{
public:
    RadarTracker(uint new_id, float center_lat, float center_long, float vr, float len, float wid, float theta);
    ~RadarTracker() {}

    void trace_predict();

private:
    /**
     * @names:
     * @description: Briefly describe the function of your function
     * @param {double} std_accLat_
     * @param {double} std_accLong_
     * @return {*}
     */
    void Set_Q(double std_accLat_, double std_accLong_)
    {
        MatrixXf G = MatrixXf::Zero(SSIZE, MSIZE);
        MatrixXf q = MatrixXf::Zero(MSIZE, MSIZE);

        double dt = 0.1;
        double dt3 = powf(dt, 3.0);
        double dt2 = powf(dt, 2.0);

        G << 1 / 6 * dt3, 0, 0, 1 / 6 * dt3, 1 / 2 * dt2, 0,
            0, 1 / 2 * dt2, dt, 0, 0, dt;

        q << pow(std_accLong_, 2.0), 0.0, 0.0, pow(std_accLat_, 2.0);

        trace_status.trace_kalman.Q = G * q * G.transpose();
    }

    /**
     * @names:
     * @description: Briefly describe the function of your function
     * @return {*}
     */
    void Set_R(void)
    {
        trace_status.trace_kalman.R << std::pow(0.5, 2.0), 0.0, 0.0, std::pow(0.5, 2.0);
    }

    /**
     * @names:
     * @description: Briefly describe the function of your function
     * @return {*}
     */
    void Set_F(void)
    {
        const float dt = 0.075;
        trace_status.trace_kalman.F = trace_status.trace_kalman.F.setIdentity();
        trace_status.trace_kalman.F(iDistLat, iVrelLat) = dt;
        trace_status.trace_kalman.F(iDistLat, iAccLat) = 0.5 * dt * dt;
        trace_status.trace_kalman.F(iVrelLat, iAccLat) = dt;

        trace_status.trace_kalman.F(iDistLong, iVrelLong) = dt;
        trace_status.trace_kalman.F(iDistLong, iAccLong) = 0.5 * dt * dt;
        trace_status.trace_kalman.F(iVrelLong, iAccLong) = dt;
    }

    /**
     * @names:
     * @description: Briefly describe the function of your function
     * @return {*}
     */
    void Set_H(void)
    {
        trace_status.trace_kalman.H = trace_status.trace_kalman.H.setIdentity();
    }

    void computeJacMat(void)
    {
        double x = trace_status.trace_kalman.X[iDistLat];
        double y = trace_status.trace_kalman.X[iDistLong];
        double vx = trace_status.trace_kalman.X[iVrelLat];
        double vy = trace_status.trace_kalman.X[iVrelLong];

        double range_;
        range_ = sqrtf(x * x + y * y);

        // // 计算雅可比矩阵（RAV）
        // trace_status.trace_kalman.H << x / range_, y / range_, 0, 0, 0, 0,
        //                                 y / (range_ * range_), -x / (range_ * range_), 0, 0, 0, 0,
        //                                 y * (vx * y - vy * x) / (range_ * range_ * range_),
        //                                 x * (vy * x - vx * y) / (range_ * range_ * range_), x / range_, y / range_,
        //                                 0, 0;

        // 计算雅可比矩阵（XYV）
        trace_status.trace_kalman.H << 1.0, 0, 0, 0, 0, 0,
                                        0, 1.0, 0, 0, 0, 0,
                                        vx / range_ - x * (vx * x + vy * y) / pow(range_, 3.0),
                                        vy / range_ - y * (vx * x + vy * y) / pow(range_, 3.0), x / range_, y / range_,
                                        0, 0;
    }
    


public:
    trace_status_t trace_status;

public:
    void update_kinematic(const VectorXf &Z);

    void update_physical(float new_len, float new_wid, float new_theta);

    void manager(bool matchedFlag);
};
