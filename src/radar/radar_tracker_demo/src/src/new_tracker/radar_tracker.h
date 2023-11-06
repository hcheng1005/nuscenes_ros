/*
 * @Description:
 * @version:
 * @Author: ChengHao
 * @Date: 2022-10-12 08:58:47
 * @LastEditors: CharlesCH hcheng1005@gmail.com
 * @LastEditTime: 2023-11-06 20:28:00
 */
#pragma once

// Eigen
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

using Eigen::MatrixXf;
using Eigen::VectorXf;

#define SSIZE 6
#define MSIZE 2

#define iDistLat 0
#define iDistLong 1
#define iVrelLat 2
#define iVrelLong 3
#define iAccLat 4
#define iAccLong 5

// 用于航迹管理（起始、消亡）
struct trace_manager_t
{
    int age;
    int id;
    int match_count = 0;
    int unmatched_count = 0;

    float prob;
};

typedef struct trace_shape_t
{
    float len;
    float wid;
    float theta;
};

typedef struct trace_kalman_t
{
    VectorXf X = VectorXf(SSIZE);
    MatrixXf P = MatrixXf(SSIZE, SSIZE);

    MatrixXf Q = MatrixXf(SSIZE, SSIZE);

    MatrixXf R = MatrixXf(MSIZE, MSIZE);

    MatrixXf F = MatrixXf(SSIZE, SSIZE);
    MatrixXf H = MatrixXf(MSIZE, SSIZE);
};

typedef struct trace_status_t
{
    trace_kalman_t trace_kalman;
    trace_manager_t trace_manager;
    trace_shape_t trace_shape;
};

class RadarTracker
{
public:
    RadarTracker(int new_id, float center_lat, float center_long, float len, float wid, float theta);
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

    void trace_update_kinematic(const VectorXf &Z);

    void trace_update_physical(float new_len, float new_wid, float new_theta);

private:
    trace_status_t trace_status;
};
