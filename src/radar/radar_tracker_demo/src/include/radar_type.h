#ifndef RADAR_TYPES_H_
#define RADAR_TYPES_H_

#include <vector>
#include <math>

// Eigen
#include <Eigen/Dense>
#include "type.h"

#define X_DIM 6 // [x,y,vx,vy.ax,ay]
#define Z_DIM 2 // [x,y]

// A Simple Demo About Extended Target Tracking By Using "Random Matrix"
namespace RadarDemo
{
    using Eigen::Matrix;

    typedef struct manager_t
    {
        size_t ID;
        size_t Age;
        size_t matched_count = 0;
        size_t unmatched_coun = 0;
        bool valid = false;
    };

    class Radar_trace
    {
    public:
        manager_t manager;

    public:
        Radar_trace(radar_cluster_t &clutser, const size_t id)
        {
            X.setZero();
            X(0) = clutser.center[0];
            X(1) = clutser.center[1];

            P.setIdentity();
            Q = Q.setIdentity() * 0.01;
            R = R.setIdentity() * 0.2;

            F = F.setIdentity();
            F(0, 2) = dt;
            F(0, 3) = 0.5 * dt * dt;
            F(2, 3) = dt;

            F(1, 3) = dt;
            F(1, 4) = 0.5 * dt * dt;
            F(4, 5) = dt;

            H.setIdentity();

            alpha_ = clutser.pc_idx.size();

            manager.Age = 1;
            manager.ID = id;
            manager.matched_count = 1;
            manager.unmatched_coun = 0;
            manager.valid = false;
        }

        void Predict(void)
        {
            State_Predict();
            Extension_Predict();
        }

        void Update(radar_cluster_t &clutser)
        {
            State_Update(clutser);
            Extension_Update();
        }

        ~Radar_trace() {}

    private:
        T dt = 1.0 / 15.0;
        Matrix<T, X_DIM, 1> X;
        Matrix<T, X_DIM, X_DIM> P;
        Matrix<T, X_DIM, X_DIM> Q;
        Matrix<T, X_DIM, X_DIM> F;
        Matrix<T, Z_DIM, X_DIM> H;
        Matrix<T, Z_DIM, Z_DIM> R;

        Matrix<T, X_DIM, 1> X_et;

        T alpha_;
        T tau_ = 0.1;

        void State_Predict(void)
        {
            X = F * X;
            P = F * P * F.transpose() + Q;
        }

        void State_Update(radar_cluster_t &clutser)
        {
            Matrix S = H * P * H.transpose() + R;
            Matrix K = P * H.transpose() * S.inverse();
            Matrix z(2, 1);
            z << clutser.center[0], clutser.center[1];

            X = X + K * (z - H * X);
            P -= K * H * P;
        }

        void Extension_Predict() // 扩展信息预测
        {
            alpha_ = 2 + exp(-dt / tau_) * (alpha_ - 2);
            X_et = X_et;
        }

        void Extension_Update()
        {

            alpha_ += ;
        }
    };

}

#endif