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

namespace RandomMatrice
{
    class Tracker
    {

    public:
        Tracker(VectorXf new_Z, float len, float wid, int new_ID)
        {
            X = X.setZero();
            X(iDistLat) = new_Z(iDistLat);
            X(iDistLong) = new_Z(iDistLong);

            P = P.setIdentity() * 4;

            X_ex = X_ex.setIdentity();
            X_ex(0,0) = 0.25 * len * len;
            X_ex(1,1) = 0.25 * wid * wid;

            Set_Q(0.4, 0.4);
            Set_R();
            Set_F();
            Set_H();

            id = new_ID;
        }

        ~Tracker()
        {
        }

        /**
         * @names:
         * @description: Briefly describe the function of your function
         * @return {*}
         */
        void Predict_(void)
        {
            Predict_State();
            Predict_ExState();

            age++;
            matched = false;
        }

        VectorXf get_X(void)
        {
            return X;
        }

        /**
         * @names:
         * @description: Briefly describe the function of your function
         * @return {*}
         */
        VectorXf Get_Z(void)
        {
            return (H * X);
        }

        /**
         * @names:
         * @description: Briefly describe the function of your function
         * @param {VectorXf} Z_meas
         * @return {*}
         */
        float Compute_Meas_Likelihood(VectorXf Z_meas)
        {
            VectorXf mu_ = H * X;
            MatrixXf phi_ = X_ex + R;
            VectorXf diff_ = Z_meas - mu_;

            float likelihood = 0.0;
            if ((fabs(diff_(0)) > 5.0) || (fabs(diff_(1)) > 5.0))
            {
                likelihood = 0.0;
            }
            else
            {
                float part1 = 1.0 / pow(2.0 * M_PI, 1) / sqrt(phi_.determinant());

                likelihood = part1 * exp(-0.5 * diff_.transpose() * phi_.inverse() * diff_);

                // std::cout << "diff: " << diff_(0) << " ," << diff_(1) << " , likelihood: " << likelihood << std::endl;
            }

            return likelihood;
        }

        /**
         * @names:
         * @description: Briefly describe the function of your function
         * @param {MatrixXf} meas_mat
         * @return {*}
         */
        void Update_State(MatrixXf meas_mat)
        {
            if (meas_mat.rows() > 2)
            {
                Update_With_Extension(meas_mat);
            }
            else
            {
                Update_Without_Extension(meas_mat);
            }

            matched = true;
            match_count++;
            unmatch_cout = 0;
        }

        /**
         * @names:
         * @description: Briefly describe the function of your function
         * @return {*}
         */
        bool trace_valid(void)
        {
            bool flag = true;
            if (matched)
            {
                match_count++;
                unmatch_cout = 0;
            }
            else
            {
                match_count = 0;
                unmatch_cout++;
            }

            if (status == 0) // unactivated
            {
                if (match_count >= 4)
                {
                    status = 1;
                }
                else
                {
                    if ((unmatch_cout >= 3) || ((unmatch_cout >= 1) && (age < 5)))
                    {
                        flag = false;
                    }
                }
            }
            else
            {
                if (unmatch_cout >= 3)
                {
                    status = 0;
                    flag = false;
                }
            }

            return flag;
        }

    private:
        int id = 0;
        int age = 1;
        float dt = 0.075;

        int status = 0;

        bool matched = false;
        int match_count = 0;
        int unmatch_cout = 0;

        VectorXf X = VectorXf(SSIZE);
        MatrixXf P = MatrixXf(SSIZE, SSIZE);

        MatrixXf Q = MatrixXf(SSIZE, SSIZE);

        MatrixXf R = MatrixXf(MSIZE, MSIZE);

        MatrixXf F = MatrixXf(SSIZE, SSIZE);
        MatrixXf H = MatrixXf(MSIZE, SSIZE);

        // vari for extendsion
        MatrixXf X_ex = MatrixXf(MSIZE, MSIZE);
        float alpha_ = 2.0;
        float tau_ = 1.0;

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

            Q = G * q * G.transpose();
        }

        /**
         * @names:
         * @description: Briefly describe the function of your function
         * @return {*}
         */
        void Set_R(void)
        {
            R << std::pow(0.5, 2.0), 0.0, 0.0, std::pow(0.5, 2.0);
        }

        /**
         * @names:
         * @description: Briefly describe the function of your function
         * @return {*}
         */
        void Set_F()
        {
            F = F.setIdentity();
            F(iDistLat, iVrelLat) = dt;
            F(iDistLat, iAccLat) = 0.5 * dt * dt;
            F(iVrelLat, iAccLat) = dt;

            F(iDistLong, iVrelLong) = dt;
            F(iDistLong, iAccLong) = 0.5 * dt * dt;
            F(iVrelLong, iAccLong) = dt;
        }

        /**
         * @names:
         * @description: Briefly describe the function of your function
         * @return {*}
         */
        void Set_H()
        {
            H = H.setIdentity();
        }

        /**
         * @names:
         * @description: Briefly describe the function of your function
         * @return {*}
         */
        void Predict_State()
        {
            X = F * X;
            P = F * P * F.transpose() + Q;
        }

        /**
         * @names:
         * @description: Briefly describe the function of your function
         * @return {*}
         */
        void Predict_ExState() // extension info
        {
            // X_ex keep the same
            alpha_ = 2.0 + exp(-dt / tau_) * (alpha_ - 2.0);

            // make sure alpha_ is bigger than 2.0
            alpha_ = (alpha_ < 2.0) ? 2.01 : alpha_;
        }

        /**
         * @names:
         * @description: Briefly describe the function of your function
         * @param {MatrixXf} meas_mat
         * @return {*}
         */
        void Update_Without_Extension(MatrixXf meas_mat)
        {
            MatrixXf Yk = 0.9 * X_ex + R;
            MatrixXf S = H * P * H.transpose() + Yk;
            MatrixXf K = P * H.transpose() * S.inverse();
            VectorXf y_hat = meas_mat.colwise().mean();
            X = X + K * (y_hat - H * X);
            P = P - K * S * K.transpose();
        }

        /**
         * @names:
         * @description: Briefly describe the function of your function
         * @param {MatrixXf} meas_mat
         * @return {*}
         */
        void Update_With_Extension(MatrixXf meas_mat)
        {
            int meas_num = meas_mat.rows();
            VectorXf y_hat = meas_mat.colwise().mean();
            Eigen::RowVectorXf Z_mean(Eigen::RowVectorXf::Map(y_hat.data(), meas_mat.cols()));
            Eigen::MatrixXf ZK = meas_mat.rowwise() - Z_mean;

            MatrixXf Yhat = (ZK.adjoint() * ZK) / (meas_num - 1);

            // compute kalman S and K
            MatrixXf Yk = 0.9 * X_ex + R;
            MatrixXf S = H * P * H.transpose() + Yk / meas_num;
            MatrixXf K = P * H.transpose() * S.inverse();

            X = X + K * (y_hat - H * X);
            P = P - K * S * K.transpose();
            MatrixXf N_ = (y_hat - H * X) * ((y_hat - H * X).transpose());

            MatrixXf X_chol = X_ex.ldlt().matrixL();
            MatrixXf S_chol = S.ldlt().matrixL();
            // MatrixXf N_chol = N_.ldlt().matrixL();
            MatrixXf Yk_chol = Yk.ldlt().matrixL();

            MatrixXf N_hat = X_chol * S_chol.inverse() * N_ * S_chol.inverse().transpose() * X_chol.transpose();
            MatrixXf Y_hat = X_chol * Yk_chol.inverse() * Yhat * Yk_chol.inverse().transpose() * X_chol.transpose();

            X_ex = 1.0 / (alpha_ + meas_num) * (alpha_ * X_ex + N_hat + Y_hat);
            alpha_ = alpha_ + meas_num;


            std::cout << "SPD Matrice:" << X_ex(0,0) << " ," << X_ex(1,1) << std::endl;
        }
    };

} // namespace name
