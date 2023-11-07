#include "radar_tracker.h"

RadarTracker::RadarTracker(uint new_id, float center_lat, float center_long, float vr, float len, float wid, float theta)
{
    trace_status.trace_manager.age = 1;
    trace_status.trace_manager.id = new_id;
    trace_status.trace_manager.status = TRK_Detected;
    trace_status.trace_manager.match_count = 1;
    trace_status.trace_manager.unmatched_count = 0;
    trace_status.trace_manager.prob = 0.3;

    trace_status.trace_kalman.X = trace_status.trace_kalman.X.setZero();
    trace_status.trace_kalman.X(iDistLat) = center_lat;
    trace_status.trace_kalman.X(iDistLong) = center_long;
    trace_status.trace_kalman.X(iVrelLat) = vr * sin(atan2(center_lat, center_long));
    trace_status.trace_kalman.X(iVrelLong) = vr * cos(atan2(center_lat, center_long));
    trace_status.trace_kalman.P = trace_status.trace_kalman.P.setOnes();

    trace_status.trace_shape.len = len;
    trace_status.trace_shape.wid = wid;
    trace_status.trace_shape.theta = theta;

    Set_Q(0.2, 0.2);
    Set_R();
    Set_F();
    Set_H();
}

void RadarTracker::trace_predict()
{
    trace_status.trace_kalman.X = trace_status.trace_kalman.F * trace_status.trace_kalman.X;
    trace_status.trace_kalman.P = trace_status.trace_kalman.F * trace_status.trace_kalman.P * trace_status.trace_kalman.P.transpose() + trace_status.trace_kalman.Q;

    computeJacMat();
}

void RadarTracker::update_kinematic(const VectorXf &Z)
{
    MatrixXf S = trace_status.trace_kalman.H * trace_status.trace_kalman.P * trace_status.trace_kalman.H.transpose() + trace_status.trace_kalman.R;
    MatrixXf K = trace_status.trace_kalman.P * trace_status.trace_kalman.H.transpose() * S.inverse();

    VectorXf preX(3);
    double vr_ = (trace_status.trace_kalman.X(iDistLat) * trace_status.trace_kalman.X(iVrelLat) +
                                    trace_status.trace_kalman.X(iDistLong) * trace_status.trace_kalman.X(iVrelLong)) /
                                   (sqrt(pow(trace_status.trace_kalman.X(iDistLat), 2.0) + pow(trace_status.trace_kalman.X(iDistLong), 2.0)));
    
    preX << trace_status.trace_kalman.X(iDistLat), trace_status.trace_kalman.X(iDistLong), vr_;
    
    trace_status.trace_kalman.X = trace_status.trace_kalman.X + K * (Z - preX);
    trace_status.trace_kalman.P = trace_status.trace_kalman.P - K * S * K.transpose();
}

void RadarTracker::update_physical(float new_len, float new_wid, float new_theta)
{
    trace_status.trace_shape.len += (new_len - trace_status.trace_shape.len) * 0.9;
    trace_status.trace_shape.wid += (new_wid - trace_status.trace_shape.wid) * 0.9;
}

void RadarTracker::manager(bool matchedFlag)
{
    trace_status.trace_manager.age++;

    if(matchedFlag)
    {
        trace_status.trace_manager.match_count++;
        trace_status.trace_manager.unmatched_count = 0;
    }
    else
    {
        if(trace_status.trace_manager.match_count)
        {
            trace_status.trace_manager.match_count--;
        }

        trace_status.trace_manager.unmatched_count++;
    }

    if(trace_status.trace_manager.status == TRK_Detected)
    {
        if(trace_status.trace_manager.match_count > 3)
        {
            trace_status.trace_manager.status = TRK_Confirmed;
        }

        if(trace_status.trace_manager.unmatched_count >= 2)
        {
            trace_status.trace_manager.status = TRK_Delete; // need delete it
        }
    }
    else
    {
        if(trace_status.trace_manager.unmatched_count >= 5)
        {
            trace_status.trace_manager.status = TRK_Delete; // need delete it
        }
    }
}