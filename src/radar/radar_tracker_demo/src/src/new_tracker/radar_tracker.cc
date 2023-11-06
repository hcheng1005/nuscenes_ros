#include "radar_tracker.h"

RadarTracker::RadarTracker(int new_id, float center_lat, float center_long, float len, float wid, float theta)
{
    trace_status.trace_manager.age = 1;
    trace_status.trace_manager.id = new_id;
    trace_status.trace_manager.match_count = 1;
    trace_status.trace_manager.unmatched_count = 0;
    trace_status.trace_manager.prob = 0.3;

    trace_status.trace_kalman.X = trace_status.trace_kalman.X.setZero();
    trace_status.trace_kalman.X(iDistLat) = center_lat;
    trace_status.trace_kalman.X(iDistLong) = center_long;
    trace_status.trace_kalman.P = trace_status.trace_kalman.P.setOnes();

    Set_Q(0.2, 0.2);
    Set_R();
    Set_F();
    Set_H();
}

void RadarTracker::trace_predict()
{
    trace_status.trace_kalman.X = trace_status.trace_kalman.F * trace_status.trace_kalman.X;
    trace_status.trace_kalman.P = trace_status.trace_kalman.F * trace_status.trace_kalman.P * trace_status.trace_kalman.P.transpose() + trace_status.trace_kalman.Q;
}

void RadarTracker::trace_update_kinematic(const VectorXf &Z)
{
    MatrixXf S = trace_status.trace_kalman.H * trace_status.trace_kalman.P * trace_status.trace_kalman.H.transpose() + trace_status.trace_kalman.R;
    MatrixXf K = trace_status.trace_kalman.P * trace_status.trace_kalman.H.transpose() * S.inverse();
    
    trace_status.trace_kalman.X = trace_status.trace_kalman.X + K * (Z - trace_status.trace_kalman.H * trace_status.trace_kalman.X);
    trace_status.trace_kalman.P = trace_status.trace_kalman.P - K * S * K.transpose();
}

void RadarTracker::trace_update_physical(float new_len, float new_wid, float new_theta)
{
    trace_status.trace_shape.len += (new_len - trace_status.trace_shape.len) * 0.9;
    trace_status.trace_shape.wid += (new_wid - trace_status.trace_shape.wid) * 0.9;
}