#include <iostream>

#include "radar_tracker.h"

RadarTracker::RadarTracker(uint new_id, float center_lat, float center_long,
                           float vr, float len, float wid, float theta) {
  trace_manager.age = 1;
  trace_manager.id = new_id;
  trace_manager.status = TRK_Detected;
  trace_manager.match_count = 1;
  trace_manager.unmatched_count = 0;
  trace_manager.prob = 0.3;

  trace_kalman.X = trace_kalman.X.setZero();
  trace_kalman.X(iDistLat) = center_lat;
  trace_kalman.X(iDistLong) = center_long;
  trace_kalman.X(iVrelLat) = vr * sin(atan2(center_lat, center_long));
  trace_kalman.X(iVrelLong) = vr * cos(atan2(center_lat, center_long));
  trace_kalman.P = trace_kalman.P.setOnes();

  trace_shape.len = len;
  trace_shape.wid = wid;
  trace_shape.theta = theta;

  Set_Q(0.2, 0.2);
  Set_R();
  Set_F();
  Set_H();

  basicKalman = new basicKalmanFilter<float>(6, 2);
}

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @return {*}
 */
void RadarTracker::trace_predict() {
  trace_kalman.X = trace_kalman.F * trace_kalman.X;
  trace_kalman.P =
      trace_kalman.F * trace_kalman.P * trace_kalman.P.transpose() +
      trace_kalman.Q;

  computeJacMat();

  updetaZPre();
}

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @param {VectorXf&} Z
 * @return {*}
 */
void RadarTracker::update_kinematic(const VectorXf& Z) {
  MatrixXf S = trace_kalman.H * trace_kalman.P * trace_kalman.H.transpose() +
               trace_kalman.R;
  MatrixXf K = trace_kalman.P * trace_kalman.H.transpose() * S.inverse();

  VectorXf preX(3);
  double vr_ = (trace_kalman.X(iDistLat) * trace_kalman.X(iVrelLat) +
                trace_kalman.X(iDistLong) * trace_kalman.X(iVrelLong)) /
               (sqrt(pow(trace_kalman.X(iDistLat), 2.0) +
                     pow(trace_kalman.X(iDistLong), 2.0)));

  preX << trace_kalman.X(iDistLat), trace_kalman.X(iDistLong), vr_;

  std::cout << "Z: " << Z.transpose() << std::endl;
  std::cout << "X: " << preX.transpose() << std::endl;
  std::cout << "X_old: " << trace_kalman.X.transpose() << std::endl;

  // std::cout << "H: \n" << trace_kalman.H << std::endl;
  // std::cout << "P: \n" << trace_kalman.P << std::endl;
  // std::cout << "S: \n" << S << std::endl;
  // std::cout << "K: \n" << K << std::endl;

  trace_kalman.X = trace_kalman.X + K * (Z - preX);
  trace_kalman.P = trace_kalman.P - K * trace_kalman.H * trace_kalman.P;

  std::cout << "K * (Z - preX): " << (K * (Z - preX)).transpose() << std::endl;
  std::cout << "X_new: " << trace_kalman.X.transpose() << std::endl;
}

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @return {*}
 */
void RadarTracker::update_physical(float new_len, float new_wid,
                                   float new_theta) {
  if (new_len > trace_shape.len) {
    trace_shape.len += (new_len - trace_shape.len) * 0.9;
  } else {
    trace_shape.len += (new_len - trace_shape.len) * 0.5;
  }

  if (new_wid > trace_shape.wid) {
    trace_shape.wid += (new_wid - trace_shape.wid) * 0.9;
  } else {
    trace_shape.wid += (new_wid - trace_shape.wid) * 0.5;
  }
}

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @param {bool} matchedFlag
 * @return {*}
 */
void RadarTracker::manager(bool matchedFlag) {
  trace_manager.age++;

  if (matchedFlag) {
    trace_manager.match_count++;
    trace_manager.unmatched_count = 0;
  } else {
    if (trace_manager.match_count) {
      trace_manager.match_count--;
    }

    trace_manager.unmatched_count++;
  }

  // std::cout << "Trace ID: [" << trace_manager.id << "]"
  //           << trace_manager.age << std::endl;

  // std::cout << "match_count: [" << trace_manager.match_count <<
  // "]"
  //           << ", "
  //           << "unmatched_count: [" <<
  //           trace_manager.unmatched_count << "]"
  //           << ", "
  //           << std::endl;

  if (trace_manager.status == TRK_Detected) {
    if (trace_manager.match_count > 3) {
      trace_manager.status = TRK_Confirmed;
    }

    if (trace_manager.unmatched_count >= 2) {
      trace_manager.status = TRK_Delete;  // need delete it
    }
  } else {
    if (trace_manager.unmatched_count >= 5) {
      trace_manager.status = TRK_Delete;  // need delete it
    }
  }
}