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

  Eigen::Matrix<float, 6, 1> X;
  X = X.setZero();
  X(iDistLat) = center_lat;
  X(iDistLong) = center_long;
  X(iVrelLat) = vr * sin(atan2(center_lat, center_long));
  X(iVrelLong) = vr * cos(atan2(center_lat, center_long));

  Eigen::Matrix<float, 6, 6> P;
  P = P.setOnes();

  trace_shape.len = len;
  trace_shape.wid = wid;
  trace_shape.theta = theta;

  randomMatriceFilter = new RandomMatriceFilter<float, 6, 3>();
  randomMatriceFilter->X = X;
  randomMatriceFilter->P = P;
}

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @return {*}
 */
void RadarTracker::trace_predict() { randomMatriceFilter->kalmanPredict(); }

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @param {VectorXf&} Z
 * @return {*}
 */
void RadarTracker::trace_update_kinematic(const VectorXf &Z) {
  std::cout << "Trace ID:[ " << trace_manager.id << " ]" << std::endl;
  randomMatriceFilter->kalmanUpdate(Z.transpose());
}

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @return {*}
 */
void RadarTracker::trace_update_physical(float new_len, float new_wid,
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