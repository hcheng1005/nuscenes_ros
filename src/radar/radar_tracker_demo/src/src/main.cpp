#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <vector>

// ROS
#include "../../../../../include/nuscenes2bag/RadarDirectoryConverter.hpp"
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

// Eigen
#include <Eigen/Dense>

#include "../include/radarTracker.h"
#include "../include/type.h"

//
#include "./tracker/track_process.h"

using namespace sensor_msgs;
using namespace nuscenes2bag;

struct VehicleInfo
{
  double speed;
  double yawReate;
};

// 全局变量定义
// 跟踪算法变量
Tracker Tracker_;
RadarOutput_Struct ContiRadarOutput;
std::vector<RadarMeasure_struct> global_point;

std::vector<vehicleInfo_struct> vehi4radar;
trackTable_strcut trackList[MAXTRACKS];         // 运动航迹列表
gridTrack_t gridTrackList[STATIC_TRACK_MAXNUM]; // 静态航迹列表

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @param {RadarObjects&} radar_ptr
 * @return {*}
 */
void
RadarCallback(const RadarObjects& radar_ptr)
{
  std::cout << radar_ptr.header.stamp << ": "
            << "Rec new Msg: [Radar TOP] " << radar_ptr.header.frame_id
            << std::endl;

  std::vector<RadarDemo::radar_point_t> radar_meas;

  uint32_t idx = 0;
  for (const auto& radar_point : radar_ptr.objects) {
    float range_sc =
      sqrt(pow(radar_point.pose.x, 2.0) + pow(radar_point.pose.y, 2.0));
    float azimuth_sc = atan2(radar_point.pose.x, radar_point.pose.y);

    RadarDemo::radar_point_t pc{ range_sc,
                                 azimuth_sc,
                                 radar_point.vx,
                                 radar_point.rcs,
                                 radar_point.vx_comp,
                                 radar_point.pose.x,
                                 radar_point.pose.y,
                                 0.0,
                                 0.0,
                                 true };
    radar_meas.push_back(pc);

    ContiRadarOutput.RadarMeasure[idx].ID = idx;
    ContiRadarOutput.RadarMeasure[idx].DistLong = radar_point.pose.x;
    ContiRadarOutput.RadarMeasure[idx].DistLat = radar_point.pose.y;
    ContiRadarOutput.RadarMeasure[idx].VrelLong = radar_point.vx;
    ContiRadarOutput.RadarMeasure[idx].VrelLat = radar_point.vy;

    ContiRadarOutput.RadarMeasure[idx].RCS = radar_point.rcs;
    ContiRadarOutput.RadarMeasure[idx].AmbigState =
      static_cast<AmbigState_enum>(radar_point.ambig_state);
    ContiRadarOutput.RadarMeasure[idx].AmbigState_Valid = true;
    ContiRadarOutput.RadarMeasure[idx].InvalidState =
      static_cast<uint8_t>(radar_point.invalid_state);

    if (fabs(radar_point.vx_comp) < 0.2) {
      ContiRadarOutput.RadarMeasure[idx].DynProp = stationary; // stationary
    } else {
      ContiRadarOutput.RadarMeasure[idx].DynProp = moving; // moving
    }

    idx++;
  }

  ContiRadarOutput.Header.ActualRecNum = idx;
  ContiRadarOutput.Header.ShouldRecNum = idx;

  // 执行雷达MOT
  // radar_track_main(radar_meas);

  // 目标跟踪算法
  track_process(trackList,
                gridTrackList,
                &ContiRadarOutput,
                global_point,
                vehi4radar,
                &Tracker_,
                0.075);

  trackTable_strcut* trace = NULL;
  for (uint8_t i = 0; i < MAXTRACKS; i++) {
    trace = &trackList[i];

    // 只输出确认航迹
    if (trace->trackState == TRACK_STATE_DETECTION) {

      std::cout << "Unactivate Trace Pos:[" << trace->KalmanInfo.StateEst(0)
                << ", " << trace->KalmanInfo.StateEst(1) << "]" << std::endl;
    }

    if (trace->trackState == TRACK_STATE_ACTIVE) {
      std::cout << "Activate Trace Pos:[" << trace->KalmanInfo.StateEst(0)
                << ", " << trace->KalmanInfo.StateEst(1) << "]" << std::endl;
    }
  }
}

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @param {Odometry&} pose_ptr
 * @return {*}
 */
void
PoseCallback(const nav_msgs::Odometry& pose_ptr)
{
  // std::cout << pose_ptr.header.stamp << ": "
  //           << "Rec new Msg: [Pose] " << std::endl;

  vehicleInfo_struct vehicleInfo;
  vehicleInfo.vx = pose_ptr.twist.twist.linear.x;
  vehicleInfo.yaw_rate = pose_ptr.twist.twist.angular.z;
  vehi4radar.clear();
  vehi4radar.push_back(vehicleInfo);

  // std::cout << vehicle_info.speed << ", " << vehicle_info.yawReate <<
  // std::endl;
}

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @param {int} argc
 * @param {char*} argv
 * @return {*}
 */
int
main(int argc, char* argv[])
{
  ros::init(argc, argv, "depth_cluster");
  ros::NodeHandle radar_back_left_node, radar_back_right_node, radar_front_node,
    radar_front_left_node, radar_front_right_node;

  ros::Subscriber radar_sub1 =
    radar_back_left_node.subscribe("/radar_front", 1, RadarCallback);

  ros::NodeHandle vehicle_node;
  ros::Subscriber pose_sub =
    vehicle_node.subscribe("/vehicle", 1, PoseCallback);
  ros::spin();
  return 0;
}
