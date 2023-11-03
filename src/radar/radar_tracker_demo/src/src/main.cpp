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

using namespace sensor_msgs;
using namespace nuscenes2bag;

struct VehicleInfo
{
  double speed;
  double yawReate;
};

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
  }

  // 执行雷达MOT
  radar_track_main(radar_meas);
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
  std::cout << pose_ptr.header.stamp << ": "
            << "Rec new Msg: [Pose] " << std::endl;

  // 获取车速和YawRate
  VehicleInfo vehicle_info{ pose_ptr.twist.twist.linear.x,
                            pose_ptr.twist.twist.angular.z };

  std::cout << vehicle_info.speed << ", " << vehicle_info.yawReate << std::endl;
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
