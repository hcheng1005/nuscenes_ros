#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <iostream>

#include "nuscenes2bag/RadarDirectoryConverter.hpp"

using namespace sensor_msgs;
using namespace nuscenes2bag;

void LidarCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_ptr) {
  // std::cout << cloud_ptr->header.stamp << ": "
  //           << "Rec new Msg: [Lidar TOP] " << std::endl;

  // centerpoint.prepare();

  // step 2: run lidar infer
}

void CameraCallback(const sensor_msgs::Image& img_ptr) {
  // std::cout << img_ptr.header.stamp << ": "
  //           << "Rec new Msg: [Camera TOP] " << std::endl;
}

void RadarCallback(const RadarObjects& radar_ptr) {
  std::cout << radar_ptr.header.stamp << ": "
            << "Rec new Msg: [Radar TOP] " << radar_ptr.header.frame_id
            << std::endl;
}

struct VehicleInfo {
  double speed;
  double yawReate;
};

void PoseCallback(const nav_msgs::Odometry& pose_ptr) {
  std::cout << pose_ptr.header.stamp << ": "
            << "Rec new Msg: [Pose] " << std::endl;

  // 获取车速和YawRate
  VehicleInfo vehicle_info{pose_ptr.twist.twist.linear.x,
                           pose_ptr.twist.twist.angular.z};

  std::cout << vehicle_info.speed << ", " << vehicle_info.yawReate << std::endl;
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "depth_cluster");
  ros::NodeHandle lidar_top_node, cam_front_node;
  ros::Subscriber point_sub =
      lidar_top_node.subscribe("/lidar_top", 1, LidarCallback);
  ros::Subscriber camera_sub =
      cam_front_node.subscribe("/cam_front/raw", 1, CameraCallback);

  ros::NodeHandle radar_back_left_node, radar_back_right_node, radar_front_node,
      radar_front_left_node, radar_front_right_node;

  ros::Subscriber radar_sub1 =
      radar_back_left_node.subscribe("/radar_back_left", 1, RadarCallback);
  ros::Subscriber radar_sub2 =
      radar_back_right_node.subscribe("/radar_back_right", 1, RadarCallback);
  ros::Subscriber radar_sub3 =
      radar_front_node.subscribe("/radar_front", 1, RadarCallback);
  ros::Subscriber radar_sub4 =
      radar_front_left_node.subscribe("/radar_front_left", 1, RadarCallback);
  ros::Subscriber radar_sub5 =
      radar_front_right_node.subscribe("/radar_front_right", 1, RadarCallback);

  ros::NodeHandle vehicle_node;
  ros::Subscriber pose_sub =
      vehicle_node.subscribe("/vehicle_node", 1, PoseCallback);
  ros::spin();
  return 0;
}
