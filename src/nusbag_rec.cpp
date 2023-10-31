#include "nuscenes2bag/RadarDirectoryConverter.hpp"
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

using namespace sensor_msgs;
using namespace nuscenes2bag;

void
LidarCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_ptr)
{
  std::cout << cloud_ptr->header.stamp << ": "
            << "Rec new Msg: [Lidar TOP] " << std::endl;

  // centerpoint.prepare();

  // step 2: run lidar infer
}

void
CameraCallback(const sensor_msgs::Image& img_ptr)
{
  std::cout << img_ptr.header.stamp << ": "
            << "Rec new Msg: [Camera TOP] " << std::endl;
}

void
RadarCallback(const RadarObjects& radar_ptr)
{
  std::cout << radar_ptr.header.stamp << ": "
            << "Rec new Msg: [Radar TOP] " << std::endl;
}

int
main(int argc, char* argv[])
{
  ros::init(argc, argv, "depth_cluster");
  ros::NodeHandle n1, n2, n3;
  ros::Subscriber point_sub = n1.subscribe("/lidar_top", 1, LidarCallback);
  ros::Subscriber camera_sub =
    n2.subscribe("/cam_front/raw", 1, CameraCallback);
  ros::Subscriber radar_sub = n3.subscribe("/radar_front", 1, RadarCallback);
  ros::spin();
  return 0;
}
