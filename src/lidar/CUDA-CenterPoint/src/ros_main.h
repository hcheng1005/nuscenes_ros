#pragma once

#include <chrono>
#include <dirent.h>
#include <fstream>
#include <memory>
#include <sstream>
#include <stdio.h>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "sensor_msgs/Imu.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


// headers in PCL
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include "./include/centerpoint.h"
#include "./include/common.h"

#include "./include/tracker/lidar_tracker.h"



void pub_det_boxes(std::vector<Bndbox> boxes);
void pub_trace_boxes(std::vector<simple_tracker> track_list);
