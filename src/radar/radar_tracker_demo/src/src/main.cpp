#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <vector>

// ROS
#include <ros/ros.h>
#include "../../../../../include/nuscenes2bag/RadarDirectoryConverter.hpp"
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


// Eigen
#include <Eigen/Dense>

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

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


ros::Publisher trace_array_pub_;

/**
 * @names: 
 * @description: Briefly describe the function of your function
 * @return {*}
 */
void visualization_(void)
{
  // FOR VISUALIZATION
  cv::Mat image2;

  image2 = cv::Mat::zeros(600, 800, CV_8UC3); // 宽800，高600，3通道图像

  // 原始点云可视化
  for(int idx=0; idx<ContiRadarOutput.Header.ActualRecNum; idx++)
  {
    cv::circle(image2,
            cv::Point2f((ContiRadarOutput.RadarMeasure[idx].DistLat+ 100) / 200 * 800, 
                        600 - ContiRadarOutput.RadarMeasure[idx].DistLong / 100 * 600),
            3, cv::Scalar(200, 200, 200), -1);
  }

  // 航迹可视化
  for (uint8_t i = 0; i < MAXTRACKS; i++) {
    const auto &trace = trackList[i];
    if (trace.trackState == TRACK_STATE_FREE) {
      continue;
    }

    // 计算旋转矩阵
    double center_x = trace.KalmanInfo.StateEst(iDistLat);
    double center_y = trace.KalmanInfo.StateEst(iDistLong);
    double width = trace.ExtendInfo.Width, height = trace.ExtendInfo.Length;
    double angle_degrees = trace.ExtendInfo.box_theta  / CV_PI * 180.0;

    cv::RotatedRect rect(cv::Point2f((trace.KalmanInfo.StateEst(iDistLat) - trace.ExtendInfo.Width * 0.5 + 100) / 200 * 800,
                          600 - (trace.KalmanInfo.StateEst(iDistLong) + trace.ExtendInfo.Length * 0.5) / 100 * 600),
                          cv::Size2f(trace.ExtendInfo.Width / 200 * 800, trace.ExtendInfo.Length / 100 * 600), 
                          trace.ExtendInfo.box_theta  / CV_PI * 180.0);
    
    int thickness = 2;
    cv::Point2f vertices[4];
    rect.points(vertices);

    if(trace.trackState == TRACK_STATE_ACTIVE)
    {
      for (int i = 0; i < 4; i++) {
          cv::line(image2, vertices[i], vertices[(i + 1) % 4], cv::Scalar(255, 0, 0), thickness);
      }
    }
    else{
      for (int i = 0; i < 4; i++) {
          cv::line(image2, vertices[i], vertices[(i + 1) % 4], cv::Scalar(125, 125, 125), thickness);
      }
    }
  }

  cv::imshow("Radar Multi-Target Tracking", image2);
  cv::waitKey(50);
}

struct Point3D {
    double x;
    double y;
    double z;
};

void calculateBoxCorners(Point3D center, double length, double width, double height, double yaw, std::vector<Point3D> &corners) {
    // Calculate half dimensions
    double halfLength = length / 2;
    double halfWidth = width / 2;
    double halfHeight = height / 2;

    // Calculate the rotation matrix
    double cosYaw = cos(yaw);
    double sinYaw = sin(yaw);

    // Calculate each corner point
    for (int i = 0; i < 8; ++i) {
        double x1, y1;
        if (i & 1) {
            x1 = halfLength;
        } else {
            x1 = -halfLength;
        }
        if (i & 2) {
            y1 = halfWidth;
        } else {
            y1 = -halfWidth;
        }
        if (i & 4) {
            corners[i].z = halfHeight;
        } else {
            corners[i].z = -halfHeight;
        }

        // Apply the rotation matrix
        corners[i].x = center.x + x1 * cosYaw - y1 * sinYaw;
        corners[i].y = center.y + x1 * sinYaw + y1 * cosYaw;
    }

    // Print the corner points
    for (int i = 0; i < 8; ++i) {
        std::cout << "Corner " << i + 1 << ": (" << corners[i].x << ", " << corners[i].y << ", " << corners[i].z << ")\n";
    }


    //     double halfLength = length / 2;
    // double halfWidth = width / 2;
    // double halfHeight = height / 2;

    // // Define the transformation matrix
    // Matrix3d R;
    // R << cos(yaw), -sin(yaw), 0,
    //      sin(yaw), cos(yaw), 0,
    //      0, 0, 1;

    // // Define the local corner points
    // Matrix<double, 3, 8> localCorners;
    // localCorners << halfLength, halfLength, halfLength, halfLength, -halfLength, -halfLength, -halfLength, -halfLength,
    //                 halfWidth, halfWidth, -halfWidth, -halfWidth, halfWidth, halfWidth, -halfWidth, -halfWidth,
    //                 halfHeight, -halfHeight, halfHeight, -halfHeight, halfHeight, -halfHeight, halfHeight, -halfHeight;

    // // Transform local corners to global coordinates
    // Matrix<double, 3, 8> globalCorners = R * localCorners;
}

void pub_trace_box(void)
{
  visualization_msgs::MarkerArray marker_array;

  // for (uint8_t i = 0; i < MAXTRACKS; i++) {
  //   const auto &trace = trackList[i];
  //   if (trace.trackState == TRACK_STATE_FREE) {
  //     continue;
  //   }
    
    // // 创建一个visualization_msgs/Marker消息
    // visualization_msgs::Marker marker;
    // marker.header.frame_id = "lidar_top";
    // marker.header.stamp = ros::Time::now();

    // marker.id = trace.trackID;
    // marker.type = visualization_msgs::Marker::LINE_STRIP;
    // marker.action = visualization_msgs::Marker::ADD;
    // marker.pose.orientation.w = 1.0;
    // marker.scale.x = 0.1;  // 线条的宽度
    // marker.color.a = 1.0;
    // marker.color.r = 1.0;  // 设置线条的颜色为红色

    // // 创建一系列点来定义线条的路径
    // std::vector<geometry_msgs::Point> points;

    // Point3D center{trace.KalmanInfo.StateEst(iDistLong), trace.KalmanInfo.StateEst(iDistLat), 1.0};
    // std::vector<Point3D> corners;
    // corners.resize(8);
    // calculateBoxCorners(center, trace.ExtendInfo.Length, trace.ExtendInfo.Width, 1.8, trace.ExtendInfo.box_theta, corners);
    
    // uint cor_seq[10] = {0,1,2,3,0,4,5,6,7,4};
    // for(uint corner_idx=0; corner_idx<9; corner_idx++)
    // {
    //   geometry_msgs::Point p;
    //   p.x = corners[cor_seq[corner_idx]].x;
    //   p.y = corners[cor_seq[corner_idx]].y;
    //   p.z = corners[cor_seq[corner_idx]].z;
    //   points.push_back(p);
    // }

    // // 将点添加到marker中
    // marker.points = points;
    // marker.lifetime = ros::Duration(10);
    // marker_array.markers.push_back(marker);
  // }

  // 创建一个visualization_msgs/Marker消息
    visualization_msgs::Marker marker;
    marker.header.frame_id = "lidar_top";
    marker.header.stamp = ros::Time::now();

    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;  // 线条的宽度
    marker.color.a = 1.0;
    marker.color.r = 1.0;  // 设置线条的颜色为红色

    // 创建一系列点来定义线条的路径
    std::vector<geometry_msgs::Point> points;

    Point3D center{ 0, 10,1.0};
    std::vector<Point3D> corners;
    corners.resize(8);
    calculateBoxCorners(center, 2, 4, 1.8, 0.0, corners);
    
    uint cor_seq[12][2] = {{0, 1}, {1, 2}, {2, 3}, {3, 0}, {4, 5}, {5, 6},
         {6, 7}, {7, 4}, {0, 4}, {1, 5}, {2, 6}, {3, 7}};

    for(uint corner_idx=0; corner_idx<12; corner_idx++)
    {
      geometry_msgs::Point p;
      p.x = corners[cor_seq[corner_idx][0]].x;
      p.y = corners[cor_seq[corner_idx][0]].y;
      p.z = corners[cor_seq[corner_idx][0]].z;

      points.push_back(p);
      p.x = corners[cor_seq[corner_idx][1]].x;
      p.y = corners[cor_seq[corner_idx][1]].y;
      p.z = corners[cor_seq[corner_idx][1]].z;

      points.push_back(p);
    }

    // 将点添加到marker中
    marker.points = points;
    marker.lifetime = ros::Duration(10);
    marker_array.markers.push_back(marker);

  trace_array_pub_.publish(marker_array);

}

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

    if (fabs(radar_point.vx_comp) < 0.5) {
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

  // visualization_();

  pub_trace_box();
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
  vehicleInfo_struct vehicleInfo;
  vehicleInfo.vx = pose_ptr.twist.twist.linear.x;
  vehicleInfo.yaw_rate = pose_ptr.twist.twist.angular.z;
  vehi4radar.clear();
  vehi4radar.push_back(vehicleInfo);
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
  ros::init(argc, argv, "nuscenes");
  ros::NodeHandle radar_back_left_node, radar_back_right_node, radar_front_node,
    radar_front_left_node, radar_front_right_node;

  ros::Subscriber radar_sub1 =
    radar_back_left_node.subscribe("/radar_front", 1, RadarCallback);

  ros::NodeHandle vehicle_node;
  ros::Subscriber pose_sub =
    vehicle_node.subscribe("/vehicle", 1, PoseCallback);

  ros::NodeHandle trace_node;

  // 检测结果可视化
  trace_array_pub_ = trace_node.advertise<visualization_msgs::MarkerArray>("trace", 100);

  ros::spin();
  return 0;
}
