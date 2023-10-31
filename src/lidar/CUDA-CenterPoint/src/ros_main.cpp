#include <chrono>
#include <dirent.h>
#include <fstream>
#include <memory>
#include <sstream>
#include <stdio.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

// headers in PCL
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include "./include/centerpoint.h"
#include "./include/common.h"

using namespace sensor_msgs;

std::string Model_File =
  "/data/chenghao/mycode/nuscenes2bag-master/wk_spc/src/src/lidar/"
  "CUDA-CenterPoint/src/model/centerpoint_rpn.plan";
std::string Save_Dir = "../data/prediction/";

CenterPoint centerpoint(Model_File, true);
float* d_points = nullptr;

Params params;
cudaStream_t stream = NULL;

void
centerpoint_init(void)
{
  checkCudaErrors(cudaStreamCreate(&stream));

  centerpoint.prepare();

  checkCudaErrors(cudaMalloc(
    (void**)&d_points, MAX_POINTS_NUM * params.feature_num * sizeof(float)));
}

void
centerpoint_run(void)
{
}

void
centerpoint_die(void)
{
  checkCudaErrors(cudaFree(d_points));
  checkCudaErrors(cudaStreamDestroy(stream));
}

void
lidar_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  std::cout << msg->header.stamp << ": "
            << "Rec new Msg: [Lidar TOP] " << std::endl;

  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pc_ptr(
    new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*msg, *pcl_pc_ptr);

  int length = pcl_pc_ptr->size();
  float pc_data[length * 5];

  for (uint i = 0; i < length; i++) {
    pc_data[5 * i + 0] = pcl_pc_ptr->at(i).x;
    pc_data[5 * i + 1] = pcl_pc_ptr->at(i).y;
    pc_data[5 * i + 2] = pcl_pc_ptr->at(i).z;
    pc_data[5 * i + 3] = pcl_pc_ptr->at(i).intensity;
    pc_data[5 * i + 4] = 0;
  }

  checkCudaErrors(
    cudaMemcpy(d_points, pc_data, length * 5, cudaMemcpyHostToDevice));


  std::cout << "start infer" << std::endl;

  // // 模型推理
  centerpoint.doinfer((void*)d_points, length, stream);
}

int
main(int argc, char* argv[])
{
  ros::init(argc, argv, "nus_lidar");
  ros::NodeHandle n1;
  ros::Subscriber point_sub = n1.subscribe("/lidar_top", 1, lidar_callback);

  centerpoint_init();


  ros::spin();
  return 0;
}
