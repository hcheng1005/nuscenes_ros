
#include "ros_main.h"

using namespace sensor_msgs;

ros::Publisher det_array_pub_;
ros::Publisher trace_array_pub_;


std::string Model_File =
"/data/chenghao/mycode/nuscenes2bag-master/wk_spc/src/src/lidar/"
"CUDA-CenterPoint/src/model/centerpoint_rpn.plan";
std::string Save_Dir = "../data/prediction/";


// 检测器
CenterPoint* centerpoint;
float* d_points = nullptr;

// 跟踪器
lidar_tracker tracker;

Params params;
cudaStream_t stream = NULL;

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @return {*}
 */
void centerpoint_init(void)
{
  cudaDeviceProp prop;

  int count = 0;
  cudaGetDeviceCount(&count);
  printf("\nGPU has cuda devices: %d\n", count);
  for (int i = 0; i < count; ++i) {
    cudaGetDeviceProperties(&prop, i);
    printf("----device id: %d info----\n", i);
    printf("  GPU : %s \n", prop.name);
    printf("  Capbility: %d.%d\n", prop.major, prop.minor);
    printf("  Global memory: %luMB\n", prop.totalGlobalMem >> 20);
    printf("  Const memory: %luKB\n", prop.totalConstMem >> 10);
    printf("  SM in a block: %luKB\n", prop.sharedMemPerBlock >> 10);
    printf("  warp size: %d\n", prop.warpSize);
    printf("  threads in a block: %d\n", prop.maxThreadsPerBlock);
    printf("  block dim: (%d,%d,%d)\n",
      prop.maxThreadsDim[0],
      prop.maxThreadsDim[1],
      prop.maxThreadsDim[2]);
    printf("  grid dim: (%d,%d,%d)\n",
      prop.maxGridSize[0],
      prop.maxGridSize[1],
      prop.maxGridSize[2]);
  }
  printf("\n");

  checkCudaErrors(cudaStreamCreate(&stream));

  centerpoint = new CenterPoint(Model_File, true);

  centerpoint->prepare();

  checkCudaErrors(cudaMalloc(
    (void**)&d_points, MAX_POINTS_NUM * params.feature_num * sizeof(float)));
}

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @return {*}
 */
void centerpoint_die(void)
{
  checkCudaErrors(cudaFree(d_points));
  checkCudaErrors(cudaStreamDestroy(stream));
}

/**
 * @names:
 * @description: Briefly describe the function of your function
 * @return {*}
 */
void pub_det_boxes(std::vector<Bndbox> boxes)
{
  std::cout << "boxes number: " << boxes.size() << std::endl;

  visualization_msgs::MarkerArray marker_array;
  tf2::Quaternion myQuaternion;

  uint32_t shape = visualization_msgs::Marker::CUBE;
  uint32_t id = 0;
  for (auto box : boxes)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "lidar_top";
    marker.header.stamp = ros::Time::now();

    marker.id = id;
    marker.type = shape;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = box.x;
    marker.pose.position.y = box.y;
    marker.pose.position.z = box.z;

    myQuaternion.setRPY(0, 0, box.rt);
    marker.pose.orientation = tf2::toMsg(myQuaternion);

    marker.scale.x = box.w;
    marker.scale.y = box.l;
    marker.scale.z = box.h;

    marker.color.r = 0.0;
    marker.color.g = 0.5;
    marker.color.b = 0.0;

    marker.color.a = 0.6;

    marker.lifetime = ros::Duration(1.5);
    marker_array.markers.push_back(marker);
    id++;
  }

  det_array_pub_.publish(marker_array);
}


/**
 * @names:
 * @description: Briefly describe the function of your function
 * @return {*}
 */
void pub_trace_boxes(std::vector<simple_tracker> track_list)
{
  std::cout << "trace number: " << track_list.size() << std::endl;

  visualization_msgs::MarkerArray marker_array;
  tf2::Quaternion myQuaternion;

  uint32_t shape = visualization_msgs::Marker::CUBE;
  uint32_t id = 0;
  for (auto trace : track_list)
  {
    if (trace.track_manage.track_status != TRK_Confirmed)
    {
      continue;
    }
    visualization_msgs::Marker marker;
    marker.header.frame_id = "lidar_top";
    marker.header.stamp = ros::Time::now();

    marker.id = trace.track_manage.id;
    marker.type = shape;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = trace.X_(iDisLong);
    marker.pose.position.y = trace.X_(iDisLat);
    marker.pose.position.z = trace.X_(iDisHeight);

    myQuaternion.setRPY(0, 0, trace.X_(iBoxHeading));
    marker.pose.orientation = tf2::toMsg(myQuaternion);

    marker.scale.x = trace.X_(iBoxWid);
    marker.scale.y = trace.X_(iBoxLen);
    marker.scale.z = trace.X_(iBoxHeight);

    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;

    marker.color.a = 0.6;

    marker.lifetime = ros::Duration(1.5);
    marker_array.markers.push_back(marker);
    id++;
  }

  trace_array_pub_.publish(marker_array);
}


/**
 * @names:
 * @description: Briefly describe the function of your function
 * @param {PointCloud2ConstPtr&} msg
 * @return {*}
 */
void lidar_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  std::cout << msg->header.stamp << ": "
    << " ---------- Rec new Msg: [Lidar TOP] ---------- " << std::endl;

  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pc_ptr(
    new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*msg, *pcl_pc_ptr);

  int length = pcl_pc_ptr->size();
  float pc_data[length * 5];

  std::cout << "Points number:[ " << length << "]" << std::endl;

  for (uint i = 0; i < length; i++) {
    pc_data[5 * i + 0] = pcl_pc_ptr->at(i).x;
    pc_data[5 * i + 1] = pcl_pc_ptr->at(i).y;
    pc_data[5 * i + 2] = pcl_pc_ptr->at(i).z;
    pc_data[5 * i + 3] = pcl_pc_ptr->at(i).intensity;
    pc_data[5 * i + 4] = 0;
  }

  checkCudaErrors(
    cudaMemcpy(d_points, pc_data, length * 5, cudaMemcpyHostToDevice));

  std::cout << "start infer " << std::endl;

  // // 模型推理
  centerpoint->doinfer((void*)d_points, length, stream);

  // tracking
  tracker.tracking_proc(centerpoint->nms_pred_);

  // 获取bounding box
  pub_det_boxes(centerpoint->nms_pred_);
  pub_trace_boxes(tracker.track_list);
}



/**
 * @names:
 * @description: Briefly describe the function of your function
 * @param {int} argc
 * @param {char*} argv
 * @return {*}
 */
int main(int argc, char* argv[])
{
  ros::init(argc, argv, "nus_lidar");
  ros::NodeHandle n1, det_node, trace_node;
  ros::Subscriber point_sub = n1.subscribe("/lidar_top", 1, lidar_callback);

  // 检测结果可视化
  det_array_pub_ = det_node.advertise<visualization_msgs::MarkerArray>("det", 100);
  trace_array_pub_ = trace_node.advertise<visualization_msgs::MarkerArray>("trace", 100);

  centerpoint_init();

  ros::spin();
  return 0;
}
