#pragma once

#include "nuscenes2bag/RadarObjects.h"
#include "nuscenes2bag/PclRadarObject.hpp"
#include "nuscenes2bag/Filesystem.hpp"
#include "sensor_msgs/PointCloud2.h"
#include <pcl_ros/point_cloud.h>

namespace nuscenes2bag {

boost::optional<nuscenes2bag::RadarObjects> readRadarFile(const fs::path& filePath);
boost::optional<sensor_msgs::PointCloud2> readRadarFile2PCLXYZ(const fs::path& filePath);
}