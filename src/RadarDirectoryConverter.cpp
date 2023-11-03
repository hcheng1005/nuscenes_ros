#include "nuscenes2bag/RadarDirectoryConverter.hpp"


using namespace sensor_msgs;
using namespace std;
using namespace nuscenes2bag;

namespace nuscenes2bag {

boost::optional<RadarObjects>
readRadarFile(const fs::path& filePath)
{
  const auto fileName = filePath.string();
  pcl::PointCloud<PclRadarObject>::Ptr cloud(
    new pcl::PointCloud<PclRadarObject>);

  if (pcl::io::loadPCDFile<PclRadarObject>(fileName, *cloud) ==
      -1) //* load the file
  {
    std::string error = "Could not read ";
    error += fileName;
    cout << error << endl;
    // PCL_ERROR(error);

    return boost::none;
  }

  RadarObjects radarObjects;

  for (const auto& pclRadarObject : *cloud) {
    RadarObject obj;
    obj.pose.x = pclRadarObject.x;
    obj.pose.y = pclRadarObject.y;
    obj.pose.z = pclRadarObject.z;
    obj.dyn_prop = pclRadarObject.dyn_prop;
    obj.rcs = pclRadarObject.rcs;
    obj.vx = pclRadarObject.vx;
    obj.vy = pclRadarObject.vy;
    obj.vx_comp = pclRadarObject.vx_comp;
    obj.vy_comp = pclRadarObject.vy_comp;
    obj.is_quality_valid = pclRadarObject.is_quality_valid;
    obj.ambig_state = pclRadarObject.ambig_state;
    obj.x_rms = pclRadarObject.x_rms;
    obj.y_rms = pclRadarObject.y_rms;
    obj.invalid_state = pclRadarObject.invalid_state;
    obj.pdh0 = pclRadarObject.pdh0;
    obj.vx_rms = pclRadarObject.vx_rms;
    obj.vy_rms = pclRadarObject.vy_rms;
    radarObjects.objects.push_back(obj);
  }

  return boost::optional<RadarObjects>(radarObjects);
}

inline void
fillFieldsForPointcloudXYZIR(std::vector<PointField>& fields)
{
  PointField field;
  field.datatype = sensor_msgs::PointField::FLOAT32;
  field.offset = 0;
  field.count = 1;
  field.name = std::string("x");
  fields.push_back(field);

  field.datatype = sensor_msgs::PointField::FLOAT32;
  field.offset = 4;
  field.count = 1;
  field.name = std::string("y");
  fields.push_back(field);

  field.datatype = sensor_msgs::PointField::FLOAT32;
  field.offset = 8;
  field.count = 1;
  field.name = std::string("z");
  fields.push_back(field);

  // This field only contains positive integers but it's encoded as a float (4
  // bytes)
  field.datatype = sensor_msgs::PointField::FLOAT32;
  field.offset = 12;
  field.count = 1;
  field.name = std::string("intensity");
  fields.push_back(field);

  // This field only contains positive integers but it's encoded as a float (4
  // bytes)
  field.datatype = sensor_msgs::PointField::FLOAT32;
  field.offset = 16;
  field.count = 1;
  field.name = std::string("ring");
  fields.push_back(field);
}


// Convert float32 to 4 bytes
union
{
  float value;
  uint8_t byte[4];
} floatToBytes;

inline void
push_back_float32_XYZIR(std::vector<uint8_t>& data, float float_data)
{
  floatToBytes.value = float_data;
  data.push_back(floatToBytes.byte[0]);
  data.push_back(floatToBytes.byte[1]);
  data.push_back(floatToBytes.byte[2]);
  data.push_back(floatToBytes.byte[3]);
}

boost::optional<sensor_msgs::PointCloud2>
readRadarFile2PCLXYZ(const fs::path& filePath)
{
  const auto fileName = filePath.string();
  pcl::PointCloud<PclRadarObject>::Ptr cloud(
    new pcl::PointCloud<PclRadarObject>);

  if (pcl::io::loadPCDFile<PclRadarObject>(fileName, *cloud) ==
      -1) //* load the file
  {
    std::string error = "Could not read ";
    error += fileName;
    cout << error << endl;
    // PCL_ERROR(error);

    return boost::none;
  }

  PointCloud2 cloud_msg;
  cloud_msg.header.frame_id = std::string("radar");
  cloud_msg.is_bigendian = false;
  cloud_msg.point_step = sizeof(float) * 5; // Length of each point in bytes
  cloud_msg.height = 1;
  cloud_msg.is_dense = true;

  cloud_msg.width = (*cloud).size();

  std::vector<uint8_t> data;
  for (const auto& pclRadarObject : *cloud) {
    // RadarObject obj;
    // obj.pose.x = pclRadarObject.x;
    // obj.pose.y = pclRadarObject.y;
    // obj.pose.z = pclRadarObject.z;
    // obj.dyn_prop = pclRadarObject.dyn_prop;
    // obj.rcs = pclRadarObject.rcs;
    // obj.vx = pclRadarObject.vx;
    // obj.vy = pclRadarObject.vy;
    // obj.vx_comp = pclRadarObject.vx_comp;
    // obj.vy_comp = pclRadarObject.vy_comp;
    // obj.is_quality_valid = pclRadarObject.is_quality_valid;
    // obj.ambig_state = pclRadarObject.ambig_state;
    // obj.x_rms = pclRadarObject.x_rms;
    // obj.y_rms = pclRadarObject.y_rms;
    // obj.invalid_state = pclRadarObject.invalid_state;
    // obj.pdh0 = pclRadarObject.pdh0;
    // obj.vx_rms = pclRadarObject.vx_rms;
    // obj.vy_rms = pclRadarObject.vy_rms;
    // radarObjects.objects.push_back(obj);
    push_back_float32_XYZIR(data, pclRadarObject.x);
    push_back_float32_XYZIR(data, pclRadarObject.y);
    push_back_float32_XYZIR(data, pclRadarObject.z);
    push_back_float32_XYZIR(data, 0.0);
    push_back_float32_XYZIR(data, 0.0);
  }

  fillFieldsForPointcloudXYZIR(cloud_msg.fields);
  cloud_msg.data = data;
  cloud_msg.row_step = data.size(); // Length of row in bytes

  // return boost::optional<RadarObjects>(radarObjects);
  return boost::optional<sensor_msgs::PointCloud2>(cloud_msg); 
}

}