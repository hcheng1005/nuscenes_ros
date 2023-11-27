#include "nuscenes2bag/SceneConverter.hpp"

#include <array>
#include <iostream>
#include <regex>
#include <string>

#include "nuscenes2bag/DatasetTypes.hpp"
#include "nuscenes2bag/EgoPoseConverter.hpp"
#include "nuscenes2bag/ImageDirectoryConverter.hpp"
#include "nuscenes2bag/ImuDirectoryConverter.hpp"
#include "nuscenes2bag/LidarDirectoryConverter.hpp"
#include "nuscenes2bag/LidarDirectoryConverterXYZIR.hpp"
#include "nuscenes2bag/RadarDirectoryConverter.hpp"
#include "nuscenes2bag/VehicleDirectoryConverter.hpp"
#include "nuscenes2bag/utils.hpp"

using namespace std;

namespace nuscenes2bag {

SceneConverter::SceneConverter(const MetaDataProvider& metaDataProvider)
    : metaDataProvider(metaDataProvider) {}

boost::optional<SampleType> getSampleType(const std::string& filename) {
  std::array<std::pair<const char*, SampleType>, 3> pairs = {
      {{"CAM", SampleType::CAMERA},
       {"RADAR", SampleType::RADAR},
       {"LIDAR", SampleType::LIDAR}}};
  for (const auto& strAndSampleType : pairs) {
    const auto& str = strAndSampleType.first;
    const auto& sampleType = strAndSampleType.second;
    if (filename.find(str) != string::npos) {
      return boost::optional<SampleType>(sampleType);
    }
  }
  cout << "Unknown file " << filename << endl;
  return boost::none;
}

template <typename T>
void writeMsg(const std::string topicName, const std::string& frameID,
              const TimeStamp timeStamp, rosbag::Bag& outBag,
              boost::optional<T> msgOpt) {
  if (msgOpt) {
    auto& msg = msgOpt.value();
    msg.header.frame_id = frameID;
    msg.header.stamp = stampUs2RosTime(timeStamp);
    outBag.write(std::string(topicName).c_str(), msg.header.stamp, msg);
  }
}

static const std::regex TOPIC_REGEX = std::regex(".*__([A-Z_]+)__.*");

void SceneConverter::submit(const Token& sceneToken,
                            FileProgress& fileProgress) {
  boost::optional<SceneInfo> sceneInfoOpt =
      metaDataProvider.getSceneInfo(sceneToken);
  // if(!sceneInfoOpt) {
  //     // cout << "SceneInfo for " << sceneToken << " not found!" << endl;
  //     return;
  // }
  assert(sceneInfoOpt);
  SceneInfo& sceneInfo = sceneInfoOpt.value();

  sceneId = sceneInfo.sceneId;
  this->sceneToken = sceneToken;
  sampleDatas = metaDataProvider.getSceneSampleData(sceneToken);
  egoPoseInfos = metaDataProvider.getEgoPoseInfo(sceneToken);

  fileProgress.addToProcess(sampleDatas.size());
}

void SceneConverter::run(const fs::path& inPath,
                         const fs::path& outDirectoryPath,
                         FileProgress& fileProgress) {
  std::string bagName =
      outDirectoryPath.string() + "/" + std::to_string(sceneId) + ".bag";

  rosbag::Bag outBag;
  outBag.open(bagName, rosbag::bagmode::Write);

  auto sensorInfos = metaDataProvider.getSceneCalibratedSensorInfo(sceneToken);
  auto sceneInfoOpt = metaDataProvider.getSceneInfo(sceneToken);
  std::cout << "Processing IMU data" << std::endl;
  if (sceneInfoOpt) {
    std::cout << "Got scene exist" << std::endl;
    auto sceneName = sceneInfoOpt.value().name;
    fs::path imuPath = inPath / "can_bus" / (sceneName + "_ms_imu.json");
    std::cout << "Got imu json path:" << imuPath << std::endl;
    fs::path vehiclePath = inPath / "can_bus" / (sceneName + "_pose.json");
    std::cout << "Got vehicle json path:" << vehiclePath << std::endl;
    convertImuDatas(outBag, imuPath);
    convertVehicleDatas(outBag, vehiclePath);
  }
  std::cout << "End processing IMU data" << std::endl;
  convertEgoPoseInfos(outBag, sensorInfos);
  convertSampleDatas(outBag, inPath, fileProgress);

  outBag.close();
}

void SceneConverter::convertImuDatas(rosbag::Bag& outBag,
                                     const fs::path& inPath) {
  std::cout << "Retrieving IMU data" << std::endl;
  auto imuDatas = metaDataProvider.getImuData(inPath);
  std::cout << "End retrieving IMU data" << std::endl;
  auto topicName = "/imu";
  std::cout << "Writing IMU msgs" << std::endl;
  for (const auto& imuData : imuDatas) {
    auto msg = readImuFile(imuData);
    writeMsg(topicName, "imu", imuData.utime, outBag, msg);
  }
}

void SceneConverter::convertVehicleDatas(rosbag::Bag& outBag,
                                         const fs::path& inPath) {
  std::cout << "Retrieving vehicle data" << std::endl;
  auto vehicleDatas = metaDataProvider.getVehicleData(inPath);
  std::cout << "End retrieving vehicle data" << std::endl;
  auto topicName = "/vehicle";
  std::cout << "Writing vehicle msgs" << std::endl;
  for (const auto& vehicleData : vehicleDatas) {
    auto msg = readVehicleFile(vehicleData);
    writeMsg(topicName, "vehicle", vehicleData.utime, outBag, msg);
  }
}

void SceneConverter::convertSampleDatas(rosbag::Bag& outBag,
                                        const fs::path& inPath,
                                        FileProgress& fileProgress) {
  for (const auto& sampleData : sampleDatas) {
    fs::path sampleFilePath = inPath / sampleData.fileName;

    boost::optional<SampleType> sampleTypeOpt =
        getSampleType(sampleFilePath.string());
    if (!sampleTypeOpt) {
      continue;
    }
    SampleType& sampleType = sampleTypeOpt.value();

    CalibratedSensorInfo calibratedSensorInfo =
        metaDataProvider.getCalibratedSensorInfo(
            sampleData.calibratedSensorToken);
    CalibratedSensorName calibratedSensorName =
        metaDataProvider.getSensorName(calibratedSensorInfo.sensorToken);
    std::string sensorName = toLower(calibratedSensorName.name);

    if (sampleType == SampleType::CAMERA) {
      auto topicName = sensorName + "/raw";
      auto msg = readImageFile(sampleFilePath);
      writeMsg(topicName, sensorName, sampleData.timeStamp, outBag, msg);

    } else if (sampleType == SampleType::LIDAR) {
      auto topicName = sensorName;

      // PointCloud format:
      // auto msg = readLidarFile(sampleFilePath); // x,y,z,intensity
      auto msg = readLidarFileXYZIR(sampleFilePath);  // x,y,z,intensity,ring

      writeMsg(topicName, sensorName, sampleData.timeStamp, outBag, msg);

    } else if (sampleType == SampleType::RADAR) {
      auto topicName = sensorName;

      // 转录毫米波雷达原始信息
      auto msg1 = readRadarFile(sampleFilePath);
      writeMsg(topicName, sensorName, sampleData.timeStamp, outBag, msg1);

      // 转录成PCL点云信息便于可视化
      auto msg2 = readRadarFile2PCLXYZ(sampleFilePath);
      writeMsg((topicName + "_pc"), sensorName, sampleData.timeStamp, outBag,
               msg2);

    } else {
      cout << "Unknown sample type" << endl;
    }

    fileProgress.addToProcessed(1);
  }
}

geometry_msgs::TransformStamped makeTransform(const char* frame_id,
                                              const char* child_frame_id,
                                              const double* translation,
                                              const double* rotation,
                                              ros::Time stamp = ros::Time(0)) {
  geometry_msgs::TransformStamped msg;
  msg.header.frame_id = std::string(frame_id);
  msg.header.stamp = stamp;
  msg.child_frame_id = std::string(child_frame_id);
  assignArray2Vector3(msg.transform.translation, translation);
  assignArray2Quaternion(msg.transform.rotation, rotation);
  return msg;
}

geometry_msgs::TransformStamped makeIdentityTransform(
    const char* frame_id, const char* child_frame_id,
    ros::Time stamp = ros::Time(0)) {
  geometry_msgs::TransformStamped msg;
  msg.header.frame_id = std::string(frame_id);
  msg.header.stamp = stamp;
  msg.child_frame_id = std::string(child_frame_id);
  msg.transform.rotation.w = 1;
  return msg;
}

void SceneConverter::convertEgoPoseInfos(
    rosbag::Bag& outBag,
    const std::vector<CalibratedSensorInfoAndName>& calibratedSensorInfos) {
  std::vector<geometry_msgs::TransformStamped> constantTransforms;
  for (const auto& calibratedSensorInfo : calibratedSensorInfos) {
    auto sensorTransform = makeTransform(
        "base_link", toLower(calibratedSensorInfo.name.name).c_str(),
        calibratedSensorInfo.info.translation,
        calibratedSensorInfo.info.rotation);
    constantTransforms.push_back(sensorTransform);
  }
  geometry_msgs::TransformStamped tfMap2Odom =
      makeIdentityTransform("map", "odom");
  constantTransforms.push_back(tfMap2Odom);

  const std::string odomTopic = "/odom";
  for (const auto& egoPose : egoPoseInfos) {
    // write odom
    nav_msgs::Odometry odomMsg = egoPoseInfo2OdometryMsg(egoPose);
    outBag.write(odomTopic.c_str(), odomMsg.header.stamp, odomMsg);

    // write TFs
    geometry_msgs::TransformStamped tfOdom2Base =
        egoPoseInfo2TransformStamped(egoPose);
    tf::tfMessage tfMsg;
    tfMsg.transforms.push_back(tfOdom2Base);
    for (const auto& constantTransform : constantTransforms) {
      auto constantTransformWithNewStamp = constantTransform;
      constantTransformWithNewStamp.header.stamp = odomMsg.header.stamp;
      tfMsg.transforms.push_back(constantTransformWithNewStamp);
    }
    outBag.write("/tf", odomMsg.header.stamp, tfMsg);
  }
}

}  // namespace nuscenes2bag