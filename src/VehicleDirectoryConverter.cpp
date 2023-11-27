#include "nuscenes2bag/VehicleDirectoryConverter.hpp"

#include <thread>

#include "nuscenes2bag/utils.hpp"

namespace nuscenes2bag {

boost::optional<nav_msgs::Odometry> readVehicleFile(
    const VehicleData& vehicleData) noexcept {
  try {
    nav_msgs::Odometry msg;
    msg.header = std_msgs::Header();

    assignArray2Vector3(msg.pose.pose.position, vehicleData.pos);
    assignArray2Quaternion(msg.pose.pose.orientation, vehicleData.orientation);

    assignArray2Vector3(msg.twist.twist.linear, vehicleData.vel);
    assignArray2Vector3(msg.twist.twist.angular, vehicleData.rotation_rate);

    return boost::optional<nav_msgs::Odometry>(msg);

  } catch (const std::exception& e) {
    PRINT_EXCEPTION(e);
  }

  return boost::none;
}

}  // namespace nuscenes2bag