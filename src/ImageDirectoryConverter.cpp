#include "nuscenes2bag/ImageDirectoryConverter.hpp"

#include <thread>

#include "nuscenes2bag/utils.hpp"

namespace nuscenes2bag {

boost::optional<sensor_msgs::Image> readImageFile(
    const fs::path& filePath) noexcept {
  cv::Mat image;
  try {
    image = imread(filePath.string().c_str(), cv::IMREAD_COLOR);
    sensor_msgs::ImagePtr msg =
        cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();

    return boost::optional<sensor_msgs::Image>(*msg);

  } catch (const std::exception& e) {
    PRINT_EXCEPTION(e);
  }

  return boost::none;
}

}  // namespace nuscenes2bag