#pragma once

#include <stdexcept>
#include <string>

#include <opencv2/core.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace sonar_processing
{

// cv::Mat sharing the message's buffer (no copy). Writing to the Mat writes to the message.
// Throws if the image is not a well-formed mono8 image.
inline cv::Mat wrap_mono8(sensor_msgs::msg::Image & image)
{
  if (image.encoding != sensor_msgs::image_encodings::MONO8) {
    throw std::invalid_argument(
            "expected a mono8 image, got '" + image.encoding + "'");
  }
  if (image.step < image.width ||
    image.data.size() < static_cast<size_t>(image.step) * image.height)
  {
    throw std::invalid_argument("mono8 image buffer is smaller than step * height");
  }
  return cv::Mat(
    static_cast<int>(image.height), static_cast<int>(image.width), CV_8UC1,
    image.data.data(), image.step);
}

}  // namespace sonar_processing
