#include "sonar_processing/core/morphology_filter.hpp"

#include <stdexcept>

#include <opencv2/imgproc.hpp>

namespace sonar_processing::core
{

void MorphologyFilter::validate(const MorphologyConfig & config)
{
  if (config.kernel_x <= 0 || config.kernel_x % 2 == 0 ||
    config.kernel_y <= 0 || config.kernel_y % 2 == 0)
  {
    throw std::invalid_argument(
            "morphology kernel sizes must be positive odd values");
  }
}

void MorphologyFilter::configure(const MorphologyConfig & config)
{
  validate(config);
  config_ = config;
  // cv::Size is (width, height) = (beams, range samples).
  kernel_ = cv::getStructuringElement(
    cv::MORPH_RECT, cv::Size(config.kernel_x, config.kernel_y));
}

void MorphologyFilter::apply(const cv::Mat & mask, cv::Mat & dst)
{
  if (mask.type() != CV_8UC1) {
    throw std::invalid_argument("morphology expects a CV_8UC1 mask");
  }
  if (kernel_.empty()) {
    throw std::logic_error("MorphologyFilter used before configure()");
  }

  if (config_.kernel_x > 0 && config_.kernel_y > 0) {
    cv::morphologyEx(mask, dst, cv::MORPH_OPEN, kernel_);
  } else {
    mask.copyTo(dst);
  }

  return;
}

}  // namespace sonar_processing::core
