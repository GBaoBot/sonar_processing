#pragma once

#include <opencv2/core.hpp>

namespace sonar_processing::core
{

// All fields must be set by the caller; zero values fail validate().
struct GaussianFilterConfig
{
  int kernel_size{0};   // positive odd
  double sigma_x{0.0};  // > 0
  double sigma_y{0.0};  // > 0
};

// Gaussian smoothing of a mono8 intensity image.
class GaussianFilter
{
public:
  // Throws std::invalid_argument if the config is not usable.
  static void validate(const GaussianFilterConfig & config);

  // src: CV_8UC1. Validates the config on every call.
  static void perform(const cv::Mat & src, cv::Mat & dst, const GaussianFilterConfig & config);
};

}  // namespace sonar_processing::core
