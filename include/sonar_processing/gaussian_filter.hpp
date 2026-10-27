#pragma once

#include <opencv2/core.hpp>

namespace sonar_processing
{

struct GaussianFilterConfig
{
  int kernel_size{5};
  double sigma_x{1.0};
  double sigma_y{1.0};
};

class GaussianFilter
{
public:
  // Throws std::invalid_argument if the config is not usable.
  static void validate(const GaussianFilterConfig & config);

  static void perform(const cv::Mat & src, cv::Mat & dst, const GaussianFilterConfig & config);
};

}  // namespace sonar_processing
