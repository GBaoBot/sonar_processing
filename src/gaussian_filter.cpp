#include "sonar_processing/gaussian_filter.hpp"

#include <stdexcept>

#include <opencv2/imgproc.hpp>

namespace sonar_processing
{

void GaussianFilter::validate(const GaussianFilterConfig & config)
{
  if (config.kernel_size <= 0 || config.kernel_size % 2 == 0) {
    throw std::invalid_argument("Sonar Processing: kernel size must be a positive odd value");
  }
  if (config.sigma_x <= 0 || config.sigma_y <= 0) {
    throw std::invalid_argument("Sonar Processing: sigma x and y must be positive");
  }
}

void GaussianFilter::perform(
  const cv::Mat & src, cv::Mat & dst, const GaussianFilterConfig & config)
{
  validate(config);
  const cv::Size kernel_size(config.kernel_size, config.kernel_size);
  cv::GaussianBlur(src, dst, kernel_size, config.sigma_x, config.sigma_y);
}

}  // namespace sonar_processing
