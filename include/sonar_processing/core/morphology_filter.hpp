#pragma once

#include <cstdint>
#include <vector>

#include <opencv2/core.hpp>

namespace sonar_processing::core
{

// Kernel sizes are in cells. Rows = range samples, cols = beams (polar sonar image).
// All fields must be set by the caller; zero kernels fail validate().
struct MorphologyConfig
{
  int kernel_x{0};  // opening kernel width (beams/cols), positive odd; 1 = no opening along x
  int kernel_y{0};  // opening kernel height (range/rows), positive odd; 1 = no opening along y
};

// Cleans a binary detection mask: morphological opening (erode then dilate) removes specks
// thinner than the kernel.
class MorphologyFilter
{
public:
  // Throws std::invalid_argument if the config is not usable.
  static void validate(const MorphologyConfig & config);
  void configure(const MorphologyConfig & config);

  // mask: CV_8UC1, 255 = detection. dst: CV_8UC1 0/255; may wrap an external buffer of the
  // same size (it is not reallocated then). dst must not alias mask.
  void apply(const cv::Mat & mask, cv::Mat & dst);

private:
  MorphologyConfig config_;
  cv::Mat kernel_;
  cv::Mat labels_, stats_, centroids_;   // connectedComponentsWithStats buffers, reused
  std::vector<uint8_t> keep_;            // per label: 255 = keep, 0 = drop
};

}  // namespace sonar_processing::core
