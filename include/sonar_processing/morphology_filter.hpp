#pragma once

#include <cstdint>
#include <vector>

#include <opencv2/core.hpp>

namespace sonar_processing
{

// Kernel sizes are in cells. Rows = range samples, cols = beams (polar sonar image).
struct MorphologyConfig
{
  int kernel_range{3};     // opening kernel height, positive odd; 1 = no opening along range
  int kernel_beam{3};      // opening kernel width, positive odd; 1 = no opening along beams
  int min_blob_cells{4};   // 8-connected blobs with fewer cells are removed; <= 1 disables
};

// Cleans a binary detection mask: morphological opening (erode then dilate) removes specks
// thinner than the kernel, then connected components smaller than min_blob_cells are dropped.
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

}  // namespace sonar_processing
