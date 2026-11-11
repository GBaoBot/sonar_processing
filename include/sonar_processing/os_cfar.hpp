#pragma once

#include <array>
#include <cstdint>
#include <vector>

#include <opencv2/core.hpp>

namespace sonar_processing
{

struct OsCfarConfig
{
  int guard_cells_x{2};
  int guard_cells_y{2};
  int training_cells_x{10};
  int training_cells_y{10};
  double false_alarm_rate{1.0e-3};
  double min_intensity{0.0};
  int kth_order_statistic{8};      // rank k (1-based) of the sorted training cells, in [1, N]
};

class OsCfar
{
public:
  // Validate and Configure parameters
  static void validate(const OsCfarConfig & config);
  void configure(const OsCfarConfig & config);

  void detect(const cv::Mat & src, cv::Mat & dst);

private:
  // OsCfar configuration
  OsCfarConfig config_;
  int outer_x_{0}, outer_y_{0};          // outer window half-sizes
  int n_training_{0};                    // N: training cells in a full (interior) window
  double alpha_os_{1.0};                 // threshold factor: detection iff cut > alpha * X_(k)
  std::array<uint16_t, 256> cut_limit_{};  // detection iff cut >= cut_limit_[X_(k)], 256 = never

  // Image size
  int n_rows_{640};
  int n_cols_{256};
  
  // Fast OS-CFAR buffers (Villar et al. 2019), reused between frames.
  // P (previous k-th statistic) and L (#training values < P) are locals in detect() so the
  // compiler can keep them in registers; G = N - L is implied and not stored.
  std::array<int32_t, 256> kernel_histogram_{};  // H: training cells only (outer minus guard)
  std::vector<uint16_t> col_hist_;                // padded_cols x 256 column histograms h
  cv::Mat padded_;                                // input padded by (outer_y_, outer_x_)
};

} // namespace sonar_processing