#pragma once

#include <array>
#include <cstdint>
#include <vector>

#include <opencv2/core.hpp>

namespace sonar_processing::core
{

// Window half-sizes, in cells, on image axes: y = rows = range, x = columns = beams.
// All fields must be set by the caller; zero values fail validate().
struct OsCfarConfig
{
  int guard_cells_x{0};
  int guard_cells_y{0};
  int training_cells_x{0};
  int training_cells_y{0};
  double false_alarm_rate{0.0};    // Pfa, in (0, 1); sets the threshold factor alpha
  double min_intensity{0.0};       // absolute floor: CUT must also exceed this
  int kth_order_statistic{0};      // rank k (1-based) of the sorted training cells, in [1, N]
};

// 2D ordered-statistic CFAR for mono8 images (fast version, Villar et al. 2019):
// Perreault-Hebert column histograms plus an incremental k-th order statistic search, so the
// cost per cell does not grow with the window. Borders are mirrored so every cell sees N
// training cells.

class OsCfar
{
public:
  // Throws std::invalid_argument if the config is not usable.
  static void validate(const OsCfarConfig & config);
  void configure(const OsCfarConfig & config);

  // src: CV_8UC1 intensity. dst: CV_8UC1 mask, 255 = detection; may alias src.
  // threshold (optional, for debugging): CV_8UC1, per cell the value the CUT had to exceed,
  // i.e. max(alpha * X_(k), min_intensity) saturated to 255 (255 = could never detect).
  void detect(const cv::Mat & src, cv::Mat & dst, cv::Mat * threshold = nullptr);

private:
  // OsCfar configuration
  OsCfarConfig config_;
  int outer_x_{0}, outer_y_{0};          // outer window half-sizes
  int n_training_{0};                    // N: training cells in a full (interior) window
  double alpha_os_{1.0};                 // threshold factor: detection iff cut > alpha * X_(k)
  std::array<uint16_t, 256> cut_limit_{};  // detection iff cut >= cut_limit_[X_(k)], 256 = never
  std::array<uint8_t, 256> threshold_lut_{};  // debug: cut_limit_ - 1, saturated to 255

  // Fast OS-CFAR buffers (Villar et al. 2019), reused between frames.
  // P (previous k-th statistic) and L (#training values < P) are locals in detect() so the
  // compiler can keep them in registers; G = N - L is implied and not stored.
  std::array<int32_t, 256> kernel_histogram_{};  // H: training cells only (outer minus guard)
  std::vector<uint16_t> col_hist_;                // padded_cols x 256 column histograms h
  cv::Mat padded_;                                // input padded by (outer_y_, outer_x_)
};

}  // namespace sonar_processing::core
