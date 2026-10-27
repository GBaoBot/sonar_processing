#pragma once

#include <array>
#include <cstdint>
#include <vector>

#include <opencv2/core.hpp>

namespace sonar_processing
{

// Window half-sizes, in cells. Rows = range samples, cols = beams (polar sonar image).
// Around each cell under test (CUT): a guard window of (2*guard+1) cells per axis, surrounded
// by a band of training cells, so the outer window is 2*(guard+training)+1 cells per axis.
struct CaCfarConfig
{
  int guard_cells_range{2};
  int guard_cells_beam{2};
  int training_cells_range{8};
  int training_cells_beam{8};
  double false_alarm_rate{1e-3};   // Pfa, in (0, 1)
  double min_intensity{0.0};       // CUT must also exceed this absolute floor
};

// 2D cell-averaging CFAR for mono8 images. The training-cell sum for every CUT is taken from
// a 32-bit integral image (outer window sum - guard window sum), so the cost per cell is
// O(1) regardless of the window size.
//
// Interior cells all have the same training cell count N, so the test
//   cut > alpha(N) / N * sum   <=>   sum < cut * N / alpha(N)
// is precomputed per CUT value (0..255) and becomes a single integer compare. Cells near the
// border use clipped windows with their actual N, which keeps Pfa constant at the edges.
class CaCfar
{
public:
  // Throws std::invalid_argument if the config is not usable.
  static void validate(const CaCfarConfig & config);

  void configure(const CaCfarConfig & config);

  // src: CV_8UC1. dst: CV_8UC1 mask, 255 = detection.
  // dst may wrap an external buffer; it is (re)allocated only if size/type differ.
  void detect(const cv::Mat & src, cv::Mat & dst);

private:
  // Clipped-window path for cells in [x0, x1) of row y.
  void detectClipped(const cv::Mat & src, cv::Mat & dst, int y, int x0, int x1) const;

  CaCfarConfig config_;
  int outer_r_{0}, outer_b_{0};          // outer window half-sizes
  std::vector<double> alpha_lut_;        // threshold factor, indexed by training cell count
  std::array<uint32_t, 256> sum_limit_{};  // interior: detection iff sum < sum_limit_[cut]
  cv::Mat integral_;                     // CV_32S, (rows+1) x (cols+1), reused between frames
};

}  // namespace sonar_processing
