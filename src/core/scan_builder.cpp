#include "sonar_processing/core/scan_builder.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <string>

namespace sonar_processing::core
{

namespace
{
constexpr double kDegToRad = M_PI / 180.0;
constexpr int kMaxOutputBins = 100000;

// Same rule as the ECHO driver's fan imager (sonoptix_echo::fovForRange).
inline double driver_fov_deg(double sonar_range_m) {return sonar_range_m <= 30.0 ? 120.0 : 90.0;}
}  // namespace

void ScanBuilder::validate(const ScanConfig & config)
{
  if (!(config.sonar_range_m > 0.0)) {
    throw std::invalid_argument("sonar_range_m must be > 0");
  }
  if (!(config.min_range_m >= 0.0)) {
    throw std::invalid_argument("min_range_m must be >= 0");
  }
  if (!(config.obstacle_max_range_m > config.min_range_m)) {
    throw std::invalid_argument(
            "obstacle_max_range_m must be > min_range_m");
  }
  if (config.beam_bearings_deg.empty() && !(config.fov_deg >= 0.0 && config.fov_deg < 360.0)) {
    throw std::invalid_argument("fov_deg must be in [0, 360)");
  }
  if (config.beam_bearings_deg.size() == 1) {
    throw std::invalid_argument("beam_bearings_deg needs at least 2 beams");
  }
  for (const double b : config.beam_bearings_deg) {
    if (!std::isfinite(b)) {
      throw std::invalid_argument("beam_bearings_deg must be finite");
    }
  }
  if (!(config.scan_angle_increment_deg >= 0.0)) {
    throw std::invalid_argument("scan_angle_increment_deg must be >= 0");
  }
  if (!(config.uniform_tol_deg >= 0.0)) {
    throw std::invalid_argument("uniform_tol_deg must be >= 0");
  }
}

void ScanBuilder::configure(const ScanConfig & config)
{
  validate(config);
  config_ = config;
  prepared_cols_ = -1;
  if (!config_.beam_bearings_deg.empty()) {
    prepare(static_cast<int>(config_.beam_bearings_deg.size()));   // fail early on a bad table
  }
}

void ScanBuilder::prepare(int cols)
{
  if (cols < 2) {
    throw std::invalid_argument("the polar image needs at least 2 beams");
  }

  std::vector<double> bearing(static_cast<size_t>(cols));
  if (config_.beam_bearings_deg.empty()) {
    // Uniform beam centres across the fan, column 0 = port (positive angle).
    const double fov =
      (config_.fov_deg > 0.0 ? config_.fov_deg : driver_fov_deg(config_.sonar_range_m)) * kDegToRad;
    for (int c = 0; c < cols; ++c) {
      bearing[c] = (0.5 - (c + 0.5) / cols) * fov;
    }
  } else {
    if (static_cast<int>(config_.beam_bearings_deg.size()) != cols) {
      throw std::invalid_argument(
              "beam_bearings_deg has " +
              std::to_string(config_.beam_bearings_deg.size()) + " entries but the image has " +
              std::to_string(cols) + " beams");
    }
    for (int c = 0; c < cols; ++c) {
      bearing[c] = config_.beam_bearings_deg[c] * kDegToRad;
    }
  }

  // LaserScan angles increase with the index, so work on the beams sorted by bearing.
  std::vector<int> order(static_cast<size_t>(cols));
  std::iota(order.begin(), order.end(), 0);
  std::stable_sort(order.begin(), order.end(),
    [&bearing](int a, int b) {return bearing[a] < bearing[b];});
  const double lo = bearing[order.front()];
  const double hi = bearing[order.back()];
  const double span = hi - lo;
  if (!(span > 0.0)) {
    throw std::invalid_argument("beam bearings must not all be equal");
  }

  // Uniform check: max deviation of the sorted bearings from an evenly spaced grid.
  const double uniform_inc = span / (cols - 1);
  double max_dev = 0.0;
  for (int i = 0; i < cols; ++i) {
    max_dev = std::max(max_dev, std::abs(bearing[order[i]] - (lo + i * uniform_inc)));
  }

  bin_of_beam_.assign(static_cast<size_t>(cols), 0);
  angle_min_ = lo;

  if (max_dev < config_.uniform_tol_deg * kDegToRad) {
    // Uniform: one output bin per beam, only reordered by bearing. No resampling.
    uniform_ = true;
    n_bins_ = cols;
    angle_inc_ = uniform_inc;
    for (int i = 0; i < cols; ++i) {
      bin_of_beam_[order[i]] = i;
    }
    fill_src_.assign(static_cast<size_t>(n_bins_), -1);
  } else {
    uniform_ = false;
    double target = config_.scan_angle_increment_deg * kDegToRad;
    if (target <= 0.0) {
      // Finest original spacing (coincident beams are ignored, they share a bin anyway).
      target = std::numeric_limits<double>::infinity();
      for (int i = 1; i < cols; ++i) {
        const double d = bearing[order[i]] - bearing[order[i - 1]];
        if (d > 1e-9) {
          target = std::min(target, d);
        }
      }
    }
    const double bins = std::ceil(span / target - 1e-6) + 1.0;
    if (!(bins <= kMaxOutputBins)) {
      throw std::invalid_argument(
              "scan angle increment too small (more than " +
              std::to_string(kMaxOutputBins) + " output bins)");
    }
    n_bins_ = std::max(2, static_cast<int>(bins));
    // Stretch the increment so the last bin lands exactly on the outermost bearing; this keeps
    // the output spacing <= the target.
    angle_inc_ = span / (n_bins_ - 1);

    std::vector<char> hit(static_cast<size_t>(n_bins_), 0);
    for (int c = 0; c < cols; ++c) {
      const long k = std::lround((bearing[c] - lo) / angle_inc_);
      bin_of_beam_[c] = static_cast<int>(std::clamp(k, 0L, static_cast<long>(n_bins_ - 1)));
      hit[bin_of_beam_[c]] = 1;
    }

    // Empty output bins copy the nearest original beam (by bearing). No interpolation between
    // beams: blending a near and a far return would invent an obstacle in between.
    fill_src_.assign(static_cast<size_t>(n_bins_), -1);
    size_t j = 0;   // order[j] = last beam with bearing <= bin angle
    for (int k = 0; k < n_bins_; ++k) {
      if (hit[k]) {
        continue;
      }
      const double a = lo + k * angle_inc_;
      while (j + 1 < order.size() && bearing[order[j + 1]] <= a) {
        ++j;
      }
      int src = order[j];
      if (j + 1 < order.size() &&
        bearing[order[j + 1]] - a < a - bearing[order[j]])
      {
        src = order[j + 1];
      }
      fill_src_[k] = src;
    }
  }

  prepared_cols_ = cols;
}

void ScanBuilder::build(
  const cv::Mat & mask, sensor_msgs::msg::LaserScan & scan, std::vector<int> & return_rows)
{
  if (mask.type() != CV_8UC1) {
    throw std::invalid_argument("scan builder expects a CV_8UC1 mask");
  }
  if (mask.cols != prepared_cols_) {
    prepare(mask.cols);
  }

  const int rows = mask.rows;
  const int cols = mask.cols;
  constexpr float kInf = std::numeric_limits<float>::infinity();

  // 1. First return per beam within [min_range_m, obstacle_max_range_m]. Scanned row by row
  //    (cache friendly), stopping once every beam has a return.
  return_rows.assign(static_cast<size_t>(cols), -1);
  beam_range_.assign(static_cast<size_t>(cols), kInf);
  if (rows > 0) {
    const double res = config_.sonar_range_m / rows;
    const int row_lo = std::max(0, static_cast<int>(std::ceil(config_.min_range_m / res - 1e-9)));
    const int row_hi = std::min(
      rows - 1, static_cast<int>(std::floor(config_.obstacle_max_range_m / res + 1e-9)));
    int remaining = cols;
    for (int r = row_lo; r <= row_hi && remaining > 0; ++r) {
      const uint8_t * p = mask.ptr<uint8_t>(r);
      for (int c = 0; c < cols; ++c) {
        if (p[c] && return_rows[c] < 0) {
          return_rows[c] = r;
          // Near edge of the cell (conservative), clamped against rounding below range_min.
          beam_range_[c] = static_cast<float>(std::max(r * res, config_.min_range_m));
          --remaining;
        }
      }
    }
  }

  // 2. Resample onto the uniform grid: each bin takes the MIN range of the beams inside it.
  scan.ranges.assign(static_cast<size_t>(n_bins_), kInf);
  for (int c = 0; c < cols; ++c) {
    float & out = scan.ranges[bin_of_beam_[c]];
    out = std::min(out, beam_range_[c]);
  }
  for (int k = 0; k < n_bins_; ++k) {
    if (fill_src_[k] >= 0) {
      scan.ranges[k] = beam_range_[fill_src_[k]];
    }
  }

  // 3. Empty beams: +inf lets the costmap clear along the full beam; range_max is the
  //    alternative for consumers that reject non-finite ranges.
  if (!config_.empty_beam_inf) {
    const float range_max = static_cast<float>(config_.obstacle_max_range_m);
    for (float & r : scan.ranges) {
      if (std::isinf(r)) {
        r = range_max;
      }
    }
  }

  scan.angle_min = static_cast<float>(angle_min_);
  scan.angle_max = static_cast<float>(angle_min_ + (n_bins_ - 1) * angle_inc_);
  scan.angle_increment = static_cast<float>(angle_inc_);
  scan.range_min = static_cast<float>(config_.min_range_m);
  scan.range_max = static_cast<float>(config_.obstacle_max_range_m);
  scan.time_increment = 0.0f;   // multibeam FLS: all beams are formed from the same ping
  scan.intensities.clear();
}

}  // namespace sonar_processing::core
