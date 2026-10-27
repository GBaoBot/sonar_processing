#include "sonar_processing/ca_cfar.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include <opencv2/imgproc.hpp>

namespace sonar_processing
{

namespace
{
// Sum over [c0, c1) between integral rows top and bottom. Unsigned wrap-around keeps the
// difference exact even if the integral itself overflows 32 bits.
inline uint32_t rowSpanSum(const uint32_t * top, const uint32_t * bottom, int c0, int c1)
{
  return bottom[c1] - bottom[c0] - top[c1] + top[c0];
}
}  // namespace

void CaCfar::validate(const CaCfarConfig & config)
{
  if (config.guard_cells_range < 0 || config.guard_cells_beam < 0) {
    throw std::invalid_argument("Sonar Processing: CFAR guard cells must be >= 0");
  }
  if (config.training_cells_range < 0 || config.training_cells_beam < 0) {
    throw std::invalid_argument("Sonar Processing: CFAR training cells must be >= 0");
  }
  if (config.training_cells_range == 0 && config.training_cells_beam == 0) {
    throw std::invalid_argument("Sonar Processing: CFAR needs training cells on at least one axis");
  }
  if (!(config.false_alarm_rate > 0.0 && config.false_alarm_rate < 1.0)) {
    throw std::invalid_argument("Sonar Processing: CFAR false alarm rate must be in (0, 1)");
  }
}

void CaCfar::configure(const CaCfarConfig & config)
{
  validate(config);
  config_ = config;
  outer_r_ = config.guard_cells_range + config.training_cells_range;
  outer_b_ = config.guard_cells_beam + config.training_cells_beam;

  const int max_n = (2 * outer_r_ + 1) * (2 * outer_b_ + 1);
  const int interior_n = max_n -
    (2 * config.guard_cells_range + 1) * (2 * config.guard_cells_beam + 1);

  // alpha(N) = N * (Pfa^(-1/N) - 1); clipped windows at the border have fewer cells.
  alpha_lut_.assign(static_cast<size_t>(max_n) + 1, 0.0);
  for (int n = 1; n <= max_n; ++n) {
    alpha_lut_[n] = n * (std::pow(config.false_alarm_rate, -1.0 / n) - 1.0);
  }

  // sum < cut * N / alpha(N), sum is an integer: sum < ceil(cut * N / alpha(N)).
  const double scale = interior_n / alpha_lut_[interior_n];
  constexpr double kMaxLimit = std::numeric_limits<uint32_t>::max();
  for (int cut = 0; cut < 256; ++cut) {
    sum_limit_[cut] = cut > config.min_intensity ?
      static_cast<uint32_t>(std::min(std::ceil(cut * scale), kMaxLimit)) : 0u;
  }
}

void CaCfar::detect(const cv::Mat & src, cv::Mat & dst)
{
  if (src.type() != CV_8UC1) {
    throw std::invalid_argument("Sonar Processing: CFAR expects a CV_8UC1 (mono8) image");
  }
  if (alpha_lut_.empty()) {
    throw std::logic_error("Sonar Processing: CFAR used before configure()");
  }

  const int rows = src.rows;
  const int cols = src.cols;
  dst.create(rows, cols, CV_8UC1);
  cv::integral(src, integral_, CV_32S);

  // Interior: full windows, fixed N, integer-only test.
  const int gr = config_.guard_cells_range;
  const int gb = config_.guard_cells_beam;
  const int y_begin = std::min(outer_r_, rows);
  const int y_end = std::max(y_begin, rows - outer_r_);
  const int x_begin = std::min(outer_b_, cols);
  const int x_end = std::max(x_begin, cols - outer_b_);

  for (int y = 0; y < rows; ++y) {
    if (y < y_begin || y >= y_end) {
      detectClipped(src, dst, y, 0, cols);
      continue;
    }
    detectClipped(src, dst, y, 0, x_begin);
    detectClipped(src, dst, y, x_end, cols);

    const uint32_t * o_top = integral_.ptr<uint32_t>(y - outer_r_);
    const uint32_t * o_bot = integral_.ptr<uint32_t>(y + outer_r_ + 1);
    const uint32_t * g_top = integral_.ptr<uint32_t>(y - gr);
    const uint32_t * g_bot = integral_.ptr<uint32_t>(y + gr + 1);
    const uint8_t * cut_row = src.ptr<uint8_t>(y);
    uint8_t * out_row = dst.ptr<uint8_t>(y);

    for (int x = x_begin; x < x_end; ++x) {
      const uint32_t training_sum =
        rowSpanSum(o_top, o_bot, x - outer_b_, x + outer_b_ + 1) -
        rowSpanSum(g_top, g_bot, x - gb, x + gb + 1);
      out_row[x] = training_sum < sum_limit_[cut_row[x]] ? 255 : 0;
    }
  }
}

void CaCfar::detectClipped(const cv::Mat & src, cv::Mat & dst, int y, int x0, int x1) const
{
  const int rows = src.rows;
  const int cols = src.cols;
  const int gr = config_.guard_cells_range;
  const int gb = config_.guard_cells_beam;

  const int oy0 = std::max(0, y - outer_r_);
  const int oy1 = std::min(rows, y + outer_r_ + 1);
  const int gy0 = std::max(0, y - gr);
  const int gy1 = std::min(rows, y + gr + 1);
  const uint32_t * o_top = integral_.ptr<uint32_t>(oy0);
  const uint32_t * o_bot = integral_.ptr<uint32_t>(oy1);
  const uint32_t * g_top = integral_.ptr<uint32_t>(gy0);
  const uint32_t * g_bot = integral_.ptr<uint32_t>(gy1);
  const uint8_t * cut_row = src.ptr<uint8_t>(y);
  uint8_t * out_row = dst.ptr<uint8_t>(y);

  for (int x = x0; x < x1; ++x) {
    const int ox0 = std::max(0, x - outer_b_);
    const int ox1 = std::min(cols, x + outer_b_ + 1);
    const int gx0 = std::max(0, x - gb);
    const int gx1 = std::min(cols, x + gb + 1);

    const int n = (oy1 - oy0) * (ox1 - ox0) - (gy1 - gy0) * (gx1 - gx0);
    const uint8_t cut = cut_row[x];
    if (n <= 0 || cut <= config_.min_intensity) {   // n <= 0: image smaller than guard window
      out_row[x] = 0;
      continue;
    }

    const uint32_t training_sum =
      rowSpanSum(o_top, o_bot, ox0, ox1) - rowSpanSum(g_top, g_bot, gx0, gx1);
    // cut > alpha(n) * sum / n, without the divide
    out_row[x] = static_cast<double>(cut) * n > alpha_lut_[n] * training_sum ? 255 : 0;
  }
}

}  // namespace sonar_processing
