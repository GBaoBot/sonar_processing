#include "sonar_processing/os_cfar.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

#include <opencv2/imgproc.hpp>

namespace sonar_processing
{

namespace
{
inline int trainingCellCount(int guard_x, int guard_y, int outer_x, int outer_y)
{
  return (2 * outer_x + 1) * (2 * outer_y + 1) - (2 * guard_x + 1) * (2 * guard_y + 1);
}

inline double osPfa(int n, int k, double alpha)
{
  double log_pfa = 0.0;
  for (int i = 0; i < k; ++i) {
    log_pfa += std::log(static_cast<double>(n - i)) - std::log(n - i + alpha);
  }
  return std::exp(log_pfa);
}

// Section 3.3.4: incremental k-th order statistic search using the breakdown point.
// Starts from the previous statistic P. Invariant on entry and exit: L = #training values < P.
// Returns the smallest v with count(<= v) >= k. P stays in [0, 255]: walking down needs
// L >= k >= 1 values below P (so P > 0), and walking up stops at 255 at the latest because
// count(<= 255) = N >= k.
inline int kthOrderStatistic(const int32_t * H, int k, int P, int & L)
{
  if (L >= k) {        // answer is below P: walk down
    do {
      --P;
      L -= H[P];
    } while (L >= k);
  } else {             // answer is at or above P: walk up
    while (L + H[P] < k) {
      L += H[P];
      ++P;
    }
  }
  return P;
}
}  // namespace

void OsCfar::validate(const OsCfarConfig & config)
{
  if (config.guard_cells_y < 0 || config.guard_cells_x < 0) {
    throw std::invalid_argument("Sonar Processing: CFAR guard cells must be >= 0");
  }
  if (config.training_cells_y < 0 || config.training_cells_x < 0) {
    throw std::invalid_argument("Sonar Processing: CFAR training cells must be >= 0");
  }
  if (config.training_cells_y == 0 && config.training_cells_x == 0) {
    throw std::invalid_argument("Sonar Processing: CFAR needs training cells on at least one axis");
  }
  if (!(config.false_alarm_rate > 0.0 && config.false_alarm_rate < 1.0)) {
    throw std::invalid_argument("Sonar Processing: CFAR false alarm rate must be in (0, 1)");
  }
  const int n = trainingCellCount(
    config.guard_cells_x, config.guard_cells_y,
    config.guard_cells_x + config.training_cells_x,
    config.guard_cells_y + config.training_cells_y);
  if (config.kth_order_statistic < 1 || config.kth_order_statistic > n) {
    throw std::invalid_argument(
            "Sonar Processing: CFAR kth order statistic must be in [1, N] (N = " +
            std::to_string(n) + " training cells)");
  }
}

void OsCfar::configure(const OsCfarConfig & config)
{
  validate(config);
  config_ = config;
  outer_x_ = config.guard_cells_x + config.training_cells_x;
  outer_y_ = config.guard_cells_y + config.training_cells_y;
  n_training_ = trainingCellCount(config.guard_cells_x, config.guard_cells_y, outer_x_, outer_y_);

  const int k = config.kth_order_statistic;
  double lo = 0.0;
  double hi = 1.0;
  while (osPfa(n_training_, k, hi) > config.false_alarm_rate) {
    lo = hi;
    hi *= 2.0;
  }
  for (int i = 0; i < 100 && hi - lo > 1e-9 * hi; ++i) {
    const double mid = 0.5 * (lo + hi);
    (osPfa(n_training_, k, mid) > config.false_alarm_rate ? lo : hi) = mid;
  }
  alpha_os_ = hi;

  for (int xk = 0; xk < 256; ++xk) {
    const double limit = std::floor(std::max(alpha_os_ * xk, config.min_intensity)) + 1.0;
    cut_limit_[xk] = static_cast<uint16_t>(std::min(limit, 256.0));
  }

  kernel_histogram_.fill(0);
}

void OsCfar::detect(const cv::Mat & src, cv::Mat & dst)
{
  if (src.type() != CV_8UC1) {
    throw std::invalid_argument("Sonar Processing: CFAR expects a CV_8UC1 (mono8) image");
  }

  if (n_training_ <= 0) {
    throw std::logic_error("Sonar Processing: OS-CFAR detect() called before configure()");
  }
  if (2 * outer_y_ + 1 > std::numeric_limits<uint16_t>::max()) {
    throw std::invalid_argument("Sonar Processing: OS-CFAR window too tall for uint16 histograms");
  }

  const int rows = src.rows;
  const int cols = src.cols;
  dst.create(rows, cols, CV_8UC1);
  if (rows == 0 || cols == 0) {
    return;
  }

  // Border handling: alpha_os_ was solved for a fixed N, so every pixel must see exactly N
  // training cells. Pad once with BORDER_REFLECT_101 (mirror without repeating the edge pixel).
  // OpenCV reflects repeatedly, so images smaller than the window also work. All reads below go
  // through padded_ (including the CUT), so dst may safely alias src.
  // Output (i, j) has its window center at padded (i + outer_y_, j + outer_x_).
  cv::copyMakeBorder(
    src, padded_, outer_y_, outer_y_, outer_x_, outer_x_, cv::BORDER_REFLECT_101);

  const int padded_cols = padded_.cols;
  const int win_h = 2 * outer_y_ + 1;
  const int win_w = 2 * outer_x_ + 1;
  const int guard_x = config_.guard_cells_x;
  const int guard_y = config_.guard_cells_y;
  const int k = config_.kth_order_statistic;
  const size_t padded_step = padded_.step[0];

  // Reallocates only when the padded width changes (image size or outer_x_ changed).
  col_hist_.resize(static_cast<size_t>(padded_cols) * 256);
  std::fill(col_hist_.begin(), col_hist_.end(), uint16_t{0});
  uint16_t * const col_hist = col_hist_.data();
  int32_t * const H = kernel_histogram_.data();

  // Section 3.3.1 (Perreault-Hebert distributive histograms): initialize one histogram per
  // padded column with padded rows [0, 2*outer_y_].
  for (int r = 0; r < win_h; ++r) {
    const uint8_t * p = padded_.ptr<uint8_t>(r);
    for (int c = 0; c < padded_cols; ++c) {
      ++col_hist[c * 256 + p[c]];
    }
  }

  int P = 0;  // Pxk: previous k-th statistic, carried from the previous row's first pixel

  for (int i = 0; i < rows; ++i) {
    // Section 3.3.1: move every column histogram down one row (2 ops per column).
    if (i > 0) {
      const uint8_t * leaving = padded_.ptr<uint8_t>(i - 1);
      const uint8_t * entering = padded_.ptr<uint8_t>(i + 2 * outer_y_);
      for (int c = 0; c < padded_cols; ++c) {
        --col_hist[c * 256 + leaving[c]];
        ++col_hist[c * 256 + entering[c]];
      }
    }

    const uint8_t * cut_row = padded_.ptr<uint8_t>(i + outer_y_);
    const uint8_t * guard_row0 = padded_.ptr<uint8_t>(i + outer_y_ - guard_y);
    uint8_t * dst_row = dst.ptr<uint8_t>(i);

    // Section 3.3.2: kernel histogram for j = 0. Sum the outer-window column histograms,
    // then remove the guard rectangle (which contains the CUT) pixel by pixel.
    std::fill(H, H + 256, 0);
    for (int c = 0; c < win_w; ++c) {
      const uint16_t * h = col_hist + c * 256;
      for (int l = 0; l < 256; ++l) {
        H[l] += h[l];
      }
    }
    for (int r = 0; r <= 2 * guard_y; ++r) {
      const uint8_t * p = guard_row0 + r * padded_step;
      for (int c = outer_x_ - guard_x; c <= outer_x_ + guard_x; ++c) {
        --H[p[c]];
      }
    }

    // Section 3.3.4: recompute L for the carried P, then search.
    int L = 0;
    for (int l = 0; l < P; ++l) {
      L += H[l];
    }
    P = kthOrderStatistic(H, k, P, L);
    const int row_start_P = P;

    // Section 3.3.5: detection.
    dst_row[0] = cut_row[outer_x_] >= cut_limit_[P] ? 255 : 0;

    for (int j = 1; j < cols; ++j) {
      // a. Sections 3.3.2-3.3.3: outer update H += hr - hl, with L updated for bins < P.
      //    Split at P so neither loop branches per bin (both auto-vectorize).
      //    When training_x == 0 the entering/leaving columns are guard columns; H can then
      //    hold transient negative counts that step b cancels exactly.
      const uint16_t * hr = col_hist + (j + 2 * outer_x_) * 256;  // entering column
      const uint16_t * hl = col_hist + (j - 1) * 256;             // leaving column
      int dL = 0;
      for (int l = 0; l < P; ++l) {
        const int d = static_cast<int>(hr[l]) - static_cast<int>(hl[l]);
        H[l] += d;
        dL += d;
      }
      for (int l = P; l < 256; ++l) {
        H[l] += static_cast<int>(hr[l]) - static_cast<int>(hl[l]);
      }
      L += dL;

      // b. Guard correction (pixel-wise): the leaving guard column rejoins the training set,
      //    the entering guard column leaves it.
      const int c_guard_leave = j - 1 + outer_x_ - guard_x;
      const int c_guard_enter = j + outer_x_ + guard_x;
      for (int r = 0; r <= 2 * guard_y; ++r) {
        const uint8_t * p = guard_row0 + r * padded_step;
        const int v_back = p[c_guard_leave];
        ++H[v_back];
        L += (v_back < P);
        const int v_out = p[c_guard_enter];
        --H[v_out];
        L -= (v_out < P);
      }

      // c. Section 3.3.4: k-th search with the breakdown point, starting from previous P.
      P = kthOrderStatistic(H, k, P, L);

      // d. Section 3.3.5: detection. cut_limit_ already folds in alpha_os_, min_intensity and
      //    the strict '>' (+1); 256 means never detect.
      dst_row[j] = cut_row[j + outer_x_] >= cut_limit_[P] ? 255 : 0;
    }

    P = row_start_P;  // next row starts from this row's first-pixel statistic
  }
}

} // namespace sonar_processing