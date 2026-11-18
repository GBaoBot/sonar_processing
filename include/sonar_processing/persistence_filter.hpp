#pragma once

#include <vector>

#include <opencv2/core.hpp>

namespace sonar_processing
{

struct PersistenceConfig
{
  int persist_n{2};          // detections needed ...
  int persist_m{3};          // ... within the last M pings (current ping included), N <= M
  int persist_tol_range{1};  // tolerance dilation half-size along range, in cells (>= 0)
  int persist_tol_beam{1};   // tolerance dilation half-size along beams, in cells (>= 0)
};

// M-of-N style temporal filter in polar index space (stands in for a log-odds grid).
//
// A cell detected in the current ping is kept only if a detection fell within
// +/- (persist_tol_range, persist_tol_beam) cells of it in at least persist_n of the last
// persist_m pings. Each ping's mask is dilated by the tolerance window, stored in a ring of M
// slots, and a running per-cell count of the stored masks is updated incrementally (add the new
// slot, subtract the evicted one), so the cost per ping does not depend on M.
//
// NOTE: this is approximate under fast motion. Polar cells are tied to the sonar, not the
// world, so when the vehicle translates or yaws by more than the tolerance window between pings
// a real, static obstacle moves to different (range, beam) cells and loses its history. The
// tolerance only absorbs small motion; fast motion causes missed detections (never phantom
// ones, because the current ping must still detect the cell). Ego-motion compensation would
// require projecting the history into a world-fixed frame.
class PersistenceFilter
{
public:
  // Throws std::invalid_argument if the config is not usable.
  static void validate(const PersistenceConfig & config);
  void configure(const PersistenceConfig & config);

  // Drops the ping history (e.g. after a pause or a geometry change).
  void reset();

  // mask: CV_8UC1, 255 = detection in the current ping. dst: CV_8UC1 0/255; may wrap an external
  // buffer of the same size and may alias mask. An image size change resets the history.
  void apply(const cv::Mat & mask, cv::Mat & dst);

private:
  PersistenceConfig config_;
  cv::Mat kernel_;
  std::vector<cv::Mat> history_;  // ring of M dilated masks, values 0/1
  int head_{0};                   // slot the next ping is written to
  int filled_{0};                 // number of valid slots, <= M
  cv::Mat count_;                 // CV_8UC1: sum of the valid history slots per cell
  cv::Mat persistent_;            // CV_8UC1: 255 where count_ >= N
};

}  // namespace sonar_processing
