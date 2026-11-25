#pragma once

#include <vector>

#include <opencv2/core.hpp>

namespace sonar_processing::core
{

// All fields must be set by the caller; zero N/M fail validate().
struct PersistenceConfig
{
  int persist_n{0};          // detections needed ...
  int persist_m{0};          // ... within the last M pings (current ping included), N <= M
  int persist_tol_range{0};  // tolerance dilation half-size along range, in cells (>= 0)
  int persist_tol_beam{0};   // tolerance dilation half-size along beams, in cells (>= 0)
};

// M-of-N style temporal filter in polar index space (stands in for a log-odds grid).
//
// A cell detected in the current ping is kept only if a detection fell within
// +/- (persist_tol_range, persist_tol_beam) cells of it in at least persist_n of the last
// persist_m pings. Each ping's mask is dilated by the tolerance window, stored in a ring of M
// slots, and a running per-cell count of the stored masks is updated incrementally (add the new
// slot, subtract the evicted one).
//
// NOTE: The dilation with tolerance range and beam is for compensating vehicle's motion. 
// The tolerance only absorbs small motion; fast motion causes missed detections (never phantom
// ones, because the current ping must still detect the cell). Ego-motion compensation would
// require projecting the history into a world-fixed frame.
class PersistenceFilter
{
public:
  static void validate(const PersistenceConfig & config);
  void configure(const PersistenceConfig & config);

  void reset();

  // Steps in apply():
  //   1. Remove the count from oldest ping mask
  //   2. Replace oldest ping with latest one (which is dilated if tolerance is enabled)
  //   3. Add latest ping to count
  //   4. Compare count with threshold N, apply result in place of input mask
  //   5. Update head of history and repeat from step 1
  // Note that when the history is not filled, the step 1 is not executed.
  void apply(const cv::Mat & mask, cv::Mat & dst);

  // For debugging, valid after apply(): CV_8UC1, per cell the number of stored pings (out of
  // the last M) with a detection within the tolerance window.
  const cv::Mat & hit_count() const {return count_;}

private:
  PersistenceConfig config_;
  cv::Mat kernel_;
  std::vector<cv::Mat> history_;  // ring of M dilated masks, values 0/1
  int head_{0};                   // slot the next ping is written to
  int filled_{0};                 // number of valid slots, <= M
  cv::Mat count_;                 // CV_8UC1: sum of the valid history slots per cell
  cv::Mat persistent_;            // CV_8UC1: 255 where count_ >= N
};

}  // namespace sonar_processing::core
