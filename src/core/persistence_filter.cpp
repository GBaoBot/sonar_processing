#include "sonar_processing/core/persistence_filter.hpp"

#include <algorithm>
#include <stdexcept>

#include <opencv2/imgproc.hpp>

namespace sonar_processing::core
{

void PersistenceFilter::validate(const PersistenceConfig & config)
{
  if (config.persist_m < 1 || config.persist_m > 255) {
    throw std::invalid_argument("persist_m must be in [1, 255]");
  }
  if (config.persist_n < 1 || config.persist_n > config.persist_m) {
    throw std::invalid_argument("persist_n must be in [1, persist_m]");
  }
  if (config.persist_tol_range < 0 || config.persist_tol_beam < 0) {
    throw std::invalid_argument("persistence tolerances must be >= 0");
  }
}

void PersistenceFilter::configure(const PersistenceConfig & config)
{
  validate(config);
  config_ = config;
  kernel_ = cv::getStructuringElement(
    cv::MORPH_RECT,
    cv::Size(2 * config.persist_tol_beam + 1, 2 * config.persist_tol_range + 1));
  history_.assign(static_cast<size_t>(config.persist_m), cv::Mat());
  reset();
}

void PersistenceFilter::reset()
{
  head_ = 0;
  filled_ = 0;
  count_.release();   // reallocated (zeroed) on the next apply()
}

void PersistenceFilter::apply(const cv::Mat & mask, cv::Mat & dst)
{
  if (mask.type() != CV_8UC1) {
    throw std::invalid_argument("persistence expects a CV_8UC1 mask");
  }
  if (history_.empty()) {
    throw std::logic_error("PersistenceFilter used before configure()");
  }

  if (count_.size() != mask.size()) {
    reset();
    count_ = cv::Mat::zeros(mask.size(), CV_8UC1);
  }

  cv::Mat & slot = history_[head_];

  // Remove the oldest ping
  if (filled_ == config_.persist_m) {
    cv::subtract(count_, slot, count_);
  }

  // Replace oldest ping with latest ping
  if (config_.persist_tol_range > 0 || config_.persist_tol_beam > 0) {
    cv::dilate(mask, slot, kernel_);
  } else {
    mask.copyTo(slot);
  }
  cv::min(slot, 1, slot);                 // 0/255 -> 0/1
  cv::add(count_, slot, count_);

  // Keep current detections whose neighbourhood was hit in >= N of the stored pings. 
  // During the first N - 1 pings after a reset nothing passes.
  cv::compare(count_, config_.persist_n, persistent_, cv::CMP_GE);
  cv::bitwise_and(mask, persistent_, dst);

  // Move head pointer, update filled_ if the queue is not filled
  head_ = (head_ + 1) % config_.persist_m;
  filled_ = std::min(filled_ + 1, config_.persist_m);
}

}  // namespace sonar_processing::core
