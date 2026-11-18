#include "sonar_processing/persistence_filter.hpp"

#include <algorithm>
#include <stdexcept>

#include <opencv2/imgproc.hpp>

namespace sonar_processing
{

void PersistenceFilter::validate(const PersistenceConfig & config)
{
  if (config.persist_m < 1 || config.persist_m > 255) {
    throw std::invalid_argument("Sonar Processing: persist_m must be in [1, 255]");
  }
  if (config.persist_n < 1 || config.persist_n > config.persist_m) {
    throw std::invalid_argument("Sonar Processing: persist_n must be in [1, persist_m]");
  }
  if (config.persist_tol_range < 0 || config.persist_tol_beam < 0) {
    throw std::invalid_argument("Sonar Processing: persistence tolerances must be >= 0");
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
    throw std::invalid_argument("Sonar Processing: persistence expects a CV_8UC1 mask");
  }
  if (history_.empty()) {
    throw std::logic_error("Sonar Processing: PersistenceFilter used before configure()");
  }

  if (count_.size() != mask.size()) {
    reset();
    count_ = cv::Mat::zeros(mask.size(), CV_8UC1);
  }

  cv::Mat & slot = history_[head_];
  if (filled_ == config_.persist_m) {
    cv::subtract(count_, slot, count_);   // evict the oldest ping
  }
  if (config_.persist_tol_range > 0 || config_.persist_tol_beam > 0) {
    cv::dilate(mask, slot, kernel_);
  } else {
    mask.copyTo(slot);
  }
  cv::min(slot, 1, slot);                 // 0/255 -> 0/1
  cv::add(count_, slot, count_);

  head_ = (head_ + 1) % config_.persist_m;
  filled_ = std::min(filled_ + 1, config_.persist_m);

  // Keep current detections whose neighbourhood was hit in >= N of the stored pings. During the
  // first N - 1 pings after a reset nothing passes.
  cv::compare(count_, config_.persist_n, persistent_, cv::CMP_GE);
  cv::bitwise_and(mask, persistent_, dst);
}

}  // namespace sonar_processing
