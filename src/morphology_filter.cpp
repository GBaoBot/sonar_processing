#include "sonar_processing/morphology_filter.hpp"

#include <stdexcept>

#include <opencv2/imgproc.hpp>

namespace sonar_processing
{

void MorphologyFilter::validate(const MorphologyConfig & config)
{
  if (config.kernel_range <= 0 || config.kernel_range % 2 == 0 ||
    config.kernel_beam <= 0 || config.kernel_beam % 2 == 0)
  {
    throw std::invalid_argument(
            "Sonar Processing: morphology kernel sizes must be positive odd values");
  }
  if (config.min_blob_cells < 0) {
    throw std::invalid_argument("Sonar Processing: min_blob_cells must be >= 0");
  }
}

void MorphologyFilter::configure(const MorphologyConfig & config)
{
  validate(config);
  config_ = config;
  // cv::Size is (width, height) = (beams, range samples).
  kernel_ = cv::getStructuringElement(
    cv::MORPH_RECT, cv::Size(config.kernel_beam, config.kernel_range));
}

void MorphologyFilter::apply(const cv::Mat & mask, cv::Mat & dst)
{
  if (mask.type() != CV_8UC1) {
    throw std::invalid_argument("Sonar Processing: morphology expects a CV_8UC1 mask");
  }
  if (kernel_.empty()) {
    throw std::logic_error("Sonar Processing: MorphologyFilter used before configure()");
  }

  if (config_.kernel_range > 1 || config_.kernel_beam > 1) {
    cv::morphologyEx(mask, dst, cv::MORPH_OPEN, kernel_);
  } else {
    mask.copyTo(dst);
  }

  if (config_.min_blob_cells <= 1) {
    return;
  }

  const int n_labels = cv::connectedComponentsWithStats(
    dst, labels_, stats_, centroids_, 8, CV_32S);
  keep_.assign(static_cast<size_t>(n_labels), 0);
  bool any_dropped = false;
  for (int l = 1; l < n_labels; ++l) {   // label 0 is the background
    const bool keep = stats_.at<int>(l, cv::CC_STAT_AREA) >= config_.min_blob_cells;
    keep_[l] = keep ? 255 : 0;
    any_dropped |= !keep;
  }
  if (!any_dropped) {
    return;
  }

  for (int y = 0; y < dst.rows; ++y) {
    const int32_t * lab = labels_.ptr<int32_t>(y);
    uint8_t * out = dst.ptr<uint8_t>(y);
    for (int x = 0; x < dst.cols; ++x) {
      out[x] = keep_[lab[x]];
    }
  }
}

}  // namespace sonar_processing
