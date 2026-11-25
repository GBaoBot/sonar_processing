#pragma once

#include <memory>
#include <string>

#include <opencv2/core.hpp>

#include "sonar_processing/core/persistence_filter.hpp"
#include "sonar_processing/debug_image_publisher.hpp"
#include "sonar_processing/stage_base.hpp"

namespace sonar_processing::stage_plugins
{

// Keep a detection only if seen in >= N of the last M pings (mask -> mask).
// Debug (when debug: true): debug/<id> = filtered mask; debug/<id>/hit_count = per-cell hits in
// the last M pings, scaled to 0..255 (255 = hit in all M).
class Persistence : public StageBase
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & node,
    const std::shared_ptr<tf2_ros::Buffer> & tf_buffer,
    const rclcpp::Logger & logger,
    const std::string & stage_id) override;

  bool process(sensor_msgs::msg::Image & image) override;

  // Drops the ping history.
  void reset() override;

private:
  core::PersistenceFilter filter_;
  int persist_m_{1};
  cv::Mat hit_count_scaled_;
  DebugImagePublisher debug_out_;
  DebugImagePublisher debug_hit_count_;
};

}  // namespace sonar_processing::stage_plugins
