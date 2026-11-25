#pragma once

#include <memory>
#include <string>

#include <opencv2/core.hpp>

#include "sonar_processing/core/cfar/os_cfar.hpp"
#include "sonar_processing/debug_image_publisher.hpp"
#include "sonar_processing/stage_base.hpp"

namespace sonar_processing::stage_plugins
{

// Image stage: ordered-statistic CFAR detector (intensity -> mask, 255 = detected).
// Params under "<id>.": guard_cells_range, guard_cells_beam, training_cells_range,
//   training_cells_beam, kth_order_statistic, false_alarm_rate, min_intensity, debug.
// Debug (when debug: true): debug/<id> = detection mask; debug/<id>/threshold = per-cell value
// the intensity had to exceed (compare with the input image).
class OsCfar : public StageBase
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & node,
    const std::shared_ptr<tf2_ros::Buffer> & tf_buffer,
    const rclcpp::Logger & logger,
    const std::string & stage_id) override;

  bool process(sensor_msgs::msg::Image & image) override;

private:
  core::OsCfar cfar_;
  cv::Mat threshold_;
  DebugImagePublisher debug_out_;
  DebugImagePublisher debug_threshold_;
};

}  // namespace sonar_processing::stage_plugins
