#pragma once

#include <memory>
#include <string>

#include <opencv2/core.hpp>

#include "sonar_processing/core/gaussian_filter.hpp"
#include "sonar_processing/debug_image_publisher.hpp"
#include "sonar_processing/stage_base.hpp"

namespace sonar_processing::stage_plugins
{

// Image stage: Gaussian smoothing of the intensity image (intensity -> intensity).
// Params under "<id>.": kernel_size, sigma_x, sigma_y, debug.
// Debug (when debug: true): debug/<id> = smoothed image.
class Gaussian : public StageBase
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & node,
    const std::shared_ptr<tf2_ros::Buffer> & tf_buffer,
    const rclcpp::Logger & logger,
    const std::string & stage_id) override;

  bool process(sensor_msgs::msg::Image & image) override;

private:
  core::GaussianFilterConfig config_;
  cv::Mat out_;
  DebugImagePublisher debug_out_;
};

}  // namespace sonar_processing::stage_plugins
