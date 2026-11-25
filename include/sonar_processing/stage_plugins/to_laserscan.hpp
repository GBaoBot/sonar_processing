#pragma once

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "sonar_processing/core/scan_builder.hpp"
#include "sonar_processing/stage_base.hpp"

namespace sonar_processing::stage_plugins
{

// Output stage: publishes the first return per beam as a sensor_msgs/LaserScan and passes
// the mask through unchanged, so it can sit anywhere in the pipeline.
// Params under "<id>.": topic, sonar_range_m, min_range_m, obstacle_max_range_m, fov_deg,
//   scan_angle_increment_deg, uniform_tol_deg, empty_beam_value ("inf" or "range_max"),
//   beam_bearings_deg (optional; unset = uniform fan of fov_deg).
class ToLaserScan : public StageBase
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & node,
    const std::shared_ptr<tf2_ros::Buffer> & tf_buffer,
    const rclcpp::Logger & logger,
    const std::string & stage_id) override;

  bool process(sensor_msgs::msg::Image & image) override;

  // Forgets the previous stamp used for scan_time.
  void reset() override;

private:
  core::ScanBuilder builder_;
  std::vector<int> return_rows_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rcl_clock_type_t clock_type_{RCL_ROS_TIME};
  std::optional<rclcpp::Time> last_stamp_;   // for scan_time
};

}  // namespace sonar_processing::stage_plugins
