#include "sonar_processing/stage_plugins/to_laserscan.hpp"

#include <stdexcept>
#include <utility>

#include <pluginlib/class_list_macros.hpp>

#include "sonar_processing/image_utils.hpp"
#include "sonar_processing/param_utils.hpp"

namespace sonar_processing::stage_plugins
{

void ToLaserScan::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & node,
  const std::shared_ptr<tf2_ros::Buffer> &, const rclcpp::Logger & logger,
  const std::string & id)
{
  auto n = lock_node(node, id);
  clock_type_ = n->get_clock()->get_clock_type();
  const std::string topic = declare_and_get<std::string>(n, id + ".topic");

  core::ScanConfig c;
  c.sonar_range_m = declare_and_get<double>(n, id + ".sonar_range_m");
  c.min_range_m = declare_and_get<double>(n, id + ".min_range_m");
  c.obstacle_max_range_m = declare_and_get<double>(n, id + ".obstacle_max_range_m");
  // Empty = uniform fan of fov_deg. The only stage param with a default: an empty array
  // cannot be written reliably in a ROS 2 params file, so "unset" means "no table".
  c.beam_bearings_deg = declare_and_get_or<std::vector<double>>(
    n, id + ".beam_bearings_deg", std::vector<double>{});
  c.fov_deg = declare_and_get<double>(n, id + ".fov_deg");
  c.scan_angle_increment_deg = declare_and_get<double>(n, id + ".scan_angle_increment_deg");
  c.uniform_tol_deg = declare_and_get<double>(n, id + ".uniform_tol_deg");
  const std::string empty_beam = declare_and_get<std::string>(n, id + ".empty_beam_value");
  if (empty_beam != "inf" && empty_beam != "range_max") {
    throw std::invalid_argument(
            "empty_beam_value must be \"inf\" or \"range_max\", got \"" + empty_beam + "\"");
  }
  c.empty_beam_inf = (empty_beam == "inf");
  builder_.configure(c);
  last_stamp_.reset();

  // Created on the lifecycle node, so the node's on_activate/on_deactivate (de)activate it.
  // Released with this stage on cleanup.
  scan_pub_ = n->create_publisher<sensor_msgs::msg::LaserScan>(topic, rclcpp::SensorDataQoS());

  RCLCPP_INFO(logger,
    "[%s] -> %s, sonar_range=%.2f m, returns in [%.2f, %.2f] m, bearings=%s, empty beams=%s",
    id.c_str(), scan_pub_->get_topic_name(), c.sonar_range_m, c.min_range_m,
    c.obstacle_max_range_m, c.beam_bearings_deg.empty() ? "uniform (fov)" : "table",
    empty_beam.c_str());
}

bool ToLaserScan::process(sensor_msgs::msg::Image & mask)
{
  auto scan = std::make_unique<sensor_msgs::msg::LaserScan>();
  builder_.build(wrap_mono8(mask), *scan, return_rows_);
  scan->header = mask.header;

  // scan_time = time since the previous ping, left at 0 for the first one.
  const rclcpp::Time stamp(mask.header.stamp, clock_type_);
  if (last_stamp_ && stamp > *last_stamp_) {
    scan->scan_time = static_cast<float>((stamp - *last_stamp_).seconds());
  }
  last_stamp_ = stamp;

  scan_pub_->publish(std::move(scan));
  return true;   // the mask is passed through unchanged
}

void ToLaserScan::reset()
{
  last_stamp_.reset();
}

}  // namespace sonar_processing::stage_plugins

PLUGINLIB_EXPORT_CLASS(sonar_processing::stage_plugins::ToLaserScan, sonar_processing::StageBase)
