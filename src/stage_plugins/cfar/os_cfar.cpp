#include "sonar_processing/stage_plugins/cfar/os_cfar.hpp"

#include <pluginlib/class_list_macros.hpp>

#include "sonar_processing/image_utils.hpp"
#include "sonar_processing/param_utils.hpp"

namespace sonar_processing::stage_plugins
{

void OsCfar::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & node,
  const std::shared_ptr<tf2_ros::Buffer> &, const rclcpp::Logger & logger,
  const std::string & id)
{
  auto n = lock_node(node, id);
  // OsCfarConfig uses image axes: y = rows = range, x = columns = beam.
  core::OsCfarConfig c;
  c.guard_cells_y = declare_and_get<int>(n, id + ".guard_cells_range");
  c.guard_cells_x = declare_and_get<int>(n, id + ".guard_cells_beam");
  c.training_cells_y = declare_and_get<int>(n, id + ".training_cells_range");
  c.training_cells_x = declare_and_get<int>(n, id + ".training_cells_beam");
  c.kth_order_statistic = declare_and_get<int>(n, id + ".kth_order_statistic");
  c.false_alarm_rate = declare_and_get<double>(n, id + ".false_alarm_rate");
  c.min_intensity = declare_and_get<double>(n, id + ".min_intensity");
  cfar_.configure(c);
  const bool debug = DebugImagePublisher::enabled(n, id);
  debug_out_.configure(n, debug, id);
  debug_threshold_.configure(n, debug, id, "threshold");

  RCLCPP_INFO(logger,
    "[%s] guard=(%d, %d) training=(%d, %d) (range, beam) k=%d pfa=%g min_intensity=%.2f",
    id.c_str(), c.guard_cells_y, c.guard_cells_x, c.training_cells_y, c.training_cells_x,
    c.kth_order_statistic, c.false_alarm_rate, c.min_intensity);
}

bool OsCfar::process(sensor_msgs::msg::Image & image)
{
  cv::Mat img = wrap_mono8(image);
  // The threshold map is only computed while someone watches it.
  cv::Mat * threshold = debug_threshold_.active() ? &threshold_ : nullptr;
  cfar_.detect(img, img, threshold);   // OS-CFAR reads only its padded copy, so in place is safe
  debug_out_.publish(image);
  if (threshold) {
    debug_threshold_.publish(threshold_, image.header);
  }
  return true;
}

}  // namespace sonar_processing::stage_plugins

PLUGINLIB_EXPORT_CLASS(sonar_processing::stage_plugins::OsCfar, sonar_processing::StageBase)
