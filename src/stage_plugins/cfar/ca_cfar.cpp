#include "sonar_processing/stage_plugins/cfar/ca_cfar.hpp"

#include <pluginlib/class_list_macros.hpp>

#include "sonar_processing/image_utils.hpp"
#include "sonar_processing/param_utils.hpp"

namespace sonar_processing::stage_plugins
{

void CaCfar::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & node,
  const std::shared_ptr<tf2_ros::Buffer> &, const rclcpp::Logger & logger,
  const std::string & id)
{
  auto n = lock_node(node, id);
  core::CaCfarConfig c;
  c.guard_cells_range = declare_and_get<int>(n, id + ".guard_cells_range");
  c.guard_cells_beam = declare_and_get<int>(n, id + ".guard_cells_beam");
  c.training_cells_range = declare_and_get<int>(n, id + ".training_cells_range");
  c.training_cells_beam = declare_and_get<int>(n, id + ".training_cells_beam");
  c.false_alarm_rate = declare_and_get<double>(n, id + ".false_alarm_rate");
  c.min_intensity = declare_and_get<double>(n, id + ".min_intensity");
  cfar_.configure(c);
  const bool debug = DebugImagePublisher::enabled(n, id);
  debug_out_.configure(n, debug, id);
  debug_threshold_.configure(n, debug, id, "threshold");

  RCLCPP_INFO(logger,
    "[%s] guard=(%d, %d) training=(%d, %d) (range, beam) pfa=%g min_intensity=%.2f",
    id.c_str(), c.guard_cells_range, c.guard_cells_beam, c.training_cells_range,
    c.training_cells_beam, c.false_alarm_rate, c.min_intensity);
}

bool CaCfar::process(sensor_msgs::msg::Image & image)
{
  cv::Mat img = wrap_mono8(image);
  // The threshold map is only computed while someone watches it.
  cv::Mat * threshold = debug_threshold_.active() ? &threshold_ : nullptr;
  cfar_.detect(img, out_, threshold);
  out_.copyTo(img);
  debug_out_.publish(image);
  if (threshold) {
    debug_threshold_.publish(threshold_, image.header);
  }
  return true;
}

}  // namespace sonar_processing::stage_plugins

PLUGINLIB_EXPORT_CLASS(sonar_processing::stage_plugins::CaCfar, sonar_processing::StageBase)
