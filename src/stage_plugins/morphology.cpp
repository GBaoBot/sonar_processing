#include "sonar_processing/stage_plugins/morphology.hpp"

#include <pluginlib/class_list_macros.hpp>

#include "sonar_processing/image_utils.hpp"
#include "sonar_processing/param_utils.hpp"

namespace sonar_processing::stage_plugins
{

void Morphology::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & node,
  const std::shared_ptr<tf2_ros::Buffer> &, const rclcpp::Logger & logger,
  const std::string & id)
{
  auto n = lock_node(node, id);
  core::MorphologyConfig c;
  c.kernel_range = declare_and_get<int>(n, id + ".kernel_range");
  c.kernel_beam = declare_and_get<int>(n, id + ".kernel_beam");
  filter_.configure(c);
  debug_out_.configure(n, DebugImagePublisher::enabled(n, id), id);

  RCLCPP_INFO(logger, "[%s] open kernel=(%d, %d) (range, beam)",
    id.c_str(), c.kernel_range, c.kernel_beam);
}

bool Morphology::process(sensor_msgs::msg::Image & image)
{
  cv::Mat img = wrap_mono8(image);
  filter_.apply(img, out_);   // dst must not alias the input
  out_.copyTo(img);
  debug_out_.publish(image);
  return true;
}

}  // namespace sonar_processing::stage_plugins

PLUGINLIB_EXPORT_CLASS(sonar_processing::stage_plugins::Morphology, sonar_processing::StageBase)
