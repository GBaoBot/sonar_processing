#include "sonar_processing/stage_plugins/gaussian.hpp"

#include <pluginlib/class_list_macros.hpp>

#include "sonar_processing/image_utils.hpp"
#include "sonar_processing/param_utils.hpp"

namespace sonar_processing::stage_plugins
{

void Gaussian::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & node,
  const std::shared_ptr<tf2_ros::Buffer> &, const rclcpp::Logger & logger,
  const std::string & id)
{
  auto n = lock_node(node, id);
  config_.kernel_size = declare_and_get<int>(n, id + ".kernel_size");
  config_.sigma_x = declare_and_get<double>(n, id + ".sigma_x");
  config_.sigma_y = declare_and_get<double>(n, id + ".sigma_y");
  core::GaussianFilter::validate(config_);
  debug_out_.configure(n, DebugImagePublisher::enabled(n, id), id);

  RCLCPP_INFO(logger, "[%s] kernel=%d sigma=(%.2f, %.2f)",
    id.c_str(), config_.kernel_size, config_.sigma_x, config_.sigma_y);
}

bool Gaussian::process(sensor_msgs::msg::Image & image)
{
  cv::Mat img = wrap_mono8(image);
  core::GaussianFilter::perform(img, out_, config_);
  out_.copyTo(img);
  debug_out_.publish(image);
  return true;
}

}  // namespace sonar_processing::stage_plugins

PLUGINLIB_EXPORT_CLASS(sonar_processing::stage_plugins::Gaussian, sonar_processing::StageBase)
