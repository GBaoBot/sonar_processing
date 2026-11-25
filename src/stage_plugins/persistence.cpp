#include "sonar_processing/stage_plugins/persistence.hpp"

#include <pluginlib/class_list_macros.hpp>

#include "sonar_processing/image_utils.hpp"
#include "sonar_processing/param_utils.hpp"

namespace sonar_processing::stage_plugins
{

void Persistence::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & node,
  const std::shared_ptr<tf2_ros::Buffer> &, const rclcpp::Logger & logger,
  const std::string & id)
{
  auto n = lock_node(node, id);
  core::PersistenceConfig c;
  c.persist_n = declare_and_get<int>(n, id + ".persist_n");
  c.persist_m = declare_and_get<int>(n, id + ".persist_m");
  c.persist_tol_range = declare_and_get<int>(n, id + ".persist_tol_range");
  c.persist_tol_beam = declare_and_get<int>(n, id + ".persist_tol_beam");
  filter_.configure(c);
  persist_m_ = c.persist_m;
  const bool debug = DebugImagePublisher::enabled(n, id);
  debug_out_.configure(n, debug, id);
  debug_hit_count_.configure(n, debug, id, "hit_count");

  RCLCPP_INFO(logger, "[%s] %d of %d pings, tol=(%d, %d) (range, beam)",
    id.c_str(), c.persist_n, c.persist_m, c.persist_tol_range, c.persist_tol_beam);
}

bool Persistence::process(sensor_msgs::msg::Image & image)
{
  cv::Mat img = wrap_mono8(image);
  filter_.apply(img, img);   // dst may alias the mask
  debug_out_.publish(image);
  if (debug_hit_count_.active()) {
    filter_.hit_count().convertTo(hit_count_scaled_, CV_8U, 255.0 / persist_m_);
    debug_hit_count_.publish(hit_count_scaled_, image.header);
  }
  return true;
}

void Persistence::reset()
{
  filter_.reset();
}

}  // namespace sonar_processing::stage_plugins

PLUGINLIB_EXPORT_CLASS(sonar_processing::stage_plugins::Persistence, sonar_processing::StageBase)
