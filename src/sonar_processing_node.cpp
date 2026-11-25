#include "sonar_processing/sonar_processing_node.hpp"

#include <memory>
#include <set>
#include <sstream>
#include <stdexcept>
#include <utility>

#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <tf2_ros/create_timer_ros.h>

#include "sonar_processing/param_utils.hpp"

namespace sonar_processing
{

namespace
{
constexpr auto kTimingLogPeriod = std::chrono::seconds(10);

double ms_since(const std::chrono::steady_clock::time_point & t0)
{
  return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
}
}  // namespace

SonarProcessingNode::SonarProcessingNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("sonar_processing", options),
  stage_loader_("sonar_processing", "sonar_processing::StageBase")
{
}

SonarProcessingNode::CallbackReturn SonarProcessingNode::on_configure(
  const rclcpp_lifecycle::State &)
{
  try {
    auto node = shared_from_this();
    input_topic_ = declare_and_get<std::string>(node, "input_topic");

    // One TF buffer and listener for the whole node, shared with every stage.
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_buffer_->setCreateTimerInterface(std::make_shared<tf2_ros::CreateTimerROS>(
        get_node_base_interface(), get_node_timers_interface()));
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node);

    load_stages();
  } catch (const std::exception & e) {
    RCLCPP_FATAL(get_logger(), "Configuration failed: %s", e.what());
    teardown();
    return CallbackReturn::FAILURE;
  }

  RCLCPP_INFO(get_logger(), "Configured: %s, %zu stages",
    input_topic_.c_str(), stages_.size());
  return CallbackReturn::SUCCESS;
}

void SonarProcessingNode::load_stages()
{
  auto node = shared_from_this();
  std::vector<std::string> ids = declare_and_get<std::vector<std::string>>(node, "pipeline");
  if (ids.empty()) {
    throw std::runtime_error("'pipeline' is empty");
  }
  std::set<std::string> seen;
  for (const auto & id : ids) {
    if (!seen.insert(id).second) {
      throw std::runtime_error("stage '" + id + "' is listed twice in 'pipeline'");
    }
  }

  // On a throw, on_configure's teardown() drops the stages loaded so far.
  for (size_t i = 0; i < ids.size(); ++i) {
    const std::string & id = ids[i];
    const std::string type = get_plugin_type(node, id);
    if (!stage_loader_.isClassAvailable(type)) {
      throw std::runtime_error("stage '" + id + "': unknown plugin class '" + type + "'");
    }
    try {
      auto stage = stage_loader_.createSharedInstance(type);
      stage->configure(node, tf_buffer_, get_logger(), id);
      stages_.push_back(std::move(stage));
    } catch (const std::exception & e) {
      throw std::runtime_error("stage '" + id + "' (" + type + "): " + e.what());
    }
    RCLCPP_INFO(get_logger(), "Pipeline stage %zu: '%s' (%s)", i, id.c_str(), type.c_str());
  }

  stage_ids_ = std::move(ids);
}

SonarProcessingNode::CallbackReturn SonarProcessingNode::on_activate(
  const rclcpp_lifecycle::State & state)
{
  if (stages_.empty()) {
    RCLCPP_ERROR(get_logger(), "Cannot activate before the pipeline is configured.");
    return CallbackReturn::FAILURE;
  }

  LifecycleNode::on_activate(state);

  // History from before a pause no longer matches the scene.
  for (auto & stage : stages_) {
    stage->reset();
  }
  ping_times_ms_.assign(stages_.size(), 0.0);
  time_sums_ms_.assign(stages_.size(), 0.0);
  timed_pings_ = 0;
  last_timing_log_ = std::chrono::steady_clock::now();

  sonar_image_sub_ = create_subscription<Image>(
    input_topic_, rclcpp::SensorDataQoS(),
    [this](Image::UniquePtr msg) {image_callback(std::move(msg));});

  RCLCPP_INFO(get_logger(), "Activated: processing started.");
  return CallbackReturn::SUCCESS;
}

SonarProcessingNode::CallbackReturn SonarProcessingNode::on_deactivate(
  const rclcpp_lifecycle::State & state)
{
  sonar_image_sub_.reset();

  // This will automatically deactivate all lifecycle publishers
  LifecycleNode::on_deactivate(state);

  RCLCPP_INFO(get_logger(), "Deactivated: processing paused.");
  return CallbackReturn::SUCCESS;
}

SonarProcessingNode::CallbackReturn SonarProcessingNode::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  teardown();
  RCLCPP_INFO(get_logger(), "Cleaned up.");
  return CallbackReturn::SUCCESS;
}

SonarProcessingNode::CallbackReturn SonarProcessingNode::on_shutdown(
  const rclcpp_lifecycle::State & state)
{
  teardown();
  RCLCPP_INFO(get_logger(), "Shut down from state: %s", state.label().c_str());
  return CallbackReturn::SUCCESS;
}

void SonarProcessingNode::teardown()
{
  sonar_image_sub_.reset();
  stages_.clear();        // stages may hold the TF buffer: release them first
  stage_ids_.clear();
  tf_listener_.reset();
  tf_buffer_.reset();
}

bool SonarProcessingNode::run_stages(Image & image)
{
  for (size_t i = 0; i < stages_.size(); ++i) {
    const auto t0 = std::chrono::steady_clock::now();
    const bool success = stages_[i]->process(image);
    ping_times_ms_[i] = ms_since(t0);
    if (!success) {
      return false;
    }
  }
  return true;
}

void SonarProcessingNode::image_callback(Image::UniquePtr msg)
{
  if (msg->encoding != sensor_msgs::image_encodings::MONO8) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
      "Unsupported encoding '%s', the detection pipeline needs mono8", msg->encoding.c_str());
    return;
  }
  if (msg->height == 0 || msg->width == 0) {return;}

  // The image is processed in place: the message is ours (UniquePtr), no copy needed.
  // Output stages (e.g. ToLaserScan) publish from inside their process().
  try {
    if (!run_stages(*msg)) {return;}
  } catch (const std::exception & e) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Dropping ping: %s", e.what());
    return;
  }

  // Only completed pings count, so the means are not skewed by partial runs.
  for (size_t i = 0; i < ping_times_ms_.size(); ++i) {
    time_sums_ms_[i] += ping_times_ms_[i];
  }
  ++timed_pings_;
  log_timing();
}

void SonarProcessingNode::log_timing()
{
  const auto now = std::chrono::steady_clock::now();
  if (now - last_timing_log_ < kTimingLogPeriod || timed_pings_ == 0) {
    return;
  }

  std::ostringstream line;
  line.precision(2);
  line << std::fixed << "Mean stage time over " << timed_pings_ << " pings (ms):";
  double total = 0.0;
  for (size_t i = 0; i < stage_ids_.size(); ++i) {
    const double mean = time_sums_ms_[i] / timed_pings_;
    total += mean;
    line << " " << stage_ids_[i] << "=" << mean;
  }
  line << " | total=" << total;
  RCLCPP_INFO(get_logger(), "%s", line.str().c_str());

  time_sums_ms_.assign(stage_ids_.size(), 0.0);
  timed_pings_ = 0;
  last_timing_log_ = now;
}

}  // namespace sonar_processing

RCLCPP_COMPONENTS_REGISTER_NODE(sonar_processing::SonarProcessingNode)
