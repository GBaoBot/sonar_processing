#pragma once

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "sonar_processing/stage_base.hpp"

namespace sonar_processing
{

// mono8 polar sonar image -> ordered stage plugins (from the "pipeline" param). The node
// subscribes, owns TF, loads and runs the stages and logs timing; all processing, the outputs
// (e.g. the LaserScan from ToLaserScan) and the debug images live in the stage plugins.
class SonarProcessingNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  using Image = sensor_msgs::msg::Image;

  explicit SonarProcessingNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  // Reads the string-array param "pipeline", loads only those ids and configures each one.
  // Throws std::runtime_error naming the stage id on any problem.
  void load_stages();
  // Runs the stages in order on `image` (in place), filling ping_times_ms_. Returns false if a
  // stage dropped the ping. Exceptions propagate.
  bool run_stages(Image & image);

  void image_callback(Image::UniquePtr msg);
  void log_timing();
  void teardown();

  std::string input_topic_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Loader before the stages, so it outlives the instances it created.
  pluginlib::ClassLoader<StageBase> stage_loader_;
  std::vector<std::string> stage_ids_;
  std::vector<StageBase::Ptr> stages_;   // same order as stage_ids_

  rclcpp::Subscription<Image>::SharedPtr sonar_image_sub_;

  // Per-stage timing: this ping's times, and the sums over completed pings since the last log.
  std::vector<double> ping_times_ms_;
  std::vector<double> time_sums_ms_;
  int timed_pings_{0};
  std::chrono::steady_clock::time_point last_timing_log_;
};

}  // namespace sonar_processing
