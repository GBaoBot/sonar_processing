#pragma once

#include <cstring>
#include <memory>
#include <string>

#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

#include "sonar_processing/param_utils.hpp"

namespace sonar_processing
{

// Optional mono8 debug image owned by a stage. A stage reads its "<id>.debug" flag once with
// enabled() and configures one DebugImagePublisher per image it offers. When the flag is false
// no publisher is created and every call is a no-op. Publishing only happens while something
// subscribes, so a stage can leave debug on and pay nothing until someone looks.
//
// Topics are relative to the node: "debug/<id>" for the stage output, "debug/<id>/<name>" for
// extra images (e.g. a CFAR threshold map). Created on the lifecycle node, so the node's
// on_activate/on_deactivate (de)activate them, and released with the stage on cleanup.
class DebugImagePublisher
{
public:
  using Image = sensor_msgs::msg::Image;

  // Reads the bool param "<stage_id>.debug" (required, like every stage param).
  static bool enabled(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & node, const std::string & stage_id)
  {
    return declare_and_get<bool>(node, stage_id + ".debug");
  }

  void configure(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & node, bool enabled,
    const std::string & stage_id, const std::string & name = "")
  {
    pub_.reset();
    if (!enabled) {return;}
    const std::string topic = "debug/" + stage_id + (name.empty() ? "" : "/" + name);
    pub_ = node->create_publisher<Image>(topic, rclcpp::SensorDataQoS());
  }

  // True when enabled, active and subscribed. Check it before building a costly debug image.
  bool active() const
  {
    return pub_ && pub_->is_activated() && pub_->get_subscription_count() > 0;
  }

  // Publishes a copy of `image` (later stages keep modifying it in place).
  void publish(const Image & image) const
  {
    if (!active()) {return;}
    pub_->publish(std::make_unique<Image>(image));
  }

  // Publishes a copy of a CV_8UC1 Mat with `header`.
  void publish(const cv::Mat & mono8, const std_msgs::msg::Header & header) const
  {
    if (!active() || mono8.type() != CV_8UC1) {return;}
    auto msg = std::make_unique<Image>();
    msg->header = header;
    msg->height = static_cast<uint32_t>(mono8.rows);
    msg->width = static_cast<uint32_t>(mono8.cols);
    msg->encoding = sensor_msgs::image_encodings::MONO8;
    msg->is_bigendian = 0;
    msg->step = static_cast<uint32_t>(mono8.cols);
    msg->data.resize(static_cast<size_t>(mono8.rows) * mono8.cols);
    for (int r = 0; r < mono8.rows; ++r) {
      std::memcpy(msg->data.data() + static_cast<size_t>(r) * mono8.cols, mono8.ptr(r),
        static_cast<size_t>(mono8.cols));
    }
    pub_->publish(std::move(msg));
  }

  const char * topic() const {return pub_ ? pub_->get_topic_name() : "off";}

private:
  rclcpp_lifecycle::LifecyclePublisher<Image>::SharedPtr pub_;
};

}  // namespace sonar_processing
