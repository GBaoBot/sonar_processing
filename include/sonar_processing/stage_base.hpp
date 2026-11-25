#pragma once

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_ros/buffer.h>

namespace sonar_processing
{

// A pipeline stage that turns a mono8 polar image into another mono8 polar image of the same
// size (rows = range, row 0 nearest; columns = beams, column 0 = port). Before CFAR the image
// holds intensity; from CFAR on it holds the detection mask (255 = detected, 0 = not).
class StageBase
{
public:
  using Ptr = std::shared_ptr<StageBase>;
  virtual ~StageBase() = default;

  // Declare and read params under "<stage_id>." (typed, no defaults).
  // tf_buffer and logger come from the parent node: keep them, never create a TransformListener.
  virtual void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & node,
    const std::shared_ptr<tf2_ros::Buffer> & tf_buffer,
    const rclcpp::Logger & logger,
    const std::string & stage_id) = 0;

  // Modify the mono8 image in place. Size, encoding and header must not change.
  // Return false to drop this ping (nothing is published).
  virtual bool process(sensor_msgs::msg::Image & image) = 0;

  // Clear any history kept between pings.
  virtual void reset() {}
};

}  // namespace sonar_processing
