#pragma once

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>

#include <sensor_msgs/msg/image.hpp>

#include "sonar_processing/ca_cfar.hpp"
#include "sonar_processing/gaussian_filter.hpp"

namespace sonar_processing
{

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
  void imageCallback(Image::UniquePtr msg);
  void teardown();
  static Image::UniquePtr makeImage(
    const std_msgs::msg::Header & header, uint32_t height, uint32_t width,
    const std::string & encoding, size_t elem_size);

  // Stored parameters (declared in constructor, read in on_configure)
  std::string input_topic_;
  std::string output_topic_;
  std::string cfar_topic_;
  bool gaussian_enabled_{true};
  bool cfar_enabled_{true};

  // ROS2
  rclcpp::Subscription<Image>::SharedPtr sonar_image_sub_;
  rclcpp_lifecycle::LifecyclePublisher<Image>::SharedPtr processed_image_pub_;
  rclcpp_lifecycle::LifecyclePublisher<Image>::SharedPtr cfar_mask_pub_;

  // Gaussian Filter
  GaussianFilterConfig gaussian_filter_config_;

  // CA-CFAR detector (runs on the filtered image)
  CaCfarConfig cfar_config_;
  CaCfar cfar_;
};

}  // namespace sonar_processing
