#pragma once

#include <optional>
#include <utility>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "sonar_processing/gaussian_filter.hpp"
#include "sonar_processing/morphology_filter.hpp"
#include "sonar_processing/os_cfar.hpp"
#include "sonar_processing/persistence_filter.hpp"
#include "sonar_processing/scan_builder.hpp"

namespace sonar_processing
{

// Pipeline per ping (mono8 polar image, rows = range, cols = beams):
//   Gaussian smoothing -> OS-CFAR -> morphology -> persistence -> first return per beam
//   -> sensor_msgs/LaserScan (uniform angular grid) in the sonar frame.
class SonarProcessingNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  using Image = sensor_msgs::msg::Image;
  using LaserScan = sensor_msgs::msg::LaserScan;

  explicit SonarProcessingNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  // One toggleable mono8 polar debug image.
  struct DebugOutput
  {
    explicit DebugOutput(std::string n)
    : name(std::move(n)) {}

    std::string name;   // parameter prefix: debug.<name>.{enabled,topic}
    bool enabled{false};
    std::string topic;
    rclcpp_lifecycle::LifecyclePublisher<Image>::SharedPtr pub;
  };

  void imageCallback(Image::UniquePtr msg);
  void teardown();
  void declareDebugOutput(const std::string & name, bool enabled, const std::string & topic);
  // A CV_8UC1 buffer for one pipeline stage. When the stage's debug output is on, the buffer
  // wraps a new message (returned in msg) so publishing it needs no copy; otherwise it is the
  // reusable scratch Mat.
  static cv::Mat stageBuffer(
    const DebugOutput & out, cv::Mat & scratch, const std_msgs::msg::Header & header,
    int rows, int cols, Image::UniquePtr & msg);
  static Image::UniquePtr makeImage(
    const std_msgs::msg::Header & header, uint32_t height, uint32_t width,
    const std::string & encoding, size_t elem_size);

  // Stored parameters (declared in constructor, read in on_configure)
  std::string input_topic_;
  std::string scan_topic_;
  bool gaussian_enabled_{true};

  // Debug images (mono8, polar)
  DebugOutput debug_smoothed_{"smoothed"};
  DebugOutput debug_cfar_{"cfar_mask"};
  DebugOutput debug_morphology_{"morphology"};
  DebugOutput debug_persistence_{"persistence"};
  DebugOutput debug_returns_{"returns"};

  // ROS2
  rclcpp::Subscription<Image>::SharedPtr sonar_image_sub_;
  rclcpp_lifecycle::LifecyclePublisher<LaserScan>::SharedPtr scan_pub_;
  std::optional<rclcpp::Time> last_ping_stamp_;   // for LaserScan::scan_time

  // Pipeline stages
  GaussianFilterConfig gaussian_filter_config_;
  OsCfarConfig cfar_config_;
  OsCfar cfar_;
  MorphologyConfig morphology_config_;
  MorphologyFilter morphology_;
  PersistenceConfig persistence_config_;
  PersistenceFilter persistence_;
  ScanConfig scan_config_;
  ScanBuilder scan_builder_;

  // Scratch buffers for stages whose debug output is off, reused between pings
  cv::Mat smoothed_buf_, cfar_buf_, morphology_buf_, persistence_buf_;
  std::vector<int> return_rows_;
};

}  // namespace sonar_processing
