#include "sonar_processing/sonar_processing_node.hpp"

#include <memory>
#include <stdexcept>
#include <utility>

#include <opencv2/core.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/image_encodings.hpp>

namespace sonar_processing
{

namespace enc = sensor_msgs::image_encodings;

SonarProcessingNode::SonarProcessingNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("sonar_processing", options)
{
  declare_parameter<std::string>("input_topic", "sonar/echo/data");
  declare_parameter<std::string>("output_topic", "sonar/echo/data/processed");
  declare_parameter<bool>("gaussian_filter.enabled", true);
  declare_parameter<int>("gaussian_filter.kernel_size", 5);
  declare_parameter<double>("gaussian_filter.sigma_x", 1.0);
  declare_parameter<double>("gaussian_filter.sigma_y", 1.0);
  declare_parameter<bool>("os_cfar.enabled", true);
  declare_parameter<std::string>("os_cfar.output_topic", "sonar/echo/data/cfar");
  declare_parameter<int>("os_cfar.guard_cells_range", 2);
  declare_parameter<int>("os_cfar.guard_cells_beam", 2);
  declare_parameter<int>("os_cfar.training_cells_range", 8);
  declare_parameter<int>("os_cfar.training_cells_beam", 8);
  // Default window (2, 2, 8, 8) has N = 21*21 - 5*5 = 416 training cells; k = 0.75 * N.
  declare_parameter<int>("os_cfar.kth_order_statistic", 312);
  declare_parameter<double>("os_cfar.false_alarm_rate", 1e-3);
  declare_parameter<double>("os_cfar.min_intensity", 0.0);
}

SonarProcessingNode::CallbackReturn SonarProcessingNode::on_configure(
  const rclcpp_lifecycle::State &)
{
  input_topic_ = get_parameter("input_topic").as_string();
  output_topic_ = get_parameter("output_topic").as_string();
  gaussian_enabled_ = get_parameter("gaussian_filter.enabled").as_bool();
  gaussian_filter_config_.kernel_size =
    static_cast<int>(get_parameter("gaussian_filter.kernel_size").as_int());
  gaussian_filter_config_.sigma_x = get_parameter("gaussian_filter.sigma_x").as_double();
  gaussian_filter_config_.sigma_y = get_parameter("gaussian_filter.sigma_y").as_double();

  cfar_enabled_ = get_parameter("os_cfar.enabled").as_bool();
  cfar_topic_ = get_parameter("os_cfar.output_topic").as_string();
  // OsCfarConfig uses image axes: y = rows = range, x = columns = beam.
  cfar_config_.guard_cells_y =
    static_cast<int>(get_parameter("os_cfar.guard_cells_range").as_int());
  cfar_config_.guard_cells_x =
    static_cast<int>(get_parameter("os_cfar.guard_cells_beam").as_int());
  cfar_config_.training_cells_y =
    static_cast<int>(get_parameter("os_cfar.training_cells_range").as_int());
  cfar_config_.training_cells_x =
    static_cast<int>(get_parameter("os_cfar.training_cells_beam").as_int());
  cfar_config_.kth_order_statistic =
    static_cast<int>(get_parameter("os_cfar.kth_order_statistic").as_int());
  cfar_config_.false_alarm_rate = get_parameter("os_cfar.false_alarm_rate").as_double();
  cfar_config_.min_intensity = get_parameter("os_cfar.min_intensity").as_double();

  try {
    GaussianFilter::validate(gaussian_filter_config_);
    if (cfar_enabled_) {
      cfar_.configure(cfar_config_);
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Configuration failed: %s", e.what());
    return CallbackReturn::FAILURE;
  }

  processed_image_pub_ = create_publisher<Image>(output_topic_, rclcpp::SensorDataQoS());
  if (cfar_enabled_) {
    cfar_mask_pub_ = create_publisher<Image>(cfar_topic_, rclcpp::SensorDataQoS());
  }

  RCLCPP_INFO(get_logger(),
    "Configured: %s -> %s, gaussian_filter enabled=%s kernel=%d sigma=(%.2f, %.2f)",
    input_topic_.c_str(), output_topic_.c_str(), gaussian_enabled_ ? "true" : "false",
    gaussian_filter_config_.kernel_size, gaussian_filter_config_.sigma_x,
    gaussian_filter_config_.sigma_y);
  if (cfar_enabled_) {
    RCLCPP_INFO(get_logger(),
      "OS-CFAR -> %s: guard=(%d, %d) training=(%d, %d) (range, beam) k=%d pfa=%g "
      "min_intensity=%.2f",
      cfar_topic_.c_str(), cfar_config_.guard_cells_y, cfar_config_.guard_cells_x,
      cfar_config_.training_cells_y, cfar_config_.training_cells_x,
      cfar_config_.kth_order_statistic, cfar_config_.false_alarm_rate,
      cfar_config_.min_intensity);
  }
  return CallbackReturn::SUCCESS;
}

SonarProcessingNode::CallbackReturn SonarProcessingNode::on_activate(
  const rclcpp_lifecycle::State & state)
{
  if (!processed_image_pub_) {
    RCLCPP_ERROR(get_logger(), "Cannot activate before the publisher is configured.");
    return CallbackReturn::FAILURE;
  }

  LifecycleNode::on_activate(state);

  sonar_image_sub_ = create_subscription<Image>(
    input_topic_, rclcpp::SensorDataQoS(),
    [this](Image::UniquePtr msg) {imageCallback(std::move(msg));});

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
  processed_image_pub_.reset();
  cfar_mask_pub_.reset();
}

void SonarProcessingNode::imageCallback(Image::UniquePtr msg)
{
  if (!processed_image_pub_ || !processed_image_pub_->is_activated()) {return;}

  int type;
  if (msg->encoding == enc::MONO8) {
    type = CV_8UC1;
  } else if (msg->encoding == enc::MONO16) {
    type = CV_16UC1;
  } else if (msg->encoding == enc::TYPE_32FC1) {
    type = CV_32FC1;
  } else {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
      "Unsupported encoding '%s', expected a single-channel image", msg->encoding.c_str());
    return;
  }

  // Wrap the message buffers directly, no copy in or out.
  const cv::Mat src(msg->height, msg->width, type, msg->data.data(), msg->step);

  // Perform Gaussian Filter if enabled
  Image::UniquePtr processed = std::move(msg);
  cv::Mat filtered = src;
  if (gaussian_enabled_) {
    auto out = makeImage(processed->header, processed->height, processed->width,
        processed->encoding, src.elemSize());
    out->is_bigendian = processed->is_bigendian;
    cv::Mat dst(out->height, out->width, type, out->data.data(), out->step);
    GaussianFilter::perform(src, dst, gaussian_filter_config_);
    filtered = dst;
    processed = std::move(out);
  }

  // Perform CFAR filter if enabled
  if (cfar_enabled_ && cfar_mask_pub_ && type != CV_8UC1) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
      "OS-CFAR supports mono8 only, skipping detection for '%s'", processed->encoding.c_str());
  } else if (cfar_enabled_ && cfar_mask_pub_) {
    auto mask_msg = makeImage(processed->header, processed->height, processed->width,
        enc::MONO8, 1);
    cv::Mat mask(mask_msg->height, mask_msg->width, CV_8UC1, mask_msg->data.data(),
      mask_msg->step);
    cfar_.detect(filtered, mask);
    cfar_mask_pub_->publish(std::move(mask_msg));
  }

  processed_image_pub_->publish(std::move(processed));
}

SonarProcessingNode::Image::UniquePtr SonarProcessingNode::makeImage(
  const std_msgs::msg::Header & header, uint32_t height, uint32_t width,
  const std::string & encoding, size_t elem_size)
{
  auto img = std::make_unique<Image>();
  img->header = header;
  img->height = height;
  img->width = width;
  img->encoding = encoding;
  img->is_bigendian = false;
  img->step = static_cast<uint32_t>(width * elem_size);
  img->data.resize(static_cast<size_t>(img->step) * height);
  return img;
}

}  // namespace sonar_processing

RCLCPP_COMPONENTS_REGISTER_NODE(sonar_processing::SonarProcessingNode)
