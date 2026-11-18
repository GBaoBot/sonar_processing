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

  declare_parameter<bool>("gaussian_filter.enabled", true);
  declare_parameter<int>("gaussian_filter.kernel_size", 5);
  declare_parameter<double>("gaussian_filter.sigma_x", 1.0);
  declare_parameter<double>("gaussian_filter.sigma_y", 1.0);

  declare_parameter<int>("os_cfar.guard_cells_range", 2);
  declare_parameter<int>("os_cfar.guard_cells_beam", 2);
  declare_parameter<int>("os_cfar.training_cells_range", 8);
  declare_parameter<int>("os_cfar.training_cells_beam", 8);
  // Default window (2, 2, 8, 8) has N = 21*21 - 5*5 = 416 training cells; k = 0.75 * N.
  declare_parameter<int>("os_cfar.kth_order_statistic", 312);
  declare_parameter<double>("os_cfar.false_alarm_rate", 1e-3);
  declare_parameter<double>("os_cfar.min_intensity", 0.0);

  declare_parameter<int>("morphology.kernel_range", 3);
  declare_parameter<int>("morphology.kernel_beam", 3);
  declare_parameter<int>("morphology.min_blob_cells", 4);

  declare_parameter<int>("persistence.persist_n", 2);
  declare_parameter<int>("persistence.persist_m", 3);
  declare_parameter<int>("persistence.persist_tol_range", 1);
  declare_parameter<int>("persistence.persist_tol_beam", 1);

  declare_parameter<std::string>("scan.topic", "sonar/echo/scan");
  declare_parameter<double>("scan.sonar_range_m", 50.0);
  declare_parameter<double>("scan.min_range_m", 0.5);
  declare_parameter<double>("scan.obstacle_max_range_m", 30.0);
  declare_parameter<std::vector<double>>("scan.beam_bearings_deg", std::vector<double>{});
  declare_parameter<double>("scan.fov_deg", 0.0);
  declare_parameter<double>("scan.scan_angle_increment_deg", 0.0);
  declare_parameter<double>("scan.uniform_tol_deg", 0.01);
  declare_parameter<std::string>("scan.empty_beam_value", "inf");

  declareDebugOutput(debug_smoothed_.name, false, "sonar/echo/debug/smoothed");
  declareDebugOutput(debug_cfar_.name, false, "sonar/echo/debug/cfar_mask");
  declareDebugOutput(debug_morphology_.name, false, "sonar/echo/debug/morphology");
  declareDebugOutput(debug_persistence_.name, false, "sonar/echo/debug/persistence");
  declareDebugOutput(debug_returns_.name, false, "sonar/echo/debug/returns");
}

void SonarProcessingNode::declareDebugOutput(
  const std::string & name, bool enabled, const std::string & topic)
{
  declare_parameter<bool>("debug." + name + ".enabled", enabled);
  declare_parameter<std::string>("debug." + name + ".topic", topic);
}

SonarProcessingNode::CallbackReturn SonarProcessingNode::on_configure(
  const rclcpp_lifecycle::State &)
{
  auto get_int = [this](const std::string & n) {
      return static_cast<int>(get_parameter(n).as_int());
    };
  auto get_double = [this](const std::string & n) {return get_parameter(n).as_double();};

  input_topic_ = get_parameter("input_topic").as_string();

  gaussian_enabled_ = get_parameter("gaussian_filter.enabled").as_bool();
  gaussian_filter_config_.kernel_size = get_int("gaussian_filter.kernel_size");
  gaussian_filter_config_.sigma_x = get_double("gaussian_filter.sigma_x");
  gaussian_filter_config_.sigma_y = get_double("gaussian_filter.sigma_y");

  // OsCfarConfig uses image axes: y = rows = range, x = columns = beam.
  cfar_config_.guard_cells_y = get_int("os_cfar.guard_cells_range");
  cfar_config_.guard_cells_x = get_int("os_cfar.guard_cells_beam");
  cfar_config_.training_cells_y = get_int("os_cfar.training_cells_range");
  cfar_config_.training_cells_x = get_int("os_cfar.training_cells_beam");
  cfar_config_.kth_order_statistic = get_int("os_cfar.kth_order_statistic");
  cfar_config_.false_alarm_rate = get_double("os_cfar.false_alarm_rate");
  cfar_config_.min_intensity = get_double("os_cfar.min_intensity");

  morphology_config_.kernel_range = get_int("morphology.kernel_range");
  morphology_config_.kernel_beam = get_int("morphology.kernel_beam");
  morphology_config_.min_blob_cells = get_int("morphology.min_blob_cells");

  persistence_config_.persist_n = get_int("persistence.persist_n");
  persistence_config_.persist_m = get_int("persistence.persist_m");
  persistence_config_.persist_tol_range = get_int("persistence.persist_tol_range");
  persistence_config_.persist_tol_beam = get_int("persistence.persist_tol_beam");

  scan_topic_ = get_parameter("scan.topic").as_string();
  scan_config_.sonar_range_m = get_double("scan.sonar_range_m");
  scan_config_.min_range_m = get_double("scan.min_range_m");
  scan_config_.obstacle_max_range_m = get_double("scan.obstacle_max_range_m");
  scan_config_.beam_bearings_deg = get_parameter("scan.beam_bearings_deg").as_double_array();
  scan_config_.fov_deg = get_double("scan.fov_deg");
  scan_config_.scan_angle_increment_deg = get_double("scan.scan_angle_increment_deg");
  scan_config_.uniform_tol_deg = get_double("scan.uniform_tol_deg");
  const std::string empty_beam_value = get_parameter("scan.empty_beam_value").as_string();

  for (DebugOutput * d :
    {&debug_smoothed_, &debug_cfar_, &debug_morphology_, &debug_persistence_, &debug_returns_})
  {
    d->enabled = get_parameter("debug." + d->name + ".enabled").as_bool();
    d->topic = get_parameter("debug." + d->name + ".topic").as_string();
  }

  try {
    if (empty_beam_value != "inf" && empty_beam_value != "range_max") {
      throw std::invalid_argument(
              "scan.empty_beam_value must be \"inf\" or \"range_max\", got \"" +
              empty_beam_value + "\"");
    }
    scan_config_.empty_beam_inf = (empty_beam_value == "inf");

    GaussianFilter::validate(gaussian_filter_config_);
    cfar_.configure(cfar_config_);
    morphology_.configure(morphology_config_);
    persistence_.configure(persistence_config_);
    scan_builder_.configure(scan_config_);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Configuration failed: %s", e.what());
    return CallbackReturn::FAILURE;
  }

  scan_pub_ = create_publisher<LaserScan>(scan_topic_, rclcpp::SensorDataQoS());
  for (DebugOutput * d :
    {&debug_smoothed_, &debug_cfar_, &debug_morphology_, &debug_persistence_, &debug_returns_})
  {
    if (d->enabled) {
      d->pub = create_publisher<Image>(d->topic, rclcpp::SensorDataQoS());
    }
  }

  RCLCPP_INFO(get_logger(),
    "Configured: %s -> %s, gaussian_filter enabled=%s kernel=%d sigma=(%.2f, %.2f)",
    input_topic_.c_str(), scan_topic_.c_str(), gaussian_enabled_ ? "true" : "false",
    gaussian_filter_config_.kernel_size, gaussian_filter_config_.sigma_x,
    gaussian_filter_config_.sigma_y);
  RCLCPP_INFO(get_logger(),
    "OS-CFAR: guard=(%d, %d) training=(%d, %d) (range, beam) k=%d pfa=%g min_intensity=%.2f",
    cfar_config_.guard_cells_y, cfar_config_.guard_cells_x,
    cfar_config_.training_cells_y, cfar_config_.training_cells_x,
    cfar_config_.kth_order_statistic, cfar_config_.false_alarm_rate,
    cfar_config_.min_intensity);
  RCLCPP_INFO(get_logger(),
    "Morphology: open kernel=(%d, %d) (range, beam) min_blob_cells=%d; "
    "persistence: %d of %d pings, tol=(%d, %d)",
    morphology_config_.kernel_range, morphology_config_.kernel_beam,
    morphology_config_.min_blob_cells, persistence_config_.persist_n,
    persistence_config_.persist_m, persistence_config_.persist_tol_range,
    persistence_config_.persist_tol_beam);
  RCLCPP_INFO(get_logger(),
    "Scan: sonar_range=%.2f m, returns in [%.2f, %.2f] m, bearings=%s, empty beams=%s",
    scan_config_.sonar_range_m, scan_config_.min_range_m, scan_config_.obstacle_max_range_m,
    scan_config_.beam_bearings_deg.empty() ? "uniform (fov)" : "table",
    empty_beam_value.c_str());
  return CallbackReturn::SUCCESS;
}

SonarProcessingNode::CallbackReturn SonarProcessingNode::on_activate(
  const rclcpp_lifecycle::State & state)
{
  if (!scan_pub_) {
    RCLCPP_ERROR(get_logger(), "Cannot activate before the publisher is configured.");
    return CallbackReturn::FAILURE;
  }

  LifecycleNode::on_activate(state);

  // History from before a pause no longer matches the scene.
  persistence_.reset();
  last_ping_stamp_.reset();

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
  scan_pub_.reset();
  for (DebugOutput * d :
    {&debug_smoothed_, &debug_cfar_, &debug_morphology_, &debug_persistence_, &debug_returns_})
  {
    d->pub.reset();
  }
}

cv::Mat SonarProcessingNode::stageBuffer(
  const DebugOutput & out, cv::Mat & scratch, const std_msgs::msg::Header & header,
  int rows, int cols, Image::UniquePtr & msg)
{
  if (out.pub) {
    msg = makeImage(header, rows, cols, enc::MONO8, 1);
    return cv::Mat(rows, cols, CV_8UC1, msg->data.data(), msg->step);
  }
  scratch.create(rows, cols, CV_8UC1);
  return scratch;
}

void SonarProcessingNode::imageCallback(Image::UniquePtr msg)
{
  if (!scan_pub_ || !scan_pub_->is_activated()) {return;}

  if (msg->encoding != enc::MONO8) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
      "Unsupported encoding '%s', the detection pipeline needs mono8", msg->encoding.c_str());
    return;
  }
  const int rows = static_cast<int>(msg->height);
  const int cols = static_cast<int>(msg->width);
  if (rows == 0 || cols == 0) {return;}

  // The ECHO driver stamps each image when the frame is decoded; that stamp is the closest
  // thing to the ping time we have, and the scan keeps it unchanged.
  const std_msgs::msg::Header header = msg->header;

  // Wrap the message buffer directly, no copy in.
  const cv::Mat src(rows, cols, CV_8UC1, msg->data.data(), msg->step);

  // Debug messages are published only at the end: the Mats below wrap their buffers.
  Image::UniquePtr smoothed_msg, cfar_msg, morphology_msg, persistence_msg;
  std::unique_ptr<LaserScan> scan = std::make_unique<LaserScan>();

  try {
    // 1. Smoothing.
    cv::Mat smoothed = src;
    if (gaussian_enabled_) {
      smoothed = stageBuffer(debug_smoothed_, smoothed_buf_, header, rows, cols, smoothed_msg);
      GaussianFilter::perform(src, smoothed, gaussian_filter_config_);
    } else if (debug_smoothed_.pub) {
      // Pass-through: republish the input itself. Moving the unique_ptr leaves the buffer (and
      // src, which wraps it) in place.
      smoothed_msg = std::move(msg);
    }

    // 2. OS-CFAR.
    cv::Mat cfar_mask = stageBuffer(debug_cfar_, cfar_buf_, header, rows, cols, cfar_msg);
    cfar_.detect(smoothed, cfar_mask);

    // 3. Morphological cleanup.
    cv::Mat morph_mask =
      stageBuffer(debug_morphology_, morphology_buf_, header, rows, cols, morphology_msg);
    morphology_.apply(cfar_mask, morph_mask);

    // 4. Persistence (N of the last M pings).
    cv::Mat persist_mask =
      stageBuffer(debug_persistence_, persistence_buf_, header, rows, cols, persistence_msg);
    persistence_.apply(morph_mask, persist_mask);

    // 5. First return per beam + resampling to a uniform LaserScan.
    scan_builder_.build(persist_mask, *scan, return_rows_);
  } catch (const std::exception & e) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Dropping ping: %s", e.what());
    return;
  }

  scan->header = header;
  const rclcpp::Time stamp(header.stamp, get_clock()->get_clock_type());
  if (last_ping_stamp_ && last_ping_stamp_->get_clock_type() == stamp.get_clock_type() &&
    stamp > *last_ping_stamp_)
  {
    scan->scan_time = static_cast<float>((stamp - *last_ping_stamp_).seconds());
  }
  last_ping_stamp_ = stamp;

  if (debug_smoothed_.pub && smoothed_msg) {
    debug_smoothed_.pub->publish(std::move(smoothed_msg));
  }
  if (debug_cfar_.pub && cfar_msg) {
    debug_cfar_.pub->publish(std::move(cfar_msg));
  }
  if (debug_morphology_.pub && morphology_msg) {
    debug_morphology_.pub->publish(std::move(morphology_msg));
  }
  if (debug_persistence_.pub && persistence_msg) {
    debug_persistence_.pub->publish(std::move(persistence_msg));
  }
  if (debug_returns_.pub) {
    // One lit cell per beam at its first return (polar, original beam columns).
    auto returns_msg = makeImage(header, rows, cols, enc::MONO8, 1);
    for (int c = 0; c < cols; ++c) {
      if (return_rows_[c] >= 0) {
        returns_msg->data[static_cast<size_t>(return_rows_[c]) * returns_msg->step + c] = 255;
      }
    }
    debug_returns_.pub->publish(std::move(returns_msg));
  }

  scan_pub_->publish(std::move(scan));
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
