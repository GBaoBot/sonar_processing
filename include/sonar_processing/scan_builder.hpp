#pragma once

#include <vector>

#include <opencv2/core.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

namespace sonar_processing
{

// Geometry of the polar image: row r covers range [r * res, (r + 1) * res) with
// res = sonar_range_m / rows (row 0 at the transducer); column c is beam c.
struct ScanConfig
{
  double sonar_range_m{50.0};          // range at the far edge of the polar image
  double min_range_m{0.5};             // returns closer than this are ignored (ringdown)
  double obstacle_max_range_m{30.0};   // returns farther than this are ignored
  // Per-column bearings in degrees, ROS convention (0 = sonar +x, positive = CCW = port).
  // Empty = uniform table from fov_deg, with column 0 the leftmost (port) beam.
  std::vector<double> beam_bearings_deg;
  double fov_deg{0.0};                 // only for the uniform table; 0 = driver rule (see .cpp)
  double scan_angle_increment_deg{0.0};  // output spacing; 0 = finest original beam spacing
  double uniform_tol_deg{0.01};        // bearings within this of uniform skip resampling
  bool empty_beam_inf{true};           // empty beam: true = +inf, false = range_max
};

// Turns a binary polar detection mask into a LaserScan: first return per beam, then a
// conservative resampling onto the uniform angular grid LaserScan requires.
class ScanBuilder
{
public:
  // Throws std::invalid_argument if the config is not usable.
  static void validate(const ScanConfig & config);
  void configure(const ScanConfig & config);

  // mask: CV_8UC1 polar mask, nonzero = detection. Fills every LaserScan field except header
  // and scan_time. return_rows gets, per column, the row of the first return (-1 = none).
  // Throws std::invalid_argument if the mask width does not match the bearing table.
  void build(
    const cv::Mat & mask, sensor_msgs::msg::LaserScan & scan, std::vector<int> & return_rows);

  // Valid after the first build(): false when the bearings were uniform (no resampling).
  bool resampling() const {return !uniform_;}
  int outputBins() const {return n_bins_;}

private:
  // Builds the bearing table for this image width and the beam -> output bin mapping.
  void prepare(int cols);

  ScanConfig config_;
  int prepared_cols_{-1};
  bool uniform_{true};
  int n_bins_{0};
  double angle_min_{0.0};
  double angle_inc_{0.0};
  std::vector<int> bin_of_beam_;   // per original beam: output bin it falls in
  std::vector<int> fill_src_;      // per output bin: nearest beam if no beam falls in it, else -1
  std::vector<float> beam_range_;  // per original beam: first-return range, +inf = none
};

}  // namespace sonar_processing
