#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

class LaserScanFilterNode : public rclcpp::Node
{
public:
  LaserScanFilterNode()
  : Node("laser_scan_filter_node")
  {
    declare_parameter<std::string>("input_topic", "/hardware/base/scan");
    declare_parameter<std::string>("output_topic", "/navigation/base/scan_filtered");
    declare_parameter<bool>("enabled", true);
    declare_parameter<bool>("debug", false);
    declare_parameter<double>("min_valid_range", 0.08);
    declare_parameter<double>("max_valid_range", 4.0);
    declare_parameter<int>("neighbor_window", 2);
    declare_parameter<int>("min_similar_neighbors", 1);
    declare_parameter<double>("similar_range_tolerance", 0.18);
    declare_parameter<bool>("publish_inf_for_invalid", true);

    load_parameters();

    param_callback_handle_ = add_on_set_parameters_callback(
      std::bind(&LaserScanFilterNode::on_parameter_change, this, std::placeholders::_1));

    pub_scan_ = create_publisher<sensor_msgs::msg::LaserScan>(
      output_topic_, rclcpp::SensorDataQoS());

    sub_scan_ = create_subscription<sensor_msgs::msg::LaserScan>(
      input_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&LaserScanFilterNode::scan_callback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "LaserScanFilter.-> Filtering %s -> %s",
      input_topic_.c_str(),
      output_topic_.c_str());
  }

private:
  std::string input_topic_;
  std::string output_topic_;
  bool enabled_{true};
  bool debug_{false};
  double min_valid_range_{0.08};
  double max_valid_range_{4.0};
  int neighbor_window_{2};
  int min_similar_neighbors_{1};
  double similar_range_tolerance_{0.18};
  bool publish_inf_for_invalid_{true};

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_scan_;
  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  void load_parameters()
  {
    get_parameter("input_topic", input_topic_);
    get_parameter("output_topic", output_topic_);
    get_parameter("enabled", enabled_);
    get_parameter("debug", debug_);
    get_parameter("min_valid_range", min_valid_range_);
    get_parameter("max_valid_range", max_valid_range_);
    get_parameter("neighbor_window", neighbor_window_);
    get_parameter("min_similar_neighbors", min_similar_neighbors_);
    get_parameter("similar_range_tolerance", similar_range_tolerance_);
    get_parameter("publish_inf_for_invalid", publish_inf_for_invalid_);

    min_valid_range_ = std::max(0.0, min_valid_range_);
    max_valid_range_ = std::max(min_valid_range_, max_valid_range_);
    neighbor_window_ = std::max(0, neighbor_window_);
    min_similar_neighbors_ = std::max(0, min_similar_neighbors_);
    similar_range_tolerance_ = std::max(0.0, similar_range_tolerance_);
  }

  rcl_interfaces::msg::SetParametersResult on_parameter_change(
    const std::vector<rclcpp::Parameter> &params)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "LaserScanFilter.-> Parameters updated successfully.";

    for (const auto &param : params) {
      const auto &name = param.get_name();
      if (name == "enabled") {
        enabled_ = param.as_bool();
      } else if (name == "debug") {
        debug_ = param.as_bool();
      } else if (name == "min_valid_range") {
        min_valid_range_ = std::max(0.0, param.as_double());
      } else if (name == "max_valid_range") {
        max_valid_range_ = std::max(min_valid_range_, param.as_double());
      } else if (name == "neighbor_window") {
        neighbor_window_ = std::max(0, static_cast<int>(param.as_int()));
      } else if (name == "min_similar_neighbors") {
        min_similar_neighbors_ = std::max(0, static_cast<int>(param.as_int()));
      } else if (name == "similar_range_tolerance") {
        similar_range_tolerance_ = std::max(0.0, param.as_double());
      } else if (name == "publish_inf_for_invalid") {
        publish_inf_for_invalid_ = param.as_bool();
      } else if (name == "input_topic" || name == "output_topic") {
        result.successful = false;
        result.reason = "LaserScanFilter.-> Topic parameters require node restart.";
        break;
      } else {
        result.successful = false;
        result.reason = "LaserScanFilter.-> Unsupported parameter: " + name;
        break;
      }
    }

    return result;
  }

  bool is_valid_range(const float range, const sensor_msgs::msg::LaserScan &scan) const
  {
    if (!std::isfinite(range)) return false;

    const double scan_min = std::max(static_cast<double>(scan.range_min), min_valid_range_);
    const double scan_max = std::min(static_cast<double>(scan.range_max), max_valid_range_);
    return range >= scan_min && range <= scan_max;
  }

  float invalid_range_value() const
  {
    if (publish_inf_for_invalid_) {
      return std::numeric_limits<float>::infinity();
    }
    return std::numeric_limits<float>::quiet_NaN();
  }

  bool has_similar_neighbors(
    const std::vector<float> &ranges,
    const std::vector<bool> &valid,
    const int index) const
  {
    if (min_similar_neighbors_ <= 0 || neighbor_window_ <= 0) return true;

    int similar_count = 0;
    const int size = static_cast<int>(ranges.size());
    const int first = std::max(0, index - neighbor_window_);
    const int last = std::min(size - 1, index + neighbor_window_);

    for (int i = first; i <= last; ++i) {
      if (i == index || !valid[i]) continue;
      if (std::fabs(static_cast<double>(ranges[i] - ranges[index])) <= similar_range_tolerance_) {
        ++similar_count;
        if (similar_count >= min_similar_neighbors_) return true;
      }
    }

    return false;
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    if (!enabled_) {
      pub_scan_->publish(*msg);
      return;
    }

    sensor_msgs::msg::LaserScan filtered = *msg;
    if (filtered.ranges.empty()) {
      pub_scan_->publish(filtered);
      return;
    }

    std::vector<bool> valid(filtered.ranges.size(), false);
    size_t removed_by_range = 0;
    size_t removed_isolated = 0;

    for (size_t i = 0; i < filtered.ranges.size(); ++i) {
      valid[i] = is_valid_range(filtered.ranges[i], filtered);
      if (!valid[i]) {
        filtered.ranges[i] = invalid_range_value();
        if (i < filtered.intensities.size()) filtered.intensities[i] = 0.0f;
        ++removed_by_range;
      }
    }

    const auto range_checked = filtered.ranges;
    for (size_t i = 0; i < range_checked.size(); ++i) {
      if (!valid[i]) continue;
      if (!has_similar_neighbors(range_checked, valid, static_cast<int>(i))) {
        filtered.ranges[i] = invalid_range_value();
        if (i < filtered.intensities.size()) filtered.intensities[i] = 0.0f;
        ++removed_isolated;
      }
    }

    filtered.range_min = std::max(filtered.range_min, static_cast<float>(min_valid_range_));
    filtered.range_max = std::min(filtered.range_max, static_cast<float>(max_valid_range_));

    if (debug_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "LaserScanFilter.-> beams=%zu removed_by_range=%zu removed_isolated=%zu",
        filtered.ranges.size(),
        removed_by_range,
        removed_isolated);
    }

    pub_scan_->publish(filtered);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LaserScanFilterNode>());
  rclcpp::shutdown();
  return 0;
}
