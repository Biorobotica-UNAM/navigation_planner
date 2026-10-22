#include "map_augmenter/grid_utils.hpp"

#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

class ObstacleDetectorNode : public rclcpp::Node
{
  using Grid = map_augmenter::Grid;
  using Cloud = sensor_msgs::msg::PointCloud2;
  using Scan = sensor_msgs::msg::LaserScan;
  using Field = sensor_msgs::msg::PointField;
  static constexpr int64_t never_observed = std::numeric_limits<int64_t>::min();

public:
  ObstacleDetectorNode()
  : Node("obstacle_detector_node"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    use_namespace_ = parameter("use_namespace", false, "Namespace the default internal endpoints.");
    use_lidar_ = parameter("use_lidar", false, "Enable the LaserScan input.");
    use_cloud_ = parameter("use_point_cloud", false, "Enable camera input 1 (PointCloud2).");
    use_cloud2_ = parameter("use_point_cloud2", false, "Enable optional camera input 2 (also PointCloud2).");
    laser_box_ = declare_box("laser");
    cloud_box_ = declare_box("cloud");
    base_link_ = parameter("base_link_name", std::string("base_footprint"), "Robot base TF frame.");
    map_frame_ = parameter("map_frame", std::string("map"), "Required map TF frame.");
    odom_frame_ = parameter("odom_frame", std::string("odom"), "Odometry TF frame.");
    const double period = parameter("processing_period_sec", 0.075, "Integration and publication interval.");
    obstacle_timeout_ = parameter("obstacle_timeout_sec", 0.75, "Lifetime since the last sensor observation.");
    sensor_max_age_ = parameter("sensor_max_age_sec", 0.5, "Maximum accepted sensor-message age.");
    future_tolerance_ = parameter("sensor_future_tolerance_sec", 0.05, "Maximum sensor timestamp lead.");
    allow_zero_stamp_ = parameter("allow_zero_sensor_stamp", false,
      "Interpret a zero sensor stamp as reception/processing time; false rejects it.");
    tf_timeout_ = parameter("tf_timeout_sec", 0.03, "Per-transform lookup timeout.");
    allow_tf_fallback_ = parameter("tf_allow_latest_fallback", true, "Allow a recent latest TF fallback.");
    const double legacy_tf_warning = parameter("tf_fallback_warn_threshold_sec", 0.20,
      "Compatibility default for tf_fallback_max_age_sec; stale fallback transforms are now rejected.");
    tf_max_age_ = parameter("tf_fallback_max_age_sec", legacy_tf_warning, "Maximum fallback TF time difference.");
    static_ignore_radius_ = parameter("static_obstacle_ignore_radius", 0.10,
      "Suppress detections near raw static obstacles, in metres.");
    ignore_static_ = parameter("ignore_static_obstacles", true, "Suppress detections already represented by the base map.");
    self_ignore_radius_ = parameter("self_obstacle_ignore_radius", 0.0,
      "Ignore sensor points within this horizontal distance of the robot base.");
    laser_min_range_ = parameter("laser_min_range", 0.10, "Additional minimum laser range.");
    cloud_downsampling_ = parameter("cloud_downsampling", 1, "Camera input 1 sample stride.");
    cloud_downsampling2_ = parameter("cloud_downsampling2", 1, "Camera input 2 sample stride.");
    lidar_downsampling_ = parameter("lidar_downsampling", 1, "LaserScan sample stride.");
    cloud_min_points_ = parameter("cloud_min_points_per_cell", 2, "Minimum per-frame cloud evidence per cell.");
    cloud_min_neighbors_ = parameter("cloud_min_neighbor_cells", 1,
      "Minimum neighboring cells with sufficient evidence in the surrounding 3x3 cells.");
    clear_on_pose_jump_ = parameter("clear_on_pose_jump", true, "Clear transient evidence after large map-to-odom corrections.");
    pose_jump_translation_ = parameter("pose_jump_translation_m", 0.5, "Translation change that clears transient evidence.");
    pose_jump_rotation_ = parameter("pose_jump_rotation_rad", 0.35, "Rotation change that clears transient evidence.");
    log_throttle_ms_ = parameter("log_throttle_ms", 2000, "Repeated warning interval in milliseconds.");
    const int grid_depth = parameter("map_qos_depth", 1, "Grid history depth.");
    const int sensor_depth = parameter("sensor_qos_depth", 5, "Sensor subscription history depth.");
    const bool map_latched = parameter("map_input_transient_local", true, "Receive the retained base map.");
    const bool map_reliable = parameter("map_input_reliable", true, "Request reliable base maps.");
    const bool output_latched = parameter("map_output_transient_local", true, "Retain the obstacle grid.");
    const bool output_reliable = parameter("map_output_reliable", true, "Publish reliable obstacle grids.");
    const bool sensor_reliable = parameter("sensor_input_reliable", false, "Request reliable sensor data instead of best effort.");
    if (!positive(period) || !positive(obstacle_timeout_) || !nonnegative(sensor_max_age_) ||
      !nonnegative(future_tolerance_) || !nonnegative(tf_timeout_) || !nonnegative(tf_max_age_) ||
      !nonnegative(static_ignore_radius_) || !nonnegative(self_ignore_radius_) ||
      !nonnegative(laser_min_range_) || !positive(pose_jump_translation_) || !positive(pose_jump_rotation_) ||
      cloud_downsampling_ < 1 || cloud_downsampling2_ < 1 || lidar_downsampling_ < 1 ||
      cloud_min_points_ < 1 || cloud_min_neighbors_ < 0 || cloud_min_neighbors_ > 8 ||
      log_throttle_ms_ < 1 || grid_depth < 1 || sensor_depth < 1 ||
      map_frame_.empty() || base_link_.empty() || odom_frame_.empty())
    {
      throw std::invalid_argument("Invalid obstacle detector parameter range");
    }

    // Existing launch files still pass these. They no longer control this node.
    parameter("decay_factor", 10, "Deprecated; use obstacle_timeout_sec instead of per-cycle decay.");
    parameter("use_online", false, "Compatibility only; the augmenter relays all base-map updates.");
    parameter("static_map_server", std::string("/static_map_server/map"),
      "Compatibility only; map acquisition now belongs to map_augmenter.");
    parameter("prohibition_map_server", std::string("/prohibition_map_server/map"),
      "Compatibility only; prohibition overlay belongs to map_augmenter.");
    parameter("inflation_radius", 0.25, "Compatibility only; inflation belongs to map_augmenter.");
    parameter("cost_radius", 0.25, "Compatibility only; soft costs belong to map_augmenter.");
    parameter("use_ground_filter", false, "Legacy unused option; sensor crop limits control accepted heights.");
    parameter("use_ransac", true, "Legacy unused option; no plane fitting is performed here.");
    parameter("ground_height_threshold", 0.05, "Legacy unused ground-filter setting.");
    parameter("ransac_distance_threshold", 0.03, "Legacy unused ground-filter setting.");

    auto map_qos = rclcpp::QoS(grid_depth);
    if (map_latched) {map_qos.transient_local();}
    if (map_reliable) {map_qos.reliable();} else {map_qos.best_effort();}
    auto output_qos = rclcpp::QoS(grid_depth);
    if (output_latched) {output_qos.transient_local();}
    if (output_reliable) {output_qos.reliable();} else {output_qos.best_effort();}
    auto sensor_qos = rclcpp::QoS(sensor_depth);
    if (sensor_reliable) {sensor_qos.reliable();} else {sensor_qos.best_effort();}

    pub_obstacles_ = create_publisher<Grid>(endpoint("obstacles_map_topic", "/obstacles_map"), output_qos);
    sub_base_ = create_subscription<Grid>(
      endpoint("base_map_topic", "/map_augmenter/base_map"), map_qos,
      [this](Grid::ConstSharedPtr msg) {on_base_map(*msg);});
    const auto topic1 = parameter("point_cloud_topic", std::string("/point_cloud"), "Camera input 1 PointCloud2 topic.");
    const auto topic2 = parameter("point_cloud_topic2", std::string("/point_cloud2"), "Optional camera input 2 PointCloud2 topic.");
    const auto laser_topic = parameter("laser_scan_topic", std::string("/scan"), "Hokuyo or other LaserScan topic.");
    if (use_cloud_) {
      sub_cloud1_ = create_subscription<Cloud>(topic1, sensor_qos,
        [this](Cloud::ConstSharedPtr msg) {latest_cloud1_ = std::move(msg);});
    }
    if (use_cloud2_) {
      sub_cloud2_ = create_subscription<Cloud>(topic2, sensor_qos,
        [this](Cloud::ConstSharedPtr msg) {latest_cloud2_ = std::move(msg);});
    }
    if (use_lidar_) {
      sub_scan_ = create_subscription<Scan>(laser_topic, sensor_qos,
        [this](Scan::ConstSharedPtr msg) {latest_scan_ = std::move(msg);});
    }

    // Sensor, map and timer callbacks share the default mutually-exclusive group.
    // A frame is consumed once, including when it is rejected for age or bad TF.
    processing_timer_ = create_wall_timer(std::chrono::duration<double>(period), [this]() {processing();});
    RCLCPP_INFO(get_logger(), "Obstacle detector ready; configuration parameters are startup-only");
  }

private:
  struct Box
  {
    double min_x, max_x, min_y, max_y, min_z, max_z;
    bool contains(const Eigen::Vector3d & p) const
    {
      return p.allFinite() && p.x() > min_x && p.x() < max_x &&
             p.y() > min_y && p.y() < max_y && p.z() > min_z && p.z() < max_z;
    }
  };

  template<typename T>
  T parameter(const std::string & name, const T & value, const std::string & description)
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    descriptor.description = description;
    return declare_parameter<T>(name, value, descriptor);
  }

  std::string endpoint(const std::string & name, const std::string & old_default)
  {
    std::string value = use_namespace_ ? std::string(get_namespace()) + old_default : old_default;
    if (value.compare(0, 2, "//") == 0) {value.erase(0, 1);}
    return parameter(name, value, "ROS endpoint; explicit relative values use the node namespace.");
  }

  Box declare_box(const std::string & prefix)
  {
    Box box{
      parameter(prefix + "_min_x", -10.0, "Minimum sensor X in the robot base frame, metres."),
      parameter(prefix + "_max_x", 10.0, "Maximum sensor X in the robot base frame, metres."),
      parameter(prefix + "_min_y", -10.0, "Minimum sensor Y in the robot base frame, metres."),
      parameter(prefix + "_max_y", 10.0, "Maximum sensor Y in the robot base frame, metres."),
      parameter(prefix + "_min_z", -1.0, "Minimum sensor Z in the robot base frame, metres."),
      parameter(prefix + "_max_z", 2.0, "Maximum sensor Z in the robot base frame, metres.")};
    if (!std::isfinite(box.min_x) || !std::isfinite(box.max_x) ||
      !std::isfinite(box.min_y) || !std::isfinite(box.max_y) ||
      !std::isfinite(box.min_z) || !std::isfinite(box.max_z) ||
      box.min_x >= box.max_x || box.min_y >= box.max_y || box.min_z >= box.max_z)
    {
      throw std::invalid_argument("Invalid " + prefix + " crop box");
    }
    return box;
  }

  static bool positive(double value) {return std::isfinite(value) && value > 0.0;}
  static bool nonnegative(double value) {return std::isfinite(value) && value >= 0.0;}

  void on_base_map(const Grid & map)
  {
    std::string reason;
    if (!map_augmenter::validate_grid(map, &reason)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), log_throttle_ms_, "Rejected base map: %s", reason.c_str());
      return;
    }
    if (map.header.frame_id != map_frame_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), log_throttle_ms_,
        "Rejected base-map frame '%s'; expected '%s'", map.header.frame_id.c_str(), map_frame_.c_str());
      return;
    }
    const bool changed = !have_grid_ || !map_augmenter::same_geometry(obstacles_map_, map);
    const bool reference_changed = changed || static_reference_.data != map.data;
    if (changed) {
      Grid resized = map_augmenter::empty_obstacle_grid(map);
      std::vector<int64_t> times(resized.data.size(), never_observed);
      if (have_grid_) {
        map_augmenter::for_each_projected_obstacle(obstacles_map_, resized, [&](size_t from, size_t to) {
          resized.data[to] = std::max(resized.data[to], obstacles_map_.data[from]);
          times[to] = std::max(times[to], last_observed_ns_[from]);
        });
      }
      obstacles_map_ = std::move(resized);
      last_observed_ns_ = std::move(times);
    }
    static_reference_ = map;
    if (reference_changed && ignore_static_) {
      static_ignore_mask_ = map_augmenter::inflate_obstacles(map, static_ignore_radius_, true);
    }
    have_grid_ = true;
  }

  bool sensor_stamp(const builtin_interfaces::msg::Time & header, rclcpp::Time & stamp)
  {
    stamp = rclcpp::Time(header, get_clock()->get_clock_type());
    if (stamp.nanoseconds() == 0) {
      if (!allow_zero_stamp_) {return false;}
      stamp = now();
    }
    if (stamp.nanoseconds() <= reset_before_ns_) {return false;}
    const double age = (now() - stamp).seconds();
    return age <= sensor_max_age_ && age >= -future_tolerance_;
  }

  bool lookup_transform(const std::string & target, const std::string & source,
    const rclcpp::Time & stamp, Eigen::Affine3d & result)
  {
    if (source.empty()) {return false;}
    try {
      const auto tf = tf_buffer_.lookupTransform(target, source, stamp,
        rclcpp::Duration::from_seconds(tf_timeout_));
      result = tf2::transformToEigen(tf.transform);
      return true;
    } catch (const tf2::TransformException &) {
      if (!allow_tf_fallback_) {return false;}
    }
    try {
      const auto tf = tf_buffer_.lookupTransform(target, source,
        rclcpp::Time(0, 0, get_clock()->get_clock_type()),
        rclcpp::Duration::from_seconds(tf_timeout_));
      const rclcpp::Time tf_stamp(tf.header.stamp, get_clock()->get_clock_type());
      // A zero timestamp is allowed for a static transform.
      if (tf_stamp.nanoseconds() != 0 && std::abs((stamp - tf_stamp).seconds()) > tf_max_age_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), log_throttle_ms_,
          "Rejected stale fallback TF from '%s' to '%s'", source.c_str(), target.c_str());
        return false;
      }
      result = tf2::transformToEigen(tf.transform);
      return true;
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), log_throttle_ms_, "TF unavailable: %s", error.what());
      return false;
    }
  }

  void mark_obstacle(int x, int y, const rclcpp::Time & stamp)
  {
    const size_t index = map_augmenter::cell_index(obstacles_map_, x, y);
    if (ignore_static_ && static_ignore_mask_.data[index] > 0) {return;}
    // Occupancy encoding is fixed: unknown=-1, free=0, occupied=100.
    obstacles_map_.data[index] = 100;
    last_observed_ns_[index] = std::max(last_observed_ns_[index], stamp.nanoseconds());
  }

  // Validate arbitrary PointCloud2 layouts instead of assuming x/y/z at 0/4/8.
  static const Field * coordinate_field(const Cloud & cloud, const std::string & name)
  {
    for (const auto & field : cloud.fields) {
      if (field.name != name || field.count < 1) {continue;}
      const uint32_t size = field.datatype == Field::FLOAT32 ? sizeof(float) :
        field.datatype == Field::FLOAT64 ? sizeof(double) : 0;
      if (size != 0 && static_cast<uint64_t>(field.offset) + size <= cloud.point_step) {return &field;}
    }
    return nullptr;
  }

  template<typename Scalar>
  static double read_scalar(const uint8_t * source, bool big_endian)
  {
    std::array<uint8_t, sizeof(Scalar)> bytes;
    std::copy_n(source, bytes.size(), bytes.begin());
    const uint16_t native = 1;
    const bool native_big = *reinterpret_cast<const uint8_t *>(&native) == 0;
    if (big_endian != native_big) {std::reverse(bytes.begin(), bytes.end());}
    Scalar value;
    std::memcpy(&value, bytes.data(), sizeof(value));
    return static_cast<double>(value);
  }

  static double read_coordinate(const uint8_t * point, const Field & field, bool big_endian)
  {
    const auto * source = point + field.offset;
    return field.datatype == Field::FLOAT32 ?
      read_scalar<float>(source, big_endian) : read_scalar<double>(source, big_endian);
  }

  struct CellEvidence
  {
    size_t count{0};
    int x{0}, y{0};
  };

  void add_cloud(const Cloud & cloud, int stride)
  {
    if (cloud.width == 0 || cloud.height == 0) {return;}
    const auto * fx = coordinate_field(cloud, "x");
    const auto * fy = coordinate_field(cloud, "y");
    const auto * fz = coordinate_field(cloud, "z");
    const uint64_t minimum_row = static_cast<uint64_t>(cloud.width) * cloud.point_step;
    const uint64_t required = static_cast<uint64_t>(cloud.row_step) * cloud.height;
    if (!fx || !fy || !fz || cloud.row_step < minimum_row || required > cloud.data.size()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), log_throttle_ms_, "Rejected malformed PointCloud2 layout");
      return;
    }
    rclcpp::Time stamp(0, 0, get_clock()->get_clock_type());
    if (!sensor_stamp(cloud.header.stamp, stamp)) {return;}
    Eigen::Affine3d sensor_to_base, base_to_map;
    if (!lookup_transform(base_link_, cloud.header.frame_id, stamp, sensor_to_base) ||
      !lookup_transform(map_frame_, base_link_, stamp, base_to_map)) {return;}

    std::unordered_map<size_t, CellEvidence> hits;
    const size_t total = static_cast<size_t>(cloud.width) * cloud.height;
    for (size_t i = 0; i < total; i += static_cast<size_t>(stride)) {
      // Respect padding between rows in organised Xtion clouds.
      const auto * p = cloud.data.data() + (i / cloud.width) * cloud.row_step +
        (i % cloud.width) * cloud.point_step;
      const Eigen::Vector3d point(
        read_coordinate(p, *fx, cloud.is_bigendian),
        read_coordinate(p, *fy, cloud.is_bigendian),
        read_coordinate(p, *fz, cloud.is_bigendian));
      if (!point.allFinite()) {continue;}
      const Eigen::Vector3d base_point = sensor_to_base * point;
      if (!cloud_box_.contains(base_point) || base_point.head<2>().norm() < self_ignore_radius_) {continue;}
      const Eigen::Vector3d map_point = base_to_map * base_point;
      int x, y;
      if (!map_augmenter::world_to_cell(obstacles_map_, map_point.x(), map_point.y(), x, y)) {continue;}
      auto & hit = hits[map_augmenter::cell_index(obstacles_map_, x, y)];
      ++hit.count;
      hit.x = x;
      hit.y = y;
    }
    for (const auto & entry : hits) {
      const auto & hit = entry.second;
      if (hit.count < static_cast<size_t>(cloud_min_points_)) {continue;}
      int neighbors = 0;
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          if (dx == 0 && dy == 0) {continue;}
          const int x = hit.x + dx, y = hit.y + dy;
          if (x < 0 || y < 0 || x >= static_cast<int>(obstacles_map_.info.width) ||
            y >= static_cast<int>(obstacles_map_.info.height)) {continue;}
          const auto found = hits.find(map_augmenter::cell_index(obstacles_map_, x, y));
          if (found != hits.end() && found->second.count >= static_cast<size_t>(cloud_min_points_)) {++neighbors;}
        }
      }
      if (neighbors >= cloud_min_neighbors_) {mark_obstacle(hit.x, hit.y, stamp);}
    }
  }

  void add_scan(const Scan & scan)
  {
    if (!std::isfinite(scan.range_min) || !std::isfinite(scan.range_max) ||
      scan.range_min < 0.0f || scan.range_max < scan.range_min ||
      !std::isfinite(scan.angle_min) || !std::isfinite(scan.angle_increment)) {return;}
    rclcpp::Time stamp(0, 0, get_clock()->get_clock_type());
    if (!sensor_stamp(scan.header.stamp, stamp)) {return;}
    Eigen::Affine3d sensor_to_base, base_to_map;
    if (!lookup_transform(base_link_, scan.header.frame_id, stamp, sensor_to_base) ||
      !lookup_transform(map_frame_, base_link_, stamp, base_to_map)) {return;}
    for (size_t i = 0; i < scan.ranges.size(); i += static_cast<size_t>(lidar_downsampling_)) {
      const double range = scan.ranges[i];
      if (!std::isfinite(range) || range < scan.range_min ||
        range > scan.range_max || range < laser_min_range_) {continue;}
      const double angle = scan.angle_min + i * scan.angle_increment;
      const Eigen::Vector3d base_point = sensor_to_base *
        Eigen::Vector3d(range * std::cos(angle), range * std::sin(angle), 0.0);
      if (!laser_box_.contains(base_point) || base_point.head<2>().norm() < self_ignore_radius_) {continue;}
      const Eigen::Vector3d point = base_to_map * base_point;
      int x, y;
      if (map_augmenter::world_to_cell(obstacles_map_, point.x(), point.y(), x, y)) {mark_obstacle(x, y, stamp);}
    }
  }

  void clear_transient_obstacles()
  {
    std::fill(obstacles_map_.data.begin(), obstacles_map_.data.end(), 0);
    std::fill(last_observed_ns_.begin(), last_observed_ns_.end(), never_observed);
    latest_cloud1_.reset();
    latest_cloud2_.reset();
    latest_scan_.reset();
    reset_before_ns_ = now().nanoseconds();
  }

  void check_pose_correction()
  {
    if (!clear_on_pose_jump_) {return;}
    try {
      const auto tf = tf_buffer_.lookupTransform(map_frame_, odom_frame_, tf2::TimePointZero);
      const Eigen::Affine3d current = tf2::transformToEigen(tf.transform);
      if (have_pose_correction_) {
        const Eigen::Affine3d delta = previous_map_to_odom_.inverse() * current;
        if (delta.translation().norm() > pose_jump_translation_ ||
          std::abs(Eigen::AngleAxisd(delta.rotation()).angle()) > pose_jump_rotation_)
        {
          clear_transient_obstacles();
          RCLCPP_INFO(get_logger(), "Cleared transient obstacles after map-to-odom correction");
        }
      }
      previous_map_to_odom_ = current;
      have_pose_correction_ = true;
    } catch (const tf2::TransformException &) {
      // No fabricated identity transform. Sensors still need valid TF below.
    }
  }

  void processing()
  {
    if (!have_grid_) {return;}
    const auto current = now();
    const int64_t current_ns = current.nanoseconds();
    if (previous_processing_ns_ != never_observed && current_ns < previous_processing_ns_) {
      clear_transient_obstacles();
      have_pose_correction_ = false;
    }
    previous_processing_ns_ = current_ns;
    check_pose_correction();

    const int64_t lifetime = rclcpp::Duration::from_seconds(obstacle_timeout_).nanoseconds();
    for (size_t i = 0; i < obstacles_map_.data.size(); ++i) {
      if (obstacles_map_.data[i] <= 0) {continue;}
      const auto observed = last_observed_ns_[i];
      // Future stamps within the configured tolerance retain their evidence.
      if (observed == never_observed || (current_ns >= observed && current_ns - observed >= lifetime)) {
        obstacles_map_.data[i] = 0;
        last_observed_ns_[i] = never_observed;
      }
    }

    if (auto cloud = std::exchange(latest_cloud1_, nullptr)) {add_cloud(*cloud, cloud_downsampling_);}
    if (auto cloud = std::exchange(latest_cloud2_, nullptr)) {add_cloud(*cloud, cloud_downsampling2_);}
    if (auto scan = std::exchange(latest_scan_, nullptr)) {add_scan(*scan);}
    // Late, but still acceptable sensor frames must not resurrect already expired evidence.
    const int64_t publish_ns = now().nanoseconds();
    for (size_t i = 0; i < obstacles_map_.data.size(); ++i) {
      const int64_t observed = last_observed_ns_[i];
      if (obstacles_map_.data[i] > 0 && observed != never_observed &&
        publish_ns >= observed && publish_ns - observed >= lifetime)
      {
        obstacles_map_.data[i] = 0;
        last_observed_ns_[i] = never_observed;
      }
    }
    obstacles_map_.header.stamp = now();
    pub_obstacles_->publish(obstacles_map_);
  }

  bool use_namespace_{false}, use_lidar_{false}, use_cloud_{false}, use_cloud2_{false};
  bool have_grid_{false}, ignore_static_{true}, allow_zero_stamp_{false};
  bool allow_tf_fallback_{true}, clear_on_pose_jump_{true}, have_pose_correction_{false};
  Box laser_box_, cloud_box_;
  std::string base_link_, map_frame_, odom_frame_;
  double obstacle_timeout_, sensor_max_age_, future_tolerance_, tf_timeout_, tf_max_age_;
  double static_ignore_radius_, self_ignore_radius_, laser_min_range_;
  double pose_jump_translation_, pose_jump_rotation_;
  int cloud_downsampling_, cloud_downsampling2_, lidar_downsampling_;
  int cloud_min_points_, cloud_min_neighbors_, log_throttle_ms_;
  int64_t reset_before_ns_{never_observed}, previous_processing_ns_{never_observed};
  Grid obstacles_map_, static_reference_, static_ignore_mask_;
  std::vector<int64_t> last_observed_ns_;
  Eigen::Affine3d previous_map_to_odom_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  Cloud::ConstSharedPtr latest_cloud1_, latest_cloud2_;
  Scan::ConstSharedPtr latest_scan_;
  rclcpp::Publisher<Grid>::SharedPtr pub_obstacles_;
  rclcpp::Subscription<Grid>::SharedPtr sub_base_;
  rclcpp::Subscription<Cloud>::SharedPtr sub_cloud1_, sub_cloud2_;
  rclcpp::Subscription<Scan>::SharedPtr sub_scan_;
  rclcpp::TimerBase::SharedPtr processing_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstacleDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
