#include "map_augmenter/grid_utils.hpp"

#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>
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
#include <iomanip>
#include <limits>
#include <memory>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

class ObstacleDetectorNode : public rclcpp::Node
{
  using Grid = map_augmenter::Grid;
  using Cloud = sensor_msgs::msg::PointCloud2;
  using Scan = sensor_msgs::msg::LaserScan;
  using Field = sensor_msgs::msg::PointField;
  using SteadyClock = std::chrono::steady_clock;
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
    cloud_follow_head_ = parameter("cloud_follow_head", true,
      "Rotate the horizontal cloud crop about the base origin to follow the camera viewing yaw.");
    const auto forward_axis = parameter("cloud_forward_axis", std::string("z"),
      "Forward axis of the cloud TF frame: x, y or z; optical camera frames use z.");
    if (forward_axis == "x") {cloud_forward_axis_ = Eigen::Vector3d::UnitX();}
    else if (forward_axis == "y") {cloud_forward_axis_ = Eigen::Vector3d::UnitY();}
    else if (forward_axis == "z") {cloud_forward_axis_ = Eigen::Vector3d::UnitZ();}
    else {throw std::invalid_argument("cloud_forward_axis must be x, y or z");}
    cloud_yaw_min_projection_ = parameter("cloud_yaw_min_horizontal_projection", 0.1,
      "Minimum horizontal forward-axis projection for a defined viewing yaw; otherwise use base-aligned crop.");
    cloud_gating_enabled_ = parameter("cloud_gating_enabled", false,
      "Use navigation/cloud enable topics to create and destroy cloud subscriptions; LiDAR is independent.");
    navigation_active_ = parameter("cloud_navigation_initially_active", false,
      "Initial navigation gate when cloud gating is enabled; false waits for an enable message.");
    cloud_override_enabled_ = parameter("cloud_override_initially_enabled", true,
      "Initial cloud gate; both navigation and cloud gates must be true to subscribe.");
    remember_all_obstacles_ = parameter("remember_all_obstacles", false,
      "Keep accepted obstacle cells in map coordinates until cleared; pose/clock resets also clear memory.");
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
    rcl_interfaces::msg::ParameterDescriptor debug_descriptor;
    debug_descriptor.description = "Enable detection-stage counters at runtime without changing detection behavior.";
    declare_parameter<bool>("debug_detection", false, debug_descriptor);
    debug_period_ = parameter("debug_detection_period_sec", 2.0,
      "Detection diagnostic summary interval; configured at startup.");
    const int grid_depth = parameter("map_qos_depth", 1, "Grid history depth.");
    const int sensor_depth = parameter("sensor_qos_depth", 5, "Sensor subscription history depth.");
    const bool map_latched = parameter("map_input_transient_local", true, "Receive the retained base map.");
    const bool map_reliable = parameter("map_input_reliable", true, "Request reliable base maps.");
    const bool output_latched = parameter("map_output_transient_local", true, "Retain the obstacle grid.");
    const bool output_reliable = parameter("map_output_reliable", true, "Publish reliable obstacle grids.");
    const bool sensor_reliable = parameter("sensor_input_reliable", false, "Request reliable sensor data instead of best effort.");
    const int control_depth = parameter("cloud_control_qos_depth", 1, "Cloud control topic history depth.");
    const bool control_latched = parameter("cloud_control_transient_local", true, "Receive retained cloud control messages.");
    const bool control_reliable = parameter("cloud_control_reliable", true, "Request reliable cloud control messages.");
    if (!positive(period) || !positive(obstacle_timeout_) || !nonnegative(sensor_max_age_) ||
      !nonnegative(future_tolerance_) || !nonnegative(tf_timeout_) || !nonnegative(tf_max_age_) ||
      !nonnegative(static_ignore_radius_) || !nonnegative(self_ignore_radius_) ||
      !nonnegative(laser_min_range_) || !positive(pose_jump_translation_) || !positive(pose_jump_rotation_) ||
      cloud_downsampling_ < 1 || cloud_downsampling2_ < 1 || lidar_downsampling_ < 1 ||
      cloud_min_points_ < 1 || cloud_min_neighbors_ < 0 || cloud_min_neighbors_ > 8 ||
      log_throttle_ms_ < 1 || !positive(debug_period_) || grid_depth < 1 || sensor_depth < 1 ||
      map_frame_.empty() || base_link_.empty() || odom_frame_.empty() ||
      !nonnegative(cloud_yaw_min_projection_) || cloud_yaw_min_projection_ >= 1.0 || control_depth < 1)
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
    cloud_qos_ = sensor_qos;

    pub_obstacles_ = create_publisher<Grid>(endpoint("obstacles_map_topic", "/obstacles_map"), output_qos);
    sub_base_ = create_subscription<Grid>(
      endpoint("base_map_topic", "/map_augmenter/base_map"), map_qos,
      [this](Grid::ConstSharedPtr msg) {on_base_map(*msg);});
    cloud_topic1_ = parameter("point_cloud_topic", std::string("/point_cloud"), "Camera input 1 PointCloud2 topic.");
    cloud_topic2_ = parameter("point_cloud_topic2", std::string("/point_cloud2"), "Optional camera input 2 PointCloud2 topic.");
    const auto laser_topic = parameter("laser_scan_topic", std::string("/scan"), "Hokuyo or other LaserScan topic.");
    const auto navigation_topic = endpoint("cloud_navigation_enable_topic", "/map_augmenter/enable");
    const auto cloud_enable_topic = endpoint("cloud_enable_topic", "/map_augmenter/enable_cloud");
    if (cloud_gating_enabled_) {
      auto control_qos = rclcpp::QoS(control_depth);
      if (control_latched) {control_qos.transient_local();}
      if (control_reliable) {control_qos.reliable();} else {control_qos.best_effort();}
      sub_navigation_enable_ = create_subscription<std_msgs::msg::Bool>(navigation_topic, control_qos,
        [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
          navigation_active_ = msg->data;
          update_cloud_subscriptions();
        });
      sub_cloud_enable_ = create_subscription<std_msgs::msg::Bool>(cloud_enable_topic, control_qos,
        [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
          cloud_override_enabled_ = msg->data;
          update_cloud_subscriptions();
        });
    }
    update_cloud_subscriptions();
    if (use_lidar_) {
      sub_scan_ = create_subscription<Scan>(laser_topic, sensor_qos,
        [this](Scan::ConstSharedPtr msg) {
          if (diagnostics_enabled_) {record_received(lidar_diagnostics_, msg->header.stamp, bool(latest_scan_));}
          latest_scan_ = std::move(msg);
        });
    }
    srv_clear_memory_ = create_service<std_srvs::srv::Trigger>(
      endpoint("clear_obstacle_memory_service", "/map_augmenter/clear_obstacle_memory"),
      [this](std_srvs::srv::Trigger::Request::SharedPtr,
      std_srvs::srv::Trigger::Response::SharedPtr response) {
        const auto count = memory_cells_.size();
        clear_obstacle_memory();
        if (have_grid_) {publish_obstacles();}
        response->success = true;
        response->message = "Cleared " + std::to_string(count) +
          " remembered cells; live detections remain until they expire.";
      });

    // Sensor, map and timer callbacks share the default mutually-exclusive group.
    // A frame is consumed once, including when it is rejected for age or bad TF.
    processing_timer_ = create_wall_timer(std::chrono::duration<double>(period), [this]() {processing();});
    RCLCPP_INFO(get_logger(), "Obstacle detector ready; debug_detection is runtime-switchable; other configuration is startup-only");
  }

private:
  struct AgeRange
  {
    uint64_t samples{0};
    double minimum{std::numeric_limits<double>::infinity()};
    double maximum{-std::numeric_limits<double>::infinity()};

    void record(double seconds)
    {
      ++samples;
      minimum = std::min(minimum, seconds);
      maximum = std::max(maximum, seconds);
    }

    std::string milliseconds() const
    {
      if (samples == 0) {return "n/a";}
      std::ostringstream text;
      text << std::fixed << std::setprecision(1) << minimum * 1000.0 << ".." << maximum * 1000.0;
      return text.str();
    }
  };

  struct SensorDiagnostics
  {
    // Frame counts and reception/processing delays.
    uint64_t received{0}, overwritten{0}, processed{0}, empty{0}, malformed{0};
    uint64_t zero_stamp{0}, before_reset{0}, too_old{0}, future_stamp{0}, stamp_ok{0};
    uint64_t tf_ok{0}, sensor_tf_rejected{0}, map_tf_rejected{0};
    // Transform lookup counts (up to two transforms per accepted frame).
    uint64_t exact_tf_failed{0}, fallback_used{0}, fallback_stale{0}, fallback_failed{0};
    // Point counts, except evidence/density/neighbor fields, which count cells.
    uint64_t sampled{0}, finite{0}, range_rejected{0}, crop_hits{0}, self_rejected{0};
    uint64_t outside_map{0}, on_map{0}, evidence_cells{0}, density_rejected{0};
    uint64_t neighbors_rejected{0}, density_ok{0}, static_rejected{0}, marked{0}, new_cells{0};
    AgeRange receive_age, processing_age;
    double processing_ms{0.0}, maximum_processing_ms{0.0}, maximum_fallback_delta_ms{0.0};
  };

  // Measure complete frame processing, including every early-return path.
  struct FrameTimer
  {
    SensorDiagnostics * stats;
    SteadyClock::time_point started;

    explicit FrameTimer(SensorDiagnostics * diagnostics) : stats(diagnostics)
    {
      if (stats) {
        ++stats->processed;
        started = SteadyClock::now();
      }
    }

    ~FrameTimer()
    {
      if (stats) {
        const double ms = std::chrono::duration<double, std::milli>(SteadyClock::now() - started).count();
        stats->processing_ms += ms;
        stats->maximum_processing_ms = std::max(stats->maximum_processing_ms, ms);
      }
    }
  };

  void record_received(SensorDiagnostics & stats,
    const builtin_interfaces::msg::Time & stamp, bool overwritten)
  {
    ++stats.received;
    if (overwritten) {++stats.overwritten;}
    if (stamp.sec != 0 || stamp.nanosec != 0) {
      stats.receive_age.record((now() - rclcpp::Time(stamp, get_clock()->get_clock_type())).seconds());
    }
  }

  void reset_diagnostics()
  {
    cloud1_diagnostics_ = SensorDiagnostics{};
    cloud2_diagnostics_ = SensorDiagnostics{};
    lidar_diagnostics_ = SensorDiagnostics{};
    diagnostic_cycles_ = diagnostic_no_grid_ = diagnostic_expired_before_ = diagnostic_expired_after_ = 0;
    diagnostic_pose_resets_ = diagnostic_clock_resets_ = diagnostic_cleared_cells_ = 0;
    diagnostic_processing_ms_ = diagnostic_maximum_processing_ms_ = 0.0;
    diagnostics_window_ = SteadyClock::now();
  }

  void refresh_diagnostics()
  {
    const bool enabled = get_parameter("debug_detection").as_bool();
    if (enabled == diagnostics_enabled_) {return;}
    diagnostics_enabled_ = enabled;
    reset_diagnostics();
    if (enabled) {
      RCLCPP_INFO(get_logger(),
        "Detection diagnostics enabled: sensor_age=[%.3f, %.3f]s TF_fallback_max=%.3fs obstacle_lifetime=%.3fs; counters reset each %.2fs",
        -future_tolerance_, sensor_max_age_, tf_max_age_, obstacle_timeout_, debug_period_);
    } else {
      RCLCPP_INFO(get_logger(), "Detection diagnostics disabled");
    }
  }

  void log_sensor_diagnostics(const char * name, const SensorDiagnostics & s)
  {
    std::ostringstream text;
    text << std::fixed << std::setprecision(1)
         << "Detection[" << name << "] frames{rx=" << s.received << " overwritten=" << s.overwritten
         << " processed=" << s.processed << " empty=" << s.empty << " malformed=" << s.malformed
         << " zero_stamp=" << s.zero_stamp << " before_reset=" << s.before_reset
         << " too_old=" << s.too_old << " future=" << s.future_stamp << " age_ok=" << s.stamp_ok
         << " tf_ok=" << s.tf_ok << " sensor_tf_rejected=" << s.sensor_tf_rejected
         << " map_tf_rejected=" << s.map_tf_rejected << "}"
         << " age_ms{receive=" << s.receive_age.milliseconds() << " processing=" << s.processing_age.milliseconds() << "}"
         << " tf{exact_failed=" << s.exact_tf_failed << " fallback_used=" << s.fallback_used
         << " stale=" << s.fallback_stale << " failed=" << s.fallback_failed
         << " max_delta_ms=" << s.maximum_fallback_delta_ms << "}"
         << " points{sampled=" << s.sampled << " finite=" << s.finite << " range_rejected=" << s.range_rejected
         << " crop=" << s.crop_hits << " self_rejected=" << s.self_rejected
         << " outside_map=" << s.outside_map << " on_map=" << s.on_map << "}"
         << " cells{evidence=" << s.evidence_cells << " density_rejected=" << s.density_rejected
         << " neighbors_rejected=" << s.neighbors_rejected << " density_ok=" << s.density_ok << "}"
         << " writes{static_rejected=" << s.static_rejected << " marked=" << s.marked << " new_cells=" << s.new_cells << "}"
         << " frame_ms{avg=" << (s.processed ? s.processing_ms / s.processed : 0.0)
         << " max=" << s.maximum_processing_ms << "}";
    RCLCPP_INFO(get_logger(), "%s", text.str().c_str());
  }

  void log_diagnostics()
  {
    if (!diagnostics_enabled_) {return;}
    const double seconds = std::chrono::duration<double>(SteadyClock::now() - diagnostics_window_).count();
    if (seconds < debug_period_) {return;}
    const size_t occupied = std::count_if(obstacles_map_.data.begin(), obstacles_map_.data.end(),
      [](int8_t value) {return value > 0;});
    std::ostringstream text;
    text << std::fixed << std::setprecision(1)
         << "Detection[summary] window_s=" << seconds << " grid_ready=" << have_grid_
         << " published=" << diagnostic_cycles_ << " observed_hz=" << diagnostic_cycles_ / seconds
         << " no_grid_cycles=" << diagnostic_no_grid_ << " occupied_now=" << occupied
         << " expired_before=" << diagnostic_expired_before_ << " expired_after=" << diagnostic_expired_after_
         << " pose_resets=" << diagnostic_pose_resets_ << " clock_resets=" << diagnostic_clock_resets_
         << " cleared_cells=" << diagnostic_cleared_cells_
         << " cycle_ms{avg=" << (diagnostic_cycles_ ? diagnostic_processing_ms_ / diagnostic_cycles_ : 0.0)
         << " max=" << diagnostic_maximum_processing_ms_ << "}";
    RCLCPP_INFO(get_logger(), "%s", text.str().c_str());
    if (use_lidar_) {log_sensor_diagnostics("lidar", lidar_diagnostics_);}
    if (use_cloud_) {log_sensor_diagnostics("cloud1", cloud1_diagnostics_);}
    if (use_cloud2_) {log_sensor_diagnostics("cloud2", cloud2_diagnostics_);}
    reset_diagnostics();
  }

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

  bool sensor_stamp(const builtin_interfaces::msg::Time & header, rclcpp::Time & stamp,
    SensorDiagnostics * stats)
  {
    stamp = rclcpp::Time(header, get_clock()->get_clock_type());
    if (stamp.nanoseconds() == 0) {
      if (!allow_zero_stamp_) {
        if (stats) {++stats->zero_stamp;}
        return false;
      }
      stamp = now();
    }
    if (stamp.nanoseconds() <= reset_before_ns_) {
      if (stats) {++stats->before_reset;}
      return false;
    }
    const double age = (now() - stamp).seconds();
    if (stats) {
      stats->processing_age.record(age);
      if (age > sensor_max_age_) {++stats->too_old;}
      else if (age < -future_tolerance_) {++stats->future_stamp;}
      else {++stats->stamp_ok;}
    }
    return age <= sensor_max_age_ && age >= -future_tolerance_;
  }

  bool lookup_transform(const std::string & target, const std::string & source,
    const rclcpp::Time & stamp, Eigen::Affine3d & result, SensorDiagnostics * stats)
  {
    if (source.empty()) {return false;}
    try {
      const auto tf = tf_buffer_.lookupTransform(target, source, stamp,
        rclcpp::Duration::from_seconds(tf_timeout_));
      result = tf2::transformToEigen(tf.transform);
      return true;
    } catch (const tf2::TransformException &) {
      if (stats) {++stats->exact_tf_failed;}
      if (!allow_tf_fallback_) {return false;}
    }
    try {
      const auto tf = tf_buffer_.lookupTransform(target, source,
        rclcpp::Time(0, 0, get_clock()->get_clock_type()),
        rclcpp::Duration::from_seconds(tf_timeout_));
      const rclcpp::Time tf_stamp(tf.header.stamp, get_clock()->get_clock_type());
      if (stats && tf_stamp.nanoseconds() != 0) {
        stats->maximum_fallback_delta_ms = std::max(stats->maximum_fallback_delta_ms,
          std::abs((stamp - tf_stamp).seconds()) * 1000.0);
      }
      // A zero timestamp is allowed for a static transform.
      if (tf_stamp.nanoseconds() != 0 && std::abs((stamp - tf_stamp).seconds()) > tf_max_age_) {
        if (stats) {++stats->fallback_stale;}
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), log_throttle_ms_,
          "Rejected stale fallback TF from '%s' to '%s'", source.c_str(), target.c_str());
        return false;
      }
      result = tf2::transformToEigen(tf.transform);
      if (stats) {++stats->fallback_used;}
      return true;
    } catch (const tf2::TransformException & error) {
      if (stats) {++stats->fallback_failed;}
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), log_throttle_ms_, "TF unavailable: %s", error.what());
      return false;
    }
  }

  void mark_obstacle(int x, int y, const rclcpp::Time & stamp, SensorDiagnostics * stats)
  {
    const size_t index = map_augmenter::cell_index(obstacles_map_, x, y);
    if (ignore_static_ && static_ignore_mask_.data[index] > 0) {
      if (stats) {++stats->static_rejected;}
      return;
    }
    if (stats) {
      ++stats->marked;
      if (obstacles_map_.data[index] <= 0) {++stats->new_cells;}
    }
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

  void add_cloud(const Cloud & cloud, int stride, SensorDiagnostics * stats)
  {
    FrameTimer frame_timer(stats);
    if (cloud.width == 0 || cloud.height == 0) {
      if (stats) {++stats->empty;}
      return;
    }
    const auto * fx = coordinate_field(cloud, "x");
    const auto * fy = coordinate_field(cloud, "y");
    const auto * fz = coordinate_field(cloud, "z");
    const uint64_t minimum_row = static_cast<uint64_t>(cloud.width) * cloud.point_step;
    const uint64_t required = static_cast<uint64_t>(cloud.row_step) * cloud.height;
    if (!fx || !fy || !fz || cloud.row_step < minimum_row || required > cloud.data.size()) {
      if (stats) {++stats->malformed;}
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), log_throttle_ms_, "Rejected malformed PointCloud2 layout");
      return;
    }
    rclcpp::Time stamp(0, 0, get_clock()->get_clock_type());
    if (!sensor_stamp(cloud.header.stamp, stamp, stats)) {return;}
    Eigen::Affine3d sensor_to_base, base_to_map;
    if (!lookup_transform(base_link_, cloud.header.frame_id, stamp, sensor_to_base, stats)) {
      if (stats) {++stats->sensor_tf_rejected;}
      return;
    }
    if (!lookup_transform(map_frame_, base_link_, stamp, base_to_map, stats)) {
      if (stats) {++stats->map_tf_rejected;}
      return;
    }
    if (stats) {++stats->tf_ok;}

    std::unordered_map<size_t, CellEvidence> hits;
    const size_t total = static_cast<size_t>(cloud.width) * cloud.height;
    for (size_t i = 0; i < total; i += static_cast<size_t>(stride)) {
      if (stats) {++stats->sampled;}
      // Respect padding between rows in organised Xtion clouds.
      const auto * p = cloud.data.data() + (i / cloud.width) * cloud.row_step +
        (i % cloud.width) * cloud.point_step;
      const Eigen::Vector3d point(
        read_coordinate(p, *fx, cloud.is_bigendian),
        read_coordinate(p, *fy, cloud.is_bigendian),
        read_coordinate(p, *fz, cloud.is_bigendian));
      if (!point.allFinite()) {continue;}
      if (stats) {++stats->finite;}
      const Eigen::Vector3d base_point = sensor_to_base * point;
      if (!cloud_box_.contains(base_point)) {continue;}
      if (stats) {++stats->crop_hits;}
      if (base_point.head<2>().norm() < self_ignore_radius_) {
        if (stats) {++stats->self_rejected;}
        continue;
      }
      const Eigen::Vector3d map_point = base_to_map * base_point;
      int x, y;
      if (!map_augmenter::world_to_cell(obstacles_map_, map_point.x(), map_point.y(), x, y)) {
        if (stats) {++stats->outside_map;}
        continue;
      }
      if (stats) {++stats->on_map;}
      auto & hit = hits[map_augmenter::cell_index(obstacles_map_, x, y)];
      ++hit.count;
      hit.x = x;
      hit.y = y;
    }
    if (stats) {stats->evidence_cells += hits.size();}
    for (const auto & entry : hits) {
      const auto & hit = entry.second;
      if (hit.count < static_cast<size_t>(cloud_min_points_)) {
        if (stats) {++stats->density_rejected;}
        continue;
      }
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
      if (neighbors >= cloud_min_neighbors_) {
        if (stats) {++stats->density_ok;}
        mark_obstacle(hit.x, hit.y, stamp, stats);
      } else if (stats) {
        ++stats->neighbors_rejected;
      }
    }
  }

  void add_scan(const Scan & scan, SensorDiagnostics * stats)
  {
    FrameTimer frame_timer(stats);
    if (!std::isfinite(scan.range_min) || !std::isfinite(scan.range_max) ||
      scan.range_min < 0.0f || scan.range_max < scan.range_min ||
      !std::isfinite(scan.angle_min) || !std::isfinite(scan.angle_increment))
    {
      if (stats) {++stats->malformed;}
      return;
    }
    rclcpp::Time stamp(0, 0, get_clock()->get_clock_type());
    if (!sensor_stamp(scan.header.stamp, stamp, stats)) {return;}
    Eigen::Affine3d sensor_to_base, base_to_map;
    if (!lookup_transform(base_link_, scan.header.frame_id, stamp, sensor_to_base, stats)) {
      if (stats) {++stats->sensor_tf_rejected;}
      return;
    }
    if (!lookup_transform(map_frame_, base_link_, stamp, base_to_map, stats)) {
      if (stats) {++stats->map_tf_rejected;}
      return;
    }
    if (stats) {++stats->tf_ok;}
    for (size_t i = 0; i < scan.ranges.size(); i += static_cast<size_t>(lidar_downsampling_)) {
      if (stats) {++stats->sampled;}
      const double range = scan.ranges[i];
      if (!std::isfinite(range) || range < scan.range_min ||
        range > scan.range_max || range < laser_min_range_)
      {
        if (stats) {++stats->range_rejected;}
        continue;
      }
      if (stats) {++stats->finite;}
      const double angle = scan.angle_min + i * scan.angle_increment;
      const Eigen::Vector3d base_point = sensor_to_base *
        Eigen::Vector3d(range * std::cos(angle), range * std::sin(angle), 0.0);
      if (!laser_box_.contains(base_point)) {continue;}
      if (stats) {++stats->crop_hits;}
      if (base_point.head<2>().norm() < self_ignore_radius_) {
        if (stats) {++stats->self_rejected;}
        continue;
      }
      const Eigen::Vector3d point = base_to_map * base_point;
      int x, y;
      if (map_augmenter::world_to_cell(obstacles_map_, point.x(), point.y(), x, y)) {
        if (stats) {++stats->on_map;}
        mark_obstacle(x, y, stamp, stats);
      } else if (stats) {
        ++stats->outside_map;
      }
    }
  }

  void clear_transient_obstacles()
  {
    if (diagnostics_enabled_) {
      diagnostic_cleared_cells_ += std::count_if(obstacles_map_.data.begin(), obstacles_map_.data.end(),
        [](int8_t value) {return value > 0;});
    }
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
          if (diagnostics_enabled_) {++diagnostic_pose_resets_;}
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
    refresh_diagnostics();
    if (!have_grid_) {
      if (diagnostics_enabled_) {++diagnostic_no_grid_; log_diagnostics();}
      return;
    }
    const auto processing_started = diagnostics_enabled_ ? SteadyClock::now() : SteadyClock::time_point{};
    const auto current = now();
    const int64_t current_ns = current.nanoseconds();
    if (previous_processing_ns_ != never_observed && current_ns < previous_processing_ns_) {
      if (diagnostics_enabled_) {++diagnostic_clock_resets_;}
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
        if (diagnostics_enabled_) {++diagnostic_expired_before_;}
        obstacles_map_.data[i] = 0;
        last_observed_ns_[i] = never_observed;
      }
    }

    // Integrate the lightweight scan before camera processing can age it out.
    if (auto scan = std::exchange(latest_scan_, nullptr)) {
      add_scan(*scan, diagnostics_enabled_ ? &lidar_diagnostics_ : nullptr);
    }
    if (auto cloud = std::exchange(latest_cloud1_, nullptr)) {
      add_cloud(*cloud, cloud_downsampling_, diagnostics_enabled_ ? &cloud1_diagnostics_ : nullptr);
    }
    if (auto cloud = std::exchange(latest_cloud2_, nullptr)) {
      add_cloud(*cloud, cloud_downsampling2_, diagnostics_enabled_ ? &cloud2_diagnostics_ : nullptr);
    }
    // Late, but still acceptable sensor frames must not resurrect already expired evidence.
    const int64_t publish_ns = now().nanoseconds();
    for (size_t i = 0; i < obstacles_map_.data.size(); ++i) {
      const int64_t observed = last_observed_ns_[i];
      if (obstacles_map_.data[i] > 0 && observed != never_observed &&
        publish_ns >= observed && publish_ns - observed >= lifetime)
      {
        if (diagnostics_enabled_) {++diagnostic_expired_after_;}
        obstacles_map_.data[i] = 0;
        last_observed_ns_[i] = never_observed;
      }
    }
    obstacles_map_.header.stamp = now();
    pub_obstacles_->publish(obstacles_map_);
    if (diagnostics_enabled_) {
      ++diagnostic_cycles_;
      const double ms = std::chrono::duration<double, std::milli>(SteadyClock::now() - processing_started).count();
      diagnostic_processing_ms_ += ms;
      diagnostic_maximum_processing_ms_ = std::max(diagnostic_maximum_processing_ms_, ms);
      log_diagnostics();
    }
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
  bool diagnostics_enabled_{false};
  double debug_period_;
  SteadyClock::time_point diagnostics_window_;
  SensorDiagnostics cloud1_diagnostics_, cloud2_diagnostics_, lidar_diagnostics_;
  uint64_t diagnostic_cycles_{0}, diagnostic_no_grid_{0};
  uint64_t diagnostic_expired_before_{0}, diagnostic_expired_after_{0};
  uint64_t diagnostic_pose_resets_{0}, diagnostic_clock_resets_{0}, diagnostic_cleared_cells_{0};
  double diagnostic_processing_ms_{0.0}, diagnostic_maximum_processing_ms_{0.0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstacleDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
