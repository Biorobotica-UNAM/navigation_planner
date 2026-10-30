// ----- ROS 2 -----
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>

// ----- ROS 2 messages -----
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <opencv2/highgui.hpp>

// ----- C++ standard library -----
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <string>
#include <sstream>
#include <functional>
#include <memory>
#include <mutex>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

class PotentialFieldsNode : public rclcpp::Node
{
  using Clock = std::chrono::steady_clock;
  struct SensorState
  {
    double min_x, max_x, min_y, max_y, min_z, max_z, collision_min_y, collision_max_y, radius, gain;
    int stride, threshold;
    bool risk = false;
    int critical_points = 0;
    double speed_limit = std::numeric_limits<double>::infinity();
    double clearance = std::numeric_limits<double>::infinity();
    geometry_msgs::msg::Vector3 force;
    Clock::time_point received, last_message, measurement_time;
    std::string rejection = "no_messages";
    cv::Mat image;
  };

public:
  PotentialFieldsNode()
  : Node("potential_fields_node"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    rcl_interfaces::msg::ParameterDescriptor startup;
    startup.read_only = true;startup.description = "Configured at startup.";
    const bool namespaced = declare_parameter<bool>("use_namespace", false, startup);
    base_frame_ = declare_parameter<std::string>("base_link_name", "base_link", startup);
    declare_parameter<bool>("debug", false);
    declare_parameter<bool>("show_image", false);
    declare_parameter<bool>("use_pot_fields", false);
    use_lidar_ = declare_parameter<bool>("use_lidar", true, startup);
    use_cloud_ = declare_parameter<bool>("use_point_cloud", false, startup);
    laser_topic_ = declare_parameter<std::string>("laser_scan_topic", "/scan", startup);
    cloud_topic_ = declare_parameter<std::string>("point_cloud_topic", "/points", startup);
    timeout_ = declare_parameter<double>("no_sensor_data_timeout", 0.5, startup);
    future_tolerance_ = declare_parameter<double>("sensor_future_tolerance_sec", 0.05, startup);
    tf_timeout_ = declare_parameter<double>("tf_timeout_sec", 0.03, startup);
    allow_tf_fallback_ = declare_parameter<bool>("tf_allow_latest_fallback", true, startup);
    tf_max_age_ = declare_parameter<double>("tf_fallback_max_age_sec", 0.2, startup);
    footprint_ = declare_parameter<double>("collision_footprint_radius", 0.30, startup);
    margin_ = declare_parameter<double>("collision_stop_margin", 0.08, startup);
    reaction_time_ = declare_parameter<double>("collision_reaction_time_sec", 0.10, startup);
    critical_margin_ = declare_parameter<double>("collision_critical_margin", 0.02, startup);
    deceleration_ = declare_parameter<double>("collision_deceleration", 0.3, startup);
    max_force_ = declare_parameter<double>("repulsion_max_speed", 0.15, startup);
    filter_time_ = declare_parameter<double>("repulsion_filter_time_sec", 0.15, startup);
    if (!std::isfinite(reaction_time_) || reaction_time_ <= 0.0 ||
      !std::isfinite(critical_margin_) || critical_margin_ < 0.0 || critical_margin_ > margin_ ||
      !std::isfinite(future_tolerance_) || future_tolerance_ < 0.0 ||
      !std::isfinite(tf_timeout_) || tf_timeout_ < 0.0 ||
      !std::isfinite(tf_max_age_) || tf_max_age_ < 0.0 ||
      !std::isfinite(timeout_) || timeout_ <= 0 || !std::isfinite(footprint_) ||
      footprint_ <= 0 ||
      !std::isfinite(margin_) || margin_ < 0 || !std::isfinite(deceleration_) ||
      deceleration_ <= 0 ||
      !std::isfinite(max_force_) || max_force_ <= 0 || !std::isfinite(filter_time_) ||
      filter_time_ <= 0)
    {
      throw std::invalid_argument("Invalid potential field limits");
    }
    for (size_t i = 0; i < 2; ++i) {
      const std::string prefix = i == 0 ? "laser" : "cloud";
      auto & sensor = sensors_[i];
      sensor.min_x = declare_parameter<double>(prefix + "_min_x", 0.30, startup);
      sensor.max_x = declare_parameter<double>(prefix + "_max_x", 0.80, startup);
      sensor.min_y = declare_parameter<double>(prefix + "_min_y", -0.30, startup);
      sensor.max_y = declare_parameter<double>(prefix + "_max_y", 0.30, startup);
      sensor.min_z = declare_parameter<double>(prefix + "_min_z", 0.05, startup);
      sensor.max_z = declare_parameter<double>(prefix + "_max_z", 1.50, startup);
      sensor.collision_min_y =
        declare_parameter<double>(prefix + "_collision_min_y", -0.25, startup);
      sensor.collision_max_y =
        declare_parameter<double>(prefix + "_collision_max_y", 0.25, startup);
      sensor.radius = declare_parameter<double>(prefix + "_pot_fields_d0", 1.0, startup);
      sensor.gain = declare_parameter<double>(prefix + "_pot_fields_k_rej", 1.0, startup);
      sensor.stride = declare_parameter<int>(
        i == 0 ? "lidar_downsampling" : "cloud_downsampling",
        1, startup);
      sensor.threshold = declare_parameter<int>(
        i == 0 ? "lidar_points_threshold" : "cloud_points_threshold", i == 0 ? 10 : 100, startup);
      const std::array<double, 10> values{sensor.min_x, sensor.max_x, sensor.min_y, sensor.max_y,
        sensor.min_z, sensor.max_z, sensor.collision_min_y, sensor.collision_max_y, sensor.radius,
        sensor.gain};
      if (!std::all_of(values.begin(), values.end(), [](double v) {return std::isfinite(v);}) ||
        sensor.min_x >= sensor.max_x || sensor.min_y >= sensor.max_y ||
        sensor.min_z >= sensor.max_z ||
        sensor.collision_min_y >= sensor.collision_max_y || sensor.radius <= 0 || sensor.gain < 0 ||
        sensor.stride < 1 || sensor.threshold < 0)
      {
        throw std::invalid_argument("Invalid " + prefix + " limits");
      }
    }
    std::string prefix = namespaced ? get_namespace() : "";
    if (prefix == "/") {prefix.clear();}
    risk_pub_ = create_publisher<std_msgs::msg::Bool>(
      prefix + "/potential_fields/collision_risk",
      10);
    speed_limit_pub_ = create_publisher<std_msgs::msg::Float32>(
      prefix + "/potential_fields/speed_limit", 10);
    ready_pub_ = create_publisher<std_msgs::msg::Bool>(prefix + "/potential_fields/ready", 10);
    force_pub_ = create_publisher<geometry_msgs::msg::Vector3>(
      prefix + "/potential_fields/pf_rejection_force", 10);
    markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      prefix + "/potential_fields/pot_field_markers", 10);
    intent_sub_ = create_subscription<geometry_msgs::msg::Vector3>(
      prefix + "/simple_move/motion_direction", 10,
      [this](geometry_msgs::msg::Vector3::ConstSharedPtr intent) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        const double norm = std::hypot(intent->x, intent->y);
        if (std::isfinite(norm) && norm > 1e-6) {direction_ = std::atan2(intent->y, intent->x);}
      });
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", rclcpp::SensorDataQoS(),
      [this](geometry_msgs::msg::Twist::ConstSharedPtr command) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        speed_ = std::hypot(command->linear.x, command->linear.y); angular_ = command->angular.z;
      });
    enable_sub_ = create_subscription<std_msgs::msg::Bool>(
      prefix + "/potential_fields/enable", rclcpp::QoS(1),
      [this](std_msgs::msg::Bool::ConstSharedPtr message) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (enabled_ == message->data) {return;}
        ++cloud_generation_;
        pending_cloud_.reset();
        enabled_ = message->data;
        RCLCPP_INFO(get_logger(), "Obstacle monitor %s", enabled_ ? "enabled" : "disabled");
        sensors_[0].received = sensors_[1].received = Clock::time_point{};
        filtered_ = geometry_msgs::msg::Vector3();
        if (!enabled_) {
          laser_sub_.reset();cloud_sub_.reset();ready_pub_->publish(std_msgs::msg::Bool());return;
        }
        if (use_lidar_) {
          laser_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            laser_topic_, rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::LaserScan::ConstSharedPtr scan) {
              auto & sensor = sensors_[0];
              Eigen::Affine3d tf;
              if (!prepare_observation(scan->header, sensor, tf)) {return;}
              if (scan->ranges.empty() || !std::isfinite(scan->angle_min) ||
              !std::isfinite(scan->angle_increment) || !std::isfinite(scan->range_min) ||
              !std::isfinite(scan->range_max) || scan->range_max <= scan->range_min)
              {
                sensor.rejection = "invalid_laser_scan";return;
              }
              std::vector<Eigen::Vector3d> points;
              for (size_t i = 0; i < scan->ranges.size(); i += sensors_[0].stride) {
                const double range = scan->ranges[i],
                angle = scan->angle_min + i * scan->angle_increment;
                if (std::isfinite(range) &&
                range >= std::max(0.10f, scan->range_min) && range <= scan->range_max)
                {
                  points.push_back(
                    tf *
                    Eigen::Vector3d(range * std::cos(angle), range * std::sin(angle), 0.0));
                }
              }
              evaluate_points(points, sensors_[0], direction_, angular_);
            });
        }
        if (use_cloud_ && cloud_enabled_) {subscribe_cloud();}
      });
    cloud_enable_sub_ = create_subscription<std_msgs::msg::Bool>(
      prefix + "/potential_fields/enable_cloud", rclcpp::QoS(1),
      [this](std_msgs::msg::Bool::ConstSharedPtr message) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        ++cloud_generation_;
        pending_cloud_.reset();
        cloud_enabled_ = message->data;
        if (!cloud_enabled_) {
          cloud_sub_.reset();sensors_[1].force = geometry_msgs::msg::Vector3();
        } else if (enabled_ && use_cloud_ && !cloud_sub_) {subscribe_cloud();}
      });
    RCLCPP_INFO(
      get_logger(), "Obstacle monitor: lidar=%s topic=%s; cloud=%s topic=%s; frame=%s; waiting for enable on %s/potential_fields/enable",
      use_lidar_ ? "on" : "off", laser_topic_.c_str(), use_cloud_ ? "on" : "off",
      cloud_topic_.c_str(), base_frame_.c_str(), prefix.c_str());
    cloud_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    cloud_timer_ = create_wall_timer(
      std::chrono::milliseconds(30),
      std::bind(&PotentialFieldsNode::process_cloud, this), cloud_group_);
    timer_ =
      create_wall_timer(
      std::chrono::milliseconds(30),
      std::bind(&PotentialFieldsNode::publish_fields, this));
  }

private:
  bool prepare_observation(
    const std_msgs::msg::Header & header, SensorState & sensor,
    Eigen::Affine3d & result)
  {
    sensor.last_message = Clock::now();
    const rclcpp::Time stamp(header.stamp, get_clock()->get_clock_type());
    const double age = (now() - stamp).seconds();
    if (header.frame_id.empty()) {sensor.rejection = "empty_sensor_frame";return false;}
    if (stamp.nanoseconds() == 0 || age > timeout_ || age < -future_tolerance_) {
      std::ostringstream reason;
      reason << "sensor_stamp_age=" << age << "s (allowed -" << future_tolerance_ << ".." <<
        timeout_ << "s)";
      sensor.rejection = reason.str();return false;
    }
    sensor.measurement_time = Clock::now() - std::chrono::duration_cast<Clock::duration>(
      std::chrono::duration<double>(std::max(0.0, age)));
    try {
      const auto transform = tf_buffer_.lookupTransform(
        base_frame_, header.frame_id, stamp,
        rclcpp::Duration::from_seconds(tf_timeout_));
      result = tf2::transformToEigen(transform.transform);
      sensor.rejection.clear();return true;
    } catch (const tf2::TransformException & error) {
      sensor.rejection = std::string("exact_tf: ") + error.what();
      if (!allow_tf_fallback_) {return false;}
    }
    try {
      const auto transform = tf_buffer_.lookupTransform(
        base_frame_, header.frame_id,
        tf2::TimePointZero);
      const rclcpp::Time tf_stamp(transform.header.stamp, get_clock()->get_clock_type());
      const double delta = std::abs((stamp - tf_stamp).seconds());
      if (tf_stamp.nanoseconds() != 0 && delta > tf_max_age_) {
        std::ostringstream reason;
        reason << "stale_tf_fallback: " << header.frame_id << " -> " << base_frame_ << " delta=" <<
          delta << "s limit=" << tf_max_age_ << "s";
        sensor.rejection = reason.str();return false;
      }
      result = tf2::transformToEigen(transform.transform);
      sensor.rejection.clear();
      if (get_parameter("debug").as_bool()) {
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Using recent TF fallback: %s -> %s, delta=%.3f s",
          header.frame_id.c_str(), base_frame_.c_str(), delta);
      }
      return true;
    } catch (const tf2::TransformException & error) {
      sensor.rejection = std::string("tf_unavailable: ") + error.what();return false;
    }
  }

  void subscribe_cloud()
  {
    sensors_[1].received = Clock::time_point{};
    // Reception stays in the control group and only replaces the latest frame.
    // Dense point clouds must never hold up LiDAR or the safety heartbeat.
    const auto generation = cloud_generation_;
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_,
      rclcpp::SensorDataQoS(),
      [this, generation](sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (generation != cloud_generation_ || !enabled_ || !cloud_enabled_) {return;}
        pending_cloud_ = cloud;
        sensors_[1].last_message = Clock::now();
      });
  }

  void process_cloud()
  {
    sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud;
    SensorState sensor;
    uint64_t generation;
    double direction, angular;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (!enabled_ || !cloud_enabled_ || !pending_cloud_) {return;}
      cloud = std::move(pending_cloud_);
      generation = cloud_generation_;
      sensor = sensors_[1];
      direction = direction_;angular = angular_;
    }
    const auto started = Clock::now();
    const auto previous_valid = sensor.received;
    const auto previous_force = sensor.force;
    const bool previous_risk = sensor.risk;
    const double previous_limit = sensor.speed_limit;
    // Keep early validation returns local so every rejection is committed too.
    const auto process = [&]() {

        Eigen::Affine3d tf;
        if (!prepare_observation(cloud->header, sensor, tf)) {return;}
        if (cloud->width == 0 || cloud->height == 0 || cloud->point_step == 0 ||
          cloud->row_step < static_cast<uint64_t>(cloud->width) * cloud->point_step ||
          cloud->data.size() < static_cast<uint64_t>(cloud->row_step) * cloud->height)
        {
          sensor.rejection = "invalid_cloud_layout";return;
        }
        std::array<uint32_t, 3> offsets{};
        for (size_t i = 0; i < 3; ++i) {
          const std::string name = i == 0 ? "x" : (i == 1 ? "y" : "z");
          const auto field =
            std::find_if(
            cloud->fields.begin(), cloud->fields.end(), [&name](const auto & f) {
              return f.name == name;
            });
          if (field == cloud->fields.end() ||
            field->datatype != sensor_msgs::msg::PointField::FLOAT32 ||
            field->count != 1 ||
            static_cast<uint64_t>(field->offset) + sizeof(float) > cloud->point_step)
          {
            sensor.rejection = "unsupported_cloud_field_" + name;return;
          }
          offsets[i] = field->offset;
        }
        std::vector<Eigen::Vector3d> points;
        points.reserve(
          (static_cast<uint64_t>(cloud->width) * cloud->height + sensor.stride - 1) /
          sensor.stride);
        const uint16_t endian_test = 1;
        const bool host_big = *reinterpret_cast<const uint8_t *>(&endian_test) == 0;
        for (size_t i = 0; i < static_cast<uint64_t>(cloud->width) * cloud->height;
          i += sensor.stride)
        {
          const size_t offset = (i / cloud->width) * cloud->row_step + (i % cloud->width) *
            cloud->point_step;
          std::array<float, 3> xyz;
          for (size_t axis = 0; axis < 3; ++axis) {
            std::array<uint8_t, 4> bytes;
            std::memcpy(bytes.data(), cloud->data.data() + offset + offsets[axis], 4);
            if (host_big != cloud->is_bigendian) {std::reverse(bytes.begin(), bytes.end());}
            std::memcpy(&xyz[axis], bytes.data(), 4);
          }
          const Eigen::Vector3d p(xyz[0], xyz[1], xyz[2]);
          if (p.allFinite()) {points.push_back(tf * p);}
        }
        evaluate_points(points, sensor, direction, angular);

      };
    process();
    const double elapsed = std::chrono::duration<double>(Clock::now() - started).count();
    const double age =
      (now() - rclcpp::Time(cloud->header.stamp, get_clock()->get_clock_type())).seconds();
    if (sensor.received != previous_valid && age > timeout_) {
      sensor.received = previous_valid;
      sensor.force = previous_force;sensor.risk = previous_risk;sensor.speed_limit = previous_limit;
      sensor.image.release();
      std::ostringstream reason;
      reason << "cloud_processing_overrun: duration=" << elapsed << "s sensor_age=" << age << "s";
      sensor.rejection = reason.str();
    }
    if (elapsed > std::min(0.1, timeout_ * 0.5)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Cloud processing slow: %.3f s, %llu points; LiDAR and monitor run independently",
        elapsed, static_cast<unsigned long long>(cloud->width) * cloud->height);
    }
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      // A stopped/restarted sensor must not accept an older in-flight result.
      if (generation == cloud_generation_ && enabled_ && cloud_enabled_) {
        sensor.last_message = sensors_[1].last_message;
        sensors_[1] = std::move(sensor);
      }
    }
  }

  void evaluate_points(
    const std::vector<Eigen::Vector3d> & points, SensorState & sensor,
    double direction, double angular)
  {
    int obstacles = 0, count = 0;
    sensor.force = geometry_msgs::msg::Vector3();
    const double c = std::cos(direction), s = std::sin(direction);
    const double stop_radius = footprint_ + margin_;
    const double critical_radius = footprint_ + critical_margin_;
    std::vector<double> clearances;
    clearances.reserve(points.size());
    cv::Mat image;
    if (get_parameter("show_image").as_bool()) {image = cv::Mat::zeros(300, 300, CV_8UC1);}
    for (const auto & p : points) {
      if (!p.allFinite() || p.z() < sensor.min_z || p.z() > sensor.max_z) {continue;}
      const double along = c * p.x() + s * p.y(), across = -s * p.x() + c * p.y(),
        distance = std::hypot(p.x(), p.y());
      // A swept circle uses actual lateral clearance; the full braking zone
      // limits speed instead of being interpreted as an immediate collision.
      if (distance <= 0.10) {continue;}
      if (along >= 0.0 && std::abs(across) < stop_radius) {
        clearances.push_back(
          along -
          std::sqrt(std::max(0.0, stop_radius * stop_radius - across * across)));
      }
      const bool danger = (along >= 0.0 && distance < critical_radius) ||
        (std::abs(angular) > 0.02 && distance < critical_radius);
      if (danger) {++obstacles;}
      if (distance > 0.10 && distance < sensor.radius && along > sensor.min_x &&
        along < sensor.max_x &&
        across > sensor.min_y && across < sensor.max_y)
      {
        const double force = sensor.gain *
          std::sqrt(std::max(0.0, 1.0 / distance - 1.0 / sensor.radius));
        sensor.force.x -= force * p.x() / distance;sensor.force.y -= force * p.y() / distance;
        ++count;
      }
      if (!image.empty()) {
        const int ix = 150 + static_cast<int>(across * 100),
          iy = 299 - static_cast<int>(along * 100);
        if (ix >= 0 && ix < image.cols && iy >= 0 && iy < image.rows) {
          image.at<uint8_t>(iy, ix) = danger ? 255 : 100;
        }
      }
    }
    if (count > 0) {sensor.force.x /= count;sensor.force.y /= count;}
    sensor.critical_points = obstacles;
    sensor.risk = obstacles > sensor.threshold;
    sensor.speed_limit = std::numeric_limits<double>::infinity();
    sensor.clearance = std::numeric_limits<double>::infinity();
    // Require the same amount of sensor support as collision detection; a
    // single isolated cloud point must not force an unrelated slowdown.
    if (clearances.size() > static_cast<size_t>(sensor.threshold)) {
      auto supported = clearances.begin() + sensor.threshold;
      std::nth_element(clearances.begin(), supported, clearances.end());
      sensor.clearance = std::max(0.0, *supported);
      const double age =
        std::max(
        0.0,
        std::chrono::duration<double>(Clock::now() - sensor.measurement_time).count());
      const double reaction = std::max(reaction_time_, age + 0.03);
      const double term = deceleration_ * reaction;
      sensor.speed_limit =
        std::max(0.0, std::sqrt(term * term + 2.0 * deceleration_ * sensor.clearance) - term);
    }
    sensor.received = sensor.measurement_time;
    sensor.image = std::move(image);
  }

  void publish_fields()
  {
    if (!enabled_) {return;}
    std::array<SensorState, 2> sensors;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      sensors = sensors_;
    }
    const auto tick = Clock::now();
    geometry_msgs::msg::Vector3 force;
    bool risk = false, ready = true;
    double speed_limit = std::numeric_limits<double>::infinity();
    std::ostringstream unavailable;
    for (size_t i = 0; i < 2; ++i) {
      if ((i == 0 && !use_lidar_) || (i == 1 && (!use_cloud_ || !cloud_enabled_))) {continue;}
      const auto & sensor = sensors[i];
      if (sensor.received == Clock::time_point{} ||
        std::chrono::duration<double>(tick - sensor.received).count() > timeout_)
      {
        ready = false;
        unavailable << (i == 0 ? " lidar=" : " cloud=") << (i == 0 ? laser_topic_ : cloud_topic_);
        if (sensor.last_message == Clock::time_point{}) {unavailable << " no_messages";} else {
          unavailable << " received_age=" << std::chrono::duration<double>(
            tick - sensor.last_message).count() << "s";
          if (sensor.received == Clock::time_point{}) {
            unavailable << " no_valid_observation";
          } else {
            unavailable << " valid_age=" <<
              std::chrono::duration<double>(tick - sensor.received).count() << "s";
          }
          if (!sensor.rejection.empty()) {unavailable << " reason=" << sensor.rejection;}
        }
        continue;
      }
      if (!sensor.image.empty() && get_parameter("show_image").as_bool()) {
        cv::imshow(i == 0 ? "Potential fields LiDAR" : "Potential fields cloud", sensor.image);
        cv::waitKey(1);
      }
      risk = risk || sensor.risk;
      speed_limit = std::min(speed_limit, sensor.speed_limit);
      force.x += sensor.force.x;force.y += sensor.force.y;
    }
    if (!ready) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(), 2000, "Obstacle sensors not ready:%s", unavailable.str().c_str());
    }
    const double scale = std::min(1.0, max_force_ / std::max(1e-9, std::hypot(force.x, force.y)));
    const double alpha = 1.0 - std::exp(-0.03 / filter_time_);
    filtered_.x += alpha * (force.x * scale - filtered_.x);
    filtered_.y += alpha * (force.y * scale - filtered_.y);
    if (ready && risk && !previous_risk_) {
      RCLCPP_WARN(
        get_logger(), "Critical obstacle: commanded_speed=%.3f m/s, radius=%.3f m, lidar_points=%d threshold=%d, cloud_points=%d threshold=%d; speed_limit=%.3f m/s",
        speed_, footprint_ + critical_margin_, sensors[0].critical_points, sensors[0].threshold,
        sensors[1].critical_points, sensors[1].threshold, speed_limit);
    }
    previous_risk_ = ready && risk;
    std_msgs::msg::Float32 limit;limit.data = ready ? speed_limit : 0.0;speed_limit_pub_->publish(
      limit);
    std_msgs::msg::Bool readiness;readiness.data = ready;ready_pub_->publish(readiness);
    std_msgs::msg::Bool collision;collision.data = risk;risk_pub_->publish(collision);
    force_pub_->publish(
      get_parameter(
        "use_pot_fields").as_bool() ? filtered_ : geometry_msgs::msg::Vector3());
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker arrow;
    arrow.header.frame_id = base_frame_;arrow.header.stamp = now();arrow.ns = "pot_fields";
    arrow.id = 0;
    arrow.type = visualization_msgs::msg::Marker::ARROW;
    arrow.action = visualization_msgs::msg::Marker::ADD;
    arrow.pose.orientation.w = 1.0;arrow.scale.x = 0.03;arrow.scale.y = 0.06;arrow.scale.z = 0.06;
    arrow.color.r = 1.0;arrow.color.a = 1.0;
    geometry_msgs::msg::Point end;end.x = filtered_.x;end.y = filtered_.y;
    arrow.points = {geometry_msgs::msg::Point(), end};markers.markers.push_back(arrow);
    markers_pub_->publish(markers);
  }

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::array<SensorState, 2> sensors_;
  std::mutex state_mutex_;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr pending_cloud_;
  uint64_t cloud_generation_ = 0;
  rclcpp::CallbackGroup::SharedPtr cloud_group_;
  std::string base_frame_, laser_topic_, cloud_topic_;
  bool use_lidar_, use_cloud_, enabled_ = false, cloud_enabled_ = true;
  bool allow_tf_fallback_;
  double future_tolerance_, tf_timeout_, tf_max_age_;
  bool previous_risk_ = false;
  double reaction_time_, critical_margin_;
  double timeout_, footprint_, margin_, deceleration_, max_force_, filter_time_;
  double speed_ = 0.0, angular_ = 0.0, direction_ = 0.0;
  geometry_msgs::msg::Vector3 filtered_;
  rclcpp::TimerBase::SharedPtr timer_, cloud_timer_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr speed_limit_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr risk_pub_, ready_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr force_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_, cloud_enable_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr intent_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PotentialFieldsNode>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions{}, 2);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
