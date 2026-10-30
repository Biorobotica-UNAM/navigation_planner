// ----- ROS 2 -----
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

// ----- ROS 2 messages -----
#include <actionlib_msgs/msg/goal_status.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

// ----- C++ standard library -----
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <functional>
#include <memory>
#include <stdexcept>
#include <utility>
#include <string>
#include <vector>

class SimpleMoveNode : public rclcpp::Node
{
  using Path = nav_msgs::msg::Path;
  using Status = actionlib_msgs::msg::GoalStatus;
  using Clock = std::chrono::steady_clock;

public:
  SimpleMoveNode()
  : Node("simple_move_node"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    rcl_interfaces::msg::ParameterDescriptor startup;
    startup.read_only = true;
    startup.description = "Configured at startup.";
    const bool namespaced = declare_parameter<bool>("use_namespace", false, startup);
    base_frame_ = declare_parameter<std::string>("base_link_name", "base_link", startup);
    odom_frame_ = declare_parameter<std::string>("odom_name", "odom", startup);
    declare_parameter<std::string>("motion_mode", "omni");
    declare_parameter<bool>("move_head", true);
    declare_parameter<bool>("use_pot_fields", false);
    // Existing launch arguments remain accepted for relative-motion clients.
    declare_parameter<double>("control_alpha", 0.6548, startup);
    declare_parameter<double>("control_beta", 0.2, startup);
    const std::vector<std::pair<std::string, double>> settings = {
      {"max_linear_speed", 0.3}, {"min_linear_speed", 0.05},
      {"max_angular_speed", 1.0}, {"final_angle_max_angular_speed", 0.35},
      {"linear_acceleration", 0.3}, {"angular_acceleration", 0.8},
      {"linear_jerk", 1.5}, {"angular_jerk", 4.0},
      {"fine_dist_tolerance", 0.03}, {"coarse_dist_tolerance", 0.2},
      {"angle_tolerance", 0.05}, {"lookahead_min_m", 0.20},
      {"lookahead_max_m", 0.60}, {"lookahead_time_sec", 0.6},
      {"initial_alignment_fraction", 0.10}, {"final_alignment_fraction", 0.10},
      {"head_pan_min", -3.14}, {"head_pan_max", 1.74}, {"head_tilt", -0.5},
      {"head_alignment_tolerance", 0.15}, {"head_feedback_timeout_sec", 0.5},
      {"head_alignment_timeout_sec", 10.0}, {"sensor_timeout_sec", 0.5},
      {"map_timeout_sec", 3.0}, {"tf_timeout_sec", 0.5}, {"max_path_deviation_m", 0.35},
      {"deviation_persistence_sec", 0.5}, {"progress_timeout_sec", 8.0},
      {"collision_reaction_time_sec", 0.10}, {"repulsion_max_speed", 0.15}};
    for (const auto & setting : settings) {
      rcl_interfaces::msg::ParameterDescriptor descriptor;
      descriptor.description =
        "Navigation control setting; distances in metres, angles in radians, times in seconds.";
      declare_parameter<double>(setting.first, setting.second, descriptor);
    }
    const auto validate = [this](const std::vector<rclcpp::Parameter> & changes) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        try {
          for (const auto & p : changes) {
            if (p.get_name() == "motion_mode") {
              if (p.as_string() != "omni" && p.as_string() != "differential") {
                throw std::invalid_argument("motion_mode must be omni or differential");
              }
            } else if (p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
              if (!std::isfinite(p.as_double()) ||
                (p.get_name() != "head_pan_min" && p.get_name() != "head_tilt" &&
                p.as_double() <= 0.0))
              {
                throw std::invalid_argument("Control limits must be finite and positive");
              }
            }
          }
          const auto value = [this, &changes](const std::string & name) {
              for (const auto & p : changes) {if (p.get_name() == name) {return p.as_double();}}
              return get_parameter(name).as_double();
            };
          if (value("head_pan_min") >= value("head_pan_max") ||
            value("head_pan_min") > 0.0 || value("head_pan_max") < 0.0 ||
            value("lookahead_min_m") > value("lookahead_max_m") ||
            value("min_linear_speed") > value("max_linear_speed") ||
            value("initial_alignment_fraction") > 0.5 || value("final_alignment_fraction") > 0.5)
          {
            throw std::invalid_argument("Inconsistent control limits");
          }
        } catch (const std::exception & error) {
          result.successful = false;
          result.reason = error.what();
        }
        // Read committed parameters in the timer; rejected batches never mutate control state.
        return result;
      };
    parameter_callback_ = add_on_set_parameters_callback(validate);
    std::vector<rclcpp::Parameter> initial;
    for (const auto & setting : settings) {initial.push_back(get_parameter(setting.first));}
    initial.push_back(get_parameter("motion_mode"));
    initial.push_back(get_parameter("control_alpha"));
    initial.push_back(get_parameter("control_beta"));
    const auto validation = validate(initial);
    if (!validation.successful) {throw std::invalid_argument(validation.reason);}
    std::string prefix = namespaced ? get_namespace() : "";
    if (prefix == "/") {prefix.clear();}
    const auto control = prefix + "/simple_move/";
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    fields_enable_pub_ = create_publisher<std_msgs::msg::Bool>(
      prefix + "/potential_fields/enable",
      10);
    status_pub_ = create_publisher<Status>(
      control + "goal_reached", rclcpp::QoS(
        10).transient_local());
    head_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/hardware/head/goal_pose", 10);
    intent_pub_ = create_publisher<geometry_msgs::msg::Vector3>(control + "motion_direction", 10);
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      prefix + "/augmented_map", rclcpp::QoS(1).transient_local(),
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr grid) {
        const auto & q = grid->info.origin.orientation;
        const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
        if (grid->header.frame_id == "map" && grid->info.width > 0 && grid->info.height > 0 &&
        std::isfinite(grid->info.resolution) && grid->info.resolution > 0.0 &&
        std::isfinite(grid->info.origin.position.x) &&
        std::isfinite(grid->info.origin.position.y) &&
        std::isfinite(norm) && std::abs(norm - 1.0) < 1e-5 && std::abs(q.x) < 1e-8 &&
        std::abs(q.y) < 1e-8 &&
        static_cast<uint64_t>(grid->info.width) * grid->info.height == grid->data.size())
        {
          map_ = grid;map_received_ = Clock::now();
        }
      });
    path_sub_ = create_subscription<Path>(
      control + "goal_path", 10,
      [this](Path::ConstSharedPtr path) {start_motion(*path, false);});
    head_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "/hardware/head/current_pose", rclcpp::SensorDataQoS(),
      [this](std_msgs::msg::Float64MultiArray::ConstSharedPtr msg) {
        if (msg->data.size() >= 2 && std::isfinite(msg->data[0])) {
          head_pan_ = msg->data[0]; head_received_ = Clock::now();
        }
      });
    speed_limit_sub_ = create_subscription<std_msgs::msg::Float32>(
      prefix + "/potential_fields/speed_limit", rclcpp::SensorDataQoS(),
      [this](std_msgs::msg::Float32::ConstSharedPtr message) {
        if (!std::isnan(message->data) && message->data >= 0.0f) {
          obstacle_speed_limit_ = message->data;speed_limit_received_ = Clock::now();
        }
      });
    ready_sub_ = create_subscription<std_msgs::msg::Bool>(
      prefix + "/potential_fields/ready", rclcpp::SensorDataQoS(),
      [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        fields_ready_ = msg->data;ready_received_ = Clock::now();
      });
    risk_sub_ = create_subscription<std_msgs::msg::Bool>(
      prefix + "/potential_fields/collision_risk",
      rclcpp::SensorDataQoS(), [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        collision_ = msg->data; risk_received_ = Clock::now();
      });
    force_sub_ = create_subscription<geometry_msgs::msg::Vector3>(
      prefix + "/potential_fields/pf_rejection_force",
      rclcpp::SensorDataQoS(), [this](geometry_msgs::msg::Vector3::ConstSharedPtr msg) {
        if (std::isfinite(msg->x) && std::isfinite(msg->y)) {
          force_ = *msg; force_received_ = Clock::now();
        }
      });
    bypass_sub_ = create_subscription<std_msgs::msg::Bool>(
      prefix + "/camera_obstacles/bypass", 10,
      [this](std_msgs::msg::Bool::ConstSharedPtr msg) {head_bypass_ = msg->data;});
    for (const auto & topic : {prefix + "/stop", prefix + "/navigation/stop", control + "stop"}) {
      stop_subs_.push_back(
        create_subscription<std_msgs::msg::Empty>(
          topic, 10,
          [this](std_msgs::msg::Empty::ConstSharedPtr) {
            if (active_) {
              finish(Status::ABORTED, "stop_received");
            }
          }));
    }
    nav_stop_sub_ = create_subscription<std_msgs::msg::Bool>(
      prefix + "/navigation_ctrl/stop", 10,
      [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        if (msg->data && active_) {
          finish(Status::ABORTED, "stop_received");
        }
      });
    for (const auto & topic : {"goal_dist", "goal_dist_lateral"}) {
      distance_subs_.push_back(
        create_subscription<std_msgs::msg::Float32>(
          control + topic, 10,
          [this, topic](std_msgs::msg::Float32::ConstSharedPtr msg) {
            start_relative(msg->data, 0.0, std::string(topic) == "goal_dist_lateral");
          }));
    }
    relative_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      control + "goal_dist_angle", 10,
      [this](std_msgs::msg::Float32MultiArray::ConstSharedPtr msg) {
        if (msg->data.size() == 2) {start_relative(msg->data[0], msg->data[1], false);}
      });
    timer_ =
      create_wall_timer(
      std::chrono::milliseconds(30),
      std::bind(&SimpleMoveNode::process_motion, this));
  }

private:
  void start_relative(double distance, double angle, bool lateral)
  {
    if (!std::isfinite(distance) || !std::isfinite(angle)) {return;}
    if (lateral && get_parameter("motion_mode").as_string() == "differential") {
      finish(Status::ABORTED, "lateral_motion_requires_omni"); return;
    }
    try {
      const auto tf = tf_buffer_.lookupTransform(odom_frame_, base_frame_, tf2::TimePointZero);
      const auto & q = tf.transform.rotation;
      const double yaw =
        std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
      Path path;
      path.header.frame_id = odom_frame_;
      path.header.stamp = now();
      geometry_msgs::msg::PoseStamped pose;
      pose.pose.position.x = tf.transform.translation.x;
      pose.pose.position.y = tf.transform.translation.y;
      pose.pose.orientation = q;
      path.poses.push_back(pose);
      pose.pose.position.x += distance * std::cos(yaw + angle + (lateral ? M_PI / 2.0 : 0.0));
      pose.pose.position.y += distance * std::sin(yaw + angle + (lateral ? M_PI / 2.0 : 0.0));
      pose.pose.orientation.z = std::sin((yaw + angle) / 2.0);
      pose.pose.orientation.w = std::cos((yaw + angle) / 2.0);
      pose.pose.orientation.x = pose.pose.orientation.y = 0.0;
      path.poses.push_back(pose);
      start_motion(path, true);
      reverse_ = distance < 0.0 && !lateral;
    } catch (const tf2::TransformException & error) {
      finish(Status::ABORTED, "relative_goal_tf_unavailable");
      RCLCPP_WARN(get_logger(), "%s", error.what());
    }
  }

  void start_motion(const Path & path, bool relative)
  {
    if (active_) {finish(Status::ABORTED, "preempted");}
    status_.goal_id.id = relative ? "-1" : std::to_string(path.header.stamp.sec) + "." +
      std::to_string(path.header.stamp.nanosec);
    status_.goal_id.stamp = path.header.stamp;
    if (path.poses.empty() || path.header.frame_id.empty()) {
      finish(Status::ABORTED, "invalid_path"); return;
    }
    for (const auto & pose : path.poses) {
      const auto & p = pose.pose.position;
      const auto & q = pose.pose.orientation;
      const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(norm) ||
        std::abs(norm - 1.0) > 1e-3 ||
        (!pose.header.frame_id.empty() && pose.header.frame_id != path.header.frame_id))
      {
        finish(Status::ABORTED, "invalid_path"); return;
      }
    }
    path_ = path;
    lengths_.assign(path.poses.size(), 0.0);
    for (size_t i = 1; i < path.poses.size(); ++i) {
      lengths_[i] = lengths_[i - 1] + std::hypot(
        path.poses[i].pose.position.x - path.poses[i - 1].pose.position.x,
        path.poses[i].pose.position.y - path.poses[i - 1].pose.position.y);
    }
    active_ = true;
    relative_ = relative;
    reverse_ = false;
    initialized_ = false;
    final_heading_ = 0.0;
    for (size_t i = path.poses.size(); i > 1; --i) {
      const auto & a = path.poses[i - 2].pose.position;
      const auto & b = path.poses[i - 1].pose.position;
      if (std::hypot(b.x - a.x, b.y - a.y) > 1e-8) {
        final_heading_ = std::atan2(b.y - a.y, b.x - a.x);break;
      }
    }
    progress_ = best_progress_ = 0.0;
    velocity_.fill(0.0); acceleration_.fill(0.0);
    started_ = progress_time_ = head_wait_ = last_tick_ = Clock::now();
    deviation_since_ = Clock::time_point{};
    status_.status = Status::ACTIVE;
    status_.text = "tracking_path";
    status_pub_->publish(status_);
  }

  void finish(uint8_t code, const std::string & reason)
  {
    active_ = false;
    velocity_.fill(0.0); acceleration_.fill(0.0);
    cmd_pub_->publish(geometry_msgs::msg::Twist());
    if (relative_ && get_parameter("use_pot_fields").as_bool()) {
      std_msgs::msg::Bool enable;enable.data = false;fields_enable_pub_->publish(enable);
    }
    status_.status = code;
    status_.text = reason;
    status_pub_->publish(status_);
    RCLCPP_INFO(get_logger(), "Motion %s: %s", status_.goal_id.id.c_str(), reason.c_str());
  }

  void process_motion()
  {
    if (!active_) {return;}
    const auto tick = Clock::now();
    if (std::chrono::duration<double>(tick - last_feedback_).count() >= 0.2) {
      status_.status = Status::ACTIVE;
      status_pub_->publish(status_);
      last_feedback_ = tick;
    }
    const double dt = std::clamp(
      std::chrono::duration<double>(
        tick - last_tick_).count(), 0.001, 0.1);
    last_tick_ = tick;
    // A single mutually-exclusive callback group serializes goals, parameters and control.
    const auto parameters = get_parameters(
      {"max_linear_speed", "max_angular_speed", "linear_acceleration",
        "angular_acceleration", "linear_jerk", "angular_jerk", "fine_dist_tolerance",
        "angle_tolerance",
        "head_pan_min", "head_pan_max", "head_alignment_tolerance", "sensor_timeout_sec"});
    const double vmax = parameters[0].as_double(), wmax = parameters[1].as_double();
    const double amax = parameters[2].as_double(), angular_accel = parameters[3].as_double();
    const double tolerance = parameters[6].as_double(), angle_tolerance = parameters[7].as_double();
    const double pan_min = parameters[8].as_double(), pan_max = parameters[9].as_double();
    const double head_tolerance = parameters[10].as_double(),
      sensor_timeout = parameters[11].as_double();
    const bool omni = get_parameter("motion_mode").as_string() == "omni";
    const bool use_head = get_parameter("move_head").as_bool() && !head_bypass_;
    const bool use_fields = get_parameter("use_pot_fields").as_bool();
    if (omni != last_omni_) {velocity_.fill(0.0); acceleration_.fill(0.0); last_omni_ = omni;}
    if (relative_ && use_fields &&
      std::chrono::duration<double>(tick - last_fields_enable_).count() >= 0.2)
    {
      std_msgs::msg::Bool enable;enable.data = true;fields_enable_pub_->publish(enable);
      last_fields_enable_ = tick;
    }
    double x, y, yaw;
    try {
      const auto tf = tf_buffer_.lookupTransform(
        path_.header.frame_id, base_frame_,
        tf2::TimePointZero);
      const auto stamp = rclcpp::Time(tf.header.stamp, get_clock()->get_clock_type());
      if (stamp.nanoseconds() != 0 &&
        std::abs((now() - stamp).seconds()) > get_parameter("tf_timeout_sec").as_double())
      {
        throw std::runtime_error("Stale robot transform");
      }
      x = tf.transform.translation.x; y = tf.transform.translation.y;
      const auto & q = tf.transform.rotation;
      yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    } catch (const std::exception & error) {
      finish(Status::ABORTED, "robot_tf_unavailable");
      RCLCPP_WARN(get_logger(), "%s", error.what()); return;
    }
    const auto & destination = path_.poses.back().pose;
    const auto & q = destination.orientation;
    const double final_yaw =
      std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    const double final_error = std::remainder(final_yaw - yaw, 2.0 * M_PI);
    const double distance = std::hypot(destination.position.x - x, destination.position.y - y);
    if (!initialized_) {initial_yaw_ = yaw; initialized_ = true;}
    if (use_fields && (speed_limit_received_ < started_ ||
      std::chrono::duration<double>(tick - speed_limit_received_).count() > sensor_timeout ||
      !fields_ready_ || ready_received_ < started_ || risk_received_ < started_ ||
      std::chrono::duration<double>(tick - ready_received_).count() > sensor_timeout ||
      std::chrono::duration<double>(tick - risk_received_).count() > sensor_timeout))
    {
      cmd_pub_->publish(geometry_msgs::msg::Twist());
      velocity_.fill(0.0); acceleration_.fill(0.0);
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(), 2000,
        "Waiting for obstacle monitor: ready=%s, ready_message_age=%.3f s, risk_message_age=%.3f s (-1 means no message)",
        fields_ready_ ? "true" : "false",
        ready_received_ ==
        Clock::time_point{} ? -1.0 : std::chrono::duration<double>(tick - ready_received_).count(),
        risk_received_ ==
        Clock::time_point{} ? -1.0 : std::chrono::duration<double>(tick - risk_received_).count());
      if (std::chrono::duration<double>(tick - started_).count() > 2.0) {
        finish(Status::ABORTED, "obstacle_data_stale");
      }
      return;
    }
    if (distance <= tolerance && std::abs(final_error) <= angle_tolerance) {
      finish(Status::SUCCEEDED, "pose_reached"); return;
    }
    if (std::chrono::duration<double>(tick - started_).count() >
      lengths_.back() / vmax * 6.0 + 2.0 * M_PI / wmax * 3.0 + 30.0)
    {
      finish(Status::ABORTED, "motion_timeout"); return;
    }
    double nearest = std::numeric_limits<double>::infinity();
    size_t segment = 0;
    double projected = progress_;
    // Restrict projection to a local arc-length window so crossings cannot skip a loop.
    for (size_t i = 1; i < path_.poses.size(); ++i) {
      if (lengths_[i] < progress_ - 0.2 || lengths_[i - 1] > progress_ + 1.0) {continue;}
      const auto & a = path_.poses[i - 1].pose.position;
      const auto & b = path_.poses[i].pose.position;
      const double dx = b.x - a.x, dy = b.y - a.y, square = dx * dx + dy * dy;
      const double t = square > 1e-12 ? std::clamp(
        ((x - a.x) * dx + (y - a.y) * dy) / square, 0.0,
        1.0) : 0.0;
      const double error = std::hypot(x - a.x - t * dx, y - a.y - t * dy);
      if (error < nearest) {
        nearest = error; projected = lengths_[i - 1] + t * std::sqrt(square); segment = i;
      }
    }
    progress_ = std::max(progress_, projected);
    if (nearest > get_parameter("max_path_deviation_m").as_double() && path_.poses.size() > 1) {
      if (deviation_since_ == Clock::time_point{}) {deviation_since_ = tick;}
      if (std::chrono::duration<double>(tick - deviation_since_).count() >
        get_parameter("deviation_persistence_sec").as_double())
      {
        finish(Status::ABORTED, "path_deviation"); return;
      }
    } else {deviation_since_ = Clock::time_point{};}
    const double lookahead = std::clamp(
      get_parameter("lookahead_min_m").as_double() +
      std::hypot(velocity_[0], velocity_[1]) * get_parameter("lookahead_time_sec").as_double(),
      get_parameter("lookahead_min_m").as_double(), get_parameter("lookahead_max_m").as_double());
    const double target_s = std::min(lengths_.back(), progress_ + lookahead);
    double target_x = destination.position.x, target_y = destination.position.y;
    double tangent = std::atan2(target_y - y, target_x - x);
    for (size_t i = std::max(size_t{1}, segment); i < path_.poses.size(); ++i) {
      if (lengths_[i] + 1e-9 < target_s) {continue;}
      const auto & a = path_.poses[i - 1].pose.position;
      const auto & b = path_.poses[i].pose.position;
      const double t = (target_s - lengths_[i - 1]) / std::max(1e-9, lengths_[i] - lengths_[i - 1]);
      target_x = a.x + t * (b.x - a.x); target_y = a.y + t * (b.y - a.y);
      tangent = std::atan2(b.y - a.y, b.x - a.x); break;
    }
    double direction = std::atan2(target_y - y, target_x - x);
    const double remaining = std::max(distance, lengths_.back() - progress_);
    const double final_turn = std::abs(std::remainder(final_yaw - final_heading_, 2.0 * M_PI));
    const double base_window = std::max(
      get_parameter("final_alignment_fraction").as_double() * lengths_.back(), 2.0 * tolerance);
    const double turn_time = std::max(
      {final_turn / (0.6 * wmax),
        std::sqrt(2.0 * final_turn / angular_accel),
        std::cbrt(6.0 * final_turn / parameters[5].as_double())});
    const double final_window = std::max(
      base_window, std::min(
        vmax, std::sqrt(
          amax * base_window)) * turn_time);
    const double approach_speed = std::min(vmax, 0.5 * final_window / std::max(0.1, turn_time));
    const double approach_limit = std::sqrt(
      approach_speed * approach_speed +
      2.0 * amax * std::max(0.0, remaining - tolerance - final_window));
    double speed = std::min(vmax, std::sqrt(amax * remaining));
    speed = std::min(speed, 1.5 * distance);
    const double curvature = 2.0 * std::abs(std::sin(tangent - direction)) / std::max(
      0.05,
      lookahead);
    const double curve_limit = std::min(
      std::sqrt(amax / std::max(1e-6, curvature)), wmax / std::max(
        1e-6, curvature));
    speed = std::min(speed, curve_limit);
    if (distance > tolerance) {
      speed = std::min(curve_limit, std::max(get_parameter("min_linear_speed").as_double(), speed));
    }
    if (omni && !relative_ && final_turn > angle_tolerance) {
      speed = std::min(speed, approach_limit);
    }
    speed *=
      std::exp(
      -get_parameter("control_alpha").as_double() *
      std::pow(std::remainder(tangent - direction, 2.0 * M_PI), 2));
    double vx = speed * std::cos(direction - yaw), vy = speed * std::sin(direction - yaw);
    if (use_fields &&
      std::chrono::duration<double>(tick - force_received_).count() < sensor_timeout)
    {
      const double limit = get_parameter("repulsion_max_speed").as_double();
      const double scale = std::min(1.0, limit / std::max(1e-9, std::hypot(force_.x, force_.y)));
      vx += scale * force_.x; vy += scale * force_.y;
      if (std::hypot(vx, vy) > 1e-5) {direction = yaw + std::atan2(vy, vx);}
    }
    double yaw_feedforward = 0.0;
    const double initial_fraction = get_parameter("initial_alignment_fraction").as_double();
    double blend = std::clamp(
      progress_ / std::max(
        0.01,
        initial_fraction * lengths_.back()), 0.0, 1.0);
    const double initial_phase = blend;
    blend = blend * blend * (3.0 - 2.0 * blend);
    double desired_yaw = initial_yaw_ + blend * std::remainder(tangent - initial_yaw_, 2.0 * M_PI);
    // Complete the angular profile at the positional tolerance boundary.
    // A fixed distance window avoids chasing a shrinking window while braking.
    double final_blend = std::clamp(
      1.0 - std::max(
        0.0,
        remaining - tolerance) / final_window, 0.0, 1.0);
    const double final_phase = final_blend;
    final_blend = final_blend * final_blend * (3.0 - 2.0 * final_blend);
    desired_yaw += final_blend * std::remainder(final_yaw - desired_yaw, 2.0 * M_PI);
    if (omni && !relative_) {
      // Bound translational progress by the angular rate required by the yaw
      // profile. Short routes slow down instead of exhausting the first/last
      // ten percent before the base can turn.
      const double initial_slope = 6.0 * initial_phase * (1.0 - initial_phase) *
        std::abs(std::remainder(tangent - initial_yaw_, 2.0 * M_PI)) /
        std::max(0.01, initial_fraction * lengths_.back());
      const double final_slope = 6.0 * final_phase * (1.0 - final_phase) *
        std::abs(std::remainder(final_yaw - tangent, 2.0 * M_PI)) / std::max(0.01, final_window);
      yaw_feedforward = (
        (1.0 - final_blend) * 6.0 * initial_phase * (1.0 - initial_phase) *
        std::remainder(
          tangent - initial_yaw_,
          2.0 * M_PI) / std::max(0.01, initial_fraction * lengths_.back()) +
        6.0 * final_phase * (1.0 - final_phase) *
        std::remainder(final_yaw - tangent, 2.0 * M_PI) / std::max(0.01, final_window)
        ) * std::hypot(velocity_[0], velocity_[1]);
      const double yaw_error = std::abs(std::remainder(desired_yaw - yaw, 2.0 * M_PI));
      const double limited_speed =
        std::min(
        {vmax, final_turn > angle_tolerance ? approach_limit : vmax,
          0.6 * wmax / std::max(1e-6, initial_slope + final_slope)});
      const double scale = std::min(
        {1.0, limited_speed / std::max(1e-6, std::hypot(vx, vy)),
          0.2 / std::max(0.2, yaw_error)});
      vx *= scale;vy *= scale;
    }
    double heading_error = std::remainder(direction - yaw - (reverse_ ? M_PI : 0.0), 2.0 * M_PI);
    if (relative_) {desired_yaw = final_yaw;}
    const double angular_gain = 1.0 / std::max(0.05, get_parameter("control_beta").as_double());
    double angular = std::clamp(
      yaw_feedforward + angular_gain * std::remainder(
        desired_yaw - yaw,
        2.0 * M_PI), -wmax, wmax);
    if (!omni) {
      angular = std::clamp(angular_gain * heading_error, -wmax, wmax);
      vx = (reverse_ ? -1.0 : 1.0) * std::hypot(vx, vy) * std::max(0.0, std::cos(heading_error));
      vy = 0.0;
    }
    if (distance <= tolerance) {
      vx = vy = 0.0;
      const double limit = get_parameter("final_angle_max_angular_speed").as_double();
      angular = std::clamp(2.0 * final_error, -limit, limit);
    }
    bool head_blocked = false;
    if (use_head && distance > tolerance) {
      const double pan = std::remainder(direction - yaw, 2.0 * M_PI);
      const double clamped = std::clamp(pan, pan_min, pan_max);
      std_msgs::msg::Float64MultiArray goal;
      goal.data = {clamped, get_parameter("head_tilt").as_double()};
      head_pub_->publish(goal);
      const bool fresh = head_received_ != Clock::time_point{} &&
      std::chrono::duration<double>(tick - head_received_).count() <= get_parameter(
        "head_feedback_timeout_sec").as_double();
      head_blocked = !fresh ||
        std::abs(std::remainder(pan - head_pan_, 2.0 * M_PI)) > head_tolerance;
      if (head_blocked) {
        if (progress_ < 0.001 && std::abs(pan - clamped) > 0.01) {initial_yaw_ = yaw;}
        vx = vy = 0.0;
        angular = fresh && std::abs(pan - clamped) >
          0.01 ? std::clamp(2.0 * (pan - clamped), -wmax, wmax) : 0.0;
        if (std::chrono::duration<double>(tick - head_wait_).count() >
          get_parameter("head_alignment_timeout_sec").as_double())
        {
          finish(Status::ABORTED, "head_alignment_timeout"); return;
        }
      } else {head_wait_ = tick;}
    } else {head_wait_ = tick;}
    if (progress_ > best_progress_ + 0.02 || distance <= tolerance || head_blocked) {
      best_progress_ = progress_; progress_time_ = tick;
    } else if (std::chrono::duration<double>(tick - progress_time_).count() >
      get_parameter("progress_timeout_sec").as_double())
    {
      finish(Status::ABORTED, "no_progress"); return;
    }
    geometry_msgs::msg::Vector3 intent;
    intent.x = std::cos(direction - yaw); intent.y = std::sin(direction - yaw);
    intent_pub_->publish(intent);
    if (use_fields && collision_) {finish(Status::ABORTED, "collision_risk"); return;}
    const double magnitude = std::hypot(vx, vy);
    const double translation_limit = use_fields ? std::min(vmax, obstacle_speed_limit_) : vmax;
    if (magnitude > translation_limit) {
      vx *= translation_limit / magnitude;vy *= translation_limit / magnitude;
    }
    std::array<double, 3> target{vx, vy, angular};
    for (size_t i = 0; i < 3; ++i) {
      const double a = i == 2 ? angular_accel : amax;
      const double jerk = parameters[i == 2 ? 5 : 4].as_double();
      const double requested = std::clamp((target[i] - velocity_[i]) / dt, -a, a);
      acceleration_[i] += std::clamp(requested - acceleration_[i], -jerk * dt, jerk * dt);
      const double next = velocity_[i] + acceleration_[i] * dt;
      if ((target[i] - velocity_[i]) * (target[i] - next) <= 0.0) {
        velocity_[i] = target[i]; acceleration_[i] = 0.0;
      } else {velocity_[i] = next;}
    }
    // Do not translate while the camera is aligning; emergency stops bypass smoothing.
    if (head_blocked || distance <= tolerance) {
      velocity_[0] = velocity_[1] = acceleration_[0] = acceleration_[1] = 0.0;
    }
    if (head_blocked && std::abs(angular) < 1e-9) {velocity_[2] = acceleration_[2] = 0.0;}
    if (!omni) {velocity_[1] = acceleration_[1] = 0.0;}
    const double limited_speed = std::hypot(velocity_[0], velocity_[1]);
    if (limited_speed > translation_limit) {
      velocity_[0] *= translation_limit / limited_speed;
      velocity_[1] *= translation_limit / limited_speed;
    }
    velocity_[2] = std::clamp(velocity_[2], -wmax, wmax);
    geometry_msgs::msg::Twist command;
    command.linear.x = velocity_[0]; command.linear.y = velocity_[1];
    command.angular.z = velocity_[2];
    if (path_.header.frame_id == "map" && !local_motion_is_free(x, y, yaw, command)) {
      finish(Status::ABORTED, "local_trajectory_blocked");return;
    }
    cmd_pub_->publish(command);
  }

  // Check the actual limited command, including repulsion, over its braking
  // horizon in the already-inflated grid. A half-cell guard covers sampling
  // at grid corners without adding another robot footprint.
  bool local_motion_is_free(
    double x, double y, double yaw,
    const geometry_msgs::msg::Twist & command) const
  {
    if (!map_ || std::chrono::duration<double>(Clock::now() - map_received_).count() >
      get_parameter("map_timeout_sec").as_double()) {return false;}
    const auto stamp = rclcpp::Time(map_->header.stamp, get_clock()->get_clock_type());
    if (stamp.nanoseconds() != 0 &&
      std::abs((now() - stamp).seconds()) > get_parameter("map_timeout_sec").as_double())
    {
      return false;
    }
    const auto & info = map_->info;
    const auto & q = info.origin.orientation;
    const double map_yaw = std::atan2(2.0 * q.w * q.z, 1.0 - 2.0 * q.z * q.z);
    const double c = std::cos(map_yaw), s = std::sin(map_yaw);
    const double speed = std::hypot(command.linear.x, command.linear.y);
    const double reaction = get_parameter("collision_reaction_time_sec").as_double();
    const double deceleration = get_parameter("linear_acceleration").as_double();
    const double angular_deceleration = get_parameter("angular_acceleration").as_double();
    const double horizon = reaction + speed / deceleration;
    const double dt = std::min(0.03, info.resolution / (4.0 * std::max(0.01, speed)));
    for (double t = 0.0; t <= horizon; t += dt) {
      const double gx = (c * (x - info.origin.position.x) + s * (y - info.origin.position.y)) /
        info.resolution;
      const double gy = (-s * (x - info.origin.position.x) + c * (y - info.origin.position.y)) /
        info.resolution;
      if (!std::isfinite(gx) || !std::isfinite(gy) || gx < 0 || gy < 0 || gx >= info.width ||
        gy >= info.height) {return false;}
      const int ix = static_cast<int>(std::floor(gx)), iy = static_cast<int>(std::floor(gy));
      for (int cy = std::max(0, iy - 1); cy <= std::min(static_cast<int>(info.height) - 1, iy + 1);
        ++cy)
      {
        for (int cx = std::max(0, ix - 1); cx <= std::min(static_cast<int>(info.width) - 1, ix + 1);
          ++cx)
        {
          const double dx = std::max({double(cx) - gx, 0.0, gx - double(cx + 1)});
          const double dy = std::max({double(cy) - gy, 0.0, gy - double(cy + 1)});
          if (std::hypot(
              dx,
              dy) <= 0.5 && map_->data[static_cast<size_t>(cy) * info.width + cx] != 0)
          {
            return false;
          }
        }
      }
      const double scale = speed >
        1e-9 ? std::clamp(
        1.0 - deceleration * std::max(
          0.0,
          t + 0.5 * dt - reaction) / speed, 0.0,
        1.0) : 0.0;
      x += (std::cos(yaw) * command.linear.x - std::sin(yaw) * command.linear.y) * dt * scale;
      y += (std::sin(yaw) * command.linear.x + std::cos(yaw) * command.linear.y) * dt * scale;
      const double angular_scale = std::abs(command.angular.z) > 1e-9 ?
        std::clamp(
        1.0 - angular_deceleration *
        std::max(0.0, t + 0.5 * dt - reaction) / std::abs(command.angular.z), 0.0, 1.0) : 0.0;
      yaw += command.angular.z * dt * angular_scale;
    }
    return true;
  }

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::string base_frame_, odom_frame_;
  Path path_;
  Status status_;
  std::vector<double> lengths_;
  std::array<double, 3> velocity_{}, acceleration_{};
  double progress_ = 0.0, best_progress_ = 0.0, initial_yaw_ = 0.0, head_pan_ = 0.0,
    final_heading_ = 0.0;
  bool active_ = false, relative_ = false, reverse_ = false, initialized_ = false;
  bool head_bypass_ = false, collision_ = false, fields_ready_ = false, last_omni_ = true;
  double obstacle_speed_limit_ = 0.0;
  Clock::time_point speed_limit_received_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr speed_limit_sub_;
  geometry_msgs::msg::Vector3 force_;
  Clock::time_point started_, progress_time_, head_wait_, last_tick_, deviation_since_;
  Clock::time_point risk_received_, force_received_, head_received_, last_feedback_, map_received_,
    last_fields_enable_, ready_received_;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr map_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr intent_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr fields_enable_pub_;
  rclcpp::Publisher<Status>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr head_pub_;
  rclcpp::Subscription<Path>::SharedPtr path_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr head_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr risk_sub_, ready_sub_, bypass_sub_,
    nav_stop_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr force_sub_;
  std::vector<rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr> stop_subs_;
  std::vector<rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr> distance_subs_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr relative_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleMoveNode>());
  rclcpp::shutdown();
  return 0;
}
