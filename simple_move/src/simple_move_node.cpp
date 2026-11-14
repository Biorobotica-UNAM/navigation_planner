// ----- ROS 2 -----
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

// ----- ROS 2 messages -----
#include <actionlib_msgs/msg/goal_status.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <path_planner/msg/navigation_path.hpp>
#include <path_planner/motion_observation.hpp>
#include <path_planner/motion_constraints.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <simple_move/tracking_reference.hpp>

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
#include <string_view>
#include <vector>

class SimpleMoveNode : public rclcpp::Node
{
  using Path = nav_msgs::msg::Path;
  using Status = actionlib_msgs::msg::GoalStatus;
  using Clock = std::chrono::steady_clock;
  using Command = path_planner::msg::NavigationPath;
  struct LocalContext
  {
    nav_msgs::msg::OccupancyGrid::ConstSharedPtr grid;
    const char * rejection = "map_unavailable";
    bool physical = false, omni = true;
    double c = 1.0, s = 0.0, reaction = 0.1, deceleration = 0.3, angular_deceleration = 0.8,
      escape_radius = 0.6;
  };
  struct LocalCheck
  {
    const char * reason = "not_checked";
    const char * layer = "none";
    int cell_x = -1, cell_y = -1, value = -1;
    double time = 0.0, separation = -1.0, required = -1.0;
    std::array<double, 3> tested{};
  };
  struct MotionTrace
  {
    double x = 0.0, y = 0.0, yaw = 0.0, distance = -1.0, tolerance = 0.0;
    double yaw_error = 0.0, reference = 0.0, direction = 0.0, remaining = -1.0, dt = 0.0;
    double head_error = 0.0, head_scale = 1.0, raw_direction = 0.0, direction_rate = 0.0;
    const char * head_gate = "inactive";
    std::array<double, 3> nominal{}, requested{};
    size_t candidates = 0, nominal_rejected = 0, reachable_rejected = 0, corridor_rejected = 0;
    LocalCheck rejected, published;
  };

public:
  SimpleMoveNode()
  : Node("simple_move_node"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    rcl_interfaces::msg::ParameterDescriptor startup;
    startup.read_only = true;
    startup.description = "Configured at startup.";
    const bool namespaced = declare_parameter<bool>("use_namespace", false, startup);
    allow_unknown_ = declare_parameter<bool>("allow_unknown", false, startup);
    base_frame_ = declare_parameter<std::string>("base_link_name", "base_link", startup);
    odom_frame_ = declare_parameter<std::string>("odom_name", "odom", startup);
    map_frame_ = declare_parameter<std::string>("map_frame", "map", startup);
    const auto head_goal_topic = declare_parameter<std::string>(
      "head_goal_topic", "/hardware/head/goal_pose", startup);
    const auto head_feedback_topic = declare_parameter<std::string>(
      "head_feedback_topic", "/hardware/head/current_pose", startup);
    declare_parameter<std::string>("motion_mode", "omni");
    declare_parameter<bool>("move_head", true);
    declare_parameter<bool>("use_pot_fields", false);
    declare_parameter<bool>("debug_detection", false);
    declare_parameter<double>("robot_radius", 0.23, startup);
    // Existing launch arguments remain accepted for relative-motion clients.
    declare_parameter<double>("control_alpha", 0.2, startup);
    declare_parameter<double>("control_beta", 0.6, startup);
    const std::vector<std::pair<std::string, double>> settings = {
      {"max_linear_speed", 0.3}, {"min_linear_speed", 0.05},
      {"max_angular_speed", 1.3}, {"final_angle_max_angular_speed", 0.65},
      {"linear_acceleration", 0.3}, {"angular_acceleration", 0.8},
      {"linear_jerk", 1.5}, {"angular_jerk", 4.0},
      {"fine_dist_tolerance", 0.03}, {"coarse_dist_tolerance", 0.2},
      {"angle_tolerance", 0.05}, {"lookahead_min_m", 0.20},
      {"lookahead_max_m", 0.60}, {"lookahead_time_sec", 0.6},
      {"direction_filter_time_sec", 0.15}, {"direction_max_rate_rad_sec", 1.2},
      {"initial_alignment_fraction", 0.10}, {"final_alignment_fraction", 0.10},
      {"head_pan_min", -3.14}, {"head_pan_max", 1.74}, {"head_tilt", -0.5},
      {"head_alignment_tolerance", 0.15}, {"head_feedback_timeout_sec", 0.5},
      {"head_stop_error_rad", 0.55}, {"head_lookahead_time_sec", 0.25},
      {"head_preview_angle_rad", 0.25},
      {"head_pan_limit_margin", 0.05},
      {"head_max_pan_speed", 1.5},
      {"head_max_pan_acceleration", 2.5},
      {"final_position_hysteresis_m", 0.03},
      {"final_position_settle_sec", 0.30}, {"head_goal_deadband_rad", 0.04},
      {"final_capture_distance_m", 0.40}, {"angular_damping", 0.35},
      {"start_escape_radius", 0.6}, {"start_escape_speed", 0.08},
      {"local_avoidance_speed", 0.20},
      {"avoidance_switch_hold_sec", 0.25}, {"avoidance_switch_score_margin", 0.03},
      {"local_avoidance_timeout_sec", 1.5}, {"max_avoidance_deviation_m", 0.55},
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
            value("head_alignment_tolerance") >= value("head_stop_error_rad") ||
            value("head_preview_angle_rad") >= value("head_stop_error_rad") ||
            2.0 * value("head_pan_limit_margin") >= value("head_pan_max") - value("head_pan_min") ||
            value("max_path_deviation_m") > value("max_avoidance_deviation_m") ||
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
    // Match the retained command transport used by the HSR relay and bumper.
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(
      "/cmd_vel", rclcpp::QoS(10).transient_local());
    fields_enable_pub_ = create_publisher<std_msgs::msg::Bool>(
      prefix + "/potential_fields/enable",
      10);
    status_pub_ = create_publisher<Status>(
      control + "goal_reached", rclcpp::QoS(
        10).transient_local());
    head_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(head_goal_topic, 1);
    intent_pub_ = create_publisher<geometry_msgs::msg::Vector3>(control + "motion_direction", 10);
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      prefix + "/augmented_map", rclcpp::QoS(1).transient_local(),
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr grid) {
        const auto & q = grid->info.origin.orientation;
        const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
        if (grid->header.frame_id == map_frame_ && grid->info.width > 0 && grid->info.height > 0 &&
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
    clearance_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      prefix + "/clearance_map", rclcpp::QoS(1).transient_local(),
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr grid) {clearance_map_ = grid;});
    uncertainty_sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
      prefix + "/uncertain_obstacles",
      rclcpp::QoS(1).transient_local(), [this](
        geometry_msgs::msg::PoseArray::ConstSharedPtr points) {
        if (points->header.frame_id != map_frame_) {return;}
        for (const auto & pose : points->poses) {
          if (!std::isfinite(pose.position.x) || !std::isfinite(pose.position.y)) {return;}
        }
        uncertain_obstacles_ = points;uncertainty_received_ = Clock::now();
      });
    path_sub_ = create_subscription<Path>(
      control + "goal_path", 10,
      [this](Path::ConstSharedPtr path) {start_motion(*path, false);});
    navigation_path_sub_ = create_subscription<path_planner::msg::NavigationPath>(
      control + "navigation_path", 10,
      [this](path_planner::msg::NavigationPath::ConstSharedPtr command) {
        if (!std::isfinite(command->arrival_radius) || command->arrival_radius < 0.0 ||
        !std::isfinite(command->arrival_center.x) || !std::isfinite(command->arrival_center.y))
        {
          RCLCPP_WARN(get_logger(), "Invalid arrival region");return;
        }
        double precision = 0.0;
        if (command->arrival_radius > 0.0) {
          if (command->path.poses.empty()) {return;}
          const auto & p = command->path.poses.back().pose.position;
          const double slack = command->arrival_radius - std::hypot(
            p.x - command->arrival_center.x, p.y - command->arrival_center.y);
          if (slack <= 0.0) {
            RCLCPP_WARN(get_logger(), "Path endpoint outside arrival region");return;
          }
          precision = 0.5 * slack;
        }
        if (command->mode > Command::OBSERVE) {return;}
        start_motion(command->path, false, command->mode);
        if (active_) {
          motion_position_tolerance_ = precision;
          arrival_radius_ = command->arrival_radius;arrival_center_ = command->arrival_center;
        }
      });
    obstruction_sub_ = create_subscription<std_msgs::msg::Bool>(
      control + "route_obstructed", 1,
      [this](std_msgs::msg::Bool::ConstSharedPtr message) {route_obstructed_ = message->data;});
    head_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      head_feedback_topic, rclcpp::SensorDataQoS(),
      [this](std_msgs::msg::Float64MultiArray::ConstSharedPtr msg) {
        if (msg->data.size() >= 2 && std::isfinite(msg->data[0])) {
          const auto tick = Clock::now();
          const double dt = std::chrono::duration<double>(tick - head_received_).count();
          measured_head_rate_ = dt >= 0.005 && dt <= 0.5 ?
            std::clamp((msg->data[0] - head_pan_) / dt,
              -get_parameter("head_max_pan_speed").as_double(),
              get_parameter("head_max_pan_speed").as_double()) : 0.0;
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
  double select_head_pan(double angle, double minimum, double maximum) const
  {
    const double principal = std::remainder(angle, 2.0 * M_PI);
    const int center = int(std::round((head_pan_ - principal) / (2.0 * M_PI)));
    double best = principal, best_correction = std::numeric_limits<double>::infinity();
    double best_travel = std::numeric_limits<double>::infinity();
    for (int turn = center - 1; turn <= center + 1; ++turn) {
      const double candidate = principal + turn * 2.0 * M_PI;
      const double correction = std::abs(candidate - std::clamp(candidate, minimum, maximum));
      const double travel = std::abs(std::clamp(candidate, minimum, maximum) - head_pan_);
      if (correction < best_correction - 1e-9 ||
        (std::abs(correction - best_correction) <= 1e-9 && travel < best_travel))
      {
        best = candidate;best_correction = correction;best_travel = travel;
      }
    }
    return best;
  }

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

  void start_motion(const Path & path, bool relative, uint8_t kind = Command::FOLLOW_PATH)
  {
    const auto previous_velocity = velocity_, previous_acceleration = acceleration_;
    const bool braking = active_ && kind == Command::BRAKE &&
      path.header.frame_id == path_.header.frame_id;
    if (active_ && !braking) {finish(Status::ABORTED, "preempted");}
    if (braking) {
      status_.status = Status::ABORTED;status_.text = "preempted_by_braking";status_pub_->publish(
        status_);
    }
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
    motion_kind_ = kind;
    motion_settled_since_ = Clock::time_point{};
    odometry_ = path_planner::MotionObservation{};
    head_wait_elapsed_ = 0.0;
    challenger_direction_ = std::numeric_limits<double>::quiet_NaN();
    challenger_since_ = Clock::time_point{};
    recovery_clear_since_ = Clock::time_point{};
    observation_aligned_since_ = Clock::time_point{};
    translation_direction_.reset();
    last_phase_.clear();last_diagnostic_ = Clock::time_point{};
    trace_ = MotionTrace{};
    motion_position_tolerance_ = 0.0;
    arrival_radius_ = 0.0;
    relative_ = relative;
    reverse_ = false;
    initialized_ = false;
    final_align_ = false;
    escape_active_ = escape_checked_ = false;
    obstacle_stale_since_ = Clock::time_point{};
    intent_started_ = Clock::time_point{};
    final_profile_ = capture_ = route_obstructed_ = local_detour_ = false;
    terminal_angle_holding_ = false;
    terminal_angle_since_ = Clock::time_point{};
    best_terminal_error_ = std::numeric_limits<double>::infinity();
    final_progress_ = measured_yaw_rate_ = 0.0;
    initial_elapsed_ = final_elapsed_ = 0.0;
    escape_side_ = 0;
    local_direction_ = std::numeric_limits<double>::quiet_NaN();
    local_blocked_elapsed_ = 0.0;
    sensor_blocked_since_ = sensor_clear_since_ = Clock::time_point{};
    last_head_goal_ = std::numeric_limits<double>::quiet_NaN();
    head_holding_ = true;head_limit_holding_ = false;
    terminal_position_since_ = Clock::time_point{};
    last_use_head_ = false;
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
    if (braking) {velocity_ = previous_velocity;acceleration_ = previous_acceleration;}
    started_ = progress_time_ = last_tick_ = Clock::now();
    deviation_since_ = Clock::time_point{};
    status_.status = Status::ACTIVE;
    status_.text = kind == Command::OBSERVE ? "observing_blockage" : "tracking_path";
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
    log_motion(geometry_msgs::msg::Twist());
  }

  void log_motion(const geometry_msgs::msg::Twist & command)
  {
    const auto tick = Clock::now();
    const bool changed = status_.text != last_phase_;
    if (changed) {
      RCLCPP_INFO(
        get_logger(), "Motion %s: %s requested=%.3f published=%.3f obstacle_limit=%.3f angular=%.3f",
        status_.goal_id.id.c_str(), status_.text.c_str(),
        std::hypot(trace_.requested[0], trace_.requested[1]),
        std::hypot(command.linear.x, command.linear.y), obstacle_speed_limit_, command.angular.z);
      last_phase_ = status_.text;
    }
    if (!get_parameter("debug_detection").as_bool()) {return;}
    const int flags = int(final_align_) |
      (int(head_holding_) << 1) | (int(capture_) << 2) | (int(escape_active_) << 3);
    const double interval = final_profile_ || final_align_ || capture_ || local_detour_ ||
      escape_active_ || (get_parameter("move_head").as_bool() && !head_bypass_ && head_holding_) ||
      status_.text.compare(0, 8, "waiting_") == 0 ? 0.2 : 1.0;
    if (!changed && flags == diagnostic_flags_ &&
      std::chrono::duration<double>(tick - last_diagnostic_).count() < interval) {return;}
    diagnostic_flags_ = flags;last_diagnostic_ = tick;
    const double map_age =
      map_ ? (now() -
      rclcpp::Time(map_->header.stamp, get_clock()->get_clock_type())).seconds() : -1.0;
    const double clearance_age =
      clearance_map_ ? (now() -
      rclcpp::Time(clearance_map_->header.stamp, get_clock()->get_clock_type())).seconds() : -1.0;
    const auto & check = std::string_view(trace_.published.reason) ==
      "not_checked" ? trace_.rejected : trace_.published;
    RCLCPP_INFO(
      get_logger(),
      "MotionDiagnostic motion=%s phase=%s frame=%s dt=%.3f pose=(%.3f,%.3f,%.3f) progress=%.3f remaining=%.3f distance=%.3f tolerance=%.3f yaw_error=%.3f yaw_reference=%.3f yaw_rate=%.3f direction=%.3f final_align=%d capture=%d escape=%d head_gate=%s head=%.3f head_goal=%.3f head_error=%.3f head_age=%.3f nominal=(%.3f,%.3f,%.3f) requested=(%.3f,%.3f,%.3f) published=(%.3f,%.3f,%.3f) obstacle_limit=%.3f ready=%d ready_age=%.3f risk_age=%.3f collision=%d map_age=%.3f clearance_age=%.3f local_reason=%s layer=%s tested=(%.3f,%.3f,%.3f) cell=(%d,%d,%d) sample_t=%.3f separation_m=%.4f required_m=%.4f candidates=%zu rejected=(%zu,%zu,%zu) linear_speed=%.4f rest_angular_speed=%.4f odom_span=%.3f position_span=%.4f stationary=%d quiet_sec=%.3f head_wait_sec=%.3f arrival_inside=%d raw_direction=%.3f direction_rate=%.3f head_scale=%.3f",
      status_.goal_id.id.c_str(), status_.text.c_str(),
      path_.header.frame_id.c_str(), trace_.dt,
      trace_.x, trace_.y, trace_.yaw, progress_, trace_.remaining, trace_.distance, trace_.tolerance,
      trace_.yaw_error, trace_.reference, measured_yaw_rate_, trace_.direction, final_align_, capture_, escape_active_,
      trace_.head_gate, head_pan_, last_head_goal_, trace_.head_error,
      head_received_ ==
      Clock::time_point{} ? -1.0 : std::chrono::duration<double>(
        tick - head_received_).count(),
      trace_.nominal[0], trace_.nominal[1], trace_.nominal[2], trace_.requested[0],
      trace_.requested[1], trace_.requested[2],
      command.linear.x, command.linear.y, command.angular.z, obstacle_speed_limit_, fields_ready_,
      ready_received_ ==
      Clock::time_point{} ? -1.0 : std::chrono::duration<double>(tick - ready_received_).count(),
      risk_received_ ==
      Clock::time_point{} ? -1.0 : std::chrono::duration<double>(
        tick - risk_received_).count(),
      collision_, map_age, clearance_age,
      check.reason, check.layer, check.tested[0], check.tested[1], check.tested[2], check.cell_x, check.cell_y, check.value, check.time, check.separation, check.required,
      trace_.candidates, trace_.nominal_rejected, trace_.reachable_rejected,
      trace_.corridor_rejected, odometry_.linear_speed, odometry_.angular_speed,
      odometry_.span, odometry_.position_span, odometry_.stationary,
      motion_settled_since_ == Clock::time_point{} ? 0.0 :
      std::chrono::duration<double>(tick - motion_settled_since_).count(), head_wait_elapsed_,
      arrival_radius_ <= 0.0 || std::hypot(trace_.x - arrival_center_.x,
      trace_.y - arrival_center_.y) < arrival_radius_,
      trace_.raw_direction, trace_.direction_rate, trace_.head_scale);
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
    const double elapsed = std::chrono::duration<double>(tick - last_tick_).count();
    const double dt = std::clamp(elapsed, 0.001, 0.1);
    last_tick_ = tick;
    trace_ = MotionTrace{};trace_.dt = dt;
    const bool diagnostics = get_parameter("debug_detection").as_bool();
    // A single mutually-exclusive callback group serializes goals, parameters and control.
    const auto parameters = get_parameters(
      {"max_linear_speed", "max_angular_speed", "linear_acceleration",
        "angular_acceleration", "linear_jerk", "angular_jerk", "fine_dist_tolerance",
        "angle_tolerance",
        "head_pan_min", "head_pan_max", "head_alignment_tolerance", "sensor_timeout_sec"});
    const double vmax = parameters[0].as_double(), wmax = parameters[1].as_double();
    const double amax = parameters[2].as_double(), angular_accel = parameters[3].as_double();
    double tolerance = parameters[6].as_double();
    const double angle_tolerance = parameters[7].as_double();
    const double pan_min = parameters[8].as_double(), pan_max = parameters[9].as_double();
    const double head_tolerance = parameters[10].as_double(),
      sensor_timeout = parameters[11].as_double();
    const double head_speed_limit = get_parameter("head_max_pan_speed").as_double();
    const double head_acceleration_limit = get_parameter("head_max_pan_acceleration").as_double();
    const bool omni = get_parameter("motion_mode").as_string() == "omni";
    const bool use_head = get_parameter("move_head").as_bool() && !head_bypass_;
    if (use_head && !last_use_head_) {head_holding_ = true;}
    last_use_head_ = use_head;
    const bool use_fields = get_parameter("use_pot_fields").as_bool();
    if (omni != last_omni_) {
      velocity_.fill(0.0);acceleration_.fill(0.0);
      translation_direction_.reset();last_omni_ = omni;
    }
    if (relative_ && use_fields &&
      std::chrono::duration<double>(tick - last_fields_enable_).count() >= 0.2)
    {
      std_msgs::msg::Bool enable;enable.data = true;fields_enable_pub_->publish(enable);
      last_fields_enable_ = tick;
    }
    double x, y, yaw, odom_yaw, odom_x, odom_y;
    int64_t odom_stamp_ns = 0;
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
      const auto odom_tf = tf_buffer_.lookupTransform(odom_frame_, base_frame_, tf2::TimePointZero);
      const auto odom_stamp = rclcpp::Time(odom_tf.header.stamp, get_clock()->get_clock_type());
      odom_stamp_ns = odom_stamp.nanoseconds();
      if (odom_stamp.nanoseconds() != 0 &&
        std::abs((now() - odom_stamp).seconds()) > get_parameter("tf_timeout_sec").as_double())
      {
        throw std::runtime_error("Stale odometry transform");
      }
      const auto & oq = odom_tf.transform.rotation;
      odom_x = odom_tf.transform.translation.x;odom_y = odom_tf.transform.translation.y;
      odom_yaw = std::atan2(
        2.0 * (oq.w * oq.z + oq.x * oq.y),
        1.0 - 2.0 * (oq.y * oq.y + oq.z * oq.z));
    } catch (const std::exception & error) {
      finish(Status::ABORTED, "robot_tf_unavailable");
      RCLCPP_WARN(get_logger(), "%s", error.what()); return;
    }
    const auto & destination = path_.poses.back().pose;
    const auto & q = destination.orientation;
    const bool recovering = motion_kind_ == Command::RECOVER_START;
    const double final_yaw = recovering && omni ? yaw :
      std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    const double final_error = std::remainder(final_yaw - yaw, 2.0 * M_PI);
    const double distance = std::hypot(destination.position.x - x, destination.position.y - y);
    trace_.x = x;trace_.y = y;trace_.yaw = yaw;trace_.distance = distance;
    trace_.yaw_error = final_error;
    if (!initialized_) {
      initial_yaw_ = yaw;previous_yaw_ = odom_yaw;initialized_ = true;
      feasible_yaw_reference_ = yaw;
      recovery_yaw_reference_ = yaw;
      best_position_error_ = distance;
      // Avoid turning away and back during short omni transfers. Head limits
      // still request the minimum necessary base rotation before translation.
      if (omni && !relative_ && !recovering && lengths_.back() <= 1.0) {
        final_profile_ = true;final_profile_yaw_ = yaw;final_profile_turn_ = final_error;
        final_profile_distance_ = std::max(0.01, std::max(distance, lengths_.back()) - tolerance);
      }
    } else {
      const double change = std::remainder(odom_yaw - previous_yaw_, 2.0 * M_PI);
      // Preserve the odometry-frame translation/acceleration vectors while the
      // body axes rotate. Independent body-axis filters otherwise bend the path.
      for (auto * vector : {&velocity_, &acceleration_}) {
        const double vx = (*vector)[0], vy = (*vector)[1];
        (*vector)[0] = std::cos(change) * vx + std::sin(change) * vy;
        (*vector)[1] = -std::sin(change) * vx + std::cos(change) * vy;
      }
      previous_yaw_ = odom_yaw;
    }
    odometry_.observe(odom_stamp_ns, odom_x, odom_y, odom_yaw);
    measured_yaw_rate_ = odometry_.yaw_rate;
    const bool quiet_command = std::hypot(velocity_[0], velocity_[1]) < 0.01 &&
      std::abs(velocity_[2]) < 0.03;
    // Command settling and observed rest overlap; neither replaces the other.
    if (!quiet_command) {motion_settled_since_ = Clock::time_point{};}
    if (quiet_command && motion_settled_since_ == Clock::time_point{}) {motion_settled_since_ = tick;}
    const bool stopped = quiet_command && odometry_.stationary;
    const bool inside_arrival_region = arrival_radius_ <= 0.0 ||
      std::hypot(x - arrival_center_.x, y - arrival_center_.y) < arrival_radius_;
    if (!inside_arrival_region && motion_position_tolerance_ > 0.0) {
      tolerance = std::min(tolerance, motion_position_tolerance_);
    }
    trace_.tolerance = tolerance;
    const bool obstacle_ready = !(use_fields &&
      (speed_limit_received_<started_ ||
      std::chrono::duration<double>(tick - speed_limit_received_).count()> sensor_timeout ||
      !fields_ready_ ||
      ready_received_<started_ ||
      risk_received_<started_ ||
      std::chrono::duration<double>(tick - ready_received_).count()> sensor_timeout ||
      std::chrono::duration<double>(tick - risk_received_).count()> sensor_timeout));

    if (motion_kind_ == Command::OBSERVE) {
      velocity_.fill(0.0);acceleration_.fill(0.0);
      cmd_pub_->publish(geometry_msgs::msg::Twist());
      if (!use_head) {finish(Status::ABORTED, "observation_disabled");return;}
      const double pan = select_head_pan(
        std::atan2(destination.position.y - y, destination.position.x - x) - yaw, pan_min, pan_max);
      if (pan < pan_min || pan > pan_max) {
        finish(Status::ABORTED, "observation_outside_pan_range");return;
      }
      const bool fresh = head_received_ != Clock::time_point{} &&
        std::chrono::duration<double>(tick - head_received_).count() <=
        get_parameter("head_feedback_timeout_sec").as_double();
      if (!std::isfinite(last_head_goal_) || std::abs(pan - last_head_goal_) >
        get_parameter("head_goal_deadband_rad").as_double())
      {
        std_msgs::msg::Float64MultiArray goal;
        goal.data = {pan, get_parameter("head_tilt").as_double()};
        head_pub_->publish(goal);last_head_goal_ = pan;head_goal_sent_ = tick;
      }
      if (fresh && std::abs(pan - head_pan_) <= head_tolerance && stopped && obstacle_ready) {
        if (observation_aligned_since_ == Clock::time_point{}) {observation_aligned_since_ = tick;}
        if (std::chrono::duration<double>(tick - observation_aligned_since_).count() >=
          std::max(0.3, sensor_timeout))
        {
          finish(Status::SUCCEEDED, "observation_completed");return;
        }
      } else {observation_aligned_since_ = Clock::time_point{};}
      if (std::chrono::duration<double>(tick - started_).count() >
        get_parameter("head_alignment_timeout_sec").as_double())
      {
        finish(Status::ABORTED, "observation_timeout");return;
      }
      status_.text = "observing_blockage";trace_.head_gate = "observation";
      trace_.head_error = pan - head_pan_;
      log_motion(geometry_msgs::msg::Twist());return;
    }

    // Geometry and limits are immutable throughout this mutually-exclusive tick.
    local_context_.grid.reset();
    local_context_.rejection = "map_unavailable";
    local_context_.omni = omni;
    local_context_.reaction = get_parameter("collision_reaction_time_sec").as_double();
    local_context_.deceleration = amax;local_context_.angular_deceleration = angular_accel;
    local_context_.escape_radius = get_parameter("start_escape_radius").as_double();
    const double map_timeout = get_parameter("map_timeout_sec").as_double();
    if (map_ && std::chrono::duration<double>(tick - map_received_).count() <= map_timeout) {
      const auto stamp = rclcpp::Time(map_->header.stamp, get_clock()->get_clock_type());
      const bool fresh = stamp.nanoseconds() == 0 ||
        std::abs((now() - stamp).seconds()) <= map_timeout;
      local_context_.rejection = "map_stamp_stale";
      if (fresh) {
        local_context_.physical = clearance_map_ && clearance_map_->info == map_->info &&
          clearance_map_->header.frame_id == map_frame_ &&
          clearance_map_->data.size() == map_->data.size() &&
          rclcpp::Time(clearance_map_->header.stamp, get_clock()->get_clock_type()) >= stamp &&
          std::abs(
          (now() -
          rclcpp::Time(
            clearance_map_->header.stamp,
            get_clock()->get_clock_type())).seconds()) <= map_timeout;
        local_context_.grid = local_context_.physical ? clearance_map_ : map_;
        const auto & qmap = map_->info.origin.orientation;
        const double angle = std::atan2(2.0 * qmap.w * qmap.z, 1.0 - 2.0 * qmap.z * qmap.z);
        local_context_.c = std::cos(angle);local_context_.s = std::sin(angle);
      }
    }
    if (local_context_.grid && motion_kind_ == Command::BRAKE) {
      motion_constraints_.update(*local_context_.grid,
        uncertain_obstacles_ ? *uncertain_obstacles_ : empty_uncertainty_,
        get_parameter("robot_radius").as_double() + path_planner::uncertainty_margin_m,
        allow_unknown_);
    }
    if (uncertain_obstacles_) {
      if (std::chrono::duration<double>(tick - uncertainty_received_).count() > map_timeout) {
        local_context_.grid.reset();local_context_.rejection = "uncertainty_stale";
      }
    }

    if (motion_kind_ == Command::BRAKE) {
      status_.text = "braking_near_goal";
      if (use_fields && collision_) {finish(Status::ABORTED, "collision_risk");return;}
      const auto step = predict_velocity(0.0, 0.0, 0.0, dt, parameters, vmax, omni, false);
      velocity_ = step.first;acceleration_ = step.second;
      geometry_msgs::msg::Twist command;
      command.linear.x = velocity_[0];command.linear.y = velocity_[1];
      command.angular.z = velocity_[2];
      const bool free = path_.header.frame_id != map_frame_ || local_motion_is_free(
        x, y, yaw, command,
        std::numeric_limits<double>::infinity(), diagnostics ? &trace_.published : nullptr);
      if (!obstacle_ready || !free) {
        command = geometry_msgs::msg::Twist();velocity_.fill(0.0);acceleration_.fill(0.0);
        status_.text = obstacle_ready ? "waiting_for_local_clearance" : "waiting_for_obstacle_data";
        if (std::chrono::duration<double>(tick - started_).count() > sensor_timeout + 2.0) {
          finish(
            Status::ABORTED,
            obstacle_ready ? "local_trajectory_blocked" : "obstacle_data_stale");return;
        }
      }
      cmd_pub_->publish(command);log_motion(command);
      if (free && obstacle_ready && stopped && motion_settled_since_ != Clock::time_point{} &&
        std::chrono::duration<double>(tick - motion_settled_since_).count() >= 0.3)
      {
        finish(Status::SUCCEEDED, "base_stopped");
      }
      return;
    }

    bool recovery_clear = false;
    if (map_ && path_.header.frame_id == map_frame_) {
      const auto & info = map_->info;
      const double angle = 2.0 * std::atan2(info.origin.orientation.z, info.origin.orientation.w);
      const double c = std::cos(angle), sn = std::sin(angle);
      const double gx = (c * (x - info.origin.position.x) + sn * (y - info.origin.position.y)) /
        info.resolution;
      const double gy = (-sn * (x - info.origin.position.x) + c * (y - info.origin.position.y)) /
        info.resolution;
      if (gx >= 0 && gy >= 0 && gx < info.width && gy < info.height) {
        const size_t index = size_t(gy) * info.width + size_t(gx);
        if (map_->data[index] == 0) {
          escape_active_ = false;escape_checked_ = true;
          recovery_clear = local_context_.grid && local_context_.physical &&
            clearance_map_->data[index] == 0;
        } else if (!escape_checked_ && clearance_map_ && clearance_map_->info == info &&
          clearance_map_->header.frame_id == map_frame_ &&
          clearance_map_->data.size() == map_->data.size() &&
          rclcpp::Time(clearance_map_->header.stamp, get_clock()->get_clock_type()) >=
          rclcpp::Time(map_->header.stamp, get_clock()->get_clock_type()) &&
          clearance_map_->data[index] == 0 && map_->data[index] > 0)
        {
          for (size_t i = 1; i < path_.poses.size(); ++i) {
            const auto & p = path_.poses[i].pose.position;
            if (lengths_[i] > get_parameter("start_escape_radius").as_double()) {break;}
            const double px =
              (c * (p.x - info.origin.position.x) + sn * (p.y - info.origin.position.y)) /
              info.resolution;
            const double py =
              (-sn * (p.x - info.origin.position.x) + c * (p.y - info.origin.position.y)) /
              info.resolution;
            if (px >= 0 && py >= 0 && px < info.width && py < info.height &&
              map_->data[size_t(py) * info.width + size_t(px)] == 0)
            {
              escape_active_ = escape_checked_ = true;escape_end_s_ = lengths_[i];
              escape_x_ = x;escape_y_ = y;break;
            }
          }
        }
      }
    }
    if (recovering && local_context_.grid) {
      motion_constraints_.update(*map_,
        uncertain_obstacles_ ? *uncertain_obstacles_ : empty_uncertainty_,
        get_parameter("robot_radius").as_double() + path_planner::uncertainty_margin_m,
        allow_unknown_);
      recovery_clear = recovery_clear && motion_constraints_.uncertainty_free(x, y, x, y, false) &&
        motion_constraints_.sampling_free(x, y, x, y, false);
    }
    if (local_context_.grid) {
      // Preserve planning clearance while tracking. Escapes and stationary
      // alignment use the physical layer, without erasing occupied cells.
      motion_constraints_.update(escape_active_ || recovering || final_align_ || motion_kind_ == Command::ROTATE ?
        *local_context_.grid : *map_,
        uncertain_obstacles_ ? *uncertain_obstacles_ : empty_uncertainty_,
        get_parameter("robot_radius").as_double() + path_planner::uncertainty_margin_m,
        allow_unknown_);
      recovery_clear = recovery_clear && motion_constraints_.uncertainty_free(x, y, x, y, false) &&
        motion_constraints_.sampling_free(x, y, x, y, false);
    }
    if (recovering) {
      if (!recovery_clear) {recovery_clear_since_ = Clock::time_point{};}
      else if (recovery_clear_since_ == Clock::time_point{}) {recovery_clear_since_ = tick;}
      // A short escape must leave inflation before applying normal arrival tolerance.
      if (!recovery_clear) {tolerance = std::min(tolerance, 0.005);}
      trace_.tolerance = tolerance;
    }
    const bool position_reached = recovering ? recovery_clear :
      inside_arrival_region && distance <= tolerance;
    const bool recovery_stable = !recovering || (recovery_clear_since_ != Clock::time_point{} &&
      std::chrono::duration<double>(tick - recovery_clear_since_).count() >= 0.3);
    if (!escape_active_ && stopped && recovery_stable &&
      (motion_kind_ == Command::ROTATE || position_reached) &&
      std::abs(final_error) <= angle_tolerance &&
      obstacle_ready && !(use_fields && collision_) &&
      motion_settled_since_ != Clock::time_point{} &&
      std::chrono::duration<double>(tick - motion_settled_since_).count() >= 0.3)
    {
      finish(Status::SUCCEEDED, "pose_reached"); return;
    }
    if ((position_reached && !escape_active_) ||
      motion_kind_ == Command::ROTATE) {final_align_ = true;}
    if (final_align_ && (distance > tolerance || !inside_arrival_region) &&
      std::abs(final_error) <= angle_tolerance)
    {
      if (terminal_position_since_ == Clock::time_point{}) {terminal_position_since_ = tick;}
    } else {terminal_position_since_ = Clock::time_point{};}
    if (recovering && !recovery_clear) {final_align_ = false;}
    if (!recovering && motion_kind_ != Command::ROTATE && final_align_ && (escape_active_ ||
      distance > tolerance + get_parameter("final_position_hysteresis_m").as_double() ||
      (terminal_position_since_ != Clock::time_point{} &&
      std::chrono::duration<double>(tick - terminal_position_since_).count() >=
      get_parameter("final_position_settle_sec").as_double()))) {final_align_ = false;}
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
    const double deviation_limit = get_parameter(
      route_obstructed_ || local_detour_ || escape_side_ != 0 ?
      "max_avoidance_deviation_m" : "max_path_deviation_m").as_double();
    if (nearest > deviation_limit && path_.poses.size() > 1) {
      if (deviation_since_ == Clock::time_point{}) {deviation_since_ = tick;}
      if (std::chrono::duration<double>(tick - deviation_since_).count() >
        get_parameter("deviation_persistence_sec").as_double())
      {
        finish(Status::ABORTED, "path_deviation"); return;
      }
    } else {deviation_since_ = Clock::time_point{};}
    double lookahead = std::clamp(
      get_parameter("lookahead_min_m").as_double() +
      std::hypot(velocity_[0], velocity_[1]) * get_parameter("lookahead_time_sec").as_double(),
      get_parameter("lookahead_min_m").as_double(), get_parameter("lookahead_max_m").as_double());
    double target_s = std::min(
      escape_active_ ? escape_end_s_ : lengths_.back(), progress_ + lookahead);
    const double minimum_target_s = std::min(
      target_s, progress_ + get_parameter("lookahead_min_m").as_double());
    double target_x = destination.position.x, target_y = destination.position.y;
    double tangent = std::atan2(target_y - y, target_x - x);
    // A long lookahead chord can cut across a valid planned corner. Shorten
    // it until the connector is free rather than repeatedly stopping at the cut.
    for (int attempt = 0; attempt < 5; ++attempt) {
      for (size_t i = std::max(size_t{1}, segment); i < path_.poses.size(); ++i) {
        if (lengths_[i] + 1e-9 < target_s) {continue;}
        const auto & a = path_.poses[i - 1].pose.position;
        const auto & b = path_.poses[i].pose.position;
        const double t = (target_s - lengths_[i - 1]) /
          std::max(1e-9, lengths_[i] - lengths_[i - 1]);
        target_x = a.x + t * (b.x - a.x); target_y = a.y + t * (b.y - a.y);
        tangent = std::atan2(b.y - a.y, b.x - a.x); break;
      }
      const double target_distance = std::hypot(target_x - x, target_y - y);
      if (path_.header.frame_id != map_frame_ || target_distance < 0.05) {break;}
      geometry_msgs::msg::Twist probe;
      const double bearing = std::atan2(target_y - y, target_x - x) - yaw;
      const double probe_speed = std::max(vmax, std::sqrt(2.0 * amax * target_distance) + 0.1);
      probe.linear.x = probe_speed * std::cos(bearing);probe.linear.y = probe_speed * std::sin(
        bearing);
      if (local_motion_is_free(x, y, yaw, probe, target_distance)) {break;}
      if (target_s <= minimum_target_s + 1e-9) {break;}
      if (attempt < 4) {
        target_s = std::max(minimum_target_s, progress_ + 0.5 * (target_s - progress_));
      }
    }
    lookahead = std::max(0.05, target_s - progress_);
    double direction = std::atan2(target_y - y, target_x - x);
    const double remaining = std::max(distance, lengths_.back() - progress_);
    trace_.remaining = remaining;
    capture_ = false;
    if (omni && !relative_ && !escape_active_ &&
      remaining <= get_parameter("final_capture_distance_m").as_double())
    {
      geometry_msgs::msg::Twist direct;
      const double bearing = std::atan2(destination.position.y - y, destination.position.x - x);
      const double probe_speed = std::max(vmax, std::sqrt(2.0 * amax * distance) + 0.1);
      direct.linear.x = probe_speed * std::cos(bearing - yaw);
      direct.linear.y = probe_speed * std::sin(bearing - yaw);
      capture_ = path_.header.frame_id != map_frame_ ||
        local_motion_is_free(x, y, yaw, direct, distance);
      if (capture_) {direction = bearing;}
    }
    const double final_turn = final_profile_ ? std::abs(final_profile_turn_) :
      std::abs(std::remainder(final_yaw - final_heading_, 2.0 * M_PI));
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
    double speed = std::min(vmax, std::sqrt(2.0 * amax * std::max(0.0, remaining - tolerance)));
    speed = std::min(speed, 2.0 * distance);
    // Omni rejoining can translate laterally without following the base's
    // turning circle. The path-curvature braking envelope below still applies.
    const double curvature = omni ? 0.0 : 2.0 * std::abs(std::sin(tangent - direction)) / std::max(
      0.05,
      lookahead);
    const double curve_limit = std::min(
      std::sqrt(amax / std::max(1e-6, curvature)), wmax / std::max(
        1e-6, curvature));
    speed = std::min(speed, curve_limit);
    // Brake before a bend enters the lookahead, allowing full speed on clear straights.
    // A reaction/jerk reserve prevents the velocity filter from arriving too fast.
    const double reserve = get_parameter("collision_reaction_time_sec").as_double() +
      amax / parameters[4].as_double();
    for (size_t i = std::max(size_t{1}, segment); i + 1 < path_.poses.size(); ++i) {
      if (lengths_[i] < progress_) {continue;}
      if (lengths_[i] - progress_ > vmax * vmax / (2.0 * amax) + vmax * reserve + lookahead) {
        break;
      }
      const auto & a = path_.poses[i - 1].pose.position;
      const auto & b = path_.poses[i].pose.position;
      const auto & c = path_.poses[i + 1].pose.position;
      const double l1 = std::hypot(b.x - a.x, b.y - a.y), l2 = std::hypot(c.x - b.x, c.y - b.y);
      if (std::min(l1, l2) < 1e-5) {continue;}
      const double turn = std::abs(
        std::remainder(
          std::atan2(c.y - b.y, c.x - b.x) -
          std::atan2(b.y - a.y, b.x - a.x), 2.0 * M_PI));
      const double k = turn / (0.5 * (l1 + l2));
      const double corner_speed = std::min(
        std::sqrt(amax / std::max(1e-6, k)),
        wmax / std::max(1e-6, k));
      const double available = std::max(
        0.0, lengths_[i] - progress_ -
        std::hypot(velocity_[0], velocity_[1]) * reserve);
      speed = std::min(speed, std::sqrt(corner_speed * corner_speed + 2.0 * amax * available));
    }
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
    if (escape_active_ || recovering) {speed = std::min(speed, get_parameter("start_escape_speed").as_double());}
    double vx = speed * std::cos(direction - yaw), vy = speed * std::sin(direction - yaw);
    if (use_fields &&
      std::chrono::duration<double>(tick - force_received_).count() < sensor_timeout)
    {
      const double limit = get_parameter("repulsion_max_speed").as_double();
      const double scale = std::min(1.0, limit / std::max(1e-9, std::hypot(force_.x, force_.y)));
      const double c = std::cos(direction - yaw), s = std::sin(direction - yaw);
      // Prefer a lateral correction. Repulsion must not reverse/cancel the route
      // velocity or dominate its direction during the slow final approach.
      const double along = std::clamp(
        scale * (c * force_.x + s * force_.y), -0.5 * speed,
        0.25 * speed);
      const double lateral = std::clamp(
        scale * (-s * force_.x + c * force_.y), -0.6 * speed,
        0.6 * speed);
      vx += c * along - s * lateral; vy += s * along + c * lateral;
      if (std::hypot(vx, vy) > 1e-5) {direction = yaw + std::atan2(vy, vx);}
    }
    double yaw_feedforward = 0.0;
    const double initial_fraction = get_parameter("initial_alignment_fraction").as_double();
    const double initial_turn = std::abs(std::remainder(tangent - initial_yaw_, 2.0 * M_PI));
    const double initial_duration = std::max(
      {0.1, initial_turn / (0.6 * wmax), std::sqrt(6.0 * initial_turn / angular_accel),
        std::cbrt(12.0 * initial_turn / parameters[5].as_double())});
    const double initial_window = std::max(
      initial_fraction * lengths_.back(),
      std::min(0.45 * lengths_.back(), 0.5 * vmax * initial_duration));
    if (obstacle_ready && !escape_active_ && (!use_head || !head_holding_) && !route_obstructed_ &&
      (!use_fields || obstacle_speed_limit_ > 0.03)) {initial_elapsed_ += dt;final_elapsed_ += dt;}
    double blend = std::clamp(
      std::max(progress_ / std::max(0.01, initial_window), initial_elapsed_ / initial_duration),
      0.0, 1.0);
    const double initial_phase = blend;
    blend = blend * blend * (3.0 - 2.0 * blend);
    double desired_yaw = initial_yaw_ + blend * std::remainder(tangent - initial_yaw_, 2.0 * M_PI);
    // Complete the angular profile at the positional tolerance boundary.
    // A fixed distance window avoids chasing a shrinking window while braking.
    if (!final_profile_ && (remaining <= final_window || capture_)) {
      final_profile_ = true;
      final_profile_yaw_ = desired_yaw;
      final_profile_turn_ = std::remainder(final_yaw - desired_yaw, 2.0 * M_PI);
      final_profile_distance_ = std::max(0.01, remaining - tolerance);
      final_elapsed_ = 0.0;
    }
    if (final_profile_) {
      final_progress_ = std::max(
        final_progress_, std::clamp(
          1.0 - std::max(0.0, remaining - tolerance) / final_profile_distance_, 0.0, 1.0));
      final_progress_ = std::max(
        final_progress_, std::min(
          1.0, final_elapsed_ /
          std::max(0.1, 1.5 * turn_time)));
    }
    double final_blend = final_progress_;
    const double final_phase = final_blend;
    final_blend = final_blend * final_blend * (3.0 - 2.0 * final_blend);
    if (final_profile_) {desired_yaw = final_profile_yaw_ + final_blend * final_profile_turn_;}
    if (omni && !relative_) {
      // Bound translational progress by the angular rate required by the yaw
      // profile. Short routes slow down instead of exhausting the first/last
      // ten percent before the base can turn.
      const double initial_slope = final_profile_ ? 0.0 : 6.0 * initial_phase *
        (1.0 - initial_phase) *
        std::abs(std::remainder(tangent - initial_yaw_, 2.0 * M_PI)) /
        std::max(0.01, initial_window);
      const double final_slope = final_profile_ ? 6.0 * final_phase * (1.0 - final_phase) *
        std::abs(final_profile_turn_) / final_profile_distance_ : 0.0;
      const bool phase_active = obstacle_ready && !escape_active_ &&
        (!use_head || !head_holding_) && !route_obstructed_ &&
        (!use_fields || obstacle_speed_limit_ > 0.03);
      const double initial_rate = std::max(
        std::hypot(velocity_[0], velocity_[1]) /
        std::max(0.01, initial_window), phase_active ? 1.0 / initial_duration : 0.0);
      const double final_rate = std::max(
        std::hypot(velocity_[0], velocity_[1]) /
        final_profile_distance_, phase_active ? 1.0 / std::max(0.1, 1.5 * turn_time) : 0.0);
      yaw_feedforward =
        (final_profile_ ? 0.0 : 1.0) * 6.0 * initial_phase * (1.0 - initial_phase) *
        std::remainder(
        tangent - initial_yaw_,
        2.0 * M_PI) * initial_rate +
        6.0 * final_phase * (1.0 - final_phase) *
        (final_profile_ ? final_profile_turn_ * final_rate : 0.0);
      const double limited_speed =
        std::min(
        {vmax, final_turn > angle_tolerance ? approach_limit : vmax,
          0.6 * wmax / std::max(1e-6, initial_slope + final_slope)});
      const double scale = std::min(
        1.0, limited_speed / std::max(1e-6, std::hypot(vx, vy)));
      vx *= scale;vy *= scale;
      double alignment_limit = vmax;
      const double goal_pan = select_head_pan(direction - final_yaw, pan_min, pan_max);
      const bool goal_heading_visible = !use_head || (goal_pan >= pan_min && goal_pan <= pan_max);
      if (final_profile_ && goal_heading_visible &&
        std::abs(final_error) > angle_tolerance + 0.05)
      {
        const double time = amax / parameters[4].as_double() + std::max(
          std::abs(final_error) / (0.7 * wmax),
          std::sqrt(2.0 * std::abs(final_error) / angular_accel));
        alignment_limit = std::min(alignment_limit, std::max(0.0, remaining - tolerance) / time);
      }
      const double alignment_scale =
        std::min(1.0, alignment_limit / std::max(1e-6, std::hypot(vx, vy)));
      vx *= alignment_scale;vy *= alignment_scale;
    }
    if (escape_active_) {desired_yaw = initial_yaw_;yaw_feedforward = 0.0;}
    if (recovering && omni) {desired_yaw = recovery_yaw_reference_;yaw_feedforward = 0.0;}
    double heading_error = std::remainder(direction - yaw - (reverse_ ? M_PI : 0.0), 2.0 * M_PI);
    if (relative_) {desired_yaw = final_yaw;}
    const double angular_gain = 1.0 / std::max(0.05, get_parameter("control_beta").as_double());
    double angular = std::clamp(
      yaw_feedforward + angular_gain * std::remainder(
        desired_yaw - yaw,
        2.0 * M_PI) - get_parameter("angular_damping").as_double() *
      (measured_yaw_rate_ - yaw_feedforward), -wmax, wmax);
    if (!omni) {
      angular = std::clamp(angular_gain * heading_error, -wmax, wmax);
      vx = (reverse_ ? -1.0 : 1.0) * std::hypot(vx, vy) * std::max(0.0, std::cos(heading_error));
      vy = 0.0;
    }
    if (final_align_) {
      vx = vy = 0.0;
      // Stop inside the requested tolerance so the rest check can complete.
      // Brief localization excursions must not restart a terminal correction.
      if (std::abs(final_error) <= angle_tolerance) {
        terminal_angle_holding_ = true;terminal_angle_since_ = Clock::time_point{};
      } else if (terminal_angle_holding_) {
        if (terminal_angle_since_ == Clock::time_point{}) {terminal_angle_since_ = tick;}
        if (std::abs(final_error) > 1.5 * angle_tolerance ||
          std::chrono::duration<double>(tick - terminal_angle_since_).count() >=
          get_parameter("final_position_settle_sec").as_double()) {terminal_angle_holding_ = false;}
      }
      const double limit = get_parameter("final_angle_max_angular_speed").as_double();
      angular = terminal_angle_holding_ ? 0.0 : std::clamp(
        2.0 * final_error - get_parameter("angular_damping").as_double() *
        measured_yaw_rate_, -limit, limit);
    } else {terminal_angle_holding_ = false;terminal_angle_since_ = Clock::time_point{};}
    bool local_holding = false;
    trace_.nominal = {vx, vy, angular};
    bool avoiding = route_obstructed_ || local_detour_;
    if (path_.header.frame_id == map_frame_ && !final_align_) {
      geometry_msgs::msg::Twist proposed;
      proposed.linear.x = vx;proposed.linear.y = vy;proposed.angular.z = angular;
      const bool direct_free = local_motion_is_free(
        x, y, yaw, proposed,
        std::numeric_limits<double>::infinity(), diagnostics ? &trace_.rejected : nullptr);
      if (capture_ && direct_free && !route_obstructed_ && (!use_head || !head_holding_)) {
        // Capture already checked the full connector to the goal. Retaining an
        // old detour here can carry the robot past the endpoint indefinitely.
        local_detour_ = false;escape_side_ = 0;avoiding = false;
        local_direction_ = std::numeric_limits<double>::quiet_NaN();
        challenger_since_ = Clock::time_point{};
      }
      if (!direct_free || route_obstructed_) {local_detour_ = true;avoiding = true;}
      if (escape_side_ != 0) {
        const double target_distance = std::hypot(target_x - x, target_y - y);
        const double probe_speed = std::max(vmax, std::sqrt(2.0 * amax * target_distance) + 0.1);
        geometry_msgs::msg::Twist corridor;
        const double bearing = std::atan2(target_y - y, target_x - x) - yaw;
        corridor.linear.x = probe_speed * std::cos(bearing);
        corridor.linear.y = probe_speed * std::sin(bearing);
        if (local_motion_is_free(x, y, yaw, corridor, target_distance)) {escape_side_ = 0;}
      }
      if (!direct_free || route_obstructed_ || local_detour_ || escape_side_ != 0) {
        avoiding = true;
        const double nominal = std::min(
          {escape_active_ || recovering ? std::min(vmax, get_parameter("start_escape_speed").as_double()) : vmax,
            std::max(
              std::hypot(vx, vy),
              final_profile_ && std::abs(final_error) > angle_tolerance ? 0.0 :
              get_parameter("local_avoidance_speed").as_double()),
            std::sqrt(2.0 * amax * std::max(0.0, remaining - tolerance))});
        const double bearing = tangent - yaw;
        const double rejoin_offset = std::remainder(direction - tangent, 2.0 * M_PI);
        double best_score = -std::numeric_limits<double>::infinity();
        double retained_score = -std::numeric_limits<double>::infinity();
        double side_score = -std::numeric_limits<double>::infinity();
        geometry_msgs::msg::Twist best, retained, same_side;
        double best_offset = 0.0, side_offset = 0.0;
        const double retained_offset = std::isfinite(local_direction_) && omni ?
          std::remainder(local_direction_ - odom_yaw - bearing, 2.0 * M_PI) : rejoin_offset;
        const std::array<double, 15> offsets = {retained_offset, rejoin_offset, 0.0, 0.30,
          -0.30, 0.60, -0.60, 0.95, -0.95, 1.35, -1.35, M_PI / 2.0, -M_PI / 2.0, 2.1, -2.1};
        std::array<double, 15> cos_offset{}, cos_direction{}, sin_direction{}, steering_cost{};
        const double base_cos = std::cos(yaw), base_sin = std::sin(yaw);
        const double preview_limit = use_fields ? std::min(vmax, obstacle_speed_limit_) : vmax;
        // Direction costs and trigonometry are shared by all four speed fractions.
        for (size_t i = 0; i < offsets.size(); ++i) {
          cos_offset[i] = std::cos(offsets[i]);
          cos_direction[i] = std::cos(bearing + offsets[i]);
          sin_direction[i] = std::sin(bearing + offsets[i]);
          if (use_head) {
            const double pan = select_head_pan(bearing + offsets[i], pan_min, pan_max);
            const double error = std::clamp(pan, pan_min, pan_max) - head_pan_;
            const double along_rate = std::copysign(1.0, error) * measured_head_rate_;
            const double braking = along_rate > 0.0 ? along_rate * along_rate / (2.0 * head_acceleration_limit) : 0.0;
            const double travel = std::max(0.0, std::abs(error) - head_tolerance);
            // Include counter-motion and stopping distance when a reversal would overshoot.
            const double alignment_time = std::max(travel / head_speed_limit,
              std::sqrt(2.0 * travel / head_acceleration_limit)) +
              (along_rate < 0.0 || braking > travel ? std::abs(along_rate) / head_acceleration_limit : 0.0);
            steering_cost[i] = nominal * (0.35 * alignment_time +
              std::abs(pan - std::clamp(pan, pan_min, pan_max)));
          }
          if (std::isfinite(local_direction_)) {
            steering_cost[i] += 0.5 * nominal * std::abs(std::remainder(
              bearing + offsets[i] + odom_yaw - local_direction_, 2.0 * M_PI));
          }
        }
        for (double fraction : {1.0, 0.75, 0.5, 0.25}) {
          for (size_t candidate_index = 0; candidate_index < offsets.size(); ++candidate_index)
          {
            const double offset = offsets[candidate_index];
            ++trace_.candidates;
            geometry_msgs::msg::Twist candidate;
            if (omni) {
              candidate.linear.x = nominal * fraction * cos_direction[candidate_index];
              candidate.linear.y = nominal * fraction * sin_direction[candidate_index];
              candidate.angular.z = angular;
            } else {
              candidate.linear.x = vx * fraction;
              candidate.angular.z = std::clamp(angular + 2.0 * offset, -wmax, wmax);
            }
            if (!local_motion_is_free(
                x, y, yaw, candidate, std::numeric_limits<double>::infinity(),
                diagnostics ? &trace_.rejected : nullptr)) {++trace_.nominal_rejected;continue;}
            const auto reachable = predict_velocity(
              candidate.linear.x, candidate.linear.y,
              candidate.angular.z, dt, parameters, preview_limit, omni, false);
            geometry_msgs::msg::Twist next_command;
            next_command.linear.x = reachable.first[0];next_command.linear.y = reachable.first[1];
            next_command.angular.z = reachable.first[2];
            if (!local_motion_is_free(
                x, y, yaw, next_command, std::numeric_limits<double>::infinity(),
                diagnostics ? &trace_.rejected : nullptr)) {++trace_.reachable_rejected;continue;}
            if (omni && route_obstructed_ && !escape_active_) {
              // A tiny braking prefix must not justify creeping towards the
              // same obstacle. Select a direction with room to make a detour.
              const double candidate_speed = nominal * fraction;
              const double corridor_distance = std::min(
                remaining, std::max(
                  0.10,
                  candidate_speed * local_context_.reaction +
                  candidate_speed * candidate_speed / (2.0 * amax) + map_->info.resolution));
              const double probe_speed = std::max(
                vmax, std::sqrt(
                  2.0 * amax * corridor_distance) + 0.1);
              geometry_msgs::msg::Twist probe;
              probe.linear.x = probe_speed * cos_direction[candidate_index];
              probe.linear.y = probe_speed * sin_direction[candidate_index];
              if (!local_motion_is_free(
                  x, y, yaw, probe, corridor_distance,
                  diagnostics ? &trace_.rejected : nullptr)) {++trace_.corridor_rejected;continue;}
            }
            // Favor commands reachable from the current motion, not just an
            // individually free endpoint that would require an abrupt direction change.
            const double change = std::hypot(
              candidate.linear.x - velocity_[0],
              candidate.linear.y - velocity_[1]);
            // Score a short endpoint against nearby route segments. Tangential
            // advance alone rewards a parallel detour indefinitely.
            const double preview_time = std::min(1.0, lookahead / std::max(0.05, nominal));
            const double world_x = base_cos * candidate.linear.x - base_sin * candidate.linear.y;
            const double world_y = base_sin * candidate.linear.x + base_cos * candidate.linear.y;
            const double end_x = x + preview_time * world_x, end_y = y + preview_time * world_y;
            double cross_track = std::numeric_limits<double>::infinity();
            for (size_t i = std::max(size_t{1}, segment); i < path_.poses.size(); ++i) {
              if (lengths_[i - 1] > progress_ + lookahead + nominal * preview_time) {break;}
              const auto & a = path_.poses[i - 1].pose.position;
              const auto & b = path_.poses[i].pose.position;
              const double dx = b.x - a.x, dy = b.y - a.y;
              const double projection = std::clamp(
                ((end_x - a.x) * dx + (end_y - a.y) * dy) / std::max(1e-12, dx * dx + dy * dy), 0.0, 1.0);
              cross_track = std::min(cross_track, std::hypot(end_x - a.x - projection * dx, end_y - a.y - projection * dy));
            }
            if (!std::isfinite(cross_track)) {cross_track = std::hypot(end_x - target_x, end_y - target_y);}
            const double score = 4.0 * nominal * fraction * cos_offset[candidate_index] +
              0.5 * nominal * fraction - 0.15 * nominal * std::abs(offset) -
              6.0 * cross_track / preview_time -
              2.0 * change - 0.25 * std::abs(candidate.angular.z - velocity_[2]) -
              steering_cost[candidate_index] -
              (escape_side_ * offset < -0.01 ? 0.35 * nominal : 0.0);
            if (score > best_score) {best_score = score;best = candidate;best_offset = offset;}
            if (escape_side_ * offset >= -0.01 && score > side_score) {
              side_score = score;same_side = candidate;side_offset = offset;
            }
            if (candidate_index == 0 && score > retained_score) {
              retained_score = score;retained = candidate;
            }
          }
        }
        if (std::isfinite(best_score)) {
          // Keep the chosen side of an obstacle until the route connector opens.
          // Otherwise cross-track cost can repeatedly pull a lateral detour back
          // across the route before it has passed the obstacle. An unsafe retained
          // direction still permits an immediate change to any safe candidate.
          if (omni && !(use_head && head_holding_) && escape_side_ * best_offset < -0.01 && std::isfinite(side_score))
          {
            best = same_side;best_offset = side_offset;best_score = side_score;
          }
          // Both choices pass the same geometry and braking checks above. Holding
          // a direction never preserves an unsafe command or a body-frame twist.
          const double candidate_direction = odom_yaw + std::atan2(best.linear.y, best.linear.x);
          if (omni && use_head && head_holding_ && std::isfinite(local_direction_) && std::isfinite(retained_score)) {
            // Finish observing a viable direction before requesting another pan.
            best = retained;best_offset = retained_offset;challenger_since_ = Clock::time_point{};
          } else if (omni && std::isfinite(local_direction_) && std::isfinite(retained_score) &&
            std::abs(std::remainder(candidate_direction - local_direction_, 2.0 * M_PI)) > 0.10)
          {
            if (best_score <= retained_score + get_parameter("avoidance_switch_score_margin").as_double()) {
              challenger_since_ = Clock::time_point{};
            } else if (challenger_since_ == Clock::time_point{} ||
              std::abs(std::remainder(candidate_direction - challenger_direction_, 2.0 * M_PI)) > 0.10)
            {
              challenger_direction_ = candidate_direction;challenger_since_ = tick;
            }
            if (challenger_since_ == Clock::time_point{} ||
              std::chrono::duration<double>(tick - challenger_since_).count() <
              get_parameter("avoidance_switch_hold_sec").as_double())
            {
              best = retained;best_offset = retained_offset;
            } else {challenger_since_ = Clock::time_point{};}
          } else {challenger_since_ = Clock::time_point{};}
          vx = best.linear.x;vy = best.linear.y;angular = best.angular.z;
          if (std::hypot(vx, vy) > 1e-5) {local_direction_ = odom_yaw + std::atan2(vy, vx);}
          if (direct_free && !route_obstructed_ &&
            std::abs(std::remainder(best_offset - rejoin_offset, 2.0 * M_PI)) < 0.10)
          {
            local_detour_ = false;escape_side_ = 0;
          }
          else if (escape_side_ == 0 && std::abs(best_offset) > 0.1) {
            escape_side_ = best_offset > 0.0 ? 1 : -1;
          }
          if (std::hypot(vx, vy) > 1e-5) {direction = yaw + std::atan2(vy, vx);}
        } else {
          vx = vy = angular = 0.0;local_holding = true;
          // No viable command is not a new direction. Keep the last safe
          // observation bearing while waiting for clearance or replanning.
          challenger_since_ = Clock::time_point{};
          if (std::isfinite(local_direction_)) {direction = yaw + local_direction_ - odom_yaw;}
        }
      }
    }
    if (std::hypot(vx, vy) > 1e-6) {direction = yaw + std::atan2(vy, vx);}
    trace_.raw_direction = direction;
    if (omni && !relative_ && !final_align_ && !local_holding && std::hypot(vx, vy) > 1e-6) {
      const double magnitude = std::hypot(vx, vy);
      const double raw = odom_yaw + std::atan2(vy, vx);
      const double filtered = translation_direction_.update(
        raw, dt, get_parameter("direction_filter_time_sec").as_double(),
        get_parameter("direction_max_rate_rad_sec").as_double());
      geometry_msgs::msg::Twist candidate;
      candidate.linear.x = magnitude * std::cos(filtered - odom_yaw);
      candidate.linear.y = magnitude * std::sin(filtered - odom_yaw);
      candidate.angular.z = angular;
      // A smoothed bearing must never turn an executable detour into an
      // occupied connector. The final limited command is checked again below.
      if (path_.header.frame_id != map_frame_ || local_motion_is_free(x, y, yaw, candidate)) {
        vx = candidate.linear.x;vy = candidate.linear.y;
      } else {
        translation_direction_.reset(raw);
      }
      direction = yaw + std::atan2(vy, vx);
      trace_.direction_rate = translation_direction_.rate;
    } else {
      translation_direction_.rate = 0.0;
    }
    if (omni && use_head && !final_align_) {
      const double reserve = std::min(0.5 * head_tolerance, 0.25 * (pan_max - pan_min));
      // Project only after avoidance and direction filtering, using the same
      // bearing that will drive the camera and translation on this cycle.
      const double reference_pan = select_head_pan(direction - yaw, pan_min, pan_max) +
        std::remainder(yaw - desired_yaw, 2.0 * M_PI);
      const double feasible_pan = std::clamp(reference_pan, pan_min + reserve, pan_max - reserve);
      const double correction = reference_pan - feasible_pan;
      const double projected = desired_yaw + correction;
      const double rate = std::min(wmax, 0.8 * get_parameter("head_max_pan_speed").as_double());
      feasible_yaw_reference_ +=
        std::clamp(
        std::remainder(
          projected - feasible_yaw_reference_,
          2.0 * M_PI), -rate * dt, rate * dt);
      desired_yaw = feasible_yaw_reference_;
      if (recovering) {recovery_yaw_reference_ = desired_yaw;}
      if (std::abs(correction) > 1e-6) {yaw_feedforward = 0.0;}
      angular = std::clamp(
        yaw_feedforward + angular_gain * std::remainder(desired_yaw - yaw, 2.0 * M_PI) -
        get_parameter(
          "angular_damping").as_double() * (measured_yaw_rate_ - yaw_feedforward), -wmax, wmax);
    }
    bool head_blocked = false;
    trace_.direction = direction;trace_.reference = desired_yaw;
    if (use_head && !final_align_) {
      const double pan = select_head_pan(direction - yaw, pan_min, pan_max);
      const double clamped = std::clamp(pan, pan_min, pan_max);
      const double preview_s = std::min(
        lengths_.back(), target_s +
        std::hypot(velocity_[0], velocity_[1]) * get_parameter(
          "head_lookahead_time_sec").as_double());
      double preview = direction;
      const double preview_angle = std::min(
        {get_parameter("head_preview_angle_rad").as_double(),
          0.5 * get_parameter("head_stop_error_rad").as_double(),
          get_parameter("head_max_pan_speed").as_double() *
          get_parameter("head_lookahead_time_sec").as_double()});
      for (size_t i = std::max(size_t{1}, segment); i < path_.poses.size(); ++i) {
        if (avoiding || escape_active_) {break;}
        if (lengths_[i] < preview_s) {continue;}
        const auto & p = path_.poses[i].pose.position;
        preview += std::clamp(
          std::remainder(
            std::atan2(p.y - y, p.x - x) - direction,
            2.0 * M_PI), -preview_angle, preview_angle);
        break;
      }
      const double preview_pan = select_head_pan(
        preview - yaw - measured_yaw_rate_ * 0.10, pan_min,
        pan_max);
      const bool fresh = head_received_ != Clock::time_point{} &&
      std::chrono::duration<double>(tick - head_received_).count() <= get_parameter(
        "head_feedback_timeout_sec").as_double();
      const double error = std::abs(pan - head_pan_);
      const double stop_error = get_parameter("head_stop_error_rad").as_double();
      const double limit_margin = get_parameter("head_pan_limit_margin").as_double();
      if (std::abs(pan - clamped) > 0.01) {
        head_limit_holding_ = true;
      } else if (pan >= pan_min + limit_margin && pan <= pan_max - limit_margin) {
        head_limit_holding_ = false;
      }
      if (!fresh || error >= stop_error || head_limit_holding_) {
        head_holding_ = true;
      } else if (error <= head_tolerance) {head_holding_ = false;}
      head_blocked = head_holding_;
      trace_.head_scale = head_blocked ? 0.0 : 1.0;
      if (head_blocked) {head_wait_elapsed_ += elapsed;}
      trace_.head_error = pan - head_pan_;
      trace_.head_gate = !fresh ? "feedback_stale" : head_limit_holding_ ? "pan_limit" :
        head_holding_ ? "alignment" : "tracking";
      std_msgs::msg::Float64MultiArray goal;
      goal.data =
      {std::clamp(
          head_holding_ ? pan : std::clamp(
            preview_pan, pan - 0.5 * head_tolerance,
            pan + 0.5 * head_tolerance), pan_min, pan_max),
        get_parameter("head_tilt").as_double()};
      const double head_deadband = final_profile_ && std::hypot(vx, vy) < 0.05 && !head_holding_ ?
        std::max(get_parameter("head_goal_deadband_rad").as_double(), 0.5 * head_tolerance) :
        get_parameter("head_goal_deadband_rad").as_double();
      if (!std::isfinite(last_head_goal_) ||
        std::abs(goal.data[0] - last_head_goal_) > head_deadband ||
        goal.data[1] != last_head_tilt_ || (head_holding_ &&
        std::chrono::duration<double>(tick - head_goal_sent_).count() > 0.25))
      {
        head_pub_->publish(goal);last_head_goal_ = goal.data[0];last_head_tilt_ = goal.data[1];
        head_goal_sent_ = tick;
      }
      if (head_blocked) {
        if (progress_ < 0.001 && std::abs(pan - clamped) > 0.01) {initial_yaw_ = yaw;}
        vx = vy = 0.0;
        // Recover inside the pan range, not exactly on the switching boundary.
        const double recovery_margin = std::min(
          2.0 * limit_margin, 0.25 * (pan_max - pan_min) + 0.5 * limit_margin);
        const double recovery_pan = std::clamp(
          pan, pan_min + recovery_margin,
          pan_max - recovery_margin);
        angular = fresh && head_limit_holding_ ?
          std::clamp(2.0 * (pan - recovery_pan), -wmax, wmax) : 0.0;
        if (head_wait_elapsed_ >
          get_parameter("head_alignment_timeout_sec").as_double())
        {
          finish(Status::ABORTED, "head_alignment_timeout"); return;
        }
      } else {
        const double required_pan_rate = translation_direction_.rate - measured_yaw_rate_;
        const double scale = simple_move::head_tracking_scale(
          error, head_tolerance, stop_error, required_pan_rate, head_speed_limit);
        trace_.head_scale = scale;
        trace_.head_gate = scale < 0.99 ? "slowing" : "tracking";
        vx *= scale;vy *= scale;
        if (angular * (pan - head_pan_) < 0.0) {
          angular *= scale;
        }
        // Reserve head tracking speed for the body's counter-rotation.
        const double head_speed = get_parameter("head_max_pan_speed").as_double();
        angular = std::clamp(angular, -0.8 * head_speed, 0.8 * head_speed);
      }
    }
    const double terminal_pan = select_head_pan(direction - final_yaw, pan_min, pan_max);
    if (!recovering && final_profile_ && !head_blocked &&
      (!use_head || final_align_ || (terminal_pan >= pan_min && terminal_pan <= pan_max)))
    {
      // Leave angular braking distance, including the jerk ramp, before the
      // fixed goal orientation. Clipping only the tracking error brakes too late.
      const double reserve = angular_accel * angular_accel / parameters[5].as_double();
      const double limit = std::max(
        0.0,
        std::sqrt(reserve * reserve + 2.0 * angular_accel * std::abs(final_error)) - reserve);
      angular = std::clamp(angular, -limit, limit);
    }
    const bool terminal_region = final_align_ || motion_kind_ == Command::ROTATE ||
      (!recovering && distance <= get_parameter("final_capture_distance_m").as_double());
    if (!terminal_region) {best_terminal_error_ = std::abs(final_error);}
    const bool angular_progress = terminal_region && std::abs(final_error) + 0.02 < best_terminal_error_;
    const bool route_progress = progress_ > best_progress_ + 0.02;
    const bool position_progress = distance + 0.02 < best_position_error_;
    if (route_progress || position_progress || angular_progress || !obstacle_ready) {
      progress_time_ = tick;
      if (route_progress) {best_progress_ = progress_;}
      if (position_progress) {best_position_error_ = distance;}
      if (angular_progress) {best_terminal_error_ = std::abs(final_error);}
      if (route_progress || position_progress || angular_progress) {head_wait_elapsed_ = 0.0;}
    } else if (std::chrono::duration<double>(tick - progress_time_).count() >
      std::max(get_parameter("progress_timeout_sec").as_double(),
      head_wait_elapsed_ > 0.0 ? get_parameter("head_alignment_timeout_sec").as_double() : 0.0))
    {
      finish(Status::ABORTED,
        final_align_ || motion_kind_ == Command::ROTATE ? "orientation_stalled" : "position_stalled");
      return;
    }
    geometry_msgs::msg::Vector3 intent;
    intent.x = std::cos(direction - yaw); intent.y = std::sin(direction - yaw);
    intent_pub_->publish(intent);
    if (intent_started_ == Clock::time_point{}) {intent_started_ = tick;}
    if (use_fields && obstacle_ready &&
      (std::chrono::duration<double>(tick - intent_started_).count() < 0.06 ||
      risk_received_ < intent_started_))
    {
      // The first risk sample may describe the previous direction. Wait for
      // the monitor to evaluate this motion before translating or aborting it.
      velocity_.fill(0.0);acceleration_.fill(0.0);
      status_.text = "checking_motion_direction";
      cmd_pub_->publish(geometry_msgs::msg::Twist());log_motion(geometry_msgs::msg::Twist());return;
    }
    if (!obstacle_ready) {
      status_.text = "waiting_for_obstacle_data";
      if (obstacle_stale_since_ == Clock::time_point{}) {obstacle_stale_since_ = tick;}
      cmd_pub_->publish(geometry_msgs::msg::Twist());
      velocity_.fill(0.0); acceleration_.fill(0.0);
      log_motion(geometry_msgs::msg::Twist());
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(), 2000,
        "Waiting for obstacle monitor: ready=%s, ready_message_age=%.3f s, risk_message_age=%.3f s (-1 means no message)",
        fields_ready_ ? "true" : "false",
        ready_received_ ==
        Clock::time_point{} ? -1.0 : std::chrono::duration<double>(tick - ready_received_).count(),
        risk_received_ ==
        Clock::time_point{} ? -1.0 : std::chrono::duration<double>(tick - risk_received_).count());
      if (std::chrono::duration<double>(tick - obstacle_stale_since_).count() > 2.0) {
        finish(Status::ABORTED, "obstacle_data_stale");
      }
      return;
    }
    obstacle_stale_since_ = Clock::time_point{};
    if (use_fields && collision_) {finish(Status::ABORTED, "collision_risk"); return;}
    status_.text =
      escape_active_ ? "escaping_inflation" : head_blocked ? "aligning_head" : avoiding ?
      "local_avoidance" :
      capture_ || final_align_ ? "approaching_goal" : "tracking_path";
    const double magnitude = std::hypot(vx, vy);
    const double translation_limit = std::min(
      {vmax, use_fields ? obstacle_speed_limit_ : vmax,
        escape_active_ || recovering ? get_parameter("start_escape_speed").as_double() : vmax});
    if (use_fields && translation_limit < 0.015 && magnitude > 0.03) {
      if (sensor_blocked_since_ == Clock::time_point{}) {sensor_blocked_since_ = tick;}
      sensor_clear_since_ = Clock::time_point{};
    } else if (sensor_blocked_since_ != Clock::time_point{} && translation_limit > 0.04) {
      if (sensor_clear_since_ == Clock::time_point{}) {sensor_clear_since_ = tick;}
      if (std::chrono::duration<double>(tick - sensor_clear_since_).count() >= 0.3) {
        sensor_blocked_since_ = sensor_clear_since_ = Clock::time_point{};
      }
    } else {sensor_clear_since_ = Clock::time_point{};}
    if (sensor_blocked_since_ != Clock::time_point{}) {
      vx = vy = 0.0;status_.text = "waiting_for_obstacle_clearance";
      if (std::chrono::duration<double>(tick - sensor_blocked_since_).count() >
        get_parameter("local_avoidance_timeout_sec").as_double())
      {
        finish(Status::ABORTED, "obstacle_speed_blocked");return;
      }
    }
    if (magnitude > translation_limit) {
      vx *= translation_limit / magnitude;vy *= translation_limit / magnitude;
    }
    trace_.requested = {vx, vy, angular};
    const auto step = predict_velocity(
      vx, vy, angular, dt, parameters, translation_limit,
      omni, head_blocked);
    velocity_ = step.first;acceleration_ = step.second;
    geometry_msgs::msg::Twist command;
    command.linear.x = velocity_[0]; command.linear.y = velocity_[1];
    command.angular.z = velocity_[2];
    if (path_.header.frame_id == map_frame_ &&
      (local_holding || !local_motion_is_free(
        x, y, yaw, command,
        std::numeric_limits<double>::infinity(), diagnostics ? &trace_.published : nullptr)))
    {
      velocity_.fill(0.0);acceleration_.fill(0.0);
      cmd_pub_->publish(geometry_msgs::msg::Twist());
      local_blocked_elapsed_ += elapsed;
      status_.text = "waiting_for_local_clearance";
      log_motion(geometry_msgs::msg::Twist());
      if (local_blocked_elapsed_ >
        get_parameter("local_avoidance_timeout_sec").as_double())
      {
        finish(Status::ABORTED, "local_trajectory_blocked");
      }
      return;
    }
    // Selection checked nominal and reachable candidates before the head veto.
    // A viable candidate waiting for its camera has a separate head deadline.
    local_blocked_elapsed_ = 0.0;
    log_motion(command);
    cmd_pub_->publish(command);
  }

  // Preview and publish use the same vector acceleration/jerk integration.
  std::pair<std::array<double, 3>, std::array<double, 3>> predict_velocity(
    double vx, double vy, double angular, double dt,
    const std::vector<rclcpp::Parameter> & parameters, double translation_limit,
    bool omni, bool head_blocked) const
  {
    auto velocity = velocity_, acceleration = acceleration_;
    const double amax = parameters[2].as_double(), angular_accel = parameters[3].as_double();
    const double wmax = parameters[1].as_double();
    const double magnitude = std::hypot(vx, vy);
    if (magnitude > translation_limit) {
      vx *= translation_limit / magnitude;vy *= translation_limit / magnitude;
    }
    const double ax = (vx - velocity[0]) / dt, ay = (vy - velocity[1]) / dt;
    const double acceleration_scale = std::min(1.0, amax / std::max(1e-9, std::hypot(ax, ay)));
    const double dax = ax * acceleration_scale - acceleration[0];
    const double day = ay * acceleration_scale - acceleration[1];
    const double jerk_scale = std::min(
      1.0, parameters[4].as_double() * dt /
      std::max(1e-9, std::hypot(dax, day)));
    acceleration[0] += dax * jerk_scale;acceleration[1] += day * jerk_scale;
    const double next_x = velocity[0] + acceleration[0] * dt;
    const double next_y = velocity[1] + acceleration[1] * dt;
    if ((vx - velocity[0]) * (vx - next_x) + (vy - velocity[1]) * (vy - next_y) <= 0.0) {
      velocity[0] = vx;velocity[1] = vy;acceleration[0] = acceleration[1] = 0.0;
    } else {velocity[0] = next_x;velocity[1] = next_y;}
    const double requested_angular_acceleration = std::clamp(
      (angular - velocity[2]) / dt, -angular_accel, angular_accel);
    const double angular_jerk = parameters[5].as_double();
    acceleration[2] += std::clamp(
      requested_angular_acceleration - acceleration[2],
      -angular_jerk * dt, angular_jerk * dt);
    const double next_angular = velocity[2] + acceleration[2] * dt;
    if ((angular - velocity[2]) * (angular - next_angular) <= 0.0) {
      velocity[2] = angular;acceleration[2] = 0.0;
    } else {velocity[2] = next_angular;}
    // Do not translate while the camera is aligning; emergency stops bypass smoothing.
    if (head_blocked) {
      velocity[0] = velocity[1] = acceleration[0] = acceleration[1] = 0.0;
    }
    if (head_blocked && std::abs(angular) < 1e-9) {velocity[2] = acceleration[2] = 0.0;}
    if (!omni) {velocity[1] = acceleration[1] = 0.0;}
    const double limited_speed = std::hypot(velocity[0], velocity[1]);
    if (limited_speed > translation_limit) {
      velocity[0] *= translation_limit / limited_speed;
      velocity[1] *= translation_limit / limited_speed;
    }
    velocity[2] = std::clamp(velocity[2], -wmax, wmax);
    return {velocity, acceleration};
  }

  // Check the actual limited command, including repulsion, over its braking
  // horizon in the already-inflated grid. A half-cell guard covers sampling
  // at grid corners without adding another robot footprint.
  bool local_motion_is_free(
    double x, double y, double yaw,
    const geometry_msgs::msg::Twist & command,
    double max_distance = std::numeric_limits<double>::infinity(),
    LocalCheck * diagnostic = nullptr) const
  {
    if (diagnostic) {
      *diagnostic = LocalCheck{};diagnostic->reason = "map_unavailable";
      diagnostic->tested = {command.linear.x, command.linear.y, command.angular.z};
    }
    if (!local_context_.grid) {
      if (diagnostic) {diagnostic->reason = local_context_.rejection;}
      return false;
    }
    const auto & grid = *local_context_.grid;
    const auto & info = grid.info;
    const double c = local_context_.c, s = local_context_.s;
    const double speed = std::hypot(command.linear.x, command.linear.y);
    bool escape = escape_active_;
    if (diagnostic) {
      diagnostic->layer = local_context_.physical ? "physical" : "planning_fallback";
    }
    const double reaction = local_context_.reaction;
    const bool omni = local_context_.omni;
    const double braking_yaw = yaw + command.angular.z * reaction;
    const double braking_x = std::cos(braking_yaw) * command.linear.x -
      std::sin(braking_yaw) * command.linear.y;
    const double braking_y = std::sin(braking_yaw) * command.linear.x +
      std::cos(braking_yaw) * command.linear.y;
    const double deceleration = local_context_.deceleration;
    const double angular_deceleration = local_context_.angular_deceleration;
    const double horizon = reaction + speed / deceleration;
    const double dt = std::min(0.03, info.resolution / (4.0 * std::max(0.01, speed)));
    double travelled = 0.0;
    double previous_x = x, previous_y = y;
    for (double t = 0.0; t <= horizon; t += dt) {
      if (diagnostic) {diagnostic->time = t;diagnostic->reason = "outside_grid";}
      const double gx = (c * (x - info.origin.position.x) + s * (y - info.origin.position.y)) /
        info.resolution;
      const double gy = (-s * (x - info.origin.position.x) + c * (y - info.origin.position.y)) /
        info.resolution;
      if (!std::isfinite(gx) || !std::isfinite(gy) || gx < 0 || gy < 0 || gx >= info.width ||
        gy >= info.height) {return false;}
      const int ix = static_cast<int>(std::floor(gx)), iy = static_cast<int>(std::floor(gy));
      const auto index = size_t(iy) * info.width + ix;
      if (diagnostic) {
        diagnostic->cell_x = ix;diagnostic->cell_y = iy;diagnostic->value = grid.data[index];
      }
      if (escape && map_->data[index] == 0) {escape = false;}
      if (grid.data[index] != 0 && !(allow_unknown_ && grid.data[index] == -1)) {
        if (diagnostic) {
          diagnostic->reason = grid.data[index] < 0 ? "unknown_cell" : "occupied_cell";
        }
        return false;
      }
      if (escape && speed >= 1e-6 && std::hypot(x - escape_x_, y - escape_y_) >
        local_context_.escape_radius)
      {
        if (diagnostic) {diagnostic->reason = "escape_radius";}
        return false;
      }
      if (!motion_constraints_.uncertainty_free(previous_x, previous_y, x, y, true,
          diagnostic ? &diagnostic->separation : nullptr, diagnostic ? &diagnostic->required : nullptr)) {
        if (diagnostic) {diagnostic->reason = "uncertain_obstacle";}
        return false;
      }
      if (!motion_constraints_.sampling_free(previous_x, previous_y, x, y, true,
          diagnostic ? &diagnostic->separation : nullptr, diagnostic ? &diagnostic->required : nullptr)) {
        if (diagnostic) {
          diagnostic->reason = "sampling_guard";
          if (!escape_active_ && !final_align_ && motion_kind_ == Command::FOLLOW_PATH) {
            diagnostic->layer = "planning_guard";
          }
        }
        return false;
      }
      previous_x = x;previous_y = y;
      if (travelled >= max_distance) {
        if (diagnostic) {diagnostic->reason = "ok";}
        return true;
      }
      const double scale = speed >
        1e-9 ? std::clamp(
        1.0 - deceleration * std::max(
          0.0,
          t + 0.5 * dt - reaction) / speed, 0.0,
        1.0) : 0.0;
      const double step = std::min(dt * scale, (max_distance - travelled) / std::max(1e-9, speed));
      // During reaction the last body-frame twist may still be held. Omni
      // braking then preserves the inertial translation direction while the
      // base rotates; differential braking retains its coupled turning arc.
      x += (omni && t >= reaction ? braking_x :
        std::cos(yaw) * command.linear.x - std::sin(yaw) * command.linear.y) * step;
      y += (omni && t >= reaction ? braking_y :
        std::sin(yaw) * command.linear.x + std::cos(yaw) * command.linear.y) * step;
      travelled += speed * step;
      const double angular_scale = std::abs(command.angular.z) > 1e-9 ?
        std::clamp(
        1.0 - angular_deceleration *
        std::max(0.0, t + 0.5 * dt - reaction) / std::abs(command.angular.z), 0.0, 1.0) : 0.0;
      yaw += command.angular.z * dt * angular_scale;
    }
    const bool complete = !std::isfinite(max_distance) || travelled + 1e-6 >= max_distance;
    if (diagnostic) {diagnostic->reason = complete ? "ok" : "insufficient_horizon";}
    return complete;
  }

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::string base_frame_, odom_frame_, map_frame_, last_phase_;
  LocalContext local_context_;
  bool allow_unknown_ = false;
  path_planner::MotionConstraints motion_constraints_;
  geometry_msgs::msg::PoseArray empty_uncertainty_;
  uint8_t motion_kind_ = Command::FOLLOW_PATH;
  path_planner::MotionObservation odometry_;
  double feasible_yaw_reference_ = 0.0, best_position_error_ = 0.0, head_wait_elapsed_ = 0.0;
  double recovery_yaw_reference_ = 0.0, measured_head_rate_ = 0.0;
  simple_move::DirectionReference translation_direction_;
  Clock::time_point observation_aligned_since_;
  double local_blocked_elapsed_ = 0.0;
  double challenger_direction_ = std::numeric_limits<double>::quiet_NaN();
  Clock::time_point challenger_since_, recovery_clear_since_;
  Clock::time_point motion_settled_since_, uncertainty_received_;
  geometry_msgs::msg::PoseArray::ConstSharedPtr uncertain_obstacles_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr uncertainty_sub_;
  MotionTrace trace_;
  Clock::time_point last_diagnostic_;
  int diagnostic_flags_ = -1;
  Path path_;
  Status status_;
  std::vector<double> lengths_;
  std::array<double, 3> velocity_{}, acceleration_{};
  double progress_ = 0.0, best_progress_ = 0.0, initial_yaw_ = 0.0, head_pan_ = 0.0,
    final_heading_ = 0.0;
  double final_profile_yaw_ = 0.0, final_profile_turn_ = 0.0, final_profile_distance_ = 0.01,
    final_progress_ = 0.0, previous_yaw_ = 0.0, measured_yaw_rate_ = 0.0;
  int escape_side_ = 0;
  bool active_ = false, relative_ = false, reverse_ = false, initialized_ = false;
  bool escape_active_ = false, escape_checked_ = false;
  double escape_end_s_ = 0.0, escape_x_ = 0.0, escape_y_ = 0.0;
  Clock::time_point obstacle_stale_since_;
  Clock::time_point intent_started_;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr clearance_map_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr clearance_sub_;
  bool final_align_ = false, head_holding_ = true, head_limit_holding_ = false,
    last_use_head_ = false;
  bool terminal_angle_holding_ = false;
  double best_terminal_error_ = std::numeric_limits<double>::infinity();
  Clock::time_point terminal_angle_since_;
  bool final_profile_ = false, capture_ = false, route_obstructed_ = false, local_detour_ = false;
  bool head_bypass_ = false, collision_ = false, fields_ready_ = false, last_omni_ = true;
  double obstacle_speed_limit_ = 0.0;
  double initial_elapsed_ = 0.0, final_elapsed_ = 0.0;
  double local_direction_ = std::numeric_limits<double>::quiet_NaN();
  double last_head_goal_ = std::numeric_limits<double>::quiet_NaN(), last_head_tilt_ = 0.0;
  Clock::time_point sensor_blocked_since_, sensor_clear_since_;
  Clock::time_point head_goal_sent_;
  Clock::time_point terminal_position_since_;
  double motion_position_tolerance_ = 0.0;
  double arrival_radius_ = 0.0;
  geometry_msgs::msg::Point arrival_center_;
  rclcpp::Subscription<path_planner::msg::NavigationPath>::SharedPtr navigation_path_sub_;
  Clock::time_point speed_limit_received_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr speed_limit_sub_;
  geometry_msgs::msg::Vector3 force_;
  Clock::time_point started_, progress_time_, last_tick_, deviation_since_;
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
    nav_stop_sub_, obstruction_sub_;
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
