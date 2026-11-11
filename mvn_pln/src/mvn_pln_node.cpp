// ----- ROS 2 -----
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>

// ----- ROS 2 messages -----
#include <actionlib_msgs/msg/goal_status.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <path_planner/srv/plan_path.hpp>
#include <path_planner/msg/navigation_path.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

// ----- C++ standard library -----
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <functional>
#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>

// Topic transport is kept here until navigation_manager exposes an action.
// Each accepted goal owns a generation; plans and controller results from older
// generations cannot advance it. There is deliberately no visited-goal history.
class MotionPlannerNode : public rclcpp::Node
{
  using Status = actionlib_msgs::msg::GoalStatus;
  using Grid = nav_msgs::msg::OccupancyGrid;
  using Path = nav_msgs::msg::Path;
  using PlanPath = path_planner::srv::PlanPath;
  using Command = path_planner::msg::NavigationPath;
  using Clock = std::chrono::steady_clock;
  enum class State {IDLE, PLANNING, FOLLOWING, RETRY, WAITING_SENSORS};

public:
  MotionPlannerNode()
  : Node("motion_planner_node"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    rcl_interfaces::msg::ParameterDescriptor startup;
    startup.read_only = true;
    startup.description = "Configured at startup.";
    const bool namespaced = declare_parameter<bool>("use_namespace", false, startup);
    base_frame_ = declare_parameter<std::string>("base_link_name", "base_link", startup);
    map_frame_ = declare_parameter<std::string>("map_frame", "map", startup);
    odom_frame_ = declare_parameter<std::string>("odom_name", "odom", startup);
    declare_parameter<bool>("patience", true);
    declare_parameter<bool>("debug_detection", false);
    max_recovery_attempts_ = declare_parameter<int>("max_recovery_attempts", 3, startup);
    if (max_recovery_attempts_ < 1 || max_recovery_attempts_ > 20) {
      throw std::invalid_argument("Invalid max_recovery_attempts");
    }
    const std::vector<std::pair<std::string, double>> settings = {
      {"goal_position_tolerance_m", 0.03}, {"goal_angle_tolerance_rad", 0.05},
      {"proximity_criterion", 2.0}, {"plan_request_timeout_sec", 5.0},
      {"replan_period_sec", 1.0}, {"map_timeout_sec", 3.0}, {"tf_timeout_sec", 0.5},
      {"controller_timeout_sec", 2.0}, {"path_check_horizon_m", 2.0},
      {"recovery_progress_m", 0.4}, {"recovery_path_horizon_m", 0.8},
      {"recovery_path_difference_m", 0.12}, {"local_avoidance_timeout_sec", 3.0},
      {"obstacle_evidence_timeout_sec", 4.0}, {"sensor_wait_timeout_sec", 10.0},
      {"start_escape_radius", 0.6}};
    for (const auto & setting : settings) {
      rcl_interfaces::msg::ParameterDescriptor descriptor = startup;
      descriptor.description = "Coordination setting; metres, radians or seconds as named.";
      const double value = declare_parameter<double>(setting.first, setting.second, descriptor);
      if (!std::isfinite(value) || value <= 0.0) {
        throw std::invalid_argument("Invalid " + setting.first);
      }
    }
    robot_radius_ = declare_parameter<double>("robot_radius", 0.23, startup);
    near_goal_extra_ = declare_parameter<double>("near_goal_extra_margin", 0.10, startup);
    near_goal_progress_ = declare_parameter<double>("near_goal_min_progress", 0.90, startup);
    if (!std::isfinite(robot_radius_) || robot_radius_ <= 0.0 || !std::isfinite(near_goal_extra_) ||
      near_goal_extra_ < 0.0 || !std::isfinite(near_goal_progress_) || near_goal_progress_ <= 0.0 ||
      near_goal_progress_ > 1.0) {throw std::invalid_argument("Invalid near-goal recovery limits");}
    std::string prefix = namespaced ? get_namespace() : "";
    if (prefix == "/") {prefix.clear();}
    status_pub_ = create_publisher<Status>(prefix + "/status", rclcpp::QoS(10).transient_local());
    path_pub_ = create_publisher<path_planner::msg::NavigationPath>(
      prefix + "/simple_move/navigation_path", 10);
    visual_path_pub_ = create_publisher<Path>(prefix + "/path", rclcpp::QoS(1).transient_local());
    obstruction_pub_ = create_publisher<std_msgs::msg::Bool>(
      prefix + "/simple_move/route_obstructed", 1);
    stop_pub_ = create_publisher<std_msgs::msg::Empty>(prefix + "/simple_move/stop", 10);
    enable_pub_ = create_publisher<std_msgs::msg::Bool>(
      prefix + "/potential_fields/enable", rclcpp::QoS(
        1).transient_local());
    planner_ = create_client<PlanPath>(prefix + "/path_planner/plan_navigation");
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/nav_control/goal", 10,
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr goal) {accept_goal(*goal);});
    result_sub_ = create_subscription<Status>(
      prefix + "/simple_move/goal_reached", 10,
      [this](Status::ConstSharedPtr result) {
        if (state_ != State::FOLLOWING || result->goal_id.id != motion_id_) {return;}
        controller_received_ = Clock::now();
        if (result->status == Status::SUCCEEDED) {
          motion_completed_ = true;
          motion_completed_since_ = Clock::now();
          // Re-evaluate the full pose; a controller result alone is not global success.
          state_ = State::RETRY; next_attempt_ = Clock::now();
        } else if (result->status == Status::ABORTED) {
          if (result->text == "obstacle_data_stale") {
            state_ = State::WAITING_SENSORS;sensor_wait_since_ = Clock::now();
            sensors_stable_since_ = Clock::time_point{};
            publish_status(Status::ACTIVE, "waiting_for_obstacle_data");
          } else {schedule_replan(result->text);}
        } else if (result->text != controller_phase_) {
          controller_phase_ = result->text;
          publish_status(Status::ACTIVE, controller_phase_);
        }
      });
    ready_sub_ = create_subscription<std_msgs::msg::Bool>(
      prefix + "/potential_fields/ready",
      rclcpp::SensorDataQoS(), [this](std_msgs::msg::Bool::ConstSharedPtr value) {
        sensors_ready_ = value->data;sensors_received_ = Clock::now();
      });
    clearance_sub_ = create_subscription<Grid>(
      prefix + "/clearance_map", rclcpp::QoS(1).transient_local(),
      [this](Grid::ConstSharedPtr grid) {clearance_map_ = grid;});
    witness_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      prefix + "/potential_fields/blocking_obstacle", 1,
      [this](geometry_msgs::msg::PointStamped::ConstSharedPtr point) {
        if (state_ == State::IDLE) {return;}
        try {
          auto transformed = tf_buffer_.transform(*point, map_frame_, tf2::durationFromSec(0.0));
          if (!std::isfinite(transformed.point.x) || !std::isfinite(transformed.point.y)) {return;}
          for (auto & previous : required_obstacles_) {
            if (std::hypot(
              previous.point.x - transformed.point.x,
              previous.point.y - transformed.point.y) < 0.10) {previous = transformed;return;}
          }
          if (required_obstacles_.size() == 16) {
            required_obstacles_.erase(required_obstacles_.begin());
          }
          required_obstacles_.push_back(transformed);
        } catch (const tf2::TransformException & error) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Cannot project blocking obstacle: %s", error.what());
        }
      });
    map_sub_ = create_subscription<Grid>(
      prefix + "/augmented_map", rclcpp::QoS(1).transient_local(),
      [this](Grid::ConstSharedPtr grid) {
        const auto & q = grid->info.origin.orientation;
        const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
        if (grid->header.frame_id != map_frame_ || !std::isfinite(grid->info.resolution) ||
        grid->info.resolution <= 0.0 || grid->info.width == 0 || grid->info.height == 0 ||
        static_cast<uint64_t>(grid->info.width) * grid->info.height != grid->data.size() ||
        !std::isfinite(grid->info.origin.position.x) ||
        !std::isfinite(grid->info.origin.position.y) ||
        !std::isfinite(norm) || std::abs(norm - 1.0) > 1e-5 || std::abs(q.x) > 1e-8 ||
        std::abs(q.y) > 1e-8) {return;}
        const auto stamp = rclcpp::Time(grid->header.stamp, get_clock()->get_clock_type());
        if (stamp.nanoseconds() != 0 &&
        std::abs((now() - stamp).seconds()) > get_parameter("map_timeout_sec").as_double())
        {
          return;
        }
        map_ = grid; map_received_ = Clock::now();
      });
    for (const auto & topic : {prefix + "/stop", prefix + "/navigation/stop"}) {
      stop_subs_.push_back(
        create_subscription<std_msgs::msg::Empty>(
          topic, 10,
          [this](std_msgs::msg::Empty::ConstSharedPtr) {
            if (state_ != State::IDLE) {
              end_goal(Status::ABORTED, "cancelled");
            }
          }));
    }
    patience_sub_ = create_subscription<std_msgs::msg::Bool>(
      prefix + "/set_patience", 10,
      [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        set_parameter(rclcpp::Parameter("patience", msg->data));
      });
    timer_ =
      create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&MotionPlannerNode::process_goal, this));
  }

private:
  void accept_goal(geometry_msgs::msg::PoseStamped goal)
  {
    if (state_ != State::IDLE) {end_goal(Status::ABORTED, "preempted_by_new_goal");}
    ++generation_;
    ++goal_sequence_;
    const auto & p = goal.pose.position;
    auto & q = goal.pose.orientation;
    const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
      !std::isfinite(norm) || norm < 1e-6) {end_goal(Status::ABORTED, "invalid_goal_pose"); return;}
    q.x /= norm; q.y /= norm; q.z /= norm; q.w /= norm;
    try {
      if (goal.header.frame_id.empty()) {goal.header.frame_id = map_frame_;}
      if (goal.header.frame_id != map_frame_) {
        const auto tf = tf_buffer_.lookupTransform(
          map_frame_, goal.header.frame_id,
          rclcpp::Time(goal.header.stamp, get_clock()->get_clock_type()));
        geometry_msgs::msg::PoseStamped transformed;
        tf2::doTransform(goal, transformed, tf); goal = transformed;
      }
    } catch (const tf2::TransformException & error) {
      end_goal(Status::ABORTED, "goal_tf_unavailable");
      RCLCPP_WARN(get_logger(), "%s", error.what()); return;
    }
    goal_ = goal.pose;
    target_goal_ = goal_;
    start_recovery_ = approximate_goal_ = motion_completed_ = near_braking_ = false;
    diagnostic_time_ = Clock::time_point{};
    near_goal_blocker_ = "not_evaluated";
    near_progress_ = near_remaining_ = -1.0;
    planning_blocked_attempts_ = 0;
    start_recovery_attempts_ = 0;
    pose_received_ = odom_received_ = settled_since_ = Clock::time_point{};
    near_sent_ = false;
    motion_started_ = false;
    motion_id_.clear();
    failed_paths_.clear();
    blocked_attempts_ = recovery_attempts_ = 0;
    required_obstacles_.clear();controller_phase_.clear();
    minimum_map_stamp_ = builtin_interfaces::msg::Time();
    route_blocked_since_ = Clock::time_point{};
    state_ = State::RETRY;
    next_attempt_ = Clock::now();
    publish_status(Status::ACTIVE, "evaluating_goal");
    std_msgs::msg::Bool enable; enable.data = true; enable_pub_->publish(enable);
  }

  void publish_status(uint8_t code, const std::string & reason)
  {
    Status status;
    status.goal_id.id = std::to_string(goal_sequence_);
    status.goal_id.stamp = now(); status.status = code; status.text = reason;
    status_pub_->publish(status);
    RCLCPP_INFO(get_logger(), "Goal %s: %s", status.goal_id.id.c_str(), reason.c_str());
    log_navigation(reason, true);
  }

  void end_goal(uint8_t code, const std::string & reason)
  {
    ++generation_;
    if (pending_) {planner_->remove_pending_request(request_id_); pending_ = false;}
    stop_pub_->publish(std_msgs::msg::Empty());
    std_msgs::msg::Bool enable; enable.data = false; enable_pub_->publish(enable);
    state_ = State::IDLE;
    Path empty;empty.header.frame_id = map_frame_;empty.header.stamp = now();
    visual_path_pub_->publish(empty);
    publish_status(code, reason);
  }

  void log_navigation(const std::string & reason, bool transition = false)
  {
    if (!get_parameter("debug_detection").as_bool()) {return;}
    const auto tick = Clock::now();
    if (!transition && std::chrono::duration<double>(tick - diagnostic_time_).count() < 1.0) {
      return;
    }
    diagnostic_time_ = tick;
    const double distance = std::hypot(
      goal_.position.x - robot_.position.x,
      goal_.position.y - robot_.position.y);
    const double yaw_error =
      std::remainder(tf2::getYaw(goal_.orientation) - tf2::getYaw(robot_.orientation), 2.0 * M_PI);
    const double map_age =
      map_ ? (now() -
      rclcpp::Time(map_->header.stamp, get_clock()->get_clock_type())).seconds() : -1.0;
    const double clearance_age =
      clearance_map_ ? (now() -
      rclcpp::Time(clearance_map_->header.stamp, get_clock()->get_clock_type())).seconds() : -1.0;
    const bool coherent = map_ && clearance_map_ && clearance_map_->info == map_->info &&
      clearance_map_->header.frame_id == map_frame_ &&
      rclcpp::Time(clearance_map_->header.stamp, get_clock()->get_clock_type()) >=
      rclcpp::Time(map_->header.stamp, get_clock()->get_clock_type());
    RCLCPP_INFO(
      get_logger(),
      "NavigationDiagnostic goal=%lu motion=%s reason=%s state=%d pose_valid=%d robot=(%.3f,%.3f,%.3f) requested=(%.3f,%.3f,%.3f) target=(%.3f,%.3f) distance=%.3f yaw_error=%.3f start_recovery=%d approximate=%d completed=%d near_gate=%s progress=%.3f remaining=%.3f radius=%.3f planning_cells=(%d,%d) physical_cell=%d map_age=%.3f clearance_age=%.3f coherent=%d ready=%d ready_age=%.3f settled_sec=%.3f retries=(%d,%d,%d) witnesses=%zu",
      static_cast<unsigned long>(goal_sequence_), motion_id_.c_str(), reason.c_str(),
      static_cast<int>(state_),
      pose_received_ != Clock::time_point{}, robot_.position.x, robot_.position.y,
      tf2::getYaw(robot_.orientation),
      goal_.position.x, goal_.position.y, tf2::getYaw(
        goal_.orientation), target_goal_.position.x, target_goal_.position.y,
      distance, yaw_error, start_recovery_, approximate_goal_, motion_completed_, near_goal_blocker_,
      near_progress_, near_remaining_, robot_radius_ + near_goal_extra_,
      map_ ? occupancy_at(*map_, robot_.position) : -1, map_ ? occupancy_at(
        *map_,
        goal_.position) : -1,
      coherent ? occupancy_at(
        *clearance_map_,
        robot_.position) : -1, map_age, clearance_age, coherent,
      sensors_ready_, sensors_received_ == Clock::time_point{} ? -1.0 : std::chrono::duration<double>(
        tick - sensors_received_).count(),
      settled_since_ ==
      Clock::time_point{} ? 0.0 : std::chrono::duration<double>(
        tick - settled_since_).count(),
      planning_blocked_attempts_, start_recovery_attempts_, recovery_attempts_,
      required_obstacles_.size());
  }

  void schedule_replan(const std::string & reason)
  {
    if ((reason == "start_blocked" || reason == "goal_blocked") &&
      ++planning_blocked_attempts_ >= max_recovery_attempts_)
    {
      end_goal(Status::ABORTED, "recovery_exhausted: " + reason);return;
    }
    const bool blocked = reason == "collision_risk" || reason == "route_blocked" ||
      reason == "route_blocked_without_progress" ||
      reason == "obstacle_speed_blocked" || reason == "motion_timeout" ||
      reason == "local_trajectory_blocked" || reason == "no_progress" || reason == "path_deviation";
    if (blocked && state_ == State::FOLLOWING) {
      const double moved = std::hypot(
        robot_.position.x - failure_position_.x,
        robot_.position.y - failure_position_.y);
      if (blocked_attempts_ == 0 || moved >= get_parameter("recovery_progress_m").as_double()) {
        blocked_attempts_ = recovery_attempts_ = 0;
        failed_paths_.clear();
        failure_position_ = robot_.position;
      }
      ++blocked_attempts_;
      // Require a map built after this blockage, not merely a recent cached grid.
      minimum_map_stamp_ = now();
      Path failed;
      failed.header.frame_id = map_frame_;
      double nearest = std::numeric_limits<double>::infinity();
      size_t begin = 0;
      for (size_t i = 0; i < path_.poses.size(); ++i) {
        const auto & p = path_.poses[i].pose.position;
        const double d = std::hypot(p.x - robot_.position.x, p.y - robot_.position.y);
        if (d < nearest) {nearest = d;begin = i;}
      }
      double traversed = 0.0;
      auto previous = robot_.position;
      for (size_t i = begin; i < path_.poses.size(); ++i) {
        failed.poses.push_back(path_.poses[i]);
        const auto & p = path_.poses[i].pose.position;
        traversed += std::hypot(p.x - previous.x, p.y - previous.y);
        previous = p;
        if (traversed >= get_parameter("recovery_path_horizon_m").as_double()) {break;}
      }
      if (failed.poses.size() > 1) {
        if (failed_paths_.size() == 4) {failed_paths_.erase(failed_paths_.begin());}
        failed_paths_.push_back(std::move(failed));
      }
      if (blocked_attempts_ >= max_recovery_attempts_ + 2) {
        end_goal(Status::ABORTED, "recovery_exhausted: " + reason);return;
      }
    }
    ++generation_;
    if (pending_) {planner_->remove_pending_request(request_id_); pending_ = false;}
    stop_pub_->publish(std_msgs::msg::Empty());
    state_ = State::RETRY;
    next_attempt_ = Clock::now() + std::chrono::duration_cast<Clock::duration>(
      std::chrono::duration<double>(get_parameter("replan_period_sec").as_double()));
    publish_status(Status::ACTIVE, "replanning: " + reason);
  }

  // The augmented grid already contains hard footprint inflation. Traverse all
  // crossed cells, including both side cells when a segment crosses a corner.
  // No second footprint radius is added here.
  bool route_is_free(const Path & path, double start_x, double start_y, double horizon)
  {
    if (!map_ || path.poses.empty()) {return false;}
    const auto & info = map_->info;
    const auto & q = info.origin.orientation;
    const double yaw = std::atan2(2.0 * q.w * q.z, 1.0 - 2.0 * q.z * q.z);
    const double c = std::cos(yaw), s = std::sin(yaw);
    const double rx =
      (c * (start_x - info.origin.position.x) + s * (start_y - info.origin.position.y)) /
      info.resolution;
    const double ry =
      (-s * (start_x - info.origin.position.x) + c * (start_y - info.origin.position.y)) /
      info.resolution;
    bool escape = clearance_map_ && clearance_map_->info == info &&
      clearance_map_->header.frame_id == map_frame_ &&
      clearance_map_->data.size() == map_->data.size() &&
      rclcpp::Time(clearance_map_->header.stamp, get_clock()->get_clock_type()) >=
      rclcpp::Time(map_->header.stamp, get_clock()->get_clock_type()) && rx >= 0 && ry >= 0 &&
      rx < info.width && ry < info.height && map_->data[size_t(ry) * info.width + size_t(rx)] > 0;
    auto cell_free = [this, &info, &escape, c, s, start_x, start_y](int x, int y) {
        if (x < 0 || y < 0 || x >= static_cast<int>(info.width) ||
          y >= static_cast<int>(info.height)) {return false;}
        const int value = map_->data[static_cast<size_t>(y) * info.width + x];
        if (value > 0 && escape && clearance_map_->data[size_t(y) * info.width + x] == 0) {
          const double gx = (x + 0.5) * info.resolution, gy = (y + 0.5) * info.resolution;
          if (std::hypot(
              info.origin.position.x + c * gx - s * gy - start_x,
              info.origin.position.y + s * gx + c * gy - start_y) <=
            get_parameter("start_escape_radius").as_double()) {return true;}
        }
        if (value != 0) {
          const double gx = (x + 0.5) * info.resolution, gy = (y + 0.5) * info.resolution;
          RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(), 2000,
            "Route intersects map: cell_world=(%.3f, %.3f), occupancy=%d, robot=(%.3f, %.3f), map_stamp=%d.%09u",
            info.origin.position.x + c * gx - s * gy, info.origin.position.y + s * gx + c * gy, value, start_x, start_y,
            map_->header.stamp.sec, map_->header.stamp.nanosec);
          return false;
        }
        return true;
      };
    double nearest = std::numeric_limits<double>::infinity();
    size_t begin = 0;
    for (size_t i = 0; i < path.poses.size(); ++i) {
      const auto & p = path.poses[i].pose.position;
      const double d = std::hypot(p.x - start_x, p.y - start_y);
      if (d < nearest) {nearest = d;begin = i;}
    }
    double px = start_x, py = start_y, traversed = 0.0;
    for (size_t i = begin; i < path.poses.size() && traversed <= horizon; ++i) {
      const auto & p = path.poses[i].pose.position;
      if (!std::isfinite(p.x) || !std::isfinite(p.y)) {return false;}
      const double length = std::hypot(p.x - px, p.y - py);
      const double fraction = length > 1e-9 ? std::min(1.0, (horizon - traversed) / length) : 1.0;
      const double tx = px + fraction * (p.x - px), ty = py + fraction * (p.y - py);
      const double ax = (c * (px - info.origin.position.x) + s * (py - info.origin.position.y)) /
        info.resolution;
      const double ay = (-s * (px - info.origin.position.x) + c * (py - info.origin.position.y)) /
        info.resolution;
      const double bx = (c * (tx - info.origin.position.x) + s * (ty - info.origin.position.y)) /
        info.resolution;
      const double by = (-s * (tx - info.origin.position.x) + c * (ty - info.origin.position.y)) /
        info.resolution;
      if (!std::isfinite(ax) || !std::isfinite(ay) || !std::isfinite(bx) || !std::isfinite(by) ||
        ax < 0 || ay < 0 || bx < 0 || by < 0 || ax >= info.width || bx >= info.width ||
        ay >= info.height || by >= info.height) {return false;}
      int x = static_cast<int>(std::floor(ax)), y = static_cast<int>(std::floor(ay));
      const int ex = static_cast<int>(std::floor(bx)), ey = static_cast<int>(std::floor(by));
      const int sx = bx > ax ? 1 : (bx < ax ? -1 : 0), sy = by > ay ? 1 : (by < ay ? -1 : 0);
      const double dx = sx ? 1.0 / std::abs(bx - ax) : std::numeric_limits<double>::infinity();
      const double dy = sy ? 1.0 / std::abs(by - ay) : std::numeric_limits<double>::infinity();
      double nx = sx ? (x + (sx > 0 ? 1.0 : 0.0) - ax) / (bx - ax) : dx;
      double ny = sy ? (y + (sy > 0 ? 1.0 : 0.0) - ay) / (by - ay) : dy;
      if (!cell_free(x, y) || !cell_free(ex, ey)) {return false;}
      while (x != ex || y != ey) {
        if (std::min(nx, ny) >= 1.0 - 1e-12) {break;}
        if (std::abs(nx - ny) < 1e-12) {
          if (!cell_free(x + sx, y) || !cell_free(x, y + sy)) {return false;}
          x += sx;y += sy;nx += dx;ny += dy;
        } else if (nx < ny) {x += sx;nx += dx;} else {y += sy;ny += dy;}
        if (!cell_free(x, y)) {return false;}
      }
      traversed += length;px = p.x;py = p.y;
      if (map_->data[size_t(ey) * info.width + ex] == 0) {escape = false;}
    }
    return true;
  }

  int occupancy_at(const Grid & grid, const geometry_msgs::msg::Point & point) const
  {
    const double angle = 2.0 * std::atan2(
      grid.info.origin.orientation.z,
      grid.info.origin.orientation.w);
    const double dx = point.x - grid.info.origin.position.x,
      dy = point.y - grid.info.origin.position.y;
    const double x = (std::cos(angle) * dx + std::sin(angle) * dy) / grid.info.resolution;
    const double y = (-std::sin(angle) * dx + std::cos(angle) * dy) / grid.info.resolution;
    if (!std::isfinite(x) || !std::isfinite(y) || x < 0 || y < 0 ||
      x >= grid.info.width || y >= grid.info.height ||
      grid.data.size() != size_t(grid.info.width) * grid.info.height) {return -1;}
    return grid.data[size_t(y) * grid.info.width + size_t(x)];
  }

  std::pair<double, double> route_progress(const geometry_msgs::msg::Point & robot)
  {
    double total = 0.0, projected = 0.0, nearest = std::numeric_limits<double>::infinity();
    for (size_t i = 1; i < path_.poses.size(); ++i) {
      const auto & a = path_.poses[i - 1].pose.position;
      const auto & b = path_.poses[i].pose.position;
      const double dx = b.x - a.x, dy = b.y - a.y, square = dx * dx + dy * dy;
      const double length = std::sqrt(square);
      if (total + length < route_projection_ - 0.2 || total > route_projection_ + 1.0) {
        total += length;continue;
      }
      const double t = square > 1e-12 ? std::clamp(
        ((robot.x - a.x) * dx + (robot.y - a.y) * dy) / square, 0.0, 1.0) : 0.0;
      const double error = std::hypot(robot.x - a.x - t * dx, robot.y - a.y - t * dy);
      if (error < nearest) {nearest = error;projected = total + t * length;}
      total += length;
    }
    const double endpoint = path_.poses.empty() ? std::numeric_limits<double>::infinity() :
      std::hypot(
      robot.x - path_.poses.back().pose.position.x,
      robot.y - path_.poses.back().pose.position.y);
    route_projection_ = std::max(route_projection_, projected);
    return {total > 1e-9 ? route_projection_ / total : 1.0, std::max(
        endpoint,
        total - route_projection_)};
  }

  bool try_near_goal_recovery(const geometry_msgs::msg::Pose & robot)
  {
    const auto tick = Clock::now();
    const auto progress = route_progress(robot.position);
    near_progress_ = progress.first;near_remaining_ = progress.second;
    if (start_recovery_ || approximate_goal_) {near_goal_blocker_ = "recovery_active";return false;}
    if (path_.poses.empty()) {near_goal_blocker_ = "no_path";return false;}
    if (!map_ || !clearance_map_) {near_goal_blocker_ = "map_missing";return false;}
    if (std::chrono::duration<double>(tick - map_received_).count() >
      get_parameter("map_timeout_sec").as_double())
    {
      near_goal_blocker_ = "map_stale";return false;
    }
    if (!sensors_ready_ || std::chrono::duration<double>(tick - sensors_received_).count() > 0.5) {
      near_goal_blocker_ = "sensors_unavailable";return false;
    }
    if (clearance_map_->info != map_->info || clearance_map_->header.frame_id != map_frame_ ||
      rclcpp::Time(clearance_map_->header.stamp, get_clock()->get_clock_type()) <
      rclcpp::Time(map_->header.stamp, get_clock()->get_clock_type()))
    {
      near_goal_blocker_ = "clearance_incoherent";return false;
    }
    if (occupancy_at(*map_, goal_.position) <= 0) {
      near_goal_blocker_ = "goal_not_blocked";return false;
    }
    if (occupancy_at(*clearance_map_, robot.position) != 0) {
      near_goal_blocker_ = "current_physical_blocked";return false;
    }
    const double radius = robot_radius_ + near_goal_extra_;
    near_goal_blocker_ = progress.first < near_goal_progress_ ? "progress" :
      progress.second >= radius ? "remaining" :
      std::hypot(
      goal_.position.x - robot.position.x,
      goal_.position.y - robot.position.y) >= radius ? "distance" : "eligible";
    if (progress.first < near_goal_progress_ || progress.second >= radius ||
      std::hypot(
        goal_.position.x - robot.position.x,
        goal_.position.y - robot.position.y) >= radius) {return false;}
    target_goal_ = robot;
    approximate_goal_ = true;
    near_braking_ = true;
    path_ = Path();path_.header.frame_id = map_frame_;
    geometry_msgs::msg::PoseStamped pose;pose.pose = target_goal_;path_.poses.push_back(pose);
    dispatch_path(Command::BRAKE);
    return true;
  }

  void process_goal()
  {
    if (state_ == State::IDLE) {return;}
    const auto tick = Clock::now();
    if (std::chrono::duration<double>(tick - last_enable_).count() >= 0.5) {
      std_msgs::msg::Bool enable;enable.data = true;enable_pub_->publish(enable);
      last_enable_ = tick;
    }
    geometry_msgs::msg::Pose robot;
    try {
      const auto tf = tf_buffer_.lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
      const auto stamp = rclcpp::Time(tf.header.stamp, get_clock()->get_clock_type());
      if (stamp.nanoseconds() != 0 &&
        std::abs((now() - stamp).seconds()) > get_parameter("tf_timeout_sec").as_double())
      {
        throw std::runtime_error("Robot transform is stale");
      }
      robot.position.x = tf.transform.translation.x;robot.position.y = tf.transform.translation.y;
      robot.orientation = tf.transform.rotation;
    } catch (const std::exception & error) {
      schedule_replan("robot_tf_unavailable");
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "%s", error.what()); return;
    }
    // Localization corrections must not be interpreted as physical motion.
    try {
      const auto tf = tf_buffer_.lookupTransform(odom_frame_, base_frame_, tf2::TimePointZero);
      const auto stamp = rclcpp::Time(tf.header.stamp, get_clock()->get_clock_type());
      if (stamp.nanoseconds() != 0 &&
        std::abs((now() - stamp).seconds()) > get_parameter("tf_timeout_sec").as_double())
      {
        throw std::runtime_error("Odometry transform is stale");
      }
      geometry_msgs::msg::Pose odom;
      odom.position.x = tf.transform.translation.x;odom.position.y = tf.transform.translation.y;
      odom.orientation = tf.transform.rotation;
      if (odom_received_ != Clock::time_point{}) {
      const double elapsed = std::max(
        1e-3, std::chrono::duration<double>(
          tick - odom_received_).count());
      const double linear = std::hypot(
        odom.position.x - odom_pose_.position.x,
        odom.position.y - odom_pose_.position.y) / elapsed;
      const double angular =
        std::abs(
        std::remainder(
          tf2::getYaw(odom.orientation) - tf2::getYaw(odom_pose_.orientation),
          2.0 * M_PI)) / elapsed;
      if (linear < 0.03 && angular < 0.08) {
        if (settled_since_ == Clock::time_point{}) {settled_since_ = tick;}
      } else {settled_since_ = Clock::time_point{};}
      }
      odom_pose_ = odom;odom_received_ = tick;
    } catch (const std::exception &) {
      odom_received_ = settled_since_ = Clock::time_point{};
    }
    robot_ = robot;pose_received_ = tick;
    log_navigation(controller_phase_.empty() ? "evaluating" : controller_phase_);
    if (state_ == State::WAITING_SENSORS) {
      if (!sensors_ready_ ||
        std::chrono::duration<double>(tick - sensors_received_).count() > 0.5)
      {
        sensors_stable_since_ = Clock::time_point{};
      } else {
        if (sensors_stable_since_ == Clock::time_point{}) {sensors_stable_since_ = tick;}
        if (std::chrono::duration<double>(tick - sensors_stable_since_).count() >= 0.3) {
          state_ = State::RETRY;next_attempt_ = tick;
          publish_status(Status::ACTIVE, "obstacle_data_recovered");
        }
      }
      if (state_ == State::WAITING_SENSORS &&
        std::chrono::duration<double>(tick - sensor_wait_since_).count() >
        get_parameter("sensor_wait_timeout_sec").as_double())
      {
        end_goal(Status::ABORTED, "obstacle_data_unavailable");
      }
      return;
    }
    const double distance = std::hypot(
      goal_.position.x - robot.position.x,
      goal_.position.y - robot.position.y);
    const double angular_error =
      std::remainder(tf2::getYaw(goal_.orientation) - tf2::getYaw(robot.orientation), 2.0 * M_PI);
    const bool at_position = distance <= get_parameter("goal_position_tolerance_m").as_double();
    if (motion_completed_ &&
      std::chrono::duration<double>(tick - motion_completed_since_).count() >
      get_parameter("sensor_wait_timeout_sec").as_double())
    {
      end_goal(Status::ABORTED, "goal_pose_not_settled");return;
    }
    if (state_ != State::FOLLOWING && start_recovery_ && motion_completed_) {
      start_recovery_ = false;motion_completed_ = false;target_goal_ = goal_;
      if (map_ && occupancy_at(*map_, robot.position) != 0) {
        schedule_replan("start_blocked");return;
      }
      minimum_map_stamp_ = map_ ? map_->header.stamp : builtin_interfaces::msg::Time();
      publish_status(Status::ACTIVE, "start_recovered");
    }
    if (state_ != State::FOLLOWING && approximate_goal_ && motion_completed_) {
      if (near_braking_) {
        if (settled_since_ == Clock::time_point{} ||
          std::chrono::duration<double>(tick - settled_since_).count() < 0.3) {return;}
        near_braking_ = false;
        if (distance >= robot_radius_ + near_goal_extra_) {
          approximate_goal_ = motion_completed_ = false;schedule_replan("near_goal_braking_drift");
          return;
        }
        target_goal_ = robot;target_goal_.orientation = goal_.orientation;
        path_ = Path();path_.header.frame_id = map_frame_;
        geometry_msgs::msg::PoseStamped pose;pose.pose = target_goal_;path_.poses.push_back(pose);
        publish_status(Status::ACTIVE, "aligning_near_blocked_goal");dispatch_path(Command::ROTATE);
        return;
      }
      const double target_distance = std::hypot(
        target_goal_.position.x - robot.position.x,
        target_goal_.position.y - robot.position.y);
      if (!map_ ||
        std::chrono::duration<double>(tick - map_received_).count() > get_parameter(
          "map_timeout_sec").as_double()) {return;}
      {
        if (get_parameter("debug_detection").as_bool()) {
          near_goal_blocker_ = target_distance >
            get_parameter("goal_position_tolerance_m").as_double() ? "alternative_position" :
            distance >= robot_radius_ + near_goal_extra_ ? "distance" :
            std::abs(angular_error) >
            get_parameter("goal_angle_tolerance_rad").as_double() ? "orientation" :
            !sensors_ready_ ||
            std::chrono::duration<double>(tick - sensors_received_).count() >
            0.5 ? "sensors_unavailable" :
            !clearance_map_ || clearance_map_->info != map_->info ||
            clearance_map_->header.frame_id != map_frame_ ||
            rclcpp::Time(clearance_map_->header.stamp, get_clock()->get_clock_type()) <
            rclcpp::Time(
            map_->header.stamp,
            get_clock()->get_clock_type()) ? "clearance_incoherent" :
            occupancy_at(*clearance_map_, robot.position) != 0 ? "current_physical_blocked" :
            settled_since_ == Clock::time_point{} || std::chrono::duration<double>(
            tick - settled_since_).count() < 0.3 ? "settling" : "eligible";
        }
        if (target_distance <= get_parameter("goal_position_tolerance_m").as_double() &&
          distance < robot_radius_ + near_goal_extra_ &&
          std::abs(angular_error) <= get_parameter("goal_angle_tolerance_rad").as_double() &&
          sensors_ready_ &&
          std::chrono::duration<double>(tick - sensors_received_).count() <= 0.5 &&
          clearance_map_ && clearance_map_->info == map_->info &&
          clearance_map_->header.frame_id == map_frame_ &&
          rclcpp::Time(clearance_map_->header.stamp, get_clock()->get_clock_type()) >=
          rclcpp::Time(map_->header.stamp, get_clock()->get_clock_type()) &&
          occupancy_at(*clearance_map_, robot.position) == 0 &&
          settled_since_ != Clock::time_point{} &&
          std::chrono::duration<double>(tick - settled_since_).count() >= 0.3)
        {
          RCLCPP_INFO(
            get_logger(), "Approximate arrival: requested=(%.3f, %.3f), reached=(%.3f, %.3f), position_error=%.3f yaw_error=%.3f",
            goal_.position.x, goal_.position.y, robot.position.x, robot.position.y, distance,
            angular_error);
          end_goal(Status::SUCCEEDED, "reached_near_goal: goal_blocked");
        } else if (std::chrono::duration<double>(tick - motion_completed_since_).count() >
          get_parameter("sensor_wait_timeout_sec").as_double())
        {
          end_goal(Status::ABORTED, "near_goal_pose_not_settled");
        }
        return;
      }
    }
    if (state_ != State::FOLLOWING && motion_completed_ && !start_recovery_) {
      const double elapsed = std::chrono::duration<double>(tick - motion_completed_since_).count();
      const bool settled = settled_since_ != Clock::time_point{} &&
      std::chrono::duration<double>(tick - settled_since_).count() >= 0.3;
      if (!settled || elapsed < 0.3) {return;}
      if (at_position &&
        std::abs(angular_error) <= get_parameter("goal_angle_tolerance_rad").as_double())
      {
        end_goal(Status::SUCCEEDED, "goal_pose_reached");return;
      }
      if (elapsed < 1.0) {return;}
      motion_completed_ = false;
      if (distance <= get_parameter("goal_position_tolerance_m").as_double() + 0.03 && map_ &&
        occupancy_at(*map_, robot.position) == 0)
      {
        path_ = Path();path_.header.frame_id = map_frame_;
        geometry_msgs::msg::PoseStamped pose;pose.pose = robot;path_.poses.push_back(pose);
        pose.pose = goal_;path_.poses.push_back(pose);
        if (route_is_free(
            path_, robot.position.x, robot.position.y,
            std::numeric_limits<double>::infinity()))
        {
          dispatch_path(at_position ? Command::ROTATE : Command::FOLLOW_PATH);return;
        }
      }
    }
    if (state_ != State::FOLLOWING && !motion_started_ && at_position &&
      std::abs(angular_error) <= get_parameter("goal_angle_tolerance_rad").as_double())
    {
      end_goal(Status::SUCCEEDED, motion_started_ ? "goal_pose_reached" : "already_at_goal_pose");
      return;
    }
    if (state_ == State::FOLLOWING) {
      if (try_near_goal_recovery(robot)) {return;}
      if (!map_ ||
        std::chrono::duration<double>(tick - map_received_).count() > get_parameter(
          "map_timeout_sec").as_double())
      {
        schedule_replan("augmented_map_stale");return;
      }
      const bool blocked = !route_is_free(
        path_, robot.position.x, robot.position.y,
        get_parameter("path_check_horizon_m").as_double());
      std_msgs::msg::Bool obstruction;obstruction.data = blocked;obstruction_pub_->publish(
        obstruction);
      if (blocked) {
        if (route_blocked_since_ == Clock::time_point{} ||
          std::hypot(
            robot.position.x - avoidance_position_.x,
            robot.position.y - avoidance_position_.y) > 0.15)
        {
          route_blocked_since_ = tick;avoidance_position_ = robot.position;
        }
        // Alignment may need longer than a geometric detour, but changing
        // phases cannot restart the stationary deadline indefinitely.
        const double allowance = controller_phase_ == "aligning_head" ||
          controller_phase_ == "waiting_for_obstacle_data" ?
          std::max(get_parameter("local_avoidance_timeout_sec").as_double(),
          get_parameter("sensor_wait_timeout_sec").as_double()) :
          get_parameter("local_avoidance_timeout_sec").as_double();
        if (std::chrono::duration<double>(tick - route_blocked_since_).count() > allowance)
        {
          schedule_replan("route_blocked_without_progress");return;
        }
      } else {
        route_blocked_since_ = Clock::time_point{};
      }
      if (std::chrono::duration<double>(tick - controller_received_).count() >
        get_parameter("controller_timeout_sec").as_double())
      {
        schedule_replan("controller_feedback_timeout");return;
      }
      if (!near_sent_ && distance < get_parameter("proximity_criterion").as_double()) {
        publish_status(Status::ACTIVE, "Near goal point");near_sent_ = true;
      }
      return;
    }
    if (pending_) {
      if (tick > request_deadline_) {schedule_replan("plan_request_timeout");}
      return;
    }
    if (tick < next_attempt_) {return;}
    if (!map_ ||
      std::chrono::duration<double>(tick - map_received_).count() >
      get_parameter("map_timeout_sec").as_double())
    {
      schedule_replan("waiting_for_augmented_map");return;
    }
    if (at_position) {
      path_ = Path();path_.header.frame_id = map_frame_;
      geometry_msgs::msg::PoseStamped pose;pose.pose = goal_;
      path_.poses.push_back(pose);
      dispatch_path();return;
    }
    if (!planner_->service_is_ready()) {schedule_replan("planner_unavailable");return;}
    if (rclcpp::Time(map_->header.stamp, get_clock()->get_clock_type()) <
      rclcpp::Time(minimum_map_stamp_, get_clock()->get_clock_type()))
    {
      schedule_replan("waiting_for_updated_map");return;
    }
    auto request = std::make_shared<PlanPath::Request>();
    request->start.header.frame_id = request->goal.header.frame_id = map_frame_;
    request->start.pose = robot;request->goal.pose = goal_;
    request->goal_recovery_radius = robot_radius_ + near_goal_extra_;
    if (minimum_map_stamp_.sec == 0 && minimum_map_stamp_.nanosec == 0) {
      minimum_map_stamp_ = map_->header.stamp;
    }
    request->minimum_map_stamp = minimum_map_stamp_;
    required_obstacles_.erase(
      std::remove_if(
        required_obstacles_.begin(), required_obstacles_.end(),
        [this](const auto & witness) {
          return (now() -
          rclcpp::Time(witness.header.stamp, get_clock()->get_clock_type())).seconds() >
          get_parameter("obstacle_evidence_timeout_sec").as_double();
        }), required_obstacles_.end());
    request->required_obstacles = required_obstacles_;
    if (blocked_attempts_ >= 2) {request->avoided_paths = failed_paths_;}
    const auto required_map_stamp = request->minimum_map_stamp;
    const auto generation = generation_;
    state_ = State::PLANNING;pending_ = true;
    request_deadline_ = tick + std::chrono::duration_cast<Clock::duration>(
      std::chrono::duration<double>(get_parameter("plan_request_timeout_sec").as_double()));
    auto future = planner_->async_send_request(
      request,
      [this, generation, robot, required_map_stamp](
        rclcpp::Client<PlanPath>::SharedFuture response) {
        if (generation != generation_ || !pending_) {return;}
        pending_ = false;
        try {
          const auto result = response.get();
          if (get_parameter("debug_detection").as_bool()) {
            RCLCPP_INFO(
              get_logger(),
              "PlanDiagnostic goal=%lu reason=%s start=(%.3f,%.3f) map_stamp=%d.%09u start_recovery=%d goal_relocated=%d poses=%zu",
              static_cast<unsigned long>(goal_sequence_),
              result->reason.empty() ? "accepted_response" : result->reason.c_str(),
              robot.position.x, robot.position.y, result->map_stamp.sec, result->map_stamp.nanosec,
              result->start_recovery, result->goal_relocated, result->path.poses.size());
          }
          if (result->reason == "waiting_for_updated_map") {
            schedule_replan(result->reason);return;
          }
          if (rclcpp::Time(result->map_stamp, get_clock()->get_clock_type()) <
          rclcpp::Time(required_map_stamp, get_clock()->get_clock_type()))
          {
            schedule_replan("waiting_for_updated_map");return;
          }
          if (!result->reason.empty()) {
            if (!get_parameter("patience").as_bool()) {
              end_goal(Status::ABORTED, result->reason);
            } else {schedule_replan(result->reason);}
            return;
          }
          if (!result->start_recovery && blocked_attempts_ >= 2 &&
          ++recovery_attempts_ > max_recovery_attempts_)
          {
            end_goal(Status::ABORTED, "recovery_exhausted: no_alternative_route");return;
          }
          auto path = result->path;
          if (!map_ ||
          std::chrono::duration<double>(Clock::now() - map_received_).count() >
          get_parameter("map_timeout_sec").as_double() ||
          path.header.frame_id != map_frame_ || path.poses.empty() ||
          !route_is_free(
            path, robot.position.x, robot.position.y,
            std::numeric_limits<double>::infinity()))
          {
            if (!get_parameter("patience").as_bool()) {
              end_goal(
                Status::ABORTED,
                "no_valid_plan");
            } else {
              schedule_replan("plan_invalid_on_current_map");
            }
            return;
          }
          if (!result->start_recovery && blocked_attempts_ >= 2) {
            for (const auto & failed : failed_paths_) {
              double separation = 0.0;
              for (const auto & pose : failed.poses) {
                const auto & p = pose.pose.position;
                double nearest = std::numeric_limits<double>::infinity();
                for (size_t i = 1; i < path.poses.size(); ++i) {
                  const auto & a = path.poses[i - 1].pose.position;
                  const auto & b = path.poses[i].pose.position;
                  const double dx = b.x - a.x, dy = b.y - a.y;
                  const double t = std::clamp(
                    ((p.x - a.x) * dx + (p.y - a.y) * dy) /
                    std::max(1e-12, dx * dx + dy * dy), 0.0, 1.0);
                  nearest = std::min(nearest, std::hypot(p.x - a.x - t * dx, p.y - a.y - t * dy));
                }
                separation = std::max(separation, nearest);
              }
              if (separation < get_parameter("recovery_path_difference_m").as_double()) {
                schedule_replan("recovery_no_alternative_route");return;
              }
            }
          }
          const double endpoint_error = std::hypot(
            path.poses.back().pose.position.x - goal_.position.x,
            path.poses.back().pose.position.y - goal_.position.y);
          if (result->start_recovery && occupancy_at(*map_, path.poses.back().pose.position) != 0) {
            schedule_replan("start_blocked");return;
          }
          if (!result->start_recovery &&
          endpoint_error > get_parameter("goal_position_tolerance_m").as_double() &&
          (!result->goal_relocated || endpoint_error >= robot_radius_ + near_goal_extra_ ||
          occupancy_at(*map_, goal_.position) <= 0))
          {
            schedule_replan("planner_changed_goal");return;
          }
          start_recovery_ = result->start_recovery;
          if (start_recovery_ && ++start_recovery_attempts_ > max_recovery_attempts_) {
            end_goal(Status::ABORTED, "recovery_exhausted: start_blocked");return;
          }
          approximate_goal_ = !start_recovery_ && result->goal_relocated;
          if (!start_recovery_) {path.poses.back().pose.orientation = goal_.orientation;}
          target_goal_ = path.poses.back().pose;
          if (!start_recovery_) {planning_blocked_attempts_ = start_recovery_attempts_ = 0;}
          path_ = std::move(path);dispatch_path();
        } catch (const std::exception & error) {
          schedule_replan("planner_response_error");RCLCPP_WARN(get_logger(), "%s", error.what());
        }
      });
    request_id_ = future.request_id;
  }

  void dispatch_path(uint8_t kind = Command::FOLLOW_PATH)
  {
    route_blocked_since_ = Clock::time_point{};
    controller_phase_.clear();
    std_msgs::msg::Bool obstruction;obstruction.data = false;obstruction_pub_->publish(obstruction);
    // Nanosecond IDs remain unique even when several replans occur in one second.
    const int64_t stamp = std::max(now().nanoseconds(), last_motion_stamp_ + 1);
    last_motion_stamp_ = stamp;
    path_.header.stamp = rclcpp::Time(stamp, get_clock()->get_clock_type());
    for (auto & pose : path_.poses) {pose.header = path_.header;}
    motion_id_ = std::to_string(path_.header.stamp.sec) + "." + std::to_string(
      path_.header.stamp.nanosec);
    controller_received_ = Clock::now();state_ = State::FOLLOWING;
    motion_started_ = true;
    motion_completed_ = false;
    route_projection_ = 0.0;
    path_planner::msg::NavigationPath command;
    command.path = path_;
    command.mode = kind == Command::FOLLOW_PATH && path_.poses.size() == 1 ? Command::ROTATE : kind;
    if (approximate_goal_) {
      command.arrival_center = goal_.position;
      command.arrival_radius = robot_radius_ + near_goal_extra_;
    }
    path_pub_->publish(command);
    visual_path_pub_->publish(path_);
    publish_status(
      Status::ACTIVE,
      command.mode == Command::BRAKE ? "braking_near_goal" : command.mode ==
      Command::ROTATE ? "correcting_orientation" : "following_path");
  }

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::string base_frame_, map_frame_, odom_frame_, motion_id_;
  State state_ = State::IDLE;
  geometry_msgs::msg::Pose goal_, target_goal_, robot_, odom_pose_;
  double robot_radius_, near_goal_extra_, near_goal_progress_;
  double route_projection_ = 0.0;
  bool start_recovery_ = false, approximate_goal_ = false, motion_completed_ = false;
  bool near_braking_ = false;
  int planning_blocked_attempts_ = 0;
  int start_recovery_attempts_ = 0;
  Clock::time_point pose_received_, odom_received_, settled_since_, motion_completed_since_;
  geometry_msgs::msg::Point failure_position_;
  geometry_msgs::msg::Point avoidance_position_;
  builtin_interfaces::msg::Time minimum_map_stamp_;
  std::vector<Path> failed_paths_;
  std::vector<geometry_msgs::msg::PointStamped> required_obstacles_;
  std::string controller_phase_;
  int blocked_attempts_ = 0, recovery_attempts_ = 0, max_recovery_attempts_ = 3;
  Path path_;
  Grid::ConstSharedPtr map_;
  Grid::ConstSharedPtr clearance_map_;
  bool sensors_ready_ = false;
  Clock::time_point sensor_wait_since_, sensors_stable_since_, sensors_received_;
  rclcpp::Subscription<Grid>::SharedPtr clearance_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ready_sub_;
  uint64_t generation_ = 0, goal_sequence_ = 0;
  int64_t request_id_ = 0, last_motion_stamp_ = 0;
  bool pending_ = false, near_sent_ = false, motion_started_ = false;
  Clock::time_point map_received_, next_attempt_, request_deadline_, controller_received_,
    last_enable_, route_blocked_since_;
  rclcpp::Client<PlanPath>::SharedPtr planner_;
  rclcpp::Publisher<Status>::SharedPtr status_pub_;
  rclcpp::Publisher<path_planner::msg::NavigationPath>::SharedPtr path_pub_;
  rclcpp::Publisher<Path>::SharedPtr visual_path_pub_;
  Clock::time_point diagnostic_time_;
  const char * near_goal_blocker_ = "not_evaluated";
  double near_progress_ = -1.0, near_remaining_ = -1.0;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr obstruction_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr stop_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr enable_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<Status>::SharedPtr result_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr witness_sub_;
  rclcpp::Subscription<Grid>::SharedPtr map_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr patience_sub_;
  std::vector<rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr> stop_subs_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotionPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
