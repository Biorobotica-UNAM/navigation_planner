// ----- ROS 2 -----
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>

// ----- ROS 2 messages -----
#include <actionlib_msgs/msg/goal_status.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/srv/get_plan.hpp>
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
  using GetPlan = nav_msgs::srv::GetPlan;
  using Clock = std::chrono::steady_clock;
  enum class State {IDLE, PLANNING, FOLLOWING, RETRY};

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
    declare_parameter<bool>("patience", true);
    const std::vector<std::pair<std::string, double>> settings = {
      {"goal_position_tolerance_m", 0.03}, {"goal_angle_tolerance_rad", 0.05},
      {"proximity_criterion", 2.0}, {"plan_request_timeout_sec", 5.0},
      {"replan_period_sec", 1.0}, {"map_timeout_sec", 3.0}, {"tf_timeout_sec", 0.5},
      {"controller_timeout_sec", 2.0}, {"path_check_horizon_m", 2.0}};
    for (const auto & setting : settings) {
      rcl_interfaces::msg::ParameterDescriptor descriptor = startup;
      descriptor.description = "Coordination setting; metres, radians or seconds as named.";
      const double value = declare_parameter<double>(setting.first, setting.second, descriptor);
      if (!std::isfinite(value) || value <= 0.0) {
        throw std::invalid_argument("Invalid " + setting.first);
      }
    }
    std::string prefix = namespaced ? get_namespace() : "";
    if (prefix == "/") {prefix.clear();}
    status_pub_ = create_publisher<Status>(prefix + "/status", rclcpp::QoS(10).transient_local());
    path_pub_ = create_publisher<Path>(prefix + "/simple_move/goal_path", 10);
    stop_pub_ = create_publisher<std_msgs::msg::Empty>(prefix + "/simple_move/stop", 10);
    enable_pub_ = create_publisher<std_msgs::msg::Bool>(
      prefix + "/potential_fields/enable", rclcpp::QoS(
        1).transient_local());
    planner_ = create_client<GetPlan>(prefix + "/path_planner/plan_path_with_augmented");
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/nav_control/goal", 10,
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr goal) {accept_goal(*goal);});
    result_sub_ = create_subscription<Status>(
      prefix + "/simple_move/goal_reached", 10,
      [this](Status::ConstSharedPtr result) {
        if (state_ != State::FOLLOWING || result->goal_id.id != motion_id_) {return;}
        controller_received_ = Clock::now();
        if (result->status == Status::SUCCEEDED) {
          // Re-evaluate the full pose; a controller result alone is not global success.
          state_ = State::RETRY; next_attempt_ = Clock::now();
        } else if (result->status == Status::ABORTED) {
          schedule_replan(result->text);
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
    near_sent_ = false;
    motion_started_ = false;
    motion_id_.clear();
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
  }

  void end_goal(uint8_t code, const std::string & reason)
  {
    ++generation_;
    if (pending_) {planner_->remove_pending_request(request_id_); pending_ = false;}
    stop_pub_->publish(std_msgs::msg::Empty());
    std_msgs::msg::Bool enable; enable.data = false; enable_pub_->publish(enable);
    state_ = State::IDLE;
    publish_status(code, reason);
  }

  void schedule_replan(const std::string & reason)
  {
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
    auto cell_free = [this, &info, c, s, start_x, start_y](int x, int y) {
        if (x < 0 || y < 0 || x >= static_cast<int>(info.width) ||
          y >= static_cast<int>(info.height)) {return false;}
        const int value = map_->data[static_cast<size_t>(y) * info.width + x];
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
    }
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
    const double distance = std::hypot(
      goal_.position.x - robot.position.x,
      goal_.position.y - robot.position.y);
    const double angular_error =
      std::remainder(tf2::getYaw(goal_.orientation) - tf2::getYaw(robot.orientation), 2.0 * M_PI);
    const bool at_position = distance <= get_parameter("goal_position_tolerance_m").as_double();
    if (at_position &&
      std::abs(angular_error) <= get_parameter("goal_angle_tolerance_rad").as_double())
    {
      end_goal(Status::SUCCEEDED, motion_started_ ? "goal_pose_reached" : "already_at_goal_pose");
      return;
    }
    if (state_ == State::FOLLOWING) {
      if (!map_ ||
        std::chrono::duration<double>(tick - map_received_).count() > get_parameter(
          "map_timeout_sec").as_double())
      {
        schedule_replan("augmented_map_stale");return;
      }
      if (!route_is_free(
          path_, robot.position.x, robot.position.y,
          get_parameter("path_check_horizon_m").as_double()))
      {
        schedule_replan("route_blocked");return;
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
      geometry_msgs::msg::PoseStamped pose;pose.pose = goal_;pose.pose.position = robot.position;
      path_.poses.push_back(pose);
      dispatch_path();return;
    }
    if (!planner_->service_is_ready()) {schedule_replan("planner_unavailable");return;}
    auto request = std::make_shared<GetPlan::Request>();
    request->start.header.frame_id = request->goal.header.frame_id = map_frame_;
    request->start.pose = robot;request->goal.pose = goal_;
    // Exact requested coordinates: relocation must not silently turn a different pose into success.
    request->tolerance = 0.0;
    const auto generation = generation_;
    state_ = State::PLANNING;pending_ = true;
    request_deadline_ = tick + std::chrono::duration_cast<Clock::duration>(
      std::chrono::duration<double>(get_parameter("plan_request_timeout_sec").as_double()));
    auto future = planner_->async_send_request(
      request, [this, generation, robot](rclcpp::Client<GetPlan>::SharedFuture response) {
        if (generation != generation_ || !pending_) {return;}
        pending_ = false;
        try {
          auto path = response.get()->plan;
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
              schedule_replan("no_valid_smooth_plan_on_current_map");
            }
            return;
          }
          if (std::hypot(
            path.poses.back().pose.position.x - goal_.position.x,
            path.poses.back().pose.position.y - goal_.position.y) >
          get_parameter("goal_position_tolerance_m").as_double())
          {
            schedule_replan("planner_changed_goal");return;
          }
          path.poses.back().pose.orientation = goal_.orientation;
          path_ = std::move(path);dispatch_path();
        } catch (const std::exception & error) {
          schedule_replan("planner_response_error");RCLCPP_WARN(get_logger(), "%s", error.what());
        }
      });
    request_id_ = future.request_id;
  }

  void dispatch_path()
  {
    // Nanosecond IDs remain unique even when several replans occur in one second.
    const int64_t stamp = std::max(now().nanoseconds(), last_motion_stamp_ + 1);
    last_motion_stamp_ = stamp;
    path_.header.stamp = rclcpp::Time(stamp, get_clock()->get_clock_type());
    for (auto & pose : path_.poses) {pose.header = path_.header;}
    motion_id_ = std::to_string(path_.header.stamp.sec) + "." + std::to_string(
      path_.header.stamp.nanosec);
    controller_received_ = Clock::now();state_ = State::FOLLOWING;
    motion_started_ = true;
    path_pub_->publish(path_);
    publish_status(
      Status::ACTIVE,
      path_.poses.size() == 1 ? "correcting_orientation" : "following_path");
  }

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::string base_frame_, map_frame_, motion_id_;
  State state_ = State::IDLE;
  geometry_msgs::msg::Pose goal_;
  Path path_;
  Grid::ConstSharedPtr map_;
  uint64_t generation_ = 0, goal_sequence_ = 0;
  int64_t request_id_ = 0, last_motion_stamp_ = 0;
  bool pending_ = false, near_sent_ = false, motion_started_ = false;
  Clock::time_point map_received_, next_attempt_, request_deadline_, controller_received_,
    last_enable_;
  rclcpp::Client<GetPlan>::SharedPtr planner_;
  rclcpp::Publisher<Status>::SharedPtr status_pub_;
  rclcpp::Publisher<Path>::SharedPtr path_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr stop_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr enable_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<Status>::SharedPtr result_sub_;
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
