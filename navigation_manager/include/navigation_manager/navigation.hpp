#pragma once
#include <navigation_manager/action/navigate.hpp>
#include <path_planner/srv/plan_path.hpp>
#include <path_planner/msg/navigation_path.hpp>
#include <path_planner/motion_observation.hpp>
#include <path_planner/motion_constraints.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace navigation_manager
{
using Action = action::Navigate;
using Pose = geometry_msgs::msg::PoseStamped;
using Path = nav_msgs::msg::Path;
using Grid = nav_msgs::msg::OccupancyGrid;
using Plan = path_planner::srv::PlanPath;
using Command = path_planner::msg::NavigationPath;
using Clock = std::chrono::steady_clock;
using Time = Clock::time_point;
enum class State {
  Resolving = Action::Feedback::RESOLVING,
  Configuring = Action::Feedback::CONFIGURING,
  Planning = Action::Feedback::PLANNING,
  Moving = Action::Feedback::MOVING,
  Recovering = Action::Feedback::RECOVERING,
  Verifying = Action::Feedback::VERIFYING,
  Stopping = Action::Feedback::STOPPING,
  Finished
};
enum class Operation {Replan, Observe, WaitSensors, WaitLocalization, CaptureGoal};
struct Settings
{
  std::string map_frame = "map";
  double position_tolerance = .03, angle_tolerance = .05, approximate_radius = .33, capture_progress = .9;
  double plan_timeout = 5., controller_timeout = 2., map_timeout = 3., tf_timeout = .5;
  double sensor_timeout = .5, sensor_wait = 10., replan_period = 1., stop_timeout = 4.;
  double execution_timeout = 300., progress_distance = .4, no_progress_timeout = 30.;
  double robot_radius = .23, escape_radius = .6, path_horizon = 2., local_avoidance_timeout = 3.;
  bool allow_unknown = false;
};
struct RecoveryBudget
{
  unsigned attempts = 0;
  double seconds = 0.;
};
// Goal-owned data. No ROS callbacks or decisions live in this object.
struct NavigationContext
{
  uint64_t goal_id = 0, operation_id = 0;
  Action::Goal request;
  Pose original, operational, robot;
  Path path;
  State state = State::Resolving;
  std::string diagnostic = "resolving_goal", motion_id;
  bool resolved = false, pose_valid = false, start_escape = false, approximate = false;
  bool stop_acknowledged = false, head_verified = false, progress_initialized = false;
  double arrival_radius = 0., progress_yaw = 0., route_projection = 0.;
  Time verified_since{}, started, deadline, feedback_at, progress_at, next_attempt, stop_started, route_check_at{},
    blocked_since{}, sensors_stable_since{};
  uint64_t failure_signature = 0;
  unsigned equivalent_failures = 0;
  std::string previous_failure;
  geometry_msgs::msg::Point progress_position, observation_target;
  path_planner::MotionObservation odometry;
  std::vector<Path> failed_paths;
  builtin_interfaces::msg::Time minimum_map_stamp;
  std::map<std::string, unsigned> attempts;
  std::map<std::string, Time> recovery_started;
  std::vector<std::string> recovery_history;
  uint16_t total_recoveries = 0;
  std::optional<Operation> recovery;
  Action::Result result;
};
// Transport ports permit deterministic tests without a ROS executor.
class PlannerInterface
{
public:
  virtual ~PlannerInterface() = default;
  virtual void plan(Plan::Request request, uint64_t goal, uint64_t operation) = 0;
  virtual void cancel_plan() = 0;
};
class MotionInterface
{
public:
  virtual ~MotionInterface() = default;
  virtual void configure_head(bool enabled, uint64_t goal, uint64_t operation) = 0;
  virtual std::string move(Command command) = 0;
  virtual void stop(uint64_t goal, uint64_t operation) = 0;
  virtual void enable_sensors(bool enabled) = 0;
};
class ExecutionMonitor
{
public:
  explicit ExecutionMonitor(Settings settings)
  : settings_(std::move(settings))
  {
  }
  bool accept_map(Grid::ConstSharedPtr grid, Time received);
  bool map_fresh(Time now) const;
  bool sensors_fresh(Time now) const;
  bool coherent_clearance() const;
  int occupancy(const Grid & grid, const geometry_msgs::msg::Point & point) const;
  bool route_free(const Path & path, const Pose & robot, bool escape, double horizon);
  bool safe_arrival(const NavigationContext & context, Time now);
  std::string evaluate(const NavigationContext & context, Time now) const;
  uint64_t geometry_signature(const NavigationContext & context) const;
  double recovery_radius(const Pose & goal);
  Grid::ConstSharedPtr map, clearance;
  geometry_msgs::msg::PoseArray uncertain;
  std::vector<geometry_msgs::msg::PointStamped> witnesses;
  bool sensors_ready = false;
  Time sensors_received{}, map_received{};

private:
  Settings settings_;
  path_planner::MotionConstraints constraints_;
};
class ArrivalEvaluator
{
public:
  explicit ArrivalEvaluator(Settings settings)
  : settings_(std::move(settings))
  {
  }
  std::pair<double, double> errors(const NavigationContext & context) const;
  bool reached(const NavigationContext & context, bool safe) const;
  bool can_capture(NavigationContext & context, double radius) const;

private:
  Settings settings_;
};
class RecoveryStrategy
{
public:
  virtual ~RecoveryStrategy() = default;
  virtual std::optional<Operation> select(
    const std::string & reason,
    const NavigationContext & context) const = 0;
  std::string name;
  RecoveryBudget budget;
};
class RecoveryManager
{
public:
  RecoveryManager();
  std::optional<Operation> select(NavigationContext & context, const std::string & reason,
    Time now);
  void progress(NavigationContext & context);
  std::vector<std::unique_ptr<RecoveryStrategy>> strategies;
  std::map<std::string, RecoveryBudget> condition_budgets;
};
class NavigationManager
{
public:
  NavigationManager(Settings settings, PlannerInterface & planner, MotionInterface & motion);
  void start(uint64_t id, const Action::Goal & goal, Time now);
  void resolved(uint64_t goal, Pose pose, const std::string & error, Time now);
  void head_configured(uint64_t goal, uint64_t operation, bool success, Time now);
  void planned(uint64_t goal, uint64_t operation, Plan::Response response, Time now);
  void motion_status(const std::string & id, uint8_t status, const std::string & reason, Time now);
  void stopped(uint64_t goal, uint64_t operation, bool acknowledged);
  void update_pose(Pose pose, const path_planner::MotionObservation & odometry, Time now);
  void tick(Time now);
  void cancel(bool preempt, Time now);
  const NavigationContext * context() const
  {
    return context_.get();
  }
  ExecutionMonitor monitor;
  RecoveryManager recoveries;
  std::function<void(uint64_t, const Action::Result &)> completed;

private:
  void recover(const std::string & reason, Time now);
  void begin_stop(Time now);
  void dispatch(uint8_t mode, Time now);
  void finish(uint8_t code, const std::string & reason, Time now);
  void request_plan(Time now);
  Settings settings_;
  PlannerInterface & planner_;
  MotionInterface & motion_;
  ArrivalEvaluator arrival_;
  std::unique_ptr<NavigationContext> context_;
};
} // namespace navigation_manager
