#pragma once
// ----- navigation_manager action ----
#include <navigation_manager/action/navigate.hpp>

// ----- Path planner services and messages ----
#include <path_planner/srv/plan_path.hpp>
#include <path_planner/msg/navigation_path.hpp>
// ----- Path planner libraries
#include <path_planner/motion_observation.hpp>
#include <path_planner/motion_constraints.hpp>

// ----- ROS2 messages ----
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

// ----- C++ standard library ----
#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace navigation_manager
{
  // ----- Aliases -----
  using Action  = action::Navigate;
  using Pose    = geometry_msgs::msg::PoseStamped;
  using Path    = nav_msgs::msg::Path;
  using Grid    = nav_msgs::msg::OccupancyGrid;
  using Plan    = path_planner::srv::PlanPath;
  using Command = path_planner::msg::NavigationPath;
  using Clock   = std::chrono::steady_clock;
  using Time    = Clock::time_point;

  enum class State
  {
    Resolving, Configuring, Planning, Moving, Recovering, Verifying, Stopping, Finished
  };

  enum class Operation 
  {
    Replan, 
    Observe, 
    WaitSensors, 
    WaitLocalization, 
    CaptureGoal
  };

  struct Settings
  {
    // Frame
    std::string map_frame = "map";

    // Tolerances: (position, angle, approximate, progress)
    double position_tolerance = .03, angle_tolerance = .05, approximate_radius = .33, capture_progress = .9;

    // Timeouts: (plan, controller, map, tf, sensor, sensor_wait, replan, stop, execution, no_progress)
    double plan_timeout = 5., controller_timeout = 2., map_timeout = 3., tf_timeout = .5;
    double sensor_timeout = .5, sensor_wait = 10., replan_period = 1., stop_timeout = 4.;
    double execution_timeout = 300., progress_distance = .4, no_progress_timeout = 30.;

    // Other settings (robot_radius, escape_radius, path_horizon, local_avoidance_timeout, allow_unknown)
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
    // Goal and operation identifiers
    uint64_t goal_id = 0, operation_id = 0;
    // Goal request
    Action::Goal request;
    // Poses (original, operational, robot)
    Pose original, operational, robot;
    // Navigation path
    Path path;
    // Navigation state
    State state = State::Resolving;
    // Navigation diagnostic and motion identifiers
    std::string diagnostic = "resolving_goal", motion_id, motion_phase;

    // Navigation flags:  (resolved, pose_valid, start_escape, approximate,
    //                     stop_acknowledged, head_verified, progress_initialized)
    bool resolved = false, pose_valid = false, start_escape = false, approximate = false;
    bool stop_acknowledged = false, head_verified = false, progress_initialized = false;

    // Navigation parameters (arrival_radius, progress_yaw, route_projection)
    double arrival_radius = 0., progress_yaw = 0., route_projection = 0.;

    // Timestamps:  (verified_since, started, deadline, feedback_at, 
    //               progress_at, next_attempt, stop_started, route_check_at,
    //               blocked_since, sensors_stable_since)
    Time verified_since{}, started, deadline, feedback_at, progress_at, next_attempt, stop_started, route_check_at{},
      blocked_since{}, sensors_stable_since{};

    // Recovery and failure tracking:
    //    (failure_signature, equivalent_failures, previous_failure, progress_position, 
    //    observation_target, odometry, failed_paths, minimum_map_stamp, attempts, recovery_started,
    //    recovery_history, total_recoveries, recovery) 
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
    
    // Navigation result
    Action::Result result;
  };

  // Transport ports permit deterministic tests without a ROS executor.
  // PlannerInterface and MotionInterface are implemented by NavigationNode.
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

  // ExecutionMonitor class supervises the robot's motion and environment to determine if the navigation is proceeding safely.
  class ExecutionMonitor
  {
    public:
      explicit ExecutionMonitor(Settings settings) : settings_(std::move(settings))
      {
      }
      // ------------------------------------
      // --- Execution monitoring methods ---
      // ------------------------------------

      // Accepts a new map and returns true if it is valid and fresh.
      bool accept_map(Grid::ConstSharedPtr grid, Time received);
      // Ordered by time, the most recent map is used for occupancy checks.
      bool map_fresh(Time now) const;
      // Returns true if the sensors have been reporting data recently.
      bool sensors_fresh(Time now) const;
      // Returns true if the clearance map is coherent with the occupancy map.
      bool coherent_clearance() const;
      // Returns the occupancy value at a given point in the grid.
      int occupancy(const Grid & grid, const geometry_msgs::msg::Point & point) const;
      // Returns true if the path is free of obstacles, considering the robot pose.
      bool route_free(const Path & path, const Pose & robot, bool escape, double horizon);
      // Returns true if the robot can safely reach the goal.
      bool safe_arrival(const NavigationContext & context, Time now);
      // Returns a string describing the current state of the navigation, including any detected issues.
      std::string evaluate(const NavigationContext & context, Time now) const;
      // Returns a unique signature for the geometry of the current navigation context, which can be used to detect repeated failures. 
      uint64_t geometry_signature(const NavigationContext & context) const;
      // Returns the recovery radius for the given goal pose, which is used to determine if the robot can safely recover from a failure.
      double recovery_radius(const Pose & goal);

      // ---------------------------------
      // --- Execution monitoring data ---
      // ---------------------------------
      
      // The most recent occupancy grid and clearance grid received from the map server.
      Grid::ConstSharedPtr map, clearance;
      // The most recent uncertain obstacles received from the sensors.
      geometry_msgs::msg::PoseArray uncertain;
      // The most recent time the sensors reported data.
      std::vector<geometry_msgs::msg::PointStamped> witnesses;
      bool sensors_ready = false;
      // The most recent time the sensors reported data.
      Time sensors_received{}, map_received{};
    
    private:
      Settings settings_;
      path_planner::MotionConstraints constraints_;
  };

  // ArrivalEvaluator class determines if the robot has reached its goal and calculates the position and orientation errors.
  class ArrivalEvaluator
  {
    public:
      explicit ArrivalEvaluator(Settings settings) : settings_(std::move(settings))
      {
      }
      // ----------------------------------
      // --- Arrival evaluation methods ---
      // ----------------------------------

      // Returns the position and orientation errors between the robot's current pose and the goal pose.
      std::pair<double, double> errors(const NavigationContext & context) const;
      // Returns true if the robot has reached its goal, considering the position and orientation errors and the arrival radius.
      bool reached(const NavigationContext & context, bool safe) const;
      // Returns true if the robot can safely capture the goal, considering the position and orientation errors and the arrival radius.
      bool can_capture(NavigationContext & context, double radius) const;

    private:
      Settings settings_;
  };

  // RecoveryStrategy class defines a strategy for recovering from navigation failures.
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
  
  // RecoveryManager class manages the recovery strategies and budgets for navigation failures.
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

  // NavigationManager class coordinates the navigation process, including planning, motion, recovery, etc.
  class NavigationManager
  {
    public:
      NavigationManager(Settings settings, PlannerInterface & planner, MotionInterface & motion);
      // -------------------------------------
      // --- Navigation management methods ---
      // -------------------------------------

      // Starts a new navigation goal with the given ID and request, and records the start time.
      void start(uint64_t id, const Action::Goal & goal, Time now);
      // Updates the navigation context with the given pose, odometry, and current time.
      void resolved(uint64_t goal, Pose pose, const std::string & error, Time now);
      // Updates the navigation context with the given head configuration status and current time.
      void head_configured(uint64_t goal, uint64_t operation, bool success, Time now);
      // Updates the navigation context with the given plan response and current time.
      void planned(uint64_t goal, uint64_t operation, Plan::Response response, Time now);
      // Updates the navigation context with the given motion status and current time.
      void motion_status(const std::string & id, uint8_t status, const std::string & reason, Time now);
      // Updates the navigation context with the given stop acknowledgment and current time.
      void stopped(uint64_t goal, uint64_t operation, bool acknowledged);
      // Updates the navigation context with the given sensor status and current time.
      void update_pose(Pose pose, const path_planner::MotionObservation & odometry, Time now);
      void tick(Time now);
      bool faulted() const {return faulted_;}
      bool resetting_fault() const {return reset_deadline_ != Time{};}
      bool reset_fault(Time now);
      // Returns a diagnostic when the asynchronous verification completes.
      const char * verify_fault_reset(const path_planner::MotionObservation & odometry, Time now);
      // Cancels the current navigation goal, optionally preempting it, and records the current time
      void cancel(bool preempt, Time now);

      // Returns a pointer to the current navigation context
      const NavigationContext * context() const
      {
        return context_.get();
      }

      // Other public members
      ExecutionMonitor monitor;
      RecoveryManager recoveries;
      std::function<void(uint64_t, const Action::Result &)> completed;

    private:

      // Internal methods for managing navigation state and operations
      void recover(const std::string & reason, Time now);
      void begin_stop(Time now);
      void dispatch(uint8_t mode, Time now);
      void finish(uint8_t code, const std::string & reason, Time now);
      void request_plan(Time now);
      // Internal data members for managing navigation state and operations
      Settings settings_;
      PlannerInterface & planner_;
      MotionInterface & motion_;
      ArrivalEvaluator arrival_;
      std::unique_ptr<NavigationContext> context_;
      bool faulted_ = false, reset_acknowledged_ = false;
      Time reset_deadline_{};
      uint64_t reset_operation_ = 0;
  };
} // namespace navigation_manager