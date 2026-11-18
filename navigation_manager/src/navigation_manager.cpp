#include <navigation_manager/navigation.hpp>
#include <actionlib_msgs/msg/goal_status.hpp>
#include <rclcpp/time.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <algorithm>
#include <cmath>
#include <limits>
namespace navigation_manager
{
namespace
{
Time after(Time t, double seconds)
{
  return t + std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(seconds));
}
}
NavigationManager::NavigationManager(
  Settings settings, PlannerInterface & planner,
  MotionInterface & motion)
: monitor(settings), settings_(std::move(settings)), planner_(planner), motion_(motion), arrival_(
    settings_)
{
}
void NavigationManager::start(uint64_t id, const Action::Goal & goal, Time now)
{
  if (faulted_ || (context_ && context_->state != State::Finished)) {
    throw std::logic_error("Previous navigation has not stopped");
  }
  context_ = std::make_unique<NavigationContext>();
  auto & c = *context_;
  c.goal_id = id;
  c.request = goal;
  c.started = now;
  c.progress_at = now;
  c.deadline = after(now, settings_.plan_timeout);
  c.result.result_code = Action::Result::FAILED;
  motion_.enable_sensors(true);
}
void NavigationManager::resolved(uint64_t goal, Pose pose, const std::string & error, Time now)
{
  if (!context_ || context_->goal_id != goal || context_->state != State::Resolving) {
    return;
  }
  auto & c = *context_;
  if (!error.empty()) {
    finish(Action::Result::INVALID_GOAL, error, now);
    return;
  }
  const auto & p = pose.pose.position;
  auto & q = pose.pose.orientation;
  double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  if (pose.header.frame_id != settings_.map_frame || !std::isfinite(p.x + p.y + p.z) ||
    !std::isfinite(norm) || norm < 1e-6)
  {
    finish(Action::Result::INVALID_GOAL, "invalid_goal_pose", now);
    return;
  }
  q.x /= norm;
  q.y /= norm;
  q.z /= norm;
  q.w /= norm;
  c.original = pose;
  c.observation_target = pose.pose.position;
  c.operational = std::move(pose);
  c.resolved = true;
  c.state = State::Configuring;
  c.diagnostic = "configuring_head";
  c.deadline = after(now, settings_.plan_timeout);
  motion_.configure_head(c.request.move_head, c.goal_id, ++c.operation_id);
}
void NavigationManager::head_configured(uint64_t goal, uint64_t operation, bool success, Time now)
{
  if (!context_ || context_->goal_id != goal || context_->operation_id != operation ||
    context_->state != State::Configuring)
  {
    return;
  }
  if (!success) {
    finish(Action::Result::FAILED, "head_configuration_failed", now);
    return;
  }
  auto & c = *context_;
  c.head_verified = true;
  c.state = State::Recovering;
  c.recovery = Operation::Replan;
  c.next_attempt = now;
  c.diagnostic = "ready_to_plan";
}
void NavigationManager::update_pose(
  Pose pose, const path_planner::MotionObservation & odometry,
  Time now)
{
  if (!context_) {
    return;
  }
  auto & c = *context_;
  c.pose_valid = pose.header.frame_id == settings_.map_frame;
  if (!c.pose_valid) {
    c.odometry = {};
    return;
  }
  c.robot = std::move(pose);
  c.odometry = odometry;
  // Counters reset only after physical odometric displacement, never a localization correction.
  if (odometry.count) {
    const auto & sample =
      odometry.samples[(odometry.begin + odometry.count - 1) % odometry.samples.size()];
    if (!c.progress_initialized) {
      c.progress_position.x = sample.x;
      c.progress_position.y = sample.y;
      c.progress_yaw = sample.yaw;
      c.progress_at = now;
      c.progress_initialized = true;
    } else if (std::hypot(
        sample.x - c.progress_position.x,
        sample.y - c.progress_position.y) >= settings_.progress_distance)
    {
      c.progress_position.x = sample.x;
      c.progress_position.y = sample.y;
      c.progress_yaw = sample.yaw;
      c.progress_at = now;
      recoveries.progress(c);
    } else if (std::abs(std::remainder(sample.yaw - c.progress_yaw, 2. * M_PI)) >= .15) {
      c.progress_yaw = sample.yaw;
      c.progress_at = now;
    }
  }
}
void NavigationManager::begin_stop(Time now)
{
  auto & c = *context_;
  planner_.cancel_plan();
  c.state = State::Stopping;
  c.stop_started = now;
  c.stop_acknowledged = false;
  c.motion_id.clear();
  c.motion_phase.clear();
  motion_.stop(c.goal_id, ++c.operation_id);
}
void NavigationManager::finish(uint8_t code, const std::string & reason, Time now)
{
  auto & c = *context_;
  c.result.result_code = code;
  c.result.reason = reason;
  c.diagnostic = reason;
  const char * description = code ==
    Action::Result::SUCCESS ? "Objetivo alcanzado y pose verificada" :
    code ==
    Action::Result::APPROXIMATE ? "Llegada aproximada verificada respecto al objetivo original" :
    code == Action::Result::CANCELLED ? "Navegación cancelada" :
    code == Action::Result::PREEMPTED ? "Objetivo sustituido por una nueva solicitud" :
    code ==
    Action::Result::INVALID_GOAL ? "No se pudo resolver un destino válido" :
    "La navegación no pudo completarse";
  c.result.result_message = "[" + reason + "] " + description;
  c.recovery.reset();
  if (c.state != State::Stopping) {
    begin_stop(now);
  }
}
void NavigationManager::cancel(bool preempt, Time now)
{
  if (!context_ || context_->state == State::Finished) {
    return;
  }
  if (preempt && context_->state == State::Stopping &&
    context_->result.result_code == Action::Result::CANCELLED)
  {
    return;
  }
  finish(
    preempt ? Action::Result::PREEMPTED : Action::Result::CANCELLED,
    preempt ? "preempted" : "cancelled", now);
}
void NavigationManager::stopped(uint64_t goal, uint64_t operation, bool acknowledged)
{
  if (goal == 0 && resetting_fault() && reset_operation_ == operation) {
    reset_acknowledged_ = acknowledged;
    return;
  }
  if (context_ && context_->goal_id == goal && context_->operation_id == operation &&
    context_->state == State::Stopping)
  {
    context_->stop_acknowledged = acknowledged;
  }
}
bool NavigationManager::reset_fault(Time now)
{
  if (!faulted_ || resetting_fault()) {
    return false;
  }
  reset_acknowledged_ = false;
  reset_deadline_ = after(now, settings_.stop_timeout);
  motion_.stop(0, ++reset_operation_);
  return true;
}

const char * NavigationManager::verify_fault_reset(
  const path_planner::MotionObservation & odometry, Time now)
{
  if (!resetting_fault()) {
    return nullptr;
  }
  if (now >= reset_deadline_) {
    reset_deadline_ = {};
    return reset_acknowledged_ ? "Fault reset timed out verifying odometry" :
           "Fault reset timed out waiting for controller stop confirmation";
  }
  if (reset_acknowledged_ && odometry.stationary) {
    reset_deadline_ = {};
    faulted_ = false;
    return "Navigation fault cleared";
  }
  return nullptr;
}

void NavigationManager::recover(const std::string & reason, Time now)
{
  auto & c = *context_;
  // Only actual route restrictions penalize a corridor. Controller stalls do not.
  if ((reason == "route_blocked" || reason == "collision_risk" ||
    reason == "local_trajectory_blocked" ||
    reason == "route_blocked_without_progress") && !c.start_escape && !c.path.poses.empty())
  {
    Path failed;
    failed.header = c.path.header;
    double nearest = std::numeric_limits<double>::infinity();
    size_t begin = 0;
    for (size_t i = 0; i < c.path.poses.size(); ++i) {
      const auto & p = c.path.poses[i].pose.position;
      double d = std::hypot(p.x - c.robot.pose.position.x, p.y - c.robot.pose.position.y);
      if (d < nearest) {
        nearest = d;
        begin = i;
      }
    }
    double length = 0.;
    auto previous = c.robot.pose.position;
    for (size_t i = begin; i < c.path.poses.size(); ++i) {
      const auto & p = c.path.poses[i].pose.position;
      failed.poses.push_back(c.path.poses[i]);
      length += std::hypot(p.x - previous.x, p.y - previous.y);
      previous = p;
      if (length >= .8) {
        break;
      }
    }
    if (c.failed_paths.size() == 4) {
      c.failed_paths.erase(c.failed_paths.begin());
    }
    if (failed.poses.size() > 1) {
      c.failed_paths.push_back(std::move(failed));
    }
  }
  const auto signature = monitor.geometry_signature(c);
  c.equivalent_failures =
    (signature == c.failure_signature &&
    c.previous_failure == reason) ? c.equivalent_failures + 1 : 1;
  c.failure_signature = signature;
  c.previous_failure = reason;
  auto operation = recoveries.select(c, reason, now);
  if (!operation) {
    finish(Action::Result::FAILED, reason, now);
    c.result.result_message = "[" + reason +
      "] Se agotó el presupuesto de recuperación para esta condición";
    return;
  }
  if ((reason == "route_blocked" || reason == "local_trajectory_blocked" ||
    reason == "collision_risk") && monitor.map)
  {
    c.minimum_map_stamp = monitor.map->header.stamp;
  }
  c.diagnostic = reason;
  c.recovery = operation;
  if (*operation == Operation::CaptureGoal) {
    c.operational = c.robot;
    c.operational.pose.orientation = c.original.pose.orientation;
    c.approximate = true;
    c.arrival_radius = monitor.recovery_radius(c.original);
    c.path = Path{};
    c.path.header.frame_id = settings_.map_frame;
    c.path.poses = {c.operational};
    ++c.operation_id;
    dispatch(Command::BRAKE, now);
    return;
  }
  begin_stop(now);
}
void NavigationManager::request_plan(Time now)
{
  auto & c = *context_;
  Plan::Request request;
  request.start = c.robot;
  request.goal = c.original;
  if (std::hypot(
      c.robot.pose.position.x - c.original.pose.position.x,
      c.robot.pose.position.y - c.original.pose.position.y) <= settings_.position_tolerance &&
    monitor.coherent_clearance() &&
    monitor.occupancy(*monitor.clearance, c.robot.pose.position) == 0)
  {
    c.path = Path{};
    c.path.header.frame_id = settings_.map_frame;
    c.path.poses = {c.original};
    c.operational = c.original;
    c.start_escape = c.approximate = false;
    dispatch(Command::ROTATE, now);
    return;
  }
  // A validated short correction avoids a global search while preserving the
  // original position reference after an in-place turn or a small drift.
  if (std::hypot(c.robot.pose.position.x-c.original.pose.position.x,
    c.robot.pose.position.y-c.original.pose.position.y)<=settings_.position_tolerance+.03 &&
    monitor.recovery_radius(c.original)==0.) {
    Path correction;correction.header.frame_id=settings_.map_frame;
    correction.poses={c.robot,c.original};
    if (monitor.route_free(correction,c.robot,false,std::numeric_limits<double>::infinity())) {
      c.path=std::move(correction);c.operational=c.original;c.start_escape=c.approximate=false;
      dispatch(Command::FOLLOW_PATH,now);return;
    }
  }
  // Keep an accepted alternative when it remains geometrically valid.
  Path alternative;
  alternative.header.frame_id = settings_.map_frame;
  alternative.poses = {c.operational};
  const bool preserve = c.approximate && monitor.route_free(alternative, c.operational, false, 0.);
  if (preserve) {
    request.goal = c.operational;
    request.goal_recovery_radius = 0.;
  } else {
    c.approximate = false;
    c.arrival_radius = monitor.recovery_radius(c.original);
    request.goal_recovery_radius = c.arrival_radius;
  }
  if (monitor.map && rclcpp::Time(c.minimum_map_stamp).nanoseconds() == 0) {
    c.minimum_map_stamp = monitor.map->header.stamp;
  }
  request.minimum_map_stamp = c.minimum_map_stamp;
  c.failed_paths.erase(
    std::remove_if(
      c.failed_paths.begin(), c.failed_paths.end(), [&](const Path & p) {
        return monitor.route_free(p, c.robot, false, std::numeric_limits<double>::infinity());
      }), c.failed_paths.end());
  request.avoided_paths = c.failed_paths;
  request.required_obstacles = monitor.witnesses;
  c.state = State::Planning;
  c.diagnostic = "planning";
  c.deadline = after(now, settings_.plan_timeout);
  planner_.plan(std::move(request), c.goal_id, ++c.operation_id);
}
void NavigationManager::planned(
  uint64_t goal, uint64_t operation, Plan::Response response,
  Time now)
{
  if (!context_ || context_->goal_id != goal || context_->operation_id != operation ||
    context_->state != State::Planning)
  {
    return;
  }
  auto & c = *context_;
  if (!response.reason.empty()) {
    if (response.has_observation_target) {
      c.observation_target = response.observation_target;
    }
    recover(response.reason, now);
    return;
  }
  if (rclcpp::Time(response.map_stamp) < rclcpp::Time(c.minimum_map_stamp)) {
    recover("waiting_for_updated_map", now);
    return;
  }
  auto & path = response.path;
  if (!c.pose_valid || !monitor.map_fresh(now) || path.header.frame_id != settings_.map_frame ||
    path.poses.empty() ||
    !monitor.route_free(
      path, c.robot, response.start_recovery,
      std::numeric_limits<double>::infinity()))
  {
    recover("plan_invalid_on_current_map", now);
    return;
  }
  for (const auto & pose:path.poses) {
    const auto & q = pose.pose.orientation;
    const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    if ((!pose.header.frame_id.empty() && pose.header.frame_id != settings_.map_frame) ||
      !std::isfinite(norm) || std::abs(norm - 1.) > 1e-3)
    {
      recover("invalid_plan_pose", now);
      return;
    }
  }
  const auto & endpoint = path.poses.back().pose.position;
  const auto & target = c.original.pose.position;
  double error = std::hypot(endpoint.x - target.x, endpoint.y - target.y);
  if (!response.start_recovery && error > settings_.position_tolerance &&
    (!(response.goal_relocated || c.approximate) || error >= c.arrival_radius ||
    (!c.approximate && monitor.recovery_radius(c.original) <= 0.)))
  {
    recover("planner_changed_goal", now);
    return;
  }
  if (response.start_recovery && (monitor.occupancy(*monitor.map, endpoint) != 0 ||
    std::hypot(
      endpoint.x - c.robot.pose.position.x,
      endpoint.y - c.robot.pose.position.y) > settings_.escape_radius))
  {
    recover("invalid_start_escape", now);
    return;
  }
  if (response.start_recovery) {
    auto recovery = recoveries.select(c, "start_escape", now);
    if (!recovery) {
      finish(Action::Result::FAILED, "start_recovery_exhausted", now);
      return;
    }
  }
  c.start_escape = response.start_recovery;
  c.approximate = !c.start_escape && (response.goal_relocated || c.approximate);
  if (!c.start_escape) {
    path.poses.back().pose.orientation = c.original.pose.orientation;
  }
  c.path = std::move(path);
  c.operational = c.path.poses.back();
  dispatch(c.start_escape ? Command::RECOVER_START : Command::FOLLOW_PATH, now);
}
void NavigationManager::dispatch(uint8_t mode, Time now)
{
  auto & c = *context_;
  if (!c.head_verified || (mode == Command::OBSERVE && !c.request.move_head)) {
    finish(Action::Result::FAILED, "head_configuration_failed", now);
    return;
  }
  Command command;
  command.path = c.path;
  command.mode = mode;
  if (c.approximate && mode != Command::OBSERVE) {
    command.arrival_center = c.original.pose.position;
    command.arrival_radius = c.arrival_radius;
  }
  c.blocked_since = {};
  c.route_check_at = now;
  c.route_projection = 0.;
  c.state = State::Moving;
  c.feedback_at = now;
  c.diagnostic = mode == Command::OBSERVE ? "observing_blockage" : "following_path";
  c.motion_phase.clear();
  c.motion_id = motion_.move(std::move(command));
}
void NavigationManager::motion_status(
  const std::string & id, uint8_t status,
  const std::string & reason, Time now)
{
  if (!context_ || context_->state != State::Moving || id.empty() || context_->motion_id != id) {
    return;
  }
  auto & c = *context_;
  c.feedback_at = now;
  using Status = actionlib_msgs::msg::GoalStatus;
  if (status == Status::ABORTED || status == Status::REJECTED) {
    recover(reason.empty() ? "controller_failed" : reason, now);
    return;
  }
  if (status == Status::ACTIVE) {
    if (c.motion_phase != reason) {
      c.motion_phase = reason;
    }
    return;
  }
  if (status != Status::SUCCEEDED) {
    return;
  }
  if (c.recovery == Operation::CaptureGoal) {
    c.recovery.reset();
    if (std::hypot(c.robot.pose.position.x-c.original.pose.position.x,
      c.robot.pose.position.y-c.original.pose.position.y)>=c.arrival_radius) {
      recover("near_goal_braking_drift",now);return;
    }
    c.operational=c.robot;c.operational.pose.orientation=c.original.pose.orientation;
    c.path.poses={c.operational};dispatch(Command::ROTATE,now);return;
  }
  if (c.recovery == Operation::Observe) {
    c.minimum_map_stamp = c.robot.header.stamp;
    c.recovery = Operation::WaitSensors;
    c.diagnostic = "observation_completed";
    begin_stop(now);
    return;
  }
  if (c.start_escape) {
    c.start_escape = false;
    c.operational = c.original;
    c.recovery = Operation::Replan;
    c.diagnostic = "start_recovered";
    begin_stop(now);
    return;
  }
  c.state = State::Verifying;
  c.verified_since = now;
  c.deadline = after(now, settings_.sensor_wait);
  c.diagnostic = "verifying_arrival";
}
void NavigationManager::tick(Time now)
{
  if (!context_ || context_->state == State::Finished) {
    return;
  }
  auto & c = *context_;
  if (c.state == State::Stopping) {
    bool stopped = c.stop_acknowledged && c.pose_valid && c.odometry.stationary && now >= after(
      c.stop_started, .35);
    if (!stopped && now < after(c.stop_started, settings_.stop_timeout)) {
      return;
    }
    if (!stopped) {
      faulted_ = true;
      c.result.result_code = Action::Result::FAILED;
      c.result.reason = "stop_unverified";
      c.result.result_message = "[stop_unverified] No se pudo verificar la detención";
      c.recovery.reset();
    }
    if (c.recovery) {
      c.sensors_stable_since = {};
      c.state = State::Recovering;
      c.next_attempt = after(now, settings_.replan_period);
      c.deadline = after(now, settings_.sensor_wait);
      return;
    }
    if ((c.result.result_code == Action::Result::SUCCESS ||
      c.result.result_code == Action::Result::APPROXIMATE) &&
      !arrival_.reached(c, monitor.safe_arrival(c, now)))
    {
      c.result.result_code = Action::Result::FAILED;
      c.result.reason = "arrival_not_satisfied";
      c.result.result_message =
        "[arrival_not_satisfied] La pose final no satisface las condiciones de llegada";
    }
    auto error = arrival_.errors(c);
    c.result.position_error_m = error.first;
    c.result.orientation_error_rad = error.second;
    c.state = State::Finished;
    motion_.enable_sensors(false);
    if (completed) {
      completed(c.goal_id, c.result);
    }
    return;
  }
  if (now >= after(c.started, settings_.execution_timeout)) {
    finish(Action::Result::FAILED, "execution_timeout", now);
    return;
  }
  if (c.state == State::Resolving || c.state == State::Configuring) {
    if (now >= c.deadline) {
      finish(
        Action::Result::FAILED,
        c.state == State::Resolving ? "goal_resolution_timeout" : "head_configuration_timeout",
        now);
    }
    return;
  }
  if (c.state == State::Recovering) {
    if (now < c.next_attempt) {
      return;
    }
    if (now >= c.deadline &&
      (c.recovery == Operation::WaitSensors || c.recovery == Operation::WaitLocalization))
    {
      recover(
        c.recovery == Operation::WaitSensors ? "obstacle_data_unavailable" : "robot_tf_unavailable",
        now);
      return;
    }
    if (!c.pose_valid) {
      if (c.recovery != Operation::WaitLocalization) {
        recover("robot_tf_unavailable", now);
      }
      return;
    }
    if (!monitor.map_fresh(now) || !monitor.sensors_fresh(now)) {
      c.sensors_stable_since = {};
      if (c.recovery != Operation::WaitSensors) {
        recover("obstacle_data_stale", now);
      }
      return;
    }
    if (c.recovery == Operation::WaitSensors) {
      if (c.sensors_stable_since == Time{}) {
        c.sensors_stable_since = now;
      }
      if (now < after(c.sensors_stable_since, .3)) {
        return;
      }
    }
    if (c.recovery == Operation::Observe) {
      c.path = Path{};
      c.path.header.frame_id = settings_.map_frame;
      Pose observation = c.operational;
      observation.pose.position = c.observation_target;
      c.path.poses = {c.robot, observation};
      dispatch(Command::OBSERVE, now);
      return;
    }
    c.recovery.reset();
    request_plan(now);
    return;
  }
  auto diagnostic = monitor.evaluate(c, now);
  if (!diagnostic.empty()) {
    recover(diagnostic, now);
    return;
  }
  if (c.state == State::Planning) {
    if (now >= c.deadline) {
      recover("plan_request_timeout", now);
    }
    return;
  }
  if (c.state == State::Moving) {
    // Give local avoidance its own bounded interval; fresh feedback is not progress.
    if (c.recovery != Operation::Observe && now >= after(c.route_check_at, .2)) {
      const auto elapsed = now - c.route_check_at;
      c.route_check_at = now;
      const bool waiting_head = c.motion_phase == "aligning_head" ||
        c.motion_phase == "checking_motion_direction";
      if (waiting_head && c.blocked_since != Time{}) {
        c.blocked_since += elapsed;  // Head alignment has its own controller deadline.
      }
      const double radius = monitor.recovery_radius(c.original);
      if (!c.start_escape && !c.approximate && c.recovery != Operation::Observe && radius > 0. &&
        arrival_.can_capture(c, radius) && monitor.safe_arrival(c, now)) {
        recover("goal_blocked_nearby", now);return;
      }
      if (monitor.route_free(c.path, c.robot, c.start_escape, settings_.path_horizon)) {
        c.blocked_since = {};
      } else if (c.blocked_since == Time{}) {
        c.blocked_since = now;
      } else if (now >= after(c.blocked_since, settings_.local_avoidance_timeout)) {
        recover("route_blocked", now);
      }
    }
    return;
  }
  if (c.state == State::Verifying) {
    if (arrival_.reached(c, monitor.safe_arrival(c, now))) {
      finish(
        c.approximate ? Action::Result::APPROXIMATE : Action::Result::SUCCESS,
        c.approximate ? "arrived_approximately" : "arrived", now);
    } else if (now >= c.deadline || (c.odometry.stationary && now >= after(c.verified_since, .5))) {
      recover("arrival_not_satisfied", now);
    }
  }
}
} // namespace navigation_manager
