#include <navigation_manager/navigation.hpp>
#include <algorithm>
#include <limits>
namespace navigation_manager
{
namespace
{
class SensorRecovery final : public RecoveryStrategy
{
public:
  SensorRecovery()
  {
    name = "sensors";
    budget = {4, 30.};
  }
  std::optional<Operation> select(
    const std::string & reason,
    const NavigationContext &) const override
  {
    if (reason == "obstacle_data_stale" || reason == "obstacle_data_unavailable" ||
      reason == "map_unavailable" ||
      reason == "waiting_for_updated_map" || reason == "waiting_for_clearance_map" ||
      reason == "waiting_for_obstacle_map")
    {
      return Operation::WaitSensors;
    }
    return {};
  }
};
class LocalizationRecovery final : public RecoveryStrategy
{
public:
  LocalizationRecovery()
  {
    name = "localization";
    budget = {3, 10.};
  }
  std::optional<Operation> select(
    const std::string & reason,
    const NavigationContext &) const override
  {
    if (reason == "robot_tf_unavailable" || reason == "odometry_unavailable") {
      return Operation::WaitLocalization;
    }
    return {};
  }
};
class ApproximateArrivalRecovery final : public RecoveryStrategy
{
public:
  ApproximateArrivalRecovery() {name = "arrival"; budget = {1, 10.};}
  std::optional<Operation> select(const std::string & reason, const NavigationContext &) const override
  {
    if (reason == "goal_blocked_nearby") {return Operation::CaptureGoal;}
    return {};
  }
};
class ObservationRecovery final : public RecoveryStrategy
{
public:
  ObservationRecovery()
  {
    name = "observation";
    budget = {3, 25.};
  }
  std::optional<Operation> select(
    const std::string & reason,
    const NavigationContext & c) const override
  {
    if (!c.request.move_head || reason == "observation_disabled" ||
      reason == "observation_outside_pan_range")
    {
      return {};
    }
    if ((reason == "start_blocked" || reason == "goal_blocked" || reason == "no_path" ||
      reason == "no_smooth_path") &&
      c.equivalent_failures >= 3)
    {
      return Operation::Observe;
    }
    return {};
  }
};
class ReplanRecovery final : public RecoveryStrategy
{
public:
  ReplanRecovery()
  {
    name = "replan";
    budget = {6, 40.};
  }
  std::optional<Operation> select(const std::string &, const NavigationContext &) const override
  {
    return Operation::Replan;
  }
};
}
RecoveryManager::RecoveryManager()
{
  strategies.emplace_back(std::make_unique<SensorRecovery>());
  strategies.emplace_back(std::make_unique<LocalizationRecovery>());
  strategies.emplace_back(std::make_unique<ApproximateArrivalRecovery>());
  strategies.emplace_back(std::make_unique<ObservationRecovery>());
  strategies.emplace_back(std::make_unique<ReplanRecovery>());
}
std::optional<Operation> RecoveryManager::select(
  NavigationContext & c, const std::string & reason,
  Time now)
{
  if (auto budget = condition_budgets.find(reason);budget != condition_budgets.end()) {
    const std::string key = "condition:" + reason;
    auto [start, inserted] = c.recovery_started.emplace(key, now);
    (void)inserted;
    if (c.attempts[key] >= budget->second.attempts ||
      std::chrono::duration<double>(now - start->second).count() >= budget->second.seconds)
    {
      return {};
    }
    ++c.attempts[key];
  }
  for (const auto & strategy : strategies) {
    auto operation = strategy->select(reason, c);
    if (!operation) {
      continue;
    }
    const auto key = strategy->name;
    auto [started, inserted] = c.recovery_started.emplace(key, now);
    (void)inserted;
    if (c.attempts[key] >= strategy->budget.attempts ||
      std::chrono::duration<double>(now - started->second).count() >= strategy->budget.seconds)
    {
      // Waiting for unavailable perception or localization cannot safely fall
      // through to a movement-based strategy after its budget is exhausted.
      if (*operation == Operation::WaitSensors || *operation == Operation::WaitLocalization) {
        return {};
      }
      continue;
    }
    ++c.attempts[key];
    if (c.total_recoveries < std::numeric_limits<uint16_t>::max()) {
      ++c.total_recoveries;
    }
    if (c.recovery_history.size() == 64) {
      c.recovery_history.erase(c.recovery_history.begin());
    }
    c.recovery_history.push_back(key + ":" + reason);
    return operation;
  }
  return {};
}
void RecoveryManager::progress(NavigationContext & c)
{
  c.attempts.clear();
  c.recovery_started.clear();
  c.failed_paths.clear();
  c.equivalent_failures = 0;
}
} // namespace navigation_manager
