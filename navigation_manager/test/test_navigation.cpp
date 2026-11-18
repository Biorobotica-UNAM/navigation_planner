#include <gtest/gtest.h>
#include <navigation_manager/navigation.hpp>
#include <actionlib_msgs/msg/goal_status.hpp>
using namespace navigation_manager;
struct Ports : PlannerInterface, MotionInterface
{
  uint64_t goal = 0, op = 0;
  int moves = 0, stops = 0, plans = 0;
  bool head = false;
  Plan::Request request;
  Command command;
  void plan(Plan::Request r, uint64_t g, uint64_t o) override
  {
    request = std::move(r);
    goal = g;
    op = o;
    ++plans;
  }
  void cancel_plan() override
  {
  }
  void configure_head(bool h, uint64_t g, uint64_t o) override
  {
    head = h;
    goal = g;
    op = o;
  }
  std::string move(Command c) override
  {
    command = std::move(c);
    return std::to_string(++moves);
  }
  void stop(uint64_t g, uint64_t o) override
  {
    goal = g;
    op = o;
    ++stops;
  }
  void enable_sensors(bool) override
  {
  }
};
struct NavigationTest : testing::Test
{
  Settings settings;
  Ports ports;
  NavigationManager manager{settings, ports, ports};
  Time now = Clock::now();
  Pose robot, target;
  path_planner::MotionObservation odometry;
  void SetUp() override
  {
    robot.header.frame_id = target.header.frame_id = "map";
    robot.pose.orientation.w = target.pose.orientation.w = 1.;
    robot.pose.position.x = robot.pose.position.y = 2.;
    target = robot;
    target.pose.position.x = 3.;
    auto map = std::make_shared<Grid>();
    map->header.frame_id = "map";
    map->info.resolution = .1;
    map->info.width = map->info.height = 60;
    map->info.origin.orientation.w = 1.;
    map->data.assign(3600, 0);
    manager.monitor.accept_map(map, now);
    manager.monitor.clearance = map;
    manager.monitor.sensors_ready = true;
    manager.monitor.sensors_received = now;
    odometry.stationary = true;
  }
  void start()
  {
    Action::Goal goal;
    goal.pose = target;
    goal.move_head = false;
    manager.start(1, goal, now);
    manager.resolved(1, target, "", now);
    manager.head_configured(1, ports.op, true, now);
    manager.update_pose(robot, odometry, now);
    manager.tick(now);
  }
  Plan::Response response()
  {
    Plan::Response r;
    r.path.header.frame_id = "map";
    r.path.poses = {robot, target};
    return r;
  }
  void stop_ack()
  {
    manager.stopped(ports.goal, ports.op, true);
    now += std::chrono::milliseconds(400);
    manager.monitor.sensors_received = now;
    manager.update_pose(robot, odometry, now);
    manager.tick(now);
  }
};
TEST_F(NavigationTest, HeadVerifiedBeforePlanningAndMotion) {
  Action::Goal goal;
  goal.pose = target;
  manager.start(1, goal, now);
  manager.resolved(1, target, "", now);
  manager.update_pose(robot, odometry, now);
  manager.tick(now);
  EXPECT_EQ(ports.plans, 0);
  manager.head_configured(1, ports.op, true, now);
  manager.tick(now);
  EXPECT_EQ(ports.plans, 1);
  manager.planned(1, ports.op, response(), now);
  EXPECT_EQ(ports.moves, 1);
  EXPECT_FALSE(ports.head);
}
TEST_F(NavigationTest, OldOperationsCannotMutateActiveGoal) {
  start();
  auto old = ports.op;
  manager.cancel(false, now);
  manager.planned(1, old, response(), now);
  EXPECT_EQ(ports.moves, 0);
  stop_ack();
  EXPECT_EQ(manager.context()->result.result_code, Action::Result::CANCELLED);
  Action::Goal goal;
  goal.pose = target;
  manager.start(2, goal, now);
  manager.resolved(1, target, "", now);
  EXPECT_EQ(manager.context()->state, State::Resolving);
}
TEST_F(NavigationTest, ControllerSuccessRequiresActualPose) {
  start();
  manager.planned(1, ports.op, response(), now);
  manager.motion_status("1", actionlib_msgs::msg::GoalStatus::SUCCEEDED, "done", now);
  manager.tick(now);
  EXPECT_EQ(manager.context()->state, State::Verifying);
  robot = target;
  manager.update_pose(robot, odometry, now);
  manager.tick(now);
  EXPECT_EQ(manager.context()->state, State::Stopping);
  stop_ack();
  EXPECT_EQ(manager.context()->result.result_code, Action::Result::SUCCESS);
}
TEST_F(NavigationTest, ApproximateArrivalPreservesOriginalError) {
  auto map = std::make_shared<Grid>(*manager.monitor.map);
  const int x = int(target.pose.position.x / map->info.resolution),
    y = int(target.pose.position.y / map->info.resolution);
  map->data[y * 60 + x] = 100;
  manager.monitor.accept_map(map, now);
  manager.monitor.clearance = map;
  start();
  auto r = response();
  r.goal_relocated = true;
  r.path.poses.back().pose.position.x -= .2;
  manager.planned(1, ports.op, std::move(r), now);
  robot.pose.position.x = 2.8;
  manager.update_pose(robot, odometry, now);
  manager.motion_status("1", 3, "done", now);
  manager.tick(now);
  stop_ack();
  EXPECT_EQ(manager.context()->result.result_code, Action::Result::APPROXIMATE);
  EXPECT_NEAR(manager.context()->result.position_error_m, .2, 1e-5);
  EXPECT_DOUBLE_EQ(manager.context()->original.pose.position.x, 3.);
}
TEST_F(NavigationTest, CancelRequiresControllerAndPhysicalStop) {
  start();
  manager.cancel(false, now);
  now += std::chrono::milliseconds(400);
  manager.tick(now);
  EXPECT_EQ(manager.context()->state, State::Stopping);
  manager.stopped(ports.goal, ports.op, true);
  manager.tick(now);
  EXPECT_EQ(manager.context()->state, State::Finished);
}
TEST_F(NavigationTest, StopTimeoutCannotReportCancellationAsVerified) {
  start();
  manager.cancel(false, now);
  now += std::chrono::seconds(5);
  manager.tick(now);
  EXPECT_EQ(manager.context()->result.reason, "stop_unverified");
}
TEST_F(NavigationTest, ResetFaultRequiresCurrentControllerAckAndStationaryOdometry) {
  start();
  manager.cancel(false, now);
  now += std::chrono::seconds(5);
  manager.tick(now);
  ASSERT_TRUE(manager.faulted());
  EXPECT_THROW(manager.start(2, Action::Goal{}, now), std::logic_error);

  ASSERT_TRUE(manager.reset_fault(now));
  EXPECT_FALSE(manager.reset_fault(now));
  const auto expired_operation = ports.op;
  EXPECT_EQ(manager.verify_fault_reset(odometry, now), nullptr);
  now += std::chrono::seconds(5);
  EXPECT_NE(manager.verify_fault_reset(odometry, now), nullptr);
  ASSERT_TRUE(manager.faulted());

  ASSERT_TRUE(manager.reset_fault(now));
  manager.stopped(0, expired_operation, true);
  EXPECT_EQ(manager.verify_fault_reset(odometry, now), nullptr);
  manager.stopped(0, ports.op, true);
  EXPECT_EQ(manager.verify_fault_reset(path_planner::MotionObservation{}, now), nullptr);
  EXPECT_TRUE(manager.faulted());
  EXPECT_NE(manager.verify_fault_reset(odometry, now), nullptr);
  EXPECT_FALSE(manager.faulted());
  EXPECT_EQ(manager.context()->state, State::Finished);
  EXPECT_EQ(ports.moves, 0);
}

TEST_F(NavigationTest, PlannerTimeoutInvalidatesLateResponse) {
  start();
  auto old = ports.op;
  now += std::chrono::seconds(6);
  manager.monitor.accept_map(manager.monitor.map, now);
  manager.monitor.sensors_received = now;
  manager.tick(now);
  manager.planned(1, old, response(), now);
  EXPECT_EQ(ports.moves, 0);
  EXPECT_EQ(manager.context()->state, State::Stopping);
}
TEST_F(NavigationTest, InvalidMapAndUnsafeRouteRejected) {
  auto map = std::make_shared<Grid>(*manager.monitor.map);
  map->info.resolution = 0;
  EXPECT_FALSE(manager.monitor.accept_map(map, now));
  map = std::make_shared<Grid>(*manager.monitor.map);
  map->data[20 * 60 + 25] = 100;
  ASSERT_TRUE(manager.monitor.accept_map(map, now));
  start();
  manager.planned(1, ports.op, response(), now);
  EXPECT_EQ(ports.moves, 0);
}
TEST(Recovery, BudgetsRespectHeadConstraintAndProgress) {
  RecoveryManager manager;
  NavigationContext c;
  c.request.move_head = false;
  auto now = Clock::now();
  for (int i = 0; i < 6; ++i) {
    auto op = manager.select(c, "goal_blocked", now);
    ASSERT_TRUE(op);
    EXPECT_NE(*op, Operation::Observe);
  }
  EXPECT_FALSE(manager.select(c, "goal_blocked", now));
  manager.progress(c);
  EXPECT_TRUE(manager.select(c, "goal_blocked", now));
}
TEST(Recovery, ConditionBudgetAndTimeLimit) {
  RecoveryManager manager;
  manager.condition_budgets["no_progress"] = {2, 3.};
  NavigationContext c;
  auto now = Clock::now();
  EXPECT_TRUE(manager.select(c, "no_progress", now));
  EXPECT_FALSE(manager.select(c, "no_progress", now + std::chrono::seconds(4)));
}
TEST_F(NavigationTest, FreeGoalCannotBeSilentlyRelocated) {
  start();
  auto r = response();
  r.goal_relocated = true;
  r.path.poses.back().pose.position.x -= .2;
  manager.planned(1, ports.op, std::move(r), now);
  EXPECT_EQ(ports.moves, 0);
  EXPECT_EQ(manager.context()->diagnostic, "planner_changed_goal");
}
TEST_F(NavigationTest, FailedHeadConfigurationNeverMoves) {
  Action::Goal goal;
  goal.pose = target;
  goal.move_head = true;
  manager.start(1, goal, now);
  manager.resolved(1, target, "", now);
  manager.head_configured(1, ports.op, false, now);
  EXPECT_EQ(ports.plans, 0);
  EXPECT_EQ(ports.moves, 0);
  EXPECT_EQ(manager.context()->result.reason, "head_configuration_failed");
}
TEST_F(NavigationTest, StaleMotionResultCannotCompleteGoal) {
  start();
  manager.planned(1, ports.op, response(), now);
  manager.motion_status("older-operation", 3, "done", now);
  EXPECT_EQ(manager.context()->state, State::Moving);
}
TEST_F(NavigationTest, OriginalPoseIsUsedAfterStartEscape) {
  start();
  auto r = response();
  r.start_recovery = true;
  r.path.poses.back().pose.position.x = 2.35;
  manager.planned(1, ports.op, std::move(r), now);
  ASSERT_EQ(ports.moves, 1);
  robot.pose.position.x = 2.35;
  manager.update_pose(robot, odometry, now);
  manager.motion_status("1", 3, "done", now);
  stop_ack();
  EXPECT_DOUBLE_EQ(manager.context()->original.pose.position.x, 3.);
  EXPECT_FALSE(manager.context()->start_escape);
  now += std::chrono::seconds(1);
  manager.monitor.sensors_received = now;
  manager.tick(now);
  EXPECT_EQ(ports.plans, 2);
  EXPECT_DOUBLE_EQ(ports.request.goal.pose.position.x, 3.);
  EXPECT_DOUBLE_EQ(ports.request.start.pose.position.x, 2.35);
}
TEST_F(NavigationTest, SensorWaitDoesNotSpinPlannerOrDisableSafety) {
  start();
  manager.planned(1, ports.op, response(), now);
  manager.monitor.sensors_ready = false;
  manager.tick(now);
  stop_ack();
  now += std::chrono::seconds(1);
  manager.tick(now);
  EXPECT_EQ(ports.plans, 1);
  EXPECT_EQ(manager.context()->state, State::Recovering);
}
TEST_F(NavigationTest, GeometrySignatureIgnoresStampsAndUnrelatedCells) {
  start();
  auto signature = manager.monitor.geometry_signature(*manager.context());
  auto map = std::make_shared<Grid>(*manager.monitor.map);
  map->header.stamp.sec = 100;
  map->data[59 * 60 + 59] = 100;
  manager.monitor.accept_map(map, now);
  EXPECT_EQ(signature, manager.monitor.geometry_signature(*manager.context()));
  map = std::make_shared<Grid>(*map);
  map->data[20 * 60 + 25] = 100;
  manager.monitor.accept_map(map, now);
  EXPECT_NE(signature, manager.monitor.geometry_signature(*manager.context()));
}
TEST_F(NavigationTest, OrientationCorrectionDoesNotNeedGlobalPlanning) {
  target = robot;
  target.pose.orientation.z = std::sin(.4);
  target.pose.orientation.w = std::cos(.4);
  start();
  EXPECT_EQ(ports.plans, 0);
  EXPECT_EQ(ports.moves, 1);
  EXPECT_EQ(ports.command.mode, Command::ROTATE);
  EXPECT_DOUBLE_EQ(ports.command.path.poses.back().pose.position.x, target.pose.position.x);
}
TEST_F(NavigationTest, StationaryControllerSuccessCannotBypassCollisionChecks) {
  start();
  manager.planned(1, ports.op, response(), now);
  robot = target;
  manager.update_pose(robot, odometry, now);
  auto map = std::make_shared<Grid>(*manager.monitor.map);
  map->data[20 * 60 + 30] = 100;
  manager.monitor.accept_map(map, now);
  manager.monitor.clearance = map;
  manager.motion_status("1", 3, "done", now);
  manager.tick(now);
  EXPECT_EQ(manager.context()->state, State::Verifying);
}
TEST_F(NavigationTest, ArrivalIsRecheckedAfterBraking) {
  start();
  manager.planned(1, ports.op, response(), now);
  robot = target;
  manager.update_pose(robot, odometry, now);
  manager.motion_status("1", 3, "done", now);
  manager.tick(now);
  robot.pose.position.x -= .1;
  stop_ack();
  EXPECT_EQ(manager.context()->result.result_code, Action::Result::FAILED);
  EXPECT_EQ(manager.context()->result.reason, "arrival_not_satisfied");
}
TEST(Recovery, ObservationRequiresRepeatedEquivalentGeometryAndHeadPermission) {
  RecoveryManager manager;
  NavigationContext c;
  c.request.move_head = true;
  auto now = Clock::now();
  c.equivalent_failures = 1;
  EXPECT_EQ(manager.select(c, "no_path", now), Operation::Replan);
  c.equivalent_failures = 3;
  EXPECT_EQ(manager.select(c, "no_path", now), Operation::Observe);
  c.request.move_head = false;
  EXPECT_EQ(manager.select(c, "no_path", now), Operation::Replan);
}
TEST_F(NavigationTest, LateBlockedGoalBrakesThenAlignsWithoutChangingOriginalGoal)
{
  robot.pose.position.x = .5;
  target.pose.position.x = 3.05;
  start();
  manager.planned(1, ports.op, response(), now);
  auto map = std::make_shared<Grid>(*manager.monitor.map);
  const int x = int(target.pose.position.x / map->info.resolution);
  const int y = int(target.pose.position.y / map->info.resolution);
  map->data[y * 60 + x] = 100;
  manager.monitor.accept_map(map, now);
  manager.monitor.clearance = map;
  robot.pose.position.x = 2.85;
  now += std::chrono::milliseconds(250);
  manager.update_pose(robot, odometry, now);
  manager.tick(now);
  ASSERT_EQ(ports.command.mode, Command::BRAKE);
  EXPECT_DOUBLE_EQ(manager.context()->original.pose.position.x, 3.05);
  manager.motion_status("2", 3, "base_stopped", now);
  ASSERT_EQ(ports.command.mode, Command::ROTATE);
  EXPECT_DOUBLE_EQ(ports.command.path.poses.back().pose.position.x, 2.85);
  manager.motion_status("3", 3, "pose_reached", now);
  manager.tick(now);
  stop_ack();
  EXPECT_EQ(manager.context()->result.result_code, Action::Result::APPROXIMATE);
  EXPECT_NEAR(manager.context()->result.position_error_m, .2, 1e-5);
}
TEST_F(NavigationTest, ExactArrivalMayRotateInsidePlanningPaddingWithPhysicalClearance)
{
  target = robot;
  target.pose.orientation.z = std::sin(.2);
  target.pose.orientation.w = std::cos(.2);
  auto physical = manager.monitor.map;
  auto map = std::make_shared<Grid>(*physical);
  const int x = int(robot.pose.position.x / map->info.resolution);
  const int y = int(robot.pose.position.y / map->info.resolution);
  map->data[y * 60 + x] = 100;
  manager.monitor.accept_map(map, now);
  manager.monitor.clearance = physical;
  start();
  ASSERT_EQ(ports.command.mode, Command::ROTATE);
  robot = target;
  manager.update_pose(robot, odometry, now);
  manager.motion_status("1", 3, "pose_reached", now);
  manager.tick(now);
  stop_ack();
  EXPECT_EQ(manager.context()->result.result_code, Action::Result::SUCCESS);
}
TEST(Recovery, EquivalentControllerFailuresShareTheStrategyBudget)
{
  RecoveryManager manager;
  NavigationContext context;
  const auto now = Clock::now();
  for (const auto & strategy : manager.strategies) {
    if (strategy->name == "replan") {strategy->budget.attempts = 2;}
  }
  EXPECT_TRUE(manager.select(context, "no_progress", now));
  EXPECT_TRUE(manager.select(context, "position_stalled", now));
  EXPECT_FALSE(manager.select(context, "orientation_stalled", now));
  manager.progress(context);
  EXPECT_TRUE(manager.select(context, "orientation_stalled", now));
}
TEST(Recovery, ExhaustedSensorBudgetCannotFallThroughToMotion)
{
  RecoveryManager manager;
  NavigationContext context;
  const auto now = Clock::now();
  for (const auto & strategy : manager.strategies) {
    if (strategy->name == "sensors") {strategy->budget.attempts = 1;}
  }
  EXPECT_EQ(manager.select(context, "obstacle_data_stale", now), Operation::WaitSensors);
  EXPECT_FALSE(manager.select(context, "obstacle_data_unavailable", now));
}
TEST_F(NavigationTest, ObservationDoesNotOverwriteAnAcceptedAlternative)
{
  auto map = std::make_shared<Grid>(*manager.monitor.map);
  const int x = int(target.pose.position.x / map->info.resolution);
  const int y = int(target.pose.position.y / map->info.resolution);
  map->data[y * 60 + x] = 100;
  manager.monitor.accept_map(map, now);
  manager.monitor.clearance = map;
  start();
  auto planned = response();
  planned.goal_relocated = true;
  planned.path.poses.back().pose.position.x -= .2;
  manager.planned(1, ports.op, std::move(planned), now);
  manager.motion_status("1", 4, "position_stalled", now);
  stop_ack();
  now += std::chrono::seconds(1);
  manager.monitor.sensors_received = now;
  manager.tick(now);
  ASSERT_EQ(manager.context()->state, State::Planning);
  Plan::Response failure;
  failure.reason = "no_path";
  failure.has_observation_target = true;
  failure.observation_target.x = 2.4;
  manager.planned(1, ports.op, std::move(failure), now);
  EXPECT_NEAR(manager.context()->operational.pose.position.x, 2.8, 1e-6);
  EXPECT_DOUBLE_EQ(manager.context()->observation_target.x, 2.4);
  EXPECT_DOUBLE_EQ(manager.context()->original.pose.position.x, 3.);
}
