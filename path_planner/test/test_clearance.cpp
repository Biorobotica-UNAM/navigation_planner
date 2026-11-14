#include <gtest/gtest.h>
#include "path_planner/PathPlanner.h"

#include <cmath>
#include <chrono>

namespace
{
nav_msgs::msg::OccupancyGrid make_map()
{
  nav_msgs::msg::OccupancyGrid grid;
  grid.header.frame_id = "map";
  grid.info.resolution = 0.05;
  grid.info.width = grid.info.height = 100;
  grid.info.origin.orientation.w = 1.0;
  grid.data.assign(10000, 0);
  return grid;
}

geometry_msgs::msg::Pose pose(double x, double y, double yaw = 0.0)
{
  geometry_msgs::msg::Pose result;
  result.position.x = x;
  result.position.y = y;
  result.orientation.z = std::sin(yaw / 2.0);
  result.orientation.w = std::cos(yaw / 2.0);
  return result;
}
}

TEST(PathQuality, StraightRouteAndFinalOrientation)
{
  const auto grid = make_map();
  PlanOptions options;
  options.diagonal_paths = true;
  const auto result = PathPlanner::Plan(grid, grid, pose(0.5, 0.5), pose(4.0, 0.5, 1.2), options);
  ASSERT_EQ(result.status, PlanStatus::Success);
  EXPECT_TRUE(PathPlanner::IsSmooth(result.path, options.max_path_curvature, options.max_path_heading_step));
  EXPECT_NEAR(result.path.poses.back().pose.orientation.z, std::sin(0.6), 1e-6);
}

TEST(PathQuality, UnknownPolicyPreservesOccupiedAndMapBoundsChecks)
{
  auto grid = make_map();
  const auto costs = grid;
  for (int y = 0; y < 100; ++y) {grid.data[y * 100 + 50] = -1;}
  const auto unchanged = grid.data;
  path_planner::MotionConstraints constraints;
  PlanOptions options;
  options.diagonal_paths = true;
  options.motion_constraints = &constraints;
  for (const bool allow_unknown : {false, true}) {
    options.allow_unknown = allow_unknown;
    constraints.update(grid, geometry_msgs::msg::PoseArray(), 0.28, allow_unknown);
    const auto result = PathPlanner::Plan(grid, costs, pose(1, 2), pose(4, 2), options);
    EXPECT_EQ(result.status, allow_unknown ? PlanStatus::Success : PlanStatus::NoPath);
    EXPECT_EQ(grid.data, unchanged);
  }
  // Unknown permission never permits leaving the grid or crossing occupied cells.
  EXPECT_EQ(PathPlanner::Plan(grid, costs, pose(1, 2), pose(6, 2), options).status,
    PlanStatus::OutsideMap);
  EXPECT_FALSE(constraints.sampling_free(1, 2, 6, 2));
  for (int y = 0; y < 100; ++y) {grid.data[y * 100 + 50] = 100;}
  constraints.update(grid, geometry_msgs::msg::PoseArray(), 0.28, true);
  EXPECT_EQ(PathPlanner::Plan(grid, costs, pose(1, 2), pose(4, 2), options).status,
    PlanStatus::NoPath);
  EXPECT_FALSE(constraints.sampling_free(1, 2, 4, 2));
}

TEST(PathQuality, RejectsSharpCornerAndAcceptsStraightDuplicatePoints)
{
  nav_msgs::msg::Path route;
  geometry_msgs::msg::PoseStamped p;
  for (const auto & point : {pose(0, 0), pose(1, 0), pose(1, 1)}) {
    p.pose = point; route.poses.push_back(p);
  }
  EXPECT_FALSE(PathPlanner::IsSmooth(route, 4.0, 0.35));
  route.poses[2].pose = pose(1, 0);
  EXPECT_TRUE(PathPlanner::IsSmooth(route, 4.0, 0.35));
}

TEST(PathQuality, SmoothingRemainsFiniteWithExistingLaunchWeights)
{
  nav_msgs::msg::Path route;
  geometry_msgs::msg::PoseStamped p;
  for (int i = 0; i < 100; ++i) {
    p.pose = pose(0.04 * i, 0.1 * (i % 2));
    route.poses.push_back(p);
  }
  auto smoothed = PathPlanner::SmoothPath(route, 0.1f, 0.9f);
  ASSERT_EQ(smoothed.poses.size(), route.poses.size());
  for (const auto & point : smoothed.poses) {
    EXPECT_TRUE(std::isfinite(point.pose.position.x));
    EXPECT_TRUE(std::isfinite(point.pose.position.y));
    EXPECT_LT(std::abs(point.pose.position.y), 0.2);
  }
}

TEST(PathQuality, WallCannotBeSmoothedOrRecoveredThrough)
{
  auto grid = make_map();
  for (int y = 0; y < 100; ++y) {grid.data[y * 100 + 50] = 100;}
  const auto result = PathPlanner::Plan(grid, grid, pose(1, 2), pose(4, 2));
  EXPECT_EQ(result.status, PlanStatus::NoPath);
  EXPECT_TRUE(result.path.poses.empty());
}

TEST(PathQuality, OccupiedStartDoesNotEraseFootprintClearance)
{
  auto grid = make_map();
  grid.data[20 * 100 + 20] = 100;
  const auto result = PathPlanner::Plan(grid, grid, pose(1.025, 1.025), pose(4, 2));
  EXPECT_EQ(result.status, PlanStatus::StartBlocked);
  EXPECT_TRUE(result.path.poses.empty());
}

TEST(PathQuality, ObstacleDetourIsSmoothAndCollisionFree)
{
  auto grid = make_map();
  for (int y = 20; y <= 60; ++y) {
    for (int x = 40; x <= 50; ++x) {grid.data[y * 100 + x] = 100;}
  }
  PlanOptions options;
  options.diagonal_paths = true;
  options.smoothing_time_budget_sec = 2.0;
  const auto result = PathPlanner::Plan(grid, grid, pose(0.5, 2), pose(4, 2), options);
  ASSERT_EQ(result.status, PlanStatus::Success);
  EXPECT_TRUE(PathPlanner::IsSmooth(result.path, options.max_path_curvature, options.max_path_heading_step));
  for (size_t i = 1; i < result.path.poses.size(); ++i) {
    const auto & a = result.path.poses[i - 1].pose.position;
    const auto & b = result.path.poses[i].pose.position;
    for (int sample = 0; sample <= 100; ++sample) {
      const double t = sample / 100.0;
      const int x = static_cast<int>((a.x + t * (b.x - a.x)) / grid.info.resolution);
      const int y = static_cast<int>((a.y + t * (b.y - a.y)) / grid.info.resolution);
      ASSERT_GE(x, 0); ASSERT_LT(x, 100); ASSERT_GE(y, 0); ASSERT_LT(y, 100);
      EXPECT_EQ(grid.data[y * 100 + x], 0);
    }
  }
}

TEST(PathQuality, ImpossibleCurvatureReturnsNoSmoothPath)
{
  auto grid = make_map();
  for (int y = 20; y <= 60; ++y) {
    for (int x = 40; x <= 50; ++x) {grid.data[y * 100 + x] = 100;}
  }
  PlanOptions options;
  options.diagonal_paths = true;
  options.max_path_curvature = 0.000001;
  const auto result = PathPlanner::Plan(grid, grid, pose(0.5, 2), pose(4, 2), options);
  EXPECT_EQ(result.status, PlanStatus::NoSmoothPath);
  EXPECT_TRUE(result.path.poses.empty());
}

TEST(PathQuality, RecoveryCostsCannotRemoveOccupiedCells)
{
  auto grid = make_map();
  for (int y = 0; y < 100; ++y) {grid.data[y * 100 + 50] = 100;}
  nav_msgs::msg::Path failed;
  failed.header.frame_id = "map";
  for (int i = 0; i < 30; ++i) {
    geometry_msgs::msg::PoseStamped p;
    p.pose = pose(1.0 + i * 0.1, 2.0);
    failed.poses.push_back(p);
  }
  const auto result = PathPlanner::Plan(grid, grid, pose(1, 2), pose(4, 2), {}, {}, {failed});
  EXPECT_EQ(result.status, PlanStatus::NoPath);
  EXPECT_TRUE(result.path.poses.empty());
}

TEST(PathQuality, TinyEndpointSegmentsDoNotDominateMetricCurvature)
{
  nav_msgs::msg::Path route;
  for (int i = 0; i <= 80; ++i) {
    geometry_msgs::msg::PoseStamped p;
    const double angle = M_PI * 0.5 * i / 80;
    p.pose = pose(0.4 * std::sin(angle), 0.4 * (1.0 - std::cos(angle)));
    route.poses.push_back(p);
  }
  auto endpoint = route.poses.back();
  endpoint.pose.position.x += 0.0005;
  endpoint.pose.position.y += 0.0001;
  route.poses.push_back(endpoint);
  EXPECT_TRUE(PathPlanner::IsSmooth(route, 6.0, 0.50));
}

TEST(PathQuality, GridStairStepsCannotAliasToASmoothStraightLine)
{
  nav_msgs::msg::Path route;
  for (int i = 0; i <= 40; ++i) {
    geometry_msgs::msg::PoseStamped p;
    p.pose = pose(i * 0.025, i % 2 ? 0.025 : 0.0);
    route.poses.push_back(p);
  }
  EXPECT_FALSE(PathPlanner::IsSmooth(route, 6.0, 0.50));
}

TEST(PathQuality, SmoothingPreservesPreferredCornerClearance)
{
  const auto collision = make_map();
  auto costs = collision;
  for (int y = 0; y < 60; ++y) {
    for (int x = 0; x < 60; ++x) {costs.data[y * 100 + x] = 40;}
  }
  nav_msgs::msg::Path route;
  route.header.frame_id = "map";
  for (int i = 0; i <= 40; ++i) {
    geometry_msgs::msg::PoseStamped p;
    p.pose = i <= 20 ? pose(1.025 + i * 0.1, 3.025) : pose(3.025, 3.025 - (i - 20) * 0.1);
    route.poses.push_back(p);
  }
  const auto smooth = PathPlanner::SmoothPath(route, 0.1f, 0.9f, 0.00001f, &collision, false, &costs);
  for (size_t i = 1; i < smooth.poses.size(); ++i) {
    for (int j = 0; j <= 20; ++j) {
      const auto & a = smooth.poses[i - 1].pose.position;
      const auto & b = smooth.poses[i].pose.position;
      const int x = int((a.x + (b.x - a.x) * j / 20) / costs.info.resolution);
      const int y = int((a.y + (b.y - a.y) * j / 20) / costs.info.resolution);
      EXPECT_LE(costs.data[y * 100 + x], 2);
    }
  }
}

TEST(PathQuality, InflatedStartUsesNearestReachableFreeEntryWithoutClearingObstacles)
{
  auto planning = make_map(), clearance = make_map();
  for (int y = 0; y < 100; ++y) {
    for (int x = 0; x < 100; ++x) {
      const double distance = std::hypot(x - 40, y - 40) * planning.info.resolution;
      planning.data[y * 100 + x] = distance <= 0.40 ? 100 : 0;
      clearance.data[y * 100 + x] = distance <= 0.23 ? 100 : 0;
    }
  }
  const auto original = planning.data;
  const auto start = pose(2.325, 2.025);
  const auto result = PathPlanner::Plan(planning, planning, start, pose(4, 2.025), {}, {}, {}, &clearance);
  ASSERT_EQ(result.status, PlanStatus::Success) << result.detail;
  EXPECT_TRUE(result.used_escape);
  ASSERT_GT(result.path.poses.size(), 2U);
  EXPECT_DOUBLE_EQ(result.path.poses.front().pose.position.x, start.position.x);
  const auto & entry = result.path.poses[1].pose.position;
  const int x = int(entry.x / planning.info.resolution), y = int(entry.y / planning.info.resolution);
  EXPECT_EQ(planning.data[y * 100 + x], 0);
  EXPECT_EQ(planning.data, original);
  for (int i = 0; i <= 100; ++i) {
    const int px = int((start.position.x + (entry.x - start.position.x) * i / 100) / clearance.info.resolution);
    const int py = int((start.position.y + (entry.y - start.position.y) * i / 100) / clearance.info.resolution);
    EXPECT_EQ(clearance.data[py * 100 + px], 0);
  }
  clearance.data[40 * 100 + 46] = 100;
  EXPECT_EQ(PathPlanner::Plan(planning, planning, start, pose(4, 2), {}, {}, {}, &clearance).status,
    PlanStatus::StartBlocked);
}

TEST(PathQuality, LocalStartRecoveryCanTurnAroundPhysicalObstacle)
{
  auto planning = make_map(), physical = make_map();
  std::fill(planning.data.begin(), planning.data.end(), 100);
  for (int x = 44; x < 100; ++x) planning.data[40*100+x] = 0;
  for (int y = 38; y <= 41; ++y) physical.data[y*100+41] = 100;
  const auto original = planning.data;
  PlanOptions options;options.diagonal_paths = true;
  const auto result = PathPlanner::RecoverStart(planning, physical, pose(2.025, 2.025), options);
  ASSERT_EQ(result.status, PlanStatus::Success) << result.detail;
  ASSERT_GT(result.path.poses.size(), 2U);
  EXPECT_NEAR(result.path.poses.back().pose.position.x, 2.225, 1e-6);
  for (size_t i = 1; i < result.path.poses.size(); ++i) {
    const auto & a = result.path.poses[i-1].pose.position;
    const auto & b = result.path.poses[i].pose.position;
    for (int j = 0; j <= 100; ++j) {
      const int x = int((a.x+(b.x-a.x)*j/100)/physical.info.resolution);
      const int y = int((a.y+(b.y-a.y)*j/100)/physical.info.resolution);
      ASSERT_EQ(physical.data[y*100+x], 0);
    }
  }
  EXPECT_EQ(planning.data, original);
}

TEST(PathQuality, LocalMaskCannotEraseUnknownOrReachDistantExit)
{
  auto planning = make_map(), physical = make_map();
  std::fill(planning.data.begin(), planning.data.end(), 100);
  planning.data[40*100+48] = 0;
  EXPECT_EQ(PathPlanner::RecoverStart(planning, physical, pose(2.025, 2.025)).status, PlanStatus::StartBlocked);
  PlanOptions options;options.start_recovery_mask_radius = 0.45;
  EXPECT_EQ(PathPlanner::RecoverStart(planning, physical, pose(2.025, 2.025), options).status, PlanStatus::Success);
  physical.data[40*100+40] = -1;
  EXPECT_EQ(PathPlanner::RecoverStart(planning, physical, pose(2.025, 2.025), options).status, PlanStatus::StartBlocked);
}

TEST(PathQuality, BlockedGoalAlternativeIsNearestReachableAndKeepsOrientation)
{
  auto grid = make_map();
  for (int y = 0; y < 100; ++y) grid.data[y*100+50] = 100;
  grid.data[50*100+51] = 100;
  PlanOptions options;options.diagonal_paths = true;options.goal_tolerance_m = 0.33;
  const auto result = PathPlanner::Plan(grid, grid, pose(2.025, 2.525), pose(2.575, 2.525, 1.0), options);
  ASSERT_EQ(result.status, PlanStatus::Success) << result.detail;
  EXPECT_TRUE(result.goal_relocated);
  EXPECT_NEAR(result.path.poses.back().pose.position.x, 2.475, 1e-6);
  EXPECT_NEAR(result.path.poses.back().pose.orientation.z, std::sin(0.5), 1e-6);
  options.goal_tolerance_m = 0.05;
  EXPECT_NE(PathPlanner::Plan(grid, grid, pose(2.025, 2.525), pose(2.575, 2.525), options).status, PlanStatus::Success);
}

TEST(PathQuality, LocalRecoveryCostDoesNotGrowWithGlobalGridArea)
{
  auto planning = make_map();
  planning.info.width = planning.info.height = 1000;
  planning.data.assign(1000000, 0);
  const auto physical = planning;
  for (int y = 36; y <= 44; ++y)
    for (int x = 36; x <= 44; ++x) planning.data[y*1000+x] = 100;
  const auto before = std::chrono::steady_clock::now();
  for (int i = 0; i < 100; ++i)
    ASSERT_EQ(PathPlanner::RecoverStart(planning, physical, pose(2.025, 2.025)).status, PlanStatus::Success);
  const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now()-before).count();
  RecordProperty("local_recovery_100_requests_ms", std::to_string(ms));
}
