#include <gtest/gtest.h>
#include "path_planner/PathPlanner.h"

#include <cmath>

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
