#include <gtest/gtest.h>
#include <path_planner/PathPlanner.h>
#include <map_augmenter/grid_utils.hpp>

#include <fstream>
#include <random>
#include <sstream>

namespace
{
class LaboratoryConstraints : public ::testing::Test
{
protected:
  static nav_msgs::msg::OccupancyGrid planning, physical, costs;

  static void SetUpTestSuite()
  {
    std::vector<nav_msgs::msg::OccupancyGrid> maps;
    for (const std::string layer : {"maps", "prohibition_maps"}) {
      const std::string directory = std::string(NAVIGATION_MAPS_DIR) + "/" + layer + "/lab_2026/";
      std::ifstream yaml(directory + "map.yaml");
      ASSERT_TRUE(yaml.good());
      nav_msgs::msg::OccupancyGrid grid;
      grid.header.frame_id = "map";grid.info.origin.orientation.w = 1.0;
      double occupied = 0.65, free = 0.196, yaw = 0.0;
      int negate = 0;
      std::string line, image;
      while (std::getline(yaml, line)) {
        const auto colon = line.find(':');
        if (colon == std::string::npos) {continue;}
        const auto key = line.substr(0, colon), value = line.substr(colon + 1);
        if (key == "image") {std::istringstream(value) >> image;}
        else if (key == "resolution") {grid.info.resolution = std::stod(value);}
        else if (key == "occupied_thresh") {occupied = std::stod(value);}
        else if (key == "free_thresh") {free = std::stod(value);}
        else if (key == "negate") {negate = std::stoi(value);}
        else if (key == "origin") {
          auto coordinates = value.substr(value.find('[') + 1);
          std::replace(coordinates.begin(), coordinates.end(), ',', ' ');
          std::istringstream(coordinates) >> grid.info.origin.position.x >> grid.info.origin.position.y >> yaw;
        }
      }
      grid.info.origin.orientation.z = std::sin(yaw / 2.0);
      grid.info.origin.orientation.w = std::cos(yaw / 2.0);
      std::ifstream pgm(directory + image, std::ios::binary);
      ASSERT_TRUE(pgm.good());
      std::string token;
      std::vector<std::string> header;
      while (header.size() < 4 && pgm >> token) {
        if (token.front() == '#') {std::getline(pgm, line);continue;}
        header.push_back(token);
      }
      ASSERT_EQ(header.size(), 4u);ASSERT_EQ(header[0], "P5");ASSERT_EQ(header[3], "255");
      grid.info.width = std::stoul(header[1]);grid.info.height = std::stoul(header[2]);
      pgm.get();
      std::vector<unsigned char> pixels(static_cast<size_t>(grid.info.width) * grid.info.height);
      pgm.read(reinterpret_cast<char*>(pixels.data()), pixels.size());
      ASSERT_EQ(static_cast<size_t>(pgm.gcount()), pixels.size());
      grid.data.resize(pixels.size());
      for (size_t y = 0; y < grid.info.height; ++y) {
        for (size_t x = 0; x < grid.info.width; ++x) {
          const double probability = negate ? pixels[y * grid.info.width + x] / 255.0 :
            1.0 - pixels[y * grid.info.width + x] / 255.0;
          grid.data[(grid.info.height - y - 1) * grid.info.width + x] =
            probability > occupied ? 100 : probability < free ? 0 : -1;
        }
      }
      maps.push_back(std::move(grid));
    }
    const auto raw = map_augmenter::overlay_obstacles(maps[0], maps[1]);
    planning = map_augmenter::inflate_obstacles(raw, 0.25, true);
    physical = map_augmenter::inflate_obstacles(raw, 0.23, true, false);
    costs = map_augmenter::make_cost_map(planning, 0.90, 15, true);
  }
};

nav_msgs::msg::OccupancyGrid LaboratoryConstraints::planning;
nav_msgs::msg::OccupancyGrid LaboratoryConstraints::physical;
nav_msgs::msg::OccupancyGrid LaboratoryConstraints::costs;

TEST_F(LaboratoryConstraints, SweptSegmentsProtectUncertaintyAndPermitEscape)
{
  geometry_msgs::msg::PoseArray points;
  points.poses.resize(1);points.poses[0].position.x = -1.850;points.poses[0].position.y = -3.139;
  path_planner::MotionConstraints constraints;
  constraints.update(physical, points, 0.28);
  EXPECT_TRUE(constraints.uncertainty_free(-2.100, -3.139, -2.200, -3.139));
  EXPECT_TRUE(constraints.uncertainty_free(-2.100, -3.139, -2.100, -3.000));
  EXPECT_FALSE(constraints.uncertainty_free(-2.100, -3.139, -2.000, -3.139));
  EXPECT_FALSE(constraints.uncertainty_free(-2.200, -3.139, -1.500, -3.139));
  EXPECT_FALSE(constraints.uncertainty_free(-2.100, -3.139, -2.100, -3.139, false));
  EXPECT_TRUE(constraints.sampling_free(-2.100, -3.139, -2.200, -3.139));
}

TEST_F(LaboratoryConstraints, SpatialBucketsMatchFullSearchWithRotatedMap)
{
  auto rotated = physical;
  rotated.info.origin.orientation.z = std::sin(0.35);
  rotated.info.origin.orientation.w = std::cos(0.35);
  std::mt19937 random(184);
  std::uniform_real_distribution<double> x(0.0, rotated.info.width * rotated.info.resolution);
  std::uniform_real_distribution<double> y(0.0, rotated.info.height * rotated.info.resolution);
  std::uniform_real_distribution<double> offset(-0.4, 0.4);
  geometry_msgs::msg::PoseArray points;
  for (int i = 0; i < 500; ++i) {
    const double px = x(random), py = y(random);
    geometry_msgs::msg::Pose pose;
    pose.position.x = rotated.info.origin.position.x + std::cos(0.7) * px - std::sin(0.7) * py;
    pose.position.y = rotated.info.origin.position.y + std::sin(0.7) * px + std::cos(0.7) * py;
    points.poses.push_back(pose);
  }
  path_planner::MotionConstraints constraints;
  constraints.update(rotated, points, 0.28);
  for (int sample = 0; sample < 3000; ++sample) {
    const double px = x(random), py = y(random);
    const double ax = rotated.info.origin.position.x + std::cos(0.7) * px - std::sin(0.7) * py;
    const double ay = rotated.info.origin.position.y + std::sin(0.7) * px + std::cos(0.7) * py;
    const double dx = offset(random), dy = offset(random), length2 = dx * dx + dy * dy;
    bool expected = true;
    for (const auto& pose : points.poses) {
      const double ex = ax - pose.position.x, ey = ay - pose.position.y;
      const double t = length2 > 1e-12 ? std::clamp(-(ex * dx + ey * dy) / length2, 0.0, 1.0) : 0.0;
      if (std::pow(ex + t * dx, 2) + std::pow(ey + t * dy, 2) + 1e-8 <
        std::min(ex * ex + ey * ey, 0.28 * 0.28)) {expected = false;break;}
    }
    ASSERT_EQ(constraints.uncertainty_free(ax, ay, ax + dx, ay + dy), expected) << sample;
  }
  points.poses.clear();constraints.update(rotated, points, 0.28);
  EXPECT_TRUE(constraints.uncertainty_free(-2.2, -3.139, -1.5, -3.139));
}

TEST_F(LaboratoryConstraints, SearchAndSmoothingUseTheSameConstraints)
{
  geometry_msgs::msg::PoseArray points;
  points.poses.resize(1);points.poses[0].position.x = -2.127;points.poses[0].position.y = -1.834;
  path_planner::MotionConstraints constraints;
  constraints.update(planning, points, 0.28);
  PlanOptions options;options.diagonal_paths = true;options.motion_constraints = &constraints;
  geometry_msgs::msg::Pose start, goal;
  start.position.x = -2.245;start.position.y = -1.008;start.orientation.w = 1.0;
  goal.position.x = -2.009;goal.position.y = -2.660;goal.orientation.w = 1.0;
  const auto result = PathPlanner::Plan(planning, costs, start, goal, options);
  ASSERT_EQ(result.status, PlanStatus::Success) << result.detail;
  for (size_t i = 1; i < result.path.poses.size(); ++i) {
    const auto& a = result.path.poses[i - 1].pose.position;
    const auto& b = result.path.poses[i].pose.position;
    EXPECT_TRUE(constraints.uncertainty_free(a.x, a.y, b.x, b.y));
    EXPECT_TRUE(constraints.sampling_free(a.x, a.y, b.x, b.y));
  }
}

TEST_F(LaboratoryConstraints, OccupiedGoalRelocatesWithoutChangingPhysicalFootprint)
{
  geometry_msgs::msg::Pose start, goal;
  start.position.x = -2.100;start.position.y = -3.139;start.orientation.w = 1.0;
  goal.position.x = -1.69942;goal.position.y = -2.89339;goal.orientation.w = 1.0;
  geometry_msgs::msg::PoseArray points;points.poses.push_back(goal);
  path_planner::MotionConstraints constraints;
  constraints.update(planning, points, 0.28);
  PlanOptions options;options.diagonal_paths = true;options.motion_constraints = &constraints;
  EXPECT_EQ(PathPlanner::Plan(planning, costs, start, goal, options).status, PlanStatus::GoalBlocked);
  options.goal_tolerance_m = std::nextafter(0.33, 0.0);
  const auto unchanged = physical.data;
  const auto result = PathPlanner::Plan(planning, costs, start, goal, options);
  ASSERT_EQ(result.status, PlanStatus::Success) << result.detail;
  EXPECT_TRUE(result.goal_relocated);
  const auto& p = result.path.poses.back().pose.position;
  EXPECT_GE(std::hypot(p.x - goal.position.x, p.y - goal.position.y), 0.28 - 1e-8);
  EXPECT_LT(std::hypot(p.x - goal.position.x, p.y - goal.position.y), 0.33);
  EXPECT_EQ(physical.data, unchanged);
}

TEST_F(LaboratoryConstraints, GuardedGoalKeepsTheRequestedPositionTolerance)
{
  geometry_msgs::msg::Pose start, goal;
  start.position.x = 0.041;start.position.y = 0.940;start.orientation.w = 1.0;
  goal.position.x = -0.279;goal.position.y = -4.110;goal.orientation.w = 1.0;
  path_planner::MotionConstraints constraints;
  constraints.update(planning, geometry_msgs::msg::PoseArray(), 0.28);
  EXPECT_FALSE(constraints.sampling_free(goal.position.x, goal.position.y,
    goal.position.x, goal.position.y, false));
  PlanOptions options;options.diagonal_paths = true;options.motion_constraints = &constraints;
  EXPECT_EQ(PathPlanner::Plan(planning, costs, start, goal, options).status, PlanStatus::GoalBlocked);
  options.goal_tolerance_m = 0.08;
  const auto result = PathPlanner::Plan(planning, costs, start, goal, options);
  ASSERT_EQ(result.status, PlanStatus::Success) << result.detail;
  const auto& endpoint = result.path.poses.back().pose.position;
  EXPECT_LE(std::hypot(endpoint.x - goal.position.x, endpoint.y - goal.position.y), 0.08);
}
}  // namespace
