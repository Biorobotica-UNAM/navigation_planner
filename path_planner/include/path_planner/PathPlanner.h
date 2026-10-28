#pragma once

#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>

#include <vector>

enum class PlanStatus
{
    Success,
    InvalidInput,
    OutsideMap,
    StartBlocked,
    GoalBlocked,
    NoPath
};

struct PlanOptions
{
    bool diagonal_paths = false;
    bool allow_unknown = false;
    double unknown_cost = 1.0;
    double goal_tolerance_m = 0.0;
    double escape_clear_radius = 0.6;
    bool disable_rear_path_plan = false;
    double online_behind_clearance = 1.0;
    double smooth_alpha = 0.1;
    double smooth_beta = 0.9;
    double smooth_tolerance = 0.00001;
};

struct PlanResult
{
    PlanStatus status = PlanStatus::InvalidInput;
    nav_msgs::msg::Path path;
    bool used_escape = false;
    bool goal_relocated = false;
};

class PathPlanner
{
public:
    // The original six-argument call remains valid through default arguments.
    static bool AStar(
        const nav_msgs::msg::OccupancyGrid& map,
        const nav_msgs::msg::OccupancyGrid& cost_map,
        const geometry_msgs::msg::Pose& start_pose,
        const geometry_msgs::msg::Pose& goal_pose,
        bool diagonal_paths,
        nav_msgs::msg::Path& result_path,
        const PlanOptions& options = {},
        PlanStatus* status = nullptr);

    // All segments and recovery attempts use the same input map pair.
    static PlanResult Plan(
        const nav_msgs::msg::OccupancyGrid& map,
        const nav_msgs::msg::OccupancyGrid& cost_map,
        const geometry_msgs::msg::Pose& start_pose,
        const geometry_msgs::msg::Pose& goal_pose,
        const PlanOptions& options = {},
        const std::vector<geometry_msgs::msg::Pose>& via_points = {});

    // Supply collision_map to validate all crossed cells after smoothing.
    static nav_msgs::msg::Path SmoothPath(
        const nav_msgs::msg::Path& path,
        float weight_data = 0.1f,
        float weight_smooth = 0.9f,
        float tolerance = 0.00001f,
        const nav_msgs::msg::OccupancyGrid* collision_map = nullptr,
        bool allow_unknown = false);
};
