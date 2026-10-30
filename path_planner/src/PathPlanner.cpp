#include "path_planner/PathPlanner.h"

#include <algorithm>
#include <cmath>
#include <chrono>
#include <queue>
#include <cstdint>
#include <functional>
#include <limits>
#include <queue>
#include <stdexcept>
#include <tuple>
#include <utility>
#include <vector>

namespace
{
using Grid = nav_msgs::msg::OccupancyGrid;
using Path = nav_msgs::msg::Path;
using Pose = geometry_msgs::msg::Pose;
constexpr double geometry_epsilon = 1e-9;
constexpr size_t no_parent = std::numeric_limits<size_t>::max();

struct GridGeometry
{
    double ox, oy, oz, resolution, yaw, c, s;
    int width, height;

    explicit GridGeometry(const Grid& grid)
    {
        const auto& info = grid.info;
        const auto& p = info.origin.position;
        const auto& q = info.origin.orientation;
        const uint64_t cells = static_cast<uint64_t>(info.width) * info.height;
        const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
        if (grid.header.frame_id.empty() || !std::isfinite(info.resolution) ||
            info.resolution <= 0.0 || info.width == 0 || info.height == 0 ||
            info.width > static_cast<uint32_t>(std::numeric_limits<int>::max()) ||
            info.height > static_cast<uint32_t>(std::numeric_limits<int>::max()) ||
            cells != grid.data.size() || !std::isfinite(p.x) || !std::isfinite(p.y) ||
            !std::isfinite(p.z) || !std::isfinite(norm) ||
            std::abs(norm - 1.0) > 1e-5 || std::abs(q.x) > geometry_epsilon ||
            std::abs(q.y) > geometry_epsilon)
            throw std::invalid_argument("Invalid planar occupancy-grid geometry");

        ox = p.x;
        oy = p.y;
        oz = p.z;
        resolution = info.resolution;
        width = static_cast<int>(info.width);
        height = static_cast<int>(info.height);
        yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z));
        c = std::cos(yaw);
        s = std::sin(yaw);
    }

    bool worldToCell(double wx, double wy, int& x, int& y) const
    {
        const double dx = wx - ox;
        const double dy = wy - oy;
        const double gx = (c * dx + s * dy) / resolution;
        const double gy = (-s * dx + c * dy) / resolution;
        if (!std::isfinite(gx) || !std::isfinite(gy) ||
            gx < 0.0 || gy < 0.0 || gx >= width || gy >= height)
            return false;
        x = static_cast<int>(std::floor(gx));
        y = static_cast<int>(std::floor(gy));
        return true;
    }

    geometry_msgs::msg::Point cellCenter(int x, int y) const
    {
        const double gx = (x + 0.5) * resolution;
        const double gy = (y + 0.5) * resolution;
        geometry_msgs::msg::Point point;
        point.x = ox + c * gx - s * gy;
        point.y = oy + s * gx + c * gy;
        point.z = oz;
        return point;
    }
};

void finishPath(Path& path)
{
    if (path.poses.empty())
        return;
    auto final_orientation = path.poses.back().pose.orientation;
    const double norm = std::sqrt(
        final_orientation.x * final_orientation.x +
        final_orientation.y * final_orientation.y +
        final_orientation.z * final_orientation.z +
        final_orientation.w * final_orientation.w);
    const bool preserve_final = std::isfinite(norm) && norm > geometry_epsilon;
    double heading = 0.0;
    for (size_t i = 0; i < path.poses.size(); ++i)
    {
        auto& pose = path.poses[i];
        pose.header = path.header;
        if (i + 1 < path.poses.size())
        {
            const auto& next = path.poses[i + 1].pose.position;
            const auto& current = pose.pose.position;
            heading = std::atan2(next.y - current.y, next.x - current.x);
        }
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = std::sin(heading / 2.0);
        pose.pose.orientation.w = std::cos(heading / 2.0);
    }
    if (preserve_final)
    {
        final_orientation.x /= norm;
        final_orientation.y /= norm;
        final_orientation.z /= norm;
        final_orientation.w /= norm;
        path.poses.back().pose.orientation = final_orientation;
    }
}

bool validatePathSegments(const Path& path, const Grid& map, bool allow_unknown,
                          const Grid* costs = nullptr, const Path* original = nullptr)
{
    if (path.poses.empty() || path.header.frame_id != map.header.frame_id)
        return false;
    const GridGeometry geometry(map);
    for (size_t i = 0; i < path.poses.size(); ++i)
    {
        // Preserve the local cost envelope, including every crossed segment cell.
        int cost_limit = 0;
        if (costs && original)
        {
            for (size_t j = i == 0 ? 0 : i - 1; j <= i; ++j)
            {
                int cx, cy;
                const auto& p = original->poses[j].pose.position;
                if (!geometry.worldToCell(p.x, p.y, cx, cy))
                    return false;
                cost_limit = std::max(cost_limit, static_cast<int>(costs->data[
                    static_cast<size_t>(cy) * geometry.width + cx]));
            }
        }
        const auto acceptable = [&](size_t index) {
            const int value = map.data[index];
            return (value == 0 || (value == -1 && allow_unknown)) &&
                (!costs || costs->data[index] <= cost_limit);
        };
        const auto& point = path.poses[i].pose.position;
        int end_x, end_y;
        if (!geometry.worldToCell(point.x, point.y, end_x, end_y))
            return false;
        const size_t end_index = static_cast<size_t>(end_y) * geometry.width + end_x;
        if (!acceptable(end_index))
            return false;
        if (i == 0)
            continue;

        const auto& previous = path.poses[i - 1].pose.position;
        const double start_x = (geometry.c * (previous.x - geometry.ox) +
                                geometry.s * (previous.y - geometry.oy)) / geometry.resolution;
        const double start_y = (-geometry.s * (previous.x - geometry.ox) +
                                 geometry.c * (previous.y - geometry.oy)) / geometry.resolution;
        const double target_x = (geometry.c * (point.x - geometry.ox) +
                                 geometry.s * (point.y - geometry.oy)) / geometry.resolution;
        const double target_y = (-geometry.s * (point.x - geometry.ox) +
                                  geometry.c * (point.y - geometry.oy)) / geometry.resolution;
        int x, y;
        if (!geometry.worldToCell(previous.x, previous.y, x, y))
            return false;
        if (!acceptable(static_cast<size_t>(y) * geometry.width + x))
            return false;
        const double dx = target_x - start_x;
        const double dy = target_y - start_y;
        const int step_x = dx > 0.0 ? 1 : (dx < 0.0 ? -1 : 0);
        const int step_y = dy > 0.0 ? 1 : (dy < 0.0 ? -1 : 0);
        const double infinity = std::numeric_limits<double>::infinity();
        const double delta_x = step_x == 0 ? infinity : 1.0 / std::abs(dx);
        const double delta_y = step_y == 0 ? infinity : 1.0 / std::abs(dy);
        double next_x = step_x == 0 ? infinity :
            (x + (step_x > 0 ? 1.0 : 0.0) - start_x) / dx;
        double next_y = step_y == 0 ? infinity :
            (y + (step_y > 0 ? 1.0 : 0.0) - start_y) / dy;

        // Traverse crossed cells; inspect both side cells at corner crossings.
        while (x != end_x || y != end_y)
        {
            if (std::min(next_x, next_y) >= 1.0 - 1e-12)
                break;
            if (std::abs(next_x - next_y) <= 1e-12)
            {
                const int sx = x + step_x;
                const int sy = y + step_y;
                if (sx < 0 || sx >= geometry.width || sy < 0 || sy >= geometry.height)
                    return false;
                if (!acceptable(static_cast<size_t>(y) * geometry.width + sx) ||
                    !acceptable(static_cast<size_t>(sy) * geometry.width + x))
                    return false;
                x = sx;
                y = sy;
                next_x += delta_x;
                next_y += delta_y;
            }
            else if (next_x < next_y)
            {
                x += step_x;
                next_x += delta_x;
            }
            else
            {
                y += step_y;
                next_y += delta_y;
            }
            if (x < 0 || y < 0 || x >= geometry.width || y >= geometry.height)
                return false;
            if (!acceptable(static_cast<size_t>(y) * geometry.width + x))
                return false;
        }
    }
    return true;
}

PlanResult planSegments(
    const Grid& map, const Grid& costs, const Pose& start,
    const std::vector<Pose>& stops, const PlanOptions& options,
    double final_tolerance, size_t& failed_segment)
{
    PlanResult result;
    result.path.header = map.header;
    Pose current_start = start;
    for (size_t i = 0; i < stops.size(); ++i)
    {
        failed_segment = i;
        PlanOptions segment_options = options;
        segment_options.goal_tolerance_m = i + 1 == stops.size() ? final_tolerance : 0.0;
        Path segment;
        if (!PathPlanner::AStar(map, costs, current_start, stops[i],
                               options.diagonal_paths, segment, segment_options, &result.status))
        {
            result.path.poses.clear();
            return result;
        }
        result.goal_relocated = i + 1 == stops.size() &&
            std::hypot(segment.poses.back().pose.position.x - stops[i].position.x,
                       segment.poses.back().pose.position.y - stops[i].position.y) > geometry_epsilon;
        Path smoothed;
        Grid alternate_costs = costs;
        Grid clearance_map = map;
        const Grid* shortcut_map = &map;
        const GridGeometry geometry(map);
        const auto deadline = std::chrono::steady_clock::now() +
            std::chrono::duration<double>(options.smoothing_time_budget_sec);
        bool accepted = false;
        for (int attempt = 0; attempt < options.smoothing_retry_limit; ++attempt)
        {
            if (std::chrono::steady_clock::now() >= deadline)
                break;
            // A second geometry is searched after the first smoothing schedule.
            // Penalize the previous route in soft costs; never erase hard obstacles.
            if (attempt == options.smoothing_retry_limit / 2 && attempt > 0)
            {
                for (const auto& pose : segment.poses)
                {
                    int gx, gy;
                    if (!geometry.worldToCell(pose.pose.position.x, pose.pose.position.y, gx, gy))
                        continue;
                    for (int y = std::max(0, gy - 2); y <= std::min(geometry.height - 1, gy + 2); ++y)
                        for (int x = std::max(0, gx - 2); x <= std::min(geometry.width - 1, gx + 2); ++x)
                        {
                            auto& cost = alternate_costs.data[static_cast<size_t>(y) * geometry.width + x];
                            cost = static_cast<int8_t>(std::min(90, std::max(0, static_cast<int>(cost)) + 2));
                        }
                }
                // Reserve turning room for the alternate search. This extra
                // clearance is optional; the real collision map is unchanged.
                const double radius = std::min(0.6, 1.0 / options.max_path_curvature + 2.0 * geometry.resolution);
                const int reach = static_cast<int>(std::ceil(radius / geometry.resolution));
                for (int y = 0; y < geometry.height; ++y)
                    for (int x = 0; x < geometry.width; ++x)
                    {
                        if (map.data[static_cast<size_t>(y) * geometry.width + x] <= 0)
                            continue;
                        for (int yy = std::max(0, y-reach); yy <= std::min(geometry.height-1, y+reach); ++yy)
                            for (int xx = std::max(0, x-reach); xx <= std::min(geometry.width-1, x+reach); ++xx)
                                if (std::hypot(xx-x, yy-y) * geometry.resolution <= radius)
                                    clearance_map.data[static_cast<size_t>(yy) * geometry.width + xx] = 100;
                    }
                Path alternate;
                PlanStatus status;
                if (PathPlanner::AStar(clearance_map, alternate_costs, current_start, stops[i],
                                      options.diagonal_paths, alternate, segment_options, &status))
                {
                    segment = std::move(alternate);
                    shortcut_map = &clearance_map;
                }
            }
            // Remove grid stair steps with collision-checked line-of-sight links.
            // Vary the shortcut reach between retries, then resample uniformly.
            Path sparse;
            sparse.header = segment.header;
            sparse.poses.push_back(segment.poses.front());
            for (size_t from = 0; from + 1 < segment.poses.size();)
            {
                size_t next = from + 1;
                const size_t reach = std::min(segment.poses.size() - 1, from + static_cast<size_t>(40 - 8 * (attempt % 4)));
                for (size_t to = reach; to > from + 1; --to)
                {
                    if (std::chrono::steady_clock::now() >= deadline)
                        break;
                    Path link;
                    link.header = segment.header;
                    link.poses = {segment.poses[from], segment.poses[to]};
                    if (validatePathSegments(link, *shortcut_map, options.allow_unknown))
                    {
                        next = to;
                        break;
                    }
                }
                sparse.poses.push_back(segment.poses[next]);
                from = next;
            }
            Path sampled;
            sampled.header = sparse.header;
            sampled.poses.push_back(sparse.poses.front());
            const double spacing = std::max(0.02, std::min(0.05, geometry.resolution));
            for (size_t j = 1; j < sparse.poses.size(); ++j)
            {
                const auto& a = sparse.poses[j-1].pose.position;
                const auto& b = sparse.poses[j].pose.position;
                const int count = std::max(1, static_cast<int>(std::ceil(std::hypot(b.x-a.x, b.y-a.y) / spacing)));
                for (int step = 1; step <= count; ++step)
                {
                    auto pose = sparse.poses[j];
                    const double t = static_cast<double>(step) / count;
                    pose.pose.position.x = a.x + t * (b.x-a.x);
                    pose.pose.position.y = a.y + t * (b.y-a.y);
                    sampled.poses.push_back(pose);
                }
            }
            const double data_weight = options.smooth_alpha * std::pow(0.5, attempt % 4);
            smoothed = PathPlanner::SmoothPath(sampled, static_cast<float>(data_weight),
                static_cast<float>(options.smooth_beta), static_cast<float>(options.smooth_tolerance),
                &map, options.allow_unknown);
            if (validatePathSegments(smoothed, map, options.allow_unknown) &&
                (!options.require_smooth_path || PathPlanner::IsSmooth(smoothed,
                    options.max_path_curvature, options.max_path_heading_step)))
            {
                accepted = true;
                break;
            }
        }
        if (!accepted)
        {
            result.status = PlanStatus::NoSmoothPath;
            result.path.poses.clear();
            return result;
        }
        auto first = smoothed.poses.begin();
        // Every segment starts at the endpoint already appended by the previous one.
        if (!result.path.poses.empty() && first != smoothed.poses.end())
            ++first;
        result.path.poses.insert(result.path.poses.end(), first, smoothed.poses.end());
        current_start = segment.poses.back().pose;
    }
    if (options.require_smooth_path && !PathPlanner::IsSmooth(result.path,
        options.max_path_curvature, options.max_path_heading_step))
    {
        result.status = PlanStatus::NoSmoothPath;
        result.path.poses.clear();
        return result;
    }
    result.status = PlanStatus::Success;
    finishPath(result.path);
    return result;
}
}  // namespace

bool PathPlanner::AStar(
    const Grid& map, const Grid& cost_map, const Pose& start_pose, const Pose& goal_pose,
    bool diagonal_paths, Path& result_path, const PlanOptions& options, PlanStatus* status)
{
    result_path = Path{};
    if (status)
        *status = PlanStatus::InvalidInput;
    try
    {
        const GridGeometry geometry(map);
        const GridGeometry cost_geometry(cost_map);
        if (!std::isfinite(start_pose.position.z) || !std::isfinite(goal_pose.position.z) ||
            !std::isfinite(options.unknown_cost) || options.unknown_cost < 0.0 ||
            !std::isfinite(options.goal_tolerance_m) || options.goal_tolerance_m < 0.0 ||
            map.header.frame_id != cost_map.header.frame_id ||
            geometry.width != cost_geometry.width || geometry.height != cost_geometry.height ||
            geometry.resolution != cost_geometry.resolution ||
            geometry.ox != cost_geometry.ox || geometry.oy != cost_geometry.oy ||
            geometry.oz != cost_geometry.oz ||
            std::abs(std::remainder(geometry.yaw - cost_geometry.yaw, 2.0 * std::acos(-1.0))) >
                geometry_epsilon)
            return false;

        int start_x, start_y, goal_x, goal_y;
        if (!geometry.worldToCell(start_pose.position.x, start_pose.position.y, start_x, start_y) ||
            !geometry.worldToCell(goal_pose.position.x, goal_pose.position.y, goal_x, goal_y))
        {
            if (status)
                *status = PlanStatus::OutsideMap;
            return false;
        }
        const size_t start = static_cast<size_t>(start_y) * geometry.width + start_x;
        const size_t goal = static_cast<size_t>(goal_y) * geometry.width + goal_x;
        const int8_t start_value = map.data[start];
        const int8_t goal_value = map.data[goal];
        if (start_value != 0 && !(start_value == -1 && options.allow_unknown))
        {
            if (status)
                *status = PlanStatus::StartBlocked;
            return false;
        }
        if (goal_value != 0 && !(goal_value == -1 && options.allow_unknown) &&
            options.goal_tolerance_m == 0.0)
        {
            if (status)
                *status = PlanStatus::GoalBlocked;
            return false;
        }

        struct SearchNode
        {
            double g = std::numeric_limits<double>::infinity();
            size_t parent = no_parent;
            bool closed = false;
        };
        std::vector<SearchNode> nodes(map.data.size());
        // Tuple priorities are immutable, including entries superseded by better costs.
        using Entry = std::tuple<double, double, size_t>;
        std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;
        const double initial_h = diagonal_paths ?
            std::hypot(start_x - goal_x, start_y - goal_y) :
            std::abs(start_x - goal_x) + std::abs(start_y - goal_y);
        nodes[start].g = 0.0;
        open.emplace(initial_h, 0.0, start);
        size_t endpoint = no_parent;
        size_t closest = start;
        double closest_distance = std::numeric_limits<double>::infinity();
        bool closest_is_start_pose = false;
        while (!open.empty())
        {
            const auto [priority, queued_g, index] = open.top();
            (void)priority;
            open.pop();
            if (nodes[index].closed || queued_g != nodes[index].g)
                continue;
            nodes[index].closed = true;
            const int x = static_cast<int>(index % geometry.width);
            const int y = static_cast<int>(index / geometry.width);
            const auto point = geometry.cellCenter(x, y);
            double distance = std::hypot(
                point.x - goal_pose.position.x, point.y - goal_pose.position.y);
            bool use_start_pose = false;
            if (index == start)
            {
                const double start_distance = std::hypot(
                    start_pose.position.x - goal_pose.position.x,
                    start_pose.position.y - goal_pose.position.y);
                if (start_distance <= distance)
                {
                    distance = start_distance;
                    use_start_pose = true;
                }
            }
            if (distance < closest_distance ||
                (distance == closest_distance && nodes[index].g < nodes[closest].g))
            {
                closest = index;
                closest_distance = distance;
                closest_is_start_pose = use_start_pose;
            }
            if (index == goal)
            {
                endpoint = index;
                break;
            }
            for (int dy = -1; dy <= 1; ++dy)
            {
                for (int dx = -1; dx <= 1; ++dx)
                {
                    if ((dx == 0 && dy == 0) || (!diagonal_paths && dx != 0 && dy != 0))
                        continue;
                    const int nx = x + dx;
                    const int ny = y + dy;
                    if (nx < 0 || ny < 0 || nx >= geometry.width || ny >= geometry.height)
                        continue;
                    const size_t next = static_cast<size_t>(ny) * geometry.width + nx;
                    const int8_t occupancy = map.data[next];
                    if ((occupancy != 0 && !(occupancy == -1 && options.allow_unknown)) ||
                        nodes[next].closed)
                        continue;
                    if (dx != 0 && dy != 0)
                    {
                        const int8_t side_x = map.data[static_cast<size_t>(y) * geometry.width + nx];
                        const int8_t side_y = map.data[static_cast<size_t>(ny) * geometry.width + x];
                        if ((side_x != 0 && !(side_x == -1 && options.allow_unknown)) ||
                            (side_y != 0 && !(side_y == -1 && options.allow_unknown)))
                            continue;
                    }
                    const double step = dx != 0 && dy != 0 ? std::sqrt(2.0) : 1.0;
                    const double extra_cost = occupancy == -1 ? options.unknown_cost :
                        std::max(0, static_cast<int>(cost_map.data[next]));
                    const double candidate = nodes[index].g + step + extra_cost;
                    if (candidate < nodes[next].g)
                    {
                        nodes[next].g = candidate;
                        nodes[next].parent = index;
                        const double heuristic = diagonal_paths ?
                            std::hypot(nx - goal_x, ny - goal_y) :
                            std::abs(nx - goal_x) + std::abs(ny - goal_y);
                        open.emplace(candidate + heuristic, candidate, next);
                    }
                }
            }
        }
        if (endpoint == no_parent)
        {
            // An exhausted search has visited the entire reachable component.
            if (options.goal_tolerance_m > 0.0 &&
                closest_distance <= options.goal_tolerance_m)
                endpoint = closest;
            else
            {
                if (status)
                    *status = PlanStatus::NoPath;
                return false;
            }
        }

        result_path.header = map.header;
        std::vector<size_t> cells;
        for (size_t index = endpoint; index != no_parent; index = nodes[index].parent)
            cells.push_back(index);
        std::reverse(cells.begin(), cells.end());
        result_path.poses.reserve(cells.size() + 1);
        for (const size_t index : cells)
        {
            geometry_msgs::msg::PoseStamped pose;
            pose.pose.position = geometry.cellCenter(
                static_cast<int>(index % geometry.width),
                static_cast<int>(index / geometry.width));
            result_path.poses.push_back(pose);
        }
        result_path.poses.front().pose = start_pose;
        if (endpoint == goal)
        {
            if (cells.size() == 1 &&
                std::hypot(goal_pose.position.x - start_pose.position.x,
                           goal_pose.position.y - start_pose.position.y) > geometry_epsilon)
                result_path.poses.emplace_back();
            result_path.poses.back().pose = goal_pose;
        }
        else if (cells.size() == 1 && !closest_is_start_pose)
        {
            // Relocation may require a short move inside the robot's current cell.
            geometry_msgs::msg::PoseStamped pose;
            pose.pose.position = geometry.cellCenter(start_x, start_y);
            result_path.poses.push_back(pose);
        }
        finishPath(result_path);
        if (status)
            *status = PlanStatus::Success;
        return true;
    }
    catch (const std::invalid_argument&)
    {
        return false;
    }
}

Path PathPlanner::SmoothPath(
    const Path& path, float weight_data, float weight_smooth, float tolerance,
    const Grid* collision_map, bool allow_unknown, const Grid* cost_map)
{
    Path result = path;
    if (!std::isfinite(weight_data) || !std::isfinite(weight_smooth) ||
        !std::isfinite(tolerance) || weight_data < 0.0f || weight_data > 1.0f ||
        weight_smooth < 0.0f || weight_smooth > 1.0f || tolerance <= 0.0f)
        throw std::invalid_argument("Invalid smoothing weights or tolerance");
    if (cost_map && (!collision_map ||
        (cost_map->header.frame_id != collision_map->header.frame_id ||
         cost_map->info.width != collision_map->info.width ||
         cost_map->info.height != collision_map->info.height ||
         cost_map->info.resolution != collision_map->info.resolution ||
         cost_map->info.origin != collision_map->info.origin ||
         cost_map->data.size() != collision_map->data.size())))
        throw std::invalid_argument("Cost map requires matching collision geometry");
    const double scale = std::min(1.0, 0.9 / std::max(1e-9,
        static_cast<double>(weight_data) + 2.0 * weight_smooth));
    weight_data *= scale;
    weight_smooth *= scale;
    const double threshold = static_cast<double>(tolerance) * path.poses.size();
    if (path.poses.size() >= 3)
    {
        for (int attempt = 0; attempt < 1000; ++attempt)
        {
            double change = 0.0;
            for (size_t i = 1; i + 1 < path.poses.size(); ++i)
            {
                auto& point = result.poses[i].pose.position;
                const auto old = point;
                const auto& original = path.poses[i].pose.position;
                const auto& previous = result.poses[i - 1].pose.position;
                const auto& next = result.poses[i + 1].pose.position;
                point.x += weight_data * (original.x - point.x) +
                    weight_smooth * (previous.x + next.x - 2.0 * point.x);
                point.y += weight_data * (original.y - point.y) +
                    weight_smooth * (previous.y + next.y - 2.0 * point.y);
                change += std::abs(point.x - old.x) + std::abs(point.y - old.y);
            }
            if (!std::isfinite(change))
            {
                result = path;
                break;
            }
            if (change < threshold)
                break;
        }
    }
    if (collision_map && !validatePathSegments(result, *collision_map, allow_unknown, cost_map, &path))
        result = path;
    finishPath(result);
    return result;
}

bool PathPlanner::IsSmooth(const Path& path, double max_curvature, double max_heading_step)
{
    if (path.poses.empty() || !std::isfinite(max_curvature) || max_curvature <= 0.0 ||
        !std::isfinite(max_heading_step) || max_heading_step <= 0.0)
        return false;
    double previous_heading = 0.0, previous_length = 0.0;
    bool have_heading = false;
    for (size_t i = 0; i < path.poses.size(); ++i)
    {
        const auto& p = path.poses[i].pose.position;
        if (!std::isfinite(p.x) || !std::isfinite(p.y))
            return false;
        if (i == 0)
            continue;
        const auto& a = path.poses[i-1].pose.position;
        const double length = std::hypot(p.x-a.x, p.y-a.y);
        if (length < 1e-8)
            continue;
        const double heading = std::atan2(p.y-a.y, p.x-a.x);
        if (have_heading)
        {
            const double turn = std::abs(std::remainder(heading-previous_heading, 2.0*M_PI));
            if (turn > max_heading_step || turn / ((length+previous_length)*0.5) > max_curvature)
                return false;
        }
        previous_heading = heading;
        previous_length = length;
        have_heading = true;
    }
    return true;
}

PlanResult PathPlanner::Plan(
    const Grid& map, const Grid& cost_map, const Pose& start_pose, const Pose& goal_pose,
    const PlanOptions& options, const std::vector<Pose>& via_points)
{
    PlanResult result;
    try
    {
        const GridGeometry geometry(map);
        if (options.smoothing_retry_limit < 1 || options.smoothing_retry_limit > 64 ||
            !std::isfinite(options.smoothing_time_budget_sec) || options.smoothing_time_budget_sec <= 0.0 ||
            !std::isfinite(options.max_path_curvature) || options.max_path_curvature <= 0.0 ||
            !std::isfinite(options.max_path_heading_step) || options.max_path_heading_step <= 0.0 ||
            !std::isfinite(options.escape_clear_radius) || options.escape_clear_radius < 0.0 ||
            !std::isfinite(options.online_behind_clearance) || options.online_behind_clearance < 0.0 ||
            !std::isfinite(options.smooth_alpha) || options.smooth_alpha < 0.0 || options.smooth_alpha > 1.0 ||
            !std::isfinite(options.smooth_beta) || options.smooth_beta < 0.0 || options.smooth_beta > 1.0 ||
            !std::isfinite(options.smooth_tolerance) || options.smooth_tolerance <= 0.0 ||
            !std::isfinite(options.unknown_cost) || options.unknown_cost < 0.0 ||
            !std::isfinite(options.goal_tolerance_m) || options.goal_tolerance_m < 0.0)
            return result;
        std::vector<Pose> stops = via_points;
        stops.push_back(goal_pose);
        int x, y;
        if (!geometry.worldToCell(start_pose.position.x, start_pose.position.y, x, y))
        {
            result.status = PlanStatus::OutsideMap;
            return result;
        }
        for (const auto& stop : stops)
        {
            if (!geometry.worldToCell(stop.position.x, stop.position.y, x, y))
            {
                result.status = PlanStatus::OutsideMap;
                return result;
            }
        }

        const Grid* working = &map;
        Grid modified;
        if (options.disable_rear_path_plan)
        {
            modified = map;
            for (int gy = 0; gy < geometry.height; ++gy)
                for (int gx = 0; gx < geometry.width; ++gx)
                    if (geometry.cellCenter(gx, gy).x < -options.online_behind_clearance)
                        modified.data[static_cast<size_t>(gy) * geometry.width + gx] = 100;
            working = &modified;
        }
        size_t failed_segment = 0;
        result = planSegments(*working, cost_map, start_pose, stops, options, 0.0, failed_segment);
        if (result.status == PlanStatus::Success || result.status == PlanStatus::InvalidInput)
            return result;

        if (options.goal_tolerance_m > 0.0 && failed_segment + 1 == stops.size() &&
            (result.status == PlanStatus::GoalBlocked || result.status == PlanStatus::NoPath))
        {
            result = planSegments(*working, cost_map, start_pose, stops, options,
                                  options.goal_tolerance_m, failed_segment);
        }
        return result;
    }
    catch (const std::invalid_argument&)
    {
        return result;
    }
}
