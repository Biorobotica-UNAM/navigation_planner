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

// Integrate travel distance and soft costs, independent of waypoint density.
double pathCost(const Path& path, const Grid& costs, int* peak = nullptr)
{
    const GridGeometry geometry(costs);
    double total = 0.0;
    if (peak) *peak = 0;
    for (size_t i = 1; i < path.poses.size(); ++i)
    {
        const auto& a = path.poses[i - 1].pose.position;
        const auto& b = path.poses[i].pose.position;
        const double length = std::hypot(b.x - a.x, b.y - a.y);
        const int samples = std::max(1, static_cast<int>(std::ceil(length / (0.5 * geometry.resolution))));
        for (int j = 0; j < samples; ++j)
        {
            const double t = (j + 0.5) / samples;
            int x, y;
            if (!geometry.worldToCell(a.x + t * (b.x - a.x), a.y + t * (b.y - a.y), x, y))
                return std::numeric_limits<double>::infinity();
            const int cost = std::max(0, static_cast<int>(
                costs.data[static_cast<size_t>(y) * geometry.width + x]));
            if (peak) *peak = std::max(*peak, cost);
            total += length / samples * (1.0 + cost);
        }
    }
    return total;
}

bool validatePathSegments(const Path& path, const Grid& map, bool allow_unknown,
    const path_planner::MotionConstraints* constraints = nullptr)
{
    if (path.poses.empty() || path.header.frame_id != map.header.frame_id)
        return false;
    const GridGeometry geometry(map);
    for (size_t i = 0; i < path.poses.size(); ++i)
    {
        const auto acceptable = [&](size_t index) {
            const int value = map.data[index];
            return value == 0 || (value == -1 && allow_unknown);
        };
        const auto& point = path.poses[i].pose.position;
        if (constraints) {
            const auto& from = path.poses[i == 0 ? 0 : i - 1].pose.position;
            if (!constraints->uncertainty_free(from.x, from.y, point.x, point.y) ||
                !constraints->sampling_free(from.x, from.y, point.x, point.y)) return false;
        }
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
        const Grid* working_costs = &costs;
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
                // Prefer turning room through a soft gradient. Narrow corridors
                // remain available; the hard base footprint never grows on retries.
                const double radius = std::min(0.4, 1.0 / options.max_path_curvature + 0.10);
                const int reach = static_cast<int>(std::ceil(radius / geometry.resolution));
                for (int y = 0; y < geometry.height; ++y)
                    for (int x = 0; x < geometry.width; ++x)
                    {
                        if (map.data[static_cast<size_t>(y) * geometry.width + x] <= 0)
                            continue;
                        for (int yy = std::max(0, y-reach); yy <= std::min(geometry.height-1, y+reach); ++yy)
                            for (int xx = std::max(0, x-reach); xx <= std::min(geometry.width-1, x+reach); ++xx)
                            {
                                const size_t index = static_cast<size_t>(yy) * geometry.width + xx;
                                if (map.data[index] != 0) continue;
                                const double distance = std::hypot(xx-x, yy-y) * geometry.resolution;
                                const int cost = static_cast<int>(12.0 * std::max(0.0, 1.0 - distance / radius));
                                alternate_costs.data[index] = static_cast<int8_t>(std::max(cost,
                                    static_cast<int>(alternate_costs.data[index])));
                            }
                    }
                Path alternate;
                PlanStatus status;
                if (PathPlanner::AStar(map, alternate_costs, current_start, stops[i],
                                      options.diagonal_paths, alternate, segment_options, &status))
                {
                    segment = std::move(alternate);
                    working_costs = &alternate_costs;
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
                    Path reference;
                    reference.poses.assign(segment.poses.begin() + from, segment.poses.begin() + to + 1);
                    int link_peak = 0, reference_peak = 0;
                    const double link_cost = pathCost(link, *working_costs, &link_peak);
                    const double reference_cost = pathCost(reference, *working_costs, &reference_peak);
                    if (validatePathSegments(link, map, options.allow_unknown, options.motion_constraints) &&
                        link_cost <= 1.10 * reference_cost && link_peak <= reference_peak + 2)
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
            std::string rejection;
            smoothed = PathPlanner::SmoothPath(sampled, static_cast<float>(data_weight),
                static_cast<float>(options.smooth_beta), static_cast<float>(options.smooth_tolerance),
                &map, options.allow_unknown, working_costs, &rejection, options.motion_constraints);
            if (validatePathSegments(smoothed, map, options.allow_unknown, options.motion_constraints) &&
                (!options.require_smooth_path || PathPlanner::IsSmooth(smoothed,
                    options.max_path_curvature, options.max_path_heading_step)))
            {
                accepted = true;
                break;
            }
            result.detail = rejection.empty() ? "curvature_or_heading_limit" : rejection;
        }
        if (!accepted)
        {
            result.status = PlanStatus::NoSmoothPath;
            if (std::chrono::steady_clock::now() >= deadline)
                result.detail = "smoothing_time_budget: " + result.detail;
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
    result.detail.clear();
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
        const auto* constraints = options.motion_constraints;
        const bool goal_blocked = (goal_value != 0 && !(goal_value == -1 && options.allow_unknown)) ||
            (constraints && (!constraints->uncertainty_free(goal_pose.position.x, goal_pose.position.y,
                goal_pose.position.x, goal_pose.position.y, false) ||
                !constraints->sampling_free(goal_pose.position.x, goal_pose.position.y,
                goal_pose.position.x, goal_pose.position.y, false)));
        if (start_value != 0 && !(start_value == -1 && options.allow_unknown))
        {
            if (status)
                *status = PlanStatus::StartBlocked;
            return false;
        }
        if (goal_blocked && options.goal_tolerance_m == 0.0)
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
                if (start_distance <= distance && (!constraints ||
                    (constraints->uncertainty_free(start_pose.position.x, start_pose.position.y,
                        start_pose.position.x, start_pose.position.y, false) &&
                     constraints->sampling_free(start_pose.position.x, start_pose.position.y,
                        start_pose.position.x, start_pose.position.y, false))))
                {
                    distance = start_distance;
                    use_start_pose = true;
                }
            }
            const bool endpoint_safe = !constraints ||
                (constraints->uncertainty_free(point.x, point.y, point.x, point.y, false) &&
                 constraints->sampling_free(point.x, point.y, point.x, point.y, false));
            if (endpoint_safe && (distance < closest_distance ||
                (distance == closest_distance && nodes[index].g < nodes[closest].g)))
            {
                closest = index;
                closest_distance = distance;
                closest_is_start_pose = use_start_pose;
            }
            if (index == goal && !goal_blocked)
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
                    if (constraints) {
                        const auto& from = index == start ? start_pose.position : point;
                        const auto to = next == goal && !goal_blocked ? goal_pose.position : geometry.cellCenter(nx, ny);
                        if (!constraints->uncertainty_free(from.x, from.y, to.x, to.y) ||
                            !constraints->sampling_free(from.x, from.y, to.x, to.y)) continue;
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
        if (endpoint == goal && !goal_blocked)
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
        result_path.poses.back().pose.orientation = goal_pose.orientation;
        finishPath(result_path);
        if (constraints && !validatePathSegments(result_path, map, options.allow_unknown, constraints)) {
            result_path.poses.clear();
            if (status) *status = PlanStatus::NoPath;
            return false;
        }
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
    const Grid* collision_map, bool allow_unknown, const Grid* cost_map, std::string* rejection,
    const path_planner::MotionConstraints* constraints)
{
    Path result = path;
    if (rejection) rejection->clear();
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
                if (collision_map)
                {
                    Path local;
                    local.header = result.header;
                    for (int trial = 0; trial < 8; ++trial)
                    {
                        local.poses = {result.poses[i - 1], result.poses[i], result.poses[i + 1]};
                        if (validatePathSegments(local, *collision_map, allow_unknown, constraints))
                            break;
                        point.x = 0.5 * (point.x + old.x);
                        point.y = 0.5 * (point.y + old.y);
                        if (trial == 7) point = old;
                    }
                }
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
    if (collision_map && !validatePathSegments(result, *collision_map, allow_unknown, constraints))
    {
        if (rejection) *rejection = "smoothing_intersects_obstacle";
        result = path;
    }
    else if (cost_map)
    {
        int result_peak = 0, reference_peak = 0;
        const double result_cost = pathCost(result, *cost_map, &result_peak);
        const double reference_cost = pathCost(path, *cost_map, &reference_peak);
        if (result_cost > 1.30 * reference_cost || result_peak > reference_peak + 2)
        {
            if (rejection) *rejection = "smoothing_cost_budget";
            result = path;
        }
    }
    finishPath(result);
    return result;
}

bool PathPlanner::IsSmooth(const Path& path, double max_curvature, double max_heading_step)
{
    if (path.poses.empty() || !std::isfinite(max_curvature) || max_curvature <= 0.0 ||
        !std::isfinite(max_heading_step) || max_heading_step <= 0.0)
        return false;
    std::vector<double> lengths(path.poses.size(), 0.0);
    for (size_t i = 0; i < path.poses.size(); ++i)
    {
        const auto& p = path.poses[i].pose.position;
        if (!std::isfinite(p.x) || !std::isfinite(p.y))
            return false;
        if (i > 0)
        {
            const auto& a = path.poses[i - 1].pose.position;
            lengths[i] = lengths[i - 1] + std::hypot(p.x - a.x, p.y - a.y);
        }
    }
    if (lengths.back() < 1e-8)
        return true;
    // Equal metric intervals prevent tiny endpoint/grid segments from dominating
    // curvature. Retain a geometric residual check so stair steps cannot alias to a line.
    const size_t count = std::max(size_t{2}, static_cast<size_t>(std::ceil(lengths.back() / 0.10)));
    const double spacing = lengths.back() / count;
    std::vector<geometry_msgs::msg::Point> sampled;
    size_t segment = 1;
    for (size_t i = 0; i <= count; ++i)
    {
        const double target = i * spacing;
        while (segment + 1 < lengths.size() && lengths[segment] < target)
            ++segment;
        const auto& a = path.poses[segment - 1].pose.position;
        const auto& b = path.poses[segment].pose.position;
        const double t = std::clamp((target - lengths[segment - 1]) /
            std::max(1e-12, lengths[segment] - lengths[segment - 1]), 0.0, 1.0);
        auto point = a;
        point.x += t * (b.x - a.x); point.y += t * (b.y - a.y);
        sampled.push_back(point);
    }
    double previous_heading = 0.0, previous_length = 0.0;
    for (size_t i = 1; i < sampled.size(); ++i)
    {
        const double length = std::hypot(sampled[i].x - sampled[i-1].x, sampled[i].y - sampled[i-1].y);
        if (length < 1e-8)
            return false;
        const double heading = std::atan2(sampled[i].y - sampled[i-1].y, sampled[i].x - sampled[i-1].x);
        if (i > 1)
        {
            const double turn = std::abs(std::remainder(heading - previous_heading, 2.0 * M_PI));
            if (turn > max_heading_step || turn / ((length + previous_length) * 0.5) > max_curvature)
                return false;
        }
        previous_heading = heading; previous_length = length;
    }
    for (size_t i = 0; i < path.poses.size(); ++i)
    {
        const size_t bin = std::min(count - 1, static_cast<size_t>(lengths[i] / spacing));
        const auto& a = sampled[bin]; const auto& b = sampled[bin + 1];
        const auto& p = path.poses[i].pose.position;
        const double dx = b.x - a.x, dy = b.y - a.y;
        const double t = std::clamp(((p.x - a.x) * dx + (p.y - a.y) * dy) /
            std::max(1e-12, dx * dx + dy * dy), 0.0, 1.0);
        if (std::hypot(p.x - a.x - t * dx, p.y - a.y - t * dy) >
            std::max(0.01, max_curvature * spacing * spacing / 8.0))
            return false;
    }
    return true;
}

PlanResult PathPlanner::RecoverStart(
    const Grid& map, const Grid& clearance, const Pose& start_pose, const PlanOptions& options)
{
    PlanResult result;
    result.status = PlanStatus::StartBlocked;
    try
    {
        const GridGeometry geometry(map);
        if (clearance.info != map.info || clearance.header.frame_id != map.header.frame_id ||
            clearance.data.size() != map.data.size() ||
            !std::isfinite(options.start_recovery_mask_radius) || options.start_recovery_mask_radius <= 0.0 ||
            !std::isfinite(options.escape_clear_radius) || options.escape_clear_radius <= 0.0)
            return result;
        int sx, sy;
        if (!geometry.worldToCell(start_pose.position.x, start_pose.position.y, sx, sy))
            return result;
        if (clearance.data[static_cast<size_t>(sy) * geometry.width + sx] != 0)
        {
            result.detail = "start_inside_physical_clearance";
            return result;
        }
        if (map.data[static_cast<size_t>(sy) * geometry.width + sx] < 0) return result;
        // Allocate only the local mask. Neither input grid is copied or cleared.
        const double radius = std::min(options.escape_clear_radius, options.start_recovery_mask_radius);
        const int reach = static_cast<int>(std::ceil(radius / geometry.resolution)) + 1;
        const int xmin = std::max(0, sx - reach), ymin = std::max(0, sy - reach);
        const int width = std::min(geometry.width - 1, sx + reach) - xmin + 1;
        const int height = std::min(geometry.height - 1, sy + reach) - ymin + 1;
        const size_t start = static_cast<size_t>(sy - ymin) * width + sx - xmin;
        std::vector<double> distances(static_cast<size_t>(width) * height,
            std::numeric_limits<double>::infinity());
        std::vector<size_t> parents(distances.size(), no_parent);
        using Entry = std::pair<double, size_t>;
        std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;
        distances[start] = 0.0;open.emplace(0.0, start);
        size_t endpoint = no_parent;
        while (!open.empty())
        {
            const auto [distance, index] = open.top();open.pop();
            if (distance != distances[index]) continue;
            const int x = xmin + index % width, y = ymin + index / width;
            const auto from = index == start ? start_pose.position : geometry.cellCenter(x, y);
            if (map.data[static_cast<size_t>(y) * geometry.width + x] == 0 &&
                (!options.motion_constraints ||
                 (options.motion_constraints->uncertainty_free(from.x, from.y, from.x, from.y, false) &&
                  options.motion_constraints->sampling_free(from.x, from.y, from.x, from.y, false))))
            {
                endpoint = index;
                break;
            }
            for (int dy = -1; dy <= 1; ++dy)
                for (int dx = -1; dx <= 1; ++dx)
                {
                    if ((!dx && !dy) || (!options.diagonal_paths && dx && dy)) continue;
                    const int nx = x + dx, ny = y + dy;
                    if (nx < xmin || ny < ymin || nx >= xmin + width || ny >= ymin + height ||
                        clearance.data[static_cast<size_t>(ny) * geometry.width + nx] != 0 ||
                        map.data[static_cast<size_t>(ny) * geometry.width + nx] < 0) continue;
                    const auto p = geometry.cellCenter(nx, ny);
                    if (std::hypot(p.x - start_pose.position.x, p.y - start_pose.position.y) > radius) continue;
                    if (dx && dy && (clearance.data[static_cast<size_t>(y) * geometry.width + nx] != 0 ||
                                    clearance.data[static_cast<size_t>(ny) * geometry.width + x] != 0)) continue;
                    if (options.motion_constraints &&
                        (!options.motion_constraints->uncertainty_free(from.x, from.y, p.x, p.y) ||
                         !options.motion_constraints->sampling_free(from.x, from.y, p.x, p.y))) continue;
                    const double candidate = distance + (index == start ?
                        std::hypot(p.x-start_pose.position.x, p.y-start_pose.position.y) :
                        geometry.resolution * (dx && dy ? std::sqrt(2.0) : 1.0));
                    const size_t next = static_cast<size_t>(ny - ymin) * width + nx - xmin;
                    if (candidate <= options.escape_clear_radius && candidate < distances[next])
                    {
                        distances[next] = candidate;parents[next] = index;open.emplace(candidate, next);
                    }
                }
        }
        if (endpoint == no_parent)
        {
            result.detail = "no_safe_exit_in_local_mask";
            return result;
        }
        Path prefix;prefix.header = map.header;
        for (size_t index = endpoint; index != no_parent; index = parents[index])
        {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = map.header;pose.pose = start_pose;
            pose.pose.position = geometry.cellCenter(xmin + index % width, ymin + index / width);
            prefix.poses.push_back(pose);
        }
        std::reverse(prefix.poses.begin(), prefix.poses.end());
        prefix.poses.front().pose = start_pose;
        if (!validatePathSegments(prefix, clearance, false, options.motion_constraints)) return result;
        result.path.header = map.header;
        result.path.poses.push_back(prefix.poses.front());
        for (size_t from = 0; from + 1 < prefix.poses.size();)
        {
            size_t next = from + 1;
            for (size_t to = prefix.poses.size() - 1; to > from + 1; --to)
            {
                Path link;link.header = map.header;link.poses = {prefix.poses[from], prefix.poses[to]};
                if (validatePathSegments(link, clearance, false, options.motion_constraints)) {next = to;break;}
            }
            result.path.poses.push_back(prefix.poses[next]);from = next;
        }
        result.status = PlanStatus::Success;result.used_escape = true;
        result.detail = "escaping_planning_inflation";
        return result;
    }
    catch (const std::invalid_argument&) {return result;}
}

PlanResult PathPlanner::Plan(
    const Grid& map, const Grid& cost_map, const Pose& start_pose, const Pose& goal_pose,
    const PlanOptions& options, const std::vector<Pose>& via_points,
    const std::vector<Path>& avoided_paths, const Grid* start_clearance_map)
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
            !std::isfinite(options.goal_tolerance_m) || options.goal_tolerance_m < 0.0 ||
            !std::isfinite(options.recovery_cost_radius) || options.recovery_cost_radius <= 0.0 ||
            !std::isfinite(options.recovery_cost_peak) || options.recovery_cost_peak <= 0.0 ||
            options.recovery_cost_peak > 99.0)
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

        geometry.worldToCell(start_pose.position.x, start_pose.position.y, x, y);
        if (map.data[static_cast<size_t>(y) * geometry.width + x] != 0 && start_clearance_map)
        {
            auto recovery = RecoverStart(map, *start_clearance_map, start_pose, options);
            if (recovery.status != PlanStatus::Success) return recovery;
            auto continuation = Plan(map, cost_map, recovery.path.poses.back().pose,
                goal_pose, options, via_points, avoided_paths);
            if (continuation.status != PlanStatus::Success) return continuation;
            recovery.path.poses.pop_back();
            continuation.path.poses.insert(continuation.path.poses.begin(),
                recovery.path.poses.begin(), recovery.path.poses.end());
            continuation.used_escape = true;
            continuation.detail = recovery.detail;
            return continuation;
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
        Grid recovery_costs = cost_map;
        if (recovery_costs.data.size() != map.data.size())
            return result;
        const int reach = static_cast<int>(std::ceil(options.recovery_cost_radius / geometry.resolution));
        for (const auto& route : avoided_paths)
        {
            if (route.header.frame_id != map.header.frame_id)
                return result;
            for (const auto& pose : route.poses)
            {
                int gx, gy;
                if (!geometry.worldToCell(pose.pose.position.x, pose.pose.position.y, gx, gy))
                    continue;
                for (int y = std::max(0, gy - reach); y <= std::min(geometry.height - 1, gy + reach); ++y)
                    for (int x = std::max(0, gx - reach); x <= std::min(geometry.width - 1, gx + reach); ++x)
                    {
                        const auto center = geometry.cellCenter(x, y);
                        const double distance = std::hypot(center.x - pose.pose.position.x,
                                                          center.y - pose.pose.position.y);
                        // Leave room to depart the current pose; penalties only discourage routes.
                        const double departure = std::min(1.0, std::hypot(center.x - start_pose.position.x,
                            center.y - start_pose.position.y) / options.recovery_cost_radius);
                        const int penalty = static_cast<int>(options.recovery_cost_peak * departure *
                            std::max(0.0, 1.0 - distance / options.recovery_cost_radius));
                        auto& cost = recovery_costs.data[static_cast<size_t>(y) * geometry.width + x];
                        if (cost >= 0)
                            cost = static_cast<int8_t>(std::max(static_cast<int>(cost), penalty));
                    }
            }
        }
        size_t failed_segment = 0;
        result = planSegments(*working, recovery_costs, start_pose, stops, options, 0.0, failed_segment);
        if (result.status == PlanStatus::Success || result.status == PlanStatus::InvalidInput)
            return result;

        if (options.goal_tolerance_m > 0.0 && failed_segment + 1 == stops.size() &&
            (result.status == PlanStatus::GoalBlocked || result.status == PlanStatus::NoPath))
        {
            result = planSegments(*working, recovery_costs, start_pose, stops, options,
                                  options.goal_tolerance_m, failed_segment);
        }
        return result;
    }
    catch (const std::invalid_argument&)
    {
        return result;
    }
}
