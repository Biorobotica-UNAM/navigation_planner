#include "path_planner/PathPlanner.h"
#include "path_planner/srv/plan_path.hpp"
#include "path_planner/srv/get_planning_snapshot.hpp"

#include <nav_msgs/srv/get_map.hpp>
#include <nav_msgs/srv/get_plan.hpp>
#include <pumas_interfaces/srv/get_plan_with_via.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

class PathPlannerNode : public rclcpp::Node
{
    using Grid = nav_msgs::msg::OccupancyGrid;
    using GetMap = nav_msgs::srv::GetMap;
    using GetPlan = nav_msgs::srv::GetPlan;
    using PlanPath = path_planner::srv::PlanPath;
    using GetPlanWithVia = pumas_interfaces::srv::GetPlanWithVia;
    using MapClient = rclcpp::Client<GetMap>;
    using GetSnapshot = path_planner::srv::GetPlanningSnapshot;
    using SteadyClock = std::chrono::steady_clock;

    struct MapPair
    {
        Grid map;
        Grid costs;
        Grid clearance;
        geometry_msgs::msg::PoseArray uncertain_obstacles;
        uint64_t revision;
        SteadyClock::time_point received;
    };
    using Snapshot = std::shared_ptr<const MapPair>;

    struct UpdateCycle
    {
        MapClient::SharedPtr map_client, cost_client, active_client;
        Grid staged_map;
        bool pending = false, atomic_request = false;
        uint64_t generation = 0;
        int64_t request_id = 0;
        int retries = 0;
        SteadyClock::time_point deadline;
    };

public:
    PathPlannerNode() : Node("path_planner_node")
    {
        rcl_interfaces::msg::ParameterDescriptor startup;
        startup.read_only = true;
        startup.description = "Configured at startup.";
        const bool use_namespace = declare_parameter<bool>("use_namespace", false, startup);
        const double refresh_period = declare_parameter<double>("map_refresh_period_sec", 0.2, startup);
        snapshot_timeout_ = declare_parameter<double>("map_snapshot_timeout_sec", 1.0, startup);
        request_timeout_ = declare_parameter<double>("map_request_timeout_sec", 5.0, startup);
        retry_limit_ = declare_parameter<int>("map_pair_retry_limit", 1, startup);

        declare_parameter<bool>("diagonal_paths", false);
        declare_parameter<bool>("allow_unknown", false);
        declare_parameter<double>("unknown_cost", 1.0);
        declare_parameter<double>("smooth_alpha", 0.1);
        declare_parameter<double>("smooth_beta", 0.9);
        declare_parameter<double>("smooth_tolerance", 0.00001);
        declare_parameter<bool>("require_smooth_path", true);
        declare_parameter<int>("smoothing_retry_limit", 8);
        declare_parameter<double>("smoothing_time_budget_sec", 1.0);
        declare_parameter<double>("max_path_curvature", 6.0);
        declare_parameter<double>("max_path_heading_step", 0.50);
        declare_parameter<double>("recovery_cost_radius", 0.35);
        declare_parameter<double>("recovery_cost_peak", 40.0);
        declare_parameter<double>("escape_clear_radius", 0.6);
        declare_parameter<double>("robot_radius", 0.23);
        declare_parameter<double>("start_recovery_margin", 0.05);
        if (!std::isfinite(get_parameter("robot_radius").as_double()) ||
            get_parameter("robot_radius").as_double() <= 0.0 ||
            !std::isfinite(get_parameter("start_recovery_margin").as_double()) ||
            get_parameter("start_recovery_margin").as_double() < 0.0)
            throw std::invalid_argument("Invalid recovery footprint limits");
        declare_parameter<double>("goal_relocation_radius", 1.0);
        declare_parameter<bool>("disable_rear_path_plan", false);
        declare_parameter<double>("online_behind_clearance", 1.0);

        if (!std::isfinite(refresh_period) || refresh_period <= 0.0 ||
            !std::isfinite(request_timeout_) || request_timeout_ <= 0.0 ||
            !std::isfinite(snapshot_timeout_) || snapshot_timeout_ <= 0.0 ||
            retry_limit_ < 0 || retry_limit_ > 10)
            throw std::invalid_argument("Invalid map refresh, timeout, or retry configuration");

        // Validate overrides before advertising services.
        const auto initial = readOptions();
        if (initial.smoothing_retry_limit < 1 || initial.smoothing_retry_limit > 64 ||
            !std::isfinite(initial.smoothing_time_budget_sec) || initial.smoothing_time_budget_sec <= 0.0 ||
            !std::isfinite(initial.max_path_curvature) || initial.max_path_curvature <= 0.0 ||
            !std::isfinite(initial.max_path_heading_step) || initial.max_path_heading_step <= 0.0 ||
            !std::isfinite(initial.unknown_cost) || initial.unknown_cost < 0.0 ||
            !std::isfinite(initial.smooth_alpha) || initial.smooth_alpha < 0.0 || initial.smooth_alpha > 1.0 ||
            !std::isfinite(initial.smooth_beta) || initial.smooth_beta < 0.0 || initial.smooth_beta > 1.0 ||
            !std::isfinite(initial.smooth_tolerance) || initial.smooth_tolerance < 1e-12 ||
            !std::isfinite(initial.escape_clear_radius) || initial.escape_clear_radius < 0.0 ||
            !std::isfinite(initial.start_recovery_mask_radius) || initial.start_recovery_mask_radius <= 0.0 ||
            !std::isfinite(initial.goal_tolerance_m) || initial.goal_tolerance_m < 0.0 ||
            !std::isfinite(initial.online_behind_clearance) || initial.online_behind_clearance < 0.0 ||
            !std::isfinite(initial.recovery_cost_radius) || initial.recovery_cost_radius <= 0.0 ||
            !std::isfinite(initial.recovery_cost_peak) || initial.recovery_cost_peak <= 0.0 ||
            initial.recovery_cost_peak > 99.0)
            throw std::invalid_argument("Invalid planner options");

        parameter_callback_ = add_on_set_parameters_callback(
            std::bind(&PathPlannerNode::validateParameters, this, std::placeholders::_1));
        update_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        // Planning and parameter services stay in the default mutually-exclusive group.
        std::string prefix = use_namespace ? get_namespace() : "";
        if (prefix == "/")
            prefix.clear();
        const std::string map_prefix = prefix + "/map_augmenter/";
        const std::string plan_prefix = prefix + "/path_planner/";
        clearance_sub_ = create_subscription<Grid>(prefix + "/clearance_map",
            rclcpp::QoS(1).transient_local(),
            [this](Grid::ConstSharedPtr grid) {clearance_map_ = grid;});
        // ROS 2 Humble service clients take an rmw QoS profile.
        cycles_[0].map_client = create_client<GetMap>(
            map_prefix + "get_static_map", rmw_qos_profile_services_default, update_group_);
        cycles_[0].cost_client = create_client<GetMap>(
            map_prefix + "get_static_cost_map", rmw_qos_profile_services_default, update_group_);
        cycles_[1].map_client = create_client<GetMap>(
            map_prefix + "get_augmented_map", rmw_qos_profile_services_default, update_group_);
        cycles_[1].cost_client = create_client<GetMap>(
            map_prefix + "get_augmented_cost_map", rmw_qos_profile_services_default, update_group_);

        snapshot_client_ = create_client<GetSnapshot>(map_prefix + "get_planning_snapshot",
            rmw_qos_profile_services_default, update_group_);

        static_service_ = create_service<GetPlan>(
            plan_prefix + "plan_path_with_static",
            std::bind(&PathPlannerNode::planRequest, this, 0,
                      std::placeholders::_1, std::placeholders::_2));
        augmented_service_ = create_service<GetPlan>(
            plan_prefix + "plan_path_with_augmented",
            std::bind(&PathPlannerNode::planRequest, this, 1,
                      std::placeholders::_1, std::placeholders::_2));
        via_service_ = create_service<GetPlanWithVia>(
            plan_prefix + "plan_path_with_via",
            std::bind(&PathPlannerNode::viaRequest, this,
                      std::placeholders::_1, std::placeholders::_2));
        navigation_service_ = create_service<PlanPath>(
            plan_prefix + "plan_navigation",
            std::bind(&PathPlannerNode::navigationRequest, this,
                      std::placeholders::_1, std::placeholders::_2));

        refresh_timer_ = create_wall_timer(
            std::chrono::duration<double>(refresh_period),
            std::bind(&PathPlannerNode::refreshMaps, this), update_group_);
        watchdog_timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&PathPlannerNode::expireRequests, this), update_group_);
        RCLCPP_INFO(get_logger(), "Path planner ready; waiting for complete map snapshots.");
    }

private:
    PlanOptions readOptions() const
    {
        PlanOptions options;
        options.diagonal_paths = get_parameter("diagonal_paths").as_bool();
        options.allow_unknown = get_parameter("allow_unknown").as_bool();
        options.unknown_cost = get_parameter("unknown_cost").as_double();
        options.smooth_alpha = get_parameter("smooth_alpha").as_double();
        options.smooth_beta = get_parameter("smooth_beta").as_double();
        options.smooth_tolerance = get_parameter("smooth_tolerance").as_double();
        options.require_smooth_path = get_parameter("require_smooth_path").as_bool();
        options.smoothing_retry_limit = get_parameter("smoothing_retry_limit").as_int();
        options.smoothing_time_budget_sec = get_parameter("smoothing_time_budget_sec").as_double();
        options.max_path_curvature = get_parameter("max_path_curvature").as_double();
        options.max_path_heading_step = get_parameter("max_path_heading_step").as_double();
        options.recovery_cost_radius = get_parameter("recovery_cost_radius").as_double();
        options.recovery_cost_peak = get_parameter("recovery_cost_peak").as_double();
        options.escape_clear_radius = get_parameter("escape_clear_radius").as_double();
        options.start_recovery_mask_radius = get_parameter("robot_radius").as_double() +
            get_parameter("start_recovery_margin").as_double();
        options.goal_tolerance_m = get_parameter("goal_relocation_radius").as_double();
        options.disable_rear_path_plan = get_parameter("disable_rear_path_plan").as_bool();
        options.online_behind_clearance = get_parameter("online_behind_clearance").as_double();
        return options;
    }

    rcl_interfaces::msg::SetParametersResult validateParameters(
        const std::vector<rclcpp::Parameter>& parameters)
    {
        auto candidate = readOptions();
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = false;
        try
        {
            for (const auto& parameter : parameters)
            {
                const auto& name = parameter.get_name();
                if ((name == "robot_radius" && (!std::isfinite(parameter.as_double()) || parameter.as_double() <= 0.0)) ||
                    (name == "start_recovery_margin" && (!std::isfinite(parameter.as_double()) || parameter.as_double() < 0.0)))
                    throw std::invalid_argument("Invalid recovery footprint limits");
                if (name == "diagonal_paths") candidate.diagonal_paths = parameter.as_bool();
                else if (name == "allow_unknown") candidate.allow_unknown = parameter.as_bool();
                else if (name == "unknown_cost") candidate.unknown_cost = parameter.as_double();
                else if (name == "smooth_alpha") candidate.smooth_alpha = parameter.as_double();
                else if (name == "smooth_beta") candidate.smooth_beta = parameter.as_double();
                else if (name == "smooth_tolerance") candidate.smooth_tolerance = parameter.as_double();
                else if (name == "require_smooth_path") candidate.require_smooth_path = parameter.as_bool();
                else if (name == "smoothing_retry_limit") candidate.smoothing_retry_limit = parameter.as_int();
                else if (name == "smoothing_time_budget_sec") candidate.smoothing_time_budget_sec = parameter.as_double();
                else if (name == "max_path_curvature") candidate.max_path_curvature = parameter.as_double();
                else if (name == "max_path_heading_step") candidate.max_path_heading_step = parameter.as_double();
                else if (name == "recovery_cost_radius") candidate.recovery_cost_radius = parameter.as_double();
                else if (name == "recovery_cost_peak") candidate.recovery_cost_peak = parameter.as_double();
                else if (name == "escape_clear_radius") candidate.escape_clear_radius = parameter.as_double();
                else if (name == "robot_radius") candidate.start_recovery_mask_radius = parameter.as_double() + get_parameter("start_recovery_margin").as_double();
                else if (name == "start_recovery_margin") candidate.start_recovery_mask_radius = get_parameter("robot_radius").as_double() + parameter.as_double();
                else if (name == "goal_relocation_radius") candidate.goal_tolerance_m = parameter.as_double();
                else if (name == "disable_rear_path_plan") candidate.disable_rear_path_plan = parameter.as_bool();
                else if (name == "online_behind_clearance") candidate.online_behind_clearance = parameter.as_double();
            }
            if (candidate.smoothing_retry_limit < 1 || candidate.smoothing_retry_limit > 64 ||
            !std::isfinite(candidate.smoothing_time_budget_sec) || candidate.smoothing_time_budget_sec <= 0.0 ||
            !std::isfinite(candidate.max_path_curvature) || candidate.max_path_curvature <= 0.0 ||
            !std::isfinite(candidate.max_path_heading_step) || candidate.max_path_heading_step <= 0.0 ||
            !std::isfinite(candidate.unknown_cost) || candidate.unknown_cost < 0.0 ||
                !std::isfinite(candidate.smooth_alpha) || candidate.smooth_alpha < 0.0 || candidate.smooth_alpha > 1.0 ||
                !std::isfinite(candidate.smooth_beta) || candidate.smooth_beta < 0.0 || candidate.smooth_beta > 1.0 ||
                !std::isfinite(candidate.smooth_tolerance) || candidate.smooth_tolerance < 1e-12 ||
                !std::isfinite(candidate.escape_clear_radius) || candidate.escape_clear_radius < 0.0 ||
                !std::isfinite(candidate.start_recovery_mask_radius) || candidate.start_recovery_mask_radius <= 0.0 ||
                !std::isfinite(candidate.goal_tolerance_m) || candidate.goal_tolerance_m < 0.0 ||
                !std::isfinite(candidate.online_behind_clearance) || candidate.online_behind_clearance < 0.0 ||
                !std::isfinite(candidate.recovery_cost_radius) || candidate.recovery_cost_radius <= 0.0 ||
                !std::isfinite(candidate.recovery_cost_peak) || candidate.recovery_cost_peak <= 0.0 ||
                candidate.recovery_cost_peak > 99.0)
                result.reason = "Planner costs, radii, and smoothing values are out of range.";
            else
                result.successful = true;
        }
        catch (const std::exception& error)
        {
            result.reason = error.what();
        }
        // Do not mutate cached options before the parameter transaction commits.
        return result;
    }

    void refreshMaps()
    {
        for (size_t i = 0; i < cycles_.size(); ++i)
            if (!cycles_[i].pending)
                startUpdate(i, false);
    }

    void startUpdate(size_t index, bool retry)
    {
        auto& cycle = cycles_[index];
        if (cycle.pending)
            return;
        if (snapshot_client_->service_is_ready())
        {
            cycle.pending = cycle.atomic_request = true;
            const auto generation = ++cycle.generation;
            cycle.deadline = SteadyClock::now() + std::chrono::duration_cast<SteadyClock::duration>(
                std::chrono::duration<double>(request_timeout_));
            auto request = std::make_shared<GetSnapshot::Request>();
            request->augmented = index == 1;
            const auto pending = snapshot_client_->async_send_request(request,
                [this,index,generation](rclcpp::Client<GetSnapshot>::SharedFuture future) {
                    auto& state = cycles_[index];
                    if (!state.pending || state.generation != generation) return;
                    state.pending = false;
                    try
                    {
                        auto response = future.get();
                        if (!response->ready || response->clearance.data.empty()) return;
                        storeSnapshot(index, std::move(response->map), std::move(response->costs),
                            std::move(response->clearance), response->revision,
                            std::move(response->uncertain_obstacles));
                    }
                    catch (const std::exception& error) {RCLCPP_WARN(get_logger(), "Snapshot response failed: %s", error.what());}
                });
            cycle.request_id = pending.request_id;
            return;
        }
        cycle.atomic_request = false;
        if (!cycle.map_client->service_is_ready() || !cycle.cost_client->service_is_ready())
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                "Waiting for map services for snapshot %zu.", index);
            return;
        }
        if (!retry)
            cycle.retries = 0;
        cycle.pending = true;
        const auto generation = ++cycle.generation;
        cycle.active_client = cycle.map_client;
        cycle.deadline = SteadyClock::now() +
            std::chrono::duration_cast<SteadyClock::duration>(std::chrono::duration<double>(request_timeout_));
        try
        {
            MapClient::CallbackType callback = std::bind(
                &PathPlannerNode::receiveMap, this, index, generation, std::placeholders::_1);
            const auto pending = cycle.map_client->async_send_request(
                std::make_shared<GetMap::Request>(), std::move(callback));
            cycle.request_id = pending.request_id;
        }
        catch (const std::exception& error)
        {
            cycle.pending = false;
            RCLCPP_WARN(get_logger(), "Map request failed: %s", error.what());
        }
    }

    void receiveMap(size_t index, uint64_t generation, MapClient::SharedFuture ready)
    {
        auto& cycle = cycles_[index];
        if (!cycle.pending || cycle.generation != generation)
            return;
        try
        {
            cycle.staged_map = std::move(ready.get()->map);
            if (cycle.staged_map.data.empty())
            {
                cycle.pending = false;
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                    "Map snapshot %zu is not ready.", index);
                return;
            }
            cycle.active_client = cycle.cost_client;
            cycle.deadline = SteadyClock::now() +
                std::chrono::duration_cast<SteadyClock::duration>(std::chrono::duration<double>(request_timeout_));
            MapClient::CallbackType callback = std::bind(
                &PathPlannerNode::receiveCosts, this, index, generation, std::placeholders::_1);
            const auto pending = cycle.cost_client->async_send_request(
                std::make_shared<GetMap::Request>(), std::move(callback));
            cycle.request_id = pending.request_id;
        }
        catch (const std::exception& error)
        {
            cycle.pending = false;
            cycle.staged_map = Grid{};
            RCLCPP_WARN(get_logger(), "Map response failed: %s", error.what());
        }
    }

    void receiveCosts(size_t index, uint64_t generation, MapClient::SharedFuture ready)
    {
        auto& cycle = cycles_[index];
        if (!cycle.pending || cycle.generation != generation)
            return;
        cycle.pending = false;
        try
        {
            Grid costs = std::move(ready.get()->map);
            if (!storeSnapshot(index,std::move(cycle.staged_map),std::move(costs),Grid{},0) && cycle.retries < retry_limit_)
            {
                ++cycle.retries;
                startUpdate(index,true);
            }
        }
        catch (const std::exception& error)
        {
            cycle.staged_map = Grid{};
            RCLCPP_WARN(get_logger(), "Cost-map response failed: %s", error.what());
        }
    }

    bool storeSnapshot(size_t index, Grid map, Grid costs, Grid clearance, uint64_t revision,
        geometry_msgs::msg::PoseArray uncertain = geometry_msgs::msg::PoseArray())
    {
        const auto coherent = [&map](const Grid& layer) {
            return layer.header == map.header && layer.info == map.info && layer.data.size() == map.data.size();
        };
        if (map.header.frame_id.empty() || map.info.width == 0 || map.info.height == 0 ||
            !std::isfinite(map.info.resolution) || map.info.resolution <= 0.0 ||
            static_cast<uint64_t>(map.info.width)*map.info.height != map.data.size() ||
            !coherent(costs) || (!clearance.data.empty() && !coherent(clearance)))
        {
            RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),2000,
                "Map/cost snapshot mismatch; retaining the previous snapshot %zu.",index);
            return false;
        }
        if (!uncertain.header.frame_id.empty() && uncertain.header.frame_id != map.header.frame_id) return false;
        for (const auto& pose : uncertain.poses)
            if (!std::isfinite(pose.position.x) || !std::isfinite(pose.position.y)) return false;
        auto next = std::make_shared<const MapPair>(MapPair{
            std::move(map),std::move(costs),std::move(clearance),std::move(uncertain),revision,SteadyClock::now()});
        std::lock_guard<std::mutex> lock(snapshots_mutex_);
        snapshots_[index].swap(next);
        return true;
    }

    void expireRequests()
    {
        if (refresh_requested_.exchange(false))
            refreshMaps();
        const auto current = SteadyClock::now();
        for (auto& cycle : cycles_)
        {
            if (cycle.pending && current >= cycle.deadline)
            {
                if (cycle.atomic_request) snapshot_client_->remove_pending_request(cycle.request_id);
                else cycle.active_client->remove_pending_request(cycle.request_id);
                ++cycle.generation;
                cycle.pending = false;
                cycle.staged_map = Grid{};
                RCLCPP_WARN(get_logger(), "Map request timed out; retaining the previous snapshot.");
            }
        }
    }

    Snapshot planningSnapshot(size_t index, const builtin_interfaces::msg::Time& minimum,
                              std::string& reason)
    {
        Snapshot snapshot;
        {
            std::lock_guard<std::mutex> lock(snapshots_mutex_);
            snapshot = snapshots_[index];
        }
        reason = "waiting_for_updated_map";
        if (snapshot)
        {
            const auto stamp = rclcpp::Time(snapshot->map.header.stamp, get_clock()->get_clock_type());
            const auto required = rclcpp::Time(minimum, get_clock()->get_clock_type());
            if (stamp.nanoseconds() != 0 && stamp >= required &&
                (now() - stamp).seconds() >= -0.05 &&
                (now() - stamp).seconds() <= snapshot_timeout_ &&
                std::chrono::duration<double>(SteadyClock::now() - snapshot->received).count() <= snapshot_timeout_)
            {
                reason.clear();
                return snapshot;
            }
        }
        // Update-cycle state belongs to its callback group; request refresh without racing it.
        refresh_requested_ = true;
        return {};
    }

    void navigationRequest(PlanPath::Request::SharedPtr request, PlanPath::Response::SharedPtr response)
    {
        auto snapshot = planningSnapshot(1, request->minimum_map_stamp, response->reason);
        if (!snapshot)
            return;
        response->map_stamp = snapshot->map.header.stamp;
        const auto& frame = snapshot->map.header.frame_id;
        if (request->start.header.frame_id != frame || request->goal.header.frame_id != frame ||
            request->avoided_paths.size() > 4 || request->required_obstacles.size() > 16)
        {
            response->reason = "invalid_planning_request";
            return;
        }
        const auto& grid = snapshot->map;
        const double yaw = std::atan2(2.0 * grid.info.origin.orientation.w *
            grid.info.origin.orientation.z, 1.0 - 2.0 * grid.info.origin.orientation.z *
            grid.info.origin.orientation.z);
        for (const auto& witness : request->required_obstacles)
        {
            const double dx = witness.point.x - grid.info.origin.position.x;
            const double dy = witness.point.y - grid.info.origin.position.y;
            const double x = (std::cos(yaw) * dx + std::sin(yaw) * dy) / grid.info.resolution;
            const double y = (-std::sin(yaw) * dx + std::cos(yaw) * dy) / grid.info.resolution;
            if (witness.header.frame_id != frame || !std::isfinite(x) || !std::isfinite(y) ||
                x < 0 || y < 0 || x >= grid.info.width || y >= grid.info.height ||
                grid.data[static_cast<size_t>(y) * grid.info.width + static_cast<size_t>(x)] <= 0)
            {
                response->reason = "waiting_for_obstacle_map";
                refresh_requested_ = true;
                return;
            }
        }
        auto options = readOptions();
        if (!std::isfinite(request->goal_recovery_radius) || request->goal_recovery_radius < 0.0)
        {
            response->reason = "invalid_planning_request";
            return;
        }
        options.goal_tolerance_m = 0.0;
        const Grid* clearance = snapshot->clearance.data.empty() ? nullptr : &snapshot->clearance;
        if (!clearance && clearance_map_ && clearance_map_->header.frame_id == frame &&
            clearance_map_->info == snapshot->map.info &&
            clearance_map_->data.size() == snapshot->map.data.size())
        {
            const auto stamp = rclcpp::Time(clearance_map_->header.stamp, get_clock()->get_clock_type());
            if (stamp >= rclcpp::Time(snapshot->map.header.stamp, get_clock()->get_clock_type()) &&
                (now() - stamp).seconds() >= -0.05 && (now() - stamp).seconds() <= snapshot_timeout_)
                clearance = clearance_map_.get();
        }
        const auto cell = [&grid](const geometry_msgs::msg::Point& p) {
            const double yaw = 2.0 * std::atan2(grid.info.origin.orientation.z, grid.info.origin.orientation.w);
            const double dx = p.x - grid.info.origin.position.x, dy = p.y - grid.info.origin.position.y;
            const double x = (std::cos(yaw) * dx + std::sin(yaw) * dy) / grid.info.resolution;
            const double y = (-std::sin(yaw) * dx + std::cos(yaw) * dy) / grid.info.resolution;
            if (!std::isfinite(x) || !std::isfinite(y) || x < 0 || y < 0 ||
                x >= grid.info.width || y >= grid.info.height) return -1;
            return int(grid.data[static_cast<size_t>(y) * grid.info.width + static_cast<size_t>(x)]);
        };
        path_planner::MotionConstraints constraints;
        constraints.update(clearance ? *clearance : grid, snapshot->uncertain_obstacles,
            get_parameter("robot_radius").as_double() + path_planner::uncertainty_margin_m,
            options.allow_unknown);
        options.motion_constraints = &constraints;
        const auto& start = request->start.pose.position;
        const auto& goal = request->goal.pose.position;
        const bool recover_start = clearance && (cell(start) > 0 ||
            !constraints.uncertainty_free(start.x, start.y, start.x, start.y, false));
        if (!recover_start) {
            constraints.update(grid, snapshot->uncertain_obstacles,
                get_parameter("robot_radius").as_double() + path_planner::uncertainty_margin_m,
                options.allow_unknown);
        }
        const bool goal_blocked = cell(goal) > 0 ||
            !constraints.uncertainty_free(goal.x, goal.y, goal.x, goal.y, false) ||
            !constraints.sampling_free(goal.x, goal.y, goal.x, goal.y, false);
        // Relocation also applies to uncertainty and interpolation guards.
        if (goal_blocked && request->goal_recovery_radius > 0.0)
            options.goal_tolerance_m = std::nextafter(request->goal_recovery_radius, 0.0);
        auto result = recover_start ? PathPlanner::RecoverStart(grid, *clearance, request->start.pose, options) :
            PathPlanner::Plan(grid, snapshot->costs, request->start.pose, request->goal.pose,
                options, {}, request->avoided_paths);
        if (result.status == PlanStatus::StartBlocked && !clearance)
        {
            response->reason = "waiting_for_clearance_map";
            refresh_requested_ = true;
            return;
        }
        if (result.status != PlanStatus::Success)
        {
            response->reason = result.status == PlanStatus::NoSmoothPath ? "no_smooth_path" :
                result.status == PlanStatus::StartBlocked ? "start_blocked" :
                result.status == PlanStatus::GoalBlocked ||
                (result.status == PlanStatus::NoPath && goal_blocked) ? "goal_blocked" : "no_path";
            RCLCPP_WARN(get_logger(), "Navigation planning failed: %s (status=%d, detail=%s)",
                        response->reason.c_str(), static_cast<int>(result.status), result.detail.c_str());
            return;
        }
        response->path = std::move(result.path);
        response->start_recovery = recover_start;
        response->goal_relocated = result.goal_relocated;
        response->path.header.stamp = response->map_stamp;
        for (auto& pose : response->path.poses)
            pose.header = response->path.header;
    }

    void planRequest(size_t index, GetPlan::Request::SharedPtr request, GetPlan::Response::SharedPtr response)
    {
        std::string reason;
        const auto snapshot = planningSnapshot(index, builtin_interfaces::msg::Time(), reason);
        if (!snapshot)
            return;
        const auto& frame = snapshot->map.header.frame_id;
        // Empty request frames preserve compatibility with existing clients.
        if ((!request->start.header.frame_id.empty() && request->start.header.frame_id != frame) ||
            (!request->goal.header.frame_id.empty() && request->goal.header.frame_id != frame))
        {
            RCLCPP_WARN(get_logger(), "Planning request frame does not match '%s'.", frame.c_str());
            return;
        }
        auto options = readOptions();
        options.goal_tolerance_m = request->tolerance;
        path_planner::MotionConstraints constraints;
        constraints.update(snapshot->map,
            snapshot->uncertain_obstacles,
            get_parameter("robot_radius").as_double() + path_planner::uncertainty_margin_m,
            options.allow_unknown);
        options.motion_constraints = &constraints;
        auto result = PathPlanner::Plan(snapshot->map, snapshot->costs,
                                        request->start.pose, request->goal.pose, options);
        if (result.status != PlanStatus::Success)
        {
            RCLCPP_WARN(get_logger(), "Planning failed with status %d%s.", static_cast<int>(result.status),
                        result.status == PlanStatus::NoSmoothPath ? ": no collision-free path meets smoothness limits" : "");
            return;
        }
        response->plan = std::move(result.path);
        response->plan.header.stamp = now();
        for (auto& pose : response->plan.poses)
            pose.header = response->plan.header;
        if (result.used_escape || result.goal_relocated)
            RCLCPP_WARN(get_logger(), "Planning recovery used: start_clear=%s, goal_relocated=%s.",
                        result.used_escape ? "true" : "false", result.goal_relocated ? "true" : "false");
    }

    void viaRequest(GetPlanWithVia::Request::SharedPtr request, GetPlanWithVia::Response::SharedPtr response)
    {
        std::string reason;
        const auto snapshot = planningSnapshot(1, builtin_interfaces::msg::Time(), reason);
        if (!snapshot)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "No complete augmented snapshot is available.");
            return;
        }
        const auto& frame = snapshot->map.header.frame_id;
        if ((!request->start.header.frame_id.empty() && request->start.header.frame_id != frame) ||
            (!request->goal.header.frame_id.empty() && request->goal.header.frame_id != frame))
        {
            RCLCPP_WARN(get_logger(), "Waypoint request frame does not match '%s'.", frame.c_str());
            return;
        }
        std::vector<geometry_msgs::msg::Pose> via_points;
        via_points.reserve(request->via_points.size());
        for (const auto& waypoint : request->via_points)
        {
            if (!waypoint.header.frame_id.empty() && waypoint.header.frame_id != frame)
            {
                RCLCPP_WARN(get_logger(), "Waypoint frame does not match '%s'.", frame.c_str());
                return;
            }
            via_points.push_back(waypoint.pose);
        }
        auto options = readOptions();
        path_planner::MotionConstraints constraints;
        constraints.update(snapshot->map,
            snapshot->uncertain_obstacles,
            get_parameter("robot_radius").as_double() + path_planner::uncertainty_margin_m,
            options.allow_unknown);
        options.motion_constraints = &constraints;
        auto result = PathPlanner::Plan(snapshot->map, snapshot->costs,
            request->start.pose, request->goal.pose, options, via_points);
        if (result.status != PlanStatus::Success)
        {
            RCLCPP_WARN(get_logger(), "Waypoint planning failed with status %d.", static_cast<int>(result.status));
            return;
        }
        response->plan = std::move(result.path);
        response->plan.header.stamp = now();
        for (auto& pose : response->plan.poses)
            pose.header = response->plan.header;
        if (result.used_escape || result.goal_relocated)
            RCLCPP_WARN(get_logger(), "Waypoint recovery used: start_clear=%s, goal_relocated=%s.",
                        result.used_escape ? "true" : "false", result.goal_relocated ? "true" : "false");
    }

    rclcpp::Client<GetSnapshot>::SharedPtr snapshot_client_;
    std::array<UpdateCycle, 2> cycles_;
    std::array<Snapshot, 2> snapshots_;
    std::mutex snapshots_mutex_;
    double request_timeout_, snapshot_timeout_;
    std::atomic<bool> refresh_requested_{false};
    int retry_limit_;
    rclcpp::CallbackGroup::SharedPtr update_group_;
    rclcpp::TimerBase::SharedPtr refresh_timer_, watchdog_timer_;
    OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
    rclcpp::Service<GetPlan>::SharedPtr static_service_, augmented_service_;
    rclcpp::Service<GetPlanWithVia>::SharedPtr via_service_;
    rclcpp::Service<PlanPath>::SharedPtr navigation_service_;
    Grid::ConstSharedPtr clearance_map_;
    rclcpp::Subscription<Grid>::SharedPtr clearance_sub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    int result = 0;
    try
    {
        auto node = std::make_shared<PathPlannerNode>();
        rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
        executor.add_node(node);
        executor.spin();
    }
    catch (const std::exception& error)
    {
        RCLCPP_FATAL(rclcpp::get_logger("path_planner_node"), "Planner stopped: %s", error.what());
        result = 1;
    }
    rclcpp::shutdown();
    return result;
}
