#include "path_planner/PathPlanner.h"

#include <nav_msgs/srv/get_map.hpp>
#include <nav_msgs/srv/get_plan.hpp>
#include <pumas_interfaces/srv/get_plan_with_via.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>

#include <array>
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
    using GetPlanWithVia = pumas_interfaces::srv::GetPlanWithVia;
    using MapClient = rclcpp::Client<GetMap>;
    using SteadyClock = std::chrono::steady_clock;

    struct MapPair
    {
        Grid map;
        Grid costs;
    };
    using Snapshot = std::shared_ptr<const MapPair>;

    struct UpdateCycle
    {
        MapClient::SharedPtr map_client, cost_client, active_client;
        Grid staged_map;
        bool pending = false;
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
        const double refresh_period = declare_parameter<double>("map_refresh_period_sec", 1.0, startup);
        request_timeout_ = declare_parameter<double>("map_request_timeout_sec", 5.0, startup);
        retry_limit_ = declare_parameter<int>("map_pair_retry_limit", 1, startup);

        declare_parameter<bool>("diagonal_paths", false);
        declare_parameter<bool>("allow_unknown", false);
        declare_parameter<double>("unknown_cost", 1.0);
        declare_parameter<double>("smooth_alpha", 0.1);
        declare_parameter<double>("smooth_beta", 0.9);
        declare_parameter<double>("smooth_tolerance", 0.00001);
        declare_parameter<double>("escape_clear_radius", 0.6);
        declare_parameter<double>("goal_relocation_radius", 1.0);
        declare_parameter<bool>("disable_rear_path_plan", false);
        declare_parameter<double>("online_behind_clearance", 1.0);

        if (!std::isfinite(refresh_period) || refresh_period <= 0.0 ||
            !std::isfinite(request_timeout_) || request_timeout_ <= 0.0 ||
            retry_limit_ < 0 || retry_limit_ > 10)
            throw std::invalid_argument("Invalid map refresh, timeout, or retry configuration");

        // Validate overrides before advertising services.
        const auto initial = readOptions();
        if (!std::isfinite(initial.unknown_cost) || initial.unknown_cost < 0.0 ||
            !std::isfinite(initial.smooth_alpha) || initial.smooth_alpha < 0.0 || initial.smooth_alpha > 1.0 ||
            !std::isfinite(initial.smooth_beta) || initial.smooth_beta < 0.0 || initial.smooth_beta > 1.0 ||
            !std::isfinite(initial.smooth_tolerance) || initial.smooth_tolerance < 1e-12 ||
            !std::isfinite(initial.escape_clear_radius) || initial.escape_clear_radius < 0.0 ||
            !std::isfinite(initial.goal_tolerance_m) || initial.goal_tolerance_m < 0.0 ||
            !std::isfinite(initial.online_behind_clearance) || initial.online_behind_clearance < 0.0)
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
        // ROS 2 Humble service clients take an rmw QoS profile.
        cycles_[0].map_client = create_client<GetMap>(
            map_prefix + "get_static_map", rmw_qos_profile_services_default, update_group_);
        cycles_[0].cost_client = create_client<GetMap>(
            map_prefix + "get_static_cost_map", rmw_qos_profile_services_default, update_group_);
        cycles_[1].map_client = create_client<GetMap>(
            map_prefix + "get_augmented_map", rmw_qos_profile_services_default, update_group_);
        cycles_[1].cost_client = create_client<GetMap>(
            map_prefix + "get_augmented_cost_map", rmw_qos_profile_services_default, update_group_);

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
        options.escape_clear_radius = get_parameter("escape_clear_radius").as_double();
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
                if (name == "diagonal_paths") candidate.diagonal_paths = parameter.as_bool();
                else if (name == "allow_unknown") candidate.allow_unknown = parameter.as_bool();
                else if (name == "unknown_cost") candidate.unknown_cost = parameter.as_double();
                else if (name == "smooth_alpha") candidate.smooth_alpha = parameter.as_double();
                else if (name == "smooth_beta") candidate.smooth_beta = parameter.as_double();
                else if (name == "smooth_tolerance") candidate.smooth_tolerance = parameter.as_double();
                else if (name == "escape_clear_radius") candidate.escape_clear_radius = parameter.as_double();
                else if (name == "goal_relocation_radius") candidate.goal_tolerance_m = parameter.as_double();
                else if (name == "disable_rear_path_plan") candidate.disable_rear_path_plan = parameter.as_bool();
                else if (name == "online_behind_clearance") candidate.online_behind_clearance = parameter.as_double();
            }
            if (!std::isfinite(candidate.unknown_cost) || candidate.unknown_cost < 0.0 ||
                !std::isfinite(candidate.smooth_alpha) || candidate.smooth_alpha < 0.0 || candidate.smooth_alpha > 1.0 ||
                !std::isfinite(candidate.smooth_beta) || candidate.smooth_beta < 0.0 || candidate.smooth_beta > 1.0 ||
                !std::isfinite(candidate.smooth_tolerance) || candidate.smooth_tolerance < 1e-12 ||
                !std::isfinite(candidate.escape_clear_radius) || candidate.escape_clear_radius < 0.0 ||
                !std::isfinite(candidate.goal_tolerance_m) || candidate.goal_tolerance_m < 0.0 ||
                !std::isfinite(candidate.online_behind_clearance) || candidate.online_behind_clearance < 0.0)
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
            const auto& map = cycle.staged_map;
            // The producer stamps every layer of a snapshot together.
            if (map.header.stamp != costs.header.stamp ||
                map.header.frame_id.empty() || map.header.frame_id != costs.header.frame_id ||
                map.info.width == 0 || map.info.height == 0 ||
                map.info.width != costs.info.width || map.info.height != costs.info.height ||
                !std::isfinite(map.info.resolution) || map.info.resolution <= 0.0 ||
                map.info.resolution != costs.info.resolution || map.info.origin != costs.info.origin ||
                static_cast<uint64_t>(map.info.width) * map.info.height != map.data.size() ||
                map.data.size() != costs.data.size())
            {
                cycle.staged_map = Grid{};
                if (cycle.retries < retry_limit_)
                {
                    ++cycle.retries;
                    startUpdate(index, true);
                }
                else
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                        "Map/cost snapshot mismatch; retaining the previous snapshot %zu.", index);
                return;
            }
            auto next = std::make_shared<const MapPair>(
                MapPair{std::move(cycle.staged_map), std::move(costs)});
            {
                std::lock_guard<std::mutex> lock(snapshots_mutex_);
                snapshots_[index].swap(next);
            }
            // Release the old grids after leaving the critical section.
        }
        catch (const std::exception& error)
        {
            cycle.staged_map = Grid{};
            RCLCPP_WARN(get_logger(), "Cost-map response failed: %s", error.what());
        }
    }

    void expireRequests()
    {
        const auto current = SteadyClock::now();
        for (auto& cycle : cycles_)
        {
            if (cycle.pending && current >= cycle.deadline)
            {
                cycle.active_client->remove_pending_request(cycle.request_id);
                ++cycle.generation;
                cycle.pending = false;
                cycle.staged_map = Grid{};
                RCLCPP_WARN(get_logger(), "Map request timed out; retaining the previous snapshot.");
            }
        }
    }

    void planRequest(size_t index, GetPlan::Request::SharedPtr request, GetPlan::Response::SharedPtr response)
    {
        Snapshot snapshot;
        {
            std::lock_guard<std::mutex> lock(snapshots_mutex_);
            snapshot = snapshots_[index];
        }
        if (!snapshot)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "No complete map snapshot is available.");
            return;
        }
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
        auto result = PathPlanner::Plan(snapshot->map, snapshot->costs,
                                        request->start.pose, request->goal.pose, options);
        if (result.status != PlanStatus::Success)
        {
            RCLCPP_WARN(get_logger(), "Planning failed with status %d.", static_cast<int>(result.status));
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
        Snapshot snapshot;
        {
            std::lock_guard<std::mutex> lock(snapshots_mutex_);
            snapshot = snapshots_[1];
        }
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
        const auto options = readOptions();
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

    std::array<UpdateCycle, 2> cycles_;
    std::array<Snapshot, 2> snapshots_;
    std::mutex snapshots_mutex_;
    double request_timeout_;
    int retry_limit_;
    rclcpp::CallbackGroup::SharedPtr update_group_;
    rclcpp::TimerBase::SharedPtr refresh_timer_, watchdog_timer_;
    OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
    rclcpp::Service<GetPlan>::SharedPtr static_service_, augmented_service_;
    rclcpp::Service<GetPlanWithVia>::SharedPtr via_service_;
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
