#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/srv/get_map.hpp"
#include "nav_msgs/srv/get_plan.hpp"
#include "path_planner/PathPlanner.h"

#include <mutex>

class PathPlannerNode : public rclcpp::Node
{
public:
    PathPlannerNode() : Node("path_planner_node")
    {
        this->declare_parameter("use_namespace",        false);
        this->declare_parameter<float>("smooth_alpha",  0.1f);
        this->declare_parameter<float>("smooth_beta",   0.9f);
        this->declare_parameter<bool>("diagonal_paths", false);
        this->declare_parameter<float>("cost_radius",   0.25f);

        this->get_parameter("use_namespace",    use_namespace_);
        this->get_parameter("smooth_alpha",     smooth_alpha_);
        this->get_parameter("smooth_beta",      smooth_beta_);
        this->get_parameter("diagonal_paths",   diagonal_paths_);
        this->get_parameter("cost_radius",      cost_radius_);

        RCLCPP_INFO(this->get_logger(), "PathPlanner.-> Smooth Alpha: %.2f, Beta: %.2f, Diagonal: %s",
                    smooth_alpha_, smooth_beta_, diagonal_paths_ ? "true" : "false");

        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&PathPlannerNode::on_parameter_change, this, std::placeholders::_1));

        init_service_clients();

        // Subscribe to the augmented map published by map_augmenter (transient_local = latched).
        // This runs on the executor thread so there are no threading issues with the writer side.
        sub_augmented_map_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            make_name("/augmented_map"),
            rclcpp::QoS(1).transient_local(),
            std::bind(&PathPlannerNode::on_augmented_map, this, std::placeholders::_1));

        srv_plan_static_ = this->create_service<nav_msgs::srv::GetPlan>(
            make_name("/path_planner/plan_path_with_static"),
            std::bind(&PathPlannerNode::callback_a_star_with_static_map, this, std::placeholders::_1, std::placeholders::_2));

        srv_plan_augmented_ = this->create_service<nav_msgs::srv::GetPlan>(
            make_name("/path_planner/plan_path_with_augmented"),
            std::bind(&PathPlannerNode::callback_a_star_with_augmented_map, this, std::placeholders::_1, std::placeholders::_2));

        messages_call_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&PathPlannerNode::messages_caller, this));

        RCLCPP_INFO(this->get_logger(), "PathPlanner.-> PathPlannerNode is ready.");
    }

private:
    bool use_namespace_;
    float smooth_alpha_;
    float smooth_beta_;
    bool diagonal_paths_;
    float cost_radius_;

    // Static map fetched via services
    rclcpp::Client<nav_msgs::srv::GetMap>::SharedPtr clt_get_static_map_;
    rclcpp::Client<nav_msgs::srv::GetMap>::SharedPtr clt_get_static_cost_map_;

    // Augmented map received via topic subscription (replaces the service+thread approach)
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_augmented_map_;

    rclcpp::Service<nav_msgs::srv::GetPlan>::SharedPtr srv_plan_static_;
    rclcpp::Service<nav_msgs::srv::GetPlan>::SharedPtr srv_plan_augmented_;

    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    // Protects all four cached maps; held briefly for copy-in and copy-out only
    std::mutex maps_mutex_;
    nav_msgs::msg::OccupancyGrid map_;
    nav_msgs::msg::OccupancyGrid cost_map_;
    nav_msgs::msg::OccupancyGrid augmented_map_;
    nav_msgs::msg::OccupancyGrid augmented_cost_map_;

    rclcpp::TimerBase::SharedPtr messages_call_timer_;
    rclcpp::TimerBase::SharedPtr service_check_timer_;
    bool services_ready_ = false;

    rcl_interfaces::msg::SetParametersResult on_parameter_change(
        const std::vector<rclcpp::Parameter> &params)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "PathPlanner.-> Parameters updated successfully.";

        for (const auto &param : params)
        {
            if      (param.get_name() == "use_namespace")   use_namespace_  = param.as_bool();
            else if (param.get_name() == "smooth_alpha")    smooth_alpha_   = param.as_double();
            else if (param.get_name() == "smooth_beta")     smooth_beta_    = param.as_double();
            else if (param.get_name() == "diagonal_paths")  diagonal_paths_ = param.as_bool();
            else if (param.get_name() == "cost_radius")     cost_radius_    = param.as_double();
            else
            {
                result.successful = false;
                result.reason = "PathPlanner.-> Unsupported parameter: " + param.get_name();
                RCLCPP_WARN(this->get_logger(), "PathPlanner.-> Attempted to update unsupported parameter: %s",
                            param.get_name().c_str());
                break;
            }
        }
        return result;
    }

    std::string make_name(const std::string &suffix) const
    {
        std::string sfx = suffix;
        if (!sfx.empty() && sfx.front() != '/')
            sfx = "/" + sfx;

        std::string name;
        if (use_namespace_) {
            name = this->get_namespace() + sfx;
            if (name.size() > 1 && name[0] == '/' && name[1] == '/')
                name.erase(0, 1);
        } else {
            name = sfx;
        }
        return name;
    }

    void init_service_clients()
    {
        clt_get_static_map_ = this->create_client<nav_msgs::srv::GetMap>(
            make_name("/map_augmenter/get_static_map"));
        clt_get_static_cost_map_ = this->create_client<nav_msgs::srv::GetMap>(
            make_name("/map_augmenter/get_static_cost_map"));

        service_check_timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            [this]()
            {
                if (clt_get_static_map_->wait_for_service(std::chrono::seconds(0)) &&
                    clt_get_static_cost_map_->wait_for_service(std::chrono::seconds(0)))
                {
                    RCLCPP_INFO(this->get_logger(), "PathPlanner.-> All map services are now available.");
                    services_ready_ = true;
                    service_check_timer_->cancel();
                }
                else
                {
                    RCLCPP_WARN(this->get_logger(), "PathPlanner.-> Waiting for map services to become available...");
                }
            });
    }

    // Same cost-map algorithm as map_augmenter's get_cost_map().
    // Guards against zero/negative resolution to avoid division-by-zero or infinite loops.
    nav_msgs::msg::OccupancyGrid compute_cost_map(const nav_msgs::msg::OccupancyGrid &map)
    {
        if (map.info.resolution <= 0.0f || map.data.empty()) return map;

        nav_msgs::msg::OccupancyGrid cost_map = map;
        int steps = static_cast<int>(cost_radius_ / map.info.resolution);
        if (steps <= 0) return map;

        int box_size = (steps * 2 + 1) * (steps * 2 + 1);
        std::vector<int> cell_costs(box_size);
        std::vector<int> neighbors(box_size);

        int counter = 0;
        for (int i = -steps; i <= steps; ++i) {
            for (int j = -steps; j <= steps; ++j) {
                neighbors[counter] = i * static_cast<int>(map.info.width) + j;
                cell_costs[counter] = (steps - std::max(std::abs(i), std::abs(j)) + 1) * 2;
                ++counter;
            }
        }

        int start_idx = steps * static_cast<int>(map.info.width) + steps;
        int end_idx   = static_cast<int>(map.data.size()) - steps * static_cast<int>(map.info.width) - steps;

        for (int i = start_idx; i < end_idx; ++i) {
            if (map.data[i] > 0) {
                for (int j = 0; j < box_size; ++j) {
                    int nb = i + neighbors[j];
                    if (nb >= 0 && nb < static_cast<int>(cost_map.data.size())) {
                        if (cost_map.data[nb] < cell_costs[j])
                            cost_map.data[nb] = static_cast<int8_t>(cell_costs[j]);
                    }
                }
            }
        }
        return cost_map;
    }

    // Called on the executor thread whenever map_augmenter publishes a new augmented map.
    // With MultiThreadedExecutor this can overlap with service callbacks, hence the mutex.
    void on_augmented_map(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        if (msg->data.empty()) return;

        auto cost = compute_cost_map(*msg);

        std::unique_lock<std::mutex> lk(maps_mutex_);
        augmented_map_      = *msg;
        augmented_cost_map_ = std::move(cost);
        lk.unlock();

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "PathPlanner.-> Augmented map received (%u x %u).",
                             msg->info.width, msg->info.height);
    }

    // Refresh static maps via async service calls (no blocking future.get(), no threads).
    void update_static_maps()
    {
        if (!services_ready_) return;

        auto req_map = std::make_shared<nav_msgs::srv::GetMap::Request>();
        clt_get_static_map_->async_send_request(req_map,
            [this](rclcpp::Client<nav_msgs::srv::GetMap>::SharedFuture fut) {
                try {
                    auto result = fut.get();
                    if (!result->map.data.empty()) {
                        std::unique_lock<std::mutex> lk(maps_mutex_);
                        map_ = result->map;
                    }
                } catch (const std::exception &e) {
                    RCLCPP_ERROR(this->get_logger(), "PathPlanner.-> Failed to get static map: %s", e.what());
                }
            });

        auto req_cost = std::make_shared<nav_msgs::srv::GetMap::Request>();
        clt_get_static_cost_map_->async_send_request(req_cost,
            [this](rclcpp::Client<nav_msgs::srv::GetMap>::SharedFuture fut) {
                try {
                    auto result = fut.get();
                    if (!result->map.data.empty()) {
                        std::unique_lock<std::mutex> lk(maps_mutex_);
                        cost_map_ = result->map;
                    }
                } catch (const std::exception &e) {
                    RCLCPP_ERROR(this->get_logger(), "PathPlanner.-> Failed to get static cost map: %s", e.what());
                }
            });
    }

    void callback_a_star_with_static_map(
        const std::shared_ptr<nav_msgs::srv::GetPlan::Request> request,
        std::shared_ptr<nav_msgs::srv::GetPlan::Response> response)
    {
        nav_msgs::msg::OccupancyGrid map_copy, cost_copy;
        {
            std::unique_lock<std::mutex> lk(maps_mutex_);
            map_copy  = map_;
            cost_copy = cost_map_;
        }

        if (map_copy.data.empty() || cost_copy.data.empty()) {
            response->plan.poses.clear();
            return;
        }

        nav_msgs::msg::Path path;
        bool success = PathPlanner::AStar(map_copy, cost_copy,
            request->start.pose, request->goal.pose,
            diagonal_paths_, path);

        if (success) {
            nav_msgs::msg::Path smoothed = PathPlanner::SmoothPath(path, smooth_alpha_, smooth_beta_);
            smoothed.header.stamp    = this->get_clock()->now();
            smoothed.header.frame_id = "map";

            if (!smoothed.poses.empty()) {
                response->plan = smoothed;
                RCLCPP_INFO(this->get_logger(), "PathPlanner.-> Path planned successfully with size: %ld",
                            response->plan.poses.size());
            } else {
                RCLCPP_WARN(this->get_logger(), "PathPlanner.-> Failed to plan path.");
                response->plan.poses.clear();
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "PathPlanner.-> Failed to plan path.");
            response->plan.poses.clear();
        }
    }

    void callback_a_star_with_augmented_map(
        const std::shared_ptr<nav_msgs::srv::GetPlan::Request> request,
        std::shared_ptr<nav_msgs::srv::GetPlan::Response> response)
    {
        nav_msgs::msg::OccupancyGrid aug_copy, aug_cost_copy;
        {
            std::unique_lock<std::mutex> lk(maps_mutex_);
            aug_copy      = augmented_map_;
            aug_cost_copy = augmented_cost_map_;
        }

        if (aug_copy.data.empty() || aug_cost_copy.data.empty()) {
            RCLCPP_WARN(this->get_logger(),
                        "PathPlanner.-> Augmented map not yet available. Cannot plan path (map_augmenter still initializing).");
            response->plan.poses.clear();
            return;
        }

        nav_msgs::msg::Path path;
        bool success = PathPlanner::AStar(aug_copy, aug_cost_copy,
            request->start.pose, request->goal.pose,
            diagonal_paths_, path);

        if (success) {
            nav_msgs::msg::Path smoothed = PathPlanner::SmoothPath(path, smooth_alpha_, smooth_beta_);
            smoothed.header.stamp    = this->get_clock()->now();
            smoothed.header.frame_id = "map";

            if (!smoothed.poses.empty()) {
                response->plan = smoothed;
                RCLCPP_INFO(this->get_logger(), "PathPlanner.-> Path planned successfully with size: %ld",
                            response->plan.poses.size());
            } else {
                RCLCPP_WARN(this->get_logger(), "PathPlanner.-> Failed to plan path.");
                response->plan.poses.clear();
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "PathPlanner.-> Failed to plan path.");
            response->plan.poses.clear();
        }
    }

    void messages_caller()
    {
        update_static_maps();
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PathPlannerNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
