#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

// Message types
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/srv/get_map.hpp"

class MapServerBypassNode : public rclcpp::Node
{
public:
    MapServerBypassNode() : Node("map_server_bypass_node_node")
    {
        //############
        // Declare parameters with default values
        this->declare_parameter<int>("occupied_threshold", 70);

        // Initialize internal variables from declared parameters
        this->get_parameter("occupied_threshold", occupied_threshold_);

        // Setup parameter change callback
        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&MapServerBypassNode::on_parameter_change, this, std::placeholders::_1));

        //############
        // Publishers
        map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/cartographer_map", rclcpp::QoS(10).transient_local());

        //############
        // Subscribers
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
            std::bind(&MapServerBypassNode::callback_map, this, std::placeholders::_1));

        //############
        // Services
        static_map_srv_ = this->create_service<nav_msgs::srv::GetMap>(
            "/static_map",
            std::bind(&MapServerBypassNode::callback_static_map, this, std::placeholders::_1, std::placeholders::_2));

        cartographer_map_srv_ = this->create_service<nav_msgs::srv::GetMap>(
            "/get_cartographer_map",
            std::bind(&MapServerBypassNode::callback_get_cartographer_map, this, std::placeholders::_1, std::placeholders::_2));

        // Timer
        processing_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(30),
            std::bind(&MapServerBypassNode::map_server_bypass_processing, this));
    }

private:
    // Parameters
    int occupied_threshold_;

    //############
    // Publishers
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;

    //############
    // Subscribers
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;

    //############
    // Service servers
    rclcpp::Service<nav_msgs::srv::GetMap>::SharedPtr static_map_srv_;
    rclcpp::Service<nav_msgs::srv::GetMap>::SharedPtr cartographer_map_srv_;

    //############
    // Parameter callback handle
    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    // Main processing loop
    rclcpp::TimerBase::SharedPtr processing_timer_;

    // Stored map
    nav_msgs::msg::OccupancyGrid latest_map_;
    bool map_received_ = false;

    //############
    // Runtime parameter update callback
    rcl_interfaces::msg::SetParametersResult on_parameter_change(
        const std::vector<rclcpp::Parameter> &params)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        for (const auto &param : params)
        {
            if (param.get_name()      == "occupied_threshold")      occupied_threshold_        = param.as_int();

            else {
                result.successful = false;
                result.reason = "MapServerBypass.-> Unsupported parameter: " + param.get_name();
                RCLCPP_WARN(this->get_logger(), "MapServerBypass.-> Attempted to update unsupported parameter: %s", param.get_name().c_str());
                break;
            }
        }

        return result;
    }

    // Subscribers callbacks
    // map
    void callback_map(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        nav_msgs::msg::OccupancyGrid new_map;
        new_map.header = msg->header;
        new_map.header.stamp = this->now();
        new_map.info = msg->info;

        std::vector<int8_t> new_data;
        new_data.reserve(msg->data.size());

        for (auto val : msg->data)
        {
            if (val > occupied_threshold_)
                new_data.push_back(100);
            else if (val >= 0)
                new_data.push_back(0);
            else
                new_data.push_back(0); // unknown treated as free
        }

        new_map.data = std::move(new_data);
        latest_map_ = std::move(new_map);
        map_received_ = true;
    }

    // Services callbacks
    // static map
    void callback_static_map(
        const std::shared_ptr<nav_msgs::srv::GetMap::Request> request,
        std::shared_ptr<nav_msgs::srv::GetMap::Response> response)
    {
        (void)request;
        response->map = latest_map_;
    }

    // cartographer map
    void callback_get_cartographer_map(
        const std::shared_ptr<nav_msgs::srv::GetMap::Request> request,
        std::shared_ptr<nav_msgs::srv::GetMap::Response> response)
    {
        (void)request;
        response->map = latest_map_;
    }

    //############
    //Map Server Bypass main processing
    void map_server_bypass_processing()
    {
        if (map_received_)
        {
            latest_map_.header.stamp = this->now();
            map_pub_->publish(latest_map_);
        }
    }
};

// Main function
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MapServerBypassNode>();
    //rclcpp::spin(node);
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}
