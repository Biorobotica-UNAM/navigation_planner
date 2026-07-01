#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

// Message types
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/empty.hpp"
#include "actionlib_msgs/msg/goal_status.hpp"

// TF2 (Transform listener)
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/exceptions.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

// Action messages (used less directly in ROS 2; here for compatibility)
#include "action_msgs/msg/goal_status.hpp"

// Standard
#include <iostream>
#include <vector>
#include <exception>
#include <algorithm>
#include <cmath>
#include <sstream>
#include <string>
#include <limits>
///

#define RATE 30

enum class SM_STATE{
    INIT,
    WAITING_FOR_TASK,
    WAIT_FOR_HEAD_START_DELAY,
    GOAL_POSE_ACCEL,
    GOAL_POSE_CRUISE,
    GOAL_POSE_DECCEL,
    GOAL_POSE_CORRECT_ANGLE,
    GOAL_POSE_FINISH,
    GOAL_POSE_FAILED,
    GOAL_PATH_ACCEL,
    GOAL_PATH_CRUISE,
    GOAL_PATH_DECCEL,
    GOAL_PATH_FINISH,
    GOAL_PATH_FAILED,
    COLLISION_RISK,
};


class SimpleMoveNode : public rclcpp::Node
{
public:
    SimpleMoveNode() : Node("simple_move_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
    {
        //############
        // Declare parameters with default values
        this->declare_parameter<bool>("use_namespace",          false);
        this->declare_parameter<float>("max_linear_speed",      0.3f);
        this->declare_parameter<float>("min_linear_speed",      0.05f);
        this->declare_parameter<float>("max_angular_speed",     1.0f);
        this->declare_parameter<float>("final_angle_max_angular_speed", 0.35f);
        this->declare_parameter<float>("control_alpha",         0.6548f);
        this->declare_parameter<float>("control_beta",          0.2f);
        this->declare_parameter<float>("linear_acceleration",   0.1f);
        this->declare_parameter<float>("fine_dist_tolerance",   0.03f);
        this->declare_parameter<float>("coarse_dist_tolerance", 0.2f);
        this->declare_parameter<float>("angle_tolerance",       0.05f);
        this->declare_parameter<bool>("mapped_area_speed_enabled", false);
        this->declare_parameter<float>("mapped_area_max_linear_speed", 0.55f);
        this->declare_parameter<float>("mapped_area_radius", 0.75f);
        this->declare_parameter<float>("mapped_area_min_known_ratio", 0.85f);
        this->declare_parameter<std::string>("mapped_area_map_topic", "map");

        this->declare_parameter<bool>("move_head", true);
        // Optional SLAM exploration mode: while the local map is still sparse,
        // add a slow lateral pan sweep to the normal path-following head angle.
        this->declare_parameter<bool>("head_unknown_scan_enabled", false);
        this->declare_parameter<float>("head_unknown_scan_min_known_ratio", 0.85f);
        this->declare_parameter<float>("head_unknown_scan_amplitude", 0.75f);
        this->declare_parameter<float>("head_unknown_scan_period", 3.0f);
        this->declare_parameter<float>("head_pan_limit", 1.2f);
        this->declare_parameter<float>("head_tilt", -1.0f);
        this->declare_parameter<float>("head_start_delay_sec", 1.5f);
        this->declare_parameter<bool>("use_pot_fields", false);
        // Disabled by default: immediate abort/replan is smoother than
        // rotating in place when collision_risk flickers near obstacles.
        this->declare_parameter<bool>("allow_rotation_on_collision", false);
        this->declare_parameter<float>("rotation_collision_angle_threshold", 0.35f);
        this->declare_parameter<float>("rotation_collision_angular_speed", 0.45f);
        this->declare_parameter<std::string>("base_link_name", "base_footprint");
        this->declare_parameter<std::string>("odom_name", "odom");

        // Initialize internal variables from declared parameters
        this->get_parameter("use_namespace",         use_namespace_);
        this->get_parameter("max_linear_speed",      max_linear_speed_);
        this->get_parameter("min_linear_speed",      min_linear_speed_);
        this->get_parameter("max_angular_speed",     max_angular_speed_);
        this->get_parameter("final_angle_max_angular_speed", final_angle_max_angular_speed_);
        final_angle_max_angular_speed_ = std::max(0.05f, final_angle_max_angular_speed_);
        this->get_parameter("control_alpha",         alpha_);
        this->get_parameter("control_beta",          beta_);
        this->get_parameter("linear_acceleration",   linear_acceleration_);
        linear_acceleration_ = std::max(0.01f, linear_acceleration_);
        this->get_parameter("fine_dist_tolerance",   fine_dist_tolerance_);
        this->get_parameter("coarse_dist_tolerance", coarse_dist_tolerance_);
        this->get_parameter("angle_tolerance",       angle_tolerance_);
        this->get_parameter("mapped_area_speed_enabled", mapped_area_speed_enabled_);
        this->get_parameter("mapped_area_max_linear_speed", mapped_area_max_linear_speed_);
        this->get_parameter("mapped_area_radius", mapped_area_radius_);
        this->get_parameter("mapped_area_min_known_ratio", mapped_area_min_known_ratio_);
        this->get_parameter("mapped_area_map_topic", mapped_area_map_topic_);

        this->get_parameter("move_head",             move_head_);
        this->get_parameter("head_unknown_scan_enabled", head_unknown_scan_enabled_);
        this->get_parameter("head_unknown_scan_min_known_ratio", head_unknown_scan_min_known_ratio_);
        this->get_parameter("head_unknown_scan_amplitude", head_unknown_scan_amplitude_);
        this->get_parameter("head_unknown_scan_period", head_unknown_scan_period_);
        this->get_parameter("head_pan_limit", head_pan_limit_);
        this->get_parameter("head_tilt", head_tilt_);
        this->get_parameter("head_start_delay_sec", head_start_delay_sec_);
        head_unknown_scan_min_known_ratio_ = std::clamp(head_unknown_scan_min_known_ratio_, 0.0f, 1.0f);
        head_unknown_scan_amplitude_ = std::max(0.0f, head_unknown_scan_amplitude_);
        head_unknown_scan_period_ = std::max(0.5f, head_unknown_scan_period_);
        head_pan_limit_ = std::max(0.0f, head_pan_limit_);
        head_start_delay_sec_ = std::max(0.0f, head_start_delay_sec_);
        this->get_parameter("use_pot_fields",        use_pot_fields_);
        this->get_parameter("allow_rotation_on_collision", allow_rotation_on_collision_);
        this->get_parameter("rotation_collision_angle_threshold", rotation_collision_angle_threshold_);
        this->get_parameter("rotation_collision_angular_speed", rotation_collision_angular_speed_);
        this->get_parameter("base_link_name",        base_link_name_);
        this->get_parameter("odom_name",             odom_name_);

        // Setup parameter change callback
        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&SimpleMoveNode::on_parameter_change, this, std::placeholders::_1));

        //############
        // Publishers
        pub_cmd_vel_        = this->create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel", 
            rclcpp::QoS(10).transient_local());
        pub_goal_reached_   = this->create_publisher<actionlib_msgs::msg::GoalStatus>(
            make_name("/simple_move/goal_reached"), 
            rclcpp::QoS(10).transient_local());
        pub_head_goal_pose_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/hardware/head/goal_pose", 
            rclcpp::QoS(10).durability_volatile());

        //############
        // Subscribers
        //// stop
        sub_generalStop_ = this->create_subscription<std_msgs::msg::Empty>(
            make_name("/stop"), 
            rclcpp::SensorDataQoS(), 
            std::bind(&SimpleMoveNode::callback_general_stop, this, std::placeholders::_1));

        //sub_navCtrlStop_ = this->create_subscription<std_msgs::msg::Bool>(
        //    make_name("/navigation_ctrl/stop"), 
        //    rclcpp::SensorDataQoS(), 
        //    std::bind(&SimpleMoveNode::callback_navigation_stop, this, std::placeholders::_1));

        sub_navSimpleMvStop_ = this->create_subscription<std_msgs::msg::Empty>(
            make_name("/simple_move/stop"), 
            rclcpp::SensorDataQoS(), 
            std::bind(&SimpleMoveNode::callback_simple_move_stop, this, std::placeholders::_1));

        //// goal
        sub_goalDistance_ = this->create_subscription<std_msgs::msg::Float32>(
            make_name("/simple_move/goal_dist"), 
            rclcpp::SensorDataQoS(), 
            std::bind(&SimpleMoveNode::callback_goal_dist, this, std::placeholders::_1));
        
        sub_goalDistAngle_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            make_name("/simple_move/goal_dist_angle"), 
            rclcpp::SensorDataQoS(), 
            std::bind(&SimpleMoveNode::callback_goal_dist_angle, this, std::placeholders::_1));

        sub_goalPath_ = this->create_subscription<nav_msgs::msg::Path>(
            make_name("/simple_move/goal_path"), 
            rclcpp::SensorDataQoS(), 
            std::bind(&SimpleMoveNode::callback_goal_path, this, std::placeholders::_1));

        sub_slam_map_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            resolve_topic_name(mapped_area_map_topic_),
            rclcpp::QoS(1).transient_local(),
            std::bind(&SimpleMoveNode::callback_slam_map, this, std::placeholders::_1));

        sub_headCurrentPose_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/hardware/head/current_pose",
            rclcpp::SensorDataQoS(),
            std::bind(&SimpleMoveNode::callback_head_current_pose, this, std::placeholders::_1));

        //// motion
        sub_moveLateral_ = this->create_subscription<std_msgs::msg::Float32>(
            make_name("/simple_move/goal_dist_lateral"), 
            rclcpp::SensorDataQoS(), 
            std::bind(&SimpleMoveNode::callback_move_lateral, this, std::placeholders::_1));

        sub_collisionRisk_ = this->create_subscription<std_msgs::msg::Bool>(
            make_name("/potential_fields/collision_risk"), 
            rclcpp::SensorDataQoS(), 
            std::bind(&SimpleMoveNode::callback_collision_risk, this, std::placeholders::_1));

        sub_rejectionForce_ = this->create_subscription<geometry_msgs::msg::Vector3>(
            make_name("/potential_fields/pf_rejection_force"), 
            rclcpp::SensorDataQoS(), 
            std::bind(&SimpleMoveNode::callback_rejection_force, this, std::placeholders::_1));

        sub_bypass_ = this->create_subscription<std_msgs::msg::Bool>(
        "/navigation/camera_obstacles/bypass", rclcpp::QoS(10),
        [this](const std_msgs::msg::Bool::SharedPtr msg) { this->camera_bypass_ = msg->data; });
        
        //############
        // Wait for transforms
        wait_for_transforms("map", base_link_name_);
        wait_for_transforms(odom_name_, base_link_name_);

        // Simple Move main processing
        processing_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(30),
            std::bind(&SimpleMoveNode::simple_move_processing, this));

        RCLCPP_INFO(this->get_logger(), "SimpleMove.-> SimpleMoveNode is ready.");
    }

private:
    //############
    // State variables
    bool use_namespace_   = false;
    float goal_distance_  = 0;
    float goal_angle_     = 0;
    bool  new_pose_       = false;
    bool  new_path_       = false;
    bool  collision_risk_ = false;
    bool  move_lat_       = false;
    bool  stop_           = false;

    float robot_x_ = 0;
    float robot_y_ = 0;
    float robot_t_ = 0;
    float goal_x_ = 0;
    float goal_y_ = 0;
    float goal_t_ = 0;
    float global_goal_x_ = 0;
    float global_goal_y_ = 0;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    nav_msgs::msg::Path           goal_path_;
    nav_msgs::msg::OccupancyGrid  slam_map_;
    geometry_msgs::msg::Vector3   rejection_force_;
    bool is_slam_map_ = false;

    // Internal parameter values
    float max_linear_speed_;
    float min_linear_speed_;
    float max_angular_speed_;
    float final_angle_max_angular_speed_;
    float alpha_;
    float beta_;
    float linear_acceleration_;
    float fine_dist_tolerance_;
    float coarse_dist_tolerance_;
    float angle_tolerance_;
    bool mapped_area_speed_enabled_;
    float mapped_area_max_linear_speed_;
    float mapped_area_radius_;
    float mapped_area_min_known_ratio_;
    std::string mapped_area_map_topic_;

    bool move_head_;
    bool head_unknown_scan_enabled_;
    float head_unknown_scan_min_known_ratio_;
    float head_unknown_scan_amplitude_;
    float head_unknown_scan_period_;
    float head_pan_limit_;
    float head_tilt_;
    float head_start_delay_sec_;
    bool use_pot_fields_;
    bool allow_rotation_on_collision_;
    float rotation_collision_angle_threshold_;
    float rotation_collision_angular_speed_;
    std::string base_link_name_;
    std::string odom_name_;

    // Simple Move processing variables
    actionlib_msgs::msg::GoalStatus msg_goal_reached;

    SM_STATE state = SM_STATE::INIT;
    float current_linear_speed = 0;
    size_t prev_pose_idx = 0;
    size_t next_pose_idx = 0;
    float temp_k = 0;
    int attempts = 0;
    float global_error = 0;
    rclcpp::Time head_start_delay_start_;

    bool camera_bypass_ = false;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_bypass_;


    //############
    // Publishers
    rclcpp::Publisher<actionlib_msgs::msg::GoalStatus>::SharedPtr   pub_goal_reached_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr         pub_cmd_vel_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr  pub_head_goal_pose_;

    //############
    // Subscribers
    //// stop
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_generalStop_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_navCtrlStop_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_navSimpleMvStop_;

    //// goal
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_goalDistance_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_goalDistAngle_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_goalPath_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_slam_map_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_headCurrentPose_;

    //// motion
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_moveLateral_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_collisionRisk_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr sub_rejectionForce_;

    //############
    // Parameter callback handle
    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    // Main processing loop
    rclcpp::TimerBase::SharedPtr processing_timer_;

    bool current_head_pose_received_ = false;
    double current_head_pan_ = 0.0;
    double current_head_tilt_ = 0.0;
    bool head_start_delay_armed_ = true;

    //############
    // Runtime parameter update callback
    rcl_interfaces::msg::SetParametersResult on_parameter_change(
        const std::vector<rclcpp::Parameter> &params)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        for (const auto &param : params)
        {
            if (param.get_name()      == "use_namespace")         use_namespace_           = param.as_bool();
            else if (param.get_name() == "max_linear_speed")      max_linear_speed_        = param.as_double();
            else if (param.get_name() == "min_linear_speed")      min_linear_speed_        = param.as_double();
            else if (param.get_name() == "max_angular_speed")     max_angular_speed_       = param.as_double();
            else if (param.get_name() == "final_angle_max_angular_speed") final_angle_max_angular_speed_ = std::max(0.05, param.as_double());
            else if (param.get_name() == "control_alpha")         alpha_                   = param.as_double();
            else if (param.get_name() == "control_beta")          beta_                    = param.as_double();
            else if (param.get_name() == "linear_acceleration")   linear_acceleration_     = std::max(0.01, param.as_double());
            else if (param.get_name() == "fine_dist_tolerance")   fine_dist_tolerance_     = param.as_double();
            else if (param.get_name() == "coarse_dist_tolerance") coarse_dist_tolerance_   = param.as_double();
            else if (param.get_name() == "angle_tolerance")       angle_tolerance_         = param.as_double();
            else if (param.get_name() == "mapped_area_speed_enabled") mapped_area_speed_enabled_ = param.as_bool();
            else if (param.get_name() == "mapped_area_max_linear_speed") mapped_area_max_linear_speed_ = param.as_double();
            else if (param.get_name() == "mapped_area_radius") mapped_area_radius_ = std::max(0.0, param.as_double());
            else if (param.get_name() == "mapped_area_min_known_ratio") mapped_area_min_known_ratio_ = std::clamp(param.as_double(), 0.0, 1.0);
            else if (param.get_name() == "mapped_area_map_topic") {
                result.successful = false;
                result.reason = "SimpleMove.-> mapped_area_map_topic requires node restart.";
                break;
            }

            else if (param.get_name() == "move_head")             move_head_               = param.as_bool();
            else if (param.get_name() == "head_unknown_scan_enabled") head_unknown_scan_enabled_ = param.as_bool();
            else if (param.get_name() == "head_unknown_scan_min_known_ratio") head_unknown_scan_min_known_ratio_ = std::clamp(param.as_double(), 0.0, 1.0);
            else if (param.get_name() == "head_unknown_scan_amplitude") head_unknown_scan_amplitude_ = std::max(0.0, param.as_double());
            else if (param.get_name() == "head_unknown_scan_period") head_unknown_scan_period_ = std::max(0.5, param.as_double());
            else if (param.get_name() == "head_pan_limit") head_pan_limit_ = std::max(0.0, param.as_double());
            else if (param.get_name() == "head_tilt") head_tilt_ = param.as_double();
            else if (param.get_name() == "head_start_delay_sec") head_start_delay_sec_ = std::max(0.0, param.as_double());
            else if (param.get_name() == "use_pot_fields")        use_pot_fields_          = param.as_bool();
            else if (param.get_name() == "allow_rotation_on_collision") allow_rotation_on_collision_ = param.as_bool();
            else if (param.get_name() == "rotation_collision_angle_threshold") rotation_collision_angle_threshold_ = param.as_double();
            else if (param.get_name() == "rotation_collision_angular_speed") rotation_collision_angular_speed_ = param.as_double();
            else if (param.get_name() == "base_link_name")        base_link_name_          = param.as_string();
            else if (param.get_name() == "odom_name")             odom_name_               = param.as_string();

            else {
                result.successful = false;
                result.reason = "SimpleMove.-> Unsupported parameter: " + param.get_name();
                RCLCPP_WARN(this->get_logger(), "SimpleMove.-> Attempted to update unsupported parameter: %s", param.get_name().c_str());
                break;
            }
        }

        return result;
    }

    std::string make_name(const std::string &suffix) const
    {
        // Ensure suffix starts with "/"
        std::string sfx = suffix;
        if (!sfx.empty() && sfx.front() != '/')
            sfx = "/" + sfx;

        std::string name;

        if (use_namespace_) {
            // Use node namespace prefix
            name = this->get_namespace() + sfx;

            // Avoid accidental double slash (e.g., when namespace is "/")
            if (name.size() > 1 && name[0] == '/' && name[1] == '/')
                name.erase(0, 1);
        } else {
            // Use global namespace (no node namespace prefix)
            name = sfx;
        }

        return name;
    }

    std::string resolve_topic_name(const std::string &topic) const
    {
        if (!topic.empty() && topic.front() == '/')
            return topic;
        return make_name(topic);
    }

    // Wait for transforms 
    void wait_for_transforms(const std::string &target_frame, const std::string &source_frame)
    {
        RCLCPP_INFO(this->get_logger(),
                    "SimpleMove.-> Waiting for transform from '%s' to '%s'...", source_frame.c_str(), target_frame.c_str());

        rclcpp::Time start_time = this->now();
        rclcpp::Duration timeout = rclcpp::Duration::from_seconds(10.0); 

        bool transform_ok = false;

        while (rclcpp::ok() && (this->now() - start_time) < timeout) {
            try {
                tf_buffer_.lookupTransform(
                    target_frame,
                    source_frame,
                    tf2::TimePointZero,
                    tf2::durationFromSec(0.1)
                );
                transform_ok = true;
                break;
            } catch (const tf2::TransformException& ex) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                    "SimpleMove.-> Still waiting for transform: %s", ex.what());
            }

            rclcpp::sleep_for(std::chrono::milliseconds(200));
        }

        if (!transform_ok) {
            RCLCPP_WARN(this->get_logger(),
                        "SimpleMove.-> Timeout while waiting for transform from '%s' to '%s'.",
                        source_frame.c_str(), target_frame.c_str());
        } else {
            RCLCPP_INFO(this->get_logger(),
                        "SimpleMove.-> Transform from '%s' to '%s' is now available.",
                        source_frame.c_str(), target_frame.c_str());
        }
    }


    //############
    //Simple Move callbacks
    void callback_general_stop(const std_msgs::msg::Empty::SharedPtr msg) 
	{
		(void) msg;
		try 
		{
		    std::cout << "SimpleMove.-> General Stop signal received" << std::endl;
		    stop_     = true;
		    new_pose_ = false;
		    new_path_ = false;
		    move_lat_ = false;
		} 
		catch (const std::exception &e) 
		{
		    RCLCPP_ERROR(this->get_logger(), "SimpleMove.-> Error processing callback_general_stop: %s", e.what());
		}
	}

	    void callback_navigation_stop(const std_msgs::msg::Bool::SharedPtr msg) 
		{
			try 
			{
			    if (msg->data) {
			        std::cout << "SimpleMove.-> Navigation Stop signal received" << std::endl;
			        stop_     = true;
			        new_pose_ = false;
		        new_path_ = false;
		        move_lat_ = false;
		    }
		} 
		catch (const std::exception &e) 
		{
		    RCLCPP_ERROR(this->get_logger(), "SimpleMove.-> Error processing callback_navigation_stop: %s", e.what());
		}
	}

    void callback_simple_move_stop(const std_msgs::msg::Empty::SharedPtr msg) 
    {
        (void) msg;
        try 
        {
            std::cout << "SimpleMove.-> Simple move stop signal received" << std::endl;
            stop_     = true;
            new_pose_ = false;
            new_path_ = false;
        } 
        catch (const std::exception &e) 
        {
            RCLCPP_ERROR(this->get_logger(), "SimpleMove.-> Error processing callback_simple_move_stop: %s", e.what());
        }
    }

    void callback_goal_dist(const std_msgs::msg::Float32::SharedPtr msg) 
    {
        try 
        {
            std::cout << "SimpleMove.-> New move received: goal dist= " << msg->data << std::endl;     
            goal_distance_ = msg->data;
            goal_angle_    = 0;
            new_pose_ = true;
            new_path_ = false;
            move_lat_ = false;
        } 
        catch (const std::exception &e) 
        {
            RCLCPP_ERROR(this->get_logger(), "SimpleMove.-> Error processing callback_goal_dist: %s", e.what());
        }
    }

    void callback_goal_dist_angle(const std_msgs::msg::Float32MultiArray::SharedPtr msg) 
    {
        try 
        {
            std::cout << "SimpleMove.-> New move received: goal dist= " << msg->data[0] << " and goal angle= " << msg->data[1] << std::endl;
            goal_distance_ = msg->data[0];
            goal_angle_    = msg->data[1];
            new_pose_ = true;
            new_path_ = false;
            move_lat_ = false;
        } 
        catch (const std::exception &e) 
        {
            RCLCPP_ERROR(this->get_logger(), "SimpleMove.-> Error processing callback_goal_dist_angle: %s", e.what());
        }
    }

    void callback_goal_path(const nav_msgs::msg::Path::SharedPtr msg) 
    {
        try 
        {
	            std::cout << "SimpleMove.-> New path received with " << msg->poses.size()
	                    << " points. Stamp= " << msg->header.stamp.sec << "s, Frame="  << msg->header.frame_id
	                    << std::endl;
	            move_lat_ = false;
            if (msg->poses.size() <= 0)
            {
                new_pose_ = false;
                new_path_ = false;   
            }else{
                goal_path_ = *msg;
                new_pose_ = false;
                new_path_ = true;
            }
        } 
        catch (const std::exception &e) 
        {
            RCLCPP_ERROR(this->get_logger(), "SimpleMove.-> Error processing callback_goal_path: %s", e.what());
        }
    }

    void callback_slam_map(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        slam_map_ = *msg;
        is_slam_map_ = !slam_map_.data.empty() &&
                       slam_map_.info.width > 0 &&
                       slam_map_.info.height > 0 &&
                       slam_map_.info.resolution > 0.0f;
    }

    void callback_head_current_pose(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if (msg->data.size() < 2) {
            return;
        }
        current_head_pan_ = msg->data[0];
        current_head_tilt_ = msg->data[1];
        current_head_pose_received_ = true;
        if (!is_current_head_straight()) {
            head_start_delay_armed_ = true;
        }
    }

    void callback_move_lateral(const std_msgs::msg::Float32::SharedPtr msg) 
    {
        try 
        {
            goal_distance_ = msg->data;
            goal_angle_    = 0;
            new_pose_ = true;
            new_path_ = false;
            move_lat_ = true;
        } 
        catch (const std::exception &e) 
        {
            RCLCPP_ERROR(this->get_logger(), "SimpleMove.-> Error processing callback_move_lateral: %s", e.what());
        }
    }

	    void callback_collision_risk(const std_msgs::msg::Bool::SharedPtr msg) 
	    {
	        try 
	        {
	            collision_risk_ = msg->data;
	        } 
        catch (const std::exception &e) 
        {
            RCLCPP_ERROR(this->get_logger(), "SimpleMove.-> Error processing callback_collision_risk: %s", e.what());
        }
    }

    void callback_rejection_force(const geometry_msgs::msg::Vector3::SharedPtr msg) 
    {
        try 
	        {
	            rejection_force_ = *msg;
	        } 
        catch (const std::exception &e) 
        {
            RCLCPP_ERROR(this->get_logger(), "SimpleMove.-> Error processing callback_rejection_force: %s", e.what());
        }
    }


    //############
    //Simple Move additional functions
	    geometry_msgs::msg::Twist calculate_speeds( float robot_x, float robot_y, float robot_t,
                                                float goal_x, float goal_y,
                                                float min_linear_speed, float max_linear_speed, float angular_speed, 
                                                float alpha, float beta,
                                                bool backwards, bool move_lat, 
                                                bool use_pot_fields=false,  double rejection_force_y=0.0)
    {
        float angle_error = 0;
        if(backwards) angle_error = (atan2(robot_y - goal_y, robot_x -goal_x)-robot_t);
        else angle_error = (atan2(goal_y - robot_y, goal_x - robot_x)-robot_t);
        if(angle_error >   M_PI) angle_error -= 2*M_PI;
        if(angle_error <= -M_PI) angle_error += 2*M_PI;
        if(move_lat) angle_error -= M_PI/2;
        if(angle_error <= -M_PI) angle_error += 2*M_PI;
        if(backwards) max_linear_speed *= -1;
        
        geometry_msgs::msg::Twist result;
        if(move_lat)
            result.linear.y   = max_linear_speed  * exp(-(angle_error * angle_error) / (alpha));
        else result.linear.x  = max_linear_speed  * exp(-(angle_error * angle_error) / (alpha));
        result.angular.z = angular_speed * (2 / (1 + exp(-angle_error / beta)) - 1);

        if(fabs(result.linear.x) < min_linear_speed)
            result.linear.x = 0;
        if(fabs(result.linear.y) < min_linear_speed)
            result.linear.y = 0;
        
        if(use_pot_fields && !move_lat) result.linear.y = rejection_force_y;
        
	        return result;
	    }

    bool should_apply_potential_fields() const
    {
        // Apply the rejection force whenever it has meaningful magnitude —
        // not just when collision_risk fires — so the robot starts deflecting
        // as soon as an obstacle enters the influence zone (d0), before the
        // hard-stop threshold is reached.
        return use_pot_fields_ && (std::fabs(rejection_force_.y) > 0.005f);
    }

    float get_mapped_area_known_ratio(const float x, const float y) const
    {
        if (!is_slam_map_ || slam_map_.info.resolution <= 0.0f) {
            return 0.0f;
        }

        const float resolution = slam_map_.info.resolution;
        const float origin_x = static_cast<float>(slam_map_.info.origin.position.x);
        const float origin_y = static_cast<float>(slam_map_.info.origin.position.y);
        const int width = static_cast<int>(slam_map_.info.width);
        const int height = static_cast<int>(slam_map_.info.height);
        const int center_x = static_cast<int>(std::floor((x - origin_x) / resolution));
        const int center_y = static_cast<int>(std::floor((y - origin_y) / resolution));

        if (center_x < 0 || center_x >= width || center_y < 0 || center_y >= height) {
            return 0.0f;
        }

        const int radius_cells = std::max(1, static_cast<int>(std::ceil(mapped_area_radius_ / resolution)));
        int known_cells = 0;
        int total_cells = 0;

        for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
            for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
                if (dx * dx + dy * dy > radius_cells * radius_cells) continue;
                ++total_cells;

                const int mx = center_x + dx;
                const int my = center_y + dy;
                if (mx < 0 || mx >= width || my < 0 || my >= height) continue;

                const size_t idx = static_cast<size_t>(my) * static_cast<size_t>(width) +
                                   static_cast<size_t>(mx);
                if (idx < slam_map_.data.size() && slam_map_.data[idx] >= 0) {
                    ++known_cells;
                }
            }
        }

        if (total_cells == 0) return 0.0f;
        return static_cast<float>(known_cells) / static_cast<float>(total_cells);
    }

    float get_path_max_linear_speed_cap() const
    {
        if (!mapped_area_speed_enabled_ || collision_risk_ ||
            mapped_area_max_linear_speed_ <= max_linear_speed_) {
            return max_linear_speed_;
        }

        const float known_ratio = get_mapped_area_known_ratio(robot_x_, robot_y_);
        if (known_ratio >= mapped_area_min_known_ratio_) {
            return mapped_area_max_linear_speed_;
        }
        return max_linear_speed_;
    }

    bool should_scan_head_for_unknown_space() const
    {
        if (!head_unknown_scan_enabled_ || !is_slam_map_) {
            return false;
        }

        // Reuse the same local known-cell ratio used for speed limiting: below
        // this threshold, Kinect coverage is improved by looking to the sides.
        const float known_ratio = get_mapped_area_known_ratio(robot_x_, robot_y_);
        return known_ratio < head_unknown_scan_min_known_ratio_;
    }

    void clamp_current_linear_speed_to(const float speed_cap)
    {
        if (current_linear_speed > speed_cap) {
            current_linear_speed = std::max(speed_cap, current_linear_speed - linear_acceleration_ / RATE);
        }
    }

    geometry_msgs::msg::Twist calculate_speeds( float robot_angle, float goal_angle, float angular_speed, float beta)
    {
        float angle_error = goal_angle - robot_angle;
        if(angle_error >   M_PI) angle_error -= 2*M_PI;
        if(angle_error <= -M_PI) angle_error += 2*M_PI;

        geometry_msgs::msg::Twist result;
        result.linear.x  = 0;
        result.angular.z = angular_speed * (2 / (1 + exp(-angle_error / beta)) - 1);
        
        return result;
    }

    float get_heading_error_to_current_path_goal() const
    {
        float angle_error = atan2(goal_y_ - robot_y_, goal_x_ - robot_x_) - robot_t_;
        if(angle_error >   M_PI) angle_error -= 2*M_PI;
        if(angle_error <= -M_PI) angle_error += 2*M_PI;
        return angle_error;
    }

    bool try_rotate_in_place_on_collision(const char * state_label)
    {
        if (!allow_rotation_on_collision_)
            return false;

        get_robot_position_wrt_map();
        get_next_goal_from_path(prev_pose_idx, next_pose_idx);

        const float angle_error = get_heading_error_to_current_path_goal();
        if (fabs(angle_error) < rotation_collision_angle_threshold_)
            return false;

        // Optional recovery for narrow passages: rotate in place only when an
        // operator explicitly enables it, because flickering obstacle reports
        // can otherwise make the base oscillate.
        const float angular_speed = std::min(max_angular_speed_, rotation_collision_angular_speed_);
        pub_cmd_vel_->publish(calculate_speeds(robot_t_, robot_t_ + angle_error, angular_speed, beta_));
        if(move_head_ && !camera_bypass_) {
            pub_head_goal_pose_->publish(get_next_goal_head_angles(next_pose_idx));
        }

        current_linear_speed = 0;

        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,
            "SimpleMove.-> Collision risk in %s, rotating in place to recover. heading_error=%.2f",
            state_label,
            angle_error);
        return true;
    }

	    bool get_robot_position_wrt_map()
    {
        try
        {
            geometry_msgs::msg::TransformStamped transformStamped =
                tf_buffer_.lookupTransform("map", base_link_name_, tf2::TimePointZero);

            robot_x_ = transformStamped.transform.translation.x;
            robot_y_ = transformStamped.transform.translation.y;

            tf2::Quaternion q(
                transformStamped.transform.rotation.x,
                transformStamped.transform.rotation.y,
                transformStamped.transform.rotation.z,
                transformStamped.transform.rotation.w);

            double roll, pitch, yaw;
            tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
            robot_t_ = static_cast<float>(yaw);

            return true;
        }
	        catch (const tf2::TransformException &ex)
	        {
	            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                    "SimpleMove.-> map TF lookup failed (%s -> map): %s",
                                    base_link_name_.c_str(), ex.what());
	            return false;
	        }
    }

    bool get_robot_position_wrt_odom()
    {
        try
        {
            geometry_msgs::msg::TransformStamped transformStamped =
                tf_buffer_.lookupTransform(odom_name_, base_link_name_, tf2::TimePointZero);

            robot_x_ = transformStamped.transform.translation.x;
            robot_y_ = transformStamped.transform.translation.y;

            tf2::Quaternion q(
                transformStamped.transform.rotation.x,
                transformStamped.transform.rotation.y,
                transformStamped.transform.rotation.z,
                transformStamped.transform.rotation.w);

            double roll, pitch, yaw;
            tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
            robot_t_ = static_cast<float>(yaw);

            return true;
        }
	        catch (const tf2::TransformException &ex)
	        {
	            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                    "SimpleMove.-> odom TF lookup failed (%s -> %s): %s",
                                    base_link_name_.c_str(), odom_name_.c_str(), ex.what());
	            return false;
	        }
    }

    bool get_goal_position_wrt_odom()
    {
        try
        {
            geometry_msgs::msg::TransformStamped transformStamped =
                tf_buffer_.lookupTransform(odom_name_, base_link_name_, tf2::TimePointZero);

            float robot_x = transformStamped.transform.translation.x;
            float robot_y = transformStamped.transform.translation.y;

            tf2::Quaternion q(
                transformStamped.transform.rotation.x,
                transformStamped.transform.rotation.y,
                transformStamped.transform.rotation.z,
                transformStamped.transform.rotation.w);

            double roll, pitch, yaw;
            tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
            float robot_t = static_cast<float>(yaw);

            goal_t_ = robot_t + goal_angle_;
            if(goal_t_ >   M_PI) goal_t_ -= 2*M_PI;
            if(goal_t_ <= -M_PI) goal_t_ += 2*M_PI;
            
            goal_x_ = robot_x + goal_distance_ * cos(robot_t + goal_angle_ + (move_lat_?M_PI/2:0));
            goal_y_ = robot_y + goal_distance_ * sin(robot_t + goal_angle_ + (move_lat_?M_PI/2:0));

            return true;
        }
        catch (const tf2::TransformException &ex)
        {
            //RCLCPP_WARN(this->get_logger(), "SimpleMove.-> TF Exception: %s", ex.what());
            return false;
        }
    }
    
    float get_path_total_distance(nav_msgs::msg::Path& path)
    {
        float dist = 0;
        for(size_t i=1; i < path.poses.size(); i++)
            dist += sqrt(pow(path.poses[i].pose.position.x - path.poses[i-1].pose.position.x,2) +
                        pow(path.poses[i].pose.position.y - path.poses[i-1].pose.position.y,2));
        return dist;
    }

    bool is_current_head_straight() const
    {
        if (!current_head_pose_received_) {
            return false;
        }

        constexpr double kHeadStraightToleranceRad = 0.12;
        return std::fabs(current_head_pan_) <= kHeadStraightToleranceRad &&
               std::fabs(current_head_tilt_) <= kHeadStraightToleranceRad;
    }

    bool should_wait_for_initial_head_motion() const
    {
        return move_head_ &&
               !camera_bypass_ &&
               head_start_delay_sec_ > 0.0f &&
               head_start_delay_armed_ &&
               is_current_head_straight();
    }

    void fail_path(const std::string & reason)
    {
        // Keep the failure reason in GoalStatus.text so mvn_pln can distinguish
        // a real obstacle from a path-tracking timeout.
        msg_goal_reached.text = reason;
        std::cout << "SimpleMove.-> FAILED path traking. Reason: " << reason << std::endl;
        state = SM_STATE::INIT;
        msg_goal_reached.status = actionlib_msgs::msg::GoalStatus::ABORTED;
        pub_goal_reached_->publish(msg_goal_reached);
        pub_cmd_vel_->publish(geometry_msgs::msg::Twist());
        current_linear_speed = 0;
    }

    void get_next_goal_from_path(size_t& last_pose_idx, size_t& next_pose_idx)
    {
        if(next_pose_idx >= goal_path_.poses.size()) next_pose_idx = goal_path_.poses.size() - 1;
        float prev_goal_x = goal_path_.poses[last_pose_idx].pose.position.x;
        float prev_goal_y = goal_path_.poses[last_pose_idx].pose.position.y;
        float robot_error_x = robot_x_ - prev_goal_x;
        float robot_error_y = robot_y_ - prev_goal_y;
        float error = 0;
        do
        {
            goal_x_ = goal_path_.poses[next_pose_idx].pose.position.x;
            goal_y_ = goal_path_.poses[next_pose_idx].pose.position.y;
            float path_vector_x = goal_x_ - prev_goal_x;
            float path_vector_y = goal_y_ - prev_goal_y;
            float path_vector_mag = sqrt(path_vector_x*path_vector_x + path_vector_y*path_vector_y);
            path_vector_x = path_vector_mag > 0 ? path_vector_x / path_vector_mag : 0;
            path_vector_y = path_vector_mag > 0 ? path_vector_y / path_vector_mag : 0;
            float scalar_projection = robot_error_x*path_vector_x + robot_error_y*path_vector_y;
            //Error is not euclidean distance, but it is measured projected on the current path segment
            error = path_vector_mag - scalar_projection;
            if(error < 0.25)
                last_pose_idx = next_pose_idx;
        }while(error < 0.25 && ++next_pose_idx < goal_path_.poses.size());
    }

    std_msgs::msg::Float64MultiArray get_next_goal_head_angles(size_t next_pose_idx)
    {
        std_msgs::msg::Float64MultiArray msg;

        if (camera_bypass_) {
            // If camera bypass is active, keep the head fixed at a neutral position
            msg.data.push_back(0.0); // pan
            msg.data.push_back(0.0); // tilt
            return msg;
        }
        
        size_t idx = next_pose_idx + 5 >=  goal_path_.poses.size() - 1 ? goal_path_.poses.size() - 1 : next_pose_idx + 5;
        float goal_x = goal_path_.poses[idx].pose.position.x;
        float goal_y = goal_path_.poses[idx].pose.position.y;
        float angle = atan2(goal_y - robot_y_, goal_x - robot_x_) - robot_t_;
        if(angle >   M_PI) angle -= 2*M_PI;
        if(angle <= -M_PI) angle += 2*M_PI;

        if (should_scan_head_for_unknown_space()) {
            // Keep the path direction as the center of attention and sweep pan
            // around it, so the robot still looks generally where it is going.
            constexpr double TWO_PI = 6.28318530717958647692;
            const double period = std::max(0.5f, head_unknown_scan_period_);
            const double phase = std::fmod(this->now().seconds(), period) / period;
            const double scan_offset =
                head_unknown_scan_amplitude_ * std::sin(TWO_PI * phase);
            angle += static_cast<float>(scan_offset);
        }

        const float pan_limit = std::max(0.0f, head_pan_limit_);
        angle = std::clamp(angle, -pan_limit, pan_limit);
        msg.data.push_back(angle);
        msg.data.push_back(head_tilt_);
        return msg;
    }


    //############
    //Simple Move main processing
    void simple_move_processing() 
    {
        try
        {
            if(stop_)
            {
                stop_ = false;
                state = SM_STATE::INIT;
                collision_risk_ = false;
                msg_goal_reached.status = actionlib_msgs::msg::GoalStatus::ABORTED;
                msg_goal_reached.text = "stop_received";
                pub_cmd_vel_->publish(geometry_msgs::msg::Twist());
                pub_goal_reached_->publish(msg_goal_reached);
            }
            if(new_pose_ || new_path_)
                state = SM_STATE::WAITING_FOR_TASK;

            switch(state)
            {
            case SM_STATE::INIT:
                std::cout << "SimpleMove.-> Low level control ready. Waiting for new goal. " << std::endl;
                collision_risk_ = false;
                state = SM_STATE::WAITING_FOR_TASK;
                break;
            
            
            case SM_STATE::WAITING_FOR_TASK:
                if(new_pose_)
                {
                    collision_risk_ = false;
                    get_goal_position_wrt_odom();
                    state = SM_STATE::GOAL_POSE_ACCEL;
                    new_pose_ = false;
                    msg_goal_reached.goal_id.id = "-1";
                    msg_goal_reached.text.clear();
                    attempts = (int)((fabs(goal_distance_)*5)/max_linear_speed_*RATE + fabs(goal_angle_*5)/max_angular_speed_*RATE + RATE*5);
                }
                if(new_path_)
                {
                    collision_risk_ = false;
                    state = SM_STATE::GOAL_PATH_ACCEL;
                    new_path_ = false;
                    prev_pose_idx = 0;
                    next_pose_idx = 0;
                    global_goal_x_ = goal_path_.poses[goal_path_.poses.size() - 1].pose.position.x;
                    global_goal_y_ = goal_path_.poses[goal_path_.poses.size() - 1].pose.position.y;
                    std::stringstream ss;
                    ss << goal_path_.header.stamp.sec;
                    ss >> msg_goal_reached.goal_id.id;
                    const float path_total_distance = get_path_total_distance(goal_path_);
                    attempts = (int)(path_total_distance/max_linear_speed_*4*RATE + 5*RATE);
                    msg_goal_reached.text.clear();
                    RCLCPP_INFO(
                        this->get_logger(),
                        "SimpleMove.-> Path tracking budget: distance=%.2f m max_speed=%.2f m/s attempts=%d timeout=%.1f s",
                        path_total_distance,
                        max_linear_speed_,
                        attempts,
                        static_cast<double>(attempts) / RATE);

                    if (should_wait_for_initial_head_motion())
                    {
                        if (!get_robot_position_wrt_map()) {
                            break;
                        }
                        get_next_goal_from_path(prev_pose_idx, next_pose_idx);
                        pub_head_goal_pose_->publish(get_next_goal_head_angles(next_pose_idx));
                        pub_cmd_vel_->publish(geometry_msgs::msg::Twist());
                        head_start_delay_start_ = this->now();
                        head_start_delay_armed_ = false;
                        state = SM_STATE::WAIT_FOR_HEAD_START_DELAY;
                        RCLCPP_INFO(
                            this->get_logger(),
                            "SimpleMove.-> Head was straight. Waiting %.2f sec after initial head command before moving.",
                            head_start_delay_sec_);
                    }
                }
                break;

            case SM_STATE::WAIT_FOR_HEAD_START_DELAY:
                if ((this->now() - head_start_delay_start_).seconds() >= head_start_delay_sec_)
                {
                    state = SM_STATE::GOAL_PATH_ACCEL;
                }
                else
                {
                    pub_cmd_vel_->publish(geometry_msgs::msg::Twist());
                }
                break;

            
            case SM_STATE::GOAL_POSE_ACCEL:
                get_robot_position_wrt_odom();
                global_error = sqrt((goal_x_ - robot_x_)*(goal_x_ - robot_x_) + (goal_y_ - robot_y_)*(goal_y_ - robot_y_));

                if(global_error < fine_dist_tolerance_)
                    state = SM_STATE::GOAL_POSE_CORRECT_ANGLE;
                else if(global_error < current_linear_speed*current_linear_speed/linear_acceleration_)
                {
                    state = SM_STATE::GOAL_POSE_DECCEL;
                    temp_k = current_linear_speed/sqrt(global_error);
                }
                else if(current_linear_speed >= max_linear_speed_)
                {
                    state = SM_STATE::GOAL_POSE_CRUISE;
                    current_linear_speed = max_linear_speed_;
                }
                if(--attempts <= 0)
                {
                    state = SM_STATE::GOAL_POSE_FAILED;
                    std::cout << "SimpleMove.-> Timeout exceeded while trying to reach goal position. Current state: GOAL_POSE_ACCEL." << std::endl;
                }
	                {
                        auto cmd = calculate_speeds(robot_x_, robot_y_, robot_t_, goal_x_, goal_y_, 
	                                                min_linear_speed_, current_linear_speed, max_angular_speed_, 
	                                                alpha_*2, beta_/4, goal_distance_ < 0, move_lat_);
	                    pub_cmd_vel_->publish(cmd);
                    }
	                current_linear_speed += linear_acceleration_/RATE;
	                break;
                
            case SM_STATE::GOAL_POSE_CRUISE:
                get_robot_position_wrt_odom();
                global_error = sqrt((goal_x_ - robot_x_)*(goal_x_ - robot_x_) + (goal_y_ - robot_y_)*(goal_y_ - robot_y_));
                if(global_error < fine_dist_tolerance_)
                    state = SM_STATE::GOAL_POSE_CORRECT_ANGLE;
                else if(global_error < current_linear_speed*current_linear_speed/linear_acceleration_)
                {
                    state = SM_STATE::GOAL_POSE_DECCEL;
                    temp_k = current_linear_speed/sqrt(global_error);
                }
                if(--attempts <= 0)
                {
                    state = SM_STATE::GOAL_POSE_FAILED;
                    std::cout << "SimpleMove.-> Timeout exceeded while trying to reach goal position. Current state: GOAL_POSE_CRUISE." << std::endl;
                }
	                {
                        auto cmd = calculate_speeds(robot_x_, robot_y_, robot_t_, goal_x_, goal_y_, 
	                                                min_linear_speed_, current_linear_speed, max_angular_speed_,
	                                                alpha_*2, beta_/4, goal_distance_ < 0, move_lat_);
	                    pub_cmd_vel_->publish(cmd);
                    }
	                break;

            case SM_STATE::GOAL_POSE_DECCEL:
                get_robot_position_wrt_odom();
                global_error = sqrt((goal_x_ - robot_x_)*(goal_x_ - robot_x_) + (goal_y_ - robot_y_)*(goal_y_ - robot_y_));

                if(global_error < fine_dist_tolerance_)
                        state = SM_STATE::GOAL_POSE_CORRECT_ANGLE;
                if(--attempts <= 0)
                {
                    state = SM_STATE::GOAL_POSE_FAILED;
                    std::cout << "SimpleMove.-> Timeout exceeded while trying to reach goal position. Current state: GOAL_POSE_DECCEL." << std::endl;
                }
                current_linear_speed = temp_k * sqrt(global_error);
                if(current_linear_speed < min_linear_speed_) current_linear_speed = min_linear_speed_;
	                {
                        auto cmd = calculate_speeds(robot_x_, robot_y_, robot_t_, goal_x_, goal_y_, 
	                                                min_linear_speed_, current_linear_speed, max_angular_speed_,
	                                                alpha_*2, beta_/4, goal_distance_ < 0, move_lat_,
	                                                should_apply_potential_fields(), rejection_force_.y);
	                    pub_cmd_vel_->publish(cmd);
                    }
	                break;

            case SM_STATE::GOAL_POSE_CORRECT_ANGLE:
                get_robot_position_wrt_odom();
                global_error = (goal_t_ - robot_t_);
                if (global_error > M_PI) global_error-=2*M_PI;
                if (global_error <= -M_PI) global_error+=2*M_PI;
                global_error = fabs(global_error);
                if(global_error < angle_tolerance_)
                    state = SM_STATE::GOAL_POSE_FINISH;
	                {
                        auto cmd = calculate_speeds(robot_t_, goal_t_, final_angle_max_angular_speed_, beta_/4);
	                    pub_cmd_vel_->publish(cmd);
                    }
                if(--attempts <= 0)
                {
                    state = SM_STATE::GOAL_POSE_FAILED;
                    std::cout << "SimpleMove.-> Timeout exceeded while trying to reach goal position. Current state: GOAL_POSE_CORRECT_ANGLE." << std::endl;
                }   
                break;

            case SM_STATE::GOAL_POSE_FINISH:
                std::cout << "SimpleMove.-> Successful move with dist=" << goal_distance_ << " angle=" << goal_angle_ << std::endl;
                state = SM_STATE::INIT;
                msg_goal_reached.status = actionlib_msgs::msg::GoalStatus::SUCCEEDED;
                pub_goal_reached_->publish(msg_goal_reached);
                pub_cmd_vel_->publish(geometry_msgs::msg::Twist());
                current_linear_speed = 0;
                break;

            case SM_STATE::GOAL_POSE_FAILED:
                std::cout << "SimpleMove.-> FAILED move with dist=" << goal_distance_ << " angle=" << goal_angle_ << std::endl;
                state = SM_STATE::INIT;
                msg_goal_reached.status = actionlib_msgs::msg::GoalStatus::ABORTED;
                msg_goal_reached.text = "pose_failed";
                pub_goal_reached_->publish(msg_goal_reached);
                pub_cmd_vel_->publish(geometry_msgs::msg::Twist());
                current_linear_speed = 0;
                break;

            case SM_STATE::GOAL_PATH_ACCEL:
                if(collision_risk_)
                {
                    if (!try_rotate_in_place_on_collision("GOAL_PATH_ACCEL"))
                    {
                        state = SM_STATE::GOAL_PATH_FAILED;
                        msg_goal_reached.text = "collision_risk";
                        pub_cmd_vel_->publish(geometry_msgs::msg::Twist());
                        std::cout << "SimpleMove.-> WARNING! Collision risk detected!!!!!" << std::endl;
                    }
                }
                else{
	                    get_robot_position_wrt_map();
	                    get_next_goal_from_path(prev_pose_idx, next_pose_idx);
	                    global_error = sqrt((global_goal_x_ - robot_x_)*(global_goal_x_ - robot_x_) + (global_goal_y_ - robot_y_)*(global_goal_y_- robot_y_));
                        const float path_speed_cap = get_path_max_linear_speed_cap();
                        clamp_current_linear_speed_to(path_speed_cap);

	                    if(global_error < coarse_dist_tolerance_)
                    {
                        state = SM_STATE::GOAL_PATH_FINISH;
                    }
                    else if(global_error < current_linear_speed*current_linear_speed/linear_acceleration_)
                    {
                        state = SM_STATE::GOAL_PATH_DECCEL;
                        temp_k = current_linear_speed/sqrt(global_error);
                    }
	                    else if(current_linear_speed >= path_speed_cap)
	                    {
	                        state = SM_STATE::GOAL_PATH_CRUISE;
	                        current_linear_speed = path_speed_cap;
                    }
                    if(--attempts <= 0)
                    {
                        state = SM_STATE::GOAL_PATH_FAILED;
                        msg_goal_reached.text = "timeout_path_accel";
                        std::cout<<"SimpleMove.-> Timeout exceeded while trying to reach goal path. Current state: GOAL_PATH_ACCEL."<<std::endl;
                    }
	                    auto cmd = calculate_speeds(robot_x_, robot_y_, robot_t_, goal_x_, goal_y_, 
	                                                min_linear_speed_, current_linear_speed, max_angular_speed_,
	                                                alpha_, beta_, false, move_lat_,
	                                                should_apply_potential_fields(), rejection_force_.y);
	                    pub_cmd_vel_->publish(cmd);
                    if(move_head_ && !camera_bypass_) {
                        pub_head_goal_pose_->publish(get_next_goal_head_angles(next_pose_idx));
                    }

	                    current_linear_speed += linear_acceleration_/RATE;
                        if(current_linear_speed > path_speed_cap) current_linear_speed = path_speed_cap;
	                }
                break;

            case SM_STATE::GOAL_PATH_CRUISE:
                if(collision_risk_)
                {
                    if (!try_rotate_in_place_on_collision("GOAL_PATH_CRUISE"))
                    {
                        state = SM_STATE::GOAL_PATH_FAILED;
                        msg_goal_reached.text = "collision_risk";
                        pub_cmd_vel_->publish(geometry_msgs::msg::Twist());
                        std::cout << "SimpleMove.-> WARNING! Collision risk detected!!!!!" << std::endl;
                    }
                }
                else
                {
	                    get_robot_position_wrt_map();
	                    get_next_goal_from_path(prev_pose_idx, next_pose_idx);
	                    global_error = sqrt((global_goal_x_ - robot_x_)*(global_goal_x_ - robot_x_) + (global_goal_y_ - robot_y_)*(global_goal_y_ - robot_y_));
                        const float path_speed_cap = get_path_max_linear_speed_cap();
                        clamp_current_linear_speed_to(path_speed_cap);
                        if(current_linear_speed < path_speed_cap) {
                            current_linear_speed = std::min(path_speed_cap, current_linear_speed + linear_acceleration_/RATE);
                        }
	                    if(global_error < coarse_dist_tolerance_)
	                        state = SM_STATE::GOAL_PATH_FINISH;
                    else if(global_error < current_linear_speed*current_linear_speed/linear_acceleration_)
                    {
                        state = SM_STATE::GOAL_PATH_DECCEL;
                        temp_k = current_linear_speed/sqrt(global_error);
                    }
                    if(--attempts <= 0)
                    {
                        state = SM_STATE::GOAL_PATH_FAILED;
                        msg_goal_reached.text = "timeout_path_cruise";
                        std::cout<<"SimpleMove.-> Timeout exceeded while trying to reach goal path. Current state: GOAL_PATH_CRUISE."<<std::endl;
                    }
	                    auto cmd = calculate_speeds(robot_x_, robot_y_, robot_t_, goal_x_, goal_y_, 
	                                                min_linear_speed_, current_linear_speed, max_angular_speed_, 
	                                                alpha_, beta_, false, move_lat_,
	                                                should_apply_potential_fields(), rejection_force_.y);
	                    pub_cmd_vel_->publish(cmd);
                    if(move_head_ && !camera_bypass_) {
                        pub_head_goal_pose_->publish(get_next_goal_head_angles(next_pose_idx));
                    }
                }
                break;

            case SM_STATE::GOAL_PATH_DECCEL:
                if(collision_risk_)
                {
                    if (!try_rotate_in_place_on_collision("GOAL_PATH_DECCEL"))
                    {
                        state = SM_STATE::GOAL_PATH_FAILED;
                        msg_goal_reached.text = "collision_risk";
                        pub_cmd_vel_->publish(geometry_msgs::msg::Twist());
                        std::cout << "SimpleMove.-> WARNING! Collision risk detected!!!!!" << std::endl;
                    }
                }
                else
                {
	                    get_robot_position_wrt_map();
	                    get_next_goal_from_path(prev_pose_idx, next_pose_idx);
	                    global_error = sqrt((global_goal_x_ - robot_x_)*(global_goal_x_ - robot_x_) + (global_goal_y_ - robot_y_)*(global_goal_y_ - robot_y_));
                        const float path_speed_cap = get_path_max_linear_speed_cap();
	                    if(global_error < coarse_dist_tolerance_)
	                        state = SM_STATE::GOAL_PATH_FINISH;
                    if(--attempts <= 0)
                    {
                        state = SM_STATE::GOAL_PATH_FAILED;
                        msg_goal_reached.text = "timeout_path_deccel";
                        std::cout<<"SimpleMove.-> Timeout exceeded while trying to reach goal path. Current state:GOAL_PATH_DECCEL."<<std::endl;
	                    }
	                    current_linear_speed = temp_k * sqrt(global_error);
                        if(current_linear_speed > path_speed_cap) current_linear_speed = path_speed_cap;
	                    if(current_linear_speed < min_linear_speed_) current_linear_speed = min_linear_speed_;
	                    auto cmd = calculate_speeds(robot_x_, robot_y_, robot_t_, goal_x_, goal_y_, 
	                                                min_linear_speed_, current_linear_speed, max_angular_speed_,
	                                                alpha_, beta_, false, move_lat_,
	                                                should_apply_potential_fields(), rejection_force_.y);
	                    pub_cmd_vel_->publish(cmd);
                    if(move_head_ && !camera_bypass_) {
                        pub_head_goal_pose_->publish(get_next_goal_head_angles(next_pose_idx));
                    }
                }
                break;

            case SM_STATE::GOAL_PATH_FINISH:
                std::cout << "SimpleMove.-> Path succesfully followed." << std::endl;
                state = SM_STATE::INIT;
                msg_goal_reached.status = actionlib_msgs::msg::GoalStatus::SUCCEEDED;
                msg_goal_reached.text = "path_reached";
                pub_goal_reached_->publish(msg_goal_reached);
                pub_cmd_vel_->publish(geometry_msgs::msg::Twist());
                current_linear_speed = 0;
                break;

            case SM_STATE::GOAL_PATH_FAILED:
                if (msg_goal_reached.text.empty())
                    msg_goal_reached.text = "path_failed_unknown";
                fail_path(msg_goal_reached.text);
                break;

            default:
                std::cout<<"SimpleMove.-> A VERY STUPID PERSON PROGRAMMED THIS SHIT. APOLOGIES FOR THE INCONVINIENCE :'(" << std::endl;
                return;
            }

        } 
        catch (const std::exception& e) 
        {
            RCLCPP_ERROR(this->get_logger(), "SimpleMove.-> Error in SimpleMove processing: %s", e.what());
        }
    }

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleMoveNode>();
    //rclcpp::spin(node);
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    
    return 0;
}
