// ----- Navigation Library ----- 
#include <navigation_manager/navigation.hpp>

// ----- ROS 2 -----
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>

// ----- Services -----
#include <known_locations_interfaces/srv/locations_get.hpp>
#include <std_srvs/srv/trigger.hpp>

// ----- Messages -----
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <actionlib_msgs/msg/goal_status.hpp>

// ----- Actions -----
#include <rclcpp_action/rclcpp_action.hpp>

// ----- TF2 -----
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>

// ----- C++ -----
#include <limits>

namespace navigation_manager
{
  class NavigationNode final : public rclcpp::Node, public PlannerInterface, public MotionInterface
  {
    using Handle    = rclcpp_action::ServerGoalHandle<Action>;
    using Locations = known_locations_interfaces::srv::LocationsGet;

    public:
      NavigationNode() : Node("navigation_manager"), tf_(get_clock()), listener_(tf_)
      {
        Settings s;
        rcl_interfaces::msg::ParameterDescriptor startup;
        startup.read_only = true;
        auto number = [&](const std::string & name, double value)
        {
          value = declare_parameter<double>(name, value, startup);
          if (!std::isfinite(value) || value <= 0.)
          {
            throw std::invalid_argument("Invalid " + name);
          }
          return value;
        };

        s.map_frame           = declare_parameter<std::string>("map_frame", "map", startup);
        base_                 = declare_parameter<std::string>("base_link_name", "base_link", startup);
        odom_                 = declare_parameter<std::string>("odom_name", "odom", startup);
        const bool namespaced = declare_parameter<bool>("use_namespace", false, startup);
      
        prefix_               = namespaced ? get_namespace() : "";
        if (prefix_ == "/")
        {
          prefix_.clear();
        }

        s.allow_unknown     = declare_parameter<bool>("allow_unknown", false, startup);
        
        s.position_tolerance  = number("goal_position_tolerance_m", s.position_tolerance);
        s.angle_tolerance     = number("goal_angle_tolerance_rad", s.angle_tolerance);
        
        s.capture_progress    = number("near_goal_min_progress",s.capture_progress);
        if (s.capture_progress>1.)
        {
          throw std::invalid_argument("Invalid near_goal_min_progress");
        }
      
        s.robot_radius            = number("robot_radius", s.robot_radius);
        s.approximate_radius      = s.robot_radius + number("near_goal_extra_margin", .10);
        s.plan_timeout            = number("plan_request_timeout_sec", s.plan_timeout);
        s.controller_timeout      = number("controller_timeout_sec", s.controller_timeout);
        s.map_timeout             = number("map_timeout_sec", s.map_timeout);
        s.tf_timeout              = number("tf_timeout_sec", s.tf_timeout);
        s.sensor_timeout          = number("sensor_timeout_sec", s.sensor_timeout);
        s.sensor_wait             = number("sensor_wait_timeout_sec", s.sensor_wait);
        s.replan_period           = number("replan_period_sec", s.replan_period);
        s.stop_timeout            = number("stop_timeout_sec", s.stop_timeout);
        s.execution_timeout       = number("execution_timeout_sec", s.execution_timeout);
        s.progress_distance       = number("recovery_progress_m", s.progress_distance);
        s.no_progress_timeout     = number("no_progress_timeout_sec", s.no_progress_timeout);
        s.escape_radius           = number("start_escape_radius", s.escape_radius);
        s.path_horizon            = number("path_check_horizon_m", s.path_horizon);
        s.local_avoidance_timeout = number("local_avoidance_timeout_sec", s.local_avoidance_timeout);
        
        settings_ = s;
        
        // Create the navigation manager and configure recovery strategies and budgets.
        manager_ = std::make_unique<NavigationManager>(s, *this, *this);
        for (auto & strategy : manager_->recoveries.strategies)
        {
          const auto key = "recovery." + strategy->name;
          int attempts = declare_parameter<int>(key + ".attempts", strategy->budget.attempts, startup);
          
          if (attempts < 0 || attempts > 1000)
          {
            throw std::invalid_argument("Invalid " + key);
          }
          strategy->budget.attempts = attempts;
          strategy->budget.seconds  = number(key + ".timeout_sec", strategy->budget.seconds);
        }

        // Override budgets for a concrete condition without changing the coordinator.
        for (const auto & reason: {"start_blocked",
                                   "goal_blocked", 
                                   "no_path", 
                                   "no_progress", 
                                   "plan_request_timeout",
                                   "start_escape"})
        {
          const std::string key = "recovery.condition." + std::string(reason);
          int attempts = declare_parameter<int>(key + ".attempts", 6, startup);
          
          if (attempts < 0 || attempts > 1000)
          {
            throw std::invalid_argument("Invalid " + key);
          }
          manager_->recoveries.condition_budgets[reason] = {unsigned(attempts), number(key + ".timeout_sec", 40.)};
        }
        
        // Create ROS2 services clients
        planner_      = create_client<Plan>(prefix_ + "/path_planner/plan_navigation");
        locations_    = create_client<Locations>( declare_parameter<std::string>("locations_service","/locations_manager/get", startup));
        stop_client_  = create_client<std_srvs::srv::Trigger>(prefix_ + "/simple_move/stop_navigation");
      
        // Create ROS2 publishers
        head_ = std::make_shared<rclcpp::AsyncParametersClient>(this, declare_parameter<std::string>("motion_node", prefix_ + "/simple_move", startup));
        path_pub_   = create_publisher<Command>(prefix_ + "/simple_move/navigation_path", 10);
        visual_pub_ = create_publisher<Path>(prefix_ + "/path", rclcpp::QoS(1).transient_local());
        stop_pub_   = create_publisher<std_msgs::msg::Empty>(prefix_ + "/simple_move/stop", 10);
        enable_pub_ = create_publisher<std_msgs::msg::Bool>(prefix_ + "/potential_fields/enable", rclcpp::QoS(1).transient_local());
        status_pub_ = create_publisher<actionlib_msgs::msg::GoalStatus>(prefix_ + "/status", rclcpp::QoS(10).transient_local());
      
        // Create ROS2 Action server
        server_ = rclcpp_action::create_server<Action>
        (
          this, 
          "navigate",                                                 // Action name
          [](const rclcpp_action::GoalUUID &,                         // Goal callback
          std::shared_ptr<const Action::Goal> goal)                   // Goal request
          {
            if(goal->target_type != Action::Goal::POSE &&            
               goal->target_type != Action::Goal::LOCATION)
            {
              return rclcpp_action::GoalResponse::REJECT;             // Reject invalid goal types
              RCLCPP_WARN(rclcpp::get_logger("navigation_manager"), "Invalid goal type: %d", goal->target_type);
            }
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            RCLCPP_INFO(rclcpp::get_logger("navigation_manager"), "Accepted goal type: %d", goal->target_type);
          },
          [this](std::shared_ptr<Handle> h)                           // Cancel callback
          {
            if(pending_handle_ == h)
            {
              // Complete after the action server has committed its cancel transition.
              pending_cancel_ = true;
            } else if (active_handle_ == h) {
              active_cancel_ = true;
            }
            return rclcpp_action::CancelResponse::ACCEPT;
          },                                                          // Accepted callback
          [this](std::shared_ptr<Handle> h)
          {
            queue(*h->get_goal(),h);
          }
        );

        // This subscription is for rviz goals
        legacy_sub_ = create_subscription<Pose>
        (
          "/nav_control/goal",                        // Topic name
          10,                                         // QoS depth
          [this](Pose::ConstSharedPtr pose)           // Callback function
          {
            Action::Goal goal;
            goal.target_type = Action::Goal::POSE;
            goal.pose = *pose;
            goal.move_head = get_parameter("legacy_move_head").as_bool();
            queue(goal, nullptr);
          }
        );

        declare_parameter<bool>("legacy_move_head", true);
        declare_parameter<bool>("debug_detection", false);

        // Subscription for goal status rached
        result_sub_ = create_subscription<actionlib_msgs::msg::GoalStatus>(
          prefix_ + "/simple_move/goal_reached",
          10,
          [this](actionlib_msgs::msg::GoalStatus::ConstSharedPtr status)
          {
            manager_->motion_status(status->goal_id.id, status->status, status->text, Clock::now());
          });

        // Subscription for stop command from bumper (only for HSR Robots... TODO: make this more generic (e.g. use with Justina))
        stop_sub_ = create_subscription<std_msgs::msg::Empty>(
          prefix_ + "/stop",
          10,
          [this](std_msgs::msg::Empty::ConstSharedPtr)
          {
            if (pending_)
            {
              reject_pending("cancelled", Action::Result::CANCELLED);
            }
            manager_->cancel(false, Clock::now());
          });

        // Subscriptions for map, clearance map, uncertain obstacles, potential fields ready, and blocking obstacle
        map_sub_ = create_subscription<Grid>(
          prefix_ + "/augmented_map",
          rclcpp::QoS(1).transient_local(),
          [this](Grid::ConstSharedPtr grid)
          {
            if (!fresh(grid->header.stamp, settings_.map_timeout))
            {
              return;
            }
            manager_->monitor.accept_map(grid, Clock::now());
          });
      
        clearance_sub_ = create_subscription<Grid>(
          prefix_ + "/clearance_map",
          rclcpp::QoS(1).transient_local(),
          [this](Grid::ConstSharedPtr grid)
          {
            if (fresh(grid->header.stamp, settings_.map_timeout))
            {
              manager_->monitor.clearance = grid;
            }
          });

        uncertain_sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
          prefix_ + "/uncertain_obstacles",
          rclcpp::QoS(1).transient_local(),
          [this](geometry_msgs::msg::PoseArray::ConstSharedPtr points)
          {
            if (points->header.frame_id != settings_.map_frame)
            {
              return;
            }
            for (const auto & p : points->poses)
            {
              if (!std::isfinite(p.position.x + p.position.y))
              {
                return;
              }
            }
            manager_->monitor.uncertain = *points;
          });

        ready_sub_ = create_subscription<std_msgs::msg::Bool>(
          prefix_ + "/potential_fields/ready",
          rclcpp::SensorDataQoS(),
          [this](std_msgs::msg::Bool::ConstSharedPtr value)
          {
            manager_->monitor.sensors_ready = value->data;
            manager_->monitor.sensors_received = Clock::now();
          });

        witness_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
          prefix_ + "/potential_fields/blocking_obstacle",
          10,
          [this](geometry_msgs::msg::PointStamped::ConstSharedPtr point)
          {
            try
            {
              auto p = tf_.transform(*point, settings_.map_frame, tf2::durationFromSec(0.));
              if (!fresh(p.header.stamp, 4.) || !std::isfinite(p.point.x + p.point.y))
              {
                return;
              }
              auto & witnesses = manager_->monitor.witnesses;
              for (auto & old:witnesses)
              {
                if (std::hypot(old.point.x - p.point.x, old.point.y - p.point.y) < .1)
                {
                  old = p;
                  return;
                }
              }
              if (witnesses.size() == 16)
              {
                witnesses.erase(witnesses.begin());
              }
              witnesses.push_back(std::move(p));
            }
            catch (const tf2::TransformException &)
            {
              RCLCPP_WARN(get_logger(), "Failed to transform witness point to map frame");
            }
          });

        manager_->completed = [this](uint64_t id, const Action::Result & value)
        {
          if (id != sequence_)
          {
            return;
          }
          if (active_handle_)
          {
            auto result = std::make_shared<Action::Result>(value);
            if (value.result_code == Action::Result::CANCELLED && active_handle_->is_canceling())
            {
              active_handle_->canceled(result);
            }
            else if (value.result_code == Action::Result::SUCCESS ||
                     value.result_code == Action::Result::APPROXIMATE)
            {
              active_handle_->succeed(result);
            }
            else
            {
              active_handle_->abort(result);
            }
            active_handle_.reset();
          }
          if (value.reason == "stop_unverified")
          {
            unsafe_stop_ = true;
            if (pending_)
            {
              reject_pending("stop_unverified", Action::Result::FAILED);
            }
          }
          publish_legacy(
            value.result_code == Action::Result::SUCCESS ||
            value.result_code == Action::Result::APPROXIMATE ?
            actionlib_msgs::msg::GoalStatus::SUCCEEDED : actionlib_msgs::msg::GoalStatus::ABORTED,
            value.reason);

          if (location_request_)
          {
            locations_->remove_pending_request(*location_request_);
            location_request_.reset();
          }
          if (stop_request_)
          {
            stop_client_->remove_pending_request(*stop_request_);
            stop_request_.reset();
          }
          Path empty;
          empty.header.frame_id = settings_.map_frame;
          visual_pub_->publish(empty);

        };
        timer_ = create_wall_timer(std::chrono::milliseconds(50), [this] {tick();});
      }

    private:
      bool fresh(const builtin_interfaces::msg::Time & stamp, double timeout) const
      {
        auto time = rclcpp::Time(stamp, get_clock()->get_clock_type());
        return time.nanoseconds() != 0 && std::abs((now() - time).seconds()) <= timeout;
      }
    
      void reject_pending(const std::string & reason, uint8_t code)
      {
        if (pending_handle_)
        {
          auto result = std::make_shared<Action::Result>();
          result->result_code       = code;
          result->reason            = reason;
          result->result_message    = "[" + reason + "] Goal closed before activation";
          result->position_error_m  = result->orientation_error_rad = std::numeric_limits<float>::quiet_NaN();
          if (pending_handle_->is_canceling())
          {
            pending_handle_->canceled(result);
          }
          else
          {
            pending_handle_->abort(result);
          }
        }
        pending_handle_.reset();
        pending_.reset();
        pending_cancel_ = false;
      }

      void queue(const Action::Goal & goal, std::shared_ptr<Handle> handle)
      {
        if (pending_)
        {
          reject_pending("preempted", Action::Result::PREEMPTED);
        }
        pending_ = goal;
        pending_handle_ = std::move(handle);
        const auto c = manager_->context();
        if (c && c->state != State::Finished)
        {
          manager_->cancel(true, Clock::now());
        }
      }

      void resolve(Pose pose, uint64_t id)
      {
        try
        {
          if (pose.header.frame_id.empty())
          {
            pose.header.frame_id = settings_.map_frame;
          }
          if (pose.header.frame_id != settings_.map_frame)
          {
            pose = tf_.transform(pose, settings_.map_frame, tf2::durationFromSec(0.));
          }
          manager_->resolved(id, std::move(pose), "", Clock::now());
        }
        catch (const tf2::TransformException &)
        {
          manager_->resolved(id, Pose{}, "goal_tf_unavailable", Clock::now());
        }
      }

      void tick()
      {
        const auto tick = Clock::now();
        if (pending_cancel_ && pending_)
        {
          reject_pending("cancelled", Action::Result::CANCELLED);
        }
        if (active_cancel_)
        {
          active_cancel_ = false;
          manager_->cancel(false, tick);
        }
        auto c = manager_->context();
        if (pending_ && (!c || c->state == State::Finished))
        {
          if (unsafe_stop_)
          {
            reject_pending("stop_unverified", Action::Result::FAILED);
            return;
          }
          active_handle_ = std::move(pending_handle_);
          auto goal = std::move(*pending_);
          pending_.reset();
          manager_->monitor.witnesses.clear();
          odometry_ = {};
          manager_->start(++sequence_, goal, tick);
          if (goal.target_type == Action::Goal::POSE)
          {
            resolve(goal.pose, sequence_);
          }
          else if (!locations_->service_is_ready())
          {
            manager_->resolved(sequence_, Pose{}, "locations_unavailable", tick);
          }
          else
          {
            auto request = std::make_shared<Locations::Request>();
            request->location_name = goal.location_name;
            const auto id = sequence_;
            auto future = locations_->async_send_request(
              request,
              [this, id](rclcpp::Client<Locations>::SharedFuture f)
              {
                if (id != sequence_     || 
                  !manager_->context()  ||
                  manager_->context()->state == State::Resolving)
                {
                  return;
                }
                
                location_request_.reset();
                try
                {
                  auto result = f.get();
                  Pose pose;
                  pose.header.frame_id = settings_.map_frame;
                  pose.pose = result->pose;
                  if (result->success)
                  {
                    resolve(std::move(pose), id);
                  }
                  else
                  {
                    manager_->resolved(id, Pose{}, "unknown_location", Clock::now());
                  }
                }
                catch (const std::exception &)
                {
                  manager_->resolved(id, Pose{}, "location_resolution_failed", Clock::now());
                }
              });
            location_request_ = future.request_id;
          }
        }
        c = manager_->context();
        if (!c || c->state == State::Finished)
        {
          return;
        }
        
        Pose pose;
        try
        {
          auto tf = tf_.lookupTransform(settings_.map_frame, base_, tf2::TimePointZero);
          if (!fresh(tf.header.stamp, settings_.tf_timeout))
          {
            throw std::runtime_error("stale pose");
          }
          pose.header = tf.header;
          pose.pose.position.x = tf.transform.translation.x;
          pose.pose.position.y = tf.transform.translation.y;
          pose.pose.orientation = tf.transform.rotation;
          auto odom = tf_.lookupTransform(odom_, base_, tf2::TimePointZero);
          if (!fresh(odom.header.stamp, settings_.tf_timeout))
          {
            throw std::runtime_error("stale odometry");
          }

          odometry_.observe(
            rclcpp::Time(odom.header.stamp).nanoseconds(),
            odom.transform.translation.x,
            odom.transform.translation.y,
            tf2::getYaw(odom.transform.rotation));

        }
        catch (const std::exception &)
        {
          pose.header.frame_id.clear();
          odometry_ = {};
        }
        
        manager_->update_pose(std::move(pose), odometry_, tick);
        auto & witnesses = manager_->monitor.witnesses;
        witnesses.erase(
          std::remove_if(
            witnesses.begin(),
            witnesses.end(),
            [this](const auto & p)
            {
              return !fresh(p.header.stamp, 4.);
            }),
          witnesses.end());

        // Map age also follows ROS time, including jumps and paused simulation.
        if (manager_->monitor.map &&
          !fresh(manager_->monitor.map->header.stamp, settings_.map_timeout))
        {
          manager_->monitor.map_received = {};
        }

        manager_->tick(tick);
        c = manager_->context();
        if (c->state != State::Finished && tick - last_feedback_ >= std::chrono::milliseconds(200))
        {
          last_feedback_ = tick;
          enable_sensors(true);
          if (active_handle_)
          {
            auto f = std::make_shared<Action::Feedback>();
            f->status_code = static_cast<uint8_t>(c->state);
            f->status_message = "[" + c->diagnostic + "] " + c->diagnostic;
            f->distance_remaining_m = c->pose_valid && c->resolved ? std::hypot(
              c->robot.pose.position.x - c->original.pose.position.x,
              c->robot.pose.position.y -
              c->original.pose.position.y) : std::numeric_limits<float>::quiet_NaN();
            f->total_recoveries = c->total_recoveries;
            active_handle_->publish_feedback(f);
          }
          publish_legacy(actionlib_msgs::msg::GoalStatus::ACTIVE, c->diagnostic);
        }
      }

      void publish_legacy(uint8_t status, const std::string & reason)
      {
        actionlib_msgs::msg::GoalStatus value;
        value.goal_id.id = std::to_string(sequence_);
        value.goal_id.stamp = now();
        value.status = status;
        value.text = reason;
        status_pub_->publish(value);
        if (get_parameter("debug_detection").as_bool() &&
          (reason != last_diagnostic_ || status != last_status_)) {
          const auto c = manager_->context();
          RCLCPP_INFO(
            get_logger(), "NavigationDiagnostic goal=%lu motion=%s reason=%s state=%u recoveries=%u pose_valid=%d",
            static_cast<unsigned long>(sequence_), c ? c->motion_id.c_str() : "",
            reason.c_str(), status,
            c ? c->total_recoveries : 0, c ? c->pose_valid : false);
        }
        last_diagnostic_ = reason;
        last_status_ = status;
      }

      void plan(Plan::Request request, uint64_t goal, uint64_t operation) override
      {
        cancel_plan();
        if (!planner_->service_is_ready())
        {
          Plan::Response response;
          response.reason = "planner_unavailable";
          manager_->planned(goal, operation, std::move(response), Clock::now());
          return;
        }
        
        auto future = planner_->async_send_request(
          std::make_shared<Plan::Request>(std::move(request)),
          [this, goal, operation](rclcpp::Client<Plan>::SharedFuture f)
          {
            const auto c = manager_->context();
            if (!c || c->goal_id != goal || c->operation_id != operation)
            {
            return;
            }
            plan_request_.reset();
            Plan::Response response;
            try
            {
              response = *f.get();
            } 
            catch (const std::exception &)
            {
            response.reason = "planner_response_error";
            }
            if (get_parameter("debug_detection").as_bool())
            {
              RCLCPP_INFO(
                get_logger(),
                "PlanDiagnostic goal=%lu operation=%lu reason=%s poses=%zu",
                static_cast<unsigned long>(goal),
                static_cast<unsigned long>(operation),
                response.reason.c_str(),
                response.path.poses.size());
            }
            manager_->planned(goal, operation, std::move(response), Clock::now());
          });
        plan_request_ = future.request_id;
      }
    
      void cancel_plan() override
      {
        if (plan_request_)
        {
          planner_->remove_pending_request(*plan_request_);
          plan_request_.reset();
        }
      }
      
      void configure_head(bool enabled, uint64_t goal, uint64_t operation) override
      {
        if (!head_->service_is_ready())
        {
          manager_->head_configured(goal, operation, false, Clock::now());
          return;
        }
        head_->set_parameters_atomically(
          {rclcpp::Parameter("move_head", enabled)},
          [this, enabled, goal, operation](std::shared_future<rcl_interfaces::msg::SetParametersResult> f)
          {
            auto c = manager_->context();
            if (!c || c->goal_id != goal || c->operation_id != operation)
            {
              return;
            }
            
            try
            {
              if (!f.get().successful)
              {
                manager_->head_configured(goal, operation, false, Clock::now());
                return;
              }
              head_->get_parameters(
                {"move_head"},
                [this, enabled, goal, operation](std::shared_future<std::vector<rclcpp::Parameter>> values)
                {
                  bool ok = false;
                  try
                  {
                    auto v = values.get();
                    ok = v.size() == 1 && v[0].as_bool() == enabled;
                  }
                  catch (const std::exception &)
                  {
                  }
                  manager_->head_configured(goal, operation, ok, Clock::now());
                });
            }
            catch (const std::exception &)
            {
              manager_->head_configured(goal, operation, false, Clock::now());
            }
          });
      }
    
      std::string move(Command command) override
      {
        int64_t stamp = std::max(now().nanoseconds(), last_motion_stamp_ + 1);
        last_motion_stamp_ = stamp;
        command.path.header.stamp = rclcpp::Time(stamp, get_clock()->get_clock_type());
        for (auto & pose:command.path.poses)
        {
          pose.header = command.path.header;
        }
        std::string id = std::to_string(command.path.header.stamp.sec) + "." + std::to_string(command.path.header.stamp.nanosec);
        publish_legacy(actionlib_msgs::msg::GoalStatus::ACTIVE,manager_->context()->diagnostic);
        visual_pub_->publish(command.path);
        path_pub_->publish(command);
        return id;
      }

      void stop(uint64_t goal, uint64_t operation) override
      {
        stop_pub_->publish(std_msgs::msg::Empty());
        if (stop_request_)
        {
          stop_client_->remove_pending_request(*stop_request_);
          stop_request_.reset();
        }
        if (!stop_client_->service_is_ready())
        {
          return;
        }
        auto future = stop_client_->async_send_request(
          std::make_shared<std_srvs::srv::Trigger::Request>(),
          [this, goal, operation](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture f)
          {
            auto c = manager_->context();
            if (!c || c->goal_id != goal || c->operation_id != operation)
            {
              return;
            }
            stop_request_.reset();
            bool ack = false;
            try
            {
              ack = f.get()->success;
            }
            catch (const std::exception &)
            {
            }
            manager_->stopped(goal, operation, ack);
          });
        stop_request_ = future.request_id;
      }

      void enable_sensors(bool enabled) override
      {
        std_msgs::msg::Bool value;
        value.data = enabled;
        enable_pub_->publish(value);
      }

      Settings settings_;
      tf2_ros::Buffer tf_;
      tf2_ros::TransformListener listener_;
      std::string prefix_, base_, odom_, last_diagnostic_;
      uint8_t last_status_ = 0;
      std::unique_ptr<NavigationManager> manager_;
      rclcpp_action::Server<Action>::SharedPtr server_;
      std::shared_ptr<Handle> active_handle_, pending_handle_;
      std::optional<Action::Goal> pending_;
      bool pending_cancel_ = false, active_cancel_ = false, unsafe_stop_ = false;
      uint64_t sequence_ = 0;
      int64_t last_motion_stamp_ = 0;
      Time last_feedback_{};
      path_planner::MotionObservation odometry_;
      std::optional<int64_t> plan_request_, location_request_, stop_request_;
      rclcpp::Client<Plan>::SharedPtr planner_;
      rclcpp::Client<Locations>::SharedPtr locations_;
      rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_client_;
      std::shared_ptr<rclcpp::AsyncParametersClient> head_;
      rclcpp::Publisher<Command>::SharedPtr path_pub_;
      rclcpp::Publisher<Path>::SharedPtr visual_pub_;
      rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr stop_pub_;
      rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr enable_pub_;
      rclcpp::Publisher<actionlib_msgs::msg::GoalStatus>::SharedPtr status_pub_;
      rclcpp::Subscription<Pose>::SharedPtr legacy_sub_;
      rclcpp::Subscription<actionlib_msgs::msg::GoalStatus>::SharedPtr result_sub_;
      rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr stop_sub_;
      rclcpp::Subscription<Grid>::SharedPtr map_sub_, clearance_sub_;
      rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr uncertain_sub_;
      rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ready_sub_;
      rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr witness_sub_;
      rclcpp::TimerBase::SharedPtr timer_;
  };
}
  
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<navigation_manager::NavigationNode>());
  rclcpp::shutdown();
  return 0;
}