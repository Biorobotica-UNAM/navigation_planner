// ----- Navigation Library ----- 
#include <navigation_manager/navigation.hpp>
#include <navigation_manager/msg/navigation_status.hpp>

// ----- ROS 2 -----
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_type.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <rcl_interfaces/srv/set_parameters_atomically.hpp>

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
    using Status    = msg::NavigationStatus;
    using LegacyStatus = actionlib_msgs::msg::GoalStatus;
    using Trigger   = std_srvs::srv::Trigger;
    using SetHead   = rcl_interfaces::srv::SetParametersAtomically;
    using GetHead   = rcl_interfaces::srv::GetParameters;
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
        const auto motion_node = declare_parameter<std::string>("motion_node", "simple_move_node", startup);
        set_head_ = create_client<SetHead>(motion_node + "/set_parameters_atomically");
        get_head_ = create_client<GetHead>(motion_node + "/get_parameters");
        path_pub_   = create_publisher<Command>(prefix_ + "/simple_move/navigation_path", 10);
        visual_pub_ = create_publisher<Path>(prefix_ + "/path", rclcpp::QoS(1).transient_local());
        stop_pub_   = create_publisher<std_msgs::msg::Empty>(prefix_ + "/simple_move/stop", 10);
        enable_pub_ = create_publisher<std_msgs::msg::Bool>(prefix_ + "/potential_fields/enable", rclcpp::QoS(1).transient_local());
        legacy_status_pub_ = create_publisher<LegacyStatus>(prefix_ + "/status", rclcpp::QoS(10).transient_local());
      
        status_pub_ = create_publisher<Status>(
          prefix_ + "/navigation_status", rclcpp::QoS(1).reliable().transient_local());
        reset_service_ = create_service<Trigger>(
          "reset_fault",
          [this](Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response)
          {
            response->success = manager_->reset_fault(Clock::now());
            if (response->success)
            {
              response->message = "Verification started. Check navigation_status for the outcome.";
              report_fault_reset("Verifying controller stop and odometry");
            }
            else
            {
              response->message = manager_->faulted() ?
                "Fault reset verification already in progress" : "No navigation fault to reset";
            }
          });

        // Create ROS2 Action server
        server_ = rclcpp_action::create_server<Action>
        (
          this, 
          "navigate",                                                 // Action name
          [this](const rclcpp_action::GoalUUID &,                         // Goal callback
          std::shared_ptr<const Action::Goal> goal)                   // Goal request
          {
            if(goal->target_type != Action::Goal::POSE &&            
               goal->target_type != Action::Goal::LOCATION)
            {
              RCLCPP_WARN(get_logger(), "Invalid goal type %u", unsigned(goal->target_type));
              return rclcpp_action::GoalResponse::REJECT;             // Reject invalid goal types
            }
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
          },
          [this](std::shared_ptr<Handle> h)                           // Cancel callback
          {
            if(pending_handle_ == h)
            {
              // Complete after the action server has committed its cancel transition.
              pending_cancel_ = true;
            }
            else if (active_handle_ == h)
            {
              active_cancel_ = true;
            }
            else
            {
              return rclcpp_action::CancelResponse::REJECT;
            }
            RCLCPP_INFO(get_logger(), "Cancel requested");
            return rclcpp_action::CancelResponse::ACCEPT;
          },                                                          // Accepted callback
          [this](std::shared_ptr<Handle> h)
          {
            queue(*h->get_goal(),h);
          }
        );

        const auto pose_goal = [this](Pose::ConstSharedPtr pose)
        {
          Action::Goal goal;
          goal.target_type = Action::Goal::POSE;
          goal.pose = *pose;
          goal.move_head = get_parameter("legacy_move_head").as_bool();
          queue(goal, nullptr);
        };
        pose_sub_ = create_subscription<Pose>("/navigation/goal", 10, pose_goal);
        legacy_sub_ = create_subscription<Pose>("/nav_control/goal", 10, pose_goal);

        declare_parameter<bool>("legacy_move_head", true);
        declare_parameter<bool>("debug_detection", false);

        // Subscription for goal status rached
        result_sub_ = create_subscription<actionlib_msgs::msg::GoalStatus>(
          prefix_ + "/simple_move/goal_reached",
          10,
          [this](actionlib_msgs::msg::GoalStatus::ConstSharedPtr status)
          {
            if (!active_cancel_)
            {
              const auto tick = Clock::now();
              manager_->motion_status(status->goal_id.id, status->status, status->text, tick);
              update_status(tick);
            }
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
              RCLCPP_DEBUG(get_logger(), "Failed to transform witness point to map frame");
            }
          });

        manager_->completed = [this](uint64_t id, const Action::Result & value)
        {
          if (id != active_id_)
          {
            return;
          }
          publish_result(last_status_, value);
          finish_action(std::move(active_handle_), value);
          log_result(value);
          publish_legacy(id, reached(value) ? LegacyStatus::SUCCEEDED : LegacyStatus::ABORTED,
            value.reason);

          clear_stop_request();
          Path empty;
          empty.header.frame_id = settings_.map_frame;
          visual_pub_->publish(empty);

        };
        last_status_ = make_status(0, Status::SOURCE_NONE);
        last_status_.status_code = Status::IDLE;
        last_status_.status_message = "Navigation idle";
        publish_status(last_status_);
        timer_ = create_wall_timer(std::chrono::milliseconds(50), [this] {tick();});
      }

    private:
      bool fresh(const builtin_interfaces::msg::Time & stamp, double timeout) const
      {
        auto time = rclcpp::Time(stamp, get_clock()->get_clock_type());
        return time.nanoseconds() != 0 && std::abs((now() - time).seconds()) <= timeout;
      }
    
      static bool reached(const Action::Result & result)
      {
        return result.result_code == Action::Result::SUCCESS ||
               result.result_code == Action::Result::APPROXIMATE;
      }

      static void finish_action(std::shared_ptr<Handle> handle, const Action::Result & value)
      {
        if (!handle)
        {
          return;
        }
        auto result = std::make_shared<Action::Result>(value);
        if (handle->is_canceling())
        {
          handle->canceled(result);
        }
        else if (reached(value))
        {
          handle->succeed(result);
        }
        else
        {
          handle->abort(result);
        }
      }

      void reject_pending(const std::string & reason, uint8_t code)
      {
        Action::Result result;
        result.result_code = pending_cancel_ ? Action::Result::CANCELLED : code;
        result.reason = pending_cancel_ ? "cancelled" : reason;
        result.result_message = "Goal closed before activation";
        result.position_error_m = result.orientation_error_rad =
          std::numeric_limits<float>::quiet_NaN();
        auto status = make_status(pending_id_, pending_handle_ ? Status::SOURCE_ACTION : Status::SOURCE_TOPIC);
        publish_result(status, result);
        finish_action(std::move(pending_handle_), result);
        log_result(result);
        const auto c = manager_->context();
        if (c && c->state != State::Finished)
        {
          publish_status(last_status_);
        }
        else
        {
          last_status_ = std::move(status);
        }
        pending_.reset();
        pending_cancel_ = false;
      }

      void queue(const Action::Goal & goal, std::shared_ptr<Handle> handle)
      {
        if (pending_)
        {
          reject_pending("preempted", Action::Result::PREEMPTED);
        }
        pending_id_ = ++sequence_;
        pending_ = goal;
        pending_handle_ = std::move(handle);
        if (goal.target_type == Action::Goal::LOCATION)
        {
          RCLCPP_INFO(get_logger(), "Goal received for %s", goal.location_name.c_str());
        }
        else
        {
          RCLCPP_INFO(get_logger(), "Pose goal received (%.2f, %.2f)",
            goal.pose.pose.position.x, goal.pose.pose.position.y);
        }
        if (manager_->faulted())
        {
          reject_pending("stop_unverified", Action::Result::FAILED);
          return;
        }
        const auto c = manager_->context();
        if (c && c->state != State::Finished && !active_cancel_)
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
        if (manager_->resetting_fault())
        {
          if (reset_odom_after_)
          {
            observe_odometry(reset_odom_after_);
          }
          if (const auto outcome = manager_->verify_fault_reset(odometry_, tick))
          {
            clear_stop_request();
            reset_odom_after_.reset();
            report_fault_reset(outcome);
          }
        }
        auto c = manager_->context();
        if (pending_ && (!c || c->state == State::Finished))
        {
          if (manager_->faulted())
          {
            reject_pending("stop_unverified", Action::Result::FAILED);
            return;
          }
          active_handle_ = std::move(pending_handle_);
          auto goal = std::move(*pending_);
          pending_.reset();
          manager_->monitor.witnesses.clear();
          odometry_ = {};
          active_id_ = pending_id_;
          last_status_ = make_status(active_id_, active_handle_ ? Status::SOURCE_ACTION : Status::SOURCE_TOPIC);
          manager_->start(active_id_, goal, tick);
          last_sensor_enable_ = tick;
          update_status(tick, true);
          if (goal.target_type == Action::Goal::POSE)
          {
            resolve(goal.pose, active_id_);
          }
          else if (!locations_->service_is_ready())
          {
            manager_->resolved(active_id_, Pose{}, "locations_unavailable", tick);
          }
          else
          {
            auto request = std::make_shared<Locations::Request>();
            request->location_name = goal.location_name;
            const auto id = active_id_;
            auto future = locations_->async_send_request(
              request,
              [this, id](rclcpp::Client<Locations>::SharedFuture f)
              {
                const auto c = manager_->context();
                if (!c || c->goal_id != id || c->state != State::Resolving || active_cancel_)
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
                    RCLCPP_INFO(get_logger(), "Location resolved (%.2f, %.2f)",
                      pose.pose.position.x, pose.pose.position.y);
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
                update_status(Clock::now());
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
          const auto & p = pose.pose.position;
          const auto & q = pose.pose.orientation;
          if (!std::isfinite(p.x + p.y + q.x + q.y + q.z + q.w) ||
              q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w < 1e-12)
          {
            pose.header.frame_id.clear();
          }
        }
        catch (const std::exception &)
        {
          pose.header.frame_id.clear();
        }
        observe_odometry();

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
        update_status(tick);
        c = manager_->context();
        if (c && c->state != State::Finished && tick - last_sensor_enable_ >= std::chrono::milliseconds(200))
        {
          last_sensor_enable_ = tick;
          enable_sensors(true);
        }
      }

      void observe_odometry(std::optional<int64_t> after_stamp = std::nullopt)
      {
        try
        {
          const auto odom = tf_.lookupTransform(odom_, base_, tf2::TimePointZero);
          const auto stamp = rclcpp::Time(odom.header.stamp).nanoseconds();
          const auto & q = odom.transform.rotation;
          const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
          if ((!after_stamp || (stamp > *after_stamp && stamp <= now().nanoseconds())) &&
              fresh(odom.header.stamp, settings_.tf_timeout) &&
              std::isfinite(norm) && norm >= 1e-12)
          {
            odometry_.observe(stamp, odom.transform.translation.x, odom.transform.translation.y,
              tf2::getYaw(q));
            return;
          }
        }
        catch (const std::exception &)
        {
        }
        odometry_ = {};
      }

      static uint8_t public_state(const NavigationContext & c)
      {
        switch (c.state)
        {
          case State::Resolving: return Action::Feedback::RESOLVING;
          case State::Configuring: return Action::Feedback::CONFIGURING;
          case State::Planning: return c.total_recoveries ? Action::Feedback::RECOVERING : Action::Feedback::PLANNING;
          case State::Moving:
            return c.recovery || c.start_escape ? Action::Feedback::RECOVERING : Action::Feedback::MOVING;
          case State::Recovering: return Action::Feedback::RECOVERING;
          case State::Verifying: return Action::Feedback::VERIFYING;
          case State::Stopping: return c.recovery ? Action::Feedback::RECOVERING : Action::Feedback::STOPPING;
          case State::Finished: return Status::FINISHED;
        }
        return Status::IDLE;
      }

      static std::string status_message(const NavigationContext & c)
      {
        const auto & phase = c.motion_phase;
        if (c.state == State::Moving)
        {
          if (phase == "aligning_head") {return "Aligning robot head";}
          if (phase == "checking_motion_direction") {return "Checking motion direction";}
          if (phase == "local_avoidance") {return "Avoiding local obstacle";}
          if (phase == "waiting_for_obstacle_clearance" || phase == "waiting_for_local_clearance")
          {
            return "Waiting for obstacle clearance";
          }
          if (phase == "waiting_for_obstacle_data") {return "Waiting for obstacle data";}
          if (phase == "approaching_goal") {return "Approaching goal";}
          if (phase == "braking_near_goal") {return "Braking near goal";}
          if (c.start_escape) {return "Moving clear of blocked start";}
          if (c.recovery == Operation::Observe) {return "Observing blocked route";}
          return "Following navigation path";
        }
        switch (c.state)
        {
          case State::Resolving: return "Resolving navigation goal";
          case State::Configuring: return "Configuring head movement";
          case State::Planning: return c.total_recoveries ? "Replanning navigation path" : "Computing navigation path";
          case State::Verifying: return "Verifying final pose";
          case State::Stopping:
            if (c.recovery)
            {
              if (c.diagnostic == "route_blocked" || c.diagnostic == "local_trajectory_blocked")
              {
                return "Navigation path blocked. Stopping robot";
              }
              return "Stopping robot for recovery (" + c.diagnostic + ")";
            }
            return "Stopping robot";
          case State::Recovering:
            if (c.recovery == Operation::Observe) {return "Preparing to observe blocked route";}
            if (c.recovery == Operation::WaitSensors) {return "Waiting for obstacle data";}
            if (c.recovery == Operation::WaitLocalization) {return "Waiting for localization";}
            return c.total_recoveries ? "Replanning navigation path" : "Preparing navigation path";
          default: return "Navigation finished";
        }
      }

      Status make_status(uint64_t id, uint8_t source) const
      {
        Status value;
        value.header.frame_id = settings_.map_frame;
        value.goal_id = id;
        value.source = source;
        value.distance_remaining_m = value.position_error_m = value.orientation_error_rad =
          std::numeric_limits<float>::quiet_NaN();
        return value;
      }

      void publish_status(Status & value)
      {
        value.header.stamp = now();
        value.faulted = manager_->faulted();
        status_pub_->publish(value);
      }

      void update_status(Time tick, bool force = false)
      {
        const auto c = manager_->context();
        if (!c || c->state == State::Finished)
        {
          return;
        }
        const auto code = public_state(*c);
        const bool transition = force || last_state_ != c->state || code != last_status_.status_code;
        const bool recovery = last_status_.total_recoveries != c->total_recoveries;
        const bool issue_changed = last_diagnostic_ != c->diagnostic;
        const bool changed = transition || recovery || issue_changed || last_phase_ != c->motion_phase;
        if (!changed && tick - last_feedback_ < std::chrono::milliseconds(200))
        {
          return;
        }

        if (changed)
        {
          last_status_.status_code = code;
          const auto message = status_message(*c);
          const bool log_state = transition && last_status_.status_message != message;
          last_status_.status_message = message;
          last_status_.total_recoveries = c->total_recoveries;
          log_transition(*c, log_state, recovery, issue_changed);
          last_state_ = c->state;
          last_diagnostic_ = c->diagnostic;
          last_phase_ = c->motion_phase;
        }
        last_status_.distance_remaining_m = c->pose_valid && c->resolved ? std::hypot(
          c->robot.pose.position.x - c->original.pose.position.x,
          c->robot.pose.position.y - c->original.pose.position.y) :
          std::numeric_limits<float>::quiet_NaN();
        last_feedback_ = tick;
        publish_status(last_status_);
        if (active_handle_ && active_handle_->is_executing())
        {
          auto feedback = std::make_shared<Action::Feedback>();
          feedback->status_code = code;
          feedback->status_message = last_status_.status_message;
          feedback->distance_remaining_m = last_status_.distance_remaining_m;
          feedback->total_recoveries = last_status_.total_recoveries;
          active_handle_->publish_feedback(feedback);
        }
        publish_legacy(c->goal_id, LegacyStatus::ACTIVE,
          c->motion_phase.empty() ? c->diagnostic : c->motion_phase);
      }

      void publish_result(Status & status, const Action::Result & result)
      {
        status.status_code = Status::FINISHED;
        status.status_message = result.result_message;
        status.result_valid = true;
        status.result_code = result.result_code;
        status.reason = result.reason;
        status.result_message = result.result_message;
        status.distance_remaining_m = status.position_error_m = result.position_error_m;
        status.orientation_error_rad = result.orientation_error_rad;
        publish_status(status);
      }

      void report_fault_reset(const char * message)
      {
        last_status_.status_message = message;
        publish_status(last_status_);
        if (manager_->faulted() && !manager_->resetting_fault())
        {
          RCLCPP_ERROR(get_logger(), "%s", message);
        }
        else
        {
          RCLCPP_INFO(get_logger(), "%s", message);
        }
      }

      void log_transition(const NavigationContext & c, bool transition, bool recovery, bool issue_changed)
      {
        if (recovery && !c.recovery_history.empty())
        {
          RCLCPP_WARN(get_logger(), "Recovery %u (%s)", unsigned(c.total_recoveries),
            c.recovery_history.back().c_str());
        }
        if (transition)
        {
          RCLCPP_INFO(get_logger(), "%s", last_status_.status_message.c_str());
        }
        if (issue_changed && get_parameter("debug_detection").as_bool())
        {
          RCLCPP_DEBUG(get_logger(), "Navigation diagnostic %s", c.diagnostic.c_str());
        }
      }

      void log_result(const Action::Result & result)
      {
        if (result.result_code == Action::Result::SUCCESS)
        {
          RCLCPP_INFO(get_logger(), "Goal reached (position error %.3f m, orientation error %.3f rad)",
            result.position_error_m, result.orientation_error_rad);
        }
        else if (result.result_code == Action::Result::APPROXIMATE)
        {
          RCLCPP_WARN(get_logger(), "Approximate arrival (position error %.3f m, orientation error %.3f rad)",
            result.position_error_m, result.orientation_error_rad);
        }
        else if (result.result_code == Action::Result::CANCELLED ||
                 result.result_code == Action::Result::PREEMPTED)
        {
          RCLCPP_INFO(get_logger(), "Goal %s", result.reason.c_str());
        }
        else
        {
          RCLCPP_ERROR(get_logger(), "Navigation failed (%s)", result.reason.c_str());
        }
      }

      void publish_legacy(uint64_t id, uint8_t status, const std::string & reason)
      {
        LegacyStatus value;
        value.goal_id.id = std::to_string(id);
        value.goal_id.stamp = now();
        value.status = status;
        value.text = reason;
        legacy_status_pub_->publish(value);
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
            if (!c || c->goal_id != goal || c->operation_id != operation ||
                c->state != State::Planning || active_cancel_)
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
              RCLCPP_DEBUG(
                get_logger(),
                "PlanDiagnostic goal=%lu operation=%lu reason=%s poses=%zu",
                static_cast<unsigned long>(goal),
                static_cast<unsigned long>(operation),
                response.reason.c_str(),
                response.path.poses.size());
            }
            manager_->planned(goal, operation, std::move(response), Clock::now());
            update_status(Clock::now());
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
      
      bool configuring_head(uint64_t goal, uint64_t operation) const
      {
        const auto c = manager_->context();
        return c && c->goal_id == goal && c->operation_id == operation &&
               c->state == State::Configuring && !active_cancel_;
      }

      void configure_head(bool enabled, uint64_t goal, uint64_t operation) override
      {
        if (!set_head_->service_is_ready() || !get_head_->service_is_ready())
        {
          manager_->head_configured(goal, operation, false, Clock::now());
          return;
        }
        auto request = std::make_shared<SetHead::Request>();
        request->parameters = {rclcpp::Parameter("move_head", enabled).to_parameter_msg()};
        set_head_request_ = set_head_->async_send_request(
          request,
          [this, enabled, goal, operation](rclcpp::Client<SetHead>::SharedFuture f)
          {
            set_head_request_.reset();
            if (!configuring_head(goal, operation))
            {
              return;
            }

            try
            {
              if (f.get()->result.successful)
              {
                read_head(enabled, goal, operation);
                return;
              }
            }
            catch (const std::exception &)
            {
            }
            manager_->head_configured(goal, operation, false, Clock::now());
          }).request_id;
      }

      void read_head(bool enabled, uint64_t goal, uint64_t operation)
      {
        auto request = std::make_shared<GetHead::Request>();
        request->names = {"move_head"};
        get_head_request_ = get_head_->async_send_request(
          request,
          [this, enabled, goal, operation](rclcpp::Client<GetHead>::SharedFuture f)
          {
            get_head_request_.reset();
            if (!configuring_head(goal, operation))
            {
              return;
            }

            bool ok = false;
            try
            {
              const auto & values = f.get()->values;
              ok = values.size() == 1 &&
                values.front().type == rcl_interfaces::msg::ParameterType::PARAMETER_BOOL &&
                values.front().bool_value == enabled;
            }
            catch (const std::exception &)
            {
            }
            manager_->head_configured(goal, operation, ok, Clock::now());
            if (ok)
            {
              RCLCPP_INFO(get_logger(), "Head movement %s", enabled ? "enabled" : "disabled");
            }
          }).request_id;
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
        visual_pub_->publish(command.path);
        path_pub_->publish(command);
        return id;
      }

      void clear_stop_request()
      {
        if (stop_request_)
        {
          stop_client_->remove_pending_request(*stop_request_);
          stop_request_.reset();
        }
      }

      void stop(uint64_t goal, uint64_t operation) override
      {
        clear_stop_request();
        if (goal == 0)
        {
          odometry_ = {};
          reset_odom_after_.reset();
        }
        else
        {
          if (location_request_)
          {
            locations_->remove_pending_request(*location_request_);
            location_request_.reset();
          }
          if (set_head_request_)
          {
            set_head_->remove_pending_request(*set_head_request_);
            set_head_request_.reset();
          }
          if (get_head_request_)
          {
            get_head_->remove_pending_request(*get_head_request_);
            get_head_request_.reset();
          }
          stop_pub_->publish(std_msgs::msg::Empty());
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
            if (goal == 0 ? !manager_->resetting_fault() :
                (!c || c->goal_id != goal || c->operation_id != operation || c->state != State::Stopping))
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
            if (goal == 0 && ack)
            {
              // Require a full window of TF samples acquired after the controller confirms idle.
              odometry_ = {};
              reset_odom_after_ = now().nanoseconds();
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
      std::string prefix_, base_, odom_, last_diagnostic_, last_phase_;
      Status last_status_;
      State last_state_ = State::Finished;
      std::unique_ptr<NavigationManager> manager_;
      rclcpp_action::Server<Action>::SharedPtr server_;
      std::shared_ptr<Handle> active_handle_, pending_handle_;
      std::optional<Action::Goal> pending_;
      bool pending_cancel_ = false, active_cancel_ = false;
      uint64_t sequence_ = 0, active_id_ = 0, pending_id_ = 0;
      int64_t last_motion_stamp_ = 0;
      std::optional<int64_t> reset_odom_after_;
      Time last_feedback_{}, last_sensor_enable_{};
      path_planner::MotionObservation odometry_;
      std::optional<int64_t> plan_request_, location_request_, stop_request_;
      std::optional<int64_t> set_head_request_, get_head_request_;
      rclcpp::Client<Plan>::SharedPtr planner_;
      rclcpp::Client<Locations>::SharedPtr locations_;
      rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_client_;
      rclcpp::Client<SetHead>::SharedPtr set_head_;
      rclcpp::Client<GetHead>::SharedPtr get_head_;
      rclcpp::Publisher<Command>::SharedPtr path_pub_;
      rclcpp::Publisher<Path>::SharedPtr visual_pub_;
      rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr stop_pub_;
      rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr enable_pub_;
      rclcpp::Publisher<Status>::SharedPtr status_pub_;
      rclcpp::Publisher<LegacyStatus>::SharedPtr legacy_status_pub_;
      rclcpp::Service<Trigger>::SharedPtr reset_service_;
      rclcpp::Subscription<Pose>::SharedPtr pose_sub_, legacy_sub_;
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