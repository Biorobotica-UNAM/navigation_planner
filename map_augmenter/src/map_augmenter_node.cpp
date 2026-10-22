#include "map_augmenter/grid_utils.hpp"

#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/srv/get_map.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

class MapAugmenterNode : public rclcpp::Node
{
  using Grid = map_augmenter::Grid;
  using GetMap = nav_msgs::srv::GetMap;
  using MapClient = rclcpp::Client<GetMap>;
  using SteadyClock = std::chrono::steady_clock;

public:
  MapAugmenterNode()
  : Node("map_augmenter_node"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    use_namespace_ = parameter("use_namespace", false, "Namespace the default internal endpoints.");
    use_online_ = parameter("use_online", false,
      "Continue polling the base-map service when map topics are unavailable.");
    use_prohibition_ = parameter("use_prohibition_map", true, "Enable the prohibition overlay.");
    require_prohibition_ = parameter("require_prohibition_map", true,
      "Wait for the prohibition overlay before serving planning maps.") && use_prohibition_;
    circular_ = parameter("circular_inflation", false, "Circular inflation; false uses a square kernel.");
    const double legacy_radius = parameter("inflation_radius", 0.15,
      "Fallback inflation radius in metres when a specific radius is negative.");
    const double shared_radius = parameter("obstacle_inflation_radius", -1.0,
      "Fallback for both inflation radii; negative selects inflation_radius.");
    static_radius_ = parameter("static_obstacle_inflation_radius", 0.05,
      "Static clearance in metres; negative selects the fallback radius.");
    dynamic_radius_ = parameter("dynamic_obstacle_inflation_radius", 0.15,
      "Dynamic clearance in metres; negative selects the fallback radius.");
    const double fallback = shared_radius >= 0.0 ? shared_radius : legacy_radius;
    if (static_radius_ < 0.0) {static_radius_ = fallback;}
    if (dynamic_radius_ < 0.0) {dynamic_radius_ = fallback;}
    cost_radius_ = parameter("cost_radius", 0.35, "Soft-cost influence radius in metres.");
    cost_peak_ = parameter("cost_peak", 6, "Maximum generated soft cost, from 1 to 99.");
    debug_ = parameter("debug_maps", false, "Log processed map statistics.");
    log_throttle_ms_ = parameter("log_throttle_ms", 2000, "Repeated warning interval in milliseconds.");
    topic_timeout_ = parameter("map_topic_timeout_sec", 3.0,
      "Time without topic messages before allowing service fallback.");
    request_timeout_ = parameter("map_request_timeout_sec", 3.0, "Unanswered map-request timeout.");
    const double refresh_period = parameter("map_refresh_period_sec", 1.0, "Map-service polling interval.");
    const double publish_period = parameter("publish_period_sec", 1.1, "Cached map republishing interval.");
    tf_timeout_ = parameter("tf_timeout_sec", 0.03, "Robot-position TF lookup timeout.");
    robot_tf_max_age_ = parameter("robot_tf_max_age_sec", 0.5,
      "Maximum age of a non-static robot-position transform.");
    base_link_ = parameter("base_link_name", std::string("base_footprint"), "Robot base TF frame.");
    map_frame_ = parameter("map_frame", std::string("map"), "Required frame for all input grids.");
    const int depth = parameter("map_qos_depth", 1, "Grid history depth.");
    const int point_depth = parameter("point_qos_depth", 5, "Point-obstacle subscription history depth.");
    const bool input_latched = parameter("map_input_transient_local", true, "Request retained map inputs.");
    const bool input_reliable = parameter("map_input_reliable", true, "Request reliable map inputs.");
    const bool obstacle_reliable = parameter("obstacles_input_reliable", true, "Request reliable obstacle grids.");
    const bool output_latched = parameter("map_output_transient_local", true, "Retain published grids.");
    if (!nonnegative(static_radius_) || !nonnegative(dynamic_radius_) ||
      !nonnegative(cost_radius_) || !std::isfinite(legacy_radius) || !std::isfinite(shared_radius) ||
      cost_peak_ < 1 || cost_peak_ > 99 || !nonnegative(topic_timeout_) ||
      !nonnegative(tf_timeout_) || !nonnegative(robot_tf_max_age_) || !positive(request_timeout_) ||
      !positive(refresh_period) || !positive(publish_period) || depth < 1 || point_depth < 1 ||
      log_throttle_ms_ < 1 || map_frame_.empty() || base_link_.empty())
    {
      throw std::invalid_argument("Invalid map augmenter parameter range");
    }

    // Compatibility with existing launches: neither value changes the explicit
    // clearance radii, and expiration is exclusively the detector's job.
    parameter("decay_factor", 0, "Deprecated here; obstacle_detector owns expiration.");
    parameter("robot_radius", 0.35, "Legacy metadata; explicit inflation radii control clearance.");

    auto output_qos = rclcpp::QoS(depth).reliable();
    if (output_latched) {output_qos.transient_local();}
    auto input_qos = rclcpp::QoS(depth);
    if (input_latched) {input_qos.transient_local();}
    if (input_reliable) {input_qos.reliable();} else {input_qos.best_effort();}
    auto obstacle_qos = rclcpp::QoS(depth);
    if (obstacle_reliable) {obstacle_qos.reliable();} else {obstacle_qos.best_effort();}

    pub_augmented_ = create_publisher<Grid>(endpoint("augmented_map_topic", "/augmented_map"), output_qos);
    pub_planning_ = create_publisher<Grid>(endpoint("planning_map_topic", "/planning_map"), output_qos);
    pub_base_ = create_publisher<Grid>(endpoint("base_map_output_topic", "/map_augmenter/base_map"), output_qos);
    const auto base_topic = parameter("base_map_topic", std::string("map"),
      "Raw map-server or SLAM topic; empty disables the subscription.");
    if (!base_topic.empty()) {
      sub_base_ = create_subscription<Grid>(base_topic, input_qos, [this](Grid::ConstSharedPtr msg) {
        on_map(base_input_, *msg, true, true);
      });
      if (std::string(sub_base_->get_topic_name()) == pub_base_->get_topic_name() ||
        std::string(sub_base_->get_topic_name()) == pub_planning_->get_topic_name() ||
        std::string(sub_base_->get_topic_name()) == pub_augmented_->get_topic_name())
      {
        throw std::invalid_argument("base_map_topic must not subscribe to an augmenter output");
      }
    }
    sub_obstacles_ = create_subscription<Grid>(
      endpoint("obstacles_map_topic", "/obstacles_map"), obstacle_qos, [this](Grid::ConstSharedPtr msg) {
        if (!accept_grid(*msg)) {return;}
        dynamic_obstacles_ = *msg;
        have_obstacles_ = true;
        rebuild_augmented_layers();
      });
    base_input_.client = create_client<GetMap>(parameter(
      "static_map_server", std::string("/static_map_server/map"), "Base-map GetMap service fallback."));
    const auto prohibition_topic = parameter("prohibition_map_topic", std::string(""),
      "Optional prohibition topic; empty preserves the existing service source.");
    const auto prohibition_service = parameter("prohibition_map_server",
      std::string("/prohibition_map_server/map"), "Prohibition GetMap service fallback.");
    if (use_prohibition_) {
      if (!prohibition_topic.empty()) {
        sub_prohibition_ = create_subscription<Grid>(prohibition_topic, input_qos,
          [this](Grid::ConstSharedPtr msg) {on_map(prohibition_input_, *msg, false, true);});
      }
      prohibition_input_.client = create_client<GetMap>(prohibition_service);
    }
    auto point_qos = rclcpp::SensorDataQoS();
    point_qos.keep_last(point_depth);
    sub_point_ = create_subscription<geometry_msgs::msg::PointStamped>(
      endpoint("point_obstacle_topic", "/point_obstacle"), point_qos,
      [this](geometry_msgs::msg::PointStamped::ConstSharedPtr) {poll_maps(true);});

    add_map_service("get_static_map_service", "/map_augmenter/get_static_map",
      [this]() -> const Grid & {return snapshot_.static_collision;});
    add_map_service("get_static_cost_map_service", "/map_augmenter/get_static_cost_map",
      [this]() -> const Grid & {return snapshot_.static_cost;});
    add_map_service("get_augmented_map_service", "/map_augmenter/get_augmented_map",
      [this]() -> const Grid & {return snapshot_.augmented_collision;});
    add_map_service("get_augmented_cost_map_service", "/map_augmenter/get_augmented_cost_map",
      [this]() -> const Grid & {return snapshot_.augmented_cost;});
    srv_obstacles_ = create_service<std_srvs::srv::Trigger>(
      endpoint("are_there_obstacles_service", "/map_augmenter/are_there_obstacles"),
      [this](std_srvs::srv::Trigger::Request::SharedPtr,
      std_srvs::srv::Trigger::Response::SharedPtr response) {
        response->success = ready_ && snapshot_.has_dynamic_obstacles;
        response->message = ready_ ? "Dynamic obstacle status" : "Map is not ready";
      });
    srv_inside_ = create_service<std_srvs::srv::Trigger>(
      endpoint("is_inside_obstacles_service", "/map_augmenter/is_inside_obstacles"),
      [this](std_srvs::srv::Trigger::Request::SharedPtr,
      std_srvs::srv::Trigger::Response::SharedPtr response) {inside_obstacles(*response);});

    // All state callbacks share the default mutually-exclusive group. Only the
    // TF listener uses a separate thread, so no callback can read half a snapshot.
    refresh_timer_ = create_wall_timer(std::chrono::duration<double>(refresh_period),
      [this]() {poll_maps(false);});
    publish_timer_ = create_wall_timer(std::chrono::duration<double>(publish_period),
      [this]() {
        if (base_input_.ready) {pub_base_->publish(base_input_.grid);}
        publish_snapshot();
      });
    RCLCPP_INFO(get_logger(), "Map augmenter ready; configuration parameters are startup-only");
  }

private:
  struct MapInput
  {
    Grid grid;
    bool ready{false}, topic_received{false}, request_pending{false};
    uint64_t topic_revision{0};
    int64_t request_id{0};
    SteadyClock::time_point last_topic, request_started;
    MapClient::SharedPtr client;
  };

  struct Snapshot
  {
    Grid static_collision, static_cost, augmented_collision, augmented_cost;
    bool has_dynamic_obstacles{false};
  };

  template<typename T>
  T parameter(const std::string & name, const T & value, const std::string & description)
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    descriptor.description = description;
    return declare_parameter<T>(name, value, descriptor);
  }

  std::string endpoint(const std::string & name, const std::string & old_default)
  {
    std::string default_name = use_namespace_ ? std::string(get_namespace()) + old_default : old_default;
    if (default_name.compare(0, 2, "//") == 0) {default_name.erase(0, 1);}
    return parameter(name, default_name, "ROS endpoint; explicit relative values use the node namespace.");
  }

  static bool nonnegative(double value) {return std::isfinite(value) && value >= 0.0;}
  static bool positive(double value) {return std::isfinite(value) && value > 0.0;}
  static double elapsed(SteadyClock::time_point since)
  {
    return std::chrono::duration<double>(SteadyClock::now() - since).count();
  }

  bool accept_grid(const Grid & map)
  {
    std::string reason;
    if (!map_augmenter::validate_grid(map, &reason)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), log_throttle_ms_,
        "Rejected grid: %s", reason.c_str());
      return false;
    }
    if (map.header.frame_id != map_frame_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), log_throttle_ms_,
        "Rejected grid frame '%s'; expected '%s'", map.header.frame_id.c_str(), map_frame_.c_str());
      return false;
    }
    return true;
  }

  void on_map(MapInput & input, const Grid & map, bool is_base, bool from_topic)
  {
    if (!accept_grid(map)) {return;}
    if (from_topic) {
      input.last_topic = SteadyClock::now();
      input.topic_received = true;
      ++input.topic_revision;
    }
    const bool changed = !input.ready || !map_augmenter::same_geometry(input.grid, map) ||
      input.grid.data != map.data;
    input.grid = map;
    input.ready = true;
    if (is_base) {pub_base_->publish(input.grid);}
    if (changed) {rebuild_base_layers();}
  }

  void rebuild_base_layers()
  {
    if (!base_input_.ready || (require_prohibition_ && !prohibition_input_.ready)) {return;}
    const Grid merged = use_prohibition_ && prohibition_input_.ready ?
      map_augmenter::overlay_obstacles(base_input_.grid, prohibition_input_.grid) : base_input_.grid;
    static_collision_ = map_augmenter::inflate_obstacles(merged, static_radius_, circular_);
    static_cost_ = map_augmenter::make_cost_map(static_collision_, cost_radius_, cost_peak_, circular_);
    rebuild_augmented_layers();
  }

  void rebuild_augmented_layers()
  {
    if (static_collision_.data.empty()) {return;}
    Snapshot next;
    next.static_collision = static_collision_;
    next.static_cost = static_cost_;
    next.augmented_collision = static_collision_;
    next.augmented_cost = static_cost_;
    if (have_obstacles_) {
      const Grid projected = map_augmenter::reproject_obstacles(dynamic_obstacles_, base_input_.grid);
      next.has_dynamic_obstacles = std::any_of(projected.data.begin(), projected.data.end(),
        [](int8_t value) {return value > 0;});
      const Grid inflated = map_augmenter::inflate_obstacles(projected, dynamic_radius_, circular_);
      next.augmented_collision = map_augmenter::overlay_obstacles(static_collision_, inflated);
      const Grid dynamic_cost = map_augmenter::make_cost_map(inflated, cost_radius_, cost_peak_, circular_);
      for (size_t i = 0; i < dynamic_cost.data.size(); ++i) {
        if (next.augmented_collision.data[i] == -1) {
          next.augmented_cost.data[i] = -1;
        } else if (dynamic_cost.data[i] > 0) {
          next.augmented_cost.data[i] = std::max(next.augmented_cost.data[i], dynamic_cost.data[i]);
        }
      }
    }
    const auto stamp = now();
    next.static_collision.header.stamp = stamp;
    next.static_cost.header.stamp = stamp;
    next.augmented_collision.header.stamp = stamp;
    next.augmented_cost.header.stamp = stamp;
    snapshot_ = std::move(next);
    ready_ = true;
    if (debug_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), log_throttle_ms_,
        "Rebuilt %ux%u map; dynamic obstacles=%s", base_input_.grid.info.width,
        base_input_.grid.info.height, snapshot_.has_dynamic_obstacles ? "true" : "false");
    }
    publish_snapshot();
  }

  void publish_snapshot()
  {
    if (!ready_) {return;}
    pub_augmented_->publish(snapshot_.augmented_collision);
    pub_planning_->publish(snapshot_.augmented_collision);
  }

  void add_map_service(const std::string & name, const std::string & default_name,
    std::function<const Grid &()> select)
  {
    map_services_.push_back(create_service<GetMap>(endpoint(name, default_name),
      [this, select](GetMap::Request::SharedPtr, GetMap::Response::SharedPtr response) {
        if (!ready_) {
          // GetMap has no success field: an empty grid means not ready.
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), log_throttle_ms_, "Map snapshot is not ready");
          return;
        }
        response->map = select();
      }));
  }

  void inside_obstacles(std_srvs::srv::Trigger::Response & response)
  {
    if (!ready_) {response.message = "Map is not ready"; return;}
    try {
      const auto tf = tf_buffer_.lookupTransform(map_frame_, base_link_,
        rclcpp::Time(0, 0, get_clock()->get_clock_type()),
        rclcpp::Duration::from_seconds(tf_timeout_));
      const rclcpp::Time stamp(tf.header.stamp, get_clock()->get_clock_type());
      if (stamp.nanoseconds() != 0 && std::abs((now() - stamp).seconds()) > robot_tf_max_age_) {
        response.message = "Robot transform is stale";
        return;
      }
      int x, y;
      if (!map_augmenter::world_to_cell(snapshot_.augmented_collision,
        tf.transform.translation.x, tf.transform.translation.y, x, y))
      {
        response.message = "Robot is outside the map";
        return;
      }
      response.success = snapshot_.augmented_collision.data[
        map_augmenter::cell_index(snapshot_.augmented_collision, x, y)] > 0;
      response.message = "Robot collision-cell status";
    } catch (const tf2::TransformException & error) {
      response.message = error.what();
    }
  }

  void poll_map(MapInput & input, bool is_base, bool force)
  {
    if (!input.client) {return;}
    // Clean up even when topics resume or the static map is already cached.
    if (input.request_pending && elapsed(input.request_started) > request_timeout_) {
      input.client->remove_pending_request(input.request_id);
      input.request_pending = false;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), log_throttle_ms_, "Map service request timed out");
    }
    const bool fresh_topic = input.topic_received && elapsed(input.last_topic) <= topic_timeout_;
    const bool needs_refresh = force || (!fresh_topic && (!input.ready || !is_base || use_online_));
    if (!needs_refresh || input.request_pending) {return;}
    if (!input.client->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), log_throttle_ms_,
        "Waiting for map service '%s'", input.client->get_service_name());
      return;
    }
    input.request_pending = true;
    input.request_started = SteadyClock::now();
    const uint64_t revision = input.topic_revision;
    MapInput * state = &input;
    const auto pending = input.client->async_send_request(std::make_shared<GetMap::Request>(),
      [this, state, is_base, revision](MapClient::SharedFuture future) {
        state->request_pending = false;
        if (state->topic_revision != revision) {return;}
        try {
          on_map(*state, future.get()->map, is_base, false);
        } catch (const std::exception & error) {
          RCLCPP_WARN(get_logger(), "Map service failed: %s", error.what());
        }
      });
    input.request_id = pending.request_id;
  }

  void poll_maps(bool force)
  {
    poll_map(base_input_, true, force);
    if (use_prohibition_) {poll_map(prohibition_input_, false, force);}
  }

  MapInput base_input_, prohibition_input_;
  Snapshot snapshot_;
  Grid dynamic_obstacles_, static_collision_, static_cost_;
  bool use_namespace_{false}, use_online_{false}, use_prohibition_{true}, require_prohibition_{true};
  bool circular_{false}, debug_{false}, ready_{false}, have_obstacles_{false};
  double static_radius_, dynamic_radius_, cost_radius_, topic_timeout_, request_timeout_;
  double tf_timeout_, robot_tf_max_age_;
  int cost_peak_, log_throttle_ms_;
  std::string base_link_, map_frame_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<Grid>::SharedPtr pub_augmented_, pub_planning_, pub_base_;
  rclcpp::Subscription<Grid>::SharedPtr sub_base_, sub_prohibition_, sub_obstacles_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_point_;
  std::vector<rclcpp::Service<GetMap>::SharedPtr> map_services_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_obstacles_, srv_inside_;
  rclcpp::TimerBase::SharedPtr refresh_timer_, publish_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapAugmenterNode>());
  rclcpp::shutdown();
  return 0;
}
