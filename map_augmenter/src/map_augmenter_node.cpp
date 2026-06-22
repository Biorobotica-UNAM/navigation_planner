#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

#include "nav_msgs/srv/get_map.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2/utils.h"
#include "Eigen/Geometry"

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <cmath>

class MapAugmenterNode : public rclcpp::Node
{
public:
  MapAugmenterNode()
  : Node("map_augmenter_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    // === Mismos params que tu nodo original ===
    this->declare_parameter("use_namespace",          false);
    this->declare_parameter("use_lidar",              false);
    this->declare_parameter("use_point_cloud",        false);
    this->declare_parameter("use_point_cloud2",       false);
    this->declare_parameter("use_online",             false);
    // Default false: solo launches SLAM explicitos deben permitir planear sobre desconocido.
    this->declare_parameter("unknown_as_free",        false);
    this->declare_parameter("free_space_padding",     0.0);
    this->declare_parameter("prefer_known_space",     false);
    this->declare_parameter("unknown_space_cost",     2);
    this->declare_parameter("unknown_space_cost_radius", 0.0);

    this->declare_parameter("laser_min_x",            -10.0);
    this->declare_parameter("laser_max_x",            10.0);
    this->declare_parameter("laser_min_y",            -10.0);
    this->declare_parameter("laser_max_y",            10.0);
    this->declare_parameter("laser_min_z",            -1.0);
    this->declare_parameter("laser_max_z",            2.0);

    this->declare_parameter("cloud_min_x",            -10.0);
    this->declare_parameter("cloud_max_x",            10.0);
    this->declare_parameter("cloud_min_y",            -10.0);
    this->declare_parameter("cloud_max_y",            10.0);
    this->declare_parameter("cloud_min_z",            -1.0);
    this->declare_parameter("cloud_max_z",            2.0);

    this->declare_parameter("decay_factor",           10);
    this->declare_parameter("robot_radius",           0.35);
    this->declare_parameter("static_obstacle_inflation_radius",  0.05);
    this->declare_parameter("dynamic_obstacle_inflation_radius", 0.15);
    this->declare_parameter("obstacle_inflation_radius",         -1.0);
    this->declare_parameter("inflation_radius",       0.15);
    this->declare_parameter("cost_radius",            0.35);

    this->declare_parameter("cloud_downsampling",     1);
    this->declare_parameter("cloud_downsampling2",    1);
    this->declare_parameter("lidar_downsampling",     1);

    this->declare_parameter("point_cloud_topic",      "/point_cloud");  // se mantiene por compatibilidad
    this->declare_parameter("point_cloud_topic2",     "/point_cloud2"); // se mantiene por compatibilidad
    this->declare_parameter("laser_scan_topic",       "/scan");         // se mantiene por compatibilidad

    this->declare_parameter("static_map_server",      "/static_map_server/map");
    this->declare_parameter("prohibition_map_server", "/prohibition_map_server/map");
    this->declare_parameter("base_link_name",         "base_footprint");
    this->declare_parameter("debug_maps",             false);

    // load
    this->get_parameter("use_namespace", use_namespace_);
    this->get_parameter("use_online", use_online_);
    this->get_parameter("unknown_as_free", unknown_as_free_);
    this->get_parameter("free_space_padding", free_space_padding_);
    this->get_parameter("prefer_known_space", prefer_known_space_);
    this->get_parameter("unknown_space_cost", unknown_space_cost_);
    this->get_parameter("unknown_space_cost_radius", unknown_space_cost_radius_);
    this->get_parameter("decay_factor", decay_factor_);
    this->get_parameter("robot_radius", robot_radius_);
    this->get_parameter("static_obstacle_inflation_radius", static_obstacle_inflation_radius_);
    this->get_parameter("dynamic_obstacle_inflation_radius", dynamic_obstacle_inflation_radius_);
    this->get_parameter("obstacle_inflation_radius", obstacle_inflation_radius_);
    this->get_parameter("inflation_radius", inflation_radius_);
    this->get_parameter("cost_radius", cost_radius_);
    this->get_parameter("static_map_server", static_map_server_);
    this->get_parameter("prohibition_map_server", prohibition_map_server_);
    this->get_parameter("base_link_name", base_link_name_);
    this->get_parameter("debug_maps", debug_maps_);

    if (static_obstacle_inflation_radius_ < 0.0) {
      static_obstacle_inflation_radius_ =
        obstacle_inflation_radius_ >= 0.0 ? obstacle_inflation_radius_ : inflation_radius_;
    }
    if (dynamic_obstacle_inflation_radius_ < 0.0) {
      dynamic_obstacle_inflation_radius_ =
        obstacle_inflation_radius_ >= 0.0 ? obstacle_inflation_radius_ : inflation_radius_;
    }
    robot_radius_ = std::max(0.0, robot_radius_);
    static_obstacle_inflation_radius_ = std::max(0.0, static_obstacle_inflation_radius_);
    dynamic_obstacle_inflation_radius_ = std::max(0.0, dynamic_obstacle_inflation_radius_);
    unknown_space_cost_ = std::clamp(unknown_space_cost_, 0, 99);
    unknown_space_cost_radius_ = std::max(0.0, unknown_space_cost_radius_);

    // param callback
    param_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&MapAugmenterNode::on_parameter_change, this, std::placeholders::_1));

    // clients mapas
    init_service_clients();

    // Publisher (MISMO)
    pub_augmented_map_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
      make_name("/augmented_map"),
      rclcpp::QoS(10).transient_local());
    pub_planning_map_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
      make_name("/planning_map"),
      rclcpp::QoS(10).transient_local());

    // Subs (MISMO point_obstacle) + NEW obstacles_map
    sub_clicked_point_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      make_name("/point_obstacle"),
      rclcpp::SensorDataQoS(),
      std::bind(&MapAugmenterNode::callback_point_obstacle, this, std::placeholders::_1));

    sub_obstacles_map_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      make_name("/obstacles_map"),
      rclcpp::QoS(10),
      std::bind(&MapAugmenterNode::callback_obstacles_map, this, std::placeholders::_1));

    sub_persistent_cloud_map_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      make_name("/persistent_cloud_map"),
      rclcpp::QoS(1).transient_local(),
      std::bind(&MapAugmenterNode::callback_persistent_cloud_map, this, std::placeholders::_1));

    // Services (MISMOS)
    srv_static_map_ = this->create_service<nav_msgs::srv::GetMap>(
      make_name("/map_augmenter/get_static_map"),
      std::bind(&MapAugmenterNode::callback_static_map, this, std::placeholders::_1, std::placeholders::_2));

    srv_static_cost_map_ = this->create_service<nav_msgs::srv::GetMap>(
      make_name("/map_augmenter/get_static_cost_map"),
      std::bind(&MapAugmenterNode::callback_static_cost_map, this, std::placeholders::_1, std::placeholders::_2));

    srv_augmented_map_ = this->create_service<nav_msgs::srv::GetMap>(
      make_name("/map_augmenter/get_augmented_map"),
      std::bind(&MapAugmenterNode::callback_augmented_map, this, std::placeholders::_1, std::placeholders::_2));

    srv_augmented_cost_map_ = this->create_service<nav_msgs::srv::GetMap>(
      make_name("/map_augmenter/get_augmented_cost_map"),
      std::bind(&MapAugmenterNode::callback_augmented_cost_map, this, std::placeholders::_1, std::placeholders::_2));

    srv_are_there_obstacles_ = this->create_service<std_srvs::srv::Trigger>(
      make_name("/map_augmenter/are_there_obstacles"),
      std::bind(&MapAugmenterNode::callback_are_there_obstacles, this, std::placeholders::_1, std::placeholders::_2));

    srv_is_inside_obstacles_ = this->create_service<std_srvs::srv::Trigger>(
      make_name("/map_augmenter/is_inside_obstacles"),
      std::bind(&MapAugmenterNode::callback_is_inside_obstacles, this, std::placeholders::_1, std::placeholders::_2));

    // Init mapas estáticos
    //get_first_maps();

    // processing igual que el tuyo (10 Hz timer + publica cada ~1s)
    processing_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&MapAugmenterNode::map_augmenter_processing, this));

    RCLCPP_INFO(this->get_logger(), "MapAugmenter.-> MapAugmenterNode (augmenter+merger) ready.");
  }

private:
  // ===== state =====
  int counter_ = 0;
  bool are_there_obstacles_ = false;

  nav_msgs::msg::OccupancyGrid static_map_;
  nav_msgs::msg::OccupancyGrid merged_static_raw_map_;
  nav_msgs::msg::OccupancyGrid static_obstacle_map_;
  nav_msgs::msg::OccupancyGrid prohibition_map_;
  nav_msgs::msg::OccupancyGrid static_cost_map_;
  nav_msgs::msg::OccupancyGrid unknown_space_cost_map_;
  nav_msgs::msg::OccupancyGrid persistent_cloud_map_;

  nav_msgs::msg::OccupancyGrid obstacles_map_;          // recibido
  nav_msgs::msg::OccupancyGrid obstacles_inflated_map_;
  nav_msgs::msg::OccupancyGrid obstacles_visual_inflated_map_;
  nav_msgs::msg::OccupancyGrid augmented_map_;
  nav_msgs::msg::OccupancyGrid visual_augmented_map_;

  std::atomic<bool> obstacles_received_{false};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // params
  bool use_namespace_{false};
  bool use_online_{false};
  bool unknown_as_free_{false};
  bool prefer_known_space_{false};
  double free_space_padding_{0.0};
  int unknown_space_cost_{2};
  double unknown_space_cost_radius_{0.0};
  std::uint64_t persistent_cloud_signature_{0};
  int decay_factor_{10};
  double robot_radius_{0.35};
  double static_obstacle_inflation_radius_{0.05};
  double dynamic_obstacle_inflation_radius_{0.15};
  double obstacle_inflation_radius_{-1.0};
  double inflation_radius_{0.15};
  double cost_radius_{0.35};
  std::string static_map_server_;
  std::string prohibition_map_server_;
  std::string base_link_name_;
  bool debug_maps_{false};

  // comms
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_augmented_map_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_planning_map_;

  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_clicked_point_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_obstacles_map_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_persistent_cloud_map_;

  rclcpp::Client<nav_msgs::srv::GetMap>::SharedPtr clt_get_static_map_;
  rclcpp::Client<nav_msgs::srv::GetMap>::SharedPtr clt_get_prohibition_map_;

  rclcpp::Service<nav_msgs::srv::GetMap>::SharedPtr srv_static_map_;
  rclcpp::Service<nav_msgs::srv::GetMap>::SharedPtr srv_static_cost_map_;
  rclcpp::Service<nav_msgs::srv::GetMap>::SharedPtr srv_augmented_map_;
  rclcpp::Service<nav_msgs::srv::GetMap>::SharedPtr srv_augmented_cost_map_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_are_there_obstacles_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_is_inside_obstacles_;

  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  rclcpp::TimerBase::SharedPtr service_check_timer_;
  rclcpp::TimerBase::SharedPtr retry_timer_;
  std::atomic<bool> online_refresh_pending_{false};


  bool services_ready_ = false;
  bool static_ready_ = false;
  std::atomic<bool> obstacles_dirty_{false};
  bool is_static_map_ = false;
  bool is_prohibition_map_ = false;

  rclcpp::TimerBase::SharedPtr processing_timer_;

  // ===== utils =====
  std::string make_name(const std::string &suffix) const
  {
    std::string sfx = suffix;
    if (!sfx.empty() && sfx.front() != '/') sfx = "/" + sfx;

    if (!use_namespace_) return sfx;

    std::string name = this->get_namespace() + sfx;
    if (name.size() > 1 && name[0] == '/' && name[1] == '/') name.erase(0, 1);
    return name;
  }

  rcl_interfaces::msg::SetParametersResult on_parameter_change(
    const std::vector<rclcpp::Parameter> &params)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    bool planning_cost_changed = false;
    bool unknown_preference_changed = false;

    for (const auto &p : params)
    {
      const auto &n = p.get_name();
      if (n == "use_namespace") use_namespace_ = p.as_bool();
      else if (n == "use_online") use_online_ = p.as_bool();
      else if (n == "unknown_as_free") unknown_as_free_ = p.as_bool();
      else if (n == "free_space_padding") free_space_padding_ = p.as_double();
      else if (n == "prefer_known_space") {
        prefer_known_space_ = p.as_bool();
        planning_cost_changed = true;
        unknown_preference_changed = true;
      }
      else if (n == "unknown_space_cost") {
        unknown_space_cost_ = std::clamp(static_cast<int>(p.as_int()), 0, 99);
        planning_cost_changed = true;
        unknown_preference_changed = true;
      }
      else if (n == "unknown_space_cost_radius") {
        unknown_space_cost_radius_ = std::max(0.0, p.as_double());
        planning_cost_changed = true;
        unknown_preference_changed = true;
      }
      else if (n == "decay_factor") decay_factor_ = p.as_int();
      else if (n == "robot_radius") robot_radius_ = std::max(0.0, p.as_double());
      else if (n == "static_obstacle_inflation_radius") static_obstacle_inflation_radius_ = std::max(0.0, p.as_double());
      else if (n == "dynamic_obstacle_inflation_radius") dynamic_obstacle_inflation_radius_ = std::max(0.0, p.as_double());
      else if (n == "obstacle_inflation_radius") {
        obstacle_inflation_radius_ = std::max(0.0, p.as_double());
        static_obstacle_inflation_radius_ = obstacle_inflation_radius_;
        dynamic_obstacle_inflation_radius_ = obstacle_inflation_radius_;
      }
      else if (n == "inflation_radius") inflation_radius_ = p.as_double();
      else if (n == "cost_radius") {
        cost_radius_ = std::max(0.0, p.as_double());
        planning_cost_changed = true;
      }
      else if (n == "static_map_server") static_map_server_ = p.as_string();
      else if (n == "prohibition_map_server") prohibition_map_server_ = p.as_string();
      else if (n == "base_link_name") base_link_name_ = p.as_string();
      else if (n == "debug_maps") debug_maps_ = p.as_bool();
      else {
        // Mantén compat: ignora los que no usa este nodo (sensores ahora viven en obstacle_detector)
      }
    }
    if (unknown_preference_changed && services_ready_) {
      get_first_maps();
    } else if (planning_cost_changed && static_ready_ && !static_map_.data.empty()) {
      static_cost_map_ = build_planning_cost_map(static_map_);
    }
    return result;
  }

  double static_navigation_clearance_radius() const
  {
    return std::max(0.0, static_obstacle_inflation_radius_);
  }

  double dynamic_navigation_clearance_radius() const
  {
    return std::max(0.0, dynamic_obstacle_inflation_radius_);
  }

  size_t count_occupied_cells(const nav_msgs::msg::OccupancyGrid &map) const
  {
    return static_cast<size_t>(std::count_if(
      map.data.begin(),
      map.data.end(),
      [](int8_t c) { return c > 0; }));
  }

  int max_cell_value(const nav_msgs::msg::OccupancyGrid &map) const
  {
    if (map.data.empty()) return 0;
    auto it = std::max_element(map.data.begin(), map.data.end());
    return static_cast<int>(*it);
  }

  void debug_log_map_stats(const char *context, bool force = false)
  {
    const size_t raw_static_occ = count_occupied_cells(merged_static_raw_map_);
    const size_t static_occ = count_occupied_cells(static_map_);
    const size_t static_visual_occ = count_occupied_cells(static_obstacle_map_);
    const size_t obs_raw_occ = count_occupied_cells(obstacles_map_);
    const size_t obs_inflated_occ = count_occupied_cells(obstacles_inflated_map_);
    const size_t obs_visual_occ = count_occupied_cells(obstacles_visual_inflated_map_);
    const size_t planning_occ = count_occupied_cells(augmented_map_);
    const size_t visual_occ = count_occupied_cells(visual_augmented_map_);
    const size_t static_cost_occ = count_occupied_cells(static_cost_map_);

    if (obs_raw_occ == 0 && planning_occ > static_occ) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "MapAugmenter.DEBUG[%s].-> Planning map grew without raw dynamic obstacles: static=%zu planning=%zu raw_static=%zu",
        context, static_occ, planning_occ, raw_static_occ);
    }

    if (!debug_maps_ && !force) return;

    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "MapAugmenter.DEBUG[%s].-> occ raw_static=%zu static_inflated=%zu static_visual=%zu static_cost=%zu obs_raw=%zu obs_inflated=%zu obs_visual=%zu planning=%zu visual=%zu max(obs=%d, planning=%d, visual=%d)",
      context,
      raw_static_occ,
      static_occ,
      static_visual_occ,
      static_cost_occ,
      obs_raw_occ,
      obs_inflated_occ,
      obs_visual_occ,
      planning_occ,
      visual_occ,
      max_cell_value(obstacles_map_),
      max_cell_value(augmented_map_),
      max_cell_value(visual_augmented_map_));
  }

  Eigen::Affine3d get_relative_position(const std::string &target_frame, const std::string &source_frame)
  {
    try {
      auto tf = tf_buffer_.lookupTransform(target_frame, source_frame, tf2::TimePointZero);
      return tf2::transformToEigen(tf.transform);
    } catch (...) {
      return Eigen::Affine3d::Identity();
    }
  }

  void convert_unknown_to_free(nav_msgs::msg::OccupancyGrid &map) const
  {
    if (!unknown_as_free_) return;

    // En modo SLAM permitimos navegar por celdas no observadas; los obstaculos
    // reales siguen llegando por el mapa de obstaculos y por potential_fields.
    for (auto &cell : map.data) {
      if (cell < 0) cell = 0;
    }
  }

  int8_t merge_cell(int8_t a, int8_t b) const
  {
    if (unknown_as_free_) {
      int av = a < 0 ? 0 : static_cast<int>(a);
      int bv = b < 0 ? 0 : static_cast<int>(b);
      return static_cast<int8_t>(std::max(av, bv));
    }

    if (a < 0 && b < 0) return -1;
    if (a < 0) return b > 0 ? b : static_cast<int8_t>(-1);
    if (b < 0) return a > 0 ? a : static_cast<int8_t>(-1);
    return static_cast<int8_t>(std::max(static_cast<int>(a), static_cast<int>(b)));
  }

  nav_msgs::msg::OccupancyGrid pad_map_with_free_space(const nav_msgs::msg::OccupancyGrid &map) const
  {
    if (!unknown_as_free_ || free_space_padding_ <= 0.0 ||
        map.info.resolution <= 0.0f || map.info.width == 0 || map.info.height == 0 ||
        map.data.empty()) {
      return map;
    }

    const int old_width = static_cast<int>(map.info.width);
    const int old_height = static_cast<int>(map.info.height);
    const int padding_cells = static_cast<int>(std::ceil(free_space_padding_ / map.info.resolution));
    if (padding_cells <= 0) return map;

    const int new_width = old_width + 2 * padding_cells;
    const int new_height = old_height + 2 * padding_cells;

    nav_msgs::msg::OccupancyGrid padded = map;
    padded.info.width = static_cast<uint32_t>(new_width);
    padded.info.height = static_cast<uint32_t>(new_height);
    padded.info.origin.position.x -= padding_cells * map.info.resolution;
    padded.info.origin.position.y -= padding_cells * map.info.resolution;
    padded.data.assign(static_cast<size_t>(new_width) * static_cast<size_t>(new_height), 0);

    for (int y = 0; y < old_height; ++y) {
      for (int x = 0; x < old_width; ++x) {
        const int old_idx = y * old_width + x;
        const int new_idx = (y + padding_cells) * new_width + (x + padding_cells);
        const int8_t value = map.data[old_idx];
        padded.data[new_idx] = value < 0 ? 0 : value;
      }
    }

    return padded;
  }

  nav_msgs::msg::OccupancyGrid pad_cost_map_with_value(
    const nav_msgs::msg::OccupancyGrid &map,
    const int8_t padding_value) const
  {
    if (!unknown_as_free_ || free_space_padding_ <= 0.0 ||
        map.info.resolution <= 0.0f || map.info.width == 0 || map.info.height == 0 ||
        map.data.empty()) {
      return map;
    }

    const int old_width = static_cast<int>(map.info.width);
    const int old_height = static_cast<int>(map.info.height);
    const int padding_cells = static_cast<int>(std::ceil(free_space_padding_ / map.info.resolution));
    if (padding_cells <= 0) return map;

    const int new_width = old_width + 2 * padding_cells;
    const int new_height = old_height + 2 * padding_cells;

    nav_msgs::msg::OccupancyGrid padded = map;
    padded.info.width = static_cast<uint32_t>(new_width);
    padded.info.height = static_cast<uint32_t>(new_height);
    padded.info.origin.position.x -= padding_cells * map.info.resolution;
    padded.info.origin.position.y -= padding_cells * map.info.resolution;
    padded.data.assign(
      static_cast<size_t>(new_width) * static_cast<size_t>(new_height),
      padding_value);

    for (int y = 0; y < old_height; ++y) {
      for (int x = 0; x < old_width; ++x) {
        const int old_idx = y * old_width + x;
        const int new_idx = (y + padding_cells) * new_width + (x + padding_cells);
        padded.data[new_idx] = map.data[old_idx];
      }
    }

    return padded;
  }

  bool same_geometry(const nav_msgs::msg::OccupancyGrid &a,
                     const nav_msgs::msg::OccupancyGrid &b) const
  {
    return a.info.width == b.info.width &&
           a.info.height == b.info.height &&
           std::abs(a.info.resolution - b.info.resolution) <= 1e-6 &&
           std::abs(a.info.origin.position.x - b.info.origin.position.x) <= 1e-6 &&
           std::abs(a.info.origin.position.y - b.info.origin.position.y) <= 1e-6;
  }

  nav_msgs::msg::OccupancyGrid burn_persistent_cloud_into_base(
    const nav_msgs::msg::OccupancyGrid &base_map)
  {
    if (base_map.data.empty() ||
        persistent_cloud_map_.data.empty()) {
      return base_map;
    }

    nav_msgs::msg::OccupancyGrid persistent_layer;
    if (same_geometry(base_map, persistent_cloud_map_)) {
      persistent_layer = persistent_cloud_map_;
    } else {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "MapAugmenter.-> persistent_cloud_map geometry does not match base map; reprojecting into base.");
      persistent_layer = project_grid_to_geometry(persistent_cloud_map_, base_map);
    }

    nav_msgs::msg::OccupancyGrid burned = base_map;
    for (size_t i = 0; i < burned.data.size(); ++i) {
      if (persistent_layer.data[i] > 0) {
        burned.data[i] = std::max(burned.data[i], persistent_layer.data[i]);
      }
    }
    return burned;
  }

  std::uint64_t persistent_cloud_signature(const nav_msgs::msg::OccupancyGrid &map) const
  {
    std::uint64_t signature = 1469598103934665603ULL;
    bool has_occupied = false;
    for (size_t i = 0; i < map.data.size(); ++i) {
      if (map.data[i] <= 0) continue;
      has_occupied = true;
      signature ^= static_cast<std::uint64_t>(i + 1);
      signature *= 1099511628211ULL;
      signature ^= static_cast<std::uint64_t>(static_cast<unsigned char>(map.data[i]));
      signature *= 1099511628211ULL;
    }
    return has_occupied ? signature : 0;
  }

  nav_msgs::msg::OccupancyGrid build_unknown_space_cost_map(
    const nav_msgs::msg::OccupancyGrid &raw_map) const
  {
    if (!prefer_known_space_ || unknown_space_cost_ <= 0 ||
        raw_map.info.resolution <= 0.0f || raw_map.data.empty()) {
      return nav_msgs::msg::OccupancyGrid();
    }

    nav_msgs::msg::OccupancyGrid unknown_cost = raw_map;
    const int8_t unknown_cost_value = static_cast<int8_t>(unknown_space_cost_);
    for (size_t i = 0; i < unknown_cost.data.size(); ++i) {
      unknown_cost.data[i] = raw_map.data[i] < 0 ? unknown_cost_value : 0;
    }

    unknown_cost = pad_cost_map_with_value(unknown_cost, unknown_cost_value);
    if (unknown_space_cost_radius_ <= 0.0) {
      return unknown_cost;
    }

    nav_msgs::msg::OccupancyGrid expanded = unknown_cost;
    const int steps = static_cast<int>(std::ceil(
      unknown_space_cost_radius_ / unknown_cost.info.resolution));
    if (steps <= 0) return unknown_cost;

    const int width = static_cast<int>(unknown_cost.info.width);
    const int height = static_cast<int>(unknown_cost.info.height);
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        const int idx = y * width + x;
        if (unknown_cost.data[idx] <= 0) continue;

        for (int dy = -steps; dy <= steps; ++dy) {
          for (int dx = -steps; dx <= steps; ++dx) {
            if (dx * dx + dy * dy > steps * steps) continue;
            const int nx = x + dx;
            const int ny = y + dy;
            if (nx < 0 || nx >= width || ny < 0 || ny >= height) continue;
            const int nidx = ny * width + nx;
            expanded.data[nidx] = std::max(expanded.data[nidx], unknown_cost_value);
          }
        }
      }
    }

    return expanded;
  }

  void apply_unknown_space_cost(nav_msgs::msg::OccupancyGrid &cost_map)
  {
    if (!prefer_known_space_ || unknown_space_cost_map_.data.empty()) return;

    if (!same_geometry(cost_map, unknown_space_cost_map_)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "MapAugmenter.-> unknown_space_cost_map geometry does not match cost map; known-space preference disabled for this update.");
      return;
    }

    for (size_t i = 0; i < cost_map.data.size(); ++i) {
      if (unknown_space_cost_map_.data[i] <= 0 || cost_map.data[i] >= 100) continue;
      const int updated = static_cast<int>(cost_map.data[i]) +
                          static_cast<int>(unknown_space_cost_map_.data[i]);
      cost_map.data[i] = static_cast<int8_t>(std::min(99, updated));
    }
  }

  nav_msgs::msg::OccupancyGrid project_obstacles_to_static_map(
      const nav_msgs::msg::OccupancyGrid &obstacles) const
  {
    return project_grid_to_geometry(obstacles, static_map_);
  }

  nav_msgs::msg::OccupancyGrid project_grid_to_geometry(
      const nav_msgs::msg::OccupancyGrid &source,
      const nav_msgs::msg::OccupancyGrid &target_geometry) const
  {
    nav_msgs::msg::OccupancyGrid projected = target_geometry;
    for (auto &cell : projected.data) cell = 0;

    if (source.info.resolution <= 0.0f || source.data.empty() ||
        target_geometry.info.resolution <= 0.0f || target_geometry.data.empty()) {
      return projected;
    }

    const int src_width = static_cast<int>(source.info.width);
    const int src_height = static_cast<int>(source.info.height);
    const int dst_width = static_cast<int>(projected.info.width);
    const int dst_height = static_cast<int>(projected.info.height);
    const double src_res = source.info.resolution;
    const double dst_res = projected.info.resolution;

    for (int y = 0; y < src_height; ++y) {
      for (int x = 0; x < src_width; ++x) {
        const int src_idx = y * src_width + x;
        if (src_idx < 0 || static_cast<size_t>(src_idx) >= source.data.size() ||
            source.data[src_idx] <= 0) {
          continue;
        }

        const double world_x = source.info.origin.position.x + (x + 0.5) * src_res;
        const double world_y = source.info.origin.position.y + (y + 0.5) * src_res;
        const int dst_x = static_cast<int>((world_x - projected.info.origin.position.x) / dst_res);
        const int dst_y = static_cast<int>((world_y - projected.info.origin.position.y) / dst_res);

        if (dst_x >= 0 && dst_x < dst_width && dst_y >= 0 && dst_y < dst_height) {
          projected.data[dst_y * dst_width + dst_x] = source.data[src_idx];
        }
      }
    }

    return projected;
  }

  nav_msgs::msg::OccupancyGrid merge_maps(const nav_msgs::msg::OccupancyGrid &a,
                                         const nav_msgs::msg::OccupancyGrid &b)
  {
    if (a.info.width != b.info.width || a.info.height != b.info.height) {
      RCLCPP_WARN(this->get_logger(), "MapAugmenter.-> WARNING!!! Cannot merge maps of different sizes!");
      return a;
    }

    nav_msgs::msg::OccupancyGrid c = a;
    for (size_t i = 0; i < c.data.size(); ++i) {
      c.data[i] = merge_cell(a.data[i], b.data[i]);
    }
    return c;
  }

  nav_msgs::msg::OccupancyGrid inflate_map(const nav_msgs::msg::OccupancyGrid &map, float inflation)
  {
    if (inflation <= 0) return map;

    nav_msgs::msg::OccupancyGrid new_map = map;
    int n = static_cast<int>(inflation / map.info.resolution);
    int lower_limit = n * map.info.width + n;
    int upper_limit = static_cast<int>(map.data.size()) - n * map.info.width - n;

    for (int k = lower_limit; k < upper_limit; ++k) {
      if (map.data[k] > 0) {
        for (int i = -n; i <= n; ++i) {
          for (int j = -n; j <= n; ++j) {
            int idx = k + j * map.info.width + i;
            if (idx >= 0 && idx < static_cast<int>(new_map.data.size())) {
              new_map.data[idx] = map.data[k];
            }
          }
        }
      }
    }
    return new_map;
  }

  nav_msgs::msg::OccupancyGrid get_cost_map(const nav_msgs::msg::OccupancyGrid &map, float cost_radius)
  {
    if (cost_radius < 0) return map;

    nav_msgs::msg::OccupancyGrid cost_map = map;
    int steps = static_cast<int>(cost_radius / map.info.resolution);
    int box_size = (steps * 2 + 1) * (steps * 2 + 1);
    std::vector<int> cell_costs(box_size);
    std::vector<int> neighbors(box_size);

    int counter = 0;
    for (int i = -steps; i <= steps; ++i) {
      for (int j = -steps; j <= steps; ++j) {
        neighbors[counter] = i * map.info.width + j;
        cell_costs[counter] = (steps - std::max(std::abs(i), std::abs(j)) + 1) * 2;
        ++counter;
      }
    }

    int start_idx = steps * map.info.width + steps;
    int end_idx = static_cast<int>(map.data.size()) - steps * map.info.width - steps;

    for (int i = start_idx; i < end_idx; ++i) {
      if (map.data[i] > 0) {
        for (int j = 0; j < box_size; ++j) {
          int neighbor_idx = i + neighbors[j];
          if (neighbor_idx >= 0 && neighbor_idx < static_cast<int>(cost_map.data.size())) {
            if (cost_map.data[neighbor_idx] < cell_costs[j]) {
              cost_map.data[neighbor_idx] = static_cast<int8_t>(cell_costs[j]);
            }
          }
        }
      }
    }
    return cost_map;
  }

  nav_msgs::msg::OccupancyGrid build_planning_cost_map(const nav_msgs::msg::OccupancyGrid &planning_map)
  {
    auto cost_map = get_cost_map(planning_map, static_cast<float>(cost_radius_));
    apply_unknown_space_cost(cost_map);
    return cost_map;
  }

  void get_relative_position(const std::string &target_frame, const std::string &source_frame,
                            float &robot_x, float &robot_y, float &robot_t)
  {
    try {
      auto tf = tf_buffer_.lookupTransform(target_frame, source_frame, tf2::TimePointZero);
      robot_x = static_cast<float>(tf.transform.translation.x);
      robot_y = static_cast<float>(tf.transform.translation.y);
      tf2::Quaternion q;
      tf2::fromMsg(tf.transform.rotation, q);
      robot_t = static_cast<float>(tf2::getYaw(q));
    } catch (...) {
      robot_x = robot_y = robot_t = 0.0f;
    }
  }

  // ===== clients/maps init =====
  void init_service_clients()
  {
    clt_get_static_map_ = this->create_client<nav_msgs::srv::GetMap>(static_map_server_);
    clt_get_prohibition_map_ = this->create_client<nav_msgs::srv::GetMap>(prohibition_map_server_);

    service_check_timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      [this]() {
        bool ok1 = clt_get_static_map_->wait_for_service(std::chrono::seconds(0));
        bool ok2 = clt_get_prohibition_map_->wait_for_service(std::chrono::seconds(0));
        if (ok1 && ok2) {
          RCLCPP_INFO(this->get_logger(), "MapAugmenter.-> All map services are now available.");
          services_ready_ = true;
          get_first_maps();
          service_check_timer_->cancel();
        } else {
          if (!ok1) RCLCPP_WARN(this->get_logger(), "MapAugmenter.-> Waiting for static_map service...");
          if (!ok2) RCLCPP_WARN(this->get_logger(), "MapAugmenter.-> Waiting for prohibition_map service...");
        }
      }
    );
  }

  void get_first_maps()
  {
    rclcpp::sleep_for(std::chrono::seconds(1));

    auto req1 = std::make_shared<nav_msgs::srv::GetMap::Request>();
    clt_get_static_map_->async_send_request(req1,
      [this](rclcpp::Client<nav_msgs::srv::GetMap>::SharedFuture fut) {
        try {
          static_map_ = fut.get()->map;
          is_static_map_ = true;
          process_maps();
        } catch (const std::exception &e) {
          RCLCPP_ERROR(this->get_logger(), "MapAugmenter.-> Failed to get static map: %s", e.what());
        }
      });

    auto req2 = std::make_shared<nav_msgs::srv::GetMap::Request>();
    clt_get_prohibition_map_->async_send_request(req2,
      [this](rclcpp::Client<nav_msgs::srv::GetMap>::SharedFuture fut) {
        try {
          prohibition_map_ = fut.get()->map;
          is_prohibition_map_ = true;
          process_maps();
        } catch (const std::exception &e) {
          RCLCPP_ERROR(this->get_logger(), "MapAugmenter.-> Failed to get prohibition map: %s", e.what());
        }
      });
  }

  void process_maps()
  {
    if (!(is_static_map_ && is_prohibition_map_)) return;

    // SLAM map not built yet — retry rather than publishing an empty map
    if (static_map_.info.width == 0 || static_map_.info.height == 0) {
      RCLCPP_WARN(this->get_logger(),
        "MapAugmenter.-> Static map is empty (SLAM not ready), retrying in 3s...");
      is_static_map_ = false;
      // Keep is_prohibition_map_ / prohibition_map_ — no need to re-fetch the static prohibition
      if (retry_timer_) retry_timer_->cancel();
      retry_timer_ = this->create_wall_timer(std::chrono::seconds(3), [this]() {
        retry_timer_->cancel();
        if (!services_ready_) return;
        auto req = std::make_shared<nav_msgs::srv::GetMap::Request>();
        clt_get_static_map_->async_send_request(req,
          [this](rclcpp::Client<nav_msgs::srv::GetMap>::SharedFuture fut) {
            try {
              static_map_ = fut.get()->map;
              is_static_map_ = true;
              process_maps();
            } catch (const std::exception &e) {
              RCLCPP_ERROR(get_logger(),
                "MapAugmenter.-> Static map retry failed: %s", e.what());
            }
          });
      });
      return;
    }

    unknown_space_cost_map_ = build_unknown_space_cost_map(static_map_);
    convert_unknown_to_free(static_map_);
    auto base_map = pad_map_with_free_space(static_map_);
    const auto merged_static_map = merge_maps(base_map, prohibition_map_);
    merged_static_raw_map_ = merged_static_map;
    const auto effective_static_map = burn_persistent_cloud_into_base(merged_static_map);
    static_obstacle_map_ = inflate_map(
      effective_static_map,
      static_cast<float>(std::max(0.0, static_obstacle_inflation_radius_)));
    static_map_ = inflate_map(
      effective_static_map,
      static_cast<float>(static_navigation_clearance_radius()));
    static_cost_map_ = build_planning_cost_map(static_map_);

    // init obstáculos con geometría correcta (si todavía no llega del detector)
    obstacles_map_ = static_map_;
    for (auto &c : obstacles_map_.data) c = 0;

    static_ready_ = true;

    is_static_map_ = false;
    is_prohibition_map_ = false;
    RCLCPP_INFO(this->get_logger(),
      "MapAugmenter.-> Static maps updated. robot_radius=%.2f static_obstacle_radius=%.2f dynamic_obstacle_radius=%.2f cost_radius=%.2f static_clearance=%.2f dynamic_clearance=%.2f",
      robot_radius_, static_obstacle_inflation_radius_, dynamic_obstacle_inflation_radius_,
      cost_radius_, static_navigation_clearance_radius(), dynamic_navigation_clearance_radius());
    debug_log_map_stats("process_maps", true);
  }

  void publish_augmented_now()
  {
    if (!services_ready_ || !static_ready_) return;

    // Si quieres evitar “double-decay”, deja decay_factor=0 en este nodo desde launch.
    // Si NO, puedes dejar esta línea para mantener la lógica original:
    are_there_obstacles_ = decay_map_and_check_if_obstacles(obstacles_map_, decay_factor_);

    obstacles_visual_inflated_map_ = inflate_map(
      obstacles_map_,
      static_cast<float>(std::max(0.0, dynamic_obstacle_inflation_radius_)));
    obstacles_inflated_map_ = inflate_map(obstacles_map_, dynamic_navigation_clearance_radius());
    augmented_map_ = merge_maps(static_map_, obstacles_inflated_map_);
    visual_augmented_map_ = merge_maps(static_obstacle_map_, obstacles_visual_inflated_map_);
    debug_log_map_stats("publish_now");

    pub_augmented_map_->publish(visual_augmented_map_);
    pub_planning_map_->publish(augmented_map_);
  }


  // ===== callbacks =====
  /*void callback_obstacles_map(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    obstacles_map_ = *msg;
    obstacles_received_ = true;
  }*/

  void callback_obstacles_map(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    // Aún no hay base -> no aceptes nada
    if (!static_ready_ || static_map_.info.width == 0 || static_map_.info.height == 0) {
      return;
    }

    // En SLAM el mapa puede crecer o llevar padding libre; si cambia la geometria,
    // reproyectamos los obstaculos al mapa base actual en vez de perderlos.
    if (!same_geometry(*msg, static_map_)) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "MapAugmenter.-> Reprojecting obstacles_map: geometry mismatch (obs %ux%u res=%.3f) vs (static %ux%u res=%.3f)",
        msg->info.width, msg->info.height, msg->info.resolution,
        static_map_.info.width, static_map_.info.height, static_map_.info.resolution);
      obstacles_map_ = project_obstacles_to_static_map(*msg);
    } else {
      obstacles_map_ = *msg;
    }

    obstacles_dirty_ = true;
    publish_augmented_now();
    debug_log_map_stats("obstacles_callback");
  }

  void callback_persistent_cloud_map(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    const auto previous_signature = persistent_cloud_signature_;
    persistent_cloud_map_ = *msg;
    persistent_cloud_signature_ = persistent_cloud_signature(persistent_cloud_map_);

    if (services_ready_ && persistent_cloud_signature_ != previous_signature) {
      get_first_maps();
    }
  }


  void callback_point_obstacle(const geometry_msgs::msg::PointStamped::SharedPtr /*msg*/)
  {
    // Mantengo tu comportamiento: re-leer mapas al recibir point_obstacle
    rclcpp::sleep_for(std::chrono::seconds(1));
    get_first_maps();
  }

  // ===== services =====
  void callback_static_map(const std::shared_ptr<nav_msgs::srv::GetMap::Request> req,
                           std::shared_ptr<nav_msgs::srv::GetMap::Response> res)
  { (void)req; res->map = static_map_; }

  void callback_static_cost_map(const std::shared_ptr<nav_msgs::srv::GetMap::Request> req,
                                std::shared_ptr<nav_msgs::srv::GetMap::Response> res)
  { (void)req; res->map = static_cost_map_; }

  /*void callback_augmented_map(const std::shared_ptr<nav_msgs::srv::GetMap::Request>,
                              std::shared_ptr<nav_msgs::srv::GetMap::Response> res)
  {
    if (!services_ready_) {
      RCLCPP_ERROR(this->get_logger(), "MapAugmenter.-> Services not ready.");
      return;
    }

    if (use_online_) get_first_maps(); // mantengo tu idea “online” (no bloquea)

    // si todavía no llega obstacles_map, devolvemos lo que tenemos (cero)
    if (!obstacles_received_) {
      // ok: usa obstacles_map_ inicializado en process_maps()
    }

    obstacles_inflated_map_ = inflate_map(obstacles_map_, static_cast<float>(dynamic_navigation_clearance_radius()));
    augmented_map_ = merge_maps(static_map_, obstacles_inflated_map_);
    res->map = augmented_map_;
  }*/

  void callback_augmented_map(const std::shared_ptr<nav_msgs::srv::GetMap::Request> request,
                            std::shared_ptr<nav_msgs::srv::GetMap::Response> response)
  {
    (void)request;

    if (!services_ready_ || !static_ready_) {
      RCLCPP_ERROR(this->get_logger(), "MapAugmenter.-> Base map not ready. Cannot serve augmented map.");
      return;
    }

    obstacles_visual_inflated_map_ = inflate_map(
      obstacles_map_,
      static_cast<float>(std::max(0.0, dynamic_obstacle_inflation_radius_)));
    obstacles_inflated_map_ = inflate_map(obstacles_map_, dynamic_navigation_clearance_radius());
    augmented_map_ = merge_maps(static_map_, obstacles_inflated_map_);
    visual_augmented_map_ = merge_maps(static_obstacle_map_, obstacles_visual_inflated_map_);
    debug_log_map_stats("get_augmented_map");
    response->map = augmented_map_;
  }

  void callback_augmented_cost_map(const std::shared_ptr<nav_msgs::srv::GetMap::Request> req,
                                   std::shared_ptr<nav_msgs::srv::GetMap::Response> res)
  {
    (void)req;
    auto obs_cost = get_cost_map(obstacles_inflated_map_, static_cast<float>(cost_radius_));
    res->map = merge_maps(static_cost_map_, obs_cost);
  }

  void callback_are_there_obstacles(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                                    std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  { (void)req; res->success = are_there_obstacles_; }

  void callback_is_inside_obstacles(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                                    std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    (void)req;
    float rx, ry, ra;
    get_relative_position("map", base_link_name_, rx, ry, ra);

    int cell_x = static_cast<int>((rx - augmented_map_.info.origin.position.x) / augmented_map_.info.resolution);
    int cell_y = static_cast<int>((ry - augmented_map_.info.origin.position.y) / augmented_map_.info.resolution);
    int idx = cell_y * static_cast<int>(augmented_map_.info.width) + cell_x;

    res->success = (idx >= 0 &&
                    static_cast<size_t>(idx) < augmented_map_.data.size() &&
                    augmented_map_.data[idx] > 0);
  }

  // ===== main loop =====
  bool decay_map_and_check_if_obstacles(nav_msgs::msg::OccupancyGrid& map, int decay_factor)
  {
    bool obstacles = false;
    for (auto &c : map.data) {
      int v = static_cast<int>(c) - decay_factor;
      c = static_cast<int8_t>(v < 0 ? 0 : v);
      obstacles |= (c > 0);
    }
    return obstacles;
  }

  void map_augmenter_processing()
  {
    if (!services_ready_) {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                            "MapAugmenter.-> Services not ready.");
      return;
    }

    if (!static_ready_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                          "MapAugmenter.-> Base map not ready yet.");
      return;
    }

    static int counter = 0;
    if (++counter > 10) {
      counter = 0;

      // Si el decay ya lo hace el detector, deja decay_factor=0 en este nodo (launch)
      are_there_obstacles_ = decay_map_and_check_if_obstacles(obstacles_map_, decay_factor_);
      obstacles_visual_inflated_map_ = inflate_map(
        obstacles_map_,
        static_cast<float>(std::max(0.0, dynamic_obstacle_inflation_radius_)));
      obstacles_inflated_map_ = inflate_map(obstacles_map_, dynamic_navigation_clearance_radius());
      augmented_map_ = merge_maps(static_map_, obstacles_inflated_map_);
      visual_augmented_map_ = merge_maps(static_obstacle_map_, obstacles_visual_inflated_map_);
      debug_log_map_stats("timer_publish");
      pub_augmented_map_->publish(visual_augmented_map_);
      pub_planning_map_->publish(augmented_map_);
    }

    // In SLAM mode (use_online_=true): refresh the static map every ~5s so the growing
    // SLAM map propagates into the augmented map as the robot explores new areas.
    if (use_online_ && !online_refresh_pending_) {
      static int online_counter = 0;
      if (++online_counter > 50) {
        online_counter = 0;
        online_refresh_pending_ = true;
        auto req = std::make_shared<nav_msgs::srv::GetMap::Request>();
        clt_get_static_map_->async_send_request(req,
          [this](rclcpp::Client<nav_msgs::srv::GetMap>::SharedFuture fut) {
            online_refresh_pending_ = false;
            try {
              auto new_map = fut.get()->map;
              if (new_map.info.width == 0) return;
              unknown_space_cost_map_ = build_unknown_space_cost_map(new_map);
              convert_unknown_to_free(new_map);
              new_map = pad_map_with_free_space(new_map);

              // merge_maps returns new_map when sizes differ (blank prohibition = no-op anyway)
              auto merged = merge_maps(new_map, prohibition_map_);
              merged_static_raw_map_ = merged;
              const auto effective_static_map = burn_persistent_cloud_into_base(merged);
              static_obstacle_map_ = inflate_map(
                effective_static_map,
                static_cast<float>(std::max(0.0, static_obstacle_inflation_radius_)));
              auto planning_static = inflate_map(
                effective_static_map,
                static_cast<float>(static_navigation_clearance_radius()));
              static_cost_map_ = build_planning_cost_map(planning_static);

              // If the SLAM map grew, reset obstacles_map_ to the new size
              if (!same_geometry(planning_static, static_map_)) {
                obstacles_map_ = planning_static;
                for (auto &c : obstacles_map_.data) c = 0;
              }

              static_map_ = planning_static;
            } catch (...) {}
          });
      }
    }
  }

};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MapAugmenterNode>();
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
