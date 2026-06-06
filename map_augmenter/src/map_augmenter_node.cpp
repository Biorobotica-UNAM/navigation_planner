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

    // load
    this->get_parameter("use_namespace", use_namespace_);
    this->get_parameter("use_online", use_online_);
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
  nav_msgs::msg::OccupancyGrid static_obstacle_map_;
  nav_msgs::msg::OccupancyGrid prohibition_map_;
  nav_msgs::msg::OccupancyGrid static_cost_map_;

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

  // comms
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_augmented_map_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_planning_map_;

  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_clicked_point_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_obstacles_map_;

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
    bool cost_radius_changed = false;

    for (const auto &p : params)
    {
      const auto &n = p.get_name();
      if (n == "use_namespace") use_namespace_ = p.as_bool();
      else if (n == "use_online") use_online_ = p.as_bool();
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
        cost_radius_changed = true;
      }
      else if (n == "static_map_server") static_map_server_ = p.as_string();
      else if (n == "prohibition_map_server") prohibition_map_server_ = p.as_string();
      else if (n == "base_link_name") base_link_name_ = p.as_string();
      else {
        // Mantén compat: ignora los que no usa este nodo (sensores ahora viven en obstacle_detector)
      }
    }
    if (cost_radius_changed && static_ready_ && !static_map_.data.empty()) {
      static_cost_map_ = get_cost_map(static_map_, static_cast<float>(cost_radius_));
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

  Eigen::Affine3d get_relative_position(const std::string &target_frame, const std::string &source_frame)
  {
    try {
      auto tf = tf_buffer_.lookupTransform(target_frame, source_frame, tf2::TimePointZero);
      return tf2::transformToEigen(tf.transform);
    } catch (...) {
      return Eigen::Affine3d::Identity();
    }
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
      c.data[i] = static_cast<int8_t>(
        std::max(static_cast<uint8_t>(a.data[i]), static_cast<uint8_t>(b.data[i])));
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

    const auto merged_static_map = merge_maps(static_map_, prohibition_map_);
    static_obstacle_map_ = inflate_map(
      merged_static_map,
      static_cast<float>(std::max(0.0, static_obstacle_inflation_radius_)));
    static_map_ = inflate_map(
      merged_static_map,
      static_cast<float>(static_navigation_clearance_radius()));
    static_cost_map_ = get_cost_map(static_map_, static_cast<float>(cost_radius_));

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

    // Geometría distinta -> ignora (evita spam de merge)
    if (msg->info.width  != static_map_.info.width ||
        msg->info.height != static_map_.info.height ||
        std::abs(msg->info.resolution - static_map_.info.resolution) > 1e-6)
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "MapAugmenter.-> Ignoring obstacles_map: geometry mismatch (obs %ux%u res=%.3f) vs (static %ux%u res=%.3f)",
        msg->info.width, msg->info.height, msg->info.resolution,
        static_map_.info.width, static_map_.info.height, static_map_.info.resolution);
      return;
    }

    obstacles_map_ = *msg;
    obstacles_dirty_ = true;
    publish_augmented_now();
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
      pub_augmented_map_->publish(visual_augmented_map_);
      pub_planning_map_->publish(augmented_map_);
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
