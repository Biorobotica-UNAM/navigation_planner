#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/srv/get_map.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2/utils.h"
#include "Eigen/Geometry"

#include <atomic>
#include <cmath>
#include <unordered_map>

class ObstacleDetectorNode : public rclcpp::Node
{
public:
  ObstacleDetectorNode()
  : Node("obstacle_detector_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    // === MISMA lista de params (para compatibilidad con tu YAML) ===
    this->declare_parameter("use_namespace",          false);
    this->declare_parameter("use_lidar",              false);
    this->declare_parameter("use_point_cloud",        false);
    this->declare_parameter("use_point_cloud2",       false);
    this->declare_parameter("use_online",             false); // no se usa aquí, pero se mantiene

    this->declare_parameter("laser_min_x",            -10.0);
    this->declare_parameter("laser_max_x",             10.0);
    this->declare_parameter("laser_min_y",            -10.0);
    this->declare_parameter("laser_max_y",             10.0);
    this->declare_parameter("laser_min_z",             -1.0);
    this->declare_parameter("laser_max_z",              2.0);

    this->declare_parameter("cloud_min_x",            -10.0);
    this->declare_parameter("cloud_max_x",             10.0);
    this->declare_parameter("cloud_min_y",            -10.0);
    this->declare_parameter("cloud_max_y",             10.0);
    this->declare_parameter("cloud_min_z",             -1.0);
    this->declare_parameter("cloud_max_z",              2.0);

    this->declare_parameter("decay_factor",           10);
    this->declare_parameter("inflation_radius",       0.25); // se mantiene por compatibilidad
    this->declare_parameter("cost_radius",            0.25);  // se mantiene por compatibilidad

    this->declare_parameter("cloud_downsampling",     1);
    this->declare_parameter("cloud_downsampling2",    1);
    this->declare_parameter("lidar_downsampling",     1);
    this->declare_parameter("cloud_min_points_per_cell", 2);
    this->declare_parameter("cloud_min_neighbor_cells",  1);
    this->declare_parameter("tf_fallback_warn_threshold_sec", 0.20);

    this->declare_parameter("point_cloud_topic",      "/point_cloud");
    this->declare_parameter("point_cloud_topic2",     "/point_cloud2");
    this->declare_parameter("laser_scan_topic",       "/scan");

    this->declare_parameter("static_map_server",      "/static_map_server/map");
    this->declare_parameter("prohibition_map_server", "/prohibition_map_server/map"); // no se usa aquí
    this->declare_parameter("base_link_name",         "base_footprint");

    this->declare_parameter("use_ground_filter", false);
    this->declare_parameter("use_ransac", true);
    this->declare_parameter("ground_height_threshold", 0.05);
    this->declare_parameter("ransac_distance_threshold", 0.03);

    // === load params ===
    this->get_parameter("use_namespace", use_namespace_);
    this->get_parameter("use_lidar", use_lidar_);
    this->get_parameter("use_point_cloud", use_cloud_);
    this->get_parameter("use_point_cloud2", use_cloud2_);

    this->get_parameter("laser_min_x", laser_min_x_);
    this->get_parameter("laser_max_x", laser_max_x_);
    this->get_parameter("laser_min_y", laser_min_y_);
    this->get_parameter("laser_max_y", laser_max_y_);
    this->get_parameter("laser_min_z", laser_min_z_);
    this->get_parameter("laser_max_z", laser_max_z_);

    this->get_parameter("cloud_min_x", cloud_min_x_);
    this->get_parameter("cloud_max_x", cloud_max_x_);
    this->get_parameter("cloud_min_y", cloud_min_y_);
    this->get_parameter("cloud_max_y", cloud_max_y_);
    this->get_parameter("cloud_min_z", cloud_min_z_);
    this->get_parameter("cloud_max_z", cloud_max_z_);

    this->get_parameter("decay_factor", decay_factor_);
    this->get_parameter("cloud_downsampling", cloud_downsampling_);
    this->get_parameter("cloud_downsampling2", cloud_downsampling2_);
    this->get_parameter("lidar_downsampling", lidar_downsampling_);
    this->get_parameter("cloud_min_points_per_cell", cloud_min_points_per_cell_);
    this->get_parameter("cloud_min_neighbor_cells", cloud_min_neighbor_cells_);
    this->get_parameter("tf_fallback_warn_threshold_sec", tf_fallback_warn_threshold_sec_);

    this->get_parameter("point_cloud_topic", point_cloud_topic_);
    this->get_parameter("point_cloud_topic2", point_cloud_topic2_);
    this->get_parameter("laser_scan_topic", laser_scan_topic_);
    this->get_parameter("static_map_server", static_map_server_);
    this->get_parameter("base_link_name", base_link_name_);

    // param callback (opcional pero útil si afinas rápido)
    param_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&ObstacleDetectorNode::on_parameter_change, this, std::placeholders::_1));

    // Publisher: mapa de obstáculos (interno)
    pub_obstacles_map_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
      make_name("/obstacles_map"),
      rclcpp::QoS(1).transient_local()); // latched para que el merger tenga algo inmediato

    // Subs sensores
    sub_point_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      point_cloud_topic_, rclcpp::SensorDataQoS(),
      std::bind(&ObstacleDetectorNode::cb_cloud1, this, std::placeholders::_1));

    sub_point_cloud2_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      point_cloud_topic2_, rclcpp::SensorDataQoS(),
      std::bind(&ObstacleDetectorNode::cb_cloud2, this, std::placeholders::_1));;

    sub_laser_scan_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      laser_scan_topic_, rclcpp::SensorDataQoS(),
      std::bind(&ObstacleDetectorNode::cb_scan, this, std::placeholders::_1));

    // Cliente: static map (solo para geometry del grid)
    clt_get_static_map_ = this->create_client<nav_msgs::srv::GetMap>(static_map_server_);

    // Espera servicio + pide mapa una vez
    wait_map_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&ObstacleDetectorNode::try_get_static_map_once, this));

    // Loop rápido: decay + integración + publish
    // (aquí es donde lo haces “rápido”)
    processing_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(75), // 20 Hz
      std::bind(&ObstacleDetectorNode::processing, this));

    RCLCPP_INFO(this->get_logger(), "ObstacleDetector.-> obstacle_detector_node ready.");
  }

private:
  // ====== state ======
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  nav_msgs::msg::OccupancyGrid obstacles_map_;
  nav_msgs::msg::OccupancyGrid static_reference_map_;
  std::atomic<bool> have_grid_{false};

  // latest sensors
  sensor_msgs::msg::PointCloud2::SharedPtr latest_cloud1_;
  sensor_msgs::msg::PointCloud2::SharedPtr latest_cloud2_;
  sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;

  std::atomic<bool> cloud1_received_{false};
  std::atomic<bool> cloud2_received_{false};
  std::atomic<bool> scan_received_{false};

  // params
  bool use_namespace_{false};
  bool use_lidar_{false};
  bool use_cloud_{false};
  bool use_cloud2_{false};

  float laser_min_x_, laser_max_x_, laser_min_y_, laser_max_y_, laser_min_z_, laser_max_z_;
  float cloud_min_x_, cloud_max_x_, cloud_min_y_, cloud_max_y_, cloud_min_z_, cloud_max_z_;
  int decay_factor_{10};
  int cloud_downsampling_{1}, cloud_downsampling2_{1}, lidar_downsampling_{1};
  int cloud_min_points_per_cell_{2};
  int cloud_min_neighbor_cells_{1};
  double tf_fallback_warn_threshold_sec_{0.20};

  std::string point_cloud_topic_, point_cloud_topic2_, laser_scan_topic_;
  std::string static_map_server_;
  std::string base_link_name_;

  // comms
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_obstacles_map_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_point_cloud_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_point_cloud2_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_laser_scan_;

  rclcpp::Client<nav_msgs::srv::GetMap>::SharedPtr clt_get_static_map_;

  rclcpp::TimerBase::SharedPtr wait_map_timer_;
  rclcpp::TimerBase::SharedPtr processing_timer_;

  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

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

    for (const auto &p : params)
    {
      const auto &n = p.get_name();
      if      (n == "use_namespace")    use_namespace_ = p.as_bool();
      else if (n == "use_lidar")        use_lidar_     = p.as_bool();
      else if (n == "use_point_cloud")  use_cloud_     = p.as_bool();
      else if (n == "use_point_cloud2") use_cloud2_    = p.as_bool();

      else if (n == "laser_min_x") laser_min_x_ = p.as_double();
      else if (n == "laser_max_x") laser_max_x_ = p.as_double();
      else if (n == "laser_min_y") laser_min_y_ = p.as_double();
      else if (n == "laser_max_y") laser_max_y_ = p.as_double();
      else if (n == "laser_min_z") laser_min_z_ = p.as_double();
      else if (n == "laser_max_z") laser_max_z_ = p.as_double();

      else if (n == "cloud_min_x") cloud_min_x_ = p.as_double();
      else if (n == "cloud_max_x") cloud_max_x_ = p.as_double();
      else if (n == "cloud_min_y") cloud_min_y_ = p.as_double();
      else if (n == "cloud_max_y") cloud_max_y_ = p.as_double();
      else if (n == "cloud_min_z") cloud_min_z_ = p.as_double();
      else if (n == "cloud_max_z") cloud_max_z_ = p.as_double();

      else if (n == "decay_factor") decay_factor_ = p.as_int();
      else if (n == "cloud_downsampling")  cloud_downsampling_  = p.as_int();
      else if (n == "cloud_downsampling2") cloud_downsampling2_ = p.as_int();
      else if (n == "lidar_downsampling")  lidar_downsampling_  = p.as_int();
      else if (n == "cloud_min_points_per_cell") cloud_min_points_per_cell_ = p.as_int();
      else if (n == "cloud_min_neighbor_cells")  cloud_min_neighbor_cells_  = p.as_int();
      else if (n == "tf_fallback_warn_threshold_sec") {
        tf_fallback_warn_threshold_sec_ = std::max(0.0, p.as_double());
      }

      else if (n == "point_cloud_topic")  point_cloud_topic_  = p.as_string();
      else if (n == "point_cloud_topic2") point_cloud_topic2_ = p.as_string();
      else if (n == "laser_scan_topic")   laser_scan_topic_   = p.as_string();
      else if (n == "static_map_server")  static_map_server_  = p.as_string();
      else if (n == "base_link_name")     base_link_name_     = p.as_string();
      else {
        // Para no romper, aquí NO marcamos fail (tu YAML tiene más params que este nodo mantiene).
        // Solo ignoramos lo que no usa el detector.
      }
    }
    return result;
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

  bool lookup_transform_with_latest_fallback(
    const std::string &target_frame,
    const std::string &source_frame,
    const rclcpp::Time &stamp,
    Eigen::Affine3d &transform)
  {
    try {
      auto tf = tf_buffer_.lookupTransform(
        target_frame,
        source_frame,
        stamp,
        rclcpp::Duration::from_seconds(0.03));
      transform = tf2::transformToEigen(tf.transform);
      return true;
    } catch (const std::exception &exact_error) {
      try {
        // Los sensores pueden llegar unos milisegundos por delante del ultimo
        // TF publicado. Usar la TF mas reciente como respaldo evita perder
        // frames completos y mantiene estable /navigation/obstacles_map.
        auto tf = tf_buffer_.lookupTransform(
          target_frame,
          source_frame,
          rclcpp::Time(0),
          rclcpp::Duration::from_seconds(0.03));
        transform = tf2::transformToEigen(tf.transform);
        const double tf_delta_sec =
          std::fabs((stamp - rclcpp::Time(tf.header.stamp)).seconds());
        if (tf_delta_sec > tf_fallback_warn_threshold_sec_) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 3000,
            "ObstacleDetector.-> Exact TF failed from '%s' to '%s' by %.3f s, using latest TF: %s",
            source_frame.c_str(),
            target_frame.c_str(),
            tf_delta_sec,
            exact_error.what());
        }
        return true;
      } catch (const std::exception &latest_error) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "ObstacleDetector.-> TF lookup failed from '%s' to '%s': %s",
          source_frame.c_str(),
          target_frame.c_str(),
          latest_error.what());
        return false;
      }
    }
  }

  // ===== map init =====
  /*void try_get_static_map_once()
  {
    if (have_grid_) {
      //wait_map_timer_->cancel();
      return;
    }
    if (!clt_get_static_map_->wait_for_service(std::chrono::seconds(0))) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "ObstacleDetector.-> Waiting for static_map service...");
      return;
    }

    auto req = std::make_shared<nav_msgs::srv::GetMap::Request>();
    clt_get_static_map_->async_send_request(req,
      [this](rclcpp::Client<nav_msgs::srv::GetMap>::SharedFuture fut) {
        try {
          auto map = fut.get()->map;
          obstacles_map_ = map;
          for (auto &c : obstacles_map_.data) c = 0;
          have_grid_ = true;
          RCLCPP_INFO(this->get_logger(), "ObstacleDetector.-> Got grid %u x %u (res %.3f)",
                      obstacles_map_.info.width, obstacles_map_.info.height, obstacles_map_.info.resolution);
        } catch (const std::exception &e) {
          RCLCPP_ERROR(this->get_logger(), "ObstacleDetector.-> Failed to get static map: %s", e.what());
        }
      }
    );
  }*/

  void try_get_static_map_once()
{
  if (have_grid_) return;

  if (!clt_get_static_map_->wait_for_service(std::chrono::seconds(0))) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "ObstacleDetector.-> Waiting for static_map service...");
    return;
  }

  auto req = std::make_shared<nav_msgs::srv::GetMap::Request>();
  clt_get_static_map_->async_send_request(req,
    [this](rclcpp::Client<nav_msgs::srv::GetMap>::SharedFuture fut) {
      try {
        auto map = fut.get()->map;

        if (map.info.width == 0 || map.info.height == 0 || map.data.empty()) {
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                               "ObstacleDetector.-> Got EMPTY map. Will retry...");
          return;
        }

        static_reference_map_ = map;
        obstacles_map_ = map;
        for (auto &c : obstacles_map_.data) c = 0;
        have_grid_ = true;

        RCLCPP_INFO(this->get_logger(),
                    "ObstacleDetector.-> Got base grid %u x %u (res %.3f). Starting publish.",
                    obstacles_map_.info.width, obstacles_map_.info.height, obstacles_map_.info.resolution);

        if (wait_map_timer_) wait_map_timer_->cancel();

      } catch (const std::exception &e) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                              "ObstacleDetector.-> Failed to get static map: %s (will retry)", e.what());
      }
    });
}


  // ===== sensor callbacks =====
  void cb_cloud1(const sensor_msgs::msg::PointCloud2::SharedPtr msg) { latest_cloud1_ = msg; cloud1_received_ = true; }
  void cb_cloud2(const sensor_msgs::msg::PointCloud2::SharedPtr msg) { latest_cloud2_ = msg; cloud2_received_ = true; }
  void cb_scan  (const sensor_msgs::msg::LaserScan::SharedPtr msg)   { latest_scan_   = msg; scan_received_   = true; }

  // ===== core =====
  inline void mark_cell(double mx, double my)
  {
    int cell_x = static_cast<int>((mx - obstacles_map_.info.origin.position.x) / obstacles_map_.info.resolution);
    int cell_y = static_cast<int>((my - obstacles_map_.info.origin.position.y) / obstacles_map_.info.resolution);
    int idx = cell_y * static_cast<int>(obstacles_map_.info.width) + cell_x;
    if (idx >= 0 && idx < static_cast<int>(obstacles_map_.data.size())) {
      if (!static_reference_map_.data.empty() &&
          idx < static_cast<int>(static_reference_map_.data.size()) &&
          static_reference_map_.data[idx] > 0) {
        return;
      }
      obstacles_map_.data[idx] = 100;
    }
  }

  // Usamos argumentos por valor para que el compilador los ponga en registros (CPU registers)
  inline void mark_cell_fast(double x, double y, 
                            double origin_x, double origin_y, 
                            double inv_res, int width, int height, int map_size)
  {
    // Usar el inverso de la resolución (multiplicación) es más rápido que dividir
    int cell_x = static_cast<int>((x - origin_x) * inv_res);
    int cell_y = static_cast<int>((y - origin_y) * inv_res);

    if (cell_x < 0 || cell_x >= width || cell_y < 0 || cell_y >= height) return;

    int idx = cell_y * width + cell_x;

    // Verificación de límites rápida
    if (idx >= 0 && idx < map_size) {
      if (!static_reference_map_.data.empty() &&
          idx < static_cast<int>(static_reference_map_.data.size()) &&
          static_reference_map_.data[idx] > 0) {
        return;
      }
      obstacles_map_.data[idx] = 100;
    }
  }

  struct CellEvidence
  {
    int count{0};
    int cell_x{0};
    int cell_y{0};
  };
/*
void add_obstacles_from_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr &cloud, int downsample)
{
  if (!cloud) return;
  if (cloud->data.empty() || cloud->width == 0 || cloud->height == 0) return;

  const unsigned char *p = cloud->data.data();
  Eigen::Affine3d cam_to_robot = get_relative_position(base_link_name_, cloud->header.frame_id);
  Eigen::Affine3d robot_to_map = get_relative_position("map", base_link_name_);

  const size_t total = static_cast<size_t>(cloud->width) * static_cast<size_t>(cloud->height);
  
  for (size_t i = 0; i < total; i += static_cast<size_t>(downsample))
  {
    Eigen::Vector3d v(
      *reinterpret_cast<const float*>(p + 0),
      *reinterpret_cast<const float*>(p + 4),
      *reinterpret_cast<const float*>(p + 8)
    );

    v = cam_to_robot * v;

    if (v.x() > cloud_min_x_ && v.x() < cloud_max_x_ &&
        v.y() > cloud_min_y_ && v.y() < cloud_max_y_ &&
        v.z() > cloud_min_z_ && v.z() < cloud_max_z_)
    {
      v = robot_to_map * v;
      mark_cell(v.x(), v.y());
    }

    p += static_cast<std::size_t>(downsample * cloud->point_step);
  }
}

  void add_obstacles_from_scan(const sensor_msgs::msg::LaserScan::SharedPtr &scan, int downsample)
  {
    if (!scan) return;

    Eigen::Affine3d lidar_to_robot = get_relative_position(base_link_name_, scan->header.frame_id);
    Eigen::Affine3d robot_to_map   = get_relative_position("map", base_link_name_);

    for (size_t i = 0; i < scan->ranges.size(); i += static_cast<size_t>(downsample))
    {
      float range = scan->ranges[i];
      float angle = scan->angle_min + static_cast<float>(i) * scan->angle_increment;

      Eigen::Vector3d v(range * std::cos(angle), range * std::sin(angle), 0.0);
      v = lidar_to_robot * v;

      if (v.x() > laser_min_x_ && v.x() < laser_max_x_ &&
          v.y() > laser_min_y_ && v.y() < laser_max_y_ &&
          v.z() > laser_min_z_ && v.z() < laser_max_z_)
      {
        v = robot_to_map * v;
        mark_cell(v.x(), v.y());
      }
    }
  }
*/
  void add_obstacles_from_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr &cloud, int downsample)
  {
    if (!cloud || cloud->data.empty()) return;

    // --- 1. TFs ÚNICAS POR NUBE ---
    Eigen::Affine3d cam_to_robot;
    Eigen::Affine3d robot_to_map;
    // De camara a base_link (para la cajita) y de base_link a mapa (grid).
    if (!lookup_transform_with_latest_fallback(
        base_link_name_, cloud->header.frame_id, cloud->header.stamp, cam_to_robot)) {
      return;
    }
    if (!lookup_transform_with_latest_fallback(
        "map", base_link_name_, cloud->header.stamp, robot_to_map)) {
      return;
    }

    // --- 2. PRECALCULO DE REJILLA ---
    double inv_res = 1.0 / obstacles_map_.info.resolution;
    double origin_x = obstacles_map_.info.origin.position.x;
    double origin_y = obstacles_map_.info.origin.position.y;
    int width = static_cast<int>(obstacles_map_.info.width);
    int height = static_cast<int>(obstacles_map_.info.height);
    int map_size = static_cast<int>(obstacles_map_.data.size());

    // --- 3. BUCLE DE PUNTOS ---
    // La nube del Kinect puede traer ruido temporal o puntos aislados en bordes.
    // En vez de marcar una celda con un solo punto, acumulamos evidencia por celda
    // y solo publicamos obstaculos donde exista densidad y continuidad espacial.
    std::unordered_map<int, CellEvidence> cell_hits;
    cell_hits.reserve(1024);

    const unsigned char *p = cloud->data.data();
    const size_t point_step = cloud->point_step;
    const size_t total = static_cast<size_t>(cloud->width) * static_cast<size_t>(cloud->height);
    const size_t skip = static_cast<size_t>(std::max(1, downsample)) * point_step;

    for (size_t i = 0; i < total; i += static_cast<size_t>(downsample))
    {
      // Punto en coordenadas de cámara
      Eigen::Vector3d v_cam(
        *reinterpret_cast<const float*>(p + 0),
        *reinterpret_cast<const float*>(p + 4),
        *reinterpret_cast<const float*>(p + 8)
      );
      if (!std::isfinite(v_cam.x()) || !std::isfinite(v_cam.y()) || !std::isfinite(v_cam.z())) {
        p += skip;
        continue;
      }

      Eigen::Vector3d v_robot = cam_to_robot * v_cam;

      if (v_robot.x() > cloud_min_x_ && v_robot.x() < cloud_max_x_ &&
          v_robot.y() > cloud_min_y_ && v_robot.y() < cloud_max_y_ &&
          v_robot.z() > cloud_min_z_ && v_robot.z() < cloud_max_z_)
      {
        Eigen::Vector3d v_map = robot_to_map * v_robot;

        int cell_x = static_cast<int>((v_map.x() - origin_x) * inv_res);
        int cell_y = static_cast<int>((v_map.y() - origin_y) * inv_res);
        if (cell_x < 0 || cell_x >= width || cell_y < 0 || cell_y >= height) {
          p += skip;
          continue;
        }

        int idx = cell_y * width + cell_x;
        auto &hit = cell_hits[idx];
        hit.count++;
        hit.cell_x = cell_x;
        hit.cell_y = cell_y;
      }
      p += skip;
    }

    // --- 4. VALIDACION DE CELDAS ---
    // Una silla/mesa real proyectada al mapa normalmente ocupa varias celdas cercanas.
    // Si una celda no tiene suficientes puntos o no tiene vecinos con evidencia,
    // se considera ruido y no se manda al mapa de obstaculos.
    const int min_points = std::max(1, cloud_min_points_per_cell_);
    const int min_neighbors = std::max(0, cloud_min_neighbor_cells_);

    for (const auto &entry : cell_hits)
    {
      const int idx = entry.first;
      const CellEvidence &hit = entry.second;

      if (hit.count < min_points) continue;

      if (min_neighbors > 0) {
        int neighbors = 0;
        for (int dy = -1; dy <= 1; ++dy) {
          for (int dx = -1; dx <= 1; ++dx) {
            if (dx == 0 && dy == 0) continue;

            const int nx = hit.cell_x + dx;
            const int ny = hit.cell_y + dy;
            if (nx < 0 || nx >= width || ny < 0 || ny >= height) continue;

            const int nidx = ny * width + nx;
            auto nentry = cell_hits.find(nidx);
            if (nentry != cell_hits.end() && nentry->second.count >= min_points) {
              neighbors++;
            }
          }
        }
        if (neighbors < min_neighbors) continue;
      }

      if (idx >= 0 && idx < map_size) {
        if (!static_reference_map_.data.empty() &&
            idx < static_cast<int>(static_reference_map_.data.size()) &&
            static_reference_map_.data[idx] > 0) {
          continue;
        }
        obstacles_map_.data[idx] = 100;
      }
    }
  }

  void add_obstacles_from_scan(const sensor_msgs::msg::LaserScan::SharedPtr &scan, int downsample)
  {
    if (!scan) return;

    // --- 1. TFs ÚNICAS (UNA VEZ POR ESCANEO) ---
    Eigen::Affine3d lidar_to_robot;
    Eigen::Affine3d robot_to_map;
    if (!lookup_transform_with_latest_fallback(
        base_link_name_, scan->header.frame_id, scan->header.stamp, lidar_to_robot)) {
      return;
    }
    if (!lookup_transform_with_latest_fallback(
        "map", base_link_name_, scan->header.stamp, robot_to_map)) {
      return;
    }

    // --- 2. PRECALCULO DE REJILLA ---
    double inv_res = 1.0 / obstacles_map_.info.resolution;
    double origin_x = obstacles_map_.info.origin.position.x;
    double origin_y = obstacles_map_.info.origin.position.y;
    int width = static_cast<int>(obstacles_map_.info.width);
    int height = static_cast<int>(obstacles_map_.info.height);
    int map_size = static_cast<int>(obstacles_map_.data.size());

    // --- 3. BUCLE DE LECTURAS ---
    const size_t skip = std::max(1, downsample);
    for (size_t i = 0; i < scan->ranges.size(); i += skip)
    {
      float range = scan->ranges[i];
      
      // Filtro básico de hardware/ruido
      if (range < scan->range_min || range > scan->range_max || std::isnan(range)) {
          continue;
      }

      float angle = scan->angle_min + static_cast<float>(i) * scan->angle_increment;

      // Punto en coordenadas locales del LIDAR (X es hacia adelante, Y lateral)
      Eigen::Vector3d v_lidar(range * std::cos(angle), range * std::sin(angle), 0.0);

      // PASO A: Transformar a coordenadas del ROBOT
      Eigen::Vector3d v_robot = lidar_to_robot * v_lidar;

      if (v_robot.x() > laser_min_x_ && v_robot.x() < laser_max_x_ &&
          v_robot.y() > laser_min_y_ && v_robot.y() < laser_max_y_ &&
          v_robot.z() > laser_min_z_ && v_robot.z() < laser_max_z_)
      {
        // PASO B: Si está en la caja, lo pasamos al MAPA
        Eigen::Vector3d v_map = robot_to_map * v_robot;
        
        mark_cell_fast(v_map.x(), v_map.y(), origin_x, origin_y, inv_res, width, height, map_size);
      }
    }
  }

  void decay_map(nav_msgs::msg::OccupancyGrid &map, int decay)
  {
    for (auto &c : map.data) {
      int v = static_cast<int>(c) - decay;
      c = static_cast<int8_t>(v < 0 ? 0 : v);
    }
  }

  void processing()
  {
    if (!have_grid_) return;

    try{

    // decay rápido (esto te quita “ghost obstacles” sin depender del merger)
    decay_map(obstacles_map_, decay_factor_);

    // integra sensores “lo más rápido posible”
    if (use_cloud_  && cloud1_received_) { add_obstacles_from_cloud(latest_cloud1_,  std::max(1, cloud_downsampling_));  cloud1_received_ = false; }
    if (use_cloud2_ && cloud2_received_) { add_obstacles_from_cloud(latest_cloud2_, std::max(1, cloud_downsampling2_)); cloud2_received_ = false; }
    if (use_lidar_  && scan_received_)   { add_obstacles_from_scan (latest_scan_,   std::max(1, lidar_downsampling_));  scan_received_ = false; }

    obstacles_map_.header.stamp = this->now();
    obstacles_map_.header.frame_id = "map";
    pub_obstacles_map_->publish(obstacles_map_);
    }
    catch(const std::exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "ObstacleDetector.-> Error in processing loop: %s", e.what());

    }
    catch(...)
    {
        RCLCPP_ERROR(this->get_logger(), "ObstacleDetector.-> Unknown error in processing loop.");
    }
  }

};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ObstacleDetectorNode>();
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
