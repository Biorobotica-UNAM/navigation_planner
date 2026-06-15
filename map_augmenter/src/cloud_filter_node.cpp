#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_eigen/tf2_eigen.hpp"

#include "Eigen/Geometry"

#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

class CloudFilterNode : public rclcpp::Node
{
public:
  CloudFilterNode()
  : Node("cloud_filter_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    declare_parameter<std::string>("input_topic", "/hardware/camera/points");
    declare_parameter<std::string>("output_topic", "/navigation/camera/points_filtered");
    declare_parameter<std::string>("base_link_name", "base_link");
    declare_parameter<std::string>("input_qos_mode", "dual");

    declare_parameter<double>("min_x", 0.30);
    declare_parameter<double>("max_x", 0.80);
    declare_parameter<double>("min_y", -0.30);
    declare_parameter<double>("max_y", 0.30);
    declare_parameter<double>("min_z", 0.01);
    declare_parameter<double>("max_z", 1.00);

    declare_parameter<double>("rear_margin", 0.15);
    declare_parameter<double>("front_margin", 0.40);
    declare_parameter<double>("side_margin", 0.15);
    declare_parameter<double>("lower_margin", 0.05);
    declare_parameter<double>("upper_margin", 0.20);
    declare_parameter<double>("floor_min_z", 0.05);

    declare_parameter<double>("voxel_leaf_size", 0.03);
    declare_parameter<int>("input_downsampling", 4);
    declare_parameter<int>("min_points_per_voxel", 2);
    declare_parameter<int>("min_neighbor_voxels", 0);
    declare_parameter<double>("processing_rate_hz", 20.0);
    declare_parameter<double>("tf_fallback_warn_threshold_sec", 0.20);
    declare_parameter<bool>("publish_debug_markers", true);
    declare_parameter<std::string>("debug_marker_topic", "cloud_filter/debug_markers");
    declare_parameter<double>("status_period_sec", 2.0);
    declare_parameter<bool>("enabled", true);
    declare_parameter<bool>("debug", false);

    load_parameters();

    param_callback_handle_ = add_on_set_parameters_callback(
      std::bind(&CloudFilterNode::on_parameter_change, this, std::placeholders::_1));

    // La nube filtrada ya es compacta y se usa para debug/navegacion local.
    // Publicarla como reliable evita incompatibilidades con RViz y comandos
    // como `ros2 topic hz`, que suelen suscribirse con QoS reliable.
    pub_cloud_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic_, rclcpp::QoS(5).reliable());
    pub_markers_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      debug_marker_topic_, rclcpp::QoS(1).transient_local());

    enable_srv_ = create_service<std_srvs::srv::SetBool>(
      "enable",
      std::bind(
        &CloudFilterNode::enable_callback,
        this,
        std::placeholders::_1,
        std::placeholders::_2));
    set_enabled(enabled_, false);

    const auto processing_period = std::chrono::duration<double>(1.0 / processing_rate_hz_);
    processing_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(processing_period),
      std::bind(&CloudFilterNode::process_latest_cloud, this));

    marker_timer_ = create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&CloudFilterNode::publish_debug_markers, this));

    const auto status_period = std::chrono::duration<double>(status_period_sec_);
    status_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(status_period),
      std::bind(&CloudFilterNode::publish_status_if_needed, this));

    RCLCPP_INFO(
      get_logger(),
      "CloudFilter.-> Filtering %s -> %s",
      input_topic_.c_str(),
      output_topic_.c_str());
  }

private:
  // Llave entera para agrupar puntos en una rejilla 3D. Cada llave representa
  // un "voxel", es decir, un cubo pequeño dentro del prisma de detección.
  struct VoxelKey
  {
    int x;
    int y;
    int z;

    bool operator==(const VoxelKey &other) const
    {
      return x == other.x && y == other.y && z == other.z;
    }
  };

  // Hash para poder usar VoxelKey como llave en unordered_map.
  struct VoxelKeyHash
  {
    std::size_t operator()(const VoxelKey &key) const
    {
      const auto hx = std::hash<int>{}(key.x * 73856093);
      const auto hy = std::hash<int>{}(key.y * 19349663);
      const auto hz = std::hash<int>{}(key.z * 83492791);
      return hx ^ (hy << 1) ^ (hz << 2);
    }
  };

  // Acumula todos los puntos que caen dentro del mismo voxel. Guardamos las
  // coordenadas originales de la nube de entrada para publicar la nube filtrada
  // en el mismo frame que llegó, sin cambiar su sistema de referencia.
  struct VoxelAccumulator
  {
    int count = 0;
    double input_x = 0.0;
    double input_y = 0.0;
    double input_z = 0.0;
  };

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_best_effort_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_reliable_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_srv_;
  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
  rclcpp::TimerBase::SharedPtr processing_timer_;
  rclcpp::TimerBase::SharedPtr marker_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  sensor_msgs::msg::PointCloud2::SharedPtr latest_cloud_;
  std::mutex latest_cloud_mutex_;
  uint64_t latest_cloud_sequence_{0};
  uint64_t processed_cloud_sequence_{0};
  std::atomic<uint64_t> received_clouds_{0};
  std::atomic<uint64_t> processed_clouds_{0};
  std::atomic<uint64_t> published_clouds_{0};
  std::atomic<uint64_t> empty_input_clouds_{0};
  std::atomic<uint64_t> invalid_field_clouds_{0};
  std::atomic<uint64_t> tf_failed_clouds_{0};
  std::atomic<std::size_t> last_total_points_{0};
  std::atomic<std::size_t> last_finite_points_{0};
  std::atomic<std::size_t> last_in_box_points_{0};
  std::atomic<std::size_t> last_voxels_{0};
  std::atomic<std::size_t> last_published_points_{0};
  uint64_t last_status_received_{0};
  uint64_t last_status_published_{0};

  std::string input_topic_;
  std::string output_topic_;
  std::string base_link_name_;
  std::string debug_marker_topic_;
  std::string input_qos_mode_;

  // Prisma de obstáculos recibido desde el launch. Este debe coincidir con el
  // usado por obstacle_detector/potential_fields.
  double obstacle_min_x_;
  double obstacle_max_x_;
  double obstacle_min_y_;
  double obstacle_max_y_;
  double obstacle_min_z_;
  double obstacle_max_z_;

  // Prisma real del filtro. Se calcula automáticamente agrandando el prisma de
  // obstáculos, para que el filtro limpie una zona amplia sin duplicar límites
  // en el launch.
  double filter_min_x_;
  double filter_max_x_;
  double filter_min_y_;
  double filter_max_y_;
  double filter_min_z_;
  double filter_max_z_;

  double rear_margin_;
  double front_margin_;
  double side_margin_;
  double lower_margin_;
  double upper_margin_;
  double floor_min_z_;

  double voxel_leaf_size_;
  int input_downsampling_;
  int min_points_per_voxel_;
  int min_neighbor_voxels_;
  double processing_rate_hz_;
  double tf_fallback_warn_threshold_sec_;
  double status_period_sec_;
  bool publish_debug_markers_;
  bool enabled_;
  bool debug_;

  void load_parameters()
  {
    get_parameter("input_topic", input_topic_);
    get_parameter("output_topic", output_topic_);
    get_parameter("base_link_name", base_link_name_);
    get_parameter("debug_marker_topic", debug_marker_topic_);
    get_parameter("input_qos_mode", input_qos_mode_);

    get_parameter("min_x", obstacle_min_x_);
    get_parameter("max_x", obstacle_max_x_);
    get_parameter("min_y", obstacle_min_y_);
    get_parameter("max_y", obstacle_max_y_);
    get_parameter("min_z", obstacle_min_z_);
    get_parameter("max_z", obstacle_max_z_);

    get_parameter("rear_margin", rear_margin_);
    get_parameter("front_margin", front_margin_);
    get_parameter("side_margin", side_margin_);
    get_parameter("lower_margin", lower_margin_);
    get_parameter("upper_margin", upper_margin_);
    get_parameter("floor_min_z", floor_min_z_);

    get_parameter("voxel_leaf_size", voxel_leaf_size_);
    get_parameter("input_downsampling", input_downsampling_);
    get_parameter("min_points_per_voxel", min_points_per_voxel_);
    get_parameter("min_neighbor_voxels", min_neighbor_voxels_);
    get_parameter("processing_rate_hz", processing_rate_hz_);
    get_parameter("tf_fallback_warn_threshold_sec", tf_fallback_warn_threshold_sec_);
    get_parameter("publish_debug_markers", publish_debug_markers_);
    get_parameter("status_period_sec", status_period_sec_);
    get_parameter("enabled", enabled_);
    get_parameter("debug", debug_);

    voxel_leaf_size_ = std::max(0.005, voxel_leaf_size_);
    input_downsampling_ = std::max(1, input_downsampling_);
    min_points_per_voxel_ = std::max(1, min_points_per_voxel_);
    min_neighbor_voxels_ = std::max(0, min_neighbor_voxels_);
    processing_rate_hz_ = std::max(1.0, processing_rate_hz_);
    tf_fallback_warn_threshold_sec_ = std::max(0.0, tf_fallback_warn_threshold_sec_);
    status_period_sec_ = std::max(0.5, status_period_sec_);
    update_filter_bounds();
  }

  void update_filter_bounds()
  {
    rear_margin_ = std::max(0.0, rear_margin_);
    front_margin_ = std::max(0.0, front_margin_);
    side_margin_ = std::max(0.0, side_margin_);
    lower_margin_ = std::max(0.0, lower_margin_);
    upper_margin_ = std::max(0.0, upper_margin_);
    floor_min_z_ = std::max(0.0, floor_min_z_);

    filter_min_x_ = std::max(0.0, obstacle_min_x_ - rear_margin_);
    filter_max_x_ = obstacle_max_x_ + front_margin_;
    filter_min_y_ = obstacle_min_y_ - side_margin_;
    filter_max_y_ = obstacle_max_y_ + side_margin_;

    // El filtro se abre hacia abajo respecto al detector, pero nunca por debajo
    // de floor_min_z para no dejar pasar ruido del piso.
    filter_min_z_ = std::max(floor_min_z_, obstacle_min_z_ - lower_margin_);
    filter_max_z_ = obstacle_max_z_ + upper_margin_;
  }

  // Permite afinar el filtro en vivo con `ros2 param set`, sin reiniciar el
  // launch. Solo los tópicos requieren reinicio porque ya fueron suscritos.
  rcl_interfaces::msg::SetParametersResult on_parameter_change(
    const std::vector<rclcpp::Parameter> &params)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto &param : params) {
      const auto &name = param.get_name();
      if (name == "base_link_name") base_link_name_ = param.as_string();
      else if (name == "min_x") obstacle_min_x_ = param.as_double();
      else if (name == "max_x") obstacle_max_x_ = param.as_double();
      else if (name == "min_y") obstacle_min_y_ = param.as_double();
      else if (name == "max_y") obstacle_max_y_ = param.as_double();
      else if (name == "min_z") obstacle_min_z_ = param.as_double();
      else if (name == "max_z") obstacle_max_z_ = param.as_double();
      else if (name == "rear_margin") rear_margin_ = param.as_double();
      else if (name == "front_margin") front_margin_ = param.as_double();
      else if (name == "side_margin") side_margin_ = param.as_double();
      else if (name == "lower_margin") lower_margin_ = param.as_double();
      else if (name == "upper_margin") upper_margin_ = param.as_double();
      else if (name == "floor_min_z") floor_min_z_ = param.as_double();
      else if (name == "voxel_leaf_size") voxel_leaf_size_ = std::max(0.005, param.as_double());
      else if (name == "input_downsampling") {
        input_downsampling_ = std::max(1, static_cast<int>(param.as_int()));
      }
      else if (name == "min_points_per_voxel") {
        min_points_per_voxel_ = std::max(1, static_cast<int>(param.as_int()));
      }
      else if (name == "min_neighbor_voxels") {
        min_neighbor_voxels_ = std::max(0, static_cast<int>(param.as_int()));
      }
      else if (name == "processing_rate_hz") {
        processing_rate_hz_ = std::max(1.0, param.as_double());
      }
      else if (name == "tf_fallback_warn_threshold_sec") {
        tf_fallback_warn_threshold_sec_ = std::max(0.0, param.as_double());
      }
      else if (name == "status_period_sec") {
        status_period_sec_ = std::max(0.5, param.as_double());
      }
      else if (name == "publish_debug_markers") publish_debug_markers_ = param.as_bool();
      else if (name == "enabled") set_enabled(param.as_bool(), true);
      else if (name == "debug") debug_ = param.as_bool();
      else if (name == "input_topic" || name == "output_topic" || name == "debug_marker_topic") {
        result.successful = false;
        result.reason = "Topic parameters require node restart";
        break;
      }
    }

    update_filter_bounds();
    return result;
  }

  void create_input_subscriptions()
  {
    const auto cb = std::bind(&CloudFilterNode::callback_cloud, this, std::placeholders::_1);
    if (input_qos_mode_ == "reliable") {
      sub_cloud_reliable_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topic_, rclcpp::QoS(5).reliable(), cb);
      sub_cloud_best_effort_.reset();
    } else if (input_qos_mode_ == "best_effort") {
      sub_cloud_best_effort_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topic_, rclcpp::SensorDataQoS(), cb);
      sub_cloud_reliable_.reset();
    } else {
      sub_cloud_best_effort_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topic_, rclcpp::SensorDataQoS(), cb);
      sub_cloud_reliable_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topic_, rclcpp::QoS(5).reliable(), cb);
    }
  }

  void clear_input_subscriptions()
  {
    sub_cloud_best_effort_.reset();
    sub_cloud_reliable_.reset();
    std::lock_guard<std::mutex> lock(latest_cloud_mutex_);
    latest_cloud_.reset();
    ++latest_cloud_sequence_;
  }

  void set_enabled(bool enabled, bool log_change)
  {
    if (enabled_ == enabled &&
        ((enabled && (sub_cloud_best_effort_ || sub_cloud_reliable_)) ||
         (!enabled && !sub_cloud_best_effort_ && !sub_cloud_reliable_))) {
      return;
    }

    enabled_ = enabled;
    if (enabled_) {
      create_input_subscriptions();
    } else {
      clear_input_subscriptions();
    }

    if (log_change) {
      RCLCPP_INFO(
        get_logger(),
        "CloudFilter.-> %s",
        enabled_ ? "enabled" : "paused");
    }
  }

  void enable_callback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    set_enabled(request->data, true);
    response->success = true;
    response->message = enabled_ ? "cloud_filter_enabled" : "cloud_filter_paused";
  }

  visualization_msgs::msg::Marker make_box_marker(
    int id,
    const std::string &name,
    double min_x,
    double max_x,
    double min_y,
    double max_y,
    double min_z,
    double max_z,
    float r,
    float g,
    float b,
    float a) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = base_link_name_;
    // Stamp cero + frame_locked hace que RViz use la transformacion mas
    // reciente. Para estos prismas de debug es mejor que usar now(), porque
    // evita parpadeos cuando TF llega con un pequeno desfase.
    marker.header.stamp.sec = 0;
    marker.header.stamp.nanosec = 0;
    marker.ns = "cloud_filter";
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.frame_locked = true;
    marker.text = name;
    marker.pose.position.x = (min_x + max_x) * 0.5;
    marker.pose.position.y = (min_y + max_y) * 0.5;
    marker.pose.position.z = (min_z + max_z) * 0.5;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = std::max(0.001, max_x - min_x);
    marker.scale.y = std::max(0.001, max_y - min_y);
    marker.scale.z = std::max(0.001, max_z - min_z);
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = a;
    // Lifetime cero significa persistente; se actualiza periodicamente por el
    // timer, pero no desaparece entre publicaciones.
    marker.lifetime = rclcpp::Duration::from_seconds(0.0);
    return marker;
  }

  void publish_debug_markers()
  {
    if (!enabled_ || !publish_debug_markers_) {
      return;
    }

    visualization_msgs::msg::MarkerArray markers;
    markers.markers.push_back(make_box_marker(
      0, "filter_box",
      filter_min_x_, filter_max_x_,
      filter_min_y_, filter_max_y_,
      filter_min_z_, filter_max_z_,
      0.0f, 0.7f, 1.0f, 0.16f));

    markers.markers.push_back(make_box_marker(
      1, "obstacle_box",
      obstacle_min_x_, obstacle_max_x_,
      obstacle_min_y_, obstacle_max_y_,
      obstacle_min_z_, obstacle_max_z_,
      1.0f, 0.25f, 0.05f, 0.28f));

    pub_markers_->publish(markers);
  }

  void publish_status_if_needed()
  {
    const auto received = received_clouds_.load();
    const auto published = published_clouds_.load();
    const auto input_publishers = count_publishers(input_topic_);
    const auto output_subscribers = count_subscribers(output_topic_);

    if (!enabled_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "CloudFilter.-> paused by enable service.");
      return;
    }

    if (received == 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "CloudFilter.-> No input clouds received on '%s' (publishers=%zu, qos_mode=%s, output_subscribers=%zu). Check topic name and QoS.",
        input_topic_.c_str(),
        input_publishers,
        input_qos_mode_.c_str(),
        output_subscribers);
      return;
    }

    const bool received_new = received != last_status_received_;
    const bool published_new = published != last_status_published_;

    if (received_new && !published_new) {
      RCLCPP_WARN(
        get_logger(),
        "CloudFilter.-> Receiving input but not publishing. rx=%lu processed=%lu pub=%lu empty=%lu bad_fields=%lu tf_fail=%lu last(raw=%zu finite=%zu in_box=%zu voxels=%zu accepted=%zu)",
        received,
        processed_clouds_.load(),
        published,
        empty_input_clouds_.load(),
        invalid_field_clouds_.load(),
        tf_failed_clouds_.load(),
        last_total_points_.load(),
        last_finite_points_.load(),
        last_in_box_points_.load(),
        last_voxels_.load(),
        last_published_points_.load());
    }

    last_status_received_ = received;
    last_status_published_ = published;
  }

  static bool get_float32_field_offset(
    const sensor_msgs::msg::PointCloud2 &cloud,
    const std::string &field_name,
    std::size_t &offset)
  {
    for (const auto &field : cloud.fields) {
      if (field.name == field_name &&
          field.datatype == sensor_msgs::msg::PointField::FLOAT32) {
        offset = field.offset;
        return true;
      }
    }
    return false;
  }

  // Cuenta cuántos voxeles vecinos existen alrededor de una celda. Esto ayuda
  // a quitar puntos aislados: un punto espurio suele crear un voxel solitario,
  // mientras que una pared/mesa/persona produce varios voxeles conectados.
  int count_neighbors(
    const std::unordered_map<VoxelKey, VoxelAccumulator, VoxelKeyHash> &voxels,
    const VoxelKey &key) const
  {
    int neighbors = 0;
    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dz = -1; dz <= 1; ++dz) {
          if (dx == 0 && dy == 0 && dz == 0) continue;
          VoxelKey neighbor{key.x + dx, key.y + dy, key.z + dz};
          if (voxels.find(neighbor) != voxels.end()) {
            ++neighbors;
          }
        }
      }
    }
    return neighbors;
  }

  bool lookup_cloud_transform(
    const std_msgs::msg::Header &header,
    Eigen::Affine3d &cloud_to_base)
  {
    try {
      auto tf = tf_buffer_.lookupTransform(
        base_link_name_,
        header.frame_id,
        header.stamp,
        rclcpp::Duration::from_seconds(0.03));
      cloud_to_base = tf2::transformToEigen(tf.transform);
      return true;
    } catch (const std::exception &exact_error) {
      try {
        // Si la nube llega unos milisegundos por delante del ultimo TF
        // publicado por robot_state_publisher, pedir el tiempo exacto causa
        // extrapolacion al futuro y se pierde el frame completo. Para el
        // prisma del Kinect es preferible usar la TF mas reciente disponible:
        // mantiene la nube viva y evita caidas grandes de frecuencia.
        auto tf = tf_buffer_.lookupTransform(
          base_link_name_,
          header.frame_id,
          rclcpp::Time(0),
          rclcpp::Duration::from_seconds(0.03));
        cloud_to_base = tf2::transformToEigen(tf.transform);
        const double tf_delta_sec =
          std::fabs((rclcpp::Time(header.stamp) - rclcpp::Time(tf.header.stamp)).seconds());
        if (tf_delta_sec > tf_fallback_warn_threshold_sec_) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 3000,
            "CloudFilter.-> Exact TF failed from '%s' to '%s' by %.3f s, using latest TF: %s",
            header.frame_id.c_str(),
            base_link_name_.c_str(),
            tf_delta_sec,
            exact_error.what());
        }
        return true;
      } catch (const std::exception &latest_error) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "CloudFilter.-> TF lookup failed from '%s' to '%s': %s",
          header.frame_id.c_str(),
          base_link_name_.c_str(),
          latest_error.what());
        return false;
      }
    }
  }

  // Publica una nube XYZ compacta con un punto promedio por voxel aceptado.
  // La nube conserva el header/frame original para que los nodos existentes
  // puedan seguir usando TF de la misma forma que con /hardware/camera/points.
  void publish_filtered_cloud(
    const sensor_msgs::msg::PointCloud2 &input,
    const std::vector<VoxelAccumulator> &accepted_voxels)
  {
    sensor_msgs::msg::PointCloud2 output;
    output.header = input.header;
    output.height = 1;
    output.width = static_cast<uint32_t>(accepted_voxels.size());
    output.is_bigendian = false;
    output.is_dense = true;

    sensor_msgs::PointCloud2Modifier modifier(output);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(accepted_voxels.size());

    sensor_msgs::PointCloud2Iterator<float> out_x(output, "x");
    sensor_msgs::PointCloud2Iterator<float> out_y(output, "y");
    sensor_msgs::PointCloud2Iterator<float> out_z(output, "z");

    for (const auto &voxel : accepted_voxels) {
      const auto inv_count = 1.0 / static_cast<double>(voxel.count);
      *out_x = static_cast<float>(voxel.input_x * inv_count);
      *out_y = static_cast<float>(voxel.input_y * inv_count);
      *out_z = static_cast<float>(voxel.input_z * inv_count);
      ++out_x;
      ++out_y;
      ++out_z;
    }

    pub_cloud_->publish(output);
    published_clouds_++;
    last_published_points_ = accepted_voxels.size();
  }

  // Flujo principal:
  // 1) Lee la nube cruda del Kinect.
  // 2) Transforma cada punto temporalmente a base_link para saber si está en
  //    el prisma frente al robot.
  // 3) Hace un downsampling simple de entrada para no procesar 307k puntos en
  //    cada frame del Kinect.
  // 4) Descarta puntos NaN/Inf y puntos fuera del prisma.
  // 5) Agrupa puntos restantes en voxeles.
  // 6) Descarta voxeles con pocos puntos o sin vecinos.
  // 7) Publica una nube filtrada para obstacle_detector y potential_fields.
  void callback_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (!msg) {
      return;
    }
    if (!enabled_) {
      return;
    }

    // La entrada del Kinect puede llegar a 30 Hz. Si procesamos cada nube en
    // el callback, cualquier pico de CPU crea una cola de mensajes viejos y
    // RViz/TF termina rechazandolos por timestamp atrasado. Guardamos solo la
    // nube mas reciente y dejamos que el timer procese a frecuencia controlada.
    std::lock_guard<std::mutex> lock(latest_cloud_mutex_);
    latest_cloud_ = msg;
    ++latest_cloud_sequence_;
    received_clouds_++;
  }

  void process_latest_cloud()
  {
    if (!enabled_) {
      return;
    }

    sensor_msgs::msg::PointCloud2::SharedPtr msg;
    uint64_t sequence = 0;
    {
      std::lock_guard<std::mutex> lock(latest_cloud_mutex_);
      if (!latest_cloud_ || latest_cloud_sequence_ == processed_cloud_sequence_) {
        return;
      }
      msg = latest_cloud_;
      sequence = latest_cloud_sequence_;
    }

    process_cloud(msg);
    processed_clouds_++;
    processed_cloud_sequence_ = sequence;
  }

  void process_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (!msg || msg->data.empty() || msg->point_step == 0) {
      empty_input_clouds_++;
      return;
    }

    std::size_t x_offset;
    std::size_t y_offset;
    std::size_t z_offset;
    if (!get_float32_field_offset(*msg, "x", x_offset) ||
        !get_float32_field_offset(*msg, "y", y_offset) ||
        !get_float32_field_offset(*msg, "z", z_offset)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "CloudFilter.-> Input cloud does not expose float32 x/y/z fields.");
      invalid_field_clouds_++;
      return;
    }

    // La geometría de la nube se evalúa en base_link. Así los límites min_x,
    // max_x, min_y, etc. significan "frente/lado/altura del robot" aunque la
    // nube venga originalmente en camera_link u otro frame de cámara.
    Eigen::Affine3d cloud_to_base;
    if (!lookup_cloud_transform(msg->header, cloud_to_base)) {
      tf_failed_clouds_++;
      return;
    }

    std::unordered_map<VoxelKey, VoxelAccumulator, VoxelKeyHash> voxels;
    const auto total_points = static_cast<std::size_t>(msg->width) * static_cast<std::size_t>(msg->height);
    last_total_points_ = total_points;
    voxels.reserve(total_points / 8 + 1);

    const auto *base_ptr = msg->data.data();
    const double inv_leaf = 1.0 / voxel_leaf_size_;
    std::size_t finite_points = 0;
    std::size_t pass_through_points = 0;

    const auto point_skip = static_cast<std::size_t>(std::max(1, input_downsampling_));
    for (std::size_t i = 0; i < total_points; i += point_skip) {
      const auto *point_ptr = base_ptr + i * msg->point_step;

      float x;
      float y;
      float z;
      std::memcpy(&x, point_ptr + x_offset, sizeof(float));
      std::memcpy(&y, point_ptr + y_offset, sizeof(float));
      std::memcpy(&z, point_ptr + z_offset, sizeof(float));

      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        continue;
      }
      ++finite_points;

      // Prisma geométrico amplio calculado desde el prisma de obstáculos. Todo
      // lo que quede fuera no se envía a navegación, reduciendo ruido, piso y
      // paredes laterales muy lejanas antes de la detección fina.
      Eigen::Vector3d point_base = cloud_to_base * Eigen::Vector3d(x, y, z);
      if (point_base.x() < filter_min_x_ || point_base.x() > filter_max_x_ ||
          point_base.y() < filter_min_y_ || point_base.y() > filter_max_y_ ||
          point_base.z() < filter_min_z_ || point_base.z() > filter_max_z_) {
        continue;
      }
      ++pass_through_points;

      // VoxelGrid simple: varios puntos cercanos se resumen en un solo voxel.
      // Esto baja la densidad de la nube y permite rechazar lecturas aisladas.
      VoxelKey key{
        static_cast<int>(std::floor(point_base.x() * inv_leaf)),
        static_cast<int>(std::floor(point_base.y() * inv_leaf)),
        static_cast<int>(std::floor(point_base.z() * inv_leaf))};

      auto &voxel = voxels[key];
      voxel.count += 1;
      voxel.input_x += x;
      voxel.input_y += y;
      voxel.input_z += z;
    }

    std::vector<VoxelAccumulator> accepted_voxels;
    accepted_voxels.reserve(voxels.size());
    for (const auto &entry : voxels) {
      const auto &key = entry.first;
      const auto &voxel = entry.second;
      // Primer filtro anti-ruido: exigir más de un punto por voxel.
      if (voxel.count < min_points_per_voxel_) {
        continue;
      }
      // Segundo filtro anti-ruido: opcionalmente exigir voxeles vecinos.
      if (min_neighbor_voxels_ > 0 && count_neighbors(voxels, key) < min_neighbor_voxels_) {
        continue;
      }
      accepted_voxels.push_back(voxel);
    }

    last_finite_points_ = finite_points;
    last_in_box_points_ = pass_through_points;
    last_voxels_ = voxels.size();

    publish_filtered_cloud(*msg, accepted_voxels);

    if (debug_) {
      RCLCPP_DEBUG_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "CloudFilter.-> raw=%zu skip=%d finite=%zu in_box=%zu voxels=%zu published=%zu box=[%.2f %.2f] [%.2f %.2f] [%.2f %.2f]",
        total_points,
        input_downsampling_,
        finite_points,
        pass_through_points,
        voxels.size(),
        accepted_voxels.size(),
        filter_min_x_, filter_max_x_,
        filter_min_y_, filter_max_y_,
        filter_min_z_, filter_max_z_);
    }
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CloudFilterNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
