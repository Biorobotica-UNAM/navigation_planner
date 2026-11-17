#include <navigation_manager/navigation.hpp>
#include <rclcpp/time.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <limits>
namespace navigation_manager
{
bool ExecutionMonitor::accept_map(Grid::ConstSharedPtr grid, Time received)
{
  const auto & i = grid->info;
  const auto & q = i.origin.orientation;
  const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  if (grid->header.frame_id != settings_.map_frame || !std::isfinite(i.resolution) ||
    i.resolution <= 0 || !i.width || !i.height ||
    uint64_t(i.width) * i.height != grid->data.size() ||
    !std::isfinite(i.origin.position.x) || !std::isfinite(i.origin.position.y) ||
    !std::isfinite(norm) || std::abs(norm - 1.) > 1e-5 || std::abs(q.x) > 1e-8 ||
    std::abs(q.y) > 1e-8)
  {
    return false;
  }
  map = std::move(grid);
  map_received = received;
  return true;
}
bool ExecutionMonitor::map_fresh(Time now) const
{
  return map && std::chrono::duration<double>(now - map_received).count() <= settings_.map_timeout;
}
bool ExecutionMonitor::sensors_fresh(Time now) const
{
  return sensors_ready &&
         std::chrono::duration<double>(now - sensors_received).count() <= settings_.sensor_timeout;
}
bool ExecutionMonitor::coherent_clearance() const
{
  return map && clearance && clearance->info == map->info &&
         clearance->data.size() == map->data.size() &&
         clearance->header.frame_id == settings_.map_frame &&
         rclcpp::Time(clearance->header.stamp) >= rclcpp::Time(map->header.stamp);
}
bool ExecutionMonitor::route_free(
  const Path & path, const Pose & robot, bool recovering,
  double horizon)
{
  const double start_x = robot.pose.position.x, start_y = robot.pose.position.y;
  if (!map || path.poses.empty()) {
    return false;
  }
  const auto & info = map->info;
  const bool coherent = clearance && clearance->info == info &&
    clearance->header.frame_id == settings_.map_frame &&
    clearance->data.size() == map->data.size() &&
    rclcpp::Time(clearance->header.stamp, RCL_ROS_TIME) >=
    rclcpp::Time(map->header.stamp, RCL_ROS_TIME);
  const auto & q = info.origin.orientation;
  const double yaw = std::atan2(2.0 * q.w * q.z, 1.0 - 2.0 * q.z * q.z);
  const double c = std::cos(yaw), s = std::sin(yaw);
  const double rx =
    (c * (start_x - info.origin.position.x) + s * (start_y - info.origin.position.y)) /
    info.resolution;
  const double ry =
    (-s * (start_x - info.origin.position.x) + c * (start_y - info.origin.position.y)) /
    info.resolution;
  bool escape = clearance && clearance->info == info &&
    clearance->header.frame_id == settings_.map_frame &&
    clearance->data.size() == map->data.size() &&
    rclcpp::Time(clearance->header.stamp, RCL_ROS_TIME) >=
    rclcpp::Time(map->header.stamp, RCL_ROS_TIME) && rx >= 0 && ry >= 0 &&
    rx < info.width && ry < info.height &&
    (recovering || map->data[size_t(ry) * info.width + size_t(rx)] > 0);
  constraints_.update(
    coherent && escape ? *clearance : *map, uncertain,
    settings_.robot_radius + path_planner::uncertainty_margin_m, settings_.allow_unknown);
  auto cell_free = [this, &info, &escape, c, s, start_x, start_y](int x, int y) {
      if (x < 0 || y < 0 || x >= static_cast<int>(info.width) ||
        y >= static_cast<int>(info.height))
      {
        return false;
      }
      const int value = map->data[static_cast<size_t>(y) * info.width + x];
      if (value > 0 && escape && clearance->data[size_t(y) * info.width + x] == 0) {
        const double gx = (x + 0.5) * info.resolution, gy = (y + 0.5) * info.resolution;
        if (std::hypot(
            info.origin.position.x + c * gx - s * gy - start_x,
            info.origin.position.y + s * gx + c * gy - start_y) <=
          settings_.escape_radius)
        {
          return true;
        }
      }
      if (value != 0 && !(settings_.allow_unknown && value == -1)) {
        return false;
      }
      return true;
    };
  double nearest = std::numeric_limits<double>::infinity();
  size_t begin = 0;
  for (size_t i = 0; i < path.poses.size(); ++i) {
    const auto & p = path.poses[i].pose.position;
    const double d = std::hypot(p.x - start_x, p.y - start_y);
    if (d < nearest) {
      nearest = d;
      begin = i;
    }
  }
  double px = start_x, py = start_y, traversed = 0.0;
  for (size_t i = begin; i < path.poses.size() && traversed <= horizon; ++i) {
    const auto & p = path.poses[i].pose.position;
    if (!std::isfinite(p.x) || !std::isfinite(p.y)) {
      return false;
    }
    const double length = std::hypot(p.x - px, p.y - py);
    const double fraction = length > 1e-9 ? std::min(1.0, (horizon - traversed) / length) : 1.0;
    const double tx = px + fraction * (p.x - px), ty = py + fraction * (p.y - py);
    if (!constraints_.uncertainty_free(px, py, tx, ty) ||
      !constraints_.sampling_free(px, py, tx, ty))
    {
      return false;
    }
    const double ax = (c * (px - info.origin.position.x) + s * (py - info.origin.position.y)) /
      info.resolution;
    const double ay = (-s * (px - info.origin.position.x) + c * (py - info.origin.position.y)) /
      info.resolution;
    const double bx = (c * (tx - info.origin.position.x) + s * (ty - info.origin.position.y)) /
      info.resolution;
    const double by = (-s * (tx - info.origin.position.x) + c * (ty - info.origin.position.y)) /
      info.resolution;
    if (!std::isfinite(ax) || !std::isfinite(ay) || !std::isfinite(bx) || !std::isfinite(by) ||
      ax < 0 || ay < 0 || bx < 0 || by < 0 || ax >= info.width || bx >= info.width ||
      ay >= info.height || by >= info.height)
    {
      return false;
    }
    int x = static_cast<int>(std::floor(ax)), y = static_cast<int>(std::floor(ay));
    const int ex = static_cast<int>(std::floor(bx)), ey = static_cast<int>(std::floor(by));
    const int sx = bx > ax ? 1 : (bx < ax ? -1 : 0), sy = by > ay ? 1 : (by < ay ? -1 : 0);
    const double dx = sx ? 1.0 / std::abs(bx - ax) : std::numeric_limits<double>::infinity();
    const double dy = sy ? 1.0 / std::abs(by - ay) : std::numeric_limits<double>::infinity();
    double nx = sx ? (x + (sx > 0 ? 1.0 : 0.0) - ax) / (bx - ax) : dx;
    double ny = sy ? (y + (sy > 0 ? 1.0 : 0.0) - ay) / (by - ay) : dy;
    if (!cell_free(x, y) || !cell_free(ex, ey)) {
      return false;
    }
    while (x != ex || y != ey) {
      if (std::min(nx, ny) >= 1.0 - 1e-12) {
        break;
      }
      if (std::abs(nx - ny) < 1e-12) {
        if (!cell_free(x + sx, y) || !cell_free(x, y + sy)) {
          return false;
        }
        x += sx;
        y += sy;
        nx += dx;
        ny += dy;
      } else if (nx < ny) {
        x += sx;
        nx += dx;
      } else {
        y += sy;
        ny += dy;
      }
      if (!cell_free(x, y)) {
        return false;
      }
    }
    traversed += length;
    px = p.x;
    py = p.y;
    if (map->data[size_t(ey) * info.width + ex] == 0) {
      escape = false;
    }
  }
  return true;
}

int ExecutionMonitor::occupancy(const Grid & grid, const geometry_msgs::msg::Point & point) const
{
  const double angle = 2.0 * std::atan2(
    grid.info.origin.orientation.z,
    grid.info.origin.orientation.w);
  const double dx = point.x - grid.info.origin.position.x,
    dy = point.y - grid.info.origin.position.y;
  const double x = (std::cos(angle) * dx + std::sin(angle) * dy) / grid.info.resolution;
  const double y = (-std::sin(angle) * dx + std::cos(angle) * dy) / grid.info.resolution;
  if (!std::isfinite(x) || !std::isfinite(y) || x < 0 || y < 0 ||
    x >= grid.info.width || y >= grid.info.height ||
    grid.data.size() != size_t(grid.info.width) * grid.info.height)
  {
    return -1;
  }
  return grid.data[size_t(y) * grid.info.width + size_t(x)];
}


bool ExecutionMonitor::safe_arrival(const NavigationContext & c, Time now)
{
  if (!c.pose_valid || !map_fresh(now) || !sensors_fresh(now) || !coherent_clearance() ||
    (occupancy(*clearance, c.robot.pose.position) != 0 &&
    !(settings_.allow_unknown && occupancy(*clearance, c.robot.pose.position) == -1)))
  {
    return false;
  }
  const auto & p = c.robot.pose.position;
  const bool planning_free = occupancy(*map, p) == 0 || (settings_.allow_unknown && occupancy(*map,p)==-1);
  const bool exact_position = std::hypot(p.x-c.original.pose.position.x,p.y-c.original.pose.position.y)<=settings_.position_tolerance;
  // Planning padding may prevent translation while still allowing safe in-place arrival.
  const auto & grid = !planning_free && exact_position ? *clearance : *map;
  constraints_.update(grid, uncertain, settings_.robot_radius + path_planner::uncertainty_margin_m,
    settings_.allow_unknown);
  return (planning_free || exact_position) && constraints_.uncertainty_free(p.x,p.y,p.x,p.y,false) &&
    constraints_.sampling_free(p.x,p.y,p.x,p.y,false);
}
std::string ExecutionMonitor::evaluate(const NavigationContext & c, Time now) const
{
  if (!c.pose_valid) {
    return "robot_tf_unavailable";
  }
  if (!map_fresh(now)) {
    return "map_unavailable";
  }
  if (!sensors_fresh(now)) {
    return "obstacle_data_stale";
  }
  if ((c.state == State::Moving) &&
    std::chrono::duration<double>(now - c.feedback_at).count() > settings_.controller_timeout)
  {
    return "controller_feedback_timeout";
  }
  if (c.state == State::Moving &&
    std::chrono::duration<double>(now - c.progress_at).count() > settings_.no_progress_timeout)
  {
    return "no_progress";
  }
  return {};
}
double ExecutionMonitor::recovery_radius(const Pose & goal)
{
  if (!map) {
    return 0.;
  }
  const auto & p = goal.pose.position;
  constraints_.update(
    *map, uncertain, settings_.robot_radius + path_planner::uncertainty_margin_m,
    settings_.allow_unknown);
  const int value = occupancy(*map, p);
  if (value > 0 || !constraints_.uncertainty_free(p.x, p.y, p.x, p.y, false)) {
    return settings_.approximate_radius;
  }
  if (value == 0 && !constraints_.sampling_free(p.x, p.y, p.x, p.y, false)) {
    return settings_.position_tolerance;
  }
  return 0.;
}
uint64_t ExecutionMonitor::geometry_signature(const NavigationContext & c) const
{
  if (!map) {
    return 0;
  }
  const auto & info = map->info;
  const double yaw = tf2::getYaw(info.origin.orientation), co = std::cos(yaw), si = std::sin(yaw);
  uint64_t hash = 1469598103934665603ULL;
  const int reach = static_cast<int>(std::ceil(settings_.path_horizon / info.resolution));
  // Stamps and unrelated map cells must not restart equivalent-failure counting.
  for (const auto & center:{c.robot.pose.position, c.original.pose.position}) {
    const double dx = center.x - info.origin.position.x, dy = center.y - info.origin.position.y;
    const int x = static_cast<int>(std::floor((co * dx + si * dy) / info.resolution));
    const int y = static_cast<int>(std::floor((-si * dx + co * dy) / info.resolution));
    for (int row = std::max(0, y - reach); row <= std::min(int(info.height) - 1, y + reach);
      ++row)
    {
      for (int col = std::max(0, x - reach); col <= std::min(int(info.width) - 1, x + reach);
        ++col)
      {
        hash ^= static_cast<uint8_t>(map->data[size_t(row) * info.width + col]);
        hash *= 1099511628211ULL;
      }
    }
  }
  for (const auto & pose:uncertain.poses) {
    const auto & p = pose.position;
    if (std::hypot(
        p.x - c.robot.pose.position.x,
        p.y - c.robot.pose.position.y) > settings_.path_horizon)
    {
      continue;
    }
    hash ^= uint64_t(int64_t(std::floor(p.x / info.resolution))) * 1099511628211ULL ^
      uint64_t(int64_t(std::floor(p.y / info.resolution))) * 1469598103934665603ULL;
  }
  return hash;
}
std::pair<double, double> ArrivalEvaluator::errors(const NavigationContext & c) const
{
  if (!c.resolved || !c.pose_valid) {
    return {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN()};
  }
  return {std::hypot(
      c.robot.pose.position.x - c.original.pose.position.x,
      c.robot.pose.position.y - c.original.pose.position.y),
    std::abs(
      std::remainder(
        tf2::getYaw(c.robot.pose.orientation) -
        tf2::getYaw(c.original.pose.orientation), 2. * M_PI))};
}
bool ArrivalEvaluator::reached(const NavigationContext & c, bool safe) const
{
  const auto error = errors(c);
  const double operational_error = std::hypot(
    c.robot.pose.position.x - c.operational.pose.position.x,
    c.robot.pose.position.y - c.operational.pose.position.y);
  return safe && c.odometry.stationary && !c.start_escape &&
         error.second <= settings_.angle_tolerance &&
         (error.first <= settings_.position_tolerance ||
         (c.approximate && error.first < std::min(c.arrival_radius, settings_.approximate_radius) &&
         operational_error <= settings_.position_tolerance));
}
bool ArrivalEvaluator::can_capture(NavigationContext & c, double radius) const
{
  const auto & robot=c.robot.pose.position;
  const double distance=std::hypot(robot.x-c.original.pose.position.x,robot.y-c.original.pose.position.y);
  if (distance>=radius || c.path.poses.empty()) {return false;}
  double total=0.,projected=0.,nearest=std::numeric_limits<double>::infinity();
  for (size_t i=1;i<c.path.poses.size();++i) {
    const auto & a=c.path.poses[i-1].pose.position;
    const auto & b=c.path.poses[i].pose.position;
    const double dx=b.x-a.x,dy=b.y-a.y,length=std::hypot(dx,dy);
    if (total+length>=c.route_projection-.2 && total<=c.route_projection+1.) {
      const double t=std::clamp(((robot.x-a.x)*dx+(robot.y-a.y)*dy)/std::max(1e-12,length*length),0.,1.);
      const double error=std::hypot(robot.x-a.x-t*dx,robot.y-a.y-t*dy);
      if (error<nearest) {nearest=error;projected=total+t*length;}
    }
    total+=length;
  }
  c.route_projection=std::max(c.route_projection,projected);
  const auto & endpoint=c.path.poses.back().pose.position;
  const double remaining=std::max(total-c.route_projection,std::hypot(robot.x-endpoint.x,robot.y-endpoint.y));
  return (total<=1e-9 || c.route_projection/total>=settings_.capture_progress) && remaining<radius;
}
} // namespace navigation_manager
