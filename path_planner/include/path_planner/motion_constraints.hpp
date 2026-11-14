#pragma once

#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

namespace path_planner
{
inline constexpr double uncertainty_margin_m = 0.05;
// Spatial buckets are rebuilt only when geometry, points or the guard change.
// Queries allocate nothing and inspect only buckets intersecting the segment.
class MotionConstraints
{
public:
  void update(const nav_msgs::msg::OccupancyGrid& grid,
    const geometry_msgs::msg::PoseArray& obstacles, double radius, bool allow_unknown = false)
  {
    grid_ = &grid;
    allow_unknown_ = allow_unknown;
    bool changed = grid.info != info_ || radius != radius_ || obstacles.poses.size() != points_.size();
    if (!changed) {
      for (size_t i = 0; i < points_.size(); ++i) {
        if (points_[i][0] != obstacles.poses[i].position.x ||
          points_[i][1] != obstacles.poses[i].position.y) {changed = true;break;}
      }
    }
    if (!changed) {return;}
    info_ = grid.info;radius_ = radius;
    const auto& q = info_.origin.orientation;
    const double yaw = std::atan2(2.0 * q.w * q.z, 1.0 - 2.0 * q.z * q.z);
    c_ = std::cos(yaw);s_ = std::sin(yaw);
    bucket_size_ = std::max(radius, static_cast<double>(info_.resolution));
    width_ = std::max(1, static_cast<int>(std::ceil(info_.width * info_.resolution / bucket_size_)));
    height_ = std::max(1, static_cast<int>(std::ceil(info_.height * info_.resolution / bucket_size_)));
    if (obstacles.poses.empty()) {points_.clear();heads_.clear();next_.clear();return;}
    heads_.assign(static_cast<size_t>(width_) * height_, -1);
    points_.resize(obstacles.poses.size());next_.resize(points_.size());
    for (size_t i = 0; i < points_.size(); ++i) {
      const auto& p = obstacles.poses[i].position;
      points_[i] = {p.x, p.y};
      const double dx = p.x - info_.origin.position.x, dy = p.y - info_.origin.position.y;
      const double x = c_ * dx + s_ * dy, y = -s_ * dx + c_ * dy;
      next_[i] = -1;
      if (!std::isfinite(x) || !std::isfinite(y) || x < -radius || y < -radius ||
        x > info_.width * info_.resolution + radius ||
        y > info_.height * info_.resolution + radius) {continue;}
      const int bx = std::clamp(static_cast<int>(std::floor(x / bucket_size_)), 0, width_ - 1);
      const int by = std::clamp(static_cast<int>(std::floor(y / bucket_size_)), 0, height_ - 1);
      const size_t bucket = static_cast<size_t>(by) * width_ + bx;
      next_[i] = heads_[bucket];heads_[bucket] = static_cast<int>(i);
    }
  }

  bool uncertainty_free(double ax, double ay, double bx, double by, bool allow_escape = true,
    double* separation = nullptr, double* required = nullptr) const
  {
    if (points_.empty()) {return true;}
    const double adx = ax - info_.origin.position.x, ady = ay - info_.origin.position.y;
    const double bdx = bx - info_.origin.position.x, bdy = by - info_.origin.position.y;
    const double x0 = c_ * adx + s_ * ady, y0 = -s_ * adx + c_ * ady;
    const double x1 = c_ * bdx + s_ * bdy, y1 = -s_ * bdx + c_ * bdy;
    if (!std::isfinite(x0 + y0 + x1 + y1)) {return false;}
    const int xmin = static_cast<int>(std::clamp(std::floor((std::min(x0, x1) - radius_) / bucket_size_), 0.0, double(width_ - 1)));
    const int xmax = static_cast<int>(std::clamp(std::floor((std::max(x0, x1) + radius_) / bucket_size_), 0.0, double(width_ - 1)));
    const int ymin = static_cast<int>(std::clamp(std::floor((std::min(y0, y1) - radius_) / bucket_size_), 0.0, double(height_ - 1)));
    const int ymax = static_cast<int>(std::clamp(std::floor((std::max(y0, y1) + radius_) / bucket_size_), 0.0, double(height_ - 1)));
    const double dx = bx - ax, dy = by - ay, length2 = dx * dx + dy * dy;
    for (int y = ymin; y <= ymax; ++y) {
      for (int x = xmin; x <= xmax; ++x) {
        for (int i = heads_[static_cast<size_t>(y) * width_ + x]; i >= 0; i = next_[i]) {
          const double px = ax - points_[i][0], py = ay - points_[i][1];
          const double floor = allow_escape ? std::min(px * px + py * py, radius_ * radius_) : radius_ * radius_;
          const double t = length2 > 1e-12 ? std::clamp(-(px * dx + py * dy) / length2, 0.0, 1.0) : 0.0;
          const double ex = px + t * dx, ey = py + t * dy, square = ex * ex + ey * ey;
          if (square + 1e-8 < floor) {
            if (separation) {*separation = std::sqrt(square);}
            if (required) {*required = std::sqrt(floor);}
            return false;
          }
        }
      }
    }
    return true;
  }

  // The grid already includes the physical footprint. The half-cell guard
  // covers interpolation; an initially smaller clearance may only increase.
  bool sampling_free(double ax, double ay, double bx, double by, bool allow_escape = true,
    double* separation = nullptr, double* required = nullptr) const
  {
    if (!grid_) {return false;}
    const double adx = ax - info_.origin.position.x, ady = ay - info_.origin.position.y;
    const double bdx = bx - info_.origin.position.x, bdy = by - info_.origin.position.y;
    const double x0 = (c_ * adx + s_ * ady) / info_.resolution;
    const double y0 = (-s_ * adx + c_ * ady) / info_.resolution;
    const double x1 = (c_ * bdx + s_ * bdy) / info_.resolution;
    const double y1 = (-s_ * bdx + c_ * bdy) / info_.resolution;
    if (!std::isfinite(x0 + y0 + x1 + y1) || std::min(x0, x1) < 0.0 ||
      std::min(y0, y1) < 0.0 || std::max(x0, x1) >= info_.width || std::max(y0, y1) >= info_.height) {return false;}
    const int count = static_cast<int>(std::ceil(4.0 * std::hypot(x1 - x0, y1 - y0)));
    double floor = 0.25;
    for (int step = 0; step <= count; ++step) {
      const double t = count ? static_cast<double>(step) / count : 0.0;
      const double x = x0 + t * (x1 - x0), y = y0 + t * (y1 - y0);
      const int ix = static_cast<int>(x), iy = static_cast<int>(y);
      double distance2 = 1.0;
      for (int cy = std::max(0, iy - 1); cy <= std::min(static_cast<int>(info_.height) - 1, iy + 1); ++cy) {
        for (int cx = std::max(0, ix - 1); cx <= std::min(static_cast<int>(info_.width) - 1, ix + 1); ++cx) {
          const int value = grid_->data[static_cast<size_t>(cy) * info_.width + cx];
          if (value == 0 || (allow_unknown_ && value == -1)) {continue;}
          if (cx == ix && cy == iy) {return false;}
          const double dx = std::max({double(cx) - x, 0.0, x - double(cx + 1)});
          const double dy = std::max({double(cy) - y, 0.0, y - double(cy + 1)});
          distance2 = std::min(distance2, dx * dx + dy * dy);
        }
      }
      if (step == 0 && allow_escape) {floor = std::min(floor, distance2);}
      if (separation) {*separation = std::sqrt(distance2) * info_.resolution;}
      if (required) {*required = std::sqrt(floor) * info_.resolution;}
      if (distance2 + 1e-8 < floor) {return false;}
      floor = std::min(0.25, std::max(floor, distance2));
    }
    return true;
  }

private:
  const nav_msgs::msg::OccupancyGrid* grid_ = nullptr;
  nav_msgs::msg::MapMetaData info_;
  std::vector<std::array<double, 2>> points_;
  std::vector<int> heads_, next_;
  double radius_ = -1.0, bucket_size_ = 1.0, c_ = 1.0, s_ = 0.0;
  int width_ = 1, height_ = 1;
  bool allow_unknown_ = false;
};
}  // namespace path_planner
