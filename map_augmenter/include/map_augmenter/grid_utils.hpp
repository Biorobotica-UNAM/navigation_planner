#pragma once

#include <nav_msgs/msg/occupancy_grid.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>

namespace map_augmenter
{
using Grid = nav_msgs::msg::OccupancyGrid;

// Mathematical tolerances, not navigation tuning parameters.
constexpr double geometry_epsilon = 1e-9;

inline double origin_yaw(const Grid & grid)
{
  const auto & q = grid.info.origin.orientation;
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

inline bool validate_grid(const Grid & grid, std::string * reason = nullptr)
{
  auto fail = [reason](const char * message) {
      if (reason) {*reason = message;}
      return false;
    };
  const auto & info = grid.info;
  if (grid.header.frame_id.empty()) {return fail("empty frame_id");}
  if (!std::isfinite(info.resolution) || info.resolution <= 0.0 ||
    info.width == 0 || info.height == 0 ||
    info.width > static_cast<uint32_t>(std::numeric_limits<int>::max() / 2) ||
    info.height > static_cast<uint32_t>(std::numeric_limits<int>::max() / 2))
  {
    return fail("invalid dimensions or resolution");
  }
  const uint64_t cells = static_cast<uint64_t>(info.width) * info.height;
  if (cells != grid.data.size()) {return fail("data length does not match dimensions");}
  const auto & p = info.origin.position;
  const auto & q = info.origin.orientation;
  if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
    !std::isfinite(q.x) || !std::isfinite(q.y) ||
    !std::isfinite(q.z) || !std::isfinite(q.w))
  {
    return fail("non-finite origin");
  }
  // A 2-D grid supports yaw. Reject tilted or unnormalised origins explicitly.
  if (std::abs(q.x) > geometry_epsilon || std::abs(q.y) > geometry_epsilon ||
    std::abs(q.z * q.z + q.w * q.w - 1.0) > 1e-5)
  {
    return fail("origin must contain a normalised planar quaternion");
  }
  if (std::any_of(grid.data.begin(), grid.data.end(), [](int8_t v) {
      return v < -1 || v > 100;
    }))
  {
    return fail("occupancy values must be in [-1, 100]");
  }
  return true;
}

inline bool same_geometry(const Grid & a, const Grid & b)
{
  return a.header.frame_id == b.header.frame_id &&
         a.info.width == b.info.width && a.info.height == b.info.height &&
         a.info.resolution == b.info.resolution &&
         a.info.origin.position.x == b.info.origin.position.x &&
         a.info.origin.position.y == b.info.origin.position.y &&
         a.info.origin.position.z == b.info.origin.position.z &&
         std::abs(std::remainder(origin_yaw(a) - origin_yaw(b), 2.0 * std::acos(-1.0))) <
         geometry_epsilon;
}

inline bool world_to_cell(const Grid & grid, double wx, double wy, int & x, int & y)
{
  if (!std::isfinite(wx) || !std::isfinite(wy) ||
    !std::isfinite(grid.info.resolution) || grid.info.resolution <= 0.0)
  {
    return false;
  }
  const double yaw = origin_yaw(grid), c = std::cos(yaw), s = std::sin(yaw);
  const double dx = wx - grid.info.origin.position.x;
  const double dy = wy - grid.info.origin.position.y;
  const double gx = (c * dx + s * dy) / grid.info.resolution;
  const double gy = (-s * dx + c * dy) / grid.info.resolution;
  if (gx < 0.0 || gy < 0.0 || gx >= grid.info.width || gy >= grid.info.height) {
    return false;
  }
  x = static_cast<int>(std::floor(gx));
  y = static_cast<int>(std::floor(gy));
  return true;
}

inline size_t cell_index(const Grid & grid, int x, int y)
{
  return static_cast<size_t>(y) * grid.info.width + static_cast<size_t>(x);
}

inline Grid empty_obstacle_grid(const Grid & geometry)
{
  Grid result = geometry;
  std::fill(result.data.begin(), result.data.end(), 0);
  return result;
}

// Both grids must be valid and use the same TF frame. Different frames require
// an explicit transform; silently treating them as identical is unsafe.
// Visit every target cell with positive-area overlap, rather than projecting
// only the source centre (which loses coverage when resolutions differ).
template<typename Visitor>
void for_each_projected_obstacle(const Grid & source, const Grid & target, Visitor visit)
{
  if (source.header.frame_id != target.header.frame_id) {
    throw std::invalid_argument("cannot project grids in different TF frames");
  }
  if (same_geometry(source, target)) {
    for (size_t i = 0; i < source.data.size(); ++i) {
      if (source.data[i] > 0) {visit(i, i);}
    }
    return;
  }
  const double syaw = origin_yaw(source), tyaw = origin_yaw(target);
  const double tc = std::cos(tyaw), ts = std::sin(tyaw);
  const double c = std::cos(syaw - tyaw), s = std::sin(syaw - tyaw);
  const double scale = source.info.resolution / target.info.resolution;
  const double dx = source.info.origin.position.x - target.info.origin.position.x;
  const double dy = source.info.origin.position.y - target.info.origin.position.y;
  const double ox = (tc * dx + ts * dy) / target.info.resolution;
  const double oy = (-ts * dx + tc * dy) / target.info.resolution;
  const double half = scale / 2.0;
  const double extent = half * (std::abs(c) + std::abs(s));
  const double source_axis_limit = half + 0.5 * (std::abs(c) + std::abs(s));
  const double overlap_epsilon = geometry_epsilon * std::min(1.0, scale);
  for (size_t index = 0; index < source.data.size(); ++index) {
    if (source.data[index] <= 0) {continue;}
    const double sx = (index % source.info.width + 0.5) * scale;
    const double sy = (index / source.info.width + 0.5) * scale;
    const double cx = ox + c * sx - s * sy;
    const double cy = oy + s * sx + c * sy;
    if (cx + extent <= 0.0 || cy + extent <= 0.0 ||
      cx - extent >= target.info.width || cy - extent >= target.info.height)
    {
      continue;
    }
    const int xmin = static_cast<int>(std::floor(std::max(0.0, cx - extent)));
    const int ymin = static_cast<int>(std::floor(std::max(0.0, cy - extent)));
    const int xmax = static_cast<int>(std::ceil(std::min<double>(target.info.width, cx + extent))) - 1;
    const int ymax = static_cast<int>(std::ceil(std::min<double>(target.info.height, cy + extent))) - 1;
    for (int y = ymin; y <= ymax; ++y) {
      for (int x = xmin; x <= xmax; ++x) {
        const double vx = x + 0.5 - cx, vy = y + 0.5 - cy;
        // Separating-axis checks for the two cell squares.
        if (std::abs(vx) >= extent + 0.5 - overlap_epsilon ||
          std::abs(vy) >= extent + 0.5 - overlap_epsilon ||
          std::abs(c * vx + s * vy) >= source_axis_limit - overlap_epsilon ||
          std::abs(-s * vx + c * vy) >= source_axis_limit - overlap_epsilon)
        {
          continue;
        }
        visit(index, cell_index(target, x, y));
      }
    }
  }
}

inline Grid reproject_obstacles(const Grid & source, const Grid & target)
{
  Grid result = empty_obstacle_grid(target);
  for_each_projected_obstacle(source, target, [&](size_t from, size_t to) {
    result.data[to] = std::max(result.data[to], source.data[from]);
  });
  return result;
}

inline Grid overlay_obstacles(const Grid & base, const Grid & overlay)
{
  Grid result = base;
  for_each_projected_obstacle(overlay, base, [&](size_t from, size_t to) {
    // Signed comparison: an obstacle must override unknown (-1).
    result.data[to] = std::max(result.data[to], overlay.data[from]);
  });
  return result;
}

inline Grid inflate_obstacles(const Grid & grid, double radius, bool circular)
{
  Grid result = grid;
  if (radius <= 0.0) {return result;}
  const double cells = std::ceil(radius / grid.info.resolution);
  const int width = static_cast<int>(grid.info.width);
  const int height = static_cast<int>(grid.info.height);
  const int reach = static_cast<int>(std::min(cells, static_cast<double>(std::max(width, height))));
  for (size_t i = 0; i < grid.data.size(); ++i) {
    if (grid.data[i] <= 0) {continue;}
    const int x = static_cast<int>(i % grid.info.width);
    const int y = static_cast<int>(i / grid.info.width);
    for (int ny = std::max(0, y - reach); ny <= std::min(height - 1, y + reach); ++ny) {
      for (int nx = std::max(0, x - reach); nx <= std::min(width - 1, x + reach); ++nx) {
        if (circular && std::hypot(nx - x, ny - y) > cells) {continue;}
        auto & value = result.data[cell_index(grid, nx, ny)];
        value = std::max(value, grid.data[i]);
      }
    }
  }
  return result;
}

inline Grid make_cost_map(const Grid & collision, double radius, int peak, bool circular)
{
  Grid result = collision;
  if (radius <= 0.0) {return result;}
  const double cells = std::ceil(radius / collision.info.resolution);
  const int width = static_cast<int>(collision.info.width);
  const int height = static_cast<int>(collision.info.height);
  const int reach = static_cast<int>(std::min(cells, static_cast<double>(std::max(width, height))));
  for (size_t i = 0; i < collision.data.size(); ++i) {
    if (collision.data[i] <= 0) {continue;}
    const int x = static_cast<int>(i % collision.info.width);
    const int y = static_cast<int>(i / collision.info.width);
    for (int ny = std::max(0, y - reach); ny <= std::min(height - 1, y + reach); ++ny) {
      for (int nx = std::max(0, x - reach); nx <= std::min(width - 1, x + reach); ++nx) {
        const size_t idx = cell_index(collision, nx, ny);
        // A soft cost must not change unknown-space semantics.
        if (collision.data[idx] != 0) {continue;}
        const double distance = circular ? std::hypot(nx - x, ny - y) :
          static_cast<double>(std::max(std::abs(nx - x), std::abs(ny - y)));
        if (distance > cells) {continue;}
        const int cost = static_cast<int>(std::ceil(peak * (cells - distance) / cells));
        result.data[idx] = std::max(result.data[idx], static_cast<int8_t>(cost));
      }
    }
  }
  return result;
}
}  // namespace map_augmenter
