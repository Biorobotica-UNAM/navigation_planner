#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace path_planner
{
// Fixed storage for a short odometry window; no allocations in the control loop.
// Regression rejects position jitter without treating a round-trip motion as rest.
struct MotionObservation
{
  struct Sample {int64_t stamp; double x, y, yaw;};
  std::array<Sample, 64> samples{};
  size_t begin = 0, count = 0;
  double linear_speed = 0.0, angular_speed = 0.0, yaw_rate = 0.0;
  double span = 0.0, position_span = 0.0, angle_span = 0.0;
  bool stationary = false;

  void observe(int64_t stamp, double x, double y, double yaw)
  {
    if (stamp <= 0 || !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw)) {
      *this = MotionObservation{};return;
    }
    if (count) {
      const auto & last = samples[(begin + count - 1) % samples.size()];
      if (stamp == last.stamp) {return;}
      const double dt = (stamp - last.stamp) * 1e-9;
      if (dt > 0.0 && dt < 0.005) {return;}
      if (dt <= 0.0 || dt > 0.5) {
        *this = MotionObservation{};
      } else {
        const double change = std::remainder(yaw - last.yaw, 2.0 * std::acos(-1.0));
        yaw = last.yaw + change;
        yaw_rate += (1.0 - std::exp(-dt / 0.12)) * (change / dt - yaw_rate);
      }
    }
    if (count == samples.size()) {begin = (begin + 1) % samples.size();--count;}
    samples[(begin + count++) % samples.size()] = {stamp, x, y, yaw};
    // Keep one sample at/before the window boundary, including irregular updates.
    while (count > 2 && stamp - samples[(begin + 1) % samples.size()].stamp >= 300000000) {
      begin = (begin + 1) % samples.size();--count;
    }
    const auto & first = samples[begin];
    span = (stamp - first.stamp) * 1e-9;
    double st = 0.0, stt = 0.0, sx = 0.0, sy = 0.0, sa = 0.0;
    double stx = 0.0, sty = 0.0, sta = 0.0;
    double min_x = x, max_x = x, min_y = y, max_y = y, min_a = yaw, max_a = yaw;
    for (size_t i = 0; i < count; ++i) {
      const auto & s = samples[(begin + i) % samples.size()];
      const double t = (s.stamp - first.stamp) * 1e-9;
      const double dx = s.x - first.x, dy = s.y - first.y, da = s.yaw - first.yaw;
      st += t;stt += t * t;sx += dx;sy += dy;sa += da;
      stx += t * dx;sty += t * dy;sta += t * da;
      min_x = std::min(min_x, s.x);max_x = std::max(max_x, s.x);
      min_y = std::min(min_y, s.y);max_y = std::max(max_y, s.y);
      min_a = std::min(min_a, s.yaw);max_a = std::max(max_a, s.yaw);
    }
    const double denominator = count * stt - st * st;
    if (denominator > 1e-12) {
      linear_speed = std::hypot(count * stx - st * sx, count * sty - st * sy) / denominator;
      angular_speed = std::abs(count * sta - st * sa) / denominator;
    }
    position_span = std::hypot(max_x - min_x, max_y - min_y);
    angle_span = max_a - min_a;
    stationary = count >= 3 && span >= 0.3 && linear_speed < 0.02 &&
      angular_speed < 0.05 && position_span <= 0.01 && angle_span <= 0.025;
  }
};
}  // namespace path_planner
