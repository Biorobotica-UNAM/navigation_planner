#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

namespace simple_move
{
// Store a translation bearing in odom, so rotating the base does not rotate
// the reference. Wrapped differences also avoid a discontinuity at +/- pi.
struct DirectionReference
{
  double angle = std::numeric_limits<double>::quiet_NaN();
  double rate = 0.0;

  void reset(double bearing = std::numeric_limits<double>::quiet_NaN())
  {
    angle = bearing;
    rate = 0.0;
  }

  double update(double bearing, double dt, double response_time, double max_rate)
  {
    if (!std::isfinite(angle)) {reset(bearing);return angle;}
    const double error = std::remainder(bearing - angle, 2.0 * std::acos(-1.0));
    const double step = (1.0 - std::exp(-dt / response_time)) * error;
    rate = std::clamp(step / dt, -max_rate, max_rate);
    angle += rate * dt;
    return angle;
  }
};

// Use observed alignment and bounded reference motion, rather than extrapolating
// the derivative of a noisy target. An aligned camera must retain translation
// authority even when a target changes for one cycle. The caller owns the hard
// veto for stale feedback, mechanical limits and errors at the stop threshold.
inline double head_tracking_scale(
  double error, double tolerance, double stop_error,
  double required_pan_rate, double max_pan_speed)
{
  const double alignment = std::clamp(
    (stop_error - std::abs(error)) / (stop_error - tolerance), 0.0, 1.0);
  const double capacity = 0.8 * max_pan_speed;
  const double tracking = std::min(1.0, capacity / std::max(capacity, std::abs(required_pan_rate)));
  return alignment * tracking;
}
}  // namespace simple_move
