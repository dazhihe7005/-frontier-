#pragma once

#include <array>
#include <cmath>

namespace mine_uav_control {

struct TrackingSpeedGovernorResult {
  std::array<double, 3> feedforward_velocity{{0.0, 0.0, 0.0}};
  std::array<double, 3> closed_loop_velocity{{0.0, 0.0, 0.0}};
  double scale{1.0};
  bool active{false};
  bool valid{false};
};

// PX4's multicopter position controller forms its velocity setpoint as
//
//   velocity_feedforward + position_gain * position_error.
//
// Preserve the position reference, but replace the velocity feed-forward so
// that PX4 reconstructs a vector inside the requested 3-D speed ball. This is
// deliberately a vector projection rather than three independent axis clamps.
inline TrackingSpeedGovernorResult governTrackingSpeed(
    const std::array<double, 3>& planned_velocity,
    const std::array<double, 3>& position_error,
    const std::array<double, 3>& position_gain,
    const double max_closed_loop_speed) {
  TrackingSpeedGovernorResult result;
  if (!std::isfinite(max_closed_loop_speed) || max_closed_loop_speed <= 0.0) {
    return result;
  }

  double speed_squared = 0.0;
  std::array<double, 3> position_correction{{0.0, 0.0, 0.0}};
  for (std::size_t i = 0; i < 3; ++i) {
    if (!std::isfinite(planned_velocity[i]) ||
        !std::isfinite(position_error[i]) ||
        !std::isfinite(position_gain[i]) || position_gain[i] < 0.0) {
      return result;
    }
    position_correction[i] = position_gain[i] * position_error[i];
    result.closed_loop_velocity[i] =
        planned_velocity[i] + position_correction[i];
    speed_squared += result.closed_loop_velocity[i] *
                     result.closed_loop_velocity[i];
  }

  const double speed = std::sqrt(speed_squared);
  result.scale = speed > max_closed_loop_speed
      ? max_closed_loop_speed / speed
      : 1.0;
  result.active = result.scale < 1.0;
  for (std::size_t i = 0; i < 3; ++i) {
    result.closed_loop_velocity[i] *= result.scale;
    result.feedforward_velocity[i] =
        result.closed_loop_velocity[i] - position_correction[i];
  }
  result.valid = true;
  return result;
}

}  // namespace mine_uav_control
