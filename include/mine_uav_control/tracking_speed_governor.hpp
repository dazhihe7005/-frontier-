#pragma once

#include <algorithm>
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

enum class OverspeedRecoveryAction {
  kNormalFlight,
  kReduceSpeed,
  kExitOffboard,
};

// A measured hard-limit excursion is first handled by the continuous speed
// governor. Only an uninterrupted excursion lasting the complete recovery
// window may hand control back to PX4. Returning to the safe range at any
// point resumes normal flight and starts a future excursion from zero.
inline OverspeedRecoveryAction overspeedRecoveryAction(
    const double measured_speed, const double hard_limit,
    const double continuous_overspeed_seconds,
    const double recovery_timeout_seconds) {
  if (!std::isfinite(measured_speed) || !std::isfinite(hard_limit) ||
      !std::isfinite(continuous_overspeed_seconds) ||
      !std::isfinite(recovery_timeout_seconds) || hard_limit <= 0.0 ||
      continuous_overspeed_seconds < 0.0 ||
      recovery_timeout_seconds <= 0.0) {
    return OverspeedRecoveryAction::kExitOffboard;
  }
  if (measured_speed <= hard_limit) {
    return OverspeedRecoveryAction::kNormalFlight;
  }
  return continuous_overspeed_seconds < recovery_timeout_seconds
      ? OverspeedRecoveryAction::kReduceSpeed
      : OverspeedRecoveryAction::kExitOffboard;
}

// Turn measured overspeed into a continuous PX4 closed-loop request limit.
// The result never speeds up the planner: it is only a ceiling applied by
// governTrackingSpeed(). Keep the normal request headroom at and below the
// soft threshold. Above it, reduce the ceiling proportionally so ordinary
// estimator noise around cruise speed cannot create a brake/recover cycle.
inline double measuredSpeedRequestLimit(
    const double measured_speed, const double cruise_speed,
    const double feedback_gain, const double normal_request_limit,
    const double minimum_request_limit) {
  if (!std::isfinite(measured_speed) || !std::isfinite(cruise_speed) ||
      !std::isfinite(feedback_gain) ||
      !std::isfinite(normal_request_limit) ||
      !std::isfinite(minimum_request_limit) || cruise_speed <= 0.0 ||
      feedback_gain <= 0.0 || normal_request_limit < cruise_speed ||
      minimum_request_limit <= 0.0 ||
      minimum_request_limit >= cruise_speed) {
    return 0.0;
  }
  const double overspeed =
      std::max(0.0, std::max(0.0, measured_speed) - cruise_speed);
  const double request = normal_request_limit - feedback_gain * overspeed;
  return std::max(minimum_request_limit,
                  std::min(normal_request_limit, request));
}

// Apply asymmetric slew to the adaptive request limit.  Reducing the limit
// may be prompt, but it must still respect the command bridge's velocity
// continuity guard.  Recovery is deliberately slower to avoid repeatedly
// accelerating into the same threshold.
inline double slewMeasuredSpeedRequestLimit(
    const double current_limit, const double target_limit, const double dt,
    const double reduction_rate, const double recovery_rate) {
  if (!std::isfinite(current_limit) || !std::isfinite(target_limit) ||
      !std::isfinite(dt) || !std::isfinite(reduction_rate) ||
      !std::isfinite(recovery_rate) || current_limit <= 0.0 ||
      target_limit <= 0.0 || dt <= 0.0 || reduction_rate <= 0.0 ||
      recovery_rate <= 0.0) {
    return 0.0;
  }
  if (target_limit < current_limit) {
    return std::max(target_limit, current_limit - reduction_rate * dt);
  }
  return std::min(target_limit, current_limit + recovery_rate * dt);
}

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
