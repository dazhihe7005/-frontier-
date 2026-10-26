#pragma once

#include <algorithm>
#include <cmath>

#include "mine_uav_control/lio_mount_alignment.hpp"

namespace mine_uav_control {

struct FiveMeterGoal {
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double yaw{0.0};
};

inline double normalizeAngle(double angle) {
  constexpr double kPi = 3.14159265358979323846;
  while (angle > kPi) angle -= 2.0 * kPi;
  while (angle < -kPi) angle += 2.0 * kPi;
  return angle;
}

inline FiveMeterGoal makeForwardGoal(double start_x, double start_y,
                                     double start_z, double yaw,
                                     double distance) {
  FiveMeterGoal goal;
  goal.x = start_x + distance * std::cos(yaw);
  goal.y = start_y + distance * std::sin(yaw);
  goal.z = start_z;
  goal.yaw = normalizeAngle(yaw);
  return goal;
}

// Build a goal in the raw Fast-LIO map whose displacement becomes horizontal
// and forward after the map-to-PX4 alignment is applied.  The planner must
// continue to receive camera_init coordinates; only this temporary test goal
// is pulled back through the inverse alignment.
inline FiveMeterGoal makeOutputHorizontalForwardGoal(
    double start_x, double start_y, double start_z, double raw_map_yaw,
    double distance, const Quaternion& map_to_output) {
  const Vector3 raw_heading(std::cos(raw_map_yaw),
                            std::sin(raw_map_yaw), 0.0);
  const Vector3 output_heading = rotateVector(map_to_output, raw_heading);
  const double horizontal_norm =
      std::hypot(output_heading.x, output_heading.y);

  // A valid 3-D rotation can only make this degenerate when the requested raw
  // heading is vertical in the output frame.  Keep a deterministic fallback;
  // the caller's normal 25-degree mount compensation is far from this case.
  const double output_x = horizontal_norm > 1e-9
      ? output_heading.x / horizontal_norm
      : 1.0;
  const double output_y = horizontal_norm > 1e-9
      ? output_heading.y / horizontal_norm
      : 0.0;
  const Vector3 raw_displacement = rotateVector(
      conjugate(map_to_output),
      Vector3(distance * output_x, distance * output_y, 0.0));

  FiveMeterGoal goal;
  goal.x = start_x + raw_displacement.x;
  goal.y = start_y + raw_displacement.y;
  goal.z = start_z + raw_displacement.z;
  goal.yaw = normalizeAngle(raw_map_yaw);
  return goal;
}

inline bool hoverStable(double speed, double vertical_speed,
                        double max_speed, double max_vertical_speed) {
  return std::isfinite(speed) && std::isfinite(vertical_speed) &&
         speed <= std::max(0.0, max_speed) &&
         std::abs(vertical_speed) <= std::max(0.0, max_vertical_speed);
}

inline bool goalSettled(const FiveMeterGoal& goal, double x, double y,
                        double z, double vx, double vy, double vz,
                        double position_tolerance,
                        double speed_tolerance) {
  const double dx = goal.x - x;
  const double dy = goal.y - y;
  const double dz = goal.z - z;
  const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
  const double speed = std::sqrt(vx * vx + vy * vy + vz * vz);
  return distance <= std::max(0.0, position_tolerance) &&
         speed <= std::max(0.0, speed_tolerance);
}

}  // namespace mine_uav_control
