#pragma once

#include <algorithm>
#include <cmath>

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
