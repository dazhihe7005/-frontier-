#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace mine_uav_control {

struct ObstaclePoint {
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct LiveClearanceResult {
  bool violated{false};
  double horizontal_distance{std::numeric_limits<double>::infinity()};
  double vertical_distance{std::numeric_limits<double>::infinity()};
  ObstaclePoint obstacle;
};

inline LiveClearanceResult liveHorizontalCylinderClearance(
    const std::vector<ObstaclePoint>& points, const ObstaclePoint& vehicle,
    double horizontal_radius, double vertical_half_height,
    double min_obstacle_z = -std::numeric_limits<double>::infinity()) {
  LiveClearanceResult result;
  for (const auto& point : points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z)) {
      continue;
    }
    if (point.z < min_obstacle_z) continue;
    const double dz = std::abs(point.z - vehicle.z);
    if (dz > vertical_half_height + 1e-9) continue;
    const double horizontal = std::hypot(point.x - vehicle.x,
                                         point.y - vehicle.y);
    if (horizontal < result.horizontal_distance) {
      result.horizontal_distance = horizontal;
      result.vertical_distance = dz;
      result.obstacle = point;
    }
  }
  result.violated = result.horizontal_distance < horizontal_radius - 1e-9;
  return result;
}

struct SurfaceCoverageResult {
  bool obstacle_nearby{false};
  bool complete{false};
  std::size_t point_count{0};
  double nearest_horizontal{std::numeric_limits<double>::infinity()};
  double horizontal_span{0.0};
  double vertical_span{0.0};
};

inline SurfaceCoverageResult evaluateForwardObstacleCoverage(
    const std::vector<ObstaclePoint>& points, const ObstaclePoint& vehicle,
    double forward_x, double forward_y, double lookahead,
    double lateral_half_width, double vertical_half_height,
    std::size_t min_points, double min_horizontal_span,
    double min_vertical_span,
    double min_obstacle_z = -std::numeric_limits<double>::infinity()) {
  SurfaceCoverageResult result;
  const double norm = std::hypot(forward_x, forward_y);
  if (!(std::isfinite(norm) && norm > 1e-6 && lookahead > 0.0 &&
        lateral_half_width > 0.0 && vertical_half_height > 0.0)) {
    return result;
  }
  forward_x /= norm;
  forward_y /= norm;
  const double left_x = -forward_y;
  const double left_y = forward_x;
  double min_forward = std::numeric_limits<double>::infinity();
  double max_forward = -std::numeric_limits<double>::infinity();
  double min_lateral = std::numeric_limits<double>::infinity();
  double max_lateral = -std::numeric_limits<double>::infinity();
  double min_z = std::numeric_limits<double>::infinity();
  double max_z = -std::numeric_limits<double>::infinity();
  for (const auto& point : points) {
    if (point.z < min_obstacle_z) continue;
    const double dx = point.x - vehicle.x;
    const double dy = point.y - vehicle.y;
    const double dz = point.z - vehicle.z;
    const double forward = dx * forward_x + dy * forward_y;
    const double lateral = dx * left_x + dy * left_y;
    if (forward < 0.0 || forward > lookahead ||
        std::abs(lateral) > lateral_half_width ||
        std::abs(dz) > vertical_half_height) {
      continue;
    }
    ++result.point_count;
    result.nearest_horizontal = std::min(
        result.nearest_horizontal, std::hypot(dx, dy));
    min_forward = std::min(min_forward, forward);
    max_forward = std::max(max_forward, forward);
    min_lateral = std::min(min_lateral, lateral);
    max_lateral = std::max(max_lateral, lateral);
    min_z = std::min(min_z, point.z);
    max_z = std::max(max_z, point.z);
  }
  result.obstacle_nearby = result.point_count > 0;
  if (!result.obstacle_nearby) return result;
  result.horizontal_span = std::max(max_forward - min_forward,
                                    max_lateral - min_lateral);
  result.vertical_span = max_z - min_z;
  result.complete = result.point_count >= min_points &&
                    result.horizontal_span >= min_horizontal_span &&
                    result.vertical_span >= min_vertical_span;
  return result;
}

}  // namespace mine_uav_control
