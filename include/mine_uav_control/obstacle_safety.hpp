#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
#include <tuple>
#include <vector>

namespace mine_uav_control {

struct ObstaclePoint {
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

enum class ObstacleMonitorAction {
  kClear,
  kHold,
  kPersistent,
};

// Persistence decides when a return is no longer transient.  It does not
// redefine the hard radius: only a persistent return that is also inside the
// independent configured hard cylinder is allowed to request an OFFBOARD exit.
inline bool persistentHardIntrusionRequiresOffboardExit(
    ObstacleMonitorAction action, bool hard_clearance_intrusion) {
  return action == ObstacleMonitorAction::kPersistent &&
      hard_clearance_intrusion;
}

// A newly observed obstacle first causes a fixed-position hold. Persistence
// alone does not imply a terminal fault; the caller applies the separate hard
// clearance policy above.
// Several consecutive clear scans are required before resuming so a sparse
// lidar packet cannot make the vehicle alternate between hold and flight.
class TransientObstacleMonitor {
 public:
  void configure(double persistence_seconds, int clear_frames) {
    persistence_seconds_ = std::max(0.1, persistence_seconds);
    clear_frames_ = std::max(1, clear_frames);
    reset();
  }

  void reset() {
    active_ = false;
    persistent_ = false;
    first_seen_seconds_ = 0.0;
    last_update_seconds_ = 0.0;
    clear_streak_ = 0;
  }

  ObstacleMonitorAction update(bool obstacle_present, double now_seconds) {
    if (!std::isfinite(now_seconds)) {
      reset();
      return ObstacleMonitorAction::kClear;
    }
    if (active_ && now_seconds + 1e-9 < last_update_seconds_) {
      reset();
    }
    last_update_seconds_ = now_seconds;

    if (obstacle_present) {
      clear_streak_ = 0;
      if (!active_) {
        active_ = true;
        first_seen_seconds_ = now_seconds;
      }
      if (now_seconds - first_seen_seconds_ + 1e-9 >=
          persistence_seconds_) {
        persistent_ = true;
        return ObstacleMonitorAction::kPersistent;
      }
      return ObstacleMonitorAction::kHold;
    }

    if (!active_) return ObstacleMonitorAction::kClear;
    ++clear_streak_;
    if (clear_streak_ >= clear_frames_) {
      reset();
      return ObstacleMonitorAction::kClear;
    }
    return persistent_ ? ObstacleMonitorAction::kPersistent
                       : ObstacleMonitorAction::kHold;
  }

  bool active() const { return active_; }
  double elapsed(double now_seconds) const {
    if (!active_ || !std::isfinite(now_seconds)) return 0.0;
    return std::max(0.0, now_seconds - first_seen_seconds_);
  }

 private:
  double persistence_seconds_{3.0};
  double first_seen_seconds_{0.0};
  double last_update_seconds_{0.0};
  int clear_frames_{3};
  int clear_streak_{0};
  bool active_{false};
  bool persistent_{false};
};

// Keeps a static obstacle memory without promoting an isolated lidar return
// into a permanent hazard. A voxel must be seen in multiple distinct cloud
// callbacks inside a bounded frame window before it becomes actionable.
// Confirmed voxels remain persistent so temporary occlusion cannot erase a
// real obstacle while the vehicle is passing it.
class PersistentObstacleMap {
 public:
  using VoxelKey = std::tuple<int, int, int>;

  void configure(double voxel_resolution, std::size_t max_voxels,
                 int confirmation_frames, int confirmation_window_frames) {
    voxel_resolution_ = std::max(0.01, voxel_resolution);
    max_voxels_ = std::max<std::size_t>(1, max_voxels);
    confirmation_frames_ = std::max(1, confirmation_frames);
    confirmation_window_frames_ = std::max(
        confirmation_frames_, confirmation_window_frames);
    clear();
  }

  void clear() {
    voxels_.clear();
    frame_index_ = 0;
  }

  void observeFrame(const std::vector<ObstaclePoint>& points) {
    ++frame_index_;

    // Unconfirmed returns are useful only inside their confirmation window.
    // Removing them also prevents one-shot noise from exhausting the map.
    for (auto it = voxels_.begin(); it != voxels_.end();) {
      const auto& evidence = it->second;
      if (!evidence.confirmed &&
          frame_index_ - evidence.first_frame >=
              static_cast<std::uint64_t>(confirmation_window_frames_)) {
        it = voxels_.erase(it);
      } else {
        ++it;
      }
    }

    // Count a voxel no more than once per scan. A dense cluster of points in
    // one corrupted packet is still only one temporal observation.
    std::map<VoxelKey, ObstaclePoint> frame_voxels;
    for (const auto& point : points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
          !std::isfinite(point.z)) {
        continue;
      }
      const auto key = std::make_tuple(
          static_cast<int>(std::floor(point.x / voxel_resolution_)),
          static_cast<int>(std::floor(point.y / voxel_resolution_)),
          static_cast<int>(std::floor(point.z / voxel_resolution_)));
      frame_voxels[key] = point;
    }

    for (const auto& entry : frame_voxels) {
      auto found = voxels_.find(entry.first);
      if (found == voxels_.end()) {
        if (voxels_.size() >= max_voxels_) continue;
        VoxelEvidence evidence;
        evidence.point = entry.second;
        evidence.hits = 1;
        evidence.first_frame = frame_index_;
        evidence.confirmed = confirmation_frames_ <= 1;
        voxels_.emplace(entry.first, evidence);
        continue;
      }

      auto& evidence = found->second;
      evidence.point = entry.second;
      if (evidence.confirmed) continue;
      ++evidence.hits;
      if (evidence.hits >= confirmation_frames_) {
        evidence.confirmed = true;
      }
    }
  }

  std::vector<ObstaclePoint> confirmedPoints() const {
    std::vector<ObstaclePoint> result;
    result.reserve(voxels_.size());
    for (const auto& entry : voxels_) {
      if (entry.second.confirmed) result.push_back(entry.second.point);
    }
    return result;
  }

  std::size_t confirmedSize() const {
    std::size_t result = 0;
    for (const auto& entry : voxels_) {
      if (entry.second.confirmed) ++result;
    }
    return result;
  }

 private:
  struct VoxelEvidence {
    ObstaclePoint point;
    int hits{0};
    std::uint64_t first_frame{0};
    bool confirmed{false};
  };

  std::map<VoxelKey, VoxelEvidence> voxels_;
  double voxel_resolution_{0.1};
  std::size_t max_voxels_{20000};
  int confirmation_frames_{2};
  int confirmation_window_frames_{3};
  std::uint64_t frame_index_{0};
};

struct LiveClearanceResult {
  bool violated{false};
  double horizontal_distance{std::numeric_limits<double>::infinity()};
  double vertical_distance{std::numeric_limits<double>::infinity()};
  ObstaclePoint obstacle;
};

inline LiveClearanceResult liveHorizontalCylinderClearance(
    const std::vector<ObstaclePoint>& points, const ObstaclePoint& vehicle,
    double horizontal_radius, double vertical_clearance_below,
    double vertical_clearance_above,
    double min_obstacle_z = -std::numeric_limits<double>::infinity()) {
  LiveClearanceResult result;
  for (const auto& point : points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z)) {
      continue;
    }
    if (point.z < min_obstacle_z) continue;
    const double signed_dz = point.z - vehicle.z;
    if (signed_dz < -vertical_clearance_below - 1e-9 ||
        signed_dz > vertical_clearance_above + 1e-9) {
      continue;
    }
    const double dz = std::abs(signed_dz);
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
    double lateral_half_width, double vertical_clearance_below,
    double vertical_clearance_above,
    std::size_t min_points, double min_horizontal_span,
    double min_vertical_span = 0.0,
    double min_obstacle_z = -std::numeric_limits<double>::infinity()) {
  SurfaceCoverageResult result;
  const double norm = std::hypot(forward_x, forward_y);
  if (!(std::isfinite(norm) && norm > 1e-6 && lookahead > 0.0 &&
        lateral_half_width > 0.0 && vertical_clearance_below > 0.0 &&
        vertical_clearance_above > 0.0 && min_vertical_span >= 0.0)) {
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
        dz < -vertical_clearance_below || dz > vertical_clearance_above) {
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
