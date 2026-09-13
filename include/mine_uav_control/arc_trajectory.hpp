#ifndef MINE_UAV_CONTROL_ARC_TRAJECTORY_HPP_
#define MINE_UAV_CONTROL_ARC_TRAJECTORY_HPP_

#include <string>

namespace mine_uav_control {

struct Pose2D {
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double yaw{0.0};
};

struct ArcConfig {
  double radius{3.0};
  double speed{0.8};
  double arc_angle_deg{60.0};
  int direction{1};
  double rate{50.0};
};

struct TrajectoryPoint {
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double vx{0.0};
  double vy{0.0};
  double vz{0.0};
  double yaw{0.0};
  bool finished{false};
};

class ArcTrajectory {
 public:
  ArcTrajectory(const Pose2D& start, const ArcConfig& config);

  bool valid() const noexcept;
  const std::string& error() const noexcept;
  double duration() const noexcept;

  // elapsed_sec is clamped to the interval [0, duration()].
  TrajectoryPoint sample(double elapsed_sec) const noexcept;

  // Wrap an angle to the inclusive interval [-pi, pi].
  static double wrapYaw(double yaw) noexcept;

 private:
  Pose2D start_;
  ArcConfig config_;
  double center_x_{0.0};
  double center_y_{0.0};
  double initial_radius_angle_{0.0};
  double arc_angle_rad_{0.0};
  double omega_{0.0};
  double duration_{0.0};
  bool valid_{false};
  std::string error_;
};

}  // namespace mine_uav_control

#endif  // MINE_UAV_CONTROL_ARC_TRAJECTORY_HPP_
