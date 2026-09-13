#include "mine_uav_control/arc_trajectory.hpp"

#include <cmath>

namespace mine_uav_control {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;

bool finite(double value) {
  return std::isfinite(value);
}

}  // namespace

ArcTrajectory::ArcTrajectory(const Pose2D& start, const ArcConfig& config)
    : start_(start), config_(config) {
  if (!finite(start_.x) || !finite(start_.y) || !finite(start_.z) ||
      !finite(start_.yaw)) {
    error_ = "start pose contains a non-finite value";
    return;
  }
  if (!finite(config_.radius) || config_.radius <= 0.0) {
    error_ = "radius must be finite and greater than zero";
    return;
  }
  if (!finite(config_.speed) || config_.speed <= 0.0) {
    error_ = "speed must be finite and greater than zero";
    return;
  }
  if (!finite(config_.arc_angle_deg) || config_.arc_angle_deg < 0.0) {
    error_ = "arc_angle_deg must be finite and non-negative";
    return;
  }
  if (config_.direction != 1 && config_.direction != -1) {
    error_ = "direction must be +1 or -1";
    return;
  }
  if (!finite(config_.rate) || config_.rate <= 0.0) {
    error_ = "rate must be finite and greater than zero";
    return;
  }

  start_.yaw = wrapYaw(start_.yaw);
  arc_angle_rad_ = config_.arc_angle_deg * kPi / 180.0;
  omega_ = config_.speed / config_.radius;
  duration_ = arc_angle_rad_ / omega_;

  // The sign selects the left (+1, counter-clockwise) or right (-1) side.
  const double side = static_cast<double>(config_.direction);
  center_x_ = start_.x - side * config_.radius * std::sin(start_.yaw);
  center_y_ = start_.y + side * config_.radius * std::cos(start_.yaw);

  const double initial_radius_x = start_.x - center_x_;
  const double initial_radius_y = start_.y - center_y_;
  initial_radius_angle_ = std::atan2(initial_radius_y, initial_radius_x);
  valid_ = true;
}

bool ArcTrajectory::valid() const noexcept {
  return valid_;
}

const std::string& ArcTrajectory::error() const noexcept {
  return error_;
}

double ArcTrajectory::duration() const noexcept {
  return duration_;
}

TrajectoryPoint ArcTrajectory::sample(double elapsed_sec) const noexcept {
  TrajectoryPoint point;
  if (!valid_) {
    return point;
  }

  const bool finished = elapsed_sec >= duration_;
  double elapsed = elapsed_sec;
  if (!finite(elapsed) || elapsed < 0.0) {
    elapsed = 0.0;
  }
  if (elapsed > duration_) {
    elapsed = duration_;
  }

  const double side = static_cast<double>(config_.direction);
  const double radius_angle = initial_radius_angle_ + side * omega_ * elapsed;
  const double radius_x = config_.radius * std::cos(radius_angle);
  const double radius_y = config_.radius * std::sin(radius_angle);

  point.x = center_x_ + radius_x;
  point.y = center_y_ + radius_y;
  point.z = start_.z;
  point.yaw = wrapYaw(start_.yaw + side * omega_ * elapsed);
  point.finished = finished;

  if (!finished) {
    point.vx = -side * config_.speed * std::sin(radius_angle);
    point.vy = side * config_.speed * std::cos(radius_angle);
    point.vz = 0.0;
    // Keep yaw exactly aligned with the velocity vector.
    point.yaw = wrapYaw(std::atan2(point.vy, point.vx));
  } else {
    point.vx = 0.0;
    point.vy = 0.0;
    point.vz = 0.0;
    point.yaw = wrapYaw(start_.yaw + side * arc_angle_rad_);
  }

  return point;
}

double ArcTrajectory::wrapYaw(double yaw) noexcept {
  if (!finite(yaw)) {
    return 0.0;
  }
  double wrapped = std::fmod(yaw, kTwoPi);
  if (wrapped > kPi) {
    wrapped -= kTwoPi;
  } else if (wrapped < -kPi) {
    wrapped += kTwoPi;
  }
  return wrapped;
}

}  // namespace mine_uav_control
