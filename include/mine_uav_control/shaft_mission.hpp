#pragma once

#include <algorithm>
#include <cmath>

namespace mine_uav_control {

// Depth is positive downward from an external, independently validated source.
// A downward rangefinder alone cannot supply this input or locate the entrance.
class ShaftMission {
 public:
  enum class State { kIdle, kDescending, kReturning, kComplete, kFault };

  struct Config {
    double descent_speed{0.5};
    double return_speed{0.5};
    // A fresh downward-laser reading at or below this distance starts
    // acceleration-limited braking and bottom confirmation.
    double bottom_trigger{5.0};
    double bottom_stop_margin{0.8};
    double bottom_confirm_seconds{0.3};
    double max_depth{450.0};
    double max_duration{1800.0};
    double entrance_tolerance{0.5};
    double max_depth_jump{2.0};
    double max_acceleration{0.5};
  };

  struct Input {
    bool enabled{false};
    bool depth_valid{false};
    double depth{0.0};
    bool range_fresh{false};
    double bottom_range{INFINITY};
    double range_min{0.0};
    double range_max{30.0};
    double dt{0.0};
  };

  struct Output {
    State state{State::kIdle};
    double vertical_speed_enu{0.0};
    bool command_valid{false};
  };

  explicit ShaftMission(const Config& config)
      : config_(config), config_valid_(validConfig(config)) {}

  Output step(const Input& in) {
    if (!in.enabled) {
      state_ = State::kIdle;
      elapsed_ = 0.0;
      confirm_ = 0.0;
      have_previous_depth_ = false;
      commanded_vertical_speed_ = 0.0;
      return {state_, 0.0, false};
    }
    if (state_ == State::kFault || state_ == State::kComplete) {
      return {state_, 0.0, false};
    }
    if (!config_valid_) {
      state_ = State::kFault;
      commanded_vertical_speed_ = 0.0;
      return {state_, 0.0, false};
    }
    if (state_ == State::kIdle &&
        (!in.depth_valid || !in.range_fresh)) {
      // An enable edge may precede first sensor frames. Wait without motion;
      // only an already active mission treats loss of data as a fault.
      return {state_, 0.0, false};
    }
    if (!std::isfinite(in.dt) || in.dt <= 0.0 || in.dt > 1.0 ||
        !in.depth_valid || !std::isfinite(in.depth) ||
        !in.range_fresh || !validRange(in)) {
      state_ = State::kFault;
      commanded_vertical_speed_ = 0.0;
      return {state_, 0.0, false};
    }
    if (have_previous_depth_ &&
        std::abs(in.depth - previous_depth_) > config_.max_depth_jump) {
      state_ = State::kFault;
      commanded_vertical_speed_ = 0.0;
      return {state_, 0.0, false};
    }
    previous_depth_ = in.depth;
    have_previous_depth_ = true;
    if (state_ == State::kIdle) {
      home_depth_ = in.depth;
      state_ = State::kDescending;
      commanded_vertical_speed_ = 0.0;
    }
    elapsed_ += in.dt;
    const double traveled = in.depth - home_depth_;
    if (elapsed_ > config_.max_duration ||
        traveled < -config_.entrance_tolerance - config_.max_depth_jump ||
        traveled > config_.max_depth) {
      state_ = State::kFault;
      commanded_vertical_speed_ = 0.0;
      return {state_, 0.0, false};
    }
    if (state_ == State::kDescending) {
      const bool bottom_close = std::isfinite(in.bottom_range) &&
          in.bottom_range <= config_.bottom_trigger;
      if (bottom_close) {
        confirm_ += in.dt;
      } else {
        confirm_ = 0.0;
      }
      if (confirm_ >= config_.bottom_confirm_seconds) {
        state_ = State::kReturning;
      } else {
        // At the configured bottom threshold, begin braking immediately while the
        // reading is confirmed. A transient close reading therefore slows the
        // vehicle but cannot reverse it. Outside the threshold, retain the
        // stopping-distance guard as an additional bound.
        double target_speed = 0.0;
        if (!bottom_close) {
          const double available = std::isfinite(in.bottom_range)
                                       ? in.bottom_range - config_.bottom_stop_margin
                                       : INFINITY;
          const double braking_speed = std::sqrt(
              2.0 * config_.max_acceleration * std::max(0.0, available));
          target_speed = -std::min(config_.descent_speed, braking_speed);
        }
        return {state_, slewSpeed(target_speed, in.dt), true};
      }
    }
    if (traveled <= config_.entrance_tolerance) {
      state_ = State::kComplete;
      commanded_vertical_speed_ = 0.0;
      return {state_, 0.0, false};
    }
    return {state_, slewSpeed(config_.return_speed, in.dt), true};
  }

  State state() const { return state_; }
  double homeDepth() const { return home_depth_; }

 private:
  static bool validRange(const Input& in) {
    if (!std::isfinite(in.range_min) || !std::isfinite(in.range_max) ||
        in.range_min < 0.0 || in.range_max <= in.range_min) {
      return false;
    }
    return (std::isinf(in.bottom_range) && in.bottom_range > 0.0) ||
           (std::isfinite(in.bottom_range) &&
            in.bottom_range >= in.range_min &&
            in.bottom_range <= in.range_max);
  }

  static bool validConfig(const Config& c) {
    return std::isfinite(c.descent_speed) && c.descent_speed > 0.0 &&
           std::isfinite(c.return_speed) && c.return_speed > 0.0 &&
           std::isfinite(c.bottom_trigger) && c.bottom_trigger > 0.0 &&
           std::isfinite(c.bottom_stop_margin) && c.bottom_stop_margin >= 0.0 &&
           c.bottom_trigger > c.bottom_stop_margin &&
           std::isfinite(c.bottom_confirm_seconds) &&
           c.bottom_confirm_seconds > 0.0 &&
           std::isfinite(c.max_depth) && c.max_depth > 0.0 &&
           std::isfinite(c.max_duration) && c.max_duration > 0.0 &&
           std::isfinite(c.entrance_tolerance) &&
           c.entrance_tolerance >= 0.0 &&
           std::isfinite(c.max_depth_jump) && c.max_depth_jump > 0.0 &&
           std::isfinite(c.max_acceleration) && c.max_acceleration > 0.0;
  }

  double slewSpeed(double target, double dt) {
    const double max_delta = config_.max_acceleration * dt;
    const double delta = std::max(-max_delta, std::min(max_delta,
                                                       target - commanded_vertical_speed_));
    commanded_vertical_speed_ += delta;
    return commanded_vertical_speed_;
  }

  Config config_;
  bool config_valid_{false};
  State state_{State::kIdle};
  double home_depth_{0.0};
  double previous_depth_{0.0};
  double elapsed_{0.0};
  double confirm_{0.0};
  double commanded_vertical_speed_{0.0};
  bool have_previous_depth_{false};
};

}  // namespace mine_uav_control
