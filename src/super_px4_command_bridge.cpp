#include "mine_uav_control/motion_continuity.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_srvs/SetBool.h>

#include "mine_uav_control/offboard_mode_guard.hpp"

namespace {

constexpr double kPi = 3.14159265358979323846;

bool finite(double value) { return std::isfinite(value); }

double normalizeAngle(double angle) {
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

double yawFromQuaternion(const geometry_msgs::Quaternion& q) {
  const double norm =
      std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  if (!finite(norm) || norm < 1e-6) {
    return 0.0;
  }
  const double x = q.x / norm;
  const double y = q.y / norm;
  const double z = q.z / norm;
  const double w = q.w / norm;
  return std::atan2(2.0 * (w * z + x * y),
                    1.0 - 2.0 * (y * y + z * z));
}

double vectorNorm(double x, double y, double z) {
  return std::sqrt(x * x + y * y + z * z);
}

class SuperPx4CommandBridge {
 public:
  SuperPx4CommandBridge(const ros::NodeHandle& nh,
                        const ros::NodeHandle& private_nh)
      : nh_(nh), private_nh_(private_nh) {
    private_nh_.param("command_topic", command_topic_,
                      std::string("/planning/pos_cmd"));
    private_nh_.param("output_topic", output_topic_,
                      std::string("/mine_uav/setpoint_cmd"));
    private_nh_.param("expected_frame", expected_frame_,
                      std::string("camera_init"));
    private_nh_.param("command_timeout", command_timeout_, 0.25);
    private_nh_.param("local_pose_timeout", local_pose_timeout_, 0.5);
    private_nh_.param("output_rate", output_rate_, 50.0);
    private_nh_.param("max_speed", max_speed_, 1.0);
    private_nh_.param("max_acceleration", max_acceleration_, 3.0);
    private_nh_.param("max_yaw_rate", max_yaw_rate_, 0.75);
    private_nh_.param("max_position_rate", max_position_rate_, 1.10);
    private_nh_.param("max_velocity_slew", max_velocity_slew_, 1.75);
    private_nh_.param("max_acceleration_slew", max_acceleration_slew_, 6.0);
    private_nh_.param("continuity_reset_gap", continuity_reset_gap_, 0.30);
    private_nh_.param("continuity_nominal_dt", continuity_nominal_dt_, 0.01);
    private_nh_.param("reacquire_blend_duration", reacquire_blend_duration_, 3.5);
    private_nh_.param("max_initial_command_distance",
                      max_initial_command_distance_, 0.65);
    private_nh_.param("max_initial_yaw_error", max_initial_yaw_error_, 0.70);
    private_nh_.param("max_horizontal_radius", max_horizontal_radius_, 40.0);
    private_nh_.param("min_height", min_height_, -0.5);
    private_nh_.param("max_height", max_height_, 1.8);
    private_nh_.param("height_clamp_tolerance", height_clamp_tolerance_, 0.25);
    private_nh_.param("max_alignment_translation_change",
                      max_alignment_translation_change_, 0.05);
    private_nh_.param("max_alignment_yaw_change",
                      max_alignment_yaw_change_, 0.05);
    private_nh_.param("use_acceleration", use_acceleration_, true);
    private_nh_.param("auto_enable_topic", auto_enable_topic_,
                      std::string("/mine_uav/mission/auto_enable"));
    private_nh_.param("exploration_status_topic", exploration_status_topic_,
                      std::string("/mine_uav/exploration/status"));
    private_nh_.param("automatic_mode_switch", automatic_mode_switch_, true);
    private_nh_.param("require_armed_for_offboard",
                      require_armed_for_offboard_, true);
    private_nh_.param("prestream_duration", prestream_duration_, 1.0);
    private_nh_.param("mode_request_interval", mode_request_interval_, 1.0);
    private_nh_.param("offboard_mode", offboard_mode_,
                      std::string("OFFBOARD"));
    private_nh_.param("fallback_mode", fallback_mode_,
                      std::string("POSCTL"));
    private_nh_.param("return_recovery_timeout", return_recovery_timeout_,
                      12.0);
    private_nh_.param("software_enable_default", software_enabled_, true);

    command_timeout_ = std::max(0.05, command_timeout_);
    local_pose_timeout_ = std::max(0.1, local_pose_timeout_);
    output_rate_ = std::max(10.0, output_rate_);
    max_speed_ = std::max(0.1, max_speed_);
    max_acceleration_ = std::max(0.1, max_acceleration_);
    max_yaw_rate_ = std::max(0.1, max_yaw_rate_);
    max_position_rate_ = std::max(max_speed_, max_position_rate_);
    max_velocity_slew_ = std::max(max_acceleration_, max_velocity_slew_);
    max_acceleration_slew_ = std::max(0.5, max_acceleration_slew_);
    continuity_reset_gap_ = std::max(0.05, continuity_reset_gap_);
    continuity_nominal_dt_ = std::max(0.002, continuity_nominal_dt_);
    reacquire_blend_duration_ = std::max(0.5, reacquire_blend_duration_);
    max_initial_command_distance_ = std::max(0.1, max_initial_command_distance_);
    max_initial_yaw_error_ = std::max(0.1, max_initial_yaw_error_);
    max_horizontal_radius_ = std::max(1.0, max_horizontal_radius_);
    max_alignment_translation_change_ =
        std::max(0.001, max_alignment_translation_change_);
    max_alignment_yaw_change_ = std::max(0.001, max_alignment_yaw_change_);
    prestream_duration_ = std::max(1.0, prestream_duration_);
    mode_request_interval_ = std::max(0.5, mode_request_interval_);
    return_recovery_timeout_ = std::max(2.0, return_recovery_timeout_);
    if (min_height_ > max_height_) {
      std::swap(min_height_, max_height_);
    }

    command_subscriber_ = nh_.subscribe(
        command_topic_, 10, &SuperPx4CommandBridge::commandCallback, this);
    alignment_subscriber_ = nh_.subscribe(
        "/mine_uav/task1/fastlio_to_px4_alignment", 1,
        &SuperPx4CommandBridge::alignmentCallback, this);
    mission_subscriber_ = nh_.subscribe(
        "/mine_uav/mission/goaf_enable", 1,
        &SuperPx4CommandBridge::missionCallback, this);
    auto_enable_subscriber_ = nh_.subscribe(
        auto_enable_topic_, 1,
        &SuperPx4CommandBridge::autoEnableCallback, this);
    vision_subscriber_ = nh_.subscribe(
        "/mine_uav/task1/vision_healthy", 1,
        &SuperPx4CommandBridge::visionCallback, this);
    finished_subscriber_ = nh_.subscribe(
        "/mine_uav/exploration/finished", 1,
        &SuperPx4CommandBridge::finishedCallback, this);
    returning_subscriber_ = nh_.subscribe(
        "/mine_uav/exploration/returning", 1,
        &SuperPx4CommandBridge::returningCallback, this);
    exploration_status_subscriber_ = nh_.subscribe(
        exploration_status_topic_, 1,
        &SuperPx4CommandBridge::explorationStatusCallback, this);
    state_subscriber_ = nh_.subscribe(
        "/mavros/state", 10, &SuperPx4CommandBridge::stateCallback, this);
    local_pose_subscriber_ = nh_.subscribe(
        "/mavros/local_position/pose", 10,
        &SuperPx4CommandBridge::localPoseCallback, this);

    command_publisher_ =
        nh_.advertise<mavros_msgs::PositionTarget>(output_topic_, 10);
    ready_publisher_ = nh_.advertise<std_msgs::Bool>(
        "/mine_uav/task1/command_ready", 1, true);
    status_publisher_ = nh_.advertise<std_msgs::String>(
        "/mine_uav/task1/command_status", 1, true);
    return_request_publisher_ = nh_.advertise<std_msgs::Bool>(
        "/mine_uav/exploration/return_home", 1, true);
    enable_service_ = private_nh_.advertiseService(
        "enable", &SuperPx4CommandBridge::enableCallback, this);
    set_mode_client_ =
        nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    output_timer_ = nh_.createTimer(
        ros::Duration(1.0 / output_rate_),
        &SuperPx4CommandBridge::outputTimerCallback, this);

    publishReady(false);
    publishReturnRequest(false);
    publishStatus("WAIT_ALIGNMENT");
    ROS_WARN("SUPER-to-PX4 bridge ready: RC auto-enable and task selection are "
             "required; the bridge may request OFFBOARD but never arms PX4");
  }

 private:
  void alignmentCallback(
      const geometry_msgs::TransformStamped::ConstPtr& transform) {
    const auto& t = transform->transform.translation;
    const auto& q = transform->transform.rotation;
    const double quaternion_norm =
        std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    if (!finite(t.x) || !finite(t.y) || !finite(t.z) || !finite(q.x) ||
        !finite(q.y) || !finite(q.z) || !finite(q.w) ||
        !finite(quaternion_norm) || quaternion_norm < 1e-6) {
      alignment_ready_ = false;
      latchFault("INVALID_ALIGNMENT");
      return;
    }
    const double new_yaw = yawFromQuaternion(q);
    const bool alignment_changed = alignment_ready_ &&
        (vectorNorm(t.x - alignment_x_, t.y - alignment_y_,
                    t.z - alignment_z_) > max_alignment_translation_change_ ||
         std::abs(normalizeAngle(new_yaw - alignment_yaw_)) >
             max_alignment_yaw_change_);
    if (alignment_changed && auto_enabled_ && mission_enabled_) {
      // Updating this transform while executing would reinterpret the old
      // SUPER trajectory in a new PX4 frame. Invalidate it and exit first.
      valid_command_ = false;
      latchFault("ALIGNMENT_CHANGED_DURING_TASK");
      resetPrestream();
    }
    // Pin the frame during autonomous flight, even for changes below the
    // fault threshold. Tiny accepted alignment updates can create P/V/A steps.
    if (alignment_ready_ && auto_enabled_ && mission_enabled_) {
      updateStatus();
      return;
    }
    alignment_x_ = t.x;
    alignment_y_ = t.y;
    alignment_z_ = t.z;
    alignment_yaw_ = new_yaw;
    alignment_ready_ = true;
    updateStatus();
  }

  void missionCallback(const std_msgs::Bool::ConstPtr& enabled) {
    mission_enabled_ = enabled->data;
    if (!mission_enabled_) {
      recovery_pending_ = false;
      recovery_attempted_ = false;
      returning_home_ = false;
      return_route_wait_since_ = ros::Time();
      publishReturnRequest(false);
      beginOffboardExit("TASK1_DESELECTED");
      resetPrestream();
      resetContinuityGuard();
      resetReacquire();
    }
    updateStatus();
  }

  void autoEnableCallback(const std_msgs::Bool::ConstPtr& enabled) {
    const bool was_enabled = auto_enabled_;
    auto_enabled_ = enabled->data;
    if (was_enabled && !auto_enabled_) {
      recovery_pending_ = false;
      recovery_attempted_ = false;
      publishReturnRequest(false);
      beginOffboardExit("AUTO_DISABLED");
      fault_latched_ = false;
      resetPrestream();
      resetContinuityGuard();
      resetReacquire();
    }
    updateStatus();
  }

  void visionCallback(const std_msgs::Bool::ConstPtr& healthy) {
    if (vision_healthy_ && !healthy->data && auto_enabled_) {
      latchFault("VISION_UNHEALTHY");
    }
    vision_healthy_ = healthy->data;
    updateStatus();
  }

  void finishedCallback(const std_msgs::Bool::ConstPtr& finished) {
    mission_complete_ = finished->data;
    if (mission_complete_) {
      recovery_pending_ = false;
      publishReturnRequest(false);
      beginOffboardExit("TASK1_COMPLETE");
      resetPrestream();
      resetContinuityGuard();
      resetReacquire();
    }
    updateStatus();
  }

  void returningCallback(const std_msgs::Bool::ConstPtr& returning) {
    returning_home_ = returning->data;
  }

  void explorationStatusCallback(
      const std_msgs::String::ConstPtr& status) {
    const std::string state = status->data.substr(
        0, status->data.find(':'));
    hold_requested_ = state == "WAIT_DATA" || state == "DISABLED" ||
        state == "WAIT_GOAL" ||
        state == "WAIT_FRONTIER" || state == "WAIT_MAP_CLOSURE" ||
        state == "WAIT_MODEL_COVERAGE" || state == "RETURN_ROUTE_WAIT";
    if (state == "RETURN_ROUTE_WAIT" && returning_home_) {
      if (return_route_wait_since_.isZero())
        return_route_wait_since_ = ros::Time::now();
    } else {
      return_route_wait_since_ = ros::Time();
    }
    if (hold_requested_ && (!have_published_target_ ||
                            outputNearRest(published_target_))) {
      activateExplorationHold(ros::Time::now());
    } else if (!hold_requested_) {
      exploration_status_hold_ = false;
    }
  }

  bool outputNearRest(const mavros_msgs::PositionTarget& target) const {
    return vectorNorm(target.velocity.x, target.velocity.y,
                      target.velocity.z) <= 0.02 &&
           vectorNorm(target.acceleration_or_force.x,
                      target.acceleration_or_force.y,
                      target.acceleration_or_force.z) <= 0.05 &&
           std::abs(target.yaw_rate) <= 0.02;
  }

  void activateExplorationHold(const ros::Time& now) {
    if (exploration_status_hold_) return;
    exploration_status_hold_ = true;
    valid_command_ = false;
    resetReacquire();
    if (px4_mode_ == offboard_mode_ && !hold_target_latched_ &&
        localPoseFresh(now)) {
      latched_hold_target_ = makeLastCommandHoldTarget(now);
      hold_target_latched_ = true;
    }
  }

  void stateCallback(const mavros_msgs::State::ConstPtr& state) {
    if (mavros_connected_ && !state->connected) {
      latchFault("MAVROS_DISCONNECTED");
    }
    mavros_connected_ = state->connected;
    mavros_armed_ = state->armed;
    const std::string previous_mode = px4_mode_;
    px4_mode_ = state->mode;
    if (px4_mode_ != offboard_mode_) {
      have_published_target_ = false;
      hold_target_latched_ = false;
    }
    if (mine_uav_control::externalOffboardExit(
            previous_mode, px4_mode_, offboard_mode_, offboard_owned_,
            exit_requested_)) {
      // CH5/manual takeover and PX4 failsafe exits both revoke this
      // bridge's permission to request OFFBOARD again. A deliberate
      // task-enable transition is required before the next attempt.
      latchFault("OFFBOARD_EXITED_EXTERNALLY");
      valid_command_ = false;
      resetPrestream();
      resetContinuityGuard();
      resetReacquire();
    }
    if (px4_mode_ != offboard_mode_ && exit_requested_) {
      exit_requested_ = false;
      offboard_owned_ = false;
    }
    updateStatus();
  }

  void localPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& pose) {
    const auto& p = pose->pose.position;
    const auto& q = pose->pose.orientation;
    if (!finite(p.x) || !finite(p.y) || !finite(p.z) || !finite(q.x) ||
        !finite(q.y) || !finite(q.z) || !finite(q.w)) {
      return;
    }
    latest_local_pose_ = *pose;
    last_local_pose_time_ = ros::Time::now();
    have_local_pose_ = true;
  }

  bool gatesReady() const {
    return software_enabled_ && auto_enabled_ && mission_enabled_ &&
           !mission_complete_ &&
           alignment_ready_ && vision_healthy_ && mavros_connected_ &&
           !fault_latched_;
  }

  bool localPoseFresh(const ros::Time& now) const {
    return have_local_pose_ && !last_local_pose_time_.isZero() &&
           (now - last_local_pose_time_).toSec() <= local_pose_timeout_;
  }

  bool enableCallback(std_srvs::SetBool::Request& request,
                      std_srvs::SetBool::Response& response) {
    if (!request.data) {
      software_enabled_ = false;
      beginOffboardExit("SOFTWARE_INHIBIT");
      response.success = true;
      response.message = "task-one automatic control inhibited";
      updateStatus();
      return true;
    }
    software_enabled_ = true;
    response.success = true;
    response.message =
        "software inhibit cleared; RC auto-enable still controls execution";
    updateStatus();
    return true;
  }

  bool validCommand(const quadrotor_msgs::PositionCommand& command,
                    std::string* reason) const {
    if (!expected_frame_.empty() &&
        command.header.frame_id != expected_frame_) {
      *reason = "FRAME_MISMATCH";
      return false;
    }
    const auto& p = command.position;
    const auto& v = command.velocity;
    const auto& a = command.acceleration;
    if (!finite(p.x) || !finite(p.y) || !finite(p.z) || !finite(v.x) ||
        !finite(v.y) || !finite(v.z) || !finite(a.x) || !finite(a.y) ||
        !finite(a.z) || !finite(command.yaw) || !finite(command.yaw_dot)) {
      *reason = "NONFINITE_COMMAND";
      return false;
    }
    if (command.trajectory_flag !=
            quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY &&
        command.trajectory_flag !=
            quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_EMER) {
      *reason = "TRAJECTORY_NOT_ACTIVE";
      return false;
    }
    if (vectorNorm(v.x, v.y, v.z) > max_speed_) {
      *reason = "SPEED_LIMIT";
      return false;
    }
    if (vectorNorm(a.x, a.y, a.z) > max_acceleration_) {
      *reason = "ACCELERATION_LIMIT";
      return false;
    }
    return true;
  }

  mavros_msgs::PositionTarget convert(
      const quadrotor_msgs::PositionCommand& command) const {
    const double c = std::cos(alignment_yaw_);
    const double s = std::sin(alignment_yaw_);
    mavros_msgs::PositionTarget output;
    output.header.stamp = ros::Time::now();
    output.header.frame_id = "map";
    output.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    output.type_mask = 0;

    output.position.x = c * command.position.x - s * command.position.y +
                        alignment_x_;
    output.position.y = s * command.position.x + c * command.position.y +
                        alignment_y_;
    output.position.z = command.position.z + alignment_z_;
    output.velocity.x =
        c * command.velocity.x - s * command.velocity.y;
    output.velocity.y =
        s * command.velocity.x + c * command.velocity.y;
    output.velocity.z = command.velocity.z;
    output.acceleration_or_force.x =
        c * command.acceleration.x - s * command.acceleration.y;
    output.acceleration_or_force.y =
        s * command.acceleration.x + c * command.acceleration.y;
    output.acceleration_or_force.z = command.acceleration.z;
    output.yaw = normalizeAngle(command.yaw + alignment_yaw_);
    output.yaw_rate = command.yaw_dot;
    if (!use_acceleration_) {
      output.type_mask |= mavros_msgs::PositionTarget::IGNORE_AFX |
                          mavros_msgs::PositionTarget::IGNORE_AFY |
                          mavros_msgs::PositionTarget::IGNORE_AFZ;
    }
    return output;
  }

  void resetContinuityGuard() {
    have_previous_target_ = false;
    previous_target_time_ = ros::Time();
  }

  void resetReacquire() {
    reacquire_active_ = false;
    reacquire_start_time_ = ros::Time();
    source_stream_active_ = false;
  }

  void startEmergencyBrake() {
    if (brake_active_ || !have_published_target_) return;
    brake_start_target_ = published_target_;
    brake_start_time_ = published_target_time_;
    const auto& v = brake_start_target_.velocity;
    const double speed = vectorNorm(v.x, v.y, v.z);
    brake_duration_ = std::max(1.5, 2.0 * speed / max_velocity_slew_);
    const double p[3] = {brake_start_target_.position.x,
                         brake_start_target_.position.y,
                         brake_start_target_.position.z};
    const double velocity[3] = {v.x, v.y, v.z};
    const auto& a = brake_start_target_.acceleration_or_force;
    const double acceleration[3] = {a.x, a.y, a.z};
    for (int i = 0; i < 3; ++i) {
      brake_stop_position_[i] = p[i] + 0.5 * velocity[i] * brake_duration_;
      brake_xyz_[i] = mine_uav_control::QuinticOffset1D::make(
          p[i] - brake_stop_position_[i], velocity[i], acceleration[i],
          brake_duration_);
    }
    brake_stop_yaw_ = brake_start_target_.yaw +
                      0.5 * brake_start_target_.yaw_rate * brake_duration_;
    brake_yaw_ = mine_uav_control::QuinticOffset1D::make(
        brake_start_target_.yaw - brake_stop_yaw_,
        brake_start_target_.yaw_rate, 0.0, brake_duration_);
    brake_active_ = true;
    hold_target_latched_ = false;
    ROS_WARN("SUPER command interrupted during motion; braking continuously over %.2f s",
             brake_duration_);
  }

  mavros_msgs::PositionTarget emergencyBrakeTarget(const ros::Time& now) const {
    auto target = brake_start_target_;
    const double elapsed = std::max(0.0,
        (now - brake_start_time_).toSec());
    const auto x = brake_xyz_[0].eval(elapsed);
    const auto y = brake_xyz_[1].eval(elapsed);
    const auto z = brake_xyz_[2].eval(elapsed);
    target.position.x = brake_stop_position_[0] + x.p;
    target.position.y = brake_stop_position_[1] + y.p;
    target.position.z = brake_stop_position_[2] + z.p;
    target.velocity.x = x.v;
    target.velocity.y = y.v;
    target.velocity.z = z.v;
    target.acceleration_or_force.x = x.a;
    target.acceleration_or_force.y = y.a;
    target.acceleration_or_force.z = z.a;
    const auto yaw = brake_yaw_.eval(elapsed);
    target.yaw = normalizeAngle(brake_stop_yaw_ + yaw.p);
    target.yaw_rate = yaw.v;
    target.header.stamp = now;
    return target;
  }

  mavros_msgs::PositionTarget makeLastCommandHoldTarget(
      const ros::Time& now) const {
    if (last_command_time_.isZero()) {
      return makeHoldTarget(now);
    }
    auto hold = have_published_target_ ? published_target_ : latest_target_;
    hold.header.stamp = now;
    hold.header.frame_id = "map";
    hold.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    hold.type_mask = mavros_msgs::PositionTarget::IGNORE_VX |
                     mavros_msgs::PositionTarget::IGNORE_VY |
                     mavros_msgs::PositionTarget::IGNORE_VZ |
                     mavros_msgs::PositionTarget::IGNORE_AFX |
                     mavros_msgs::PositionTarget::IGNORE_AFY |
                     mavros_msgs::PositionTarget::IGNORE_AFZ |
                     mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
    hold.velocity.x = hold.velocity.y = hold.velocity.z = 0.0;
    hold.acceleration_or_force.x = 0.0;
    hold.acceleration_or_force.y = 0.0;
    hold.acceleration_or_force.z = 0.0;
    hold.yaw_rate = 0.0;
    return hold;
  }

  void startReacquire(const mavros_msgs::PositionTarget& start,
                      const mavros_msgs::PositionTarget& desired,
                      const ros::Time& now) {
    const double T = reacquire_blend_duration_;
    const double sp[3] = {start.position.x, start.position.y, start.position.z};
    const double sv[3] = {start.velocity.x, start.velocity.y, start.velocity.z};
    const double sa[3] = {start.acceleration_or_force.x, start.acceleration_or_force.y, start.acceleration_or_force.z};
    const double dp[3] = {desired.position.x, desired.position.y, desired.position.z};
    const double dv[3] = {desired.velocity.x, desired.velocity.y, desired.velocity.z};
    const double da[3] = {desired.acceleration_or_force.x, desired.acceleration_or_force.y, desired.acceleration_or_force.z};
    for (int i = 0; i < 3; ++i) {
      reacquire_xyz_[i] = mine_uav_control::QuinticOffset1D::make(
          sp[i]-dp[i], sv[i]-dv[i], sa[i]-da[i], T);
    }
    reacquire_yaw_ = mine_uav_control::QuinticOffset1D::make(
        normalizeAngle(start.yaw-desired.yaw), start.yaw_rate-desired.yaw_rate, 0.0, T);
    reacquire_start_time_ = now;
    reacquire_active_ = true;
    ROS_INFO("Starting derivative-consistent SUPER handoff over %.2f s (offset %.3f m)",
             T, vectorNorm(sp[0]-dp[0], sp[1]-dp[1], sp[2]-dp[2]));
  }

  mavros_msgs::PositionTarget blendReacquire(
      const mavros_msgs::PositionTarget& desired, const ros::Time& now) {
    if (!reacquire_active_) return desired;
    double elapsed=(now-reacquire_start_time_).toSec();
    if (!std::isfinite(elapsed) || elapsed<0.0) { elapsed=0.0; reacquire_start_time_=now; }
    mavros_msgs::PositionTarget output=desired;
    const auto x=reacquire_xyz_[0].eval(elapsed);
    const auto y=reacquire_xyz_[1].eval(elapsed);
    const auto z=reacquire_xyz_[2].eval(elapsed);
    output.position.x+=x.p; output.position.y+=y.p; output.position.z+=z.p;
    output.velocity.x+=x.v; output.velocity.y+=y.v; output.velocity.z+=z.v;
    output.acceleration_or_force.x+=x.a; output.acceleration_or_force.y+=y.a; output.acceleration_or_force.z+=z.a;
    const auto yaw=reacquire_yaw_.eval(elapsed);
    output.yaw=normalizeAngle(output.yaw+yaw.p);
    output.yaw_rate+=yaw.v;
    output.header.stamp=now;
    if (elapsed>=reacquire_blend_duration_-1e-9) {
      reacquire_active_=false;
      ROS_INFO("Derivative-consistent SUPER handoff complete");
    }
    return output;
  }

  bool continuityAccept(const mavros_msgs::PositionTarget& command,
                        const ros::Time& now, std::string* reason) {
    if (std::abs(command.yaw_rate) > max_yaw_rate_) {
      *reason = "YAW_RATE_LIMIT";
      return false;
    }

    if (!have_previous_target_ || previous_target_time_.isZero()) {
      if (have_local_pose_) {
        const auto& p = latest_local_pose_.pose.position;
        const double initial_distance = vectorNorm(
            command.position.x - p.x,
            command.position.y - p.y,
            command.position.z - p.z);
        if (initial_distance > max_initial_command_distance_ + 1e-6) {
          *reason = "INITIAL_COMMAND_POSITION_JUMP";
          return false;
        }
        const double local_yaw = yawFromQuaternion(latest_local_pose_.pose.orientation);
        if (std::abs(normalizeAngle(command.yaw - local_yaw)) >
            max_initial_yaw_error_) {
          *reason = "INITIAL_COMMAND_YAW_JUMP";
          return false;
        }
      }
      previous_target_ = command;
      previous_target_time_ = now;
      have_previous_target_ = true;
      return true;
    }

    const double dt = (now - previous_target_time_).toSec();
    if (dt <= 0.0) {
      *reason = "COMMAND_NONMONOTONIC_STAMP";
      return false;
    }
    const double position_step = vectorNorm(
        command.position.x - previous_target_.position.x,
        command.position.y - previous_target_.position.y,
        command.position.z - previous_target_.position.z);
    // SUPER can emit a nominal 10 ms trajectory step only 2 ms after the
    // preceding ROS stamp. Keep the same 15 mm sample slack as the checked
    // PX4 output; a physical command jump still exceeds this bound.
    if (!mine_uav_control::sourcePositionStepAllowed(
            position_step, dt, max_position_rate_)) {
      *reason = "COMMAND_POSITION_JUMP";
      return false;
    }

    const double velocity_slew = vectorNorm(
        command.velocity.x - previous_target_.velocity.x,
        command.velocity.y - previous_target_.velocity.y,
        command.velocity.z - previous_target_.velocity.z) / dt;
    if (velocity_slew > max_velocity_slew_) {
      *reason = "COMMAND_VELOCITY_JUMP";
      return false;
    }

    const double acceleration_slew = vectorNorm(
        command.acceleration_or_force.x - previous_target_.acceleration_or_force.x,
        command.acceleration_or_force.y - previous_target_.acceleration_or_force.y,
        command.acceleration_or_force.z - previous_target_.acceleration_or_force.z) / dt;
    if (acceleration_slew > max_acceleration_slew_) {
      *reason = "COMMAND_ACCELERATION_JUMP";
      return false;
    }

    const double yaw_rate_from_samples =
        std::abs(normalizeAngle(command.yaw - previous_target_.yaw)) / dt;
    if (yaw_rate_from_samples > max_yaw_rate_) {
      *reason = "COMMAND_YAW_JUMP";
      return false;
    }

    previous_target_ = command;
    previous_target_time_ = now;
    return true;
  }

  bool insideFlightVolume(const mavros_msgs::PositionTarget& command,
                          std::string* reason) const {
    const double horizontal_radius =
        std::hypot(command.position.x, command.position.y);
    if (horizontal_radius > max_horizontal_radius_) {
      *reason = "HORIZONTAL_GEOFENCE";
      return false;
    }
    if (command.position.z < min_height_ || command.position.z > max_height_) {
      *reason = "HEIGHT_GEOFENCE";
      return false;
    }
    return true;
  }

  void commandCallback(
      const quadrotor_msgs::PositionCommand::ConstPtr& command) {
    if (recovery_pending_ &&
        (!returning_home_ ||
         command->trajectory_id == recovery_source_trajectory_id_)) {
      return;
    }
    std::string reason;
    const ros::Time receipt = ros::Time::now();
    const double source_age = (receipt - command->header.stamp).toSec();
    if (command->header.stamp.isZero() || source_age < -0.05 ||
        source_age > command_timeout_) {
      valid_command_ = false;
      latchFault("COMMAND_STALE_OR_FUTURE_STAMP");
      return;
    }
    if (have_previous_target_ && command->header.stamp == previous_target_time_) {
      // Same-time duplicates provide no new motion information; do not invent dt.
      return;
    }
    if (!validCommand(*command, &reason)) {
      valid_command_ = false;
      if (reason == "TRAJECTORY_NOT_ACTIVE") {
        publishStatus(reason);
      } else {
        latchFault(reason);
      }
      return;
    }
    if (exploration_status_hold_) {
      valid_command_ = false;
      return;
    }
    const auto output = convert(*command);
    auto bounded_output = output;
    if (bounded_output.position.z > max_height_ &&
        bounded_output.position.z <= max_height_ + height_clamp_tolerance_) {
      ROS_WARN_THROTTLE(
          2.0,
          "Clamping small SUPER height overshoot %.3f to max_height %.3f",
          bounded_output.position.z, max_height_);
      bounded_output.position.z = max_height_;
    } else if (bounded_output.position.z < min_height_ &&
               bounded_output.position.z >= min_height_ -
                   height_clamp_tolerance_) {
      ROS_WARN_THROTTLE(
          2.0,
          "Clamping small SUPER height undershoot %.3f to min_height %.3f",
          bounded_output.position.z, min_height_);
      bounded_output.position.z = min_height_;
    }
    if (!insideFlightVolume(bounded_output, &reason)) {
      ROS_ERROR_THROTTLE(
          1.0,
          "Rejecting SUPER command outside flight volume: xyz=(%.3f, %.3f, %.3f), "
          "limits: radius<=%.3f, z=[%.3f, %.3f], alignment_z=%.3f",
          bounded_output.position.x, bounded_output.position.y,
          bounded_output.position.z,
          max_horizontal_radius_, min_height_, max_height_, alignment_z_);
      valid_command_ = false;
      latchFault(reason);
      return;
    }
    const ros::Time now = ros::Time::now();
    // Preserve SUPER's collision-checked P/V/A trajectory verbatim. Interpolating
    // P, V and A independently is neither kinematically valid nor collision safe.
    auto accepted_output = bounded_output;
    if (!continuityAccept(accepted_output, command->header.stamp, &reason)) {
      ROS_ERROR_THROTTLE(
          1.0,
          "Rejecting discontinuous SUPER command: %s", reason.c_str());
      valid_command_ = false;
      resetReacquire();
      latchFault(reason);
      return;
    }
    last_command_time_ = now;
    if (recovery_pending_) {
      ROS_WARN("Verified return trajectory accepted after command fault");
      recovery_pending_ = false;
    }
    valid_command_ = true;
    hold_target_latched_ = false;
    latest_target_ = accepted_output;
    latest_source_trajectory_id_ = command->trajectory_id;
    latest_source_stamp_ = command->header.stamp;
    updateStatus();
  }

  bool publishCheckedTarget(mavros_msgs::PositionTarget target, const ros::Time& now) {
    if (have_published_target_ && px4_mode_ == offboard_mode_) {
      const double dt = (now - published_target_time_).toSec();
      if (dt <= 0.0) return false;
      using mine_uav_control::MotionSample;
      const auto sample=[](const mavros_msgs::PositionTarget& m) {
        MotionSample x; x.p={{m.position.x,m.position.y,m.position.z}};
        x.v={{m.velocity.x,m.velocity.y,m.velocity.z}};
        x.a={{m.acceleration_or_force.x,m.acceleration_or_force.y,m.acceleration_or_force.z}};
        x.yaw=m.yaw; return x;
      };
      const char* reason=mine_uav_control::motionTransitionFailure(
          sample(published_target_),sample(target),dt,max_position_rate_,
          max_velocity_slew_,max_acceleration_slew_,max_yaw_rate_);
      if(reason) {
        const double dp = vectorNorm(
            target.position.x - published_target_.position.x,
            target.position.y - published_target_.position.y,
            target.position.z - published_target_.position.z);
        const double dv = vectorNorm(
            target.velocity.x - published_target_.velocity.x,
            target.velocity.y - published_target_.velocity.y,
            target.velocity.z - published_target_.velocity.z);
        const double rx = target.position.x - published_target_.position.x -
            0.5 * (target.velocity.x + published_target_.velocity.x) * dt;
        const double ry = target.position.y - published_target_.position.y -
            0.5 * (target.velocity.y + published_target_.velocity.y) * dt;
        const double rz = target.position.z - published_target_.position.z -
            0.5 * (target.velocity.z + published_target_.velocity.z) * dt;
        ROS_ERROR_THROTTLE(1.0,
            "OUTPUT_DISCONTINUITY_REJECTED reason=%s dt=%.4f dp=%.4f dv=%.4f pva_residual=%.4f source_age=%.4f",
            reason, dt, dp, dv, vectorNorm(rx, ry, rz),
            (now - last_command_time_).toSec());
        valid_command_=false;
        latchFault("OUTPUT_DISCONTINUITY_REJECTED");
        return false;
      }
    }

    target.header.stamp = now;
    command_publisher_.publish(target);
    published_target_ = target;
    published_target_time_ = now;
    have_published_target_ = true;
    return true;
  }

  mavros_msgs::PositionTarget projectSourceTarget(const ros::Time& now) const {
    auto projected = latest_target_;
    // SUPER samples describe a P/V/A state at their header stamp. The output
    // timer runs independently, so reusing that state at a later wall time
    // would command nonzero velocity with zero displacement. Constant
    // acceleration propagation over the already validated freshness window
    // keeps all three fields at the same instant.
    const double age = std::max(0.0, std::min(command_timeout_,
        (now - latest_source_stamp_).toSec()));
    projected.position.x += projected.velocity.x * age +
        0.5 * projected.acceleration_or_force.x * age * age;
    projected.position.y += projected.velocity.y * age +
        0.5 * projected.acceleration_or_force.y * age * age;
    projected.position.z += projected.velocity.z * age +
        0.5 * projected.acceleration_or_force.z * age * age;
    projected.velocity.x += projected.acceleration_or_force.x * age;
    projected.velocity.y += projected.acceleration_or_force.y * age;
    projected.velocity.z += projected.acceleration_or_force.z * age;
    projected.yaw = normalizeAngle(projected.yaw + projected.yaw_rate * age);
    projected.header.stamp = now;
    return projected;
  }

  mavros_msgs::PositionTarget makeHoldTarget(const ros::Time& now) const {
    mavros_msgs::PositionTarget hold;
    hold.header.stamp = now;
    hold.header.frame_id = "map";
    hold.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    hold.type_mask = mavros_msgs::PositionTarget::IGNORE_VX |
                     mavros_msgs::PositionTarget::IGNORE_VY |
                     mavros_msgs::PositionTarget::IGNORE_VZ |
                     mavros_msgs::PositionTarget::IGNORE_AFX |
                     mavros_msgs::PositionTarget::IGNORE_AFY |
                     mavros_msgs::PositionTarget::IGNORE_AFZ |
                     mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
    hold.position = latest_local_pose_.pose.position;
    hold.yaw = yawFromQuaternion(latest_local_pose_.pose.orientation);
    return hold;
  }

  mavros_msgs::PositionTarget makeFixedHoldTarget(
      const ros::Time& now, bool prefer_last_command = false) {
    if (!hold_target_latched_) {
      latched_hold_target_ = prefer_last_command
          ? makeLastCommandHoldTarget(now)
          : makeHoldTarget(now);
      hold_target_latched_ = true;
    }
    auto hold = latched_hold_target_;
    hold.header.stamp = now;
    return hold;
  }

  void outputTimerCallback(const ros::TimerEvent&) {
    const ros::Time now = ros::Time::now();
    if (recovery_pending_ &&
        (now - recovery_started_).toSec() > return_recovery_timeout_) {
      recovery_pending_ = false;
      latchFault("VERIFIED_RETURN_NOT_AVAILABLE");
    }
    if (returning_home_ && !return_route_wait_since_.isZero() &&
        (now - return_route_wait_since_).toSec() > return_recovery_timeout_) {
      return_route_wait_since_ = ros::Time();
      latchFault("RETURN_ROUTE_UNAVAILABLE");
    }
    // PX4 accepts AUTO.LAND and disarms at home, but can restore the prior
    // OFFBOARD user intention immediately after disarming. The no-RC SITL
    // airframe accepts AUTO.LOITER while disarmed (its startup state); leave
    // the terminal state there rather than reporting an OFFBOARD mode again.
    if (mission_complete_ && !mavros_armed_ &&
        fallback_mode_ == "AUTO.LAND" && px4_mode_ == offboard_mode_) {
      publishReady(false);
      requestMode("AUTO.LOITER", now, false);
      publishStatus("POST_LAND_EXITING_OFFBOARD");
      return;
    }
    if (exit_requested_ ||
        (offboard_owned_ && px4_mode_ == offboard_mode_ && !gatesReady())) {
      handleOffboardExit(now);
      return;
    }
    if (!gatesReady()) {
      hold_target_latched_ = false;
      resetPrestream();
      publishReady(false);
      updateStatus();
      return;
    }
    if (!localPoseFresh(now)) {
      latchFault("LOCAL_POSE_TIMEOUT");
      handleOffboardExit(now);
      return;
    }
    if (!offboard_owned_ &&
        (latest_local_pose_.pose.position.z < min_height_ ||
         latest_local_pose_.pose.position.z > max_height_)) {
      // Reject task activation at the pilot's current height. Clamping the
      // first SUPER target would otherwise initiate an unrequested climb or
      // descent before the route has even been accepted.
      latchFault("START_HEIGHT_OUTSIDE_FENCE");
      return;
    }

    const bool command_fresh =
        valid_command_ && !last_command_time_.isZero() &&
        (now - last_command_time_).toSec() <= command_timeout_;
    if (prestream_started_.isZero()) {
      prestream_started_ = now;
    }

    if (px4_mode_ != offboard_mode_) {
      hold_target_latched_ = false;
      publishCheckedTarget(makeFixedHoldTarget(now, true), now);
      publishReady(false);
      if (require_armed_for_offboard_ && !mavros_armed_) {
        publishStatus("PRESTREAM_WAIT_ARMED");
      } else if (!command_fresh) {
        publishStatus("PRESTREAM_WAIT_SUPER_COMMAND");
      } else if ((now - prestream_started_).toSec() < prestream_duration_) {
        publishStatus("PRESTREAM_HOLD");
      } else if (!automatic_mode_switch_) {
        publishStatus("PRESTREAM_WAIT_MANUAL_OFFBOARD");
      } else {
        requestMode(offboard_mode_, now, true);
        publishStatus("REQUESTING_OFFBOARD");
      }
      return;
    }

    offboard_owned_ = offboard_owned_ || automatic_mode_switch_;
    if (command_fresh) {
      hold_target_latched_ = false;
      if (brake_active_) {
        brake_active_ = false;
        source_stream_active_ = false;
      }
      if (!source_stream_active_) {
        if (have_published_target_) {
          startReacquire(published_target_, projectSourceTarget(now), now);
        }
        source_stream_active_ = true;
      }
      const auto desired = projectSourceTarget(now);
      if (!reacquire_active_ && have_published_target_ &&
          latest_source_trajectory_id_ != published_source_trajectory_id_) {
        const double dt = (now - published_target_time_).toSec();
        if (dt > 0.0 && dt <= 0.25) {
          const auto sample=[](const mavros_msgs::PositionTarget& m) {
            mine_uav_control::MotionSample x;
            x.p={{m.position.x,m.position.y,m.position.z}};
            x.v={{m.velocity.x,m.velocity.y,m.velocity.z}};
            x.a={{m.acceleration_or_force.x,m.acceleration_or_force.y,
                  m.acceleration_or_force.z}};
            x.yaw=m.yaw;
            return x;
          };
          const char* discontinuity=mine_uav_control::motionTransitionFailure(
              sample(published_target_),sample(desired),dt,max_position_rate_,
              max_velocity_slew_,max_acceleration_slew_,max_yaw_rate_);
          const double offset=vectorNorm(
              desired.position.x-published_target_.position.x,
              desired.position.y-published_target_.position.y,
              desired.position.z-published_target_.position.z);
          if (discontinuity && offset <= 0.09 &&
              std::abs(normalizeAngle(desired.yaw-published_target_.yaw)) <= 0.25) {
            // Replans and post-brake rest plans can differ by a few centimetres
            // from the last actually transmitted state. Splice the verified
            // new source with a derivative-consistent, bounded offset.
            startReacquire(published_target_,desired,now);
          }
        }
      }
      auto output_target = blendReacquire(desired, now);
      if (!publishCheckedTarget(output_target, now)) return;
      published_source_trajectory_id_=latest_source_trajectory_id_;
      if (hold_requested_ && outputNearRest(output_target)) {
        activateExplorationHold(now);
      }
      publishReady(true);
      publishStatus(reacquire_active_ ? "SMOOTH_REACQUIRE" : "STREAMING");
    } else {
      valid_command_ = false;
      resetReacquire();
      if (brake_active_ || (have_published_target_ &&
                            !outputNearRest(published_target_))) {
        startEmergencyBrake();
        if (!publishCheckedTarget(emergencyBrakeTarget(now), now)) return;
        if ((now - brake_start_time_).toSec() >= brake_duration_)
          brake_active_ = false;
        publishReady(false);
        publishStatus("BRAKING_COMMAND_TIMEOUT");
        return;
      }
      if (!publishCheckedTarget(makeFixedHoldTarget(now, true), now)) return;
      publishReady(true);
      publishStatus("HOLD_COMMAND_TIMEOUT");
    }
  }

  void latchFault(const std::string& reason) {
    const bool returnable_command_fault =
        reason == "HEIGHT_GEOFENCE" || reason == "HORIZONTAL_GEOFENCE" ||
        reason == "SPEED_LIMIT" || reason == "ACCELERATION_LIMIT" ||
        reason == "COMMAND_STALE_OR_FUTURE_STAMP" ||
        reason == "OUTPUT_DISCONTINUITY_REJECTED";
    if (returnable_command_fault && !recovery_attempted_ &&
        auto_enabled_ && mission_enabled_ && offboard_owned_ &&
        px4_mode_ == offboard_mode_ && alignment_ready_ && vision_healthy_ &&
        localPoseFresh(ros::Time::now())) {
      recovery_attempted_ = true;
      recovery_pending_ = true;
      recovery_started_ = ros::Time::now();
      recovery_source_trajectory_id_ = latest_source_trajectory_id_;
      valid_command_ = false;
      resetReacquire();
      resetContinuityGuard();
      publishReturnRequest(true);
      publishReady(false);
      publishStatus("REQUESTING_VERIFIED_RETURN: " + reason);
      ROS_WARN("Command fault %s: braking and requesting verified outbound retrace",
               reason.c_str());
      return;
    }
    if (auto_enabled_ && !fault_latched_) {
      ROS_ERROR("Task-one automatic control fault latched: %s", reason.c_str());
      fault_latched_ = true;
      beginOffboardExit(reason);
    }
    publishReady(false);
    publishStatus(reason);
  }

  void publishReturnRequest(bool requested) {
    std_msgs::Bool message;
    message.data = requested;
    return_request_publisher_.publish(message);
  }

  void beginOffboardExit(const std::string& reason) {
    hold_target_latched_ = false;
    resetReacquire();
    if (!exit_requested_ && offboard_owned_ && px4_mode_ == offboard_mode_) {
      exit_requested_ = true;
      ROS_WARN("Leaving managed OFFBOARD: %s", reason.c_str());
    }
  }

  void handleOffboardExit(const ros::Time& now) {
    publishReady(false);
    if (px4_mode_ != offboard_mode_) {
      exit_requested_ = false;
      offboard_owned_ = false;
      updateStatus();
      return;
    }
    if (brake_active_ || (have_published_target_ &&
                          !outputNearRest(published_target_))) {
      startEmergencyBrake();
      if (publishCheckedTarget(emergencyBrakeTarget(now), now)) {
        if ((now - brake_start_time_).toSec() >= brake_duration_)
          brake_active_ = false;
        publishStatus("BRAKING_BEFORE_OFFBOARD_EXIT");
      }
      return;
    }
    if (localPoseFresh(now))
      publishCheckedTarget(makeFixedHoldTarget(now, true), now);
    requestMode(fallback_mode_, now, false);
    publishStatus("EXITING_OFFBOARD_TO_" + fallback_mode_);
  }

  void requestMode(const std::string& mode, const ros::Time& now,
                   bool entering_offboard) {
    if (!last_mode_request_time_.isZero() &&
        (now - last_mode_request_time_).toSec() < mode_request_interval_) {
      return;
    }
    last_mode_request_time_ = now;
    mavros_msgs::SetMode request;
    request.request.base_mode = 0;
    request.request.custom_mode = mode;
    if (!set_mode_client_.call(request) || !request.response.mode_sent) {
      ROS_WARN_THROTTLE(1.0, "PX4 rejected or did not answer mode request: %s",
                        mode.c_str());
      return;
    }
    if (entering_offboard) {
      offboard_owned_ = true;
    }
    ROS_INFO("PX4 mode request acknowledged for %s; awaiting /mavros/state confirmation",
             mode.c_str());
  }

  void resetPrestream() { prestream_started_ = ros::Time(); }

  void updateStatus() {
    const bool ready = gatesReady() && valid_command_ &&
                       px4_mode_ == offboard_mode_;
    publishReady(ready);
    if (!alignment_ready_) {
      publishStatus("WAIT_ALIGNMENT");
    } else if (!software_enabled_) {
      publishStatus("SOFTWARE_INHIBIT");
    } else if (!auto_enabled_) {
      publishStatus("WAIT_AUTO_ENABLE");
    } else if (!mission_enabled_) {
      publishStatus("WAIT_TASK1_SELECTION");
    } else if (mission_complete_) {
      publishStatus("TASK1_COMPLETE");
    } else if (!vision_healthy_) {
      publishStatus("WAIT_VISION");
    } else if (!mavros_connected_) {
      publishStatus("WAIT_MAVROS");
    } else if (!localPoseFresh(ros::Time::now())) {
      publishStatus("WAIT_LOCAL_POSE");
    } else if (fault_latched_) {
      publishStatus("FAULT_LATCHED_TOGGLE_AUTO_LOW");
    } else if (require_armed_for_offboard_ && !mavros_armed_) {
      publishStatus("PRESTREAM_WAIT_ARMED");
    } else if (px4_mode_ != offboard_mode_) {
      publishStatus("PRESTREAM");
    } else if (!valid_command_) {
      publishStatus("WAIT_SUPER_COMMAND");
    } else {
      publishStatus("READY");
    }
  }

  void publishReady(bool ready) {
    if (ready_published_ && ready == last_ready_) {
      return;
    }
    std_msgs::Bool message;
    message.data = ready;
    ready_publisher_.publish(message);
    last_ready_ = ready;
    ready_published_ = true;
  }

  void publishStatus(const std::string& status) {
    if (status == last_status_) {
      return;
    }
    std_msgs::String message;
    message.data = status;
    status_publisher_.publish(message);
    last_status_ = status;
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber command_subscriber_;
  ros::Subscriber alignment_subscriber_;
  ros::Subscriber mission_subscriber_;
  ros::Subscriber auto_enable_subscriber_;
  ros::Subscriber vision_subscriber_;
  ros::Subscriber finished_subscriber_;
  ros::Subscriber returning_subscriber_;
  ros::Subscriber exploration_status_subscriber_;
  ros::Subscriber state_subscriber_;
  ros::Subscriber local_pose_subscriber_;
  ros::Publisher command_publisher_;
  ros::Publisher ready_publisher_;
  ros::Publisher status_publisher_;
  ros::Publisher return_request_publisher_;
  ros::ServiceServer enable_service_;
  ros::ServiceClient set_mode_client_;
  ros::Timer output_timer_;

  geometry_msgs::PoseStamped latest_local_pose_;
  mavros_msgs::PositionTarget latest_target_;
  mavros_msgs::PositionTarget latched_hold_target_;
  mavros_msgs::PositionTarget previous_target_;
  mavros_msgs::PositionTarget reacquire_start_target_;
  std::array<mine_uav_control::QuinticOffset1D, 3> reacquire_xyz_;
  mine_uav_control::QuinticOffset1D reacquire_yaw_;
  mavros_msgs::PositionTarget brake_start_target_;
  std::array<mine_uav_control::QuinticOffset1D, 3> brake_xyz_;
  mine_uav_control::QuinticOffset1D brake_yaw_;
  std::array<double, 3> brake_stop_position_{{0.0, 0.0, 0.0}};
  double brake_stop_yaw_{0.0};
  double brake_duration_{1.5};
  ros::Time brake_start_time_;
  bool brake_active_{false};
  std::string command_topic_;
  std::string output_topic_;
  std::string expected_frame_;
  std::string auto_enable_topic_;
  std::string exploration_status_topic_;
  std::string offboard_mode_{"OFFBOARD"};
  std::string fallback_mode_{"POSCTL"};
  double return_recovery_timeout_{12.0};
  bool returning_home_{false};
  bool recovery_pending_{false};
  bool recovery_attempted_{false};
  uint32_t recovery_source_trajectory_id_{0};
  ros::Time recovery_started_;
  ros::Time return_route_wait_since_;
  std::string px4_mode_;
  std::string last_status_;
  ros::Time last_command_time_;
  ros::Time latest_source_stamp_;
  uint32_t latest_source_trajectory_id_{0};
  uint32_t published_source_trajectory_id_{0};
  ros::Time last_local_pose_time_;
  ros::Time prestream_started_;
  ros::Time last_mode_request_time_;
  ros::Time previous_target_time_;
  ros::Time reacquire_start_time_;
  double command_timeout_{0.25};
  double local_pose_timeout_{0.5};
  double output_rate_{50.0};
  double max_speed_{1.0};
  double max_acceleration_{3.0};
  double max_yaw_rate_{0.75};
  double max_position_rate_{1.10};
  double max_velocity_slew_{1.75};
  double max_acceleration_slew_{6.0};
  double continuity_reset_gap_{0.30};
  double continuity_nominal_dt_{0.01};
  double reacquire_blend_duration_{3.5};
  double max_initial_command_distance_{0.65};
  double max_initial_yaw_error_{0.70};
  double max_horizontal_radius_{40.0};
  double min_height_{-0.5};
  double max_height_{1.8};
  double height_clamp_tolerance_{0.25};
  double max_alignment_translation_change_{0.05};
  double max_alignment_yaw_change_{0.05};
  double alignment_x_{0.0};
  double alignment_y_{0.0};
  double alignment_z_{0.0};
  double alignment_yaw_{0.0};
  double prestream_duration_{1.0};
  double mode_request_interval_{1.0};
  bool use_acceleration_{true};
  bool automatic_mode_switch_{true};
  bool require_armed_for_offboard_{true};
  bool alignment_ready_{false};
  bool mission_enabled_{false};
  bool auto_enabled_{false};
  bool vision_healthy_{false};
  bool mission_complete_{false};
  bool exploration_status_hold_{true};
  bool hold_requested_{true};
  bool mavros_connected_{false};
  bool mavros_armed_{false};
  bool software_enabled_{true};
  bool fault_latched_{false};
  bool offboard_owned_{false};
  bool exit_requested_{false};
  bool valid_command_{false};
  bool have_local_pose_{false};
  mavros_msgs::PositionTarget published_target_;
  ros::Time published_target_time_;
  bool have_published_target_{false};
  bool have_previous_target_{false};
  bool hold_target_latched_{false};
  bool reacquire_active_{false};
  bool source_stream_active_{false};
  bool ready_published_{false};
  bool last_ready_{false};
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "super_px4_command_bridge");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  SuperPx4CommandBridge bridge(nh, private_nh);
  ros::spin();
  return 0;
}
