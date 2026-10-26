#include "mine_uav_control/motion_continuity.hpp"
#include "mine_uav_control/lio_mount_alignment.hpp"
#include "mine_uav_control/obstacle_safety.hpp"
#include "mine_uav_control/tracking_speed_governor.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <map>
#include <stdexcept>
#include <string>
#include <tuple>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <std_srvs/SetBool.h>
#include <vector>

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

mine_uav_control::Quaternion toMathQuaternion(
    const geometry_msgs::Quaternion& q) {
  return mine_uav_control::normalized(
      mine_uav_control::Quaternion(q.x, q.y, q.z, q.w));
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
    private_nh_.param(
        "alignment_topic", alignment_topic_,
        std::string("/mine_uav/task1/planning_to_px4_alignment"));
    private_nh_.param(
        "sensor_position_topic", sensor_position_topic_,
        std::string("/mine_uav/task1/lio_sensor_position_in_body"));
    private_nh_.param("expected_frame", expected_frame_,
                      std::string("task1_planning"));
    private_nh_.param("command_timeout", command_timeout_, 0.25);
    private_nh_.param("local_pose_timeout", local_pose_timeout_, 0.5);
    private_nh_.param("output_rate", output_rate_, 50.0);
    private_nh_.param("max_speed", max_speed_, 1.0);
    private_nh_.param("max_measured_speed", max_measured_speed_, max_speed_);
    max_predicted_speed_ = max_measured_speed_;
    private_nh_.param("max_predicted_speed", max_predicted_speed_,
                      max_predicted_speed_);
    private_nh_.param("measured_speed_target",
                      measured_speed_target_, 0.65);
    private_nh_.param("measured_speed_feedback_gain",
                      measured_speed_feedback_gain_, 1.0);
    private_nh_.param("measured_speed_governor_floor",
                      measured_speed_governor_floor_, 0.35);
    private_nh_.param("measured_speed_governor_reduction_rate",
                      measured_speed_governor_reduction_rate_, 0.80);
    private_nh_.param("measured_speed_governor_recovery_rate",
                      measured_speed_governor_recovery_rate_, 0.50);
    private_nh_.param("tracking_position_gain_xy",
                      tracking_position_gain_xy_, 0.9);
    private_nh_.param("tracking_position_gain_z",
                      tracking_position_gain_z_, 1.0);
    private_nh_.param("max_governor_tracking_error",
                      max_governor_tracking_error_, 0.65);
    private_nh_.param("measured_speed_timeout", measured_speed_timeout_, 0.5);
    private_nh_.param("overspeed_duration", overspeed_duration_, 1.5);
    private_nh_.param("max_offboard_entry_speed",
                      max_offboard_entry_speed_, 0.20);
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
    private_nh_.param("max_managed_reacquire_distance",
                      max_managed_reacquire_distance_, 1.5);
    private_nh_.param("max_managed_reacquire_yaw_error",
                      max_managed_reacquire_yaw_error_, 2.6);
    private_nh_.param("max_horizontal_radius", max_horizontal_radius_, 0.0);
    private_nh_.param("min_height", min_height_, -5.0);
    private_nh_.param("max_height", max_height_, 8.0);
    private_nh_.param("height_clamp_tolerance", height_clamp_tolerance_, 0.25);
    private_nh_.param("max_alignment_translation_change",
                      max_alignment_translation_change_, 0.05);
    if (!private_nh_.getParam("max_alignment_rotation_change",
                              max_alignment_rotation_change_)) {
      // Accept the old yaw-only parameter name for existing launch files.
      private_nh_.param("max_alignment_yaw_change",
                        max_alignment_rotation_change_, 0.05);
    }
    private_nh_.param("use_acceleration", use_acceleration_, true);
    private_nh_.param("auto_enable_topic", auto_enable_topic_,
                      std::string("/mine_uav/mission/auto_enable"));
    private_nh_.param("mission_enable_topic", mission_enable_topic_,
                      std::string("/mine_uav/mission/goaf_enable"));
    private_nh_.param("exploration_status_topic", exploration_status_topic_,
                      std::string("/mine_uav/exploration/status"));
    private_nh_.param("execution_authorization_topic",
                      execution_authorization_topic_,
                      std::string("/mine_uav/super/execution_enable"));
    private_nh_.param("automatic_mode_switch", automatic_mode_switch_, true);
    private_nh_.param("require_armed_for_offboard",
                      require_armed_for_offboard_, true);
    private_nh_.param("prestream_duration", prestream_duration_, 1.0);
    private_nh_.param("prestream_stability_timeout",
                      prestream_stability_timeout_, 15.0);
    private_nh_.param("preoffboard_vision_recovery_timeout",
                      preoffboard_vision_recovery_timeout_, 2.0);
    private_nh_.param("mode_request_interval", mode_request_interval_, 1.0);
    private_nh_.param("offboard_mode", offboard_mode_,
                      std::string("OFFBOARD"));
    private_nh_.param("fallback_mode", fallback_mode_,
                      std::string("POSCTL"));
    private_nh_.param("return_recovery_timeout", return_recovery_timeout_,
                      12.0);
    private_nh_.param("software_enable_default", software_enabled_, true);
    private_nh_.param("obstacle_safety_enabled", obstacle_safety_enabled_,
                      false);
    private_nh_.param("obstacle_cloud_topic", obstacle_cloud_topic_,
                      std::string("/mine_uav/task1/cloud_registered"));
    private_nh_.param("task_odometry_topic", task_odometry_topic_,
                      std::string("/mine_uav/task1/odometry"));
    private_nh_.param("obstacle_cloud_timeout", obstacle_cloud_timeout_, 0.35);
    private_nh_.param("obstacle_cloud_fault_timeout",
                      obstacle_cloud_fault_timeout_, 2.0);
    private_nh_.param("live_horizontal_clearance",
                      live_horizontal_clearance_, 0.65);
    private_nh_.param("obstacle_monitor_horizontal_clearance",
                      obstacle_monitor_horizontal_clearance_, 0.8);
    private_nh_.param("obstacle_persistence_duration",
                      obstacle_persistence_duration_, 3.0);
    private_nh_.param("obstacle_replan_topic", obstacle_replan_topic_,
                      std::string("/mine_uav/exploration/obstacle_replan"));
    private_nh_.param("obstacle_replan_escape_duration",
                      obstacle_replan_escape_duration_, 4.0);
    private_nh_.param("obstacle_clear_confirm_frames",
                      obstacle_clear_confirm_frames_, 3);
    double legacy_live_vertical_half_height = 0.3;
    private_nh_.param("live_vertical_half_height",
                      legacy_live_vertical_half_height, 0.3);
    private_nh_.param("live_vertical_clearance_below",
                      live_vertical_clearance_below_, 0.5);
    private_nh_.param("live_vertical_clearance_above",
                      live_vertical_clearance_above_,
                      0.5);
    private_nh_.param("obstacle_min_z", obstacle_min_z_, -1.0e6);
    private_nh_.param("coverage_guard_distance",
                      coverage_guard_distance_, 2.5);
    private_nh_.param("coverage_lateral_half_width",
                      coverage_lateral_half_width_, 1.0);
    private_nh_.param("coverage_vertical_uncertainty_margin",
                      coverage_vertical_uncertainty_margin_, 0.2);
    private_nh_.param("coverage_min_points", coverage_min_points_, 30);
    private_nh_.param("coverage_min_horizontal_span",
                      coverage_min_horizontal_span_, 0.6);
    private_nh_.param("coverage_min_vertical_span",
                      coverage_min_vertical_span_, 0.0);
    private_nh_.param("coverage_confirm_frames",
                      coverage_confirm_frames_, 3);
    private_nh_.param("coverage_reject_frames",
                      coverage_reject_frames_, 2);
    private_nh_.param("max_obstacle_points", max_obstacle_points_, 20000);

    command_timeout_ = std::max(0.05, command_timeout_);
    local_pose_timeout_ = std::max(0.1, local_pose_timeout_);
    output_rate_ = std::max(10.0, output_rate_);
    max_speed_ = std::max(0.1, max_speed_);
    max_measured_speed_ = std::max(0.1, max_measured_speed_);
    max_predicted_speed_ = std::max(
        0.1, std::min(max_predicted_speed_, max_measured_speed_));
    measured_speed_target_ = std::max(
        0.1, std::min(measured_speed_target_, max_predicted_speed_));
    measured_speed_feedback_gain_ = std::max(
        0.1, measured_speed_feedback_gain_);
    measured_speed_governor_floor_ = std::max(
        0.05, std::min(measured_speed_governor_floor_,
                       measured_speed_target_ - 0.02));
    measured_speed_governor_reduction_rate_ = std::max(
        0.1, std::min(measured_speed_governor_reduction_rate_,
                      max_velocity_slew_));
    measured_speed_governor_recovery_rate_ = std::max(
        0.05, std::min(measured_speed_governor_recovery_rate_,
                       max_velocity_slew_));
    adaptive_predicted_speed_limit_ = max_predicted_speed_;
    tracking_position_gain_xy_ =
        std::max(0.0, tracking_position_gain_xy_);
    tracking_position_gain_z_ =
        std::max(0.0, tracking_position_gain_z_);
    max_governor_tracking_error_ =
        std::max(0.1, max_governor_tracking_error_);
    measured_speed_timeout_ = std::max(0.1, measured_speed_timeout_);
    overspeed_duration_ = std::max(0.1, overspeed_duration_);
    max_offboard_entry_speed_ = std::max(
        0.02, std::min(max_offboard_entry_speed_, max_measured_speed_));
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
    max_managed_reacquire_distance_ = std::max(
        max_initial_command_distance_, max_managed_reacquire_distance_);
    max_managed_reacquire_yaw_error_ = std::max(
        max_initial_yaw_error_,
        std::min(kPi, max_managed_reacquire_yaw_error_));
    // Zero disables the optional scenario fence. Height, obstacle, motion and
    // localization checks remain mandatory and independent of mission range.
    max_horizontal_radius_ = std::max(0.0, max_horizontal_radius_);
    max_alignment_translation_change_ =
        std::max(0.001, max_alignment_translation_change_);
    max_alignment_rotation_change_ =
        std::max(0.001, max_alignment_rotation_change_);
    prestream_duration_ = std::max(1.0, prestream_duration_);
    prestream_stability_timeout_ = std::max(
        prestream_duration_ + 0.5, prestream_stability_timeout_);
    preoffboard_vision_recovery_timeout_ = std::max(
        0.2, preoffboard_vision_recovery_timeout_);
    mode_request_interval_ = std::max(0.5, mode_request_interval_);
    return_recovery_timeout_ = std::max(2.0, return_recovery_timeout_);
    obstacle_cloud_timeout_ = std::max(0.1, obstacle_cloud_timeout_);
    obstacle_cloud_fault_timeout_ = std::max(
        obstacle_cloud_timeout_ + 0.1, obstacle_cloud_fault_timeout_);
    live_horizontal_clearance_ = std::max(0.1, live_horizontal_clearance_);
    obstacle_monitor_horizontal_clearance_ = std::max(
        live_horizontal_clearance_, obstacle_monitor_horizontal_clearance_);
    obstacle_persistence_duration_ = std::max(
        0.1, obstacle_persistence_duration_);
    obstacle_replan_escape_duration_ = std::max(
        1.0, obstacle_replan_escape_duration_);
    obstacle_clear_confirm_frames_ = std::max(
        1, obstacle_clear_confirm_frames_);
    live_vertical_clearance_below_ =
        std::max(0.1, live_vertical_clearance_below_);
    live_vertical_clearance_above_ =
        std::max(0.1, live_vertical_clearance_above_);
    coverage_guard_distance_ = std::max(live_horizontal_clearance_,
                                        coverage_guard_distance_);
    coverage_lateral_half_width_ = std::max(
        live_horizontal_clearance_, coverage_lateral_half_width_);
    coverage_vertical_uncertainty_margin_ = std::max(
        0.0, std::min(0.5, coverage_vertical_uncertainty_margin_));
    coverage_min_points_ = std::max(1, coverage_min_points_);
    coverage_min_horizontal_span_ = std::max(
        0.1, coverage_min_horizontal_span_);
    coverage_min_vertical_span_ = std::max(0.0, coverage_min_vertical_span_);
    coverage_confirm_frames_ = std::max(1, coverage_confirm_frames_);
    coverage_reject_frames_ = std::max(1, coverage_reject_frames_);
    max_obstacle_points_ = std::max(100, max_obstacle_points_);
    obstacle_monitor_.configure(obstacle_persistence_duration_,
                                obstacle_clear_confirm_frames_);
    if (min_height_ > max_height_) {
      std::swap(min_height_, max_height_);
    }

    command_subscriber_ = nh_.subscribe(
        command_topic_, 10, &SuperPx4CommandBridge::commandCallback, this);
    alignment_subscriber_ = nh_.subscribe(
        alignment_topic_, 1,
        &SuperPx4CommandBridge::alignmentCallback, this);
    sensor_position_subscriber_ = nh_.subscribe(
        sensor_position_topic_, 1,
        &SuperPx4CommandBridge::sensorPositionCallback, this);
    mission_subscriber_ = nh_.subscribe(
        mission_enable_topic_, 1,
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
    local_velocity_subscriber_ = nh_.subscribe(
        "/mavros/local_position/velocity_local", 20,
        &SuperPx4CommandBridge::localVelocityCallback, this);
    task_odometry_subscriber_ = nh_.subscribe(
        task_odometry_topic_, 20,
        &SuperPx4CommandBridge::taskOdometryCallback, this);
    obstacle_cloud_subscriber_ = nh_.subscribe(
        obstacle_cloud_topic_, 2,
        &SuperPx4CommandBridge::obstacleCloudCallback, this);

    command_publisher_ =
        nh_.advertise<mavros_msgs::PositionTarget>(output_topic_, 10);
    ready_publisher_ = nh_.advertise<std_msgs::Bool>(
        "/mine_uav/task1/command_ready", 1, true);
    status_publisher_ = nh_.advertise<std_msgs::String>(
        "/mine_uav/task1/command_status", 1, true);
    fault_publisher_ = nh_.advertise<std_msgs::String>(
        "/mine_uav/task1/command_fault", 1, true);
    return_request_publisher_ = nh_.advertise<std_msgs::Bool>(
        "/mine_uav/exploration/return_home", 1, true);
    obstacle_replan_publisher_ = nh_.advertise<std_msgs::Empty>(
        obstacle_replan_topic_, 1, false);
    execution_authorization_publisher_ = nh_.advertise<std_msgs::Bool>(
        execution_authorization_topic_, 1, true);
    enable_service_ = private_nh_.advertiseService(
        "enable", &SuperPx4CommandBridge::enableCallback, this);
    set_mode_client_ =
        nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    output_timer_ = nh_.createTimer(
        ros::Duration(1.0 / output_rate_),
        &SuperPx4CommandBridge::outputTimerCallback, this);

    publishReady(false);
    publishExecutionAuthorization();
    publishReturnRequest(false);
    publishFault("");
    publishStatus("WAIT_ALIGNMENT");
    ROS_WARN("SUPER-to-PX4 bridge ready: RC auto-enable and task selection are "
             "required; the bridge may request OFFBOARD but never arms PX4");
  }

 private:
  void sensorPositionCallback(
      const geometry_msgs::Vector3Stamped::ConstPtr& position) {
    const auto& p = position->vector;
    const double norm = vectorNorm(p.x, p.y, p.z);
    if (!finite(p.x) || !finite(p.y) || !finite(p.z) || norm > 1.0) {
      latchFault("INVALID_SENSOR_LEVER_ARM");
      return;
    }
    const mine_uav_control::Vector3 next(p.x, p.y, p.z);
    const bool changed = have_sensor_position_ &&
        vectorNorm(next.x - sensor_position_in_body_.x,
                   next.y - sensor_position_in_body_.y,
                   next.z - sensor_position_in_body_.z) > 0.01;
    if (changed && auto_enabled_ && mission_enabled_) {
      valid_command_ = false;
      latchFault("SENSOR_LEVER_ARM_CHANGED_DURING_TASK");
      resetPrestream();
      return;
    }
    sensor_position_in_body_ = next;
    have_sensor_position_ = true;
  }

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
    const auto new_rotation = toMathQuaternion(q);
    const bool alignment_changed = alignment_ready_ &&
        (vectorNorm(t.x - alignment_x_, t.y - alignment_y_,
                    t.z - alignment_z_) > max_alignment_translation_change_ ||
         mine_uav_control::quaternionAngularDistance(
             new_rotation, alignment_rotation_) >
             max_alignment_rotation_change_);
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
    alignment_rotation_ = new_rotation;
    alignment_ready_ = true;
    updateStatus();
  }

  void missionCallback(const std_msgs::Bool::ConstPtr& enabled) {
    const bool was_enabled = mission_enabled_;
    mission_enabled_ = enabled->data;
    if (!was_enabled && mission_enabled_) {
      // A new task selection is a new command epoch. Never let a terminal
      // sample retained from the previous goal qualify the next OFFBOARD
      // prestream as fresh.
      valid_command_ = false;
      last_command_time_ = ros::Time();
      latest_source_stamp_ = ros::Time();
      resetPrestream();
      resetContinuityGuard();
      resetReacquire();
      managed_hold_reacquire_pending_ = false;
      hold_target_latched_ = false;
      obstacle_coverage_complete_streak_ = 0;
      obstacle_coverage_incomplete_streak_ = 0;
      obstacle_surface_coverage_ready_ = true;
      obstacle_safety_ready_ = !obstacle_safety_enabled_;
      obstacle_hazard_stop_active_ = false;
      resetTransientObstacleMonitor();
      if (!vision_healthy_) {
        preoffboard_vision_unhealthy_since_ = ros::Time::now();
      }
    }
    if (!mission_enabled_) {
      overspeed_since_ = ros::Time();
      recovery_pending_ = false;
      recovery_attempted_ = false;
      returning_home_ = false;
      return_route_wait_since_ = ros::Time();
      publishReturnRequest(false);
      beginOffboardExit("TASK1_DESELECTED");
      valid_command_ = false;
      last_command_time_ = ros::Time();
      latest_source_stamp_ = ros::Time();
      resetPrestream();
      resetContinuityGuard();
      resetReacquire();
      managed_hold_reacquire_pending_ = false;
      publishFault("");
      obstacle_hazard_stop_active_ = false;
      resetTransientObstacleMonitor();
      preoffboard_vision_unhealthy_since_ = ros::Time();
    }
    updateStatus();
  }

  void autoEnableCallback(const std_msgs::Bool::ConstPtr& enabled) {
    const bool was_enabled = auto_enabled_;
    auto_enabled_ = enabled->data;
    if (!was_enabled && auto_enabled_) {
      // Goal acceptance is the start of a fresh control epoch. Discard any
      // planner sample emitted during map warmup or before the new goal was
      // accepted; OFFBOARD must wait for a post-acceptance command.
      valid_command_ = false;
      last_command_time_ = ros::Time();
      latest_source_stamp_ = ros::Time();
      resetPrestream();
      resetContinuityGuard();
      resetReacquire();
      managed_hold_reacquire_pending_ = false;
      overspeed_since_ = ros::Time();
      obstacle_coverage_complete_streak_ = 0;
      obstacle_coverage_incomplete_streak_ = 0;
      obstacle_surface_coverage_ready_ = true;
      obstacle_safety_ready_ = !obstacle_safety_enabled_;
      obstacle_hazard_stop_active_ = false;
      resetTransientObstacleMonitor();
      if (!vision_healthy_) {
        preoffboard_vision_unhealthy_since_ = ros::Time::now();
      }
    }
    if (was_enabled && !auto_enabled_) {
      overspeed_since_ = ros::Time();
      recovery_pending_ = false;
      recovery_attempted_ = false;
      publishReturnRequest(false);
      beginOffboardExit("AUTO_DISABLED");
      fault_latched_ = false;
      resetPrestream();
      resetContinuityGuard();
      resetReacquire();
      managed_hold_reacquire_pending_ = false;
      publishFault("");
      obstacle_hazard_stop_active_ = false;
      resetTransientObstacleMonitor();
      preoffboard_vision_unhealthy_since_ = ros::Time();
    }
    updateStatus();
  }

  void visionCallback(const std_msgs::Bool::ConstPtr& healthy) {
    const bool was_healthy = vision_healthy_;
    vision_healthy_ = healthy->data;
    if (!vision_healthy_ && auto_enabled_ && mission_enabled_) {
      if (offboard_owned_ && px4_mode_ == offboard_mode_) {
        if (!preoffboard_vision_unhealthy_since_.isZero()) {
          // PX4 confirmed an earlier OFFBOARD request after the vision gate
          // had already fallen in POSCTL. SUPER was never authorized, so this
          // is an entry cancellation rather than an in-flight localization
          // failure. Keep the frozen prestream hold and return to POSCTL.
          beginOffboardExit(
              "LATE_OFFBOARD_CONFIRMATION_DURING_VISION_RECOVERY");
        } else {
          // Once PX4 is genuinely executing the trajectory, vision loss
          // remains an immediate hard fault.
          latchFault("VISION_UNHEALTHY");
        }
      } else {
        // Before authority transfer PX4 is still in pilot-controlled POSCTL.
        // Restart prestream on a delayed estimator sample and give the
        // estimator a bounded opportunity to recover.
        if (preoffboard_vision_unhealthy_since_.isZero()) {
          preoffboard_vision_unhealthy_since_ = ros::Time::now();
        }
        resetPrestream();
      }
    } else if (vision_healthy_ && !was_healthy) {
      preoffboard_vision_unhealthy_since_ = ros::Time();
      resetPrestream();
      if (px4_mode_ != offboard_mode_) {
        // The hold used to cover an outstanding, asynchronously confirmed
        // OFFBOARD request must not become the next entry hold after recovery.
        // Capture the aircraft's then-current POSCTL pose and run a complete
        // prestream interval again.
        hold_target_latched_ = false;
      }
    }
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
      managed_hold_reacquire_pending_ = false;
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
        state == "WAIT_HARD_SAFE_ROUTE" ||
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
      managed_hold_reacquire_pending_ = false;
      if (previous_mode == offboard_mode_) {
        hold_target_latched_ = false;
      }
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

  void taskOdometryCallback(const nav_msgs::Odometry::ConstPtr& odometry) {
    const auto& p = odometry->pose.pose.position;
    if (!finite(p.x) || !finite(p.y) || !finite(p.z)) {
      have_task_odometry_ = false;
      return;
    }
    latest_task_position_ = {p.x, p.y, p.z};
    last_task_odometry_time_ = ros::Time::now();
    have_task_odometry_ = true;
  }

  bool obstacleCloudFresh(const ros::Time& now) const {
    return !obstacle_safety_enabled_ ||
        (have_obstacle_cloud_ && have_task_odometry_ &&
         !last_obstacle_cloud_time_.isZero() &&
         !last_task_odometry_time_.isZero() &&
         (now - last_obstacle_cloud_time_).toSec() <=
             obstacle_cloud_timeout_ &&
         (now - last_task_odometry_time_).toSec() <=
             obstacle_cloud_timeout_);
  }

  bool obstacleCloudFaulted(const ros::Time& now) const {
    if (!obstacle_safety_enabled_) return false;
    if (!have_obstacle_cloud_ || !have_task_odometry_ ||
        last_obstacle_cloud_time_.isZero() ||
        last_task_odometry_time_.isZero()) {
      return true;
    }
    return (now - last_obstacle_cloud_time_).toSec() >
               obstacle_cloud_fault_timeout_ ||
           (now - last_task_odometry_time_).toSec() >
               obstacle_cloud_fault_timeout_;
  }

  bool autonomousReturnHealthReady(const ros::Time& now) const {
    return alignment_ready_ && vision_healthy_ && mavros_connected_ &&
        localPoseFresh(now) && localVelocityFresh(now) &&
        obstacleCloudFresh(now);
  }

  void triggerObstacleHazard(const std::string& reason) {
    if (obstacle_hazard_stop_active_) return;
    obstacle_hazard_stop_active_ = true;
    managed_hold_reacquire_pending_ = false;
    valid_command_ = false;
    resetReacquire();
    resetContinuityGuard();
    latchFault(reason);
  }

  void resetTransientObstacleMonitor() {
    obstacle_monitor_.reset();
    obstacle_monitor_reason_.clear();
    obstacle_replan_requested_ = false;
    obstacle_replan_escape_active_ = false;
    obstacle_replan_escape_started_ = ros::Time();
    obstacle_hold_source_trajectory_id_ = 0;
    obstacle_replan_requires_away_motion_ = false;
  }

  void beginTransientObstacleHold(const std::string& reason,
                                  bool hard_clearance_intrusion,
                                  const ros::Time& now) {
    const bool was_active = obstacle_monitor_.active();
    const auto action = obstacle_monitor_.update(true, now.toSec());
    obstacle_monitor_reason_ = reason;
    obstacle_safety_ready_ = false;
    if (!was_active) {
      // Capture one measured-pose hold. Recomputing the target on every timer
      // tick would follow a drifting vehicle instead of actually hovering.
      hold_target_latched_ = false;
      // Planner output produced during a fixed obstacle hold belongs to the
      // old motion epoch. Do not compare a later replan against that stale
      // endpoint; the first fresh command will instead be checked against the
      // current measured pose and then reacquired from this hold target.
      valid_command_ = false;
      last_command_time_ = ros::Time();
      latest_source_stamp_ = ros::Time();
      source_stream_active_ = false;
      managed_hold_reacquire_pending_ =
          offboard_owned_ && px4_mode_ == offboard_mode_ &&
          have_published_target_;
      resetContinuityGuard();
      resetReacquire();
      obstacle_replan_requested_ = false;
      obstacle_replan_escape_active_ = false;
      obstacle_replan_escape_started_ = ros::Time();
      obstacle_hold_source_trajectory_id_ = latest_source_trajectory_id_;
      ROS_WARN("Obstacle entered the monitored avoidance zone; holding for "
               "%.1f s of continuous observation (%s)",
               obstacle_persistence_duration_, reason.c_str());
    }
    if (mine_uav_control::persistentHardIntrusionRequiresVerifiedReturn(
            action, hard_clearance_intrusion)) {
      if (!autonomousReturnHealthReady(now)) {
        ROS_ERROR("Persistent hard obstacle cannot be escaped autonomously "
                  "because localization or perception is unhealthy");
        triggerObstacleHazard("PERSISTENT_HARD_OBSTACLE_NO_RETURN_SENSING");
        return;
      }
      obstacle_monitor_reason_ = "PERSISTENT_HARD_OBSTACLE_RETURN_WAIT";
      obstacle_replan_requires_away_motion_ = true;
      if (!obstacle_replan_requested_) {
        obstacle_replan_requested_ = true;
        obstacle_replan_publisher_.publish(std_msgs::Empty());
        publishReturnRequest(true);
        ROS_ERROR("Obstacle remained inside the %.3f m hard radius for %.3f "
                  "s; retaining OFFBOARD and requesting a verified "
                  "clearance-increasing return trajectory",
                  live_horizontal_clearance_,
                  obstacle_monitor_.elapsed(now.toSec()));
      }
    } else if (action ==
               mine_uav_control::ObstacleMonitorAction::kPersistent) {
      // A persistent return outside the hard radius is a planning problem,
      // not a reason to wait forever or leave OFFBOARD. Ask the exploration
      // layer to invalidate this endpoint exactly once. The hold remains
      // fixed until SUPER emits a genuinely new trajectory.
      obstacle_monitor_reason_ = "PERSISTENT_OBSTACLE_REPLAN_WAIT";
      obstacle_replan_requires_away_motion_ =
          mine_uav_control::persistentObstacleReplanRequiresAwayMotion(action);
      if (!obstacle_replan_requested_) {
        obstacle_replan_requested_ = true;
        obstacle_replan_publisher_.publish(std_msgs::Empty());
        ROS_WARN("Persistent soft obstacle requested a fresh verified, "
                 "clearance-increasing route");
      }
      ROS_WARN_THROTTLE(
          1.0,
          "Obstacle persisted for %.3f s in the planning zone but remains "
          "outside the %.3f m hard radius; holding while map/planner update",
          obstacle_monitor_.elapsed(now.toSec()), live_horizontal_clearance_);
    }
  }

  bool clearTransientObstacleHold(const ros::Time& now) {
    const bool was_active = obstacle_monitor_.active();
    const auto action = obstacle_monitor_.update(false, now.toSec());
    if (action != mine_uav_control::ObstacleMonitorAction::kClear) {
      obstacle_safety_ready_ = false;
      return false;
    }
    if (was_active) {
      ROS_INFO("Transient obstacle cleared for %d scans; resuming the "
               "current task with smooth reacquisition",
               obstacle_clear_confirm_frames_);
      obstacle_monitor_reason_.clear();
      hold_target_latched_ = false;
      valid_command_ = false;
      last_command_time_ = ros::Time();
      latest_source_stamp_ = ros::Time();
      source_stream_active_ = false;
      resetContinuityGuard();
      resetReacquire();
    }
    return true;
  }

  void obstacleCloudCallback(
      const sensor_msgs::PointCloud2::ConstPtr& cloud) {
    if (!obstacle_safety_enabled_) return;
    std::vector<mine_uav_control::ObstaclePoint> points;
    const std::size_t total =
        static_cast<std::size_t>(cloud->width) * cloud->height;
    const std::size_t stride = std::max<std::size_t>(
        1, (total + static_cast<std::size_t>(max_obstacle_points_) - 1) /
               static_cast<std::size_t>(max_obstacle_points_));
    points.reserve(std::min<std::size_t>(total, max_obstacle_points_));
    try {
      sensor_msgs::PointCloud2ConstIterator<float> x(*cloud, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y(*cloud, "y");
      sensor_msgs::PointCloud2ConstIterator<float> z(*cloud, "z");
      std::size_t index = 0;
      for (; x != x.end(); ++x, ++y, ++z, ++index) {
        if (index % stride != 0) continue;
        if (!finite(*x) || !finite(*y) || !finite(*z)) continue;
        points.push_back({*x, *y, *z});
      }
    } catch (const std::runtime_error& error) {
      have_obstacle_cloud_ = false;
      if (auto_enabled_ && mission_enabled_) {
        triggerObstacleHazard("INVALID_OBSTACLE_CLOUD");
      }
      ROS_ERROR_THROTTLE(1.0, "Invalid obstacle cloud: %s", error.what());
      return;
    }
    const ros::Time now = ros::Time::now();
    last_obstacle_cloud_time_ = now;
    have_obstacle_cloud_ = true;
    if (!have_task_odometry_ ||
        (now - last_task_odometry_time_).toSec() > obstacle_cloud_timeout_) {
      obstacle_safety_ready_ = false;
      return;
    }

    // The monitor intentionally evaluates the current scan rather than the
    // permanent map. A leaf, loose rock, or lidar speckle must stop motion
    // immediately, but it must also be allowed to clear and resume the task.
    const auto clearance = mine_uav_control::liveHorizontalCylinderClearance(
        points, latest_task_position_, obstacle_monitor_horizontal_clearance_,
        live_vertical_clearance_below_, live_vertical_clearance_above_,
        obstacle_min_z_);
    bool obstacle_requires_monitoring = clearance.violated;
    std::string monitor_reason =
        clearance.horizontal_distance < live_horizontal_clearance_
            ? "HARD_CLEARANCE_INTRUSION"
            : "OBSTACLE_IN_AVOIDANCE_ZONE";
    if (clearance.violated) {
      ROS_WARN_THROTTLE(
          0.5,
          "OBSTACLE_MONITOR: vehicle task xyz=(%.3f, %.3f, %.3f), "
          "obstacle=(%.3f, %.3f, %.3f), horizontal=%.3f m, "
          "hard/monitor radii=%.3f/%.3f m, vertical=%.3f m",
          latest_task_position_.x, latest_task_position_.y,
          latest_task_position_.z, clearance.obstacle.x,
          clearance.obstacle.y, clearance.obstacle.z,
          clearance.horizontal_distance, live_horizontal_clearance_,
          obstacle_monitor_horizontal_clearance_,
          clearance.vertical_distance);
    }

    double forward_x = latest_planning_velocity_x_;
    double forward_y = latest_planning_velocity_y_;
    if (std::hypot(forward_x, forward_y) < 0.05) {
      forward_x = last_planning_forward_x_;
      forward_y = last_planning_forward_y_;
    } else {
      const double speed = std::hypot(forward_x, forward_y);
      last_planning_forward_x_ = forward_x / speed;
      last_planning_forward_y_ = forward_y / speed;
    }
    const auto coverage = mine_uav_control::evaluateForwardObstacleCoverage(
        points, latest_task_position_, forward_x, forward_y,
        coverage_guard_distance_, coverage_lateral_half_width_,
        live_vertical_clearance_below_ +
            coverage_vertical_uncertainty_margin_,
        live_vertical_clearance_above_ +
            coverage_vertical_uncertainty_margin_,
        static_cast<std::size_t>(coverage_min_points_),
        coverage_min_horizontal_span_, coverage_min_vertical_span_,
        obstacle_min_z_);
    bool surface_ready = mine_uav_control::updateSurfaceCoverageReadiness(
        coverage.obstacle_nearby, coverage.complete,
        coverage_confirm_frames_, coverage_reject_frames_,
        &obstacle_coverage_complete_streak_,
        &obstacle_coverage_incomplete_streak_,
        &obstacle_surface_coverage_ready_);
    if (coverage.obstacle_nearby && !surface_ready) {
      if (returning_home_ && !clearance.violated) {
        // Surface completeness is an outbound approach precaution. During a
        // verified return it must not pin the vehicle beside sparse wall
        // returns that are already outside the 0.8 m planning cylinder. The
        // live 0.65/0.8 m clearance checks and cloud freshness remain active.
        surface_ready = true;
        ROS_INFO_THROTTLE(
            1.0,
            "Return route continuing past incomplete surface outside the "
            "planning cylinder (nearest %.3f m)",
            coverage.nearest_horizontal);
      } else {
        ROS_WARN_THROTTLE(
            0.5,
            "OBSTACLE_SURFACE_INCOMPLETE: points=%zu horizontal_span=%.3f m "
            "vertical_span=%.3f m nearest=%.3f m; holding before approach",
            coverage.point_count, coverage.horizontal_span,
            coverage.vertical_span, coverage.nearest_horizontal);
        obstacle_requires_monitoring = true;
        latest_monitor_obstacle_ = coverage.nearest_obstacle;
        have_latest_monitor_obstacle_ = true;
        if (!clearance.violated) {
          monitor_reason = "OBSTACLE_SURFACE_INCOMPLETE";
        }
      }
    }

    const bool hard_clearance_intrusion =
        clearance.violated &&
        clearance.horizontal_distance < live_horizontal_clearance_ - 1e-9;
    if (clearance.violated) {
      latest_monitor_obstacle_ = clearance.obstacle;
      have_latest_monitor_obstacle_ = true;
    }
    const mine_uav_control::ObstaclePoint planned_target{
        latest_planning_position_x_, latest_planning_position_y_,
        latest_task_position_.z};
    if (!obstacle_monitor_.active() &&
        mine_uav_control::verifiedReturnSoftZoneEscapeAllowed(
            returning_home_, valid_command_, clearance.violated,
            hard_clearance_intrusion, latest_task_position_,
            clearance.obstacle, planned_target,
            latest_planning_velocity_x_, latest_planning_velocity_y_)) {
      // The current SUPER return is already taking the vehicle away from the
      // nearest wall. Freezing it at the 0.8 m planning boundary would discard
      // the only clearance-increasing motion and create a hold/replan loop.
      // The 0.65 m hard cylinder and cloud freshness checks remain active.
      obstacle_safety_ready_ = true;
      ROS_INFO_THROTTLE(
          0.5,
          "Continuing verified return escape through soft obstacle zone: "
          "clearance %.3f m, target=(%.3f, %.3f)",
          clearance.horizontal_distance, planned_target.x, planned_target.y);
      return;
    }
    if (obstacle_replan_escape_active_) {
      const bool escape_expired =
          obstacle_replan_escape_started_.isZero() ||
          (now - obstacle_replan_escape_started_).toSec() >=
              obstacle_replan_escape_duration_;
      const mine_uav_control::ObstaclePoint planned_target{
          latest_planning_position_x_, latest_planning_position_y_,
          latest_task_position_.z};
      const bool moving_away = have_latest_monitor_obstacle_ &&
          mine_uav_control::plannedMotionIncreasesHorizontalClearance(
              latest_task_position_, latest_monitor_obstacle_, planned_target,
              latest_planning_velocity_x_, latest_planning_velocity_y_);
      if (!obstacle_requires_monitoring) {
        obstacle_replan_escape_active_ = false;
        obstacle_replan_escape_started_ = ros::Time();
      } else if (obstacle_replan_requires_away_motion_ && !moving_away) {
        // A replacement trajectory may continue only while it increases
        // clearance from the live obstacle. This applies to the soft planning
        // zone too; a new trajectory id alone is not evidence of avoidance.
        obstacle_replan_escape_active_ = false;
      } else if (!escape_expired) {
        // The replacement trajectory has been checked by SUPER. Give it a
        // bounded opportunity to move away from the old soft-obstacle zone;
        // the 0.65 m hard cylinder and cloud freshness remain enforced.
        obstacle_safety_ready_ = true;
        return;
      } else {
        obstacle_replan_escape_active_ = false;
        obstacle_replan_escape_started_ = ros::Time();
      }
    }

    if (obstacle_requires_monitoring) {
      beginTransientObstacleHold(monitor_reason,
                                 hard_clearance_intrusion, now);
      return;
    }
    if (!clearTransientObstacleHold(now)) {
      return;
    }
    obstacle_safety_ready_ = surface_ready;
  }

  void localVelocityCallback(
      const geometry_msgs::TwistStamped::ConstPtr& velocity) {
    const auto& v = velocity->twist.linear;
    if (!finite(v.x) || !finite(v.y) || !finite(v.z)) {
      have_local_velocity_ = false;
      overspeed_since_ = ros::Time();
      if (auto_enabled_ && offboard_owned_) {
        latchFault("INVALID_LOCAL_VELOCITY");
      }
      return;
    }
    const ros::Time now = ros::Time::now();
    latest_measured_speed_ = vectorNorm(v.x, v.y, v.z);
    last_local_velocity_time_ = now;
    have_local_velocity_ = true;
    if (!(auto_enabled_ && offboard_owned_ &&
          px4_mode_ == offboard_mode_)) {
      overspeed_since_ = ros::Time();
      return;
    }
    const bool recovering = !overspeed_since_.isZero();
    const double continuous_overspeed_seconds = recovering
        ? (now - overspeed_since_).toSec()
        : 0.0;
    const auto action = mine_uav_control::overspeedRecoveryAction(
        latest_measured_speed_, max_measured_speed_,
        continuous_overspeed_seconds, overspeed_duration_);
    if (action == mine_uav_control::OverspeedRecoveryAction::kNormalFlight) {
      if (recovering) {
        ROS_INFO("Measured speed recovered to %.3f m/s within the %.1f s "
                 "window; continuing automatic flight",
                 latest_measured_speed_, overspeed_duration_);
      }
      overspeed_since_ = ros::Time();
      return;
    }
    if (overspeed_since_.isZero()) {
      overspeed_since_ = now;
    }
    if (action == mine_uav_control::OverspeedRecoveryAction::kReduceSpeed) {
      ROS_WARN_THROTTLE(
          0.5,
          "Measured speed %.3f m/s exceeds %.3f m/s; speed governor is "
          "reducing the request (%.2f/%.2f s before OFFBOARD exit)",
          latest_measured_speed_, max_measured_speed_,
          continuous_overspeed_seconds, overspeed_duration_);
      return;
    }
    if (action == mine_uav_control::OverspeedRecoveryAction::kExitOffboard) {
      ROS_ERROR_THROTTLE(
          0.5,
          "Measured automatic-flight speed %.3f m/s remained above hard "
          "limit %.3f m/s for %.2f s",
          latest_measured_speed_, max_measured_speed_,
          continuous_overspeed_seconds);
      valid_command_ = false;
      latchFault("MEASURED_SPEED_LIMIT");
    }
  }

  bool gatesReady() const {
    return software_enabled_ && auto_enabled_ && mission_enabled_ &&
           !mission_complete_ &&
           alignment_ready_ && vision_healthy_ && mavros_connected_ &&
           localVelocityFresh(ros::Time::now()) &&
           obstacleCloudFresh(ros::Time::now()) && obstacle_safety_ready_ &&
           !fault_latched_;
  }

  bool localPoseFresh(const ros::Time& now) const {
    return have_local_pose_ && !last_local_pose_time_.isZero() &&
           (now - last_local_pose_time_).toSec() <= local_pose_timeout_;
  }

  bool localVelocityFresh(const ros::Time& now) const {
    return have_local_velocity_ && !last_local_velocity_time_.isZero() &&
           (now - last_local_velocity_time_).toSec() <=
               measured_speed_timeout_;
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
    mavros_msgs::PositionTarget output;
    output.header.stamp = ros::Time::now();
    output.header.frame_id = "map";
    output.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    output.type_mask = 0;

    const auto position = mine_uav_control::rotateVector(
        alignment_rotation_, mine_uav_control::Vector3(
            command.position.x, command.position.y, command.position.z));
    const auto velocity = mine_uav_control::rotateVector(
        alignment_rotation_, mine_uav_control::Vector3(
            command.velocity.x, command.velocity.y, command.velocity.z));
    const auto acceleration = mine_uav_control::rotateVector(
        alignment_rotation_, mine_uav_control::Vector3(
            command.acceleration.x, command.acceleration.y,
            command.acceleration.z));
    output.position.x = position.x + alignment_x_;
    output.position.y = position.y + alignment_y_;
    output.position.z = position.z + alignment_z_;
    output.velocity.x = velocity.x;
    output.velocity.y = velocity.y;
    output.velocity.z = velocity.z;
    output.acceleration_or_force.x = acceleration.x;
    output.acceleration_or_force.y = acceleration.y;
    output.acceleration_or_force.z = acceleration.z;

    // Transform the horizontal heading and its derivative through the same
    // 3-D map rotation. This remains correct when pitch compensation is active.
    const auto heading = mine_uav_control::rotateVector(
        alignment_rotation_, mine_uav_control::Vector3(
            std::cos(command.yaw), std::sin(command.yaw), 0.0));
    const auto heading_derivative = mine_uav_control::rotateVector(
        alignment_rotation_, mine_uav_control::Vector3(
            -std::sin(command.yaw), std::cos(command.yaw), 0.0));
    const double horizontal_heading_squared =
        heading.x * heading.x + heading.y * heading.y;
    output.yaw = normalizeAngle(std::atan2(heading.y, heading.x));
    output.yaw_rate = horizontal_heading_squared > 1e-8
        ? command.yaw_dot *
              (heading.x * heading_derivative.y -
               heading.y * heading_derivative.x) /
              horizontal_heading_squared
        : 0.0;
    // The planner trajectory is expressed at the LIO sensor/IMU origin,
    // while PX4 controls the aircraft body origin. Remove the attitude-
    // dependent lever-arm displacement. At handoff the body yaw in PX4 is the
    // planning-to-PX4 alignment yaw, so use that same rotated initial offset
    // as the baseline; the correction must be zero at handoff.
    if (have_sensor_position_) {
      const auto desired_sensor_offset = mine_uav_control::rotateVector(
          mine_uav_control::quaternionFromRpy(0.0, 0.0, output.yaw),
          sensor_position_in_body_);
      const auto initial_sensor_offset = mine_uav_control::rotateVector(
          alignment_rotation_, sensor_position_in_body_);
      output.position.x -=
          desired_sensor_offset.x - initial_sensor_offset.x;
      output.position.y -=
          desired_sensor_offset.y - initial_sensor_offset.y;
      output.position.z -=
          desired_sensor_offset.z - initial_sensor_offset.z;
    }
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
    active_reacquire_duration_ = reacquire_blend_duration_;
    source_stream_active_ = false;
    speed_governor_active_ = false;
    adaptive_predicted_speed_limit_ = max_predicted_speed_;
    last_speed_governor_update_time_ = ros::Time();
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
    if (!have_published_target_) {
      return makeHoldTarget(now);
    }
    // Task teardown intentionally clears the source-command timestamp. The
    // already transmitted endpoint is nevertheless the only continuous hold
    // reference; switching to measured pose here injects the tracking error
    // as a position step at the end of the brake.
    auto hold = published_target_;
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
    const double sp[3] = {start.position.x, start.position.y, start.position.z};
    const double sv[3] = {start.velocity.x, start.velocity.y, start.velocity.z};
    const double sa[3] = {start.acceleration_or_force.x, start.acceleration_or_force.y, start.acceleration_or_force.z};
    const double dp[3] = {desired.position.x, desired.position.y, desired.position.z};
    const double dv[3] = {desired.velocity.x, desired.velocity.y, desired.velocity.z};
    const double da[3] = {desired.acceleration_or_force.x, desired.acceleration_or_force.y, desired.acceleration_or_force.z};
    const double position_offset =
        vectorNorm(sp[0]-dp[0], sp[1]-dp[1], sp[2]-dp[2]);
    const double yaw_offset =
        std::abs(normalizeAngle(start.yaw-desired.yaw));
    active_reacquire_duration_ = mine_uav_control::smoothReacquireDuration(
        reacquire_blend_duration_, position_offset, yaw_offset,
        max_position_rate_, max_yaw_rate_);
    const double T = active_reacquire_duration_;
    for (int i = 0; i < 3; ++i) {
      reacquire_xyz_[i] = mine_uav_control::QuinticOffset1D::make(
          sp[i]-dp[i], sv[i]-dv[i], sa[i]-da[i], T);
    }
    reacquire_yaw_ = mine_uav_control::QuinticOffset1D::make(
        normalizeAngle(start.yaw-desired.yaw), start.yaw_rate-desired.yaw_rate, 0.0, T);
    reacquire_start_time_ = now;
    reacquire_active_ = true;
    managed_hold_reacquire_pending_ = false;
    ROS_INFO("Starting derivative-consistent SUPER handoff over %.2f s (offset %.3f m)",
             T, position_offset);
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
    if (elapsed>=active_reacquire_duration_-1e-9) {
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
      if (have_published_target_) {
        const double managed_position_offset = vectorNorm(
            command.position.x - published_target_.position.x,
            command.position.y - published_target_.position.y,
            command.position.z - published_target_.position.z);
        const double managed_yaw_offset = std::abs(normalizeAngle(
            command.yaw - published_target_.yaw));
        if (mine_uav_control::managedHoldReacquireAllowed(
                managed_hold_reacquire_pending_, offboard_owned_,
                px4_mode_ == offboard_mode_, have_published_target_,
                managed_position_offset, max_managed_reacquire_distance_,
                managed_yaw_offset, max_managed_reacquire_yaw_error_)) {
          // This raw planner sample is never transmitted directly. The output
          // timer splices it to the fixed, already-published hold with a
          // derivative-consistent quintic before PX4 sees it.
          previous_target_ = command;
          previous_target_time_ = now;
          have_previous_target_ = true;
          ROS_WARN("Accepting post-hold SUPER source %.3f m / %.1f deg from "
                   "the fixed hold; smooth reacquisition will bridge it",
                   managed_position_offset,
                   managed_yaw_offset * 180.0 / kPi);
          return true;
        }
      }
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
                          const double planning_height,
                          std::string* reason) const {
    const double horizontal_radius =
        std::hypot(command.position.x, command.position.y);
    if (max_horizontal_radius_ > 0.0 &&
        horizontal_radius > max_horizontal_radius_) {
      *reason = "HORIZONTAL_GEOFENCE";
      return false;
    }
    if (planning_height < min_height_ || planning_height > max_height_) {
      *reason = "HEIGHT_GEOFENCE";
      return false;
    }
    return true;
  }

  void commandCallback(
      const quadrotor_msgs::PositionCommand::ConstPtr& command) {
    if (!mission_enabled_) {
      // SUPER can keep publishing a terminal command after task teardown.
      // It must not refill the cache that the next task epoch will use.
      valid_command_ = false;
      return;
    }
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
    latest_planning_velocity_x_ = command->velocity.x;
    latest_planning_velocity_y_ = command->velocity.y;
    latest_planning_position_x_ = command->position.x;
    latest_planning_position_y_ = command->position.y;
    if (obstacle_monitor_.active()) {
      // Continue learning the planner's forward direction for coverage, but
      // never let the old trajectory advance the cache while output is pinned
      // to a measured-pose hold. A persistent soft obstacle may be released
      // only by a newly numbered SUPER trajectory requested by this bridge.
      const bool fresh_replan = obstacle_replan_requested_ &&
          command->trajectory_id != obstacle_hold_source_trajectory_id_;
      const mine_uav_control::ObstaclePoint planned_target{
          command->position.x, command->position.y, command->position.z};
      const bool escape_moves_away =
          !obstacle_replan_requires_away_motion_ ||
          (have_latest_monitor_obstacle_ &&
           mine_uav_control::plannedMotionIncreasesHorizontalClearance(
               latest_task_position_, latest_monitor_obstacle_,
               planned_target, command->velocity.x, command->velocity.y));
      if (!fresh_replan || !escape_moves_away) {
        valid_command_ = false;
        resetContinuityGuard();
        return;
      }
      const bool require_away_motion = obstacle_replan_requires_away_motion_;
      const auto escape_obstacle = latest_monitor_obstacle_;
      const bool have_escape_obstacle = have_latest_monitor_obstacle_;
      resetTransientObstacleMonitor();
      obstacle_replan_escape_active_ = true;
      obstacle_replan_requires_away_motion_ = require_away_motion;
      obstacle_replan_escape_started_ = receipt;
      latest_monitor_obstacle_ = escape_obstacle;
      have_latest_monitor_obstacle_ = have_escape_obstacle;
      obstacle_safety_ready_ = true;
      hold_target_latched_ = false;
      resetContinuityGuard();
      resetReacquire();
      ROS_WARN("Accepting newly numbered SUPER avoidance trajectory %u with "
               "bounded soft-obstacle escape",
               command->trajectory_id);
    }
    auto bounded_command = *command;
    if (bounded_command.position.z > max_height_ &&
        bounded_command.position.z <= max_height_ + height_clamp_tolerance_) {
      ROS_WARN_THROTTLE(
          2.0,
          "Clamping small SUPER planning-frame height overshoot %.3f to "
          "max_height %.3f",
          bounded_command.position.z, max_height_);
      bounded_command.position.z = max_height_;
    } else if (bounded_command.position.z < min_height_ &&
               bounded_command.position.z >= min_height_ -
                   height_clamp_tolerance_) {
      ROS_WARN_THROTTLE(
          2.0,
          "Clamping small SUPER planning-frame height undershoot %.3f to "
          "min_height %.3f",
          bounded_command.position.z, min_height_);
      bounded_command.position.z = min_height_;
    }
    auto bounded_output = convert(bounded_command);
    if (!insideFlightVolume(bounded_output, bounded_command.position.z,
                            &reason)) {
      if (max_horizontal_radius_ > 0.0) {
        ROS_ERROR_THROTTLE(
            1.0,
            "Rejecting SUPER command outside flight volume: px4_xyz=(%.3f, "
            "%.3f, %.3f), planning_z=%.3f, limits: radius<=%.3f, "
            "planning_z=[%.3f, %.3f], alignment_z=%.3f",
            bounded_output.position.x, bounded_output.position.y,
            bounded_output.position.z, bounded_command.position.z,
            max_horizontal_radius_, min_height_, max_height_, alignment_z_);
      } else {
        ROS_ERROR_THROTTLE(
            1.0,
            "Rejecting SUPER command outside flight volume: px4_xyz=(%.3f, "
            "%.3f, %.3f), planning_z=%.3f, limits: horizontal radius "
            "disabled, planning_z=[%.3f, %.3f], alignment_z=%.3f",
            bounded_output.position.x, bounded_output.position.y,
            bounded_output.position.z, bounded_command.position.z,
            min_height_, max_height_, alignment_z_);
      }
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
      recovery_attempted_ = false;
    }
    valid_command_ = true;
    if (px4_mode_ == offboard_mode_) {
      hold_target_latched_ = false;
    }
    latest_target_ = accepted_output;
    latest_source_trajectory_id_ = command->trajectory_id;
    latest_source_stamp_ = command->header.stamp;
    updateStatus();
  }

  bool publishCheckedTarget(mavros_msgs::PositionTarget target, const ros::Time& now,
                            bool feedforward_governed = false) {
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
          max_velocity_slew_,max_acceleration_slew_,max_yaw_rate_,
          !feedforward_governed && !published_target_feedforward_governed_);
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
    published_target_feedforward_governed_ = feedforward_governed;
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

  bool governPredictedTrackingSpeed(mavros_msgs::PositionTarget* target,
                                    const ros::Time& now) {
    const double error_x =
        target->position.x - latest_local_pose_.pose.position.x;
    const double error_y =
        target->position.y - latest_local_pose_.pose.position.y;
    const double error_z =
        target->position.z - latest_local_pose_.pose.position.z;
    const double tracking_error = vectorNorm(error_x, error_y, error_z);
    if (tracking_error > max_governor_tracking_error_) {
      ROS_ERROR(
          "PX4 tracking error %.3f m exceeds governor limit %.3f m; refusing "
          "to hide a localization or trajectory fault",
          tracking_error, max_governor_tracking_error_);
      return false;
    }

    const std::array<double, 3> planned_velocity{{
        target->velocity.x, target->velocity.y, target->velocity.z}};
    const std::array<double, 3> position_error{{error_x, error_y, error_z}};
    const std::array<double, 3> position_gain{{
        tracking_position_gain_xy_, tracking_position_gain_xy_,
        tracking_position_gain_z_}};
    const double requested_limit =
        mine_uav_control::measuredSpeedRequestLimit(
            latest_measured_speed_, measured_speed_target_,
            measured_speed_feedback_gain_, max_predicted_speed_,
            measured_speed_governor_floor_);
    if (requested_limit <= 0.0) {
      ROS_ERROR("Measured-speed governor received invalid configuration");
      return false;
    }
    double governor_dt = continuity_nominal_dt_;
    if (!last_speed_governor_update_time_.isZero()) {
      governor_dt = (now - last_speed_governor_update_time_).toSec();
      if (!finite(governor_dt) || governor_dt <= 0.0) {
        governor_dt = continuity_nominal_dt_;
      }
      governor_dt = std::min(0.10, governor_dt);
    }
    last_speed_governor_update_time_ = now;
    adaptive_predicted_speed_limit_ =
        mine_uav_control::slewMeasuredSpeedRequestLimit(
            adaptive_predicted_speed_limit_, requested_limit, governor_dt,
            measured_speed_governor_reduction_rate_,
            measured_speed_governor_recovery_rate_);
    if (adaptive_predicted_speed_limit_ <= 0.0) {
      ROS_ERROR("Measured-speed governor failed to slew its request limit");
      return false;
    }

    const auto governed = mine_uav_control::governTrackingSpeed(
        planned_velocity, position_error, position_gain,
        adaptive_predicted_speed_limit_);
    if (!governed.valid) {
      ROS_ERROR("PX4 tracking speed governor received non-finite input");
      return false;
    }

    const bool measured_speed_governing =
        adaptive_predicted_speed_limit_ < max_predicted_speed_ - 1e-6;
    speed_governor_active_ = governed.active || measured_speed_governing;
    if (!governed.active) {
      if (measured_speed_governing) {
        ROS_WARN_THROTTLE(
            1.0,
            "Measured-speed headroom governor active: measured=%.3f m/s, "
            "adaptive request limit=%.3f m/s",
            latest_measured_speed_, adaptive_predicted_speed_limit_);
      }
      return true;
    }

    const double governed_feedforward_speed = vectorNorm(
        governed.feedforward_velocity[0], governed.feedforward_velocity[1],
        governed.feedforward_velocity[2]);
    if (governed_feedforward_speed > max_speed_ + 1e-6) {
      ROS_ERROR(
          "PX4 speed governor would require %.3f m/s feed-forward, above "
          "bridge limit %.3f m/s (tracking error %.3f m)",
          governed_feedforward_speed, max_speed_, tracking_error);
      return false;
    }

    const double raw_speed = vectorNorm(
        planned_velocity[0] + position_gain[0] * position_error[0],
        planned_velocity[1] + position_gain[1] * position_error[1],
        planned_velocity[2] + position_gain[2] * position_error[2]);
    target->velocity.x = governed.feedforward_velocity[0];
    target->velocity.y = governed.feedforward_velocity[1];
    target->velocity.z = governed.feedforward_velocity[2];
    // Acceleration is feed-forward too. Scale it continuously with the speed
    // intervention instead of leaving the original trajectory acceleration
    // free to push through an actively governed velocity request.
    target->acceleration_or_force.x *= governed.scale;
    target->acceleration_or_force.y *= governed.scale;
    target->acceleration_or_force.z *= governed.scale;
    ROS_WARN_THROTTLE(
        1.0,
        "Governing PX4 tracking request %.3f -> %.3f m/s without abort "
        "(measured=%.3f m/s): "
        "planner_v=(%.3f, %.3f, %.3f), error=(%.3f, %.3f, %.3f), "
        "governed_feedforward=(%.3f, %.3f, %.3f)",
        raw_speed, adaptive_predicted_speed_limit_, latest_measured_speed_,
        planned_velocity[0], planned_velocity[1], planned_velocity[2],
        error_x, error_y, error_z,
        target->velocity.x, target->velocity.y, target->velocity.z);
    return true;
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

  bool offboardEntrySafe(const ros::Time& now, std::string* reason) const {
    if (!localPoseFresh(now) || !localVelocityFresh(now)) {
      *reason = "PRESTREAM_STATE_NOT_FRESH";
      return false;
    }
    if (latest_measured_speed_ > max_offboard_entry_speed_) {
      *reason = "PRESTREAM_VEHICLE_NOT_STABLE";
      return false;
    }
    const auto desired = projectSourceTarget(now);
    const auto& p = latest_local_pose_.pose.position;
    const double error_x = desired.position.x - p.x;
    const double error_y = desired.position.y - p.y;
    const double error_z = desired.position.z - p.z;
    const double tracking_error = vectorNorm(error_x, error_y, error_z);
    if (tracking_error > max_governor_tracking_error_) {
      ROS_ERROR(
          "Refusing OFFBOARD entry: initial tracking error %.3f m exceeds %.3f m",
          tracking_error, max_governor_tracking_error_);
      *reason = "PRESTREAM_TRACKING_ERROR";
      return false;
    }
    const double predicted_speed = vectorNorm(
        desired.velocity.x + tracking_position_gain_xy_ * error_x,
        desired.velocity.y + tracking_position_gain_xy_ * error_y,
        desired.velocity.z + tracking_position_gain_z_ * error_z);
    if (!finite(predicted_speed)) {
      *reason = "PRESTREAM_INVALID_PREDICTED_SPEED";
      return false;
    }
    return true;
  }

  void outputTimerCallback(const ros::TimerEvent&) {
    const ros::Time now = ros::Time::now();
    if (auto_enabled_ && mission_enabled_ && !vision_healthy_ &&
        px4_mode_ != offboard_mode_) {
      if (preoffboard_vision_unhealthy_since_.isZero()) {
        preoffboard_vision_unhealthy_since_ = now;
      }
      if ((now - preoffboard_vision_unhealthy_since_).toSec() >=
          preoffboard_vision_recovery_timeout_) {
        latchFault("PREOFFBOARD_VISION_RECOVERY_TIMEOUT");
      }
    } else if (!auto_enabled_ || !mission_enabled_ || vision_healthy_) {
      preoffboard_vision_unhealthy_since_ = ros::Time();
    }
    if (obstacle_safety_enabled_ && auto_enabled_ && mission_enabled_ &&
        offboard_owned_ && px4_mode_ == offboard_mode_ &&
        obstacleCloudFaulted(now)) {
      triggerObstacleHazard("OBSTACLE_CLOUD_TIMEOUT");
    }
    if (obstacle_hazard_stop_active_ && px4_mode_ == offboard_mode_) {
      // Collision proximity is different from an ordinary command timeout:
      // do not extrapolate the old velocity for the normal smooth brake.
      // Command the measured pose with zero derivatives, then immediately
      // request the pilot/fallback position mode.
      auto hold = makeHoldTarget(now);
      command_publisher_.publish(hold);
      published_target_ = hold;
      published_target_time_ = now;
      published_target_feedforward_governed_ = false;
      have_published_target_ = true;
      publishReady(false);
      requestMode(fallback_mode_, now, false);
      publishStatus("OBSTACLE_HAZARD_STOP_EXITING_OFFBOARD");
      return;
    }
    if (obstacle_safety_enabled_ && auto_enabled_ && mission_enabled_ &&
        offboard_owned_ && px4_mode_ == offboard_mode_ &&
        !obstacleCloudFresh(now) && !fault_latched_) {
      // Omni-LIVO occasionally withholds a single cloud while its output
      // clock catches up.  Treat that bounded gap like a monitored stop: keep
      // the last certified OFFBOARD trajectory continuous, brake to a fixed
      // hold, and smoothly reacquire when clouds resume.  A gap reaching the
      // separate fault timeout above still exits OFFBOARD.
      if (!cloud_gap_hold_active_) {
        cloud_gap_hold_active_ = true;
        source_stream_active_ = false;
        resetContinuityGuard();
        resetReacquire();
        ROS_WARN("Obstacle cloud briefly stale; braking in OFFBOARD while "
                 "waiting up to %.2f s for recovery",
                 obstacle_cloud_fault_timeout_);
      }
      if (brake_active_ ||
          (have_published_target_ && !outputNearRest(published_target_))) {
        startEmergencyBrake();
        if (!publishCheckedTarget(emergencyBrakeTarget(now), now)) return;
        if ((now - brake_start_time_).toSec() >= brake_duration_) {
          brake_active_ = false;
          hold_target_latched_ = false;
        }
        publishReady(false);
        publishStatus("OBSTACLE_CLOUD_RECOVERY_BRAKE");
        return;
      }
      if (!publishCheckedTarget(makeFixedHoldTarget(now, true), now)) return;
      publishReady(false);
      publishStatus("OBSTACLE_CLOUD_RECOVERY_HOLD");
      return;
    }
    if (cloud_gap_hold_active_ && obstacleCloudFresh(now)) {
      cloud_gap_hold_active_ = false;
      hold_target_latched_ = false;
      source_stream_active_ = false;
      resetContinuityGuard();
      resetReacquire();
      ROS_INFO("Obstacle cloud recovered; smoothly reacquiring the current "
               "certified trajectory");
    }
    if (obstacle_monitor_.active() && px4_mode_ == offboard_mode_ &&
        !fault_latched_) {
      // Retain OFFBOARD for the bounded observation window, but never replace
      // a moving command with measured pose/yaw in one sample. Brake from the
      // last state actually sent to PX4, then pin that exact endpoint. This
      // keeps position, velocity and yaw continuous while the planner replans.
      if (brake_active_ ||
          (have_published_target_ && !outputNearRest(published_target_))) {
        startEmergencyBrake();
        if (!publishCheckedTarget(emergencyBrakeTarget(now), now)) return;
        if ((now - brake_start_time_).toSec() >= brake_duration_) {
          brake_active_ = false;
          hold_target_latched_ = false;
        }
        publishReady(false);
        publishStatus("OBSTACLE_MONITORING_BRAKE:" +
                      obstacle_monitor_reason_);
        return;
      }
      if (!publishCheckedTarget(makeFixedHoldTarget(now, true), now)) return;
      publishReady(false);
      publishStatus("OBSTACLE_MONITORING_HOLD:" + obstacle_monitor_reason_);
      return;
    }
    if (recovery_pending_ &&
        (now - recovery_started_).toSec() > return_recovery_timeout_) {
      if (autonomousReturnHealthReady(now)) {
        recovery_started_ = now;
        publishReturnRequest(true);
        publishStatus("VERIFIED_RETURN_RETRYING");
        ROS_WARN("Verified return is not available yet; retaining OFFBOARD "
                 "hold and retrying because localization and perception are healthy");
      } else {
        recovery_pending_ = false;
        latchFault("VERIFIED_RETURN_NOT_AVAILABLE");
      }
    }
    if (returning_home_ && !return_route_wait_since_.isZero() &&
        (now - return_route_wait_since_).toSec() > return_recovery_timeout_) {
      if (autonomousReturnHealthReady(now)) {
        return_route_wait_since_ = now;
        publishReturnRequest(true);
        publishStatus("RETURN_ROUTE_RETRYING");
        ROS_WARN("Return route is temporarily unavailable; retaining "
                 "OFFBOARD hold and continuing live-map retries");
      } else {
        return_route_wait_since_ = ros::Time();
        latchFault("RETURN_ROUTE_UNAVAILABLE");
      }
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
      const bool cover_outstanding_entry =
          software_enabled_ && auto_enabled_ && mission_enabled_ &&
          !mission_complete_ && !fault_latched_ && mavros_connected_ &&
          px4_mode_ != offboard_mode_ && localPoseFresh(now);
      if (cover_outstanding_entry) {
        // MAVROS acknowledges SetMode before /mavros/state confirms it. Keep
        // the already-frozen, zero-derivative hold streaming while a bounded
        // pre-OFFBOARD health recovery is in progress. If the old request is
        // confirmed late, PX4 still receives a continuous safe reference and
        // the branch above immediately returns it to POSCTL without granting
        // SUPER execution authority.
        publishCheckedTarget(makeFixedHoldTarget(now, false), now);
      } else {
        hold_target_latched_ = false;
      }
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
        (latest_task_position_.z < min_height_ ||
         latest_task_position_.z > max_height_)) {
      // Reject task activation at the pilot's current height. Clamping the
      // first SUPER target would otherwise initiate an unrequested climb or
      // descent before the route has even been accepted. Use the same
      // task1_planning frame as SUPER; PX4 local z includes alignment_z.
      latchFault("START_HEIGHT_OUTSIDE_FENCE");
      return;
    }

    const bool command_fresh =
        valid_command_ && !last_command_time_.isZero() &&
        (now - last_command_time_).toSec() <= command_timeout_;
    if (px4_mode_ != offboard_mode_) {
      // Prestream a hold captured from the aircraft's current pose. Reusing a
      // planner endpoint here made the second 2026-10-14 attempt command the
      // previous goal, 6.1 m away, at the instant OFFBOARD engaged.
      publishCheckedTarget(makeFixedHoldTarget(now, false), now);
      publishReady(false);
      if (require_armed_for_offboard_ && !mavros_armed_) {
        resetPrestream();
        hold_target_latched_ = false;
        publishStatus("PRESTREAM_WAIT_ARMED");
      } else if (!command_fresh) {
        resetPrestream();
        hold_target_latched_ = false;
        publishStatus("PRESTREAM_WAIT_SUPER_COMMAND");
      } else if (latest_measured_speed_ > max_offboard_entry_speed_) {
        if (prestream_wait_started_.isZero()) {
          prestream_wait_started_ = now;
        }
        prestream_started_ = ros::Time();
        // Keep following the actual vehicle while the pilot/POSCTL state is
        // still moving.  The first stationary sample will capture a new hold
        // and begin a complete prestream interval from that exact pose.
        hold_target_latched_ = false;
        if ((now - prestream_wait_started_).toSec() >=
            prestream_stability_timeout_) {
          valid_command_ = false;
          latchFault("PRESTREAM_STABILITY_TIMEOUT");
          return;
        }
        ROS_WARN_THROTTLE(
            0.5,
            "Waiting for stable hover before OFFBOARD: measured speed %.3f "
            "m/s exceeds %.3f m/s (timeout %.1f s)",
            latest_measured_speed_, max_offboard_entry_speed_,
            prestream_stability_timeout_);
        publishStatus("PRESTREAM_WAIT_VEHICLE_STABLE");
      } else if (prestream_started_.isZero()) {
        if (prestream_wait_started_.isZero()) {
          prestream_wait_started_ = now;
        }
        prestream_started_ = now;
        hold_target_latched_ = false;
        publishStatus("PRESTREAM_HOLD");
      } else if ((now - prestream_started_).toSec() < prestream_duration_) {
        publishStatus("PRESTREAM_HOLD");
      } else if (!automatic_mode_switch_) {
        publishStatus("PRESTREAM_WAIT_MANUAL_OFFBOARD");
      } else if (!modeRequestDue(offboard_mode_, now)) {
        // The entry gates were checked immediately before the outstanding
        // OFFBOARD request.  MAVROS acknowledges the service call before the
        // new mode appears on /mavros/state; do not re-run the boundary-speed
        // check on every 50 Hz timer tick during that confirmation interval.
        // If confirmation never arrives, the next actual retry revalidates
        // all entry gates after mode_request_interval_.
        publishStatus("REQUESTING_OFFBOARD");
      } else {
        std::string entry_failure;
        if (!offboardEntrySafe(now, &entry_failure)) {
          valid_command_ = false;
          latchFault(entry_failure);
          return;
        }
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
      // PX4 adds MPC_XY_P/MPC_Z_P times position error to the trajectory
      // feed-forward velocity. Govern that sum before publishing while
      // preserving SUPER's collision-checked position reference. Routine
      // limiting continues the mission; only an infeasible/abnormal tracking
      // error exits OFFBOARD.
      if (!governPredictedTrackingSpeed(&output_target, now)) {
        valid_command_ = false;
        latchFault("SPEED_GOVERNOR_INFEASIBLE");
        return;
      }
      if (!publishCheckedTarget(output_target, now, speed_governor_active_)) return;
      published_source_trajectory_id_=latest_source_trajectory_id_;
      if (hold_requested_ && outputNearRest(output_target)) {
        activateExplorationHold(now);
      }
      publishReady(true);
      publishStatus(speed_governor_active_ ? "SPEED_GOVERNING" :
                    (reacquire_active_ ? "SMOOTH_REACQUIRE" : "STREAMING"));
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
        reason == "OUTPUT_DISCONTINUITY_REJECTED" ||
        reason == "INITIAL_COMMAND_POSITION_JUMP" ||
        reason == "INITIAL_COMMAND_YAW_JUMP" ||
        reason == "COMMAND_POSITION_JUMP" ||
        reason == "COMMAND_VELOCITY_JUMP" ||
        reason == "COMMAND_ACCELERATION_JUMP" ||
        reason == "COMMAND_YAW_JUMP" ||
        reason == "COMMAND_NONMONOTONIC_STAMP" ||
        reason == "YAW_RATE_LIMIT";
    if (returnable_command_fault &&
        auto_enabled_ && mission_enabled_ && offboard_owned_ &&
        px4_mode_ == offboard_mode_ && alignment_ready_ && vision_healthy_ &&
        localPoseFresh(ros::Time::now())) {
      recovery_attempted_ = true;
      if (!recovery_pending_) {
        recovery_pending_ = true;
        recovery_started_ = ros::Time::now();
        recovery_source_trajectory_id_ = latest_source_trajectory_id_;
        valid_command_ = false;
        managed_hold_reacquire_pending_ =
            have_published_target_ && px4_mode_ == offboard_mode_;
        resetReacquire();
        resetContinuityGuard();
      }
      publishReturnRequest(true);
      publishReady(false);
      publishStatus("REQUESTING_VERIFIED_RETURN: " + reason);
      ROS_WARN("Command fault %s: braking and requesting verified outbound retrace",
               reason.c_str());
      return;
    }
    if (auto_enabled_ && mission_enabled_) {
      // MissionScheduler deliberately revokes task authority on this topic.
      // Publish only after autonomous in-OFFBOARD return has been ruled out;
      // publishing before the branch above turned recoverable command gaps
      // into an immediate task cancellation.
      publishFault(reason);
    }
    if (auto_enabled_ && !fault_latched_) {
      ROS_ERROR("Task-one automatic control fault latched: %s", reason.c_str());
      fault_latched_ = true;
      beginOffboardExit(reason);
    }
    publishExecutionAuthorization();
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
    managed_hold_reacquire_pending_ = false;
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
    if (localPoseFresh(now) &&
        !publishCheckedTarget(makeFixedHoldTarget(now, true), now)) return;
    requestMode(fallback_mode_, now, false);
    publishStatus("EXITING_OFFBOARD_TO_" + fallback_mode_);
  }

  void requestMode(const std::string& mode, const ros::Time& now,
                   bool entering_offboard) {
    // A safety exit must not wait behind the throttle for the immediately
    // preceding OFFBOARD-entry request. Throttle only repeated requests for
    // the same mode.
    if (!modeRequestDue(mode, now)) {
      return;
    }
    last_mode_request_time_ = now;
    last_requested_mode_ = mode;
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

  bool modeRequestDue(const std::string& mode, const ros::Time& now) const {
    return mode != last_requested_mode_ || last_mode_request_time_.isZero() ||
           (now - last_mode_request_time_).toSec() >= mode_request_interval_;
  }

  void resetPrestream() {
    prestream_started_ = ros::Time();
    prestream_wait_started_ = ros::Time();
  }

  void updateStatus() {
    publishExecutionAuthorization();
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
      publishStatus(px4_mode_ == offboard_mode_ ? "WAIT_VISION" :
                    "PRESTREAM_WAIT_VISION_RECOVERY");
    } else if (!mavros_connected_) {
      publishStatus("WAIT_MAVROS");
    } else if (!localPoseFresh(ros::Time::now())) {
      publishStatus("WAIT_LOCAL_POSE");
    } else if (!localVelocityFresh(ros::Time::now())) {
      publishStatus("WAIT_LOCAL_VELOCITY");
    } else if (obstacle_safety_enabled_ &&
               !obstacleCloudFresh(ros::Time::now())) {
      publishStatus("WAIT_OBSTACLE_CLOUD");
    } else if (obstacle_monitor_.active()) {
      publishStatus("OBSTACLE_MONITORING_HOLD:" + obstacle_monitor_reason_);
    } else if (obstacle_safety_enabled_ && !obstacle_safety_ready_) {
      publishStatus("WAIT_OBSTACLE_SURFACE_COVERAGE");
    } else if (fault_latched_) {
      publishStatus("FAULT_LATCHED_TOGGLE_AUTO_LOW");
    } else if (require_armed_for_offboard_ && !mavros_armed_) {
      publishStatus("PRESTREAM_WAIT_ARMED");
    } else if (px4_mode_ != offboard_mode_) {
      publishStatus("PRESTREAM");
    } else if (!valid_command_) {
      publishStatus("WAIT_SUPER_COMMAND");
    } else if (speed_governor_active_) {
      publishStatus("SPEED_GOVERNING");
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

  void publishExecutionAuthorization() {
    const bool authorized = gatesReady() && !exit_requested_ &&
        offboard_owned_ && px4_mode_ == offboard_mode_;
    if (execution_authorization_published_ &&
        authorized == last_execution_authorization_) {
      return;
    }
    std_msgs::Bool message;
    message.data = authorized;
    execution_authorization_publisher_.publish(message);
    last_execution_authorization_ = authorized;
    execution_authorization_published_ = true;
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

  void publishFault(const std::string& reason) {
    if (!reason.empty() && !active_fault_reason_.empty()) {
      return;
    }
    if (fault_published_ && reason == active_fault_reason_) {
      return;
    }
    std_msgs::String message;
    message.data = reason;
    fault_publisher_.publish(message);
    active_fault_reason_ = reason;
    fault_published_ = true;
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber command_subscriber_;
  ros::Subscriber alignment_subscriber_;
  ros::Subscriber sensor_position_subscriber_;
  ros::Subscriber mission_subscriber_;
  ros::Subscriber auto_enable_subscriber_;
  ros::Subscriber vision_subscriber_;
  ros::Subscriber finished_subscriber_;
  ros::Subscriber returning_subscriber_;
  ros::Subscriber exploration_status_subscriber_;
  ros::Subscriber state_subscriber_;
  ros::Subscriber local_pose_subscriber_;
  ros::Subscriber local_velocity_subscriber_;
  ros::Subscriber task_odometry_subscriber_;
  ros::Subscriber obstacle_cloud_subscriber_;
  ros::Publisher command_publisher_;
  ros::Publisher ready_publisher_;
  ros::Publisher status_publisher_;
  ros::Publisher fault_publisher_;
  ros::Publisher return_request_publisher_;
  ros::Publisher obstacle_replan_publisher_;
  ros::Publisher execution_authorization_publisher_;
  ros::ServiceServer enable_service_;
  ros::ServiceClient set_mode_client_;
  ros::Timer output_timer_;

  geometry_msgs::PoseStamped latest_local_pose_;
  mine_uav_control::ObstaclePoint latest_task_position_;
  mine_uav_control::TransientObstacleMonitor obstacle_monitor_;
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
  std::string alignment_topic_;
  std::string sensor_position_topic_;
  std::string expected_frame_;
  std::string auto_enable_topic_;
  std::string mission_enable_topic_;
  std::string exploration_status_topic_;
  std::string obstacle_cloud_topic_;
  std::string task_odometry_topic_;
  std::string obstacle_replan_topic_;
  std::string execution_authorization_topic_;
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
  std::string active_fault_reason_;
  std::string obstacle_monitor_reason_;
  std::string last_requested_mode_;
  ros::Time last_command_time_;
  ros::Time latest_source_stamp_;
  uint32_t latest_source_trajectory_id_{0};
  uint32_t published_source_trajectory_id_{0};
  ros::Time last_local_pose_time_;
  ros::Time last_local_velocity_time_;
  ros::Time last_task_odometry_time_;
  ros::Time last_obstacle_cloud_time_;
  ros::Time obstacle_replan_escape_started_;
  ros::Time overspeed_since_;
  ros::Time prestream_started_;
  ros::Time prestream_wait_started_;
  ros::Time preoffboard_vision_unhealthy_since_;
  ros::Time last_mode_request_time_;
  ros::Time previous_target_time_;
  ros::Time reacquire_start_time_;
  ros::Time last_speed_governor_update_time_;
  double command_timeout_{0.25};
  double local_pose_timeout_{0.5};
  double output_rate_{50.0};
  double max_speed_{1.0};
  double max_measured_speed_{1.0};
  double max_predicted_speed_{1.0};
  double measured_speed_target_{0.65};
  double measured_speed_feedback_gain_{1.0};
  double measured_speed_governor_floor_{0.35};
  double measured_speed_governor_reduction_rate_{0.80};
  double measured_speed_governor_recovery_rate_{0.50};
  double adaptive_predicted_speed_limit_{1.0};
  double tracking_position_gain_xy_{0.9};
  double tracking_position_gain_z_{1.0};
  double max_governor_tracking_error_{0.65};
  double measured_speed_timeout_{0.5};
  double overspeed_duration_{1.5};
  double max_offboard_entry_speed_{0.20};
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
  double max_managed_reacquire_distance_{1.5};
  double max_managed_reacquire_yaw_error_{2.6};
  // Optional scenario fence; zero means no home-centred distance limit.
  double max_horizontal_radius_{0.0};
  // SUPER task1_planning-frame limits. They are checked before applying the
  // per-flight planning-to-PX4 alignment transform.
  double min_height_{-5.0};
  double max_height_{8.0};
  double height_clamp_tolerance_{0.25};
  double max_alignment_translation_change_{0.05};
  double max_alignment_rotation_change_{0.05};
  double alignment_x_{0.0};
  double alignment_y_{0.0};
  double alignment_z_{0.0};
  mine_uav_control::Quaternion alignment_rotation_;
  mine_uav_control::Vector3 sensor_position_in_body_;
  double prestream_duration_{1.0};
  double prestream_stability_timeout_{15.0};
  double preoffboard_vision_recovery_timeout_{2.0};
  double mode_request_interval_{1.0};
  double obstacle_cloud_timeout_{0.35};
  double obstacle_cloud_fault_timeout_{2.0};
  double live_horizontal_clearance_{0.65};
  double obstacle_monitor_horizontal_clearance_{0.8};
  double obstacle_persistence_duration_{3.0};
  double obstacle_replan_escape_duration_{4.0};
  double live_vertical_clearance_below_{0.5};
  double live_vertical_clearance_above_{0.5};
  double obstacle_min_z_{-1.0e6};
  double coverage_guard_distance_{2.5};
  double coverage_lateral_half_width_{1.0};
  double coverage_vertical_uncertainty_margin_{0.2};
  double coverage_min_horizontal_span_{0.6};
  double coverage_min_vertical_span_{0.0};
  double latest_planning_velocity_x_{1.0};
  double latest_planning_velocity_y_{0.0};
  double latest_planning_position_x_{0.0};
  double latest_planning_position_y_{0.0};
  double last_planning_forward_x_{1.0};
  double last_planning_forward_y_{0.0};
  int coverage_min_points_{30};
  int coverage_confirm_frames_{3};
  int coverage_reject_frames_{2};
  int max_obstacle_points_{20000};
  int obstacle_clear_confirm_frames_{3};
  bool use_acceleration_{true};
  bool automatic_mode_switch_{true};
  bool require_armed_for_offboard_{true};
  bool alignment_ready_{false};
  bool have_sensor_position_{false};
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
  bool fault_published_{false};
  bool offboard_owned_{false};
  bool execution_authorization_published_{false};
  bool last_execution_authorization_{false};
  bool exit_requested_{false};
  bool valid_command_{false};
  bool have_local_pose_{false};
  bool have_local_velocity_{false};
  bool have_task_odometry_{false};
  bool have_obstacle_cloud_{false};
  bool obstacle_safety_enabled_{false};
  bool obstacle_safety_ready_{true};
  bool obstacle_hazard_stop_active_{false};
  bool cloud_gap_hold_active_{false};
  bool obstacle_replan_requested_{false};
  bool obstacle_replan_escape_active_{false};
  bool obstacle_replan_requires_away_motion_{false};
  mine_uav_control::ObstaclePoint latest_monitor_obstacle_;
  bool have_latest_monitor_obstacle_{false};
  uint32_t obstacle_hold_source_trajectory_id_{0};
  int obstacle_coverage_complete_streak_{0};
  int obstacle_coverage_incomplete_streak_{0};
  bool obstacle_surface_coverage_ready_{true};
  bool speed_governor_active_{false};
  double latest_measured_speed_{0.0};
  mavros_msgs::PositionTarget published_target_;
  ros::Time published_target_time_;
  bool published_target_feedforward_governed_{false};
  bool have_published_target_{false};
  bool have_previous_target_{false};
  bool hold_target_latched_{false};
  bool reacquire_active_{false};
  bool managed_hold_reacquire_pending_{false};
  bool source_stream_active_{false};
  double active_reacquire_duration_{3.5};
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
