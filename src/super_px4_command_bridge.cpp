#include <algorithm>
#include <cmath>
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
    private_nh_.param("software_enable_default", software_enabled_, true);

    command_timeout_ = std::max(0.05, command_timeout_);
    local_pose_timeout_ = std::max(0.1, local_pose_timeout_);
    output_rate_ = std::max(10.0, output_rate_);
    max_speed_ = std::max(0.1, max_speed_);
    max_acceleration_ = std::max(0.1, max_acceleration_);
    max_horizontal_radius_ = std::max(1.0, max_horizontal_radius_);
    max_alignment_translation_change_ =
        std::max(0.001, max_alignment_translation_change_);
    max_alignment_yaw_change_ = std::max(0.001, max_alignment_yaw_change_);
    prestream_duration_ = std::max(1.0, prestream_duration_);
    mode_request_interval_ = std::max(0.5, mode_request_interval_);
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
    enable_service_ = private_nh_.advertiseService(
        "enable", &SuperPx4CommandBridge::enableCallback, this);
    set_mode_client_ =
        nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    output_timer_ = nh_.createTimer(
        ros::Duration(1.0 / output_rate_),
        &SuperPx4CommandBridge::outputTimerCallback, this);

    publishReady(false);
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
      beginOffboardExit("TASK1_DESELECTED");
      resetPrestream();
    }
    updateStatus();
  }

  void autoEnableCallback(const std_msgs::Bool::ConstPtr& enabled) {
    const bool was_enabled = auto_enabled_;
    auto_enabled_ = enabled->data;
    if (was_enabled && !auto_enabled_) {
      beginOffboardExit("AUTO_DISABLED");
      fault_latched_ = false;
      resetPrestream();
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
      beginOffboardExit("TASK1_COMPLETE");
      resetPrestream();
    }
    updateStatus();
  }

  void explorationStatusCallback(
      const std_msgs::String::ConstPtr& status) {
    const std::string state = status->data.substr(
        0, status->data.find(':'));
    const bool should_hold =
        state == "WAIT_DATA" || state == "WAIT_GOAL" ||
        state == "WAIT_FRONTIER" || state == "WAIT_MAP_CLOSURE" ||
        state == "WAIT_MODEL_COVERAGE" || state == "DISABLED";
    exploration_status_hold_ = should_hold;
    if (should_hold) {
      valid_command_ = false;
      if (px4_mode_ == offboard_mode_ && !hold_target_latched_ &&
          localPoseFresh(ros::Time::now())) {
        latched_hold_target_ = makeHoldTarget(ros::Time::now());
        hold_target_latched_ = true;
      }
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
      hold_target_latched_ = false;
    }
    if (mine_uav_control::externalOffboardExit(
            previous_mode, px4_mode_, offboard_mode_, offboard_owned_,
            exit_requested_)) {
      // CH5/manual takeover and PX4 failsafe exits both revoke this
      // bridge's permission to request OFFBOARD again. A deliberate
      // auto-enable low->high reset is required before the next attempt.
      latchFault("OFFBOARD_EXITED_EXTERNALLY");
      valid_command_ = false;
      resetPrestream();
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
    last_command_time_ = ros::Time::now();
    std::string reason;
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
    valid_command_ = true;
    hold_target_latched_ = false;
    latest_target_ = bounded_output;
    updateStatus();
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

  mavros_msgs::PositionTarget makeFixedHoldTarget(const ros::Time& now) {
    if (!hold_target_latched_) {
      latched_hold_target_ = makeHoldTarget(now);
      hold_target_latched_ = true;
    }
    auto hold = latched_hold_target_;
    hold.header.stamp = now;
    return hold;
  }

  void outputTimerCallback(const ros::TimerEvent&) {
    const ros::Time now = ros::Time::now();
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

    const bool command_fresh =
        valid_command_ && !last_command_time_.isZero() &&
        (now - last_command_time_).toSec() <= command_timeout_;
    if (prestream_started_.isZero()) {
      prestream_started_ = now;
    }

    if (px4_mode_ != offboard_mode_) {
      hold_target_latched_ = false;
      command_publisher_.publish(makeHoldTarget(now));
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
      latest_target_.header.stamp = now;
      command_publisher_.publish(latest_target_);
      publishReady(true);
      publishStatus("STREAMING");
    } else {
      valid_command_ = false;
      command_publisher_.publish(makeFixedHoldTarget(now));
      publishReady(true);
      publishStatus("HOLD_COMMAND_TIMEOUT");
    }
  }

  void latchFault(const std::string& reason) {
    if (auto_enabled_ && !fault_latched_) {
      ROS_ERROR("Task-one automatic control fault latched: %s", reason.c_str());
      fault_latched_ = true;
      beginOffboardExit(reason);
    }
    publishReady(false);
    publishStatus(reason);
  }

  void beginOffboardExit(const std::string& reason) {
    hold_target_latched_ = false;
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
    if (localPoseFresh(now)) {
      command_publisher_.publish(makeHoldTarget(now));
    }
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
    ROS_INFO("PX4 mode request accepted: %s", mode.c_str());
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
  ros::Subscriber exploration_status_subscriber_;
  ros::Subscriber state_subscriber_;
  ros::Subscriber local_pose_subscriber_;
  ros::Publisher command_publisher_;
  ros::Publisher ready_publisher_;
  ros::Publisher status_publisher_;
  ros::ServiceServer enable_service_;
  ros::ServiceClient set_mode_client_;
  ros::Timer output_timer_;

  geometry_msgs::PoseStamped latest_local_pose_;
  mavros_msgs::PositionTarget latest_target_;
  mavros_msgs::PositionTarget latched_hold_target_;
  std::string command_topic_;
  std::string output_topic_;
  std::string expected_frame_;
  std::string auto_enable_topic_;
  std::string exploration_status_topic_;
  std::string offboard_mode_{"OFFBOARD"};
  std::string fallback_mode_{"POSCTL"};
  std::string px4_mode_;
  std::string last_status_;
  ros::Time last_command_time_;
  ros::Time last_local_pose_time_;
  ros::Time prestream_started_;
  ros::Time last_mode_request_time_;
  double command_timeout_{0.25};
  double local_pose_timeout_{0.5};
  double output_rate_{50.0};
  double max_speed_{1.0};
  double max_acceleration_{3.0};
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
  bool mavros_connected_{false};
  bool mavros_armed_{false};
  bool software_enabled_{true};
  bool fault_latched_{false};
  bool offboard_owned_{false};
  bool exit_requested_{false};
  bool valid_command_{false};
  bool have_local_pose_{false};
  bool hold_target_latched_{false};
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
