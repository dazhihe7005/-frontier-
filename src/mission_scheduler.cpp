#include "mine_uav_control/mission_scheduler.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>

namespace mine_uav_control {

MissionScheduler::MissionScheduler(const ros::NodeHandle& nh,
                                   const ros::NodeHandle& private_nh)
    : nh_(nh), private_nh_(private_nh) {
  private_nh_.param("rc_topic", rc_topic_, std::string("/mavros/rc/in"));
  private_nh_.param("odometry_topic", odometry_topic_,
                    std::string("/mine_uav/task1/odometry"));
  private_nh_.param("alignment_topic", alignment_topic_,
                    std::string("/mine_uav/task1/planning_to_px4_alignment"));
  private_nh_.param("expected_planning_frame", expected_planning_frame_,
                    std::string("task1_planning"));
  private_nh_.param("mavros_state_topic", mavros_state_topic_,
                    std::string("/mavros/state"));
  private_nh_.param("command_fault_topic", command_fault_topic_,
                    std::string("/mine_uav/task1/command_fault"));
  private_nh_.param("initial_goal_accepted_topic",
                    initial_goal_accepted_topic_,
                    std::string("/mine_uav/exploration/accepted_initial_goal_id"));
  private_nh_.param("goal_command_topic", goal_command_topic_,
                    std::string("/mine_uav/super/goal_command"));
  private_nh_.param("super_goal_accepted_topic", super_goal_accepted_topic_,
                    std::string("/mine_uav/super/accepted_goal_id"));
  private_nh_.param("active_task_topic", active_task_topic_,
                    std::string("/mine_uav/mission/active_task"));
  private_nh_.param("auto_enable_topic", auto_enable_topic_,
                    std::string("/mine_uav/mission/auto_enable"));
  private_nh_.param("goaf_enable_topic", goaf_enable_topic_,
                    std::string("/mine_uav/mission/goaf_enable"));
  private_nh_.param("shaft_enable_topic", shaft_enable_topic_,
                    std::string("/mine_uav/mission/shaft_enable"));
  private_nh_.param("shaft_command_ready_topic", shaft_command_ready_topic_,
                    std::string("/mine_uav/task2/command_ready"));
  private_nh_.param("task3_enable_topic", task3_enable_topic_,
                    std::string("/mine_uav/mission/task3_enable"));
  private_nh_.param("task3_command_ready_topic", task3_command_ready_topic_,
                    std::string("/mine_uav/task3/command_ready"));
  private_nh_.param("tunnel_enable_topic", tunnel_enable_topic_,
                    std::string("/mine_uav/mission/tunnel_enable"));
  private_nh_.param("exploration_enable_topic", exploration_enable_topic_,
                    std::string("/mine_uav/mission/exploration_enable"));
  private_nh_.param("return_home_topic", return_home_topic_,
                    std::string("/mine_uav/mission/return_home"));
  private_nh_.param("switch_event_topic", switch_event_topic_,
                    std::string("/mine_uav/mission/switch_event"));
  private_nh_.param("status_topic", status_topic_,
                    std::string("/mine_uav/mission/status"));

  private_nh_.param("goaf_trigger_channel", goaf_trigger_channel_,
                    goaf_trigger_channel_);
  private_nh_.param("shaft_trigger_channel", shaft_trigger_channel_,
                    shaft_trigger_channel_);
  private_nh_.param("task3_trigger_channel", task3_trigger_channel_,
                    task3_trigger_channel_);
  private_nh_.param("tunnel_trigger_channel", tunnel_trigger_channel_,
                    tunnel_trigger_channel_);
  private_nh_.param("low_threshold", low_threshold_, low_threshold_);
  private_nh_.param("high_threshold", high_threshold_, high_threshold_);
  private_nh_.param("switch_stable_time", switch_stable_time_, switch_stable_time_);
  private_nh_.param("rc_timeout", rc_timeout_, rc_timeout_);
  private_nh_.param("odometry_timeout", odometry_timeout_, odometry_timeout_);
  private_nh_.param("mavros_state_timeout", mavros_state_timeout_,
                    mavros_state_timeout_);
  private_nh_.param("shaft_status_timeout", shaft_status_timeout_,
                    shaft_status_timeout_);
  private_nh_.param("shaft_start_timeout", shaft_start_timeout_,
                    shaft_start_timeout_);
  private_nh_.param("shaft_ready_timeout", shaft_ready_timeout_,
                    shaft_ready_timeout_);
  private_nh_.param("task3_ready_timeout", task3_ready_timeout_,
                    task3_ready_timeout_);
  private_nh_.param("task3_start_timeout", task3_start_timeout_,
                    task3_start_timeout_);
  private_nh_.param("decision_rate", decision_rate_, decision_rate_);
  private_nh_.param("start_max_speed", start_max_speed_, start_max_speed_);
  private_nh_.param("start_max_vertical_speed", start_max_vertical_speed_,
                    start_max_vertical_speed_);
  private_nh_.param("start_stable_time", start_stable_time_,
                    start_stable_time_);
  private_nh_.param("start_check_timeout", start_check_timeout_,
                    start_check_timeout_);
  private_nh_.param("initial_goal_accept_timeout",
                    initial_goal_accept_timeout_,
                    initial_goal_accept_timeout_);
  private_nh_.param("require_odometry", require_odometry_, require_odometry_);
  private_nh_.param("require_mavros_connection", require_mavros_connection_,
                    require_mavros_connection_);
  private_nh_.param("shaft_task_available", shaft_task_available_,
                    shaft_task_available_);
  private_nh_.param("task3_task_available", task3_task_available_,
                    task3_task_available_);
  private_nh_.param("tunnel_task_available", tunnel_task_available_,
                    tunnel_task_available_);

  if (goaf_trigger_channel_ < 0 || shaft_trigger_channel_ < 0 ||
      goaf_trigger_channel_ == shaft_trigger_channel_) {
    ROS_WARN("Task trigger channels must be distinct and non-negative; using CH9/CH11");
    goaf_trigger_channel_ = 8;
    shaft_trigger_channel_ = 10;
  }
  if (task3_trigger_channel_ >= 0 &&
      (task3_trigger_channel_ == goaf_trigger_channel_ ||
       task3_trigger_channel_ == shaft_trigger_channel_)) {
    ROS_WARN("Task 3 trigger conflicts with another task; leaving it unassigned");
    task3_trigger_channel_ = -1;
  }
  if (tunnel_trigger_channel_ >= 0 &&
      (tunnel_trigger_channel_ == goaf_trigger_channel_ ||
       tunnel_trigger_channel_ == shaft_trigger_channel_ ||
       tunnel_trigger_channel_ == task3_trigger_channel_)) {
    ROS_WARN("Task 4 trigger conflicts with another task; leaving it unassigned");
    tunnel_trigger_channel_ = -1;
  }
  if (tunnel_task_available_ && tunnel_trigger_channel_ < 0) {
    ROS_WARN("Task 4 requested without an assigned RC trigger; disabling it");
    tunnel_task_available_ = false;
  }
  if (low_threshold_ >= high_threshold_) {
    ROS_WARN("low_threshold must be below high_threshold; using 1300/1700");
    low_threshold_ = 1300;
    high_threshold_ = 1700;
  }
  if (!std::isfinite(switch_stable_time_) || switch_stable_time_ < 0.0) {
    switch_stable_time_ = 0.5;
  }
  if (!std::isfinite(rc_timeout_) || rc_timeout_ <= 0.0) {
    rc_timeout_ = 1.0;
  }
  if (!std::isfinite(odometry_timeout_) || odometry_timeout_ <= 0.0) {
    odometry_timeout_ = 1.0;
  }
  if (!std::isfinite(mavros_state_timeout_) || mavros_state_timeout_ <= 0.0) {
    mavros_state_timeout_ = 2.5;
  }
  if (!std::isfinite(shaft_status_timeout_) || shaft_status_timeout_ <= 0.0) {
    shaft_status_timeout_ = 0.6;
  }
  if (!std::isfinite(shaft_start_timeout_) ||
      shaft_start_timeout_ <= shaft_status_timeout_) {
    shaft_start_timeout_ = std::max(4.0, shaft_status_timeout_ + 1.0);
  }
  if (!std::isfinite(shaft_ready_timeout_) || shaft_ready_timeout_ <= 0.0) {
    shaft_ready_timeout_ = 0.6;
  }
  if (!std::isfinite(task3_ready_timeout_) || task3_ready_timeout_ <= 0.0) {
    task3_ready_timeout_ = 0.6;
  }
  if (!std::isfinite(task3_start_timeout_) ||
      task3_start_timeout_ <= task3_ready_timeout_) {
    task3_start_timeout_ = std::max(4.0, task3_ready_timeout_ + 1.0);
  }
  if (!std::isfinite(decision_rate_) || decision_rate_ <= 0.0) {
    decision_rate_ = 10.0;
  }
  start_max_speed_ = std::max(0.02, start_max_speed_);
  start_max_vertical_speed_ = std::max(0.01, start_max_vertical_speed_);
  start_stable_time_ = std::max(0.5, start_stable_time_);
  start_check_timeout_ = std::max(start_stable_time_, start_check_timeout_);
  initial_goal_accept_timeout_ = std::max(5.0, initial_goal_accept_timeout_);

  rc_subscriber_ = nh_.subscribe(rc_topic_, 10, &MissionScheduler::rcCallback, this);
  odometry_subscriber_ = nh_.subscribe(
      odometry_topic_, 10, &MissionScheduler::odometryCallback, this);
  alignment_subscriber_ = nh_.subscribe(
      alignment_topic_, 1, &MissionScheduler::alignmentCallback, this);
  mavros_state_subscriber_ = nh_.subscribe(
      mavros_state_topic_, 10, &MissionScheduler::mavrosStateCallback, this);
  command_fault_subscriber_ = nh_.subscribe(
      command_fault_topic_, 1, &MissionScheduler::commandFaultCallback, this);
  initial_goal_accepted_subscriber_ = nh_.subscribe(
      initial_goal_accepted_topic_, 1,
      &MissionScheduler::initialGoalAcceptedCallback, this);
  // The exploration callback also relays this acknowledgement, but its map
  // update runs on one CPU-heavy callback queue. Observe SUPER's lifecycle
  // handshake directly so a valid initial goal cannot wait behind a large
  // point cloud for several seconds.
  goal_command_subscriber_ = nh_.subscribe(
      goal_command_topic_, 10, &MissionScheduler::goalCommandCallback, this);
  super_goal_accepted_subscriber_ = nh_.subscribe(
      super_goal_accepted_topic_, 10,
      &MissionScheduler::superGoalAcceptedCallback, this);
  goaf_finished_subscriber_ = nh_.subscribe(
      "/mine_uav/exploration/finished", 1,
      &MissionScheduler::goafFinishedCallback, this);
  shaft_status_subscriber_ = nh_.subscribe(
      "/mine_uav/shaft/status", 1,
      &MissionScheduler::shaftStatusCallback, this);
  shaft_command_ready_subscriber_ = nh_.subscribe(
      shaft_command_ready_topic_, 1,
      &MissionScheduler::shaftCommandReadyCallback, this);
  task3_command_ready_subscriber_ = nh_.subscribe(
      task3_command_ready_topic_, 1,
      &MissionScheduler::task3CommandReadyCallback, this);

  // Latched outputs let a task node receive the current selection immediately.
  active_task_publisher_ = nh_.advertise<std_msgs::UInt8>(active_task_topic_, 1, true);
  auto_enable_publisher_ =
      nh_.advertise<std_msgs::Bool>(auto_enable_topic_, 1, true);
  goaf_enable_publisher_ = nh_.advertise<std_msgs::Bool>(goaf_enable_topic_, 1, true);
  shaft_enable_publisher_ = nh_.advertise<std_msgs::Bool>(shaft_enable_topic_, 1, true);
  task3_enable_publisher_ = nh_.advertise<std_msgs::Bool>(task3_enable_topic_, 1, true);
  tunnel_enable_publisher_ =
      nh_.advertise<std_msgs::Bool>(tunnel_enable_topic_, 1, true);
  exploration_enable_publisher_ =
      nh_.advertise<std_msgs::Bool>(exploration_enable_topic_, 1, true);
  return_home_publisher_ = nh_.advertise<std_msgs::Bool>(return_home_topic_, 1, true);
  switch_event_publisher_ = nh_.advertise<std_msgs::String>(switch_event_topic_, 10, true);
  status_publisher_ = nh_.advertise<std_msgs::String>(status_topic_, 10);

  timer_ = nh_.createTimer(ros::Duration(1.0 / decision_rate_),
                           &MissionScheduler::timerCallback, this);
  publishOutputs("startup");

  ROS_INFO("mission_scheduler ready: goaf trigger CH%d, shaft trigger CH%d, "
           "Task 3 trigger %s, Task 4 trigger %s; "
           "Tasks 1/4 require a debounced edge, stable hover, and an accepted initial goal; "
           "odometry required=%s, shaft available=%s, Task 3 available=%s, "
           "Task 4 available=%s",
           goaf_trigger_channel_ + 1, shaft_trigger_channel_ + 1,
           task3_trigger_channel_ >= 0
               ? ("CH" + std::to_string(task3_trigger_channel_ + 1)).c_str()
               : "UNASSIGNED",
           tunnel_trigger_channel_ >= 0
               ? ("CH" + std::to_string(tunnel_trigger_channel_ + 1)).c_str()
               : "UNASSIGNED",
           require_odometry_ ? "true" : "false",
           shaft_task_available_ ? "true" : "false",
           task3_task_available_ ? "true" : "false",
           tunnel_task_available_ ? "true" : "false");
}

void MissionScheduler::rcCallback(const mavros_msgs::RCIn::ConstPtr& message) {
  last_rc_time_ = ros::Time::now();
  have_rc_ = true;

  goaf_channel_available_ =
      goaf_trigger_channel_ < static_cast<int>(message->channels.size());
  shaft_channel_available_ =
      shaft_trigger_channel_ < static_cast<int>(message->channels.size());
  task3_channel_available_ = task3_trigger_channel_ >= 0 &&
      task3_trigger_channel_ < static_cast<int>(message->channels.size());
  tunnel_channel_available_ = tunnel_trigger_channel_ >= 0 &&
      tunnel_trigger_channel_ < static_cast<int>(message->channels.size());
  goaf_rc_value_ = goaf_channel_available_
                       ? message->channels[goaf_trigger_channel_] : 0;
  shaft_rc_value_ = shaft_channel_available_
                        ? message->channels[shaft_trigger_channel_] : 0;
  task3_rc_value_ = task3_channel_available_
                        ? message->channels[task3_trigger_channel_] : 0;
  tunnel_rc_value_ = tunnel_channel_available_
                         ? message->channels[tunnel_trigger_channel_] : 0;
  if (!goaf_channel_available_ || !shaft_channel_available_) {
    ROS_WARN_THROTTLE(2.0, "CH9/CH11 trigger channels unavailable: received %zu channels",
                      message->channels.size());
  }
}

void MissionScheduler::task3CommandReadyCallback(
    const std_msgs::Bool::ConstPtr& message) {
  last_task3_command_ready_time_ = ros::Time::now();
  have_task3_command_ready_ = true;
  task3_command_ready_ = message->data;
  if (active_task_ == Task::kPressureControl && task3_command_ready_) {
    task3_started_ = true;
  }
}

void MissionScheduler::odometryCallback(
    const nav_msgs::Odometry::ConstPtr& message) {
  const auto& position = message->pose.pose.position;
  const auto& orientation = message->pose.pose.orientation;
  const bool finite_pose = std::isfinite(position.x) &&
      std::isfinite(position.y) && std::isfinite(position.z) &&
      std::isfinite(orientation.x) && std::isfinite(orientation.y) &&
      std::isfinite(orientation.z) && std::isfinite(orientation.w);
  if (!finite_pose) {
    odometry_frame_valid_ = false;
    return;
  }
  if (!expected_planning_frame_.empty() &&
      message->header.frame_id != expected_planning_frame_) {
    odometry_frame_valid_ = false;
    ROS_ERROR_THROTTLE(
        1.0, "Mission scheduler rejected odometry frame '%s'; expected '%s'",
        message->header.frame_id.c_str(), expected_planning_frame_.c_str());
    return;
  }

  const ros::Time now = ros::Time::now();
  if (have_odometry_) {
    const double dt = ageSec(last_odometry_time_, now);
    if (dt >= 0.01 && dt <= odometry_timeout_) {
      const auto& previous = latest_odometry_.pose.pose.position;
      const double dx = position.x - previous.x;
      const double dy = position.y - previous.y;
      const double dz = position.z - previous.z;
      const double sample_speed = std::sqrt(dx * dx + dy * dy + dz * dz) / dt;
      const double sample_vertical_speed = std::abs(dz) / dt;
      estimated_speed_ = have_speed_estimate_
          ? 0.25 * sample_speed + 0.75 * estimated_speed_
          : sample_speed;
      estimated_vertical_speed_ = have_speed_estimate_
          ? 0.25 * sample_vertical_speed + 0.75 * estimated_vertical_speed_
          : sample_vertical_speed;
      have_speed_estimate_ = true;
    }
  }
  latest_odometry_ = *message;
  last_odometry_time_ = now;
  have_odometry_ = true;
  odometry_frame_valid_ = true;
}

void MissionScheduler::alignmentCallback(
    const geometry_msgs::TransformStamped::ConstPtr& message) {
  const auto& translation = message->transform.translation;
  const auto& q = message->transform.rotation;
  const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  if (!std::isfinite(translation.x) || !std::isfinite(translation.y) ||
      !std::isfinite(translation.z) || !std::isfinite(norm) || norm < 1e-6 ||
      (!expected_planning_frame_.empty() &&
       message->child_frame_id != expected_planning_frame_)) {
    alignment_ready_ = false;
    return;
  }
  const double qx = q.x / norm;
  const double qy = q.y / norm;
  const double up_z = 1.0 - 2.0 * (qx * qx + qy * qy);
  const double tilt = std::acos(std::max(-1.0, std::min(1.0, up_z)));
  // A startup yaw offset is expected: the planner starts at yaw zero while
  // PX4 keeps its established heading. Only roll/pitch would make the
  // planning frame non-level and must keep the start gate closed.
  alignment_ready_ = tilt <= 1e-3;
  if (!alignment_ready_) {
    ROS_ERROR_THROTTLE(
        1.0, "Mission scheduler requires a level planning-to-PX4 alignment "
             "(yaw is allowed)");
  }
}

void MissionScheduler::mavrosStateCallback(const mavros_msgs::State::ConstPtr& message) {
  last_mavros_state_time_ = ros::Time::now();
  const bool manual_mode = message->mode == "MANUAL" ||
                           message->mode == "POSCTL" ||
                           message->mode == "ALTCTL" ||
                           message->mode == "STABILIZED";
  if (active_task_ != Task::kHold && offboard_seen_for_task_ &&
      px4_mode_ == "OFFBOARD" && message->mode != "OFFBOARD") {
    external_mode_exit_pending_ = true;
  } else if (active_task_ != Task::kHold && !offboard_seen_for_task_ &&
             !px4_mode_.empty() && px4_mode_ != message->mode && manual_mode) {
    // The pilot can abort during prestream, before this task ever owns OFFBOARD.
    external_mode_exit_pending_ = true;
  }
  have_mavros_state_ = true;
  mavros_connected_ = message->connected;
  mavros_armed_ = message->armed;
  px4_mode_ = message->mode;
  if (active_task_ != Task::kHold && px4_mode_ == "OFFBOARD") {
    offboard_seen_for_task_ = true;
  }
}

void MissionScheduler::commandFaultCallback(
    const std_msgs::String::ConstPtr& message) {
  command_fault_ = message->data;
  if (isExplorationTask(active_task_) && !command_fault_.empty()) {
    // Close both planning and control authority in this callback. Waiting for
    // the next scheduler tick would leave a needless permission window and
    // could race a simultaneous initial-goal acknowledgement.
    const std::string prefix =
        active_task_ == Task::kTunnelExploration ? "task4" : "task1";
    applyTask(Task::kHold, prefix + "_command_fault_" + command_fault_, false);
  }
}

void MissionScheduler::initialGoalAcceptedCallback(
    const std_msgs::UInt64::ConstPtr& message) {
  if (message->data == 0) {
    initial_goal_accepted_id_ = 0;
    task_ready_reset_seen_ = true;
    exploration_authority_stable_since_ = ros::Time();
    return;
  }
  handleInitialGoalAccepted(message->data, "exploration relay");
}

void MissionScheduler::goalCommandCallback(
    const super_planner::GoalCommand::ConstPtr& message) {
  if (!isExplorationTask(active_task_) || auto_enabled_ ||
      message->command != super_planner::GoalCommand::SET_GOAL ||
      message->goal_id == 0) {
    return;
  }
  pending_initial_goal_id_ = message->goal_id;
  // Seeing a new SET_GOAL after this task was activated is itself an epoch
  // marker. It cannot be a latched goal from the preceding HOLD state because
  // that state ignores goal commands and this callback records only messages
  // received while the exploration task is active.
  task_ready_reset_seen_ = true;
  if (last_super_accepted_goal_id_ == pending_initial_goal_id_) {
    handleInitialGoalAccepted(pending_initial_goal_id_, "direct SUPER handshake");
  }
}

void MissionScheduler::superGoalAcceptedCallback(
    const std_msgs::UInt64::ConstPtr& message) {
  last_super_accepted_goal_id_ = message->data;
  if (message->data != 0 && message->data == pending_initial_goal_id_) {
    handleInitialGoalAccepted(message->data, "direct SUPER handshake");
  }
}

void MissionScheduler::handleInitialGoalAccepted(uint64_t goal_id,
                                                  const char* source) {
  if (goal_id != 0 && initial_goal_accepted_id_ == goal_id) {
    return;
  }
  initial_goal_accepted_id_ = goal_id;
  if (!isExplorationTask(active_task_)) {
    return;
  }
  if (!task_ready_reset_seen_ || auto_enabled_) {
    return;
  }
  const ros::Time now = ros::Time::now();
  const bool odometry_fresh = have_odometry_ &&
      ageSec(last_odometry_time_, now) <= odometry_timeout_;
  const bool mavros_state_fresh = have_mavros_state_ &&
      ageSec(last_mavros_state_time_, now) <= mavros_state_timeout_;
  if (!command_fault_.empty() || !odometry_fresh ||
      !odometry_frame_valid_ || !alignment_ready_ ||
      !mavros_state_fresh || !mavros_connected_ || !mavros_armed_ ||
      external_mode_exit_pending_) {
    return;
  }
  const std::string prefix =
      active_task_ == Task::kTunnelExploration ? "task4" : "task1";
  // Goal selection can take several seconds after the first hover gate. The
  // vehicle or its estimator may start moving during that interval, so goal
  // acceptance is not itself permission to enter OFFBOARD. The timer below
  // requires a fresh continuous hover window at the actual handoff.
  exploration_authority_stable_since_ = ros::Time();
  last_reason_ = prefix + "_initial_goal_accepted_wait_hover";
  publishOutputs(last_reason_);
  ROS_INFO("%s initial SUPER goal %llu accepted; revalidating stable hover "
           "before automatic authority (%s)", taskName(active_task_).c_str(),
           static_cast<unsigned long long>(goal_id), source);
}

void MissionScheduler::goafFinishedCallback(const std_msgs::Bool::ConstPtr& message) {
  goaf_finished_ = message->data;
}

void MissionScheduler::shaftStatusCallback(const std_msgs::String::ConstPtr& message) {
  last_shaft_status_time_ = ros::Time::now();
  have_shaft_status_ = true;
  shaft_status_ = message->data;
  if (active_task_ == Task::kShaftExploration &&
      (shaft_status_ == "DESCENDING" || shaft_status_ == "RETURNING")) {
    shaft_started_ = true;
  }
}

void MissionScheduler::shaftCommandReadyCallback(
    const std_msgs::Bool::ConstPtr& message) {
  last_shaft_command_ready_time_ = ros::Time::now();
  have_shaft_command_ready_ = true;
  shaft_command_ready_ = message->data;
  if (active_task_ == Task::kShaftExploration && shaft_command_ready_) {
    shaft_command_started_ = true;
  }
}

MissionScheduler::RcLevel MissionScheduler::classifyRc(uint16_t value) const {
  if (value > 0 && value <= static_cast<uint16_t>(low_threshold_)) {
    return RcLevel::kLow;
  }
  if (value >= static_cast<uint16_t>(high_threshold_)) {
    return RcLevel::kHigh;
  }
  return RcLevel::kInvalid;
}

MissionScheduler::RcEvent MissionScheduler::updateEdge(
    RcEdge* edge, RcLevel level, const ros::Time& now) {
  if (!edge->pending_since.isZero() && now < edge->pending_since) {
    *edge = RcEdge();  // /clock moved backwards: rebuild a safe baseline.
  }
  if (level == RcLevel::kInvalid) {
    // The centre/dead-band position is a valid idle state for a three-position
    // RC switch even though it is not an actionable task edge.  Remember when
    // it became stable so an unrelated channel left at centre cannot keep the
    // post-task rearm barrier asserted forever.
    if (edge->pending != RcLevel::kInvalid ||
        edge->pending_since.isZero()) {
      edge->pending = RcLevel::kInvalid;
      edge->pending_since = now;
    }
    return RcEvent::kNone;
  }
  if (level != edge->pending) {
    edge->pending = level;
    edge->pending_since = now;
    return RcEvent::kNone;
  }
  if (ageSec(edge->pending_since, now) < switch_stable_time_ ||
      level == edge->stable) {
    if (level == edge->stable && !edge->initialized &&
        ageSec(edge->pending_since, now) >= switch_stable_time_) {
      edge->initialized = true;
    }
    return RcEvent::kNone;
  }
  const RcLevel previous = edge->stable;
  edge->stable = level;
  if (!edge->initialized) {
    edge->initialized = true;  // The first stable reading is only a baseline.
    return RcEvent::kNone;
  }
  if (previous == RcLevel::kLow && level == RcLevel::kHigh) {
    return RcEvent::kRising;
  }
  if (previous == RcLevel::kHigh && level == RcLevel::kLow) {
    return RcEvent::kFalling;
  }
  return RcEvent::kNone;
}

void MissionScheduler::requestExplorationStart(Task task,
                                               const ros::Time& now,
                                               bool odometry_fresh,
                                               bool mavros_state_fresh) {
  if (!isExplorationTask(task)) {
    return;
  }
  const std::string prefix =
      task == Task::kTunnelExploration ? "task4" : "task1";
  std::string rejection;
  if (!mavros_state_fresh || !mavros_connected_ || !mavros_armed_) {
    rejection = prefix + "_start_rejected_px4_not_connected_or_armed";
  } else if (!odometry_fresh || !odometry_frame_valid_) {
    rejection = prefix + "_start_rejected_planning_odometry";
  } else if (!alignment_ready_) {
    rejection = prefix + "_start_rejected_alignment_not_ready";
  } else if (!command_fault_.empty()) {
    rejection = prefix + "_start_rejected_command_fault_not_reset";
  }
  if (!rejection.empty()) {
    last_reason_ = rejection;
    publishOutputs(last_reason_);
    ROS_ERROR("%s start rejected: %s", taskName(task).c_str(),
              rejection.c_str());
    return;
  }

  goaf_start_pending_ = true;
  pending_exploration_task_ = task;
  goaf_start_check_started_ = now;
  goaf_stable_since_ = ros::Time();
  last_reason_ = prefix + "_wait_hover";
  publishOutputs(last_reason_);
  ROS_INFO("%s trigger accepted; waiting for a stable armed hover",
           taskName(task).c_str());
}

void MissionScheduler::cancelGoafStart(const std::string& reason) {
  goaf_start_pending_ = false;
  pending_exploration_task_ = Task::kHold;
  goaf_start_check_started_ = ros::Time();
  goaf_stable_since_ = ros::Time();
  last_reason_ = reason;
  publishOutputs(reason);
  ROS_WARN("Exploration pending start cancelled: %s", reason.c_str());
}

void MissionScheduler::updateGoafStartCheck(
    const ros::Time& now, bool odometry_fresh, bool mavros_state_fresh) {
  const std::string prefix =
      pending_exploration_task_ == Task::kTunnelExploration ? "task4" : "task1";
  if (!mavros_state_fresh || !mavros_connected_ || !mavros_armed_) {
    cancelGoafStart(prefix + "_start_check_px4_not_connected_or_armed");
    return;
  }
  if (!odometry_fresh || !odometry_frame_valid_) {
    cancelGoafStart(prefix + "_start_check_planning_odometry_lost");
    return;
  }
  if (!alignment_ready_) {
    cancelGoafStart(prefix + "_start_check_alignment_lost");
    return;
  }
  if (!command_fault_.empty()) {
    cancelGoafStart(prefix + "_start_check_command_fault");
    return;
  }
  if (ageSec(goaf_start_check_started_, now) > start_check_timeout_) {
    cancelGoafStart(prefix + "_stable_hover_timeout");
    return;
  }
  const bool stable = have_speed_estimate_ &&
      std::isfinite(estimated_speed_) &&
      std::isfinite(estimated_vertical_speed_) &&
      estimated_speed_ <= start_max_speed_ &&
      estimated_vertical_speed_ <= start_max_vertical_speed_;
  if (!stable) {
    goaf_stable_since_ = ros::Time();
    return;
  }
  if (goaf_stable_since_.isZero()) {
    goaf_stable_since_ = now;
    return;
  }
  if (ageSec(goaf_stable_since_, now) >= start_stable_time_) {
    const Task task = pending_exploration_task_;
    goaf_start_pending_ = false;
    pending_exploration_task_ = Task::kHold;
    applyTask(task, prefix + "_stable_hover_confirmed", false);
  }
}

bool MissionScheduler::explorationHoverStable() const {
  return have_speed_estimate_ && std::isfinite(estimated_speed_) &&
      std::isfinite(estimated_vertical_speed_) &&
      estimated_speed_ <= start_max_speed_ &&
      estimated_vertical_speed_ <= start_max_vertical_speed_;
}

void MissionScheduler::updateExplorationAuthorityCheck(
    const ros::Time& now, bool odometry_fresh, bool mavros_state_fresh) {
  if (!isExplorationTask(active_task_) || auto_enabled_ ||
      initial_goal_accepted_id_ == 0 || !task_ready_reset_seen_) {
    exploration_authority_stable_since_ = ros::Time();
    return;
  }
  if (!odometry_fresh || !odometry_frame_valid_ || !alignment_ready_ ||
      !mavros_state_fresh || !mavros_connected_ || !mavros_armed_ ||
      !command_fault_.empty() || external_mode_exit_pending_ ||
      !explorationHoverStable()) {
    exploration_authority_stable_since_ = ros::Time();
    return;
  }
  if (exploration_authority_stable_since_.isZero()) {
    exploration_authority_stable_since_ = now;
    return;
  }
  if (ageSec(exploration_authority_stable_since_, now) <
      start_stable_time_) {
    return;
  }

  auto_enabled_ = true;
  const std::string prefix =
      active_task_ == Task::kTunnelExploration ? "task4" : "task1";
  last_reason_ = prefix + "_initial_goal_accepted";
  publishOutputs(last_reason_);
  ROS_INFO("%s automatic authority enabled after initial goal %llu and %.2f "
           "s of revalidated stable hover",
           taskName(active_task_).c_str(),
           static_cast<unsigned long long>(initial_goal_accepted_id_),
           start_stable_time_);
}

void MissionScheduler::timerCallback(const ros::TimerEvent&) {
  const ros::Time now = ros::Time::now();
  const bool rc_fresh = have_rc_ && ageSec(last_rc_time_, now) <= rc_timeout_;
  const bool odometry_fresh =
      have_odometry_ && ageSec(last_odometry_time_, now) <= odometry_timeout_;
  const bool mavros_state_fresh = have_mavros_state_ &&
      ageSec(last_mavros_state_time_, now) <= mavros_state_timeout_;
  const bool mavros_ok = !require_mavros_connection_ ||
                         (mavros_state_fresh && mavros_connected_);
  // Task switches are flight-authority inputs.  While PX4 is connected, every
  // task channel may acquire and update its stable baseline before arming.
  // Events observed while disarmed are consumed as baseline updates; only a
  // subsequent debounced level change observed while armed is actionable.
  // Either LOW or HIGH can therefore be the preflight baseline.
  const bool task_switches_enabled = rc_fresh && mavros_state_fresh &&
      mavros_connected_ && mavros_armed_;
  const bool switch_baselines_enabled = rc_fresh && mavros_state_fresh &&
      mavros_connected_;
  const bool shaft_status_fresh = have_shaft_status_ &&
      ageSec(last_shaft_status_time_, now) <= shaft_status_timeout_;
  const bool shaft_ready_fresh = have_shaft_command_ready_ &&
      ageSec(last_shaft_command_ready_time_, now) <= shaft_ready_timeout_;
  const bool task3_ready_fresh = have_task3_command_ready_ &&
      ageSec(last_task3_command_ready_time_, now) <= task3_ready_timeout_;
  const bool was_rearming = rearm_pending_;
  // Consume edges even while busy, so an ignored switch cannot start a task
  // later when the current task finishes. A stale RC link never creates an edge.
  const RcEvent observed_goaf_event =
      switch_baselines_enabled && goaf_channel_available_
      ? updateEdge(&goaf_edge_, classifyRc(goaf_rc_value_), now)
      : RcEvent::kNone;
  const RcEvent observed_shaft_event =
      switch_baselines_enabled && shaft_channel_available_
      ? updateEdge(&shaft_edge_, classifyRc(shaft_rc_value_), now)
      : RcEvent::kNone;
  const RcEvent observed_task3_event =
      switch_baselines_enabled && task3_channel_available_
      ? updateEdge(&task3_edge_, classifyRc(task3_rc_value_), now)
      : RcEvent::kNone;
  const RcEvent observed_tunnel_event =
      switch_baselines_enabled && tunnel_channel_available_
      ? updateEdge(&tunnel_edge_, classifyRc(tunnel_rc_value_), now)
      : RcEvent::kNone;
  const RcEvent goaf_event = task_switches_enabled
      ? observed_goaf_event : RcEvent::kNone;
  const RcEvent shaft_event = task_switches_enabled
      ? observed_shaft_event : RcEvent::kNone;
  const RcEvent task3_event = task_switches_enabled
      ? observed_task3_event : RcEvent::kNone;
  const RcEvent tunnel_event = task_switches_enabled
      ? observed_tunnel_event : RcEvent::kNone;
  if (!switch_baselines_enabled || !goaf_channel_available_) {
    goaf_edge_.pending = RcLevel::kInvalid;
    goaf_edge_.stable = RcLevel::kInvalid;
    goaf_edge_.initialized = false;
  }
  if (!switch_baselines_enabled || !shaft_channel_available_) {
    shaft_edge_.pending = RcLevel::kInvalid;
    shaft_edge_.stable = RcLevel::kInvalid;
    shaft_edge_.initialized = false;
  }
  if (!switch_baselines_enabled || !task3_channel_available_) {
    task3_edge_.pending = RcLevel::kInvalid;
    task3_edge_.stable = RcLevel::kInvalid;
    task3_edge_.initialized = false;
  }
  if (!switch_baselines_enabled || !tunnel_channel_available_) {
    tunnel_edge_.pending = RcLevel::kInvalid;
    tunnel_edge_.stable = RcLevel::kInvalid;
    tunnel_edge_.initialized = false;
  }

  const auto channel_rearmed = [this, &now](const RcEdge& edge) {
    if (edge.initialized) {
      return ageSec(edge.pending_since, now) >= switch_stable_time_;
    }
    // Mid-position is deliberately non-triggering, but it is safe as a
    // post-task baseline once it has remained there for the debounce period.
    return edge.pending == RcLevel::kInvalid &&
        !edge.pending_since.isZero() &&
        ageSec(edge.pending_since, now) >= switch_stable_time_;
  };
  if (rearm_pending_ && rc_fresh && goaf_channel_available_ &&
      shaft_channel_available_ && channel_rearmed(goaf_edge_) &&
      channel_rearmed(shaft_edge_) &&
      (task3_trigger_channel_ < 0 || channel_rearmed(task3_edge_)) &&
      (tunnel_trigger_channel_ < 0 || channel_rearmed(tunnel_edge_))) {
    rearm_pending_ = false;
  }

  bool goaf_event_consumed = false;
  if (goaf_start_pending_) {
    const bool pending_is_tunnel =
        pending_exploration_task_ == Task::kTunnelExploration;
    const RcEvent pending_event = pending_is_tunnel ? tunnel_event : goaf_event;
    if (pending_event != RcEvent::kNone) {
      cancelGoafStart(pending_is_tunnel
                          ? "task4_trigger_cancelled_during_start_check"
                          : "task1_trigger_cancelled_during_start_check");
      goaf_event_consumed = true;
    } else {
      updateGoafStartCheck(now, odometry_fresh, mavros_state_fresh);
    }
  }

  updateExplorationAuthorityCheck(now, odometry_fresh, mavros_state_fresh);

  std::string reason;
  if (active_task_ == Task::kGoafExploration &&
             goaf_event != RcEvent::kNone) {
    reason = "task1_trigger_cancelled";
  } else if (active_task_ == Task::kTunnelExploration &&
             tunnel_event != RcEvent::kNone) {
    reason = "task4_trigger_cancelled";
  } else if (active_task_ == Task::kPressureControl &&
             task3_event != RcEvent::kNone) {
    reason = "task3_trigger_cancelled";
  } else if (!rc_fresh || !goaf_channel_available_ ||
             !shaft_channel_available_) {
    reason = "rc_lost_or_channel_unavailable";
  } else if (active_task_ == Task::kTunnelExploration &&
             !tunnel_channel_available_) {
    reason = "task4_trigger_channel_unavailable";
  } else if (!mavros_ok) {
    reason = mavros_state_fresh ? "mavros_disconnected" : "mavros_state_lost";
  } else if (isExplorationTask(active_task_) &&
             (!mavros_armed_ || !mavros_state_fresh)) {
    reason = active_task_ == Task::kTunnelExploration
                 ? "task4_px4_disarmed_or_state_lost"
                 : "task1_px4_disarmed_or_state_lost";
  } else if (isExplorationTask(active_task_) && require_odometry_ &&
             (!odometry_fresh || !odometry_frame_valid_)) {
    reason = active_task_ == Task::kTunnelExploration
                 ? "task4_planning_odometry_lost"
                 : "task1_planning_odometry_lost";
  } else if (isExplorationTask(active_task_) && !alignment_ready_) {
    reason = active_task_ == Task::kTunnelExploration
                 ? "task4_alignment_lost" : "task1_alignment_lost";
  } else if (isExplorationTask(active_task_) && !auto_enabled_ &&
             ageSec(goaf_activation_time_, now) >
                 initial_goal_accept_timeout_) {
    reason = active_task_ == Task::kTunnelExploration
                 ? "task4_initial_goal_accept_timeout"
                 : "task1_initial_goal_accept_timeout";
  } else if (isExplorationTask(active_task_) && goaf_finished_) {
    reason = active_task_ == Task::kTunnelExploration
                 ? "task4_complete" : "task1_complete";
  } else if (active_task_ == Task::kShaftExploration && shaft_started_ &&
             (shaft_status_ == "COMPLETE" ||
              shaft_status_ == "FAULT_NO_SAFE_AUTONOMOUS_RECOVERY")) {
    reason = shaft_status_ == "COMPLETE" ? "task2_complete" : "task2_fault";
  } else if (active_task_ == Task::kShaftExploration &&
             ageSec(shaft_activation_time_, now) > shaft_status_timeout_ &&
             !shaft_status_fresh) {
    reason = "task2_status_lost";
  } else if (active_task_ == Task::kShaftExploration && !shaft_started_ &&
             ageSec(shaft_activation_time_, now) > shaft_start_timeout_) {
    reason = "task2_start_timeout";
  } else if (active_task_ == Task::kShaftExploration &&
             shaft_command_started_ &&
             (!shaft_ready_fresh || !shaft_command_ready_)) {
    reason = "task2_command_not_ready";
  } else if (active_task_ == Task::kShaftExploration && shaft_started_ &&
             !shaft_command_started_ &&
             ageSec(shaft_activation_time_, now) > shaft_start_timeout_) {
    reason = "task2_command_start_timeout";
  } else if (active_task_ == Task::kPressureControl &&
             task3_started_ &&
             (!task3_ready_fresh || !task3_command_ready_)) {
    reason = "task3_command_not_ready";
  } else if (active_task_ == Task::kPressureControl && !task3_started_ &&
             ageSec(task3_activation_time_, now) > task3_start_timeout_) {
    reason = "task3_start_timeout";
  } else if (active_task_ == Task::kPressureControl &&
             !task3_channel_available_) {
    reason = "task3_trigger_channel_unavailable";
  } else if (active_task_ != Task::kHold && external_mode_exit_pending_) {
    reason = "px4_offboard_exited";
  }

  if (!reason.empty() && active_task_ != Task::kHold) {
    // A failed health gate revokes task authority. The task bridge handles
    // leaving OFFBOARD. Lost localization cannot safely support an autonomous
    // return, so do not publish a misleading return-home request.
    applyTask(Task::kHold, reason, false);
  } else if (active_task_ == Task::kHold && !goaf_start_pending_ &&
             reason.empty() &&
             !was_rearming && !rearm_pending_ && !goaf_event_consumed) {
    const bool goaf_start = goaf_event != RcEvent::kNone;
    const bool shaft_start = shaft_event != RcEvent::kNone;
    const bool task3_start = task3_event != RcEvent::kNone;
    const bool tunnel_start = tunnel_event != RcEvent::kNone;
    const int requested_tasks = static_cast<int>(goaf_start) +
        static_cast<int>(shaft_start) + static_cast<int>(task3_start) +
        static_cast<int>(tunnel_start);
    if (requested_tasks > 1) {
      reason = "simultaneous_task_edges_ignored";
    } else if (goaf_start) {
      requestExplorationStart(Task::kGoafExploration, now, odometry_fresh,
                              mavros_state_fresh);
    } else if (shaft_start && shaft_task_available_) {
      applyTask(Task::kShaftExploration, "ch11_edge", false);
    } else if (shaft_start) {
      reason = "shaft_task_unavailable";
    } else if (task3_start && task3_task_available_) {
      applyTask(Task::kPressureControl, "task3_trigger_edge", false);
    } else if (task3_start) {
      reason = "task3_unavailable";
    } else if (tunnel_start && tunnel_task_available_) {
      requestExplorationStart(Task::kTunnelExploration, now, odometry_fresh,
                              mavros_state_fresh);
    } else if (tunnel_start) {
      reason = "task4_unavailable";
    }
  }
  if (active_task_ == Task::kHold && !reason.empty() &&
      last_reason_ != reason) {
    last_reason_ = reason;
    publishOutputs(reason);
  }

  std::ostringstream status;
  status << "active_task=" << static_cast<int>(active_task_)
         << "(" << taskName(active_task_) << ")"
         << " ch9_rc=" << goaf_rc_value_
         << " ch11_rc=" << shaft_rc_value_
         << " task3_rc=" << task3_rc_value_
         << " task4_rc=" << tunnel_rc_value_
         << " auto_enabled=" << (auto_enabled_ ? "true" : "false")
         << " exploration_start_pending="
         << (goaf_start_pending_ ? taskName(pending_exploration_task_) : "false")
         << " planning_frame_ok=" << (odometry_frame_valid_ ? "true" : "false")
         << " alignment_ready=" << (alignment_ready_ ? "true" : "false")
         << " speed=" << (have_speed_estimate_ ? estimated_speed_ : -1.0)
         << " vertical_speed="
         << (have_speed_estimate_ ? estimated_vertical_speed_ : -1.0)
         << " initial_goal_id=" << initial_goal_accepted_id_
         << " command_fault="
         << (command_fault_.empty() ? "none" : command_fault_)
         << " rc_age=" << (have_rc_ ? ageSec(last_rc_time_, now) : -1.0)
         << " odom_age=" << (have_odometry_ ? ageSec(last_odometry_time_, now) : -1.0)
         << " mavros_age=" << (have_mavros_state_ ?
             ageSec(last_mavros_state_time_, now) : -1.0)
         << " shaft_status_age=" << (have_shaft_status_ ?
             ageSec(last_shaft_status_time_, now) : -1.0)
         << " task3_ready_age=" << (have_task3_command_ready_ ?
             ageSec(last_task3_command_ready_time_, now) : -1.0)
         << " task3_ready=" << (task3_command_ready_ ? "true" : "false")
         << " mavros_connected=" << (mavros_connected_ ? "true" : "false")
         << " armed=" << (mavros_armed_ ? "true" : "false")
         << " reason=" << last_reason_
         << " return_home=" << (return_home_requested_ ? "true" : "false");
  std_msgs::String status_message;
  status_message.data = status.str();
  status_publisher_.publish(status_message);
}

void MissionScheduler::applyTask(Task task, const std::string& reason,
                                 bool request_return) {
  const Task previous_task = active_task_;
  active_task_ = task;
  // Tasks 1 and 4 share the two-stage exploration handoff: first open
  // perception/goal selection, then enable automatic control only after
  // SUPER accepts this mission's initial goal. Task 2 retains its independent
  // immediate authority path.
  auto_enabled_ = task == Task::kShaftExploration ||
                  task == Task::kPressureControl;
  goaf_start_pending_ = false;
  pending_exploration_task_ = Task::kHold;
  goaf_start_check_started_ = ros::Time();
  goaf_stable_since_ = ros::Time();
  exploration_authority_stable_since_ = ros::Time();
  return_home_requested_ = request_return;
  last_reason_ = reason;
  offboard_seen_for_task_ = task != Task::kHold && px4_mode_ == "OFFBOARD";
  external_mode_exit_pending_ = false;
  if (isExplorationTask(task)) {
    goaf_finished_ = false;
    task_ready_reset_seen_ = false;
    initial_goal_accepted_id_ = 0;
    pending_initial_goal_id_ = 0;
    goaf_activation_time_ = ros::Time::now();
  }
  if (task == Task::kHold) {
    pending_initial_goal_id_ = 0;
  }
  if (task == Task::kShaftExploration) {
    shaft_started_ = false;
    shaft_command_started_ = false;
    shaft_command_ready_ = false;
    shaft_status_.clear();
    shaft_activation_time_ = ros::Time::now();
  }
  if (task == Task::kPressureControl) {
    task3_started_ = false;
    task3_command_ready_ = false;
    task3_activation_time_ = ros::Time::now();
  }
  if (task == Task::kHold && previous_task != Task::kHold) {
    // A switch movement that began while the old task was active must not
    // complete its debounce after the task ends and launch a new task.
    rearm_pending_ = true;
    goaf_edge_ = RcEdge();
    shaft_edge_ = RcEdge();
    task3_edge_ = RcEdge();
    tunnel_edge_ = RcEdge();
  }

  std_msgs::String event;
  std::ostringstream event_text;
  event_text << "task_change: " << taskName(previous_task) << " -> "
             << taskName(active_task_) << ", reason=" << reason;
  event.data = event_text.str();
  switch_event_publisher_.publish(event);
  publishOutputs(reason);

  ROS_INFO("Mission task changed: %s -> %s (%s)%s", taskName(previous_task).c_str(),
           taskName(active_task_).c_str(), reason.c_str(),
           request_return ? ", return requested" : "");
}

void MissionScheduler::publishOutputs(const std::string& reason) {
  std_msgs::UInt8 active_task;
  active_task.data = static_cast<uint8_t>(active_task_);
  active_task_publisher_.publish(active_task);

  std_msgs::Bool auto_enable;
  auto_enable.data = auto_enabled_;
  auto_enable_publisher_.publish(auto_enable);

  std_msgs::Bool goaf_enable;
  goaf_enable.data = active_task_ == Task::kGoafExploration;
  goaf_enable_publisher_.publish(goaf_enable);

  std_msgs::Bool shaft_enable;
  shaft_enable.data = active_task_ == Task::kShaftExploration;
  shaft_enable_publisher_.publish(shaft_enable);

  std_msgs::Bool task3_enable;
  task3_enable.data = active_task_ == Task::kPressureControl;
  task3_enable_publisher_.publish(task3_enable);

  std_msgs::Bool tunnel_enable;
  tunnel_enable.data = active_task_ == Task::kTunnelExploration;
  tunnel_enable_publisher_.publish(tunnel_enable);

  std_msgs::Bool exploration_enable;
  exploration_enable.data = isExplorationTask(active_task_);
  exploration_enable_publisher_.publish(exploration_enable);

  std_msgs::Bool return_home;
  return_home.data = return_home_requested_;
  return_home_publisher_.publish(return_home);

  std_msgs::String event;
  event.data = "state=" + taskName(active_task_) + ", reason=" + reason;
  switch_event_publisher_.publish(event);
}

std::string MissionScheduler::taskName(Task task) const {
  switch (task) {
    case Task::kGoafExploration:
      return "goaf_exploration";
    case Task::kShaftExploration:
      return "shaft_exploration";
    case Task::kPressureControl:
      return "pressure_control";
    case Task::kTunnelExploration:
      return "tunnel_exploration";
    case Task::kHold:
    default:
      return "hold";
  }
}

bool MissionScheduler::isExplorationTask(Task task) const {
  return task == Task::kGoafExploration ||
         task == Task::kTunnelExploration;
}

double MissionScheduler::ageSec(const ros::Time& stamp, const ros::Time& now) const {
  if (stamp.isZero()) {
    return std::numeric_limits<double>::infinity();
  }
  const double age = (now - stamp).toSec();
  return age < -0.05 ? std::numeric_limits<double>::infinity() :
                      std::max(0.0, age);
}

}  // namespace mine_uav_control

int main(int argc, char** argv) {
  ros::init(argc, argv, "mission_scheduler");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  mine_uav_control::MissionScheduler scheduler(nh, private_nh);
  ros::spin();
  return 0;
}
