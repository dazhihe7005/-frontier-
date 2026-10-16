#include "mine_uav_control/five_meter_avoidance_logic.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <sstream>
#include <string>

#include <geometry_msgs/TransformStamped.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt64.h>
#include <super_planner/GoalCommand.h>

namespace {

double ageSeconds(const ros::Time& stamp, const ros::Time& now) {
  if (stamp.isZero()) return std::numeric_limits<double>::infinity();
  const double age = (now - stamp).toSec();
  return age < -0.05 ? std::numeric_limits<double>::infinity()
                     : std::max(0.0, age);
}

double yawFromQuaternion(const geometry_msgs::Quaternion& q) {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

bool finitePose(const nav_msgs::Odometry& odometry) {
  const auto& p = odometry.pose.pose.position;
  const auto& q = odometry.pose.pose.orientation;
  const auto& v = odometry.twist.twist.linear;
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
         std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) &&
         std::isfinite(q.w) && std::isfinite(v.x) && std::isfinite(v.y) &&
         std::isfinite(v.z);
}

class FiveMeterAvoidanceTest {
 public:
  FiveMeterAvoidanceTest(const ros::NodeHandle& node,
                         const ros::NodeHandle& private_node)
      : node_(node), private_node_(private_node) {
    private_node_.param("rc_topic", rc_topic_, std::string("/mavros/rc/in"));
    private_node_.param("state_topic", state_topic_,
                        std::string("/mavros/state"));
    private_node_.param("odometry_topic", odometry_topic_,
                        std::string("/mine_uav/task1/odometry"));
    private_node_.param("cloud_topic", cloud_topic_,
                        std::string("/mine_uav/task1/cloud_registered"));
    private_node_.param("goal_topic", goal_topic_,
                        std::string("/mine_uav/super/goal_command"));
    private_node_.param(
        "alignment_topic", alignment_topic_,
        std::string("/mine_uav/task1/planning_to_px4_alignment"));
    private_node_.param("world_frame", world_frame_,
                        std::string("task1_planning"));
    private_node_.param("trigger_channel", trigger_channel_, 6);
    private_node_.param("low_threshold", low_threshold_, 1300);
    private_node_.param("high_threshold", high_threshold_, 1700);
    private_node_.param("switch_stable_time", switch_stable_time_, 0.35);
    private_node_.param("rc_timeout", rc_timeout_, 0.6);
    private_node_.param("odometry_timeout", odometry_timeout_, 0.5);
    private_node_.param("cloud_timeout", cloud_timeout_, 1.0);
    private_node_.param("map_warmup_time", map_warmup_time_, 1.0);
    private_node_.param("map_warmup_clouds", map_warmup_clouds_, 5);
    private_node_.param("start_max_speed", start_max_speed_, 0.10);
    private_node_.param("start_max_vertical_speed",
                        start_max_vertical_speed_, 0.05);
    private_node_.param("start_stable_time", start_stable_time_, 1.5);
    private_node_.param("start_check_timeout", start_check_timeout_, 8.0);
    private_node_.param("goal_distance", goal_distance_, 5.0);
    private_node_.param("goal_timeout", goal_timeout_, 25.0);
    private_node_.param("goal_accept_timeout", goal_accept_timeout_, 4.0);
    private_node_.param("position_tolerance", position_tolerance_, 0.35);
    private_node_.param("speed_tolerance", speed_tolerance_, 0.20);
    private_node_.param("settle_time", settle_time_, 1.0);
    private_node_.param("control_rate", control_rate_, 20.0);

    sanitizeParameters();

    rc_subscriber_ = node_.subscribe(rc_topic_, 10,
        &FiveMeterAvoidanceTest::rcCallback, this);
    state_subscriber_ = node_.subscribe(state_topic_, 10,
        &FiveMeterAvoidanceTest::stateCallback, this);
    odometry_subscriber_ = node_.subscribe(odometry_topic_, 20,
        &FiveMeterAvoidanceTest::odometryCallback, this);
    cloud_subscriber_ = node_.subscribe(cloud_topic_, 5,
        &FiveMeterAvoidanceTest::cloudCallback, this);
    alignment_subscriber_ = node_.subscribe(
        alignment_topic_, 1, &FiveMeterAvoidanceTest::alignmentCallback, this);
    accepted_subscriber_ = node_.subscribe(
        "/mine_uav/super/accepted_goal_id", 10,
        &FiveMeterAvoidanceTest::acceptedCallback, this);
    rejected_subscriber_ = node_.subscribe(
        "/mine_uav/super/rejected_goal_id", 10,
        &FiveMeterAvoidanceTest::rejectedCallback, this);
    command_fault_subscriber_ = node_.subscribe(
        "/mine_uav/task1/command_fault", 1,
        &FiveMeterAvoidanceTest::commandFaultCallback, this);

    goal_publisher_ = node_.advertise<super_planner::GoalCommand>(
        goal_topic_, 10, true);
    auto_enable_publisher_ = node_.advertise<std_msgs::Bool>(
        "/mine_uav/mission/auto_enable", 1, true);
    goaf_enable_publisher_ = node_.advertise<std_msgs::Bool>(
        "/mine_uav/mission/goaf_enable", 1, true);
    finished_publisher_ = node_.advertise<std_msgs::Bool>(
        "/mine_uav/exploration/finished", 1, true);
    status_publisher_ = node_.advertise<std_msgs::String>(
        "/mine_uav/exploration/status", 2, true);
    test_status_publisher_ = node_.advertise<std_msgs::String>(
        "/mine_uav/five_meter_test/status", 2, true);

    publishAuthority(false);
    publishFinished(false);
    publishStatus("IDLE", "lower CH7, establish a stable hover, then raise CH7");
    timer_ = node_.createTimer(ros::Duration(1.0 / control_rate_),
                               &FiveMeterAvoidanceTest::timerCallback, this);
    ROS_WARN("Temporary five-metre avoidance test active. It replaces the task-one "
             "decider for this launch and never arms PX4.");
  }

 private:
  enum class Phase { kIdle, kStartCheck, kWarmup, kActive, kTerminal };
  enum class RcLevel { kInvalid, kLow, kHigh };
  enum class RcEvent { kNone, kRising, kFalling };

  void sanitizeParameters() {
    trigger_channel_ = std::max(0, trigger_channel_);
    if (low_threshold_ >= high_threshold_) {
      low_threshold_ = 1300;
      high_threshold_ = 1700;
    }
    switch_stable_time_ = std::max(0.1, switch_stable_time_);
    rc_timeout_ = std::max(0.2, rc_timeout_);
    odometry_timeout_ = std::max(0.2, odometry_timeout_);
    cloud_timeout_ = std::max(0.3, cloud_timeout_);
    map_warmup_time_ = std::max(0.5, map_warmup_time_);
    map_warmup_clouds_ = std::max(1, map_warmup_clouds_);
    start_max_speed_ = std::max(0.02, start_max_speed_);
    start_max_vertical_speed_ =
        std::max(0.01, start_max_vertical_speed_);
    start_stable_time_ = std::max(0.5, start_stable_time_);
    start_check_timeout_ = std::max(start_stable_time_, start_check_timeout_);
    goal_distance_ = std::max(0.5, goal_distance_);
    goal_timeout_ = std::max(5.0, goal_timeout_);
    goal_accept_timeout_ = std::max(1.0, goal_accept_timeout_);
    position_tolerance_ = std::max(0.1, position_tolerance_);
    speed_tolerance_ = std::max(0.05, speed_tolerance_);
    settle_time_ = std::max(0.2, settle_time_);
    control_rate_ = std::max(5.0, control_rate_);
  }

  void rcCallback(const mavros_msgs::RCIn::ConstPtr& message) {
    last_rc_time_ = ros::Time::now();
    if (trigger_channel_ >= static_cast<int>(message->channels.size())) {
      rc_available_ = false;
      return;
    }
    rc_available_ = true;
    rc_value_ = message->channels[trigger_channel_];
  }

  void stateCallback(const mavros_msgs::State::ConstPtr& message) {
    mavros_connected_ = message->connected;
    mavros_armed_ = message->armed;
    have_mavros_state_ = true;
  }

  void odometryCallback(const nav_msgs::Odometry::ConstPtr& message) {
    if (!finitePose(*message)) return;
    if (!world_frame_.empty() && message->header.frame_id != world_frame_) {
      ROS_ERROR_THROTTLE(1.0, "Five-metre test rejected odometry frame '%s'; expected '%s'",
                         message->header.frame_id.c_str(), world_frame_.c_str());
      return;
    }
    const ros::Time now = ros::Time::now();
    if (have_odometry_) {
      const double dt = ageSeconds(last_odometry_time_, now);
      if (dt >= 0.01 && dt <= odometry_timeout_) {
        const auto& previous = odometry_.pose.pose.position;
        const auto& current = message->pose.pose.position;
        const double dx = current.x - previous.x;
        const double dy = current.y - previous.y;
        const double dz = current.z - previous.z;
        const double sample_speed = std::sqrt(dx * dx + dy * dy + dz * dz) / dt;
        // FAST-LIO's Odometry twist is zero in the current integration. Use a
        // filtered pose derivative so the completion check still requires a
        // genuinely settled vehicle rather than trusting that placeholder.
        estimated_speed_ = have_speed_estimate_
            ? 0.25 * sample_speed + 0.75 * estimated_speed_
            : sample_speed;
        const double sample_vertical_speed = std::abs(dz) / dt;
        estimated_vertical_speed_ = have_speed_estimate_
            ? 0.25 * sample_vertical_speed +
                  0.75 * estimated_vertical_speed_
            : sample_vertical_speed;
        have_speed_estimate_ = true;
      }
    }
    odometry_ = *message;
    have_odometry_ = true;
    last_odometry_time_ = now;
  }

  void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& message) {
    if (phase_ != Phase::kWarmup && phase_ != Phase::kActive) return;
    if (!world_frame_.empty() && message->header.frame_id != world_frame_) return;
    last_cloud_time_ = ros::Time::now();
    if (phase_ == Phase::kWarmup) ++warmup_cloud_count_;
  }

  void alignmentCallback(
      const geometry_msgs::TransformStamped::ConstPtr& message) {
    const auto& q = message->transform.rotation;
    const double norm =
        std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    if (!std::isfinite(q.x) || !std::isfinite(q.y) ||
        !std::isfinite(q.z) || !std::isfinite(q.w) ||
        !std::isfinite(norm) || norm < 1e-6) {
      ROS_ERROR_THROTTLE(1.0,
                         "Five-metre test rejected invalid alignment");
      return;
    }
    if (!world_frame_.empty() && message->child_frame_id != world_frame_) {
      ROS_ERROR_THROTTLE(
          1.0,
          "Five-metre test rejected alignment child frame '%s'; expected '%s'",
          message->child_frame_id.c_str(), world_frame_.c_str());
      return;
    }
    const auto rotation = mine_uav_control::normalized(
        mine_uav_control::Quaternion(q.x, q.y, q.z, q.w));
    if (mine_uav_control::quaternionAngularDistance(
            rotation, mine_uav_control::Quaternion()) > 1e-3) {
      have_alignment_ = false;
      ROS_ERROR_THROTTLE(
          1.0, "Five-metre test requires a pre-levelled planning frame");
      return;
    }
    have_alignment_ = true;
  }

  void acceptedCallback(const std_msgs::UInt64::ConstPtr& message) {
    if (phase_ == Phase::kActive && message->data == active_goal_id_) {
      if (!goal_accepted_) {
        goal_accepted_ = true;
        // Do not let the command bridge begin its OFFBOARD prestream until
        // SUPER has committed a trajectory for this exact goal lifecycle.
        publishAutomaticControl(true);
        ROS_INFO("SUPER accepted five-metre test goal %llu",
                 static_cast<unsigned long long>(active_goal_id_));
      }
    }
  }

  void rejectedCallback(const std_msgs::UInt64::ConstPtr& message) {
    if (phase_ == Phase::kActive && message->data == active_goal_id_) {
      finish(false, "SUPER_REJECTED_GOAL");
    }
  }

  void commandFaultCallback(const std_msgs::String::ConstPtr& message) {
    if ((phase_ == Phase::kWarmup || phase_ == Phase::kActive) &&
        !message->data.empty()) {
      ROS_ERROR("Five-metre test aborting on command bridge fault: %s",
                message->data.c_str());
      finish(false, "COMMAND_BRIDGE_FAULT_" + message->data);
    }
  }

  RcLevel classifyRc() const {
    if (!rc_available_) return RcLevel::kInvalid;
    if (rc_value_ > 0 && rc_value_ <= low_threshold_) return RcLevel::kLow;
    if (rc_value_ >= high_threshold_) return RcLevel::kHigh;
    return RcLevel::kInvalid;
  }

  RcEvent updateDebouncedSwitch(const ros::Time& now) {
    const RcLevel level = classifyRc();
    if (level == RcLevel::kInvalid) {
      pending_rc_level_ = RcLevel::kInvalid;
      pending_rc_since_ = ros::Time();
      return RcEvent::kNone;
    }
    if (level != pending_rc_level_) {
      pending_rc_level_ = level;
      pending_rc_since_ = now;
      return RcEvent::kNone;
    }
    if (pending_rc_since_.isZero() ||
        ageSeconds(pending_rc_since_, now) < switch_stable_time_ ||
        stable_rc_level_ == level) {
      return RcEvent::kNone;
    }
    const RcLevel previous = stable_rc_level_;
    stable_rc_level_ = level;
    if (!rc_baseline_ready_) {
      rc_baseline_ready_ = true;
      return RcEvent::kNone;
    }
    if (previous == RcLevel::kLow && level == RcLevel::kHigh)
      return RcEvent::kRising;
    if (previous == RcLevel::kHigh && level == RcLevel::kLow)
      return RcEvent::kFalling;
    return RcEvent::kNone;
  }

  void timerCallback(const ros::TimerEvent&) {
    const ros::Time now = ros::Time::now();
    const bool rc_fresh = ageSeconds(last_rc_time_, now) <= rc_timeout_;
    if (!rc_fresh) {
      rc_available_ = false;
      rc_baseline_ready_ = false;
      stable_rc_level_ = RcLevel::kInvalid;
      pending_rc_level_ = RcLevel::kInvalid;
      pending_rc_since_ = ros::Time();
      if (phase_ == Phase::kStartCheck || phase_ == Phase::kWarmup ||
          phase_ == Phase::kActive)
        finish(false, "RC_LOST");
      return;
    }

    const RcEvent event = updateDebouncedSwitch(now);
    if (event == RcEvent::kFalling &&
        (phase_ == Phase::kStartCheck || phase_ == Phase::kWarmup ||
         phase_ == Phase::kActive)) {
      finish(false, "CH7_CANCELLED");
      return;
    }
    if (phase_ == Phase::kTerminal && stable_rc_level_ == RcLevel::kLow) {
      phase_ = Phase::kIdle;
      publishFinished(false);
      publishStatus("IDLE", "ready for next CH7 rising edge");
      return;
    }
    if (phase_ == Phase::kIdle && event == RcEvent::kRising) {
      requestStart(now);
      return;
    }
    if (phase_ == Phase::kStartCheck) updateStartCheck(now);
    if (phase_ == Phase::kWarmup) updateWarmup(now);
    if (phase_ == Phase::kActive) updateActive(now);
  }

  void requestStart(const ros::Time& now) {
    if (!have_mavros_state_ || !mavros_connected_ || !mavros_armed_) {
      publishStatus("IDLE", "START_REJECTED_PX4_NOT_CONNECTED_OR_ARMED");
      ROS_ERROR("Five-metre test start rejected: PX4 is not connected and armed");
      return;
    }
    if (!have_alignment_) {
      publishStatus("IDLE", "START_REJECTED_ALIGNMENT_NOT_READY");
      ROS_ERROR("Five-metre test start rejected: alignment is not ready");
      return;
    }
    if (!have_odometry_ ||
        ageSeconds(last_odometry_time_, now) > odometry_timeout_) {
      publishStatus("IDLE", "START_REJECTED_ODOMETRY_NOT_FRESH");
      ROS_ERROR("Five-metre test start rejected: no fresh odometry");
      return;
    }
    start_check_started_ = now;
    start_stable_since_ = ros::Time();
    phase_ = Phase::kStartCheck;
    publishFinished(false);
    publishAuthority(false);
    publishStatus("WAIT_HOVER", "waiting for a stable hover before task handoff");
    ROS_INFO("Five-metre test trigger accepted; checking hover stability before handoff");
  }

  void updateStartCheck(const ros::Time& now) {
    if (!have_mavros_state_ || !mavros_connected_ || !mavros_armed_) {
      finish(false, "PX4_DISCONNECTED_OR_DISARMED_DURING_START_CHECK");
      return;
    }
    if (!have_odometry_ ||
        ageSeconds(last_odometry_time_, now) > odometry_timeout_) {
      finish(false, "ODOMETRY_LOST_DURING_START_CHECK");
      return;
    }
    if (ageSeconds(start_check_started_, now) > start_check_timeout_) {
      finish(false, "STABLE_HOVER_TIMEOUT");
      return;
    }
    if (!have_speed_estimate_ || !mine_uav_control::hoverStable(
            estimated_speed_, estimated_vertical_speed_, start_max_speed_,
            start_max_vertical_speed_)) {
      start_stable_since_ = ros::Time();
      return;
    }
    if (start_stable_since_.isZero()) {
      start_stable_since_ = now;
      return;
    }
    if (ageSeconds(start_stable_since_, now) >= start_stable_time_)
      start(now);
  }

  void start(const ros::Time& now) {
    const auto& pose = odometry_.pose.pose;
    const double yaw = yawFromQuaternion(pose.orientation);
    goal_ = mine_uav_control::makeForwardGoal(
        pose.position.x, pose.position.y, pose.position.z, yaw,
        goal_distance_);
    activation_time_ = now;
    last_cloud_time_ = ros::Time();
    warmup_cloud_count_ = 0;
    active_goal_id_ = 0;
    goal_accepted_ = false;
    settled_since_ = ros::Time();
    phase_ = Phase::kWarmup;
    publishFinished(false);
    // Perception must run during map warmup, but automatic flight authority
    // remains inhibited until the newly published goal is accepted.
    publishMissionSelection(true);
    publishAutomaticControl(false);
    publishStatus("WAIT_DATA", "warming SUPER map with fresh gated clouds");
    ROS_INFO("Five-metre test armed at (%.2f, %.2f, %.2f), yaw %.1f deg; "
             "target (%.2f, %.2f, %.2f)",
             pose.position.x, pose.position.y, pose.position.z,
             yaw * 180.0 / 3.14159265358979323846,
             goal_.x, goal_.y, goal_.z);
  }

  void updateWarmup(const ros::Time& now) {
    if (ageSeconds(last_odometry_time_, now) > odometry_timeout_) {
      finish(false, "ODOMETRY_LOST_DURING_WARMUP");
      return;
    }
    if (ageSeconds(activation_time_, now) > goal_timeout_) {
      finish(false, "MAP_WARMUP_TIMEOUT");
      return;
    }
    if (warmup_cloud_count_ < map_warmup_clouds_ ||
        ageSeconds(activation_time_, now) < map_warmup_time_) {
      return;
    }
    if (ageSeconds(last_cloud_time_, now) > cloud_timeout_) return;
    publishGoal(now);
  }

  void publishGoal(const ros::Time& now) {
    active_goal_id_ = next_goal_id_++;
    if (active_goal_id_ == 0) active_goal_id_ = next_goal_id_++;
    super_planner::GoalCommand command;
    command.header.stamp = now;
    command.header.frame_id = world_frame_;
    command.command = super_planner::GoalCommand::SET_GOAL;
    command.goal_id = active_goal_id_;
    command.goal.position.x = goal_.x;
    command.goal.position.y = goal_.y;
    command.goal.position.z = goal_.z;
    command.goal.orientation.z = std::sin(goal_.yaw * 0.5);
    command.goal.orientation.w = std::cos(goal_.yaw * 0.5);
    command.reason = "temporary_five_meter_avoidance_test";
    goal_publisher_.publish(command);
    goal_sent_time_ = now;
    phase_ = Phase::kActive;
    publishStatus("ACTIVE", "SUPER owns avoidance to the single 5 m goal");
    ROS_INFO("Published SUPER goal %llu at (%.2f, %.2f, %.2f)",
             static_cast<unsigned long long>(active_goal_id_),
             goal_.x, goal_.y, goal_.z);
  }

  void updateActive(const ros::Time& now) {
    if (ageSeconds(last_odometry_time_, now) > odometry_timeout_) {
      finish(false, "ODOMETRY_LOST");
      return;
    }
    if (ageSeconds(last_cloud_time_, now) > cloud_timeout_) {
      finish(false, "POINT_CLOUD_LOST");
      return;
    }
    if (!goal_accepted_ &&
        ageSeconds(goal_sent_time_, now) > goal_accept_timeout_) {
      finish(false, "SUPER_GOAL_ACCEPT_TIMEOUT");
      return;
    }
    if (ageSeconds(goal_sent_time_, now) > goal_timeout_) {
      finish(false, "GOAL_TIMEOUT");
      return;
    }

    const auto& p = odometry_.pose.pose.position;
    if (!have_speed_estimate_ || !mine_uav_control::goalSettled(
            goal_, p.x, p.y, p.z, estimated_speed_, 0.0, 0.0,
            position_tolerance_, speed_tolerance_)) {
      settled_since_ = ros::Time();
      return;
    }
    if (settled_since_.isZero()) {
      settled_since_ = now;
      return;
    }
    if (ageSeconds(settled_since_, now) >= settle_time_)
      finish(true, "GOAL_REACHED_AND_SETTLED");
  }

  void cancelGoal(const std::string& reason) {
    if (active_goal_id_ == 0) return;
    super_planner::GoalCommand command;
    command.header.stamp = ros::Time::now();
    command.header.frame_id = world_frame_;
    command.command = super_planner::GoalCommand::CANCEL_GOAL;
    command.goal_id = active_goal_id_;
    command.goal.orientation.w = 1.0;
    command.reason = reason;
    goal_publisher_.publish(command);
    active_goal_id_ = 0;
  }

  void finish(bool success, const std::string& reason) {
    if (phase_ == Phase::kTerminal || phase_ == Phase::kIdle) return;
    cancelGoal(reason);
    publishStatus(success ? "COMPLETE" : "ABORTED", reason);
    publishFinished(true);
    publishAuthority(false);
    phase_ = Phase::kTerminal;
    ROS_WARN("Five-metre test %s: %s. Lower CH7 before another attempt.",
             success ? "completed" : "aborted", reason.c_str());
  }

  void publishAuthority(bool enabled) {
    publishMissionSelection(enabled);
    publishAutomaticControl(enabled);
  }

  void publishMissionSelection(bool enabled) {
    std_msgs::Bool message;
    message.data = enabled;
    goaf_enable_publisher_.publish(message);
  }

  void publishAutomaticControl(bool enabled) {
    std_msgs::Bool message;
    message.data = enabled;
    auto_enable_publisher_.publish(message);
  }

  void publishFinished(bool finished) {
    std_msgs::Bool message;
    message.data = finished;
    finished_publisher_.publish(message);
  }

  void publishStatus(const std::string& state, const std::string& detail) {
    std_msgs::String exploration;
    exploration.data = state + ":" + detail;
    status_publisher_.publish(exploration);

    std::ostringstream text;
    text << "state=" << state << " detail=" << detail
         << " ch7=" << rc_value_ << " clouds=" << warmup_cloud_count_
         << " speed=" << estimated_speed_
         << " vertical_speed=" << estimated_vertical_speed_;
    std_msgs::String diagnostic;
    diagnostic.data = text.str();
    test_status_publisher_.publish(diagnostic);
  }

  ros::NodeHandle node_;
  ros::NodeHandle private_node_;
  ros::Subscriber rc_subscriber_;
  ros::Subscriber state_subscriber_;
  ros::Subscriber odometry_subscriber_;
  ros::Subscriber cloud_subscriber_;
  ros::Subscriber alignment_subscriber_;
  ros::Subscriber accepted_subscriber_;
  ros::Subscriber rejected_subscriber_;
  ros::Subscriber command_fault_subscriber_;
  ros::Publisher goal_publisher_;
  ros::Publisher auto_enable_publisher_;
  ros::Publisher goaf_enable_publisher_;
  ros::Publisher finished_publisher_;
  ros::Publisher status_publisher_;
  ros::Publisher test_status_publisher_;
  ros::Timer timer_;

  std::string rc_topic_;
  std::string state_topic_;
  std::string odometry_topic_;
  std::string cloud_topic_;
  std::string goal_topic_;
  std::string alignment_topic_;
  std::string world_frame_;
  int trigger_channel_{6};
  int low_threshold_{1300};
  int high_threshold_{1700};
  int map_warmup_clouds_{5};
  double switch_stable_time_{0.35};
  double rc_timeout_{0.6};
  double odometry_timeout_{0.5};
  double cloud_timeout_{1.0};
  double map_warmup_time_{1.0};
  double start_max_speed_{0.10};
  double start_max_vertical_speed_{0.05};
  double start_stable_time_{1.5};
  double start_check_timeout_{8.0};
  double goal_distance_{5.0};
  double goal_timeout_{25.0};
  double goal_accept_timeout_{4.0};
  double position_tolerance_{0.35};
  double speed_tolerance_{0.20};
  double settle_time_{1.0};
  double control_rate_{20.0};

  Phase phase_{Phase::kIdle};
  RcLevel pending_rc_level_{RcLevel::kInvalid};
  RcLevel stable_rc_level_{RcLevel::kInvalid};
  ros::Time pending_rc_since_;
  bool rc_baseline_ready_{false};
  bool rc_available_{false};
  uint16_t rc_value_{0};
  ros::Time last_rc_time_;

  bool have_mavros_state_{false};
  bool mavros_connected_{false};
  bool mavros_armed_{false};

  bool have_alignment_{false};

  nav_msgs::Odometry odometry_;
  bool have_odometry_{false};
  bool have_speed_estimate_{false};
  double estimated_speed_{0.0};
  double estimated_vertical_speed_{0.0};
  ros::Time last_odometry_time_;
  ros::Time last_cloud_time_;
  ros::Time activation_time_;
  ros::Time start_check_started_;
  ros::Time start_stable_since_;
  ros::Time goal_sent_time_;
  ros::Time settled_since_;
  int warmup_cloud_count_{0};
  mine_uav_control::FiveMeterGoal goal_;
  uint64_t next_goal_id_{1};
  uint64_t active_goal_id_{0};
  bool goal_accepted_{false};
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "five_meter_avoidance_test");
  ros::NodeHandle node;
  ros::NodeHandle private_node("~");
  FiveMeterAvoidanceTest test(node, private_node);
  ros::spin();
  return 0;
}
