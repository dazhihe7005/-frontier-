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
  private_nh_.param("odometry_topic", odometry_topic_, std::string("/Odometry"));
  private_nh_.param("mavros_state_topic", mavros_state_topic_,
                    std::string("/mavros/state"));
  private_nh_.param("active_task_topic", active_task_topic_,
                    std::string("/mine_uav/mission/active_task"));
  private_nh_.param("auto_enable_topic", auto_enable_topic_,
                    std::string("/mine_uav/mission/auto_enable"));
  private_nh_.param("goaf_enable_topic", goaf_enable_topic_,
                    std::string("/mine_uav/mission/goaf_enable"));
  private_nh_.param("shaft_enable_topic", shaft_enable_topic_,
                    std::string("/mine_uav/mission/shaft_enable"));
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
  private_nh_.param("decision_rate", decision_rate_, decision_rate_);
  private_nh_.param("require_odometry", require_odometry_, require_odometry_);
  private_nh_.param("require_mavros_connection", require_mavros_connection_,
                    require_mavros_connection_);
  private_nh_.param("shaft_task_available", shaft_task_available_,
                    shaft_task_available_);

  if (goaf_trigger_channel_ < 0 || shaft_trigger_channel_ < 0 ||
      goaf_trigger_channel_ == shaft_trigger_channel_) {
    ROS_WARN("Task trigger channels must be distinct and non-negative; using CH7/CH11");
    goaf_trigger_channel_ = 6;
    shaft_trigger_channel_ = 10;
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
  if (!std::isfinite(decision_rate_) || decision_rate_ <= 0.0) {
    decision_rate_ = 10.0;
  }

  rc_subscriber_ = nh_.subscribe(rc_topic_, 10, &MissionScheduler::rcCallback, this);
  odometry_subscriber_ = nh_.subscribe(
      odometry_topic_, 10, &MissionScheduler::odometryCallback, this);
  mavros_state_subscriber_ = nh_.subscribe(
      mavros_state_topic_, 10, &MissionScheduler::mavrosStateCallback, this);
  goaf_finished_subscriber_ = nh_.subscribe(
      "/mine_uav/exploration/finished", 1,
      &MissionScheduler::goafFinishedCallback, this);
  shaft_status_subscriber_ = nh_.subscribe(
      "/mine_uav/shaft/status", 1,
      &MissionScheduler::shaftStatusCallback, this);

  // Latched outputs let a task node receive the current selection immediately.
  active_task_publisher_ = nh_.advertise<std_msgs::UInt8>(active_task_topic_, 1, true);
  auto_enable_publisher_ =
      nh_.advertise<std_msgs::Bool>(auto_enable_topic_, 1, true);
  goaf_enable_publisher_ = nh_.advertise<std_msgs::Bool>(goaf_enable_topic_, 1, true);
  shaft_enable_publisher_ = nh_.advertise<std_msgs::Bool>(shaft_enable_topic_, 1, true);
  return_home_publisher_ = nh_.advertise<std_msgs::Bool>(return_home_topic_, 1, true);
  switch_event_publisher_ = nh_.advertise<std_msgs::String>(switch_event_topic_, 10, true);
  status_publisher_ = nh_.advertise<std_msgs::String>(status_topic_, 10);

  timer_ = nh_.createTimer(ros::Duration(1.0 / decision_rate_),
                           &MissionScheduler::timerCallback, this);
  publishOutputs("startup");

  ROS_INFO("mission_scheduler ready: goaf trigger CH%d, shaft trigger CH%d, "
           "either stable edge starts an idle task; odometry required=%s, shaft available=%s",
           goaf_trigger_channel_ + 1, shaft_trigger_channel_ + 1,
           require_odometry_ ? "true" : "false",
           shaft_task_available_ ? "true" : "false");
}

void MissionScheduler::rcCallback(const mavros_msgs::RCIn::ConstPtr& message) {
  last_rc_time_ = ros::Time::now();
  have_rc_ = true;

  goaf_channel_available_ =
      goaf_trigger_channel_ < static_cast<int>(message->channels.size());
  shaft_channel_available_ =
      shaft_trigger_channel_ < static_cast<int>(message->channels.size());
  goaf_rc_value_ = goaf_channel_available_
                       ? message->channels[goaf_trigger_channel_] : 0;
  shaft_rc_value_ = shaft_channel_available_
                        ? message->channels[shaft_trigger_channel_] : 0;
  if (!goaf_channel_available_ || !shaft_channel_available_) {
    ROS_WARN_THROTTLE(2.0, "CH7/CH11 trigger channels unavailable: received %zu channels",
                      message->channels.size());
  }
}

void MissionScheduler::odometryCallback(const nav_msgs::Odometry::ConstPtr&) {
  last_odometry_time_ = ros::Time::now();
  have_odometry_ = true;
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

MissionScheduler::RcLevel MissionScheduler::classifyRc(uint16_t value) const {
  if (value > 0 && value <= static_cast<uint16_t>(low_threshold_)) {
    return RcLevel::kLow;
  }
  if (value >= static_cast<uint16_t>(high_threshold_)) {
    return RcLevel::kHigh;
  }
  return RcLevel::kInvalid;
}

bool MissionScheduler::updateEdge(RcEdge* edge, RcLevel level,
                                  const ros::Time& now) {
  if (!edge->pending_since.isZero() && now < edge->pending_since) {
    *edge = RcEdge();  // /clock moved backwards: rebuild a safe baseline.
  }
  if (level == RcLevel::kInvalid) {
    edge->pending = RcLevel::kInvalid;
    return false;
  }
  if (level != edge->pending) {
    edge->pending = level;
    edge->pending_since = now;
    return false;
  }
  if (ageSec(edge->pending_since, now) < switch_stable_time_ ||
      level == edge->stable) {
    if (level == edge->stable && !edge->initialized &&
        ageSec(edge->pending_since, now) >= switch_stable_time_) {
      edge->initialized = true;
    }
    return false;
  }
  edge->stable = level;
  if (!edge->initialized) {
    edge->initialized = true;  // The first stable reading is only a baseline.
    return false;
  }
  return true;
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
  const bool shaft_status_fresh = have_shaft_status_ &&
      ageSec(last_shaft_status_time_, now) <= shaft_status_timeout_;
  const bool was_rearming = rearm_pending_;
  // Consume edges even while busy, so an ignored switch cannot start a task
  // later when the current task finishes. A stale RC link never creates an edge.
  const bool goaf_edge = rc_fresh && goaf_channel_available_ &&
      updateEdge(&goaf_edge_, classifyRc(goaf_rc_value_), now);
  const bool shaft_edge = rc_fresh && shaft_channel_available_ &&
      updateEdge(&shaft_edge_, classifyRc(shaft_rc_value_), now);
  if (!rc_fresh || !goaf_channel_available_) {
    goaf_edge_.pending = RcLevel::kInvalid;
    goaf_edge_.stable = RcLevel::kInvalid;
    goaf_edge_.initialized = false;
  }
  if (!rc_fresh || !shaft_channel_available_) {
    shaft_edge_.pending = RcLevel::kInvalid;
    shaft_edge_.stable = RcLevel::kInvalid;
    shaft_edge_.initialized = false;
  }

  if (rearm_pending_ && rc_fresh && goaf_channel_available_ &&
      shaft_channel_available_ && goaf_edge_.initialized &&
      shaft_edge_.initialized &&
      ageSec(goaf_edge_.pending_since, now) >= switch_stable_time_ &&
      ageSec(shaft_edge_.pending_since, now) >= switch_stable_time_) {
    rearm_pending_ = false;
  }

  std::string reason;
  if (!rc_fresh || !goaf_channel_available_ || !shaft_channel_available_) {
    reason = "rc_lost_or_channel_unavailable";
  } else if (!mavros_ok) {
    reason = mavros_state_fresh ? "mavros_disconnected" : "mavros_state_lost";
  } else if (active_task_ == Task::kGoafExploration &&
             require_odometry_ && !odometry_fresh) {
    reason = "fastlio2_odometry_lost";
  } else if (active_task_ == Task::kGoafExploration && goaf_finished_) {
    reason = "task1_complete";
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
  } else if (active_task_ != Task::kHold && external_mode_exit_pending_) {
    reason = "px4_offboard_exited";
  }

  if (!reason.empty() && active_task_ != Task::kHold) {
    // A failed health gate revokes task authority. The task bridge handles
    // leaving OFFBOARD. Lost localization cannot safely support an autonomous
    // return, so do not publish a misleading return-home request.
    applyTask(Task::kHold, reason, false);
  } else if (active_task_ == Task::kHold && reason.empty() &&
             !was_rearming && !rearm_pending_) {
    if (goaf_edge && shaft_edge) {
      reason = "simultaneous_task_edges_ignored";
    } else if (goaf_edge) {
      if (require_odometry_ && !odometry_fresh) {
        reason = "fastlio2_odometry_unavailable";
      } else {
        applyTask(Task::kGoafExploration, "ch7_edge", false);
      }
    } else if (shaft_edge && shaft_task_available_) {
      applyTask(Task::kShaftExploration, "ch11_edge", false);
    } else if (shaft_edge) {
      reason = "shaft_task_unavailable";
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
         << " ch7_rc=" << goaf_rc_value_
         << " ch11_rc=" << shaft_rc_value_
         << " auto_enabled=" << (auto_enabled_ ? "true" : "false")
         << " rc_age=" << (have_rc_ ? ageSec(last_rc_time_, now) : -1.0)
         << " odom_age=" << (have_odometry_ ? ageSec(last_odometry_time_, now) : -1.0)
         << " mavros_age=" << (have_mavros_state_ ?
             ageSec(last_mavros_state_time_, now) : -1.0)
         << " shaft_status_age=" << (have_shaft_status_ ?
             ageSec(last_shaft_status_time_, now) : -1.0)
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
  auto_enabled_ = task != Task::kHold;
  return_home_requested_ = request_return;
  last_reason_ = reason;
  offboard_seen_for_task_ = task != Task::kHold && px4_mode_ == "OFFBOARD";
  external_mode_exit_pending_ = false;
  if (task == Task::kGoafExploration) {
    goaf_finished_ = false;
  }
  if (task == Task::kShaftExploration) {
    shaft_started_ = false;
    shaft_status_.clear();
    shaft_activation_time_ = ros::Time::now();
  }
  if (task == Task::kHold && previous_task != Task::kHold) {
    // A switch movement that began while the old task was active must not
    // complete its debounce after the task ends and launch a new task.
    rearm_pending_ = true;
    goaf_edge_ = RcEdge();
    shaft_edge_ = RcEdge();
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
    case Task::kHold:
    default:
      return "hold";
  }
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
