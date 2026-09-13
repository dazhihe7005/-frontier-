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

  private_nh_.param("rc_switch_channel", rc_switch_channel_, rc_switch_channel_);
  private_nh_.param("low_threshold", low_threshold_, low_threshold_);
  private_nh_.param("high_threshold", high_threshold_, high_threshold_);
  private_nh_.param("switch_stable_time", switch_stable_time_, switch_stable_time_);
  private_nh_.param("rc_timeout", rc_timeout_, rc_timeout_);
  private_nh_.param("odometry_timeout", odometry_timeout_, odometry_timeout_);
  private_nh_.param("decision_rate", decision_rate_, decision_rate_);
  private_nh_.param("require_odometry", require_odometry_, require_odometry_);
  private_nh_.param("require_mavros_connection", require_mavros_connection_,
                    require_mavros_connection_);

  if (rc_switch_channel_ < 0) {
    ROS_WARN("rc_switch_channel must be non-negative; using channel 5 (ROS index)");
    rc_switch_channel_ = 5;
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
  if (!std::isfinite(decision_rate_) || decision_rate_ <= 0.0) {
    decision_rate_ = 10.0;
  }

  rc_subscriber_ = nh_.subscribe(rc_topic_, 10, &MissionScheduler::rcCallback, this);
  odometry_subscriber_ = nh_.subscribe(
      odometry_topic_, 10, &MissionScheduler::odometryCallback, this);
  mavros_state_subscriber_ = nh_.subscribe(
      mavros_state_topic_, 10, &MissionScheduler::mavrosStateCallback, this);

  // Latched outputs let a task node receive the current selection immediately.
  active_task_publisher_ = nh_.advertise<std_msgs::UInt8>(active_task_topic_, 1, true);
  goaf_enable_publisher_ = nh_.advertise<std_msgs::Bool>(goaf_enable_topic_, 1, true);
  shaft_enable_publisher_ = nh_.advertise<std_msgs::Bool>(shaft_enable_topic_, 1, true);
  return_home_publisher_ = nh_.advertise<std_msgs::Bool>(return_home_topic_, 1, true);
  switch_event_publisher_ = nh_.advertise<std_msgs::String>(switch_event_topic_, 10, true);
  status_publisher_ = nh_.advertise<std_msgs::String>(status_topic_, 10);

  timer_ = nh_.createTimer(ros::Duration(1.0 / decision_rate_),
                           &MissionScheduler::timerCallback, this);
  publishOutputs("startup");

  ROS_INFO("mission_scheduler ready: RC ROS channel index %d (physical CH%d), "
           "low->goaf, high->shaft, thresholds [%d, %d], odometry required=%s",
           rc_switch_channel_, rc_switch_channel_ + 1, low_threshold_, high_threshold_,
           require_odometry_ ? "true" : "false");
}

void MissionScheduler::rcCallback(const mavros_msgs::RCIn::ConstPtr& message) {
  last_rc_time_ = ros::Time::now();
  have_rc_ = true;

  if (rc_switch_channel_ >= static_cast<int>(message->channels.size())) {
    rc_value_ = 0;
    rc_selection_ = RcSelection::kInvalid;
    ROS_WARN_THROTTLE(
        2.0, "RC channel index %d is unavailable; received %zu channels",
        rc_switch_channel_, message->channels.size());
    return;
  }

  rc_value_ = message->channels[rc_switch_channel_];
  rc_selection_ = classifyRc(rc_value_);
}

void MissionScheduler::odometryCallback(const nav_msgs::Odometry::ConstPtr&) {
  last_odometry_time_ = ros::Time::now();
  have_odometry_ = true;
}

void MissionScheduler::mavrosStateCallback(const mavros_msgs::State::ConstPtr& message) {
  have_mavros_state_ = true;
  mavros_connected_ = message->connected;
  mavros_armed_ = message->armed;
}

MissionScheduler::RcSelection MissionScheduler::classifyRc(uint16_t value) const {
  if (value > 0 && value <= static_cast<uint16_t>(low_threshold_)) {
    return RcSelection::kGoafExploration;
  }
  if (value >= static_cast<uint16_t>(high_threshold_)) {
    return RcSelection::kShaftExploration;
  }
  return RcSelection::kInvalid;
}

void MissionScheduler::timerCallback(const ros::TimerEvent&) {
  const ros::Time now = ros::Time::now();
  const bool rc_fresh = have_rc_ && ageSec(last_rc_time_, now) <= rc_timeout_;
  const bool odometry_fresh =
      have_odometry_ && ageSec(last_odometry_time_, now) <= odometry_timeout_;
  const bool mavros_ok = !require_mavros_connection_ ||
                         (have_mavros_state_ && mavros_connected_);

  RcSelection desired_selection = RcSelection::kInvalid;
  std::string reason;
  if (!rc_fresh) {
    reason = "rc_lost";
  } else if (rc_selection_ == RcSelection::kInvalid) {
    reason = "rc_switch_invalid_or_mid";
  } else if (!mavros_ok) {
    reason = "mavros_disconnected";
  } else if (require_odometry_ && !odometry_fresh) {
    reason = "fastlio2_odometry_lost";
  } else {
    desired_selection = rc_selection_;
    reason = selectionName(desired_selection);
  }

  if (desired_selection == RcSelection::kInvalid) {
    pending_selection_ = RcSelection::kInvalid;
    if (active_task_ != Task::kHold) {
      applyTask(Task::kHold, reason, true);
    } else if (last_reason_ != reason) {
      last_reason_ = reason;
      publishOutputs(reason);
    }
  } else {
    if (desired_selection == pending_selection_) {
      if (ageSec(pending_since_, now) >= switch_stable_time_) {
        const Task desired_task =
            desired_selection == RcSelection::kGoafExploration
                ? Task::kGoafExploration
                : Task::kShaftExploration;
        if (desired_task != active_task_) {
          applyTask(desired_task, "rc_switch_stable", false);
        }
      }
    } else {
      pending_selection_ = desired_selection;
      pending_since_ = now;
    }
  }

  std::ostringstream status;
  status << "active_task=" << static_cast<int>(active_task_)
         << "(" << taskName(active_task_) << ")"
         << " desired=" << selectionName(desired_selection)
         << " rc=" << rc_value_
         << " rc_age=" << (have_rc_ ? ageSec(last_rc_time_, now) : -1.0)
         << " odom_age=" << (have_odometry_ ? ageSec(last_odometry_time_, now) : -1.0)
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
  return_home_requested_ = request_return;
  last_reason_ = reason;

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

std::string MissionScheduler::selectionName(RcSelection selection) const {
  switch (selection) {
    case RcSelection::kGoafExploration:
      return "goaf_exploration";
    case RcSelection::kShaftExploration:
      return "shaft_exploration";
    case RcSelection::kInvalid:
    default:
      return "invalid";
  }
}

double MissionScheduler::ageSec(const ros::Time& stamp, const ros::Time& now) const {
  if (stamp.isZero()) {
    return std::numeric_limits<double>::infinity();
  }
  return std::max(0.0, (now - stamp).toSec());
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
