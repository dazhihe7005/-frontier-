#pragma once

#include <cstdint>
#include <string>

#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>

namespace mine_uav_control {

class MissionScheduler {
 public:
  MissionScheduler(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);

 private:
  enum class Task : uint8_t {
    kHold = 0,
    kGoafExploration = 1,
    kShaftExploration = 2,
  };

  enum class RcSelection : uint8_t {
    kInvalid = 0,
    kGoafExploration = 1,
    kShaftExploration = 2,
  };

  void rcCallback(const mavros_msgs::RCIn::ConstPtr& message);
  void odometryCallback(const nav_msgs::Odometry::ConstPtr& message);
  void mavrosStateCallback(const mavros_msgs::State::ConstPtr& message);
  void timerCallback(const ros::TimerEvent& event);

  RcSelection classifyRc(uint16_t value) const;
  void applyTask(Task task, const std::string& reason, bool request_return);
  void publishOutputs(const std::string& reason);
  std::string taskName(Task task) const;
  std::string selectionName(RcSelection selection) const;
  double ageSec(const ros::Time& stamp, const ros::Time& now) const;

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  ros::Subscriber rc_subscriber_;
  ros::Subscriber odometry_subscriber_;
  ros::Subscriber mavros_state_subscriber_;

  ros::Publisher active_task_publisher_;
  ros::Publisher auto_enable_publisher_;
  ros::Publisher goaf_enable_publisher_;
  ros::Publisher shaft_enable_publisher_;
  ros::Publisher return_home_publisher_;
  ros::Publisher switch_event_publisher_;
  ros::Publisher status_publisher_;

  ros::Timer timer_;

  std::string rc_topic_;
  std::string odometry_topic_;
  std::string mavros_state_topic_;
  std::string active_task_topic_;
  std::string auto_enable_topic_;
  std::string goaf_enable_topic_;
  std::string shaft_enable_topic_;
  std::string return_home_topic_;
  std::string switch_event_topic_;
  std::string status_topic_;

  int rc_switch_channel_{5};
  int auto_enable_channel_{6};
  int low_threshold_{1300};
  int high_threshold_{1700};
  int auto_enable_threshold_{1700};
  double switch_stable_time_{0.5};
  double auto_enable_stable_time_{0.5};
  double rc_timeout_{1.0};
  double odometry_timeout_{1.0};
  double decision_rate_{10.0};
  bool require_odometry_{true};
  bool require_mavros_connection_{false};
  bool require_auto_enable_low_before_enable_{true};
  bool force_goaf_task_{false};
  bool shaft_task_available_{false};

  bool have_rc_{false};
  bool have_odometry_{false};
  bool have_mavros_state_{false};
  bool mavros_connected_{false};
  bool mavros_armed_{false};

  uint16_t rc_value_{0};
  uint16_t auto_enable_value_{0};
  bool auto_channel_available_{false};
  bool auto_enable_requested_{false};
  bool auto_enable_low_seen_{false};
  bool auto_enabled_{false};
  RcSelection rc_selection_{RcSelection::kInvalid};
  RcSelection pending_selection_{RcSelection::kInvalid};
  Task active_task_{Task::kHold};
  bool return_home_requested_{false};
  std::string last_reason_{"startup"};

  ros::Time last_rc_time_;
  ros::Time last_odometry_time_;
  ros::Time pending_since_;
  ros::Time auto_enable_pending_since_;
};

}  // namespace mine_uav_control
