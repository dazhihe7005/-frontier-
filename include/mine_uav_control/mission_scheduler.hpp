#pragma once

#include <cstdint>
#include <string>

#include <geometry_msgs/TransformStamped.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt64.h>

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

  enum class RcLevel : uint8_t { kInvalid = 0, kLow = 1, kHigh = 2 };
  enum class RcEvent : uint8_t { kNone = 0, kRising = 1, kFalling = 2 };

  struct RcEdge {
    RcLevel stable{RcLevel::kInvalid};
    RcLevel pending{RcLevel::kInvalid};
    ros::Time pending_since;
    bool initialized{false};
  };

  void rcCallback(const mavros_msgs::RCIn::ConstPtr& message);
  void odometryCallback(const nav_msgs::Odometry::ConstPtr& message);
  void alignmentCallback(
      const geometry_msgs::TransformStamped::ConstPtr& message);
  void mavrosStateCallback(const mavros_msgs::State::ConstPtr& message);
  void commandFaultCallback(const std_msgs::String::ConstPtr& message);
  void initialGoalAcceptedCallback(
      const std_msgs::UInt64::ConstPtr& message);
  void goafFinishedCallback(const std_msgs::Bool::ConstPtr& message);
  void shaftStatusCallback(const std_msgs::String::ConstPtr& message);
  void timerCallback(const ros::TimerEvent& event);

  RcLevel classifyRc(uint16_t value) const;
  RcEvent updateEdge(RcEdge* edge, RcLevel level, const ros::Time& now);
  void requestGoafStart(const ros::Time& now, bool odometry_fresh,
                        bool mavros_state_fresh);
  void updateGoafStartCheck(const ros::Time& now, bool odometry_fresh,
                            bool mavros_state_fresh);
  void cancelGoafStart(const std::string& reason);
  void applyTask(Task task, const std::string& reason, bool request_return);
  void publishOutputs(const std::string& reason);
  std::string taskName(Task task) const;
  double ageSec(const ros::Time& stamp, const ros::Time& now) const;

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  ros::Subscriber rc_subscriber_;
  ros::Subscriber odometry_subscriber_;
  ros::Subscriber alignment_subscriber_;
  ros::Subscriber mavros_state_subscriber_;
  ros::Subscriber command_fault_subscriber_;
  ros::Subscriber initial_goal_accepted_subscriber_;
  ros::Subscriber goaf_finished_subscriber_;
  ros::Subscriber shaft_status_subscriber_;

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
  std::string alignment_topic_;
  std::string expected_planning_frame_;
  std::string mavros_state_topic_;
  std::string command_fault_topic_;
  std::string initial_goal_accepted_topic_;
  std::string active_task_topic_;
  std::string auto_enable_topic_;
  std::string goaf_enable_topic_;
  std::string shaft_enable_topic_;
  std::string return_home_topic_;
  std::string switch_event_topic_;
  std::string status_topic_;

  int goaf_trigger_channel_{6};
  int shaft_trigger_channel_{10};
  int low_threshold_{1300};
  int high_threshold_{1700};
  double switch_stable_time_{0.5};
  double rc_timeout_{1.0};
  double odometry_timeout_{1.0};
  double mavros_state_timeout_{2.5};
  double shaft_status_timeout_{0.6};
  double shaft_start_timeout_{4.0};
  double decision_rate_{10.0};
  double start_max_speed_{0.25};
  double start_max_vertical_speed_{0.10};
  double start_stable_time_{1.0};
  double start_check_timeout_{15.0};
  double initial_goal_accept_timeout_{30.0};
  bool require_odometry_{true};
  bool require_mavros_connection_{false};
  bool shaft_task_available_{false};

  bool have_rc_{false};
  bool have_odometry_{false};
  bool odometry_frame_valid_{false};
  bool have_speed_estimate_{false};
  bool alignment_ready_{false};
  bool have_mavros_state_{false};
  bool have_shaft_status_{false};
  bool mavros_connected_{false};
  bool mavros_armed_{false};

  uint16_t goaf_rc_value_{0};
  uint16_t shaft_rc_value_{0};
  bool goaf_channel_available_{false};
  bool shaft_channel_available_{false};
  bool auto_enabled_{false};
  bool goaf_start_pending_{false};
  bool task_ready_reset_seen_{false};
  RcEdge goaf_edge_;
  RcEdge shaft_edge_;
  bool rearm_pending_{false};
  Task active_task_{Task::kHold};
  bool goaf_finished_{false};
  std::string shaft_status_;
  bool shaft_started_{false};
  std::string px4_mode_;
  bool offboard_seen_for_task_{false};
  bool external_mode_exit_pending_{false};
  bool return_home_requested_{false};
  double estimated_speed_{0.0};
  double estimated_vertical_speed_{0.0};
  nav_msgs::Odometry latest_odometry_;
  uint64_t initial_goal_accepted_id_{0};
  std::string command_fault_;
  std::string last_reason_{"startup"};

  ros::Time last_rc_time_;
  ros::Time last_odometry_time_;
  ros::Time goaf_start_check_started_;
  ros::Time goaf_stable_since_;
  ros::Time goaf_activation_time_;
  ros::Time last_mavros_state_time_;
  ros::Time last_shaft_status_time_;
  ros::Time shaft_activation_time_;
};

}  // namespace mine_uav_control
