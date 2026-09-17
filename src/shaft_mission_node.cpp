#include <cmath>
#include <string>

#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>

#include "mine_uav_control/shaft_mission.hpp"

namespace {

class ShaftMissionNode {
 public:
  ShaftMissionNode() : private_nh_("~"), mission_(loadConfig()) {
    private_nh_.param("input_timeout", input_timeout_, 0.4);
    private_nh_.param("control_rate", control_rate_, 20.0);
    enable_sub_ = nh_.subscribe("/mine_uav/mission/shaft_enable", 1,
                                &ShaftMissionNode::enableCallback, this);
    depth_sub_ = nh_.subscribe("/mine_uav/shaft/relative_depth_m", 10,
                               &ShaftMissionNode::depthCallback, this);
    range_sub_ = nh_.subscribe("/mine_uav/shaft/bottom_range", 10,
                               &ShaftMissionNode::rangeCallback, this);
    command_pub_ = nh_.advertise<geometry_msgs::TwistStamped>(
        "/mine_uav/shaft/velocity_intent_enu", 10);
    status_pub_ = nh_.advertise<std_msgs::String>(
        "/mine_uav/shaft/status", 1, true);
    timer_ = nh_.createTimer(ros::Duration(1.0 / control_rate_),
                             &ShaftMissionNode::tick, this);
    ROS_WARN("Shaft mission emits abstract velocity intent only; no PX4 output is connected");
  }

 private:
  mine_uav_control::ShaftMission::Config loadConfig() {
    mine_uav_control::ShaftMission::Config config;
    private_nh_.param("descent_speed", config.descent_speed, config.descent_speed);
    private_nh_.param("return_speed", config.return_speed, config.return_speed);
    private_nh_.param("bottom_trigger", config.bottom_trigger, config.bottom_trigger);
    private_nh_.param("bottom_stop_margin", config.bottom_stop_margin,
                      config.bottom_stop_margin);
    private_nh_.param("bottom_confirm_seconds", config.bottom_confirm_seconds,
                      config.bottom_confirm_seconds);
    private_nh_.param("max_depth", config.max_depth, config.max_depth);
    private_nh_.param("max_duration", config.max_duration, config.max_duration);
    private_nh_.param("entrance_tolerance", config.entrance_tolerance,
                      config.entrance_tolerance);
    private_nh_.param("max_depth_jump", config.max_depth_jump,
                      config.max_depth_jump);
    private_nh_.param("max_acceleration", config.max_acceleration,
                      config.max_acceleration);
    return config;
  }

  void enableCallback(const std_msgs::Bool::ConstPtr& message) {
    enabled_ = message->data;
  }

  void depthCallback(const std_msgs::Float64::ConstPtr& message) {
    depth_ = message->data;
    depth_time_ = ros::Time::now();
  }

  void rangeCallback(const sensor_msgs::Range::ConstPtr& message) {
    range_ = message->range;
    range_min_ = message->min_range;
    range_max_ = message->max_range;
    range_time_ = ros::Time::now();
  }

  void tick(const ros::TimerEvent& event) {
    const ros::Time now = ros::Time::now();
    mine_uav_control::ShaftMission::Input in;
    in.enabled = enabled_;
    in.depth_valid = !depth_time_.isZero() &&
                     (now - depth_time_).toSec() <= input_timeout_;
    in.depth = depth_;
    in.range_fresh = !range_time_.isZero() &&
                     (now - range_time_).toSec() <= input_timeout_;
    in.bottom_range = range_;
    in.range_min = range_min_;
    in.range_max = range_max_;
    in.dt = (event.current_real - event.last_real).toSec();
    const auto result = mission_.step(in);
    std_msgs::String status;
    switch (result.state) {
      case mine_uav_control::ShaftMission::State::kIdle:
        status.data = "IDLE";
        break;
      case mine_uav_control::ShaftMission::State::kDescending:
        status.data = "DESCENDING";
        break;
      case mine_uav_control::ShaftMission::State::kReturning:
        status.data = "RETURNING";
        break;
      case mine_uav_control::ShaftMission::State::kComplete:
        status.data = "COMPLETE";
        break;
      case mine_uav_control::ShaftMission::State::kFault:
        status.data = "FAULT_NO_SAFE_AUTONOMOUS_RECOVERY";
        break;
    }
    status_pub_.publish(status);
    if (!result.command_valid) {
      return;
    }
    geometry_msgs::TwistStamped command;
    command.header.stamp = now;
    command.header.frame_id = "map";
    command.twist.linear.z = result.vertical_speed_enu;
    command_pub_.publish(command);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber enable_sub_;
  ros::Subscriber depth_sub_;
  ros::Subscriber range_sub_;
  ros::Publisher command_pub_;
  ros::Publisher status_pub_;
  ros::Timer timer_;
  mine_uav_control::ShaftMission mission_;
  ros::Time depth_time_;
  ros::Time range_time_;
  double depth_{0.0};
  double range_{INFINITY};
  double range_min_{0.0};
  double range_max_{30.0};
  double input_timeout_{0.4};
  double control_rate_{20.0};
  bool enabled_{false};
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "shaft_mission_node");
  ShaftMissionNode node;
  ros::spin();
  return 0;
}
