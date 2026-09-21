#include <cmath>
#include <stdexcept>
#include <string>

#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include "mine_uav_control/ShaftDepthEstimate.h"
#include "mine_uav_control/control_dt.hpp"
#include "mine_uav_control/shaft_mission.hpp"

namespace {

class ShaftMissionNode {
 public:
  ShaftMissionNode() : private_nh_("~"), mission_(loadConfig()) {
    private_nh_.param("input_timeout", input_timeout_, 0.4);
    private_nh_.param("control_rate", control_rate_, 20.0);
    private_nh_.param<std::string>("required_depth_source", required_depth_source_, "");
    private_nh_.param<std::string>("required_range_frame", required_range_frame_, "");
    private_nh_.param("max_depth_sigma_m", max_depth_sigma_m_, 0.25);
    if (!std::isfinite(control_rate_) || control_rate_ <= 0.0 ||
        !std::isfinite(input_timeout_) || input_timeout_ <= 0.0) {
      throw std::invalid_argument("shaft input_timeout and control_rate must be positive");
    }
    enable_sub_ = nh_.subscribe("/mine_uav/mission/shaft_enable", 1,
                                &ShaftMissionNode::enableCallback, this);
    depth_sub_ = nh_.subscribe("/mine_uav/shaft/depth_estimate", 10,
                               &ShaftMissionNode::depthCallback, this);
    range_sub_ = nh_.subscribe("/mine_uav/shaft/bottom_range", 10,
                               &ShaftMissionNode::rangeCallback, this);
    command_pub_ = nh_.advertise<geometry_msgs::TwistStamped>(
        "/mine_uav/shaft/velocity_intent_enu", 10);
    status_pub_ = nh_.advertise<std_msgs::String>(
        "/mine_uav/shaft/status", 1, true);
    gate_pub_ = nh_.advertise<std_msgs::String>(
        "/mine_uav/shaft/input_gate", 1, true);
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

  void depthCallback(const mine_uav_control::ShaftDepthEstimate::ConstPtr& message) {
    depth_ = message->relative_depth_m;
    depth_sigma_ = message->sigma_m;
    depth_source_ = message->source_id;
    depth_good_ = message->valid;
    depth_stamp_ = message->header.stamp;
    depth_time_ = ros::Time::now();
  }

  void rangeCallback(const sensor_msgs::Range::ConstPtr& message) {
    range_ = message->range;
    range_min_ = message->min_range;
    range_max_ = message->max_range;
    range_frame_ = message->header.frame_id;
    range_stamp_ = message->header.stamp;
    range_time_ = ros::Time::now();
  }

  bool freshStamp(const ros::Time& stamp, const ros::Time& now) const {
    if (stamp.isZero()) return false;
    const double age = (now - stamp).toSec();
    return age >= -0.05 && age <= input_timeout_;
  }

  void publishGate(const std::string& value) {
    if (value == last_gate_) return;
    last_gate_ = value;
    std_msgs::String message;
    message.data = value;
    gate_pub_.publish(message);
  }

  void tick(const ros::TimerEvent& event) {
    const ros::Time now = ros::Time::now();
    mine_uav_control::ShaftMission::Input in;
    in.enabled = enabled_;
    const bool configured = !required_depth_source_.empty() &&
        !required_range_frame_.empty() && std::isfinite(max_depth_sigma_m_) &&
        max_depth_sigma_m_ > 0.0 && std::isfinite(input_timeout_) &&
        input_timeout_ > 0.0 && std::isfinite(control_rate_) &&
        control_rate_ > 0.0;
    const bool depth_fresh = !depth_time_.isZero() &&
        (now - depth_time_).toSec() >= -0.05 &&
        (now - depth_time_).toSec() <= input_timeout_ &&
        freshStamp(depth_stamp_, now);
    in.depth_valid = configured && depth_fresh && depth_good_ &&
        depth_source_ == required_depth_source_ && std::isfinite(depth_) &&
        std::isfinite(depth_sigma_) && depth_sigma_ >= 0.0 &&
        depth_sigma_ <= max_depth_sigma_m_;
    in.depth = depth_;
    in.range_fresh = configured && !range_time_.isZero() &&
        (now - range_time_).toSec() >= -0.05 &&
        (now - range_time_).toSec() <= input_timeout_ &&
        freshStamp(range_stamp_, now) &&
        range_frame_ == required_range_frame_;
    in.bottom_range = range_;
    in.range_min = range_min_;
    in.range_max = range_max_;
    const double actual_dt =
        (event.current_real - event.last_real).toSec();
    const double expected_dt =
        (event.current_expected - event.last_expected).toSec();
    in.dt = mine_uav_control::task2ControlDt(actual_dt, expected_dt);
    if (actual_dt >= 0.0 && actual_dt <= 1e-9 && in.dt > 0.0) {
      ROS_WARN_THROTTLE(5.0,
                        "Task2 timer repeated current_real stamp; using "
                        "expected dt %.6f s instead of zero",
                        in.dt);
    }
    if (!configured) publishGate("UNCONFIGURED");
    else if (depth_time_.isZero()) publishGate("WAIT_DEPTH");
    else if (depth_source_ != required_depth_source_) publishGate("SOURCE_MISMATCH");
    else if (!depth_fresh) publishGate("STALE_DEPTH");
    else if (!in.depth_valid) publishGate("BAD_DEPTH_QUALITY");
    else if (!in.range_fresh) publishGate("BAD_OR_STALE_RANGE");
    else publishGate("OPEN");
    const auto result = mission_.step(in);
    if (result.state == mine_uav_control::ShaftMission::State::kFault) {
      ROS_ERROR(
          "ShaftMission fault input: enabled=%d dt=%.6f expected_dt=%.6f "
          "depth_valid=%d depth=%.6f depth_age=%.6f depth_stamp_age=%.6f "
          "range_fresh=%d range=%.6f range=[%.3f,%.3f] "
          "range_age=%.6f range_stamp_age=%.6f gate=%s",
          in.enabled, in.dt,
          (event.current_expected - event.last_expected).toSec(),
          in.depth_valid, in.depth,
          depth_time_.isZero() ? INFINITY : (now - depth_time_).toSec(),
          depth_stamp_.isZero() ? INFINITY : (now - depth_stamp_).toSec(),
          in.range_fresh, in.bottom_range, in.range_min, in.range_max,
          range_time_.isZero() ? INFINITY : (now - range_time_).toSec(),
          range_stamp_.isZero() ? INFINITY : (now - range_stamp_).toSec(),
          last_gate_.c_str());
    }
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
  ros::Publisher gate_pub_;
  ros::Timer timer_;
  mine_uav_control::ShaftMission mission_;
  ros::Time depth_time_;
  ros::Time depth_stamp_;
  ros::Time range_time_;
  ros::Time range_stamp_;
  double depth_{0.0};
  double depth_sigma_{INFINITY};
  bool depth_good_{false};
  std::string depth_source_;
  std::string range_frame_;
  std::string required_depth_source_;
  std::string required_range_frame_;
  std::string last_gate_;
  double range_{INFINITY};
  double range_min_{0.0};
  double range_max_{30.0};
  double input_timeout_{0.4};
  double max_depth_sigma_m_{0.25};
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
