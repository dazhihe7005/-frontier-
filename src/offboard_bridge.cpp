#include <cmath>
#include <string>

#include <ros/ros.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/State.h>

namespace {

constexpr double kSetpointTimeoutSec = 0.2;

class OffboardBridge {
 public:
  OffboardBridge(const ros::NodeHandle& nh, const ros::NodeHandle& pnh)
      : nh_(nh), pnh_(pnh) {
    double rate_hz = 50.0;
    pnh_.param("rate", rate_hz, rate_hz);
    pnh_.param("offboard_mode", offboard_mode_, std::string("OFFBOARD"));
    if (!std::isfinite(rate_hz) || rate_hz <= 0.0) {
      ROS_WARN("Invalid bridge rate; using 50 Hz");
      rate_hz = 50.0;
    }

    command_sub_ = nh_.subscribe("/mine_uav/setpoint_cmd", 10,
                                 &OffboardBridge::commandCallback, this);
    state_sub_ = nh_.subscribe("/mavros/state", 10,
                               &OffboardBridge::stateCallback, this);
    mavros_pub_ = nh_.advertise<mavros_msgs::PositionTarget>(
        "/mavros/setpoint_raw/local", 10);
    timer_ = nh_.createTimer(ros::Duration(1.0 / rate_hz),
                             &OffboardBridge::timerCallback, this);
  }

 private:
  void commandCallback(const mavros_msgs::PositionTarget::ConstPtr& command) {
    latest_command_ = *command;
    last_update_ = ros::Time::now();
    have_command_ = true;
  }

  void stateCallback(const mavros_msgs::State::ConstPtr& state) {
    latest_state_ = *state;
    have_state_ = true;
  }

  void timerCallback(const ros::TimerEvent&) {
    // A silent upstream is expected after the managed bridge has switched PX4
    // out of OFFBOARD. Keep the warning active while flight still depends on
    // this stream, including startup before the first MAVROS state arrives.
    const bool offboard_stream_required =
        !have_state_ ||
        (latest_state_.connected && latest_state_.mode == offboard_mode_);
    if (!have_command_) {
      if (offboard_stream_required) {
        ROS_WARN_THROTTLE(1.0,
                          "offboard_bridge has not received a setpoint yet");
      }
      return;
    }

    const double age = (ros::Time::now() - last_update_).toSec();
    if (age > kSetpointTimeoutSec) {
      if (offboard_stream_required) {
        ROS_WARN_THROTTLE(
            1.0,
            "Setpoint command is %.3f s old; stopping publication of old target",
            age);
      }
      return;
    }

    mavros_pub_.publish(latest_command_);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber command_sub_;
  ros::Subscriber state_sub_;
  ros::Publisher mavros_pub_;
  ros::Timer timer_;
  mavros_msgs::PositionTarget latest_command_;
  mavros_msgs::State latest_state_;
  ros::Time last_update_;
  std::string offboard_mode_{"OFFBOARD"};
  bool have_command_{false};
  bool have_state_{false};
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "offboard_bridge");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  OffboardBridge bridge(nh, pnh);
  ros::spin();
  return 0;
}
