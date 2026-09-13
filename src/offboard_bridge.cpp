#include <cmath>

#include <ros/ros.h>
#include <mavros_msgs/PositionTarget.h>

namespace {

constexpr double kSetpointTimeoutSec = 0.2;

class OffboardBridge {
 public:
  OffboardBridge(const ros::NodeHandle& nh, const ros::NodeHandle& pnh)
      : nh_(nh), pnh_(pnh) {
    double rate_hz = 50.0;
    pnh_.param("rate", rate_hz, rate_hz);
    if (!std::isfinite(rate_hz) || rate_hz <= 0.0) {
      ROS_WARN("Invalid bridge rate; using 50 Hz");
      rate_hz = 50.0;
    }

    command_sub_ = nh_.subscribe("/mine_uav/setpoint_cmd", 10,
                                 &OffboardBridge::commandCallback, this);
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

  void timerCallback(const ros::TimerEvent&) {
    if (!have_command_) {
      ROS_WARN_THROTTLE(1.0,
                        "offboard_bridge has not received a setpoint yet");
      return;
    }

    const double age = (ros::Time::now() - last_update_).toSec();
    if (age > kSetpointTimeoutSec) {
      ROS_WARN_THROTTLE(
          1.0,
          "Setpoint command is %.3f s old; stopping publication of old target",
          age);
      return;
    }

    mavros_pub_.publish(latest_command_);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber command_sub_;
  ros::Publisher mavros_pub_;
  ros::Timer timer_;
  mavros_msgs::PositionTarget latest_command_;
  ros::Time last_update_;
  bool have_command_{false};
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
