#include <cmath>
#include <memory>

#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/PositionTarget.h>
#include <ros/ros.h>

#include "mine_uav_control/arc_trajectory.hpp"

namespace {

class ArcTrajectoryNode {
 public:
  ArcTrajectoryNode(const ros::NodeHandle& nh, const ros::NodeHandle& pnh)
      : nh_(nh), pnh_(pnh) {
    pnh_.param("radius", config_.radius, config_.radius);
    pnh_.param("speed", config_.speed, config_.speed);
    pnh_.param("arc_angle_deg", config_.arc_angle_deg, config_.arc_angle_deg);
    pnh_.param("direction", config_.direction, config_.direction);
    pnh_.param("rate", config_.rate, config_.rate);

    setpoint_pub_ = nh_.advertise<mavros_msgs::PositionTarget>(
        "/mine_uav/setpoint_cmd", 10);
    pose_sub_ = nh_.subscribe("/mavros/local_position/pose", 10,
                              &ArcTrajectoryNode::poseCallback, this);
  }

  bool validConfiguration() const {
    if (!std::isfinite(config_.rate) || config_.rate <= 0.0) {
      ROS_ERROR("rate must be finite and greater than zero");
      return false;
    }
    return true;
  }

  void spin() {
    ros::Rate rate(config_.rate);
    while (ros::ok()) {
      ros::spinOnce();
      if (trajectory_) {
        const double elapsed = (ros::Time::now() - trajectory_start_).toSec();
        publish(trajectory_->sample(elapsed));
      }
      rate.sleep();
    }
  }

 private:
  void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& pose) {
    if (trajectory_) {
      return;
    }

    const auto& q = pose->pose.orientation;
    const double yaw = std::atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    mine_uav_control::Pose2D start;
    start.x = pose->pose.position.x;
    start.y = pose->pose.position.y;
    start.z = pose->pose.position.z;
    start.yaw = yaw;

    auto candidate = std::make_shared<mine_uav_control::ArcTrajectory>(
        start, config_);
    if (!candidate->valid()) {
      ROS_ERROR_STREAM("Cannot create arc trajectory: " << candidate->error());
      return;
    }

    trajectory_ = candidate;
    trajectory_start_ = ros::Time::now();
    ROS_INFO("Arc trajectory initialized from current local pose");
  }

  void publish(const mine_uav_control::TrajectoryPoint& point) {
    mavros_msgs::PositionTarget message;
    message.header.stamp = ros::Time::now();
    message.header.frame_id = "local_enu";

    // MAVROS' local raw setpoint interface performs the ENU-to-NED conversion.
    message.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    message.type_mask = mavros_msgs::PositionTarget::IGNORE_AFX |
                        mavros_msgs::PositionTarget::IGNORE_AFY |
                        mavros_msgs::PositionTarget::IGNORE_AFZ |
                        mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

    message.position.x = point.x;
    message.position.y = point.y;
    message.position.z = point.z;
    message.velocity.x = point.vx;
    message.velocity.y = point.vy;
    message.velocity.z = point.vz;
    message.yaw = static_cast<float>(point.yaw);
    setpoint_pub_.publish(message);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher setpoint_pub_;
  ros::Subscriber pose_sub_;
  mine_uav_control::ArcConfig config_;
  std::shared_ptr<mine_uav_control::ArcTrajectory> trajectory_;
  ros::Time trajectory_start_;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "arc_trajectory_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  ArcTrajectoryNode node(nh, pnh);
  if (!node.validConfiguration()) {
    return 1;
  }
  node.spin();
  return 0;
}
