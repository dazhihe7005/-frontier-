#include <cmath>
#include <string>

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

namespace {

class Task1PerceptionGate {
 public:
  Task1PerceptionGate(const ros::NodeHandle& nh,
                      const ros::NodeHandle& private_nh)
      : nh_(nh), private_nh_(private_nh) {
    private_nh_.param("input_cloud_topic", input_cloud_topic_,
                      std::string("/cloud_registered"));
    private_nh_.param("input_odom_topic", input_odom_topic_,
                      std::string("/Odometry"));
    private_nh_.param("output_cloud_topic", output_cloud_topic_,
                      std::string("/mine_uav/task1/cloud_registered"));
    private_nh_.param("output_odom_topic", output_odom_topic_,
                      std::string("/mine_uav/task1/odometry"));
    private_nh_.param("mission_enable_topic", mission_enable_topic_,
                      std::string("/mine_uav/mission/goaf_enable"));
    private_nh_.param("vision_health_topic", vision_health_topic_,
                      std::string("/mine_uav/task1/vision_healthy"));
    private_nh_.param("expected_frame", expected_frame_,
                      std::string("camera_init"));
    private_nh_.param("require_vision_healthy", require_vision_healthy_, true);

    cloud_publisher_ =
        nh_.advertise<sensor_msgs::PointCloud2>(output_cloud_topic_, 2);
    odom_publisher_ = nh_.advertise<nav_msgs::Odometry>(output_odom_topic_, 20);
    status_publisher_ = nh_.advertise<std_msgs::String>(
        "/mine_uav/task1/perception_gate_status", 1, true);
    mission_subscriber_ = nh_.subscribe(
        mission_enable_topic_, 2, &Task1PerceptionGate::missionCallback, this);
    vision_subscriber_ = nh_.subscribe(
        vision_health_topic_, 2, &Task1PerceptionGate::visionCallback, this);
    cloud_subscriber_ = nh_.subscribe(
        input_cloud_topic_, 2, &Task1PerceptionGate::cloudCallback, this);
    odom_subscriber_ = nh_.subscribe(
        input_odom_topic_, 20, &Task1PerceptionGate::odomCallback, this);

    publishStatus("CLOSED_WAIT_TASK1");
    ROS_INFO("Task-one perception gate: raw localization remains available to "
             "PX4, planner input opens only while task one is enabled");
  }

 private:
  bool open() const {
    return mission_enabled_ &&
           (!require_vision_healthy_ || vision_healthy_);
  }

  void missionCallback(const std_msgs::Bool::ConstPtr& message) {
    const bool was_open = open();
    mission_enabled_ = message->data;
    updateStatus(was_open);
  }

  void visionCallback(const std_msgs::Bool::ConstPtr& message) {
    const bool was_open = open();
    vision_healthy_ = message->data;
    updateStatus(was_open);
  }

  void updateStatus(bool was_open) {
    if (was_open != open()) {
      ++generation_;
      ROS_INFO("Task-one perception gate %s (generation %u)",
               open() ? "opened" : "closed", generation_);
    }
    if (!mission_enabled_) {
      publishStatus("CLOSED_WAIT_TASK1");
    } else if (require_vision_healthy_ && !vision_healthy_) {
      publishStatus("CLOSED_WAIT_VISION");
    } else {
      publishStatus("OPEN");
    }
  }

  void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud) {
    if (!open()) {
      return;
    }
    if (!expected_frame_.empty() && cloud->header.frame_id != expected_frame_) {
      ROS_ERROR_THROTTLE(
          1.0, "Task-one perception gate rejected cloud frame '%s', expected '%s'",
          cloud->header.frame_id.c_str(), expected_frame_.c_str());
      publishStatus("FRAME_MISMATCH");
      return;
    }
    cloud_publisher_.publish(cloud);
  }

  void odomCallback(const nav_msgs::Odometry::ConstPtr& odom) {
    if (!open()) {
      return;
    }
    if (!expected_frame_.empty() && odom->header.frame_id != expected_frame_) {
      ROS_ERROR_THROTTLE(
          1.0, "Task-one perception gate rejected odom frame '%s', expected '%s'",
          odom->header.frame_id.c_str(), expected_frame_.c_str());
      publishStatus("FRAME_MISMATCH");
      return;
    }
    const auto& p = odom->pose.pose.position;
    const auto& q = odom->pose.pose.orientation;
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
        !std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) ||
        !std::isfinite(q.w)) {
      ROS_ERROR_THROTTLE(1.0,
                         "Task-one perception gate rejected non-finite odometry");
      publishStatus("INVALID_ODOMETRY");
      return;
    }
    odom_publisher_.publish(odom);
  }

  void publishStatus(const std::string& status) {
    if (status == last_status_) {
      return;
    }
    std_msgs::String message;
    message.data = status;
    status_publisher_.publish(message);
    last_status_ = status;
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber mission_subscriber_;
  ros::Subscriber vision_subscriber_;
  ros::Subscriber cloud_subscriber_;
  ros::Subscriber odom_subscriber_;
  ros::Publisher cloud_publisher_;
  ros::Publisher odom_publisher_;
  ros::Publisher status_publisher_;
  std::string input_cloud_topic_;
  std::string input_odom_topic_;
  std::string output_cloud_topic_;
  std::string output_odom_topic_;
  std::string mission_enable_topic_;
  std::string vision_health_topic_;
  std::string expected_frame_;
  std::string last_status_;
  bool require_vision_healthy_{true};
  bool mission_enabled_{false};
  bool vision_healthy_{false};
  unsigned int generation_{0};
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "task1_perception_gate");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  Task1PerceptionGate gate(nh, private_nh);
  ros::spin();
  return 0;
}
