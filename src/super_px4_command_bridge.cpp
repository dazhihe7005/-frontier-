#include <algorithm>
#include <cmath>
#include <string>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/State.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_srvs/SetBool.h>

namespace {

constexpr double kPi = 3.14159265358979323846;

bool finite(double value) { return std::isfinite(value); }

double normalizeAngle(double angle) {
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

double yawFromQuaternion(const geometry_msgs::Quaternion& q) {
  const double norm =
      std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  if (!finite(norm) || norm < 1e-6) {
    return 0.0;
  }
  const double x = q.x / norm;
  const double y = q.y / norm;
  const double z = q.z / norm;
  const double w = q.w / norm;
  return std::atan2(2.0 * (w * z + x * y),
                    1.0 - 2.0 * (y * y + z * z));
}

double vectorNorm(double x, double y, double z) {
  return std::sqrt(x * x + y * y + z * z);
}

class SuperPx4CommandBridge {
 public:
  SuperPx4CommandBridge(const ros::NodeHandle& nh,
                        const ros::NodeHandle& private_nh)
      : nh_(nh), private_nh_(private_nh) {
    private_nh_.param("command_topic", command_topic_,
                      std::string("/planning/pos_cmd"));
    private_nh_.param("output_topic", output_topic_,
                      std::string("/mine_uav/setpoint_cmd"));
    private_nh_.param("expected_frame", expected_frame_,
                      std::string("camera_init"));
    private_nh_.param("command_timeout", command_timeout_, 0.25);
    private_nh_.param("local_pose_timeout", local_pose_timeout_, 0.5);
    private_nh_.param("output_rate", output_rate_, 50.0);
    private_nh_.param("max_speed", max_speed_, 2.0);
    private_nh_.param("max_acceleration", max_acceleration_, 3.0);
    private_nh_.param("max_horizontal_radius", max_horizontal_radius_, 40.0);
    private_nh_.param("min_height", min_height_, -0.5);
    private_nh_.param("max_height", max_height_, 3.0);
    private_nh_.param("use_acceleration", use_acceleration_, true);

    command_timeout_ = std::max(0.05, command_timeout_);
    local_pose_timeout_ = std::max(0.1, local_pose_timeout_);
    output_rate_ = std::max(10.0, output_rate_);
    max_speed_ = std::max(0.1, max_speed_);
    max_acceleration_ = std::max(0.1, max_acceleration_);
    max_horizontal_radius_ = std::max(1.0, max_horizontal_radius_);
    if (min_height_ > max_height_) {
      std::swap(min_height_, max_height_);
    }

    command_subscriber_ = nh_.subscribe(
        command_topic_, 10, &SuperPx4CommandBridge::commandCallback, this);
    alignment_subscriber_ = nh_.subscribe(
        "/mine_uav/task1/fastlio_to_px4_alignment", 1,
        &SuperPx4CommandBridge::alignmentCallback, this);
    mission_subscriber_ = nh_.subscribe(
        "/mine_uav/mission/goaf_enable", 1,
        &SuperPx4CommandBridge::missionCallback, this);
    vision_subscriber_ = nh_.subscribe(
        "/mine_uav/task1/vision_healthy", 1,
        &SuperPx4CommandBridge::visionCallback, this);
    state_subscriber_ = nh_.subscribe(
        "/mavros/state", 10, &SuperPx4CommandBridge::stateCallback, this);
    local_pose_subscriber_ = nh_.subscribe(
        "/mavros/local_position/pose", 10,
        &SuperPx4CommandBridge::localPoseCallback, this);

    command_publisher_ =
        nh_.advertise<mavros_msgs::PositionTarget>(output_topic_, 10);
    ready_publisher_ = nh_.advertise<std_msgs::Bool>(
        "/mine_uav/task1/command_ready", 1, true);
    status_publisher_ = nh_.advertise<std_msgs::String>(
        "/mine_uav/task1/command_status", 1, true);
    enable_service_ = private_nh_.advertiseService(
        "enable", &SuperPx4CommandBridge::enableCallback, this);
    output_timer_ = nh_.createTimer(
        ros::Duration(1.0 / output_rate_),
        &SuperPx4CommandBridge::outputTimerCallback, this);

    publishReady(false);
    publishStatus("WAIT_ALIGNMENT");
    ROS_WARN("SUPER-to-PX4 command output starts disabled; enable it explicitly "
             "with the private SetBool service after all health gates pass");
  }

 private:
  void alignmentCallback(
      const geometry_msgs::TransformStamped::ConstPtr& transform) {
    const auto& t = transform->transform.translation;
    const auto& q = transform->transform.rotation;
    if (!finite(t.x) || !finite(t.y) || !finite(t.z) || !finite(q.x) ||
        !finite(q.y) || !finite(q.z) || !finite(q.w)) {
      alignment_ready_ = false;
      disableOutput("INVALID_ALIGNMENT");
      return;
    }
    alignment_x_ = t.x;
    alignment_y_ = t.y;
    alignment_z_ = t.z;
    alignment_yaw_ = yawFromQuaternion(q);
    alignment_ready_ = true;
    updateStatus();
  }

  void missionCallback(const std_msgs::Bool::ConstPtr& enabled) {
    if (mission_enabled_ && !enabled->data) {
      disableOutput("TASK1_DESELECTED");
    }
    mission_enabled_ = enabled->data;
    updateStatus();
  }

  void visionCallback(const std_msgs::Bool::ConstPtr& healthy) {
    if (vision_healthy_ && !healthy->data) {
      disableOutput("VISION_UNHEALTHY");
    }
    vision_healthy_ = healthy->data;
    updateStatus();
  }

  void stateCallback(const mavros_msgs::State::ConstPtr& state) {
    if (mavros_connected_ && !state->connected) {
      disableOutput("MAVROS_DISCONNECTED");
    }
    mavros_connected_ = state->connected;
    updateStatus();
  }

  void localPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& pose) {
    const auto& p = pose->pose.position;
    const auto& q = pose->pose.orientation;
    if (!finite(p.x) || !finite(p.y) || !finite(p.z) || !finite(q.x) ||
        !finite(q.y) || !finite(q.z) || !finite(q.w)) {
      return;
    }
    latest_local_pose_ = *pose;
    last_local_pose_time_ = ros::Time::now();
    have_local_pose_ = true;
  }

  bool gatesReady() const {
    return alignment_ready_ && mission_enabled_ && vision_healthy_ &&
           mavros_connected_;
  }

  bool localPoseFresh(const ros::Time& now) const {
    return have_local_pose_ && !last_local_pose_time_.isZero() &&
           (now - last_local_pose_time_).toSec() <= local_pose_timeout_;
  }

  bool enableCallback(std_srvs::SetBool::Request& request,
                      std_srvs::SetBool::Response& response) {
    if (!request.data) {
      manual_enable_ = false;
      response.success = true;
      response.message = "task-one PX4 command output disabled";
      updateStatus();
      return true;
    }
    const ros::Time now = ros::Time::now();
    if (!gatesReady() || !valid_command_ ||
        !localPoseFresh(now) ||
        (now - last_command_time_).toSec() > command_timeout_) {
      manual_enable_ = false;
      response.success = false;
      response.message =
          "refused: health gates, local pose, or fresh SUPER command is not ready";
      updateStatus();
      return true;
    }
    manual_enable_ = true;
    response.success = true;
    response.message =
        "task-one command output enabled; this does not arm or enter OFFBOARD";
    updateStatus();
    return true;
  }

  bool validCommand(const quadrotor_msgs::PositionCommand& command,
                    std::string* reason) const {
    if (!expected_frame_.empty() &&
        command.header.frame_id != expected_frame_) {
      *reason = "FRAME_MISMATCH";
      return false;
    }
    const auto& p = command.position;
    const auto& v = command.velocity;
    const auto& a = command.acceleration;
    if (!finite(p.x) || !finite(p.y) || !finite(p.z) || !finite(v.x) ||
        !finite(v.y) || !finite(v.z) || !finite(a.x) || !finite(a.y) ||
        !finite(a.z) || !finite(command.yaw) || !finite(command.yaw_dot)) {
      *reason = "NONFINITE_COMMAND";
      return false;
    }
    if (command.trajectory_flag !=
            quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY &&
        command.trajectory_flag !=
            quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_EMER) {
      *reason = "TRAJECTORY_NOT_ACTIVE";
      return false;
    }
    if (vectorNorm(v.x, v.y, v.z) > max_speed_) {
      *reason = "SPEED_LIMIT";
      return false;
    }
    if (vectorNorm(a.x, a.y, a.z) > max_acceleration_) {
      *reason = "ACCELERATION_LIMIT";
      return false;
    }
    return true;
  }

  mavros_msgs::PositionTarget convert(
      const quadrotor_msgs::PositionCommand& command) const {
    const double c = std::cos(alignment_yaw_);
    const double s = std::sin(alignment_yaw_);
    mavros_msgs::PositionTarget output;
    output.header.stamp = ros::Time::now();
    output.header.frame_id = "map";
    output.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    output.type_mask = 0;

    output.position.x = c * command.position.x - s * command.position.y +
                        alignment_x_;
    output.position.y = s * command.position.x + c * command.position.y +
                        alignment_y_;
    output.position.z = command.position.z + alignment_z_;
    output.velocity.x =
        c * command.velocity.x - s * command.velocity.y;
    output.velocity.y =
        s * command.velocity.x + c * command.velocity.y;
    output.velocity.z = command.velocity.z;
    output.acceleration_or_force.x =
        c * command.acceleration.x - s * command.acceleration.y;
    output.acceleration_or_force.y =
        s * command.acceleration.x + c * command.acceleration.y;
    output.acceleration_or_force.z = command.acceleration.z;
    output.yaw = normalizeAngle(command.yaw + alignment_yaw_);
    output.yaw_rate = command.yaw_dot;
    if (!use_acceleration_) {
      output.type_mask |= mavros_msgs::PositionTarget::IGNORE_AFX |
                          mavros_msgs::PositionTarget::IGNORE_AFY |
                          mavros_msgs::PositionTarget::IGNORE_AFZ;
    }
    return output;
  }

  bool insideFlightVolume(const mavros_msgs::PositionTarget& command,
                          std::string* reason) const {
    const double horizontal_radius =
        std::hypot(command.position.x, command.position.y);
    if (horizontal_radius > max_horizontal_radius_) {
      *reason = "HORIZONTAL_GEOFENCE";
      return false;
    }
    if (command.position.z < min_height_ || command.position.z > max_height_) {
      *reason = "HEIGHT_GEOFENCE";
      return false;
    }
    return true;
  }

  void commandCallback(
      const quadrotor_msgs::PositionCommand::ConstPtr& command) {
    last_command_time_ = ros::Time::now();
    std::string reason;
    if (!validCommand(*command, &reason)) {
      valid_command_ = false;
      if (reason == "TRAJECTORY_NOT_ACTIVE") {
        publishStatus(reason);
      } else {
        disableOutput(reason);
      }
      return;
    }
    const auto output = convert(*command);
    if (!insideFlightVolume(output, &reason)) {
      valid_command_ = false;
      disableOutput(reason);
      return;
    }
    valid_command_ = true;
    latest_target_ = output;
    updateStatus();
  }

  mavros_msgs::PositionTarget makeHoldTarget(const ros::Time& now) const {
    mavros_msgs::PositionTarget hold;
    hold.header.stamp = now;
    hold.header.frame_id = "map";
    hold.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    hold.type_mask = mavros_msgs::PositionTarget::IGNORE_VX |
                     mavros_msgs::PositionTarget::IGNORE_VY |
                     mavros_msgs::PositionTarget::IGNORE_VZ |
                     mavros_msgs::PositionTarget::IGNORE_AFX |
                     mavros_msgs::PositionTarget::IGNORE_AFY |
                     mavros_msgs::PositionTarget::IGNORE_AFZ |
                     mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
    hold.position = latest_local_pose_.pose.position;
    hold.yaw = yawFromQuaternion(latest_local_pose_.pose.orientation);
    return hold;
  }

  void outputTimerCallback(const ros::TimerEvent&) {
    const ros::Time now = ros::Time::now();
    if (!manual_enable_ || !gatesReady()) {
      updateStatus();
      return;
    }
    if (!localPoseFresh(now)) {
      disableOutput("LOCAL_POSE_TIMEOUT");
      return;
    }

    const bool command_fresh =
        valid_command_ && !last_command_time_.isZero() &&
        (now - last_command_time_).toSec() <= command_timeout_;
    if (command_fresh) {
      latest_target_.header.stamp = now;
      command_publisher_.publish(latest_target_);
      publishReady(true);
      publishStatus("STREAMING");
    } else {
      valid_command_ = false;
      command_publisher_.publish(makeHoldTarget(now));
      publishReady(true);
      publishStatus("HOLD_COMMAND_TIMEOUT");
    }
  }

  void disableOutput(const std::string& reason) {
    if (manual_enable_) {
      ROS_ERROR("Task-one PX4 command output latched off: %s", reason.c_str());
    }
    manual_enable_ = false;
    publishReady(false);
    publishStatus(reason);
  }

  void updateStatus() {
    const bool ready = manual_enable_ && gatesReady() && valid_command_;
    publishReady(ready);
    if (!alignment_ready_) {
      publishStatus("WAIT_ALIGNMENT");
    } else if (!mission_enabled_) {
      publishStatus("WAIT_TASK1_SELECTION");
    } else if (!vision_healthy_) {
      publishStatus("WAIT_VISION");
    } else if (!mavros_connected_) {
      publishStatus("WAIT_MAVROS");
    } else if (!localPoseFresh(ros::Time::now())) {
      publishStatus("WAIT_LOCAL_POSE");
    } else if (!manual_enable_) {
      publishStatus("WAIT_OPERATOR_ENABLE");
    } else if (!valid_command_) {
      publishStatus("WAIT_SUPER_COMMAND");
    } else {
      publishStatus("READY");
    }
  }

  void publishReady(bool ready) {
    if (ready_published_ && ready == last_ready_) {
      return;
    }
    std_msgs::Bool message;
    message.data = ready;
    ready_publisher_.publish(message);
    last_ready_ = ready;
    ready_published_ = true;
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
  ros::Subscriber command_subscriber_;
  ros::Subscriber alignment_subscriber_;
  ros::Subscriber mission_subscriber_;
  ros::Subscriber vision_subscriber_;
  ros::Subscriber state_subscriber_;
  ros::Subscriber local_pose_subscriber_;
  ros::Publisher command_publisher_;
  ros::Publisher ready_publisher_;
  ros::Publisher status_publisher_;
  ros::ServiceServer enable_service_;
  ros::Timer output_timer_;

  geometry_msgs::PoseStamped latest_local_pose_;
  mavros_msgs::PositionTarget latest_target_;
  std::string command_topic_;
  std::string output_topic_;
  std::string expected_frame_;
  std::string last_status_;
  ros::Time last_command_time_;
  ros::Time last_local_pose_time_;
  double command_timeout_{0.25};
  double local_pose_timeout_{0.5};
  double output_rate_{50.0};
  double max_speed_{2.0};
  double max_acceleration_{3.0};
  double max_horizontal_radius_{40.0};
  double min_height_{-0.5};
  double max_height_{3.0};
  double alignment_x_{0.0};
  double alignment_y_{0.0};
  double alignment_z_{0.0};
  double alignment_yaw_{0.0};
  bool use_acceleration_{true};
  bool alignment_ready_{false};
  bool mission_enabled_{false};
  bool vision_healthy_{false};
  bool mavros_connected_{false};
  bool manual_enable_{false};
  bool valid_command_{false};
  bool have_local_pose_{false};
  bool ready_published_{false};
  bool last_ready_{false};
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "super_px4_command_bridge");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  SuperPx4CommandBridge bridge(nh, private_nh);
  ros::spin();
  return 0;
}
