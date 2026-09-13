#include <algorithm>
#include <cmath>
#include <string>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>

namespace {

constexpr double kPi = 3.14159265358979323846;

bool finite(double value) { return std::isfinite(value); }

bool finiteQuaternion(const geometry_msgs::Quaternion& q) {
  return finite(q.x) && finite(q.y) && finite(q.z) && finite(q.w) &&
         (q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w) > 1e-8;
}

geometry_msgs::Quaternion normalized(geometry_msgs::Quaternion q) {
  const double norm =
      std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  q.x /= norm;
  q.y /= norm;
  q.z /= norm;
  q.w /= norm;
  return q;
}

double yawFromQuaternion(const geometry_msgs::Quaternion& input) {
  const auto q = normalized(input);
  const double sin_yaw = 2.0 * (q.w * q.z + q.x * q.y);
  const double cos_yaw = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(sin_yaw, cos_yaw);
}

geometry_msgs::Quaternion yawQuaternion(double yaw) {
  geometry_msgs::Quaternion q;
  q.z = std::sin(0.5 * yaw);
  q.w = std::cos(0.5 * yaw);
  return q;
}

geometry_msgs::Quaternion multiply(const geometry_msgs::Quaternion& a,
                                   const geometry_msgs::Quaternion& b) {
  geometry_msgs::Quaternion q;
  q.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
  q.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
  q.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
  q.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;
  return normalized(q);
}

double squaredDistance(const geometry_msgs::Point& a,
                       const geometry_msgs::Point& b) {
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  const double dz = a.z - b.z;
  return dx * dx + dy * dy + dz * dz;
}

class FastlioPx4VisionBridge {
 public:
  FastlioPx4VisionBridge(const ros::NodeHandle& nh,
                         const ros::NodeHandle& private_nh)
      : nh_(nh), private_nh_(private_nh) {
    private_nh_.param("input_topic", input_topic_, std::string("/Odometry"));
    private_nh_.param("output_topic", output_topic_,
                      std::string("/mavros/vision_pose/pose_cov"));
    private_nh_.param("input_frame", input_frame_,
                      std::string("camera_init"));
    private_nh_.param("output_frame", output_frame_, std::string("odom"));
    private_nh_.param("output_child_frame", output_child_frame_,
                      std::string("base_link"));
    private_nh_.param("publish_rate", publish_rate_, 30.0);
    private_nh_.param("data_timeout", data_timeout_, 0.3);
    private_nh_.param("max_input_jump", max_input_jump_, 2.0);
    private_nh_.param("position_stddev", position_stddev_, 0.05);
    private_nh_.param("orientation_stddev", orientation_stddev_, 0.10);
    private_nh_.param("zero_initial_xy", zero_initial_xy_, true);
    private_nh_.param("zero_initial_z", zero_initial_z_, true);
    private_nh_.param("zero_initial_yaw", zero_initial_yaw_, true);
    private_nh_.param("require_mavros_connection", require_mavros_connection_,
                      true);
    private_nh_.param("allow_alignment_while_armed",
                      allow_alignment_while_armed_, false);

    if (!finite(publish_rate_) || publish_rate_ < 1.0) {
      publish_rate_ = 30.0;
    }
    if (!finite(data_timeout_) || data_timeout_ <= 0.1) {
      data_timeout_ = 0.3;
    }
    if (!finite(max_input_jump_) || max_input_jump_ <= 0.0) {
      max_input_jump_ = 2.0;
    }
    position_stddev_ = std::max(0.01, position_stddev_);
    orientation_stddev_ = std::max(0.01, orientation_stddev_);

    odometry_subscriber_ = nh_.subscribe(
        input_topic_, 10, &FastlioPx4VisionBridge::odometryCallback, this);
    mavros_state_subscriber_ = nh_.subscribe(
        "/mavros/state", 10, &FastlioPx4VisionBridge::mavrosStateCallback,
        this);
    pose_publisher_ =
        nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(output_topic_,
                                                                 10);
    alignment_publisher_ = nh_.advertise<geometry_msgs::TransformStamped>(
        "/mine_uav/task1/fastlio_to_px4_alignment", 1, true);
    healthy_publisher_ = nh_.advertise<std_msgs::Bool>(
        "/mine_uav/task1/vision_healthy", 1, true);
    status_publisher_ = nh_.advertise<std_msgs::String>(
        "/mine_uav/task1/vision_status", 1, true);
    reset_service_ = private_nh_.advertiseService(
        "reset_alignment", &FastlioPx4VisionBridge::resetAlignment, this);
    publish_timer_ = nh_.createTimer(ros::Duration(1.0 / publish_rate_),
                                     &FastlioPx4VisionBridge::timerCallback,
                                     this);

    publishHealth(false);
    publishStatus("WAIT_ODOMETRY");
  }

 private:
  void mavrosStateCallback(const mavros_msgs::State::ConstPtr& state) {
    mavros_connected_ = state->connected;
    mavros_armed_ = state->armed;
    have_mavros_state_ = true;
  }

  bool validOdometry(const nav_msgs::Odometry& odometry) const {
    const auto& p = odometry.pose.pose.position;
    if (!finite(p.x) || !finite(p.y) || !finite(p.z) ||
        !finiteQuaternion(odometry.pose.pose.orientation)) {
      return false;
    }
    return input_frame_.empty() || odometry.header.frame_id == input_frame_;
  }

  void captureAlignment(const nav_msgs::Odometry& odometry) {
    origin_ = odometry.pose.pose.position;
    initial_yaw_ = zero_initial_yaw_
                       ? yawFromQuaternion(odometry.pose.pose.orientation)
                       : 0.0;
    alignment_ready_ = true;
    jump_detected_ = false;

    geometry_msgs::TransformStamped transform;
    transform.header.stamp = ros::Time::now();
    transform.header.frame_id = output_frame_;
    transform.child_frame_id = input_frame_;
    const double c = std::cos(initial_yaw_);
    const double s = std::sin(initial_yaw_);
    const double ox = zero_initial_xy_ ? origin_.x : 0.0;
    const double oy = zero_initial_xy_ ? origin_.y : 0.0;
    transform.transform.translation.x = -(c * ox + s * oy);
    transform.transform.translation.y = -(-s * ox + c * oy);
    transform.transform.translation.z = zero_initial_z_ ? -origin_.z : 0.0;
    transform.transform.rotation = yawQuaternion(-initial_yaw_);
    alignment_publisher_.publish(transform);
    publishStatus("ALIGNMENT_READY");
    ROS_INFO("Captured task-one alignment at [%.3f %.3f %.3f], yaw %.2f deg",
             origin_.x, origin_.y, origin_.z,
             initial_yaw_ * 180.0 / kPi);
  }

  void odometryCallback(const nav_msgs::Odometry::ConstPtr& odometry) {
    if (!validOdometry(*odometry)) {
      publishStatus("INVALID_ODOMETRY");
      ROS_WARN_THROTTLE(1.0,
                        "Rejecting invalid Fast-LIO odometry or frame mismatch");
      return;
    }
    if (!alignment_ready_) {
      if (!allow_alignment_while_armed_ && have_mavros_state_ &&
          mavros_armed_) {
        publishStatus("WAIT_DISARMED");
        ROS_ERROR_THROTTLE(
            1.0, "Will not capture a new vision alignment while PX4 is armed");
        return;
      }
      captureAlignment(*odometry);
    }
    if (have_previous_input_ &&
        squaredDistance(previous_input_position_,
                        odometry->pose.pose.position) >
            max_input_jump_ * max_input_jump_) {
      jump_detected_ = true;
      publishStatus("INPUT_JUMP");
      ROS_ERROR("Fast-LIO position jumped more than %.2f m; vision output stopped",
                max_input_jump_);
    }
    previous_input_position_ = odometry->pose.pose.position;
    have_previous_input_ = true;
    latest_odometry_ = *odometry;
    last_input_time_ = ros::Time::now();
    have_odometry_ = true;
  }

  geometry_msgs::PoseWithCovarianceStamped makeVisionPose(
      const ros::Time& stamp) const {
    geometry_msgs::PoseWithCovarianceStamped output;
    output.header.stamp = stamp;
    output.header.frame_id = output_frame_;

    const auto& input_pose = latest_odometry_.pose.pose;
    const double dx = input_pose.position.x -
                      (zero_initial_xy_ ? origin_.x : 0.0);
    const double dy = input_pose.position.y -
                      (zero_initial_xy_ ? origin_.y : 0.0);
    const double c = std::cos(initial_yaw_);
    const double s = std::sin(initial_yaw_);
    output.pose.pose.position.x = c * dx + s * dy;
    output.pose.pose.position.y = -s * dx + c * dy;
    output.pose.pose.position.z =
        input_pose.position.z - (zero_initial_z_ ? origin_.z : 0.0);
    output.pose.pose.orientation =
        multiply(yawQuaternion(-initial_yaw_), input_pose.orientation);

    const double position_variance = position_stddev_ * position_stddev_;
    const double orientation_variance =
        orientation_stddev_ * orientation_stddev_;
    output.pose.covariance.fill(0.0);
    output.pose.covariance[0] =
        std::max(position_variance, latest_odometry_.pose.covariance[0]);
    output.pose.covariance[7] =
        std::max(position_variance, latest_odometry_.pose.covariance[7]);
    output.pose.covariance[14] =
        std::max(position_variance, latest_odometry_.pose.covariance[14]);
    output.pose.covariance[21] =
        std::max(orientation_variance, latest_odometry_.pose.covariance[21]);
    output.pose.covariance[28] =
        std::max(orientation_variance, latest_odometry_.pose.covariance[28]);
    output.pose.covariance[35] =
        std::max(orientation_variance, latest_odometry_.pose.covariance[35]);
    return output;
  }

  void timerCallback(const ros::TimerEvent&) {
    const ros::Time now = ros::Time::now();
    const bool data_fresh =
        have_odometry_ && !last_input_time_.isZero() &&
        (now - last_input_time_).toSec() <= data_timeout_;
    const bool mavros_ok = !require_mavros_connection_ ||
                           (have_mavros_state_ && mavros_connected_);
    const bool healthy = alignment_ready_ && data_fresh && mavros_ok &&
                         !jump_detected_;
    publishHealth(healthy);
    if (!healthy) {
      if (!alignment_ready_ || !have_odometry_) {
        publishStatus("WAIT_ODOMETRY");
      } else if (!data_fresh) {
        publishStatus("ODOMETRY_TIMEOUT");
      } else if (!mavros_ok) {
        publishStatus("MAVROS_DISCONNECTED");
      }
      return;
    }
    pose_publisher_.publish(makeVisionPose(now));
    publishStatus("STREAMING");
  }

  void publishHealth(bool healthy) {
    if (health_published_ && healthy == last_health_) {
      return;
    }
    std_msgs::Bool message;
    message.data = healthy;
    healthy_publisher_.publish(message);
    last_health_ = healthy;
    health_published_ = true;
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

  bool resetAlignment(std_srvs::Trigger::Request&,
                      std_srvs::Trigger::Response& response) {
    if (!allow_alignment_while_armed_ && have_mavros_state_ && mavros_armed_) {
      response.success = false;
      response.message = "PX4 is armed; refusing to reset vision alignment";
      return true;
    }
    alignment_ready_ = false;
    have_previous_input_ = false;
    jump_detected_ = false;
    publishHealth(false);
    publishStatus("WAIT_ODOMETRY");
    response.success = true;
    response.message = "alignment cleared; next valid odometry will set origin";
    return true;
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber odometry_subscriber_;
  ros::Subscriber mavros_state_subscriber_;
  ros::Publisher pose_publisher_;
  ros::Publisher alignment_publisher_;
  ros::Publisher healthy_publisher_;
  ros::Publisher status_publisher_;
  ros::ServiceServer reset_service_;
  ros::Timer publish_timer_;

  nav_msgs::Odometry latest_odometry_;
  geometry_msgs::Point origin_;
  geometry_msgs::Point previous_input_position_;
  ros::Time last_input_time_;
  std::string input_topic_;
  std::string output_topic_;
  std::string input_frame_;
  std::string output_frame_;
  std::string output_child_frame_;
  std::string last_status_;
  double publish_rate_{30.0};
  double data_timeout_{0.3};
  double max_input_jump_{2.0};
  double position_stddev_{0.05};
  double orientation_stddev_{0.10};
  double initial_yaw_{0.0};
  bool zero_initial_xy_{true};
  bool zero_initial_z_{true};
  bool zero_initial_yaw_{true};
  bool require_mavros_connection_{true};
  bool allow_alignment_while_armed_{false};
  bool have_odometry_{false};
  bool have_previous_input_{false};
  bool alignment_ready_{false};
  bool jump_detected_{false};
  bool have_mavros_state_{false};
  bool mavros_connected_{false};
  bool mavros_armed_{false};
  bool health_published_{false};
  bool last_health_{false};
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "fastlio_px4_vision_bridge");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  FastlioPx4VisionBridge bridge(nh, private_nh);
  ros::spin();
  return 0;
}
