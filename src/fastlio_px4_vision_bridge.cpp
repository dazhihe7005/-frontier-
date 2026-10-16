#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>

#include "mine_uav_control/fastlio_mount_alignment.hpp"

namespace {

constexpr double kPi = 3.14159265358979323846;

bool finite(double value) { return std::isfinite(value); }

bool finiteQuaternion(const geometry_msgs::Quaternion& q) {
  return finite(q.x) && finite(q.y) && finite(q.z) && finite(q.w) &&
         (q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w) > 1e-8;
}

mine_uav_control::Quaternion toMathQuaternion(
    const geometry_msgs::Quaternion& q) {
  return mine_uav_control::normalized(
      mine_uav_control::Quaternion(q.x, q.y, q.z, q.w));
}

geometry_msgs::Quaternion toRosQuaternion(
    const mine_uav_control::Quaternion& input) {
  const auto q = mine_uav_control::normalized(input);
  geometry_msgs::Quaternion output;
  output.x = q.x;
  output.y = q.y;
  output.z = q.z;
  output.w = q.w;
  return output;
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
    private_nh_.param("timestamp_timeout", timestamp_timeout_, 0.6);
    private_nh_.param("max_input_jump", max_input_jump_, 2.0);
    private_nh_.param("max_input_speed", max_input_speed_, 2.0);
    private_nh_.param("max_input_angular_speed", max_input_angular_speed_,
                      2.0);
    private_nh_.param("max_prediction_horizon", max_prediction_horizon_,
                      0.35);
    private_nh_.param("prediction_position_noise_rate",
                      prediction_position_noise_rate_, 0.15);
    private_nh_.param("prediction_orientation_noise_rate",
                      prediction_orientation_noise_rate_, 0.20);
    private_nh_.param("position_stddev", position_stddev_, 0.05);
    private_nh_.param("orientation_stddev", orientation_stddev_, 0.10);
    private_nh_.param("max_input_position_variance",
                      max_input_position_variance_, 4.0);
    private_nh_.param("require_fastlio_health", require_fastlio_health_, true);
    private_nh_.param("fastlio_health_topic", fastlio_health_topic_,
                      std::string("/fast_lio/localization_healthy"));
    private_nh_.param("fastlio_health_timeout", fastlio_health_timeout_, 0.5);
    private_nh_.param("zero_initial_xy", zero_initial_xy_, true);
    private_nh_.param("zero_initial_z", zero_initial_z_, true);
    private_nh_.param("zero_initial_yaw", zero_initial_yaw_, true);
    private_nh_.param("zero_initial_roll_pitch", zero_initial_roll_pitch_,
                      true);
    double temporary_sensor_roll_in_body_deg = 0.0;
    double temporary_sensor_pitch_in_body_deg = 0.0;
    double temporary_sensor_yaw_in_body_deg = 0.0;
    private_nh_.param("temporary_sensor_roll_in_body_deg",
                      temporary_sensor_roll_in_body_deg, 0.0);
    private_nh_.param("temporary_sensor_pitch_in_body_deg",
                      temporary_sensor_pitch_in_body_deg, 0.0);
    private_nh_.param("temporary_sensor_yaw_in_body_deg",
                      temporary_sensor_yaw_in_body_deg, 0.0);
    private_nh_.param("sensor_position_in_body_x",
                      sensor_position_in_body_.x, 0.0);
    private_nh_.param("sensor_position_in_body_y",
                      sensor_position_in_body_.y, 0.0);
    private_nh_.param("sensor_position_in_body_z",
                      sensor_position_in_body_.z, 0.0);
    private_nh_.param("require_mavros_connection", require_mavros_connection_,
                      true);
    private_nh_.param("allow_alignment_while_armed",
                      allow_alignment_while_armed_, false);
    private_nh_.param("auto_realign_after_odometry_timeout",
                      auto_realign_after_odometry_timeout_, true);

    if (!finite(publish_rate_) || publish_rate_ < 1.0) {
      publish_rate_ = 30.0;
    }
    if (!finite(data_timeout_) || data_timeout_ <= 0.1) {
      data_timeout_ = 0.3;
    }
    timestamp_timeout_ = std::max(0.3, timestamp_timeout_);
    if (!finite(max_input_jump_) || max_input_jump_ <= 0.0) {
      max_input_jump_ = 2.0;
    }
    max_input_speed_ = std::max(0.1, max_input_speed_);
    max_input_angular_speed_ = std::max(0.1, max_input_angular_speed_);
    max_prediction_horizon_ = std::max(
        0.0, std::min(timestamp_timeout_, max_prediction_horizon_));
    prediction_position_noise_rate_ =
        std::max(0.0, prediction_position_noise_rate_);
    prediction_orientation_noise_rate_ =
        std::max(0.0, prediction_orientation_noise_rate_);
    max_input_position_variance_ =
        std::max(0.01, max_input_position_variance_);
    fastlio_health_timeout_ = std::max(0.1, fastlio_health_timeout_);
    if (!finite(temporary_sensor_roll_in_body_deg) ||
        !finite(temporary_sensor_pitch_in_body_deg) ||
        !finite(temporary_sensor_yaw_in_body_deg) ||
        std::abs(temporary_sensor_roll_in_body_deg) > 180.0 ||
        std::abs(temporary_sensor_pitch_in_body_deg) > 180.0 ||
        std::abs(temporary_sensor_yaw_in_body_deg) > 180.0) {
      throw std::invalid_argument(
          "temporary sensor mount angles must be finite and within "
          "[-180, 180]");
    }
    temporary_sensor_roll_in_body_ =
        temporary_sensor_roll_in_body_deg * kPi / 180.0;
    temporary_sensor_pitch_in_body_ =
        temporary_sensor_pitch_in_body_deg * kPi / 180.0;
    temporary_sensor_yaw_in_body_ =
        temporary_sensor_yaw_in_body_deg * kPi / 180.0;
    if (!finite(sensor_position_in_body_.x) ||
        !finite(sensor_position_in_body_.y) ||
        !finite(sensor_position_in_body_.z) ||
        std::sqrt(sensor_position_in_body_.x * sensor_position_in_body_.x +
                  sensor_position_in_body_.y * sensor_position_in_body_.y +
                  sensor_position_in_body_.z * sensor_position_in_body_.z) >
            1.0) {
      throw std::invalid_argument(
          "sensor_position_in_body must be finite and within 1 metre");
    }
    position_stddev_ = std::max(0.01, position_stddev_);
    orientation_stddev_ = std::max(0.01, orientation_stddev_);

    if (std::abs(temporary_sensor_roll_in_body_) > 1e-9 ||
        std::abs(temporary_sensor_pitch_in_body_) > 1e-9 ||
        std::abs(temporary_sensor_yaw_in_body_) > 1e-9) {
      ROS_WARN("Radar mounting compensation enabled: sensor R/P/Y in body = "
               "[%.1f %.1f %.1f] deg. Keep only the angles that remain in "
               "the physical installation.",
               temporary_sensor_roll_in_body_deg,
               temporary_sensor_pitch_in_body_deg,
               temporary_sensor_yaw_in_body_deg);
    }

    odometry_subscriber_ = nh_.subscribe(
        input_topic_, 10, &FastlioPx4VisionBridge::odometryCallback, this);
    mavros_state_subscriber_ = nh_.subscribe(
        "/mavros/state", 10, &FastlioPx4VisionBridge::mavrosStateCallback,
        this);
    fastlio_health_subscriber_ = nh_.subscribe(
        fastlio_health_topic_, 10,
        &FastlioPx4VisionBridge::fastlioHealthCallback, this);
    pose_publisher_ =
        nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(output_topic_,
                                                                 10);
    alignment_publisher_ = nh_.advertise<geometry_msgs::TransformStamped>(
        "/mine_uav/task1/fastlio_to_px4_alignment", 1, true);
    sensor_position_publisher_ = nh_.advertise<geometry_msgs::Vector3Stamped>(
        "/mine_uav/task1/fastlio_sensor_position_in_body", 1, true);
    sensor_rotation_publisher_ = nh_.advertise<geometry_msgs::QuaternionStamped>(
        "/mine_uav/task1/fastlio_sensor_from_body_rotation", 1, true);
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

  void fastlioHealthCallback(const std_msgs::Bool::ConstPtr& health) {
    fastlio_localization_healthy_ = health->data;
    have_fastlio_health_ = true;
    last_fastlio_health_time_ = ros::Time::now();
  }

  bool fastlioHealthReady(const ros::Time& now) const {
    if (!require_fastlio_health_) {
      return true;
    }
    return have_fastlio_health_ && fastlio_localization_healthy_ &&
           !last_fastlio_health_time_.isZero() &&
           (now - last_fastlio_health_time_).toSec() <=
               fastlio_health_timeout_;
  }

  bool inputCovarianceHealthy(const nav_msgs::Odometry& odometry) const {
    for (const int index : {0, 7, 14}) {
      const double variance = odometry.pose.covariance[index];
      if (!finite(variance) || variance < 0.0 ||
          variance > max_input_position_variance_) {
        return false;
      }
    }
    return true;
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
    const auto mount_alignment =
        mine_uav_control::makeMountAlignment(
            toMathQuaternion(odometry.pose.pose.orientation),
            temporary_sensor_roll_in_body_, temporary_sensor_pitch_in_body_,
            temporary_sensor_yaw_in_body_, zero_initial_roll_pitch_,
            zero_initial_yaw_);
    map_to_output_ = mount_alignment.map_to_output;
    sensor_from_body_ = mount_alignment.sensor_from_body;
    const auto initial_body_in_map = mine_uav_control::multiply(
        toMathQuaternion(odometry.pose.pose.orientation), sensor_from_body_);
    initial_sensor_offset_in_map_ = mine_uav_control::rotateVector(
        initial_body_in_map, sensor_position_in_body_);
    alignment_ready_ = true;
    jump_detected_ = false;
    input_velocity_ = geometry_msgs::Point();

    geometry_msgs::TransformStamped transform;
    transform.header.stamp = ros::Time::now();
    transform.header.frame_id = output_frame_;
    transform.child_frame_id = input_frame_;
    const auto rotated_origin = mine_uav_control::rotateVector(
        map_to_output_, mine_uav_control::Vector3(
            zero_initial_xy_ ? origin_.x : 0.0,
            zero_initial_xy_ ? origin_.y : 0.0,
            zero_initial_z_ ? origin_.z : 0.0));
    transform.transform.translation.x = -rotated_origin.x;
    transform.transform.translation.y = -rotated_origin.y;
    transform.transform.translation.z = -rotated_origin.z;
    transform.transform.rotation = toRosQuaternion(map_to_output_);
    alignment_publisher_.publish(transform);
    geometry_msgs::Vector3Stamped sensor_position;
    sensor_position.header = transform.header;
    sensor_position.header.frame_id = output_child_frame_;
    sensor_position.vector.x = sensor_position_in_body_.x;
    sensor_position.vector.y = sensor_position_in_body_.y;
    sensor_position.vector.z = sensor_position_in_body_.z;
    sensor_position_publisher_.publish(sensor_position);
    geometry_msgs::QuaternionStamped sensor_rotation;
    sensor_rotation.header = transform.header;
    sensor_rotation.header.frame_id = input_frame_;
    sensor_rotation.quaternion = toRosQuaternion(sensor_from_body_);
    sensor_rotation_publisher_.publish(sensor_rotation);
    publishStatus("ALIGNMENT_READY");
    ROS_INFO("Captured task-one 3-D alignment at [%.3f %.3f %.3f], "
             "map-to-PX4 yaw %.2f deg, sensor-in-body R/P/Y "
             "[%.2f %.2f %.2f] deg, body-to-sensor lever "
             "[%.3f %.3f %.3f] m",
             origin_.x, origin_.y, origin_.z,
             mine_uav_control::yawFromQuaternion(map_to_output_) *
                 180.0 / kPi,
             temporary_sensor_roll_in_body_ * 180.0 / kPi,
             temporary_sensor_pitch_in_body_ * 180.0 / kPi,
             temporary_sensor_yaw_in_body_ * 180.0 / kPi,
             sensor_position_in_body_.x, sensor_position_in_body_.y,
             sensor_position_in_body_.z);
  }

  void odometryCallback(const nav_msgs::Odometry::ConstPtr& odometry) {
    if (!validOdometry(*odometry)) {
      publishStatus("INVALID_ODOMETRY");
      ROS_WARN_THROTTLE(1.0,
                        "Rejecting invalid Fast-LIO odometry or frame mismatch");
      return;
    }
    if (odometry_outage_seen_ && auto_realign_after_odometry_timeout_) {
      if (!allow_alignment_while_armed_ && have_mavros_state_ &&
          mavros_armed_) {
        publishHealth(false);
        publishStatus("WAIT_DISARMED_AFTER_ODOMETRY_RESTART");
        return;
      }
      odometry_outage_seen_ = false;
      alignment_ready_ = false;
      have_previous_input_ = false;
      jump_detected_ = false;
      ROS_WARN("Fast-LIO odometry recovered after an outage; capturing a fresh "
               "alignment while PX4 is disarmed");
    }
    if (!alignment_ready_) {
      if (!allow_alignment_while_armed_ && have_mavros_state_ &&
          mavros_armed_) {
        publishStatus("WAIT_DISARMED");
        ROS_ERROR_THROTTLE(
            1.0, "Will not capture a new vision alignment while PX4 is armed");
        return;
      }
      if (!fastlioHealthReady(ros::Time::now())) {
        publishStatus("WAIT_FASTLIO_HEALTH");
        return;
      }
      if (!inputCovarianceHealthy(*odometry)) {
        publishStatus("WAIT_FASTLIO_COVARIANCE");
        return;
      }
      captureAlignment(*odometry);
    }
    const double displacement_squared = have_previous_input_ ?
        squaredDistance(previous_input_position_,
                        odometry->pose.pose.position) : 0.0;
    if (have_previous_input_ &&
        displacement_squared > max_input_jump_ * max_input_jump_) {
      jump_detected_ = true;
      publishStatus("INPUT_JUMP");
      ROS_ERROR("Fast-LIO position jumped more than %.2f m; vision output stopped",
                max_input_jump_);
    }
    input_velocity_ = geometry_msgs::Point();
    input_orientation_delta_ = mine_uav_control::Quaternion();
    input_orientation_delta_dt_ = 0.0;
    if (have_previous_input_ && !jump_detected_) {
      const double dt = (odometry->header.stamp - previous_input_stamp_).toSec();
      if (dt >= 0.03 && dt <= 0.3) {
        input_velocity_.x = (odometry->pose.pose.position.x -
                             previous_input_position_.x) / dt;
        input_velocity_.y = (odometry->pose.pose.position.y -
                             previous_input_position_.y) / dt;
        input_velocity_.z = (odometry->pose.pose.position.z -
                             previous_input_position_.z) / dt;
        const double speed = std::sqrt(
            input_velocity_.x * input_velocity_.x +
            input_velocity_.y * input_velocity_.y +
            input_velocity_.z * input_velocity_.z);
        if (speed > max_input_speed_) {
          const double scale = max_input_speed_ / speed;
          input_velocity_.x *= scale;
          input_velocity_.y *= scale;
          input_velocity_.z *= scale;
        }
        input_orientation_delta_ = mine_uav_control::multiply(
            toMathQuaternion(odometry->pose.pose.orientation),
            mine_uav_control::conjugate(previous_input_orientation_));
        input_orientation_delta_dt_ = dt;
      }
    }
    previous_input_position_ = odometry->pose.pose.position;
    previous_input_orientation_ =
        toMathQuaternion(odometry->pose.pose.orientation);
    previous_input_stamp_ = odometry->header.stamp;
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
    const double sample_age = std::max(
        0.0, std::min(max_prediction_horizon_,
                      (stamp - latest_odometry_.header.stamp).toSec()));
    const double predicted_x = input_pose.position.x +
                               input_velocity_.x * sample_age;
    const double predicted_y = input_pose.position.y +
                               input_velocity_.y * sample_age;
    const double predicted_z = input_pose.position.z +
                               input_velocity_.z * sample_age;
    auto predicted_sensor_orientation =
        toMathQuaternion(input_pose.orientation);
    if (input_orientation_delta_dt_ > 0.0 && sample_age > 0.0) {
      const double measured_angle =
          mine_uav_control::quaternionAngularDistance(
              mine_uav_control::Quaternion(), input_orientation_delta_);
      if (measured_angle > 1e-9) {
        const double measured_rate =
            measured_angle / input_orientation_delta_dt_;
        const double predicted_angle =
            std::min(measured_rate, max_input_angular_speed_) * sample_age;
        predicted_sensor_orientation = mine_uav_control::multiply(
            mine_uav_control::scaledRotation(
                input_orientation_delta_, predicted_angle / measured_angle),
            predicted_sensor_orientation);
      }
    }
    const auto predicted_body_in_map = mine_uav_control::multiply(
        predicted_sensor_orientation, sensor_from_body_);
    const auto sensor_offset_in_map = mine_uav_control::rotateVector(
        predicted_body_in_map, sensor_position_in_body_);
    const double dx = predicted_x - sensor_offset_in_map.x -
        (zero_initial_xy_
             ? origin_.x - initial_sensor_offset_in_map_.x
             : 0.0);
    const double dy = predicted_y - sensor_offset_in_map.y -
        (zero_initial_xy_
             ? origin_.y - initial_sensor_offset_in_map_.y
             : 0.0);
    const double dz = predicted_z - sensor_offset_in_map.z -
        (zero_initial_z_
             ? origin_.z - initial_sensor_offset_in_map_.z
             : 0.0);
    const auto rotated_position = mine_uav_control::rotateVector(
        map_to_output_, mine_uav_control::Vector3(dx, dy, dz));
    output.pose.pose.position.x = rotated_position.x;
    output.pose.pose.position.y = rotated_position.y;
    output.pose.pose.position.z = rotated_position.z;
    // Fast-LIO reports the sensor pose. Apply the world-axis alignment on the
    // left and the fixed body-to-sensor rotation on the right so MAVROS gets
    // aircraft attitude as well as correctly rotated position.
    output.pose.pose.orientation = toRosQuaternion(mine_uav_control::multiply(
        mine_uav_control::multiply(map_to_output_,
                                   predicted_sensor_orientation),
        sensor_from_body_));

    const double position_stddev = std::hypot(
        position_stddev_, prediction_position_noise_rate_ * sample_age);
    const double orientation_stddev = std::hypot(
        orientation_stddev_, prediction_orientation_noise_rate_ * sample_age);
    const double position_variance = position_stddev * position_stddev;
    const double orientation_variance =
        orientation_stddev * orientation_stddev;
    output.pose.covariance.fill(0.0);
    const auto rx = mine_uav_control::rotateVector(
        map_to_output_, mine_uav_control::Vector3(1.0, 0.0, 0.0));
    const auto ry = mine_uav_control::rotateVector(
        map_to_output_, mine_uav_control::Vector3(0.0, 1.0, 0.0));
    const auto rz = mine_uav_control::rotateVector(
        map_to_output_, mine_uav_control::Vector3(0.0, 0.0, 1.0));
    const double rotation[3][3] = {
        {rx.x, ry.x, rz.x}, {rx.y, ry.y, rz.y}, {rx.z, ry.z, rz.z}};
    const double offset_skew[3][3] = {
        {0.0, -sensor_offset_in_map.z, sensor_offset_in_map.y},
        {sensor_offset_in_map.z, 0.0, -sensor_offset_in_map.x},
        {-sensor_offset_in_map.y, sensor_offset_in_map.x, 0.0}};
    double jacobian[6][6] = {};
    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 3; ++col) {
        jacobian[row][col] = rotation[row][col];
        jacobian[row + 3][col + 3] = rotation[row][col];
        for (int axis = 0; axis < 3; ++axis) {
          jacobian[row][col + 3] +=
              rotation[row][axis] * offset_skew[axis][col];
        }
      }
    }
    // Propagate the complete 6x6 covariance. The upper-right Jacobian block
    // accounts for attitude uncertainty moving the off-centre sensor origin.
    for (int output_row = 0; output_row < 6; ++output_row) {
      for (int output_col = 0; output_col < 6; ++output_col) {
        double covariance = 0.0;
        for (int input_row = 0; input_row < 6; ++input_row) {
          for (int input_col = 0; input_col < 6; ++input_col) {
            const double input_covariance =
                latest_odometry_.pose.covariance[input_row * 6 + input_col];
            if (finite(input_covariance)) {
              covariance += jacobian[output_row][input_row] *
                            input_covariance *
                            jacobian[output_col][input_col];
            }
          }
        }
        output.pose.covariance[output_row * 6 + output_col] = covariance;
      }
    }
    for (int axis = 0; axis < 3; ++axis) {
      output.pose.covariance[axis * 7] = std::max(
          position_variance, output.pose.covariance[axis * 7]);
      output.pose.covariance[(axis + 3) * 7] = std::max(
          orientation_variance,
          output.pose.covariance[(axis + 3) * 7]);
    }
    return output;
  }

  void timerCallback(const ros::TimerEvent&) {
    const ros::Time now = ros::Time::now();
    const bool data_fresh =
        have_odometry_ && !last_input_time_.isZero() &&
        (now - last_input_time_).toSec() <= data_timeout_;
    const double source_age = have_odometry_
        ? (now - latest_odometry_.header.stamp).toSec()
        : timestamp_timeout_ + 1.0;
    const bool timestamp_healthy = source_age >= -0.1 &&
                                   source_age <= timestamp_timeout_;
    const bool fastlio_ok = fastlioHealthReady(now);
    const bool covariance_ok = have_odometry_ &&
                               inputCovarianceHealthy(latest_odometry_);
    const bool mavros_ok = !require_mavros_connection_ ||
                           (have_mavros_state_ && mavros_connected_);
    const bool healthy = alignment_ready_ && data_fresh &&
                         timestamp_healthy && fastlio_ok && covariance_ok &&
                         mavros_ok && !jump_detected_;
    publishHealth(healthy);
    if (!healthy) {
      if (!have_odometry_) {
        publishStatus("WAIT_ODOMETRY");
      } else if (!data_fresh) {
        if (auto_realign_after_odometry_timeout_) {
          odometry_outage_seen_ = true;
          alignment_ready_ = false;
          have_previous_input_ = false;
          jump_detected_ = false;
        }
        publishStatus("ODOMETRY_TIMEOUT");
      } else if (!timestamp_healthy) {
        publishStatus("ODOMETRY_TIMESTAMP_STALE");
      } else if (!fastlio_ok) {
        const bool health_fresh = have_fastlio_health_ &&
            !last_fastlio_health_time_.isZero() &&
            (now - last_fastlio_health_time_).toSec() <=
                fastlio_health_timeout_;
        publishStatus(health_fresh ? "FASTLIO_DEGENERATE" :
                                     "FASTLIO_HEALTH_TIMEOUT");
      } else if (!covariance_ok) {
        publishStatus("FASTLIO_COVARIANCE_REJECTED");
      } else if (!alignment_ready_) {
        publishStatus("WAIT_ODOMETRY");
      } else if (!mavros_ok) {
        publishStatus("MAVROS_DISCONNECTED");
      }
      return;
    }
    // Predict the 10 Hz Fast-LIO position and attitude to this 30 Hz output
    // tick, bound that prediction, and inflate covariance by its horizon
    // before restamping for PX4's fusion horizon.
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
  ros::Subscriber fastlio_health_subscriber_;
  ros::Publisher pose_publisher_;
  ros::Publisher alignment_publisher_;
  ros::Publisher sensor_position_publisher_;
  ros::Publisher sensor_rotation_publisher_;
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
  std::string fastlio_health_topic_;
  std::string last_status_;
  double publish_rate_{30.0};
  double data_timeout_{0.3};
  double timestamp_timeout_{0.6};
  double max_input_jump_{2.0};
  double max_input_speed_{2.0};
  double max_input_angular_speed_{2.0};
  double max_prediction_horizon_{0.35};
  double prediction_position_noise_rate_{0.15};
  double prediction_orientation_noise_rate_{0.20};
  double max_input_position_variance_{4.0};
  double fastlio_health_timeout_{0.5};
  double position_stddev_{0.05};
  double orientation_stddev_{0.10};
  mine_uav_control::Quaternion map_to_output_;
  mine_uav_control::Quaternion sensor_from_body_;
  mine_uav_control::Quaternion previous_input_orientation_;
  mine_uav_control::Quaternion input_orientation_delta_;
  mine_uav_control::Vector3 sensor_position_in_body_;
  mine_uav_control::Vector3 initial_sensor_offset_in_map_;
  double input_orientation_delta_dt_{0.0};
  double temporary_sensor_roll_in_body_{0.0};
  double temporary_sensor_pitch_in_body_{0.0};
  double temporary_sensor_yaw_in_body_{0.0};
  bool zero_initial_xy_{true};
  bool zero_initial_z_{true};
  bool zero_initial_yaw_{true};
  bool zero_initial_roll_pitch_{true};
  bool require_mavros_connection_{true};
  bool require_fastlio_health_{true};
  bool allow_alignment_while_armed_{false};
  bool auto_realign_after_odometry_timeout_{true};
  bool have_odometry_{false};
  bool have_previous_input_{false};
  bool alignment_ready_{false};
  bool jump_detected_{false};
  bool odometry_outage_seen_{false};
  bool have_mavros_state_{false};
  bool have_fastlio_health_{false};
  bool fastlio_localization_healthy_{false};
  bool mavros_connected_{false};
  bool mavros_armed_{false};
  bool health_published_{false};
  ros::Time previous_input_stamp_;
  ros::Time last_fastlio_health_time_;
  geometry_msgs::Point input_velocity_;
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
