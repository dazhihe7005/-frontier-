#include <algorithm>
#include <cmath>
#include <deque>
#include <stdexcept>
#include <string>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>

#include "mine_uav_control/lio_mount_alignment.hpp"

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

struct GravitySample {
  ros::Time received_at;
  mine_uav_control::Vector3 direction;
  double acceleration_norm{0.0};
  double angular_speed{0.0};
};

class LioPx4VisionBridge {
 public:
  LioPx4VisionBridge(const ros::NodeHandle& nh,
                     const ros::NodeHandle& private_nh)
      : nh_(nh), private_nh_(private_nh) {
    private_nh_.param("input_topic", input_topic_,
                      std::string("/omnilivo/odometry"));
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
    private_nh_.param("max_physical_angular_speed",
                      max_physical_angular_speed_, 4.0);
    private_nh_.param("max_input_orientation_jump",
                      max_input_orientation_jump_, 0.75);
    private_nh_.param("high_angular_rate_orientation_stddev",
                      high_angular_rate_orientation_stddev_, 0.35);
    private_nh_.param("input_rate_fault_confirm_samples",
                      input_rate_fault_confirm_samples_, 2);
    private_nh_.param("jump_recovery_stable_samples",
                      jump_recovery_stable_samples_, 5);
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
    if (!private_nh_.getParam("require_localization_health",
                              require_localization_health_)) {
      private_nh_.param("require_fastlio_health",
                        require_localization_health_, true);
    }
    if (!private_nh_.getParam("localization_health_topic",
                              localization_health_topic_)) {
      private_nh_.param("fastlio_health_topic", localization_health_topic_,
                        std::string("/omnilivo/localization_healthy"));
    }
    if (!private_nh_.getParam("localization_health_timeout",
                              localization_health_timeout_)) {
      private_nh_.param("fastlio_health_timeout",
                        localization_health_timeout_, 0.5);
    }
    private_nh_.param("localization_unhealthy_grace",
                      localization_unhealthy_grace_, 0.30);
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
    private_nh_.param("auto_enable_topic", auto_enable_topic_,
                      std::string("/mine_uav/mission/auto_enable"));
    private_nh_.param("allow_alignment_while_armed",
                      allow_alignment_while_armed_, false);
    private_nh_.param("auto_realign_after_odometry_timeout",
                      auto_realign_after_odometry_timeout_, true);
    private_nh_.param("align_vision_heading_to_mavros",
                      align_vision_heading_to_mavros_, false);
    private_nh_.param("mavros_attitude_topic", mavros_attitude_topic_,
                      std::string("/mavros/imu/data"));
    private_nh_.param("mavros_attitude_timeout", mavros_attitude_timeout_,
                      0.5);
    private_nh_.param(
        "vision_heading_alignment_topic", vision_heading_alignment_topic_,
        std::string("/mine_uav/task1/vision_heading_from_planning"));
    private_nh_.param("auto_calibrate_mount_from_gravity",
                      auto_calibrate_mount_from_gravity_, false);
    private_nh_.param("sensor_imu_topic", sensor_imu_topic_,
                      std::string("/livox/imu"));
    private_nh_.param("body_imu_topic", body_imu_topic_,
                      std::string("/mavros/imu/data_raw"));
    private_nh_.param("gravity_alignment_duration",
                      gravity_alignment_duration_, 2.0);
    private_nh_.param("gravity_imu_timeout", gravity_imu_timeout_, 0.2);
    private_nh_.param("gravity_min_samples", gravity_min_samples_, 40);
    private_nh_.param("gravity_max_angular_speed",
                      gravity_max_angular_speed_, 0.15);
    private_nh_.param("gravity_max_direction_spread_deg",
                      gravity_max_direction_spread_deg_, 5.0);
    private_nh_.param("gravity_max_accel_norm_relative_spread",
                      gravity_max_accel_norm_relative_spread_, 0.15);
    private_nh_.param("gravity_alignment_max_tilt_deg",
                      gravity_alignment_max_tilt_deg_, 60.0);

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
    max_physical_angular_speed_ = std::max(
        max_input_angular_speed_ + 0.1, max_physical_angular_speed_);
    max_input_orientation_jump_ = std::max(
        0.1, std::min(kPi, max_input_orientation_jump_));
    high_angular_rate_orientation_stddev_ = std::max(
        orientation_stddev_, high_angular_rate_orientation_stddev_);
    input_rate_fault_confirm_samples_ = std::max(
        1, std::min(10, input_rate_fault_confirm_samples_));
    jump_recovery_stable_samples_ = std::max(
        2, std::min(100, jump_recovery_stable_samples_));
    max_prediction_horizon_ = std::max(
        0.0, std::min(timestamp_timeout_, max_prediction_horizon_));
    prediction_position_noise_rate_ =
        std::max(0.0, prediction_position_noise_rate_);
    prediction_orientation_noise_rate_ =
        std::max(0.0, prediction_orientation_noise_rate_);
    max_input_position_variance_ =
        std::max(0.01, max_input_position_variance_);
    localization_health_timeout_ =
        std::max(0.1, localization_health_timeout_);
    localization_unhealthy_grace_ = std::max(
        0.0, std::min(localization_health_timeout_,
                      localization_unhealthy_grace_));
    mavros_attitude_timeout_ = std::max(0.1, mavros_attitude_timeout_);
    gravity_alignment_duration_ = std::max(0.5, gravity_alignment_duration_);
    gravity_imu_timeout_ = std::max(0.05, gravity_imu_timeout_);
    gravity_min_samples_ = std::max(10, gravity_min_samples_);
    gravity_max_angular_speed_ =
        std::max(0.01, gravity_max_angular_speed_);
    gravity_max_direction_spread_ = std::max(
        0.1, std::min(20.0, gravity_max_direction_spread_deg_)) *
        kPi / 180.0;
    gravity_max_accel_norm_relative_spread_ = std::max(
        0.01, std::min(0.5, gravity_max_accel_norm_relative_spread_));
    gravity_alignment_max_tilt_ = std::max(
        5.0, std::min(120.0, gravity_alignment_max_tilt_deg_)) *
        kPi / 180.0;
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
        input_topic_, 10, &LioPx4VisionBridge::odometryCallback, this);
    mavros_state_subscriber_ = nh_.subscribe(
        "/mavros/state", 10, &LioPx4VisionBridge::mavrosStateCallback,
        this);
    auto_enable_subscriber_ = nh_.subscribe(
        auto_enable_topic_, 10,
        &LioPx4VisionBridge::autoEnableCallback, this);
    mavros_attitude_subscriber_ = nh_.subscribe(
        mavros_attitude_topic_, 10,
        &LioPx4VisionBridge::mavrosAttitudeCallback, this);
    if (auto_calibrate_mount_from_gravity_) {
      sensor_imu_subscriber_ = nh_.subscribe(
          sensor_imu_topic_, 100,
          &LioPx4VisionBridge::sensorImuCallback, this);
      body_imu_subscriber_ = nh_.subscribe(
          body_imu_topic_, 100,
          &LioPx4VisionBridge::bodyImuCallback, this);
      ROS_INFO("Automatic roll/pitch mount alignment enabled: sensor IMU %s, "
               "body IMU %s, %.1f s stationary window",
               sensor_imu_topic_.c_str(), body_imu_topic_.c_str(),
               gravity_alignment_duration_);
    }
    localization_health_subscriber_ = nh_.subscribe(
        localization_health_topic_, 10,
        &LioPx4VisionBridge::localizationHealthCallback, this);
    pose_publisher_ =
        nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(output_topic_,
                                                                 10);
    alignment_publisher_ = nh_.advertise<geometry_msgs::TransformStamped>(
        "/mine_uav/task1/lio_to_px4_alignment", 1, true);
    sensor_position_publisher_ = nh_.advertise<geometry_msgs::Vector3Stamped>(
        "/mine_uav/task1/lio_sensor_position_in_body", 1, true);
    sensor_rotation_publisher_ = nh_.advertise<geometry_msgs::QuaternionStamped>(
        "/mine_uav/task1/lio_sensor_from_body_rotation", 1, true);
    vision_heading_alignment_publisher_ =
        nh_.advertise<geometry_msgs::QuaternionStamped>(
            vision_heading_alignment_topic_, 1, true);
    healthy_publisher_ = nh_.advertise<std_msgs::Bool>(
        "/mine_uav/task1/vision_healthy", 1, true);
    status_publisher_ = nh_.advertise<std_msgs::String>(
        "/mine_uav/task1/vision_status", 1, true);
    reset_service_ = private_nh_.advertiseService(
        "reset_alignment", &LioPx4VisionBridge::resetAlignment, this);
    publish_timer_ = nh_.createTimer(ros::Duration(1.0 / publish_rate_),
                                     &LioPx4VisionBridge::timerCallback,
                                     this);

    publishHealth(false);
    publishStatus(auto_calibrate_mount_from_gravity_
                      ? "WAIT_GRAVITY_ALIGNMENT"
                      : "WAIT_ODOMETRY");
  }

 private:
  void mavrosStateCallback(const mavros_msgs::State::ConstPtr& state) {
    mavros_connected_ = state->connected;
    mavros_armed_ = state->armed;
    have_mavros_state_ = true;
  }

  void autoEnableCallback(const std_msgs::Bool::ConstPtr& enabled) {
    auto_enabled_ = enabled->data;
    have_auto_enable_ = true;
    if (auto_enabled_ && jump_detected_) {
      resetJumpRecoveryWindow();
    }
  }

  void mavrosAttitudeCallback(const sensor_msgs::Imu::ConstPtr& attitude) {
    if (!finiteQuaternion(attitude->orientation)) {
      return;
    }
    latest_mavros_attitude_ = toMathQuaternion(attitude->orientation);
    last_mavros_attitude_time_ = ros::Time::now();
    have_mavros_attitude_ = true;
  }

  bool mavrosAttitudeReady(const ros::Time& now) const {
    return !align_vision_heading_to_mavros_ ||
        (have_mavros_attitude_ && !last_mavros_attitude_time_.isZero() &&
         (now - last_mavros_attitude_time_).toSec() <=
             mavros_attitude_timeout_);
  }

  void sensorImuCallback(const sensor_msgs::Imu::ConstPtr& imu) {
    recordGravitySample(*imu, &sensor_gravity_samples_);
  }

  void bodyImuCallback(const sensor_msgs::Imu::ConstPtr& imu) {
    recordGravitySample(*imu, &body_gravity_samples_);
  }

  void recordGravitySample(const sensor_msgs::Imu& imu,
                           std::deque<GravitySample>* samples) {
    const auto& a = imu.linear_acceleration;
    const auto& w = imu.angular_velocity;
    if (!finite(a.x) || !finite(a.y) || !finite(a.z) ||
        !finite(w.x) || !finite(w.y) || !finite(w.z)) {
      return;
    }
    const mine_uav_control::Vector3 acceleration(a.x, a.y, a.z);
    const double acceleration_norm =
        mine_uav_control::vectorNorm(acceleration);
    const double angular_speed = std::sqrt(
        w.x * w.x + w.y * w.y + w.z * w.z);
    if (!finite(acceleration_norm) || acceleration_norm < 1e-3 ||
        !finite(angular_speed)) {
      return;
    }
    const ros::Time now = ros::Time::now();
    if (!samples->empty() && now < samples->back().received_at) {
      samples->clear();
    }
    GravitySample sample;
    sample.received_at = now;
    sample.direction = mine_uav_control::normalizedVector(acceleration);
    sample.acceleration_norm = acceleration_norm;
    sample.angular_speed = angular_speed;
    samples->push_back(sample);
    const double retained_duration = gravity_alignment_duration_ +
                                     std::max(0.5, gravity_imu_timeout_);
    while (!samples->empty() &&
           (now - samples->front().received_at).toSec() > retained_duration) {
      samples->pop_front();
    }
  }

  bool stableGravityMean(const std::deque<GravitySample>& samples,
                         const ros::Time& now,
                         const std::string& missing_status,
                         mine_uav_control::Vector3* mean_direction) {
    if (samples.empty() ||
        (now - samples.back().received_at).toSec() > gravity_imu_timeout_) {
      gravity_alignment_status_ = missing_status;
      return false;
    }
    if (static_cast<int>(samples.size()) < gravity_min_samples_ ||
        (samples.back().received_at - samples.front().received_at).toSec() <
            gravity_alignment_duration_) {
      gravity_alignment_status_ = "COLLECTING_GRAVITY_ALIGNMENT";
      return false;
    }
    mine_uav_control::Vector3 sum;
    double acceleration_norm_sum = 0.0;
    for (const auto& sample : samples) {
      sum.x += sample.direction.x;
      sum.y += sample.direction.y;
      sum.z += sample.direction.z;
      acceleration_norm_sum += sample.acceleration_norm;
    }
    const auto mean = mine_uav_control::normalizedVector(sum);
    const double mean_norm = acceleration_norm_sum / samples.size();
    if (mine_uav_control::vectorNorm(mean) < 0.5 || mean_norm < 1e-3) {
      gravity_alignment_status_ = "WAIT_STATIONARY_GRAVITY";
      return false;
    }
    for (const auto& sample : samples) {
      const double dot = std::max(-1.0, std::min(
          1.0, mine_uav_control::vectorDot(sample.direction, mean)));
      const double direction_error = std::acos(dot);
      const double norm_error =
          std::abs(sample.acceleration_norm - mean_norm) / mean_norm;
      if (sample.angular_speed > gravity_max_angular_speed_ ||
          direction_error > gravity_max_direction_spread_ ||
          norm_error > gravity_max_accel_norm_relative_spread_) {
        gravity_alignment_status_ = "WAIT_STATIONARY_GRAVITY";
        return false;
      }
    }
    *mean_direction = mean;
    return true;
  }

  bool ensureGravityMountAlignment(const ros::Time& now) {
    if (!auto_calibrate_mount_from_gravity_ || gravity_mount_ready_) {
      return true;
    }
    if (!have_mavros_state_) {
      gravity_alignment_status_ = "WAIT_MAVROS_STATE_FOR_GRAVITY";
      return false;
    }
    if (mavros_armed_ && !allow_alignment_while_armed_) {
      gravity_alignment_status_ = "WAIT_DISARMED_FOR_GRAVITY";
      return false;
    }
    mine_uav_control::Vector3 sensor_gravity;
    mine_uav_control::Vector3 body_gravity;
    if (!stableGravityMean(sensor_gravity_samples_, now,
                           "WAIT_SENSOR_IMU_GRAVITY", &sensor_gravity) ||
        !stableGravityMean(body_gravity_samples_, now,
                           "WAIT_BODY_IMU_GRAVITY", &body_gravity)) {
      return false;
    }
    mine_uav_control::Quaternion candidate;
    if (!mine_uav_control::gravityAlignmentRotation(
            sensor_gravity, body_gravity, &candidate)) {
      gravity_alignment_status_ = "INVALID_GRAVITY_ALIGNMENT";
      return false;
    }
    const double tilt = mine_uav_control::quaternionAngularDistance(
        mine_uav_control::Quaternion(), candidate);
    if (!finite(tilt) || tilt > gravity_alignment_max_tilt_) {
      gravity_alignment_status_ = "GRAVITY_ALIGNMENT_TILT_REJECTED";
      ROS_ERROR_THROTTLE(
          1.0, "Automatic sensor mount tilt %.2f deg exceeds %.2f deg safety "
               "limit; vision remains disabled",
          tilt * 180.0 / kPi, gravity_alignment_max_tilt_ * 180.0 / kPi);
      return false;
    }
    calibrated_body_from_sensor_ = candidate;
    gravity_mount_ready_ = true;
    gravity_alignment_status_ = "GRAVITY_ALIGNMENT_READY";
    ROS_INFO("Automatic sensor roll/pitch alignment captured: tilt %.2f deg, "
             "sensor gravity [%.4f %.4f %.4f], body gravity "
             "[%.4f %.4f %.4f]",
             tilt * 180.0 / kPi,
             sensor_gravity.x, sensor_gravity.y, sensor_gravity.z,
             body_gravity.x, body_gravity.y, body_gravity.z);
    return true;
  }

  void localizationHealthCallback(const std_msgs::Bool::ConstPtr& health) {
    localization_healthy_ = health->data;
    have_localization_health_ = true;
    last_localization_health_time_ = ros::Time::now();
    if (localization_healthy_) {
      last_localization_healthy_time_ = last_localization_health_time_;
    }
  }

  bool localizationHealthReady(const ros::Time& now) const {
    if (!require_localization_health_) {
      return true;
    }
    if (!have_localization_health_ || last_localization_health_time_.isZero()) {
      return false;
    }
    const double message_age =
        (now - last_localization_health_time_).toSec();
    if (message_age < 0.0 || message_age > localization_health_timeout_) {
      return false;
    }
    if (localization_healthy_) {
      return true;
    }
    if (last_localization_healthy_time_.isZero()) {
      return false;
    }
    const double unhealthy_age =
        (now - last_localization_healthy_time_).toSec();
    return unhealthy_age >= 0.0 &&
           unhealthy_age <= localization_unhealthy_grace_;
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
        !finiteQuaternion(odometry.pose.pose.orientation) ||
        odometry.header.stamp.isZero()) {
      return false;
    }
    const double source_age =
        (ros::Time::now() - odometry.header.stamp).toSec();
    if (!finite(source_age) || source_age < -0.1 ||
        source_age > timestamp_timeout_) {
      return false;
    }
    return input_frame_.empty() || odometry.header.frame_id == input_frame_;
  }

  void captureAlignment(const nav_msgs::Odometry& odometry) {
    origin_ = odometry.pose.pose.position;
    const auto mount_alignment = auto_calibrate_mount_from_gravity_
        ? mine_uav_control::makeMountAlignmentFromBodyFromSensor(
              toMathQuaternion(odometry.pose.pose.orientation),
              calibrated_body_from_sensor_, zero_initial_roll_pitch_,
              zero_initial_yaw_)
        : mine_uav_control::makeMountAlignment(
              toMathQuaternion(odometry.pose.pose.orientation),
              temporary_sensor_roll_in_body_, temporary_sensor_pitch_in_body_,
              temporary_sensor_yaw_in_body_, zero_initial_roll_pitch_,
              zero_initial_yaw_);
    map_to_output_ = mount_alignment.map_to_output;
    sensor_from_body_ = mount_alignment.sensor_from_body;
    const auto initial_body_in_map = mine_uav_control::multiply(
        toMathQuaternion(odometry.pose.pose.orientation), sensor_from_body_);
    const auto initial_body_in_planning =
        mine_uav_control::bodyOrientationInPlanningFrame(
            map_to_output_, toMathQuaternion(odometry.pose.pose.orientation),
            sensor_from_body_);
    planning_to_px4_heading_ = align_vision_heading_to_mavros_
        ? mine_uav_control::planningToReferenceHeading(
              latest_mavros_attitude_, initial_body_in_planning)
        : mine_uav_control::Quaternion();
    initial_sensor_offset_in_map_ = mine_uav_control::rotateVector(
        initial_body_in_map, sensor_position_in_body_);
    alignment_ready_ = true;
    jump_detected_ = false;
    high_angular_motion_ = false;
    resetJumpRecoveryWindow();
    input_rate_fault_count_ = 0;
    last_input_rate_fault_.clear();
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
    geometry_msgs::QuaternionStamped heading_alignment;
    heading_alignment.header = transform.header;
    heading_alignment.header.frame_id = output_frame_;
    heading_alignment.quaternion = toRosQuaternion(planning_to_px4_heading_);
    vision_heading_alignment_publisher_.publish(heading_alignment);
    publishStatus("ALIGNMENT_READY");
    ROS_INFO("Captured task-one 3-D alignment at [%.3f %.3f %.3f], "
             "map-to-planning yaw %.2f deg, mount source %s, "
             "body-to-sensor lever "
             "[%.3f %.3f %.3f] m",
             origin_.x, origin_.y, origin_.z,
             mine_uav_control::yawFromQuaternion(map_to_output_) *
                 180.0 / kPi,
             auto_calibrate_mount_from_gravity_ ? "startup gravity" :
                                                  "static parameters",
             sensor_position_in_body_.x, sensor_position_in_body_.y,
             sensor_position_in_body_.z);
    ROS_INFO("External-vision heading aligned to PX4 by %.2f deg",
             mine_uav_control::yawFromQuaternion(planning_to_px4_heading_) *
                 180.0 / kPi);
  }

  void resetJumpRecoveryWindow() {
    jump_recovery_stable_count_ = 0;
    have_jump_recovery_sample_ = false;
  }

  void latchInputJump(const std::string& status,
                      const std::string& message) {
    jump_detected_ = true;
    high_angular_motion_ = false;
    input_orientation_delta_ = mine_uav_control::Quaternion();
    input_orientation_delta_dt_ = 0.0;
    resetJumpRecoveryWindow();
    publishHealth(false);
    publishStatus(status);
    ROS_ERROR("%s", message.c_str());
  }

  bool recoverLatchedJumpFromStableManualData(
      const nav_msgs::Odometry& odometry) {
    if (!have_auto_enable_ || auto_enabled_) {
      resetJumpRecoveryWindow();
      publishStatus(have_auto_enable_
                        ? "JUMP_LATCHED_WAIT_AUTO_DISABLED"
                        : "JUMP_LATCHED_WAIT_AUTO_STATE");
      return false;
    }
    const ros::Time now = ros::Time::now();
    if (!localizationHealthReady(now) || !inputCovarianceHealthy(odometry)) {
      resetJumpRecoveryWindow();
      publishStatus("JUMP_LATCHED_WAIT_STABLE_LIO");
      return false;
    }

    bool stable = false;
    if (have_jump_recovery_sample_) {
      const double dt =
          (odometry.header.stamp - jump_recovery_stamp_).toSec();
      const double distance = std::sqrt(squaredDistance(
          jump_recovery_position_, odometry.pose.pose.position));
      const auto delta = mine_uav_control::multiply(
          toMathQuaternion(odometry.pose.pose.orientation),
          mine_uav_control::conjugate(jump_recovery_orientation_));
      const double angle = mine_uav_control::quaternionAngularDistance(
          mine_uav_control::Quaternion(), delta);
      stable = finite(dt) && dt > 0.0 && dt <= timestamp_timeout_ &&
               finite(distance) && distance <= max_input_jump_ &&
               distance / dt <= max_input_speed_ && finite(angle) &&
               angle <= max_input_orientation_jump_ &&
               angle / dt <= max_input_angular_speed_;
    }

    jump_recovery_position_ = odometry.pose.pose.position;
    jump_recovery_orientation_ =
        toMathQuaternion(odometry.pose.pose.orientation);
    jump_recovery_stamp_ = odometry.header.stamp;
    have_jump_recovery_sample_ = true;
    jump_recovery_stable_count_ = stable
        ? jump_recovery_stable_count_ + 1 : 0;
    if (jump_recovery_stable_count_ < jump_recovery_stable_samples_) {
      publishStatus("JUMP_LATCHED_COLLECTING_STABLE_MANUAL_DATA");
      return false;
    }

    // Preserve the original map/PX4 alignment. In particular, an armed
    // aircraft may resume a stable existing alignment after the pilot has
    // removed automatic authority, but it must never capture a new origin.
    jump_detected_ = false;
    high_angular_motion_ = false;
    input_rate_fault_count_ = 0;
    last_input_rate_fault_.clear();
    input_velocity_ = geometry_msgs::Point();
    input_orientation_delta_ = mine_uav_control::Quaternion();
    input_orientation_delta_dt_ = 0.0;
    previous_input_position_ = odometry.pose.pose.position;
    previous_input_orientation_ =
        toMathQuaternion(odometry.pose.pose.orientation);
    previous_input_stamp_ = odometry.header.stamp;
    have_previous_input_ = true;
    latest_odometry_ = odometry;
    last_input_time_ = now;
    have_odometry_ = true;
    resetJumpRecoveryWindow();
    publishStatus("RECOVERED_STABLE_MANUAL_DATA");
    ROS_WARN("External vision recovered from a latched input jump after %d "
             "stable manual-control samples; the existing alignment was "
             "preserved%s",
             jump_recovery_stable_samples_,
             mavros_armed_ ? " while PX4 remained armed" : "");
    return true;
  }

  void odometryCallback(const nav_msgs::Odometry::ConstPtr& odometry) {
    if (!validOdometry(*odometry)) {
      publishStatus("INVALID_ODOMETRY");
      ROS_WARN_THROTTLE(1.0,
                        "Rejecting invalid LIO odometry or frame mismatch");
      return;
    }
    if (odometry_outage_seen_ && auto_realign_after_odometry_timeout_) {
      if (!allow_alignment_while_armed_ &&
          (!have_mavros_state_ || mavros_armed_)) {
        publishHealth(false);
        publishStatus(have_mavros_state_
                          ? "WAIT_DISARMED_AFTER_ODOMETRY_RESTART"
                          : "WAIT_MAVROS_STATE_AFTER_ODOMETRY_RESTART");
        return;
      }
      odometry_outage_seen_ = false;
      alignment_ready_ = false;
      have_previous_input_ = false;
      jump_detected_ = false;
      high_angular_motion_ = false;
      resetJumpRecoveryWindow();
      ROS_WARN("LIO odometry recovered after an outage; capturing a fresh "
               "alignment while PX4 is disarmed");
    }
    if (!alignment_ready_) {
      if (!allow_alignment_while_armed_ &&
          (!have_mavros_state_ || mavros_armed_)) {
        publishStatus(have_mavros_state_ ? "WAIT_DISARMED" :
                                           "WAIT_MAVROS_STATE");
        ROS_ERROR_THROTTLE(1.0,
            "Will not capture a new vision alignment until PX4 state is "
            "known and disarmed");
        return;
      }
      const ros::Time now = ros::Time::now();
      if (!ensureGravityMountAlignment(now)) {
        publishHealth(false);
        publishStatus(gravity_alignment_status_);
        return;
      }
      if (!mavrosAttitudeReady(now)) {
        publishHealth(false);
        publishStatus("WAIT_PX4_ATTITUDE");
        return;
      }
      if (!localizationHealthReady(now)) {
        publishStatus("WAIT_LIO_HEALTH");
        return;
      }
      if (!inputCovarianceHealthy(*odometry)) {
        publishStatus("WAIT_LIO_COVARIANCE");
        return;
      }
      captureAlignment(*odometry);
    }
    if (jump_detected_) {
      recoverLatchedJumpFromStableManualData(*odometry);
      return;
    }
    const double displacement_squared = have_previous_input_ ?
        squaredDistance(previous_input_position_,
                        odometry->pose.pose.position) : 0.0;
    const double input_dt = have_previous_input_
        ? (odometry->header.stamp - previous_input_stamp_).toSec()
        : 0.0;
    if (have_previous_input_ && (!finite(input_dt) || input_dt <= 0.0)) {
      latchInputJump("INPUT_TIME_NONMONOTONIC",
                     "LIO odometry time did not advance; vision output "
                     "latched off");
      return;
    }
    if (have_previous_input_ &&
        displacement_squared > max_input_jump_ * max_input_jump_) {
      latchInputJump("INPUT_JUMP",
                     "LIO position jumped more than " +
                         std::to_string(max_input_jump_) +
                         " m; vision output latched off");
      return;
    }
    if (have_previous_input_) {
      const double measured_speed =
          std::sqrt(displacement_squared) / input_dt;
      if (!finite(measured_speed) || measured_speed > max_input_speed_) {
        if (recordInputRateFault("INPUT_SPEED")) {
          latchInputJump(
              "INPUT_SPEED",
              "LIO implied speed " + std::to_string(measured_speed) +
                  " m/s exceeds " + std::to_string(max_input_speed_) +
                  " m/s for consecutive samples; vision output latched off");
        } else {
          publishStatus("INPUT_SPEED_TRANSIENT");
          ROS_WARN("Discarding isolated LIO speed outlier %.3f m/s "
                   "(limit %.3f m/s, confirmation %d/%d)",
                   measured_speed, max_input_speed_, input_rate_fault_count_,
                   input_rate_fault_confirm_samples_);
        }
        return;
      }

      const auto orientation_delta = mine_uav_control::multiply(
          toMathQuaternion(odometry->pose.pose.orientation),
          mine_uav_control::conjugate(previous_input_orientation_));
      const double measured_angular_speed =
          mine_uav_control::quaternionAngularDistance(
              mine_uav_control::Quaternion(), orientation_delta) / input_dt;
      const double orientation_angle =
          mine_uav_control::quaternionAngularDistance(
              mine_uav_control::Quaternion(), orientation_delta);
      if (!finite(orientation_angle) || !finite(measured_angular_speed)) {
        latchInputJump("INPUT_ORIENTATION_INVALID",
                       "LIO orientation delta is not finite; vision output "
                       "latched off");
        return;
      }
      if (orientation_angle > max_input_orientation_jump_) {
        latchInputJump(
            "INPUT_ORIENTATION_JUMP",
            "LIO single-frame orientation increment " +
                std::to_string(orientation_angle) + " rad exceeds " +
                std::to_string(max_input_orientation_jump_) +
                " rad; vision output latched off");
        return;
      }
      if (measured_angular_speed > max_physical_angular_speed_) {
        if (recordInputRateFault("INPUT_ANGULAR_SPEED_IMPOSSIBLE")) {
          latchInputJump(
              "INPUT_ANGULAR_SPEED_IMPOSSIBLE",
              "LIO implied angular speed " +
                  std::to_string(measured_angular_speed) +
                  " rad/s exceeds physical limit " +
                  std::to_string(max_physical_angular_speed_) +
                  " rad/s for consecutive samples; vision output latched off");
        } else {
          publishStatus("INPUT_ANGULAR_SPEED_IMPOSSIBLE_TRANSIENT");
          ROS_WARN("Discarding isolated physically impossible LIO angular "
                   "rate %.3f rad/s (limit %.3f rad/s, confirmation %d/%d)",
                   measured_angular_speed, max_physical_angular_speed_,
                   input_rate_fault_count_,
                   input_rate_fault_confirm_samples_);
        }
        return;
      }
      high_angular_motion_ =
          measured_angular_speed > max_input_angular_speed_;
      if (high_angular_motion_) {
        ROS_WARN_THROTTLE(
            1.0,
            "LIO reports continuous high angular motion %.3f rad/s; "
            "forward attitude prediction is paused and covariance inflated",
            measured_angular_speed);
      }
    }
    input_rate_fault_count_ = 0;
    last_input_rate_fault_.clear();
    input_velocity_ = geometry_msgs::Point();
    input_orientation_delta_ = mine_uav_control::Quaternion();
    input_orientation_delta_dt_ = 0.0;
    if (have_previous_input_ && !jump_detected_) {
      if (input_dt >= 0.03 && input_dt <= 0.3) {
        input_velocity_.x = (odometry->pose.pose.position.x -
                             previous_input_position_.x) / input_dt;
        input_velocity_.y = (odometry->pose.pose.position.y -
                             previous_input_position_.y) / input_dt;
        input_velocity_.z = (odometry->pose.pose.position.z -
                             previous_input_position_.z) / input_dt;
        if (!high_angular_motion_) {
          input_orientation_delta_ = mine_uav_control::multiply(
              toMathQuaternion(odometry->pose.pose.orientation),
              mine_uav_control::conjugate(previous_input_orientation_));
          input_orientation_delta_dt_ = input_dt;
        }
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
    const auto planning_position = mine_uav_control::rotateVector(
        map_to_output_, mine_uav_control::Vector3(dx, dy, dz));
    const auto rotated_position = mine_uav_control::rotateVector(
        planning_to_px4_heading_, planning_position);
    output.pose.pose.position.x = rotated_position.x;
    output.pose.pose.position.y = rotated_position.y;
    output.pose.pose.position.z = rotated_position.z;
    // The estimator reports the sensor pose. Apply world-axis alignment on the
    // left and the fixed body-to-sensor rotation on the right so MAVROS gets
    // aircraft attitude as well as correctly rotated position.
    const auto body_in_planning = mine_uav_control::multiply(
        mine_uav_control::multiply(map_to_output_,
                                   predicted_sensor_orientation),
        sensor_from_body_);
    output.pose.pose.orientation = toRosQuaternion(mine_uav_control::multiply(
        planning_to_px4_heading_, body_in_planning));

    const double position_stddev = std::hypot(
        position_stddev_, prediction_position_noise_rate_ * sample_age);
    double orientation_stddev = std::hypot(
        orientation_stddev_, prediction_orientation_noise_rate_ * sample_age);
    if (high_angular_motion_) {
      orientation_stddev = std::max(
          orientation_stddev, high_angular_rate_orientation_stddev_);
    }
    const double position_variance = position_stddev * position_stddev;
    const double orientation_variance =
        orientation_stddev * orientation_stddev;
    output.pose.covariance.fill(0.0);
    const auto map_to_vision = mine_uav_control::multiply(
        planning_to_px4_heading_, map_to_output_);
    const auto rx = mine_uav_control::rotateVector(
        map_to_vision, mine_uav_control::Vector3(1.0, 0.0, 0.0));
    const auto ry = mine_uav_control::rotateVector(
        map_to_vision, mine_uav_control::Vector3(0.0, 1.0, 0.0));
    const auto rz = mine_uav_control::rotateVector(
        map_to_vision, mine_uav_control::Vector3(0.0, 0.0, 1.0));
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

  bool recordInputRateFault(const std::string& reason) {
    if (reason == last_input_rate_fault_) {
      ++input_rate_fault_count_;
    } else {
      last_input_rate_fault_ = reason;
      input_rate_fault_count_ = 1;
    }
    return input_rate_fault_count_ >= input_rate_fault_confirm_samples_;
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
    const bool localization_ok = localizationHealthReady(now);
    const bool covariance_ok = have_odometry_ &&
                               inputCovarianceHealthy(latest_odometry_);
    const bool mavros_ok = !require_mavros_connection_ ||
                           (have_mavros_state_ && mavros_connected_);
    const bool healthy = alignment_ready_ && data_fresh &&
                         timestamp_healthy && localization_ok && covariance_ok &&
                         mavros_ok && !jump_detected_;
    publishHealth(healthy);
    if (!healthy) {
      if (jump_detected_) {
        publishStatus(!have_auto_enable_
                          ? "JUMP_LATCHED_WAIT_AUTO_STATE"
                          : (auto_enabled_
                                 ? "JUMP_LATCHED_WAIT_AUTO_DISABLED"
                                 : "JUMP_LATCHED_COLLECTING_STABLE_MANUAL_DATA"));
      } else if (!alignment_ready_ && !ensureGravityMountAlignment(now)) {
        publishStatus(gravity_alignment_status_);
      } else if (!alignment_ready_ && !mavrosAttitudeReady(now)) {
        publishStatus("WAIT_PX4_ATTITUDE");
      } else if (!have_odometry_) {
        publishStatus("WAIT_ODOMETRY");
      } else if (!data_fresh) {
        if (auto_realign_after_odometry_timeout_) {
          odometry_outage_seen_ = true;
          alignment_ready_ = false;
          have_previous_input_ = false;
          jump_detected_ = false;
          high_angular_motion_ = false;
          resetJumpRecoveryWindow();
        }
        publishStatus("ODOMETRY_TIMEOUT");
      } else if (!timestamp_healthy) {
        publishStatus("ODOMETRY_TIMESTAMP_STALE");
      } else if (!localization_ok) {
        const bool health_fresh = have_localization_health_ &&
            !last_localization_health_time_.isZero() &&
            (now - last_localization_health_time_).toSec() <=
                localization_health_timeout_;
        publishStatus(health_fresh ? "LIO_DEGENERATE" :
                                     "LIO_HEALTH_TIMEOUT");
      } else if (!covariance_ok) {
        publishStatus("LIO_COVARIANCE_REJECTED");
      } else if (!alignment_ready_) {
        publishStatus("WAIT_ODOMETRY");
      } else if (!mavros_ok) {
        publishStatus("MAVROS_DISCONNECTED");
      }
      return;
    }
    // Predict the 10 Hz LIO position and attitude to this 30 Hz output
    // tick, bound that prediction, and inflate covariance by its horizon
    // before restamping for PX4's fusion horizon.
    pose_publisher_.publish(makeVisionPose(now));
    publishStatus(high_angular_motion_
                      ? "STREAMING_HIGH_ANGULAR_RATE"
                      : "STREAMING");
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
    high_angular_motion_ = false;
    resetJumpRecoveryWindow();
    input_rate_fault_count_ = 0;
    last_input_rate_fault_.clear();
    if (auto_calibrate_mount_from_gravity_) {
      gravity_mount_ready_ = false;
      calibrated_body_from_sensor_ = mine_uav_control::Quaternion();
      sensor_gravity_samples_.clear();
      body_gravity_samples_.clear();
      gravity_alignment_status_ = "WAIT_GRAVITY_ALIGNMENT";
    }
    publishHealth(false);
    publishStatus(auto_calibrate_mount_from_gravity_
                      ? gravity_alignment_status_
                      : "WAIT_ODOMETRY");
    response.success = true;
    response.message = auto_calibrate_mount_from_gravity_
        ? "alignment cleared; waiting for a fresh stationary gravity window"
        : "alignment cleared; next valid odometry will set origin";
    return true;
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber odometry_subscriber_;
  ros::Subscriber mavros_state_subscriber_;
  ros::Subscriber auto_enable_subscriber_;
  ros::Subscriber mavros_attitude_subscriber_;
  ros::Subscriber sensor_imu_subscriber_;
  ros::Subscriber body_imu_subscriber_;
  ros::Subscriber localization_health_subscriber_;
  ros::Publisher pose_publisher_;
  ros::Publisher alignment_publisher_;
  ros::Publisher sensor_position_publisher_;
  ros::Publisher sensor_rotation_publisher_;
  ros::Publisher vision_heading_alignment_publisher_;
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
  std::string localization_health_topic_;
  std::string mavros_attitude_topic_;
  std::string sensor_imu_topic_;
  std::string body_imu_topic_;
  std::string vision_heading_alignment_topic_;
  std::string auto_enable_topic_;
  std::string gravity_alignment_status_{"WAIT_GRAVITY_ALIGNMENT"};
  std::string last_status_;
  double publish_rate_{30.0};
  double data_timeout_{0.3};
  double timestamp_timeout_{0.6};
  double max_input_jump_{2.0};
  double max_input_speed_{2.0};
  double max_input_angular_speed_{2.0};
  double max_physical_angular_speed_{4.0};
  double max_input_orientation_jump_{0.75};
  double high_angular_rate_orientation_stddev_{0.35};
  int input_rate_fault_confirm_samples_{2};
  int jump_recovery_stable_samples_{5};
  double max_prediction_horizon_{0.35};
  double prediction_position_noise_rate_{0.15};
  double prediction_orientation_noise_rate_{0.20};
  double max_input_position_variance_{4.0};
  double localization_health_timeout_{0.5};
  double mavros_attitude_timeout_{0.5};
  double gravity_alignment_duration_{2.0};
  double gravity_imu_timeout_{0.2};
  int gravity_min_samples_{40};
  double gravity_max_angular_speed_{0.15};
  double gravity_max_direction_spread_deg_{5.0};
  double gravity_max_direction_spread_{5.0 * kPi / 180.0};
  double gravity_max_accel_norm_relative_spread_{0.15};
  double gravity_alignment_max_tilt_deg_{60.0};
  double gravity_alignment_max_tilt_{60.0 * kPi / 180.0};
  double position_stddev_{0.05};
  double orientation_stddev_{0.10};
  double localization_unhealthy_grace_{0.30};
  mine_uav_control::Quaternion map_to_output_;
  mine_uav_control::Quaternion sensor_from_body_;
  mine_uav_control::Quaternion planning_to_px4_heading_;
  mine_uav_control::Quaternion latest_mavros_attitude_;
  mine_uav_control::Quaternion calibrated_body_from_sensor_;
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
  bool require_localization_health_{true};
  bool allow_alignment_while_armed_{false};
  bool auto_realign_after_odometry_timeout_{true};
  bool align_vision_heading_to_mavros_{false};
  bool auto_calibrate_mount_from_gravity_{false};
  bool gravity_mount_ready_{false};
  bool have_odometry_{false};
  bool have_previous_input_{false};
  bool alignment_ready_{false};
  bool jump_detected_{false};
  bool high_angular_motion_{false};
  bool odometry_outage_seen_{false};
  bool have_mavros_state_{false};
  bool have_mavros_attitude_{false};
  bool have_localization_health_{false};
  bool localization_healthy_{false};
  bool mavros_connected_{false};
  bool mavros_armed_{false};
  bool have_auto_enable_{false};
  bool auto_enabled_{false};
  bool health_published_{false};
  ros::Time previous_input_stamp_;
  ros::Time last_localization_health_time_;
  ros::Time last_localization_healthy_time_;
  ros::Time last_mavros_attitude_time_;
  std::deque<GravitySample> sensor_gravity_samples_;
  std::deque<GravitySample> body_gravity_samples_;
  geometry_msgs::Point input_velocity_;
  bool last_health_{false};
  int input_rate_fault_count_{0};
  std::string last_input_rate_fault_;
  geometry_msgs::Point jump_recovery_position_;
  mine_uav_control::Quaternion jump_recovery_orientation_;
  ros::Time jump_recovery_stamp_;
  int jump_recovery_stable_count_{0};
  bool have_jump_recovery_sample_{false};
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "lio_px4_vision_bridge");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  LioPx4VisionBridge bridge(nh, private_nh);
  ros::spin();
  return 0;
}
