#include "mine_uav_control/lio_mount_alignment.hpp"
#include "mine_uav_control/motion_continuity.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <string>

#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

namespace {

using mine_uav_control::Quaternion;
using mine_uav_control::Vector3;

bool finite(double value) { return std::isfinite(value); }

bool finiteQuaternion(const geometry_msgs::Quaternion& q) {
  return finite(q.x) && finite(q.y) && finite(q.z) && finite(q.w) &&
         q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w > 1e-8;
}

Quaternion toMathQuaternion(const geometry_msgs::Quaternion& q) {
  return mine_uav_control::normalized(Quaternion(q.x, q.y, q.z, q.w));
}

geometry_msgs::Quaternion toRosQuaternion(const Quaternion& input) {
  const Quaternion q = mine_uav_control::normalized(input);
  geometry_msgs::Quaternion output;
  output.x = q.x;
  output.y = q.y;
  output.z = q.z;
  output.w = q.w;
  return output;
}

double vectorNorm(double x, double y, double z) {
  return std::sqrt(x * x + y * y + z * z);
}

std::array<std::array<double, 3>, 3> rotationMatrix(
    const Quaternion& rotation) {
  const Vector3 x = mine_uav_control::rotateVector(
      rotation, Vector3(1.0, 0.0, 0.0));
  const Vector3 y = mine_uav_control::rotateVector(
      rotation, Vector3(0.0, 1.0, 0.0));
  const Vector3 z = mine_uav_control::rotateVector(
      rotation, Vector3(0.0, 0.0, 1.0));
  return {{{{x.x, y.x, z.x}},
           {{x.y, y.y, z.y}},
           {{x.z, y.z, z.z}}}};
}

void rotatePoseCovariance(
    const Quaternion& rotation,
    const boost::array<double, 36>& input,
    boost::array<double, 36>* output) {
  const auto r = rotationMatrix(rotation);
  double transform[6][6] = {};
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      transform[row][column] = r[row][column];
      transform[row + 3][column + 3] = r[row][column];
    }
  }
  for (int row = 0; row < 6; ++row) {
    for (int column = 0; column < 6; ++column) {
      double value = 0.0;
      for (int left = 0; left < 6; ++left) {
        for (int right = 0; right < 6; ++right) {
          value += transform[row][left] * input[left * 6 + right] *
                   transform[column][right];
        }
      }
      (*output)[row * 6 + column] = value;
    }
  }
}

bool hasFloat32Field(const sensor_msgs::PointCloud2& cloud,
                     const std::string& name) {
  for (const auto& field : cloud.fields) {
    if (field.name == name) {
      return field.datatype == sensor_msgs::PointField::FLOAT32 &&
             field.count == 1;
    }
  }
  return false;
}

class Task1PerceptionGate {
 public:
  Task1PerceptionGate(const ros::NodeHandle& nh,
                      const ros::NodeHandle& private_nh)
      : nh_(nh), private_nh_(private_nh) {
    private_nh_.param("input_cloud_topic", input_cloud_topic_,
                      std::string("/omnilivo/cloud_registered"));
    private_nh_.param("input_odom_topic", input_odom_topic_,
                      std::string("/omnilivo/odometry"));
    private_nh_.param("auxiliary_input_odom_topic",
                      auxiliary_input_odom_topic_, std::string());
    private_nh_.param("output_cloud_topic", output_cloud_topic_,
                      std::string("/mine_uav/task1/cloud_registered"));
    private_nh_.param("output_odom_topic", output_odom_topic_,
                      std::string("/mine_uav/task1/odometry"));
    private_nh_.param("auxiliary_output_odom_topic",
                      auxiliary_output_odom_topic_, std::string());
    private_nh_.param("source_alignment_topic", source_alignment_topic_,
                      std::string(
                          "/mine_uav/task1/lio_to_px4_alignment"));
    private_nh_.param("sensor_rotation_topic", sensor_rotation_topic_,
                      std::string(
                          "/mine_uav/task1/lio_sensor_from_body_rotation"));
    private_nh_.param(
        "vision_heading_alignment_topic", vision_heading_alignment_topic_,
        std::string("/mine_uav/task1/vision_heading_from_planning"));
    private_nh_.param("output_alignment_topic", output_alignment_topic_,
                      std::string(
                          "/mine_uav/task1/planning_to_px4_alignment"));
    private_nh_.param("mission_enable_topic", mission_enable_topic_,
                      std::string("/mine_uav/mission/goaf_enable"));
    private_nh_.param("additional_cloud_enable_topic",
                      additional_cloud_enable_topic_, std::string());
    private_nh_.param("second_additional_cloud_enable_topic",
                      second_additional_cloud_enable_topic_, std::string());
    private_nh_.param("vision_health_topic", vision_health_topic_,
                      std::string("/mine_uav/task1/vision_healthy"));
    private_nh_.param("expected_frame", expected_frame_,
                      std::string("camera_init"));
    private_nh_.param("output_frame", output_frame_,
                      std::string("task1_planning"));
    private_nh_.param("output_child_frame", output_child_frame_,
                      std::string("base_link"));
    private_nh_.param("require_vision_healthy", require_vision_healthy_, true);
    private_nh_.param("require_vision_heading_alignment",
                      require_vision_heading_alignment_, false);
    private_nh_.param("max_alignment_translation_change",
                      max_alignment_translation_change_, 0.05);
    private_nh_.param("max_alignment_rotation_change",
                      max_alignment_rotation_change_, 0.05);
    max_alignment_translation_change_ =
        std::max(0.001, max_alignment_translation_change_);
    max_alignment_rotation_change_ =
        std::max(0.001, max_alignment_rotation_change_);

    cloud_publisher_ =
        nh_.advertise<sensor_msgs::PointCloud2>(output_cloud_topic_, 2);
    odom_publisher_ = nh_.advertise<nav_msgs::Odometry>(output_odom_topic_, 20);
    if (auxiliary_input_odom_topic_.empty() !=
        auxiliary_output_odom_topic_.empty()) {
      throw std::invalid_argument(
          "auxiliary odometry input and output topics must be configured together");
    }
    if (!auxiliary_output_odom_topic_.empty()) {
      auxiliary_odom_publisher_ = nh_.advertise<nav_msgs::Odometry>(
          auxiliary_output_odom_topic_, 50);
    }
    planning_alignment_publisher_ =
        nh_.advertise<geometry_msgs::TransformStamped>(
            output_alignment_topic_, 1, true);
    status_publisher_ = nh_.advertise<std_msgs::String>(
        "/mine_uav/task1/perception_gate_status", 1, true);
    mission_subscriber_ = nh_.subscribe(
        mission_enable_topic_, 2, &Task1PerceptionGate::missionCallback, this);
    if (!additional_cloud_enable_topic_.empty()) {
      additional_mission_subscriber_ = nh_.subscribe(
          additional_cloud_enable_topic_, 2,
          &Task1PerceptionGate::additionalMissionCallback, this);
    }
    if (!second_additional_cloud_enable_topic_.empty()) {
      second_additional_mission_subscriber_ = nh_.subscribe(
          second_additional_cloud_enable_topic_, 2,
          &Task1PerceptionGate::secondAdditionalMissionCallback, this);
    }
    vision_subscriber_ = nh_.subscribe(
        vision_health_topic_, 2, &Task1PerceptionGate::visionCallback, this);
    alignment_subscriber_ = nh_.subscribe(
        source_alignment_topic_, 1,
        &Task1PerceptionGate::alignmentCallback, this);
    sensor_rotation_subscriber_ = nh_.subscribe(
        sensor_rotation_topic_, 1,
        &Task1PerceptionGate::sensorRotationCallback, this);
    vision_heading_alignment_subscriber_ = nh_.subscribe(
        vision_heading_alignment_topic_, 1,
        &Task1PerceptionGate::visionHeadingAlignmentCallback, this);
    cloud_subscriber_ = nh_.subscribe(
        input_cloud_topic_, 2, &Task1PerceptionGate::cloudCallback, this);
    odom_subscriber_ = nh_.subscribe(
        input_odom_topic_, 20, &Task1PerceptionGate::odomCallback, this);
    if (!auxiliary_input_odom_topic_.empty()) {
      auxiliary_odom_subscriber_ = nh_.subscribe(
          auxiliary_input_odom_topic_, 100,
          &Task1PerceptionGate::auxiliaryOdomCallback, this);
    }

    publishStatus("CLOSED_WAIT_TASK1");
    ROS_INFO("Task-one perception gate levels point clouds and odometry before "
             "they reach the planner; raw localization remains available to PX4");
  }

 private:
  bool open() const {
    return (mission_enabled_ || additional_mission_enabled_ ||
            second_additional_mission_enabled_) &&
           transformReady();
  }

  bool transformReady() const {
    return !alignment_fault_ && alignment_ready_ && sensor_rotation_ready_ &&
           (!require_vision_heading_alignment_ ||
            vision_heading_alignment_ready_) &&
           (!require_vision_healthy_ || vision_healthy_);
  }

  void missionCallback(const std_msgs::Bool::ConstPtr& message) {
    const bool was_open = open();
    mission_enabled_ = message->data;
    if (!mission_enabled_ && !additional_mission_enabled_ &&
        !second_additional_mission_enabled_) {
      alignment_fault_ = false;
    }
    updateStatus(was_open);
  }

  void additionalMissionCallback(const std_msgs::Bool::ConstPtr& message) {
    const bool was_open = open();
    additional_mission_enabled_ = message->data;
    if (!mission_enabled_ && !additional_mission_enabled_ &&
        !second_additional_mission_enabled_) {
      alignment_fault_ = false;
    }
    updateStatus(was_open);
  }

  void secondAdditionalMissionCallback(
      const std_msgs::Bool::ConstPtr& message) {
    const bool was_open = open();
    second_additional_mission_enabled_ = message->data;
    if (!mission_enabled_ && !additional_mission_enabled_ &&
        !second_additional_mission_enabled_) {
      alignment_fault_ = false;
    }
    updateStatus(was_open);
  }

  void visionCallback(const std_msgs::Bool::ConstPtr& message) {
    const bool was_open = open();
    vision_healthy_ = message->data;
    updateStatus(was_open);
  }

  void alignmentCallback(
      const geometry_msgs::TransformStamped::ConstPtr& message) {
    const auto& t = message->transform.translation;
    const auto& q = message->transform.rotation;
    if (!finite(t.x) || !finite(t.y) || !finite(t.z) ||
        !finiteQuaternion(q) ||
        (!expected_frame_.empty() &&
         message->child_frame_id != expected_frame_)) {
      alignment_ready_ = false;
      publishStatus("INVALID_SOURCE_ALIGNMENT");
      ROS_ERROR_THROTTLE(1.0, "Task-one perception gate rejected invalid "
                              "LIO alignment");
      return;
    }

    const bool was_open = open();
    const Quaternion next_rotation = toMathQuaternion(q);
    const bool changed = alignment_ready_ &&
        (vectorNorm(t.x - output_translation_.x,
                    t.y - output_translation_.y,
                    t.z - output_translation_.z) >
             max_alignment_translation_change_ ||
         mine_uav_control::quaternionAngularDistance(
             next_rotation, map_to_planning_) >
             max_alignment_rotation_change_);
    if (was_open && changed) {
      alignment_fault_ = true;
      ROS_ERROR("Task-one source alignment changed while the planner gate was "
                "open; planner input stopped");
    }
    map_to_planning_ = next_rotation;
    output_translation_ = Vector3(t.x, t.y, t.z);
    alignment_parent_frame_ = message->header.frame_id;
    alignment_ready_ = true;
    publishPlanningAlignment(message->header.stamp);
    updateStatus(was_open);
  }

  void sensorRotationCallback(
      const geometry_msgs::QuaternionStamped::ConstPtr& message) {
    if (!finiteQuaternion(message->quaternion)) {
      sensor_rotation_ready_ = false;
      publishStatus("INVALID_SENSOR_ROTATION");
      return;
    }
    const bool was_open = open();
    const Quaternion next = toMathQuaternion(message->quaternion);
    if (was_open && sensor_rotation_ready_ &&
        mine_uav_control::quaternionAngularDistance(
            next, sensor_from_body_) > max_alignment_rotation_change_) {
      alignment_fault_ = true;
      ROS_ERROR("Task-one sensor mounting rotation changed while the planner "
                "gate was open; planner input stopped");
    }
    sensor_from_body_ = next;
    sensor_rotation_ready_ = true;
    publishPlanningAlignment(message->header.stamp);
    updateStatus(was_open);
  }

  void visionHeadingAlignmentCallback(
      const geometry_msgs::QuaternionStamped::ConstPtr& message) {
    if (!finiteQuaternion(message->quaternion)) {
      vision_heading_alignment_ready_ = false;
      publishStatus("INVALID_VISION_HEADING_ALIGNMENT");
      return;
    }
    const bool was_open = open();
    const Quaternion next = toMathQuaternion(message->quaternion);
    if (was_open && vision_heading_alignment_ready_ &&
        mine_uav_control::quaternionAngularDistance(
            next, planning_to_px4_heading_) >
            max_alignment_rotation_change_) {
      alignment_fault_ = true;
      ROS_ERROR("Task-one vision heading alignment changed while the planner "
                "gate was open; planner input stopped");
    }
    planning_to_px4_heading_ = next;
    vision_heading_alignment_ready_ = true;
    publishPlanningAlignment(message->header.stamp);
    updateStatus(was_open);
  }

  void publishPlanningAlignment(const ros::Time& stamp) {
    if (!alignment_ready_ || !sensor_rotation_ready_ ||
        (require_vision_heading_alignment_ &&
         !vision_heading_alignment_ready_)) {
      return;
    }
    geometry_msgs::TransformStamped output;
    output.header.stamp = stamp.isZero() ? ros::Time::now() : stamp;
    output.header.frame_id = alignment_parent_frame_;
    output.child_frame_id = output_frame_;
    // Planning remains level and starts at yaw zero. Rotate its commands into
    // the PX4 external-vision heading, including the source-origin
    // translation, so changing this heading contract cannot rotate a mission.
    const Vector3 translation = mine_uav_control::rotateVector(
        planning_to_px4_heading_, output_translation_);
    output.transform.translation.x = translation.x;
    output.transform.translation.y = translation.y;
    output.transform.translation.z = translation.z;
    output.transform.rotation = toRosQuaternion(planning_to_px4_heading_);
    planning_alignment_publisher_.publish(output);
  }

  void updateStatus(bool was_open) {
    if (was_open != open()) {
      ++generation_;
      ROS_INFO("Task-one perception gate %s (generation %u)",
               open() ? "opened" : "closed", generation_);
    }
    if (!mission_enabled_ && !additional_mission_enabled_ &&
        !second_additional_mission_enabled_) {
      publishStatus("CLOSED_WAIT_TASK1");
    } else if (alignment_fault_) {
      publishStatus("CLOSED_ALIGNMENT_CHANGED_DURING_TASK");
    } else if (!alignment_ready_ || !sensor_rotation_ready_) {
      publishStatus("CLOSED_WAIT_LEVEL_ALIGNMENT");
    } else if (require_vision_heading_alignment_ &&
               !vision_heading_alignment_ready_) {
      publishStatus("CLOSED_WAIT_VISION_HEADING_ALIGNMENT");
    } else if (require_vision_healthy_ && !vision_healthy_) {
      publishStatus("CLOSED_WAIT_VISION");
    } else {
      publishStatus("OPEN_LEVEL_PLANNING_FRAME");
    }
  }

  void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud) {
    if (!open()) {
      return;
    }
    if (!mine_uav_control::sourceStampFresh(ros::Time::now().toSec(),
            cloud->header.stamp.toSec(), 1.0)) {
      publishStatus("STALE_OR_FUTURE_CLOUD");
      return;
    }
    if (!expected_frame_.empty() && cloud->header.frame_id != expected_frame_) {
      ROS_ERROR_THROTTLE(
          1.0, "Task-one perception gate rejected cloud frame '%s', expected '%s'",
          cloud->header.frame_id.c_str(), expected_frame_.c_str());
      publishStatus("FRAME_MISMATCH");
      return;
    }
    if (!hasFloat32Field(*cloud, "x") ||
        !hasFloat32Field(*cloud, "y") ||
        !hasFloat32Field(*cloud, "z")) {
      ROS_ERROR_THROTTLE(1.0, "Task-one perception gate requires FLOAT32 x/y/z "
                              "point-cloud fields");
      publishStatus("UNSUPPORTED_CLOUD_LAYOUT");
      return;
    }

    sensor_msgs::PointCloud2 output = *cloud;
    output.header.frame_id = output_frame_;
    try {
      sensor_msgs::PointCloud2Iterator<float> x(output, "x");
      sensor_msgs::PointCloud2Iterator<float> y(output, "y");
      sensor_msgs::PointCloud2Iterator<float> z(output, "z");
      for (; x != x.end(); ++x, ++y, ++z) {
        const Vector3 rotated = mine_uav_control::positionInPlanningFrame(
            map_to_planning_, Vector3(*x, *y, *z));
        *x = static_cast<float>(rotated.x);
        *y = static_cast<float>(rotated.y);
        *z = static_cast<float>(rotated.z);
      }
    } catch (const std::runtime_error& error) {
      ROS_ERROR_THROTTLE(1.0, "Task-one cloud rotation failed: %s", error.what());
      publishStatus("UNSUPPORTED_CLOUD_LAYOUT");
      return;
    }
    cloud_publisher_.publish(output);
  }

  void publishLevelOdom(const nav_msgs::Odometry::ConstPtr& odom,
                        ros::Publisher* publisher,
                        const char* source_name) {
    // Keep the level odometry available for the scheduler and pre-task hover
    // checks.  Only point clouds remain task-gated, so takeoff returns still
    // cannot enter the planner map before mission enable.
    if (!transformReady()) {
      return;
    }
    if (!mine_uav_control::sourceStampFresh(ros::Time::now().toSec(),
            odom->header.stamp.toSec(), 1.0)) {
      publishStatus("STALE_OR_FUTURE_ODOM");
      return;
    }
    if (!expected_frame_.empty() && odom->header.frame_id != expected_frame_) {
      ROS_ERROR_THROTTLE(
          1.0, "Task-one perception gate rejected %s odom frame '%s', expected '%s'",
          source_name, odom->header.frame_id.c_str(), expected_frame_.c_str());
      publishStatus("FRAME_MISMATCH");
      return;
    }
    const auto& p = odom->pose.pose.position;
    const auto& q = odom->pose.pose.orientation;
    if (!finite(p.x) || !finite(p.y) || !finite(p.z) ||
        !finiteQuaternion(q)) {
      ROS_ERROR_THROTTLE(
          1.0, "Task-one perception gate rejected non-finite %s odometry",
          source_name);
      publishStatus("INVALID_ODOMETRY");
      return;
    }

    nav_msgs::Odometry output = *odom;
    output.header.frame_id = output_frame_;
    output.child_frame_id = output_child_frame_;
    const Vector3 position = mine_uav_control::positionInPlanningFrame(
        map_to_planning_, Vector3(p.x, p.y, p.z));
    output.pose.pose.position.x = position.x;
    output.pose.pose.position.y = position.y;
    output.pose.pose.position.z = position.z;
    output.pose.pose.orientation = toRosQuaternion(
        mine_uav_control::bodyOrientationInPlanningFrame(
            map_to_planning_, toMathQuaternion(q), sensor_from_body_));
    rotatePoseCovariance(map_to_planning_, odom->pose.covariance,
                         &output.pose.covariance);
    publisher->publish(output);
  }

  void odomCallback(const nav_msgs::Odometry::ConstPtr& odom) {
    publishLevelOdom(odom, &odom_publisher_, "optimized");
  }

  void auxiliaryOdomCallback(const nav_msgs::Odometry::ConstPtr& odom) {
    publishLevelOdom(odom, &auxiliary_odom_publisher_, "auxiliary");
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
  ros::Subscriber additional_mission_subscriber_;
  ros::Subscriber second_additional_mission_subscriber_;
  ros::Subscriber vision_subscriber_;
  ros::Subscriber alignment_subscriber_;
  ros::Subscriber sensor_rotation_subscriber_;
  ros::Subscriber vision_heading_alignment_subscriber_;
  ros::Subscriber cloud_subscriber_;
  ros::Subscriber odom_subscriber_;
  ros::Subscriber auxiliary_odom_subscriber_;
  ros::Publisher cloud_publisher_;
  ros::Publisher odom_publisher_;
  ros::Publisher auxiliary_odom_publisher_;
  ros::Publisher planning_alignment_publisher_;
  ros::Publisher status_publisher_;
  std::string input_cloud_topic_;
  std::string input_odom_topic_;
  std::string auxiliary_input_odom_topic_;
  std::string output_cloud_topic_;
  std::string output_odom_topic_;
  std::string auxiliary_output_odom_topic_;
  std::string source_alignment_topic_;
  std::string sensor_rotation_topic_;
  std::string vision_heading_alignment_topic_;
  std::string output_alignment_topic_;
  std::string mission_enable_topic_;
  std::string additional_cloud_enable_topic_;
  std::string second_additional_cloud_enable_topic_;
  std::string vision_health_topic_;
  std::string expected_frame_;
  std::string output_frame_;
  std::string output_child_frame_;
  std::string alignment_parent_frame_;
  std::string last_status_;
  Quaternion map_to_planning_;
  Quaternion sensor_from_body_;
  Quaternion planning_to_px4_heading_;
  Vector3 output_translation_;
  double max_alignment_translation_change_{0.05};
  double max_alignment_rotation_change_{0.05};
  bool require_vision_healthy_{true};
  bool require_vision_heading_alignment_{false};
  bool mission_enabled_{false};
  bool additional_mission_enabled_{false};
  bool second_additional_mission_enabled_{false};
  bool vision_healthy_{false};
  bool alignment_ready_{false};
  bool sensor_rotation_ready_{false};
  bool vision_heading_alignment_ready_{false};
  bool alignment_fault_{false};
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
