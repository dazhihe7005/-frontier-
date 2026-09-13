#include "mine_uav_control/super_exploration_decider.hpp"

#include <algorithm>
#include <limits>
#include <sstream>

#include <boost/bind/bind.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

namespace mine_uav_control {

namespace {

constexpr uint8_t kFree = 1;
constexpr uint8_t kOccupied = 2;

double squaredDistance(const geometry_msgs::Point& a,
                       const geometry_msgs::Point& b) {
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  const double dz = a.z - b.z;
  return dx * dx + dy * dy + dz * dz;
}

geometry_msgs::Quaternion yawQuaternion(double yaw) {
  geometry_msgs::Quaternion q;
  q.w = std::cos(yaw * 0.5);
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw * 0.5);
  return q;
}

}  // namespace

SuperExplorationDecider::SuperExplorationDecider(
    const ros::NodeHandle& nh, const ros::NodeHandle& private_nh)
    : nh_(nh),
      private_nh_(private_nh),
      cloud_subscriber_(nh_, "/cloud_registered", 1),
      odom_subscriber_(nh_, "/Odometry", 1) {
  loadParameters();

  cloud_subscriber_.subscribe(nh_, cloud_topic_, 1);
  odom_subscriber_.subscribe(nh_, odom_topic_, 1);
  synchronizer_.reset(new Synchronizer(
      SyncPolicy(sync_queue_size_), cloud_subscriber_, odom_subscriber_));
  synchronizer_->setMaxIntervalDuration(ros::Duration(sync_slop_));
  synchronizer_->registerCallback(
      boost::bind(&SuperExplorationDecider::synchronizedCallback, this,
                  boost::placeholders::_1, boost::placeholders::_2));

  goal_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>(goal_topic_, 1,
                                                                true);
  status_publisher_ = nh_.advertise<std_msgs::String>(status_topic_, 1, true);
  finished_publisher_ = nh_.advertise<std_msgs::Bool>(finished_topic_, 1, true);
  returning_publisher_ =
      nh_.advertise<std_msgs::Bool>(returning_topic_, 1, true);
  visualization_publisher_ =
      nh_.advertise<visualization_msgs::MarkerArray>(visualization_topic_, 1);

  return_request_subscriber_ = nh_.subscribe(
      return_request_topic_, 1,
      &SuperExplorationDecider::returnRequestCallback, this);
  battery_subscriber_ =
      nh_.subscribe(battery_topic_, 1, &SuperExplorationDecider::batteryCallback,
                    this);
  enable_service_ = private_nh_.advertiseService(
      "enable", &SuperExplorationDecider::enableCallback, this);
  reset_service_ = private_nh_.advertiseService(
      "reset", &SuperExplorationDecider::resetCallback, this);

  decision_timer_ = nh_.createTimer(
      ros::Duration(1.0 / decision_rate_),
      &SuperExplorationDecider::decisionTimerCallback, this);

  std_msgs::Bool false_msg;
  false_msg.data = false;
  finished_publisher_.publish(false_msg);
  returning_publisher_.publish(false_msg);
  publishStatus("WAIT_DATA", "waiting for synchronized Fast-LIO2 data");
}

bool SuperExplorationDecider::loadParameters() {
  private_nh_.param("cloud_topic", cloud_topic_, std::string("/cloud_registered"));
  private_nh_.param("odom_topic", odom_topic_, std::string("/Odometry"));
  private_nh_.param("goal_topic", goal_topic_, std::string("/goal"));
  private_nh_.param("world_frame", world_frame_, std::string("world"));
  private_nh_.param("return_request_topic", return_request_topic_,
                    std::string("/mine_uav/exploration/return_home"));
  private_nh_.param("battery_topic", battery_topic_,
                    std::string("/mavros/battery"));
  private_nh_.param("status_topic", status_topic_,
                    std::string("/mine_uav/exploration/status"));
  private_nh_.param("finished_topic", finished_topic_,
                    std::string("/mine_uav/exploration/finished"));
  private_nh_.param("returning_topic", returning_topic_,
                    std::string("/mine_uav/exploration/returning"));
  private_nh_.param("visualization_topic", visualization_topic_,
                    std::string("/mine_uav/exploration/frontiers"));

  private_nh_.param("voxel_resolution", voxel_resolution_, voxel_resolution_);
  private_nh_.param("max_map_radius", max_map_radius_, max_map_radius_);
  private_nh_.param("raycast_max_range", raycast_max_range_, raycast_max_range_);
  private_nh_.param("frontier_search_radius", frontier_search_radius_,
                    frontier_search_radius_);
  private_nh_.param("min_goal_distance", min_goal_distance_, min_goal_distance_);
  private_nh_.param("goal_reached_distance", goal_reached_distance_,
                    goal_reached_distance_);
  private_nh_.param("goal_timeout", goal_timeout_, goal_timeout_);
  private_nh_.param("no_frontier_timeout", no_frontier_timeout_,
                    no_frontier_timeout_);
  private_nh_.param("min_data_duration", min_data_duration_, min_data_duration_);
  private_nh_.param("candidate_spacing", candidate_spacing_, candidate_spacing_);
  private_nh_.param("vehicle_radius", vehicle_radius_, vehicle_radius_);
  private_nh_.param("data_timeout", data_timeout_, data_timeout_);
  private_nh_.param("decision_rate", decision_rate_, decision_rate_);
  private_nh_.param("sync_slop", sync_slop_, sync_slop_);
  private_nh_.param("battery_return_threshold", battery_return_threshold_,
                    battery_return_threshold_);
  private_nh_.param("distance_weight", distance_weight_, distance_weight_);
  private_nh_.param("information_weight", information_weight_,
                    information_weight_);
  private_nh_.param("sync_queue_size", sync_queue_size_, sync_queue_size_);
  private_nh_.param("max_points_per_cloud", max_points_per_cloud_,
                    max_points_per_cloud_);
  private_nh_.param("min_unknown_neighbors", min_unknown_neighbors_,
                    min_unknown_neighbors_);
  private_nh_.param("min_goals_before_complete", min_goals_before_complete_,
                    min_goals_before_complete_);
  private_nh_.param("raycast_enable", raycast_enable_, raycast_enable_);
  private_nh_.param("strict_cloud_frame", strict_cloud_frame_,
                    strict_cloud_frame_);

  if (!std::isfinite(voxel_resolution_) || voxel_resolution_ <= 0.0 ||
      !std::isfinite(decision_rate_) || decision_rate_ <= 0.0 ||
      sync_queue_size_ < 2 || max_points_per_cloud_ < 1) {
    ROS_FATAL("Invalid exploration decider parameters");
    return false;
  }
  if (min_unknown_neighbors_ < 1) {
    min_unknown_neighbors_ = 1;
  }
  return true;
}

void SuperExplorationDecider::synchronizedCallback(
    const Cloud::ConstPtr& cloud, const Odom::ConstPtr& odom) {
  if (strict_cloud_frame_ && !cloud->header.frame_id.empty() &&
      cloud->header.frame_id != world_frame_) {
    ROS_WARN_THROTTLE(
        2.0,
        "Rejecting cloud frame '%s'; expected '%s'. No TF conversion is done.",
        cloud->header.frame_id.c_str(), world_frame_.c_str());
    return;
  }
  if (!cloud->header.frame_id.empty() && cloud->header.frame_id != world_frame_) {
    ROS_WARN_THROTTLE(
        5.0,
        "Cloud frame is '%s', configured world_frame is '%s'; assuming the "
        "coordinates are already in the same world frame.",
        cloud->header.frame_id.c_str(), world_frame_.c_str());
  }

  current_pose_.header = odom->header;
  current_pose_.header.frame_id = world_frame_;
  current_pose_.pose = odom->pose.pose;
  last_sync_time_ = ros::Time::now();
  have_data_ = true;
  if (first_data_time_.isZero()) {
    first_data_time_ = last_sync_time_;
  }
  if (!have_home_) {
    home_pose_ = current_pose_;
    home_pose_.header.frame_id = world_frame_;
    have_home_ = true;
    ROS_INFO("Exploration home captured at %.2f %.2f %.2f",
             home_pose_.pose.position.x, home_pose_.pose.position.y,
             home_pose_.pose.position.z);
  }
  updateMap(*cloud, current_pose_);
}

bool SuperExplorationDecider::dataIsFresh() const {
  if (!have_data_ || last_sync_time_.isZero()) {
    return false;
  }
  return (ros::Time::now() - last_sync_time_).toSec() <= data_timeout_;
}

VoxelKey SuperExplorationDecider::positionToKey(double x, double y,
                                                 double z) const {
  return {static_cast<int>(std::floor(x / voxel_resolution_)),
          static_cast<int>(std::floor(y / voxel_resolution_)),
          static_cast<int>(std::floor(z / voxel_resolution_))};
}

geometry_msgs::Point SuperExplorationDecider::keyToPoint(
    const VoxelKey& key) const {
  geometry_msgs::Point point;
  point.x = (static_cast<double>(key.x) + 0.5) * voxel_resolution_;
  point.y = (static_cast<double>(key.y) + 0.5) * voxel_resolution_;
  point.z = (static_cast<double>(key.z) + 0.5) * voxel_resolution_;
  return point;
}

void SuperExplorationDecider::markFree(const VoxelKey& key) {
  auto it = voxels_.find(key);
  if (it == voxels_.end()) {
    voxels_.emplace(key, kFree);
  } else if (it->second != kOccupied) {
    it->second = kFree;
  }
}

void SuperExplorationDecider::markOccupied(const VoxelKey& key) {
  voxels_[key] = kOccupied;
}

bool SuperExplorationDecider::isOccupied(const VoxelKey& key) const {
  const auto it = voxels_.find(key);
  return it != voxels_.end() && it->second == kOccupied;
}

void SuperExplorationDecider::updateMap(
    const sensor_msgs::PointCloud2& cloud,
    const geometry_msgs::PoseStamped& pose) {
  pcl::PointCloud<pcl::PointXYZ> input;
  try {
    pcl::fromROSMsg(cloud, input);
  } catch (const std::exception& error) {
    ROS_WARN_THROTTLE(2.0, "Cannot convert Fast-LIO2 PointCloud2: %s",
                      error.what());
    return;
  }
  if (input.empty()) {
    return;
  }

  const VoxelKey sensor_key = positionToKey(
      pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
  markFree(sensor_key);

  const int stride = std::max(
      1, static_cast<int>((input.size() + max_points_per_cloud_ - 1) /
                          max_points_per_cloud_));
  int accepted = 0;
  for (std::size_t index = 0; index < input.size(); index += stride) {
    const auto& point = input[index];
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z)) {
      continue;
    }
    const double dx = point.x - pose.pose.position.x;
    const double dy = point.y - pose.pose.position.y;
    const double dz = point.z - pose.pose.position.z;
    const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (distance < voxel_resolution_ * 0.5 || distance > raycast_max_range_) {
      continue;
    }

    const VoxelKey endpoint = positionToKey(point.x, point.y, point.z);
    markOccupied(endpoint);
    ++accepted;

    if (!raycast_enable_) {
      continue;
    }
    const int steps = std::max(
        1, static_cast<int>(std::ceil(distance / voxel_resolution_)));
    for (int step = 1; step < steps; ++step) {
      const double ratio = static_cast<double>(step) / steps;
      const VoxelKey ray_key = positionToKey(
          pose.pose.position.x + ratio * dx,
          pose.pose.position.y + ratio * dy,
          pose.pose.position.z + ratio * dz);
      if (ray_key == endpoint) {
        break;
      }
      if (!isOccupied(ray_key)) {
        markFree(ray_key);
      }
    }
  }

  if (accepted > 0) {
    pruneMap();
  }
}

void SuperExplorationDecider::pruneMap() {
  const double radius_squared = max_map_radius_ * max_map_radius_;
  for (auto it = voxels_.begin(); it != voxels_.end();) {
    const geometry_msgs::Point point = keyToPoint(it->first);
    if (squaredDistance(point, current_pose_.pose.position) > radius_squared) {
      it = voxels_.erase(it);
    } else {
      ++it;
    }
  }
}

bool SuperExplorationDecider::isClearForVehicle(const VoxelKey& key) const {
  const int radius = std::max(
      1, static_cast<int>(std::ceil(vehicle_radius_ / voxel_resolution_)));
  for (int dx = -radius; dx <= radius; ++dx) {
    for (int dy = -radius; dy <= radius; ++dy) {
      for (int dz = -radius; dz <= radius; ++dz) {
        if (dx * dx + dy * dy + dz * dz > radius * radius) {
          continue;
        }
        if (isOccupied({key.x + dx, key.y + dy, key.z + dz})) {
          return false;
        }
      }
    }
  }
  return true;
}

bool SuperExplorationDecider::isNearCoveredGoal(
    const geometry_msgs::Point& point) const {
  const double threshold_squared = candidate_spacing_ * candidate_spacing_;
  for (const auto& covered : covered_goals_) {
    if (squaredDistance(point, covered) < threshold_squared) {
      return true;
    }
  }
  return false;
}

std::vector<SuperExplorationDecider::FrontierCandidate>
SuperExplorationDecider::findFrontiers() const {
  std::vector<FrontierCandidate> candidates;
  const double max_distance_squared =
      frontier_search_radius_ * frontier_search_radius_;
  const double min_distance_squared = min_goal_distance_ * min_goal_distance_;

  for (const auto& entry : voxels_) {
    if (entry.second != kFree) {
      continue;
    }
    const geometry_msgs::Point point = keyToPoint(entry.first);
    const double distance_squared =
        squaredDistance(point, current_pose_.pose.position);
    if (distance_squared < min_distance_squared ||
        distance_squared > max_distance_squared ||
        !isClearForVehicle(entry.first) || isNearCoveredGoal(point)) {
      continue;
    }

    int unknown_neighbors = 0;
    double direction_x = 0.0;
    double direction_y = 0.0;
    double direction_z = 0.0;
    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dz = -1; dz <= 1; ++dz) {
          if (dx == 0 && dy == 0 && dz == 0) {
            continue;
          }
          const VoxelKey neighbor{entry.first.x + dx, entry.first.y + dy,
                                  entry.first.z + dz};
          if (voxels_.find(neighbor) == voxels_.end()) {
            ++unknown_neighbors;
            direction_x += dx;
            direction_y += dy;
            direction_z += dz;
          }
        }
      }
    }
    if (unknown_neighbors < min_unknown_neighbors_) {
      continue;
    }

    FrontierCandidate candidate;
    candidate.key = entry.first;
    candidate.goal.header.frame_id = world_frame_;
    candidate.goal.header.stamp = ros::Time::now();
    candidate.goal.pose.position = point;
    const double yaw = (std::abs(direction_x) + std::abs(direction_y) > 1e-6)
                           ? std::atan2(direction_y, direction_x)
                           : std::atan2(current_pose_.pose.position.y - point.y,
                                        current_pose_.pose.position.x - point.x);
    candidate.goal.pose.orientation = yawQuaternion(yaw);
    candidate.unknown_neighbors = unknown_neighbors;
    candidate.score = information_weight_ * unknown_neighbors -
                      distance_weight_ * std::sqrt(distance_squared);
    candidates.push_back(candidate);
  }

  std::sort(candidates.begin(), candidates.end(),
            [](const FrontierCandidate& left, const FrontierCandidate& right) {
              return left.score > right.score;
            });
  return candidates;
}

void SuperExplorationDecider::publishVisualization(
    const std::vector<FrontierCandidate>& candidates) {
  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker points;
  points.header.frame_id = world_frame_;
  points.header.stamp = ros::Time::now();
  points.ns = "frontier_candidates";
  points.id = 0;
  points.type = visualization_msgs::Marker::SPHERE_LIST;
  points.action = visualization_msgs::Marker::ADD;
  points.pose.orientation.w = 1.0;
  points.scale.x = voxel_resolution_ * 0.5;
  points.scale.y = voxel_resolution_ * 0.5;
  points.scale.z = voxel_resolution_ * 0.5;
  points.color.r = 0.1;
  points.color.g = 1.0;
  points.color.b = 0.1;
  points.color.a = 0.8;
  const std::size_t max_visualized = std::min<std::size_t>(candidates.size(), 200);
  for (std::size_t index = 0; index < max_visualized; ++index) {
    points.points.push_back(candidates[index].goal.pose.position);
  }
  marker_array.markers.push_back(points);

  if (have_active_goal_) {
    visualization_msgs::Marker goal;
    goal.header = points.header;
    goal.ns = "active_frontier_goal";
    goal.id = 1;
    goal.type = visualization_msgs::Marker::ARROW;
    goal.action = visualization_msgs::Marker::ADD;
    goal.pose = current_goal_.pose;
    goal.scale.x = 1.0;
    goal.scale.y = 0.12;
    goal.scale.z = 0.12;
    goal.color.r = 1.0;
    goal.color.g = 0.2;
    goal.color.b = 0.1;
    goal.color.a = 1.0;
    marker_array.markers.push_back(goal);
  }
  visualization_publisher_.publish(marker_array);
}

void SuperExplorationDecider::publishGoal(
    const geometry_msgs::PoseStamped& goal, const std::string& reason) {
  current_goal_ = goal;
  current_goal_.header.frame_id = world_frame_;
  current_goal_.header.stamp = ros::Time::now();
  goal_publisher_.publish(current_goal_);
  goal_sent_time_ = current_goal_.header.stamp;
  have_active_goal_ = true;
  exploration_started_ = true;
  ROS_INFO("New SUPER goal (%s): %.2f %.2f %.2f", reason.c_str(),
           current_goal_.pose.position.x, current_goal_.pose.position.y,
           current_goal_.pose.position.z);
}

bool SuperExplorationDecider::selectAndPublishFrontier() {
  const auto candidates = findFrontiers();
  publishVisualization(candidates);
  if (candidates.empty()) {
    return false;
  }

  last_frontier_time_ = ros::Time::now();
  publishGoal(candidates.front().goal, "frontier_information_gain");
  return true;
}

void SuperExplorationDecider::beginReturnHome(const std::string& reason) {
  if (!have_home_ || returning_home_ || mission_finished_) {
    return;
  }
  returning_home_ = true;
  have_active_goal_ = false;
  publishGoal(home_pose_, reason);
  std_msgs::Bool message;
  message.data = true;
  returning_publisher_.publish(message);
  publishStatus("RETURNING", reason);
}

void SuperExplorationDecider::decisionTimerCallback(const ros::TimerEvent&) {
  if (!enabled_) {
    publishStatus("DISABLED");
    return;
  }
  if (!dataIsFresh() || !have_home_) {
    publishStatus("WAIT_DATA", "Fast-LIO2 synchronized data is stale or absent");
    return;
  }

  if (have_battery_ && battery_percentage_ >= 0.0 &&
      battery_percentage_ <= battery_return_threshold_) {
    return_requested_ = true;
  }
  if (return_requested_) {
    beginReturnHome("external request or low battery");
  }

  if (returning_home_) {
    if (squaredDistance(current_pose_.pose.position,
                       home_pose_.pose.position) <=
        goal_reached_distance_ * goal_reached_distance_) {
      returning_home_ = false;
      mission_finished_ = true;
      have_active_goal_ = false;
      std_msgs::Bool message;
      message.data = true;
      finished_publisher_.publish(message);
      message.data = false;
      returning_publisher_.publish(message);
      publishStatus("COMPLETE", "home reached");
    } else {
      publishStatus("RETURNING", "following home goal through SUPER");
    }
    return;
  }
  if (mission_finished_) {
    publishStatus("COMPLETE");
    return;
  }

  const auto candidates = findFrontiers();
  publishVisualization(candidates);
  if (!candidates.empty()) {
    last_frontier_time_ = ros::Time::now();
  }

  if (have_active_goal_) {
    const double distance = std::sqrt(squaredDistance(
        current_pose_.pose.position, current_goal_.pose.position));
    const double goal_age = (ros::Time::now() - goal_sent_time_).toSec();
    if (distance <= goal_reached_distance_) {
      covered_goals_.push_back(current_goal_.pose.position);
      ++reached_goal_count_;
      have_active_goal_ = false;
      publishStatus("GOAL_REACHED", "selecting next frontier");
    } else if (goal_age > goal_timeout_) {
      covered_goals_.push_back(current_goal_.pose.position);
      have_active_goal_ = false;
      publishStatus("GOAL_TIMEOUT", "discarding unreachable candidate");
    } else {
      publishStatus("EXPLORING", "SUPER is following the active frontier goal");
      return;
    }
  }

  if (!have_active_goal_ && selectAndPublishFrontier()) {
    publishStatus("EXPLORING", "new frontier goal sent to SUPER");
    return;
  }

  if (exploration_started_ &&
      reached_goal_count_ >= min_goals_before_complete_ &&
      !first_data_time_.isZero() &&
      (ros::Time::now() - first_data_time_).toSec() >= min_data_duration_ &&
      !last_frontier_time_.isZero() &&
      (ros::Time::now() - last_frontier_time_).toSec() >=
          no_frontier_timeout_) {
    publishStatus("MODEL_COMPLETE", "no new frontier for configured timeout");
    beginReturnHome("exploration complete");
  } else {
    publishStatus("WAIT_FRONTIER", "no usable frontier yet");
  }
}

void SuperExplorationDecider::returnRequestCallback(
    const std_msgs::Bool::ConstPtr& message) {
  if (message->data) {
    return_requested_ = true;
  }
}

void SuperExplorationDecider::batteryCallback(
    const sensor_msgs::BatteryState::ConstPtr& message) {
  if (std::isfinite(message->percentage) && message->percentage >= 0.0) {
    have_battery_ = true;
    battery_percentage_ = message->percentage;
  }
}

void SuperExplorationDecider::publishStatus(const std::string& state,
                                            const std::string& detail) {
  std_msgs::String message;
  message.data = detail.empty() ? state : state + ": " + detail;
  status_publisher_.publish(message);
}

void SuperExplorationDecider::clearMissionState() {
  voxels_.clear();
  covered_goals_.clear();
  current_goal_ = geometry_msgs::PoseStamped();
  have_data_ = false;
  have_home_ = false;
  have_active_goal_ = false;
  exploration_started_ = false;
  returning_home_ = false;
  mission_finished_ = false;
  return_requested_ = false;
  have_battery_ = false;
  battery_percentage_ = -1.0;
  reached_goal_count_ = 0;
  first_data_time_ = ros::Time();
  last_frontier_time_ = ros::Time();
  std_msgs::Bool false_message;
  false_message.data = false;
  finished_publisher_.publish(false_message);
  returning_publisher_.publish(false_message);
}

bool SuperExplorationDecider::enableCallback(
    std_srvs::SetBool::Request& request,
    std_srvs::SetBool::Response& response) {
  enabled_ = request.data;
  response.success = true;
  response.message = enabled_ ? "exploration enabled" : "exploration paused";
  publishStatus(enabled_ ? "WAIT_DATA" : "DISABLED", response.message);
  return true;
}

bool SuperExplorationDecider::resetCallback(
    std_srvs::Trigger::Request&, std_srvs::Trigger::Response& response) {
  clearMissionState();
  enabled_ = true;
  response.success = true;
  response.message = "exploration state reset";
  publishStatus("WAIT_DATA", response.message);
  return true;
}

}  // namespace mine_uav_control

int main(int argc, char** argv) {
  ros::init(argc, argv, "super_exploration_decider");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  mine_uav_control::SuperExplorationDecider decider(nh, private_nh);
  ros::spin();
  return 0;
}
