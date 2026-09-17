#include "mine_uav_control/super_exploration_decider.hpp"

#include <algorithm>
#include <iomanip>
#include <limits>
#include <map>
#include <queue>
#include <set>
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

double normalizeAngle(double angle) {
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

double poseYaw(const geometry_msgs::Pose& pose) {
  const auto& q = pose.orientation;
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
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
  if (require_mission_enable_edge_) {
    enabled_ = false;
  }

  cloud_subscriber_.subscribe(nh_, cloud_topic_, 1);
  odom_subscriber_.subscribe(nh_, odom_topic_, 1);
  synchronizer_.reset(new Synchronizer(
      SyncPolicy(sync_queue_size_), cloud_subscriber_, odom_subscriber_));
  synchronizer_->setMaxIntervalDuration(ros::Duration(sync_slop_));
  synchronizer_->registerCallback(
      boost::bind(&SuperExplorationDecider::synchronizedCallback, this,
                  boost::placeholders::_1, boost::placeholders::_2));

  goal_publisher_ = nh_.advertise<super_planner::GoalCommand>(
      goal_command_topic_, 20, true);
  cancelActiveGoal("startup", true);
  status_publisher_ = nh_.advertise<std_msgs::String>(status_topic_, 1, true);
  finished_publisher_ = nh_.advertise<std_msgs::Bool>(finished_topic_, 1, true);
  returning_publisher_ =
      nh_.advertise<std_msgs::Bool>(returning_topic_, 1, true);
  model_complete_publisher_ =
      nh_.advertise<std_msgs::Bool>(model_complete_topic_, 1, true);
  coverage_status_publisher_ =
      nh_.advertise<std_msgs::String>(coverage_status_topic_, 1, true);
  visualization_publisher_ =
      nh_.advertise<visualization_msgs::MarkerArray>(visualization_topic_, 1);

  return_request_subscriber_ = nh_.subscribe(
      return_request_topic_, 1,
      &SuperExplorationDecider::returnRequestCallback, this);
  mission_enable_subscriber_ = nh_.subscribe(
      mission_enable_topic_, 1,
      &SuperExplorationDecider::missionEnableCallback, this);
  battery_subscriber_ =
      nh_.subscribe(battery_topic_, 1, &SuperExplorationDecider::batteryCallback,
                    this);
  if (!free_ray_topic_.empty()) {
    free_ray_subscriber_ = nh_.subscribe(
        free_ray_topic_, 1, &SuperExplorationDecider::freeRayCallback, this);
  }
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
  model_complete_publisher_.publish(false_msg);
  publishStatus("WAIT_DATA", "waiting for synchronized Fast-LIO2 data");
}

bool SuperExplorationDecider::loadParameters() {
  private_nh_.param("cloud_topic", cloud_topic_, std::string("/cloud_registered"));
  private_nh_.param("free_ray_topic", free_ray_topic_, std::string());
  private_nh_.param("odom_topic", odom_topic_, std::string("/Odometry"));
  private_nh_.param("goal_command_topic", goal_command_topic_,
                    std::string("/mine_uav/super/goal_command"));
  private_nh_.param("world_frame", world_frame_, std::string("world"));
  private_nh_.param("return_request_topic", return_request_topic_,
                    std::string("/mine_uav/exploration/return_home"));
  private_nh_.param("mission_enable_topic", mission_enable_topic_,
                    std::string("/mine_uav/mission/goaf_enable"));
  private_nh_.param("battery_topic", battery_topic_,
                    std::string("/mavros/battery"));
  private_nh_.param("status_topic", status_topic_,
                    std::string("/mine_uav/exploration/status"));
  private_nh_.param("finished_topic", finished_topic_,
                    std::string("/mine_uav/exploration/finished"));
  private_nh_.param("returning_topic", returning_topic_,
                    std::string("/mine_uav/exploration/returning"));
  private_nh_.param("model_complete_topic", model_complete_topic_,
                    std::string("/mine_uav/exploration/model_complete"));
  private_nh_.param("coverage_status_topic", coverage_status_topic_,
                    std::string("/mine_uav/exploration/model_coverage"));
  private_nh_.param("visualization_topic", visualization_topic_,
                    std::string("/mine_uav/exploration/frontiers"));
  private_nh_.param("require_mission_enable_edge",
                    require_mission_enable_edge_,
                    require_mission_enable_edge_);

  private_nh_.param("voxel_resolution", voxel_resolution_, voxel_resolution_);
  private_nh_.param("max_map_radius", max_map_radius_, max_map_radius_);
  private_nh_.param("raycast_max_range", raycast_max_range_, raycast_max_range_);
  private_nh_.param("frontier_search_radius", frontier_search_radius_,
                    frontier_search_radius_);
  private_nh_.param("max_exploration_radius_from_home",
                    max_exploration_radius_from_home_,
                    max_exploration_radius_from_home_);
  private_nh_.param("min_goal_distance", min_goal_distance_, min_goal_distance_);
  private_nh_.param("goal_reached_distance", goal_reached_distance_,
                    goal_reached_distance_);
  private_nh_.param("goal_timeout", goal_timeout_, goal_timeout_);
  private_nh_.param("no_frontier_timeout", no_frontier_timeout_,
                    no_frontier_timeout_);
  private_nh_.param("min_data_duration", min_data_duration_, min_data_duration_);
  private_nh_.param("candidate_spacing", candidate_spacing_, candidate_spacing_);
  private_nh_.param("vehicle_radius", vehicle_radius_, vehicle_radius_);
  private_nh_.param("min_observation_height_above_home",
                    min_observation_height_above_home_,
                    min_observation_height_above_home_);
  private_nh_.param("max_observation_height_above_home",
                    max_observation_height_above_home_,
                    max_observation_height_above_home_);
  private_nh_.param("data_timeout", data_timeout_, data_timeout_);
  private_nh_.param("decision_rate", decision_rate_, decision_rate_);
  private_nh_.param("sync_slop", sync_slop_, sync_slop_);
  private_nh_.param("battery_return_threshold", battery_return_threshold_,
                    battery_return_threshold_);
  private_nh_.param("return_home_height_offset", return_home_height_offset_,
                    return_home_height_offset_);
  private_nh_.param("return_breadcrumb_spacing", return_breadcrumb_spacing_,
                    return_breadcrumb_spacing_);
  private_nh_.param("distance_weight", distance_weight_, distance_weight_);
  private_nh_.param("information_weight", information_weight_,
                    information_weight_);
  private_nh_.param("heading_priority_weight", heading_priority_weight_,
                    heading_priority_weight_);
  private_nh_.param("fallback_heading_weight", fallback_heading_weight_,
                    fallback_heading_weight_);
  private_nh_.param("forward_sector_deg", forward_sector_deg_,
                    forward_sector_deg_);
  private_nh_.param("side_wall_sector_deg", side_wall_sector_deg_,
                    side_wall_sector_deg_);
  private_nh_.param("side_wall_min_range", side_wall_min_range_,
                    side_wall_min_range_);
  private_nh_.param("side_wall_max_range", side_wall_max_range_,
                    side_wall_max_range_);
  private_nh_.param("front_obstacle_range", front_obstacle_range_,
                    front_obstacle_range_);
  private_nh_.param("front_obstacle_sector_deg", front_obstacle_sector_deg_,
                    front_obstacle_sector_deg_);
  private_nh_.param("front_obstacle_min_points", front_obstacle_min_points_,
                    front_obstacle_min_points_);
  private_nh_.param("front_obstacle_min_lateral_span",
                    front_obstacle_min_lateral_span_,
                    front_obstacle_min_lateral_span_);
  private_nh_.param("front_obstacle_min_vertical_span",
                    front_obstacle_min_vertical_span_,
                    front_obstacle_min_vertical_span_);
  private_nh_.param("forward_corridor_half_width",
                    forward_corridor_half_width_,
                    forward_corridor_half_width_);
  private_nh_.param("max_task_lateral_offset", max_task_lateral_offset_,
                    max_task_lateral_offset_);
  private_nh_.param("forward_progress_weight", forward_progress_weight_,
                    forward_progress_weight_);
  private_nh_.param("forward_lateral_penalty", forward_lateral_penalty_,
                    forward_lateral_penalty_);
  private_nh_.param("forward_height_penalty", forward_height_penalty_,
                    forward_height_penalty_);
  private_nh_.param("forward_goal_handover_distance",
                    forward_goal_handover_distance_,
                    forward_goal_handover_distance_);
  private_nh_.param("forward_lookahead_distance", forward_lookahead_distance_,
                    forward_lookahead_distance_);
  private_nh_.param("forward_lookahead_step", forward_lookahead_step_,
                    forward_lookahead_step_);
  private_nh_.param("cruise_height_above_home", cruise_height_above_home_,
                    cruise_height_above_home_);
  private_nh_.param("directional_vertical_tolerance",
                    directional_vertical_tolerance_,
                    directional_vertical_tolerance_);
  private_nh_.param("directional_floor_exclusion",
                    directional_floor_exclusion_,
                    directional_floor_exclusion_);
  private_nh_.param("wall_coverage_bin_size", wall_coverage_bin_size_,
                    wall_coverage_bin_size_);
  private_nh_.param("wall_coverage_min_depth", wall_coverage_min_depth_,
                    wall_coverage_min_depth_);
  private_nh_.param("wall_coverage_end_min_depth",
                    wall_coverage_end_min_depth_,
                    wall_coverage_end_min_depth_);
  private_nh_.param("wall_coverage_min_ratio", wall_coverage_min_ratio_,
                    wall_coverage_min_ratio_);
  private_nh_.param("wall_coverage_side_min_distance",
                    wall_coverage_side_min_distance_,
                    wall_coverage_side_min_distance_);
  private_nh_.param("wall_coverage_side_max_distance",
                    wall_coverage_side_max_distance_,
                    wall_coverage_side_max_distance_);
  private_nh_.param("wall_coverage_min_height", wall_coverage_min_height_,
                    wall_coverage_min_height_);
  private_nh_.param("wall_coverage_max_height", wall_coverage_max_height_,
                    wall_coverage_max_height_);
  private_nh_.param("wall_coverage_end_min_span",
                    wall_coverage_end_min_span_,
                    wall_coverage_end_min_span_);
  private_nh_.param("wall_coverage_end_center_half_width",
                    wall_coverage_end_center_half_width_,
                    wall_coverage_end_center_half_width_);
  private_nh_.param("wall_coverage_end_approach_distance",
                    wall_coverage_end_approach_distance_,
                    wall_coverage_end_approach_distance_);
  private_nh_.param("wall_coverage_end_standoff_distance",
                    wall_coverage_end_standoff_distance_,
                    wall_coverage_end_standoff_distance_);
  private_nh_.param("side_wall_missing_confirm_frames",
                    side_wall_missing_confirm_frames_,
                    side_wall_missing_confirm_frames_);
  private_nh_.param("front_obstacle_confirm_frames",
                    front_obstacle_confirm_frames_,
                    front_obstacle_confirm_frames_);
  private_nh_.param("wall_coverage_max_gap_bins",
                    wall_coverage_max_gap_bins_,
                    wall_coverage_max_gap_bins_);
  private_nh_.param("wall_coverage_end_max_gap_bins",
                    wall_coverage_end_max_gap_bins_,
                    wall_coverage_end_max_gap_bins_);
  private_nh_.param("three_wall_confirm_cycles", three_wall_confirm_cycles_,
                    three_wall_confirm_cycles_);
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
  private_nh_.param("require_three_wall_completion",
                    require_three_wall_completion_,
                    require_three_wall_completion_);
  private_nh_.param("use_map_closure_completion", use_map_closure_completion_,
                    use_map_closure_completion_);
  private_nh_.param("map_closure_min_progress", map_closure_min_progress_,
                    map_closure_min_progress_);
  private_nh_.param("map_closure_no_frontier_time",
                    map_closure_no_frontier_time_,
                    map_closure_no_frontier_time_);
  private_nh_.param("map_closure_stable_time", map_closure_stable_time_,
                    map_closure_stable_time_);
  private_nh_.param("map_closure_max_actionable_frontiers",
                    map_closure_max_actionable_frontiers_,
                    map_closure_max_actionable_frontiers_);
  private_nh_.param("map_closure_growth_voxels", map_closure_growth_voxels_,
                    map_closure_growth_voxels_);
  private_nh_.param("map_closure_confirm_cycles",
                    map_closure_confirm_cycles_,
                    map_closure_confirm_cycles_);
  private_nh_.param("map_closure_require_front_boundary",
                    map_closure_require_front_boundary_,
                    map_closure_require_front_boundary_);
  private_nh_.param("max_reachable_voxels", max_reachable_voxels_,
                    max_reachable_voxels_);
  private_nh_.param("max_frontier_candidates", max_frontier_candidates_,
                    max_frontier_candidates_);

  if (!std::isfinite(voxel_resolution_) || voxel_resolution_ <= 0.0 ||
      !std::isfinite(decision_rate_) || decision_rate_ <= 0.0 ||
      !std::isfinite(max_exploration_radius_from_home_) ||
      max_exploration_radius_from_home_ <= 0.0 ||
      !std::isfinite(return_home_height_offset_) ||
      return_home_height_offset_ < 0.0 ||
      !std::isfinite(return_breadcrumb_spacing_) ||
      return_breadcrumb_spacing_ < 1.0 ||
      !std::isfinite(heading_priority_weight_) ||
      !std::isfinite(fallback_heading_weight_) ||
      !std::isfinite(forward_sector_deg_) || forward_sector_deg_ <= 0.0 ||
      forward_sector_deg_ >= 180.0 ||
      !std::isfinite(side_wall_sector_deg_) || side_wall_sector_deg_ <= 0.0 ||
      side_wall_sector_deg_ >= 180.0 ||
      !std::isfinite(side_wall_min_range_) || side_wall_min_range_ <= 0.0 ||
      !std::isfinite(side_wall_max_range_) ||
      side_wall_max_range_ <= side_wall_min_range_ ||
      !std::isfinite(front_obstacle_range_) || front_obstacle_range_ <= 0.0 ||
      !std::isfinite(front_obstacle_sector_deg_) ||
      front_obstacle_sector_deg_ <= 0.0 ||
      front_obstacle_sector_deg_ >= 180.0 ||
      front_obstacle_min_points_ < 1 ||
      !std::isfinite(front_obstacle_min_lateral_span_) ||
      front_obstacle_min_lateral_span_ <= 0.0 ||
      !std::isfinite(front_obstacle_min_vertical_span_) ||
      front_obstacle_min_vertical_span_ <= 0.0 ||
      !std::isfinite(forward_corridor_half_width_) ||
      forward_corridor_half_width_ <= vehicle_radius_ ||
      !std::isfinite(max_task_lateral_offset_) ||
      max_task_lateral_offset_ < forward_corridor_half_width_ ||
      !std::isfinite(forward_progress_weight_) ||
      forward_progress_weight_ <= 0.0 ||
      !std::isfinite(forward_lateral_penalty_) ||
      forward_lateral_penalty_ < 0.0 ||
      !std::isfinite(forward_height_penalty_) ||
      forward_height_penalty_ < 0.0 ||
      !std::isfinite(forward_goal_handover_distance_) ||
      forward_goal_handover_distance_ <= goal_reached_distance_ ||
      !std::isfinite(forward_lookahead_distance_) ||
      forward_lookahead_distance_ <= forward_goal_handover_distance_ ||
      !std::isfinite(forward_lookahead_step_) ||
      forward_lookahead_step_ <= 0.0 ||
      !std::isfinite(cruise_height_above_home_) ||
      cruise_height_above_home_ < min_observation_height_above_home_ ||
      cruise_height_above_home_ > max_observation_height_above_home_ ||
      !std::isfinite(directional_vertical_tolerance_) ||
      directional_vertical_tolerance_ <= 0.0 ||
      !std::isfinite(directional_floor_exclusion_) ||
      directional_floor_exclusion_ < 0.0 ||
      !std::isfinite(wall_coverage_bin_size_) ||
      wall_coverage_bin_size_ <= 0.0 ||
      !std::isfinite(wall_coverage_min_depth_) ||
      wall_coverage_min_depth_ <= wall_coverage_bin_size_ ||
      !std::isfinite(wall_coverage_end_min_depth_) ||
      wall_coverage_end_min_depth_ < wall_coverage_min_depth_ ||
      !std::isfinite(wall_coverage_min_ratio_) ||
      wall_coverage_min_ratio_ <= 0.0 || wall_coverage_min_ratio_ > 1.0 ||
      !std::isfinite(wall_coverage_side_min_distance_) ||
      wall_coverage_side_min_distance_ <= 0.0 ||
      !std::isfinite(wall_coverage_side_max_distance_) ||
      wall_coverage_side_max_distance_ <=
          wall_coverage_side_min_distance_ ||
      !std::isfinite(wall_coverage_min_height_) ||
      wall_coverage_min_height_ < 0.0 ||
      !std::isfinite(wall_coverage_max_height_) ||
      wall_coverage_max_height_ <= wall_coverage_min_height_ ||
      !std::isfinite(wall_coverage_end_min_span_) ||
      wall_coverage_end_min_span_ <= wall_coverage_bin_size_ ||
      !std::isfinite(wall_coverage_end_center_half_width_) ||
      wall_coverage_end_center_half_width_ <= 0.0 ||
      !std::isfinite(wall_coverage_end_approach_distance_) ||
      wall_coverage_end_approach_distance_ <= 0.0 ||
      !std::isfinite(wall_coverage_end_standoff_distance_) ||
      wall_coverage_end_standoff_distance_ <= vehicle_radius_ ||
      wall_coverage_end_standoff_distance_ >= wall_coverage_min_depth_ ||
      !std::isfinite(min_observation_height_above_home_) ||
      !std::isfinite(max_observation_height_above_home_) ||
      min_observation_height_above_home_ < 0.0 ||
      max_observation_height_above_home_ <=
          min_observation_height_above_home_ ||
      side_wall_missing_confirm_frames_ < 1 ||
      front_obstacle_confirm_frames_ < 1 ||
      wall_coverage_max_gap_bins_ < 0 || three_wall_confirm_cycles_ < 1 ||
      !std::isfinite(map_closure_min_progress_) ||
      map_closure_min_progress_ < 0.0 ||
      !std::isfinite(map_closure_no_frontier_time_) ||
      map_closure_no_frontier_time_ <= 0.0 ||
      !std::isfinite(map_closure_stable_time_) ||
      map_closure_stable_time_ <= 0.0 ||
      map_closure_max_actionable_frontiers_ < 0 ||
      map_closure_growth_voxels_ < 1 || map_closure_confirm_cycles_ < 1 ||
      max_reachable_voxels_ < 1000 || max_frontier_candidates_ < 1 ||
      sync_queue_size_ < 2 || max_points_per_cloud_ < 1) {
    ROS_FATAL("Invalid exploration decider parameters");
    return false;
  }
  if (min_unknown_neighbors_ < 1) {
    min_unknown_neighbors_ = 1;
  }
  if (require_three_wall_completion_ && use_map_closure_completion_) {
    ROS_FATAL("Only one completion policy may be enabled");
    return false;
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
  // Capture the mission origin only after task one is actually enabled.  The
  // node normally starts while the pilot is still taking off; capturing the
  // first odometry sample there would make a later return target point at the
  // ground instead of at the hover where autonomy was accepted.
  if (!have_home_ && enabled_ &&
      (!require_mission_enable_edge_ || mission_start_pending_)) {
    home_pose_ = current_pose_;
    home_pose_.header.frame_id = world_frame_;
    mission_heading_yaw_ = poseYaw(home_pose_.pose);
    have_home_ = true;
    outbound_breadcrumbs_.clear();
    outbound_breadcrumbs_.push_back(home_pose_);
    mission_start_pending_ = false;
    ROS_INFO("Exploration home captured at %.2f %.2f %.2f, heading %.1f deg",
             home_pose_.pose.position.x, home_pose_.pose.position.y,
             home_pose_.pose.position.z,
             mission_heading_yaw_ * 180.0 / M_PI);
  }
  updateMap(*cloud, current_pose_);
  updateDirectionalEvidence(*cloud, current_pose_);
  if (enabled_ && exploration_started_ && !returning_home_ &&
      !mission_finished_ && !outbound_breadcrumbs_.empty() &&
      std::sqrt(squaredDistance(current_pose_.pose.position,
          outbound_breadcrumbs_.back().pose.position)) >=
          return_breadcrumb_spacing_) {
    outbound_breadcrumbs_.push_back(current_pose_);
  }
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

bool SuperExplorationDecider::isKnownFree(const VoxelKey& key) const {
  const auto it = voxels_.find(key);
  return it != voxels_.end() && it->second == kFree;
}

void SuperExplorationDecider::freeRayCallback(const Cloud::ConstPtr& cloud) {
  // This optional simulation-only stream carries *ends of traversed rays*,
  // not obstacle hits.  Real Fast-LIO2 deployments leave free_ray_topic unset.
  if (!enabled_ || !have_data_ || !have_home_ ||
      cloud->header.frame_id != world_frame_ ||
      std::abs((cloud->header.stamp - current_pose_.header.stamp).toSec()) >
          sync_slop_ + 0.15) {
    return;
  }
  pcl::PointCloud<pcl::PointXYZ> endpoints;
  try {
    pcl::fromROSMsg(*cloud, endpoints);
  } catch (const std::exception& error) {
    ROS_WARN_THROTTLE(2.0, "Cannot convert simulated free rays: %s",
                      error.what());
    return;
  }
  const geometry_msgs::Point& origin = current_pose_.pose.position;
  for (const auto& point : endpoints) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z)) continue;
    const double dx = point.x - origin.x;
    const double dy = point.y - origin.y;
    const double dz = point.z - origin.z;
    const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (distance < voxel_resolution_ ||
        distance > raycast_max_range_ + voxel_resolution_) continue;
    const int steps = static_cast<int>(std::ceil(distance / voxel_resolution_));
    for (int step = 1; step <= steps; ++step) {
      const double ratio = static_cast<double>(step) / steps;
      const VoxelKey key = positionToKey(origin.x + ratio * dx,
                                         origin.y + ratio * dy,
                                         origin.z + ratio * dz);
      if (isOccupied(key)) break;
      markFree(key);
    }
  }
}

SuperExplorationDecider::VoxelSet
SuperExplorationDecider::reachableFreeVoxels() const {
  VoxelSet reachable;
  if (!have_home_) {
    return reachable;
  }

  VoxelKey start = positionToKey(current_pose_.pose.position.x,
                                 current_pose_.pose.position.y,
                                 current_pose_.pose.position.z);
  if (!isKnownFree(start) || !isClearForVehicle(start)) {
    bool found = false;
    for (int radius = 1; radius <= 2 && !found; ++radius) {
      for (int dx = -radius; dx <= radius && !found; ++dx) {
        for (int dy = -radius; dy <= radius && !found; ++dy) {
          for (int dz = -radius; dz <= radius; ++dz) {
            const VoxelKey candidate{start.x + dx, start.y + dy, start.z + dz};
            if (isKnownFree(candidate) && isClearForVehicle(candidate)) {
              start = candidate;
              found = true;
              break;
            }
          }
        }
      }
    }
    if (!found) {
      return reachable;
    }
  }

  static const int kNeighbors[6][3] = {
      {1, 0, 0}, {-1, 0, 0}, {0, 1, 0},
      {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};
  std::queue<VoxelKey> queue;
  reachable.insert(start);
  queue.push(start);
  const double radius_squared =
      frontier_search_radius_ * frontier_search_radius_;
  const double c = std::cos(mission_heading_yaw_);
  const double s = std::sin(mission_heading_yaw_);

  while (!queue.empty() &&
         reachable.size() < static_cast<std::size_t>(max_reachable_voxels_)) {
    const VoxelKey current = queue.front();
    queue.pop();
    for (const auto& offset : kNeighbors) {
      const VoxelKey neighbor{current.x + offset[0], current.y + offset[1],
                              current.z + offset[2]};
      if (reachable.count(neighbor) > 0 || !isKnownFree(neighbor) ||
          !isClearForVehicle(neighbor)) {
        continue;
      }
      const geometry_msgs::Point point = keyToPoint(neighbor);
      if (squaredDistance(point, current_pose_.pose.position) >
          radius_squared) {
        continue;
      }
      const double home_dx = point.x - home_pose_.pose.position.x;
      const double home_dy = point.y - home_pose_.pose.position.y;
      const double lateral = -s * home_dx + c * home_dy;
      const double relative_height =
          point.z - home_pose_.pose.position.z;
      if (std::abs(lateral) > max_task_lateral_offset_ ||
          relative_height < min_observation_height_above_home_ -
                                voxel_resolution_ ||
          relative_height > max_observation_height_above_home_ +
                                voxel_resolution_) {
        continue;
      }
      reachable.insert(neighbor);
      queue.push(neighbor);
    }
  }
  if (reachable.size() >=
      static_cast<std::size_t>(max_reachable_voxels_)) {
    ROS_WARN_THROTTLE(5.0, "Reachable-free flood fill hit its configured cap");
  }
  return reachable;
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

void SuperExplorationDecider::updateDirectionalEvidence(
    const sensor_msgs::PointCloud2& cloud,
    const geometry_msgs::PoseStamped& pose) {
  pcl::PointCloud<pcl::PointXYZ> input;
  try {
    pcl::fromROSMsg(cloud, input);
  } catch (const std::exception& error) {
    ROS_WARN_THROTTLE(2.0,
                      "Cannot inspect directional Fast-LIO2 cloud: %s",
                      error.what());
    return;
  }

  // Directional mission semantics stay tied to the entry heading. SUPER is
  // free to rotate the vehicle while tracking, but that must not rotate the
  // definition of "deeper into the mine" or swap the three structural walls.
  const double yaw = have_home_ ? mission_heading_yaw_ : poseYaw(pose.pose);
  const double side_half_angle = side_wall_sector_deg_ * M_PI / 360.0;
  // Obstacle evidence needs a much narrower cone than frontier selection.
  // Reusing the broad candidate cone makes oblique returns from the side
  // walls look like a closed end wall.
  const double front_half_angle =
      front_obstacle_sector_deg_ * M_PI / 360.0;
  bool left_seen = false;
  bool right_seen = false;
  int front_point_count = 0;
  double front_min_lateral = std::numeric_limits<double>::infinity();
  double front_max_lateral = -std::numeric_limits<double>::infinity();
  double front_min_z = std::numeric_limits<double>::infinity();
  double front_max_z = -std::numeric_limits<double>::infinity();

  for (const auto& point : input) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z)) {
      continue;
    }
    const double dx = point.x - pose.pose.position.x;
    const double dy = point.y - pose.pose.position.y;
    const double dz = point.z - pose.pose.position.z;
    const double horizontal_range = std::hypot(dx, dy);
    if (horizontal_range < side_wall_min_range_ ||
        horizontal_range > side_wall_max_range_ ||
        dz < -directional_floor_exclusion_ ||
        dz > directional_vertical_tolerance_) {
      continue;
    }

    const double relative_angle = normalizeAngle(std::atan2(dy, dx) - yaw);
    if (std::abs(normalizeAngle(relative_angle - M_PI_2)) <=
        side_half_angle) {
      left_seen = true;
    }
    if (std::abs(normalizeAngle(relative_angle + M_PI_2)) <=
        side_half_angle) {
      right_seen = true;
    }
    if (horizontal_range <= front_obstacle_range_ &&
        std::abs(relative_angle) <= front_half_angle) {
      const double lateral = -std::sin(yaw) * dx + std::cos(yaw) * dy;
      ++front_point_count;
      front_min_lateral = std::min(front_min_lateral, lateral);
      front_max_lateral = std::max(front_max_lateral, lateral);
      front_min_z = std::min(front_min_z, static_cast<double>(point.z));
      front_max_z = std::max(front_max_z, static_cast<double>(point.z));
    }
  }

  const double front_lateral_span =
      front_point_count > 0 ? front_max_lateral - front_min_lateral : 0.0;
  const double front_vertical_span =
      front_point_count > 0 ? front_max_z - front_min_z : 0.0;
  const bool front_seen =
      front_point_count >= front_obstacle_min_points_ &&
      front_lateral_span >= front_obstacle_min_lateral_span_ &&
      front_vertical_span >= front_obstacle_min_vertical_span_;
  if (front_point_count > 0 && !front_seen) {
    ROS_DEBUG_THROTTLE(
        1.0,
        "Front returns rejected as a wall: points=%d lateral_span=%.2f "
        "vertical_span=%.2f",
        front_point_count, front_lateral_span, front_vertical_span);
  }

  left_wall_visible_ = left_seen;
  right_wall_visible_ = right_seen;
  front_obstacle_visible_ = front_seen;
  if (!left_seen && !right_seen) {
    side_wall_missing_streak_ = std::min(
        side_wall_missing_streak_ + 1, side_wall_missing_confirm_frames_);
  } else {
    side_wall_missing_streak_ = 0;
  }
  if (front_seen) {
    front_obstacle_streak_ = std::min(
        front_obstacle_streak_ + 1, front_obstacle_confirm_frames_);
  } else {
    front_obstacle_streak_ = 0;
  }
}

SuperExplorationDecider::ThreeWallCoverage
SuperExplorationDecider::evaluateThreeWallCoverage() const {
  ThreeWallCoverage result;
  if (!have_home_) {
    return result;
  }

  struct EndPlaneEvidence {
    std::set<int> lateral_bins;
    std::set<int> center_bins;
  };

  const double c = std::cos(mission_heading_yaw_);
  const double s = std::sin(mission_heading_yaw_);
  std::set<int> left_bins;
  std::set<int> right_bins;
  std::map<int, EndPlaneEvidence> end_planes;

  for (const auto& entry : voxels_) {
    if (entry.second != kOccupied) {
      continue;
    }
    const geometry_msgs::Point point = keyToPoint(entry.first);
    const double dx = point.x - home_pose_.pose.position.x;
    const double dy = point.y - home_pose_.pose.position.y;
    const double forward = c * dx + s * dy;
    const double lateral = -s * dx + c * dy;
    const double height = point.z - home_pose_.pose.position.z;
    if (forward < 0.0 || height < wall_coverage_min_height_ ||
        height > wall_coverage_max_height_) {
      continue;
    }

    const int forward_bin =
        static_cast<int>(std::floor(forward / wall_coverage_bin_size_));
    const double lateral_abs = std::abs(lateral);
    if (lateral >= wall_coverage_side_min_distance_ &&
        lateral <= wall_coverage_side_max_distance_) {
      left_bins.insert(forward_bin);
    } else if (lateral <= -wall_coverage_side_min_distance_ &&
               lateral >= -wall_coverage_side_max_distance_) {
      right_bins.insert(forward_bin);
    }

    if (forward < wall_coverage_end_min_depth_ ||
        lateral_abs > wall_coverage_side_max_distance_) {
      continue;
    }
    EndPlaneEvidence& plane = end_planes[forward_bin];
    const int lateral_bin =
        static_cast<int>(std::floor(lateral / wall_coverage_bin_size_));
    plane.lateral_bins.insert(lateral_bin);
    if (lateral_abs <= wall_coverage_end_center_half_width_) {
      plane.center_bins.insert(lateral_bin);
    }
  }

  int end_bin = -1;
  for (const auto& candidate : end_planes) {
    if (candidate.second.center_bins.empty() ||
        candidate.second.lateral_bins.empty()) {
      continue;
    }

    // A genuine end wall must form one center-connected lateral component.
    // Merely taking the outermost returns is unsafe: two distant side-wall
    // points plus a short platform edge can otherwise look like a full-width
    // end wall.  Reuse the configured gap allowance to tolerate sparse LiDAR
    // sampling without bridging large unobserved regions.
    int component_first = 0;
    int component_last = 0;
    int previous_bin = 0;
    bool component_has_center = false;
    bool first_bin = true;
    double center_connected_span = 0.0;
    const auto finish_component = [&]() {
      if (!first_bin && component_has_center) {
        center_connected_span = std::max(
            center_connected_span,
            (component_last - component_first + 1) *
                wall_coverage_bin_size_);
      }
    };
    for (const int lateral_bin : candidate.second.lateral_bins) {
      if (first_bin ||
          lateral_bin - previous_bin > wall_coverage_end_max_gap_bins_ + 1) {
        finish_component();
        component_first = lateral_bin;
        component_has_center = false;
        first_bin = false;
      }
      component_last = lateral_bin;
      component_has_center =
          component_has_center ||
          candidate.second.center_bins.count(lateral_bin) > 0;
      previous_bin = lateral_bin;
    }
    finish_component();
    if (center_connected_span < wall_coverage_end_min_span_) {
      continue;
    }
    if (candidate.first > end_bin) {
      end_bin = candidate.first;
      result.end_lateral_span = center_connected_span;
    }
  }

  const double current_dx =
      current_pose_.pose.position.x - home_pose_.pose.position.x;
  const double current_dy =
      current_pose_.pose.position.y - home_pose_.pose.position.y;
  result.vehicle_progress = c * current_dx + s * current_dy;
  if (end_bin < 1) {
    return result;
  }

  result.end_wall_found = true;
  result.end_depth = (static_cast<double>(end_bin) + 0.5) *
                     wall_coverage_bin_size_;
  result.expected_side_bins = end_bin;

  int left_count = 0;
  int right_count = 0;
  int left_gap = 0;
  int right_gap = 0;
  int left_max_gap = 0;
  int right_max_gap = 0;
  for (int bin = 0; bin < end_bin; ++bin) {
    if (left_bins.count(bin) > 0) {
      ++left_count;
      left_gap = 0;
    } else {
      left_max_gap = std::max(left_max_gap, ++left_gap);
    }
    if (right_bins.count(bin) > 0) {
      ++right_count;
      right_gap = 0;
    } else {
      right_max_gap = std::max(right_max_gap, ++right_gap);
    }
  }
  result.left_ratio = static_cast<double>(left_count) / end_bin;
  result.right_ratio = static_cast<double>(right_count) / end_bin;
  result.left_max_gap_bins = left_max_gap;
  result.right_max_gap_bins = right_max_gap;

  const bool approached_end =
      result.end_depth - result.vehicle_progress <=
      wall_coverage_end_approach_distance_;
  result.complete = result.end_depth >= wall_coverage_min_depth_ &&
                    approached_end &&
                    result.left_ratio >= wall_coverage_min_ratio_ &&
                    result.right_ratio >= wall_coverage_min_ratio_ &&
                    result.left_max_gap_bins <= wall_coverage_max_gap_bins_ &&
                    result.right_max_gap_bins <= wall_coverage_max_gap_bins_;
  return result;
}

SuperExplorationDecider::MapClosureStatus
SuperExplorationDecider::evaluateMapClosure(
    const std::vector<FrontierCandidate>& candidates,
    bool end_wall_seen) {
  MapClosureStatus result;
  if (!have_home_) {
    return result;
  }

  const ros::Time now = ros::Time::now();
  const double c = std::cos(mission_heading_yaw_);
  const double s = std::sin(mission_heading_yaw_);
  const double dx = current_pose_.pose.position.x -
                    home_pose_.pose.position.x;
  const double dy = current_pose_.pose.position.y -
                    home_pose_.pose.position.y;
  result.vehicle_progress = c * dx + s * dy;
  result.front_boundary_seen =
      front_obstacle_streak_ >= front_obstacle_confirm_frames_ ||
      end_wall_seen;

  for (const auto& entry : voxels_) {
    if (entry.second == kOccupied) {
      ++result.occupied_voxels;
    }
  }

  for (const auto& candidate : candidates) {
    if (isForwardCandidate(candidate) &&
        candidateMissionProgress(candidate) >=
            result.vehicle_progress + 0.5 * min_goal_distance_ &&
        std::abs(candidateMissionLateral(candidate)) <=
            max_task_lateral_offset_) {
      ++result.actionable_frontiers;
    }
  }

  if (last_actionable_frontier_time_.isZero()) {
    last_actionable_frontier_time_ = now;
  }
  if (result.actionable_frontiers >
      static_cast<std::size_t>(map_closure_max_actionable_frontiers_)) {
    last_actionable_frontier_time_ = now;
  }
  result.no_frontier_duration =
      (now - last_actionable_frontier_time_).toSec();

  if (last_significant_map_growth_time_.isZero() ||
      map_growth_reference_count_ == 0) {
    last_significant_map_growth_time_ = now;
    map_growth_reference_count_ = result.occupied_voxels;
  } else if (result.occupied_voxels < map_growth_reference_count_ ||
             result.occupied_voxels >=
                 map_growth_reference_count_ +
                     static_cast<std::size_t>(map_closure_growth_voxels_)) {
    last_significant_map_growth_time_ = now;
    map_growth_reference_count_ = result.occupied_voxels;
  }
  result.stable_map_duration =
      (now - last_significant_map_growth_time_).toSec();

  const bool frontier_closed =
      result.actionable_frontiers <=
          static_cast<std::size_t>(map_closure_max_actionable_frontiers_) &&
      result.no_frontier_duration >= map_closure_no_frontier_time_;
  const bool front_boundary_ok =
      !map_closure_require_front_boundary_ || result.front_boundary_seen;
  result.complete = result.vehicle_progress >= map_closure_min_progress_ &&
                    front_boundary_ok && frontier_closed &&
                    result.stable_map_duration >= map_closure_stable_time_;
  return result;
}

void SuperExplorationDecider::publishCoverageStatus(
    const ThreeWallCoverage& coverage, const MapClosureStatus& closure) {
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(2)
         << "map_closure=" << (closure.complete ? "ready" : "incomplete")
         << " front_closed="
         << (closure.front_boundary_seen ? "true" : "false")
         << " actionable=" << closure.actionable_frontiers
         << " no_frontier_s=" << closure.no_frontier_duration
         << " stable_s=" << closure.stable_map_duration
         << " occupied=" << closure.occupied_voxels
         << " closure_confirm=" << map_closure_complete_streak_ << "/"
         << map_closure_confirm_cycles_
         << " three_wall=" << (coverage.complete ? "ready" : "diagnostic")
         << " end_seen=" << (coverage.end_wall_found ? "true" : "false")
         << " end_depth=" << coverage.end_depth
         << " progress=" << coverage.vehicle_progress
         << " left=" << coverage.left_ratio
         << " right=" << coverage.right_ratio
         << " end_span=" << coverage.end_lateral_span
         << " gaps=" << coverage.left_max_gap_bins << "/"
         << coverage.right_max_gap_bins
         << " confirm=" << three_wall_complete_streak_ << "/"
         << three_wall_confirm_cycles_;
  std_msgs::String message;
  message.data = stream.str();
  coverage_status_publisher_.publish(message);
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
SuperExplorationDecider::findFrontiers(VoxelSet& reachable) const {
  std::vector<FrontierCandidate> candidates;
  reachable = reachableFreeVoxels();
  if (reachable.empty()) {
    return candidates;
  }
  const double max_distance_squared =
      frontier_search_radius_ * frontier_search_radius_;
  const double min_distance_squared = min_goal_distance_ * min_goal_distance_;

  for (const auto& entry : voxels_) {
    if (entry.second != kFree) {
      continue;
    }
    if (reachable.count(entry.first) == 0) {
      continue;
    }
    const geometry_msgs::Point point = keyToPoint(entry.first);
    if (have_home_) {
      const double home_dx = point.x - home_pose_.pose.position.x;
      const double home_dy = point.y - home_pose_.pose.position.y;
      const double max_home_distance_squared =
          max_exploration_radius_from_home_ *
          max_exploration_radius_from_home_;
      if (home_dx * home_dx + home_dy * home_dy >
          max_home_distance_squared) {
        continue;
      }
      const double min_goal_z = home_pose_.pose.position.z +
                                min_observation_height_above_home_;
      const double max_goal_z = home_pose_.pose.position.z +
                                max_observation_height_above_home_;
      if (point.z < min_goal_z || point.z > max_goal_z) {
        continue;
      }
      const double mission_lateral =
          -std::sin(mission_heading_yaw_) * home_dx +
          std::cos(mission_heading_yaw_) * home_dy;
      if (std::abs(mission_lateral) > max_task_lateral_offset_) {
        continue;
      }
    }
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
  std::vector<FrontierCandidate> representatives;
  const double spacing_squared = candidate_spacing_ * candidate_spacing_;
  for (const auto& candidate : candidates) {
    bool separated = true;
    for (const auto& accepted : representatives) {
      if (squaredDistance(candidate.goal.pose.position,
                          accepted.goal.pose.position) < spacing_squared) {
        separated = false;
        break;
      }
    }
    if (separated) {
      representatives.push_back(candidate);
      if (representatives.size() >=
          static_cast<std::size_t>(max_frontier_candidates_)) {
        break;
      }
    }
  }
  return representatives;
}

bool SuperExplorationDecider::isForwardCandidate(
    const FrontierCandidate& candidate) const {
  const double dx = candidate.goal.pose.position.x -
                    current_pose_.pose.position.x;
  const double dy = candidate.goal.pose.position.y -
                    current_pose_.pose.position.y;
  if (std::hypot(dx, dy) < min_goal_distance_) {
    return false;
  }
  const double relative_angle = normalizeAngle(
      std::atan2(dy, dx) - mission_heading_yaw_);
  return std::abs(relative_angle) <= forward_sector_deg_ * M_PI / 360.0 &&
         std::cos(relative_angle) > 0.0;
}

double SuperExplorationDecider::candidateHeadingAlignment(
    const FrontierCandidate& candidate) const {
  const double dx = candidate.goal.pose.position.x -
                    current_pose_.pose.position.x;
  const double dy = candidate.goal.pose.position.y -
                    current_pose_.pose.position.y;
  const double distance = std::hypot(dx, dy);
  if (distance < 1e-6) {
    return -1.0;
  }
  const double relative_angle = normalizeAngle(
      std::atan2(dy, dx) - mission_heading_yaw_);
  return std::cos(relative_angle);
}

double SuperExplorationDecider::candidateMissionProgress(
    const FrontierCandidate& candidate) const {
  const double dx = candidate.goal.pose.position.x - home_pose_.pose.position.x;
  const double dy = candidate.goal.pose.position.y - home_pose_.pose.position.y;
  return std::cos(mission_heading_yaw_) * dx +
         std::sin(mission_heading_yaw_) * dy;
}

double SuperExplorationDecider::candidateMissionLateral(
    const FrontierCandidate& candidate) const {
  const double dx = candidate.goal.pose.position.x - home_pose_.pose.position.x;
  const double dy = candidate.goal.pose.position.y - home_pose_.pose.position.y;
  return -std::sin(mission_heading_yaw_) * dx +
         std::cos(mission_heading_yaw_) * dy;
}

const SuperExplorationDecider::FrontierCandidate*
SuperExplorationDecider::selectForwardCandidate(
    const std::vector<FrontierCandidate>& candidates) const {
  const FrontierCandidate* best = nullptr;
  double best_score = -std::numeric_limits<double>::infinity();
  const double current_dx =
      current_pose_.pose.position.x - home_pose_.pose.position.x;
  const double current_dy =
      current_pose_.pose.position.y - home_pose_.pose.position.y;
  const double current_progress = std::cos(mission_heading_yaw_) * current_dx +
                                  std::sin(mission_heading_yaw_) * current_dy;
  const double preferred_height =
      home_pose_.pose.position.z +
      0.5 * (min_observation_height_above_home_ +
             max_observation_height_above_home_);
  const bool side_structure_visible =
      left_wall_visible_ || right_wall_visible_;
  for (const auto& candidate : candidates) {
    if (!isForwardCandidate(candidate)) {
      continue;
    }
    const double progress = candidateMissionProgress(candidate);
    const double progress_ahead = progress - current_progress;
    const double lateral = std::abs(candidateMissionLateral(candidate));
    if (progress_ahead < min_goal_distance_ ||
        (side_structure_visible && lateral > forward_corridor_half_width_)) {
      continue;
    }
    const double height_error =
        std::abs(candidate.goal.pose.position.z - preferred_height);
    const double score =
        forward_progress_weight_ * progress_ahead -
        forward_lateral_penalty_ * lateral -
        forward_height_penalty_ * height_error +
        heading_priority_weight_ * candidateHeadingAlignment(candidate) +
        0.05 * static_cast<double>(candidate.unknown_neighbors);
    if (score > best_score) {
      best_score = score;
      best = &candidate;
    }
  }
  return best;
}

void SuperExplorationDecider::updateExplorationPhase(
    const std::vector<FrontierCandidate>& candidates) {
  const FrontierCandidate* forward_candidate =
      selectForwardCandidate(candidates);
  const bool side_walls_unavailable =
      side_wall_missing_streak_ >= side_wall_missing_confirm_frames_;

  const bool front_obstacle_confirmed =
      front_obstacle_streak_ >= front_obstacle_confirm_frames_;
  if (front_obstacle_confirmed) {
    if (exploration_phase_ != ExplorationPhase::kFrontierFallback) {
      ROS_INFO("Exploration phase: forward obstacle confirmed; enabling "
               "frontier fallback");
    }
    exploration_phase_ = ExplorationPhase::kFrontierFallback;
  } else if (forward_candidate != nullptr) {
    if (exploration_phase_ != ExplorationPhase::kForwardPriority) {
      ROS_INFO("Exploration phase: forward candidate available; restoring "
               "heading priority");
    }
    exploration_phase_ = ExplorationPhase::kForwardPriority;
  } else if (side_walls_unavailable) {
    if (exploration_phase_ != ExplorationPhase::kFrontierFallback) {
      ROS_INFO("Exploration phase: both side-wall observations missing for "
               "configured confirmation window; enabling frontier fallback");
    }
    exploration_phase_ = ExplorationPhase::kFrontierFallback;
  }
}

const char* SuperExplorationDecider::explorationPhaseName() const {
  switch (exploration_phase_) {
    case ExplorationPhase::kForwardPriority:
      return "FORWARD_PRIORITY";
    case ExplorationPhase::kFrontierFallback:
      return "FRONTIER_FALLBACK";
  }
  return "UNKNOWN";
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
    const geometry_msgs::PoseStamped& goal, const std::string& reason,
    bool continuous_handover) {
  // SET while a goal is active requests a trajectory-continuous retarget.
  // An explicit CANCEL retains the old hard-stop semantics for unsafe or
  // non-continuous transitions (including exploration -> return).
  if (!continuous_handover) cancelActiveGoal("replace_goal");
  active_goal_is_end_approach_ = false;
  current_goal_ = goal;
  current_goal_.header.frame_id = world_frame_;
  current_goal_.header.stamp = ros::Time::now();
  super_planner::GoalCommand command;
  command.header = current_goal_.header;
  command.command = super_planner::GoalCommand::SET_GOAL;
  command.goal_id = next_goal_id_++;
  command.goal = current_goal_.pose;
  command.reason = reason;
  goal_publisher_.publish(command);
  active_goal_id_ = command.goal_id;
  goal_sent_time_ = current_goal_.header.stamp;
  active_goal_initial_distance_ = std::sqrt(squaredDistance(
      current_pose_.pose.position, current_goal_.pose.position));
  have_active_goal_ = true;
  exploration_started_ = true;
  ROS_INFO("New SUPER goal (%s): %.2f %.2f %.2f", reason.c_str(),
           current_goal_.pose.position.x, current_goal_.pose.position.y,
           current_goal_.pose.position.z);
}

void SuperExplorationDecider::cancelActiveGoal(const std::string& reason,
                                                bool unconditional) {
  if (!unconditional && active_goal_id_ == 0) {
    return;
  }
  super_planner::GoalCommand command;
  command.header.stamp = ros::Time::now();
  command.header.frame_id = world_frame_;
  command.command = super_planner::GoalCommand::CANCEL_GOAL;
  command.goal_id = unconditional ? 0 : active_goal_id_;
  command.reason = reason;
  goal_publisher_.publish(command);
  active_goal_id_ = 0;
  have_active_goal_ = false;
}

bool SuperExplorationDecider::publishEndApproachGoal(
    const ThreeWallCoverage& coverage) {
  const bool side_walls_continuous =
      coverage.left_ratio >= wall_coverage_min_ratio_ &&
      coverage.right_ratio >= wall_coverage_min_ratio_ &&
      coverage.left_max_gap_bins <= wall_coverage_max_gap_bins_ &&
      coverage.right_max_gap_bins <= wall_coverage_max_gap_bins_;
  if (!coverage.end_wall_found || !side_walls_continuous) {
    return false;
  }

  const double target_forward =
      coverage.end_depth - wall_coverage_end_standoff_distance_;
  if (target_forward - coverage.vehicle_progress < min_goal_distance_) {
    return false;
  }
  const double c = std::cos(mission_heading_yaw_);
  const double s = std::sin(mission_heading_yaw_);
  geometry_msgs::PoseStamped goal;
  goal.header.frame_id = world_frame_;
  goal.pose.position.x =
      home_pose_.pose.position.x + c * target_forward;
  goal.pose.position.y =
      home_pose_.pose.position.y + s * target_forward;
  goal.pose.position.z = home_pose_.pose.position.z +
      0.5 * (min_observation_height_above_home_ +
             max_observation_height_above_home_);
  goal.pose.orientation = yawQuaternion(mission_heading_yaw_);
  if (!isClearForVehicle(positionToKey(goal.pose.position.x,
                                       goal.pose.position.y,
                                       goal.pose.position.z))) {
    ROS_WARN_THROTTLE(2.0,
                      "Three-wall end approach target is not collision-free");
    return false;
  }
  publishGoal(goal, "three_wall_end_approach");
  active_goal_is_end_approach_ = true;
  return true;
}

bool SuperExplorationDecider::publishForwardLookaheadGoal(
    const VoxelSet& reachable) {
  if (!have_home_ || exploration_phase_ != ExplorationPhase::kForwardPriority ||
      front_obstacle_streak_ >= front_obstacle_confirm_frames_) {
    return false;
  }

  const double c = std::cos(mission_heading_yaw_);
  const double s = std::sin(mission_heading_yaw_);
  const double current_dx =
      current_pose_.pose.position.x - home_pose_.pose.position.x;
  const double current_dy =
      current_pose_.pose.position.y - home_pose_.pose.position.y;
  const double current_progress = c * current_dx + s * current_dy;
  const double remaining_radius =
      max_exploration_radius_from_home_ - current_progress;
  const double maximum_lookahead =
      std::min(forward_lookahead_distance_, remaining_radius);
  if (maximum_lookahead < min_goal_distance_) {
    return false;
  }

  std::vector<double> lateral_offsets{0.0};
  for (double offset = voxel_resolution_;
       offset <= forward_corridor_half_width_ + 1e-6;
       offset += voxel_resolution_) {
    lateral_offsets.push_back(offset);
    lateral_offsets.push_back(-offset);
  }
  std::vector<double> height_offsets{0.0};
  for (double offset = voxel_resolution_;
       cruise_height_above_home_ - offset >=
               min_observation_height_above_home_ - 1e-6 ||
           cruise_height_above_home_ + offset <=
               max_observation_height_above_home_ + 1e-6;
       offset += voxel_resolution_) {
    if (cruise_height_above_home_ - offset >=
        min_observation_height_above_home_ - 1e-6) {
      height_offsets.push_back(-offset);
    }
    if (cruise_height_above_home_ + offset <=
        max_observation_height_above_home_ + 1e-6) {
      height_offsets.push_back(offset);
    }
  }

  // Search all centerline distances before considering any side offset. This
  // avoids turning sideways merely because the farthest centerline voxel has
  // one noisy/occupied endpoint; a slightly shorter straight target is the
  // smoother and more faithful task-level choice.
  for (const double lateral : lateral_offsets) {
    for (double lookahead = maximum_lookahead;
         lookahead >= min_goal_distance_ - 1e-6;
         lookahead -= forward_lookahead_step_) {
      const double target_progress = current_progress + lookahead;
      for (const double height_offset : height_offsets) {
        geometry_msgs::Point target;
        target.x = home_pose_.pose.position.x + c * target_progress -
                   s * lateral;
        target.y = home_pose_.pose.position.y + s * target_progress +
                   c * lateral;
        target.z = home_pose_.pose.position.z + cruise_height_above_home_ +
                   height_offset;
        const VoxelKey target_key = positionToKey(target.x, target.y, target.z);
        // A task-level goal must be proven free and connected by a clear,
        // observed segment. Unknown is not free: allowing a centerline goal
        // merely because it is not occupied can place the goal behind a wall.
        if (reachable.count(target_key) == 0 ||
            !isClearForVehicle(target_key)) {
          continue;
        }
        geometry_msgs::PoseStamped goal;
        goal.header.frame_id = world_frame_;
        goal.pose.position = target;
        goal.pose.orientation = yawQuaternion(mission_heading_yaw_);
        publishGoal(goal, "forward_lookahead", true);
        return true;
      }
    }
  }
  return false;
}

bool SuperExplorationDecider::tryForwardHandover(const VoxelSet& reachable) {
  if (!have_active_goal_) {
    return false;
  }
  const geometry_msgs::Point previous_goal = current_goal_.pose.position;
  // publishForwardLookaheadGoal only mutates goal state after it has found a
  // known-free, reachable successor. A failed search must leave the current
  // trajectory active instead of creating an unbounded no-goal interval.
  if (!publishForwardLookaheadGoal(reachable)) {
    return false;
  }
  covered_goals_.push_back(previous_goal);
  ++reached_goal_count_;
  return true;
}

bool SuperExplorationDecider::selectAndPublishFrontier(
    const std::vector<FrontierCandidate>& candidates) {
  if (candidates.empty()) {
    return false;
  }
  const FrontierCandidate* selected = nullptr;
  std::string reason;
  if (exploration_phase_ == ExplorationPhase::kForwardPriority) {
    selected = selectForwardCandidate(candidates);
    if (selected == nullptr) {
      return false;
    }
    reason = "heading_priority";
  } else {
    // A confirmed front obstacle must not be crossed just because a frontier
    // candidate happens to have a high information score. Prefer a lateral or
    // rear candidate in fallback mode; if none exists, wait for more map data.
    const bool front_obstacle_confirmed =
        front_obstacle_streak_ >= front_obstacle_confirm_frames_;
    if (front_obstacle_confirmed) {
      for (const auto& candidate : candidates) {
        if (!isForwardCandidate(candidate)) {
          if (selected == nullptr || candidate.score > selected->score) {
            selected = &candidate;
          }
        }
      }
      if (selected == nullptr) {
        return false;
      }
      reason = "front_obstacle_fallback";
    } else {
      selected = &*std::max_element(
          candidates.begin(), candidates.end(),
          [this](const FrontierCandidate& left,
                 const FrontierCandidate& right) {
            const double left_score =
                left.score + fallback_heading_weight_ *
                                 candidateHeadingAlignment(left);
            const double right_score =
                right.score + fallback_heading_weight_ *
                                  candidateHeadingAlignment(right);
            return left_score < right_score;
          });
      reason = "frontier_fallback";
    }
  }

  last_frontier_time_ = ros::Time::now();
  geometry_msgs::PoseStamped selected_goal = selected->goal;
  if (exploration_phase_ == ExplorationPhase::kForwardPriority) {
    // Keep the commanded yaw aligned with the latched entry mission heading. SUPER may
    // apply its own yaw policy, but this preserves the task-level intent.
    selected_goal.pose.orientation = yawQuaternion(mission_heading_yaw_);
  }
  publishGoal(selected_goal, reason);
  return true;
}

void SuperExplorationDecider::beginReturnHome(const std::string& reason) {
  if (!have_home_ || returning_home_ || mission_finished_) {
    return;
  }
  returning_home_ = true;
  cancelActiveGoal("return_home");
  return_waypoints_.clear();
  return_waypoint_index_ = 0;
  for (auto it = outbound_breadcrumbs_.rbegin();
       it != outbound_breadcrumbs_.rend(); ++it) {
    if (std::sqrt(squaredDistance(it->pose.position,
                                  current_pose_.pose.position)) >
        goal_reached_distance_ &&
        squaredDistance(it->pose.position, home_pose_.pose.position) >
            goal_reached_distance_ * goal_reached_distance_) {
      return_waypoints_.push_back(*it);
    }
  }
  geometry_msgs::PoseStamped home_target = home_pose_;
  home_target.pose.position.z += return_home_height_offset_;
  return_waypoints_.push_back(home_target);
  publishGoal(return_waypoints_.front(), reason);
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
    cancelActiveGoal("wait_data");
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
    if (active_goal_id_ == 0) {
      publishGoal(current_goal_, "resume_return_home_after_data_gap");
    }
    if (squaredDistance(current_pose_.pose.position,
                       current_goal_.pose.position) <=
        goal_reached_distance_ * goal_reached_distance_ &&
        return_waypoint_index_ + 1 < return_waypoints_.size()) {
      ++return_waypoint_index_;
      publishGoal(return_waypoints_[return_waypoint_index_],
                  "return_breadcrumb", true);
      publishStatus("RETURNING", "following observed outbound route");
      return;
    }
    if (squaredDistance(current_pose_.pose.position,
                       current_goal_.pose.position) <=
        goal_reached_distance_ * goal_reached_distance_ &&
        return_waypoint_index_ + 1 == return_waypoints_.size()) {
      returning_home_ = false;
      mission_finished_ = true;
      cancelActiveGoal("home_reached");
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

  const ThreeWallCoverage coverage = evaluateThreeWallCoverage();
  VoxelSet reachable;
  const auto candidates = findFrontiers(reachable);
  // Diagnose the observation-to-reachability boundary at a low rate.  The
  // forward planner must not silently wait when observed free space exists
  // ahead but its connected component is lost by the flood fill.
  static ros::Time last_reachability_diagnostic;
  if (have_home_ && !free_ray_topic_.empty() &&
      (last_reachability_diagnostic.isZero() ||
       (ros::Time::now() - last_reachability_diagnostic).toSec() >= 5.0)) {
    last_reachability_diagnostic = ros::Time::now();
    const double c = std::cos(mission_heading_yaw_);
    const double s = std::sin(mission_heading_yaw_);
    double free_ahead = -1e9;
    double reachable_ahead = -1e9;
    std::size_t free_corridor = 0;
    for (const auto& entry : voxels_) {
      if (entry.second != kFree) continue;
      const geometry_msgs::Point point = keyToPoint(entry.first);
      const double dx = point.x - home_pose_.pose.position.x;
      const double dy = point.y - home_pose_.pose.position.y;
      const double lateral = -s * dx + c * dy;
      if (std::abs(lateral) > max_task_lateral_offset_ ||
          std::abs(point.z - home_pose_.pose.position.z -
                   cruise_height_above_home_) > voxel_resolution_) continue;
      ++free_corridor;
      const double progress = c * dx + s * dy;
      free_ahead = std::max(free_ahead, progress);
      if (reachable.count(entry.first) > 0 &&
          isClearForVehicle(entry.first)) {
        reachable_ahead = std::max(reachable_ahead, progress);
      }
    }
    const VoxelKey current_key = positionToKey(
        current_pose_.pose.position.x, current_pose_.pose.position.y,
        current_pose_.pose.position.z);
    ROS_INFO(
        "Task1 map reachability: voxels=%zu corridor_free=%zu reachable=%zu "
        "free_forward=%.2f reachable_forward=%.2f current_free=%d "
        "current_clear=%d candidates=%zu",
        voxels_.size(), free_corridor, reachable.size(), free_ahead,
        reachable_ahead, isKnownFree(current_key),
        isClearForVehicle(current_key), candidates.size());
  }
  const MapClosureStatus closure =
      evaluateMapClosure(candidates, coverage.end_wall_found);
  const bool completion_prerequisites =
      exploration_started_ &&
      reached_goal_count_ >= min_goals_before_complete_ &&
      !first_data_time_.isZero() &&
      (ros::Time::now() - first_data_time_).toSec() >= min_data_duration_;
  if (require_three_wall_completion_ && coverage.complete &&
      completion_prerequisites) {
    three_wall_complete_streak_ = std::min(
        three_wall_complete_streak_ + 1, three_wall_confirm_cycles_);
  } else {
    three_wall_complete_streak_ = 0;
  }
  if (use_map_closure_completion_ && closure.complete &&
      completion_prerequisites) {
    map_closure_complete_streak_ = std::min(
        map_closure_complete_streak_ + 1, map_closure_confirm_cycles_);
  } else {
    map_closure_complete_streak_ = 0;
  }
  publishCoverageStatus(coverage, closure);
  if (use_map_closure_completion_ &&
      map_closure_complete_streak_ >= map_closure_confirm_cycles_) {
    std_msgs::Bool message;
    message.data = true;
    model_complete_publisher_.publish(message);
    publishStatus("MODEL_COMPLETE",
                  "point-cloud boundary closed and map converged");
    beginReturnHome("map closure confirmed");
    return;
  }
  if (require_three_wall_completion_ &&
      three_wall_complete_streak_ >= three_wall_confirm_cycles_) {
    std_msgs::Bool message;
    message.data = true;
    model_complete_publisher_.publish(message);
    publishStatus("MODEL_COMPLETE",
                  "continuous left, right, and end walls confirmed");
    beginReturnHome("three-wall model complete");
    return;
  }

  publishVisualization(candidates);
  if (!candidates.empty()) {
    last_frontier_time_ = ros::Time::now();
  }
  updateExplorationPhase(candidates);

  if (have_active_goal_) {
    const double distance = std::sqrt(squaredDistance(
        current_pose_.pose.position, current_goal_.pose.position));
    const double goal_age = (ros::Time::now() - goal_sent_time_).toSec();
    const bool front_obstacle_confirmed =
        front_obstacle_streak_ >= front_obstacle_confirm_frames_;
    const bool forward_goal_handover =
        exploration_phase_ == ExplorationPhase::kForwardPriority &&
        !front_obstacle_confirmed && !active_goal_is_end_approach_ &&
        distance <= forward_goal_handover_distance_ &&
        distance <= active_goal_initial_distance_ - 0.5;
    const bool preempt_blocked_forward_goal =
        exploration_phase_ == ExplorationPhase::kFrontierFallback &&
        front_obstacle_confirmed && !active_goal_is_end_approach_ &&
        isForwardCandidate(
            FrontierCandidate{VoxelKey{}, current_goal_, 0.0, 0});
    if (forward_goal_handover && tryForwardHandover(reachable)) {
      publishStatus("GOAL_HANDOVER", "reachable successor accepted before old goal cancellation");
      return;
    }
    // A one-metre handover radius is not an arrival tolerance. If the next
    // target is still unknown, keep following the current forward target
    // until within half a decision voxel, allowing fresh lidar viewpoints.
    const bool forward_target =
        exploration_phase_ == ExplorationPhase::kForwardPriority &&
        !front_obstacle_confirmed && !active_goal_is_end_approach_;
    const double arrival_distance = forward_target
        ? std::min(goal_reached_distance_, 0.5 * voxel_resolution_)
        : goal_reached_distance_;
    if (distance <= arrival_distance) {
      covered_goals_.push_back(current_goal_.pose.position);
      ++reached_goal_count_;
      cancelActiveGoal("goal_reached");
      publishStatus("GOAL_REACHED", "selecting next frontier");
    } else if (preempt_blocked_forward_goal) {
      covered_goals_.push_back(current_goal_.pose.position);
      cancelActiveGoal("front_wall_preempt");
      publishStatus("GOAL_PREEMPTED",
                    "confirmed front wall blocks the forward goal");
    } else if (goal_age > goal_timeout_) {
      covered_goals_.push_back(current_goal_.pose.position);
      cancelActiveGoal("goal_timeout");
      publishStatus("GOAL_TIMEOUT", "discarding unreachable candidate");
    } else {
      publishStatus("EXPLORING",
                    "active frontier goal published; awaiting odometry progress");
      return;
    }
  }

  if (!have_active_goal_ && require_three_wall_completion_ &&
      publishEndApproachGoal(coverage)) {
    publishStatus("APPROACHING_END_WALL",
                  "three-wall map seen; moving to configured stand-off");
    return;
  }

  if (!have_active_goal_ && use_map_closure_completion_ &&
      closure.front_boundary_seen) {
    publishStatus("WAIT_MAP_CLOSURE",
                  "front boundary closed; waiting for frontier and map convergence");
    return;
  }

  if (!have_active_goal_ && publishForwardLookaheadGoal(reachable)) {
    publishStatus("EXPLORING",
                  "continuous forward look-ahead goal sent to SUPER");
    return;
  }

  if (!have_active_goal_ && selectAndPublishFrontier(candidates)) {
    publishStatus("EXPLORING", "new frontier goal sent to SUPER");
    return;
  }

  if (!require_three_wall_completion_ && !use_map_closure_completion_ &&
      completion_prerequisites &&
      !last_frontier_time_.isZero() &&
      (ros::Time::now() - last_frontier_time_).toSec() >=
          no_frontier_timeout_) {
    publishStatus("MODEL_COMPLETE", "no new frontier for configured timeout");
    beginReturnHome("exploration complete");
  } else if (require_three_wall_completion_) {
    publishStatus("WAIT_MODEL_COVERAGE",
                  "three continuous walls are not confirmed yet");
  } else if (use_map_closure_completion_) {
    publishStatus("WAIT_MAP_CLOSURE",
                  "map boundary or convergence conditions are incomplete");
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

void SuperExplorationDecider::missionEnableCallback(
    const std_msgs::Bool::ConstPtr& message) {
  if (enabled_ == message->data) {
    return;
  }
  enabled_ = message->data;
  if (!enabled_) {
    cancelActiveGoal("mission_disabled", true);
    publishStatus("DISABLED", "task scheduler selected another task");
    ROS_INFO("SUPER exploration paused by mission scheduler");
  } else {
    // A new enable transition starts a fresh test mission.  This clears any
    // map/home left from the previous run and arms home capture for the first
    // synchronized Fast-LIO2 cloud+odometry sample after CH7 is raised.
    cancelActiveGoal("mission_reenabled", true);
    clearMissionState();
    mission_start_pending_ = true;
    publishStatus("WAIT_DATA", "task scheduler selected goaf exploration");
    ROS_INFO("SUPER exploration enabled by mission scheduler; waiting for "
             "the first synchronized sample to capture home");
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
  outbound_breadcrumbs_.clear();
  return_waypoints_.clear();
  return_waypoint_index_ = 0;
  current_goal_ = geometry_msgs::PoseStamped();
  active_goal_initial_distance_ = 0.0;
  have_data_ = false;
  have_home_ = false;
  have_active_goal_ = false;
  active_goal_is_end_approach_ = false;
  exploration_started_ = false;
  returning_home_ = false;
  mission_finished_ = false;
  return_requested_ = false;
  have_battery_ = false;
  battery_percentage_ = -1.0;
  reached_goal_count_ = 0;
  three_wall_complete_streak_ = 0;
  map_closure_complete_streak_ = 0;
  map_growth_reference_count_ = 0;
  mission_heading_yaw_ = 0.0;
  exploration_phase_ = ExplorationPhase::kForwardPriority;
  left_wall_visible_ = false;
  right_wall_visible_ = false;
  front_obstacle_visible_ = false;
  side_wall_missing_streak_ = 0;
  front_obstacle_streak_ = 0;
  first_data_time_ = ros::Time();
  last_frontier_time_ = ros::Time();
  last_actionable_frontier_time_ = ros::Time();
  last_significant_map_growth_time_ = ros::Time();
  std_msgs::Bool false_message;
  false_message.data = false;
  finished_publisher_.publish(false_message);
  returning_publisher_.publish(false_message);
  model_complete_publisher_.publish(false_message);
}

bool SuperExplorationDecider::enableCallback(
    std_srvs::SetBool::Request& request,
    std_srvs::SetBool::Response& response) {
  if (!request.data) {
    cancelActiveGoal("service_disabled", true);
  } else if (!enabled_) {
    cancelActiveGoal("service_reenabled", true);
    clearMissionState();
  }
  enabled_ = request.data;
  mission_start_pending_ = enabled_;
  response.success = true;
  response.message = enabled_ ? "exploration enabled" : "exploration paused";
  publishStatus(enabled_ ? "WAIT_DATA" : "DISABLED", response.message);
  return true;
}

bool SuperExplorationDecider::resetCallback(
    std_srvs::Trigger::Request&, std_srvs::Trigger::Response& response) {
  cancelActiveGoal("service_reset", true);
  clearMissionState();
  enabled_ = !require_mission_enable_edge_;
  mission_start_pending_ = false;
  response.success = true;
  response.message = "exploration state reset";
  publishStatus("WAIT_DATA", response.message);
  return true;
}

}  // namespace mine_uav_control
