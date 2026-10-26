#include "mine_uav_control/super_exploration_decider.hpp"
#include "mine_uav_control/anisotropic_clearance.hpp"
#include "mine_uav_control/outbound_goal_contract.hpp"
#include "mine_uav_control/observation_height_band.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <stdexcept>
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
constexpr uint8_t kPendingOccupied = 3;

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
      cloud_subscriber_(nh_, "/mine_uav/task1/cloud_registered", 1),
      odom_subscriber_(nh_, "/mine_uav/task1/odometry", 1) {
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
  // The map still uses cloud-synchronized odometry. Progress toward an
  // already verified queued goal uses the newest planning odometry so a
  // delayed cloud cannot postpone the configured two-metre handover.
  progress_odometry_subscriber_ = nh_.subscribe(
      odom_topic_, 10,
      &SuperExplorationDecider::odometryProgressCallback, this);

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
  initial_goal_accepted_publisher_ =
      nh_.advertise<std_msgs::UInt64>(initial_goal_accepted_topic_, 1, true);
  visualization_publisher_ =
      nh_.advertise<visualization_msgs::MarkerArray>(visualization_topic_, 1);

  return_request_subscriber_ = nh_.subscribe(
      return_request_topic_, 1,
      &SuperExplorationDecider::returnRequestCallback, this);
  obstacle_replan_subscriber_ = nh_.subscribe(
      obstacle_replan_topic_, 1,
      &SuperExplorationDecider::obstacleReplanCallback, this);
  rejected_goal_subscriber_ = nh_.subscribe(
      "/mine_uav/super/rejected_goal_id", 10,
      &SuperExplorationDecider::rejectedGoalCallback, this);
  accepted_goal_subscriber_ = nh_.subscribe(
      "/mine_uav/super/accepted_goal_id", 10,
      &SuperExplorationDecider::acceptedGoalCallback, this);
  mission_enable_subscriber_ = nh_.subscribe(
      mission_enable_topic_, 1,
      &SuperExplorationDecider::missionEnableCallback, this);
  if (follow_active_task_profile_) {
    active_task_subscriber_ = nh_.subscribe(
        active_task_topic_, 1,
        &SuperExplorationDecider::activeTaskCallback, this);
  }
  battery_subscriber_ =
      nh_.subscribe(battery_topic_, 1, &SuperExplorationDecider::batteryCallback,
                    this);
  if (!free_ray_topic_.empty()) {
    free_ray_subscriber_ = nh_.subscribe(
        free_ray_topic_, 1, &SuperExplorationDecider::freeRayCallback, this);
  }
  super_occupied_subscriber_ = nh_.subscribe(
      super_occupied_topic_, 1,
      &SuperExplorationDecider::superOccupiedCallback, this);
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
  publishInitialGoalAccepted(0);
  publishStatus("WAIT_DATA", "waiting for synchronized Omni-LIVO data");
  ROS_INFO("Exploration mission profile: %s; corridor guidance=%s; "
           "return behavior=normal",
           explorationMissionProfileName(mission_profile_),
           corridorGuidanceEnabled() ? "enabled" : "disabled");
}

bool SuperExplorationDecider::loadParameters() {
  private_nh_.param("mission_profile", mission_profile_name_,
                    mission_profile_name_);
  if (!parseExplorationMissionProfile(mission_profile_name_,
                                      &mission_profile_)) {
    throw std::invalid_argument(
        "mission_profile must be either 'goaf' or 'tunnel'");
  }
  private_nh_.param("cloud_topic", cloud_topic_,
                    std::string("/mine_uav/task1/cloud_registered"));
  private_nh_.param("free_ray_topic", free_ray_topic_, std::string());
  private_nh_.param("odom_topic", odom_topic_,
                    std::string("/mine_uav/task1/odometry"));
  private_nh_.param("goal_command_topic", goal_command_topic_,
                    std::string("/mine_uav/super/goal_command"));
  private_nh_.param("world_frame", world_frame_, std::string("world"));
  private_nh_.param("return_request_topic", return_request_topic_,
                    std::string("/mine_uav/exploration/return_home"));
  private_nh_.param("obstacle_replan_topic", obstacle_replan_topic_,
                    std::string("/mine_uav/exploration/obstacle_replan"));
  private_nh_.param("mission_enable_topic", mission_enable_topic_,
                    std::string("/mine_uav/mission/goaf_enable"));
  private_nh_.param("active_task_topic", active_task_topic_,
                    std::string("/mine_uav/mission/active_task"));
  private_nh_.param("follow_active_task_profile", follow_active_task_profile_,
                    follow_active_task_profile_);
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
  private_nh_.param(
      "initial_goal_accepted_topic", initial_goal_accepted_topic_,
      std::string("/mine_uav/exploration/accepted_initial_goal_id"));
  private_nh_.param("visualization_topic", visualization_topic_,
                    std::string("/mine_uav/exploration/frontiers"));
  private_nh_.param("require_mission_enable_edge",
                    require_mission_enable_edge_,
                    require_mission_enable_edge_);
  private_nh_.param("mission_heading_offset", mission_heading_offset_,
                    mission_heading_offset_);
  private_nh_.param("use_absolute_mission_heading",
                    use_absolute_mission_heading_,
                    use_absolute_mission_heading_);
  private_nh_.param("absolute_mission_heading_yaw",
                    absolute_mission_heading_yaw_,
                    absolute_mission_heading_yaw_);
  private_nh_.param("dead_end_reversal_delay", dead_end_reversal_delay_,
                    dead_end_reversal_delay_);
  private_nh_.param("max_dead_end_heading_reversals",
                    max_dead_end_heading_reversals_,
                    max_dead_end_heading_reversals_);
  private_nh_.param("return_on_confirmed_dead_end",
                    return_on_confirmed_dead_end_,
                    return_on_confirmed_dead_end_);
  private_nh_.param("carve_traversed_vehicle_envelope",
                    carve_traversed_vehicle_envelope_,
                    carve_traversed_vehicle_envelope_);

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
  private_nh_.param("preferred_vehicle_radius", preferred_vehicle_radius_,
                    vehicle_radius_);
  private_nh_.param("max_outbound_backtrack", max_outbound_backtrack_,
                    max_outbound_backtrack_);
  private_nh_.param("vertical_vehicle_radius", vertical_vehicle_radius_,
                    vertical_vehicle_radius_);
  private_nh_.param("vertical_vehicle_clearance_below",
                    vertical_vehicle_clearance_below_,
                    vertical_vehicle_clearance_below_);
  private_nh_.param("vertical_vehicle_clearance_above",
                    vertical_vehicle_clearance_above_,
                    vertical_vehicle_clearance_above_);
  private_nh_.param("min_observation_height_above_home",
                    min_observation_height_above_home_,
                    min_observation_height_above_home_);
  private_nh_.param("max_observation_height_above_home",
                    max_observation_height_above_home_,
                    max_observation_height_above_home_);
  private_nh_.param("tunnel_min_planning_height",
                    tunnel_min_planning_height_,
                    tunnel_min_planning_height_);
  private_nh_.param("tunnel_max_planning_height",
                    tunnel_max_planning_height_,
                    tunnel_max_planning_height_);
  private_nh_.param("data_timeout", data_timeout_, data_timeout_);
  private_nh_.param("active_goal_data_grace", active_goal_data_grace_,
                    active_goal_data_grace_);
  private_nh_.param("decision_rate", decision_rate_, decision_rate_);
  private_nh_.param("sync_slop", sync_slop_, sync_slop_);
  private_nh_.param("battery_start_guard_enabled",
                    battery_start_guard_enabled_,
                    battery_start_guard_enabled_);
  private_nh_.param("battery_percentage_return_enabled",
                    battery_percentage_return_enabled_,
                    battery_percentage_return_enabled_);
  private_nh_.param("battery_start_minimum_percentage",
                    battery_start_minimum_percentage_,
                    battery_start_minimum_percentage_);
  private_nh_.param("battery_start_minimum_voltage",
                    battery_start_minimum_voltage_,
                    battery_start_minimum_voltage_);
  private_nh_.param("battery_return_threshold", battery_return_threshold_,
                    battery_return_threshold_);
  private_nh_.param("battery_return_confirm_duration",
                    battery_return_confirm_duration_,
                    battery_return_confirm_duration_);
  private_nh_.param("battery_return_hysteresis",
                    battery_return_hysteresis_,
                    battery_return_hysteresis_);
  private_nh_.param("battery_return_voltage", battery_return_voltage_,
                    battery_return_voltage_);
  private_nh_.param("return_home_height_offset", return_home_height_offset_,
                    return_home_height_offset_);
  private_nh_.param("return_max_height_above_home",
                    return_max_height_above_home_,
                    max_observation_height_above_home_);
  private_nh_.param("return_home_arrival_radius",
                    return_home_arrival_radius_, return_home_arrival_radius_);
  private_nh_.param("return_breadcrumb_spacing", return_breadcrumb_spacing_,
                    return_breadcrumb_spacing_);
  private_nh_.param("tunnel_return_breadcrumb_spacing",
                    tunnel_return_breadcrumb_spacing_,
                    tunnel_return_breadcrumb_spacing_);
  private_nh_.param("super_occupied_topic", super_occupied_topic_,
                    super_occupied_topic_);
  private_nh_.param("return_segment_length", return_segment_length_,
                    return_segment_length_);
  private_nh_.param("return_goal_handover_distance",
                    return_goal_handover_distance_,
                    return_goal_handover_distance_);
  private_nh_.param("return_map_timeout", return_map_timeout_,
                    return_map_timeout_);
  private_nh_.param("return_clearance_margin", return_clearance_margin_,
                    return_clearance_margin_);
  private_nh_.param("return_super_radius", return_super_radius_,
                    return_super_radius_);
  private_nh_.param("min_forward_super_clearance",
                    min_forward_super_clearance_,
                    min_forward_super_clearance_);
  private_nh_.param("min_clearance_return_progress",
                    min_clearance_return_progress_,
                    min_clearance_return_progress_);
  return_segment_length_ = std::max(1.0, return_segment_length_);
  return_goal_handover_distance_ = std::max(
      goal_reached_distance_, return_goal_handover_distance_);
  return_map_timeout_ = std::max(0.5, return_map_timeout_);
  return_clearance_margin_ = std::max(0.0, return_clearance_margin_);
  return_super_radius_ = std::max(0.0, return_super_radius_);
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
  private_nh_.param("corridor_centering_enabled",
                    corridor_centering_enabled_,
                    corridor_centering_enabled_);
  private_nh_.param("corridor_centering_min_wall_points",
                    corridor_centering_min_wall_points_,
                    corridor_centering_min_wall_points_);
  private_nh_.param("corridor_centering_confirm_frames",
                    corridor_centering_confirm_frames_,
                    corridor_centering_confirm_frames_);
  private_nh_.param("corridor_centering_release_frames",
                    corridor_centering_release_frames_,
                    corridor_centering_release_frames_);
  private_nh_.param("corridor_centering_max_width",
                    corridor_centering_max_width_,
                    corridor_centering_max_width_);
  private_nh_.param("corridor_centering_filter_gain",
                    corridor_centering_filter_gain_,
                    corridor_centering_filter_gain_);
  private_nh_.param("corridor_centering_max_center_jump",
                    corridor_centering_max_center_jump_,
                    corridor_centering_max_center_jump_);
  private_nh_.param("corridor_centering_max_goal_shift",
                    corridor_centering_max_goal_shift_,
                    corridor_centering_max_goal_shift_);
  private_nh_.param("corridor_preview_enabled", corridor_preview_enabled_,
                    corridor_preview_enabled_);
  private_nh_.param("corridor_preview_start_distance",
                    corridor_preview_start_distance_,
                    corridor_preview_start_distance_);
  private_nh_.param("corridor_preview_distance", corridor_preview_distance_,
                    corridor_preview_distance_);
  private_nh_.param("corridor_preview_longitudinal_step",
                    corridor_preview_longitudinal_step_,
                    corridor_preview_longitudinal_step_);
  private_nh_.param("corridor_preview_lateral_half_width",
                    corridor_preview_lateral_half_width_,
                    corridor_preview_lateral_half_width_);
  private_nh_.param("corridor_preview_lateral_step",
                    corridor_preview_lateral_step_,
                    corridor_preview_lateral_step_);
  private_nh_.param("corridor_preview_max_lateral_slope",
                    corridor_preview_max_lateral_slope_,
                    corridor_preview_max_lateral_slope_);
  private_nh_.param("corridor_preview_min_slices",
                    corridor_preview_min_slices_,
                    corridor_preview_min_slices_);
  private_nh_.param("corridor_preview_min_reliable_center_band_width",
                    corridor_preview_min_reliable_center_band_width_,
                    corridor_preview_min_reliable_center_band_width_);
  private_nh_.param("corridor_preview_min_center_shift",
                    corridor_preview_min_center_shift_,
                    corridor_preview_min_center_shift_);
  private_nh_.param("corridor_preview_min_width_contraction",
                    corridor_preview_min_width_contraction_,
                    corridor_preview_min_width_contraction_);
  private_nh_.param("corridor_preview_max_goal_shift",
                    corridor_preview_max_goal_shift_,
                    corridor_preview_max_goal_shift_);
  private_nh_.param("front_obstacle_range", front_obstacle_range_,
                    front_obstacle_range_);
  private_nh_.param("front_obstacle_min_progress",
                    front_obstacle_min_progress_,
                    front_obstacle_min_progress_);
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
  private_nh_.param("goaf_front_obstacle_min_lateral_span",
                    goaf_front_obstacle_min_lateral_span_,
                    goaf_front_obstacle_min_lateral_span_);
  private_nh_.param("goaf_front_obstacle_min_vertical_span",
                    goaf_front_obstacle_min_vertical_span_,
                    goaf_front_obstacle_min_vertical_span_);
  private_nh_.param("forward_corridor_half_width",
                    forward_corridor_half_width_,
                    forward_corridor_half_width_);
  private_nh_.param("forward_goal_lateral_search_width",
                    forward_goal_lateral_search_width_,
                    forward_goal_lateral_search_width_);
  private_nh_.param("lateral_route_switch_clearance_advantage",
                    lateral_route_switch_clearance_advantage_,
                    lateral_route_switch_clearance_advantage_);
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
  private_nh_.param("goaf_forward_lookahead_distance",
                    goaf_forward_lookahead_distance_,
                    goaf_forward_lookahead_distance_);
  private_nh_.param("max_frontier_goal_distance",
                    max_frontier_goal_distance_,
                    max_frontier_goal_distance_);
  private_nh_.param("max_frontier_goal_vertical_step",
                    max_frontier_goal_vertical_step_,
                    max_frontier_goal_vertical_step_);
  private_nh_.param("forward_lookahead_step", forward_lookahead_step_,
                    forward_lookahead_step_);
  private_nh_.param("exploration_goal_queue_enabled",
                    exploration_goal_queue_enabled_,
                    exploration_goal_queue_enabled_);
  private_nh_.param("exploration_goal_queue_max_future_goals",
                    exploration_goal_queue_max_future_goals_,
                    exploration_goal_queue_max_future_goals_);
  private_nh_.param("exploration_goal_queue_horizon",
                    exploration_goal_queue_horizon_,
                    exploration_goal_queue_horizon_);
  private_nh_.param("goaf_exploration_goal_queue_horizon",
                    goaf_exploration_goal_queue_horizon_,
                    goaf_exploration_goal_queue_horizon_);
  private_nh_.param("exploration_goal_queue_spacing",
                    exploration_goal_queue_spacing_,
                    exploration_goal_queue_spacing_);
  private_nh_.param("exploration_goal_queue_end_reserve",
                    exploration_goal_queue_end_reserve_,
                    exploration_goal_queue_end_reserve_);
  private_nh_.param("connected_turn_continuation_enabled",
                    connected_turn_continuation_enabled_,
                    connected_turn_continuation_enabled_);
  private_nh_.param("connected_turn_max_angle_deg",
                    connected_turn_max_angle_deg_,
                    connected_turn_max_angle_deg_);
  private_nh_.param("connected_turn_min_new_space",
                    connected_turn_min_new_space_,
                    connected_turn_min_new_space_);
  private_nh_.param("tunnel_route_lock_min_turn_deg",
                    tunnel_route_lock_min_turn_deg_,
                    tunnel_route_lock_min_turn_deg_);
  private_nh_.param("tunnel_connected_turn_max_angle_deg",
                    tunnel_connected_turn_max_angle_deg_,
                    tunnel_connected_turn_max_angle_deg_);
  private_nh_.param("tunnel_straight_preference_angle_deg",
                    tunnel_straight_preference_angle_deg_,
                    tunnel_straight_preference_angle_deg_);
  private_nh_.param("tunnel_connected_turn_confirm_frames",
                    tunnel_connected_turn_confirm_frames_,
                    tunnel_connected_turn_confirm_frames_);
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
  private_nh_.param("task1_end_wall_return_enabled",
                    task1_end_wall_return_enabled_,
                    task1_end_wall_return_enabled_);
  private_nh_.param("task1_end_wall_return_distance",
                    task1_end_wall_return_distance_,
                    task1_end_wall_return_distance_);
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
  private_nh_.param("occupied_hit_increment", occupied_hit_increment_,
                    occupied_hit_increment_);
  private_nh_.param("free_ray_decrement", free_ray_decrement_,
                    free_ray_decrement_);
  private_nh_.param("occupied_confirm_threshold", occupied_confirm_threshold_,
                    occupied_confirm_threshold_);
  private_nh_.param("free_confirm_threshold", free_confirm_threshold_,
                    free_confirm_threshold_);
  private_nh_.param("occupancy_evidence_min", occupancy_evidence_min_,
                    occupancy_evidence_min_);
  private_nh_.param("occupancy_evidence_max", occupancy_evidence_max_,
                    occupancy_evidence_max_);
  private_nh_.param("protect_traversed_centerline", protect_traversed_centerline_,
                    protect_traversed_centerline_);
  private_nh_.param("traversed_centerline_radius", traversed_centerline_radius_,
                    traversed_centerline_radius_);
  private_nh_.param("traversed_centerline_step", traversed_centerline_step_,
                    traversed_centerline_step_);

  if (occupied_hit_increment_ <= 0 || free_ray_decrement_ <= 0 ||
      occupied_confirm_threshold_ <= free_confirm_threshold_ ||
      occupancy_evidence_min_ > free_confirm_threshold_ ||
      occupancy_evidence_max_ < occupied_confirm_threshold_) {
    throw std::invalid_argument("invalid occupancy evidence thresholds");
  }
  if (!std::isfinite(voxel_resolution_) || voxel_resolution_ <= 0.0 ||
      !std::isfinite(vehicle_radius_) || vehicle_radius_ <= 0.0 ||
      !std::isfinite(preferred_vehicle_radius_) ||
      preferred_vehicle_radius_ < vehicle_radius_ ||
      !std::isfinite(mission_heading_offset_) ||
      !std::isfinite(max_outbound_backtrack_) || max_outbound_backtrack_ < 0.0 ||
      !std::isfinite(vertical_vehicle_radius_) ||
      (vertical_vehicle_radius_ != -1.0 && vertical_vehicle_radius_ <= 0.0) ||
      !std::isfinite(vertical_vehicle_clearance_below_) ||
      vertical_vehicle_clearance_below_ <= 0.0 ||
      !std::isfinite(vertical_vehicle_clearance_above_) ||
      vertical_vehicle_clearance_above_ <= 0.0 ||
      !std::isfinite(decision_rate_) || decision_rate_ <= 0.0 ||
      !std::isfinite(data_timeout_) || data_timeout_ <= 0.0 ||
      !std::isfinite(active_goal_data_grace_) ||
      active_goal_data_grace_ < 0.0 ||
      !std::isfinite(max_exploration_radius_from_home_) ||
      max_exploration_radius_from_home_ < 0.0 ||
      !std::isfinite(mission_heading_offset_) ||
      !std::isfinite(absolute_mission_heading_yaw_) ||
      !std::isfinite(return_home_height_offset_) ||
      return_home_height_offset_ < 0.0 ||
      !std::isfinite(return_max_height_above_home_) ||
      return_max_height_above_home_ < return_home_height_offset_ ||
      return_max_height_above_home_ > max_observation_height_above_home_ ||
      !std::isfinite(return_home_arrival_radius_) ||
      return_home_arrival_radius_ < 0.0 ||
      !std::isfinite(return_breadcrumb_spacing_) ||
      return_breadcrumb_spacing_ < 1.0 ||
      !std::isfinite(tunnel_return_breadcrumb_spacing_) ||
      tunnel_return_breadcrumb_spacing_ < 0.5 ||
      tunnel_return_breadcrumb_spacing_ > return_breadcrumb_spacing_ ||
      !std::isfinite(heading_priority_weight_) ||
      !std::isfinite(fallback_heading_weight_) ||
      !std::isfinite(forward_sector_deg_) || forward_sector_deg_ <= 0.0 ||
      forward_sector_deg_ >= 180.0 ||
      !std::isfinite(side_wall_sector_deg_) || side_wall_sector_deg_ <= 0.0 ||
      side_wall_sector_deg_ >= 180.0 ||
      !std::isfinite(side_wall_min_range_) || side_wall_min_range_ <= 0.0 ||
      !std::isfinite(side_wall_max_range_) ||
      side_wall_max_range_ <= side_wall_min_range_ ||
      corridor_centering_min_wall_points_ < 1 ||
      corridor_centering_confirm_frames_ < 1 ||
      corridor_centering_release_frames_ < 1 ||
      !std::isfinite(corridor_centering_max_width_) ||
      corridor_centering_max_width_ <= 2.0 * vehicle_radius_ ||
      !std::isfinite(corridor_centering_filter_gain_) ||
      corridor_centering_filter_gain_ <= 0.0 ||
      corridor_centering_filter_gain_ > 1.0 ||
      !std::isfinite(corridor_centering_max_center_jump_) ||
      corridor_centering_max_center_jump_ <= 0.0 ||
      !std::isfinite(corridor_centering_max_goal_shift_) ||
      corridor_centering_max_goal_shift_ <= 0.0 ||
      corridor_centering_max_goal_shift_ > forward_goal_lateral_search_width_ ||
      !std::isfinite(corridor_preview_start_distance_) ||
      corridor_preview_start_distance_ <= 0.0 ||
      !std::isfinite(corridor_preview_distance_) ||
      corridor_preview_distance_ < corridor_preview_start_distance_ ||
      !std::isfinite(corridor_preview_longitudinal_step_) ||
      corridor_preview_longitudinal_step_ <= 0.0 ||
      !std::isfinite(corridor_preview_lateral_half_width_) ||
      corridor_preview_lateral_half_width_ <= vehicle_radius_ ||
      !std::isfinite(corridor_preview_lateral_step_) ||
      corridor_preview_lateral_step_ <= 0.0 ||
      !std::isfinite(corridor_preview_max_lateral_slope_) ||
      corridor_preview_max_lateral_slope_ <= 0.0 ||
      corridor_preview_min_slices_ < 2 ||
      !std::isfinite(corridor_preview_min_reliable_center_band_width_) ||
      corridor_preview_min_reliable_center_band_width_ <
          2.0 * corridor_preview_lateral_step_ ||
      !std::isfinite(corridor_preview_min_center_shift_) ||
      corridor_preview_min_center_shift_ < 0.0 ||
      !std::isfinite(corridor_preview_min_width_contraction_) ||
      corridor_preview_min_width_contraction_ < 0.0 ||
      !std::isfinite(corridor_preview_max_goal_shift_) ||
      corridor_preview_max_goal_shift_ <= 0.0 ||
      !std::isfinite(front_obstacle_range_) || front_obstacle_range_ <= 0.0 ||
      !std::isfinite(front_obstacle_min_progress_) ||
      front_obstacle_min_progress_ < 0.0 ||
      !std::isfinite(front_obstacle_sector_deg_) ||
      front_obstacle_sector_deg_ <= 0.0 ||
      front_obstacle_sector_deg_ >= 180.0 ||
      front_obstacle_min_points_ < 1 ||
      !std::isfinite(front_obstacle_min_lateral_span_) ||
      front_obstacle_min_lateral_span_ <= 0.0 ||
      !std::isfinite(front_obstacle_min_vertical_span_) ||
      front_obstacle_min_vertical_span_ <= 0.0 ||
      !std::isfinite(goaf_front_obstacle_min_lateral_span_) ||
      goaf_front_obstacle_min_lateral_span_ <= 0.0 ||
      !std::isfinite(goaf_front_obstacle_min_vertical_span_) ||
      goaf_front_obstacle_min_vertical_span_ <= 0.0 ||
      !std::isfinite(forward_corridor_half_width_) ||
      forward_corridor_half_width_ <= vehicle_radius_ ||
      !std::isfinite(forward_goal_lateral_search_width_) ||
      forward_goal_lateral_search_width_ < 0.0 ||
      forward_goal_lateral_search_width_ > forward_corridor_half_width_ ||
      !std::isfinite(lateral_route_switch_clearance_advantage_) ||
      lateral_route_switch_clearance_advantage_ < 0.0 ||
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
      !std::isfinite(goaf_forward_lookahead_distance_) ||
      (goaf_forward_lookahead_distance_ > 0.0 &&
       goaf_forward_lookahead_distance_ <= forward_goal_handover_distance_) ||
      !std::isfinite(max_frontier_goal_distance_) ||
      max_frontier_goal_distance_ < min_goal_distance_ ||
      !std::isfinite(max_frontier_goal_vertical_step_) ||
      max_frontier_goal_vertical_step_ <= 0.0 ||
      !std::isfinite(forward_lookahead_step_) ||
      forward_lookahead_step_ <= 0.0 ||
      exploration_goal_queue_max_future_goals_ < 1 ||
      exploration_goal_queue_max_future_goals_ > 3 ||
      !std::isfinite(exploration_goal_queue_horizon_) ||
      exploration_goal_queue_horizon_ < forward_lookahead_distance_ ||
      !std::isfinite(goaf_exploration_goal_queue_horizon_) ||
      (goaf_exploration_goal_queue_horizon_ > 0.0 &&
       goaf_exploration_goal_queue_horizon_ <
           effectiveForwardLookaheadDistance()) ||
      !std::isfinite(exploration_goal_queue_spacing_) ||
      exploration_goal_queue_spacing_ <= forward_goal_handover_distance_ ||
      !std::isfinite(exploration_goal_queue_end_reserve_) ||
      exploration_goal_queue_end_reserve_ < 0.0 ||
      exploration_goal_queue_end_reserve_ >=
          exploration_goal_queue_spacing_ ||
      !std::isfinite(connected_turn_max_angle_deg_) ||
      connected_turn_max_angle_deg_ <= 0.0 ||
      connected_turn_max_angle_deg_ >= 90.0 ||
      !std::isfinite(connected_turn_min_new_space_) ||
      connected_turn_min_new_space_ < 0.0 ||
      !std::isfinite(tunnel_route_lock_min_turn_deg_) ||
      tunnel_route_lock_min_turn_deg_ <= 0.0 ||
      tunnel_route_lock_min_turn_deg_ >= 90.0 ||
      !std::isfinite(tunnel_connected_turn_max_angle_deg_) ||
      tunnel_connected_turn_max_angle_deg_ <= 90.0 ||
      tunnel_connected_turn_max_angle_deg_ >= 170.0 ||
      !std::isfinite(tunnel_straight_preference_angle_deg_) ||
      tunnel_straight_preference_angle_deg_ <= 0.0 ||
      tunnel_straight_preference_angle_deg_ >=
          tunnel_connected_turn_max_angle_deg_ ||
      tunnel_connected_turn_confirm_frames_ < 1 ||
      tunnel_connected_turn_confirm_frames_ > 10 ||
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
      !std::isfinite(task1_end_wall_return_distance_) ||
      task1_end_wall_return_distance_ <= vehicle_radius_ ||
      task1_end_wall_return_distance_ > front_obstacle_range_ ||
      !std::isfinite(min_observation_height_above_home_) ||
      !std::isfinite(max_observation_height_above_home_) ||
      max_observation_height_above_home_ <=
          min_observation_height_above_home_ ||
      !std::isfinite(tunnel_min_planning_height_) ||
      !std::isfinite(tunnel_max_planning_height_) ||
      tunnel_max_planning_height_ <= tunnel_min_planning_height_ ||
      (battery_start_guard_enabled_ &&
       (!std::isfinite(battery_start_minimum_percentage_) ||
        battery_start_minimum_percentage_ < 0.0 ||
        battery_start_minimum_percentage_ > 1.0 ||
        !std::isfinite(battery_start_minimum_voltage_) ||
        battery_start_minimum_voltage_ <= battery_return_voltage_)) ||
      (battery_percentage_return_enabled_ &&
       (!std::isfinite(battery_return_threshold_) ||
        battery_return_threshold_ < 0.0 ||
        battery_return_threshold_ > 1.0 ||
        !std::isfinite(battery_return_hysteresis_) ||
        battery_return_hysteresis_ < 0.0 ||
        battery_return_threshold_ + battery_return_hysteresis_ > 1.0 ||
        (battery_start_guard_enabled_ &&
         battery_start_minimum_percentage_ <
             battery_return_threshold_ + battery_return_hysteresis_))) ||
      !std::isfinite(battery_return_voltage_) ||
      battery_return_voltage_ <= 0.0 ||
      !std::isfinite(battery_return_confirm_duration_) ||
      battery_return_confirm_duration_ < 0.0 ||
      side_wall_missing_confirm_frames_ < 1 ||
      front_obstacle_confirm_frames_ < 1 ||
      wall_coverage_max_gap_bins_ < 0 || three_wall_confirm_cycles_ < 1 ||
      !std::isfinite(map_closure_min_progress_) ||
      map_closure_min_progress_ < 0.0 ||
      !std::isfinite(map_closure_no_frontier_time_) ||
      map_closure_no_frontier_time_ <= 0.0 ||
      !std::isfinite(map_closure_stable_time_) ||
      map_closure_stable_time_ <= 0.0 ||
      !std::isfinite(dead_end_reversal_delay_) ||
      dead_end_reversal_delay_ < 0.0 ||
      map_closure_max_actionable_frontiers_ < 0 ||
      map_closure_growth_voxels_ < 1 || map_closure_confirm_cycles_ < 1 ||
      max_dead_end_heading_reversals_ < 0 ||
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
  if (last_motion_progress_time_.isZero() ||
      squaredDistance(current_pose_.pose.position,
                      last_motion_progress_position_) >= 0.25) {
    last_motion_progress_position_ = current_pose_.pose.position;
    last_motion_progress_time_ = last_sync_time_;
  }
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
    const double raw_mission_heading = use_absolute_mission_heading_
        ? absolute_mission_heading_yaw_
        : poseYaw(home_pose_.pose) + mission_heading_offset_;
    mission_heading_yaw_ = std::atan2(std::sin(raw_mission_heading),
                                     std::cos(raw_mission_heading));
    have_home_ = true;
    outbound_breadcrumbs_.clear();
    outbound_breadcrumbs_.push_back(home_pose_);
    mission_start_pending_ = false;
    ROS_INFO("Exploration home captured at %.2f %.2f %.2f, heading %.1f deg",
             home_pose_.pose.position.x, home_pose_.pose.position.y,
             home_pose_.pose.position.z,
             mission_heading_yaw_ * 180.0 / M_PI);
  }
  // PointCloud2 conversion is linear in every raw point.  Convert once and
  // share that immutable cloud between occupancy raycasting and directional
  // wall evidence; the old path converted the same high-density Omni-LIVO
  // frame twice on the single callback thread.
  pcl::PointCloud<pcl::PointXYZ> input;
  try {
    pcl::fromROSMsg(*cloud, input);
  } catch (const std::exception& error) {
    ROS_WARN_THROTTLE(2.0, "Cannot convert Omni-LIVO PointCloud2: %s",
                      error.what());
    return;
  }
  updateMap(input, cloud->header.stamp, current_pose_);
  if (protect_traversed_centerline_) {
    protectTraversedCenterline(current_pose_.pose.position);
  }
  if (carve_traversed_vehicle_envelope_) {
    carveTraversedVehicleEnvelope(current_pose_.pose.position);
  }
  updateDirectionalEvidence(input, current_pose_);
  if (enabled_ && exploration_started_ && !returning_home_ &&
      !dead_end_backtracking_ && !mission_finished_ &&
      !outbound_breadcrumbs_.empty() &&
      std::sqrt(squaredDistance(current_pose_.pose.position,
          outbound_breadcrumbs_.back().pose.position)) >=
          (mission_profile_ == ExplorationMissionProfile::kTunnel
               ? tunnel_return_breadcrumb_spacing_
               : return_breadcrumb_spacing_)) {
    outbound_breadcrumbs_.push_back(current_pose_);
  }
}

void SuperExplorationDecider::odometryProgressCallback(
    const Odom::ConstPtr& odom) {
  const auto& p = odom->pose.pose.position;
  if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
      (!odom->header.frame_id.empty() &&
       odom->header.frame_id != world_frame_)) {
    return;
  }
  latest_progress_pose_.header = odom->header;
  latest_progress_pose_.header.frame_id = world_frame_;
  latest_progress_pose_.pose = odom->pose.pose;
  last_progress_odometry_time_ = ros::Time::now();
  have_progress_odometry_ = true;
}

ros::Time SuperExplorationDecider::freshnessReferenceTime() const {
  return decision_snapshot_time_.isZero()
      ? ros::Time::now() : decision_snapshot_time_;
}

bool SuperExplorationDecider::dataIsFresh() const {
  if (!have_data_ || last_sync_time_.isZero()) {
    return false;
  }
  return (freshnessReferenceTime() - last_sync_time_).toSec() <=
      data_timeout_;
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
  int16_t& evidence = occupancy_evidence_[key];
  evidence = std::max(occupancy_evidence_min_, evidence - free_ray_decrement_);
  if (evidence <= free_confirm_threshold_) voxels_[key] = kFree;
}

void SuperExplorationDecider::markOccupied(const VoxelKey& key) {
  int16_t& evidence = occupancy_evidence_[key];
  // A new return immediately blocks navigation; "not yet confirmed" is NOT free.
  evidence = std::min(occupancy_evidence_max_, std::max(0, int(evidence)) + occupied_hit_increment_);
  voxels_[key] = evidence >= occupied_confirm_threshold_ ? kOccupied : kPendingOccupied;
}

bool SuperExplorationDecider::isOccupied(const VoxelKey& key) const {
  const auto it = voxels_.find(key);
  return it != voxels_.end() &&
      (it->second == kOccupied || it->second == kPendingOccupied);
}

bool SuperExplorationDecider::isKnownFree(const VoxelKey& key) const {
  const auto it = voxels_.find(key);
  return it != voxels_.end() && it->second == kFree;
}

void SuperExplorationDecider::superOccupiedCallback(const Cloud::ConstPtr& cloud) {
  if (cloud->header.frame_id != world_frame_ || cloud->header.stamp.isZero()) {
    return;
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr points(new pcl::PointCloud<pcl::PointXYZ>);
  try {
    pcl::fromROSMsg(*cloud, *points);
  } catch (const std::exception& error) {
    ROS_WARN_THROTTLE(2.0, "Cannot read SUPER occupied map: %s", error.what());
    return;
  }
  points->points.erase(std::remove_if(points->points.begin(), points->points.end(),
      [](const pcl::PointXYZ& p) {
        return !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z);
      }), points->points.end());
  points->width = points->points.size();
  points->height = 1;
  super_occupied_points_ = points;
  if (!points->empty()) super_occupied_tree_.setInputCloud(points);
  super_occupied_stamp_ = cloud->header.stamp;
}

void SuperExplorationDecider::freeRayCallback(const Cloud::ConstPtr& cloud) {
  // This optional simulation-only stream carries *ends of traversed rays*,
  // not obstacle hits.  Real Omni-LIVO deployments leave free_ray_topic unset.
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
  const geometry_msgs::Point& origin=current_pose_.pose.position;
  VoxelSet misses;
  for (const auto& point:endpoints) {
    if (!std::isfinite(point.x)||!std::isfinite(point.y)||!std::isfinite(point.z)) continue;
    const double dx=point.x-origin.x,dy=point.y-origin.y,dz=point.z-origin.z;
    const double distance=std::sqrt(dx*dx+dy*dy+dz*dz);
    if (distance<voxel_resolution_||distance>raycast_max_range_+voxel_resolution_) continue;
    const int steps=static_cast<int>(std::ceil(distance/(voxel_resolution_*0.5)));
    for(int k=1;k<=steps;++k) {
      const double r=double(k)/steps;
      const VoxelKey key=positionToKey(origin.x+r*dx,origin.y+r*dy,origin.z+r*dz);
      if (last_scan_hits_.count(key) &&
          std::abs((cloud->header.stamp-last_scan_stamp_).toSec()) <= sync_slop_+0.15) break;
      if (isOccupied(key)) break; // unbounded/no-return rays cannot disprove walls
      misses.insert(key);
    }
  }
  for(const auto& key:misses) markFree(key);
}

SuperExplorationDecider::VoxelSet
SuperExplorationDecider::reachableFreeVoxels() const {
  VoxelSet reachable;
  if (!have_home_) {
    return reachable;
  }

  const bool use_decision_grid = decision_clearance_cache_active_ &&
      decision_clearance_index_ready_ && !decision_grid_free_.empty();
  std::vector<uint8_t> visited;
  if (use_decision_grid) visited.assign(decision_grid_free_.size(), 0);
  const auto known_free = [&](const VoxelKey& key) {
    if (use_decision_grid) {
      std::size_t index = 0;
      return decisionGridIndex(key, &index) &&
             decision_grid_free_[index] != 0;
    }
    return isKnownFree(key);
  };
  // Clearance is cached for the whole immutable decision snapshot, not just
  // this flood fill, because frontier/closure/queue validation reuse it. The
  // production fast path reads the pre-dilated byte grid directly.
  const auto vehicle_clear = [&](const VoxelKey& key) {
    if (use_decision_grid) {
      std::size_t index = 0;
      return decisionGridIndex(key, &index) &&
             decision_grid_blocked_[index] == 0;
    }
    return isClearForVehicle(key);
  };
  const auto transition_clear = [&](const VoxelKey& from,
                                    const VoxelKey& to) {
    const int dx = to.x - from.x;
    const int dy = to.y - from.y;
    const int dz = to.z - from.z;
    for (int ix = 0; ix <= (dx != 0); ++ix) {
      for (int iy = 0; iy <= (dy != 0); ++iy) {
        for (int iz = 0; iz <= (dz != 0); ++iz) {
          const VoxelKey key{from.x + ix * dx,
                             from.y + iy * dy,
                             from.z + iz * dz};
          if (!known_free(key) || !vehicle_clear(key)) return false;
        }
      }
    }
    return true;
  };

  VoxelKey start = positionToKey(current_pose_.pose.position.x,
                                 current_pose_.pose.position.y,
                                 current_pose_.pose.position.z);
  if (!known_free(start) || !vehicle_clear(start)) {
    bool found = false;
    for (int radius = 1; radius <= 2 && !found; ++radius) {
      for (int dx = -radius; dx <= radius && !found; ++dx) {
        for (int dy = -radius; dy <= radius && !found; ++dy) {
          for (int dz = -radius; dz <= radius; ++dz) {
            const VoxelKey candidate{start.x + dx, start.y + dy, start.z + dz};
            if (known_free(candidate) && vehicle_clear(candidate)) {
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

  std::queue<VoxelKey> queue;
  reachable.reserve(std::min<std::size_t>(
      static_cast<std::size_t>(max_reachable_voxels_),
      use_decision_grid ? decision_grid_free_.size() : voxels_.size()));
  reachable.insert(start);
  if (use_decision_grid) {
    std::size_t start_index = 0;
    if (!decisionGridIndex(start, &start_index)) return VoxelSet{};
    visited[start_index] = 1;
  }
  queue.push(start);
  const double radius_squared =
      frontier_search_radius_ * frontier_search_radius_;
  const double c = std::cos(mission_heading_yaw_);
  const double s = std::sin(mission_heading_yaw_);

  while (!queue.empty() &&
         reachable.size() < static_cast<std::size_t>(max_reachable_voxels_)) {
    const VoxelKey current = queue.front();
    queue.pop();
    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dz = -1; dz <= 1; ++dz) {
          if (dx == 0 && dy == 0 && dz == 0) {
            continue;
          }
          const VoxelKey neighbor{current.x + dx, current.y + dy,
                                  current.z + dz};
          std::size_t neighbor_index = 0;
          const bool already_reached = use_decision_grid
              ? (decisionGridIndex(neighbor, &neighbor_index) &&
                 visited[neighbor_index] != 0)
              : reachable.count(neighbor) > 0;
          if (already_reached || !known_free(neighbor)) {
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
          // The fixed entry-axis band is useful before a route is chosen,
          // but it truncates a real Task-4 branch after a 90-degree bend.
          // A locked tunnel branch remains bounded by the local search and
          // home-radius checks in its goal publisher instead.
          if ((!tunnel_route_locked_ &&
               std::abs(lateral) > max_task_lateral_offset_) ||
              !missionObservationHeightAllowed(point.z, voxel_resolution_) ||
              !vehicle_clear(neighbor) ||
              !transition_clear(current, neighbor)) {
            continue;
          }
          if (use_decision_grid) visited[neighbor_index] = 1;
          reachable.insert(neighbor);
          queue.push(neighbor);
        }
      }
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
    ROS_WARN_THROTTLE(2.0, "Cannot convert Omni-LIVO PointCloud2: %s",
                      error.what());
    return;
  }
  updateMap(input, cloud.header.stamp, pose);
}

void SuperExplorationDecider::updateMap(
    const pcl::PointCloud<pcl::PointXYZ>& input,
    const ros::Time& cloud_stamp,
    const geometry_msgs::PoseStamped& pose) {
  if (input.empty()) {
    return;
  }

  // Deduplicate per scan. Thousands of points/rays in one voxel are one
  // observation, not thousands of independent confirmations. Hits win ties.
  VoxelSet hits, misses;
  const auto& origin = pose.pose.position;
  const int stride = std::max(1, static_cast<int>((input.size()+max_points_per_cloud_-1)/max_points_per_cloud_));
  for (std::size_t index=0; index<input.size(); index+=stride) {
    const auto& p=input[index];
    if (!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z)) continue;
    const double dx=p.x-origin.x, dy=p.y-origin.y, dz=p.z-origin.z;
    const double distance=std::sqrt(dx*dx+dy*dy+dz*dz);
    if (distance < voxel_resolution_*0.5 || distance > raycast_max_range_) continue;
    hits.insert(positionToKey(p.x,p.y,p.z));
  }
  if (raycast_enable_) for (std::size_t index=0; index<input.size(); index+=stride) {
    const auto& p=input[index];
    if (!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z)) continue;
    const double dx=p.x-origin.x, dy=p.y-origin.y, dz=p.z-origin.z;
    const double distance=std::sqrt(dx*dx+dy*dy+dz*dz);
    if (distance < voxel_resolution_*0.5 || distance > raycast_max_range_) continue;
    const VoxelKey endpoint=positionToKey(p.x,p.y,p.z);
    const int steps=std::max(1,static_cast<int>(std::ceil(distance/(voxel_resolution_*0.5))));
    for (int k=0;k<steps;++k) {
      const double r=double(k)/steps;
      const VoxelKey key=positionToKey(origin.x+r*dx,origin.y+r*dy,origin.z+r*dz);
      if (key==endpoint || hits.count(key)) break;
      misses.insert(key);
      // A contradictory ray may weaken the FIRST old obstruction only. Never
      // carve unseen space behind a still-blocking cell in the same scan.
      if (isOccupied(key)) break;
    }
  }
  for (const auto& key:hits) markOccupied(key);
  for (const auto& key:misses) if (!hits.count(key)) markFree(key);
  last_scan_hits_=std::move(hits);
  last_scan_stamp_=cloud_stamp;
  pruneMap();
}

void SuperExplorationDecider::updateDirectionalEvidence(
    const pcl::PointCloud<pcl::PointXYZ>& input,
    const geometry_msgs::PoseStamped& pose) {
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
  std::vector<double> left_wall_laterals;
  std::vector<double> right_wall_laterals;
  left_wall_laterals.reserve(input.size() / 4);
  right_wall_laterals.reserve(input.size() / 4);
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
    const double point_lateral = have_home_
        ? -std::sin(yaw) * (point.x - home_pose_.pose.position.x) +
              std::cos(yaw) * (point.y - home_pose_.pose.position.y)
        : -std::sin(yaw) * dx + std::cos(yaw) * dy;
    if (std::abs(normalizeAngle(relative_angle - M_PI_2)) <=
        side_half_angle) {
      left_seen = true;
      left_wall_laterals.push_back(point_lateral);
    }
    if (std::abs(normalizeAngle(relative_angle + M_PI_2)) <=
        side_half_angle) {
      right_seen = true;
      right_wall_laterals.push_back(point_lateral);
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
  double vehicle_progress = 0.0;
  if (have_home_) {
    const double vehicle_dx =
        pose.pose.position.x - home_pose_.pose.position.x;
    const double vehicle_dy =
        pose.pose.position.y - home_pose_.pose.position.y;
    vehicle_progress = std::cos(mission_heading_yaw_) * vehicle_dx +
                       std::sin(mission_heading_yaw_) * vehicle_dy;
  }
  // The first outbound leg keeps the full progress gate so structure around
  // the launch platform cannot masquerade as a dead end.  After one
  // confirmed dead end and a breadcrumb return to the origin, however, the
  // opposite branch may genuinely be shorter than that gate (the surveyed
  // Bai Xiang Shan spawn has exactly this shape).  In that state accept only
  // *live* directional evidence after one normal goal length of departure.
  // Historic map-wall evidence remains subject to the full gate below in
  // evaluateMapClosure().
  const double live_front_min_progress =
      dead_end_heading_reversal_count_ > 0
          ? std::min(front_obstacle_min_progress_, min_goal_distance_)
          : front_obstacle_min_progress_;
  const bool front_progress_allowed =
      !have_home_ || vehicle_progress >= live_front_min_progress;
  const double required_front_lateral_span =
      mission_profile_ == ExplorationMissionProfile::kGoaf
          ? goaf_front_obstacle_min_lateral_span_
          : front_obstacle_min_lateral_span_;
  const double required_front_vertical_span =
      mission_profile_ == ExplorationMissionProfile::kGoaf
          ? goaf_front_obstacle_min_vertical_span_
          : front_obstacle_min_vertical_span_;
  const bool front_seen =
      front_progress_allowed &&
      front_point_count >= front_obstacle_min_points_ &&
      front_lateral_span >= required_front_lateral_span &&
      front_vertical_span >= required_front_vertical_span;
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
  double vehicle_lateral = 0.0;
  if (have_home_) {
    const double vehicle_dx =
        pose.pose.position.x - home_pose_.pose.position.x;
    const double vehicle_dy =
        pose.pose.position.y - home_pose_.pose.position.y;
    vehicle_lateral = -std::sin(mission_heading_yaw_) * vehicle_dx +
                      std::cos(mission_heading_yaw_) * vehicle_dy;
  }
  updateCorridorCentering(left_wall_laterals, right_wall_laterals,
                          vehicle_lateral, front_seen);
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

void SuperExplorationDecider::updateCorridorCentering(
    const std::vector<double>& left_wall_laterals,
    const std::vector<double>& right_wall_laterals,
    double vehicle_lateral,
    bool front_obstacle_visible) {
  if (!corridorGuidanceEnabled() || !corridor_centering_enabled_ ||
      !have_home_) {
    resetCorridorCentering();
    return;
  }

  const CorridorCenterEstimate estimate = estimateCorridorCenter(
      left_wall_laterals, right_wall_laterals, vehicle_lateral,
      static_cast<std::size_t>(corridor_centering_min_wall_points_),
      2.0 * vehicle_radius_, corridor_centering_max_width_);
  const bool measurement_valid = estimate.valid && !front_obstacle_visible;
  const bool was_valid = corridor_center_valid_;

  if (!measurement_valid) {
    corridor_center_confirm_streak_ = 0;
    corridor_center_missing_streak_ = std::min(
        corridor_center_missing_streak_ + 1,
        corridor_centering_release_frames_);
    if (corridor_center_missing_streak_ >=
        corridor_centering_release_frames_) {
      corridor_center_valid_ = false;
    }
  } else {
    corridor_center_missing_streak_ = 0;
    if (corridor_center_valid_ &&
        std::abs(estimate.center_lateral - corridor_center_lateral_) >
            corridor_centering_max_center_jump_) {
      // A discontinuous medial-axis jump usually means a turn, branch or
      // temporary structure. Release the old center immediately and demand a
      // fresh confirmation instead of steering toward stale geometry.
      corridor_center_valid_ = false;
      corridor_center_confirm_streak_ = 0;
    }

    if (corridor_center_confirm_streak_ == 0 ||
        std::abs(estimate.center_lateral -
                 pending_corridor_center_lateral_) >
            corridor_centering_max_center_jump_) {
      pending_corridor_center_lateral_ = estimate.center_lateral;
      pending_corridor_width_ = estimate.width;
      corridor_center_confirm_streak_ = 1;
    } else {
      pending_corridor_center_lateral_ +=
          corridor_centering_filter_gain_ *
          (estimate.center_lateral - pending_corridor_center_lateral_);
      pending_corridor_width_ += corridor_centering_filter_gain_ *
          (estimate.width - pending_corridor_width_);
      corridor_center_confirm_streak_ = std::min(
          corridor_center_confirm_streak_ + 1,
          corridor_centering_confirm_frames_);
    }

    if (corridor_center_valid_) {
      corridor_center_lateral_ += corridor_centering_filter_gain_ *
          (estimate.center_lateral - corridor_center_lateral_);
      corridor_center_width_ += corridor_centering_filter_gain_ *
          (estimate.width - corridor_center_width_);
    } else if (corridor_center_confirm_streak_ >=
               corridor_centering_confirm_frames_) {
      corridor_center_valid_ = true;
      corridor_center_lateral_ = pending_corridor_center_lateral_;
      corridor_center_width_ = pending_corridor_width_;
    }
  }

  if (!was_valid && corridor_center_valid_) {
    ROS_INFO("Corridor centering enabled: center lateral %.2f m, width %.2f m",
             corridor_center_lateral_, corridor_center_width_);
  } else if (was_valid && !corridor_center_valid_) {
    ROS_INFO("Corridor centering released; using clearance-only planning");
  }
}

bool SuperExplorationDecider::corridorCenteringActive() const {
  return corridorGuidanceEnabled() && corridor_centering_enabled_ &&
      corridor_center_valid_ &&
      left_wall_visible_ && right_wall_visible_ &&
      !front_obstacle_visible_ &&
      exploration_phase_ == ExplorationPhase::kForwardPriority;
}

bool SuperExplorationDecider::corridorGuidanceEnabled() const {
  return explorationMissionCapabilities(mission_profile_).corridor_guidance;
}

double SuperExplorationDecider::minimumMissionObservationHeight() const {
  if (mission_profile_ == ExplorationMissionProfile::kTunnel) {
    return tunnel_min_planning_height_;
  }
  return home_pose_.pose.position.z + min_observation_height_above_home_;
}

double SuperExplorationDecider::maximumMissionObservationHeight() const {
  if (mission_profile_ == ExplorationMissionProfile::kTunnel) {
    return tunnel_max_planning_height_;
  }
  return home_pose_.pose.position.z + max_observation_height_above_home_;
}

double SuperExplorationDecider::preferredMissionObservationHeight() const {
  if (mission_profile_ == ExplorationMissionProfile::kTunnel) {
    // Keep each short tunnel goal near the current height. A connected route
    // may then climb or descend by max_frontier_goal_vertical_step per leg,
    // following the surveyed drift rather than returning toward launch z.
    return std::max(tunnel_min_planning_height_,
                    std::min(current_pose_.pose.position.z,
                             tunnel_max_planning_height_));
  }
  return home_pose_.pose.position.z + cruise_height_above_home_;
}

double SuperExplorationDecider::maximumReturnHeight() const {
  if (mission_profile_ == ExplorationMissionProfile::kTunnel) {
    return tunnel_max_planning_height_;
  }
  return home_pose_.pose.position.z + return_max_height_above_home_;
}

bool SuperExplorationDecider::missionObservationHeightAllowed(
    const double height, const double margin) const {
  return std::isfinite(height) && std::isfinite(margin) && margin >= 0.0 &&
         height >= minimumMissionObservationHeight() - margin &&
         height <= maximumMissionObservationHeight() + margin;
}

double SuperExplorationDecider::effectiveForwardLookaheadDistance() const {
  if (mission_profile_ == ExplorationMissionProfile::kGoaf &&
      goaf_forward_lookahead_distance_ > 0.0) {
    return goaf_forward_lookahead_distance_;
  }
  return forward_lookahead_distance_;
}

double SuperExplorationDecider::effectiveExplorationGoalQueueHorizon() const {
  if (mission_profile_ == ExplorationMissionProfile::kGoaf &&
      goaf_exploration_goal_queue_horizon_ > 0.0) {
    return goaf_exploration_goal_queue_horizon_;
  }
  return exploration_goal_queue_horizon_;
}

void SuperExplorationDecider::resetCorridorCentering() {
  corridor_center_valid_ = false;
  corridor_center_lateral_ = 0.0;
  corridor_center_width_ = 0.0;
  pending_corridor_center_lateral_ = 0.0;
  pending_corridor_width_ = 0.0;
  corridor_center_confirm_streak_ = 0;
  corridor_center_missing_streak_ = 0;
}

CorridorPreviewEstimate
SuperExplorationDecider::estimateForwardCorridorPreview(
    const VoxelSet& reachable, double current_progress,
    double current_lateral) const {
  CorridorPreviewEstimate invalid;
  if (!corridorGuidanceEnabled() || !corridor_preview_enabled_ ||
      !have_home_ || reachable.empty()) {
    return invalid;
  }
  const double lateral_min = std::max(
      -max_task_lateral_offset_,
      current_lateral - corridor_preview_lateral_half_width_);
  const double lateral_max = std::min(
      max_task_lateral_offset_,
      current_lateral + corridor_preview_lateral_half_width_);
  if (lateral_max < lateral_min) return invalid;
  const std::size_t columns = static_cast<std::size_t>(std::floor(
      (lateral_max - lateral_min) / corridor_preview_lateral_step_ + 1e-9)) +
      1;
  const std::size_t rows = static_cast<std::size_t>(std::floor(
      (corridor_preview_distance_ - corridor_preview_start_distance_) /
          corridor_preview_longitudinal_step_ + 1e-9)) + 1;
  if (columns == 0 || rows == 0) return invalid;

  const double c = std::cos(mission_heading_yaw_);
  const double s = std::sin(mission_heading_yaw_);
  const double z = home_pose_.pose.position.z + cruise_height_above_home_;
  std::vector<std::vector<bool>> passable(
      rows, std::vector<bool>(columns, false));
  for (std::size_t row = 0; row < rows; ++row) {
    const double progress = current_progress +
        corridor_preview_start_distance_ +
        row * corridor_preview_longitudinal_step_;
    for (std::size_t column = 0; column < columns; ++column) {
      const double lateral = lateral_min +
          column * corridor_preview_lateral_step_;
      const VoxelKey key = positionToKey(
          home_pose_.pose.position.x + c * progress - s * lateral,
          home_pose_.pose.position.y + s * progress + c * lateral, z);
      passable[row][column] =
          reachable.count(key) != 0 && isClearForVehicle(key);
    }
  }
  return estimateCorridorPreview(
      passable, corridor_preview_start_distance_,
      corridor_preview_longitudinal_step_, lateral_min,
      corridor_preview_lateral_step_, current_lateral,
      corridor_preview_max_lateral_slope_,
      static_cast<std::size_t>(corridor_preview_min_slices_),
      corridor_preview_min_reliable_center_band_width_);
}

void SuperExplorationDecider::updateCorridorPreviewPreference(
    const VoxelSet& reachable) {
  if (!corridorGuidanceEnabled() || !corridor_preview_enabled_ ||
      !have_home_ || returning_home_ || dead_end_backtracking_) {
    resetCorridorPreviewPreference();
    return;
  }
  const double c = std::cos(mission_heading_yaw_);
  const double s = std::sin(mission_heading_yaw_);
  const double dx = current_pose_.pose.position.x -
                    home_pose_.pose.position.x;
  const double dy = current_pose_.pose.position.y -
                    home_pose_.pose.position.y;
  const double current_progress = c * dx + s * dy;
  const double current_lateral = -s * dx + c * dy;
  const CorridorPreviewEstimate preview = estimateForwardCorridorPreview(
      reachable, current_progress, current_lateral);
  const bool active = preview.valid &&
      (std::abs(preview.center_shift) >=
           corridor_preview_min_center_shift_ ||
       preview.near_center_band_width - preview.minimum_center_band_width >=
           corridor_preview_min_width_contraction_);
  if (!active) {
    corridor_preview_missing_streak_ = std::min(
        corridor_preview_missing_streak_ + 1,
        corridor_centering_release_frames_);
    if (corridor_preview_missing_streak_ >=
        corridor_centering_release_frames_) {
      resetCorridorPreviewPreference();
    }
    return;
  }

  corridor_preview_missing_streak_ = 0;
  const double raw_preference = std::max(
      current_lateral - corridor_preview_max_goal_shift_,
      std::min(preview.target_lateral,
               current_lateral + corridor_preview_max_goal_shift_));
  if (!corridor_preview_preference_valid_) {
    corridor_preview_preference_valid_ = true;
    corridor_preview_preferred_lateral_ = current_lateral;
  }
  // Advance the preference by at most one decision voxel per cycle. This is
  // fast enough to align several metres before a throat, but a newly exposed
  // map edge cannot reverse the next goal by a metre in one update. Starting
  // a newly detected preview at the current lateral position applies the same
  // limit to the first observation instead of introducing a one-shot jump.
  const double delta = raw_preference -
                       corridor_preview_preferred_lateral_;
  const double maximum_step = voxel_resolution_;
  corridor_preview_preferred_lateral_ += std::max(
      -maximum_step, std::min(delta, maximum_step));
  ROS_INFO_THROTTLE(
      1.0,
      "Corridor preview: %.1f m connected, safe-center band %.2f->%.2f m "
      "(not wall width), lateral %.2f raw %.2f filtered %.2f",
      preview.lookahead, preview.near_center_band_width,
      preview.minimum_center_band_width, current_lateral, raw_preference,
      corridor_preview_preferred_lateral_);
}

void SuperExplorationDecider::resetCorridorPreviewPreference() {
  corridor_preview_preference_valid_ = false;
  corridor_preview_preferred_lateral_ = 0.0;
  corridor_preview_missing_streak_ = 0;
}

CorridorPreviewEstimate
SuperExplorationDecider::estimateReturnCorridorPreview(
    const geometry_msgs::Point& start, double return_heading) const {
  CorridorPreviewEstimate invalid;
  const ros::Time freshness_time = freshnessReferenceTime();
  if (!corridorGuidanceEnabled() || !corridor_preview_enabled_ ||
      super_occupied_stamp_.isZero() ||
      (freshness_time - super_occupied_stamp_).toSec() >
          return_map_timeout_) {
    return invalid;
  }

  const double lateral_min = -corridor_preview_lateral_half_width_;
  const double lateral_max = corridor_preview_lateral_half_width_;
  const std::size_t columns = static_cast<std::size_t>(std::floor(
      (lateral_max - lateral_min) / corridor_preview_lateral_step_ + 1e-9)) +
      1;
  const std::size_t rows = static_cast<std::size_t>(std::floor(
      (corridor_preview_distance_ - corridor_preview_start_distance_) /
          corridor_preview_longitudinal_step_ + 1e-9)) + 1;
  if (columns == 0 || rows == 0) return invalid;

  const double c = std::cos(return_heading);
  const double s = std::sin(return_heading);
  const double required_clearance =
      std::max(vehicle_radius_, return_super_radius_) +
      return_clearance_margin_;
  std::vector<std::vector<bool>> passable(
      rows, std::vector<bool>(columns, false));
  for (std::size_t row = 0; row < rows; ++row) {
    const double forward = corridor_preview_start_distance_ +
        row * corridor_preview_longitudinal_step_;
    for (std::size_t column = 0; column < columns; ++column) {
      const double lateral = lateral_min +
          column * corridor_preview_lateral_step_;
      geometry_msgs::Point point;
      point.x = start.x + c * forward - s * lateral;
      point.y = start.y + s * forward + c * lateral;
      point.z = start.z;
      if (!isKnownFree(positionToKey(point.x, point.y, point.z))) continue;
      if (!super_occupied_points_->empty()) {
        pcl::PointXYZ query(point.x, point.y, point.z);
        std::vector<int> indices(1);
        std::vector<float> distances(1);
        if (super_occupied_tree_.nearestKSearch(
                query, 1, indices, distances) <= 0 ||
            std::sqrt(static_cast<double>(distances.front())) + 1e-9 <
                required_clearance) {
          continue;
        }
      }
      passable[row][column] = true;
    }
  }
  return estimateCorridorPreview(
      passable, corridor_preview_start_distance_,
      corridor_preview_longitudinal_step_, lateral_min,
      corridor_preview_lateral_step_, 0.0,
      corridor_preview_max_lateral_slope_,
      static_cast<std::size_t>(corridor_preview_min_slices_),
      corridor_preview_min_reliable_center_band_width_);
}

bool SuperExplorationDecider::applyReturnCorridorPreview(
    const geometry_msgs::Point& start, geometry_msgs::Point* target) const {
  if (target == nullptr) return false;
  const double dx = target->x - start.x;
  const double dy = target->y - start.y;
  if (std::hypot(dx, dy) < 0.5 * voxel_resolution_) return false;
  const double heading = std::atan2(dy, dx);
  const CorridorPreviewEstimate preview =
      estimateReturnCorridorPreview(start, heading);
  const bool preview_active = preview.valid &&
      (std::abs(preview.center_shift) >=
           corridor_preview_min_center_shift_ ||
       preview.near_center_band_width - preview.minimum_center_band_width >=
           corridor_preview_min_width_contraction_);
  if (!preview_active) return false;

  const double shift = std::max(
      -corridor_preview_max_goal_shift_,
      std::min(preview.target_lateral, corridor_preview_max_goal_shift_));
  geometry_msgs::Point adjusted = *target;
  adjusted.x -= std::sin(heading) * shift;
  adjusted.y += std::cos(heading) * shift;
  if (!returnSegmentClear(start, adjusted)) return false;
  *target = adjusted;
  ROS_INFO_THROTTLE(
      1.0,
      "Return corridor preview: %.1f m connected, safe-center band "
      "%.2f->%.2f m, early lateral shift %.2f m",
      preview.lookahead, preview.near_center_band_width,
      preview.minimum_center_band_width, shift);
  return true;
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

bool SuperExplorationDecider::task1EndWallReturnReady(
    const ThreeWallCoverage& coverage) const {
  if (!task1_end_wall_return_enabled_ ||
      mission_profile_ != ExplorationMissionProfile::kGoaf ||
      !coverage.end_wall_found ||
      front_obstacle_streak_ < front_obstacle_confirm_frames_) {
    return false;
  }
  return coverage.end_depth - coverage.vehicle_progress <=
      task1_end_wall_return_distance_;
}

SuperExplorationDecider::MapClosureStatus
SuperExplorationDecider::evaluateMapClosure(
    const std::vector<FrontierCandidate>& candidates,
    const VoxelSet& reachable,
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
  // Global wall coverage may already contain the wall behind the aircraft
  // after a heading reversal. Do not let that old evidence block departure
  // from the mission origin in the new direction. The live front detector
  // applies the same minimum-progress gate.
  const bool live_front_boundary =
      front_obstacle_streak_ >= front_obstacle_confirm_frames_;
  const double live_front_min_progress =
      dead_end_heading_reversal_count_ > 0
          ? std::min(front_obstacle_min_progress_, min_goal_distance_)
          : front_obstacle_min_progress_;
  result.front_boundary_seen =
      (live_front_boundary &&
       result.vehicle_progress >= live_front_min_progress) ||
      (end_wall_seen &&
       result.vehicle_progress >= front_obstacle_min_progress_);

  // A transverse wall is not a terminal boundary if observed free space is
  // connected around it. Use the same vehicle-clear reachable component as
  // frontier selection, not raw free voxels behind a solid wall.
  result.max_reachable_progress = result.vehicle_progress;
  for (const auto& key : reachable) {
    const geometry_msgs::Point point = keyToPoint(key);
    const double lateral = -s * (point.x - home_pose_.pose.position.x) +
                           c * (point.y - home_pose_.pose.position.y);
    if (std::abs(lateral) > max_task_lateral_offset_ ||
        !missionObservationHeightAllowed(point.z,
                                         0.5 * voxel_resolution_) ||
        !isClearForVehicle(key)) {
      continue;
    }
    const double progress = c * (point.x - home_pose_.pose.position.x) +
                            s * (point.y - home_pose_.pose.position.y);
    result.max_reachable_progress =
        std::max(result.max_reachable_progress, progress);
  }
  // Diagnostic reachability may extend through a winding component or end in
  // a cell which cannot produce a straight, observed, SUPER-clear local leg.
  // Closure must stay open only for a route we could actually publish now.
  LocalRouteCandidate executable_forward_route;
  result.reachable_forward_passage = selectForwardLookaheadGoal(
      reachable, false, &executable_forward_route);
  if (decision_clearance_cache_active_) {
    decision_forward_route_evaluated_ = true;
    decision_forward_route_available_ = result.reachable_forward_passage;
    if (result.reachable_forward_passage) {
      decision_forward_route_ = executable_forward_route;
    }
  }

  for (const auto& entry : voxels_) {
    if (entry.second == kOccupied) {
      ++result.occupied_voxels;
    }
  }

  for (const auto& candidate : candidates) {
    // Sideways reachable frontiers at a transverse wall are still actionable:
    // they can lead around the wall even without immediate forward progress.
    if (candidateMissionProgress(candidate) >=
            result.vehicle_progress - min_goal_distance_ &&
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
  } else if ((result.occupied_voxels > map_growth_reference_count_ &&
              result.occupied_voxels - map_growth_reference_count_ >=
                  static_cast<std::size_t>(map_closure_growth_voxels_)) ||
             (result.occupied_voxels < map_growth_reference_count_ &&
              map_growth_reference_count_ - result.occupied_voxels >=
                  static_cast<std::size_t>(map_closure_growth_voxels_))) {
    last_significant_map_growth_time_ = now;
    map_growth_reference_count_ = result.occupied_voxels;
  }
  result.stable_map_duration =
      (now - last_significant_map_growth_time_).toSec();

  const bool frontier_closed =
      result.actionable_frontiers <=
          static_cast<std::size_t>(map_closure_max_actionable_frontiers_) &&
      result.no_frontier_duration >= map_closure_no_frontier_time_;
  // Once one confirmed dead end has already caused a reversal, reaching a
  // second closed boundary with no connected passage or actionable frontier
  // completes the explored component even if neither individual branch is as
  // long as the single-direction nominal progress threshold.
  const bool progress_sufficient =
      result.vehicle_progress >= map_closure_min_progress_ ||
      dead_end_heading_reversal_count_ > 0;
  // A first dead end still needs a real front boundary before it can reverse.
  // After that confirmed reversal and a return through recorded free space,
  // the opposite side can terminate at the edge of the vehicle-reachable
  // connected component even when there is no transverse wall (for example,
  // a surveyed branch which becomes disconnected by permanent structure).
  // Require departure from the origin, no reachable passage/frontier, and a
  // converged map; this is not a license to treat an arbitrary temporary map
  // gap as a wall.
  const bool reversed_component_exhausted =
      dead_end_heading_reversal_count_ > 0 &&
      result.vehicle_progress >= min_goal_distance_ &&
      !result.reachable_forward_passage && frontier_closed;
  const bool front_boundary_ok =
      !map_closure_require_front_boundary_ || result.front_boundary_seen ||
      reversed_component_exhausted;
  // A live wall after reversal is stronger evidence and need not wait for the
  // noisy binary occupied-voxel count to settle.
  const bool reversed_live_boundary_confirmed =
      dead_end_heading_reversal_count_ > 0 && live_front_boundary;
  const bool map_converged =
      result.stable_map_duration >= map_closure_stable_time_;
  result.complete = progress_sufficient &&
                    front_boundary_ok && !result.reachable_forward_passage &&
                    frontier_closed &&
                    (map_converged || reversed_live_boundary_confirmed);
  return result;
}

void SuperExplorationDecider::publishCoverageStatus(
    const ThreeWallCoverage& coverage, const MapClosureStatus& closure) {
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(2)
         << "map_closure=" << (closure.complete ? "ready" : "incomplete")
         << " front_closed="
         << (closure.front_boundary_seen ? "true" : "false")
         << " reachable_passage="
         << (closure.reachable_forward_passage ? "true" : "false")
         << " reachable_depth=" << closure.max_reachable_progress
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
      occupancy_evidence_.erase(it->first);
      it = voxels_.erase(it);
    } else {
      ++it;
    }
  }
}

void SuperExplorationDecider::carveTraversedVehicleEnvelope(
    const geometry_msgs::Point& position) {
  // This is deliberately opt-in. In a collision-enabled simulation the
  // volume currently occupied by the aircraft is known to be traversable,
  // so stale accumulated endpoints caused by pose/voxel jitter must not turn
  // that swept route into a permanent occupied barrier. Real deployments
  // keep the option disabled because localization error could otherwise
  // erase a genuine wall from the task-level map.
  const VoxelKey center = positionToKey(position.x, position.y, position.z);
  const double vertical_below = vertical_vehicle_clearance_below_ > 0.0
      ? vertical_vehicle_clearance_below_ : vertical_vehicle_radius_;
  const double vertical_above = vertical_vehicle_clearance_above_ > 0.0
      ? vertical_vehicle_clearance_above_ : vertical_vehicle_radius_;
  if (vertical_below > 0.0 && vertical_above > 0.0) {
    const double search_xy =
        vehicle_radius_ + voxel_resolution_ * 0.7071067811865476;
    const double search_z =
        std::max(vertical_below, vertical_above) +
        voxel_resolution_ * 0.5;
    const int xy_cells =
        static_cast<int>(std::ceil(search_xy / voxel_resolution_));
    const int z_cells =
        static_cast<int>(std::ceil(search_z / voxel_resolution_));
    for (int dx = -xy_cells; dx <= xy_cells; ++dx) {
      for (int dy = -xy_cells; dy <= xy_cells; ++dy) {
        for (int dz = -z_cells; dz <= z_cells; ++dz) {
          if (insideAnisotropicClearance(
                  dx, dy, dz, voxel_resolution_, vehicle_radius_,
                  vertical_below, vertical_above)) {
            voxels_[{center.x + dx, center.y + dy, center.z + dz}] = kFree;
          }
        }
      }
    }
    return;
  }

  const int radius = std::max(
      1, static_cast<int>(std::ceil(vehicle_radius_ / voxel_resolution_)));
  for (int dx = -radius; dx <= radius; ++dx) {
    for (int dy = -radius; dy <= radius; ++dy) {
      for (int dz = -radius; dz <= radius; ++dz) {
        if (dx * dx + dy * dy + dz * dz <= radius * radius) {
          voxels_[{center.x + dx, center.y + dy, center.z + dz}] = kFree;
        }
      }
    }
  }
}

bool SuperExplorationDecider::transitionClear(const VoxelKey& from, const VoxelKey& to) const {
  const int dx=to.x-from.x,dy=to.y-from.y,dz=to.z-from.z;
  if(std::abs(dx)>1||std::abs(dy)>1||std::abs(dz)>1) return false;
  // Full supercover of an adjacent-cell diagonal: validate face/edge cells,
  // not just endpoint samples which can tunnel through a voxel corner.
  for(int ix=0;ix<=(dx!=0);++ix)
    for(int iy=0;iy<=(dy!=0);++iy)
      for(int iz=0;iz<=(dz!=0);++iz) {
        const VoxelKey key{from.x+ix*dx,from.y+iy*dy,from.z+iz*dz};
        if(!isKnownFree(key)||!isClearForVehicle(key)) return false;
      }
  return true;
}

void SuperExplorationDecider::protectTraversedCenterline(
    const geometry_msgs::Point& position) {
  if (!have_last_centerline_position_) {
    last_centerline_position_ = position;
    have_last_centerline_position_ = true;
  }
  const double dx = position.x - last_centerline_position_.x;
  const double dy = position.y - last_centerline_position_.y;
  const double dz = position.z - last_centerline_position_.z;
  const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
  if (!std::isfinite(distance) || distance > 1.0) {
    have_last_centerline_position_ = false;
    return;
  }
  const int steps = std::max(
      1, static_cast<int>(std::ceil(distance / std::max(0.05, traversed_centerline_step_))));
  const int radius_cells = std::max(
      0, static_cast<int>(std::ceil(
             traversed_centerline_radius_ / voxel_resolution_)));
  for (int i = 0; i <= steps; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(steps);
    geometry_msgs::Point sample;
    sample.x = last_centerline_position_.x + dx * t;
    sample.y = last_centerline_position_.y + dy * t;
    sample.z = last_centerline_position_.z + dz * t;
    const VoxelKey center = positionToKey(sample.x, sample.y, sample.z);
    for (int ox = -radius_cells; ox <= radius_cells; ++ox) {
      for (int oy = -radius_cells; oy <= radius_cells; ++oy) {
        for (int oz = -radius_cells; oz <= radius_cells; ++oz) {
          const double r = std::sqrt(
              (ox * voxel_resolution_) * (ox * voxel_resolution_) +
              (oy * voxel_resolution_) * (oy * voxel_resolution_) +
              (oz * voxel_resolution_) * (oz * voxel_resolution_));
          if (r > traversed_centerline_radius_ + 1e-6) {
            continue;
          }
          const VoxelKey key{center.x + ox, center.y + oy, center.z + oz};
          if (isKnownFree(key)) occupancy_evidence_[key] = occupancy_evidence_min_;
        }
      }
    }
  }
  last_centerline_position_ = position;
}

bool SuperExplorationDecider::isClearForVehicle(
    const VoxelKey& key, double horizontal_radius) const {
  const double required_radius = horizontal_radius > 0.0
      ? horizontal_radius : vehicle_radius_;
  std::unordered_map<VoxelKey, bool, VoxelKeyHash>* cache = nullptr;
  if (decision_clearance_cache_active_) {
    if (std::abs(required_radius - vehicle_radius_) <= 1e-9) {
      cache = &decision_vehicle_clearance_cache_;
    } else if (std::abs(required_radius - preferred_vehicle_radius_) <=
               1e-9) {
      cache = &decision_preferred_clearance_cache_;
    }
    if (cache != nullptr) {
      const auto cached = cache->find(key);
      if (cached != cache->end()) return cached->second;
    }
  }
  const auto remember = [cache, &key](bool clear) {
    if (cache != nullptr) cache->emplace(key, clear);
    return clear;
  };
  const double vertical_below = vertical_vehicle_clearance_below_ > 0.0
      ? vertical_vehicle_clearance_below_ : vertical_vehicle_radius_;
  const double vertical_above = vertical_vehicle_clearance_above_ > 0.0
      ? vertical_vehicle_clearance_above_ : vertical_vehicle_radius_;
  if (decision_clearance_cache_active_ &&
      decision_clearance_index_ready_ &&
      std::abs(required_radius - vehicle_radius_) <= 1e-9) {
    std::size_t index = 0;
    if (decisionGridCoversClearance(key) &&
        decisionGridIndex(key, &index)) {
      return remember(decision_grid_blocked_[index] == 0);
    }
  }
  if (vertical_below > 0.0 && vertical_above > 0.0) {
    const double search_xy = required_radius +
                             voxel_resolution_ * 0.7071067811865476;
    const double search_z =
        std::max(vertical_below, vertical_above) +
        voxel_resolution_ * 0.5;
    if (decision_clearance_cache_active_ &&
        decision_clearance_index_ready_ &&
        decisionGridCoversClearance(key)) {
      if (decision_occupied_points_->empty()) return remember(true);
      const geometry_msgs::Point center = keyToPoint(key);
      pcl::PointXYZ query;
      query.x = center.x;
      query.y = center.y;
      query.z = center.z;
      decision_clearance_indices_.clear();
      decision_clearance_squared_distances_.clear();
      const double search_radius =
          std::hypot(search_xy, search_z) + voxel_resolution_ * 1e-3;
      decision_occupied_tree_.radiusSearch(
          query, search_radius, decision_clearance_indices_,
          decision_clearance_squared_distances_);
      for (const int index : decision_clearance_indices_) {
        if (index < 0 ||
            static_cast<std::size_t>(index) >=
                decision_occupied_keys_.size()) {
          continue;
        }
        const VoxelKey& occupied = decision_occupied_keys_[index];
        if (insideAnisotropicClearance(
                occupied.x - key.x, occupied.y - key.y,
                occupied.z - key.z, voxel_resolution_, required_radius,
                vertical_below, vertical_above)) {
          return remember(false);
        }
      }
      return remember(true);
    }
    const int xy_cells = static_cast<int>(std::ceil(search_xy / voxel_resolution_));
    const int z_cells = static_cast<int>(std::ceil(search_z / voxel_resolution_));
    for (int dx = -xy_cells; dx <= xy_cells; ++dx) {
      for (int dy = -xy_cells; dy <= xy_cells; ++dy) {
        for (int dz = -z_cells; dz <= z_cells; ++dz) {
          if (insideAnisotropicClearance(
                  dx, dy, dz, voxel_resolution_, required_radius,
                  vertical_below, vertical_above) &&
              isOccupied({key.x + dx, key.y + dy, key.z + dz})) {
            return remember(false);
          }
        }
      }
    }
    return remember(true);
  }
  const int radius = std::max(
      1, static_cast<int>(std::ceil(required_radius / voxel_resolution_)));
  if (decision_clearance_cache_active_ &&
      decision_clearance_index_ready_ &&
      decisionGridCoversClearance(key)) {
    if (decision_occupied_points_->empty()) return remember(true);
    const geometry_msgs::Point center = keyToPoint(key);
    pcl::PointXYZ query;
    query.x = center.x;
    query.y = center.y;
    query.z = center.z;
    decision_clearance_indices_.clear();
    decision_clearance_squared_distances_.clear();
    decision_occupied_tree_.radiusSearch(
        query, radius * voxel_resolution_ + voxel_resolution_ * 1e-3,
        decision_clearance_indices_,
        decision_clearance_squared_distances_);
    for (const int index : decision_clearance_indices_) {
      if (index < 0 ||
          static_cast<std::size_t>(index) >=
              decision_occupied_keys_.size()) {
        continue;
      }
      const VoxelKey& occupied = decision_occupied_keys_[index];
      const int dx = occupied.x - key.x;
      const int dy = occupied.y - key.y;
      const int dz = occupied.z - key.z;
      if (dx * dx + dy * dy + dz * dz <= radius * radius) {
        return remember(false);
      }
    }
    return remember(true);
  }
  for (int dx = -radius; dx <= radius; ++dx) {
    for (int dy = -radius; dy <= radius; ++dy) {
      for (int dz = -radius; dz <= radius; ++dz) {
        if (dx * dx + dy * dy + dz * dz > radius * radius) {
          continue;
        }
        if (isOccupied({key.x + dx, key.y + dy, key.z + dz})) {
          return remember(false);
        }
      }
    }
  }
  return remember(true);
}

bool SuperExplorationDecider::decisionGridIndex(
    const VoxelKey& key, std::size_t* index) const {
  if (decision_grid_size_x_ <= 0 || decision_grid_size_y_ <= 0 ||
      decision_grid_size_z_ <= 0) {
    return false;
  }
  const int x = key.x - decision_grid_min_.x;
  const int y = key.y - decision_grid_min_.y;
  const int z = key.z - decision_grid_min_.z;
  if (x < 0 || x >= decision_grid_size_x_ ||
      y < 0 || y >= decision_grid_size_y_ ||
      z < 0 || z >= decision_grid_size_z_) {
    return false;
  }
  if (index != nullptr) {
    *index = (static_cast<std::size_t>(x) * decision_grid_size_y_ + y) *
                 decision_grid_size_z_ +
             z;
  }
  return true;
}

bool SuperExplorationDecider::decisionGridCoversClearance(
    const VoxelKey& key) const {
  return decision_clearance_index_ready_ &&
         std::abs(key.x - decision_grid_center_.x) <=
             decision_grid_core_radius_cells_ &&
         std::abs(key.y - decision_grid_center_.y) <=
             decision_grid_core_radius_cells_ &&
         std::abs(key.z - decision_grid_center_.z) <=
             decision_grid_core_radius_cells_;
}

void SuperExplorationDecider::rebuildDecisionClearanceIndex() {
  decision_clearance_index_ready_ = false;
  decision_occupied_points_->clear();
  decision_occupied_keys_.clear();
  decision_grid_free_.clear();
  decision_grid_blocked_.clear();
  decision_grid_size_x_ = 0;
  decision_grid_size_y_ = 0;
  decision_grid_size_z_ = 0;
  decision_occupied_points_->reserve(voxels_.size());
  decision_occupied_keys_.reserve(voxels_.size());

  std::vector<VoxelKey> hard_clearance_stencil;
  const double vertical_below = vertical_vehicle_clearance_below_ > 0.0
      ? vertical_vehicle_clearance_below_ : vertical_vehicle_radius_;
  const double vertical_above = vertical_vehicle_clearance_above_ > 0.0
      ? vertical_vehicle_clearance_above_ : vertical_vehicle_radius_;
  if (vertical_below > 0.0 && vertical_above > 0.0) {
    const double search_xy =
        vehicle_radius_ + voxel_resolution_ * 0.7071067811865476;
    const double search_z =
        std::max(vertical_below, vertical_above) + voxel_resolution_ * 0.5;
    const int xy_cells =
        static_cast<int>(std::ceil(search_xy / voxel_resolution_));
    const int z_cells =
        static_cast<int>(std::ceil(search_z / voxel_resolution_));
    for (int dx = -xy_cells; dx <= xy_cells; ++dx) {
      for (int dy = -xy_cells; dy <= xy_cells; ++dy) {
        for (int dz = -z_cells; dz <= z_cells; ++dz) {
          if (insideAnisotropicClearance(
                  dx, dy, dz, voxel_resolution_, vehicle_radius_,
                  vertical_below, vertical_above)) {
            hard_clearance_stencil.push_back({dx, dy, dz});
          }
        }
      }
    }
  } else {
    const int radius = std::max(
        1, static_cast<int>(std::ceil(
               vehicle_radius_ / voxel_resolution_)));
    for (int dx = -radius; dx <= radius; ++dx) {
      for (int dy = -radius; dy <= radius; ++dy) {
        for (int dz = -radius; dz <= radius; ++dz) {
          if (dx * dx + dy * dy + dz * dz <= radius * radius) {
            hard_clearance_stencil.push_back({dx, dy, dz});
          }
        }
      }
    }
  }
  decision_grid_center_ = positionToKey(
      current_pose_.pose.position.x, current_pose_.pose.position.y,
      current_pose_.pose.position.z);
  decision_grid_core_radius_cells_ = std::max(
      1, static_cast<int>(std::ceil(
             frontier_search_radius_ / voxel_resolution_)));
  const double preferred_search_xy =
      preferred_vehicle_radius_ + voxel_resolution_ * 0.7071067811865476;
  const double preferred_search_z =
      std::max(vertical_below, vertical_above) + voxel_resolution_ * 0.5;
  const int padding_cells = std::max(
      1, static_cast<int>(std::ceil(
             std::max(preferred_search_xy, preferred_search_z) /
             voxel_resolution_)) + 1);
  const int selection_radius =
      decision_grid_core_radius_cells_ + padding_cells;

  bool have_bounds = false;
  VoxelKey maximum;
  const auto extend_bounds = [&](const VoxelKey& key) {
    if (!have_bounds) {
      decision_grid_min_ = key;
      maximum = key;
      have_bounds = true;
      return;
    }
    decision_grid_min_.x = std::min(decision_grid_min_.x, key.x);
    decision_grid_min_.y = std::min(decision_grid_min_.y, key.y);
    decision_grid_min_.z = std::min(decision_grid_min_.z, key.z);
    maximum.x = std::max(maximum.x, key.x);
    maximum.y = std::max(maximum.y, key.y);
    maximum.z = std::max(maximum.z, key.z);
  };
  extend_bounds(decision_grid_center_);
  for (const auto& entry : voxels_) {
    const VoxelKey& key = entry.first;
    if (std::abs(key.x - decision_grid_center_.x) > selection_radius ||
        std::abs(key.y - decision_grid_center_.y) > selection_radius ||
        std::abs(key.z - decision_grid_center_.z) > selection_radius) {
      continue;
    }
    extend_bounds(key);
  }
  // Preserve the full clearance padding even when no known voxel happens to
  // occupy one side of the local map bounds.
  decision_grid_min_.x -= padding_cells;
  decision_grid_min_.y -= padding_cells;
  decision_grid_min_.z -= padding_cells;
  maximum.x += padding_cells;
  maximum.y += padding_cells;
  maximum.z += padding_cells;
  decision_grid_size_x_ = maximum.x - decision_grid_min_.x + 1;
  decision_grid_size_y_ = maximum.y - decision_grid_min_.y + 1;
  decision_grid_size_z_ = maximum.z - decision_grid_min_.z + 1;
  const std::size_t grid_size =
      static_cast<std::size_t>(decision_grid_size_x_) *
      decision_grid_size_y_ * decision_grid_size_z_;
  decision_grid_free_.assign(grid_size, 0);
  decision_grid_blocked_.assign(grid_size, 0);

  for (const auto& entry : voxels_) {
    std::size_t index = 0;
    if (!decisionGridIndex(entry.first, &index)) continue;
    if (entry.second == kFree) decision_grid_free_[index] = 1;
    if (entry.second != kOccupied && entry.second != kPendingOccupied) {
      continue;
    }
    const geometry_msgs::Point center = keyToPoint(entry.first);
    decision_occupied_points_->push_back(
        pcl::PointXYZ(center.x, center.y, center.z));
    decision_occupied_keys_.push_back(entry.first);
  }

  for (const VoxelKey& occupied : decision_occupied_keys_) {
    for (const VoxelKey& offset : hard_clearance_stencil) {
      const VoxelKey blocked{occupied.x - offset.x,
                             occupied.y - offset.y,
                             occupied.z - offset.z};
      std::size_t index = 0;
      if (decisionGridIndex(blocked, &index)) {
        decision_grid_blocked_[index] = 1;
      }
    }
  }

  decision_occupied_points_->width = decision_occupied_points_->size();
  decision_occupied_points_->height = 1;
  decision_occupied_points_->is_dense = true;
  if (!decision_occupied_points_->empty()) {
    decision_occupied_tree_.setInputCloud(decision_occupied_points_);
  }
  decision_clearance_index_ready_ = true;
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

bool SuperExplorationDecider::isNearFailedGoal(
    const geometry_msgs::Point& point) const {
  const double radius = std::max(0.75, 2.0 * voxel_resolution_);
  for (const auto& failed : failed_goals_) {
    if (squaredDistance(point, failed) < radius * radius) return true;
  }
  return false;
}

void SuperExplorationDecider::rememberFailedGoal(
    const geometry_msgs::Point& point) {
  if (isNearFailedGoal(point)) return;
  failed_goals_.push_back(point);
  if (failed_goals_.size() > 100) failed_goals_.erase(failed_goals_.begin());
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

  // Reachability has already filtered the task map down to connected,
  // hard-clear free voxels. Iterating the entire (mostly occupied, pending,
  // or disconnected) map doubled large-flight frontier latency and then
  // repeated the same reachability/clearance hash lookups.
  for (const VoxelKey& key : reachable) {
    const geometry_msgs::Point point = keyToPoint(key);
    if (have_home_) {
      const double home_dx = point.x - home_pose_.pose.position.x;
      const double home_dy = point.y - home_pose_.pose.position.y;
      if (max_exploration_radius_from_home_ > 0.0) {
        const double max_home_distance_squared =
            max_exploration_radius_from_home_ *
            max_exploration_radius_from_home_;
        if (home_dx * home_dx + home_dy * home_dy >
            max_home_distance_squared) {
          continue;
        }
      }
      const double min_goal_z = minimumMissionObservationHeight();
      const double max_goal_z = maximumMissionObservationHeight();
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
        isNearCoveredGoal(point) || isNearFailedGoal(point)) {
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
          const VoxelKey neighbor{key.x + dx, key.y + dy, key.z + dz};
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
    candidate.key = key;
    candidate.goal.header.frame_id = world_frame_;
    candidate.goal.header.stamp = ros::Time::now();
    candidate.goal.pose.position = point;
    const double yaw = (std::abs(direction_x) + std::abs(direction_y) > 1e-6)
                           ? std::atan2(direction_y, direction_x)
                           : std::atan2(current_pose_.pose.position.y - point.y,
                                        current_pose_.pose.position.x - point.x);
    candidate.goal.pose.orientation = yawQuaternion(yaw);
    candidate.unknown_neighbors = unknown_neighbors;
    candidate.preferred_clearance =
        isClearForVehicle(key, preferred_vehicle_radius_);
    candidate.score = information_weight_ * unknown_neighbors -
                      distance_weight_ * std::sqrt(distance_squared);
    candidates.push_back(candidate);
  }

  std::sort(candidates.begin(), candidates.end(),
            [](const FrontierCandidate& left, const FrontierCandidate& right) {
              if (std::abs(left.score - right.score) > 1e-9) {
                return left.score > right.score;
              }
              return left.preferred_clearance &&
                     !right.preferred_clearance;
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
  double best_center_error = std::numeric_limits<double>::infinity();
  const double current_dx =
      current_pose_.pose.position.x - home_pose_.pose.position.x;
  const double current_dy =
      current_pose_.pose.position.y - home_pose_.pose.position.y;
  const double current_progress = std::cos(mission_heading_yaw_) * current_dx +
                                  std::sin(mission_heading_yaw_) * current_dy;
  const double preferred_height = preferredMissionObservationHeight();
  const bool side_structure_visible =
      left_wall_visible_ || right_wall_visible_;
  const bool center_on_medial_axis = corridorCenteringActive();
  const double preferred_lateral = center_on_medial_axis
      ? corridor_center_lateral_ : 0.0;
  for (const auto& candidate : candidates) {
    if (!isForwardCandidate(candidate)) {
      continue;
    }
    const double progress = candidateMissionProgress(candidate);
    const double progress_ahead = progress - current_progress;
    const double mission_lateral = candidateMissionLateral(candidate);
    const double lateral = std::abs(mission_lateral - preferred_lateral);
    if (progress_ahead < min_goal_distance_ ||
        (side_structure_visible &&
         std::abs(mission_lateral) > forward_corridor_half_width_)) {
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
    const bool better = best == nullptr ||
        (center_on_medial_axis &&
         (lateral < best_center_error - 1e-9 ||
          (std::abs(lateral - best_center_error) <= 1e-9 &&
           score > best_score + 1e-9))) ||
        (!center_on_medial_axis &&
         ((candidate.preferred_clearance && !best->preferred_clearance) ||
          (candidate.preferred_clearance == best->preferred_clearance &&
           score > best_score + 1e-9)));
    if (better) {
      best_score = score;
      best_center_error = lateral;
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
  active_goal_is_lateral_detour_ =
      reason.compare(0, std::string("lateral_detour").size(),
                     "lateral_detour") == 0;
  // Diagnostic suffixes must not change the handover semantics of this whole
  // short-segment goal family.
  active_goal_is_forward_lookahead_ =
      reason.compare(0, std::string("forward_lookahead").size(),
                     "forward_lookahead") == 0;
  active_goal_is_blocked_frontier_route_ =
      reason == "blocked_frontier_route" ||
      reason == "connected_turn_route";
  active_goal_allows_rolling_handover_ =
      active_goal_is_forward_lookahead_ ||
      reason == "heading_priority" || reason == "frontier_fallback" ||
      active_goal_is_blocked_frontier_route_ ||
      reason == "frontier_route_queued_successor";
  active_goal_commits_tunnel_heading_ = false;
  active_goal_tunnel_heading_yaw_ = 0.0;
  active_goal_updates_exploration_heading_ = false;
  active_goal_exploration_heading_yaw_ = 0.0;
  if (!returning_home_ && !dead_end_backtracking_) {
    const double route_dx = goal.pose.position.x -
                            current_pose_.pose.position.x;
    const double route_dy = goal.pose.position.y -
                            current_pose_.pose.position.y;
    if (std::hypot(route_dx, route_dy) > 0.5 * voxel_resolution_) {
      const bool explicit_tunnel_route =
          reason == "connected_turn_route";
      // A connected path can travel mostly forward before ending in a sharp
      // bend.  Its aircraft-to-goal chord therefore understates the turn (34
      // degrees versus a 90-degree tangent in the 2026-10-25 flight).  Commit
      // Task 4 to the verified endpoint tangent carried in the goal pose.
      const double route_heading = explicit_tunnel_route
          ? poseYaw(goal.pose) : std::atan2(route_dy, route_dx);
      const double reference_heading = tunnel_route_locked_
          ? tunnel_route_heading_yaw_
          : (have_last_exploration_route_heading_
                 ? last_exploration_route_heading_yaw_
                 : mission_heading_yaw_);
      const bool branch_selecting_route =
          reason == "frontier_fallback" ||
          reason == "blocked_frontier_route" ||
          reason == "frontier_route_queued_successor" ||
          reason.compare(0, std::string("lateral_detour").size(),
                         "lateral_detour") == 0;
      const double turn = std::abs(normalizeAngle(
          route_heading - reference_heading));
      if (mission_profile_ == ExplorationMissionProfile::kTunnel &&
          (tunnel_route_locked_ || explicit_tunnel_route ||
           (branch_selecting_route &&
            turn >= tunnel_route_lock_min_turn_deg_ * M_PI / 180.0))) {
        // Publication is not commitment: a rejected SUPER goal must never
        // alter the route reference used by the next planning cycle.
        active_goal_commits_tunnel_heading_ = true;
        active_goal_tunnel_heading_yaw_ = route_heading;
      }
      // A published candidate is not route history. Keep it pending until
      // SUPER acknowledges this exact goal id; rejection/cancellation must
      // leave the last executable tangent untouched.
      active_goal_updates_exploration_heading_ = true;
      active_goal_exploration_heading_yaw_ = route_heading;
    }
  }
  super_planner::GoalCommand command;
  command.header = current_goal_.header;
  command.command = super_planner::GoalCommand::SET_GOAL;
  command.goal_id = next_goal_id_++;
  command.goal = current_goal_.pose;
  command.reason = reason;
  goal_publisher_.publish(command);
  active_goal_id_ = command.goal_id;
  accepted_goal_id_ = 0;
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
    clearExplorationGoalQueue();
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
  accepted_goal_id_ = 0;
  have_active_goal_ = false;
  active_goal_is_end_approach_ = false;
  active_goal_is_lateral_detour_ = false;
  active_goal_is_forward_lookahead_ = false;
  active_goal_is_blocked_frontier_route_ = false;
  active_goal_allows_rolling_handover_ = false;
  active_goal_commits_tunnel_heading_ = false;
  active_goal_tunnel_heading_yaw_ = 0.0;
  active_goal_updates_exploration_heading_ = false;
  active_goal_exploration_heading_yaw_ = 0.0;
  clearExplorationGoalQueue();
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
  goal.pose.position.z = preferredMissionObservationHeight();
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

bool SuperExplorationDecider::selectForwardLookaheadGoal(
    const VoxelSet& reachable, bool require_successor,
    LocalRouteCandidate* selected) const {
  // Frontier fallback controls how unstructured frontier candidates are
  // ranked; it must not suppress a directly observed, vehicle-clear forward
  // segment. Side-wall visibility can disappear in a surveyed junction while
  // the centerline remains safely reachable.
  if (!have_home_) {
    return false;
  }

  const double c = std::cos(mission_heading_yaw_);
  const double s = std::sin(mission_heading_yaw_);
  const double current_dx =
      current_pose_.pose.position.x - home_pose_.pose.position.x;
  const double current_dy =
      current_pose_.pose.position.y - home_pose_.pose.position.y;
  const double current_progress = c * current_dx + s * current_dy;
  double maximum_lookahead = effectiveForwardLookaheadDistance();
  if (max_exploration_radius_from_home_ > 0.0) {
    const double remaining_radius =
        max_exploration_radius_from_home_ - current_progress;
    maximum_lookahead = std::min(maximum_lookahead, remaining_radius);
  }
  // After the first confirmed dead end, live-map return can end at the
  // launch voxel with less than the normal two-metre connected free segment
  // visible in the opposite direction. Permit one-voxel observation steps
  // to leave that already traversed origin envelope and obtain a fresh scan.
  // The initial outbound leg retains the normal minimum goal distance.
  const double minimum_lookahead = dead_end_heading_reversal_count_ > 0
      ? std::min(min_goal_distance_, voxel_resolution_)
      : min_goal_distance_;
  if (maximum_lookahead < minimum_lookahead) {
    return false;
  }

  const double current_lateral = -s * current_dx + c * current_dy;
  const bool center_on_medial_axis = corridorCenteringActive();
  double preferred_lateral = 0.0;
  if (center_on_medial_axis) {
    preferred_lateral = std::max(
        current_lateral - corridor_centering_max_goal_shift_,
        std::min(corridor_center_lateral_,
                 current_lateral + corridor_centering_max_goal_shift_));
  }
  const bool preview_active = corridor_preview_preference_valid_;
  if (preview_active) {
    preferred_lateral = std::max(
        current_lateral - corridor_preview_max_goal_shift_,
        std::min(corridor_preview_preferred_lateral_,
                 current_lateral + corridor_preview_max_goal_shift_));
  }
  const bool use_geometric_preference =
      preview_active || center_on_medial_axis;
  std::vector<double> lateral_offsets = centeredLateralOffsets(
      preferred_lateral, forward_goal_lateral_search_width_,
      voxel_resolution_);
  const double preferred_height = preferredMissionObservationHeight();
  const double minimum_height = minimumMissionObservationHeight();
  const double maximum_height = maximumMissionObservationHeight();
  std::vector<double> height_offsets{0.0};
  for (double offset = voxel_resolution_;
       preferred_height - offset >= minimum_height - 1e-6 ||
           preferred_height + offset <= maximum_height + 1e-6;
       offset += voxel_resolution_) {
    if (preferred_height - offset >= minimum_height - 1e-6) {
      height_offsets.push_back(-offset);
    }
    if (preferred_height + offset <= maximum_height + 1e-6) {
      height_offsets.push_back(offset);
    }
  }
  bool have_hard_fallback = false;
  LocalRouteCandidate hard_fallback;

  // Preserve the task intent of preferring a level, longer observation step,
  // but evaluate every lateral alternative at that distance before choosing.
  // This avoids the old centre,+left,+right first-feasible bias.
  for (const double height_offset : height_offsets) {
    for (double lookahead = maximum_lookahead;
         lookahead >= minimum_lookahead - 1e-6;
         lookahead -= forward_lookahead_step_) {
      std::vector<LocalRouteCandidate> feasible;
      for (const double lateral : lateral_offsets) {
        const double target_progress = current_progress + lookahead;
        if (require_successor &&
            target_progress <=
                std::cos(mission_heading_yaw_) *
                    (current_goal_.pose.position.x - home_pose_.pose.position.x) +
                std::sin(mission_heading_yaw_) *
                    (current_goal_.pose.position.y - home_pose_.pose.position.y) +
                0.5 * voxel_resolution_) {
          continue;
        }
        geometry_msgs::Point target;
        target.x = home_pose_.pose.position.x + c * target_progress -
                   s * lateral;
        target.y = home_pose_.pose.position.y + s * target_progress +
                   c * lateral;
        target.z = preferred_height + height_offset;
        const VoxelKey target_key = positionToKey(target.x, target.y, target.z);
        // A task-level goal must be proven free and connected by a clear,
        // observed segment. Unknown is not free: allowing a centerline goal
        // merely because it is not occupied can place the goal behind a wall.
        if (isNearFailedGoal(target) || reachable.count(target_key) == 0 ||
            !isClearForVehicle(target_key)) {
          continue;
        }
        // A confirmed wall can be inside the detector's six-metre range while
        // several metres of safe passage still remain in front of it.  The
        // old global front-wall gate refused every such goal, so after the
        // first dead-end backtrack the aircraft waited at the origin forever.
        // Permit a short approach only when every half-voxel sample on the
        // straight segment is both observed-reachable and body-clear.  This
        // advances the viewpoint without ever commanding through that wall.
        const auto& start = current_pose_.pose.position;
        const double segment_distance = std::sqrt(
            squaredDistance(start, target));
        const int segment_samples = std::max(
            1, static_cast<int>(std::ceil(
                   segment_distance / (0.5 * voxel_resolution_))));
        bool segment_clear = true;
        bool preferred_segment_clear = isClearForVehicle(
            target_key, preferred_vehicle_radius_);
        for (int sample = 1; sample <= segment_samples; ++sample) {
          const double ratio =
              static_cast<double>(sample) / segment_samples;
          const VoxelKey sample_key = positionToKey(
              start.x + ratio * (target.x - start.x),
              start.y + ratio * (target.y - start.y),
              start.z + ratio * (target.z - start.z));
          if (reachable.count(sample_key) == 0 ||
              !isClearForVehicle(sample_key)) {
            segment_clear = false;
            break;
          }
          if (preferred_segment_clear &&
              !isClearForVehicle(sample_key, preferred_vehicle_radius_)) {
            preferred_segment_clear = false;
          }
        }
        if (!segment_clear ||
            !returnSegmentClear(start, target, 0.0, false)) {
          continue;
        }
        LocalRouteCandidate route;
        route.goal.header.frame_id = world_frame_;
        route.goal.pose.position = target;
        const double route_dx = target.x - start.x;
        const double route_dy = target.y - start.y;
        const double route_yaw = std::hypot(route_dx, route_dy) > 1e-6
            ? std::atan2(route_dy, route_dx) : mission_heading_yaw_;
        route.goal.pose.orientation = yawQuaternion(route_yaw);
        const double preferred_extra_clearance = std::max(
            0.0, preferred_vehicle_radius_ -
                     (std::max(vehicle_radius_, return_super_radius_) +
                      return_clearance_margin_));
        route.preferred_clearance =
            preferred_segment_clear &&
            returnSegmentClear(start, target,
                               preferred_extra_clearance, false);
        route.mission_lateral = lateral;
        route.lookahead = lookahead;
        route.minimum_clearance = minimumSuperSegmentClearance(start, target);
        feasible.push_back(route);
      }

      if (feasible.empty()) {
        continue;
      }

      auto side_of = [this](double lateral) {
        if (lateral > 0.5 * voxel_resolution_) return 1;
        if (lateral < -0.5 * voxel_resolution_) return -1;
        return 0;
      };
      auto clearance_better = [](double lhs, double rhs) {
        if (!std::isfinite(lhs)) return std::isfinite(rhs);
        if (!std::isfinite(rhs)) return false;
        return lhs > rhs + 1e-9;
      };
      const auto center_error = [preferred_lateral](
                                    const LocalRouteCandidate& route) {
        return std::abs(route.mission_lateral - preferred_lateral);
      };

      // Stable two-sided wall evidence makes the measured medial axis the
      // primary geometric preference. Comfort clearance only breaks a tie;
      // it must not pull a hard-safe route away from the corridor centre.
      if (use_geometric_preference) {
        const LocalRouteCandidate* centered = &feasible.front();
        for (const auto& route : feasible) {
          const double route_error = center_error(route);
          const double best_error = center_error(*centered);
          if (route_error < best_error - 1e-9 ||
              (std::abs(route_error - best_error) <= 1e-9 &&
               route.preferred_clearance &&
               !centered->preferred_clearance) ||
              (std::abs(route_error - best_error) <= 1e-9 &&
               route.preferred_clearance == centered->preferred_clearance &&
               clearance_better(route.minimum_clearance,
                                centered->minimum_clearance))) {
            centered = &route;
          }
        }
        *selected = *centered;
        return true;
      }

      // Without a reliable two-wall centre (including one-sided wall
      // observations), prefer a complete 1.1 m-clear path. Only after every
      // height and look-ahead candidate has failed that envelope may the
      // 0.8--1.1 m hard-safe band be used.
      const bool preferred_available = std::any_of(
          feasible.begin(), feasible.end(),
          [](const LocalRouteCandidate& route) {
            return route.preferred_clearance;
          });
      const auto eligible = [preferred_available](
                                const LocalRouteCandidate& route) {
        return !preferred_available || route.preferred_clearance;
      };
      auto tie_better = [](const LocalRouteCandidate& lhs,
                           const LocalRouteCandidate& rhs) {
        return std::abs(lhs.mission_lateral) <
               std::abs(rhs.mission_lateral) - 1e-9;
      };

      const LocalRouteCandidate* clearest = nullptr;
      for (const auto& route : feasible) {
        if (!eligible(route)) continue;
        if (clearest == nullptr ||
            clearance_better(route.minimum_clearance,
                             clearest->minimum_clearance) ||
            (!clearance_better(clearest->minimum_clearance,
                               route.minimum_clearance) &&
             tie_better(route, *clearest))) {
          clearest = &route;
        }
      }

      // Side holding is a clearance hysteresis, not a permanent preference:
      // retain the previous side only while its worst point is within the
      // configured margin of the globally clearest eligible path.
      const LocalRouteCandidate* held = nullptr;
      if (held_route_side_ != 0) {
        for (const auto& route : feasible) {
          if (!eligible(route) ||
              side_of(route.mission_lateral) != held_route_side_) continue;
          const bool close_enough =
              (!std::isfinite(clearest->minimum_clearance) &&
               !std::isfinite(route.minimum_clearance)) ||
              (std::isfinite(clearest->minimum_clearance) &&
               route.minimum_clearance +
                       lateral_route_switch_clearance_advantage_ >=
                   clearest->minimum_clearance);
          if (!close_enough) continue;
          if (held == nullptr ||
              clearance_better(route.minimum_clearance,
                               held->minimum_clearance) ||
              (!clearance_better(held->minimum_clearance,
                                 route.minimum_clearance) &&
               tie_better(route, *held))) {
            held = &route;
          }
        }
      }
      *selected = held != nullptr ? *held : *clearest;
      if (preferred_available) return true;
      if (!have_hard_fallback) {
        hard_fallback = *selected;
        have_hard_fallback = true;
      }
    }
  }
  if (have_hard_fallback) {
    *selected = hard_fallback;
    return true;
  }
  return false;
}

void SuperExplorationDecider::clearExplorationGoalQueue() {
  exploration_goal_queue_.clear();
}

bool SuperExplorationDecider::queuedExplorationGoalValid(
    const QueuedExplorationGoal& queued, const VoxelSet& reachable,
    bool validate_from_current_pose) const {
  if (!exploration_goal_queue_enabled_ || !have_home_ ||
      queued.route.empty()) {
    return false;
  }
  const geometry_msgs::Point& target = queued.goal.pose.position;
  const VoxelKey target_key = positionToKey(target.x, target.y, target.z);
  if (reachable.count(target_key) == 0 || !isClearForVehicle(target_key) ||
      isNearFailedGoal(target)) {
    return false;
  }

  const double c = std::cos(mission_heading_yaw_);
  const double s = std::sin(mission_heading_yaw_);
  const double dx = target.x - home_pose_.pose.position.x;
  const double dy = target.y - home_pose_.pose.position.y;
  const double progress = c * dx + s * dy;
  const double lateral = -s * dx + c * dy;
  if ((max_exploration_radius_from_home_ > 0.0 &&
       progress > max_exploration_radius_from_home_ + 1e-6) ||
      std::abs(lateral) > max_task_lateral_offset_ + 1e-6 ||
      !missionObservationHeightAllowed(target.z,
                                       0.5 * voxel_resolution_)) {
    return false;
  }

  for (const VoxelKey& key : queued.route) {
    if (reachable.count(key) == 0 || !isClearForVehicle(key)) return false;
  }

  if (queued.direct_segment) {
    const geometry_msgs::Point& start = validate_from_current_pose
        ? current_pose_.pose.position : keyToPoint(queued.route.front());
    return returnSegmentClear(start, target, 0.0, false);
  }

  // A frontier route may bend. Revalidate every cached edge instead of
  // replacing it by a chord that could cut through the inside wall.
  for (std::size_t index = 1; index < queued.route.size(); ++index) {
    if (!returnSegmentClear(keyToPoint(queued.route[index - 1]),
                            keyToPoint(queued.route[index]), 0.0, false)) {
      return false;
    }
  }
  return returnSegmentClear(target, target, 0.0, false);
}

void SuperExplorationDecider::pruneExplorationGoalQueue(
    const VoxelSet& reachable) {
  for (std::size_t index = 0; index < exploration_goal_queue_.size();
       ++index) {
    if (!queuedExplorationGoalValid(exploration_goal_queue_[index], reachable,
                                    false)) {
      const std::size_t removed = exploration_goal_queue_.size() - index;
      exploration_goal_queue_.erase(exploration_goal_queue_.begin() + index,
                                    exploration_goal_queue_.end());
      ROS_WARN_THROTTLE(
          1.0,
          "Dropped %zu queued exploration goal(s): latest map no longer "
          "certifies the next complete leg",
          removed);
      return;
    }
  }
}

void SuperExplorationDecider::buildForwardExplorationQueue(
    const VoxelSet& reachable, const LocalRouteCandidate& active_goal) {
  clearExplorationGoalQueue();
  if (!exploration_goal_queue_enabled_ ||
      exploration_goal_queue_max_future_goals_ <= 0 || !have_home_) {
    return;
  }

  const double c = std::cos(mission_heading_yaw_);
  const double s = std::sin(mission_heading_yaw_);
  const double current_dx =
      current_pose_.pose.position.x - home_pose_.pose.position.x;
  const double current_dy =
      current_pose_.pose.position.y - home_pose_.pose.position.y;
  const double current_progress = c * current_dx + s * current_dy;
  const double usable_horizon = std::max(
      active_goal.lookahead,
      effectiveExplorationGoalQueueHorizon() -
          exploration_goal_queue_end_reserve_);
  double maximum_progress = current_progress + usable_horizon;
  if (max_exploration_radius_from_home_ > 0.0) {
    maximum_progress =
        std::min(maximum_progress, max_exploration_radius_from_home_);
  }

  geometry_msgs::Point previous = active_goal.goal.pose.position;
  double next_progress = current_progress + active_goal.lookahead +
                         exploration_goal_queue_spacing_;
  while (next_progress <= maximum_progress + 1e-6 &&
         exploration_goal_queue_.size() <
             static_cast<std::size_t>(
                 exploration_goal_queue_max_future_goals_)) {
    geometry_msgs::Point target;
    target.x = home_pose_.pose.position.x + c * next_progress -
               s * active_goal.mission_lateral;
    target.y = home_pose_.pose.position.y + s * next_progress +
               c * active_goal.mission_lateral;
    target.z = active_goal.goal.pose.position.z;

    QueuedExplorationGoal queued;
    queued.goal.header.frame_id = world_frame_;
    queued.goal.pose.position = target;
    queued.goal.pose.orientation = yawQuaternion(
        std::atan2(target.y - previous.y, target.x - previous.x));
    queued.direct_segment = true;
    queued.frontier_route = false;

    const double distance = std::sqrt(squaredDistance(previous, target));
    const int samples = std::max(
        1, static_cast<int>(std::ceil(
               distance / (0.5 * voxel_resolution_))));
    bool task_segment_clear = true;
    bool preferred_segment_clear = true;
    for (int sample = 0; sample <= samples; ++sample) {
      const double ratio = static_cast<double>(sample) / samples;
      const VoxelKey key = positionToKey(
          previous.x + ratio * (target.x - previous.x),
          previous.y + ratio * (target.y - previous.y),
          previous.z + ratio * (target.z - previous.z));
      if (queued.route.empty() || !(queued.route.back() == key)) {
        queued.route.push_back(key);
      }
      if (reachable.count(key) == 0 || !isClearForVehicle(key)) {
        task_segment_clear = false;
        break;
      }
      if (preferred_segment_clear &&
          !isClearForVehicle(key, preferred_vehicle_radius_)) {
        preferred_segment_clear = false;
      }
    }

    geometry_msgs::Point reserve_end = target;
    reserve_end.x += c * exploration_goal_queue_end_reserve_;
    reserve_end.y += s * exploration_goal_queue_end_reserve_;
    const double reserve_distance = exploration_goal_queue_end_reserve_;
    const int reserve_samples = std::max(
        1, static_cast<int>(std::ceil(
               reserve_distance / (0.5 * voxel_resolution_))));
    bool reserve_clear = true;
    for (int sample = 1; sample <= reserve_samples; ++sample) {
      const double ratio = static_cast<double>(sample) / reserve_samples;
      const VoxelKey key = positionToKey(
          target.x + ratio * (reserve_end.x - target.x),
          target.y + ratio * (reserve_end.y - target.y), target.z);
      if (reachable.count(key) == 0 || !isClearForVehicle(key)) {
        reserve_clear = false;
        break;
      }
    }
    if (!task_segment_clear || !reserve_clear || isNearFailedGoal(target) ||
        !returnSegmentClear(previous, target, 0.0, false) ||
        !returnSegmentClear(target, reserve_end, 0.0, false)) {
      break;
    }
    const double preferred_extra_clearance = std::max(
        0.0, preferred_vehicle_radius_ -
                 (std::max(vehicle_radius_, return_super_radius_) +
                  return_clearance_margin_));
    queued.preferred_clearance =
        preferred_segment_clear &&
        returnSegmentClear(previous, target,
                           preferred_extra_clearance, false);
    exploration_goal_queue_.push_back(queued);
    previous = target;
    next_progress += exploration_goal_queue_spacing_;
  }

  if (!exploration_goal_queue_.empty()) {
    ROS_INFO("Cached %zu verified forward exploration successor(s)",
             exploration_goal_queue_.size());
  }
}

void SuperExplorationDecider::buildFrontierExplorationQueue(
    const VoxelSet& reachable, const std::vector<VoxelKey>& route,
    std::size_t active_goal_index) {
  clearExplorationGoalQueue();
  if (!exploration_goal_queue_enabled_ || route.empty() ||
      active_goal_index >= route.size() ||
      exploration_goal_queue_max_future_goals_ <= 0) {
    return;
  }

  const double reserve_cells = std::ceil(
      exploration_goal_queue_end_reserve_ / voxel_resolution_);
  const std::size_t last_safe_index = route.size() > reserve_cells
      ? route.size() - 1 - static_cast<std::size_t>(reserve_cells) : 0;
  const std::size_t horizon_cells = static_cast<std::size_t>(std::floor(
      effectiveExplorationGoalQueueHorizon() / voxel_resolution_ + 1e-6));
  const std::size_t maximum_index = std::min(last_safe_index, horizon_cells);
  if (maximum_index <= active_goal_index) return;

  std::size_t previous_index = active_goal_index;
  double next_route_distance =
      active_goal_index * voxel_resolution_ +
      exploration_goal_queue_spacing_;
  while (exploration_goal_queue_.size() <
             static_cast<std::size_t>(
                 exploration_goal_queue_max_future_goals_)) {
    const std::size_t candidate_index = static_cast<std::size_t>(std::ceil(
        next_route_distance / voxel_resolution_ - 1e-9));
    if (candidate_index > maximum_index || candidate_index >= route.size()) {
      break;
    }
    QueuedExplorationGoal queued;
    queued.goal.header.frame_id = world_frame_;
    queued.goal.pose.position = keyToPoint(route[candidate_index]);
    const geometry_msgs::Point previous = keyToPoint(route[previous_index]);
    queued.goal.pose.orientation = yawQuaternion(std::atan2(
        queued.goal.pose.position.y - previous.y,
        queued.goal.pose.position.x - previous.x));
    queued.direct_segment = false;
    queued.frontier_route = true;
    queued.route.assign(route.begin() + previous_index,
                        route.begin() + candidate_index + 1);
    if (std::abs(queued.goal.pose.position.z - previous.z) >
            max_frontier_goal_vertical_step_ + 1e-6 ||
        !queuedExplorationGoalValid(queued, reachable, false)) {
      break;
    }
    exploration_goal_queue_.push_back(queued);
    previous_index = candidate_index;
    next_route_distance = candidate_index * voxel_resolution_ +
                          exploration_goal_queue_spacing_;
  }

  if (!exploration_goal_queue_.empty()) {
    ROS_INFO("Cached %zu verified connected-frontier successor(s)",
             exploration_goal_queue_.size());
  }
}

bool SuperExplorationDecider::publishNextQueuedExplorationGoal(
    const VoxelSet& reachable) {
  if (exploration_goal_queue_.empty()) return false;
  if (!queuedExplorationGoalValid(exploration_goal_queue_.front(), reachable,
                                  true)) {
    ROS_WARN_THROTTLE(
        1.0,
        "Cached exploration successor no longer passes the latest task/SUPER "
        "map checks; recomputing without cancelling the active trajectory");
    clearExplorationGoalQueue();
    return false;
  }
  const QueuedExplorationGoal queued = exploration_goal_queue_.front();
  exploration_goal_queue_.erase(exploration_goal_queue_.begin());
  if (!queued.frontier_route) {
    // Refill from the newly selected leg and the newest map instead of
    // draining a snapshot made several seconds earlier. This keeps two or
    // three future hints available as visibility advances while still
    // issuing only this one goal to SUPER.
    const double c = std::cos(mission_heading_yaw_);
    const double s = std::sin(mission_heading_yaw_);
    const double current_dx =
        current_pose_.pose.position.x - home_pose_.pose.position.x;
    const double current_dy =
        current_pose_.pose.position.y - home_pose_.pose.position.y;
    const double target_dx =
        queued.goal.pose.position.x - home_pose_.pose.position.x;
    const double target_dy =
        queued.goal.pose.position.y - home_pose_.pose.position.y;
    LocalRouteCandidate active_goal;
    active_goal.goal = queued.goal;
    active_goal.mission_lateral = -s * target_dx + c * target_dy;
    active_goal.lookahead =
        (c * target_dx + s * target_dy) -
        (c * current_dx + s * current_dy);
    active_goal.preferred_clearance = queued.preferred_clearance;
    buildForwardExplorationQueue(reachable, active_goal);
  }
  const std::string reason = queued.frontier_route
      ? "frontier_route_queued_successor"
      : (queued.preferred_clearance
             ? "forward_lookahead_queued_preferred"
             : "forward_lookahead_queued_hard_fallback");
  publishGoal(queued.goal, reason, true);
  return true;
}

bool SuperExplorationDecider::publishForwardLookaheadGoal(
    const VoxelSet& reachable, bool require_successor) {
  LocalRouteCandidate selected;
  if (!require_successor && decision_clearance_cache_active_ &&
      decision_forward_route_evaluated_) {
    if (!decision_forward_route_available_) return false;
    selected = decision_forward_route_;
  } else {
    if (!selectForwardLookaheadGoal(
            reachable, require_successor, &selected)) {
      return false;
    }
  }
  if (selected.mission_lateral > 0.5 * voxel_resolution_) {
    held_route_side_ = 1;
  } else if (selected.mission_lateral < -0.5 * voxel_resolution_) {
    held_route_side_ = -1;
  }
  buildForwardExplorationQueue(reachable, selected);
  const bool centered = corridorCenteringActive();
  publishGoal(selected.goal,
              selected.preferred_clearance
                  ? (centered ? "forward_lookahead_centered_preferred"
                              : "forward_lookahead_preferred")
                  : (centered ? "forward_lookahead_centered_hard_fallback"
                              : "forward_lookahead_hard_fallback"),
              true);
  return true;
}

bool SuperExplorationDecider::publishLateralDetourGoal(
    const VoxelSet& reachable,
    const std::vector<FrontierCandidate>& candidates) {
  if (!have_home_ ||
      front_obstacle_streak_ < front_obstacle_confirm_frames_) return false;
  const double c = std::cos(mission_heading_yaw_);
  const double s = std::sin(mission_heading_yaw_);
  const double dx = current_pose_.pose.position.x - home_pose_.pose.position.x;
  const double dy = current_pose_.pose.position.y - home_pose_.pose.position.y;
  const double progress = c * dx + s * dy;
  const double lateral = -s * dx + c * dy;
  const double target_progress = std::max(0.0, progress - 0.25);
  std::vector<LocalRouteCandidate> feasible;

  for (const auto& candidate : candidates) {
    const double candidate_progress = candidateMissionProgress(candidate);
    if (!outboundProgressAllowed(
            progress, candidate_progress, max_outbound_backtrack_)) continue;
    const double lateral_delta =
        candidateMissionLateral(candidate) - lateral;
    const double step = std::copysign(
        std::min(std::abs(lateral_delta), 4.0), lateral_delta);
    // A lateral bypass is a transition into a neighbouring safe corridor,
    // not a normal outbound frontier.  Requiring the generic 2 m goal
    // distance rejected valid 0.5--1.5 m shifts around surveyed pipes and
    // machinery even when the full segment was known-free below.
    if (std::abs(step) < voxel_resolution_ - 1e-6) continue;
    geometry_msgs::PoseStamped goal;
    goal.header.frame_id = world_frame_;
    goal.pose.position.x =
        home_pose_.pose.position.x + c * target_progress - s * (lateral + step);
    goal.pose.position.y =
        home_pose_.pose.position.y + s * target_progress + c * (lateral + step);
    goal.pose.position.z = current_pose_.pose.position.z;
    goal.pose.orientation = yawQuaternion(std::atan2(
        goal.pose.position.y - current_pose_.pose.position.y,
        goal.pose.position.x - current_pose_.pose.position.x));
    // A candidate on the same side of the mission centreline can still point
    // straight back down the just-traversed lateral leg. That happened at the
    // Task-4 narrow entrance: -6.2 m -> -2.2 m was incorrectly treated as
    // preserving the negative-side choice. Lateral detours must open genuinely
    // new space; the connected-route and return states own deliberate
    // backtracking.
    if (outbound_breadcrumbs_.size() >= 2 &&
        outboundSeparation(goal.pose.position) + 1e-6 <
            connected_turn_min_new_space_) {
      continue;
    }
    const VoxelKey goal_key = positionToKey(
        goal.pose.position.x, goal.pose.position.y, goal.pose.position.z);
    if (reachable.count(goal_key) == 0 || !isClearForVehicle(goal_key)) continue;

    // Demand a known-free, body-clear straight lateral transition before
    // handing the goal to SUPER. A connected component alone may wind around
    // a wall and does not prove the direct transition is safe.
    const double distance = std::hypot(
        goal.pose.position.x - current_pose_.pose.position.x,
        goal.pose.position.y - current_pose_.pose.position.y);
    const int samples = std::max(
        1, static_cast<int>(std::ceil(distance / (0.5 * voxel_resolution_))));
    bool direct_clear = true;
    bool preferred_direct_clear = isClearForVehicle(
        goal_key, preferred_vehicle_radius_);
    for (int i = 1; i <= samples; ++i) {
      const double t = static_cast<double>(i) / samples;
      const auto& start = current_pose_.pose.position;
      const VoxelKey key = positionToKey(
          start.x + t * (goal.pose.position.x - start.x),
          start.y + t * (goal.pose.position.y - start.y),
          start.z);
      if (reachable.count(key) == 0 || !isClearForVehicle(key)) {
        direct_clear = false;
        break;
      }
      if (preferred_direct_clear &&
          !isClearForVehicle(key, preferred_vehicle_radius_)) {
        preferred_direct_clear = false;
      }
    }
    if (!direct_clear ||
        !returnSegmentClear(current_pose_.pose.position,
                            goal.pose.position, 0.0, false)) continue;
    const double preferred_extra_clearance = std::max(
        0.0, preferred_vehicle_radius_ -
                 (std::max(vehicle_radius_, return_super_radius_) +
                  return_clearance_margin_));
    const bool has_preferred_clearance =
        preferred_direct_clear &&
        returnSegmentClear(current_pose_.pose.position, goal.pose.position,
                           preferred_extra_clearance, false);
    const double score = candidate_progress - 0.1 * distance +
                         0.01 * candidate.unknown_neighbors;
    LocalRouteCandidate route;
    route.goal = goal;
    route.mission_lateral = lateral + step;
    route.minimum_clearance = minimumSuperSegmentClearance(
        current_pose_.pose.position, goal.pose.position);
    route.selection_score = score;
    route.preferred_clearance = has_preferred_clearance;
    feasible.push_back(route);
  }

  if (feasible.empty()) {
    // A wide, fully observed side passage can have no usable frontier inside
    // the mission band: every ray may terminate on the far side wall, placing
    // the free/unknown boundary too close to that wall for the vehicle-radius
    // filter.  The 2026-10-18 real flight had exactly this geometry on the
    // right: six metres of clear lateral space, but the only retained
    // frontier was behind the aircraft.  Before declaring a dead end, sample
    // both lateral directions directly and accept only a complete segment
    // that is reachable, known free, body-clear, and clear in SUPER's live
    // occupancy map.
    for (const double direction : {-1.0, 1.0}) {
      for (double displacement = 4.0;
           displacement >= voxel_resolution_ - 1e-6;
           displacement -= voxel_resolution_) {
        const double step = direction * displacement;
        const double target_lateral = lateral + step;
        if (std::abs(target_lateral) > max_task_lateral_offset_ + 1e-6) {
          continue;
        }

        geometry_msgs::PoseStamped goal;
        goal.header.frame_id = world_frame_;
        goal.pose.position.x =
            home_pose_.pose.position.x + c * target_progress -
            s * target_lateral;
        goal.pose.position.y =
            home_pose_.pose.position.y + s * target_progress +
            c * target_lateral;
        goal.pose.position.z = current_pose_.pose.position.z;
        goal.pose.orientation = yawQuaternion(std::atan2(
            goal.pose.position.y - current_pose_.pose.position.y,
            goal.pose.position.x - current_pose_.pose.position.x));
        if (outbound_breadcrumbs_.size() >= 2 &&
            outboundSeparation(goal.pose.position) + 1e-6 <
                connected_turn_min_new_space_) {
          continue;
        }
        const VoxelKey goal_key = positionToKey(
            goal.pose.position.x, goal.pose.position.y,
            goal.pose.position.z);
        if (reachable.count(goal_key) == 0 ||
            !isClearForVehicle(goal_key)) {
          continue;
        }

        const auto& start = current_pose_.pose.position;
        const double distance = std::sqrt(squaredDistance(start,
                                                           goal.pose.position));
        const int samples = std::max(
            1, static_cast<int>(std::ceil(
                   distance / (0.5 * voxel_resolution_))));
        bool direct_clear = true;
        bool preferred_direct_clear =
            isClearForVehicle(goal_key, preferred_vehicle_radius_);
        for (int i = 1; i <= samples; ++i) {
          const double t = static_cast<double>(i) / samples;
          const VoxelKey key = positionToKey(
              start.x + t * (goal.pose.position.x - start.x),
              start.y + t * (goal.pose.position.y - start.y), start.z);
          if (reachable.count(key) == 0 || !isClearForVehicle(key)) {
            direct_clear = false;
            break;
          }
          if (preferred_direct_clear &&
              !isClearForVehicle(key, preferred_vehicle_radius_)) {
            preferred_direct_clear = false;
          }
        }
        if (!direct_clear ||
            !returnSegmentClear(start, goal.pose.position, 0.0, false)) {
          continue;
        }
        const double preferred_extra_clearance = std::max(
            0.0, preferred_vehicle_radius_ -
                     (std::max(vehicle_radius_, return_super_radius_) +
                      return_clearance_margin_));
        LocalRouteCandidate route;
        route.goal = goal;
        route.mission_lateral = target_lateral;
        route.minimum_clearance =
            minimumSuperSegmentClearance(start, goal.pose.position);
        route.selection_score = displacement;
        route.preferred_clearance =
            preferred_direct_clear &&
            returnSegmentClear(start, goal.pose.position,
                               preferred_extra_clearance, false);
        feasible.push_back(route);
        // Only the farthest verified point on each side is needed. The
        // clearance comparison below still chooses the safer side.
        break;
      }
    }
  }
  if (feasible.empty()) return false;

  auto side_of = [this](double value) {
    if (value > 0.5 * voxel_resolution_) return 1;
    if (value < -0.5 * voxel_resolution_) return -1;
    return 0;
  };
  auto better = [](const LocalRouteCandidate& lhs,
                   const LocalRouteCandidate& rhs) {
    if (!std::isfinite(lhs.minimum_clearance)) {
      if (std::isfinite(rhs.minimum_clearance)) return true;
    } else if (!std::isfinite(rhs.minimum_clearance)) {
      return false;
    } else if (lhs.minimum_clearance > rhs.minimum_clearance + 1e-9) {
      return true;
    } else if (rhs.minimum_clearance > lhs.minimum_clearance + 1e-9) {
      return false;
    }
    return lhs.selection_score > rhs.selection_score + 1e-9;
  };
  const bool preferred_available = std::any_of(
      feasible.begin(), feasible.end(),
      [](const LocalRouteCandidate& route) {
        return route.preferred_clearance;
      });
  const auto eligible = [preferred_available](
                            const LocalRouteCandidate& route) {
    return !preferred_available || route.preferred_clearance;
  };
  const LocalRouteCandidate* clearest = nullptr;
  for (const auto& route : feasible) {
    if (!eligible(route)) continue;
    if (clearest == nullptr || better(route, *clearest)) clearest = &route;
  }
  const LocalRouteCandidate* selected = clearest;
  if (held_route_side_ != 0) {
    const LocalRouteCandidate* held = nullptr;
    for (const auto& route : feasible) {
      if (!eligible(route) ||
          side_of(route.mission_lateral) != held_route_side_) continue;
      const bool close_enough =
          (!std::isfinite(clearest->minimum_clearance) &&
           !std::isfinite(route.minimum_clearance)) ||
          (std::isfinite(clearest->minimum_clearance) &&
           route.minimum_clearance +
                   lateral_route_switch_clearance_advantage_ >=
               clearest->minimum_clearance);
      if (close_enough && (held == nullptr || better(route, *held))) {
        held = &route;
      }
    }
    if (held != nullptr) selected = held;
  }
  const int selected_side = side_of(selected->mission_lateral);
  if (selected_side != 0) held_route_side_ = selected_side;
  publishGoal(selected->goal, selected->preferred_clearance
                             ? "lateral_detour_preferred"
                             : "lateral_detour_hard_fallback");
  return true;
}

bool SuperExplorationDecider::publishConnectedTurnContinuation(
    const VoxelSet& reachable, bool continuous_handover) {
  if (!connected_turn_continuation_enabled_ || !have_home_ ||
      reachable.empty()) {
    return false;
  }

  // The entry-axis look-ahead is intentionally straight. At a real bend it
  // can therefore report no executable segment even though the same
  // hard-clear reachable component continues around the corner. Follow that
  // component for one local horizon, using the last accepted route tangent as
  // the turn reference. Unknown cells and cells below the 0.8 m planning
  // floor never enter this graph.
  VoxelKey start = positionToKey(current_pose_.pose.position.x,
                                 current_pose_.pose.position.y,
                                 current_pose_.pose.position.z);
  if (reachable.count(start) == 0) {
    double nearest = std::numeric_limits<double>::infinity();
    for (const VoxelKey& key : reachable) {
      const double distance = squaredDistance(
          keyToPoint(key), current_pose_.pose.position);
      if (distance < nearest) {
        nearest = distance;
        start = key;
      }
    }
  }
  if (reachable.count(start) == 0) return false;

  static const int kNeighbors[6][3] = {
      {1, 0, 0}, {-1, 0, 0}, {0, 1, 0},
      {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};
  const int maximum_steps = std::max(
      1, static_cast<int>(std::floor(
             effectiveExplorationGoalQueueHorizon() /
                 voxel_resolution_ + 1e-9)));
  std::queue<VoxelKey> queue;
  std::unordered_map<VoxelKey, VoxelKey, VoxelKeyHash> parent;
  std::unordered_map<VoxelKey, int, VoxelKeyHash> steps;
  queue.push(start);
  parent.emplace(start, start);
  steps.emplace(start, 0);

  const double route_heading = tunnel_route_locked_
      ? tunnel_route_heading_yaw_
      : (have_last_exploration_route_heading_
             ? last_exploration_route_heading_yaw_
             : mission_heading_yaw_);
  const double route_c = std::cos(route_heading);
  const double route_s = std::sin(route_heading);
  // Task 4 treats the accepted local tangent as a preference, not as a
  // straight-only constraint. Keep following the current passage whenever it
  // exists, but allow a verified sharp branch when that passage genuinely
  // ends. Reversal toward the surveyed junction remains outside this cone.
  const bool tunnel_profile =
      mission_profile_ == ExplorationMissionProfile::kTunnel;
  const double maximum_turn =
      (tunnel_profile ? tunnel_connected_turn_max_angle_deg_
                      : connected_turn_max_angle_deg_) * M_PI / 180.0;
  const double preferred_turn =
      tunnel_straight_preference_angle_deg_ * M_PI / 180.0;
  const double mission_c = std::cos(mission_heading_yaw_);
  const double mission_s = std::sin(mission_heading_yaw_);
  const auto& home = home_pose_.pose.position;
  const auto& current = current_pose_.pose.position;
  const double current_progress =
      mission_c * (current.x - home.x) + mission_s * (current.y - home.y);
  const bool home_distance_limited =
      max_exploration_radius_from_home_ > 0.0;
  const double maximum_home_distance_squared = home_distance_limited
      ? max_exploration_radius_from_home_ * max_exploration_radius_from_home_
      : std::numeric_limits<double>::infinity();

  struct ConnectedRouteEndpoint {
    VoxelKey key;
    double score{0.0};
    double tangent_yaw{0.0};
    int turn_side{0};
  };
  std::vector<ConnectedRouteEndpoint> endpoints;
  const double tangent_window = std::max(1.0, min_goal_distance_ * 0.5);
  const auto route_tangent_yaw = [&](VoxelKey key) {
    const geometry_msgs::Point end = keyToPoint(key);
    double length = 0.0;
    while (!(key == start) && length < tangent_window) {
      const VoxelKey previous = parent.at(key);
      length += std::sqrt(squaredDistance(keyToPoint(previous),
                                          keyToPoint(key)));
      key = previous;
    }
    const geometry_msgs::Point tangent_start = keyToPoint(key);
    return std::atan2(end.y - tangent_start.y,
                      end.x - tangent_start.x);
  };
  while (!queue.empty()) {
    const VoxelKey key = queue.front();
    queue.pop();
    const int route_steps = steps.at(key);
    const geometry_msgs::Point point = keyToPoint(key);
    if (route_steps > 0) {
      const double dx = point.x - current.x;
      const double dy = point.y - current.y;
      const double horizontal_distance = std::hypot(dx, dy);
      const double home_dx = point.x - home.x;
      const double home_dy = point.y - home.y;
      const double progress = mission_c * home_dx + mission_s * home_dy;
      const double lateral = -mission_s * home_dx + mission_c * home_dy;
      const double route_progress = route_c * dx + route_s * dy;
      const double chord_turn = std::abs(normalizeAngle(
          std::atan2(dy, dx) - route_heading));
      const double tangent_yaw = route_tangent_yaw(key);
      const double signed_tangent_turn = normalizeAngle(
          tangent_yaw - route_heading);
      const double tangent_turn = std::abs(signed_tangent_turn);
      const double new_space = outboundSeparation(point);
      const bool new_space_ready = outbound_breadcrumbs_.size() < 2 ||
          new_space >= connected_turn_min_new_space_;
      if (horizontal_distance >= min_goal_distance_ - 1e-6 &&
          home_dx * home_dx + home_dy * home_dy <=
              maximum_home_distance_squared + 1e-6 &&
          (tunnel_route_locked_ ||
           std::abs(lateral) <= max_task_lateral_offset_ + 1e-6) &&
          missionObservationHeightAllowed(point.z,
                                          0.5 * voxel_resolution_) &&
          (tunnel_route_locked_
               ? route_progress >= -max_outbound_backtrack_ - 1e-6
               : outboundProgressAllowed(current_progress, progress,
                                         max_outbound_backtrack_)) &&
          chord_turn <= maximum_turn + 1e-9 &&
          tangent_turn <= maximum_turn + 1e-9 && new_space_ready &&
          !isNearCoveredGoal(point) && !isNearFailedGoal(point) &&
          returnSegmentClear(point, point, 0.0, false)) {
        const double route_distance = route_steps * voxel_resolution_;
        const double alignment = std::cos(tangent_turn);
        const double preferred_bonus =
            isClearForVehicle(key, preferred_vehicle_radius_) ? 0.25 : 0.0;
        const double directional_progress = tunnel_route_locked_
            ? route_progress : progress - current_progress;
        const double score = route_distance + 1.5 * new_space +
            2.0 * alignment + 0.25 * directional_progress +
            preferred_bonus;
        const int turn_side = signed_tangent_turn > preferred_turn ? 1 :
            (signed_tangent_turn < -preferred_turn ? -1 : 0);
        endpoints.push_back({key, score, tangent_yaw, turn_side});
      }
    }
    if (route_steps >= maximum_steps) continue;
    for (const auto& offset : kNeighbors) {
      const VoxelKey next{key.x + offset[0], key.y + offset[1],
                          key.z + offset[2]};
      if (reachable.count(next) == 0 || parent.count(next) > 0) continue;
      parent.emplace(next, key);
      steps.emplace(next, route_steps + 1);
      queue.push(next);
    }
  }
  if (endpoints.empty()) return false;

  // Retain several strong endpoints for each actual turn direction.  The old
  // implementation chose one endpoint from its aircraft-to-goal chord before
  // examining the route geometry.  At the 2026-10-25 bend that chord was only
  // 34 degrees right while the path tangent was 90 degrees right, so a much
  // clearer left branch never reached the comparison below.
  std::array<std::vector<ConnectedRouteEndpoint>, 3> directional_endpoints;
  for (const auto& endpoint : endpoints) {
    directional_endpoints[endpoint.turn_side + 1].push_back(endpoint);
  }
  constexpr std::size_t kEndpointShortlistPerDirection = 8;
  for (auto& direction : directional_endpoints) {
    std::sort(direction.begin(), direction.end(),
              [](const ConnectedRouteEndpoint& lhs,
                 const ConnectedRouteEndpoint& rhs) {
                return lhs.score > rhs.score;
              });
    if (direction.size() > kEndpointShortlistPerDirection) {
      direction.resize(kEndpointShortlistPerDirection);
    }
  }

  struct EvaluatedConnectedRoute {
    std::vector<VoxelKey> route;
    std::size_t local_goal_index{0};
    double tangent_yaw{0.0};
    double score{0.0};
    double minimum_clearance{0.0};
    bool follows_current_route{false};
    bool preferred_clearance{false};
  };
  std::vector<EvaluatedConnectedRoute> evaluated;
  for (const auto& direction : directional_endpoints) {
    for (const auto& endpoint : direction) {
      EvaluatedConnectedRoute candidate;
      candidate.score = endpoint.score;
      for (VoxelKey key = endpoint.key;; key = parent.at(key)) {
        candidate.route.push_back(key);
        if (key == start) break;
      }
      std::reverse(candidate.route.begin(), candidate.route.end());

      bool found_local_goal = false;
      for (std::size_t index = 1; index < candidate.route.size(); ++index) {
        const double route_distance = index * voxel_resolution_;
        if (route_distance > max_frontier_goal_distance_ + 1e-6) break;
        const geometry_msgs::Point point = keyToPoint(candidate.route[index]);
        const double horizontal_distance =
            std::hypot(point.x - current.x, point.y - current.y);
        if (horizontal_distance < min_goal_distance_ - 1e-6 ||
            std::abs(point.z - current.z) >
                max_frontier_goal_vertical_step_ + 1e-6 ||
            isNearFailedGoal(point) ||
            !returnSegmentClear(point, point, 0.0, false)) {
          continue;
        }
        candidate.local_goal_index = index;
        found_local_goal = true;
      }
      if (!found_local_goal) continue;

      const geometry_msgs::Point local_goal =
          keyToPoint(candidate.route[candidate.local_goal_index]);
      candidate.score -= forward_height_penalty_ *
          std::abs(local_goal.z - current.z);

      std::size_t tangent_start_index = candidate.local_goal_index;
      double tangent_length = 0.0;
      while (tangent_start_index > 0 && tangent_length < tangent_window) {
        const geometry_msgs::Point a =
            keyToPoint(candidate.route[tangent_start_index - 1]);
        const geometry_msgs::Point b =
            keyToPoint(candidate.route[tangent_start_index]);
        tangent_length += std::sqrt(squaredDistance(a, b));
        --tangent_start_index;
      }
      const geometry_msgs::Point tangent_start =
          keyToPoint(candidate.route[tangent_start_index]);
      candidate.tangent_yaw = std::atan2(
          local_goal.y - tangent_start.y,
          local_goal.x - tangent_start.x);
      const double tangent_turn = std::abs(normalizeAngle(
          candidate.tangent_yaw - route_heading));
      if (tangent_turn > maximum_turn + 1e-9) continue;
      candidate.follows_current_route =
          tunnel_profile && tangent_turn <= preferred_turn + 1e-9;

      candidate.minimum_clearance = minimumSuperSegmentClearance(
          current, keyToPoint(candidate.route.front()));
      candidate.preferred_clearance = true;
      for (std::size_t index = 0; index < candidate.route.size(); ++index) {
        if (!isClearForVehicle(candidate.route[index],
                               preferred_vehicle_radius_)) {
          candidate.preferred_clearance = false;
        }
        if (index == 0) continue;
        candidate.minimum_clearance = std::min(
            candidate.minimum_clearance,
            minimumSuperSegmentClearance(
                keyToPoint(candidate.route[index - 1]),
                keyToPoint(candidate.route[index])));
      }
      if (std::isfinite(candidate.minimum_clearance) &&
          candidate.minimum_clearance + 1e-9 < preferred_vehicle_radius_) {
        candidate.preferred_clearance = false;
      }
      ROS_DEBUG("Connected-route candidate end=(%d,%d,%d) local=(%.2f,%.2f) "
                "yaw=%.1f clearance=%.2f score=%.2f follows=%d",
                endpoint.key.x, endpoint.key.y, endpoint.key.z,
                local_goal.x, local_goal.y,
                candidate.tangent_yaw * 180.0 / M_PI,
                candidate.minimum_clearance, candidate.score,
                candidate.follows_current_route);
      evaluated.push_back(std::move(candidate));
    }
  }
  if (evaluated.empty()) return false;

  // Clearance is the primary branch decision.  The accepted tunnel tangent
  // is retained only when competing routes are within the existing 0.25 m
  // side-switch hysteresis; a materially safer branch must be allowed to win.
  const auto materially_clearer = [this](double lhs, double rhs) {
    if (!std::isfinite(lhs)) return std::isfinite(rhs);
    if (!std::isfinite(rhs)) return false;
    return lhs > rhs + lateral_route_switch_clearance_advantage_ + 1e-9;
  };
  const EvaluatedConnectedRoute* selected = &evaluated.front();
  for (const auto& candidate : evaluated) {
    if (materially_clearer(candidate.minimum_clearance,
                           selected->minimum_clearance)) {
      selected = &candidate;
      continue;
    }
    if (materially_clearer(selected->minimum_clearance,
                           candidate.minimum_clearance)) {
      continue;
    }
    if (candidate.follows_current_route != selected->follows_current_route) {
      if (candidate.follows_current_route) selected = &candidate;
      continue;
    }
    if (candidate.preferred_clearance != selected->preferred_clearance) {
      if (candidate.preferred_clearance) selected = &candidate;
      continue;
    }
    if (candidate.minimum_clearance > selected->minimum_clearance + 1e-9 ||
        (!(selected->minimum_clearance >
           candidate.minimum_clearance + 1e-9) &&
         candidate.score > selected->score + 1e-9)) {
      selected = &candidate;
    }
  }

  const std::vector<VoxelKey>& route = selected->route;
  const std::size_t selected_index = selected->local_goal_index;

  geometry_msgs::PoseStamped goal;
  goal.header.frame_id = world_frame_;
  goal.pose.position = keyToPoint(route[selected_index]);
  // The chord from the aircraft to a point beyond a bend points diagonally
  // into the corner. Use the route tangent near the endpoint so SUPER can
  // distribute the yaw change along one continuous curved trajectory and
  // finish aligned with the outgoing passage.
  goal.pose.orientation = yawQuaternion(selected->tangent_yaw);
  const double selected_yaw = selected->tangent_yaw;
  const double selected_turn = normalizeAngle(selected_yaw - route_heading);
  if (tunnel_profile && !tunnel_route_locked_ &&
      std::abs(selected_turn) > preferred_turn + 1e-9 &&
      tunnel_connected_turn_confirm_frames_ > 1) {
    const int selected_turn_side = selected_turn > 0.0 ? 1 : -1;
    const ros::Time now = ros::Time::now();
    const double confirmation_timeout = std::max(
        1.0, 2.5 / std::max(0.1, decision_rate_));
    if (pending_connected_turn_side_ != selected_turn_side ||
        pending_connected_turn_time_.isZero() ||
        (now - pending_connected_turn_time_).toSec() >
            confirmation_timeout) {
      pending_connected_turn_side_ = selected_turn_side;
      pending_connected_turn_streak_ = 1;
    } else {
      pending_connected_turn_streak_ = std::min(
          tunnel_connected_turn_confirm_frames_,
          pending_connected_turn_streak_ + 1);
    }
    pending_connected_turn_time_ = now;
    if (pending_connected_turn_streak_ <
        tunnel_connected_turn_confirm_frames_) {
      ROS_INFO_THROTTLE(
          0.5,
          "Sharp tunnel turn %.1f deg awaiting stable branch evidence "
          "(%d/%d frames); current trajectory remains active",
          selected_turn * 180.0 / M_PI,
          pending_connected_turn_streak_,
          tunnel_connected_turn_confirm_frames_);
      return false;
    }
  }
  pending_connected_turn_side_ = 0;
  pending_connected_turn_streak_ = 0;
  pending_connected_turn_time_ = ros::Time();
  ROS_INFO("Connected turn continuation: %.1f m verified route, local goal "
           "%.2f m away, heading %.1f -> %.1f deg, minimum clearance %.2f m",
           (route.size() - 1) * voxel_resolution_,
           std::hypot(goal.pose.position.x - current.x,
                      goal.pose.position.y - current.y),
           route_heading * 180.0 / M_PI,
           selected_yaw * 180.0 / M_PI,
           selected->minimum_clearance);
  publishGoal(goal, "connected_turn_route", continuous_handover);
  buildFrontierExplorationQueue(reachable, route, selected_index);
  return true;
}

bool SuperExplorationDecider::reverseMissionHeadingAtDeadEnd(
    const MapClosureStatus& closure) {
  if (!have_home_ || have_active_goal_ ||
      max_dead_end_heading_reversals_ <= 0 ||
      dead_end_heading_reversal_count_ >= max_dead_end_heading_reversals_ ||
      !closure.front_boundary_seen || closure.reachable_forward_passage ||
      closure.actionable_frontiers != 0 ||
      closure.no_frontier_duration < dead_end_reversal_delay_ ||
      closure.vehicle_progress < front_obstacle_min_progress_) {
    return false;
  }

  const double previous_heading = mission_heading_yaw_;
  mission_heading_yaw_ = normalizeAngle(mission_heading_yaw_ + M_PI);
  resetCorridorCentering();
  resetCorridorPreviewPreference();
  ++dead_end_heading_reversal_count_;
  exploration_phase_ = ExplorationPhase::kForwardPriority;
  front_obstacle_streak_ = 0;
  side_wall_missing_streak_ = 0;
  map_closure_complete_streak_ = 0;
  last_actionable_frontier_time_ = ros::Time::now();
  dead_end_backtracking_ = true;
  return_waypoints_.clear();
  return_waypoint_index_ = 0;
  if (!publishFreshReturnGoal("dead_end_live_map")) {
    // A long frontier evaluation can make its input pose stale before the
    // return goal is published. Do not latch the reversed heading or enter
    // backtrack mode without an accepted, live-map-clear first leg.
    mission_heading_yaw_ = previous_heading;
    --dead_end_heading_reversal_count_;
    dead_end_backtracking_ = false;
    exploration_phase_ = ExplorationPhase::kFrontierFallback;
    ROS_WARN_THROTTLE(2.0,
        "Dead-end return has no fresh, SUPER-map-clear first leg; holding for new sensor data");
    return false;
  }
  // Left/right signs are defined in the mission-heading frame. A 180-degree
  // reversal swaps those signs, so carrying the old hold into the opposite
  // exploration leg would bias the first new route toward the wrong side.
  held_route_side_ = 0;
  ROS_WARN(
      "Dead end has no vehicle-clear detour; reversing mission heading "
      "%.1f -> %.1f deg (%d/%d); return goals use the live map",
      previous_heading * 180.0 / M_PI,
      mission_heading_yaw_ * 180.0 / M_PI,
      dead_end_heading_reversal_count_, max_dead_end_heading_reversals_);
  return true;
}

bool SuperExplorationDecider::tryForwardHandover(const VoxelSet& reachable) {
  if (!have_active_goal_) {
    return false;
  }
  const geometry_msgs::Point previous_goal = current_goal_.pose.position;
  const bool previous_goal_reached =
      squaredDistance(current_pose_.pose.position, previous_goal) <=
      goal_reached_distance_ * goal_reached_distance_;
  // A cached leg is only a planning hint. It is re-certified against the
  // latest task and SUPER maps before a single successor is handed over.
  // If it has gone stale, calculate one fresh successor while leaving the
  // current accepted trajectory active until that calculation succeeds.
  // Preserve the rolling relay contract: hand the already selected and
  // revalidated queued point to SUPER first. Only recompute from the live map
  // when that cached successor is no longer safe.
  bool successor_published = publishNextQueuedExplorationGoal(reachable);
  if (!successor_published &&
      mission_profile_ == ExplorationMissionProfile::kTunnel) {
    // Look through the end of the current straight leg before the aircraft
    // reaches its stop point. A normal forward successor remains preferred;
    // when it is absent, hand the verified bend to SUPER as a continuous
    // retarget instead of stopping and replanning from rest at the corner.
    successor_published = publishForwardLookaheadGoal(reachable, true) ||
        publishConnectedTurnContinuation(reachable, true);
  } else if (!successor_published) {
    successor_published = publishForwardLookaheadGoal(reachable, true);
  }
  if (!successor_published) {
    return false;
  }
  if (previous_goal_reached) {
    covered_goals_.push_back(previous_goal);
    ++reached_goal_count_;
  }
  return true;
}

bool SuperExplorationDecider::selectAndPublishFrontier(
    const std::vector<FrontierCandidate>& candidates,
    const VoxelSet& reachable) {
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
    // Prefer the separately verified lateral transition while it is
    // available.  Once that short transition has actually completed, an
    // obstacle or T junction can still leave a safe actionable frontier in
    // the connected component.  Continue toward that frontier through the
    // short-route limiter below instead of waiting forever merely because
    // the original entry-axis cone remains blocked.
    const bool front_obstacle_confirmed =
        front_obstacle_streak_ >= front_obstacle_confirm_frames_;
    const double current_dx =
        current_pose_.pose.position.x - home_pose_.pose.position.x;
    const double current_dy =
        current_pose_.pose.position.y - home_pose_.pose.position.y;
    const double current_progress =
        std::cos(mission_heading_yaw_) * current_dx +
        std::sin(mission_heading_yaw_) * current_dy;
    const double allowed_backtrack = front_obstacle_confirmed
        ? std::min(max_outbound_backtrack_, max_frontier_goal_distance_)
        : max_outbound_backtrack_;
    double best_score = -std::numeric_limits<double>::infinity();
    for (const auto& candidate : candidates) {
      if (!outboundProgressAllowed(
              current_progress, candidateMissionProgress(candidate),
              allowed_backtrack)) continue;
      const double score =
          candidate.score + fallback_heading_weight_ *
                                candidateHeadingAlignment(candidate);
      if (selected == nullptr ||
          (candidate.preferred_clearance &&
           !selected->preferred_clearance) ||
          (candidate.preferred_clearance == selected->preferred_clearance &&
           score > best_score + 1e-9)) {
        selected = &candidate;
        best_score = score;
      }
    }
    if (selected == nullptr) return false;
    reason = front_obstacle_confirmed
        ? "blocked_frontier_route"
        : "frontier_fallback";
  }

  geometry_msgs::PoseStamped selected_goal = selected->goal;
  const geometry_msgs::Point& start = current_pose_.pose.position;
  const geometry_msgs::Point remote_goal = selected_goal.pose.position;
  const double dx = remote_goal.x - start.x;
  const double dy = remote_goal.y - start.y;
  const double horizontal_distance = std::hypot(dx, dy);

  // The frontier chooses a branch, while the reachable component describes
  // how that branch can actually be entered. Follow its six-connected route
  // for one short horizon instead of aiming through the inside of a corner.
  VoxelKey route_start = positionToKey(start.x, start.y, start.z);
  if (reachable.count(route_start) == 0) {
    double nearest_distance = std::numeric_limits<double>::infinity();
    for (const VoxelKey& key : reachable) {
      const double distance = squaredDistance(keyToPoint(key), start);
      if (distance < nearest_distance) {
        route_start = key;
        nearest_distance = distance;
      }
    }
  }
  const VoxelKey route_target = selected->key;
  static const int kRouteNeighbors[6][3] = {
      {1, 0, 0}, {-1, 0, 0}, {0, 1, 0},
      {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};
  std::queue<VoxelKey> route_queue;
  std::unordered_map<VoxelKey, VoxelKey, VoxelKeyHash> route_parent;
  route_queue.push(route_start);
  route_parent.emplace(route_start, route_start);
  while (!route_queue.empty() && route_parent.count(route_target) == 0) {
    const VoxelKey current = route_queue.front();
    route_queue.pop();
    for (const auto& offset : kRouteNeighbors) {
      const VoxelKey neighbor{current.x + offset[0], current.y + offset[1],
                              current.z + offset[2]};
      if (reachable.count(neighbor) == 0 ||
          route_parent.count(neighbor) > 0) {
        continue;
      }
      route_parent.emplace(neighbor, current);
      route_queue.push(neighbor);
    }
  }

  std::vector<VoxelKey> route;
  if (route_parent.count(route_target) > 0) {
    for (VoxelKey key = route_target;; key = route_parent.at(key)) {
      route.push_back(key);
      if (key == route_start) break;
    }
    std::reverse(route.begin(), route.end());
  }

  bool found_local_goal = false;
  std::size_t selected_route_index = 0;
  double route_distance = 0.0;
  for (std::size_t index = 1; index < route.size(); ++index) {
    route_distance += voxel_resolution_;
    if (route_distance > max_frontier_goal_distance_ + 1e-6) break;
    const geometry_msgs::Point candidate_goal = keyToPoint(route[index]);
    const double horizontal_step = std::hypot(
        candidate_goal.x - start.x, candidate_goal.y - start.y);
    if (horizontal_step + 1e-6 < min_goal_distance_ ||
        horizontal_step > max_frontier_goal_distance_ + 1e-6 ||
        std::abs(candidate_goal.z - start.z) >
            max_frontier_goal_vertical_step_ + 1e-6) {
      continue;
    }
    // The connected route may bend around a corner. SUPER owns its curved
    // corridor, so validate this local target in the current occupancy map
    // without imposing a straight line through the inside of the bend.
    if (isNearFailedGoal(candidate_goal) ||
        !returnSegmentClear(candidate_goal, candidate_goal, 0.0)) continue;
    selected_goal.pose.position = candidate_goal;
    found_local_goal = true;
    selected_route_index = index;
  }
  if (!found_local_goal) {
    ROS_WARN_THROTTLE(
        2.0,
        "Frontier direction has no short connected progressive segment; "
        "waiting for map growth instead of commanding a remote arc");
    return false;
  }

  // The remote frontier only chooses a branch. Face the verified local route
  // segment in both exploration phases; keeping the entry heading here made
  // oblique heading-priority legs translate sideways without turning the nose.
  selected_goal.pose.orientation = yawQuaternion(std::atan2(
      selected_goal.pose.position.y - start.y,
      selected_goal.pose.position.x - start.x));
  if (horizontal_distance > max_frontier_goal_distance_) {
    ROS_INFO("Localized %.2f m frontier to connected %.2f m segment",
             horizontal_distance,
             std::hypot(selected_goal.pose.position.x - start.x,
                        selected_goal.pose.position.y - start.y));
  }
  last_frontier_time_ = ros::Time::now();
  publishGoal(selected_goal, reason);
  buildFrontierExplorationQueue(reachable, route, selected_route_index);
  return true;
}

double SuperExplorationDecider::outboundSeparation(
    const geometry_msgs::Point& point) const {
  double nearest = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < outbound_breadcrumbs_.size(); ++i) {
    const auto& a = outbound_breadcrumbs_[i].pose.position;
    const auto& b = i + 1 < outbound_breadcrumbs_.size()
        ? outbound_breadcrumbs_[i + 1].pose.position : a;
    const double dx = b.x - a.x, dy = b.y - a.y, dz = b.z - a.z;
    const double len2 = dx * dx + dy * dy + dz * dz;
    const double t = len2 > 1e-9 ? std::max(0.0, std::min(1.0,
        ((point.x - a.x) * dx + (point.y - a.y) * dy +
         (point.z - a.z) * dz) / len2)) : 0.0;
    geometry_msgs::Point projection;
    projection.x = a.x + t * dx;
    projection.y = a.y + t * dy;
    projection.z = a.z + t * dz;
    nearest = std::min(nearest, std::sqrt(squaredDistance(point, projection)));
  }
  return std::isfinite(nearest) ? nearest : 0.0;
}

double SuperExplorationDecider::minimumSuperSegmentClearance(
    const geometry_msgs::Point& from,
    const geometry_msgs::Point& to) const {
  if (super_occupied_points_->empty()) {
    return std::numeric_limits<double>::infinity();
  }
  const double distance = std::sqrt(squaredDistance(from, to));
  const int steps = std::max(1, static_cast<int>(std::ceil(
      distance / (0.5 * voxel_resolution_))));
  double minimum = std::numeric_limits<double>::infinity();
  for (int i = 0; i <= steps; ++i) {
    const double t = static_cast<double>(i) / steps;
    pcl::PointXYZ query(
        from.x + t * (to.x - from.x),
        from.y + t * (to.y - from.y),
        from.z + t * (to.z - from.z));
    std::vector<int> indices(1);
    std::vector<float> distances(1);
    if (super_occupied_tree_.nearestKSearch(
            query, 1, indices, distances) > 0) {
      minimum = std::min(minimum,
                         std::sqrt(static_cast<double>(distances.front())));
    }
  }
  return minimum;
}

bool SuperExplorationDecider::returnSegmentClear(
    const geometry_msgs::Point& from, const geometry_msgs::Point& to,
    double extra_clearance, bool require_known_start) const {
  const ros::Time freshness_time = freshnessReferenceTime();
  if (super_occupied_stamp_.isZero() ||
      (freshness_time - super_occupied_stamp_).toSec() > return_map_timeout_) {
    if (extra_clearance > 0.0)
      ROS_WARN_THROTTLE(5.0,
          "Forward SUPER-map check has no fresh occupancy: age=%.2f s limit=%.2f s",
          super_occupied_stamp_.isZero() ? -1.0 :
              (freshness_time - super_occupied_stamp_).toSec(),
          return_map_timeout_);
    return false;
  }
  const double distance = std::sqrt(squaredDistance(from, to));
  const int steps = std::max(1, static_cast<int>(std::ceil(
      distance / (0.5 * voxel_resolution_))));
  for (int i = 0; i <= steps; ++i) {
    const double t = static_cast<double>(i) / steps;
    const geometry_msgs::Point sample = [&]() {
      geometry_msgs::Point p;
      p.x = from.x + t * (to.x - from.x);
      p.y = from.y + t * (to.y - from.y);
      p.z = from.z + t * (to.z - from.z);
      return p;
    }();
    const VoxelKey key = positionToKey(sample.x, sample.y, sample.z);
    // SUPER's current occupancy is the clearance authority for return.
    // Frontier's historical binary hits can disagree with it near a wall;
    // still require the complete segment to be observed free here.
    if ((i > 0 || require_known_start) && !isKnownFree(key)) {
      if (extra_clearance > 0.0)
        ROS_WARN_THROTTLE(5.0,
            "Forward SUPER-map check has an unobserved task cell at (%.2f, %.2f, %.2f), sample %d/%d",
            sample.x, sample.y, sample.z, i, steps);
      return false;
    }
    if (!super_occupied_points_->empty()) {
      pcl::PointXYZ query(sample.x, sample.y, sample.z);
      std::vector<int> indices(1);
      std::vector<float> distances(1);
      if (super_occupied_tree_.nearestKSearch(query, 1, indices, distances) > 0 &&
          distances[0] < std::pow(std::max(vehicle_radius_, return_super_radius_) +
                                     return_clearance_margin_ +
                                     std::max(0.0, extra_clearance), 2)) {
        if (extra_clearance > 0.0)
          ROS_WARN_THROTTLE(5.0,
              "Forward SUPER-map check found %.3f m clearance at (%.2f, %.2f, %.2f), sample %d/%d",
              std::sqrt(distances[0]), sample.x, sample.y, sample.z,
              i, steps);
        return false;
      }
    }
  }
  return true;
}

bool SuperExplorationDecider::publishFreshReturnGoal(
    const std::string& reason, bool require_successor) {
  const ros::Time freshness_time = freshnessReferenceTime();
  if (!have_home_ || !dataIsFresh() || super_occupied_stamp_.isZero() ||
      (freshness_time - super_occupied_stamp_).toSec() > return_map_timeout_) {
    ROS_WARN_THROTTLE(2.0,
        "Live return unavailable: home=%d data=%d SUPER map age=%.2f s",
        have_home_, dataIsFresh(), super_occupied_stamp_.isZero() ? -1.0 :
            (freshness_time - super_occupied_stamp_).toSec());
    return false;
  }
  const auto& start = current_pose_.pose.position;
  geometry_msgs::Point home = home_pose_.pose.position;
  if (returning_home_) home.z += return_home_height_offset_;
  const double home_distance = std::sqrt(squaredDistance(start, home));
  // A return transition may happen while dead-end recovery is already moving
  // toward home. Never let a stale/folded breadcrumb or a graph detour reverse
  // that progress. This also protects the first return goal, for which there
  // is no meaningful previous return goal to compare against.
  const auto makes_home_progress = [&](const geometry_msgs::Point& point) {
    return std::sqrt(squaredDistance(point, home)) + 1e-3 < home_distance;
  };
  const double home_arrival_radius = return_home_arrival_radius_ > 0.0
      ? std::min(goal_reached_distance_, return_home_arrival_radius_)
      : std::min(goal_reached_distance_, 0.5 * voxel_resolution_);
  const double minimum_return_displacement = home_distance < 1.5
      ? std::min(1.0, std::max(0.25, home_distance - home_arrival_radius))
      : std::max(1.0, goal_reached_distance_);
  const bool recently_rejected = !rejected_return_point_stamp_.isZero() &&
      (ros::Time::now() - rejected_return_point_stamp_).toSec() < 10.0;
  const auto rejected_near = [&](const geometry_msgs::Point& point) {
    return recently_rejected &&
        squaredDistance(point, rejected_return_point_) < 1.0;
  };
  geometry_msgs::Point selected;
  bool found = false;
  const bool strict_outbound_trace =
      prefer_outbound_breadcrumbs_ && outbound_breadcrumbs_.size() >= 2;
  if (prefer_outbound_breadcrumbs_ && outbound_breadcrumbs_.size() >= 2) {
    // Reverse the observed outbound centreline first. Every proposed shortcut
    // is still checked against both live maps; an old breadcrumb is not proof
    // that the same space remains clear now.
    std::vector<double> lengths(outbound_breadcrumbs_.size(), 0.0);
    for (std::size_t i = 1; i < lengths.size(); ++i) {
      lengths[i] = lengths[i - 1] + std::sqrt(squaredDistance(
          outbound_breadcrumbs_[i - 1].pose.position,
          outbound_breadcrumbs_[i].pose.position));
    }
    double nearest_squared = std::numeric_limits<double>::infinity();
    double nearest_s = 0.0;
    for (std::size_t i = 1; i < lengths.size(); ++i) {
      const auto& a = outbound_breadcrumbs_[i - 1].pose.position;
      const auto& b = outbound_breadcrumbs_[i].pose.position;
      const double dx = b.x - a.x, dy = b.y - a.y, dz = b.z - a.z;
      const double squared = dx * dx + dy * dy + dz * dz;
      if (squared < 1e-8) continue;
      const double t = std::max(0.0, std::min(1.0,
          ((start.x - a.x) * dx + (start.y - a.y) * dy +
           (start.z - a.z) * dz) / squared));
      geometry_msgs::Point projected;
      projected.x = a.x + t * dx;
      projected.y = a.y + t * dy;
      projected.z = a.z + t * dz;
      const double distance = squaredDistance(start, projected);
      if (distance < nearest_squared) {
        nearest_squared = distance;
        nearest_s = lengths[i - 1] + t * (lengths[i] - lengths[i - 1]);
      }
    }
    if (nearest_squared <= 1.0 && nearest_s >= minimum_return_displacement) {
      for (double backward = minimum_return_displacement;
           backward <= std::min(return_segment_length_, nearest_s) + 1e-6;
           backward += 0.25) {
        const double target_s = std::max(0.0, nearest_s - backward);
        const auto upper = std::upper_bound(lengths.begin(), lengths.end(),
                                            target_s);
        const std::size_t index = std::max<std::size_t>(
            1, static_cast<std::size_t>(upper - lengths.begin()));
        const double span = lengths[index] - lengths[index - 1];
        if (span < 1e-8) continue;
        const double t = (target_s - lengths[index - 1]) / span;
        const auto& a = outbound_breadcrumbs_[index - 1].pose.position;
        const auto& b = outbound_breadcrumbs_[index].pose.position;
        geometry_msgs::Point point;
        point.x = a.x + t * (b.x - a.x);
        point.y = a.y + t * (b.y - a.y);
        point.z = a.z + t * (b.z - a.z);
        if (point.z <= maximumReturnHeight() + 1e-6 &&
            !rejected_near(point) && returnSegmentClear(start, point)) {
          selected = point;
          found = true;
        }
      }
    }
  }
  if (found && makes_home_progress(selected) &&
               (!require_successor ||
                std::sqrt(squaredDistance(selected, home)) <
                    std::sqrt(squaredDistance(current_goal_.pose.position,
                                              home)) -
                        0.5 * voxel_resolution_)) {
    geometry_msgs::Point previewed = selected;
    if (!strict_outbound_trace &&
        applyReturnCorridorPreview(start, &previewed) &&
        makes_home_progress(previewed) &&
        (!require_successor ||
         std::sqrt(squaredDistance(previewed, home)) <
             std::sqrt(squaredDistance(current_goal_.pose.position, home)) -
                 0.5 * voxel_resolution_)) {
      selected = previewed;
    }
    geometry_msgs::PoseStamped goal = current_pose_;
    goal.pose.position = selected;
    goal.pose.orientation = yawQuaternion(std::atan2(
        selected.y - start.y, selected.x - start.x));
    return_waypoints_.assign(1, goal);
    return_waypoint_index_ = 0;
    publishGoal(goal, reason + "_breadcrumb", have_active_goal_);
    return true;
  }
  if (strict_outbound_trace) {
    // Do not fall through to the general free-space graph: that graph is a
    // safe route finder but has no route-history semantics and may select an
    // unvisited side branch on the way home. Hold and wait for the recorded
    // outbound trace to become live-map-clear instead.
    ROS_WARN_THROTTLE(
        2.0,
        "Strict breadcrumb return has no live-map-clear inward segment; "
        "holding instead of entering a side branch");
    return false;
  }
  found = false;
  if (home_distance <= return_segment_length_ &&
      !rejected_near(home) && returnSegmentClear(start, home)) {
    selected = home;
    found = true;
  } else {
    struct SearchNode { double cost; VoxelKey key; };
    const auto later = [](const SearchNode& a, const SearchNode& b) {
      return a.cost > b.cost;
    };
    std::priority_queue<SearchNode, std::vector<SearchNode>, decltype(later)>
        pending(later);
    std::unordered_map<VoxelKey, double, VoxelKeyHash> costs;
    std::unordered_map<VoxelKey, VoxelKey, VoxelKeyHash> parents;
    std::unordered_map<VoxelKey, bool, VoxelKeyHash> traversable;
    std::unordered_map<VoxelKey, double, VoxelKeyHash> clearances;
    // The surveyed safe lane can be narrower than one 0.25 m decision voxel.
    // Search on a half-voxel lattice and use the actual continuous position
    // for the root; rounding the root toward a roof can erase every route.
    if (!returnSegmentClear(start, start)) {
      ROS_WARN_THROTTLE(2.0,
          "Live return root is not verified clear at (%.2f, %.2f, %.2f): task_free=%d",
          start.x, start.y, start.z,
          isKnownFree(positionToKey(start.x, start.y, start.z)));
      return false;
    }
    const double return_resolution = 0.5 * voxel_resolution_;
    const VoxelKey origin{
        static_cast<int>(std::floor(start.x / return_resolution)),
        static_cast<int>(std::floor(start.y / return_resolution)),
        static_cast<int>(std::floor(start.z / return_resolution))};
    const auto return_point = [&](const VoxelKey& key) {
      geometry_msgs::Point point;
      point.x = (static_cast<double>(key.x) + 0.5) * return_resolution;
      point.y = (static_cast<double>(key.y) + 0.5) * return_resolution;
      point.z = (static_cast<double>(key.z) + 0.5) * return_resolution;
      return point;
    };
    costs[origin] = 0.0;
    pending.push({0.0, origin});
    VoxelKey best = origin;
    double best_score = -std::numeric_limits<double>::infinity();
    const double search_radius = std::max(4.0, return_segment_length_ + 1.0);
    std::size_t expanded = 0;
    std::size_t known_free_nodes = 0;
    std::size_t super_clear_nodes = 0;
    std::size_t safe_edges = 0;
    while (!pending.empty() && expanded < 6000) {
      const SearchNode node = pending.top();
      pending.pop();
      if (node.cost > costs[node.key] + 1e-8) continue;
      ++expanded;
      const auto point = node.key == origin ? start : return_point(node.key);
      const double displacement = std::sqrt(squaredDistance(start, point));
      if (displacement >= minimum_return_displacement &&
          displacement <= return_segment_length_ &&
          point.z <= maximumReturnHeight() + 1e-6) {
        const double progress = home_distance -
            std::sqrt(squaredDistance(point, home));
        const double separation = std::min(2.0, outboundSeparation(point));
        const auto clearance = clearances.find(node.key);
        const double clearance_bonus = clearance == clearances.end() ? 0.0 :
            2.0 * std::min(0.5, std::max(0.0, clearance->second -
                                            std::max(vehicle_radius_, return_super_radius_)));
        const double score = 2.0 * progress +
                             0.5 * clearance_bonus -
                             1.5 * separation -
                             0.15 * node.cost;
        // Prefer the surveyed outbound lane. A lateral detour remains
        // available when the live map blocks that lane.
        if (progress >= 0.15 && !rejected_near(point) &&
            score > best_score) {
          best_score = score;
          best = node.key;
        }
      }
      for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
          for (int dz = -1; dz <= 1; ++dz) {
            if (std::abs(dx) + std::abs(dy) + std::abs(dz) != 1) continue;
            const VoxelKey next{node.key.x + dx, node.key.y + dy,
                                node.key.z + dz};
            const auto next_point = return_point(next);
            if (std::sqrt(squaredDistance(start, next_point)) > search_radius ||
                std::abs(next_point.z - start.z) > 0.75 ||
                (next_point.z > maximumReturnHeight() + 1e-6 &&
                 next_point.z >= point.z - 1e-6))
              continue;
            auto clear = traversable.find(next);
            if (clear == traversable.end()) {
              bool valid = isKnownFree(positionToKey(
                  next_point.x, next_point.y, next_point.z));
              if (valid) ++known_free_nodes;
              if (valid && !super_occupied_points_->empty()) {
                pcl::PointXYZ query(next_point.x, next_point.y, next_point.z);
                std::vector<int> indices(1);
                std::vector<float> distances(1);
                if (super_occupied_tree_.nearestKSearch(
                        query, 1, indices, distances) > 0) {
                  clearances[next] = std::sqrt(distances[0]);
                  valid = clearances[next] >=
                      std::max(vehicle_radius_, return_super_radius_) +
                          return_clearance_margin_;
                } else {
                  valid = false;
                }
              }
              if (valid) ++super_clear_nodes;
              clear = traversable.emplace(next, valid).first;
            }
            if (!clear->second || !returnSegmentClear(point, next_point)) continue;
            ++safe_edges;
            const double step = return_resolution *
                std::sqrt(static_cast<double>(dx * dx + dy * dy + dz * dz));
            const double separation = std::min(2.0,
                outboundSeparation(next_point));
            const double candidate = node.cost +
                step * (1.0 + 0.6 * separation);
            const auto old = costs.find(next);
            if (old == costs.end() || candidate + 1e-8 < old->second) {
              costs[next] = candidate;
              parents[next] = node.key;
              pending.push({candidate, next});
            }
          }
        }
      }
    }
    if (best == origin) {
      ROS_WARN_THROTTLE(2.0,
          "Live return search found no progressive point: expanded=%zu known=%zu SUPER-clear=%zu edges=%zu start=(%.2f,%.2f,%.2f)",
          expanded, known_free_nodes, super_clear_nodes, safe_edges,
          start.x, start.y, start.z);
      return false;
    }
    std::vector<VoxelKey> route;
    for (VoxelKey key = best; !(key == origin); key = parents.at(key)) {
      route.push_back(key);
    }
    std::reverse(route.begin(), route.end());
    // A shortcut is allowed only when the complete straight segment is clear
    // in both maps; otherwise publish the farthest visible point on the route.
    std::size_t eligible_points = 0;
    for (const auto& key : route) {
      const auto point = return_point(key);
      const double distance = std::sqrt(squaredDistance(start, point));
      if (distance > return_segment_length_) break;
      if (distance >= minimum_return_displacement &&
          !rejected_near(point)) {
        ++eligible_points;
      } else {
        continue;
      }
      if (returnSegmentClear(start, point)) {
        selected = point;
        found = true;
      }
    }
    if (!found) {
      ROS_WARN_THROTTLE(2.0,
          "Live return graph route has no directly clear goal: expanded=%zu route=%zu eligible=%zu best=(%.2f,%.2f,%.2f)",
          expanded, route.size(), eligible_points,
          return_point(best).x, return_point(best).y, return_point(best).z);
    }
  }
  if (!found) return false;
  geometry_msgs::Point previewed = selected;
  if (std::sqrt(squaredDistance(selected, home)) >
          home_arrival_radius + 1e-6 &&
      applyReturnCorridorPreview(start, &previewed) &&
      makes_home_progress(previewed)) {
    selected = previewed;
  }
  if (!makes_home_progress(selected)) return false;
  if (require_successor &&
      std::sqrt(squaredDistance(selected, home)) >=
          std::sqrt(squaredDistance(current_goal_.pose.position, home)) -
              0.5 * voxel_resolution_) {
    return false;
  }
  geometry_msgs::PoseStamped goal = current_pose_;
  goal.pose.position = selected;
  goal.pose.orientation = yawQuaternion(std::atan2(
      selected.y - start.y, selected.x - start.x));
  return_waypoints_.assign(1, goal);
  return_waypoint_index_ = 0;
  publishGoal(goal, reason, have_active_goal_);
  return true;
}

void SuperExplorationDecider::beginReturnHome(
    const std::string& reason, bool prefer_outbound_breadcrumbs) {
  if (!have_home_ || returning_home_ || mission_finished_) {
    return;
  }
  // Task 4 return is route replay, never a second exploration pass. Even a
  // safety-triggered return must retrace the accepted outbound centreline
  // instead of entering another connected branch that merely reduces the
  // Euclidean distance to home.
  prefer_outbound_breadcrumbs_ = prefer_outbound_breadcrumbs ||
      mission_profile_ == ExplorationMissionProfile::kTunnel;
  if (prefer_outbound_breadcrumbs_ && !outbound_breadcrumbs_.empty() &&
      squaredDistance(outbound_breadcrumbs_.back().pose.position,
                      current_pose_.pose.position) > 0.25 &&
      squaredDistance(current_pose_.pose.position,
                      home_pose_.pose.position) >
          squaredDistance(outbound_breadcrumbs_.back().pose.position,
                          home_pose_.pose.position) + 1e-3) {
    // Append only while the vehicle is still extending the outbound trace.
    // Appending an already-backtracking pose to the old outbound tail folds
    // the polyline and makes reverse traversal command an outward leg.
    outbound_breadcrumbs_.push_back(current_pose_);
  }
  returning_home_ = true;
  dead_end_backtracking_ = false;
  clearExplorationGoalQueue();
  home_arrival_streak_ = 0;
  return_waypoints_.clear();
  return_waypoint_index_ = 0;
  // The first return leg reverses the route tangent by roughly 180 degrees.
  // Do not retarget the still-moving outbound trajectory: that made position
  // and yaw reverse together and produced sustained sideways flight in the
  // 2026-10-23 Task-4 run. A cancel preserves SUPER's certified braking
  // reference; the following SET is therefore planned from rest, where SUPER
  // inserts its constant-position yaw-prealignment segment. Later return
  // handovers remain continuous because the aircraft already faces home.
  if (have_active_goal_) {
    cancelActiveGoal("return_yaw_prealignment");
  }
  if (!publishFreshReturnGoal(reason)) {
    cancelActiveGoal("waiting_for_live_return_route");
    publishStatus("RETURN_ROUTE_WAIT", "waiting for a live-map-clear return segment");
  }
  std_msgs::Bool message;
  message.data = true;
  returning_publisher_.publish(message);
  publishStatus("RETURNING", reason);
}

bool SuperExplorationDecider::batteryReadyForMissionStart(
    std::string* detail) const {
  if (!battery_start_guard_enabled_) {
    if (detail != nullptr) detail->clear();
    return true;
  }
  std::ostringstream status;
  if (!std::isfinite(battery_voltage_) || battery_voltage_ <= 0.0) {
    status << "waiting for a valid battery voltage";
  } else if (battery_percentage_return_enabled_ &&
             (!have_battery_ || !std::isfinite(battery_percentage_) ||
              battery_percentage_ < 0.0)) {
    status << "waiting for a valid battery percentage";
  } else if (battery_percentage_return_enabled_ &&
             battery_percentage_ + 1e-6 <
             battery_start_minimum_percentage_) {
    status << std::fixed << std::setprecision(0)
           << "battery " << 100.0 * battery_percentage_
           << "% is below the exploration start minimum "
           << 100.0 * battery_start_minimum_percentage_ << "%";
  } else if (battery_voltage_ + 1e-6 < battery_start_minimum_voltage_) {
    status << std::fixed << std::setprecision(2)
           << "battery " << battery_voltage_
           << " V is below the exploration start minimum "
           << battery_start_minimum_voltage_ << " V";
  } else {
    if (detail != nullptr) detail->clear();
    return true;
  }
  if (detail != nullptr) *detail = status.str();
  return false;
}

void SuperExplorationDecider::decisionTimerCallback(const ros::TimerEvent&) {
  const auto decision_wall_start = std::chrono::steady_clock::now();
  decision_snapshot_time_ = ros::Time::now();
  decision_vehicle_clearance_cache_.clear();
  decision_preferred_clearance_cache_.clear();
  decision_clearance_cache_active_ = true;
  rebuildDecisionClearanceIndex();
  decision_forward_route_evaluated_ = false;
  decision_forward_route_available_ = false;
  struct DecisionSnapshotReset {
    ros::Time* stamp;
    bool* clearance_cache_active;
    std::chrono::steady_clock::time_point wall_start;
    ~DecisionSnapshotReset() {
      const double elapsed = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - wall_start).count();
      if (elapsed > 0.45) {
        ROS_WARN_THROTTLE(
            1.0,
            "Exploration decision callback took %.3f s (target period 0.500 s)",
            elapsed);
      }
      *stamp = ros::Time();
      *clearance_cache_active = false;
    }
  } reset_snapshot{&decision_snapshot_time_,
                   &decision_clearance_cache_active_, decision_wall_start};
  const ros::Time now = decision_snapshot_time_;

  if (!enabled_) {
    publishStatus("DISABLED");
    return;
  }
  if (!dataIsFresh() || !have_home_) {
    // A dropped synchronized cloud should not cancel an already certified
    // SUPER trajectory while odometry and commands are still arriving. Wait
    // briefly for the map stream to recover, but issue no new goal on old data.
    const bool super_map_fresh = super_occupied_stamp_.isZero() ||
        (now - super_occupied_stamp_).toSec() <= 3.0;
    if (have_home_ && have_active_goal_ && have_data_ && super_map_fresh &&
        !last_sync_time_.isZero() &&
        (ros::Time::now() - last_sync_time_).toSec() <=
            data_timeout_ + active_goal_data_grace_) {
      publishStatus("WAIT_SENSOR_RECOVERY",
                    "keeping the active certified trajectory during a short cloud gap");
      return;
    }
    cancelActiveGoal("wait_data");
    publishStatus("WAIT_DATA", "Omni-LIVO synchronized data is stale or absent");
    return;
  }
  if (!exploration_started_ && !returning_home_ &&
      !dead_end_backtracking_) {
    std::string battery_detail;
    if (!batteryReadyForMissionStart(&battery_detail)) {
      // Keep the aircraft at the scheduler's already-established hover. A
      // low pack must be replaced/recharged instead of spending the return
      // reserve on an exploration that cannot reach the observed end wall.
      cancelActiveGoal("start_battery_guard");
      publishStatus("WAIT_START_BATTERY", battery_detail);
      return;
    }
  }
  if (battery_percentage_return_enabled_ && have_battery_ &&
      battery_percentage_ >= 0.0) {
    // BatteryState.percentage is float32. Treat its representation of an
    // exact configured threshold (for example 0.20) as equal.
    if (battery_percentage_ <= battery_return_threshold_ + 1e-6) {
      if (low_battery_since_.isZero()) low_battery_since_ = now;
      if ((now - low_battery_since_).toSec() >=
          battery_return_confirm_duration_) {
        low_percentage_return_requested_ = true;
      }
    } else if (!low_percentage_return_requested_ &&
               battery_percentage_ >=
                   battery_return_threshold_ + battery_return_hysteresis_) {
      low_battery_since_ = ros::Time();
    }
  }
  if (low_percentage_return_requested_ || low_voltage_return_requested_) {
    return_requested_ = true;
  }
  if (return_requested_) {
    const char* reason = low_voltage_return_requested_
        ? "battery voltage below return threshold"
        : (low_percentage_return_requested_
               ? "battery percentage below return threshold"
               : "external return request");
    beginReturnHome(reason, true);
  }

  // The occupancy map can reveal a roof or wall after a command has been
  // committed. Start a verified return while the current position still has
  // enough space for SUPER's 1 m corridor, before the next frontier carries
  // the aircraft into a narrower pocket with no valid return root.
  if (min_forward_super_clearance_ > return_super_radius_ &&
      exploration_started_ && !returning_home_ &&
      !dead_end_backtracking_ && !mission_finished_ &&
      !super_occupied_stamp_.isZero() &&
      (now - super_occupied_stamp_).toSec() <=
          return_map_timeout_ && !super_occupied_points_->empty()) {
    const auto& position = current_pose_.pose.position;
    const double dx = position.x - home_pose_.pose.position.x;
    const double dy = position.y - home_pose_.pose.position.y;
    const double progress = std::cos(mission_heading_yaw_) * dx +
                            std::sin(mission_heading_yaw_) * dy;
    if (progress >= min_clearance_return_progress_) {
      const pcl::PointXYZ query(position.x, position.y, position.z);
      std::vector<int> indices(1);
      std::vector<float> squared_distances(1);
      if (super_occupied_tree_.nearestKSearch(
              query, 1, indices, squared_distances) > 0 &&
          squared_distances[0] < min_forward_super_clearance_ *
                                     min_forward_super_clearance_) {
        ROS_WARN("Forward clearance %.3f m is approaching SUPER's %.2f m safety radius; returning before the route closes",
                 std::sqrt(squared_distances[0]), return_super_radius_);
        beginReturnHome("forward safety clearance narrowing");
      }
    }
  }

  // Preserve an in-progress closure confirmation for this entire decision
  // cycle. evaluateMapClosure() below may reset the streak on one noisy scan;
  // that scan must not immediately turn a pending confirmation into a generic
  // stalled-route return.
  const bool closure_confirmation_pending =
      use_map_closure_completion_ && map_closure_complete_streak_ > 0;

  if (returning_home_) {
    // New return legs are checked against the current SUPER occupancy map
    // before publication. During execution SUPER owns the certified curved
    // trajectory and rechecks it on each replan. A straight line from the
    // tracking pose to the goal is not that trajectory; cancelling on that
    // line repeatedly stranded the vehicle near a 0.97 m map boundary.
    geometry_msgs::Point home = home_pose_.pose.position;
    home.z += return_home_height_offset_;
    if (return_home_arrival_radius_ > 0.0) {
      const double radius = std::min(goal_reached_distance_,
                                     return_home_arrival_radius_);
      if (squaredDistance(current_pose_.pose.position, home) <=
          radius * radius) {
        ++home_arrival_streak_;
      } else {
        home_arrival_streak_ = 0;
      }
      if (home_arrival_streak_ >= 2) {
        returning_home_ = false;
        mission_finished_ = true;
        cancelActiveGoal("home_reached");
        std_msgs::Bool message;
        message.data = true;
        finished_publisher_.publish(message);
        message.data = false;
        returning_publisher_.publish(message);
        publishStatus("COMPLETE", "home launch area reached");
        return;
      }
    }
    if (active_goal_id_ == 0) {
      if (ros::Time::now() >= rejected_goal_retry_after_)
        publishFreshReturnGoal("return_replan");
      if (active_goal_id_ == 0) {
        publishStatus("RETURN_ROUTE_WAIT", "no verified return segment");
        return;
      }
    }
    const bool goal_is_home =
        squaredDistance(current_goal_.pose.position, home) < 1e-6;
    const double return_arrival_distance = goal_is_home
        ? (return_home_arrival_radius_ > 0.0
               ? std::min(goal_reached_distance_, return_home_arrival_radius_)
               : std::min(goal_reached_distance_, 0.5 * voxel_resolution_))
      // Do not abandon a clearance-increasing intermediate leg while the
      // aircraft is still half a metre from its centred target. One decision
      // voxel proves actual arrival without forcing a full stop at every
      // rolling handover.
      : std::min(goal_reached_distance_, voxel_resolution_);
    const double current_home_distance = std::sqrt(squaredDistance(
        current_pose_.pose.position, home));
    const double goal_home_distance = std::sqrt(squaredDistance(
        current_goal_.pose.position, home));
    // SUPER may accept a return goal several seconds after it was published.
    // The vehicle can continue along the previously committed inward path in
    // the meantime and pass the new endpoint laterally. A pure spherical
    // arrival test then leaves the accepted endpoint behind the aircraft and
    // makes SUPER turn outward to hit it. This occurred in the 2026-10-24
    // flight: goal 47 was accepted after the vehicle was already about 1.1 m
    // closer to home than that goal. Once an accepted intermediate endpoint
    // is at least one decision voxel behind in home-distance, advance to a
    // newly verified inward leg instead of commanding a U-turn. The final
    // home goal deliberately retains the positional arrival check.
    const bool intermediate_goal_passed =
        !goal_is_home &&
        current_home_distance + voxel_resolution_ <= goal_home_distance;
    // Reaching a candidate coordinate is not evidence that SUPER ever
    // committed a trajectory to that goal. During a rest handover the old
    // terminal command may still be held, and advancing again here can
    // preempt planning before it succeeds.
    const bool return_waypoint_reached =
        accepted_goal_id_ == active_goal_id_ &&
        (squaredDistance(current_pose_.pose.position,
                         current_goal_.pose.position) <=
             return_arrival_distance * return_arrival_distance ||
         intermediate_goal_passed);
    if (!return_waypoint_reached && !goal_sent_time_.isZero() &&
        (ros::Time::now() - goal_sent_time_).toSec() > goal_timeout_ &&
        ros::Time::now() >= rejected_goal_retry_after_) {
      rejected_return_point_ = current_goal_.pose.position;
      rejected_return_point_stamp_ = ros::Time::now();
      if (!publishFreshReturnGoal("return_goal_timeout")) {
        rejected_goal_retry_after_ = ros::Time::now() + ros::Duration(2.0);
        publishStatus("RETURN_ROUTE_WAIT",
                      "stalled return goal; waiting for a new clear segment");
      }
      return;
    }
    // Retarget an accepted intermediate leg while SUPER still has a moving
    // trajectory. The new segment must be verified against both live maps
    // and make progress beyond the old waypoint. Keep the final home leg so
    // arrival is confirmed at the actual launch position.
    if (!return_waypoint_reached && !goal_is_home &&
        accepted_goal_id_ == active_goal_id_ &&
        std::sqrt(squaredDistance(current_pose_.pose.position,
                                  current_goal_.pose.position)) <=
            std::min(return_goal_handover_distance_,
                     return_segment_length_ - 0.5) &&
        std::sqrt(squaredDistance(current_pose_.pose.position,
                                  current_goal_.pose.position)) <=
            active_goal_initial_distance_ - 0.75 &&
        publishFreshReturnGoal("return_handover", true)) {
      publishStatus("RETURN_HANDOVER", "verified successor sent before intermediate stop");
      return;
    }
    if (return_waypoint_reached && !goal_is_home) {
      if (!publishFreshReturnGoal("return_live_map")) {
        cancelActiveGoal("return_route_unavailable");
        publishStatus("RETURN_ROUTE_WAIT", "waiting for a live-map-clear return segment");
      }
      return;
    }
    if (return_waypoint_reached && goal_is_home) {
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
      publishStatus("RETURNING", "following verified return segment");
    }
    return;
  }
  if (dead_end_backtracking_) {
    if (active_goal_id_ == 0) {
      if (ros::Time::now() >= rejected_goal_retry_after_)
        publishFreshReturnGoal("dead_end_replan");
      if (active_goal_id_ == 0) {
        publishStatus("RETURN_ROUTE_WAIT", "no verified dead-end return segment");
        return;
      }
    }
    const bool goal_is_home = squaredDistance(
        current_goal_.pose.position, home_pose_.pose.position) < 1e-6;
    const double backtrack_arrival_distance = goal_is_home
        ? std::min(goal_reached_distance_, 0.5 * voxel_resolution_)
        : std::min(goal_reached_distance_, voxel_resolution_);
    if (!goal_sent_time_.isZero() &&
        squaredDistance(current_pose_.pose.position,
                        current_goal_.pose.position) >
            backtrack_arrival_distance * backtrack_arrival_distance &&
        (ros::Time::now() - goal_sent_time_).toSec() > goal_timeout_ &&
        ros::Time::now() >= rejected_goal_retry_after_) {
      rejected_return_point_ = current_goal_.pose.position;
      rejected_return_point_stamp_ = ros::Time::now();
      if (!publishFreshReturnGoal("dead_end_goal_timeout")) {
        rejected_goal_retry_after_ = ros::Time::now() + ros::Duration(2.0);
        publishStatus("RETURN_ROUTE_WAIT",
                      "stalled backtrack goal; waiting for a new clear segment");
      }
      return;
    }
    if (squaredDistance(current_pose_.pose.position,
                        current_goal_.pose.position) <=
        backtrack_arrival_distance * backtrack_arrival_distance) {
      if (!goal_is_home) {
        if (!publishFreshReturnGoal("dead_end_live_map")) {
          cancelActiveGoal("dead_end_route_unavailable");
          publishStatus("RETURN_ROUTE_WAIT", "waiting for a live-map-clear return segment");
        }
        return;
      }
      cancelActiveGoal("dead_end_origin_reached");
      dead_end_backtracking_ = false;
      return_waypoints_.clear();
      return_waypoint_index_ = 0;
      outbound_breadcrumbs_.clear();
      outbound_breadcrumbs_.push_back(current_pose_);
      front_obstacle_streak_ = 0;
      side_wall_missing_streak_ = 0;
      exploration_phase_ = ExplorationPhase::kForwardPriority;
      publishStatus("DEAD_END_BACKTRACK_COMPLETE",
                    "mission origin reached; exploring the opposite side");
    } else {
      publishStatus("DEAD_END_BACKTRACK",
                    "following verified live-map route to mission origin");
    }
    return;
  }
  if (mission_finished_) {
    publishStatus("COMPLETE");
    return;
  }

  const bool committed_tunnel_branch =
      mission_profile_ == ExplorationMissionProfile::kTunnel &&
      tunnel_route_locked_;
  const auto coverage_start = std::chrono::steady_clock::now();
  ThreeWallCoverage coverage;
  if (!committed_tunnel_branch) {
    coverage = evaluateThreeWallCoverage();
  }
  const auto coverage_done = std::chrono::steady_clock::now();
  VoxelSet reachable;
  std::vector<FrontierCandidate> candidates;
  if (committed_tunnel_branch) {
    // Branch completion depends on a coherent connected extension, not on
    // enumerating every raw unknown boundary in the full map.
    reachable = reachableFreeVoxels();
  } else {
    candidates = findFrontiers(reachable);
  }
  const auto frontiers_done = std::chrono::steady_clock::now();
  if (!committed_tunnel_branch) {
    updateCorridorPreviewPreference(reachable);
  }
  const auto preview_done = std::chrono::steady_clock::now();
  // Diagnose the observation-to-reachability boundary at a low rate.  The
  // forward planner must not silently wait when observed free space exists
  // ahead but its connected component is lost by the flood fill.
  static ros::Time last_reachability_diagnostic;
  if (!committed_tunnel_branch && have_home_ && !free_ray_topic_.empty() &&
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
          !missionObservationHeightAllowed(
              point.z, 0.5 * voxel_resolution_)) continue;
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
    if (reachable.size()<20 && free_ahead-reachable_ahead>2.0) {
      int unknown=0,blocked=0,clearance=0,band=0;
      for(int dx=-1;dx<=1;++dx) for(int dy=-1;dy<=1;++dy) for(int dz=-1;dz<=1;++dz) {
        if(!dx&&!dy&&!dz) continue;
        const VoxelKey n{current_key.x+dx,current_key.y+dy,current_key.z+dz};
        const auto p=keyToPoint(n);
        if(isOccupied(n)) ++blocked;
        else if(!isKnownFree(n)) ++unknown;
        else if(!isClearForVehicle(n)) ++clearance;
        else if(!missionObservationHeightAllowed(p.z, voxel_resolution_)) ++band;
      }
      ROS_WARN("REACHABILITY_DISCONNECTED_CHECK key=(%d,%d,%d) reach=%zu unknown=%d occupied=%d clearance=%d height_band=%d; no obstacles erased",
          current_key.x,current_key.y,current_key.z,reachable.size(),unknown,blocked,clearance,band);
    }
  }
  MapClosureStatus closure;
  if (!committed_tunnel_branch) {
    closure = evaluateMapClosure(
        candidates, reachable, coverage.end_wall_found);
  }
  const auto closure_done = std::chrono::steady_clock::now();
  const double analyzed_seconds = std::chrono::duration<double>(
      closure_done - coverage_start).count();
  if (!committed_tunnel_branch && analyzed_seconds > 0.35) {
    ROS_WARN_THROTTLE(
        1.0,
        "Exploration planning phases %.3f s: coverage=%.3f reachable/frontier=%.3f preview=%.3f closure=%.3f voxels=%zu reachable=%zu candidates=%zu",
        analyzed_seconds,
        std::chrono::duration<double>(coverage_done - coverage_start).count(),
        std::chrono::duration<double>(frontiers_done - coverage_done).count(),
        std::chrono::duration<double>(preview_done - frontiers_done).count(),
        std::chrono::duration<double>(closure_done - preview_done).count(),
        voxels_.size(), reachable.size(), candidates.size());
  }
  const bool completion_prerequisites =
      exploration_started_ &&
      reached_goal_count_ >= min_goals_before_complete_ &&
      !first_data_time_.isZero() &&
      (ros::Time::now() - first_data_time_).toSec() >= min_data_duration_;
  if (completion_prerequisites && task1EndWallReturnReady(coverage)) {
    std_msgs::Bool message;
    message.data = true;
    model_complete_publisher_.publish(message);
    std::ostringstream detail;
    detail << std::fixed << std::setprecision(1)
           << "Task-1 end wall confirmed "
           << std::max(0.0,
                       coverage.end_depth - coverage.vehicle_progress)
           << " m ahead; returning at the configured "
           << task1_end_wall_return_distance_ << " m stand-off";
    publishStatus("MODEL_COMPLETE", detail.str());
    beginReturnHome("Task-1 end wall reached the 5 m return boundary", true);
    return;
  }
  if (!committed_tunnel_branch && require_three_wall_completion_ &&
      coverage.complete &&
      completion_prerequisites) {
    three_wall_complete_streak_ = std::min(
        three_wall_complete_streak_ + 1, three_wall_confirm_cycles_);
  } else {
    three_wall_complete_streak_ = 0;
  }
  if (!committed_tunnel_branch && use_map_closure_completion_ &&
      closure.complete &&
      completion_prerequisites) {
    map_closure_complete_streak_ = std::min(
        map_closure_complete_streak_ + 1, map_closure_confirm_cycles_);
  } else {
    map_closure_complete_streak_ = 0;
  }
  if (!committed_tunnel_branch) {
    publishCoverageStatus(coverage, closure);
  }
  const bool hard_safe_route_timeout_confirmed =
      !hard_safe_route_unavailable_since_.isZero() &&
      (now - hard_safe_route_unavailable_since_).toSec() >=
          no_frontier_timeout_;
  if (!committed_tunnel_branch && use_map_closure_completion_ &&
      map_closure_complete_streak_ >= map_closure_confirm_cycles_ &&
      hard_safe_route_timeout_confirmed) {
    std_msgs::Bool message;
    message.data = true;
    model_complete_publisher_.publish(message);
    publishStatus("MODEL_COMPLETE",
                  "point-cloud boundary closed and map converged");
    beginReturnHome("map closure confirmed");
    return;
  }
  if (!committed_tunnel_branch && require_three_wall_completion_ &&
      three_wall_complete_streak_ >= three_wall_confirm_cycles_) {
    std_msgs::Bool message;
    message.data = true;
    model_complete_publisher_.publish(message);
    publishStatus("MODEL_COMPLETE",
                  "continuous left, right, and end walls confirmed");
    beginReturnHome("three-wall model complete");
    return;
  }

  if (!committed_tunnel_branch) {
    publishVisualization(candidates);
    if (!candidates.empty()) {
      last_frontier_time_ = ros::Time::now();
    }
    updateExplorationPhase(candidates);
  }
  if (!exploration_goal_queue_.empty()) {
    pruneExplorationGoalQueue(reachable);
  }

  if (have_active_goal_) {
    const bool progress_odometry_fresh = have_progress_odometry_ &&
        !last_progress_odometry_time_.isZero() &&
        (now - last_progress_odometry_time_).toSec() <= data_timeout_;
    const geometry_msgs::Point& progress_position = progress_odometry_fresh
        ? latest_progress_pose_.pose.position
        : current_pose_.pose.position;
    const double distance = std::sqrt(squaredDistance(
        progress_position, current_goal_.pose.position));
    const double goal_age = (now - goal_sent_time_).toSec();
    const bool front_obstacle_confirmed =
        front_obstacle_streak_ >= front_obstacle_confirm_frames_;
    const bool active_goal_is_forward = isForwardCandidate(
        FrontierCandidate{VoxelKey{}, current_goal_, 0.0, 0});
    const bool forward_goal_handover =
        (active_goal_allows_rolling_handover_ ||
         active_goal_is_forward_lookahead_) &&
        accepted_goal_id_ == active_goal_id_ &&
        !active_goal_is_end_approach_ &&
        distance <= forward_goal_handover_distance_ &&
        distance <= active_goal_initial_distance_ - 0.5;
    const bool preempt_blocked_forward_goal =
        exploration_phase_ == ExplorationPhase::kFrontierFallback &&
        front_obstacle_confirmed && !active_goal_is_end_approach_ &&
        !active_goal_is_forward_lookahead_ &&
        !active_goal_is_blocked_frontier_route_ &&
        isForwardCandidate(
            FrontierCandidate{VoxelKey{}, current_goal_, 0.0, 0});
    // A confirmed obstacle is not itself a reason to force a stop. The
    // handover search only succeeds for a farther, fully observed and clear
    // successor, so a wall still holds the current goal when no such leg
    // exists while a genuine side passage stays trajectory-continuous.
    if (forward_goal_handover && tryForwardHandover(reachable)) {
      publishStatus("GOAL_HANDOVER", "verified successor sent while previous goal remains active");
      return;
    }
    // A one-metre handover radius is not an arrival tolerance. For short
    // forward/lateral observation steps use one decision voxel as the arrival
    // radius. Half a voxel was too narrow for the 2 Hz task-level decision
    // loop: the vehicle could pass briefly inside that radius between ticks,
    // then settle just outside it and wait until goal timeout even though
    // SUPER had already completed its smooth trajectory.
    const bool forward_target =
        !front_obstacle_confirmed && active_goal_is_forward &&
        !active_goal_is_end_approach_;
    // A lateral detour can be only one 0.5 m decision voxel away.  Applying
    // the normal 1 m frontier tolerance caused it to be cancelled at the
    // next 2 Hz decision tick before the aircraft had moved, followed by an
    // endless cancel/replan loop at obstacles and T junctions. Keep lateral
    // detours and front-wall approaches at half a voxel, but allow unobstructed
    // straight look-ahead goals one full voxel so the 2 Hz task loop cannot
    // miss a brief valid arrival between decision ticks.
    const bool tight_short_goal =
        active_goal_is_lateral_detour_ ||
        (active_goal_is_forward_lookahead_ && front_obstacle_confirmed);
    const bool relaxed_forward_goal =
        !tight_short_goal &&
        (forward_target || active_goal_is_forward_lookahead_);
    const double arrival_distance = tight_short_goal
        ? std::min(goal_reached_distance_, 0.5 * voxel_resolution_)
        : (relaxed_forward_goal
               ? std::min(goal_reached_distance_, voxel_resolution_)
               : goal_reached_distance_);
    if (distance <= arrival_distance) {
      covered_goals_.push_back(current_goal_.pose.position);
      ++reached_goal_count_;
      // Nominal arrival retains the certified stop reference until next SET.
      have_active_goal_ = false;
      active_goal_id_ = 0;
      accepted_goal_id_ = 0;
      publishStatus("GOAL_REACHED", "selecting next frontier");
    } else if (preempt_blocked_forward_goal) {
      covered_goals_.push_back(current_goal_.pose.position);
      rememberFailedGoal(current_goal_.pose.position);
      cancelActiveGoal("front_wall_preempt");
      publishStatus("GOAL_PREEMPTED",
                    "confirmed front wall blocks the forward goal");
    } else if (goal_age > goal_timeout_) {
      covered_goals_.push_back(current_goal_.pose.position);
      rememberFailedGoal(current_goal_.pose.position);
      cancelActiveGoal("goal_timeout");
      publishStatus("GOAL_TIMEOUT", "discarding unreachable candidate");
    } else {
      publishStatus("EXPLORING",
                    "active frontier goal published; awaiting odometry progress");
      return;
    }
  }

  if (!have_active_goal_ && rejected_goal_retry_after_.isZero() &&
      publishNextQueuedExplorationGoal(reachable)) {
    publishStatus("GOAL_HANDOVER",
                  "latest-map-verified queued exploration successor sent");
    return;
  }

  if (!have_active_goal_ && !rejected_goal_retry_after_.isZero()) {
    if (ros::Time::now() < rejected_goal_retry_after_) {
      publishStatus("WAIT_SUPER_MAP", "goal rejected while SUPER map catches up");
      return;
    }
    rejected_goal_retry_after_ = ros::Time(0);
    const bool retry_route_published = tunnel_route_locked_
        ? publishConnectedTurnContinuation(reachable)
        : (publishLateralDetourGoal(reachable, candidates) ||
           publishForwardLookaheadGoal(reachable) ||
           publishConnectedTurnContinuation(reachable) ||
           selectAndPublishFrontier(candidates, reachable));
    if (retry_route_published) {
      publishStatus("EXPLORING", "retrying a reachable goal after SUPER rejection");
      return;
    }
  }

  // After Task 4 has entered an accepted bend, the tunnel tangent is the
  // route contract. Entry-axis look-ahead and arbitrary residual frontier
  // cells are no longer valid alternatives: they caused the real aircraft to
  // cross back and forth at the terminal junction and continuously reset the
  // completion timer. Only a newly observed, body-clear connected extension
  // may keep the branch open.
  if (!have_active_goal_ && tunnel_route_locked_) {
    if (publishConnectedTurnContinuation(reachable)) {
      publishStatus("EXPLORING",
                    "following the committed Task-4 tunnel branch");
      return;
    }
    if (hard_safe_route_unavailable_since_.isZero()) {
      hard_safe_route_unavailable_since_ = now;
    }
    const double wait =
        (now - hard_safe_route_unavailable_since_).toSec();
    if (completion_prerequisites && wait >= no_frontier_timeout_) {
      std_msgs::Bool message;
      message.data = true;
      model_complete_publisher_.publish(message);
      publishStatus(
          "MODEL_COMPLETE",
          "committed tunnel branch has no hard-safe forward continuation");
      beginReturnHome(
          "Task-4 tunnel branch exhausted; returning on outbound trace",
          true);
    } else {
      std::ostringstream detail;
      detail << std::fixed << std::setprecision(1)
             << "committed branch has no coherent extension; confirming "
             << std::max(0.0, no_frontier_timeout_ - wait)
             << " s before return";
      publishStatus("WAIT_TUNNEL_BRANCH_END", detail.str());
    }
    return;
  }

  bool executable_route_published_this_cycle = false;
  if (!have_active_goal_ && require_three_wall_completion_) {
    executable_route_published_this_cycle = publishEndApproachGoal(coverage);
  }
  if (executable_route_published_this_cycle) {
    publishStatus("APPROACHING_END_WALL",
                  "three-wall map seen; moving to configured stand-off");
    return;
  }

  const bool tunnel_bend_detected =
      mission_profile_ == ExplorationMissionProfile::kTunnel &&
      (closure.front_boundary_seen ||
       exploration_phase_ == ExplorationPhase::kFrontierFallback);
  if (!have_active_goal_ && tunnel_bend_detected) {
    executable_route_published_this_cycle =
        publishConnectedTurnContinuation(reachable);
  }
  if (executable_route_published_this_cycle) {
    publishStatus("EXPLORING",
                  "following a verified connected route into the tunnel bend");
    return;
  }

  if (!have_active_goal_) {
    executable_route_published_this_cycle =
        publishLateralDetourGoal(reachable, candidates);
  }
  if (executable_route_published_this_cycle) {
    publishStatus("EXPLORING", "known-free lateral transition toward side opening");
    return;
  }

  if (!have_active_goal_) {
    executable_route_published_this_cycle =
        publishForwardLookaheadGoal(reachable);
  }
  if (executable_route_published_this_cycle) {
    publishStatus("EXPLORING",
                  "continuous forward look-ahead goal sent to SUPER");
    return;
  }

  if (!have_active_goal_) {
    executable_route_published_this_cycle =
        selectAndPublishFrontier(candidates, reachable);
  }
  if (executable_route_published_this_cycle) {
    publishStatus("EXPLORING", "new frontier goal sent to SUPER");
    return;
  }

  if (!have_active_goal_) {
    executable_route_published_this_cycle =
        publishConnectedTurnContinuation(reachable);
  }
  if (executable_route_published_this_cycle) {
    publishStatus("EXPLORING",
                  "following the hard-safe connected route around a bend");
    return;
  }

  if (hard_safe_route_unavailable_since_.isZero()) {
    hard_safe_route_unavailable_since_ = now;
  }
  const double hard_safe_route_wait =
      (now - hard_safe_route_unavailable_since_).toSec();

  if (!have_active_goal_ && use_map_closure_completion_ &&
      closure.front_boundary_seen && !closure.reachable_forward_passage &&
      closure.actionable_frontiers == 0 &&
      closure.vehicle_progress >= front_obstacle_min_progress_) {
    if (return_on_confirmed_dead_end_) {
      if (hard_safe_route_wait >= no_frontier_timeout_) {
        publishStatus("FORWARD_ROUTE_STALLED",
                      "front boundary closed and no hard-safe route remained; returning");
        beginReturnHome(
            "front boundary closed and no hard-safe route remained", true);
      } else {
        std::ostringstream detail;
        detail << std::fixed << std::setprecision(1)
               << "checking the live map for a hard-safe bend or passage; "
               << std::max(0.0, no_frontier_timeout_ - hard_safe_route_wait)
               << " s before verified return";
        publishStatus("WAIT_HARD_SAFE_ROUTE", detail.str());
      }
      return;
    }
    if (reverseMissionHeadingAtDeadEnd(closure)) {
      publishStatus(
          "DEAD_END_BACKTRACK",
          "heading reversed; planning a fresh route on the current map");
      return;
    }
    publishStatus("WAIT_MAP_CLOSURE",
                  "front boundary closed; waiting for frontier and map convergence");
    return;
  }

  // Reaching here means every hard-safe publisher was evaluated this cycle.
  // Raw frontier presence is not evidence of an executable route: a frontier
  // can be behind the vehicle, outside the mission band, or fail the local
  // segment check. Stall on the publication outcome instead.
  if (!have_active_goal_ && !executable_route_published_this_cycle &&
      exploration_started_ &&
      !closure_confirmation_pending &&
      !(use_map_closure_completion_ && map_closure_complete_streak_ > 0) &&
      hard_safe_route_wait >= no_frontier_timeout_) {
    publishStatus("FORWARD_ROUTE_STALLED",
                  "no hard-safe exploration route; returning on the live map");
    beginReturnHome("no hard-safe unexplored route remained available", true);
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

void SuperExplorationDecider::obstacleReplanCallback(
    const std_msgs::Empty::ConstPtr&) {
  if (!enabled_ || !have_home_ || mission_finished_) return;

  if (have_active_goal_) {
    if (returning_home_ || dead_end_backtracking_) {
      rejected_return_point_ = current_goal_.pose.position;
      rejected_return_point_stamp_ = ros::Time::now();
    } else {
      // Do not immediately select the same endpoint that led into the
      // persistent soft-obstacle hold. The next decision cycle first tries a
      // verified lateral detour, then a fresh rolling-horizon segment.
      rememberFailedGoal(current_goal_.pose.position);
    }
    cancelActiveGoal("persistent_obstacle_replan");
  }
  rejected_goal_retry_after_ = ros::Time();
  publishStatus("OBSTACLE_REPLAN",
                "persistent soft obstacle; selecting a fresh verified route");
}

void SuperExplorationDecider::rejectedGoalCallback(
    const std_msgs::UInt64::ConstPtr& message) {
  if (!have_active_goal_ || message->data != active_goal_id_) return;
  if (returning_home_ || dead_end_backtracking_) {
    rejected_return_point_ = current_goal_.pose.position;
    rejected_return_point_stamp_ = ros::Time::now();
  }
  if (!returning_home_ && !dead_end_backtracking_) {
    rememberFailedGoal(current_goal_.pose.position);
  }
  ROS_WARN("SUPER rejected goal %llu; waiting for a fresh map before retry",
           static_cast<unsigned long long>(message->data));
  have_active_goal_ = false;
  active_goal_id_ = 0;
  accepted_goal_id_ = 0;
  active_goal_allows_rolling_handover_ = false;
  active_goal_commits_tunnel_heading_ = false;
  active_goal_tunnel_heading_yaw_ = 0.0;
  active_goal_updates_exploration_heading_ = false;
  active_goal_exploration_heading_yaw_ = 0.0;
  clearExplorationGoalQueue();
  rejected_goal_retry_after_ = ros::Time::now() + ros::Duration(1.0);
  publishStatus("GOAL_REJECTED", "SUPER rejected unavailable target; not counted as explored");
}

void SuperExplorationDecider::acceptedGoalCallback(
    const std_msgs::UInt64::ConstPtr& message) {
  if (have_active_goal_ && message->data == active_goal_id_) {
    accepted_goal_id_ = message->data;
    if (!returning_home_ && !dead_end_backtracking_) {
      // Only SUPER acceptance proves that a task-level candidate became an
      // executable route. Merely publishing goals that SUPER then rejects
      // must not keep resetting the bounded no-route return timer.
      hard_safe_route_unavailable_since_ = ros::Time();
      if (active_goal_updates_exploration_heading_) {
        have_last_exploration_route_heading_ = true;
        last_exploration_route_heading_yaw_ =
            active_goal_exploration_heading_yaw_;
      }
      if (active_goal_commits_tunnel_heading_) {
        const bool first_lock = !tunnel_route_locked_;
        tunnel_route_locked_ = true;
        tunnel_route_heading_yaw_ = active_goal_tunnel_heading_yaw_;
        if (first_lock) {
          ROS_WARN("Task-4 tunnel branch committed after SUPER acceptance; "
                   "local route heading locked at %.1f deg",
                   tunnel_route_heading_yaw_ * 180.0 / M_PI);
        }
      }
    }
    if (enabled_ && initial_accepted_goal_id_ == 0) {
      initial_accepted_goal_id_ = message->data;
      publishInitialGoalAccepted(initial_accepted_goal_id_);
      ROS_INFO("Task-one initial SUPER goal %llu accepted; scheduler may "
               "enable automatic authority",
               static_cast<unsigned long long>(initial_accepted_goal_id_));
    }
  }
}

void SuperExplorationDecider::missionEnableCallback(
    const std_msgs::Bool::ConstPtr& message) {
  if (!follow_active_task_profile_) {
    setMissionEnabled(message->data, "mission scheduler");
    return;
  }
  scheduler_mission_enabled_ = message->data;
  updateScheduledMissionSelection();
}

void SuperExplorationDecider::activeTaskCallback(
    const std_msgs::UInt8::ConstPtr& message) {
  scheduler_active_task_ = message->data;
  if (follow_active_task_profile_) {
    updateScheduledMissionSelection();
  }
}

void SuperExplorationDecider::updateScheduledMissionSelection() {
  const bool valid_exploration_task =
      scheduler_active_task_ == 1 || scheduler_active_task_ == 4;
  const bool should_enable =
      scheduler_mission_enabled_ && valid_exploration_task;
  if (valid_exploration_task) {
    const ExplorationMissionProfile selected_profile =
        scheduler_active_task_ == 4
            ? ExplorationMissionProfile::kTunnel
            : ExplorationMissionProfile::kGoaf;
    if (mission_profile_ != selected_profile) {
      if (enabled_) {
        setMissionEnabled(false, "mission profile transition");
      }
      mission_profile_ = selected_profile;
      mission_profile_name_ = explorationMissionProfileName(mission_profile_);
      ROS_INFO("Exploration profile selected from active task %u: %s",
               static_cast<unsigned int>(scheduler_active_task_),
               mission_profile_name_.c_str());
    }
  }
  setMissionEnabled(should_enable, "mission scheduler");
}

void SuperExplorationDecider::setMissionEnabled(
    bool enabled, const std::string& source) {
  if (enabled_ == enabled) {
    return;
  }
  enabled_ = enabled;
  if (!enabled_) {
    initial_accepted_goal_id_ = 0;
    publishInitialGoalAccepted(0);
    cancelActiveGoal("mission_disabled", true);
    publishStatus("DISABLED", "task scheduler selected another task");
    ROS_INFO("SUPER exploration paused by %s", source.c_str());
  } else {
    // A new enable transition starts a fresh test mission.  This clears any
    // map/home left from the previous run and arms home capture for the first
    // synchronized Omni-LIVO cloud+odometry sample after task authorization.
    cancelActiveGoal("mission_reenabled", true);
    clearMissionState();
    mission_start_pending_ = true;
    publishStatus("WAIT_DATA", "task scheduler selected " +
                      mission_profile_name_ + " exploration");
    ROS_INFO("SUPER %s exploration enabled by %s; waiting for the first "
             "synchronized sample to capture home",
             mission_profile_name_.c_str(), source.c_str());
  }
}

void SuperExplorationDecider::batteryCallback(
    const sensor_msgs::BatteryState::ConstPtr& message) {
  if (std::isfinite(message->percentage) && message->percentage >= 0.0) {
    have_battery_ = true;
    battery_percentage_ = message->percentage;
  }
  if (std::isfinite(message->voltage) && message->voltage > 0.0) {
    battery_voltage_ = message->voltage;
    if (battery_voltage_ < battery_return_voltage_ &&
        exploration_started_) {
      low_voltage_return_requested_ = true;
      return_requested_ = true;
      if (enabled_ && have_home_ && !returning_home_ && !mission_finished_) {
        beginReturnHome("battery voltage below return threshold", true);
      }
    }
  }
}

void SuperExplorationDecider::publishStatus(const std::string& state,
                                            const std::string& detail) {
  std_msgs::String message;
  message.data = detail.empty() ? state : state + ": " + detail;
  status_publisher_.publish(message);
}

void SuperExplorationDecider::publishInitialGoalAccepted(uint64_t goal_id) {
  std_msgs::UInt64 message;
  message.data = goal_id;
  initial_goal_accepted_publisher_.publish(message);
}

void SuperExplorationDecider::clearMissionState() {
  voxels_.clear();
  occupancy_evidence_.clear();
  last_scan_hits_.clear();
  last_scan_stamp_=ros::Time();
  super_occupied_stamp_ = ros::Time();
  super_occupied_points_->clear();
  rejected_return_point_stamp_ = ros::Time();
  have_last_centerline_position_ = false;
  covered_goals_.clear();
  failed_goals_.clear();
  outbound_breadcrumbs_.clear();
  clearExplorationGoalQueue();
  return_waypoints_.clear();
  return_waypoint_index_ = 0;
  home_arrival_streak_ = 0;
  current_goal_ = geometry_msgs::PoseStamped();
  active_goal_initial_distance_ = 0.0;
  accepted_goal_id_ = 0;
  initial_accepted_goal_id_ = 0;
  publishInitialGoalAccepted(0);
  rejected_goal_retry_after_ = ros::Time();
  have_data_ = false;
  have_home_ = false;
  have_active_goal_ = false;
  active_goal_is_end_approach_ = false;
  active_goal_is_lateral_detour_ = false;
  active_goal_is_forward_lookahead_ = false;
  active_goal_is_blocked_frontier_route_ = false;
  active_goal_allows_rolling_handover_ = false;
  active_goal_commits_tunnel_heading_ = false;
  active_goal_tunnel_heading_yaw_ = 0.0;
  active_goal_updates_exploration_heading_ = false;
  active_goal_exploration_heading_yaw_ = 0.0;
  tunnel_route_locked_ = false;
  tunnel_route_heading_yaw_ = 0.0;
  pending_connected_turn_side_ = 0;
  pending_connected_turn_streak_ = 0;
  pending_connected_turn_time_ = ros::Time();
  have_last_exploration_route_heading_ = false;
  last_exploration_route_heading_yaw_ = 0.0;
  exploration_started_ = false;
  returning_home_ = false;
  dead_end_backtracking_ = false;
  mission_finished_ = false;
  return_requested_ = false;
  have_battery_ = false;
  battery_percentage_ = -1.0;
  battery_voltage_ = -1.0;
  low_percentage_return_requested_ = false;
  low_voltage_return_requested_ = false;
  low_battery_since_ = ros::Time();
  prefer_outbound_breadcrumbs_ = false;
  reached_goal_count_ = 0;
  three_wall_complete_streak_ = 0;
  map_closure_complete_streak_ = 0;
  dead_end_heading_reversal_count_ = 0;
  map_growth_reference_count_ = 0;
  mission_heading_yaw_ = 0.0;
  exploration_phase_ = ExplorationPhase::kForwardPriority;
  left_wall_visible_ = false;
  right_wall_visible_ = false;
  front_obstacle_visible_ = false;
  side_wall_missing_streak_ = 0;
  front_obstacle_streak_ = 0;
  held_route_side_ = 0;
  resetCorridorCentering();
  resetCorridorPreviewPreference();
  first_data_time_ = ros::Time();
  last_frontier_time_ = ros::Time();
  last_motion_progress_time_ = ros::Time();
  hard_safe_route_unavailable_since_ = ros::Time();
  last_motion_progress_position_ = geometry_msgs::Point();
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
