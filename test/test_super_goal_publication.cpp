#include "mine_uav_control/super_exploration_decider.hpp"

#include <gtest/gtest.h>
#include <pcl_conversions/pcl_conversions.h>
#include <boost/make_shared.hpp>

#include <chrono>
#include <algorithm>
#include <mutex>
#include <vector>

namespace mine_uav_control {

class SuperExplorationDeciderTestPeer {
 public:
  static void publish(SuperExplorationDecider& decider,
                      const geometry_msgs::PoseStamped& goal,
                      const std::string& reason,
                      bool continuous_handover = false) {
    decider.publishGoal(goal, reason, continuous_handover);
  }

  static bool forwardLookaheadVariantsKeepHandoverSemantics(
      SuperExplorationDecider& decider) {
    geometry_msgs::PoseStamped goal;
    goal.pose.orientation.w = 1.0;
    for (const std::string& reason : {
             "forward_lookahead_preferred",
             "forward_lookahead_hard_fallback",
             "forward_lookahead_centered_preferred",
             "forward_lookahead_centered_hard_fallback"}) {
      decider.publishGoal(goal, reason, true);
      if (!decider.active_goal_is_forward_lookahead_) return false;
    }
    decider.publishGoal(goal, "lateral_detour", true);
    return !decider.active_goal_is_forward_lookahead_;
  }

  static bool corridorCenteringRequiresStableTwoSidedEvidence(
      SuperExplorationDecider& decider) {
    decider.have_home_ = true;
    decider.corridor_centering_enabled_ = true;
    decider.corridor_centering_min_wall_points_ = 4;
    decider.corridor_centering_confirm_frames_ = 3;
    decider.corridor_centering_release_frames_ = 3;
    decider.corridor_centering_max_width_ = 6.0;
    decider.corridor_centering_filter_gain_ = 0.25;
    decider.corridor_centering_max_center_jump_ = 0.75;
    decider.vehicle_radius_ = 1.0;
    decider.exploration_phase_ =
        SuperExplorationDecider::ExplorationPhase::kForwardPriority;
    decider.left_wall_visible_ = true;
    decider.right_wall_visible_ = true;
    decider.front_obstacle_visible_ = false;
    const std::vector<double> left(4, 2.5);
    const std::vector<double> right(4, -1.5);
    for (int frame = 0; frame < 2; ++frame) {
      decider.updateCorridorCentering(left, right, 0.0, false);
      if (decider.corridorCenteringActive()) return false;
    }
    decider.updateCorridorCentering(left, right, 0.0, false);
    if (!decider.corridorCenteringActive()) return false;

    // One missing side or a front wall disables steering immediately, even
    // while the filtered estimate is retained briefly for noise tolerance.
    decider.left_wall_visible_ = false;
    if (decider.corridorCenteringActive()) return false;
    decider.left_wall_visible_ = true;
    decider.front_obstacle_visible_ = true;
    if (decider.corridorCenteringActive()) return false;

    decider.front_obstacle_visible_ = false;
    const std::vector<double> no_wall;
    for (int frame = 0; frame < 3; ++frame) {
      decider.updateCorridorCentering(no_wall, no_wall, 0.0, false);
    }
    return !decider.corridor_center_valid_;
  }

  static bool emptyForwardHandoverPreservesGoal(
      SuperExplorationDecider& decider) {
    decider.have_home_ = true;
    decider.home_pose_.pose.position.z = 1.5;
    decider.current_pose_.pose.position.x = 2.5;
    decider.current_pose_.pose.position.z = 1.5;
    const auto previous_id = decider.active_goal_id_;
    const auto previous_goal = decider.current_goal_.pose.position.x;
    SuperExplorationDecider::VoxelSet empty_reachable;
    const bool handed_over = decider.tryForwardHandover(empty_reachable);
    return !handed_over && decider.have_active_goal_ &&
           decider.active_goal_id_ == previous_id &&
           decider.current_goal_.pose.position.x == previous_goal;
  }

  static bool acceptedShortForwardGoalHandsOver(
      SuperExplorationDecider& decider) {
    decider.enabled_ = true;
    decider.have_data_ = true;
    decider.have_home_ = true;
    decider.exploration_started_ = true;
    decider.last_sync_time_ = ros::Time::now();
    decider.super_occupied_stamp_ = decider.last_sync_time_;
    decider.goal_sent_time_ = decider.last_sync_time_;
    decider.data_timeout_ = 100.0;
    decider.current_pose_.pose.position.x = 2.2;
    decider.current_goal_.pose.position.x = 3.0;
    decider.current_goal_.pose.orientation.w = 1.0;
    decider.active_goal_id_ = 7;
    decider.accepted_goal_id_ = 7;
    decider.have_active_goal_ = true;
    decider.active_goal_is_forward_lookahead_ = true;
    decider.front_obstacle_streak_ = decider.front_obstacle_confirm_frames_;
    decider.active_goal_initial_distance_ = 3.0;
    decider.forward_lookahead_distance_ = 3.0;
    decider.min_goal_distance_ = 2.0;
    decider.forward_goal_lateral_search_width_ = 0.0;
    decider.max_exploration_radius_from_home_ = 10.0;
    for (double x = 0.0; x <= 6.0; x += 0.125)
      decider.voxels_[decider.positionToKey(x, 0.0, 0.0)] =
          SuperExplorationDecider::FREE;
    // The ordinary frontier predicate rejects this 0.8 m remaining leg.
    // Handover must still be possible for its already accepted lookahead goal.
    if (decider.isForwardCandidate({{}, decider.current_goal_, 0.0, 0}))
      return false;
    decider.decisionTimerCallback(ros::TimerEvent());
    return decider.have_active_goal_ && decider.active_goal_id_ != 7 &&
           decider.current_goal_.pose.position.x > 3.0;
  }

  static bool returnHandsOverOnlyAcceptedProgressiveLeg(
      SuperExplorationDecider& decider) {
    decider.enabled_ = true;
    decider.have_data_ = true;
    decider.have_home_ = true;
    decider.returning_home_ = true;
    decider.last_sync_time_ = ros::Time::now();
    decider.super_occupied_stamp_ = decider.last_sync_time_;
    decider.goal_sent_time_ = decider.last_sync_time_;
    decider.data_timeout_ = 100.0;
    decider.return_segment_length_ = 3.0;
    decider.current_pose_.pose.position.x = 4.8;
    decider.current_goal_.pose.position.x = 4.0;
    decider.active_goal_id_ = 7;
    decider.have_active_goal_ = true;
    decider.active_goal_initial_distance_ = 2.0;
    for (double x = 0.0; x <= 6.0; x += 0.25)
      decider.voxels_[decider.positionToKey(x, 0.0, 0.0)] =
          SuperExplorationDecider::FREE;
    decider.decisionTimerCallback(ros::TimerEvent());
    if (decider.active_goal_id_ != 7) return false;
    decider.accepted_goal_id_ = 7;
    decider.last_sync_time_ = ros::Time::now();
    decider.super_occupied_stamp_ = decider.last_sync_time_;
    decider.decisionTimerCallback(ros::TimerEvent());
    return decider.have_active_goal_ && decider.active_goal_id_ != 7 &&
           decider.current_goal_.pose.position.x < 4.0;
  }

  static bool returnPrefersSurveyedLane(
      SuperExplorationDecider& decider) {
    decider.have_home_ = true;
    decider.have_data_ = true;
    decider.last_sync_time_ = ros::Time::now();
    decider.super_occupied_stamp_ = decider.last_sync_time_;
    decider.home_pose_.pose.orientation.w = 1.0;
    decider.current_pose_.pose.position.x = 4.0;
    decider.outbound_breadcrumbs_.clear();
    for (double x : {0.0, 2.0, 4.0}) {
      geometry_msgs::PoseStamped point;
      point.pose.position.x = x;
      point.pose.orientation.w = 1.0;
      decider.outbound_breadcrumbs_.push_back(point);
    }
    for (double x = 0.0; x <= 5.0; x += decider.voxel_resolution_) {
      for (double y = -2.0; y <= 2.0; y += decider.voxel_resolution_) {
        decider.voxels_[decider.positionToKey(x, y, 0.0)] =
            SuperExplorationDecider::FREE;
      }
    }
    decider.beginReturnHome("test_return");
    return decider.returning_home_ &&
           decider.return_waypoints_.size() == 1 &&
           decider.current_goal_.pose.position.x < 4.0 &&
           std::abs(decider.current_goal_.pose.position.y) < 0.25;
  }

  static bool lowVoltageImmediatelyStartsVerifiedBreadcrumbReturn(
      SuperExplorationDecider& decider) {
    decider.enabled_ = true;
    decider.have_home_ = true;
    decider.have_data_ = true;
    decider.last_sync_time_ = ros::Time::now();
    decider.super_occupied_stamp_ = decider.last_sync_time_;
    decider.current_pose_.pose.position.x = 4.0;
    decider.home_pose_.pose.orientation.w = 1.0;
    for (double x : {0.0, 2.0, 4.0}) {
      geometry_msgs::PoseStamped breadcrumb;
      breadcrumb.pose.position.x = x;
      breadcrumb.pose.orientation.w = 1.0;
      decider.outbound_breadcrumbs_.push_back(breadcrumb);
    }
    for (double x = 0.0; x <= 4.0; x += 0.25)
      for (double y = -1.0; y <= 1.0; y += 0.25)
        decider.voxels_[decider.positionToKey(x, y, 0.0)] =
            SuperExplorationDecider::FREE;
    sensor_msgs::BatteryState battery;
    battery.percentage = 0.8;
    battery.voltage = 21.6;
    decider.batteryCallback(boost::make_shared<sensor_msgs::BatteryState>(battery));
    if (decider.returning_home_) return false;
    battery.voltage = 21.59;
    decider.batteryCallback(boost::make_shared<sensor_msgs::BatteryState>(battery));
    return decider.low_voltage_return_requested_ && decider.returning_home_ &&
           decider.prefer_outbound_breadcrumbs_ &&
           decider.have_active_goal_ &&
           decider.current_goal_.pose.position.x < 4.0;
  }

  static bool lowPercentageRequiresSustainedConfirmation(
      SuperExplorationDecider& decider) {
    decider.enabled_ = true;
    decider.have_home_ = true;
    decider.have_data_ = true;
    decider.exploration_started_ = false;
    decider.data_timeout_ = 100.0;
    decider.battery_return_confirm_duration_ = 3.0;
    decider.last_sync_time_ = ros::Time::now();
    decider.super_occupied_stamp_ = decider.last_sync_time_;
    decider.current_pose_.pose.position.x = 4.0;
    for (double x : {0.0, 2.0, 4.0}) {
      geometry_msgs::PoseStamped breadcrumb;
      breadcrumb.pose.position.x = x;
      breadcrumb.pose.orientation.w = 1.0;
      decider.outbound_breadcrumbs_.push_back(breadcrumb);
    }
    for (double x = 0.0; x <= 4.0; x += 0.25)
      decider.voxels_[decider.positionToKey(x, 0.0, 0.0)] =
          SuperExplorationDecider::FREE;
    sensor_msgs::BatteryState battery;
    battery.percentage = 0.20;
    battery.voltage = 22.0;
    decider.batteryCallback(
        boost::make_shared<sensor_msgs::BatteryState>(battery));
    decider.decisionTimerCallback(ros::TimerEvent());
    if (decider.return_requested_ || decider.returning_home_ ||
        decider.low_battery_since_.isZero()) {
      return false;
    }
    decider.low_battery_since_ = ros::Time::now() - ros::Duration(3.1);
    decider.decisionTimerCallback(ros::TimerEvent());
    return decider.low_percentage_return_requested_ &&
           decider.return_requested_ && decider.returning_home_ &&
           decider.current_goal_.pose.position.x < 4.0;
  }

  static bool returnTransitionDuringBacktrackNeverMovesAwayFromHome(
      SuperExplorationDecider& decider) {
    decider.have_home_ = true;
    decider.have_data_ = true;
    decider.last_sync_time_ = ros::Time::now();
    decider.super_occupied_stamp_ = decider.last_sync_time_;
    decider.return_segment_length_ = 3.0;
    decider.current_pose_.pose.position.x = 3.5;
    decider.current_goal_.pose.position.x = 5.1;
    decider.have_active_goal_ = true;
    decider.dead_end_backtracking_ = true;
    for (double x : {0.0, 2.0, 4.0, 6.0, 8.0}) {
      geometry_msgs::PoseStamped breadcrumb;
      breadcrumb.pose.position.x = x;
      breadcrumb.pose.orientation.w = 1.0;
      decider.outbound_breadcrumbs_.push_back(breadcrumb);
    }
    for (double x = 0.0; x <= 8.0; x += 0.25)
      decider.voxels_[decider.positionToKey(x, 0.0, 0.0)] =
          SuperExplorationDecider::FREE;
    decider.beginReturnHome("test_backtrack_transition", true);
    return decider.returning_home_ && !decider.dead_end_backtracking_ &&
           decider.have_active_goal_ &&
           decider.current_goal_.pose.position.x < 3.5 &&
           std::abs(decider.outbound_breadcrumbs_.back().pose.position.x -
                    8.0) < 1e-9;
  }

  static bool faultReturnFollowsBentOutboundLane(
      SuperExplorationDecider& decider) {
    decider.have_home_ = true;
    decider.have_data_ = true;
    decider.last_sync_time_ = ros::Time::now();
    decider.super_occupied_stamp_ = decider.last_sync_time_;
    decider.return_segment_length_ = 2.5;
    decider.current_pose_.pose.position.x = 4.0;
    decider.current_pose_.pose.position.y = 2.0;
    for (const auto& xy : {std::pair<double, double>{0.0, 0.0},
                           {2.0, 0.0}, {2.0, 2.0}, {4.0, 2.0}}) {
      geometry_msgs::PoseStamped breadcrumb;
      breadcrumb.pose.position.x = xy.first;
      breadcrumb.pose.position.y = xy.second;
      decider.outbound_breadcrumbs_.push_back(breadcrumb);
    }
    for (double x = 0.0; x <= 4.0; x += 0.25)
      for (double y = 0.0; y <= 2.0; y += 0.25)
        decider.voxels_[decider.positionToKey(x, y, 0.0)] =
            SuperExplorationDecider::FREE;
    decider.beginReturnHome("test_fault", true);
    return decider.returning_home_ && decider.have_active_goal_ &&
           decider.current_goal_.pose.position.x <= 2.1 &&
           decider.current_goal_.pose.position.y >= 1.0;
  }

  static bool returnOffsetStaysInsideTheClearLane(
      SuperExplorationDecider& decider) {
    decider.have_home_ = true;
    decider.have_data_ = true;
    decider.last_sync_time_ = ros::Time::now();
    decider.super_occupied_stamp_ = decider.last_sync_time_;
    decider.current_pose_.pose.position.x = 4.0;
    decider.home_pose_.pose.orientation.w = 1.0;
    for (double x = 0.0; x <= 5.0; x += 0.25) {
      geometry_msgs::PoseStamped breadcrumb;
      breadcrumb.pose.position.x = x;
      decider.outbound_breadcrumbs_.push_back(breadcrumb);
      for (double y = -2.0; y <= 2.0; y += 0.25) {
        decider.voxels_[decider.positionToKey(x, y, 0.0)] =
            SuperExplorationDecider::FREE;
      }
      decider.super_occupied_points_->push_back(
          pcl::PointXYZ(x, -2.4, 0.0));
      decider.super_occupied_points_->push_back(
          pcl::PointXYZ(x, 1.4, 0.0));
    }
    decider.super_occupied_tree_.setInputCloud(decider.super_occupied_points_);
    decider.beginReturnHome("test_clear_return_lane");
    const auto& target = decider.current_goal_.pose.position;
    return decider.returning_home_ && decider.have_active_goal_ &&
           target.x < 4.0 && target.y <= 0.2 && target.y >= -1.1 &&
           decider.returnSegmentClear(decider.current_pose_.pose.position,
                                      target);
  }

  static bool returnGoalRespectsHeightCeiling(
      SuperExplorationDecider& decider) {
    decider.have_home_ = true;
    decider.have_data_ = true;
    decider.last_sync_time_ = ros::Time::now();
    decider.super_occupied_stamp_ = decider.last_sync_time_;
    decider.home_pose_.pose.position.z = 1.5;
    decider.current_pose_.pose.position.x = 4.0;
    decider.current_pose_.pose.position.z = 1.8;
    decider.return_max_height_above_home_ = 0.2;
    for (double x = 0.0; x <= 5.0; x += 0.25) {
      for (double y = -1.0; y <= 1.0; y += 0.25) {
        for (double z = 1.5; z <= 2.0; z += 0.25) {
          decider.voxels_[decider.positionToKey(x, y, z)] =
              SuperExplorationDecider::FREE;
        }
      }
    }
    return decider.publishFreshReturnGoal("test_height_ceiling") &&
           decider.current_goal_.pose.position.z <= 1.7 + 1e-6;
  }

  static bool returnChecksSuperObstacleAndFreshness(
      SuperExplorationDecider& decider) {
    decider.have_home_ = true;
    decider.super_occupied_stamp_ = ros::Time::now();
    for (double x = 0.0; x <= 4.0; x += decider.voxel_resolution_) {
      for (double y = -2.0; y <= 2.0; y += decider.voxel_resolution_) {
        decider.voxels_[decider.positionToKey(x, y, 0.0)] =
            SuperExplorationDecider::FREE;
      }
    }
    decider.super_occupied_points_->push_back(pcl::PointXYZ(2.0, 0.0, 0.0));
    decider.super_occupied_tree_.setInputCloud(decider.super_occupied_points_);
    geometry_msgs::Point start, home, offset;
    start.x = 4.0;
    home.x = 0.0;
    offset.x = 2.0;
    offset.y = 1.5;
    const bool obstacle_respected =
        !decider.returnSegmentClear(start, home) &&
        decider.returnSegmentClear(start, offset);
    decider.super_occupied_stamp_ = ros::Time::now() - ros::Duration(3.0);
    return obstacle_respected && !decider.returnSegmentClear(start, offset);
  }

  static bool returnCanStartBelowAnUnsafeVoxelCentre(
      SuperExplorationDecider& decider) {
    decider.voxel_resolution_ = 0.25;
    decider.have_home_ = true;
    decider.have_data_ = true;
    decider.last_sync_time_ = ros::Time::now();
    decider.super_occupied_stamp_ = decider.last_sync_time_;
    decider.current_pose_.pose.position.x = 4.125;
    decider.current_pose_.pose.position.y = 0.125;
    decider.current_pose_.pose.position.z = 1.30;
    decider.home_pose_.pose.position.x = 0.125;
    decider.home_pose_.pose.position.y = 0.125;
    decider.home_pose_.pose.position.z = 1.30;
    decider.return_clearance_margin_ = 0.0;
    for (int x = 0; x <= 17; ++x) {
      for (int y = -4; y <= 4; ++y) {
        for (int z : {4, 5}) {
          decider.voxels_[{x, y, z}] = SuperExplorationDecider::FREE;
        }
      }
    }
    // This case deliberately removes the live-map margin. Keep the continuous
    // pose 0.85 m below the point (safe for the 0.8 m cylinder), while its
    // 1.375 m voxel centre is only 0.775 m below it (unsafe).
    decider.super_occupied_points_->push_back(
        pcl::PointXYZ(4.125, 0.125, 2.15));
    decider.super_occupied_tree_.setInputCloud(decider.super_occupied_points_);
    const auto containing = decider.positionToKey(4.125, 0.125, 1.30);
    if (decider.returnSegmentClear(decider.current_pose_.pose.position,
                                   decider.keyToPoint(containing))) {
      return false;
    }
    const bool published = decider.publishFreshReturnGoal("test_safe_seed");
    return published && decider.current_goal_.pose.position.x < 4.125;
  }

  static bool nearHomeCanUseSubmetreClearStep(
      SuperExplorationDecider& decider) {
    decider.voxel_resolution_ = 0.25;
    decider.goal_reached_distance_ = 1.0;
    decider.return_home_arrival_radius_ = 0.75;
    decider.have_home_ = true;
    decider.have_data_ = true;
    decider.last_sync_time_ = ros::Time::now();
    decider.super_occupied_stamp_ = decider.last_sync_time_;
    decider.returning_home_ = true;
    decider.current_pose_.pose.position.x = -0.4;
    decider.current_pose_.pose.position.y = -0.85;
    for (double x = -2.0; x <= 1.0; x += 0.25) {
      for (double y = -2.0; y <= 1.0; y += 0.25) {
        decider.voxels_[decider.positionToKey(x, y, 0.0)] =
            SuperExplorationDecider::FREE;
      }
    }
    // The exact home coordinate is map-blocked, but a short step toward the
    // launch area remains fully clear under SUPER's 1 m radius.
    decider.super_occupied_points_->push_back(
        pcl::PointXYZ(0.35, 0.25, 0.0));
    decider.super_occupied_tree_.setInputCloud(decider.super_occupied_points_);
    const auto start = decider.current_pose_.pose.position;
    if (decider.returnSegmentClear(start, decider.home_pose_.pose.position))
      return false;
    if (!decider.publishFreshReturnGoal("test_near_home")) return false;
    const auto goal = decider.current_goal_.pose.position;
    const auto distance_squared = [](const geometry_msgs::Point& a,
                                     const geometry_msgs::Point& b) {
      const double dx = a.x - b.x;
      const double dy = a.y - b.y;
      const double dz = a.z - b.z;
      return dx * dx + dy * dy + dz * dz;
    };
    const double step = std::sqrt(distance_squared(start, goal));
    return step >= 0.25 && step < 1.0 &&
           distance_squared(goal, decider.home_pose_.pose.position) <
               distance_squared(start, decider.home_pose_.pose.position) &&
           decider.returnSegmentClear(start, goal);
  }

  static bool launchAreaArrivalNeedsTwoObservations(
      SuperExplorationDecider& decider) {
    decider.have_home_ = true;
    decider.have_data_ = true;
    decider.enabled_ = true;
    decider.goal_reached_distance_ = 1.0;
    decider.last_sync_time_ = ros::Time::now();
    decider.return_home_arrival_radius_ = 0.75;
    decider.returning_home_ = true;
    decider.current_pose_.pose.position.x = -0.4;
    decider.current_pose_.pose.position.y = -0.6;
    decider.decisionTimerCallback(ros::TimerEvent());
    if (decider.mission_finished_ || !decider.returning_home_) return false;
    decider.decisionTimerCallback(ros::TimerEvent());
    return decider.mission_finished_ && !decider.returning_home_;
  }


  static bool freeRayStopsAtOccupiedCell(SuperExplorationDecider& decider) {
    decider.enabled_ = true;
    decider.have_data_ = true;
    decider.have_home_ = true;
    decider.current_pose_.header.stamp = ros::Time::now();
    decider.current_pose_.pose.position.x = 0.0;
    decider.current_pose_.pose.position.y = 0.0;
    decider.current_pose_.pose.position.z = 0.0;
    const auto obstacle = decider.positionToKey(2.0, 0.0, 0.0);
    const int hits = std::max(
        1, (decider.occupied_confirm_threshold_ +
            std::max(1, decider.occupied_hit_increment_) - 1) /
               std::max(1, decider.occupied_hit_increment_));
    for (int i = 0; i < hits; ++i) {
      decider.markOccupied(obstacle);
    }
    pcl::PointCloud<pcl::PointXYZ> endpoints;
    endpoints.points.emplace_back(5.0f, 0.0f, 0.0f);
    sensor_msgs::PointCloud2 cloud;
    pcl::toROSMsg(endpoints, cloud);
    cloud.header.frame_id = decider.world_frame_;
    cloud.header.stamp = decider.current_pose_.header.stamp;
    decider.freeRayCallback(boost::make_shared<sensor_msgs::PointCloud2>(cloud));
    return decider.isKnownFree(decider.positionToKey(1.0, 0.0, 0.0)) &&
           decider.isOccupied(obstacle) &&
           !decider.isKnownFree(decider.positionToKey(3.0, 0.0, 0.0));
  }

  static bool reversedBranchUsesOnlyNearbyLiveBoundary(
      SuperExplorationDecider& decider) {
    decider.have_home_ = true;
    decider.mission_heading_yaw_ = 0.0;
    decider.front_obstacle_min_progress_ = 8.0;
    decider.min_goal_distance_ = 2.0;
    decider.map_closure_min_progress_ = 80.0;
    decider.map_closure_no_frontier_time_ = 0.0;
    decider.map_closure_stable_time_ = 100.0;
    decider.dead_end_heading_reversal_count_ = 1;
    decider.current_pose_.pose.position.x = 3.0;
    decider.front_obstacle_streak_ = decider.front_obstacle_confirm_frames_;
    std::vector<SuperExplorationDecider::FrontierCandidate> candidates;
    SuperExplorationDecider::VoxelSet reachable;
    const auto live = decider.evaluateMapClosure(
        candidates, reachable, false);

    decider.front_obstacle_streak_ = 0;
    const auto historic_near_origin = decider.evaluateMapClosure(
        candidates, reachable, true);

    decider.current_pose_.pose.position.x = 9.0;
    const auto historic_after_full_gate = decider.evaluateMapClosure(
        candidates, reachable, true);
    return live.front_boundary_seen && live.complete &&
           !historic_near_origin.front_boundary_seen &&
           historic_after_full_gate.front_boundary_seen;
  }

  static bool reversedStableReachableComponentCanCloseWithoutWall(
      SuperExplorationDecider& decider) {
    decider.have_home_ = true;
    decider.mission_heading_yaw_ = 0.0;
    decider.min_goal_distance_ = 2.0;
    decider.map_closure_min_progress_ = 80.0;
    decider.map_closure_no_frontier_time_ = 0.0;
    decider.map_closure_stable_time_ = 4.0;
    decider.dead_end_heading_reversal_count_ = 1;
    decider.current_pose_.pose.position.x = 5.0;
    decider.front_obstacle_streak_ = 0;
    decider.voxels_[{20, 20, 20}] = SuperExplorationDecider::OCCUPIED;
    decider.map_growth_reference_count_ = 1;
    const ros::Time now = ros::Time::now();
    decider.last_significant_map_growth_time_ = now - ros::Duration(5.0);
    std::vector<SuperExplorationDecider::FrontierCandidate> candidates;
    SuperExplorationDecider::VoxelSet reachable;
    const auto stable = decider.evaluateMapClosure(
        candidates, reachable, false);

    decider.current_pose_.pose.position.x = 0.5;
    const auto too_close_to_origin = decider.evaluateMapClosure(
        candidates, reachable, false);
    return !stable.front_boundary_seen && stable.complete &&
           !too_close_to_origin.complete;
  }

  static bool remoteReachableCellWithoutLocalRouteDoesNotKeepPassageOpen(
      SuperExplorationDecider& decider) {
    decider.have_home_ = true;
    decider.mission_heading_yaw_ = 0.0;
    decider.home_pose_.pose.position.z = 1.0;
    decider.current_pose_.pose.position.x = 2.0;
    decider.current_pose_.pose.position.z = 3.5;
    decider.cruise_height_above_home_ = 0.0;
    decider.min_observation_height_above_home_ = 0.0;
    decider.max_observation_height_above_home_ = 5.0;
    decider.front_obstacle_range_ = 3.0;
    SuperExplorationDecider::VoxelSet reachable;
    reachable.insert(decider.positionToKey(6.0, 0.0, 3.5));
    std::vector<SuperExplorationDecider::FrontierCandidate> candidates;
    const auto closure = decider.evaluateMapClosure(
        candidates, reachable, false);
    return closure.max_reachable_progress > 5.0 &&
           !closure.reachable_forward_passage;
  }

  static bool shortExecutablePassageIsStillOpen(
      SuperExplorationDecider& decider) {
    decider.have_home_ = true;
    decider.current_pose_.pose.position.x = 17.0;
    decider.super_occupied_stamp_ = ros::Time::now();
    decider.front_obstacle_range_ = 6.0;
    decider.min_goal_distance_ = 2.0;
    decider.forward_lookahead_distance_ = 4.0;
    decider.forward_goal_lateral_search_width_ = 0.0;
    SuperExplorationDecider::VoxelSet reachable;
    for (double x = 17.0; x <= 21.0 + 1e-9;
         x += 0.5 * decider.voxel_resolution_) {
      const auto key = decider.positionToKey(x, 0.0, 0.0);
      decider.voxels_[key] = SuperExplorationDecider::FREE;
      reachable.insert(key);
    }
    std::vector<SuperExplorationDecider::FrontierCandidate> candidates;
    return decider.evaluateMapClosure(candidates, reachable, true)
        .reachable_forward_passage;
  }

  static bool smallMapCountDropKeepsStableTimer(
      SuperExplorationDecider& decider) {
    decider.have_home_ = true;
    decider.map_closure_growth_voxels_ = 10;
    decider.map_growth_reference_count_ = 100;
    decider.last_significant_map_growth_time_ =
        ros::Time::now() - ros::Duration(5.0);
    for (int i = 0; i < 95; ++i)
      decider.voxels_[{i, 0, 0}] = SuperExplorationDecider::OCCUPIED;
    std::vector<SuperExplorationDecider::FrontierCandidate> candidates;
    SuperExplorationDecider::VoxelSet reachable;
    const auto small_drop = decider.evaluateMapClosure(
        candidates, reachable, false);
    for (int i = 95; i < 115; ++i)
      decider.voxels_[{i, 0, 0}] = SuperExplorationDecider::OCCUPIED;
    const auto large_growth = decider.evaluateMapClosure(
        candidates, reachable, false);
    return small_drop.stable_map_duration >= 4.0 &&
           large_growth.stable_map_duration < 1.0;
  }

  static bool pendingClosureConfirmationDefersStall(
      SuperExplorationDecider& decider) {
    decider.enabled_ = true;
    decider.have_data_ = true;
    decider.have_home_ = true;
    decider.exploration_started_ = true;
    decider.use_map_closure_completion_ = true;
    decider.map_closure_complete_streak_ = 3;
    decider.last_sync_time_ = ros::Time::now();
    decider.last_motion_progress_time_ =
        decider.last_sync_time_ - ros::Duration(decider.no_frontier_timeout_ + 1.0);
    decider.decisionTimerCallback(ros::TimerEvent());
    return !decider.returning_home_;
  }

  static bool fallbackPhaseStillAllowsKnownFreeForwardLookahead(
      SuperExplorationDecider& decider) {
    decider.have_home_ = true;
    decider.super_occupied_stamp_ = ros::Time::now();
    decider.mission_heading_yaw_ = 0.0;
    decider.exploration_phase_ =
        SuperExplorationDecider::ExplorationPhase::kFrontierFallback;
    decider.forward_lookahead_distance_ = 2.0;
    decider.forward_lookahead_step_ = 0.5;
    decider.min_goal_distance_ = 2.0;
    decider.max_exploration_radius_from_home_ = 10.0;
    SuperExplorationDecider::VoxelSet reachable;
    for (double x = 0.0; x <= 2.0 + 1e-9;
         x += 0.5 * decider.voxel_resolution_) {
      const auto key = decider.positionToKey(x, 0.0, 0.0);
      decider.voxels_[key] = SuperExplorationDecider::FREE;
      reachable.insert(key);
    }
    return decider.publishForwardLookaheadGoal(reachable) &&
           decider.have_active_goal_;
  }

  static bool failedForwardGoalIsNotReissued(
      SuperExplorationDecider& decider) {
    decider.have_home_ = true;
    decider.super_occupied_stamp_ = ros::Time::now();
    decider.mission_heading_yaw_ = 0.0;
    decider.forward_lookahead_distance_ = 2.0;
    decider.forward_lookahead_step_ = 0.5;
    decider.min_goal_distance_ = 1.0;
    decider.forward_goal_lateral_search_width_ = 0.0;
    decider.max_exploration_radius_from_home_ = 10.0;
    SuperExplorationDecider::VoxelSet reachable;
    for (double x = 0.0; x <= 2.0 + 1e-9;
         x += 0.5 * decider.voxel_resolution_) {
      const auto key = decider.positionToKey(x, 0.0, 0.0);
      decider.voxels_[key] = SuperExplorationDecider::FREE;
      reachable.insert(key);
    }
    if (!decider.publishForwardLookaheadGoal(reachable)) return false;
    const auto failed = decider.current_goal_.pose.position;
    decider.rememberFailedGoal(failed);
    decider.cancelActiveGoal("test_timeout");
    if (!decider.publishForwardLookaheadGoal(reachable)) return false;
    return decider.current_goal_.pose.position.x < failed.x - 0.75 &&
           !decider.isNearFailedGoal(decider.current_goal_.pose.position);
  }

  static bool shortCloudGapKeepsCertifiedGoal(
      SuperExplorationDecider& decider) {
    decider.have_home_ = true;
    decider.have_data_ = true;
    decider.have_active_goal_ = true;
    decider.active_goal_id_ = 42;
    decider.data_timeout_ = 1.0;
    decider.active_goal_data_grace_ = 2.0;
    decider.last_sync_time_ = ros::Time::now() - ros::Duration(1.5);
    decider.decisionTimerCallback(ros::TimerEvent());
    return decider.have_active_goal_ && decider.active_goal_id_ == 42;
  }

  static bool confirmedWallAllowsOnlyClearForwardApproach(
      SuperExplorationDecider& decider) {
    decider.have_home_ = true;
    decider.super_occupied_stamp_ = ros::Time::now();
    decider.mission_heading_yaw_ = 0.0;
    decider.front_obstacle_streak_ = decider.front_obstacle_confirm_frames_;
    decider.forward_lookahead_distance_ = 2.0;
    decider.forward_lookahead_step_ = 0.5;
    decider.min_goal_distance_ = 1.0;
    decider.max_exploration_radius_from_home_ = 10.0;
    SuperExplorationDecider::VoxelSet reachable;
    for (double x = 0.0; x <= 2.0 + 1e-9;
         x += 0.5 * decider.voxel_resolution_) {
      const auto key = decider.positionToKey(x, 0.0, 0.0);
      decider.voxels_[key] = SuperExplorationDecider::FREE;
      reachable.insert(key);
    }
    if (!decider.publishForwardLookaheadGoal(reachable) ||
        std::abs(decider.current_goal_.pose.position.x - 2.0) > 1e-9) {
      return false;
    }
    decider.cancelActiveGoal("test_reset");
    const auto obstruction = decider.positionToKey(1.0, 0.0, 0.0);
    reachable.erase(obstruction);
    return !decider.publishForwardLookaheadGoal(reachable);
  }

  static bool forwardLookaheadPrefersShorterLevelGoalOverClimb(
      SuperExplorationDecider& decider) {
    decider.have_home_ = true;
    decider.super_occupied_stamp_ = ros::Time::now();
    decider.mission_heading_yaw_ = 0.0;
    decider.forward_lookahead_distance_ = 2.0;
    decider.forward_lookahead_step_ = 0.5;
    decider.min_goal_distance_ = 1.0;
    decider.max_exploration_radius_from_home_ = 10.0;
    decider.cruise_height_above_home_ = 0.0;
    decider.min_observation_height_above_home_ = 0.0;
    decider.max_observation_height_above_home_ = 0.5;
    const auto shorter_level = decider.positionToKey(1.5, 0.0, 0.0);
    const auto farther_climb = decider.positionToKey(2.0, 0.0, 0.5);
    decider.voxels_[shorter_level] = SuperExplorationDecider::FREE;
    decider.voxels_[farther_climb] = SuperExplorationDecider::FREE;
    SuperExplorationDecider::VoxelSet reachable;
    for (double x = 0.0; x <= 1.5 + 1e-9;
         x += 0.5 * decider.voxel_resolution_) {
      const auto key = decider.positionToKey(x, 0.0, 0.0);
      decider.voxels_[key] = SuperExplorationDecider::FREE;
      reachable.insert(key);
    }
    reachable.insert(farther_climb);
    return decider.publishForwardLookaheadGoal(reachable) &&
           decider.have_active_goal_ &&
           std::abs(decider.current_goal_.pose.position.x - 1.5) < 1e-9 &&
           std::abs(decider.current_goal_.pose.position.z) < 1e-9;
  }

  static bool forwardLookaheadDescendsOnlyWhenLevelCorridorIsBlocked(
      SuperExplorationDecider& decider) {
    decider.voxel_resolution_ = 0.25;
    decider.vehicle_radius_ = 0.8;
    decider.preferred_vehicle_radius_ = 1.1;
    decider.vertical_vehicle_clearance_below_ = 0.5;
    decider.vertical_vehicle_clearance_above_ = 0.5;
    decider.have_home_ = true;
    decider.home_pose_.pose.position.z = 1.0;
    decider.current_pose_.pose.position.z = 1.0;
    decider.super_occupied_stamp_ = ros::Time::now();
    decider.mission_heading_yaw_ = 0.0;
    decider.forward_lookahead_distance_ = 2.0;
    decider.forward_lookahead_step_ = 0.5;
    decider.min_goal_distance_ = 2.0;
    decider.forward_goal_lateral_search_width_ = 0.0;
    decider.max_exploration_radius_from_home_ = 10.0;
    decider.cruise_height_above_home_ = 0.0;
    decider.min_observation_height_above_home_ = -0.25;
    decider.max_observation_height_above_home_ = 0.3;
    SuperExplorationDecider::VoxelSet reachable;
    for (double x = 0.0; x <= 2.0 + 1e-9; x += 0.125) {
      for (double z : {0.75, 1.0}) {
        const auto key = decider.positionToKey(x, 0.0, z);
        decider.voxels_[key] = SuperExplorationDecider::FREE;
        reachable.insert(key);
      }
    }
    // This roof voxel violates the 0.5 m upward envelope at cruise height,
    // but leaves the complete 0.25 m-lower segment vertically clear.
    decider.voxels_[decider.positionToKey(2.0, 0.0, 1.5)] =
        SuperExplorationDecider::OCCUPIED;
    return decider.publishForwardLookaheadGoal(reachable) &&
           decider.have_active_goal_ &&
           std::abs(decider.current_goal_.pose.position.x - 2.0) < 1e-9 &&
           std::abs(decider.current_goal_.pose.position.z - 0.75) < 1e-9;
  }

  static bool reversedOriginCanUseOneVoxelObservationStep(
      SuperExplorationDecider& decider) {
    decider.have_home_ = true;
    decider.super_occupied_stamp_ = ros::Time::now();
    decider.mission_heading_yaw_ = 0.0;
    decider.dead_end_heading_reversal_count_ = 1;
    decider.forward_lookahead_distance_ = 2.0;
    decider.forward_lookahead_step_ = 0.5;
    decider.min_goal_distance_ = 2.0;
    decider.voxel_resolution_ = 0.5;
    decider.max_exploration_radius_from_home_ = 10.0;
    decider.forward_goal_lateral_search_width_ = 0.0;
    const auto target = decider.positionToKey(0.5, 0.0, 0.0);
    const auto start = decider.positionToKey(0.0, 0.0, 0.0);
    decider.voxels_[target] = SuperExplorationDecider::FREE;
    decider.voxels_[start] = SuperExplorationDecider::FREE;
    SuperExplorationDecider::VoxelSet reachable;
    reachable.insert(target);
    reachable.insert(start);
    return decider.publishForwardLookaheadGoal(reachable) &&
           std::abs(decider.current_goal_.pose.position.x - 0.5) < 1e-9;
  }

  static bool finalBacktrackWaypointUsesTightArrivalRadius(
      SuperExplorationDecider& decider) {
    decider.enabled_ = true;
    decider.have_data_ = true;
    decider.have_home_ = true;
    decider.last_sync_time_ = ros::Time::now();
    decider.super_occupied_stamp_ = decider.last_sync_time_;
    decider.data_timeout_ = 100.0;
    decider.dead_end_backtracking_ = true;
    decider.goal_reached_distance_ = 1.0;
    decider.voxel_resolution_ = 0.5;
    decider.return_waypoint_index_ = 0;
    decider.return_waypoints_.resize(1);
    decider.current_goal_ = decider.return_waypoints_.front();
    for (double x = 0.0; x <= 1.0; x += decider.voxel_resolution_)
      decider.voxels_[decider.positionToKey(x, 0.0, 0.0)] =
          SuperExplorationDecider::FREE;
    decider.active_goal_id_ = 1;
    decider.have_active_goal_ = true;
    decider.current_pose_.pose.position.x = 0.5;
    decider.decisionTimerCallback(ros::TimerEvent());
    const bool half_metre_not_complete = decider.dead_end_backtracking_;
    decider.current_pose_.pose.position.x = 0.2;
    decider.last_sync_time_ = ros::Time::now();
    decider.decisionTimerCallback(ros::TimerEvent());
    return half_metre_not_complete && !decider.dead_end_backtracking_;
  }

  static bool intermediateReturnWaypointWaitsForActualArrival(
      SuperExplorationDecider& decider) {
    decider.enabled_ = true;
    decider.have_data_ = true;
    decider.have_home_ = true;
    decider.last_sync_time_ = ros::Time::now();
    decider.super_occupied_stamp_ = decider.last_sync_time_;
    decider.data_timeout_ = 100.0;
    decider.dead_end_backtracking_ = true;
    decider.goal_reached_distance_ = 1.0;
    decider.voxel_resolution_ = 0.5;
    decider.current_pose_.pose.position.x = 0.45;
    decider.current_goal_.pose.position.x = 1.0;
    decider.current_goal_.pose.orientation.w = 1.0;
    decider.active_goal_id_ = 7;
    decider.have_active_goal_ = true;
    for (double x = 0.0; x <= 1.0; x += 0.25)
      decider.voxels_[decider.positionToKey(x, 0.0, 0.0)] =
          SuperExplorationDecider::FREE;
    decider.decisionTimerCallback(ros::TimerEvent());
    const bool outside_half_metre = decider.have_active_goal_ &&
        decider.active_goal_id_ == 7 &&
        std::abs(decider.current_goal_.pose.position.x - 1.0) < 1e-9;
    decider.current_pose_.pose.position.x = 0.6;
    decider.last_sync_time_ = ros::Time::now();
    decider.decisionTimerCallback(ros::TimerEvent());
    return outside_half_metre && decider.active_goal_id_ != 7;
  }

  static bool executingReturnDoesNotUseAPoseToGoalStraightLine(
      SuperExplorationDecider& decider) {
    decider.have_data_ = true;
    decider.have_home_ = true;
    decider.last_sync_time_ = ros::Time::now();
    decider.super_occupied_stamp_ = decider.last_sync_time_;
    decider.data_timeout_ = 100.0;
    decider.dead_end_backtracking_ = true;
    decider.current_pose_.pose.position.x = 2.0;
    decider.current_pose_.pose.position.y = 0.97;
    decider.current_goal_.pose.position.x = 1.0;
    decider.current_goal_.pose.position.y = 1.1;
    decider.current_goal_.pose.orientation.w = 1.0;
    decider.active_goal_id_ = 7;
    decider.have_active_goal_ = true;
    // Remain just inside the scaled 0.9625 m return clearance so the direct
    // pose-to-goal chord is blocked while SUPER's certified curved trajectory
    // remains authoritative.
    decider.super_occupied_points_->push_back(pcl::PointXYZ(2.0, 0.05, 0.0));
    decider.super_occupied_tree_.setInputCloud(decider.super_occupied_points_);
    for (double x = 1.0; x <= 2.0; x += decider.voxel_resolution_)
      for (double y = 0.75; y <= 1.25; y += decider.voxel_resolution_)
        decider.voxels_[decider.positionToKey(x, y, 0.0)] =
            SuperExplorationDecider::FREE;
    if (decider.returnSegmentClear(decider.current_pose_.pose.position,
                                   decider.current_goal_.pose.position))
      return false;
    decider.decisionTimerCallback(ros::TimerEvent());
    return decider.have_active_goal_ && decider.active_goal_id_ == 7;
  }

  static bool shortLateralDetourIsNotImmediatelyComplete(
      SuperExplorationDecider& decider) {
    decider.enabled_ = true;
    decider.have_data_ = true;
    decider.have_home_ = true;
    decider.exploration_started_ = true;
    decider.last_sync_time_ = ros::Time::now();
    decider.data_timeout_ = 100.0;
    decider.goal_reached_distance_ = 1.0;
    decider.voxel_resolution_ = 0.5;
    decider.current_pose_.pose.orientation.w = 1.0;
    geometry_msgs::PoseStamped goal;
    goal.pose.position.y = 0.5;
    goal.pose.orientation.w = 1.0;
    decider.publishGoal(goal, "lateral_detour");
    decider.last_sync_time_ = ros::Time::now();
    decider.decisionTimerCallback(ros::TimerEvent());
    return decider.have_active_goal_ &&
           decider.active_goal_is_lateral_detour_;
  }

  static bool connectedBlockedRouteIsNotFrontWallPreempted(
      SuperExplorationDecider& decider) {
    decider.enabled_ = true;
    decider.have_data_ = true;
    decider.have_home_ = true;
    decider.exploration_started_ = true;
    decider.exploration_phase_ =
        SuperExplorationDecider::ExplorationPhase::kFrontierFallback;
    decider.last_sync_time_ = ros::Time::now();
    decider.data_timeout_ = 100.0;
    decider.goal_reached_distance_ = 1.0;
    decider.current_pose_.pose.orientation.w = 1.0;
    geometry_msgs::PoseStamped goal;
    goal.pose.position.x = 2.0;
    goal.pose.position.y = 1.0;
    goal.pose.orientation.w = 1.0;
    decider.publishGoal(goal, "blocked_frontier_route");
    decider.front_obstacle_streak_ = decider.front_obstacle_confirm_frames_;
    decider.last_sync_time_ = ros::Time::now();
    decider.decisionTimerCallback(ros::TimerEvent());
    return decider.have_active_goal_ &&
           decider.active_goal_is_blocked_frontier_route_;
  }

  static bool wallApproachIsHeldUntilHalfVoxelArrival(
      SuperExplorationDecider& decider) {
    decider.enabled_ = true;
    decider.have_data_ = true;
    decider.have_home_ = true;
    decider.exploration_started_ = true;
    decider.exploration_phase_ =
        SuperExplorationDecider::ExplorationPhase::kFrontierFallback;
    decider.last_sync_time_ = ros::Time::now();
    decider.data_timeout_ = 100.0;
    decider.goal_reached_distance_ = 1.0;
    decider.voxel_resolution_ = 0.5;
    decider.current_pose_.pose.orientation.w = 1.0;
    geometry_msgs::PoseStamped goal;
    goal.pose.position.x = 0.5;
    goal.pose.orientation.w = 1.0;
    decider.publishGoal(goal, "forward_lookahead");
    decider.front_obstacle_streak_ = decider.front_obstacle_confirm_frames_;
    decider.current_pose_.pose.position.x = 0.1;
    decider.last_sync_time_ = ros::Time::now();
    decider.decisionTimerCallback(ros::TimerEvent());
    const bool held_at_point_four =
        decider.have_active_goal_ && decider.active_goal_is_forward_lookahead_;
    decider.current_pose_.pose.position.x = 0.3;
    decider.last_sync_time_ = ros::Time::now();
    decider.decisionTimerCallback(ros::TimerEvent());
    return held_at_point_four && !decider.have_active_goal_;
  }

  static bool remoteFrontierBecomesShortReachableSegment(
      SuperExplorationDecider& decider) {
    decider.have_home_ = true;
    decider.super_occupied_stamp_ = ros::Time::now();
    decider.mission_heading_yaw_ = 0.0;
    decider.exploration_phase_ =
        SuperExplorationDecider::ExplorationPhase::kForwardPriority;
    decider.current_pose_.pose.orientation.w = 1.0;
    decider.min_goal_distance_ = 2.0;
    decider.voxel_resolution_ = 0.5;
    decider.max_frontier_goal_distance_ = 4.0;
    decider.max_frontier_goal_vertical_step_ = 0.35;

    SuperExplorationDecider::FrontierCandidate remote;
    remote.goal.pose.position.x = 20.0;
    remote.goal.pose.position.z = 5.0;
    remote.goal.pose.orientation.w = 1.0;
    remote.unknown_neighbors = 10;
    remote.key = decider.positionToKey(
        remote.goal.pose.position.x, remote.goal.pose.position.y,
        remote.goal.pose.position.z);

    SuperExplorationDecider::VoxelSet reachable;
    for (int x = 0; x <= remote.key.x; ++x) {
      const auto key = VoxelKey{x, 0, 0};
      reachable.insert(key);
      decider.voxels_[key] = SuperExplorationDecider::FREE;
    }
    for (int z = 1; z <= remote.key.z; ++z) {
      const auto key = VoxelKey{remote.key.x, 0, z};
      reachable.insert(key);
      decider.voxels_[key] = SuperExplorationDecider::FREE;
    }

    return decider.selectAndPublishFrontier({remote}, reachable) &&
           decider.have_active_goal_ &&
           std::hypot(decider.current_goal_.pose.position.x,
                      decider.current_goal_.pose.position.y) <= 4.0 + 1e-9 &&
           decider.current_goal_.pose.position.x >= 2.0 &&
           std::abs(decider.current_goal_.pose.position.z) <= 0.35 + 1e-9;
  }

  static bool headingPriorityFrontierYawFollowsLocalTangent(
      SuperExplorationDecider& decider) {
    decider.have_home_ = true;
    decider.super_occupied_stamp_ = ros::Time::now();
    decider.mission_heading_yaw_ = 0.0;
    decider.exploration_phase_ =
        SuperExplorationDecider::ExplorationPhase::kForwardPriority;
    decider.current_pose_.pose.position.x = 0.25;
    decider.current_pose_.pose.position.y = 0.25;
    decider.current_pose_.pose.position.z = 0.25;
    decider.current_pose_.pose.orientation.w = 1.0;
    decider.min_goal_distance_ = 1.0;
    decider.voxel_resolution_ = 0.5;
    decider.max_frontier_goal_distance_ = 4.0;
    decider.max_frontier_goal_vertical_step_ = 0.5;

    std::vector<VoxelKey> route{{0, 0, 0}};
    int x = 0;
    int y = 0;
    for (int block = 0; block < 10; ++block) {
      route.push_back({++x, y, 0});
      route.push_back({++x, y, 0});
      route.push_back({x, ++y, 0});
    }
    SuperExplorationDecider::VoxelSet reachable;
    for (const auto& key : route) {
      reachable.insert(key);
      decider.voxels_[key] = SuperExplorationDecider::FREE;
    }
    SuperExplorationDecider::FrontierCandidate frontier;
    frontier.key = route.back();
    frontier.goal.pose.position = decider.keyToPoint(frontier.key);
    frontier.goal.pose.orientation.w = 1.0;
    frontier.score = 10.0;
    frontier.unknown_neighbors = 10;
    if (!decider.selectAndPublishFrontier({frontier}, reachable)) {
      return false;
    }
    const auto& q = decider.current_goal_.pose.orientation;
    const double yaw = std::atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    const double tangent = std::atan2(
        decider.current_goal_.pose.position.y -
            decider.current_pose_.pose.position.y,
        decider.current_goal_.pose.position.x -
            decider.current_pose_.pose.position.x);
    return tangent > 0.1 && std::abs(yaw - tangent) < 1e-9;
  }

  static bool successfulHeadingReversalClearsSideHold(
      SuperExplorationDecider& decider) {
    decider.have_home_ = true;
    decider.have_data_ = true;
    decider.last_sync_time_ = ros::Time::now();
    decider.super_occupied_stamp_ = decider.last_sync_time_;
    decider.data_timeout_ = 100.0;
    decider.current_pose_.pose.position.x = 5.25;
    decider.current_pose_.pose.position.z = 0.25;
    decider.home_pose_.pose.position.x = 0.25;
    decider.home_pose_.pose.position.z = 0.25;
    decider.max_dead_end_heading_reversals_ = 1;
    decider.front_obstacle_min_progress_ = 1.0;
    decider.dead_end_reversal_delay_ = 1.0;
    decider.held_route_side_ = 1;
    for (int x = 0; x <= 10; ++x) {
      decider.voxels_[{x, 0, 0}] = SuperExplorationDecider::FREE;
    }
    SuperExplorationDecider::MapClosureStatus closure;
    closure.front_boundary_seen = true;
    closure.reachable_forward_passage = false;
    closure.actionable_frontiers = 0;
    closure.no_frontier_duration = 2.0;
    closure.vehicle_progress = 5.0;
    return decider.reverseMissionHeadingAtDeadEnd(closure) &&
           decider.held_route_side_ == 0;
  }

  static bool blockedFrontierContinuesAlongLocalConnectedRoute(
      SuperExplorationDecider& decider) {
    decider.have_home_ = true;
    decider.super_occupied_stamp_ = ros::Time::now();
    decider.mission_heading_yaw_ = 0.0;
    decider.exploration_phase_ =
        SuperExplorationDecider::ExplorationPhase::kFrontierFallback;
    decider.front_obstacle_streak_ = decider.front_obstacle_confirm_frames_;
    decider.current_pose_.pose.orientation.w = 1.0;
    decider.min_goal_distance_ = 1.0;
    decider.voxel_resolution_ = 0.5;
    decider.max_frontier_goal_distance_ = 2.0;
    decider.max_frontier_goal_vertical_step_ = 0.5;

    SuperExplorationDecider::FrontierCandidate branch;
    branch.goal.pose.position.y = 4.0;
    branch.goal.pose.orientation.w = 1.0;
    branch.key = decider.positionToKey(0.0, 4.0, 0.0);
    branch.score = 1.0;
    branch.unknown_neighbors = 5;

    SuperExplorationDecider::VoxelSet reachable;
    for (int y = 0; y <= branch.key.y; ++y) {
      const VoxelKey key{0, y, 0};
      reachable.insert(key);
      decider.voxels_[key] = SuperExplorationDecider::FREE;
    }
    if (!decider.selectAndPublishFrontier({branch}, reachable) ||
        !decider.have_active_goal_) {
      return false;
    }
    const auto& q = decider.current_goal_.pose.orientation;
    const double yaw = std::atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    const double local_route_yaw = std::atan2(
        decider.current_goal_.pose.position.y -
            decider.current_pose_.pose.position.y,
        decider.current_goal_.pose.position.x -
            decider.current_pose_.pose.position.x);
    return decider.current_goal_.pose.position.y >= 1.0 &&
           decider.current_goal_.pose.position.y <= 2.0 + 1e-9 &&
           std::abs(yaw - local_route_yaw) < 1e-9;
  }

  static bool smoothBendUsesConnectedLocalRoute(
      SuperExplorationDecider& decider) {
    decider.have_home_ = true;
    decider.super_occupied_stamp_ = ros::Time::now();
    decider.home_pose_.pose.orientation.w = 1.0;
    decider.mission_heading_yaw_ = 0.0;
    decider.exploration_phase_ =
        SuperExplorationDecider::ExplorationPhase::kFrontierFallback;
    decider.front_obstacle_streak_ = decider.front_obstacle_confirm_frames_;
    decider.current_pose_.pose.position.x = 0.25;
    decider.current_pose_.pose.position.y = 0.25;
    decider.current_pose_.pose.position.z = 0.25;
    decider.current_pose_.pose.orientation.w = 1.0;
    decider.min_goal_distance_ = 1.0;
    decider.voxel_resolution_ = 0.5;
    decider.max_frontier_goal_distance_ = 4.0;
    decider.max_frontier_goal_vertical_step_ = 0.5;
    decider.max_outbound_backtrack_ = 4.0;

    // Six-connected stair-step sampling of a smooth quarter-circle corridor.
    // The remote frontier is around a 90-degree bend, so a direct chord would
    // cut through the inside wall; the selected goal must stay on this route.
    const std::vector<VoxelKey> curve{
        {0, 0, 0}, {1, 0, 0}, {2, 0, 0}, {3, 0, 0},
        {4, 0, 0}, {4, 1, 0}, {5, 1, 0}, {6, 1, 0},
        {6, 2, 0}, {7, 2, 0}, {7, 3, 0}, {8, 3, 0},
        {8, 4, 0}, {9, 4, 0}, {9, 5, 0}, {10, 5, 0},
        {10, 6, 0}, {10, 7, 0}, {11, 7, 0}, {11, 8, 0},
        {11, 9, 0}, {12, 9, 0}, {12, 10, 0}, {12, 11, 0},
        {12, 12, 0}};
    SuperExplorationDecider::VoxelSet reachable;
    for (const VoxelKey& key : curve) {
      reachable.insert(key);
      decider.voxels_[key] = SuperExplorationDecider::FREE;
    }

    SuperExplorationDecider::FrontierCandidate bend;
    bend.key = curve.back();
    bend.goal.pose.position = decider.keyToPoint(bend.key);
    bend.goal.pose.orientation.w = 1.0;
    bend.score = 10.0;
    bend.unknown_neighbors = 12;
    if (!decider.selectAndPublishFrontier({bend}, reachable) ||
        !decider.have_active_goal_) {
      return false;
    }

    const VoxelKey selected = decider.positionToKey(
        decider.current_goal_.pose.position.x,
        decider.current_goal_.pose.position.y,
        decider.current_goal_.pose.position.z);
    const double distance = std::hypot(
        decider.current_goal_.pose.position.x -
            decider.current_pose_.pose.position.x,
        decider.current_goal_.pose.position.y -
            decider.current_pose_.pose.position.y);
    const auto& q = decider.current_goal_.pose.orientation;
    const double yaw = std::atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    return reachable.count(selected) == 1 && distance >= 1.0 &&
           distance <= 4.0 + 1e-9 && yaw > 0.0 && yaw < M_PI_4;
  }

  static bool occupancyEvidenceRejectsTransientNoise(
      SuperExplorationDecider& decider) {
    const VoxelKey key{4, 0, 0};
    decider.occupied_hit_increment_ = 2;
    decider.occupied_confirm_threshold_ = 4;
    decider.free_ray_decrement_ = 1;
    decider.free_confirm_threshold_ = 0;
    decider.markOccupied(key);
    if (!decider.isOccupied(key) || decider.isKnownFree(key)) return false;
    decider.markFree(key);
    decider.markFree(key);
    return decider.isKnownFree(key) && !decider.isOccupied(key);
  }

  static bool persistentOccupancyStillBlocks(
      SuperExplorationDecider& decider) {
    const VoxelKey key{5, 0, 0};
    decider.occupied_hit_increment_ = 2;
    decider.occupied_confirm_threshold_ = 4;
    decider.markOccupied(key);
    decider.markOccupied(key);
    return decider.isOccupied(key);
  }

  static bool preferredClearanceIsDistinctFromHardFloor(
      SuperExplorationDecider& decider) {
    decider.voxel_resolution_ = 0.25;
    decider.vehicle_radius_ = 1.0;
    decider.preferred_vehicle_radius_ = 1.5;
    decider.vertical_vehicle_clearance_below_ = 0.5;
    decider.vertical_vehicle_clearance_above_ = 0.5;
    decider.voxels_[{5, 0, 0}] = SuperExplorationDecider::OCCUPIED;
    return decider.isClearForVehicle({0, 0, 0}, 1.0) &&
           !decider.isClearForVehicle({0, 0, 0}, 1.5);
  }

  static bool oneSidedForwardSelectionPrioritizesComfortClearance(
      SuperExplorationDecider& decider) {
    decider.have_home_ = true;
    decider.mission_heading_yaw_ = 0.0;
    decider.min_goal_distance_ = 1.0;
    decider.forward_corridor_half_width_ = 3.0;
    decider.min_observation_height_above_home_ = 0.0;
    decider.max_observation_height_above_home_ = 2.0;
    decider.current_pose_.pose.position.x = 0.0;
    decider.home_pose_.pose.position.x = 0.0;
    decider.left_wall_visible_ = true;
    decider.right_wall_visible_ = false;

    SuperExplorationDecider::FrontierCandidate hard_candidate;
    hard_candidate.goal.pose.position.x = 4.0;
    hard_candidate.goal.pose.position.z = 1.0;
    hard_candidate.goal.pose.orientation.w = 1.0;
    hard_candidate.unknown_neighbors = 100;
    hard_candidate.preferred_clearance = false;

    SuperExplorationDecider::FrontierCandidate preferred_candidate;
    preferred_candidate.goal.pose.position.x = 2.0;
    preferred_candidate.goal.pose.position.z = 1.0;
    preferred_candidate.goal.pose.orientation.w = 1.0;
    preferred_candidate.preferred_clearance = true;

    const std::vector<SuperExplorationDecider::FrontierCandidate> candidates{
        hard_candidate, preferred_candidate};
    return !decider.corridorCenteringActive() &&
           decider.selectForwardCandidate(candidates) == &candidates.back();
  }

  static bool stalledExplorerUsesHardSafeForwardSegmentBeforeReturn(
      SuperExplorationDecider& decider) {
    decider.enabled_ = true;
    decider.have_data_ = true;
    decider.have_home_ = true;
    decider.exploration_started_ = true;
    decider.last_sync_time_ = ros::Time::now();
    decider.super_occupied_stamp_ = decider.last_sync_time_;
    decider.data_timeout_ = 100.0;
    decider.last_motion_progress_time_ =
        decider.last_sync_time_ -
        ros::Duration(decider.no_frontier_timeout_ + 1.0);
    decider.mission_heading_yaw_ = 0.0;
    decider.forward_lookahead_distance_ = 2.0;
    decider.forward_lookahead_step_ = 0.5;
    decider.min_goal_distance_ = 1.0;
    decider.forward_goal_lateral_search_width_ = 0.0;
    decider.max_exploration_radius_from_home_ = 10.0;
    decider.vehicle_radius_ = 0.5;
    decider.preferred_vehicle_radius_ = 1.25;
    for (double x = 0.0; x <= 2.0 + 1e-9;
         x += 0.5 * decider.voxel_resolution_) {
      decider.voxels_[decider.positionToKey(x, 0.0, 0.0)] =
          SuperExplorationDecider::FREE;
    }
    // This obstacle violates only the preferred envelope around the farther
    // segment. The hard vehicle-radius test still permits exploration.
    decider.voxels_[decider.positionToKey(2.0, 1.0, 0.0)] =
        SuperExplorationDecider::OCCUPIED;
    decider.decisionTimerCallback(ros::TimerEvent());
    return !decider.returning_home_ && decider.have_active_goal_ &&
           decider.current_goal_.pose.position.x >= 1.0;
  }

  static bool forwardSelectionUsesConfirmedCorridorCenter(
      SuperExplorationDecider& decider) {
    decider.have_home_ = true;
    decider.mission_heading_yaw_ = 0.0;
    decider.min_goal_distance_ = 1.0;
    decider.forward_corridor_half_width_ = 3.0;
    decider.min_observation_height_above_home_ = 0.0;
    decider.max_observation_height_above_home_ = 2.0;
    decider.current_pose_.pose.position.x = 0.0;
    decider.home_pose_.pose.position.x = 0.0;
    decider.left_wall_visible_ = true;
    decider.right_wall_visible_ = true;
    decider.front_obstacle_visible_ = false;
    decider.exploration_phase_ =
        SuperExplorationDecider::ExplorationPhase::kForwardPriority;
    decider.corridor_centering_enabled_ = true;
    decider.corridor_center_valid_ = true;
    decider.corridor_center_lateral_ = 1.0;

    SuperExplorationDecider::FrontierCandidate mission_axis;
    mission_axis.goal.pose.position.x = 4.0;
    mission_axis.goal.pose.position.y = 0.0;
    mission_axis.goal.pose.position.z = 1.0;
    mission_axis.preferred_clearance = true;

    SuperExplorationDecider::FrontierCandidate medial_axis = mission_axis;
    medial_axis.goal.pose.position.y = 1.0;
    medial_axis.preferred_clearance = false;

    const std::vector<SuperExplorationDecider::FrontierCandidate> candidates{
        mission_axis, medial_axis};
    return decider.selectForwardCandidate(candidates) == &candidates.back();
  }

  static bool medialAxisDominatesComfortUntilCenteringIsUnavailable(
      SuperExplorationDecider& decider) {
    decider.have_home_ = true;
    decider.super_occupied_stamp_ = ros::Time::now();
    decider.mission_heading_yaw_ = 0.0;
    decider.voxel_resolution_ = 0.5;
    decider.vehicle_radius_ = 0.1;
    decider.preferred_vehicle_radius_ = 0.75;
    decider.return_super_radius_ = 0.1;
    decider.return_clearance_margin_ = 0.0;
    decider.forward_lookahead_distance_ = 2.0;
    decider.forward_lookahead_step_ = 0.5;
    decider.min_goal_distance_ = 2.0;
    decider.forward_goal_lateral_search_width_ = 1.0;
    decider.max_exploration_radius_from_home_ = 10.0;
    decider.corridor_centering_enabled_ = true;
    decider.corridor_center_valid_ = true;
    decider.corridor_center_lateral_ = 0.0;
    decider.exploration_phase_ =
        SuperExplorationDecider::ExplorationPhase::kForwardPriority;
    decider.left_wall_visible_ = true;
    decider.right_wall_visible_ = true;
    decider.front_obstacle_visible_ = false;

    SuperExplorationDecider::VoxelSet reachable;
    for (int x = 0; x <= 4; ++x) {
      for (int y = -2; y <= 2; ++y) {
        const VoxelKey key{x, y, 0};
        decider.voxels_[key] = SuperExplorationDecider::FREE;
        reachable.insert(key);
      }
    }
    // The obstacle makes the medial-axis segment hard-safe but not comfort-
    // clear. The left segment retains the full comfort radius.
    decider.super_occupied_points_->push_back(
        pcl::PointXYZ(2.0f, 0.6f, 0.0f));
    decider.super_occupied_tree_.setInputCloud(
        decider.super_occupied_points_);

    SuperExplorationDecider::LocalRouteCandidate centered;
    if (!decider.selectForwardLookaheadGoal(reachable, false, &centered) ||
        std::abs(centered.mission_lateral) > 1e-9 ||
        centered.preferred_clearance) {
      return false;
    }

    decider.right_wall_visible_ = false;
    SuperExplorationDecider::LocalRouteCandidate comfort;
    return !decider.corridorCenteringActive() &&
           decider.selectForwardLookaheadGoal(reachable, false, &comfort) &&
           comfort.preferred_clearance && comfort.mission_lateral < -0.5;
  }

  static bool diagonalKnownFreeCorridorIsReachable(
      SuperExplorationDecider& decider) {
    decider.have_home_ = true;
    decider.voxel_resolution_ = 0.5;
    decider.vehicle_radius_ = 1.0;
    decider.vertical_vehicle_radius_ = 1.0;
    decider.frontier_search_radius_ = 20.0;
    decider.max_task_lateral_offset_ = 20.0;
    decider.min_observation_height_above_home_ = -5.0;
    decider.max_observation_height_above_home_ = 5.0;
    decider.current_pose_.pose.position.x = 0.25;
    decider.current_pose_.pose.position.y = 0.25;
    decider.current_pose_.pose.position.z = 0.25;
    decider.home_pose_ = decider.current_pose_;
    for (int i = 0; i <= 8; ++i) {
      for(int dx=-1;dx<=1;++dx) for(int dy=-1;dy<=1;++dy)
        decider.voxels_[{i+dx, i+dy, 0}] = SuperExplorationDecider::FREE;
      decider.voxels_[{i + 1, i, 0}] = SuperExplorationDecider::FREE;
      decider.voxels_[{i, i + 1, 0}] = SuperExplorationDecider::FREE;
    }
    const auto reachable = decider.reachableFreeVoxels();
    return reachable.count(VoxelKey{8, 8, 0}) == 1;
  }

  static bool longCorridorAndRealWall(SuperExplorationDecider& d) {
    d.voxel_resolution_=0.5;d.vehicle_radius_=1.0;d.vertical_vehicle_radius_=1.0;
    d.have_home_=true;d.frontier_search_radius_=40;d.max_task_lateral_offset_=4;
    d.min_observation_height_above_home_=-2;d.max_observation_height_above_home_=2;
    d.current_pose_.pose.position.x=.25;d.current_pose_.pose.position.y=.25;d.current_pose_.pose.position.z=.25;
    d.home_pose_=d.current_pose_;
    for(int x=0;x<=72;++x)for(int y=-6;y<=6;++y)for(int z=-4;z<=4;++z)
      d.voxels_[{x,y,z}]=(std::abs(y)==6||std::abs(z)==4)?SuperExplorationDecider::OCCUPIED:SuperExplorationDecider::FREE;
    d.markOccupied({20,0,0}); d.markFree({20,0,0});d.markFree({20,0,0});
    if(!d.reachableFreeVoxels().count({65,0,0}))return false;
    for(int y=-6;y<=6;++y)for(int z=-4;z<=4;++z){d.markOccupied({40,y,z});d.markOccupied({40,y,z});}
    return !d.reachableFreeVoxels().count({65,0,0});
  }
  static bool noCornerCutThroughUnknown(SuperExplorationDecider& d) {
    d.voxels_[{0,0,0}]=SuperExplorationDecider::FREE;
    d.voxels_[{1,1,0}]=SuperExplorationDecider::FREE;
    return !d.transitionClear({0,0,0},{1,1,0});
  }
  static bool denseSingleScanIsOneHit(SuperExplorationDecider& d) {
    d.voxel_resolution_=0.25; d.max_points_per_cloud_=5000;
    pcl::PointCloud<pcl::PointXYZ> hits;
    for(int i=0;i<1000;++i) hits.points.emplace_back(2.05f,0.05f,0.05f);
    sensor_msgs::PointCloud2 cloud; pcl::toROSMsg(hits,cloud);
    cloud.header.stamp=ros::Time::now(); geometry_msgs::PoseStamped pose;
    d.updateMap(cloud,pose); const VoxelKey key=d.positionToKey(2.05,0.05,0.05);
    return d.occupancy_evidence_[key]==d.occupied_hit_increment_ && d.isOccupied(key);
  }
  static bool traversalCannotEraseOccupied(SuperExplorationDecider& d) {
    geometry_msgs::Point p; const VoxelKey k=d.positionToKey(0,0,0);
    d.markOccupied(k); d.markOccupied(k); d.protectTraversedCenterline(p);
    return d.isOccupied(k) && !d.isKnownFree(k);
  }
  static bool fixedMissionBandTruncatesADeepBend(
      SuperExplorationDecider& decider) {
    decider.have_home_ = true;
    decider.mission_heading_yaw_ = 0.0;
    decider.voxel_resolution_ = 0.5;
    decider.max_task_lateral_offset_ = 2.0;
    decider.frontier_search_radius_ = 20.0;
    decider.current_pose_.pose.position.x = 0.25;
    decider.current_pose_.pose.position.y = 0.25;
    decider.current_pose_.pose.position.z = 0.25;
    for (int y = 0; y <= 12; ++y) {
      decider.voxels_[{0, y, 0}] = SuperExplorationDecider::FREE;
    }
    const auto reachable = decider.reachableFreeVoxels();
    return reachable.count(VoxelKey{0, 3, 0}) == 1 &&
           reachable.count(VoxelKey{0, 4, 0}) == 0 &&
           reachable.count(VoxelKey{0, 12, 0}) == 0;
  }

  static bool forwardRouteUsesWholePathClearanceAndSideHysteresis(
      SuperExplorationDecider& decider) {
    decider.have_home_ = true;
    decider.super_occupied_stamp_ = ros::Time::now();
    decider.mission_heading_yaw_ = 0.0;
    decider.voxel_resolution_ = 0.5;
    decider.vehicle_radius_ = 0.1;
    decider.preferred_vehicle_radius_ = 0.1;
    decider.return_super_radius_ = 0.1;
    decider.return_clearance_margin_ = 0.0;
    decider.forward_lookahead_distance_ = 2.0;
    decider.forward_lookahead_step_ = 0.5;
    decider.min_goal_distance_ = 2.0;
    decider.forward_goal_lateral_search_width_ = 1.0;
    decider.max_exploration_radius_from_home_ = 10.0;
    decider.lateral_route_switch_clearance_advantage_ = 0.25;

    SuperExplorationDecider::VoxelSet reachable;
    for (int x = 0; x <= 4; ++x) {
      for (int y = -2; y <= 2; ++y) {
        const VoxelKey key{x, y, 0};
        decider.voxels_[key] = SuperExplorationDecider::FREE;
        reachable.insert(key);
      }
    }

    // With equal clearance, retain the previously selected left side.
    decider.held_route_side_ = 1;
    SuperExplorationDecider::LocalRouteCandidate held;
    if (!decider.selectForwardLookaheadGoal(reachable, false, &held) ||
        held.mission_lateral <= 0.0) {
      return false;
    }

    // A real minimum-clearance advantage larger than the hysteresis must
    // override that hold and select the open right-hand route.
    decider.super_occupied_points_->push_back(
        pcl::PointXYZ(2.0f, 1.4f, 0.0f));
    decider.super_occupied_tree_.setInputCloud(
        decider.super_occupied_points_);
    SuperExplorationDecider::LocalRouteCandidate switched;
    if (!decider.selectForwardLookaheadGoal(reachable, false, &switched) ||
        switched.mission_lateral >= 0.0) {
      return false;
    }
    const double yaw = 2.0 * std::atan2(
        switched.goal.pose.orientation.z,
        switched.goal.pose.orientation.w);
    const double expected = std::atan2(
        switched.goal.pose.position.y - decider.current_pose_.pose.position.y,
        switched.goal.pose.position.x - decider.current_pose_.pose.position.x);
    return std::abs(yaw - expected) < 1e-9;
  }

  static bool lateralDetourYawFollowsLocalTangent(
      SuperExplorationDecider& decider) {
    decider.have_home_ = true;
    decider.super_occupied_stamp_ = ros::Time::now();
    decider.mission_heading_yaw_ = 0.0;
    decider.current_pose_.pose.position.x = 2.0;
    decider.current_pose_.pose.position.z = 0.0;
    decider.front_obstacle_streak_ = decider.front_obstacle_confirm_frames_;
    decider.vehicle_radius_ = 0.1;
    decider.preferred_vehicle_radius_ = 0.1;
    decider.return_super_radius_ = 0.1;
    decider.return_clearance_margin_ = 0.0;

    SuperExplorationDecider::FrontierCandidate opening;
    opening.goal.pose.position.x = 2.0;
    opening.goal.pose.position.y = -2.0;
    opening.unknown_neighbors = 10;
    SuperExplorationDecider::VoxelSet reachable;
    const geometry_msgs::Point start = decider.current_pose_.pose.position;
    geometry_msgs::Point target;
    target.x = 1.75;
    target.y = -2.0;
    const int samples = static_cast<int>(std::ceil(
        std::hypot(target.x - start.x, target.y - start.y) /
        (0.5 * decider.voxel_resolution_)));
    for (int i = 0; i <= samples; ++i) {
      const double t = static_cast<double>(i) / samples;
      const auto key = decider.positionToKey(
          start.x + t * (target.x - start.x),
          start.y + t * (target.y - start.y), 0.0);
      decider.voxels_[key] = SuperExplorationDecider::FREE;
      reachable.insert(key);
    }
    if (!decider.publishLateralDetourGoal(reachable, {opening}) ||
        !decider.active_goal_is_lateral_detour_) {
      return false;
    }
    const double yaw = 2.0 * std::atan2(
        decider.current_goal_.pose.orientation.z,
        decider.current_goal_.pose.orientation.w);
    const double expected = std::atan2(
        decider.current_goal_.pose.position.y - start.y,
        decider.current_goal_.pose.position.x - start.x);
    return std::abs(yaw - expected) < 1e-9 &&
           std::abs(yaw - decider.mission_heading_yaw_) > 0.5;
  }

  static bool knownFreeSidePassageDoesNotRequireAFrontier(
      SuperExplorationDecider& decider) {
    decider.have_home_ = true;
    decider.super_occupied_stamp_ = ros::Time::now();
    decider.mission_heading_yaw_ = 0.0;
    decider.voxel_resolution_ = 0.25;
    decider.vehicle_radius_ = 0.1;
    decider.preferred_vehicle_radius_ = 0.1;
    decider.return_super_radius_ = 0.1;
    decider.return_clearance_margin_ = 0.0;
    decider.max_task_lateral_offset_ = 10.0;
    decider.current_pose_.pose.position.x = 8.0;
    decider.current_pose_.pose.position.z = 0.0;
    decider.front_obstacle_streak_ =
        decider.front_obstacle_confirm_frames_;

    SuperExplorationDecider::VoxelSet reachable;
    // Model four metres of directly observed free space to the right without
    // adding an unknown-boundary frontier at its far end.
    for (int ix = -2; ix <= 1; ++ix) {
      for (int iy = -17; iy <= 1; ++iy) {
        const VoxelKey key{32 + ix, iy, 0};
        decider.voxels_[key] = SuperExplorationDecider::FREE;
        reachable.insert(key);
      }
    }

    if (!decider.publishLateralDetourGoal(reachable, {}) ||
        !decider.active_goal_is_lateral_detour_ ||
        decider.current_goal_.pose.position.y >= -3.5) {
      return false;
    }
    const double yaw = 2.0 * std::atan2(
        decider.current_goal_.pose.orientation.z,
        decider.current_goal_.pose.orientation.w);
    return yaw < -1.5 && yaw > -1.7;
  }

  static bool unusableRawFrontierDoesNotSuppressStallReturn(
      SuperExplorationDecider& decider) {
    decider.enabled_ = true;
    decider.have_data_ = true;
    decider.have_home_ = true;
    decider.exploration_started_ = true;
    decider.use_map_closure_completion_ = false;
    decider.require_three_wall_completion_ = false;
    decider.last_sync_time_ = ros::Time::now();
    decider.super_occupied_stamp_ = decider.last_sync_time_;
    decider.data_timeout_ = 100.0;
    decider.no_frontier_timeout_ = 1.0;
    decider.last_motion_progress_time_ =
        decider.last_sync_time_ - ros::Duration(2.0);
    decider.current_pose_.pose.position.x = 5.25;
    decider.current_pose_.pose.position.z = 0.25;
    decider.home_pose_.pose.position.x = 0.25;
    decider.home_pose_.pose.position.z = 0.25;
    for (int x = 4; x <= 10; ++x) {
      decider.voxels_[{x, 0, 0}] = SuperExplorationDecider::FREE;
    }
    SuperExplorationDecider::VoxelSet reachable;
    const auto candidates = decider.findFrontiers(reachable);
    if (candidates.empty()) return false;
    decider.decisionTimerCallback(ros::TimerEvent());
    return decider.returning_home_;
  }

  static bool initialGoalHandshakeIsBoundToEachMissionEpoch(
      SuperExplorationDecider& decider) {
    decider.enabled_ = true;
    decider.have_active_goal_ = true;
    decider.active_goal_id_ = 41;

    std_msgs::UInt64 accepted;
    accepted.data = 40;
    decider.acceptedGoalCallback(
        boost::make_shared<std_msgs::UInt64>(accepted));
    if (decider.initial_accepted_goal_id_ != 0) return false;
    accepted.data = 41;
    decider.acceptedGoalCallback(
        boost::make_shared<std_msgs::UInt64>(accepted));
    if (decider.initial_accepted_goal_id_ != 41) return false;

    std_msgs::Bool disabled;
    disabled.data = false;
    decider.missionEnableCallback(boost::make_shared<std_msgs::Bool>(disabled));
    if (decider.initial_accepted_goal_id_ != 0) return false;

    // Model the next enabled epoch after mission state has been rebuilt. A
    // delayed acceptance from goal 41 must not authorize the new goal 42.
    decider.enabled_ = true;
    decider.have_active_goal_ = true;
    decider.active_goal_id_ = 42;
    accepted.data = 41;
    decider.acceptedGoalCallback(
        boost::make_shared<std_msgs::UInt64>(accepted));
    if (decider.initial_accepted_goal_id_ != 0) return false;
    accepted.data = 42;
    decider.acceptedGoalCallback(
        boost::make_shared<std_msgs::UInt64>(accepted));
    return decider.initial_accepted_goal_id_ == 42;
  }

  static void reset(SuperExplorationDecider& decider) {
    std_srvs::Trigger::Request request;
    std_srvs::Trigger::Response response;
    ASSERT_TRUE(decider.resetCallback(request, response));
  }

  static std::size_t subscribers(const SuperExplorationDecider& decider) {
    return decider.goal_publisher_.getNumSubscribers();
  }
};

TEST(SuperGoalPublication, StartupReplacementAndResetUseOneOrderedTopic) {
  if (!ros::isInitialized()) {
    int argc = 0;
    ros::init(argc, nullptr, "super_goal_publication_test",
              ros::init_options::AnonymousName);
  }
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  private_nh.setParam("goal_command_topic", "/mine_uav/test/goal_command");
  private_nh.setParam("require_mission_enable_edge", true);
  SuperExplorationDecider decider(nh, private_nh);

  std::mutex mutex;
  std::vector<super_planner::GoalCommand> received;
  auto subscriber = nh.subscribe<super_planner::GoalCommand>(
      "/mine_uav/test/goal_command", 20,
      [&](const super_planner::GoalCommand::ConstPtr& message) {
        std::lock_guard<std::mutex> lock(mutex);
        received.push_back(*message);
      });
  ros::AsyncSpinner spinner(1);
  spinner.start();
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::seconds(5);
  while (ros::ok() &&
         SuperExplorationDeciderTestPeer::subscribers(decider) == 0 &&
         std::chrono::steady_clock::now() < deadline) {
    ros::WallDuration(0.01).sleep();
  }
  ASSERT_GT(SuperExplorationDeciderTestPeer::subscribers(decider), 0u);

  geometry_msgs::PoseStamped first;
  first.pose.position.x = 3.0;
  first.pose.orientation.w = 1.0;
  SuperExplorationDeciderTestPeer::publish(decider, first, "test_first");
  EXPECT_TRUE(
      SuperExplorationDeciderTestPeer::emptyForwardHandoverPreservesGoal(decider));
  geometry_msgs::PoseStamped second = first;
  second.pose.position.x = 6.0;
  SuperExplorationDeciderTestPeer::publish(decider, second,
                                          "forward_lookahead", true);
  geometry_msgs::PoseStamped third = second;
  third.pose.position.x = 7.0;
  SuperExplorationDeciderTestPeer::publish(decider, third, "test_hard");
  SuperExplorationDeciderTestPeer::reset(decider);

  while (ros::ok() && std::chrono::steady_clock::now() < deadline) {
    {
      std::lock_guard<std::mutex> lock(mutex);
      if (received.size() >= 6) break;
    }
    ros::WallDuration(0.01).sleep();
  }
  {
    std::lock_guard<std::mutex> lock(mutex);
    ASSERT_EQ(received.size(), 6u);
    EXPECT_EQ(received[0].command, super_planner::GoalCommand::CANCEL_GOAL);
    EXPECT_EQ(received[0].goal_id, 0u);
    EXPECT_EQ(received[1].command, super_planner::GoalCommand::SET_GOAL);
    EXPECT_EQ(received[1].goal_id, 1u);
    EXPECT_EQ(received[1].reason, "test_first");
    EXPECT_EQ(received[2].command, super_planner::GoalCommand::SET_GOAL);
    EXPECT_EQ(received[2].goal_id, 2u);
    EXPECT_EQ(received[2].reason, "forward_lookahead");
    EXPECT_EQ(received[3].command, super_planner::GoalCommand::CANCEL_GOAL);
    EXPECT_EQ(received[3].goal_id, 2u);
    EXPECT_EQ(received[4].command, super_planner::GoalCommand::SET_GOAL);
    EXPECT_EQ(received[4].goal_id, 3u);
    EXPECT_EQ(received[5].command, super_planner::GoalCommand::CANCEL_GOAL);
    EXPECT_EQ(received[5].goal_id, 0u);
  }
  spinner.stop();
}

TEST(SuperGoalPublication, ReturnPrefersSurveyedLaneInOpenSpace) {
  if (!ros::isInitialized()) {
    int argc = 0;
    ros::init(argc, nullptr, "super_return_breadcrumb_test",
              ros::init_options::AnonymousName);
  }
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  private_nh.setParam("goal_command_topic", "/mine_uav/test/return_goal_command");
  SuperExplorationDecider decider(nh, private_nh);
  EXPECT_TRUE(SuperExplorationDeciderTestPeer::returnPrefersSurveyedLane(
      decider));
}

TEST(SuperGoalPublication, LowVoltageImmediatelyStartsVerifiedBreadcrumbReturn) {
  ros::NodeHandle nh;
  SuperExplorationDecider decider(nh, ros::NodeHandle("~voltage_return"));
  EXPECT_TRUE(SuperExplorationDeciderTestPeer::
              lowVoltageImmediatelyStartsVerifiedBreadcrumbReturn(decider));
}

TEST(SuperGoalPublication, LowPercentageRequiresSustainedConfirmation) {
  ros::NodeHandle nh;
  SuperExplorationDecider decider(nh, ros::NodeHandle("~percentage_return"));
  EXPECT_TRUE(SuperExplorationDeciderTestPeer::
                  lowPercentageRequiresSustainedConfirmation(decider));
}

TEST(SuperGoalPublication, ReturnTransitionDuringBacktrackNeverMovesAway) {
  ros::NodeHandle nh;
  SuperExplorationDecider decider(nh, ros::NodeHandle("~backtrack_return"));
  EXPECT_TRUE(SuperExplorationDeciderTestPeer::
                  returnTransitionDuringBacktrackNeverMovesAwayFromHome(
                      decider));
}

TEST(SuperGoalPublication, FaultReturnFollowsBentOutboundLane) {
  ros::NodeHandle nh;
  SuperExplorationDecider decider(nh, ros::NodeHandle("~fault_return"));
  EXPECT_TRUE(SuperExplorationDeciderTestPeer::
              faultReturnFollowsBentOutboundLane(decider));
}

TEST(SuperGoalPublication, ReturnGoalRespectsHeightCeiling) {
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~return_height_ceiling");
  SuperExplorationDecider decider(nh, private_nh);
  EXPECT_TRUE(SuperExplorationDeciderTestPeer::returnGoalRespectsHeightCeiling(
      decider));
}

TEST(SuperGoalPublication, ReturnOffsetStaysInsideTheClearLane) {
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~return_clear_lane");
  SuperExplorationDecider decider(nh, private_nh);
  EXPECT_TRUE(SuperExplorationDeciderTestPeer::returnOffsetStaysInsideTheClearLane(
      decider));
}

TEST(SuperGoalPublication, ReturnChecksCurrentSuperMapAndMapAge) {
  if (!ros::isInitialized()) {
    int argc = 0;
    ros::init(argc, nullptr, "super_return_map_test",
              ros::init_options::AnonymousName);
  }
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  SuperExplorationDecider decider(nh, private_nh);
  EXPECT_TRUE(SuperExplorationDeciderTestPeer::returnChecksSuperObstacleAndFreshness(
      decider));
}

TEST(SuperGoalPublication, ReturnCanStartBelowAnUnsafeVoxelCentre) {
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  SuperExplorationDecider decider(nh, private_nh);
  EXPECT_TRUE(SuperExplorationDeciderTestPeer::returnCanStartBelowAnUnsafeVoxelCentre(decider));
}

TEST(SuperGoalPublication, NearHomeCanUseSubmetreClearStep) {
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  SuperExplorationDecider decider(nh, private_nh);
  EXPECT_TRUE(SuperExplorationDeciderTestPeer::
                  nearHomeCanUseSubmetreClearStep(decider));
}

TEST(SuperGoalPublication, LaunchAreaArrivalNeedsTwoObservations) {
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  SuperExplorationDecider decider(nh, private_nh);
  EXPECT_TRUE(SuperExplorationDeciderTestPeer::
                  launchAreaArrivalNeedsTwoObservations(decider));
}

TEST(SuperGoalPublication, SimulatedNoReturnRayCannotClearBehindWall) {
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  private_nh.setParam("goal_command_topic", "/mine_uav/test/free_ray_goal_command");
  SuperExplorationDecider decider(nh, private_nh);
  EXPECT_TRUE(SuperExplorationDeciderTestPeer::freeRayStopsAtOccupiedCell(decider));
}

TEST(SuperGoalPublication, ReversedShortBranchRequiresLiveWallEvidence) {
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  private_nh.setParam("goal_command_topic",
                      "/mine_uav/test/reversed_boundary_goal_command");
  SuperExplorationDecider decider(nh, private_nh);
  EXPECT_TRUE(
      SuperExplorationDeciderTestPeer::reversedBranchUsesOnlyNearbyLiveBoundary(
          decider));
}

TEST(SuperGoalPublication, ReversedStableReachableComponentCanCloseWithoutWall) {
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  private_nh.setParam("goal_command_topic",
                      "/mine_uav/test/reversed_component_goal_command");
  SuperExplorationDecider decider(nh, private_nh);
  EXPECT_TRUE(SuperExplorationDeciderTestPeer::
                  reversedStableReachableComponentCanCloseWithoutWall(decider));
}

TEST(SuperGoalPublication, SuccessfulHeadingReversalClearsSideHold) {
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~heading_reversal_side_hold");
  SuperExplorationDecider decider(nh, private_nh);
  EXPECT_TRUE(SuperExplorationDeciderTestPeer::
                  successfulHeadingReversalClearsSideHold(decider));
}

TEST(SuperGoalPublication, RemoteReachableCellWithoutLocalRouteIsClosed) {
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  SuperExplorationDecider decider(nh, private_nh);
  EXPECT_TRUE(SuperExplorationDeciderTestPeer::
                  remoteReachableCellWithoutLocalRouteDoesNotKeepPassageOpen(
                      decider));
}

TEST(SuperGoalPublication, ShortExecutablePassageIsStillOpen) {
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~short_passage");
  SuperExplorationDecider decider(nh, private_nh);
  EXPECT_TRUE(SuperExplorationDeciderTestPeer::shortExecutablePassageIsStillOpen(
      decider));
}

TEST(SuperGoalPublication, SmallMapCountDropKeepsStableTimer) {
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~stable_map_count");
  SuperExplorationDecider decider(nh, private_nh);
  EXPECT_TRUE(SuperExplorationDeciderTestPeer::
                  smallMapCountDropKeepsStableTimer(decider));
}

TEST(SuperGoalPublication, PendingClosureConfirmationDefersStall) {
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~pending_closure");
  SuperExplorationDecider decider(nh, private_nh);
  EXPECT_TRUE(SuperExplorationDeciderTestPeer::
                  pendingClosureConfirmationDefersStall(decider));
}

TEST(SuperGoalPublication, FallbackPhaseDoesNotBlockKnownFreeForwardLookahead) {
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  private_nh.setParam("goal_command_topic",
                      "/mine_uav/test/fallback_forward_goal_command");
  SuperExplorationDecider decider(nh, private_nh);
  EXPECT_TRUE(SuperExplorationDeciderTestPeer::
                  fallbackPhaseStillAllowsKnownFreeForwardLookahead(decider));
}

TEST(SuperGoalPublication, FailedForwardGoalIsNotReissued) {
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  SuperExplorationDecider decider(nh, private_nh);
  EXPECT_TRUE(SuperExplorationDeciderTestPeer::failedForwardGoalIsNotReissued(decider));
}

TEST(SuperGoalPublication, ShortCloudGapKeepsCertifiedGoal) {
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  SuperExplorationDecider decider(nh, private_nh);
  EXPECT_TRUE(SuperExplorationDeciderTestPeer::shortCloudGapKeepsCertifiedGoal(decider));
}

TEST(SuperGoalPublication, ConfirmedWallAllowsOnlyClearForwardApproach) {
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  private_nh.setParam("goal_command_topic",
                      "/mine_uav/test/wall_approach_goal_command");
  SuperExplorationDecider decider(nh, private_nh);
  EXPECT_TRUE(SuperExplorationDeciderTestPeer::
                  confirmedWallAllowsOnlyClearForwardApproach(decider));
}

TEST(SuperGoalPublication, ForwardLookaheadPrefersShorterLevelGoalOverClimb) {
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  private_nh.setParam("goal_command_topic",
                      "/mine_uav/test/level_forward_goal_command");
  SuperExplorationDecider decider(nh, private_nh);
  EXPECT_TRUE(SuperExplorationDeciderTestPeer::
                  forwardLookaheadPrefersShorterLevelGoalOverClimb(decider));
}

TEST(SuperGoalPublication, ForwardLookaheadDescendsWhenLevelCorridorBlocked) {
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~lower_clear_corridor");
  SuperExplorationDecider decider(nh, private_nh);
  EXPECT_TRUE(SuperExplorationDeciderTestPeer::
                  forwardLookaheadDescendsOnlyWhenLevelCorridorIsBlocked(
                      decider));
}

TEST(SuperGoalPublication, ReversedOriginCanUseOneVoxelObservationStep) {
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  private_nh.setParam("goal_command_topic",
                      "/mine_uav/test/reversed_origin_step_goal_command");
  SuperExplorationDecider decider(nh, private_nh);
  EXPECT_TRUE(SuperExplorationDeciderTestPeer::
                  reversedOriginCanUseOneVoxelObservationStep(decider));
}

TEST(SuperGoalPublication, FinalBacktrackWaypointUsesTightArrivalRadius) {
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  private_nh.setParam("goal_command_topic",
                      "/mine_uav/test/tight_backtrack_goal_command");
  SuperExplorationDecider decider(nh, private_nh);
  EXPECT_TRUE(SuperExplorationDeciderTestPeer::
                  finalBacktrackWaypointUsesTightArrivalRadius(decider));
}

TEST(SuperGoalPublication, IntermediateReturnWaypointWaitsForActualArrival) {
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  SuperExplorationDecider decider(nh, private_nh);
  EXPECT_TRUE(SuperExplorationDeciderTestPeer::intermediateReturnWaypointWaitsForActualArrival(decider));
}

TEST(SuperGoalPublication, ConfirmedObstacleStillAllowsVerifiedHandover) {
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  SuperExplorationDecider decider(nh, private_nh);
  EXPECT_TRUE(SuperExplorationDeciderTestPeer::
                  acceptedShortForwardGoalHandsOver(decider));
}

TEST(SuperGoalPublication, ForwardLookaheadVariantsKeepHandoverSemantics) {
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  SuperExplorationDecider decider(nh, private_nh);
  EXPECT_TRUE(SuperExplorationDeciderTestPeer::
                  forwardLookaheadVariantsKeepHandoverSemantics(decider));
}

TEST(SuperGoalPublication, ReturnHandsOverOnlyAcceptedProgressiveLeg) {
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  SuperExplorationDecider decider(nh, private_nh);
  EXPECT_TRUE(SuperExplorationDeciderTestPeer::
                  returnHandsOverOnlyAcceptedProgressiveLeg(decider));
}

TEST(SuperGoalPublication, ExecutingReturnKeepsSupersCertifiedTrajectory) {
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  SuperExplorationDecider decider(nh, private_nh);
  EXPECT_TRUE(SuperExplorationDeciderTestPeer::
                  executingReturnDoesNotUseAPoseToGoalStraightLine(decider));
}


TEST(SuperGoalPublication, ShortLateralDetourIsNotImmediatelyComplete) {
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  private_nh.setParam("goal_command_topic",
                      "/mine_uav/test/lateral_detour_goal_command");
  SuperExplorationDecider decider(nh, private_nh);
  EXPECT_TRUE(SuperExplorationDeciderTestPeer::
                  shortLateralDetourIsNotImmediatelyComplete(decider));
}

TEST(SuperGoalPublication, ConnectedBlockedRouteIsNotFrontWallPreempted) {
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  private_nh.setParam("goal_command_topic",
                      "/mine_uav/test/blocked_route_hold_goal_command");
  SuperExplorationDecider decider(nh, private_nh);
  EXPECT_TRUE(SuperExplorationDeciderTestPeer::
                  connectedBlockedRouteIsNotFrontWallPreempted(decider));
}

TEST(SuperGoalPublication, WallApproachIsHeldUntilHalfVoxelArrival) {
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  private_nh.setParam("goal_command_topic",
                      "/mine_uav/test/wall_approach_hold_goal_command");
  SuperExplorationDecider decider(nh, private_nh);
  EXPECT_TRUE(SuperExplorationDeciderTestPeer::
                  wallApproachIsHeldUntilHalfVoxelArrival(decider));
}

TEST(SuperGoalPublication, RemoteFrontierBecomesShortReachableSegment) {
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  private_nh.setParam("goal_command_topic",
                      "/mine_uav/test/local_frontier_goal_command");
  SuperExplorationDecider decider(nh, private_nh);
  EXPECT_TRUE(SuperExplorationDeciderTestPeer::
                  remoteFrontierBecomesShortReachableSegment(decider));
}

TEST(SuperGoalPublication, HeadingPriorityFrontierYawFollowsLocalTangent) {
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~heading_priority_tangent_yaw");
  SuperExplorationDecider decider(nh, private_nh);
  EXPECT_TRUE(SuperExplorationDeciderTestPeer::
                  headingPriorityFrontierYawFollowsLocalTangent(decider));
}

TEST(SuperGoalPublication, BlockedFrontierContinuesAlongLocalConnectedRoute) {
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  private_nh.setParam("goal_command_topic",
                      "/mine_uav/test/blocked_frontier_goal_command");
  SuperExplorationDecider decider(nh, private_nh);
  EXPECT_TRUE(SuperExplorationDeciderTestPeer::
                  blockedFrontierContinuesAlongLocalConnectedRoute(decider));
}

TEST(SuperGoalPublication, SmoothBendUsesConnectedLocalRoute) {
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  private_nh.setParam("goal_command_topic",
                      "/mine_uav/test/smooth_bend_goal_command");
  SuperExplorationDecider decider(nh, private_nh);
  EXPECT_TRUE(SuperExplorationDeciderTestPeer::
                  smoothBendUsesConnectedLocalRoute(decider));
}

TEST(SuperGoalPublication, OccupancyEvidenceRejectsTransientNoise) {
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  SuperExplorationDecider decider(nh, private_nh);
  EXPECT_TRUE(SuperExplorationDeciderTestPeer::
                  occupancyEvidenceRejectsTransientNoise(decider));
}

TEST(SuperGoalPublication, PersistentOccupancyStillBlocks) {
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  SuperExplorationDecider decider(nh, private_nh);
  EXPECT_TRUE(SuperExplorationDeciderTestPeer::
                  persistentOccupancyStillBlocks(decider));
}

TEST(SuperGoalPublication, DiagonalKnownFreeCorridorIsReachable) {
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  SuperExplorationDecider decider(nh, private_nh);
  EXPECT_TRUE(SuperExplorationDeciderTestPeer::
                  diagonalKnownFreeCorridorIsReachable(decider));
}

TEST(SuperGoalPublication, FullOneMetreEnvelopeLongCorridorAndRealWall) {
  ros::NodeHandle nh,pnh("~"); SuperExplorationDecider d(nh,pnh);
  EXPECT_TRUE(SuperExplorationDeciderTestPeer::longCorridorAndRealWall(d));
}

TEST(SuperGoalPublication, NoCornerCutThroughUnknown) {
  ros::NodeHandle nh,pnh("~"); SuperExplorationDecider d(nh,pnh);
  EXPECT_TRUE(SuperExplorationDeciderTestPeer::noCornerCutThroughUnknown(d));
}

TEST(SuperGoalPublication, PreferredClearanceIsDistinctFromHardFloor) {
  ros::NodeHandle nh, pnh("~");
  SuperExplorationDecider decider(nh, pnh);
  EXPECT_TRUE(SuperExplorationDeciderTestPeer::
                  preferredClearanceIsDistinctFromHardFloor(decider));
}

TEST(SuperGoalPublication,
     OneSidedForwardSelectionPrioritizesComfortClearance) {
  ros::NodeHandle nh, pnh("~");
  SuperExplorationDecider decider(nh, pnh);
  EXPECT_TRUE(SuperExplorationDeciderTestPeer::
                  oneSidedForwardSelectionPrioritizesComfortClearance(
                      decider));
}

TEST(SuperGoalPublication,
     StalledExplorerUsesHardSafeForwardSegmentBeforeReturn) {
  ros::NodeHandle nh, pnh("~");
  pnh.setParam("goal_command_topic",
               "/mine_uav/test/exploration_first_goal_command");
  SuperExplorationDecider decider(nh, pnh);
  EXPECT_TRUE(SuperExplorationDeciderTestPeer::
                  stalledExplorerUsesHardSafeForwardSegmentBeforeReturn(
                      decider));
}

TEST(SuperGoalPublication, CorridorCenterEstimateRejectsOneSidedEvidence) {
  const std::vector<double> left(12, 2.5);
  const std::vector<double> no_right;
  EXPECT_FALSE(estimateCorridorCenter(
      left, no_right, 0.0, 12, 2.0, 6.0).valid);
}

TEST(SuperGoalPublication, CorridorCenterEstimateUsesRobustWallMedians) {
  std::vector<double> left(12, 2.5);
  std::vector<double> right(12, -1.5);
  left.push_back(20.0);
  right.push_back(-20.0);
  const CorridorCenterEstimate estimate = estimateCorridorCenter(
      left, right, 0.0, 12, 2.0, 6.0);
  ASSERT_TRUE(estimate.valid);
  EXPECT_NEAR(estimate.center_lateral, 0.5, 1e-9);
  EXPECT_NEAR(estimate.width, 4.0, 1e-9);
}

TEST(SuperGoalPublication, CorridorCenteringRequiresStableTwoSidedEvidence) {
  ros::NodeHandle nh, pnh("~");
  SuperExplorationDecider decider(nh, pnh);
  EXPECT_TRUE(SuperExplorationDeciderTestPeer::
                  corridorCenteringRequiresStableTwoSidedEvidence(decider));
}

TEST(SuperGoalPublication, CenteredOffsetsKeepAllFallbackChoices) {
  const std::vector<double> offsets =
      centeredLateralOffsets(0.5, 2.0, 0.25);
  ASSERT_FALSE(offsets.empty());
  EXPECT_NEAR(offsets.front(), 0.5, 1e-9);
  EXPECT_NE(std::find(offsets.begin(), offsets.end(), 2.0), offsets.end());
  EXPECT_NE(std::find(offsets.begin(), offsets.end(), -2.0), offsets.end());
}

TEST(SuperGoalPublication, CorridorPreviewAnticipatesOffsetNarrowing) {
  std::vector<std::vector<bool>> passable(
      12, std::vector<bool>(17, false));
  for (std::size_t row = 0; row < passable.size(); ++row) {
    const std::size_t first = row < 4 ? 1 : 10;
    const std::size_t last = row < 4 ? 15 : 14;
    for (std::size_t column = first; column <= last; ++column) {
      passable[row][column] = true;
    }
  }
  const CorridorPreviewEstimate preview = estimateCorridorPreview(
      passable, 1.0, 0.5, -2.0, 0.25, 0.0, 1.0, 6);
  ASSERT_TRUE(preview.valid);
  EXPECT_EQ(preview.slices, 12u);
  EXPECT_GT(preview.target_lateral, 0.5);
  EXPECT_GT(preview.near_width - preview.minimum_width, 0.5);
}

TEST(SuperGoalPublication, CorridorPreviewRejectsUnreachableLateralJump) {
  std::vector<std::vector<bool>> passable(
      10, std::vector<bool>(17, false));
  for (std::size_t row = 0; row < passable.size(); ++row) {
    const std::size_t first = row < 3 ? 5 : 14;
    const std::size_t last = row < 3 ? 7 : 16;
    for (std::size_t column = first; column <= last; ++column) {
      passable[row][column] = true;
    }
  }
  EXPECT_FALSE(estimateCorridorPreview(
      passable, 1.0, 0.5, -2.0, 0.25, 0.0, 0.5, 6).valid);
}

TEST(SuperGoalPublication, ForwardSelectionUsesConfirmedCorridorCenter) {
  ros::NodeHandle nh, pnh("~");
  SuperExplorationDecider decider(nh, pnh);
  EXPECT_TRUE(SuperExplorationDeciderTestPeer::
                  forwardSelectionUsesConfirmedCorridorCenter(decider));
}

TEST(SuperGoalPublication,
     MedialAxisDominatesComfortUntilCenteringIsUnavailable) {
  ros::NodeHandle nh, pnh("~center_over_comfort");
  SuperExplorationDecider decider(nh, pnh);
  EXPECT_TRUE(SuperExplorationDeciderTestPeer::
                  medialAxisDominatesComfortUntilCenteringIsUnavailable(
                      decider));
}

TEST(SuperGoalPublication, ForwardRouteUsesClearanceAndSideHysteresis) {
  ros::NodeHandle nh, pnh("~clearance_side_selection");
  SuperExplorationDecider decider(nh, pnh);
  EXPECT_TRUE(SuperExplorationDeciderTestPeer::
                  forwardRouteUsesWholePathClearanceAndSideHysteresis(
                      decider));
}

TEST(SuperGoalPublication, LateralDetourYawFollowsLocalTangent) {
  ros::NodeHandle nh, pnh("~lateral_tangent_yaw");
  SuperExplorationDecider decider(nh, pnh);
  EXPECT_TRUE(SuperExplorationDeciderTestPeer::
                  lateralDetourYawFollowsLocalTangent(decider));
}

TEST(SuperGoalPublication, KnownFreeSidePassageDoesNotRequireAFrontier) {
  ros::NodeHandle nh, pnh("~known_free_side_passage");
  SuperExplorationDecider decider(nh, pnh);
  EXPECT_TRUE(SuperExplorationDeciderTestPeer::
                  knownFreeSidePassageDoesNotRequireAFrontier(decider));
}

TEST(SuperGoalPublication, UnusableRawFrontierDoesNotSuppressStallReturn) {
  ros::NodeHandle nh, pnh("~stalled_unusable_frontier");
  SuperExplorationDecider decider(nh, pnh);
  EXPECT_TRUE(SuperExplorationDeciderTestPeer::
                  unusableRawFrontierDoesNotSuppressStallReturn(decider));
}

TEST(SuperGoalPublication, DenseSingleScanIsOneHit) {
  ros::NodeHandle nh,pnh("~"); SuperExplorationDecider d(nh,pnh);
  EXPECT_TRUE(SuperExplorationDeciderTestPeer::denseSingleScanIsOneHit(d));
}

TEST(SuperGoalPublication, TraversalCannotEraseOccupied) {
  ros::NodeHandle nh,pnh("~"); SuperExplorationDecider d(nh,pnh);
  EXPECT_TRUE(SuperExplorationDeciderTestPeer::traversalCannotEraseOccupied(d));
}

TEST(SuperGoalPublication, FixedMissionBandTruncatesADeepBend) {
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  private_nh.setParam("goal_command_topic",
                      "/mine_uav/test/fixed_band_goal_command");
  SuperExplorationDecider decider(nh, private_nh);
  EXPECT_TRUE(SuperExplorationDeciderTestPeer::
                  fixedMissionBandTruncatesADeepBend(decider));
}

TEST(SuperGoalPublication, InitialGoalHandshakeIsBoundToEachMissionEpoch) {
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~initial_goal_epoch");
  SuperExplorationDecider decider(nh, private_nh);
  EXPECT_TRUE(SuperExplorationDeciderTestPeer::
                  initialGoalHandshakeIsBoundToEachMissionEpoch(decider));
}

}  // namespace mine_uav_control

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "super_goal_publication_test");
  return RUN_ALL_TESTS();
}
