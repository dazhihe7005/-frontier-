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
  static bool schedulerSelectsOnlyTheProfileForTheActiveExplorationTask(
      SuperExplorationDecider& decider) {
    decider.follow_active_task_profile_ = true;
    decider.enabled_ = false;
    decider.tunnel_vehicle_radius_ = 0.8;
    decider.goaf_vehicle_radius_ = 1.0;
    decider.tunnel_preferred_vehicle_radius_ = 1.1;
    decider.goaf_preferred_vehicle_radius_ = 1.5;

    auto enabled = boost::make_shared<std_msgs::Bool>();
    enabled->data = true;
    decider.missionEnableCallback(enabled);
    if (decider.enabled_) return false;  // No task identity yet.

    auto task = boost::make_shared<std_msgs::UInt8>();
    task->data = 4;
    decider.activeTaskCallback(task);
    if (!decider.enabled_ ||
        decider.mission_profile_ != ExplorationMissionProfile::kTunnel ||
        !decider.corridorGuidanceEnabled() ||
        std::abs(decider.vehicle_radius_ - 0.8) > 1e-9 ||
        std::abs(decider.preferred_vehicle_radius_ - 1.1) > 1e-9) {
      return false;
    }

    task->data = 0;
    decider.activeTaskCallback(task);
    if (decider.enabled_) return false;
    enabled->data = false;
    decider.missionEnableCallback(enabled);

    task->data = 1;
    decider.activeTaskCallback(task);
    enabled->data = true;
    decider.missionEnableCallback(enabled);
    return decider.enabled_ &&
        decider.mission_profile_ == ExplorationMissionProfile::kGoaf &&
        std::abs(decider.vehicle_radius_ - 1.0) <= 1e-9 &&
        std::abs(decider.preferred_vehicle_radius_ - 1.5) <= 1e-9 &&
        !decider.corridorGuidanceEnabled();
  }

  static void publish(SuperExplorationDecider& decider,
                      const geometry_msgs::PoseStamped& goal,
                      const std::string& reason,
                      bool continuous_handover = false) {
    decider.publishGoal(goal, reason, continuous_handover);
  }

  static bool goafUsesLongerPreviewWithoutRetuningTunnel(
      SuperExplorationDecider& decider) {
    decider.forward_lookahead_distance_ = 4.0;
    decider.goaf_forward_lookahead_distance_ = 6.0;
    decider.exploration_goal_queue_horizon_ = 10.0;
    decider.goaf_exploration_goal_queue_horizon_ = 15.0;
    decider.mission_profile_ = ExplorationMissionProfile::kGoaf;
    if (decider.effectiveForwardLookaheadDistance() != 6.0 ||
        decider.effectiveExplorationGoalQueueHorizon() != 15.0) {
      return false;
    }
    decider.mission_profile_ = ExplorationMissionProfile::kTunnel;
    return decider.effectiveForwardLookaheadDistance() == 4.0 &&
        decider.effectiveExplorationGoalQueueHorizon() == 10.0;
  }

  static bool batteryAdmissionRequiresOutboundAndReturnReserve(
      SuperExplorationDecider& decider) {
    decider.battery_start_guard_enabled_ = true;
    decider.battery_percentage_return_enabled_ = true;
    decider.battery_start_minimum_percentage_ = 0.50;
    decider.battery_start_minimum_voltage_ = 22.2;
    decider.have_battery_ = false;
    decider.battery_percentage_ = -1.0;
    decider.battery_voltage_ = -1.0;
    if (decider.batteryReadyForMissionStart()) return false;

    decider.have_battery_ = true;
    decider.battery_percentage_ = 0.29;
    decider.battery_voltage_ = 22.12;
    if (decider.batteryReadyForMissionStart()) return false;

    decider.battery_percentage_ = 0.75;
    if (decider.batteryReadyForMissionStart()) return false;

    decider.battery_voltage_ = 23.5;
    return decider.batteryReadyForMissionStart();
  }

  static bool task1ReturnsAtFiveMetresFromConfirmedEndWall(
      SuperExplorationDecider& decider) {
    decider.task1_end_wall_return_enabled_ = true;
    decider.task1_end_wall_return_distance_ = 5.0;
    decider.front_obstacle_confirm_frames_ = 3;
    decider.front_obstacle_streak_ = 3;
    decider.mission_profile_ = ExplorationMissionProfile::kGoaf;
    SuperExplorationDecider::ThreeWallCoverage coverage;
    coverage.end_wall_found = true;
    coverage.end_depth = 40.0;
    coverage.vehicle_progress = 35.0;
    if (!decider.task1EndWallReturnReady(coverage)) return false;
    coverage.vehicle_progress = 34.9;
    if (decider.task1EndWallReturnReady(coverage)) return false;
    coverage.vehicle_progress = 35.0;
    decider.front_obstacle_streak_ = 2;
    if (decider.task1EndWallReturnReady(coverage)) return false;
    decider.front_obstacle_streak_ = 3;
    decider.mission_profile_ = ExplorationMissionProfile::kTunnel;
    return !decider.task1EndWallReturnReady(coverage);
  }

  static bool disabledBatteryAdmissionIgnoresUnreliablePercentage(
      SuperExplorationDecider& decider) {
    decider.battery_start_guard_enabled_ = false;
    decider.have_battery_ = false;
    decider.battery_percentage_ = -1.0;
    decider.battery_voltage_ = -1.0;
    return decider.batteryReadyForMissionStart();
  }

  static bool voltageOnlyBatteryAdmissionIgnoresUnreliablePercentage(
      SuperExplorationDecider& decider) {
    decider.battery_start_guard_enabled_ = true;
    decider.battery_percentage_return_enabled_ = false;
    decider.battery_start_minimum_voltage_ = 22.2;
    decider.have_battery_ = false;
    decider.battery_percentage_ = -1.0;
    decider.battery_voltage_ = 22.3;
    if (!decider.batteryReadyForMissionStart()) return false;
    decider.battery_voltage_ = 22.1;
    return !decider.batteryReadyForMissionStart();
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
    decider.mission_profile_ = ExplorationMissionProfile::kTunnel;
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

  static bool rollingForwardQueueHandsOverOneGoalAtATime(
      SuperExplorationDecider& decider) {
    decider.have_home_ = true;
    decider.super_occupied_stamp_ = ros::Time::now();
    decider.mission_heading_yaw_ = 0.0;
    decider.voxel_resolution_ = 0.5;
    decider.vehicle_radius_ = 0.4;
    decider.preferred_vehicle_radius_ = 0.4;
    decider.forward_lookahead_distance_ = 4.0;
    decider.forward_lookahead_step_ = 0.5;
    decider.forward_goal_lateral_search_width_ = 0.0;
    decider.min_goal_distance_ = 2.0;
    decider.max_exploration_radius_from_home_ = 20.0;
    decider.exploration_goal_queue_enabled_ = true;
    decider.exploration_goal_queue_max_future_goals_ = 3;
    decider.exploration_goal_queue_horizon_ = 10.0;
    decider.exploration_goal_queue_spacing_ = 2.5;
    decider.exploration_goal_queue_end_reserve_ = 1.0;
    SuperExplorationDecider::VoxelSet reachable;
    for (double x = 0.0; x <= 15.0 + 1e-9; x += 0.125) {
      const auto key = decider.positionToKey(x, 0.0, 0.0);
      decider.voxels_[key] = SuperExplorationDecider::FREE;
      reachable.insert(key);
    }
    if (!decider.publishForwardLookaheadGoal(reachable) ||
        decider.exploration_goal_queue_.size() != 2u ||
        std::abs(decider.current_goal_.pose.position.x - 4.0) > 1e-9) {
      return false;
    }
    const double first_successor =
        decider.exploration_goal_queue_.front().goal.pose.position.x;
    const uint64_t first_goal_id = decider.active_goal_id_;
    decider.accepted_goal_id_ = first_goal_id;
    decider.current_pose_.pose.position.x = 2.5;
    decider.super_occupied_stamp_ = ros::Time::now();
    if (!decider.tryForwardHandover(reachable)) return false;
    return decider.active_goal_id_ != first_goal_id &&
           std::abs(decider.current_goal_.pose.position.x -
                    first_successor) < 1e-9 &&
           decider.exploration_goal_queue_.size() == 2u &&
           decider.exploration_goal_queue_.front().goal.pose.position.x >
               decider.current_goal_.pose.position.x;
  }

  static bool rollingForwardQueueDropsInvalidTail(
      SuperExplorationDecider& decider) {
    decider.have_home_ = true;
    decider.super_occupied_stamp_ = ros::Time::now();
    decider.mission_heading_yaw_ = 0.0;
    decider.voxel_resolution_ = 0.5;
    decider.vehicle_radius_ = 0.4;
    decider.preferred_vehicle_radius_ = 0.4;
    decider.forward_lookahead_distance_ = 4.0;
    decider.forward_lookahead_step_ = 0.5;
    decider.forward_goal_lateral_search_width_ = 0.0;
    decider.min_goal_distance_ = 2.0;
    decider.max_exploration_radius_from_home_ = 20.0;
    decider.exploration_goal_queue_enabled_ = true;
    decider.exploration_goal_queue_max_future_goals_ = 3;
    decider.exploration_goal_queue_horizon_ = 10.0;
    decider.exploration_goal_queue_spacing_ = 2.5;
    decider.exploration_goal_queue_end_reserve_ = 1.0;
    SuperExplorationDecider::VoxelSet reachable;
    for (double x = 0.0; x <= 10.0 + 1e-9; x += 0.125) {
      const auto key = decider.positionToKey(x, 0.0, 0.0);
      decider.voxels_[key] = SuperExplorationDecider::FREE;
      reachable.insert(key);
    }
    if (!decider.publishForwardLookaheadGoal(reachable) ||
        decider.exploration_goal_queue_.empty()) {
      return false;
    }
    const auto newly_blocked = decider.positionToKey(5.0, 0.0, 0.0);
    reachable.erase(newly_blocked);
    decider.voxels_[newly_blocked] = SuperExplorationDecider::OCCUPIED;
    decider.pruneExplorationGoalQueue(reachable);
    return decider.exploration_goal_queue_.empty() &&
           decider.have_active_goal_;
  }

  static bool cancellingExplorationClearsRollingQueue(
      SuperExplorationDecider& decider) {
    SuperExplorationDecider::QueuedExplorationGoal queued;
    queued.goal.pose.orientation.w = 1.0;
    queued.route.push_back({0, 0, 0});
    decider.exploration_goal_queue_.push_back(queued);
    decider.have_active_goal_ = true;
    decider.active_goal_id_ = 9;
    decider.cancelActiveGoal("test_queue_cancel");
    return decider.exploration_goal_queue_.empty() &&
           !decider.active_goal_allows_rolling_handover_;
  }

  static bool frontierRouteCachesConnectedSuccessors(
      SuperExplorationDecider& decider) {
    decider.have_home_ = true;
    decider.home_pose_.pose.position.x = 0.25;
    decider.home_pose_.pose.position.y = 0.25;
    decider.home_pose_.pose.position.z = 0.25;
    decider.current_pose_ = decider.home_pose_;
    decider.super_occupied_stamp_ = ros::Time::now();
    decider.mission_heading_yaw_ = 0.0;
    decider.exploration_phase_ =
        SuperExplorationDecider::ExplorationPhase::kForwardPriority;
    decider.voxel_resolution_ = 0.5;
    decider.vehicle_radius_ = 0.4;
    decider.preferred_vehicle_radius_ = 0.4;
    decider.min_goal_distance_ = 2.0;
    decider.max_frontier_goal_distance_ = 4.0;
    decider.max_frontier_goal_vertical_step_ = 0.5;
    decider.max_exploration_radius_from_home_ = 20.0;
    decider.exploration_goal_queue_enabled_ = true;
    decider.exploration_goal_queue_max_future_goals_ = 3;
    decider.exploration_goal_queue_horizon_ = 10.0;
    decider.exploration_goal_queue_spacing_ = 2.5;
    decider.exploration_goal_queue_end_reserve_ = 1.0;

    SuperExplorationDecider::VoxelSet reachable;
    for (int x = 0; x <= 24; ++x) {
      const VoxelKey key{x, 0, 0};
      reachable.insert(key);
      decider.voxels_[key] = SuperExplorationDecider::FREE;
    }
    SuperExplorationDecider::FrontierCandidate frontier;
    frontier.key = {24, 0, 0};
    frontier.goal.pose.position = decider.keyToPoint(frontier.key);
    frontier.goal.pose.orientation.w = 1.0;
    frontier.score = 10.0;
    frontier.unknown_neighbors = 10;
    if (!decider.selectAndPublishFrontier({frontier}, reachable) ||
        decider.exploration_goal_queue_.size() != 2u ||
        !decider.active_goal_allows_rolling_handover_) {
      return false;
    }
    const double next_x =
        decider.exploration_goal_queue_.front().goal.pose.position.x;
    const uint64_t previous_id = decider.active_goal_id_;
    decider.accepted_goal_id_ = previous_id;
    decider.current_pose_.pose.position.x =
        decider.current_goal_.pose.position.x - 1.5;
    decider.super_occupied_stamp_ = ros::Time::now();
    return decider.tryForwardHandover(reachable) &&
           decider.active_goal_id_ != previous_id &&
           std::abs(decider.current_goal_.pose.position.x - next_x) < 1e-9 &&
           decider.exploration_goal_queue_.size() == 1u;
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

  static bool acceptedReturnGoalPassedWhilePlanningNeverCommandsTurnAway(
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
    // Reproduce the final flight's delayed acceptance: the aircraft has
    // already moved more than a metre closer to home than goal 47.
    decider.current_goal_.pose.position.x = 5.9;
    decider.active_goal_id_ = 47;
    decider.accepted_goal_id_ = 47;
    decider.have_active_goal_ = true;
    decider.active_goal_initial_distance_ = 1.1;
    for (double x = 0.0; x <= 7.0; x += 0.25)
      decider.voxels_[decider.positionToKey(x, 0.0, 0.0)] =
          SuperExplorationDecider::FREE;

    decider.decisionTimerCallback(ros::TimerEvent());
    return decider.have_active_goal_ && decider.active_goal_id_ != 47 &&
           decider.current_goal_.pose.position.x < 4.8;
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
    decider.exploration_started_ = true;
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
    decider.battery_percentage_return_enabled_ = true;
    decider.enabled_ = true;
    decider.have_home_ = true;
    decider.have_data_ = true;
    decider.exploration_started_ = true;
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

  static bool tunnelReturnRetracesInclinedBreadcrumbs(
      SuperExplorationDecider& decider) {
    decider.mission_profile_ = ExplorationMissionProfile::kTunnel;
    decider.tunnel_min_planning_height_ = -5.0;
    decider.tunnel_max_planning_height_ = 8.0;
    decider.have_home_ = true;
    decider.have_data_ = true;
    decider.last_sync_time_ = ros::Time::now();
    decider.super_occupied_stamp_ = decider.last_sync_time_;
    decider.home_pose_.pose.position.z = 1.5;
    decider.home_pose_.pose.orientation.w = 1.0;
    decider.current_pose_.pose.position.x = 4.0;
    decider.current_pose_.pose.position.z = 3.5;
    for (double x : {0.0, 2.0, 4.0}) {
      geometry_msgs::PoseStamped breadcrumb;
      breadcrumb.pose.position.x = x;
      breadcrumb.pose.position.z = 1.5 + 0.5 * x;
      breadcrumb.pose.orientation.w = 1.0;
      decider.outbound_breadcrumbs_.push_back(breadcrumb);
    }
    for (double x = 0.0; x <= 4.0; x += 0.25) {
      const double center_z = 1.5 + 0.5 * x;
      for (double y = -1.0; y <= 1.0; y += 0.25) {
        for (double dz = -0.75; dz <= 0.75; dz += 0.25) {
          decider.voxels_[decider.positionToKey(x, y, center_z + dz)] =
              SuperExplorationDecider::FREE;
        }
      }
    }
    decider.beginReturnHome("test_inclined_tunnel_return");
    return decider.returning_home_ && decider.have_active_goal_ &&
           decider.current_goal_.pose.position.x < 4.0 &&
           decider.current_goal_.pose.position.z > 1.7 &&
           decider.current_goal_.pose.position.z < 3.5;
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

  static bool zeroMissionRadiusAllowsDistantForwardPassage(
      SuperExplorationDecider& decider) {
    decider.have_home_ = true;
    decider.home_pose_.pose.position.x = 0.0;
    decider.current_pose_.pose.position.x = 50.0;
    decider.super_occupied_stamp_ = ros::Time::now();
    decider.max_exploration_radius_from_home_ = 0.0;
    decider.min_goal_distance_ = 2.0;
    decider.forward_lookahead_distance_ = 4.0;
    decider.forward_lookahead_step_ = 0.5;
    decider.forward_goal_lateral_search_width_ = 0.0;
    decider.vehicle_radius_ = 0.1;
    decider.preferred_vehicle_radius_ = 0.1;

    SuperExplorationDecider::VoxelSet reachable;
    for (double x = 50.0; x <= 54.0 + 1e-9;
         x += 0.5 * decider.voxel_resolution_) {
      const auto key = decider.positionToKey(x, 0.0, 0.0);
      decider.voxels_[key] = SuperExplorationDecider::FREE;
      reachable.insert(key);
    }
    std::vector<SuperExplorationDecider::FrontierCandidate> candidates;
    const auto closure = decider.evaluateMapClosure(
        candidates, reachable, true);
    return closure.max_reachable_progress > 53.0 &&
           closure.reachable_forward_passage;
  }

  static bool tunnelHeightContractFollowsCurrentIncline(
      SuperExplorationDecider& decider) {
    decider.have_home_ = true;
    decider.home_pose_.pose.position.z = 1.5;
    decider.current_pose_.pose.position.z = 4.25;
    decider.min_observation_height_above_home_ = -0.3;
    decider.max_observation_height_above_home_ = 0.2;
    decider.cruise_height_above_home_ = 0.0;
    decider.tunnel_min_planning_height_ = -5.0;
    decider.tunnel_max_planning_height_ = 8.0;

    decider.mission_profile_ = ExplorationMissionProfile::kTunnel;
    const bool tunnel_ok =
        std::abs(decider.preferredMissionObservationHeight() - 4.25) < 1e-9 &&
        decider.missionObservationHeightAllowed(7.5) &&
        !decider.missionObservationHeightAllowed(8.1) &&
        std::abs(decider.maximumReturnHeight() - 8.0) < 1e-9;

    decider.mission_profile_ = ExplorationMissionProfile::kGoaf;
    const bool goaf_stays_level =
        std::abs(decider.preferredMissionObservationHeight() - 1.5) < 1e-9 &&
        decider.missionObservationHeightAllowed(1.65) &&
        !decider.missionObservationHeightAllowed(4.25);
    return tunnel_ok && goaf_stays_level;
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
    decider.voxel_resolution_ = 0.25;
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
    const bool outside_one_voxel = decider.have_active_goal_ &&
        decider.active_goal_id_ == 7;
    decider.current_pose_.pose.position.x = 0.8;
    decider.last_sync_time_ = ros::Time::now();
    decider.decisionTimerCallback(ros::TimerEvent());
    return outside_half_metre && outside_one_voxel &&
        decider.active_goal_id_ != 7;
  }

  static bool returnPreviewCentersBeforeOffsetNarrowing(
      SuperExplorationDecider& decider) {
    decider.mission_profile_ = ExplorationMissionProfile::kTunnel;
    decider.corridor_preview_enabled_ = true;
    decider.corridor_preview_min_slices_ = 6;
    decider.corridor_preview_min_reliable_center_band_width_ = 0.75;
    decider.super_occupied_stamp_ = ros::Time::now();
    decider.return_map_timeout_ = 100.0;
    decider.vehicle_radius_ = 0.1;
    decider.return_super_radius_ = 0.1;
    decider.return_clearance_margin_ = 0.0;
    geometry_msgs::Point start;
    start.x = 10.0;
    geometry_msgs::Point target;
    target.x = 8.5;

    // Returning along -X: the first two metres are broad, then the connected
    // safe-centre band contracts around +Y. The short return goal must begin
    // moving laterally before the vehicle reaches that throat.
    for (double x = 1.5; x <= 10.0; x += 0.25) {
      const bool narrow = x <= 7.0;
      const double min_y = narrow ? 0.5 : -2.0;
      const double max_y = narrow ? 1.5 : 2.0;
      for (double y = min_y; y <= max_y + 1e-9; y += 0.25) {
        decider.voxels_[decider.positionToKey(x, y, 0.0)] =
            SuperExplorationDecider::FREE;
      }
    }
    return decider.applyReturnCorridorPreview(start, &target) &&
        target.y >= 0.25 && target.y <= 1.0;
  }

  static bool forwardPreviewIsSlewLimitedAndHeldAcrossSparseFrames(
      SuperExplorationDecider& decider) {
    decider.mission_profile_ = ExplorationMissionProfile::kTunnel;
    decider.have_home_ = true;
    decider.corridor_preview_enabled_ = true;
    decider.corridor_preview_min_slices_ = 6;
    decider.corridor_preview_min_reliable_center_band_width_ = 0.75;
    decider.corridor_centering_release_frames_ = 3;
    decider.voxel_resolution_ = 0.25;
    decider.vehicle_radius_ = 0.1;
    decider.vertical_vehicle_clearance_below_ = 0.1;
    decider.vertical_vehicle_clearance_above_ = 0.1;
    decider.cruise_height_above_home_ = 0.0;
    decider.mission_heading_yaw_ = 0.0;
    decider.current_pose_.pose.position.x = 0.0;
    decider.current_pose_.pose.position.y = 0.0;
    SuperExplorationDecider::VoxelSet reachable;
    for (double x = 1.0; x <= 8.0 + 1e-9; x += 0.25) {
      const bool narrow = x >= 3.0;
      const double min_y = narrow ? 0.5 : -2.0;
      const double max_y = narrow ? 1.5 : 2.0;
      for (double y = min_y; y <= max_y + 1e-9; y += 0.25) {
        const auto key = decider.positionToKey(x, y, 0.0);
        decider.voxels_[key] = SuperExplorationDecider::FREE;
        reachable.insert(key);
      }
    }
    decider.updateCorridorPreviewPreference(reachable);
    if (!decider.corridor_preview_preference_valid_ ||
        decider.corridor_preview_preferred_lateral_ <= 0.0 ||
        decider.corridor_preview_preferred_lateral_ > 0.25 + 1e-9) {
      return false;
    }
    const double first = decider.corridor_preview_preferred_lateral_;
    SuperExplorationDecider::VoxelSet empty;
    decider.updateCorridorPreviewPreference(empty);
    decider.updateCorridorPreviewPreference(empty);
    if (!decider.corridor_preview_preference_valid_ ||
        std::abs(decider.corridor_preview_preferred_lateral_ - first) > 1e-9) {
      return false;
    }
    decider.updateCorridorPreviewPreference(empty);
    return !decider.corridor_preview_preference_valid_;
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

  static bool indexedClearanceMatchesExactVoxelScan(
      SuperExplorationDecider& decider) {
    decider.voxel_resolution_ = 0.25;
    decider.vehicle_radius_ = 0.8;
    decider.preferred_vehicle_radius_ = 1.1;
    decider.vertical_vehicle_clearance_below_ = 0.5;
    decider.vertical_vehicle_clearance_above_ = 0.5;
    for (const VoxelKey occupied : {
             VoxelKey{0, 0, 0}, VoxelKey{4, 0, 0},
             VoxelKey{-3, 2, 2}, VoxelKey{8, 1, -2},
             VoxelKey{20, 20, 20}}) {
      decider.voxels_[occupied] = SuperExplorationDecider::OCCUPIED;
    }
    const std::vector<VoxelKey> queries{
        {0, 0, 0}, {1, 0, 0}, {7, 1, 0}, {10, 1, 0},
        {-1, 2, 0}, {20, 20, 17}, {30, 30, 30}};
    std::vector<bool> exact;
    decider.decision_clearance_cache_active_ = false;
    for (const VoxelKey& query : queries) {
      exact.push_back(decider.isClearForVehicle(query, 0.8));
      exact.push_back(decider.isClearForVehicle(query, 1.1));
    }
    decider.decision_vehicle_clearance_cache_.clear();
    decider.decision_preferred_clearance_cache_.clear();
    decider.decision_clearance_cache_active_ = true;
    decider.rebuildDecisionClearanceIndex();
    std::size_t index = 0;
    for (const VoxelKey& query : queries) {
      if (decider.isClearForVehicle(query, 0.8) != exact[index++]) {
        return false;
      }
      if (decider.isClearForVehicle(query, 1.1) != exact[index++]) {
        return false;
      }
    }
    return true;
  }

  static double representativeIndexedReachabilityMilliseconds(
      SuperExplorationDecider& decider, std::size_t* reachable_count) {
    decider.voxels_.clear();
    decider.voxel_resolution_ = 0.25;
    decider.vehicle_radius_ = 0.8;
    decider.preferred_vehicle_radius_ = 1.1;
    decider.vertical_vehicle_clearance_below_ = 0.5;
    decider.vertical_vehicle_clearance_above_ = 0.5;
    decider.have_home_ = true;
    decider.current_pose_.pose.position.x = 0.125;
    decider.current_pose_.pose.position.y = 0.125;
    decider.current_pose_.pose.position.z = 0.125;
    decider.home_pose_ = decider.current_pose_;
    decider.mission_heading_yaw_ = 0.0;
    decider.frontier_search_radius_ = 100.0;
    decider.max_task_lateral_offset_ = 100.0;
    decider.mission_profile_ = ExplorationMissionProfile::kTunnel;
    decider.tunnel_min_planning_height_ = -100.0;
    decider.tunnel_max_planning_height_ = 100.0;
    decider.max_reachable_voxels_ = 300000;

    // Roughly the same number of known-free cells as the slowest decision in
    // the 2026-10-24 22:39 flight. Occupied side/roof/floor voxels exercise
    // spatial-index queries rather than the empty-map fast path.
    for (int x = 0; x < 200; ++x) {
      for (int y = -15; y <= 15; ++y) {
        for (int z = -12; z <= 12; ++z) {
          const bool boundary = y == -15 || y == 15 || z == -12 || z == 12;
          decider.voxels_[{x, y, z}] = boundary
              ? SuperExplorationDecider::OCCUPIED
              : SuperExplorationDecider::FREE;
        }
      }
    }
    decider.decision_vehicle_clearance_cache_.clear();
    decider.decision_preferred_clearance_cache_.clear();
    decider.decision_clearance_cache_active_ = true;
    const auto start = std::chrono::steady_clock::now();
    decider.rebuildDecisionClearanceIndex();
    const auto reachable = decider.reachableFreeVoxels();
    const auto finish = std::chrono::steady_clock::now();
    *reachable_count = reachable.size();
    return std::chrono::duration<double, std::milli>(finish - start).count();
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
    decider.mission_profile_ = ExplorationMissionProfile::kTunnel;
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
    decider.mission_profile_ = ExplorationMissionProfile::kTunnel;
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

  static bool forwardRouteKeepsReachedLaneWhenClearanceTiesAtStart(
      SuperExplorationDecider& decider) {
    decider.have_home_ = true;
    decider.mission_profile_ = ExplorationMissionProfile::kGoaf;
    decider.mission_heading_yaw_ = 0.0;
    decider.super_occupied_stamp_ = ros::Time::now();
    decider.voxel_resolution_ = 0.5;
    decider.vehicle_radius_ = 0.1;
    decider.preferred_vehicle_radius_ = 1.5;
    decider.return_super_radius_ = 0.1;
    decider.return_clearance_margin_ = 0.0;
    decider.forward_lookahead_distance_ = 2.0;
    decider.forward_lookahead_step_ = 0.5;
    decider.min_goal_distance_ = 2.0;
    decider.forward_goal_lateral_search_width_ = 4.0;
    decider.home_pose_.pose.position = decider.keyToPoint({0, 0, 0});
    decider.current_pose_.pose.position = decider.keyToPoint({0, 8, 0});
    decider.held_route_side_ = 1;
    SuperExplorationDecider::VoxelSet reachable;
    for (int x = 0; x <= 5; ++x) {
      for (int y = -8; y <= 10; ++y) {
        const VoxelKey key{x, y, 0};
        decider.voxels_[key] = SuperExplorationDecider::FREE;
        reachable.insert(key);
      }
    }
    // All safe paths share the same 1.1 m minimum at the current position.
    // The previous axis-centering tie break picked y=0 and crossed the lane.
    decider.super_occupied_points_->push_back(
        pcl::PointXYZ(0.25f, 5.35f, 0.25f));
    decider.super_occupied_tree_.setInputCloud(
        decider.super_occupied_points_);
    SuperExplorationDecider::LocalRouteCandidate selected;
    return decider.selectForwardLookaheadGoal(reachable, false, &selected) &&
           std::abs(selected.mission_lateral - 4.0) <= 0.5 + 1e-9 &&
           selected.minimum_clearance >= 1.0;
  }

  static bool goafReturnSearchesLiveMapWhenTraceChordIsBlocked(
      SuperExplorationDecider& decider) {
    const ros::Time now = ros::Time::now();
    decider.have_data_ = true;
    decider.have_home_ = true;
    decider.returning_home_ = true;
    decider.prefer_outbound_breadcrumbs_ = true;
    decider.mission_profile_ = ExplorationMissionProfile::kGoaf;
    decider.last_sync_time_ = now;
    decider.super_occupied_stamp_ = now;
    decider.data_timeout_ = 100.0;
    decider.return_map_timeout_ = 100.0;
    decider.return_segment_length_ = 3.0;
    decider.voxel_resolution_ = 0.5;
    decider.vehicle_radius_ = 1.0;
    decider.return_super_radius_ = 1.0;
    decider.return_clearance_margin_ = 0.0;
    decider.home_pose_.pose.position = decider.keyToPoint({0, 0, 0});
    decider.current_pose_.pose.position = decider.keyToPoint({8, 8, 0});
    for (int x = 0; x <= 10; ++x) {
      for (int y = 0; y <= 10; ++y) {
        decider.voxels_[{x, y, 0}] = SuperExplorationDecider::FREE;
      }
    }
    geometry_msgs::PoseStamped crumb = decider.home_pose_;
    decider.outbound_breadcrumbs_.push_back(crumb);
    crumb.pose.position = decider.keyToPoint({8, 0, 0});
    decider.outbound_breadcrumbs_.push_back(crumb);
    crumb.pose.position = decider.current_pose_.pose.position;
    decider.outbound_breadcrumbs_.push_back(crumb);
    decider.super_occupied_points_->push_back(
        pcl::PointXYZ(4.25f, 2.75f, 0.25f));
    decider.super_occupied_tree_.setInputCloud(
        decider.super_occupied_points_);
    const bool published = decider.publishFreshReturnGoal("goaf_map_detour");
    const auto& goal = decider.current_goal_.pose.position;
    return published && decider.have_active_goal_ &&
           goal.x < decider.current_pose_.pose.position.x - 0.5 &&
           std::hypot(goal.x - decider.home_pose_.pose.position.x,
                      goal.y - decider.home_pose_.pose.position.y) <
               std::hypot(decider.current_pose_.pose.position.x -
                              decider.home_pose_.pose.position.x,
                          decider.current_pose_.pose.position.y -
                              decider.home_pose_.pose.position.y);
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

  static bool lateralDetourDoesNotReverseOntoOutboundBreadcrumbs(
      SuperExplorationDecider& decider) {
    decider.have_home_ = true;
    decider.super_occupied_stamp_ = ros::Time::now();
    decider.mission_heading_yaw_ = 0.0;
    decider.voxel_resolution_ = 0.25;
    decider.vehicle_radius_ = 0.1;
    decider.preferred_vehicle_radius_ = 0.1;
    decider.return_super_radius_ = 0.1;
    decider.return_clearance_margin_ = 0.0;
    decider.connected_turn_min_new_space_ = 1.0;
    decider.current_pose_.pose.position.x = 8.0;
    decider.current_pose_.pose.position.y = -4.0;
    decider.front_obstacle_streak_ =
        decider.front_obstacle_confirm_frames_;

    for (double y = 0.0; y >= -4.0; y -= 0.5) {
      geometry_msgs::PoseStamped breadcrumb;
      breadcrumb.pose.position.x = 8.0;
      breadcrumb.pose.position.y = y;
      breadcrumb.pose.orientation.w = 1.0;
      decider.outbound_breadcrumbs_.push_back(breadcrumb);
    }

    SuperExplorationDecider::FrontierCandidate old_lane;
    old_lane.goal.pose.position.x = 8.0;
    old_lane.goal.pose.position.y = 0.0;
    old_lane.unknown_neighbors = 20;
    SuperExplorationDecider::VoxelSet reachable;
    for (double y = -4.0; y <= 0.0; y += 0.125) {
      const auto key = decider.positionToKey(7.75, y, 0.0);
      decider.voxels_[key] = SuperExplorationDecider::FREE;
      reachable.insert(key);
    }
    return !decider.publishLateralDetourGoal(reachable, {old_lane});
  }

  static bool connectedTurnFollowsKnownFreeBendWithoutFrontier(
      SuperExplorationDecider& decider) {
    decider.have_home_ = true;
    decider.super_occupied_stamp_ = ros::Time::now();
    decider.mission_heading_yaw_ = 0.0;
    decider.have_last_exploration_route_heading_ = true;
    decider.last_exploration_route_heading_yaw_ = -35.0 * M_PI / 180.0;
    decider.voxel_resolution_ = 0.25;
    decider.vehicle_radius_ = 0.1;
    decider.preferred_vehicle_radius_ = 0.1;
    decider.return_super_radius_ = 0.1;
    decider.return_clearance_margin_ = 0.0;
    decider.min_goal_distance_ = 2.0;
    decider.max_frontier_goal_distance_ = 4.0;
    decider.max_frontier_goal_vertical_step_ = 0.5;
    decider.max_task_lateral_offset_ = 12.0;
    decider.max_exploration_radius_from_home_ = 30.0;
    decider.exploration_goal_queue_horizon_ = 10.0;
    decider.exploration_goal_queue_spacing_ = 2.5;
    decider.exploration_goal_queue_end_reserve_ = 1.0;
    decider.connected_turn_max_angle_deg_ = 80.0;
    decider.current_pose_.pose.position = decider.keyToPoint({32, 0, 0});

    SuperExplorationDecider::VoxelSet reachable;
    for (int y = 0; y >= -40; --y) {
      const VoxelKey key{32, y, 0};
      decider.voxels_[key] = SuperExplorationDecider::FREE;
      reachable.insert(key);
    }
    if (!decider.publishConnectedTurnContinuation(reachable) ||
        !decider.have_active_goal_ ||
        !decider.active_goal_is_blocked_frontier_route_ ||
        !decider.active_goal_allows_rolling_handover_ ||
        decider.current_goal_.pose.position.y > -3.0 ||
        decider.exploration_goal_queue_.empty()) {
      return false;
    }
    const double yaw = 2.0 * std::atan2(
        decider.current_goal_.pose.orientation.z,
        decider.current_goal_.pose.orientation.w);
    return yaw < -1.5 && yaw > -1.7;
  }

  static bool connectedTurnRejectsTonightSubFloorStartClearance(
      SuperExplorationDecider& decider) {
    decider.have_home_ = true;
    decider.super_occupied_stamp_ = ros::Time::now();
    decider.mission_heading_yaw_ = 0.0;
    decider.have_last_exploration_route_heading_ = true;
    decider.last_exploration_route_heading_yaw_ = 0.0;
    decider.voxel_resolution_ = 0.25;
    decider.vehicle_radius_ = 0.8;
    decider.preferred_vehicle_radius_ = 1.1;
    decider.return_super_radius_ = 0.8;
    decider.return_clearance_margin_ = 0.0;
    decider.min_goal_distance_ = 1.0;
    decider.max_frontier_goal_distance_ = 4.0;
    decider.max_frontier_goal_vertical_step_ = 0.5;
    decider.max_task_lateral_offset_ = 12.0;
    decider.max_exploration_radius_from_home_ = 30.0;
    decider.exploration_goal_queue_horizon_ = 10.0;
    decider.connected_turn_max_angle_deg_ = 80.0;
    decider.current_pose_.pose.position = decider.keyToPoint({0, 0, 0});

    SuperExplorationDecider::VoxelSet reachable;
    // The nearest graph cell is safely beyond the wall point, but the real
    // aircraft starts only 0.70 m from it. Tonight's old route selection
    // ignored that current-to-graph entrance clearance and advertised the
    // resulting route as verified.
    for (int x = 2; x <= 20; ++x) {
      const VoxelKey key{x, 0, 0};
      reachable.insert(key);
      decider.voxels_[key] = SuperExplorationDecider::FREE;
    }
    const auto current = decider.current_pose_.pose.position;
    decider.super_occupied_points_->push_back(pcl::PointXYZ(
        static_cast<float>(current.x),
        static_cast<float>(current.y + 0.70),
        static_cast<float>(current.z)));
    decider.super_occupied_tree_.setInputCloud(decider.super_occupied_points_);
    return !decider.publishConnectedTurnContinuation(reachable);
  }

  static bool tunnelHeadingCommitsOnlyAfterSuperAcceptance(
      SuperExplorationDecider& decider) {
    decider.mission_profile_ = ExplorationMissionProfile::kTunnel;
    decider.mission_heading_yaw_ = 0.0;
    decider.have_last_exploration_route_heading_ = true;
    decider.last_exploration_route_heading_yaw_ = 0.0;
    decider.current_pose_.pose.position.x = 1.0;

    geometry_msgs::PoseStamped bend;
    bend.pose.position.x = 1.0;
    bend.pose.position.y = -3.0;
    bend.pose.orientation.w = 1.0;
    decider.publishGoal(bend, "frontier_fallback");
    if (!decider.active_goal_commits_tunnel_heading_ ||
        decider.tunnel_route_locked_) {
      return false;
    }

    std_msgs::UInt64 rejected;
    rejected.data = decider.active_goal_id_;
    decider.rejectedGoalCallback(
        boost::make_shared<std_msgs::UInt64>(rejected));
    if (decider.tunnel_route_locked_) return false;

    decider.publishGoal(bend, "frontier_fallback");
    std_msgs::UInt64 accepted;
    accepted.data = decider.active_goal_id_;
    decider.acceptedGoalCallback(
        boost::make_shared<std_msgs::UInt64>(accepted));
    return decider.tunnel_route_locked_ &&
        std::abs(decider.tunnel_route_heading_yaw_ + 0.5 * M_PI) < 1e-9;
  }

  static bool lockedTunnelHandoverCannotReturnToEntryAxis(
      SuperExplorationDecider& decider) {
    decider.have_home_ = true;
    decider.mission_profile_ = ExplorationMissionProfile::kTunnel;
    decider.mission_heading_yaw_ = 0.0;
    decider.tunnel_route_locked_ = true;
    decider.tunnel_route_heading_yaw_ = -0.5 * M_PI;
    decider.voxel_resolution_ = 0.5;
    decider.vehicle_radius_ = 0.1;
    decider.preferred_vehicle_radius_ = 0.1;
    decider.return_super_radius_ = 0.1;
    decider.return_clearance_margin_ = 0.0;
    decider.min_goal_distance_ = 1.0;
    decider.max_frontier_goal_distance_ = 3.0;
    decider.connected_turn_max_angle_deg_ = 80.0;
    decider.current_pose_.pose.position = decider.keyToPoint({8, 0, 0});
    decider.current_goal_.pose.position = decider.keyToPoint({8, -2, 0});
    decider.have_active_goal_ = true;
    decider.active_goal_id_ = 12;
    decider.accepted_goal_id_ = 12;

    SuperExplorationDecider::VoxelSet reachable;
    for (int x = 8; x <= 16; ++x) {
      const VoxelKey key{x, 0, 0};
      decider.voxels_[key] = SuperExplorationDecider::FREE;
      reachable.insert(key);
    }
    return !decider.tryForwardHandover(reachable) &&
        decider.active_goal_id_ == 12;
  }

  static bool lockedTunnelRouteCanContinueBeyondEntryAxisBand(
      SuperExplorationDecider& decider) {
    decider.have_home_ = true;
    decider.super_occupied_stamp_ = ros::Time::now();
    decider.mission_profile_ = ExplorationMissionProfile::kTunnel;
    decider.mission_heading_yaw_ = 0.0;
    decider.tunnel_route_locked_ = true;
    decider.tunnel_route_heading_yaw_ = -0.5 * M_PI;
    decider.voxel_resolution_ = 0.25;
    decider.vehicle_radius_ = 0.1;
    decider.preferred_vehicle_radius_ = 0.1;
    decider.return_super_radius_ = 0.1;
    decider.return_clearance_margin_ = 0.0;
    decider.min_goal_distance_ = 2.0;
    decider.max_frontier_goal_distance_ = 4.0;
    decider.max_frontier_goal_vertical_step_ = 0.5;
    decider.max_task_lateral_offset_ = 2.0;
    decider.max_exploration_radius_from_home_ = 30.0;
    decider.exploration_goal_queue_horizon_ = 10.0;
    decider.connected_turn_max_angle_deg_ = 80.0;
    decider.current_pose_.pose.position = decider.keyToPoint({32, -16, 0});

    SuperExplorationDecider::VoxelSet reachable;
    for (int y = -16; y >= -40; --y) {
      const VoxelKey key{32, y, 0};
      decider.voxels_[key] = SuperExplorationDecider::FREE;
      reachable.insert(key);
    }
    return decider.publishConnectedTurnContinuation(reachable) &&
        decider.current_goal_.pose.position.y < -6.0;
  }

  static bool lockedTunnelAllowsSharpBranchWhenCurrentRouteEnds(
      SuperExplorationDecider& decider) {
    decider.have_home_ = true;
    decider.super_occupied_stamp_ = ros::Time::now();
    decider.mission_profile_ = ExplorationMissionProfile::kTunnel;
    decider.mission_heading_yaw_ = 0.0;
    decider.tunnel_route_locked_ = true;
    decider.tunnel_route_heading_yaw_ = 0.0;
    decider.tunnel_connected_turn_max_angle_deg_ = 120.0;
    decider.tunnel_straight_preference_angle_deg_ = 35.0;
    decider.voxel_resolution_ = 0.25;
    decider.vehicle_radius_ = 0.1;
    decider.preferred_vehicle_radius_ = 0.1;
    decider.return_super_radius_ = 0.1;
    decider.return_clearance_margin_ = 0.0;
    decider.min_goal_distance_ = 2.0;
    decider.max_frontier_goal_distance_ = 4.0;
    decider.max_frontier_goal_vertical_step_ = 0.5;
    decider.max_exploration_radius_from_home_ = 30.0;
    decider.exploration_goal_queue_horizon_ = 10.0;
    decider.current_pose_.pose.position = decider.keyToPoint({32, 0, 0});

    SuperExplorationDecider::VoxelSet reachable;
    reachable.insert({32, 0, 0});
    decider.voxels_[{32, 0, 0}] = SuperExplorationDecider::FREE;
    // The current +X passage is closed. The only connected continuation first
    // steps left, then turns down: its local endpoint is about 100 degrees
    // from the accepted tangent and must remain explorable, not be mistaken
    // for a terminal branch.
    reachable.insert({31, 0, 0});
    decider.voxels_[{31, 0, 0}] = SuperExplorationDecider::FREE;
    for (int y = -1; y >= -24; --y) {
      reachable.insert({31, y, 0});
      decider.voxels_[{31, y, 0}] = SuperExplorationDecider::FREE;
    }
    if (!decider.publishConnectedTurnContinuation(reachable)) return false;
    const double yaw = std::atan2(
        decider.current_goal_.pose.position.y -
            decider.current_pose_.pose.position.y,
        decider.current_goal_.pose.position.x -
            decider.current_pose_.pose.position.x);
    return std::abs(yaw) > 0.5 * M_PI &&
        std::abs(yaw) < 120.0 * M_PI / 180.0 + 1e-9;
  }

  static bool lockedTunnelPrefersCurrentRouteOverAvailableSideBranch(
      SuperExplorationDecider& decider) {
    decider.have_home_ = true;
    decider.super_occupied_stamp_ = ros::Time::now();
    decider.mission_profile_ = ExplorationMissionProfile::kTunnel;
    decider.mission_heading_yaw_ = 0.0;
    decider.tunnel_route_locked_ = true;
    decider.tunnel_route_heading_yaw_ = 0.0;
    decider.tunnel_connected_turn_max_angle_deg_ = 120.0;
    decider.tunnel_straight_preference_angle_deg_ = 35.0;
    decider.voxel_resolution_ = 0.25;
    decider.vehicle_radius_ = 0.1;
    decider.preferred_vehicle_radius_ = 0.1;
    decider.return_super_radius_ = 0.1;
    decider.return_clearance_margin_ = 0.0;
    decider.min_goal_distance_ = 2.0;
    decider.max_frontier_goal_distance_ = 4.0;
    decider.max_frontier_goal_vertical_step_ = 0.5;
    decider.max_exploration_radius_from_home_ = 30.0;
    decider.exploration_goal_queue_horizon_ = 10.0;
    decider.current_pose_.pose.position = decider.keyToPoint({32, 0, 0});

    SuperExplorationDecider::VoxelSet reachable;
    for (int x = 32; x <= 48; ++x) {
      reachable.insert({x, 0, 0});
      decider.voxels_[{x, 0, 0}] = SuperExplorationDecider::FREE;
    }
    // A longer side branch is already visible at the same junction. The
    // first-version tunnel policy must keep the accepted +X route while that
    // route remains executable, rather than chasing the larger side region.
    for (int y = -1; y >= -24; --y) {
      reachable.insert({32, y, 0});
      decider.voxels_[{32, y, 0}] = SuperExplorationDecider::FREE;
    }
    if (!decider.publishConnectedTurnContinuation(reachable)) return false;
    const double dx = decider.current_goal_.pose.position.x -
                      decider.current_pose_.pose.position.x;
    const double dy = decider.current_goal_.pose.position.y -
                      decider.current_pose_.pose.position.y;
    return dx > 2.0 && std::abs(std::atan2(dy, dx)) < 35.0 * M_PI / 180.0;
  }

  static bool tunnelBendChoosesClearLeftRouteOverMisleadingRightChord(
      SuperExplorationDecider& decider) {
    decider.have_home_ = true;
    decider.super_occupied_stamp_ = ros::Time::now();
    decider.mission_profile_ = ExplorationMissionProfile::kTunnel;
    decider.mission_heading_yaw_ = 0.0;
    decider.have_last_exploration_route_heading_ = true;
    decider.last_exploration_route_heading_yaw_ = 0.0;
    decider.tunnel_route_locked_ = false;
    decider.tunnel_connected_turn_max_angle_deg_ = 120.0;
    decider.tunnel_straight_preference_angle_deg_ = 35.0;
    decider.lateral_route_switch_clearance_advantage_ = 0.25;
    decider.voxel_resolution_ = 0.25;
    decider.vehicle_radius_ = 0.1;
    decider.preferred_vehicle_radius_ = 0.1;
    decider.return_super_radius_ = 0.1;
    decider.return_clearance_margin_ = 0.0;
    decider.min_goal_distance_ = 2.0;
    decider.max_frontier_goal_distance_ = 4.0;
    decider.max_frontier_goal_vertical_step_ = 0.5;
    decider.max_task_lateral_offset_ = 12.0;
    decider.max_exploration_radius_from_home_ = 30.0;
    decider.exploration_goal_queue_horizon_ = 10.0;
    decider.current_pose_.pose.position = decider.keyToPoint({40, 0, 0});

    SuperExplorationDecider::VoxelSet reachable;
    reachable.insert({40, 0, 0});
    decider.voxels_[{40, 0, 0}] = SuperExplorationDecider::FREE;

    // The right route first advances 2.5 m, making its four-metre local-goal
    // chord appear to be a mild 31-degree continuation, then turns 90 degrees
    // toward a nearby obstacle surface.
    for (int x = 41; x <= 50; ++x) {
      reachable.insert({x, 0, 0});
      decider.voxels_[{x, 0, 0}] = SuperExplorationDecider::FREE;
    }
    for (int y = -1; y >= -28; --y) {
      reachable.insert({50, y, 0});
      decider.voxels_[{50, y, 0}] = SuperExplorationDecider::FREE;
    }

    // The left branch turns immediately. Its chord looks like a sharp branch,
    // but the complete connected route has substantially more clearance.
    reachable.insert({39, 0, 0});
    decider.voxels_[{39, 0, 0}] = SuperExplorationDecider::FREE;
    for (int y = 1; y <= 28; ++y) {
      reachable.insert({39, y, 0});
      decider.voxels_[{39, y, 0}] = SuperExplorationDecider::FREE;
    }

    const geometry_msgs::Point right_corner = decider.keyToPoint({50, 0, 0});
    for (int y = -24; y <= 0; ++y) {
      const geometry_msgs::Point route_point = decider.keyToPoint({50, y, 0});
      decider.super_occupied_points_->push_back(pcl::PointXYZ(
          static_cast<float>(right_corner.x + 0.6),
          static_cast<float>(route_point.y),
          static_cast<float>(route_point.z)));
    }
    decider.super_occupied_tree_.setInputCloud(decider.super_occupied_points_);

    if (decider.publishConnectedTurnContinuation(reachable) ||
        decider.pending_connected_turn_streak_ != 1 ||
        decider.publishConnectedTurnContinuation(reachable) ||
        decider.pending_connected_turn_streak_ != 2 ||
        !decider.publishConnectedTurnContinuation(reachable) ||
        decider.current_goal_.pose.position.y <= 2.0 ||
        !decider.active_goal_commits_tunnel_heading_) {
      return false;
    }
    const double commanded_yaw = 2.0 * std::atan2(
        decider.current_goal_.pose.orientation.z,
        decider.current_goal_.pose.orientation.w);
    return commanded_yaw > 1.45 && commanded_yaw < 1.70 &&
        decider.active_goal_tunnel_heading_yaw_ > 1.45 &&
        decider.active_goal_tunnel_heading_yaw_ < 1.70;
  }

  static bool tunnelBendHandsOverBeforeTheStraightGoalStops(
      SuperExplorationDecider& decider) {
    decider.have_home_ = true;
    decider.mission_profile_ = ExplorationMissionProfile::kTunnel;
    decider.super_occupied_stamp_ = ros::Time::now();
    decider.mission_heading_yaw_ = 0.0;
    decider.have_last_exploration_route_heading_ = true;
    decider.last_exploration_route_heading_yaw_ = 0.0;
    decider.tunnel_route_locked_ = false;
    decider.voxel_resolution_ = 0.25;
    decider.vehicle_radius_ = 0.1;
    decider.preferred_vehicle_radius_ = 0.1;
    decider.return_super_radius_ = 0.1;
    decider.return_clearance_margin_ = 0.0;
    decider.forward_lookahead_distance_ = 4.0;
    decider.forward_lookahead_step_ = 0.5;
    decider.forward_goal_lateral_search_width_ = 0.0;
    decider.min_goal_distance_ = 2.0;
    decider.max_frontier_goal_distance_ = 4.0;
    decider.max_frontier_goal_vertical_step_ = 0.5;
    decider.max_task_lateral_offset_ = 12.0;
    decider.max_exploration_radius_from_home_ = 30.0;
    decider.exploration_goal_queue_horizon_ = 10.0;
    decider.tunnel_connected_turn_max_angle_deg_ = 120.0;
    decider.tunnel_straight_preference_angle_deg_ = 35.0;
    decider.current_pose_.pose.position = decider.keyToPoint({32, 0, 0});
    decider.current_goal_.pose.position = decider.keyToPoint({40, 0, 0});
    decider.current_goal_.pose.orientation.w = 1.0;
    decider.have_active_goal_ = true;
    decider.active_goal_id_ = 50;
    decider.accepted_goal_id_ = 50;

    SuperExplorationDecider::VoxelSet reachable;
    for (int x = 32; x <= 34; ++x) {
      reachable.insert({x, 0, 0});
      decider.voxels_[{x, 0, 0}] = SuperExplorationDecider::FREE;
    }
    for (int y = -1; y >= -24; --y) {
      reachable.insert({34, y, 0});
      decider.voxels_[{34, y, 0}] = SuperExplorationDecider::FREE;
    }

    if (decider.tryForwardHandover(reachable) ||
        decider.active_goal_id_ != 50 ||
        decider.tryForwardHandover(reachable) ||
        decider.active_goal_id_ != 50 ||
        !decider.tryForwardHandover(reachable) ||
        decider.active_goal_id_ == 50 ||
        decider.current_goal_.pose.position.y >= -2.0) {
      return false;
    }
    const double yaw = 2.0 * std::atan2(
        decider.current_goal_.pose.orientation.z,
        decider.current_goal_.pose.orientation.w);
    return yaw < -1.45 && yaw > -1.70;
  }

  static bool strictTunnelReturnWaitsInsteadOfTakingSideBranch(
      SuperExplorationDecider& decider) {
    const ros::Time now = ros::Time::now();
    decider.have_data_ = true;
    decider.have_home_ = true;
    decider.returning_home_ = true;
    decider.prefer_outbound_breadcrumbs_ = true;
    decider.mission_profile_ = ExplorationMissionProfile::kTunnel;
    decider.last_sync_time_ = now;
    decider.super_occupied_stamp_ = now;
    decider.data_timeout_ = 100.0;
    decider.return_map_timeout_ = 100.0;
    decider.return_segment_length_ = 3.0;
    decider.goal_reached_distance_ = 0.25;
    decider.voxel_resolution_ = 0.5;
    decider.vehicle_radius_ = 0.1;
    decider.return_super_radius_ = 0.1;
    decider.return_clearance_margin_ = 0.0;
    decider.home_pose_.pose.position = decider.keyToPoint({0, 0, 0});
    decider.current_pose_.pose.position = decider.keyToPoint({8, 8, 0});
    for (int x = 0; x <= 8; ++x) {
      for (int y = 0; y <= 8; ++y) {
        decider.voxels_[{x, y, 0}] = SuperExplorationDecider::FREE;
      }
    }
    geometry_msgs::PoseStamped crumb = decider.home_pose_;
    decider.outbound_breadcrumbs_.push_back(crumb);
    crumb.pose.position = decider.keyToPoint({8, 0, 0});
    decider.outbound_breadcrumbs_.push_back(crumb);
    crumb.pose.position = decider.current_pose_.pose.position;
    decider.outbound_breadcrumbs_.push_back(crumb);

    // Block the recorded inward leg immediately behind the aircraft. The
    // surrounding square remains known free, so the general graph could take
    // an unvisited side route; strict Task-4 return must hold instead.
    decider.super_occupied_points_->push_back(pcl::PointXYZ(
        static_cast<float>(decider.current_pose_.pose.position.x),
        static_cast<float>(decider.current_pose_.pose.position.y - 0.5),
        static_cast<float>(decider.current_pose_.pose.position.z)));
    decider.super_occupied_tree_.setInputCloud(
        decider.super_occupied_points_);
    return !decider.publishFreshReturnGoal("strict_trace_test") &&
        !decider.have_active_goal_;
  }

  static bool lockedTunnelBranchReturnsAfterCoherentRouteTimeout(
      SuperExplorationDecider& decider) {
    const ros::Time now = ros::Time::now();
    decider.enabled_ = true;
    decider.have_data_ = true;
    decider.have_home_ = true;
    decider.exploration_started_ = true;
    decider.mission_profile_ = ExplorationMissionProfile::kTunnel;
    decider.tunnel_route_locked_ = true;
    decider.tunnel_route_heading_yaw_ = -0.5 * M_PI;
    decider.last_sync_time_ = now;
    decider.super_occupied_stamp_ = now;
    decider.data_timeout_ = 100.0;
    decider.first_data_time_ = now - ros::Duration(20.0);
    decider.min_data_duration_ = 1.0;
    decider.reached_goal_count_ = 2;
    decider.min_goals_before_complete_ = 1;
    decider.no_frontier_timeout_ = 2.0;
    decider.hard_safe_route_unavailable_since_ =
        now - ros::Duration(3.0);
    decider.voxel_resolution_ = 0.5;
    decider.vehicle_radius_ = 0.1;
    decider.return_super_radius_ = 0.1;
    decider.return_clearance_margin_ = 0.0;
    decider.current_pose_.pose.position = decider.keyToPoint({4, -4, 0});
    decider.home_pose_.pose.position = decider.keyToPoint({0, 0, 0});
    const VoxelKey current = decider.positionToKey(
        decider.current_pose_.pose.position.x,
        decider.current_pose_.pose.position.y,
        decider.current_pose_.pose.position.z);
    decider.voxels_[current] = SuperExplorationDecider::FREE;

    decider.decisionTimerCallback(ros::TimerEvent());
    return decider.returning_home_;
  }

  static bool noRouteTimerClearsOnlyAfterSuperAcceptance(
      SuperExplorationDecider& decider) {
    decider.enabled_ = true;
    decider.current_pose_.pose.position.x = 1.0;
    decider.hard_safe_route_unavailable_since_ =
        ros::Time::now() - ros::Duration(5.0);
    const ros::Time original_timer =
        decider.hard_safe_route_unavailable_since_;

    geometry_msgs::PoseStamped goal;
    goal.pose.position.x = 3.0;
    goal.pose.orientation.w = 1.0;
    decider.publishGoal(goal, "frontier_fallback");
    if (decider.hard_safe_route_unavailable_since_ != original_timer) {
      return false;
    }

    std_msgs::UInt64::Ptr accepted(new std_msgs::UInt64());
    accepted->data = decider.active_goal_id_;
    decider.acceptedGoalCallback(accepted);
    return decider.hard_safe_route_unavailable_since_.isZero();
  }

  static bool decisionSnapshotDoesNotExpireDuringItsOwnPass(
      SuperExplorationDecider& decider) {
    decider.return_map_timeout_ = 1.0;
    decider.voxel_resolution_ = 0.5;
    decider.vehicle_radius_ = 0.1;
    decider.return_super_radius_ = 0.1;
    decider.return_clearance_margin_ = 0.0;
    geometry_msgs::Point point;
    point.x = 0.25;
    point.y = 0.25;
    point.z = 0.25;
    decider.voxels_[decider.positionToKey(point.x, point.y, point.z)] =
        SuperExplorationDecider::FREE;
    decider.super_occupied_stamp_ = ros::Time::now() - ros::Duration(5.0);
    decider.decision_snapshot_time_ =
        decider.super_occupied_stamp_ + ros::Duration(0.5);
    const bool snapshot_clear = decider.returnSegmentClear(
        point, point, 0.0, false);
    decider.decision_snapshot_time_ = ros::Time();
    const bool wall_clock_stale = !decider.returnSegmentClear(
        point, point, 0.0, false);
    return snapshot_clear && wall_clock_stale;
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
    decider.hard_safe_route_unavailable_since_ =
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

  static bool persistentObstacleReplanInvalidatesCurrentEndpoint(
      SuperExplorationDecider& decider) {
    decider.enabled_ = true;
    decider.have_home_ = true;
    decider.have_active_goal_ = true;
    decider.active_goal_id_ = 17;
    decider.current_goal_.pose.position.x = 4.0;
    decider.current_goal_.pose.position.y = 1.0;
    const std::size_t failed_before = decider.failed_goals_.size();
    decider.obstacleReplanCallback(boost::make_shared<std_msgs::Empty>());
    return !decider.have_active_goal_ && decider.active_goal_id_ == 0 &&
           decider.failed_goals_.size() == failed_before + 1 &&
           decider.rejected_goal_retry_after_.isZero();
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

TEST(SuperGoalPublication,
     SchedulerSelectsOnlyTheProfileForTheActiveExplorationTask) {
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~scheduled_profile_selection");
  SuperExplorationDecider decider(nh, private_nh);
  EXPECT_TRUE(SuperExplorationDeciderTestPeer::
                  schedulerSelectsOnlyTheProfileForTheActiveExplorationTask(
                      decider));
}

TEST(SuperGoalPublication, GoafUsesLongerPreviewWithoutRetuningTunnel) {
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~goaf_long_preview");
  SuperExplorationDecider decider(nh, private_nh);
  EXPECT_TRUE(SuperExplorationDeciderTestPeer::
                  goafUsesLongerPreviewWithoutRetuningTunnel(decider));
}

TEST(SuperGoalPublication,
     BatteryAdmissionRequiresOutboundAndReturnReserve) {
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~battery_admission");
  SuperExplorationDecider decider(nh, private_nh);
  EXPECT_TRUE(SuperExplorationDeciderTestPeer::
                  batteryAdmissionRequiresOutboundAndReturnReserve(decider));
}

TEST(SuperGoalPublication,
     DisabledBatteryAdmissionIgnoresUnreliablePercentage) {
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~battery_admission_disabled");
  SuperExplorationDecider decider(nh, private_nh);
  EXPECT_TRUE(SuperExplorationDeciderTestPeer::
                  disabledBatteryAdmissionIgnoresUnreliablePercentage(
                      decider));
}

TEST(SuperGoalPublication,
     VoltageOnlyBatteryAdmissionIgnoresUnreliablePercentage) {
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~battery_admission_voltage_only");
  SuperExplorationDecider decider(nh, private_nh);
  EXPECT_TRUE(SuperExplorationDeciderTestPeer::
                  voltageOnlyBatteryAdmissionIgnoresUnreliablePercentage(
                      decider));
}

TEST(SuperGoalPublication, Task1ReturnsAtFiveMetresFromConfirmedEndWall) {
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~task1_five_metre_end_wall_return");
  SuperExplorationDecider decider(nh, private_nh);
  EXPECT_TRUE(SuperExplorationDeciderTestPeer::
                  task1ReturnsAtFiveMetresFromConfirmedEndWall(decider));
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

TEST(SuperGoalPublication, TunnelReturnRetracesInclinedBreadcrumbs) {
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~inclined_tunnel_return");
  SuperExplorationDecider decider(nh, private_nh);
  EXPECT_TRUE(SuperExplorationDeciderTestPeer::
                  tunnelReturnRetracesInclinedBreadcrumbs(decider));
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

TEST(SuperGoalPublication, ZeroMissionRadiusAllowsDistantForwardPassage) {
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~unlimited_mission_radius");
  SuperExplorationDecider decider(nh, private_nh);
  EXPECT_TRUE(SuperExplorationDeciderTestPeer::
                  zeroMissionRadiusAllowsDistantForwardPassage(decider));
}

TEST(SuperGoalPublication, TunnelHeightContractFollowsCurrentIncline) {
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~inclined_tunnel_height_contract");
  SuperExplorationDecider decider(nh, private_nh);
  EXPECT_TRUE(SuperExplorationDeciderTestPeer::
                  tunnelHeightContractFollowsCurrentIncline(decider));
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

TEST(SuperGoalPublication, ReturnPreviewCentersBeforeOffsetNarrowing) {
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~return_corridor_preview");
  SuperExplorationDecider decider(nh, private_nh);
  EXPECT_TRUE(SuperExplorationDeciderTestPeer::
                  returnPreviewCentersBeforeOffsetNarrowing(decider));
}

TEST(SuperGoalPublication, ForwardPreviewIsSlewLimitedAndHeldAcrossSparseFrames) {
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~forward_corridor_preview_latch");
  SuperExplorationDecider decider(nh, private_nh);
  EXPECT_TRUE(SuperExplorationDeciderTestPeer::
                  forwardPreviewIsSlewLimitedAndHeldAcrossSparseFrames(decider));
}

TEST(SuperGoalPublication, ConfirmedObstacleStillAllowsVerifiedHandover) {
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  SuperExplorationDecider decider(nh, private_nh);
  EXPECT_TRUE(SuperExplorationDeciderTestPeer::
                  acceptedShortForwardGoalHandsOver(decider));
}

TEST(SuperGoalPublication, RollingForwardQueueHandsOverOneGoalAtATime) {
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~rolling_forward_queue");
  SuperExplorationDecider decider(nh, private_nh);
  EXPECT_TRUE(SuperExplorationDeciderTestPeer::
                  rollingForwardQueueHandsOverOneGoalAtATime(decider));
}

TEST(SuperGoalPublication, RollingForwardQueueDropsInvalidTail) {
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~rolling_queue_invalidation");
  SuperExplorationDecider decider(nh, private_nh);
  EXPECT_TRUE(SuperExplorationDeciderTestPeer::
                  rollingForwardQueueDropsInvalidTail(decider));
}

TEST(SuperGoalPublication, CancellingExplorationClearsRollingQueue) {
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~rolling_queue_cancel");
  SuperExplorationDecider decider(nh, private_nh);
  EXPECT_TRUE(SuperExplorationDeciderTestPeer::
                  cancellingExplorationClearsRollingQueue(decider));
}

TEST(SuperGoalPublication, FrontierRouteCachesConnectedSuccessors) {
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~frontier_route_queue");
  SuperExplorationDecider decider(nh, private_nh);
  EXPECT_TRUE(SuperExplorationDeciderTestPeer::
                  frontierRouteCachesConnectedSuccessors(decider));
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

TEST(SuperGoalPublication,
     AcceptedReturnGoalPassedWhilePlanningNeverCommandsTurnAway) {
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  SuperExplorationDecider decider(nh, private_nh);
  EXPECT_TRUE(SuperExplorationDeciderTestPeer::
                  acceptedReturnGoalPassedWhilePlanningNeverCommandsTurnAway(
                      decider));
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

TEST(SuperGoalPublication, LateralDetourCannotReverseOntoSurveyedLane) {
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~lateral_no_outbound_backtrack");
  SuperExplorationDecider decider(nh, private_nh);
  EXPECT_TRUE(SuperExplorationDeciderTestPeer::
                  lateralDetourDoesNotReverseOntoOutboundBreadcrumbs(decider));
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

TEST(SuperGoalPublication, IndexedClearanceMatchesExactVoxelScan) {
  ros::NodeHandle nh, pnh("~");
  SuperExplorationDecider decider(nh, pnh);
  EXPECT_TRUE(SuperExplorationDeciderTestPeer::
                  indexedClearanceMatchesExactVoxelScan(decider));
}

TEST(SuperGoalPublication, RepresentativeReachabilityStaysBelowCallbackBudget) {
  ros::NodeHandle nh, pnh("~");
  SuperExplorationDecider decider(nh, pnh);
  std::size_t reachable_count = 0;
  const double elapsed_ms = SuperExplorationDeciderTestPeer::
      representativeIndexedReachabilityMilliseconds(
          decider, &reachable_count);
  EXPECT_GT(reachable_count, 80000U);
  // The former three-second budget was already longer than a 0.9 m/s
  // aircraft's three-metre rolling handover window.  Keep enough margin for
  // an older NUC under load while making callback regressions fail before
  // they reintroduce rest-to-rest waypoint transitions.
  EXPECT_LT(elapsed_ms, 500.0)
      << "representative flood fill took " << elapsed_ms << " ms for "
      << reachable_count << " cells";
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
      passable, 1.0, 0.5, -2.0, 0.25, 0.0, 1.0, 6, 0.75);
  ASSERT_TRUE(preview.valid);
  EXPECT_EQ(preview.slices, 12u);
  EXPECT_GT(preview.target_lateral, 0.5);
  EXPECT_GT(preview.near_center_band_width -
                preview.minimum_center_band_width,
            0.5);
}

TEST(SuperGoalPublication, CorridorPreviewDoesNotSteerFromSingleCellTail) {
  std::vector<std::vector<bool>> passable(
      12, std::vector<bool>(17, false));
  for (std::size_t row = 0; row < passable.size(); ++row) {
    if (row < 8) {
      for (std::size_t column = 1; column <= 15; ++column) {
        passable[row][column] = true;
      }
    } else {
      // The distant sensor horizon sees one safe centre cell at +0.5 m. It
      // must not masquerade as a 0.25 m physical passage and pull the goal.
      passable[row][10] = true;
    }
  }
  const CorridorPreviewEstimate preview = estimateCorridorPreview(
      passable, 1.0, 0.5, -2.0, 0.25, 0.0, 1.0, 6, 0.75);
  ASSERT_TRUE(preview.valid);
  EXPECT_EQ(preview.slices, 8u);
  EXPECT_NEAR(preview.target_lateral, 0.0, 0.25);
  EXPECT_GE(preview.minimum_center_band_width, 0.75);
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

TEST(SuperGoalPublication, ForwardRouteKeepsReachedLaneOnClearanceTie) {
  ros::NodeHandle nh, pnh("~reached_lane_clearance_tie");
  SuperExplorationDecider decider(nh, pnh);
  EXPECT_TRUE(SuperExplorationDeciderTestPeer::
                  forwardRouteKeepsReachedLaneWhenClearanceTiesAtStart(
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

TEST(SuperGoalPublication, ConnectedTurnFollowsKnownFreeBendWithoutFrontier) {
  ros::NodeHandle nh, pnh("~connected_turn_without_frontier");
  SuperExplorationDecider decider(nh, pnh);
  EXPECT_TRUE(SuperExplorationDeciderTestPeer::
                  connectedTurnFollowsKnownFreeBendWithoutFrontier(decider));
}

TEST(SuperGoalPublication, ConnectedTurnRejectsSubFloorStartClearance) {
  ros::NodeHandle nh, pnh("~connected_turn_start_clearance");
  SuperExplorationDecider decider(nh, pnh);
  EXPECT_TRUE(SuperExplorationDeciderTestPeer::
                  connectedTurnRejectsTonightSubFloorStartClearance(decider));
}

TEST(SuperGoalPublication, TunnelHeadingCommitsOnlyAfterSuperAcceptance) {
  ros::NodeHandle nh, pnh("~tunnel_heading_acceptance");
  SuperExplorationDecider decider(nh, pnh);
  EXPECT_TRUE(SuperExplorationDeciderTestPeer::
                  tunnelHeadingCommitsOnlyAfterSuperAcceptance(decider));
}

TEST(SuperGoalPublication, LockedTunnelHandoverCannotReturnToEntryAxis) {
  ros::NodeHandle nh, pnh("~tunnel_entry_axis_guard");
  SuperExplorationDecider decider(nh, pnh);
  EXPECT_TRUE(SuperExplorationDeciderTestPeer::
                  lockedTunnelHandoverCannotReturnToEntryAxis(decider));
}

TEST(SuperGoalPublication, LockedTunnelRouteCanContinueBeyondEntryAxisBand) {
  ros::NodeHandle nh, pnh("~tunnel_local_axis");
  SuperExplorationDecider decider(nh, pnh);
  EXPECT_TRUE(SuperExplorationDeciderTestPeer::
                  lockedTunnelRouteCanContinueBeyondEntryAxisBand(decider));
}

TEST(SuperGoalPublication, LockedTunnelAllowsSharpBranchWhenRouteEnds) {
  ros::NodeHandle nh, pnh("~tunnel_sharp_branch");
  SuperExplorationDecider decider(nh, pnh);
  EXPECT_TRUE(SuperExplorationDeciderTestPeer::
                  lockedTunnelAllowsSharpBranchWhenCurrentRouteEnds(decider));
}

TEST(SuperGoalPublication, LockedTunnelPrefersCurrentRouteOverSideBranch) {
  ros::NodeHandle nh;
  SuperExplorationDecider decider(
      nh, ros::NodeHandle("~locked_tunnel_straight_preference"));
  EXPECT_TRUE(SuperExplorationDeciderTestPeer::
                  lockedTunnelPrefersCurrentRouteOverAvailableSideBranch(
                      decider));
}

TEST(SuperGoalPublication,
     TunnelBendChoosesClearLeftRouteOverMisleadingRightChord) {
  ros::NodeHandle nh;
  SuperExplorationDecider decider(
      nh, ros::NodeHandle("~tunnel_bend_whole_route_clearance"));
  EXPECT_TRUE(SuperExplorationDeciderTestPeer::
                  tunnelBendChoosesClearLeftRouteOverMisleadingRightChord(
                      decider));
}

TEST(SuperGoalPublication, TunnelBendUsesContinuousPreStopHandover) {
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~tunnel_bend_continuous_handover");
  private_nh.setParam(
      "goal_command_topic",
      "/mine_uav/test/tunnel_bend_continuous_handover_goal_command");
  SuperExplorationDecider decider(nh, private_nh);

  std::mutex mutex;
  std::vector<super_planner::GoalCommand> received;
  auto subscriber = nh.subscribe<super_planner::GoalCommand>(
      "/mine_uav/test/tunnel_bend_continuous_handover_goal_command", 10,
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
  EXPECT_TRUE(SuperExplorationDeciderTestPeer::
                  tunnelBendHandsOverBeforeTheStraightGoalStops(decider));
  while (ros::ok() && std::chrono::steady_clock::now() < deadline) {
    {
      std::lock_guard<std::mutex> lock(mutex);
      if (received.size() >= 2u) break;
    }
    ros::WallDuration(0.01).sleep();
  }
  {
    std::lock_guard<std::mutex> lock(mutex);
    ASSERT_EQ(received.size(), 2u);
    EXPECT_EQ(received[0].command,
              super_planner::GoalCommand::CANCEL_GOAL);
    EXPECT_EQ(received[0].goal_id, 0u);
    EXPECT_EQ(received[1].command,
              super_planner::GoalCommand::SET_GOAL);
    EXPECT_EQ(received[1].reason, "connected_turn_route");
  }
  spinner.stop();
}

TEST(SuperGoalPublication, StrictTunnelReturnNeverTakesSideBranch) {
  ros::NodeHandle nh;
  SuperExplorationDecider decider(
      nh, ros::NodeHandle("~strict_tunnel_return_trace"));
  EXPECT_TRUE(SuperExplorationDeciderTestPeer::
                  strictTunnelReturnWaitsInsteadOfTakingSideBranch(decider));
}

TEST(SuperGoalPublication, GoafReturnSearchesLiveMapAfterBlockedTrace) {
  ros::NodeHandle nh, pnh("~goaf_blocked_trace_detour");
  SuperExplorationDecider decider(nh, pnh);
  EXPECT_TRUE(SuperExplorationDeciderTestPeer::
                  goafReturnSearchesLiveMapWhenTraceChordIsBlocked(decider));
}

TEST(SuperGoalPublication, LockedTunnelBranchReturnsAfterRouteTimeout) {
  ros::NodeHandle nh, pnh("~tunnel_branch_terminal");
  SuperExplorationDecider decider(nh, pnh);
  EXPECT_TRUE(SuperExplorationDeciderTestPeer::
                  lockedTunnelBranchReturnsAfterCoherentRouteTimeout(decider));
}

TEST(SuperGoalPublication, NoRouteTimerClearsOnlyAfterSuperAcceptance) {
  ros::NodeHandle nh, pnh("~no_route_timer_acceptance");
  SuperExplorationDecider decider(nh, pnh);
  EXPECT_TRUE(SuperExplorationDeciderTestPeer::
                  noRouteTimerClearsOnlyAfterSuperAcceptance(decider));
}

TEST(SuperGoalPublication, DecisionSnapshotDoesNotExpireDuringItsOwnPass) {
  ros::NodeHandle nh, pnh("~decision_snapshot_freshness");
  SuperExplorationDecider decider(nh, pnh);
  EXPECT_TRUE(SuperExplorationDeciderTestPeer::
                  decisionSnapshotDoesNotExpireDuringItsOwnPass(decider));
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

TEST(SuperGoalPublication, PersistentObstacleReplanInvalidatesCurrentEndpoint) {
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~obstacle_replan");
  SuperExplorationDecider decider(nh, private_nh);
  EXPECT_TRUE(SuperExplorationDeciderTestPeer::
                  persistentObstacleReplanInvalidatesCurrentEndpoint(decider));
}

}  // namespace mine_uav_control

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "super_goal_publication_test");
  return RUN_ALL_TESTS();
}
