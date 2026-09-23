#include "mine_uav_control/super_exploration_decider.hpp"

#include <gtest/gtest.h>
#include <pcl_conversions/pcl_conversions.h>
#include <boost/make_shared.hpp>

#include <chrono>
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

  static bool returnStartsAtObservedBreadcrumb(
      SuperExplorationDecider& decider) {
    decider.have_home_ = true;
    decider.home_pose_.pose.orientation.w = 1.0;
    decider.current_pose_.pose.position.x = 12.0;
    decider.outbound_breadcrumbs_.clear();
    for (double x : {0.0, 4.0, 8.0}) {
      geometry_msgs::PoseStamped point;
      point.pose.position.x = x;
      point.pose.orientation.w = 1.0;
      decider.outbound_breadcrumbs_.push_back(point);
    }
    decider.beginReturnHome("test_return");
    return decider.returning_home_ &&
           decider.return_waypoints_.size() == 3 &&
           decider.current_goal_.pose.position.x == 8.0 &&
           decider.return_waypoints_.back().pose.position.x == 0.0;
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
    decider.markOccupied(obstacle);
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

  static bool fallbackPhaseStillAllowsKnownFreeForwardLookahead(
      SuperExplorationDecider& decider) {
    decider.have_home_ = true;
    decider.mission_heading_yaw_ = 0.0;
    decider.exploration_phase_ =
        SuperExplorationDecider::ExplorationPhase::kFrontierFallback;
    decider.forward_lookahead_distance_ = 2.0;
    decider.forward_lookahead_step_ = 0.5;
    decider.min_goal_distance_ = 2.0;
    decider.max_exploration_radius_from_home_ = 10.0;
    const auto target = decider.positionToKey(2.0, 0.0, 0.0);
    decider.voxels_[target] = SuperExplorationDecider::FREE;
    SuperExplorationDecider::VoxelSet reachable;
    reachable.insert(target);
    return decider.publishForwardLookaheadGoal(reachable) &&
           decider.have_active_goal_;
  }

  static bool forwardLookaheadPrefersShorterLevelGoalOverClimb(
      SuperExplorationDecider& decider) {
    decider.have_home_ = true;
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
    reachable.insert(shorter_level);
    reachable.insert(farther_climb);
    return decider.publishForwardLookaheadGoal(reachable) &&
           decider.have_active_goal_ &&
           std::abs(decider.current_goal_.pose.position.x - 1.5) < 1e-9 &&
           std::abs(decider.current_goal_.pose.position.z) < 1e-9;
  }

  static bool reversedOriginCanUseOneVoxelObservationStep(
      SuperExplorationDecider& decider) {
    decider.have_home_ = true;
    decider.mission_heading_yaw_ = 0.0;
    decider.dead_end_heading_reversal_count_ = 1;
    decider.forward_lookahead_distance_ = 2.0;
    decider.forward_lookahead_step_ = 0.5;
    decider.min_goal_distance_ = 2.0;
    decider.voxel_resolution_ = 0.5;
    decider.max_exploration_radius_from_home_ = 10.0;
    decider.forward_goal_lateral_search_width_ = 0.0;
    const auto target = decider.positionToKey(0.5, 0.0, 0.0);
    decider.voxels_[target] = SuperExplorationDecider::FREE;
    SuperExplorationDecider::VoxelSet reachable;
    reachable.insert(target);
    return decider.publishForwardLookaheadGoal(reachable) &&
           std::abs(decider.current_goal_.pose.position.x - 0.5) < 1e-9;
  }

  static bool finalBacktrackWaypointUsesTightArrivalRadius(
      SuperExplorationDecider& decider) {
    decider.enabled_ = true;
    decider.have_data_ = true;
    decider.have_home_ = true;
    decider.last_sync_time_ = ros::Time::now();
    decider.data_timeout_ = 100.0;
    decider.dead_end_backtracking_ = true;
    decider.goal_reached_distance_ = 1.0;
    decider.voxel_resolution_ = 0.5;
    decider.return_waypoint_index_ = 0;
    decider.return_waypoints_.resize(1);
    decider.current_goal_ = decider.return_waypoints_.front();
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

TEST(SuperGoalPublication, ReturnDoesNotSendOneLongHomeGoal) {
  if (!ros::isInitialized()) {
    int argc = 0;
    ros::init(argc, nullptr, "super_return_breadcrumb_test",
              ros::init_options::AnonymousName);
  }
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  private_nh.setParam("goal_command_topic", "/mine_uav/test/return_goal_command");
  SuperExplorationDecider decider(nh, private_nh);
  EXPECT_TRUE(SuperExplorationDeciderTestPeer::returnStartsAtObservedBreadcrumb(
      decider));
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

TEST(SuperGoalPublication, FallbackPhaseDoesNotBlockKnownFreeForwardLookahead) {
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  private_nh.setParam("goal_command_topic",
                      "/mine_uav/test/fallback_forward_goal_command");
  SuperExplorationDecider decider(nh, private_nh);
  EXPECT_TRUE(SuperExplorationDeciderTestPeer::
                  fallbackPhaseStillAllowsKnownFreeForwardLookahead(decider));
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

}  // namespace mine_uav_control

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "super_goal_publication_test");
  return RUN_ALL_TESTS();
}
