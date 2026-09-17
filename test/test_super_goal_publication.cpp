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

}  // namespace mine_uav_control

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "super_goal_publication_test");
  return RUN_ALL_TESTS();
}
