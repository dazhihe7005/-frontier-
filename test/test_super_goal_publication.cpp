#include "mine_uav_control/super_exploration_decider.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <mutex>
#include <vector>

namespace mine_uav_control {

class SuperExplorationDeciderTestPeer {
 public:
  static void publish(SuperExplorationDecider& decider,
                      const geometry_msgs::PoseStamped& goal,
                      const std::string& reason) {
    decider.publishGoal(goal, reason);
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
  geometry_msgs::PoseStamped second = first;
  second.pose.position.x = 6.0;
  SuperExplorationDeciderTestPeer::publish(decider, second, "test_second");
  SuperExplorationDeciderTestPeer::reset(decider);

  while (ros::ok() && std::chrono::steady_clock::now() < deadline) {
    {
      std::lock_guard<std::mutex> lock(mutex);
      if (received.size() >= 5) break;
    }
    ros::WallDuration(0.01).sleep();
  }
  {
    std::lock_guard<std::mutex> lock(mutex);
    ASSERT_EQ(received.size(), 5u);
    EXPECT_EQ(received[0].command, super_planner::GoalCommand::CANCEL_GOAL);
    EXPECT_EQ(received[0].goal_id, 0u);
    EXPECT_EQ(received[1].command, super_planner::GoalCommand::SET_GOAL);
    EXPECT_EQ(received[1].goal_id, 1u);
    EXPECT_EQ(received[1].reason, "test_first");
    EXPECT_EQ(received[2].command, super_planner::GoalCommand::CANCEL_GOAL);
    EXPECT_EQ(received[2].goal_id, 1u);
    EXPECT_EQ(received[3].command, super_planner::GoalCommand::SET_GOAL);
    EXPECT_EQ(received[3].goal_id, 2u);
    EXPECT_EQ(received[4].command, super_planner::GoalCommand::CANCEL_GOAL);
    EXPECT_EQ(received[4].goal_id, 0u);
  }
  spinner.stop();
}

}  // namespace mine_uav_control

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "super_goal_publication_test");
  return RUN_ALL_TESTS();
}
