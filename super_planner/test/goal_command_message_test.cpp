#include <gtest/gtest.h>
#include <super_planner/GoalCommand.h>

TEST(GoalCommandMessage, ExposesAtomicLifecycleContract) {
  super_planner::GoalCommand message;
  message.command = super_planner::GoalCommand::SET_GOAL;
  message.goal_id = 42;
  message.goal.position.x = 3.0;
  message.reason = "forward_lookahead";
  EXPECT_EQ(message.command, 1);
  EXPECT_EQ(message.goal_id, 42u);
  EXPECT_DOUBLE_EQ(message.goal.position.x, 3.0);
}
