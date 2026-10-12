#include <gtest/gtest.h>
#include <fsm/goal_command_validation.h>
#include <limits>

TEST(GoalCommandValidation, RejectsUnknownAndMalformedSet) {
  super_planner::GoalCommand command;
  command.command = 99;
  EXPECT_FALSE(fsm::validateGoalCommand(command, 0));
  command.command = super_planner::GoalCommand::SET_GOAL;
  command.goal_id = 0;
  command.goal.orientation.w = 1.0;
  EXPECT_FALSE(fsm::validateGoalCommand(command, 0));
  command.goal_id = 1;
  command.goal.position.x = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(fsm::validateGoalCommand(command, 0));
  command.goal.position.x = 0.0;
  command.goal.orientation.w = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(fsm::validateGoalCommand(command, 0));
  command.goal.orientation.w = 1.0;
  EXPECT_TRUE(fsm::validateGoalCommand(command, 0));
}

TEST(GoalCommandValidation, SpecificCancelMustMatchActiveGoal) {
  super_planner::GoalCommand command;
  command.command = super_planner::GoalCommand::CANCEL_GOAL;
  command.goal_id = 11;
  EXPECT_FALSE(fsm::validateGoalCommand(command, 12));
  EXPECT_TRUE(fsm::validateGoalCommand(command, 11));
  command.goal_id = 0;
  EXPECT_TRUE(fsm::validateGoalCommand(command, 11));
  EXPECT_TRUE(fsm::validateGoalCommand(command, 0));
}
