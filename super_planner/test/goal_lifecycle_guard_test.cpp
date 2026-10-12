#include <gtest/gtest.h>

#include <fsm/goal_lifecycle_guard.h>

#include <chrono>
#include <future>

TEST(GoalLifecycleGuard, CancelInvalidatesInflightPlanningToken) {
  fsm::GoalLifecycleGuard guard;
  const auto old = guard.activate(11);
  ASSERT_TRUE(guard.cancel(11));
  bool committed = false;
  EXPECT_FALSE(guard.commitIfCurrent(old, [&] { committed = true; }));
  EXPECT_FALSE(committed);
}

TEST(GoalLifecycleGuard, NewGoalRejectsOldResultAndAcceptsNewResult) {
  fsm::GoalLifecycleGuard guard;
  const auto old = guard.activate(11);
  const auto current = guard.activate(12);
  EXPECT_FALSE(guard.commitIfCurrent(old, [] {}));
  EXPECT_TRUE(guard.commitIfCurrent(current, [] {}));
}

TEST(GoalLifecycleGuard, StaleSpecificCancelCannotCancelNewGoal) {
  fsm::GoalLifecycleGuard guard;
  guard.activate(12);
  EXPECT_FALSE(guard.cancel(11));
  EXPECT_TRUE(guard.isActive());
  EXPECT_EQ(guard.activeGoalId(), 12u);
}

TEST(GoalLifecycleGuard, ZeroIdCancelIsUnconditional) {
  fsm::GoalLifecycleGuard guard;
  guard.activate(12);
  EXPECT_TRUE(guard.cancel(0));
  EXPECT_FALSE(guard.isActive());
}

TEST(GoalLifecycleGuard, ReactivatingSameGoalAdvancesEpoch) {
  fsm::GoalLifecycleGuard guard;
  const auto old = guard.activate(12);
  const auto current = guard.activate(12);
  EXPECT_GT(current.epoch, old.epoch);
  EXPECT_FALSE(guard.runIfCurrent(old, [] {}));
  EXPECT_TRUE(guard.runIfCurrent(current, [] {}));
}

TEST(GoalLifecycleGuard, ActivationCallbackRunsWithNewGoalActive) {
  fsm::GoalLifecycleGuard guard;
  bool callback_ran = false;
  const auto token = guard.activate(21, [&] { callback_ran = true; });
  EXPECT_TRUE(callback_ran);
  EXPECT_EQ(token.goal_id, 21u);
  EXPECT_TRUE(guard.isActive());
  EXPECT_EQ(guard.activeGoalId(), 21u);
}

TEST(GoalLifecycleGuard, RejectedCancelDoesNotRunCallback) {
  fsm::GoalLifecycleGuard guard;
  guard.activate(21);
  bool callback_ran = false;
  EXPECT_FALSE(guard.cancel(22, [&] { callback_ran = true; }));
  EXPECT_FALSE(callback_ran);
  EXPECT_TRUE(guard.isActive());
}

TEST(GoalLifecycleGuard, SuccessfulCancelRunsCallbackAndClearsGoal) {
  fsm::GoalLifecycleGuard guard;
  guard.activate(21);
  bool callback_ran = false;
  EXPECT_TRUE(guard.cancel(21, [&] { callback_ran = true; }));
  EXPECT_TRUE(callback_ran);
  EXPECT_FALSE(guard.isActive());
  EXPECT_EQ(guard.activeGoalId(), 0u);
}

TEST(GoalLifecycleGuard, SnapshotTracksEpochAndCancellation) {
  fsm::GoalLifecycleGuard guard;
  EXPECT_FALSE(guard.snapshot().has_value());

  const auto first = guard.activate(12);
  const auto first_snapshot = guard.snapshot();
  ASSERT_TRUE(first_snapshot.has_value());
  EXPECT_EQ(first_snapshot->epoch, first.epoch);
  EXPECT_EQ(first_snapshot->goal_id, first.goal_id);

  const auto second = guard.activate(12);
  const auto second_snapshot = guard.snapshot();
  ASSERT_TRUE(second_snapshot.has_value());
  EXPECT_EQ(second_snapshot->epoch, second.epoch);
  EXPECT_EQ(second_snapshot->goal_id, second.goal_id);
  EXPECT_NE(second_snapshot->epoch, first_snapshot->epoch);
  EXPECT_FALSE(guard.commitIfCurrent(*first_snapshot, [] {}));
  EXPECT_TRUE(guard.runIfCurrent(*second_snapshot, [] {}));

  ASSERT_TRUE(guard.cancel(12));
  EXPECT_FALSE(guard.snapshot().has_value());
  EXPECT_FALSE(guard.commitIfCurrent(*second_snapshot, [] {}));
}

TEST(GoalLifecycleGuard, CommitCallbackAndCancelAreLinearized) {
  using namespace std::chrono;
  fsm::GoalLifecycleGuard guard;
  const auto token = guard.activate(42);
  std::promise<void> callback_entered;
  std::promise<void> release_callback;
  std::promise<void> cancel_started;
  auto entered = callback_entered.get_future();
  auto release = release_callback.get_future();
  auto started = cancel_started.get_future();

  auto commit = std::async(std::launch::async, [&] {
    return guard.commitIfCurrent(token, [&] {
      callback_entered.set_value();
      release.wait();
    });
  });
  entered.wait();

  auto cancel = std::async(std::launch::async, [&] {
    cancel_started.set_value();
    return guard.cancel(42);
  });
  started.wait();
  EXPECT_EQ(cancel.wait_for(milliseconds(50)), std::future_status::timeout);
  release_callback.set_value();

  EXPECT_TRUE(commit.get());
  EXPECT_TRUE(cancel.get());
  EXPECT_FALSE(guard.snapshot().has_value());
  EXPECT_FALSE(guard.commitIfCurrent(token, [] {}));
}
