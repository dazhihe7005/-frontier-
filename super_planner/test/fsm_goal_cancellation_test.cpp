#include <fsm/fsm.h>
#include <super_core/planner_commit.h>
#include <gtest/gtest.h>
#include <future>
#include <atomic>
#include <chrono>
#include <thread>

namespace {
class ProbeFsm : public fsm::Fsm {
 public:
  void activate(uint64_t id) {
    lifecycle_.activate(id, [&] {
      machine_state_ = FOLLOW_TRAJ;
      started_ = true;
      gi_.new_goal = true;
      plan_from_rest_ = true;
      finish_plan = false;
    });
  }
  void retarget(uint64_t id, bool can_continue) {
    activateGoalState(super_utils::Vec3f(id, 0, 1.5), 0.0,
                      id, can_continue);
  }
  void allowReplan() {
    lifecycle_.runIfCurrent(*lifecycle_.snapshot(), [&] { plan_from_rest_ = false; });
  }
  void runReplan() { callReplanOnce(); }
  std::function<super_utils::RET_CODE(super_planner::PlannerRunSnapshot*)> replan_body;
  std::function<super_utils::RET_CODE(const super_planner::PlannerCommitGate&,
                                     super_planner::PlannerRunSnapshot*)> replan_gate_body;
  int trajectoryPublications() const { return trajectory_publications_; }
  std::vector<int> recordedCodes() {
    std::lock_guard<std::mutex> lock(replan_log_mutex_);
    std::vector<int> result;
    for (const auto& log : replan_logs_) result.push_back(log.getRetCode());
    return result;
  }

  auto token() { return lifecycle_.snapshot(); }
  bool waitingForGoal() const { return machine_state_ == WAIT_GOAL; }
  bool following() const { return machine_state_ == FOLLOW_TRAJ; }
  bool hasNewGoal() const { return gi_.new_goal; }
  bool finishPlan() const { return finish_plan; }
  bool planFromRestPending() const { return plan_from_rest_; }
  int resetPathCalls() const { return reset_path_calls_; }
  uint64_t committedEpoch() const { return committed_traj_epoch_; }
  int commandCallbacks() const { return command_callbacks_; }
  bool commandAllowed(const fsm::GoalLifecycleGuard::Token& token) {
    return runCommandForCurrentTrajectory(token, [&] { ++command_callbacks_; });
  }
  bool commandAllowedAtRevision(const fsm::GoalLifecycleGuard::Token& token, uint64_t revision) {
    return runCommandForCurrentTrajectory(token, revision, [&] { ++command_callbacks_; });
  }
  bool commandWithPublish(const fsm::GoalLifecycleGuard::Token& token,
                          const std::function<void()>& publish) {
    return runCommandForCurrentTrajectory(token, publish);
  }
  bool simulatedCommit(const fsm::GoalLifecycleGuard::Token& token,
                       int& shared, int value) {
    return commitTrajectoryIfCurrent(token, [&] {
      shared = value;
      return true;
    });
  }
  bool simulatedPlannerGate(const fsm::GoalLifecycleGuard::Token& token,
                            const std::function<bool()>& install,
                            bool installs_trajectory = true) {
    return commitTrajectoryIfCurrent(token, install, installs_trajectory);
  }
  bool simulatedStateResult(const fsm::GoalLifecycleGuard::Token& token) {
    return lifecycle_.runIfCurrent(token, [&] { machine_state_ = FOLLOW_TRAJ; });
  }
  uint64_t stage() const { return planning_stage_; }
  void mainPlanCompleted() {
    lifecycle_.runIfCurrent(*token(), [&] {
      ChangeState("test main plan", FOLLOW_TRAJ);
      ++committed_traj_revision_;
    });
  }
  void trajectoryEnded() {
    lifecycle_.runIfCurrent(*token(), [&] { ChangeState("test trajectory end", GENERATE_TRAJ); });
  }
  bool replanResult(const fsm::GoalLifecycleGuard::Token& t, uint64_t stage,
                    uint64_t revision, super_utils::RET_CODE code) {
    return applyReplanResultIfCurrent(t, stage, revision, code);
  }
  uint64_t revision() const { return committed_traj_revision_; }

 protected:
  super_utils::RET_CODE executeReplan(const super_utils::Vec3f&, double, bool,
      const super_planner::PlannerCommitGate& gate,
      super_planner::PlannerRunSnapshot* snapshot) override {
    if (replan_gate_body) return replan_gate_body(gate, snapshot);
    return replan_body(snapshot);
  }
  void publishPolyTraj() override { ++trajectory_publications_; }
  void publishCurPoseToPath() override {}
  void resetVisualizedPath() override { ++reset_path_calls_; }

 private:
  int reset_path_calls_{0};
  int command_callbacks_{0};
  int trajectory_publications_{0};
};

TEST(FsmGoalCancellation, ClearsActiveGoalAndRejectsOldCommit) {
  ProbeFsm fsm;
  fsm.activate(21);
  const auto old = *fsm.token();
  int shared = 7;

  ASSERT_TRUE(fsm.cancelGoal(21));
  EXPECT_TRUE(fsm.waitingForGoal());
  EXPECT_FALSE(fsm.hasNewGoal());
  EXPECT_TRUE(fsm.finishPlan());
  EXPECT_FALSE(fsm.planFromRestPending());
  EXPECT_EQ(fsm.resetPathCalls(), 1);
  EXPECT_FALSE(fsm.simulatedCommit(old, shared, 21));
  EXPECT_EQ(shared, 7);
  EXPECT_FALSE(fsm.simulatedStateResult(old));
  EXPECT_FALSE(fsm.commandAllowed(old));
  EXPECT_EQ(fsm.commandCallbacks(), 0);
  EXPECT_FALSE(fsm.following());
}

TEST(FsmGoalCancellation, ReplacementRejectsOldCommitAndGatesOldTrajectory) {
  ProbeFsm fsm;
  fsm.activate(21);
  const auto old = *fsm.token();
  int shared = 0;
  ASSERT_TRUE(fsm.simulatedCommit(old, shared, 21));
  ASSERT_TRUE(fsm.commandAllowed(old));

  ASSERT_TRUE(fsm.cancelGoal(21));
  fsm.activate(22);
  const auto current = *fsm.token();
  EXPECT_FALSE(fsm.commandAllowed(current));
  EXPECT_EQ(fsm.commandCallbacks(), 1);
  EXPECT_FALSE(fsm.simulatedCommit(old, shared, 99));
  EXPECT_FALSE(fsm.simulatedStateResult(old));
  EXPECT_EQ(shared, 21);
  EXPECT_TRUE(fsm.simulatedCommit(current, shared, 22));
  EXPECT_EQ(shared, 22);
  EXPECT_TRUE(fsm.commandAllowed(current));
  EXPECT_FALSE(fsm.cancelGoal(21));
}

TEST(FsmGoalCancellation, DirectRetargetCarriesOnlyCurrentSafeTrajectory) {
  ProbeFsm fsm;
  fsm.activate(21);
  const auto old = *fsm.token();
  int shared = 0;
  ASSERT_TRUE(fsm.simulatedPlannerGate(old, [&] {
    shared = 21;
    return true;
  }));
  ASSERT_TRUE(fsm.commandAllowed(old));
  fsm.retarget(22, true);
  const auto current = *fsm.token();
  EXPECT_TRUE(fsm.following());
  EXPECT_TRUE(fsm.hasNewGoal());
  EXPECT_FALSE(fsm.planFromRestPending());
  EXPECT_TRUE(fsm.commandAllowed(current));
  EXPECT_FALSE(fsm.commandAllowed(old));
  EXPECT_FALSE(fsm.simulatedCommit(old, shared, 99));
  EXPECT_EQ(shared, 21);
  EXPECT_EQ(fsm.committedEpoch(), current.epoch);

  ASSERT_TRUE(fsm.cancelGoal(22));
  fsm.retarget(23, true);
  EXPECT_TRUE(fsm.waitingForGoal());
  EXPECT_FALSE(fsm.commandAllowed(*fsm.token()));
}

TEST(FsmGoalCancellation, RetargetWithoutSafeContinuationFallsBackToRest) {
  ProbeFsm fsm;
  fsm.activate(21);
  const auto old = *fsm.token();
  int shared = 0;
  ASSERT_TRUE(fsm.simulatedPlannerGate(old, [&] {
    shared = 21;
    return true;
  }));
  fsm.retarget(22, false);
  EXPECT_TRUE(fsm.waitingForGoal());
  EXPECT_TRUE(fsm.hasNewGoal());
  EXPECT_FALSE(fsm.commandAllowed(*fsm.token()));
  EXPECT_FALSE(fsm.commandAllowed(old));

  fsm.cancelGoal(22);
  fsm.retarget(23, true);
  EXPECT_TRUE(fsm.waitingForGoal());
  EXPECT_FALSE(fsm.commandAllowed(*fsm.token()));
}

TEST(FsmGoalCancellation, PlannerCommitHookSkipsActualCmdTrajInstallAfterCancel) {
  ProbeFsm fsm;
  fsm.activate(21);
  const auto old = *fsm.token();
  super_planner::CmdTraj shared;
  super_planner::CmdTraj prepared;
  super_planner::ExpTraj exp;
  geometry_utils::Trajectory pos, yaw;
  exp.setTrajectory(12.0, pos, yaw);
  prepared.setTrajectory(exp);
  int state_commits = 0;

  ASSERT_TRUE(fsm.cancelGoal(21));
  EXPECT_FALSE(super_planner::commitPreparedTrajectory(
      shared, prepared,
      [&](const std::function<bool()>& install, bool installs_trajectory) {
        return fsm.simulatedPlannerGate(old, install, installs_trajectory);
      },
      [&] { ++state_commits; }));
  EXPECT_TRUE(shared.empty());
  EXPECT_EQ(state_commits, 0);

  fsm.activate(22);
  const auto current = *fsm.token();
  EXPECT_FALSE(super_planner::commitPreparedTrajectory(
      shared, prepared,
      [&](const std::function<bool()>& install, bool installs_trajectory) {
        return fsm.simulatedPlannerGate(old, install, installs_trajectory);
      },
      [&] { ++state_commits; }));
  EXPECT_TRUE(shared.empty());
  EXPECT_TRUE(super_planner::commitPreparedTrajectory(
      shared, prepared,
      [&](const std::function<bool()>& install, bool installs_trajectory) {
        return fsm.simulatedPlannerGate(current, install, installs_trajectory);
      },
      [&] { ++state_commits; }));
  EXPECT_FALSE(shared.empty());
  EXPECT_EQ(state_commits, 1);
}

TEST(FsmGoalCancellation, StateOnlyReplanCannotPromoteOldTrajectoryEpoch) {
  ProbeFsm fsm;
  fsm.activate(21);
  const auto old = *fsm.token();
  int state_commits = 0;
  ASSERT_TRUE(fsm.simulatedPlannerGate(old, [&] { ++state_commits; return true; }, false));
  EXPECT_FALSE(fsm.commandAllowed(old));
  ASSERT_TRUE(fsm.cancelGoal(21));
  fsm.activate(22);
  const auto current = *fsm.token();
  ASSERT_TRUE(fsm.simulatedPlannerGate(current, [&] { ++state_commits; return true; }, false));
  EXPECT_FALSE(fsm.commandAllowed(current));
  EXPECT_EQ(fsm.committedEpoch(), 0u);
  EXPECT_EQ(state_commits, 2);
}

void expectDelayedPlannerCommitRejected(bool replace_goal) {
  ProbeFsm fsm;
  fsm.activate(21);
  const auto old = *fsm.token();
  super_planner::CmdTraj shared;
  super_planner::ExpTraj exp;
  geometry_utils::Trajectory pos, yaw;
  exp.setTrajectory(12.0, pos, yaw);
  std::promise<void> planning_started;
  std::promise<void> finish_planning;
  auto finish = finish_planning.get_future();
  bool accepted = true;
  int state_commits = 0;
  std::thread planning([&] {
    planning_started.set_value();
    finish.wait();
    super_planner::CmdTraj prepared;
    prepared.setTrajectory(exp);
    accepted = super_planner::commitPreparedTrajectory(
        shared, prepared,
        [&](const std::function<bool()>& install, bool installs_trajectory) {
          return fsm.simulatedPlannerGate(old, install, installs_trajectory);
        },
        [&] { ++state_commits; });
  });
  planning_started.get_future().wait();
  EXPECT_TRUE(fsm.cancelGoal(21));
  if (replace_goal) fsm.activate(22);
  finish_planning.set_value();
  planning.join();
  EXPECT_FALSE(accepted);
  EXPECT_TRUE(shared.empty());
  EXPECT_EQ(state_commits, 0);
  if (replace_goal) {
    EXPECT_FALSE(fsm.commandAllowed(*fsm.token()));
  }
}

TEST(FsmGoalCancellation, ActualInstallIsRejectedWhenCancelFinishesFirst) {
  expectDelayedPlannerCommitRejected(false);
}

TEST(FsmGoalCancellation, ActualInstallIsRejectedWhenReplacementFinishesFirst) {
  expectDelayedPlannerCommitRejected(true);
}

TEST(FsmGoalCancellation, OldReplanNewTrajCannotReplaceNewMainPlan) {
  ProbeFsm fsm;
  fsm.activate(21);
  auto t = *fsm.token();
  const auto stage = fsm.stage(), revision = fsm.revision();
  fsm.trajectoryEnded();
  fsm.mainPlanCompleted();
  EXPECT_FALSE(fsm.replanResult(t, stage, revision, super_utils::NEW_TRAJ));
  EXPECT_TRUE(fsm.following());
}

TEST(FsmGoalCancellation, OldReplanEmerCannotReplaceNewMainPlan) {
  ProbeFsm fsm;
  fsm.activate(21);
  auto t = *fsm.token();
  const auto stage = fsm.stage(), revision = fsm.revision();
  fsm.trajectoryEnded();
  fsm.mainPlanCompleted();
  EXPECT_FALSE(fsm.replanResult(t, stage, revision, super_utils::EMER));
  EXPECT_TRUE(fsm.following());
}

TEST(FsmGoalCancellation, HeartbeatSnapshotSurvivesConcurrentInstalls) {
  super_planner::CmdTraj shared, first, second;
  super_planner::ExpTraj exp_a, exp_b;
  geometry_utils::Trajectory pos, yaw;
  exp_a.setTrajectory(12.0, pos, yaw);
  exp_b.setTrajectory(34.0, pos, yaw);
  first.setTrajectory(exp_a);
  second.setTrajectory(exp_b);
  std::thread writer([&] {
    for (int i = 0; i < 2000; ++i) shared.installPrepared(i % 2 ? first : second);
  });
  for (int i = 0; i < 2000; ++i) {
    const auto snapshot = shared.heartbeatSnapshot();
    EXPECT_TRUE(snapshot.start_wall_time == 0.0 || snapshot.start_wall_time == 12.0 ||
                snapshot.start_wall_time == 34.0);
  }
  writer.join();
}

void expectDelayedRealReplanResultDropped(super_utils::RET_CODE result) {
  ProbeFsm fsm;
  fsm.activate(21);
  fsm.allowReplan();
  std::promise<void> entered, resume;
  auto release = resume.get_future();
  fsm.replan_body = [&](super_planner::PlannerRunSnapshot* snapshot) {
    entered.set_value();
    release.wait();
    snapshot->log.setRetCode(77);
    snapshot->module_time.resize(8);
    return result;
  };
  std::thread old_replan([&] { fsm.runReplan(); });
  entered.get_future().wait();
  fsm.trajectoryEnded();
  fsm.mainPlanCompleted();
  resume.set_value();
  old_replan.join();
  EXPECT_TRUE(fsm.following());
  EXPECT_EQ(fsm.recordedCodes(), std::vector<int>({77}));
}

TEST(FsmGoalCancellation, RealReplanEntryDropsDelayedNewTraj) {
  expectDelayedRealReplanResultDropped(super_utils::NEW_TRAJ);
}

TEST(FsmGoalCancellation, RealReplanEntryDropsDelayedEmergency) {
  expectDelayedRealReplanResultDropped(super_utils::EMER);
}

TEST(FsmGoalCancellation, AdjacentReplansRecordTheirOwnReturnedSnapshots) {
  ProbeFsm fsm;
  fsm.activate(21);
  fsm.allowReplan();
  std::promise<void> first_entered, first_release;
  auto resume = first_release.get_future();
  std::atomic<int> calls{0};
  fsm.replan_body = [&](super_planner::PlannerRunSnapshot* snapshot) {
    const int call = ++calls;
    if (call == 1) {
      first_entered.set_value();
      resume.wait();
    }
    snapshot->log.setRetCode(call);
    snapshot->module_time.resize(8);
    return super_utils::FAILED;
  };
  std::thread first([&] { fsm.runReplan(); });
  first_entered.get_future().wait();
  std::thread second([&] { fsm.runReplan(); });
  second.join();
  first_release.set_value();
  first.join();
  EXPECT_EQ(fsm.recordedCodes(), std::vector<int>({2, 1}));
}

TEST(FsmGoalCancellation, SlowCommandPreparationDoesNotDelayCancellation) {
  ProbeFsm fsm;
  fsm.activate(21);
  auto token = *fsm.token();
  int shared = 0;
  ASSERT_TRUE(fsm.simulatedCommit(token, shared, 1));
  std::promise<void> preparing, resume;
  auto release = resume.get_future();
  bool published = true;
  std::thread command([&] {
    preparing.set_value();
    release.wait();  // Controlled slow sampling/message construction.
    published = fsm.commandAllowed(token);
  });
  preparing.get_future().wait();
  auto cancel = std::async(std::launch::async, [&] { return fsm.cancelGoal(21); });
  EXPECT_EQ(cancel.wait_for(std::chrono::milliseconds(100)), std::future_status::ready);
  EXPECT_TRUE(cancel.get());
  resume.set_value();
  command.join();
  EXPECT_FALSE(published);
  EXPECT_EQ(fsm.commandCallbacks(), 0);
}

TEST(FsmGoalCancellation, NewTrajectoryRejectsPreparedOldCommandInSameEpoch) {
  ProbeFsm fsm;
  fsm.activate(21);
  const auto token = *fsm.token();
  int shared = 0;
  ASSERT_TRUE(fsm.simulatedCommit(token, shared, 1));
  const auto old_revision = fsm.revision();
  ASSERT_TRUE(fsm.simulatedCommit(token, shared, 2));
  EXPECT_FALSE(fsm.commandAllowedAtRevision(token, old_revision));
  EXPECT_TRUE(fsm.commandAllowedAtRevision(token, fsm.revision()));
  EXPECT_EQ(fsm.commandCallbacks(), 1);
}

TEST(FsmGoalCancellation, CancellationWaitsForAlreadyStartedPublishCall) {
  ProbeFsm fsm;
  fsm.activate(21);
  auto token = *fsm.token();
  int shared = 0;
  ASSERT_TRUE(fsm.simulatedCommit(token, shared, 1));
  std::promise<void> publishing, release_publish;
  auto resume = release_publish.get_future();
  std::thread command([&] {
    fsm.commandWithPublish(token, [&] {
      publishing.set_value();
      resume.wait();  // Controlled slow ROS publish boundary.
    });
  });
  publishing.get_future().wait();
  auto cancel = std::async(std::launch::async, [&] { return fsm.cancelGoal(21); });
  EXPECT_EQ(cancel.wait_for(std::chrono::milliseconds(20)), std::future_status::timeout);
  release_publish.set_value();
  command.join();
  EXPECT_TRUE(cancel.get());
  EXPECT_FALSE(fsm.commandAllowed(token));
}

TEST(FsmGoalCancellation, RealReplanEntryPublishesOnlyCommittedSuccess) {
  for (auto result : {super_utils::SUCCESS, super_utils::FINISH}) {
    ProbeFsm fsm;
    fsm.activate(21);
    fsm.allowReplan();
    super_planner::CmdTraj shared, prepared;
    super_planner::ExpTraj exp;
    geometry_utils::Trajectory pos, yaw;
    exp.setTrajectory(12.0, pos, yaw);
    prepared.setTrajectory(exp);
    fsm.replan_gate_body = [&](const super_planner::PlannerCommitGate& gate,
                               super_planner::PlannerRunSnapshot* snapshot) {
      snapshot->module_time.resize(8);
      EXPECT_TRUE(super_planner::commitPreparedTrajectory(shared, prepared, gate, [] {}));
      return result;
    };
    fsm.runReplan();
    EXPECT_FALSE(shared.empty());
    EXPECT_TRUE(fsm.commandAllowed(*fsm.token()));
    EXPECT_EQ(fsm.trajectoryPublications(), 1);
  }
}

TEST(FsmGoalCancellation, RealReplanEntryStateOnlyDoesNotEnableCommand) {
  ProbeFsm fsm;
  fsm.activate(21);
  fsm.allowReplan();
  fsm.replan_gate_body = [&](const super_planner::PlannerCommitGate& gate,
                             super_planner::PlannerRunSnapshot* snapshot) {
    snapshot->module_time.resize(8);
    EXPECT_TRUE(gate([] { return true; }, false));
    return super_utils::SUCCESS;
  };
  fsm.runReplan();
  EXPECT_FALSE(fsm.commandAllowed(*fsm.token()));
  EXPECT_EQ(fsm.trajectoryPublications(), 0);
}
}  // namespace

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
