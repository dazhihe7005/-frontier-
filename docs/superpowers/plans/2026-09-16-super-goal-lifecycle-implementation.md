# SUPER Goal Lifecycle Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add an explicit, race-safe `SET_GOAL/CANCEL_GOAL` protocol so SUPER cannot continue planning or publishing a canceled task-one target.

**Architecture:** `super_planner` owns one generated `GoalCommand` message and a goal-epoch guard. The exploration decider publishes every goal transition on one latched topic; SUPER snapshots the current epoch before expensive planning and may commit state or output only if that epoch is still active. PX4 hold behavior remains an independent downstream safety barrier.

**Tech Stack:** ROS1 Noetic, catkin, C++17 in SUPER, C++14 in `mine_uav_control`, GoogleTest, rosbag, PX4 SITL, Gazebo Classic.

**Spec:** `docs/superpowers/specs/2026-09-16-super-goal-lifecycle-design.md`

## Global Constraints

- Do not change trajectory limits, optimizer penalties, obstacle inflation, frontier scoring, return criteria, PX4 geofences, or the verified geometric time lower-bound fix.
- Preserve the existing downstream hold behavior in `super_px4_command_bridge`.
- Do not use a current-position fake goal to represent cancellation.
- Every expensive plan runs outside the lifecycle critical section; only snapshot and commit operations hold the lifecycle lock.
- Every result from an obsolete goal epoch is discarded before changing FSM state or publishing trajectory output.
- Preserve official click-goal mode; task one enables exactly one command input mode.
- The runtime SUPER tree is already dirty with known adaptations and tests. Never reset or overwrite it. Implement the standalone SUPER patch in an isolated official-baseline worktree, then apply that patch to the runtime tree.
- The repository copy of `mine_uav_control` is the source of truth. Mirror only reviewed files into `/home/nuc/super_ws/src/mine_uav_control` for integration tests.
- Do not claim real-flight readiness from unit tests or SITL.

---

### Task 1: Establish isolated source trees and preserve the baseline

**Files:**
- Read: `/home/nuc/frontier-upload`
- Read: `/home/nuc/SUPER-upstream-clean`
- Read: `/home/nuc/super_ws/src/SUPER`
- Create via worktree tooling: `/home/nuc/.worktrees/frontier-super-goal-lifecycle`
- Create via worktree tooling: `/home/nuc/.worktrees/SUPER-goal-lifecycle`

**Interfaces:**
- Consumes: the project `main` commit containing this plan and official SUPER commit `2ad3419c127a617c6d7df6925e81a14175a9c096`.
- Produces: Two isolated, clean implementation trees and baseline status captured in the work log.

- [ ] **Step 1: Invoke the worktree safety procedure**

Read and follow `superpowers:using-git-worktrees` before creating either worktree. Confirm ignore rules and choose paths outside the active source directories.

- [ ] **Step 2: Record the three current states without changing them**

Run:

```bash
git -C /home/nuc/frontier-upload status --short
git -C /home/nuc/frontier-upload rev-parse HEAD
git -C /home/nuc/SUPER-upstream-clean status --short
git -C /home/nuc/SUPER-upstream-clean rev-parse HEAD
git -C /home/nuc/super_ws/src/SUPER status --short
```

Expected: the project and upstream-clean repositories are clean; the runtime SUPER tree retains the documented local adaptations, geometric-time fix and diagnostic tests.

- [ ] **Step 3: Create project and SUPER worktrees**

Use branches `feature/super-goal-lifecycle` and `fix/super-goal-lifecycle` respectively. Verify both new worktrees report an empty `git status --short`.

- [ ] **Step 4: Run baseline tests without interpreting known alias failures as regressions**

Run the existing SUPER guide-time test and project tests. Record separately that the two ROGMap alias assertions are pre-existing known failures; every other newly failing test is a blocker.

---

### Task 2: Define and generate the atomic goal command message

**Files:**
- Create: `super_planner/msg/GoalCommand.msg`
- Modify: `super_planner/ros/ros1.CMakeLists.txt`
- Modify: `super_planner/ros/ros1.package.xml`
- Modify: selected copies `super_planner/CMakeLists.txt` and `super_planner/package.xml`
- Create: `super_planner/test/goal_command_message_test.cpp`

**Interfaces:**
- Consumes: `std_msgs/Header` and `geometry_msgs/Pose`.
- Produces: `super_planner::GoalCommand` with constants `SET_GOAL=1`, `CANCEL_GOAL=2`, fields `command`, `goal_id`, `goal`, and `reason`.

- [ ] **Step 1: Write the failing message contract test**

Add:

```cpp
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
```

Register the test target with dependencies on `${${PROJECT_NAME}_EXPORTED_TARGETS}` and `${catkin_EXPORTED_TARGETS}`.

- [ ] **Step 2: Run the target and verify RED**

Expected: compilation fails because `super_planner/GoalCommand.h` does not exist.

- [ ] **Step 3: Add the exact message definition**

```text
uint8 SET_GOAL=1
uint8 CANCEL_GOAL=2

std_msgs/Header header
uint8 command
uint64 goal_id
geometry_msgs/Pose goal
string reason
```

Add `add_message_files`, `generate_messages(DEPENDENCIES std_msgs geometry_msgs)`, `message_runtime`, and generated-target dependencies to both ROS1 templates and selected copies.

- [ ] **Step 4: Rebuild and verify GREEN**

Expected: the message header is generated and `GoalCommandMessage.ExposesAtomicLifecycleContract` passes.

- [ ] **Step 5: Commit**

```bash
git add super_planner/msg/GoalCommand.msg super_planner/ros/ros1.CMakeLists.txt   super_planner/ros/ros1.package.xml super_planner/CMakeLists.txt   super_planner/package.xml super_planner/test/goal_command_message_test.cpp
git commit -m "feat: define atomic SUPER goal command"
```

---

### Task 3: Implement the epoch guard with deterministic race tests

**Files:**
- Create: `super_planner/include/fsm/goal_lifecycle_guard.h`
- Create: `super_planner/test/goal_lifecycle_guard_test.cpp`
- Modify: both ROS1 CMake files

**Interfaces:**
- Produces: `GoalLifecycleGuard::Token {epoch, goal_id}`, `activate(goal_id, callback)`, `cancel(requested_goal_id, callback)`, no-callback convenience overloads for tests, `commitIfCurrent(token, callback)`, `runIfCurrent(token, callback)`, `activeGoalId()`, and `isActive()`.
- Contract: `cancel(0)` is unconditional; a nonzero cancellation only succeeds for the current external goal ID.

- [ ] **Step 1: Write deterministic failing tests**

```cpp
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
```

- [ ] **Step 2: Run the tests and verify RED**

Expected: compilation fails because `GoalLifecycleGuard` does not exist.

- [ ] **Step 3: Implement the minimal synchronized guard**

Use one private `std::mutex`, monotonically increasing `uint64_t epoch_`, `uint64_t active_goal_id_`, and `bool active_`. Activation and cancellation callbacks execute in the same critical section as the epoch change so FSM fields cannot diverge from the guard. `commitIfCurrent` and `runIfCurrent` execute the supplied short callback while the mutex is held only when token and active goal still match. Do not put planner calls inside these callbacks.

- [ ] **Step 4: Run the lifecycle and message tests**

Expected: all new tests pass, including stale-token and stale-specific-cancel cases.

- [ ] **Step 5: Commit**

```bash
git add super_planner/include/fsm/goal_lifecycle_guard.h   super_planner/test/goal_lifecycle_guard_test.cpp   super_planner/ros/ros1.CMakeLists.txt super_planner/CMakeLists.txt
git commit -m "test: guard SUPER goal epochs"
```

---

### Task 4: Make the SUPER FSM cancelable without committing stale plans

**Files:**
- Modify: `super_planner/include/fsm/fsm.h`
- Modify: `super_planner/src/super_core/fsm.cpp`
- Modify: `super_planner/include/super_core/super_planner.h`
- Modify: `super_planner/src/super_core/super_planner.cpp`
- Create: `super_planner/test/fsm_goal_cancellation_test.cpp`
- Modify: both ROS1 CMake files

**Interfaces:**
- Consumes: `GoalLifecycleGuard`.
- Produces: `bool setGoalPosiAndYaw(..., uint64_t goal_id)`, `bool cancelGoal(uint64_t requested_goal_id)`, and guarded planning-result commits.

- [ ] **Step 1: Add a failing FSM probe test**

Create a test-only subclass implementing the abstract publication methods and exposing protected state. Set it to `FOLLOW_TRAJ`, activate goal 21, call `cancelGoal(21)`, and require:

```cpp
EXPECT_TRUE(fsm.cancelGoal(21));
EXPECT_TRUE(fsm.waitingForGoal());
EXPECT_FALSE(fsm.hasNewGoal());
EXPECT_TRUE(fsm.finishPlan());
EXPECT_FALSE(fsm.planFromRestPending());
EXPECT_EQ(fsm.resetPathCalls(), 1);
```

Snapshot a token before cancel and assert a simulated old planning commit cannot change the probe back to `FOLLOW_TRAJ`.

- [ ] **Step 2: Run and verify RED**

Expected: cancellation API and lifecycle state are absent.

- [ ] **Step 3: Change goal acceptance to report success**

Change `setGoalPosiAndYaw` from `void` to `bool`. Return `false` for deeply occupied or too-close targets and `true` only after `gi_`, `started_`, and `new_goal` are committed inside the callback passed to `activate(goal_id, callback)`. Legacy click-goal and `callPlanOnce` callers supply generated nonzero IDs.

- [ ] **Step 4: Implement cancellation as one guarded transition**

For matching or unconditional cancellation, execute the following inside `cancel(requested_goal_id, callback)`:

```cpp
gi_.new_goal = false;
plan_from_rest_ = false;
finish_plan = true;
started_ = false;
machine_state_ = WAIT_GOAL;
```

Then clear the visualized path. A stale specific ID returns `false` without state mutation.

- [ ] **Step 5: Gate the planner's actual trajectory commit by epoch**

In `callMainFsmOnce` and `callReplanOnce`: atomically snapshot the active token and its matching FSM goal; run expensive planning outside the lifecycle guard. Official `PlanFromRest` and `ReplanOnce` call `cmd_traj_info_.setTrajectory(...)` *before returning*; a post-return FSM check alone is unsafe. Extend the planner API with a narrowly scoped commit gate so its locally computed trajectory and corresponding committed state are installed only while the current token is validated under the lifecycle guard. If rejected, do not alter the shared command trajectory and do not publish it. After planner return, guard all result-driven FSM state changes and `publishPolyTraj()`. Record the epoch of the actually committed trajectory; the command timer must not sample or publish while no active goal exists or while the active epoch differs from the committed trajectory epoch. Keep lock order lifecycle → CmdTraj consistent; do not hold lifecycle during optimization. Add deterministic interleaving tests for cancel-during-plan and cancel-then-new-goal-before-old-plan-completes. A rejected stale commit logs one throttled diagnostic and performs no mutation.

- [ ] **Step 6: Run SUPER lifecycle and guide-time tests**

Expected: cancellation and seven geometric-time tests pass. The documented ROGMap alias test remains the only known failure.

- [ ] **Step 7: Commit**

```bash
git add super_planner/include/fsm/fsm.h super_planner/src/super_core/fsm.cpp super_planner/include/super_core/super_planner.h super_planner/src/super_core/super_planner.cpp   super_planner/test/fsm_goal_cancellation_test.cpp   super_planner/ros/ros1.CMakeLists.txt super_planner/CMakeLists.txt
git commit -m "fix: cancel stale SUPER planning epochs"
```

---

### Task 5: Add the ROS1 command adapter and preserve click-goal mode

**Files:**
- Modify: `super_planner/include/fsm/config.hpp`
- Modify: `super_planner/include/ros_interface/ros1/fsm_ros1.hpp`
- Modify: `super_planner/config/click_smooth_ros1.yaml`
- Create: `super_planner/test/goal_command_validation_test.cpp`

**Interfaces:**
- Consumes: `/mine_uav/super/goal_command` as `super_planner/GoalCommand`.
- Produces: accepted set/cancel transitions while preserving `/goal` in legacy click mode.

- [ ] **Step 1: Write failing validation tests**

Extract pure validation helpers. Reject unknown command values, `SET_GOAL` with ID zero, non-finite position or quaternion, and cancellation for a non-current nonzero ID. Accept unconditional and matching cancellations.

- [ ] **Step 2: Run and verify RED**

Expected: validation helper symbols are missing.

- [ ] **Step 3: Add configuration fields**

```yaml
fsm:
  click_goal_en: false
  goal_command_en: true
  goal_command_topic: /mine_uav/super/goal_command
```

Initialization counts click and command modes and exits unless exactly one is enabled.

- [ ] **Step 4: Implement `goalCommandCallback`**

For `SET_GOAL`, validate fields and call `setGoalPosiAndYaw(position, quaternion, goal_id)`. For `CANCEL_GOAL`, call `cancelGoal(goal_id)` and reset wrapper-local `traj_finish_` only on success. Unknown or stale commands warn and do not mutate state.

- [ ] **Step 5: Gate ROS1 command publication**

Before generating output obtain the active token. Publish heartbeat or `PositionCommand` and perform trajectory-finish transitions only through `runIfCurrent(token, ...)`. Do not hold the lifecycle lock while planning.

- [ ] **Step 6: Verify both modes**

Build task command mode and official click mode. Exactly one target subscriber must be active in each configuration.

- [ ] **Step 7: Commit**

```bash
git add super_planner/include/fsm/config.hpp   super_planner/include/ros_interface/ros1/fsm_ros1.hpp   super_planner/config/click_smooth_ros1.yaml   super_planner/test/goal_command_validation_test.cpp
git commit -m "feat: accept ordered SUPER goal commands"
```

---

### Task 6: Migrate the exploration decider to the unified topic

**Files:**
- Modify: `include/mine_uav_control/super_exploration_decider.hpp`
- Modify: `src/super_exploration_decider.cpp`
- Create: `src/super_exploration_decider_node.cpp`
- Modify: `CMakeLists.txt`
- Modify: `package.xml`
- Create: `test/test_super_goal_publication.cpp`

**Interfaces:**
- Consumes: synchronized cloud/odometry and scheduler state as before.
- Produces: latched `/mine_uav/super/goal_command`; no task-mode `/goal`.

- [ ] **Step 1: Extract the node entry point without changing behavior**

Move only `main()` from `src/super_exploration_decider.cpp` to `src/super_exploration_decider_node.cpp`. Build the implementation as `super_exploration_decider_lib` and link the existing node executable against it. Run the current build before continuing; ROS node name, parameters and behavior must remain unchanged.

- [ ] **Step 2: Write a failing publisher test**

Use a friend test peer to invoke lifecycle exits. Subscribe and verify:

```text
startup CANCEL_GOAL goal_id=0
SET_GOAL goal_id=1 reason=test_first
CANCEL_GOAL goal_id=1 reason=test_replace
SET_GOAL goal_id=2 reason=test_second
CANCEL_GOAL goal_id=0 reason=test_reset
```

IDs must be nonzero and monotonically increasing.

- [ ] **Step 3: Run and verify RED**

Expected: decider still publishes `PoseStamped` and has no cancellation exit.

- [ ] **Step 4: Replace the goal publisher**

Depend on `super_planner`, include `GoalCommand.h`, rename `goal_topic_` to `goal_command_topic_`, and add `next_goal_id_` and `active_goal_id_`. `publishGoal` increments and publishes SET before updating local state. `cancelActiveGoal(reason, unconditional)` publishes CANCEL before clearing local state.

- [ ] **Step 5: Route every lifecycle exit through cancellation**

Cover reached goal, forward handover, front-wall preemption, timeout, replacement by return-home, home reached, mission disable, reset service, and active mission entering `WAIT_DATA`. Publish unconditional cancel after publisher creation. Keep `clearMissionState()` in-memory only; callers cancel before invoking it.

- [ ] **Step 6: Run publisher and existing project tests**

Expected: exact sequence passes; scheduler, bridge and arc behavior remains unchanged.

- [ ] **Step 7: Commit**

```bash
git add include/mine_uav_control/super_exploration_decider.hpp \
  src/super_exploration_decider.cpp src/super_exploration_decider_node.cpp \
  CMakeLists.txt package.xml test/test_super_goal_publication.cpp
git commit -m "fix: publish complete SUPER goal lifecycle"
```

---

### Task 7: Configure task one and add repeatable bag assertions

**Files:**
- Modify: `config/super_task1.yaml`
- Modify: `config/super_exploration_decider.yaml`
- Modify: `launch/task1_px4_sitl.launch`
- Modify: `launch/task1_real.launch`
- Modify: `scripts/analyze_task1_sitl_bag.py`
- Create: `test/test_analyze_goal_lifecycle.py`

**Interfaces:**
- Consumes: lifecycle commands, statuses, `/planning/pos_cmd`, and `/rosout_agg`.
- Produces: stale command count, stale `PlanFromRest` count, cancel-to-silence latency, and nonzero exit on regression.

- [ ] **Step 1: Write analyzer tests**

Cover:

```text
SET(1), planning command, CANCEL(1), no command             => PASS
SET(1), CANCEL(1), old command after 0.02 s                => FAIL
SET(1), CANCEL(1), SET(2), new command                     => PASS
CANCEL(1), SET(2), late CANCEL(1), goal-2 command           => PASS
```

Use a 20 ms grace interval only for one in-flight message.

- [ ] **Step 2: Run and verify RED**

Expected: lifecycle metrics are absent.

- [ ] **Step 3: Switch task configuration**

Set `click_goal_en: false`, `goal_command_en: true`, and the identical topic in both components. Replace task-mode recording of `/goal` with the command topic.

- [ ] **Step 4: Implement metrics**

Read bag events in timestamp order, track the active external ID, and count planning output or `PlanFromRest` more than 20 ms after cancellation and before the next valid SET. Include results in text and JSON.

- [ ] **Step 5: Run tests and syntax checks**

Expected: all four sequences match the required result and all project Python scripts pass `python3 -m py_compile`.

- [ ] **Step 6: Commit**

```bash
git add config/super_task1.yaml config/super_exploration_decider.yaml   launch/task1_px4_sitl.launch launch/task1_real.launch   scripts/analyze_task1_sitl_bag.py test/test_analyze_goal_lifecycle.py
git commit -m "test: enforce canceled-goal silence"
```

---

### Task 8: Build the standalone patch and integrate without overwriting work

**Files:**
- Create: `patches/super_goal_lifecycle_protocol.patch`
- Apply scoped changes to: `/home/nuc/super_ws/src/SUPER`
- Mirror reviewed project files to: `/home/nuc/super_ws/src/mine_uav_control`

**Interfaces:**
- Consumes: committed isolated SUPER and project branches.
- Produces: reproducible patch and runtime workspace containing both geometric-time and lifecycle fixes.

- [ ] **Step 1: Generate the patch against official commit `2ad3419`**

Verify it contains no ROGMap changes, optimizer tuning, build products, logs, or existing `camera_init` adaptation.

- [ ] **Step 2: Dry-run on a clean official worktree**

Run `git apply --check`. Expected: no rejected hunks.

- [ ] **Step 3: Apply carefully to runtime SUPER**

Save pre-apply status and scoped diffs. Run `git apply --check`. Resolve any overlap with existing frame adaptation only by a narrow `apply_patch`; never replace complete files.

- [ ] **Step 4: Mirror reviewed project files**

Compare each changed repository file with runtime `mine_uav_control`; preserve runtime-only parameter adjustments unless the reviewed change intentionally replaces them.

- [ ] **Step 5: Build and test**

```bash
source /opt/ros/noetic/setup.bash
cd /home/nuc/super_ws
catkin_make -DCMAKE_BUILD_TYPE=Release
catkin_make run_tests
catkin_test_results build/test_results
```

Expected: all new lifecycle tests and seven guide-time tests pass. The two documented ROGMap alias assertions may remain red with identical text; any other failure blocks SITL.

---

### Task 9: Perform the same-scene causal A/B

**Files:**
- Create runtime evidence: `/home/nuc/task1_logs/failure_cases/goal_lifecycle_fixed/`
- Modify after evidence: `docs/super_optimizer_and_goal_protocol_root_cause_report.md`
- Modify after evidence: `docs/mine_uav_project_conversation.md`

**Interfaces:**
- Consumes: existing official baseline bag and a new 40×40×30 m, 30 m lidar run.
- Produces: before/after counts proving or falsifying the lifecycle chain.

- [ ] **Step 1: Stop only confirmed stale simulation processes**

List matches first; do not use broad destructive process patterns.

- [ ] **Step 2: Start the exact scene**

```bash
source /opt/ros/noetic/setup.bash
source /home/nuc/super_ws/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11312
roslaunch mine_uav_control task1_40m_platform_sitl.launch gui:=true rviz:=true
```

- [ ] **Step 3: Record evidence**

Record `/clock`, command topic, status, `/planning/pos_cmd`, task odometry, MAVROS pose/state, command status and `/rosout_agg`. Continue through one no-active-goal interval or mission completion.

- [ ] **Step 4: Compare exact metrics**

Baseline:

```text
WAIT_MAP_CLOSURE interval: 21.7 s
new goals: 0
PlanFromRest calls: 316
/planning/pos_cmd messages: 1904
```

Fixed acceptance:

```text
stale PlanFromRest after cancel and before next SET: 0
stale /planning/pos_cmd after 20 ms grace: 0
new SET after cancel plans only its own goal ID: true
PX4 bridge holds current position with no goal: true
active-goal command rate: >= 50 Hz
```

- [ ] **Step 5: Stress cancellation**

While a goal is active, switch CH7 low. Verify no old output reappears. Perform the required low-to-high enable edge and verify a new epoch starts without reusing the old target.

- [ ] **Step 6: Decide from evidence**

Any stale result means stop and trace that epoch path before further edits. Passing closes only the lifecycle chain, not optimizer or coverage issues.

---

### Task 10: Review, document, and publish the bounded fix

**Files:**
- Modify: `README.md`
- Modify: `docs/super_optimizer_and_goal_protocol_root_cause_report.md`
- Modify: `docs/mine_uav_project_conversation.md`
- Add: `patches/super_goal_lifecycle_protocol.patch`

**Interfaces:**
- Consumes: fresh tests, Release build and A/B metrics.
- Produces: reproducible source, evidence-backed documentation and clean GitHub history.

- [ ] **Step 1: Request independent review**

Use `superpowers:requesting-code-review`. Review message ownership, stale cancel handling, epoch locking, planner calls outside locks, every cancellation exit, and scope creep.

- [ ] **Step 2: Address accepted findings test-first**

Each accepted defect receives a failing test before implementation. Reject unrelated tuning and refactors.

- [ ] **Step 3: Verify before completion**

Use `superpowers:verification-before-completion`. Re-run scoped tests, Release build, lifecycle analyzer and `git diff --check`. Confirm known ROGMap failures remain exactly two and no new failures exist.

- [ ] **Step 4: Update documentation with measured results**

Replace task-mode `/goal` instructions, document click compatibility, record exact before/after counts, and retain the real-flight warning.

- [ ] **Step 5: Commit and push**

Commit source by task boundary, then one evidence/documentation commit. Push only after clean status and fresh verification.

- [ ] **Step 6: Start the next independent root-cause audit**

Choose one fresh symptom: non-degenerate L-BFGS line-search failure, position-constraint failure, or goal-near discontinuity. Start a new causal investigation without parameter changes until its own runtime propagation chain is proven.
