# Task 2: shaft mission logic prototype (2026-09-18)

Status: **isolated 22 m PX4/Gazebo flight passed with a Gazebo downward ray range**, but depth still comes from ideal Gazebo world truth. No physical depth/laser source or degraded PX4-Z validation exists. Task 2 remains disabled in production `config/mission_scheduler.yaml` (`shaft_task_available: false`). The generic mission node publishes `/mine_uav/shaft/velocity_intent_enu`; only the explicitly named SITL launch connects this to MAVROS and arms a simulated vehicle.

## Control contract

- `/mine_uav/mission/shaft_enable` (`std_msgs/Bool`) enables a mission. False resets to IDLE; a new true edge recaptures the entrance depth.
- `/mine_uav/shaft/depth_estimate` (`mine_uav_control/ShaftDepthEstimate`) is the **control input**: time-stamped entrance-relative depth (positive down), declared standard uncertainty, validity, and source ID. The node rejects stale/future stamps, an unconfigured or mismatched source, invalid quality, and uncertainty above its configured bound. The configured source/frame are blank in generic/production configuration, so the node cannot command motion until a validated source is explicitly configured. `/mine_uav/shaft/relative_depth_m` remains **diagnostics only** for the SITL adapter, analyzer and takeover trigger. A source's self-declared uncertainty is not an independent proof of accuracy or provenance.
- `/mine_uav/shaft/bottom_range` (`sensor_msgs/Range`) is downward distance. Positive infinity means no target within range. Timestamp freshness, configured frame ID, sensor min/max and debounce are checked. The frame ID check is a contract, not proof that a physical laser points down.
- `/mine_uav/shaft/input_gate` (`std_msgs/String`) reports why the node refuses inputs (`UNCONFIGURED`, `WAIT_DEPTH`, `SOURCE_MISMATCH`, `STALE_DEPTH`, `BAD_DEPTH_QUALITY`, `BAD_OR_STALE_RANGE`) or `OPEN`.
- `/mine_uav/shaft/velocity_intent_enu` (`geometry_msgs/TwistStamped`) contains only an ENU Z velocity. Descend is negative, return is positive. The separate SITL-only router holds XY and handles a single MAVROS setpoint stream. Production XY control, yaw, corridor clearance, PX4 mode and setpoint arbitration are **not implemented**.
- `/mine_uav/shaft/status` reports IDLE, DESCENDING, RETURNING, COMPLETE or FAULT_NO_SAFE_AUTONOMOUS_RECOVERY. A fault suppresses intent; it does **not** guarantee the vehicle can hover, stop or return.

The algorithm records entrance depth on activation, descends with speed reduced according to bottom stopping distance, requires 0.3 s consecutive bottom detections, then returns toward the recorded entrance depth. Stale input, a 2 m depth jump, the 450 m depth limit, or 1800 s time limit enters FAULT. Invalid parameters fail before motion. This is a state-machine contract, not a flight safety case.

## Evidence

`catkin_make -C /home/nuc/super_ws shaft_mission_node test_shaft_mission` compiled. Six GTests pass, including an **idealized** 420 m kinematic loop that descended until the bottom became visible, triggered return and reached the starting depth. A ROS-topic integration test also passes: enable→descent→confirmed bottom→return→complete, then stale-range reset→fault. The first integration attempt correctly faulted because the test injected an impossible 0→5 m depth jump; after correcting that test input and rebuilding the node, the test passed. The model assumes perfect independent depth and bottom range measurements; it does not represent PX4 dynamics, barometer drift, sensor dropouts over hundreds of metres, walls, airflow, communications loss or battery reserves. The 22 m PX4/Gazebo probe below extends interface coverage but still does not address these real-world conditions.

The COMPLETE and FAULT states now remain latched even if sensors subsequently stop, until the enable signal goes low. This avoids turning an already-completed mission into a spurious post-completion fault. The node publishes no velocity intent in either terminal state.

## Next validation gates

1. Select, calibrate and independently validate a real depth/entrance-reference source and the PX4 navigation source needed to hold the shaft trajectory. A bottom-only laser cannot locate the starting elevation for return; commanded speed integrated over time is insufficient as a safety-critical substitute. The software quality gate below is not calibration.
2. Define and validate a recovery policy when the real depth source or PX4 Z estimate fails at depth. Current FAULT withdraws intent and requests PX4 mode fallback only in the SITL router; it cannot guarantee safe hover or return without reliable navigation. Add real XY/yaw clearance, battery/airflow/communications budgets and a production single-owner PX4 router only after sources and safety behavior are specified.
3. Replace the Gazebo ray approximation with the actual downward laser, deliberately degrade PX4 localization, verify real RC CH5 takeover and failsafes, and test depth/battery margins at multiple shaft sizes including 400+ m. Keep real-flight selection disabled until these pass.

## 22 m PX4/Gazebo interface probe (same day)

The first flights in this section used a computed world-truth range. The current default uses the Gazebo ray and qualified depth interface documented in the final section below; the historical bags are retained for causal comparison.

A separate simulation launch, `launch/task2_shaft_22m_px4_sitl.launch`, now runs PX4 SITL, a 10×10 m shaft of about 22 m depth, the existing simulated takeoff operator, a temporary entrance platform, the scheduler with **SITL-only** `shaft_task_available=true`, the shaft state machine, and a **SITL-only** single-owner MAVROS router. The router holds the captured XY position while sending vertical velocity, does not request OFFBOARD, and requests AUTO.LOITER on completion/fault. If the pilot/PX4 leaves OFFBOARD externally, it latches the takeover until enable goes low. Production `mission_scheduler.yaml` remains disabled for Task 2.

Two separate root causes were exposed before obtaining a passing flight:

1. Without an entrance platform, the disarmed Gazebo vehicle fell to the shaft bottom before the simulated pilot armed it. The first run therefore took off from the bottom and cannot be interpreted as a shaft mission. A named temporary platform now supports initial takeoff and is deleted only after an armed stable hover; the mission waits without motion for valid sensor frames after enable.
2. With that platform, the first full 22 m closed loop still had only **0.504 m** minimum body-outside bottom clearance (0.4 m equivalent radius). The synthetic range was generated from PX4 odometry whose origin was established after the vehicle settled on the platform, but its nominal bottom depth was fixed relative to the earlier spawn position. At the physical lowest point, sensor range said **1.804 m** while actual center-to-floor distance was **0.904 m**. Across the flight the maximum range/world discrepancy was **1.086 m**. This constant-frame error, not a proved PX4 braking failure, explains the missed 1 m clearance requirement. That failure bag is `/home/nuc/task2_logs/probes/task2_22m_pad_px4_sitl_20260918.bag`.

The adapter now uses Gazebo world truth to generate idealized, internally aligned depth and bottom range, and publishes nothing while the entrance pad is present. The same 22 m flight then reached RETURNING at 60.109 s and COMPLETE at 102.159 s, followed by PX4 AUTO.LOITER at 102.520 s; no task fault or in-task disarm. Minimum body-outside bottom clearance was **1.373 m**, minimum side-wall clearance **4.465 m**, and maximum XY deviation from the initial active position **0.142 m**. The maximum range/world disagreement fell to **0.032 m**. The pass bag is `/home/nuc/task2_logs/probes/task2_22m_world_truth_px4_sitl_20260918.bag`. Both bags are local evidence, not in GitHub. A later router change latches external OFFBOARD takeover; it has now been rerun in PX4 SITL using the separate mode-exit test below.

To re-audit the passing bag:

```bash
source /opt/ros/noetic/setup.bash
python3 /home/nuc/frontier-upload/scripts/analyze_task2_sitl_bag.py \
  /home/nuc/task2_logs/probes/task2_22m_world_truth_px4_sitl_20260918.bag \
  --require-complete --min-bottom-margin 1 \
  --max-xy-deviation 0.5 --max-range-alignment-error 0.1
```

The preceding bag fails both bottom-margin and range-alignment limits with the same command. This is a PX4/Gazebo **interface** result, not a real 420 m shaft validation: the downward range is calculated from Gazebo truth, depth is ideal world truth, PX4 still has standard SITL localization, side walls are wide, and no real lidar or loss-of-Z/failsafe behavior is exercised. The actual shaft task still requires an independent depth sensor and verified recovery under sensor failures.

## CH11 scheduler integration and range-loss fault injection

The scheduler now treats the first stable CH7/CH11 readings as a baseline and starts Task 1/Task 2 only on a subsequent debounced edge in either direction. A running task ignores further edges; terminal status revokes its enable and requires fresh baseline settling before another start. The ROS `mission_scheduler_edges` test passed for mutual exclusion, repeat runs, manual mode takeover, shaft operation without Fast-LIO odometry, and Task 1 odometry-loss behavior. An odometry loss now requests HOLD without a misleading autonomous return-home command. Production Task 2 remains disabled.

The same 22 m PX4/Gazebo launch was rerun under the CH11 scheduler with ideal aligned world-truth depth/range. The saved bag `/home/nuc/task2_logs/probes/task2_22m_ch11_scheduler_20260918.bag` passed the existing analyzer: DESCENDING 17.371 s, RETURNING 59.421 s, COMPLETE 101.371 s, AUTO.LOITER about 101.52 s; deepest entrance-relative depth 19.282 m, minimum body-outside bottom margin 1.417 m, minimum side margin 4.457 m, maximum XY deviation 0.142 m and range/world alignment discrepancy 0.034 m. This rerun covers the router's previously unrerun change in the normal completion path; deliberate pilot takeover still needs separate verification.

For a controlled sensor failure, `task2_shaft_22m_px4_sitl.launch` accepts `range_drop_after_depth:=6.0`. Its SITL-only adapter then stops publishing bottom range once ideal entrance-relative depth reaches 6 m while continuing independent depth. In the extended bag `/home/nuc/task2_logs/probes/task2_22m_range_loss_ch11_observed_20260918.bag`, descent began at 17.377 s, `FAULT_NO_SAFE_AUTONOMOUS_RECOVERY` occurred at 32.978 s near depth 6.212 m, the scheduler revoked Task 2 by 33.027 s, and PX4 entered AUTO.LOITER by 33.517 s. The fault-to-LOITER interval was 0.539 s. Across 24.667 s of Gazebo observations after the fault, maximum additional descent was 0.152 m. This is a **controlled SITL result with valid PX4 localization**, not proof of safe recovery if Z is unreliable. A first, shorter bag is also retained locally but is not used to claim stable post-fault behavior.

Re-audit the extended fault bag:

```bash
source /opt/ros/noetic/setup.bash
python3 /home/nuc/frontier-upload/scripts/analyze_task2_sitl_bag.py \
  /home/nuc/task2_logs/probes/task2_22m_range_loss_ch11_observed_20260918.bag \
  --require-fault --max-fault-to-loiter 1.0 \
  --max-post-fault-descent 1.0 --min-post-fault-observation 10
```

The analyzer deliberately does not apply its range-alignment threshold to a range-loss bag: the most recent range becomes stale by design. Remaining blockers are a physically independent depth/entrance reference, actual laser integration, degraded-Z tests, a non-GPS recovery policy, real RC takeover checks, and 400+ m/airflow/battery validation. None is solved by this fault injection.

## External OFFBOARD exit while descending (SITL only)

With the real MID360s not mounted, we tested another radar-independent boundary: whether Task 2 keeps commanding the vehicle after PX4 leaves OFFBOARD. An initial attempt sent `POSCTL` with `/mavros/set_mode` at about 6.0 m depth. MAVROS returned `mode_sent=true`, but PX4 logged command result 1 and `/mavros/state` stayed in OFFBOARD. That is **not** a successful manual takeover. With the same isolated 22 m SITL, requesting `AUTO.LOITER` produced PX4 command result 0 and an observed `/mavros/state` mode change. This is an external **mode-service** exit, not a physical RC CH5 takeover.

The reproducible injector `scripts/sitl_shaft_takeover_injector.py` is opt-in (`inject_takeover:=true`), checks simulated time and the launch's local UDP FCU URL, sends one mode request after 6 m descent, then distinguishes `SENT_WAIT_CONFIRM` from `CONFIRMED` using PX4's actual `/mavros/state`. In `/home/nuc/task2_logs/probes/task2_22m_auto_takeover_20260918.bag`, shaft descent began at 17.369 s; the injector sent at 32.601 s, and PX4 was observed in AUTO.LOITER at 33.522 s. The scheduler entered HOLD 0.078 s later and the task command-ready flag went false 0.028 s later. Over the next 36.137 s of observation, there was no observed OFFBOARD re-entry and **zero** new MAVROS position/raw setpoints. This first bag predates the extra local FCU URL guard. A second flight with that guard and the stricter auditor, `/home/nuc/task2_logs/probes/task2_22m_auto_takeover_guarded_20260918.bag`, again confirmed OFFBOARD during descent followed by actual AUTO.LOITER at 33.526 s, HOLD 0.074 s later, command-ready false 0.024 s later, no observed OFFBOARD re-entry or new setpoints over 17.646 s. Both bags stay local, not in GitHub.

To repeat and audit (isolated SITL only; never against a real FCU):

```bash
source /opt/ros/noetic/setup.bash
source /home/nuc/super_ws/devel/setup.bash
source /home/nuc/PX4-Autopilot/Tools/simulation/gazebo-classic/setup_gazebo.bash \
  /home/nuc/PX4-Autopilot /home/nuc/PX4-Autopilot/build/px4_sitl_default
export ROS_MASTER_URI=http://localhost:11319
export GAZEBO_MASTER_URI=http://localhost:11499
export ROS_PACKAGE_PATH=/home/nuc/PX4-Autopilot:/home/nuc/PX4-Autopilot/Tools/simulation/gazebo-classic:${ROS_PACKAGE_PATH}
roslaunch -p 11319 mine_uav_control task2_shaft_22m_px4_sitl.launch \
  gui:=false inject_takeover:=true takeover_after_depth:=6.0
```

Record the listed mode, mission, readiness and setpoint topics during the run; audit the saved evidence with:

```bash
python3 /home/nuc/frontier-upload/scripts/analyze_task2_takeover_bag.py \
  /home/nuc/task2_logs/probes/task2_22m_auto_takeover_guarded_20260918.bag
```

The analyzer also correctly fails when asked to verify `--expected-mode POSCTL` against the LOITER bag. This verifies command withdrawal after an accepted external mode exit with ideal SITL localization, **not** physical RC override, no-Z flight, or safe hover with a failed real positioning source. Production Task 2 remains unavailable.

## Gazebo ray range and fail-closed depth input (same day)

The shared SITL model now includes a downward one-beam Gazebo ray sensor. The 22 m launch defaults to `range_source:=gazebo`; the adapter relays the `sensor_msgs/Range` from `/mine_uav/sitl/shaft_downward_range` to the mission input only after deleting the temporary entrance platform. It still uses **Gazebo world truth for the independent entrance-relative depth**. The previous synthetic range is retained only as an explicit `range_source:=truth` regression option; launch selects the matching required range frame automatically. This is a ray/range interface test, not a MID360s or real laser packet emulator.

The mission node no longer accepts the unqualified `/mine_uav/shaft/relative_depth_m` diagnostic scalar as its control input. It requires `ShaftDepthEstimate` with source ID, measurement timestamp, declared sigma and valid flag, plus the configured downward range frame. Default `config/shaft_mission_logic.yaml` leaves required source and frame blank, failing closed outside an explicitly configured test/validated real source. The SITL launch sets `gazebo_world_truth` and `shaft_downward_range` and declares a simulated 0.02 m sigma; that number is only a simulation declaration. Because a ROS publisher can claim any source ID/sigma, the interface **does not authenticate a source or prove physical accuracy**. The real depth producer and independent calibration are still missing.

The isolated PX4/Gazebo bags are local-only evidence in `/home/nuc/task2_logs/probes/`:

| Bag suffix | Input/fault | Observed result |
| --- | --- | --- |
| `task2_22m_gazebo_ray_20260918.bag` | Gazebo ray, pre-quality-gate baseline | RETURNING 59.471 s, COMPLETE 101.471 s; minimum body-outside bottom margin 1.409 m; max range/world error 0.028 m. |
| `task2_22m_ray_depth_dropout_20260918.bag` | Depth stream stops at 6 m while ray range continues, pre-quality-gate baseline | FAULT 32.921 s, AUTO.LOITER 0.404 s later; 27.697 s post-fault observation, max additional descent 0.173 m. |
| `task2_22m_ray_quality_gate_20260918.bag` | Final depth message contract and Gazebo ray | `input_gate=OPEN`, RETURNING 59.503 s, COMPLETE 101.453 s; minimum body-outside bottom margin 1.406 m, minimum side margin 4.453 m, max XY deviation 0.143 m, max range/world error 0.029 m. All 2124 forwarded range samples in the bag match a raw ray sample by timestamp and value. |
| `task2_22m_ray_bad_sigma_20260918.bag` | At 6 m, the simulated depth producer reports sigma 1.0 m, above the 0.25 m gate | `BAD_DEPTH_QUALITY` and FAULT at 32.624 s; AUTO.LOITER 0.698 s later; 27.15 s post-fault observation, max additional descent 0.166 m. |

Run the normal 22 m SITL with the launch command above, omitting `inject_takeover:=true`; `range_source:=gazebo` is now the default. For fault injection add exactly one of `depth_drop_after_depth:=6.0`, `range_drop_after_depth:=6.0`, or `bad_sigma_after_depth:=6.0`. These arguments affect **only** the isolated SITL adapter. Re-audit the final normal and quality-fault bags:

```bash
source /opt/ros/noetic/setup.bash
python3 /home/nuc/frontier-upload/scripts/analyze_task2_sitl_bag.py \
  /home/nuc/task2_logs/probes/task2_22m_ray_quality_gate_20260918.bag \
  --require-complete --require-ray-relay --require-gate-event OPEN \
  --min-bottom-margin 1 --max-xy-deviation 0.5 --max-range-alignment-error 0.1
python3 /home/nuc/frontier-upload/scripts/analyze_task2_sitl_bag.py \
  /home/nuc/task2_logs/probes/task2_22m_ray_bad_sigma_20260918.bag \
  --require-fault --require-gate-event BAD_DEPTH_QUALITY \
  --max-fault-to-loiter 1 --max-post-fault-descent 1 \
  --min-post-fault-observation 10
```

`rostest mine_uav_control shaft_mission_ros.test` passes 3 tests, including wrong source, excessive sigma, invalid flag, replayed depth timestamp, wrong range frame and active fault. Six shaft state-machine GTests and the CH7/CH11 scheduler edge integration test pass. The 420 m result remains a perfect-sensor kinematic unit test, **not** a 420 m PX4/Gazebo or physical flight. No reliable real Z/depth source, physical laser, real RC takeover or safe recovery under PX4 Z loss has been validated. Production Task 2 must remain disabled.

As a fail-closed default check, the generic `task2_shaft_logic.launch` was started alone on isolated ROS master port 11323 without test overrides: `/mine_uav/shaft/input_gate=UNCONFIGURED`, mission status `IDLE`, and no PX4 router was launched. The process was then stopped. The pre-existing 11312 master was left untouched.

## Scheduler and Task 2 ownership watchdogs

The CH7/CH11 scheduler and SITL-only Task 2 router were audited for mission handoff, not just a single successful flight. Three scheduler gaps had the same root pattern: stale positive state was retained indefinitely. `/mavros/state` had no freshness check; a stopped state stream left the last `connected=true` usable forever. `/mine_uav/shaft/status` had no freshness check; a dead shaft node could leave Task 2 selected. Finally, a shaft node that stayed `IDLE` for lack of sensor input never caused the scheduler to time out. Each was reproduced in the extended ROS edge test before the scheduler change (the first exposed case kept Task 2 selected past the expected MAVROS timeout). The scheduler now revokes Task 2 after stale MAVROS state (default 2.5 s), stale shaft status (default 0.6 s), or failure to reach `DESCENDING`/`RETURNING` within 4 s of selection. A backward `/clock` jump also invalidates cached freshness and rebuilds RC switch baselines. The test uses shorter override values to exercise all three exits, then verifies a fresh CH11 edge can select Task 2 again. These timeouts withdraw NUC task authority; they are not a proof that PX4 can safely hold altitude on a lost estimator.

A separate route-ownership defect was confirmed by a callback-level regression: if the scheduler sent its one `shaft_enable=false` on completion while PX4 was still in OFFBOARD, then PX4 changed mode a moment later, the router latched the mode exit **after** the false edge. No further false message was guaranteed, so a later CH11 run could remain blocked. The router now latches an external takeover only while that task is still enabled; on disable it also discards the previous velocity intent and rejects any late-arriving intent while disabled. It independently rejects stale PX4-state heartbeats (default 1.5 s) and replayed intent timestamps, withdrawing command readiness and requesting the SITL fallback instead of continuing a descent command. Five callback-level tests cover delayed exit/restart, pilot takeover latch, stale PX4 state, replayed intent and a late intent after disable; the delayed-exit and late-intent tests failed against the old router and pass after these changes. Real RC takeover is still untested.

The final code was rerun in isolated 22 m PX4/Gazebo SITL with the Gazebo ray and depth quality gate. Normal bag `/home/nuc/task2_logs/probes/task2_22m_scheduler_router_watchdogs_20260918.bag`: DESCENDING 17.455 s, RETURNING 59.455 s, COMPLETE 101.456 s, PX4 AUTO.LOITER 102.520 s; minimum body-outside bottom margin 1.384 m, minimum side margin 4.445 m, max XY deviation 0.141 m; all 2006 forwarded bottom-range samples in the bag match raw ray samples. Mid-descent mode exit bag `/home/nuc/task2_logs/probes/task2_22m_takeover_watchdogs_20260918.bag`: PX4 actually entered AUTO.LOITER at 33.520 s, scheduler withdrew Task 2 0.080 s later, command-ready became false 0.031 s later, and over 28.097 s there was no observed OFFBOARD re-entry or further setpoint. Bags are local only. The normal flight and mode-service takeover passed their analyzers; no physical CH5 or real no-Z response is implied.

After the late-intent race fix, the **actual final code** was rerun once more in the normal 22 m SITL. `/home/nuc/task2_logs/probes/task2_22m_final_handoff_20260918.bag` shows DESCENDING 17.424 s, RETURNING 59.424 s, COMPLETE 101.274 s, AUTO.LOITER 102.322 s, minimum body-outside bottom margin 1.434 m, minimum side margin 4.476 m and max XY deviation 0.139 m. All 2106 forwarded range samples recorded in this bag match the raw Gazebo ray by timestamp and value. The callback-level late-intent regression is additionally registered with catkin nosetests (5/5 passed). The previous takeover bag predates only the disabled-intent filter; that filter rejects messages while disabled and does not change the enabled takeover path. Real takeover remains untested.

```bash
python3 /home/nuc/frontier-upload/scripts/analyze_task2_sitl_bag.py \
  /home/nuc/task2_logs/probes/task2_22m_final_handoff_20260918.bag \
  --require-complete --require-ray-relay --min-bottom-margin 1 \
  --max-xy-deviation 0.5 --max-range-alignment-error 0.1
```

Recheck with:

```bash
source /opt/ros/noetic/setup.bash
source /home/nuc/super_ws/devel/setup.bash
rostest mine_uav_control mission_scheduler_edges.test
python3 -m unittest discover -s /home/nuc/frontier-upload/test \
  -p test_sitl_shaft_router_lifecycle.py -v
python3 /home/nuc/frontier-upload/scripts/analyze_task2_sitl_bag.py \
  /home/nuc/task2_logs/probes/task2_22m_scheduler_router_watchdogs_20260918.bag \
  --require-complete --require-ray-relay --require-gate-event OPEN \
  --min-bottom-margin 1 --max-xy-deviation 0.5 --max-range-alignment-error 0.1
python3 /home/nuc/frontier-upload/scripts/analyze_task2_takeover_bag.py \
  /home/nuc/task2_logs/probes/task2_22m_takeover_watchdogs_20260918.bag
```

## RViz display and replayed-pose gate

The isolated `task2_shaft_22m_px4_sitl.launch` now accepts `gui:=true rviz:=true` for simultaneous Gazebo and RViz viewing. RViz uses `camera_init` as its fixed frame and shows the simulated PX4 path, pose and Gazebo ray cloud. The ray plugin actually publishes `sensor_msgs/PointCloud` (not `PointCloud2`); an initial PointCloud2 RViz display opened but was rejected for MD5/type mismatch, so the task-two config uses the `rviz/PointCloud` display. A visualization-only TF helper uses PX4 SITL odometry and the model's fixed +0.14 m sensor mount to connect `camera_init -> base_link -> mid360_link`. The TF lookup and cloud publishing were observed live; the corrected RViz display was not reopened after the user asked to stop viewing. This is a simulated ray cloud, not MID360s/FAST-LIO2 mapping. The helper does not feed PX4 localization or control.

A separate SITL-only router defect was reproduced with callback tests before modification: it used only callback receipt time for PX4 pose freshness, so replaying an old `PoseStamped` with a fresh receipt time still permitted descent setpoints. The same missing validity gate let NaN/Inf position through; on first XY capture it could put a nonfinite coordinate into the PX4 target. The router now requires both receipt and message timestamps within `pose_timeout` and finite X/Y/Z before publishing descent or fallback hold. If it already owns OFFBOARD, invalid pose withdraws command readiness and requests the existing SITL AUTO.LOITER fallback; this assumes valid PX4 navigation and is not a no-Z safety solution. The original stale/nonfinite tests failed against old code and all 8 router lifecycle tests pass after the change, including a positive fresh-pose/intent target case.

The post-change full 22 m headless PX4/Gazebo bag `/home/nuc/task2_logs/probes/task2_22m_pose_gate_20260918.bag` passed the standard analyzer: DESCENDING 17.373 s, RETURNING 59.424 s, COMPLETE 101.373 s, AUTO.LOITER 102.325 s; minimum body-outside bottom clearance 1.417 m, minimum side clearance 4.446 m and maximum XY deviation 0.133 m. All 1911 forwarded range samples in this bag match raw Gazebo ray samples. The 3146 recorded `/mavros/local_position/pose` messages all used `map`; header-to-record time discrepancy was -0.008 to +0.007 s. The bag is local evidence only, not uploaded. All launched 11319 SITL processes were stopped afterward; the real 11312 master and production `shaft_task_available=false` were untouched.

## 45 m range-limited shaft regression

The new `shaft_45m_logic_sitl.world` has the same 10×10 m interior, an entrance reference at world Z=+0.25 m and bottom top at Z=-44.85 m. `task2_shaft_45m_px4_sitl.launch` binds that world and geometry reference together, preventing a world-only override from accidentally retaining the 22 m truth-range constant. The actual default range remains the Gazebo downward ray with a 30 m maximum; independent depth remains ideal Gazebo world truth. No real depth source, no real laser, and no degraded PX4 Z are exercised.

The full isolated PX4/Gazebo bag `/home/nuc/task2_logs/probes/task2_45m_range_transition_20260918.bag` recorded DESCENDING at 17.423 s, RETURNING at 106.323 s, COMPLETE at 194.273 s and AUTO.LOITER at 195.325 s. Minimum body-outside bottom clearance was 1.386 m, minimum side-wall clearance 4.477 m, and maximum XY deviation 0.135 m. All 3991 task range messages match a raw Gazebo ray message by timestamp and value. During descent, 666 task range samples were at the 30.0 m sensor maximum before finite sub-limit readings resumed; this Gazebo plugin uses `30.0`, not `+inf`, for its out-of-range condition. A saturated reading cannot be interpreted as a true 30.0 m floor distance.

The old bag analyzer made exactly that interpretation, reporting a false 16.779 m ray/world discrepancy and failing the 0.1 m limit on this otherwise completed flight. The analyzer now treats `range >= max_range - 0.01 m` as a censored measurement when evaluating distance accuracy, while still requiring every forwarded value to match the raw Gazebo ray. It also provides `--require-range-reacquisition`, which requires at least one saturated descent sample followed by a sub-limit sample. On the same 45 m bag the corrected finite-range alignment discrepancy is 0.030 m and all requested assertions pass. The prior 22 m bag still passes its standard assertions, and deliberately fails `--require-range-reacquisition` because its bottom was always in range. These are log-auditor changes, not flight-controller changes.

Three standalone unit tests in `test/test_task2_range_censoring.py` cover a censored 30 m reading, a resolved 17 m hit, and an infinite out-of-range reading. All pass with `python3 -m unittest /home/nuc/frontier-upload/test/test_task2_range_censoring.py -v` after sourcing ROS Noetic.

To rerun on an isolated master (never use the real FCU master), use the PX4/Gazebo environment setup shown above and launch `roslaunch -p 11319 mine_uav_control task2_shaft_45m_px4_sitl.launch gui:=false rviz:=false`. Audit the local bag with:

```bash
source /opt/ros/noetic/setup.bash
python3 /home/nuc/frontier-upload/scripts/analyze_task2_sitl_bag.py \
  /home/nuc/task2_logs/probes/task2_45m_range_transition_20260918.bag \
  --bottom-top -44.85 --require-complete --require-ray-relay \
  --require-range-reacquisition --min-bottom-margin 1 \
  --max-xy-deviation 0.5 --max-range-alignment-error 0.1
```

The 45 m bag is local-only. This run verifies one additional idealized geometry beyond the 30 m sensor range; it does **not** establish a 400+ m PX4/Gazebo or real shaft safety case. Production Task 2 remains disabled.

## PX4 estimator Z-loss result

The missing PX4-Z failure case has now been injected in isolated 22 m SITL. It exposed bottom contact in the pre-guard run; a new SITL-only estimator-status gate reduces post-invalid raw setpoints from 80 to zero, but the guarded run became inconclusive for physical recovery after Gazebo slowed and MAVROS disconnected. **Task 2 remains unsafe without reliable PX4 Z and a validated independent recovery source.** Full chronology, A/B limitations, replay commands and the `SYS_FAILURE_EN` reset requirement are in [the PX4 Z-loss report](task2_px4_z_loss_report_2026-09-18.md).

## PX4 Z versus independent-depth consistency (follow-up)

The previous router checked PX4 estimator *valid flags* but not whether PX4's numerical Z agreed with the task's independent entrance-relative depth. A callback-level regression first reproduced this gap: an 8 m depth paired with PX4 Z indicating only 4 m change still published a descent setpoint. The SITL-only router now requires a fresh, correctly sourced, finite, quality-gated `ShaftDepthEstimate` and latches a pose/depth reference at its first active setpoint. It withdraws command readiness and requests its existing mode fallback if `abs((pose_z - reference_z) + (depth - reference_depth)) > 1.0 m`. It does not reset that reference until mission disable. This catches *disagreement*, not shared-mode errors or an untruthful source; it still depends on a physically valid depth sensor and PX4 navigation. The production router remains unimplemented and Task 2 remains disabled.

The new opt-in SITL adapter injection `depth_drift_after_depth:=6.0 depth_drift_rate_mps:=1.0` gradually biases the declared-good control-depth stream without altering Gazebo truth or PX4 pose. The previously passing normal 22 m flight was rerun with the gate and no injection: DESCENDING 17.325 s, RETURNING 59.376 s, COMPLETE 101.325 s, AUTO.LOITER 102.326 s; minimum body-outside bottom margin 1.411 m, maximum XY deviation 0.156 m, 3751/3751 forwarded ranges matched the raw ray. Bag: `/home/nuc/task2_logs/probes/task2_22m_depth_crosscheck_20260918.bag` (local only). The bag auditor was adjusted to accept the qualified depth topic when the redundant diagnostic scalar is absent.

In the opt-in drift flight, command-ready became true at 17.201 s and false at 33.101 s, just before the injected depth bias crossed 1 m at 33.249 s; last task setpoint was 33.050 s. There were **zero** later task setpoints, PX4 AUTO.LOITER was observed at 33.331 s, and 27.742 s of Gazebo observation after withdrawal showed 0.168 m additional descent. Bag: `/home/nuc/task2_logs/probes/task2_22m_depth_drift_guard_20260918.bag` (local only). `scripts/analyze_task2_depth_drift_bag.py` passes the drift bag and deliberately fails the normal bag (no injected bias). Router callback tests 12/12 and injection helper tests 2/2 pass. This proves timely withdrawal for this synthetic disagreement under *otherwise valid PX4 localization*, not a safe no-Z fallback or real-sensor performance.

For repeatable isolated SITL only, add the two injection arguments to the 22 m launch shown above; omit them for the normal path. Audit with `python3 scripts/analyze_task2_depth_drift_bag.py /home/nuc/task2_logs/probes/task2_22m_depth_drift_guard_20260918.bag`. The 11319 simulation was stopped afterward; the pre-existing 11312 master was not changed.
