# Task 2: shaft mission logic prototype (2026-09-18)

Status: **isolated 22 m PX4/Gazebo interface flight passed**, using ideal Gazebo world-truth depth/range; no physical sensor or degraded-Z validation. Task 2 remains disabled in production `config/mission_scheduler.yaml` (`shaft_task_available: false`). The generic mission node publishes `/mine_uav/shaft/velocity_intent_enu`; only the explicitly named SITL launch connects this to MAVROS and arms a simulated vehicle.

## Control contract

- `/mine_uav/mission/shaft_enable` (`std_msgs/Bool`) enables a mission. False resets to IDLE; a new true edge recaptures the entrance depth.
- `/mine_uav/shaft/relative_depth_m` (`std_msgs/Float64`) must come from a separately validated depth source; positive is downward. It is **not** supplied by the downward-facing bottom rangefinder. The ROS node currently checks freshness and value finiteness, not whether the producer is physically independent or accurate.
- `/mine_uav/shaft/bottom_range` (`sensor_msgs/Range`) is downward distance. Positive infinity means no target within range. Freshness, sensor min/max and debounce are checked.
- `/mine_uav/shaft/velocity_intent_enu` (`geometry_msgs/TwistStamped`) contains only an ENU Z velocity. Descend is negative, return is positive. The separate SITL-only router holds XY and handles a single MAVROS setpoint stream. Production XY control, yaw, corridor clearance, PX4 mode and setpoint arbitration are **not implemented**.
- `/mine_uav/shaft/status` reports IDLE, DESCENDING, RETURNING, COMPLETE or FAULT_NO_SAFE_AUTONOMOUS_RECOVERY. A fault suppresses intent; it does **not** guarantee the vehicle can hover, stop or return.

The algorithm records entrance depth on activation, descends with speed reduced according to bottom stopping distance, requires 0.3 s consecutive bottom detections, then returns toward the recorded entrance depth. Stale input, a 2 m depth jump, the 450 m depth limit, or 1800 s time limit enters FAULT. Invalid parameters fail before motion. This is a state-machine contract, not a flight safety case.

## Evidence

`catkin_make -C /home/nuc/super_ws shaft_mission_node test_shaft_mission` compiled. Six GTests pass, including an **idealized** 420 m kinematic loop that descended until the bottom became visible, triggered return and reached the starting depth. A ROS-topic integration test also passes: enable→descent→confirmed bottom→return→complete, then stale-range reset→fault. The first integration attempt correctly faulted because the test injected an impossible 0→5 m depth jump; after correcting that test input and rebuilding the node, the test passed. The model assumes perfect independent depth and bottom range measurements; it does not represent PX4 dynamics, barometer drift, sensor dropouts over hundreds of metres, walls, airflow, communications loss or battery reserves. The 22 m PX4/Gazebo probe below extends interface coverage but still does not address these real-world conditions.

The COMPLETE and FAULT states now remain latched even if sensors subsequently stop, until the enable signal goes low. This avoids turning an already-completed mission into a spurious post-completion fault. The node publishes no velocity intent in either terminal state.

## Next validation gates

1. Select and independently validate a real depth/entrance-reference source. A bottom-only laser cannot locate the starting elevation for return; commanded speed integrated over time is insufficient as a safety-critical substitute.
2. Add a source-quality/uncertainty contract and a way to handle loss at depth. Currently FAULT has no guaranteed safe autonomous recovery.
3. Replace the synthetic Gazebo-truth range with a Gazebo ray sensor, deliberately degrade localization, then verify manual takeover, collision margin, return and failsafes at multiple shaft depths including 400+ m. Keep real-flight selection disabled until these pass.

## 22 m PX4/Gazebo interface probe (same day)

A separate simulation launch, `launch/task2_shaft_22m_px4_sitl.launch`, now runs PX4 SITL, a 10×10 m shaft of about 22 m depth, the existing simulated takeoff operator, a temporary entrance platform, the scheduler with **SITL-only** `shaft_task_available=true`, the shaft state machine, and a **SITL-only** single-owner MAVROS router. The router holds the captured XY position while sending vertical velocity, does not request OFFBOARD, and requests AUTO.LOITER on completion/fault. If the pilot/PX4 leaves OFFBOARD externally, it latches the takeover until enable goes low. Production `mission_scheduler.yaml` remains disabled for Task 2.

Two separate root causes were exposed before obtaining a passing flight:

1. Without an entrance platform, the disarmed Gazebo vehicle fell to the shaft bottom before the simulated pilot armed it. The first run therefore took off from the bottom and cannot be interpreted as a shaft mission. A named temporary platform now supports initial takeoff and is deleted only after an armed stable hover; the mission waits without motion for valid sensor frames after enable.
2. With that platform, the first full 22 m closed loop still had only **0.504 m** minimum body-outside bottom clearance (0.4 m equivalent radius). The synthetic range was generated from PX4 odometry whose origin was established after the vehicle settled on the platform, but its nominal bottom depth was fixed relative to the earlier spawn position. At the physical lowest point, sensor range said **1.804 m** while actual center-to-floor distance was **0.904 m**. Across the flight the maximum range/world discrepancy was **1.086 m**. This constant-frame error, not a proved PX4 braking failure, explains the missed 1 m clearance requirement. That failure bag is `/home/nuc/task2_logs/probes/task2_22m_pad_px4_sitl_20260918.bag`.

The adapter now uses Gazebo world truth to generate idealized, internally aligned depth and bottom range, and publishes nothing while the entrance pad is present. The same 22 m flight then reached RETURNING at 60.109 s and COMPLETE at 102.159 s, followed by PX4 AUTO.LOITER at 102.520 s; no task fault or in-task disarm. Minimum body-outside bottom clearance was **1.373 m**, minimum side-wall clearance **4.465 m**, and maximum XY deviation from the initial active position **0.142 m**. The maximum range/world disagreement fell to **0.032 m**. The pass bag is `/home/nuc/task2_logs/probes/task2_22m_world_truth_px4_sitl_20260918.bag`. Both bags are local evidence, not in GitHub. A later small router change latches external OFFBOARD takeover and has syntax/launch validation but **has not been rerun in PX4 SITL**.

To re-audit the passing bag:

```bash
source /opt/ros/noetic/setup.bash
python3 /home/nuc/frontier-upload/scripts/analyze_task2_sitl_bag.py \
  /home/nuc/task2_logs/probes/task2_22m_world_truth_px4_sitl_20260918.bag \
  --require-complete --min-bottom-margin 1 \
  --max-xy-deviation 0.5 --max-range-alignment-error 0.1
```

The preceding bag fails both bottom-margin and range-alignment limits with the same command. This is a PX4/Gazebo **interface** result, not a real 420 m shaft validation: the downward range is calculated from Gazebo truth, depth is ideal world truth, PX4 still has standard SITL localization, side walls are wide, and no real lidar or loss-of-Z/failsafe behavior is exercised. The actual shaft task still requires an independent depth sensor and verified recovery under sensor failures.
