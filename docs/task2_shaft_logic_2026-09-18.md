# Task 2: shaft mission logic prototype (2026-09-18)

Status: **unit-test and ROS build only**, not PX4/Gazebo flight validation. Task 2 remains disabled in `config/mission_scheduler.yaml` (`shaft_task_available: false`). The node publishes `/mine_uav/shaft/velocity_intent_enu` and **nothing is connected to MAVROS**. It will not arm or move a vehicle.

## Control contract

- `/mine_uav/mission/shaft_enable` (`std_msgs/Bool`) enables a mission. False resets to IDLE; a new true edge recaptures the entrance depth.
- `/mine_uav/shaft/relative_depth_m` (`std_msgs/Float64`) must come from a separately validated depth source; positive is downward. It is **not** supplied by the downward-facing bottom rangefinder. The ROS node currently checks freshness and value finiteness, not whether the producer is physically independent or accurate.
- `/mine_uav/shaft/bottom_range` (`sensor_msgs/Range`) is downward distance. Positive infinity means no target within range. Freshness, sensor min/max and debounce are checked.
- `/mine_uav/shaft/velocity_intent_enu` (`geometry_msgs/TwistStamped`) contains only an ENU Z velocity. Descend is negative, return is positive. XY control, yaw, corridor clearance, PX4 mode and setpoint arbitration are **not implemented**.
- `/mine_uav/shaft/status` reports IDLE, DESCENDING, RETURNING, COMPLETE or FAULT_NO_SAFE_AUTONOMOUS_RECOVERY. A fault suppresses intent; it does **not** guarantee the vehicle can hover, stop or return.

The algorithm records entrance depth on activation, descends with speed reduced according to bottom stopping distance, requires 0.3 s consecutive bottom detections, then returns toward the recorded entrance depth. Stale input, a 2 m depth jump, the 450 m depth limit, or 1800 s time limit enters FAULT. Invalid parameters fail before motion. This is a state-machine contract, not a flight safety case.

## Evidence

`catkin_make -C /home/nuc/super_ws shaft_mission_node test_shaft_mission` compiled. Six GTests pass, including an **idealized** 420 m kinematic loop that descended until the bottom became visible, triggered return and reached the starting depth. A ROS-topic integration test also passes: enable→descent→confirmed bottom→return→complete, then stale-range reset→fault. The first integration attempt correctly faulted because the test injected an impossible 0→5 m depth jump; after correcting that test input and rebuilding the node, the test passed. The model assumes perfect independent depth and bottom range measurements; it does not represent PX4 dynamics, barometer drift, sensor dropouts over hundreds of metres, walls, airflow, communications loss or battery reserves. No Gazebo shaft flight has passed.

The COMPLETE and FAULT states now remain latched even if sensors subsequently stop, until the enable signal goes low. This avoids turning an already-completed mission into a spurious post-completion fault. The node publishes no velocity intent in either terminal state.

## Next validation gates

1. Select and independently validate a real depth/entrance-reference source. A bottom-only laser cannot locate the starting elevation for return; commanded speed integrated over time is insufficient as a safety-critical substitute.
2. Add a source-quality/uncertainty contract and a way to handle loss at depth. Currently FAULT has no guaranteed safe autonomous recovery.
3. Build a Gazebo shaft with bottom laser and deliberately degraded localization, then verify MAVROS routing, XY hold, offboard handover, manual takeover, collision margin and return in multiple shaft depths. Keep real-flight selection disabled until these pass.
