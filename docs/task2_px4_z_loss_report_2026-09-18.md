# Task 2: PX4 Z-loss SITL investigation (2026-09-18)

## Conclusion

**Task 2 is not safe with no reliable PX4 Z estimate.** In an isolated 22 m PX4/Gazebo flight, accepted PX4 SITL failure commands disabled simulated barometer and GPS while descending. The PX4 vertical-position validity flag became false. In the pre-guard run, the vehicle eventually contacted the shaft bottom instead of returning to the entrance. The production task remains disabled (`shaft_task_available: false`). A new SITL-only estimator-status guard stops NUC descent commands much sooner, but it does **not** restore reliable Z, ensure hover, or demonstrate a safe landing. A real independent navigation source and a validated recovery policy are still required.

The failure command was MAVLink `MAV_CMD_INJECT_FAILURE` (420), unit 3 (BARO) and unit 4 (GPS), type 1 (OFF), with PX4 SITL `SYS_FAILURE_EN=1`. Both commands returned accepted result 0. These values are from the locally installed PX4 sources and were applied only on an isolated ROS master 11319 connected to PX4 SITL at `udp://:14540@localhost:14557`; the real master 11312 and real FCU were not used. Source: local PX4 `VehicleCommand.msg`, `SimulatorMavlink.cpp`, and `mavlink_receiver.cpp`.

| Evidence | Before estimator guard | With estimator guard |
| --- | ---: | ---: |
| Bag (local only) | `task2_22m_px4_baro_gps_failure_20260918.bag` | `task2_22m_px4_z_failure_guard_20260918.bag` |
| Failure injection | Manual, later in shaft | Automated at about 6 m depth |
| PX4 Z first invalid | 52.325 s | 34.316 s |
| NUC command-ready false after invalid | 4.025 s later | 0.034 s later |
| Raw local setpoints after invalid | 80 | 0 |
| PX4 mode after invalid | ALTCTL, then STABILIZED | AUTO.LOITER, then AUTO.LAND, then MAVROS disconnected |
| Minimum body-outside bottom margin observed after invalid | **−0.346 m** (floor contact/penetration in Gazebo) | +9.769 m by last world sample; **not a safe final outcome** |
| World observation after invalid | 44.437 s | 8.684 s; simulation lost real-time progress and later disconnected |

The injection depths differ, so the bottom margins must **not** be interpreted as a controlled before/after safety comparison. The causal improvement demonstrated is narrower: after PX4 itself marked Z invalid, the router's command withdrawal changed from 80 further setpoints to zero in the recorded flights. The unguarded bottom contact also shows why an OFFBOARD exit and `active_task=HOLD` do not equal safe return. In the guarded run, Gazebo real-time factor fell near zero around sim time 43 s, PX4 simulation reported repeated poll timeouts, and MAVROS disconnected. The precise cause of that simulation stall has not been established; no final landing or no-contact claim is made from that bag.

## Change and verification

The SITL-only `sitl_shaft_px4_router.py` now subscribes to `/mavros/estimator_status`. It requires a fresh estimator message, horizontal relative or absolute position valid, and vertical absolute or AGL position valid before taking or continuing ownership. On invalid estimator state while it owns OFFBOARD, it withdraws `/mine_uav/task2/command_ready`, stops raw setpoints (including untrustworthy position-hold targets), and requests the existing PX4 mode fallback. The latter request cannot guarantee a hover when PX4 Z is invalid. The failure was first reproduced by a callback test against the old router; the expanded router lifecycle suite passes 9/9 tests.

The normal post-change 22 m PX4/Gazebo flight still passed: DESCENDING 17.620 s, RETURNING 59.621 s, COMPLETE 101.520 s, AUTO.LOITER 102.525 s; minimum body-outside bottom clearance 1.412 m, maximum XY deviation 0.137 m, all 2235 forwarded range messages matched the Gazebo ray. Bag: `/home/nuc/task2_logs/probes/task2_22m_estimator_gate_normal_20260918.bag` (local only). This validates no observed normal-path regression, not real-flight safety.

The opt-in `inject_px4_z_failure:=true` launch node refuses to run unless `/use_sim_time=true`, `ROS_MASTER_URI` uses port 11319, and MAVROS FCU URL is exactly `udp://:14540@localhost:14557`; its 7 safety/cleanup unit tests pass. A first injector version enabled `SYS_FAILURE_EN=1` while waiting for the trigger, and an interrupted run left that setting across SITL restarts; it was explicitly reset to 0 and read back. The current injector now performs the preflight parameter check while keeping permission **off**, enables it only upon reaching the trigger depth, and resets it in a `finally` block even if a failure command is rejected. A short isolated SITL retest confirmed `SITL_FAILURE_INJECTION_PREPARED` with `SYS_FAILURE_EN=0` before the trigger, then `INJECTED_BARO_GPS_OFF_PERMISSION_DISABLED` and `SYS_FAILURE_EN=0` after accepted commands. A process crash during the brief enable/disable window could still leave it at 1: check and reset the parameter before another run. The real FCU was never modified.

Re-audit the two local bags:

```bash
source /opt/ros/noetic/setup.bash
python3 /home/nuc/frontier-upload/scripts/analyze_task2_px4_z_failure_bag.py \
  /home/nuc/task2_logs/probes/task2_22m_px4_baro_gps_failure_20260918.bag \
  --max-ready-withdraw-delay 0.2 --max-post-invalid-setpoints 0
# The old-code bag must fail these early-withdrawal limits.
python3 /home/nuc/frontier-upload/scripts/analyze_task2_px4_z_failure_bag.py \
  /home/nuc/task2_logs/probes/task2_22m_px4_z_failure_guard_20260918.bag \
  --require-injection --max-ready-withdraw-delay 0.2 \
  --max-post-invalid-setpoints 0
# Passing means prompt withdrawal only; the analyzer never claims safe recovery.
```

To reproduce the failure injection, use the isolated SITL setup documented in the main Task 2 report, then launch `roslaunch -p 11319 mine_uav_control task2_shaft_22m_px4_sitl.launch gui:=false rviz:=false inject_px4_z_failure:=true px4_z_failure_after_depth:=6.0`. Do not use this on the real master or a real FCU. Do not enable production Task 2 based on these tests. The essential unsolved requirement is an independently trustworthy position/height solution and a physically validated action for its failure at depth; the current fallback has demonstrated no safe no-Z recovery.
