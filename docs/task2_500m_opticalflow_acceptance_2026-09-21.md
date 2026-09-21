# Task 2 — 500 m side-optical-flow depth PX4/Gazebo acceptance (2026-09-21)

Status: **nominal isolated PX4/Gazebo SITL passed; this is not a real-flight safety case.**

## Test contract

- Shaft depth: 500 m.
- Task depth source: `sitl_side_optical_flow_integrated`, a SITL-only frame-to-frame displacement integration surrogate.
- Downward rangefinder: 0.30–8.00 m only; it is a bottom trigger, not the long-range depth source.
- Bottom braking threshold: 5.0 m with 0.3 s confirmation.
- Descent / return speed: 0.5 m/s.
- Vertical command acceleration limit: 0.5 m/s².
- 500 m launch overrides: `max_depth=505 m`, `max_duration=2400 s`.
- PX4 SITL airframe: `iris_xy_vision`; external vision is fused for XY only, GPS is disabled.
- Task router uses `require_px4_vertical_position=false` and `require_px4_depth_agreement=false`.

PX4 still retains its internal barometer-based vertical estimator in the nominal acceptance run. Therefore this test proves that **Task 2 mission depth and return logic do not consume PX4 absolute Z**; it does not prove that PX4 can control vertical velocity after losing every vertical estimator.

## Reproducible evidence

Short regression bag:
`/home/nuc/task2_logs/acceptance_20260921/task2_22m_latest_xyvision_20260921.bag`

500 m nominal bag:
`/home/nuc/task2_logs/acceptance_20260921/task2_500m_latest_xyvision_20260921.bag`

The 500 m bag is 15.4 MB, 985 s long, and contains the mission state, command-ready gate, independent depth, 0.3–8 m range, vertical intent, PX4 state/estimator status, and a 2 Hz Gazebo world-truth audit stream.

## 22 m latest-code regression

The final 22 m run followed:

`IDLE -> DESCENDING -> RETURNING -> COMPLETE -> IDLE`

and PX4 changed from OFFBOARD to AUTO.LOITER immediately after completion.

Measured values:
- DESCENDING: +7.552 s.
- RETURNING: +28.147 s.
- COMPLETE: +48.394 s.
- AUTO.LOITER: +48.431 s.
- Maximum integrated depth: 18.118 m.
- Minimum downward range: 4.673 m.
- Maximum commanded vertical acceleration: 0.500 m/s².
- No command-acceleration sample exceeded 0.505 m/s².

## 500 m nominal flight result

State timing relative to the first recorded IDLE:
- DESCENDING: +6.026 s.
- RETURNING: +464.370 s.
- COMPLETE: +922.150 s.
- IDLE: +922.192 s.
- PX4 AUTO.LOITER: +922.552 s.
- `command_ready` becomes true at +6.033 s and false at +922.159 s.

The input gate is OPEN throughout active descent and return. The post-completion STALE_DEPTH indication occurs only after the task has already completed and the depth producer is disabled.

### Bottom approach

- Maximum integrated side-flow depth: 498.027 m.
- Range first re-enters the 8 m sensor window at 7.986 m, near integrated depth 493.664 m.
- First range <= 5.0 m: 4.946 m, near integrated depth 496.733 m.
- Minimum measured downward range: 3.646 m.
- Gazebo world-truth minimum bottom clearance: 3.657 m.
- World-truth maximum displacement from the captured task-start hover: 498.077 m.

### Acceleration-limited reversal

The recorded vertical intent changes smoothly from -0.5 m/s through zero to +0.5 m/s.

- Maximum observed command acceleration: 0.500000000001 m/s².
- 99th percentile of non-zero command acceleration: 0.5 m/s².
- Number of command changes above 0.505 m/s²: 0 / 60.

The recorded sequence around reversal uses approximately 0.05 s control intervals and every non-zero speed change respects the 0.5 m/s² limit.

### Return accuracy

Using the 2 Hz Gazebo audit stream:
- Horizontal return error relative to the captured DESCENDING start hover: 0.016 m.
- Vertical return error relative to that hover: 0.524 m.
- 3D return error: 0.524 m.
- Final integrated depth before completion: 0.416 m.

This is consistent with the mission's 0.5 m entrance-depth tolerance plus sampling/dynamics error. The task returns to the **captured airborne entrance hover**, not to the geometric shaft-mouth plane.

## Current automated regression

Final package regression:
- 94 tests.
- 0 errors.
- 0 failures.
- 0 skipped.

Task-2-specific direct checks also pass:
- `ShaftMission`: 9/9.
- control-dt fallback: 3/3.
- optical-flow integration surrogate: 4/4.
- PX4 router lifecycle: 13/13.
- XY-only SITL vision isolation: 4/4.

The control-dt fallback explicitly handles repeated simulation timestamps by using the expected timer period instead of injecting a zero-dt fault.

## Critical limitation — integrated-depth drift is unobservable

A deterministic 500 m round-trip simulation using the current `ShaftMission` shows that a persistent additive bias in the side-flow depth integrator can make the state machine report COMPLETE while the vehicle is physically displaced from the captured start altitude.

Synthetic constant bias results:
- 0.0000 m/s bias -> 0.488 m physical return error.
- 0.0005 m/s bias -> 0.513 m physical return error.
- 0.0010 m/s bias -> 1.488 m physical return error.
- 0.0020 m/s bias -> 3.488 m physical return error.

These are fault-model values, not measured hardware performance. They demonstrate the architectural limitation: with no independent long-range Z reference, a persistent integration bias cannot be detected by the mission state machine itself.

Before real 500 m operation, the side optical-flow implementation therefore needs hardware characterization and an explicit bound on scale/bias/dropout error, or an independent entrance-reference reacquisition mechanism near return.

## What this acceptance does and does not prove

Passed:
- 0.3–8 m downward range is sufficient as a near-bottom trigger in the nominal 500 m SITL.
- The mission can use side-flow integrated relative depth for the long descent/return state logic.
- The 5 m trigger begins acceleration-limited braking and maintains >3.6 m bottom clearance in this SITL.
- Return remains in OFFBOARD until COMPLETE, then command ownership is released and PX4 enters AUTO.LOITER.
- Task-level logic does not require PX4 absolute Z or PX4/depth agreement.

Not proved:
- Real side optical-flow accuracy over a 1000 m round trip.
- Real downward rangefinder validity; the current real STP23 probe still reports no valid points.
- Safe flight if PX4 loses all vertical-velocity/height estimation.
- Real shaft aerodynamics, wall clearance, battery reserve, radio loss, or optical texture robustness.
- A real 500 m flight safety case.

Production `shaft_task_available=false` must remain unchanged until the real depth source, rangefinder, production PX4 router/control path, and recovery behavior are validated.
