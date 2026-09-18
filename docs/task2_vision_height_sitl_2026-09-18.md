# Task 2 PX4 vision-height SITL experiment (2026-09-18)

## Outcome and limits

**Only in isolated PX4/Gazebo SITL**, a synthetic full external-vision pose from Gazebo world truth was fused by PX4 EKF2 as its height reference. A 22 m shaft mission completed after the simulator's GPS and barometer were switched off. This demonstrates that the NUC→MAVROS→PX4 external-vision *software path* can supply Z when a trustworthy full pose exists. It does not identify, calibrate, or validate such a source on the aircraft, nor prove a safe no-source recovery in a 400+ m shaft. Production `shaft_task_available: false` is unchanged.

The SITL-only publisher `scripts/sitl_shaft_vision_truth.py` sends `/gazebo/model_states` pose as `/mavros/vision_pose/pose_cov` at about 30 Hz. Its guard requires ROS master port 11319, `/use_sim_time=true`, and the exact local UDP FCU URL `udp://:14540@localhost:14557`. The 0.05 m position and 0.10 rad attitude standard deviations are declared simulation values, **not** real sensor accuracy. `stop_vision_after_depth` deliberately stops the stream for one failure test. This publisher must never run on the real 11312 master.

PX4's local `1013_gazebo-classic_iris_vision` SITL airframe had an immediate startup defect: it sourced nonexistent `10016_gazebo-classic_iris`; the base file here is `10015_gazebo-classic_iris`. The local PX4 source and generated SITL copy were changed to that reference. The stock external-yaw setup produced `Preflight Fail: Yaw estimate error`; changing **both** `EKF2_EV_CTRL` to 3 (vision XY+Z, no vision yaw/velocity) and `ATT_EXT_HDG_M` to 0 allowed arming. This experiment does not isolate which of those two yaw settings caused the original error. The local airframe source and built SITL copy were set to these defaults, and a cold-start flight then armed automatically. The repository includes the intended SITL-only file under `px4_airframes/`; that file is **not automatically installed** into another PX4 checkout. Verify its base-airframe numbering against the local PX4 version before applying it. Do not install it on a real FCU.

The clean-start PX4 values read through MAVROS were `EKF2_HGT_REF=3`, `EKF2_EV_CTRL=3`, `EKF2_GPS_CTRL=0`, `EKF2_BARO_CTRL=1`, `ATT_EXT_HDG_M=0`; `SYS_FAILURE_EN=0` both before injection and after its short authorization window. The earlier normal bag (`task2_22m_px4_vision_truth_20260918.bag`) includes a **mid-run manual** yaw-parameter correction and is not the clean-start parameter regression; it nevertheless reached COMPLETE with 1.380 m minimum body-outside bottom margin.

## Clean-start barometer/GPS failure flight

To repeat on the **isolated SITL master only**, first verify that the local PX4 SITL airframe matches `px4_airframes/1013_gazebo-classic_iris_vision` (the project does not install it automatically), then run:

```bash
source /opt/ros/noetic/setup.bash
source /home/nuc/super_ws/devel/setup.bash
source /home/nuc/PX4-Autopilot/Tools/simulation/gazebo-classic/setup_gazebo.bash \
  /home/nuc/PX4-Autopilot /home/nuc/PX4-Autopilot/build/px4_sitl_default
export ROS_MASTER_URI=http://localhost:11319
export GAZEBO_MASTER_URI=http://localhost:11499
export ROS_PACKAGE_PATH=/home/nuc/PX4-Autopilot:/home/nuc/PX4-Autopilot/Tools/simulation/gazebo-classic:${ROS_PACKAGE_PATH}
roslaunch -p 11319 mine_uav_control task2_shaft_22m_vision_px4_sitl.launch \
  gui:=false inject_baro_gps_failure:=true
```

Do not run that launch or its fault injector against a real FCU. The all-height-loss variant additionally sets `stop_vision_after_depth:=8.0`; it is a failure test, not an operational mode.

`/home/nuc/task2_logs/probes/task2_22m_px4_vision_baro_gps_loss_20260918.bag` and PX4 ULog `/home/nuc/.ros/log/2026-09-18/12_31_18.ulg` are **local only**. The SITL failure injector reported barometer+GPS OFF accepted at 29.118 s, after descent began at 14.375 s. Task state changed to RETURNING at 55.474 s and COMPLETE at 95.974 s; PX4 entered AUTO.LOITER at 96.330 s. Minimum body-outside bottom clearance was 1.430 m, maximum XY deviation 0.070 m, and 2480/2480 task range messages matched the raw Gazebo ray. During the 66.856 s from injection to completion, 1790 vision messages and 67 PX4 estimator status messages were recorded; zero post-injection status messages marked PX4 Z invalid. PX4 ULog has `cs_ev_hgt=1` for 144/145 sampled flags and `cs_ev_pos=1` for 139/145; `cs_baro_hgt` falls to 0 after the injected barometer failure and `cs_gps` remains 0. The last flag sample has vision height fused, baro and GPS not fused. This is stronger evidence of EKF2 participation than merely observing a MAVROS input topic.

Recheck the same local evidence with `scripts/analyze_task2_sitl_bag.py --vehicle-model iris_vision --require-complete --require-ray-relay --min-bottom-margin 1 --max-xy-deviation 0.5 --max-range-alignment-error 0.1` and `scripts/analyze_task2_vision_fusion.py BAG ULOG`. The second auditor deliberately rejects the earlier no-injection bag. The first auditor now accepts a configurable Gazebo vehicle model name; the default remains `iris` for all older bags.

## All-height-source loss

One further isolated run used `inject_baro_gps_failure:=true stop_vision_after_depth:=8.0`. In `/home/nuc/task2_logs/probes/task2_22m_px4_vision_loss_after_baro_20260918.bag`, barometer+GPS OFF was accepted at 29.404 s; the synthetic vision stream stopped at 33.199 s; PX4 Z first became invalid at 34.527 s. The NUC command-ready flag went false 0.023 s later, with zero raw local setpoints after the invalid-Z observation; PX4 left OFFBOARD about 1.0 s after invalid. The simulated environment then slowed/stalled, MAVROS disconnected, and only 3.932 s of world truth after invalid Z was observed. The last observed body-outside bottom margin was 11.397 m; **that is not a final outcome or proof of safe landing/return**. The existing Z-loss auditor now accepts this scenario's `iris_vision` model and checks a true-to-false vision stream transition. This run reinforces the physical blocker: once all trustworthy height sources are gone, withdrawal/fallback does not provide a validated recovery trajectory.

## What still blocks production

1. Choose and independently calibrate an actual entrance-referenced depth/full-position source that remains valid over 400+ m, including timing, extrinsics, drift and failure detection. Gazebo truth is not a design for that hardware. The downward bottom laser cannot supply whole-shaft entrance depth by itself.
2. Validate PX4 XY/yaw control, obstacle clearance, communication and power margins, plus a physically feasible fallback when the final trusted height source fails. No source-independent autonomous hover/return has been demonstrated.
3. Integrate a production single-owner command router and verify real CH11 selection/CH5 takeover, actual laser data, EKF2 settings on the real FCU only after source validation, then disarmed/propeller-off and staged flight tests. Production Task 2 stays disabled.

Official PX4 source for the EKF2 vision-height configuration: [external position estimation](https://docs.px4.io/main/en/ros/external_position_estimation) and [EKF2 height sources](https://docs.px4.io/main/en/advanced_config/tuning_the_ecl_ekf).
