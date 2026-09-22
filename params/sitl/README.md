# PX4 SITL parameter migration

Source: `params/real_snapshots/px4_real_2026-09-21_qgc.params` (1102
read-only exported parameters).

The dedicated SITL airframe
`px4_airframes/1018_gazebo-classic_iris_xy_vision` activates 76 same-name
parameters covering multicopter attitude/rate control, position/velocity
control, limits and IMU/control filters. It then applies simulation-specific
overrides for the 6S parallel battery, no-GPS operation and FAST-LIO2 external
vision.

Six real-PX4 candidates are not present in this checkout's SITL parameter
metadata and are therefore not loaded:

- `MC_YAW_TQ_CUTOFF`
- `MPC_ACC_DECOUPLE`
- `MPC_VEL_LP`
- `MPC_VEL_NF_BW`
- `MPC_VEL_NF_FRQ`
- `MPC_YAWRAUTO_ACC`

All other parameters remain snapshot-only unless individually reviewed.
Notably excluded are hardware IDs/calibration, board/sensor configuration,
serial and MAVLink routing, PWM/DSHOT outputs, RC mapping, driver/bus settings,
real control allocation geometry, and temporary test-battery values. EKF2
source-selection parameters are not bulk-copied: the SITL airframe explicitly
sets GPS off and external-vision XY on so simulation does not inherit a
hardware-specific estimator topology.
