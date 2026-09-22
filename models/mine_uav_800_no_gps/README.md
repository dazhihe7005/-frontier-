# Shared STEP-derived no-GPS SITL base

This model represents the currently described aircraft.

Known inputs:

- total takeoff mass: approximately 6.0 kg, including the LiDAR;
- STEP-measured adjacent motor spacing: 439.002 mm;
- STEP-measured diagonal motor-to-motor wheelbase: 620.843 mm;
- the previously reported 800 mm is retained as the frame class/overall-size
  description, not used as the motor-centre wheelbase;
- body frame: right-handed FLU (`+X` forward, `+Y` left, `+Z` up);
- localization sensor: Livox MID360S, no GPS;
- MID360S pose in FLU: `[0.1315, 0, 0.223]` m with +25 deg pitch about
  `+Y` (forward axis tilted toward `-Z`);
- motors: four T-Motor U7 KV420, 12N14P;
- motor resistance: 33 mOhm;
- motor dimensions: 60.7 x 39.5 mm, 6 mm shaft;
- motor cable: 750 mm, 16 AWG;
- motor mass: 255 g without cable, 296 g including cable;
- no-load current: 0.9 A;
- supported supply: 3-8S;
- user-supplied motor limits: 40 A maximum, 350 W continuous;
- propellers: P15x5 inch carbon fibre;
- supply: two DJI BPX230-6768-22.14 6768 mAh Li-ion 6S packs in parallel
  (22.14 V nominal, 25.5 V full, 13536 mAh and 299.8 Wh total);
- battery chemistry: LiNiMnCoO2/NMC, 4C, 640 g per pack, 154 x 96 x 59 mm;
- ESC: LANRC 60A 4-in-1, BLHeli_S, DSHOT600, 2-6S, 60 A continuous per channel.

Mass allocation:

- body, frame, avionics and ESC excluding batteries: 3.256 kg;
- two battery packs: 2 x 0.640 kg = 1.280 kg;
- four U7 motors including cables: 4 x 0.296 kg;
- simulated IMU link: 0.015 kg;
- MID360S: 0.265 kg;
- total: 6.000 kg.

Gazebo's motor plugin requires angular-speed, thrust and reaction-torque
coefficients rather than KV, current and power limits. Until a P15x5
thrust/RPM/current table is available, the following explicit approximation is
used:

- 6S nominal voltage: 22.14 V;
- no-load speed ceiling: 420 KV x 22.14 V = 9298.8 rpm = 973.768 rad/s;
- peak static thrust: the user-supplied 4.64 kgf per motor;
- `motorConstant=4.79874427e-05 N/(rad/s)^2`;
- `momentConstant=0.00789903182 m`, derived by treating 350 W as the
  continuous shaft-power point at the speed ceiling.

This gives 18.56 kgf total peak static thrust, a theoretical 3.09
thrust-to-weight ratio, and about 5287 rpm ideal hover speed at 6 kg. These are
not a substitute for a measured propeller curve. In particular, 40 A at
22.14 V is 885.6 W electrical input, so the supplied 40 A maximum and 350 W continuous
ratings are treated as separate peak-current and continuous-power constraints.

Gazebo Classic does not electrically couple motor load, ESC current, battery
energy or thermal limits. The 60 A ESC rating and motor electrical limits are
documented but are not enforced by the aerodynamic plugin. The PX4 SITL
airframe is configured with `BAT1_N_CELLS=6`, `BAT1_CAPACITY=13536` and
`BAT1_V_CHARGED=4.25`.
Capacity alone does not reproduce load-dependent current, voltage sag, thermal
limits, or trustworthy endurance. Those still require pack internal
resistance/discharge data and a motor/prop bench curve.

The authoritative machine-readable inputs are in
`config/mine_uav_800_no_gps.yaml`. The supplied `【二代】总装.STEP` is a
SolidWorks 2025 STEP AP203 assembly in millimetres. Its four U7 centres and
native bounding box were read directly, with native axes mapped to FLU as
`[X, -Z, Y]`. The high-detail assembly is used as the source for visual
geometry; real-time collision remains a simplified deterministic model. The
aircraft centre of mass is currently treated as the body origin because it is
reported to be near the body centre.

`meshes/uav_body_step_visual.stl` is a 101750-triangle binary STL generated
from the exterior body/frame/battery/landing-structure subset of the assembly.
The STEP FOV helper solid, internal details, motors, propellers and lidar were
excluded: motors/propellers remain dynamic rotor-link visuals and the lidar is
owned by the MID360S wrapper. The raw STL remains in millimetres and the SDF
applies the documented millimetre-to-metre scale and native-to-FLU transform.
