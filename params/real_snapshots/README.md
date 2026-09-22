# Real PX4 read-only snapshots

`px4_real_2026-09-21_qgc.params` is an unchanged QGroundControl-format export
of the 1102 parameters read from PX4 system/component `1/1` over the verified
CH340/TELEM2 link at 500000 baud. Its SHA-256 is
`544ca9ca9f9b5748855f18cfc2b3a387b87e4a656d70800f74757945b5da7f9b`.

The export operation only requested parameters. It did not arm the aircraft,
change mode, or call a parameter-set service.

Do not load this whole file into SITL or back into the aircraft. It contains
hardware IDs, calibrations, serial/driver settings and values for the temporary
test battery. In particular, its battery values are not the simulation battery
specification. The canonical simulation values are in
`config/mine_uav_800_no_gps.yaml`.
