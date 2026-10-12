# NUC SUPER/ROG-Map snapshot

This branch contains the path-planning libraries used by the NUC project on
2026-10-12. It is a source-only snapshot of `super_planner`, `rog_map` and the
required `quadrotor_msgs` package; simulation assets, generated logs, build
products and the upstream Git history are intentionally excluded.

The matching mission/frontier/PX4 integration is on the repository's `main`
branch. The snapshot was independently compiled with ROS Noetic and tested
before upload:

- `rog_map`: 8 tests passed.
- `super_planner`: 114 tests passed.
- Matching `mine_uav_control`: 217 tests passed in an isolated workspace.

The code is derived from `hku-mars/SUPER` commit `2ad3419` plus the NUC-local
planning, lifecycle, clearance, height-contract and regression-test changes.
