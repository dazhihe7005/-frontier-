#!/usr/bin/env bash
set -euo pipefail

export ROS_MASTER_URI=http://127.0.0.1:11331
export ROS_HOSTNAME=127.0.0.1
export ROS_LOG_DIR=/home/nuc/gbplanner2_isolated_ws/runtime/logs/ros
mkdir -p "$ROS_LOG_DIR" /home/nuc/gbplanner2_isolated_ws/runtime/px4_sitl_2

# Keep the PX4 1018 definition inside this isolated repository.  The private
# PX4 root may have been created by an older run with a link into the
# frontier/SUPER workspace; replace only that private link on every launch.
px4_private_airframes=/home/nuc/gbplanner2_isolated_ws/runtime/px4_root/etc/init.d-posix/airframes
mkdir -p "$px4_private_airframes"
ln -sfn \
  /home/nuc/gbplanner2_isolated_ws/isolated_assets/px4_airframes/1018_gazebo-classic_iris_xy_vision \
  "$px4_private_airframes/1018_gazebo-classic_iris_xy_vision"

# PX4 persists calibration offsets and flight counters in its work directory.
# A failed or interrupted simulation must not contaminate the next cold start.
# An explicit px4_work_dir launch argument still takes precedence for debugging.
has_px4_work_dir=false
for launch_arg in "$@"; do
  if [[ "$launch_arg" == px4_work_dir:=* ]]; then
    has_px4_work_dir=true
    break
  fi
done
if [[ "$has_px4_work_dir" == false ]]; then
  run_stamp="$(date +%Y%m%d_%H%M%S)_$$"
  px4_run_dir="/home/nuc/gbplanner2_isolated_ws/runtime/px4_runs/$run_stamp"
  mkdir -p "$px4_run_dir"
  set -- "px4_work_dir:=$px4_run_dir" "$@"
fi

# The diagnostic reference is built from the collision meshes before the
# launch, never delivered to mapping, planning, estimation, or flight control.
coverage_requested=false
for launch_arg in "$@"; do
  if [[ "$launch_arg" == coverage_audit:=true ]]; then
    coverage_requested=true
    break
  fi
done
if [[ "$coverage_requested" == true ]]; then
  blender_bin=/home/nuc/.local/opt/blender-portable/blender-5.2.0-linux-x64/blender
  reference_dir=/home/nuc/gbplanner2_isolated_ws/runtime/map_reference
  mkdir -p "$reference_dir"
  "$blender_bin" --background --python \
    /home/nuc/gbplanner2_isolated_ws/tools/build_baixiangshan_reference.py -- \
    --resolution 0.5 \
    --output "$reference_dir/baixianshan_reachable_0p5m.json.gz"
  "$blender_bin" --background --python \
    /home/nuc/gbplanner2_isolated_ws/tools/build_baixiangshan_reference_3d.py -- \
    --resolution 0.5 \
    --output "$reference_dir/baixianshan_reachable_3d_0p5m.json.gz"
fi

source /opt/ros/noetic/setup.bash
source /home/nuc/fastlio2_ws/devel/setup.bash
source /home/nuc/gbplanner2_isolated_ws/devel/setup.bash --extend
: "${GAZEBO_PLUGIN_PATH:=}"
: "${GAZEBO_MODEL_PATH:=}"
: "${LD_LIBRARY_PATH:=}"
source /home/nuc/PX4-Autopilot/Tools/simulation/gazebo-classic/setup_gazebo.bash \
  /home/nuc/PX4-Autopilot \
  /home/nuc/PX4-Autopilot/build/px4_sitl_default
export ROS_PACKAGE_PATH=/home/nuc/PX4-Autopilot:"$ROS_PACKAGE_PATH"

exec roslaunch mine_uav_gbplanner gbplanner_baixiangshan_full_chain.launch "$@"
