#!/usr/bin/env bash
# Start the isolated Task-4 Bai Xiang Shan PX4 -> MAVROS -> Omni-LIVO -> SUPER SITL.
# The scenario auto-arms and flies after its sensor and takeoff checks.
set -eo pipefail

source /opt/ros/noetic/setup.bash
source /home/nuc/omni_livo_fisheye_ws/devel/setup.bash --extend
source /home/nuc/super_ws/devel/setup.bash --extend
source /home/nuc/frontier_runtime_ws/devel/setup.bash --extend
source /home/nuc/PX4-Autopilot/Tools/simulation/gazebo-classic/setup_gazebo.bash \
  /home/nuc/PX4-Autopilot /home/nuc/PX4-Autopilot/build/px4_sitl_default

# rospack searches the parent directory of a package, not the package itself.
export ROS_PACKAGE_PATH="/home/nuc/PX4-Autopilot:/home/nuc/PX4-Autopilot/Tools/simulation/gazebo-classic:${ROS_PACKAGE_PATH}"
export ROS_MASTER_URI=http://localhost:11319
export GAZEBO_MASTER_URI=http://localhost:11499

for package in mine_uav_control mavlink_sitl_gazebo mavros super_planner omni_livo; do
  rospack find "$package" >/dev/null
done

if [[ "${1:-}" == "--check" ]]; then
  printf 'Bai Xiang Shan SITL resources found; ROS master %s; Gazebo master %s\n' \
    "$ROS_MASTER_URI" "$GAZEBO_MASTER_URI"
  exit 0
fi

exec roslaunch -p 11319 mine_uav_control \
  task4_baixiangshan_px4_sitl.launch gui:=true rviz:=true "$@"
