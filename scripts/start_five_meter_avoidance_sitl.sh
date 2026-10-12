#!/usr/bin/env bash
set -euo pipefail

# Use an isolated ROS/Gazebo master. This script starts and arms PX4 SITL only;
# it never connects to the real flight controller.
source /opt/ros/noetic/setup.bash
source /home/nuc/frontier_runtime_ws/devel/setup.bash

export CMAKE_PREFIX_PATH=/home/nuc/frontier_runtime_ws/devel:/home/nuc/super_ws/devel:/home/nuc/fastlio2_ws/devel:/opt/ros/noetic
export ROS_PACKAGE_PATH=/home/nuc/frontier_runtime_ws/src:/home/nuc/super_ws/src/SUPER:/home/nuc/fastlio2_ws/src:/home/nuc/PX4-Autopilot:/home/nuc/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic:/opt/ros/noetic/share
export PYTHONPATH=/home/nuc/frontier_runtime_ws/devel/lib/python3/dist-packages:/home/nuc/super_ws/devel/lib/python3/dist-packages:/home/nuc/fastlio2_ws/devel/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages
export GAZEBO_MODEL_PATH=/home/nuc/frontier-upload/models:/home/nuc/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models:${GAZEBO_MODEL_PATH:-}
export GAZEBO_PLUGIN_PATH=/home/nuc/frontier_runtime_ws/devel/lib:/home/nuc/super_ws/devel/lib:/home/nuc/PX4-Autopilot/build/px4_sitl_default/build_gazebo-classic:${GAZEBO_PLUGIN_PATH:-}
export LD_LIBRARY_PATH=/home/nuc/frontier_runtime_ws/devel/lib:/home/nuc/super_ws/devel/lib:/home/nuc/fastlio2_ws/devel/lib:/home/nuc/PX4-Autopilot/build/px4_sitl_default/build_gazebo-classic:/opt/ros/noetic/lib
export ROS_MASTER_URI=http://127.0.0.1:11319
export ROS_HOSTNAME=127.0.0.1
export GAZEBO_MASTER_URI=http://127.0.0.1:11349

rospack profile >/dev/null
cd /home/nuc/PX4-Autopilot
exec roslaunch /home/nuc/frontier-upload/launch/five_meter_avoidance_sitl.launch "$@"
