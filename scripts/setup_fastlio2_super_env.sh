#!/usr/bin/env bash

# Source this file before launching or inspecting the integrated ROS1 stack:
#   source /home/nuc/super_ws/src/mine_uav_control/scripts/setup_fastlio2_super_env.sh
#
# Both workspaces were built directly on top of ROS Noetic, so sourcing one
# workspace after the other drops the first workspace from several Catkin
# search paths.  Keep both overlays visible explicitly.

source /opt/ros/noetic/setup.bash
source /home/nuc/fastlio2_ws/devel/setup.bash
source /home/nuc/super_ws/devel/setup.bash --extend

export CMAKE_PREFIX_PATH="/home/nuc/super_ws/devel:/home/nuc/fastlio2_ws/devel:/opt/ros/noetic"
export ROS_PACKAGE_PATH="/home/nuc/super_ws/src:/home/nuc/fastlio2_ws/src:/opt/ros/noetic/share"
export PYTHONPATH="/home/nuc/super_ws/devel/lib/python3/dist-packages:/home/nuc/fastlio2_ws/devel/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages"
export LD_LIBRARY_PATH="/home/nuc/super_ws/devel/lib:/home/nuc/fastlio2_ws/devel/lib:/opt/ros/noetic/lib"
export PKG_CONFIG_PATH="/home/nuc/super_ws/devel/lib/pkgconfig:/home/nuc/fastlio2_ws/devel/lib/pkgconfig:/opt/ros/noetic/lib/pkgconfig"
