#!/usr/bin/env python3

"""Fail closed if a real-aircraft launch shares a simulated-time ROS master."""

import time

import rospy


def require_wall_time():
    if rospy.get_param("/use_sim_time", False):
        rospy.logfatal(
            "Real Task 1 refused: /use_sim_time=true. Stop SITL and use a "
            "real-time ROS master before connecting a real aircraft."
        )
        raise SystemExit(2)


if __name__ == "__main__":
    rospy.init_node("task1_real_time_guard")
    while not rospy.is_shutdown():
        require_wall_time()
        # Wall-clock polling still works if /clock is absent or frozen.
        time.sleep(0.2)
