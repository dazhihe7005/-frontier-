#!/usr/bin/env python3
"""Run one SUPER planner with the clearance profile of the selected task."""

import os
import signal
import subprocess
import sys
import time

import rospy
from std_msgs.msg import UInt8


class ProfiledSuperLauncher:
    def __init__(self):
        self.goaf_config = rospy.get_param("~goaf_config_path")
        self.tunnel_config = rospy.get_param("~tunnel_config_path")
        for path in (self.goaf_config, self.tunnel_config):
            if not os.path.isfile(path):
                raise RuntimeError("SUPER profile is missing: " + path)
        self.profile = "goaf"
        self.requested_profile = "goaf"
        self.child = None
        self.stopping = False
        rospy.Subscriber(
            rospy.get_param("~active_task_topic", "/mine_uav/mission/active_task"),
            UInt8, self._active_task_callback, queue_size=1,
        )
        rospy.on_shutdown(self._stop)

    def _active_task_callback(self, message):
        # Task zero is idle. Keep the current planner and its warm map until
        # the next exploration task is selected. Other tasks never own SUPER.
        if message.data == 1:
            self.requested_profile = "goaf"
        elif message.data == 4:
            self.requested_profile = "tunnel"

    def _start(self):
        config = self.goaf_config if self.profile == "goaf" else self.tunnel_config
        self.child = subprocess.Popen(
            ["rosrun", "super_planner", "fsm_node", "__name:=fsm_node",
             "_config_path:=" + config],
            start_new_session=True,
        )
        rospy.loginfo("Started SUPER %s profile from %s (pid %d)",
                      self.profile, config, self.child.pid)

    def _terminate_child(self):
        if self.child is None:
            return
        child = self.child
        if child.poll() is None:
            os.killpg(child.pid, signal.SIGTERM)
            try:
                child.wait(timeout=4.0)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGKILL)
                child.wait(timeout=2.0)
        self.child = None

    def _stop(self):
        self.stopping = True
        self._terminate_child()

    def run(self):
        self._start()
        while not rospy.is_shutdown():
            if self.requested_profile != self.profile:
                rospy.logwarn("Switching SUPER clearance profile %s -> %s",
                              self.profile, self.requested_profile)
                self._terminate_child()
                self.profile = self.requested_profile
                self._start()
            elif self.child.poll() is not None:
                rospy.logerr("SUPER planner exited unexpectedly (status %s)",
                             self.child.returncode)
                return 1
            time.sleep(0.05)
        return 0


if __name__ == "__main__":
    rospy.init_node("task_profiled_super_launcher")
    try:
        sys.exit(ProfiledSuperLauncher().run())
    except (OSError, RuntimeError, subprocess.SubprocessError) as error:
        rospy.logerr("Could not run SUPER planner: %s", error)
        sys.exit(1)
