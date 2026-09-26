#!/usr/bin/env python3
"""Record Gazebo truth for offline geometry audit; publish no flight inputs."""

import csv
from datetime import datetime
import os
from pathlib import Path

import rospy
from gazebo_msgs.msg import ModelStates
from mavros_msgs.msg import State
from std_msgs.msg import String


class TruthTrajectoryRecorder:
    def __init__(self):
        self.model_name = rospy.get_param("~model_name", "iris")
        self.sample_period = max(0.02, float(rospy.get_param("~sample_period", 0.1)))
        default_dir = Path("/home/nuc/gbplanner2_isolated_ws/runtime/map_reference")
        default_name = "truth_trajectory_{}_{}.csv".format(
            datetime.now().strftime("%Y%m%d_%H%M%S_%f"), os.getpid())
        self.output = Path(rospy.get_param("~output_file", str(default_dir / default_name)))
        self.output.parent.mkdir(parents=True, exist_ok=True)
        self.file = self.output.open("w", encoding="utf-8", newline="")
        self.writer = csv.writer(self.file)
        self.writer.writerow(("sim_time", "x", "y", "z", "armed", "offboard",
                              "exploration_started"))
        self.armed = False
        self.offboard = False
        self.exploration_started = False
        self.last_sample = None
        self.rows = 0
        rospy.Subscriber("/mavros/state", State, self._state_cb, queue_size=10)
        rospy.Subscriber("/mine_uav/gbplanner/mission_status", String,
                         self._mission_cb, queue_size=5)
        rospy.Subscriber("/gazebo/model_states", ModelStates,
                         self._model_cb, queue_size=20)
        rospy.on_shutdown(self._close)
        rospy.loginfo("Read-only truth trajectory: %s", self.output)

    def _state_cb(self, msg):
        self.armed = bool(msg.armed)
        self.offboard = msg.mode == "OFFBOARD"

    def _mission_cb(self, msg):
        if msg.data == "AUTOMATIC_EXPLORATION_STARTED":
            self.exploration_started = True

    def _model_cb(self, msg):
        try:
            index = msg.name.index(self.model_name)
        except ValueError:
            return
        stamp = rospy.Time.now().to_sec()
        if self.last_sample is not None and stamp-self.last_sample < self.sample_period:
            return
        self.last_sample = stamp
        position = msg.pose[index].position
        self.writer.writerow(("{:.3f}".format(stamp), "{:.5f}".format(position.x),
                              "{:.5f}".format(position.y), "{:.5f}".format(position.z),
                              int(self.armed), int(self.offboard),
                              int(self.exploration_started)))
        self.rows += 1
        if self.rows % 100 == 0:
            self.file.flush()

    def _close(self):
        if not self.file.closed:
            self.file.flush()
            self.file.close()


if __name__ == "__main__":
    rospy.init_node("diagnostic_truth_trajectory")
    TruthTrajectoryRecorder()
    rospy.spin()
