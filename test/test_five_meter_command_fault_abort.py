#!/usr/bin/env python3
"""A command-bridge fault must terminate the five-metre task immediately."""

import threading
import time
import unittest

import rospy
import rostest
from geometry_msgs.msg import TransformStamped
from mavros_msgs.msg import RCIn, State
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, String, UInt64
from super_planner.msg import GoalCommand


class FiveMeterCommandFaultAbortTest(unittest.TestCase):
    def setUp(self):
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._rc_high = False
        self._auto_samples = []
        self._mission_samples = []
        self._statuses = []
        self._goal_id = 0

        self._rc_pub = rospy.Publisher("/test/rc", RCIn, queue_size=5)
        self._state_pub = rospy.Publisher("/test/state", State, queue_size=5)
        self._odom_pub = rospy.Publisher("/test/odometry", Odometry, queue_size=5)
        self._cloud_pub = rospy.Publisher("/test/cloud", PointCloud2, queue_size=5)
        self._alignment_pub = rospy.Publisher(
            "/test/alignment", TransformStamped, queue_size=1, latch=True
        )
        self._accepted_pub = rospy.Publisher(
            "/mine_uav/super/accepted_goal_id", UInt64, queue_size=2
        )
        self._fault_pub = rospy.Publisher(
            "/mine_uav/task1/command_fault", String, queue_size=1, latch=True
        )
        self._auto_sub = rospy.Subscriber(
            "/mine_uav/mission/auto_enable", Bool, self._auto_callback
        )
        self._mission_sub = rospy.Subscriber(
            "/mine_uav/mission/goaf_enable", Bool, self._mission_callback
        )
        self._status_sub = rospy.Subscriber(
            "/mine_uav/five_meter_test/status", String, self._status_callback
        )
        self._goal_sub = rospy.Subscriber("/test/goal", GoalCommand,
                                          self._goal_callback)
        self._worker = threading.Thread(target=self._publish_loop, daemon=True)
        self._worker.start()

    def tearDown(self):
        self._stop.set()
        self._worker.join(timeout=2)
        for subscriber in (self._auto_sub, self._mission_sub,
                           self._status_sub, self._goal_sub):
            subscriber.unregister()

    def _auto_callback(self, message):
        with self._lock:
            self._auto_samples.append(message.data)

    def _mission_callback(self, message):
        with self._lock:
            self._mission_samples.append(message.data)

    def _status_callback(self, message):
        with self._lock:
            self._statuses.append(message.data)

    def _goal_callback(self, message):
        if message.command == GoalCommand.SET_GOAL:
            with self._lock:
                self._goal_id = message.goal_id

    def _publish_loop(self):
        while not self._stop.is_set() and not rospy.is_shutdown():
            stamp = rospy.Time.now()
            with self._lock:
                high = self._rc_high
            rc = RCIn()
            rc.header.stamp = stamp
            rc.channels = [1500] * 8
            rc.channels[6] = 1800 if high else 1000
            self._rc_pub.publish(rc)

            state = State()
            state.header.stamp = stamp
            state.connected = True
            state.armed = True
            state.mode = "POSCTL"
            self._state_pub.publish(state)

            odometry = Odometry()
            odometry.header.stamp = stamp
            odometry.header.frame_id = "task1_planning"
            odometry.pose.pose.position.z = 0.6
            odometry.pose.pose.orientation.w = 1.0
            self._odom_pub.publish(odometry)

            cloud = PointCloud2()
            cloud.header.stamp = stamp
            cloud.header.frame_id = "task1_planning"
            self._cloud_pub.publish(cloud)

            alignment = TransformStamped()
            alignment.header.stamp = stamp
            alignment.child_frame_id = "task1_planning"
            alignment.transform.rotation.w = 1.0
            self._alignment_pub.publish(alignment)
            self._stop.wait(0.033)

    def _wait_for(self, predicate, timeout=6.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not rospy.is_shutdown():
            with self._lock:
                if predicate():
                    return True
            time.sleep(0.04)
        return False

    def test_bridge_fault_aborts_task_and_revokes_authority(self):
        self.assertTrue(self._wait_for(
            lambda: self._rc_pub.get_num_connections() > 0
            and self._state_pub.get_num_connections() > 0
            and self._odom_pub.get_num_connections() > 0
            and self._cloud_pub.get_num_connections() > 0
            and self._alignment_pub.get_num_connections() > 0
        ))
        self.assertTrue(self._wait_for(
            lambda: any("state=IDLE" in value for value in self._statuses)
        ))
        # The coordinator only accepts a rising edge after it has observed a
        # debounced low level.  Start this dwell after all subscriptions exist;
        # the latched IDLE status can otherwise arrive before the RC link does.
        time.sleep(0.5)
        with self._lock:
            self._rc_high = True
        self.assertTrue(self._wait_for(
            lambda: any("state=WAIT_HOVER" in value for value in self._statuses)
        ))
        self.assertTrue(self._wait_for(lambda: bool(self._goal_id)))
        with self._lock:
            goal_id = self._goal_id
        self.assertTrue(self._wait_for(
            lambda: self._accepted_pub.get_num_connections() > 0
            and self._fault_pub.get_num_connections() > 0
        ))
        self._accepted_pub.publish(UInt64(data=goal_id))
        self.assertTrue(self._wait_for(
            lambda: bool(self._auto_samples) and self._auto_samples[-1]
        ))

        self._fault_pub.publish(String(data="SPEED_GOVERNOR_INFEASIBLE"))
        self.assertTrue(self._wait_for(
            lambda: any(
                "state=ABORTED detail=COMMAND_BRIDGE_FAULT_SPEED_GOVERNOR_INFEASIBLE"
                in value for value in self._statuses
            )
        ))
        self.assertTrue(self._wait_for(
            lambda: bool(self._auto_samples) and not self._auto_samples[-1]
        ))
        with self._lock:
            self.assertTrue(self._mission_samples)
            self.assertFalse(self._mission_samples[-1])

        # Clear the bridge's latched fault and exercise a complete second task
        # lifecycle without restarting either node. This is the regression for
        # the second 2026-10-14 attempt remaining ACTIVE after an obvious fault.
        self._fault_pub.publish(String(data=""))
        with self._lock:
            first_goal_id = self._goal_id
            self._rc_high = False
        self.assertTrue(self._wait_for(
            lambda: any("state=IDLE" in value for value in self._statuses[-5:])
        ))
        time.sleep(0.2)
        with self._lock:
            self._rc_high = True
        self.assertTrue(self._wait_for(
            lambda: self._goal_id != first_goal_id
        ))
        with self._lock:
            second_goal_id = self._goal_id
        self._accepted_pub.publish(UInt64(data=second_goal_id))
        self.assertTrue(self._wait_for(
            lambda: bool(self._auto_samples) and self._auto_samples[-1]
        ))
        self._fault_pub.publish(String(data="MEASURED_SPEED_LIMIT"))
        self.assertTrue(self._wait_for(
            lambda: any(
                "state=ABORTED detail=COMMAND_BRIDGE_FAULT_MEASURED_SPEED_LIMIT"
                in value for value in self._statuses
            )
        ))
        self.assertTrue(self._wait_for(
            lambda: bool(self._auto_samples) and not self._auto_samples[-1]
            and bool(self._mission_samples) and not self._mission_samples[-1]
        ))


if __name__ == "__main__":
    rospy.init_node("test_five_meter_command_fault_abort")
    rostest.rosrun("mine_uav_control", "five_meter_command_fault_abort",
                   FiveMeterCommandFaultAbortTest)
