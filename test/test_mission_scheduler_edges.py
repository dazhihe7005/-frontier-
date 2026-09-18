#!/usr/bin/env python3

"""Exercise edge triggering, repeat runs, exclusion, and pilot takeover."""

import threading
import time
import unittest

import rospy
import rostest
from mavros_msgs.msg import RCIn, State
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String, UInt8


class MissionSchedulerEdgesTest(unittest.TestCase):
    def setUp(self):
        self.lock = threading.Lock()
        self.channels = [1500] * 11
        self.channels[6] = 1000
        self.channels[10] = 1000
        self.mode = "POSCTL"
        self.publish_odom = True
        self.publish_state = True
        self.publish_shaft_status = True
        self.shaft_status = "IDLE"
        self.task = None
        self.reason = None
        self.goaf_enabled = None
        self.shaft_enabled = None
        self.auto_enabled = None
        self.return_home = None
        self.rc_pub = rospy.Publisher("/test/mission/rc", RCIn, queue_size=2)
        self.odom_pub = rospy.Publisher("/test/mission/odom", Odometry, queue_size=2)
        self.state_pub = rospy.Publisher("/test/mission/mavros_state", State, queue_size=2)
        self.finished_pub = rospy.Publisher(
            "/mine_uav/exploration/finished", Bool, queue_size=2, latch=True
        )
        self.shaft_status_pub = rospy.Publisher(
            "/mine_uav/shaft/status", String, queue_size=2, latch=True
        )
        rospy.Subscriber("/mine_uav/mission/active_task", UInt8, self.on_task)
        rospy.Subscriber("/mine_uav/mission/goaf_enable", Bool, self.on_goaf)
        rospy.Subscriber("/mine_uav/mission/shaft_enable", Bool, self.on_shaft)
        rospy.Subscriber("/mine_uav/mission/auto_enable", Bool, self.on_auto)
        rospy.Subscriber("/mine_uav/mission/return_home", Bool, self.on_return)
        rospy.Subscriber("/mine_uav/mission/status", String, self.on_scheduler_status)
        self.timer = rospy.Timer(rospy.Duration(0.03), self.publish_inputs)
        self.finished_pub.publish(Bool(data=False))
        self.shaft_status_pub.publish(String(data="IDLE"))

    def tearDown(self):
        self.timer.shutdown()

    def on_task(self, message):
        with self.lock:
            self.task = message.data

    def on_goaf(self, message):
        with self.lock:
            self.goaf_enabled = message.data

    def on_shaft(self, message):
        with self.lock:
            self.shaft_enabled = message.data

    def on_auto(self, message):
        with self.lock:
            self.auto_enabled = message.data

    def on_return(self, message):
        with self.lock:
            self.return_home = message.data

    def on_scheduler_status(self, message):
        with self.lock:
            self.reason = message.data.rsplit(" reason=", 1)[-1].split(" ", 1)[0]

    def publish_inputs(self, _event):
        with self.lock:
            channels = list(self.channels)
            mode = self.mode
            publish_odom = self.publish_odom
            publish_state = self.publish_state
            publish_shaft_status = self.publish_shaft_status
            shaft_status = self.shaft_status
        rc = RCIn()
        rc.header.stamp = rospy.Time.now()
        rc.channels = channels
        self.rc_pub.publish(rc)
        if publish_odom:
            self.odom_pub.publish(Odometry())
        if publish_state:
            state = State()
            state.connected = True
            state.armed = True
            state.mode = mode
            self.state_pub.publish(state)
        if publish_shaft_status:
            self.shaft_status_pub.publish(String(data=shaft_status))

    def set_channel(self, index, value):
        with self.lock:
            self.channels[index] = value

    def set_shaft_status(self, value):
        with self.lock:
            self.shaft_status = value

    def assert_reason(self, reason, timeout=3.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self.lock:
                if self.reason == reason:
                    return
            time.sleep(0.02)
        self.fail("expected reason {} but saw {}".format(reason, self.reason))

    def assert_task(self, task, timeout=3.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not rospy.is_shutdown():
            with self.lock:
                if (self.task == task and
                        self.goaf_enabled == (task == 1) and
                        self.shaft_enabled == (task == 2) and
                        self.auto_enabled == (task != 0)):
                    return
            time.sleep(0.02)
        self.fail("expected task {} but saw {}".format(task, self.task))

    def assert_stays(self, task, seconds=0.3):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            with self.lock:
                self.assertEqual(self.task, task)
            time.sleep(0.02)

    def test_edge_lifecycle(self):
        self.assert_task(0)
        deadline = time.monotonic() + 3.0
        while self.rc_pub.get_num_connections() == 0 and time.monotonic() < deadline:
            time.sleep(0.02)
        self.assertGreater(self.rc_pub.get_num_connections(), 0)
        self.assert_stays(0, 0.35)  # Let both initial levels become baselines.
        self.set_channel(6, 2000)
        self.assert_task(1)
        self.set_channel(10, 2000)
        self.assert_stays(1)  # Other task cannot preempt.
        self.set_channel(6, 1000)
        self.assert_stays(1)  # Same task cannot restart while running.

        self.finished_pub.publish(Bool(data=True))
        self.assert_task(0)
        self.finished_pub.publish(Bool(data=False))
        time.sleep(0.25)  # Let the post-task switch baselines settle.
        self.set_channel(6, 2000)
        self.assert_task(1)  # Same task can start again without power cycling.

        with self.lock:
            self.mode = "OFFBOARD"
        time.sleep(0.15)
        with self.lock:
            self.mode = "POSCTL"
        self.assert_task(0)  # Pilot takeover revokes task authority.

        time.sleep(0.25)
        with self.lock:
            self.publish_odom = False
        time.sleep(0.6)
        self.set_channel(10, 1000)
        self.assert_task(2)  # Shaft task does not depend on Fast-LIO2 odometry.
        self.set_shaft_status("DESCENDING")
        time.sleep(0.12)
        self.set_shaft_status("COMPLETE")
        self.assert_task(0)
        self.set_shaft_status("IDLE")

        with self.lock:
            self.publish_odom = True
        time.sleep(0.25)
        self.set_channel(6, 1000)
        self.assert_task(1)
        self.set_channel(10, 2000)
        self.finished_pub.publish(Bool(data=True))
        self.assert_task(0)
        self.assert_stays(0, 0.35)  # Pending CH11 movement must not launch later.

        self.finished_pub.publish(Bool(data=False))
        self.set_channel(6, 2000)
        self.assert_task(1)
        with self.lock:
            self.mode = "MANUAL"
        self.assert_task(0)  # Pilot may cancel before entering OFFBOARD.

        with self.lock:
            self.mode = "POSCTL"
        time.sleep(0.25)
        self.set_channel(6, 1000)
        self.assert_task(1)
        with self.lock:
            self.publish_odom = False
        self.assert_task(0, timeout=2.0)
        with self.lock:
            self.assertFalse(self.return_home)  # No autonomous return without pose.

        # A stopped MAVROS state stream must revoke Task 2; the last heartbeat
        # cannot remain a permanent permission to keep controlling the FCU.
        with self.lock:
            self.publish_odom = True
        time.sleep(0.25)  # The scheduler must first establish fresh switch baselines.
        self.set_channel(10, 1000)
        self.assert_task(2)
        self.set_shaft_status("DESCENDING")
        time.sleep(0.15)
        with self.lock:
            self.publish_state = False
        self.assert_task(0, timeout=2.0)
        self.assert_reason("mavros_state_lost")

        with self.lock:
            self.publish_state = True
        self.set_shaft_status("IDLE")
        time.sleep(0.25)
        self.set_channel(10, 2000)
        self.assert_task(2)
        self.set_shaft_status("DESCENDING")
        time.sleep(0.15)
        with self.lock:
            self.publish_shaft_status = False
        self.assert_task(0, timeout=2.0)
        self.assert_reason("task2_status_lost")

        with self.lock:
            self.publish_shaft_status = True
        self.set_shaft_status("IDLE")
        time.sleep(0.25)
        self.set_channel(10, 1000)
        self.assert_task(2)
        self.assert_task(0, timeout=2.0)  # No sensors: IDLE never becomes DESCENDING.
        self.assert_reason("task2_start_timeout")


if __name__ == "__main__":
    rospy.init_node("test_mission_scheduler_edges")
    rostest.rosrun("mine_uav_control", "mission_scheduler_edges", MissionSchedulerEdgesTest)
