#!/usr/bin/env python3

"""ROS topic-level check for isolated Task2 logic; never publishes to MAVROS."""

import threading
import time
import unittest

import rospy
import rostest
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Range
from std_msgs.msg import Bool, Float64, String


class ShaftRosLogicTest(unittest.TestCase):
    def setUp(self):
        self.lock = threading.Lock()
        self.depth = 0.0
        self.bottom_range = float("inf")
        self.publish_range = True
        self.status = None
        self.latest_command = None
        self.last_command_time = 0.0
        self.enable_pub = rospy.Publisher(
            "/mine_uav/mission/shaft_enable", Bool, queue_size=1, latch=True
        )
        self.depth_pub = rospy.Publisher(
            "/mine_uav/shaft/relative_depth_m", Float64, queue_size=10
        )
        self.range_pub = rospy.Publisher(
            "/mine_uav/shaft/bottom_range", Range, queue_size=10
        )
        rospy.Subscriber("/mine_uav/shaft/status", String, self.on_status)
        rospy.Subscriber(
            "/mine_uav/shaft/velocity_intent_enu", TwistStamped, self.on_command
        )
        self.timer = rospy.Timer(rospy.Duration(0.05), self.publish_inputs)
        self.enable_pub.publish(Bool(data=False))

    def tearDown(self):
        self.enable_pub.publish(Bool(data=False))
        self.timer.shutdown()

    def on_status(self, message):
        with self.lock:
            self.status = message.data

    def on_command(self, message):
        with self.lock:
            self.latest_command = message
            self.last_command_time = time.monotonic()

    def publish_inputs(self, _event):
        with self.lock:
            depth = self.depth
            bottom_range = self.bottom_range
            publish_range = self.publish_range
        self.depth_pub.publish(Float64(data=depth))
        if publish_range:
            message = Range()
            message.header.stamp = rospy.Time.now()
            message.radiation_type = Range.INFRARED
            message.min_range = 0.2
            message.max_range = 30.0
            message.range = bottom_range
            self.range_pub.publish(message)

    def wait_for(self, predicate, label, timeout=5.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end and not rospy.is_shutdown():
            with self.lock:
                if predicate():
                    return
            time.sleep(0.03)
        self.fail("timeout waiting for " + label)

    def test_topic_lifecycle(self):
        self.wait_for(lambda: self.status == "IDLE", "idle")
        self.wait_for(lambda: self.depth_pub.get_num_connections() > 0 and
                      self.range_pub.get_num_connections() > 0, "sensor subscribers")
        time.sleep(0.2)
        self.enable_pub.publish(Bool(data=True))
        self.wait_for(lambda: self.status == "DESCENDING" and
                      self.latest_command is not None and
                      self.latest_command.twist.linear.z < 0.0, "descent")
        with self.lock:
            self.depth = 1.0
            self.bottom_range = 1.5
        self.wait_for(lambda: self.status == "RETURNING" and
                      self.latest_command.twist.linear.z > 0.0, "return")
        with self.lock:
            self.depth = 0.3
            self.bottom_range = float("inf")
        self.wait_for(lambda: self.status == "COMPLETE", "completion")
        with self.lock:
            last_command = self.last_command_time
            self.publish_range = False
        time.sleep(0.55)
        with self.lock:
            self.assertEqual(self.status, "COMPLETE")
            self.assertEqual(last_command, self.last_command_time)
        self.enable_pub.publish(Bool(data=False))
        self.wait_for(lambda: self.status == "IDLE", "reset")
        self.enable_pub.publish(Bool(data=True))
        time.sleep(0.1)
        with self.lock:
            self.assertEqual(self.status, "IDLE")
            self.publish_range = True
            self.bottom_range = float("inf")
        self.wait_for(lambda: self.status == "DESCENDING", "restart")
        with self.lock:
            self.publish_range = False
        self.wait_for(lambda: self.status == "FAULT_NO_SAFE_AUTONOMOUS_RECOVERY",
                      "stale-range fault")


if __name__ == "__main__":
    rospy.init_node("shaft_mission_ros_logic_test")
    rostest.rosrun("mine_uav_control", "shaft_mission_ros_logic", ShaftRosLogicTest)
