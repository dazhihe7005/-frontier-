#!/usr/bin/env python3
"""ROS integration regression: a CH5/PX4 mode takeover stays manual."""

import threading
import time
import unittest

import rospy
import rostest
from geometry_msgs.msg import PoseStamped, TransformStamped
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import SetMode, SetModeResponse
from quadrotor_msgs.msg import PositionCommand
from std_msgs.msg import Bool, String


class ManualOverrideTest(unittest.TestCase):
    def setUp(self):
        self._lock = threading.Lock()
        self._mode = "POSCTL"
        self._auto = False
        self._alignment_z = 0.0
        self._pose_z = 1.5
        self._command_z = 1.5
        self._exploration_status = "EXPLORING"
        self._setpoints = []
        self._requests = []
        self._status = ""
        self._stop = threading.Event()
        self._service = rospy.Service("/mavros/set_mode", SetMode, self._set_mode)
        self._state_pub = rospy.Publisher("/mavros/state", State, queue_size=5)
        self._pose_pub = rospy.Publisher(
            "/mavros/local_position/pose", PoseStamped, queue_size=5
        )
        self._command_pub = rospy.Publisher(
            "/planning/pos_cmd", PositionCommand, queue_size=5
        )
        self._alignment_pub = rospy.Publisher(
            "/mine_uav/task1/fastlio_to_px4_alignment",
            TransformStamped, queue_size=1, latch=True
        )
        self._mission_pub = rospy.Publisher(
            "/mine_uav/mission/goaf_enable", Bool, queue_size=1, latch=True
        )
        self._auto_pub = rospy.Publisher(
            "/mine_uav/mission/auto_enable", Bool, queue_size=1, latch=True
        )
        self._vision_pub = rospy.Publisher(
            "/mine_uav/task1/vision_healthy", Bool, queue_size=1, latch=True
        )
        self._exploration_pub = rospy.Publisher(
            "/mine_uav/exploration/status", String, queue_size=1, latch=True
        )
        self._status_sub = rospy.Subscriber(
            "/mine_uav/task1/command_status", String, self._status_callback
        )
        self._setpoint_sub = rospy.Subscriber(
            "/mine_uav/setpoint_cmd", PositionTarget, self._setpoint_callback
        )
        self._worker = threading.Thread(target=self._publish_loop, daemon=True)
        self._worker.start()

    def tearDown(self):
        with self._lock:
            self._auto = False
        time.sleep(0.1)
        self._stop.set()
        self._worker.join(timeout=2)
        self._status_sub.unregister()
        self._setpoint_sub.unregister()
        self._service.shutdown("test complete")

    def _set_mode(self, request):
        with self._lock:
            self._requests.append(request.custom_mode)
        return SetModeResponse(mode_sent=True)

    def _status_callback(self, message):
        with self._lock:
            self._status = message.data

    def _setpoint_callback(self, message):
        with self._lock:
            self._setpoints.append((message.position.z, message.type_mask))
            self._setpoints = self._setpoints[-100:]

    def _publish_loop(self):
        while not self._stop.is_set() and not rospy.is_shutdown():
            with self._lock:
                mode = self._mode
                auto = self._auto
                alignment_z = self._alignment_z
                pose_z = self._pose_z
                command_z = self._command_z
                exploration_status = self._exploration_status
            stamp = rospy.Time.now()
            state = State()
            state.header.stamp = stamp
            state.connected = True
            state.armed = True
            state.mode = mode
            self._state_pub.publish(state)
            pose = PoseStamped()
            pose.header.stamp = stamp
            pose.header.frame_id = "map"
            pose.pose.position.z = pose_z
            pose.pose.orientation.w = 1.0
            self._pose_pub.publish(pose)
            command = PositionCommand()
            command.header.stamp = stamp
            command.header.frame_id = "camera_init"
            command.position.z = command_z
            command.trajectory_flag = PositionCommand.TRAJECTORY_STATUS_READY
            self._command_pub.publish(command)
            alignment = TransformStamped()
            alignment.header.stamp = stamp
            alignment.transform.rotation.w = 1.0
            alignment.transform.translation.z = alignment_z
            self._alignment_pub.publish(alignment)
            self._mission_pub.publish(Bool(data=True))
            self._auto_pub.publish(Bool(data=auto))
            self._vision_pub.publish(Bool(data=True))
            self._exploration_pub.publish(String(data=exploration_status))
            self._stop.wait(0.033)

    def _wait_for(self, predicate, timeout=5):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not rospy.is_shutdown():
            with self._lock:
                if predicate():
                    return True
            time.sleep(0.05)
        return False

    def test_manual_mode_is_not_overridden_until_auto_reset(self):
        self.assertTrue(self._wait_for(lambda: self._status == "WAIT_AUTO_ENABLE"))
        time.sleep(0.5)  # Let pose and command subscriptions receive initial samples.
        with self._lock:
            self._auto = True
        offboard_requested = self._wait_for(lambda: "OFFBOARD" in self._requests)
        with self._lock:
            diagnostic = (self._status, list(self._requests),
                          self._pose_pub.get_num_connections(),
                          self._command_pub.get_num_connections())
        self.assertTrue(offboard_requested, "bridge diagnostics: %r" % (diagnostic,))
        with self._lock:
            self._mode = "OFFBOARD"
        self.assertTrue(self._wait_for(lambda: self._status == "STREAMING"))
        # Front-wall/map-closure waiting must hold the entry pose, not chase
        # a subsequently drifting localization estimate downward.
        with self._lock:
            self._exploration_status = "WAIT_MAP_CLOSURE"
        self.assertTrue(self._wait_for(lambda: self._status == "HOLD_COMMAND_TIMEOUT"))
        with self._lock:
            self._pose_z = 1.0
            self._setpoints.clear()
        self.assertTrue(self._wait_for(lambda: len(self._setpoints) >= 8))
        with self._lock:
            held = list(self._setpoints[-8:])
        self.assertTrue(all(abs(z - 1.5) < 0.01 for z, _ in held), held)
        with self._lock:
            self._exploration_status = "EXPLORING"
            self._command_z = 1.6
        self.assertTrue(self._wait_for(lambda: self._status == "STREAMING"))
        self.assertTrue(self._wait_for(
            lambda: bool(self._setpoints) and
            abs(self._setpoints[-1][0] - 1.6) < 0.01
        ))
        with self._lock:
            self._mode = "POSCTL"
        self.assertTrue(self._wait_for(
            lambda: self._status == "FAULT_LATCHED_TOGGLE_AUTO_LOW"
        ))
        with self._lock:
            requests_at_takeover = self._requests.count("OFFBOARD")
        time.sleep(1.5)
        with self._lock:
            self.assertEqual(self._requests.count("OFFBOARD"), requests_at_takeover)
            self._auto = False
        time.sleep(0.2)
        with self._lock:
            self._auto = True
        self.assertTrue(self._wait_for(
            lambda: self._requests.count("OFFBOARD") > requests_at_takeover
        ))
        with self._lock:
            self._mode = "OFFBOARD"
        self.assertTrue(self._wait_for(lambda: self._status == "STREAMING"))
        with self._lock:
            self._alignment_z = 0.02
        time.sleep(0.2)
        with self._lock:
            self.assertNotIn("POSCTL", self._requests)
            self._alignment_z = 0.1
        self.assertTrue(self._wait_for(lambda: "POSCTL" in self._requests))
        with self._lock:
            requests_at_alignment_change = self._requests.count("OFFBOARD")
            self._mode = "POSCTL"
        time.sleep(1.0)
        with self._lock:
            self.assertEqual(self._requests.count("OFFBOARD"),
                             requests_at_alignment_change)


if __name__ == "__main__":
    rospy.init_node("test_super_bridge_manual_override")
    rostest.rosrun("mine_uav_control", "super_bridge_manual_override",
                   ManualOverrideTest)
