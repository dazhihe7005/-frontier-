#!/usr/bin/env python3
"""ROS integration regression: a CH5/PX4 mode takeover stays manual."""

import threading
import time
import unittest

import rospy
import rostest
from geometry_msgs.msg import PoseStamped, TransformStamped, TwistStamped
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import SetMode, SetModeResponse
from quadrotor_msgs.msg import PositionCommand
from std_msgs.msg import Bool, String


class ManualOverrideTest(unittest.TestCase):
    def setUp(self):
        self._lock = threading.Lock()
        self._mode = "POSCTL"
        self._auto = False
        self._mission = True
        self._alignment_z = 0.0
        self._pose_x = 0.0
        self._pose_z = 1.5
        self._command_x = 0.0
        self._command_z = 1.5
        self._command_speed = 0.0
        self._measured_speed = 0.0
        self._exploration_status = "EXPLORING"
        self._returning = False
        self._trajectory_id = 0
        self._setpoints = []
        self._setpoint_velocities = []
        self._requests = []
        self._return_requests = []
        self._status = ""
        self._statuses = []
        self._fault = ""
        self._stop = threading.Event()
        self._service = rospy.Service("/mavros/set_mode", SetMode, self._set_mode)
        self._state_pub = rospy.Publisher("/mavros/state", State, queue_size=5)
        self._pose_pub = rospy.Publisher(
            "/mavros/local_position/pose", PoseStamped, queue_size=5
        )
        self._velocity_pub = rospy.Publisher(
            "/mavros/local_position/velocity_local", TwistStamped, queue_size=5
        )
        self._command_pub = rospy.Publisher(
            "/planning/pos_cmd", PositionCommand, queue_size=5
        )
        self._alignment_pub = rospy.Publisher(
            "/mine_uav/task1/planning_to_px4_alignment",
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
        self._returning_pub = rospy.Publisher(
            "/mine_uav/exploration/returning", Bool, queue_size=1, latch=True
        )
        self._return_request_sub = rospy.Subscriber(
            "/mine_uav/exploration/return_home", Bool,
            self._return_request_callback
        )
        self._status_sub = rospy.Subscriber(
            "/mine_uav/task1/command_status", String, self._status_callback
        )
        self._fault_sub = rospy.Subscriber(
            "/mine_uav/task1/command_fault", String, self._fault_callback
        )
        self._setpoint_sub = rospy.Subscriber(
            "/mine_uav/setpoint_cmd", PositionTarget, self._setpoint_callback
        )
        self._worker = threading.Thread(target=self._publish_loop, daemon=True)
        self._worker.start()

    def tearDown(self):
        with self._lock:
            self._auto = False
            self._mission = False
        time.sleep(0.1)
        self._stop.set()
        self._worker.join(timeout=2)
        self._status_sub.unregister()
        self._fault_sub.unregister()
        self._return_request_sub.unregister()
        self._setpoint_sub.unregister()
        self._service.shutdown("test complete")

    def _set_mode(self, request):
        with self._lock:
            self._requests.append(request.custom_mode)
        return SetModeResponse(mode_sent=True)

    def _status_callback(self, message):
        with self._lock:
            self._status = message.data
            self._statuses.append(message.data)
            self._statuses = self._statuses[-200:]

    def _fault_callback(self, message):
        with self._lock:
            self._fault = message.data

    def _return_request_callback(self, message):
        with self._lock:
            self._return_requests.append(message.data)

    def _setpoint_callback(self, message):
        with self._lock:
            self._setpoints.append((message.position.z, message.type_mask))
            self._setpoints = self._setpoints[-100:]
            self._setpoint_velocities.append((
                message.velocity.x, message.velocity.y, message.velocity.z,
                message.position.x, message.position.y, message.position.z
            ))
            self._setpoint_velocities = self._setpoint_velocities[-100:]

    def _publish_loop(self):
        while not self._stop.is_set() and not rospy.is_shutdown():
            with self._lock:
                mode = self._mode
                auto = self._auto
                mission = self._mission
                alignment_z = self._alignment_z
                pose_x = self._pose_x
                pose_z = self._pose_z
                command_x = self._command_x
                command_z = self._command_z
                command_speed = self._command_speed
                measured_speed = self._measured_speed
                exploration_status = self._exploration_status
                returning = self._returning
                trajectory_id = self._trajectory_id
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
            pose.pose.position.x = pose_x
            pose.pose.position.z = pose_z
            pose.pose.orientation.w = 1.0
            self._pose_pub.publish(pose)
            velocity = TwistStamped()
            velocity.header.stamp = stamp
            velocity.header.frame_id = "map"
            velocity.twist.linear.x = measured_speed
            self._velocity_pub.publish(velocity)
            command = PositionCommand()
            command.header.stamp = stamp
            command.header.frame_id = "task1_planning"
            command.position.x = command_x
            command.position.z = command_z
            command.velocity.x = command_speed
            command.trajectory_flag = PositionCommand.TRAJECTORY_STATUS_READY
            command.trajectory_id = trajectory_id
            self._command_pub.publish(command)
            alignment = TransformStamped()
            alignment.header.stamp = stamp
            alignment.transform.rotation.w = 1.0
            alignment.transform.translation.z = alignment_z
            self._alignment_pub.publish(alignment)
            self._mission_pub.publish(Bool(data=mission))
            self._auto_pub.publish(Bool(data=auto))
            self._vision_pub.publish(Bool(data=True))
            self._exploration_pub.publish(String(data=exploration_status))
            self._returning_pub.publish(Bool(data=returning))
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
        # Resume from the SAME terminal reference. Discontinuous restarts are
        # rejected by the separate motion-continuity regression suite.
        with self._lock:
            self._exploration_status = "EXPLORING"
            self._pose_z = 1.5
            self._command_z = 1.5
        self.assertTrue(self._wait_for(lambda: self._status == "STREAMING"))
        self.assertTrue(self._wait_for(
            lambda: bool(self._setpoints) and
            abs(self._setpoints[-1][0] - 1.5) < 0.01
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

    def test_command_fault_returns_before_route_timeout_fallback(self):
        self.assertTrue(self._wait_for(lambda: self._status == "WAIT_AUTO_ENABLE"))
        with self._lock:
            self._auto = True
        self.assertTrue(self._wait_for(lambda: "OFFBOARD" in self._requests))
        with self._lock:
            self._mode = "OFFBOARD"
        self.assertTrue(self._wait_for(lambda: self._status == "STREAMING"))
        with self._lock:
            self._command_z = 2.2
        self.assertTrue(self._wait_for(lambda: True in self._return_requests))
        with self._lock:
            self.assertNotIn("POSCTL", self._requests)
            self._returning = True
            self._trajectory_id = 1
            self._command_z = 1.5
        self.assertTrue(self._wait_for(lambda: self._status == "STREAMING"))
        with self._lock:
            self.assertNotIn("POSCTL", self._requests)
            self._exploration_status = "RETURN_ROUTE_WAIT"
        self.assertTrue(self._wait_for(lambda: "POSCTL" in self._requests,
                                       timeout=5.0))

    def test_measured_overspeed_brakes_and_exits_offboard(self):
        self.assertTrue(self._wait_for(lambda: self._status == "WAIT_AUTO_ENABLE"))
        time.sleep(0.5)  # Ensure this test's fresh command/velocity stream is connected.
        with self._lock:
            self._auto = True
        self.assertTrue(self._wait_for(lambda: "OFFBOARD" in self._requests))
        with self._lock:
            self._mode = "OFFBOARD"
        self.assertTrue(self._wait_for(lambda: self._status == "STREAMING"))
        started = time.monotonic()
        with self._lock:
            self._measured_speed = 0.41
        self.assertTrue(self._wait_for(lambda: "POSCTL" in self._requests,
                                       timeout=5.0))
        with self._lock:
            self.assertNotEqual(self._status, "STREAMING")
            self.assertEqual(self._fault, "MEASURED_SPEED_LIMIT")
        # The fallback request is a different mode and must not be delayed by
        # the throttle on the immediately preceding OFFBOARD request.
        self.assertLess(time.monotonic() - started, 0.8)

    def test_stale_position_target_is_rejected_before_offboard(self):
        self.assertTrue(self._wait_for(lambda: self._status == "WAIT_AUTO_ENABLE"))
        time.sleep(0.5)
        with self._lock:
            # Simulate a terminal command retained from the previous task.
            self._pose_x = -1.0
            self._auto = True
        self.assertTrue(self._wait_for(
            lambda: self._fault in (
                "INITIAL_COMMAND_POSITION_JUMP",
                "PRESTREAM_TRACKING_ERROR",
            ), timeout=4.0
        ))
        time.sleep(0.3)
        with self._lock:
            self.assertNotIn("OFFBOARD", self._requests)
            samples = list(self._setpoint_velocities[-5:])
        # Immediate fail-closed rejection may publish no prestream sample. If
        # the output timer wins the callback race, it may publish only a local
        # hold; the stale x=0 planner target is never allowed through.
        self.assertTrue(all(abs(px - (-1.0)) < 0.05
                            for _, _, _, px, _, _ in samples), samples)

    def test_planner_command_above_hard_limit_never_enters_offboard(self):
        self.assertTrue(self._wait_for(lambda: self._status == "WAIT_AUTO_ENABLE"))
        time.sleep(0.5)  # Do not act on the previous test's latched status sample.
        with self._lock:
            self._command_speed = 0.41
            self._auto = True
        self.assertTrue(self._wait_for(
            lambda: self._status == "FAULT_LATCHED_TOGGLE_AUTO_LOW"
        ))
        time.sleep(0.5)
        with self._lock:
            self.assertNotIn("OFFBOARD", self._requests)

    def test_predicted_px4_tracking_speed_is_governed_without_abort(self):
        self.assertTrue(self._wait_for(lambda: self._status == "WAIT_AUTO_ENABLE"))
        time.sleep(0.5)
        with self._lock:
            self._auto = True
        self.assertTrue(self._wait_for(lambda: "OFFBOARD" in self._requests))
        with self._lock:
            self._mode = "OFFBOARD"
        self.assertTrue(self._wait_for(lambda: self._status == "STREAMING"))
        with self._lock:
            # Planner velocity remains zero and valid, but a 0.45 m tracking
            # error would make PX4's 0.9 position gain request 0.405 m/s.
            self._setpoint_velocities.clear()
            self._pose_x = -0.45
        self.assertTrue(self._wait_for(
            lambda: "SPEED_GOVERNING" in self._statuses,
            timeout=6.0
        ))
        self.assertTrue(self._wait_for(
            lambda: len(self._setpoint_velocities) >= 5, timeout=3.0
        ))
        time.sleep(0.3)
        with self._lock:
            self.assertEqual(self._measured_speed, 0.0)
            self.assertNotIn("POSCTL", self._requests)
            samples = list(self._setpoint_velocities[-5:])
        for vx, vy, vz, px, py, pz in samples:
            predicted_x = vx + 0.9 * (px - (-0.45))
            predicted_y = vy + 0.9 * py
            predicted_z = vz + 1.0 * (pz - 1.5)
            self.assertLessEqual(
                (predicted_x ** 2 + predicted_y ** 2 + predicted_z ** 2) ** 0.5,
                0.400001
            )

    def test_task_teardown_holds_last_output_before_position_handover(self):
        self.assertTrue(self._wait_for(lambda: self._status == "WAIT_AUTO_ENABLE"))
        time.sleep(0.5)
        with self._lock:
            self._auto = True
        self.assertTrue(self._wait_for(lambda: "OFFBOARD" in self._requests))
        with self._lock:
            self._mode = "OFFBOARD"
        self.assertTrue(self._wait_for(lambda: self._status == "STREAMING"))

        # Leave a realistic 5 cm tracking error, then tear down the task. The
        # source timestamp is cleared at this boundary, but the bridge must
        # hold its last transmitted endpoint. Jumping to measured pose here
        # used to trigger OUTPUT_DISCONTINUITY_REJECTED and suppress POSCTL.
        with self._lock:
            self._pose_x = -0.05
            self._setpoint_velocities.clear()
        self.assertTrue(self._wait_for(
            lambda: len(self._setpoint_velocities) >= 5, timeout=3.0
        ))
        with self._lock:
            self._mission = False
            self._auto = False
        self.assertTrue(self._wait_for(lambda: "POSCTL" in self._requests,
                                       timeout=4.0))
        time.sleep(0.3)
        with self._lock:
            statuses = list(self._statuses)
            samples = list(self._setpoint_velocities[-5:])
        self.assertNotIn("OUTPUT_DISCONTINUITY_REJECTED", statuses)
        self.assertTrue(samples)
        self.assertTrue(all(abs(px) < 0.01 for _, _, _, px, _, _ in samples),
                        samples)


if __name__ == "__main__":
    rospy.init_node("test_super_bridge_manual_override")
    rostest.rosrun("mine_uav_control", "super_bridge_manual_override",
                   ManualOverrideTest)
