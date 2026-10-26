#!/usr/bin/env python3
"""ROS integration regression: a CH5/PX4 mode takeover stays manual."""

import math
import threading
import time
import unittest

import rospy
import rostest
from geometry_msgs.msg import (PoseStamped, TransformStamped, TwistStamped,
                               Vector3Stamped)
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import SetMode, SetModeResponse
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, Header, String


class ManualOverrideTest(unittest.TestCase):
    def setUp(self):
        self._lock = threading.Lock()
        self._mode = "POSCTL"
        self._auto = False
        self._mission = True
        self._alignment_z = 0.0
        self._alignment_yaw = 0.0
        self._pose_x = 0.0
        self._pose_y = 0.0
        self._pose_z = 1.5
        self._command_x = 0.0
        self._command_y = 0.0
        self._command_z = 1.5
        self._command_speed = 0.0
        self._measured_speed = 0.0
        self._exploration_status = "EXPLORING"
        self._returning = False
        self._vision_healthy = True
        self._trajectory_id = 0
        self._obstacle_distance = None
        self._setpoints = []
        self._setpoint_velocities = []
        self._requests = []
        self._return_requests = []
        self._execution_authorizations = []
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
        self._task_odometry_pub = rospy.Publisher(
            "/mine_uav/task1/odometry", Odometry, queue_size=5
        )
        self._obstacle_cloud_pub = rospy.Publisher(
            "/mine_uav/task1/cloud_registered", PointCloud2, queue_size=2
        )
        self._alignment_pub = rospy.Publisher(
            "/mine_uav/task1/planning_to_px4_alignment",
            TransformStamped, queue_size=1, latch=True
        )
        self._sensor_position_pub = rospy.Publisher(
            "/mine_uav/task1/lio_sensor_position_in_body",
            Vector3Stamped, queue_size=1, latch=True
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
        self._execution_authorization_sub = rospy.Subscriber(
            "/mine_uav/super/execution_enable", Bool,
            self._execution_authorization_callback
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
        self._execution_authorization_sub.unregister()
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

    def _execution_authorization_callback(self, message):
        with self._lock:
            self._execution_authorizations.append(message.data)

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
                alignment_yaw = self._alignment_yaw
                pose_x = self._pose_x
                pose_y = self._pose_y
                pose_z = self._pose_z
                command_x = self._command_x
                command_y = self._command_y
                command_z = self._command_z
                command_speed = self._command_speed
                measured_speed = self._measured_speed
                exploration_status = self._exploration_status
                returning = self._returning
                vision_healthy = self._vision_healthy
                trajectory_id = self._trajectory_id
                obstacle_distance = self._obstacle_distance
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
            pose.pose.position.y = pose_y
            pose.pose.position.z = pose_z
            pose.pose.orientation.z = math.sin(0.5 * alignment_yaw)
            pose.pose.orientation.w = math.cos(0.5 * alignment_yaw)
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
            command.position.y = command_y
            command.position.z = command_z
            command.velocity.x = command_speed
            command.trajectory_flag = PositionCommand.TRAJECTORY_STATUS_READY
            command.trajectory_id = trajectory_id
            self._command_pub.publish(command)
            task_odometry = Odometry()
            task_odometry.header.stamp = stamp
            task_odometry.header.frame_id = "task1_planning"
            # This topic is in SUPER's task1_planning frame, unlike the PX4
            # local pose above. The fixture's command coordinates represent
            # that planning-frame vehicle state across alignment tests.
            task_odometry.pose.pose.position.x = command_x
            task_odometry.pose.pose.position.y = command_y
            task_odometry.pose.pose.position.z = command_z
            task_odometry.pose.pose.orientation.w = 1.0
            self._task_odometry_pub.publish(task_odometry)
            cloud_header = Header(stamp=stamp, frame_id="task1_planning")
            obstacle_points = []
            if obstacle_distance is not None:
                obstacle_points.append(
                    (command_x + obstacle_distance, command_y, command_z)
                )
            self._obstacle_cloud_pub.publish(
                point_cloud2.create_cloud_xyz32(cloud_header, obstacle_points)
            )
            alignment = TransformStamped()
            alignment.header.stamp = stamp
            alignment.transform.rotation.z = math.sin(0.5 * alignment_yaw)
            alignment.transform.rotation.w = math.cos(0.5 * alignment_yaw)
            alignment.transform.translation.z = alignment_z
            self._alignment_pub.publish(alignment)
            sensor_position = Vector3Stamped()
            sensor_position.header.stamp = stamp
            sensor_position.vector.x = 0.1
            self._sensor_position_pub.publish(sensor_position)
            self._mission_pub.publish(Bool(data=mission))
            self._auto_pub.publish(Bool(data=auto))
            self._vision_pub.publish(Bool(data=vision_healthy))
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
        with self._lock:
            self.assertTrue(self._execution_authorizations[-1])
        # Front-wall/map-closure waiting must hold the entry pose, not chase
        # a subsequently drifting localization estimate downward.
        with self._lock:
            self._exploration_status = "WAIT_MAP_CLOSURE"
        self.assertTrue(self._wait_for(lambda: self._status == "HOLD_COMMAND_TIMEOUT"))
        self.assertTrue(self._wait_for(
            lambda: bool(self._execution_authorizations) and
            not self._execution_authorizations[-1]
        ), "SUPER execution clock remained authorized during fixed hold")
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
            lambda: bool(self._execution_authorizations) and
            self._execution_authorizations[-1]
        ), "SUPER execution clock did not resume after hold cleared")
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

    def test_transient_preoffboard_vision_gap_restarts_prestream(self):
        self.assertTrue(self._wait_for(lambda: self._status == "WAIT_AUTO_ENABLE"))
        time.sleep(0.3)
        with self._lock:
            self._auto = True
        time.sleep(0.2)
        with self._lock:
            self._vision_healthy = False
            requests_before_recovery = self._requests.count("OFFBOARD")
        self.assertTrue(self._wait_for(
            lambda: self._status == "PRESTREAM_WAIT_VISION_RECOVERY"))
        time.sleep(0.15)
        with self._lock:
            self.assertEqual(self._fault, "")
            self.assertEqual(self._requests.count("OFFBOARD"),
                             requests_before_recovery)
            self._vision_healthy = True
            recovery_time = time.monotonic()
        self.assertTrue(self._wait_for(lambda: "OFFBOARD" in self._requests,
                                       timeout=3.0))
        self.assertGreaterEqual(time.monotonic() - recovery_time, 0.9)

    def test_late_offboard_confirmation_during_vision_gap_stays_unauthorized(self):
        self.assertTrue(self._wait_for(lambda: self._status == "WAIT_AUTO_ENABLE"))
        time.sleep(0.3)
        with self._lock:
            self._auto = True
        self.assertTrue(self._wait_for(lambda: "OFFBOARD" in self._requests,
                                       timeout=3.0))
        with self._lock:
            self._vision_healthy = False
        self.assertTrue(self._wait_for(
            lambda: self._status == "PRESTREAM_WAIT_VISION_RECOVERY"))
        # Reproduce the latest flight: PX4 reports the previously requested
        # mode after the health gate has already fallen.
        with self._lock:
            self._mode = "OFFBOARD"
        self.assertTrue(self._wait_for(lambda: "POSCTL" in self._requests,
                                       timeout=2.0))
        time.sleep(0.15)
        with self._lock:
            self.assertEqual(self._fault, "")
            self.assertFalse(self._execution_authorizations[-1])
            self.assertNotIn("OUTPUT_DISCONTINUITY_REJECTED", self._statuses)
            self._mode = "POSCTL"
            self._vision_healthy = True
            requests_at_recovery = self._requests.count("OFFBOARD")
            recovery_time = time.monotonic()
        self.assertTrue(self._wait_for(
            lambda: self._requests.count("OFFBOARD") > requests_at_recovery,
            timeout=3.0))
        self.assertGreaterEqual(time.monotonic() - recovery_time, 0.9)

    def test_disabled_horizontal_fence_accepts_far_valid_command(self):
        self.assertTrue(self._wait_for(lambda: self._status == "WAIT_AUTO_ENABLE"))
        with self._lock:
            # A home-centred 40 m fence used to reject this otherwise valid
            # command. Position and command move together so entry continuity
            # checks remain satisfied and this isolates flight-volume policy.
            self._pose_x = 45.0
            self._command_x = 45.0
        time.sleep(0.5)
        with self._lock:
            self.assertEqual(self._fault, "")
            self._auto = True
        self.assertTrue(self._wait_for(lambda: "OFFBOARD" in self._requests))
        with self._lock:
            self.assertNotEqual(self._fault, "HORIZONTAL_GEOFENCE")
            self._mode = "OFFBOARD"
        self.assertTrue(self._wait_for(lambda: self._status == "STREAMING"))

    def test_height_fence_uses_super_planning_frame_before_alignment(self):
        self.assertTrue(self._wait_for(lambda: self._status == "WAIT_AUTO_ENABLE"))
        # Configure the frame alignment while the task is deselected. The
        # planning command remains inside this fixture's 1.8 m SUPER ceiling,
        # while the correctly transformed PX4-local setpoint is 2.5 m. A
        # PX4-local fence would reject it; a SUPER-frame fence must accept it.
        with self._lock:
            self._mission = False
            self._alignment_z = 1.0
            self._pose_z = 2.5
            self._command_z = 1.5
        time.sleep(0.5)
        with self._lock:
            self._mission = True
        self.assertTrue(self._wait_for(lambda: self._status == "WAIT_AUTO_ENABLE"))
        with self._lock:
            self._auto = True
        self.assertTrue(self._wait_for(lambda: "OFFBOARD" in self._requests))
        with self._lock:
            self.assertNotEqual(self._fault, "HEIGHT_GEOFENCE")
            self._mode = "OFFBOARD"
        self.assertTrue(self._wait_for(lambda: self._status == "STREAMING"))
        self.assertTrue(self._wait_for(
            lambda: any(abs(z - 2.5) < 0.05 for z, _ in self._setpoints)
        ))

    def test_command_fault_keeps_retrying_return_without_offboard_exit(self):
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
        time.sleep(3.0)
        with self._lock:
            self.assertNotIn("POSCTL", self._requests)
            self.assertTrue(any(
                status in ("RETURN_ROUTE_RETRYING", "HOLD_COMMAND_TIMEOUT")
                for status in self._statuses
            ), self._statuses[-20:])

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

    def test_obstacle_hold_freezes_and_resumes_super_execution_clock(self):
        self.assertTrue(self._wait_for(lambda: self._status == "WAIT_AUTO_ENABLE"))
        time.sleep(0.5)
        with self._lock:
            self._auto = True
        self.assertTrue(self._wait_for(lambda: "OFFBOARD" in self._requests))
        with self._lock:
            self._mode = "OFFBOARD"
        self.assertTrue(self._wait_for(lambda: self._status == "STREAMING"))
        self.assertTrue(self._wait_for(
            lambda: bool(self._execution_authorizations) and
            self._execution_authorizations[-1]
        ))

        # Reproduce the latest flight's soft-zone stop: outside the 0.65 m
        # hard cylinder but inside the 0.8 m planning monitor. PX4 must retain
        # OFFBOARD hold while SUPER's polynomial clock is frozen.
        with self._lock:
            self._obstacle_distance = 0.75
        self.assertTrue(self._wait_for(
            lambda: self._status.startswith("OBSTACLE_MONITORING_")
        ))
        self.assertTrue(self._wait_for(
            lambda: bool(self._execution_authorizations) and
            not self._execution_authorizations[-1]
        ), "SUPER clock remained authorized during obstacle hold")
        with self._lock:
            self.assertNotIn("POSCTL", self._requests)
            self.assertEqual(self._fault, "")
            self._obstacle_distance = None

        self.assertTrue(self._wait_for(lambda: self._status == "STREAMING"))
        self.assertTrue(self._wait_for(
            lambda: bool(self._execution_authorizations) and
            self._execution_authorizations[-1]
        ), "SUPER clock did not resume after obstacle cleared")
        with self._lock:
            self.assertNotIn("POSCTL", self._requests)
            self.assertEqual(self._fault, "")

    def test_entry_speed_is_not_rechecked_while_mode_confirmation_is_pending(self):
        self.assertTrue(self._wait_for(lambda: self._status == "WAIT_AUTO_ENABLE"))
        time.sleep(0.5)
        with self._lock:
            self._auto = True
        self.assertTrue(self._wait_for(lambda: "OFFBOARD" in self._requests))
        with self._lock:
            request_count = self._requests.count("OFFBOARD")
            # This is above the 0.20 m/s entry limit but below the 0.40 m/s
            # in-flight hard limit.  A mode request is already outstanding,
            # so a boundary change during MAVROS/PX4 confirmation must not
            # retroactively invalidate the accepted entry decision.
            self._measured_speed = 0.21
        time.sleep(0.2)  # Less than the configured 0.5 s retry interval.
        with self._lock:
            self.assertEqual(self._fault, "")
            self.assertEqual(self._requests.count("OFFBOARD"), request_count)
            self._mode = "OFFBOARD"
        self.assertTrue(self._wait_for(lambda: self._status == "STREAMING"))

    def test_heading_alignment_rotates_command_without_lever_arm_jump(self):
        self.assertTrue(self._wait_for(lambda: self._status == "WAIT_AUTO_ENABLE"))
        time.sleep(0.5)
        with self._lock:
            self._alignment_yaw = math.pi / 2.0
            self._pose_x = 0.0
            self._pose_y = 1.0
            self._command_x = 1.0
            self._command_speed = 0.1
        time.sleep(0.3)
        with self._lock:
            self._auto = True
        self.assertTrue(self._wait_for(lambda: "OFFBOARD" in self._requests))
        with self._lock:
            self._mode = "OFFBOARD"
        self.assertTrue(self._wait_for(lambda: self._status == "STREAMING"))
        self.assertTrue(self._wait_for(
            lambda: len(self._setpoint_velocities) >= 5, timeout=3.0))
        with self._lock:
            vx, vy, _, px, py, _ = self._setpoint_velocities[-1]
        # The smooth handoff can advance a few millimetres along the valid
        # transformed trajectory before this sample is inspected.
        self.assertAlmostEqual(px, 0.0, delta=0.01)
        self.assertAlmostEqual(py, 1.0, delta=0.01)
        self.assertAlmostEqual(vx, 0.0, delta=0.01)
        self.assertAlmostEqual(vy, 0.1, delta=0.01)

    def test_unconfirmed_offboard_retry_rechecks_entry_speed(self):
        self.assertTrue(self._wait_for(lambda: self._status == "WAIT_AUTO_ENABLE"))
        time.sleep(0.5)
        with self._lock:
            self._auto = True
        self.assertTrue(self._wait_for(lambda: "OFFBOARD" in self._requests))
        with self._lock:
            self._measured_speed = 0.21
        self.assertTrue(self._wait_for(
            lambda: self._status == "PRESTREAM_WAIT_VEHICLE_STABLE",
            timeout=2.0
        ))
        with self._lock:
            self.assertEqual(self._fault, "")
            first_request_count = self._requests.count("OFFBOARD")
            self._measured_speed = 0.0
        # Recovery must complete a fresh prestream interval; only then may a
        # second OFFBOARD request be sent.
        self.assertTrue(self._wait_for(
            lambda: self._requests.count("OFFBOARD") > first_request_count,
            timeout=3.0
        ))

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

    def test_same_path_lag_above_cross_track_limit_is_governed(self):
        self.assertTrue(self._wait_for(lambda: self._status == "WAIT_AUTO_ENABLE"))
        with self._lock:
            self._command_speed = 0.2
        time.sleep(0.5)
        with self._lock:
            self._auto = True
        self.assertTrue(self._wait_for(lambda: "OFFBOARD" in self._requests))
        with self._lock:
            self._mode = "OFFBOARD"
        self.assertTrue(self._wait_for(lambda: self._status == "STREAMING"))

        # The latest flight reached 0.655 m almost entirely along the moving
        # path tangent. This must remain governed, while the unit tests retain
        # the strict 0.65 m rule for lateral, vertical, ahead-of-reference and
        # stationary-reference errors.
        # Reproduce a physically possible accumulation of lag. An
        # instantaneous 0.655 m estimator jump is a different fault class and
        # is intentionally rejected by the output-continuity guard.
        for step in range(1, 67):
            with self._lock:
                self._pose_x = -0.655 * step / 66.0
            time.sleep(0.04)
        self.assertTrue(self._wait_for(
            lambda: "SPEED_GOVERNING" in self._statuses, timeout=3.0
        ))
        time.sleep(0.3)
        with self._lock:
            self.assertEqual(self._fault, "")
            self.assertNotIn("POSCTL", self._requests)

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
