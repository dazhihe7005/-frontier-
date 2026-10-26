#!/usr/bin/env python3

"""ROS smoke test for the Omni-LIVO to PX4 covariance bridge."""

import math
import threading
import time
import unittest

import rospy
import rostest
from geometry_msgs.msg import PoseWithCovarianceStamped, QuaternionStamped
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger


class OmniLivoPx4VisionBridgeTest(unittest.TestCase):
    def setUp(self):
        self._lock = threading.Lock()
        self._pose = None
        self._pose_count = 0
        self._status = None
        self._statuses = []
        self._heading_alignment = None
        self._heading_alignment_count = 0
        self._sensor_rotation = None
        self._odom_pub = rospy.Publisher(
            # Production PX4 fusion consumes Omni-LIVO's corrected,
            # IMU-rate control state; the lower-rate mapping odometry remains
            # the exploration/planning input.
            "/omnilivo/control_odometry", Odometry, queue_size=5)
        self._health_pub = rospy.Publisher(
            "/omnilivo/localization_healthy", Bool, queue_size=2, latch=True)
        self._state_pub = rospy.Publisher(
            "/mavros/state", State, queue_size=2, latch=True)
        self._auto_pub = rospy.Publisher(
            "/mine_uav/mission/auto_enable", Bool, queue_size=2, latch=True)
        self._imu_pub = rospy.Publisher(
            "/mavros/imu/data", Imu, queue_size=2, latch=True)
        self._sensor_imu_pub = rospy.Publisher(
            "/livox/imu", Imu, queue_size=20)
        self._body_imu_pub = rospy.Publisher(
            "/mavros/imu/data_raw", Imu, queue_size=20)
        self._pose_sub = rospy.Subscriber(
            "/mavros/vision_pose/pose_cov", PoseWithCovarianceStamped,
            self._pose_callback)
        self._status_sub = rospy.Subscriber(
            "/mine_uav/task1/vision_status", String, self._status_callback)
        self._heading_alignment_sub = rospy.Subscriber(
            "/mine_uav/task1/vision_heading_from_planning",
            QuaternionStamped, self._heading_alignment_callback)
        self._sensor_rotation_sub = rospy.Subscriber(
            "/mine_uav/task1/lio_sensor_from_body_rotation",
            QuaternionStamped, self._sensor_rotation_callback)
        rospy.wait_for_service(
            "/omnilivo_px4_vision_bridge_test/reset_alignment", timeout=5.0)
        self._reset = rospy.ServiceProxy(
            "/omnilivo_px4_vision_bridge_test/reset_alignment", Trigger)
        self.assertTrue(self._reset().success)
        with self._lock:
            self._pose = None
            self._pose_count = 0
            self._status = None
            self._statuses = []
            self._heading_alignment = None
            self._heading_alignment_count = 0
            self._sensor_rotation = None

    def tearDown(self):
        self._auto_pub.publish(Bool(data=False))
        self._state_pub.publish(State(connected=True, armed=False))
        time.sleep(0.1)
        self._pose_sub.unregister()
        self._status_sub.unregister()
        self._heading_alignment_sub.unregister()
        self._sensor_rotation_sub.unregister()

    def _pose_callback(self, message):
        with self._lock:
            self._pose = message
            self._pose_count += 1

    def _status_callback(self, message):
        with self._lock:
            self._status = message.data
            self._statuses.append(message.data)

    def _heading_alignment_callback(self, message):
        with self._lock:
            self._heading_alignment = message
            self._heading_alignment_count += 1

    def _sensor_rotation_callback(self, message):
        with self._lock:
            self._sensor_rotation = message

    def _publish_px4_attitude(self, sensor_angular_speed=0.0):
        yaw = math.radians(60.0)
        attitude = Imu()
        attitude.header.stamp = rospy.Time.now()
        attitude.orientation.z = math.sin(0.5 * yaw)
        attitude.orientation.w = math.cos(0.5 * yaw)
        self._imu_pub.publish(attitude)

        mount_pitch = math.radians(25.0)
        sensor_imu = Imu()
        sensor_imu.header.stamp = attitude.header.stamp
        sensor_imu.linear_acceleration.x = -math.sin(mount_pitch)
        sensor_imu.linear_acceleration.z = math.cos(mount_pitch)
        sensor_imu.angular_velocity.x = sensor_angular_speed
        self._sensor_imu_pub.publish(sensor_imu)

        body_imu = Imu()
        body_imu.header.stamp = attitude.header.stamp
        body_imu.linear_acceleration.z = 9.81
        self._body_imu_pub.publish(body_imu)

    def test_healthy_omnilivo_odometry_reaches_px4_topic(self):
        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline and not rospy.is_shutdown():
            self._health_pub.publish(Bool(data=True))
            self._state_pub.publish(State(connected=True, armed=False))
            self._publish_px4_attitude()
            odometry = Odometry()
            odometry.header.stamp = rospy.Time.now()
            odometry.header.frame_id = "camera_init"
            odometry.child_frame_id = "aft_mapped"
            odometry.pose.pose.position.x = 1.0
            odometry.pose.pose.position.y = 2.0
            odometry.pose.pose.position.z = 3.0
            odometry.pose.pose.orientation.w = 1.0
            for index in (0, 7, 14):
                odometry.pose.covariance[index] = 0.01
            for index in (21, 28, 35):
                odometry.pose.covariance[index] = 0.02
            self._odom_pub.publish(odometry)
            with self._lock:
                if (self._pose is not None and
                        self._heading_alignment is not None and
                        self._sensor_rotation is not None and
                        self._status == "STREAMING"):
                    break
            time.sleep(0.05)

        with self._lock:
            pose = self._pose
            status = self._status
            heading_alignment = self._heading_alignment
            sensor_rotation = self._sensor_rotation
        self.assertIsNotNone(pose)
        self.assertEqual(status, "STREAMING")
        self.assertEqual(pose.header.frame_id, "odom")
        expected_z = math.sin(math.radians(30.0))
        expected_w = math.cos(math.radians(30.0))
        self.assertAlmostEqual(pose.pose.pose.orientation.z,
                               expected_z, places=5)
        self.assertAlmostEqual(pose.pose.pose.orientation.w,
                               expected_w, places=5)
        self.assertAlmostEqual(heading_alignment.quaternion.z,
                               expected_z, places=5)
        self.assertAlmostEqual(heading_alignment.quaternion.w,
                               expected_w, places=5)
        self.assertAlmostEqual(sensor_rotation.quaternion.y,
                               -math.sin(math.radians(12.5)), places=5)
        self.assertAlmostEqual(sensor_rotation.quaternion.w,
                               math.cos(math.radians(12.5)), places=5)
        self.assertAlmostEqual(pose.pose.pose.position.x, 0.0, places=6)
        self.assertAlmostEqual(pose.pose.pose.position.y, 0.0, places=6)
        self.assertAlmostEqual(pose.pose.pose.position.z, 0.0, places=6)
        self.assertGreaterEqual(pose.pose.covariance[0], 0.01)
        self.assertGreaterEqual(pose.pose.covariance[21], 0.01)

    def test_brief_localization_health_false_is_debounced(self):
        def publish_sample(healthy):
            self._health_pub.publish(Bool(data=healthy))
            self._state_pub.publish(State(connected=True, armed=False))
            self._publish_px4_attitude()
            odometry = Odometry()
            odometry.header.stamp = rospy.Time.now()
            odometry.header.frame_id = "camera_init"
            odometry.child_frame_id = "aft_mapped"
            odometry.pose.pose.orientation.w = 1.0
            for index in (0, 7, 14):
                odometry.pose.covariance[index] = 0.01
            for index in (21, 28, 35):
                odometry.pose.covariance[index] = 0.02
            self._odom_pub.publish(odometry)

        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and not rospy.is_shutdown():
            publish_sample(True)
            with self._lock:
                if self._status == "STREAMING" and self._pose_count >= 3:
                    break
            time.sleep(0.03)
        with self._lock:
            self.assertEqual(self._status, "STREAMING")
            count_before_pulse = self._pose_count

        # The production grace is 0.30 s. A short optimizer-boundary pulse
        # must not interrupt the 30 Hz PX4 external-vision stream.
        pulse_end = time.monotonic() + 0.18
        while time.monotonic() < pulse_end and not rospy.is_shutdown():
            publish_sample(False)
            time.sleep(0.03)
        with self._lock:
            self.assertGreater(self._pose_count, count_before_pulse)
            self.assertNotEqual(self._status, "LIO_DEGENERATE")

        # A sustained unhealthy state still fails closed after the grace.
        fault_deadline = time.monotonic() + 0.50
        while time.monotonic() < fault_deadline and not rospy.is_shutdown():
            publish_sample(False)
            with self._lock:
                if self._status == "LIO_DEGENERATE":
                    break
            time.sleep(0.03)
        with self._lock:
            self.assertEqual(self._status, "LIO_DEGENERATE")
            count_after_fault = self._pose_count
        time.sleep(0.12)
        with self._lock:
            self.assertEqual(self._pose_count, count_after_fault)

    def test_isolated_speed_outlier_recovers_but_sustained_fault_stops(self):
        def publish_odometry(x):
            self._health_pub.publish(Bool(data=True))
            self._state_pub.publish(State(connected=True, armed=False))
            self._publish_px4_attitude()
            odometry = Odometry()
            odometry.header.stamp = rospy.Time.now()
            odometry.header.frame_id = "camera_init"
            odometry.child_frame_id = "aft_mapped"
            odometry.pose.pose.position.x = x
            odometry.pose.pose.orientation.w = 1.0
            for index in (0, 7, 14):
                odometry.pose.covariance[index] = 0.01
            for index in (21, 28, 35):
                odometry.pose.covariance[index] = 0.02
            self._odom_pub.publish(odometry)

        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and not rospy.is_shutdown():
            publish_odometry(0.0)
            with self._lock:
                if self._status == "STREAMING" and self._pose_count >= 3:
                    break
            time.sleep(0.05)

        with self._lock:
            self.assertEqual(self._status, "STREAMING")
        time.sleep(0.06)
        publish_odometry(0.5)  # <2 m jump, but far above the 2 m/s rate gate.

        deadline = time.monotonic() + 0.3
        while time.monotonic() < deadline and not rospy.is_shutdown():
            with self._lock:
                if "INPUT_SPEED_TRANSIENT" in self._statuses:
                    break
            time.sleep(0.01)
        with self._lock:
            self.assertIn("INPUT_SPEED_TRANSIENT", self._statuses)
        # A plausible sample relative to the last accepted input clears the
        # transient; the rejected pose was never made the velocity baseline.
        time.sleep(0.06)
        publish_odometry(0.02)
        deadline = time.monotonic() + 0.3
        while time.monotonic() < deadline and not rospy.is_shutdown():
            with self._lock:
                if self._status == "STREAMING":
                    break
            time.sleep(0.01)
        with self._lock:
            self.assertEqual(self._status, "STREAMING")

        # A genuinely sustained over-limit rate is still fail-closed after
        # two consecutive rejected samples.
        time.sleep(0.06)
        publish_odometry(0.5)
        time.sleep(0.06)
        publish_odometry(1.0)
        deadline = time.monotonic() + 0.3
        while time.monotonic() < deadline and not rospy.is_shutdown():
            with self._lock:
                # The timer may immediately refine the latched reason to the
                # current recovery state. Preserve the causal status from the
                # callback history instead of racing the timer's latest text.
                if "INPUT_SPEED" in self._statuses:
                    break
            time.sleep(0.01)
        with self._lock:
            self.assertIn("INPUT_SPEED", self._statuses)
            count_after_fault = self._pose_count
        time.sleep(0.12)
        with self._lock:
            self.assertEqual(self._pose_count, count_after_fault)

    def test_motion_prevents_startup_gravity_alignment(self):
        deadline = time.monotonic() + 1.5
        while time.monotonic() < deadline and not rospy.is_shutdown():
            self._health_pub.publish(Bool(data=True))
            self._state_pub.publish(State(connected=True, armed=False))
            self._publish_px4_attitude(sensor_angular_speed=0.3)
            odometry = Odometry()
            odometry.header.stamp = rospy.Time.now()
            odometry.header.frame_id = "camera_init"
            odometry.pose.pose.orientation.w = 1.0
            for index in (0, 7, 14):
                odometry.pose.covariance[index] = 0.01
            self._odom_pub.publish(odometry)
            time.sleep(0.02)

        with self._lock:
            self.assertIsNone(self._pose)
            self.assertIn("WAIT_STATIONARY_GRAVITY", self._statuses)

    def test_real_high_rate_rotation_streams_but_jump_latches_and_recovers(self):
        def publish_odometry(yaw, armed, auto):
            self._health_pub.publish(Bool(data=True))
            self._state_pub.publish(State(connected=True, armed=armed))
            self._auto_pub.publish(Bool(data=auto))
            self._publish_px4_attitude()
            odometry = Odometry()
            odometry.header.stamp = rospy.Time.now()
            odometry.header.frame_id = "camera_init"
            odometry.child_frame_id = "aft_mapped"
            odometry.pose.pose.orientation.z = math.sin(0.5 * yaw)
            odometry.pose.pose.orientation.w = math.cos(0.5 * yaw)
            for index in (0, 7, 14):
                odometry.pose.covariance[index] = 0.01
            for index in (21, 28, 35):
                odometry.pose.covariance[index] = 0.02
            self._odom_pub.publish(odometry)

        # Establish the only permitted alignment while disarmed.
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and not rospy.is_shutdown():
            publish_odometry(0.0, armed=False, auto=False)
            with self._lock:
                if self._status == "STREAMING" and self._pose_count >= 3:
                    break
            time.sleep(0.04)
        with self._lock:
            self.assertEqual(self._status, "STREAMING")
            alignment_count = self._heading_alignment_count
            start_pose_count = self._pose_count

        # 2.1 rad/s is just above the prediction threshold but well within the
        # physical envelope. It must remain healthy, disable extrapolation and
        # advertise inflated attitude covariance instead of latching off.
        yaw = 0.0
        last_stamp = time.monotonic()
        for _ in range(8):
            time.sleep(0.07)
            now = time.monotonic()
            yaw += 2.1 * (now - last_stamp)
            last_stamp = now
            publish_odometry(yaw, armed=True, auto=True)
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline and not rospy.is_shutdown():
            with self._lock:
                if ("STREAMING_HIGH_ANGULAR_RATE" in self._statuses and
                        self._pose_count > start_pose_count):
                    break
            time.sleep(0.02)
        with self._lock:
            self.assertIn("STREAMING_HIGH_ANGULAR_RATE", self._statuses)
            self.assertGreater(self._pose_count, start_pose_count)
            self.assertGreaterEqual(self._pose.pose.covariance[21], 0.35 ** 2)

        # A one-frame one-radian increment is not a plausible continuation and
        # must immediately latch, regardless of the rate confirmation count.
        yaw += 1.0
        publish_odometry(yaw, armed=True, auto=True)
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline and not rospy.is_shutdown():
            with self._lock:
                if "INPUT_ORIENTATION_JUMP" in self._statuses:
                    break
            time.sleep(0.01)
        with self._lock:
            self.assertIn("INPUT_ORIENTATION_JUMP", self._statuses)
            latched_pose_count = self._pose_count
        time.sleep(0.15)
        with self._lock:
            self.assertEqual(self._pose_count, latched_pose_count)

        # Stable data cannot clear the latch while automatic permission is
        # still high.
        for _ in range(5):
            time.sleep(0.06)
            publish_odometry(yaw, armed=True, auto=True)
        with self._lock:
            self.assertIn("JUMP_LATCHED_WAIT_AUTO_DISABLED", self._statuses)
            self.assertEqual(self._pose_count, latched_pose_count)

        # The pilot removes automatic authority but stays armed. Stable LIO may
        # resume the existing alignment; it must not recapture a new origin.
        for _ in range(6):
            time.sleep(0.06)
            publish_odometry(yaw, armed=True, auto=False)
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline and not rospy.is_shutdown():
            with self._lock:
                if self._pose_count > latched_pose_count:
                    break
            time.sleep(0.02)
        with self._lock:
            self.assertIn("RECOVERED_STABLE_MANUAL_DATA", self._statuses)
            self.assertGreater(self._pose_count, latched_pose_count)
            self.assertEqual(self._heading_alignment_count, alignment_count)


if __name__ == "__main__":
    rospy.init_node("test_omnilivo_px4_vision_bridge")
    rostest.rosrun("mine_uav_control", "omnilivo_px4_vision_bridge",
                   OmniLivoPx4VisionBridgeTest)
