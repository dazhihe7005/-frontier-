#!/usr/bin/env python3
"""Safely execute GBPlanner2 trajectories through MAVROS/PX4.

GBPlanner2 remains the geometric planner.  This node only time-samples its
MultiDOF trajectory into PX4 local position/yaw setpoints; all flight-control
loops remain inside PX4.
"""

import bisect
import math
import threading

import rospy
from geometry_msgs.msg import PoseStamped, TransformStamped
from mavros_msgs.msg import PositionTarget, State
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from trajectory_msgs.msg import MultiDOFJointTrajectory


def finite(*values):
    return all(math.isfinite(value) for value in values)


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_of(q):
    norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
    if not finite(norm) or norm < 1.0e-8:
        raise ValueError("invalid quaternion")
    x, y, z, w = q.x / norm, q.y / norm, q.z / norm, q.w / norm
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class GbplannerPx4Executor:
    def __init__(self):
        self.trajectory_topic = rospy.get_param(
            "~trajectory_topic", "/gbplanner/command/trajectory")
        self.expected_frame = rospy.get_param("~expected_frame", "camera_init")
        self.output_topic = rospy.get_param(
            "~output_topic", "/mavros/setpoint_raw/local")
        self.output_rate = max(20.0, float(rospy.get_param("~output_rate", 50.0)))
        self.max_speed = max(0.1, float(rospy.get_param("~max_speed", 0.85)))
        self.max_yaw_rate = max(
            0.05, float(rospy.get_param("~max_yaw_rate", 0.45)))
        self.max_first_point_error = max(
            0.2, float(rospy.get_param("~max_first_point_error", 1.5)))
        self.local_pose_timeout = max(
            0.1, float(rospy.get_param("~local_pose_timeout", 0.5)))
        self.odom_timeout = max(0.1, float(rospy.get_param("~odom_timeout", 0.5)))
        self.max_horizontal_radius = max(
            5.0, float(rospy.get_param("~max_horizontal_radius", 220.0)))
        self.min_height = float(rospy.get_param("~min_height", -3.0))
        self.max_height = float(rospy.get_param("~max_height", 10.0))
        self._lock = threading.Lock()
        self._alignment = None
        self._trajectory = None
        self._trajectory_start = rospy.Time(0)
        self._odom = None
        self._odom_rx = rospy.Time(0)
        self._local_pose = None
        self._local_pose_rx = rospy.Time(0)
        self._state = State()
        self._vision_healthy = False
        self._owner = False
        self._last_ready = None
        self._last_status = None
        self._accepted = 0
        self._rejected = 0

        rospy.Subscriber(self.trajectory_topic, MultiDOFJointTrajectory,
                         self._trajectory_cb, queue_size=3)
        rospy.Subscriber("/mine_uav/gbplanner/fastlio_to_px4_alignment",
                         TransformStamped, self._alignment_cb, queue_size=1)
        rospy.Subscriber("/mine_uav/gbplanner/vision_healthy", Bool,
                         self._vision_cb, queue_size=2)
        rospy.Subscriber("/mine_uav/gbplanner/planner_odometry", Odometry,
                         self._odom_cb, queue_size=20)
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped,
                         self._local_pose_cb, queue_size=20)
        rospy.Subscriber("/mavros/state", State, self._state_cb, queue_size=10)
        self._setpoint_pub = rospy.Publisher(
            self.output_topic, PositionTarget, queue_size=20)
        self._ready_pub = rospy.Publisher(
            "/mine_uav/gbplanner/command_ready", Bool, queue_size=1, latch=True)
        self._status_pub = rospy.Publisher(
            "/mine_uav/gbplanner/executor_status", String, queue_size=1,
            latch=True)
        rospy.Timer(rospy.Duration(1.0 / self.output_rate), self._timer)
        self._publish_ready(False)
        self._publish_status("WAIT_TRAJECTORY")

    def _alignment_cb(self, message):
        try:
            alignment_yaw = yaw_of(message.transform.rotation)
        except ValueError:
            self._publish_status("INVALID_ALIGNMENT")
            return
        t = message.transform.translation
        if not finite(t.x, t.y, t.z, alignment_yaw):
            self._publish_status("INVALID_ALIGNMENT")
            return
        with self._lock:
            if self._owner and self._alignment is not None:
                old = self._alignment
                translation_change = math.sqrt(
                    (t.x - old[0]) ** 2 + (t.y - old[1]) ** 2 +
                    (t.z - old[2]) ** 2)
                if translation_change > 0.05 or abs(wrap(alignment_yaw-old[3])) > 0.05:
                    self._trajectory = None
                    self._publish_status("ALIGNMENT_CHANGED")
            self._alignment = (t.x, t.y, t.z, alignment_yaw)

    def _vision_cb(self, message):
        with self._lock:
            self._vision_healthy = bool(message.data)

    def _odom_cb(self, message):
        with self._lock:
            self._odom = message
            self._odom_rx = rospy.Time.now()

    def _local_pose_cb(self, message):
        with self._lock:
            self._local_pose = message
            self._local_pose_rx = rospy.Time.now()

    def _state_cb(self, message):
        with self._lock:
            self._state = message

    def _trajectory_cb(self, message):
        reason = self._validate(message)
        if reason:
            self._rejected += 1
            self._publish_status("REJECT_TRAJECTORY:" + reason)
            rospy.logerr("Rejected GBPlanner trajectory: %s", reason)
            return
        with self._lock:
            self._trajectory = message
            # Use receipt time so a CPU-heavy plan cannot make the first
            # points expire before the executor sees them.
            self._trajectory_start = rospy.Time.now()
            self._accepted += 1
        self._publish_status("TRAJECTORY_ACCEPTED")
        rospy.loginfo("Accepted GBPlanner trajectory %u with %d samples",
                      message.header.seq, len(message.points))

    def _validate(self, message):
        if message.header.frame_id != self.expected_frame:
            return "FRAME_" + message.header.frame_id
        if len(message.points) < 2:
            return "TOO_SHORT"
        times, samples = [], []
        for point in message.points:
            if not point.transforms:
                return "MISSING_TRANSFORM"
            transform = point.transforms[0]
            p = transform.translation
            try:
                yaw = yaw_of(transform.rotation)
            except ValueError:
                return "BAD_QUATERNION"
            seconds = point.time_from_start.to_sec()
            if not finite(p.x, p.y, p.z, yaw, seconds):
                return "NONFINITE"
            if times and seconds <= times[-1]:
                return "NON_MONOTONIC_TIME"
            times.append(seconds)
            samples.append((p.x, p.y, p.z, yaw))
        for index in range(1, len(samples)):
            dt = times[index] - times[index - 1]
            distance = math.sqrt(sum(
                (samples[index][axis] - samples[index - 1][axis]) ** 2
                for axis in range(3)))
            if distance / dt > self.max_speed * 1.20:
                return "SPEED_LIMIT"
            if abs(wrap(samples[index][3] - samples[index-1][3])) / dt > \
                    self.max_yaw_rate * 1.20:
                return "YAW_RATE_LIMIT"
        with self._lock:
            odom = self._odom
        if odom is not None:
            p = odom.pose.pose.position
            first_error = math.sqrt(
                (samples[0][0] - p.x) ** 2 +
                (samples[0][1] - p.y) ** 2 +
                (samples[0][2] - p.z) ** 2)
            if first_error > self.max_first_point_error:
                return "FIRST_POINT_JUMP"
        return ""

    @staticmethod
    def _sample(message, elapsed):
        times = [point.time_from_start.to_sec() for point in message.points]
        if elapsed <= times[0]:
            low = high = 0
            ratio = 0.0
        elif elapsed >= times[-1]:
            low = high = len(times) - 1
            ratio = 0.0
        else:
            high = bisect.bisect_right(times, elapsed)
            low = high - 1
            ratio = (elapsed - times[low]) / (times[high] - times[low])
        left = message.points[low].transforms[0]
        right = message.points[high].transforms[0]
        lp, rp = left.translation, right.translation
        lyaw, ryaw = yaw_of(left.rotation), yaw_of(right.rotation)
        return (
            lp.x + ratio * (rp.x - lp.x),
            lp.y + ratio * (rp.y - lp.y),
            lp.z + ratio * (rp.z - lp.z),
            wrap(lyaw + ratio * wrap(ryaw - lyaw)),
        )

    @staticmethod
    def _apply_alignment(sample, alignment):
        x, y, z, yaw = sample
        tx, ty, tz, ayaw = alignment
        c, s = math.cos(ayaw), math.sin(ayaw)
        return (tx + c*x - s*y, ty + s*x + c*y, tz + z,
                wrap(ayaw + yaw))

    def _timer(self, _event):
        now = rospy.Time.now()
        with self._lock:
            trajectory = self._trajectory
            start = self._trajectory_start
            alignment = self._alignment
            vision = self._vision_healthy
            state = self._state
            odom_fresh = (not self._odom_rx.is_zero() and
                          (now-self._odom_rx).to_sec() <= self.odom_timeout)
            local_fresh = (not self._local_pose_rx.is_zero() and
                           (now-self._local_pose_rx).to_sec() <= self.local_pose_timeout)
        if trajectory is None:
            self._publish_ready(False)
            self._publish_status("WAIT_TRAJECTORY")
            return
        if alignment is None:
            self._publish_ready(False)
            self._publish_status("WAIT_ALIGNMENT")
            return
        if not vision or not odom_fresh:
            self._publish_ready(False)
            self._publish_status("FASTLIO_UNHEALTHY")
            return
        if not state.connected or not state.armed or not local_fresh:
            self._publish_ready(False)
            self._publish_status("PX4_NOT_READY")
            return
        sample = self._apply_alignment(
            self._sample(trajectory, max(0.0, (now-start).to_sec())), alignment)
        x, y, z, yaw = sample
        if math.hypot(x, y) > self.max_horizontal_radius or not (
                self.min_height <= z <= self.max_height):
            self._publish_ready(False)
            self._publish_status("FLIGHT_VOLUME_VIOLATION")
            return
        command = PositionTarget()
        command.header.stamp = now
        command.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        command.type_mask = (
            PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY |
            PositionTarget.IGNORE_VZ | PositionTarget.IGNORE_AFX |
            PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW_RATE)
        command.position.x, command.position.y, command.position.z = x, y, z
        command.yaw = yaw
        self._setpoint_pub.publish(command)
        with self._lock:
            self._owner = True
        self._publish_ready(True)
        self._publish_status("STREAMING_TO_PX4")

    def _publish_ready(self, value):
        if value == self._last_ready:
            return
        self._last_ready = value
        self._ready_pub.publish(Bool(data=value))

    def _publish_status(self, value):
        if value == self._last_status:
            return
        self._last_status = value
        self._status_pub.publish(String(data=value))


if __name__ == "__main__":
    rospy.init_node("gbplanner_px4_executor")
    GbplannerPx4Executor()
    rospy.spin()
