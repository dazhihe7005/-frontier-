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
from geometry_msgs.msg import PoseStamped, TransformStamped, TwistStamped
from mavros_msgs.msg import PositionTarget, State
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud
from std_msgs.msg import Bool, Float32, String
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
        self.max_horizontal_accel = max(
            0.10, float(rospy.get_param("~max_horizontal_accel", 0.70)))
        self.max_vertical_accel = max(
            0.10, float(rospy.get_param("~max_vertical_accel", 0.45)))
        self.endpoint_standoff = max(
            0.0, float(rospy.get_param("~endpoint_standoff", 0.50)))
        self.endpoint_standoff_min_path = max(
            self.endpoint_standoff, float(rospy.get_param(
                "~endpoint_standoff_min_path", 1.0)))
        self.tracking_slowdown_error = max(
            0.05, float(rospy.get_param("~tracking_slowdown_error", 0.25)))
        self.tracking_hold_error = max(
            self.tracking_slowdown_error + 0.05,
            float(rospy.get_param("~tracking_hold_error", 0.60)))
        self.safety_radius = max(
            0.5, float(rospy.get_param("~safety_radius", 1.0)))
        self.proximity_slowdown_margin = max(
            0.10, float(rospy.get_param(
                "~proximity_slowdown_margin", 1.40)))
        self.proximity_hold_margin = min(
            self.proximity_slowdown_margin - 0.05,
            max(0.05, float(rospy.get_param(
                "~proximity_hold_margin", 0.65))))
        self.absolute_clearance_hold_margin = max(
            0.05, float(rospy.get_param(
                "~absolute_clearance_hold_margin", 0.25)))
        self.absolute_clearance_slowdown_margin = max(
            self.absolute_clearance_hold_margin + 0.05,
            float(rospy.get_param(
                "~absolute_clearance_slowdown_margin", 0.55)))
        self.frontier_guard_min_duration = max(
            1.0, float(rospy.get_param(
                "~frontier_guard_min_duration", 4.0)))
        self.frontier_guard_horizon = max(
            1.0, float(rospy.get_param(
                "~frontier_guard_horizon", 5.0)))
        self.frontier_hold_margin = max(
            self.proximity_hold_margin, float(rospy.get_param(
                "~frontier_hold_margin", 0.90)))
        self.frontier_slowdown_margin = max(
            self.frontier_hold_margin + 0.05, float(rospy.get_param(
                "~frontier_slowdown_margin", 1.70)))
        self.proximity_timeout = max(
            0.2, float(rospy.get_param("~proximity_timeout", 0.50)))
        self.proximity_block_timeout = max(
            0.5, float(rospy.get_param("~proximity_block_timeout", 2.0)))
        self.proximity_z_min = float(rospy.get_param("~proximity_z_min", -0.35))
        self.proximity_z_max = float(rospy.get_param("~proximity_z_max", 0.35))
        self.self_filter_z_min = float(rospy.get_param(
            "~self_filter_z_min", -0.32))
        self.self_filter_z_max = float(rospy.get_param(
            "~self_filter_z_max", 0.35))
        self.vertical_avoidance_margin = max(
            self.absolute_clearance_hold_margin, float(rospy.get_param(
                "~vertical_avoidance_margin", 0.30)))
        self.vertical_escape_distance = max(
            0.10, float(rospy.get_param(
                "~vertical_escape_distance", 0.35)))
        self.sensor_offset = tuple(float(value) for value in rospy.get_param(
            "~sensor_offset", [0.1315, 0.0, 0.223]))
        self.sensor_pitch = float(rospy.get_param("~sensor_pitch", 0.436332313))
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
        self._trajectory_alignment = None
        self._trajectory = None
        self._trajectory_start = rospy.Time(0)
        self._trajectory_progress = 0.0
        self._trajectory_end_progress = 0.0
        self._last_progress_tick = rospy.Time(0)
        self._odom = None
        self._odom_rx = rospy.Time(0)
        self._local_pose = None
        self._local_pose_rx = rospy.Time(0)
        self._local_velocity = None
        self._state = State()
        self._vision_healthy = False
        self._proximity = float("inf")
        self._spatial_clearance = float("inf")
        self._nearest_spatial_z = float("nan")
        self._path_margin = float("inf")
        self._proximity_rx = rospy.Time(0)
        self._last_command = None
        self._planned_velocity = (0.0, 0.0, 0.0)
        self._recovery_active = False
        self._proximity_blocked_since = rospy.Time(0)
        self._proximity_hold_pose = None
        self._vertical_escape_active = False
        self._blocked_state = False
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
        rospy.Subscriber("/mine_uav/gbplanner/recovery_active", Bool,
                         self._recovery_cb, queue_size=2)
        rospy.Subscriber("/mine_uav/gbplanner/planner_odometry", Odometry,
                         self._odom_cb, queue_size=20)
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped,
                         self._local_pose_cb, queue_size=20)
        rospy.Subscriber("/mavros/local_position/velocity_local", TwistStamped,
                         self._local_velocity_cb, queue_size=20)
        rospy.Subscriber("/mavros/state", State, self._state_cb, queue_size=10)
        rospy.Subscriber("/mine_uav/sitl/mid360/points", PointCloud,
                         self._pointcloud_cb, queue_size=1)
        self._setpoint_pub = rospy.Publisher(
            self.output_topic, PositionTarget, queue_size=20)
        self._ready_pub = rospy.Publisher(
            "/mine_uav/gbplanner/command_ready", Bool, queue_size=1, latch=True)
        self._status_pub = rospy.Publisher(
            "/mine_uav/gbplanner/executor_status", String, queue_size=1,
            latch=True)
        self._speed_scale_pub = rospy.Publisher(
            "/mine_uav/gbplanner/speed_scale", Float32, queue_size=5)
        self._tracking_error_pub = rospy.Publisher(
            "/mine_uav/gbplanner/tracking_error", Float32, queue_size=5)
        self._proximity_pub = rospy.Publisher(
            "/mine_uav/gbplanner/horizontal_clearance", Float32, queue_size=5)
        self._spatial_clearance_pub = rospy.Publisher(
            "/mine_uav/gbplanner/spatial_clearance", Float32, queue_size=5)
        self._path_margin_pub = rospy.Publisher(
            "/mine_uav/gbplanner/directional_safety_margin", Float32,
            queue_size=5)
        self._blocked_pub = rospy.Publisher(
            "/mine_uav/gbplanner/execution_blocked", Bool, queue_size=1,
            latch=True)
        rospy.Timer(rospy.Duration(1.0 / self.output_rate), self._timer)
        self._publish_ready(False)
        self._publish_status("WAIT_TRAJECTORY")
        self._blocked_pub.publish(Bool(data=False))

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
                    self._trajectory_alignment = None
                    self._publish_status("ALIGNMENT_CHANGED")
            self._alignment = (t.x, t.y, t.z, alignment_yaw)

    def _vision_cb(self, message):
        with self._lock:
            self._vision_healthy = bool(message.data)

    def _recovery_cb(self, message):
        with self._lock:
            self._recovery_active = bool(message.data)

    def _odom_cb(self, message):
        with self._lock:
            self._odom = message
            self._odom_rx = rospy.Time.now()

    def _local_pose_cb(self, message):
        with self._lock:
            self._local_pose = message
            self._local_pose_rx = rospy.Time.now()

    def _local_velocity_cb(self, message):
        with self._lock:
            self._local_velocity = message

    def _state_cb(self, message):
        with self._lock:
            self._state = message

    def _pointcloud_cb(self, message):
        """Measure absolute and motion-direction safety margins from MID360."""
        if rospy.is_shutdown():
            return
        c, s = math.cos(self.sensor_pitch), math.sin(self.sensor_pitch)
        ox, oy, oz = self.sensor_offset
        with self._lock:
            local_pose = self._local_pose
            local_velocity = self._local_velocity
            last_command = self._last_command
            planned_velocity = self._planned_velocity
            recovery_active = self._recovery_active
        vx = vy = vz = 0.0
        if local_velocity is not None:
            vx = local_velocity.twist.linear.x
            vy = local_velocity.twist.linear.y
            vz = local_velocity.twist.linear.z
        if last_command is not None:
            command_speed = math.sqrt(
                last_command.velocity.x**2 + last_command.velocity.y**2 +
                last_command.velocity.z**2)
            if command_speed > math.sqrt(vx*vx + vy*vy + vz*vz):
                vx = last_command.velocity.x
                vy = last_command.velocity.y
                vz = last_command.velocity.z
        planned_speed = math.sqrt(sum(value*value for value in planned_velocity))
        # During a backtrack the measured velocity can still point toward the
        # obstacle for a moment.  Evaluate the directional corridor along the
        # commanded reverse path, otherwise the recovery guard looks forward
        # and can permanently inhibit the only safe escape motion.
        if recovery_active and planned_speed > 0.10:
            vx, vy, vz = planned_velocity
        elif planned_speed > math.sqrt(vx*vx + vy*vy + vz*vz):
            vx, vy, vz = planned_velocity
        body_vx = body_vy = body_vz = 0.0
        body_speed = 0.0
        if local_pose is not None and finite(vx, vy, vz):
            try:
                yaw = yaw_of(local_pose.pose.orientation)
                body_vx = math.cos(yaw)*vx + math.sin(yaw)*vy
                body_vy = -math.sin(yaw)*vx + math.cos(yaw)*vy
                body_vz = vz
                body_speed = math.sqrt(
                    body_vx*body_vx + body_vy*body_vy + body_vz*body_vz)
            except ValueError:
                pass
        nearest_spatial = float("inf")
        nearest_spatial_z = float("nan")
        nearest_horizontal = float("inf")
        path_margin = float("inf")
        for point in message.points:
            if not finite(point.x, point.y, point.z):
                continue
            # Sensor frame -> FLU body frame (fixed +25 deg Y mount).
            bx = ox + c*point.x + s*point.z
            by = oy + point.y
            bz = oz - s*point.x + c*point.z
            horizontal = math.hypot(bx, by)
            # Ignore only the actual vehicle cylinder. The old unbounded
            # horizontal test discarded the floor directly below the UAV;
            # extending the cylinder down to -0.65 m also hid the floor until
            # collision was already unavoidable.
            if (horizontal <= 0.65 and
                    self.self_filter_z_min <= bz <= self.self_filter_z_max):
                continue
            spatial = math.sqrt(bx*bx + by*by + bz*bz)
            if spatial < nearest_spatial:
                nearest_spatial = spatial
                nearest_spatial_z = bz
            if self.proximity_z_min <= bz <= self.proximity_z_max:
                nearest_horizontal = min(nearest_horizontal, horizontal)
            if body_speed > 0.10:
                along = (bx*body_vx + by*body_vy + bz*body_vz) / body_speed
                cross = math.sqrt(max(0.0, spatial*spatial-along*along))
                if along > 0.0 and cross < self.safety_radius:
                    boundary = math.sqrt(
                        max(0.0, self.safety_radius**2-cross**2))
                    path_margin = min(path_margin, along-boundary)
        with self._lock:
            self._proximity = nearest_horizontal
            self._spatial_clearance = nearest_spatial
            self._nearest_spatial_z = nearest_spatial_z
            self._path_margin = path_margin
            self._proximity_rx = rospy.Time.now()
        if math.isfinite(nearest_horizontal):
            self._proximity_pub.publish(Float32(data=nearest_horizontal))
        if math.isfinite(nearest_spatial):
            self._spatial_clearance_pub.publish(Float32(data=nearest_spatial))
        if math.isfinite(path_margin):
            self._path_margin_pub.publish(Float32(data=path_margin))

    def _trajectory_cb(self, message):
        reason = self._validate(message)
        if reason:
            self._rejected += 1
            self._publish_status("REJECT_TRAJECTORY:" + reason)
            rospy.logerr("Rejected GBPlanner trajectory: %s", reason)
            return
        with self._lock:
            alignment = self._alignment
            local_pose = self._local_pose
            first = message.points[0].transforms[0].translation
            # PX4 deliberately keeps barometric height while FAST-LIO2 owns
            # horizontal position.  Re-anchor only Z at every trajectory
            # handoff so long-term lidar-odometry height drift cannot become
            # a physical climb, while preserving the plan's relative climbs
            # and descents inside this trajectory.
            z_correction = (local_pose.pose.position.z -
                            (alignment[2] + first.z))
            self._trajectory_alignment = (
                alignment[0], alignment[1],
                alignment[2] + z_correction, alignment[3])
            self._trajectory = message
            # Use receipt time so a CPU-heavy plan cannot make the first
            # points expire before the executor sees them.
            self._trajectory_start = rospy.Time.now()
            self._trajectory_progress = 0.0
            self._trajectory_end_progress = self._executable_end_time(message)
            self._last_progress_tick = self._trajectory_start
            self._accepted += 1
            self._proximity_blocked_since = rospy.Time(0)
        if abs(z_correction) > 0.05:
            rospy.logwarn("Trajectory Z re-anchored by %.3f m", z_correction)
        self._publish_blocked(False)
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
            alignment = self._alignment
            local_pose = self._local_pose
        if alignment is None or local_pose is None:
            return "ALIGNMENT_NOT_READY"
        if odom is not None:
            p = odom.pose.pose.position
            first_error = math.sqrt(
                (samples[0][0] - p.x) ** 2 +
                (samples[0][1] - p.y) ** 2 +
                (samples[0][2] - p.z) ** 2)
            if first_error > self.max_first_point_error:
                return "FIRST_POINT_JUMP"
        first_local = self._apply_alignment(samples[0], alignment)
        actual = local_pose.pose.position
        # Z is intentionally re-anchored to barometric local height on
        # acceptance.  XY must still be continuous because it is controlled
        # from FAST-LIO2 external vision and cannot be silently shifted.
        first_local_xy_error = math.hypot(
            first_local[0]-actual.x, first_local[1]-actual.y)
        if first_local_xy_error > self.max_first_point_error:
            return "FIRST_POINT_PX4_XY_JUMP"
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

    def _executable_end_time(self, message):
        """Leave a mapped-frontier standoff without shortening recovery paths."""
        times = [point.time_from_start.to_sec() for point in message.points]
        if (self.endpoint_standoff <= 0.0 or
                "gbplanner_stall_recovery_backtrack" in message.joint_names):
            return times[-1]
        positions = [point.transforms[0].translation for point in message.points]
        total = sum(math.sqrt(
            (positions[index].x-positions[index-1].x) ** 2 +
            (positions[index].y-positions[index-1].y) ** 2 +
            (positions[index].z-positions[index-1].z) ** 2)
                    for index in range(1, len(positions)))
        if total < self.endpoint_standoff_min_path:
            return times[-1]
        remaining = self.endpoint_standoff
        for index in range(len(positions)-1, 0, -1):
            segment = math.sqrt(
                (positions[index].x-positions[index-1].x) ** 2 +
                (positions[index].y-positions[index-1].y) ** 2 +
                (positions[index].z-positions[index-1].z) ** 2)
            if segment <= 1.0e-9:
                continue
            if remaining <= segment:
                ratio = remaining / segment
                return times[index] - ratio*(times[index]-times[index-1])
            remaining -= segment
        return times[0]

    @staticmethod
    def _apply_alignment(sample, alignment):
        x, y, z, yaw = sample
        tx, ty, tz, ayaw = alignment
        c, s = math.cos(ayaw), math.sin(ayaw)
        return (tx + c*x - s*y, ty + s*x + c*y, tz + z,
                wrap(ayaw + yaw))

    @staticmethod
    def _sample_velocity(message, elapsed):
        """Return the piecewise-linear feed-forward velocity in planner frame."""
        times = [point.time_from_start.to_sec() for point in message.points]
        if elapsed >= times[-1]:
            return (0.0, 0.0, 0.0)
        high = 1 if elapsed <= times[0] else bisect.bisect_right(times, elapsed)
        high = min(max(1, high), len(times) - 1)
        low = high - 1
        dt = times[high] - times[low]
        left = message.points[low].transforms[0].translation
        right = message.points[high].transforms[0].translation
        return ((right.x-left.x) / dt,
                (right.y-left.y) / dt,
                (right.z-left.z) / dt)

    @staticmethod
    def _apply_alignment_velocity(velocity, alignment):
        vx, vy, vz = velocity
        ayaw = alignment[3]
        c, s = math.cos(ayaw), math.sin(ayaw)
        return (c*vx - s*vy, s*vx + c*vy, vz)

    def _tracking_scale(self, error):
        """Slow virtual trajectory time before error can consume clearance."""
        if error <= self.tracking_slowdown_error:
            return 1.0
        if error >= self.tracking_hold_error:
            return 0.0
        span = self.tracking_hold_error - self.tracking_slowdown_error
        return (self.tracking_hold_error - error) / span

    def _proximity_scale(self, clearance, spatial_clearance, path_margin,
                         nearest_spatial_z, fresh, recovery_active=False,
                         frontier_approach=False):
        """Brake before motion intersects the live 1 m lidar envelope."""
        if not fresh:
            return 0.0
        vertical_hazard = (
            math.isfinite(nearest_spatial_z) and
            abs(nearest_spatial_z) > self.proximity_z_max and
            spatial_clearance <=
            self.safety_radius + self.vertical_avoidance_margin)
        if vertical_hazard:
            # A blind backtrack can include a small climb or descent and make
            # a floor/ceiling hazard worse.  Stop every trajectory mode here;
            # _timer will command a short motion away from the measured side.
            return 0.0
        if recovery_active:
            # This is the exact reverse of an already executed path at no more
            # than 0.45 m/s.  If a delayed observation has already put the
            # reference point inside the 1 m envelope, immobilising it cannot
            # restore clearance. Permit only a very slow retreat when the
            # live cloud confirms the reverse corridor itself is clear.
            if min(clearance, spatial_clearance) <= self.safety_radius:
                if path_margin > 0.20 or math.isinf(path_margin):
                    return 0.25
                return 0.0
            recovery_full = self.safety_radius + 0.30
            nearest_clearance = min(clearance, spatial_clearance)
            if nearest_clearance >= recovery_full:
                return 1.0
            return (nearest_clearance-self.safety_radius) / \
                (recovery_full-self.safety_radius)
        nearest_clearance = min(clearance, spatial_clearance)
        if nearest_clearance <= self.safety_radius:
            return 0.0
        absolute_hold = (
            self.safety_radius + self.absolute_clearance_hold_margin)
        absolute_full = (
            self.safety_radius + self.absolute_clearance_slowdown_margin)
        # Apply the absolute guard to the nearest 3-D return too.  Using only
        # the horizontal slice leaves the floor/ceiling without a braking
        # band and turns the 1 m threshold into an unrecoverable emergency
        # stop instead of a preventive limit.
        if nearest_clearance <= absolute_hold:
            absolute_scale = 0.0
        elif nearest_clearance >= absolute_full:
            absolute_scale = 1.0
        else:
            absolute_scale = ((nearest_clearance-absolute_hold) /
                              (absolute_full-absolute_hold))
        directional_hold = (self.frontier_hold_margin if frontier_approach
                            else self.proximity_hold_margin)
        directional_full = (self.frontier_slowdown_margin if frontier_approach
                            else self.proximity_slowdown_margin)
        if path_margin >= directional_full:
            directional_scale = 1.0
        elif path_margin <= directional_hold:
            directional_scale = 0.0
        else:
            span = directional_full-directional_hold
            directional_scale = (
                path_margin-directional_hold) / span
        return min(absolute_scale, directional_scale)

    def _publish_last_safe_hold(self, now, state, local_pose, local_fresh):
        """Keep OFFBOARD alive instead of dropping all setpoints on vision loss."""
        if not state.connected or not state.armed or not local_fresh:
            self._publish_ready(False)
            self._publish_status("PX4_NOT_READY")
            return
        # Always build a fresh zero-velocity command. Reusing _last_command
        # would retain its feed-forward velocity and is not a hold at all.
        command = PositionTarget()
        command.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        command.type_mask = (
            PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY |
            PositionTarget.IGNORE_AFZ | PositionTarget.IGNORE_YAW_RATE)
        position = local_pose.pose.position
        command.position.x = position.x
        command.position.y = position.y
        command.position.z = position.z
        command.velocity.x = 0.0
        command.velocity.y = 0.0
        command.velocity.z = 0.0
        try:
            command.yaw = yaw_of(local_pose.pose.orientation)
        except ValueError:
            command.yaw = 0.0
        command.header.stamp = now
        self._setpoint_pub.publish(command)
        with self._lock:
            self._last_command = command
        self._publish_ready(False)
        self._publish_status("FASTLIO_UNHEALTHY_HOLD")

    def _shape_velocity_and_yaw(self, velocity, yaw, speed_scale, now):
        """Limit command handoff acceleration and yaw discontinuities."""
        target = [component * speed_scale for component in velocity]
        with self._lock:
            previous = self._last_command
        if previous is None:
            return target, yaw
        dt = (now-previous.header.stamp).to_sec()
        if not finite(dt) or dt <= 0.0:
            dt = 1.0 / self.output_rate
        dt = min(dt, 2.0 / self.output_rate)

        # A hard safety/tracking stop takes precedence over smoothness. For
        # ordinary path handoffs, vector slew limiting prevents an immediate
        # +v to -v reversal that a 6 kg airframe cannot track.
        if speed_scale > 0.01:
            old_x, old_y = previous.velocity.x, previous.velocity.y
            dv_x, dv_y = target[0]-old_x, target[1]-old_y
            dv_norm = math.hypot(dv_x, dv_y)
            max_dv = self.max_horizontal_accel * dt
            if dv_norm > max_dv and dv_norm > 1.0e-9:
                ratio = max_dv / dv_norm
                target[0] = old_x + ratio*dv_x
                target[1] = old_y + ratio*dv_y
            max_dvz = self.max_vertical_accel * dt
            target[2] = min(previous.velocity.z+max_dvz,
                            max(previous.velocity.z-max_dvz, target[2]))
        else:
            target = [0.0, 0.0, 0.0]

        if finite(previous.yaw):
            max_dyaw = self.max_yaw_rate * dt
            yaw = wrap(previous.yaw + min(max_dyaw,
                                          max(-max_dyaw,
                                              wrap(yaw-previous.yaw))))
        return target, yaw

    def _timer(self, _event):
        if rospy.is_shutdown():
            return
        now = rospy.Time.now()
        with self._lock:
            trajectory = self._trajectory
            progress = self._trajectory_progress
            end_progress = self._trajectory_end_progress
            last_progress_tick = self._last_progress_tick
            alignment = self._trajectory_alignment
            vision = self._vision_healthy
            state = self._state
            local_pose = self._local_pose
            proximity = self._proximity
            spatial_clearance = self._spatial_clearance
            nearest_spatial_z = self._nearest_spatial_z
            path_margin = self._path_margin
            recovery_active = self._recovery_active
            vertical_escape_active = self._vertical_escape_active
            existing_hold_pose = self._proximity_hold_pose
            proximity_fresh = (
                not self._proximity_rx.is_zero() and
                (now-self._proximity_rx).to_sec() <= self.proximity_timeout)
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
            self._publish_last_safe_hold(
                now, state, local_pose, local_fresh)
            return
        if not state.connected or not state.armed or not local_fresh:
            self._publish_ready(False)
            self._publish_status("PX4_NOT_READY")
            return
        # Advance a virtual trajectory clock. In clear, well-tracked flight it
        # runs at real time. Tracking lag and live MID360 wall clearance can
        # only slow or stop it; neither guard can accelerate the command.
        desired_now = self._apply_alignment(
            self._sample(trajectory, progress), alignment)
        actual = local_pose.pose.position
        tracking_error = math.sqrt(
            (desired_now[0]-actual.x) ** 2 +
            (desired_now[1]-actual.y) ** 2 +
            (desired_now[2]-actual.z) ** 2)
        tracking_scale = self._tracking_scale(tracking_error)
        total_duration = trajectory.points[-1].time_from_start.to_sec()
        frontier_approach = (
            not recovery_active and
            total_duration >= self.frontier_guard_min_duration and
            end_progress-progress <= self.frontier_guard_horizon)
        proximity_scale = self._proximity_scale(
            proximity, spatial_clearance, path_margin, nearest_spatial_z,
            proximity_fresh,
            recovery_active,
            frontier_approach)
        vertical_detected = (
            proximity_fresh and math.isfinite(nearest_spatial_z) and
            abs(nearest_spatial_z) > self.proximity_z_max and
            spatial_clearance <=
            self.safety_radius + self.vertical_avoidance_margin)
        vertical_release_clearance = (
            self.safety_radius + self.vertical_avoidance_margin + 0.15)
        vertical_escape_pending = (
            vertical_escape_active and existing_hold_pose is not None and
            (abs(actual.z-existing_hold_pose[2]) > 0.08 or
             spatial_clearance < vertical_release_clearance))
        vertical_stop = vertical_detected or vertical_escape_pending
        if vertical_stop:
            # Keep both the trajectory clock and velocity command stopped for
            # the entire latched escape.  A single clear/noisy lidar frame is
            # not allowed to advance the old trajectory toward the obstacle.
            proximity_scale = 0.0
        proximity_stop = (proximity_scale <= 0.10 and
                          (not recovery_active or vertical_stop))
        with self._lock:
            if proximity_stop:
                if self._proximity_blocked_since.is_zero():
                    self._proximity_blocked_since = now
                # Capture the pose only on the transition into a safety stop.
                # Replacing this tuple with the newest measured pose every
                # timer tick would make the setpoint follow a coasting
                # aircraft and remove the PX4 position controller's braking
                # error.
                if self._proximity_hold_pose is None:
                    escape_z = actual.z
                    if vertical_stop:
                        # MID360/body Z is positive upward. Move opposite the
                        # closest vertical return: climb from a floor point,
                        # descend from a ceiling point.
                        direction = -1.0 if nearest_spatial_z > 0.0 else 1.0
                        escape_z += direction * self.vertical_escape_distance
                        escape_z = min(self.max_height,
                                       max(self.min_height, escape_z))
                        rospy.logwarn(
                            "Vertical obstacle escape: clearance %.3f m, "
                            "body z %.3f m, target dz %+.3f m",
                            spatial_clearance, nearest_spatial_z,
                            escape_z-actual.z)
                        self._vertical_escape_active = True
                    self._proximity_hold_pose = (
                        actual.x, actual.y, escape_z)
                blocked = ((now-self._proximity_blocked_since).to_sec() >=
                           self.proximity_block_timeout)
            else:
                self._proximity_blocked_since = rospy.Time(0)
                self._proximity_hold_pose = None
                self._vertical_escape_active = False
                blocked = False
            proximity_hold_pose = self._proximity_hold_pose
        self._publish_blocked(blocked)
        speed_scale = min(tracking_scale, proximity_scale)
        tick_dt = 0.0 if last_progress_tick.is_zero() else (now-last_progress_tick).to_sec()
        if not finite(tick_dt) or tick_dt < 0.0:
            tick_dt = 0.0
        tick_dt = min(tick_dt, 2.0 / self.output_rate)
        progress = min(end_progress, progress + tick_dt * speed_scale)
        sample = self._apply_alignment(self._sample(trajectory, progress), alignment)
        velocity = self._apply_alignment_velocity(
            self._sample_velocity(trajectory, progress), alignment)
        if progress >= end_progress - 1.0e-3:
            velocity = (0.0, 0.0, 0.0)
        # Keep the trajectory's intended direction separate from the command
        # sent to PX4.  A safety stop commands zero velocity, but the point
        # cloud guard must continue checking the blocked direction.  If the
        # zero command is fed back here, the directional corridor becomes
        # undefined on the next scan and the stop can incorrectly release.
        safety_velocity = velocity
        x, y, z, yaw = sample
        # Freezing virtual trajectory time is not itself a safety stop: the
        # frozen trajectory position can still be ahead of the aircraft, so
        # PX4 will continue closing that error toward the wall.  On a live
        # proximity stop, latch the current measured pose as the setpoint.
        if proximity_stop:
            x, y, z = proximity_hold_pose
            velocity = (0.0, 0.0, 0.0)
        if math.hypot(x, y) > self.max_horizontal_radius or not (
                self.min_height <= z <= self.max_height):
            self._publish_ready(False)
            self._publish_status("FLIGHT_VOLUME_VIOLATION")
            return
        velocity_command, yaw = self._shape_velocity_and_yaw(
            velocity, yaw, speed_scale, now)
        command = PositionTarget()
        command.header.stamp = now
        command.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        command.type_mask = (
            PositionTarget.IGNORE_AFX |
            PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW_RATE)
        command.position.x, command.position.y, command.position.z = x, y, z
        command.velocity.x = velocity_command[0]
        command.velocity.y = velocity_command[1]
        command.velocity.z = velocity_command[2]
        command.yaw = yaw
        try:
            self._setpoint_pub.publish(command)
        except rospy.ROSException:
            if rospy.is_shutdown():
                return
            raise
        with self._lock:
            if trajectory is self._trajectory:
                self._trajectory_progress = progress
                self._last_progress_tick = now
            self._owner = True
            self._last_command = command
            self._planned_velocity = safety_velocity
        self._speed_scale_pub.publish(Float32(data=speed_scale))
        self._tracking_error_pub.publish(Float32(data=tracking_error))
        if proximity_scale < 0.999:
            rospy.logwarn_throttle(
                1.0,
                "MID360 predictive scale %.2f, horizontal %.3f m, "
                "spatial %.3f m, directional margin %.3f m",
                proximity_scale, proximity, spatial_clearance, path_margin)
        elif tracking_scale < 0.999:
            rospy.logwarn_throttle(
                1.0, "Adaptive speed scale %.2f, tracking error %.3f m",
                tracking_scale, tracking_error)
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

    def _publish_blocked(self, value):
        if value == self._blocked_state:
            return
        self._blocked_state = value
        self._blocked_pub.publish(Bool(data=value))


if __name__ == "__main__":
    rospy.init_node("gbplanner_px4_executor")
    GbplannerPx4Executor()
    rospy.spin()
