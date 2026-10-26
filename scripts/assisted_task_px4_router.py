#!/usr/bin/env python3
"""Single production PX4 command owner for assisted Tasks 2 and 3."""

import math
import time

import rospy
import sensor_msgs.point_cloud2 as point_cloud2
from geometry_msgs.msg import PoseStamped, QuaternionStamped, TwistStamped
from mavros_msgs.msg import EstimatorStatus, PositionTarget, State
from mavros_msgs.srv import SetMode
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, String, UInt8


TASK_SHAFT = 2
TASK_PRESSURE = 3
TASK3_HARD_MAX_HORIZONTAL_SPEED = 1.6


def quaternion_yaw(q):
    norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
    if not math.isfinite(norm) or norm < 1.0e-9:
        return None
    x, y, z, w = q.x / norm, q.y / norm, q.z / norm, q.w / norm
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class ClearanceAssistCore:
    """Max-clearance centering preference plus 2-D CBF-style constraints."""

    def __init__(
        self,
        hard_distance=1.0,
        release_distance=1.2,
        brake_distance=1.8,
        max_speed=0.25,
        max_acceleration=0.4,
        escape_speed=0.10,
        center_gain=0.5,
        search_radius=0.8,
        search_resolution=0.2,
        coverage_sectors=16,
        minimum_covered_sectors=10,
        maximum_wall_range=6.0,
    ):
        self.hard_distance = float(hard_distance)
        self.release_distance = float(release_distance)
        self.brake_distance = float(brake_distance)
        self.max_speed = float(max_speed)
        self.max_acceleration = float(max_acceleration)
        self.escape_speed = float(escape_speed)
        self.center_gain = float(center_gain)
        self.search_radius = float(search_radius)
        self.search_resolution = float(search_resolution)
        self.coverage_sectors = int(coverage_sectors)
        self.minimum_covered_sectors = int(minimum_covered_sectors)
        self.maximum_wall_range = float(maximum_wall_range)
        if not (
            0.0 < self.hard_distance < self.release_distance < self.brake_distance
            and self.max_speed > 0.0
            and self.max_acceleration > 0.0
            and self.escape_speed > 0.0
            and self.search_radius > 0.0
            and self.search_resolution > 0.0
            and 1 <= self.minimum_covered_sectors <= self.coverage_sectors
        ):
            raise ValueError("invalid shaft horizontal safety configuration")
        self.previous_velocity = (0.0, 0.0)
        self.state = "WAIT_ENVIRONMENT"
        self.escape_latched = False

    @staticmethod
    def _limit(vector_x, vector_y, maximum):
        norm = math.hypot(vector_x, vector_y)
        if norm <= maximum or norm < 1.0e-12:
            return vector_x, vector_y
        scale = maximum / norm
        return vector_x * scale, vector_y * scale

    def _coverage(self, points):
        sectors = set()
        for x, y in points:
            distance = math.hypot(x, y)
            if distance < 1.0e-6 or distance > self.maximum_wall_range:
                continue
            angle = (math.atan2(y, x) + math.pi) / (2.0 * math.pi)
            sectors.add(min(self.coverage_sectors - 1,
                            int(angle * self.coverage_sectors)))
        return len(sectors)

    @staticmethod
    def _clearance(points, candidate_x, candidate_y):
        if not points:
            return float("inf")
        return min(
            math.hypot(x - candidate_x, y - candidate_y) for x, y in points
        )

    def _solve_qp(self, preferred_x, preferred_y, constraints):
        """Solve min ||v-v_pref||^2 for 2-D CBF half-spaces and speed ball.

        In two dimensions the optimum is the preference itself or lies on one
        or two active boundaries. Enumerating those candidates is exact here
        and avoids a heavyweight optimizer in the flight process.
        """
        tolerance = 1.0e-7

        def feasible(x, y):
            return (
                math.hypot(x, y) <= self.max_speed + tolerance
                and all(nx * x + ny * y >= lower - tolerance
                        for _, nx, ny, lower in constraints)
            )

        candidates = []

        def add(x, y):
            if math.isfinite(x) and math.isfinite(y) and feasible(x, y):
                cost = (x - preferred_x) ** 2 + (y - preferred_y) ** 2
                candidates.append((cost, x, y))

        clipped_x, clipped_y = self._limit(
            preferred_x, preferred_y, self.max_speed
        )
        add(clipped_x, clipped_y)

        for _, nx, ny, lower in constraints:
            projection = nx * preferred_x + ny * preferred_y
            add(
                preferred_x + (lower - projection) * nx,
                preferred_y + (lower - projection) * ny,
            )
            tangent_x, tangent_y = -ny, nx
            remaining = self.max_speed ** 2 - lower ** 2
            if remaining >= -tolerance:
                tangent = math.sqrt(max(0.0, remaining))
                add(lower * nx + tangent * tangent_x,
                    lower * ny + tangent * tangent_y)
                add(lower * nx - tangent * tangent_x,
                    lower * ny - tangent * tangent_y)

        for first in range(len(constraints)):
            _, nx1, ny1, lower1 = constraints[first]
            for second in range(first + 1, len(constraints)):
                _, nx2, ny2, lower2 = constraints[second]
                determinant = nx1 * ny2 - ny1 * nx2
                if abs(determinant) < 1.0e-9:
                    continue
                add(
                    (lower1 * ny2 - ny1 * lower2) / determinant,
                    (nx1 * lower2 - lower1 * nx2) / determinant,
                )

        if not candidates:
            return None
        _, result_x, result_y = min(candidates, key=lambda item: item[0])
        return result_x, result_y

    def _centering_preference(self, points):
        if self._coverage(points) < self.minimum_covered_sectors:
            return 0.0, 0.0, False
        steps = int(math.floor(self.search_radius / self.search_resolution))
        best = (self._clearance(points, 0.0, 0.0), 0.0, 0.0)
        for ix in range(-steps, steps + 1):
            for iy in range(-steps, steps + 1):
                x = ix * self.search_resolution
                y = iy * self.search_resolution
                if math.hypot(x, y) > self.search_radius + 1.0e-9:
                    continue
                clearance = self._clearance(points, x, y)
                if clearance < self.hard_distance:
                    continue
                score = clearance - 0.08 * math.hypot(x, y)
                if score > best[0] + 1.0e-9:
                    best = (score, x, y)
        return (*self._limit(
            best[1] * self.center_gain,
            best[2] * self.center_gain,
            self.max_speed,
        ), True)

    def step(self, points, dt):
        clean = [
            (float(x), float(y))
            for x, y in points
            if math.isfinite(x) and math.isfinite(y)
            and math.hypot(x, y) <= self.maximum_wall_range
        ]
        if not clean:
            self.previous_velocity = (0.0, 0.0)
            self.state = "WAIT_ENVIRONMENT"
            return 0.0, 0.0, False, self.state

        preferred_x, preferred_y, coverage_ready = self._centering_preference(clean)
        nearest = min(math.hypot(x, y) for x, y in clean)
        if nearest <= self.hard_distance:
            self.escape_latched = True
        elif nearest >= self.release_distance:
            self.escape_latched = False
        constraints = []
        alpha = self.max_speed / max(
            1.0e-6, self.brake_distance - self.hard_distance
        )
        for x, y in clean:
            distance = math.hypot(x, y)
            if distance < 1.0e-6 or distance > self.brake_distance:
                continue
            away_x, away_y = -x / distance, -y / distance
            lower = self.escape_speed if (
                self.escape_latched and distance < self.release_distance
            ) else -alpha * (distance - self.hard_distance)
            constraints.append((distance, away_x, away_y, lower))
        constraints.sort(key=lambda item: item[0])

        solution = self._solve_qp(preferred_x, preferred_y, constraints)
        if solution is None:
            self.previous_velocity = (0.0, 0.0)
            self.state = "NO_FEASIBLE_ESCAPE"
            return 0.0, 0.0, False, self.state
        vx, vy = solution

        dt = max(0.0, min(float(dt), 0.2))
        old_x, old_y = self.previous_velocity
        delta_x, delta_y = vx - old_x, vy - old_y
        delta_norm = math.hypot(delta_x, delta_y)
        maximum_delta = self.max_acceleration * dt
        if delta_norm > maximum_delta and delta_norm > 1.0e-12:
            scale = maximum_delta / delta_norm
            limited_x = old_x + delta_x * scale
            limited_y = old_y + delta_y * scale
            # Acceleration shaping is subordinate to a CBF safety boundary.
            # If slew limiting would re-enter an obstacle, use the QP result.
            if all(nx * limited_x + ny * limited_y >= lower - 1.0e-6
                   for _, nx, ny, lower in constraints):
                vx, vy = limited_x, limited_y
        self.previous_velocity = (vx, vy)
        if self.escape_latched:
            self.state = "ESCAPING"
        elif constraints:
            self.state = "AVOIDING"
        elif coverage_ready:
            self.state = "CENTERING"
        else:
            self.state = "WAIT_COVERAGE"
        return vx, vy, coverage_ready, self.state


class AssistedTaskPx4Router:
    def __init__(self):
        self.timeout = float(rospy.get_param("~input_timeout", 0.5))
        # MAVROS publishes State and EstimatorStatus at about 1 Hz on the
        # production FCU.  They need their own freshness windows: sharing the
        # 0.5 s high-rate command timeout makes navigation_ready() fail between
        # every pair of healthy samples and retracts a newly accepted OFFBOARD
        # request.  Local pose and task commands remain on the fast timeout.
        self.state_timeout = float(rospy.get_param("~state_timeout", 1.5))
        self.estimator_timeout = float(
            rospy.get_param("~estimator_timeout", 1.5)
        )
        if not math.isfinite(self.state_timeout) or self.state_timeout <= 0.0:
            raise ValueError("state_timeout must be finite and positive")
        if (
            not math.isfinite(self.estimator_timeout)
            or self.estimator_timeout <= 0.0
        ):
            raise ValueError("estimator_timeout must be finite and positive")
        self.prestream_duration = float(rospy.get_param("~prestream_duration", 1.0))
        self.mode_request_interval = float(rospy.get_param("~mode_request_interval", 1.0))
        self.offboard_mode = rospy.get_param("~offboard_mode", "OFFBOARD")
        self.fallback_mode = rospy.get_param("~fallback_mode", "POSCTL")
        self.expected_frame = rospy.get_param("~expected_frame", "task1_planning")
        configured_task3_speed = float(
            rospy.get_param(
                "~task3_max_horizontal_speed", TASK3_HARD_MAX_HORIZONTAL_SPEED
            )
        )
        if not math.isfinite(configured_task3_speed) or configured_task3_speed <= 0.0:
            raise ValueError("task3_max_horizontal_speed must be finite and positive")
        # Keep the final PX4 owner independently bounded even if the isolated
        # pressure controller is accidentally misconfigured or replaced.
        self.task3_max_horizontal_speed = min(
            configured_task3_speed, TASK3_HARD_MAX_HORIZONTAL_SPEED
        )
        self.vertical_below = float(rospy.get_param("~vertical_below", 0.5))
        self.vertical_above = float(rospy.get_param("~vertical_above", 0.5))
        self.max_points = int(rospy.get_param("~max_points", 100000))
        self.clearance = ClearanceAssistCore(
            hard_distance=rospy.get_param("~hard_distance", 1.0),
            release_distance=rospy.get_param("~release_distance", 1.2),
            brake_distance=rospy.get_param("~brake_distance", 1.8),
            max_speed=rospy.get_param("~max_horizontal_speed", 0.25),
            max_acceleration=rospy.get_param("~max_horizontal_acceleration", 0.4),
            escape_speed=rospy.get_param("~escape_speed", 0.10),
            center_gain=rospy.get_param("~center_gain", 0.5),
        )

        self.active_task = 0
        self.shaft_enabled = False
        self.task3_enabled = False
        self.task3_controller_ready = False
        self.task3_controller_ready_time = None
        self.shaft_status = "IDLE"
        self.shaft_intent = None
        self.shaft_intent_time = None
        self.task3_command = None
        self.task3_command_time = None
        self.task_odometry = None
        self.task_odometry_time = None
        self.relative_points = []
        self.cloud_time = None
        self.cloud_valid = False
        self.heading_yaw = None
        self.heading_time = None
        self.local_pose = None
        self.local_pose_time = None
        self.state = State()
        self.state_time = None
        self.estimator = EstimatorStatus()
        self.estimator_time = None
        self.prestream_started = None
        self.last_mode_request = 0.0
        self.owned_task = 0
        # An OFFBOARD mode request can be accepted by PX4 before the next
        # /mavros/state sample reports it.  Keep that request latched so a task
        # cancellation cannot leave a late OFFBOARD transition unowned.
        self.offboard_request_pending = False
        self.fallback_pending = False
        self.fallback_request_time = None
        self.fallback_mode_seen_since = None
        self.takeover_latched = False
        self.hold_z = None
        self.hold_yaw = None
        self.failsafe_position = None
        self.last_tick = time.monotonic()

        self.command_pub = rospy.Publisher(
            rospy.get_param("~output_topic", "/mine_uav/setpoint_cmd"),
            PositionTarget,
            queue_size=10,
        )
        self.task2_ready_pub = rospy.Publisher(
            "/mine_uav/task2/command_ready", Bool, queue_size=1, latch=True
        )
        self.task3_ready_pub = rospy.Publisher(
            "/mine_uav/task3/command_ready", Bool, queue_size=1, latch=True
        )
        self.status_pub = rospy.Publisher(
            "/mine_uav/assisted_router/status", String, queue_size=10
        )
        self.mode_client = rospy.ServiceProxy("/mavros/set_mode", SetMode)

        rospy.Subscriber("/mine_uav/mission/active_task", UInt8, self.active_task_cb)
        rospy.Subscriber("/mine_uav/mission/shaft_enable", Bool, self.shaft_enable_cb)
        rospy.Subscriber("/mine_uav/mission/task3_enable", Bool, self.task3_enable_cb)
        rospy.Subscriber("/mine_uav/shaft/status", String, self.shaft_status_cb)
        rospy.Subscriber("/mine_uav/shaft/velocity_intent_enu", TwistStamped,
                         self.shaft_intent_cb)
        rospy.Subscriber("/mine_uav/task3/controller_ready", Bool,
                         self.task3_ready_cb)
        rospy.Subscriber("/mine_uav/task3/setpoint_cmd", PositionTarget,
                         self.task3_command_cb)
        rospy.Subscriber("/mine_uav/task1/odometry", Odometry, self.odometry_cb)
        rospy.Subscriber("/mine_uav/task1/cloud_registered", PointCloud2, self.cloud_cb,
                         queue_size=2)
        rospy.Subscriber("/mine_uav/task1/vision_heading_from_planning",
                         QuaternionStamped, self.heading_cb)
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped,
                         self.local_pose_cb)
        rospy.Subscriber("/mavros/state", State, self.state_cb)
        rospy.Subscriber("/mavros/estimator_status", EstimatorStatus,
                         self.estimator_cb)
        rate = max(10.0, float(rospy.get_param("~rate", 30.0)))
        rospy.Timer(rospy.Duration(1.0 / rate), self.tick)
        self.task2_ready_pub.publish(Bool(data=False))
        self.task3_ready_pub.publish(Bool(data=False))

    @staticmethod
    def _fresh(stamp, now, timeout):
        return stamp is not None and 0.0 <= now - stamp <= timeout

    def active_task_cb(self, message):
        self.active_task = int(message.data)

    def shaft_enable_cb(self, message):
        self.shaft_enabled = bool(message.data)
        if not self.shaft_enabled:
            self.reset_task()

    def task3_enable_cb(self, message):
        self.task3_enabled = bool(message.data)
        if not self.task3_enabled:
            self.reset_task()

    def shaft_status_cb(self, message):
        self.shaft_status = message.data

    def shaft_intent_cb(self, message):
        self.shaft_intent = message
        self.shaft_intent_time = time.monotonic()

    def task3_ready_cb(self, message):
        self.task3_controller_ready = bool(message.data)
        self.task3_controller_ready_time = time.monotonic()

    def task3_command_cb(self, message):
        self.task3_command = message
        self.task3_command_time = time.monotonic()

    def odometry_cb(self, message):
        self.task_odometry = message
        self.task_odometry_time = time.monotonic()

    def heading_cb(self, message):
        self.heading_yaw = quaternion_yaw(message.quaternion)
        self.heading_time = time.monotonic()

    def local_pose_cb(self, message):
        self.local_pose = message
        self.local_pose_time = time.monotonic()

    def state_cb(self, message):
        if (
            self.owned_task
            and not self.fallback_pending
            and self.state.mode == self.offboard_mode
            and message.mode != self.offboard_mode
        ):
            self.takeover_latched = True
            self.owned_task = 0
        self.state = message
        self.state_time = time.monotonic()

    def estimator_cb(self, message):
        self.estimator = message
        self.estimator_time = time.monotonic()

    def cloud_cb(self, message):
        now = time.monotonic()
        # Only Task 2 consumes the assisted router's horizontal-clearance
        # cloud.  Decoding up to 100k Python points while the shaft task is
        # idle (or while Task 3 is selected) needlessly competes with the
        # exploration planner on the flight computer.  Keep the input
        # explicitly unavailable until a fresh post-enable cloud arrives.
        if self.active_task != TASK_SHAFT or not self.shaft_enabled:
            self.relative_points = []
            self.cloud_valid = False
            self.cloud_time = None
            return
        if (
            message.header.frame_id != self.expected_frame
            or not self._fresh(self.task_odometry_time, now, self.timeout)
            or self.task_odometry is None
        ):
            self.cloud_valid = False
            self.cloud_time = now
            return
        position = self.task_odometry.pose.pose.position
        points = []
        try:
            for index, point in enumerate(point_cloud2.read_points(
                message, field_names=("x", "y", "z"), skip_nans=True
            )):
                if index >= self.max_points:
                    break
                dz = float(point[2]) - position.z
                if -self.vertical_below <= dz <= self.vertical_above:
                    points.append((float(point[0]) - position.x,
                                   float(point[1]) - position.y))
        except (AssertionError, KeyError, RuntimeError, TypeError, ValueError):
            self.cloud_valid = False
            self.cloud_time = now
            return
        self.relative_points = points
        self.cloud_valid = True
        self.cloud_time = now

    def reset_task(self):
        if not self.shaft_enabled and not self.task3_enabled:
            self.prestream_started = None
            self.hold_z = None
            self.hold_yaw = None
            self.clearance.previous_velocity = (0.0, 0.0)
            self.clearance.escape_latched = False
            self.takeover_latched = False
            self.failsafe_position = None

    def navigation_ready(self, now):
        estimator_ok = (
            self._fresh(self.estimator_time, now, self.estimator_timeout)
            and (self.estimator.pos_horiz_rel_status_flag
                 or self.estimator.pos_horiz_abs_status_flag)
            and (self.estimator.pos_vert_abs_status_flag
                 or self.estimator.pos_vert_agl_status_flag)
        )
        return (
            self._fresh(self.state_time, now, self.state_timeout)
            and self.state.connected
            and self.state.armed
            and self._fresh(self.local_pose_time, now, self.timeout)
            and self.local_pose is not None
            and estimator_ok
            and not self.takeover_latched
        )

    def build_task2(self, now, dt):
        ready = (
            self.active_task == TASK_SHAFT
            and self.shaft_enabled
            and self.shaft_status in ("DESCENDING", "RETURNING")
            and self._fresh(self.shaft_intent_time, now, self.timeout)
            and self._fresh(self.task_odometry_time, now, self.timeout)
            and self._fresh(self.cloud_time, now, self.timeout)
            and self.cloud_valid
            and self._fresh(self.heading_time, now, self.timeout)
            and self.heading_yaw is not None
        )
        if not ready:
            return None, "TASK2_INPUT_NOT_READY"
        vx, vy, coverage_ready, horizontal_state = self.clearance.step(
            self.relative_points, dt
        )
        if not coverage_ready or horizontal_state == "NO_FEASIBLE_ESCAPE":
            return None, "TASK2_" + horizontal_state
        cosine, sine = math.cos(self.heading_yaw), math.sin(self.heading_yaw)
        target = PositionTarget()
        target.header.stamp = rospy.Time.now()
        target.header.frame_id = "map"
        target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        target.type_mask = (
            PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY |
            PositionTarget.IGNORE_PZ | PositionTarget.IGNORE_AFX |
            PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW_RATE
        )
        target.velocity.x = cosine * vx - sine * vy
        target.velocity.y = sine * vx + cosine * vy
        target.velocity.z = self.shaft_intent.twist.linear.z
        target.yaw = self.hold_yaw
        return target, horizontal_state

    def build_task3(self, now):
        ready = (
            self.active_task == TASK_PRESSURE
            and self.task3_enabled
            and self._fresh(self.task3_controller_ready_time, now, self.timeout)
            and self.task3_controller_ready
            and self._fresh(self.task3_command_time, now, self.timeout)
            and self.task3_command is not None
        )
        if not ready:
            return None, "TASK3_INPUT_NOT_READY"
        if (
            self.task3_command.coordinate_frame != PositionTarget.FRAME_BODY_NED
            or self.task3_command.type_mask & PositionTarget.IGNORE_VX
            or self.task3_command.type_mask & PositionTarget.IGNORE_VY
        ):
            return None, "TASK3_COMMAND_SEMANTICS_INVALID"
        local_yaw = quaternion_yaw(self.local_pose.pose.orientation)
        if local_yaw is None:
            return None, "TASK3_YAW_INVALID"
        forward = self.task3_command.velocity.x
        right = self.task3_command.velocity.y
        if not math.isfinite(forward) or not math.isfinite(right):
            return None, "TASK3_COMMAND_INVALID"
        if math.hypot(forward, right) > self.task3_max_horizontal_speed + 1.0e-6:
            return None, "TASK3_COMMAND_OVERSPEED"
        target = PositionTarget()
        target.header.stamp = rospy.Time.now()
        target.header.frame_id = "map"
        target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        target.type_mask = (
            PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY |
            PositionTarget.IGNORE_VZ | PositionTarget.IGNORE_AFX |
            PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW_RATE
        )
        target.position.z = self.hold_z
        target.velocity.x = math.cos(local_yaw) * forward + math.sin(local_yaw) * right
        target.velocity.y = math.sin(local_yaw) * forward - math.cos(local_yaw) * right
        target.yaw = self.hold_yaw
        return target, "TASK3_ACTIVE"

    def publish_ready(self, task2, task3):
        self.task2_ready_pub.publish(Bool(data=task2))
        self.task3_ready_pub.publish(Bool(data=task3))

    def build_hold_target(self):
        """Keep the last measured pose streamed until PX4 accepts POSCTL."""
        if self.local_pose is None:
            return None
        if self.failsafe_position is None:
            position = self.local_pose.pose.position
            self.failsafe_position = (position.x, position.y, position.z)
        yaw = quaternion_yaw(self.local_pose.pose.orientation)
        if yaw is None:
            yaw = self.hold_yaw
        if yaw is None:
            return None
        target = PositionTarget()
        target.header.stamp = rospy.Time.now()
        target.header.frame_id = "map"
        target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        target.type_mask = (
            PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY |
            PositionTarget.IGNORE_VZ | PositionTarget.IGNORE_AFX |
            PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW_RATE
        )
        target.position.x, target.position.y, target.position.z = self.failsafe_position
        target.yaw = yaw
        return target

    def fail_safe(self, status, now):
        self.publish_ready(False, False)
        # OFFBOARD is shared with the exploration bridge (Tasks 1 and 4).
        # Never revoke a mode that this router neither requested nor owned.
        # A cancelled Task 2/3 request is still covered by
        # offboard_request_pending, including a late PX4 mode transition.  The
        # selected-task clause preserves safe recovery if this node restarts
        # while its own Task 2/3 is already in OFFBOARD.
        assisted_task_selected = (
            (self.active_task == TASK_SHAFT and self.shaft_enabled)
            or (self.active_task == TASK_PRESSURE and self.task3_enabled)
        )
        fallback_required = (
            self.owned_task
            or self.offboard_request_pending
            or self.fallback_pending
            or (assisted_task_selected and self.state.mode == self.offboard_mode)
        )
        if fallback_required:
            self.fallback_pending = True
            hold = self.build_hold_target()
            if hold is not None:
                self.command_pub.publish(hold)
            # A POSCTL state received before this request does not acknowledge
            # the fallback: it may simply precede a late OFFBOARD transition.
            # Require POSCTL observed after our request to remain stable for a
            # full request interval before releasing the latch.
            fallback_confirmed = (
                self.fallback_request_time is not None
                and self.state_time is not None
                and self.state_time >= self.fallback_request_time
                and self.state.mode == self.fallback_mode
            )
            if fallback_confirmed:
                if self.fallback_mode_seen_since is None:
                    self.fallback_mode_seen_since = self.state_time
                if now - self.fallback_mode_seen_since >= self.mode_request_interval:
                    self.fallback_pending = False
                    self.offboard_request_pending = False
                    self.fallback_request_time = None
                    self.fallback_mode_seen_since = None
            else:
                self.fallback_mode_seen_since = None
            if self.fallback_pending and not fallback_confirmed:
                self.request_mode(self.fallback_mode, now)
        self.status_pub.publish(String(data=status))

    def request_mode(self, mode, now):
        if now - self.last_mode_request < self.mode_request_interval:
            return False
        self.last_mode_request = now
        try:
            response = self.mode_client(custom_mode=mode)
        except rospy.ServiceException as error:
            rospy.logwarn_throttle(1.0, "Assisted task mode request failed: %s", error)
            return False
        if not response.mode_sent:
            rospy.logwarn_throttle(1.0, "Assisted task mode request rejected: %s", mode)
            return False
        if mode == self.offboard_mode:
            self.offboard_request_pending = True
        elif mode == self.fallback_mode:
            self.fallback_pending = True
            self.fallback_request_time = now
        return True

    def tick(self, _event):
        now = time.monotonic()
        dt = now - self.last_tick
        self.last_tick = now
        task = self.active_task if self.active_task in (TASK_SHAFT, TASK_PRESSURE) else 0
        enabled = ((task == TASK_SHAFT and self.shaft_enabled)
                   or (task == TASK_PRESSURE and self.task3_enabled))
        if not enabled:
            self.fail_safe("IDLE", now)
            self.owned_task = 0
            return
        if self.fallback_pending:
            self.fail_safe("FALLBACK_PENDING", now)
            return
        if not self.navigation_ready(now):
            self.fail_safe("NAVIGATION_NOT_READY", now)
            return

        if self.hold_z is None:
            self.hold_z = self.local_pose.pose.position.z
            self.hold_yaw = quaternion_yaw(self.local_pose.pose.orientation)
            self.prestream_started = now
        target, status = (
            self.build_task2(now, dt)
            if task == TASK_SHAFT else self.build_task3(now)
        )
        if target is None or self.hold_yaw is None:
            self.fail_safe(status, now)
            return

        self.failsafe_position = None
        self.command_pub.publish(target)
        if self.state.mode != self.offboard_mode:
            self.publish_ready(False, False)
            if now - self.prestream_started >= self.prestream_duration:
                self.request_mode(self.offboard_mode, now)
            self.status_pub.publish(String(data="PRESTREAM:" + status))
            return
        self.owned_task = task
        self.offboard_request_pending = False
        self.publish_ready(task == TASK_SHAFT, task == TASK_PRESSURE)
        self.status_pub.publish(String(data="OFFBOARD:" + status))


if __name__ == "__main__":
    rospy.init_node("assisted_task_px4_router")
    AssistedTaskPx4Router()
    rospy.spin()
