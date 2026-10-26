#!/usr/bin/env python3

"""Emulate the shared exploration pilot procedure in PX4 SITL only.

The node pre-streams a hover setpoint, enters OFFBOARD, arms, climbs to the
configured height, holds for one second, and only then raises the configured
emulated exploration switch.
It does not publish localization odometry or lidar data.
"""

import math
import threading
from collections import deque

import rospy
from geometry_msgs.msg import PoseStamped, Quaternion
from mavros_msgs.msg import RCIn, State
from mavros_msgs.srv import CommandBool, SetMode
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String


class SitlTaskOperator:
    def __init__(self):
        self.odom_topic = rospy.get_param(
            "~odom_topic", "/mavros/local_position/odom"
        )
        self.allow_auto_arm = bool(rospy.get_param("~allow_auto_arm", True))
        self.takeoff_height = max(0.5, float(rospy.get_param("~takeoff_height", 1.5)))
        self.takeoff_climb_rate = min(
            0.7, max(0.1, float(rospy.get_param("~takeoff_climb_rate", 0.35)))
        )
        self.ground_z_stable_duration = max(
            1.0, float(rospy.get_param("~ground_z_stable_duration", 3.0))
        )
        self.ground_z_stability_tolerance = max(
            0.02, float(rospy.get_param("~ground_z_stability_tolerance", 0.15))
        )
        self.takeoff_yaw_offset = float(
            rospy.get_param("~takeoff_yaw_offset", 0.0)
        )
        self.hover_duration = max(
            0.5, float(rospy.get_param("~takeoff_hover_duration", 1.0))
        )
        self.hover_horizontal_tolerance = max(
            0.1, float(rospy.get_param("~hover_horizontal_tolerance", 0.30))
        )
        self.hover_vertical_tolerance = max(
            0.1, float(rospy.get_param("~hover_vertical_tolerance", 0.25))
        )
        self.hover_yaw_tolerance = math.radians(max(
            1.0, float(rospy.get_param("~hover_yaw_tolerance_deg", 10.0))
        ))
        self.prestream_duration = max(
            1.0, float(rospy.get_param("~offboard_prestream_duration", 1.0))
        )
        self.auto_enable_delay = max(
            2.0, float(rospy.get_param("~auto_enable_delay", 8.0))
        )
        self.task_switch_pwm = int(rospy.get_param("~task_switch_pwm", 1000))
        configured_trigger = int(
            rospy.get_param("~exploration_trigger_channel", -1)
        )
        if configured_trigger < 0:
            configured_trigger = 10 if self.task_switch_pwm >= 1700 else 6
        if configured_trigger > 17:
            raise ValueError("exploration_trigger_channel must be in [0, 17]")
        self.exploration_trigger_channel = configured_trigger
        self.bridge_ready_topic = rospy.get_param(
            "~bridge_ready_topic", "/mine_uav/task1/command_ready"
        )
        self.vision_status_topic = rospy.get_param(
            "~vision_status_topic", "/mine_uav/task1/vision_status"
        )
        self.vision_stable_duration = max(
            1.0, float(rospy.get_param("~vision_stable_duration", 3.0))
        )
        self.rate = max(10.0, float(rospy.get_param("~rate", 20.0)))

        self._lock = threading.Lock()
        self._state = State()
        self._odom = None
        self._ground_z_history = deque(maxlen=400)
        self._start = rospy.Time.now()
        self._origin = None
        self._takeoff_command_z = None
        self._takeoff_command_stamp = rospy.Time(0)
        self._prestream_start = rospy.Time(0)
        self._hover_start = rospy.Time(0)
        self._last_mode_request = rospy.Time(0)
        self._last_arm_request = rospy.Time(0)
        self._ready = False
        self._takeoff_arm_seen = False
        self._takeoff_aborted = False
        self._bridge_handoff_complete = False
        self._vision_streaming_since = rospy.Time(0)

        self.rc_pub = rospy.Publisher("/mine_uav/sitl/rc/in", RCIn, queue_size=2)
        self.setpoint_pub = rospy.Publisher(
            "/mavros/setpoint_position/local", PoseStamped, queue_size=10
        )
        rospy.Subscriber("/mavros/state", State, self._state_callback, queue_size=10)
        rospy.Subscriber(self.odom_topic, Odometry, self._odom_callback, queue_size=20)
        rospy.Subscriber(self.bridge_ready_topic, Bool,
                         self._bridge_ready_callback, queue_size=2)
        rospy.Subscriber(self.vision_status_topic, String,
                         self._vision_status_callback, queue_size=5)
        self.arm_client = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        self.mode_client = rospy.ServiceProxy("/mavros/set_mode", SetMode)
        rospy.Timer(rospy.Duration(1.0 / self.rate), self._timer)
        rospy.logwarn("SITL task operator active; never run against a real FCU")

    def _state_callback(self, message):
        with self._lock:
            if message.armed:
                self._takeoff_arm_seen = True
            elif (self._takeoff_arm_seen and self._state.armed and
                  not self._ready):
                self._takeoff_aborted = True
                rospy.logerr(
                    "SITL pre-task takeoff disarmed unexpectedly; automatic "
                    "re-arm is latched off for this run"
                )
            self._state = message

    def _odom_callback(self, message):
        with self._lock:
            self._odom = message
            self._ground_z_history.append(
                (rospy.Time.now(), message.pose.pose.position.z)
            )

    def _bridge_ready_callback(self, message):
        if message.data:
            with self._lock:
                self._bridge_handoff_complete = True

    def _vision_status_callback(self, message):
        with self._lock:
            if message.data == "STREAMING":
                if self._vision_streaming_since.is_zero():
                    self._vision_streaming_since = rospy.Time.now()
            else:
                self._vision_streaming_since = rospy.Time(0)

    def _timer(self, _event):
        with self._lock:
            state = self._state
            odom = self._odom
            ground_z_history = list(self._ground_z_history)
        now = rospy.Time.now()
        if self.allow_auto_arm and not self._ready:
            self._advance_takeoff(now, state, odom, ground_z_history)
        elif self.allow_auto_arm and not self._bridge_handoff_complete:
            # Keep the pre-task hover setpoint alive until the task bridge has
            # produced its first valid OFFBOARD command. Otherwise PX4 may
            # leave OFFBOARD before SUPER finishes its first plan. Never
            # resume this publisher after the bridge has taken ownership.
            self._publish_takeoff_target(now)
        # Once the simulated pilot raises the task switch, keep it high.
        # Dropping it merely because AUTO.LAND disarmed the vehicle creates a
        # second switch edge and can start task one again immediately after a
        # successful mission.
        self._publish_rc(auto_enabled=self._ready)

    def _publish_rc(self, auto_enabled):
        message = RCIn()
        message.header.stamp = rospy.Time.now()
        message.channels = [1500] * max(
            11, self.exploration_trigger_channel + 1
        )
        # Keep known task channels at a stable baseline, then raise exactly the
        # configured exploration channel. task_switch_pwm remains a fallback
        # for older Task-1/Task-2 launch files.
        message.channels[6] = 1000
        message.channels[8] = 1000
        message.channels[9] = 1000
        message.channels[10] = 1000
        message.channels[self.exploration_trigger_channel] = (
            2000 if auto_enabled else 1000
        )
        self.rc_pub.publish(message)

    def _advance_takeoff(self, now, state, odom, ground_z_history):
        if self._takeoff_aborted:
            rospy.logerr_throttle(
                5.0, "SITL takeoff aborted; restart the regression before retrying"
            )
            return
        if odom is None or not state.connected:
            return
        if not rospy.get_param("/use_sim_time", False):
            rospy.logerr_throttle(2.0, "Refusing SITL automation without /use_sim_time")
            return
        with self._lock:
            vision_streaming_since = self._vision_streaming_since
        if (vision_streaming_since.is_zero() or
                (now - vision_streaming_since).to_sec() < self.vision_stable_duration):
            rospy.loginfo_throttle(
                2.0, "Waiting for stable LIO external vision before SITL takeoff"
            )
            return
        if self._origin is None:
            recent_ground_z = [
                z for stamp, z in ground_z_history
                if (now - stamp).to_sec() <= self.ground_z_stable_duration
            ]
            history_duration = (
                (now - ground_z_history[0][0]).to_sec()
                if ground_z_history else 0.0
            )
            if (history_duration < self.ground_z_stable_duration or
                    not recent_ground_z or
                    max(recent_ground_z) - min(recent_ground_z) >
                    self.ground_z_stability_tolerance):
                rospy.loginfo_throttle(
                    2.0, "Waiting for stable PX4 ground Z before SITL takeoff"
                )
                return
            pose = odom.pose.pose
            self._origin = (
                pose.position.x,
                pose.position.y,
                pose.position.z,
                pose.orientation,
            )
            self._takeoff_command_z = self._origin[2]
            self._takeoff_command_stamp = now
            self._prestream_start = now
            rospy.loginfo(
                "SITL pre-task takeoff: ground z=%.2f, target z=%.2f",
                self._origin[2],
                self._origin[2] + self.takeoff_height,
            )

        # A position step to the full takeoff altitude lets PX4 climb at its
        # own (much higher) default limit and invalidates the Task-1 speed
        # regression.  Ramp only after arming, below the production 0.8 m/s
        # mission limit.
        target = self._publish_takeoff_target(now, advance=state.armed)

        if (now - self._prestream_start).to_sec() < self.prestream_duration:
            return
        if state.mode != "OFFBOARD":
            self._request_mode(now)
            return
        if not state.armed:
            self._request_arm(now)
            return

        position = odom.pose.pose.position
        horizontal_error = math.hypot(
            position.x - target.pose.position.x,
            position.y - target.pose.position.y,
        )
        vertical_error = abs(position.z - target.pose.position.z)
        yaw_error = abs(self._angle_difference(
            self._yaw_from_quaternion(odom.pose.pose.orientation),
            self._yaw_from_quaternion(target.pose.orientation),
        ))
        if (horizontal_error <= self.hover_horizontal_tolerance and
                vertical_error <= self.hover_vertical_tolerance and
                yaw_error <= self.hover_yaw_tolerance):
            if self._hover_start.is_zero():
                self._hover_start = now
                rospy.loginfo("SITL reached pre-task hover")
            elif (now - self._hover_start).to_sec() >= self.hover_duration:
                self._ready = True
                rospy.loginfo("SITL hover stable; emulated task trigger changed")
        else:
            self._hover_start = rospy.Time(0)

    def _publish_takeoff_target(self, now, advance=False):
        target = PoseStamped()
        target.header.stamp = now
        target.header.frame_id = "map"
        target.pose.position.x = self._origin[0]
        target.pose.position.y = self._origin[1]
        final_z = self._origin[2] + self.takeoff_height
        if self._takeoff_command_z is None:
            self._takeoff_command_z = self._origin[2]
        if advance:
            dt = max(0.0, min(0.2, (now - self._takeoff_command_stamp).to_sec()))
            self._takeoff_command_z = min(
                final_z, self._takeoff_command_z + self.takeoff_climb_rate * dt
            )
        self._takeoff_command_stamp = now
        target.pose.position.z = self._takeoff_command_z
        target_yaw = (
            self._yaw_from_quaternion(self._origin[3]) +
            self.takeoff_yaw_offset
        )
        target.pose.orientation = Quaternion(
            0.0, 0.0, math.sin(0.5 * target_yaw),
            math.cos(0.5 * target_yaw)
        )
        self.setpoint_pub.publish(target)
        return target

    @staticmethod
    def _yaw_from_quaternion(quaternion):
        return math.atan2(
            2.0 * (quaternion.w * quaternion.z +
                   quaternion.x * quaternion.y),
            1.0 - 2.0 * (quaternion.y * quaternion.y +
                         quaternion.z * quaternion.z),
        )

    @staticmethod
    def _angle_difference(left, right):
        return math.atan2(math.sin(left - right), math.cos(left - right))

    def _request_mode(self, now):
        if (now - self._last_mode_request).to_sec() < 1.0:
            return
        self._last_mode_request = now
        try:
            response = self.mode_client(custom_mode="OFFBOARD")
            if response.mode_sent:
                rospy.loginfo("PX4 SITL pre-task OFFBOARD request accepted")
        except rospy.ServiceException as error:
            rospy.logwarn_throttle(2.0, "SITL mode service failed: %s", error)

    def _request_arm(self, now):
        if (now - self._last_arm_request).to_sec() < 1.0:
            return
        self._last_arm_request = now
        try:
            response = self.arm_client(True)
            if response.success:
                rospy.loginfo("PX4 SITL armed for pre-task takeoff")
        except rospy.ServiceException as error:
            rospy.logwarn_throttle(2.0, "SITL arm service failed: %s", error)


if __name__ == "__main__":
    rospy.init_node("sitl_task_operator")
    SitlTaskOperator()
    rospy.spin()
