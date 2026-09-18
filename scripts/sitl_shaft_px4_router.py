#!/usr/bin/env python3

"""SITL-only single-owner shaft velocity/XY-hold router to MAVROS."""

import math

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import SetMode
from std_msgs.msg import Bool, String


class SitlShaftPx4Router:
    def __init__(self):
        self.intent_timeout = float(rospy.get_param("~intent_timeout", 0.3))
        self.pose_timeout = float(rospy.get_param("~pose_timeout", 0.5))
        self.state_timeout = float(rospy.get_param("~state_timeout", 1.5))
        self.enable = False
        self.owned = False
        self.takeover_latched = False
        self.ready = False
        self.intent = None
        self.intent_time = rospy.Time(0)
        self.pose = None
        self.pose_time = rospy.Time(0)
        self.fixed_xy = None
        self.status = "IDLE"
        self.state = State()
        self.state_time = rospy.Time(0)
        self.last_fallback = rospy.Time(0)
        self.command_pub = rospy.Publisher(
            "/mavros/setpoint_raw/local", PositionTarget, queue_size=10
        )
        self.ready_pub = rospy.Publisher(
            "/mine_uav/task2/command_ready", Bool, queue_size=1, latch=True
        )
        self.mode_client = rospy.ServiceProxy("/mavros/set_mode", SetMode)
        rospy.Subscriber("/mine_uav/mission/shaft_enable", Bool, self.on_enable)
        rospy.Subscriber("/mine_uav/shaft/velocity_intent_enu", TwistStamped,
                         self.on_intent)
        rospy.Subscriber("/mine_uav/shaft/status", String, self.on_status)
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.on_pose)
        rospy.Subscriber("/mavros/state", State, self.on_state)
        rospy.Timer(rospy.Duration(0.05), self.tick)
        self.ready_pub.publish(Bool(data=False))
        rospy.logwarn("SITL-only shaft PX4 router; do not use with a real FCU")

    def on_enable(self, message):
        if not message.data:
            self.fixed_xy = None
            self.takeover_latched = False
            self.intent = None  # A later task must supply a new command.
            self.intent_time = rospy.Time(0)
        self.enable = message.data

    def on_intent(self, message):
        if not self.enable or message.header.frame_id != "map" or not math.isfinite(
                message.twist.linear.z):
            return
        self.intent = message
        self.intent_time = rospy.Time.now()

    def on_status(self, message):
        self.status = message.data

    def on_pose(self, message):
        self.pose = message
        self.pose_time = rospy.Time.now()

    def on_state(self, message):
        self.state = message
        self.state_time = rospy.Time.now()

    def publish_ready(self, value):
        if self.ready != value:
            self.ready = value
            self.ready_pub.publish(Bool(data=value))

    def publish_hold(self, now):
        if self.pose is None:
            return
        target = PositionTarget()
        target.header.stamp = now
        target.header.frame_id = "map"
        target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        target.type_mask = (PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY |
                            PositionTarget.IGNORE_VZ | PositionTarget.IGNORE_AFX |
                            PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
                            PositionTarget.IGNORE_YAW |
                            PositionTarget.IGNORE_YAW_RATE)
        target.position = self.pose.pose.position
        self.command_pub.publish(target)

    def request_fallback(self, now):
        if (now - self.last_fallback).to_sec() < 1.0:
            return
        self.last_fallback = now
        try:
            self.mode_client(custom_mode="AUTO.LOITER")
        except rospy.ServiceException as error:
            rospy.logwarn_throttle(2.0, "SITL shaft fallback mode request: %s", error)

    def tick(self, _event):
        if not rospy.get_param("/use_sim_time", False):
            rospy.logerr_throttle(2.0, "SITL shaft router requires /use_sim_time")
            return
        now = rospy.Time.now()
        state_age = (now - self.state_time).to_sec()
        state_fresh = (not self.state_time.is_zero() and
                       -0.05 <= state_age <= self.state_timeout)
        pose_fresh = self.pose is not None and (
            -0.05 <= (now - self.pose_time).to_sec() <= self.pose_timeout and
            not self.pose.header.stamp.is_zero() and
            -0.05 <= (now - self.pose.header.stamp).to_sec() <=
            self.pose_timeout and
            all(math.isfinite(value) for value in (
                self.pose.pose.position.x, self.pose.pose.position.y,
                self.pose.pose.position.z)))
        intent_fresh = self.intent is not None and (
            -0.05 <= (now - self.intent_time).to_sec() <= self.intent_timeout and
            not self.intent.header.stamp.is_zero() and
            -0.05 <= (now - self.intent.header.stamp).to_sec() <=
            self.intent_timeout)
        if self.owned and self.state.mode != "OFFBOARD":
            self.owned = False
            # External takeover is latched while the old task is still
            # enabled. A completion/fault already revoked by the scheduler
            # must not re-latch after its one false enable message.
            self.takeover_latched = self.enable
            self.publish_ready(False)
            return  # A pilot/PX4 takeover must not be undone.
        if self.takeover_latched:
            self.publish_ready(False)
            return
        if self.owned and not state_fresh:
            self.publish_ready(False)
            if pose_fresh:
                self.publish_hold(now)
            self.request_fallback(now)
            return
        if self.owned and (not self.enable or not pose_fresh or
                           not intent_fresh or self.status not in
                           ("DESCENDING", "RETURNING")):
            self.publish_ready(False)
            if self.state.mode == "OFFBOARD":
                if pose_fresh:
                    self.publish_hold(now)
                self.request_fallback(now)
            return
        if (not self.enable or not state_fresh or not self.state.connected or
                not self.state.armed or
                self.state.mode != "OFFBOARD" or not pose_fresh or
                not intent_fresh or self.status not in ("DESCENDING", "RETURNING")):
            self.publish_ready(False)
            return
        if self.fixed_xy is None:
            self.fixed_xy = (self.pose.pose.position.x, self.pose.pose.position.y)
        target = PositionTarget()
        target.header.stamp = now
        target.header.frame_id = "map"
        target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        target.type_mask = (PositionTarget.IGNORE_PZ | PositionTarget.IGNORE_VX |
                            PositionTarget.IGNORE_VY | PositionTarget.IGNORE_AFX |
                            PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
                            PositionTarget.IGNORE_YAW |
                            PositionTarget.IGNORE_YAW_RATE)
        target.position.x, target.position.y = self.fixed_xy
        target.velocity.z = self.intent.twist.linear.z
        self.command_pub.publish(target)
        self.owned = True
        self.publish_ready(True)


if __name__ == "__main__":
    rospy.init_node("sitl_shaft_px4_router")
    SitlShaftPx4Router()
    rospy.spin()
