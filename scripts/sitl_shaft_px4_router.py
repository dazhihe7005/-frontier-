#!/usr/bin/env python3

"""SITL-only single-owner shaft velocity/XY-hold router to MAVROS."""

import math

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import EstimatorStatus, PositionTarget, State
from mavros_msgs.srv import SetMode
from mine_uav_control.msg import ShaftDepthEstimate
from std_msgs.msg import Bool, String


class SitlShaftPx4Router:
    def __init__(self):
        self.intent_timeout = float(rospy.get_param("~intent_timeout", 0.3))
        self.pose_timeout = float(rospy.get_param("~pose_timeout", 0.5))
        self.state_timeout = float(rospy.get_param("~state_timeout", 1.5))
        self.estimator_timeout = float(rospy.get_param("~estimator_timeout", 1.5))
        self.depth_timeout = float(rospy.get_param("~depth_timeout", 0.5))
        self.required_depth_source = rospy.get_param("~required_depth_source", "")
        self.max_depth_sigma_m = float(rospy.get_param("~max_depth_sigma_m", 0.25))
        self.max_depth_disagreement_m = float(
            rospy.get_param("~max_depth_disagreement_m", 1.0))
        if (not self.required_depth_source or
                not math.isfinite(self.depth_timeout) or self.depth_timeout <= 0.0 or
                not math.isfinite(self.max_depth_sigma_m) or
                self.max_depth_sigma_m <= 0.0 or
                not math.isfinite(self.max_depth_disagreement_m) or
                self.max_depth_disagreement_m <= 0.0):
            raise ValueError("SITL shaft depth consistency gate is unconfigured")
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
        self.estimator = EstimatorStatus()
        self.estimator_time = rospy.Time(0)
        self.depth = None
        self.depth_time = rospy.Time(0)
        self.depth_pose_reference = None
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
        rospy.Subscriber("/mavros/estimator_status", EstimatorStatus,
                         self.on_estimator)
        rospy.Subscriber("/mine_uav/shaft/depth_estimate", ShaftDepthEstimate,
                         self.on_depth)
        rospy.Timer(rospy.Duration(0.05), self.tick)
        self.ready_pub.publish(Bool(data=False))
        rospy.logwarn("SITL-only shaft PX4 router; do not use with a real FCU")

    def on_enable(self, message):
        if not message.data:
            self.fixed_xy = None
            self.depth_pose_reference = None
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

    def on_estimator(self, message):
        self.estimator = message
        self.estimator_time = rospy.Time.now()

    def on_depth(self, message):
        self.depth = message
        self.depth_time = rospy.Time.now()

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
        estimator_age = (now - self.estimator_time).to_sec()
        estimator_ok = (
            not self.estimator_time.is_zero() and
            -0.05 <= estimator_age <= self.estimator_timeout and
            not self.estimator.header.stamp.is_zero() and
            -0.05 <= (now - self.estimator.header.stamp).to_sec() <=
            self.estimator_timeout and
            (self.estimator.pos_horiz_rel_status_flag or
             self.estimator.pos_horiz_abs_status_flag) and
            (self.estimator.pos_vert_abs_status_flag or
             self.estimator.pos_vert_agl_status_flag))
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
        depth_fresh = self.depth is not None and (
            -0.05 <= (now - self.depth_time).to_sec() <= self.depth_timeout and
            not self.depth.header.stamp.is_zero() and
            -0.05 <= (now - self.depth.header.stamp).to_sec() <=
            self.depth_timeout and
            self.depth.source_id == self.required_depth_source and
            self.depth.valid and
            math.isfinite(self.depth.relative_depth_m) and
            math.isfinite(self.depth.sigma_m) and
            0.0 <= self.depth.sigma_m <= self.max_depth_sigma_m)
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
        if self.owned and not estimator_ok:
            # PX4 itself reports unusable local position. No position hold
            # target can be trusted here; request fallback, but never claim
            # this guarantees safe hover without a valid PX4 Z estimate.
            self.publish_ready(False)
            self.request_fallback(now)
            return
        if self.owned and self.enable and not depth_fresh:
            self.publish_ready(False)
            self.request_fallback(now)
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
        if (not self.enable or not state_fresh or not estimator_ok or
                not depth_fresh or
                not self.state.connected or
                not self.state.armed or
                self.state.mode != "OFFBOARD" or not pose_fresh or
                not intent_fresh or self.status not in ("DESCENDING", "RETURNING")):
            self.publish_ready(False)
            return
        if self.depth_pose_reference is None:
            self.depth_pose_reference = (
                self.pose.pose.position.z, self.depth.relative_depth_m)
        reference_z, reference_depth = self.depth_pose_reference
        disagreement = abs((self.pose.pose.position.z - reference_z) +
                           (self.depth.relative_depth_m - reference_depth))
        if disagreement > self.max_depth_disagreement_m:
            rospy.logerr_throttle(
                2.0, "SITL shaft PX4 Z/depth disagree by %.2f m; withdrawing command",
                disagreement)
            self.publish_ready(False)
            if self.owned:
                self.request_fallback(now)
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
