#!/usr/bin/env python3

"""Pure callback-level checks for the SITL router's ownership lifecycle."""

import importlib.util
import math
import pathlib
import unittest
from unittest.mock import MagicMock, patch

from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Bool


MODULE_PATH = pathlib.Path(__file__).resolve().parents[1] / "scripts" / "sitl_shaft_px4_router.py"
SPEC = importlib.util.spec_from_file_location("sitl_shaft_px4_router", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def owned_router():
    router = object.__new__(MODULE.SitlShaftPx4Router)
    router.enable = True
    router.owned = True
    router.takeover_latched = False
    router.fixed_xy = (1.0, 2.0)
    router.ready = True
    router.ready_pub = MagicMock()
    router.pose = None
    router.intent = None
    router.pose_timeout = 0.5
    router.intent_timeout = 0.3
    router.state = State()
    router.state.mode = "OFFBOARD"
    router.state.connected = True
    router.state.armed = True
    router.state_time = MODULE.rospy.Time(10)
    router.state_timeout = 1.5
    return router


def deliver_state(router, state):
    with patch.object(MODULE.rospy.Time, "now",
                      return_value=MODULE.rospy.Time(10)):
        router.on_state(state)


class ShaftRouterLifecycleTest(unittest.TestCase):
    def test_completion_then_late_mode_exit_allows_next_task(self):
        router = owned_router()
        router.intent = TwistStamped()
        router.on_enable(Bool(data=False))  # scheduler revokes completed task
        self.assertTrue(router.owned)  # still owns setpoints until mode exit
        self.assertIsNone(router.intent)  # no replay into a later task
        exited = State()
        exited.mode = "AUTO.LOITER"
        deliver_state(router, exited)  # PX4 changes mode after the false edge
        with patch.object(MODULE.rospy, "get_param", return_value=True), \
                patch.object(MODULE.rospy.Time, "now",
                             return_value=MODULE.rospy.Time(10)):
            router.tick(None)
        self.assertFalse(router.owned)
        self.assertFalse(router.takeover_latched)
        router.on_enable(Bool(data=True))
        self.assertFalse(router.takeover_latched)

    def test_external_takeover_latches_until_scheduler_revokes(self):
        router = owned_router()
        exited = State()
        exited.mode = "AUTO.LOITER"
        deliver_state(router, exited)
        with patch.object(MODULE.rospy, "get_param", return_value=True), \
                patch.object(MODULE.rospy.Time, "now",
                             return_value=MODULE.rospy.Time(10)):
            router.tick(None)
        self.assertFalse(router.owned)
        self.assertTrue(router.takeover_latched)
        returned = State()
        returned.mode = "OFFBOARD"
        deliver_state(router, returned)
        self.assertTrue(router.takeover_latched)  # cannot silently regain ownership
        router.on_enable(Bool(data=False))
        self.assertFalse(router.takeover_latched)

    def test_stale_px4_state_cannot_keep_publishing_descent(self):
        router = owned_router()
        router.state_time = MODULE.rospy.Time(1)
        router.state_timeout = 1.5
        router.pose = PoseStamped()
        router.pose_time = MODULE.rospy.Time(10)
        router.intent = TwistStamped()
        router.intent.header.stamp = MODULE.rospy.Time(10)
        router.intent.header.frame_id = "map"
        router.intent.twist.linear.z = -0.5
        router.intent_time = MODULE.rospy.Time(10)
        router.status = "DESCENDING"
        router.command_pub = MagicMock()
        router.publish_hold = MagicMock()
        router.request_fallback = MagicMock()
        with patch.object(MODULE.rospy, "get_param", return_value=True), \
                patch.object(MODULE.rospy.Time, "now",
                             return_value=MODULE.rospy.Time(10)):
            router.tick(None)
        router.command_pub.publish.assert_not_called()
        self.assertFalse(router.ready)

    def test_replayed_intent_stamp_cannot_publish_descent(self):
        router = owned_router()
        router.pose = PoseStamped()
        router.pose.header.stamp = MODULE.rospy.Time(10)
        router.pose_time = MODULE.rospy.Time(10)
        router.intent = TwistStamped()
        router.intent.header.stamp = MODULE.rospy.Time(1)
        router.intent_time = MODULE.rospy.Time(10)  # freshly replayed old data
        router.status = "DESCENDING"
        router.command_pub = MagicMock()
        router.publish_hold = MagicMock()
        router.request_fallback = MagicMock()
        with patch.object(MODULE.rospy, "get_param", return_value=True), \
                patch.object(MODULE.rospy.Time, "now",
                             return_value=MODULE.rospy.Time(10)):
            router.tick(None)
        router.command_pub.publish.assert_not_called()
        self.assertFalse(router.ready)

    def test_late_intent_after_disable_is_discarded(self):
        router = owned_router()
        router.on_enable(Bool(data=False))
        old = TwistStamped()
        old.header.frame_id = "map"
        old.header.stamp = MODULE.rospy.Time(10)
        old.twist.linear.z = -0.5
        with patch.object(MODULE.rospy.Time, "now",
                          return_value=MODULE.rospy.Time(10)):
            router.on_intent(old)
        self.assertIsNone(router.intent)

    def test_replayed_pose_stamp_cannot_publish_descent(self):
        router = owned_router()
        router.pose = PoseStamped()
        router.pose.header.stamp = MODULE.rospy.Time(1)  # republished at t=10
        router.pose.pose.position.x = 1.0
        router.pose.pose.position.y = 2.0
        router.pose.pose.position.z = 3.0
        router.pose_time = MODULE.rospy.Time(10)
        router.intent = TwistStamped()
        router.intent.header.stamp = MODULE.rospy.Time(10)
        router.intent.twist.linear.z = -0.5
        router.intent_time = MODULE.rospy.Time(10)
        router.status = "DESCENDING"
        router.command_pub = MagicMock()
        router.publish_hold = MagicMock()
        router.request_fallback = MagicMock()
        with patch.object(MODULE.rospy, "get_param", return_value=True), \
                patch.object(MODULE.rospy.Time, "now",
                             return_value=MODULE.rospy.Time(10)):
            router.tick(None)
        router.command_pub.publish.assert_not_called()
        router.publish_hold.assert_not_called()
        self.assertFalse(router.ready)

    def test_nonfinite_pose_cannot_publish_descent_or_hold(self):
        for bad_value in (math.nan, math.inf):
            with self.subTest(bad_value=bad_value):
                router = owned_router()
                router.fixed_xy = None  # first command captures the invalid XY
                router.pose = PoseStamped()
                router.pose.header.stamp = MODULE.rospy.Time(10)
                router.pose.pose.position.x = bad_value
                router.pose.pose.position.y = 2.0
                router.pose.pose.position.z = 3.0
                router.pose_time = MODULE.rospy.Time(10)
                router.intent = TwistStamped()
                router.intent.header.stamp = MODULE.rospy.Time(10)
                router.intent.twist.linear.z = -0.5
                router.intent_time = MODULE.rospy.Time(10)
                router.status = "DESCENDING"
                router.command_pub = MagicMock()
                router.publish_hold = MagicMock()
                router.request_fallback = MagicMock()
                with patch.object(MODULE.rospy, "get_param", return_value=True), \
                        patch.object(MODULE.rospy.Time, "now",
                                     return_value=MODULE.rospy.Time(10)):
                    router.tick(None)
                router.command_pub.publish.assert_not_called()
                router.publish_hold.assert_not_called()
                self.assertFalse(router.ready)

    def test_fresh_finite_pose_and_intent_publish_target(self):
        router = owned_router()
        router.pose = PoseStamped()
        router.pose.header.stamp = MODULE.rospy.Time(10)
        router.pose.pose.position.x = 1.0
        router.pose.pose.position.y = 2.0
        router.pose.pose.position.z = 3.0
        router.pose_time = MODULE.rospy.Time(10)
        router.intent = TwistStamped()
        router.intent.header.stamp = MODULE.rospy.Time(10)
        router.intent.twist.linear.z = -0.5
        router.intent_time = MODULE.rospy.Time(10)
        router.status = "DESCENDING"
        router.command_pub = MagicMock()
        with patch.object(MODULE.rospy, "get_param", return_value=True), \
                patch.object(MODULE.rospy.Time, "now",
                             return_value=MODULE.rospy.Time(10)):
            router.tick(None)
        target = router.command_pub.publish.call_args.args[0]
        self.assertEqual((target.position.x, target.position.y), (1.0, 2.0))
        self.assertEqual(target.velocity.z, -0.5)
        self.assertTrue(router.ready)


if __name__ == "__main__":
    unittest.main()
