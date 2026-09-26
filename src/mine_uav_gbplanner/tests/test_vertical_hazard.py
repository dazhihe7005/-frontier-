#!/usr/bin/env python3
"""Protect the distinction between a side-wall return and a roof/floor return."""

import importlib.util
import math
from pathlib import Path
import threading
import unittest
from unittest.mock import MagicMock, patch
import xml.etree.ElementTree as ET
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint


EXECUTOR = Path(__file__).resolve().parents[1] / "scripts/gbplanner_px4_executor.py"
SPEC = importlib.util.spec_from_file_location("gbplanner_px4_executor", EXECUTOR)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class VerticalHazardTest(unittest.TestCase):
    def setUp(self):
        self.executor = MODULE.GbplannerPx4Executor.__new__(
            MODULE.GbplannerPx4Executor)
        self.executor.proximity_z_max = 0.35
        self.executor.safety_radius = 1.0
        self.executor.vertical_avoidance_margin = 0.30
        self.executor.absolute_clearance_hold_margin = 0.25
        self.executor.absolute_clearance_slowdown_margin = 0.55

    def test_raised_side_wall_is_not_a_ceiling(self):
        # This return previously locked the UAV at x ~= 2.5 m for >100 s.
        self.assertFalse(self.executor._is_vertical_hazard(
            1.276, 0.356, True))

    def test_roof_and_floor_remain_protected(self):
        self.assertTrue(self.executor._is_vertical_hazard(1.276, 1.20, True))
        self.assertTrue(self.executor._is_vertical_hazard(1.276, -1.20, True))

    def test_clear_or_stale_return_does_not_trigger_escape(self):
        self.assertFalse(self.executor._is_vertical_hazard(1.40, 1.35, True))
        self.assertFalse(self.executor._is_vertical_hazard(1.276, 1.20, False))

    def test_opposite_floor_bounds_roof_escape(self):
        # A 0.35 m descent would violate the 1 m radius when the floor is
        # only 1.2 m below. The extra 0.05 m reserve limits it to 0.15 m.
        self.assertAlmostEqual(
            MODULE.vertical_escape_room(0.0, -1.20, 1.05), 0.15)
        self.assertEqual(
            MODULE.vertical_escape_room(0.0, -1.00, 1.05), 0.0)
        self.assertTrue(math.isinf(
            MODULE.vertical_escape_room(1.20, -0.10, 1.05)))

    def test_sidewall_does_not_latch_completed_vertical_escape(self):
        # Previous code used overall spatial clearance. In a narrow tunnel
        # a 1.3 m sidewall kept a roof escape latched indefinitely.
        self.assertFalse(MODULE.vertical_escape_pending(
            True, 0.02, 1.50, 1.25))
        self.assertTrue(MODULE.vertical_escape_pending(
            True, 0.02, 1.20, 1.25))
        self.assertTrue(MODULE.vertical_escape_pending(
            True, 0.12, 1.50, 1.25))

    def test_downward_blind_cone_brakes_before_floor(self):
        scale = MODULE.floor_descent_scale
        self.assertEqual(scale(1.20, 1.0, 0.25, 0.55), 0.0)
        self.assertAlmostEqual(scale(1.40, 1.0, 0.25, 0.55), 0.5)
        self.assertEqual(scale(1.60, 1.0, 0.25, 0.55), 1.0)

    def test_rising_floor_is_guarded_during_level_flight(self):
        required = MODULE.floor_guard_required
        self.assertTrue(required(1.34, 1.0, 0.0, 0.0))
        self.assertFalse(required(1.50, 1.0, 0.0, 0.0))
        self.assertTrue(required(1.50, 1.0, -0.04, 0.0))

    def test_descending_recovery_floor_abort_is_latched_early(self):
        abort = MODULE.recovery_floor_abort_required
        self.assertTrue(abort(True, -0.35, 1.55, 1.0, 0.60))
        self.assertFalse(abort(True, -0.35, 1.70, 1.0, 0.60))
        self.assertFalse(abort(False, -0.35, 1.30, 1.0, 0.60))
        self.assertFalse(abort(True, +0.10, 1.30, 1.0, 0.60))
        self.executor._lock = threading.Lock()
        self.executor._recovery_floor_abort = True
        self.executor._recovery_cb(MODULE.Bool(data=False))
        self.assertTrue(self.executor._recovery_floor_abort)
        self.assertTrue(MODULE.proactive_floor_escape_required(True, 0.80))
        self.assertFalse(MODULE.proactive_floor_escape_required(True, 0.10))
        self.assertFalse(MODULE.proactive_floor_escape_required(
            True, float("inf")))

    def test_floor_follow_raises_only_with_observed_roof_room(self):
        target = MODULE.floor_follow_target_z
        self.assertAlmostEqual(target(3.70, 3.80, 1.30, 1.70, 0.80), 4.20)
        self.assertAlmostEqual(target(3.70, 3.80, 1.30, 1.70, 0.15), 3.95)
        self.assertAlmostEqual(target(4.30, 3.80, 1.30, 1.70, 0.80), 4.30)
        self.assertAlmostEqual(target(3.70, 3.80, 1.80, 1.70, 0.80), 3.70)
        self.assertAlmostEqual(target(3.70, 4.20, 1.80, 1.70, 0.80), 4.10)
        self.assertAlmostEqual(target(3.70, 4.20, 2.30, 1.70, 0.80), 3.70)
        self.assertAlmostEqual(target(3.70, 3.80, 1.30, 1.70,
                                      float("inf")), 3.70)

    def test_ceiling_cap_stays_below_roof_until_clear(self):
        cap = MODULE.ceiling_follow_cap
        first = cap(None, 0.70, 1.29, 1.20, 2.24, 1.70, 1.60)
        self.assertAlmostEqual(first, 0.39)
        self.assertAlmostEqual(
            cap(first, 0.39, 1.60, 1.60, 1.93, 1.70, 1.60), first)
        self.assertIsNone(
            cap(first, 0.39, 2.01, 2.00, 1.93, 1.70, 1.60))

    def test_ceiling_cap_never_competes_with_floor(self):
        cap = MODULE.ceiling_follow_cap
        self.assertIsNone(
            cap(None, 0.70, 1.29, 1.20, 1.60, 1.70, 1.60))
        self.assertIsNone(
            cap(0.39, 0.50, 1.45, 1.40, 1.70, 1.70, 1.60))
        self.assertIsNone(
            cap(None, 0.70, 1.29, -1.20, 2.24, 1.70, 1.60))
        self.assertIsNone(
            cap(None, 0.70, 1.29, float("nan"), 2.24, 1.70, 1.60))

    def test_balanced_band_resolves_conflicting_preferred_clearances(self):
        target = MODULE.balanced_vertical_target_z
        # 1.70 m + 1.60 m cannot fit in a 3.0 m tunnel. The safe center
        # preserves 1.50 m on both sides instead of alternating escapes.
        self.assertAlmostEqual(
            target(3.8, 3.0, 1.7, 1.3, 1.7, 1.6, 1.0), 2.8)
        self.assertAlmostEqual(
            target(2.0, 3.0, 1.3, 1.6, 1.7, 1.6, 1.0), 3.15)
        # A roomy shaft leaves the planner height untouched inside the band.
        self.assertAlmostEqual(
            target(3.0, 3.0, 2.0, 2.0, 1.7, 1.6, 1.0), 3.0)

    def test_balanced_band_fails_closed_when_hard_radius_has_no_reserve(self):
        target = MODULE.balanced_vertical_target_z
        self.assertIsNone(target(3.0, 3.0, 1.1, 1.1,
                                 1.7, 1.6, 1.0))
        self.assertIsNone(target(3.0, 3.0, 1.4, float("inf"),
                                 1.7, 1.6, 1.0))

    def test_no_hit_downward_range_cannot_be_treated_as_clear_floor(self):
        valid = MODULE.valid_downward_range
        self.assertTrue(valid(1.4, 0.1, 30.0))
        self.assertFalse(valid(30.0, 0.1, 30.0))
        self.assertFalse(valid(29.98, 0.1, 30.0))
        self.assertFalse(valid(float("inf"), 0.1, 30.0))

    def test_fan_slant_range_is_not_body_center_clearance(self):
        lower = MODULE.cone_body_clearance_lower_bound
        self.assertAlmostEqual(lower(0.65, 0.35, 0.0), 1.0)
        self.assertAlmostEqual(lower(0.65, 0.35, math.pi/3.0),
                               math.sqrt(0.65**2+0.35**2+0.65*0.35))
        self.assertLess(lower(0.65, 0.35, math.pi/3.0, 0.03), 0.90)
        self.assertEqual(lower(float("nan"), 0.35, math.pi/3.0), 0.0)

    def test_fan_geometry_matches_launch_clearance_bound(self):
        workspace = Path(__file__).resolve().parents[3]
        sdf = ET.parse(workspace / "isolated_assets/models/iris_mid360/iris_mid360.sdf")
        launch = ET.parse(workspace / "src/mine_uav_gbplanner/launch/gbplanner_baixiangshan_full_chain.launch")
        link = sdf.find(".//link[@name='mid360_link']")
        sensor = link.find(".//sensor[@name='shaft_downward_ray']")
        link_x, _, link_z, _, link_pitch, _ = map(float, link.findtext("pose").split())
        rel_x, _, rel_z, _, sensor_pitch, _ = map(float, sensor.findtext("pose").split())
        body_x = link_x+math.cos(link_pitch)*rel_x+math.sin(link_pitch)*rel_z
        body_z = link_z-math.sin(link_pitch)*rel_x+math.cos(link_pitch)*rel_z
        self.assertAlmostEqual(body_x, 0.0, places=5)
        self.assertAlmostEqual(link_pitch+sensor_pitch, math.pi/2.0, places=5)
        horizontal = max(abs(float(sensor.findtext("ray/scan/horizontal/min_angle"))),
                         abs(float(sensor.findtext("ray/scan/horizontal/max_angle"))))
        vertical = max(abs(float(sensor.findtext("ray/scan/vertical/min_angle"))),
                       abs(float(sensor.findtext("ray/scan/vertical/max_angle"))))
        max_off_axis = math.acos(math.cos(horizontal)*math.cos(vertical))
        params = {node.get("name"): node.get("value")
                  for node in launch.findall(".//node[@name='gbplanner_px4_executor']/param")}
        self.assertAlmostEqual(-body_z,
                               float(params["downward_range_origin_below_body"]),
                               places=5)
        self.assertAlmostEqual(math.degrees(max_off_axis),
                               float(params["downward_range_max_off_axis_deg"]),
                               places=5)

    def test_new_floor_threat_reverses_old_roof_escape(self):
        direction = MODULE.vertical_escape_direction
        self.assertEqual(direction(True, True, 1.20, True, -1.0), 1.0)
        self.assertEqual(direction(False, True, 1.20, False, 0.0), -1.0)
        self.assertEqual(direction(False, True, -1.20, False, 0.0), 1.0)
        self.assertEqual(direction(False, False, float("nan"), False, 0.0), 0.0)

    def test_escape_direction_persists_after_floor_recovers(self):
        # A roof return becomes the nearest vertical point while the
        # floor-escape target is still being reached. Do not reverse then.
        direction = MODULE.vertical_escape_direction
        self.assertEqual(direction(False, False, 1.72, True, 1.0), 1.0)

    def test_recovery_cannot_bypass_floor_brake(self):
        stop = MODULE.should_proximity_stop
        self.assertTrue(stop(0.0))
        self.assertTrue(stop(0.10))
        self.assertFalse(stop(0.11))

    def test_recovery_directional_guard_stops_a_new_obstacle(self):
        self.assertEqual(self.executor._proximity_scale(
            1.50, 1.50, 0.30, 0.0, True, recovery_active=True), 0.0)
        self.assertLess(self.executor._proximity_scale(
            1.50, 1.50, 0.40, 0.0, True, recovery_active=True), 0.10)
        self.assertEqual(self.executor._proximity_scale(
            1.55, 1.55, 1.00, 0.0, True, recovery_active=True), 1.0)

    def test_recovery_never_bypasses_absolute_1m_sphere(self):
        scale = self.executor._proximity_scale
        for nearby in (0.80, 1.00, 1.20, 1.25):
            with self.subTest(nearby=nearby):
                self.assertEqual(scale(nearby, nearby, float("inf"), 0.0,
                                       True, recovery_active=True), 0.0)
        # Directional freedom cannot turn a side/overhead return into a
        # clearance exception. Above the hold reserve, speed rises smoothly.
        self.assertAlmostEqual(scale(1.40, 1.40, float("inf"), 0.0,
                                     True, recovery_active=True), 0.50)
        self.assertEqual(scale(1.55, 1.55, float("inf"), 0.0,
                               True, recovery_active=True), 1.0)
        for bad in (float("nan"),):
            self.assertEqual(scale(bad, 2.0, float("inf"), 0.0,
                                   True, recovery_active=True), 0.0)
            self.assertEqual(scale(2.0, bad, float("inf"), 0.0,
                                   True, recovery_active=True), 0.0)
            self.assertEqual(scale(2.0, 2.0, bad, 0.0,
                                   True, recovery_active=True), 0.0)

    def test_vision_loss_hold_latches_xy_and_climbs_only_with_fresh_room(self):
        executor = self.executor
        executor._lock = threading.Lock()
        executor._vision_loss_hold_pose = None
        executor._vision_loss_active = False
        executor._setpoint_pub = MagicMock()
        executor._publish_ready = MagicMock()
        executor._publish_status = MagicMock()
        executor.max_height = 10.0
        pose = MODULE.PoseStamped()
        pose.pose.orientation.w = 1.0
        pose.pose.position.x = 4.0
        pose.pose.position.y = 2.0
        pose.pose.position.z = 3.0
        state = MODULE.State(connected=True, armed=True)
        now = MODULE.rospy.Time(10)
        with patch.object(MODULE.rospy, "logerr"):
            executor._publish_last_safe_hold(
                now, state, pose, True, 1.70, True, 0.8, True,
                "FASTLIO_UNHEALTHY_HOLD")
            first = executor._setpoint_pub.publish.call_args.args[0]
            self.assertAlmostEqual(first.position.x, 4.0)
            self.assertAlmostEqual(first.position.z, 3.0)
            pose.pose.position.x = 4.7
            pose.pose.position.y = 2.4
            pose.pose.position.z = 2.8
            executor._publish_last_safe_hold(
                now, state, pose, True, 1.08, True, 0.5, True,
                "FASTLIO_UNHEALTHY_HOLD")
            second = executor._setpoint_pub.publish.call_args.args[0]
            self.assertAlmostEqual(second.position.x, 4.0)
            self.assertAlmostEqual(second.position.y, 2.0)
            self.assertAlmostEqual(second.position.z, 3.15)
            pose.pose.position.z = 2.5
            executor._publish_last_safe_hold(
                now, state, pose, True, 0.80, False, 0.5, True,
                "FASTLIO_UNHEALTHY_HOLD")
            third = executor._setpoint_pub.publish.call_args.args[0]
            self.assertAlmostEqual(third.position.z, 3.15)
            self.assertEqual(third.velocity.z, 0.0)
            self.assertTrue(executor._vision_loss_active)

    def test_vision_loss_escape_never_enters_unobserved_roof(self):
        target = MODULE.vision_loss_hold_z
        self.assertAlmostEqual(target(3.0, 3.0, 1.08, 0.12,
                                      True, True, 1.0, 10.0), 3.12)
        self.assertAlmostEqual(target(3.0, 3.0, 1.08, float("inf"),
                                      True, True, 1.0, 10.0), 3.0)
        self.assertAlmostEqual(target(3.0, 3.0, 1.08, 0.5,
                                      True, False, 1.0, 10.0), 3.0)

    def test_vision_recovery_requires_new_trajectory(self):
        executor = self.executor
        executor._lock = threading.Lock()
        executor._vision_loss_active = True
        executor._vision_loss_hold_pose = (4.0, 2.0, 3.0, 0.0)
        executor._vision_healthy = False
        executor._odom_rx = MODULE.rospy.Time(10)
        executor.odom_timeout = 0.5
        executor._alignment = (0.0, 0.0, 0.0, 0.0)
        executor._local_pose = MODULE.PoseStamped()
        executor._local_pose.pose.position.z = 3.0
        executor._validate = MagicMock(return_value="")
        executor._executable_end_time = MagicMock(return_value=1.0)
        executor._publish_status = MagicMock()
        executor._publish_blocked = MagicMock()
        executor._rejected = 0
        executor._accepted = 0
        executor._proximity_blocked_since = MODULE.rospy.Time(0)
        executor._recovery_floor_abort = False
        message = MODULE.MultiDOFJointTrajectory()
        message.points.append(MultiDOFJointTrajectoryPoint())
        message.points[0].transforms.append(MODULE.TransformStamped().transform)
        with patch.object(MODULE.rospy.Time, "now",
                          return_value=MODULE.rospy.Time(10)), \
                patch.object(MODULE.rospy, "loginfo"):
            executor._trajectory_cb(message)
            self.assertEqual(executor._rejected, 1)
            self.assertTrue(executor._vision_loss_active)
            executor._vision_healthy = True
            executor._trajectory_cb(message)
        self.assertEqual(executor._accepted, 1)
        self.assertFalse(executor._vision_loss_active)
        self.assertIsNone(executor._vision_loss_hold_pose)


if __name__ == "__main__":
    unittest.main()
