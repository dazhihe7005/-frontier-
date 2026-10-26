#!/usr/bin/env python3

import importlib.util
import math
import pathlib
import unittest
from unittest import mock

import yaml
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import SetModeResponse


PATH = pathlib.Path(__file__).resolve().parents[1] / "scripts" / "assisted_task_px4_router.py"
SPEC = importlib.util.spec_from_file_location("assisted_task_px4_router", PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def ring(radius, count=64, centre_x=0.0, centre_y=0.0):
    return [
        (
            centre_x + radius * math.cos(index * 2.0 * math.pi / count),
            centre_y + radius * math.sin(index * 2.0 * math.pi / count),
        )
        for index in range(count)
    ]


class ClearanceAssistCoreTest(unittest.TestCase):
    def test_idle_router_does_not_decode_registered_cloud(self):
        router = MODULE.AssistedTaskPx4Router.__new__(
            MODULE.AssistedTaskPx4Router
        )
        router.active_task = 0
        router.shaft_enabled = False
        router.relative_points = [(1.0, 0.0)]
        router.cloud_valid = True
        router.cloud_time = 1.0

        # The early task gate runs before any PointCloud2 field access or
        # read_points conversion, so an otherwise invalid sentinel is enough.
        router.cloud_cb(object())

        self.assertEqual(router.relative_points, [])
        self.assertFalse(router.cloud_valid)
        self.assertIsNone(router.cloud_time)

    def test_centered_ring_does_not_invent_large_motion(self):
        core = MODULE.ClearanceAssistCore(max_acceleration=10.0)
        vx, vy, ready, state = core.step(ring(2.0), 0.1)
        self.assertTrue(ready)
        self.assertEqual(state, "CENTERING")
        self.assertLess(math.hypot(vx, vy), 1.0e-6)

    def test_offset_shaft_prefers_maximum_clearance_center(self):
        core = MODULE.ClearanceAssistCore(max_acceleration=10.0)
        # Wall ring centred 0.5 m to the right of the vehicle.
        vx, vy, ready, state = core.step(ring(2.0, centre_x=0.5), 0.1)
        self.assertTrue(ready)
        self.assertIn(state, ("CENTERING", "AVOIDING"))
        self.assertGreater(vx, 0.0)
        self.assertLessEqual(math.hypot(vx, vy), 0.250001)

    def test_hard_intrusion_forces_away_component(self):
        core = MODULE.ClearanceAssistCore(max_acceleration=10.0)
        points = ring(2.0) + [(0.8, 0.0)]
        vx, _, ready, state = core.step(points, 0.1)
        self.assertTrue(ready)
        self.assertEqual(state, "ESCAPING")
        self.assertLess(vx, 0.0)

    def test_unknown_coverage_blocks_vertical_task_readiness(self):
        core = MODULE.ClearanceAssistCore(max_acceleration=10.0)
        points = [(2.0, 0.1), (2.0, 0.0), (2.0, -0.1)]
        _, _, ready, state = core.step(points, 0.1)
        self.assertFalse(ready)
        self.assertIn(state, ("AVOIDING", "WAIT_COVERAGE"))

    def test_opposed_hard_intrusions_report_no_escape(self):
        core = MODULE.ClearanceAssistCore(max_acceleration=10.0)
        points = ring(2.0) + [(0.8, 0.0), (-0.8, 0.0)]
        vx, vy, ready, state = core.step(points, 0.1)
        self.assertFalse(ready)
        self.assertEqual(state, "NO_FEASIBLE_ESCAPE")
        self.assertEqual((vx, vy), (0.0, 0.0))

    def test_hard_distance_uses_release_hysteresis(self):
        core = MODULE.ClearanceAssistCore(max_acceleration=10.0)
        core.step(ring(2.0) + [(0.8, 0.0)], 0.1)
        self.assertTrue(core.escape_latched)
        _, _, ready, state = core.step(ring(2.0) + [(1.1, 0.0)], 0.1)
        self.assertTrue(ready)
        self.assertTrue(core.escape_latched)
        self.assertEqual(state, "ESCAPING")
        core.step(ring(2.0) + [(1.3, 0.0)], 0.1)
        self.assertFalse(core.escape_latched)

    def test_qp_selects_nearest_feasible_corner_independent_of_order(self):
        core = MODULE.ClearanceAssistCore(max_acceleration=10.0)
        constraints = [
            (0.8, 1.0, 0.0, 0.1),
            (0.8, 0.0, 1.0, 0.1),
        ]
        forward = core._solve_qp(0.0, 0.0, constraints)
        reverse = core._solve_qp(0.0, 0.0, list(reversed(constraints)))
        self.assertAlmostEqual(forward[0], 0.1)
        self.assertAlmostEqual(forward[1], 0.1)
        self.assertAlmostEqual(forward[0], reverse[0])
        self.assertAlmostEqual(forward[1], reverse[1])


class Task3FinalRoutingTest(unittest.TestCase):
    def test_task3_compiled_and_yaml_hard_limits_are_1_6(self):
        config_path = pathlib.Path(__file__).resolve().parents[1] / "config" / "assisted_task_px4_router.yaml"
        config = yaml.safe_load(config_path.read_text())
        self.assertEqual(MODULE.TASK3_HARD_MAX_HORIZONTAL_SPEED, 1.6)
        self.assertEqual(config["task3_max_horizontal_speed"], 1.6)
        self.assertEqual(config["input_timeout"], 0.5)
        self.assertEqual(config["state_timeout"], 1.5)
        self.assertEqual(config["estimator_timeout"], 1.5)

    @staticmethod
    def router(now):
        router = MODULE.AssistedTaskPx4Router.__new__(
            MODULE.AssistedTaskPx4Router
        )
        router.active_task = MODULE.TASK_PRESSURE
        router.task3_enabled = True
        router.task3_controller_ready = True
        router.task3_controller_ready_time = now
        router.task3_command_time = now
        router.timeout = 0.5
        router.task3_max_horizontal_speed = 1.6
        router.hold_z = 2.5
        router.hold_yaw = 0.3
        router.local_pose = PoseStamped()
        router.local_pose.pose.orientation.z = math.sin(math.pi / 4.0)
        router.local_pose.pose.orientation.w = math.cos(math.pi / 4.0)
        command = PositionTarget()
        command.coordinate_frame = PositionTarget.FRAME_BODY_NED
        command.type_mask = (
            PositionTarget.IGNORE_PX
            | PositionTarget.IGNORE_PY
            | PositionTarget.IGNORE_PZ
            | PositionTarget.IGNORE_AFX
            | PositionTarget.IGNORE_AFY
            | PositionTarget.IGNORE_AFZ
            | PositionTarget.IGNORE_YAW
            | PositionTarget.IGNORE_YAW_RATE
        )
        command.velocity.x = 0.2
        command.velocity.y = 0.1
        router.task3_command = command
        return router

    def test_one_hz_navigation_summaries_do_not_expire_between_samples(self):
        now = 10.0
        router = self.router(now)
        router.state_timeout = 1.5
        router.estimator_timeout = 1.5
        router.state = State(connected=True, armed=True, mode="OFFBOARD")
        router.state_time = now - 1.1
        router.estimator_time = now - 1.1
        router.estimator = mock.Mock(
            pos_horiz_rel_status_flag=True,
            pos_horiz_abs_status_flag=False,
            pos_vert_abs_status_flag=True,
            pos_vert_agl_status_flag=False,
        )
        router.local_pose_time = now - 0.4
        router.takeover_latched = False

        self.assertTrue(router.navigation_ready(now))

        # High-rate pose/command inputs retain their 0.5 s fail-fast contract.
        router.local_pose_time = now - 0.51
        self.assertFalse(router.navigation_ready(now))
        router.local_pose_time = now
        router.state_time = now - 1.51
        self.assertFalse(router.navigation_ready(now))

    def test_body_command_is_rotated_and_holds_captured_height_and_yaw(self):
        now = 10.0
        with mock.patch.object(
            MODULE.rospy.Time,
            "now",
            return_value=MODULE.rospy.Time.from_sec(now),
        ):
            target, status = self.router(now).build_task3(now)
        self.assertEqual(status, "TASK3_ACTIVE")
        self.assertAlmostEqual(target.velocity.x, 0.1, places=6)
        self.assertAlmostEqual(target.velocity.y, 0.2, places=6)
        self.assertAlmostEqual(target.position.z, 2.5, places=6)
        self.assertAlmostEqual(target.yaw, 0.3, places=6)
        self.assertFalse(target.type_mask & PositionTarget.IGNORE_PZ)
        self.assertTrue(target.type_mask & PositionTarget.IGNORE_VZ)

    def test_final_owner_rejects_wrong_frame_and_overspeed(self):
        now = 10.0
        router = self.router(now)
        router.task3_command.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        target, status = router.build_task3(now)
        self.assertIsNone(target)
        self.assertEqual(status, "TASK3_COMMAND_SEMANTICS_INVALID")

        router = self.router(now)
        router.task3_command.velocity.x = 1.61
        router.task3_command.velocity.y = 0.0
        target, status = router.build_task3(now)
        self.assertIsNone(target)
        self.assertEqual(status, "TASK3_COMMAND_OVERSPEED")

    def test_final_owner_accepts_exact_task3_hard_limit(self):
        now = 10.0
        router = self.router(now)
        router.task3_command.velocity.x = 1.6
        router.task3_command.velocity.y = 0.0
        with mock.patch.object(
            MODULE.rospy.Time,
            "now",
            return_value=MODULE.rospy.Time.from_sec(now),
        ):
            target, status = router.build_task3(now)
        self.assertEqual(status, "TASK3_ACTIVE")
        self.assertIsNotNone(target)
        self.assertAlmostEqual(
            math.hypot(target.velocity.x, target.velocity.y), 1.6, places=6
        )


class LateOffboardFallbackTest(unittest.TestCase):
    @staticmethod
    def router():
        router = MODULE.AssistedTaskPx4Router.__new__(
            MODULE.AssistedTaskPx4Router
        )
        router.offboard_mode = "OFFBOARD"
        router.fallback_mode = "POSCTL"
        router.mode_request_interval = 1.0
        router.last_mode_request = 0.0
        router.active_task = 0
        router.shaft_enabled = False
        router.task3_enabled = False
        router.owned_task = 0
        router.offboard_request_pending = True
        router.fallback_pending = False
        router.fallback_request_time = None
        router.fallback_mode_seen_since = None
        router.takeover_latched = False
        router.state = State(connected=True, armed=True, mode="POSCTL")
        router.state_time = 9.0
        router.local_pose = PoseStamped()
        router.local_pose.pose.orientation.w = 1.0
        router.hold_yaw = 0.0
        router.failsafe_position = None
        router.command_pub = mock.Mock()
        router.task2_ready_pub = mock.Mock()
        router.task3_ready_pub = mock.Mock()
        router.status_pub = mock.Mock()
        router.mode_client = mock.Mock(
            return_value=SetModeResponse(mode_sent=True)
        )
        return router

    def test_idle_router_does_not_revoke_another_tasks_offboard(self):
        router = self.router()
        router.offboard_request_pending = False
        router.active_task = 4
        router.state = State(connected=True, armed=True, mode="OFFBOARD")

        router.fail_safe("IDLE", 10.0)

        router.mode_client.assert_not_called()
        router.command_pub.publish.assert_not_called()
        self.assertFalse(router.fallback_pending)

    def test_selected_assisted_task_recovers_offboard_after_router_restart(self):
        router = self.router()
        router.offboard_request_pending = False
        router.active_task = MODULE.TASK_PRESSURE
        router.task3_enabled = True
        router.state = State(connected=True, armed=True, mode="OFFBOARD")

        with mock.patch.object(
            MODULE.rospy.Time,
            "now",
            return_value=MODULE.rospy.Time.from_sec(10.0),
        ):
            router.fail_safe("TASK3_INPUT_NOT_READY", 10.0)

        router.mode_client.assert_called_once_with(custom_mode="POSCTL")
        self.assertTrue(router.command_pub.publish.called)
        self.assertTrue(router.fallback_pending)

    def test_cancelled_task_retracts_inflight_offboard_until_posctl_is_stable(self):
        router = self.router()
        with mock.patch.object(
            MODULE.rospy.Time,
            "now",
            return_value=MODULE.rospy.Time.from_sec(10.0),
        ):
            # The task is already cancelled and the last sampled mode is
            # POSCTL, but an accepted OFFBOARD request is still in flight.
            # POSCTL must be requested anyway instead of trusting that old
            # sample.
            router.fail_safe("IDLE", 10.0)
            router.mode_client.assert_called_with(custom_mode="POSCTL")
            self.assertTrue(router.fallback_pending)

            # Reproduce the flight: OFFBOARD arrives after cancellation.  The
            # fallback latch keeps a hold stream alive and retries POSCTL.
            with mock.patch.object(MODULE.time, "monotonic", return_value=10.1):
                router.state_cb(
                    State(connected=True, armed=True, mode="OFFBOARD")
                )
            router.fail_safe("IDLE", 11.1)
            self.assertEqual(
                router.mode_client.call_args_list[-1],
                mock.call(custom_mode="POSCTL"),
            )
            self.assertTrue(router.command_pub.publish.called)
            self.assertTrue(router.fallback_pending)

            # One post-request POSCTL sample starts confirmation; the latch is
            # released only after it remains stable for a full request
            # interval.
            with mock.patch.object(MODULE.time, "monotonic", return_value=11.2):
                router.state_cb(
                    State(connected=True, armed=True, mode="POSCTL")
                )
            router.fail_safe("IDLE", 11.8)
            self.assertTrue(router.fallback_pending)
            router.fail_safe("IDLE", 12.3)
            self.assertFalse(router.fallback_pending)
            self.assertFalse(router.offboard_request_pending)


if __name__ == "__main__":
    unittest.main()
