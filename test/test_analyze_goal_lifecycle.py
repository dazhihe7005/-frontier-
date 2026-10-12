#!/usr/bin/env python3

"""Pure event-stream regression cases for task-one cancellation."""

import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))
from analyze_task1_sitl_bag import (
    actual_speed_for_acceptance, analyze_goal_lifecycle_events,
    truth_return_error,
)


class TruthReturnAnalysisTest(unittest.TestCase):
    def test_physical_speed_overrides_underestimated_fcu_speed(self):
        report = {"gazebo_truth_available": True,
                  "truth_speed_max": 1.94, "speed_max": 0.81}
        self.assertEqual(actual_speed_for_acceptance(report), 1.94)
        report["truth_speed_max"] = None
        self.assertIsNone(actual_speed_for_acceptance(report))

    def test_detects_pose_drift_hidden_by_local_home_estimate(self):
        positions = [(10.0, -7.0, 0.5), (148.0, -6.2, 0.5),
                     (249.0, -14.6, -0.4)]
        self.assertAlmostEqual(truth_return_error(positions, 10.0, 148.0), 0.8)
        self.assertGreater(truth_return_error(positions, 10.0, 249.0), 7.0)

    def test_missing_or_stale_truth_cannot_pass(self):
        self.assertIsNone(truth_return_error([], 10.0, 20.0))
        self.assertIsNone(truth_return_error([(10.0, 0.0, 0.0),
                                              (19.0, 0.0, 0.0)], 10.0, 20.0))


class GoalLifecycleAnalysisTest(unittest.TestCase):
    def test_cancel_silences_old_goal(self):
        result = analyze_goal_lifecycle_events([
            (0.0, "set", 1),
            (1.0, "pos_cmd", 0),
            (2.0, "cancel", 1),
        ])
        self.assertEqual(result["stale_position_commands"], 0)
        self.assertEqual(result["stale_plan_from_rest_calls"], 0)
        self.assertEqual(result["cancel_count"], 1)

    def test_late_old_command_is_failure(self):
        result = analyze_goal_lifecycle_events([
            (0.0, "set", 1),
            (1.0, "cancel", 1),
            (1.02, "pos_cmd", 0),
            (1.03, "plan_from_rest", 0),
        ])
        self.assertEqual(result["stale_position_commands"], 1)
        self.assertEqual(result["stale_plan_from_rest_calls"], 1)
        self.assertAlmostEqual(result["cancel_to_silence_latency"], 0.03)

    def test_new_goal_after_cancel_is_valid(self):
        result = analyze_goal_lifecycle_events([
            (0.0, "set", 1),
            (1.0, "cancel", 1),
            (2.0, "set", 2),
            (2.1, "pos_cmd", 0),
        ])
        self.assertEqual(result["stale_position_commands"], 0)
        self.assertEqual(result["active_goal_id"], 2)

    def test_late_specific_cancel_cannot_cancel_new_goal(self):
        result = analyze_goal_lifecycle_events([
            (0.0, "cancel", 1),
            (1.0, "set", 2),
            (1.5, "cancel", 1),
            (1.6, "pos_cmd", 0),
        ])
        self.assertEqual(result["stale_position_commands"], 0)
        self.assertEqual(result["ignored_stale_cancels"], 2)
        self.assertEqual(result["active_goal_id"], 2)

    def test_one_inflight_message_grace_only(self):
        result = analyze_goal_lifecycle_events([
            (0.0, "set", 1),
            (1.0, "cancel", 1),
            (1.005, "pos_cmd", 0),
            (1.01, "pos_cmd", 0),
        ])
        self.assertEqual(result["stale_position_commands"], 1)

    def test_unconditional_cancel_works(self):
        result = analyze_goal_lifecycle_events([
            (0.0, "set", 1),
            (1.0, "cancel", 0),
            (1.1, "pos_cmd", 0),
        ])
        self.assertEqual(result["stale_position_commands"], 1)


if __name__ == "__main__":
    unittest.main()
