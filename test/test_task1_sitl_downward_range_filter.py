#!/usr/bin/env python3

"""The initial no-return reading must never initialize PX4 range height."""

import importlib.util
from pathlib import Path
import unittest

from sensor_msgs.msg import Range


spec = importlib.util.spec_from_file_location(
    "task1_sitl_downward_range_filter",
    str(Path(__file__).resolve().parents[1] /
        "scripts/task1_sitl_downward_range_filter.py"),
)
range_filter = importlib.util.module_from_spec(spec)
spec.loader.exec_module(range_filter)


class DownwardRangeFilterTest(unittest.TestCase):
    def test_gazebo_no_return_is_not_fused(self):
        message = Range(min_range=0.2, max_range=30.0, range=30.0)
        self.assertFalse(range_filter.valid_ground_range(message, 5.0))
        message.range = 1.55
        self.assertTrue(range_filter.valid_ground_range(message, 5.0))

    def test_unusable_measurements_are_dropped(self):
        message = Range(min_range=0.2, max_range=30.0)
        for value in (float("nan"), float("inf"), 0.1, 5.0):
            message.range = value
            self.assertFalse(range_filter.valid_ground_range(message, 5.0))


if __name__ == "__main__":
    unittest.main()
