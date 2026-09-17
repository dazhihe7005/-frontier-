#!/usr/bin/env python3
"""Small deterministic checks for the flight-height audit math."""

import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from analyze_task1_vertical_trajectory import inspect_polynomial, polynomial_extrema


class VerticalTrajectoryAuditTest(unittest.TestCase):
    def test_interior_peak_is_not_missed(self):
        low, high = polynomial_extrema([-1.0, 1.0, 1.5], 1.0)
        self.assertAlmostEqual(low[0], 1.5)
        self.assertAlmostEqual(high[0], 1.75)
        self.assertAlmostEqual(high[1], 0.5)

    def test_boundary_jump_and_fence(self):
        # First segment is z=t, second is z=1+t; both touch at z=1.
        message = SimpleNamespace(order_pos=1, piece_num_pos=2,
                                  time_pos=[1.0, 1.0],
                                  coef_pos_z=[1.0, 0.0, 1.0, 1.0])
        low, high, step, violations = inspect_polynomial(
            message, alignment_z=0.0, min_height=-2.0, max_height=1.8)
        self.assertAlmostEqual(low[0], 0.0)
        self.assertAlmostEqual(high[0], 2.0)
        self.assertAlmostEqual(step, 0.0)
        self.assertEqual(violations, 1)
        message.coef_pos_z[-1] = 1.2
        _, _, step, _ = inspect_polynomial(message)
        self.assertAlmostEqual(step, 0.2)


if __name__ == "__main__":
    unittest.main()
