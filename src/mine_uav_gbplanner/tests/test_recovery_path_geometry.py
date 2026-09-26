"""Recovery must reverse observed poses, without bridging estimator gaps."""

import importlib.util
from pathlib import Path
import unittest


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "recovery_path_geometry.py"
SPEC = importlib.util.spec_from_file_location("recovery_path_geometry", SCRIPT)
GEOMETRY = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(GEOMETRY)


class ReverseObservedPathTests(unittest.TestCase):
    def test_reverses_actual_history_without_altitude_excursion(self):
        history = [(i*0.1, 0.0, 1.0) for i in range(12)]
        points, travelled = GEOMETRY.reverse_observed_path(
            (1.1, 0.0, 1.0), history, max_distance=0.6)
        self.assertGreaterEqual(travelled, 0.59)
        self.assertTrue(all(point[2] == 1.0 for point in points))
        self.assertTrue(all(points[i+1][0] < points[i][0]
                            for i in range(len(points)-1)))

    def test_discontinuity_stops_without_long_connector(self):
        history = [(0.0, 0.0, 1.0), (0.1, 0.0, 1.0),
                   (3.0, 0.0, 1.0), (3.1, 0.0, 1.0)]
        points, travelled = GEOMETRY.reverse_observed_path(
            (3.1, 0.0, 1.0), history)
        self.assertEqual(points, [(3.1, 0.0, 1.0), (3.0, 0.0, 1.0)])
        self.assertAlmostEqual(travelled, 0.1)

    def test_max_distance_clips_last_segment(self):
        history = [(0.0, 0.0, 1.0), (0.2, 0.0, 1.0),
                   (0.4, 0.0, 1.0)]
        points, travelled = GEOMETRY.reverse_observed_path(
            (0.4, 0.0, 1.0), history, max_distance=0.3)
        self.assertAlmostEqual(travelled, 0.3)
        self.assertAlmostEqual(points[-1][0], 0.1)

    def test_gap_at_current_position_fails_closed(self):
        points, travelled = GEOMETRY.reverse_observed_path(
            (1.0, 0.0, 1.0), [(0.0, 0.0, 1.0)])
        self.assertEqual(points, [(1.0, 0.0, 1.0)])
        self.assertEqual(travelled, 0.0)


if __name__ == "__main__":
    unittest.main()
