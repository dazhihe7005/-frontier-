#!/usr/bin/env python3

import importlib.util
import math
import pathlib
import unittest


PATH = pathlib.Path(__file__).resolve().parents[1] / "scripts" / "omnilivo_shaft_depth.py"
SPEC = importlib.util.spec_from_file_location("omnilivo_shaft_depth", PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class RelativeDepthCoreTest(unittest.TestCase):
    def test_depth_is_positive_down_from_enable_reference(self):
        core = MODULE.RelativeDepthCore(max_sigma=0.25)
        core.set_enabled(True)
        valid, depth, sigma, reason = core.evaluate(
            10.0, 0.01, "task1_planning", True
        )
        self.assertTrue(valid)
        self.assertEqual(reason, "ok")
        self.assertAlmostEqual(depth, 0.0)
        self.assertAlmostEqual(sigma, 0.1)
        valid, depth, _, _ = core.evaluate(
            7.5, 0.01, "task1_planning", True
        )
        self.assertTrue(valid)
        self.assertAlmostEqual(depth, 2.5)

    def test_disable_clears_reference(self):
        core = MODULE.RelativeDepthCore()
        core.set_enabled(True)
        core.evaluate(4.0, 0.01, "task1_planning", True)
        core.set_enabled(False)
        self.assertFalse(core.evaluate(3.0, 0.01, "task1_planning", True)[0])
        core.set_enabled(True)
        self.assertAlmostEqual(
            core.evaluate(3.0, 0.01, "task1_planning", True)[1], 0.0
        )

    def test_bad_health_frame_and_uncertainty_fail_closed(self):
        core = MODULE.RelativeDepthCore(max_sigma=0.25)
        core.set_enabled(True)
        self.assertFalse(core.evaluate(0.0, 0.01, "wrong", True)[0])
        self.assertFalse(core.evaluate(0.0, 0.01, "task1_planning", False)[0])
        valid, _, sigma, reason = core.evaluate(
            0.0, 0.09, "task1_planning", True
        )
        self.assertFalse(valid)
        self.assertAlmostEqual(sigma, 0.3)
        self.assertEqual(reason, "depth_uncertainty")
        self.assertFalse(
            core.evaluate(math.nan, 0.01, "task1_planning", True)[0]
        )


if __name__ == "__main__":
    unittest.main()
