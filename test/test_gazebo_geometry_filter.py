#!/usr/bin/env python3

"""Geometry checks for the Gazebo-only MID360 point-cloud adapter."""

import importlib.util
import math
import pathlib
import unittest
import xml.etree.ElementTree as ET


ROOT = pathlib.Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location(
    "gazebo_mid360_adapter", ROOT / "scripts/gazebo_mid360_fastlio_adapter.py"
)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class GazeboGeometryFilterTest(unittest.TestCase):
    def setUp(self):
        self.adapter = module.GazeboMid360FastlioAdapter.__new__(
            module.GazeboMid360FastlioAdapter
        )
        self.adapter._world_boxes = self.adapter._load_static_boxes(
            str(ROOT / "worlds/goaf_40x40x30_platform.world")
        )
        self.adapter.surface_tolerance = 0.35

    def test_platform_ghost_above_surface_is_rejected(self):
        self.assertTrue(self.adapter._on_static_surface((0.0, 0.0, 15.0)))
        self.assertFalse(self.adapter._on_static_surface((0.0, 0.0, 17.3)))
        self.assertTrue(self.adapter._on_static_surface((39.8, 0.0, 17.3)))

    def test_no_return_ray_is_clipped_before_far_wall(self):
        distance = self.adapter._first_static_hit(
            (20.0, 0.0, 17.3), (1.0, 0.0, 0.0)
        )
        self.assertAlmostEqual(distance, 19.8, places=2)
        self.assertGreater(distance - 0.35, 19.0)

    def test_all_wrappers_use_physical_mid360s_mount(self):
        paths = [
            ROOT / "models/iris_mid360/iris_mid360.sdf",
            ROOT / "models/iris_mid360_long_range/iris_mid360_long_range.sdf",
            ROOT / "models/iris_mid360_shaft_8m/iris_mid360_shaft_8m.sdf",
        ]
        for path in paths:
            model = ET.parse(str(path)).getroot()
            pose = [float(value) for value in model.findtext(
                "model/link[@name='mid360_link']/pose").split()]
            self.assertEqual(pose[:3], [0.1315, 0.0, 0.223])
            self.assertAlmostEqual(pose[4], math.radians(25), places=9)

        launch_text = (ROOT / "launch/task1_px4_sitl.launch").read_text()
        self.assertIn("[0.1315, 0.0, 0.223]", launch_text)
        self.assertIn("[0.0, 0.436332313, 0.0]", launch_text)

    def test_tilted_lidar_self_filter_is_evaluated_in_body_frame(self):
        rotation = self.adapter._rpy_matrix(0.0, math.radians(25.0), 0.0)
        offset = (0.1315, 0.0, 0.223)

        # A fuselage return below the pitched sensor is inside the physical
        # body mask after transforming it back to body coordinates.
        self.assertTrue(
            self.adapter._inside_body_self_filter(
                (-0.2635, 0.0, -0.5650),
                rotation,
                offset,
                0.65,
                -0.65,
                0.25,
            )
        )
        # A wall return at the same sensor-frame height remains outside the
        # horizontal body footprint and must reach the planner.
        self.assertFalse(
            self.adapter._inside_body_self_filter(
                (2.0, 0.0, -0.5650),
                rotation,
                offset,
                0.65,
                -0.65,
                0.25,
            )
        )

        launch_text = (ROOT / "launch/task1_px4_sitl.launch").read_text()
        self.assertIn('name="self_filter_xy_radius" value="0.65"', launch_text)


if __name__ == "__main__":
    unittest.main()
