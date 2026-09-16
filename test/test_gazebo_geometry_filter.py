#!/usr/bin/env python3

"""Geometry checks for the Gazebo-only MID360 point-cloud adapter."""

import importlib.util
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

    def test_long_range_sensor_fixture_is_above_iris_disk(self):
        model = ET.parse(
            str(ROOT / "models/iris_mid360_long_range/iris_mid360_long_range.sdf")
        ).getroot()
        pose = model.findtext("model/link[@name='mid360_link']/pose")
        self.assertIsNotNone(pose)
        self.assertGreaterEqual(float(pose.split()[2]), 0.5)

        launch = ET.parse(
            str(ROOT / "launch/task1_40m_platform_sitl.launch")
        ).getroot()
        offset = launch.find(".//arg[@name='sensor_offset_z']")
        self.assertIsNotNone(offset)
        self.assertAlmostEqual(float(offset.attrib["value"]), float(pose.split()[2]))


if __name__ == "__main__":
    unittest.main()
