"""Check the planner's opt-in MID360 ray model against the Gazebo mount."""

import math
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).resolve().parents[3]


def body_elevation(sensor_elevation, sensor_azimuth, mount_pitch):
    x = math.cos(sensor_elevation) * math.cos(sensor_azimuth)
    y = math.cos(sensor_elevation) * math.sin(sensor_azimuth)
    z = math.sin(sensor_elevation)
    body_x = math.cos(mount_pitch)*x + math.sin(mount_pitch)*z
    body_z = -math.sin(mount_pitch)*x + math.cos(mount_pitch)*z
    return math.degrees(math.atan2(body_z, math.hypot(body_x, y)))


class Mid360GainGeometryTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        config = ROOT / "src/mine_uav_gbplanner/config/gbplanner_mid360.yaml"
        cls.sensor = yaml.safe_load(config.read_text(encoding="utf-8"))[
            "SensorParams"]["MID360S"]
        sdf = ROOT / "isolated_assets/models/iris_mid360/iris_mid360.sdf"
        model = ET.parse(sdf).getroot()
        link = model.find(".//link[@name='mid360_link']")
        cls.mount_pose = [float(value) for value in link.find("pose").text.split()]
        vertical = link.find(".//sensor[@name='mid360_ray']/ray/scan/vertical")
        cls.min_angle = math.degrees(float(vertical.find("min_angle").text))
        cls.max_angle = math.degrees(float(vertical.find("max_angle").text))

    def test_mount_and_aperture_match_gazebo(self):
        for planner, sdf in zip(self.sensor["center_offset"], self.mount_pose[:3]):
            self.assertAlmostEqual(planner, sdf, places=6)
        self.assertAlmostEqual(self.sensor["rotations"][1],
                               self.mount_pose[4], places=6)
        self.assertTrue(self.sensor["spherical_lidar_rays"])
        self.assertEqual(self.sensor["lidar_vertical_center_offset"],
                         "rad(22.5*pi/180)")
        self.assertEqual(self.sensor["fov"][1], "rad(59.0*pi/180)")
        self.assertAlmostEqual(self.min_angle, -7.0, places=5)
        self.assertAlmostEqual(self.max_angle, 52.0, places=5)

    def test_body_aperture_depends_on_azimuth(self):
        pitch = self.mount_pose[4]
        self.assertAlmostEqual(body_elevation(math.radians(-7), 0, pitch),
                               -32, places=4)
        self.assertAlmostEqual(body_elevation(math.radians(52), 0, pitch),
                               27, places=4)
        self.assertAlmostEqual(body_elevation(math.radians(-7), math.pi, pitch),
                               18, places=4)
        self.assertAlmostEqual(body_elevation(math.radians(52), math.pi, pitch),
                               77, places=4)

    def test_previous_infrastructure_violation_is_outside_aperture(self):
        # Independent mesh audit, run b9b70a98: nearest surface relative to
        # body centre at the 0.98011 m clearance violation.
        dx, dy, dz = (-0.7254, -0.45455, -0.47728)
        sx, sy, sz = self.mount_pose[:3]
        dx -= sx
        dy -= sy
        dz -= sz
        pitch = self.mount_pose[4]
        sensor_x = math.cos(pitch)*dx - math.sin(pitch)*dz
        sensor_z = math.sin(pitch)*dx + math.cos(pitch)*dz
        elevation = math.degrees(math.atan2(
            sensor_z, math.hypot(sensor_x, dy)))
        self.assertLess(elevation, self.min_angle)
        self.assertAlmostEqual(elevation, -56.43, delta=0.15)


if __name__ == "__main__":
    unittest.main()
