#!/usr/bin/env python3

import math
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / "models" / "mine_uav_800_no_gps" / "model.sdf"
WRAPPER = ROOT / "models" / "iris_mid360_shaft_8m" / "iris_mid360_shaft_8m.sdf"
AIRFRAME = ROOT / "px4_airframes" / "1018_gazebo-classic_iris_xy_vision"
WORLD = ROOT / "worlds" / "shaft_500m_logic_sitl.world"
ROCK_MODEL = ROOT / "models" / "shaft_rock_500m" / "model.sdf"
ROCK_MESH = ROOT / "models" / "shaft_rock_500m" / "meshes" / "shaft_rock_inner.obj"
VEHICLE_VISUAL_MESH = (ROOT / "models" / "mine_uav_800_no_gps" /
                       "meshes" / "uav_body_step_visual.stl")
TASK2_LAUNCH = ROOT / "launch" / "task2_shaft_500m_opticalflow_px4_sitl.launch"
VEHICLE_CONFIG = ROOT / "config" / "mine_uav_800_no_gps.yaml"
FASTLIO_CONFIG = ROOT / "config" / "fastlio_gazebo_mid360.yaml"


class Task2VehicleModelTest(unittest.TestCase):
    def setUp(self):
        self.base = ET.parse(BASE).getroot()
        self.wrapper = ET.parse(WRAPPER).getroot()

    def test_total_mass_is_six_kg(self):
        base_mass = sum(float(node.text) for node in self.base.findall(".//mass"))
        lidar_mass = sum(float(node.text) for node in self.wrapper.findall("./model/link/inertial/mass"))
        self.assertAlmostEqual(base_mass + lidar_mass, 6.0, places=6)

    def test_no_gps_and_mid360_mass(self):
        uris = [node.text for node in self.base.findall(".//uri")]
        self.assertNotIn("model://gps", uris)
        self.assertNotIn("model://iris/meshes/iris.stl", uris)
        lidar_mass = float(self.wrapper.find("./model/link/inertial/mass").text)
        self.assertAlmostEqual(lidar_mass, 0.265, places=6)

    def test_provisional_frame_geometry_uses_known_scale(self):
        base_link = self.base.find("./model/link[@name='base_link']")
        collisions = {
            node.attrib["name"]: node for node in base_link.findall("collision")
        }
        centre = collisions["centre_body_collision"].find(
            "geometry/box/size").text
        self.assertEqual([float(value) for value in centre.split()],
                         [0.325, 0.307, 0.10])
        arms = [collisions[name] for name in collisions if name.endswith(
            "_arm_collision")]
        self.assertEqual(len(arms), 4)
        for arm in arms:
            size = [float(value) for value in arm.find(
                "geometry/box/size").text.split()]
            self.assertEqual(size, [0.31, 0.035, 0.025])
        visual_mesh = base_link.find(
            "visual[@name='step_body_visual']/geometry/mesh")
        self.assertEqual(visual_mesh.findtext("uri"),
                         "model://mine_uav_800_no_gps/meshes/"
                         "uav_body_step_visual.stl")
        self.assertEqual(visual_mesh.findtext("scale"), "0.001 0.001 0.001")
        self.assertTrue(VEHICLE_VISUAL_MESH.is_file())

    def test_diagonal_motor_span_and_propeller_size(self):
        links = {node.attrib["name"]: node for node in self.base.findall("./model/link")}
        for first, second in (("rotor_0", "rotor_1"), ("rotor_2", "rotor_3")):
            p1 = [float(v) for v in links[first].find("pose").text.split()[:3]]
            p2 = [float(v) for v in links[second].find("pose").text.split()[:3]]
            span = math.sqrt(sum((a - b) ** 2 for a, b in zip(p1, p2)))
            self.assertAlmostEqual(span, 0.620842582, places=6)
        prop_radii = [float(node.text) for node in self.base.findall(
            "./model/link/collision/geometry/cylinder/radius")]
        self.assertEqual(prop_radii, [0.1905] * 4)

    def test_u7_kv420_six_s_fifteen_inch_approximation(self):
        constants = [float(node.text) for node in self.base.findall(".//motorConstant")]
        max_speeds = [float(node.text) for node in self.base.findall(".//maxRotVelocity")]
        moment_constants = [float(node.text) for node in self.base.findall(".//momentConstant")]
        self.assertEqual(constants, [4.79874427e-5] * 4)
        self.assertEqual(max_speeds, [973.768059] * 4)
        self.assertEqual(moment_constants, [0.00789903182] * 4)
        max_thrust_kgf = constants[0] * max_speeds[0] ** 2 / 9.80665
        self.assertAlmostEqual(max_thrust_kgf, 4.64, places=3)
        self.assertIn("param set-default BAT1_N_CELLS 6", AIRFRAME.read_text())

    def test_canonical_battery_and_mid360_configuration(self):
        config = yaml.safe_load(VEHICLE_CONFIG.read_text())
        battery = config["battery"]
        self.assertEqual(battery["pack_count"], 2)
        self.assertEqual(battery["topology"], "parallel")
        self.assertEqual(battery["cells_in_series"], 6)
        self.assertEqual(battery["model"], "BPX230-6768-22.14")
        self.assertEqual(battery["per_pack_capacity_mah"], 6768)
        self.assertEqual(battery["total_capacity_mah"], 13536)
        self.assertAlmostEqual(battery["nominal_voltage_v"], 22.14)
        self.assertAlmostEqual(battery["full_voltage_v"], 25.5)
        self.assertAlmostEqual(battery["total_energy_wh"], 299.8)
        self.assertAlmostEqual(battery["total_battery_mass_kg"], 1.28)
        self.assertEqual(config["localization"]["lidar"]["model"], "MID360S")
        self.assertEqual(config["vehicle"]["body_frame"]["x_axis"], "forward")
        self.assertEqual(config["vehicle"]["body_frame"]["y_axis"], "left")
        self.assertEqual(config["vehicle"]["body_frame"]["z_axis"], "up")
        self.assertAlmostEqual(config["vehicle"]["adjacent_motor_span_m"],
                               0.439002, places=6)
        self.assertAlmostEqual(config["vehicle"]["diagonal_motor_span_m"],
                               0.620842582, places=9)
        lidar_pose = [float(value) for value in self.wrapper.find(
            "./model/link[@name='mid360_link']/pose").text.split()]
        self.assertEqual(lidar_pose[:3], [0.1315, 0.0, 0.223])
        self.assertAlmostEqual(lidar_pose[4], math.radians(25), places=9)
        fastlio = yaml.safe_load(FASTLIO_CONFIG.read_text())
        self.assertEqual(fastlio["mapping"]["extrinsic_T"],
                         [0.1315, 0.0, 0.203])
        expected_rotation = [
            math.cos(math.radians(25)), 0, math.sin(math.radians(25)),
            0, 1, 0,
            -math.sin(math.radians(25)), 0, math.cos(math.radians(25)),
        ]
        for actual, expected in zip(fastlio["mapping"]["extrinsic_R"],
                                    expected_rotation):
            self.assertAlmostEqual(actual, expected, places=9)
        self.assertFalse(config["vehicle"]["gps_present"])
        airframe = AIRFRAME.read_text()
        self.assertIn("param set-default BAT1_N_CELLS 6", airframe)
        self.assertIn("param set-default BAT1_CAPACITY 13536", airframe)
        self.assertIn("param set-default BAT1_V_CHARGED 4.25", airframe)

    def test_real_px4_migration_is_curated_and_unchanged(self):
        snapshot = {}
        snapshot_path = (ROOT / "params" / "real_snapshots" /
                         "px4_real_2026-09-21_qgc.params")
        for line in snapshot_path.read_text().splitlines():
            if not line or line.startswith("#"):
                continue
            fields = line.split("\t")
            snapshot[fields[2]] = fields[3]

        airframe = AIRFRAME.read_text()
        block = airframe.split(
            "# Compatible controller, limit and filter values", 1)[1]
        block = block.split("# Physical-platform electrical layout", 1)[0]
        migrated = {}
        for line in block.splitlines():
            if line.startswith("param set-default "):
                _, _, name, value = line.split()
                migrated[name] = value

        # MPC_THR_HOVER is intentionally calibrated from the 6 kg Gazebo
        # thrust curve and therefore is not part of the verbatim migration.
        self.assertEqual(len(migrated), 75)
        self.assertNotIn("MPC_THR_HOVER", migrated)
        self.assertIn("param set-default MPC_THR_HOVER 0.57", airframe)
        for name, value in migrated.items():
            self.assertIn(name, snapshot)
            self.assertEqual(value, snapshot[name])
            self.assertFalse(name.startswith((
                "BAT", "CAL_", "MAV_", "PWM_", "RC_", "SENS_", "SER_",
                "SYS_")))

    def test_irregular_five_metre_rock_shaft_is_500_m_deep(self):
        world = ET.parse(WORLD).getroot()
        include_uris = [node.text for node in world.findall("./world/include/uri")]
        self.assertIn("model://shaft_rock_500m", include_uris)
        model = ET.parse(ROCK_MODEL).getroot()
        mesh_uris = [node.text for node in model.findall(".//mesh/uri")]
        self.assertEqual(mesh_uris, [
            "model://shaft_rock_500m/meshes/shaft_rock_inner.obj",
            "model://shaft_rock_500m/meshes/shaft_rock_inner.obj",
        ])
        vertices = []
        for line in ROCK_MESH.read_text().splitlines():
            if line.startswith("v "):
                vertices.append(tuple(float(value) for value in line.split()[1:]))
        wall_vertices = vertices[:501 * 32]
        radii = [math.hypot(x, y) for x, y, _ in wall_vertices]
        heights = [z for _, _, z in wall_vertices]
        self.assertAlmostEqual(max(heights), 0.25, places=6)
        self.assertAlmostEqual(min(heights), -499.75, places=6)
        self.assertAlmostEqual(max(heights) - min(heights), 500.0, places=6)
        self.assertLess(min(radii), 2.40)
        self.assertGreater(max(radii), 2.60)
        self.assertAlmostEqual(sum(radii) / len(radii), 2.5, places=2)
        # Conservative circular propeller envelope from the STEP motor radius.
        envelope = 0.620842582 / 2.0 + 0.1905
        self.assertAlmostEqual(envelope, 0.500921291, places=6)
        self.assertAlmostEqual(2.5 - envelope, 1.999078709, places=6)

    def test_task2_defaults_to_fastlio_sensor_chain(self):
        launch = TASK2_LAUNCH.read_text()
        self.assertIn('<arg name="use_vision_truth" default="false"/>', launch)
        self.assertIn('pkg="fast_lio" type="fastlio_mapping"', launch)
        self.assertIn('type="pointcloud1_to_pointcloud2"', launch)
        self.assertIn('type="fastlio_px4_vision_bridge"', launch)
        self.assertIn('value="fastlio2_vertical_displacement"', launch)


if __name__ == "__main__":
    unittest.main()
