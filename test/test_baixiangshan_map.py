#!/usr/bin/env python3

import json
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
MODEL_DIR = ROOT / "models" / "baixianshan_tunnel"


class BaiXiangShanMapTest(unittest.TestCase):
    def test_export_is_metric_spawn_relative_and_bounded(self):
        metadata = json.loads((MODEL_DIR / "export_metadata.json").read_text())
        self.assertEqual(metadata["source_units"], "metres")
        self.assertEqual(metadata["gazebo_uniform_scale"], 1.0)
        self.assertEqual(metadata["source_to_gazebo_yaw_degrees"], -90.0)
        self.assertEqual(metadata["gazebo_spawn_xyz"], [0.0, 0.0, 0.1404])
        triangles = sum(item["triangles"] for item in metadata["exports"])
        self.assertGreater(triangles, 500_000)
        self.assertLess(triangles, 1_500_000)

    def test_visual_ray_and_collision_geometry_are_identical(self):
        model = ET.parse(MODEL_DIR / "model.sdf").getroot()
        link = model.find("./model/link")
        visuals = {
            item.attrib["name"][:-len("_visual")]:
                item.findtext("geometry/mesh/uri")
            for item in link.findall("visual")
        }
        collisions = {
            item.attrib["name"][:-len("_collision")]:
                item.findtext("geometry/mesh/uri")
            for item in link.findall("collision")
        }
        self.assertEqual(visuals, collisions)
        self.assertEqual(set(visuals), {"rock", "ground", "roof", "infrastructure"})
        for uri in visuals.values():
            prefix = "model://baixianshan_tunnel/"
            self.assertTrue(uri.startswith(prefix))
            relative = uri[len(prefix):]
            self.assertTrue((MODEL_DIR / relative).is_file())
        visual_scales = {
            item.attrib["name"][:-len("_visual")]:
                item.findtext("geometry/mesh/scale")
            for item in link.findall("visual")
        }
        collision_scales = {
            item.attrib["name"][:-len("_collision")]:
                item.findtext("geometry/mesh/scale")
            for item in link.findall("collision")
        }
        self.assertEqual(visual_scales, collision_scales)
        self.assertEqual(set(visual_scales.values()), {"1 1 1"})

    def test_world_and_autonomous_launch_use_survey_map(self):
        world = ET.parse(ROOT / "worlds" / "baixianshan_tunnel_sitl.world").getroot()
        uris = [node.text for node in world.findall("./world/include/uri")]
        self.assertEqual(uris, ["model://baixianshan_tunnel"])
        launch = ET.parse(ROOT / "launch" / "task1_baixiangshan_px4_sitl.launch").getroot()
        args = {
            node.attrib["name"]: node.attrib["value"]
            for node in launch.findall("./include/arg")
        }
        self.assertIn("baixianshan_tunnel_sitl.world", args["world"])
        self.assertEqual(args["spawn_x"], "-7.0")
        self.assertEqual(args["spawn_y"], "0.5")
        self.assertEqual(args["spawn_z"], "-0.0334")
        self.assertEqual(args["spawn_yaw"], "0.0")
        self.assertEqual(args["takeoff_yaw_offset"], "0.0")
        self.assertEqual(args["mission_heading_offset"], "0.0")
        self.assertEqual(args["use_absolute_mission_heading"], "true")
        self.assertEqual(
            args["absolute_mission_heading_yaw"], "0.0")
        self.assertEqual(args["forward_corridor_half_width"], "1.25")
        self.assertEqual(args["forward_goal_lateral_search_width"], "0.5")
        self.assertEqual(args["max_frontier_goal_distance"], "4.0")
        self.assertEqual(args["max_frontier_goal_vertical_step"], "0.35")
        self.assertEqual(args["max_task_lateral_offset"], "8.0")
        self.assertEqual(args["decider_vehicle_radius"], "1.0")
        self.assertEqual(args["decider_vertical_vehicle_radius"], "1.0")
        self.assertEqual(args["carve_traversed_vehicle_envelope"], "true")
        self.assertEqual(
            launch.find("./arg[@name='takeoff_height']").attrib["default"],
            "1.5",
        )
        self.assertEqual(args["map_closure_min_progress"], "80.0")
        self.assertEqual(args["use_map_closure_completion"], "true")
        self.assertEqual(args["max_dead_end_heading_reversals"], "1")
        self.assertEqual(args["dead_end_reversal_delay"], "4.0")
        self.assertEqual(args["max_exploration_radius"], "180.0")
        self.assertEqual(args["decider_max_map_radius"], "220.0")
        self.assertEqual(args["bridge_max_horizontal_radius"], "220.0")
        self.assertEqual(args["bridge_max_height"], "8.0")

        generic = ET.parse(ROOT / "launch" / "task1_px4_sitl.launch").getroot()
        generic_defaults = {
            node.attrib["name"]: node.attrib.get("default")
            for node in generic.findall("./arg")
        }
        self.assertEqual(
            generic_defaults["carve_traversed_vehicle_envelope"], "false")
        self.assertEqual(generic_defaults["decider_vehicle_radius"], "1.0")
        self.assertEqual(
            generic_defaults["decider_vertical_vehicle_radius"], "1.0")

        operator = generic.find("./node[@name='sitl_task_operator']")
        operator_params = {
            node.attrib["name"]: node.attrib["value"]
            for node in operator.findall("./param")
        }
        self.assertEqual(operator_params["ground_z_stable_duration"], "3.0")
        self.assertEqual(
            operator_params["ground_z_stability_tolerance"], "0.15")

        super_config = (ROOT / "config" / "super_task1.yaml").read_text()
        self.assertIn("robot_r: 1.0", super_config)
        self.assertIn("inflation_step: 0", super_config)


if __name__ == "__main__":
    unittest.main()
