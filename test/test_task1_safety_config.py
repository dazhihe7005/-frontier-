#!/usr/bin/env python3

"""Lock Task-1 to its configured clearance contract."""

import pathlib
import unittest
import xml.etree.ElementTree as ET

import yaml


ROOT = pathlib.Path(__file__).resolve().parents[1]
HARD_CLEARANCE = 0.65
PLANNING_CLEARANCE = 0.8
PREFERRED_CLEARANCE = 1.1
VERTICAL_CLEARANCE = 0.5


class Task1SafetyConfigTest(unittest.TestCase):
    def test_planner_and_live_guard_keep_separate_soft_and_hard_radii(self):
        bridge = yaml.safe_load(
            (ROOT / "config" / "super_px4_command_bridge.yaml").read_text())

        for filename in ("super_task1.yaml", "super_task1_goaf_smooth.yaml"):
            planner = yaml.safe_load((ROOT / "config" / filename).read_text())
            config = planner["super_planner"]
            radius = (config["robot_r"] +
                      config["horizontal_clearance_extra"])
            self.assertAlmostEqual(radius, PLANNING_CLEARANCE, msg=filename)
            self.assertAlmostEqual(
                config["horizontal_planning_clearance"], PLANNING_CLEARANCE,
                msg=filename)
            self.assertAlmostEqual(
                config["horizontal_preferred_clearance"], PREFERRED_CLEARANCE,
                msg=filename)
            self.assertGreaterEqual(
                config["preferred_clearance_penalty"], 0.0, msg=filename)
            self.assertLessEqual(
                config["preferred_clearance_penalty"], 0.5, msg=filename)
            self.assertGreater(
                planner["traj_opt"]["exp_traj"]["penna_attract"], 0.0,
                msg=filename)
            self.assertGreater(
                planner["traj_opt"]["exp_traj"]["attractor_dead_distance_ratio"],
                0.0, msg=filename)
            self.assertAlmostEqual(
                planner["traj_opt"]["exp_traj"]["attractor_dead_distance_ratio"],
                0.35, msg=filename)
            self.assertLessEqual(
                planner["traj_opt"]["exp_traj"]["attractor_dead_distance_ratio"],
                0.5, msg=filename)
            self.assertAlmostEqual(
                config["horizontal_hard_clearance"], HARD_CLEARANCE,
                msg=filename)
            self.assertAlmostEqual(
                config["horizontal_hard_vertical_below"], VERTICAL_CLEARANCE,
                msg=filename)
            self.assertAlmostEqual(
                config["horizontal_hard_vertical_above"], VERTICAL_CLEARANCE,
                msg=filename)
            self.assertAlmostEqual(config["robot_r"], 0.3, msg=filename)
            self.assertAlmostEqual(
                config["clearance_vertical_band"], 0.3,
                msg=filename)
            resolution = planner["rog_map"]["resolution"]
            # The voxel map can only represent multiples of 0.1 m. Continuous
            # corridor and trajectory validation retain the exact thresholds.
            grid_floor = planner["rog_map"]["inflation_step"] * resolution
            self.assertLessEqual(grid_floor, PLANNING_CLEARANCE)
            self.assertLess(PLANNING_CLEARANCE - grid_floor, resolution)
            for key in ("inflation_vertical_below_step",
                        "inflation_vertical_above_step"):
                grid_vertical = planner["rog_map"][key] * resolution
                self.assertLessEqual(grid_vertical, VERTICAL_CLEARANCE)
                self.assertLess(VERTICAL_CLEARANCE - grid_vertical,
                                resolution)

        self.assertAlmostEqual(
            bridge["live_horizontal_clearance"], HARD_CLEARANCE)
        self.assertAlmostEqual(
            bridge["obstacle_monitor_horizontal_clearance"],
            PLANNING_CLEARANCE)
        self.assertAlmostEqual(bridge["obstacle_persistence_duration"], 3.0)
        self.assertAlmostEqual(bridge["obstacle_replan_escape_duration"], 4.0)
        self.assertAlmostEqual(bridge["overspeed_duration"], 1.5)
        self.assertAlmostEqual(bridge["measured_speed_target"], 0.65)
        self.assertAlmostEqual(bridge["measured_speed_feedback_gain"], 1.0)
        self.assertAlmostEqual(bridge["measured_speed_governor_floor"], 0.35)
        self.assertLess(
            bridge["measured_speed_target"],
            bridge["max_measured_speed"])
        self.assertLess(
            bridge["measured_speed_governor_floor"],
            bridge["max_predicted_speed"])
        # Task 1 keeps Fast-LIO/PX4 external-vision Z. The command bridge must
        # not replace it with a fixed-height target or abort merely because a
        # vertical tracking error persists.
        self.assertNotIn("lock_command_altitude_to_reference", bridge)
        self.assertNotIn("max_vertical_tracking_error", bridge)
        self.assertNotIn("vertical_tracking_error_duration", bridge)
        self.assertGreaterEqual(bridge["obstacle_clear_confirm_frames"], 2)
        self.assertAlmostEqual(
            bridge["live_vertical_clearance_below"], VERTICAL_CLEARANCE)
        self.assertAlmostEqual(
            bridge["live_vertical_clearance_above"], VERTICAL_CLEARANCE)
        self.assertGreater(bridge["coverage_vertical_uncertainty_margin"], 0.0)
        self.assertLessEqual(
            bridge["coverage_vertical_uncertainty_margin"], 0.5)
        # Coverage is limited to the collision-height band, so a low wall or
        # post is complete without requiring an artificial vertical span.
        self.assertEqual(bridge["coverage_min_vertical_span"], 0.0)

        decider = yaml.safe_load(
            (ROOT / "config" / "super_exploration_decider.yaml").read_text())
        production = yaml.safe_load(
            (ROOT / "config" / "super_task1.yaml").read_text())["super_planner"]
        self.assertAlmostEqual(
            decider["vehicle_radius"], PLANNING_CLEARANCE)
        self.assertAlmostEqual(
            decider["preferred_vehicle_radius"], PREFERRED_CLEARANCE)
        self.assertTrue(decider["corridor_centering_enabled"])
        self.assertEqual(decider["corridor_centering_min_wall_points"], 12)
        self.assertEqual(decider["corridor_centering_confirm_frames"], 3)
        self.assertEqual(decider["corridor_centering_release_frames"], 3)
        self.assertAlmostEqual(decider["corridor_centering_max_width"], 6.0)
        self.assertAlmostEqual(decider["corridor_centering_filter_gain"], 0.25)
        self.assertAlmostEqual(
            decider["corridor_centering_max_center_jump"], 0.75)
        self.assertAlmostEqual(
            decider["corridor_centering_max_goal_shift"], 0.5)
        self.assertAlmostEqual(
            decider["vehicle_radius"],
            production["horizontal_planning_clearance"])
        self.assertAlmostEqual(
            decider["preferred_vehicle_radius"],
            production["horizontal_preferred_clearance"])
        self.assertAlmostEqual(
            bridge["obstacle_monitor_horizontal_clearance"],
            production["horizontal_planning_clearance"])
        self.assertAlmostEqual(
            bridge["live_horizontal_clearance"],
            production["horizontal_hard_clearance"])
        self.assertAlmostEqual(
            decider["vertical_vehicle_clearance_below"], VERTICAL_CLEARANCE)
        self.assertAlmostEqual(
            decider["vertical_vehicle_clearance_above"], VERTICAL_CLEARANCE)
        self.assertAlmostEqual(
            decider["return_super_radius"], PLANNING_CLEARANCE)
        self.assertTrue(decider["corridor_preview_enabled"])
        self.assertGreater(
            decider["corridor_preview_distance"],
            decider["forward_lookahead_distance"])
        self.assertLessEqual(
            decider["corridor_preview_max_goal_shift"],
            decider["forward_goal_lateral_search_width"])
        self.assertGreaterEqual(decider["corridor_preview_min_slices"], 3)
        self.assertEqual(decider["forward_goal_lateral_search_width"], 4.0)
        self.assertEqual(
            decider["lateral_route_switch_clearance_advantage"], 0.25)
        self.assertEqual(decider["forward_lookahead_distance"], 4.0)
        self.assertEqual(decider["max_frontier_goal_distance"], 4.0)
        self.assertEqual(decider["forward_goal_handover_distance"], 2.0)
        self.assertTrue(decider["exploration_goal_queue_enabled"])
        self.assertIn(
            decider["exploration_goal_queue_max_future_goals"], (2, 3))
        self.assertGreaterEqual(
            decider["exploration_goal_queue_horizon"], 8.0)
        self.assertGreater(
            decider["exploration_goal_queue_spacing"],
            decider["forward_goal_handover_distance"])
        self.assertGreater(
            decider["exploration_goal_queue_end_reserve"], 0.0)
        self.assertEqual(decider["no_frontier_timeout"], 10.0)
        self.assertAlmostEqual(
            decider["min_observation_height_above_home"], -0.30)
        self.assertAlmostEqual(
            decider["max_observation_height_above_home"], 0.20)
        self.assertAlmostEqual(decider["return_max_height_above_home"], 0.20)
        self.assertLess(
            decider["min_observation_height_above_home"],
            decider["cruise_height_above_home"])
        self.assertAlmostEqual(
            decider["battery_return_confirm_duration"], 3.0)
        self.assertAlmostEqual(decider["battery_return_hysteresis"], 0.03)
        self.assertLess(
            decider["forward_goal_handover_distance"],
            decider["forward_lookahead_distance"])
        self.assertEqual(decider["max_dead_end_heading_reversals"], 1)
        self.assertEqual(decider["max_task_lateral_offset"], 10.0)
        self.assertEqual(production["yaw_tracking_sample_period"], 0.5)

        sitl = ET.parse(ROOT / "launch" / "task1_px4_sitl.launch").getroot()
        launch_args = {
            node.attrib["name"]: node.attrib.get("default")
            for node in sitl.findall("./arg")
        }
        self.assertEqual(launch_args["decider_vehicle_radius"], "0.8")
        self.assertEqual(
            launch_args["decider_preferred_vehicle_radius"], "1.1")
        self.assertEqual(launch_args["no_frontier_timeout"], "10.0")

        bridge_launch = ET.parse(
            ROOT / "launch" / "super_px4_command_bridge.launch").getroot()
        bridge_args = {
            node.attrib["name"]: node.attrib.get("default")
            for node in bridge_launch.findall("./arg")
        }
        self.assertEqual(bridge_args["overspeed_duration"], "1.5")

    def test_autostart_freezes_realtime_clock_during_ros_flight(self):
        unit = (ROOT / "systemd" / "mine-uav-task1.service").read_text()
        self.assertIn(
            "ExecStartPre=+/home/nuc/frontier-upload/scripts/stabilize_task1_clock.sh",
            unit)
        self.assertIn(
            "ExecStopPost=+/usr/bin/systemctl start systemd-timesyncd.service",
            unit)
        guard = (ROOT / "scripts" / "stabilize_task1_clock.sh").read_text()
        self.assertIn("systemctl stop systemd-timesyncd.service", guard)

        real_launch = (ROOT / "launch" / "task1_real.launch").read_text()
        self.assertNotIn("task1_px4_height_reference_guard", real_launch)

if __name__ == "__main__":
    unittest.main()
