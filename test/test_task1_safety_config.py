#!/usr/bin/env python3

"""Lock Task-1 to its configured clearance contract."""

import hashlib
import json
import pathlib
import unittest
import xml.etree.ElementTree as ET

import yaml


ROOT = pathlib.Path(__file__).resolve().parents[1]
HARD_CLEARANCE = 0.65
PLANNING_CLEARANCE = 0.8
PREFERRED_CLEARANCE = 1.1
TASK1_HARD_CLEARANCE = 1.0
TASK1_PREFERRED_CLEARANCE = 1.5
VERTICAL_CLEARANCE = 0.5
NOMINAL_SPEED = 0.9
SPEED_LIMIT = 1.1
MEASURED_SPEED_TARGET = 1.0


class Task1SafetyConfigTest(unittest.TestCase):
    def test_super_process_uses_separate_task_profiles(self):
        real = ET.parse(ROOT / "launch" / "task1_real.launch").getroot()
        sitl = ET.parse(ROOT / "launch" / "task1_px4_sitl.launch").getroot()
        expected = (
            (real, "super_task1_goaf_1m.yaml", "super_task1.yaml"),
            (sitl, "super_config_path", "super_task1_goaf_smooth.yaml"),
        )
        for root, goaf_name, tunnel_name in expected:
            nodes = root.findall(".//node[@type='task_profiled_super_launcher.py']")
            self.assertEqual(len(nodes), 1)
            params = {item.attrib["name"]: item.attrib["value"]
                      for item in nodes[0].findall("param")}
            self.assertIn(goaf_name, params["goaf_config_path"])
            self.assertIn(tunnel_name, params["tunnel_config_path"])
        for name in ("five_meter_avoidance_real.launch",
                     "five_meter_avoidance_sitl.launch"):
            text = (ROOT / "launch" / name).read_text()
            self.assertIn("super_task1_goaf_1m.yaml", text)

    def test_bridge_launch_cannot_override_production_speed_limits(self):
        launch_root = ET.parse(
            ROOT / "launch" / "super_px4_command_bridge.launch").getroot()
        defaults = {
            item.attrib["name"]: item.attrib.get("default")
            for item in launch_root.findall("arg")
        }
        self.assertAlmostEqual(float(defaults["max_speed"]), SPEED_LIMIT)
        self.assertAlmostEqual(
            float(defaults["max_measured_speed"]), SPEED_LIMIT)
        self.assertAlmostEqual(
            float(defaults["max_predicted_speed"]), SPEED_LIMIT)

        sitl_root = ET.parse(
            ROOT / "launch" / "task1_px4_sitl.launch").getroot()
        sitl_overrides = {
            item.attrib["name"]: item.attrib["value"]
            for item in sitl_root.findall(".//include/arg")
            if item.attrib.get("name") in {
                "max_speed", "max_measured_speed", "max_predicted_speed"
            }
        }
        self.assertEqual(set(sitl_overrides), {
            "max_speed", "max_measured_speed", "max_predicted_speed"
        })
        for value in sitl_overrides.values():
            self.assertAlmostEqual(float(value), SPEED_LIMIT)

    def test_planner_and_live_guard_keep_separate_soft_and_hard_radii(self):
        bridge = yaml.safe_load(
            (ROOT / "config" / "super_px4_command_bridge.yaml").read_text())

        for filename in ("super_task1.yaml", "super_task1_goaf_smooth.yaml"):
            planner = yaml.safe_load((ROOT / "config" / filename).read_text())
            self.assertAlmostEqual(
                planner["traj_opt"]["boundary"]["max_vel"],
                NOMINAL_SPEED, msg=filename)
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

        for filename in ("super_task1_goaf_1m.yaml",
                         "super_task1_goaf_1m_smooth.yaml"):
            planner = yaml.safe_load((ROOT / "config" / filename).read_text())
            config = planner["super_planner"]
            self.assertAlmostEqual(
                config["robot_r"] + config["horizontal_clearance_extra"],
                TASK1_HARD_CLEARANCE, msg=filename)
            self.assertAlmostEqual(config["horizontal_planning_clearance"],
                                   TASK1_HARD_CLEARANCE, msg=filename)
            self.assertAlmostEqual(config["horizontal_preferred_clearance"],
                                   TASK1_PREFERRED_CLEARANCE, msg=filename)
            self.assertAlmostEqual(config["horizontal_hard_clearance"],
                                   TASK1_HARD_CLEARANCE, msg=filename)
            self.assertAlmostEqual(
                planner["rog_map"]["inflation_step"] *
                planner["rog_map"]["resolution"], TASK1_HARD_CLEARANCE,
                msg=filename)

        self.assertAlmostEqual(
            bridge["live_horizontal_clearance"], HARD_CLEARANCE)
        self.assertAlmostEqual(
            bridge["goaf_live_horizontal_clearance"],
            TASK1_HARD_CLEARANCE)
        self.assertEqual(
            bridge["max_horizontal_radius"], 0.0,
            "production bridge must not impose a home-distance limit")
        self.assertAlmostEqual(
            bridge["obstacle_monitor_horizontal_clearance"],
            PLANNING_CLEARANCE)
        self.assertGreaterEqual(
            bridge["obstacle_dynamic_monitor_max_clearance"], 2.0)
        self.assertAlmostEqual(bridge["obstacle_reaction_time"], 0.25)
        self.assertAlmostEqual(
            bridge["obstacle_emergency_deceleration"], 1.75)
        self.assertLessEqual(
            bridge["obstacle_emergency_min_duration"], 0.25)
        self.assertGreaterEqual(bridge["obstacle_emergency_max_jerk"], 20.0)
        self.assertGreaterEqual(
            bridge["obstacle_dynamic_clearance_buffer"], 0.10)
        self.assertAlmostEqual(bridge["coverage_lateral_half_width"], 1.0)
        self.assertEqual(bridge["coverage_reject_frames"], 5)
        self.assertAlmostEqual(bridge["obstacle_persistence_duration"], 0.5)
        self.assertAlmostEqual(bridge["obstacle_replan_escape_duration"], 4.0)
        self.assertAlmostEqual(bridge["coverage_min_horizontal_span"], 0.5)
        self.assertEqual(bridge["coverage_narrow_vertical_min_points"], 120)
        self.assertAlmostEqual(
            bridge["coverage_narrow_vertical_min_horizontal_span"], 0.2)
        self.assertAlmostEqual(
            bridge["coverage_narrow_vertical_min_vertical_span"], 0.8)
        self.assertEqual(
            bridge["coverage_narrow_vertical_retain_min_points"], 45)
        self.assertAlmostEqual(
            bridge["coverage_narrow_vertical_retain_min_horizontal_span"],
            0.1)
        self.assertAlmostEqual(
            bridge["coverage_narrow_vertical_retain_min_vertical_span"],
            0.7)
        self.assertGreater(
            bridge["obstacle_cloud_fault_timeout"],
            bridge["obstacle_cloud_timeout"])
        self.assertGreaterEqual(bridge["obstacle_cloud_fault_timeout"], 2.0)
        self.assertAlmostEqual(bridge["overspeed_duration"], 1.5)
        self.assertAlmostEqual(bridge["max_speed"], SPEED_LIMIT)
        self.assertAlmostEqual(bridge["max_measured_speed"], SPEED_LIMIT)
        self.assertAlmostEqual(bridge["max_predicted_speed"], SPEED_LIMIT)
        self.assertAlmostEqual(
            bridge["measured_speed_target"], MEASURED_SPEED_TARGET)
        self.assertAlmostEqual(bridge["measured_speed_feedback_gain"], 1.0)
        self.assertAlmostEqual(bridge["measured_speed_governor_floor"], 0.35)
        # These values must track the current physical PX4 MPC_XY_P/MPC_Z_P
        # parameters so the companion-side requested-speed estimate is not
        # systematically high or low.
        self.assertAlmostEqual(bridge["tracking_position_gain_xy"], 0.75)
        self.assertAlmostEqual(bridge["tracking_position_gain_z"], 1.0)
        self.assertAlmostEqual(
            bridge["max_governor_tracking_error"], HARD_CLEARANCE)
        self.assertAlmostEqual(
            bridge["max_governor_along_track_lag"], 1.5)
        self.assertGreater(
            bridge["max_governor_along_track_lag"],
            bridge["max_governor_tracking_error"])
        self.assertLess(
            bridge["measured_speed_target"],
            bridge["max_measured_speed"])
        self.assertLess(
            bridge["measured_speed_governor_floor"],
            bridge["max_predicted_speed"])
        # Task 1 keeps Omni-LIVO/PX4 external-vision Z. The command bridge must
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
        self.assertGreater(
            bridge["max_managed_reacquire_distance"],
            bridge["max_initial_command_distance"])
        self.assertLessEqual(
            bridge["max_governor_along_track_lag"],
            bridge["max_managed_reacquire_distance"])
        self.assertGreater(
            bridge["max_managed_reacquire_yaw_error"],
            bridge["max_initial_yaw_error"])
        self.assertGreaterEqual(
            bridge["preoffboard_vision_recovery_timeout"], 0.2)

        production_fsm = yaml.safe_load(
            (ROOT / "config" / "super_task1.yaml").read_text())["fsm"]
        self.assertTrue(production_fsm["require_execution_authorization"])
        self.assertEqual(
            production_fsm["execution_authorization_topic"],
            bridge["execution_authorization_topic"])

        decider = yaml.safe_load(
            (ROOT / "config" / "super_exploration_decider.yaml").read_text())
        self.assertAlmostEqual(
            decider["goaf_front_obstacle_min_lateral_span"], 0.5)
        self.assertAlmostEqual(
            decider["goaf_front_obstacle_min_vertical_span"], 0.5)
        self.assertAlmostEqual(decider["front_obstacle_min_lateral_span"], 1.0)
        self.assertAlmostEqual(decider["front_obstacle_min_vertical_span"], 0.8)
        scheduler = yaml.safe_load(
            (ROOT / "config" / "mission_scheduler.yaml").read_text())
        self.assertEqual(scheduler["goaf_trigger_channel"], 8)   # CH9
        self.assertEqual(scheduler["task3_trigger_channel"], 5)  # CH6
        self.assertTrue(scheduler["task3_task_available"])
        self.assertEqual(scheduler["tunnel_trigger_channel"], 6)  # CH7
        self.assertTrue(scheduler["tunnel_task_available"])
        exploration_launch = ET.parse(
            ROOT / "launch" / "super_exploration_decider.launch").getroot()
        exploration_args = {
            node.attrib["name"]: node.attrib.get("default")
            for node in exploration_launch.findall("./arg")
        }
        self.assertEqual(
            exploration_args["tunnel_route_lock_min_turn_deg"], "35.0")
        self.assertEqual(
            exploration_args["tunnel_straight_preference_angle_deg"], "35.0")
        self.assertEqual(
            exploration_args["tunnel_connected_turn_max_angle_deg"], "120.0")
        self.assertEqual(
            exploration_args["tunnel_connected_turn_confirm_frames"], "3")
        self.assertEqual(
            exploration_args["tunnel_return_breadcrumb_spacing"], "1.0")
        self.assertEqual(scheduler["shaft_trigger_channel"], 10)  # CH11
        self.assertTrue(scheduler["shaft_task_available"])
        shaft = yaml.safe_load(
            (ROOT / "config" / "shaft_mission_logic.yaml").read_text())
        self.assertEqual(
            shaft["required_depth_source"], "omnilivo_planning_odometry")
        self.assertEqual(shaft["required_range_frame"], "sddm_down")
        self.assertAlmostEqual(shaft["max_depth_sigma_m"], 5.0)
        self.assertAlmostEqual(shaft["entrance_tolerance"], 5.0)
        production = yaml.safe_load(
            (ROOT / "config" / "super_task1.yaml").read_text())["super_planner"]
        self.assertEqual(
            bridge["min_height"], production["flight_min_z"],
            "bridge and SUPER must share the planning-frame lower bound")
        self.assertEqual(
            bridge["max_height"], production["flight_max_z"],
            "bridge and SUPER must share the planning-frame upper bound")
        self.assertAlmostEqual(
            decider["vehicle_radius"], PLANNING_CLEARANCE)
        self.assertAlmostEqual(
            decider["goaf_vehicle_radius"], TASK1_HARD_CLEARANCE)
        self.assertAlmostEqual(
            decider["preferred_vehicle_radius"], PREFERRED_CLEARANCE)
        self.assertAlmostEqual(
            decider["goaf_preferred_vehicle_radius"],
            TASK1_PREFERRED_CLEARANCE)
        self.assertEqual(decider["mission_profile"], "goaf")
        self.assertEqual(
            decider["max_exploration_radius_from_home"], 0.0,
            "production exploration must end on terminal/no-route or battery, "
            "not a fixed home distance")
        # The old, flight-tested tunnel tuning stays loaded byte-for-byte at
        # the data-model level. The goaf profile suppresses only its two
        # corridor preferences in code; Task 4 reuses these values unchanged.
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
            production["horizontal_planning_clearance"],
            msg="Task 4 retains the shared SUPER planning floor")
        self.assertAlmostEqual(
            decider["preferred_vehicle_radius"],
            production["horizontal_preferred_clearance"])
        self.assertAlmostEqual(
            bridge["obstacle_monitor_horizontal_clearance"],
            production["horizontal_planning_clearance"])
        self.assertAlmostEqual(
            bridge["live_horizontal_clearance"],
            production["horizontal_hard_clearance"],
            msg="Task 4 retains the shared SUPER hard radius")
        goaf = yaml.safe_load(
            (ROOT / "config" / "super_task1_goaf_1m.yaml").read_text())[
                "super_planner"]
        self.assertAlmostEqual(bridge["goaf_live_horizontal_clearance"],
                               goaf["horizontal_hard_clearance"])
        self.assertAlmostEqual(decider["goaf_vehicle_radius"],
                               goaf["horizontal_planning_clearance"])
        self.assertAlmostEqual(decider["goaf_preferred_vehicle_radius"],
                               goaf["horizontal_preferred_clearance"])
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
        self.assertGreaterEqual(
            decider["corridor_preview_min_reliable_center_band_width"],
            2.0 * decider["corridor_preview_lateral_step"])
        self.assertEqual(decider["forward_goal_lateral_search_width"], 4.0)
        self.assertEqual(
            decider["lateral_route_switch_clearance_advantage"], 0.25)
        self.assertEqual(decider["forward_lookahead_distance"], 4.0)
        self.assertEqual(decider["goaf_forward_lookahead_distance"], 6.0)
        self.assertEqual(decider["max_frontier_goal_distance"], 4.0)
        self.assertEqual(decider["forward_goal_handover_distance"], 3.0)
        self.assertEqual(decider["return_segment_length"], 3.0)
        self.assertEqual(decider["return_goal_handover_distance"], 2.0)
        self.assertGreater(
            decider["return_segment_length"],
            decider["return_goal_handover_distance"])
        self.assertTrue(decider["exploration_goal_queue_enabled"])
        self.assertIn(
            decider["exploration_goal_queue_max_future_goals"], (2, 3))
        self.assertGreaterEqual(
            decider["exploration_goal_queue_horizon"], 8.0)
        self.assertEqual(
            decider["goaf_exploration_goal_queue_horizon"], 15.0)
        self.assertLess(
            decider["goaf_forward_lookahead_distance"],
            production["planning_horizon"])
        self.assertGreater(
            decider["exploration_goal_queue_spacing"],
            decider["forward_goal_handover_distance"])
        self.assertEqual(decider["exploration_goal_queue_spacing"], 3.5)
        self.assertGreaterEqual(
            decider["frontier_search_radius"],
            decider["goaf_exploration_goal_queue_horizon"] + 5.0)
        self.assertGreater(
            decider["exploration_goal_queue_end_reserve"], 0.0)
        self.assertEqual(decider["no_frontier_timeout"], 7.0)
        self.assertEqual(decider["map_closure_no_frontier_time"], 10.0)
        self.assertTrue(decider["return_on_confirmed_dead_end"])
        self.assertTrue(decider["connected_turn_continuation_enabled"])
        self.assertGreater(decider["connected_turn_max_angle_deg"], 45.0)
        self.assertLess(decider["connected_turn_max_angle_deg"], 90.0)
        self.assertAlmostEqual(
            decider["min_observation_height_above_home"], -0.30)
        self.assertAlmostEqual(
            decider["max_observation_height_above_home"], 0.20)
        self.assertEqual(
            decider["tunnel_min_planning_height"],
            production["flight_min_z"])
        self.assertEqual(
            decider["tunnel_max_planning_height"],
            production["flight_max_z"])
        self.assertAlmostEqual(decider["return_max_height_above_home"], 0.20)
        self.assertAlmostEqual(decider["return_home_arrival_radius"], 0.75)
        self.assertLess(
            decider["min_observation_height_above_home"],
            decider["cruise_height_above_home"])
        self.assertAlmostEqual(
            decider["battery_return_confirm_duration"], 3.0)
        self.assertAlmostEqual(decider["battery_return_hysteresis"], 0.03)
        self.assertLess(
            decider["forward_goal_handover_distance"],
            decider["forward_lookahead_distance"])
        self.assertEqual(decider["max_dead_end_heading_reversals"], 0)
        self.assertEqual(decider["max_task_lateral_offset"], 10.0)
        for filename in ("super_task1.yaml", "super_task1_goaf_smooth.yaml"):
            yaw_config = yaml.safe_load(
                (ROOT / "config" / filename).read_text())["super_planner"]
            self.assertEqual(
                yaw_config["yaw_tracking_sample_period"], 0.2,
                msg=filename)
            self.assertTrue(yaw_config["yaw_prealign_from_rest"], msg=filename)
            self.assertAlmostEqual(
                yaw_config["yaw_prealign_threshold"], 0.35,
                msg=filename)
            self.assertAlmostEqual(
                yaw_config["yaw_prealign_min_duration"], 0.5,
                msg=filename)

        sitl = ET.parse(ROOT / "launch" / "task1_px4_sitl.launch").getroot()
        launch_args = {
            node.attrib["name"]: node.attrib.get("default")
            for node in sitl.findall("./arg")
        }
        self.assertEqual(launch_args["decider_vehicle_radius"], "0.8")
        self.assertEqual(launch_args["goaf_decider_vehicle_radius"], "1.0")
        self.assertEqual(launch_args["mission_profile"], "goaf")
        self.assertEqual(
            launch_args["corridor_centering_enabled"], "true")
        self.assertEqual(
            launch_args["corridor_preview_enabled"], "true")
        self.assertEqual(
            launch_args["decider_preferred_vehicle_radius"], "1.1")
        self.assertEqual(
            launch_args["goaf_decider_preferred_vehicle_radius"], "1.5")
        decider_node = sitl.find("./node[@name='super_exploration_decider']")
        decider_params = {node.attrib["name"]: node.attrib["value"]
                          for node in decider_node.findall("param")}
        self.assertEqual(decider_params["vehicle_radius"],
                         "$(arg decider_vehicle_radius)")
        self.assertEqual(decider_params["goaf_vehicle_radius"],
                         "$(arg goaf_decider_vehicle_radius)")
        self.assertEqual(decider_params["goaf_preferred_vehicle_radius"],
                         "$(arg goaf_decider_preferred_vehicle_radius)")
        for scenario in ("task1_40m_platform_sitl.launch",
                         "task1_maze_sitl.launch"):
            root = ET.parse(ROOT / "launch" / scenario).getroot()
            include_args = {node.attrib["name"]: node.attrib["value"]
                            for node in root.findall(".//include/arg")}
            self.assertEqual(include_args["goaf_decider_vehicle_radius"],
                             "$(arg decider_vehicle_radius)", scenario)
            self.assertEqual(
                include_args["goaf_decider_preferred_vehicle_radius"],
                "$(arg decider_preferred_vehicle_radius)", scenario)
        self.assertEqual(launch_args["no_frontier_timeout"], "7.0")
        self.assertEqual(
            launch_args["map_closure_no_frontier_time"], "10.0")

        tunnel = ET.parse(
            ROOT / "launch" / "task1_baixiangshan_px4_sitl.launch").getroot()
        tunnel_include = tunnel.find("./include")
        self.assertIsNotNone(tunnel_include)
        tunnel_args = {
            node.attrib["name"]: node.attrib.get("value")
            for node in tunnel_include.findall("./arg")
        }
        self.assertEqual(tunnel_args["mission_profile"], "tunnel")
        self.assertEqual(
            tunnel_args["sitl_exploration_trigger_channel"], "6")
        self.assertEqual(tunnel_args["tunnel_trigger_channel"], "6")
        self.assertEqual(tunnel_args["tunnel_task_available"], "true")
        self.assertEqual(
            tunnel_args["corridor_centering_enabled"], "true")
        self.assertEqual(tunnel_args["corridor_preview_enabled"], "true")
        self.assertEqual(tunnel_args["tunnel_min_planning_height"], "-3.0")
        self.assertEqual(tunnel_args["tunnel_max_planning_height"], "8.0")
        self.assertEqual(
            tunnel_args["tunnel_min_planning_height"],
            tunnel_args["bridge_min_height"])
        self.assertEqual(
            tunnel_args["tunnel_max_planning_height"],
            tunnel_args["bridge_max_height"])

        # Freeze every scenario-specific value from the pre-split Task-1
        # Bai Xiang Shan wrapper. Task 4 may add identity/routing arguments
        # and the explicit planning-frame height contract required to match
        # its bridge fence, but it must not silently retune other values.
        task4_only_args = {
            "sitl_exploration_trigger_channel",
            "tunnel_trigger_channel",
            "tunnel_task_available",
            "mission_profile",
            "corridor_centering_enabled",
            "corridor_preview_enabled",
        }
        legacy_tunnel_args = {
            key: value for key, value in tunnel_args.items()
            if key not in task4_only_args
        }
        canonical_tunnel = json.dumps(
            legacy_tunnel_args, sort_keys=True, separators=(",", ":")
        ).encode()
        self.assertEqual(len(legacy_tunnel_args), 55)
        self.assertEqual(
            hashlib.sha256(canonical_tunnel).hexdigest(),
            "6be7648b68eb62dfb6a829143515717121f9634f757065a3365b5116d0eb996e",
        )

        bridge_launch = ET.parse(
            ROOT / "launch" / "super_px4_command_bridge.launch").getroot()
        bridge_args = {
            node.attrib["name"]: node.attrib.get("default")
            for node in bridge_launch.findall("./arg")
        }
        self.assertEqual(bridge_args["overspeed_duration"], "1.5")
        self.assertEqual(bridge_args["max_horizontal_radius"], "0.0")
        self.assertEqual(bridge_args["min_height"], "-5.0")
        self.assertEqual(bridge_args["max_height"], "8.0")

        self.assertEqual(launch_args["max_exploration_radius"], "0.0")
        self.assertEqual(launch_args["bridge_max_horizontal_radius"], "0.0")
        self.assertEqual(launch_args["bridge_min_height"], "-5.0")
        self.assertEqual(launch_args["bridge_max_height"], "8.0")

    def test_task4_preserves_pre_split_task1_decider_tuning(self):
        decider = yaml.safe_load(
            (ROOT / "config" / "super_exploration_decider.yaml").read_text())
        self.assertTrue(decider["battery_start_guard_enabled"])
        self.assertFalse(decider["battery_percentage_return_enabled"])
        self.assertEqual(decider["battery_return_voltage"], 21.6)
        self.assertEqual(decider["battery_start_minimum_voltage"], 22.2)
        self.assertEqual(decider["forward_height_penalty"], 3.0)
        self.assertEqual(decider["max_frontier_goal_vertical_step"], 0.35)
        self.assertEqual(decider.pop("mission_profile"), "goaf")
        # These parameters are goaf-only and ignored when the active
        # scheduler profile is Task 4/tunnel, so they are not part of the
        # frozen tunnel compatibility model below.
        self.assertEqual(decider.pop("goaf_vehicle_radius"), 1.0)
        self.assertEqual(decider.pop("goaf_preferred_vehicle_radius"), 1.5)
        self.assertEqual(decider.pop("goaf_forward_lookahead_distance"), 6.0)
        self.assertEqual(
            decider.pop("goaf_exploration_goal_queue_horizon"), 15.0)
        self.assertTrue(decider.pop("task1_end_wall_return_enabled"))
        self.assertEqual(decider.pop("task1_end_wall_return_distance"), 5.0)
        self.assertEqual(
            decider.pop("goaf_front_obstacle_min_lateral_span"), 0.5)
        self.assertEqual(
            decider.pop("goaf_front_obstacle_min_vertical_span"), 0.5)
        canonical = json.dumps(
            decider, sort_keys=True, separators=(",", ":")
        ).encode()
        # SHA-256 of the shared Task-1/Task-4 YAML data model. The baseline was
        # deliberately advanced when the production home-distance fence was
        # disabled after both 2026-10-24 flights stopped at exactly 35 m.
        # It was advanced again when Task 4 received an absolute planning-
        # frame height contract matching SUPER, so inclined tunnels no longer
        # inherit Task 1's narrow launch-hover height band.
        # This baseline includes the 0.75 m reliable preview-centre band added
        # after the 2026-10-23 flight exposed a one-cell unknown-edge false
        # narrowing; it also includes the 0.75 m launch-area arrival contract
        # added after Task-1 SITL reached home but its zero-length final goal
        # was rejected, and the three-metre return leg required to hand over
        # at two metres instead of stopping at every breadcrumb. Hard
        # clearance and ordinary route feasibility remain intact. The
        # baseline was advanced after the 2026-10-24 23:02 Task-4 flight to
        # use a 20 m rolling computation window and a 3 m/3.5 m early-handover
        # contract; neither change reduces clearance or tunnel extent.
        # Percentage admission and return are explicitly disabled while the
        # production PX4 lacks reliable current/capacity estimation. Raw
        # voltage now blocks a new mission below 22.2 V and independently
        # starts verified return below 21.6 V. The baseline also includes the
        # flight-log-driven 0.35 m vertical-leg limit and stronger soft height
        # preference; inclined tunnels remain supported across repeated legs.
        self.assertEqual(
            hashlib.sha256(canonical).hexdigest(),
            "045c1557036a074058f2964f1fdadf400d6d86b026afd936493fad2302f78d18",
        )

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

        self.assertIn(
            "ExecStart=/home/nuc/frontier-upload/scripts/"
            "start_task1_with_telemetry.sh",
            unit)
        startup = (
            ROOT / "scripts" / "start_task1_with_telemetry.sh"
        ).read_text()
        self.assertIn(
            "roslaunch --wait nuc_mavlink_sender nuc_mavlink_sender.launch",
            startup)
        self.assertLess(
            startup.index("roslaunch --wait nuc_mavlink_sender"),
            startup.index('start_task1_real.sh" &'))
        self.assertIn("respawn_sender:=true", startup)

        real_launch = (ROOT / "launch" / "task1_real.launch").read_text()
        self.assertNotIn("task1_px4_height_reference_guard", real_launch)
        launch_root = ET.fromstring(real_launch)
        launch_args = {
            arg.attrib["name"]: arg.attrib.get("default")
            for arg in launch_root.findall("./arg")
        }
        self.assertEqual(launch_args["start_telemetry_sender"], "false")
        sender_groups = [
            group for group in launch_root.findall("./group")
            if any("nuc_mavlink_sender.launch" in include.attrib["file"]
                   for include in group.findall("./include"))
        ]
        self.assertEqual(len(sender_groups), 1)
        self.assertEqual(
            sender_groups[0].attrib.get("if"),
            "$(arg start_telemetry_sender)")

    def test_real_scheduler_and_decider_are_required_processes(self):
        critical_launches = (
            ("mission_scheduler.launch", "mission_scheduler"),
            ("super_exploration_decider.launch",
             "super_exploration_decider"),
        )
        for filename, expected_name in critical_launches:
            root = ET.parse(ROOT / "launch" / filename).getroot()
            node = root.find("./node")
            self.assertIsNotNone(node, msg=filename)
            self.assertEqual(node.attrib.get("name"), expected_name,
                             msg=filename)
            self.assertEqual(
                node.attrib.get("required"), "true",
                msg="%s must terminate the complete real roslaunch when it "
                    "exits" % filename)

        real = ET.parse(ROOT / "launch" / "task1_real.launch").getroot()
        include_files = {
            include.attrib.get("file") for include in real.findall("./include")
        }
        self.assertIn(
            "$(find mine_uav_control)/launch/mission_scheduler.launch",
            include_files)
        self.assertIn(
            "$(find mine_uav_control)/launch/super_exploration_decider.launch",
            include_files)

        wrapper = (ROOT / "scripts" / "start_task1_real.sh").read_text()
        self.assertIn('wait "${ROSLAUNCH_PID}"', wrapper)
        self.assertIn('if (( launch_status == 0 )); then', wrapper)
        self.assertIn('launch_status=1', wrapper)
        self.assertIn('exit "${launch_status}"', wrapper)

if __name__ == "__main__":
    unittest.main()
