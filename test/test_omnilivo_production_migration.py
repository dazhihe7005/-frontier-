#!/usr/bin/env python3

"""Prevent the Task-1/Task-4 graphs from regressing to Fast-LIO."""

import pathlib
import unittest
import xml.etree.ElementTree as ET

import yaml


ROOT = pathlib.Path(__file__).resolve().parents[1]


class OmniLivoProductionMigrationTest(unittest.TestCase):
    def test_real_launch_uses_omnilivo_and_new_bridge(self):
        launch_path = ROOT / "launch" / "task1_real.launch"
        launch_text = launch_path.read_text()
        root = ET.parse(launch_path).getroot()
        includes = [node.attrib["file"] for node in root.findall(".//include")]
        nodes = [(node.attrib.get("pkg"), node.attrib.get("type"))
                 for node in root.findall(".//node")]

        self.assertTrue(any("omni_livo" in item and
                            "mapping_seeker_raw.launch" in item
                            for item in includes))
        self.assertTrue(any("seeker" in item and
                            "3undistort_nodelet.launch" in item
                            for item in includes))
        seeker_include = next(
            node for node in root.findall(".//include")
            if "seeker" in node.attrib["file"] and
            "3undistort_nodelet.launch" in node.attrib["file"])
        seeker_args = {arg.attrib["name"]: arg.attrib["value"]
                       for arg in seeker_include.findall("arg")}
        self.assertEqual(seeker_args.get("publish_tf"), "false")
        self.assertTrue(any("omnilivo_px4_vision_bridge.launch" in item
                            for item in includes))
        self.assertNotIn('pkg="fast_lio"', launch_text)
        self.assertNotIn("fastlio_px4_vision_bridge.launch", launch_text)
        self.assertIn(("mine_uav_control", "task1_perception_gate"), nodes)
        self.assertIn('/omnilivo/odometry', launch_text)
        self.assertIn('/omnilivo/cloud_registered', launch_text)
        self.assertIn(
            'name="additional_cloud_enable_topic"', launch_text)
        self.assertIn('/mine_uav/mission/task3_enable', launch_text)

    def test_production_bridge_contract_is_omnilivo(self):
        config = yaml.safe_load(
            (ROOT / "config" / "omnilivo_px4_vision_bridge.yaml").read_text())
        self.assertEqual(config["input_topic"],
                         "/omnilivo/control_odometry")
        self.assertEqual(config["output_topic"],
                         "/mavros/vision_pose/pose_cov")
        self.assertTrue(config["require_localization_health"])
        self.assertEqual(config["localization_health_topic"],
                         "/omnilivo/localization_healthy")
        self.assertAlmostEqual(config["localization_unhealthy_grace"], 0.30)
        self.assertFalse(config["allow_alignment_while_armed"])
        self.assertTrue(config["require_mavros_connection"])
        self.assertTrue(config["align_vision_heading_to_mavros"])
        self.assertEqual(config["mavros_attitude_topic"],
                         "/mavros/imu/data")
        self.assertEqual(config["vision_heading_alignment_topic"],
                         "/mine_uav/task1/vision_heading_from_planning")
        self.assertTrue(config["auto_calibrate_mount_from_gravity"])
        self.assertEqual(config["max_input_angular_speed"], 2.0)
        self.assertGreater(config["max_physical_angular_speed"],
                           config["max_input_angular_speed"])
        self.assertEqual(config["max_input_orientation_jump"], 0.75)
        self.assertGreaterEqual(config["jump_recovery_stable_samples"], 3)
        self.assertEqual(config["auto_enable_topic"],
                         "/mine_uav/mission/auto_enable")
        self.assertEqual(config["sensor_imu_topic"], "/livox/imu")
        self.assertEqual(config["body_imu_topic"],
                         "/mavros/imu/data_raw")
        self.assertNotIn("temporary_sensor_pitch_in_body_deg", config)
        launch_text = (ROOT / "launch" / "task1_real.launch").read_text()
        self.assertIn('name="require_vision_heading_alignment" value="true"',
                      launch_text)

    def test_real_wrapper_sources_omni_not_fastlio(self):
        wrapper = (ROOT / "scripts" / "start_task1_real.sh").read_text()
        self.assertIn("omni_livo_fisheye_ws/devel/setup.bash", wrapper)
        self.assertIn("seeker_ws/devel/setup.bash", wrapper)
        self.assertIn("make_raw_config.py", wrapper)
        self.assertIn("sync_control.py", wrapper)
        self.assertNotIn("fastlio2_ws/devel/setup.bash", wrapper)
        self.assertIn("omnilivo_runtime_config", wrapper)
        self.assertIn("SEEKER_USB_VENDOR=2207", wrapper)
        self.assertIn("SEEKER_USB_PRODUCT=0000", wrapper)
        self.assertIn("seeker_usb_present", wrapper)
        self.assertIn("TASK3_USB_VENDOR=1a86", wrapper)
        self.assertIn("task3_ch340_present", wrapper)
        self.assertIn("exactly one Task-3 CH340 USB UART", wrapper)
        self.assertIn("/sys/bus/usb/devices", wrapper)

        generator = pathlib.Path(
            "/home/nuc/omni_livo_fisheye_ws/tools/make_raw_config.py"
        ).read_text()
        self.assertIn("image_subscriber_queue=1", generator)
        self.assertIn("max_image_buffer_groups=3", generator)
        self.assertIn("camera_sync_tolerance_sec=0.001", generator)
        self.assertIn("hardware_time_sync_verified=hardware_time_sync_verified", generator)
        self.assertIn("path_rate_hz=1.0", generator)
        self.assertIn("path_max_poses=1000", generator)
        self.assertIn("imu_rate_odom=True", generator)
        self.assertIn("max_imu_propagation_horizon=0.90", generator)
        self.assertIn("max_optimized_anchor_update_age=0.30", generator)
        self.assertIn("max_output_state_age=0.75", generator)
        self.assertIn("max_optimized_relock_position_jump=0.5", generator)
        self.assertIn("max_optimized_relock_orientation_jump=0.5", generator)
        self.assertIn("imu_propagated_odometry_topic='/omnilivo/control_odometry'",
                      generator)

    def test_all_production_pose_consumers_use_omnilivo_contract(self):
        launch_text = (ROOT / "launch" / "task1_real.launch").read_text()
        bridge = yaml.safe_load(
            (ROOT / "config" / "omnilivo_px4_vision_bridge.yaml").read_text())
        scheduler = yaml.safe_load(
            (ROOT / "config" / "mission_scheduler.yaml").read_text())
        decider = yaml.safe_load(
            (ROOT / "config" / "super_exploration_decider.yaml").read_text())
        planner = yaml.safe_load(
            (ROOT / "config" / "super_task1.yaml").read_text())
        command_bridge = yaml.safe_load(
            (ROOT / "config" / "super_px4_command_bridge.yaml").read_text())

        self.assertNotIn('/Odometry', launch_text)
        self.assertNotIn('/cloud_registered"', launch_text.replace(
            '/omnilivo/cloud_registered"', '').replace(
            '/mine_uav/task1/cloud_registered"', ''))
        self.assertEqual(bridge["input_topic"],
                         "/omnilivo/control_odometry")
        self.assertEqual(scheduler["odometry_topic"],
                         "/mine_uav/task1/odometry")
        self.assertEqual(decider["odom_topic"],
                         "/mine_uav/task1/odometry")
        self.assertEqual(decider["cloud_topic"],
                         "/mine_uav/task1/cloud_registered")
        self.assertEqual(planner["rog_map"]["ros_callback"]["odom_topic"],
                         "/mine_uav/task1/odometry")
        self.assertEqual(planner["rog_map"]["ros_callback"]["cloud_topic"],
                         "/mine_uav/task1/cloud_registered")
        self.assertEqual(command_bridge["task_odometry_topic"],
                         "/mine_uav/task1/odometry")
        self.assertEqual(command_bridge["obstacle_cloud_topic"],
                         "/mine_uav/task1/cloud_registered")

    def test_task1_sitl_uses_omnilivo_and_health_gated_bridge(self):
        launch_path = ROOT / "launch" / "task1_px4_sitl.launch"
        launch_text = launch_path.read_text()
        root = ET.parse(launch_path).getroot()
        nodes = {(node.attrib.get("pkg"), node.attrib.get("type"))
                 for node in root.findall(".//node")}

        self.assertIn(("omni_livo", "fastlivo_mapping"), nodes)
        self.assertIn(("mine_uav_control", "pointcloud1_to_pointcloud2"), nodes)
        self.assertIn(("mine_uav_control", "lio_px4_vision_bridge"), nodes)
        self.assertIn("/omnilivo/odometry", launch_text)
        self.assertIn("/omnilivo/localization_healthy", launch_text)
        self.assertIn("require_localization_health", launch_text)
        self.assertNotIn('pkg="fast_lio"', launch_text)
        self.assertNotIn("fastlio_px4_vision_bridge", launch_text)
        self.assertNotIn("/Odometry", launch_text)

        config = yaml.safe_load(
            (ROOT / "config" / "omnilivo_gazebo_mid360.yaml").read_text())
        self.assertEqual(config["common"]["img_en"], 0)
        self.assertEqual(config["common"]["lid_topic"],
                         "/mine_uav/sitl/mid360/points2")
        self.assertEqual(config["preprocess"]["lidar_type"], 4)
        self.assertEqual(config["common"]["imu_topic"],
                         "/mavros/imu/data_raw")
        self.assertEqual(config["publish"]["odometry_topic"],
                         "/omnilivo/odometry")
        self.assertEqual(config["health"]["min_translation_eigenvalue_ratio"],
                         0.05)
        self.assertEqual(
            config["health"]["min_translation_eigenvalue_ratio_release"],
            0.03)

    def test_dedicated_omnilivo_regression_defaults_to_no_arm(self):
        launch_path = (ROOT / "launch" /
                       "task1_omnilivo_regression_sitl.launch")
        root = ET.parse(launch_path).getroot()
        args = {arg.attrib["name"]: arg.attrib.get("default")
                for arg in root.findall("./arg")}
        self.assertEqual(args["allow_auto_arm"], "false")
        include = root.find("./include")
        self.assertIsNotNone(include)
        self.assertIn("task1_40m_platform_sitl.launch",
                      include.attrib["file"])

    def test_task4_launcher_uses_omnilivo_environment(self):
        wrapper = (ROOT / "scripts" / "start_baixiangshan_sitl.sh").read_text()
        self.assertIn("omni_livo_fisheye_ws/devel/setup.bash", wrapper)
        self.assertIn("omni_livo", wrapper)
        self.assertNotIn("fastlio2_ws", wrapper)
        self.assertNotIn("fast_lio", wrapper)

        task4 = ET.parse(
            ROOT / "launch" / "task4_baixiangshan_px4_sitl.launch"
        ).getroot()
        legacy_wrapper = ET.parse(
            ROOT / "launch" / "task1_baixiangshan_px4_sitl.launch"
        ).getroot()
        self.assertIn("task1_baixiangshan_px4_sitl.launch",
                      task4.find("./include").attrib["file"])
        self.assertIn("task1_px4_sitl.launch",
                      legacy_wrapper.find("./include").attrib["file"])


if __name__ == "__main__":
    unittest.main()
