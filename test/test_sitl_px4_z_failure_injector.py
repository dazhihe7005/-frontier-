#!/usr/bin/env python3

"""The destructive sensor-failure probe must refuse non-isolated masters."""

import importlib.util
import pathlib
import unittest
from unittest.mock import MagicMock, patch


MODULE_PATH = pathlib.Path(__file__).resolve().parents[1] / "scripts" / "sitl_shaft_px4_z_failure_injector.py"
SPEC = importlib.util.spec_from_file_location("sitl_shaft_px4_z_failure_injector", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class SitlPx4ZFailureInjectorSafetyTest(unittest.TestCase):
    def setUp(self):
        self.injector = object.__new__(MODULE.SitlPx4ZFailureInjector)
        self.injector.expected_fcu_url = self.injector.FCU_URL
        self.injector.allowed_master_port = 11319

    def test_real_master_is_rejected_even_with_sim_time_and_udp_url(self):
        with patch.dict(MODULE.os.environ, {"ROS_MASTER_URI": "http://localhost:11312"}), \
                patch.object(MODULE.rospy, "get_param",
                             side_effect=lambda key, default: True if key == "/use_sim_time" else self.injector.FCU_URL):
            self.assertFalse(self.injector.sitl_only())

    def test_wrong_fcu_url_is_rejected(self):
        with patch.dict(MODULE.os.environ, {"ROS_MASTER_URI": "http://localhost:11319"}), \
                patch.object(MODULE.rospy, "get_param",
                             side_effect=lambda key, default: True if key == "/use_sim_time" else "serial:///dev/ttyUSB0:500000"):
            self.assertFalse(self.injector.sitl_only())

    def test_wall_time_master_is_rejected(self):
        with patch.dict(MODULE.os.environ, {"ROS_MASTER_URI": "http://localhost:11319"}), \
                patch.object(MODULE.rospy, "get_param",
                             side_effect=lambda key, default: False if key == "/use_sim_time" else self.injector.FCU_URL):
            self.assertFalse(self.injector.sitl_only())

    def test_only_isolated_local_udp_sitl_is_accepted(self):
        with patch.dict(MODULE.os.environ, {"ROS_MASTER_URI": "http://localhost:11319"}), \
                patch.object(MODULE.rospy, "get_param",
                             side_effect=lambda key, default: True if key == "/use_sim_time" else self.injector.FCU_URL):
            self.assertTrue(self.injector.sitl_only())

    def test_explicit_isolated_master_port_is_accepted(self):
        self.injector.allowed_master_port = 11335
        with patch.dict(MODULE.os.environ, {"ROS_MASTER_URI": "http://localhost:11335"}), \
                patch.object(MODULE.rospy, "get_param",
                             side_effect=lambda key, default: True if key == "/use_sim_time" else self.injector.FCU_URL):
            self.assertTrue(self.injector.sitl_only())

    def test_injected_fault_disables_permission_for_next_sitl_run(self):
        self.injector.depth = 6.1
        self.injector.injected = False
        self.injector.terminal = False
        self.injector.status_pub = MagicMock()
        self.injector.command = MagicMock(return_value=MagicMock(
            success=True, result=0))
        self.injector.set = MagicMock(return_value=MagicMock(success=True))
        self.injector.get = MagicMock(side_effect=[
            MagicMock(success=True, value=MagicMock(integer=1)),
            MagicMock(success=True, value=MagicMock(integer=0)),
        ])
        self.injector.inject()
        self.assertTrue(self.injector.injected)
        self.assertEqual(self.injector.command.call_count, 2)
        self.assertEqual(self.injector.set.call_args.kwargs["param_id"],
                         "SYS_FAILURE_EN")
        self.assertEqual(self.injector.set.call_args.kwargs["value"].integer, 0)
        self.assertEqual(self.injector.status_pub.publish.call_args.args[0].data,
                         "INJECTED_BARO_GPS_OFF_PERMISSION_DISABLED")

    def test_preparation_does_not_enable_failure_permission(self):
        self.injector.prepared = False
        self.injector.status_pub = MagicMock()
        self.injector.pull = MagicMock(return_value=MagicMock(success=True))
        self.injector.get = MagicMock(return_value=MagicMock(
            success=True, value=MagicMock(integer=0)))
        self.injector.set = MagicMock()
        self.injector.setup()
        self.assertTrue(self.injector.prepared)
        self.injector.set.assert_not_called()
        self.assertEqual(self.injector.status_pub.publish.call_args.args[0].data,
                         "SITL_FAILURE_INJECTION_PREPARED")

    def test_rejected_failure_command_still_disables_permission(self):
        self.injector.depth = 6.1
        self.injector.injected = False
        self.injector.terminal = False
        self.injector.status_pub = MagicMock()
        self.injector.command = MagicMock(return_value=MagicMock(
            success=False, result=1))
        self.injector.set = MagicMock(return_value=MagicMock(success=True))
        self.injector.get = MagicMock(side_effect=[
            MagicMock(success=True, value=MagicMock(integer=1)),
            MagicMock(success=True, value=MagicMock(integer=0)),
        ])
        self.injector.inject()
        self.assertFalse(self.injector.injected)
        self.assertEqual(self.injector.set.call_count, 2)
        self.assertEqual(self.injector.status_pub.publish.call_args.args[0].data,
                         "FAILURE_ABORTED_PERMISSION_DISABLED")


if __name__ == "__main__":
    unittest.main()
