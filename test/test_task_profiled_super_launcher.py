#!/usr/bin/env python3
"""The shared real launch must select a complete SUPER profile per task."""

import pathlib
import sys
import unittest
from unittest import mock

import rospy
from std_msgs.msg import UInt8

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "scripts"))
from task_profiled_super_launcher import ProfiledSuperLauncher  # noqa: E402


class FakeChild:
    next_pid = 3000

    def __init__(self, command, **kwargs):
        self.command = command
        self.pid = FakeChild.next_pid
        FakeChild.next_pid += 1
        self.returncode = None

    def poll(self):
        return self.returncode

    def wait(self, timeout=None):
        self.returncode = 0
        return 0


class ProfiledSuperLauncherTest(unittest.TestCase):
    def test_task_switch_restarts_with_isolated_planner_config(self):
        paths = {
            "~goaf_config_path": "/tmp/task1.yaml",
            "~tunnel_config_path": "/tmp/task4.yaml",
        }
        children = []

        def launch(command, **kwargs):
            child = FakeChild(command, **kwargs)
            children.append(child)
            return child

        def fake_sleep(_):
            launcher._active_task_callback(UInt8(data=4))

        with mock.patch.object(rospy, "get_param", side_effect=lambda key, default=None: paths.get(key, default)), \
             mock.patch.object(rospy, "Subscriber"), \
             mock.patch.object(rospy, "on_shutdown"), \
             mock.patch.object(rospy, "is_shutdown", side_effect=[False, False, True]), \
             mock.patch.object(rospy, "loginfo"), \
             mock.patch.object(rospy, "logwarn"), \
             mock.patch("task_profiled_super_launcher.os.path.isfile", return_value=True), \
             mock.patch("task_profiled_super_launcher.subprocess.Popen", side_effect=launch), \
             mock.patch("task_profiled_super_launcher.os.killpg") as killpg, \
             mock.patch("task_profiled_super_launcher.time.sleep", side_effect=fake_sleep):
            launcher = ProfiledSuperLauncher()
            self.assertEqual(launcher.run(), 0)
            self.assertEqual(len(children), 2)
            self.assertEqual(children[0].command[-1], "_config_path:=/tmp/task1.yaml")
            self.assertEqual(children[1].command[-1], "_config_path:=/tmp/task4.yaml")
            killpg.assert_called_once_with(children[0].pid, 15)
            launcher._stop()


if __name__ == "__main__":
    unittest.main()
