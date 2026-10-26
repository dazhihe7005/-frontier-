#!/usr/bin/env python3

import math
import threading
import time
import unittest

import rospy
import rostest
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import EstimatorStatus, PositionTarget, State
from mavros_msgs.srv import SetMode, SetModeResponse
from std_msgs.msg import Bool, UInt8


class AssistedTask3RouterRosTest(unittest.TestCase):
    def setUp(self):
        self.lock = threading.Lock()
        self.mode = "POSCTL"
        self.publish_controller = True
        self.ready = False
        self.commands = []
        self.mode_requests = []
        self.last_navigation_summary = 0.0
        self.service = rospy.Service("/mavros/set_mode", SetMode, self.set_mode)
        self.state_pub = rospy.Publisher("/mavros/state", State, queue_size=2)
        self.estimator_pub = rospy.Publisher(
            "/mavros/estimator_status", EstimatorStatus, queue_size=2
        )
        self.pose_pub = rospy.Publisher(
            "/mavros/local_position/pose", PoseStamped, queue_size=2
        )
        self.task_pub = rospy.Publisher(
            "/mine_uav/mission/active_task", UInt8, queue_size=1, latch=True
        )
        self.enable_pub = rospy.Publisher(
            "/mine_uav/mission/task3_enable", Bool, queue_size=1, latch=True
        )
        self.controller_ready_pub = rospy.Publisher(
            "/mine_uav/task3/controller_ready", Bool, queue_size=1, latch=True
        )
        self.private_command_pub = rospy.Publisher(
            "/mine_uav/task3/setpoint_cmd", PositionTarget, queue_size=2
        )
        rospy.Subscriber(
            "/mine_uav/task3/command_ready", Bool, self.ready_callback
        )
        rospy.Subscriber(
            "/mine_uav/setpoint_cmd", PositionTarget, self.command_callback
        )
        self.timer = rospy.Timer(rospy.Duration(0.03), self.publish_inputs)
        self.task_pub.publish(UInt8(data=3))
        self.enable_pub.publish(Bool(data=True))

    def tearDown(self):
        self.timer.shutdown()
        self.enable_pub.publish(Bool(data=False))
        self.service.shutdown("test complete")

    def set_mode(self, request):
        with self.lock:
            self.mode_requests.append(request.custom_mode)
            self.mode = request.custom_mode
        return SetModeResponse(mode_sent=True)

    def ready_callback(self, message):
        with self.lock:
            self.ready = message.data

    def command_callback(self, message):
        with self.lock:
            self.commands.append(message)

    def publish_inputs(self, _event):
        with self.lock:
            mode = self.mode
            publish_controller = self.publish_controller
        now = rospy.Time.now()
        monotonic_now = time.monotonic()
        # Reproduce the production MAVROS cadence. State and EstimatorStatus
        # are ~1 Hz while pose and controller commands remain high-rate.
        if monotonic_now - self.last_navigation_summary >= 0.95:
            self.last_navigation_summary = monotonic_now
            self.state_pub.publish(State(connected=True, armed=True, mode=mode))
            estimator = EstimatorStatus()
            estimator.header.stamp = now
            estimator.pos_horiz_rel_status_flag = True
            estimator.pos_vert_abs_status_flag = True
            self.estimator_pub.publish(estimator)
        pose = PoseStamped()
        pose.header.stamp = now
        pose.pose.position.x = 4.0
        pose.pose.position.y = -2.0
        pose.pose.position.z = 1.5
        pose.pose.orientation.z = math.sin(math.pi / 4.0)
        pose.pose.orientation.w = math.cos(math.pi / 4.0)
        self.pose_pub.publish(pose)
        if publish_controller:
            self.controller_ready_pub.publish(Bool(data=True))
            command = PositionTarget()
            command.header.stamp = now
            command.coordinate_frame = PositionTarget.FRAME_BODY_NED
            command.type_mask = (
                PositionTarget.IGNORE_PX
                | PositionTarget.IGNORE_PY
                | PositionTarget.IGNORE_PZ
                | PositionTarget.IGNORE_AFX
                | PositionTarget.IGNORE_AFY
                | PositionTarget.IGNORE_AFZ
                | PositionTarget.IGNORE_YAW
                | PositionTarget.IGNORE_YAW_RATE
            )
            command.velocity.x = 0.2
            command.velocity.y = 0.1
            self.private_command_pub.publish(command)

    def wait_for(self, predicate, label, timeout=5.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not rospy.is_shutdown():
            with self.lock:
                if predicate():
                    return
            time.sleep(0.02)
        self.fail("timeout waiting for " + label)

    def test_offboard_handoff_transform_and_input_loss_fallback(self):
        self.wait_for(lambda: self.ready, "Task-3 OFFBOARD readiness")
        # More than two real 1 Hz summary periods must pass without the router
        # spuriously requesting POSCTL or revoking Task-3 readiness.
        time.sleep(2.2)
        with self.lock:
            self.assertTrue(self.ready)
            self.assertNotIn("POSCTL", self.mode_requests)
            active = [
                command for command in self.commands
                if command.type_mask & PositionTarget.IGNORE_PX
            ]
            self.assertIn("OFFBOARD", self.mode_requests)
            self.assertTrue(active)
            command = active[-1]
            self.assertAlmostEqual(command.velocity.x, 0.1, places=3)
            self.assertAlmostEqual(command.velocity.y, 0.2, places=3)
            self.assertAlmostEqual(command.position.z, 1.5, places=3)
            self.assertAlmostEqual(command.yaw, math.pi / 2.0, places=3)
            self.publish_controller = False
        self.wait_for(lambda: not self.ready, "readiness close after input loss")
        self.wait_for(lambda: "POSCTL" in self.mode_requests, "POSCTL fallback")
        with self.lock:
            holds = [
                command for command in self.commands
                if not (command.type_mask & PositionTarget.IGNORE_PX)
            ]
            self.assertTrue(holds)
            self.assertAlmostEqual(holds[-1].position.x, 4.0, places=3)
            self.assertAlmostEqual(holds[-1].position.y, -2.0, places=3)
            self.assertAlmostEqual(holds[-1].position.z, 1.5, places=3)


if __name__ == "__main__":
    rospy.init_node("test_assisted_task3_px4_router_ros")
    rostest.rosrun(
        "mine_uav_control",
        "assisted_task3_px4_router_ros",
        AssistedTask3RouterRosTest,
    )
