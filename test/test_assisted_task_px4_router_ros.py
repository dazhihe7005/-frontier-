#!/usr/bin/env python3

import math
import threading
import time
import unittest

import rospy
import rostest
from geometry_msgs.msg import PoseStamped, QuaternionStamped, TwistStamped
from mavros_msgs.msg import EstimatorStatus, PositionTarget, State
from mavros_msgs.srv import SetMode, SetModeResponse
from nav_msgs.msg import Odometry
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, Header, String, UInt8


class AssistedTaskRouterRosTest(unittest.TestCase):
    def setUp(self):
        self.lock = threading.Lock()
        self.mode = "POSCTL"
        self.publish_cloud = True
        self.ready = False
        self.commands = []
        self.mode_requests = []
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
            "/mine_uav/mission/shaft_enable", Bool, queue_size=1, latch=True
        )
        self.status_pub = rospy.Publisher(
            "/mine_uav/shaft/status", String, queue_size=1, latch=True
        )
        self.intent_pub = rospy.Publisher(
            "/mine_uav/shaft/velocity_intent_enu", TwistStamped, queue_size=2
        )
        self.odom_pub = rospy.Publisher(
            "/mine_uav/task1/odometry", Odometry, queue_size=2
        )
        self.cloud_pub = rospy.Publisher(
            "/mine_uav/task1/cloud_registered", PointCloud2, queue_size=2
        )
        self.heading_pub = rospy.Publisher(
            "/mine_uav/task1/vision_heading_from_planning",
            QuaternionStamped, queue_size=1, latch=True
        )
        rospy.Subscriber(
            "/mine_uav/task2/command_ready", Bool, self.ready_callback
        )
        rospy.Subscriber("/mine_uav/setpoint_cmd", PositionTarget,
                         self.command_callback)
        self.timer = rospy.Timer(rospy.Duration(0.03), self.publish_inputs)
        self.heading_pub.publish(QuaternionStamped())
        heading = QuaternionStamped()
        heading.header.stamp = rospy.Time.now()
        heading.quaternion.w = 1.0
        self.heading_pub.publish(heading)
        self.task_pub.publish(UInt8(data=2))
        self.enable_pub.publish(Bool(data=True))
        self.status_pub.publish(String(data="DESCENDING"))

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
            publish_cloud = self.publish_cloud
        now = rospy.Time.now()
        state = State(connected=True, armed=True, mode=mode)
        self.state_pub.publish(state)
        estimator = EstimatorStatus()
        estimator.header.stamp = now
        estimator.pos_horiz_rel_status_flag = True
        estimator.pos_vert_abs_status_flag = True
        self.estimator_pub.publish(estimator)
        pose = PoseStamped()
        pose.header.stamp = now
        pose.pose.position.z = 1.0
        pose.pose.orientation.w = 1.0
        self.pose_pub.publish(pose)
        odometry = Odometry()
        odometry.header.stamp = now
        odometry.header.frame_id = "task1_planning"
        odometry.pose.pose.position.z = 1.0
        odometry.pose.pose.orientation.w = 1.0
        self.odom_pub.publish(odometry)
        intent = TwistStamped()
        intent.header.stamp = now
        intent.twist.linear.z = -0.2
        self.intent_pub.publish(intent)
        if publish_cloud:
            points = [
                (2.0 * math.cos(i * math.pi / 16.0),
                 2.0 * math.sin(i * math.pi / 16.0), 1.0)
                for i in range(32)
            ]
            header = Header(stamp=now, frame_id="task1_planning")
            self.cloud_pub.publish(point_cloud2.create_cloud_xyz32(header, points))

    def wait_for(self, predicate, label, timeout=5.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not rospy.is_shutdown():
            with self.lock:
                if predicate():
                    return
            time.sleep(0.02)
        self.fail("timeout waiting for " + label)

    def test_offboard_handoff_and_cloud_loss_fallback(self):
        self.wait_for(lambda: self.ready, "OFFBOARD readiness")
        with self.lock:
            velocity_commands = [
                command for command in self.commands
                if command.type_mask & PositionTarget.IGNORE_PX
            ]
            self.assertIn("OFFBOARD", self.mode_requests)
            self.assertTrue(velocity_commands)
            command = velocity_commands[-1]
            self.assertAlmostEqual(command.velocity.z, -0.2, places=4)
            self.assertLessEqual(
                math.hypot(command.velocity.x, command.velocity.y), 0.250001
            )
            self.publish_cloud = False
        self.wait_for(lambda: not self.ready, "readiness close after cloud loss")
        self.wait_for(lambda: "POSCTL" in self.mode_requests, "POSCTL fallback")
        with self.lock:
            hold_commands = [
                command for command in self.commands
                if not (command.type_mask & PositionTarget.IGNORE_PX)
            ]
            self.assertTrue(hold_commands)


if __name__ == "__main__":
    rospy.init_node("test_assisted_task_px4_router_ros")
    rostest.rosrun(
        "mine_uav_control", "assisted_task_px4_router_ros",
        AssistedTaskRouterRosTest,
    )
