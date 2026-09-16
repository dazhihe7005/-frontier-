#!/usr/bin/env python3

"""Expose PX4 SITL odometry through the Fast-LIO2 task-one contract.

This node only adapts localization and publishes visualization/health data. It
does not emulate RC input, arm PX4, change flight mode, or generate lidar data.
"""

import math
import threading

import rospy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool


class SitlLocalizationAdapter:
    def __init__(self):
        self.world_frame = rospy.get_param("~world_frame", "camera_init")
        self.input_topic = rospy.get_param(
            "~input_odom_topic", "/mavros/local_position/odom"
        )
        self.output_topic = rospy.get_param("~output_odom_topic", "/Odometry")
        self.publish_rate = max(5.0, float(rospy.get_param("~publish_rate", 20.0)))
        self._lock = threading.Lock()
        self._latest_odom = None
        self._path = Path()
        self._path.header.frame_id = self.world_frame

        self.odom_pub = rospy.Publisher(self.output_topic, Odometry, queue_size=20)
        self.path_pub = rospy.Publisher(
            "/mine_uav/sitl/px4_path", Path, queue_size=1, latch=True
        )
        self.alignment_pub = rospy.Publisher(
            "/mine_uav/task1/fastlio_to_px4_alignment",
            TransformStamped,
            queue_size=1,
            latch=True,
        )
        self.vision_pub = rospy.Publisher(
            "/mine_uav/task1/vision_healthy", Bool, queue_size=1, latch=True
        )
        rospy.Subscriber(self.input_topic, Odometry, self._odom_callback, queue_size=20)
        rospy.Timer(rospy.Duration(1.0 / self.publish_rate), self._publish)
        rospy.Timer(rospy.Duration(1.0), self._publish_health)
        self._publish_health(None)
        rospy.logwarn(
            "SITL localization adapter active: %s -> %s; simulation only",
            self.input_topic,
            self.output_topic,
        )

    def _odom_callback(self, message):
        converted = Odometry()
        converted.header.frame_id = self.world_frame
        converted.child_frame_id = "base_link"
        converted.pose = message.pose
        converted.twist = message.twist
        with self._lock:
            self._latest_odom = converted

    def _publish(self, _event):
        with self._lock:
            odom = self._latest_odom
        if odom is None:
            return
        stamp = rospy.Time.now()
        odom.header.stamp = stamp
        self.odom_pub.publish(odom)
        pose = odom.pose.pose
        if not self._path.poses or self._distance_from_last(pose) >= 0.15:
            stamped = PoseStamped()
            stamped.header = odom.header
            stamped.pose = pose
            self._path.poses.append(stamped)
            if len(self._path.poses) > 5000:
                self._path.poses = self._path.poses[-5000:]
        self._path.header.stamp = stamp
        self.path_pub.publish(self._path)

    def _distance_from_last(self, pose):
        previous = self._path.poses[-1].pose.position
        current = pose.position
        return math.sqrt(
            (current.x - previous.x) ** 2
            + (current.y - previous.y) ** 2
            + (current.z - previous.z) ** 2
        )

    def _publish_health(self, _event):
        stamp = rospy.Time.now()
        alignment = TransformStamped()
        alignment.header.stamp = stamp
        alignment.header.frame_id = self.world_frame
        alignment.child_frame_id = "map"
        alignment.transform.rotation.w = 1.0
        self.alignment_pub.publish(alignment)
        self.vision_pub.publish(Bool(data=self._latest_odom is not None))


if __name__ == "__main__":
    rospy.init_node("sitl_localization_adapter")
    SitlLocalizationAdapter()
    rospy.spin()
