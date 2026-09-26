#!/usr/bin/env python3
"""Opt-in visual pose+velocity MAVROS ODOMETRY transport for isolated SITL.

The existing bridge validates/aligned FAST-LIO2 pose. Its PoseWithCovariance
messages are not delivered through MAVROS's vision_pose plugin in this A/B;
this node derives velocity from distinct poses and sends one ODOMETRY source.
No Gazebo truth or simulation range data enters this transport.
"""

from pathlib import Path
import math
import sys

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

sys.path.insert(0, str(Path(__file__).resolve().parent))
from vision_velocity_geometry import VelocityWindow, world_to_body


class FastlioVisionVelocityAdapter:
    def __init__(self):
        self._healthy = False
        self._window = VelocityWindow(
            max_speed=float(rospy.get_param("~max_speed", 3.5)),
            max_acceleration=float(rospy.get_param("~max_acceleration", 4.0)))
        self._velocity_stddev = max(
            0.10, float(rospy.get_param("~velocity_stddev", 0.18)))
        self._last_rx = rospy.Time(0)
        self._publisher = rospy.Publisher(
            "/mavros/odometry/out", Odometry, queue_size=10)
        rospy.Subscriber("/mine_uav/gbplanner/vision_healthy", Bool,
                         self._health_cb, queue_size=2)
        rospy.Subscriber("/mavros/vision_pose/pose_cov",
                         PoseWithCovarianceStamped, self._pose_cb,
                         queue_size=10)

    def _health_cb(self, msg):
        self._healthy = bool(msg.data)
        if not self._healthy:
            self._window.clear()

    def _pose_cb(self, msg):
        now = rospy.Time.now()
        if not self._healthy or msg.header.stamp.is_zero() or \
                (now-msg.header.stamp).to_sec() > 0.25:
            self._window.clear()
            return
        stamp = msg.header.stamp.to_sec()
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        xyz = (p.x, p.y, p.z)
        quaternion = (q.x, q.y, q.z, q.w)
        if not all(math.isfinite(value) for value in (*xyz, *quaternion)):
            self._window.clear()
            return
        velocity_world = self._window.add(stamp, xyz)
        velocity_body = None
        if velocity_world is not None:
            try:
                velocity_body = world_to_body(velocity_world, quaternion)
            except ValueError:
                self._window.clear()
        output = Odometry()
        output.header = msg.header
        output.child_frame_id = "base_link"
        output.pose = msg.pose
        # A finite-difference outlier must disable velocity fusion, not drop
        # the otherwise healthy pose and yaw. PX4's EV timeout is shorter
        # than the 0.4 s gap seen after a rejected velocity sample; dropping
        # the whole message made its local frame reset by nearly 2 m.
        if velocity_body is None:
            output.twist.twist.linear.x = math.nan
            output.twist.twist.linear.y = math.nan
            output.twist.twist.linear.z = math.nan
        else:
            output.twist.twist.linear.x = velocity_body[0]
            output.twist.twist.linear.y = velocity_body[1]
            output.twist.twist.linear.z = velocity_body[2]
        output.twist.covariance[0] = self._velocity_stddev**2
        output.twist.covariance[7] = self._velocity_stddev**2
        output.twist.covariance[14] = self._velocity_stddev**2
        # Angular rate is unavailable from the 10 Hz pose bridge. Tell the
        # receiving estimator that these zero placeholders are very uncertain.
        output.twist.covariance[21] = 100.0
        output.twist.covariance[28] = 100.0
        output.twist.covariance[35] = 100.0
        self._publisher.publish(output)


if __name__ == "__main__":
    rospy.init_node("fastlio_vision_velocity_adapter")
    FastlioVisionVelocityAdapter()
    rospy.spin()
