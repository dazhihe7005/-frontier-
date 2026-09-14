#!/usr/bin/env python3

"""Register a Gazebo body-frame 3D lidar cloud like Fast-LIO2 output.

The Gazebo ray sensor publishes points in ``mid360_link``.  This node applies
the current PX4/MAVROS body pose and the configured sensor mounting offset,
then publishes the registered cloud in ``camera_init`` on the same topic used
by the real Fast-LIO2/SUPER integration.  It is simulation-only.
"""

import math
import threading

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud, PointCloud2


class GazeboMid360FastlioAdapter:
    def __init__(self):
        self.world_frame = rospy.get_param("~world_frame", "camera_init")
        self.raw_cloud_topic = rospy.get_param(
            "~raw_cloud_topic", "/mine_uav/sitl/mid360/points"
        )
        self.odom_topic = rospy.get_param("~odom_topic", "/Odometry")
        self.output_topic = rospy.get_param("~output_topic", "/cloud_registered")
        self.global_topic = rospy.get_param(
            "~global_topic", "/mine_uav/sitl/global_cloud"
        )
        self.sensor_offset = rospy.get_param("~sensor_offset", [0.0, 0.0, 0.14])
        if not isinstance(self.sensor_offset, list) or len(self.sensor_offset) != 3:
            raise rospy.ROSInitException("~sensor_offset must contain [x, y, z]")
        self.sensor_offset = tuple(float(value) for value in self.sensor_offset)
        self.max_input_age = max(
            0.05, float(rospy.get_param("~max_input_age", 0.5))
        )
        self.min_range = max(0.0, float(rospy.get_param("~min_range", 0.30)))
        self.max_range = max(
            self.min_range + 0.1, float(rospy.get_param("~max_range", 29.5))
        )
        self.voxel_size = max(0.05, float(rospy.get_param("~voxel_size", 0.20)))
        self.max_global_points = max(
            1000, int(rospy.get_param("~max_global_points", 120000))
        )
        self.point_stride = max(1, int(rospy.get_param("~point_stride", 1)))

        self._lock = threading.Lock()
        self._latest_odom = None
        self._global_voxels = {}
        self._cloud_count = 0

        self.registered_pub = rospy.Publisher(
            self.output_topic, PointCloud2, queue_size=2
        )
        self.global_pub = rospy.Publisher(
            self.global_topic, PointCloud2, queue_size=1, latch=True
        )
        rospy.Subscriber(self.odom_topic, Odometry, self._odom_callback, queue_size=20)
        rospy.Subscriber(
            self.raw_cloud_topic,
            PointCloud,
            self._cloud_callback,
            queue_size=1,
            buff_size=8 * 1024 * 1024,
        )
        rospy.Timer(rospy.Duration(1.0), self._publish_global_cloud)

        rospy.logwarn(
            "Gazebo MID360 adapter active: %s -> %s in frame %s; simulation only",
            self.raw_cloud_topic,
            self.output_topic,
            self.world_frame,
        )

    def _odom_callback(self, message):
        with self._lock:
            self._latest_odom = message

    @staticmethod
    def _rotation_matrix(quaternion):
        norm = math.sqrt(
            quaternion.x * quaternion.x
            + quaternion.y * quaternion.y
            + quaternion.z * quaternion.z
            + quaternion.w * quaternion.w
        )
        if norm < 1.0e-9:
            return None
        x = quaternion.x / norm
        y = quaternion.y / norm
        z = quaternion.z / norm
        w = quaternion.w / norm
        return (
            (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)),
            (2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)),
            (2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)),
        )

    @staticmethod
    def _rotate(rotation, point):
        return (
            rotation[0][0] * point[0]
            + rotation[0][1] * point[1]
            + rotation[0][2] * point[2],
            rotation[1][0] * point[0]
            + rotation[1][1] * point[1]
            + rotation[1][2] * point[2],
            rotation[2][0] * point[0]
            + rotation[2][1] * point[1]
            + rotation[2][2] * point[2],
        )

    def _cloud_callback(self, cloud):
        with self._lock:
            odom = self._latest_odom
        if odom is None:
            rospy.logwarn_throttle(2.0, "MID360 cloud waiting for /Odometry")
            return

        now = rospy.Time.now()
        odom_age = (now - odom.header.stamp).to_sec()
        if odom_age < 0.0 or odom_age > self.max_input_age:
            rospy.logwarn_throttle(
                2.0, "MID360 cloud rejected: odometry is %.3f s old", odom_age
            )
            return

        rotation = self._rotation_matrix(odom.pose.pose.orientation)
        if rotation is None:
            rospy.logwarn_throttle(2.0, "MID360 cloud rejected: invalid attitude")
            return

        offset_world = self._rotate(rotation, self.sensor_offset)
        position = odom.pose.pose.position
        origin = (
            position.x + offset_world[0],
            position.y + offset_world[1],
            position.z + offset_world[2],
        )

        registered = []
        for index, raw_point in enumerate(cloud.points):
            if index % self.point_stride:
                continue
            point = (raw_point.x, raw_point.y, raw_point.z)
            if not all(math.isfinite(value) for value in point):
                continue
            range_squared = sum(value * value for value in point)
            if range_squared < self.min_range * self.min_range:
                continue
            if range_squared > self.max_range * self.max_range:
                continue
            rotated = self._rotate(rotation, point)
            registered.append(
                (
                    origin[0] + rotated[0],
                    origin[1] + rotated[1],
                    origin[2] + rotated[2],
                )
            )

        if not registered:
            rospy.logwarn_throttle(2.0, "Gazebo MID360 returned an empty cloud")
            return

        header = cloud.header
        header.stamp = now
        header.frame_id = self.world_frame
        self.registered_pub.publish(point_cloud2.create_cloud_xyz32(header, registered))
        self._accumulate(registered)
        self._cloud_count += 1
        rospy.loginfo_throttle(
            5.0,
            "Gazebo MID360 registered %d points; accumulated %d voxels",
            len(registered),
            len(self._global_voxels),
        )

    def _accumulate(self, points):
        inverse = 1.0 / self.voxel_size
        with self._lock:
            for point in points:
                key = tuple(int(math.floor(value * inverse)) for value in point)
                if key not in self._global_voxels:
                    self._global_voxels[key] = point
                if len(self._global_voxels) >= self.max_global_points:
                    break

    def _publish_global_cloud(self, _event):
        with self._lock:
            points = list(self._global_voxels.values())
        if not points:
            return
        header = PointCloud2().header
        header.stamp = rospy.Time.now()
        header.frame_id = self.world_frame
        self.global_pub.publish(point_cloud2.create_cloud_xyz32(header, points))


if __name__ == "__main__":
    rospy.init_node("gazebo_mid360_fastlio_adapter")
    GazeboMid360FastlioAdapter()
    rospy.spin()
