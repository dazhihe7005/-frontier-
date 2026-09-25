#!/usr/bin/env python3
"""Prepare MID360 rays and FAST-LIO2 poses for Voxblox.

Obstacle and no-return endpoints stay in the lidar frame.  A timestamped TF
derived only from FAST-LIO2 odometry supplies the moving ray origin, avoiding
the common (and unsafe) error of integrating registered points from the map
origin.  Gazebo vehicle pose is never subscribed.
"""

import math
import message_filters
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud, PointCloud2


def quaternion_multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw*bx + ax*bw + ay*bz - az*by,
        aw*by - ax*bz + ay*bw + az*bx,
        aw*bz + ax*by - ay*bx + az*bw,
        aw*bw - ax*bx - ay*by - az*bz,
    )


def rotate(q, p):
    x, y, z, w = q
    px, py, pz = p
    # Unit-quaternion vector rotation.
    tx, ty, tz = (2*(y*pz-z*py), 2*(z*px-x*pz), 2*(x*py-y*px))
    return (px+w*tx+y*tz-z*ty, py+w*ty+z*tx-x*tz,
            pz+w*tz+x*ty-y*tx)


class FastlioVoxbloxAdapter:
    def __init__(self):
        self.world_frame = rospy.get_param("~world_frame", "camera_init")
        self.sensor_frame = rospy.get_param("~sensor_frame", "gbplanner_mid360")
        self.sensor_offset = tuple(float(v) for v in rospy.get_param(
            "~sensor_offset", [0.1315, 0.0, 0.203]))
        sensor_pitch = float(rospy.get_param("~sensor_pitch", 0.436332313))
        self.sensor_q = (0.0, math.sin(0.5*sensor_pitch), 0.0,
                         math.cos(0.5*sensor_pitch))
        self.min_range = float(rospy.get_param("~min_range", 0.30))
        self.max_range = float(rospy.get_param("~max_range", 29.5))
        self.free_stride = max(1, int(rospy.get_param("~free_stride", 4)))
        self.self_radius = float(rospy.get_param("~self_filter_radius", 0.65))
        self.self_z_min = float(rospy.get_param("~self_filter_z_min", -0.32))
        self.self_z_max = float(rospy.get_param("~self_filter_z_max", 0.25))
        self.sync_queue = max(5, int(rospy.get_param("~sync_queue", 30)))
        self.sync_slop = max(0.001, float(rospy.get_param("~sync_slop", 0.025)))
        self._tf = tf2_ros.TransformBroadcaster()
        self._obstacle_pub = rospy.Publisher(
            "/mine_uav/gbplanner/voxblox_points", PointCloud2, queue_size=2)
        self._free_pub = rospy.Publisher(
            "/mine_uav/gbplanner/voxblox_freespace", PointCloud2, queue_size=2)
        # FAST-LIO publishes odometry after processing the corresponding scan.
        # Using the latest odometry in the cloud callback therefore applies the
        # preceding scan pose (measured as a consistent 0.10 s lag). Hold the
        # cloud until its timestamp-matched odometry arrives instead.
        cloud_subscriber = message_filters.Subscriber(
            "/mine_uav/sitl/mid360/points", PointCloud,
            queue_size=self.sync_queue, buff_size=8*1024*1024)
        odometry_subscriber = message_filters.Subscriber(
            "/mine_uav/gbplanner/planner_odometry", Odometry,
            queue_size=self.sync_queue)
        self._synchronizer = message_filters.ApproximateTimeSynchronizer(
            [cloud_subscriber, odometry_subscriber], self.sync_queue,
            self.sync_slop, allow_headerless=False)
        self._synchronizer.registerCallback(self._cloud_cb)
        rospy.loginfo("Voxblox scan/odometry synchronization: queue=%d, slop=%.3f s",
                      self.sync_queue, self.sync_slop)

    def _cloud_cb(self, message, odom):
        stamp_delta = (odom.header.stamp-message.header.stamp).to_sec()
        body_q = (odom.pose.pose.orientation.x, odom.pose.pose.orientation.y,
                  odom.pose.pose.orientation.z, odom.pose.pose.orientation.w)
        norm = math.sqrt(sum(v*v for v in body_q))
        if norm < 1.0e-8 or not math.isfinite(norm):
            return
        body_q = tuple(v/norm for v in body_q)
        sensor_offset_world = rotate(body_q, self.sensor_offset)
        sensor_q_world = quaternion_multiply(body_q, self.sensor_q)
        transform = TransformStamped()
        transform.header.stamp = message.header.stamp
        transform.header.frame_id = self.world_frame
        transform.child_frame_id = self.sensor_frame
        transform.transform.translation.x = odom.pose.pose.position.x + sensor_offset_world[0]
        transform.transform.translation.y = odom.pose.pose.position.y + sensor_offset_world[1]
        transform.transform.translation.z = odom.pose.pose.position.z + sensor_offset_world[2]
        (transform.transform.rotation.x, transform.transform.rotation.y,
         transform.transform.rotation.z, transform.transform.rotation.w) = sensor_q_world
        self._tf.sendTransform(transform)

        obstacles, freespace = [], []
        min_sq, max_sq = self.min_range**2, self.max_range**2
        for index, point in enumerate(message.points):
            p = (float(point.x), float(point.y), float(point.z))
            if not all(math.isfinite(v) for v in p):
                continue
            range_sq = sum(v*v for v in p)
            if range_sq < min_sq:
                continue
            if range_sq >= max_sq:
                if index % self.free_stride == 0:
                    scale = self.max_range / math.sqrt(range_sq)
                    freespace.append(tuple(v*scale for v in p))
                continue
            body_point_rotated = rotate(self.sensor_q, p)
            body_point = tuple(self.sensor_offset[i]+body_point_rotated[i]
                               for i in range(3))
            if (body_point[0]**2 + body_point[1]**2 <= self.self_radius**2 and
                    self.self_z_min <= body_point[2] <= self.self_z_max):
                continue
            obstacles.append(p)
        header = message.header
        header.frame_id = self.sensor_frame
        if obstacles:
            self._obstacle_pub.publish(point_cloud2.create_cloud_xyz32(header, obstacles))
        if freespace:
            self._free_pub.publish(point_cloud2.create_cloud_xyz32(header, freespace))
        rospy.loginfo_throttle(
            5.0, "Voxblox synchronized input: %d obstacles, %d free rays, dt=%+.4f s",
            len(obstacles), len(freespace), stamp_delta)


if __name__ == "__main__":
    rospy.init_node("fastlio_voxblox_adapter")
    FastlioVoxbloxAdapter()
    rospy.spin()
