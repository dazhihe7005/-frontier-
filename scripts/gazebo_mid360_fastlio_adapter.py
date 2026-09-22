#!/usr/bin/env python3

"""Register a Gazebo body-frame 3D lidar cloud like Fast-LIO2 output.

The Gazebo ray sensor publishes points in ``mid360_link``.  This node applies
the current PX4/MAVROS body pose and the configured sensor mounting offset,
then publishes the registered cloud in ``camera_init`` on the same topic used
by the real Fast-LIO2/SUPER integration.  It is simulation-only.
"""

import copy
import math
import threading
import xml.etree.ElementTree as ET
from collections import deque

import rospy
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import PoseStamped
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
        self.px4_pose_topic = rospy.get_param("~px4_pose_topic", "")
        self.output_odom_topic = rospy.get_param("~output_odom_topic", "")
        self.max_height_age = max(
            0.05, float(rospy.get_param("~max_height_age", 0.5))
        )
        self.global_topic = rospy.get_param(
            "~global_topic", "/mine_uav/sitl/global_cloud"
        )
        self.free_ray_topic = rospy.get_param("~free_ray_topic", "")
        self.world_geometry_file = rospy.get_param("~world_geometry_file", "")
        self.surface_tolerance = max(
            0.05, float(rospy.get_param("~surface_tolerance", 0.35))
        )
        self._world_boxes = self._load_static_boxes(self.world_geometry_file)
        self._link_history = deque(maxlen=100)
        self.sensor_offset = rospy.get_param(
            "~sensor_offset", [0.1315, 0.0, 0.223]
        )
        if not isinstance(self.sensor_offset, list) or len(self.sensor_offset) != 3:
            raise rospy.ROSInitException("~sensor_offset must contain [x, y, z]")
        self.sensor_offset = tuple(float(value) for value in self.sensor_offset)
        self.sensor_rpy = rospy.get_param(
            "~sensor_rpy", [0.0, 0.436332313, 0.0]
        )
        if not isinstance(self.sensor_rpy, list) or len(self.sensor_rpy) != 3:
            raise rospy.ROSInitException("~sensor_rpy must contain [roll, pitch, yaw]")
        self.sensor_rotation = self._rpy_matrix(
            *(float(value) for value in self.sensor_rpy)
        )
        self.max_input_age = max(
            0.05, float(rospy.get_param("~max_input_age", 0.5))
        )
        self.future_odom_tolerance = max(
            0.0, float(rospy.get_param("~future_odom_tolerance", 0.05))
        )
        self.min_range = max(0.0, float(rospy.get_param("~min_range", 0.30)))
        self.max_range = max(
            self.min_range + 0.1, float(rospy.get_param("~max_range", 29.5))
        )
        self.self_filter_enable = bool(
            rospy.get_param("~self_filter_enable", True)
        )
        self.self_filter_xy_radius = max(
            0.0, float(rospy.get_param("~self_filter_xy_radius", 1.00))
        )
        self.self_filter_z_min = float(
            rospy.get_param("~self_filter_z_min", -0.65)
        )
        self.self_filter_z_max = float(
            rospy.get_param("~self_filter_z_max", 0.25)
        )
        self.voxel_size = max(0.05, float(rospy.get_param("~voxel_size", 0.20)))
        self.max_global_points = max(
            1000, int(rospy.get_param("~max_global_points", 120000))
        )
        self.point_stride = max(1, int(rospy.get_param("~point_stride", 1)))

        self._lock = threading.Lock()
        self._latest_odom = None
        self._latest_px4_pose = None
        self._height_alignment = None
        self._global_voxels = {}
        self._cloud_count = 0

        self.registered_pub = rospy.Publisher(
            self.output_topic, PointCloud2, queue_size=2
        )
        self.planner_odom_pub = (
            rospy.Publisher(self.output_odom_topic, Odometry, queue_size=20)
            if self.output_odom_topic else None
        )
        self.free_ray_pub = (
            rospy.Publisher(self.free_ray_topic, PointCloud2, queue_size=2)
            if self.free_ray_topic else None
        )
        self.global_pub = rospy.Publisher(
            self.global_topic, PointCloud2, queue_size=1, latch=True
        )
        rospy.Subscriber(self.odom_topic, Odometry, self._odom_callback, queue_size=20)
        if self.px4_pose_topic:
            rospy.Subscriber(
                self.px4_pose_topic,
                PoseStamped,
                self._px4_pose_callback,
                queue_size=20,
            )
        if self._world_boxes:
            rospy.Subscriber(
                "/gazebo/link_states", LinkStates, self._links_callback,
                queue_size=20,
            )
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

    def _px4_pose_callback(self, message):
        with self._lock:
            self._latest_px4_pose = message

    def _links_callback(self, message):
        try:
            sensor_index = message.name.index("iris::mid360_link")
            base_index = message.name.index("iris::iris::base_link")
        except ValueError:
            return
        with self._lock:
            self._link_history.append(
                (rospy.Time.now(), message.pose[sensor_index],
                 message.pose[base_index])
            )

    @staticmethod
    def _load_static_boxes(world_file):
        if not world_file:
            return []
        try:
            root = ET.parse(world_file).getroot()
        except (OSError, ET.ParseError) as error:
            raise rospy.ROSInitException("cannot read Gazebo world geometry: %s" % error)
        boxes = []
        for model in root.findall(".//world/model"):
            if model.findtext("static", "false").strip().lower() != "true":
                continue
            model_pose = GazeboMid360FastlioAdapter._box_pose(model.findtext("pose"))
            for link in model.findall("link"):
                link_pose = GazeboMid360FastlioAdapter._box_pose(link.findtext("pose"))
                for collision in link.findall("collision"):
                    size_text = collision.findtext("geometry/box/size")
                    if not size_text:
                        continue
                    collision_pose = GazeboMid360FastlioAdapter._box_pose(
                        collision.findtext("pose")
                    )
                    size = tuple(float(value) for value in size_text.split())
                    if len(size) != 3:
                        raise rospy.ROSInitException("invalid Gazebo collision box")
                    center = tuple(
                        model_pose[i] + link_pose[i] + collision_pose[i]
                        for i in range(3)
                    )
                    boxes.append((center, tuple(value / 2.0 for value in size)))
        if not boxes:
            raise rospy.ROSInitException("world geometry filter found no static boxes")
        return boxes

    @staticmethod
    def _box_pose(text):
        values = [float(value) for value in text.split()] if text else [0.0] * 6
        if len(values) != 6 or any(abs(value) > 1e-6 for value in values[3:]):
            raise rospy.ROSInitException(
                "world geometry filter requires axis-aligned collision boxes"
            )
        return values

    def _on_static_surface(self, point):
        for center, half in self._world_boxes:
            outside = [abs(point[i] - center[i]) - half[i] for i in range(3)]
            if all(value <= 0.0 for value in outside):
                distance = min(-value for value in outside)
            else:
                distance = math.sqrt(sum(max(0.0, value) ** 2 for value in outside))
            if distance <= self.surface_tolerance:
                return True
        return False

    def _first_static_hit(self, origin, direction):
        nearest = float("inf")
        for center, half in self._world_boxes:
            t_enter, t_exit = -float("inf"), float("inf")
            for axis in range(3):
                low, high = center[axis] - half[axis], center[axis] + half[axis]
                if abs(direction[axis]) < 1e-9:
                    if origin[axis] < low or origin[axis] > high:
                        t_enter = float("inf")
                        break
                    continue
                a = (low - origin[axis]) / direction[axis]
                b = (high - origin[axis]) / direction[axis]
                t_enter = max(t_enter, min(a, b))
                t_exit = min(t_exit, max(a, b))
            if t_enter <= t_exit and t_exit > 0.0:
                nearest = min(nearest, t_enter if t_enter > 0.0 else t_exit)
        return nearest

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

    @staticmethod
    def _matmul(left, right):
        return tuple(
            tuple(sum(left[row][k] * right[k][column] for k in range(3))
                  for column in range(3))
            for row in range(3)
        )

    @staticmethod
    def _rpy_matrix(roll, pitch, yaw):
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)
        return (
            (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
            (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
            (-sp, cp * sr, cp * cr),
        )

    def _cloud_callback(self, cloud):
        with self._lock:
            odom = self._latest_odom
            px4_pose = self._latest_px4_pose
            link_history = list(self._link_history)
        if odom is None:
            rospy.logwarn_throttle(2.0, "MID360 cloud waiting for /Odometry")
            return

        now = rospy.Time.now()
        odom_age = (now - odom.header.stamp).to_sec()
        if odom_age < -self.future_odom_tolerance or odom_age > self.max_input_age:
            rospy.logwarn_throttle(
                2.0, "MID360 cloud rejected: odometry is %.3f s old", odom_age
            )
            return

        planner_odom = copy.deepcopy(odom)
        if self.px4_pose_topic:
            if px4_pose is None:
                rospy.logwarn_throttle(2.0, "MID360 cloud waiting for PX4 height")
                return
            px4_height = px4_pose.pose.position.z
            px4_age = (now - px4_pose.header.stamp).to_sec()
            if (not math.isfinite(px4_height)
                    or px4_age < -self.future_odom_tolerance
                    or px4_age > self.max_height_age):
                rospy.logwarn_throttle(
                    2.0, "MID360 cloud rejected: PX4 height is %.3f s old", px4_age
                )
                return
            with self._lock:
                if self._height_alignment is None:
                    self._height_alignment = (
                        odom.pose.pose.position.z - px4_height
                    )
                    rospy.loginfo(
                        "Planner height aligned: FAST-LIO z %.3f, PX4 baro z %.3f, "
                        "offset %.3f",
                        odom.pose.pose.position.z,
                        px4_height,
                        self._height_alignment,
                    )
                height_alignment = self._height_alignment
            # Task one requires MID360/FAST-LIO2 horizontal localization, while
            # PX4's 1018 profile deliberately retains barometric height.  Use
            # that same height for both the planning cloud and planner odometry
            # so vertical lidar drift cannot move the map through the vehicle.
            planner_odom.pose.pose.position.z = px4_height + height_alignment

        position = planner_odom.pose.pose.position
        if self._world_boxes:
            if not link_history:
                rospy.logwarn_throttle(2.0, "MID360 waiting for Gazebo sensor pose")
                return
            stamp, sensor_pose, base_pose = min(
                link_history,
                key=lambda item: abs((item[0] - cloud.header.stamp).to_sec()),
            )
            if abs((stamp - cloud.header.stamp).to_sec()) > 0.04:
                rospy.logwarn_throttle(2.0, "MID360 sensor pose is not synchronized")
                return
            rotation = self._rotation_matrix(sensor_pose.orientation)
            sensor_world = sensor_pose.position
            base_world = base_pose.position
            world_origin = (sensor_world.x, sensor_world.y, sensor_world.z)
            origin = (
                position.x + sensor_world.x - base_world.x,
                position.y + sensor_world.y - base_world.y,
                position.z + sensor_world.z - base_world.z,
            )
        else:
            body_rotation = self._rotation_matrix(odom.pose.pose.orientation)
            offset_world = (
                self._rotate(body_rotation, self.sensor_offset)
                if body_rotation else None
            )
            rotation = (
                self._matmul(body_rotation, self.sensor_rotation)
                if body_rotation else None
            )
            world_origin = None
            origin = (
                position.x + offset_world[0],
                position.y + offset_world[1],
                position.z + offset_world[2],
            ) if offset_world else None
        if rotation is None or origin is None:
            rospy.logwarn_throttle(2.0, "MID360 cloud rejected: invalid attitude")
            return

        registered = []
        free_ray_endpoints = []
        filtered_self = 0
        filtered_geometry = 0
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
                # Gazebo's block-laser plugin encodes a no-return beam as a
                # finite point beyond the configured sensor range (typically
                # 70 m for this 30 m sensor). The regular obstacle cloud must
                # never contain it, but the traversed portion is useful
                # free-space evidence for the simulator's decision map.
                if self.free_ray_pub is not None and index % 4 == 0:
                    distance = math.sqrt(range_squared)
                    direction = self._rotate(
                        rotation, tuple(value / distance for value in point)
                    )
                    free_distance = self.max_range - 0.5
                    if world_origin is not None:
                        free_distance = min(
                            free_distance,
                            self._first_static_hit(world_origin, direction) - 0.35,
                        )
                    if free_distance <= self.min_range:
                        continue
                    rotated = tuple(free_distance * value for value in direction)
                    free_ray_endpoints.append(tuple(origin[i] + rotated[i] for i in range(3)))
                continue
            # Gazebo's generic ray sensor can see the Iris fuselage/landing
            # gear because it has no Livox-style self-return suppression.  A
            # self return becomes a false obstacle around the takeoff point
            # and makes SUPER's CIRI corridor infeasible.  This filter is
            # deliberately expressed in the sensor frame and is only for the
            # simulation adapter; real Fast-LIO2 data must not use it.
            if (
                self.self_filter_enable
                and point[0] * point[0] + point[1] * point[1]
                <= self.self_filter_xy_radius * self.self_filter_xy_radius
                and self.self_filter_z_min <= point[2] <= self.self_filter_z_max
            ):
                filtered_self += 1
                continue
            rotated = self._rotate(rotation, point)
            if world_origin is not None:
                world_point = tuple(
                    world_origin[i] + rotated[i] for i in range(3)
                )
                if not self._on_static_surface(world_point):
                    filtered_geometry += 1
                    continue
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
        # The registered cloud and pose form one synthetic Fast-LIO2 sample.
        # Give them the same timestamp so the downstream ApproximateTime
        # synchronizer cannot lose pairs when a dense scan takes noticeable
        # CPU time to transform.
        header.stamp = planner_odom.header.stamp
        header.frame_id = self.world_frame
        if rospy.is_shutdown():
            return
        try:
            if self.planner_odom_pub is not None:
                self.planner_odom_pub.publish(planner_odom)
            self.registered_pub.publish(
                point_cloud2.create_cloud_xyz32(header, registered)
            )
            if self.free_ray_pub is not None and free_ray_endpoints:
                self.free_ray_pub.publish(
                    point_cloud2.create_cloud_xyz32(header, free_ray_endpoints)
                )
        except rospy.ROSException as error:
            # A sensor callback can finish after roslaunch has already closed
            # publishers. Suppress only that normal shutdown race.
            if rospy.is_shutdown() or "closed topic" in str(error):
                return
            raise
        self._accumulate(registered)
        self._cloud_count += 1
        rospy.loginfo_throttle(
            5.0,
            "Gazebo MID360 registered %d points; accumulated %d voxels",
            len(registered),
            len(self._global_voxels),
        )
        if filtered_self:
            rospy.loginfo_throttle(
                5.0,
                "Gazebo MID360 self-return filter removed %d raw points",
                filtered_self,
            )
        if filtered_geometry:
            rospy.loginfo_throttle(
                5.0,
                "Gazebo static geometry filter rejected %d inconsistent hits",
                filtered_geometry,
            )

    def _accumulate(self, points):
        inverse = 1.0 / self.voxel_size
        with self._lock:
            for point in points:
                key = tuple(int(math.floor(value * inverse)) for value in point)
                if key in self._global_voxels:
                    continue
                if len(self._global_voxels) >= self.max_global_points:
                    break
                self._global_voxels[key] = point

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
