#!/usr/bin/env python3

"""PX4 SITL adapter for the task-one exploration closed-loop test.

The node republishes MAVROS ENU odometry in the Fast-LIO2-compatible topics,
generates a registered point cloud for an open-entrance/three-wall mine, and
emulates the two independent RC switches used by the mission scheduler.
Automatic arming is deliberately accepted only while Gazebo simulation time
is active.
"""

import math
import threading

import rospy
from geometry_msgs.msg import PoseStamped, TransformStamped
from mavros_msgs.msg import RCIn, State
from mavros_msgs.srv import CommandBool, SetMode
from nav_msgs.msg import Odometry, Path
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool


class Task1SitlAdapter:
    def __init__(self):
        self.world_frame = rospy.get_param("~world_frame", "camera_init")
        self.input_odom_topic = rospy.get_param(
            "~input_odom_topic", "/mavros/local_position/odom"
        )
        self.sensor_range = float(rospy.get_param("~sensor_range", 10.0))
        self.cloud_rate = max(1.0, float(rospy.get_param("~cloud_rate", 10.0)))
        self.rc_rate = max(2.0, float(rospy.get_param("~rc_rate", 10.0)))
        self.auto_enable_delay = max(
            2.0, float(rospy.get_param("~auto_enable_delay", 8.0))
        )
        self.allow_auto_arm = bool(rospy.get_param("~allow_auto_arm", True))
        self.simulate_manual_takeoff = bool(
            rospy.get_param("~simulate_manual_takeoff", True)
        )
        self.takeoff_height = max(
            0.5, float(rospy.get_param("~takeoff_height", 1.5))
        )
        self.takeoff_hover_duration = max(
            0.5, float(rospy.get_param("~takeoff_hover_duration", 1.0))
        )
        self.offboard_prestream_duration = max(
            1.0, float(rospy.get_param("~offboard_prestream_duration", 1.0))
        )
        self.publish_analytic_cloud = bool(
            rospy.get_param("~publish_analytic_cloud", True)
        )
        self.arm_delay = max(1.0, float(rospy.get_param("~arm_delay", 4.0)))
        self.task_channel_index = int(rospy.get_param("~task_channel_index", 5))
        self.auto_channel_index = int(rospy.get_param("~auto_channel_index", 6))

        self._lock = threading.Lock()
        self._latest_odom = None
        self._state = State()
        self._start_time = rospy.Time.now()
        self._last_arm_request = rospy.Time(0)
        self._last_mode_request = rospy.Time(0)
        self._takeoff_origin = None
        self._takeoff_prestream_start = rospy.Time(0)
        self._takeoff_reached_time = rospy.Time(0)
        self._takeoff_complete = not self.simulate_manual_takeoff
        self._environment = (
            self._build_environment() if self.publish_analytic_cloud else []
        )
        self._path = Path()
        self._path.header.frame_id = self.world_frame

        self.odom_pub = rospy.Publisher("/Odometry", Odometry, queue_size=10)
        self.cloud_pub = None
        self.global_cloud_pub = None
        if self.publish_analytic_cloud:
            self.cloud_pub = rospy.Publisher(
                "/cloud_registered", PointCloud2, queue_size=2
            )
            self.global_cloud_pub = rospy.Publisher(
                "/mine_uav/sitl/global_cloud", PointCloud2, queue_size=1, latch=True
            )
        self.path_pub = rospy.Publisher(
            "/mine_uav/sitl/px4_path", Path, queue_size=1, latch=True
        )
        self.rc_pub = rospy.Publisher(
            "/mine_uav/sitl/rc/in", RCIn, queue_size=2
        )
        self.takeoff_setpoint_pub = rospy.Publisher(
            "/mavros/setpoint_position/local", PoseStamped, queue_size=10
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

        rospy.Subscriber(
            self.input_odom_topic, Odometry, self._odom_callback, queue_size=20
        )
        rospy.Subscriber("/mavros/state", State, self._state_callback, queue_size=10)
        self.arm_client = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        self.mode_client = rospy.ServiceProxy("/mavros/set_mode", SetMode)

        rospy.Timer(rospy.Duration(1.0 / self.cloud_rate), self._publish_sensor)
        rospy.Timer(rospy.Duration(1.0 / self.rc_rate), self._publish_operator)
        rospy.Timer(rospy.Duration(2.0), self._publish_health)
        self._publish_health(None)

        rospy.logwarn(
            "task1_sitl_adapter active: simulated mine cloud and RC switches only; "
            "never run this node against a real FCU"
        )

    @staticmethod
    def _axis_samples(start, stop, step):
        count = int(round((stop - start) / step))
        return [start + i * step for i in range(count + 1)]

    def _build_environment(self):
        points = []
        x_values = self._axis_samples(-2.0, 22.0, 0.4)
        y_values = self._axis_samples(-4.0, 4.0, 0.4)
        z_values = self._axis_samples(0.0, 4.0, 0.4)

        # Two side walls, the closed far wall, and the ceiling. The entrance at
        # x=-2 remains open, matching the intended goaf geometry.
        for x in x_values:
            for z in z_values:
                points.append((x, -4.0, z))
                points.append((x, 4.0, z))
        for y in y_values:
            for z in z_values:
                points.append((22.0, y, z))
        for x in x_values:
            for y in y_values:
                points.append((x, y, 4.0))
        return points

    def _state_callback(self, message):
        with self._lock:
            self._state = message

    def _odom_callback(self, message):
        converted = Odometry()
        converted.header.stamp = rospy.Time.now()
        converted.header.frame_id = self.world_frame
        converted.child_frame_id = "base_link"
        converted.pose = message.pose
        converted.twist = message.twist
        with self._lock:
            self._latest_odom = converted

    def _publish_sensor(self, _event):
        with self._lock:
            odom = self._latest_odom
        if odom is None:
            return

        stamp = rospy.Time.now()
        odom.header.stamp = stamp
        px = odom.pose.pose.position.x
        py = odom.pose.pose.position.y
        pz = odom.pose.pose.position.z
        header = odom.header
        self.odom_pub.publish(odom)
        if self.publish_analytic_cloud:
            range_squared = self.sensor_range * self.sensor_range
            visible = [
                point
                for point in self._environment
                if (point[0] - px) ** 2
                + (point[1] - py) ** 2
                + (point[2] - pz) ** 2
                <= range_squared
            ]
            cloud = point_cloud2.create_cloud_xyz32(header, visible)
            self.cloud_pub.publish(cloud)

        pose = odom.pose.pose
        if not self._path.poses or self._path_distance_from_last(pose) >= 0.15:
            from geometry_msgs.msg import PoseStamped

            stamped_pose = PoseStamped()
            stamped_pose.header = header
            stamped_pose.pose = pose
            self._path.poses.append(stamped_pose)
            if len(self._path.poses) > 5000:
                self._path.poses = self._path.poses[-5000:]
        self._path.header.stamp = stamp
        self.path_pub.publish(self._path)

    def _path_distance_from_last(self, pose):
        previous = self._path.poses[-1].pose.position
        current = pose.position
        return math.sqrt(
            (current.x - previous.x) ** 2
            + (current.y - previous.y) ** 2
            + (current.z - previous.z) ** 2
        )

    def _publish_operator(self, _event):
        elapsed = (rospy.Time.now() - self._start_time).to_sec()
        with self._lock:
            state = self._state

        channel_count = max(8, self.task_channel_index + 1, self.auto_channel_index + 1)
        rc = RCIn()
        rc.header.stamp = rospy.Time.now()
        rc.channels = [1500] * channel_count
        rc.channels[self.task_channel_index] = 1000  # task one
        if self.allow_auto_arm and self.simulate_manual_takeoff:
            self._run_simulated_manual_takeoff(state)
        rc.channels[self.auto_channel_index] = 2000 if (
            elapsed >= self.auto_enable_delay
            and state.armed
            and self._takeoff_complete
        ) else 1000
        self.rc_pub.publish(rc)

        if (not self.allow_auto_arm or self.simulate_manual_takeoff or
                state.armed or not state.connected):
            return
        if elapsed < self.arm_delay or self._latest_odom is None:
            return
        if not rospy.get_param("/use_sim_time", False):
            rospy.logerr_throttle(
                2.0, "Refusing SITL auto-arm because /use_sim_time is false"
            )
            return
        now = rospy.Time.now()
        if (now - self._last_arm_request).to_sec() < 1.0:
            return
        self._last_arm_request = now
        try:
            response = self.arm_client(True)
            if response.success:
                rospy.loginfo("PX4 SITL armed; automatic task permission will follow")
            else:
                rospy.logwarn_throttle(2.0, "PX4 SITL rejected arm request")
        except rospy.ServiceException as error:
            rospy.logwarn_throttle(2.0, "SITL arm service failed: %s", error)

    def _run_simulated_manual_takeoff(self, state):
        """Place SITL in the same state as a pilot-established hover.

        Task one is kept disabled while a position setpoint is pre-streamed,
        OFFBOARD is entered, the vehicle climbs 1.5 m, and the hover remains
        stable for one second. Only then may the emulated CH7 go high.
        """
        with self._lock:
            odom = self._latest_odom
        if odom is None or not state.connected or self._takeoff_complete:
            return

        now = rospy.Time.now()
        if self._takeoff_origin is None:
            self._takeoff_origin = (
                odom.pose.pose.position.x,
                odom.pose.pose.position.y,
                odom.pose.pose.position.z,
                odom.pose.pose.orientation,
            )
            self._takeoff_prestream_start = now
            rospy.loginfo(
                "SITL pre-task takeoff initialized: ground z=%.2f, target z=%.2f",
                self._takeoff_origin[2],
                self._takeoff_origin[2] + self.takeoff_height,
            )

        target = PoseStamped()
        target.header.stamp = now
        target.header.frame_id = "map"
        target.pose.position.x = self._takeoff_origin[0]
        target.pose.position.y = self._takeoff_origin[1]
        target.pose.position.z = self._takeoff_origin[2] + self.takeoff_height
        target.pose.orientation = self._takeoff_origin[3]
        self.takeoff_setpoint_pub.publish(target)

        if (now - self._takeoff_prestream_start).to_sec() < \
                self.offboard_prestream_duration:
            return
        if state.mode != "OFFBOARD":
            if (now - self._last_mode_request).to_sec() < 1.0:
                return
            self._last_mode_request = now
            try:
                response = self.mode_client(custom_mode="OFFBOARD")
                if response.mode_sent:
                    rospy.loginfo("PX4 SITL pre-task OFFBOARD request accepted")
                else:
                    rospy.logwarn_throttle(2.0, "PX4 SITL rejected OFFBOARD request")
            except rospy.ServiceException as error:
                rospy.logwarn_throttle(2.0, "SITL mode service failed: %s", error)
            return

        if not state.armed:
            if (now - self._last_arm_request).to_sec() < 1.0:
                return
            self._last_arm_request = now
            try:
                response = self.arm_client(True)
                if response.success:
                    rospy.loginfo("PX4 SITL armed for pre-task takeoff")
                else:
                    rospy.logwarn_throttle(2.0, "PX4 SITL rejected arm request")
            except rospy.ServiceException as error:
                rospy.logwarn_throttle(2.0, "SITL arm service failed: %s", error)
            return

        position = odom.pose.pose.position
        horizontal_error = math.hypot(
            position.x - target.pose.position.x,
            position.y - target.pose.position.y,
        )
        vertical_error = abs(position.z - target.pose.position.z)
        if horizontal_error <= 0.20 and vertical_error <= 0.15:
            if self._takeoff_reached_time.is_zero():
                self._takeoff_reached_time = now
                rospy.loginfo("SITL reached 1.5 m pre-task hover")
            elif (now - self._takeoff_reached_time).to_sec() >= \
                    self.takeoff_hover_duration:
                self._takeoff_complete = True
                rospy.loginfo(
                    "SITL pre-task hover stable; emulated CH7 may now enable task one"
                )
        else:
            self._takeoff_reached_time = rospy.Time(0)

    def _publish_health(self, _event):
        stamp = rospy.Time.now()
        alignment = TransformStamped()
        alignment.header.stamp = stamp
        alignment.header.frame_id = self.world_frame
        alignment.child_frame_id = "map"
        alignment.transform.rotation.w = 1.0
        self.alignment_pub.publish(alignment)
        self.vision_pub.publish(Bool(data=True))

        header = Odometry().header
        header.stamp = stamp
        header.frame_id = self.world_frame
        if self.publish_analytic_cloud:
            self.global_cloud_pub.publish(
                point_cloud2.create_cloud_xyz32(header, self._environment)
            )


if __name__ == "__main__":
    rospy.init_node("task1_sitl_adapter")
    Task1SitlAdapter()
    rospy.spin()
