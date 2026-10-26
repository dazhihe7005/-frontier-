#!/usr/bin/env python3

"""Exercise edge triggering, repeat runs, exclusion, and pilot takeover."""

import math
import threading
import time
import unittest

import rospy
import rostest
from geometry_msgs.msg import TransformStamped
from mavros_msgs.msg import RCIn, State
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String, UInt8, UInt64
from super_planner.msg import GoalCommand

GOAF_CHANNEL = 8    # Physical CH9.
TUNNEL_CHANNEL = 6  # Physical CH7.
TASK3_CHANNEL = 5   # Physical CH6; assigned but availability-gated.
SHAFT_CHANNEL = 10  # Physical CH11.


class MissionSchedulerEdgesTest(unittest.TestCase):
    def setUp(self):
        self.lock = threading.Lock()
        self.channels = [1500] * 11
        self.channels[TUNNEL_CHANNEL] = 1000
        self.channels[GOAF_CHANNEL] = 1000
        self.channels[TASK3_CHANNEL] = 1000
        self.channels[SHAFT_CHANNEL] = 1000
        self.mode = "POSCTL"
        # Production starts disarmed; task-switch baselines must still settle.
        self.armed = False
        self.publish_odom = True
        self.publish_state = True
        self.publish_shaft_status = True
        self.publish_shaft_ready = True
        self.odom_position = [0.0, 0.0, 1.0]
        self.odom_motion_step = 0.0
        self.shaft_status = "IDLE"
        self.shaft_ready = False
        self.task3_ready = False
        self.task = None
        self.reason = None
        self.goaf_enabled = None
        self.shaft_enabled = None
        self.task3_enabled = None
        self.tunnel_enabled = None
        self.exploration_enabled = None
        self.auto_enabled = None
        self.return_home = None
        self.rc_pub = rospy.Publisher("/test/mission/rc", RCIn, queue_size=2)
        self.odom_pub = rospy.Publisher("/test/mission/odom", Odometry, queue_size=2)
        self.state_pub = rospy.Publisher("/test/mission/mavros_state", State, queue_size=2)
        self.alignment_pub = rospy.Publisher(
            "/test/mission/alignment", TransformStamped, queue_size=1, latch=True
        )
        self.command_fault_pub = rospy.Publisher(
            "/test/mission/command_fault", String, queue_size=1, latch=True
        )
        self.initial_goal_pub = rospy.Publisher(
            "/test/mission/accepted_initial_goal_id", UInt64,
            queue_size=1, latch=True
        )
        self.goal_command_pub = rospy.Publisher(
            "/test/mission/goal_command", GoalCommand,
            queue_size=2, latch=True
        )
        self.super_goal_accepted_pub = rospy.Publisher(
            "/test/mission/super_accepted_goal_id", UInt64,
            queue_size=2, latch=True
        )
        self.finished_pub = rospy.Publisher(
            "/mine_uav/exploration/finished", Bool, queue_size=2, latch=True
        )
        self.shaft_status_pub = rospy.Publisher(
            "/mine_uav/shaft/status", String, queue_size=2, latch=True
        )
        self.shaft_ready_pub = rospy.Publisher(
            "/mine_uav/task2/command_ready", Bool, queue_size=2, latch=True
        )
        self.task3_ready_pub = rospy.Publisher(
            "/mine_uav/task3/command_ready", Bool, queue_size=2, latch=True
        )
        rospy.Subscriber("/mine_uav/mission/active_task", UInt8, self.on_task)
        rospy.Subscriber("/mine_uav/mission/goaf_enable", Bool, self.on_goaf)
        rospy.Subscriber("/mine_uav/mission/shaft_enable", Bool, self.on_shaft)
        rospy.Subscriber("/mine_uav/mission/task3_enable", Bool, self.on_task3)
        rospy.Subscriber("/mine_uav/mission/tunnel_enable", Bool, self.on_tunnel)
        rospy.Subscriber(
            "/mine_uav/mission/exploration_enable", Bool,
            self.on_exploration)
        rospy.Subscriber("/mine_uav/mission/auto_enable", Bool, self.on_auto)
        rospy.Subscriber("/mine_uav/mission/return_home", Bool, self.on_return)
        rospy.Subscriber("/mine_uav/mission/status", String, self.on_scheduler_status)
        self.timer = rospy.Timer(rospy.Duration(0.03), self.publish_inputs)
        self.finished_pub.publish(Bool(data=False))
        self.shaft_status_pub.publish(String(data="IDLE"))
        self.shaft_ready_pub.publish(Bool(data=False))
        self.task3_ready_pub.publish(Bool(data=False))
        alignment = TransformStamped()
        alignment.header.stamp = rospy.Time.now()
        alignment.header.frame_id = "odom"
        alignment.child_frame_id = "task1_planning"
        alignment.transform.rotation.z = math.sin(math.radians(30.0))
        alignment.transform.rotation.w = math.cos(math.radians(30.0))
        self.alignment_pub.publish(alignment)
        self.command_fault_pub.publish(String(data=""))
        self.initial_goal_pub.publish(UInt64(data=0))

    def tearDown(self):
        self.timer.shutdown()

    def on_task(self, message):
        with self.lock:
            self.task = message.data

    def on_goaf(self, message):
        with self.lock:
            self.goaf_enabled = message.data

    def on_shaft(self, message):
        with self.lock:
            self.shaft_enabled = message.data

    def on_task3(self, message):
        with self.lock:
            self.task3_enabled = message.data

    def on_tunnel(self, message):
        with self.lock:
            self.tunnel_enabled = message.data

    def on_exploration(self, message):
        with self.lock:
            self.exploration_enabled = message.data

    def on_auto(self, message):
        with self.lock:
            self.auto_enabled = message.data

    def on_return(self, message):
        with self.lock:
            self.return_home = message.data

    def on_scheduler_status(self, message):
        with self.lock:
            self.reason = message.data.rsplit(" reason=", 1)[-1].split(" ", 1)[0]

    def publish_inputs(self, _event):
        with self.lock:
            channels = list(self.channels)
            mode = self.mode
            armed = self.armed
            publish_odom = self.publish_odom
            publish_state = self.publish_state
            publish_shaft_status = self.publish_shaft_status
            publish_shaft_ready = self.publish_shaft_ready
            shaft_status = self.shaft_status
            shaft_ready = self.shaft_ready
            task3_ready = self.task3_ready
            self.odom_position[0] += self.odom_motion_step
            odom_position = list(self.odom_position)
        rc = RCIn()
        rc.header.stamp = rospy.Time.now()
        rc.channels = channels
        self.rc_pub.publish(rc)
        if publish_odom:
            odom = Odometry()
            odom.header.stamp = rospy.Time.now()
            odom.header.frame_id = "task1_planning"
            odom.child_frame_id = "base_link"
            odom.pose.pose.position.x = odom_position[0]
            odom.pose.pose.position.y = odom_position[1]
            odom.pose.pose.position.z = odom_position[2]
            odom.pose.pose.orientation.w = 1.0
            self.odom_pub.publish(odom)
        if publish_state:
            state = State()
            state.connected = True
            state.armed = armed
            state.mode = mode
            self.state_pub.publish(state)
        if publish_shaft_status:
            self.shaft_status_pub.publish(String(data=shaft_status))
        if publish_shaft_ready:
            self.shaft_ready_pub.publish(Bool(data=shaft_ready))
        self.task3_ready_pub.publish(Bool(data=task3_ready))

    def set_channel(self, index, value):
        with self.lock:
            self.channels[index] = value

    def set_shaft_status(self, value):
        with self.lock:
            self.shaft_status = value

    def set_shaft_ready(self, value):
        with self.lock:
            self.shaft_ready = value

    def set_task3_ready(self, value):
        with self.lock:
            self.task3_ready = value

    def assert_reason(self, reason, timeout=3.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self.lock:
                if self.reason == reason:
                    return
            time.sleep(0.02)
        self.fail("expected reason {} but saw {}".format(reason, self.reason))

    def assert_task(self, task, timeout=3.0, auto_enabled=None):
        if auto_enabled is None:
            auto_enabled = (task in (2, 3))
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not rospy.is_shutdown():
            with self.lock:
                if (self.task == task and
                        self.goaf_enabled == (task == 1) and
                        self.shaft_enabled == (task == 2) and
                        self.task3_enabled == (task == 3) and
                        self.tunnel_enabled == (task == 4) and
                        self.exploration_enabled == (task in (1, 4)) and
                        self.auto_enabled == auto_enabled):
                    return
            time.sleep(0.02)
        self.fail("expected task {} but saw {}".format(task, self.task))

    def accept_initial_goal(self, goal_id, task=1):
        self.initial_goal_pub.publish(UInt64(data=0))
        time.sleep(0.08)
        self.initial_goal_pub.publish(UInt64(data=goal_id))
        self.assert_task(task, auto_enabled=True)

    def assert_stays(self, task, seconds=0.3):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            with self.lock:
                self.assertEqual(self.task, task)
            time.sleep(0.02)

    def test_edge_lifecycle(self):
        self.assert_task(0)
        deadline = time.monotonic() + 3.0
        while self.rc_pub.get_num_connections() == 0 and time.monotonic() < deadline:
            time.sleep(0.02)
        self.assertGreater(self.rc_pub.get_num_connections(), 0)
        self.assert_stays(0, 0.35)  # Let both initial levels become baselines.

        # Every task channel establishes its baseline before arming.  HIGH is a
        # valid CH7 baseline: after arming, one HIGH-to-LOW change must start
        # Task 4. A disarmed level change updates the baseline but never starts
        # a task.
        with self.lock:
            self.armed = False
        self.set_channel(TUNNEL_CHANNEL, 2000)
        self.assert_stays(0, 0.80)
        with self.lock:
            self.armed = True
        self.set_channel(TUNNEL_CHANNEL, 1000)
        self.assert_task(4)
        self.set_channel(TUNNEL_CHANNEL, 2000)
        self.assert_task(0)
        self.assert_reason("task4_trigger_cancelled")
        time.sleep(0.30)

        # A Task-3 transition made before arming is consumed as a baseline
        # update. Holding that endpoint through arming does not start a task;
        # the next complete endpoint transition does.
        with self.lock:
            self.armed = False
        self.set_channel(TASK3_CHANNEL, 2000)
        self.assert_stays(0, 0.80)
        with self.lock:
            self.armed = True
        self.assert_stays(0, 0.80)
        self.set_channel(TASK3_CHANNEL, 1000)
        self.assert_task(3)
        self.set_channel(TASK3_CHANNEL, 2000)
        self.assert_task(0)
        self.assert_reason("task3_trigger_cancelled")
        time.sleep(0.70)

        # Either CH9 edge requests Task 1 while idle. A second edge during the
        # hover gate cancels the pending request instead of immediately
        # restarting it. The scheduler must keep both authorities closed.
        with self.lock:
            self.odom_motion_step = 0.03
        self.set_channel(GOAF_CHANNEL, 2000)
        time.sleep(0.20)
        self.set_channel(GOAF_CHANNEL, 1000)
        self.assert_stays(0, 0.35)

        # Request again, then open planning only after the stable-hover gate;
        # automatic control remains disabled until this epoch's first goal is
        # explicitly accepted.
        with self.lock:
            self.odom_motion_step = 0.0
        self.set_channel(GOAF_CHANNEL, 2000)
        self.assert_task(1)
        # Regression from the 2026-10-24 flight: stability can be lost while
        # SUPER spends several seconds finding and accepting the first goal.
        # Acceptance must not grant automatic authority until a new continuous
        # hover window has completed at the actual handoff.
        with self.lock:
            self.odom_motion_step = 0.03
        command = GoalCommand()
        command.header.stamp = rospy.Time.now()
        command.command = GoalCommand.SET_GOAL
        command.goal_id = 101
        self.goal_command_pub.publish(command)
        time.sleep(0.08)
        self.super_goal_accepted_pub.publish(UInt64(data=101))
        self.assert_stays(1, 0.35)
        with self.lock:
            self.assertFalse(self.auto_enabled)
            self.odom_motion_step = 0.0
        self.assert_task(1, auto_enabled=True)
        self.set_channel(GOAF_CHANNEL, 1000)
        self.assert_task(0)  # Any later CH9 edge explicitly cancels Task 1.

        # A stale, non-zero readiness ID from the previous mission must not
        # enable the next mission. A zero reset followed by a fresh ID is
        # required, and a bridge fault must close both authorities immediately.
        time.sleep(0.30)
        self.set_channel(GOAF_CHANNEL, 2000)
        self.assert_task(1)
        self.initial_goal_pub.publish(UInt64(data=999))
        self.assert_stays(1, 0.20)
        with self.lock:
            self.assertFalse(self.auto_enabled)
        self.accept_initial_goal(102)
        # Keep CH11 at its non-triggering three-position centre before the
        # fault resets all edge trackers. A centred channel at reset time must
        # acquire a stable timestamp and must not hold the global rearm barrier
        # forever (the exact failure seen on the third 2026-10-24 attempt).
        self.set_channel(SHAFT_CHANNEL, 1500)
        time.sleep(0.18)
        self.command_fault_pub.publish(String(data="MEASURED_SPEED_LIMIT"))
        self.assert_task(0)
        self.assert_reason("task1_command_fault_MEASURED_SPEED_LIMIT")
        self.command_fault_pub.publish(String(data=""))

        # Regression from the 2026-10-24 flight: CH11 was parked at its
        # three-position centre value (~1542).  That non-triggering value must
        # not keep the post-fault rearm barrier asserted and swallow every
        # later CH9 edge.
        # A second fault type in a fresh epoch must also close both authorities
        # without restarting the scheduler or clearing a fault implicitly.
        self.set_channel(GOAF_CHANNEL, 1000)
        # A real landing/disarm is itself a clean lifecycle boundary. After a
        # subsequent arm, one ordinary edge on the requested task channel is
        # sufficient; no extra reset gesture or movement of another channel is
        # required.
        with self.lock:
            self.armed = False
        time.sleep(0.30)
        with self.lock:
            self.armed = True
        time.sleep(0.20)
        self.set_channel(GOAF_CHANNEL, 2000)
        self.assert_task(1)
        self.set_channel(SHAFT_CHANNEL, 1000)
        self.accept_initial_goal(103)
        self.command_fault_pub.publish(String(data="SPEED_GOVERNOR_INFEASIBLE"))
        self.assert_task(0)
        self.assert_reason("task1_command_fault_SPEED_GOVERNOR_INFEASIBLE")
        self.command_fault_pub.publish(String(data=""))

        # Same task can start again without restarting any node. Completion
        # closes authority, and pilot takeover before or after OFFBOARD does too.
        self.set_channel(GOAF_CHANNEL, 1000)
        time.sleep(0.30)
        self.set_channel(GOAF_CHANNEL, 2000)
        self.assert_task(1)
        self.accept_initial_goal(104)
        self.finished_pub.publish(Bool(data=True))
        self.assert_task(0)
        self.finished_pub.publish(Bool(data=False))

        # Completion leaves CH9 high. After that stable level is re-armed, a
        # high-to-low edge must be able to start Task 1 too; start direction is
        # deliberately not fixed.
        time.sleep(0.30)
        self.set_channel(GOAF_CHANNEL, 1000)
        self.assert_task(1)
        self.accept_initial_goal(105)
        with self.lock:
            self.mode = "OFFBOARD"
        time.sleep(0.15)
        with self.lock:
            self.mode = "POSCTL"
        self.assert_task(0)  # Pilot takeover revokes task authority.

        # Task 1 loss of planning odometry closes authority. Task 2 remains
        # independent of Fast-LIO but retains its own status watchdog.
        self.set_channel(GOAF_CHANNEL, 1000)
        time.sleep(0.30)
        self.set_channel(GOAF_CHANNEL, 2000)
        self.assert_task(1)
        self.accept_initial_goal(106)
        with self.lock:
            self.publish_odom = False
        self.assert_task(0, timeout=2.0)
        with self.lock:
            self.assertFalse(self.return_home)

        with self.lock:
            self.publish_odom = True
        self.set_channel(GOAF_CHANNEL, 1000)
        time.sleep(0.30)
        self.set_channel(SHAFT_CHANNEL, 2000)
        self.assert_task(2)
        self.set_shaft_status("DESCENDING")
        self.set_shaft_ready(True)
        time.sleep(0.12)
        self.set_shaft_status("COMPLETE")
        self.assert_task(0)
        self.set_shaft_status("IDLE")
        self.set_shaft_ready(False)

        # A stopped MAVROS state stream must revoke Task 2; the last heartbeat
        # cannot remain a permanent permission to keep controlling the FCU.
        time.sleep(0.30)
        self.set_channel(SHAFT_CHANNEL, 1000)
        self.assert_task(2)
        self.set_shaft_status("DESCENDING")
        self.set_shaft_ready(True)
        time.sleep(0.15)
        with self.lock:
            self.publish_state = False
        self.assert_task(0, timeout=2.0)
        self.assert_reason("mavros_state_lost")

        with self.lock:
            self.publish_state = True
        self.set_shaft_status("IDLE")
        self.set_shaft_ready(False)
        time.sleep(0.30)
        self.set_channel(SHAFT_CHANNEL, 2000)
        self.assert_task(2)
        self.set_shaft_status("DESCENDING")
        self.set_shaft_ready(True)
        time.sleep(0.15)
        with self.lock:
            self.publish_shaft_status = False
        self.assert_task(0, timeout=2.0)
        self.assert_reason("task2_status_lost")

        with self.lock:
            self.publish_shaft_status = True
        self.set_shaft_status("IDLE")
        self.set_shaft_ready(False)
        time.sleep(0.30)
        self.set_channel(SHAFT_CHANNEL, 1000)
        self.assert_task(2)
        self.assert_task(0, timeout=2.0)  # No sensors: IDLE never becomes DESCENDING.
        self.assert_reason("task2_start_timeout")

        # CH6 is a real Task-3 authority path. The controller readiness alone
        # is not enough: losing the router's final OFFBOARD-ready heartbeat
        # after startup must revoke the task.
        time.sleep(0.30)
        self.set_channel(TASK3_CHANNEL, 1000)
        self.assert_task(3)
        self.set_task3_ready(True)
        self.assert_stays(3, 0.15)
        self.set_task3_ready(False)
        self.assert_task(0, timeout=2.0)
        self.assert_reason("task3_command_not_ready")

        # Task 4 must use the exact same stable-hover and initial-goal handoff
        # as Task 1. Only its task identity/profile selection differs.
        time.sleep(0.30)
        self.set_channel(TUNNEL_CHANNEL, 1000)
        self.assert_task(4)
        self.accept_initial_goal(401, task=4)
        self.finished_pub.publish(Bool(data=True))
        self.assert_task(0)
        self.assert_reason("task4_complete")

        # Completion leaves CH7 low.  Its opposite transition must also
        # start Task 4, proving that production task channels are level-change
        # triggers rather than direction-specific rising-edge triggers.
        self.finished_pub.publish(Bool(data=False))
        time.sleep(0.30)
        self.set_channel(TUNNEL_CHANNEL, 2000)
        self.assert_task(4)
        self.accept_initial_goal(402, task=4)
        self.set_channel(TUNNEL_CHANNEL, 1000)
        self.assert_task(0)
        self.assert_reason("task4_trigger_cancelled")


if __name__ == "__main__":
    rospy.init_node("test_mission_scheduler_edges")
    rostest.rosrun("mine_uav_control", "mission_scheduler_edges", MissionSchedulerEdgesTest)
