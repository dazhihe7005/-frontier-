#!/usr/bin/env python3
"""PyQt ground station with an editable PX4 local trajectory plan."""

import json
import math
import os
import sys
import time

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import PositionTarget, State
from PyQt5 import QtCore, QtWidgets

# When launched through a catkin-generated relay, make the source directory
# take precedence over devel/lib wrappers so trajectory_plan is imported as a
# normal Python module.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from trajectory_plan import TrajectoryPlan


class GroundStation(QtWidgets.QMainWindow):
    COLUMNS = ["类型", "高度/距离", "半径", "速度", "角度", "时间", "方向"]
    TYPE_NAMES = {
        "takeoff": "起飞",
        "hover": "悬停",
        "line": "直线",
        "arc": "圆弧",
    }

    def __init__(self):
        super().__init__()
        self.setWindowTitle("PX4 NUC 地面站 - 轨迹编辑器")
        self.resize(900, 620)

        self.setpoint_pub = rospy.Publisher(
            "/mavros/setpoint_raw/local", PositionTarget, queue_size=20
        )
        rospy.Subscriber("/mavros/state", State, self.state_callback, queue_size=10)
        rospy.Subscriber(
            "/mavros/local_position/pose", PoseStamped, self.pose_callback, queue_size=10
        )

        self.connected = False
        self.armed = False
        self.mode = "UNKNOWN"
        self.pose_valid = False
        self.pose = {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0}
        self.last_connection = False

        self.streaming = False
        self.phase = "idle"
        self.phase_started = 0.0
        self.plan_started = 0.0
        self.start_pose = None
        self.plan = None
        self.hold_point = None
        self.last_point = None

        self.build_ui()
        self.load_default_plan()

        self.publish_timer = QtCore.QTimer(self)
        self.publish_timer.timeout.connect(self.publish_setpoint)
        self.publish_timer.start(20)
        self.status_timer = QtCore.QTimer(self)
        self.status_timer.timeout.connect(self.update_status)
        self.status_timer.start(200)

    def build_ui(self):
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        layout = QtWidgets.QVBoxLayout(central)

        self.status_label = QtWidgets.QLabel("正在等待 MAVROS...")
        self.status_label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        layout.addWidget(self.status_label)

        editor_group = QtWidgets.QGroupBox("轨迹段编辑器")
        editor_layout = QtWidgets.QVBoxLayout(editor_group)
        self.table = QtWidgets.QTableWidget(0, len(self.COLUMNS))
        self.table.setHorizontalHeaderLabels(self.COLUMNS)
        self.table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.table.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
        self.table.horizontalHeader().setSectionResizeMode(
            QtWidgets.QHeaderView.Stretch
        )
        self.table.itemSelectionChanged.connect(self.load_selected_segment)
        editor_layout.addWidget(self.table)

        form = QtWidgets.QGridLayout()
        self.type_box = QtWidgets.QComboBox()
        for kind, name in self.TYPE_NAMES.items():
            self.type_box.addItem(name, kind)
        self.value_box = self.number_box(0.0, 100.0, 2.0, 0.1)
        self.radius_box = self.number_box(0.1, 100.0, 5.0, 0.1)
        self.speed_box = self.number_box(0.05, 20.0, 1.0, 0.05)
        self.angle_box = self.number_box(0.0, 360.0, 90.0, 1.0)
        self.duration_box = self.number_box(0.0, 3600.0, 3.0, 0.5)
        self.direction_box = QtWidgets.QComboBox()
        self.direction_box.addItem("左转 (+1)", 1)
        self.direction_box.addItem("右转 (-1)", -1)
        self.climb_speed_box = self.number_box(0.05, 5.0, 0.5, 0.05)

        form.addWidget(QtWidgets.QLabel("类型"), 0, 0)
        form.addWidget(self.type_box, 0, 1)
        form.addWidget(QtWidgets.QLabel("高度/距离"), 0, 2)
        form.addWidget(self.value_box, 0, 3)
        form.addWidget(QtWidgets.QLabel("半径"), 0, 4)
        form.addWidget(self.radius_box, 0, 5)
        form.addWidget(QtWidgets.QLabel("速度"), 0, 6)
        form.addWidget(self.speed_box, 0, 7)
        form.addWidget(QtWidgets.QLabel("角度"), 1, 0)
        form.addWidget(self.angle_box, 1, 1)
        form.addWidget(QtWidgets.QLabel("悬停时间"), 1, 2)
        form.addWidget(self.duration_box, 1, 3)
        form.addWidget(QtWidgets.QLabel("方向"), 1, 4)
        form.addWidget(self.direction_box, 1, 5)
        form.addWidget(QtWidgets.QLabel("爬升速度"), 1, 6)
        form.addWidget(self.climb_speed_box, 1, 7)
        editor_layout.addLayout(form)

        edit_buttons = QtWidgets.QHBoxLayout()
        for label, callback in (
            ("添加轨迹段", self.add_segment),
            ("更新选中段", self.update_segment),
            ("删除选中段", self.delete_segment),
            ("上移", self.move_up),
            ("下移", self.move_down),
            ("保存计划", self.save_plan),
            ("加载计划", self.load_plan),
        ):
            button = QtWidgets.QPushButton(label)
            button.clicked.connect(callback)
            edit_buttons.addWidget(button)
        editor_layout.addLayout(edit_buttons)
        layout.addWidget(editor_group)

        controls = QtWidgets.QGroupBox("轨迹发送")
        controls_layout = QtWidgets.QGridLayout(controls)
        buttons = [
            ("开始发送轨迹", self.start_trajectory),
            ("停止轨迹并保持", self.hold_position),
        ]
        for index, (label, callback) in enumerate(buttons):
            button = QtWidgets.QPushButton(label)
            button.clicked.connect(callback)
            controls_layout.addWidget(button, 0, index)
        layout.addWidget(controls)

        note = QtWidgets.QLabel(
            "本程序只发送轨迹 setpoint，不执行解锁、切换模式或降落。请使用 QGC 完成 "
            "OFFBOARD、解锁和降落；建议先开始发送轨迹，等待预发送完成后再由 QGC 切换 "
            "OFFBOARD。首次真实飞行请拆桨。"
        )
        note.setWordWrap(True)
        layout.addWidget(note)

    @staticmethod
    def number_box(minimum, maximum, value, step):
        box = QtWidgets.QDoubleSpinBox()
        box.setRange(minimum, maximum)
        box.setValue(value)
        box.setSingleStep(step)
        box.setDecimals(2)
        return box

    def load_default_plan(self):
        defaults = [
            {"kind": "takeoff", "height": 2.0, "climb_speed": 0.5},
            {"kind": "hover", "duration": 3.0},
            {
                "kind": "arc",
                "radius": 5.0,
                "speed": 1.0,
                "angle_deg": 90.0,
                "direction": 1,
            },
        ]
        self.table.setRowCount(0)
        for segment in defaults:
            self.append_segment(segment)

    def append_segment(self, segment):
        row = self.table.rowCount()
        self.table.insertRow(row)
        self.append_segment_at(row, segment)

    def append_segment_at(self, row, segment):
        self.table.setItem(row, 0, QtWidgets.QTableWidgetItem(self.TYPE_NAMES[segment["kind"]]))
        value = segment.get("height", segment.get("distance", 0.0))
        values = [
            "%.2f" % value,
            "%.2f" % segment.get("radius", 0.0),
            "%.2f" % segment.get("speed", segment.get("climb_speed", 0.0)),
            "%.2f" % segment.get("angle_deg", 0.0),
            "%.2f" % segment.get("duration", 0.0),
            "+1" if int(segment.get("direction", 1)) == 1 else "-1",
        ]
        for column, text in enumerate(values, start=1):
            self.table.setItem(row, column, QtWidgets.QTableWidgetItem(text))
        self.table.item(row, 0).setData(QtCore.Qt.UserRole, dict(segment))

    def selected_row(self):
        rows = self.table.selectionModel().selectedRows()
        return rows[0].row() if rows else -1

    def segment_from_form(self):
        kind = self.type_box.currentData()
        segment = {"kind": kind}
        if kind == "takeoff":
            segment.update(
                {"height": self.value_box.value(), "climb_speed": self.climb_speed_box.value()}
            )
        elif kind == "hover":
            segment["duration"] = self.duration_box.value()
        elif kind == "line":
            segment.update({"distance": self.value_box.value(), "speed": self.speed_box.value()})
        else:
            segment.update(
                {
                    "radius": self.radius_box.value(),
                    "speed": self.speed_box.value(),
                    "angle_deg": self.angle_box.value(),
                    "direction": int(self.direction_box.currentData()),
                }
            )
        return segment

    def add_segment(self):
        self.append_segment(self.segment_from_form())

    def update_segment(self):
        row = self.selected_row()
        if row >= 0:
            self.write_row(row, self.segment_from_form())
            self.table.selectRow(row)

    def load_selected_segment(self):
        row = self.selected_row()
        if row < 0:
            return
        item = self.table.item(row, 0)
        if item is None:
            return
        segment = item.data(QtCore.Qt.UserRole)
        if not isinstance(segment, dict):
            return
        index = self.type_box.findData(segment["kind"])
        self.type_box.setCurrentIndex(max(0, index))
        self.value_box.setValue(segment.get("height", segment.get("distance", 0.0)))
        self.radius_box.setValue(segment.get("radius", 5.0))
        self.speed_box.setValue(segment.get("speed", segment.get("climb_speed", 0.5)))
        self.angle_box.setValue(segment.get("angle_deg", 90.0))
        self.duration_box.setValue(segment.get("duration", 3.0))
        self.climb_speed_box.setValue(segment.get("climb_speed", 0.5))
        self.direction_box.setCurrentIndex(0 if int(segment.get("direction", 1)) == 1 else 1)

    def delete_segment(self):
        row = self.selected_row()
        if row >= 0:
            self.table.removeRow(row)

    def move_up(self):
        row = self.selected_row()
        if row > 0:
            self.swap_rows(row, row - 1)
            self.table.selectRow(row - 1)

    def move_down(self):
        row = self.selected_row()
        if 0 <= row < self.table.rowCount() - 1:
            self.swap_rows(row, row + 1)
            self.table.selectRow(row + 1)

    def swap_rows(self, first, second):
        first_data = self.table.item(first, 0).data(QtCore.Qt.UserRole)
        second_data = self.table.item(second, 0).data(QtCore.Qt.UserRole)
        self.write_row(first, second_data)
        self.write_row(second, first_data)

    def write_row(self, row, segment):
        for column in range(self.table.columnCount()):
            self.table.takeItem(row, column)
        self.append_segment_at(row, segment)

    def current_segments(self):
        return [
            self.table.item(row, 0).data(QtCore.Qt.UserRole)
            for row in range(self.table.rowCount())
        ]

    def save_plan(self):
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "保存轨迹计划", os.path.expanduser("~/trajectory.json"), "JSON (*.json)"
        )
        if path:
            with open(path, "w", encoding="utf-8") as handle:
                json.dump(self.current_segments(), handle, ensure_ascii=False, indent=2)

    def load_plan(self):
        path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "加载轨迹计划", os.path.expanduser("~"), "JSON (*.json)"
        )
        if not path:
            return
        try:
            with open(path, "r", encoding="utf-8") as handle:
                segments = json.load(handle)
            if not isinstance(segments, list):
                raise ValueError("计划必须是数组")
            for segment in segments:
                if not isinstance(segment, dict) or segment.get("kind") not in TrajectoryPlan.SUPPORTED:
                    raise ValueError("存在不支持的轨迹段")
            self.table.setRowCount(0)
            for segment in segments:
                self.append_segment(segment)
        except (OSError, ValueError, TypeError, json.JSONDecodeError) as error:
            self.warn("加载计划失败: %s" % error)

    def state_callback(self, message):
        self.connected = bool(message.connected)
        self.armed = bool(message.armed)
        self.mode = message.mode or "UNKNOWN"

    def pose_callback(self, message):
        q = message.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self.pose = {
            "x": message.pose.position.x,
            "y": message.pose.position.y,
            "z": message.pose.position.z,
            "yaw": yaw,
        }
        self.pose_valid = all(math.isfinite(value) for value in self.pose.values())

    def update_status(self):
        status = "已连接" if self.connected else "未连接"
        stream = "发送中" if self.streaming else "未发送"
        p = self.pose
        self.status_label.setText(
            "连接: %s | 模式: %s | 解锁: %s | %s | 阶段: %s | "
            "ENU: x=%.2f y=%.2f z=%.2f"
            % (status, self.mode, "是" if self.armed else "否", stream, self.phase,
               p["x"], p["y"], p["z"])
        )
        if self.last_connection and not self.connected and self.streaming:
            self.streaming = False
            self.phase = "idle"
            rospy.logerr("MAVROS 连接丢失，停止发送轨迹")
        if self.streaming and self.phase == "execute" and (
            self.mode != "OFFBOARD" or not self.armed
        ):
            self.hold_position()
            rospy.logwarn("已退出 OFFBOARD 或飞控未解锁，轨迹切换为保持")
        self.last_connection = self.connected

    def start_trajectory(self):
        if not self.connected:
            self.warn("MAVROS 尚未连接")
            return
        if not self.pose_valid:
            self.warn("尚未收到有效的本地位置")
            return
        segments = self.current_segments()
        if not segments:
            self.warn("轨迹计划为空")
            return
        try:
            self.plan = TrajectoryPlan(self.pose, segments)
        except (ValueError, TypeError) as error:
            self.warn("轨迹计划无效: %s" % error)
            return
        self.start_pose = dict(self.pose)
        self.phase = "prestream"
        self.phase_started = time.monotonic()
        self.streaming = True
        rospy.loginfo("轨迹计划已加载，共 %d 段，时长 %.2f 秒", len(segments), self.plan.duration)

    def hold_position(self):
        if not self.pose_valid and self.last_point is None:
            return
        self.hold_point = dict(self.pose if self.pose_valid else self.last_point)
        self.hold_point.update({"vx": 0.0, "vy": 0.0, "vz": 0.0})
        self.last_point = self.hold_point
        self.streaming = True
        self.phase = "hold"

    def publish_setpoint(self):
        if not self.streaming or not self.pose_valid:
            return
        now = time.monotonic()
        if self.phase == "prestream":
            point = dict(self.start_pose)
            point.update({"vx": 0.0, "vy": 0.0, "vz": 0.0})
            if now - self.phase_started >= 2.0:
                self.phase = "ready"
                rospy.loginfo("预发送完成，请切换 OFFBOARD 并解锁")
        elif self.phase == "ready":
            point = dict(self.start_pose)
            point.update({"vx": 0.0, "vy": 0.0, "vz": 0.0})
            if self.mode == "OFFBOARD" and self.armed:
                self.phase = "execute"
                self.plan_started = now
                rospy.loginfo("已进入 OFFBOARD 且已解锁，开始执行轨迹计划")
        elif self.phase == "execute":
            sampled, finished, _ = self.plan.sample(now - self.plan_started)
            point = sampled.__dict__.copy()
            if finished:
                self.phase = "hold"
                self.hold_point = point
                rospy.loginfo("轨迹计划执行完成，保持终点")
        elif self.phase == "hold":
            point = dict(self.hold_point or self.last_point or self.pose)
            point.update({"vx": 0.0, "vy": 0.0, "vz": 0.0})
        else:
            return

        self.last_point = dict(point)
        message = PositionTarget()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = "local_enu"
        message.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        message.type_mask = (
            PositionTarget.IGNORE_AFX
            | PositionTarget.IGNORE_AFY
            | PositionTarget.IGNORE_AFZ
            | PositionTarget.IGNORE_YAW_RATE
        )
        message.position.x = point["x"]
        message.position.y = point["y"]
        message.position.z = point["z"]
        message.velocity.x = point["vx"]
        message.velocity.y = point["vy"]
        message.velocity.z = point["vz"]
        message.yaw = point["yaw"]
        self.setpoint_pub.publish(message)

    @staticmethod
    def warn(message):
        rospy.logwarn(message)
        QtWidgets.QMessageBox.warning(None, "操作未执行", message)

    def closeEvent(self, event):
        self.streaming = False
        self.publish_timer.stop()
        rospy.signal_shutdown("ground station closed")
        event.accept()


def main():
    rospy.init_node("px4_ground_station", disable_signals=True)
    app = QtWidgets.QApplication(sys.argv)
    window = GroundStation()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
