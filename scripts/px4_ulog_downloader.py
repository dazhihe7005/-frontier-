#!/usr/bin/env python3
"""Download the PX4 ULog produced by each armed flight after disarm.

The downloader uses MAVROS' raw MAVLink log-transfer API.  It never requests
log data while the vehicle is armed, writes through a .part file, verifies the
reported size and ULog magic, and only then atomically publishes the .ulg file.
Failures are reported on a latched ROS status topic and never affect the flight
control launch.
"""

import datetime
import hashlib
import json
import os
import threading
import time

import rospy
from mavros_msgs.msg import LogData, LogEntry, State
from mavros_msgs.srv import LogRequestData, LogRequestEnd, LogRequestList
from std_msgs.msg import String


MAVLINK_LOG_PACKET_BYTES = 90
ULOG_MAGIC = b"ULog\x01\x12\x35"


def missing_ranges(received, limit, maximum_length):
    """Return missing byte ranges, bounded to safe MAVLink request blocks."""
    ranges = []
    cursor = 0
    while cursor < limit:
        if received[cursor]:
            cursor += 1
            continue
        start = cursor
        cursor += 1
        while (cursor < limit and not received[cursor]
               and cursor - start < maximum_length):
            cursor += 1
        ranges.append((start, cursor - start))
    return ranges


def ulog_filename(entry):
    stamp = entry.time_utc.to_sec()
    if stamp > 0:
        utc = datetime.datetime.fromtimestamp(
            stamp, datetime.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        return "px4_{}_id{}.ulg".format(utc, entry.id)
    return "px4_log_id{}.ulg".format(entry.id)


class Px4UlogDownloader:
    def __init__(self):
        self.output_directory = os.path.abspath(rospy.get_param(
            "~output_directory", "/tmp/px4-ulogs"))
        self.post_disarm_delay = max(
            0.0, float(rospy.get_param("~post_disarm_delay_s", 5.0)))
        self.block_bytes = max(
            MAVLINK_LOG_PACKET_BYTES,
            int(rospy.get_param("~request_block_bytes", 46080)))
        self.request_timeout = max(
            1.0, float(rospy.get_param("~request_timeout_s", 4.0)))
        self.max_retries = max(1, int(rospy.get_param("~max_retries", 8)))

        os.makedirs(self.output_directory, exist_ok=True)
        self.status_pub = rospy.Publisher(
            "/mine_uav/logging/px4_ulog_status", String,
            queue_size=10, latch=True)

        self.lock = threading.Condition()
        self.connected = False
        self.armed = False
        self.seen_armed = False
        self.download_running = False
        self.entries = {}
        self.entry_generation = 0
        self.active_id = None
        self.active_buffer = None
        self.active_received = None

        rospy.Subscriber("/mavros/state", State, self.on_state,
                         queue_size=20)
        rospy.Subscriber("/mavros/log_transfer/raw/log_entry", LogEntry,
                         self.on_log_entry, queue_size=100)
        rospy.Subscriber("/mavros/log_transfer/raw/log_data", LogData,
                         self.on_log_data, queue_size=1000)

        self.request_list = rospy.ServiceProxy(
            "/mavros/log_transfer/raw/log_request_list", LogRequestList)
        self.request_data = rospy.ServiceProxy(
            "/mavros/log_transfer/raw/log_request_data", LogRequestData)
        self.request_end = rospy.ServiceProxy(
            "/mavros/log_transfer/raw/log_request_end", LogRequestEnd)
        self.publish_status("waiting_for_armed_flight")

    def publish_status(self, state, **details):
        record = {
            "state": state,
            "wall_time": datetime.datetime.now(
                datetime.timezone.utc).isoformat(),
        }
        record.update(details)
        payload = json.dumps(record, sort_keys=True)
        self.status_pub.publish(payload)
        if state in ("failed", "incomplete"):
            rospy.logerr("PX4 ULog: %s", payload)
        else:
            rospy.loginfo("PX4 ULog: %s", payload)

    def on_state(self, message):
        launch_worker = False
        with self.lock:
            prior_armed = self.armed
            self.connected = message.connected
            self.armed = message.armed
            if message.armed:
                self.seen_armed = True
            if (prior_armed and not message.armed and self.seen_armed
                    and not self.download_running):
                self.download_running = True
                launch_worker = True
            self.lock.notify_all()
        if launch_worker:
            self.publish_status("disarmed_waiting_for_px4_flush")
            worker = threading.Thread(target=self.download_after_disarm)
            worker.daemon = True
            worker.start()

    def on_log_entry(self, message):
        with self.lock:
            self.entries[message.id] = message
            self.entry_generation += 1
            self.lock.notify_all()

    def on_log_data(self, message):
        with self.lock:
            if message.id != self.active_id or self.active_buffer is None:
                return
            start = int(message.offset)
            if start >= len(self.active_buffer):
                return
            data = bytes(message.data)
            end = min(len(self.active_buffer), start + len(data))
            if end <= start:
                return
            self.active_buffer[start:end] = data[:end - start]
            self.active_received[start:end] = b"\x01" * (end - start)
            self.lock.notify_all()

    def wait_for_services(self):
        for name in (
                "/mavros/log_transfer/raw/log_request_list",
                "/mavros/log_transfer/raw/log_request_data",
                "/mavros/log_transfer/raw/log_request_end"):
            rospy.wait_for_service(name, timeout=10.0)

    def still_safe_to_download(self):
        with self.lock:
            return self.connected and not self.armed and not rospy.is_shutdown()

    def collect_entries(self):
        with self.lock:
            self.entries.clear()
            before = self.entry_generation
        response = self.request_list(0, 0xFFFF)
        if not response.success:
            raise RuntimeError("PX4 rejected LOG_REQUEST_LIST")

        deadline = time.monotonic() + self.request_timeout
        last_generation = before
        quiet_since = time.monotonic()
        with self.lock:
            while not rospy.is_shutdown():
                now = time.monotonic()
                if self.entry_generation != last_generation:
                    last_generation = self.entry_generation
                    quiet_since = now
                expected = max(
                    (entry.num_logs for entry in self.entries.values()),
                    default=0)
                if expected and len(self.entries) >= expected:
                    break
                if self.entries and now - quiet_since >= 0.75:
                    break
                if now >= deadline:
                    break
                self.lock.wait(timeout=min(0.2, deadline - now))
            return list(self.entries.values())

    def request_range(self, log_id, offset, count):
        response = self.request_data(log_id, offset, count)
        if not response.success:
            raise RuntimeError(
                "PX4 rejected LOG_REQUEST_DATA id={} offset={} count={}".format(
                    log_id, offset, count))

        deadline = time.monotonic() + self.request_timeout
        with self.lock:
            previous = sum(self.active_received[offset:offset + count])
            quiet_deadline = time.monotonic() + 0.75
            while not rospy.is_shutdown():
                current = sum(self.active_received[offset:offset + count])
                if current >= count:
                    return
                now = time.monotonic()
                if current > previous:
                    previous = current
                    quiet_deadline = now + 0.75
                if now >= deadline or now >= quiet_deadline:
                    return
                self.lock.wait(timeout=min(0.1, deadline - now,
                                           quiet_deadline - now))

    def fetch_log(self, entry):
        if entry.size < len(ULOG_MAGIC):
            raise RuntimeError("PX4 reported an invalid ULog size: {}".format(
                entry.size))

        with self.lock:
            self.active_id = entry.id
            self.active_buffer = bytearray(entry.size)
            self.active_received = bytearray(entry.size)

        attempts = 0
        while attempts < self.max_retries:
            if not self.still_safe_to_download():
                raise RuntimeError("vehicle re-armed or MAVROS disconnected")
            with self.lock:
                gaps = missing_ranges(self.active_received, entry.size,
                                      self.block_bytes)
            if not gaps:
                break
            attempts += 1
            missing_before = sum(length for _, length in gaps)
            self.publish_status(
                "downloading", log_id=entry.id, size=entry.size,
                attempt=attempts, missing_bytes=missing_before)
            for offset, count in gaps:
                if not self.still_safe_to_download():
                    raise RuntimeError("vehicle re-armed or MAVROS disconnected")
                self.request_range(entry.id, offset, count)

        with self.lock:
            remaining = self.active_received.count(0)
            payload = bytes(self.active_buffer)
            self.active_id = None
            self.active_buffer = None
            self.active_received = None
        try:
            self.request_end()
        except rospy.ServiceException as error:
            rospy.logwarn("PX4 ULog LOG_REQUEST_END failed: %s", error)
        if remaining:
            raise RuntimeError(
                "download incomplete after {} attempts: {} bytes missing".format(
                    attempts, remaining))
        if not payload.startswith(ULOG_MAGIC):
            raise RuntimeError("downloaded data does not have a ULog header")
        return payload

    def store_log(self, entry, payload):
        filename = ulog_filename(entry)
        destination = os.path.join(self.output_directory, filename)
        partial = destination + ".part"
        digest = hashlib.sha256(payload).hexdigest()
        existed = False
        if (os.path.exists(destination)
                and os.path.getsize(destination) == entry.size):
            with open(destination, "rb") as stream:
                existing = stream.read()
            existed = (existing.startswith(ULOG_MAGIC)
                       and hashlib.sha256(existing).hexdigest() == digest)
        if not existed:
            try:
                with open(partial, "wb") as stream:
                    stream.write(payload)
                    stream.flush()
                    os.fsync(stream.fileno())
                os.replace(partial, destination)
            finally:
                if os.path.exists(partial):
                    os.unlink(partial)

        checksum_path = destination + ".sha256"
        checksum_partial = checksum_path + ".part"
        try:
            with open(checksum_partial, "w", encoding="ascii") as stream:
                stream.write("{}  {}\n".format(digest, filename))
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(checksum_partial, checksum_path)
        finally:
            if os.path.exists(checksum_partial):
                os.unlink(checksum_partial)
        return destination, existed, digest

    def download_after_disarm(self):
        try:
            deadline = time.monotonic() + self.post_disarm_delay
            while time.monotonic() < deadline and not rospy.is_shutdown():
                if not self.still_safe_to_download():
                    raise RuntimeError(
                        "vehicle re-armed or MAVROS disconnected during flush wait")
                time.sleep(min(0.2, deadline - time.monotonic()))
            self.wait_for_services()
            if not self.still_safe_to_download():
                raise RuntimeError("vehicle is not safely disarmed and connected")
            entries = self.collect_entries()
            if not entries:
                raise RuntimeError("PX4 returned no log entries")
            entry = max(
                entries, key=lambda item: (item.time_utc.to_sec(), item.id))
            payload = self.fetch_log(entry)
            destination, existed, digest = self.store_log(entry, payload)
            self.publish_status(
                "complete", log_id=entry.id, size=entry.size,
                path=destination, sha256=digest, already_present=existed)
        except Exception as error:  # keep all logging failures non-fatal
            self.publish_status("failed", error=str(error))
        finally:
            with self.lock:
                self.download_running = False
                self.seen_armed = self.armed
                self.active_id = None
                self.active_buffer = None
                self.active_received = None


def main():
    rospy.init_node("px4_ulog_downloader")
    Px4UlogDownloader()
    rospy.spin()


if __name__ == "__main__":
    main()
