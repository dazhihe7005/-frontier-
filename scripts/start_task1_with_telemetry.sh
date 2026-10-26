#!/usr/bin/env bash
set -euo pipefail

# Systemd entry point for the physical aircraft.  Telemetry is deliberately
# started before the hardware-gated flight wrapper so the downstream CH340 can
# come online and exchange Task-3/mission data while LiDAR, camera or Ethernet
# are still absent.  The later task1_real launch must not start a second sender.

readonly PROJECT_DIR=/home/nuc/frontier-upload
TELEMETRY_LAUNCH_PID=
FLIGHT_WRAPPER_PID=

cleanup() {
  local pid
  for pid in "${FLIGHT_WRAPPER_PID}" "${TELEMETRY_LAUNCH_PID}"; do
    if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
      kill -INT "${pid}" 2>/dev/null || true
    fi
  done
  for pid in "${FLIGHT_WRAPPER_PID}" "${TELEMETRY_LAUNCH_PID}"; do
    if [[ -n "${pid}" ]]; then
      wait "${pid}" 2>/dev/null || true
    fi
  done
}

on_signal() {
  trap - INT TERM
  cleanup
  exit 130
}

trap on_signal INT TERM
trap cleanup EXIT

set +u
source /opt/ros/noetic/setup.bash
source /home/nuc/super_ws/devel/setup.bash --extend
source /home/nuc/frontier_runtime_ws/devel/setup.bash --extend
set -u

export ROS_MASTER_URI=http://127.0.0.1:11312
export ROS_HOSTNAME=127.0.0.1

# The independent uav-ros-master.service owns ROS; this wrapper must not own or terminate it.
# The sender itself retries CH340 discovery/opening, so no serial-device wait is
# needed here and hot-plug after boot is supported.
roslaunch --wait nuc_mavlink_sender nuc_mavlink_sender.launch \
  output_device:=auto \
  auto_discover_output:=true \
  prefer_odom:=false \
  respawn_sender:=true &
TELEMETRY_LAUNCH_PID=$!

# Do not let the flight wrapper race roslaunch's master startup.  Failure of
# this independent preflight service is fatal because otherwise Task 3 would
# silently lose pressure RX and all tasks would lose outbound telemetry.
master_ready=false
for _ in $(seq 1 50); do
  if ! kill -0 "${TELEMETRY_LAUNCH_PID}" 2>/dev/null; then
    wait "${TELEMETRY_LAUNCH_PID}" || true
    echo 'telemetry roslaunch exited before the ROS master became ready' >&2
    exit 1
  fi
  if rosparam list >/dev/null 2>&1; then
    master_ready=true
    break
  fi
  sleep 0.1
done
if [[ "${master_ready}" != true ]]; then
  echo 'telemetry ROS master did not become ready within 5 seconds' >&2
  exit 1
fi

"${PROJECT_DIR}/scripts/start_task1_real.sh" &
FLIGHT_WRAPPER_PID=$!

# Supervise both launch owners.  The sender node has roslaunch respawn enabled;
# reaching this loop with its roslaunch parent gone is therefore a critical
# startup failure, as is termination of the physical flight wrapper.
while true; do
  if ! kill -0 "${TELEMETRY_LAUNCH_PID}" 2>/dev/null; then
    set +e
    wait "${TELEMETRY_LAUNCH_PID}"
    telemetry_status=$?
    set -e
    if (( telemetry_status == 0 )); then
      telemetry_status=1
    fi
    echo "telemetry roslaunch exited unexpectedly (status ${telemetry_status})" >&2
    exit "${telemetry_status}"
  fi
  if ! kill -0 "${FLIGHT_WRAPPER_PID}" 2>/dev/null; then
    set +e
    wait "${FLIGHT_WRAPPER_PID}"
    flight_status=$?
    set -e
    FLIGHT_WRAPPER_PID=
    if (( flight_status == 0 )); then
      echo 'physical flight wrapper exited unexpectedly' >&2
      flight_status=1
    fi
    exit "${flight_status}"
  fi
  sleep 1
done
