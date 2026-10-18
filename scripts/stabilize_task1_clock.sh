#!/usr/bin/env bash
set -euo pipefail

# ROS 1 stamps use CLOCK_REALTIME.  systemd-timesyncd is allowed to step that
# clock during its first successful synchronization, which invalidates
# Fast-LIO/PX4 queues if it happens after takeoff.  Give initial sync a bounded
# opportunity, then freeze wall time for the lifetime of the flight service.

readonly WAIT_SECONDS=60

if systemctl is-active --quiet systemd-timesyncd.service; then
  deadline=$((SECONDS + WAIT_SECONDS))
  while (( SECONDS < deadline )); do
    if [[ "$(timedatectl show -p NTPSynchronized --value 2>/dev/null || true)" == "yes" ]]; then
      echo 'Task-1 clock guard: initial NTP synchronization complete'
      break
    fi
    sleep 1
  done

  if [[ "$(timedatectl show -p NTPSynchronized --value 2>/dev/null || true)" != "yes" ]]; then
    echo "Task-1 clock guard: no NTP lock after ${WAIT_SECONDS}s; using the RTC/system time without permitting a late in-flight step" >&2
  fi
  systemctl stop systemd-timesyncd.service
fi

if systemctl is-active --quiet systemd-timesyncd.service; then
  echo 'Task-1 clock guard: failed to stop systemd-timesyncd' >&2
  exit 1
fi

echo 'Task-1 clock guard: wall clock frozen against first-sync steps'
