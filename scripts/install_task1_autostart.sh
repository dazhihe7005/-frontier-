#!/usr/bin/env bash
set -euo pipefail

readonly SOURCE_UNIT=/home/nuc/frontier-upload/systemd/mine-uav-task1.service
readonly TARGET_UNIT=/etc/systemd/system/mine-uav-task1.service

# Install and enable for the next boot only. A prior deployment may have used
# `systemctl link`, in which case SOURCE_UNIT and TARGET_UNIT resolve to the
# same file and copying would fail. Keep that valid registration in place.
if [[ "$(readlink -f "${TARGET_UNIT}" 2>/dev/null || true)" != "${SOURCE_UNIT}" ]]; then
  sudo install -o root -g root -m 0644 "${SOURCE_UNIT}" "${TARGET_UNIT}"
fi
sudo systemctl daemon-reload
sudo systemctl disable mine-uav-five-meter.service >/dev/null 2>&1 || true
sudo systemctl enable mine-uav-task1.service

echo 'enabled mine-uav-task1.service for the next boot'
echo 'disabled mine-uav-five-meter.service for the next boot'
echo 'enabled the preflight clock guard; NTP is restored when Task-1 stops'
echo 'no physical flight stack was started or stopped in this session'
