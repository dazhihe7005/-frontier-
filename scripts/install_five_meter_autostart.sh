#!/usr/bin/env bash
set -euo pipefail

readonly SOURCE_UNIT=/home/nuc/frontier-upload/systemd/mine-uav-five-meter.service
readonly TARGET_UNIT=/etc/systemd/system/mine-uav-five-meter.service

# Installation enables the next-boot start but deliberately does not start the
# physical flight stack in the current session.
sudo install -o root -g root -m 0644 "${SOURCE_UNIT}" "${TARGET_UNIT}"
sudo systemctl daemon-reload
sudo systemctl enable mine-uav-five-meter.service

echo "enabled mine-uav-five-meter.service for the next boot"
echo "it has not been started in the current session"
