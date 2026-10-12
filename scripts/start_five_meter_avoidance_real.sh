#!/usr/bin/env bash
set -euo pipefail

# Boot entry point for the physical five-metre test.  Starting this script does
# not arm PX4, take off, or generate a goal.  The test node still requires a
# fresh, debounced CH7 low-to-high edge after launch and all normal safety gates.

readonly PROJECT_DIR=/home/nuc/frontier-upload
readonly LOG_ROOT=/home/nuc/flight_logs/five_meter
readonly RADAR_INTERFACE=enp89s0
readonly RADAR_ADDRESS=192.168.1.10/24
readonly MIN_FREE_KIB=$((100 * 1024 * 1024))

find_px4_serial() {
  local candidate name resolved
  local -a serial_candidates=()
  local -a px4_candidates=()

  shopt -s nullglob
  for candidate in /dev/serial/by-id/*; do
    resolved=$(readlink -f "${candidate}" 2>/dev/null || true)
    case "${resolved}" in
      /dev/ttyACM*|/dev/ttyUSB*) ;;
      *) continue ;;
    esac

    serial_candidates+=("${candidate}")
    name=${candidate##*/}
    name=${name,,}
    case "${name}" in
      *px4*|*pixhawk*|*auterion*|*holybro*|*cube*)
        px4_candidates+=("${candidate}")
        ;;
    esac
  done
  shopt -u nullglob

  if (( ${#px4_candidates[@]} == 1 )); then
    printf '%s\n' "${px4_candidates[0]}"
    return 0
  fi
  if (( ${#px4_candidates[@]} > 1 )); then
    printf 'multiple PX4 serial devices found; waiting for an unambiguous device: %s\n' \
      "${px4_candidates[*]}" >&2
    return 1
  fi
  if (( ${#serial_candidates[@]} == 1 )); then
    printf '%s\n' "${serial_candidates[0]}"
    return 0
  fi
  if (( ${#serial_candidates[@]} > 1 )); then
    printf 'multiple generic serial devices found; cannot safely identify PX4: %s\n' \
      "${serial_candidates[*]}" >&2
  else
    echo 'waiting for a PX4 serial device under /dev/serial/by-id' >&2
  fi
  return 1
}

exec 9>/run/lock/mine-uav-five-meter.lock
if ! flock -n 9; then
  echo "five-metre stack is already running; refusing a duplicate launch" >&2
  exit 1
fi

# Catkin's generated setup files probe variables such as ROS_DISTRO before
# defining them.  Temporarily relax nounset while loading the environment, then
# restore the script's strict error handling for the actual launcher logic.
set +u
source /opt/ros/noetic/setup.bash
source /home/nuc/fastlio2_ws/devel/setup.bash --extend
source /home/nuc/super_ws/devel/setup.bash --extend
source /home/nuc/frontier_runtime_ws/devel/setup.bash --extend
set -u

export CMAKE_PREFIX_PATH=/home/nuc/frontier_runtime_ws/devel:/home/nuc/super_ws/devel:/home/nuc/fastlio2_ws/devel:/opt/ros/noetic
export ROS_PACKAGE_PATH=/home/nuc/frontier_runtime_ws/src:/home/nuc/super_ws/src/SUPER:/home/nuc/fastlio2_ws/src:/opt/ros/noetic/share
export PYTHONPATH=/home/nuc/frontier_runtime_ws/devel/lib/python3/dist-packages:/home/nuc/super_ws/devel/lib/python3/dist-packages:/home/nuc/fastlio2_ws/devel/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages
export LD_LIBRARY_PATH=/home/nuc/frontier_runtime_ws/devel/lib:/home/nuc/super_ws/devel/lib:/home/nuc/fastlio2_ws/devel/lib:/opt/ros/noetic/lib
export ROS_MASTER_URI=http://127.0.0.1:11312
export ROS_HOSTNAME=127.0.0.1

# Hardware may be powered a little later than the NUC.  Prefer a stable by-id
# name that identifies PX4 explicitly.  A single generic USB serial adapter is
# accepted for the existing CH340/TELEM setup, but ambiguity is never resolved
# by choosing an arbitrary tty.  Waiting here leaves the service inert: no ROS
# flight/control node exists until both prerequisites are present.
PX4_SERIAL=
until PX4_SERIAL=$(find_px4_serial); do
  sleep 2
done
readonly PX4_SERIAL
echo "using PX4 serial device ${PX4_SERIAL} -> $(readlink -f "${PX4_SERIAL}")"
until ip -4 address show dev "${RADAR_INTERFACE}" 2>/dev/null | grep -Fq "${RADAR_ADDRESS}"; do
  echo "waiting for ${RADAR_INTERFACE} address ${RADAR_ADDRESS}"
  sleep 2
done

mkdir -p "${LOG_ROOT}"
available_kib=$(df -Pk "${LOG_ROOT}" | awk 'NR == 2 {print $4}')
if [[ ! "${available_kib}" =~ ^[0-9]+$ ]] || (( available_kib < MIN_FREE_KIB )); then
  echo "less than 100 GiB is available for flight logs; refusing to start" >&2
  exit 1
fi

run_stamp=$(date +%Y%m%d_%H%M%S)
run_dir=${LOG_ROOT}/${run_stamp}
mkdir -p "${run_dir}/config"

{
  echo "start_time=$(date --iso-8601=seconds)"
  echo "hostname=$(hostname)"
  echo "kernel=$(uname -srmo)"
  echo "ros_master_uri=${ROS_MASTER_URI}"
  echo "project_commit=$(git -C "${PROJECT_DIR}" rev-parse HEAD 2>/dev/null || echo unknown)"
  echo "px4_serial=${PX4_SERIAL}"
  echo "radar_interface=${RADAR_INTERFACE}"
  echo "radar_address=${RADAR_ADDRESS}"
  df -h "${LOG_ROOT}"
  git -C "${PROJECT_DIR}" status --short 2>/dev/null || true
} >"${run_dir}/startup_metadata.txt"

cp -a "${PROJECT_DIR}/config/five_meter_avoidance_test.yaml" \
      "${PROJECT_DIR}/config/super_task1.yaml" \
      "${PROJECT_DIR}/config/super_px4_command_bridge.yaml" \
      "${PROJECT_DIR}/launch/five_meter_avoidance_real.launch" \
      "${run_dir}/config/"

# roslaunch starts the master.  Take a resolved parameter/topic snapshot after
# the graph has had time to settle; failure to collect metadata is non-fatal.
(
  for _ in $(seq 1 60); do
    if rosparam list >/dev/null 2>&1; then
      sleep 10
      rosparam dump "${run_dir}/rosparams.yaml" >/dev/null 2>&1 || true
      rostopic list -v >"${run_dir}/topics.txt" 2>&1 || true
      rosnode list >"${run_dir}/nodes.txt" 2>&1 || true
      exit 0
    fi
    sleep 1
  done
) &

cd "${PROJECT_DIR}"
exec roslaunch "${PROJECT_DIR}/launch/five_meter_avoidance_real.launch" \
  rviz:=false \
  record_flight:=true \
  px4_device:="${PX4_SERIAL}" \
  flight_log_dir:="${run_dir}"
