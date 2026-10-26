#!/usr/bin/env bash
set -euo pipefail

# Production entry point for the physical Task-1/Task-4 stack. This script records a
# complete run but never arms PX4; task trigger, stable-hover, goal-acceptance and bridge
# safety gates remain responsible for granting autonomous authority.

readonly PROJECT_DIR=/home/nuc/frontier-upload
readonly LOG_ROOT=/home/nuc/flight_logs/task1
readonly RADAR_INTERFACE=enp89s0
readonly RADAR_ADDRESS=192.168.1.10/24
readonly RADAR_DEVICE_IP=192.168.1.157
readonly SEEKER_USB_VENDOR=2207
readonly SEEKER_USB_PRODUCT=0000
readonly SYNC_USB_VENDOR=1209
readonly SYNC_USB_PRODUCT=8888
readonly TASK3_USB_VENDOR=1a86
readonly TASK3_USB_PRODUCTS="7523 7522 5523 e523"
readonly MIN_FREE_KIB=$((80 * 1024 * 1024))
readonly OMNI_WORKSPACE=/home/nuc/omni_livo_fisheye_ws
readonly OMNI_CALIBRATION=${OMNI_WORKSPACE}/installation_calibration.yaml
readonly OMNI_CONFIG_GENERATOR=${OMNI_WORKSPACE}/tools/make_raw_config.py
readonly SYNC_CONTROL=${OMNI_WORKSPACE}/tools/desktop/sync_control.py
ROSLAUNCH_PID=
SYNC_SERIAL=
SYNC_STARTED=false

cleanup() {
  if [[ -n "${ROSLAUNCH_PID}" ]] && kill -0 "${ROSLAUNCH_PID}" 2>/dev/null; then
    kill -INT "${ROSLAUNCH_PID}" 2>/dev/null || true
    wait "${ROSLAUNCH_PID}" 2>/dev/null || true
  fi
  if [[ "${SYNC_STARTED}" == true && -n "${SYNC_SERIAL}" ]]; then
    python3 "${SYNC_CONTROL}" stop --port "${SYNC_SERIAL}" >/dev/null 2>&1 || true
    SYNC_STARTED=false
  fi
}

on_signal() {
  trap - INT TERM
  cleanup
  exit 130
}

# systemd sends SIGINT for a graceful ROS shutdown. The wrapper owns both the
# roslaunch child and the camera/LiDAR synchronization session.
trap on_signal INT TERM

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
    printf 'multiple PX4 serial devices found: %s\n' \
      "${px4_candidates[*]}" >&2
    return 1
  fi
  if (( ${#serial_candidates[@]} == 1 )); then
    printf '%s\n' "${serial_candidates[0]}"
    return 0
  fi
  if (( ${#serial_candidates[@]} > 1 )); then
    printf 'multiple generic serial devices found: %s\n' \
      "${serial_candidates[*]}" >&2
  else
    echo 'waiting for a PX4 serial device under /dev/serial/by-id' >&2
  fi
  return 1
}

find_sync_serial() {
  local px4_real candidate resolved vendor product
  local -a candidates=()
  px4_real=$(readlink -f "${PX4_SERIAL}")
  if [[ -n "${OMNI_SYNC_PORT:-}" ]]; then
    resolved=$(readlink -f "${OMNI_SYNC_PORT}" 2>/dev/null || true)
    if [[ -z "${resolved}" || ! -e "${resolved}" ]]; then
      echo "configured OMNI_SYNC_PORT does not exist: ${OMNI_SYNC_PORT}" >&2
      return 1
    fi
    if [[ "${resolved}" == "${px4_real}" ]]; then
      echo 'OMNI_SYNC_PORT resolves to the PX4 serial device; refusing' >&2
      return 1
    fi
    printf '%s\n' "${OMNI_SYNC_PORT}"
    return 0
  fi

  shopt -s nullglob
  for candidate in /dev/ttyACM* /dev/ttyUSB*; do
    resolved=$(readlink -f "${candidate}" 2>/dev/null || true)
    [[ -n "${resolved}" && "${resolved}" != "${px4_real}" ]] || continue
    vendor=$(udevadm info --query=property --name="${resolved}" 2>/dev/null | \
      awk -F= '$1 == "ID_VENDOR_ID" {print tolower($2); exit}')
    product=$(udevadm info --query=property --name="${resolved}" 2>/dev/null | \
      awk -F= '$1 == "ID_MODEL_ID" {print tolower($2); exit}')
    [[ "${vendor}" == "${SYNC_USB_VENDOR}" &&
       "${product}" == "${SYNC_USB_PRODUCT}" ]] || continue
    candidates+=("${candidate}")
  done
  shopt -u nullglob
  if (( ${#candidates[@]} == 1 )); then
    printf '%s\n' "${candidates[0]}"
    return 0
  fi
  if (( ${#candidates[@]} > 1 )); then
    echo "multiple ${SYNC_USB_VENDOR}:${SYNC_USB_PRODUCT} synchronization devices found: ${candidates[*]}" >&2
  else
    echo "waiting for Omni-LIVO synchronization USB ${SYNC_USB_VENDOR}:${SYNC_USB_PRODUCT}" >&2
  fi
  return 1
}

seeker_usb_present() {
  local device vendor product

  shopt -s nullglob
  for device in /sys/bus/usb/devices/*; do
    [[ -r "${device}/idVendor" && -r "${device}/idProduct" ]] || continue
    IFS= read -r vendor < "${device}/idVendor" || continue
    IFS= read -r product < "${device}/idProduct" || continue
    if [[ "${vendor,,}" == "${SEEKER_USB_VENDOR}" &&
          "${product,,}" == "${SEEKER_USB_PRODUCT}" ]]; then
      shopt -u nullglob
      return 0
    fi
  done
  shopt -u nullglob
  return 1
}

task3_ch340_present() {
  local device vendor product expected count
  count=0

  shopt -s nullglob
  for device in /sys/bus/usb/devices/*; do
    [[ -r "${device}/idVendor" && -r "${device}/idProduct" ]] || continue
    IFS= read -r vendor < "${device}/idVendor" || continue
    IFS= read -r product < "${device}/idProduct" || continue
    [[ "${vendor,,}" == "${TASK3_USB_VENDOR}" ]] || continue
    for expected in ${TASK3_USB_PRODUCTS}; do
      if [[ "${product,,}" == "${expected}" ]]; then
        ((count += 1))
        break
      fi
    done
  done
  shopt -u nullglob
  # The telemetry relay refuses ambiguity as well.  Keep the flight graph
  # behind the same gate so Task 3 can never be attempted without exactly one
  # physical pressure/telemetry UART.
  (( count == 1 ))
}

# One physical aircraft must never run the five-metre and formal Task-1 graphs
# together, even if both systemd units were accidentally enabled.
exec 9>/run/lock/mine-uav-flight-stack.lock
if ! flock -n 9; then
  echo 'another physical Mine-UAV flight stack is already running' >&2
  exit 1
fi

set +u
source /opt/ros/noetic/setup.bash
source /home/nuc/seeker_ws/devel/setup.bash --extend
source /home/nuc/catkin_ws/devel/setup.bash --extend
source /home/nuc/omni_livo_fisheye_ws/devel/setup.bash --extend
source /home/nuc/super_ws/devel/setup.bash --extend
source /home/nuc/task3_ws/devel/setup.bash --extend
source /home/nuc/frontier_runtime_ws/devel/setup.bash --extend
set -u

export ROS_MASTER_URI=http://127.0.0.1:11312
export ROS_HOSTNAME=127.0.0.1
export XDG_RUNTIME_DIR=/run/user/$(id -u)
export OMP_NUM_THREADS=4 OPENBLAS_NUM_THREADS=1 OMP_WAIT_POLICY=PASSIVE GOMP_SPINCOUNT=0
if systemctl --user is-active --quiet uav-localization.service; then
  echo "Legacy localization already owns the sensors; refusing duplicate startup" >&2
  exit 1
fi

PX4_SERIAL=
until PX4_SERIAL=$(find_px4_serial); do
  sleep 2
done
readonly PX4_SERIAL
echo "using PX4 serial device ${PX4_SERIAL} -> $(readlink -f "${PX4_SERIAL}")"
until SYNC_SERIAL=$(find_sync_serial); do
  sleep 2
done
readonly SYNC_SERIAL
echo "using Omni-LIVO synchronization device ${SYNC_SERIAL} -> $(readlink -f "${SYNC_SERIAL}")"
# Camera and Task-3 UART availability cannot gate flight localization.
until ip -4 address show dev "${RADAR_INTERFACE}" 2>/dev/null | \
    grep -Fq "${RADAR_ADDRESS}"; do
  echo "waiting for ${RADAR_INTERFACE} address ${RADAR_ADDRESS}"
  sleep 2
done
until ping -c 1 -W 1 "${RADAR_DEVICE_IP}" >/dev/null 2>&1; do
  echo "waiting for Livox MID360S at ${RADAR_DEVICE_IP}"
  sleep 2
done

mkdir -p "${LOG_ROOT}"
RECORD_FLIGHT=true
available_kib=$(df -Pk "${LOG_ROOT}" | awk 'NR == 2 {print $4}')
if [[ ! "${available_kib}" =~ ^[0-9]+$ ]] ||
   (( available_kib < MIN_FREE_KIB )); then
  echo 'less than 80 GiB available: keep localization online without flight recording' >&2
  RECORD_FLIGHT=false
fi

run_stamp=$(date +%Y%m%d_%H%M%S)
run_dir=${LOG_ROOT}/${run_stamp}
mkdir -p "${run_dir}/config"
readonly OMNI_RUNTIME_CONFIG=${run_dir}/config/omnilivo_runtime.yaml

python3 "${OMNI_CONFIG_GENERATOR}" \
  "${OMNI_CALIBRATION}" "${OMNI_RUNTIME_CONFIG}"
python3 "/home/nuc/uav_modes/releases/continuous_pose_20261025_01/configure_core.py" "${OMNI_RUNTIME_CONFIG}"
python3 "${SYNC_CONTROL}" start --port "${SYNC_SERIAL}"
SYNC_STARTED=true
trap cleanup EXIT

{
  echo "start_time=$(date --iso-8601=seconds)"
  echo "hostname=$(hostname)"
  echo "kernel=$(uname -srmo)"
  echo "ros_master_uri=${ROS_MASTER_URI}"
  echo "project_commit=$(git -C "${PROJECT_DIR}" rev-parse HEAD 2>/dev/null || echo unknown)"
  echo "px4_serial=${PX4_SERIAL}"
  echo "omni_sync_serial=${SYNC_SERIAL}"
  echo "omni_sync_usb=${SYNC_USB_VENDOR}:${SYNC_USB_PRODUCT}"
  echo "seeker_usb=${SEEKER_USB_VENDOR}:${SEEKER_USB_PRODUCT}"
  echo "radar_interface=${RADAR_INTERFACE}"
  echo "radar_address=${RADAR_ADDRESS}"
  echo "radar_device_ip=${RADAR_DEVICE_IP}"
  echo 'speed_contract=super_nominal_0.9_measured_soft_1.0_predicted_and_hard_1.1_mps'
  df -h "${LOG_ROOT}"
  git -C "${PROJECT_DIR}" status --short 2>/dev/null || true
  git -C /home/nuc/super_ws/src/SUPER status --short 2>/dev/null || true
} >"${run_dir}/startup_metadata.txt"

cp -a "${PROJECT_DIR}/config/omnilivo_px4_vision_bridge.yaml" \
      "${PROJECT_DIR}/config/livox_mid360s_real.json" \
      "${PROJECT_DIR}/config/mission_scheduler.yaml" \
      "${PROJECT_DIR}/config/shaft_mission_logic.yaml" \
      "${PROJECT_DIR}/config/assisted_task_px4_router.yaml" \
      "${PROJECT_DIR}/config/super_exploration_decider.yaml" \
      "${PROJECT_DIR}/config/super_task1.yaml" \
      "${PROJECT_DIR}/config/super_px4_command_bridge.yaml" \
      "${PROJECT_DIR}/launch/task1_real.launch" \
      "${run_dir}/config/"
cp -a "${OMNI_CALIBRATION}" "${run_dir}/config/"
cp -a "/home/nuc/task3_ws/src/mine_uav_task3/config/task3_pressure_control.yaml" \
      "${run_dir}/config/"

(
  for _ in $(seq 1 60); do
    if rosparam list >/dev/null 2>&1; then
      sleep 10
      rosparam dump "${run_dir}/rosparams.yaml" >/dev/null 2>&1 || true
      rostopic list -v >"${run_dir}/topics.txt" 2>&1 || true
      rosnode list >"${run_dir}/nodes.txt" 2>&1 || true
      # Keep the FCU-side parameter set beside the ROS configuration. This is
      # diagnostic only and must never make the flight/control graph exit.
      rosrun mavros mavparam dump -f "${run_dir}/px4_params.yaml" \
        >"${run_dir}/px4_param_dump.log" 2>&1 || true
      exit 0
    fi
    sleep 1
  done
) &

cd "${PROJECT_DIR}"
roslaunch "${PROJECT_DIR}/launch/task1_real.launch" \
  rviz:=false \
  start_seeker:=false \
  record_flight:="${RECORD_FLIGHT}" \
  px4_device:="${PX4_SERIAL}" \
  flight_log_dir:="${run_dir}" \
  omnilivo_runtime_config:="${OMNI_RUNTIME_CONFIG}" &
ROSLAUNCH_PID=$!
set +e
wait "${ROSLAUNCH_PID}"
launch_status=$?
set -e
ROSLAUNCH_PID=
# roslaunch returns zero even when a required node dies and it tears down the
# graph. This production wrapper is expected to run until systemd/Ctrl+C sends
# us a signal (handled above), so reaching this point with status zero is still
# an unexpected critical-stack termination and must leave the unit failed.
if (( launch_status == 0 )); then
  echo 'task1_real roslaunch exited unexpectedly; marking flight stack failed' >&2
  launch_status=1
fi
cleanup
exit "${launch_status}"
