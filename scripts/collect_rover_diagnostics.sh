#!/usr/bin/env bash
set -Eeuo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS_DIR="${ROOT_DIR}/ros2_ws"
ROS_SETUP="/opt/ros/jazzy/setup.bash"
WS_SETUP="${WS_DIR}/install/setup.bash"
OUT_BASE="${1:-${HOME}/field_logs}"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT_DIR="${OUT_BASE}/rover_diag_${STAMP}"

mkdir -p "${OUT_DIR}"

source_safe() {
  local path="$1"
  local had_u=0
  case $- in
    *u*) had_u=1 ;;
  esac
  set +u
  # shellcheck disable=SC1090
  source "${path}"
  if [[ ${had_u} -eq 1 ]]; then
    set -u
  fi
}

timestamp_utc() {
  date -u +"%Y-%m-%dT%H:%M:%SZ"
}

run_sh() {
  local name="$1"
  local cmd="$2"
  {
    echo "# ${name}"
    echo "# command: ${cmd}"
    echo "# time: $(timestamp_utc)"
    echo
    bash -lc "${cmd}"
  } >"${OUT_DIR}/${name}.txt" 2>&1 || true
}

note() {
  echo "$1" | tee -a "${OUT_DIR}/SUMMARY.txt" >/dev/null
}

note "Rover diagnostics started: $(timestamp_utc)"
note "Output dir: ${OUT_DIR}"

run_sh "sys_info" "uname -a; echo; uptime; echo; whoami; echo; id"
run_sh "devices" "ls -l /dev/pps* /dev/ttyAMA* 2>/dev/null; echo; ls -l /dev/serial/by-id/ 2>/dev/null || true"
run_sh "pps_sysfs" 'for p in /sys/class/pps/pps*; do echo "== $(basename "$p") =="; cat "$p/name"; done'
run_sh "boot_pps_overlay" "grep -n 'dtoverlay=pps-gpio' /boot/firmware/config.txt 2>/dev/null || true"
run_sh "gpsd_default_cfg" "grep -E 'START_DAEMON|USBAUTO|DEVICES|GPSD_OPTIONS' /etc/default/gpsd 2>/dev/null || true"
run_sh "gpsd_status" "systemctl --no-pager --full status gpsd.socket gpsd.service 2>/dev/null || true"
run_sh "chrony_status" "systemctl --no-pager --full status chrony.service 2>/dev/null || true"

run_sh "gpspipe_json" "timeout 10s gpspipe -w -n 30"
run_sh "uart_nmea" "timeout 8s cat /dev/ttyAMA0 | grep -E 'RMC|GGA|ZDA'"
run_sh "chrony_sources" "chronyc sources -v"
run_sh "chrony_tracking" "chronyc tracking"

if [[ -e /dev/pps0 ]]; then
  if sudo -n true >/dev/null 2>&1; then
    run_sh "ppstest_pps0" "sudo timeout 8s ppstest /dev/pps0"
  else
    note "Skipping ppstest in script (sudo password required)."
    note "Manual command: sudo timeout 8s ppstest /dev/pps0"
  fi
fi

ROS_READY="false"
if [[ -f "${ROS_SETUP}" && -f "${WS_SETUP}" ]]; then
  source_safe "${ROS_SETUP}"
  source_safe "${WS_SETUP}"
  ROS_READY="true"
else
  note "ROS environment not sourced (missing ${ROS_SETUP} or ${WS_SETUP})."
fi

if [[ "${ROS_READY}" == "true" ]]; then
  run_sh "ros_nodes" "ros2 node list"
  run_sh "ros_topics" "ros2 topic list"
  run_sh "ros_topic_info_fix" "ros2 topic info /fix -v"
  run_sh "ros_topic_info_time_reference" "ros2 topic info /time_reference -v"
  run_sh "ros_topic_info_imu" "ros2 topic info /imu/data -v"
  run_sh "ros_topic_info_odom_local" "ros2 topic info /odometry/local -v"
  run_sh "ros_topic_info_odom_global" "ros2 topic info /odometry/global -v"
  run_sh "ros_echo_fix_once" "timeout 8s ros2 topic echo /fix --once"
  run_sh "ros_echo_time_reference_once" "timeout 8s ros2 topic echo /time_reference --once"
  run_sh "ros_echo_imu_once" "timeout 8s ros2 topic echo /imu/data --once"
  run_sh "ros_echo_capture_debug_once" "timeout 8s ros2 topic echo /capture/debug --once"
  run_sh "ros_echo_capture_event_once" "timeout 8s ros2 topic echo /capture/events --once"
  run_sh "ros_hz_fix" "timeout 8s ros2 topic hz /fix"
  run_sh "ros_hz_imu" "timeout 8s ros2 topic hz /imu/data"
fi

CAP_DIR="${HOME}/captures"
if [[ -d "${CAP_DIR}" ]]; then
  run_sh "capture_meta_listing" "ls -1t '${CAP_DIR}'/*_meta.json 2>/dev/null | head -n 10"
  mkdir -p "${OUT_DIR}/capture_meta"
  while IFS= read -r f; do
    cp -f "$f" "${OUT_DIR}/capture_meta/" || true
  done < <(ls -1t "${CAP_DIR}"/*_meta.json 2>/dev/null | head -n 5)
fi

SESSION_DIR="${CAP_DIR}/sessions"
if [[ -d "${SESSION_DIR}" ]]; then
  run_sh "session_listing" "ls -1dt '${SESSION_DIR}'/sess_* 2>/dev/null | head -n 10"
  mkdir -p "${OUT_DIR}/session_logs"
  while IFS= read -r d; do
    [[ -d "$d" ]] || continue
    b="$(basename "$d")"
    mkdir -p "${OUT_DIR}/session_logs/${b}"
    cp -f "${d}/session_manifest.json" "${OUT_DIR}/session_logs/${b}/" 2>/dev/null || true
    cp -f "${d}/rosbag_record.log" "${OUT_DIR}/session_logs/${b}/" 2>/dev/null || true
  done < <(ls -1dt "${SESSION_DIR}"/sess_* 2>/dev/null | head -n 3)
fi

if [[ -d "${HOME}/.ros/log/latest" ]]; then
  tar -czf "${OUT_DIR}/ros_latest_logs.tgz" -C "${HOME}/.ros/log/latest" . >/dev/null 2>&1 || true
fi

TAR_PATH="${OUT_DIR}.tar.gz"
tar -czf "${TAR_PATH}" -C "$(dirname "${OUT_DIR}")" "$(basename "${OUT_DIR}")"

note "Diagnostics finished: $(timestamp_utc)"
note "Archive: ${TAR_PATH}"

echo
echo "Done."
echo "Diagnostics directory: ${OUT_DIR}"
echo "Diagnostics archive:   ${TAR_PATH}"
