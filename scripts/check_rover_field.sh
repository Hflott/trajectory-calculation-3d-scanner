#!/usr/bin/env bash
set -Eeuo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS_DIR="${ROOT_DIR}/ros2_ws"
ROS_SETUP="/opt/ros/jazzy/setup.bash"
WS_SETUP="${WS_DIR}/install/setup.bash"

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

if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "ERROR: ROS setup not found at ${ROS_SETUP}" >&2
  exit 1
fi
if [[ ! -f "${WS_SETUP}" ]]; then
  echo "ERROR: Workspace setup not found at ${WS_SETUP}" >&2
  exit 1
fi

source_safe "${ROS_SETUP}"
source_safe "${WS_SETUP}"

detect_gnss_device() {
  if compgen -G "/dev/serial/by-id/*" >/dev/null 2>&1; then
    ls -1 /dev/serial/by-id/* | head -n1
    return 0
  fi

  local candidates=(
    /dev/serial0
    /dev/ttyAMA0
    /dev/ttyAMA10
    /dev/ttyAMA1
    /dev/ttyS0
    /dev/ttyACM0
    /dev/ttyACM1
    /dev/ttyUSB0
    /dev/ttyUSB1
  )
  local d
  for d in "${candidates[@]}"; do
    if [[ -e "${d}" ]]; then
      echo "${d}"
      return 0
    fi
  done

  if compgen -G "/dev/ttyAMA*" >/dev/null 2>&1; then
    ls -1 /dev/ttyAMA* | sort -V | head -n1
    return 0
  fi

  return 1
}

GNSS_DEV="$(detect_gnss_device || true)"

echo "== GNSS device/config =="
echo "Detected serial: ${GNSS_DEV:-none}"
grep -E 'START_DAEMON|USBAUTO|DEVICES|GPSD_OPTIONS' /etc/default/gpsd 2>/dev/null || true

echo
echo "== gpsd stream (6s) =="
if command -v gpspipe >/dev/null 2>&1; then
  timeout 6s gpspipe -w -n 12 || true
else
  echo "gpspipe not found"
fi

echo
echo "== GNSS UART NMEA sample (6s) =="
if [[ -n "${GNSS_DEV}" ]]; then
  timeout 6s cat "${GNSS_DEV}" | grep -aE 'RMC|GGA|ZDA' || true
else
  echo "No GNSS serial device detected"
fi

echo
echo "== PPS test (/dev/pps0) =="
if [[ -e /dev/pps0 ]]; then
  if sudo -n true >/dev/null 2>&1; then
    sudo timeout 6s ppstest /dev/pps0 || true
  else
    echo "Skipping ppstest (sudo password required). Run manually:"
    echo "  sudo timeout 6s ppstest /dev/pps0"
  fi
else
  echo "/dev/pps0 not present"
fi

echo
echo "== chrony sources =="
if command -v chronyc >/dev/null 2>&1; then
  chronyc sources -v || true
else
  echo "chronyc not found"
fi

echo
echo "== ROS topic info (/fix) =="
ros2 topic info /fix -v || true

echo
echo "== ROS message sample (/fix, 8s timeout) =="
timeout 8s ros2 topic echo /fix --qos-reliability reliable --once || echo "No /fix message within 8s"

echo
echo "== ROS topic info (/time_reference) =="
ros2 topic info /time_reference -v || true

echo
echo "== ROS message sample (/time_reference, 8s timeout) =="
timeout 8s ros2 topic echo /time_reference --once || echo "No /time_reference message within 8s (expected with gpsd_client)"
