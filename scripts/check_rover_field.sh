#!/usr/bin/env bash
set -Eeuo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS_DIR="${ROOT_DIR}/ros2_ws"
ROS_SETUP="/opt/ros/jazzy/setup.bash"
WS_SETUP="${WS_DIR}/install/setup.bash"

if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "ERROR: ROS setup not found at ${ROS_SETUP}" >&2
  exit 1
fi
if [[ ! -f "${WS_SETUP}" ]]; then
  echo "ERROR: Workspace setup not found at ${WS_SETUP}" >&2
  exit 1
fi

source "${ROS_SETUP}"
source "${WS_SETUP}"

echo "== gpsd stream (6s) =="
if command -v gpspipe >/dev/null 2>&1; then
  timeout 6s gpspipe -w -n 12 || true
else
  echo "gpspipe not found"
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
timeout 8s ros2 topic echo /fix --once || echo "No /fix message within 8s"

echo
echo "== ROS topic info (/time_reference) =="
ros2 topic info /time_reference -v || true

echo
echo "== ROS message sample (/time_reference, 8s timeout) =="
timeout 8s ros2 topic echo /time_reference --once || echo "No /time_reference message within 8s (expected with gpsd_client)"
