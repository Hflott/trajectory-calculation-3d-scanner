#!/usr/bin/env bash
set -Eeuo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS_DIR="${ROOT_DIR}/ros2_ws"
ROS_SETUP="/opt/ros/jazzy/setup.bash"
WS_SETUP="${WS_DIR}/install/setup.bash"
LAUNCH_FILE="${WS_DIR}/src/subsea_bringup/launch/rover_app.launch.py"

START_LOCALIZATION="true"
CAPTURE_MODE="stream"
RESTART_SERVICES="true"
EXTRA_ARGS=()

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

usage() {
  cat <<'EOF'
Usage: ./scripts/run_rover_field.sh [options] [launch-arg:=value ...]

Starts rover bringup with field-friendly defaults:
  - gpsd_client enabled
  - localization enabled
  - stream capture mode

Options:
  --still                 Use capture_mode:=still
  --stream                Use capture_mode:=stream (default)
  --no-localization       Set start_localization:=false
  --localization          Set start_localization:=true (default)
  --skip-service-restart  Do not restart gpsd/chrony before launch
  -h, --help              Show this help

Any additional tokens are passed through to:
  ros2 launch subsea_bringup rover_app.launch.py
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --still)
      CAPTURE_MODE="still"
      shift
      ;;
    --stream)
      CAPTURE_MODE="stream"
      shift
      ;;
    --no-localization)
      START_LOCALIZATION="false"
      shift
      ;;
    --localization)
      START_LOCALIZATION="true"
      shift
      ;;
    --skip-service-restart)
      RESTART_SERVICES="false"
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      EXTRA_ARGS+=("$1")
      shift
      ;;
  esac
done

if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "ERROR: ROS setup not found at ${ROS_SETUP}" >&2
  exit 1
fi
if [[ ! -f "${WS_SETUP}" ]]; then
  echo "ERROR: Workspace setup not found at ${WS_SETUP}" >&2
  echo "Build once first:" >&2
  echo "  cd ${WS_DIR}" >&2
  echo "  source ${ROS_SETUP}" >&2
  echo "  colcon build --symlink-install" >&2
  exit 1
fi

source_safe "${ROS_SETUP}"
source_safe "${WS_SETUP}"

if [[ "${RESTART_SERVICES}" == "true" ]] && command -v systemctl >/dev/null 2>&1; then
  if sudo -n true >/dev/null 2>&1; then
    sudo systemctl restart gpsd.socket chrony || true
  else
    echo "Requesting sudo to restart gpsd/chrony..."
    sudo systemctl restart gpsd.socket chrony || true
  fi
fi

echo
echo "Starting rover app..."
echo "  start_localization:=${START_LOCALIZATION}"
echo "  capture_mode:=${CAPTURE_MODE}"
if [[ ${#EXTRA_ARGS[@]} -gt 0 ]]; then
  echo "  extra args: ${EXTRA_ARGS[*]}"
fi
echo

LAUNCH_ARGS=(
  "start_localization:=${START_LOCALIZATION}"
  "capture_mode:=${CAPTURE_MODE}"
)
if [[ -f "${LAUNCH_FILE}" ]] && grep -q "DeclareLaunchArgument('start_gpsd_client'" "${LAUNCH_FILE}"; then
  echo "  start_gpsd_client:=true"
  LAUNCH_ARGS+=("start_gpsd_client:=true")
else
  echo "  note: launch file has no start_gpsd_client arg; skipping explicit GNSS arg"
fi
if [[ ${#EXTRA_ARGS[@]} -gt 0 ]]; then
  LAUNCH_ARGS+=("${EXTRA_ARGS[@]}")
fi

exec ros2 launch subsea_bringup rover_app.launch.py "${LAUNCH_ARGS[@]}"
