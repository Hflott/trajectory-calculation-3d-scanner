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
GNSS_PREFLIGHT="true"
ROS_CLEANUP="true"
SWAP_PREVIEW_FEEDS="false"
IMU_I2C_BUS=""
IMU_RATE_HZ=""
IMU_GYRO_ONLY="false"
USE_GPSD_JSON_BRIDGE="false"
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
  --skip-gnss-preflight   Skip GNSS UART/gpsd preflight checks
  --skip-service-restart  Do not restart gpsd/chrony before launch
  --skip-ros-cleanup      Do not stop stale ROS daemons/processes before launch
  --use-gpsd-json-bridge  Use subsea gpsd JSON bridge for /fix (and /time_reference)
  --use-gpsd-client       Use gpsd_client component (default)
  --swap-preview-feeds    Swap UI left/right feed placement (cam1 left, cam0 right)
  --imu-i2c-bus N         Set imu_i2c_bus:=N for BNO085 (e.g. 3 for i2c-gpio on GPIO5/6)
  --imu-rate-hz N         Set imu_rate_hz:=N (float, e.g. 100 or 200)
  --imu-gyro-only         Start IMU with gyro only (disable rotation+accel features)
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
    --skip-gnss-preflight)
      GNSS_PREFLIGHT="false"
      shift
      ;;
    --skip-ros-cleanup)
      ROS_CLEANUP="false"
      shift
      ;;
    --swap-preview-feeds)
      SWAP_PREVIEW_FEEDS="true"
      shift
      ;;
    --use-gpsd-json-bridge)
      USE_GPSD_JSON_BRIDGE="true"
      shift
      ;;
    --use-gpsd-client)
      USE_GPSD_JSON_BRIDGE="false"
      shift
      ;;
    --imu-i2c-bus)
      [[ $# -ge 2 ]] || { echo "ERROR: --imu-i2c-bus requires a value" >&2; exit 1; }
      if [[ ! "$2" =~ ^[0-9]+$ ]]; then
        echo "ERROR: --imu-i2c-bus must be a non-negative integer (got '$2')" >&2
        exit 1
      fi
      IMU_I2C_BUS="$2"
      shift 2
      ;;
    --imu-rate-hz)
      [[ $# -ge 2 ]] || { echo "ERROR: --imu-rate-hz requires a value" >&2; exit 1; }
      if [[ ! "$2" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
        echo "ERROR: --imu-rate-hz must be a positive number (got '$2')" >&2
        exit 1
      fi
      IMU_RATE_HZ="$2"
      shift 2
      ;;
    --imu-gyro-only)
      IMU_GYRO_ONLY="true"
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

run_ros_prelaunch_cleanup() {
  # Clear stale ROS graph cache and rover app processes from previous runs.
  ros2 daemon stop >/dev/null 2>&1 || true
  if command -v pkill >/dev/null 2>&1; then
    pkill -f "ros2 launch subsea_bringup rover_app.launch.py" >/dev/null 2>&1 || true
    pkill -f "subsea_ui_node|capture_service|navsat_transform_node|ekf_node" >/dev/null 2>&1 || true
    pkill -f "component_container_mt.*gps_container|__node:=gps_container" >/dev/null 2>&1 || true
  fi
  sleep 0.3
}

run_gnss_preflight_as_root() {
  local tty_rule='KERNEL=="ttyAMA0", GROUP="dialout", MODE="0660"'
  local tty_rule_file='/etc/udev/rules.d/99-ttyama0.rules'
  local need_reload_rules="false"

  # Free UART0 from login console if enabled.
  systemctl disable --now serial-getty@ttyAMA0.service >/dev/null 2>&1 || true

  # Ensure gpsd can open ttyAMA0.
  if id gpsd >/dev/null 2>&1; then
    usermod -aG tty,dialout gpsd || true
  fi

  if ! grep -qF "${tty_rule}" "${tty_rule_file}" 2>/dev/null; then
    mkdir -p "$(dirname "${tty_rule_file}")"
    {
      [[ -f "${tty_rule_file}" ]] && cat "${tty_rule_file}"
      echo "${tty_rule}"
    } | awk '!seen[$0]++' >"${tty_rule_file}.tmp"
    mv "${tty_rule_file}.tmp" "${tty_rule_file}"
    chmod 0644 "${tty_rule_file}"
    need_reload_rules="true"
  fi

  if [[ "${need_reload_rules}" == "true" ]]; then
    udevadm control --reload-rules >/dev/null 2>&1 || true
    udevadm trigger /dev/ttyAMA0 >/dev/null 2>&1 || true
  fi

  systemctl restart gpsd.socket gpsd.service chrony >/dev/null 2>&1 || true
}

if command -v systemctl >/dev/null 2>&1; then
  if [[ "${ROS_CLEANUP}" == "true" ]]; then
    echo "Running ROS prelaunch cleanup..."
    run_ros_prelaunch_cleanup
  fi

  if [[ "${GNSS_PREFLIGHT}" == "true" ]]; then
    if sudo -n true >/dev/null 2>&1; then
      echo "Applying GNSS UART preflight..."
      sudo bash -lc "$(declare -f run_gnss_preflight_as_root); run_gnss_preflight_as_root"
    else
      echo "Requesting sudo for GNSS UART preflight + gpsd/chrony restart..."
      sudo bash -lc "$(declare -f run_gnss_preflight_as_root); run_gnss_preflight_as_root"
    fi
  elif [[ "${RESTART_SERVICES}" == "true" ]]; then
    if sudo -n true >/dev/null 2>&1; then
      sudo systemctl restart gpsd.socket chrony || true
    else
      echo "Requesting sudo to restart gpsd/chrony..."
      sudo systemctl restart gpsd.socket chrony || true
    fi
  fi
fi

echo
echo "Starting rover app..."
echo "  start_localization:=${START_LOCALIZATION}"
echo "  capture_mode:=${CAPTURE_MODE}"
echo "  gnss_preflight:=${GNSS_PREFLIGHT}"
echo "  ros_cleanup:=${ROS_CLEANUP}"
echo "  use_gpsd_json_bridge:=${USE_GPSD_JSON_BRIDGE}"
if [[ ${#EXTRA_ARGS[@]} -gt 0 ]]; then
  echo "  extra args: ${EXTRA_ARGS[*]}"
fi
if [[ -n "${IMU_I2C_BUS}" ]]; then
  echo "  imu_i2c_bus:=${IMU_I2C_BUS}"
fi
if [[ -n "${IMU_RATE_HZ}" ]]; then
  echo "  imu_rate_hz:=${IMU_RATE_HZ}"
fi
if [[ "${IMU_GYRO_ONLY}" == "true" ]]; then
  echo "  imu_mode:=gyro_only"
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
if [[ -f "${LAUNCH_FILE}" ]] && grep -q "DeclareLaunchArgument('use_gpsd_json_bridge'" "${LAUNCH_FILE}"; then
  echo "  use_gpsd_json_bridge:=${USE_GPSD_JSON_BRIDGE}"
  LAUNCH_ARGS+=("use_gpsd_json_bridge:=${USE_GPSD_JSON_BRIDGE}")
fi
if [[ -f "${LAUNCH_FILE}" ]] && grep -q "DeclareLaunchArgument('start_imu_node'" "${LAUNCH_FILE}"; then
  echo "  start_imu_node:=true"
  LAUNCH_ARGS+=("start_imu_node:=true")
fi
if [[ -n "${IMU_I2C_BUS}" ]] && [[ -f "${LAUNCH_FILE}" ]] && grep -q "DeclareLaunchArgument('imu_i2c_bus'" "${LAUNCH_FILE}"; then
  echo "  imu_i2c_bus:=${IMU_I2C_BUS}"
  LAUNCH_ARGS+=("imu_i2c_bus:=${IMU_I2C_BUS}")
fi
if [[ -n "${IMU_RATE_HZ}" ]] && [[ -f "${LAUNCH_FILE}" ]] && grep -q "DeclareLaunchArgument('imu_rate_hz'" "${LAUNCH_FILE}"; then
  echo "  imu_rate_hz:=${IMU_RATE_HZ}"
  LAUNCH_ARGS+=("imu_rate_hz:=${IMU_RATE_HZ}")
fi
if [[ "${IMU_GYRO_ONLY}" == "true" ]] && [[ -f "${LAUNCH_FILE}" ]] && grep -q "DeclareLaunchArgument('imu_enable_rotation'" "${LAUNCH_FILE}"; then
  echo "  imu_enable_rotation:=false"
  echo "  imu_enable_accel:=false"
  echo "  imu_enable_gyro:=true"
  LAUNCH_ARGS+=("imu_enable_rotation:=false" "imu_enable_accel:=false" "imu_enable_gyro:=true")
fi
if [[ "${SWAP_PREVIEW_FEEDS}" == "true" ]] && [[ -f "${LAUNCH_FILE}" ]] && grep -q "DeclareLaunchArgument('swap_preview_feeds'" "${LAUNCH_FILE}"; then
  echo "  swap_preview_feeds:=true"
  LAUNCH_ARGS+=("swap_preview_feeds:=true")
fi
if [[ ${#EXTRA_ARGS[@]} -gt 0 ]]; then
  LAUNCH_ARGS+=("${EXTRA_ARGS[@]}")
fi

exec ros2 launch subsea_bringup rover_app.launch.py "${LAUNCH_ARGS[@]}"
