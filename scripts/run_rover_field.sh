#!/usr/bin/env bash
set -Eeuo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS_DIR="${ROOT_DIR}/ros2_ws"
ROS_SETUP="/opt/ros/jazzy/setup.bash"
WS_SETUP="${WS_DIR}/install/setup.bash"

# ---------------------------------------------------------------------------
# Field startup defaults
#
# Edit this block when you want to change the normal rover startup.
# Then run:
#
#   ./scripts/run_rover_field.sh
#
# Each entry is passed directly to:
#   ros2 launch subsea_bringup rover_app.launch.py
# ---------------------------------------------------------------------------

RUN_ROS_CLEANUP="true"
RUN_GNSS_PREFLIGHT="true"
RESTART_GPSD_CHRONY="true"

FIELD_LAUNCH_ARGS=(
  "start_gpsd_client:=true"
  "use_gpsd_json_bridge:=true"
  "start_imu_node:=true"
  "start_localization:=true"
  "capture_mode:=still"

  # IMU
  "imu_topic:=/imu/data"
  "imu_frame_id:=imu_link"
  "imu_rate_hz:=100.0"
  "imu_i2c_address:=74"
  "imu_i2c_bus:=1"
  "imu_timestamp_mode:=read_end"
  "imu_enable_rotation:=true"
  "imu_enable_accel:=true"
  "imu_enable_gyro:=true"

  # Camera/UI preview
  "manage_previews:=true"
  # Stable dual-camera Pi preset. Raise these after the feeds are proven stable.
  "preview_width:=640"
  "preview_height:=360"
  "preview_fps:=10"
  "preview_ui_width:=480"
  "preview_ui_height:=270"
  "preview_ui_fps:=8"
  "preview_ui_saturation:=0.6"
  "preview_format:=BGR888"
  "swap_preview_feeds:=false"
  "cam0_orientation:=180"
  "cam1_orientation:=180"
  "preview_source_applies_orientation:=false"

  # High-quality capture/deblur path. Live preview stays low-res above.
  "capture_width:=1456"
  "capture_height:=1088"
  "capture_quality:=95"
  "capture_warmup_ms:=700"
  "capture_timeout_ms:=1500"
  "capture_pause_previews:=true"
  "capture_awb:=auto"
  "capture_saturation:=0.6"
  "enable_motion_deblur:=true"
  "deblur_method:=richardson_lucy"
  "deblur_exposure_ms:=8.0"
  "deblur_exposure_time_us:=0"
  "deblur_image_stamp_reference:=midpoint"
  "deblur_timestamp_source:=pps_disciplined_system_clock"
  "deblur_require_time_reference:=true"
  "deblur_max_time_reference_age_ms:=2000.0"
  "deblur_fov_deg:=72.0"
  "deblur_strength:=1.0"
  "deblur_min_kernel_px:=1.2"
  "deblur_max_kernel_px:=31"
  "deblur_iterations:=12"
  "deblur_wiener_snr:=40.0"
  "deblur_imu_to_cam_yaw_deg:=0.0"
  "deblur_use_translation:=false"
  "deblur_assumed_depth_m:=1.5"
  "deblur_max_odom_age_ms:=200.0"
)

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
Usage:
  ./scripts/run_rover_field.sh

Normal field startup is configured by editing FIELD_LAUNCH_ARGS near the top
of this script. You do not need to remember launch flags.

Optional one-off overrides are still accepted:
  ./scripts/run_rover_field.sh imu_rate_hz:=200.0
  ./scripts/run_rover_field.sh capture_mode:=stream
  ./scripts/run_rover_field.sh --skip-gnss-preflight
EOF
}

set_launch_arg() {
  local arg="$1"
  local key="${arg%%:=*}"
  local updated=()
  local item

  for item in "${FIELD_LAUNCH_ARGS[@]}"; do
    if [[ "${item%%:=*}" != "${key}" ]]; then
      updated+=("${item}")
    fi
  done

  updated+=("${arg}")
  FIELD_LAUNCH_ARGS=("${updated[@]}")
}

apply_legacy_shortcut_arg() {
  case "$1" in
    --still)
      set_launch_arg "capture_mode:=still"
      return 0
      ;;
    --stream)
      set_launch_arg "capture_mode:=stream"
      return 0
      ;;
    --no-localization)
      set_launch_arg "start_localization:=false"
      return 0
      ;;
    --localization)
      set_launch_arg "start_localization:=true"
      return 0
      ;;
    --skip-service-restart)
      RESTART_GPSD_CHRONY="false"
      return 0
      ;;
    --skip-gnss-preflight)
      RUN_GNSS_PREFLIGHT="false"
      return 0
      ;;
    --skip-ros-cleanup)
      RUN_ROS_CLEANUP="false"
      return 0
      ;;
    --swap-preview-feeds)
      set_launch_arg "swap_preview_feeds:=true"
      return 0
      ;;
    --use-gpsd-json-bridge)
      set_launch_arg "use_gpsd_json_bridge:=true"
      return 0
      ;;
    --use-gpsd-client)
      set_launch_arg "use_gpsd_json_bridge:=false"
      return 0
      ;;
    --imu-gyro-only)
      set_launch_arg "imu_enable_rotation:=false"
      set_launch_arg "imu_enable_accel:=false"
      set_launch_arg "imu_enable_gyro:=true"
      return 0
      ;;
  esac
  return 1
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    -h|--help)
      usage
      exit 0
      ;;
    --imu-i2c-bus)
      [[ $# -ge 2 ]] || { echo "ERROR: --imu-i2c-bus requires a value" >&2; exit 1; }
      set_launch_arg "imu_i2c_bus:=$2"
      shift 2
      ;;
    --imu-rate-hz)
      [[ $# -ge 2 ]] || { echo "ERROR: --imu-rate-hz requires a value" >&2; exit 1; }
      set_launch_arg "imu_rate_hz:=$2"
      shift 2
      ;;
    --*)
      if apply_legacy_shortcut_arg "$1"; then
        shift
      else
        echo "ERROR: unknown option '$1'. Prefer editing FIELD_LAUNCH_ARGS in the script." >&2
        exit 1
      fi
      ;;
    *:=*)
      set_launch_arg "$1"
      shift
      ;;
    *)
      echo "ERROR: unexpected argument '$1'. Use launch_arg:=value or edit FIELD_LAUNCH_ARGS." >&2
      exit 1
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

  systemctl disable --now serial-getty@ttyAMA0.service >/dev/null 2>&1 || true

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
  if [[ "${RUN_ROS_CLEANUP}" == "true" ]]; then
    echo "Running ROS prelaunch cleanup..."
    run_ros_prelaunch_cleanup
  fi

  if [[ "${RUN_GNSS_PREFLIGHT}" == "true" ]]; then
    if sudo -n true >/dev/null 2>&1; then
      echo "Applying GNSS UART preflight..."
    else
      echo "Requesting sudo for GNSS UART preflight + gpsd/chrony restart..."
    fi
    sudo bash -lc "$(declare -f run_gnss_preflight_as_root); run_gnss_preflight_as_root"
  elif [[ "${RESTART_GPSD_CHRONY}" == "true" ]]; then
    if sudo -n true >/dev/null 2>&1; then
      sudo systemctl restart gpsd.socket chrony || true
    else
      echo "Requesting sudo to restart gpsd/chrony..."
      sudo systemctl restart gpsd.socket chrony || true
    fi
  fi
fi

echo
echo "Starting rover app with FIELD_LAUNCH_ARGS:"
printf '  %s\n' "${FIELD_LAUNCH_ARGS[@]}"
echo

exec ros2 launch subsea_bringup rover_app.launch.py "${FIELD_LAUNCH_ARGS[@]}"
