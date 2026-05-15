#!/usr/bin/env bash
set -Eeuo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS_DIR="${ROOT_DIR}/ros2_ws"
ROS_SETUP="/opt/ros/jazzy/setup.bash"
WS_SETUP="${WS_DIR}/install/setup.bash"
GNSS_PREFLIGHT_HELPER="/usr/local/sbin/subsea-rover-gnss-preflight"

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
  # Full BNO085 reports over software I2C are more stable at 50 Hz.
  "imu_rate_hz:=50.0"
  "imu_i2c_address:=74"
  "imu_i2c_bus:=3"
  "imu_timestamp_mode:=read_end"
  "imu_enable_rotation:=true"
  "imu_enable_accel:=true"
  "imu_enable_gyro:=true"
  "imu_use_driver_cached_reads:=false"
  "imu_init_retry_count:=8"
  "imu_init_retry_delay_s:=0.6"
  "imu_feature_enable_delay_s:=0.12"

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
  "still_capture_backend:=auto"
  "capture_awb:=auto"
  "capture_saturation:=0.6"
  "enable_motion_deblur:=true"
  "deblur_method:=richardson_lucy"
  "deblur_exposure_ms:=8.0"
  "deblur_exposure_time_us:=0"
  "deblur_image_stamp_reference:=midpoint"
  "deblur_timestamp_source:=pps_disciplined_system_clock"
  # Tune these after offset testing if the IMU blur direction feels time-shifted.
  "deblur_timestamp_offset_ms:=0.0"
  "deblur_cam0_timestamp_offset_ms:=0.0"
  "deblur_cam1_timestamp_offset_ms:=0.0"
  # Deblur only needs camera + IMU stamps on the same host clock. GNSS/PPS is
  # useful for absolute time, but field deblur should not be disabled by a GNSS outage.
  "deblur_require_time_reference:=false"
  "deblur_max_time_reference_age_ms:=2000.0"
  "deblur_fov_deg:=72.0"
  "deblur_strength:=1.0"
  "deblur_min_kernel_px:=1.2"
  "deblur_max_kernel_px:=31"
  "deblur_iterations:=12"
  "deblur_wiener_snr:=40.0"
  "deblur_gyro_bias_enable:=true"
  "deblur_gyro_bias_window_s:=2.0"
  "deblur_gyro_bias_min_samples:=25"
  "deblur_gyro_bias_stationary_max_rate_rad_s:=0.025"
  "deblur_gyro_bias_stationary_max_std_rad_s:=0.010"
  "deblur_gyro_bias_max_age_s:=30.0"
  "deblur_use_rig_extrinsics:=true"
  # Rig frame: +X cam0->cam1, +Y camera viewing direction, +Z up. Values are meters.
  "rig_imu_position_m:=0.080,0.000,0.020"
  "rig_cam0_position_m:=-0.367,-0.003,0.063"
  "rig_cam1_position_m:=0.354,-0.003,0.063"
  "rig_gnss_left_position_m:=-0.540,0.000,0.050"
  "rig_gnss_right_position_m:=0.540,0.000,0.050"
  # Row-major rotations. IMU axes: +X base -X, +Y base -Y, +Z up.
  "rig_imu_to_base_rotation:=-1,0,0,0,-1,0,0,0,1"
  # Camera optical frame: +X image right, +Y image down, +Z forward.
  "rig_cam0_base_to_camera_rotation:=1,0,0,0,0,-1,0,1,0"
  "rig_cam1_base_to_camera_rotation:=1,0,0,0,0,-1,0,1,0"
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

Capture mode tradeoff:
  --still   high-res still capture; pauses/restarts previews while capturing
  --stream  no preview pause; captures the current preview stream resolution

To remove the sudo password prompt from normal launch, run once:
  ./scripts/setup_rover_passwordless_launch.sh
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
    pkill -f "bno085_imu_node|gpsd_json_fix_bridge" >/dev/null 2>&1 || true
    pkill -f "component_container_mt.*gps_container|__node:=gps_container" >/dev/null 2>&1 || true
  fi
  sleep 0.8
}

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

gpsd_config_matches_detected_device() {
  local gnss_dev=""
  local devices=""
  local d
  if ! gnss_dev="$(detect_gnss_device)"; then
    return 1
  fi
  devices="$(sed -n 's/^DEVICES="//;s/"$//p' /etc/default/gpsd 2>/dev/null || true)"
  for d in ${devices}; do
    if [[ "${d}" == "${gnss_dev}" ]]; then
      return 0
    fi
  done
  return 1
}

run_gnss_preflight_as_root() {
  local tty_rule='KERNEL=="ttyAMA[0-9]*", GROUP="dialout", MODE="0660"'
  local tty_rule_file='/etc/udev/rules.d/99-ttyama.rules'
  local need_reload_rules="false"
  local gnss_dev=""
  local gnss_real=""
  local gnss_base=""

  if gnss_dev="$(detect_gnss_device)"; then
    gnss_real="$(readlink -f "${gnss_dev}" 2>/dev/null || echo "${gnss_dev}")"
    gnss_base="$(basename "${gnss_real}")"
    systemctl disable --now "serial-getty@${gnss_base}.service" >/dev/null 2>&1 || true
  else
    gnss_dev="/dev/ttyAMA10"
    gnss_base="ttyAMA10"
    systemctl disable --now serial-getty@ttyAMA0.service >/dev/null 2>&1 || true
    systemctl disable --now serial-getty@ttyAMA10.service >/dev/null 2>&1 || true
  fi

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
    udevadm trigger /dev/ttyAMA* >/dev/null 2>&1 || true
  fi

  cat >/etc/default/gpsd <<EOF
START_DAEMON="true"
USBAUTO="true"
DEVICES="${gnss_dev} /dev/pps0"
GPSD_OPTIONS="-n"
EOF

  systemctl enable gpsd.socket chrony >/dev/null 2>&1 || true
  systemctl restart gpsd.socket gpsd.service chrony >/dev/null 2>&1 || true
}

run_gnss_preflight() {
  if [[ -x "${GNSS_PREFLIGHT_HELPER}" ]]; then
    if sudo -n "${GNSS_PREFLIGHT_HELPER}" >/dev/null 2>&1; then
      if gpsd_config_matches_detected_device; then
        echo "Applied GNSS UART preflight with passwordless helper."
        return 0
      fi
      echo "Existing GNSS helper did not update gpsd to the detected UART; applying refreshed preflight..."
    fi
  fi

  if sudo -n true >/dev/null 2>&1; then
    echo "Applying GNSS UART preflight..."
  else
    echo "Requesting sudo for GNSS UART preflight + gpsd/chrony restart..."
    echo "Tip: run ./scripts/setup_rover_passwordless_launch.sh once to remove this prompt."
  fi
  sudo bash -lc "$(declare -f detect_gnss_device); $(declare -f run_gnss_preflight_as_root); run_gnss_preflight_as_root"
}

restart_gpsd_chrony() {
  local systemctl_bin
  systemctl_bin="$(command -v systemctl || echo /usr/bin/systemctl)"

  if sudo -n "${systemctl_bin}" restart gpsd.socket gpsd.service chrony >/dev/null 2>&1; then
    echo "Restarted gpsd/chrony with passwordless sudo."
    return 0
  fi

  echo "Requesting sudo to restart gpsd/chrony..."
  echo "Tip: run ./scripts/setup_rover_passwordless_launch.sh once to remove this prompt."
  sudo "${systemctl_bin}" restart gpsd.socket gpsd.service chrony || true
}

if command -v systemctl >/dev/null 2>&1; then
  if [[ "${RUN_ROS_CLEANUP}" == "true" ]]; then
    echo "Running ROS prelaunch cleanup..."
    run_ros_prelaunch_cleanup
  fi

  if [[ "${RUN_GNSS_PREFLIGHT}" == "true" ]]; then
    run_gnss_preflight
  elif [[ "${RESTART_GPSD_CHRONY}" == "true" ]]; then
    restart_gpsd_chrony
  fi
fi

echo
echo "Starting rover app with FIELD_LAUNCH_ARGS:"
printf '  %s\n' "${FIELD_LAUNCH_ARGS[@]}"
echo

exec ros2 launch subsea_bringup rover_app.launch.py "${FIELD_LAUNCH_ARGS[@]}"
