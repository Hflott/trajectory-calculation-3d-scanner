#!/usr/bin/env bash
set -Eeuo pipefail

ROS_DISTRO="jazzy"
UBUNTU_CODENAME="noble"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
WS_DIR="${REPO_ROOT}/ros2_ws"

INSTALL_NOVNC=0
SKIP_BUILD=0
AUTO_CONFIG_RPI=1
PPS_GPIO_PIN=23

log() {
  echo
  echo "==> $*"
}

warn() {
  echo "WARNING: $*" >&2
}

die() {
  echo "ERROR: $*" >&2
  exit 1
}

usage() {
  cat <<EOF
Usage: $(basename "$0") [options]

Bootstrap this repository on Ubuntu 24.04 LTS with ROS 2 ${ROS_DISTRO}.

Options:
  --with-novnc   Also install Xvfb/fluxbox/x11vnc/noVNC for browser-based GUI
  --skip-build   Install dependencies only (skip rosdep+colcon build)
  --no-rpi-autoconfig  Skip Raspberry Pi accessory auto-configuration
  --pps-gpio-pin N     GPIO pin for PPS overlay (default: 23)
  -h, --help     Show this help
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --with-novnc)
      INSTALL_NOVNC=1
      shift
      ;;
    --skip-build)
      SKIP_BUILD=1
      shift
      ;;
    --no-rpi-autoconfig)
      AUTO_CONFIG_RPI=0
      shift
      ;;
    --pps-gpio-pin)
      [[ $# -ge 2 ]] || die "--pps-gpio-pin requires a value"
      PPS_GPIO_PIN="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      die "Unknown argument: $1"
      ;;
  esac
done

[[ -d "${WS_DIR}/src" ]] || die "Expected workspace at ${WS_DIR}/src"
[[ "${PPS_GPIO_PIN}" =~ ^[0-9]+$ ]] || die "--pps-gpio-pin must be an integer"

if [[ ! -f /etc/os-release ]]; then
  die "/etc/os-release not found"
fi

# shellcheck disable=SC1091
source /etc/os-release
if [[ "${ID:-}" != "ubuntu" || "${VERSION_ID:-}" != "24.04" ]]; then
  die "This bootstrap supports Ubuntu 24.04 only (found: ${PRETTY_NAME:-unknown})"
fi

SUDO=""
if [[ "${EUID}" -ne 0 ]]; then
  command -v sudo >/dev/null 2>&1 || die "sudo is required when running as non-root"
  SUDO="sudo"
fi

run_root() {
  if [[ -n "${SUDO}" ]]; then
    "${SUDO}" "$@"
  else
    "$@"
  fi
}

apt_install() {
  run_root apt-get install -y --no-install-recommends "$@"
}

install_ros_repo() {
  log "Installing apt prerequisites"
  run_root apt-get update
  apt_install ca-certificates curl gnupg lsb-release software-properties-common git

  if [[ ! -f /etc/apt/sources.list.d/ros2.list ]]; then
    log "Adding ROS 2 apt repository"
    run_root mkdir -p /etc/apt/keyrings
    tmp_key="$(mktemp)"
    curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o "${tmp_key}"
    run_root gpg --dearmor --yes -o /etc/apt/keyrings/ros-archive-keyring.gpg "${tmp_key}"
    rm -f "${tmp_key}"
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu ${UBUNTU_CODENAME} main" \
      | run_root tee /etc/apt/sources.list.d/ros2.list >/dev/null
  fi
}

install_base_packages() {
  log "Installing ROS + system dependencies"
  run_root apt-get update
  apt_install \
    python3-colcon-common-extensions \
    python3-libgpiod \
    python3-numpy \
    python3-opencv \
    python3-pillow \
    python3-pyqt5 \
    python3-qtpy \
    python3-rosdep \
    psmisc \
    v4l-utils \
    ros-${ROS_DISTRO}-ament-cmake \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-robot-localization \
    ros-${ROS_DISTRO}-ros-base \
    ros-dev-tools
}

install_optional_gui_stack() {
  if [[ "${INSTALL_NOVNC}" -eq 1 ]]; then
    log "Installing optional noVNC GUI stack"
    apt_install xvfb x11vnc fluxbox novnc
  fi
}

install_camera_stack() {
  log "Installing camera capture prerequisites"
  if is_raspberry_pi; then
    log "Raspberry Pi detected: skipping apt camera_ros (workspace build recommended)."
  elif apt-cache show "ros-${ROS_DISTRO}-camera-ros" >/dev/null 2>&1; then
    apt_install "ros-${ROS_DISTRO}-camera-ros"
  else
    warn "ros-${ROS_DISTRO}-camera-ros not available via apt; will use workspace/submodule build."
  fi

  if command -v rpicam-still >/dev/null 2>&1; then
    return
  fi

  if apt-cache show rpicam-apps >/dev/null 2>&1; then
    apt_install rpicam-apps
  elif apt-cache show libcamera-apps >/dev/null 2>&1; then
    apt_install libcamera-apps
  else
    warn "Neither rpicam-apps nor libcamera-apps found in apt repositories."
  fi

  if ! command -v rpicam-still >/dev/null 2>&1 && command -v libcamera-still >/dev/null 2>&1; then
    log "Creating compatibility wrapper: /usr/local/bin/rpicam-still -> libcamera-still"
    run_root tee /usr/local/bin/rpicam-still >/dev/null <<'EOF'
#!/usr/bin/env bash
exec libcamera-still "$@"
EOF
    run_root chmod +x /usr/local/bin/rpicam-still
  fi

  if ! command -v rpicam-still >/dev/null 2>&1; then
    warn "rpicam-still not found. Real still capture will fail until camera apps are installed."
  fi
}

is_raspberry_pi() {
  if [[ -f /proc/device-tree/model ]]; then
    tr -d '\0' </proc/device-tree/model | grep -qi "raspberry pi"
    return $?
  fi
  return 1
}

detect_gnss_device() {
  if ls /dev/serial/by-id/* >/dev/null 2>&1; then
    ls -1 /dev/serial/by-id/* | head -n1
    return 0
  fi

  local candidates=(
    /dev/ttyAMA0
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
  return 1
}

ensure_line_in_file() {
  local line="$1"
  local file="$2"
  run_root touch "${file}"
  if ! grep -qxF "${line}" "${file}" 2>/dev/null; then
    echo "${line}" | run_root tee -a "${file}" >/dev/null
  fi
}

configure_rpi_boot_overlays() {
  local cfg=""
  if [[ -f /boot/firmware/usercfg.txt ]]; then
    cfg="/boot/firmware/usercfg.txt"
  elif [[ -f /boot/firmware/config.txt ]]; then
    cfg="/boot/firmware/config.txt"
  fi

  if [[ -z "${cfg}" ]]; then
    warn "Could not find /boot/firmware/usercfg.txt or /boot/firmware/config.txt; skipping PPS/UART/I2C boot config."
    return
  fi

  log "Configuring Raspberry Pi boot overlays in ${cfg}"
  ensure_line_in_file "enable_uart=1" "${cfg}"
  ensure_line_in_file "dtparam=i2c_arm=on" "${cfg}"
  ensure_line_in_file "dtoverlay=pps-gpio,gpiopin=${PPS_GPIO_PIN}" "${cfg}"
}

configure_gpsd() {
  local gnss_dev="$1"
  local gpsd_cfg="/etc/default/gpsd"
  local devices="${gnss_dev}"

  if [[ -e /dev/pps0 ]]; then
    devices="${devices} /dev/pps0"
  else
    # Keep pps path configured so it starts working after reboot when overlay is active.
    devices="${devices} /dev/pps0"
  fi

  log "Configuring gpsd (${gpsd_cfg})"
  run_root tee "${gpsd_cfg}" >/dev/null <<EOF
START_DAEMON="true"
USBAUTO="true"
DEVICES="${devices}"
GPSD_OPTIONS="-n"
EOF
}

configure_chrony() {
  local confd="/etc/chrony/conf.d"
  local conf="${confd}/subsea_gnss_pps.conf"
  local main_conf="/etc/chrony/chrony.conf"

  log "Configuring chrony for GNSS + PPS (${conf})"
  run_root mkdir -p "${confd}"
  run_root tee "${conf}" >/dev/null <<'EOF'
# NMEA time from gpsd shared memory
refclock SHM 0 refid NMEA precision 1e-1 offset 0.0 delay 0.2 noselect
# PPS discipline from kernel PPS
refclock PPS /dev/pps0 lock NMEA refid PPS precision 1e-7
EOF

  if ! grep -qF "confdir /etc/chrony/conf.d" "${main_conf}" 2>/dev/null; then
    ensure_line_in_file "confdir /etc/chrony/conf.d" "${main_conf}"
  fi
}

configure_rpi_accessories() {
  log "Installing Raspberry Pi accessory packages"
  apt_install gpsd gpsd-clients chrony pps-tools i2c-tools gpiod

  configure_rpi_boot_overlays

  local gnss_dev=""
  if gnss_dev="$(detect_gnss_device)"; then
    log "Detected GNSS serial device: ${gnss_dev}"
    configure_gpsd "${gnss_dev}"
  else
    warn "No GNSS serial device detected. Configuring gpsd with placeholder device."
    configure_gpsd "/dev/ttyAMA0"
  fi

  configure_chrony

  if command -v systemctl >/dev/null 2>&1; then
    log "Enabling/restarting gpsd and chrony"
    run_root systemctl enable gpsd.socket chrony >/dev/null 2>&1 || true
    run_root systemctl restart gpsd.socket chrony >/dev/null 2>&1 || true
  else
    warn "systemctl not available; please restart gpsd/chrony manually."
  fi

  # GPIO trigger button access without root.
  local target_user="${SUDO_USER:-${USER}}"
  run_root usermod -aG gpio "${target_user}" >/dev/null 2>&1 || true

  warn "Raspberry Pi boot config was updated. Reboot is required for PPS/UART/I2C changes to fully apply."
}

sync_submodules() {
  if git -C "${REPO_ROOT}" rev-parse --is-inside-work-tree >/dev/null 2>&1; then
    log "Syncing git submodules (camera_ros)"
    git -C "${REPO_ROOT}" submodule sync --recursive
    git -C "${REPO_ROOT}" submodule update --init --recursive
  else
    warn "Repository is not a git checkout; skipping submodule initialization."
  fi
}

ensure_camera_ros_source() {
  local cam_dir="${WS_DIR}/src/camera_ros"
  if [[ -d "${cam_dir}" ]]; then
    return
  fi
  log "camera_ros source not found in workspace; cloning into src/"
  git clone --depth 1 https://github.com/christianrauch/camera_ros.git "${cam_dir}"
}

setup_rosdep() {
  if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    log "Initializing rosdep"
    run_root rosdep init
  fi
  log "Updating rosdep index"
  rosdep update
}

build_workspace() {
  log "Installing ROS package dependencies via rosdep"
  # shellcheck disable=SC1091
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
  cd "${WS_DIR}"
  local skip_keys="ament_python"
  if is_raspberry_pi; then
    # Prefer Raspberry Pi/system libcamera over ros-${ROS_DISTRO}-libcamera.
    skip_keys="${skip_keys} libcamera"
  fi
  rosdep install --from-paths src --ignore-src --rosdistro "${ROS_DISTRO}" -r -y --skip-keys "${skip_keys}"

  log "Building workspace"
  colcon build --symlink-install
}

write_shell_setup() {
  local target_user="${SUDO_USER:-${USER}}"
  local target_home
  target_home="$(getent passwd "${target_user}" | cut -d: -f6)"
  if [[ -z "${target_home}" ]]; then
    target_home="${HOME}"
  fi

  local bashrc="${target_home}/.bashrc"
  touch "${bashrc}"
  if ! grep -qxF "source /opt/ros/${ROS_DISTRO}/setup.bash" "${bashrc}"; then
    echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> "${bashrc}"
  fi
  if [[ "${SKIP_BUILD}" -eq 0 ]]; then
    if ! grep -qxF "source ${WS_DIR}/install/setup.bash" "${bashrc}"; then
      echo "source ${WS_DIR}/install/setup.bash" >> "${bashrc}"
    fi
  fi
}

install_ros_repo
install_base_packages
install_optional_gui_stack
install_camera_stack
if [[ "${AUTO_CONFIG_RPI}" -eq 1 ]] && is_raspberry_pi; then
  configure_rpi_accessories
elif [[ "${AUTO_CONFIG_RPI}" -eq 0 ]] && is_raspberry_pi; then
  log "Skipping Raspberry Pi accessory auto-configuration (--no-rpi-autoconfig)"
fi
sync_submodules
ensure_camera_ros_source
setup_rosdep

if [[ "${SKIP_BUILD}" -eq 0 ]]; then
  build_workspace
else
  log "Skipping workspace build (--skip-build)"
fi

write_shell_setup

log "Bootstrap complete"
echo "New shell setup:"
echo "  source /opt/ros/${ROS_DISTRO}/setup.bash"
if [[ "${SKIP_BUILD}" -eq 0 ]]; then
  echo "  source ${WS_DIR}/install/setup.bash"
fi
echo
echo "Run mock app:"
echo "  ros2 launch subsea_mock mock_app.launch.py"
echo
echo "Run real app:"
echo "  ros2 launch subsea_bringup rover_app.launch.py"

if [[ "${INSTALL_NOVNC}" -eq 1 ]]; then
  echo
  echo "Optional GUI stack installed (noVNC)."
  echo "You can reuse: .devcontainer/start_gui_stack.sh"
fi

if is_raspberry_pi; then
  echo
  echo "Raspberry Pi accessory auto-config:"
  if [[ "${AUTO_CONFIG_RPI}" -eq 1 ]]; then
    echo "  enabled (PPS GPIO pin: ${PPS_GPIO_PIN})"
  else
    echo "  skipped"
  fi
fi
