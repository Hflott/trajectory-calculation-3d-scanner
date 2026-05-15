#!/usr/bin/env bash
set -Eeuo pipefail

HELPER_PATH="/usr/local/sbin/subsea-rover-gnss-preflight"
SUDOERS_PATH="/etc/sudoers.d/subsea-rover-launch"
SYSTEMCTL_BIN="$(command -v systemctl || echo /usr/bin/systemctl)"
VISUDO_BIN="$(command -v visudo || echo /usr/sbin/visudo)"

if [[ "${EUID}" -eq 0 && -z "${SUDO_USER:-}" ]]; then
  echo "ERROR: run this as the normal rover user, not directly as root." >&2
  exit 1
fi

TARGET_USER="${SUDO_USER:-${USER}}"
if [[ ! "${TARGET_USER}" =~ ^[A-Za-z0-9_.-]+$ ]]; then
  echo "ERROR: unsupported sudoers username '${TARGET_USER}'." >&2
  exit 1
fi

command -v sudo >/dev/null 2>&1 || {
  echo "ERROR: sudo is required." >&2
  exit 1
}
[[ -x "${VISUDO_BIN}" ]] || {
  echo "ERROR: visudo is required." >&2
  exit 1
}

tmp_helper="$(mktemp)"
tmp_sudoers="$(mktemp)"
cleanup() {
  rm -f "${tmp_helper}" "${tmp_sudoers}"
}
trap cleanup EXIT

cat >"${tmp_helper}" <<'EOF'
#!/usr/bin/env bash
set -Eeuo pipefail

detect_gnss_device() {
  if [[ -n "${GNSS_DEVICE:-}" ]]; then
    if [[ -e "${GNSS_DEVICE}" ]]; then
      echo "${GNSS_DEVICE}"
      return 0
    fi
    echo "WARNING: GNSS_DEVICE=${GNSS_DEVICE} does not exist; falling back to auto-detect." >&2
  fi

  local candidates=(
    /dev/ttyAMA0
    /dev/serial0
    /dev/ttyAMA1
    /dev/ttyS0
  )
  local d
  for d in "${candidates[@]}"; do
    if [[ -e "${d}" ]]; then
      echo "${d}"
      return 0
    fi
  done

  if compgen -G "/dev/ttyAMA*" >/dev/null 2>&1; then
    local ttyama_dev
    ttyama_dev="$(ls -1 /dev/ttyAMA* | grep -vx '/dev/ttyAMA10' | sort -V | head -n1 || true)"
    if [[ -n "${ttyama_dev}" ]]; then
      echo "${ttyama_dev}"
      return 0
    fi
  fi

  return 1
}

tty_rule='KERNEL=="ttyAMA[0-9]*", GROUP="dialout", MODE="0660"'
tty_rule_file='/etc/udev/rules.d/99-ttyama.rules'
need_reload_rules="false"
gnss_dev=""
gnss_real=""
gnss_base=""

if gnss_dev="$(detect_gnss_device)"; then
  gnss_real="$(readlink -f "${gnss_dev}" 2>/dev/null || echo "${gnss_dev}")"
  gnss_base="$(basename "${gnss_real}")"
  systemctl disable --now "serial-getty@${gnss_base}.service" >/dev/null 2>&1 || true
else
  gnss_dev="/dev/ttyAMA0"
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

cat >/etc/default/gpsd <<GPSD_EOF
START_DAEMON="true"
USBAUTO="false"
DEVICES="${gnss_dev} /dev/pps0"
GPSD_OPTIONS="-n -b -s 460800"
GPSD_EOF

systemctl enable gpsd.socket chrony >/dev/null 2>&1 || true
systemctl restart gpsd.socket gpsd.service chrony >/dev/null 2>&1 || true
EOF

cat >"${tmp_sudoers}" <<EOF
# Installed by trajectory-calculation-3d-scanner/scripts/setup_rover_passwordless_launch.sh
# Allows the rover launcher to refresh GNSS UART/gpsd/chrony without asking
# for a password every time. The helper is root-owned in /usr/local/sbin.
${TARGET_USER} ALL=(root) NOPASSWD: ${HELPER_PATH}
${TARGET_USER} ALL=(root) NOPASSWD: ${SYSTEMCTL_BIN} restart gpsd.socket gpsd.service chrony
EOF

echo "Installing root-owned GNSS preflight helper:"
echo "  ${HELPER_PATH}"
sudo install -o root -g root -m 0755 "${tmp_helper}" "${HELPER_PATH}"

echo "Validating sudoers rule for user '${TARGET_USER}'..."
sudo "${VISUDO_BIN}" -cf "${tmp_sudoers}" >/dev/null

echo "Installing sudoers rule:"
echo "  ${SUDOERS_PATH}"
sudo install -o root -g root -m 0440 "${tmp_sudoers}" "${SUDOERS_PATH}"

echo "Running one GNSS preflight now..."
sudo -n "${HELPER_PATH}"

echo
echo "Passwordless rover launch is installed."
echo "Future runs of ./scripts/run_rover_field.sh should not ask for a sudo password."
