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

tty_rule='KERNEL=="ttyAMA0", GROUP="dialout", MODE="0660"'
tty_rule_file='/etc/udev/rules.d/99-ttyama0.rules'
need_reload_rules="false"

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

systemctl enable gpsd.socket chrony >/dev/null 2>&1 || true
systemctl restart gpsd.socket gpsd.service chrony >/dev/null 2>&1 || true
EOF

cat >"${tmp_sudoers}" <<EOF
# Installed by trajectory-calculation-3d-scanner/scripts/setup_rover_passwordless_launch.sh
# Allows the rover launcher to refresh GNSS UART/gpsd/chrony without asking
# for a password every time. The helper is root-owned in /usr/local/sbin.
${TARGET_USER} ALL=(root) NOPASSWD: ${HELPER_PATH}
${TARGET_USER} ALL=(root) NOPASSWD: ${SYSTEMCTL_BIN} restart gpsd.socket chrony
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
