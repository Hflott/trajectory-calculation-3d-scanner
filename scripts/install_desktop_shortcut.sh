#!/usr/bin/env bash
set -Eeuo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
APP_NAME="Rover App"
APP_ID="subsea-rover-field"

RUN_SCRIPT="${ROOT_DIR}/scripts/run_rover_field.sh"
ICON_SRC="${ROOT_DIR}/assets/subsea_rover_icon.svg"

usage() {
  cat <<'EOF'
Usage:
  ./scripts/install_desktop_shortcut.sh

The desktop icon follows the defaults in scripts/run_rover_field.sh.
Edit FIELD_LAUNCH_ARGS there for camera orientation, IMU rate, GPS mode,
preview settings, localization, and deblur settings.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "ERROR: desktop shortcut options were removed; edit FIELD_LAUNCH_ARGS in scripts/run_rover_field.sh instead." >&2
      exit 1
      ;;
  esac
done

if [[ ! -x "${RUN_SCRIPT}" ]]; then
  echo "ERROR: launcher script not found/executable: ${RUN_SCRIPT}" >&2
  exit 1
fi
if [[ ! -f "${ICON_SRC}" ]]; then
  echo "ERROR: icon file not found: ${ICON_SRC}" >&2
  exit 1
fi

DESKTOP_DIR="${HOME}/Desktop"
if command -v xdg-user-dir >/dev/null 2>&1; then
  maybe_desktop="$(xdg-user-dir DESKTOP 2>/dev/null || true)"
  if [[ -n "${maybe_desktop}" && "${maybe_desktop}" != "${HOME}" ]]; then
    DESKTOP_DIR="${maybe_desktop}"
  fi
fi

ICON_DIR="${HOME}/.local/share/icons/hicolor/scalable/apps"
APP_DIR="${HOME}/.local/share/applications"
BIN_DIR="${HOME}/.local/bin"
LAUNCHER_FILE="${BIN_DIR}/${APP_ID}-launcher.sh"
DESKTOP_FILE="${DESKTOP_DIR}/${APP_NAME}.desktop"
LEGACY_DESKTOP_FILE="${DESKTOP_DIR}/Subsea Rover Field.desktop"
APP_FILE="${APP_DIR}/${APP_ID}.desktop"
ICON_FILE="${ICON_DIR}/${APP_ID}.svg"

mkdir -p "${DESKTOP_DIR}" "${ICON_DIR}" "${APP_DIR}" "${BIN_DIR}"
cp -f "${ICON_SRC}" "${ICON_FILE}"
chmod 0644 "${ICON_FILE}"

cat > "${LAUNCHER_FILE}" <<EOF
#!/usr/bin/env bash
set -Eeuo pipefail
cd "${ROOT_DIR}"
# Run the same field startup as the terminal command. For passwordless GNSS
# preflight, run scripts/setup_rover_passwordless_launch.sh once.
exec "${RUN_SCRIPT}" "\$@"
EOF
chmod +x "${LAUNCHER_FILE}"

cat > "${DESKTOP_FILE}" <<EOF
[Desktop Entry]
Version=1.0
Type=Application
Name=${APP_NAME}
Comment=Launch Subsea rover field app
Exec=${LAUNCHER_FILE}
Path=${ROOT_DIR}
Icon=${ICON_FILE}
Terminal=true
StartupNotify=true
Categories=Utility;
EOF
chmod +x "${DESKTOP_FILE}"
cp -f "${DESKTOP_FILE}" "${APP_FILE}"
chmod 0644 "${APP_FILE}"

if [[ -f "${LEGACY_DESKTOP_FILE}" ]]; then
  rm -f "${LEGACY_DESKTOP_FILE}" || true
fi

if command -v desktop-file-validate >/dev/null 2>&1; then
  desktop-file-validate "${APP_FILE}" >/dev/null 2>&1 || true
fi

echo "Desktop shortcut installed."
echo "  Desktop icon: ${DESKTOP_FILE}"
echo "  App menu:     ${APP_FILE}"
echo "  Launcher:     ${LAUNCHER_FILE}"
echo
echo "The icon follows FIELD_LAUNCH_ARGS in:"
echo "  ${RUN_SCRIPT}"
echo
echo "If first launch is blocked, right-click the icon and choose 'Allow Launching'."
