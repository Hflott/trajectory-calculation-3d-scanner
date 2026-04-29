#!/usr/bin/env bash
set -Eeuo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
APP_NAME="Rover App"
APP_ID="subsea-rover-field"
CAM0_ORIENTATION=""
CAM1_ORIENTATION=""

RUN_SCRIPT="${ROOT_DIR}/scripts/run_rover_field.sh"
ICON_SRC="${ROOT_DIR}/assets/subsea_rover_icon.svg"

usage() {
  cat <<'EOF'
Usage: ./scripts/install_desktop_shortcut.sh [options]

Options:
  --cam0-orientation DEG  Set cam0_orientation launch arg in desktop launcher (0/90/180/270)
  --cam1-orientation DEG  Set cam1_orientation launch arg in desktop launcher (0/90/180/270)
  -h, --help              Show this help
EOF
}

normalize_orientation() {
  local raw="$1"
  if [[ ! "${raw}" =~ ^-?[0-9]+$ ]]; then
    echo "ERROR: orientation must be an integer (got '${raw}')" >&2
    exit 1
  fi
  # Normalize any integer to [0, 90, 180, 270].
  local norm=$(( ((raw / 90) % 4 + 4) % 4 * 90 ))
  echo "${norm}"
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --cam0-orientation)
      [[ $# -ge 2 ]] || { echo "ERROR: --cam0-orientation requires a value" >&2; exit 1; }
      CAM0_ORIENTATION="$(normalize_orientation "$2")"
      shift 2
      ;;
    --cam1-orientation)
      [[ $# -ge 2 ]] || { echo "ERROR: --cam1-orientation requires a value" >&2; exit 1; }
      CAM1_ORIENTATION="$(normalize_orientation "$2")"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "ERROR: Unknown argument: $1" >&2
      usage
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

ORIENT_ARGS=""
if [[ -n "${CAM0_ORIENTATION}" ]]; then
  ORIENT_ARGS="${ORIENT_ARGS} cam0_orientation:=${CAM0_ORIENTATION}"
fi
if [[ -n "${CAM1_ORIENTATION}" ]]; then
  ORIENT_ARGS="${ORIENT_ARGS} cam1_orientation:=${CAM1_ORIENTATION}"
fi

cat > "${LAUNCHER_FILE}" <<EOF
#!/usr/bin/env bash
set -Eeuo pipefail
cd "${ROOT_DIR}"
# Touchscreen-friendly default: avoid sudo prompt on launch.
# gpsd/chrony should be enabled at boot; this still allows extra launch args.
exec "${RUN_SCRIPT}" --skip-service-restart${ORIENT_ARGS} "\$@"
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

# Remove legacy desktop shortcut name if present.
if [[ -f "${LEGACY_DESKTOP_FILE}" ]]; then
  rm -f "${LEGACY_DESKTOP_FILE}" || true
fi

if command -v desktop-file-validate >/dev/null 2>&1; then
  desktop-file-validate "${APP_FILE}" >/dev/null 2>&1 || true
fi

echo "Desktop shortcut installed."
echo "  Desktop icon: ${DESKTOP_FILE}"
echo "  App menu:     ${APP_FILE}"
if [[ -n "${CAM0_ORIENTATION}" || -n "${CAM1_ORIENTATION}" ]]; then
  echo "  Orientation args:${ORIENT_ARGS}"
fi
echo
echo "If first launch is blocked, right-click the icon and choose 'Allow Launching'."
