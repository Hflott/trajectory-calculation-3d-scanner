#!/usr/bin/env bash
set -euo pipefail

export DISPLAY="${DISPLAY:-:1}"

if ! pgrep -f "Xvfb ${DISPLAY}" >/dev/null; then
  Xvfb "${DISPLAY}" -screen 0 1920x1080x24 -ac -nolisten tcp >/tmp/xvfb.log 2>&1 &
fi

if ! pgrep -x fluxbox >/dev/null; then
  fluxbox >/tmp/fluxbox.log 2>&1 &
fi

if ! pgrep -f "x11vnc .* -rfbport 5901" >/dev/null; then
  x11vnc -display "${DISPLAY}" -rfbport 5901 -forever -shared -nopw >/tmp/x11vnc.log 2>&1 &
fi

if ! pgrep -f "novnc_proxy .*6080" >/dev/null; then
  /usr/share/novnc/utils/novnc_proxy --listen 6080 --vnc localhost:5901 >/tmp/novnc.log 2>&1 &
fi

echo "GUI stack started on DISPLAY=${DISPLAY}"
echo "noVNC: http://localhost:6080/vnc.html"
