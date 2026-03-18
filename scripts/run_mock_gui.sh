#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS_DIR="${ROOT_DIR}/ros2_ws"

"${ROOT_DIR}/.devcontainer/start_gui_stack.sh"

source /opt/ros/jazzy/setup.bash
source "${WS_DIR}/install/setup.bash"

echo
echo "Open in host browser:"
echo "  http://localhost:6080/vnc.html"
echo
echo "Starting ROS mock app..."

ros2 launch subsea_mock mock_app.launch.py
