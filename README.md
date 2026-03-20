# trajectory-calculation-3d-scanner
Trajectory calculation based on RTK-GPS and IMU for movement compensation in image data. This project is intended for use with stereo photogrammetry and a UAV.

## Cross-platform development (Mac + Windows)
Use the devcontainer on both machines. This keeps ROS, Python, Qt, and build tools identical.

### Host prerequisites
- Docker Desktop (or OrbStack on macOS)
- VS Code
- VS Code extension: Dev Containers

Windows (PowerShell, optional helper):
```powershell
./scripts/bootstrap_windows.ps1 -InstallApps
```

### First setup (inside devcontainer terminal)
```bash
./scripts/devcontainer_setup.sh
```

### Run GUI mock app (inside devcontainer terminal)
```bash
./scripts/run_mock_gui.sh
```

Open on host browser:
`http://localhost:6080/vnc.html`

The real bringup uses Raspberry Pi cameras and `rpicam-still`, so mock mode is intended for desktop simulation only.

## GNSS + IMU localization (robot_localization)
A `subsea_localization` package is included with:
- `ekf_local` (IMU + optional wheel/visual odom)
- `navsat_transform_node` (GNSS to odom)
- `ekf_global` (local odom + GPS odom)

Run standalone:
```bash
cd /workspaces/trajectory-calculation-3d-scanner/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch subsea_localization localization.launch.py \
  imu_topic:=/imu/data \
  gps_fix_topic:=/fix \
  odom_input_topic:=/odometry/wheel
```

Or from main bringup:
```bash
ros2 launch subsea_bringup rover_app.launch.py start_localization:=true
```

### PPS note (GPIO23, Raspberry Pi 5)
`robot_localization` does not configure PPS itself. PPS must be enabled in Linux (`pps-gpio` and time-sync daemon such as `chrony`/`gpsd`) so GNSS/IMU timestamps are accurate before fusion.

## Manual commands (if needed)
```bash
cd /workspaces/trajectory-calculation-3d-scanner/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
ros2 launch subsea_mock mock_app.launch.py
```
