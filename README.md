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

## Manual commands (if needed)
```bash
cd /workspaces/trajectory-calculation-3d-scanner/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
ros2 launch subsea_mock mock_app.launch.py
```
