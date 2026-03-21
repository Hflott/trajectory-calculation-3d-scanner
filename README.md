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

## Native Ubuntu 24.04 bootstrap (new device)
For a fresh Ubuntu 24.04 LTS server install, this script installs ROS 2 Jazzy, required apt repositories, camera dependencies, rosdep dependencies, git submodules (including `camera_ros`), and builds the workspace.

```bash
git clone --recurse-submodules https://github.com/Hflott/trajectory-calculation-3d-scanner.git
cd trajectory-calculation-3d-scanner
./scripts/bootstrap_ubuntu_24_04.sh
```

On Raspberry Pi, the bootstrap also auto-configures common accessories for your stack:
- Enables `UART` + `I2C`
- Adds PPS overlay (`dtoverlay=pps-gpio,gpiopin=23` by default)
- Installs/configures `gpsd` + `chrony` + `pps-tools`
- Attempts to auto-detect GNSS serial device (`/dev/serial/by-id/*`, then common `/dev/tty*` fallbacks)

After first run on Pi, reboot once to apply boot overlay changes.

Optional: install a browser-based GUI stack (Xvfb + noVNC) for headless systems:

```bash
./scripts/bootstrap_ubuntu_24_04.sh --with-novnc
```

Optional Pi flags:
```bash
./scripts/bootstrap_ubuntu_24_04.sh --pps-gpio-pin 23
./scripts/bootstrap_ubuntu_24_04.sh --no-rpi-autoconfig
```

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

## Capture mode for deblurring
`subsea_capture` now defaults to `capture_mode:=stream` in rover bringup. This keeps previews running and captures directly from live ROS image streams, preserving frame timestamps for motion-compensation workflows.

For each capture session it writes:
- `*_cam0.jpg` / `*_cam1.jpg`
- `*_meta.json` with trigger timestamp, per-image timestamps, and nearest GNSS/IMU/TimeReference samples

If you need the old still-capture behavior (`rpicam-still`, with preview pause), override:
```bash
ros2 launch subsea_bringup rover_app.launch.py \
  start_localization:=true \
  capture_mode:=still
```

## GPIO button trigger (Raspberry Pi)
You can trigger captures from a physical button wired to a GPIO input.

Default launch settings:
- enabled by default (`enable_gpio_button:=true`)
- pin `GPIO24` (`gpio_button_pin:=24`, BCM numbering)
- active-low trigger (button to `GND`)

Example:
```bash
ros2 launch subsea_bringup rover_app.launch.py \
  capture_mode:=stream \
  gpio_button_pin:=24
```

Wiring recommendation:
- one side of button to `GPIO24`
- other side to `GND`
- keep `GPIO23` reserved for PPS if you use GNSS PPS there

If you see:
- `python gpiod import failed`: install `python3-libgpiod`
- `failed to open chip '/dev/gpiochip0': [Errno 13] Permission denied`: add your user to `gpio` group, then re-login:

```bash
sudo apt-get install -y python3-libgpiod gpiod
sudo usermod -aG gpio $USER
newgrp gpio
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

### Camera feed troubleshooting on Raspberry Pi
If preview logs show `camera_ros` exits with `no cameras available`, check whether you are using the apt binary:

```bash
ros2 pkg prefix camera_ros
```

If this prints `/opt/ros/jazzy`, build workspace `camera_ros`:

```bash
cd ~/trajectory-calculation-3d-scanner/ros2_ws/src
git clone --depth 1 https://github.com/christianrauch/camera_ros.git
cd ..
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y --skip-keys ament_python
colcon build --symlink-install --packages-select camera_ros subsea_capture subsea_bringup subsea_ui
source install/setup.bash
ros2 pkg prefix camera_ros
```

The final `ros2 pkg prefix camera_ros` should point to your workspace `install/` path, not `/opt/ros/jazzy`.
