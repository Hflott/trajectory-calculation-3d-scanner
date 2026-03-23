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
- Adds PPS overlay (`dtoverlay=pps-gpio,gpiopin=11` by default)
- Installs/configures `gpsd` + `chrony` + `pps-tools`
- Attempts to auto-detect GNSS serial device (`/dev/serial/by-id/*`, then common `/dev/tty*` fallbacks)

After first run on Pi, reboot once to apply boot overlay changes.

Optional: install a browser-based GUI stack (Xvfb + noVNC) for headless systems:

```bash
./scripts/bootstrap_ubuntu_24_04.sh --with-novnc
```

Optional Pi flags:
```bash
./scripts/bootstrap_ubuntu_24_04.sh --pps-gpio-pin 11
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

`rover_app.launch.py` now starts a `gpsd_client` component by default, so `/fix` is published automatically when `gpsd` is running and has GNSS data.

Useful GNSS launch args:
```bash
ros2 launch subsea_bringup rover_app.launch.py \
  start_gpsd_client:=true \
  gpsd_host:=127.0.0.1 \
  gpsd_port:=2947
```

Useful IMU launch args (BNO085 over I2C):
```bash
ros2 launch subsea_bringup rover_app.launch.py \
  start_imu_node:=true \
  imu_topic:=/imu/data \
  imu_frame_id:=imu_link \
  imu_rate_hz:=100.0 \
  imu_i2c_address:=74
```

Disable GNSS publisher startup (if you run another GNSS ROS node):
```bash
ros2 launch subsea_bringup rover_app.launch.py start_gpsd_client:=false
```

### One-command field startup (touchscreen friendly)
Start app + GNSS publisher + localization with one command:
```bash
./scripts/run_rover_field.sh
```
This also enables the BNO085 IMU node (`start_imu_node:=true`) when available.

One-time BNO085 Python dependencies on Raspberry Pi:
```bash
sudo apt-get install -y python3-pip python3-libgpiod python3-dev i2c-tools
sudo pip3 install --break-system-packages adafruit-blinka adafruit-circuitpython-bno08x
```

Quick IMU checks:
```bash
sudo i2cdetect -y -r 1     # expect 0x4a or 0x4b
ros2 topic hz /imu/data
ros2 topic echo /imu/data --once
```

Quick field diagnostics (gpsd/PPS/chrony + ROS topic checks):
```bash
./scripts/check_rover_field.sh
```

Create a shareable diagnostics bundle (single command, includes gpsd/PPS/chrony/ROS checks + recent capture metadata/session logs):
```bash
./scripts/collect_rover_diagnostics.sh
```

Organize imported `sessions/` + `diagnostics/` folders in this repo into date-based layout:
```bash
./scripts/organize_field_data.sh
```

Useful options:
```bash
./scripts/run_rover_field.sh --still
./scripts/run_rover_field.sh --no-localization
./scripts/run_rover_field.sh --skip-service-restart
```

Create Raspberry Pi desktop shortcut + icon:
```bash
./scripts/install_desktop_shortcut.sh
```
The installed shortcut starts with `--skip-service-restart` to avoid sudo/password prompts.
Recommended one-time setup so services start at boot:
```bash
sudo systemctl enable --now gpsd.socket chrony
```

### Session Recording (UI)
The UI now has a `Start Session` / `Stop Session` button in the top bar.
It also shows a live GNSS lock badge:
- `GNSS Lock: YES` -> safe to start logging
- `GNSS Lock: NO` / `waiting` -> session start is blocked by default
It also shows a live corrections badge:
- `Corrections: ON` -> rover solution is using differential/RTK corrections
- `Corrections: OFF` / `waiting` -> no corrections in current solution yet
The GNSS tab now includes:
- `Ready to Log` summary
- fix type (`NO_FIX/FIX/SBAS/GBAS-RTK`)
- estimated horizontal/vertical accuracy from covariance
- fix/time_ref/imu freshness summary
- card-based layout and a GNSS quality bar (0-100, red/yellow/green)
The Diagnostics tab includes a live readiness summary with explicit `OK/NO` lines for:
- capture service
- GNSS lock
- corrections
- cam0/cam1 stream health
- session state
The Diagnostics tab also has a `Collect Diagnostics Bundle` button that runs
`scripts/collect_rover_diagnostics.sh` directly from the UI and reports done/failed state.

When started, it runs continuous `ros2 bag record` logging to:
- `~/captures/sessions/YYYY/MM/DD/sess_YYYYmmdd_HHMMSS/bag`
- with session metadata in `session_manifest.json`
- and recorder stdout/stderr in `rosbag_record.log`

Default recorded topics:
- `/imu/data`
- `/fix`
- `/time_reference`
- `/odometry/local`
- `/odometry/global`
- `/capture/events`
- `/capture/debug`

Optional image-stream recording can be enabled via UI node parameters:
Edit `~/.config/subsea_ui/config.json` and set:
```json
{
  "require_gnss_lock_for_session": true,
  "max_fix_age_ms_for_lock": 2000,
  "session_record_images": true,
  "session_cam0_topic": "/cam0/camera/image_raw",
  "session_cam1_topic": "/cam1/camera/image_raw"
}
```

## Capture mode for deblurring
`subsea_capture` now defaults to `capture_mode:=stream` in rover bringup. This keeps previews running and captures directly from live ROS image streams, preserving frame timestamps for motion-compensation workflows.

Bringup now uses split topics by default:
- capture stream input: `/cam0/camera/image_raw`, `/cam1/camera/image_raw`
- UI preview relay output: `/cam0/preview/image_raw`, `/cam1/preview/image_raw`

This allows lower-load UI preview (`preview_ui_*`) without reducing capture-stream quality (`preview_*`).

For each capture session it writes:
- `*_cam0.jpg` / `*_cam1.jpg`
- `*_meta.json` with trigger timestamp, per-image timestamps, and nearest GNSS/IMU/TimeReference + odometry (`/odometry/local`, `/odometry/global`) samples

Live stream-capture timing diagnostics are also published on `/capture/debug` and shown in the UI under `Last Capture -> Details / Log`.

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
- GPIO chip `/dev/gpiochip4` (Raspberry Pi 5 header GPIO controller)
- pin `GPIO24` (`gpio_button_pin:=24`, BCM numbering)
- active-low trigger (button to `GND`)
- debounce `40 ms` (`gpio_button_debounce_ms:=40`)
- physical wiring for default: pin `18` (GPIO24) to pin `20` (GND)

Example:
```bash
ros2 launch subsea_bringup rover_app.launch.py \
  capture_mode:=stream \
  gpio_button_pin:=24
```

If preview is laggy on Raspberry Pi, lower **UI preview relay** load first:

```bash
ros2 launch subsea_bringup rover_app.launch.py \
  capture_mode:=stream \
  preview_ui_width:=640 \
  preview_ui_height:=360 \
  preview_ui_fps:=10 \
  ui_fps:=10
```

If you need sharper stream captures for deblurring, keep/increase capture stream settings separately:

```bash
ros2 launch subsea_bringup rover_app.launch.py \
  capture_mode:=stream \
  preview_width:=1280 \
  preview_height:=720 \
  preview_fps:=12 \
  preview_ui_width:=640 \
  preview_ui_height:=360 \
  preview_ui_fps:=10
```

If one preview camera exits with `failed to start camera` / `Broken pipe`, reduce load further:

```bash
ros2 launch subsea_bringup rover_app.launch.py \
  capture_mode:=still \
  preview_width:=480 \
  preview_height:=270 \
  preview_fps:=10 \
  preview_format:=RGB888 \
  ui_fps:=8
```

Wiring recommendation:
- one side of button to `GPIO24`
- other side to `GND`
- keep `GPIO11` reserved for PPS if you use GNSS PPS there

If you see:
- `python gpiod import failed`: install `python3-libgpiod`
- `failed to open chip '/dev/gpiochip0': [Errno 13] Permission denied`: add your user to `gpio` group, then re-login:

```bash
sudo apt-get install -y python3-libgpiod gpiod
sudo usermod -aG gpio $USER
newgrp gpio
```

### PPS note (GPIO11, Raspberry Pi 5)
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

Also verify which `libcamera` your `camera_node` is linked against:

```bash
ldd "$(ros2 pkg prefix camera_ros)/lib/camera_ros/camera_node" | grep libcamera
```

If it points to `/opt/ros/jazzy/.../libcamera.so`, switch to Raspberry Pi/system libcamera and rebuild:

```bash
sudo apt-get remove -y "ros-jazzy-libcamera*"
sudo apt-get install -y libcamera-dev
cd ~/trajectory-calculation-3d-scanner/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y --skip-keys "ament_python libcamera"
rm -rf build/camera_ros install/camera_ros
colcon build --symlink-install --packages-select camera_ros subsea_capture subsea_bringup subsea_ui
source install/setup.bash
```
