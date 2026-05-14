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
For strict PPS-timed deblurring you also need `/time_reference`; use `use_gpsd_json_bridge:=true` (the bridge publishes both `/fix` and `/time_reference`).

Useful GNSS launch args:
```bash
ros2 launch subsea_bringup rover_app.launch.py \
  start_gpsd_client:=true \
  use_gpsd_json_bridge:=true \
  gpsd_host:=127.0.0.1 \
  gpsd_port:=2947
```
`use_gpsd_json_bridge:=true` enables `subsea_bringup/gpsd_json_fix_bridge`, which publishes both `/fix` and `/time_reference` from gpsd TPV data.

Useful IMU launch args (BNO085 over I2C):
```bash
ros2 launch subsea_bringup rover_app.launch.py \
  start_imu_node:=true \
  imu_topic:=/imu/data \
  imu_frame_id:=imu_link \
  imu_rate_hz:=100.0 \
  imu_i2c_address:=74 \
  imu_i2c_bus:=1 \
  imu_timestamp_mode:=read_end
```

Disable GNSS publisher startup (if you run another GNSS ROS node):
```bash
ros2 launch subsea_bringup rover_app.launch.py start_gpsd_client:=false
```

### One-command field startup (touchscreen friendly)
Start the normal rover field setup with one command:
```bash
./scripts/run_rover_field.sh
```

The normal startup parameters live near the top of `scripts/run_rover_field.sh`
in the `FIELD_LAUNCH_ARGS` block. Edit that block directly for the defaults you
want to use in the field, for example IMU rate, I2C bus, preview size,
localization, GPS bridge, camera orientation, color tuning, and deblur settings.

One-off overrides still work, but they are optional:
```bash
./scripts/run_rover_field.sh imu_rate_hz:=200.0
./scripts/run_rover_field.sh capture_mode:=stream
```

The default field script enables GNSS, `/time_reference`, the BNO085 IMU,
localization, and motion deblur. It uses a split camera workflow:
- low-resolution live preview for the UI
- high-resolution `rpicam-still` capture for saved image pairs and deblur output

One-time BNO085 Python dependencies on Raspberry Pi:
```bash
sudo apt-get install -y python3-pip python3-libgpiod python3-dev i2c-tools
sudo pip3 install --break-system-packages adafruit-blinka adafruit-circuitpython-bno08x adafruit-extended-bus
```

Quick IMU checks:
```bash
sudo i2cdetect -y -r 1     # expect 0x4a or 0x4b
ros2 topic hz /imu/data
ros2 topic echo /imu/data --once
```

If BNO085 is moved from GPIO 2/3 to GPIO 5/6, create a software I2C bus first:
```bash
echo "dtoverlay=i2c-gpio,bus=3,i2c_gpio_sda=5,i2c_gpio_scl=6,i2c_gpio_delay_us=2" | sudo tee -a /boot/firmware/config.txt
sudo reboot
```
After reboot:
```bash
ls /dev/i2c-3
sudo i2cdetect -y -r 3     # expect 0x4a or 0x4b
```
Then set `imu_i2c_bus:=3` in `scripts/run_rover_field.sh`.

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

If your cameras appear upside down after a sensor swap, set
`cam0_orientation:=180` and/or `cam1_orientation:=180` in
`scripts/run_rover_field.sh`.

Create Raspberry Pi desktop shortcut + icon:
```bash
./scripts/install_desktop_shortcut.sh
```
The installed shortcut follows the same `FIELD_LAUNCH_ARGS` defaults in
`scripts/run_rover_field.sh`; it does not bake in separate camera/IMU/GNSS
settings.

The desktop shortcut starts with `--skip-gnss-preflight --skip-service-restart`
to avoid sudo/password prompts.
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
The field script defaults to a split preview/capture workflow:
- live UI preview uses low-load ROS image streams
- capture/deblur uses high-resolution `rpicam-still` images

This keeps the touchscreen feed responsive without forcing saved scan images to
use the same low preview resolution. During still capture, managed previews are
paused briefly so the high-resolution capture can use the camera devices, then
restarted automatically.

Bringup now uses split topics by default:
- live camera stream input: `/cam0/camera/image_raw`, `/cam1/camera/image_raw`
- UI preview relay output: `/cam0/preview/image_raw`, `/cam1/preview/image_raw`

For field capture quality, edit `capture_width`, `capture_height`, and
`capture_quality` in `scripts/run_rover_field.sh`. For the IMX296 cameras used
on the rover, the default high-resolution still capture is `1456x1088`.
For simple color tuning, edit `capture_awb`, `capture_saturation`, and
`preview_ui_saturation` in the same script. The current field preset uses
`capture_awb:=auto`, `capture_saturation:=0.6`, and
`preview_ui_saturation:=0.6`.

For each capture session it writes:
- `*_cam0.jpg` / `*_cam1.jpg`
- `*_cam0_deblur.jpg` / `*_cam1_deblur.jpg` (IMU motion-aware deblur output)
- `*_cam0_rpicam_meta.json` / `*_cam1_rpicam_meta.json` with the camera-reported still metadata, including `ExposureTime` when available
- `*_meta.json` with trigger timestamp, per-image timestamps, and nearest GNSS/IMU/TimeReference + odometry (`/odometry/local`, `/odometry/global`) samples
- `*_trajectory.csv` (interpolated trajectory samples, default 100 Hz around trigger)

Capture metadata now also includes:
- interpolated odometry at each camera timestamp (`interp_odom_local`, `interp_odom_global`)
- a trajectory bundle sampled at `trajectory_sample_rate_hz` (default `100.0`)
- per-camera deblur diagnostics with exposure-window gyro integration (`exposure_start/end`, `gyro_samples_used`, `gyro_bias`, `delta_theta_raw_rad`, `delta_theta_rad`, blur vector, PSF settings, output path)
- rig extrinsics used by deblur (`rig_extrinsics`), including IMU/camera/GNSS positions and the IMU-to-camera rotation matrices

Capture timing diagnostics are also published on `/capture/debug` and shown in the UI under `Last Capture -> Details / Log`.

### Rover rig extrinsics
The field script contains the measured rover geometry used for motion deblur:
```bash
deblur_use_rig_extrinsics:=true
rig_imu_position_m:=0.080,0.000,0.020
rig_cam0_position_m:=-0.367,-0.003,0.063
rig_cam1_position_m:=0.354,-0.003,0.063
rig_gnss_left_position_m:=-0.540,0.000,0.050
rig_gnss_right_position_m:=0.540,0.000,0.050
rig_imu_to_base_rotation:=-1,0,0,0,-1,0,0,0,1
rig_cam0_base_to_camera_rotation:=1,0,0,0,0,-1,0,1,0
rig_cam1_base_to_camera_rotation:=1,0,0,0,0,-1,0,1,0
```

Frame convention:
- base `+X`: cam0 toward cam1
- base `+Y`: camera viewing direction
- base `+Z`: up
- camera `+X`: image right
- camera `+Y`: image down
- camera `+Z`: optical forward

The current default uses the extrinsic rotations for rotational deblur. Camera
position offsets are recorded in metadata and are only used in the deblur
projection when `deblur_use_translation:=true`, because translation correction
depends on scene depth.

### Deblur timing and gyro bias tuning
Deblur uses the image timestamp plus optional offsets before choosing the IMU
exposure interval:
```bash
deblur_timestamp_offset_ms:=0.0
deblur_cam0_timestamp_offset_ms:=0.0
deblur_cam1_timestamp_offset_ms:=0.0
```

Use the common offset first. If one camera consistently needs a different
timing correction, use the per-camera offset. The adjusted stamp and offset are
written to every capture metadata file.

Gyro bias correction is enabled by default. While the rover is stationary,
`capture_service` estimates a rolling gyro bias and subtracts it during exposure
integration:
```bash
deblur_gyro_bias_enable:=true
deblur_gyro_bias_window_s:=2.0
deblur_gyro_bias_min_samples:=25
deblur_gyro_bias_stationary_max_rate_rad_s:=0.025
deblur_gyro_bias_stationary_max_std_rad_s:=0.010
deblur_gyro_bias_max_age_s:=30.0
```

The bias estimator state and the applied bias are saved under `gyro_bias` in the
per-camera deblur diagnostics.

In still-capture mode, `capture_service` now passes `--metadata` to
`rpicam-still` and uses the per-image `ExposureTime` from that metadata for the
deblur exposure window. If the camera metadata is missing or does not contain
exposure, it falls back to `deblur_exposure_time_us` or `deblur_exposure_ms`.

If you need timestamped stream captures instead of high-resolution still
captures, override:
```bash
ros2 launch subsea_bringup rover_app.launch.py \
  start_localization:=true \
  capture_mode:=stream
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
  capture_mode:=still \
  gpio_button_pin:=24
```

If preview is laggy on Raspberry Pi, lower the live preview load:

```bash
ros2 launch subsea_bringup rover_app.launch.py \
  capture_mode:=still \
  preview_width:=640 \
  preview_height:=360 \
  preview_fps:=10 \
  preview_ui_width:=480 \
  preview_ui_height:=270 \
  preview_ui_fps:=8 \
  ui_fps:=10
```

For timestamped stream-capture experiments, raise the stream settings separately:

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

Trajectory + deblur tuning example (Pi 5 field mode):

```bash
ros2 launch subsea_bringup rover_app.launch.py \
  capture_mode:=still \
  start_localization:=true \
  preview_width:=640 \
  preview_height:=360 \
  preview_fps:=10 \
  capture_width:=1456 \
  capture_height:=1088 \
  capture_quality:=95 \
  trajectory_sample_rate_hz:=100.0 \
  trajectory_window_ms:=1000.0 \
  deblur_exposure_time_us:=3000 \
  deblur_image_stamp_reference:=midpoint \
  deblur_timestamp_source:=pps_disciplined_system_clock \
  deblur_allow_nearest_fallback:=false \
  deblur_require_time_reference:=true \
  deblur_max_time_reference_age_ms:=2000.0 \
  deblur_strength:=1.0 \
  deblur_iterations:=12
```

Notes:
- True GNSS update rate still depends on your GNSS receiver/output configuration.
- The EKF/navsat defaults in `subsea_localization` are now set to 100 Hz prediction/update cadence.

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
