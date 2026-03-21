#!/usr/bin/env python3
import json
import os
import re
import time
import subprocess
import signal
import threading
from collections import deque
from datetime import datetime
from typing import Any, Deque, Dict, List, Tuple, Optional, Callable

import cv2
import numpy as np
import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.node import Node
from builtin_interfaces.msg import Time as TimeMsg

from cv_bridge import CvBridge
from ament_index_python.packages import get_package_prefix

from subsea_interfaces.action import CapturePair as CapturePairAction
from subsea_interfaces.srv import CapturePair
from sensor_msgs.msg import Image, NavSatFix, TimeReference, Imu
from rcl_interfaces.msg import SetParametersResult
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy


def now_ros_time(node: Node) -> TimeMsg:
    return node.get_clock().now().to_msg()


def _popen_group(cmd: List[str], env: Optional[dict] = None) -> subprocess.Popen:
    return subprocess.Popen(cmd, preexec_fn=os.setsid, env=env)


def _stop_proc(proc: subprocess.Popen, timeout_s: float = 2.5) -> None:
    if proc.poll() is not None:
        return
    try:
        os.killpg(proc.pid, signal.SIGINT)
    except Exception:
        pass

    t0 = time.monotonic()
    while (time.monotonic() - t0) < timeout_s:
        if proc.poll() is not None:
            return
        time.sleep(0.05)

    try:
        os.killpg(proc.pid, signal.SIGKILL)
    except Exception:
        pass


def _write_params_file(path: str, params: dict) -> None:
    lines = ["/**:", "  ros__parameters:"]
    for k, v in params.items():
        if isinstance(v, bool):
            lines.append(f"    {k}: {'true' if v else 'false'}")
        elif isinstance(v, (int, float)):
            lines.append(f"    {k}: {v}")
        elif isinstance(v, (list, tuple)):
            inner = ", ".join(str(x) for x in v)
            lines.append(f"    {k}: [{inner}]")
        else:
            s = str(v).replace('"', '\\"')
            lines.append(f"    {k}: \"{s}\"")
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")


def _camera_ros_exe_path() -> str:
    # Use the overlayed camera_ros install (camera_ws), not hard-coded /opt/ros
    prefix = get_package_prefix("camera_ros")
    return os.path.join(prefix, "lib", "camera_ros", "camera_node")


def _dev_paths() -> List[str]:
    # PiSP pipeline devices (Pi 5): media0/1 ISP, media2/3 CFE + some /dev/video* nodes
    candidates = [
        "/dev/media0", "/dev/media1", "/dev/media2", "/dev/media3",
        "/dev/video0", "/dev/video1", "/dev/video2", "/dev/video3", "/dev/video4", "/dev/video5",
    ]
    return [p for p in candidates if os.path.exists(p)]


def _devices_in_use(devs: List[str]) -> bool:
    # fuser returns 0 if any process is using it; 1 if none; 2 on error
    for d in devs:
        try:
            p = subprocess.run(["fuser", d], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            if p.returncode == 0:
                return True
        except Exception:
            # If fuser isn't available, fall back to "assume free"
            return False
    return False


def _parse_camera_count(output: str) -> Optional[int]:
    text = output or ""
    if re.search(r"no cameras available", text, re.IGNORECASE):
        return 0

    m = re.search(r"Available cameras:\s*(\d+)", text)
    if m:
        try:
            return int(m.group(1))
        except Exception:
            return None

    if "Available cameras:" in text or "Available cameras" in text:
        lines = text.splitlines()
        count = 0
        start = False
        for line in lines:
            if not start:
                if "Available cameras" in line:
                    start = True
                continue
            # Handle both "0: imx..." and "0 : imx..." formats.
            if re.match(r"^\s*\d+\s*:", line):
                count += 1
        return count

    return None


def _libcamera_camera_count() -> Optional[int]:
    # Returns number of cameras if a libcamera CLI is available, else None.
    cmds = (
        ["rpicam-hello", "--list-cameras"],
        ["libcamera-hello", "--list-cameras"],
        ["cam", "-l"],
        ["cam", "--list"],
    )
    for cmd in cmds:
        try:
            p = subprocess.run(cmd, capture_output=True, text=True, timeout=3.0)
        except FileNotFoundError:
            continue
        except Exception:
            continue
        out = (p.stdout or "") + "\n" + (p.stderr or "")
        count = _parse_camera_count(out)
        if count is not None:
            return count
    return None


def _has_camera_device_hint(devs: List[str]) -> bool:
    # Heuristic fallback if libcamera CLI isn't available.
    for p in devs:
        if p.startswith("/dev/video") or p.startswith("/dev/media"):
            return True
    return False


def _sanitize_preview_ld_library_path(ld_path: str) -> str:
    keep: List[str] = []
    for p in ld_path.split(":"):
        s = p.strip()
        if not s:
            continue
        low = s.lower()
        # Common local overrides that break camera_ros/libcamera discovery.
        if low.startswith("/usr/local/lib"):
            continue
        if "/camera_ws/" in low:
            continue
        keep.append(s)
    return ":".join(keep)


def _linked_libcamera_path(exe_path: str) -> Optional[str]:
    try:
        p = subprocess.run(
            ["ldd", exe_path],
            capture_output=True,
            text=True,
            timeout=2.0,
        )
    except Exception:
        return None
    if p.returncode != 0:
        return None
    txt = (p.stdout or "") + "\n" + (p.stderr or "")
    for line in txt.splitlines():
        if "libcamera.so" not in line:
            continue
        m = re.search(r"=>\s+(\S+)", line)
        if m:
            return m.group(1)
    return None


def _stamp_to_ns(stamp: TimeMsg) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _stamp_to_str(stamp: TimeMsg) -> str:
    return f"{int(stamp.sec)}.{int(stamp.nanosec):09d}"


class CaptureService(Node):
    def __init__(self):
        super().__init__("capture_service")

        self.declare_parameter("cam0_index", 0)
        self.declare_parameter("cam1_index", 1)

        self.declare_parameter("width", 4056)
        self.declare_parameter("height", 3040)

        self.declare_parameter("warmup_ms", 350)
        self.declare_parameter("timeout_ms", 6000)
        self.declare_parameter("default_quality", 95)
        self.declare_parameter("capture_mode", "stream")  # stream|still
        self.declare_parameter("stream_wait_s", 1.0)
        self.declare_parameter("stream_max_frame_age_s", 1.0)
        self.declare_parameter("write_capture_metadata", True)
        self.declare_parameter("sensor_buffer_s", 20.0)

        # Preview management
        self.declare_parameter("manage_previews", False)
        self.declare_parameter("start_previews", True)
        self.declare_parameter("pause_previews", True)
        self.declare_parameter("auto_detect_cameras", True)
        self.declare_parameter("fallback_black_previews", True)

        self.declare_parameter("preview_width", 960)
        self.declare_parameter("preview_height", 540)
        self.declare_parameter("preview_fps", 20)
        self.declare_parameter("preview_format", "RGB888")
        self.declare_parameter("preview_role", "viewfinder")
        self.declare_parameter("preview_start_stagger_s", 0.7)
        self.declare_parameter("preview_restart_attempts", 2)
        self.declare_parameter("preview_restart_delay_s", 0.6)

        self.declare_parameter("cam0_namespace", "/cam0")
        self.declare_parameter("cam1_namespace", "/cam1")
        self.declare_parameter("cam0_node_name", "camera")
        self.declare_parameter("cam1_node_name", "camera")
        self.declare_parameter("use_local_libcamera_env", False)
        self.declare_parameter("sanitize_preview_env", True)
        self.declare_parameter("gnss_fix_topic", "/fix")
        self.declare_parameter("gnss_time_ref_topic", "/time_reference")
        self.declare_parameter("gnss_imu_topic", "/imu/data")
        self.declare_parameter("gpio_trigger_enable", False)
        self.declare_parameter("gpio_trigger_chip", "/dev/gpiochip0")
        self.declare_parameter("gpio_trigger_line", 24)
        self.declare_parameter("gpio_trigger_active_low", True)
        self.declare_parameter("gpio_trigger_cooldown_ms", 1000)
        self.declare_parameter("gpio_trigger_session_prefix", "btn")
        self.declare_parameter("gpio_trigger_output_dir", "")
        self.declare_parameter("gpio_trigger_quality", 0)
        self.declare_parameter("gpio_trigger_poll_ms", 20)

        # Reliability / timing knobs
        self.declare_parameter("preview_shutdown_timeout_s", 2.5)
        self.declare_parameter("device_release_timeout_s", 2.5)
        self.declare_parameter("device_release_poll_s", 0.05)

        self.declare_parameter("retries", 2)
        self.declare_parameter("retry_wait_s", 0.4)

        # Faster overall: run both still captures in parallel
        self.declare_parameter("capture_parallel", False)

        self._p0: Optional[subprocess.Popen] = None
        self._p1: Optional[subprocess.Popen] = None
        self._p0_params = "/tmp/subsea_cam0_preview_params.yaml"
        self._p1_params = "/tmp/subsea_cam1_preview_params.yaml"
        self._capture_lock = threading.Lock()
        self._preview_reconfig_lock = threading.Lock()
        self._preview_reconfig_pending = False
        self._preview_reconfig_reason = ""
        self._preview_reconfig_thread: Optional[threading.Thread] = None

        self._devs = _dev_paths()
        self._camera_node_exe: Optional[str] = None
        self._fallback_pub0 = None
        self._fallback_pub1 = None
        self._fallback_timer = None
        self._fallback_w = 0
        self._fallback_h = 0
        self._fallback_data: Optional[bytes] = None
        self._detected_cam_count: Optional[int] = None
        self._expected_preview_cams: Optional[int] = None
        self._preview_restart_count: int = 0
        self._bridge = CvBridge()
        self._stream_lock = threading.Lock()
        self._latest_cam0_msg: Optional[Image] = None
        self._latest_cam1_msg: Optional[Image] = None
        self._latest_cam0_rx_mono: Optional[float] = None
        self._latest_cam1_rx_mono: Optional[float] = None

        self._sensor_lock = threading.Lock()
        self._buf_fix: Deque[NavSatFix] = deque()
        self._buf_time_ref: Deque[TimeReference] = deque()
        self._buf_imu: Deque[Imu] = deque()
        self._gpio_mod = None
        self._gpio_chip = None
        self._gpio_line_obj = None
        self._gpio_req = None
        self._gpio_timer = None
        self._gpio_prev_pressed = False
        self._gpio_last_trigger_mono = 0.0
        self._gpio_capture_thread = None

        img_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=2,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        sens_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        ns0 = str(self.get_parameter("cam0_namespace").value)
        ns1 = str(self.get_parameter("cam1_namespace").value)
        n0 = str(self.get_parameter("cam0_node_name").value)
        n1 = str(self.get_parameter("cam1_node_name").value)
        cam0_topic = self._preview_topic(ns0, n0)
        cam1_topic = self._preview_topic(ns1, n1)
        self._cam0_sub = self.create_subscription(Image, cam0_topic, self._on_cam0_image, img_qos)
        self._cam1_sub = self.create_subscription(Image, cam1_topic, self._on_cam1_image, img_qos)
        self.get_logger().info(f"Stream capture subscribers: cam0={cam0_topic} cam1={cam1_topic}")

        fix_topic = str(self.get_parameter("gnss_fix_topic").value)
        time_ref_topic = str(self.get_parameter("gnss_time_ref_topic").value)
        imu_topic = str(self.get_parameter("gnss_imu_topic").value)
        self._fix_sub = self.create_subscription(NavSatFix, fix_topic, self._on_fix, sens_qos)
        self._time_ref_sub = self.create_subscription(TimeReference, time_ref_topic, self._on_time_ref, sens_qos)
        self._imu_sub = self.create_subscription(Imu, imu_topic, self._on_imu, sens_qos)
        self.get_logger().info(
            f"Telemetry subscribers: fix={fix_topic} time_ref={time_ref_topic} imu={imu_topic}"
        )

        self.srv = self.create_service(CapturePair, "capture_pair", self.on_capture)
        self.action = ActionServer(
            self,
            CapturePairAction,
            "capture_pair",
            execute_callback=self.on_capture_action,
            goal_callback=self.on_capture_goal,
            cancel_callback=self.on_capture_cancel,
        )
        self.get_logger().info("Capture service ready: /capture_pair")
        self.get_logger().info("Capture action ready: /capture_pair")
        self.get_logger().info(f"Capture mode: {self._capture_mode()}")
        self._setup_gpio_trigger()
        self.add_on_set_parameters_callback(self._on_set_parameters)

        if bool(self.get_parameter("manage_previews").value) and bool(self.get_parameter("start_previews").value):
            auto_detect = bool(self.get_parameter("auto_detect_cameras").value)
            fallback_black = bool(self.get_parameter("fallback_black_previews").value)
            if auto_detect:
                count = _libcamera_camera_count()
                self._detected_cam_count = count
                if count is None:
                    if fallback_black:
                        self.get_logger().warn(
                            "Camera auto-detect unavailable. Publishing black previews. "
                            "Install libcamera-apps or set auto_detect_cameras:=false to force camera previews."
                        )
                        self._start_black_previews()
                    else:
                        self.get_logger().warn(
                            "Camera auto-detect unavailable; starting previews anyway."
                        )
                        self.get_logger().info("manage_previews:=true -> starting preview camera nodes")
                        self._start_previews()
                elif count <= 0:
                    self._handle_no_cameras(fallback_black)
                else:
                    self.get_logger().info("manage_previews:=true -> starting preview camera nodes")
                    self._start_previews(camera_count=count)
            else:
                self.get_logger().info("manage_previews:=true -> starting preview camera nodes")
                self._start_previews()

    def _handle_no_cameras(self, fallback_black: bool) -> None:
        if fallback_black:
            self.get_logger().warn("No cameras detected. Publishing black preview frames.")
            self._start_black_previews(camera_count=0)
        else:
            self.get_logger().warn("No cameras detected. Previews disabled.")

    def _on_set_parameters(self, params) -> SetParametersResult:
        restart_reasons: List[str] = []
        for p in params:
            if p.name == "preview_fps":
                try:
                    v = int(p.value)
                except Exception:
                    return SetParametersResult(successful=False, reason="preview_fps must be an integer")
                if v < 1 or v > 120:
                    return SetParametersResult(successful=False, reason="preview_fps must be in [1,120]")
                restart_reasons.append(f"preview_fps={v}")
            elif p.name == "preview_width":
                try:
                    v = int(p.value)
                except Exception:
                    return SetParametersResult(successful=False, reason="preview_width must be an integer")
                if v < 64:
                    return SetParametersResult(successful=False, reason="preview_width must be >= 64")
                restart_reasons.append(f"preview_width={v}")
            elif p.name == "preview_height":
                try:
                    v = int(p.value)
                except Exception:
                    return SetParametersResult(successful=False, reason="preview_height must be an integer")
                if v < 64:
                    return SetParametersResult(successful=False, reason="preview_height must be >= 64")
                restart_reasons.append(f"preview_height={v}")
            elif p.name == "preview_format":
                restart_reasons.append(f"preview_format={str(p.value)}")
            elif p.name == "preview_role":
                restart_reasons.append(f"preview_role={str(p.value)}")

        if restart_reasons:
            self._request_preview_reconfigure(", ".join(restart_reasons))
        return SetParametersResult(successful=True, reason="")

    def _request_preview_reconfigure(self, reason: str) -> None:
        # Only meaningful when this node owns preview processes.
        if not bool(self.get_parameter("manage_previews").value):
            return
        if not bool(self.get_parameter("start_previews").value):
            return

        with self._preview_reconfig_lock:
            self._preview_reconfig_pending = True
            self._preview_reconfig_reason = reason
            running = (
                self._preview_reconfig_thread is not None
                and self._preview_reconfig_thread.is_alive()
            )
            if running:
                return
            self._preview_reconfig_thread = threading.Thread(
                target=self._preview_reconfigure_worker,
                daemon=True,
            )
            self._preview_reconfig_thread.start()

    def _preview_reconfigure_worker(self) -> None:
        # Let parameter update finish before reading new values via get_parameter().
        time.sleep(0.05)
        while True:
            with self._preview_reconfig_lock:
                if not self._preview_reconfig_pending:
                    return
                self._preview_reconfig_pending = False
                reason = self._preview_reconfig_reason

            try:
                with self._capture_lock:
                    if not bool(self.get_parameter("manage_previews").value):
                        return
                    if not bool(self.get_parameter("start_previews").value):
                        return
                    self.get_logger().info(
                        f"Applying preview parameter update ({reason}): restarting previews"
                    )
                    self._start_previews(camera_count=self._detected_cam_count)
            except Exception as e:
                self.get_logger().error(f"Failed to apply preview parameter update: {e}")

            # Coalesce bursts of updates into at most one extra restart.
            time.sleep(0.05)

    def _preview_topic(self, ns: str, node_name: str) -> str:
        ns = (ns or "").strip()
        node_name = (node_name or "").strip().strip("/")
        if ns and not ns.startswith("/"):
            ns = "/" + ns
        ns = ns.rstrip("/")
        if ns and node_name:
            return f"{ns}/{node_name}/image_raw"
        if ns:
            return f"{ns}/image_raw"
        if node_name:
            return f"/{node_name}/image_raw"
        return "/image_raw"

    def _setup_gpio_trigger(self) -> None:
        if not bool(self.get_parameter("gpio_trigger_enable").value):
            return

        try:
            import gpiod  # type: ignore
        except Exception as e:
            self.get_logger().error(f"GPIO trigger disabled: python gpiod import failed: {e}")
            return

        chip_name = str(self.get_parameter("gpio_trigger_chip").value)
        line_offset = int(self.get_parameter("gpio_trigger_line").value)
        active_low = bool(self.get_parameter("gpio_trigger_active_low").value)
        poll_ms = max(5, int(self.get_parameter("gpio_trigger_poll_ms").value))

        chip = None
        try:
            chip = gpiod.Chip(chip_name)
        except Exception:
            # Fallback for APIs expecting logical chip name instead of full path.
            try:
                chip = gpiod.Chip(os.path.basename(chip_name))
            except Exception as e:
                emsg = f"GPIO trigger disabled: failed to open chip '{chip_name}': {e}"
                if "Permission denied" in str(e):
                    emsg += (
                        " (fix: sudo usermod -aG gpio $USER && newgrp gpio)"
                    )
                self.get_logger().error(emsg)
                return

        try:
            # gpiod v2 path
            if hasattr(gpiod, "LineSettings") and hasattr(chip, "request_lines"):
                ls_kwargs = {}
                if hasattr(gpiod, "line") and hasattr(gpiod.line, "Direction"):
                    ls_kwargs["direction"] = gpiod.line.Direction.INPUT
                if hasattr(gpiod, "line") and hasattr(gpiod.line, "Bias"):
                    ls_kwargs["bias"] = (
                        gpiod.line.Bias.PULL_UP if active_low else gpiod.line.Bias.PULL_DOWN
                    )
                settings = gpiod.LineSettings(**ls_kwargs)
                self._gpio_req = chip.request_lines(
                    consumer="subsea_capture",
                    config={line_offset: settings},
                )
            else:
                # gpiod v1 path
                line = chip.get_line(line_offset)
                req_type = getattr(gpiod, "LINE_REQ_DIR_IN")
                flags = 0
                if active_low and hasattr(gpiod, "LINE_REQ_FLAG_BIAS_PULL_UP"):
                    flags |= getattr(gpiod, "LINE_REQ_FLAG_BIAS_PULL_UP")
                if (not active_low) and hasattr(gpiod, "LINE_REQ_FLAG_BIAS_PULL_DOWN"):
                    flags |= getattr(gpiod, "LINE_REQ_FLAG_BIAS_PULL_DOWN")
                line.request(consumer="subsea_capture", type=req_type, flags=flags)
                self._gpio_line_obj = line
        except Exception as e:
            emsg = f"GPIO trigger disabled: failed to request line {line_offset} on {chip_name}: {e}"
            if "Permission denied" in str(e):
                emsg += " (fix: sudo usermod -aG gpio $USER && newgrp gpio)"
            self.get_logger().error(emsg)
            try:
                chip.close()
            except Exception:
                pass
            return

        self._gpio_mod = gpiod
        self._gpio_chip = chip
        self._gpio_prev_pressed = False
        self._gpio_last_trigger_mono = 0.0
        self._gpio_timer = self.create_timer(poll_ms / 1000.0, self._gpio_poll_cb)
        self.get_logger().info(
            f"GPIO trigger enabled: chip={chip_name} line={line_offset} "
            f"active_low={active_low} poll_ms={poll_ms}"
        )

    def _read_gpio_pressed(self) -> Optional[bool]:
        raw = None
        line_offset = int(self.get_parameter("gpio_trigger_line").value)
        active_low = bool(self.get_parameter("gpio_trigger_active_low").value)

        try:
            if self._gpio_req is not None:
                raw = self._gpio_req.get_value(line_offset)
            elif self._gpio_line_obj is not None:
                raw = self._gpio_line_obj.get_value()
            else:
                return None
        except Exception as e:
            self.get_logger().error(f"GPIO read failed: {e}")
            return None

        try:
            if hasattr(raw, "value"):
                raw_i = int(raw.value)
            else:
                raw_i = int(raw)
        except Exception:
            raw_s = str(raw).upper()
            raw_i = 1 if ("ACTIVE" in raw_s or raw_s.endswith("1")) else 0

        return (raw_i == 0) if active_low else (raw_i != 0)

    def _gpio_poll_cb(self) -> None:
        pressed = self._read_gpio_pressed()
        if pressed is None:
            return

        # Trigger on press edge only.
        if pressed and not self._gpio_prev_pressed:
            now_m = time.monotonic()
            cooldown_s = max(0.05, float(self.get_parameter("gpio_trigger_cooldown_ms").value) / 1000.0)
            if (now_m - self._gpio_last_trigger_mono) >= cooldown_s:
                self._gpio_last_trigger_mono = now_m
                self._on_gpio_trigger()
        self._gpio_prev_pressed = pressed

    def _on_gpio_trigger(self) -> None:
        if self._gpio_capture_thread is not None and self._gpio_capture_thread.is_alive():
            self.get_logger().warn("GPIO trigger ignored: capture already in progress")
            return

        self._gpio_capture_thread = threading.Thread(
            target=self._gpio_capture_worker,
            daemon=True,
        )
        self._gpio_capture_thread.start()

    def _gpio_capture_worker(self) -> None:
        prefix = str(self.get_parameter("gpio_trigger_session_prefix").value).strip() or "btn"
        session = f"{prefix}_{datetime.now().strftime('%Y%m%d_%H%M%S_%f')[:-3]}"
        out_dir = str(self.get_parameter("gpio_trigger_output_dir").value).strip()
        quality = int(self.get_parameter("gpio_trigger_quality").value)

        self.get_logger().info(f"GPIO button pressed -> capture session={session}")
        with self._capture_lock:
            success, message, cam0_path, cam1_path, stamp = self._perform_capture(
                session,
                out_dir,
                quality,
                feedback_cb=None,
            )
        if success:
            self.get_logger().info(
                f"GPIO capture OK stamp={_stamp_to_str(stamp)} cam0={cam0_path} cam1={cam1_path}"
            )
        else:
            self.get_logger().error(f"GPIO capture failed: {message}")

    def _cleanup_gpio_trigger(self) -> None:
        if self._gpio_timer is not None:
            try:
                self.destroy_timer(self._gpio_timer)
            except Exception:
                try:
                    self._gpio_timer.cancel()
                except Exception:
                    pass
        self._gpio_timer = None

        if self._gpio_req is not None:
            try:
                self._gpio_req.release()
            except Exception:
                pass
        self._gpio_req = None

        if self._gpio_line_obj is not None:
            try:
                self._gpio_line_obj.release()
            except Exception:
                pass
        self._gpio_line_obj = None

        if self._gpio_chip is not None:
            try:
                self._gpio_chip.close()
            except Exception:
                pass
        self._gpio_chip = None
        self._gpio_mod = None

    def _trim_sensor_buffers_locked(self) -> None:
        keep_s = float(self.get_parameter("sensor_buffer_s").value)
        keep_s = max(2.0, keep_s)
        cutoff_ns = _stamp_to_ns(now_ros_time(self)) - int(keep_s * 1_000_000_000)

        while self._buf_fix and _stamp_to_ns(self._buf_fix[0].header.stamp) < cutoff_ns:
            self._buf_fix.popleft()
        while self._buf_time_ref and _stamp_to_ns(self._buf_time_ref[0].header.stamp) < cutoff_ns:
            self._buf_time_ref.popleft()
        while self._buf_imu and _stamp_to_ns(self._buf_imu[0].header.stamp) < cutoff_ns:
            self._buf_imu.popleft()

    def _on_fix(self, msg: NavSatFix) -> None:
        with self._sensor_lock:
            self._buf_fix.append(msg)
            self._trim_sensor_buffers_locked()

    def _on_time_ref(self, msg: TimeReference) -> None:
        with self._sensor_lock:
            self._buf_time_ref.append(msg)
            self._trim_sensor_buffers_locked()

    def _on_imu(self, msg: Imu) -> None:
        with self._sensor_lock:
            self._buf_imu.append(msg)
            self._trim_sensor_buffers_locked()

    def _on_cam0_image(self, msg: Image) -> None:
        with self._stream_lock:
            self._latest_cam0_msg = msg
            self._latest_cam0_rx_mono = time.monotonic()

    def _on_cam1_image(self, msg: Image) -> None:
        with self._stream_lock:
            self._latest_cam1_msg = msg
            self._latest_cam1_rx_mono = time.monotonic()

    def _latest_stream_snapshot(self) -> Tuple[Optional[Image], Optional[Image], Optional[float], Optional[float]]:
        with self._stream_lock:
            return (
                self._latest_cam0_msg,
                self._latest_cam1_msg,
                self._latest_cam0_rx_mono,
                self._latest_cam1_rx_mono,
            )

    def _capture_mode(self) -> str:
        mode = str(self.get_parameter("capture_mode").value).strip().lower()
        if mode not in ("stream", "still"):
            self.get_logger().warn(f"Unknown capture_mode='{mode}', falling back to 'still'")
            return "still"
        return mode

    def _imgmsg_to_bgr(self, msg: Image):
        enc = (msg.encoding or "").lower().strip()
        w = int(msg.width)
        h = int(msg.height)
        step = int(msg.step)

        if w <= 0 or h <= 0:
            raise ValueError(f"invalid image dimensions: {w}x{h}")

        if step <= 0:
            if enc in ("bgr8", "rgb8"):
                step = w * 3
            elif enc in ("bgra8", "rgba8"):
                step = w * 4
            elif enc == "mono8":
                step = w

        if step > 0 and len(msg.data) < (step * h):
            raise ValueError(
                f"truncated image payload: encoding={enc} size={w}x{h} step={step} bytes={len(msg.data)}"
            )

        try:
            mv = memoryview(msg.data)
            if enc in ("bgr8", "rgb8") and step >= (w * 3):
                frame = np.ndarray(
                    (h, w, 3),
                    dtype=np.uint8,
                    buffer=mv,
                    strides=(step, 3, 1),
                )
                if enc == "rgb8":
                    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                else:
                    frame = frame.copy()
                return frame
            if enc in ("bgra8", "rgba8") and step >= (w * 4):
                frame4 = np.ndarray(
                    (h, w, 4),
                    dtype=np.uint8,
                    buffer=mv,
                    strides=(step, 4, 1),
                )
                conv = cv2.COLOR_BGRA2BGR if enc == "bgra8" else cv2.COLOR_RGBA2BGR
                return cv2.cvtColor(frame4, conv)
            if enc == "mono8" and step >= w:
                gray = np.ndarray(
                    (h, w),
                    dtype=np.uint8,
                    buffer=mv,
                    strides=(step, 1),
                )
                return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        except Exception:
            pass

        # Generic fallback for less common encodings.
        return self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def _write_jpeg_bgr(self, path: str, frame, quality: int) -> bool:
        ok, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)])
        if not ok:
            return False
        with open(path, "wb") as f:
            f.write(buf.tobytes())
        return True

    def _nearest_fix(self, stamp_ns: int) -> Optional[NavSatFix]:
        with self._sensor_lock:
            msgs = list(self._buf_fix)
        if not msgs:
            return None
        return min(msgs, key=lambda m: abs(_stamp_to_ns(m.header.stamp) - stamp_ns))

    def _nearest_time_ref(self, stamp_ns: int) -> Optional[TimeReference]:
        with self._sensor_lock:
            msgs = list(self._buf_time_ref)
        if not msgs:
            return None
        return min(msgs, key=lambda m: abs(_stamp_to_ns(m.header.stamp) - stamp_ns))

    def _nearest_imu(self, stamp_ns: int) -> Optional[Imu]:
        with self._sensor_lock:
            msgs = list(self._buf_imu)
        if not msgs:
            return None
        return min(msgs, key=lambda m: abs(_stamp_to_ns(m.header.stamp) - stamp_ns))

    def _camera_count_for_capture(self) -> Optional[int]:
        cam_count = _libcamera_camera_count()
        self._detected_cam_count = cam_count
        if cam_count is not None:
            return cam_count

        # Auto-detect can fail on some systems; infer one-camera setups from stream.
        c0, c1, _, _ = self._latest_stream_snapshot()
        if c0 is not None and c1 is None:
            return 1
        if c0 is not None and c1 is not None:
            return 2
        return None

    def _build_capture_metadata(
        self,
        mode: str,
        session: str,
        trigger_stamp: TimeMsg,
        cam0_path: str,
        cam1_path: str,
        cam0_stamp: Optional[TimeMsg],
        cam1_stamp: Optional[TimeMsg],
    ) -> Dict[str, Any]:
        trigger_ns = _stamp_to_ns(trigger_stamp)
        metadata: Dict[str, Any] = {
            "schema_version": 1,
            "mode": mode,
            "session_id": session,
            "trigger_stamp": _stamp_to_str(trigger_stamp),
            "cameras": {},
        }

        def add_camera(name: str, path: str, stamp: Optional[TimeMsg]) -> None:
            if not path:
                return
            info: Dict[str, Any] = {
                "path": path,
                "stamp": _stamp_to_str(stamp) if stamp is not None else None,
            }
            if stamp is not None:
                s_ns = _stamp_to_ns(stamp)
                info["offset_from_trigger_ms"] = (s_ns - trigger_ns) / 1_000_000.0

                fix = self._nearest_fix(s_ns)
                if fix is not None:
                    fix_ns = _stamp_to_ns(fix.header.stamp)
                    info["nearest_fix"] = {
                        "stamp": _stamp_to_str(fix.header.stamp),
                        "dt_ms": (fix_ns - s_ns) / 1_000_000.0,
                        "lat": float(fix.latitude),
                        "lon": float(fix.longitude),
                        "alt": float(fix.altitude),
                        "status": int(fix.status.status),
                        "service": int(fix.status.service),
                    }

                tr = self._nearest_time_ref(s_ns)
                if tr is not None:
                    tr_ns = _stamp_to_ns(tr.header.stamp)
                    info["nearest_time_ref"] = {
                        "stamp": _stamp_to_str(tr.header.stamp),
                        "time_ref": _stamp_to_str(tr.time_ref),
                        "source": tr.source or "",
                        "dt_ms": (tr_ns - s_ns) / 1_000_000.0,
                    }

                imu = self._nearest_imu(s_ns)
                if imu is not None:
                    imu_ns = _stamp_to_ns(imu.header.stamp)
                    info["nearest_imu"] = {
                        "stamp": _stamp_to_str(imu.header.stamp),
                        "dt_ms": (imu_ns - s_ns) / 1_000_000.0,
                        "angular_velocity": [
                            float(imu.angular_velocity.x),
                            float(imu.angular_velocity.y),
                            float(imu.angular_velocity.z),
                        ],
                        "linear_acceleration": [
                            float(imu.linear_acceleration.x),
                            float(imu.linear_acceleration.y),
                            float(imu.linear_acceleration.z),
                        ],
                    }
            metadata["cameras"][name] = info

        add_camera("cam0", cam0_path, cam0_stamp)
        add_camera("cam1", cam1_path, cam1_stamp)
        return metadata

    def _write_capture_metadata(
        self,
        out_dir: str,
        session: str,
        metadata: Dict[str, Any],
    ) -> str:
        meta_path = os.path.join(out_dir, f"{session}_meta.json")
        tmp = meta_path + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(metadata, f, indent=2, sort_keys=True)
        os.replace(tmp, meta_path)
        return meta_path

    def _start_black_previews(self, camera_count: Optional[int] = None) -> None:
        if self._fallback_timer is not None:
            return
        ns0 = str(self.get_parameter("cam0_namespace").value)
        ns1 = str(self.get_parameter("cam1_namespace").value)
        n0 = str(self.get_parameter("cam0_node_name").value)
        n1 = str(self.get_parameter("cam1_node_name").value)
        w = int(self.get_parameter("preview_width").value)
        h = int(self.get_parameter("preview_height").value)
        fps = max(1, int(self.get_parameter("preview_fps").value))

        # If detection knows camera count, only publish those slots. This avoids
        # showing disconnected cameras as "active".
        if camera_count is None:
            camera_count = self._detected_cam_count
        if camera_count is None:
            publish_slots = 2
        else:
            publish_slots = max(0, min(2, int(camera_count)))

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        if publish_slots >= 1:
            self._fallback_pub0 = self.create_publisher(Image, self._preview_topic(ns0, n0), qos)
        else:
            self._fallback_pub0 = None

        if publish_slots >= 2:
            self._fallback_pub1 = self.create_publisher(Image, self._preview_topic(ns1, n1), qos)
        else:
            self._fallback_pub1 = None

        if self._fallback_pub0 is None and self._fallback_pub1 is None:
            self.get_logger().warn(
                "Black fallback requested but no camera slots are active (camera_count=0)"
            )
            return

        self._fallback_w = w
        self._fallback_h = h
        self._fallback_data = bytes(w * h * 3)

        period = 1.0 / float(fps)
        self._fallback_timer = self.create_timer(period, self._publish_black_previews)
        topics = []
        if self._fallback_pub0 is not None:
            topics.append(self._preview_topic(ns0, n0))
        if self._fallback_pub1 is not None:
            topics.append(self._preview_topic(ns1, n1))
        self.get_logger().info(f"Black preview publishers running: {' | '.join(topics)}")

    def _stop_black_previews(self) -> None:
        if self._fallback_timer is not None:
            try:
                self.destroy_timer(self._fallback_timer)
            except Exception:
                try:
                    self._fallback_timer.cancel()
                except Exception:
                    pass
        self._fallback_timer = None
        if self._fallback_pub0 is not None:
            try:
                self.destroy_publisher(self._fallback_pub0)
            except Exception:
                pass
        if self._fallback_pub1 is not None:
            try:
                self.destroy_publisher(self._fallback_pub1)
            except Exception:
                pass
        self._fallback_pub0 = None
        self._fallback_pub1 = None
        self._fallback_data = None

    def _publish_black_previews(self) -> None:
        if self._fallback_data is None:
            return
        stamp = now_ros_time(self)
        if self._fallback_pub0 is not None:
            msg0 = Image()
            msg0.header.stamp = stamp
            msg0.header.frame_id = "cam0_optical_frame"
            msg0.height = self._fallback_h
            msg0.width = self._fallback_w
            msg0.encoding = "bgr8"
            msg0.is_bigendian = False
            msg0.step = self._fallback_w * 3
            msg0.data = self._fallback_data
            self._fallback_pub0.publish(msg0)

        if self._fallback_pub1 is not None:
            msg1 = Image()
            msg1.header.stamp = stamp
            msg1.header.frame_id = "cam1_optical_frame"
            msg1.height = self._fallback_h
            msg1.width = self._fallback_w
            msg1.encoding = "bgr8"
            msg1.is_bigendian = False
            msg1.step = self._fallback_w * 3
            msg1.data = self._fallback_data
            self._fallback_pub1.publish(msg1)

    def _preview_params(self, cam_index: int, frame_id: str) -> dict:
        w = int(self.get_parameter("preview_width").value)
        h = int(self.get_parameter("preview_height").value)
        fps = int(self.get_parameter("preview_fps").value)
        fmt = str(self.get_parameter("preview_format").value).strip()
        role = str(self.get_parameter("preview_role").value)
        frame_us = int(1_000_000 / max(1, fps))

        params = {
            "camera": int(cam_index),
            "role": role,
            "width": w,
            "height": h,
            "FrameDurationLimits": [frame_us, frame_us],
            "use_node_time": False,
            "frame_id": frame_id,
        }
        # Avoid forcing pixel format unless explicitly requested.
        if fmt and fmt.lower() not in ("auto", "default", "native"):
            params["format"] = fmt
        return params

    def _preview_env(self) -> dict:
        env = os.environ.copy()
        use_local = bool(self.get_parameter("use_local_libcamera_env").value)
        sanitize = bool(self.get_parameter("sanitize_preview_env").value)

        if not use_local:
            # Default to system libcamera.
            if sanitize:
                # Drop stale custom libcamera overrides while keeping ROS paths.
                env.pop("LIBCAMERA_IPA_MODULE_PATH", None)
                old_ld = env.get("LD_LIBRARY_PATH", "")
                if old_ld:
                    cleaned = _sanitize_preview_ld_library_path(old_ld)
                    if cleaned:
                        env["LD_LIBRARY_PATH"] = cleaned
                    else:
                        env.pop("LD_LIBRARY_PATH", None)
            return env

        local_libs = [
            "/usr/local/lib/aarch64-linux-gnu",
            "/usr/local/lib",
            "/usr/local/lib64",
        ]
        existing = env.get("LD_LIBRARY_PATH", "")
        parts = [p for p in existing.split(":") if p]
        for p in local_libs:
            if os.path.isdir(p) and p not in parts:
                parts.insert(0, p)
        if parts:
            env["LD_LIBRARY_PATH"] = ":".join(parts)

        ipa_path = None
        for p in local_libs:
            candidate = os.path.join(p, "libcamera", "ipa")
            if os.path.isdir(candidate):
                ipa_path = candidate
                break
        if ipa_path:
            env.setdefault("LIBCAMERA_IPA_MODULE_PATH", ipa_path)
        return env

    def _start_preview_proc(self, cam_index: int, ns: str, node_name: str, params_path: str, frame_id: str) -> subprocess.Popen:
        if self._camera_node_exe is None:
            self._camera_node_exe = _camera_ros_exe_path()
            self.get_logger().info(f"camera_ros executable: {self._camera_node_exe}")
            linked_libcamera = _linked_libcamera_path(self._camera_node_exe)
            if linked_libcamera:
                self.get_logger().info(f"camera_ros linked libcamera: {linked_libcamera}")
                if "/opt/ros/" in linked_libcamera:
                    self.get_logger().warn(
                        "camera_ros links against /opt/ros libcamera. On Raspberry Pi this may report "
                        "'no cameras available'. Prefer system/RPi libcamera and rebuild camera_ros."
                    )
            if self._camera_node_exe.startswith("/opt/ros/"):
                self.get_logger().warn(
                    "Using camera_ros from /opt/ros. If previews fail with 'no cameras available', "
                    "build camera_ros in this workspace and re-source install/setup.bash."
                )
        _write_params_file(params_path, self._preview_params(cam_index, frame_id))

        cmd = [
            self._camera_node_exe,
            "--ros-args",
            "-r", f"__node:={node_name}",
            "-r", f"__ns:={ns}",
            "--params-file", params_path,
        ]
        env = self._preview_env()
        return _popen_group(cmd, env=env)

    def _start_previews(self, camera_count: Optional[int] = None) -> None:
        if self._fallback_timer is not None:
            self.get_logger().info("Stopping black preview fallback (camera previews starting)")
            self._stop_black_previews()
        cam0 = int(self.get_parameter("cam0_index").value)
        cam1 = int(self.get_parameter("cam1_index").value)
        ns0 = str(self.get_parameter("cam0_namespace").value)
        ns1 = str(self.get_parameter("cam1_namespace").value)
        n0 = str(self.get_parameter("cam0_node_name").value)
        n1 = str(self.get_parameter("cam1_node_name").value)

        if camera_count is None:
            camera_count = _libcamera_camera_count()
        self._detected_cam_count = camera_count

        if camera_count is not None and camera_count <= 0:
            fallback_black = bool(self.get_parameter("fallback_black_previews").value)
            self._handle_no_cameras(fallback_black)
            return

        expected = None if camera_count is None else max(0, min(2, camera_count))
        self._expected_preview_cams = expected
        self._preview_restart_count = 0

        if expected is None or expected >= 1:
            if self._p0 is None or self._p0.poll() is not None:
                self._p0 = self._start_preview_proc(cam0, ns0, n0, self._p0_params, "cam0_optical_frame")
                self.get_logger().info(f"cam0 preview started (pid={self._p0.pid})")
        else:
            if self._p0 is not None:
                _stop_proc(self._p0, timeout_s=float(self.get_parameter("preview_shutdown_timeout_s").value))
                self._p0 = None

        if expected is None or expected >= 2:
            if self._p1 is None or self._p1.poll() is not None:
                stagger_s = max(0.0, float(self.get_parameter("preview_start_stagger_s").value))
                if stagger_s > 0.0 and self._p0 is not None and self._p0.poll() is None:
                    time.sleep(stagger_s)
                self._p1 = self._start_preview_proc(cam1, ns1, n1, self._p1_params, "cam1_optical_frame")
                self.get_logger().info(f"cam1 preview started (pid={self._p1.pid})")
        else:
            if self._p1 is not None:
                _stop_proc(self._p1, timeout_s=float(self.get_parameter("preview_shutdown_timeout_s").value))
                self._p1 = None

        self.get_logger().info("Preview camera nodes start sequence complete")
        self._verify_previews_started()

    def _verify_previews_started(self) -> None:
        # Give camera_ros a moment to initialize; if it exits immediately,
        # fall back to black previews to keep the UI stable.
        time.sleep(0.6)
        p0_dead = (self._p0 is None) or (self._p0.poll() is not None)
        p1_dead = (self._p1 is None) or (self._p1.poll() is not None)

        if self._expected_preview_cams is None:
            # Unknown camera count: only fall back if both previews failed.
            if p0_dead and p1_dead:
                self.get_logger().warn(
                    "Preview camera nodes exited early. Falling back to black previews."
                )
                if bool(self.get_parameter("fallback_black_previews").value):
                    self._stop_previews_managed()
                    self._start_black_previews(camera_count=0)
                else:
                    self.get_logger().warn(
                        "fallback_black_previews:=false, leaving preview topics inactive."
                    )
            elif p0_dead or p1_dead:
                self.get_logger().warn(
                    "One preview camera node exited early. Continuing with remaining stream."
                )
            return

        dead = []
        if self._expected_preview_cams >= 1 and p0_dead:
            dead.append("cam0")
        if self._expected_preview_cams >= 2 and p1_dead:
            dead.append("cam1")
        if dead:
            max_restarts = max(0, int(self.get_parameter("preview_restart_attempts").value))
            if self._preview_restart_count < max_restarts:
                self._preview_restart_count += 1
                self.get_logger().warn(
                    f"Preview camera nodes exited early ({', '.join(dead)}). "
                    f"Retrying failed previews ({self._preview_restart_count}/{max_restarts})."
                )
                self._restart_failed_previews(dead)
                self._verify_previews_started()
                return

            self.get_logger().warn(
                f"Preview camera nodes exited early ({', '.join(dead)}). "
                "Falling back to black previews."
            )
            if bool(self.get_parameter("fallback_black_previews").value):
                self._stop_previews_managed()
                self._start_black_previews(camera_count=self._expected_preview_cams)
            else:
                self.get_logger().warn(
                    "fallback_black_previews:=false, leaving failed cameras inactive."
                )

    def _restart_failed_previews(self, dead: List[str]) -> None:
        cam0 = int(self.get_parameter("cam0_index").value)
        cam1 = int(self.get_parameter("cam1_index").value)
        ns0 = str(self.get_parameter("cam0_namespace").value)
        ns1 = str(self.get_parameter("cam1_namespace").value)
        n0 = str(self.get_parameter("cam0_node_name").value)
        n1 = str(self.get_parameter("cam1_node_name").value)
        timeout_s = float(self.get_parameter("preview_shutdown_timeout_s").value)
        delay_s = max(0.0, float(self.get_parameter("preview_restart_delay_s").value))
        stagger_s = max(0.0, float(self.get_parameter("preview_start_stagger_s").value))

        if delay_s > 0.0:
            time.sleep(delay_s)

        if "cam0" in dead:
            if self._p0 is not None:
                _stop_proc(self._p0, timeout_s=timeout_s)
            self._p0 = self._start_preview_proc(cam0, ns0, n0, self._p0_params, "cam0_optical_frame")
            self.get_logger().info(f"cam0 preview restarted (pid={self._p0.pid})")

        if "cam1" in dead:
            if stagger_s > 0.0 and self._p0 is not None and self._p0.poll() is None:
                time.sleep(stagger_s)
            if self._p1 is not None:
                _stop_proc(self._p1, timeout_s=timeout_s)
            self._p1 = self._start_preview_proc(cam1, ns1, n1, self._p1_params, "cam1_optical_frame")
            self.get_logger().info(f"cam1 preview restarted (pid={self._p1.pid})")

    def _stop_previews_managed(self) -> None:
        timeout_s = float(self.get_parameter("preview_shutdown_timeout_s").value)
        if self._p0 is not None:
            _stop_proc(self._p0, timeout_s=timeout_s)
        if self._p1 is not None:
            _stop_proc(self._p1, timeout_s=timeout_s)
        self._p0 = None
        self._p1 = None

        # Wait until devices are actually free (prevents rpicam-still "pipeline busy" retries)
        release_timeout = float(self.get_parameter("device_release_timeout_s").value)
        poll = float(self.get_parameter("device_release_poll_s").value)
        t0 = time.monotonic()
        while (time.monotonic() - t0) < release_timeout:
            if not _devices_in_use(self._devs):
                return
            time.sleep(poll)

    def _rpicam_cmd(self, cam_index: int, out_path: str, quality: int) -> List[str]:
        width = int(self.get_parameter("width").value)
        height = int(self.get_parameter("height").value)
        warmup_ms = int(self.get_parameter("warmup_ms").value)
        timeout_ms = int(self.get_parameter("timeout_ms").value)
        t_ms = max(timeout_ms, warmup_ms + 200)

        return [
            "rpicam-still",
            "--nopreview",
            "--immediate",
            "--camera", str(cam_index),
            "--width", str(width),
            "--height", str(height),
            "--quality", str(quality),
            "-t", str(t_ms),
            "-o", out_path,
        ]

    def _run_one(self, cam_index: int, out_path: str, quality: int, timeout_s: float) -> Tuple[bool, str]:
        cmd = self._rpicam_cmd(cam_index, out_path, quality)
        try:
            p = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout_s, env=self._preview_env())
        except subprocess.TimeoutExpired:
            return False, f"TimeoutExpired running: {' '.join(cmd)}"

        ok = (p.returncode == 0) and os.path.exists(out_path) and os.path.getsize(out_path) > 0
        if ok:
            return True, ""
        return False, f"rc={p.returncode}\nstdout:\n{p.stdout}\nstderr:\n{p.stderr}\n"

    def _perform_capture_still(
        self,
        session_in: str,
        out_dir_in: str,
        quality_in: int,
        feedback_cb: Optional[Callable[[str], None]] = None,
    ) -> Tuple[bool, str, str, str, TimeMsg]:
        cam0 = int(self.get_parameter("cam0_index").value)
        cam1 = int(self.get_parameter("cam1_index").value)
        default_quality = int(self.get_parameter("default_quality").value)
        quality = int(quality_in) if quality_in > 0 else default_quality

        out_dir = out_dir_in.strip() if out_dir_in else ""
        out_dir = out_dir or os.path.expanduser("~/captures")
        os.makedirs(out_dir, exist_ok=True)

        session = session_in.strip() if session_in else ""
        session = session or datetime.now().strftime("%Y%m%d_%H%M%S")
        cam0_path = os.path.join(out_dir, f"{session}_cam0.jpg")
        cam1_path = os.path.join(out_dir, f"{session}_cam1.jpg")

        stamp = now_ros_time(self)
        self.get_logger().info(f"Capture session={session} -> {cam0_path}, {cam1_path}")

        pause_previews = bool(self.get_parameter("pause_previews").value)
        manage_previews = bool(self.get_parameter("manage_previews").value) and bool(self.get_parameter("start_previews").value)

        if pause_previews and manage_previews:
            if feedback_cb is not None:
                feedback_cb("pausing_previews")
            self.get_logger().info("Pausing previews...")
            self._stop_previews_managed()

        retries = int(self.get_parameter("retries").value)
        retry_wait = float(self.get_parameter("retry_wait_s").value)
        parallel = bool(self.get_parameter("capture_parallel").value)

        # timeout for each subprocess.run call
        # (slightly above the -t we pass to rpicam-still)
        warmup_ms = int(self.get_parameter("warmup_ms").value)
        timeout_ms = int(self.get_parameter("timeout_ms").value)
        t_ms = max(timeout_ms, warmup_ms + 200)
        run_timeout_s = max(10.0, (t_ms / 1000.0) + 6.0)

        ok0 = ok1 = False
        d0 = d1 = ""

        cam_count = _libcamera_camera_count()
        self._detected_cam_count = cam_count
        allow_cam1 = (cam_count is None) or (cam_count >= 2)

        try:
            for attempt in range(retries + 1):
                if feedback_cb is not None:
                    feedback_cb(f"capturing_attempt_{attempt+1}")
                if parallel:
                    # Run both still captures in parallel (faster pause window)
                    p0 = subprocess.Popen(self._rpicam_cmd(cam0, cam0_path, quality), stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, env=self._preview_env())
                    p1 = None
                    if allow_cam1:
                        p1 = subprocess.Popen(self._rpicam_cmd(cam1, cam1_path, quality), stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, env=self._preview_env())
                    try:
                        out0, err0 = p0.communicate(timeout=run_timeout_s)
                    except subprocess.TimeoutExpired:
                        p0.kill()
                        out0, err0 = "", "TimeoutExpired"
                    out1 = err1 = ""
                    if p1 is not None:
                        try:
                            out1, err1 = p1.communicate(timeout=run_timeout_s)
                        except subprocess.TimeoutExpired:
                            p1.kill()
                            out1, err1 = "", "TimeoutExpired"

                    ok0 = (p0.returncode == 0) and os.path.exists(cam0_path) and os.path.getsize(cam0_path) > 0
                    if allow_cam1 and p1 is not None:
                        ok1 = (p1.returncode == 0) and os.path.exists(cam1_path) and os.path.getsize(cam1_path) > 0
                    else:
                        ok1 = True
                    if not ok0:
                        d0 = f"attempt={attempt+1}/{retries+1} cam=0 rc={p0.returncode}\nstdout:\n{out0}\nstderr:\n{err0}\n"
                    if allow_cam1 and not ok1 and p1 is not None:
                        d1 = f"attempt={attempt+1}/{retries+1} cam=1 rc={p1.returncode}\nstdout:\n{out1}\nstderr:\n{err1}\n"
                else:
                    ok0, d0 = self._run_one(cam0, cam0_path, quality, run_timeout_s)
                    if allow_cam1:
                        ok1, d1 = self._run_one(cam1, cam1_path, quality, run_timeout_s)
                    else:
                        ok1 = True

                if ok0 and ok1:
                    break

                time.sleep(retry_wait)

        finally:
            if pause_previews and manage_previews:
                if feedback_cb is not None:
                    feedback_cb("resuming_previews")
                self.get_logger().info("Resuming previews...")
                try:
                    self._start_previews()
                except Exception as e:
                    self.get_logger().error(f"Failed to restart previews: {e}")

        if not ok0 or not ok1:
            fail_cam0 = cam0_path if os.path.exists(cam0_path) else ""
            fail_cam1 = cam1_path if (allow_cam1 and os.path.exists(cam1_path)) else ""
            fail_msg = (
                "CAPTURE FAILED\n"
                f"cam0_ok={ok0} path={fail_cam0}\n{d0}\n"
                f"cam1_ok={ok1} path={fail_cam1}\n{d1}\n"
            )
            self.get_logger().error(fail_msg)
            return False, fail_msg, fail_cam0, fail_cam1, stamp

        if not allow_cam1:
            cam1_path = ""
            msg = "OK (still mode, cam1 skipped: only one camera detected)"
        else:
            msg = "OK (still mode)"

        if bool(self.get_parameter("write_capture_metadata").value):
            metadata = self._build_capture_metadata(
                mode="still",
                session=session,
                trigger_stamp=stamp,
                cam0_path=cam0_path,
                cam1_path=cam1_path,
                cam0_stamp=stamp if cam0_path else None,
                cam1_stamp=stamp if cam1_path else None,
            )
            meta_path = self._write_capture_metadata(out_dir, session, metadata)
            msg = f"{msg}; metadata={meta_path}"

        self.get_logger().info("Capture OK")
        return True, msg, cam0_path, cam1_path, stamp

    def _perform_capture_stream(
        self,
        session_in: str,
        out_dir_in: str,
        quality_in: int,
        feedback_cb: Optional[Callable[[str], None]] = None,
    ) -> Tuple[bool, str, str, str, TimeMsg]:
        default_quality = int(self.get_parameter("default_quality").value)
        quality = int(quality_in) if quality_in > 0 else default_quality
        quality = max(10, min(100, quality))

        out_dir = out_dir_in.strip() if out_dir_in else ""
        out_dir = out_dir or os.path.expanduser("~/captures")
        os.makedirs(out_dir, exist_ok=True)

        session = session_in.strip() if session_in else ""
        session = session or datetime.now().strftime("%Y%m%d_%H%M%S")
        cam0_path = os.path.join(out_dir, f"{session}_cam0.jpg")
        cam1_path = os.path.join(out_dir, f"{session}_cam1.jpg")

        trigger_stamp = now_ros_time(self)
        self.get_logger().info(f"Capture(stream) session={session} -> {cam0_path}, {cam1_path}")

        if self._fallback_timer is not None:
            fail_msg = (
                "CAPTURE FAILED (stream mode): preview fallback is active; "
                "no verified real camera stream available."
            )
            self.get_logger().error(fail_msg)
            return False, fail_msg, "", "", trigger_stamp

        cam_count = self._camera_count_for_capture()
        require_cam1 = (cam_count is not None and cam_count >= 2)

        wait_s = max(0.1, float(self.get_parameter("stream_wait_s").value))
        max_age_s = max(0.05, float(self.get_parameter("stream_max_frame_age_s").value))
        deadline = time.monotonic() + wait_s

        msg0 = msg1 = None
        while True:
            msg0, msg1, rx0, rx1 = self._latest_stream_snapshot()
            now_m = time.monotonic()
            age0 = (now_m - rx0) if rx0 is not None else 1e9
            age1 = (now_m - rx1) if rx1 is not None else 1e9

            ok0 = (msg0 is not None) and (age0 <= max_age_s)
            ok1 = (not require_cam1) or ((msg1 is not None) and (age1 <= max_age_s))
            if ok0 and ok1:
                break
            if now_m >= deadline:
                fail_msg = (
                    "CAPTURE FAILED (stream mode)\n"
                    f"cam0_ready={ok0} age_s={age0:.3f}\n"
                    f"cam1_required={require_cam1} cam1_ready={ok1} age_s={age1:.3f}\n"
                    "No sufficiently fresh preview frames available."
                )
                self.get_logger().error(fail_msg)
                return False, fail_msg, "", "", trigger_stamp
            time.sleep(0.02)

        if feedback_cb is not None:
            feedback_cb("encoding_stream_frames")

        try:
            frame0 = self._imgmsg_to_bgr(msg0)
        except Exception as e:
            fail_msg = f"CAPTURE FAILED (stream mode): cam0 conversion failed: {e}"
            self.get_logger().error(fail_msg)
            return False, fail_msg, "", "", trigger_stamp

        if not self._write_jpeg_bgr(cam0_path, frame0, quality):
            fail_msg = "CAPTURE FAILED (stream mode): cam0 JPEG encode/write failed"
            self.get_logger().error(fail_msg)
            return False, fail_msg, "", "", trigger_stamp

        cam0_stamp = msg0.header.stamp
        cam1_stamp: Optional[TimeMsg] = None
        wrote_cam1 = False

        if msg1 is not None:
            try:
                frame1 = self._imgmsg_to_bgr(msg1)
            except Exception as e:
                fail_msg = f"CAPTURE FAILED (stream mode): cam1 conversion failed: {e}"
                self.get_logger().error(fail_msg)
                return False, fail_msg, cam0_path, "", trigger_stamp
            if not self._write_jpeg_bgr(cam1_path, frame1, quality):
                fail_msg = "CAPTURE FAILED (stream mode): cam1 JPEG encode/write failed"
                self.get_logger().error(fail_msg)
                return False, fail_msg, cam0_path, "", trigger_stamp
            cam1_stamp = msg1.header.stamp
            wrote_cam1 = True
        else:
            cam1_path = ""

        stamp = cam0_stamp if cam0_stamp is not None else trigger_stamp
        if wrote_cam1:
            msg = "OK (stream mode)"
        else:
            msg = "OK (stream mode, cam1 skipped: only one stream detected)"

        if bool(self.get_parameter("write_capture_metadata").value):
            metadata = self._build_capture_metadata(
                mode="stream",
                session=session,
                trigger_stamp=trigger_stamp,
                cam0_path=cam0_path,
                cam1_path=cam1_path,
                cam0_stamp=cam0_stamp if cam0_stamp is not None else trigger_stamp,
                cam1_stamp=cam1_stamp,
            )
            meta_path = self._write_capture_metadata(out_dir, session, metadata)
            msg = f"{msg}; metadata={meta_path}"

        self.get_logger().info("Capture OK (stream mode)")
        return True, msg, cam0_path, cam1_path, stamp

    def _perform_capture(
        self,
        session_in: str,
        out_dir_in: str,
        quality_in: int,
        feedback_cb: Optional[Callable[[str], None]] = None,
    ) -> Tuple[bool, str, str, str, TimeMsg]:
        mode = self._capture_mode()
        if mode == "stream":
            return self._perform_capture_stream(session_in, out_dir_in, quality_in, feedback_cb=feedback_cb)
        return self._perform_capture_still(session_in, out_dir_in, quality_in, feedback_cb=feedback_cb)

    def on_capture(self, req: CapturePair.Request, res: CapturePair.Response) -> CapturePair.Response:
        with self._capture_lock:
            success, message, cam0_path, cam1_path, stamp = self._perform_capture(
                req.session_id,
                req.output_dir,
                int(req.jpeg_quality),
            )
        res.success = success
        res.message = message
        res.cam0_path = cam0_path
        res.cam1_path = cam1_path
        res.stamp = stamp
        return res

    def on_capture_goal(self, goal_request: CapturePairAction.Goal) -> GoalResponse:
        del goal_request
        return GoalResponse.ACCEPT

    def on_capture_cancel(self, goal_handle) -> CancelResponse:
        del goal_handle
        # Capture is not safely cancelable once camera handover starts.
        return CancelResponse.REJECT

    def on_capture_action(self, goal_handle) -> CapturePairAction.Result:
        goal = goal_handle.request

        def feedback(stage: str) -> None:
            fb = CapturePairAction.Feedback()
            fb.stage = stage
            goal_handle.publish_feedback(fb)

        with self._capture_lock:
            success, message, cam0_path, cam1_path, stamp = self._perform_capture(
                goal.session_id,
                goal.output_dir,
                int(goal.jpeg_quality),
                feedback_cb=feedback,
            )

        result = CapturePairAction.Result()
        result.success = success
        result.message = message
        result.cam0_path = cam0_path
        result.cam1_path = cam1_path
        result.stamp = stamp

        if success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result


def main():
    rclpy.init()
    node = CaptureService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Ensure we don't leave camera processes behind if we own them.
        try:
            if bool(node.get_parameter("manage_previews").value):
                node._stop_previews_managed()  # cleanup
        except Exception:
            pass
        try:
            node._stop_black_previews()
        except Exception:
            pass
        try:
            node._cleanup_gpio_trigger()
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        rclpy.try_shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
