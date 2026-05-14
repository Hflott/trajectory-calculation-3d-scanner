#!/usr/bin/env python3
import csv
import json
import math
import os
import re
from bisect import bisect_left, bisect_right
import shutil
import time
import subprocess
import signal
import threading
import traceback
from collections import deque
from datetime import datetime
from functools import lru_cache
from typing import Any, Deque, Dict, List, Tuple, Optional, Callable

import cv2
import numpy as np
import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from builtin_interfaces.msg import Time as TimeMsg

from cv_bridge import CvBridge
from ament_index_python.packages import get_package_prefix

from subsea_interfaces.action import CapturePair as CapturePairAction
from subsea_interfaces.srv import CapturePair
from sensor_msgs.msg import Image, NavSatFix, TimeReference, Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from rcl_interfaces.msg import SetParametersResult
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

StreamFrame = Tuple[Image, float, Optional[int]]
GyroSample = Tuple[int, Tuple[float, float, float]]


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


def _reliability_name(policy: ReliabilityPolicy) -> str:
    if policy == ReliabilityPolicy.BEST_EFFORT:
        return "best_effort"
    if policy == ReliabilityPolicy.RELIABLE:
        return "reliable"
    return "system_default"


def _parse_reliability_policy(
    value: Any,
    default: ReliabilityPolicy,
    *,
    logger=None,
    param_name: str = "",
) -> ReliabilityPolicy:
    text = str(value).strip().lower().replace("-", "_")
    mapping = {
        "best_effort": ReliabilityPolicy.BEST_EFFORT,
        "besteffort": ReliabilityPolicy.BEST_EFFORT,
        "reliable": ReliabilityPolicy.RELIABLE,
        "system_default": ReliabilityPolicy.SYSTEM_DEFAULT,
        "default": ReliabilityPolicy.SYSTEM_DEFAULT,
    }
    out = mapping.get(text)
    if out is not None:
        return out
    if logger is not None:
        suffix = f" for {param_name}" if param_name else ""
        logger.warn(
            f"Invalid QoS reliability{suffix}={value!r}; "
            f"using '{_reliability_name(default)}'"
        )
    return default


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


def _ns_to_stamp_str(ns: int) -> str:
    sec = int(ns // 1_000_000_000)
    nsec = int(ns % 1_000_000_000)
    return f"{sec}.{nsec:09d}"


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def _safe_float(v: Any, default: float = 0.0) -> float:
    try:
        return float(v)
    except Exception:
        return default


def _normalize_orientation_deg(v: Any) -> int:
    try:
        raw = int(v)
    except Exception:
        raw = 0
    # Orientation is expressed in 90-degree steps.
    return int((round(raw / 90.0) % 4) * 90)


def _apply_orientation_bgr(frame: np.ndarray, orientation_deg: int) -> np.ndarray:
    o = _normalize_orientation_deg(orientation_deg)
    if o == 180:
        return cv2.flip(frame, -1)
    if o == 90:
        return cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
    if o == 270:
        return cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
    return frame


def _lerp_vec3(
    a: Tuple[float, float, float],
    b: Tuple[float, float, float],
    alpha: float,
) -> Tuple[float, float, float]:
    return (
        a[0] + (b[0] - a[0]) * alpha,
        a[1] + (b[1] - a[1]) * alpha,
        a[2] + (b[2] - a[2]) * alpha,
    )


def _quat_norm(q: List[float]) -> List[float]:
    n = math.sqrt(sum(x * x for x in q))
    if n <= 1e-12:
        return [0.0, 0.0, 0.0, 1.0]
    inv = 1.0 / n
    return [q[0] * inv, q[1] * inv, q[2] * inv, q[3] * inv]


def _quat_slerp(q0: List[float], q1: List[float], t: float) -> List[float]:
    t = _clamp(float(t), 0.0, 1.0)
    a = _quat_norm(q0)
    b = _quat_norm(q1)
    dot = a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3]
    if dot < 0.0:
        b = [-b[0], -b[1], -b[2], -b[3]]
        dot = -dot
    if dot > 0.9995:
        out = [a[i] + t * (b[i] - a[i]) for i in range(4)]
        return _quat_norm(out)
    theta_0 = math.acos(_clamp(dot, -1.0, 1.0))
    sin_theta_0 = math.sin(theta_0)
    if abs(sin_theta_0) < 1e-8:
        return a
    theta = theta_0 * t
    sin_theta = math.sin(theta)
    s0 = math.cos(theta) - dot * sin_theta / sin_theta_0
    s1 = sin_theta / sin_theta_0
    return [
        s0 * a[0] + s1 * b[0],
        s0 * a[1] + s1 * b[1],
        s0 * a[2] + s1 * b[2],
        s0 * a[3] + s1 * b[3],
    ]


@lru_cache(maxsize=256)
def _line_psf_cached(length_px_rounded: int, angle_tenths: int, max_kernel: int) -> np.ndarray:
    l = int(max(1, length_px_rounded))
    base = max(3, l * 2 + 1)
    k = min(base, max_kernel)
    if k % 2 == 0:
        k += 1
    psf = np.zeros((k, k), dtype=np.float32)
    c = k // 2
    ang = math.radians(float(angle_tenths) / 10.0)
    dx = int(round((l - 1) * 0.5 * math.cos(ang)))
    dy = int(round((l - 1) * 0.5 * math.sin(ang)))
    x0 = int(_clamp(c - dx, 0, k - 1))
    y0 = int(_clamp(c - dy, 0, k - 1))
    x1 = int(_clamp(c + dx, 0, k - 1))
    y1 = int(_clamp(c + dy, 0, k - 1))
    cv2.line(psf, (x0, y0), (x1, y1), 1.0, 1)
    s = float(psf.sum())
    if s <= 1e-9:
        psf[c, c] = 1.0
        s = 1.0
    psf /= s
    psf.setflags(write=False)
    return psf


def _line_psf(length_px: float, angle_deg: float, max_kernel: int) -> np.ndarray:
    l = int(max(1, round(float(length_px))))
    max_kernel = max(3, int(max_kernel))
    if max_kernel % 2 == 0:
        max_kernel += 1
    # A line PSF is symmetric for angle+180. Quantizing to 0.1 deg keeps
    # cache churn low without changing the rasterized kernel meaningfully.
    angle_tenths = int(round(float(angle_deg) * 10.0)) % 1800
    return _line_psf_cached(l, angle_tenths, max_kernel)


def _richardson_lucy_bgr(
    img_bgr: np.ndarray,
    psf: np.ndarray,
    iterations: int,
) -> np.ndarray:
    iters = max(1, int(iterations))
    img = img_bgr.astype(np.float32) / 255.0
    est = np.clip(img, 1e-4, 1.0)
    ker = psf.astype(np.float32)
    ker /= max(1e-9, float(ker.sum()))
    ker_flip = cv2.flip(ker, -1)
    eps = 1e-6
    for _ in range(iters):
        conv = cv2.filter2D(est, -1, ker, borderType=cv2.BORDER_REPLICATE)
        rel = img / np.maximum(conv, eps)
        est *= cv2.filter2D(rel, -1, ker_flip, borderType=cv2.BORDER_REPLICATE)
        est = np.clip(est, 0.0, 1.0)
    return np.clip(est * 255.0, 0.0, 255.0).astype(np.uint8)


def _wiener_deconvolve_bgr(
    img_bgr: np.ndarray,
    psf: np.ndarray,
    snr: float = 40.0,
) -> np.ndarray:
    """Wiener deconvolution via FFT — O(N log N), single pass, much faster than Richardson-Lucy.

    snr is the assumed signal-to-noise ratio (power, not dB). Higher values sharpen more
    aggressively; lower values are more conservative. Typical range: 10–200.
    """
    img = img_bgr.astype(np.float32) / 255.0
    h, w = img.shape[:2]
    ph, pw = psf.shape
    # Embed PSF in a full-size plane and roll its centre to (0, 0) so that
    # rfft2 treats it as a circularly-centred kernel (no phase offset).
    psf_full = np.zeros((h, w), dtype=np.float32)
    psf_full[:ph, :pw] = psf
    psf_full = np.roll(np.roll(psf_full, -(ph // 2), axis=0), -(pw // 2), axis=1)
    H = np.fft.rfft2(psf_full)
    H_conj = np.conj(H)
    # Wiener formula: F_est = (H* · G) / (|H|² + 1/SNR)
    denom = (H_conj * H).real + (1.0 / max(1.0, float(snr)))
    G = np.fft.rfft2(img, axes=(0, 1))
    result = np.fft.irfft2(
        (H_conj[:, :, None] * G) / denom[:, :, None],
        s=(h, w),
        axes=(0, 1),
    )
    result = np.clip(result, 0.0, 1.0)
    return np.clip(result * 255.0, 0.0, 255.0).astype(np.uint8)


class CaptureService(Node):
    def __init__(self):
        super().__init__("capture_service")

        self.declare_parameter("cam0_index", 0)
        self.declare_parameter("cam1_index", 1)

        self.declare_parameter("width", 4056)
        self.declare_parameter("height", 3040)

        self.declare_parameter("warmup_ms", 350)
        self.declare_parameter("timeout_ms", 6000)
        self.declare_parameter("default_quality", 100)
        self.declare_parameter("capture_mode", "stream")  # stream|still
        self.declare_parameter("stream_wait_s", 1.0)
        self.declare_parameter("stream_initial_wait_s", 5.0)
        self.declare_parameter("stream_max_frame_age_s", 1.0)
        self.declare_parameter("stream_buffer_len", 60)
        self.declare_parameter("stream_pair_max_delta_ms", 80.0)
        self.declare_parameter("write_capture_metadata", True)
        self.declare_parameter("sensor_buffer_s", 20.0)
        self.declare_parameter("sensor_buffer_max_samples", 12000)
        self.declare_parameter("trajectory_sample_rate_hz", 100.0)
        self.declare_parameter("trajectory_window_ms", 1000.0)
        self.declare_parameter("write_trajectory_csv", True)
        self.declare_parameter("enable_motion_deblur", True)
        self.declare_parameter("deblur_exposure_ms", 8.0)
        self.declare_parameter("deblur_exposure_time_us", 0)
        self.declare_parameter("deblur_image_stamp_reference", "midpoint")  # midpoint|start|end
        self.declare_parameter("deblur_timestamp_source", "pps_disciplined_system_clock")
        self.declare_parameter("deblur_fov_deg", 72.0)
        self.declare_parameter("deblur_strength", 1.0)
        self.declare_parameter("deblur_min_kernel_px", 1.2)
        self.declare_parameter("deblur_max_kernel_px", 31)
        self.declare_parameter("deblur_iterations", 12)
        self.declare_parameter("deblur_method", "richardson_lucy")  # richardson_lucy|wiener
        self.declare_parameter("deblur_wiener_snr", 40.0)
        self.declare_parameter("deblur_imu_to_cam_yaw_deg", 0.0)
        self.declare_parameter("deblur_use_translation", False)
        self.declare_parameter("deblur_assumed_depth_m", 1.5)
        self.declare_parameter("deblur_max_odom_age_ms", 200.0)
        self.declare_parameter("deblur_allow_nearest_fallback", False)
        self.declare_parameter("deblur_max_integration_gap_ms", 25.0)
        self.declare_parameter("deblur_require_time_reference", True)
        self.declare_parameter("deblur_max_time_reference_age_ms", 2000.0)

        # Preview management
        self.declare_parameter("manage_previews", False)
        self.declare_parameter("start_previews", True)
        self.declare_parameter("pause_previews", True)
        self.declare_parameter("auto_detect_cameras", True)
        self.declare_parameter("fallback_black_previews", True)

        self.declare_parameter("preview_width", 960)
        self.declare_parameter("preview_height", 540)
        self.declare_parameter("preview_fps", 20)
        self.declare_parameter("preview_relay_enable", True)
        self.declare_parameter("preview_relay_width", 640)
        self.declare_parameter("preview_relay_height", 360)
        self.declare_parameter("preview_relay_fps", 10)
        self.declare_parameter("preview_format", "RGB888")
        self.declare_parameter("cam0_orientation", 0)
        self.declare_parameter("cam1_orientation", 0)
        # When True, preview camera nodes rotate frames and capture_service must NOT rotate again.
        self.declare_parameter("preview_source_applies_orientation", True)
        self.declare_parameter("preview_role", "viewfinder")
        self.declare_parameter("preview_start_stagger_s", 0.7)
        self.declare_parameter("preview_restart_attempts", 2)
        self.declare_parameter("preview_restart_delay_s", 0.6)
        self.declare_parameter("preview_watchdog_enable", True)
        self.declare_parameter("preview_watchdog_period_s", 1.0)
        self.declare_parameter("preview_watchdog_min_restart_interval_s", 2.0)
        self.declare_parameter("preview_watchdog_stale_s", 3.0)

        self.declare_parameter("cam0_namespace", "/cam0")
        self.declare_parameter("cam1_namespace", "/cam1")
        self.declare_parameter("cam0_node_name", "camera")
        self.declare_parameter("cam1_node_name", "camera")
        self.declare_parameter("ui_cam0_node_name", "preview")
        self.declare_parameter("ui_cam1_node_name", "preview")
        self.declare_parameter("ui_cam0_topic", "")
        self.declare_parameter("ui_cam1_topic", "")
        self.declare_parameter("use_local_libcamera_env", False)
        self.declare_parameter("sanitize_preview_env", True)
        self.declare_parameter("gnss_fix_topic", "/fix")
        self.declare_parameter("gnss_time_ref_topic", "/time_reference")
        self.declare_parameter("gnss_fix_qos_reliability", "best_effort")
        self.declare_parameter("gnss_time_ref_qos_reliability", "best_effort")
        self.declare_parameter("gnss_imu_topic", "/imu/data")
        self.declare_parameter("odom_local_topic", "/odometry/local")
        self.declare_parameter("odom_global_topic", "/odometry/global")
        self.declare_parameter("capture_event_topic", "/capture/events")
        self.declare_parameter("capture_debug_topic", "/capture/debug")
        self.declare_parameter("gpio_trigger_enable", False)
        self.declare_parameter("gpio_trigger_chip", "/dev/gpiochip0")
        self.declare_parameter("gpio_trigger_line", 24)
        self.declare_parameter("gpio_trigger_active_low", True)
        self.declare_parameter("gpio_trigger_cooldown_ms", 1000)
        self.declare_parameter("gpio_trigger_debounce_ms", 40)
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
        self._relay_pub0 = None
        self._relay_pub1 = None
        self._relay_timer = None
        self._preview_watchdog_timer = None
        self._preview_watchdog_lock = threading.Lock()
        self._preview_watchdog_last_restart_mono = 0.0
        self._preview_watchdog_epoch_mono = time.monotonic()
        self._relay_width = max(64, int(self.get_parameter("preview_relay_width").value))
        self._relay_height = max(64, int(self.get_parameter("preview_relay_height").value))
        self._relay_last_cam0_msg_id: Optional[int] = None
        self._relay_last_cam1_msg_id: Optional[int] = None
        self._relay_last_warn_mono = 0.0
        self._detected_cam_count: Optional[int] = None
        self._expected_preview_cams: Optional[int] = None
        self._preview_restart_count: int = 0
        self._bridge = CvBridge()
        self._stream_lock = threading.Lock()
        self._latest_cam0_msg: Optional[Image] = None
        self._latest_cam1_msg: Optional[Image] = None
        self._latest_cam0_rx_mono: Optional[float] = None
        self._latest_cam1_rx_mono: Optional[float] = None
        stream_len = max(5, int(self.get_parameter("stream_buffer_len").value))
        self._buf_cam0: Deque[StreamFrame] = deque(maxlen=stream_len)
        self._buf_cam1: Deque[StreamFrame] = deque(maxlen=stream_len)

        self._sensor_lock = threading.Lock()
        self._sensor_max_samples = max(200, int(self.get_parameter("sensor_buffer_max_samples").value))
        self._buf_fix: Deque[NavSatFix] = deque(maxlen=self._sensor_max_samples)
        self._buf_time_ref: Deque[TimeReference] = deque(maxlen=self._sensor_max_samples)
        self._buf_imu: Deque[Imu] = deque(maxlen=self._sensor_max_samples)
        self._buf_odom_local: Deque[Odometry] = deque(maxlen=self._sensor_max_samples)
        self._buf_odom_global: Deque[Odometry] = deque(maxlen=self._sensor_max_samples)
        self._sensor_keep_s = max(2.0, float(self.get_parameter("sensor_buffer_s").value))
        self._sensor_trim_period_s = 0.25
        self._next_sensor_trim_mono = 0.0
        self._gpio_mod = None
        self._gpio_chip = None
        self._gpio_line_obj = None
        self._gpio_req = None
        self._gpio_timer = None
        self._gpio_prev_pressed = False
        self._gpio_pressed_since_mono: Optional[float] = None
        self._gpio_last_trigger_mono = 0.0
        self._gpio_line_offset = int(self.get_parameter("gpio_trigger_line").value)
        self._gpio_active_low = bool(self.get_parameter("gpio_trigger_active_low").value)
        self._gpio_debounce_s = max(
            0.0,
            float(self.get_parameter("gpio_trigger_debounce_ms").value) / 1000.0,
        )
        self._gpio_cooldown_s = max(
            0.05,
            float(self.get_parameter("gpio_trigger_cooldown_ms").value) / 1000.0,
        )
        self._gpio_capture_thread = None

        # Separate callback groups so long capture handlers don't starve sensor streams.
        self._stream_cb_group = ReentrantCallbackGroup()
        self._timer_cb_group = ReentrantCallbackGroup()
        self._capture_cb_group = MutuallyExclusiveCallbackGroup()

        img_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=2,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        fix_rel = _parse_reliability_policy(
            self.get_parameter("gnss_fix_qos_reliability").value,
            ReliabilityPolicy.BEST_EFFORT,
            logger=self.get_logger(),
            param_name="gnss_fix_qos_reliability",
        )
        time_ref_rel = _parse_reliability_policy(
            self.get_parameter("gnss_time_ref_qos_reliability").value,
            ReliabilityPolicy.BEST_EFFORT,
            logger=self.get_logger(),
            param_name="gnss_time_ref_qos_reliability",
        )
        sens_qos_fix = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=fix_rel,
            durability=DurabilityPolicy.VOLATILE,
        )
        sens_qos_time_ref = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=time_ref_rel,
            durability=DurabilityPolicy.VOLATILE,
        )
        # Keep odometry on RELIABLE to match robot_localization defaults.
        sens_qos_odom = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        # IMU sources are commonly BEST_EFFORT.
        sens_qos_best_effort = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        ns0 = str(self.get_parameter("cam0_namespace").value)
        ns1 = str(self.get_parameter("cam1_namespace").value)
        n0 = str(self.get_parameter("cam0_node_name").value)
        n1 = str(self.get_parameter("cam1_node_name").value)
        self._stream_cam0_topic = self._preview_topic(ns0, n0)
        self._stream_cam1_topic = self._preview_topic(ns1, n1)
        ui_n0 = str(self.get_parameter("ui_cam0_node_name").value)
        ui_n1 = str(self.get_parameter("ui_cam1_node_name").value)
        ui_t0 = str(self.get_parameter("ui_cam0_topic").value).strip()
        ui_t1 = str(self.get_parameter("ui_cam1_topic").value).strip()
        self._ui_cam0_topic = self._ui_topic(ns0, ui_n0, ui_t0)
        self._ui_cam1_topic = self._ui_topic(ns1, ui_n1, ui_t1)
        self._cam0_sub = self.create_subscription(
            Image,
            self._stream_cam0_topic,
            self._on_cam0_image,
            img_qos,
            callback_group=self._stream_cb_group,
        )
        self._cam1_sub = self.create_subscription(
            Image,
            self._stream_cam1_topic,
            self._on_cam1_image,
            img_qos,
            callback_group=self._stream_cb_group,
        )
        self.get_logger().info(
            f"Stream capture subscribers: cam0={self._stream_cam0_topic} cam1={self._stream_cam1_topic}"
        )
        self.get_logger().info(
            f"UI preview topics: cam0={self._ui_cam0_topic} cam1={self._ui_cam1_topic}"
        )
        self._start_preview_relay()

        fix_topic = str(self.get_parameter("gnss_fix_topic").value)
        time_ref_topic = str(self.get_parameter("gnss_time_ref_topic").value)
        imu_topic = str(self.get_parameter("gnss_imu_topic").value)
        odom_local_topic = str(self.get_parameter("odom_local_topic").value)
        odom_global_topic = str(self.get_parameter("odom_global_topic").value)
        self._fix_sub = self.create_subscription(
            NavSatFix,
            fix_topic,
            self._on_fix,
            sens_qos_fix,
            callback_group=self._stream_cb_group,
        )
        self._time_ref_sub = self.create_subscription(
            TimeReference,
            time_ref_topic,
            self._on_time_ref,
            sens_qos_time_ref,
            callback_group=self._stream_cb_group,
        )
        self._imu_sub = self.create_subscription(
            Imu,
            imu_topic,
            self._on_imu,
            sens_qos_best_effort,
            callback_group=self._stream_cb_group,
        )
        self._odom_local_sub = self.create_subscription(
            Odometry,
            odom_local_topic,
            self._on_odom_local,
            sens_qos_odom,
            callback_group=self._stream_cb_group,
        )
        self._odom_global_sub = self.create_subscription(
            Odometry,
            odom_global_topic,
            self._on_odom_global,
            sens_qos_odom,
            callback_group=self._stream_cb_group,
        )
        self.get_logger().info(
            "Telemetry subscribers: "
            f"fix={fix_topic} time_ref={time_ref_topic} imu={imu_topic} "
            f"odom_local={odom_local_topic} odom_global={odom_global_topic}"
        )
        self.get_logger().info(
            "Telemetry QoS reliability: "
            f"fix={_reliability_name(fix_rel)} "
            f"time_ref={_reliability_name(time_ref_rel)} "
            "imu=best_effort odom=reliable"
        )
        event_topic = str(self.get_parameter("capture_event_topic").value)
        self._capture_evt_pub = self.create_publisher(String, event_topic, 10)
        self.get_logger().info(f"Capture event publisher: {event_topic}")
        debug_topic = str(self.get_parameter("capture_debug_topic").value)
        self._capture_dbg_pub = self.create_publisher(String, debug_topic, 10)
        self.get_logger().info(f"Capture debug publisher: {debug_topic}")

        self.srv = self.create_service(
            CapturePair,
            "capture_pair",
            self.on_capture,
            callback_group=self._capture_cb_group,
        )
        self.action = ActionServer(
            self,
            CapturePairAction,
            "capture_pair",
            goal_callback=self.on_capture_goal,
            cancel_callback=self.on_capture_cancel,
            execute_callback=self.on_capture_action,
            callback_group=self._capture_cb_group,
        )
        self.get_logger().info("Capture service ready: /capture_pair")
        self.get_logger().info("Capture action ready: /capture_pair")
        self.get_logger().info(f"Capture mode: {self._capture_mode()}")
        self.get_logger().info(
            "Camera orientation params: "
            f"cam0_orientation={self._camera_orientation_for_index(int(self.get_parameter('cam0_index').value))} "
            f"cam1_orientation={self._camera_orientation_for_index(int(self.get_parameter('cam1_index').value))}"
        )
        self._setup_gpio_trigger()
        self.add_on_set_parameters_callback(self._on_set_parameters)
        self._start_preview_watchdog()

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
        relay_reasons: List[str] = []
        sensor_keep_s: Optional[float] = None
        sensor_max_samples: Optional[int] = None
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
            elif p.name in ("cam0_orientation", "cam1_orientation"):
                ov = _normalize_orientation_deg(p.value)
                restart_reasons.append(f"{p.name}={ov}")
                relay_reasons.append(f"{p.name}={ov}")
            elif p.name == "preview_source_applies_orientation":
                restart_reasons.append(f"preview_source_applies_orientation={bool(p.value)}")
                relay_reasons.append(f"preview_source_applies_orientation={bool(p.value)}")
            elif p.name == "preview_relay_fps":
                try:
                    v = int(p.value)
                except Exception:
                    return SetParametersResult(successful=False, reason="preview_relay_fps must be an integer")
                if v < 1 or v > 120:
                    return SetParametersResult(successful=False, reason="preview_relay_fps must be in [1,120]")
                relay_reasons.append(f"preview_relay_fps={v}")
            elif p.name == "preview_relay_width":
                try:
                    v = int(p.value)
                except Exception:
                    return SetParametersResult(successful=False, reason="preview_relay_width must be an integer")
                if v < 64:
                    return SetParametersResult(successful=False, reason="preview_relay_width must be >= 64")
                relay_reasons.append(f"preview_relay_width={v}")
            elif p.name == "preview_relay_height":
                try:
                    v = int(p.value)
                except Exception:
                    return SetParametersResult(successful=False, reason="preview_relay_height must be an integer")
                if v < 64:
                    return SetParametersResult(successful=False, reason="preview_relay_height must be >= 64")
                relay_reasons.append(f"preview_relay_height={v}")
            elif p.name in (
                "preview_relay_enable",
                "ui_cam0_node_name",
                "ui_cam1_node_name",
                "ui_cam0_topic",
                "ui_cam1_topic",
            ):
                relay_reasons.append(f"{p.name}={p.value}")
            elif p.name == "sensor_buffer_s":
                try:
                    v = float(p.value)
                except Exception:
                    return SetParametersResult(successful=False, reason="sensor_buffer_s must be a number")
                sensor_keep_s = max(2.0, v)
            elif p.name == "sensor_buffer_max_samples":
                try:
                    v = int(p.value)
                except Exception:
                    return SetParametersResult(successful=False, reason="sensor_buffer_max_samples must be an integer")
                if v < 200:
                    return SetParametersResult(successful=False, reason="sensor_buffer_max_samples must be >= 200")
                sensor_max_samples = v

        if restart_reasons:
            self._request_preview_reconfigure(", ".join(restart_reasons))
        if relay_reasons:
            self._restart_preview_relay(", ".join(relay_reasons))
        if sensor_keep_s is not None or sensor_max_samples is not None:
            with self._sensor_lock:
                if sensor_keep_s is not None:
                    self._sensor_keep_s = sensor_keep_s
                if sensor_max_samples is not None and sensor_max_samples != self._sensor_max_samples:
                    self._sensor_max_samples = sensor_max_samples
                    self._buf_fix = deque(self._buf_fix, maxlen=self._sensor_max_samples)
                    self._buf_time_ref = deque(self._buf_time_ref, maxlen=self._sensor_max_samples)
                    self._buf_imu = deque(self._buf_imu, maxlen=self._sensor_max_samples)
                    self._buf_odom_local = deque(self._buf_odom_local, maxlen=self._sensor_max_samples)
                    self._buf_odom_global = deque(self._buf_odom_global, maxlen=self._sensor_max_samples)
                self._next_sensor_trim_mono = 0.0
                self._trim_sensor_buffers_locked(force=True)
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
                    self._stop_previews_managed()
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

    def _normalize_topic(self, topic: str) -> str:
        t = (topic or "").strip()
        if not t:
            return ""
        if not t.startswith("/"):
            t = "/" + t
        return t

    def _ui_topic(self, ns: str, node_name: str, explicit_topic: str) -> str:
        explicit = self._normalize_topic(explicit_topic)
        if explicit:
            return explicit
        return self._preview_topic(ns, node_name)

    def _refresh_ui_topics(self) -> None:
        ns0 = str(self.get_parameter("cam0_namespace").value)
        ns1 = str(self.get_parameter("cam1_namespace").value)
        ui_n0 = str(self.get_parameter("ui_cam0_node_name").value)
        ui_n1 = str(self.get_parameter("ui_cam1_node_name").value)
        ui_t0 = str(self.get_parameter("ui_cam0_topic").value).strip()
        ui_t1 = str(self.get_parameter("ui_cam1_topic").value).strip()
        self._ui_cam0_topic = self._ui_topic(ns0, ui_n0, ui_t0)
        self._ui_cam1_topic = self._ui_topic(ns1, ui_n1, ui_t1)

    def _warn_preview_relay(self, message: str) -> None:
        now_m = time.monotonic()
        if (now_m - self._relay_last_warn_mono) < 1.0:
            return
        self._relay_last_warn_mono = now_m
        self.get_logger().warn(message)

    def _start_preview_watchdog(self) -> None:
        if self._preview_watchdog_timer is not None:
            return
        if not bool(self.get_parameter("preview_watchdog_enable").value):
            return
        period_s = max(0.2, float(self.get_parameter("preview_watchdog_period_s").value))
        self._preview_watchdog_timer = self.create_timer(
            period_s,
            self._preview_watchdog_cb,
            callback_group=self._timer_cb_group,
        )
        self.get_logger().info(f"Preview watchdog enabled: period={period_s:.2f}s")

    def _stop_preview_watchdog(self) -> None:
        if self._preview_watchdog_timer is None:
            return
        try:
            self.destroy_timer(self._preview_watchdog_timer)
        except Exception:
            try:
                self._preview_watchdog_timer.cancel()
            except Exception:
                pass
        self._preview_watchdog_timer = None

    def _preview_watchdog_cb(self) -> None:
        # Runtime recovery: if one preview process dies after startup, restart it.
        if not bool(self.get_parameter("preview_watchdog_enable").value):
            return
        if not bool(self.get_parameter("manage_previews").value):
            return
        if not bool(self.get_parameter("start_previews").value):
            return
        if self._fallback_timer is not None:
            return
        if self._capture_lock.locked():
            return
        with self._preview_reconfig_lock:
            if self._preview_reconfig_pending:
                return
        if self._preview_reconfig_thread is not None and self._preview_reconfig_thread.is_alive():
            return
        if not self._preview_watchdog_lock.acquire(blocking=False):
            return

        try:
            expected = self._expected_preview_cams
            dead: List[str] = []
            reasons: List[str] = []
            now_m = time.monotonic()
            stale_s = max(0.5, float(self.get_parameter("preview_watchdog_stale_s").value))
            with self._stream_lock:
                rx0 = self._latest_cam0_rx_mono
                rx1 = self._latest_cam1_rx_mono

            def _stale(rx_mono: Optional[float]) -> bool:
                if rx_mono is None:
                    return (now_m - self._preview_watchdog_epoch_mono) > stale_s
                return (now_m - rx_mono) > stale_s

            if expected is None:
                # Unknown camera count: recover only if both previews are dead.
                p0_dead = (self._p0 is None) or (self._p0.poll() is not None)
                p1_dead = (self._p1 is None) or (self._p1.poll() is not None)
                p0_stale = _stale(rx0)
                p1_stale = _stale(rx1)
                if (p0_dead and p1_dead) or (p0_stale and p1_stale):
                    dead = ["cam0", "cam1"]
                    if p0_dead and p1_dead:
                        reasons.append("both processes exited")
                    else:
                        reasons.append(f"both streams stale > {stale_s:.1f}s")
            else:
                if expected >= 1:
                    p0_dead = (self._p0 is None) or (self._p0.poll() is not None)
                    if p0_dead:
                        dead.append("cam0")
                        reasons.append("cam0 process exited")
                    elif _stale(rx0):
                        dead.append("cam0")
                        reasons.append(f"cam0 stream stale > {stale_s:.1f}s")
                if expected >= 2:
                    p1_dead = (self._p1 is None) or (self._p1.poll() is not None)
                    if p1_dead:
                        dead.append("cam1")
                        reasons.append("cam1 process exited")
                    elif _stale(rx1):
                        dead.append("cam1")
                        reasons.append(f"cam1 stream stale > {stale_s:.1f}s")

            if not dead:
                return

            min_interval = max(
                0.2,
                float(self.get_parameter("preview_watchdog_min_restart_interval_s").value),
            )
            if (now_m - self._preview_watchdog_last_restart_mono) < min_interval:
                return
            self._preview_watchdog_last_restart_mono = now_m

            self.get_logger().warn(
                "Preview watchdog: restarting "
                f"({', '.join(dead)}); reason: {'; '.join(reasons) if reasons else 'unknown'}."
            )
            self._restart_failed_previews(dead)
        except Exception as e:
            self.get_logger().error(f"Preview watchdog restart failed: {e}")
        finally:
            self._preview_watchdog_lock.release()

    def _start_preview_relay(self) -> None:
        if self._relay_timer is not None:
            return
        if not bool(self.get_parameter("preview_relay_enable").value):
            return

        self._refresh_ui_topics()
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        pub_topics: List[str] = []
        if self._ui_cam0_topic and self._ui_cam0_topic != self._stream_cam0_topic:
            self._relay_pub0 = self.create_publisher(Image, self._ui_cam0_topic, qos)
            pub_topics.append(self._ui_cam0_topic)
        elif self._ui_cam0_topic:
            self.get_logger().warn(
                f"Preview relay cam0 disabled (same input/output topic): {self._ui_cam0_topic}"
            )
            self._relay_pub0 = None

        if self._ui_cam1_topic and self._ui_cam1_topic != self._stream_cam1_topic:
            self._relay_pub1 = self.create_publisher(Image, self._ui_cam1_topic, qos)
            pub_topics.append(self._ui_cam1_topic)
        elif self._ui_cam1_topic:
            self.get_logger().warn(
                f"Preview relay cam1 disabled (same input/output topic): {self._ui_cam1_topic}"
            )
            self._relay_pub1 = None

        if self._relay_pub0 is None and self._relay_pub1 is None:
            return

        fps = max(1, int(self.get_parameter("preview_relay_fps").value))
        self._relay_width = max(64, int(self.get_parameter("preview_relay_width").value))
        self._relay_height = max(64, int(self.get_parameter("preview_relay_height").value))
        self._relay_last_cam0_msg_id = None
        self._relay_last_cam1_msg_id = None
        self._relay_timer = self.create_timer(
            1.0 / float(fps),
            self._publish_preview_relay,
            callback_group=self._timer_cb_group,
        )
        self.get_logger().info(
            f"Preview relay enabled: fps={fps} topics={' | '.join(pub_topics)}"
        )

    def _stop_preview_relay(self) -> None:
        if self._relay_timer is not None:
            try:
                self.destroy_timer(self._relay_timer)
            except Exception:
                try:
                    self._relay_timer.cancel()
                except Exception:
                    pass
        self._relay_timer = None
        if self._relay_pub0 is not None:
            try:
                self.destroy_publisher(self._relay_pub0)
            except Exception:
                pass
        if self._relay_pub1 is not None:
            try:
                self.destroy_publisher(self._relay_pub1)
            except Exception:
                pass
        self._relay_pub0 = None
        self._relay_pub1 = None
        self._relay_last_cam0_msg_id = None
        self._relay_last_cam1_msg_id = None

    def _restart_preview_relay(self, reason: str) -> None:
        try:
            self.get_logger().info(f"Applying preview relay update ({reason})")
            self._stop_preview_relay()
            self._start_preview_relay()
        except Exception as e:
            self.get_logger().error(f"Failed to apply preview relay update: {e}")

    def _publish_preview_relay_image(
        self,
        src: Image,
        pub,
        width: int,
        height: int,
        default_frame_id: str,
        orientation_deg: int = 0,
    ) -> None:
        src_enc = (src.encoding or "").lower().strip()
        src_w = int(src.width)
        src_h = int(src.height)
        src_step = int(src.step)
        if (
            int(orientation_deg) == 0
            and
            src_enc == "bgr8"
            and src_w == int(width)
            and src_h == int(height)
            and src_step >= (src_w * 3)
        ):
            if src.header.frame_id:
                # Fast path: direct relay without decode/resize/re-encode.
                pub.publish(src)
            else:
                out = Image()
                out.header.stamp = src.header.stamp
                out.header.frame_id = default_frame_id
                out.height = src_h
                out.width = src_w
                out.encoding = "bgr8"
                out.is_bigendian = bool(src.is_bigendian)
                out.step = src_step
                out.data = src.data
                pub.publish(out)
            return

        frame = self._imgmsg_to_bgr(src)
        if int(orientation_deg) != 0:
            frame = _apply_orientation_bgr(frame, orientation_deg)
        src_h, src_w = frame.shape[:2]
        if src_w != width or src_h != height:
            interp = cv2.INTER_AREA if (width < src_w or height < src_h) else cv2.INTER_LINEAR
            frame = cv2.resize(frame, (int(width), int(height)), interpolation=interp)

        out = Image()
        out.header.stamp = src.header.stamp
        out.header.frame_id = src.header.frame_id or default_frame_id
        out.height = int(frame.shape[0])
        out.width = int(frame.shape[1])
        out.encoding = "bgr8"
        out.is_bigendian = False
        out.step = int(out.width * 3)
        out.data = frame.tobytes()
        pub.publish(out)

    def _publish_preview_relay(self) -> None:
        if self._relay_pub0 is None and self._relay_pub1 is None:
            return

        width = self._relay_width
        height = self._relay_height
        cam0_idx = int(self.get_parameter("cam0_index").value)
        cam1_idx = int(self.get_parameter("cam1_index").value)
        if bool(self.get_parameter("preview_source_applies_orientation").value):
            cam0_orientation = 0
            cam1_orientation = 0
        else:
            cam0_orientation = self._camera_orientation_for_index(cam0_idx)
            cam1_orientation = self._camera_orientation_for_index(cam1_idx)
        msg0, msg1, _rx0, _rx1 = self._latest_stream_snapshot()

        if self._relay_pub0 is not None and msg0 is not None:
            msg_id = id(msg0)
            if self._relay_last_cam0_msg_id != msg_id:
                try:
                    self._publish_preview_relay_image(
                        msg0,
                        self._relay_pub0,
                        width,
                        height,
                        "cam0_optical_frame",
                        cam0_orientation,
                    )
                    self._relay_last_cam0_msg_id = msg_id
                except Exception as e:
                    self._warn_preview_relay(f"Preview relay cam0 drop: {e}")

        if self._relay_pub1 is not None and msg1 is not None:
            msg_id = id(msg1)
            if self._relay_last_cam1_msg_id != msg_id:
                try:
                    self._publish_preview_relay_image(
                        msg1,
                        self._relay_pub1,
                        width,
                        height,
                        "cam1_optical_frame",
                        cam1_orientation,
                    )
                    self._relay_last_cam1_msg_id = msg_id
                except Exception as e:
                    self._warn_preview_relay(f"Preview relay cam1 drop: {e}")

    def _setup_gpio_trigger(self) -> None:
        if not bool(self.get_parameter("gpio_trigger_enable").value):
            return

        try:
            import gpiod  # type: ignore
        except Exception as e:
            self.get_logger().error(f"GPIO trigger disabled: python gpiod import failed: {e}")
            return

        chip_name = str(self.get_parameter("gpio_trigger_chip").value)
        self._gpio_line_offset = int(self.get_parameter("gpio_trigger_line").value)
        line_offset = self._gpio_line_offset
        self._gpio_active_low = bool(self.get_parameter("gpio_trigger_active_low").value)
        active_low = self._gpio_active_low
        poll_ms = max(5, int(self.get_parameter("gpio_trigger_poll_ms").value))
        self._gpio_debounce_s = max(
            0.0,
            float(self.get_parameter("gpio_trigger_debounce_ms").value) / 1000.0,
        )
        self._gpio_cooldown_s = max(
            0.05,
            float(self.get_parameter("gpio_trigger_cooldown_ms").value) / 1000.0,
        )

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
        initial_pressed = self._read_gpio_pressed()
        self._gpio_prev_pressed = bool(initial_pressed) if initial_pressed is not None else False
        self._gpio_pressed_since_mono = time.monotonic() if self._gpio_prev_pressed else None
        self._gpio_last_trigger_mono = 0.0
        self._gpio_timer = self.create_timer(
            poll_ms / 1000.0,
            self._gpio_poll_cb,
            callback_group=self._timer_cb_group,
        )
        self.get_logger().info(
            f"GPIO trigger enabled: chip={chip_name} line={line_offset} "
            f"active_low={active_low} poll_ms={poll_ms} "
            f"debounce_ms={int(self._gpio_debounce_s * 1000.0)} "
            f"initial_pressed={self._gpio_prev_pressed}"
        )

    def _read_gpio_pressed(self) -> Optional[bool]:
        raw = None
        line_offset = self._gpio_line_offset
        active_low = self._gpio_active_low

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

        now_m = time.monotonic()

        if pressed:
            if self._gpio_pressed_since_mono is None:
                self._gpio_pressed_since_mono = now_m
        else:
            self._gpio_pressed_since_mono = None

        stable_pressed = bool(
            pressed
            and self._gpio_pressed_since_mono is not None
            and ((now_m - self._gpio_pressed_since_mono) >= self._gpio_debounce_s)
        )

        # Trigger on stable press edge only.
        if stable_pressed and not self._gpio_prev_pressed:
            now_m = time.monotonic()
            if (now_m - self._gpio_last_trigger_mono) >= self._gpio_cooldown_s:
                self._gpio_last_trigger_mono = now_m
                self._on_gpio_trigger()
        self._gpio_prev_pressed = stable_pressed

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
            success, message, cam0_path, cam1_path, stamp = self._perform_capture_safe(
                session,
                out_dir,
                quality,
                feedback_cb=None,
            )
        self._publish_capture_event(
            source="gpio",
            session_id=session,
            success=success,
            message=message,
            cam0_path=cam0_path,
            cam1_path=cam1_path,
            stamp=stamp,
        )
        if success:
            self.get_logger().info(
                f"GPIO capture OK stamp={_stamp_to_str(stamp)} cam0={cam0_path} cam1={cam1_path}"
            )
        else:
            self.get_logger().error(f"GPIO capture failed: {message}")

    def _message_kv(self, message: str) -> Dict[str, str]:
        out: Dict[str, str] = {}
        txt = str(message or "")
        if not txt:
            return out
        parts = [p.strip() for p in txt.split(";")]
        for part in parts:
            if "=" not in part:
                continue
            k, v = part.split("=", 1)
            key = k.strip().lower()
            if not key:
                continue
            out[key] = v.strip()
        return out

    def _publish_capture_event(
        self,
        source: str,
        session_id: str,
        success: bool,
        message: str,
        cam0_path: str,
        cam1_path: str,
        stamp: TimeMsg,
    ) -> None:
        payload = {
            "source": source,
            "session_id": session_id or "",
            "success": bool(success),
            "message": message or "",
            "cam0_path": cam0_path or "",
            "cam1_path": cam1_path or "",
            "stamp_sec": int(stamp.sec),
            "stamp_nanosec": int(stamp.nanosec),
        }
        kv = self._message_kv(message)
        if "metadata" in kv:
            payload["meta_path"] = kv["metadata"]
        if "cam0_deblur" in kv:
            payload["cam0_deblur_path"] = kv["cam0_deblur"]
        if "cam1_deblur" in kv:
            payload["cam1_deblur_path"] = kv["cam1_deblur"]
        if "trajectory_csv" in kv:
            payload["trajectory_csv_path"] = kv["trajectory_csv"]

        meta_path = str(payload.get("meta_path", "")).strip()
        if meta_path and os.path.isfile(meta_path):
            try:
                with open(meta_path, "r", encoding="utf-8") as f:
                    meta = json.load(f)
                cams = meta.get("cameras", {})
                if "cam0_deblur_path" not in payload:
                    d0 = (((cams.get("cam0") or {}).get("deblur") or {}).get("path") or "").strip()
                    if d0:
                        payload["cam0_deblur_path"] = d0
                if "cam1_deblur_path" not in payload:
                    d1 = (((cams.get("cam1") or {}).get("deblur") or {}).get("path") or "").strip()
                    if d1:
                        payload["cam1_deblur_path"] = d1
                if "trajectory_csv_path" not in payload:
                    tcsv = str(meta.get("trajectory_csv", "") or "").strip()
                    if tcsv:
                        payload["trajectory_csv_path"] = tcsv
            except Exception:
                pass
        try:
            msg = String()
            msg.data = json.dumps(payload, separators=(",", ":"))
            self._capture_evt_pub.publish(msg)
        except Exception as e:
            self.get_logger().warn(f"Failed to publish capture event: {e}")

    def _publish_capture_debug(self, payload: Dict[str, Any]) -> None:
        try:
            msg = String()
            msg.data = json.dumps(payload, separators=(",", ":"))
            self._capture_dbg_pub.publish(msg)
        except Exception as e:
            self.get_logger().warn(f"Failed to publish capture debug: {e}")

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

    def _trim_sensor_buffers_locked(self, force: bool = False) -> None:
        now_m = time.monotonic()
        if (not force) and (now_m < self._next_sensor_trim_mono):
            return
        self._next_sensor_trim_mono = now_m + self._sensor_trim_period_s
        cutoff_ns = _stamp_to_ns(now_ros_time(self)) - int(self._sensor_keep_s * 1_000_000_000)

        def _keep_recent(buf: Deque[Any]) -> Deque[Any]:
            if not buf:
                return buf
            kept = [m for m in buf if _stamp_to_ns(m.header.stamp) >= cutoff_ns]
            if len(kept) == len(buf):
                return buf
            return deque(kept, maxlen=buf.maxlen)

        # Filter entire deques, not only the head, so out-of-order timestamps
        # cannot keep stale samples indefinitely.
        self._buf_fix = _keep_recent(self._buf_fix)
        self._buf_time_ref = _keep_recent(self._buf_time_ref)
        self._buf_imu = _keep_recent(self._buf_imu)
        self._buf_odom_local = _keep_recent(self._buf_odom_local)
        self._buf_odom_global = _keep_recent(self._buf_odom_global)

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

    def _on_odom_local(self, msg: Odometry) -> None:
        with self._sensor_lock:
            self._buf_odom_local.append(msg)
            self._trim_sensor_buffers_locked()

    def _on_odom_global(self, msg: Odometry) -> None:
        with self._sensor_lock:
            self._buf_odom_global.append(msg)
            self._trim_sensor_buffers_locked()

    def _msg_stamp_ns(self, msg: Image) -> Optional[int]:
        try:
            sec = int(msg.header.stamp.sec)
            nsec = int(msg.header.stamp.nanosec)
        except Exception:
            return None
        ns = sec * 1_000_000_000 + nsec
        if ns <= 0:
            return None
        return ns

    def _on_cam0_image(self, msg: Image) -> None:
        now_m = time.monotonic()
        stamp_ns = self._msg_stamp_ns(msg)
        with self._stream_lock:
            self._latest_cam0_msg = msg
            self._latest_cam0_rx_mono = now_m
            self._buf_cam0.append((msg, now_m, stamp_ns))

    def _on_cam1_image(self, msg: Image) -> None:
        now_m = time.monotonic()
        stamp_ns = self._msg_stamp_ns(msg)
        with self._stream_lock:
            self._latest_cam1_msg = msg
            self._latest_cam1_rx_mono = now_m
            self._buf_cam1.append((msg, now_m, stamp_ns))

    def _latest_stream_snapshot(self) -> Tuple[Optional[Image], Optional[Image], Optional[float], Optional[float]]:
        with self._stream_lock:
            return (
                self._latest_cam0_msg,
                self._latest_cam1_msg,
                self._latest_cam0_rx_mono,
                self._latest_cam1_rx_mono,
            )

    def _stream_buffers_snapshot(self) -> Tuple[List[StreamFrame], List[StreamFrame]]:
        with self._stream_lock:
            return list(self._buf_cam0), list(self._buf_cam1)

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

    def _write_jpeg_bgr(self, path: str, frame, quality: int) -> Tuple[bool, str]:
        try:
            ok, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)])
            if not ok:
                return False, "cv2.imencode failed"
            with open(path, "wb") as f:
                f.write(buf.tobytes())
            return True, ""
        except Exception as e:
            return False, str(e)

    def _nearest_from_msgs(
        self,
        msgs: List[Any],
        stamp_ns: int,
        stamp_getter: Callable[[Any], int],
    ) -> Optional[Any]:
        if not msgs:
            return None
        return min(msgs, key=lambda m: abs(stamp_getter(m) - stamp_ns))

    def _odom_interpolated_state(
        self,
        o0: Odometry,
        o1: Odometry,
        target_ns: int,
        alpha: float,
    ) -> Dict[str, Any]:
        alpha = _clamp(alpha, 0.0, 1.0)
        p0 = o0.pose.pose.position
        p1 = o1.pose.pose.position
        q0 = [
            float(o0.pose.pose.orientation.x),
            float(o0.pose.pose.orientation.y),
            float(o0.pose.pose.orientation.z),
            float(o0.pose.pose.orientation.w),
        ]
        q1 = [
            float(o1.pose.pose.orientation.x),
            float(o1.pose.pose.orientation.y),
            float(o1.pose.pose.orientation.z),
            float(o1.pose.pose.orientation.w),
        ]
        v0 = o0.twist.twist.linear
        v1 = o1.twist.twist.linear
        w0 = o0.twist.twist.angular
        w1 = o1.twist.twist.angular
        q = _quat_slerp(q0, q1, alpha)
        pos = [
            float(p0.x + (p1.x - p0.x) * alpha),
            float(p0.y + (p1.y - p0.y) * alpha),
            float(p0.z + (p1.z - p0.z) * alpha),
        ]
        lin = [
            float(v0.x + (v1.x - v0.x) * alpha),
            float(v0.y + (v1.y - v0.y) * alpha),
            float(v0.z + (v1.z - v0.z) * alpha),
        ]
        ang = [
            float(w0.x + (w1.x - w0.x) * alpha),
            float(w0.y + (w1.y - w0.y) * alpha),
            float(w0.z + (w1.z - w0.z) * alpha),
        ]
        t0_ns = _stamp_to_ns(o0.header.stamp)
        t1_ns = _stamp_to_ns(o1.header.stamp)
        return {
            "stamp": _ns_to_stamp_str(target_ns),
            "source_stamp0": _stamp_to_str(o0.header.stamp),
            "source_stamp1": _stamp_to_str(o1.header.stamp),
            "source_dt_ms": (t1_ns - t0_ns) / 1_000_000.0,
            "interpolated": bool(t0_ns != t1_ns and o0 is not o1),
            "frame_id": o0.header.frame_id or "",
            "child_frame_id": o0.child_frame_id or "",
            "position": pos,
            "orientation_xyzw": [float(q[0]), float(q[1]), float(q[2]), float(q[3])],
            "linear_velocity": lin,
            "angular_velocity": ang,
            "speed_mps": math.sqrt(lin[0] * lin[0] + lin[1] * lin[1] + lin[2] * lin[2]),
        }

    def _interp_odometry_at(self, msgs: List[Odometry], stamp_ns: int) -> Optional[Dict[str, Any]]:
        if not msgs:
            return None
        if len(msgs) == 1:
            o = msgs[0]
            out = self._odom_interpolated_state(o, o, stamp_ns, 0.0)
            out["nearest_dt_ms"] = (_stamp_to_ns(o.header.stamp) - stamp_ns) / 1_000_000.0
            return out

        first = msgs[0]
        last = msgs[-1]
        first_ns = _stamp_to_ns(first.header.stamp)
        last_ns = _stamp_to_ns(last.header.stamp)
        if stamp_ns <= first_ns:
            out = self._odom_interpolated_state(first, first, stamp_ns, 0.0)
            out["nearest_dt_ms"] = (first_ns - stamp_ns) / 1_000_000.0
            return out
        if stamp_ns >= last_ns:
            out = self._odom_interpolated_state(last, last, stamp_ns, 0.0)
            out["nearest_dt_ms"] = (last_ns - stamp_ns) / 1_000_000.0
            return out

        for i in range(1, len(msgs)):
            o0 = msgs[i - 1]
            o1 = msgs[i]
            t0 = _stamp_to_ns(o0.header.stamp)
            t1 = _stamp_to_ns(o1.header.stamp)
            if stamp_ns < t0 or stamp_ns > t1:
                continue
            if t1 <= t0:
                out = self._odom_interpolated_state(o0, o0, stamp_ns, 0.0)
                out["nearest_dt_ms"] = (t0 - stamp_ns) / 1_000_000.0
                return out
            alpha = (stamp_ns - t0) / float(t1 - t0)
            out = self._odom_interpolated_state(o0, o1, stamp_ns, alpha)
            out["nearest_dt_ms"] = min(abs(stamp_ns - t0), abs(t1 - stamp_ns)) / 1_000_000.0
            return out
        return None

    def _trajectory_bundle(self, center_ns: int) -> Dict[str, Any]:
        rate_hz = max(1.0, _safe_float(self.get_parameter("trajectory_sample_rate_hz").value, 100.0))
        window_ms = max(0.0, _safe_float(self.get_parameter("trajectory_window_ms").value, 1000.0))
        if window_ms <= 0.0:
            return {
                "enabled": False,
                "reason": "trajectory_window_ms <= 0",
                "sample_rate_hz": rate_hz,
                "window_ms": window_ms,
                "samples": [],
            }

        with self._sensor_lock:
            odom_global_msgs = list(self._buf_odom_global)
            odom_local_msgs = list(self._buf_odom_local)
            imu_msgs = list(self._buf_imu)
        odom_global_msgs.sort(key=lambda m: _stamp_to_ns(m.header.stamp))
        odom_local_msgs.sort(key=lambda m: _stamp_to_ns(m.header.stamp))
        imu_msgs.sort(key=lambda m: _stamp_to_ns(m.header.stamp))

        source_name = ""
        src_msgs: List[Odometry] = []
        if len(odom_global_msgs) >= 2:
            source_name = "odom_global"
            src_msgs = odom_global_msgs
        elif len(odom_local_msgs) >= 2:
            source_name = "odom_local"
            src_msgs = odom_local_msgs
        elif odom_global_msgs:
            source_name = "odom_global_single"
            src_msgs = odom_global_msgs
        elif odom_local_msgs:
            source_name = "odom_local_single"
            src_msgs = odom_local_msgs
        else:
            return {
                "enabled": False,
                "reason": "no odometry samples",
                "sample_rate_hz": rate_hz,
                "window_ms": window_ms,
                "samples": [],
            }

        half_ns = int(window_ms * 500_000.0)
        start_ns = center_ns - half_ns
        end_ns = center_ns + half_ns
        if end_ns < start_ns:
            end_ns = start_ns
        step_ns = max(1, int(round(1_000_000_000.0 / rate_hz)))
        span_ns = max(0, end_ns - start_ns)
        max_samples = 600
        est_samples = int(span_ns // step_ns) + 1
        if est_samples > max_samples:
            step_ns = max(1, int(math.ceil(span_ns / float(max_samples - 1))))

        samples: List[Dict[str, Any]] = []
        ns = start_ns
        while ns <= end_ns:
            pose = self._interp_odometry_at(src_msgs, ns)
            if pose is not None:
                sample: Dict[str, Any] = {
                    "stamp": _ns_to_stamp_str(ns),
                    "offset_from_center_ms": (ns - center_ns) / 1_000_000.0,
                    "position": pose["position"],
                    "orientation_xyzw": pose["orientation_xyzw"],
                    "linear_velocity": pose["linear_velocity"],
                    "angular_velocity": pose["angular_velocity"],
                    "speed_mps": pose["speed_mps"],
                    "nearest_odom_dt_ms": pose.get("nearest_dt_ms"),
                }
                imu = self._nearest_from_msgs(
                    imu_msgs,
                    ns,
                    lambda m: _stamp_to_ns(m.header.stamp),
                )
                if imu is not None:
                    imu_ns = _stamp_to_ns(imu.header.stamp)
                    sample["imu_dt_ms"] = (imu_ns - ns) / 1_000_000.0
                    sample["imu_angular_velocity"] = [
                        float(imu.angular_velocity.x),
                        float(imu.angular_velocity.y),
                        float(imu.angular_velocity.z),
                    ]
                    sample["imu_linear_acceleration"] = [
                        float(imu.linear_acceleration.x),
                        float(imu.linear_acceleration.y),
                        float(imu.linear_acceleration.z),
                    ]
                samples.append(sample)
            ns += step_ns
        if samples and samples[-1]["stamp"] != _ns_to_stamp_str(end_ns):
            pose = self._interp_odometry_at(src_msgs, end_ns)
            if pose is not None:
                samples.append(
                    {
                        "stamp": _ns_to_stamp_str(end_ns),
                        "offset_from_center_ms": (end_ns - center_ns) / 1_000_000.0,
                        "position": pose["position"],
                        "orientation_xyzw": pose["orientation_xyzw"],
                        "linear_velocity": pose["linear_velocity"],
                        "angular_velocity": pose["angular_velocity"],
                        "speed_mps": pose["speed_mps"],
                        "nearest_odom_dt_ms": pose.get("nearest_dt_ms"),
                    }
                )
        return {
            "enabled": True,
            "source": source_name,
            "sample_rate_hz": float(rate_hz),
            "window_ms": float(window_ms),
            "step_ms": float(step_ns) / 1_000_000.0,
            "samples": samples,
        }

    def _write_trajectory_csv(self, out_dir: str, session: str, traj: Dict[str, Any]) -> str:
        samples = traj.get("samples", [])
        if not isinstance(samples, list) or not samples:
            return ""
        csv_path = os.path.join(out_dir, f"{session}_trajectory.csv")
        tmp = csv_path + ".tmp"
        with open(tmp, "w", newline="", encoding="utf-8") as f:
            wr = csv.writer(f)
            wr.writerow(
                [
                    "stamp",
                    "offset_from_center_ms",
                    "px",
                    "py",
                    "pz",
                    "qx",
                    "qy",
                    "qz",
                    "qw",
                    "vx",
                    "vy",
                    "vz",
                    "speed_mps",
                    "wx",
                    "wy",
                    "wz",
                    "imu_ax",
                    "imu_ay",
                    "imu_az",
                    "imu_wx",
                    "imu_wy",
                    "imu_wz",
                    "imu_dt_ms",
                    "nearest_odom_dt_ms",
                ]
            )
            for s in samples:
                pos = s.get("position", [None, None, None])
                ori = s.get("orientation_xyzw", [None, None, None, None])
                lv = s.get("linear_velocity", [None, None, None])
                av = s.get("angular_velocity", [None, None, None])
                imu_a = s.get("imu_linear_acceleration", [None, None, None])
                imu_w = s.get("imu_angular_velocity", [None, None, None])
                wr.writerow(
                    [
                        s.get("stamp"),
                        s.get("offset_from_center_ms"),
                        pos[0],
                        pos[1],
                        pos[2],
                        ori[0],
                        ori[1],
                        ori[2],
                        ori[3],
                        lv[0],
                        lv[1],
                        lv[2],
                        s.get("speed_mps"),
                        av[0],
                        av[1],
                        av[2],
                        imu_a[0],
                        imu_a[1],
                        imu_a[2],
                        imu_w[0],
                        imu_w[1],
                        imu_w[2],
                        s.get("imu_dt_ms"),
                        s.get("nearest_odom_dt_ms"),
                    ]
                )
        os.replace(tmp, csv_path)
        return csv_path

    def _deblur_exposure_s(self) -> Tuple[float, int]:
        exposure_us = int(self.get_parameter("deblur_exposure_time_us").value)
        if exposure_us <= 0:
            exposure_ms = max(0.1, _safe_float(self.get_parameter("deblur_exposure_ms").value, 8.0))
            exposure_us = int(round(exposure_ms * 1000.0))
        exposure_us = max(100, exposure_us)
        return float(exposure_us) / 1_000_000.0, exposure_us

    def _deblur_stamp_reference(self) -> str:
        ref = str(self.get_parameter("deblur_image_stamp_reference").value).strip().lower()
        if ref not in ("start", "midpoint", "end"):
            return "midpoint"
        return ref

    def _deblur_exposure_window_ns(self, stamp_ns: int, exposure_us: int, stamp_ref: str) -> Tuple[int, int]:
        exposure_ns = max(1, int(exposure_us) * 1000)
        if stamp_ref == "start":
            t0_ns = int(stamp_ns)
            t1_ns = int(stamp_ns + exposure_ns)
        elif stamp_ref == "end":
            t0_ns = int(stamp_ns - exposure_ns)
            t1_ns = int(stamp_ns)
        else:
            half_ns = exposure_ns // 2
            t0_ns = int(stamp_ns - half_ns)
            t1_ns = int(t0_ns + exposure_ns)
        if t1_ns <= t0_ns:
            t1_ns = t0_ns + 1
        return t0_ns, t1_ns

    def _gyro_samples_snapshot(self) -> List[GyroSample]:
        with self._sensor_lock:
            msgs = list(self._buf_imu)
        if not msgs:
            return []
        samples: List[GyroSample] = []
        for msg in msgs:
            t_ns = _stamp_to_ns(msg.header.stamp)
            if t_ns <= 0:
                continue
            samples.append(
                (
                    int(t_ns),
                    (
                        float(msg.angular_velocity.x),
                        float(msg.angular_velocity.y),
                        float(msg.angular_velocity.z),
                    ),
                )
            )
        samples.sort(key=lambda x: x[0])
        # Collapse duplicate timestamps (keep latest sample for each stamp).
        dedup: List[GyroSample] = []
        for t_ns, w in samples:
            if dedup and dedup[-1][0] == t_ns:
                dedup[-1] = (t_ns, w)
            else:
                dedup.append((t_ns, w))
        return dedup

    def _interp_gyro_from_sorted(
        self,
        samples: List[GyroSample],
        times: List[int],
        target_ns: int,
    ) -> Tuple[Optional[Tuple[float, float, float]], Dict[str, Any]]:
        idx = bisect_left(times, target_ns)
        if idx < len(times) and times[idx] == target_ns:
            return samples[idx][1], {
                "mode": "exact",
                "source_stamp": _ns_to_stamp_str(times[idx]),
            }
        left = idx - 1
        right = idx
        if left < 0 or right >= len(times):
            return None, {"mode": "out_of_range"}
        t0_ns, w0 = samples[left]
        t1_ns, w1 = samples[right]
        if t1_ns <= t0_ns:
            return None, {"mode": "degenerate"}
        alpha = float(target_ns - t0_ns) / float(t1_ns - t0_ns)
        return _lerp_vec3(w0, w1, alpha), {
            "mode": "linear",
            "left_stamp": _ns_to_stamp_str(t0_ns),
            "right_stamp": _ns_to_stamp_str(t1_ns),
            "alpha": float(alpha),
        }

    def _integrate_gyro_interval(
        self,
        t0_ns: int,
        t1_ns: int,
        gyro_samples: Optional[List[GyroSample]] = None,
    ) -> Dict[str, Any]:
        out: Dict[str, Any] = {
            "ok": False,
            "status": "missing_imu",
            "gyro_samples_used": 0,
        }
        if t1_ns <= t0_ns:
            out["status"] = "invalid_exposure_interval"
            return out

        samples = gyro_samples if gyro_samples is not None else self._gyro_samples_snapshot()
        out["gyro_buffer_samples"] = int(len(samples))
        if len(samples) < 2:
            out["status"] = "not_enough_imu_samples"
            return out

        times = [s[0] for s in samples]
        out["gyro_buffer_time_min"] = _ns_to_stamp_str(times[0])
        out["gyro_buffer_time_max"] = _ns_to_stamp_str(times[-1])
        if t0_ns < times[0] or t1_ns > times[-1]:
            out["status"] = "imu_window_outside_buffer"
            return out

        w0, interp0 = self._interp_gyro_from_sorted(samples, times, t0_ns)
        w1, interp1 = self._interp_gyro_from_sorted(samples, times, t1_ns)
        out["interp_start"] = interp0
        out["interp_end"] = interp1
        if w0 is None or w1 is None:
            out["status"] = "imu_boundary_interpolation_failed"
            return out

        inside_start = bisect_right(times, t0_ns)
        inside_end = bisect_left(times, t1_ns)
        integrated_samples: List[GyroSample] = [(t0_ns, w0)]
        for idx in range(inside_start, inside_end):
            integrated_samples.append(samples[idx])
        integrated_samples.append((t1_ns, w1))

        dedup: List[GyroSample] = []
        for t_ns, w in integrated_samples:
            if dedup and dedup[-1][0] == t_ns:
                dedup[-1] = (t_ns, w)
            else:
                dedup.append((t_ns, w))
        if len(dedup) < 2:
            out["status"] = "not_enough_interval_samples"
            return out

        dtheta_x = 0.0
        dtheta_y = 0.0
        dtheta_z = 0.0
        max_step_s = 0.0
        for i in range(1, len(dedup)):
            t_a, w_a = dedup[i - 1]
            t_b, w_b = dedup[i]
            dt_s = float(t_b - t_a) / 1_000_000_000.0
            if dt_s <= 0.0:
                continue
            if dt_s > max_step_s:
                max_step_s = dt_s
            dtheta_x += 0.5 * (w_a[0] + w_b[0]) * dt_s
            dtheta_y += 0.5 * (w_a[1] + w_b[1]) * dt_s
            dtheta_z += 0.5 * (w_a[2] + w_b[2]) * dt_s

        if max_step_s <= 0.0:
            out["status"] = "invalid_interval_dt"
            return out

        max_gap_limit_ms = max(
            0.0,
            _safe_float(self.get_parameter("deblur_max_integration_gap_ms").value, 25.0),
        )
        max_step_ms = float(max_step_s * 1000.0)
        exceeds_gap = bool(max_gap_limit_ms > 0.0 and max_step_ms > max_gap_limit_ms)
        if exceeds_gap:
            out.update(
                {
                    "ok": False,
                    "status": "integration_gap_exceeded",
                    "gyro_samples_used": int(len(dedup)),
                    "gyro_samples_inside_exposure": int(max(0, inside_end - inside_start)),
                    "gyro_time_min": _ns_to_stamp_str(dedup[0][0]),
                    "gyro_time_max": _ns_to_stamp_str(dedup[-1][0]),
                    "duration_ms": float((t1_ns - t0_ns) / 1_000_000.0),
                    "max_step_ms": max_step_ms,
                    "max_step_limit_ms": float(max_gap_limit_ms),
                    "max_step_exceeds_limit": True,
                    "delta_theta_rad": [float(dtheta_x), float(dtheta_y), float(dtheta_z)],
                }
            )
            return out
        out.update(
            {
                "ok": True,
                "status": "ok",
                "gyro_samples_used": int(len(dedup)),
                "gyro_samples_inside_exposure": int(max(0, inside_end - inside_start)),
                "gyro_time_min": _ns_to_stamp_str(dedup[0][0]),
                "gyro_time_max": _ns_to_stamp_str(dedup[-1][0]),
                "duration_ms": float((t1_ns - t0_ns) / 1_000_000.0),
                "max_step_ms": max_step_ms,
                "max_step_limit_ms": float(max_gap_limit_ms),
                "max_step_exceeds_limit": False,
                "delta_theta_rad": [float(dtheta_x), float(dtheta_y), float(dtheta_z)],
            }
        )
        return out

    def _estimate_blur_kernel(
        self,
        stamp_ns: int,
        width: int,
        gyro_samples: Optional[List[GyroSample]] = None,
    ) -> Tuple[float, float, Dict[str, Any]]:
        exposure_s, exposure_us = self._deblur_exposure_s()
        stamp_ref = self._deblur_stamp_reference()
        t0_ns, t1_ns = self._deblur_exposure_window_ns(stamp_ns, exposure_us, stamp_ref)
        fov_deg = _clamp(_safe_float(self.get_parameter("deblur_fov_deg").value, 72.0), 20.0, 179.0)
        strength = max(0.0, _safe_float(self.get_parameter("deblur_strength").value, 1.0))
        focal_px = float(width) / (2.0 * math.tan(math.radians(fov_deg) * 0.5))
        timestamp_source = str(self.get_parameter("deblur_timestamp_source").value).strip() or "unknown"
        requested_nearest_fallback = bool(self.get_parameter("deblur_allow_nearest_fallback").value)
        require_time_ref = bool(self.get_parameter("deblur_require_time_reference").value)
        max_time_ref_age_ms = max(
            0.0,
            _safe_float(self.get_parameter("deblur_max_time_reference_age_ms").value, 2000.0),
        )

        diag: Dict[str, Any] = {
            "status": "missing_imu",
            "timestamp_source": timestamp_source,
            "image_stamp": _ns_to_stamp_str(stamp_ns),
            "image_stamp_reference": stamp_ref,
            "exposure_time_us": int(exposure_us),
            "exposure_ms": float(exposure_s * 1000.0),
            "exposure_start": _ns_to_stamp_str(t0_ns),
            "exposure_end": _ns_to_stamp_str(t1_ns),
            "fov_deg": float(fov_deg),
            "focal_px": float(focal_px),
            "deblur_strength": float(strength),
            "integration_method": "trapezoidal",
            "nearest_imu_fallback_policy": "disabled",
            "require_time_reference": bool(require_time_ref),
            "max_time_reference_age_ms": float(max_time_ref_age_ms),
        }
        if requested_nearest_fallback:
            diag["nearest_imu_fallback_requested"] = True

        if require_time_ref:
            tr = self._nearest_time_ref(stamp_ns)
            if tr is None:
                diag["status"] = "missing_time_reference"
                return 0.0, 0.0, diag
            tr_ns = _stamp_to_ns(tr.header.stamp)
            tr_age_ms = abs(float(tr_ns - stamp_ns) / 1_000_000.0)
            diag["nearest_time_reference"] = {
                "stamp": _stamp_to_str(tr.header.stamp),
                "time_ref": _stamp_to_str(tr.time_ref),
                "source": tr.source or "",
                "dt_ms": float((tr_ns - stamp_ns) / 1_000_000.0),
                "abs_dt_ms": tr_age_ms,
            }
            if max_time_ref_age_ms > 0.0 and tr_age_ms > max_time_ref_age_ms:
                diag["status"] = "stale_time_reference"
                return 0.0, 0.0, diag

        integration = self._integrate_gyro_interval(t0_ns, t1_ns, gyro_samples)
        dtheta = integration.get("delta_theta_rad", [0.0, 0.0, 0.0])
        if bool(integration.get("ok")):
            diag["status"] = "ok"
            diag["estimation_mode"] = "gyro_interval_integration"
            diag.update({k: v for k, v in integration.items() if k != "ok"})
        else:
            diag["integration_error"] = integration.get("status", "unknown")
            diag["gyro_samples_used"] = int(integration.get("gyro_samples_used", 0))
            diag["gyro_buffer_samples"] = int(integration.get("gyro_buffer_samples", 0))
            diag["interp_start"] = integration.get("interp_start")
            diag["interp_end"] = integration.get("interp_end")
            diag["status"] = "missing_imu_interval"
            return 0.0, 0.0, diag

        dtheta_x = float(dtheta[0]) if len(dtheta) >= 1 else 0.0
        dtheta_y = float(dtheta[1]) if len(dtheta) >= 2 else 0.0
        dtheta_z = float(dtheta[2]) if len(dtheta) >= 3 else 0.0

        # Rotational blur: map integrated angular velocity to pixel displacement.
        dx = float(dtheta_y * focal_px * strength)
        dy = float(dtheta_x * focal_px * strength)

        # Rotate blur vector from IMU frame to camera image frame.
        # Set deblur_imu_to_cam_yaw_deg to the angle (degrees CCW) between the IMU
        # X-axis and the camera +U (right) axis when viewed from behind the camera.
        imu_to_cam_yaw_deg = _safe_float(
            self.get_parameter("deblur_imu_to_cam_yaw_deg").value, 0.0
        )
        cy = math.cos(math.radians(imu_to_cam_yaw_deg))
        sy = math.sin(math.radians(imu_to_cam_yaw_deg))
        if abs(imu_to_cam_yaw_deg) > 0.01:
            dx, dy = dx * cy - dy * sy, dx * sy + dy * cy

        # Translational blur from odometry velocity (depth-dependent).
        use_translation = bool(self.get_parameter("deblur_use_translation").value)
        trans_diag: Dict[str, Any] = {"enabled": use_translation}
        if use_translation:
            odom = self._nearest_odom_local(stamp_ns)
            assumed_depth_m = max(
                0.05, _safe_float(self.get_parameter("deblur_assumed_depth_m").value, 1.5)
            )
            max_odom_age_ms = max(
                0.0, _safe_float(self.get_parameter("deblur_max_odom_age_ms").value, 200.0)
            )
            trans_diag["assumed_depth_m"] = float(assumed_depth_m)
            if odom is not None:
                odom_age_ms = abs(
                    float(_stamp_to_ns(odom.header.stamp) - stamp_ns) / 1_000_000.0
                )
                trans_diag["odom_age_ms"] = float(odom_age_ms)
                if max_odom_age_ms <= 0.0 or odom_age_ms <= max_odom_age_ms:
                    vx = float(odom.twist.twist.linear.x)
                    vy = float(odom.twist.twist.linear.y)
                    # Apply the same yaw rotation to bring velocity into camera frame.
                    vx_cam = vx * cy - vy * sy
                    vy_cam = vx * sy + vy * cy
                    trans_dx = (vx_cam / assumed_depth_m) * focal_px * exposure_s * strength
                    trans_dy = (vy_cam / assumed_depth_m) * focal_px * exposure_s * strength
                    dx += trans_dx
                    dy += trans_dy
                    trans_diag.update({
                        "vx_body": float(vx), "vy_body": float(vy),
                        "vx_cam": float(vx_cam), "vy_cam": float(vy_cam),
                        "trans_dx_px": float(trans_dx), "trans_dy_px": float(trans_dy),
                        "status": "ok",
                    })
                else:
                    trans_diag["status"] = "odom_too_old"
            else:
                trans_diag["status"] = "no_odom"

        length_raw = math.sqrt(dx * dx + dy * dy)
        angle_rad = float(math.atan2(dy, dx)) if length_raw > 1e-9 else 0.0
        angle_deg = float(math.degrees(angle_rad))
        max_kernel = max(3, int(self.get_parameter("deblur_max_kernel_px").value))
        length_px = _clamp(length_raw, 0.0, float(max_kernel))

        diag.update(
            {
                "delta_theta_rad": [dtheta_x, dtheta_y, dtheta_z],
                "imu_to_cam_yaw_deg": float(imu_to_cam_yaw_deg),
                "translation": trans_diag,
                "blur_dx_px": float(dx),
                "blur_dy_px": float(dy),
                "blur_length_px": float(length_raw),
                "blur_angle_rad": angle_rad,
                "blur_angle_deg": angle_deg,
                "blur_length_clamped_px": float(length_px),
            }
        )
        return float(length_px), angle_deg, diag

    def _deblur_capture_image(
        self,
        img_path: str,
        stamp: Optional[TimeMsg],
        quality: int,
        gyro_samples: Optional[List[GyroSample]] = None,
    ) -> Dict[str, Any]:
        info: Dict[str, Any] = {
            "enabled": bool(self.get_parameter("enable_motion_deblur").value),
            "path": "",
            "status": "disabled",
        }
        if not info["enabled"]:
            return info
        if not img_path:
            info["status"] = "missing_input_path"
            return info
        if not os.path.exists(img_path):
            info["status"] = "missing_input_file"
            return info
        stem, ext = os.path.splitext(img_path)
        ext = ext or ".jpg"
        out_path = f"{stem}_deblur{ext}"
        info["path"] = out_path

        if stamp is None:
            try:
                shutil.copy2(img_path, out_path)
                info["status"] = "copied_no_stamp"
            except Exception as e:
                info["status"] = f"copy_failed: {e}"
            return info

        img = cv2.imread(img_path, cv2.IMREAD_COLOR)
        if img is None:
            info["status"] = "decode_failed"
            return info

        stamp_ns = _stamp_to_ns(stamp)
        length_px, angle_deg, diag = self._estimate_blur_kernel(
            stamp_ns,
            int(img.shape[1]),
            gyro_samples,
        )
        info.update(diag)
        info["kernel_length_px"] = float(length_px)
        info["kernel_angle_deg"] = float(angle_deg)
        info["kernel_angle_rad"] = float(math.radians(angle_deg))
        info["psf_length_px"] = float(length_px)
        info["psf_angle_deg"] = float(angle_deg)
        info["rl_iterations"] = int(max(1, int(self.get_parameter("deblur_iterations").value)))
        estimator_status = str(diag.get("status", "")).strip().lower()
        if estimator_status != "ok":
            try:
                shutil.copy2(img_path, out_path)
                status_tag = re.sub(r"[^a-z0-9_]+", "_", estimator_status) or "unavailable"
                info["status"] = f"copied_{status_tag}"
            except Exception as e:
                info["status"] = f"copy_failed: {e}"
            return info
        min_kernel = max(0.0, _safe_float(self.get_parameter("deblur_min_kernel_px").value, 1.2))
        if length_px < min_kernel:
            try:
                shutil.copy2(img_path, out_path)
                info["status"] = "copied_low_motion"
            except Exception as e:
                info["status"] = f"copy_failed: {e}"
            return info

        max_kernel = max(3, int(self.get_parameter("deblur_max_kernel_px").value))
        iterations = int(info["rl_iterations"])
        method = str(self.get_parameter("deblur_method").value).strip().lower()
        psf = _line_psf(length_px, angle_deg, max_kernel=max_kernel)
        try:
            if method == "wiener":
                snr = max(1.0, _safe_float(self.get_parameter("deblur_wiener_snr").value, 40.0))
                deblurred = _wiener_deconvolve_bgr(img, psf, snr=snr)
                info["wiener_snr"] = float(snr)
                method_tag = "wiener_fft_line_psf"
            else:
                deblurred = _richardson_lucy_bgr(img, psf, iterations=iterations)
                method_tag = "richardson_lucy_line_psf"
            ok, err = self._write_jpeg_bgr(out_path, deblurred, quality)
            if not ok:
                info["status"] = f"write_failed: {err}"
                return info
            info["status"] = "deblurred"
            info["method"] = method_tag
            info["iterations"] = int(iterations)
            return info
        except Exception as e:
            info["status"] = f"deblur_failed: {e}"
            return info

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

    def _nearest_odom_local(self, stamp_ns: int) -> Optional[Odometry]:
        with self._sensor_lock:
            msgs = list(self._buf_odom_local)
        if not msgs:
            return None
        return min(msgs, key=lambda m: abs(_stamp_to_ns(m.header.stamp) - stamp_ns))

    def _nearest_odom_global(self, stamp_ns: int) -> Optional[Odometry]:
        with self._sensor_lock:
            msgs = list(self._buf_odom_global)
        if not msgs:
            return None
        return min(msgs, key=lambda m: abs(_stamp_to_ns(m.header.stamp) - stamp_ns))

    def _pick_stream_frame(
        self,
        frames: List[StreamFrame],
        target_ns: int,
        max_age_s: float,
        now_m: float,
    ) -> Optional[StreamFrame]:
        best: Optional[StreamFrame] = None
        best_score = float("inf")
        for frame in frames:
            _msg, rx_mono, stamp_ns = frame
            age_s = now_m - rx_mono
            if age_s > max_age_s:
                continue
            if stamp_ns is not None:
                score = abs(float(stamp_ns - target_ns))
            else:
                # Keep unstamped frames as a fallback if a stream provides invalid header stamps.
                score = 1e18 + age_s * 1e9
            if score < best_score:
                best = frame
                best_score = score
        return best

    def _fresh_stream_frames(
        self,
        frames: List[StreamFrame],
        max_age_s: float,
        now_m: float,
    ) -> List[StreamFrame]:
        fresh: List[StreamFrame] = []
        for frame in frames:
            _msg, rx_mono, _stamp_ns = frame
            if (now_m - rx_mono) <= max_age_s:
                fresh.append(frame)
        return fresh

    def _pick_stream_frame_pair(
        self,
        frames0: List[StreamFrame],
        frames1: List[StreamFrame],
        target_ns: int,
        max_age_s: float,
        now_m: float,
        pair_slop_ns: int,
    ) -> Tuple[Optional[StreamFrame], Optional[StreamFrame], Optional[float], bool]:
        """Select the best cam0/cam1 pair near target_ns.

        Returns (sel0, sel1, pair_delta_ms, pair_ok). pair_ok is True only when
        a pair satisfies the pair_slop_ns constraint (or stamps are unavailable).
        """
        fresh0 = self._fresh_stream_frames(frames0, max_age_s, now_m)
        fresh1 = self._fresh_stream_frames(frames1, max_age_s, now_m)
        if not fresh0 or not fresh1:
            return None, None, None, False

        best_ok_pair: Optional[Tuple[StreamFrame, StreamFrame, int]] = None
        best_ok_key: Optional[Tuple[float, int, float]] = None
        best_any_pair: Optional[Tuple[StreamFrame, StreamFrame, int]] = None
        best_any_key: Optional[Tuple[float, int, float]] = None

        for f0 in fresh0:
            st0 = f0[2]
            if st0 is None:
                continue
            age0 = max(0.0, now_m - f0[1])
            for f1 in fresh1:
                st1 = f1[2]
                if st1 is None:
                    continue
                age1 = max(0.0, now_m - f1[1])
                delta_ns = abs(st0 - st1)
                trigger_err_ns = max(abs(st0 - target_ns), abs(st1 - target_ns))
                # Prioritize trigger proximity, then tighter pair sync, then fresher pair.
                key = (float(trigger_err_ns), int(delta_ns), float(age0 + age1))

                if best_any_key is None or key < best_any_key:
                    best_any_key = key
                    best_any_pair = (f0, f1, delta_ns)
                if delta_ns <= pair_slop_ns and (best_ok_key is None or key < best_ok_key):
                    best_ok_key = key
                    best_ok_pair = (f0, f1, delta_ns)

        if best_ok_pair is not None:
            f0, f1, delta_ns = best_ok_pair
            return f0, f1, float(delta_ns) / 1_000_000.0, True

        if best_any_pair is not None:
            f0, f1, delta_ns = best_any_pair
            return f0, f1, float(delta_ns) / 1_000_000.0, False

        # Fallback for streams without valid stamps: pick fresh frames by receive time/age.
        sel0 = self._pick_stream_frame(fresh0, target_ns, max_age_s, now_m)
        if sel0 is None:
            return None, None, None, False
        target1_ns = sel0[2] if sel0[2] is not None else target_ns
        sel1 = self._pick_stream_frame(fresh1, target1_ns, max_age_s, now_m)
        if sel1 is None:
            return sel0, None, None, False

        pair_ok = True
        pair_delta_ms: Optional[float] = None
        if sel0[2] is not None and sel1[2] is not None:
            pair_delta_ns = abs(sel0[2] - sel1[2])
            pair_delta_ms = float(pair_delta_ns) / 1_000_000.0
            pair_ok = pair_delta_ns <= pair_slop_ns
        return sel0, sel1, pair_delta_ms, pair_ok

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
        cam0_deblur: Optional[Dict[str, Any]] = None,
        cam1_deblur: Optional[Dict[str, Any]] = None,
    ) -> Dict[str, Any]:
        trigger_ns = _stamp_to_ns(trigger_stamp)
        metadata: Dict[str, Any] = {
            "schema_version": 2,
            "mode": mode,
            "session_id": session,
            "trigger_stamp": _stamp_to_str(trigger_stamp),
            "time_base": {
                "image_timestamp_source": str(self.get_parameter("deblur_timestamp_source").value).strip()
                or "unknown",
                "image_stamp_reference": self._deblur_stamp_reference(),
            },
            "cameras": {},
        }
        with self._sensor_lock:
            odom_local_msgs = list(self._buf_odom_local)
            odom_global_msgs = list(self._buf_odom_global)
        odom_local_msgs.sort(key=lambda m: _stamp_to_ns(m.header.stamp))
        odom_global_msgs.sort(key=lambda m: _stamp_to_ns(m.header.stamp))
        metadata["trajectory"] = self._trajectory_bundle(trigger_ns)

        deblur_by_cam: Dict[str, Optional[Dict[str, Any]]] = {
            "cam0": cam0_deblur,
            "cam1": cam1_deblur,
        }

        def add_camera(name: str, path: str, stamp: Optional[TimeMsg]) -> None:
            if not path:
                return
            exposure_us = self._deblur_exposure_s()[1]
            info: Dict[str, Any] = {
                "path": path,
                "stamp": _stamp_to_str(stamp) if stamp is not None else None,
                "timestamp_source": str(self.get_parameter("deblur_timestamp_source").value).strip() or "unknown",
                "exposure_time_us": int(exposure_us),
                "stamp_reference": self._deblur_stamp_reference(),
            }
            deblur_info = deblur_by_cam.get(name)
            if deblur_info is not None:
                info["deblur"] = deblur_info
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

                interp_local = self._interp_odometry_at(odom_local_msgs, s_ns)
                if interp_local is not None:
                    info["interp_odom_local"] = interp_local

                interp_global = self._interp_odometry_at(odom_global_msgs, s_ns)
                if interp_global is not None:
                    info["interp_odom_global"] = interp_global

                odom_local = self._nearest_odom_local(s_ns)
                if odom_local is not None:
                    odom_ns = _stamp_to_ns(odom_local.header.stamp)
                    info["nearest_odom_local"] = {
                        "stamp": _stamp_to_str(odom_local.header.stamp),
                        "dt_ms": (odom_ns - s_ns) / 1_000_000.0,
                        "frame_id": odom_local.header.frame_id or "",
                        "child_frame_id": odom_local.child_frame_id or "",
                        "position": [
                            float(odom_local.pose.pose.position.x),
                            float(odom_local.pose.pose.position.y),
                            float(odom_local.pose.pose.position.z),
                        ],
                        "orientation_xyzw": [
                            float(odom_local.pose.pose.orientation.x),
                            float(odom_local.pose.pose.orientation.y),
                            float(odom_local.pose.pose.orientation.z),
                            float(odom_local.pose.pose.orientation.w),
                        ],
                        "linear_velocity": [
                            float(odom_local.twist.twist.linear.x),
                            float(odom_local.twist.twist.linear.y),
                            float(odom_local.twist.twist.linear.z),
                        ],
                        "angular_velocity": [
                            float(odom_local.twist.twist.angular.x),
                            float(odom_local.twist.twist.angular.y),
                            float(odom_local.twist.twist.angular.z),
                        ],
                    }

                odom_global = self._nearest_odom_global(s_ns)
                if odom_global is not None:
                    odom_ns = _stamp_to_ns(odom_global.header.stamp)
                    info["nearest_odom_global"] = {
                        "stamp": _stamp_to_str(odom_global.header.stamp),
                        "dt_ms": (odom_ns - s_ns) / 1_000_000.0,
                        "frame_id": odom_global.header.frame_id or "",
                        "child_frame_id": odom_global.child_frame_id or "",
                        "position": [
                            float(odom_global.pose.pose.position.x),
                            float(odom_global.pose.pose.position.y),
                            float(odom_global.pose.pose.position.z),
                        ],
                        "orientation_xyzw": [
                            float(odom_global.pose.pose.orientation.x),
                            float(odom_global.pose.pose.orientation.y),
                            float(odom_global.pose.pose.orientation.z),
                            float(odom_global.pose.pose.orientation.w),
                        ],
                        "linear_velocity": [
                            float(odom_global.twist.twist.linear.x),
                            float(odom_global.twist.twist.linear.y),
                            float(odom_global.twist.twist.linear.z),
                        ],
                        "angular_velocity": [
                            float(odom_global.twist.twist.angular.x),
                            float(odom_global.twist.twist.angular.y),
                            float(odom_global.twist.twist.angular.z),
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
        relay_enabled = bool(self.get_parameter("preview_relay_enable").value)
        if relay_enabled:
            w = int(self.get_parameter("preview_relay_width").value)
            h = int(self.get_parameter("preview_relay_height").value)
            fps = max(1, int(self.get_parameter("preview_relay_fps").value))
        else:
            w = int(self.get_parameter("preview_width").value)
            h = int(self.get_parameter("preview_height").value)
            fps = max(1, int(self.get_parameter("preview_fps").value))
        self._refresh_ui_topics()

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
            self._fallback_pub0 = self.create_publisher(Image, self._ui_cam0_topic, qos)
        else:
            self._fallback_pub0 = None

        if publish_slots >= 2:
            self._fallback_pub1 = self.create_publisher(Image, self._ui_cam1_topic, qos)
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
        self._fallback_timer = self.create_timer(
            period,
            self._publish_black_previews,
            callback_group=self._timer_cb_group,
        )
        topics = []
        if self._fallback_pub0 is not None:
            topics.append(self._ui_cam0_topic)
        if self._fallback_pub1 is not None:
            topics.append(self._ui_cam1_topic)
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
        orientation = self._camera_orientation_for_index(cam_index)
        if not bool(self.get_parameter("preview_source_applies_orientation").value):
            orientation = 0
        frame_us = int(1_000_000 / max(1, fps))

        params = {
            "camera": int(cam_index),
            "role": role,
            "width": w,
            "height": h,
            "orientation": int(orientation),
            "FrameDurationLimits": [frame_us, frame_us],
            "use_node_time": False,
            "frame_id": frame_id,
        }
        # Avoid forcing pixel format unless explicitly requested.
        if fmt and fmt.lower() not in ("auto", "default", "native"):
            params["format"] = fmt
        return params

    def _camera_orientation_for_index(self, cam_index: int) -> int:
        cam0_idx = int(self.get_parameter("cam0_index").value)
        cam1_idx = int(self.get_parameter("cam1_index").value)
        if int(cam_index) == cam0_idx:
            key = "cam0_orientation"
        elif int(cam_index) == cam1_idx:
            key = "cam1_orientation"
        else:
            key = "cam0_orientation"
        return _normalize_orientation_deg(self.get_parameter(key).value)

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
        self._preview_watchdog_epoch_mono = time.monotonic()
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
        self._preview_watchdog_epoch_mono = time.monotonic()
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
        # Stop both preview processes concurrently to minimize capture downtime.
        workers: List[threading.Thread] = []
        if self._p0 is not None:
            t0 = threading.Thread(target=_stop_proc, args=(self._p0, timeout_s), daemon=True)
            workers.append(t0)
            t0.start()
        if self._p1 is not None:
            t1 = threading.Thread(target=_stop_proc, args=(self._p1, timeout_s), daemon=True)
            workers.append(t1)
            t1.start()
        for t in workers:
            t.join(timeout_s + 0.25)
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

        cmd = [
            "rpicam-still",
            "--nopreview",
            "--camera", str(cam_index),
            "--width", str(width),
            "--height", str(height),
            "--quality", str(quality),
            "-t", str(t_ms),
            "-o", out_path,
        ]
        return cmd

    def _normalize_still_capture_orientation(
        self,
        cam_index: int,
        path: str,
        quality: int,
    ) -> Tuple[bool, str]:
        orientation = self._camera_orientation_for_index(cam_index)
        if orientation == 0 or not path:
            return True, ""
        try:
            frame = cv2.imread(path, cv2.IMREAD_COLOR)
            if frame is None:
                return False, f"failed to read captured JPEG for orientation: {path}"
            frame = _apply_orientation_bgr(frame, orientation)
            ok, err = self._write_jpeg_bgr(path, frame, quality)
            if not ok:
                return False, err
            return True, ""
        except Exception as e:
            return False, str(e)

    def _run_one(self, cam_index: int, out_path: str, quality: int, timeout_s: float) -> Tuple[bool, str, Optional[TimeMsg]]:
        cmd = self._rpicam_cmd(cam_index, out_path, quality)
        end_stamp: Optional[TimeMsg] = None
        try:
            p = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout_s, env=self._preview_env())
            end_stamp = now_ros_time(self)
        except subprocess.TimeoutExpired:
            return False, f"TimeoutExpired running: {' '.join(cmd)}", now_ros_time(self)
        except Exception as e:
            return False, f"Failed running {' '.join(cmd)}: {e}", now_ros_time(self)

        ok = (p.returncode == 0) and os.path.exists(out_path) and os.path.getsize(out_path) > 0
        if ok:
            return True, "", end_stamp
        return False, f"rc={p.returncode}\nstdout:\n{p.stdout}\nstderr:\n{p.stderr}\n", end_stamp

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

        trigger_stamp = now_ros_time(self)
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
        # rpicam-still does not publish a hardware frame stamp here. Use the
        # process completion time as the best available per-image approximation
        # for metadata and IMU deblur alignment.
        cam0_stamp: Optional[TimeMsg] = None
        cam1_stamp: Optional[TimeMsg] = None

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
                        cam0_stamp = now_ros_time(self)
                    except subprocess.TimeoutExpired:
                        p0.kill()
                        out0, err0 = "", "TimeoutExpired"
                        cam0_stamp = now_ros_time(self)
                    out1 = err1 = ""
                    if p1 is not None:
                        try:
                            out1, err1 = p1.communicate(timeout=run_timeout_s)
                            cam1_stamp = now_ros_time(self)
                        except subprocess.TimeoutExpired:
                            p1.kill()
                            out1, err1 = "", "TimeoutExpired"
                            cam1_stamp = now_ros_time(self)

                    ok0 = (p0.returncode == 0) and os.path.exists(cam0_path) and os.path.getsize(cam0_path) > 0
                    if allow_cam1 and p1 is not None:
                        ok1 = (p1.returncode == 0) and os.path.exists(cam1_path) and os.path.getsize(cam1_path) > 0
                    else:
                        ok1 = True
                        cam1_stamp = None
                    if not ok0:
                        d0 = f"attempt={attempt+1}/{retries+1} cam=0 rc={p0.returncode}\nstdout:\n{out0}\nstderr:\n{err0}\n"
                        cam0_stamp = None
                    if allow_cam1 and not ok1 and p1 is not None:
                        d1 = f"attempt={attempt+1}/{retries+1} cam=1 rc={p1.returncode}\nstdout:\n{out1}\nstderr:\n{err1}\n"
                        cam1_stamp = None
                else:
                    ok0, d0, cam0_stamp = self._run_one(cam0, cam0_path, quality, run_timeout_s)
                    if allow_cam1:
                        ok1, d1, cam1_stamp = self._run_one(cam1, cam1_path, quality, run_timeout_s)
                    else:
                        ok1 = True
                        cam1_stamp = None

                    if not ok0:
                        cam0_stamp = None
                    if not ok1:
                        cam1_stamp = None

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
            return False, fail_msg, fail_cam0, fail_cam1, trigger_stamp

        orient0_ok, orient0_err = self._normalize_still_capture_orientation(cam0, cam0_path, quality)
        orient1_ok = True
        orient1_err = ""
        if allow_cam1:
            orient1_ok, orient1_err = self._normalize_still_capture_orientation(cam1, cam1_path, quality)
        if not orient0_ok or not orient1_ok:
            fail_msg = (
                "CAPTURE FAILED (still mode): orientation normalization failed\n"
                f"cam0_ok={orient0_ok} error={orient0_err}\n"
                f"cam1_ok={orient1_ok} error={orient1_err}\n"
            )
            self.get_logger().error(fail_msg)
            return False, fail_msg, cam0_path if orient0_ok else "", cam1_path if orient1_ok else "", trigger_stamp

        if not allow_cam1:
            cam1_path = ""
            msg = "OK (still mode, cam1 skipped: only one camera detected)"
        else:
            msg = "OK (still mode)"

        cam0_stamp = cam0_stamp if cam0_path else None
        cam1_stamp = cam1_stamp if cam1_path else None
        stamp = cam0_stamp or trigger_stamp

        gyro_samples = self._gyro_samples_snapshot()
        cam0_deblur = self._deblur_capture_image(
            cam0_path,
            cam0_stamp,
            quality,
            gyro_samples,
        )
        cam1_deblur = self._deblur_capture_image(
            cam1_path,
            cam1_stamp,
            quality,
            gyro_samples,
        )
        if cam0_deblur.get("path"):
            msg = f"{msg}; cam0_deblur={cam0_deblur.get('path')}"
        if cam1_deblur.get("path"):
            msg = f"{msg}; cam1_deblur={cam1_deblur.get('path')}"
        self._publish_capture_debug(
            {
                "status": "deblur",
                "mode": "still",
                "session_id": session,
                "trigger_stamp": _stamp_to_str(trigger_stamp),
                "cam0_stamp": _stamp_to_str(cam0_stamp) if cam0_stamp is not None else None,
                "cam1_stamp": _stamp_to_str(cam1_stamp) if cam1_stamp is not None else None,
                "cam0_deblur": cam0_deblur,
                "cam1_deblur": cam1_deblur,
            }
        )

        if bool(self.get_parameter("write_capture_metadata").value):
            metadata = self._build_capture_metadata(
                mode="still",
                session=session,
                trigger_stamp=trigger_stamp,
                cam0_path=cam0_path,
                cam1_path=cam1_path,
                cam0_stamp=cam0_stamp,
                cam1_stamp=cam1_stamp,
                cam0_deblur=cam0_deblur,
                cam1_deblur=cam1_deblur,
            )
            if bool(self.get_parameter("write_trajectory_csv").value):
                try:
                    csv_path = self._write_trajectory_csv(out_dir, session, metadata.get("trajectory", {}))
                    if csv_path:
                        metadata["trajectory_csv"] = csv_path
                        msg = f"{msg}; trajectory_csv={csv_path}"
                except Exception as e:
                    self.get_logger().error(f"Trajectory CSV write failed: {e}")
            try:
                meta_path = self._write_capture_metadata(out_dir, session, metadata)
                msg = f"{msg}; metadata={meta_path}"
            except Exception as e:
                warn = f"metadata write failed: {e}"
                self.get_logger().error(f"Capture metadata write failed: {e}")
                msg = f"{msg}; warning={warn}"

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
        initial_wait_s = max(wait_s, float(self.get_parameter("stream_initial_wait_s").value))
        max_age_s = max(0.05, float(self.get_parameter("stream_max_frame_age_s").value))
        pair_slop_ns = int(max(1.0, float(self.get_parameter("stream_pair_max_delta_ms").value)) * 1_000_000.0)
        trigger_ns = _stamp_to_ns(trigger_stamp)
        pre0, pre1 = self._stream_buffers_snapshot()
        if (not pre0) or (require_cam1 and not pre1):
            wait_s = initial_wait_s
            self.get_logger().info(
                f"Stream capture warmup: waiting up to {wait_s:.1f}s for initial frames "
                f"(cam0_seen={bool(pre0)}, cam1_seen={bool(pre1)}, cam1_required={require_cam1})"
            )
        deadline = time.monotonic() + wait_s

        sel0: Optional[StreamFrame] = None
        sel1: Optional[StreamFrame] = None
        pair_delta_ms: Optional[float] = None
        while True:
            buf0, buf1 = self._stream_buffers_snapshot()
            now_m = time.monotonic()

            if require_cam1:
                sel0, sel1, pair_delta_ms, pair_ok = self._pick_stream_frame_pair(
                    buf0,
                    buf1,
                    trigger_ns,
                    max_age_s,
                    now_m,
                    pair_slop_ns,
                )
                ok0 = sel0 is not None
                ok1 = sel1 is not None
            else:
                sel0 = self._pick_stream_frame(buf0, trigger_ns, max_age_s, now_m)
                sel1 = None
                ok0 = sel0 is not None
                ok1 = True
                pair_ok = True
                pair_delta_ms = None

            if ok0 and ok1 and pair_ok:
                break

            if now_m >= deadline:
                age0 = (now_m - sel0[1]) if sel0 is not None else 1e9
                age1 = (now_m - sel1[1]) if sel1 is not None else 1e9
                pair_text = "n/a"
                if pair_delta_ms is not None:
                    pair_text = f"{pair_delta_ms:.2f}"
                fail_msg = (
                    "CAPTURE FAILED (stream mode)\n"
                    f"cam0_ready={ok0} age_s={age0:.3f}\n"
                    f"cam1_required={require_cam1} cam1_ready={ok1} age_s={age1:.3f}\n"
                    f"pair_delta_ms={pair_text} max_pair_delta_ms={pair_slop_ns/1_000_000.0:.2f}\n"
                    "No sufficiently fresh preview frames available."
                )
                self.get_logger().error(fail_msg)
                self._publish_capture_debug(
                    {
                        "status": "failed_select",
                        "mode": "stream",
                        "session_id": session,
                        "trigger_stamp": _stamp_to_str(trigger_stamp),
                        "require_cam1": bool(require_cam1),
                        "cam0_ready": bool(ok0),
                        "cam1_ready": bool(ok1),
                        "cam0_age_ms": float(age0 * 1000.0),
                        "cam1_age_ms": float(age1 * 1000.0),
                        "pair_delta_ms": pair_delta_ms,
                        "pair_limit_ms": float(pair_slop_ns / 1_000_000.0),
                        "cam0_buffer_len": int(len(buf0)),
                        "cam1_buffer_len": int(len(buf1)),
                        "message": fail_msg,
                    }
                )
                return False, fail_msg, "", "", trigger_stamp
            time.sleep(0.01)

        msg0 = sel0[0] if sel0 is not None else None
        msg1 = sel1[0] if sel1 is not None else None
        if msg0 is None:
            fail_msg = "CAPTURE FAILED (stream mode): internal frame selection error (cam0 missing)"
            self.get_logger().error(fail_msg)
            return False, fail_msg, "", "", trigger_stamp

        if sel0 is not None:
            age0 = max(0.0, time.monotonic() - sel0[1])
            if sel1 is not None:
                age1 = max(0.0, time.monotonic() - sel1[1])
                self.get_logger().info(
                    f"Stream frame pair selected: cam0_age_ms={age0*1000.0:.1f} "
                    f"cam1_age_ms={age1*1000.0:.1f} pair_delta_ms={(pair_delta_ms if pair_delta_ms is not None else -1.0):.2f}"
                )
            else:
                self.get_logger().info(
                    f"Stream frame selected: cam0_age_ms={age0*1000.0:.1f}"
                )
                age1 = None

        cam0_stamp_ns = self._msg_stamp_ns(msg0)
        cam1_stamp_ns = self._msg_stamp_ns(msg1) if msg1 is not None else None
        trigger_ns = _stamp_to_ns(trigger_stamp)
        cam0_offset_ms = ((cam0_stamp_ns - trigger_ns) / 1_000_000.0) if cam0_stamp_ns is not None else None
        cam1_offset_ms = ((cam1_stamp_ns - trigger_ns) / 1_000_000.0) if cam1_stamp_ns is not None else None
        self._publish_capture_debug(
            {
                "status": "selected",
                "mode": "stream",
                "session_id": session,
                "trigger_stamp": _stamp_to_str(trigger_stamp),
                "cam0_stamp": _stamp_to_str(msg0.header.stamp),
                "cam1_stamp": _stamp_to_str(msg1.header.stamp) if msg1 is not None else None,
                "cam0_offset_ms": cam0_offset_ms,
                "cam1_offset_ms": cam1_offset_ms,
                "cam0_age_ms": float(age0 * 1000.0),
                "cam1_age_ms": (float(age1 * 1000.0) if age1 is not None else None),
                "pair_delta_ms": pair_delta_ms,
                "pair_limit_ms": float(pair_slop_ns / 1_000_000.0),
                "require_cam1": bool(require_cam1),
                "cam0_buffer_len": int(len(buf0)),
                "cam1_buffer_len": int(len(buf1)),
            }
        )

        if feedback_cb is not None:
            feedback_cb("encoding_stream_frames")

        source_applies_orientation = bool(self.get_parameter("preview_source_applies_orientation").value)
        try:
            frame0 = self._imgmsg_to_bgr(msg0)
            if not source_applies_orientation:
                frame0 = _apply_orientation_bgr(
                    frame0,
                    self._camera_orientation_for_index(int(self.get_parameter("cam0_index").value)),
                )
        except Exception as e:
            fail_msg = f"CAPTURE FAILED (stream mode): cam0 conversion failed: {e}"
            self.get_logger().error(fail_msg)
            self._publish_capture_debug(
                {
                    "status": "failed_encode",
                    "mode": "stream",
                    "session_id": session,
                    "trigger_stamp": _stamp_to_str(trigger_stamp),
                    "message": fail_msg,
                }
            )
            return False, fail_msg, "", "", trigger_stamp

        cam0_ok, cam0_write_err = self._write_jpeg_bgr(cam0_path, frame0, quality)
        if not cam0_ok:
            fail_msg = f"CAPTURE FAILED (stream mode): cam0 JPEG encode/write failed: {cam0_write_err}"
            self.get_logger().error(fail_msg)
            self._publish_capture_debug(
                {
                    "status": "failed_encode",
                    "mode": "stream",
                    "session_id": session,
                    "trigger_stamp": _stamp_to_str(trigger_stamp),
                    "message": fail_msg,
                }
            )
            return False, fail_msg, "", "", trigger_stamp

        cam0_stamp = msg0.header.stamp
        cam1_stamp: Optional[TimeMsg] = None
        wrote_cam1 = False

        if msg1 is not None:
            try:
                frame1 = self._imgmsg_to_bgr(msg1)
                if not source_applies_orientation:
                    frame1 = _apply_orientation_bgr(
                        frame1,
                        self._camera_orientation_for_index(int(self.get_parameter("cam1_index").value)),
                    )
            except Exception as e:
                fail_msg = f"CAPTURE FAILED (stream mode): cam1 conversion failed: {e}"
                self.get_logger().error(fail_msg)
                self._publish_capture_debug(
                    {
                        "status": "failed_encode",
                        "mode": "stream",
                        "session_id": session,
                        "trigger_stamp": _stamp_to_str(trigger_stamp),
                        "message": fail_msg,
                    }
                )
                return False, fail_msg, cam0_path, "", trigger_stamp
            cam1_ok, cam1_write_err = self._write_jpeg_bgr(cam1_path, frame1, quality)
            if not cam1_ok:
                fail_msg = f"CAPTURE FAILED (stream mode): cam1 JPEG encode/write failed: {cam1_write_err}"
                self.get_logger().error(fail_msg)
                self._publish_capture_debug(
                    {
                        "status": "failed_encode",
                        "mode": "stream",
                        "session_id": session,
                        "trigger_stamp": _stamp_to_str(trigger_stamp),
                        "message": fail_msg,
                    }
                )
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

        gyro_samples = self._gyro_samples_snapshot()
        cam0_deblur = self._deblur_capture_image(cam0_path, cam0_stamp, quality, gyro_samples)
        cam1_deblur = self._deblur_capture_image(cam1_path, cam1_stamp, quality, gyro_samples)
        if cam0_deblur.get("path"):
            msg = f"{msg}; cam0_deblur={cam0_deblur.get('path')}"
        if cam1_deblur.get("path"):
            msg = f"{msg}; cam1_deblur={cam1_deblur.get('path')}"
        self._publish_capture_debug(
            {
                "status": "deblur",
                "mode": "stream",
                "session_id": session,
                "trigger_stamp": _stamp_to_str(trigger_stamp),
                "cam0_deblur": cam0_deblur,
                "cam1_deblur": cam1_deblur,
            }
        )

        if bool(self.get_parameter("write_capture_metadata").value):
            metadata = self._build_capture_metadata(
                mode="stream",
                session=session,
                trigger_stamp=trigger_stamp,
                cam0_path=cam0_path,
                cam1_path=cam1_path,
                cam0_stamp=cam0_stamp if cam0_stamp is not None else trigger_stamp,
                cam1_stamp=cam1_stamp,
                cam0_deblur=cam0_deblur,
                cam1_deblur=cam1_deblur,
            )
            if bool(self.get_parameter("write_trajectory_csv").value):
                try:
                    csv_path = self._write_trajectory_csv(out_dir, session, metadata.get("trajectory", {}))
                    if csv_path:
                        metadata["trajectory_csv"] = csv_path
                        msg = f"{msg}; trajectory_csv={csv_path}"
                except Exception as e:
                    self.get_logger().error(f"Trajectory CSV write failed: {e}")
            try:
                meta_path = self._write_capture_metadata(out_dir, session, metadata)
                msg = f"{msg}; metadata={meta_path}"
            except Exception as e:
                warn = f"metadata write failed: {e}"
                self.get_logger().error(f"Capture metadata write failed: {e}")
                msg = f"{msg}; warning={warn}"

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

    def _perform_capture_safe(
        self,
        session_in: str,
        out_dir_in: str,
        quality_in: int,
        feedback_cb: Optional[Callable[[str], None]] = None,
    ) -> Tuple[bool, str, str, str, TimeMsg]:
        try:
            return self._perform_capture(session_in, out_dir_in, quality_in, feedback_cb=feedback_cb)
        except Exception as e:
            msg = f"CAPTURE FAILED (internal error): {e}"
            self.get_logger().error(msg)
            self.get_logger().error(traceback.format_exc())
            return False, msg, "", "", now_ros_time(self)

    def on_capture(self, req: CapturePair.Request, res: CapturePair.Response) -> CapturePair.Response:
        with self._capture_lock:
            success, message, cam0_path, cam1_path, stamp = self._perform_capture_safe(
                req.session_id,
                req.output_dir,
                int(req.jpeg_quality),
            )
        self._publish_capture_event(
            source="service",
            session_id=req.session_id,
            success=success,
            message=message,
            cam0_path=cam0_path,
            cam1_path=cam1_path,
            stamp=stamp,
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
            success, message, cam0_path, cam1_path, stamp = self._perform_capture_safe(
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
        self._publish_capture_event(
            source="action",
            session_id=goal.session_id,
            success=success,
            message=message,
            cam0_path=cam0_path,
            cam1_path=cam1_path,
            stamp=stamp,
        )

        if success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result


def main():
    rclpy.init()
    node = CaptureService()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            executor.shutdown()
        except Exception:
            pass
        # Ensure we don't leave camera processes behind if we own them.
        try:
            node._stop_preview_watchdog()
        except Exception:
            pass
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
            node._stop_preview_relay()
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
