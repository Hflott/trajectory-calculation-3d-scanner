#!/usr/bin/env python3
"""Touch UI for dual-camera preview + synchronized capture.

Robustness + UX goals:
  * Avoid blocking startup if services are missing
  * Clear on-screen status for camera streams + service readiness
  * Dark theme across *all* Qt widgets (tabs, scroll areas, buttons, etc.)
  * Lightweight preview pipeline (no unnecessary frame copies)
"""

import json
import os
import sys
import threading
import time
from collections import deque
from datetime import datetime
from typing import Any, Deque, Dict, List, Optional, Tuple

import cv2
import numpy as np

from action_msgs.msg import GoalStatus
from qtpy.QtCore import Qt, QTimer
from qtpy.QtGui import QImage, QPixmap
from qtpy.QtWidgets import (
    QApplication,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPlainTextEdit,
    QPushButton,
    QScrollArea,
    QSizePolicy,
    QSpinBox,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)
from rclpy.task import Future
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import Image, Imu, NavSatFix, TimeReference
from std_msgs.msg import String
from cv_bridge import CvBridge

from subsea_interfaces.action import CapturePair as CapturePairAction
from subsea_interfaces.srv import CapturePair

from .theme import apply_dark_theme


def frame_to_pix(frame: np.ndarray, encoding: str) -> QPixmap:
    """Convert an HxWx3 uint8 frame to QPixmap.

    Prefer explicit RGB conversion for better compatibility with XQuartz/X11.
    """
    if frame is None:
        return QPixmap()
    if frame.ndim != 3 or frame.shape[2] != 3:
        return QPixmap()
    if not frame.flags["C_CONTIGUOUS"]:
        frame = np.ascontiguousarray(frame)

    enc = (encoding or "").lower()
    if enc == "rgb8":
        fmt = QImage.Format_RGB888
    elif enc == "bgr8" and hasattr(QImage, "Format_BGR888"):
        # Qt >= 5.14 supports direct BGR888; avoid extra cvtColor copy.
        fmt = QImage.Format_BGR888
    else:
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        fmt = QImage.Format_RGB888

    h, w, _ = frame.shape
    # Detach from numpy buffer to avoid artifacts from reused transport buffers.
    qimg = QImage(frame.data, w, h, frame.strides[0], fmt).copy()
    return QPixmap.fromImage(qimg)


def load_jpeg_as_pix(path: str) -> Optional[QPixmap]:
    if not path or not os.path.exists(path):
        return None
    # Loading 12MP JPEGs at full resolution is expensive on a Pi.
    # Reduced decode keeps memory + latency under control for UI use.
    bgr = cv2.imread(path, cv2.IMREAD_REDUCED_COLOR_4)
    if bgr is None:
        return None
    return frame_to_pix(bgr, "bgr8")


def _ts() -> str:
    return datetime.now().strftime("%H:%M:%S")


def _fmt_stamp(stamp) -> str:
    if stamp is None:
        return "—"
    try:
        return f"{int(stamp.sec)}.{int(stamp.nanosec):09d}"
    except Exception:
        return "—"


def _conf_path() -> str:
    return os.path.expanduser("~/.config/subsea_ui/config.json")


def load_config() -> dict:
    p = _conf_path()
    try:
        with open(p, "r", encoding="utf-8") as f:
            return json.load(f)
    except Exception:
        return {}


def save_config(cfg: dict) -> None:
    p = _conf_path()
    os.makedirs(os.path.dirname(p), exist_ok=True)
    tmp = p + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(cfg, f, indent=2, sort_keys=True)
    os.replace(tmp, p)


def _single_instance_lock() -> None:
    lock_path = "/tmp/subsea_ui.lock"
    try:
        import fcntl
        fd = os.open(lock_path, os.O_CREAT | os.O_RDWR, 0o644)
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except Exception:
        print("subsea_ui: another instance is already running.", file=sys.stderr)
        raise SystemExit(0)


class ImageSub(Node):
    def __init__(self, name: str, topic: str):
        super().__init__(name)
        self.topic = topic
        self.bridge = CvBridge()
        self._lock = threading.Lock()
        self._latest: Optional[np.ndarray] = None
        self._latest_encoding: str = "bgr8"
        self._latest_msg: Optional[Image] = None  # keep msg alive for zero-copy views
        self._got_first = False
        self._last_rx_mono: Optional[float] = None
        self._ema_fps: float = 0.0
        self._seen_encodings: set[str] = set()
        self._last_bad_frame_warn_mono: float = 0.0

        # Depth=1 prevents queue buildup when UI/processing can't keep up.
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.sub = self.create_subscription(Image, topic, self.cb, qos)
        self.get_logger().info(f"Preview sub: {topic}")

    def _warn_bad_frame(self, msg: Image, reason: str) -> None:
        now_m = time.monotonic()
        if (now_m - self._last_bad_frame_warn_mono) < 1.0:
            return
        self._last_bad_frame_warn_mono = now_m
        self.get_logger().warn(
            f"{self.topic}: dropped frame ({reason}); "
            f"encoding={msg.encoding} size={msg.width}x{msg.height} step={msg.step} bytes={len(msg.data)}"
        )

    def cb(self, msg: Image):
        # Fast path: avoid cv_bridge copy/convert when encoding is directly usable.
        frame = None
        enc = (msg.encoding or "").lower().strip()
        if enc not in self._seen_encodings:
            self._seen_encodings.add(enc)
            self.get_logger().info(
                f"Preview stream format on {self.topic}: "
                f"encoding={enc or 'unknown'} size={msg.width}x{msg.height} step={msg.step}"
            )

        w = int(msg.width)
        h = int(msg.height)
        step = int(msg.step)
        if w <= 0 or h <= 0:
            self._warn_bad_frame(msg, "invalid dimensions")
            return

        if step <= 0:
            if enc in ("bgr8", "rgb8"):
                step = w * 3
            elif enc in ("bgra8", "rgba8"):
                step = w * 4
            elif enc == "mono8":
                step = w

        if step > 0 and len(msg.data) < (step * h):
            self._warn_bad_frame(msg, "truncated payload")
            return

        try:
            mv = memoryview(msg.data)
            if enc in ("bgr8", "rgb8") and step >= (w * 3):
                frame = np.ndarray(
                    (h, w, 3),
                    dtype=np.uint8,
                    buffer=mv,
                    strides=(step, 3, 1),
                )
            elif enc in ("bgra8", "rgba8") and step >= (w * 4):
                frame4 = np.ndarray(
                    (h, w, 4),
                    dtype=np.uint8,
                    buffer=mv,
                    strides=(step, 4, 1),
                )
                conv = cv2.COLOR_BGRA2BGR if enc == "bgra8" else cv2.COLOR_RGBA2BGR
                frame = cv2.cvtColor(frame4, conv)
                enc = "bgr8"
            elif enc == "mono8" and step >= w:
                gray = np.ndarray(
                    (h, w),
                    dtype=np.uint8,
                    buffer=mv,
                    strides=(step, 1),
                )
                frame = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
                enc = "bgr8"
        except Exception:
            frame = None

        if frame is None:
            try:
                # Fallback: convert to BGR for display.
                frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                enc = "bgr8"
            except Exception as e:
                self._warn_bad_frame(msg, f"cv_bridge conversion failed: {e}")
                return
        else:
            # Keep a private copy to avoid rendering from a reused shared buffer.
            frame = frame.copy()
        now_m = time.monotonic()
        with self._lock:
            self._latest = frame
            self._latest_encoding = enc if enc in ("bgr8", "rgb8") else "bgr8"
            self._latest_msg = msg  # ensure buffer stays valid for zero-copy view
            self._got_first = True
            if self._last_rx_mono is not None:
                dt = max(1e-6, now_m - self._last_rx_mono)
                fps = 1.0 / dt
                self._ema_fps = fps if self._ema_fps <= 0.0 else (0.85 * self._ema_fps + 0.15 * fps)
            self._last_rx_mono = now_m

    def got_first_frame(self) -> bool:
        with self._lock:
            return self._got_first

    def get_latest(self) -> Optional[np.ndarray]:
        # We replace the frame pointer on each callback (no in-place mutation),
        # so returning the reference is safe and avoids a full-frame copy.
        with self._lock:
            return self._latest

    def get_latest_snapshot(self) -> Tuple[Optional[np.ndarray], str, Optional[Image]]:
        # Return frame + encoding + backing message reference in one lock scope.
        # Keeping msg_ref alive in the caller prevents buffer invalidation while rendering.
        with self._lock:
            return self._latest, self._latest_encoding, self._latest_msg

    def latest_encoding(self) -> str:
        with self._lock:
            return self._latest_encoding

    def stream_stats(self) -> Tuple[float, float]:
        """Returns (age_s, fps_ema). age_s is large if we have no frames yet."""
        with self._lock:
            t = self._last_rx_mono
            fps = self._ema_fps
        if t is None:
            return 1e9, fps
        return max(0.0, time.monotonic() - t), fps


class GnssSub(Node):
    def __init__(self, name: str, fix_topic: str, time_ref_topic: str, imu_topic: str):
        super().__init__(name)
        self.fix_topic = fix_topic
        self.time_ref_topic = time_ref_topic
        self.imu_topic = imu_topic

        self._lock = threading.Lock()
        self._fix: Optional[NavSatFix] = None
        self._time_ref: Optional[TimeReference] = None
        self._imu: Optional[Imu] = None
        self._fix_rx_mono: Optional[float] = None
        self._time_ref_rx_mono: Optional[float] = None
        self._imu_rx_mono: Optional[float] = None

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._fix_sub = self.create_subscription(NavSatFix, fix_topic, self._on_fix, qos)
        self._time_ref_sub = self.create_subscription(TimeReference, time_ref_topic, self._on_time_ref, qos)
        self._imu_sub = self.create_subscription(Imu, imu_topic, self._on_imu, qos)
        self.get_logger().info(f"GNSS sub: fix={fix_topic} time_ref={time_ref_topic} imu={imu_topic}")

    def _on_fix(self, msg: NavSatFix):
        with self._lock:
            self._fix = msg
            self._fix_rx_mono = time.monotonic()

    def _on_time_ref(self, msg: TimeReference):
        with self._lock:
            self._time_ref = msg
            self._time_ref_rx_mono = time.monotonic()

    def _on_imu(self, msg: Imu):
        with self._lock:
            self._imu = msg
            self._imu_rx_mono = time.monotonic()

    def snapshot(self):
        with self._lock:
            return (
                self._fix,
                self._time_ref,
                self._imu,
                self._fix_rx_mono,
                self._time_ref_rx_mono,
                self._imu_rx_mono,
            )


class AppNode(Node):
    def __init__(self):
        cfg = load_config()
        super().__init__("subsea_ui_node")

        # Parameters (overridable via --ros-args -p name:=value)
        self.declare_parameter("cam0_topic", cfg.get("cam0_topic", "/cam0/camera/image_raw"))
        self.declare_parameter("cam1_topic", cfg.get("cam1_topic", "/cam1/camera/image_raw"))
        self.declare_parameter("capture_service", cfg.get("capture_service", "capture_pair"))
        self.declare_parameter("capture_action", cfg.get("capture_action", "capture_pair"))
        self.declare_parameter("prefer_capture_action", bool(cfg.get("prefer_capture_action", True)))
        self.declare_parameter("capture_node", cfg.get("capture_node", "/capture_service"))
        self.declare_parameter("capture_event_topic", cfg.get("capture_event_topic", "/capture/events"))
        self.declare_parameter("mock_camera_node", cfg.get("mock_camera_node", "/mock_camera_publisher"))
        self.declare_parameter("gnss_fix_topic", cfg.get("gnss_fix_topic", "/fix"))
        self.declare_parameter("gnss_time_ref_topic", cfg.get("gnss_time_ref_topic", "/time_reference"))
        self.declare_parameter("gnss_imu_topic", cfg.get("gnss_imu_topic", "/imu/data"))
        self.declare_parameter("output_dir", cfg.get("output_dir", os.path.expanduser("~/captures")))
        self.declare_parameter("jpeg_quality", int(cfg.get("jpeg_quality", 95)))
        self.declare_parameter("ui_fps", int(cfg.get("ui_fps", 15)))
        self.declare_parameter("preview_fps", int(cfg.get("preview_fps", 15)))

        self.cli = self.create_client(CapturePair, str(self.get_parameter("capture_service").value))
        self.action_cli = ActionClient(self, CapturePairAction, str(self.get_parameter("capture_action").value))
        self._prefer_action = bool(self.get_parameter("prefer_capture_action").value)
        self._mock_cam_params = AsyncParameterClient(
            self,
            str(self.get_parameter("mock_camera_node").value),
        )
        self._capture_params = AsyncParameterClient(
            self,
            str(self.get_parameter("capture_node").value),
        )
        self._capture_event_lock = threading.Lock()
        self._capture_events: Deque[Dict[str, Any]] = deque(maxlen=64)
        evt_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        evt_topic = str(self.get_parameter("capture_event_topic").value)
        self._capture_evt_sub = self.create_subscription(String, evt_topic, self._on_capture_event, evt_qos)
        self.get_logger().info(f"Capture event sub: {evt_topic}")

    def _on_capture_event(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except Exception:
            return
        if not isinstance(payload, dict):
            return
        with self._capture_event_lock:
            self._capture_events.append(payload)

    def pop_capture_events(self) -> List[Dict[str, Any]]:
        with self._capture_event_lock:
            if not self._capture_events:
                return []
            out = list(self._capture_events)
            self._capture_events.clear()
            return out

    def capture_pair_async(self, session_id: str, out_dir: str, quality: int = 95):
        if self._prefer_action and self.action_cli.server_is_ready():
            return self._capture_pair_action_async(session_id, out_dir, quality)
        req = CapturePair.Request()
        req.session_id = session_id
        req.output_dir = out_dir
        req.jpeg_quality = quality
        return self.cli.call_async(req)

    def _capture_pair_action_async(self, session_id: str, out_dir: str, quality: int = 95):
        result_future = Future()
        goal = CapturePairAction.Goal()
        goal.session_id = session_id
        goal.output_dir = out_dir
        goal.jpeg_quality = quality

        send_goal_future = self.action_cli.send_goal_async(goal)

        def on_goal_done(fut):
            try:
                goal_handle = fut.result()
            except Exception as e:
                result_future.set_exception(e)
                return

            if goal_handle is None or not goal_handle.accepted:
                if self.cli.service_is_ready():
                    req = CapturePair.Request()
                    req.session_id = session_id
                    req.output_dir = out_dir
                    req.jpeg_quality = quality
                    service_fut = self.cli.call_async(req)

                    def on_service_done(sf):
                        try:
                            result_future.set_result(sf.result())
                        except Exception as e:
                            result_future.set_exception(e)

                    service_fut.add_done_callback(on_service_done)
                    return
                result_future.set_exception(RuntimeError("Capture action goal rejected"))
                return

            get_result_future = goal_handle.get_result_async()

            def on_result_done(rf):
                try:
                    action_result = rf.result()
                except Exception as e:
                    result_future.set_exception(e)
                    return

                resp = CapturePair.Response()
                if action_result is None:
                    resp.success = False
                    resp.message = "No action result returned"
                    result_future.set_result(resp)
                    return

                res = action_result.result
                resp.success = bool(res.success) and (action_result.status == GoalStatus.STATUS_SUCCEEDED)
                resp.message = str(res.message)
                resp.cam0_path = str(res.cam0_path)
                resp.cam1_path = str(res.cam1_path)
                resp.stamp = res.stamp
                result_future.set_result(resp)

            get_result_future.add_done_callback(on_result_done)

        send_goal_future.add_done_callback(on_goal_done)
        return result_future

    def service_ready(self) -> bool:
        return self.action_cli.server_is_ready() or self.cli.service_is_ready()

    def set_mock_camera_fps_async(self, fps: int):
        # Best-effort: if no mock node is present, caller should continue silently.
        try:
            ready = self._mock_cam_params.services_are_ready()
        except Exception:
            ready = False
        if not ready:
            return None
        params = [Parameter("fps", Parameter.Type.INTEGER, int(max(1, fps)))]
        return self._mock_cam_params.set_parameters(params)

    def set_capture_preview_fps_async(self, fps: int):
        # Best-effort: if capture node parameter service is unavailable, continue silently.
        try:
            ready = self._capture_params.services_are_ready()
        except Exception:
            ready = False
        if not ready:
            return None
        params = [Parameter("preview_fps", Parameter.Type.INTEGER, int(max(1, fps)))]
        return self._capture_params.set_parameters(params)


class MainWindow(QWidget):
    def __init__(self, ros_node: AppNode, cam0: ImageSub, cam1: ImageSub, gnss: GnssSub):
        super().__init__()
        self.ros_node = ros_node
        self.cam0 = cam0
        self.cam1 = cam1
        self.gnss = gnss

        self.setWindowTitle("Trajectory Capture UI")

        self.tabs = QTabWidget()

        # --- Connection / top status
        self.ind_cam0 = QLabel("● Cam0")
        self.ind_cam1 = QLabel("● Cam1")
        self.ind_srv = QLabel("● Capture service")
        for ind in (self.ind_cam0, self.ind_cam1, self.ind_srv):
            ind.setStyleSheet("font-weight:600;")

        self.status = QLabel("Status: ready")
        self.status.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        self.status.setStyleSheet("background:#1B1B1B; border:1px solid #2B2B2B; border-radius:10px; padding:8px;")

        # --- Preview widgets
        self.prev0 = QLabel("Cam0: waiting…")
        self.prev1 = QLabel("Cam1: waiting…")
        self.prev0_info = QLabel("—")
        self.prev1_info = QLabel("—")

        # --- Capture widgets
        self.cap0 = QLabel("Cam0 capture: (none)")
        self.cap1 = QLabel("Cam1 capture: (none)")
        self.res0 = QLabel("Cam0 deblur: (placeholder)")
        self.res1 = QLabel("Cam1 deblur: (placeholder)")
        self.capture_details = QPlainTextEdit()
        self.capture_details.setReadOnly(True)
        self.capture_details.setMaximumBlockCount(2000)

        for lab in (self.prev0, self.prev1, self.cap0, self.cap1, self.res0, self.res1):
            lab.setAlignment(Qt.AlignCenter)
            lab.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
            lab.setStyleSheet("background-color:black; color:white; font-size:16px; border-radius:12px;")
            lab.setMinimumSize(140, 90)
        for info in (self.prev0_info, self.prev1_info):
            info.setStyleSheet("color:#B0B0B0; padding:4px 2px;")

        # --- Actions
        self.quit_btn = QPushButton("Quit")
        self.quit_btn.setMinimumHeight(34)
        self.quit_btn.setStyleSheet("font-size:14px; padding:4px 10px;")
        self.quit_btn.clicked.connect(self.close)

        # Store last pixmaps so we can rescale on resize
        self._cap0_pix: Optional[QPixmap] = None
        self._cap1_pix: Optional[QPixmap] = None
        self._res0_pix: Optional[QPixmap] = None
        self._res1_pix: Optional[QPixmap] = None

        # When capturing a still image, we intentionally pause/stop the preview camera
        # pipelines (to give libcamera exclusive access for rpicam-still). Keep the
        # last frame on-screen instead of flashing "no signal".
        self._preview_paused: bool = False
        self._ind_state = {"cam0": None, "cam1": None, "srv": None}
        self._last_capture_event_key: Optional[str] = None

        # User-settable values (persisted)
        self.out_dir = str(self.ros_node.get_parameter("output_dir").value)
        self.jpeg_quality = int(self.ros_node.get_parameter("jpeg_quality").value)
        self.ui_fps = max(1, int(self.ros_node.get_parameter("ui_fps").value))
        self.preview_fps = max(1, int(self.ros_node.get_parameter("preview_fps").value))

        # ---- Preview tab
        top_row = QHBoxLayout()
        top_row.setSpacing(10)
        top_row.addWidget(self.ind_cam0)
        top_row.addWidget(self.ind_cam1)
        top_row.addWidget(self.ind_srv)
        top_row.addStretch(1)
        top_row.addWidget(self.quit_btn, 0, Qt.AlignRight)

        preview_row = QHBoxLayout()
        preview_row.setSpacing(10)

        left = QVBoxLayout()
        left.setSpacing(6)
        left.addWidget(QLabel("Cam0"))
        left.addWidget(self.prev0, 1)
        left.addWidget(self.prev0_info)

        right = QVBoxLayout()
        right.setSpacing(6)
        right.addWidget(QLabel("Cam1"))
        right.addWidget(self.prev1, 1)
        right.addWidget(self.prev1_info)

        preview_row.addLayout(left, 1)
        preview_row.addLayout(right, 1)

        preview_root = QVBoxLayout()
        preview_root.setContentsMargins(10, 10, 10, 10)
        preview_root.setSpacing(10)
        preview_root.addLayout(top_row, 0)
        preview_root.addLayout(preview_row, 1)
        preview_root.addWidget(self.status, 0)

        preview_tab = QWidget()
        preview_tab.setLayout(preview_root)
        preview_scroll = QScrollArea()
        preview_scroll.setWidgetResizable(True)
        preview_scroll.setWidget(preview_tab)
        self._tab_idx_preview = self.tabs.addTab(preview_scroll, "Preview")

        # ---- GNSS tab
        gnss_tab = QWidget()
        gnss_root = QVBoxLayout()
        gnss_root.setContentsMargins(10, 10, 10, 10)
        gnss_root.setSpacing(8)

        self.gnss_status = QLabel("GNSS: waiting…")
        self.gnss_status.setStyleSheet("font-size:18px; font-weight:700;")
        self.gnss_fix_age = QLabel("Fix age: —")
        self.gnss_fix_stamp = QLabel("Fix stamp: —")
        self.gnss_latlon = QLabel("Lat/Lon: —")
        self.gnss_alt = QLabel("Alt: —")
        self.gnss_cov = QLabel("Covariance: —")
        self.gnss_fix_meta = QLabel("Status: —")

        self.gnss_time_ref = QLabel("TimeRef stamp: —")
        self.gnss_time_ref_src = QLabel("TimeRef source: —")
        self.gnss_time_ref_age = QLabel("TimeRef age: —")

        self.imu_stamp = QLabel("IMU stamp: —")
        self.imu_vals = QLabel("IMU ang vel / lin acc: —")
        self.imu_age = QLabel("IMU age: —")

        for l in (
            self.gnss_fix_age,
            self.gnss_fix_stamp,
            self.gnss_latlon,
            self.gnss_alt,
            self.gnss_cov,
            self.gnss_fix_meta,
            self.gnss_time_ref,
            self.gnss_time_ref_src,
            self.gnss_time_ref_age,
            self.imu_stamp,
            self.imu_vals,
            self.imu_age,
        ):
            l.setStyleSheet("font-size:15px;")
            gnss_root.addWidget(l)

        gnss_root.insertWidget(0, self.gnss_status)
        gnss_tab.setLayout(gnss_root)
        self._tab_idx_gnss = self.tabs.addTab(gnss_tab, "GNSS")

        # ---- Capture tab (scrollable to prevent cut-off)
        cap_row = QHBoxLayout()
        cap_row.setSpacing(10)
        cap_row.addWidget(self.cap0, 1)
        cap_row.addWidget(self.cap1, 1)

        res_row = QHBoxLayout()
        res_row.setSpacing(10)
        res_row.addWidget(self.res0, 1)
        res_row.addWidget(self.res1, 1)

        capture_inner = QWidget()
        capture_layout = QVBoxLayout()
        capture_layout.setContentsMargins(10, 10, 10, 10)
        capture_layout.setSpacing(10)
        capture_layout.addLayout(cap_row, 1)
        capture_layout.addLayout(res_row, 1)
        capture_layout.addWidget(QLabel("Details"), 0)
        capture_layout.addWidget(self.capture_details, 0)
        capture_inner.setLayout(capture_layout)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(capture_inner)

        self._tab_idx_capture = self.tabs.addTab(scroll, "Last Capture")

        # ---- Settings tab (split into sub-tabs + per-page scrolling)
        settings = QWidget()
        sroot = QVBoxLayout()
        sroot.setContentsMargins(10, 10, 10, 10)
        sroot.setSpacing(10)

        self.out_dir_edit = QLineEdit(self.out_dir)
        self.out_dir_edit.setPlaceholderText("/path/to/output")
        self.out_dir_apply = QPushButton("Save Settings")
        self.out_dir_apply.clicked.connect(self.on_save_settings)

        self.quality_spin = QSpinBox()
        self.quality_spin.setRange(10, 100)
        self.quality_spin.setValue(self.jpeg_quality)
        self.quality_spin.setSingleStep(1)

        self.ui_fps_spin = QSpinBox()
        self.ui_fps_spin.setRange(5, 60)
        self.ui_fps_spin.setValue(self.ui_fps)
        self.ui_fps_spin.setSingleStep(5)
        self.preview_fps_spin = QSpinBox()
        self.preview_fps_spin.setRange(1, 60)
        self.preview_fps_spin.setValue(self.preview_fps)
        self.preview_fps_spin.setSingleStep(1)

        # Topics are shown (and persisted) but changing them mid-run is risky.
        # Keep them editable but require restart.
        self.cam0_topic_edit = QLineEdit(str(self.ros_node.get_parameter("cam0_topic").value))
        self.cam1_topic_edit = QLineEdit(str(self.ros_node.get_parameter("cam1_topic").value))
        self.srv_name_edit = QLineEdit(str(self.ros_node.get_parameter("capture_service").value))
        self.gnss_fix_topic_edit = QLineEdit(str(self.ros_node.get_parameter("gnss_fix_topic").value))
        self.gnss_time_ref_topic_edit = QLineEdit(str(self.ros_node.get_parameter("gnss_time_ref_topic").value))
        self.gnss_imu_topic_edit = QLineEdit(str(self.ros_node.get_parameter("gnss_imu_topic").value))

        for w in (
            self.cam0_topic_edit,
            self.cam1_topic_edit,
            self.srv_name_edit,
            self.gnss_fix_topic_edit,
            self.gnss_time_ref_topic_edit,
            self.gnss_imu_topic_edit,
        ):
            w.setToolTip("Changing topics/services requires restarting the UI node")

        def row(label: str, widget: QWidget) -> QHBoxLayout:
            r = QHBoxLayout()
            r.setSpacing(10)
            l = QLabel(label)
            l.setMinimumWidth(150)
            r.addWidget(l)
            r.addWidget(widget, 1)
            return r

        def as_scroll(content: QWidget) -> QScrollArea:
            scroll = QScrollArea()
            scroll.setWidgetResizable(True)
            scroll.setWidget(content)
            return scroll

        settings_tabs = QTabWidget()

        capture_settings_page = QWidget()
        cap_root = QVBoxLayout()
        cap_root.setContentsMargins(10, 10, 10, 10)
        cap_root.setSpacing(10)
        cap_root.addLayout(row("Output directory", self.out_dir_edit))
        cap_root.addLayout(row("JPEG quality", self.quality_spin))
        cap_root.addStretch(1)
        capture_settings_page.setLayout(cap_root)
        settings_tabs.addTab(as_scroll(capture_settings_page), "Capture")

        preview_settings_page = QWidget()
        prev_root = QVBoxLayout()
        prev_root.setContentsMargins(10, 10, 10, 10)
        prev_root.setSpacing(10)
        prev_root.addLayout(row("Preview UI FPS", self.ui_fps_spin))
        prev_root.addLayout(row("Preview camera FPS", self.preview_fps_spin))
        prev_root.addStretch(1)
        preview_settings_page.setLayout(prev_root)
        settings_tabs.addTab(as_scroll(preview_settings_page), "Preview")

        ros_settings_page = QWidget()
        ros_root = QVBoxLayout()
        ros_root.setContentsMargins(10, 10, 10, 10)
        ros_root.setSpacing(10)
        ros_note = QLabel("ROS topics/services apply after UI restart.")
        ros_note.setStyleSheet("color:#B0B0B0;")
        ros_root.addWidget(ros_note)
        ros_root.addLayout(row("Cam0 topic", self.cam0_topic_edit))
        ros_root.addLayout(row("Cam1 topic", self.cam1_topic_edit))
        ros_root.addLayout(row("Capture service", self.srv_name_edit))
        ros_root.addLayout(row("GNSS fix topic", self.gnss_fix_topic_edit))
        ros_root.addLayout(row("GNSS time ref topic", self.gnss_time_ref_topic_edit))
        ros_root.addLayout(row("GNSS IMU topic", self.gnss_imu_topic_edit))
        ros_root.addStretch(1)
        ros_settings_page.setLayout(ros_root)
        settings_tabs.addTab(as_scroll(ros_settings_page), "ROS")

        sroot.addWidget(settings_tabs, 1)
        sroot.addWidget(self.out_dir_apply, 0)
        settings.setLayout(sroot)
        self._tab_idx_settings = self.tabs.addTab(settings, "Settings")

        # ---- Diagnostics tab
        diag = QWidget()
        droot = QVBoxLayout()
        droot.setContentsMargins(10, 10, 10, 10)
        droot.setSpacing(10)
        self.log_box = QPlainTextEdit()
        self.log_box.setReadOnly(True)
        self.log_box.setMaximumBlockCount(5000)
        clear_btn = QPushButton("Clear")
        clear_btn.clicked.connect(lambda: self.log_box.setPlainText(""))
        droot.addWidget(self.log_box, 1)
        droot.addWidget(clear_btn, 0)
        diag.setLayout(droot)
        self._tab_idx_diag = self.tabs.addTab(diag, "Diagnostics")

        outer = QVBoxLayout()
        outer.setContentsMargins(0, 0, 0, 0)
        outer.addWidget(self.tabs)
        self.setLayout(outer)
        self.setMinimumSize(460, 320)

        os.makedirs(self.out_dir, exist_ok=True)

        # Preview refresh (UI-side). Actual camera FPS is independent.
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh_preview)
        self.timer.start(max(10, int(1000 / self.ui_fps)))

        QTimer.singleShot(0, self._set_default_window_geometry)
        self._log(f"UI started. cam0_topic={self.cam0.topic} cam1_topic={self.cam1.topic}")

    def _set_default_window_geometry(self):
        scr = QApplication.primaryScreen()
        if not scr:
            return
        geo = scr.availableGeometry()
        w = max(640, int(geo.width() * 0.9))
        h = max(360, int(geo.height() * 0.9))
        w = min(w, geo.width())
        h = min(h, geo.height())
        x = geo.x() + max(0, (geo.width() - w) // 2)
        y = geo.y() + max(0, (geo.height() - h) // 2)
        self.setGeometry(x, y, w, h)
        self._apply_compact_mode_if_small(geo.width(), geo.height())

    def _apply_compact_mode_if_small(self, screen_w: int, screen_h: int):
        if screen_h > 650 and screen_w > 1100:
            return
        self.quit_btn.setMinimumHeight(30)
        self.quit_btn.setStyleSheet("font-size:13px; padding:2px 8px;")
        for lab in (self.prev0, self.prev1, self.cap0, self.cap1, self.res0, self.res1):
            lab.setMinimumSize(110, 70)

    def resizeEvent(self, e):
        super().resizeEvent(e)
        # Rescale last capture pixmaps to avoid cut-off / stale scaling
        self._apply_capture_pixmaps()

    def keyPressEvent(self, e):
        super().keyPressEvent(e)

    def refresh_preview(self):
        self._update_indicators()
        if self._preview_paused:
            self.prev0_info.setText("paused for capture…")
            self.prev1_info.setText("paused for capture…")
        else:
            self._render_preview(self.prev0, self.prev0_info, self.cam0, "Cam0")
            self._render_preview(self.prev1, self.prev1_info, self.cam1, "Cam1")
        self._consume_capture_events()
        self._refresh_gnss()

    def _consume_capture_events(self) -> None:
        events = self.ros_node.pop_capture_events()
        if not events:
            return

        for ev in events:
            source = str(ev.get("source", "")).strip().lower()
            if source != "gpio":
                continue

            session = str(ev.get("session_id", ""))
            try:
                sec = int(ev.get("stamp_sec", 0))
            except Exception:
                sec = 0
            try:
                nsec = int(ev.get("stamp_nanosec", 0))
            except Exception:
                nsec = 0

            key = f"{source}|{session}|{sec}|{nsec}"
            if self._last_capture_event_key == key:
                continue
            self._last_capture_event_key = key

            success = bool(ev.get("success", False))
            message = str(ev.get("message", ""))
            cam0_path = str(ev.get("cam0_path", ""))
            cam1_path = str(ev.get("cam1_path", ""))

            if success:
                self.status.setText("Status: GPIO capture OK")
                self._log(f"GPIO capture OK: session={session} cam0={cam0_path} cam1={cam1_path}")
                self._cap0_pix = load_jpeg_as_pix(cam0_path)
                self._cap1_pix = load_jpeg_as_pix(cam1_path)
                self._res0_pix = self._cap0_pix
                self._res1_pix = self._cap1_pix
                self._apply_capture_pixmaps()
                self.capture_details.setPlainText(
                    "\n".join(
                        [
                            f"source: gpio",
                            f"session: {session}",
                            f"stamp: {sec}.{nsec:09d}",
                            f"cam0:  {cam0_path}",
                            f"cam1:  {cam1_path}",
                            f"msg:   {message}",
                        ]
                    )
                )
            else:
                self.status.setText("Status: GPIO capture FAILED (see details)")
                self._log(f"GPIO capture failed: session={session} msg={message}")
                self.cap0.setText(message[:800])
                self.cap1.setText(message[:800])
                self.capture_details.setPlainText(
                    "\n".join(
                        [
                            f"source: gpio",
                            f"session: {session}",
                            f"stamp: {sec}.{nsec:09d}",
                            "",
                            message,
                        ]
                    )
                )

    def _refresh_gnss(self) -> None:
        fix, time_ref, imu, fix_rx, time_rx, imu_rx = self.gnss.snapshot()
        now_m = time.monotonic()

        if fix is None:
            self.gnss_status.setText("GNSS: waiting for NavSatFix...")
            self.gnss_fix_age.setText("Fix age: —")
            self.gnss_fix_stamp.setText("Fix stamp: —")
            self.gnss_latlon.setText("Lat/Lon: —")
            self.gnss_alt.setText("Alt: —")
            self.gnss_cov.setText("Covariance: —")
            self.gnss_fix_meta.setText("Status: —")
        else:
            age_ms = (now_m - fix_rx) * 1000.0 if fix_rx is not None else 1e9
            self.gnss_status.setText("GNSS: receiving")
            self.gnss_fix_age.setText(f"Fix age: {age_ms:.0f} ms")
            self.gnss_fix_stamp.setText(f"Fix stamp: {_fmt_stamp(fix.header.stamp)}")
            self.gnss_latlon.setText(f"Lat/Lon: {fix.latitude:.8f}, {fix.longitude:.8f}")
            self.gnss_alt.setText(f"Alt: {fix.altitude:.3f} m")
            cov = fix.position_covariance
            self.gnss_cov.setText(
                f"Covariance diag: [{cov[0]:.4f}, {cov[4]:.4f}, {cov[8]:.4f}] type={int(fix.position_covariance_type)}"
            )
            self.gnss_fix_meta.setText(f"Status: status={int(fix.status.status)} service={int(fix.status.service)}")

        if time_ref is None:
            self.gnss_time_ref.setText("TimeRef stamp: —")
            self.gnss_time_ref_src.setText("TimeRef source: —")
            self.gnss_time_ref_age.setText("TimeRef age: —")
        else:
            age_ms = (now_m - time_rx) * 1000.0 if time_rx is not None else 1e9
            self.gnss_time_ref.setText(
                f"TimeRef stamp: ros={_fmt_stamp(time_ref.header.stamp)} ref={_fmt_stamp(time_ref.time_ref)}"
            )
            self.gnss_time_ref_src.setText(f"TimeRef source: {time_ref.source or '—'}")
            self.gnss_time_ref_age.setText(f"TimeRef age: {age_ms:.0f} ms")

        if imu is None:
            self.imu_stamp.setText("IMU stamp: —")
            self.imu_vals.setText("IMU ang vel / lin acc: —")
            self.imu_age.setText("IMU age: —")
        else:
            age_ms = (now_m - imu_rx) * 1000.0 if imu_rx is not None else 1e9
            self.imu_stamp.setText(f"IMU stamp: {_fmt_stamp(imu.header.stamp)}")
            self.imu_vals.setText(
                "IMU ang vel [rad/s]: "
                f"{imu.angular_velocity.x:.4f}, {imu.angular_velocity.y:.4f}, {imu.angular_velocity.z:.4f} | "
                "lin acc [m/s^2]: "
                f"{imu.linear_acceleration.x:.4f}, {imu.linear_acceleration.y:.4f}, {imu.linear_acceleration.z:.4f}"
            )
            self.imu_age.setText(f"IMU age: {age_ms:.0f} ms")

    def _render_preview(self, label: QLabel, info: QLabel, sub: ImageSub, name: str):
        if not sub.got_first_frame():
            label.setText(f"{name}: waiting…")
            info.setText("—")
            return

        frame, enc, msg_ref = sub.get_latest_snapshot()
        if frame is None:
            label.setText(f"{name}: waiting…")
            info.setText("—")
            return

        age_s, fps = sub.stream_stats()
        info.setText(f"{fps:4.1f} FPS   |   age {age_s*1000:4.0f} ms")

        # Keep per-frame Python work minimal; do scaling in Qt/C++.
        pix = frame_to_pix(frame, enc)
        _ = msg_ref  # keep backing ROS message alive until pixmap conversion is done
        if not pix.isNull():
            target_w = max(2, label.width())
            target_h = max(2, label.height())
            if pix.width() != target_w or pix.height() != target_h:
                pix = pix.scaled(target_w, target_h, Qt.KeepAspectRatioByExpanding, Qt.FastTransformation)
        label.setPixmap(pix)

    def _update_indicators(self) -> None:
        def set_ind(key: str, ind: QLabel, ok: bool, warn: bool = False, paused: bool = False):
            if paused:
                state = "paused"
                style = "color:#74A8FF; font-weight:700;"
            elif ok:
                state = "ok"
                style = "color:#52D273; font-weight:700;"
            elif warn:
                state = "warn"
                style = "color:#F3C969; font-weight:700;"
            else:
                state = "bad"
                style = "color:#FF6B6B; font-weight:700;"
            if self._ind_state[key] != state:
                ind.setStyleSheet(style)
                self._ind_state[key] = state

        age0, _ = self.cam0.stream_stats()
        age1, _ = self.cam1.stream_stats()
        paused = self._preview_paused
        set_ind("cam0", self.ind_cam0, ok=self.cam0.got_first_frame() and age0 < 0.7, warn=self.cam0.got_first_frame(), paused=paused)
        set_ind("cam1", self.ind_cam1, ok=self.cam1.got_first_frame() and age1 < 0.7, warn=self.cam1.got_first_frame(), paused=paused)
        set_ind("srv", self.ind_srv, ok=self.ros_node.service_ready(), warn=False)

    def _apply_capture_pixmaps(self):
        def setpix(label: QLabel, pix: Optional[QPixmap]):
            if not pix:
                return
            label.setPixmap(pix.scaled(label.size(), Qt.KeepAspectRatio, Qt.FastTransformation))

        setpix(self.cap0, self._cap0_pix)
        setpix(self.cap1, self._cap1_pix)
        setpix(self.res0, self._res0_pix)
        setpix(self.res1, self._res1_pix)

    def on_save_settings(self):
        self.out_dir = self.out_dir_edit.text().strip() or self.out_dir
        self.jpeg_quality = int(self.quality_spin.value())
        self.ui_fps = max(1, int(self.ui_fps_spin.value()))
        self.preview_fps = max(1, int(self.preview_fps_spin.value()))
        os.makedirs(self.out_dir, exist_ok=True)

        cfg = {
            "output_dir": self.out_dir,
            "jpeg_quality": self.jpeg_quality,
            "ui_fps": self.ui_fps,
            "preview_fps": self.preview_fps,
            "capture_node": str(self.ros_node.get_parameter("capture_node").value),
            "cam0_topic": self.cam0_topic_edit.text().strip(),
            "cam1_topic": self.cam1_topic_edit.text().strip(),
            "capture_service": self.srv_name_edit.text().strip(),
            "gnss_fix_topic": self.gnss_fix_topic_edit.text().strip(),
            "gnss_time_ref_topic": self.gnss_time_ref_topic_edit.text().strip(),
            "gnss_imu_topic": self.gnss_imu_topic_edit.text().strip(),
        }
        save_config(cfg)
        self.status.setText("Status: settings saved")
        self._log("Settings saved")

        # Apply UI FPS immediately
        self.timer.setInterval(max(10, int(1000 / self.ui_fps)))
        # Also update mock publisher FPS when running in desktop simulation.
        fut = self.ros_node.set_mock_camera_fps_async(self.ui_fps)
        if fut is not None:
            def _done(f):
                try:
                    r = f.result()
                    if r and bool(r[0].successful):
                        self._log(f"Mock camera fps updated to {self.ui_fps}")
                except Exception:
                    self._log("Mock camera fps update failed")
            fut.add_done_callback(_done)

        # Update real preview producer FPS (capture_service managed previews).
        cap_fut = self.ros_node.set_capture_preview_fps_async(self.preview_fps)
        if cap_fut is not None:
            def _done_cap(f):
                try:
                    r = f.result()
                    if r and bool(r[0].successful):
                        self._log(f"Capture preview fps updated to {self.preview_fps}")
                    else:
                        reason = ""
                        if r and len(r) > 0:
                            reason = str(getattr(r[0], "reason", ""))
                        self._log(f"Capture preview fps update rejected: {reason or 'unknown reason'}")
                except Exception as e:
                    self._log(f"Capture preview fps update failed: {e}")

            cap_fut.add_done_callback(_done_cap)
        else:
            self._log("Capture preview fps update skipped (capture node parameter service not ready)")

    def _log(self, msg: str) -> None:
        line = f"[{_ts()}] {msg}"
        try:
            self.log_box.appendPlainText(line)
        except Exception:
            pass


def main():
    _single_instance_lock()

    os.environ.setdefault("QT_AUTO_SCREEN_SCALE_FACTOR", "1")
    os.environ.setdefault("QT_ENABLE_HIGHDPI_SCALING", "1")

    ros_argv = sys.argv
    qt_argv = remove_ros_args(sys.argv)

    rclpy.init(args=ros_argv)
    app_node = AppNode()
    cam0 = ImageSub("cam0_preview_sub", str(app_node.get_parameter("cam0_topic").value))
    cam1 = ImageSub("cam1_preview_sub", str(app_node.get_parameter("cam1_topic").value))
    gnss = GnssSub(
        "gnss_sub",
        str(app_node.get_parameter("gnss_fix_topic").value),
        str(app_node.get_parameter("gnss_time_ref_topic").value),
        str(app_node.get_parameter("gnss_imu_topic").value),
    )

    app = QApplication(qt_argv)
    apply_dark_theme(app)

    # Spin ROS in a background thread.
    # This keeps camera callbacks off the Qt UI thread, improving FPS stability.
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(app_node)
    executor.add_node(cam0)
    executor.add_node(cam1)
    executor.add_node(gnss)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    w = MainWindow(app_node, cam0, cam1, gnss)
    if os.environ.get("SUBSEA_UI_FULLSCREEN", "0") == "1":
        w.setWindowState(w.windowState() | Qt.WindowFullScreen)
    w.show()
    ret = app.exec()

    executor.shutdown()
    try:
        spin_thread.join(timeout=1.0)
    except Exception:
        pass
    app_node.destroy_node()
    cam0.destroy_node()
    cam1.destroy_node()
    gnss.destroy_node()
    rclpy.try_shutdown()
    return int(ret)


if __name__ == "__main__":
    raise SystemExit(main())
