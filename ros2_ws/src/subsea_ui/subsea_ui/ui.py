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
import signal
import subprocess
import sys
import threading
import time
import math
from collections import deque
from datetime import datetime
from pathlib import Path
from typing import Any, Deque, Dict, List, Optional, Tuple

import cv2
import numpy as np

from action_msgs.msg import GoalStatus
from qtpy.QtCore import Qt, QTimer
from qtpy.QtGui import QImage, QPixmap
from qtpy.QtWidgets import (
    QApplication,
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPlainTextEdit,
    QProgressBar,
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
    # fromImage() performs the conversion/copy into a pixmap immediately.
    qimg = QImage(frame.data, w, h, frame.strides[0], fmt)
    return QPixmap.fromImage(qimg)


def load_jpeg_as_pix(path: str) -> Optional[QPixmap]:
    if not path or not os.path.exists(path):
        return None
    # Quality-first display for Last Capture / Deblurred tabs.
    bgr = cv2.imread(path, cv2.IMREAD_COLOR)
    if bgr is None:
        return None
    return frame_to_pix(bgr, "bgr8")


def _ts() -> str:
    return datetime.now().strftime("%H:%M:%S")


def _utc_ts() -> str:
    return datetime.utcnow().strftime("%Y-%m-%dT%H:%M:%SZ")


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

        # GNSS publishers vary in QoS (gpsd_client uses RELIABLE, many drivers use
        # BEST_EFFORT). Use BEST_EFFORT here so the UI can display data from both.
        qos_fix = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        qos_best_effort = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._fix_sub = self.create_subscription(NavSatFix, fix_topic, self._on_fix, qos_fix)
        self._time_ref_sub = self.create_subscription(
            TimeReference,
            time_ref_topic,
            self._on_time_ref,
            qos_fix,
        )
        self._imu_sub = self.create_subscription(Imu, imu_topic, self._on_imu, qos_best_effort)
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
        self.declare_parameter("cam0_topic", cfg.get("cam0_topic", "/cam0/preview/image_raw"))
        self.declare_parameter("cam1_topic", cfg.get("cam1_topic", "/cam1/preview/image_raw"))
        self.declare_parameter("capture_service", cfg.get("capture_service", "capture_pair"))
        self.declare_parameter("capture_action", cfg.get("capture_action", "capture_pair"))
        self.declare_parameter("prefer_capture_action", bool(cfg.get("prefer_capture_action", True)))
        self.declare_parameter("capture_node", cfg.get("capture_node", "/capture_service"))
        self.declare_parameter("capture_event_topic", cfg.get("capture_event_topic", "/capture/events"))
        self.declare_parameter("capture_debug_topic", cfg.get("capture_debug_topic", "/capture/debug"))
        self.declare_parameter("mock_camera_node", cfg.get("mock_camera_node", "/mock_camera_publisher"))
        self.declare_parameter("gnss_fix_topic", cfg.get("gnss_fix_topic", "/fix"))
        self.declare_parameter("gnss_time_ref_topic", cfg.get("gnss_time_ref_topic", "/time_reference"))
        self.declare_parameter("gnss_imu_topic", cfg.get("gnss_imu_topic", "/imu/data"))
        self.declare_parameter("output_dir", cfg.get("output_dir", os.path.expanduser("~/captures")))
        self.declare_parameter("jpeg_quality", int(cfg.get("jpeg_quality", 95)))
        self.declare_parameter("ui_fps", int(cfg.get("ui_fps", 15)))
        self.declare_parameter("preview_fps", int(cfg.get("preview_fps", 15)))
        self.declare_parameter("preview_relay_fps", int(cfg.get("preview_relay_fps", 10)))
        self.declare_parameter(
            "require_gnss_lock_for_session",
            bool(cfg.get("require_gnss_lock_for_session", True)),
        )
        self.declare_parameter(
            "max_fix_age_ms_for_lock",
            int(cfg.get("max_fix_age_ms_for_lock", 2000)),
        )
        self.declare_parameter(
            "session_bag_topics",
            cfg.get(
                "session_bag_topics",
                "/imu/data /fix /time_reference /odometry/local /odometry/global /capture/events /capture/debug",
            ),
        )
        self.declare_parameter("session_record_images", bool(cfg.get("session_record_images", False)))
        self.declare_parameter("session_cam0_topic", cfg.get("session_cam0_topic", "/cam0/camera/image_raw"))
        self.declare_parameter("session_cam1_topic", cfg.get("session_cam1_topic", "/cam1/camera/image_raw"))

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
        self._capture_debug_lock = threading.Lock()
        self._capture_debug_events: Deque[Dict[str, Any]] = deque(maxlen=128)
        evt_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        evt_topic = str(self.get_parameter("capture_event_topic").value)
        self._capture_evt_sub = self.create_subscription(String, evt_topic, self._on_capture_event, evt_qos)
        self.get_logger().info(f"Capture event sub: {evt_topic}")
        dbg_topic = str(self.get_parameter("capture_debug_topic").value)
        self._capture_dbg_sub = self.create_subscription(String, dbg_topic, self._on_capture_debug, evt_qos)
        self.get_logger().info(f"Capture debug sub: {dbg_topic}")

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

    def _on_capture_debug(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except Exception:
            return
        if not isinstance(payload, dict):
            return
        with self._capture_debug_lock:
            self._capture_debug_events.append(payload)

    def pop_capture_debug_events(self) -> List[Dict[str, Any]]:
        with self._capture_debug_lock:
            if not self._capture_debug_events:
                return []
            out = list(self._capture_debug_events)
            self._capture_debug_events.clear()
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

    def set_capture_preview_relay_fps_async(self, fps: int):
        # Best-effort: if capture node parameter service is unavailable, continue silently.
        try:
            ready = self._capture_params.services_are_ready()
        except Exception:
            ready = False
        if not ready:
            return None
        params = [Parameter("preview_relay_fps", Parameter.Type.INTEGER, int(max(1, fps)))]
        return self._capture_params.set_parameters(params)

    def session_topics(self) -> List[str]:
        raw = str(self.get_parameter("session_bag_topics").value or "").strip()
        topics = [t.strip() for t in raw.split() if t.strip()]
        if bool(self.get_parameter("session_record_images").value):
            cam0_topic = str(self.get_parameter("session_cam0_topic").value or "").strip()
            cam1_topic = str(self.get_parameter("session_cam1_topic").value or "").strip()
            if cam0_topic:
                topics.append(cam0_topic)
            if cam1_topic:
                topics.append(cam1_topic)
        out: List[str] = []
        seen = set()
        for t in topics:
            topic = t if t.startswith("/") else f"/{t}"
            if topic in seen:
                continue
            seen.add(topic)
            out.append(topic)
        return out


class MainWindow(QWidget):
    def __init__(self, ros_node: AppNode, cam0: ImageSub, cam1: ImageSub, gnss: GnssSub):
        super().__init__()
        self.ros_node = ros_node
        self.cam0 = cam0
        self.cam1 = cam1
        self.gnss = gnss

        self.setWindowTitle("Rover App")

        self.tabs = QTabWidget()

        # --- Connection / top status
        self.ind_cam0 = QLabel("● Cam0")
        self.ind_cam1 = QLabel("● Cam1")
        self.ind_srv = QLabel("● Capture service")
        self.ind_gnss_lock = QLabel("● GNSS Lock")
        self.ind_corr_link = QLabel("● Corrections")
        for ind in (self.ind_cam0, self.ind_cam1, self.ind_srv):
            ind.setStyleSheet("font-weight:600;")
        self.ind_gnss_lock.setStyleSheet("color:#F3C969; font-weight:700;")
        self.ind_corr_link.setStyleSheet("color:#F3C969; font-weight:700;")

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
        self.capture_details.setLineWrapMode(QPlainTextEdit.NoWrap)
        self.capture_details.setStyleSheet("font-family: monospace; font-size:14px;")
        self.capture_log = QPlainTextEdit()
        self.capture_log.setReadOnly(True)
        self.capture_log.setMaximumBlockCount(2000)
        self.capture_log.setLineWrapMode(QPlainTextEdit.NoWrap)
        self.capture_log.setStyleSheet("font-family: monospace; font-size:14px;")
        self.capture_debug = QPlainTextEdit()
        self.capture_debug.setReadOnly(True)
        self.capture_debug.setMaximumBlockCount(800)
        self.capture_debug.setLineWrapMode(QPlainTextEdit.NoWrap)
        self.capture_debug.setStyleSheet("font-family: monospace; font-size:14px;")

        for lab in (self.prev0, self.prev1, self.cap0, self.cap1, self.res0, self.res1):
            lab.setAlignment(Qt.AlignCenter)
            lab.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
            lab.setStyleSheet("background-color:black; color:white; font-size:16px; border-radius:12px;")
            lab.setMinimumSize(140, 90)
        for info in (self.prev0_info, self.prev1_info):
            info.setStyleSheet("color:#B0B0B0; padding:4px 2px;")

        # --- Actions
        self.full_btn = QPushButton("Toggle Fullscreen")
        self.full_btn.setMinimumHeight(34)
        self.full_btn.setStyleSheet("font-size:14px; padding:4px 10px;")
        self.full_btn.clicked.connect(self.toggle_fullscreen)

        self.session_btn = QPushButton("Start Session")
        self.session_btn.setMinimumHeight(34)
        self.session_btn.setStyleSheet("font-size:14px; padding:4px 10px;")
        self.session_btn.clicked.connect(self._on_session_toggle_clicked)

        self.quit_btn = QPushButton("Quit")
        self.quit_btn.setMinimumHeight(34)
        self.quit_btn.setStyleSheet("font-size:14px; padding:4px 10px;")
        self.quit_btn.clicked.connect(self.close)

        self.session_status = QLabel("Session: idle")
        self.session_status.setStyleSheet("font-size:14px; color:#B0B0B0;")

        self._session_active = False
        self._session_id: Optional[str] = None
        self._session_dir: Optional[str] = None
        self._session_bag_dir: Optional[str] = None
        self._session_manifest_path: Optional[str] = None
        self._session_start_mono: Optional[float] = None
        self._session_start_utc: Optional[str] = None
        self._session_bag_proc: Optional[subprocess.Popen] = None
        self._session_bag_log_fp = None
        self._gnss_locked = False
        self._gnss_lock_reason = "waiting for NavSatFix"
        self._corr_active = False
        self._corr_reason = "waiting for NavSatFix"
        self._diag_collect_running = False
        self._diag_collect_result_lock = threading.Lock()
        self._diag_collect_result: Optional[Tuple[int, str, float]] = None
        self._diag_collect_live_lock = threading.Lock()
        self._diag_collect_live_lines: Deque[str] = deque(maxlen=256)
        self._diag_collect_started_mono: Optional[float] = None
        self._diag_collect_spin_idx: int = 0
        self._require_gnss_lock_for_session = bool(
            self.ros_node.get_parameter("require_gnss_lock_for_session").value
        )
        self._max_fix_age_ms_for_lock = int(self.ros_node.get_parameter("max_fix_age_ms_for_lock").value)
        self._session_started_with_gnss_lock = False
        self._session_gnss_lock_reason = ""

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
        self._last_capture_debug_key: Optional[str] = None
        self._preview_render_cache: Dict[str, Tuple[int, int, int]] = {}

        # User-settable values (persisted)
        self.out_dir = str(self.ros_node.get_parameter("output_dir").value)
        self.jpeg_quality = int(self.ros_node.get_parameter("jpeg_quality").value)
        self.ui_fps = max(1, int(self.ros_node.get_parameter("ui_fps").value))
        self.preview_fps = max(1, int(self.ros_node.get_parameter("preview_fps").value))
        self.preview_relay_fps = max(1, int(self.ros_node.get_parameter("preview_relay_fps").value))

        # ---- Preview tab
        # Keep indicators readable on small screens by splitting into two rows.
        indicator_grid = QGridLayout()
        indicator_grid.setHorizontalSpacing(10)
        indicator_grid.setVerticalSpacing(4)
        indicator_grid.addWidget(self.ind_cam0, 0, 0)
        indicator_grid.addWidget(self.ind_cam1, 0, 1)
        indicator_grid.addWidget(self.ind_srv, 0, 2)
        indicator_grid.addWidget(self.ind_gnss_lock, 1, 0)
        indicator_grid.addWidget(self.ind_corr_link, 1, 1)
        indicator_grid.setColumnStretch(3, 1)

        controls_row = QHBoxLayout()
        controls_row.setSpacing(10)
        controls_row.addWidget(self.session_status)
        controls_row.addStretch(1)
        controls_row.addWidget(self.session_btn, 0, Qt.AlignRight)
        controls_row.addWidget(self.full_btn, 0, Qt.AlignRight)
        controls_row.addWidget(self.quit_btn, 0, Qt.AlignRight)

        top_block = QVBoxLayout()
        top_block.setSpacing(6)
        top_block.addLayout(indicator_grid)
        top_block.addLayout(controls_row)

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
        preview_root.addLayout(top_block, 0)
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
        gnss_root.setSpacing(10)

        self.gnss_status = QLabel("GNSS: waiting…")
        self.gnss_status.setStyleSheet("font-size:18px; font-weight:700;")
        self.gnss_ready = QLabel("Ready to Log: waiting…")
        self.gnss_ready.setStyleSheet("font-size:20px; font-weight:800; color:#F3C969;")
        self.gnss_fix_type = QLabel("Fix type: —")
        self.gnss_pos_acc = QLabel("Estimated accuracy: —")
        self.gnss_freshness = QLabel("Data freshness: —")
        self.gnss_fix_age = QLabel("Fix age: —")
        self.gnss_fix_stamp = QLabel("Fix stamp: —")
        self.gnss_latlon = QLabel("Lat/Lon: —")
        self.gnss_alt = QLabel("Alt: —")
        self.gnss_cov = QLabel("Covariance: —")
        self.gnss_fix_meta = QLabel("Status: —")
        self.gnss_corr = QLabel("Corrections: —")

        self.gnss_time_ref = QLabel("TimeRef stamp: —")
        self.gnss_time_ref_src = QLabel("TimeRef source: —")
        self.gnss_time_ref_age = QLabel("TimeRef age: —")

        self.imu_stamp = QLabel("IMU stamp: —")
        self.imu_vals = QLabel("IMU ang vel / lin acc: —")
        self.imu_age = QLabel("IMU age: —")

        self.gnss_quality = QProgressBar()
        self.gnss_quality.setRange(0, 100)
        self.gnss_quality.setValue(0)
        self.gnss_quality.setFormat("GNSS quality: %p%")
        self.gnss_quality.setTextVisible(True)
        self.gnss_quality.setMinimumHeight(22)
        self.gnss_quality.setStyleSheet(
            "QProgressBar { border:1px solid #2B2B2B; border-radius:8px; text-align:center; background:#151515; } "
            "QProgressBar::chunk { border-radius:8px; background:#6B6B6B; }"
        )

        for l in (
            self.gnss_fix_age,
            self.gnss_fix_stamp,
            self.gnss_latlon,
            self.gnss_alt,
            self.gnss_cov,
            self.gnss_fix_meta,
            self.gnss_corr,
            self.gnss_fix_type,
            self.gnss_pos_acc,
            self.gnss_freshness,
            self.gnss_time_ref,
            self.gnss_time_ref_src,
            self.gnss_time_ref_age,
            self.imu_stamp,
            self.imu_vals,
            self.imu_age,
        ):
            l.setStyleSheet("font-size:15px;")
        self.imu_vals.setStyleSheet("font-size:14px;")

        self.imu_vals.setWordWrap(True)
        self.gnss_time_ref.setWordWrap(True)
        self.gnss_freshness.setWordWrap(True)
        self.gnss_pos_acc.setWordWrap(True)

        self._gnss_card_solution = self._make_gnss_card(
            "Solution", self.gnss_status, self.gnss_fix_type, self.gnss_fix_meta, self.gnss_corr
        )
        self._gnss_card_accuracy = self._make_gnss_card("Accuracy", self.gnss_pos_acc, self.gnss_cov)
        self._gnss_card_position = self._make_gnss_card("Position", self.gnss_latlon, self.gnss_alt, self.gnss_fix_stamp)
        self._gnss_card_timing = self._make_gnss_card(
            "Timing", self.gnss_freshness, self.gnss_fix_age, self.gnss_time_ref_age, self.gnss_time_ref_src
        )
        self._gnss_card_imu = self._make_gnss_card("IMU", self.imu_stamp, self.imu_age, self.imu_vals)
        self._gnss_cards = [
            self._gnss_card_solution,
            self._gnss_card_accuracy,
            self._gnss_card_position,
            self._gnss_card_timing,
            self._gnss_card_imu,
        ]

        self._gnss_grid = QGridLayout()
        self._gnss_grid.setSpacing(10)
        self._reflow_gnss_cards(1200)

        gnss_root.addWidget(self.gnss_ready)
        gnss_root.addWidget(self.gnss_quality)
        gnss_root.addLayout(self._gnss_grid)
        gnss_root.addWidget(self.gnss_time_ref)
        gnss_root.addStretch(1)
        gnss_tab.setLayout(gnss_root)
        gnss_scroll = QScrollArea()
        gnss_scroll.setWidgetResizable(True)
        gnss_scroll.setWidget(gnss_tab)
        self._tab_idx_gnss = self.tabs.addTab(gnss_scroll, "GNSS")

        # ---- Capture tab (nested tabs for larger image views)
        cap_page = QWidget()
        cap_row = QHBoxLayout()
        cap_row.setContentsMargins(10, 10, 10, 10)
        cap_row.setSpacing(10)
        cap_row.addWidget(self.cap0, 1)
        cap_row.addWidget(self.cap1, 1)
        cap_page.setLayout(cap_row)

        deblur_page = QWidget()
        res_row = QHBoxLayout()
        res_row.setContentsMargins(10, 10, 10, 10)
        res_row.setSpacing(10)
        res_row.addWidget(self.res0, 1)
        res_row.addWidget(self.res1, 1)
        deblur_page.setLayout(res_row)

        details_page = QWidget()
        details_layout = QVBoxLayout()
        details_layout.setContentsMargins(10, 10, 10, 10)
        details_layout.setSpacing(8)
        details_layout.addWidget(QLabel("Logs and debug"), 0)
        details_tabs = QTabWidget()
        details_tabs.addTab(self.capture_details, "Capture Details")
        details_tabs.addTab(self.capture_debug, "Debug")
        details_tabs.addTab(self.capture_log, "Event Log")
        details_layout.addWidget(details_tabs, 1)
        details_page.setLayout(details_layout)

        capture_tabs = QTabWidget()
        capture_tabs.addTab(cap_page, "Last Capture")
        capture_tabs.addTab(deblur_page, "Deblurred")
        capture_tabs.addTab(details_page, "Details / Log")

        capture_inner = QWidget()
        capture_layout = QVBoxLayout()
        capture_layout.setContentsMargins(10, 10, 10, 10)
        capture_layout.setSpacing(10)
        capture_layout.addWidget(capture_tabs, 1)
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
        self.preview_relay_fps_spin = QSpinBox()
        self.preview_relay_fps_spin.setRange(1, 60)
        self.preview_relay_fps_spin.setValue(self.preview_relay_fps)
        self.preview_relay_fps_spin.setSingleStep(1)

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
        preview_note = QLabel(
            "Capture stream FPS affects capture quality/timing. Relay FPS and UI FPS cap visible preview "
            "(effective on-screen FPS is roughly min(stream, relay, UI))."
        )
        preview_note.setStyleSheet("color:#B0B0B0;")
        preview_note.setWordWrap(True)
        prev_root.addWidget(preview_note)
        prev_root.addLayout(row("Preview UI FPS", self.ui_fps_spin))
        prev_root.addLayout(row("Capture stream FPS", self.preview_fps_spin))
        prev_root.addLayout(row("Preview relay FPS", self.preview_relay_fps_spin))
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
        self.diag_overall = QLabel("Field Ready: —")
        self.diag_overall.setStyleSheet("font-size:16px; font-weight:700; color:#B0B0B0;")
        self.diag_details = QPlainTextEdit()
        self.diag_details.setReadOnly(True)
        self.diag_details.setMaximumBlockCount(300)
        self.diag_details.setLineWrapMode(QPlainTextEdit.NoWrap)
        self.diag_details.setStyleSheet("font-family: monospace; font-size:13px;")
        self.diag_details.setMaximumHeight(170)
        self.log_box = QPlainTextEdit()
        self.log_box.setReadOnly(True)
        self.log_box.setMaximumBlockCount(5000)
        self.diag_collect_status = QLabel("Diagnostics bundle: idle")
        self.diag_collect_status.setStyleSheet("font-size:14px; color:#B0B0B0;")
        self.diag_collect_stage = QLabel("Diagnostics step: idle")
        self.diag_collect_stage.setStyleSheet("font-size:13px; color:#8A8A8A;")
        self.diag_collect_progress = QProgressBar()
        self.diag_collect_progress.setRange(0, 1)
        self.diag_collect_progress.setValue(0)
        self.diag_collect_progress.setTextVisible(False)
        self.diag_collect_progress.setMaximumHeight(8)
        self.diag_collect_progress.hide()
        self.diag_collect_btn = QPushButton("Collect Diagnostics Bundle")
        self.diag_collect_btn.clicked.connect(self.on_collect_diagnostics)
        self.diag_collect_btn.setMinimumHeight(34)
        self.diag_collect_btn.setStyleSheet("font-size:14px; padding:4px 10px;")
        clear_btn = QPushButton("Clear")
        clear_btn.clicked.connect(lambda: self.log_box.setPlainText(""))
        controls = QHBoxLayout()
        controls.setSpacing(10)
        controls.addWidget(self.diag_collect_btn, 0)
        controls.addWidget(self.diag_collect_status, 1)
        controls.addWidget(clear_btn, 0)
        droot.addWidget(self.diag_overall, 0)
        droot.addWidget(self.diag_details, 0)
        droot.addWidget(self.log_box, 1)
        droot.addWidget(self.diag_collect_stage, 0)
        droot.addWidget(self.diag_collect_progress, 0)
        droot.addLayout(controls, 0)
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
        self.session_btn.setMinimumHeight(30)
        self.session_btn.setStyleSheet("font-size:13px; padding:2px 8px;")
        self.full_btn.setMinimumHeight(30)
        self.full_btn.setStyleSheet("font-size:13px; padding:2px 8px;")
        self.quit_btn.setMinimumHeight(30)
        self.quit_btn.setStyleSheet("font-size:13px; padding:2px 8px;")
        for lab in (self.prev0, self.prev1, self.cap0, self.cap1, self.res0, self.res1):
            lab.setMinimumSize(110, 70)

    def _make_gnss_card(self, title: str, *widgets: QWidget) -> QFrame:
        box = QFrame()
        box.setFrameShape(QFrame.StyledPanel)
        box.setStyleSheet(
            "QFrame { background:#171717; border:1px solid #2E2E2E; border-radius:10px; }"
        )
        lay = QVBoxLayout()
        lay.setContentsMargins(10, 10, 10, 10)
        lay.setSpacing(6)
        ttl = QLabel(title)
        ttl.setStyleSheet("font-size:14px; font-weight:700; color:#D8D8D8;")
        lay.addWidget(ttl)
        for w in widgets:
            lay.addWidget(w)
        box.setLayout(lay)
        return box

    def _reflow_gnss_cards(self, width_px: int) -> None:
        # Keep cards readable on non-fullscreen/touch windows.
        if not hasattr(self, "_gnss_grid"):
            return
        cols = 1 if int(width_px) < 1200 else 2
        while self._gnss_grid.count() > 0:
            self._gnss_grid.takeAt(0)

        if cols == 1:
            for row, card in enumerate(self._gnss_cards):
                self._gnss_grid.addWidget(card, row, 0)
            self._gnss_grid.setColumnStretch(0, 1)
            return

        self._gnss_grid.addWidget(self._gnss_card_solution, 0, 0)
        self._gnss_grid.addWidget(self._gnss_card_accuracy, 0, 1)
        self._gnss_grid.addWidget(self._gnss_card_position, 1, 0)
        self._gnss_grid.addWidget(self._gnss_card_timing, 1, 1)
        self._gnss_grid.addWidget(self._gnss_card_imu, 2, 0, 1, 2)
        self._gnss_grid.setColumnStretch(0, 1)
        self._gnss_grid.setColumnStretch(1, 1)

    def toggle_fullscreen(self):
        if self.windowState() & Qt.WindowFullScreen:
            self.setWindowState(self.windowState() & ~Qt.WindowFullScreen)
        else:
            self.setWindowState(self.windowState() | Qt.WindowFullScreen)

    def closeEvent(self, e):
        if self._session_active:
            self._stop_session(reason="ui_closed")
        super().closeEvent(e)

    def resizeEvent(self, e):
        super().resizeEvent(e)
        self._reflow_gnss_cards(self.width())
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
        self._consume_capture_debug_events()
        self._refresh_gnss()
        self._refresh_session_status()
        self._refresh_diag_status()
        self._drain_diag_collect_live_lines()
        self._update_diag_collect_running_status()
        self._poll_diag_collect_result()

    def _refresh_diag_status(self) -> None:
        cam0_age_s, cam0_fps = self.cam0.stream_stats()
        cam1_age_s, cam1_fps = self.cam1.stream_stats()
        cam0_ok = self.cam0.got_first_frame() and cam0_age_s < 0.7
        cam1_ok = self.cam1.got_first_frame() and cam1_age_s < 0.7
        srv_ok = self.ros_node.service_ready()
        gnss_ok = bool(self._gnss_locked)
        corr_ok = bool(self._corr_active)

        if srv_ok and gnss_ok and corr_ok and cam0_ok and cam1_ok:
            overall_text = "Field Ready: OK"
            overall_style = "font-size:16px; font-weight:700; color:#52D273;"
        elif srv_ok and gnss_ok and cam0_ok and cam1_ok:
            overall_text = "Field Ready: WARN (no corrections)"
            overall_style = "font-size:16px; font-weight:700; color:#F3C969;"
        else:
            overall_text = "Field Ready: NOT OK"
            overall_style = "font-size:16px; font-weight:700; color:#FF6B6B;"

        self.diag_overall.setText(overall_text)
        self.diag_overall.setStyleSheet(overall_style)

        session_state = "RUNNING" if self._session_active else "IDLE"
        lines = [
            f"Capture service : {'OK' if srv_ok else 'NO'}",
            f"GNSS lock       : {'OK' if gnss_ok else 'NO'} ({self._gnss_lock_reason})",
            f"Corrections     : {'ON' if corr_ok else 'OFF'} ({self._corr_reason})",
            f"Cam0 stream     : {'OK' if cam0_ok else 'NO'} (age={cam0_age_s*1000.0:.0f} ms fps={cam0_fps:.1f})",
            f"Cam1 stream     : {'OK' if cam1_ok else 'NO'} (age={cam1_age_s*1000.0:.0f} ms fps={cam1_fps:.1f})",
            f"Session         : {session_state}",
        ]
        self.diag_details.setPlainText("\n".join(lines))

    def _resolve_diag_script_path(self) -> Optional[str]:
        # Preferred explicit override.
        env_path = os.environ.get("SUBSEA_DIAG_SCRIPT", "").strip()
        if env_path and os.path.isfile(env_path) and os.access(env_path, os.X_OK):
            return env_path

        candidates: List[Path] = []
        # Common deploy location on target Raspberry Pi.
        candidates.append(Path.home() / "trajectory-calculation-3d-scanner" / "scripts" / "collect_rover_diagnostics.sh")
        # Search parents of this module path and cwd.
        for base in [Path(__file__).resolve(), Path.cwd().resolve()]:
            p = base
            for _ in range(8):
                p = p.parent
                candidates.append(p / "scripts" / "collect_rover_diagnostics.sh")

        seen = set()
        for c in candidates:
            cs = str(c)
            if cs in seen:
                continue
            seen.add(cs)
            if os.path.isfile(cs) and os.access(cs, os.X_OK):
                return cs
        return None

    def on_collect_diagnostics(self) -> None:
        if self._diag_collect_running:
            self._log("Diagnostics bundle is already running")
            return
        script = self._resolve_diag_script_path()
        if not script:
            self.diag_collect_status.setText("Diagnostics bundle: script not found")
            self.diag_collect_status.setStyleSheet("font-size:14px; color:#FF6B6B;")
            self.diag_collect_stage.setText("Diagnostics step: script not found")
            self.diag_collect_stage.setStyleSheet("font-size:13px; color:#FF6B6B;")
            self._log("Diagnostics bundle failed: cannot locate collect_rover_diagnostics.sh")
            return

        self._diag_collect_running = True
        self._diag_collect_started_mono = time.monotonic()
        self._diag_collect_spin_idx = 0
        with self._diag_collect_live_lock:
            self._diag_collect_live_lines.clear()
        self.diag_collect_btn.setEnabled(False)
        self.diag_collect_btn.setText("Collecting...")
        self.diag_collect_status.setText("Diagnostics bundle: running...")
        self.diag_collect_status.setStyleSheet("font-size:14px; color:#F3C969; font-weight:700;")
        self.diag_collect_stage.setText("Diagnostics step: starting...")
        self.diag_collect_stage.setStyleSheet("font-size:13px; color:#F3C969;")
        self.diag_collect_progress.setRange(0, 0)  # indeterminate busy state
        self.diag_collect_progress.show()
        self._log(f"Diagnostics bundle started: {script}")

        def _worker() -> None:
            t0 = time.monotonic()
            rc = 1
            out_lines: List[str] = []
            try:
                proc = subprocess.Popen(
                    [script],
                    cwd=os.path.dirname(os.path.dirname(script)),
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    text=True,
                    bufsize=1,
                )
                if proc.stdout is not None:
                    for raw in proc.stdout:
                        line = raw.rstrip("\n")
                        out_lines.append(line)
                        with self._diag_collect_live_lock:
                            self._diag_collect_live_lines.append(line)
                rc = int(proc.wait())
            except Exception as e:
                out_lines.append(f"Diagnostics runner exception: {e}")
            out = "\n".join(out_lines)
            dur = max(0.0, time.monotonic() - t0)
            with self._diag_collect_result_lock:
                self._diag_collect_result = (rc, out, dur)

        threading.Thread(target=_worker, daemon=True).start()

    def _update_diag_collect_running_status(self) -> None:
        if not self._diag_collect_running:
            return
        start_m = self._diag_collect_started_mono or time.monotonic()
        elapsed = max(0.0, time.monotonic() - start_m)
        spinner = "|/-\\"
        ch = spinner[self._diag_collect_spin_idx % len(spinner)]
        self._diag_collect_spin_idx += 1
        self.diag_collect_status.setText(
            f"Diagnostics bundle: running {ch} {elapsed:.0f}s"
        )

    def _drain_diag_collect_live_lines(self) -> None:
        if not self._diag_collect_running:
            return
        lines: List[str] = []
        with self._diag_collect_live_lock:
            while self._diag_collect_live_lines:
                lines.append(self._diag_collect_live_lines.popleft())
        if not lines:
            return
        for line in lines:
            txt = line.strip()
            if not txt:
                continue
            if txt.startswith("[diag]"):
                step = txt[len("[diag]"):].strip()
                self.diag_collect_stage.setText(f"Diagnostics step: {step}")
            self._log(f"diag> {txt}")

    def _poll_diag_collect_result(self) -> None:
        if not self._diag_collect_running:
            return
        result = None
        with self._diag_collect_result_lock:
            if self._diag_collect_result is not None:
                result = self._diag_collect_result
                self._diag_collect_result = None
        if result is None:
            return

        rc, out, dur = result
        self._diag_collect_running = False
        self.diag_collect_btn.setEnabled(True)
        self.diag_collect_btn.setText("Collect Diagnostics Bundle")
        self.diag_collect_progress.setRange(0, 1)
        self.diag_collect_progress.setValue(0)
        self.diag_collect_progress.hide()
        self._diag_collect_started_mono = None

        archive_path = ""
        for line in out.splitlines():
            if line.strip().startswith("Diagnostics archive:"):
                archive_path = line.split(":", 1)[1].strip()
                break

        if rc == 0:
            self.diag_collect_status.setText("Diagnostics bundle: done")
            self.diag_collect_status.setStyleSheet("font-size:14px; color:#52D273; font-weight:700;")
            self.diag_collect_stage.setText("Diagnostics step: finished")
            self.diag_collect_stage.setStyleSheet("font-size:13px; color:#52D273;")
            if archive_path:
                self._log(f"Diagnostics bundle done in {dur:.1f}s: {archive_path}")
                self.status.setText(f"Status: diagnostics ready -> {archive_path}")
            else:
                self._log(f"Diagnostics bundle done in {dur:.1f}s")
                self.status.setText("Status: diagnostics ready")
        else:
            self.diag_collect_status.setText("Diagnostics bundle: failed")
            self.diag_collect_status.setStyleSheet("font-size:14px; color:#FF6B6B; font-weight:700;")
            self.diag_collect_stage.setText("Diagnostics step: failed")
            self.diag_collect_stage.setStyleSheet("font-size:13px; color:#FF6B6B;")
            self._log(f"Diagnostics bundle failed in {dur:.1f}s (exit={rc})")
            self.status.setText("Status: diagnostics bundle failed (see Diagnostics tab)")

        tail = [ln for ln in out.splitlines() if ln.strip()]
        if tail:
            self._log("Diagnostics output tail:")
            for ln in tail[-8:]:
                self._log(f"  {ln}")

    def _session_duration_s(self) -> float:
        if (not self._session_active) or (self._session_start_mono is None):
            return 0.0
        return max(0.0, time.monotonic() - self._session_start_mono)

    def _refresh_session_status(self) -> None:
        if not self._session_active:
            self.session_status.setText("Session: IDLE")
            self.session_status.setStyleSheet("font-size:14px; color:#B0B0B0;")
            return
        if self._session_bag_proc is not None:
            rc = self._session_bag_proc.poll()
            if rc is not None:
                self._log(f"Session recorder exited unexpectedly (code={rc})")
                self._stop_session(reason=f"process_exit_{rc}")
                return
        dur_s = int(self._session_duration_s())
        hh = dur_s // 3600
        mm = (dur_s % 3600) // 60
        ss = dur_s % 60
        self.session_status.setText(f"Session: RUNNING {hh:02d}:{mm:02d}:{ss:02d}")
        self.session_status.setStyleSheet("font-size:14px; color:#52D273; font-weight:700;")

    def _session_manifest_data(self, state: str, reason: str = "", return_code: Optional[int] = None) -> Dict[str, Any]:
        return {
            "schema_version": 1,
            "session_id": self._session_id,
            "state": state,
            "reason": reason,
            "start_utc": self._session_start_utc,
            "end_utc": _utc_ts() if state != "running" else None,
            "duration_s": self._session_duration_s() if state != "running" else None,
            "capture_output_dir": self.out_dir,
            "bag_dir": self._session_bag_dir,
            "topics": self.ros_node.session_topics(),
            "gnss_lock_required": bool(self._require_gnss_lock_for_session),
            "gnss_lock_at_start": bool(self._session_started_with_gnss_lock),
            "gnss_lock_reason_at_start": self._session_gnss_lock_reason,
            "corrections_active_at_start": bool(self._corr_active),
            "corrections_reason_at_start": self._corr_reason,
            "return_code": return_code,
        }

    def _write_session_manifest(self, data: Dict[str, Any]) -> None:
        if not self._session_manifest_path:
            return
        tmp = self._session_manifest_path + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2, sort_keys=True)
        os.replace(tmp, self._session_manifest_path)

    def _session_root_dir(self) -> str:
        return os.path.join(self.out_dir, "sessions")

    def _start_session(self) -> None:
        if self._session_active:
            return
        if self._require_gnss_lock_for_session and (not self._gnss_locked):
            self.status.setText("Status: session blocked (no GNSS lock)")
            self._log(f"Session start blocked: GNSS lock required ({self._gnss_lock_reason})")
            return
        topics = self.ros_node.session_topics()
        if not topics:
            self.status.setText("Status: session start failed (no topics configured)")
            self._log("Session start failed: no topics configured for bag recording")
            return

        os.makedirs(self._session_root_dir(), exist_ok=True)
        now = datetime.now()
        session_id = now.strftime("sess_%Y%m%d_%H%M%S")
        session_day_dir = now.strftime("%Y/%m/%d")
        session_dir = os.path.join(self._session_root_dir(), session_day_dir, session_id)
        bag_dir = os.path.join(session_dir, "bag")
        os.makedirs(session_dir, exist_ok=True)

        bag_log_path = os.path.join(session_dir, "rosbag_record.log")
        manifest_path = os.path.join(session_dir, "session_manifest.json")
        cmd = ["ros2", "bag", "record", "-o", bag_dir] + topics

        try:
            log_fp = open(bag_log_path, "w", encoding="utf-8")
        except Exception as e:
            self.status.setText("Status: session start failed (log file)")
            self._log(f"Session start failed: cannot open bag log file: {e}")
            return

        try:
            proc = subprocess.Popen(cmd, stdout=log_fp, stderr=subprocess.STDOUT)
        except Exception as e:
            try:
                log_fp.close()
            except Exception:
                pass
            self.status.setText("Status: session start failed (ros2 bag)")
            self._log(f"Session start failed: cannot launch ros2 bag record: {e}")
            return

        # Fast-fail check for obvious startup errors.
        time.sleep(0.2)
        rc = proc.poll()
        if rc is not None:
            try:
                log_fp.close()
            except Exception:
                pass
            self.status.setText("Status: session start failed (ros2 bag exited)")
            self._log(f"Session start failed: ros2 bag exited immediately (code={rc})")
            return

        self._session_active = True
        self._session_id = session_id
        self._session_dir = session_dir
        self._session_bag_dir = bag_dir
        self._session_manifest_path = manifest_path
        self._session_start_mono = time.monotonic()
        self._session_start_utc = _utc_ts()
        self._session_bag_proc = proc
        self._session_bag_log_fp = log_fp
        self._session_started_with_gnss_lock = bool(self._gnss_locked)
        self._session_gnss_lock_reason = str(self._gnss_lock_reason)
        self.session_btn.setText("Stop Session")
        self.session_btn.setStyleSheet("font-size:14px; padding:4px 10px; background:#3A1E1E;")
        self.status.setText(f"Status: session recording ({session_id})")
        self._log(f"Session started: id={session_id}")
        self._log(f"Bag topics: {' '.join(topics)}")
        self._log(f"Bag dir: {bag_dir}")
        self._write_session_manifest(self._session_manifest_data(state="running"))

    def _stop_session(self, reason: str = "user_stop") -> None:
        if not self._session_active:
            return
        proc = self._session_bag_proc
        rc: Optional[int] = None
        if proc is not None:
            try:
                if proc.poll() is None:
                    proc.send_signal(signal.SIGINT)
                    try:
                        proc.wait(timeout=12.0)
                    except subprocess.TimeoutExpired:
                        proc.terminate()
                        try:
                            proc.wait(timeout=3.0)
                        except subprocess.TimeoutExpired:
                            proc.kill()
                            proc.wait(timeout=2.0)
                rc = proc.poll()
            except Exception as e:
                self._log(f"Session stop warning: {e}")
                try:
                    proc.kill()
                except Exception:
                    pass
                rc = proc.poll()

        if self._session_bag_log_fp is not None:
            try:
                self._session_bag_log_fp.flush()
                self._session_bag_log_fp.close()
            except Exception:
                pass

        self._write_session_manifest(self._session_manifest_data(state="stopped", reason=reason, return_code=rc))

        sid = self._session_id or "session"
        self._log(f"Session stopped: id={sid} reason={reason} return_code={rc}")
        if self._session_dir:
            self._log(f"Session files: {self._session_dir}")
        self.status.setText(f"Status: session stopped ({sid})")
        self.session_btn.setText("Start Session")
        self.session_btn.setStyleSheet("font-size:14px; padding:4px 10px;")

        self._session_active = False
        self._session_id = None
        self._session_dir = None
        self._session_bag_dir = None
        self._session_manifest_path = None
        self._session_start_mono = None
        self._session_start_utc = None
        self._session_bag_proc = None
        self._session_bag_log_fp = None
        self._session_started_with_gnss_lock = False
        self._session_gnss_lock_reason = ""

    def _on_session_toggle_clicked(self) -> None:
        if self._session_active:
            self._stop_session(reason="user_stop")
        else:
            self._start_session()

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
                self._append_capture_log(
                    source="gpio",
                    session=session,
                    success=True,
                    message=message,
                    cam0_path=cam0_path,
                    cam1_path=cam1_path,
                    sec=sec,
                    nsec=nsec,
                )
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
                self._append_capture_log(
                    source="gpio",
                    session=session,
                    success=False,
                    message=message,
                    cam0_path=cam0_path,
                    cam1_path=cam1_path,
                    sec=sec,
                    nsec=nsec,
                )
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

    def _append_capture_log(
        self,
        source: str,
        session: str,
        success: bool,
        message: str,
        cam0_path: str,
        cam1_path: str,
        sec: int,
        nsec: int,
    ) -> None:
        status = "OK" if success else "FAILED"
        summary = (
            f"[{_ts()}] {source.upper()} {status} "
            f"session={session} stamp={sec}.{nsec:09d}\n"
            f"cam0={cam0_path}\n"
            f"cam1={cam1_path}\n"
            f"msg={message}\n"
        )
        self.capture_log.appendPlainText(summary)

    def _consume_capture_debug_events(self) -> None:
        events = self.ros_node.pop_capture_debug_events()
        if not events:
            return

        for ev in events:
            session = str(ev.get("session_id", ""))
            status = str(ev.get("status", ""))
            trigger = str(ev.get("trigger_stamp", ""))
            msg = str(ev.get("message", ""))
            key = f"{session}|{status}|{trigger}|{msg}"
            if self._last_capture_debug_key == key:
                continue
            self._last_capture_debug_key = key

            try:
                pretty = json.dumps(ev, indent=2, sort_keys=True)
            except Exception:
                pretty = str(ev)
            self.capture_debug.setPlainText(pretty)

            pair_ms = ev.get("pair_delta_ms")
            cam0_age_ms = ev.get("cam0_age_ms")
            cam1_age_ms = ev.get("cam1_age_ms")
            self._log(
                "Capture debug: "
                f"status={status} session={session} "
                f"pair_ms={pair_ms} cam0_age_ms={cam0_age_ms} cam1_age_ms={cam1_age_ms}"
            )

    def _refresh_gnss(self) -> None:
        fix, time_ref, imu, fix_rx, time_rx, imu_rx = self.gnss.snapshot()
        now_m = time.monotonic()
        lock_state = "waiting"
        lock_reason = "waiting for NavSatFix"
        corr_state = "waiting"
        corr_reason = "waiting for NavSatFix"
        fix_age_ms: Optional[float] = None
        time_age_ms: Optional[float] = None
        imu_age_ms: Optional[float] = None
        status_code: Optional[int] = None
        service_code: Optional[int] = None

        if fix is None:
            self.gnss_status.setText("GNSS: waiting for NavSatFix...")
            self.gnss_fix_age.setText("Fix age: —")
            self.gnss_fix_stamp.setText("Fix stamp: —")
            self.gnss_latlon.setText("Lat/Lon: —")
            self.gnss_alt.setText("Alt: —")
            self.gnss_cov.setText("Covariance: —")
            self.gnss_fix_meta.setText("Status: —")
            self.gnss_corr.setText("Corrections: —")
            self.gnss_fix_type.setText("Fix type: —")
            self.gnss_pos_acc.setText("Estimated accuracy: —")
        else:
            age_ms = (now_m - fix_rx) * 1000.0 if fix_rx is not None else 1e9
            fix_age_ms = age_ms
            status_code = int(fix.status.status)
            service_code = int(fix.status.service)

            if age_ms > float(self._max_fix_age_ms_for_lock):
                lock_state = "unlocked"
                lock_reason = f"stale fix ({age_ms:.0f} ms)"
            elif status_code < 0:
                lock_state = "unlocked"
                lock_reason = "receiver reports NO_FIX"
            elif status_code == 0:
                lock_state = "locked"
                lock_reason = "FIX"
            elif status_code == 1:
                lock_state = "locked"
                lock_reason = "SBAS_FIX"
            elif status_code == 2:
                lock_state = "locked"
                lock_reason = "GBAS/RTK_FIX"
            else:
                lock_state = "locked"
                lock_reason = f"status={status_code}"

            if age_ms > float(self._max_fix_age_ms_for_lock):
                corr_state = "off"
                corr_reason = f"stale fix ({age_ms:.0f} ms)"
            elif status_code == 2:
                corr_state = "on"
                corr_reason = "RTK/DGPS corrections used"
            elif status_code in (0, 1):
                corr_state = "off"
                corr_reason = "no differential correction in solution"
            elif status_code < 0:
                corr_state = "off"
                corr_reason = "no GNSS fix"
            else:
                corr_state = "off"
                corr_reason = f"status={status_code}"

            if lock_state == "locked":
                self.gnss_status.setText(f"GNSS: lock acquired ({lock_reason})")
            else:
                self.gnss_status.setText(f"GNSS: no lock ({lock_reason})")
            self.gnss_fix_age.setText(f"Fix age: {age_ms:.0f} ms")
            fix_stamp = _fmt_stamp(fix.header.stamp)
            try:
                fix_utc = datetime.utcfromtimestamp(int(fix.header.stamp.sec)).strftime("%H:%M:%S")
                self.gnss_fix_stamp.setText(f"Fix stamp: {fix_stamp} (UTC {fix_utc})")
            except Exception:
                self.gnss_fix_stamp.setText(f"Fix stamp: {fix_stamp}")
            self.gnss_latlon.setText(f"Lat/Lon: {fix.latitude:.8f}, {fix.longitude:.8f}")
            self.gnss_alt.setText(f"Alt: {fix.altitude:.3f} m")
            cov = fix.position_covariance
            self.gnss_cov.setText(
                f"Covariance diag: [{cov[0]:.4f}, {cov[4]:.4f}, {cov[8]:.4f}] type={int(fix.position_covariance_type)}"
            )
            self.gnss_fix_meta.setText(f"Status: status={status_code} service={service_code}")
            self.gnss_corr.setText(f"Corrections: {corr_reason}")
            fix_type_map = {
                -1: "NO_FIX",
                0: "FIX",
                1: "SBAS_FIX",
                2: "GBAS/RTK_FIX",
            }
            fix_type = fix_type_map.get(status_code, f"UNKNOWN({status_code})")
            self.gnss_fix_type.setText(f"Fix type: {fix_type}")
            try:
                cov_x = max(0.0, float(cov[0]))
                cov_y = max(0.0, float(cov[4]))
                cov_z = max(0.0, float(cov[8]))
                sigma_h = math.sqrt(cov_x + cov_y)
                sigma_v = math.sqrt(cov_z)
                self.gnss_pos_acc.setText(
                    f"Estimated accuracy (1-sigma): horizontal≈{sigma_h:.3f} m, vertical≈{sigma_v:.3f} m"
                )
            except Exception:
                self.gnss_pos_acc.setText("Estimated accuracy: unavailable")

        if time_ref is None:
            self.gnss_time_ref.setText("TimeRef stamp: —")
            self.gnss_time_ref_src.setText("TimeRef source: —")
            self.gnss_time_ref_age.setText("TimeRef age: —")
        else:
            age_ms = (now_m - time_rx) * 1000.0 if time_rx is not None else 1e9
            time_age_ms = age_ms
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
            imu_age_ms = age_ms
            self.imu_stamp.setText(f"IMU stamp: {_fmt_stamp(imu.header.stamp)}")
            self.imu_vals.setText(
                "IMU ang vel [rad/s]: "
                f"{imu.angular_velocity.x:.4f}, {imu.angular_velocity.y:.4f}, {imu.angular_velocity.z:.4f} | "
                "lin acc [m/s^2]: "
                f"{imu.linear_acceleration.x:.4f}, {imu.linear_acceleration.y:.4f}, {imu.linear_acceleration.z:.4f}"
            )
            self.imu_age.setText(f"IMU age: {age_ms:.0f} ms")

        fresh_bits = []
        if fix_age_ms is None:
            fresh_bits.append("fix=missing")
        else:
            fresh_bits.append(f"fix={fix_age_ms:.0f} ms")
        if time_age_ms is None:
            fresh_bits.append("time_ref=missing")
        else:
            fresh_bits.append(f"time_ref={time_age_ms:.0f} ms")
        if imu_age_ms is None:
            fresh_bits.append("imu=missing")
        else:
            fresh_bits.append(f"imu={imu_age_ms:.0f} ms")
        self.gnss_freshness.setText("Data freshness: " + " | ".join(fresh_bits))

        # GNSS-only quality score (do not mix IMU state into this bar).
        score = 0
        if lock_state == "locked":
            score += 40
        if corr_state == "on":
            score += 25
        if fix_age_ms is not None:
            if fix_age_ms <= 500.0:
                score += 15
            elif fix_age_ms <= float(self._max_fix_age_ms_for_lock):
                score += 8
        try:
            if fix is not None:
                cov = fix.position_covariance
                cov_x = max(0.0, float(cov[0]))
                cov_y = max(0.0, float(cov[4]))
                sigma_h = math.sqrt(cov_x + cov_y)
                if sigma_h <= 0.03:
                    score += 10
                elif sigma_h <= 0.10:
                    score += 7
                elif sigma_h <= 0.50:
                    score += 3
        except Exception:
            pass

        score = max(0, min(100, int(score)))
        self.gnss_quality.setValue(score)
        if score >= 80:
            chunk = "#52D273"
        elif score >= 50:
            chunk = "#F3C969"
        else:
            chunk = "#FF6B6B"
        self.gnss_quality.setStyleSheet(
            "QProgressBar { border:1px solid #2B2B2B; border-radius:8px; text-align:center; background:#151515; } "
            f"QProgressBar::chunk {{ border-radius:8px; background:{chunk}; }}"
        )

        if lock_state == "locked" and corr_state == "on":
            self.gnss_ready.setText("Ready to Log: YES (RTK corrections)")
            self.gnss_ready.setStyleSheet("font-size:20px; font-weight:800; color:#52D273;")
        elif lock_state == "locked":
            self.gnss_ready.setText("Ready to Log: YES (GNSS lock, no corrections)")
            self.gnss_ready.setStyleSheet("font-size:20px; font-weight:800; color:#F3C969;")
        elif lock_state == "unlocked":
            self.gnss_ready.setText("Ready to Log: NO (no GNSS lock)")
            self.gnss_ready.setStyleSheet("font-size:20px; font-weight:800; color:#FF6B6B;")
        else:
            self.gnss_ready.setText("Ready to Log: waiting for GNSS...")
            self.gnss_ready.setStyleSheet("font-size:20px; font-weight:800; color:#F3C969;")

        self._gnss_locked = lock_state == "locked"
        self._gnss_lock_reason = lock_reason
        self._corr_active = corr_state == "on"
        self._corr_reason = corr_reason
        if lock_state == "locked":
            self.ind_gnss_lock.setText("● GNSS Lock: YES")
            self.ind_gnss_lock.setStyleSheet("color:#52D273; font-weight:700;")
        elif lock_state == "unlocked":
            self.ind_gnss_lock.setText("● GNSS Lock: NO")
            self.ind_gnss_lock.setStyleSheet("color:#FF6B6B; font-weight:700;")
        else:
            self.ind_gnss_lock.setText("● GNSS Lock: waiting")
            self.ind_gnss_lock.setStyleSheet("color:#F3C969; font-weight:700;")

        if corr_state == "on":
            self.ind_corr_link.setText("● Corrections: ON")
            self.ind_corr_link.setStyleSheet("color:#52D273; font-weight:700;")
        elif corr_state == "off":
            self.ind_corr_link.setText("● Corrections: OFF")
            self.ind_corr_link.setStyleSheet("color:#FF6B6B; font-weight:700;")
        else:
            self.ind_corr_link.setText("● Corrections: waiting")
            self.ind_corr_link.setStyleSheet("color:#F3C969; font-weight:700;")

    def _render_preview(self, label: QLabel, info: QLabel, sub: ImageSub, name: str):
        if not sub.got_first_frame():
            label.setText(f"{name}: waiting…")
            info.setText("—")
            self._preview_render_cache.pop(name, None)
            return

        frame, enc, msg_ref = sub.get_latest_snapshot()
        if frame is None:
            label.setText(f"{name}: waiting…")
            info.setText("—")
            self._preview_render_cache.pop(name, None)
            return

        age_s, fps = sub.stream_stats()
        info.setText(f"{fps:4.1f} FPS   |   age {age_s*1000:4.0f} ms")

        target_w = max(2, label.width())
        target_h = max(2, label.height())
        frame_key = (id(msg_ref), target_w, target_h)
        if self._preview_render_cache.get(name) == frame_key:
            return

        # Keep per-frame Python work minimal; do scaling in Qt/C++.
        pix = frame_to_pix(frame, enc)
        _ = msg_ref  # keep backing ROS message alive until pixmap conversion is done
        if not pix.isNull():
            if pix.width() != target_w or pix.height() != target_h:
                pix = pix.scaled(target_w, target_h, Qt.KeepAspectRatioByExpanding, Qt.FastTransformation)
        label.setPixmap(pix)
        self._preview_render_cache[name] = frame_key

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
            label.setPixmap(pix.scaled(label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))

        setpix(self.cap0, self._cap0_pix)
        setpix(self.cap1, self._cap1_pix)
        setpix(self.res0, self._res0_pix)
        setpix(self.res1, self._res1_pix)

    def on_save_settings(self):
        self.out_dir = self.out_dir_edit.text().strip() or self.out_dir
        self.jpeg_quality = int(self.quality_spin.value())
        self.ui_fps = max(1, int(self.ui_fps_spin.value()))
        self.preview_fps = max(1, int(self.preview_fps_spin.value()))
        self.preview_relay_fps = max(1, int(self.preview_relay_fps_spin.value()))
        os.makedirs(self.out_dir, exist_ok=True)

        cfg = {
            "output_dir": self.out_dir,
            "jpeg_quality": self.jpeg_quality,
            "ui_fps": self.ui_fps,
            "preview_fps": self.preview_fps,
            "preview_relay_fps": self.preview_relay_fps,
            "capture_node": str(self.ros_node.get_parameter("capture_node").value),
            "cam0_topic": self.cam0_topic_edit.text().strip(),
            "cam1_topic": self.cam1_topic_edit.text().strip(),
            "capture_service": self.srv_name_edit.text().strip(),
            "gnss_fix_topic": self.gnss_fix_topic_edit.text().strip(),
            "gnss_time_ref_topic": self.gnss_time_ref_topic_edit.text().strip(),
            "gnss_imu_topic": self.gnss_imu_topic_edit.text().strip(),
            "require_gnss_lock_for_session": bool(self.ros_node.get_parameter("require_gnss_lock_for_session").value),
            "max_fix_age_ms_for_lock": int(self.ros_node.get_parameter("max_fix_age_ms_for_lock").value),
            "session_bag_topics": str(self.ros_node.get_parameter("session_bag_topics").value),
            "session_record_images": bool(self.ros_node.get_parameter("session_record_images").value),
            "session_cam0_topic": str(self.ros_node.get_parameter("session_cam0_topic").value),
            "session_cam1_topic": str(self.ros_node.get_parameter("session_cam1_topic").value),
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
                        self._log(f"Capture stream fps updated to {self.preview_fps}")
                    else:
                        reason = ""
                        if r and len(r) > 0:
                            reason = str(getattr(r[0], "reason", ""))
                        self._log(f"Capture stream fps update rejected: {reason or 'unknown reason'}")
                except Exception as e:
                    self._log(f"Capture stream fps update failed: {e}")

            cap_fut.add_done_callback(_done_cap)
        else:
            self._log("Capture stream fps update skipped (capture node parameter service not ready)")

        relay_fut = self.ros_node.set_capture_preview_relay_fps_async(self.preview_relay_fps)
        if relay_fut is not None:
            def _done_relay(f):
                try:
                    r = f.result()
                    if r and bool(r[0].successful):
                        self._log(f"Preview relay fps updated to {self.preview_relay_fps}")
                    else:
                        reason = ""
                        if r and len(r) > 0:
                            reason = str(getattr(r[0], "reason", ""))
                        self._log(f"Preview relay fps update rejected: {reason or 'unknown reason'}")
                except Exception as e:
                    self._log(f"Preview relay fps update failed: {e}")

            relay_fut.add_done_callback(_done_relay)
        else:
            self._log("Preview relay fps update skipped (capture node parameter service not ready)")

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
