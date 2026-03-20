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
from datetime import datetime
from typing import Optional, Tuple

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

    if encoding == "rgb8":
        fmt = QImage.Format_RGB888
    else:
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        fmt = QImage.Format_RGB888

    h, w, _ = frame.shape
    qimg = QImage(frame.data, w, h, frame.strides[0], fmt)
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

        # Depth=1 prevents queue buildup when UI/processing can't keep up.
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.sub = self.create_subscription(Image, topic, self.cb, qos)
        self.get_logger().info(f"Preview sub: {topic}")

    def cb(self, msg: Image):
        # Fast path: avoid cv_bridge copy/convert when encoding is already usable.
        frame = None
        enc = (msg.encoding or "").lower()
        if enc in ("bgr8", "rgb8") and msg.step == msg.width * 3:
            try:
                mv = memoryview(msg.data)
                frame = np.ndarray(
                    (msg.height, msg.width, 3),
                    dtype=np.uint8,
                    buffer=mv,
                )
            except Exception:
                frame = None

        if frame is None:
            try:
                # Fallback: convert to BGR for display.
                frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                enc = "bgr8"
            except Exception:
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
        self.declare_parameter("mock_camera_node", cfg.get("mock_camera_node", "/mock_camera_publisher"))
        self.declare_parameter("gnss_fix_topic", cfg.get("gnss_fix_topic", "/fix"))
        self.declare_parameter("gnss_time_ref_topic", cfg.get("gnss_time_ref_topic", "/time_reference"))
        self.declare_parameter("gnss_imu_topic", cfg.get("gnss_imu_topic", "/imu/data"))
        self.declare_parameter("output_dir", cfg.get("output_dir", os.path.expanduser("~/captures")))
        self.declare_parameter("jpeg_quality", int(cfg.get("jpeg_quality", 95)))
        self.declare_parameter("ui_fps", int(cfg.get("ui_fps", 15)))

        self.cli = self.create_client(CapturePair, str(self.get_parameter("capture_service").value))
        self.action_cli = ActionClient(self, CapturePairAction, str(self.get_parameter("capture_action").value))
        self._prefer_action = bool(self.get_parameter("prefer_capture_action").value)
        self._mock_cam_params = AsyncParameterClient(
            self,
            str(self.get_parameter("mock_camera_node").value),
        )

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
            lab.setMinimumSize(240, 160)
        for info in (self.prev0_info, self.prev1_info):
            info.setStyleSheet("color:#B0B0B0; padding:4px 2px;")

        # --- Actions
        self.capture_btn = QPushButton("CAPTURE (12MP JPEG)")
        self.capture_btn.setMinimumHeight(64)
        self.capture_btn.setStyleSheet("font-size:24px; font-weight:700;")
        self.capture_btn.clicked.connect(self.on_capture)

        self.full_btn = QPushButton("Toggle Fullscreen (F11)")
        self.full_btn.setMinimumHeight(64)
        self.full_btn.setStyleSheet("font-size:18px;")
        self.full_btn.clicked.connect(self.toggle_fullscreen)

        self.quit_btn = QPushButton("Quit")
        self.quit_btn.setMinimumHeight(64)
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

        # User-settable values (persisted)
        self.out_dir = str(self.ros_node.get_parameter("output_dir").value)
        self.jpeg_quality = int(self.ros_node.get_parameter("jpeg_quality").value)
        self.ui_fps = max(1, int(self.ros_node.get_parameter("ui_fps").value))

        # ---- Preview tab
        top_row = QHBoxLayout()
        top_row.setSpacing(10)
        top_row.addWidget(self.ind_cam0)
        top_row.addWidget(self.ind_cam1)
        top_row.addWidget(self.ind_srv)
        top_row.addStretch(1)

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

        btn_row = QHBoxLayout()
        btn_row.setSpacing(10)
        btn_row.addWidget(self.capture_btn, 3)
        btn_row.addWidget(self.full_btn, 2)
        btn_row.addWidget(self.quit_btn, 1)

        preview_root = QVBoxLayout()
        preview_root.setContentsMargins(10, 10, 10, 10)
        preview_root.setSpacing(10)
        preview_root.addLayout(top_row, 0)
        preview_root.addLayout(preview_row, 1)
        preview_root.addLayout(btn_row, 0)
        preview_root.addWidget(self.status, 0)

        preview_tab = QWidget()
        preview_tab.setLayout(preview_root)
        self.tabs.addTab(preview_tab, "Preview")

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
        self.tabs.addTab(gnss_tab, "GNSS")

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

        self.tabs.addTab(scroll, "Last Capture")

        # ---- Settings tab
        settings = QWidget()
        sroot = QVBoxLayout()
        sroot.setContentsMargins(10, 10, 10, 10)
        sroot.setSpacing(10)

        self.out_dir_edit = QLineEdit(self.out_dir)
        self.out_dir_edit.setPlaceholderText("/path/to/output")
        self.out_dir_apply = QPushButton("Save")
        self.out_dir_apply.clicked.connect(self.on_save_settings)

        self.quality_spin = QSpinBox()
        self.quality_spin.setRange(10, 100)
        self.quality_spin.setValue(self.jpeg_quality)
        self.quality_spin.setSingleStep(1)

        self.ui_fps_spin = QSpinBox()
        self.ui_fps_spin.setRange(5, 60)
        self.ui_fps_spin.setValue(self.ui_fps)
        self.ui_fps_spin.setSingleStep(5)

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
            l.setMinimumWidth(170)
            r.addWidget(l)
            r.addWidget(widget, 1)
            return r

        sroot.addWidget(QLabel("Capture"))
        sroot.addLayout(row("Output directory", self.out_dir_edit))
        sroot.addLayout(row("JPEG quality", self.quality_spin))
        sroot.addWidget(QLabel("UI"))
        sroot.addLayout(row("Preview UI FPS", self.ui_fps_spin))
        sroot.addWidget(QLabel("ROS"))
        sroot.addLayout(row("Cam0 topic", self.cam0_topic_edit))
        sroot.addLayout(row("Cam1 topic", self.cam1_topic_edit))
        sroot.addLayout(row("Capture service", self.srv_name_edit))
        sroot.addLayout(row("GNSS fix topic", self.gnss_fix_topic_edit))
        sroot.addLayout(row("GNSS time ref topic", self.gnss_time_ref_topic_edit))
        sroot.addLayout(row("GNSS IMU topic", self.gnss_imu_topic_edit))
        sroot.addWidget(self.out_dir_apply)
        sroot.addStretch(1)
        settings.setLayout(sroot)
        self.tabs.addTab(settings, "Settings")

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
        self.tabs.addTab(diag, "Diagnostics")

        outer = QVBoxLayout()
        outer.setContentsMargins(0, 0, 0, 0)
        outer.addWidget(self.tabs)
        self.setLayout(outer)

        os.makedirs(self.out_dir, exist_ok=True)

        # Preview refresh (UI-side). Actual camera FPS is independent.
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh_preview)
        self.timer.start(max(10, int(1000 / self.ui_fps)))

        # Capture future polling
        self._capture_future = None
        self._capture_poll = QTimer(self)
        self._capture_poll.timeout.connect(self._poll_capture_future)

        QTimer.singleShot(0, self._set_default_window_geometry)
        self._log(f"UI started. cam0_topic={self.cam0.topic} cam1_topic={self.cam1.topic}")

    def _set_default_window_geometry(self):
        scr = QApplication.primaryScreen()
        if not scr:
            return
        geo = scr.availableGeometry()
        w = max(900, int(geo.width() * 0.85))
        h = max(600, int(geo.height() * 0.85))
        w = min(w, geo.width())
        h = min(h, geo.height())
        x = geo.x() + max(0, (geo.width() - w) // 2)
        y = geo.y() + max(0, (geo.height() - h) // 2)
        self.setGeometry(x, y, w, h)

    def toggle_fullscreen(self):
        if self.windowState() & Qt.WindowFullScreen:
            self.setWindowState(self.windowState() & ~Qt.WindowFullScreen)
        else:
            self.setWindowState(self.windowState() | Qt.WindowFullScreen)

    def resizeEvent(self, e):
        super().resizeEvent(e)
        # Rescale last capture pixmaps to avoid cut-off / stale scaling
        self._apply_capture_pixmaps()

    def keyPressEvent(self, e):
        # Touch UIs often still have a keyboard during development; make common
        # actions quick.
        if e.key() == Qt.Key_F11:
            self.toggle_fullscreen()
            return
        if e.key() in (Qt.Key_Space, Qt.Key_Return, Qt.Key_Enter):
            if self.capture_btn.isEnabled():
                self.on_capture()
            return
        super().keyPressEvent(e)

    def refresh_preview(self):
        self._update_indicators()
        if self._preview_paused:
            self.prev0_info.setText("paused for capture…")
            self.prev1_info.setText("paused for capture…")
        else:
            self._render_preview(self.prev0, self.prev0_info, self.cam0, "Cam0")
            self._render_preview(self.prev1, self.prev1_info, self.cam1, "Cam1")
        self._refresh_gnss()

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

        # Keep per-frame work minimal:
        #  - avoid cvtColor via Qt's BGR888/RGB888
        #  - resize only when needed
        target_w = max(2, label.width())
        target_h = max(2, label.height())
        if frame.shape[1] != target_w or frame.shape[0] != target_h:
            # INTER_LINEAR is a good speed/quality trade for preview.
            frame = cv2.resize(frame, (target_w, target_h), interpolation=cv2.INTER_LINEAR)

        pix = frame_to_pix(frame, enc)
        _ = msg_ref  # keep backing ROS message alive until pixmap conversion is done
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

        # Gate capture button on service readiness
        self.capture_btn.setEnabled(self.ros_node.service_ready() and self._capture_future is None)

    def on_capture(self):
        if not self.ros_node.service_ready():
            self.status.setText("Status: capture service not ready")
            self._log("Capture refused: service not ready")
            return

        # Apply current settings
        self.out_dir = self.out_dir_edit.text().strip() or self.out_dir
        os.makedirs(self.out_dir, exist_ok=True)
        self.jpeg_quality = int(self.quality_spin.value())

        self.capture_btn.setEnabled(False)
        self.capture_btn.setText("CAPTURING…")
        self.status.setText("Status: capturing…")
        self._preview_paused = True

        session = datetime.now().strftime("%Y%m%d_%H%M%S")
        self._capture_started_mono = time.monotonic()
        self._capture_future = self.ros_node.capture_pair_async(session, self.out_dir, self.jpeg_quality)
        self._log(f"Capture requested: session={session} out_dir={self.out_dir} q={self.jpeg_quality}")
        self._capture_poll.start(50)

    def _poll_capture_future(self):
        fut = self._capture_future
        if fut is None or not fut.done():
            # Safety: surface hung service calls
            if fut is not None and (time.monotonic() - self._capture_started_mono) > 12.0:
                self.status.setText("Status: capture timeout (no response)")
                self._log("Capture timeout (>12s)")
                self._capture_poll.stop()
                self._capture_future = None
                self._preview_paused = False
                self.capture_btn.setEnabled(True)
                self.capture_btn.setText("CAPTURE (12MP JPEG)")
            return

        self._capture_poll.stop()
        self._capture_future = None
        self._preview_paused = False

        try:
            resp = fut.result()
        except Exception as e:
            self.status.setText(f"Status: capture call failed: {e}")
            self._log(f"Capture call failed: {e}")
            self.capture_btn.setEnabled(True)
            self.capture_btn.setText("CAPTURE (12MP JPEG)")
            return

        if (resp is None) or (not resp.success):
            msg = "(no response)" if resp is None else resp.message
            self.status.setText("Status: CAPTURE FAILED (see details)")
            self._log("Capture failed")
            # Show error in capture boxes so it’s visible on-screen
            self.cap0.setText(msg[:800])
            self.cap1.setText(msg[:800])
            self.capture_details.setPlainText(msg)
        else:
            self.status.setText("Status: capture OK")
            self._log(f"Capture OK: cam0={resp.cam0_path} cam1={resp.cam1_path}")
            self._cap0_pix = load_jpeg_as_pix(resp.cam0_path)
            self._cap1_pix = load_jpeg_as_pix(resp.cam1_path)
            self._res0_pix = self._cap0_pix
            self._res1_pix = self._cap1_pix
            self._apply_capture_pixmaps()
            self.tabs.setCurrentIndex(1)

            self.capture_details.setPlainText(
                "\n".join(
                    [
                        f"stamp: {resp.stamp.sec}.{resp.stamp.nanosec:09d}",
                        f"cam0:  {resp.cam0_path}",
                        f"cam1:  {resp.cam1_path}",
                        f"out:   {self.out_dir}",
                        f"q:     {self.jpeg_quality}",
                    ]
                )
            )

        self.capture_btn.setEnabled(True)
        self.capture_btn.setText("CAPTURE (12MP JPEG)")

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
        os.makedirs(self.out_dir, exist_ok=True)

        cfg = {
            "output_dir": self.out_dir,
            "jpeg_quality": self.jpeg_quality,
            "ui_fps": self.ui_fps,
            "cam0_topic": self.cam0_topic_edit.text().strip(),
            "cam1_topic": self.cam1_topic_edit.text().strip(),
            "capture_service": self.srv_name_edit.text().strip(),
            "gnss_fix_topic": self.gnss_fix_topic_edit.text().strip(),
            "gnss_time_ref_topic": self.gnss_time_ref_topic_edit.text().strip(),
            "gnss_imu_topic": self.gnss_imu_topic_edit.text().strip(),
        }
        save_config(cfg)
        self.status.setText("Status: settings saved (restart for topic/service changes)")
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
