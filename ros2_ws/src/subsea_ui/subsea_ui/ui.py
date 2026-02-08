#!/usr/bin/env python3
import os
import sys
import threading
from typing import Optional

import numpy as np
import cv2

from qtpy.QtCore import Qt, QTimer
from qtpy.QtGui import QImage, QPixmap
from qtpy.QtWidgets import (
    QApplication, QWidget, QLabel, QPushButton,
    QHBoxLayout, QVBoxLayout, QSizePolicy, QTabWidget, QScrollArea
)

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from subsea_interfaces.srv import CapturePair


def cv_to_pix(bgr: np.ndarray) -> QPixmap:
    if bgr is None:
        return QPixmap()
    if not bgr.flags["C_CONTIGUOUS"]:
        bgr = np.ascontiguousarray(bgr)
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    h, w, ch = rgb.shape
    qimg = QImage(rgb.data, w, h, ch * w, QImage.Format_RGB888)
    return QPixmap.fromImage(qimg)


def load_jpeg_as_pix(path: str) -> Optional[QPixmap]:
    if not path or not os.path.exists(path):
        return None
    bgr = cv2.imread(path, cv2.IMREAD_COLOR)
    if bgr is None:
        return None
    return cv_to_pix(bgr)


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
        self.bridge = CvBridge()
        self._lock = threading.Lock()
        self._latest: Optional[np.ndarray] = None
        self._got_first = False
        self.sub = self.create_subscription(Image, topic, self.cb, qos_profile_sensor_data)
        self.get_logger().info(f"Preview sub: {topic}")

    def cb(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception:
            return
        with self._lock:
            self._latest = frame
            self._got_first = True

    def got_first_frame(self) -> bool:
        with self._lock:
            return self._got_first

    def get_latest_copy(self) -> Optional[np.ndarray]:
        with self._lock:
            return None if self._latest is None else self._latest.copy()


class AppNode(Node):
    def __init__(self):
        super().__init__("subsea_ui_node")
        self.cli = self.create_client(CapturePair, "capture_pair")
        self.get_logger().info("Waiting for /capture_pair service...")
        self.cli.wait_for_service()
        self.get_logger().info("/capture_pair available.")

    def capture_pair_async(self, session_id: str, out_dir: str, quality: int = 95):
        req = CapturePair.Request()
        req.session_id = session_id
        req.output_dir = out_dir
        req.jpeg_quality = quality
        return self.cli.call_async(req)


class MainWindow(QWidget):
    def __init__(self, ros_node: AppNode, cam0: ImageSub, cam1: ImageSub):
        super().__init__()
        self.ros_node = ros_node
        self.cam0 = cam0
        self.cam1 = cam1

        self.setWindowTitle("Subsea Rover App")

        self.tabs = QTabWidget()

        # Preview widgets
        self.prev0 = QLabel("Cam0: waiting…")
        self.prev1 = QLabel("Cam1: waiting…")

        # Capture widgets
        self.cap0 = QLabel("Cam0 capture: (none)")
        self.cap1 = QLabel("Cam1 capture: (none)")
        self.res0 = QLabel("Cam0 deblur: (placeholder)")
        self.res1 = QLabel("Cam1 deblur: (placeholder)")

        for lab in (self.prev0, self.prev1, self.cap0, self.cap1, self.res0, self.res1):
            lab.setAlignment(Qt.AlignCenter)
            lab.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
            lab.setStyleSheet("background-color:black; color:white; font-size:16px;")
            lab.setMinimumSize(200, 140)

        self.status = QLabel("Status: ready")
        self.status.setStyleSheet("color:white; background:#202020; padding:6px;")
        self.status.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)

        self.capture_btn = QPushButton("CAPTURE (12MP JPEG)")
        self.capture_btn.setMinimumHeight(60)
        self.capture_btn.setStyleSheet("font-size:22px;")
        self.capture_btn.clicked.connect(self.on_capture)

        self.full_btn = QPushButton("Fullscreen")
        self.full_btn.setMinimumHeight(60)
        self.full_btn.setStyleSheet("font-size:18px;")
        self.full_btn.clicked.connect(self.toggle_fullscreen)

        # Store last pixmaps so we can rescale on resize
        self._cap0_pix: Optional[QPixmap] = None
        self._cap1_pix: Optional[QPixmap] = None
        self._res0_pix: Optional[QPixmap] = None
        self._res1_pix: Optional[QPixmap] = None

        # ---- Preview tab
        preview_row = QHBoxLayout()
        preview_row.setSpacing(6)
        preview_row.addWidget(self.prev0, 1)
        preview_row.addWidget(self.prev1, 1)

        btn_row = QHBoxLayout()
        btn_row.setSpacing(6)
        btn_row.addWidget(self.capture_btn, 2)
        btn_row.addWidget(self.full_btn, 1)

        preview_root = QVBoxLayout()
        preview_root.setContentsMargins(6, 6, 6, 6)
        preview_root.setSpacing(6)
        preview_root.addLayout(preview_row, 1)
        preview_root.addLayout(btn_row, 0)
        preview_root.addWidget(self.status, 0)

        preview_tab = QWidget()
        preview_tab.setLayout(preview_root)
        self.tabs.addTab(preview_tab, "Preview")

        # ---- Capture tab (scrollable to prevent cut-off)
        cap_row = QHBoxLayout()
        cap_row.setSpacing(6)
        cap_row.addWidget(self.cap0, 1)
        cap_row.addWidget(self.cap1, 1)

        res_row = QHBoxLayout()
        res_row.setSpacing(6)
        res_row.addWidget(self.res0, 1)
        res_row.addWidget(self.res1, 1)

        capture_inner = QWidget()
        capture_layout = QVBoxLayout()
        capture_layout.setContentsMargins(6, 6, 6, 6)
        capture_layout.setSpacing(6)
        capture_layout.addLayout(cap_row, 1)
        capture_layout.addLayout(res_row, 1)
        capture_inner.setLayout(capture_layout)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(capture_inner)

        self.tabs.addTab(scroll, "Last Capture")

        outer = QVBoxLayout()
        outer.setContentsMargins(0, 0, 0, 0)
        outer.addWidget(self.tabs)
        self.setLayout(outer)

        self.out_dir = os.path.expanduser("~/captures")
        os.makedirs(self.out_dir, exist_ok=True)

        # Preview refresh: 15 fps reduces lag and CPU
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh_preview)
        self.timer.start(66)

        # Capture future polling
        self._capture_future = None
        self._capture_poll = QTimer(self)
        self._capture_poll.timeout.connect(self._poll_capture_future)

        QTimer.singleShot(0, self._force_fullscreen_geometry)

    def _force_fullscreen_geometry(self):
        scr = QApplication.primaryScreen()
        if scr:
            geo = scr.availableGeometry()  # availableGeometry avoids taskbar/odd offsets
            self.setGeometry(geo)

    def toggle_fullscreen(self):
        if self.windowState() & Qt.WindowFullScreen:
            self.setWindowState(self.windowState() & ~Qt.WindowFullScreen)
        else:
            self.setWindowState(self.windowState() | Qt.WindowFullScreen)
        QTimer.singleShot(0, self._force_fullscreen_geometry)

    def resizeEvent(self, e):
        super().resizeEvent(e)
        # Rescale last capture pixmaps to avoid cut-off / stale scaling
        self._apply_capture_pixmaps()

    def refresh_preview(self):
        self._render_preview(self.prev0, self.cam0, "Cam0")
        self._render_preview(self.prev1, self.cam1, "Cam1")

    def _render_preview(self, label: QLabel, sub: ImageSub, name: str):
        if not sub.got_first_frame():
            label.setText(f"{name}: waiting…")
            return

        frame = sub.get_latest_copy()
        if frame is None:
            label.setText(f"{name}: waiting…")
            return

        # Lower latency: scale in OpenCV before QImage conversion
        # (keeps Python+Qt work bounded)
        target_w = max(2, label.width())
        target_h = max(2, label.height())
        frame = cv2.resize(frame, (target_w, target_h), interpolation=cv2.INTER_AREA)

        pix = cv_to_pix(frame)
        label.setPixmap(pix)  # already sized; no Qt scaling here

    def on_capture(self):
        self.capture_btn.setEnabled(False)
        self.capture_btn.setText("CAPTURING…")
        self.status.setText("Status: capturing…")

        import datetime
        session = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self._capture_future = self.ros_node.capture_pair_async(session, self.out_dir, 95)
        self._capture_poll.start(50)

    def _poll_capture_future(self):
        fut = self._capture_future
        if fut is None or not fut.done():
            return

        self._capture_poll.stop()
        self._capture_future = None

        try:
            resp = fut.result()
        except Exception as e:
            self.status.setText(f"Status: capture call failed: {e}")
            self.capture_btn.setEnabled(True)
            self.capture_btn.setText("CAPTURE (12MP JPEG)")
            return

        if (resp is None) or (not resp.success):
            msg = "(no response)" if resp is None else resp.message
            self.status.setText("Status: CAPTURE FAILED (see details)")
            # Show error in capture boxes so it’s visible on-screen
            self.cap0.setText(msg[:800])
            self.cap1.setText(msg[:800])
        else:
            self.status.setText("Status: capture OK")
            self._cap0_pix = load_jpeg_as_pix(resp.cam0_path)
            self._cap1_pix = load_jpeg_as_pix(resp.cam1_path)
            self._res0_pix = self._cap0_pix
            self._res1_pix = self._cap1_pix
            self._apply_capture_pixmaps()
            self.tabs.setCurrentIndex(1)

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


def main():
    _single_instance_lock()

    cam0_topic = "/cam0/camera0/image_raw"
    cam1_topic = "/cam1/camera1/image_raw"

    os.environ.setdefault("QT_AUTO_SCREEN_SCALE_FACTOR", "1")
    os.environ.setdefault("QT_ENABLE_HIGHDPI_SCALING", "1")

    rclpy.init()
    app_node = AppNode()
    cam0 = ImageSub("cam0_preview_sub", cam0_topic)
    cam1 = ImageSub("cam1_preview_sub", cam1_topic)

    executor = SingleThreadedExecutor()
    executor.add_node(app_node)
    executor.add_node(cam0)
    executor.add_node(cam1)
    threading.Thread(target=executor.spin, daemon=True).start()

    app = QApplication(sys.argv)
    w = MainWindow(app_node, cam0, cam1)
    w.setWindowState(w.windowState() | Qt.WindowFullScreen)
    w.show()
    ret = app.exec()

    executor.shutdown()
    app_node.destroy_node()
    cam0.destroy_node()
    cam1.destroy_node()
    rclpy.shutdown()
    return int(ret)


if __name__ == "__main__":
    raise SystemExit(main())
