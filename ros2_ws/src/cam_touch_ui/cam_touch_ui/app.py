#!/usr/bin/env python3
import os
import sys
import time
import signal
import threading
import subprocess
from typing import Optional, Tuple

import numpy as np
import cv2

from qtpy.QtCore import Qt, QTimer
from qtpy.QtGui import QImage, QPixmap
from qtpy.QtWidgets import QApplication, QWidget, QLabel, QPushButton, QHBoxLayout, QVBoxLayout

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from .theme import apply_dark_theme

from .theme import apply_dark_theme


def frame_to_qpixmap(frame: np.ndarray, encoding: str) -> QPixmap:
    """Convert an HxWx3 uint8 frame to QPixmap with minimal work.

    Avoid cv2.cvtColor when possible by using Qt's RGB/BGR888 support.
    """
    if frame is None:
        return QPixmap()
    if frame.ndim != 3 or frame.shape[2] != 3:
        return QPixmap()
    if not frame.flags["C_CONTIGUOUS"]:
        frame = np.ascontiguousarray(frame)

    if encoding == "rgb8":
        fmt = QImage.Format_RGB888
    elif hasattr(QImage, "Format_BGR888"):
        fmt = QImage.Format_BGR888
    else:
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        fmt = QImage.Format_RGB888

    h, w, _ = frame.shape
    qimg = QImage(frame.data, w, h, frame.strides[0], fmt)
    return QPixmap.fromImage(qimg)


def start_camera_ros(namespace: str, camera_index: int, width: int, height: int, fps: int) -> subprocess.Popen:
    # Do NOT force format: improves dual-camera stability.
    frame_us = int(1_000_000 / max(1, int(fps)))
    cmd = (
        "bash -lc '"
        "set -e; "
        "source /opt/ros/jazzy/setup.bash && "
        "if [ -f ~/ros2_ws/install/setup.bash ]; then source ~/ros2_ws/install/setup.bash; fi && "
        "exec ros2 run camera_ros camera_node --ros-args "
        f"-r __ns:={namespace} "
        f"-p camera:={camera_index} "
        f"-p width:={width} -p height:={height} "
        f"-p FrameDurationLimits:=[{frame_us},{frame_us}]"
        "'"
    )
    return subprocess.Popen(cmd, shell=True, preexec_fn=os.setsid)


def kill_process_group(p: subprocess.Popen, sig=signal.SIGINT, timeout_s: float = 2.0) -> None:
    if p is None:
        return
    try:
        pgid = os.getpgid(p.pid)
        os.killpg(pgid, sig)
    except Exception:
        return

    t0 = time.time()
    while time.time() - t0 < timeout_s:
        if p.poll() is not None:
            return
        time.sleep(0.05)

    try:
        os.killpg(pgid, signal.SIGKILL)
    except Exception:
        pass


class CamSubscriber(Node):
    """
    RAW Image subscriber (low latency). Uses sensor_data QoS to avoid queue buildup.
    Stores latest frame in BGR without copying on read.
    """
    def __init__(self, name: str, topic: str):
        super().__init__(name)
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.latest: Optional[np.ndarray] = None
        self._latest_encoding: str = "bgr8"
        self._latest_msg: Optional[Image] = None  # keep msg alive for zero-copy views
        self._last_rx_mono: Optional[float] = None
        self._ema_fps: float = 0.0

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.sub = self.create_subscription(
            Image,
            topic,
            self.cb_raw,
            qos,
        )
        self.get_logger().info(f"Subscribing (raw) to: {topic}")

    def cb_raw(self, msg: Image) -> None:
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
                frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                enc = "bgr8"
            except Exception:
                return

        now_m = time.monotonic()
        with self.lock:
            self.latest = frame
            self._latest_encoding = enc if enc in ("bgr8", "rgb8") else "bgr8"
            self._latest_msg = msg
            if self._last_rx_mono is not None:
                dt = max(1e-6, now_m - self._last_rx_mono)
                fps = 1.0 / dt
                self._ema_fps = fps if self._ema_fps <= 0.0 else (0.85 * self._ema_fps + 0.15 * fps)
            self._last_rx_mono = now_m

    def get_latest(self) -> Optional[np.ndarray]:
        # <-- FIX: no frame.copy() (huge CPU/mem win). We only read in UI thread.
        with self.lock:
            return self.latest

    def latest_encoding(self) -> str:
        with self.lock:
            return self._latest_encoding

    def stats(self) -> Tuple[float, float]:
        """Returns (age_s, fps_ema)."""
        with self.lock:
            t = self._last_rx_mono
            fps = self._ema_fps
        if t is None:
            return 1e9, fps
        return max(0.0, time.monotonic() - t), fps


class MainWindow(QWidget):
    def __init__(self, cam0: CamSubscriber, cam1: CamSubscriber):
        super().__init__()
        self.cam0 = cam0
        self.cam1 = cam1

        self.setWindowTitle("Touch Camera Viewer (ROS 2)")

        # Connection indicators (use colored dot via stylesheet)
        self.ind0 = QLabel("● Cam0")
        self.ind1 = QLabel("● Cam1")
        for ind in (self.ind0, self.ind1):
            ind.setStyleSheet("font-weight:700;")
        self.left_label = QLabel("Left camera: waiting…")
        self.right_label = QLabel("Right camera: waiting…")

        for lab in (self.left_label, self.right_label):
            lab.setAlignment(Qt.AlignCenter)
            lab.setMinimumSize(400, 240)
            lab.setStyleSheet("background-color: black; color: white; font-size: 18px;")

        self.swap_btn = QPushButton("Swap")
        self.swap_btn.setMinimumHeight(60)
        self.swap_btn.setStyleSheet("font-size: 22px;")
        self.swap_btn.clicked.connect(self.swap_views)

        self.full_btn = QPushButton("Toggle Fullscreen")
        self.full_btn.setMinimumHeight(60)
        self.full_btn.setStyleSheet("font-size: 22px;")
        self.full_btn.clicked.connect(self.toggle_fullscreen)

        btn_row = QHBoxLayout()
        btn_row.addWidget(self.swap_btn)
        btn_row.addWidget(self.full_btn)

        top_row = QHBoxLayout()
        top_row.addWidget(self.ind0)
        top_row.addWidget(self.ind1)
        top_row.addStretch(1)

        self.status = QLabel("Status: waiting for frames…")
        self.status.setStyleSheet("background:#1B1B1B; border:1px solid #2B2B2B; border-radius:10px; padding:8px;")

        cams_row = QHBoxLayout()
        cams_row.addWidget(self.left_label, 1)
        cams_row.addWidget(self.right_label, 1)

        root = QVBoxLayout()
        root.addLayout(top_row, 0)
        root.addLayout(cams_row, 1)
        root.addLayout(btn_row)
        root.addWidget(self.status, 0)
        self.setLayout(root)

        self.left_is_cam0 = True

        # <-- FIX: UI refresh 25 FPS (less CPU than 30)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_frames)
        self.timer.start(40)

    def swap_views(self) -> None:
        self.left_is_cam0 = not self.left_is_cam0

    def toggle_fullscreen(self) -> None:
        self.showNormal() if self.isFullScreen() else self.showFullScreen()

    def update_frames(self) -> None:
        left_src = self.cam0 if self.left_is_cam0 else self.cam1
        right_src = self.cam1 if self.left_is_cam0 else self.cam0

        # Update connection indicators
        age0, fps0 = self.cam0.stats()
        age1, fps1 = self.cam1.stats()
        self._set_ind(self.ind0, ok=age0 < 0.7, warn=age0 < 2.0)
        self._set_ind(self.ind1, ok=age1 < 0.7, warn=age1 < 2.0)
        self.status.setText(f"Cam0 {fps0:4.1f} FPS (age {age0*1000:4.0f} ms)  |  Cam1 {fps1:4.1f} FPS (age {age1*1000:4.0f} ms)")

        self._render_label(self.left_label, left_src, "Left")
        self._render_label(self.right_label, right_src, "Right")

    @staticmethod
    def _set_ind(ind: QLabel, ok: bool, warn: bool = False) -> None:
        if ok:
            ind.setStyleSheet("color:#52D273; font-weight:700;")
        elif warn:
            ind.setStyleSheet("color:#F3C969; font-weight:700;")
        else:
            ind.setStyleSheet("color:#FF6B6B; font-weight:700;")

    def _render_label(self, label: QLabel, src: CamSubscriber, name: str) -> None:
        frame = src.get_latest()
        if frame is None:
            label.setText(f"{name}: waiting…")
            return

        pix = frame_to_qpixmap(frame, src.latest_encoding())

        # <-- FIX: FastTransformation avoids expensive filtering every frame
        label.setPixmap(
            pix.scaled(label.size(), Qt.KeepAspectRatio, Qt.FastTransformation)
        )


def main() -> int:
    # camera_ros topics (raw)
    CAM0_TOPIC = "/cam0/camera/image_raw"
    CAM1_TOPIC = "/cam1/camera/image_raw"

    # <-- FIX: lower preview resolution for low latency (still good on small touchscreens)
    WIDTH, HEIGHT, FPS = 960, 540, 30

    CAM0_INDEX = 0
    CAM1_INDEX = 1

    # Non-destructive check: warn about existing camera nodes but never kill
    # processes we did not start ourselves.
    try:
        existing = subprocess.run(
            [
                "pgrep",
                "-af",
                "camera_ros.*camera_node|install/camera_ros/lib/camera_ros/camera_node",
            ],
            capture_output=True,
            text=True,
            check=False,
        )
        if existing.returncode == 0 and existing.stdout.strip():
            print(
                "cam_touch_ui: existing camera_node processes detected; "
                "not terminating them automatically.",
                file=sys.stderr,
            )
    except Exception:
        pass

    ros_argv = sys.argv
    qt_argv = remove_ros_args(sys.argv)

    rclpy.init(args=ros_argv)

    cam0_proc = start_camera_ros("/cam0", CAM0_INDEX, WIDTH, HEIGHT, FPS)
    cam1_proc = start_camera_ros("/cam1", CAM1_INDEX, WIDTH, HEIGHT, FPS)

    cam0 = CamSubscriber("cam0_sub", CAM0_TOPIC)
    cam1 = CamSubscriber("cam1_sub", CAM1_TOPIC)

    app = QApplication(qt_argv)
    apply_dark_theme(app)

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(cam0)
    executor.add_node(cam1)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    w = MainWindow(cam0, cam1)
    w.showFullScreen()
    ret = app.exec()

    executor.shutdown()
    try:
        spin_thread.join(timeout=1.0)
    except Exception:
        pass
    cam0.destroy_node()
    cam1.destroy_node()
    rclpy.try_shutdown()

    kill_process_group(cam0_proc)
    kill_process_group(cam1_proc)

    return int(ret)


if __name__ == "__main__":
    raise SystemExit(main())
