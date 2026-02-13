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
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data  # <-- FIX: required import
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from .theme import apply_dark_theme

from .theme import apply_dark_theme


def cv_to_qpixmap(bgr: np.ndarray) -> QPixmap:
    # Keep this cheap: no extra copies besides cvtColor
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    h, w, ch = rgb.shape
    bytes_per_line = ch * w
    qimg = QImage(rgb.data, w, h, bytes_per_line, QImage.Format_RGB888)
    return QPixmap.fromImage(qimg)


def start_camera_ros(namespace: str, camera_index: int, width: int, height: int, fps: int) -> subprocess.Popen:
    # Do NOT force format: improves dual-camera stability.
    cmd = (
        "bash -lc '"
        "set -e; "
        "source /opt/ros/jazzy/setup.bash && "
        "if [ -f ~/ros2_ws/install/setup.bash ]; then source ~/ros2_ws/install/setup.bash; fi && "
        "exec ros2 run camera_ros camera_node --ros-args "
        f"-r __ns:={namespace} "
        f"-p camera:={camera_index} "
        f"-p width:={width} -p height:={height} -p fps:={fps}"
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
        self._last_rx_mono: Optional[float] = None
        self._ema_fps: float = 0.0

        self.sub = self.create_subscription(
            Image,
            topic,
            self.cb_raw,
            qos_profile_sensor_data,  # <-- low-latency QoS (best effort, small queue)
        )
        self.get_logger().info(f"Subscribing (raw) to: {topic}")

    def cb_raw(self, msg: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"raw decode failed: {e}")
            return
        now_m = time.monotonic()
        with self.lock:
            self.latest = frame
            if self._last_rx_mono is not None:
                dt = max(1e-6, now_m - self._last_rx_mono)
                fps = 1.0 / dt
                self._ema_fps = fps if self._ema_fps <= 0.0 else (0.85 * self._ema_fps + 0.15 * fps)
            self._last_rx_mono = now_m

    def get_latest(self) -> Optional[np.ndarray]:
        # <-- FIX: no frame.copy() (huge CPU/mem win). We only read in UI thread.
        with self.lock:
            return self.latest

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

        pix = cv_to_qpixmap(frame)

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

    # Clean out any stale camera nodes from previous runs (prevents “busy” / weirdness)
    subprocess.run(["bash", "-lc", "pkill -f 'camera_ros.*camera_node' || true; pkill -f 'install/camera_ros/lib/camera_ros/camera_node' || true"], check=False)

    ros_argv = sys.argv
    qt_argv = remove_ros_args(sys.argv)

    rclpy.init(args=ros_argv)

    cam0_proc = start_camera_ros("/cam0", CAM0_INDEX, WIDTH, HEIGHT, FPS)
    cam1_proc = start_camera_ros("/cam1", CAM1_INDEX, WIDTH, HEIGHT, FPS)

    cam0 = CamSubscriber("cam0_sub", CAM0_TOPIC)
    cam1 = CamSubscriber("cam1_sub", CAM1_TOPIC)

    executor = SingleThreadedExecutor()
    executor.add_node(cam0)
    executor.add_node(cam1)

    app = QApplication(qt_argv)
    apply_dark_theme(app)

    # Integrate ROS spinning into the Qt event loop (avoids non-QThread warnings)
    spin_timer = QTimer()
    spin_timer.setInterval(10)  # ms

    def _spin_ros_once():
        try:
            executor.spin_once(timeout_sec=0.0)
        except Exception:
            try:
                spin_timer.stop()
            except Exception:
                pass

    spin_timer.timeout.connect(_spin_ros_once)
    spin_timer.start()
    w = MainWindow(cam0, cam1)
    w.showFullScreen()
    ret = app.exec()

    try:
        spin_timer.stop()
    except Exception:
        pass

    executor.shutdown()
    cam0.destroy_node()
    cam1.destroy_node()
    rclpy.try_shutdown()

    kill_process_group(cam0_proc)
    kill_process_group(cam1_proc)

    return int(ret)


if __name__ == "__main__":
    raise SystemExit(main())
