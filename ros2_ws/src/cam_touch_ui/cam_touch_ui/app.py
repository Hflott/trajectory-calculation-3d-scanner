#!/usr/bin/env python3
import os
import sys
import time
import signal
import threading
import subprocess
from typing import Optional

import numpy as np
import cv2

from qtpy.QtCore import Qt, QTimer
from qtpy.QtGui import QImage, QPixmap
from qtpy.QtWidgets import QApplication, QWidget, QLabel, QPushButton, QHBoxLayout, QVBoxLayout

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data  # <-- FIX: required import
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


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
        with self.lock:
            self.latest = frame

    def get_latest(self) -> Optional[np.ndarray]:
        # <-- FIX: no frame.copy() (huge CPU/mem win). We only read in UI thread.
        with self.lock:
            return self.latest


class MainWindow(QWidget):
    def __init__(self, cam0: CamSubscriber, cam1: CamSubscriber):
        super().__init__()
        self.cam0 = cam0
        self.cam1 = cam1

        self.setWindowTitle("Touch Camera Viewer (ROS 2)")
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

        cams_row = QHBoxLayout()
        cams_row.addWidget(self.left_label, 1)
        cams_row.addWidget(self.right_label, 1)

        root = QVBoxLayout()
        root.addLayout(cams_row, 1)
        root.addLayout(btn_row)
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

        self._render_label(self.left_label, left_src, "Left")
        self._render_label(self.right_label, right_src, "Right")

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

    rclpy.init()

    cam0_proc = start_camera_ros("/cam0", CAM0_INDEX, WIDTH, HEIGHT, FPS)
    cam1_proc = start_camera_ros("/cam1", CAM1_INDEX, WIDTH, HEIGHT, FPS)

    cam0 = CamSubscriber("cam0_sub", CAM0_TOPIC)
    cam1 = CamSubscriber("cam1_sub", CAM1_TOPIC)

    executor = SingleThreadedExecutor()
    executor.add_node(cam0)
    executor.add_node(cam1)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    app = QApplication(sys.argv)
    w = MainWindow(cam0, cam1)
    w.showFullScreen()
    ret = app.exec()

    executor.shutdown()
    cam0.destroy_node()
    cam1.destroy_node()
    rclpy.shutdown()

    kill_process_group(cam0_proc)
    kill_process_group(cam1_proc)

    return int(ret)


if __name__ == "__main__":
    raise SystemExit(main())
