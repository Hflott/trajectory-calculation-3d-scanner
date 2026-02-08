#!/usr/bin/env python3
import os
import time
import subprocess
from datetime import datetime
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time as TimeMsg

from subsea_interfaces.srv import CapturePair


def now_ros_time(node: Node) -> TimeMsg:
    return node.get_clock().now().to_msg()


def _pkill(pattern: str) -> None:
    # SIGINT so launch can respawn
    subprocess.run(
        ["pkill", "-2", "-f", pattern],
        check=False,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )


def stop_previews(wait_s: float = 2.0) -> None:
    """
    Stop camera_ros nodes so rpicam-still can acquire the pipeline.
    These patterns match your actual launched command lines.
    """
    # Match by namespace + node name (very reliable)
    _pkill(r"__ns:=/cam0")
    _pkill(r"__ns:=/cam1")
    _pkill(r"__node:=camera0")
    _pkill(r"__node:=camera1")

    # Give libcamera time to fully release devices
    time.sleep(wait_s)


class CaptureService(Node):
    def __init__(self):
        super().__init__("capture_service")

        self.declare_parameter("cam0_index", 0)
        self.declare_parameter("cam1_index", 1)
        self.declare_parameter("width", 4056)
        self.declare_parameter("height", 3040)

        self.declare_parameter("warmup_ms", 300)
        self.declare_parameter("timeout_ms", 1500)

        self.declare_parameter("default_quality", 95)

        # Option A controls
        self.declare_parameter("pause_previews", True)
        self.declare_parameter("pause_wait_s", 2.0)

        # Reliability knobs
        self.declare_parameter("retries", 2)
        self.declare_parameter("retry_wait_s", 0.8)

        self.srv = self.create_service(CapturePair, "capture_pair", self.on_capture)
        self.get_logger().info("Capture service ready: /capture_pair")

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

    def _run_capture_once(self, cam_index: int, out_path: str, quality: int) -> Tuple[bool, str, str, int]:
        cmd = self._rpicam_cmd(cam_index, out_path, quality)
        p = subprocess.run(cmd, capture_output=True, text=True)
        ok = (p.returncode == 0) and os.path.exists(out_path) and os.path.getsize(out_path) > 0
        return ok, p.stdout, p.stderr, p.returncode

    def _run_capture_with_retries(self, cam_index: int, out_path: str, quality: int) -> Tuple[bool, str]:
        retries = int(self.get_parameter("retries").value)
        retry_wait = float(self.get_parameter("retry_wait_s").value)

        last_detail = ""
        for attempt in range(retries + 1):
            ok, out, err, rc = self._run_capture_once(cam_index, out_path, quality)

            if ok:
                return True, ""

            last_detail = (
                f"attempt={attempt+1}/{retries+1} cam={cam_index} rc={rc}\n"
                f"stdout:\n{out}\n"
                f"stderr:\n{err}\n"
            )

            # If it's a busy pipeline, waiting helps.
            time.sleep(retry_wait)

        return False, last_detail

    def on_capture(self, req: CapturePair.Request, res: CapturePair.Response) -> CapturePair.Response:
        cam0 = int(self.get_parameter("cam0_index").value)
        cam1 = int(self.get_parameter("cam1_index").value)
        default_quality = int(self.get_parameter("default_quality").value)
        quality = int(req.jpeg_quality) if req.jpeg_quality > 0 else default_quality

        out_dir = req.output_dir.strip() or os.path.expanduser("~/captures")
        os.makedirs(out_dir, exist_ok=True)

        session = req.session_id.strip() or datetime.now().strftime("%Y%m%d_%H%M%S")
        cam0_path = os.path.join(out_dir, f"{session}_cam0.jpg")
        cam1_path = os.path.join(out_dir, f"{session}_cam1.jpg")

        res.stamp = now_ros_time(self)

        self.get_logger().info(f"Capture session={session} -> {cam0_path}, {cam1_path}")

        if bool(self.get_parameter("pause_previews").value):
            wait_s = float(self.get_parameter("pause_wait_s").value)
            self.get_logger().info("Pausing previews...")
            stop_previews(wait_s=wait_s)

        # Sequential capture (most reliable)
        ok0, d0 = self._run_capture_with_retries(cam0, cam0_path, quality)
        ok1, d1 = self._run_capture_with_retries(cam1, cam1_path, quality)

        if not ok0 or not ok1:
            res.success = False
            res.cam0_path = cam0_path if os.path.exists(cam0_path) else ""
            res.cam1_path = cam1_path if os.path.exists(cam1_path) else ""
            res.message = (
                "CAPTURE FAILED\n"
                f"cam0_ok={ok0} path={res.cam0_path}\n{d0}\n"
                f"cam1_ok={ok1} path={res.cam1_path}\n{d1}\n"
                "If stderr mentions pipeline busy, increase pause_wait_s.\n"
            )
            self.get_logger().error(res.message)
            return res

        res.success = True
        res.message = "OK"
        res.cam0_path = cam0_path
        res.cam1_path = cam1_path
        self.get_logger().info("Capture OK")
        return res


def main():
    rclpy.init()
    node = CaptureService()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
