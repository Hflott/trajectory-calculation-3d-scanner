#!/usr/bin/env python3
import os
import time
import subprocess
import signal
import threading
from datetime import datetime
from typing import List, Tuple, Optional, Callable

import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.node import Node
from builtin_interfaces.msg import Time as TimeMsg

from ament_index_python.packages import get_package_prefix

from subsea_interfaces.action import CapturePair as CapturePairAction
from subsea_interfaces.srv import CapturePair


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

        # Preview management
        self.declare_parameter("manage_previews", False)
        self.declare_parameter("start_previews", True)
        self.declare_parameter("pause_previews", True)

        self.declare_parameter("preview_width", 960)
        self.declare_parameter("preview_height", 540)
        self.declare_parameter("preview_fps", 20)
        self.declare_parameter("preview_role", "viewfinder")

        self.declare_parameter("cam0_namespace", "/cam0")
        self.declare_parameter("cam1_namespace", "/cam1")
        self.declare_parameter("cam0_node_name", "camera")
        self.declare_parameter("cam1_node_name", "camera")

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

        self._devs = _dev_paths()
        self._camera_node_exe: Optional[str] = None

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

        if bool(self.get_parameter("manage_previews").value) and bool(self.get_parameter("start_previews").value):
            self.get_logger().info("manage_previews:=true -> starting preview camera nodes")
            self._start_previews()

    def _preview_params(self, cam_index: int, frame_id: str) -> dict:
        w = int(self.get_parameter("preview_width").value)
        h = int(self.get_parameter("preview_height").value)
        fps = int(self.get_parameter("preview_fps").value)
        role = str(self.get_parameter("preview_role").value)
        frame_us = int(1_000_000 / max(1, fps))

        return {
            "camera": int(cam_index),
            "role": role,
            "width": w,
            "height": h,
            "FrameDurationLimits": [frame_us, frame_us],
            "use_node_time": False,
            "frame_id": frame_id,
        }

    def _start_preview_proc(self, cam_index: int, ns: str, node_name: str, params_path: str, frame_id: str) -> subprocess.Popen:
        if self._camera_node_exe is None:
            self._camera_node_exe = _camera_ros_exe_path()
        _write_params_file(params_path, self._preview_params(cam_index, frame_id))

        cmd = [
            self._camera_node_exe,
            "--ros-args",
            "-r", f"__node:={node_name}",
            "-r", f"__ns:={ns}",
            "--params-file", params_path,
        ]
        env = os.environ.copy()
        return _popen_group(cmd, env=env)

    def _start_previews(self) -> None:
        cam0 = int(self.get_parameter("cam0_index").value)
        cam1 = int(self.get_parameter("cam1_index").value)
        ns0 = str(self.get_parameter("cam0_namespace").value)
        ns1 = str(self.get_parameter("cam1_namespace").value)
        n0 = str(self.get_parameter("cam0_node_name").value)
        n1 = str(self.get_parameter("cam1_node_name").value)

        if self._p0 is None or self._p0.poll() is not None:
            self._p0 = self._start_preview_proc(cam0, ns0, n0, self._p0_params, "cam0_optical_frame")
            self.get_logger().info(f"cam0 preview started (pid={self._p0.pid})")

        if self._p1 is None or self._p1.poll() is not None:
            self._p1 = self._start_preview_proc(cam1, ns1, n1, self._p1_params, "cam1_optical_frame")
            self.get_logger().info(f"cam1 preview started (pid={self._p1.pid})")

        self.get_logger().info("Preview camera nodes start sequence complete")

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
            p = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout_s)
        except subprocess.TimeoutExpired:
            return False, f"TimeoutExpired running: {' '.join(cmd)}"

        ok = (p.returncode == 0) and os.path.exists(out_path) and os.path.getsize(out_path) > 0
        if ok:
            return True, ""
        return False, f"rc={p.returncode}\nstdout:\n{p.stdout}\nstderr:\n{p.stderr}\n"

    def _perform_capture(
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

        try:
            for attempt in range(retries + 1):
                if feedback_cb is not None:
                    feedback_cb(f"capturing_attempt_{attempt+1}")
                if parallel:
                    # Run both still captures in parallel (faster pause window)
                    p0 = subprocess.Popen(self._rpicam_cmd(cam0, cam0_path, quality), stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
                    p1 = subprocess.Popen(self._rpicam_cmd(cam1, cam1_path, quality), stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
                    try:
                        out0, err0 = p0.communicate(timeout=run_timeout_s)
                    except subprocess.TimeoutExpired:
                        p0.kill()
                        out0, err0 = "", "TimeoutExpired"
                    try:
                        out1, err1 = p1.communicate(timeout=run_timeout_s)
                    except subprocess.TimeoutExpired:
                        p1.kill()
                        out1, err1 = "", "TimeoutExpired"

                    ok0 = (p0.returncode == 0) and os.path.exists(cam0_path) and os.path.getsize(cam0_path) > 0
                    ok1 = (p1.returncode == 0) and os.path.exists(cam1_path) and os.path.getsize(cam1_path) > 0
                    if not ok0:
                        d0 = f"attempt={attempt+1}/{retries+1} cam=0 rc={p0.returncode}\nstdout:\n{out0}\nstderr:\n{err0}\n"
                    if not ok1:
                        d1 = f"attempt={attempt+1}/{retries+1} cam=1 rc={p1.returncode}\nstdout:\n{out1}\nstderr:\n{err1}\n"
                else:
                    ok0, d0 = self._run_one(cam0, cam0_path, quality, run_timeout_s)
                    ok1, d1 = self._run_one(cam1, cam1_path, quality, run_timeout_s)

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
            fail_cam1 = cam1_path if os.path.exists(cam1_path) else ""
            fail_msg = (
                "CAPTURE FAILED\n"
                f"cam0_ok={ok0} path={fail_cam0}\n{d0}\n"
                f"cam1_ok={ok1} path={fail_cam1}\n{d1}\n"
            )
            self.get_logger().error(fail_msg)
            return False, fail_msg, fail_cam0, fail_cam1, stamp

        self.get_logger().info("Capture OK")
        return True, "OK", cam0_path, cam1_path, stamp

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
            node.destroy_node()
        except Exception:
            pass
        rclpy.try_shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
