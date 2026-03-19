#!/usr/bin/env python3
import os
import re
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
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy


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

    if "Available cameras:" in text:
        lines = text.splitlines()
        count = 0
        start = False
        for line in lines:
            if not start:
                if "Available cameras:" in line:
                    start = True
                continue
            if re.match(r"^\s*\d+:", line):
                count += 1
        return count

    return None


def _libcamera_camera_count() -> Optional[int]:
    # Returns number of cameras if a libcamera CLI is available, else None.
    cmds = (
        ["cam", "-l"],
        ["cam", "--list"],
        ["libcamera-hello", "--list-cameras"],
        ["rpicam-hello", "--list-cameras"],
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
        self.declare_parameter("auto_detect_cameras", True)
        self.declare_parameter("fallback_black_previews", True)

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
        self._fallback_pub0 = None
        self._fallback_pub1 = None
        self._fallback_timer = None
        self._fallback_w = 0
        self._fallback_h = 0
        self._fallback_data: Optional[bytes] = None
        self._detected_cam_count: Optional[int] = None
        self._expected_preview_cams: Optional[int] = None

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
            self._start_black_previews()
        else:
            self.get_logger().warn("No cameras detected. Previews disabled.")

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

    def _start_black_previews(self) -> None:
        if self._fallback_timer is not None:
            return
        ns0 = str(self.get_parameter("cam0_namespace").value)
        ns1 = str(self.get_parameter("cam1_namespace").value)
        n0 = str(self.get_parameter("cam0_node_name").value)
        n1 = str(self.get_parameter("cam1_node_name").value)
        w = int(self.get_parameter("preview_width").value)
        h = int(self.get_parameter("preview_height").value)
        fps = max(1, int(self.get_parameter("preview_fps").value))

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._fallback_pub0 = self.create_publisher(Image, self._preview_topic(ns0, n0), qos)
        self._fallback_pub1 = self.create_publisher(Image, self._preview_topic(ns1, n1), qos)
        self._fallback_w = w
        self._fallback_h = h
        self._fallback_data = bytes(w * h * 3)

        period = 1.0 / float(fps)
        self._fallback_timer = self.create_timer(period, self._publish_black_previews)
        self.get_logger().info(
            f"Black preview publishers running: {self._preview_topic(ns0, n0)} | {self._preview_topic(ns1, n1)}"
        )

    def _stop_black_previews(self) -> None:
        if self._fallback_timer is not None:
            try:
                self._fallback_timer.cancel()
            except Exception:
                pass
        self._fallback_timer = None
        self._fallback_pub0 = None
        self._fallback_pub1 = None
        self._fallback_data = None

    def _publish_black_previews(self) -> None:
        if self._fallback_pub0 is None or self._fallback_pub1 is None or self._fallback_data is None:
            return
        stamp = now_ros_time(self)
        msg0 = Image()
        msg0.header.stamp = stamp
        msg0.header.frame_id = "cam0_optical_frame"
        msg0.height = self._fallback_h
        msg0.width = self._fallback_w
        msg0.encoding = "bgr8"
        msg0.is_bigendian = False
        msg0.step = self._fallback_w * 3
        msg0.data = self._fallback_data

        msg1 = Image()
        msg1.header.stamp = stamp
        msg1.header.frame_id = "cam1_optical_frame"
        msg1.height = self._fallback_h
        msg1.width = self._fallback_w
        msg1.encoding = "bgr8"
        msg1.is_bigendian = False
        msg1.step = self._fallback_w * 3
        msg1.data = self._fallback_data

        self._fallback_pub0.publish(msg0)
        self._fallback_pub1.publish(msg1)

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
            "format": "RGB888",
            "FrameDurationLimits": [frame_us, frame_us],
            "use_node_time": False,
            "frame_id": frame_id,
        }

    def _preview_env(self) -> dict:
        env = os.environ.copy()
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
                self._stop_previews_managed()
                self._start_black_previews()
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
            self.get_logger().warn(
                f"Preview camera nodes exited early ({', '.join(dead)}). "
                "Falling back to black previews."
            )
            self._stop_previews_managed()
            self._start_black_previews()

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
            p = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout_s, env=self._preview_env())
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
                    except subprocess.TimeoutExpired:
                        p0.kill()
                        out0, err0 = "", "TimeoutExpired"
                    out1 = err1 = ""
                    if p1 is not None:
                        try:
                            out1, err1 = p1.communicate(timeout=run_timeout_s)
                        except subprocess.TimeoutExpired:
                            p1.kill()
                            out1, err1 = "", "TimeoutExpired"

                    ok0 = (p0.returncode == 0) and os.path.exists(cam0_path) and os.path.getsize(cam0_path) > 0
                    if allow_cam1 and p1 is not None:
                        ok1 = (p1.returncode == 0) and os.path.exists(cam1_path) and os.path.getsize(cam1_path) > 0
                    else:
                        ok1 = True
                    if not ok0:
                        d0 = f"attempt={attempt+1}/{retries+1} cam=0 rc={p0.returncode}\nstdout:\n{out0}\nstderr:\n{err0}\n"
                    if allow_cam1 and not ok1 and p1 is not None:
                        d1 = f"attempt={attempt+1}/{retries+1} cam=1 rc={p1.returncode}\nstdout:\n{out1}\nstderr:\n{err1}\n"
                else:
                    ok0, d0 = self._run_one(cam0, cam0_path, quality, run_timeout_s)
                    if allow_cam1:
                        ok1, d1 = self._run_one(cam1, cam1_path, quality, run_timeout_s)
                    else:
                        ok1 = True

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
            return False, fail_msg, fail_cam0, fail_cam1, stamp

        if not allow_cam1:
            cam1_path = ""
            msg = "OK (cam1 skipped: only one camera detected)"
        else:
            msg = "OK"

        self.get_logger().info("Capture OK")
        return True, msg, cam0_path, cam1_path, stamp

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
            node._stop_black_previews()
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        rclpy.try_shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
