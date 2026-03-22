#!/usr/bin/env python3
import json
import socket
import threading
import time
from datetime import datetime, timezone
from typing import Optional, Tuple

import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import NavSatFix, NavSatStatus


def _to_stamp(tpv_time: Optional[str]) -> Optional[TimeMsg]:
    if not tpv_time:
        return None
    try:
        text = str(tpv_time).strip()
        if text.endswith("Z"):
            text = text[:-1] + "+00:00"
        dt = datetime.fromisoformat(text)
        if dt.tzinfo is None:
            dt = dt.replace(tzinfo=timezone.utc)
        ts = dt.timestamp()
        sec = int(ts)
        nsec = int((ts - sec) * 1e9)
        if nsec < 0:
            nsec = 0
        if nsec >= 1_000_000_000:
            sec += 1
            nsec -= 1_000_000_000
        return TimeMsg(sec=sec, nanosec=nsec)
    except Exception:
        return None


def _to_float(v) -> Optional[float]:
    try:
        x = float(v)
    except Exception:
        return None
    if x != x:  # NaN
        return None
    return x


class GpsdJsonFixBridge(Node):
    def __init__(self) -> None:
        super().__init__("gpsd_json_fix_bridge")
        self.declare_parameter("host", "127.0.0.1")
        self.declare_parameter("port", 2947)
        self.declare_parameter("fix_topic", "/fix")
        self.declare_parameter("frame_id", "gps")
        self.declare_parameter("reconnect_s", 1.0)
        self.declare_parameter("publish_no_fix", False)

        self._host = str(self.get_parameter("host").value)
        self._port = int(self.get_parameter("port").value)
        self._fix_topic = str(self.get_parameter("fix_topic").value)
        self._frame_id = str(self.get_parameter("frame_id").value)
        self._reconnect_s = max(0.2, float(self.get_parameter("reconnect_s").value))
        self._publish_no_fix = bool(self.get_parameter("publish_no_fix").value)

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._pub = self.create_publisher(NavSatFix, self._fix_topic, qos)

        self._stop_evt = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        self._got_first_fix = False
        self.get_logger().info(
            f"gpsd_json_fix_bridge started: host={self._host} port={self._port} topic={self._fix_topic}"
        )

    def destroy_node(self):
        self._stop_evt.set()
        try:
            if self._thread.is_alive():
                self._thread.join(timeout=1.0)
        except Exception:
            pass
        return super().destroy_node()

    def _run(self) -> None:
        while rclpy.ok() and (not self._stop_evt.is_set()):
            try:
                self._stream_once()
            except Exception as e:
                self.get_logger().warn(f"gpsd stream error: {e}")
            if self._stop_evt.wait(self._reconnect_s):
                return

    def _stream_once(self) -> None:
        sock = socket.create_connection((self._host, self._port), timeout=5.0)
        try:
            sock.settimeout(5.0)
            watch = '?WATCH={"enable":true,"json":true}\n'
            sock.sendall(watch.encode("ascii"))
            f = sock.makefile("r", encoding="utf-8", errors="ignore")
            for raw_line in f:
                if self._stop_evt.is_set() or (not rclpy.ok()):
                    return
                line = raw_line.strip()
                if not line:
                    continue
                try:
                    msg = json.loads(line)
                except Exception:
                    continue
                if msg.get("class") != "TPV":
                    continue
                self._publish_from_tpv(msg)
        finally:
            try:
                sock.close()
            except Exception:
                pass

    def _publish_from_tpv(self, tpv: dict) -> None:
        mode = int(tpv.get("mode", 0) or 0)
        if mode < 2 and (not self._publish_no_fix):
            return

        out = NavSatFix()
        out.header.frame_id = self._frame_id
        stamp = _to_stamp(tpv.get("time"))
        out.header.stamp = stamp if stamp is not None else self.get_clock().now().to_msg()

        if mode < 2:
            out.status.status = NavSatStatus.STATUS_NO_FIX
            out.status.service = NavSatStatus.SERVICE_GPS
            out.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
            self._pub.publish(out)
            return

        lat = _to_float(tpv.get("lat"))
        lon = _to_float(tpv.get("lon"))
        alt = _to_float(tpv.get("altMSL"))
        if alt is None:
            alt = _to_float(tpv.get("altHAE"))
        if alt is None:
            alt = _to_float(tpv.get("alt"))

        if lat is None or lon is None:
            return
        if alt is None:
            alt = 0.0

        out.status.status = NavSatStatus.STATUS_FIX
        out.status.service = NavSatStatus.SERVICE_GPS
        out.latitude = lat
        out.longitude = lon
        out.altitude = alt

        eph = _to_float(tpv.get("eph"))
        epv = _to_float(tpv.get("epv"))
        if eph is not None and eph > 0.0:
            sigma_h2 = eph * eph
            sigma_v2 = (epv * epv) if (epv is not None and epv > 0.0) else sigma_h2
            out.position_covariance = [
                sigma_h2,
                0.0,
                0.0,
                0.0,
                sigma_h2,
                0.0,
                0.0,
                0.0,
                sigma_v2,
            ]
            out.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        else:
            out.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

        self._pub.publish(out)
        if not self._got_first_fix:
            self._got_first_fix = True
            self.get_logger().info(
                f"First fix published: lat={out.latitude:.8f} lon={out.longitude:.8f} alt={out.altitude:.3f}"
            )


def main(args=None) -> int:
    rclpy.init(args=args)
    node = GpsdJsonFixBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
