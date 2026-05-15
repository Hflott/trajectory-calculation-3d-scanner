#!/usr/bin/env python3
import math
import threading
import time
from typing import Any, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu


class Bno085ImuNode(Node):
    def __init__(self) -> None:
        super().__init__("bno085_imu_node")
        self.declare_parameter("imu_topic", "/imu/data")
        self.declare_parameter("frame_id", "imu_link")
        self.declare_parameter("rate_hz", 100.0)
        self.declare_parameter("i2c_address", 0x4A)
        self.declare_parameter("i2c_bus", 1)
        self.declare_parameter("enable_rotation", True)
        self.declare_parameter("enable_accel", True)
        self.declare_parameter("enable_gyro", True)
        # "read_end" approximates measurement time when HW sample timestamps are unavailable.
        self.declare_parameter("timestamp_mode", "read_end")  # read_start|read_end
        self.declare_parameter("orientation_covariance", 0.05)
        self.declare_parameter("angular_velocity_covariance", 0.02)
        self.declare_parameter("linear_acceleration_covariance", 0.1)
        self.declare_parameter("use_driver_cached_reads", True)
        self.declare_parameter("read_error_reset_threshold", 30)

        self._imu_topic = str(self.get_parameter("imu_topic").value)
        self._frame_id = str(self.get_parameter("frame_id").value)
        self._rate_hz = max(1.0, float(self.get_parameter("rate_hz").value))
        self._addr = int(self.get_parameter("i2c_address").value)
        self._i2c_bus = int(self.get_parameter("i2c_bus").value)
        if self._i2c_bus < 0:
            self.get_logger().warn(f"Invalid i2c_bus={self._i2c_bus}; falling back to 1")
            self._i2c_bus = 1
        self._en_rot = bool(self.get_parameter("enable_rotation").value)
        self._en_acc = bool(self.get_parameter("enable_accel").value)
        self._en_gyr = bool(self.get_parameter("enable_gyro").value)
        stamp_mode = str(self.get_parameter("timestamp_mode").value).strip().lower()
        self._stamp_mode = stamp_mode if stamp_mode in ("read_start", "read_end") else "read_end"
        self._cov_o = float(self.get_parameter("orientation_covariance").value)
        self._cov_w = float(self.get_parameter("angular_velocity_covariance").value)
        self._cov_a = float(self.get_parameter("linear_acceleration_covariance").value)
        self._use_driver_cached_reads = bool(self.get_parameter("use_driver_cached_reads").value)
        self._read_error_reset_threshold = max(
            1, int(self.get_parameter("read_error_reset_threshold").value)
        )

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=30,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._pub = self.create_publisher(Imu, self._imu_topic, qos)

        # All sensor state is protected by _sensor_lock so the read thread and
        # the ROS executor reconnect timer never race on these fields.
        self._sensor_lock = threading.Lock()
        self._sensor = None
        self._i2c = None
        self._lib_error: Optional[str] = None
        self._last_warn = ""
        self._enabled_rotation = False
        self._enabled_accel = False
        self._enabled_gyro = False
        self._read_error_count = 0
        self._supports_cached_reads = False
        self._bno_report_accel: Optional[int] = None
        self._bno_report_gyro: Optional[int] = None
        self._bno_report_rot: Optional[int] = None

        self._stop_event = threading.Event()
        self._try_init_sensor()
        # Periodic reconnect check runs in the ROS executor (low frequency, non-critical).
        self.create_timer(3.0, self._try_reconnect_if_needed)
        # Dedicated OS thread drives IMU reads at the target rate, bypassing ROS
        # executor scheduling jitter that caps effective rate to ~30 Hz in Python.
        self._read_thread = threading.Thread(
            target=self._read_loop,
            name="bno085_imu_read",
            daemon=True,
        )
        self._read_thread.start()

    def destroy_node(self) -> None:
        self._stop_event.set()
        if hasattr(self, "_read_thread") and self._read_thread.is_alive():
            self._read_thread.join(timeout=2.0)
        with self._sensor_lock:
            self._close_sensor_locked()
        super().destroy_node()

    def _close_sensor_locked(self) -> None:
        self._sensor = None
        self._supports_cached_reads = False
        i2c = self._i2c
        self._i2c = None
        if i2c is None:
            return
        for method in ("deinit", "close", "unlock"):
            fn = getattr(i2c, method, None)
            if callable(fn):
                try:
                    fn()
                except Exception:
                    pass

    def _warn_once(self, text: str) -> None:
        if text != self._last_warn:
            self._last_warn = text
            self.get_logger().warn(text)

    def _try_init_sensor(self) -> None:
        try:
            import board
            import busio
            from adafruit_bno08x import (
                BNO_REPORT_ACCELEROMETER,
                BNO_REPORT_GYROSCOPE,
                BNO_REPORT_ROTATION_VECTOR,
            )
            from adafruit_bno08x.i2c import BNO08X_I2C
        except Exception as e:
            with self._sensor_lock:
                self._sensor = None
            self._lib_error = (
                f"BNO085 Python libs not available ({e}). "
                "Install on Pi: sudo pip3 install adafruit-blinka adafruit-circuitpython-bno08x"
            )
            self._warn_once(self._lib_error)
            return

        self._lib_error = None
        i2c = None
        try:
            if self._i2c_bus == 1:
                i2c = busio.I2C(board.SCL, board.SDA)
                i2c_path = "/dev/i2c-1 (board.SCL/board.SDA)"
            else:
                try:
                    from adafruit_extended_bus import ExtendedI2C
                except Exception as e:
                    raise RuntimeError(
                        "i2c_bus is not 1, but adafruit_extended_bus is unavailable "
                        f"({e}). Install on Pi: sudo pip3 install --break-system-packages adafruit-extended-bus"
                    ) from e
                i2c = ExtendedI2C(self._i2c_bus)
                i2c_path = f"/dev/i2c-{self._i2c_bus}"

            sensor = BNO08X_I2C(i2c, address=self._addr)
            # The BNO085 can be slow to settle after a warm process restart.
            # A short pause avoids feature-enable races on software I2C buses.
            time.sleep(0.2)
            bno_report_accel = int(BNO_REPORT_ACCELEROMETER)
            bno_report_gyro = int(BNO_REPORT_GYROSCOPE)
            bno_report_rot = int(BNO_REPORT_ROTATION_VECTOR)
            # Interval in microseconds expected by Adafruit API.
            interval_us = max(5_000, int(1_000_000.0 / self._rate_hz))
            enabled_rotation = False
            enabled_accel = False
            enabled_gyro = False

            if self._en_rot:
                sensor.enable_feature(BNO_REPORT_ROTATION_VECTOR, interval_us)
                enabled_rotation = True
            if self._en_acc:
                sensor.enable_feature(BNO_REPORT_ACCELEROMETER, interval_us)
                enabled_accel = True
            if self._en_gyr:
                sensor.enable_feature(BNO_REPORT_GYROSCOPE, interval_us)
                enabled_gyro = True

            supports_cached = bool(
                self._use_driver_cached_reads
                and hasattr(sensor, "_process_available_packets")
                and hasattr(sensor, "_readings")
            )
            # Write all fields atomically; assign _sensor last so the read thread
            # only picks up a fully-configured sensor object.
            with self._sensor_lock:
                self._bno_report_accel = bno_report_accel
                self._bno_report_gyro = bno_report_gyro
                self._bno_report_rot = bno_report_rot
                self._enabled_rotation = enabled_rotation
                self._enabled_accel = enabled_accel
                self._enabled_gyro = enabled_gyro
                self._supports_cached_reads = supports_cached
                self._read_error_count = 0
                old_i2c = self._i2c
                self._i2c = i2c
                self._sensor = sensor
            if old_i2c is not None and old_i2c is not i2c:
                for method in ("deinit", "close", "unlock"):
                    fn = getattr(old_i2c, method, None)
                    if callable(fn):
                        try:
                            fn()
                        except Exception:
                            pass

            self.get_logger().info(
                f"BNO085 ready on I2C {i2c_path} address 0x{self._addr:02X}; "
                f"features: rot={enabled_rotation} acc={enabled_accel} gyro={enabled_gyro}; "
                f"publishing {self._imu_topic} at ~{self._rate_hz:.1f} Hz "
                f"(cached_reads={supports_cached}) "
                f"(timestamp_mode={self._stamp_mode})"
            )
        except Exception as e:
            with self._sensor_lock:
                self._close_sensor_locked()
            if i2c is not None:
                for method in ("deinit", "close", "unlock"):
                    fn = getattr(i2c, method, None)
                    if callable(fn):
                        try:
                            fn()
                        except Exception:
                            pass
            self._warn_once(
                f"BNO085 init failed on /dev/i2c-{self._i2c_bus} at 0x{self._addr:02X}: {e} "
                f"(check wiring and i2cdetect -y -r {self._i2c_bus})"
            )

    def _try_reconnect_if_needed(self) -> None:
        with self._sensor_lock:
            needs_reconnect = self._sensor is None
        if needs_reconnect:
            self._try_init_sensor()

    def _read_loop(self) -> None:
        """Dedicated OS thread: reads IMU and publishes at _rate_hz without executor jitter."""
        period_s = 1.0 / self._rate_hz
        deadline = time.perf_counter() + period_s
        while not self._stop_event.is_set():
            self._publish_tick()
            now = time.perf_counter()
            remaining = deadline - now
            deadline += period_s
            # If we fell badly behind (long I2C stall), reset rather than spinning to catch up.
            if remaining < -2.0 * period_s:
                deadline = now + period_s
            elif remaining > 0.001:
                # Sleep leaving ~1 ms margin for OS wakeup latency.
                time.sleep(remaining - 0.001)

    def _read_sensor_values(
        self,
        sensor: Any,
        supports_cached: bool,
        enabled_rotation: bool,
        enabled_accel: bool,
        enabled_gyro: bool,
        bno_report_rot: Optional[int],
        bno_report_gyro: Optional[int],
        bno_report_accel: Optional[int],
    ) -> Tuple[
        Optional[Tuple[float, float, float, float]],
        Optional[Tuple[float, float, float]],
        Optional[Tuple[float, float, float]],
    ]:
        quat_raw: Any = None
        gyro_raw: Any = None
        accel_raw: Any = None

        if supports_cached:
            sensor._process_available_packets()
            readings = getattr(sensor, "_readings", {})
            if enabled_rotation and bno_report_rot is not None:
                quat_raw = readings.get(bno_report_rot)
            if enabled_gyro and bno_report_gyro is not None:
                gyro_raw = readings.get(bno_report_gyro)
            if enabled_accel and bno_report_accel is not None:
                accel_raw = readings.get(bno_report_accel)
        else:
            if enabled_rotation:
                quat_raw = sensor.quaternion
            if enabled_gyro:
                gyro_raw = sensor.gyro
            if enabled_accel:
                accel_raw = sensor.acceleration

        quat = self._safe_quat(quat_raw) if quat_raw is not None else None
        gyro = self._safe_triplet(gyro_raw) if gyro_raw is not None else None
        accel = self._safe_triplet(accel_raw) if accel_raw is not None else None
        return quat, gyro, accel

    def _safe_triplet(self, value) -> Optional[Tuple[float, float, float]]:
        try:
            x, y, z = value
            return float(x), float(y), float(z)
        except Exception:
            return None

    def _safe_quat(self, value) -> Optional[Tuple[float, float, float, float]]:
        try:
            x, y, z, w = value
            x = float(x)
            y = float(y)
            z = float(z)
            w = float(w)
            n = math.sqrt(x * x + y * y + z * z + w * w)
            if n > 1e-8:
                inv = 1.0 / n
                return x * inv, y * inv, z * inv, w * inv
            return x, y, z, w
        except Exception:
            return None

    def _publish_tick(self) -> None:
        # Snapshot all sensor-related state under the lock; I2C reads happen outside it.
        with self._sensor_lock:
            sensor = self._sensor
            supports_cached = self._supports_cached_reads
            enabled_rotation = self._enabled_rotation
            enabled_accel = self._enabled_accel
            enabled_gyro = self._enabled_gyro
            bno_report_rot = self._bno_report_rot
            bno_report_gyro = self._bno_report_gyro
            bno_report_accel = self._bno_report_accel

        if sensor is None:
            return

        msg = Imu()
        if self._stamp_mode == "read_start":
            msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id

        # Defaults for unknown fields per sensor_msgs/Imu conventions.
        msg.orientation_covariance[0] = -1.0
        msg.angular_velocity_covariance[0] = -1.0
        msg.linear_acceleration_covariance[0] = -1.0

        try:
            q, g, a = self._read_sensor_values(
                sensor, supports_cached,
                enabled_rotation, enabled_accel, enabled_gyro,
                bno_report_rot, bno_report_gyro, bno_report_accel,
            )
        except Exception as e:
            with self._sensor_lock:
                self._read_error_count += 1
                error_count = self._read_error_count
                if error_count >= self._read_error_reset_threshold:
                    self._close_sensor_locked()
            if error_count >= self._read_error_reset_threshold:
                self._warn_once(
                    f"BNO085 read error: {e}; reached {error_count} consecutive errors, "
                    "resetting sensor and retrying init"
                )
            elif error_count in (1, 5, 10):
                self.get_logger().warn(
                    f"BNO085 transient read error ({error_count}/"
                    f"{self._read_error_reset_threshold}): {e}"
                )
            return
        else:
            with self._sensor_lock:
                prev_count = self._read_error_count
                self._read_error_count = 0
            if prev_count:
                self.get_logger().info(
                    f"BNO085 read recovered after {prev_count} consecutive errors"
                )

        if enabled_rotation and q is not None:
            msg.orientation.x = q[0]
            msg.orientation.y = q[1]
            msg.orientation.z = q[2]
            msg.orientation.w = q[3]
            msg.orientation_covariance = [
                self._cov_o, 0.0, 0.0,
                0.0, self._cov_o, 0.0,
                0.0, 0.0, self._cov_o,
            ]

        if enabled_gyro and g is not None:
            msg.angular_velocity.x = g[0]
            msg.angular_velocity.y = g[1]
            msg.angular_velocity.z = g[2]
            msg.angular_velocity_covariance = [
                self._cov_w, 0.0, 0.0,
                0.0, self._cov_w, 0.0,
                0.0, 0.0, self._cov_w,
            ]

        if enabled_accel and a is not None:
            msg.linear_acceleration.x = a[0]
            msg.linear_acceleration.y = a[1]
            msg.linear_acceleration.z = a[2]
            msg.linear_acceleration_covariance = [
                self._cov_a, 0.0, 0.0,
                0.0, self._cov_a, 0.0,
                0.0, 0.0, self._cov_a,
            ]

        if self._stamp_mode == "read_end":
            msg.header.stamp = self.get_clock().now().to_msg()
        self._pub.publish(msg)


def main(args=None) -> int:
    rclpy.init(args=args)
    node = Bno085ImuNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
