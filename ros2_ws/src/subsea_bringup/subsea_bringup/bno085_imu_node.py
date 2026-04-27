#!/usr/bin/env python3
import math
from typing import Optional, Tuple

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
        self.declare_parameter("enable_rotation", True)
        self.declare_parameter("enable_accel", True)
        self.declare_parameter("enable_gyro", True)
        self.declare_parameter("orientation_covariance", 0.05)
        self.declare_parameter("angular_velocity_covariance", 0.02)
        self.declare_parameter("linear_acceleration_covariance", 0.1)

        self._imu_topic = str(self.get_parameter("imu_topic").value)
        self._frame_id = str(self.get_parameter("frame_id").value)
        self._rate_hz = max(1.0, float(self.get_parameter("rate_hz").value))
        self._addr = int(self.get_parameter("i2c_address").value)
        self._en_rot = bool(self.get_parameter("enable_rotation").value)
        self._en_acc = bool(self.get_parameter("enable_accel").value)
        self._en_gyr = bool(self.get_parameter("enable_gyro").value)
        self._cov_o = float(self.get_parameter("orientation_covariance").value)
        self._cov_w = float(self.get_parameter("angular_velocity_covariance").value)
        self._cov_a = float(self.get_parameter("linear_acceleration_covariance").value)

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=30,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._pub = self.create_publisher(Imu, self._imu_topic, qos)

        self._sensor = None
        self._lib_error: Optional[str] = None
        self._last_warn = ""
        self._enabled_rotation = False
        self._enabled_accel = False
        self._enabled_gyro = False

        self._try_init_sensor()
        self.create_timer(3.0, self._try_reconnect_if_needed)
        self.create_timer(1.0 / self._rate_hz, self._publish_tick)

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
            self._sensor = None
            self._lib_error = (
                f"BNO085 Python libs not available ({e}). "
                "Install on Pi: sudo pip3 install adafruit-blinka adafruit-circuitpython-bno08x"
            )
            self._warn_once(self._lib_error)
            return

        self._lib_error = None
        try:
            i2c = busio.I2C(board.SCL, board.SDA)
            sensor = BNO08X_I2C(i2c, address=self._addr)
            # Interval in microseconds expected by Adafruit API.
            interval_us = max(5_000, int(1_000_000.0 / self._rate_hz))
            self._enabled_rotation = False
            self._enabled_accel = False
            self._enabled_gyro = False

            if self._en_rot:
                sensor.enable_feature(BNO_REPORT_ROTATION_VECTOR, interval_us)
                self._enabled_rotation = True
            if self._en_acc:
                sensor.enable_feature(BNO_REPORT_ACCELEROMETER, interval_us)
                self._enabled_accel = True
            if self._en_gyr:
                sensor.enable_feature(BNO_REPORT_GYROSCOPE, interval_us)
                self._enabled_gyro = True

            self._sensor = sensor
            self.get_logger().info(
                f"BNO085 ready on I2C address 0x{self._addr:02X}; "
                f"features: rot={self._enabled_rotation} acc={self._enabled_accel} gyro={self._enabled_gyro}; "
                f"publishing {self._imu_topic} at ~{self._rate_hz:.1f} Hz"
            )
        except Exception as e:
            self._sensor = None
            self._warn_once(
                f"BNO085 init failed at 0x{self._addr:02X}: {e} "
                "(check I2C wiring and i2cdetect -y -r 1)"
            )

    def _try_reconnect_if_needed(self) -> None:
        if self._sensor is None:
            self._try_init_sensor()

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
        if self._sensor is None:
            return

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id

        # Defaults for unknown fields per sensor_msgs/Imu conventions.
        msg.orientation_covariance[0] = -1.0
        msg.angular_velocity_covariance[0] = -1.0
        msg.linear_acceleration_covariance[0] = -1.0

        try:
            if self._enabled_rotation:
                q = self._safe_quat(self._sensor.quaternion)
                if q is not None:
                    msg.orientation.x = q[0]
                    msg.orientation.y = q[1]
                    msg.orientation.z = q[2]
                    msg.orientation.w = q[3]
                    msg.orientation_covariance = [
                        self._cov_o,
                        0.0,
                        0.0,
                        0.0,
                        self._cov_o,
                        0.0,
                        0.0,
                        0.0,
                        self._cov_o,
                    ]

            if self._enabled_gyro:
                g = self._safe_triplet(self._sensor.gyro)
                if g is not None:
                    msg.angular_velocity.x = g[0]
                    msg.angular_velocity.y = g[1]
                    msg.angular_velocity.z = g[2]
                    msg.angular_velocity_covariance = [
                        self._cov_w,
                        0.0,
                        0.0,
                        0.0,
                        self._cov_w,
                        0.0,
                        0.0,
                        0.0,
                        self._cov_w,
                    ]

            if self._enabled_accel:
                a = self._safe_triplet(self._sensor.acceleration)
                if a is not None:
                    msg.linear_acceleration.x = a[0]
                    msg.linear_acceleration.y = a[1]
                    msg.linear_acceleration.z = a[2]
                    msg.linear_acceleration_covariance = [
                        self._cov_a,
                        0.0,
                        0.0,
                        0.0,
                        self._cov_a,
                        0.0,
                        0.0,
                        0.0,
                        self._cov_a,
                    ]
        except Exception as e:
            self._sensor = None
            self._warn_once(f"BNO085 read error: {e}; will retry init")
            return

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
