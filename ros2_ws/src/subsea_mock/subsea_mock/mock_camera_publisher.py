#!/usr/bin/env python3
import math
import time

import numpy as np
import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from sensor_msgs.msg import Image


def _make_frame(w: int, h: int, t: float, hue_offset: float) -> np.ndarray:
    # Simple animated gradient to make motion obvious in the UI.
    x = np.linspace(0, 1, w, dtype=np.float32)
    y = np.linspace(0, 1, h, dtype=np.float32)[:, None]
    phase = (t * 0.3 + hue_offset) % 1.0
    r = (0.5 + 0.5 * np.sin(2 * math.pi * (x + phase)))[None, :]
    g = (0.5 + 0.5 * np.sin(2 * math.pi * (y + phase)))[..., None]
    b = (0.5 + 0.5 * np.sin(2 * math.pi * (x + y + phase)))

    frame = np.zeros((h, w, 3), dtype=np.uint8)
    frame[..., 2] = (r * 255).astype(np.uint8)
    frame[..., 1] = (g * 255).astype(np.uint8).reshape(h, 1)
    frame[..., 0] = (b * 255).astype(np.uint8)
    return frame


class MockCameraPublisher(Node):
    def __init__(self):
        super().__init__("mock_camera_publisher")

        self.declare_parameter("width", 960)
        self.declare_parameter("height", 540)
        self.declare_parameter("fps", 15)
        self.declare_parameter("cam0_topic", "/cam0/camera/image_raw")
        self.declare_parameter("cam1_topic", "/cam1/camera/image_raw")

        self._w = int(self.get_parameter("width").value)
        self._h = int(self.get_parameter("height").value)
        self._fps = max(1, int(self.get_parameter("fps").value))
        self._cam0_topic = str(self.get_parameter("cam0_topic").value)
        self._cam1_topic = str(self.get_parameter("cam1_topic").value)

        self._pub0 = self.create_publisher(Image, self._cam0_topic, 1)
        self._pub1 = self.create_publisher(Image, self._cam1_topic, 1)

        self._t0 = time.time()
        self._timer = self.create_timer(1.0 / self._fps, self._on_timer)
        self.add_on_set_parameters_callback(self._on_set_parameters)
        self.get_logger().info(
            f"Mock camera publisher ready: {self._cam0_topic}, {self._cam1_topic} @ {self._fps} FPS"
        )

    def _on_set_parameters(self, params):
        new_fps = self._fps
        for p in params:
            if p.name == "fps":
                try:
                    new_fps = max(1, int(p.value))
                except Exception:
                    return SetParametersResult(successful=False, reason="fps must be int >= 1")

        if new_fps != self._fps:
            self._fps = new_fps
            try:
                self._timer.cancel()
            except Exception:
                pass
            self._timer = self.create_timer(1.0 / self._fps, self._on_timer)
            self.get_logger().info(f"Updated mock camera fps={self._fps}")

        return SetParametersResult(successful=True)

    def _publish(self, pub: rclpy.publisher.Publisher, frame: np.ndarray) -> None:
        msg = Image()
        msg.height = frame.shape[0]
        msg.width = frame.shape[1]
        msg.encoding = "bgr8"
        msg.is_bigendian = False
        msg.step = frame.shape[1] * 3
        msg.data = frame.tobytes()
        pub.publish(msg)

    def _on_timer(self) -> None:
        t = time.time() - self._t0
        frame0 = _make_frame(self._w, self._h, t, 0.0)
        frame1 = _make_frame(self._w, self._h, t, 0.4)
        self._publish(self._pub0, frame0)
        self._publish(self._pub1, frame1)


def main():
    rclpy.init()
    node = MockCameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
