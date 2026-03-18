#!/usr/bin/env python3
import os
from datetime import datetime
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node

from subsea_interfaces.srv import CapturePair

try:
    import cv2  # type: ignore
except Exception:  # pragma: no cover - optional dependency
    cv2 = None

try:
    from PIL import Image as PilImage  # type: ignore
except Exception:  # pragma: no cover - optional dependency
    PilImage = None


def _make_image(w: int, h: int, bgr: tuple) -> np.ndarray:
    img = np.zeros((h, w, 3), dtype=np.uint8)
    img[:, :] = np.array(bgr, dtype=np.uint8)
    return img


def _write_jpeg(path: str, img: np.ndarray, quality: int) -> Optional[str]:
    if cv2 is not None:
        ok = cv2.imwrite(path, img, [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)])
        return None if ok else "cv2.imwrite failed"
    if PilImage is not None:
        rgb = img[:, :, ::-1]
        pil = PilImage.fromarray(rgb, mode="RGB")
        pil.save(path, format="JPEG", quality=int(quality))
        return None
    return "No JPEG writer available (install python3-opencv or python3-pillow)"


class MockCaptureService(Node):
    def __init__(self):
        super().__init__("mock_capture_service")
        self.declare_parameter("width", 1280)
        self.declare_parameter("height", 720)
        self.declare_parameter("default_quality", 90)
        self.declare_parameter("output_dir", os.path.expanduser("~/captures"))

        self.srv = self.create_service(CapturePair, "capture_pair", self.on_capture)
        self.get_logger().info("Mock capture service ready: /capture_pair")

    def on_capture(self, req: CapturePair.Request, res: CapturePair.Response) -> CapturePair.Response:
        width = int(self.get_parameter("width").value)
        height = int(self.get_parameter("height").value)
        default_quality = int(self.get_parameter("default_quality").value)

        out_dir = req.output_dir.strip() or str(self.get_parameter("output_dir").value)
        os.makedirs(out_dir, exist_ok=True)

        session = req.session_id.strip() or datetime.now().strftime("%Y%m%d_%H%M%S")
        cam0_path = os.path.join(out_dir, f"{session}_cam0.jpg")
        cam1_path = os.path.join(out_dir, f"{session}_cam1.jpg")

        quality = int(req.jpeg_quality) if req.jpeg_quality > 0 else default_quality

        img0 = _make_image(width, height, (20, 40, 200))
        img1 = _make_image(width, height, (20, 200, 40))

        err0 = _write_jpeg(cam0_path, img0, quality)
        err1 = _write_jpeg(cam1_path, img1, quality)

        if err0 or err1:
            res.success = False
            res.message = f"Mock capture failed: {err0 or ''} {err1 or ''}".strip()
            res.cam0_path = cam0_path if os.path.exists(cam0_path) else ""
            res.cam1_path = cam1_path if os.path.exists(cam1_path) else ""
            return res

        res.success = True
        res.message = "OK"
        res.cam0_path = cam0_path
        res.cam1_path = cam1_path
        return res


def main():
    rclpy.init()
    node = MockCaptureService()
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
