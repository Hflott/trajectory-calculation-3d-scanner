#!/usr/bin/env python3
import argparse
import json
import math
import os
import sys
import time
from typing import Any, Dict, Optional, Tuple


def _jsonable(value: Any) -> Any:
    if value is None or isinstance(value, (bool, int, float, str)):
        if isinstance(value, float) and not math.isfinite(value):
            return str(value)
        return value
    if isinstance(value, dict):
        return {str(k): _jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(v) for v in value]
    return repr(value)


def _parse_awbgains(value: str) -> Optional[Tuple[float, float]]:
    if not value:
        return None
    parts = [p.strip() for p in value.replace(";", ",").split(",") if p.strip()]
    if len(parts) != 2:
        return None
    try:
        return float(parts[0]), float(parts[1])
    except Exception:
        return None


def _awb_mode_value(name: str) -> Optional[Any]:
    if not name:
        return None
    try:
        from libcamera import controls
    except Exception:
        return None

    key = name.strip().lower().replace("-", "_")
    mapping = {
        "auto": "Auto",
        "normal": "Auto",
        "incandescent": "Incandescent",
        "tungsten": "Tungsten",
        "fluorescent": "Fluorescent",
        "indoor": "Indoor",
        "daylight": "Daylight",
        "cloudy": "Cloudy",
        "custom": "Custom",
    }
    attr = mapping.get(key)
    if not attr:
        return None
    return getattr(controls.AwbModeEnum, attr, None)


def _write_metadata(path: str, payload: Dict[str, Any]) -> None:
    if not path:
        return
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(_jsonable(payload), f, indent=2, sort_keys=True)
    os.replace(tmp, path)


def capture(args: argparse.Namespace) -> Dict[str, Any]:
    try:
        from picamera2 import Picamera2
    except Exception as e:
        return {
            "backend": "picamera2",
            "status": "picamera2_import_failed",
            "error": str(e),
        }

    payload: Dict[str, Any] = {
        "backend": "picamera2",
        "status": "starting",
        "camera_index": int(args.camera),
        "image_path": args.output,
        "metadata_path": args.metadata_json,
        "width": int(args.width),
        "height": int(args.height),
        "quality": int(args.quality),
        "warmup_ms": int(args.warmup_ms),
        "timeout_ms": int(args.timeout_ms),
    }

    picam2 = None
    try:
        capture_start_system_ns = time.time_ns()
        capture_start_monotonic_ns = time.monotonic_ns()

        picam2 = Picamera2(camera_num=int(args.camera))
        try:
            config = picam2.create_still_configuration(
                main={"size": (int(args.width), int(args.height))},
                buffer_count=1,
                display=None,
            )
        except TypeError:
            config = picam2.create_still_configuration(
                main={"size": (int(args.width), int(args.height))},
                buffer_count=1,
            )
        picam2.configure(config)
        try:
            picam2.options["quality"] = int(args.quality)
        except Exception:
            pass

        controls: Dict[str, Any] = {}
        awb_mode = _awb_mode_value(args.awb)
        if awb_mode is not None:
            controls["AwbMode"] = awb_mode
        gains = _parse_awbgains(args.awbgains)
        if gains is not None:
            controls["AwbEnable"] = False
            controls["ColourGains"] = gains
        if abs(float(args.saturation) - 1.0) >= 0.001:
            controls["Saturation"] = float(args.saturation)
        if controls:
            picam2.set_controls(controls)
            payload["requested_controls"] = {
                key: _jsonable(value) for key, value in controls.items()
            }

        picam2.start()
        if int(args.warmup_ms) > 0:
            time.sleep(float(args.warmup_ms) / 1000.0)

        request = picam2.capture_request()
        request_system_ns = time.time_ns()
        request_monotonic_ns = time.monotonic_ns()
        metadata = request.get_metadata()
        try:
            request.save("main", args.output)
        finally:
            request.release()

        capture_done_system_ns = time.time_ns()
        capture_done_monotonic_ns = time.monotonic_ns()

        exposure_us = metadata.get("ExposureTime")
        sensor_timestamp_ns = metadata.get("SensorTimestamp")
        try:
            exposure_us_int = int(round(float(exposure_us))) if exposure_us is not None else None
        except Exception:
            exposure_us_int = None
        try:
            sensor_timestamp_ns_int = int(round(float(sensor_timestamp_ns))) if sensor_timestamp_ns is not None else None
        except Exception:
            sensor_timestamp_ns_int = None

        # libcamera SensorTimestamp is on a monotonic clock. Convert it to system
        # clock using the observed system-monotonic offset around request handling.
        clock_offset_ns = int(round(
            (
                (request_system_ns - request_monotonic_ns)
                + (capture_done_system_ns - capture_done_monotonic_ns)
            ) / 2.0
        ))
        sensor_timestamp_system_ns = None
        exposure_midpoint_system_ns = None
        if sensor_timestamp_ns_int is not None:
            sensor_timestamp_system_ns = int(sensor_timestamp_ns_int + clock_offset_ns)
            if exposure_us_int is not None:
                # SensorTimestamp/FrameWallClock represent the frame produced/readout
                # time, so move back half an exposure for the midpoint estimate.
                exposure_midpoint_system_ns = int(sensor_timestamp_system_ns - (exposure_us_int * 1000) // 2)

        payload.update(
            {
                "status": "ok",
                "capture_start_system_ns": int(capture_start_system_ns),
                "capture_start_monotonic_ns": int(capture_start_monotonic_ns),
                "request_system_ns": int(request_system_ns),
                "request_monotonic_ns": int(request_monotonic_ns),
                "capture_done_system_ns": int(capture_done_system_ns),
                "capture_done_monotonic_ns": int(capture_done_monotonic_ns),
                "system_minus_monotonic_offset_ns": int(clock_offset_ns),
                "sensor_timestamp_ns": sensor_timestamp_ns_int,
                "sensor_timestamp_system_ns": sensor_timestamp_system_ns,
                "exposure_time_us": exposure_us_int,
                "exposure_ms": (float(exposure_us_int) / 1000.0) if exposure_us_int is not None else None,
                "exposure_midpoint_system_ns": exposure_midpoint_system_ns,
                "exposure_midpoint_source": (
                    "sensor_timestamp_minus_half_exposure"
                    if exposure_midpoint_system_ns is not None
                    else None
                ),
                "timestamp_source": (
                    "picamera2_sensor_timestamp"
                    if exposure_midpoint_system_ns is not None
                    else "picamera2_capture_done_system_clock"
                ),
                "metadata": metadata,
            }
        )
        return payload
    except Exception as e:
        payload.update(
            {
                "status": "capture_failed",
                "error": str(e),
                "capture_done_system_ns": int(time.time_ns()),
                "capture_done_monotonic_ns": int(time.monotonic_ns()),
            }
        )
        return payload
    finally:
        if picam2 is not None:
            try:
                picam2.stop()
            except Exception:
                pass
            try:
                picam2.close()
            except Exception:
                pass


def main() -> int:
    parser = argparse.ArgumentParser(description="Capture one still image using Picamera2 metadata")
    parser.add_argument("--camera", type=int, required=True)
    parser.add_argument("--width", type=int, required=True)
    parser.add_argument("--height", type=int, required=True)
    parser.add_argument("--quality", type=int, required=True)
    parser.add_argument("--warmup-ms", type=int, default=700)
    parser.add_argument("--timeout-ms", type=int, default=1500)
    parser.add_argument("--awb", default="")
    parser.add_argument("--awbgains", default="")
    parser.add_argument("--saturation", type=float, default=1.0)
    parser.add_argument("--output", required=True)
    parser.add_argument("--metadata-json", required=True)
    args = parser.parse_args()

    payload = capture(args)
    try:
        _write_metadata(args.metadata_json, payload)
    except Exception as e:
        payload["metadata_write_error"] = str(e)
    print(json.dumps(_jsonable(payload), separators=(",", ":")))

    ok = (
        payload.get("status") == "ok"
        and os.path.exists(args.output)
        and os.path.getsize(args.output) > 0
    )
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
