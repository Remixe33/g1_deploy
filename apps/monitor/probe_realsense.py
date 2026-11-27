import argparse
import json
import math
import time

from camera_sources import RealSenseSource
from depth_stream import DEFAULT_CAMERA_CONFIG


def main():
    parser = argparse.ArgumentParser(description="Measure Monitor's RealSense input without preview or recording.")
    parser.add_argument("--camera-config", default=DEFAULT_CAMERA_CONFIG,
                        help="Camera module JSON (default: D435 640x480 at 30 Hz)")
    parser.add_argument("--seconds", type=float, default=30)
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or args.seconds <= 0:
        parser.error("--seconds must be positive and finite")
    camera = RealSenseSource(args.camera_config)
    try:
        camera.open()
        started_at = time.monotonic()
        deadline = started_at + args.seconds
        while time.monotonic() < deadline:
            camera.capture(timeout_ms=max(1, min(1000, int((deadline - time.monotonic()) * 1000))))
    finally:
        camera.close()
    print(json.dumps({"window_s": args.seconds, **camera.info()}, indent=2))
    return 0 if camera.sequence else 1


if __name__ == "__main__":
    raise SystemExit(main())
