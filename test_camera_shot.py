#!/usr/bin/env python3
"""
test_camera_shot.py — quick ground test for the CSI camera's current
exposure/gain settings (config.py's EXPOSURE_TIME_RANGE_NS / GAIN_RANGE /
ISP_DIGITAL_GAIN_RANGE), without needing to fly.

Opens the camera with the exact same gstreamer_pipeline() the flight
scripts use, prints the exposure/gain settings in effect, gives you a few
seconds to pan/move the camera by hand (simulating in-flight motion), then
saves one photo to camera_test_shots/ -- filename includes the exposure/
gain values used, so shots from different config.py tunings don't get
mixed up.

Usage:
    python3 test_camera_shot.py                   # default 3s pan window
    python3 test_camera_shot.py --wave-seconds 5
    python3 test_camera_shot.py --sensor-id 1
"""
import argparse
import os
import sys
import time
from datetime import datetime

# Same JetPack-vs-pip-opencv workaround as live_vo_gps.py / vo_flight_capture.py.
sys.path.insert(0, '/usr/lib/python3/dist-packages')
import cv2  # noqa: E402

import config  # noqa: E402
from live_vo_gps import gstreamer_pipeline  # noqa: E402

OUT_DIR = 'camera_test_shots'


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--sensor-id', type=int, default=0)
    parser.add_argument('--wave-seconds', type=float, default=3.0,
                         help='seconds to pan/move the camera by hand before the shot is taken (default 3)')
    args = parser.parse_args()

    exp_lo_ms = config.EXPOSURE_TIME_RANGE_NS[0] / 1e6
    exp_hi_ms = config.EXPOSURE_TIME_RANGE_NS[1] / 1e6
    print(f"Camera shutter/exposure: exposuretimerange={exp_lo_ms:.2f}-{exp_hi_ms:.2f}ms  "
          f"gainrange={config.GAIN_RANGE[0]:.1f}-{config.GAIN_RANGE[1]:.2f}x  "
          f"ispdigitalgainrange={config.ISP_DIGITAL_GAIN_RANGE[0]:.1f}-{config.ISP_DIGITAL_GAIN_RANGE[1]:.1f}x")

    pipeline = gstreamer_pipeline(sensor_id=args.sensor_id, width=config.IMAGE_WIDTH, height=config.IMAGE_HEIGHT)
    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        raise SystemExit("Could not open CSI camera -- check sensor connection / nvargus-daemon.")

    try:
        for _ in range(10):  # let exposure settle
            cap.read()

        print(f"Move/pan the camera by hand now for {args.wave_seconds:.0f}s to simulate in-flight motion...")
        deadline = time.time() + args.wave_seconds
        while time.time() < deadline:
            cap.read()  # keep draining frames so the final shot below is fresh, not stale
            remaining = deadline - time.time()
            print(f"  {max(remaining, 0):.0f}s...", end=' ', flush=True)
            time.sleep(min(1.0, max(0.0, remaining)))
        print()

        ok, frame = cap.read()  # taken while you're (ideally) still mid-pan
        if not ok or frame is None:
            raise SystemExit("Frame grab failed.")

        os.makedirs(OUT_DIR, exist_ok=True)
        filename = (f"test_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
                    f"_exp{exp_lo_ms:.1f}-{exp_hi_ms:.1f}ms"
                    f"_gain{config.GAIN_RANGE[0]:.1f}-{config.GAIN_RANGE[1]:.1f}x.png")
        path = os.path.join(OUT_DIR, filename)
        if not cv2.imwrite(path, frame):
            raise SystemExit("Failed to save photo (disk full / write error?).")
        print(f"Saved: {path}")
    finally:
        cap.release()


if __name__ == '__main__':
    main()
