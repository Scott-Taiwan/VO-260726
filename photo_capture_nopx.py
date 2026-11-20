#!/usr/bin/env python3
"""
photo_capture_nopx.py — photo-only version of vo_capture_nopx.py.

No Pixhawk, no GPS, no VO / displacement calculation: it opens the CSI camera
and captures one photo every --interval seconds (default 2) until --shots
photos (default 10) are saved, then exits. Same camera setup as
vo_capture_nopx.py (same resolution, --cam, --exposure-ms, --self-check).
TNR (temporal noise reduction) is OFF by default (--tnr-mode 0); the camera
driver's own default is ON, which blends frames and smears moving shots.

Output: photo_obtained/<yyyymmdd_hhmmss>_photo_nopx_cam<0|1>/
    PHOTO_<yyyymmddhhmmss>_<nn>.png
    photo_log.csv   one row per photo (shot, time, file, dt_s)

Usage:
    python3 photo_capture_nopx.py                     # 10 shots, every 2 s
    python3 photo_capture_nopx.py --shots 20 --interval 1
    python3 photo_capture_nopx.py --cam 0             # use the camera on CAM0 (default CAM1)
    python3 photo_capture_nopx.py --exposure-ms 1 3   # shutter range 1-3 ms (default 1-5, config.py)
    python3 photo_capture_nopx.py --exposure-ms 2 2   # fixed 2 ms shutter (1/500 s)
    python3 photo_capture_nopx.py --tnr-mode 1 --tnr-strength 0   # turn TNR on (moderate)
"""
import argparse
import csv
import os
import sys
import time
from datetime import datetime

# Same JetPack-vs-pip-opencv workaround as the other scripts -- before `import cv2`.
sys.path.insert(0, '/usr/lib/python3/dist-packages')
import cv2  # noqa: E402

import config  # noqa: E402
from live_vo_gps import capture_gray, gstreamer_pipeline  # noqa: E402
from vo_flight_capture import IMAGE_WIDTH, IMAGE_HEIGHT, self_check_camera  # noqa: E402
from vo_capture_nopx import SENSOR_EXPOSURE_MIN_MS, SENSOR_EXPOSURE_MAX_MS  # noqa: E402


def open_camera(sensor_id, exposure_range_ns, tnr_mode=0, tnr_strength=None):
    """Like vo_capture_nopx.open_camera(), plus tnr-mode / tnr-strength on
    nvarguscamerasrc (same way vo_flight_capture_fix2.build_pipeline() does it).
    tnr_mode is ALWAYS written, so the driver's default (1 = ON) never applies."""
    print(f"Camera CAM{sensor_id} shutter/exposure: exposuretimerange="
          f"{exposure_range_ns[0] / 1e6:.3f}-{exposure_range_ns[1] / 1e6:.3f}ms  "
          f"gainrange={config.GAIN_RANGE[0]:.1f}-{config.GAIN_RANGE[1]:.2f}x  "
          f"ispdigitalgainrange={config.ISP_DIGITAL_GAIN_RANGE[0]:.1f}-"
          f"{config.ISP_DIGITAL_GAIN_RANGE[1]:.1f}x")
    print(f"Camera CAM{sensor_id} ISP: tnr-mode={tnr_mode} ({'OFF' if tnr_mode == 0 else 'ON'})  "
          f"tnr-strength={'not set' if tnr_strength is None else tnr_strength}")
    pipeline = gstreamer_pipeline(sensor_id=sensor_id, width=IMAGE_WIDTH, height=IMAGE_HEIGHT,
                                  exposure_time_range_ns=exposure_range_ns)
    extra = [f"tnr-mode={tnr_mode}"]
    if tnr_strength is not None:
        extra.append(f"tnr-strength={tnr_strength}")
    src = f"nvarguscamerasrc sensor-id={sensor_id} "
    if src not in pipeline:
        raise RuntimeError("could not add tnr-mode to nvarguscamerasrc -- pipeline format changed?")
    pipeline = pipeline.replace(src, src + ' '.join(extra) + ' ', 1)
    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        raise RuntimeError("Could not open CSI camera — check sensor connection / nvargus-daemon.")
    for _ in range(10):  # let exposure settle
        cap.read()
    return cap


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--interval', type=float, default=2.0, help='seconds between captures (default 2)')
    ap.add_argument('--shots', type=int, default=10, help='number of photos to take (default 10)')
    ap.add_argument('--cam', '--sensor-id', dest='sensor_id', type=int, choices=[0, 1], default=1,
                    help='which CSI camera port to use: 0 = CAM0, 1 = CAM1 (default 1)')
    ap.add_argument('--out-dir', default=None,
                    help='base folder (default: photo_obtained/ next to this script)')
    ap.add_argument('--self-check', action='store_true',
                    help='run the rotate-the-camera frozen-frame self-check before capturing')
    ap.add_argument('--wave-seconds', type=int, default=3)
    def_lo_ms = config.EXPOSURE_TIME_RANGE_NS[0] / 1e6
    def_hi_ms = config.EXPOSURE_TIME_RANGE_NS[1] / 1e6
    ap.add_argument('--exposure-ms', type=float, nargs=2, metavar=('MIN', 'MAX'),
                    default=[def_lo_ms, def_hi_ms],
                    help=f'shutter (exposure time) range in ms; auto-exposure stays inside it. '
                         f'Same value twice = fixed shutter, e.g. 2 2 (default {def_lo_ms:g} {def_hi_ms:g}, '
                         f'from config.py; sensor limit {SENSOR_EXPOSURE_MIN_MS}-{SENSOR_EXPOSURE_MAX_MS})')
    ap.add_argument('--tnr-mode', type=int, choices=[0, 1, 2], default=0,
                    help='temporal noise reduction: 0=off, 1=fast, 2=high quality (default 0 = OFF; '
                         'the camera driver itself would default to 1 = ON, which smears moving shots)')
    ap.add_argument('--tnr-strength', type=float, default=None,
                    help='TNR strength -1.0..1.0 (only matters if --tnr-mode > 0; default: not set)')
    args = ap.parse_args()
    if args.tnr_strength is not None and not -1.0 <= args.tnr_strength <= 1.0:
        ap.error(f"--tnr-strength must be between -1.0 and 1.0 (got {args.tnr_strength})")

    exp_lo, exp_hi = args.exposure_ms
    if not (SENSOR_EXPOSURE_MIN_MS <= exp_lo <= exp_hi <= SENSOR_EXPOSURE_MAX_MS):
        ap.error(f"--exposure-ms needs {SENSOR_EXPOSURE_MIN_MS} <= MIN <= MAX <= {SENSOR_EXPOSURE_MAX_MS}")
    exposure_range_ns = (int(round(exp_lo * 1e6)), int(round(exp_hi * 1e6)))

    base_out_dir = args.out_dir or os.path.join(os.path.dirname(os.path.abspath(__file__)), 'photo_obtained')
    out_dir = os.path.join(base_out_dir,
                           datetime.now().strftime('%Y%m%d_%H%M%S') + f'_photo_nopx_cam{args.sensor_id}')
    os.makedirs(out_dir, exist_ok=True)
    print(f"Photos will be saved to: {out_dir}")

    print(f"Opening camera CAM{args.sensor_id} ({IMAGE_WIDTH}x{IMAGE_HEIGHT})...")
    cap = open_camera(args.sensor_id, exposure_range_ns, args.tnr_mode, args.tnr_strength)
    # cap.isOpened() is True even when Argus failed to create its
    # CaptureSession, so prove a real frame arrives before going on.
    test_frame, _ = capture_gray(cap)
    if test_frame is None:
        cap.release()
        raise SystemExit(
            "Camera opened but delivers NO frames (Argus 'Failed to create CaptureSession' /\n"
            "'Device 0 is in use'). Usually nvargus-daemon is still holding a crashed session.\n"
            "Fix:  sudo systemctl restart nvargus-daemon   then re-run.  Still failing -> reboot.")
    print("Camera open (test frame OK).\n")

    n_saved = 0
    n_save_failures = 0
    csv_path = os.path.join(out_dir, 'photo_log.csv')
    try:
        if args.self_check:
            self_check_camera(cap, args.wave_seconds)

        print(f"Capturing {args.shots} photo(s), one every {args.interval}s (no VO).\n")
        with open(csv_path, 'w', newline='') as fcsv:
            w = csv.writer(fcsv)
            w.writerow(['shot', 'time', 'file', 'dt_s'])
            prev_time = None
            fails_in_row = 0
            while n_saved < args.shots:
                t0 = time.time()
                frame, _ = capture_gray(cap)
                if frame is None:
                    fails_in_row += 1
                    if fails_in_row >= 3:
                        print("ERROR: 3 frame grabs failed in a row -- camera stopped delivering. "
                              "Try: sudo systemctl restart nvargus-daemon")
                        break
                    print("WARNING: frame grab failed, retrying")
                    time.sleep(0.5)
                    continue
                fails_in_row = 0
                shot = n_saved + 1
                dt_s = 0.0 if prev_time is None else t0 - prev_time

                ts = datetime.now()
                fname = f"PHOTO_{ts.strftime('%Y%m%d%H%M%S')}_{shot:02d}.png"
                if cv2.imwrite(os.path.join(out_dir, fname), frame):
                    n_saved += 1
                else:
                    n_save_failures += 1
                    print(f"WARNING: failed to save {fname}")
                    continue

                print(f"[{shot:2d}/{args.shots}] {fname}")
                w.writerow([shot, ts.isoformat(timespec='milliseconds'), fname, f"{dt_s:.2f}"])
                fcsv.flush()

                prev_time = t0
                if n_saved < args.shots:
                    time.sleep(max(0.0, args.interval - (time.time() - t0)))
    except KeyboardInterrupt:
        print("\nStopped early by user.")
    finally:
        cap.release()

    print(f"\nDone: {n_saved}/{args.shots} photo(s) saved to {out_dir}")
    if n_save_failures:
        print(f"WARNING: {n_save_failures} photo(s) failed to save -- check disk space.")


if __name__ == '__main__':
    main()
