#!/usr/bin/env python3
"""
vo_capture_nopx.py — camera-only version of vo_flight_capture.py.

No Pixhawk, no GPS, no takeoff/landing wait: it opens the CSI camera and
immediately captures one photo every --interval seconds (default 2) until
--shots photos (default 10) are saved, then exits.

Between each consecutive pair it also runs the same full-frame VO as
vo_flight_capture.py (same 77-deg camera constants, same process_pair()),
using a fixed --altitude-m (default 20) since there is no Pixhawk altitude.
The VO numbers are only printed/logged for a quick sanity check.

Output: photo_obtained/<yyyymmdd_hhmmss>_nopx_cam<0|1>/
    NOPX_<yyyymmddhhmmss>_<nn>_<matches>_<inliers>_<disp_m>.png
    vo_log.csv   one row per photo (time, file, matches, inliers, dx, dy, disp, yaw)

Usage:
    python3 vo_capture_nopx.py                     # 10 shots, every 2 s
    python3 vo_capture_nopx.py --shots 20 --interval 1 --altitude-m 30
    python3 vo_capture_nopx.py --cam 0                # use the camera on CAM0 (default CAM1)
    python3 vo_capture_nopx.py --exposure-ms 1 3      # shutter range 1-3 ms (default 1-5, config.py)
    python3 vo_capture_nopx.py --exposure-ms 2 2      # fixed 2 ms shutter (1/500 s)
    python3 vo_capture_nopx.py --tnr-mode 1 --tnr-strength 0   # TNR on, moderate (default OFF)
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
from vo_flight_capture import (  # noqa: E402
    IMAGE_WIDTH, IMAGE_HEIGHT, CAMERA_HFOV_DEG, FX_PIXELS,
    process_pair, self_check_camera,
    add_tnr_arguments, check_tnr_arguments, add_tnr_to_pipeline,
)

# Sensor hardware limits (from config.py's comment): 13,000 - 683,709,000 ns.
SENSOR_EXPOSURE_MIN_MS = 0.013
SENSOR_EXPOSURE_MAX_MS = 683.709


def open_camera(sensor_id, exposure_range_ns, tnr_mode=0, tnr_strength=None):
    """Same as vo_flight_capture.open_camera(), but with the shutter range
    passed in instead of always using config.EXPOSURE_TIME_RANGE_NS."""
    print(f"Camera shutter/exposure: exposuretimerange="
          f"{exposure_range_ns[0] / 1e6:.3f}-{exposure_range_ns[1] / 1e6:.3f}ms  "
          f"gainrange={config.GAIN_RANGE[0]:.1f}-{config.GAIN_RANGE[1]:.2f}x  "
          f"ispdigitalgainrange={config.ISP_DIGITAL_GAIN_RANGE[0]:.1f}-"
          f"{config.ISP_DIGITAL_GAIN_RANGE[1]:.1f}x")
    pipeline = gstreamer_pipeline(sensor_id=sensor_id, width=IMAGE_WIDTH, height=IMAGE_HEIGHT,
                                  exposure_time_range_ns=exposure_range_ns)
    pipeline = add_tnr_to_pipeline(pipeline, sensor_id, tnr_mode, tnr_strength)
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
    ap.add_argument('--altitude-m', type=float, default=20.0,
                    help='fixed altitude used for VO meters conversion (default 20)')
    ap.add_argument('--cam', '--sensor-id', dest='sensor_id', type=int, choices=[0, 1], default=1,
                    help='which CSI camera port to use: 0 = CAM0, 1 = CAM1 (default 1)')
    ap.add_argument('--out-dir', default=None,
                    help='base folder (default: photo_obtained/ next to this script)')
    ap.add_argument('--self-check', action='store_true',
                    help='run the rotate-the-camera frozen-frame self-check before capturing')
    ap.add_argument('--wave-seconds', type=int, default=3)
    ap.add_argument('--no-vo', action='store_true', help='only save photos, skip VO matching')
    def_lo_ms = config.EXPOSURE_TIME_RANGE_NS[0] / 1e6
    def_hi_ms = config.EXPOSURE_TIME_RANGE_NS[1] / 1e6
    ap.add_argument('--exposure-ms', type=float, nargs=2, metavar=('MIN', 'MAX'),
                    default=[def_lo_ms, def_hi_ms],
                    help=f'shutter (exposure time) range in ms; auto-exposure stays inside it. '
                         f'Same value twice = fixed shutter, e.g. 2 2 (default {def_lo_ms:g} {def_hi_ms:g}, '
                         f'from config.py; sensor limit {SENSOR_EXPOSURE_MIN_MS}-{SENSOR_EXPOSURE_MAX_MS})')
    add_tnr_arguments(ap)
    args = ap.parse_args()
    check_tnr_arguments(ap, args)

    exp_lo, exp_hi = args.exposure_ms
    if not (SENSOR_EXPOSURE_MIN_MS <= exp_lo <= exp_hi <= SENSOR_EXPOSURE_MAX_MS):
        ap.error(f"--exposure-ms needs {SENSOR_EXPOSURE_MIN_MS} <= MIN <= MAX <= {SENSOR_EXPOSURE_MAX_MS}")
    exposure_range_ns = (int(round(exp_lo * 1e6)), int(round(exp_hi * 1e6)))

    base_out_dir = args.out_dir or os.path.join(os.path.dirname(os.path.abspath(__file__)), 'photo_obtained')
    out_dir = os.path.join(base_out_dir,
                           datetime.now().strftime('%Y%m%d_%H%M%S') + f'_nopx_cam{args.sensor_id}')
    os.makedirs(out_dir, exist_ok=True)
    print(f"Photos will be saved to: {out_dir}")

    print(f"Opening camera CAM{args.sensor_id} ({IMAGE_WIDTH}x{IMAGE_HEIGHT},{CAMERA_HFOV_DEG:.0f}deg HFOV, "
          f"FX_PIXELS={FX_PIXELS:.1f})...")
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
    total_disp_m = 0.0
    csv_path = os.path.join(out_dir, 'vo_log.csv')
    try:
        if args.self_check:
            self_check_camera(cap, args.wave_seconds)

        print(f"Capturing {args.shots} photo(s), one every {args.interval}s "
              f"(VO altitude {args.altitude_m:.1f}m).\n")
        with open(csv_path, 'w', newline='') as fcsv:
            w = csv.writer(fcsv)
            w.writerow(['shot', 'time', 'file', 'matches', 'inliers', 'status',
                        'dx_m', 'dy_m', 'disp_m', 'yaw_deg', 'dt_s'])
            prev_gray = prev_time = None
            fails_in_row = 0
            while n_saved < args.shots:
                t0 = time.time()
                frame, gray = capture_gray(cap)
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

                r = {'n_matches': 0, 'n_inliers': 0, 'ok': False}
                status, dt_s = 'FIRST', 0.0
                if prev_gray is not None and not args.no_vo:
                    dt_s = t0 - prev_time
                    r = process_pair(prev_gray, gray, args.altitude_m, args.altitude_m)
                    status = 'OK' if r.get('confident') else ('FAIL' if not r['ok'] else 'LOW-CONF')
                    if r['ok']:
                        total_disp_m += r['magnitude_m']
                elif args.no_vo:
                    status = 'NO-VO'
                disp = r.get('magnitude_m', 0.0) if r['ok'] else 0.0

                ts = datetime.now()
                fname = (f"NOPX_{ts.strftime('%Y%m%d%H%M%S')}_{shot:02d}_"
                         f"{r['n_matches']}_{r['n_inliers']}_{disp:.2f}.png")
                if cv2.imwrite(os.path.join(out_dir, fname), frame):
                    n_saved += 1
                else:
                    n_save_failures += 1
                    print(f"WARNING: failed to save {fname}")
                    continue

                msg = f"[{shot:2d}/{args.shots}] [{status}] {fname}"
                if r['ok']:
                    msg += (f"  dx={r['dx_m']:+.2f} dy={r['dy_m']:+.2f} disp={disp:.2f}m "
                            f"yaw={r['yaw_change_deg']:+.2f}deg")
                elif status == 'FAIL':
                    msg += f"  ({r.get('reason', 'no match')})"
                print(msg)
                w.writerow([shot, ts.isoformat(timespec='milliseconds'), fname,
                            r['n_matches'], r['n_inliers'], status,
                            f"{r['dx_m']:.3f}" if r['ok'] else '',
                            f"{r['dy_m']:.3f}" if r['ok'] else '',
                            f"{disp:.3f}", f"{r['yaw_change_deg']:.3f}" if r['ok'] else '',
                            f"{dt_s:.2f}"])
                fcsv.flush()

                prev_gray, prev_time = gray, t0
                if n_saved < args.shots:
                    time.sleep(max(0.0, args.interval - (time.time() - t0)))
    except KeyboardInterrupt:
        print("\nStopped early by user.")
    finally:
        cap.release()

    print(f"\nDone: {n_saved}/{args.shots} photo(s) saved to {out_dir}"
          + ("" if args.no_vo else f", total VO displacement {total_disp_m:.2f}m"))
    if n_save_failures:
        print(f"WARNING: {n_save_failures} photo(s) failed to save -- check disk space.")


if __name__ == '__main__':
    main()
