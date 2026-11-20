#!/usr/bin/env python3
"""
photo_capture_dualcam.py — photo_capture_nopx.py with camera choice: CAM1 (default), CAM0, or both.

No Pixhawk, no GPS, no VO / displacement calculation: it opens the CSI camera
and captures one photo every --interval seconds (default 2) until --shots
photos (default 10) are saved, then exits. Same camera setup as
vo_capture_nopx.py (same resolution, --exposure-ms, --self-check).

--cam picks the camera: 1 (default), 0, or "both". With "both", every shot
grabs one frame from CAM0 and one from CAM1 back-to-back (each pipeline
drops stale frames, so the pair is near-simultaneous).

TNR (temporal noise reduction) is OFF by default (--tnr-mode 0); the camera
driver's own default is ON, which blends frames and smears moving shots.

Output: photo_obtained/<yyyymmdd_hhmmss>_photo_dualcam_cam<1|0|both>/
    PHOTO_<yyyymmddhhmmss>_<nn>_cam<0|1>.png
    photo_log.csv   one row per photo (shot, cam, time, file, dt_s)

Usage:
    python3 photo_capture_dualcam.py                     # 10 shots, every 2 s, CAM1
    python3 photo_capture_dualcam.py --shots 20 --interval 1
    python3 photo_capture_dualcam.py --cam 0             # use the camera on CAM0 (default CAM1)
    python3 photo_capture_dualcam.py --cam both          # CAM0 + CAM1 together
    python3 photo_capture_dualcam.py --exposure-ms 1 3   # shutter range 1-3 ms (default 1-5, config.py)
    python3 photo_capture_dualcam.py --exposure-ms 2 2   # fixed 2 ms shutter (1/500 s)
    python3 photo_capture_dualcam.py --tnr-mode 1 --tnr-strength 0   # turn TNR on (moderate)
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
from live_vo_gps import capture_gray  # noqa: E402
from vo_flight_capture import IMAGE_WIDTH, IMAGE_HEIGHT, self_check_camera  # noqa: E402
from vo_capture_nopx import SENSOR_EXPOSURE_MIN_MS, SENSOR_EXPOSURE_MAX_MS  # noqa: E402
from photo_capture_nopx import open_camera  # noqa: E402  (sets tnr-mode)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--interval', type=float, default=2.0, help='seconds between captures (default 2)')
    ap.add_argument('--shots', type=int, default=10, help='number of photos to take (default 10)')
    ap.add_argument('--cam', '--sensor-id', dest='cam', choices=['0', '1', 'both'], default='1',
                    help='which CSI camera(s) to use: 0 = CAM0, 1 = CAM1, both = CAM0+CAM1 (default 1)')
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

    sensor_ids = [0, 1] if args.cam == 'both' else [int(args.cam)]
    cams_txt = '+'.join(f'CAM{sid}' for sid in sensor_ids)

    base_out_dir = args.out_dir or os.path.join(os.path.dirname(os.path.abspath(__file__)), 'photo_obtained')
    out_dir = os.path.join(base_out_dir,
                           datetime.now().strftime('%Y%m%d_%H%M%S') + f'_photo_dualcam_cam{args.cam}')
    os.makedirs(out_dir, exist_ok=True)
    print(f"Photos will be saved to: {out_dir}")

    caps = {}  # sensor_id -> VideoCapture
    try:
        for sid in sensor_ids:
            print(f"Opening camera CAM{sid} ({IMAGE_WIDTH}x{IMAGE_HEIGHT})...")
            caps[sid] = open_camera(sid, exposure_range_ns, args.tnr_mode, args.tnr_strength)
            # cap.isOpened() is True even when Argus failed to create its
            # CaptureSession, so prove a real frame arrives before going on.
            test_frame, _ = capture_gray(caps[sid])
            if test_frame is None:
                raise SystemExit(
                    f"CAM{sid} opened but delivers NO frames (Argus 'Failed to create CaptureSession' /\n"
                    "'Device is in use', or no camera connected on that port). Usually nvargus-daemon\n"
                    "is still holding a crashed session.\n"
                    "Fix:  sudo systemctl restart nvargus-daemon   then re-run.  Still failing -> reboot.")
            print(f"CAM{sid} open (test frame OK).\n")
    except BaseException:
        for c in caps.values():
            c.release()
        raise

    n_saved = 0
    n_save_failures = 0
    csv_path = os.path.join(out_dir, 'photo_log.csv')
    try:
        if args.self_check:
            for sid, cap in caps.items():
                print(f"Self-check on CAM{sid}:")
                self_check_camera(cap, args.wave_seconds)

        print(f"Capturing {args.shots} shot(s) from {cams_txt}, one every {args.interval}s (no VO).\n")
        with open(csv_path, 'w', newline='') as fcsv:
            w = csv.writer(fcsv)
            w.writerow(['shot', 'cam', 'time', 'file', 'dt_s'])
            prev_time = None
            fails_in_row = 0
            while n_saved < args.shots:
                t0 = time.time()
                # Grab every camera first, back-to-back, then save -- keeps
                # a "both" pair as close in time as possible.
                frames = {sid: capture_gray(cap)[0] for sid, cap in caps.items()}
                ts = datetime.now()
                bad = [sid for sid, f in frames.items() if f is None]
                if bad:
                    fails_in_row += 1
                    if fails_in_row >= 3:
                        print(f"ERROR: 3 frame grabs failed in a row on "
                              f"{', '.join(f'CAM{b}' for b in bad)} -- camera stopped delivering. "
                              "Try: sudo systemctl restart nvargus-daemon")
                        break
                    print(f"WARNING: frame grab failed on {', '.join(f'CAM{b}' for b in bad)}, retrying")
                    time.sleep(0.5)
                    continue
                fails_in_row = 0
                shot = n_saved + 1
                dt_s = 0.0 if prev_time is None else t0 - prev_time

                saved_names = []
                for sid, frame in frames.items():
                    fname = f"PHOTO_{ts.strftime('%Y%m%d%H%M%S')}_{shot:02d}_cam{sid}.png"
                    if cv2.imwrite(os.path.join(out_dir, fname), frame):
                        saved_names.append(fname)
                        w.writerow([shot, sid, ts.isoformat(timespec='milliseconds'), fname, f"{dt_s:.2f}"])
                    else:
                        n_save_failures += 1
                        print(f"WARNING: failed to save {fname}")
                fcsv.flush()
                if not saved_names:
                    continue
                n_saved += 1
                print(f"[{shot:2d}/{args.shots}] {'  '.join(saved_names)}")

                prev_time = t0
                if n_saved < args.shots:
                    time.sleep(max(0.0, args.interval - (time.time() - t0)))
    except KeyboardInterrupt:
        print("\nStopped early by user.")
    finally:
        for c in caps.values():
            c.release()

    print(f"\nDone: {n_saved}/{args.shots} shot(s) from {cams_txt} saved to {out_dir}")
    if n_save_failures:
        print(f"WARNING: {n_save_failures} photo(s) failed to save -- check disk space.")


if __name__ == '__main__':
    main()
