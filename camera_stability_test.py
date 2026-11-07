#!/usr/bin/env python3
"""
camera_stability_test.py — simple CSI camera stability test.

Takes one photo every --interval seconds (default 3s), saved as
yyyymmdd_nn.jpg (nn starting at 01) into a fresh subfolder created at
startup, so you can flip through the photos afterward and visually confirm
the camera is actually capturing live, changing frames -- not the same
stuck frame over and over (see the 2026-11-01 flight incident).

Uses the exact same camera-opening code vo_flight_capture.py uses in real
flight, so this is representative of what the actual flight capture path
will see. No Pixhawk connection needed -- camera only.

Usage:
    python3 camera_stability_test.py                  # every 3s, Ctrl+C to stop
    python3 camera_stability_test.py --interval 1
    python3 camera_stability_test.py --count 50        # stop after 50 photos
"""
import argparse
import os
import time
from datetime import datetime

import vo_flight_capture as vfc


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--interval', type=float, default=3.0, help='seconds between photos (default 3)')
    ap.add_argument('--sensor-id', type=int, default=0)
    ap.add_argument('--count', type=int, default=None, help='stop after this many photos (default: run until Ctrl+C)')
    ap.add_argument('--out-dir', default=None,
                     help='base folder (default: camera_stability_test/ next to this script) -- '
                          'each run gets its own timestamped subfolder underneath this')
    args = ap.parse_args()

    base_dir = args.out_dir or os.path.join(os.path.dirname(os.path.abspath(__file__)), 'camera_stability_test')
    run_id = datetime.now().strftime('%Y%m%d_%H%M%S')
    out_dir = os.path.join(base_dir, run_id)
    os.makedirs(out_dir, exist_ok=True)
    print(f"Saving photos to: {out_dir}")

    date_str = datetime.now().strftime('%Y%m%d')

    print("Opening camera...")
    cap = vfc.open_camera(args.sensor_id)
    print(f"Camera open. Taking a photo every {args.interval}s. Ctrl+C to stop.\n")

    n = 0
    try:
        while args.count is None or n < args.count:
            frame, _gray = vfc.capture_gray(cap)
            now = time.time()
            if frame is None:
                print("WARNING: frame grab failed, skipping")
                time.sleep(args.interval)
                continue

            n += 1
            fname = f"{date_str}_{n:02d}.jpg"
            save_path = os.path.join(out_dir, fname)
            if not vfc.cv2.imwrite(save_path, frame):
                print(f"[{n:02d}] WARNING: failed to save {fname}")
            else:
                print(f"[{n:02d}] {datetime.now().strftime('%H:%M:%S')}  saved {fname}")

            elapsed = time.time() - now
            time.sleep(max(0.0, args.interval - elapsed))

    except KeyboardInterrupt:
        print(f"\nStopped by user after {n} photo(s).")
    finally:
        cap.release()

    print(f"\nFinal: {n} photo(s) saved to {out_dir}")
    print("Flip through them (e.g. in the IDE file browser) -- if every photo looks the "
          "same despite the camera/surroundings changing, the capture pipeline is stuck.")


if __name__ == '__main__':
    main()
