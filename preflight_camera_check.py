#!/usr/bin/env python3
"""
preflight_camera_check.py — ground test to catch a stuck/frozen camera
capture pipeline BEFORE takeoff (the 2026-11-01 flight only showed this
after landing, because the pipeline silently kept returning a stale frame
without ever reporting an error).

Uses the exact same camera-opening code vo_flight_capture.py uses in real
flight, so a PASS here means the actual flight capture path is healthy.

How it works: takes a photo, asks you to physically wave/rotate the camera
for a few seconds, takes a second photo, then checks TWO independent
signals that both must show real change:
  1. Raw pixel difference between the two frames (catches a frozen buffer
     even if feature matching still runs "successfully" on it).
  2. VO's own displacement estimate between the two frames.
A stuck pipeline fails signal 1 even though it may still report normal-
looking match/inlier counts on signal 2 (exactly what happened this morning
-- 2300+ matches, 0.00m displacement, because it matched the same frozen
image against itself).

Usage:
    python3 preflight_camera_check.py               # one quick check
    python3 preflight_camera_check.py --trials 3     # repeat 3 times
    python3 preflight_camera_check.py --wave-seconds 5
"""
import argparse
import os
import sys
import time

# vo_flight_capture must be imported BEFORE cv2/numpy here: it does its own
# sys.path.insert(0, '/usr/lib/python3/dist-packages') before its `import
# cv2`, to dodge a pip-installed opencv-python (no GStreamer support) that
# shadows the system one. If this script's own `import cv2` ran first,
# Python would cache that wrong module and vo_flight_capture.py's fix would
# never get a chance to take effect -- exactly what caused this script's
# "Could not open CSI camera" failures.
import vo_flight_capture as vfc
import cv2
import numpy as np

PIXEL_DIFF_PASS_THRESHOLD = 8.0  # mean abs diff (0-255 scale, 160x90 downsample) -- well above the
                                  # ~1.5 noise-floor we measured on a genuinely frozen buffer


def _small_gray(gray):
    return cv2.resize(gray, (160, 90))


def run_one_trial(cap, wave_seconds, trial_num):
    print(f"\n--- Trial {trial_num} ---")
    frame1, gray1 = vfc.capture_gray(cap)
    if frame1 is None:
        print("FAIL: could not grab a frame from the camera at all.")
        return False

    print(f"Captured reference frame. Now WAVE / ROTATE the camera for {wave_seconds} seconds...")
    for i in range(wave_seconds, 0, -1):
        print(f"  {i}...", end=' ', flush=True)
        time.sleep(1)
    print()

    frame2, gray2 = vfc.capture_gray(cap)
    if frame2 is None:
        print("FAIL: could not grab a second frame from the camera.")
        return False

    pixel_diff = float(np.mean(np.abs(_small_gray(gray2).astype(int) - _small_gray(gray1).astype(int))))

    r = vfc.process_pair(gray1, gray2, vfc.FALLBACK_ALTITUDE_M, vfc.FALLBACK_ALTITUDE_M)
    vo_desc = (f"matches={r['n_matches']} inliers={r['n_inliers']} "
               f"disp={r.get('magnitude_m', 0):.2f}m" if r['ok'] else
               f"matches={r['n_matches']} inliers={r['n_inliers']} ({r.get('reason', 'no match')})")

    print(f"Pixel diff: {pixel_diff:.1f}  (need >= {PIXEL_DIFF_PASS_THRESHOLD:.1f} to pass)")
    print(f"VO result:  {vo_desc}")

    if pixel_diff >= PIXEL_DIFF_PASS_THRESHOLD:
        print("PASS -- the camera is capturing fresh, live frames.")
        return True
    else:
        print("FAIL -- the two frames are nearly IDENTICAL despite moving the camera.")
        print("        This is the exact signature of a stuck/frozen capture pipeline")
        print("        (e.g. a wedged Argus session). DO NOT FLY. Try:")
        print("          sudo systemctl restart nvargus-daemon")
        print("        and re-run this check; if it still fails, a full reboot is next.")
        return False


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--sensor-id', type=int, default=0)
    ap.add_argument('--trials', type=int, default=1, help='how many times to repeat the test (default 1)')
    ap.add_argument('--wave-seconds', type=int, default=3,
                     help='seconds to wave/move the camera between the two test shots (default 3)')
    args = ap.parse_args()

    print("Opening camera (same pipeline vo_flight_capture.py uses in real flight)...")
    cap = vfc.open_camera(args.sensor_id)
    print("Camera open.\n")

    results = []
    try:
        for t in range(1, args.trials + 1):
            results.append(run_one_trial(cap, args.wave_seconds, t))
    finally:
        cap.release()

    print(f"\n=== Result: {sum(results)}/{len(results)} trial(s) passed ===")
    if all(results):
        print("Camera looks healthy -- safe to proceed with the flight.")
        sys.exit(0)
    else:
        print("At least one trial FAILED -- do not fly until this passes.")
        sys.exit(1)


if __name__ == '__main__':
    main()
