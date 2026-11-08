#!/usr/bin/env python3
"""
vo_flight_capture.py — logging-only VO flight data collector for a 77-deg
(horizontal FOV) downward camera, no central crop.

This is a NEW script derived from visual_odometry.py + live_vo_gps.py, not a
replacement for either — visual_odometry.py / config.py are left untouched
because other scripts in this project (recompute_vo_trajectory.py,
process_dji_sequence.py, the photo_obtained/ test set, dead_reckoning.py's
self-test) still depend on the OLD ~166-deg fisheye camera's crop and
FX_PIXELS/FY_PIXELS. This script defines its own camera constants below
instead of touching config.py.

What this does, end to end:
  1. Connects to Pixhawk and reads real GPS (lat/lon/alt MSL) + true compass
     heading EXACTLY ONCE, as soon as the program starts. That single
     reading seeds dead_reckoning.PositionTracker as a fixed anchor and is
     never re-read or reset for the rest of the run.
     CAVEAT: capture only starts once relative altitude exceeds
     --takeoff-altitude-m (default 20m), so the climb from wherever the
     anchor was taken (typically on the ground) up to 20m is NOT covered by
     any VO leg -- the tracker will simply be sitting at the anchor
     position until the first leg (2nd photo) is computed. This will show
     up as a real, non-zero [diff] on the first saved photo -- that's
     expected, not a bug.
  2. Polls Pixhawk relative (AGL) altitude until it exceeds
     --takeoff-altitude-m, then starts capturing one photo every
     --interval seconds (default 2s) from the CSI camera.
  3. Between each consecutive pair of photos, runs feature matching (SIFT/
     ORB per config.py) + RANSAC similarity fit + metric displacement
     (using THIS script's own 77-deg camera constants, full frame, no
     central crop) and advances the PositionTracker.
  4. Every captured photo is saved into photo_obtained/<run_timestamp>/ --
     a fresh, uniquely-named subfolder created at the start of every run
     (see --out-dir), so separate runs never mix their photos together in
     the same folder -- with a filename encoding both the live real GPS
     (from Pixhawk, read fresh at capture time -- validation only, never
     fed into the VO estimate) and the VO dead-reckoned estimate, so
     accuracy can be assessed after the flight exactly like the existing
     photo_obtained/ test set.
  5. Stops automatically once relative altitude drops back below
     --land-altitude-m (default 2m) AFTER having exceeded the takeoff
     altitude -- i.e. on landing. Ctrl+C also stops it early.

THIS SCRIPT DOES NOT WRITE ANYTHING BACK TO PIXHAWK. It only reads
telemetry (GLOBAL_POSITION_INT, VFR_HUD) and writes photo files -- unlike
live_vo_gps.py / vo_gps_bridge.py, it never calls send_gps_input() /
GPS_INPUT. The real GPS keeps flying the aircraft; this is a data-logging
tool for later accuracy analysis (see recompute_vo_trajectory.py /
fit_correction_scales.py for that offline workflow).

Filename format (saved into photo_obtained/<run_timestamp>/), one photo per line below the
field it corresponds to -- all distances/altitudes in meters, lat/lon to 7
decimal places, heights are MSL (matching the anchor and tracker units):

    VO_yyyymmddhhmmss_<real_lat>_<real_lon>_<real_height>_
        <vo_lat>_<vo_lon>_<vo_height>_<real_vo_diff_m>_
        <vo_move_x>_<vo_move_y>_<vo_move_z>_
        <matches>_<inliers>_<aggregated_fly_distance>.png

  real_lat/lon/height   : live Pixhawk GPS at the moment this photo was
                           taken (validation only -- not fed into VO).
  vo_lat/lon/height     : PositionTracker's dead-reckoned estimate after
                           applying the leg ending at this photo.
  real_vo_diff_m        : horizontal (2D) haversine distance between the
                           two positions above, meters.
  vo_move_x/y/z         : this leg's displacement in world North / East /
                           Up meters (x=north, y=east, z=altitude change,
                           +up). Zero on the first photo (no prior frame)
                           and on any failed leg.
  matches/inliers       : this leg's raw feature-match count and RANSAC
                           inlier count (both 0 on the first photo).
  aggregated_fly_distance: running sum of magnitude_m over every leg that
                           produced a usable displacement (an odometer
                           total, not straight-line distance from start).
                           Starts at 0 at the first captured photo.

Usage:
    python3 vo_flight_capture.py --port /dev/ttyTHS1 --interval 2
    python3 vo_flight_capture.py --port /dev/ttyTHS1 \\
        --takeoff-altitude-m 20 --land-altitude-m 2 --out-dir photo_obtained
"""
import argparse
import math
import os
import shutil
import sys
import time
from datetime import datetime

# Same JetPack-vs-pip-opencv workaround as live_vo_gps.py -- see its header
# for the full explanation. Must happen before `import cv2`.
sys.path.insert(0, '/usr/lib/python3/dist-packages')
import cv2  # noqa: E402
import numpy as np  # noqa: E402

import config  # noqa: E402
import visual_odometry as vo  # noqa: E402
from dead_reckoning import PositionTracker  # noqa: E402
from live_vo_gps import (  # noqa: E402
    gstreamer_pipeline, capture_gray, connect_pixhawk,
    read_global_position, read_heading,
)

# ── New camera: 77-deg HORIZONTAL FOV, no fisheye crop ──────────────────────
# Only the horizontal FOV was specified, so this assumes square pixels
# (fx == fy in pixel units) and derives the focal length from a plain
# pinhole model rather than reusing config.py's footprint-derived
# FX_PIXELS/FY_PIXELS, which were empirically fit to the OLD ~166-deg
# fisheye lens and do not apply here.
IMAGE_WIDTH = 1280
IMAGE_HEIGHT = 720
CAMERA_HFOV_DEG = 77.0
FX_PIXELS = (IMAGE_WIDTH / 2.0) / math.tan(math.radians(CAMERA_HFOV_DEG / 2.0))
FY_PIXELS = FX_PIXELS  # square-pixel assumption -- see note above

# No AGL reading available yet (e.g. first Pixhawk read glitches) -- use the
# takeoff altitude itself as a fallback, since capture never starts below it.
FALLBACK_ALTITUDE_M = 20.0

# "Stuck camera" detection: compares real GPS movement against VO's own
# odometer over a rolling window of frames. Catches a frozen/stale capture
# pipeline (camera silently stops returning fresh frames, e.g. a wedged
# Argus session) without false-alarming on a genuine deliberate hover --
# a real hover makes BOTH real GPS and VO agree at ~0, whereas a stuck
# camera shows real GPS moving while VO stays flat. Found the hard way on
# the 2026-11-01 flight: real GPS drifted >50m while VO reported 0.0m the
# entire time because every captured frame was an identical stale buffer.
STUCK_CHECK_INTERVAL_FRAMES = 10   # check every N frames (~20s at the default 2s interval)
STUCK_REAL_MOVE_THRESHOLD_M = 3.0  # real GPS must have moved at least this much to count as "flying"
STUCK_VO_MOVE_THRESHOLD_M = 0.5    # VO's odometer barely moving despite that = suspicious

# Landing auto-stop requires this many CONSECUTIVE captured frames with AGL
# below --land-altitude-m before declaring "landed" -- a single reading is
# not enough, since GPS/baro noise can make altitude briefly dip below the
# threshold while still genuinely airborne (this is exactly what caused a
# false-positive stop during handheld testing of vo_flight_capture_always.py
# on 2026-11-08, where altitude hovered right around the 2m threshold:
# agl=2.0m, 1.9m, 2.0m, 1.9m ... -- a single-reading check stopped
# mid-test). 3 frames at the default 2s interval is ~6s of sustained
# low altitude.
LAND_DEBOUNCE_FRAMES = 3

# One-time self-check on the SAME camera session used for the whole flight
# (opened once, never closed/reopened -- see self_check_camera() below for
# why that matters). Same pixel-diff threshold calibrated in
# preflight_camera_check.py: a genuinely frozen buffer measures ~1.5 here;
# 8.0 leaves a wide safety margin while still requiring real motion.
SELF_CHECK_PIXEL_DIFF_THRESHOLD = 8.0


def _detect_and_match_full_frame(gray1, gray2):
    """Same as visual_odometry.detect_and_match(), minus the central crop --
    the 77-deg lens isn't expected to have the old lens's edge distortion,
    so the full frame is used directly."""
    detector, norm = vo._make_detector()
    kp1, des1 = detector.detectAndCompute(gray1, None)
    kp2, des2 = detector.detectAndCompute(gray2, None)
    if des1 is None or des2 is None or len(kp1) < 4 or len(kp2) < 4:
        return np.empty((0, 2)), np.empty((0, 2))

    bf = cv2.BFMatcher(norm)
    raw_matches = bf.knnMatch(des1, des2, k=2)
    good = [m for m, n in raw_matches if n is not None and m.distance < config.MATCH_RATIO * n.distance]

    pts1 = np.float32([kp1[m.queryIdx].pt for m in good])
    pts2 = np.float32([kp2[m.trainIdx].pt for m in good])
    return pts1, pts2


def _displacement_from_transform(M, altitude_m):
    """Same math as visual_odometry.displacement_from_transform(), using
    this script's own IMAGE_WIDTH/HEIGHT/FX_PIXELS/FY_PIXELS."""
    a, b, tx = M[0]
    c, d, ty = M[1]
    scale = math.sqrt(a * a + c * c)
    theta_rad = math.atan2(c, a)

    cx, cy = IMAGE_WIDTH / 2.0, IMAGE_HEIGHT / 2.0
    center2 = np.array([a * cx + b * cy + tx, c * cx + d * cy + ty])
    apparent_flow = center2 - np.array([cx, cy])
    cam_shift_px = -apparent_flow

    gsd_x = altitude_m / FX_PIXELS
    gsd_y = altitude_m / FY_PIXELS
    dx_m = cam_shift_px[0] * gsd_x
    dy_m = cam_shift_px[1] * gsd_y

    scale_lo, scale_hi = config.SCALE_SANE_RANGE
    altitude_change_m = (
        altitude_m * (1.0 / scale - 1.0) if scale_lo <= scale <= scale_hi else 0.0
    )

    return {
        'dx_m': dx_m,
        'dy_m': dy_m,
        'magnitude_m': math.hypot(dx_m, dy_m),
        'yaw_change_deg': -math.degrees(theta_rad),
        'scale': scale,
        'altitude_change_m': altitude_change_m,
    }


def process_pair(gray1, gray2, alt1, alt2):
    """Full-frame, 77-deg-camera equivalent of
    visual_odometry.process_pair_arrays()."""
    pts1, pts2 = _detect_and_match_full_frame(gray1, gray2)
    M, inliers = vo.estimate_similarity(pts1, pts2)

    n_matches = len(pts1)
    n_inliers = int(inliers.sum()) if inliers is not None else 0
    result = {'n_matches': n_matches, 'n_inliers': n_inliers, 'ok': False, 'confident': False}
    if M is None or inliers is None or n_inliers < config.MIN_INLIERS:
        result['reason'] = 'too few inliers'
        return result

    altitude_m = (alt1 + alt2) / 2.0 if (alt1 and alt2) else (alt1 or alt2 or FALLBACK_ALTITUDE_M)
    result.update(_displacement_from_transform(M, altitude_m))
    result['ok'] = True
    result['altitude_used_m'] = altitude_m
    result['confident'] = (n_inliers / n_matches) >= config.MIN_INLIER_RATIO
    if not result['confident']:
        result['reason'] = 'low inlier ratio -- displacement UNTRUSTED'
    return result


# ── Camera (new camera's own resolution, explicit -- not config.py's) ──────

def open_camera(sensor_id):
    exp_lo_ms = config.EXPOSURE_TIME_RANGE_NS[0] / 1e6
    exp_hi_ms = config.EXPOSURE_TIME_RANGE_NS[1] / 1e6
    print(f"Camera shutter/exposure: exposuretimerange={exp_lo_ms:.2f}-{exp_hi_ms:.2f}ms  "
          f"gainrange={config.GAIN_RANGE[0]:.1f}-{config.GAIN_RANGE[1]:.2f}x  "
          f"ispdigitalgainrange={config.ISP_DIGITAL_GAIN_RANGE[0]:.1f}-{config.ISP_DIGITAL_GAIN_RANGE[1]:.1f}x")
    pipeline = gstreamer_pipeline(sensor_id=sensor_id, width=IMAGE_WIDTH, height=IMAGE_HEIGHT)
    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        raise RuntimeError("Could not open CSI camera — check sensor connection / nvargus-daemon.")
    for _ in range(10):  # let exposure settle
        cap.read()
    return cap


def self_check_camera(cap, wave_seconds):
    """Verify the ACTUAL camera session that will fly the whole mission is
    really capturing fresh frames -- on the SAME `cap` object, with no
    close()/reopen() in between. This matters: on 2026-11-07 a separate
    preflight_camera_check.py run (its own session, closed afterward)
    PASSED minutes before takeoff, yet the real flight's own freshly-opened
    session was already frozen on a stale buffer from its very first frame.
    Testing a different session than the one that actually flies does not
    reliably catch this -- only self-checking the real session does."""
    print("Self-check: capturing a reference frame...")
    frame1, gray1 = capture_gray(cap)
    if frame1 is None:
        raise SystemExit("Camera self-check FAILED: could not grab a frame from the camera at all.")

    print(f"Now ROTATE / MOVE the camera for {wave_seconds} seconds...")
    for i in range(wave_seconds, 0, -1):
        print(f"  {i}...", end=' ', flush=True)
        time.sleep(1)
    print()

    frame2, gray2 = capture_gray(cap)
    if frame2 is None:
        raise SystemExit("Camera self-check FAILED: could not grab a second frame from the camera.")

    small1 = cv2.resize(gray1, (160, 90)).astype(int)
    small2 = cv2.resize(gray2, (160, 90)).astype(int)
    diff = float(np.mean(np.abs(small2 - small1)))
    print(f"Pixel diff: {diff:.1f}  (need >= {SELF_CHECK_PIXEL_DIFF_THRESHOLD:.1f} to pass)")

    if diff < SELF_CHECK_PIXEL_DIFF_THRESHOLD:
        raise SystemExit(
            f"Camera self-check FAILED -- the two frames are nearly IDENTICAL despite "
            f"moving the camera (diff={diff:.1f}). This is the exact signature of a "
            f"frozen/stale capture pipeline (wedged Argus session) seen on the "
            f"2026-11-01 and 2026-11-07 flights. DO NOT FLY.\n"
            f"Try: sudo systemctl restart nvargus-daemon, then re-run this script; "
            f"if it still fails, a full reboot is next."
        )
    print("Camera self-check PASSED -- capturing live, changing frames. Proceeding.\n")


# ── Main flight loop ─────────────────────────────────────────────────────

def wait_for_takeoff(conn, takeoff_altitude_m, poll_s=0.5):
    print(f"Waiting for relative altitude > {takeoff_altitude_m:.1f}m ...")
    last_print = 0.0
    while True:
        pos = read_global_position(conn, timeout=2.0)
        if pos is not None:
            alt_agl = pos[3]
            now = time.time()
            if now - last_print > 5.0:
                print(f"  current relative altitude: {alt_agl:.1f}m")
                last_print = now
            if alt_agl > takeoff_altitude_m:
                print(f"Takeoff altitude reached ({alt_agl:.1f}m). Starting capture.\n")
                return
        time.sleep(poll_s)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--interval', type=float, default=2.0, help='seconds between captures (default 2)')
    ap.add_argument('--takeoff-altitude-m', type=float, default=20.0,
                     help='relative (AGL) altitude that must be exceeded before capture starts (default 20)')
    ap.add_argument('--land-altitude-m', type=float, default=2.0,
                     help='relative (AGL) altitude, after takeoff, below which the program stops (default 2)')
    ap.add_argument('--port', default='/dev/ttyTHS1')
    ap.add_argument('--baud', type=int, default=57600)
    ap.add_argument('--sensor-id', type=int, default=0)
    ap.add_argument('--out-dir', default=None,
                     help='base folder for photos (default: photo_obtained/ next to this script) -- '
                          'each run gets its own timestamped subfolder underneath this')
    ap.add_argument('--wave-seconds', type=int, default=3,
                     help='seconds to rotate/move the camera for during the startup self-check (default 3)')
    args = ap.parse_args()

    base_out_dir = args.out_dir or os.path.join(os.path.dirname(os.path.abspath(__file__)), 'photo_obtained')
    run_id = datetime.now().strftime('%Y%m%d_%H%M%S')
    out_dir = os.path.join(base_out_dir, run_id)
    os.makedirs(out_dir, exist_ok=True)
    print(f"This run's photos will be saved to: {out_dir}\n")

    conn = connect_pixhawk(args.port, args.baud)

    # Anchor: read real GPS + true heading from Pixhawk exactly once, here,
    # and never again -- see the module docstring's CAVEAT about the
    # untracked climb from here to the takeoff altitude.
    pos = read_global_position(conn, timeout=10)
    if pos is None:
        raise SystemExit(
            "Connected to Pixhawk (heartbeat OK), but no GLOBAL_POSITION_INT arrived "
            "within 10s. ArduPilot only publishes this once it has a valid position "
            "estimate, so this almost always means: no GPS fix yet (no sky view, or "
            "just powered on and still acquiring satellites), or EKF origin not set. "
            "Check GPS status in Mission Planner/QGroundControl, then retry."
        )
    heading = read_heading(conn, timeout=10)
    if heading is None:
        raise SystemExit(
            "Connected to Pixhawk and got a GPS position, but no VFR_HUD heading "
            "arrived within 10s. GPS fix is fine -- this points at the telemetry "
            "data stream rate/config instead, not GPS. Check the MAVLink link/radio "
            "and telemetry stream rate (request_data_stream), then retry."
        )
    anchor_lat, anchor_lon, anchor_alt_msl, anchor_alt_agl = pos
    tracker = PositionTracker(lat=anchor_lat, lon=anchor_lon, alt_m=anchor_alt_msl, heading_deg=heading)
    print(f"Anchor (read once at startup): {anchor_lat:.7f}, {anchor_lon:.7f}  "
          f"alt_msl={anchor_alt_msl:.1f}m  agl={anchor_alt_agl:.1f}m  heading={heading:.0f}deg\n")

    # Camera opens HERE, before wait_for_takeoff -- and self-checks itself --
    # so there is exactly one camera session for the entire run, opened once
    # and never closed/reopened. See self_check_camera()'s docstring for why
    # that matters (a separate preflight check, its own session, does not
    # reliably predict whether THIS session will be healthy).
    print(f"Opening camera ({IMAGE_WIDTH}x{IMAGE_HEIGHT}, {CAMERA_HFOV_DEG:.0f}deg HFOV, "
          f"FX_PIXELS=FY_PIXELS={FX_PIXELS:.1f})...")
    cap = open_camera(args.sensor_id)
    print("Camera open.\n")

    # EVERYTHING from here on is inside one try/finally so cap.release() is
    # guaranteed no matter what happens -- including a self-check FAILURE
    # (self_check_camera raises SystemExit) or a Ctrl+C during the self-check
    # itself. An earlier version only wrapped the main loop below, so every
    # self-check failure today leaked the camera session without releasing
    # it -- a real contributor to the repeated CMA memory-exhaustion crashes,
    # not just an unrelated OS limitation.
    try:
        self_check_camera(cap, args.wave_seconds)

        wait_for_takeoff(conn, args.takeoff_altitude_m)
        print(f"Capturing every {args.interval}s.\n")

        prev_gray = prev_time = prev_alt_agl = None
        n_frame = 0
        n_save_failures = 0
        agg_distance_m = 0.0
        has_landed = False
        land_debounce_count = 0

        # Checkpoint for stuck-camera detection -- reset every STUCK_CHECK_INTERVAL_FRAMES.
        stuck_check_frame = 0
        stuck_check_real_latlon = None
        stuck_check_agg_distance_m = 0.0

        while not has_landed:
            frame, gray = capture_gray(cap)
            now = time.time()
            if frame is None:
                print("WARNING: frame grab failed, skipping")
                time.sleep(args.interval)
                continue
            n_frame += 1

            real_pos = read_global_position(conn, timeout=0.5)
            if real_pos is not None:
                real_lat, real_lon, real_alt_msl, cur_alt_agl = real_pos
            else:
                real_lat, real_lon, real_alt_msl = anchor_lat, anchor_lon, anchor_alt_msl
                cur_alt_agl = prev_alt_agl if prev_alt_agl is not None else FALLBACK_ALTITUDE_M

            if prev_gray is None:
                # First captured frame: no prior frame to match against, so
                # the VO estimate is still exactly the anchor. See the
                # "Filename format" note in the module docstring.
                r = {'n_matches': 0, 'n_inliers': 0, 'ok': False}
                move_x = move_y = move_z = 0.0
            else:
                dt_s = now - prev_time
                r = process_pair(prev_gray, gray, prev_alt_agl, cur_alt_agl)
                state = tracker.apply_leg(r, dt_s=dt_s)
                move_x = state['vn'] * dt_s  # north, meters, this leg
                move_y = state['ve'] * dt_s  # east, meters, this leg
                move_z = r.get('altitude_change_m', 0.0) if r['ok'] else 0.0
                if r['ok']:
                    agg_distance_m += r['magnitude_m']

                tag = "OK  " if r.get('confident') else ("FAIL" if not r['ok'] else "LOW-CONF")
                print(f"[frame {n_frame}] [{tag}] dt={dt_s:.2f}s "
                      f"matches={r['n_matches']} inliers={r['n_inliers']}", end='  ')
                if r['ok']:
                    print(f"disp={r['magnitude_m']:.2f}m yaw={r['yaw_change_deg']:+.2f}deg", end='  ')
                else:
                    print(f"({r.get('reason', 'no match')})", end='  ')
                print(f"-> vo={tracker.lat:.7f},{tracker.lon:.7f} agg_dist={agg_distance_m:.1f}m")

            diff_m = vo.haversine_m(real_lat, real_lon, tracker.lat, tracker.lon)

            ts = datetime.now().strftime('%Y%m%d%H%M%S')
            fname = (
                f"VO_{ts}_{real_lat:.7f}_{real_lon:.7f}_{real_alt_msl:.2f}_"
                f"{tracker.lat:.7f}_{tracker.lon:.7f}_{tracker.alt_m:.2f}_{diff_m:.2f}_"
                f"{move_x:.2f}_{move_y:.2f}_{move_z:.2f}_"
                f"{r['n_matches']}_{r['n_inliers']}_{agg_distance_m:.2f}.png"
            )
            save_path = os.path.join(out_dir, fname)
            if not cv2.imwrite(save_path, frame):
                n_save_failures += 1
                free_gb = shutil.disk_usage(out_dir).free / (1024 ** 3)
                print(f"WARNING: failed to save photo (disk full / write error?) -- "
                      f"{save_path}  [{free_gb:.1f}GB free on that filesystem, "
                      f"{n_save_failures} failure(s) so far this run]")

            prev_gray, prev_time, prev_alt_agl = gray, now, cur_alt_agl

            # Stuck-camera check: every STUCK_CHECK_INTERVAL_FRAMES, compare how
            # far real GPS moved against how far VO's odometer advanced over the
            # same window. Real motion + flat VO = the capture pipeline is very
            # likely frozen on a stale frame (see the constants' comment above).
            if stuck_check_real_latlon is None:
                stuck_check_real_latlon = (real_lat, real_lon)
                stuck_check_agg_distance_m = agg_distance_m
                stuck_check_frame = n_frame
            elif n_frame - stuck_check_frame >= STUCK_CHECK_INTERVAL_FRAMES:
                real_moved_m = vo.haversine_m(
                    stuck_check_real_latlon[0], stuck_check_real_latlon[1], real_lat, real_lon)
                vo_moved_m = agg_distance_m - stuck_check_agg_distance_m
                if real_moved_m >= STUCK_REAL_MOVE_THRESHOLD_M and vo_moved_m <= STUCK_VO_MOVE_THRESHOLD_M:
                    print(f"WARNING: camera may be STUCK -- real GPS moved {real_moved_m:.1f}m over the "
                          f"last {n_frame - stuck_check_frame} frames, but VO's odometer only advanced "
                          f"{vo_moved_m:.2f}m. Likely a frozen/stale capture pipeline (e.g. wedged Argus "
                          f"session), not a real hover. Check the live camera feed if possible.")
                stuck_check_real_latlon = (real_lat, real_lon)
                stuck_check_agg_distance_m = agg_distance_m
                stuck_check_frame = n_frame

            if cur_alt_agl < args.land_altitude_m:
                land_debounce_count += 1
                if land_debounce_count < LAND_DEBOUNCE_FRAMES:
                    print(f"Relative altitude {cur_alt_agl:.1f}m < land threshold "
                          f"{args.land_altitude_m:.1f}m ({land_debounce_count}/{LAND_DEBOUNCE_FRAMES} "
                          f"consecutive reading(s) -- waiting for confirmation before stopping).")
                else:
                    print(f"\nRelative altitude {cur_alt_agl:.1f}m < land threshold "
                          f"{args.land_altitude_m:.1f}m for {LAND_DEBOUNCE_FRAMES} consecutive readings "
                          f"— landed, stopping.")
                    has_landed = True
                    continue
            else:
                land_debounce_count = 0

            elapsed = time.time() - now
            time.sleep(max(0.0, args.interval - elapsed))

    except KeyboardInterrupt:
        print(f"\nStopped early by user after {n_frame} frame(s).")
    finally:
        cap.release()

    print(f"Final: {n_frame} frame(s) captured, {tracker.legs_since_anchor} leg(s) applied, "
          f"aggregated flight distance {agg_distance_m:.1f}m, "
          f"final VO position {tracker.lat:.7f},{tracker.lon:.7f} hacc={tracker.hacc_m:.2f}m")
    if n_save_failures:
        print(f"WARNING: {n_save_failures}/{n_frame} photo(s) FAILED TO SAVE this run "
              f"(see WARNING lines above) -- check disk space before the next flight.")


if __name__ == '__main__':
    main()
