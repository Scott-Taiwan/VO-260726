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
     --takeoff-altitude-m (default 40m), so the climb from wherever the
     anchor was taken (typically on the ground) up to 40m is NOT covered by
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
  4. Every captured photo is saved into photo_obtained/ with a filename
     encoding both the live real GPS (from Pixhawk, read fresh at capture
     time -- validation only, never fed into the VO estimate) and the VO
     dead-reckoned estimate, so accuracy can be assessed after the flight
     exactly like the existing photo_obtained/ test set.
  5. Stops automatically once relative altitude drops back below
     --land-altitude-m (default 2m) AFTER having exceeded the takeoff
     altitude -- i.e. on landing. Ctrl+C also stops it early.

THIS SCRIPT DOES NOT WRITE ANYTHING BACK TO PIXHAWK. It only reads
telemetry (GLOBAL_POSITION_INT, VFR_HUD) and writes photo files -- unlike
live_vo_gps.py / vo_gps_bridge.py, it never calls send_gps_input() /
GPS_INPUT. The real GPS keeps flying the aircraft; this is a data-logging
tool for later accuracy analysis (see recompute_vo_trajectory.py /
fit_correction_scales.py for that offline workflow).

Filename format (saved into photo_obtained/), one photo per line below the
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
        --takeoff-altitude-m 40 --land-altitude-m 2 --out-dir photo_obtained
"""
import argparse
import math
import os
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
FALLBACK_ALTITUDE_M = 40.0


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

    return {
        'dx_m': dx_m,
        'dy_m': dy_m,
        'magnitude_m': math.hypot(dx_m, dy_m),
        'yaw_change_deg': -math.degrees(theta_rad),
        'scale': scale,
        'altitude_change_m': altitude_m * (1.0 / scale - 1.0) if scale > 0 else float('nan'),
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
    pipeline = gstreamer_pipeline(sensor_id=sensor_id, width=IMAGE_WIDTH, height=IMAGE_HEIGHT)
    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        raise RuntimeError("Could not open CSI camera — check sensor connection / nvargus-daemon.")
    for _ in range(10):  # let exposure settle
        cap.read()
    return cap


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
    ap.add_argument('--takeoff-altitude-m', type=float, default=40.0,
                     help='relative (AGL) altitude that must be exceeded before capture starts (default 40)')
    ap.add_argument('--land-altitude-m', type=float, default=2.0,
                     help='relative (AGL) altitude, after takeoff, below which the program stops (default 2)')
    ap.add_argument('--port', default='/dev/ttyTHS1')
    ap.add_argument('--baud', type=int, default=57600)
    ap.add_argument('--sensor-id', type=int, default=0)
    ap.add_argument('--out-dir', default=None,
                     help='where to save photos (default: photo_obtained/ next to this script)')
    args = ap.parse_args()

    out_dir = args.out_dir or os.path.join(os.path.dirname(os.path.abspath(__file__)), 'photo_obtained')
    os.makedirs(out_dir, exist_ok=True)

    conn = connect_pixhawk(args.port, args.baud)

    # Anchor: read real GPS + true heading from Pixhawk exactly once, here,
    # and never again -- see the module docstring's CAVEAT about the
    # untracked climb from here to the takeoff altitude.
    pos = read_global_position(conn, timeout=10)
    if pos is None:
        raise SystemExit("Could not read GLOBAL_POSITION_INT from Pixhawk for the anchor fix.")
    heading = read_heading(conn, timeout=10)
    if heading is None:
        raise SystemExit("Could not read VFR_HUD heading from Pixhawk for the anchor fix.")
    anchor_lat, anchor_lon, anchor_alt_msl, anchor_alt_agl = pos
    tracker = PositionTracker(lat=anchor_lat, lon=anchor_lon, alt_m=anchor_alt_msl, heading_deg=heading)
    print(f"Anchor (read once at startup): {anchor_lat:.7f}, {anchor_lon:.7f}  "
          f"alt_msl={anchor_alt_msl:.1f}m  agl={anchor_alt_agl:.1f}m  heading={heading:.0f}deg\n")

    wait_for_takeoff(conn, args.takeoff_altitude_m)

    cap = open_camera(args.sensor_id)
    print(f"Camera open ({IMAGE_WIDTH}x{IMAGE_HEIGHT}, {CAMERA_HFOV_DEG:.0f}deg HFOV, "
          f"FX_PIXELS=FY_PIXELS={FX_PIXELS:.1f}). Capturing every {args.interval}s.\n")

    prev_gray = prev_time = prev_alt_agl = None
    n_frame = 0
    agg_distance_m = 0.0
    has_landed = False

    try:
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
            cv2.imwrite(os.path.join(out_dir, fname), frame)

            prev_gray, prev_time, prev_alt_agl = gray, now, cur_alt_agl

            if cur_alt_agl < args.land_altitude_m:
                print(f"\nRelative altitude {cur_alt_agl:.1f}m < land threshold "
                      f"{args.land_altitude_m:.1f}m — landed, stopping.")
                has_landed = True
                continue

            elapsed = time.time() - now
            time.sleep(max(0.0, args.interval - elapsed))

    except KeyboardInterrupt:
        print(f"\nStopped early by user after {n_frame} frame(s).")
    finally:
        cap.release()

    print(f"Final: {n_frame} frame(s) captured, {tracker.legs_since_anchor} leg(s) applied, "
          f"aggregated flight distance {agg_distance_m:.1f}m, "
          f"final VO position {tracker.lat:.7f},{tracker.lon:.7f} hacc={tracker.hacc_m:.2f}m")


if __name__ == '__main__':
    main()
