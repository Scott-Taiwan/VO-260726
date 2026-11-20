#!/usr/bin/env python3
"""
vo_flight_capture_fix.py — fixed version of vo_flight_capture.py (logging-only
VO flight data collector, 77-deg HFOV downward camera, full frame, no crop).

vo_flight_capture.py is left untouched; run_vo_flight_capture.sh still
launches the OLD script. Run this one directly (see Usage below).

Changes vs vo_flight_capture.py:
  1. RE-ANCHOR AT TAKEOFF. The startup GPS/heading read is still done (to fail
     fast if there is no fix), but the PositionTracker is re-anchored with a
     fresh position + heading the moment --takeoff-altitude-m is reached. The
     old version anchored on the ground, so any horizontal drift AND any yaw
     during the climb were never seen by VO -- a yaw during the climb rotated
     the whole VO track, and vo_height started ~takeoff-altitude below real.
  2. FRESH TELEMETRY. A background thread continuously drains the MAVLink
     link and keeps only the LATEST GLOBAL_POSITION_INT / VFR_HUD. The old
     version used recv_match(blocking=True), which returns the OLDEST unread
     message, so "real GPS", the GSD altitude and landing detection could lag,
     and the serial buffer could overflow between reads.
  3. Ctrl+C / `stop` during the self-check or while waiting for takeoff no
     longer crashes with UnboundLocalError (counters are initialised before
     the try block).
  4. A missing/stale telemetry reading no longer writes the ANCHOR position
     as "real GPS". The last known real position is used and the CSV marks
     it stale (real_age_s); stale readings are excluded from the stuck-camera
     check and do not count toward landing detection.
  5. Camera appsink gets an explicit max-buffers=1 (drop=True alone only
     works with a buffer limit), so each read returns the newest frame.
  6. Camera warm-up reads now happen inside the try/finally, so a Ctrl+C
     during warm-up still releases the camera.
  7. Every frame is also written to vo_log.csv in the run folder, including
     the REAL compass heading (VFR_HUD), the VO heading, the altitude used
     for GSD, leg status (OK / LOW-CONF / FAIL), yaw change and hacc.
  8. Default --takeoff-altitude-m is now 30 m (was 20 m).

THIS SCRIPT DOES NOT WRITE ANYTHING BACK TO PIXHAWK (apart from the same
request_data_stream call as before). It only reads telemetry and writes
photo files + a CSV.

Filename format (unchanged, saved into photo_obtained/<run_timestamp>/):

    VO_yyyymmddhhmmss_<real_lat>_<real_lon>_<real_height>_
        <vo_lat>_<vo_lon>_<vo_height>_<real_vo_diff_m>_
        <vo_move_x>_<vo_move_y>_<vo_move_z>_
        <matches>_<inliers>_<aggregated_fly_distance>.png

  See vo_flight_capture.py's docstring for the meaning of each field.
  vo_move_x = north, vo_move_y = east, vo_move_z = up (meters, this leg).

Usage:
    python3 vo_flight_capture_fix.py --port /dev/ttyTHS1 --baud 57600
    python3 vo_flight_capture_fix.py --takeoff-altitude-m 30 --land-altitude-m 2 --interval 2

    # Camera exposure overrides (defaults come from config.py). Each is a
    # LO HI auto-exposure range; set LO == HI to fix the value:
    python3 vo_flight_capture_fix.py --interval 1.5 --exposure-ms 1 3 --gain 1 10.625 \\
        --isp-gain 1 4 --ev -0.5
    # Fixed 1/500s shutter:
    python3 vo_flight_capture_fix.py --exposure-ms 2 2

  Every run writes its exact settings to run_args.txt in the run folder.
"""
import argparse
import csv
import math
import os
import shutil
import sys
import threading
import time
from datetime import datetime

# Same JetPack-vs-pip-opencv workaround as live_vo_gps.py -- must happen
# before `import cv2`.
sys.path.insert(0, '/usr/lib/python3/dist-packages')
import cv2  # noqa: E402
import numpy as np  # noqa: E402

import config  # noqa: E402
import visual_odometry as vo  # noqa: E402
from dead_reckoning import PositionTracker  # noqa: E402
from live_vo_gps import gstreamer_pipeline, capture_gray, connect_pixhawk  # noqa: E402

# ── Camera: 77-deg HORIZONTAL FOV, no fisheye crop ──────────────────────────
# NOTE: if the lens's 77 deg is actually the DIAGONAL FOV (common for IMX219
# modules), HFOV at 16:9 is ~69 deg and every distance below comes out ~15%
# too large. Calibrate, or fit a scale against GPS with fit_correction_scales.py.
IMAGE_WIDTH = 1280
IMAGE_HEIGHT = 720
CAMERA_HFOV_DEG = 77.0
FX_PIXELS = (IMAGE_WIDTH / 2.0) / math.tan(math.radians(CAMERA_HFOV_DEG / 2.0))
FY_PIXELS = FX_PIXELS  # square-pixel assumption

DEFAULT_TAKEOFF_ALTITUDE_M = 30.0

# A telemetry value older than this is considered stale.
TELEMETRY_STALE_S = 2.0

# Stuck-camera detection (same thresholds as vo_flight_capture.py).
STUCK_CHECK_INTERVAL_FRAMES = 10
STUCK_REAL_MOVE_THRESHOLD_M = 3.0
STUCK_VO_MOVE_THRESHOLD_M = 0.5

# Consecutive FRESH low-altitude readings required before declaring "landed".
LAND_DEBOUNCE_FRAMES = 3

# Startup camera self-check threshold (frozen buffer measures ~1.5).
SELF_CHECK_PIXEL_DIFF_THRESHOLD = 8.0

CSV_FIELDS = [
    'frame', 'timestamp', 'filename', 'status',
    'real_lat', 'real_lon', 'real_alt_msl', 'real_alt_agl', 'real_age_s', 'real_heading_deg', 'heading_age_s',
    'vo_lat', 'vo_lon', 'vo_alt_msl', 'vo_heading_deg', 'real_vo_diff_m',
    'move_north_m', 'move_east_m', 'move_up_m',
    'matches', 'inliers', 'magnitude_m', 'yaw_change_deg', 'scale', 'altitude_used_m',
    'agg_distance_m', 'hacc_m', 'dt_s',
]


# ── Telemetry: background reader keeping only the latest values ────────────

class Telemetry:
    """Continuously drains the MAVLink link in a background thread and keeps
    only the LATEST GLOBAL_POSITION_INT and VFR_HUD. After start(), no other
    code may call recv_match on the same connection."""

    def __init__(self, conn):
        self._conn = conn
        self._lock = threading.Lock()
        self._pos = None
        self._pos_t = None
        self._heading = None
        self._heading_t = None
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name='mavlink-reader', daemon=True)
        self._thread.start()

    def _run(self):
        while not self._stop.is_set():
            try:
                msg = self._conn.recv_match(type=['GLOBAL_POSITION_INT', 'VFR_HUD'],
                                            blocking=True, timeout=0.5)
            except Exception as e:  # keep reading -- a transient serial error must not kill logging
                print(f"WARNING: MAVLink read error: {e}")
                time.sleep(0.1)
                continue
            if msg is None:
                continue
            now = time.time()
            mtype = msg.get_type()
            if mtype == 'GLOBAL_POSITION_INT':
                if msg.lat == 0 and msg.lon == 0:
                    continue
                pos = (msg.lat / 1e7, msg.lon / 1e7, msg.alt / 1000.0, msg.relative_alt / 1000.0)
                with self._lock:
                    self._pos, self._pos_t = pos, now
            elif mtype == 'VFR_HUD':
                with self._lock:
                    self._heading, self._heading_t = float(msg.heading), now

    def position(self):
        """((lat, lon, alt_msl_m, relative_alt_m), age_s) of the latest fix, or (None, None)."""
        with self._lock:
            if self._pos is None:
                return None, None
            return self._pos, time.time() - self._pos_t

    def heading(self):
        """(heading_deg, age_s) of the latest VFR_HUD, or (None, None)."""
        with self._lock:
            if self._heading is None:
                return None, None
            return self._heading, time.time() - self._heading_t

    def wait_fresh(self, getter, timeout):
        """Block until getter() returns a value no older than TELEMETRY_STALE_S."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            value, age = getter()
            if value is not None and age <= TELEMETRY_STALE_S:
                return value
            time.sleep(0.1)
        return None

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=2.0)


# ── VO (same math as vo_flight_capture.py) ─────────────────────────────────

def _detect_and_match_full_frame(gray1, gray2):
    detector, norm = vo._make_detector()
    kp1, des1 = detector.detectAndCompute(gray1, None)
    kp2, des2 = detector.detectAndCompute(gray2, None)
    if des1 is None or des2 is None or len(kp1) < 4 or len(kp2) < 4:
        return np.empty((0, 2)), np.empty((0, 2))

    bf = cv2.BFMatcher(norm)
    raw_matches = bf.knnMatch(des1, des2, k=2)
    good = [p[0] for p in raw_matches
            if len(p) == 2 and p[0].distance < config.MATCH_RATIO * p[1].distance]

    pts1 = np.float32([kp1[m.queryIdx].pt for m in good])
    pts2 = np.float32([kp2[m.trainIdx].pt for m in good])
    return pts1, pts2


def _displacement_from_transform(M, altitude_m):
    a, b, tx = M[0]
    c, d, ty = M[1]
    scale = math.sqrt(a * a + c * c)
    theta_rad = math.atan2(c, a)

    cx, cy = IMAGE_WIDTH / 2.0, IMAGE_HEIGHT / 2.0
    center2 = np.array([a * cx + b * cy + tx, c * cx + d * cy + ty])
    cam_shift_px = -(center2 - np.array([cx, cy]))

    dx_m = cam_shift_px[0] * altitude_m / FX_PIXELS
    dy_m = cam_shift_px[1] * altitude_m / FY_PIXELS

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


def process_pair(gray1, gray2, alt1, alt2, fallback_altitude_m):
    pts1, pts2 = _detect_and_match_full_frame(gray1, gray2)
    M, inliers = vo.estimate_similarity(pts1, pts2)

    n_matches = len(pts1)
    n_inliers = int(inliers.sum()) if inliers is not None else 0
    result = {'n_matches': n_matches, 'n_inliers': n_inliers, 'ok': False, 'confident': False}
    if M is None or inliers is None or n_inliers < config.MIN_INLIERS:
        result['reason'] = 'too few inliers'
        return result

    altitude_m = (alt1 + alt2) / 2.0 if (alt1 and alt2) else (alt1 or alt2 or fallback_altitude_m)
    result.update(_displacement_from_transform(M, altitude_m))
    result['ok'] = True
    result['altitude_used_m'] = altitude_m
    result['confident'] = (n_inliers / n_matches) >= config.MIN_INLIER_RATIO
    if not result['confident']:
        result['reason'] = 'low inlier ratio -- displacement UNTRUSTED'
    return result


# ── Camera ─────────────────────────────────────────────────────────────────

def build_pipeline(sensor_id, exposure_ns, gain, isp_gain, ev=None, tnr_mode=0, tnr_strength=None):
    pipeline = gstreamer_pipeline(
        sensor_id=sensor_id, width=IMAGE_WIDTH, height=IMAGE_HEIGHT,
        exposure_time_range_ns=exposure_ns, gain_range=gain, isp_digital_gain_range=isp_gain,
    )
    if ev is not None:
        src = f"nvarguscamerasrc sensor-id={sensor_id} "
        if src not in pipeline:
            raise RuntimeError("could not add exposurecompensation -- pipeline format changed?")
        pipeline = pipeline.replace(src, f"{src}exposurecompensation={ev} ", 1)
    # TNR: tnr-mode is ALWAYS written (default 0 = OFF), so the driver's own
    # default (1 = ON, blends frames -> smear while moving) never applies.
    tnr = [f"tnr-mode={tnr_mode}"] + ([f"tnr-strength={tnr_strength}"] if tnr_strength is not None else [])
    src = f"nvarguscamerasrc sensor-id={sensor_id} "
    if src not in pipeline:
        raise RuntimeError("could not add tnr-mode -- pipeline format changed?")
    pipeline = pipeline.replace(src, src + ' '.join(tnr) + ' ', 1)
    if 'max-buffers' not in pipeline:
        fixed = pipeline.replace('appsink drop=True', 'appsink drop=True max-buffers=1')
        if fixed == pipeline:
            print("WARNING: could not add max-buffers=1 to the appsink -- pipeline format changed?")
        pipeline = fixed
    return pipeline


def self_check_camera(cap, wave_seconds):
    """Verify the camera session that will fly the mission really delivers
    fresh frames (same session, never closed/reopened)."""
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
            f"moving the camera (diff={diff:.1f}). Frozen/stale capture pipeline. DO NOT FLY.\n"
            f"Try: sudo systemctl restart nvargus-daemon, then re-run; if it still fails, reboot."
        )
    print("Camera self-check PASSED -- capturing live, changing frames. Proceeding.\n")


# ── Flight ─────────────────────────────────────────────────────────────────

def wait_for_takeoff(telem, takeoff_altitude_m):
    """Returns the first FRESH position above takeoff_altitude_m."""
    print(f"Waiting for relative altitude > {takeoff_altitude_m:.1f}m ...")
    last_print = 0.0
    while True:
        pos, age = telem.position()
        now = time.time()
        fresh = pos is not None and age <= TELEMETRY_STALE_S
        if now - last_print > 5.0:
            if fresh:
                print(f"  current relative altitude: {pos[3]:.1f}m")
            else:
                print("  WARNING: no fresh GLOBAL_POSITION_INT -- check the telemetry link")
            last_print = now
        if fresh and pos[3] > takeoff_altitude_m:
            print(f"Takeoff altitude reached ({pos[3]:.1f}m). Starting capture.\n")
            return pos
        time.sleep(0.2)


def _fmt(v, nd):
    return '' if v is None else f"{v:.{nd}f}"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--interval', type=float, default=2.0, help='seconds between captures (default 2)')
    ap.add_argument('--takeoff-altitude-m', type=float, default=DEFAULT_TAKEOFF_ALTITUDE_M,
                    help=f'relative (AGL) altitude that must be exceeded before capture starts '
                         f'(default {DEFAULT_TAKEOFF_ALTITUDE_M:.0f})')
    ap.add_argument('--land-altitude-m', type=float, default=2.0,
                    help='relative (AGL) altitude, after takeoff, below which the program stops (default 2)')
    ap.add_argument('--port', default='/dev/ttyTHS1')
    ap.add_argument('--baud', type=int, default=57600)
    ap.add_argument('--sensor-id', '--cam', dest='sensor_id', type=int, choices=[0, 1], default=1,
                    help='which CSI camera port to use: 0 = CAM0, 1 = CAM1 (default 1)')
    ap.add_argument('--out-dir', default=None,
                    help='base folder for photos (default: photo_obtained/ next to this script) -- '
                         'each run gets its own timestamped subfolder underneath this')
    ap.add_argument('--wave-seconds', type=int, default=3,
                    help='seconds to rotate/move the camera for during the startup self-check (default 3)')
    cam = ap.add_argument_group(
        'camera exposure (defaults come from config.py; LO HI are the auto-exposure limits -- '
        'set LO == HI to fix the value)')
    cam.add_argument('--exposure-ms', type=float, nargs=2, metavar=('LO', 'HI'),
                     default=[v / 1e6 for v in config.EXPOSURE_TIME_RANGE_NS],
                     help='shutter / exposure time range in ms (HW: 0.013-683.709). '
                          'Default: %(default)s')
    cam.add_argument('--gain', type=float, nargs=2, metavar=('LO', 'HI'),
                     default=list(config.GAIN_RANGE),
                     help='sensor analog gain range (HW: 1.0-10.625). Default: %(default)s')
    cam.add_argument('--isp-gain', type=float, nargs=2, metavar=('LO', 'HI'),
                     default=list(config.ISP_DIGITAL_GAIN_RANGE),
                     help='ISP digital gain range (1.0-256.0). Default: %(default)s')
    cam.add_argument('--ev', type=float, default=None,
                     help='exposure compensation in EV, -2.0 to +2.0 (default: not set = 0)')
    cam.add_argument('--tnr-mode', type=int, choices=[0, 1, 2], default=0,
                     help='temporal noise reduction: 0=off, 1=fast, 2=high quality (default 0 = OFF; '
                          'the camera driver itself would default to 1 = ON, which smears moving shots)')
    cam.add_argument('--tnr-strength', type=float, default=None,
                     help='TNR strength -1.0..1.0 (only matters if --tnr-mode > 0; default: not set)')
    args = ap.parse_args()

    def _check_range(name, rng, lo_lim, hi_lim):
        lo, hi = rng
        if not (lo_lim <= lo <= hi <= hi_lim):
            ap.error(f"{name} must satisfy {lo_lim} <= LO <= HI <= {hi_lim} (got {lo} {hi})")
    _check_range('--exposure-ms', args.exposure_ms, 0.013, 683.709)
    _check_range('--gain', args.gain, 1.0, 10.625)
    _check_range('--isp-gain', args.isp_gain, 1.0, 256.0)
    if args.tnr_strength is not None and not -1.0 <= args.tnr_strength <= 1.0:
        ap.error(f"--tnr-strength must be between -1.0 and 1.0 (got {args.tnr_strength})")
    if args.ev is not None and not -2.0 <= args.ev <= 2.0:
        ap.error(f"--ev must be between -2.0 and 2.0 (got {args.ev})")
    if args.interval <= 0:
        ap.error(f"--interval must be > 0 (got {args.interval})")
    exposure_ns = tuple(int(round(v * 1e6)) for v in args.exposure_ms)

    base_out_dir = args.out_dir or os.path.join(os.path.dirname(os.path.abspath(__file__)), 'photo_obtained')
    run_id = datetime.now().strftime('%Y%m%d_%H%M%S')
    out_dir = os.path.join(base_out_dir, run_id)
    os.makedirs(out_dir, exist_ok=True)
    print(f"This run's photos will be saved to: {out_dir}\n")

    # Record the exact settings of this run next to its photos.
    with open(os.path.join(out_dir, 'run_args.txt'), 'w') as f:
        f.write(f"started: {datetime.now().isoformat(timespec='seconds')}\n")
        f.write(f"command: {' '.join(sys.argv)}\n")
        for k, v in sorted(vars(args).items()):
            f.write(f"{k}: {v}\n")
        f.write(f"exposure_time_range_ns: {exposure_ns[0]} {exposure_ns[1]}\n")

    conn = connect_pixhawk(args.port, args.baud)
    telem = Telemetry(conn)

    # Startup read: only to fail fast if there is no fix. The tracker is
    # re-anchored at takeoff (see change 1 in the docstring).
    pos = telem.wait_fresh(telem.position, timeout=10)
    if pos is None:
        telem.stop()
        raise SystemExit(
            "Connected to Pixhawk (heartbeat OK), but no GLOBAL_POSITION_INT arrived within 10s. "
            "Usually: no GPS fix yet, or EKF origin not set. Check GPS status in the GCS, then retry."
        )
    heading = telem.wait_fresh(telem.heading, timeout=10)
    if heading is None:
        telem.stop()
        raise SystemExit(
            "Got a GPS position, but no VFR_HUD heading within 10s. Check the MAVLink "
            "telemetry stream rate/config, then retry."
        )
    tracker = PositionTracker(lat=pos[0], lon=pos[1], alt_m=pos[2], heading_deg=heading)
    print(f"Startup fix: {pos[0]:.7f}, {pos[1]:.7f}  alt_msl={pos[2]:.1f}m  agl={pos[3]:.1f}m  "
          f"heading={heading:.0f}deg  (tracker will be RE-ANCHORED at takeoff)\n")

    # Initialised BEFORE the try so the except/summary never hit undefined names.
    n_frame = 0
    n_save_failures = 0
    agg_distance_m = 0.0
    capture_started = False
    cap = None
    csv_file = None

    try:
        print(f"Opening camera CAM{args.sensor_id} ({IMAGE_WIDTH}x{IMAGE_HEIGHT}, {CAMERA_HFOV_DEG:.0f}deg HFOV, "
              f"FX_PIXELS=FY_PIXELS={FX_PIXELS:.1f})...")
        print(f"Camera shutter/exposure: exposuretimerange={args.exposure_ms[0]:.3f}-"
              f"{args.exposure_ms[1]:.3f}ms  "
              f"gainrange={args.gain[0]:.2f}-{args.gain[1]:.3f}x  "
              f"ispdigitalgainrange={args.isp_gain[0]:.2f}-{args.isp_gain[1]:.2f}x  "
              f"ev={'not set' if args.ev is None else f'{args.ev:+.2f}'}")
        print(f"Camera ISP: tnr-mode={args.tnr_mode} ({'OFF' if args.tnr_mode == 0 else 'ON'})  "
              f"tnr-strength={'not set' if args.tnr_strength is None else args.tnr_strength}")
        pipeline = build_pipeline(args.sensor_id, exposure_ns, tuple(args.gain),
                                  tuple(args.isp_gain), args.ev, args.tnr_mode, args.tnr_strength)
        print(f"GStreamer pipeline: {pipeline}")
        cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        if not cap.isOpened():
            raise RuntimeError("Could not open CSI camera — check sensor connection / nvargus-daemon.")
        for _ in range(10):  # let exposure settle
            cap.read()
        print("Camera open.\n")

        self_check_camera(cap, args.wave_seconds)

        # ── Re-anchor at takeoff ──
        takeoff_pos = wait_for_takeoff(telem, args.takeoff_altitude_m)
        takeoff_heading = telem.wait_fresh(telem.heading, timeout=5)
        if takeoff_heading is None:
            takeoff_heading = heading
            print(f"WARNING: no fresh heading at takeoff -- re-using startup heading {heading:.0f}deg")
        tracker.reset_anchor(takeoff_pos[0], takeoff_pos[1], takeoff_pos[2], takeoff_heading)
        last_real = takeoff_pos
        print(f"Anchor (at takeoff): {takeoff_pos[0]:.7f}, {takeoff_pos[1]:.7f}  "
              f"alt_msl={takeoff_pos[2]:.1f}m  agl={takeoff_pos[3]:.1f}m  heading={takeoff_heading:.0f}deg")

        csv_path = os.path.join(out_dir, 'vo_log.csv')
        csv_file = open(csv_path, 'w', newline='')
        writer = csv.DictWriter(csv_file, fieldnames=CSV_FIELDS)
        writer.writeheader()
        print(f"CSV log: {csv_path}")
        print(f"Capturing every {args.interval}s.\n")
        capture_started = True

        prev_gray = prev_time = prev_alt_agl = None
        land_debounce_count = 0
        stuck_check_frame = 0
        stuck_check_real_latlon = None
        stuck_check_agg_distance_m = 0.0

        while True:
            frame, gray = capture_gray(cap)
            now = time.time()
            if frame is None:
                print("WARNING: frame grab failed, skipping")
                time.sleep(args.interval)
                continue
            n_frame += 1

            # Latest real telemetry; if stale, keep the LAST KNOWN real values
            # (never the anchor) and flag it.
            pos, real_age = telem.position()
            real_fresh = pos is not None and real_age <= TELEMETRY_STALE_S
            if pos is not None:
                last_real = pos
            real_lat, real_lon, real_alt_msl, cur_alt_agl = last_real
            if not real_fresh:
                print(f"WARNING: real GPS is stale ({_fmt(real_age, 1) or 'n/a'}s old) -- "
                      f"using last known position, not counted for stuck/landing checks")
            real_heading, heading_age = telem.heading()

            dt_s = None
            if prev_gray is None:
                r = {'n_matches': 0, 'n_inliers': 0, 'ok': False}
                status = 'FIRST'
                move_n = move_e = move_z = 0.0
            else:
                dt_s = now - prev_time
                r = process_pair(prev_gray, gray, prev_alt_agl, cur_alt_agl, args.takeoff_altitude_m)
                state = tracker.apply_leg(r, dt_s=dt_s)
                move_n = state['vn'] * dt_s
                move_e = state['ve'] * dt_s
                move_z = r.get('altitude_change_m', 0.0) if r['ok'] else 0.0
                if r['ok']:
                    agg_distance_m += r['magnitude_m']
                status = 'OK' if r.get('confident') else ('FAIL' if not r['ok'] else 'LOW-CONF')

                print(f"[frame {n_frame}] [{status:<8}] dt={dt_s:.2f}s "
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
                f"{move_n:.2f}_{move_e:.2f}_{move_z:.2f}_"
                f"{r['n_matches']}_{r['n_inliers']}_{agg_distance_m:.2f}.png"
            )
            save_path = os.path.join(out_dir, fname)
            if not cv2.imwrite(save_path, frame):
                n_save_failures += 1
                free_gb = shutil.disk_usage(out_dir).free / (1024 ** 3)
                print(f"WARNING: failed to save photo (disk full / write error?) -- "
                      f"{save_path}  [{free_gb:.1f}GB free, {n_save_failures} failure(s) so far]")
            print(f"[frame {n_frame}] photo: {fname}")

            writer.writerow({
                'frame': n_frame, 'timestamp': f"{now:.3f}", 'filename': fname, 'status': status,
                'real_lat': f"{real_lat:.7f}", 'real_lon': f"{real_lon:.7f}",
                'real_alt_msl': f"{real_alt_msl:.2f}", 'real_alt_agl': f"{cur_alt_agl:.2f}",
                'real_age_s': _fmt(real_age, 2),
                'real_heading_deg': _fmt(real_heading, 1), 'heading_age_s': _fmt(heading_age, 2),
                'vo_lat': f"{tracker.lat:.7f}", 'vo_lon': f"{tracker.lon:.7f}",
                'vo_alt_msl': f"{tracker.alt_m:.2f}", 'vo_heading_deg': f"{tracker.heading_deg % 360:.2f}",
                'real_vo_diff_m': f"{diff_m:.2f}",
                'move_north_m': f"{move_n:.3f}", 'move_east_m': f"{move_e:.3f}", 'move_up_m': f"{move_z:.3f}",
                'matches': r['n_matches'], 'inliers': r['n_inliers'],
                'magnitude_m': _fmt(r.get('magnitude_m'), 3),
                'yaw_change_deg': _fmt(r.get('yaw_change_deg'), 3),
                'scale': _fmt(r.get('scale'), 5),
                'altitude_used_m': _fmt(r.get('altitude_used_m'), 2),
                'agg_distance_m': f"{agg_distance_m:.2f}", 'hacc_m': f"{tracker.hacc_m:.2f}",
                'dt_s': _fmt(dt_s, 3),
            })
            csv_file.flush()

            prev_gray, prev_time, prev_alt_agl = gray, now, cur_alt_agl

            # Stuck-camera check -- only on fresh real positions.
            if real_fresh:
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
                              f"{vo_moved_m:.2f}m (or legs kept failing). Check the live camera feed.")
                    stuck_check_real_latlon = (real_lat, real_lon)
                    stuck_check_agg_distance_m = agg_distance_m
                    stuck_check_frame = n_frame

            # Landing check -- only FRESH readings count (a stale low reading
            # neither advances nor resets the debounce counter).
            if real_fresh:
                if cur_alt_agl < args.land_altitude_m:
                    land_debounce_count += 1
                    if land_debounce_count < LAND_DEBOUNCE_FRAMES:
                        print(f"Relative altitude {cur_alt_agl:.1f}m < land threshold "
                              f"{args.land_altitude_m:.1f}m ({land_debounce_count}/{LAND_DEBOUNCE_FRAMES})")
                    else:
                        print(f"\nRelative altitude {cur_alt_agl:.1f}m < land threshold "
                              f"{args.land_altitude_m:.1f}m for {LAND_DEBOUNCE_FRAMES} consecutive readings "
                              f"— landed, stopping.")
                        break
                else:
                    land_debounce_count = 0

            elapsed = time.time() - now
            if elapsed > args.interval:
                print(f"WARNING: processing this frame took {elapsed:.2f}s, longer than "
                      f"--interval {args.interval}s -- actual capture interval is {elapsed:.2f}s")
            time.sleep(max(0.0, args.interval - elapsed))

    except KeyboardInterrupt:
        where = "during capture" if capture_started else "before capture started"
        print(f"\nStopped early by user ({where}) after {n_frame} frame(s).")
    finally:
        if cap is not None:
            cap.release()
        if csv_file is not None:
            csv_file.close()
        telem.stop()

    print(f"Final: {n_frame} frame(s) captured, {tracker.legs_since_anchor} leg(s) applied, "
          f"aggregated flight distance {agg_distance_m:.1f}m, "
          f"final VO position {tracker.lat:.7f},{tracker.lon:.7f} hacc={tracker.hacc_m:.2f}m")
    if n_save_failures:
        print(f"WARNING: {n_save_failures}/{n_frame} photo(s) FAILED TO SAVE this run -- "
              f"check disk space before the next flight.")


if __name__ == '__main__':
    main()
