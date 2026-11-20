#!/usr/bin/env python3
"""
vo_flight_capture_fix2.py — vo_flight_capture_fix1.py + BURST CAPTURE with
best-frame selection ("lucky imaging").

vo_flight_capture_fix.py / _fix1.py are left untouched. Run this one directly.

New in fix2 (vs fix1):

  B1. At every capture time, --burst N consecutive frames are grabbed (default
      3; at 30 fps that is ~0.07 s, i.e. ~0.5 m even at 7 m/s -- the "same
      place" for VO). The frame used for VO is picked by --pick:
        sharpness (default): highest variance of the Laplacian on a 640x360
                             downscale. Only compared WITHIN a burst (same
                             ground), never across bursts (score is
                             scene-dependent: a running track scores far
                             higher than grass).
        gyro:                lowest body angular rate |omega| from ATTITUDE
                             rollspeed/pitchspeed/yawspeed at that frame's
                             time -- a scene-independent motion-blur proxy.
      Both scores are always logged for every burst frame, so the two
      strategies can be compared offline. --burst 1 = fix1 behaviour.
      Rationale (11/14 data): per-frame sharpness swung 20x between
      neighbouring cruise frames (e.g. 51 vs 1178), i.e. blur was
      intermittent -- the case where best-of-N helps. It does NOT help
      against TNR smear (all frames equally blended; TNR stays off) nor
      against too little overlap / too much yaw per leg (use --interval 1).
  B2. The chosen frame's OWN timestamp is used for attitude interpolation
      and dt.
  B3. --save-burst also writes the non-chosen burst frames to a burst/
      subfolder (not VO_* names, so replays of the run folder ignore them).
  B4. ATTITUDE angular rates are kept in the history and attitude_log.csv.
  B5. New vo_log.csv columns: burst_n, burst_pick, sharpness, gyro_dps,
      burst_sharpness, burst_gyro_dps ('|'-separated, in capture order).

Changes in fix1 (vs vo_flight_capture_fix.py), all kept:

  1. TEMPORAL NOISE REDUCTION OFF (--tnr-mode, default 0).
     The 11/14 cruise frames were smeared by tens of pixels although a 5 ms
     shutter at 7 m/s / 60 m is < 1 px of motion blur. nvarguscamerasrc runs
     TNR by default (tnr-mode=1); in dim light with high gain it blends
     several consecutive frames, which leaves exactly this kind of streak
     when the camera moves. TNR is now off by default. Edge enhancement can
     also be set with --ee-mode (default: not set = Argus default).

  2. HEADING FROM THE COMPASS, NOT FROM ACCUMULATED VO YAW.
     On 11/14 the aircraft yawed ~60 deg while VO legs were failing, so the VO
     heading never saw it and ended 95 deg off. Heading now comes from the
     Pixhawk ATTITUDE yaw (EKF / compass) at every frame -- it is available
     with or without GPS.

  3. TILT COMPENSATION (ATTITUDE roll/pitch).
     The old math assumed a perfectly nadir camera, so every degree of
     roll/pitch change looked like ~1 m of motion at 60 m, and climbs/descents
     with a tilted camera produced fake horizontal displacement (1.5-4.5 m per
     leg while really hovering). Now every matched keypoint of each frame is
     projected onto the ground plane using THAT frame's roll/pitch/yaw and
     baro AGL, giving ground coordinates in metres (north/east, relative to
     the drone's nadir). A static ground point P satisfies
         pos1 + g1 = pos2 + g2   ->   pos2 - pos1 = g1 - g2
     so the leg displacement is fitted directly in metres (RANSAC similarity on
     ground coordinates; a correct solution has scale ~1 and rotation ~0, which
     are logged as a sanity check). Displacement is already north/east, so no
     heading integration is involved at all.
     Assumes flat ground and an uncalibrated pinhole camera (FX_PIXELS below,
     principal point = image centre, no lens distortion).

  4. ATTITUDE is requested at --attitude-hz (default 20 Hz) with
     MAV_CMD_SET_MESSAGE_INTERVAL (not persistent -- resets when the Pixhawk
     reboots). Every ATTITUDE sample is written to attitude_log.csv in the run
     folder so camera latency / mount offset can be re-tuned offline.
     Attitude is interpolated at (frame time - --camera-latency-ms).

  5. vo_alt_msl now follows baro (anchor_msl + AGL change) instead of the
     accumulated image-scale altitude. The old pixel-only estimate is still
     computed and logged (raw_pixel_* columns) for A/B comparison.

THIS SCRIPT DOES NOT WRITE PARAMETERS TO THE PIXHAWK. It only sends the same
request_data_stream as before plus a non-persistent SET_MESSAGE_INTERVAL for
ATTITUDE, reads telemetry, and writes photo files + CSVs.

Filename format (unchanged, saved into photo_obtained/<run_timestamp>/):

    VO_yyyymmddhhmmss_<real_lat>_<real_lon>_<real_height>_
        <vo_lat>_<vo_lon>_<vo_height>_<real_vo_diff_m>_
        <vo_move_x>_<vo_move_y>_<vo_move_z>_
        <matches>_<inliers>_<aggregated_fly_distance>.png

  vo_move_x = north, vo_move_y = east, vo_move_z = up (meters, this leg).
  <inliers> is now the GROUND-fit inlier count.

Usage:
    python3 vo_flight_capture_fix2.py --port /dev/ttyTHS1 --baud 57600
    python3 vo_flight_capture_fix2.py --interval 1 --burst 3 --pick sharpness
    python3 vo_flight_capture_fix2.py --burst 5 --pick gyro --save-burst
    python3 vo_flight_capture_fix2.py --camera-latency-ms 60 --attitude-hz 25

  Every run writes its exact settings to run_args.txt in the run folder.
"""
import argparse
import bisect
import collections
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
CX_PIXELS = IMAGE_WIDTH / 2.0
CY_PIXELS = IMAGE_HEIGHT / 2.0

DEFAULT_TAKEOFF_ALTITUDE_M = 30.0

# A telemetry value older than this is considered stale.
TELEMETRY_STALE_S = 2.0

# ATTITUDE: keep this much history for interpolation, and accept a frame
# time up to this far past the newest sample (hold, no extrapolation).
ATTITUDE_HISTORY_S = 20.0
ATTITUDE_MAX_HOLD_S = 0.25
MAVLINK_MSG_ID_ATTITUDE = 30

# Ground projection: drop rays closer than this to horizontal (cos of the
# angle from straight down; 0.26 ~= 75 deg off-nadir).
MIN_RAY_DOWN_COMPONENT = 0.26
# Never project with an AGL below this (flat-ground model breaks down).
MIN_PROJECTION_AGL_M = 1.0
# Ground-fit sanity: a correct fit has scale ~1 and rotation ~0. Outside
# these the leg is still applied but flagged LOW-CONF.
GROUND_SCALE_SANE = (0.85, 1.15)
GROUND_ROT_SANE_DEG = 8.0

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
    'roll_deg', 'pitch_deg', 'att_yaw_deg',
    'vo_lat', 'vo_lon', 'vo_alt_msl', 'vo_heading_deg', 'real_vo_diff_m',
    'move_north_m', 'move_east_m', 'move_up_m',
    'matches', 'inliers', 'magnitude_m', 'yaw_change_deg',
    'ground_scale', 'ground_rot_deg',
    'raw_inliers', 'raw_pixel_magnitude_m', 'scale', 'raw_yaw_change_deg',
    'altitude_used_m', 'agg_distance_m', 'hacc_m', 'dt_s',
    'burst_n', 'burst_pick', 'sharpness', 'gyro_dps', 'burst_sharpness', 'burst_gyro_dps',
]

# Sharpness is computed on this downscale (fast, and less noise-dominated).
SHARPNESS_SIZE = (640, 360)


# ── Telemetry: background reader keeping only the latest values ────────────

class Telemetry:
    """Continuously drains the MAVLink link in a background thread. Keeps the
    LATEST GLOBAL_POSITION_INT and VFR_HUD, plus a short ATTITUDE history for
    interpolation at frame time. After construction, no other code may call
    recv_match on the same connection."""

    def __init__(self, conn):
        self._conn = conn
        self._lock = threading.Lock()
        self._pos = None
        self._pos_t = None
        self._heading = None
        self._heading_t = None
        self._att_t = collections.deque()
        self._att_v = collections.deque()   # (roll, pitch, yaw) radians
        self._att_w = collections.deque()   # |body angular rate| rad/s
        self._att_log = None
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name='mavlink-reader', daemon=True)
        self._thread.start()

    def set_attitude_log(self, f):
        """Start writing every ATTITUDE sample to the (already open) file f."""
        f.write('host_time,time_boot_ms,roll_deg,pitch_deg,yaw_deg,'
                'rollspeed_dps,pitchspeed_dps,yawspeed_dps\n')
        with self._lock:
            self._att_log = f

    def _run(self):
        while not self._stop.is_set():
            try:
                msg = self._conn.recv_match(type=['GLOBAL_POSITION_INT', 'VFR_HUD', 'ATTITUDE'],
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
            elif mtype == 'ATTITUDE':
                w = math.sqrt(msg.rollspeed ** 2 + msg.pitchspeed ** 2 + msg.yawspeed ** 2)
                with self._lock:
                    self._att_t.append(now)
                    self._att_v.append((msg.roll, msg.pitch, msg.yaw))
                    self._att_w.append(w)
                    while self._att_t and now - self._att_t[0] > ATTITUDE_HISTORY_S:
                        self._att_t.popleft()
                        self._att_v.popleft()
                        self._att_w.popleft()
                    if self._att_log is not None:
                        try:
                            self._att_log.write(
                                f"{now:.3f},{msg.time_boot_ms},{math.degrees(msg.roll):.3f},"
                                f"{math.degrees(msg.pitch):.3f},{math.degrees(msg.yaw) % 360:.3f},"
                                f"{math.degrees(msg.rollspeed):.3f},{math.degrees(msg.pitchspeed):.3f},"
                                f"{math.degrees(msg.yawspeed):.3f}\n")
                        except ValueError:  # file closed during shutdown
                            self._att_log = None

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

    def attitude_latest(self):
        """((roll, pitch, yaw) radians, age_s) of the newest ATTITUDE, or (None, None)."""
        with self._lock:
            if not self._att_t:
                return None, None
            return self._att_v[-1], time.time() - self._att_t[-1]

    def attitude_at(self, t):
        """(roll, pitch, yaw) radians interpolated at host time t, or None if
        t is outside the buffered history (or too far past the newest sample)."""
        with self._lock:
            times = list(self._att_t)
            vals = list(self._att_v)
        if not times or t < times[0]:
            return None
        if t >= times[-1]:
            return vals[-1] if t - times[-1] <= ATTITUDE_MAX_HOLD_S else None
        i = bisect.bisect_right(times, t)
        t0, t1 = times[i - 1], times[i]
        (r0, p0, y0), (r1, p1, y1) = vals[i - 1], vals[i]
        w = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
        dyaw = (y1 - y0 + math.pi) % (2 * math.pi) - math.pi
        return (r0 + w * (r1 - r0), p0 + w * (p1 - p0), y0 + w * dyaw)

    def angular_rate_at(self, t):
        """|body angular rate| (rad/s) interpolated at host time t, or None."""
        with self._lock:
            times = list(self._att_t)
            rates = list(self._att_w)
        if not times or t < times[0]:
            return None
        if t >= times[-1]:
            return rates[-1] if t - times[-1] <= ATTITUDE_MAX_HOLD_S else None
        i = bisect.bisect_right(times, t)
        t0, t1 = times[i - 1], times[i]
        w = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
        return rates[i - 1] + w * (rates[i] - rates[i - 1])

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


def request_attitude_rate(conn, hz):
    """Ask ArduPilot to stream ATTITUDE at `hz` (non-persistent)."""
    from pymavlink import mavutil
    conn.mav.command_long_send(
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        MAVLINK_MSG_ID_ATTITUDE, int(1e6 / hz), 0, 0, 0, 0, 0)


# ── Geometry: pixels -> ground plane (north/east metres from nadir) ────────

def _rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def _rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def _rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def pixels_to_ground(pts, attitude, agl_m):
    """Project Nx2 pixel coords onto the flat ground.

    attitude: (roll, pitch, yaw) radians, ArduPilot convention (body FRD ->
    NED = Rz(yaw) Ry(pitch) Rx(roll)). Camera looks straight down along body
    +Z with image-up = body forward (rotated by CAMERA_MOUNT_YAW_OFFSET_DEG,
    same convention as dead_reckoning.PositionTracker).

    Returns (Nx2 float32 array of (north_m, east_m) relative to the drone's
    nadir, boolean valid mask).
    """
    pts = np.asarray(pts, dtype=np.float64).reshape(-1, 2)
    x = (pts[:, 0] - CX_PIXELS) / FX_PIXELS    # image right
    y = (pts[:, 1] - CY_PIXELS) / FY_PIXELS    # image down
    ray_body = np.vstack([-y, x, np.ones_like(x)])   # FRD: forward, right, down
    roll, pitch, yaw = attitude
    R = (_rot_z(yaw) @ _rot_y(pitch) @ _rot_x(roll)
         @ _rot_z(math.radians(config.CAMERA_MOUNT_YAW_OFFSET_DEG)))
    ray_ned = R @ ray_body
    norm = np.linalg.norm(ray_ned, axis=0)
    valid = ray_ned[2] / norm > MIN_RAY_DOWN_COMPONENT
    h = max(agl_m, MIN_PROJECTION_AGL_M)
    with np.errstate(divide='ignore', invalid='ignore'):
        g = (h * ray_ned[:2] / ray_ned[2]).T
    g[~valid] = 0.0
    return g.astype(np.float32), valid


# ── VO ─────────────────────────────────────────────────────────────────────

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


def _raw_pixel_displacement(M, altitude_m):
    """The old nadir-only pixel math (kept for A/B logging only)."""
    a, b, tx = M[0]
    c, d, ty = M[1]
    scale = math.sqrt(a * a + c * c)
    center2 = np.array([a * CX_PIXELS + b * CY_PIXELS + tx, c * CX_PIXELS + d * CY_PIXELS + ty])
    shift = -(center2 - np.array([CX_PIXELS, CY_PIXELS]))
    return {
        'raw_pixel_magnitude_m': math.hypot(shift[0] * altitude_m / FX_PIXELS,
                                            shift[1] * altitude_m / FY_PIXELS),
        'scale': scale,
        'raw_yaw_change_deg': -math.degrees(math.atan2(c, a)),
    }


def process_pair_ground(gray1, gray2, att1, att2, agl1, agl2):
    """Tilt- and heading-compensated leg: pos2 - pos1 in metres (north, east)."""
    pts1, pts2 = _detect_and_match_full_frame(gray1, gray2)
    n_matches = len(pts1)
    result = {'n_matches': n_matches, 'n_inliers': 0, 'ok': False, 'confident': False}
    altitude_m = (agl1 + agl2) / 2.0
    result['altitude_used_m'] = altitude_m

    # Old pixel-only estimate, for comparison in the CSV.
    M_raw, inl_raw = vo.estimate_similarity(pts1, pts2)
    result['raw_inliers'] = int(inl_raw.sum()) if inl_raw is not None else 0
    if M_raw is not None:
        result.update(_raw_pixel_displacement(M_raw, altitude_m))

    if att1 is None or att2 is None:
        result['reason'] = 'no ATTITUDE for this frame pair'
        return result
    if n_matches < 4:
        result['reason'] = 'too few matches'
        return result

    g1, v1 = pixels_to_ground(pts1, att1, agl1)
    g2, v2 = pixels_to_ground(pts2, att2, agl2)
    keep = v1 & v2
    g1, g2 = g1[keep], g2[keep]
    if len(g1) < 4:
        result['reason'] = 'too few matches after ground projection'
        return result

    thresh_m = config.RANSAC_REPROJ_THRESHOLD_PX * altitude_m / FX_PIXELS
    M, inliers = cv2.estimateAffinePartial2D(g1, g2, method=cv2.RANSAC,
                                             ransacReprojThreshold=thresh_m, confidence=0.99)
    n_inliers = int(inliers.sum()) if inliers is not None else 0
    result['n_inliers'] = n_inliers
    if M is None or n_inliers < config.MIN_INLIERS:
        result['reason'] = 'too few inliers'
        return result

    A, t = M[:, :2], M[:, 2]
    # Frame-2 nadir (g2 = 0) expressed in frame-1 ground coords = pos2 - pos1.
    north_m, east_m = np.linalg.solve(A, -t)
    scale = math.hypot(A[0, 0], A[1, 0])
    rot_deg = math.degrees(math.atan2(A[1, 0], A[0, 0]))

    result.update({
        'ok': True,
        'north_m': float(north_m), 'east_m': float(east_m),
        'magnitude_m': float(math.hypot(north_m, east_m)),
        'ground_scale': scale, 'ground_rot_deg': rot_deg,
        'yaw_change_deg': math.degrees((att2[2] - att1[2] + math.pi) % (2 * math.pi) - math.pi),
    })
    reasons = []
    if n_inliers / max(1, len(g1)) < config.MIN_INLIER_RATIO:
        reasons.append('low inlier ratio')
    if not GROUND_SCALE_SANE[0] <= scale <= GROUND_SCALE_SANE[1]:
        reasons.append(f'ground scale {scale:.2f} not ~1 (altitude/FX error?)')
    if abs(rot_deg) > GROUND_ROT_SANE_DEG:
        reasons.append(f'ground rotation {rot_deg:+.1f}deg not ~0 (compass/timing error?)')
    result['confident'] = not reasons
    if reasons:
        result['reason'] = '; '.join(reasons) + ' -- displacement UNTRUSTED'
    return result


class CompassTracker(PositionTracker):
    """PositionTracker whose heading is SET from the compass each leg and
    whose legs arrive already in north/east metres."""

    def apply_ground_leg(self, r, heading_deg, alt_msl_m, dt_s=None):
        self.legs_since_anchor += 1
        if heading_deg is not None:
            self.heading_deg = heading_deg + config.CAMERA_MOUNT_YAW_OFFSET_DEG
        if alt_msl_m is not None:
            self.alt_m = alt_msl_m

        if not r['ok']:
            self.hacc_m = min(config.HACC_MAX_M, self.hacc_m + config.HACC_PENALTY_NO_MATCH_M)
            return self._state(vn=0.0, ve=0.0)

        north_m, east_m = r['north_m'], r['east_m']
        self.lat += north_m / 111111.0
        self.lon += east_m / (111111.0 * math.cos(math.radians(self.lat)))

        if r['confident']:
            growth = config.HACC_GROWTH_FRACTION_CONFIDENT * r['magnitude_m']
            self.hacc_m = min(config.HACC_MAX_M, math.hypot(self.hacc_m, growth))
        else:
            growth = config.HACC_GROWTH_FRACTION_LOW_CONF * r['magnitude_m']
            self.hacc_m = min(config.HACC_MAX_M,
                              math.hypot(self.hacc_m, growth) + config.HACC_PENALTY_LOW_CONFIDENCE_M)

        vn = north_m / dt_s if dt_s else 0.0
        ve = east_m / dt_s if dt_s else 0.0
        return self._state(vn=vn, ve=ve)


# ── Camera ─────────────────────────────────────────────────────────────────

def build_pipeline(sensor_id, exposure_ns, gain, isp_gain, ev=None,
                   tnr_mode=0, tnr_strength=None, ee_mode=None, ee_strength=None):
    pipeline = gstreamer_pipeline(
        sensor_id=sensor_id, width=IMAGE_WIDTH, height=IMAGE_HEIGHT,
        exposure_time_range_ns=exposure_ns, gain_range=gain, isp_digital_gain_range=isp_gain,
    )
    extra = []
    if ev is not None:
        extra.append(f"exposurecompensation={ev}")
    if tnr_mode is not None:
        extra.append(f"tnr-mode={tnr_mode}")
    if tnr_strength is not None:
        extra.append(f"tnr-strength={tnr_strength}")
    if ee_mode is not None:
        extra.append(f"ee-mode={ee_mode}")
    if ee_strength is not None:
        extra.append(f"ee-strength={ee_strength}")
    if extra:
        src = f"nvarguscamerasrc sensor-id={sensor_id} "
        if src not in pipeline:
            raise RuntimeError("could not add nvarguscamerasrc properties -- pipeline format changed?")
        pipeline = pipeline.replace(src, src + ' '.join(extra) + ' ', 1)
    if 'max-buffers' not in pipeline:
        fixed = pipeline.replace('appsink drop=True', 'appsink drop=True max-buffers=1')
        if fixed == pipeline:
            print("WARNING: could not add max-buffers=1 to the appsink -- pipeline format changed?")
        pipeline = fixed
    return pipeline


def sharpness_score(gray):
    """Variance of the Laplacian on a downscale. Higher = sharper. Only
    meaningful when comparing frames of (nearly) the same scene."""
    small = cv2.resize(gray, SHARPNESS_SIZE, interpolation=cv2.INTER_AREA)
    return float(cv2.Laplacian(small, cv2.CV_64F).var())


def capture_burst(cap, telem, n, pick, latency_s):
    """Grab n consecutive frames and pick one.

    Returns (chosen_index, frames) where frames is a list of dicts
    {frame, gray, t, sharpness, gyro_dps}, or (None, frames) if no frame
    could be grabbed. pick='gyro' falls back to sharpness when any frame
    has no angular-rate sample.
    """
    frames = []
    for _ in range(n):
        frame, gray = capture_gray(cap)
        t = time.time()
        if frame is None:
            continue
        w = telem.angular_rate_at(t - latency_s)
        frames.append({'frame': frame, 'gray': gray, 't': t,
                       'sharpness': sharpness_score(gray),
                       'gyro_dps': None if w is None else math.degrees(w)})
    if not frames:
        return None, frames
    if pick == 'gyro' and all(f['gyro_dps'] is not None for f in frames):
        best = min(range(len(frames)), key=lambda k: frames[k]['gyro_dps'])
    else:
        best = max(range(len(frames)), key=lambda k: frames[k]['sharpness'])
    return best, frames


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


def _deg(att, i, wrap=False):
    if att is None:
        return None
    v = math.degrees(att[i])
    return v % 360 if wrap else v


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
    ap.add_argument('--attitude-hz', type=float, default=20.0,
                    help='ATTITUDE stream rate requested from the Pixhawk (default 20)')
    ap.add_argument('--camera-latency-ms', type=float, default=0.0,
                    help='frame age at cap.read() return; attitude is sampled at '
                         '(read time - this). Uncalibrated -- tune offline from attitude_log.csv (default 0)')
    ap.add_argument('--burst', type=int, default=3,
                    help='frames grabbed per capture; the best one is used for VO (default 3, 1 = off)')
    ap.add_argument('--pick', choices=['sharpness', 'gyro'], default='sharpness',
                    help='how to pick the burst frame: sharpness = highest Laplacian variance (default), '
                         'gyro = lowest ATTITUDE angular rate')
    ap.add_argument('--save-burst', action='store_true',
                    help='also save the non-chosen burst frames into <run>/burst/')
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
                     help='temporal noise reduction: 0=off, 1=fast, 2=high quality (default 0 = OFF)')
    cam.add_argument('--tnr-strength', type=float, default=None,
                     help='TNR strength -1.0..1.0 (only matters if --tnr-mode > 0; default: not set)')
    cam.add_argument('--ee-mode', type=int, choices=[0, 1, 2], default=None,
                     help='edge enhancement: 0=off, 1=fast, 2=high quality (default: not set = Argus default)')
    cam.add_argument('--ee-strength', type=float, default=None,
                     help='edge enhancement strength -1.0..1.0 (default: not set)')
    args = ap.parse_args()

    def _check_range(name, rng, lo_lim, hi_lim):
        lo, hi = rng
        if not (lo_lim <= lo <= hi <= hi_lim):
            ap.error(f"{name} must satisfy {lo_lim} <= LO <= HI <= {hi_lim} (got {lo} {hi})")
    _check_range('--exposure-ms', args.exposure_ms, 0.013, 683.709)
    _check_range('--gain', args.gain, 1.0, 10.625)
    _check_range('--isp-gain', args.isp_gain, 1.0, 256.0)
    if args.ev is not None and not -2.0 <= args.ev <= 2.0:
        ap.error(f"--ev must be between -2.0 and 2.0 (got {args.ev})")
    for name in ('tnr_strength', 'ee_strength'):
        v = getattr(args, name)
        if v is not None and not -1.0 <= v <= 1.0:
            ap.error(f"--{name.replace('_', '-')} must be between -1.0 and 1.0 (got {v})")
    if args.interval <= 0:
        ap.error(f"--interval must be > 0 (got {args.interval})")
    if args.attitude_hz <= 0:
        ap.error(f"--attitude-hz must be > 0 (got {args.attitude_hz})")
    if not 1 <= args.burst <= 10:
        ap.error(f"--burst must be 1..10 (got {args.burst})")
    exposure_ns = tuple(int(round(v * 1e6)) for v in args.exposure_ms)
    latency_s = args.camera_latency_ms / 1000.0

    base_out_dir = args.out_dir or os.path.join(os.path.dirname(os.path.abspath(__file__)), 'photo_obtained')
    run_id = datetime.now().strftime('%Y%m%d_%H%M%S')
    out_dir = os.path.join(base_out_dir, run_id)
    os.makedirs(out_dir, exist_ok=True)
    burst_dir = os.path.join(out_dir, 'burst')
    if args.save_burst:
        os.makedirs(burst_dir, exist_ok=True)
    print(f"This run's photos will be saved to: {out_dir}\n")

    # Record the exact settings of this run next to its photos.
    with open(os.path.join(out_dir, 'run_args.txt'), 'w') as f:
        f.write(f"started: {datetime.now().isoformat(timespec='seconds')}\n")
        f.write(f"command: {' '.join(sys.argv)}\n")
        for k, v in sorted(vars(args).items()):
            f.write(f"{k}: {v}\n")
        f.write(f"exposure_time_range_ns: {exposure_ns[0]} {exposure_ns[1]}\n")
        f.write(f"fx_pixels: {FX_PIXELS:.2f}\n")
        f.write(f"camera_mount_yaw_offset_deg: {config.CAMERA_MOUNT_YAW_OFFSET_DEG}\n")

    conn = connect_pixhawk(args.port, args.baud)
    request_attitude_rate(conn, args.attitude_hz)
    telem = Telemetry(conn)
    att_log_file = open(os.path.join(out_dir, 'attitude_log.csv'), 'w')
    telem.set_attitude_log(att_log_file)

    # Startup read: only to fail fast if there is no fix / no attitude. The
    # tracker is re-anchored at takeoff.
    pos = telem.wait_fresh(telem.position, timeout=10)
    if pos is None:
        telem.stop()
        att_log_file.close()
        raise SystemExit(
            "Connected to Pixhawk (heartbeat OK), but no GLOBAL_POSITION_INT arrived within 10s. "
            "Usually: no GPS fix yet, or EKF origin not set. Check GPS status in the GCS, then retry."
        )
    att = telem.wait_fresh(telem.attitude_latest, timeout=10)
    if att is None:
        telem.stop()
        att_log_file.close()
        raise SystemExit(
            "Got a GPS position, but no ATTITUDE message within 10s. Tilt/heading compensation "
            "needs it -- check the MAVLink stream rate (SRx_EXTRA1) / SET_MESSAGE_INTERVAL support."
        )
    heading = math.degrees(att[2]) % 360
    tracker = CompassTracker(lat=pos[0], lon=pos[1], alt_m=pos[2], heading_deg=heading)
    print(f"Startup fix: {pos[0]:.7f}, {pos[1]:.7f}  alt_msl={pos[2]:.1f}m  agl={pos[3]:.1f}m  "
          f"heading={heading:.0f}deg roll={math.degrees(att[0]):+.1f} pitch={math.degrees(att[1]):+.1f}  "
          f"(tracker will be RE-ANCHORED at takeoff)\n")

    # Initialised BEFORE the try so the except/summary never hit undefined names.
    n_frame = 0
    n_save_failures = 0
    agg_distance_m = 0.0
    capture_started = False
    cap = None
    csv_file = None

    try:
        print(f"Opening camera CAM{args.sensor_id} ({IMAGE_WIDTH}x{IMAGE_HEIGHT},{CAMERA_HFOV_DEG:.0f}deg HFOV, "
              f"FX_PIXELS=FY_PIXELS={FX_PIXELS:.1f})...")
        print(f"Camera shutter/exposure: exposuretimerange={args.exposure_ms[0]:.3f}-"
              f"{args.exposure_ms[1]:.3f}ms  "
              f"gainrange={args.gain[0]:.2f}-{args.gain[1]:.3f}x  "
              f"ispdigitalgainrange={args.isp_gain[0]:.2f}-{args.isp_gain[1]:.2f}x  "
              f"ev={'not set' if args.ev is None else f'{args.ev:+.2f}'}")
        print(f"Camera ISP: tnr-mode={args.tnr_mode} ({'OFF' if args.tnr_mode == 0 else 'ON'})  "
              f"tnr-strength={'not set' if args.tnr_strength is None else args.tnr_strength}  "
              f"ee-mode={'not set' if args.ee_mode is None else args.ee_mode}  "
              f"ee-strength={'not set' if args.ee_strength is None else args.ee_strength}")
        pipeline = build_pipeline(args.sensor_id, exposure_ns, tuple(args.gain), tuple(args.isp_gain),
                                  args.ev, args.tnr_mode, args.tnr_strength, args.ee_mode, args.ee_strength)
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
        takeoff_att = telem.wait_fresh(telem.attitude_latest, timeout=5)
        if takeoff_att is not None:
            takeoff_heading = math.degrees(takeoff_att[2]) % 360
        else:
            takeoff_heading = heading
            print(f"WARNING: no fresh ATTITUDE at takeoff -- re-using startup heading {heading:.0f}deg")
        tracker.reset_anchor(takeoff_pos[0], takeoff_pos[1], takeoff_pos[2], takeoff_heading)
        anchor_ground_msl = takeoff_pos[2] - takeoff_pos[3]  # baro: alt_msl = this + AGL
        last_real = takeoff_pos
        print(f"Anchor (at takeoff): {takeoff_pos[0]:.7f}, {takeoff_pos[1]:.7f}  "
              f"alt_msl={takeoff_pos[2]:.1f}m  agl={takeoff_pos[3]:.1f}m  heading={takeoff_heading:.0f}deg")

        csv_path = os.path.join(out_dir, 'vo_log.csv')
        csv_file = open(csv_path, 'w', newline='')
        writer = csv.DictWriter(csv_file, fieldnames=CSV_FIELDS)
        writer.writeheader()
        print(f"CSV log: {csv_path}")
        print(f"Capturing every {args.interval}s, burst={args.burst} pick={args.pick}"
              f"{' (saving all burst frames to burst/)' if args.save_burst else ''}.\n")
        capture_started = True

        prev_gray = prev_time = prev_alt_agl = prev_att = None
        land_debounce_count = 0
        stuck_check_frame = 0
        stuck_check_real_latlon = None
        stuck_check_agg_distance_m = 0.0

        while True:
            loop_start = time.time()
            best, burst = capture_burst(cap, telem, args.burst, args.pick, latency_s)
            if best is None:
                print("WARNING: frame grab failed (whole burst), skipping")
                time.sleep(args.interval)
                continue
            if len(burst) < args.burst:
                print(f"WARNING: only {len(burst)}/{args.burst} burst frames grabbed")
            chosen = burst[best]
            frame, gray, now = chosen['frame'], chosen['gray'], chosen['t']
            n_frame += 1
            cur_att = telem.attitude_at(now - latency_s)
            if cur_att is None:
                print("WARNING: no ATTITUDE sample covering this frame time -- this leg cannot be compensated")

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

            compass_heading = _deg(cur_att, 2, wrap=True)
            if compass_heading is None and real_heading is not None and heading_age <= TELEMETRY_STALE_S:
                compass_heading = real_heading
            baro_alt_msl = anchor_ground_msl + cur_alt_agl

            dt_s = None
            if prev_gray is None:
                r = {'n_matches': 0, 'n_inliers': 0, 'ok': False}
                status = 'FIRST'
                move_n = move_e = move_z = 0.0
                if compass_heading is not None:
                    tracker.heading_deg = compass_heading + config.CAMERA_MOUNT_YAW_OFFSET_DEG
            else:
                dt_s = now - prev_time
                r = process_pair_ground(prev_gray, gray, prev_att, cur_att, prev_alt_agl, cur_alt_agl)
                tracker.apply_ground_leg(r, compass_heading, baro_alt_msl, dt_s=dt_s)
                move_n = r['north_m'] if r['ok'] else 0.0
                move_e = r['east_m'] if r['ok'] else 0.0
                move_z = cur_alt_agl - prev_alt_agl
                if r['ok']:
                    agg_distance_m += r['magnitude_m']
                status = 'OK' if r.get('confident') else ('FAIL' if not r['ok'] else 'LOW-CONF')

                print(f"[frame {n_frame}] [{status:<8}] pick {best + 1}/{len(burst)} "
                      f"sharp={chosen['sharpness']:.0f} (min {min(f['sharpness'] for f in burst):.0f}) "
                      f"dt={dt_s:.2f}s "
                      f"matches={r['n_matches']} inliers={r['n_inliers']} (raw {r.get('raw_inliers', 0)})",
                      end='  ')
                if r['ok']:
                    print(f"N={r['north_m']:+.2f} E={r['east_m']:+.2f} ({r['magnitude_m']:.2f}m) "
                          f"gscale={r['ground_scale']:.3f} grot={r['ground_rot_deg']:+.1f}deg", end='  ')
                if r.get('reason'):
                    print(f"({r['reason']})", end='  ')
                if cur_att is not None:
                    print(f"att r={math.degrees(cur_att[0]):+.1f} p={math.degrees(cur_att[1]):+.1f} "
                          f"y={compass_heading:.0f}", end='  ')
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
            if args.save_burst:
                for k, bf in enumerate(burst):
                    if k == best:
                        continue
                    bname = f"{n_frame:04d}_{k + 1}of{len(burst)}_sharp{bf['sharpness']:.0f}.png"
                    bpath = os.path.join(burst_dir, bname)
                    if not cv2.imwrite(bpath, bf['frame']):
                        n_save_failures += 1
                        print(f"WARNING: failed to save burst frame {bpath}")
                    else:
                        print(f"[frame {n_frame}] burst (not used): burst/{bname}")

            writer.writerow({
                'frame': n_frame, 'timestamp': f"{now:.3f}", 'filename': fname, 'status': status,
                'real_lat': f"{real_lat:.7f}", 'real_lon': f"{real_lon:.7f}",
                'real_alt_msl': f"{real_alt_msl:.2f}", 'real_alt_agl': f"{cur_alt_agl:.2f}",
                'real_age_s': _fmt(real_age, 2),
                'real_heading_deg': _fmt(real_heading, 1), 'heading_age_s': _fmt(heading_age, 2),
                'roll_deg': _fmt(_deg(cur_att, 0), 2), 'pitch_deg': _fmt(_deg(cur_att, 1), 2),
                'att_yaw_deg': _fmt(_deg(cur_att, 2, wrap=True), 2),
                'vo_lat': f"{tracker.lat:.7f}", 'vo_lon': f"{tracker.lon:.7f}",
                'vo_alt_msl': f"{tracker.alt_m:.2f}", 'vo_heading_deg': f"{tracker.heading_deg % 360:.2f}",
                'real_vo_diff_m': f"{diff_m:.2f}",
                'move_north_m': f"{move_n:.3f}", 'move_east_m': f"{move_e:.3f}", 'move_up_m': f"{move_z:.3f}",
                'matches': r['n_matches'], 'inliers': r['n_inliers'],
                'magnitude_m': _fmt(r.get('magnitude_m'), 3),
                'yaw_change_deg': _fmt(r.get('yaw_change_deg'), 3),
                'ground_scale': _fmt(r.get('ground_scale'), 4),
                'ground_rot_deg': _fmt(r.get('ground_rot_deg'), 2),
                'raw_inliers': r.get('raw_inliers', ''),
                'raw_pixel_magnitude_m': _fmt(r.get('raw_pixel_magnitude_m'), 3),
                'scale': _fmt(r.get('scale'), 5),
                'raw_yaw_change_deg': _fmt(r.get('raw_yaw_change_deg'), 3),
                'altitude_used_m': _fmt(r.get('altitude_used_m'), 2),
                'agg_distance_m': f"{agg_distance_m:.2f}", 'hacc_m': f"{tracker.hacc_m:.2f}",
                'dt_s': _fmt(dt_s, 3),
                'burst_n': len(burst), 'burst_pick': best + 1,
                'sharpness': f"{chosen['sharpness']:.1f}",
                'gyro_dps': _fmt(chosen['gyro_dps'], 2),
                'burst_sharpness': '|'.join(f"{f['sharpness']:.1f}" for f in burst),
                'burst_gyro_dps': '|'.join(_fmt(f['gyro_dps'], 2) for f in burst),
            })
            csv_file.flush()

            prev_gray, prev_time, prev_alt_agl, prev_att = gray, now, cur_alt_agl, cur_att

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

            elapsed = time.time() - loop_start
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
        att_log_file.close()

    print(f"Final: {n_frame} frame(s) captured, {tracker.legs_since_anchor} leg(s) applied, "
          f"aggregated flight distance {agg_distance_m:.1f}m, "
          f"final VO position {tracker.lat:.7f},{tracker.lon:.7f} hacc={tracker.hacc_m:.2f}m")
    if n_save_failures:
        print(f"WARNING: {n_save_failures}/{n_frame} photo(s) FAILED TO SAVE this run -- "
              f"check disk space before the next flight.")


if __name__ == '__main__':
    main()
