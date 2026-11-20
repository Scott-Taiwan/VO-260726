#!/usr/bin/env python3
"""
photo_capture.py — plain photo + GPS logger. No VO, no takeoff/landing gating:
it starts capturing as soon as the camera is open and keeps going until it is
stopped (Ctrl+C, `./run_photo_capture.sh stop`, or --duration-s).

Every --interval seconds (default 0.5) it saves one frame named with the
capture time and the real GPS reading:

    CAP_<yyyymmdd>_<hhmmss>_<mmm>_<lat>_<lon>_<alt_msl>.png
    e.g. CAP_20261114_092227_500_25.0430236_121.4739233_37.32.png

    If no GPS position has been received yet: CAP_<yyyymmdd>_<hhmmss>_<mmm>_nogps.png

Files go into photo_obtained/<yyyymmdd_hhmmss>_capture/ together with:
    capture_log.csv  one row per photo: time, file name, lat/lon/alt_msl/AGL,
                     GPS age, compass heading, roll/pitch/yaw (if the Pixhawk
                     sends ATTITUDE), save time
    run_args.txt     the exact settings of this run

The Pixhawk is only READ (plus the same request_data_stream the other scripts
send). If no Pixhawk answers, capture still runs and names files "nogps".

Usage:
    python3 photo_capture.py --port /dev/ttyTHS1 --baud 57600
    python3 photo_capture.py --interval 0.5 --exposure-ms 1 3 --tnr-mode 0
    python3 photo_capture.py --no-mavlink --duration-s 60      # camera only, 1 minute
"""
import argparse
import csv
import os
import shutil
import signal
import sys
import threading
import time
import math
from datetime import datetime

# Same JetPack-vs-pip-opencv workaround as the other scripts -- before `import cv2`.
sys.path.insert(0, '/usr/lib/python3/dist-packages')
import cv2  # noqa: E402

import config  # noqa: E402

TELEMETRY_STALE_S = 2.0
MAX_CONSECUTIVE_GRAB_FAILURES = 20

CSV_FIELDS = [
    'index', 'time', 'unix_time', 'filename', 'gps_fresh',
    'lat', 'lon', 'alt_msl', 'alt_agl', 'gps_age_s', 'heading_deg',
    'roll_deg', 'pitch_deg', 'yaw_deg', 'att_age_s', 'save_ms',
]


class Telemetry:
    """Background MAVLink reader keeping only the latest GPS / heading / attitude."""

    def __init__(self, conn):
        self._conn = conn
        self._lock = threading.Lock()
        self._pos = self._pos_t = None
        self._hdg = self._hdg_t = None
        self._att = self._att_t = None
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name='mavlink-reader', daemon=True)
        self._thread.start()

    def _run(self):
        while not self._stop.is_set():
            try:
                msg = self._conn.recv_match(type=['GLOBAL_POSITION_INT', 'VFR_HUD', 'ATTITUDE'],
                                            blocking=True, timeout=0.5)
            except Exception as e:  # a transient serial error must not stop logging
                print(f"WARNING: MAVLink read error: {e}")
                time.sleep(0.1)
                continue
            if msg is None:
                continue
            now = time.time()
            t = msg.get_type()
            with self._lock:
                if t == 'GLOBAL_POSITION_INT':
                    if msg.lat == 0 and msg.lon == 0:
                        continue
                    self._pos = (msg.lat / 1e7, msg.lon / 1e7, msg.alt / 1000.0, msg.relative_alt / 1000.0)
                    self._pos_t = now
                elif t == 'VFR_HUD':
                    self._hdg, self._hdg_t = float(msg.heading), now
                elif t == 'ATTITUDE':
                    self._att = (math.degrees(msg.roll), math.degrees(msg.pitch),
                                 math.degrees(msg.yaw) % 360)
                    self._att_t = now

    def snapshot(self):
        now = time.time()
        with self._lock:
            return {
                'pos': self._pos, 'pos_age': None if self._pos_t is None else now - self._pos_t,
                'hdg': self._hdg, 'hdg_age': None if self._hdg_t is None else now - self._hdg_t,
                'att': self._att, 'att_age': None if self._att_t is None else now - self._att_t,
            }

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=2.0)


def connect_mavlink(port, baud):
    """Returns a connection, or None (with a warning) if nothing answers."""
    try:
        from pymavlink import mavutil
    except ImportError:
        print("WARNING: pymavlink not installed -- capturing WITHOUT GPS.")
        return None
    print(f"Connecting to {port} @ {baud} baud ...", end=' ', flush=True)
    try:
        conn = mavutil.mavlink_connection(port, baud=baud)
        if not conn.wait_heartbeat(timeout=10):
            print("no heartbeat.")
            print("WARNING: Pixhawk not answering -- capturing WITHOUT GPS (files named 'nogps').")
            return None
    except Exception as e:
        print(f"failed ({e}).")
        print("WARNING: could not open the serial port -- capturing WITHOUT GPS (files named 'nogps').")
        return None
    print(f"OK  sys={conn.target_system} comp={conn.target_component}")
    conn.mav.request_data_stream_send(conn.target_system, conn.target_component,
                                      mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1)
    return conn


def build_pipeline(a):
    exp = f"{int(round(a.exposure_ms[0] * 1e6))} {int(round(a.exposure_ms[1] * 1e6))}"
    props = [
        f"sensor-id={a.sensor_id}",
        f'exposuretimerange="{exp}"',
        f'gainrange="{a.gain[0]} {a.gain[1]}"',
        f'ispdigitalgainrange="{a.isp_gain[0]} {a.isp_gain[1]}"',
        f"tnr-mode={a.tnr_mode}",
    ]
    if a.tnr_strength is not None:
        props.append(f"tnr-strength={a.tnr_strength}")
    if a.ee_mode is not None:
        props.append(f"ee-mode={a.ee_mode}")
    if a.ee_strength is not None:
        props.append(f"ee-strength={a.ee_strength}")
    if a.ev is not None:
        props.append(f"exposurecompensation={a.ev}")
    return (
        f"nvarguscamerasrc {' '.join(props)} ! "
        f"video/x-raw(memory:NVMM), width={a.width}, height={a.height}, format=NV12, framerate=30/1 ! "
        f"nvvidconv flip-method=0 ! video/x-raw, format=BGRx ! videoconvert ! "
        f"video/x-raw, format=BGR ! appsink drop=True max-buffers=1"
    )


def _fmt(v, nd):
    return '' if v is None else f"{v:.{nd}f}"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--interval', type=float, default=0.5, help='seconds between photos (default 0.5)')
    ap.add_argument('--duration-s', type=float, default=0,
                    help='stop after this many seconds (default 0 = run until stopped)')
    ap.add_argument('--port', default='/dev/ttyTHS1')
    ap.add_argument('--baud', type=int, default=57600)
    ap.add_argument('--no-mavlink', action='store_true', help='do not connect to the Pixhawk (no GPS in names)')
    ap.add_argument('--out-dir', default=None,
                    help='base folder (default: photo_obtained/ next to this script); '
                         'each run gets a <timestamp>_capture subfolder')
    ap.add_argument('--format', choices=['png', 'jpg'], default='png',
                    help='png = lossless, ~1.5 MB/photo (default); jpg = ~0.3 MB/photo')
    ap.add_argument('--jpeg-quality', type=int, default=95, help='1-100, only for --format jpg (default 95)')
    ap.add_argument('--min-free-gb', type=float, default=2.0,
                    help='stop cleanly when free disk space drops below this (default 2)')
    cam = ap.add_argument_group('camera')
    cam.add_argument('--sensor-id', '--cam', dest='sensor_id', type=int, choices=[0, 1], default=1,
                     help='which CSI camera port to use: 0 = CAM0, 1 = CAM1 (default 1)')
    cam.add_argument('--width', type=int, default=1280)
    cam.add_argument('--height', type=int, default=720)
    cam.add_argument('--exposure-ms', type=float, nargs=2, metavar=('LO', 'HI'),
                     default=[v / 1e6 for v in config.EXPOSURE_TIME_RANGE_NS],
                     help='shutter (exposure time) range in ms, HW 0.013-683.709; LO == HI fixes it. '
                          'Default: %(default)s')
    cam.add_argument('--gain', type=float, nargs=2, metavar=('LO', 'HI'), default=list(config.GAIN_RANGE),
                     help='sensor analog gain range, HW 1.0-10.625. Default: %(default)s')
    cam.add_argument('--isp-gain', type=float, nargs=2, metavar=('LO', 'HI'),
                     default=list(config.ISP_DIGITAL_GAIN_RANGE),
                     help='ISP digital gain range, 1.0-256.0. Default: %(default)s')
    cam.add_argument('--ev', type=float, default=None, help='exposure compensation -2.0..+2.0 (default: not set)')
    cam.add_argument('--tnr-mode', type=int, choices=[0, 1, 2], default=0,
                     help='temporal noise reduction: 0=OFF (default), 1=fast, 2=high quality')
    cam.add_argument('--tnr-strength', type=float, default=None, help='-1.0..1.0 (default: not set)')
    cam.add_argument('--ee-mode', type=int, choices=[0, 1, 2], default=None,
                     help='edge enhancement: 0=off, 1=fast, 2=high quality (default: not set)')
    cam.add_argument('--ee-strength', type=float, default=None, help='-1.0..1.0 (default: not set)')
    a = ap.parse_args()

    def _rng(name, r, lo, hi):
        if not (lo <= r[0] <= r[1] <= hi):
            ap.error(f"{name} must satisfy {lo} <= LO <= HI <= {hi} (got {r[0]} {r[1]})")
    _rng('--exposure-ms', a.exposure_ms, 0.013, 683.709)
    _rng('--gain', a.gain, 1.0, 10.625)
    _rng('--isp-gain', a.isp_gain, 1.0, 256.0)
    if a.interval <= 0:
        ap.error("--interval must be > 0")
    if a.ev is not None and not -2.0 <= a.ev <= 2.0:
        ap.error("--ev must be between -2.0 and 2.0")
    for n in ('tnr_strength', 'ee_strength'):
        v = getattr(a, n)
        if v is not None and not -1.0 <= v <= 1.0:
            ap.error(f"--{n.replace('_', '-')} must be between -1.0 and 1.0")
    if not 1 <= a.jpeg_quality <= 100:
        ap.error("--jpeg-quality must be 1..100")

    # SIGTERM (e.g. a plain `kill`) -> same clean shutdown path as Ctrl+C.
    def _on_sigterm(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, _on_sigterm)

    base = a.out_dir or os.path.join(os.path.dirname(os.path.abspath(__file__)), 'photo_obtained')
    out_dir = os.path.join(base, datetime.now().strftime('%Y%m%d_%H%M%S') + '_capture')
    os.makedirs(out_dir, exist_ok=True)
    print(f"Photos + logs: {out_dir}")
    pipeline = build_pipeline(a)
    with open(os.path.join(out_dir, 'run_args.txt'), 'w') as f:
        f.write(f"started: {datetime.now().isoformat(timespec='seconds')}\n")
        f.write(f"command: {' '.join(sys.argv)}\n")
        for k, v in sorted(vars(a).items()):
            f.write(f"{k}: {v}\n")
        f.write(f"pipeline: {pipeline}\n")

    telem = None
    if not a.no_mavlink:
        conn = connect_mavlink(a.port, a.baud)
        if conn is not None:
            telem = Telemetry(conn)

    print(f"Camera CAM{a.sensor_id}: {a.width}x{a.height}  shutter {a.exposure_ms[0]:.3f}-{a.exposure_ms[1]:.3f} ms  "
          f"gain {a.gain[0]}-{a.gain[1]}x  isp-gain {a.isp_gain[0]}-{a.isp_gain[1]}x  "
          f"ev={'not set' if a.ev is None else a.ev}")
    print(f"Camera ISP: tnr-mode={a.tnr_mode} ({'OFF' if a.tnr_mode == 0 else 'ON'})  "
          f"tnr-strength={'not set' if a.tnr_strength is None else a.tnr_strength}  "
          f"ee-mode={'not set' if a.ee_mode is None else a.ee_mode}  "
          f"ee-strength={'not set' if a.ee_strength is None else a.ee_strength}")
    print(f"GStreamer pipeline: {pipeline}")

    cap = None
    csv_file = None
    n = n_fail_save = n_late = 0
    t_start = time.time()
    try:
        cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        if not cap.isOpened():
            raise SystemExit("ERROR: could not open the CSI camera -- check the cable / "
                             "'sudo systemctl restart nvargus-daemon'.")
        for _ in range(10):  # let auto-exposure settle
            cap.read()
        print("Camera open.")

        csv_file = open(os.path.join(out_dir, 'capture_log.csv'), 'w', newline='')
        writer = csv.DictWriter(csv_file, fieldnames=CSV_FIELDS)
        writer.writeheader()
        stop_msg = '' if not a.duration_s else f" for {a.duration_s:.0f}s"
        print(f"Capturing every {a.interval}s{stop_msg} as .{a.format} -- "
              f"stop with Ctrl+C or './run_photo_capture.sh stop'.\n")

        grab_fail = 0
        next_t = time.monotonic()
        t_start = time.time()
        while True:
            if a.duration_s and time.time() - t_start >= a.duration_s:
                print(f"\n--duration-s {a.duration_s:.0f} reached, stopping.")
                break

            ok, frame = cap.read()
            now = time.time()
            if not ok or frame is None:
                grab_fail += 1
                print(f"WARNING: frame grab failed ({grab_fail} in a row)")
                if grab_fail >= MAX_CONSECUTIVE_GRAB_FAILURES:
                    print("ERROR: camera stopped delivering frames -- stopping. "
                          "Try 'sudo systemctl restart nvargus-daemon'.")
                    break
                next_t = time.monotonic() + a.interval
                time.sleep(a.interval)
                continue
            grab_fail = 0
            n += 1

            s = telem.snapshot() if telem else {'pos': None, 'pos_age': None, 'hdg': None,
                                                 'hdg_age': None, 'att': None, 'att_age': None}
            pos = s['pos']
            fresh = pos is not None and s['pos_age'] <= TELEMETRY_STALE_S
            dt = datetime.fromtimestamp(now)
            stamp = dt.strftime('%Y%m%d_%H%M%S') + f"_{dt.microsecond // 1000:03d}"
            if pos is None:
                fname = f"CAP_{stamp}_nogps.{a.format}"
            else:
                fname = f"CAP_{stamp}_{pos[0]:.7f}_{pos[1]:.7f}_{pos[2]:.2f}.{a.format}"

            t0 = time.time()
            params = [cv2.IMWRITE_JPEG_QUALITY, a.jpeg_quality] if a.format == 'jpg' else []
            saved = cv2.imwrite(os.path.join(out_dir, fname), frame, params)
            save_ms = (time.time() - t0) * 1000
            if not saved:
                n_fail_save += 1
                print(f"WARNING: failed to save {fname} ({n_fail_save} failure(s) so far)")

            att = s['att']
            writer.writerow({
                'index': n, 'time': dt.isoformat(timespec='milliseconds'), 'unix_time': f"{now:.3f}",
                'filename': fname if saved else f"(NOT SAVED) {fname}", 'gps_fresh': int(fresh),
                'lat': _fmt(pos and pos[0], 7), 'lon': _fmt(pos and pos[1], 7),
                'alt_msl': _fmt(pos and pos[2], 2), 'alt_agl': _fmt(pos and pos[3], 2),
                'gps_age_s': _fmt(s['pos_age'], 2), 'heading_deg': _fmt(s['hdg'], 1),
                'roll_deg': _fmt(att and att[0], 2), 'pitch_deg': _fmt(att and att[1], 2),
                'yaw_deg': _fmt(att and att[2], 2), 'att_age_s': _fmt(s['att_age'], 2),
                'save_ms': f"{save_ms:.0f}",
            })
            csv_file.flush()

            if pos is None:
                gps_txt = 'no GPS'
            else:
                gps_txt = f"{pos[0]:.7f},{pos[1]:.7f} msl={pos[2]:.1f}m agl={pos[3]:.1f}m"
                if not fresh:
                    gps_txt += f" (STALE {s['pos_age']:.1f}s)"
            print(f"[{n}] {fname}  {gps_txt}  save={save_ms:.0f}ms")

            if n % 20 == 0:
                free_gb = shutil.disk_usage(out_dir).free / 1024 ** 3
                if free_gb < a.min_free_gb:
                    print(f"\nFree disk space {free_gb:.1f} GB < --min-free-gb {a.min_free_gb} -- stopping.")
                    break

            next_t += a.interval
            sleep = next_t - time.monotonic()
            if sleep > 0:
                time.sleep(sleep)
            else:
                n_late += 1
                if sleep < -a.interval:
                    print(f"WARNING: running {-sleep:.2f}s behind schedule -- interval too short for "
                          f"this save speed (try --format jpg or a longer --interval)")
                    next_t = time.monotonic()

    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        if cap is not None:
            cap.release()
        if csv_file is not None:
            csv_file.close()
        if telem is not None:
            telem.stop()

    elapsed = time.time() - t_start
    print(f"Done: {n} photo(s) in {elapsed:.0f}s, {n_fail_save} save failure(s), "
          f"{n_late} late frame(s). Folder: {out_dir}")


if __name__ == '__main__':
    main()
