#!/usr/bin/env python3
"""
Replay the DJ_photo/ DJI sequence as a live-paced VO "flight" into a real
Pixhawk: read one new photo every --interval seconds (like a live capture
loop would), run VO against the previous photo, walk the dead-reckoning
PositionTracker forward, and continuously push the result to Pixhawk as
GPS2 via GPS_INPUT — so Mission Planner shows the vehicle moving according
to the VO estimate, not a real GPS.

Requires GPS2_TYPE=14 and GPS_AUTO_SWITCH=4 already set on the Pixhawk
(and an initial GPS2 fix already established, so home/EKF origin exists).

Usage:
    python3 simulate_dji_flight.py \
        --anchor-lat 25.0428908 --anchor-lon 121.4741081 \
        --altitude 60 --interval 2 --port /dev/ttyTHS1
"""
import argparse
import os
import sys
import time

import cv2

import config
import visual_odometry as vo
from dead_reckoning import PositionTracker
from process_dji_sequence import list_dji_sequence, focal_px_from_diagonal_fov, displacement_from_transform_generic

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'gpsless_mapping'))
from gps_sim import send_gps_input, maps_url


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--photo-dir', default='DJ_photo')
    ap.add_argument('--anchor-lat', type=float, required=True)
    ap.add_argument('--anchor-lon', type=float, required=True)
    ap.add_argument('--anchor-alt', type=float, default=0.0, help='anchor altitude, m MSL (matches the fix already sent)')
    ap.add_argument('--altitude', type=float, default=60.0, help='constant AGL altitude for GSD scaling')
    ap.add_argument('--fov', type=float, default=84.0, help='assumed diagonal FOV, deg (see process_dji_sequence.py)')
    ap.add_argument('--interval', type=float, default=2.0, help='seconds between photos (real-time paced)')
    ap.add_argument('--gps-hz', type=float, default=5.0)
    ap.add_argument('--port', default='/dev/ttyTHS1')
    ap.add_argument('--baud', type=int, default=57600)
    ap.add_argument('--heading', type=float, default=None, help='override starting heading; default reads live from Pixhawk')
    ap.add_argument('--distance-scale', type=float, default=1.0,
                     help='multiply each leg\'s calculated straight-line displacement (dx/dy/magnitude) by this '
                          'factor -- a manual correction for the assumed FOV/altitude not being precisely '
                          'calibrated for this camera. Does not affect altitude_change_m (that comes from the '
                          'affine transform\'s own zoom factor, not the translation distance).')
    ap.add_argument('--yaw-scale', type=float, default=1.0,
                     help='multiply each leg\'s calculated yaw_change_deg by this factor -- a separate manual '
                          'correction for turning legs overshooting the real turn angle, independent of the '
                          'distance-scale correction applied to translation.')
    args = ap.parse_args()

    items, skipped = list_dji_sequence(args.photo_dir)
    if skipped:
        print(f"Skipping {len(skipped)} non-matching file(s): {skipped}", flush=True)
    if len(items) < 2:
        raise SystemExit("need at least 2 DJI_*.JPG files")

    from pymavlink import mavutil
    print(f"Connecting to {args.port} @ {args.baud} ...", flush=True)
    conn = mavutil.mavlink_connection(args.port, baud=args.baud)
    if not conn.wait_heartbeat(timeout=10):
        raise SystemExit("no heartbeat")
    print(f"Connected: sys={conn.target_system} comp={conn.target_component}", flush=True)
    conn.mav.request_data_stream_send(conn.target_system, conn.target_component,
                                       mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1)

    heading = args.heading
    if heading is None:
        msg = conn.recv_match(type='VFR_HUD', blocking=True, timeout=5)
        heading = msg.heading if msg else 0.0
        print(f"Live heading from Pixhawk: {heading:.1f} deg", flush=True)

    seq0, ts0, w0, h0, path0 = items[0]
    focal_px = focal_px_from_diagonal_fov(w0, h0, args.fov)
    print(f"{len(items)} frames, seq {items[0][0]:04d}..{items[-1][0]:04d}, "
          f"image {w0}x{h0}, assumed FOV={args.fov}deg -> focal={focal_px:.1f}px, "
          f"altitude={args.altitude}m -> GSD={args.altitude/focal_px:.4f} m/px, "
          f"distance_scale={args.distance_scale}, yaw_scale={args.yaw_scale}", flush=True)

    tracker = PositionTracker(lat=args.anchor_lat, lon=args.anchor_lon,
                               alt_m=args.anchor_alt, heading_deg=heading)
    print(f"Anchor: {args.anchor_lat:.7f},{args.anchor_lon:.7f} alt={args.anchor_alt}m "
          f"heading={heading:.1f}deg  interval={args.interval}s  gps_hz={args.gps_hz}", flush=True)
    print(flush=True)

    prev_gray, prev_path = None, None
    vn, ve = 0.0, 0.0
    n_confident = 0
    t_start = time.time()

    try:
        for i, (seq, ts, w_i, h_i, path) in enumerate(items):
            gray = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
            if gray is None:
                print(f"WARNING: could not read {path}", flush=True)
            elif prev_gray is None:
                print(f"[  0.0s][  0/{len(items)-1}] [ANCHOR] {os.path.basename(path)}  "
                      f"-> {tracker.lat:.7f},{tracker.lon:.7f}  hacc={tracker.hacc_m:.2f}m  "
                      f"{maps_url(tracker.lat, tracker.lon)}", flush=True)
            elif prev_gray is not None:
                pts1, pts2 = vo.detect_and_match(prev_gray, gray)
                M, inliers = vo.estimate_similarity(pts1, pts2)
                n_matches = len(pts1)
                n_inliers = int(inliers.sum()) if inliers is not None else 0

                result = {'ok': False, 'confident': False}
                if M is not None and inliers is not None and n_inliers >= config.MIN_INLIERS:
                    disp = displacement_from_transform_generic(M, args.altitude, w_i, h_i, focal_px)
                    result['ok'] = True
                    result['confident'] = (n_inliers / n_matches) >= config.MIN_INLIER_RATIO
                    result.update(dx_m=disp['dx_m'] * args.distance_scale,
                                  dy_m=disp['dy_m'] * args.distance_scale,
                                  magnitude_m=disp['magnitude_m'] * args.distance_scale,
                                  yaw_change_deg=disp['yaw_change_deg'] * args.yaw_scale,
                                  altitude_change_m=disp['altitude_change_m'])

                state = tracker.apply_leg(result, dt_s=args.interval)
                vn, ve = state['vn'], state['ve']
                if result['confident']:
                    n_confident += 1

                tag = 'OK  ' if result['confident'] else ('FAIL' if not result['ok'] else 'LOW-CONF')
                elapsed = time.time() - t_start
                print(f"[{elapsed:6.1f}s][{i:3d}/{len(items)-1}] [{tag}] "
                      f"{os.path.basename(prev_path)} -> {os.path.basename(path)}  "
                      f"matches={n_matches:4d} inliers={n_inliers:4d}  "
                      f"disp={result.get('magnitude_m', 0):5.2f}m  "
                      f"-> {state['lat']:.7f},{state['lon']:.7f}  hacc={state['hacc_m']:.2f}m  "
                      f"{maps_url(state['lat'], state['lon'])}", flush=True)

            prev_gray, prev_path = gray, path

            # keep sending GPS_INPUT at gps_hz for the rest of this interval
            next_t = time.time() + args.interval
            while True:
                send_gps_input(conn, tracker.lat, tracker.lon, tracker.alt_m, vn, ve, hacc=tracker.hacc_m)
                remaining = next_t - time.time()
                if remaining <= 0:
                    break
                time.sleep(min(1.0 / args.gps_hz, remaining))

    except KeyboardInterrupt:
        print("\nStopped by user.", flush=True)

    print(flush=True)
    print(f"Done: {tracker.legs_since_anchor} legs, {n_confident} confident. "
          f"Final: {tracker.lat:.7f},{tracker.lon:.7f} hacc={tracker.hacc_m:.2f}m "
          f"{maps_url(tracker.lat, tracker.lon)}", flush=True)


if __name__ == '__main__':
    main()
