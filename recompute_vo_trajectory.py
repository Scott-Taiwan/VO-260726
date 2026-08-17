#!/usr/bin/env python3
"""
Rebuild vo_trajectory_640X480_diff_New.csv from scratch, using DJ_photo/
directly. Everything from `x` through `gps_diff_distance_m` is recomputed;
`gps_latitude`/`gps_longitude` are read from the existing CSV and preserved
untouched as ground truth (this program never invents or adjusts them).

No distance/yaw correction factor is applied here (unlike
simulate_dji_flight.py's --distance-scale/--yaw-scale) -- the whole point
of this run is to see the RAW, uncorrected VO output against real GPS, so
any systematic bias shows up honestly instead of being papered over.

Column semantics (reverse-engineered from the existing file, matched
exactly so this is a drop-in replacement):
  x, z    : cumulative local-frame position, meters, frame-0 anchored
            (x = lateral/right, z = forward -- image "up" = forward,
            heading = 270 deg / due West, per the real_action.png
            measurement earlier in this project). y is always 0 (altitude
            not modeled here, matching the original file).
  latitude/longitude : VO-estimated absolute position (anchored at
            gps_latitude[0]/gps_longitude[0], dead-reckoned forward)
  raw_step_distance_m / step_distance_m : identical here (no smoothing
            applied) -- per-leg displacement magnitude
  gps_diff_distance_m : haversine distance between (latitude,longitude)
            and (gps_latitude,gps_longitude) at that frame -- i.e. the
            VO's absolute position error against real GPS at that instant

Usage:
    python3 recompute_vo_trajectory.py \
        CSV/vo_trajectory_640X480_diff_New.csv DJ_photo/ \
        --out CSV/vo_trajectory_640X480_diff_New.csv
"""
import argparse
import csv
import math
import os

import cv2

import config
import visual_odometry as vo
from process_dji_sequence import focal_px_from_diagonal_fov, displacement_from_transform_generic
from visual_odometry import haversine_m

ANCHOR_HEADING_DEG = 270.0  # due West, measured from real_action.png


def load_reference(csv_path):
    with open(csv_path) as f:
        rows = list(csv.DictReader(f))
    return [(r['filename'], float(r['gps_latitude']), float(r['gps_longitude'])) for r in rows]


def rotate(dx, dy, heading_deg):
    """Same convention as dead_reckoning.PositionTracker: image 'up' (-dy)
    is forward at heading 0, image right (dx) is right of nose."""
    th = math.radians(heading_deg)
    north = -dy * math.cos(th) - dx * math.sin(th)
    east = -dy * math.sin(th) + dx * math.cos(th)
    return north, east


def offset_latlon(lat, lon, north_m, east_m):
    dlat = north_m / 111111.0
    dlon = east_m / (111111.0 * math.cos(math.radians(lat)))
    return lat + dlat, lon + dlon


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('reference_csv', help='existing CSV to read filenames + gps_latitude/gps_longitude from')
    ap.add_argument('photo_dir')
    ap.add_argument('--altitude', type=float, default=60.0)
    ap.add_argument('--fov', type=float, default=84.0)
    ap.add_argument('--distance-scale', type=float, default=1.0,
                     help='multiply each leg\'s raw displacement magnitude by this factor')
    ap.add_argument('--yaw-scale', type=float, default=1.0,
                     help='multiply each leg\'s raw yaw_change_deg by this factor')
    ap.add_argument('--out', required=True)
    args = ap.parse_args()

    ref = load_reference(args.reference_csv)
    print(f"{len(ref)} reference rows loaded from {args.reference_csv}", flush=True)

    sample = cv2.imread(os.path.join(args.photo_dir, ref[0][0]))
    h0, w0 = sample.shape[:2]
    focal_px = focal_px_from_diagonal_fov(w0, h0, args.fov)
    print(f"image {w0}x{h0}, FOV={args.fov}deg -> focal={focal_px:.1f}px, altitude={args.altitude}m, "
          f"heading={ANCHOR_HEADING_DEG}deg, distance_scale={args.distance_scale}, yaw_scale={args.yaw_scale}",
          flush=True)

    anchor_lat, anchor_lon = ref[0][1], ref[0][2]
    x_cum, y_cum, z_cum = 0.0, 0.0, 0.0
    lat, lon = anchor_lat, anchor_lon
    heading = ANCHOR_HEADING_DEG
    total_dist = 0.0
    gps_total_dist = 0.0
    prev_gray = None

    out_rows = []
    for i, (filename, gps_lat, gps_lon) in enumerate(ref):
        path = os.path.join(args.photo_dir, filename)
        gray = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
        if gray is None:
            raise SystemExit(f"could not read {path}")

        if i == 0:
            row = dict(frame=i, time_s=0.0, filename=filename, x=0.0, y=0.0, z=0.0,
                       latitude=lat, longitude=lon, matches=0, inliers=0, status='起點',
                       inlier_ratio=0.0, rotation_deg=0.0, raw_step_distance_m=0.0,
                       step_distance_m=0.0, total_distance_m=0.0, speed_m_s=0.0, speed_km_h=0.0,
                       gps_latitude=gps_lat, gps_longitude=gps_lon,
                       gps_step_distance_m=0.0, gps_total_distance_m=0.0,
                       gps_speed_m_s=0.0, gps_speed_km_h=0.0, gps_diff_distance_m=0.0)
            out_rows.append(row)
            prev_gray = gray
            prev_gps_lat, prev_gps_lon = gps_lat, gps_lon
            continue

        dt = 2.0  # real DJI capture cadence, confirmed constant from filename timestamps

        pts1, pts2 = vo.detect_and_match(prev_gray, gray)
        M, inliers = vo.estimate_similarity(pts1, pts2)
        n_matches = len(pts1)
        n_inliers = int(inliers.sum()) if inliers is not None else 0
        inlier_ratio = (n_inliers / n_matches) if n_matches else 0.0

        if M is None or inliers is None or n_inliers < config.MIN_INLIERS:
            status = '失敗'
            dx_m = dy_m = step_dist = yaw_change = 0.0
        else:
            disp = displacement_from_transform_generic(M, args.altitude, w0, h0, focal_px)
            dx_m = disp['dx_m'] * args.distance_scale
            dy_m = disp['dy_m'] * args.distance_scale
            step_dist = disp['magnitude_m'] * args.distance_scale
            yaw_change = disp['yaw_change_deg'] * args.yaw_scale
            status = '成功' if inlier_ratio >= config.MIN_INLIER_RATIO else '低信心'

        heading += yaw_change
        north_m, east_m = rotate(dx_m, dy_m, heading)
        x_cum += east_m
        z_cum += north_m
        lat, lon = offset_latlon(lat, lon, north_m, east_m)

        total_dist += step_dist
        speed_m_s = step_dist / dt
        speed_km_h = speed_m_s * 3.6

        gps_step_dist = haversine_m(prev_gps_lat, prev_gps_lon, gps_lat, gps_lon)
        gps_total_dist += gps_step_dist
        gps_speed_m_s = gps_step_dist / dt
        gps_speed_km_h = gps_speed_m_s * 3.6

        diff_dist = haversine_m(lat, lon, gps_lat, gps_lon)

        row = dict(frame=i, time_s=i * dt, filename=filename, x=x_cum, y=y_cum, z=z_cum,
                   latitude=lat, longitude=lon, matches=n_matches, inliers=n_inliers, status=status,
                   inlier_ratio=inlier_ratio, rotation_deg=yaw_change,
                   raw_step_distance_m=step_dist, step_distance_m=step_dist,
                   total_distance_m=total_dist, speed_m_s=speed_m_s, speed_km_h=speed_km_h,
                   gps_latitude=gps_lat, gps_longitude=gps_lon,
                   gps_step_distance_m=gps_step_dist, gps_total_distance_m=gps_total_dist,
                   gps_speed_m_s=gps_speed_m_s, gps_speed_km_h=gps_speed_km_h,
                   gps_diff_distance_m=diff_dist)
        out_rows.append(row)

        print(f"[{i:3d}/{len(ref)-1}] [{status}] matches={n_matches:4d} inliers={n_inliers:4d} "
              f"step={step_dist:5.2f}m gps_step={gps_step_dist:5.2f}m diff={diff_dist:6.2f}m", flush=True)

        prev_gray = gray
        prev_gps_lat, prev_gps_lon = gps_lat, gps_lon

    fieldnames = ['frame', 'time_s', 'filename', 'x', 'y', 'z', 'latitude', 'longitude',
                  'matches', 'inliers', 'status', 'inlier_ratio', 'rotation_deg',
                  'raw_step_distance_m', 'step_distance_m', 'total_distance_m',
                  'speed_m_s', 'speed_km_h', 'gps_latitude', 'gps_longitude',
                  'gps_step_distance_m', 'gps_total_distance_m', 'gps_speed_m_s',
                  'gps_speed_km_h', 'gps_diff_distance_m']
    with open(args.out, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(out_rows)

    print(flush=True)
    print(f"Wrote {args.out}", flush=True)
    print(f"VO total_distance_m: {total_dist:.2f}   GPS gps_total_distance_m: {gps_total_dist:.2f}   "
          f"ratio VO/GPS: {total_dist/gps_total_dist:.4f}", flush=True)
    print(f"Final position error (gps_diff_distance_m): {diff_dist:.2f} m", flush=True)


if __name__ == '__main__':
    main()
