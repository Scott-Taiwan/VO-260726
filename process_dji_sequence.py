#!/usr/bin/env python3
"""
Batch VO over a DJI photo sequence — DJI_yyyymmddhhmmss_XXXX_D_WxH.JPG.

Reuses visual_odometry.py's feature-matching core (detect_and_match,
estimate_similarity), but does NOT reuse config.py's camera constants
(FX_PIXELS/FY_PIXELS/IMAGE_WIDTH/IMAGE_HEIGHT) — those were derived for the
Jetson's IMX219 camera at 1280x720. This DJI sequence has no EXIF, no GPS,
no altitude, and an unknown camera model, so the pixel->metric conversion
here uses its own explicit, clearly-flagged assumptions instead:

  - ALTITUDE_M: constant altitude for the whole sequence, given by the user
    (60 m) — DJI photos carry no altitude metadata to read this from.
  - ASSUMED_DIAGONAL_FOV_DEG: the user didn't know this camera's exact
    model/FOV, so this uses a representative value for DJI consumer camera
    drones (~84 deg diagonal, common across the Mavic/Air/Mini lines) rather
    than a calibrated figure. The metric (meters) output is only as good as
    this guess — treat it as an estimate, not a calibrated measurement.

Files are ordered by the 4-digit sequence number embedded in the filename
(not mtime) — it's explicit and copy-safe, unlike mtime.

Usage:
    python3 process_dji_sequence.py DJ_photo/ --altitude 60 --out dji_vo_results.csv
"""
import argparse
import csv
import math
import os
import re

import cv2

import visual_odometry as vo
import config

DJI_RE = re.compile(r'^DJI_(\d{14})_(\d{4})_D_(\d+)x(\d+)\.JPG$', re.IGNORECASE)


def list_dji_sequence(directory):
    """Sorted by the embedded sequence number. Non-matching files (e.g. stray
    unrelated files in the folder) are skipped, not silently mis-ordered."""
    items = []
    skipped = []
    for name in os.listdir(directory):
        m = DJI_RE.match(name)
        if m:
            ts, seq, w, h = m.groups()
            items.append((int(seq), ts, int(w), int(h), os.path.join(directory, name)))
        else:
            skipped.append(name)
    items.sort(key=lambda x: x[0])
    return items, skipped


def focal_px_from_diagonal_fov(width, height, diagonal_fov_deg):
    """Derive an equivalent focal length in pixels from an assumed diagonal
    FOV, for a sensor with square pixels (fx == fy in pixel units)."""
    diag_px = math.hypot(width, height)
    return (diag_px / 2.0) / math.tan(math.radians(diagonal_fov_deg) / 2.0)


def displacement_from_transform_generic(M, altitude_m, width, height, focal_px):
    """Same derivation as visual_odometry.displacement_from_transform(), but
    parameterized by this sequence's own image size and focal length instead
    of the Jetson-specific config.py constants."""
    a, b, tx = M[0]
    c, d, ty = M[1]
    scale = math.sqrt(a * a + c * c)
    theta_rad = math.atan2(c, a)

    cx, cy = width / 2.0, height / 2.0
    center2 = (a * cx + b * cy + tx, c * cx + d * cy + ty)
    apparent_flow = (center2[0] - cx, center2[1] - cy)
    cam_shift_px = (-apparent_flow[0], -apparent_flow[1])

    gsd = altitude_m / focal_px  # same GSD for x and y: square-pixel assumption
    dx_m = cam_shift_px[0] * gsd
    dy_m = cam_shift_px[1] * gsd

    return {
        'dx_px': cam_shift_px[0], 'dy_px': cam_shift_px[1],
        'dx_m': dx_m, 'dy_m': dy_m,
        'magnitude_m': math.hypot(dx_m, dy_m),
        'yaw_change_deg': -math.degrees(theta_rad),
        'scale': scale,
        'altitude_change_m': altitude_m * (1.0 / scale - 1.0) if scale > 0 else float('nan'),
    }


def process_sequence(directory, altitude_m, diagonal_fov_deg):
    items, skipped = list_dji_sequence(directory)
    if skipped:
        print(f"Skipping {len(skipped)} non-matching file(s): {skipped}")
    if len(items) < 2:
        raise SystemExit("need at least 2 DJI_*.JPG files")

    seq0, ts0, w, h, path0 = items[0]
    focal_px = focal_px_from_diagonal_fov(w, h, diagonal_fov_deg)
    print(f"{len(items)} frames, seq {items[0][0]:04d}..{items[-1][0]:04d}, "
          f"image {w}x{h}, assumed diagonal FOV={diagonal_fov_deg}deg -> focal={focal_px:.1f}px, "
          f"altitude={altitude_m}m -> GSD={altitude_m / focal_px:.4f} m/px")
    print()

    rows = []
    traj = [(0.0, 0.0)]
    heading_deg = 0.0
    prev_gray, prev_path = None, path0

    for i, (seq, ts, w_i, h_i, path) in enumerate(items):
        gray = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
        if gray is None:
            print(f"WARNING: could not read {path}, skipping")
            continue
        if prev_gray is None:
            prev_gray, prev_path, prev_seq = gray, path, seq
            continue

        pts1, pts2 = vo.detect_and_match(prev_gray, gray)
        M, inliers = vo.estimate_similarity(pts1, pts2)
        n_matches = len(pts1)
        n_inliers = int(inliers.sum()) if inliers is not None else 0

        row = {
            'from': os.path.basename(prev_path), 'to': os.path.basename(path),
            'n_matches': n_matches, 'n_inliers': n_inliers,
            'ok': False, 'confident': False, 'reason': '',
            'dx_m': 0.0, 'dy_m': 0.0, 'magnitude_m': 0.0,
            'yaw_change_deg': 0.0, 'altitude_change_m': 0.0,
            'traj_x_m': traj[-1][0], 'traj_y_m': traj[-1][1],
        }

        if M is None or inliers is None or n_inliers < config.MIN_INLIERS:
            row['reason'] = 'too few inliers'
        else:
            disp = displacement_from_transform_generic(M, altitude_m, w_i, h_i, focal_px)
            row['ok'] = True
            row['confident'] = (n_inliers / n_matches) >= config.MIN_INLIER_RATIO
            row['dx_m'], row['dy_m'] = disp['dx_m'], disp['dy_m']
            row['magnitude_m'] = disp['magnitude_m']
            row['yaw_change_deg'] = disp['yaw_change_deg']
            row['altitude_change_m'] = disp['altitude_change_m']
            if not row['confident']:
                row['reason'] = 'low inlier ratio'

            if row['confident']:
                heading_deg += disp['yaw_change_deg']
                th = math.radians(heading_deg)
                wx = disp['dx_m'] * math.cos(th) - disp['dy_m'] * math.sin(th)
                wy = disp['dx_m'] * math.sin(th) + disp['dy_m'] * math.cos(th)
                px, py = traj[-1]
                traj.append((px + wx, py + wy))
            else:
                traj.append(traj[-1])
            row['traj_x_m'], row['traj_y_m'] = traj[-1]

        rows.append(row)
        tag = 'OK  ' if row['confident'] else ('FAIL' if not row['ok'] else 'LOW-CONF')
        print(f"[{i:3d}] [{tag}] {row['from']} -> {row['to']}  "
              f"matches={n_matches:4d} inliers={n_inliers:4d}  "
              f"disp={row['magnitude_m']:6.2f}m  yaw={row['yaw_change_deg']:+6.2f}deg  "
              f"traj=({row['traj_x_m']:+7.2f},{row['traj_y_m']:+7.2f})")

        prev_gray, prev_path = gray, path

    return rows


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('directory')
    ap.add_argument('--altitude', type=float, required=True, help='constant altitude (m) for the whole sequence')
    ap.add_argument('--fov', type=float, default=84.0, help='assumed diagonal FOV in degrees (default 84, typical DJI consumer drone camera)')
    ap.add_argument('--out', default='dji_vo_results.csv')
    args = ap.parse_args()

    rows = process_sequence(args.directory, args.altitude, args.fov)

    with open(args.out, 'w', newline='') as f:
        fieldnames = ['from', 'to', 'n_matches', 'n_inliers', 'ok', 'confident', 'reason',
                      'dx_m', 'dy_m', 'magnitude_m', 'yaw_change_deg', 'altitude_change_m',
                      'traj_x_m', 'traj_y_m']
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    n_ok = sum(1 for r in rows if r['confident'])
    print()
    print(f"Done: {len(rows)} pairs, {n_ok} confident ({n_ok/len(rows)*100:.0f}%). Wrote {args.out}")
    print(f"Final cumulative position (frame-0 reference): "
          f"({rows[-1]['traj_x_m']:+.2f}, {rows[-1]['traj_y_m']:+.2f}) m")


if __name__ == '__main__':
    main()
