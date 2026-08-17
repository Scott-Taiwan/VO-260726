#!/usr/bin/env python3
"""
Fit the best distance_scale/yaw_scale pair against real GPS ground truth,
via least-squares -- not manual trial-and-error against Mission Planner.

Requires a trajectory CSV (as produced by recompute_vo_trajectory.py) with
UNCORRECTED (distance_scale=1.0, yaw_scale=1.0) x/z/rotation_deg columns,
plus the real gps_latitude/gps_longitude columns as ground truth. Recovers
each leg's raw local-frame displacement by inverting the heading rotation
baked into the stored x/z trajectory, then replays the trajectory under
arbitrary (k_d, k_y) and minimizes total squared position error against
real GPS (converted to local ENU meters) using Nelder-Mead. Verified to
converge to the same optimum from multiple starting points.

Usage:
    python3 fit_correction_scales.py CSV/vo_trajectory_640X480_diff_New.csv
"""
import argparse
import csv
import math

import numpy as np
from scipy.optimize import minimize

ANCHOR_HEADING_DEG = 270.0


def to_enu(lat, lon, lat0, lon0):
    north = (lat - lat0) * 111111.0
    east = (lon - lon0) * (111111.0 * math.cos(math.radians(lat0)))
    return east, north


def rotate(dx, dy, heading_deg):
    th = math.radians(heading_deg)
    north = -dy * math.cos(th) - dx * math.sin(th)
    east = -dy * math.sin(th) + dx * math.cos(th)
    return north, east


def inverse_rotate(north, east, heading_deg):
    th = math.radians(heading_deg)
    dx = -math.sin(th) * north + math.cos(th) * east
    dy = -math.cos(th) * north - math.sin(th) * east
    return dx, dy


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('csv_path', help='UNCORRECTED trajectory CSV (distance_scale=1.0, yaw_scale=1.0)')
    ap.add_argument('--anchor-heading', type=float, default=ANCHOR_HEADING_DEG)
    args = ap.parse_args()

    rows = list(csv.DictReader(open(args.csv_path)))
    lat0, lon0 = float(rows[0]['gps_latitude']), float(rows[0]['gps_longitude'])
    gps_enu = np.array([to_enu(float(r['gps_latitude']), float(r['gps_longitude']), lat0, lon0) for r in rows])

    x = np.array([float(r['x']) for r in rows])
    z = np.array([float(r['z']) for r in rows])
    rot = np.array([float(r['rotation_deg']) for r in rows])

    # reconstruct each leg's raw local-frame (image-axis) displacement by undoing
    # the heading rotation that was baked into the stored x/z trajectory
    heading = args.anchor_heading
    local_dx = np.zeros(len(rows))
    local_dy = np.zeros(len(rows))
    for i in range(1, len(rows)):
        heading += rot[i]
        dx, dy = inverse_rotate(z[i] - z[i - 1], x[i] - x[i - 1], heading)
        local_dx[i], local_dy[i] = dx, dy

    def replay(k_d, k_y):
        heading = args.anchor_heading
        xs, zs = [0.0], [0.0]
        xc, zc = 0.0, 0.0
        for i in range(1, len(rows)):
            heading += k_y * rot[i]
            north, east = rotate(k_d * local_dx[i], k_d * local_dy[i], heading)
            xc += east
            zc += north
            xs.append(xc)
            zs.append(zc)
        return np.array(xs), np.array(zs)

    xs_check, zs_check = replay(1.0, 1.0)
    assert np.abs(xs_check - x).max() < 1e-9 and np.abs(zs_check - z).max() < 1e-9, \
        "round-trip mismatch -- is this really an uncorrected (scale=1.0) CSV?"

    def cost(params):
        k_d, k_y = params
        xs, zs = replay(k_d, k_y)
        return ((xs - gps_enu[:, 0]) ** 2 + (zs - gps_enu[:, 1]) ** 2).sum()

    def rms_and_final(k_d, k_y):
        xs, zs = replay(k_d, k_y)
        err = np.sqrt((xs - gps_enu[:, 0]) ** 2 + (zs - gps_enu[:, 1]) ** 2)
        return err.mean(), err[-1]

    baseline_rms, baseline_final = rms_and_final(1.0, 1.0)
    print(f"baseline (k_d=1.0, k_y=1.0): mean_err={baseline_rms:.2f}m  final_err={baseline_final:.2f}m")

    starts = [[0.856, 0.97], [1.0, 1.0], [0.7, 1.05]]
    results = [minimize(cost, x0=x0, method='Nelder-Mead',
                         options={'xatol': 1e-6, 'fatol': 1e-6, 'maxiter': 5000}) for x0 in starts]
    for x0, res in zip(starts, results):
        print(f"  from x0={x0}: k_d={res.x[0]:.4f} k_y={res.x[1]:.4f}")

    k_d_opt, k_y_opt = results[0].x
    opt_rms, opt_final = rms_and_final(k_d_opt, k_y_opt)
    print(f"\noptimum: k_d={k_d_opt:.4f}  k_y={k_y_opt:.4f}")
    print(f"  mean_err={opt_rms:.2f}m  final_err={opt_final:.2f}m  "
          f"(baseline was {baseline_rms:.2f}m / {baseline_final:.2f}m)")


if __name__ == '__main__':
    main()
