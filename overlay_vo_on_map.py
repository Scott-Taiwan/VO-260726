#!/usr/bin/env python3
"""
Overlay a VO trajectory (in red) onto a satellite screenshot that already
has the real GPS flight path drawn on it (e.g. exported from Google Maps),
for visual comparison.

Geo-registration is automatic, not manual: extracts the drawn path's pixels
by color threshold, then fits a similarity transform (scale + rotation +
translation, ICP-style against the CSV's own gps_latitude/gps_longitude
column) mapping real-world meters to image pixels. The same transform is
then applied to the CSV's VO-estimated latitude/longitude column to draw
the comparison trace. A converged rotation near 0 deg confirms the source
image is north-up (true for a default, unrotated Google Maps screenshot).

Usage:
    python3 overlay_vo_on_map.py real_action.png CSV/vo_trajectory_640X480_diff_New1.csv \
        --out combined_VO_New1.png
"""
import argparse
import csv
import math

import cv2
import numpy as np
from scipy.spatial import cKDTree


def to_enu(lat, lon, lat0, lon0):
    north = (lat - lat0) * 111111.0
    east = (lon - lon0) * (111111.0 * math.cos(math.radians(lat0)))
    return east, north


def umeyama(P, Q):
    """Fit s,R,t minimizing sum||s*R@P_i + t - Q_i||^2. No reflection."""
    mu_p, mu_q = P.mean(axis=0), Q.mean(axis=0)
    P0, Q0 = P - mu_p, Q - mu_q
    cov = (P0.T @ Q0) / len(P)
    U, D, Vt = np.linalg.svd(cov)
    S = np.eye(2)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[-1, -1] = -1
    R = U @ S @ Vt
    var_p = (P0 ** 2).sum(axis=1).mean()
    s = np.trace(np.diag(D) @ S) / var_p
    t = mu_q - s * R @ mu_p
    return s, R, t


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('image', help='satellite screenshot with the real GPS path already drawn on it')
    ap.add_argument('csv_path', help='trajectory CSV with gps_latitude/gps_longitude and latitude/longitude columns')
    ap.add_argument('--out', required=True)
    ap.add_argument('--path-color-bgr', default='0,0,255',
                     help='BGR of the already-drawn real-GPS path to detect (default matches bright yellow)')
    ap.add_argument('--iters', type=int, default=30)
    ap.add_argument('--init-scale', type=float, default=4.0, help='initial guess, pixels per meter')
    args = ap.parse_args()

    img = cv2.imread(args.image)
    b, g, r = cv2.split(img)
    # bright yellow: high R, high G, low B (Google Maps' default path color)
    yellow_mask = (r > 180) & (g > 180) & (b < 120)
    ys, xs = np.where(yellow_mask)
    yellow_pts = np.stack([xs, ys], axis=1).astype(np.float64)
    print("path pixels detected:", len(yellow_pts))

    rows = list(csv.DictReader(open(args.csv_path)))
    lat0, lon0 = float(rows[0]['gps_latitude']), float(rows[0]['gps_longitude'])
    gps_enu = np.array([to_enu(float(r['gps_latitude']), float(r['gps_longitude']), lat0, lon0) for r in rows])
    vo_enu = np.array([to_enu(float(r['latitude']), float(r['longitude']), lat0, lon0) for r in rows])

    # source points for registration: (east, south) so a pure rotation+scale+translation
    # (no reflection) maps correctly onto pixel space (pixel-x = east-like, pixel-y = south-like)
    src = np.stack([gps_enu[:, 0], -gps_enu[:, 1]], axis=1)

    s, R, t = args.init_scale, np.eye(2), np.array([1285.0, 340.0]) - args.init_scale * src[0]
    tree = cKDTree(yellow_pts)
    for it in range(args.iters):
        transformed = (s * (src @ R.T)) + t
        dist, idx = tree.query(transformed)
        targets = yellow_pts[idx]
        s, R, t = umeyama(src, targets)

    rot_deg = math.degrees(math.atan2(R[1, 0], R[0, 0]))
    print(f"Fitted transform: scale={s:.4f} px/m  rotation={rot_deg:.3f}deg  translation={t}")

    def project(east, north):
        p = np.array([east, -north])
        return s * (R @ p) + t

    gps_proj = np.array([project(e, n) for e, n in gps_enu])
    res = np.linalg.norm(gps_proj - yellow_pts[tree.query(gps_proj)[1]], axis=1)
    print(f"Registration check (real GPS re-projected vs drawn path): mean={res.mean():.2f}px max={res.max():.2f}px")

    vo_proj = np.array([project(e, n) for e, n in vo_enu])

    out = img.copy()
    for i in range(1, len(vo_proj)):
        p1 = tuple(np.round(vo_proj[i - 1]).astype(int))
        p2 = tuple(np.round(vo_proj[i]).astype(int))
        cv2.line(out, p1, p2, (0, 0, 255), 3, cv2.LINE_AA)  # BGR red

    cv2.imwrite(args.out, out)
    print(f"Saved {args.out}")


if __name__ == '__main__':
    main()
