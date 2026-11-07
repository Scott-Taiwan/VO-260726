#!/usr/bin/env python3
"""
parse_flight_results.py — turn a photo_obtained/ folder of vo_flight_capture.py
output into a labeled CSV and a drift/trajectory plot, so you don't have to
count underscores in filenames by eye after a flight.

Each filename encodes 14 fields (see vo_flight_capture.py's module docstring
for the exact semantics):
    VO_<ts>_<real_lat>_<real_lon>_<real_alt>_<vo_lat>_<vo_lon>_<vo_alt>_
       <diff_m>_<move_x>_<move_y>_<move_z>_<matches>_<inliers>_<agg_dist>.png

Usage:
    python3 parse_flight_results.py photo_obtained/
    python3 parse_flight_results.py photo_obtained/ --out-csv flight.csv --out-plot flight.png
"""
import argparse
import csv
import math
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

FIELDS = [
    'filename', 'timestamp',
    'real_lat', 'real_lon', 'real_alt_m',
    'vo_lat', 'vo_lon', 'vo_alt_m',
    'diff_m',
    'move_north_m', 'move_east_m', 'move_alt_m',
    'matches', 'inliers', 'agg_distance_m',
]


def parse_filename(fname):
    """Returns a dict of the 14 fields, or None if fname doesn't match the
    expected VO_... naming convention."""
    stem = fname[:-4] if fname.lower().endswith('.png') else fname
    parts = stem.split('_')
    if len(parts) != 15 or parts[0] != 'VO':
        return None
    try:
        return {
            'filename': fname,
            'timestamp': parts[1],
            'real_lat': float(parts[2]), 'real_lon': float(parts[3]), 'real_alt_m': float(parts[4]),
            'vo_lat': float(parts[5]), 'vo_lon': float(parts[6]), 'vo_alt_m': float(parts[7]),
            'diff_m': float(parts[8]),
            'move_north_m': float(parts[9]), 'move_east_m': float(parts[10]), 'move_alt_m': float(parts[11]),
            'matches': int(parts[12]), 'inliers': int(parts[13]), 'agg_distance_m': float(parts[14]),
        }
    except ValueError:
        return None


def load_rows(photo_dir):
    rows = []
    skipped = []
    for fname in sorted(os.listdir(photo_dir)):
        if not fname.lower().endswith('.png'):
            continue
        row = parse_filename(fname)
        if row is None:
            skipped.append(fname)
        else:
            rows.append(row)
    rows.sort(key=lambda r: r['timestamp'])
    return rows, skipped


def write_csv(rows, out_csv):
    with open(out_csv, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=FIELDS)
        writer.writeheader()
        writer.writerows(rows)


def local_enu(lat0, lon0, lat, lon):
    """Flat-earth offset in meters from (lat0, lon0) -- fine at this scale,
    same approximation dead_reckoning.py itself uses."""
    north_m = (lat - lat0) * 111111.0
    east_m = (lon - lon0) * 111111.0 * math.cos(math.radians(lat0))
    return north_m, east_m


def make_plot(rows, out_plot):
    n = list(range(1, len(rows) + 1))
    diffs = [r['diff_m'] for r in rows]

    lat0, lon0 = rows[0]['real_lat'], rows[0]['real_lon']
    real_xy = [local_enu(lat0, lon0, r['real_lat'], r['real_lon']) for r in rows]
    vo_xy = [local_enu(lat0, lon0, r['vo_lat'], r['vo_lon']) for r in rows]

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))

    ax1.plot(n, diffs, marker='o', markersize=3, color='#d9534f')
    ax1.set_xlabel('photo number')
    ax1.set_ylabel('real vs VO drift (m)')
    ax1.set_title('Drift over the flight')
    ax1.grid(True, alpha=0.3)

    real_e = [xy[1] for xy in real_xy]
    real_n = [xy[0] for xy in real_xy]
    vo_e = [xy[1] for xy in vo_xy]
    vo_n = [xy[0] for xy in vo_xy]
    ax2.plot(real_e, real_n, marker='o', markersize=3, label='real GPS', color='#5cb85c')
    ax2.plot(vo_e, vo_n, marker='o', markersize=3, label='VO estimate', color='#d9534f')
    ax2.set_xlabel('east (m)')
    ax2.set_ylabel('north (m)')
    ax2.set_title('Trajectory: real GPS vs VO')
    ax2.axis('equal')
    ax2.legend()
    ax2.grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig(out_plot, dpi=150)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('photo_dir', help='folder of vo_flight_capture.py output (e.g. photo_obtained/)')
    ap.add_argument('--out-csv', default='flight_results.csv')
    ap.add_argument('--out-plot', default='flight_results.png')
    args = ap.parse_args()

    rows, skipped = load_rows(args.photo_dir)
    if not rows:
        raise SystemExit(f"No VO_*.png files found in {args.photo_dir}")

    write_csv(rows, args.out_csv)
    make_plot(rows, args.out_plot)

    print(f"Parsed {len(rows)} photo(s) from {args.photo_dir}")
    if skipped:
        print(f"Skipped {len(skipped)} file(s) that didn't match the naming convention: {skipped[:5]}"
              f"{' ...' if len(skipped) > 5 else ''}")
    print(f"CSV:  {args.out_csv}")
    print(f"Plot: {args.out_plot}")

    diffs = [r['diff_m'] for r in rows]
    print(f"\nDrift: first={diffs[0]:.2f}m  last={diffs[-1]:.2f}m  max={max(diffs):.2f}m")
    print(f"Final aggregated flight distance: {rows[-1]['agg_distance_m']:.1f}m")
    n_fail = sum(1 for r in rows if r['matches'] == 0 and r['inliers'] == 0)
    print(f"Frames with no usable match (matches=inliers=0): {n_fail}/{len(rows)}")


if __name__ == '__main__':
    main()
