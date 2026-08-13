#!/usr/bin/env python3
"""
Build a motion-replay video from a process_dji_sequence.py CSV: each frame
shows the actual photo side-by-side with the trajectory computed so far.

Usage:
    python3 make_motion_movie.py dji_vo_results.csv DJ_photo/ --out dji_motion.mp4
"""
import argparse
import csv
import os

import cv2
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_agg import FigureCanvasAgg

BG = '#12171a'
PANEL = '#1a2124'
GRID = '#2e3a3d'
ACCENT = '#5fd0a8'
WARN = '#e8a23d'
TEXT = '#e7ecea'
TEXT_DIM = '#8fa39c'


def load_rows(csv_path):
    with open(csv_path) as f:
        return list(csv.DictReader(f))


def render_traj_panel(xs, ys, confidents, i, xlim, ylim, size_px):
    fig = plt.Figure(figsize=(size_px / 100, size_px / 100), dpi=100, facecolor=BG)
    canvas = FigureCanvasAgg(fig)
    ax = fig.add_subplot(111)
    ax.set_facecolor(PANEL)

    # faint full path for context
    ax.plot(xs, ys, color=GRID, linewidth=1.2, zorder=1)
    # revealed path so far
    ok_mask = confidents[:i + 1]
    ax.plot(xs[:i + 1], ys[:i + 1], color=ACCENT, linewidth=2.2, zorder=2)
    marker_color = ACCENT if confidents[i] else WARN
    ax.plot(xs[i], ys[i], 'o', color=marker_color, markersize=9, zorder=3,
            markeredgecolor=BG, markeredgewidth=1.5)

    ax.set_xlim(*xlim)
    ax.set_ylim(*ylim)
    ax.invert_yaxis()  # image-down axis convention -> "forward" reads as up on screen
    ax.set_aspect('equal')
    ax.tick_params(colors=TEXT_DIM, labelsize=7)
    for spine in ax.spines.values():
        spine.set_color(GRID)
    ax.grid(True, color=GRID, linewidth=0.5, alpha=0.6)
    ax.set_xlabel('meters (east-ish)', color=TEXT_DIM, fontsize=7)
    ax.set_ylabel('meters (forward-ish)', color=TEXT_DIM, fontsize=7)
    fig.tight_layout(pad=0.6)

    canvas.draw()
    buf = np.asarray(canvas.buffer_rgba())
    rgb = cv2.cvtColor(buf, cv2.COLOR_RGBA2BGR)
    plt.close(fig)
    return cv2.resize(rgb, (size_px, size_px))


def hex_to_bgr(h):
    h = h.lstrip('#')
    r, g, b = int(h[0:2], 16), int(h[2:4], 16), int(h[4:6], 16)
    return (b, g, r)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('csv_path')
    ap.add_argument('photo_dir')
    ap.add_argument('--out', default='motion_movie.mp4')
    ap.add_argument('--fps', type=float, default=10.0)
    args = ap.parse_args()

    rows = load_rows(args.csv_path)
    photos = [rows[0]['from']] + [r['to'] for r in rows]
    xs = [0.0] + [float(r['traj_x_m']) for r in rows]
    ys = [0.0] + [float(r['traj_y_m']) for r in rows]
    confidents = [True] + [r['confident'] == 'True' for r in rows]
    mags = [0.0] + [float(r['magnitude_m']) for r in rows]

    xpad = max(max(xs) - min(xs), 1.0) * 0.12
    ypad = max(max(ys) - min(ys), 1.0) * 0.12
    xlim = (min(xs) - xpad, max(xs) + xpad)
    ylim = (min(ys) - ypad, max(ys) + ypad)

    sample = cv2.imread(os.path.join(args.photo_dir, photos[0]))
    ph, pw = sample.shape[:2]
    panel_size = ph  # square trajectory panel, same height as the photo
    out_w, out_h = pw + panel_size, ph

    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    writer = cv2.VideoWriter(args.out, fourcc, args.fps, (out_w, out_h))
    if not writer.isOpened():
        raise SystemExit("could not open VideoWriter")

    accent_bgr = hex_to_bgr(ACCENT)
    warn_bgr = hex_to_bgr(WARN)
    text_bgr = hex_to_bgr(TEXT)

    n = len(photos)
    for i, name in enumerate(photos):
        photo = cv2.imread(os.path.join(args.photo_dir, name))
        if photo is None:
            photo = np.zeros((ph, pw, 3), dtype='uint8')
        panel = render_traj_panel(xs, ys, confidents, i, xlim, ylim, panel_size)

        frame = np.zeros((out_h, out_w, 3), dtype='uint8')
        frame[:, :pw] = photo
        frame[:, pw:] = panel

        tag_color = accent_bgr if confidents[i] else warn_bgr
        tag = 'OK' if confidents[i] else 'LOW-CONF'
        cv2.rectangle(frame, (0, 0), (pw, 42), (20, 20, 20), -1)
        cv2.putText(frame, f"[{i+1}/{n}] {name}", (8, 17), cv2.FONT_HERSHEY_SIMPLEX, 0.45, text_bgr, 1, cv2.LINE_AA)
        cv2.putText(frame, f"leg {mags[i]:.2f}m  [{tag}]", (8, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.45, tag_color, 1, cv2.LINE_AA)

        writer.write(frame)
        if (i + 1) % 50 == 0 or i == n - 1:
            print(f"  {i+1}/{n} frames rendered")

    writer.release()
    print(f"Wrote {args.out} ({n} frames @ {args.fps}fps, {n/args.fps:.1f}s, {out_w}x{out_h})")


if __name__ == '__main__':
    main()
