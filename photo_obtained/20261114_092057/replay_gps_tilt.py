"""Replay 20261114_092057 through vo_flight_capture_fix1's ground-projection VO,
with roll/pitch ESTIMATED FROM REAL GPS (no attitude was logged that flight).

Tilt model (multirotor, thrust along body -Z):
    body-down axis  ∝  g_vec - a_eff,   a_eff = a_gps + K_DRAG * v_gps
  a_gps: GPS position 2nd difference, v_gps: central difference.
  K_DRAG: linear drag/tilt coefficient (assumed, not fitted): 0.375 1/s
          -> ~15 deg tilt at 7 m/s steady cruise. Set 0 for accel-only.
Wind-induced steady tilt and fast attitude wobble are NOT observable from GPS.
"""
import math, sys, csv, os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import cv2

proj = r'c:\Claude-Code\Jetson_drone\gpsless_VO_run1\gpsless_VO_image'
sys.path.insert(0, proj)
import vo_flight_capture_fix1 as m

G = 9.81
run = os.path.join(proj, 'photo_obtained', '20261114_092057')
out_png = os.path.join(run, 'flight_20261114_analysis_gps_tilt.png')
rows = list(csv.DictReader(open(os.path.join(run, 'vo_log.csv'), encoding='utf-8')))
N = len(rows)
lat0, lon0 = float(rows[0]['real_lat']), float(rows[0]['real_lon'])
kN = 111111.0; kE = 111111.0 * math.cos(math.radians(lat0))
t = np.array([float(r['timestamp']) for r in rows])
P = np.array([[(float(r['real_lat']) - lat0) * kN, (float(r['real_lon']) - lon0) * kE, -float(r['real_alt_agl'])]
              for r in rows])  # NED
yaw = np.radians([float(r['real_heading_deg']) for r in rows])
agl = np.array([float(r['real_alt_agl']) for r in rows])


def tilt_from_gps(k_drag):
    v = np.zeros_like(P); a = np.zeros_like(P)
    for i in range(N):
        i0, i1 = max(0, i - 1), min(N - 1, i + 1)
        v[i] = (P[i1] - P[i0]) / (t[i1] - t[i0])
        if 0 < i < N - 1:
            dt1, dt2 = t[i] - t[i - 1], t[i + 1] - t[i]
            a[i] = 2 * ((P[i + 1] - P[i]) / dt2 - (P[i] - P[i - 1]) / dt1) / (dt1 + dt2)
    a_eff = a + k_drag * v * np.array([1, 1, 0])
    atts = []
    for i in range(N):
        zb = np.array([-a_eff[i, 0], -a_eff[i, 1], G - a_eff[i, 2]])
        zb /= np.linalg.norm(zb)
        c, s = math.cos(yaw[i]), math.sin(yaw[i])
        fwd = c * zb[0] + s * zb[1]
        right = -s * zb[0] + c * zb[1]
        roll = math.asin(max(-1, min(1, -right)))
        pitch = math.atan2(fwd, zb[2])
        atts.append((roll, pitch, yaw[i]))
    return atts


def replay(atts):
    grays = [cv2.imread(os.path.join(run, r['filename']), cv2.IMREAD_GRAYSCALE) for r in rows]
    pos = np.zeros(2); track = [pos.copy()]; status = ['FIRST']; inl = [0]; err = [0.0]; agg = 0.0
    for i in range(1, N):
        r = m.process_pair_ground(grays[i - 1], grays[i], atts[i - 1], atts[i], agl[i - 1], agl[i])
        if r['ok']:
            pos = pos + [r['north_m'], r['east_m']]; agg += r['magnitude_m']
        track.append(pos.copy())
        status.append('OK' if r.get('confident') else ('FAIL' if not r['ok'] else 'LOW-CONF'))
        inl.append(r['n_inliers'])
        err.append(float(np.linalg.norm(pos - P[i, :2])))
    return np.array(track), status, inl, err, agg


results = {}
for name, k in [('zero tilt', None), ('GPS accel only', 0.0), ('GPS accel + drag', 0.375)]:
    atts = [(0.0, 0.0, y) for y in yaw] if k is None else tilt_from_gps(k)
    tr, st, inl, err, agg = replay(atts)
    results[name] = (tr, st, inl, err, agg, atts)
    nok = sum(s != 'FAIL' for s in st[1:])
    hover = [i for i in range(1, N) if np.linalg.norm(P[i, :2] - P[i - 1, :2]) < 0.5 and st[i] != 'FAIL']
    hover_fake = np.mean([np.linalg.norm(tr[i] - tr[i - 1]) for i in hover]) if hover else float('nan')
    tilt_max = max(math.degrees(math.hypot(a[0], a[1])) for a in atts)
    print(f"{name:18s} legs ok={nok}/{N-1}  final err={err[-1]:5.1f}m  max err={max(err):5.1f}m  "
          f"agg={agg:5.1f}m  mean fake move on near-static legs={hover_fake:.2f}m  max tilt={tilt_max:.1f}deg")

# old run (as flown)
old = np.array([[(float(r['vo_lat']) - lat0) * kN, (float(r['vo_lon']) - lon0) * kE] for r in rows])
old_err = [float(r['real_vo_diff_m']) for r in rows]
print(f"{'as flown (fix)':18s} final err={old_err[-1]:5.1f}m  max err={max(old_err):5.1f}m")

# ---- plot (same layout as flight_20261114_analysis.png) ----
name = 'GPS accel + drag'
tr, st, inl, err, agg, atts = results[name]
fig, ax = plt.subplots(1, 2, figsize=(14, 6))
ax[0].plot(P[:, 1], P[:, 0], 'b.-', label='real GPS')
ax[0].plot(old[:, 1], old[:, 0], '-', color='0.6', lw=1, label='VO as flown (fix)')
ax[0].plot(tr[:, 1], tr[:, 0], 'r.-', label='VO fix1 + GPS-derived roll/pitch')
ax[0].plot(0, 0, 'k*', ms=14, label='anchor (takeoff)')
for i in range(0, N, 5):
    ax[0].annotate(str(i + 1), (P[i, 1], P[i, 0]), fontsize=7, color='b')
ax[0].set_xlabel('East (m)'); ax[0].set_ylabel('North (m)'); ax[0].axis('equal')
ax[0].legend(fontsize=8); ax[0].grid(alpha=.3)
ax[0].set_title('20261114_092057  real vs VO (fix1, roll/pitch from GPS)')
fr = np.arange(1, N + 1)
ax2 = ax[1]
ax2.bar(fr, inl, color=['tab:green' if s in ('OK', 'FIRST') else ('tab:orange' if s == 'LOW-CONF' else 'tab:red')
                        for s in st], label='ground-fit inliers (green OK / orange LOW / red FAIL)')
ax2.axhline(12, color='k', ls='--', lw=1, label='MIN_INLIERS=12')
ax2.set_yscale('symlog'); ax2.set_xlabel('frame'); ax2.set_ylabel('inliers')
ax3 = ax2.twinx()
ax3.plot(fr, agl, 'b-', label='AGL (m)')
ax3.plot(fr, err, 'm-', label='VO fix1 error (m)')
ax3.plot(fr, old_err, '-', color='0.6', lw=1, label='VO as flown error (m)')
ax3.plot(fr, [math.degrees(math.hypot(a[0], a[1])) for a in atts], 'c--', lw=1, label='GPS-derived tilt (deg)')
ax3.set_ylabel('AGL m / error m / tilt deg')
ax2.legend(loc='upper left', fontsize=7); ax3.legend(loc='upper right', fontsize=7)
ax2.set_title('inliers / altitude / error / tilt')
plt.tight_layout(); plt.savefig(out_png, dpi=110)
print('saved', out_png)
