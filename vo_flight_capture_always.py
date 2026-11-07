#!/usr/bin/env python3
"""
vo_flight_capture_always.py — same as vo_flight_capture.py, EXCEPT it does
NOT wait for any takeoff altitude threshold: capture starts immediately
once the startup checks pass -- Pixhawk connected, GPS position confirmed,
camera self-check passed -- including while still on the ground.

Everything else is identical to vo_flight_capture.py and reused directly
from that module rather than duplicated here: the VO math (feature
matching, RANSAC, dead-reckoning), the camera pipeline, the filename
format, the photo_obtained/<run_timestamp>/ folder structure, the
stuck-camera detection, and the save-failure warnings.

Startup sequence, printed to the terminal as it happens:
  1. Connect to Pixhawk.
  2. Confirm a GPS position (GLOBAL_POSITION_INT) and heading (VFR_HUD) can
     actually be read -- this also becomes the one-time dead-reckoning
     anchor, exactly like vo_flight_capture.py.
  3. Open the camera ONCE (never closed/reopened for the rest of the run)
     and run the camera self-check (rotate/move it when prompted) -- the
     PASS/FAIL result is printed clearly either way.
  4. Start capturing immediately -- no altitude wait.

No automatic landing-stop: unlike vo_flight_capture.py, this variant has no
altitude-based auto-stop at all -- it keeps capturing indefinitely,
including through ground-level/handheld testing where altitude may
casually cross back and forth near any fixed threshold. Ctrl+C is the only
way to stop it. (An earlier version tried to auto-stop on landing using
the same AGL-dip logic as vo_flight_capture.py; that falsely triggered
during a handheld test where altitude hovered right around the 2m
threshold -- "capturing regardless of altitude" is this script's entire
point, so that logic was removed rather than tuned.)

Usage:
    python3 vo_flight_capture_always.py --port /dev/ttyTHS1
    python3 vo_flight_capture_always.py --interval 1
"""
import argparse
import os
import time
from datetime import datetime

# vo_flight_capture must be imported BEFORE cv2/numpy here -- it does its
# own sys.path.insert(0, '/usr/lib/python3/dist-packages') before its
# `import cv2`, to dodge a pip-installed opencv-python (no GStreamer
# support) that shadows the system one. Importing cv2 first here would
# cache the wrong module and silently break camera access -- this exact
# bug bit preflight_camera_check.py earlier.
import vo_flight_capture as vfc
import cv2  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--interval', type=float, default=2.0, help='seconds between captures (default 2)')
    ap.add_argument('--port', default='/dev/ttyTHS1')
    ap.add_argument('--baud', type=int, default=57600)
    ap.add_argument('--sensor-id', type=int, default=0)
    ap.add_argument('--out-dir', default=None,
                     help='base folder for photos (default: photo_obtained/ next to this script) -- '
                          'each run gets its own timestamped subfolder underneath this')
    ap.add_argument('--wave-seconds', type=int, default=3,
                     help='seconds to rotate/move the camera for during the startup self-check (default 3)')
    args = ap.parse_args()

    base_out_dir = args.out_dir or os.path.join(os.path.dirname(os.path.abspath(__file__)), 'photo_obtained')
    run_id = datetime.now().strftime('%Y%m%d_%H%M%S')
    out_dir = os.path.join(base_out_dir, run_id)
    os.makedirs(out_dir, exist_ok=True)
    print(f"This run's photos will be saved to: {out_dir}\n")

    conn = vfc.connect_pixhawk(args.port, args.baud)

    print("Confirming GPS position is available...")
    pos = vfc.read_global_position(conn, timeout=10)
    if pos is None:
        raise SystemExit(
            "Connected to Pixhawk (heartbeat OK), but no GLOBAL_POSITION_INT arrived "
            "within 10s. ArduPilot only publishes this once it has a valid position "
            "estimate, so this almost always means: no GPS fix yet (no sky view, or "
            "just powered on and still acquiring satellites), or EKF origin not set. "
            "Check GPS status in Mission Planner/QGroundControl, then retry."
        )
    heading = vfc.read_heading(conn, timeout=10)
    if heading is None:
        raise SystemExit(
            "Connected to Pixhawk and got a GPS position, but no VFR_HUD heading "
            "arrived within 10s. GPS fix is fine -- this points at the telemetry "
            "data stream rate/config instead, not GPS. Check the MAVLink link/radio "
            "and telemetry stream rate (request_data_stream), then retry."
        )
    anchor_lat, anchor_lon, anchor_alt_msl, anchor_alt_agl = pos
    print(f"GPS confirmed: {anchor_lat:.7f}, {anchor_lon:.7f}  "
          f"alt_msl={anchor_alt_msl:.1f}m  agl={anchor_alt_agl:.1f}m  heading={heading:.0f}deg\n")
    tracker = vfc.PositionTracker(lat=anchor_lat, lon=anchor_lon, alt_m=anchor_alt_msl, heading_deg=heading)

    print(f"Opening camera ({vfc.IMAGE_WIDTH}x{vfc.IMAGE_HEIGHT}, {vfc.CAMERA_HFOV_DEG:.0f}deg HFOV, "
          f"FX_PIXELS=FY_PIXELS={vfc.FX_PIXELS:.1f})...")
    cap = vfc.open_camera(args.sensor_id)
    print("Camera open.\n")

    # EVERYTHING from here on is inside one try/finally so cap.release() is
    # guaranteed no matter what happens -- including a self-check FAILURE
    # (self_check_camera raises SystemExit) or a Ctrl+C during the self-check
    # itself. An earlier version only wrapped the self-check's own SystemExit
    # case (and not even that for the main loop), so some failure paths
    # leaked the camera session without releasing it -- a real contributor
    # to the repeated CMA memory-exhaustion crashes seen today, not just an
    # unrelated OS limitation.
    try:
        try:
            vfc.self_check_camera(cap, args.wave_seconds)
        except SystemExit as e:
            print(f"\n=== PREFLIGHT CHECK: FAIL ===\n{e}\n")
            raise
        print("=== PREFLIGHT CHECK: PASS ===\n")

        print(f"Capturing every {args.interval}s, starting now (no altitude wait) -- Ctrl+C to stop.\n")

        prev_gray = prev_time = prev_alt_agl = None
        n_frame = 0
        n_save_failures = 0
        agg_distance_m = 0.0

        stuck_check_frame = 0
        stuck_check_real_latlon = None
        stuck_check_agg_distance_m = 0.0

        while True:
            frame, gray = vfc.capture_gray(cap)
            now = time.time()
            if frame is None:
                print("WARNING: frame grab failed, skipping")
                time.sleep(args.interval)
                continue
            n_frame += 1

            real_pos = vfc.read_global_position(conn, timeout=0.5)
            if real_pos is not None:
                real_lat, real_lon, real_alt_msl, cur_alt_agl = real_pos
            else:
                real_lat, real_lon, real_alt_msl = anchor_lat, anchor_lon, anchor_alt_msl
                cur_alt_agl = prev_alt_agl if prev_alt_agl is not None else vfc.FALLBACK_ALTITUDE_M

            if prev_gray is None:
                # First captured frame: no prior frame to match against, so
                # the VO estimate is still exactly the anchor.
                r = {'n_matches': 0, 'n_inliers': 0, 'ok': False}
                move_x = move_y = move_z = 0.0
            else:
                dt_s = now - prev_time
                r = vfc.process_pair(prev_gray, gray, prev_alt_agl, cur_alt_agl)
                state = tracker.apply_leg(r, dt_s=dt_s)
                move_x = state['vn'] * dt_s  # north, meters, this leg
                move_y = state['ve'] * dt_s  # east, meters, this leg
                move_z = r.get('altitude_change_m', 0.0) if r['ok'] else 0.0
                if r['ok']:
                    agg_distance_m += r['magnitude_m']

                tag = "OK  " if r.get('confident') else ("FAIL" if not r['ok'] else "LOW-CONF")
                print(f"[frame {n_frame}] [{tag}] dt={dt_s:.2f}s "
                      f"matches={r['n_matches']} inliers={r['n_inliers']}", end='  ')
                if r['ok']:
                    print(f"disp={r['magnitude_m']:.2f}m yaw={r['yaw_change_deg']:+.2f}deg", end='  ')
                else:
                    print(f"({r.get('reason', 'no match')})", end='  ')
                print(f"-> vo={tracker.lat:.7f},{tracker.lon:.7f} agg_dist={agg_distance_m:.1f}m "
                      f"agl={cur_alt_agl:.1f}m")

            diff_m = vfc.vo.haversine_m(real_lat, real_lon, tracker.lat, tracker.lon)

            ts = datetime.now().strftime('%Y%m%d%H%M%S')
            fname = (
                f"VO_{ts}_{real_lat:.7f}_{real_lon:.7f}_{real_alt_msl:.2f}_"
                f"{tracker.lat:.7f}_{tracker.lon:.7f}_{tracker.alt_m:.2f}_{diff_m:.2f}_"
                f"{move_x:.2f}_{move_y:.2f}_{move_z:.2f}_"
                f"{r['n_matches']}_{r['n_inliers']}_{agg_distance_m:.2f}.png"
            )
            save_path = os.path.join(out_dir, fname)
            if not cv2.imwrite(save_path, frame):
                n_save_failures += 1
                free_gb = vfc.shutil.disk_usage(out_dir).free / (1024 ** 3)
                print(f"WARNING: failed to save photo (disk full / write error?) -- "
                      f"{save_path}  [{free_gb:.1f}GB free on that filesystem, "
                      f"{n_save_failures} failure(s) so far this run]")

            prev_gray, prev_time, prev_alt_agl = gray, now, cur_alt_agl

            # Stuck-camera check: same logic as vo_flight_capture.py -- every
            # STUCK_CHECK_INTERVAL_FRAMES, compare real GPS movement against
            # VO's own odometer over the same window.
            if stuck_check_real_latlon is None:
                stuck_check_real_latlon = (real_lat, real_lon)
                stuck_check_agg_distance_m = agg_distance_m
                stuck_check_frame = n_frame
            elif n_frame - stuck_check_frame >= vfc.STUCK_CHECK_INTERVAL_FRAMES:
                real_moved_m = vfc.vo.haversine_m(
                    stuck_check_real_latlon[0], stuck_check_real_latlon[1], real_lat, real_lon)
                vo_moved_m = agg_distance_m - stuck_check_agg_distance_m
                if real_moved_m >= vfc.STUCK_REAL_MOVE_THRESHOLD_M and vo_moved_m <= vfc.STUCK_VO_MOVE_THRESHOLD_M:
                    print(f"WARNING: camera may be STUCK -- real GPS moved {real_moved_m:.1f}m over the "
                          f"last {n_frame - stuck_check_frame} frames, but VO's odometer only advanced "
                          f"{vo_moved_m:.2f}m. Likely a frozen/stale capture pipeline (e.g. wedged Argus "
                          f"session), not a real hover. Check the live camera feed if possible.")
                stuck_check_real_latlon = (real_lat, real_lon)
                stuck_check_agg_distance_m = agg_distance_m
                stuck_check_frame = n_frame

            elapsed = time.time() - now
            time.sleep(max(0.0, args.interval - elapsed))

    except KeyboardInterrupt:
        print(f"\nStopped early by user after {n_frame} frame(s).")
    finally:
        cap.release()

    print(f"Final: {n_frame} frame(s) captured, {tracker.legs_since_anchor} leg(s) applied, "
          f"aggregated flight distance {agg_distance_m:.1f}m, "
          f"final VO position {tracker.lat:.7f},{tracker.lon:.7f} hacc={tracker.hacc_m:.2f}m")
    if n_save_failures:
        print(f"WARNING: {n_save_failures}/{n_frame} photo(s) FAILED TO SAVE this run "
              f"(see WARNING lines above) -- check disk space before the next flight.")


if __name__ == '__main__':
    main()
